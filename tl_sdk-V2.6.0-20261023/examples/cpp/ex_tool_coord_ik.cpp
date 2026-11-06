/**
 * ex_tool_coord_ik.cpp
 * @brief 末端系逆解示例：接收末端坐标系（相对当前末端）下的目标位姿，
 *        变换到指定坐标系后逆解为关节坐标，并用 MoveJ 下发运动到该关节角。
 *        逆解路径可选（通过命令行 -c 指定）:
 *        - base（默认）: 基坐标(1) → 关节坐标(0) 逆解
 *        - user: 用户坐标(3) → 关节坐标(0) 逆解，使用控制器当前激活的用户坐标系
 *        演示流程: 选择 2 号工具手 → 获取当前关节角 → 给定末端系目标位姿
 *                 → 调用封装接口 solve_endframe_ik（内部: 末端系位姿复合当前位姿
 *                   → 按逆解路径取值: base 直接 1→0 / user 本地变换到用户坐标后 3→0
 *                   （SDK 的 3→0 与 1→0 逆解结果可能不同，USER 模式保留
 *                   控制器原生的用户坐标求解路径））
 *                   → 正解回验 → MoveJ 下发
 * @attention 运行前请确保:
 *            1. 机械臂供电正常、网络通信正常（默认 IP 192.168.1.13）
 *            2. 已标定 2 号工具手（TCP）并已保存到控制器。标定方式二选一:
 *               - 推荐: 在示教器上手动标定，操作更快、可实时可视化确认标定结果
 *               - 代码标定: 工具手用 2 点/20 点法（tool_hand_2_or_20_point_calibrate +
 *                 tool_hand_2_or_20_point_calibrate_caculate）或 7 点法（tool_hand_7_point_calibrate +
 *                 tool_hand_7_point_calibrate_caculate）
 *               本示例只使用控制器中已保存的标定结果，不执行标定流程
 *            3. 使用 -c user 时需已标定对应的用户坐标（calculate_user_coordinate
 *               三点 OXY 标定或 set_user_coordinate_data 直接写入），并确认
 *               控制器当前激活的用户坐标系编号符合预期（示教器或 set_user_coord_number 设定）
 * @warning 本示例会真实驱动机械臂运动到逆解点，运行前请确认机械臂周围无人员/障碍物
 * @note 末端坐标系说明: 末端坐标系固连于当前工具（TCP），其 X/Y/Z 轴方向随工具姿态变化。
 *       末端坐标系下的位姿表示"相对当前末端"的偏移与旋转:
 *       - 位置增量沿末端当前轴方向投影（如沿末端 Z 轴 +195mm，即沿 TCP 正前方 195mm）
 *       - 姿态增量绕末端当前轴旋转（右乘当前姿态旋转矩阵）
 *       相机识别 + 手眼变换（T_target2tool = T_cam2gripper @ T_target2cam，T_cam2gripper
 *       为手眼标定结果）得到的目标位姿即为此形式，由调用方在视觉程序中完成，
 *       标定流程参考 examples/py/ex_handeye_calibration.py。本示例不做手眼变换。
 *       USER 模式目标位姿的本地变换（get_user_coord_para 读取定义 + invert_pose 求逆）
 *       仅用于将末端系目标表达为用户坐标输入；3→0 逆解由控制器按当前激活的
 *       用户坐标系执行，本示例不调用 set_user_coord_number，不修改控制器状态。
 * @note 运行步骤
 *       编译: cd examples/cpp && cmake --preset linux-native && cmake --build --preset linux-native
 *       运行: ./build/linux-native/ex_tool_coord_ik [-c base|user] [X Y Z Rx Ry Rz]
 *             （可选 6 个参数为末端坐标系下的目标位姿，位移单位 mm、姿态单位 rad；
 *               缺省时以沿末端 Z 轴正方向偏移 100mm 作为默认演示点）
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "tl_api.h"


#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif

// 弧度常量（避免依赖 POSIX 扩展宏 M_PI，保证 MSVC 可编译）
constexpr double kPi = 3.14159265358979323846;

// ============================================================
// 工具函数
// ============================================================

/// @brief 打印接口调用结果
void print_result(const std::string& msg, int ret)
{
    std::cout << "  [结果] " << msg << " -> " << ret << " (0=成功)" << std::endl;
}

/// @brief 打印分隔标题
void print_separator(const std::string& title)
{
    std::cout << "\n========== " << title << " ==========" << std::endl;
}

/// @brief 打印 double 向量
void print_vector(const std::string& name, const std::vector<double>& v)
{
    std::cout << "  " << name << " [";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i] << (i + 1 < v.size() ? ", " : "");
    }
    std::cout << "]" << std::endl;
}

// ============================================================
// 位姿变换: out = lhs @ rhs（SE(3) 齐次变换左乘）
// ============================================================

/**
 * @brief 位姿变换: 将右乘位姿（相对变换）应用到左乘位姿上，out = lhs @ rhs
 * @param lhs_pose 左乘位姿（齐次变换 T_lhs），至少 6 元素 [X, Y, Z, Rx, Ry, Rz]
 *                 （位移 mm、姿态 rad）
 * @param rhs_pose 右乘位姿（相对变换 T_rhs），至少 6 元素 [dX, dY, dZ, dRx, dRy, dRz]
 *                 （位移 mm、姿态 rad）
 * @param out_pose 输出: 变换后的位姿（与 lhs_pose 同一参考坐标系表达），7 元素
 * @return true 计算成功；false 参数不足
 *
 * @note 数学: T_target = T_lhs @ T_rhs
 *       位置: p_target = p_lhs + R_lhs @ p_rhs   （目标位置 = 左乘位姿位置 + 沿其轴投影）
 *       姿态: R_target = R_lhs @ R_rhs            （右乘 = 相对旋转）
 * @note 应用场景举例: 传入当前位姿（左乘）与末端系目标位姿（相对变换，右乘），
 *       得到目标位姿在左乘位姿参考坐标系下的表达（endframe_to_base /
 *       endframe_to_user 即基于此）
 * @note 姿态使用 XYZ 欧拉角（R = Rx·Ry·Rz），与 SDK 及 ex_handeye_calibration.py 约定一致
 */
bool transform_pose(
    const std::vector<double>& lhs_pose,
    const std::vector<double>& rhs_pose,
    std::vector<double>& out_pose)
{
    if (lhs_pose.size() < 6 || rhs_pose.size() < 6) {
        return false;
    }

    // ---- R_lhs 列向量（左乘位姿旋转矩阵各列，其轴方向）----
    double crx = cos(lhs_pose[3]), srx = sin(lhs_pose[3]);
    double cry = cos(lhs_pose[4]), sry = sin(lhs_pose[4]);
    double crz = cos(lhs_pose[5]), srz = sin(lhs_pose[5]);
    double c0[3] = {cry * crz, crx * srz + srx * sry * crz, srx * srz - crx * sry * crz};
    double c1[3] = {-cry * srz, crx * crz - srx * sry * srz, srx * crz + crx * sry * srz};
    double c2[3] = {sry, -srx * cry, crx * cry};

    // ---- R_rhs 列向量（右乘相对旋转，XYZ 欧拉角）----
    double rrx = cos(rhs_pose[3]), rsx = sin(rhs_pose[3]);
    double rry = cos(rhs_pose[4]), rsy = sin(rhs_pose[4]);
    double rrz = cos(rhs_pose[5]), rsz = sin(rhs_pose[5]);
    double r0[3] = {rry * rrz, rrx * rsz + rsx * rsy * rrz, rsx * rsz - rrx * rsy * rrz};
    double r1[3] = {-rry * rsz, rrx * rrz - rsx * rsy * rsz, rsx * rrz + rrx * rsy * rsz};
    double r2[3] = {rsy, -rsx * rry, rrx * rry};

    // ---- 位置: p_target = p_lhs + R_lhs @ p_rhs ----
    out_pose.assign(7, 0.0);
    out_pose[0] = lhs_pose[0] + c0[0] * rhs_pose[0] + c1[0] * rhs_pose[1] + c2[0] * rhs_pose[2];
    out_pose[1] = lhs_pose[1] + c0[1] * rhs_pose[0] + c1[1] * rhs_pose[1] + c2[1] * rhs_pose[2];
    out_pose[2] = lhs_pose[2] + c0[2] * rhs_pose[0] + c1[2] * rhs_pose[1] + c2[2] * rhs_pose[2];

    // ---- 姿态: R_target = R_lhs @ R_rhs（新列 = R_lhs @ R_rhs 第 j 列）----
    double n0[3], n1[3], n2[3];
    for (int i = 0; i < 3; ++i) {
        n0[i] = c0[i] * r0[0] + c1[i] * r0[1] + c2[i] * r0[2];
        n1[i] = c0[i] * r1[0] + c1[i] * r1[1] + c2[i] * r1[2];
        n2[i] = c0[i] * r2[0] + c1[i] * r2[1] + c2[i] * r2[2];
    }

    // ---- 从 R_target 提取 XYZ 欧拉角 ----
    double ry = n2[0];
    if (ry > 1.0) ry = 1.0;          // 防 asin 域外（数值误差）
    if (ry < -1.0) ry = -1.0;

    if (std::fabs(ry) >= 1.0 - 1e-12) {
        // Gimbal Lock: |Ry| = 90°，Rx 与 Rz 退化到同一旋转轴，固定 rz=0。
        // 提取 Rx = ±atan2(R[1][0], R[1][1])（R[1][0] = n0[1]，R[1][1] = n1[1]）：
        // Ry=+90° 时 Rx 取正（对应 Rx+Rz 相位），Ry=-90° 时需取负（对应 Rz-Rx），
        // 否则重建矩阵与原始矩阵不等（数值验证误差 2 rad）。与 ex_handeye_calibration.py
        // matrix_to_rpy 约定一致（该脚本同样存在 -90° 分支缺陷，本示例已修正）。
        out_pose[3] = (ry > 0) ? atan2(n0[1], n1[1]) : -atan2(n0[1], n1[1]);  // Rx
        out_pose[4] = (ry > 0) ? kPi / 2.0 : -kPi / 2.0;         // Ry = ±90°
        out_pose[5] = 0.0;                                        // Rz = 0
    } else {
        out_pose[3] = atan2(-n2[1], n2[2]);    // Rx
        out_pose[4] = asin(ry);                 // Ry
        out_pose[5] = atan2(-n1[0], n0[0]);     // Rz
    }
    return true;
}

// ============================================================
// 位姿求逆: out = inv(pose)（SE(3) 齐次变换求逆）
// ============================================================

/**
 * @brief 位姿求逆: 计算 SE(3) 齐次变换的逆变换，out = inv(pose)
 * @param pose  输入位姿 [X, Y, Z, Rx, Ry, Rz]（位移 mm、姿态 rad，XYZ 欧拉角）
 * @param out   输出: 逆位姿，7 元素；失败时 out 不变
 * @return true 计算成功；false 参数不足
 *
 * @note 数学: inv(T) = [R^T, -R^T @ t; 0, 1]
 *       旋转: R_inv = R^T（欧拉角反向提取）
 *       位置: t_inv = -R^T @ t
 * @note 应用场景举例: 用户坐标系定义 T_user2base 由 get_user_coord_para 读出，
 *       求逆得 T_base2user 后可用于本地坐标系变换（免改控制器当前用户坐标）
 * @note 姿态使用 XYZ 欧拉角（R = Rx·Ry·Rz），与 SDK 及 ex_handeye_calibration.py 约定一致
 */
bool invert_pose(
    const std::vector<double>& pose,
    std::vector<double>& out)
{
    if (pose.size() < 6) {
        return false;
    }

    // ---- R 列向量（XYZ 欧拉角）----
    double crx = cos(pose[3]), srx = sin(pose[3]);
    double cry = cos(pose[4]), sry = sin(pose[4]);
    double crz = cos(pose[5]), srz = sin(pose[5]);
    double c0[3] = {cry * crz, crx * srz + srx * sry * crz, srx * srz - crx * sry * crz};
    double c1[3] = {-cry * srz, crx * crz - srx * sry * srz, srx * crz + crx * sry * srz};
    double c2[3] = {sry, -srx * cry, crx * cry};

    // ---- R^T 的行 = R 的列；R^T 的列向量 = R 的行向量 ----
    // R^T 列 j = R 行 j = {c0[j], c1[j], c2[j]}
    double rt0[3] = {c0[0], c1[0], c2[0]};   // R 行 0
    double rt1[3] = {c0[1], c1[1], c2[1]};   // R 行 1
    double rt2[3] = {c0[2], c1[2], c2[2]};   // R 行 2

    // ---- 位置: t_inv = -R^T @ t（R^T 行 i = R 列 i = ci）----
    out.assign(7, 0.0);
    out[0] = -(c0[0] * pose[0] + c0[1] * pose[1] + c0[2] * pose[2]);
    out[1] = -(c1[0] * pose[0] + c1[1] * pose[1] + c1[2] * pose[2]);
    out[2] = -(c2[0] * pose[0] + c2[1] * pose[1] + c2[2] * pose[2]);

    // ---- 姿态: 从 R^T 提取 XYZ 欧拉角 ----
    // 用 R^T 的列 rt0/rt1/rt2 代入 transform_pose 的提取公式：
    //   ry = n2[0]（n2 = 第 3 列）→ 对应 rt2[0] = c0[2] = sry
    double ry = rt2[0];
    if (ry > 1.0) ry = 1.0;          // 防 asin 域外（数值误差）
    if (ry < -1.0) ry = -1.0;

    if (std::fabs(ry) >= 1.0 - 1e-12) {
        // Gimbal Lock: |Ry| = 90°，Rx 与 Rz 退化到同一旋转轴，固定 rz=0。
        // 提取 Rx = ±atan2(R^T[1][0], R^T[1][1])（R^T[1][0] = rt0[1]，R^T[1][1] = rt1[1]）
        out[3] = (ry > 0) ? atan2(rt0[1], rt1[1]) : -atan2(rt0[1], rt1[1]);  // Rx
        out[4] = (ry > 0) ? kPi / 2.0 : -kPi / 2.0;         // Ry = ±90°
        out[5] = 0.0;                                        // Rz = 0
    } else {
        out[3] = atan2(-rt2[1], rt2[2]);    // Rx
        out[4] = asin(ry);                   // Ry
        out[5] = atan2(-rt1[0], rt0[0]);     // Rz
    }
    return true;
}

// ============================================================
// 末端系 → 基坐标 / 用户坐标 变换
// ============================================================

/**
 * @brief 将末端坐标系（相对当前末端）下的目标位姿变换为基坐标系（SDK 坐标系 1）表达
 * @param socket_fd        控制器 socket（6001 端口连接句柄）
 * @param endframe_target  末端坐标系下的目标位姿，至少 6 元素
 *                         [dX, dY, dZ, dRx, dRy, dRz]（位移 mm、姿态 rad）
 * @param out_base_pose    输出: 目标位姿的基坐标系表达，7 元素
 * @return Result::SUCCESS 变换成功；其余为失败原因
 * @note 处理流程: 内部取当前位姿（get_current_position(sock, tl::Coord::BASE)，控制器按
 *       当前 TCP 语义返回）→ 变换末端系目标位姿 T_target = T_cur @ T_rel
 * @note 仅返回 SUCCESS 时 out_base_pose 输出有效；失败时输出被清空
 * @note 逆解依赖当前已选中的工具手编号（set_tool_hand_number 设置），调用前须先启用工具手
 */
tl::Result endframe_to_base(
    const tl::SOCKETFD socket_fd,
    const std::vector<double>& endframe_target,
    std::vector<double>& out_base_pose)
{
    out_base_pose.clear();   // 契约: 失败时输出为空

    if (endframe_target.size() < 6) {
        std::cerr << "[ERROR] 末端系→基坐标变换: 目标位姿元素不足 6 个（实际 "
                  << endframe_target.size() << " 个）" << std::endl;
        return tl::Result::PARAM_ERR;
    }

    // 取当前位姿（基坐标系，控制器按当前 TCP 语义返回）
    std::vector<double> current_pose;
    tl::Result ret = tl::get_current_position(socket_fd, tl::Coord::BASE /* 基坐标 */, current_pose);
    if (ret != tl::Result::SUCCESS || current_pose.size() < 6) {
        std::cerr << "[ERROR] 末端系→基坐标变换: 获取当前位姿失败（ret=" << ret
                  << ", size=" << current_pose.size() << "）" << std::endl;
        return (ret == tl::Result::SUCCESS) ? tl::Result::PARAM_ERR : ret;
    }

    if (!transform_pose(current_pose, endframe_target, out_base_pose)) {
        out_base_pose.clear();
        std::cerr << "[ERROR] 末端系→基坐标变换: 末端系位姿变换失败" << std::endl;
        return tl::Result::PARAM_ERR;
    }
    return tl::Result::SUCCESS;
}

/**
 * @brief 将末端坐标系（相对当前末端）下的目标位姿变换为用户坐标系（SDK 坐标系 3）表达
 * @param socket_fd        控制器 socket（6001 端口连接句柄）
 * @param endframe_target  末端坐标系下的目标位姿，至少 6 元素
 *                         [dX, dY, dZ, dRx, dRy, dRz]（位移 mm、姿态 rad）
 * @param out_user_pose    输出: 目标位姿的用户坐标系表达，7 元素
 * @return Result::SUCCESS 变换成功；其余为失败原因
 * @note 处理流程（全本地计算，不修改控制器用户坐标系状态）:
 *       1. get_user_coord_number 读取当前激活的用户坐标编号（并打印提醒）
 *       2. get_user_coord_para 读取该用户坐标系定义（相对基坐标的位姿 T_user2base）
 *       3. 取当前位姿（基坐标，get_current_position(sock, tl::Coord::BASE)）
 *       4. 复合末端系目标位姿 → 基坐标目标: T_target = T_cur @ T_rel
 *       5. 求逆 T_base2user = inv(T_user2base)，变换基坐标目标 → 用户坐标目标
 * @attention 使用控制器当前激活的用户坐标系（由示教器或 set_user_coord_number 设定），
 *            调用方须确认当前激活编号符合预期；本函数不修改该状态
 * @note 仅返回 SUCCESS 时 out_user_pose 输出有效；失败时输出被清空
 * @note 逆解依赖当前已选中的工具手编号（set_tool_hand_number 设置），调用前须先启用工具手
 */
tl::Result endframe_to_user(
    const tl::SOCKETFD socket_fd,
    const std::vector<double>& endframe_target,
    std::vector<double>& out_user_pose)
{
    out_user_pose.clear();   // 契约: 失败时输出为空

    if (endframe_target.size() < 6) {
        std::cerr << "[ERROR] 末端系→用户坐标变换: 目标位姿元素不足 6 个（实际 "
                  << endframe_target.size() << " 个）" << std::endl;
        return tl::Result::PARAM_ERR;
    }

    // ---- 1. 读取当前激活的用户坐标编号（并提醒）----
    int user_coord_id = -1;
    tl::Result ret = tl::get_user_coord_number(socket_fd, user_coord_id);
    if (ret != tl::Result::SUCCESS || user_coord_id < 0) {
        std::cerr << "[ERROR] 末端系→用户坐标变换: 获取当前用户坐标编号失败（ret="
                  << ret << ", id=" << user_coord_id << "）" << std::endl;
        return (ret == tl::Result::SUCCESS) ? tl::Result::PARAM_ERR : ret;
    }
    std::cout << "  [提醒] 使用控制器当前激活的用户坐标系编号: " << user_coord_id << std::endl;

    // ---- 2. 读取用户坐标系定义（相对基坐标的位姿 T_user2base）----
    std::vector<double> user_para;
    ret = tl::get_user_coord_para(socket_fd, user_coord_id, user_para);
    if (ret != tl::Result::SUCCESS || user_para.size() < 6) {
        std::cerr << "[ERROR] 末端系→用户坐标变换: 获取用户坐标 " << user_coord_id
                  << " 参数失败（ret=" << ret << ", size=" << user_para.size() << "）" << std::endl;
        std::cerr << "        请确认该用户坐标已标定（calculate_user_coordinate 或 set_user_coordinate_data）" << std::endl;
        return (ret == tl::Result::SUCCESS) ? tl::Result::PARAM_ERR : ret;
    }

    // ---- 3. 取当前位姿（基坐标系）----
    std::vector<double> current_pose;
    ret = tl::get_current_position(socket_fd, tl::Coord::BASE /* 基坐标 */, current_pose);
    if (ret != tl::Result::SUCCESS || current_pose.size() < 6) {
        std::cerr << "[ERROR] 末端系→用户坐标变换: 获取当前位姿失败（ret=" << ret
                  << ", size=" << current_pose.size() << "）" << std::endl;
        return (ret == tl::Result::SUCCESS) ? tl::Result::PARAM_ERR : ret;
    }

    // ---- 4. 复合末端系目标位姿 → 基坐标目标 ----
    std::vector<double> target_base;
    if (!transform_pose(current_pose, endframe_target, target_base)) {
        out_user_pose.clear();
        std::cerr << "[ERROR] 末端系→用户坐标变换: 末端系位姿变换失败" << std::endl;
        return tl::Result::PARAM_ERR;
    }

    // ---- 5. 求逆 T_base2user = inv(T_user2base)，变换到用户坐标目标 ----
    std::vector<double> base2user;
    if (!invert_pose(user_para, base2user)) {
        out_user_pose.clear();
        std::cerr << "[ERROR] 末端系→用户坐标变换: 用户坐标系求逆失败" << std::endl;
        return tl::Result::PARAM_ERR;
    }
    if (!transform_pose(base2user, target_base, out_user_pose)) {
        out_user_pose.clear();
        std::cerr << "[ERROR] 末端系→用户坐标变换: 基坐标目标变换到用户坐标失败" << std::endl;
        return tl::Result::PARAM_ERR;
    }
    return tl::Result::SUCCESS;
}

// ============================================================
// 通用逆解接口
// ============================================================

/**
 * @brief 逆解求解坐标系
 * @note SDK 的 3→0（用户坐标）与 1→0（基坐标）逆解结果可能不同
 *       （控制器内部求解路径差异），故 USER 模式必须走真正的 3→0 逆解
 *      才能得到用户坐标系语义的关节角；solve_coord 决定逆解路径与回验方式
 */
enum class IkSolveCoord : int {
    BASE = 1,   // 基坐标(1) → 关节坐标(0) 逆解
    USER = 3,   // 用户坐标(3) → 关节坐标(0) 逆解，使用控制器当前激活的用户坐标系
};

/**
 * @brief 末端系逆解接口: 将末端坐标系（相对当前末端）下的目标位姿逆解为关节坐标
 * @param socket_fd        控制器 socket（6001 端口连接句柄）
 * @param endframe_target  末端坐标系下的目标位姿，至少 6 元素
 *                         [dX, dY, dZ, dRx, dRy, dRz]（位移 mm、姿态 rad）
 *                         —— 表示目标相对"当前末端"的位姿（如 [0,0,195,0,0,0] =
 *                         沿当前末端 Z 轴正前方 195mm）。相机识别+手眼变换的
 *                         结果即为此形式（T_target2tool = T_cam2gripper @ T_target2cam），
 *                         拖动示教、力控偏移等场景的目标位姿同样适用
 * @param reference_joints 参考关节角 [°]，用于形态选择（推荐传当前关节角；可为空）
 * @param joint_solution   输出: 逆解关节角 [°]（7 元素，前 6 位本体关节 + 第 7 位外部轴占位）
 * @param solve_coord      逆解路径:
 *                         IkSolveCoord::BASE（默认，基坐标 1→0）/
 *                         IkSolveCoord::USER（用户坐标 3→0，使用控制器当前激活的用户坐标系）
 * @return Result::SUCCESS 逆解成功；其余为失败原因
 * @note 处理流程:
 *       1. 按 solve_coord 确定逆解输入:
 *          - BASE: 取当前位姿（基坐标系）→ 复合末端系目标位姿 → 基坐标目标
 *          - USER: endframe_to_user 本地变换目标位姿到当前激活用户坐标系表达
 *            （get_user_coord_number 读编号 + get_user_coord_para 读定义 + invert_pose 求逆）
 *       2. 逆解: 源坐标系 → 关节坐标(0):
 *          - BASE: 基坐标(1) → 关节坐标(0)
 *          - USER: 用户坐标(3) → 关节坐标(0)；SDK 的 3→0 与 1→0 结果可能不同，
 *            此处保留控制器原生的用户坐标求解路径
 * @attention USER 模式使用控制器当前激活的用户坐标系（由示教器或 set_user_coord_number
 *            设定），调用方须确认当前激活编号符合预期；本函数不修改该状态
 * @note 仅返回 SUCCESS 时 joint_solution 输出有效；失败时输出被清空，调用方不应读取
 * @note 逆解依赖当前已选中的工具手编号（set_tool_hand_number 设置），调用前须先启用工具手
 * @warning 目标点超出机械臂工作空间或工具手/用户坐标未标定正确时，逆解会失败并返回错误码
 */
tl::Result solve_endframe_ik(
    const tl::SOCKETFD socket_fd,
    const std::vector<double>& endframe_target,
    const std::vector<double>& reference_joints,
    std::vector<double>& joint_solution,
    const IkSolveCoord solve_coord = IkSolveCoord::BASE)
{
    joint_solution.clear();   // 契约: 失败时输出为空；成功后由下方填充

    if (endframe_target.size() < 6) {
        std::cerr << "[ERROR] 末端系逆解: 目标位姿元素不足 6 个（实际 "
                  << endframe_target.size() << " 个）" << std::endl;
        return tl::Result::PARAM_ERR;
    }

    // ---- 1. 按 solve_coord 确定逆解输入与源坐标系 ----
    tl::Result ret = tl::Result::SUCCESS;
    tl::Coord origin_coord = tl::Coord::BASE;  // 默认: 基坐标系（直角坐标系）
    std::vector<double> ik_input;
    if (solve_coord == IkSolveCoord::USER) {
        // USER: 目标位姿本地变换到当前激活用户坐标系表达，走 SDK 原生 3→0 逆解
        ret = endframe_to_user(socket_fd, endframe_target, ik_input);
        if (ret != tl::Result::SUCCESS) {
            return ret;   // 变换函数已输出错误信息
        }
        origin_coord = tl::Coord::USER;              // 用户坐标系
    } else {
        // BASE: 取当前位姿（基坐标系）→ 复合末端系目标位姿 → 基坐标目标
        std::vector<double> current_pose;
        ret = tl::get_current_position(socket_fd, tl::Coord::BASE /* 基坐标 */, current_pose);
        if (ret != tl::Result::SUCCESS || current_pose.size() < 6) {
            std::cerr << "[ERROR] 末端系逆解: 获取当前位姿失败（ret=" << ret
                      << ", size=" << current_pose.size() << "）" << std::endl;
            std::cerr << "        请确认伺服已上电且工具手编号设置正确" << std::endl;
            return (ret == tl::Result::SUCCESS) ? tl::Result::PARAM_ERR : ret;
        }
        if (!transform_pose(current_pose, endframe_target, ik_input)) {
            std::cerr << "[ERROR] 末端系逆解: 末端系位姿变换失败" << std::endl;
            return tl::Result::PARAM_ERR;
        }
    }

    // ---- 4. 逆解: 源坐标系 -> 关节坐标(0) ----
    // SDK 协议点位统一为 7 元素，不足补 0
    if (ik_input.size() < 7) {
        ik_input.resize(7, 0.0);
    }
    // form=0 形态自适应；reference_pos 传入参考关节角，逆解时优先选择最近的形态
    bool convert_state = false;
    ret = tl::get_origin_coord_to_target_coord(socket_fd, origin_coord, ik_input,
                                           tl::Coord::JOINT /* 关节 */, joint_solution, convert_state,
                                           0 /* form */, reference_joints);
    if (ret != tl::Result::SUCCESS || !convert_state) {
        joint_solution.clear();   // 失败时输出无效，清空防止调用方读到残留值
        std::cerr << "[ERROR] 末端系逆解失败（ret=" << ret
                  << ", convert_state=" << convert_state << "）" << std::endl;
        std::cerr << "        请检查: 目标点是否在机械臂工作空间内 / 工具手与用户坐标是否标定正确" << std::endl;
        // 底层调用成功但求解失败（如目标点不可达、形态不存在），统一映射为参数错误
        return (ret == tl::Result::SUCCESS) ? tl::Result::PARAM_ERR : ret;
    }
    return tl::Result::SUCCESS;
}

// ============================================================
// 伺服上电
// ============================================================

/// @brief 清除错误并执行标准两步上电流程
/// @return true 上电成功（伺服状态=3 运行）
bool enable_servo(const int socket_fd)
{
    std::cout << "--- 伺服上电 ---" << std::endl;

    // 1. 先清错、下电复位，确保控制器状态同步
    tl::clear_error(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 2. 检查当前伺服状态
    int state = -1;
    auto ret = tl::get_servo_state(socket_fd, state);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
        return false;
    }
    std::cout << "  [信息] 当前伺服状态: " << state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

    if (state == 2) {
        std::cerr << "[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试" << std::endl;
        return false;
    }

    if (state == 3) {
        std::cout << "  [信息] 机械臂已在使能上电状态" << std::endl;
        return true;
    }

    // 3. 两步上电: 伺服就绪 -> 上电使能
    if (state == 0) {
        std::cout << "  [信息] 设置伺服就绪..." << std::endl;
        ret = tl::set_servo_state(socket_fd, 1);
        if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
            std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "  [信息] 上电使能..." << std::endl;
    ret = tl::set_servo_poweron(socket_fd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 4. 确认上电结果
    tl::get_servo_state(socket_fd, state);
    if (state == 3) {
        std::cout << "  [信息] 上电成功（servo_state = " << state << "）" << std::endl;
        return true;
    }
    std::cerr << "[ERROR] 上电失败（servo_state = " << state << "）" << std::endl;
    return false;
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // ---- 参数解析 ----
    // 用法: ./ex_tool_coord_ik [-c base|user] [X Y Z Rx Ry Rz]
    //   -c 逆解路径: base=基坐标 1→0（默认）/ user=用户坐标 3→0
    //      （使用控制器当前激活的用户坐标系）
    //   可选 6 个位置参数为末端坐标系下的目标位姿，位移 mm、姿态 rad；
    //   缺省时使用默认演示点（沿末端 Z 轴 +100mm）。
    IkSolveCoord solve_coord = IkSolveCoord::BASE;
    std::vector<double> target_args;     // 命令行指定的末端系目标位姿；空 = 使用默认演示点
    std::string ip = "192.168.1.13";     // 控制器 IP，默认 192.168.1.13，可用 -ip 覆盖
    int argi = 1;
    while (argi < argc) {
        std::string arg(argv[argi]);
        if (arg == "-c") {
            if (argi + 1 >= argc) {
                std::cerr << "[ERROR] 用法: " << argv[0]
                          << " [-c base|user] [X Y Z Rx Ry Rz]" << std::endl;
                return -1;
            }
            std::string coord_str(argv[argi + 1]);
            if (coord_str == "base") {
                solve_coord = IkSolveCoord::BASE;
            } else if (coord_str == "user") {
                solve_coord = IkSolveCoord::USER;
            } else {
                std::cerr << "[ERROR] 未知逆解路径 '" << coord_str
                          << "'（可选: base / user）" << std::endl;
                return -1;
            }
            argi += 2;
        } else if (arg == "-ip" || arg == "--ip") {
            if (argi + 1 >= argc) {
                std::cerr << "[ERROR] 用法: " << argv[0]
                          << " [-c base|user] [-ip 192.168.1.13] [X Y Z Rx Ry Rz]" << std::endl;
                return -1;
            }
            ip = argv[argi + 1];
            argi += 2;
        } else {
            break;   // 首个位置参数出现，剩余全部按位姿参数解析
        }
    }
    int pose_count = argc - argi;
    if (pose_count != 0 && pose_count != 6) {
        std::cerr << "[ERROR] 用法: " << argv[0]
                  << " [-c base|user] [X Y Z Rx Ry Rz]" << std::endl;
        std::cerr << "        位姿参数须为 6 个（X Y Z Rx Ry Rz），姿态单位为 rad；缺省时使用默认演示点" << std::endl;
        return -1;
    }
    for (int i = argi; i < argc; ++i) {
        try {
            std::size_t parsed = 0;
            double v = std::stod(argv[i], &parsed);
            if (parsed != std::string(argv[i]).size()) {
                std::cerr << "[ERROR] 参数 " << i << " ('" << argv[i]
                          << "') 不是有效数字，请检查输入" << std::endl;
                return -1;
            }
            target_args.push_back(v);
        } catch (const std::exception&) {
            std::cerr << "[ERROR] 参数 " << i << " ('" << argv[i]
                      << "') 不是有效数字，请检查输入" << std::endl;
            return -1;
        }
    }
    std::cout << "[信息] 逆解路径: "
              << (solve_coord == IkSolveCoord::BASE ? "基坐标 1→0 (base)"
                  : "用户坐标 3→0 (user, 使用当前激活的用户坐标系)") << std::endl;

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    tl::SOCKETFD sock = tl::connect_robot(ip, "6001");
    if (sock < 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket=" << sock << ")" << std::endl;

    tl::Result ret;

    // ---- 切换示教模式（运动指令需要）----
    ret = tl::set_current_mode(sock, 0);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 切换示教模式失败: " << ret << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 上电 ----
    if (!enable_servo(sock)) {
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 1. 启用 2 号工具手（用户已标定）----
    // 工具坐标系由工具手（TCP）标定结果定义，必须先选中正确的工具手编号。
    // 标定说明: 优先在示教器上手动标定 2 号工具手（更快，实时可视化）；
    //           也可用代码标定: 先 set_tool_hand_number 选中该工具手，再采集标定点
    //           （tool_hand_2_or_20_point_calibrate 2/20 点法 或 tool_hand_7_point_calibrate 7 点法），
    //           最后 tool_hand_7_point_calibrate_caculate / tool_hand_2_or_20_point_calibrate_caculate 计算保存。
    //           本示例假设已标定完成，直接启用即可。
    print_separator("启用工具手");
    constexpr int kToolNum = 2;    // 用户标定的工具手编号
    ret = tl::set_tool_hand_number(sock, kToolNum);
    print_result("set_tool_hand_number(2)", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置工具手编号失败，请确认示教器上已标定 2 号工具手" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    // 控制器对 set 的写入是异步的，需延迟后才能正确读取（与 ex_tool_hand_number 约定一致）
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int cur_tool_num = -1;
    ret = tl::get_tool_hand_number(sock, cur_tool_num);
    print_result("get_tool_hand_number", ret);
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [信息] 当前工具手编号: " << cur_tool_num << std::endl;
    }

    // ---- 1.5 用户坐标模式: 确认当前激活的用户坐标系并校验已标定 ----
    // 标定说明: 优先在示教器上手动标定用户坐标（更快，实时可视化）；
    //           也可用代码标定: calculate_user_coordinate 三点 OXY 标定计算，
    //           或 set_user_coordinate_data 直接写入已知的用户坐标数据。
    //           本示例假设已标定完成，直接校验即可。
    // 注意: USER 模式使用控制器当前激活的用户坐标系（由示教器或
    // set_user_coord_number 设定），本示例不修改该状态，仅读取并提醒。
    if (solve_coord == IkSolveCoord::USER) {
        print_separator("确认用户坐标");
        int user_coord_cur = -1;
        ret = tl::get_user_coord_number(sock, user_coord_cur);
        print_result("get_user_coord_number", ret);
        if (ret != tl::Result::SUCCESS || user_coord_cur < 0) {
            std::cerr << "[ERROR] 获取当前用户坐标编号失败（ret=" << ret << "）" << std::endl;
            tl::disconnect_robot(sock);
            return -1;
        }
        std::cout << "  [提醒] 将使用控制器当前激活的用户坐标系编号: " << user_coord_cur << std::endl;
        std::cout << "  [提醒] 如需其他编号，请先在示教器或代码中 set_user_coord_number 切换" << std::endl;
        std::vector<double> user_para;
        ret = tl::get_user_coord_para(sock, user_coord_cur, user_para);
        print_result("get_user_coord_para(" + std::to_string(user_coord_cur) + ")", ret);
        if (ret != tl::Result::SUCCESS || user_para.size() < 6) {
            std::cerr << "[ERROR] 获取用户坐标 " << user_coord_cur << " 参数失败，"
                      << "请确认示教器上已标定该用户坐标" << std::endl;
            tl::disconnect_robot(sock);
            return -1;
        }
        print_vector("用户坐标定义 [mm, rad]（相对基坐标）", user_para);
    }

    // ---- 2. 获取当前关节角（作为逆解参考点，选择最近形态）----
    print_separator("获取当前位姿");
    std::vector<double> current_joints;
    ret = tl::get_current_position(sock, tl::Coord::JOINT /* 关节坐标系 */, current_joints);
    print_result("get_current_position(关节)", ret);
    if (ret != tl::Result::SUCCESS || current_joints.size() < 6) {
        std::cerr << "[ERROR] 获取当前关节角失败，请确认伺服已上电" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    print_vector("当前关节角 [°]", current_joints);

    // ---- 3. 确定目标点: 末端坐标系下的目标位姿 ----
    // 命令行参数为末端坐标系（相对当前末端）下的目标位姿（相机识别 + 手眼变换、
    // 拖动示教、力控偏移等场景均适用；T_target2tool = T_cam2gripper @ T_target2cam
    // 由调用方完成，参考 examples/py/ex_handeye_calibration.py 的标定流程）。
    // 本接口内部将其复合到当前基坐标位姿后按逆解路径求解:
    // base → 基坐标(1)→关节(0) 直接逆解 / user → 本地变换到当前激活用户坐标后 3→0 逆解。
    print_separator("目标位姿（末端坐标系下）");

    std::vector<double> endframe_target;   // 末端系目标位姿（solve_endframe_ik 的输入）
    if (!target_args.empty()) {
        // 用户通过命令行指定了末端系目标位姿
        endframe_target = target_args;
        std::cout << "  [信息] 使用命令行指定的末端系目标位姿" << std::endl;
    } else {
        // 默认演示点: 沿当前末端 Z 轴正方向偏移 100mm（相对末端，无需当前位姿）
        endframe_target = {0.0, 0.0, 100.0, 0.0, 0.0, 0.0};
        std::cout << "  [信息] 未指定目标点，使用默认演示点（沿末端 Z 轴 +100mm）" << std::endl;
    }
    std::cout << "  目标位姿 [dX,dY,dZ]=[" << endframe_target[0] << ", " << endframe_target[1] << ", " << endframe_target[2]
              << "]mm  [dRx,dRy,dRz]=[" << endframe_target[3] << ", " << endframe_target[4] << ", " << endframe_target[5]
              << "]rad（相对当前末端）" << std::endl;

    // ---- 4. 逆解: 末端系位姿复合当前位姿 -> 关节坐标(0) ----
    // 调用封装的末端系逆解接口，内部完成: 取当前位姿（基坐标）→ 复合末端系目标位姿
    // → 按逆解路径求解: base 基坐标(1)→关节(0) / user 用户坐标(3)→关节(0)
    //   （USER 模式目标位姿本地变换到当前激活用户坐标，SDK 3→0 与 1→0 结果可能不同）；
    // reference 传入当前关节角，逆解时优先选择最近的形态
    print_separator("逆解: 末端系目标 -> 关节坐标");
    std::vector<double> joint_solution;
    ret = solve_endframe_ik(sock, endframe_target, current_joints, joint_solution,
                            solve_coord);
    print_result("solve_endframe_ik (末端系->关节)", ret);
    if (ret != tl::Result::SUCCESS) {
        tl::disconnect_robot(sock);
        return -1;
    }
    print_vector("逆解关节坐标 [°]", joint_solution);

    // ---- 5. 正解回验: 关节坐标(0) -> 目标坐标系，验证逆解一致性 ----
    print_separator("正解回验");
    // 回验基准 = 末端系目标位姿变换到目标坐标系（与 solve_endframe_ik 内部一致，
    // 机械臂未运动故当前位姿不变）
    // 回验路径: 关节角正解到目标坐标系（base→基坐标1 / user→用户坐标3，
    // 用户坐标系使用控制器当前激活编号），与基准对比
    std::vector<double> target_ref;    // 回验基准位姿（目标坐标系下表达）
    ret = (solve_coord == IkSolveCoord::USER)
              ? endframe_to_user(sock, endframe_target, target_ref)
              : endframe_to_base(sock, endframe_target, target_ref);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 正解回验: 末端系位姿变换失败（ret=" << ret << "）" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    std::vector<double> pose_back;
    bool back_state = false;
    tl::Coord back_coord = (solve_coord == IkSolveCoord::USER)
                               ? tl::Coord::USER      /* 用户 */
                               : tl::Coord::BASE /* 基坐标 */;
    ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::JOINT /* 关节 */, joint_solution,
                                           back_coord, pose_back, back_state);
    print_result("get_origin_coord_to_target_coord (关节->目标坐标系)", ret);
    if (ret == tl::Result::SUCCESS && back_state && pose_back.size() >= 6) {
        print_vector("正解回验位姿（目标坐标系下） [mm, rad]", pose_back);
        double pos_err = 0.0, att_err = 0.0;
        for (int i = 0; i < 3; ++i) {
            pos_err += std::fabs(pose_back[i] - target_ref[i]);
        }
        for (int i = 3; i < 6; ++i) {
            // 姿态角相差 2π 等价，取最小角度差
            double d = std::fabs(pose_back[i] - target_ref[i]);
            d = std::fmod(d, 2.0 * kPi);
            if (d > kPi) d = 2.0 * kPi - d;
            att_err += d;
        }
        std::cout << "  位置误差 [mm]: " << pos_err << "（应接近 0）" << std::endl;
        std::cout << "  姿态误差 [rad]: " << att_err << "（应接近 0）" << std::endl;
    } else {
        // 逆解已由控制器校验成功（convert_state=true），回验失败多为状态抖动/网络瞬时错误，
        // 仅跳过误差对比，继续执行运动（保守起见也可在此 return -1 中止，按需调整）。
        std::cerr << "[警告] 正解回验失败（ret=" << ret << ", back_state=" << back_state
                  << ", size=" << pose_back.size() << "），跳过误差对比，继续运动" << std::endl;
    }

    // ---- 6. MoveJ 下发逆解关节角 ----
    print_separator("MoveJ 下发逆解关节角");
    std::cout << "  [信息] 即将运动到逆解点，请确认机械臂周围安全..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 设置全局速度倍率
    ret = tl::set_speed(sock, 30);
    print_result("set_speed(30)", ret);
    // 控制器对 set 的写入是异步的，延迟后再下发运动指令
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    tl::MoveCmd move_cmd;
    move_cmd.targetPosType  = tl::PosType::data;    // 直接使用数值点位
    move_cmd.coord          = tl::Coord::JOINT;     // 关节坐标系
    move_cmd.velocity       = 40;               // 关节速度 (1,100]（最大速度百分比）
    move_cmd.acc            = 10;               // 加速度 (1,100]（百分比）
    move_cmd.dec            = 10;               // 减速度 (1,100]（百分比）
    move_cmd.pl             = 0;                // 平滑参数 [0,5]
    // 逆解关节角 [°]：前 6 位为 6 轴本体关节，第 7 位为 7 轴本体槽（6 轴机型补 0 占位）
    move_cmd.targetPosValue = joint_solution;
    if (move_cmd.targetPosValue.size() < 7) {
        move_cmd.targetPosValue.resize(7, 0.0);
    }

    ret = tl::robot_movej(sock, move_cmd);
    print_result("robot_movej", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] MoveJ 下发失败，程序退出!" << std::endl;
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    // robot_movej 为非阻塞接口，轮询当前关节角直到接近目标（逐轴阈值 0.5°），超时 20s
    constexpr double kArriveTol = 0.5;          // 到位阈值 [°]
    constexpr int kWaitStepMs = 200;            // 轮询周期 [ms]
    constexpr int kWaitTimeoutMs = 20000;       // 到位超时 [ms]
    std::vector<double> cur_joints;
    bool arrived = false;
    for (int waited = 0; waited < kWaitTimeoutMs; waited += kWaitStepMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kWaitStepMs));
        ret = tl::get_current_position(sock, tl::Coord::JOINT, cur_joints);
        if (ret != tl::Result::SUCCESS) continue;
        // 逐轴判断: 所有关节最大偏差小于阈值视为到位（6 轴/7 轴通用，按实际返回轴数比较）
        size_t n = std::min(cur_joints.size(), joint_solution.size());
        if (n == 0) continue;
        double max_err = 0.0;
        for (size_t i = 0; i < n; ++i) {
            max_err = std::max(max_err, std::fabs(cur_joints[i] - joint_solution[i]));
        }
        if (max_err < kArriveTol) {
            arrived = true;
            break;
        }
    }
    if (arrived) {
        std::cout << "  [信息] MoveJ 运动到位" << std::endl;
        print_vector("到位关节角 [°]", cur_joints);
    } else {
        std::cerr << "[ERROR] MoveJ 运动超时（20s 内未到位），下电停止运动" << std::endl;
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 断开连接 ----
    print_separator("断开连接");
    tl::set_servo_poweroff(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    tl::disconnect_robot(sock);
    std::cout << "[信息] 已断开连接，机械臂切换到示教模式" << std::endl;

    std::cout << "\n[信息] 末端系逆解示例程序运行完毕" << std::endl;
    return 0;
}
