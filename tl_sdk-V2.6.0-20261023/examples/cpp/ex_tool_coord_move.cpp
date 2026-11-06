/**
 * ex_tool_coord_move.cpp
 * @brief 实现末端在工具坐标系下增量点动的示例程序（模拟示教器上在工具坐标系下的点动效果）
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 需要双端口连接: 6001（控制）+ 7000（伺服高频透传）
 *   - servoJ 周期约 10ms
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 *   - 运行模式: 透传模式（连接 7000 端口后调用 open_servoJ 进入透传模式）
 *   - 开始前需: 双端口连接 → 上电 → open_servoJ
 *   - 退出前需: close_servoJ → tl::set_current_mode(0) → tl::disconnect_robot(双端口)
 *   - 工具坐标平移单位 mm，旋转单位 rad
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_tool_coord_move
 */

#include "tl_api.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#endif
// ============================================================
// 工具函数
// ============================================================

void print_result(const std::string& msg, int ret)
{
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [成功] " << msg << std::endl;
    } else {
        std::cout << "  [失败] " << msg << "，错误码: " << ret << std::endl;
    }
}

void print_separator(const std::string& title)
{
    std::cout << "\n========== " << title << " ==========" << std::endl;
}

/// @brief 打印工具坐标（7 元素）
void print_tool_coord(const std::vector<double>& pos)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  TX=" << pos[0] << "mm"
              << "  TY=" << pos[1] << "mm"
              << "  TZ=" << pos[2] << "mm"
              << "  Rx=" << pos[3] << "rad"
              << "  Ry=" << pos[4] << "rad"
              << "  Rz=" << pos[5] << "rad" << std::endl;
}

// ============================================================
// 工具坐标系增量计算
// ============================================================

/**
 * @brief 在工具坐标系下计算增量后的目标工具坐标
 * @param current_tool_pos  当前工具坐标 [TX, TY, TZ, Rx, Ry, Rz]（位移 mm，姿态 rad）
 * @param axis_idx  轴索引: 0=X, 1=Y, 2=Z, 3=Rx, 4=Ry, 5=Rz
 * @param delta     增量值（平移 mm，旋转 rad）
 * @param is_rotation 是否为旋转增量
 * @param out_pos   输出：增量后的工具坐标
 * @return true 计算成功
 *
 * @note 姿态使用 XYZ 欧拉角（绕固定轴顺序 X→Y→Z），增量采用右乘方式：
 *       平移增量沿工具坐标系方向投影到世界系；
 *       旋转增量右乘到当前姿态的旋转矩阵，再提取新的 XYZ 欧拉角。
 */
bool apply_tool_delta(
    const std::vector<double>& current_tool_pos,
    int axis_idx, double delta, bool is_rotation,
    std::vector<double>& out_pos)
{
    // ---- 当前旋转矩阵（R = Rz * Ry * Rx，列主序存储三列）----
    // XYZ 欧拉角: R = Rx(roll) * Ry(pitch) * Rz(yaw)
    double crx = cos(current_tool_pos[3]), srx = sin(current_tool_pos[3]);
    double cry = cos(current_tool_pos[4]), sry = sin(current_tool_pos[4]);
    double crz = cos(current_tool_pos[5]), srz = sin(current_tool_pos[5]);

    // 旋转矩阵的三列 = 工具坐标系 X/Y/Z 轴在世界系的方向向量
    double col0[3] = {cry * crz, crx * srz + srx * sry * crz, srx * srz - crx * sry * crz};
    double col1[3] = {-cry * srz, crx * crz - srx * sry * srz, srx * crz + crx * sry * srz};
    double col2[3] = {sry, -srx * cry, crx * cry};

    out_pos = current_tool_pos;

    if (is_rotation) {
        // ---- 姿态增量：在工具坐标系下绕对应轴旋转 delta 弧度 ----
        // R_new = R_current * R_delta(axis) （右乘 = 工具系中的旋转）
        double cd = cos(delta), sd = sin(delta);
        double new_col0[3], new_col1[3], new_col2[3];

        if (axis_idx == 3) {        // Rx: 绕工具 X 轴旋转
            for (int i = 0; i < 3; i++) {
                new_col0[i] = col0[i];
                new_col1[i] = cd * col1[i] + sd * col2[i];
                new_col2[i] = -sd * col1[i] + cd * col2[i];
            }
        } else if (axis_idx == 4) { // Ry: 绕工具 Y 轴旋转
            for (int i = 0; i < 3; i++) {
                new_col0[i] = cd * col0[i] - sd * col2[i];
                new_col1[i] = col1[i];
                new_col2[i] = sd * col0[i] + cd * col2[i];
            }
        } else {                     // Rz: 绕工具 Z 轴旋转
            for (int i = 0; i < 3; i++) {
                new_col0[i] =  cd * col0[i] + sd * col1[i];
                new_col1[i] = -sd * col0[i] + cd * col1[i];
                new_col2[i] = col2[i];
            }
        }

        // 从新旋转矩阵提取 XYZ 欧拉角
        out_pos[3] = atan2(-new_col2[1], new_col2[2]);    // Rx
        out_pos[4] = asin(new_col2[0]);                    // Ry
        out_pos[5] = atan2(-new_col1[0], new_col0[0]);     // Rz
    } else {
        // ---- 位置增量：沿工具坐标系方向平移 ----
        // 选择对应的工具轴方向向量，叠加到世界系坐标
        const double* axis = (axis_idx == 0) ? col0
                           : (axis_idx == 1) ? col1
                           : col2;
        out_pos[0] += delta * axis[0];
        out_pos[1] += delta * axis[1];
        out_pos[2] += delta * axis[2];
    }

    return true;
}

// ============================================================
// 伺服上电/下电
// ============================================================

bool enable_servo(const int socket_fd)
{
    std::cout << "--- 检查伺服状态 ---" << std::endl;

    // 1. 先清除控制器错误和报警，再下电复位确保状态同步
    tl::clear_error(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 2. 检查伺服状态
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
        std::cout << "[INFO] 机械臂已在使能上电状态" << std::endl;
        return true;
    }

    // 3. 上电流程
    if (state == 0) {
        std::cout << "  [信息] 伺服处于停止状态，设置就绪..." << std::endl;
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

    // 4. 确认结果
    tl::get_servo_state(socket_fd, state);
    if (state == 3) {
        std::cout << "[INFO] 机械臂使能上电成功（servo_state = " << state << "）" << std::endl;
        return true;
    } else {
        std::cerr << "[ERROR] 机械臂使能上电失败（servo_state = " << state << "）" << std::endl;
        return false;
    }
}

void disable_servo(const int socket_fd)
{
    int state = -1;
    tl::get_servo_state(socket_fd, state);
    switch (state) {
        case 1:
            std::cout << "[INFO] 机械臂已使能下电" << std::endl;
            break;
        case 3:
            tl::set_servo_poweroff(socket_fd);
            tl::get_servo_state(socket_fd, state);
            std::cout << "[INFO] 机械臂使能下电成功（servo_state = " << state << "）" << std::endl;
            return;
        default:
            break;
    }
    std::cout << "[INFO] 机械臂使能下电失败（servo_state = " << state << "）" << std::endl;
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 解析命令行参数: ./ex_tool_coord_move [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    // ---- 获取 SDK 版本 ----
    {
        std::string version = tl::get_library_version();
        std::cout << "[信息] SDK 版本: " << version << std::endl;
    }

    // ---- 双端口连接 ----
    print_separator("双端口连接");
    tl::SOCKETFD sock = tl::connect_robot(ip, "6001");
    if (sock <= 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败" << std::endl;
        return -1;
    }
    tl::SOCKETFD sock_servo = tl::connect_robot(ip, "7000");
    if (sock_servo <= 0) {
        std::cerr << "[ERROR] 连接 7000 端口失败" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    std::cout << "[信息] 双端口连接成功 (6001=" << sock << ", 7000=" << sock_servo << ")" << std::endl;

    tl::Result ret;

    // ---- 伺服上电 ----
    print_separator("伺服上电");
    if (!enable_servo(sock)) {
        tl::disconnect_robot(sock);
        tl::disconnect_robot(sock_servo);
        return -1;
    }

    // ---- 切换为运行模式（透传模式）----
    print_separator("切换为运行模式");
    ret = tl::set_current_mode(sock, 2);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 切换运行模式失败: " << ret << std::endl;
        disable_servo(sock);
        tl::disconnect_robot(sock);
        tl::disconnect_robot(sock_servo);
        return -1;
    }
    std::cout << "  [信息] 已切换为运行模式" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ---- 打开 servoJ 模式 ----
    print_separator("打开 servoJ 透传模式");
    // 注意: SDK 底层协议要求 servoJ 相关 vector 固定为 7 元素
    std::vector<double> vmax(7,300.0);
    std::vector<double> amax(7,3000.0);
    std::vector<double> jmax(7,50000.0);

    ret = tl::open_servoJ(sock_servo, vmax, amax, jmax);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] open_servoJ 失败: " << ret << std::endl;
        disable_servo(sock);
        tl::disconnect_robot(sock);
        tl::disconnect_robot(sock_servo);
        return -1;
    }
    std::cout << "  [信息] servoJ 透传模式已打开" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- 交互式工具坐标系点动控制 ----
    print_separator("工具坐标系增量点动");
    std::cout << "说明: 在工具坐标系下对末端进行平移/旋转增量点动。" << std::endl;
    std::cout << "      每次操作: 获取当前工具坐标 → 计算增量 → 逆解至关节角 → servoJ 下发" << std::endl;
    std::cout << "      平移单位: mm    旋转单位: rad" << std::endl;
    std::cout << std::endl;

    while (true) {
        std::cout << "\n--- 点动控制菜单 ---" << std::endl;
        std::cout << "  [平移]" << std::endl;
        std::cout << "    1. TX+ (沿工具 X 正方向 +10mm)" << std::endl;
        std::cout << "    2. TX- (沿工具 X 负方向 -10mm)" << std::endl;
        std::cout << "    3. TY+ (沿工具 Y 正方向 +10mm)" << std::endl;
        std::cout << "    4. TY- (沿工具 Y 负方向 -10mm)" << std::endl;
        std::cout << "    5. TZ+ (沿工具 Z 正方向 +10mm)" << std::endl;
        std::cout << "    6. TZ- (沿工具 Z 负方向 -10mm)" << std::endl;
        std::cout << "  [旋转]" << std::endl;
        std::cout << "    7. Rx+ (绕工具 X 轴 +0.1rad)" << std::endl;
        std::cout << "    8. Rx- (绕工具 X 轴 -0.1rad)" << std::endl;
        std::cout << "    9. Ry+ (绕工具 Y 轴 +0.1rad)" << std::endl;
        std::cout << "    10. Ry- (绕工具 Y 轴 -0.1rad)" << std::endl;
        std::cout << "    11. Rz+ (绕工具 Z 轴 +0.1rad)" << std::endl;
        std::cout << "    12. Rz- (绕工具 Z 轴 -0.1rad)" << std::endl;
        std::cout << "  [其他]" << std::endl;
        std::cout << "    0. 退出" << std::endl;
        std::cout << "请输入指令: ";

        int cmd = 0;
        std::cin >> cmd;

        if (cmd == 0) {
            std::cout << "  [信息] 退出点动控制" << std::endl;
            break;
        }

        if (cmd < 1 || cmd > 12) {
            std::cout << "  [错误] 无效输入，请输入 0-12" << std::endl;
            continue;
        }

        // ---- 解析增量参数 ----
        int axis_idx = 0;       // 0=X, 1=Y, 2=Z, 3=Rx, 4=Ry, 5=Rz
        double delta = 10.0;    // 平移mm 或 旋转rad
        bool is_rotation = false;
        const char* axis_name = "";

        switch (cmd) {
            case 1:  axis_idx = 0; delta =  10.0; axis_name = "TX"; break;
            case 2:  axis_idx = 0; delta = -10.0; axis_name = "TX"; break;
            case 3:  axis_idx = 1; delta = -10.0; axis_name = "TY"; break;
            case 4:  axis_idx = 1; delta =  10.0; axis_name = "TY"; break;
            case 5:  axis_idx = 2; delta = -10.0; axis_name = "TZ"; break;
            case 6:  axis_idx = 2; delta =  10.0; axis_name = "TZ"; break;
            case 7:  axis_idx = 3; delta =   0.1; is_rotation = true; axis_name = "Rx"; break;
            case 8:  axis_idx = 3; delta =  -0.1; is_rotation = true; axis_name = "Rx"; break;
            case 9:  axis_idx = 4; delta =   0.1; is_rotation = true; axis_name = "Ry"; break;
            case 10: axis_idx = 4; delta =  -0.1; is_rotation = true; axis_name = "Ry"; break;
            case 11: axis_idx = 5; delta =   0.1; is_rotation = true; axis_name = "Rz"; break;
            case 12: axis_idx = 5; delta =  -0.1; is_rotation = true; axis_name = "Rz"; break;
        }

        std::cout << "\n  指令: " << axis_name << (delta > 0 ? "+" : "") << delta
                  << (is_rotation ? " rad" : " mm") << std::endl;

        // ---- Step 1: 获取当前工具坐标 ----
        std::vector<double> tool_pos(7, 0.0);
        ret = tl::get_current_position(sock, tl::Coord::TOOL, tool_pos);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "  [错误] 获取工具坐标失败，错误码: " << ret << std::endl;
            continue;
        }
        std::cout << "  当前工具坐标:\n";
        print_tool_coord(tool_pos);

        // ---- Step 2: 计算目标工具坐标（工具坐标系增量）----
        std::vector<double> target_tool_pos;
        if (!apply_tool_delta(tool_pos, axis_idx, delta, is_rotation, target_tool_pos)) {
            std::cerr << "  [错误] 工具坐标增量计算失败" << std::endl;
            continue;
        }
        std::cout << "  目标工具坐标:\n";
        print_tool_coord(target_tool_pos);

        // ---- Step 3: 获取当前关节位置作为逆解参考点 ----
        std::vector<double> ref_joint_pos(7, 0.0);
        ret = tl::get_current_position(sock, tl::Coord::JOINT, ref_joint_pos);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "  [错误] 获取参考关节位置失败，错误码: " << ret << std::endl;
            continue;
        }

        // ---- Step 4: 逆解 工具坐标 → 关节坐标 ----
        // 将工具坐标系（tl::Coord::TOOL）下的目标坐标逆解到关节坐标系（tl::Coord::JOINT）
        std::vector<double> joint_pos(7, 0.0);
        bool convert_state = false;
        ret = tl::get_origin_coord_to_target_coord(
            sock,
            tl::Coord::TOOL,     // originCoord: 工具坐标系
            target_tool_pos,     // originPos: 目标工具坐标
            tl::Coord::JOINT,    // targetCoord: 关节坐标系
            joint_pos,           // 输出: 关节坐标
            convert_state,       // 输出: 逆解是否成功
            0,                   // form: 形态
            ref_joint_pos        // reference_pos: 参考点（当前关节位置）
        );

        if (ret != tl::Result::SUCCESS || !convert_state) {
            std::cerr << "  [错误] 逆解失败: ret=" << ret
                      << " convert_state=" << convert_state << std::endl;
            continue;
        }

        // ---- 打印逆解结果 ----
        std::cout << "  逆解关节坐标: [";
        for (size_t i = 0; i < joint_pos.size(); i++) {
            std::cout << std::fixed << std::setprecision(2) << joint_pos[i];
            if (i < joint_pos.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // ---- Step 5: 通过 servoJ 下发关节角度 ----
        // 平滑过渡：分步发送，从当前关节位置逐步到达目标
        std::vector<double> step_pos(7, 0.0);
        size_t n_axes = joint_pos.size();
        if (n_axes > step_pos.size()) n_axes = step_pos.size();

        // 第一步使用参考位置，后续逐步插值（共 30 步，约 300ms）
        for (size_t j = 0; j < n_axes; ++j) {
            step_pos[j] = ref_joint_pos[j];
        }

        int steps = 30;
        for (int i = 0; i < steps; ++i) {
            for (size_t j = 0; j < n_axes; ++j) {
                step_pos[j] = ref_joint_pos[j]
                            + (joint_pos[j] - ref_joint_pos[j]) * (i + 1) / steps;
            }
            // 多余元素由 step_pos(7) 初始化保持 0，无需额外处理
            tl::set_servoJ_pos(sock_servo, step_pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "  [信息] 已下发 servoJ 到位" << std::endl;

        // 等待机械臂稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // ---- 关闭 servoJ 模式 ----
    print_separator("关闭 servoJ");
    ret = tl::close_servoJ(sock_servo);
    print_result("close_servoJ", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ---- 切回示教模式（自动下电）----
    print_separator("切回示教模式");
    ret = tl::set_current_mode(sock, 0);
    print_result("tl::set_current_mode(0) 切回示教模式（自动下电）", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ---- 断开连接 ----
    print_separator("断开连接");
    tl::disconnect_robot(sock);
    tl::disconnect_robot(sock_servo);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 工具坐标系点动示例程序运行完毕" << std::endl;
    return 0;
}
