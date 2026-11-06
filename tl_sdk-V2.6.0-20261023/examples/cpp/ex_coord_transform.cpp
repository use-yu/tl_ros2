/**
 * ex_coord_transform.cpp
 * @brief 坐标变换算法示例程序 —— 展示常用的坐标系转换接口
 * @attention
 *   - 需要控制器连接正常、已上电
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_coord_transform
 */

// MSVC: 启用 M_PI / M_PI_2 等 POSIX 数学常量
#define _USE_MATH_DEFINES
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
// 跨平台兜底：部分编译器在严格 C++ 模式下不定义 M_PI / M_PI_2
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
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

/// @brief 打印 double 向量
void print_vector(const std::string& name, const std::vector<double>& v, int width = 6)
{
    std::cout << "  " << name << " [" << v.size() << "] = {";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << v[i];
        if (i < v.size() - 1) std::cout << ", ";
    }
    std::cout << "}" << std::endl;
}

/// @brief 打印矩阵（行主序）
void print_matrix(const std::string& name, const std::vector<double>& m, int rows, int cols)
{
    std::cout << "  " << name << " [" << m.size() << "]:" << std::endl;
    for (int r = 0; r < rows; ++r) {
        std::cout << "    ";
        for (int c = 0; c < cols; ++c) {
            int idx = r * cols + c;
            if (idx < (int)m.size()) {
                std::cout << std::fixed << std::setprecision(4) << std::setw(10) << m[idx] << " ";
            }
        }
        std::cout << std::endl;
    }
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 解析命令行参数: ./ex_coord_transform [-ip 192.168.1.13]，默认 192.168.1.13
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
    print_separator("连接控制器");
    tl::SOCKETFD sock = tl::connect_robot(ip, "6001");
    if (sock < 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败" << std::endl;
        return -1;
    }
    tl::SOCKETFD sock_servo = tl::connect_robot(ip, "7000");
    if (sock_servo < 0) {
        std::cerr << "[ERROR] 连接 7000 端口失败" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    std::cout << "[信息] 双端口连接成功 (6001=" << sock << ", 7000=" << sock_servo << ")" << std::endl;

    tl::Result ret;

    // ---- 上电（servoJ 需要上电状态）----
    {
        print_separator("清错与上电");
        tl::set_current_mode(sock, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        tl::clear_error(sock);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        tl::set_servo_poweroff(sock);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        int servo_state = -1;
        ret = tl::get_servo_state(sock, servo_state);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
            tl::disconnect_robot(sock);
            tl::disconnect_robot(sock_servo);
            return -1;
        }
        std::cout << "  [信息] 当前伺服状态: " << servo_state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

        if (servo_state == 2) {
            std::cerr << "[ERROR] 伺服处于报警状态，无法上电" << std::endl;
            tl::disconnect_robot(sock);
            tl::disconnect_robot(sock_servo);
            return -1;
        }

        if (servo_state != 3) {
            std::cout << "  [信息] 伺服未运行，执行上电流程..." << std::endl;
            ret = tl::set_servo_state(sock, 1);
            if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
                std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
                tl::disconnect_robot(sock);
                tl::disconnect_robot(sock_servo);
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            ret = tl::set_servo_poweron(sock);
            if (ret != tl::Result::SUCCESS) {
                std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
                tl::disconnect_robot(sock);
                tl::disconnect_robot(sock_servo);
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cout << "  [信息] 上电成功" << std::endl;
        } else {
            std::cout << "  [信息] 伺服已运行 (servo_state=3)" << std::endl;
        }
    }

    // ============================================================
    // 1. 四元数 ↔ 欧拉角 (RPY) — 正反验证
    // ============================================================
    // 单位说明:
    //   四元数 [x, y, z, w]      — 无量纲（归一化）
    //   欧拉角 [roll, pitch, yaw] — 弧度 rad
    //   旋转矩阵 (3×3 行主序)     — 无量纲
    print_separator("四元数 ↔ 欧拉角");

    // 1a. 欧拉角 → 四元数 → 欧拉角（正反验证）
    {
        std::cout << "--- 1a. RPY ↔ 四元数 正反验证 ---" << std::endl;
        std::vector<double> rpy_in = {0.2, 0.3, 0.5}; // rad
        std::vector<double> quat, rpy_out;

        ret = tl::get_rpy2quat(sock, rpy_in, quat);
        print_result("tl::get_rpy2quat(RPY→四元数)", ret);
        if (ret != tl::Result::SUCCESS) return -1;
        print_vector("  输入欧拉角 [rad]", rpy_in);
        print_vector("  中间四元数 [x, y, z, w]", quat);

        ret = tl::get_quat2rpy(sock, quat, rpy_out);
        print_result("tl::get_quat2rpy(四元数→RPY)", ret);
        if (ret != tl::Result::SUCCESS) return -1;
        print_vector("  还原欧拉角 [rad]", rpy_out);

        if (rpy_out.size() >= 3 && rpy_in.size() >= 3) {
            double err = fabs(rpy_out[0] - rpy_in[0])
                       + fabs(rpy_out[1] - rpy_in[1])
                       + fabs(rpy_out[2] - rpy_in[2]);
            std::cout << "  正反变换误差 (应接近0): " << err << std::endl;
        }
    }

    // 1b. 欧拉角 → 旋转矩阵
    {
        std::cout << "\n--- 1b. get_rpy2r: 欧拉角转旋转矩阵 ---" << std::endl;
        std::vector<double> rpy = {0.0, 0.0, M_PI_2}; // rad
        std::vector<double> rot_mat;

        ret = tl::get_rpy2r(sock, rpy, rot_mat);
        print_result("get_rpy2r", ret);
        print_vector("输入欧拉角 [rad]", rpy);
        if (ret == tl::Result::SUCCESS) {
            print_matrix("输出旋转矩阵 (3x3, 行主序, 无量纲)", rot_mat, 3, 3);
        }
    }

    // ============================================================
    // 2. 位姿矩阵 ↔ 旋转矩阵 — 正反验证
    // ============================================================
    // 单位说明:
    //   旋转矩阵 R (3×3)     — 无量纲
    //   平移向量 T [Tx,Ty,Tz] — 毫米 mm
    //   位姿矩阵 TR (4×4)  — R 部分无量纲，T 部分单位 mm
    // 位姿矩阵 (行主序 16 元素):
    //   [Rxx Rxy Rxz Tx]  第 0 行
    //   [Ryx Ryy Ryz Ty]  第 1 行
    //   [Rzx Rzy Rzz Tz]  第 2 行
    //   [  0   0   0  1]  第 3 行
    print_separator("位姿矩阵 ↔ 旋转矩阵");

    // 2a. TR → R → TR（正反验证）
    {
        std::cout << "--- 2a. TR ↔ R 正反验证 ---" << std::endl;
        // 构造一个绕 Z 转 90°、平移 (100, 200, 300) mm 的位姿矩阵
        double cos90 = cos(M_PI_2), sin90 = sin(M_PI_2);
        std::vector<double> tr_in = {
            cos90, -sin90, 0.0, 100.0,
            sin90,  cos90, 0.0, 200.0,
            0.0,    0.0,   1.0, 300.0,
            0.0,    0.0,   0.0, 1.0
        };
        std::vector<double> R, tr_out;

        ret = tl::get_tr2r(sock, tr_in, R);
        print_result("tl::get_tr2r(TR→R)", ret);
        if (ret != tl::Result::SUCCESS) return -1;
        print_matrix("输入位姿矩阵 (4x4, R无量纲, T:mm)", tr_in, 4, 4);
        print_matrix("中间旋转矩阵 (3x3, 无量纲)", R, 3, 3);

        ret = tl::get_r2tr(sock, R, tr_out);
        print_result("tl::get_r2tr(R→TR)", ret);
        if (ret != tl::Result::SUCCESS) return -1;
        print_matrix("还原位姿矩阵 (4x4, R无量纲, T:mm)", tr_out, 4, 4);

        // 说明: get_tr2r 只提取旋转矩阵，平移 T 在此丢弃；
        // get_r2tr 的输入不含平移，还原出的 TR 平移部分恒为 0。
        // SDK 无 "R+T 构造 TR" 接口，完整位姿矩阵需由调用方自行拼装 T。
        std::cout << "  注: get_tr2r 仅输出旋转矩阵，平移 T 不参与往返；" << std::endl;
        std::cout << "      还原位姿矩阵的平移部分为 0，需调用方另行拼装。" << std::endl;

        // 对比旋转部分（TR→R→TR 链路仅保证旋转信息无损）
        // 注意: 行主序 4x4 中旋转块位于 0,1,2, 4,5,6, 8,9,10，
        //       位置 3/7/11 是平移列，不能按连续 0..8 遍历。
        if (tr_out.size() >= 16 && tr_in.size() >= 16) {
            const int rot_idx[9] = {0, 1, 2, 4, 5, 6, 8, 9, 10};
            double r_err = 0.0;
            for (int k = 0; k < 9; ++k) {
                r_err += fabs(tr_out[rot_idx[k]] - tr_in[rot_idx[k]]);
            }
            std::cout << "  旋转矩阵误差 (应接近0): " << r_err << std::endl;
        }
    }

    // ============================================================
    // 3. 坐标系转换（获取机器人当前位姿在不同坐标系下的表示）
    // ============================================================
    // 单位说明:
    //   关节坐标 [J1..J6] — 度 °
    //   基坐标 [X,Y,Z,Rx,Ry,Rz] — 位移 mm，姿态 rad
    //   工具/用户坐标 — 同基坐标
    print_separator("坐标系转换");
    {
        std::cout << "--- 3. get_origin_coord_to_target_coord ---" << std::endl;
        std::cout << "  坐标系: 0=关节(°)  1=基坐标(mm+rad)  2=工具(mm+rad)  3=用户(mm+rad)" << std::endl;

        // 从关节坐标系(0) 转换到基坐标系(1)
        // 关节角度: J1=10°, J2=20°, J3=30°, J4=0°, J5=0°, J6=0°
        std::vector<double> joint_pos = {10, 20, 30, 0, 0, 0};
        std::vector<double> cartesian_pos;
        bool convert_state = false;

        ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::JOINT, joint_pos, tl::Coord::BASE, cartesian_pos, convert_state);
        print_result("tl::get_origin_coord_to_target_coord(关节→基坐标) 正解", ret);
        print_vector("输入关节角度 [°]", joint_pos);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  输出基坐标 — 前3位为位移[mm]，后3位为姿态[rad]:" << std::endl;
            print_vector("  基坐标", cartesian_pos);
            if (cartesian_pos.size() >= 6) {
                std::cout << "    位移 X=" << cartesian_pos[0] << "mm, Y=" << cartesian_pos[1]
                          << "mm, Z=" << cartesian_pos[2] << "mm" << std::endl;
                std::cout << "    姿态 Rx=" << cartesian_pos[3] << "rad, Ry=" << cartesian_pos[4]
                          << "rad, Rz=" << cartesian_pos[5] << "rad" << std::endl;
                std::cout << "    姿态换算(°): Rx=" << cartesian_pos[3] * 180.0 / M_PI
                          << ", Ry=" << cartesian_pos[4] * 180.0 / M_PI
                          << ", Rz=" << cartesian_pos[5] * 180.0 / M_PI << std::endl;
            }
        }

        // 逆解: 基坐标系(1) → 关节坐标系(0)
        // 使用正解输出的基坐标作为输入，验证正反解一致性
        if (ret == tl::Result::SUCCESS && cartesian_pos.size() >= 6) {
            std::vector<double> joint_back;
            ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::BASE, cartesian_pos, tl::Coord::JOINT, joint_back, convert_state);
            print_result("tl::get_origin_coord_to_target_coord(基坐标→关节) 逆解", ret);
            std::cout << "  输入基坐标:" << std::endl;
            std::cout << "    位移 [mm]: X=" << cartesian_pos[0] << ", Y=" << cartesian_pos[1]
                      << ", Z=" << cartesian_pos[2] << std::endl;
            std::cout << "    姿态 [rad]: Rx=" << cartesian_pos[3] << ", Ry=" << cartesian_pos[4]
                      << ", Rz=" << cartesian_pos[5] << std::endl;
            if (ret == tl::Result::SUCCESS) {
                print_vector("  逆解关节角度 [°]", joint_back);
                // 对比原始关节角度，计算误差
                double err = 0.0;
                for (size_t i = 0; i < joint_pos.size() && i < joint_back.size(); ++i) {
                    err += fabs(joint_pos[i] - joint_back[i]);
                }
                std::cout << "  正反解误差 (应接近0): " << err << std::endl;
            }
        }
    }

    // ============================================================
    // 4. 逆解关节坐标 → servoJ 下发运动
    // ============================================================
    // 演示流程: 给定基坐标 → 逆解到关节角 → 通过 servoJ 发送到控制器
    print_separator("逆解 + servoJ 运动");

    // 取一个基坐标点，逆解为关节角
    std::vector<double> target_pose = {300, 0, 100, 3.141, 0, 0}; // X=300mm, Y=0, Z=100mm
    std::vector<double> target_joints;
    bool convert_state2 = false;

    ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::BASE, target_pose, tl::Coord::JOINT, target_joints, convert_state2);
    print_result("逆解: 基坐标→关节", ret);
    if (ret == tl::Result::SUCCESS) {
        print_vector("目标基坐标 [mm, rad]", target_pose);
        print_vector("逆解关节角度 [°]", target_joints);

        // 通过 servoJ 发送关节角度到控制器
        // servoJ 需要连接 7000 端口，已在开头连接
        std::cout << "\n  --- 通过 servoJ 下发逆解结果 ---" << std::endl;

        // servoJ 约束参数（7 元素: 6 本体轴 + 1 外部轴）
        // 注意: SDK 底层协议要求 servoJ 相关向量固定为 7 元素
        std::vector<double> vmax = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
        std::vector<double> amax = {60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0};
        std::vector<double> jmax = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

        ret = tl::open_servoJ(sock_servo, vmax, amax, jmax);
        print_result("open_servoJ", ret);
        if (ret == tl::Result::SUCCESS) {
            // 逐步发送逆解出的关节角度，确保 servoJ 跟踪到位
            // 分 50 步从当前角度平滑过渡到目标角度
            // q 必须为 7 元素，与 open_servoJ 一致
            std::vector<double> current = {0, 0, 0, 0, 0, 0, 0};
            int steps = 50;
            for (int i = 0; i < steps; ++i) {
                std::vector<double> q(7);
                size_t n_axes = target_joints.size();
                if (n_axes > q.size()) n_axes = q.size();
                for (size_t j = 0; j < n_axes; ++j) {
                    q[j] = current[j] + (target_joints[j] - current[j]) * (i + 1) / steps;
                }
                // 多余元素由 q(7) 初始化保持 0，无需额外处理
                tl::set_servoJ_pos(sock_servo, q);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // servoJ 是实时跟踪模式，等待固定时间确保运动到位
            std::cout << "  [信息] 等待 servoJ 运动到位..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));

            ret = tl::close_servoJ(sock_servo);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            print_result("close_servoJ", ret);
        }
    }

    // ---- 断开连接 ----
    print_separator("断开连接");
    tl::set_current_mode(sock, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    tl::disconnect_robot(sock);
    tl::disconnect_robot(sock_servo);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "[信息] 已断开连接，机械臂切换到示教模式" << std::endl;

    std::cout << "\n[信息] 坐标变换示例程序运行完毕" << std::endl;
    return 0;
}
