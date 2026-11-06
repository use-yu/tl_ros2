/**
 * ex_admittance_control.cpp
 * @brief 导纳（柔顺）控制示例程序
 *
 * 展示 admittance_move_to_pose() 导纳运动接口的使用。
 * 该接口封装了完整的导纳伺服运动流程：
 *   连接 → 上电 → 配置传感器 → 配置导纳 → 测量偏置 → 导纳运动 → 断开
 *
 * 导纳运动接口 admittance_move_to_pose():
 *   - 接受目标位姿，立即将其设为最终目标
 *   - 伺服按配置速度（默认 30%）朝目标运动
 *   - 运动过程中，外力通过导纳实时修正目标位姿
 *   - 定时运行 duration_sec 秒后自动退出，不判断是否到位
 *   - 外力消失后，机器人返回并停在目标位姿附近
 *
 * @attention
 *   - 需要双端口连接: 6001（控制）+ 7000（伺服高频透传）
 *   - 需要六维力传感器正常连接并已配置通讯参数
 *   - 导纳控制器使用 M-B-K 弹簧-质量-阻尼模型
 *   - 运行前手动将机械臂移至预期接触位置附近
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 *   - 运行模式: 运行模式（透传模式，tl::set_current_mode(2)）
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_admittance_control
 */

#include "tl_api.h"
#include "tl_api.h"
#include "tl_admittance.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// ============================================================
// 【用户设定区】—— 修改此处以适配实际环境
// ============================================================

/// 控制器 IP 地址（可用命令行参数 -ip 覆盖，默认 192.168.1.13）
static std::string ROBOT_IP = "192.168.1.13";
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

/// @brief 将传感器的去皮数据填入六维力数组 [Fx,Fy,Fz,Mx,My,Mz]
static inline void fill_wrench_array(const tl::Sensor6DData& data, double wrench[6])
{
    wrench[0] = data.fxDataSubBase;   // Fx (N)
    wrench[1] = data.fyDataSubBase;   // Fy (N)
    wrench[2] = data.fzDataSubBase;   // Fz (N)
    wrench[3] = data.mxDataSubBase;   // Mx (N·m)
    wrench[4] = data.myDataSubBase;   // My (N·m)
    wrench[5] = data.mzDataSubBase;   // Mz (N·m)
}

/// @brief 打印六维力数组
static void print_wrench(const double w[6])
{
    std::cout << "  Fx=" << w[0] << "  Fy=" << w[1] << "  Fz=" << w[2]
              << "  Mx=" << w[3] << "  My=" << w[4] << "  Mz=" << w[5];
}
 

// ============================================================
// 导纳运动接口
// ============================================================

/**
 * @brief 导纳运动 — 朝目标位姿运动，过程中实时融合导纳柔顺控制
 *
 * 封装完整的导纳伺服运动流程（直接 servoj，无异步线程）:
 *   1. 开启伺服跟踪（open_servoJ，直接 7 元素绝对单位）
 *   2. 以给定频率循环运行指定时长：
 *      读力传感器 → 低通滤波 → 导纳步进 → 修正目标 → IK 转关节角 → set_servoJ_pos
 *   3. 关闭伺服跟踪（close_servoJ）
 *
 * 使用直接 servoj 替代原 servo_movej（异步背景线程 + 插值），消除抖动：
 *   - 每个控制周期直接调用 set_servoJ_pos，无后台线程延迟
 *   - Cartesian→joint 逆解在主循环同步完成
 *   - 导纳补偿修正立即下发至控制器
 *
 * 结束后直接关闭伺服跟踪，无需 tl::wait_servo()（无异步队列）。
 *
 * 坐标系约定:
 *   - 目标/输出位姿: robot frame [X,Y,Z,RX,RY,RZ]，线 mm，角 rad
 *   - 传感器力: six-axis [Fx,Fy,Fz,Mx,My,Mz]
 *   - 导纳增量（内部）: [ΔX,ΔY,ΔZ,ΔRX,ΔRY,ΔRZ]，线 m，角 rad
 *   - 传感器通讯参数 YDirection=-1 使传感器 Y 轴与末端坐标系对齐（参考 ex_massage.cpp）
 *   - 传感器各轴与末端坐标系一致，无需软件翻转传感器信号
 *
 * @param sock_6001     TCP socket (6001 端口)
 * @param sock_7000     UDP socket (7000 端口, servoJ 高频透传)
 * @param target_pose   目标笛卡尔位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)，至少 6 元素
 *                      机器人以 servoJ 速度直驱至此目标，不做轨迹插补
 * @param admittance    已配置并设置偏置的导纳控制器
 * @param duration_sec  导纳运动持续时间（秒），到达时长后自动退出
 * @param freq          控制循环频率（Hz），默认 100
 * @param filter_alpha  力低通滤波系数 [0,1]，默认 0.3（越小越平滑）
 * @param sv_pct        servoJ 速度百分比 (0-100)，默认 30
 */
tl::Result admittance_move_to_pose(
    tl::SOCKETFD sock_6001,
    tl::SOCKETFD sock_7000,
    const std::vector<double>& target_pose,
    tl::AdmittanceController& admittance,
    double duration_sec,
    double freq = 100.0,
    double filter_alpha = 0.3,
    double sv_pct = 30.0,
    double sa_pct = 80.0,
    double sj_pct = 20.0,
    double force_deadband = 2.0,
    double torque_deadband = 0.05,
    bool verbose = true)
{
    // ---- 参数校验 ----
    if (target_pose.size() < 6) {
        std::cerr << "[ERROR] target_pose 长度不足 6" << std::endl;
        return tl::Result::PARAM_ERR;
    }
    if (duration_sec <= 0.0) {
        std::cerr << "[ERROR] duration_sec 必须为正数" << std::endl;
        return tl::Result::PARAM_ERR;
    }

    const double dt = 1.0 / freq;
    if (filter_alpha < 0.0 || filter_alpha > 1.0) {
        std::cerr << "[ERROR] filter_alpha 应在 [0, 1] 范围内" << std::endl;
        return tl::Result::PARAM_ERR;
    }
    if (force_deadband < 0.0 || torque_deadband < 0.0) {
        std::cerr << "[ERROR] deadband 必须 >= 0" << std::endl;
        return tl::Result::PARAM_ERR;
    }
    if (verbose) {
        std::cout << "  [信息] 导纳运动时长: " << duration_sec << "s  "
                  << "伺服速度: " << sv_pct << "%  "
                  << "频率: " << static_cast<int>(freq) << "Hz  "
                  << "死区: " << force_deadband << "N / " << torque_deadband << "Nm"
                  << std::endl;
    }
    // ---- 1a. 百分比转绝对单位（与 tl_extension 内部常量一致）----
    constexpr double kMaxVel = 300.0;    // 最大关节速度 (deg/s), 对应 100%
    constexpr double kMaxAcc = 3000.0;   // 最大关节加速度 (deg/s²), 对应 100%
    constexpr double kMaxJerk = 50000.0; // 最大关节加加速度 (deg/s³), 对应 100%

    std::vector<double> vmax(7, sv_pct / 100.0 * kMaxVel);
    std::vector<double> amax(7, sa_pct / 100.0 * kMaxAcc);
    std::vector<double> jmax(7, sj_pct / 100.0 * kMaxJerk);
    tl::Result ret_open = tl::open_servoJ(sock_7000, vmax, amax, jmax);
    if (ret_open != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] open_servoJ 失败: " << ret_open << std::endl;
        return ret_open;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ---- 2. 初始化循环状态 ----
    const auto control_step = std::chrono::milliseconds(static_cast<int>(dt * 1000.0));
    auto next_time = std::chrono::steady_clock::now();
    const auto start_time = next_time;
    const auto end_time = start_time + std::chrono::milliseconds(static_cast<int>(duration_sec * 1000.0));

    tl::Sensor6DData sensor_data;
    double sensor_wrench[6] = {0.0};
    double filtered_wrench[6] = {0.0};
    bool filter_initialized = false;

    int sensor_fail_count = 0;
    int movej_fail_count = 0;
    tl::Result ret;

    if (verbose) {
        std::cout << "  [信息] 导纳运动开始，请对末端施加外力...\n" << std::endl;
    }

    // ---- 3. 主循环（定时运行） ----
    for (int cycle = 0; ; ++cycle) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= end_time) {
            break;
        }
        const auto cycle_start = now;

        // ---- 3a. 读取六维力传感器 ----
        ret = tl::get_sensor_6d_data(sock_6001, sensor_data);
        if (ret != tl::Result::SUCCESS || !sensor_data.sensorConnected) {
            ++sensor_fail_count;
            next_time += control_step;
            std::this_thread::sleep_until(next_time);
            continue;
        }
        fill_wrench_array(sensor_data, sensor_wrench);

        // 传感器已通过 YDirection=-1 与末端坐标系对齐，无需软件翻转
        // ---- 低通滤波 — EMA 一阶 ----
        if (!filter_initialized) {
            for (int i = 0; i < 6; ++i) filtered_wrench[i] = sensor_wrench[i];
            filter_initialized = true;
        } else {
            for (int i = 0; i < 6; ++i) {
                filtered_wrench[i] = filter_alpha * sensor_wrench[i]
                                   + (1.0 - filter_alpha) * filtered_wrench[i];
            }
        }
        // ---- 力/力矩死区: 低于阈值的噪声置零 ----
        for (int i = 0; i < 3; ++i) {
            if (std::abs(filtered_wrench[i]) < force_deadband) filtered_wrench[i] = 0.0;
        }
        for (int i = 3; i < 6; ++i) {
            if (std::abs(filtered_wrench[i]) < torque_deadband) filtered_wrench[i] = 0.0;
        }
        // ---- 3c. 导纳补偿: 使用库函数 admittance_compensate_pose ----
        const std::vector<double> wrench_vec(filtered_wrench, filtered_wrench + 6);
        const std::vector<double> target_vec(target_pose.begin(), target_pose.begin() + 6);
        std::vector<double> compensated(6, 0.0);
        std::vector<double> delta_log(6, 0.0);
        tl::Result adm_ret = admittance.CompensatePose(
            wrench_vec, target_vec, compensated, &delta_log);
        if (adm_ret != tl::Result::SUCCESS) {
            ++movej_fail_count;
            next_time += control_step;
            std::this_thread::sleep_until(next_time);
            continue;
        }

        // ---- 3d. Cartesian→joint 转换与 servoj 直接下发 ----
        std::vector<double> compensated_7 = compensated;
        compensated_7.resize(7, 0.0);
        std::vector<double> target_joints;
        bool dummy_convert = false;
        std::vector<double> ref_pos(7, 0.0);
        tl::Result ret_ik = tl::get_origin_coord_to_target_coord(
            sock_6001, tl::Coord::BASE, compensated_7,
            tl::Coord::JOINT, target_joints, dummy_convert, 0, ref_pos);
        if (ret_ik != tl::Result::SUCCESS || target_joints.size() < 6) {
            ++movej_fail_count;
            next_time += control_step;
            std::this_thread::sleep_until(next_time);
            continue;
        }

        // 构造 7 元素向量（兼容 6/7 轴）
        std::vector<double> q(7, 0.0);
        size_t nq = std::min(target_joints.size(), q.size());
        for (size_t j = 0; j < nq; ++j) {
            q[j] = target_joints[j];
        }

        ret = tl::set_servoJ_pos(sock_7000, q);
        if (ret != tl::Result::SUCCESS) {
            ++movej_fail_count;
        }

        // ---- 3e. 周期耗时 ----
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - cycle_start).count();

        // ---- 3f. 周期日志 ----
        if (verbose) {
            // 原始传感器值（滤波前、死区前，坐标系已通过 YDirection=-1 对齐）
            std::cout << "[#" << cycle << "]"
                      << "  F_raw=" << sensor_wrench[0] << "," << sensor_wrench[1]
                      << "," << sensor_wrench[2]
                      << "  M_raw=" << sensor_wrench[3] << "," << sensor_wrench[4]
                      << "," << sensor_wrench[5]
                      << "  F=" << filtered_wrench[0] << "," << filtered_wrench[1]
                      << "," << filtered_wrench[2]
                      << "  M=" << filtered_wrench[3] << "," << filtered_wrench[4]
                      << "," << filtered_wrench[5]
                      << "  ΔP=" << delta_log[0] << "," << delta_log[1]
                      << "," << delta_log[2] << "mm"
                      << "  ΔR=" << delta_log[3] << "," << delta_log[4] << "," << delta_log[5] << "rad"
                      << "  t=" << elapsed << "μs"
                      << std::endl;
        }

        next_time += control_step;
        std::this_thread::sleep_until(next_time);
    }

    // ---- 4. 关闭伺服跟踪 ----
    ret = tl::close_servoJ(sock_7000);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[WARN] close_servoJ 返回: " << ret << std::endl;
    }

    if (verbose) {
        std::cout << "\n  [信息] 导纳运动完成（运行 " << duration_sec << "s）" << std::endl;
        if (sensor_fail_count > 0) {
            std::cout << "  [WARN] 传感器读取失败次数: " << sensor_fail_count << std::endl;
        }
        if (movej_fail_count > 0) {
            std::cout << "  [WARN] set_servoJ_pos 下发失败次数: " << movej_fail_count << std::endl;
        }
    }

    return tl::Result::SUCCESS;
}


// ============================================================
// 主函数 — 演示 admittance_move_to_pose() 用法
// ============================================================

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // 解析命令行参数: ./ex_xxx [-ip 192.168.1.13]，默认 192.168.1.13
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ROBOT_IP = argv[i + 1];
            break;
        }
    }

    std::cout << std::fixed << std::setprecision(3);
    tl::set_log_level(tl::WARNING);
    std::cout << "=== 导纳运动接口示例 ===" << std::endl;

    // 获取 SDK 版本
    std::string version = tl::get_library_version();
    std::cout << "[信息] SDK 版本: " << version << std::endl;

    tl::Result ret;

    // ============================================================
    // 1. 双端口连接
    // ============================================================
    print_separator("双端口连接");
    tl::SOCKETFD sock_6001 = tl::connect_robot(ROBOT_IP, "6001");
    if (sock_6001 <= 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败" << std::endl;
        return -1;
    }
    tl::SOCKETFD sock_7000 = tl::connect_robot(ROBOT_IP, "7000");
    if (sock_7000 <= 0) {
        std::cerr << "[ERROR] 连接 7000 端口失败" << std::endl;
        tl::disconnect_robot(sock_6001);
        return -1;
    }
    std::cout << "[信息] 双端口连接成功 (6001=" << sock_6001
              << ", 7000=" << sock_7000 << ")" << std::endl;

    // ============================================================
    // 2. 清错与伺服复位
    // ============================================================
    print_separator("清错与伺服复位");
    ret = tl::clear_error(sock_6001);
    print_result("clear_error", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    ret = tl::set_servo_poweroff(sock_6001);
    if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
        std::cerr << "[WARN] 下电时状态异常: " << ret
                  << "（非严重错误，继续流程）" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ============================================================
    // 3. 伺服上电
    // ============================================================
    print_separator("伺服使能");
    ret = tl::set_servo_state(sock_6001, 1);
    print_result("tl::set_servo_state(1) 伺服就绪", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // 4. 伺服上电（示教模式下手动上电）
    // ============================================================
    print_separator("伺服上电");
    ret = tl::set_servo_poweron(sock_6001);
    print_result("set_servo_poweron 上电使能", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ============================================================
    // 5. 回零 — 关节空间运动至零位
    // ============================================================
    print_separator("关节回零");
    {
        ret = tl::robot_go_home(sock_6001);
        print_result("robot_go_home 回到零点", ret);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 回零失败，退出程序" << std::endl;
            tl::set_servo_poweroff(sock_6001);
            tl::disconnect_robot(sock_7000);
            tl::disconnect_robot(sock_6001);
            return -1;
        }
        // 等待运动到位（非阻塞接口需等待）
        std::this_thread::sleep_for(std::chrono::seconds(6));
    }

    // ============================================================
    // 切换为运行模式（透传模式）
    // ============================================================
    print_separator("切换运行模式");
    ret = tl::set_current_mode(sock_6001, 2);
    print_result("tl::set_current_mode(2) 运行模式", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ============================================================
    // 6. 读取初始位姿（用作示教目标的参照）
    // ============================================================
    print_separator("读取初始位姿");
    std::vector<double> init_pose(7, 0.0);
    ret = tl::get_current_position(sock_6001, tl::Coord::BASE, init_pose);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取初始笛卡尔位姿失败: " << ret << std::endl;
        tl::set_servo_poweroff(sock_6001);
        tl::disconnect_robot(sock_7000);
        tl::disconnect_robot(sock_6001);
        return -1;
    }
    std::cout << "  [信息] 初始位姿（基坐标）:" << std::endl;
    std::cout << "  [信息] X=" << init_pose[0] << "mm  Y=" << init_pose[1] << "mm"
              << "  Z=" << init_pose[2] << "mm" << std::endl;
    std::cout << "  [信息] RX=" << init_pose[3] << "rad  RY=" << init_pose[4] << "rad"
              << "  RZ=" << init_pose[5] << "rad" << std::endl;

    // ============================================================
    // 7. 配置六维力传感器通讯参数
    // ============================================================
    print_separator("六维力传感器配置");
    {
        tl::SixDimensionalForceCommunicationParams commParams;
        commParams.sensorCommunicationType = 0;      // 0: EtherCAT
        commParams.startupAutoConnectSensor = true;
        commParams.sensorDragEnable = true;
        commParams.etherCat_mapNum = 1;
        commParams.YDirection = 1;
        commParams.ZDirection = -1;

        ret = tl::set_six_dimensional_force_communication_params(sock_6001, commParams);
        print_result("set_six_dimensional_force_communication_params", ret);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "  [WARN] 请确认传感器配置是否正确" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // ============================================================
    // 8. 配置导纳控制器
    // ============================================================
    print_separator("导纳控制器参数配置");
    tl::AdmittanceController admittance;
    tl::AdmittanceParams params;

    params.mass[0] = 8.0;   params.mass[1] = 8.0;   params.mass[2] = 8.0;
    params.mass[3] = 0.5;   params.mass[4] = 0.5;   params.mass[5] = 0.5;

    params.damping[0] = 150.0; params.damping[1] = 150.0; params.damping[2] = 150.0;
    params.damping[3] = 2.0;   params.damping[4] = 2.0;   params.damping[5] = 2.0;

    params.stiffness[0] = 80.0; params.stiffness[1] = 80.0; params.stiffness[2] = 80.0;
    params.stiffness[3] = 3.0;    params.stiffness[4] = 3.0;    params.stiffness[5] = 3.0;

    for (int i = 0; i < 6; ++i) {
        params.max_delta[i] = (i < 3) ? 0.0005 : 0.005;
    }
    params.control_period = 1.0 / 100.0;  // 100Hz 导纳计算

    tl::Result adm_status = admittance.Configure(params);
    if (adm_status != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 导纳控制器配置失败" << std::endl;
        tl::set_servo_poweroff(sock_6001);
        tl::disconnect_robot(sock_7000);
        tl::disconnect_robot(sock_6001);
        return -1;
    }
    std::cout << "  [成功] 导纳控制器配置完成" << std::endl;
    std::cout << "  M: [" << params.mass[0] << ", " << params.mass[1] << ", "
              << params.mass[2] << ", " << params.mass[3] << ", "
              << params.mass[4] << ", " << params.mass[5] << "]" << std::endl;
    std::cout << "  B: [" << params.damping[0] << ", " << params.damping[1] << ", "
              << params.damping[2] << ", " << params.damping[3] << ", "
              << params.damping[4] << ", " << params.damping[5] << "]" << std::endl;
    std::cout << "  K: [" << params.stiffness[0] << ", " << params.stiffness[1] << ", "
              << params.stiffness[2] << ", " << params.stiffness[3] << ", "
              << params.stiffness[4] << ", " << params.stiffness[5] << "]" << std::endl;

    // ============================================================
    // 9. 自动测量传感器静态偏置（重力补偿）
    // ============================================================
    print_separator("传感器偏置测量（重力补偿）");
    {
        const int BIAS_SAMPLES = 50;
        double bias_sum[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        int valid_samples = 0;

        for (int i = 0; i < BIAS_SAMPLES; ++i) {
            tl::Sensor6DData tmp;
            ret = tl::get_sensor_6d_data(sock_6001, tmp);
            if (ret == tl::Result::SUCCESS && tmp.sensorConnected) {
                bias_sum[0] += tmp.fxDataSubBase;
                bias_sum[1] += tmp.fyDataSubBase;
                bias_sum[2] += tmp.fzDataSubBase;
                bias_sum[3] += tmp.mxDataSubBase;
                bias_sum[4] += tmp.myDataSubBase;
                bias_sum[5] += tmp.mzDataSubBase;
                ++valid_samples;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (valid_samples > 0) {
            double bias_wrench[6];
            for (int i = 0; i < 6; ++i) {
                bias_wrench[i] = bias_sum[i] / valid_samples;
            }

            // 打印未翻转的原始偏置值，用于诊断传感器是否真的读到数据
            std::cout << "  [信息] 原始偏置(未翻转): Fx=" << bias_wrench[0]
                      << "  Fy=" << bias_wrench[1] << "  Fz=" << bias_wrench[2]
                      << " N   Mx=" << bias_wrench[3] << "  My=" << bias_wrench[4]
                      << "  Mz=" << bias_wrench[5] << " Nm" << std::endl;

            // 检查偏置是否全接近零（传感器可能未正常工作）
            bool bias_all_zero = true;
            for (int i = 0; i < 6; ++i) {
                if (std::abs(bias_wrench[i]) > 1e-6) {
                    bias_all_zero = false;
                    break;
                }
            }

            if (bias_all_zero) {
                std::cerr << "  [WARN] 偏置全部接近零，传感器可能未连接或通讯异常，跳过重力补偿" << std::endl;
            } else {
                // 传感器已通过 YDirection=-1 与末端坐标系对齐，无需翻转
                // 直接使用原始偏置作为 F_desired（参考 ex_massage.cpp）
                admittance.SetDesiredWrench(std::vector<double>(bias_wrench, bias_wrench + 6));

                std::cout << "  [成功] 重力补偿完成（" << valid_samples << " 次采样平均）" << std::endl;
                std::cout << "  [信息] 补偿后偏置力: Fx=" << bias_wrench[0]
                          << "  Fy=" << bias_wrench[1] << "  Fz=" << bias_wrench[2]
                          << " N" << std::endl;
                std::cout << "  [信息] 补偿后偏置矩: Mx=" << bias_wrench[3]
                          << "  My=" << bias_wrench[4] << "  Mz=" << bias_wrench[5]
                          << " N·m" << std::endl;
            }
        } else {
            std::cerr << "  [WARN] 传感器偏置测量失败（valid_samples=0），未启用重力补偿" << std::endl;
        }
    }

    // ============================================================
    // 10. 导纳运动 — 使用 admittance_move_to_pose() 接口
    // ============================================================
    print_separator("导纳运动 — 朝目标位姿运动");
    {
        // 重置导纳内部状态（清除上次运动的累积位移）
        admittance.Reset();

        // 构造目标位姿: 初始位姿沿 Z 方向移动 -100mm
        std::vector<double> target = {
            init_pose[0],
            init_pose[1] + 200.0,
            init_pose[2],
            init_pose[3],
            init_pose[4],
            init_pose[5]
        };
        ret = admittance_move_to_pose(
            sock_6001, sock_7000, target, admittance,
            20.0,     // duration_sec: 导纳运动持续时间
            100.0,    // freq
            0.15,     // filter_alpha: 更低滤波系数，力信号更平滑
            5.0,     // sv_pct: 5% 低速，防止一碰就冲出去
            40.0,     // sa_pct: 40% 加速度
            10.0,     // sj_pct: 10% 加加速度
            0.5,      // force_deadband: 0.5N 死区（传感器读数约3N，5N会滤掉真实力）
            0.02,     // torque_deadband: 0.02Nm 死区（传感器读数约0.04Nm，0.15会滤掉真实力矩）
            true      // verbose
        );

        if (ret == tl::Result::SUCCESS) {
            print_result("导纳运动完成", ret);
        } else {
            print_result("导纳运动异常结束", ret);
        }
    }

    // ============================================================
    // 11. 关闭与断开
    // ============================================================
    print_separator("关闭与断开");
    ret = tl::set_current_mode(sock_6001, 0);
    print_result("tl::set_current_mode(0) 切回示教模式（自动下电）", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    tl::disconnect_robot(sock_7000);
    tl::disconnect_robot(sock_6001);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 导纳控制示例程序运行完毕" << std::endl;
    std::cout << "  [提示] 调柔顺: ↑B / ↑M / ↓K / ↓伺服速度 / ↑死区 / ↓max_delta" << std::endl;
    std::cout << "  [提示] 调灵敏: ↓B / ↓M / ↑K / ↑伺服速度 / ↓死区 / ↑max_delta" << std::endl;
    std::cout << "  [提示] K 太小会导致回弹力被死区吃掉，残余位移 = deadband/K" << std::endl;
    std::cout << "  [提示] filter_alpha 控制力信号平滑度(越低越平滑)，deadband 屏蔽小力噪声" << std::endl;
    return 0;
}
