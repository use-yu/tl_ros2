/**
 * ex_massage.cpp
 * @brief 恒力按摩控制示例（直接力控）
 *
 * 使用导纳控制实现恒力按摩：设置期望接触力 F_desired，
 * 导纳控制器自动调节机器人位置使接触力稳定在目标值。
 * 接触力 > 目标 → 退让；接触力 < 目标 → 跟进。
 *
 * 按摩流程：
 *   回零 → 读取当前位置 → 恒力接触 + 直线扫动 → 抬起
 *
 * 力控原理：
 *   - F_desired = bias（重力补偿）+ target_force（目标力，如 Z 方向 10N）
 *   - 目标位姿停靠在接触面上（不做压入偏移）
 *   - 导纳控制器通过 F_error = F_sensor - F_desired 自主调节位置
 *   - 力控轴刚度 K=0，实现零稳态误差力跟踪
 *   - 表面凸起 → 力增大 → F_error > 0 → 退让
 *   - 表面凹陷 → 力减小 → F_error < 0 → 跟进
 *
 * @attention
 *   - 需要六维力传感器正常连接
 *   - 运行前确保按摩头/工具已安装，末端朝下
 *   - 首次运行时建议不接触表面，观察轨迹是否正常
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 * @note 运行步骤
 *       编译: cd examples/cpp && cmake --preset linux-native && cmake --build
 *       运行: ./build/linux-native/ex_massage
 */

#define _USE_MATH_DEFINES

#include "tl_api.h"
#include "tl_api.h"
#include "tl_admittance.h"

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// ============================================================
// 【用户设定区】
// ============================================================

/// 控制器 IP 地址（可用命令行参数 -ip 覆盖，默认 192.168.1.13）
static std::string ROBOT_IP = "192.168.1.13";

// ============================================================
// 【按摩参数设定区】
// ============================================================

/// 恒力控制方向 (0=X, 1=Y, 2=Z)
static constexpr int DESIRED_FORCE_AXIS = 2;

/// 目标按压力 (N)：导纳控制器将调节位置使接触力稳定在此值附近
/// 导纳内部 K=0，稳态误差趋近于 0
/// 建议从 5~10N 开始调，逐步增加
static constexpr double DESIRED_FORCE = -10.0;

/// 扫动方向（末端坐标系，0=X, 1=Y）
static constexpr int SWEEP_AXIS = 1;

/// 扫动参数
static constexpr double SWEEP_AMPLITUDE = 40.0;    // 扫动幅度 (mm)
static constexpr double SWEEP_PERIOD = 5.0;         // 单程周期 (s)
static constexpr double MASSAGE_DURATION = 50.0;    // 总时长 (s)

/// 伺服参数（百分比）
static constexpr double SERVO_SPEED = 10.0;
static constexpr double SERVO_ACCEL = 30.0;
static constexpr double SERVO_JERK = 10.0;

// ============================================================
// RAII 清理守卫 — 确保所有路径上都断开连接
// ============================================================

struct SocketGuard {
    tl::SOCKETFD sock_6001{-1};
    tl::SOCKETFD sock_7000{-1};

    ~SocketGuard() {
        if (sock_7000 > 0) { tl::disconnect_robot(sock_7000); }
        if (sock_6001 > 0) {
            tl::set_servo_poweroff(sock_6001);
            tl::disconnect_robot(sock_6001);
        }
    }
};

// ============================================================
// 按摩伺服循环参数
// ============================================================

struct MassageParams {
    int sweep_axis{SWEEP_AXIS};
    double sweep_amplitude{SWEEP_AMPLITUDE};
    double sweep_period{SWEEP_PERIOD};
    double duration_sec{MASSAGE_DURATION};

    double freq{100.0};
    // 每维独立的 EMA 滤波系数（[0..2]=Fx,Fy,Fz 力  [3..5]=Mx,My,Mz 力矩）
    std::array<double, 6> filter_alpha{{0.15, 0.15, 0.3, 0.15, 0.15, 0.15}};
    double sv_pct{SERVO_SPEED};
    double sa_pct{SERVO_ACCEL};
    double sj_pct{SERVO_JERK};
    // 每维独立的死区（[0..2]=力 N  [3..5]=力矩 N·m），小于该值的分量置零
    std::array<double, 6> deadband{{0.5, 0.5, 0.1, 0.02, 0.02, 0.12}};
    bool verbose{true};
};

// ============================================================
// 工具函数
// ============================================================

static void print_result(const std::string& msg, int ret)
{
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [成功] " << msg << '\n';
    } else {
        std::cout << "  [失败] " << msg << "，错误码: " << ret << '\n';
    }
}

static void print_separator(const std::string& title)
{
    std::cout << "\n========== " << title << " ==========\n";
}

/// @brief 轮询等待运动完成，替代固定延时
/// @param sock TCP socket
/// @param timeout_sec 超时秒数
/// @return true 运动完成 / false 超时
static bool wait_motion_done(tl::SOCKETFD sock, int timeout_sec = 30)
{
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::seconds(timeout_sec);
    // 初始延时：等控制器接收指令并置 state=2 后再轮询
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int last_print = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        int state = -1;
        const tl::Result ret = tl::get_robot_running_state(sock, state);
        if (ret == tl::Result::SUCCESS && state == 0) {
            std::cout << "  [信息] 运动完成\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const int elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count());
        if (elapsed - last_print >= 5) {
            std::cout << "  [信息] 等待运动中... (已过 " << elapsed << "s)\n";
            last_print = elapsed;
        }
    }
    std::cerr << "  [WARN] 等待运动超时 (" << timeout_sec << "s)\n";
    return false;
}

/// @brief 将传感器的去皮数据填入六维力数组
static inline void fill_wrench_array(const tl::Sensor6DData& data, double wrench[6]) noexcept
{
    wrench[0] = data.fxDataSubBase;
    wrench[1] = data.fyDataSubBase;
    wrench[2] = data.fzDataSubBase;
    wrench[3] = data.mxDataSubBase;
    wrench[4] = data.myDataSubBase;
    wrench[5] = data.mzDataSubBase;
}

/**
 * @brief 将末端坐标系下的位移向量旋转到基坐标系
 * @param tool_vec  末端坐标系下的 3D 位移 (mm)
 * @param ori       末端姿态 XYZ 固定角 [rx, ry, rz] (rad)
 * @return 基坐标系下的 3D 位移 (mm)
 *
 * R = Rz(rz) * Ry(ry) * Rx(rx)  (XYZ 固定角)
 */
static std::array<double, 3> rotate_tool_to_base(
    const std::array<double, 3>& tool_vec,
    const std::array<double, 3>& ori) noexcept
{
    const double cx = std::cos(ori[0]), sx = std::sin(ori[0]);
    const double cy = std::cos(ori[1]), sy = std::sin(ori[1]);
    const double cz = std::cos(ori[2]), sz = std::sin(ori[2]);

    return {{
        (cy*cz) * tool_vec[0]
            + (cz*sx*sy - cx*sz) * tool_vec[1]
            + (sx*sz + cx*cz*sy) * tool_vec[2],
        (cy*sz) * tool_vec[0]
            + (cx*cz + sx*sy*sz) * tool_vec[1]
            + (cx*sy*sz - cz*sx) * tool_vec[2],
        (-sy) * tool_vec[0]
            + (cy*sx) * tool_vec[1]
            + (cx*cy) * tool_vec[2]
    }};
}

// ============================================================
// 按摩导纳伺服循环
// ============================================================

/**
 * @brief 按摩导纳伺服循环（恒力模式）
 *
 * 原理：F_desired = bias + DESIRED_FORCE（重力补偿 + 目标力）。
 * 目标位姿在末端坐标系下沿 Y 轴做直线扫动，
 * 经工具姿态旋转矩阵转换到基坐标系后叠加。
 * 目标位姿停留在接触面上，不做压入偏移。
 * 接触力由 F_desired 中的目标力值控制——导纳通过 F_error
 * 自主调节位置：力大退让，力小跟进。
 *
 * @param sock_6001     TCP socket
 * @param sock_7000     UDP socket
 * @param base_pose     起始位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)
 * @param admittance    tl::AdmittanceController 实例（F_desired 已设为目标力）
 * @param params        按摩参数
 */
static tl::Result massage_servo_loop(
    tl::SOCKETFD sock_6001,
    tl::SOCKETFD sock_7000,
    const std::vector<double>& base_pose,
    tl::AdmittanceController& admittance,
    const MassageParams& params)
{
    constexpr double kMaxVel = 300.0;
    constexpr double kMaxAcc = 3000.0;
    constexpr double kMaxJerk = 50000.0;

    const double dt = 1.0 / params.freq;
    // 每维独立的 EMA 系数（与结构体 filter_alpha 逐维对应）
    const std::array<double, 6> filter_alpha = params.filter_alpha;
    std::array<double, 6> filter_one_minus_alpha{};
    for (int i = 0; i < 6; ++i) filter_one_minus_alpha[i] = 1.0 - filter_alpha[i];
    // 扫动角频率
    const double angular_freq = 2.0 * M_PI / params.sweep_period;
    const double vel_scale = params.sv_pct / 100.0;
    const double acc_scale = params.sa_pct / 100.0;
    const double jerk_scale = params.sj_pct / 100.0;

    const std::vector<double> vmax(7, vel_scale * kMaxVel);
    const std::vector<double> amax(7, acc_scale * kMaxAcc);
    const std::vector<double> jmax(7, jerk_scale * kMaxJerk);

    // ---- 1. 开启伺服跟踪 ----
    const tl::Result ret_open = tl::open_servoJ(sock_7000, vmax, amax, jmax);
    if (ret_open != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] open_servoJ 失败: " << ret_open << '\n';
        return ret_open;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    if (params.verbose) {
        std::cout << "  [信息] 恒力按摩伺服开始，时长: " << params.duration_sec << "s\n";
        std::cout << "  [信息] 扫动幅度: " << params.sweep_amplitude << "mm (直线往复)"
                  << "  周期: " << params.sweep_period << "s\n";
        std::cout << "  [信息] 恒力控制模式：F_desired 包含目标力，导纳自动调节位置\n";
        std::cout << "  [信息] 调大力: ↑DESIRED_FORCE | 调小力: ↓DESIRED_FORCE\n";
    }

    // ---- 2. 主循环 ----
    const auto start_time = std::chrono::steady_clock::now();
    const auto end_time = start_time + std::chrono::milliseconds(
        static_cast<int>(params.duration_sec * 1000.0));
    const auto control_step = std::chrono::microseconds(static_cast<int>(dt * 1'000'000.0));
    auto next_time = start_time;
    int cycle = 0;

    tl::Sensor6DData sensor_data;
    double sensor_wrench[6]{};
    double filtered_wrench[6]{};
    bool filter_initialized = false;

    // 循环内复用 vector，避免每周期堆分配
    std::vector<double> target_tool(6, 0.0);         // 末端系下目标位姿（扫动）
    std::vector<double> wrench_vec(6);                // 滤波后六维力，喂给导纳
    std::vector<double> compensated_tool(6);          // 导纳补偿后位姿（末端系）
    std::vector<double> delta_log_vec(6);             // 导纳本轮增量（日志用）
    std::vector<double> compensated(6);               // 基坐标系下最终位姿
    std::vector<double> compensated_7(7);              // 补成 7 元素送 IK
    std::vector<double> ref_pos(7, 0.0);              // IK 参考关节位置
    std::vector<double> q(7, 0.0);                    // 关节角度指令
    std::vector<double> target_joints;                 // IK 输出关节角

    for (; ; ++cycle) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= end_time) break;
        const auto cycle_start = now;
        const double elapsed_s = std::chrono::duration<double>(now - start_time).count();

        // ---- 2a. 目标位姿（末端坐标系下沿扫动轴做正弦往复）----
        // target_tool 其余元素保持 0（vector 复用，仅扫动轴被写入）
        const double sweep_offset = params.sweep_amplitude * std::sin(angular_freq * elapsed_s);
        target_tool[params.sweep_axis] = sweep_offset;

        // ---- 2b. 读取传感器 ----
        const tl::Result ret = tl::get_sensor_6d_data(sock_6001, sensor_data);
        if (ret != tl::Result::SUCCESS || !sensor_data.sensorConnected) {
            next_time += control_step;
            std::this_thread::sleep_until(next_time);
            continue;
        }
        fill_wrench_array(sensor_data, sensor_wrench);

        // ---- 2c. EMA 低通滤波（每维独立系数）----
        if (!filter_initialized) {
            for (int i = 0; i < 6; ++i) filtered_wrench[i] = sensor_wrench[i];
            filter_initialized = true;
        } else {
            for (int i = 0; i < 6; ++i) {
                filtered_wrench[i] = filter_alpha[i] * sensor_wrench[i]
                                   + filter_one_minus_alpha[i] * filtered_wrench[i];
            }
        }

        // ---- 2d. 小死区（每维独立阈值）----
        for (int i = 0; i < 6; ++i) {
            if (std::abs(filtered_wrench[i]) < params.deadband[i]) filtered_wrench[i] = 0.0;
        }

        // ---- 2e. 导纳补偿（末端坐标系下）----
        std::copy(filtered_wrench, filtered_wrench + 6, wrench_vec.begin());
        const tl::Result adm_ret = admittance.CompensatePose(
            wrench_vec, target_tool, compensated_tool, &delta_log_vec);
        if (adm_ret != tl::Result::SUCCESS) {
            next_time += control_step;
            std::this_thread::sleep_until(next_time);
            continue;
        }

        // ---- 补偿后位姿：工具系 -> 基坐标系 ----
        const std::array<double, 3> tool_pos = {{
            compensated_tool[0], compensated_tool[1], compensated_tool[2]
        }};
        const std::array<double, 3> current_ori = {{
            base_pose[3] + compensated_tool[3],
            base_pose[4] + compensated_tool[4],
            base_pose[5] + compensated_tool[5]
        }};
        const auto comp_base_off = rotate_tool_to_base(tool_pos, current_ori);

        compensated[0] = base_pose[0] + comp_base_off[0];
        compensated[1] = base_pose[1] + comp_base_off[1];
        compensated[2] = base_pose[2] + comp_base_off[2];
        compensated[3] = current_ori[0];
        compensated[4] = current_ori[1];
        compensated[5] = current_ori[2];

        // ---- 2f. IK 转换 + servoj ----
        compensated_7.resize(7);
        std::copy(compensated.begin(), compensated.begin() + 6, compensated_7.begin());
        compensated_7[6] = 0.0;

        bool dummy_convert = false;
        const tl::Result ret_ik = tl::get_origin_coord_to_target_coord(
            sock_6001, tl::Coord::BASE, compensated_7,
            tl::Coord::JOINT, target_joints, dummy_convert, 0, ref_pos);
        if (ret_ik != tl::Result::SUCCESS || target_joints.size() < 6) {
            next_time += control_step;
            std::this_thread::sleep_until(next_time);
            continue;
        }

        std::copy(target_joints.begin(), target_joints.begin() + 6, q.begin());
        q[6] = 0.0;

        tl::set_servoJ_pos(sock_7000, q);

        // ---- 2g. 周期日志 ----
        if (params.verbose && (cycle % 10 == 0)) {
            const auto log_time = std::chrono::steady_clock::now();
            const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                log_time - cycle_start).count();
            const long long elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                log_time - start_time).count();

            std::cout << "[t=" << elapsed_ms << "ms]"
                      << "  raw_F=" << std::setprecision(2)
                      << sensor_wrench[0] << ',' << sensor_wrench[1] << ',' << sensor_wrench[2]
                      << "N  "
                      << sensor_wrench[3] << ',' << sensor_wrench[4] << ',' << sensor_wrench[5]
                      << "Nm"
                      << "  ftr_F="
                      << filtered_wrench[0] << ',' << filtered_wrench[1] << ',' << filtered_wrench[2]
                      << "N  "
                      << filtered_wrench[3] << ',' << filtered_wrench[4] << ',' << filtered_wrench[5]
                      << "Nm"
                      << "  adm_d=" << std::setprecision(3)
                      << delta_log_vec[0] << ',' << delta_log_vec[1] << ',' << delta_log_vec[2]
                      << "mm  "
                      << delta_log_vec[3] << ',' << delta_log_vec[4] << ',' << delta_log_vec[5]
                      << "rad"
                      << "  cyc=" << elapsed_us << "us\n";
        }

        next_time += control_step;
        std::this_thread::sleep_until(next_time);
    }

    // ---- 3. 关闭伺服 ----
    const tl::Result close_ret = tl::close_servoJ(sock_7000);
    if (close_ret != tl::Result::SUCCESS) {
        std::cerr << "[WARN] close_servoJ 返回: " << close_ret << '\n';
    }

    if (params.verbose) {
        std::cout << "  [信息] 按摩伺服完成（运行 " << params.duration_sec << "s）\n";
    }

    return tl::Result::SUCCESS;
}

// ============================================================
// 主函数
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
    std::cout << "=== 力位混合按摩控制示例 ===" << '\n';

    const std::string version = tl::get_library_version();
    std::cout << "[信息] SDK 版本: " << version << '\n';

    // ============================================================
    // 1. 双端口连接
    // ============================================================
    print_separator("双端口连接");
    SocketGuard guard;
    guard.sock_6001 = tl::connect_robot(ROBOT_IP, "6001");
    if (guard.sock_6001 <= 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败\n";
        return -1;
    }
    guard.sock_7000 = tl::connect_robot(ROBOT_IP, "7000");
    if (guard.sock_7000 <= 0) {
        std::cerr << "[ERROR] 连接 7000 端口失败\n";
        return -1;
    }
    std::cout << "[信息] 双端口连接成功 (6001=" << guard.sock_6001
              << ", 7000=" << guard.sock_7000 << ")\n";

    const tl::SOCKETFD sock_6001 = guard.sock_6001;
    const tl::SOCKETFD sock_7000 = guard.sock_7000;
    tl::Result ret;

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
                  << "（非严重错误，继续流程）\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ============================================================
    // 3. 伺服上电
    // ============================================================
    print_separator("伺服使能");
    ret = tl::set_servo_state(sock_6001, 1);
    print_result("tl::set_servo_state(1) 伺服就绪", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    print_separator("伺服上电");
    ret = tl::set_servo_poweron(sock_6001);
    print_result("set_servo_poweron 上电使能", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ============================================================
    // 4. 回零
    // ============================================================
    print_separator("关节回零");
    ret = tl::robot_go_home(sock_6001);
    print_result("robot_go_home 回到零点", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 回零失败\n";
        return -1;  // SocketGuard 自动清理
    }
    if (!wait_motion_done(sock_6001, 30)) {
        std::cerr << "  [WARN] 回零超时，继续后续流程\n";
    }

    // ============================================================
    // 5. 读取初始位姿
    // ============================================================
    print_separator("读取初始位姿");
    std::vector<double> init_pose(7, 0.0);
    ret = tl::get_current_position(sock_6001, tl::Coord::BASE, init_pose);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取初始笛卡尔位姿失败\n";
        return -1;
    }
    std::cout << "  [信息] 初始位姿（基坐标）:\n";
    std::cout << "  [信息] X=" << init_pose[0] << "mm  Y=" << init_pose[1] << "mm"
              << "  Z=" << init_pose[2] << "mm\n";

    // ============================================================
    // 6. 切换为运行模式（透传模式）
    // ============================================================
    print_separator("切换运行模式");
    ret = tl::set_current_mode(sock_6001, 2);
    print_result("tl::set_current_mode(2) 运行模式", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ============================================================
    // 7. 配置六维力传感器
    // ============================================================
    print_separator("六维力传感器配置");
    {
        tl::SixDimensionalForceCommunicationParams commParams;
        commParams.sensorCommunicationType = 0;
        commParams.startupAutoConnectSensor = true;
        commParams.sensorDragEnable = true;
        commParams.etherCat_mapNum = 1;
        commParams.YDirection = -1; // 与末端坐标系对齐
        commParams.ZDirection = 1;

        ret = tl::set_six_dimensional_force_communication_params(sock_6001, commParams);
        print_result("set_six_dimensional_force_communication_params", ret);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "  [WARN] 请确认传感器配置是否正确\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // ============================================================
    // 8. 配置导纳控制器
    // ============================================================
    print_separator("导纳控制器参数配置");
    tl::AdmittanceController admittance;
    tl::AdmittanceParams adm_params;

    // 线轴 SI 单位: N/m 和 N/(m/s)；角轴: N·m/rad
    // @note 扫动平面 K=5000 N/m（5 N/mm）：压入 5mm 产生 ~25N 接触力（视表面硬度），
    //        保留柔顺退让性；力控轴 K=0 实现零稳态误差力跟踪
    adm_params.mass[0] = 80.0;
    adm_params.mass[1] = 80.0;
    adm_params.mass[2] = 160.0;
    adm_params.mass[3] = 20.0;
    adm_params.mass[4] = 20.0;
    adm_params.mass[5] = 20.0;

    adm_params.damping[0] = 1200.0;
    adm_params.damping[1] = 1200.0;
    adm_params.damping[2] = 10000.0;
    adm_params.damping[3] = 200.0;
    adm_params.damping[4] = 200.0;
    adm_params.damping[5] = 200.0;

    adm_params.stiffness[0] = 5000.0;
    adm_params.stiffness[1] = 5000.0;
    adm_params.stiffness[2] = 0.0;      // 力控轴 K=0，消除稳态误差
    adm_params.stiffness[3] = 50;
    adm_params.stiffness[4] = 50;
    adm_params.stiffness[5] = 50;

    adm_params.max_delta[0] = 0.001;
    adm_params.max_delta[1] = 0.001;
    adm_params.max_delta[2] = 0.0005;
    adm_params.max_delta[3] = 0.005;
    adm_params.max_delta[4] = 0.005;
    adm_params.max_delta[5] = 0.005;

    adm_params.control_period = 0.01;   // 100Hz

    const auto adm_status = admittance.Configure(adm_params);
    if (adm_status != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 导纳控制器配置失败\n";
        return -1;
    }
    std::cout << "  [成功] 导纳控制器配置完成\n";

    // ============================================================
    // 9. 自动测量传感器静态偏置（重力补偿）+ 设置 F_desired
    // ============================================================
    print_separator("传感器偏置测量（重力补偿）");
    {
        constexpr int BIAS_SAMPLES = 50;
        double bias_sum[6]{};
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
            double bias_raw[6];
            for (int i = 0; i < 6; ++i) {
                bias_raw[i] = bias_sum[i] / valid_samples;
            }
            std::cout << "  [信息] 原始偏置: Fx=" << bias_raw[0]
                      << "  Fy=" << bias_raw[1] << "  Fz=" << bias_raw[2]
                      << " N   Mx=" << bias_raw[3] << "  My=" << bias_raw[4]
                      << "  Mz=" << bias_raw[5] << " Nm\n";

            // 力控模式：F_desired = bias + DESIRED_FORCE（重力补偿 + 目标力）
            double desired_wrench[6];
            for (int i = 0; i < 6; ++i) desired_wrench[i] = bias_raw[i];
            desired_wrench[DESIRED_FORCE_AXIS] += DESIRED_FORCE;  // 叠加目标按压力
            admittance.SetDesiredWrench(std::vector<double>(desired_wrench, desired_wrench + 6));

            std::cout << "  [成功] 重力补偿 + 恒力设置完成（" << valid_samples << " 次采样平均）\n";
            std::cout << "  [信息] F_desired = bias + target_force = Fx=" << desired_wrench[0]
                      << "  Fy=" << desired_wrench[1] << "  Fz=" << desired_wrench[2]
                      << " N   Mx=" << desired_wrench[3] << "  My=" << desired_wrench[4]
                      << "  Mz=" << desired_wrench[5] << " Nm\n";
            std::cout << "  [信息] 恒力模式：目标 " << DESIRED_FORCE << "N，导纳自动调节位置\n";
        } else {
            std::cerr << "  [WARN] 传感器偏置测量失败，未启用重力补偿\n";
        }
    }

    // ============================================================
    // 10. 运动到按摩起始位姿
    // ============================================================
    print_separator("运动到按摩起始位姿");
    {
        tl::MoveCmd move_cmd;
        move_cmd.targetPosType = tl::PosType::data;
        move_cmd.targetPosName = "";
        move_cmd.coord = tl::Coord::BASE;  // 基坐标
        move_cmd.velocity = 80;
        move_cmd.acc = 10;
        move_cmd.dec = 10;
        move_cmd.pl = 0;
        // [X, Y, Z, RX, RY, RZ]  (mm, rad)
        move_cmd.targetPosValue = std::vector<double>{
            257.265, -0.020, -80.0,
            -3.141, 0.003, 0.0};
        const tl::Result ret_move = tl::robot_movel(sock_6001, move_cmd);
        print_result("robot_movel 运动到按摩起始位姿", ret_move);
        if (ret_move != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 移动到按摩起始位姿失败\n";
            return -1;
        }
        if (!wait_motion_done(sock_6001, 30)) {
            std::cerr << "  [WARN] 运动到按摩起始位姿超时，继续后续流程\n";
        }
    }

    // ============================================================
    // 11. 按摩流程
    // ============================================================
    print_separator("按摩流程");
    std::cout << "  [信息] 开始恒力按摩\n";
    std::cout << "  [信息] 力控轴: " << DESIRED_FORCE_AXIS
              << "  目标力: " << DESIRED_FORCE << "N\n";
    std::cout << "  [信息] 扫动轴: " << SWEEP_AXIS
              << "  幅度: " << SWEEP_AMPLITUDE << "mm\n";
    std::cout << "  [信息] 轨迹: 直线往复（仅扫动轴）\n";
    std::cout << "  [信息] 按摩时长: " << MASSAGE_DURATION << "s\n";

    {
        // 运动到位后读取实际位姿作为按摩基准
        std::vector<double> base_pose(7, 0.0);
        ret = tl::get_current_position(sock_6001, tl::Coord::BASE, base_pose);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取按摩起始位姿失败\n";
            return -1;
        }
        std::cout << "  [信息] 按摩基准位姿: X=" << base_pose[0]
                  << "mm  Y=" << base_pose[1] << "mm  Z=" << base_pose[2] << "mm\n";

        MassageParams params;
        admittance.Reset();

        ret = massage_servo_loop(
            sock_6001, sock_7000,
            base_pose, admittance,
            params);

        print_result(ret == tl::Result::SUCCESS ? "按摩伺服完成" : "按摩伺服异常结束", ret);
    }

    // ============================================================
    // 12. 关闭与断开
    // ============================================================
    print_separator("关闭与断开");
    ret = tl::set_current_mode(sock_6001, 0);
    print_result("tl::set_current_mode(0) 切回示教模式（自动下电）", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // SocketGuard 析构自动断开并下电
    std::cout << "[信息] 已断开连接\n";

    std::cout << "\n[信息] 恒力按摩示例运行完毕\n";
    std::cout << "  [提示] ↑DESIRED_FORCE = 力更大  |  ↓DESIRED_FORCE = 力更小\n";
    std::cout << "  [提示] ↑SWEEP_AMPLITUDE = 扫动范围更大 | ↓SWEEP_PERIOD = 扫动更快\n";
    std::cout << "  [提示] 修改 DESIRED_FORCE_AXIS/SWEEP_AXIS 调整力控方向/扫动方向\n";
    return 0;
}
