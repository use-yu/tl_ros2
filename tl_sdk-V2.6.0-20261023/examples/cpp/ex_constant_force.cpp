/**
 * ex_constant_force.cpp
 * @brief 恒力驻留控制示例（无扫动）
 *
 * 业务逻辑与 ex_massage.cpp 对齐，恒力控制使用 tl_extension 的
 * open_constforce/close_constforce 封装接口（后台线程运行闭环）：
 * 目标位姿固定于基准位姿，导纳控制器通过力误差自动调节位置，
 * 使接触力稳定在 F_desired（重力补偿 + 目标力）。
 *
 * 流程：
 *   连接 → 清错 → 上电 → 回零 → 读位姿 → 切运行模式 → 配置传感器/导纳
 *   → 测量偏置（重力补偿）→ movel 到接触位 → 恒力驻留 N 秒 → 关闭
 *
 * @attention 需要六维力传感器正常连接
 * @note 运行步骤
 *       编译: cd examples/cpp && cmake --preset linux-native && cmake --build
 *       运行: ./build/linux-native/ex_constant_force
 */

#include "tl_api.h"
#include "tl_api.h"
#include "tl_admittance.h"
#include "tl_constant_force.h"

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

// ============================================================
// 【用户设定区】
// ============================================================

/// 控制器 IP 地址（可用命令行参数 -ip 覆盖，默认 192.168.1.13）
static std::string ROBOT_IP = "192.168.1.13";

// ============================================================
// 【恒力驻留参数设定区】
// ============================================================

/// 恒力控制方向 (0=X, 1=Y, 2=Z)
static constexpr int DESIRED_FORCE_AXIS = 2;

/// 目标按压力 (N)：导纳控制器将调节位置使接触力稳定在此值附近
/// 导纳内部 K=0，稳态误差趋近于 0
/// 建议从 5~10N 开始调，逐步增加
static constexpr double DESIRED_FORCE = -10.0;

/// 恒力驻留时长 (s)
static constexpr double HOLD_DURATION = 20.0;

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

// ============================================================
// 六维力传感器读取回调（恒力接口在后台线程中调用）
// ============================================================

/// @brief 封装 get_sensor_6d_data 的传感器回调
/// @param user_data 指向 SOCKETFD（6001）的指针
static bool sensor_callback_impl(double wrench[6], void* user_data)
{
    if (user_data == nullptr) {
        return false;  // 计入断连计数，触发安全自动停止
    }
    const tl::SOCKETFD sock = *static_cast<const tl::SOCKETFD*>(user_data);
    tl::Sensor6DData data;
    if (tl::get_sensor_6d_data(sock, data) != tl::Result::SUCCESS || !data.sensorConnected) {
        return false;
    }
    wrench[0] = data.fxDataSubBase;
    wrench[1] = data.fyDataSubBase;
    wrench[2] = data.fzDataSubBase;
    wrench[3] = data.mxDataSubBase;
    wrench[4] = data.myDataSubBase;
    wrench[5] = data.mzDataSubBase;
    return true;
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[]) {
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
    std::cout << "=== 恒力驻留控制示例 ===" << '\n';

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

    tl::SOCKETFD sock_6001 = guard.sock_6001;
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
    // @note K=50 N/m 时弹簧过软：压入 5mm 只需 0.25N，导纳完全退让，接触力 ≈ 0
    //        对应 K=50000 N/m（50 N/mm）：压入 5mm 产生 ~25~125N 接触力（视表面硬度）
    adm_params.mass[0] = 80.0;
    adm_params.mass[1] = 80.0;
    adm_params.mass[2] = 160.0;
    adm_params.mass[3] = 20.0;
    adm_params.mass[4] = 20.0;
    adm_params.mass[5] = 20.0;

    adm_params.damping[0] = 1200.0;
    adm_params.damping[1] = 1200.0;
    adm_params.damping[2] = 10000.0;
    adm_params.damping[3] = 20.0;
    adm_params.damping[4] = 20.0;
    adm_params.damping[5] = 20.0;

    adm_params.stiffness[0] = 50000.0;
    adm_params.stiffness[1] = 50000.0;
    adm_params.stiffness[2] = 0.0;      // 力控轴 K=0，消除稳态误差
    adm_params.stiffness[3] = 10;
    adm_params.stiffness[4] = 10;
    adm_params.stiffness[5] = 10;

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
    // 10. 运动到恒力起始位姿
    // ============================================================
    print_separator("运动到恒力起始位姿");
    {
        tl::MoveCmd move_cmd;
        move_cmd.targetPosType = tl::PosType::data;
        move_cmd.targetPosName = "";
        move_cmd.coord = tl::Coord::BASE;  // 基坐标
        move_cmd.velocity = 80;
        move_cmd.acc = 10;
        move_cmd.dec = 10;
        move_cmd.pl = 0;
        // [X, Y, Z, RX, RY, RZ]  (mm, rad) — 按实际工况修改
        move_cmd.targetPosValue = std::vector<double>{
            230.083, -0.020, 243.598,
            -3.141, 0.003, 0.0};
        const tl::Result ret_move = tl::robot_movel(sock_6001, move_cmd);
        print_result("robot_movel 运动到恒力起始位姿", ret_move);
        if (ret_move != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 移动到恒力起始位姿失败\n";
            return -1;
        }
        if (!wait_motion_done(sock_6001, 30)) {
            std::cerr << "  [WARN] 运动到恒力起始位姿超时，继续后续流程\n";
        }
    }

    // ============================================================
    // 11. 恒力驻留
    // ============================================================
    print_separator("恒力驻留");
    std::cout << "  [信息] 开始恒力驻留控制\n";
    std::cout << "  [信息] 力控轴: " << DESIRED_FORCE_AXIS
              << "  目标力: " << DESIRED_FORCE << "N\n";
    std::cout << "  [信息] 驻留时长: " << HOLD_DURATION << "s\n";

    {
        // 运动到位后读取实际位姿作为恒力基准
        std::vector<double> base_pose(7, 0.0);
        ret = tl::get_current_position(sock_6001, tl::Coord::BASE, base_pose);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取恒力起始位姿失败\n";
            return -1;
        }
        std::cout << "  [信息] 恒力基准位姿: X=" << base_pose[0]
                  << "mm  Y=" << base_pose[1] << "mm  Z=" << base_pose[2] << "mm\n";

        // 恒力控制参数：伺服速度/加速度/加加速度（百分比）+ 显式力控轴
        tl::ConstantForceParams cf_params;
        cf_params.force_control_axis = DESIRED_FORCE_AXIS;
        cf_params.sv_pct = SERVO_SPEED;
        cf_params.sa_pct = SERVO_ACCEL;
        cf_params.sj_pct = SERVO_JERK;

        admittance.Reset();

        ret = tl::open_constforce(sock_6001, sock_7000, base_pose, admittance,
                              sensor_callback_impl, &sock_6001, cf_params);
        print_result("open_constforce 开启恒力控制", ret);
        if (ret != tl::Result::SUCCESS) {
            return -1;
        }

        // 主线程轮询状态：实时读取接触力（原始/滤波）与当前目标位姿
        const auto status_deadline = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(static_cast<int>(HOLD_DURATION * 1000.0));
        int status_count = 0;
        bool worker_alive = true;
        while (std::chrono::steady_clock::now() < status_deadline && worker_alive) {
            tl::ConstForceStatus cf_status;
            ret = tl::get_constforce_status(cf_status);
            if (ret == tl::Result::SUCCESS && !cf_status.running) {
                // 后台闭环已提前退出（如传感器断连自动停止）
                std::cerr << "  [WARN] 恒力后台闭环已停止，原因: " << cf_status.last_result << "\n";
                worker_alive = false;
                break;
            }
            if (ret == tl::Result::SUCCESS && cf_status.running && status_count % 10 == 0) {
                std::cout << "  [状态] F_raw=" << std::setprecision(2)
                          << cf_status.sensor_wrench[0] << ','
                          << cf_status.sensor_wrench[1] << ','
                          << cf_status.sensor_wrench[2] << "N  "
                          << "F_ftr="
                          << cf_status.filtered_wrench[0] << ','
                          << cf_status.filtered_wrench[1] << ','
                          << cf_status.filtered_wrench[2] << "N"
                          << "  target=(" << std::setprecision(3)
                          << cf_status.target_pose[0] << ','
                          << cf_status.target_pose[1] << ','
                          << cf_status.target_pose[2] << ")"
                          << "mm\n";
            }
            ++status_count;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        ret = tl::close_constforce();
        print_result(ret == tl::Result::SUCCESS ? "close_constforce 正常关闭" : "close_constforce 异常结束", ret);
        if (ret == tl::Result::EXCEPTION) {
            std::cerr << "  [WARN] 恒力控制因传感器断连自动停止\n";
        }
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

    std::cout << "\n[信息] 恒力驻留示例运行完毕\n";
    std::cout << "  [提示] ↑DESIRED_FORCE = 力更大  |  ↓DESIRED_FORCE = 力更小\n";
    std::cout << "  [提示] ↑HOLD_DURATION = 驻留更久\n";
    std::cout << "  [提示] 修改 DESIRED_FORCE_AXIS 调整力控方向\n";
    return 0;
}
