/**
 * ex_servoj.cpp
 * @brief 关节实时跟踪（servoJ）示例程序
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 需要双端口连接: 6001（控制）+ 7000（伺服高频透传）
 *   - servoJ 周期约 10ms，回调内不要做耗时操作
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 *   - 运行模式: 透传模式（实时的将关节数据下发给控制器，控制器直接响应。连接7000端口后，并调用open_servoJ即可进入透传模式。在此之前建议set_current_mode(sock, 2)切换到运行模式）
 *   - 开始前需: 双端口连接(6001+7000) → 上电 → open_servoJ
 *     （注意: servoJ 相关向量必须为 7 元素，传 6 元素会导致协议解析错乱断开连接）
 *   - 退出前需: close_servoJ → tl::set_current_mode(0) → tl::disconnect_robot(双端口)
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_servoj
 */

#include "tl_api.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

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

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 解析命令行参数: ./ex_servoj [-ip 192.168.1.13]，默认 192.168.1.13
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

    // ---- 清错与下电复位 ----
    tl::clear_error(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ---- 切换示教模式并设置伺服就绪 ----
    print_separator("切换示教模式");
    ret = tl::set_current_mode(sock, 0);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 切换示教模式失败: " << ret << std::endl;
        tl::disconnect_robot(sock);
        tl::disconnect_robot(sock_servo);
        return -1;
    }
    std::cout << "  [信息] 已切换为示教模式" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::cout << "  [信息] 设置伺服就绪..." << std::endl;
    ret = tl::set_servo_state(sock, 1);
    if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
        std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
        tl::disconnect_robot(sock);
        tl::disconnect_robot(sock_servo);
        return -1;
    }
    print_result("tl::set_servo_state(1)", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- 切换为运行模式 ----
    print_separator("切换运行模式");
    ret = tl::set_current_mode(sock, 2);
    print_result("tl::set_current_mode(2)", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ============================================================
    // 打开 servoJ 模式
    // ============================================================
    print_separator("servoJ 关节实时跟踪");

    // 速度/加速度/加加速度约束（7 元素，单位 °/s, °/s², °/s³）
    // 注意: SDK 底层协议要求 servoJ 相关向量固定为 7 元素，传 6 元素会导致
    //       C++ vector marshaling 时长度不足，控制器协议解析错乱并断开 7000 连接
    std::vector<double> vmax = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
    std::vector<double> amax = {60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0};
    std::vector<double> jmax = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    ret = tl::open_servoJ(sock_servo, vmax, amax, jmax);
    print_result("open_servoJ", ret);
    if (ret != tl::Result::SUCCESS) {
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        tl::disconnect_robot(sock_servo);
        return -1;
    }

    // ---- 动作 1: J1 从 0° 平滑转到 30° ----
    std::cout << "\n--- 动作 1: J1 转到 30° ---" << std::endl;
    {
        // q 必须为 7 元素，与 open_servoJ 一致
        std::vector<double> q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 200; ++i) {
            q[0] += (30.0 / 200.0);       // 每次增加 0.15°
            tl::set_servoJ_pos(sock_servo, q);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100Hz
        }
        std::cout << "  [信息] J1=30° 到位" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ---- 动作 2: J1 回到 0°，同时 J2 转到 20° ----
    std::cout << "\n--- 动作 2: J1 回零, J2 转到 20° ---" << std::endl;
    {
        // q 必须为 7 元素
        std::vector<double> q = {30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 200; ++i) {
            q[0] -= (30.0 / 200.0);       // J1 逐步回到 0°
            q[1] += (20.0 / 200.0);       // J2 逐步增加到 20°
            tl::set_servoJ_pos(sock_servo, q);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "  [信息] J1=0°, J2=20° 到位" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ---- 动作 3: J1=0°, J2 回到 0°, J3 转到 15° ----
    std::cout << "\n--- 动作 3: J2 回零, J3 转到 15° ---" << std::endl;
    {
        // q 必须为 7 元素
        std::vector<double> q = {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 200; ++i) {
            q[1] -= (20.0 / 200.0);       // J2 逐步回到 0°
            q[2] += (15.0 / 200.0);       // J3 逐步增加到 15°
            tl::set_servoJ_pos(sock_servo, q);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "  [信息] J2=0°, J3=15° 到位" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ---- 动作 4: 所有轴回到零位 ----
    std::cout << "\n--- 动作 4: 所有轴回零 ---" << std::endl;
    {
        std::vector<double> q = {0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < 150; ++i) {
            q[2] -= (15.0 / 150.0);       // J3 逐步回到 0°
            tl::set_servoJ_pos(sock_servo, q);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "  [信息] 所有轴回零到位" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ---- 关闭 servoJ 模式 ----
    ret = tl::close_servoJ(sock_servo);
    print_result("close_servoJ", ret);

    std::cout << "\n  [信息] servoJ 运动全部完成" << std::endl;

    // ============================================================
    // 切回示教模式（自动下电）& 断开
    // ============================================================
    print_separator("切回示教模式");

    ret = tl::set_current_mode(sock, 0);
    print_result("tl::set_current_mode(0) 切回示教模式（自动下电）", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    tl::disconnect_robot(sock);
    tl::disconnect_robot(sock_servo);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] servoJ 示例程序运行完毕" << std::endl;
    return 0;
}
