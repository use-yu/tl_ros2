/**
 * @file ex_servo_movel.cpp
 * @brief Servo MoveL 笛卡尔空间直线伺服运动测试
 *
 * 适用场景:
 *   - 通过高层封装函数测试 servo_movel
 *   - 验证关节空间输入与笛卡尔空间输入两种模式
 *   - 观察直线插值运动的执行过程
 *
 * 构建:
 *   cd <tl_sdk_root>
 *   cmake -B build -DBUILD_EXTENSION_EXAMPLES=ON
 *   cmake --build build --target ex_servo_movel
 *
 * 运行:
 *   ./build/examples/extension/cpp/ex_servo_movel
 *
 * 使用说明:
 *   - 修改 COORD_FLAG 和 TARGET 以切换测试目标
 *   - COORD_FLAG=0 时 TARGET 为关节角度 [J1..J7]（度）
 *   - COORD_FLAG=1/2/3 时 TARGET 为笛卡尔位姿 [x, y, z, rx, ry, rz]（mm, rad）
 *   - 默认示例会先运动到目标，再回到零位
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>
#include "tl_api.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif

// ============================================================
// 【用户设定区】—— 修改此处以改变测试目标
// ============================================================

/// 坐标系标志: 0=关节坐标, 1=基坐标系, 2=工具, 3=用户
/// COORD_FLAG=0 时 TARGET 为关节角度 [J1..J7]（度）
/// COORD_FLAG=1/2/3 时 TARGET 为笛卡尔位姿 [x,y,z,rx,ry,rz]（mm, rad）
static const int COORD_FLAG = 1;

/// 目标位置（根据 COORD_FLAG 自动解释为关节角或笛卡尔位姿）
/// COORD_FLAG=0 时填 7 个关节角度，COORD_FLAG!=0 时前 6 个为笛卡尔位姿（第 7 个忽略）
static const std::vector<double> TARGET = {
    285.0,    // x (mm)
    173.0,    // y (mm)
    251.0,    // z (mm)
     -3.14,   // rx (rad)
      0.0,    // ry (rad)
      0.0,    // rz (rad)
};

// Joint mode example (COORD_FLAG=0):
// static const std::vector<double> TARGET = {
//     45.0,    // J1 (度)
//    -30.0,    // J2 (度)
//     60.0,    // J3 (度)
//    -15.0,    // J4 (度)
//     25.0,    // J5 (度)
//    -10.0,    // J6 (度)
//     90.0     // J7（仅关节模式有效）
// };

/// 到达目标后暂停时间（秒）
static const int PAUSE_SECONDS = 3;

// ============================================================
// 辅助函数：打印目标
// ============================================================

static void print_target(const std::vector<double> &t, int coord) {
    std::cout << (coord == 0 ? "[关节] " : "[笛卡尔] ");
    std::cout << "[";
    size_t n = (coord == 0) ? std::min(t.size(), size_t(7)) : std::min(t.size(), size_t(6));
    for (size_t i = 0; i < n; ++i) {
        int prec = (coord == 0) ? 1 : 3;
        std::cout << std::fixed << std::setprecision(prec) << t[i];
        if (i < n - 1) std::cout << ", ";
    }
    std::cout << "]";
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 解析命令行参数: ./ex_servo_movel [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    std::cout << "=== Servo MoveL 笛卡尔空间直线伺服运动测试 ===\n\n";

    // ---- 打印测试参数 ----
    std::cout << "坐标系: coord=" << COORD_FLAG
              << (COORD_FLAG == 0 ? " (关节坐标→FK→笛卡尔)" : " (笛卡尔坐标)") << "\n";
    std::cout << "目标: ";
    print_target(TARGET, COORD_FLAG);
    std::cout << "\n\n";

    // ---- 连接 ----
    tl::SOCKETFD sock_6001 = tl::connect_robot(ip, "6001");
    tl::SOCKETFD sock_7000 = tl::connect_robot(ip, "7000");
    if (sock_6001 < 0 || sock_7000 < 0) {
        std::cerr << "连接失败\n";
        return -1;
    }
    std::cout << "  已连接到机械臂\n";

    // ---- 切换到示教模式 ----
    std::cout << "\n--- 切换到示教模式 ---\n";
    tl::set_current_mode(sock_6001, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ---- 清除错误 ----
    tl::clear_error(sock_6001);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(sock_6001);

    // ---- 切换到运行模式 ----
    std::cout << "\n--- 切换到运行模式 ---\n";
    tl::set_servo_state(sock_6001, 1);
    tl::set_current_mode(sock_6001, 2);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 开启 ServoJ（通过 open_servo_movel）----
    std::cout << "\n========== 开启伺服跟踪模式 ==========\n";
    tl::Result ret = tl::open_servo_movel(sock_7000, 10.0, 80.0, 10.0);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "open_servo_movel 失败\n";
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 零位（7 个关节角）----
    std::vector<double> zero(7, 0.0);

    // ---- MoveL：零位 → 目标 ----
    std::cout << "\n========== MoveL（零位 → 目标）==========\n";
    std::cout << "  目标: ";
    print_target(TARGET, COORD_FLAG);
    std::cout << "\n";

    ret = tl::servo_movel(sock_6001, sock_7000, TARGET, COORD_FLAG);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "servo_movel 正向运动失败, ret=" << ret << "\n";
    }

    tl::wait_servo();
    std::cout << "\n  到达目标，暂停 " << PAUSE_SECONDS << " 秒...\n";
    std::this_thread::sleep_for(std::chrono::seconds(PAUSE_SECONDS));

    // ---- MoveL：目标 → 零位（使用关节角 coord=0）----
    std::cout << "\n========== MoveL（目标 → 零位）==========\n";
    std::cout << "  目标: ";
    print_target(zero, 0);
    std::cout << "\n";

    ret = tl::servo_movel(sock_6001, sock_7000, zero, 0);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "servo_movel 反向运动失败, ret=" << ret << "\n";
    }

    tl::wait_servo();
    std::cout << "\n  回到零位，暂停 " << PAUSE_SECONDS << " 秒...\n";
    std::this_thread::sleep_for(std::chrono::seconds(PAUSE_SECONDS));

    // ---- 打断测试 ----
    {
        std::cout << "\n========== 打断测试（Cancel）==========\n";
        std::vector<double> far_target = {520.0, 0.0, 300.0, -3.141, 0.0, 0.0};
        std::cout << "  启动远距离运动（目标: ";
        print_target(far_target, 1);
        std::cout << "）...\n";

        ret = tl::servo_movel(sock_6001, sock_7000, far_target, 1);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "servo_movel 入队失败, ret=" << ret << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "  调用 tl::cancel_servo() 打断运动\n";
        tl::cancel_servo();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        bool busy = tl::servo_busy();
        std::cout << "  tl::servo_busy() = " << (busy ? "true（运动中）" : "false（空闲）")
                  << "  → " << (busy ? "⚠ 运动可能已完成" : "✓ 打断成功") << "\n";
    }

    // ---- 关闭 ----
    tl::wait_servo();
    tl::close_servo_movel(sock_7000);
    tl::set_current_mode(sock_6001, 0);

    tl::disconnect_robot(sock_7000);
    tl::disconnect_robot(sock_6001);
    std::cout << "\n=== Servo MoveL 测试完毕 ===\n";
    return 0;
}
