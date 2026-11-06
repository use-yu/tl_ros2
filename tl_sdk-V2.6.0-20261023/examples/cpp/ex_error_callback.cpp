/**
 * ex_error_callback.cpp
 * @brief 错误/警告回调示例程序
 * @attention
 *   - 确保控制器网络通信正常
 *   - 回调函数内不要做耗时操作或阻塞
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_error_callback
 */

#include "tl_api.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#endif
// ============================================================
// 回调函数（由用户实现，在收到错误/警告时被调用）
// ============================================================

/// @brief 错误/警告消息回调
/// @param messageType 消息类型（0=警告 1=错误）
/// @param message     消息内容
/// @param messageCode 消息码
void on_error_or_warning(int messageType, const char* message, int messageCode)
{
    const char* type_str = (messageType == 0) ? "警告" : "错误";
    std::cout << "\n>>> [回调触发] " << type_str
              << " | code=" << messageCode
              << " | " << message << std::endl;
}

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
    // 解析命令行参数: ./ex_error_callback [-ip 192.168.1.13]，默认 192.168.1.13
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

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    tl::SOCKETFD sock = tl::connect_robot(ip, "6001");
    if (sock < 0) {
        std::cerr << "[ERROR] 连接控制器失败" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket = " << sock << ")" << std::endl;

    tl::Result ret;

    // ============================================================
    // 1. 注册错误/警告回调
    // ============================================================
    print_separator("注册回调");
    ret = tl::set_receive_error_or_warnning_message_callback(sock, on_error_or_warning);
    print_result("set_receive_error_or_warnning_message_callback", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 注册回调失败" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    std::cout << "  [信息] 回调已注册，后续收到错误/警告时会自动触发" << std::endl;

    // ============================================================
    // 2. 逆解不可达点，触发错误回调
    // ============================================================
    // 通过 get_origin_coord_to_target_coord 尝试逆解一个超出工作空间的点，
    // 控制器会返回错误/警告，从而触发注册的回调函数
    print_separator("逆解不可达点");

    std::cout << "--- 逆解一个超出工作空间的点，观察回调触发 ---" << std::endl;
    {
        // 设置一个明显不可达的基坐标点（X 方向超出机械臂最大臂展）
        std::vector<double> unreachable_pose = {99999, 0, 0, 0, 0, 0};
        std::vector<double> joint_result;
        bool convert_state = false;

        ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::BASE, unreachable_pose, tl::Coord::JOINT, joint_result, convert_state);
        print_result("逆解不可达点 (基坐标→关节)", ret);
        std::cout << "  输入基坐标 [mm, rad]: X=99999, Y=0, Z=0" << std::endl;
        if (ret != tl::Result::SUCCESS) {
            std::cout << "  [信息] 逆解失败，控制器的错误消息已通过回调输出" << std::endl;
        }
    }

    // 等待回调处理完成
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ============================================================
    // 3. 断开连接
    // ============================================================
    print_separator("断开连接");
    tl::disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 错误回调示例程序运行完毕" << std::endl;
    return 0;
}
