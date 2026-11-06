/**
 * ex_robot_state_callback.cpp
 * @brief 7000端口状态回调函数测试（TCP）
 * @attention
 *   - 需连接控制器的 7000 端口（高频状态端口）
 *   - 回调函数在独立线程中触发，不能做耗时操作
 *   - 7000 端口与 6001 端口相互独立，需分别 connect/disconnect
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_robot_state_callback
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include "tl_api.h"

// 回调函数：收到控制器状态时被调用
void on_robot_state(const char* message)
{
    if (message != nullptr) {
        std::cout << "[回调] 收到状态: " << message << std::endl;
    }
}

// 测试一次完整的 7000 端口状态订阅流程
tl::Result test_state_subscription(const std::string& ip, const std::string& port,
                                   const std::string& mode_name)
{
    std::cout << "\n========== 测试 " << mode_name << " 连接 7000 端口 ==========" << std::endl;

    // 连接 7000
    auto sock_7000 = tl::connect_robot(ip, port);
    if (sock_7000 < 0) {
        std::cerr << "[ERROR] " << mode_name << " 连接 7000 失败" << std::endl;
        return tl::Result::DISCONNECT;
    }
    std::cout << "[INFO] " << mode_name << " 7000 连接成功 (socket=" << sock_7000 << ")" << std::endl;

    // 注册回调
    std::cout << "[INFO] 注册状态回调..." << std::endl;
    auto ret = tl::robot_state_callback(sock_7000, on_robot_state);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 注册回调失败，错误码: " << ret << std::endl;
        tl::disconnect_robot(sock_7000);
        return ret;
    }
    std::cout << "[INFO] 回调注册成功" << std::endl;

    // 分别尝试单次查询(mode=0) 和 持续推送(mode=1)
    for (int mode = 0; mode <= 1; mode++) {
        std::cout << "[INFO] --- mode=" << mode << " 测试 ---" << std::endl;
        tl::RobotState param;
        param.channel = 1;
        param.stop = false;
        param.mode = mode;
        param.interval = 1000;
        param.position = 0;
        param.servoStatus = true;
        param.programRunStatus = true;

        ret = tl::get_robot_state(sock_7000, param);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] tl::get_robot_state(mode=" << mode << ") 失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 请求已发送，等待5秒看是否有回调..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    // 停止推送
    std::cout << "[INFO] 停止推送..." << std::endl;
    {
        tl::RobotState param;
        param.channel = 1;
        param.stop = true;
        ret = tl::get_robot_state(sock_7000, param);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "[INFO] 推送已停止" << std::endl;
        }
    }

    tl::disconnect_robot(sock_7000);
    std::cout << "[INFO] " << mode_name << " 7000 端口已断开" << std::endl;
    return tl::Result::SUCCESS;
}

int main(int argc, char* argv[])
{
    // 解析命令行参数: ./ex_robot_state_callback [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    std::string port_6001 = "6001";
    std::string port_7000 = "7000";

    std::string version = tl::get_library_version();
    std::cout << "[INFO] SDK 版本: " << version << std::endl;

    // ---- 连接 6001 端口（控制端口）----
    std::cout << "[INFO] 连接 6001 端口..." << std::endl;
    auto sock_6001 = tl::connect_robot(ip, port_6001);
    if (sock_6001 < 0) {
        std::cerr << "[ERROR] 连接 6001 失败" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 6001 连接成功 (socket=" << sock_6001 << ")" << std::endl;

    // ---- 测试: TCP 连接 7000 ----
    test_state_subscription(ip, port_7000, "TCP");

    // ---- 断开 6001 ----
    std::cout << "\n========== 断开 6001 端口 ==========" << std::endl;
    tl::disconnect_robot(sock_6001);

    std::cout << "[INFO] 测试完毕" << std::endl;
    return 0;
}
