/**
 * ex_error_zero.cpp
 * @brief 错误处理接口（清除错误）
 * @attention 确保机械臂供电正常、网络通信正常
 * @warning
 *   set_axis_zero_position（设置关节零位）属于高级标定操作，
 *   错误设置会导致机器人定位偏差甚至机械碰撞，默认已注释。
 *   如需使用，请仔细阅读控制器手册并在专业人员指导下操作。
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_error_zero
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "tl_api.h"

#ifdef _WIN32
#include <windows.h>
#endif
void enable_servo(const int socket_fd)
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
    return;
  }
  std::cout << "  [信息] 当前伺服状态: " << state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

  if (state == 2) {
    std::cerr << "[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试" << std::endl;
    return;
  }

  if (state == 3) {
    std::cout << "[INFO] 机械臂已在使能上电状态" << std::endl;
    return;
  }

  // 3. 上电流程
  if (state == 0) {
    std::cout << "  [信息] 伺服处于停止状态，设置就绪..." << std::endl;
    ret = tl::set_servo_state(socket_fd, 1);
    if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
      std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "  [信息] 上电使能..." << std::endl;
  ret = tl::set_servo_poweron(socket_fd);
  if (ret != tl::Result::SUCCESS) {
    std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 4. 确认结果
  tl::get_servo_state(socket_fd, state);
  if (state == 3) {
    std::cout << "[INFO] 机械臂使能上电成功（servo_state = " << state << "）" << std::endl;
  } else {
    std::cerr << "[ERROR] 机械臂使能上电失败（servo_state = " << state << "）" << std::endl;
  }
}

void disable_servo(const int socket_fd)
{
  int state = -1;
  tl::get_servo_state(socket_fd, state);
  switch (state)
  {
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

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 解析命令行参数: ./ex_error_zero [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    // 定义端口
    std::string port = "6001";
    
    // 获取 SDK 版本
    std::string version = tl::get_library_version();
    std::cout << "[INFO] SDK 版本: " << version << std::endl;

    // 连接控制器
    std::cout << "[INFO] 连接控制器..." << std::endl;
    auto socket_fd = tl::connect_robot(ip, port);
    if (socket_fd < 0) {
        std::cerr << "[ERROR] 连接失败，程序退出!"<< std::endl;
        return -1;
    }
    std::cout << "[INFO] 连接成功 (socket=" << socket_fd << ")" << std::endl;
    
    // ----清除错误----
    std::cout << "[INFO] **********清除错误**********" << std::endl;
    auto ret = tl::clear_error(socket_fd);
    if (ret != tl::Result::SUCCESS) {
      std::cerr << "[ERROR] 清除错误失败：" << ret << "，程序退出!" << std::endl;
      return -1;
    }
    std::cout << "[INFO] 清除错误成功" << std::endl;
    std::cout << std::endl;

    // 使能上电
    enable_servo(socket_fd);

    // ----设置指定关节零位（危险操作，默认注释）----
    // ⚠️  危险: set_axis_zero_position 会修改机器人关节零位标定，
    //    错误的值会导致所有后续运动出现定位偏差，严重时可能造成机械碰撞。
    //    仅在专业指导下、确认当前零位确需校正时使用。
    //    取消下面代码的注释前，请确认 axis 值正确且已理解全部后果。
    //
    // std::cout << "[INFO] **********设置指定关节零位**********" << std::endl;
    // int axis = 1;
    // ret = set_axis_zero_position(socket_fd, axis);
    // if (ret != Result::SUCCESS) {
    //     std::cerr << "[ERROR] 设置指定关节零位失败，程序退出!" << std::endl;
    //     return -1;
    // }
    // std::cout << "[INFO] 设置指定关节零位成功" << std::endl;
    // std::cout << std::endl;

    // ----标定六维力传感器零位/重心----
    // 操作前请确保:
    //   1. 机械臂末端无负载或已安装标准负载
    //   2. 机械臂处于竖直姿态（关节各轴在合理位置）
    //   3. 周围安全，标定过程中机械臂会缓慢运动
    std::cout << "[INFO] **********标定六维力传感器零位/重心**********" << std::endl;
    {
        std::cout << "[INFO] 即将执行六维力传感器标定，请确认:" << std::endl;
        std::cout << "  1. 机械臂末端无负载（或已安装标准负载）" << std::endl;
        std::cout << "  2. 机械臂周围安全无人" << std::endl;
        std::cout << "  是否继续? (y/n): ";
        char confirm = 'n';
        std::cin >> confirm;

        if (confirm == 'y' || confirm == 'Y') {
            bool calib_success = false;
            ret = tl::mark_base_sensor(socket_fd, calib_success);
            if (ret != tl::Result::SUCCESS) {
                std::cerr << "[ERROR] 标定六维力传感器失败，错误码: " << ret << std::endl;
            } else if (calib_success) {
                std::cout << "[INFO] 标定六维力传感器成功" << std::endl;

                // 查询标定后的基础参数
                tl::SensorBaseParam base_param;
                ret = tl::get_sensor_base_param(socket_fd, base_param);
                if (ret == tl::Result::SUCCESS) {
                    std::cout << "  传感器质量: " << base_param.sensorMass << std::endl;
                    std::cout << "  质心 - X: " << base_param.sensorMassCenterX
                              << ", Y: " << base_param.sensorMassCenterY
                              << ", Z: " << base_param.sensorMassCenterZ << std::endl;
                    std::cout << "  标零状态: " << (base_param.saveZero ? "已标零" : "未标零") << std::endl;
                }
            } else {
                std::cerr << "[ERROR] 标定六维力传感器失败，标定结果为 false" << std::endl;
            }
        } else {
            std::cout << "  [信息] 用户取消标定" << std::endl;
        }
    }
    std::cout << std::endl;

    // 使能下电
    disable_servo(socket_fd);

    // 断开控制器连接
    std::cout << "[INFO] 断开控制器连接..." << std::endl;
    ret = tl::disconnect_robot(socket_fd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 断开连接失败，程序退出!"<< std::endl;
        return -1;
    }
    std::cout << "[INFO] 断开连接成功" << std::endl;

    return 0;
}
