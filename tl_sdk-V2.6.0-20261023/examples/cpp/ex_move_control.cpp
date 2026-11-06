/**
 * ex_move_control.cpp
 * @brief 运动控制相关接口（包括MoveJ、MoveL）
 * @attention
 *   - 运行模式：示教模式
 *   - 开始前需: connect_robot → tl::set_servo_state(1) → set_servo_poweron
 *   - 退出前需: set_servo_poweroff → disconnect_robot
 *   - 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_move_control
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

/// @brief 轮询机器人运行状态等待运动完成，替代固定延时
/// @param sock TCP socket
/// @param timeout_sec 超时秒数
/// @return true 运动完成 / false 超时
static bool wait_motion_done(tl::SOCKETFD sock, int timeout_sec)
{
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::seconds(timeout_sec);
    int last_print = 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (std::chrono::steady_clock::now() < deadline) {
        int state = -1;
        tl::Result ret = tl::get_robot_running_state(sock, state);
        if (ret == tl::Result::SUCCESS && state == 0) {
            std::cout << "  [信息] 运动完成" << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        int elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() -
            (deadline - std::chrono::seconds(timeout_sec))).count());
        if (elapsed - last_print >= 5) {
            std::cout << "  [信息] 等待运动中... (已过 " << elapsed << "s)" << std::endl;
            last_print = elapsed;
        }
    }
    std::cerr << "  [WARN] 等待运动超时 (" << timeout_sec << "s)" << std::endl;
    return false;
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
    // 解析命令行参数: ./ex_move_control [-ip 192.168.1.13]，默认 192.168.1.13
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

    // ----切换为示教模式----
    std::cout << "--- 切换示教模式 ---" << std::endl;
    auto ret = tl::set_current_mode(socket_fd, 0);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 切换示教模式失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 已切换为示教模式" << std::endl;
    std::cout << std::endl;

    // 使能上电
    enable_servo(socket_fd);

    // 设置全局速度
    ret = tl::set_speed(socket_fd, 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ----MoveJ运动控制----
    std::cout << "[INFO] **********MoveJ运动控制**********" << std::endl;
    tl::MoveCmd cmd;
    cmd.targetPosType  = tl::PosType::PType;    
    cmd.targetPosName  = "";          
    cmd.coord          = tl::Coord::JOINT;                 
    cmd.velocity       = 40;               
    cmd.acc            = 10;               
    cmd.dec            = 10;              
    cmd.pl             = 0;                 
    cmd.targetPosValue = std::vector<double>{
      20.0, 10.0, -10.0, 
      0.0, 0.0, 0.0, 0.0}; // 关节坐标下单位为度
    ret = tl::robot_movej(socket_fd, cmd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] MoveJ运动控制失败，程序退出!" << std::endl;
        return -1;
    }
    // 等待运动到位
    wait_motion_done(socket_fd, 30);

    std::cout << "[INFO] MoveJ运动控制成功" << std::endl;
    std::cout << std::endl;

    // ----MoveL运动控制----
    std::cout << "[INFO] **********MoveL运动控制**********" << std::endl;
    cmd.targetPosName  = "";
    cmd.coord          = tl::Coord::BASE;  
    cmd.targetPosValue = std::vector<double>{
      280, 90.0, 270, 
      3.14, 0.0, 0.0}; // 基坐标下位移单位是mm，旋转单位是弧度
    ret = tl::robot_movel(socket_fd, cmd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] MoveL运动控制失败，程序退出!" << std::endl;
        return -1;
    }
    // 等待运动到位
    wait_motion_done(socket_fd, 30);

    std::cout << "[INFO] MoveL运动控制成功" << std::endl;
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
