/**
 * ex_digital_io.cpp
 * @brief  模式与 IO 控制接口（包括设置机械臂模式、设置数字 IO 输出、设置数字 IO 输出、获取所有 IO 输入状态）
 * @attention 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_digital_io
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
    // 解析命令行参数: ./ex_digital_io [-ip 192.168.1.13]，默认 192.168.1.13
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

    // 使能上电
    enable_servo(socket_fd);

    // ----设置机器人模式----
    std::cout << "[INFO] **********设置机器人模式**********" << std::endl;
    int mode = 1;
    auto ret = tl::set_current_mode(socket_fd, mode);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置机器人模式失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置机器人模式成功" << std::endl;
    std::cout << std::endl;

    // ----设置数字 IO 输出----
    std::cout << "[INFO] **********设置数字 IO 输出**********" << std::endl;
    int io_port = 1;
    int value = 0;
    ret = tl::set_digital_output(socket_fd, io_port, value);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置数字 IO 输出失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置数字 IO 输出成功" << std::endl;
    std::cout << std::endl;

    // ----查询数字 IO 输出----
    std::cout << "[INFO] **********查询数字 IO 输出**********" << std::endl;
    std::vector<int> outputValues;
    ret = tl::get_digital_output(socket_fd, outputValues);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 查询数字 IO 输出失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 查询数字 IO 输出成功，状态为：";
    for (const auto& state : outputValues) {
        std::cout << state << " ";
    }
    std::cout << std::endl;

    // ----获取所有 IO 输入状态----
    std::cout << "[INFO] **********获取所有 IO 输入状态**********" << std::endl;
    std::vector<int> inputValues;
    ret = tl::get_digital_input(socket_fd, inputValues);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 查询数字 IO 输入失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 查询数字 IO 输入成功，状态为：";
    for (const auto& state : inputValues) {
        std::cout << state << " ";
    }
    std::cout << std::endl;

    // ----设置远程IO功能----
    std::cout << "[INFO] **********设置远程IO功能**********" << std::endl;
    {
        tl::RemoteControl remote_general;
        remote_general.clearStashPort = 1;     // 清除断电保持数据绑定端口
        remote_general.faultResetPort = 2;     // 清除报警绑定端口
        remote_general.pausePort      = 3;     // 暂停绑定端口
        remote_general.startPort      = 4;     // 启动绑定端口
        remote_general.stopPort       = 5;     // 停止绑定端口

        remote_general.clearStashValue = 1;    // 清除断电保持有效参数 (0/1)
        remote_general.faultResetValue = 1;    // 清除报警有效参数 (0/1)
        remote_general.pauseValue      = 1;    // 暂停有效参数 (0/1)
        remote_general.startValue      = 1;    // 启动有效参数 (0/1)
        remote_general.stopValue       = 1;    // 停止有效参数 (0/1)

        std::vector<tl::RemoteProgram> remote_program;
        for (int i = 0; i < 3; i++) {
            tl::RemoteProgram prog;
            prog.port  = 6 + i;   // 远程程序绑定端口
            prog.value = 1;        // 有效参数 (0/1/2)
            remote_program.push_back(prog);
        }

        int remote_num = 10;
        ret = tl::set_remote_function(socket_fd, 1, remote_general, remote_program, remote_num);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 设置远程IO功能失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 设置远程IO功能成功" << std::endl;
            std::cout << "  清除断电保持: 端口" << remote_general.clearStashPort
                      << " (有效参数=" << remote_general.clearStashValue << ")" << std::endl;
            std::cout << "  清除报警:     端口" << remote_general.faultResetPort
                      << " (有效参数=" << remote_general.faultResetValue << ")" << std::endl;
            std::cout << "  暂停:         端口" << remote_general.pausePort
                      << " (有效参数=" << remote_general.pauseValue << ")" << std::endl;
            std::cout << "  启动:         端口" << remote_general.startPort
                      << " (有效参数=" << remote_general.startValue << ")" << std::endl;
            std::cout << "  停止:         端口" << remote_general.stopPort
                      << " (有效参数=" << remote_general.stopValue << ")" << std::endl;
            for (size_t i = 0; i < remote_program.size(); i++) {
                std::cout << "  程序" << (i + 1) << ": 端口" << remote_program[i].port
                          << " (有效参数=" << remote_program[i].value << ")" << std::endl;
            }
            std::cout << "  远程IO数量: " << remote_num << std::endl;
        }
    }
    std::cout << std::endl;

    // ----查询流程图编程运行状态----
    std::cout << "[INFO] **********查询流程图编程运行状态**********" << std::endl;
    {
        int run_status = -1;
        ret = tl::get_robot_running_state(socket_fd, run_status);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 查询流程图编程运行状态失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 流程图编程运行状态: ";
            switch (run_status) {
                case 0: std::cout << "停止 (status=0)"; break;
                case 1: std::cout << "暂停 (status=1)"; break;
                case 2: std::cout << "运行 (status=2)"; break;
                default: std::cout << "未知 (status=" << run_status << ")"; break;
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    // 使能下电
    tl::set_current_mode(socket_fd, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
