/**
 * ex_coord_tool.cpp
 * @brief 坐标系与工具管理相关接口（包括设置工具手参数、设置用户坐标系、设置用户坐标编号、获取当前工作坐标系、设置当前工作坐标系）
 * @attention 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_coord_tool
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#ifdef _WIN32
#include <windows.h>
#endif
#include "tl_api.h"

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
    // 解析命令行参数: ./ex_coord_tool [-ip 192.168.1.13]，默认 192.168.1.13
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

    // 注意: set_tool_hand_param/set_user_coordinate_data 涉及配置修改，
    //       错误的值可能导致路径偏差，请确认参数正确。
    // ----设置工具手参数---
    std::cout << "[INFO] **********设置工具手参数**********" << std::endl;
    int tool_num = 1;
    tl::ToolParam tool_param;
    tool_param.X = 0;
    tool_param.Y = 0;
    tool_param.Z = 0;
    tool_param.A = 0;
    tool_param.B = 0;
    tool_param.C = 0;
    auto ret = tl::set_tool_hand_param(socket_fd, tool_num, tool_param);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置工具手参数失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置工具手参数成功" << std::endl;
    std::cout << std::endl;

    // ----设置用户坐标系----
    std::cout << "[INFO] **********设置用户坐标系**********" << std::endl;
    int userNum = 1;
    std::vector<double> user_coord = {0, 0, 0, 0, 0, 0, 0};
    ret = tl::set_user_coordinate_data(socket_fd, userNum, user_coord);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置用户坐标系失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置用户坐标系成功" << std::endl;
    std::cout << std::endl;

    // ----设置用户坐标编号----
    std::cout << "[INFO] **********设置用户坐标编号**********" << std::endl;
    userNum = 2;
    ret = tl::set_user_coord_number(socket_fd, userNum);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置用户坐标编号失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置用户坐标编号成功" << std::endl;
    std::cout << std::endl;

    // ----获取当前工作坐标系----
    std::cout << "[INFO] **********获取当前工作坐标系**********" << std::endl;
    tl::Coord coord;
    ret = tl::get_current_coord(socket_fd, coord);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取当前工作坐标系失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 当前坐标系编号为：" << static_cast<int>(coord) << std::endl;
    std::cout << std::endl;

    // ----设置当前工作坐标系----
    std::cout << "[INFO] **********设置当前工作坐标系**********" << std::endl;
    coord = tl::Coord::BASE;
    ret = tl::set_current_coord(socket_fd, coord);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置当前工作坐标系失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置当前工作坐标系成功" << std::endl;
    std::cout << std::endl;

    // ----设置工具包络球参数----
    std::cout << "[INFO] **********设置工具包络球参数**********" << std::endl;
    {
        int tool_number = 1;
        tl::ToolCoordinateRange range_param;
        range_param.min_range_x = -500.0;
        range_param.max_range_x = 500.0;
        range_param.min_range_y = -400.0;
        range_param.max_range_y = 400.0;
        range_param.min_range_z = -300.0;
        range_param.max_range_z = 300.0;
        range_param.tool_hand_method = 1;

        ret = tl::set_tool_coordinate_range(socket_fd, tool_number, range_param);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 设置工具" << tool_number << "坐标范围参数失败，错误码: " << ret << std::endl;
        } else {
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "[INFO] 已设置工具" << tool_number << "坐标范围参数:" << std::endl;
            std::cout << "  X轴范围: [" << range_param.min_range_x
                      << ", " << range_param.max_range_x << "] mm" << std::endl;
            std::cout << "  Y轴范围: [" << range_param.min_range_y
                      << ", " << range_param.max_range_y << "] mm" << std::endl;
            std::cout << "  Z轴范围: [" << range_param.min_range_z
                      << ", " << range_param.max_range_z << "] mm" << std::endl;
            std::cout << "  工具握持方式: " << range_param.tool_hand_method << std::endl;
        }
    }
    std::cout << std::endl;

    // ----三点法用户坐标系标定----
    // 步骤: 建议先在示教器上标记好三个点位，然后在示教器上移动机械臂到原点O → 标定O → 移到X方向点 → 标定X → 移到Y方向点 → 标定Y → 计算坐标
    // 注意: 此操作为交互式，需要手动移机后在终端输入对应编号
    std::cout << "[INFO] **********三点法用户坐标系标定**********" << std::endl;
    std::cout << "[INFO] 请手动移动机械臂到目标位置后，在终端输入对应数字:" << std::endl;
    std::cout << "  0 - 标定原点 O" << std::endl;
    std::cout << "  1 - 标定 X 方向点" << std::endl;
    std::cout << "  2 - 标定 Y 方向点" << std::endl;
    std::cout << "  3 - 计算坐标" << std::endl;
    std::cout << "  其他数字 - 退出" << std::endl;
    {
        int calib_cmd = -1;
        while (true) {
            std::cout << "请输入标定类型: ";
            std::cin >> calib_cmd;

            if (calib_cmd == 0) {
                ret = tl::calibration_oxy(socket_fd, 1, "O");
                std::cout << "  标定原点 O，结果: " << ret << std::endl;
            } else if (calib_cmd == 1) {
                ret = tl::calibration_oxy(socket_fd, 1, "X");
                std::cout << "  标定 X 方向点，结果: " << ret << std::endl;
            } else if (calib_cmd == 2) {
                ret = tl::calibration_oxy(socket_fd, 1, "Y");
                std::cout << "  标定 Y 方向点，结果: " << ret << std::endl;
            } else if (calib_cmd == 3) {
                ret = tl::calculate_user_coordinate(socket_fd, 1);
                std::cout << "  计算坐标，结果: " << ret << std::endl;
                std::cout << "  [信息] 标定完成，退出流程" << std::endl;
                break;
            } else {
                std::cout << "  [错误] 无效输入，请输入 0-3 (0=原点O, 1=X方向, 2=Y方向, 3=计算坐标)" << std::endl;
            }
        }
    }
    std::cout << std::endl;

    // ----设置机械臂末端参数为默认值----
    std::cout << "[INFO] **********设置机械臂末端参数为默认值**********" << std::endl;
    ret = tl::set_default_cartesian_params(socket_fd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置机械臂末端参数为默认值失败，错误码: " << ret << std::endl;
    } else {
        std::cout << "[INFO] 设置机械臂末端参数为默认值成功" << std::endl;
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
