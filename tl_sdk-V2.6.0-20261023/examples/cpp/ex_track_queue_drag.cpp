/**
 * ex_track_queue_drag.cpp
 * @brief 拖拽示教 + 轨迹记录/回放示例
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 拖拽需在示教模式（mode=0）下进行，示教模式下也可直接回放
 *   - 拖拽期间请注意安全，确保周围无人
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_track_queue_drag
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <ctime>
#include "tl_api.h"

void enable_servo(const int socket_fd)
{
  std::cout << "--- 检查伺服状态 ---" << std::endl;

  tl::clear_error(socket_fd);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  tl::set_servo_poweroff(socket_fd);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

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
    // 解析命令行参数: ./ex_track_queue_drag [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    std::string port = "6001";

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

    // ====================================================================
    // 第一部分：拖拽示教 + 轨迹记录/回放
    // ====================================================================

    // ----切换为示教模式（拖拽需要在示教模式下执行）----
    std::cout << "[INFO] **********切换为示教模式**********" << std::endl;
    {
        auto ret = tl::set_current_mode(socket_fd, 0);  // 0=示教模式（0=示教 1=远程 2=运行）
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 切换示教模式失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 已切换为示教模式" << std::endl;
        }
    }
    std::cout << std::endl;

    // ----设置拖拽方式（力矩模式，需在切拖拽类型前设置）----
    std::cout << "[INFO] **********设置拖拽方式（力矩模式）**********" << std::endl;
    {
        int mode = 1;  // 0-无   1-力矩模式  2-位置
        auto ret = tl::set_darg_mode(socket_fd, mode);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 设置拖拽模式失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 设置拖拽模式成功（mode = " << mode << "，力矩模式）" << std::endl;
        }
    }
    std::cout << std::endl;

    // ----设置示教类型为拖拽（0=点动 1=拖拽）----
    std::cout << "[INFO] **********设置示教类型为拖拽**********" << std::endl;
    {
        auto ret = tl::set_teach_type(socket_fd, 1);  // 1=拖拽
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 设置示教类型失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 已切换为拖拽示教类型" << std::endl;
        }
    }
    std::cout << std::endl;


    // ----轨迹记录开始（记录拖拽路径）----
    std::cout << "[INFO] **********轨迹记录开始**********" << std::endl;
    {
        double maxSamplingNum = 10000;   // 最大采样点数 [200, 12000]
        double samplingInterval = 1;   // 采样间隔 [0.03, 1]（用1秒，Python版本以此值成功回放）
        auto ret = tl::track_record_start(socket_fd, maxSamplingNum, samplingInterval);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 轨迹记录开始失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 轨迹记录已开始（maxSamplingNum=" << maxSamplingNum
                      << ", samplingInterval=" << samplingInterval << "s）" << std::endl;
        }
    }
    std::cout << std::endl;

    // 查询轨迹记录状态
    {
        bool recordStatus = false;
        auto ret = tl::get_track_record_status(socket_fd, recordStatus);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "[INFO] 轨迹记录状态: " << (recordStatus ? "开启" : "关闭") << std::endl;
        }
    }

    // 等待用户手动拖拽记录轨迹（用户拖拽完成后按回车继续）
    std::cout << "[INFO] 机械臂已进入拖拽状态，请手动拖拽机械臂记录轨迹..." << std::endl;
    std::cout << "[INFO] 拖拽完成后请按 Enter 键继续..." << std::endl;
    std::cin.get();

    // ----轨迹记录关闭----
    std::cout << "[INFO] **********轨迹记录关闭**********" << std::endl;
    {
        auto ret = tl::track_record_stop(socket_fd);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 轨迹记录关闭失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 轨迹记录已关闭" << std::endl;
        }
    }
    std::cout << std::endl;

    // ----退出拖拽模式（回放前需切回点动）----
    std::cout << "[INFO] **********退出拖拽模式**********" << std::endl;
    {
        auto ret = tl::set_teach_type(socket_fd, 0);  // 切回点动模式
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 切回点动模式失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 已切回点动模式" << std::endl;
        }
    }
    std::cout << std::endl;

    // ----轨迹记录保存（先保存到控制器，再回放）----
    std::cout << "[INFO] 等待1秒后保存..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "[INFO] **********轨迹记录保存**********" << std::endl;
    {
        std::time_t now = std::time(nullptr);
        std::tm* local = std::localtime(&now);
        char buf[64];
        std::strftime(buf, sizeof(buf), "轨迹名-%m月%d日%H时%M分%S", local);
        std::string trajName(buf);
        auto ret = tl::track_record_save(socket_fd, trajName);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 轨迹记录保存失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 轨迹记录已保存（trajName = " << trajName << "）" << std::endl;
        }
    }
    std::cout << std::endl;

    // ----轨迹回放（复现已保存的轨迹）----
    std::cout << "[INFO] 等待1秒后开始回放..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(4));
    std::cout << "[INFO] **********轨迹回放（复现拖拽路径）**********" << std::endl;
    {
        int vel = 100;  // 回放速度
        auto ret = tl::track_record_playback(socket_fd, vel);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 轨迹回放失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 轨迹回放开始（vel = " << vel << "），等待完成..." << std::endl;
            int runningState = -1;
            int timeout = 0;
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                tl::get_robot_running_state(socket_fd, runningState);
                timeout++;
            } while (runningState != 0 && timeout < 120);
            if (runningState == 0) {
                std::cout << "[INFO] 轨迹回放完成" << std::endl;
            }
        }
    }
    std::cout << std::endl;

    // ----轨迹清除（清除缓冲区）----
    std::cout << "[INFO] **********轨迹清除**********" << std::endl;
    {
        auto ret = tl::track_record_delete(socket_fd);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 轨迹清除失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 轨迹已清除" << std::endl;
        }
    }
    std::cout << std::endl;

    // 使能下电
    disable_servo(socket_fd);

    // 断开控制器连接
    std::cout << "[INFO] 断开控制器连接..." << std::endl;
    auto ret = tl::disconnect_robot(socket_fd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 断开连接失败，程序退出!"<< std::endl;
        return -1;
    }
    std::cout << "[INFO] 断开连接成功" << std::endl;

    return 0;
}