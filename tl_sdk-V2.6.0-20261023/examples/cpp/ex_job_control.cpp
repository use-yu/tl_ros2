/**
 * ex_job_control.cpp
 * @brief 作业控制相关接口（包括作业文件插入MoveJ、MoveL、MoveC，打开作业文件，创建作业文件，获取全部作业文件名，运行作业文件）
 * @attention
 *   - 插入作业之前需要先打开一个作业文件
 *   - 运行一个作业文件需要切换到运行模式
 *   - 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_job_control
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
    // 解析命令行参数: ./ex_job_control [-ip 192.168.1.13]，默认 192.168.1.13
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

    // ----创建作业文件----
    std::string jobName = "TEST_CPP";
    std::cout << "[INFO] **********创建作业文件**********" << std::endl;
    auto ret = tl::job_create(socket_fd, jobName);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 创建作业文件失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 创建作业文件成功" << std::endl;
    std::cout << std::endl;

    // ----打开作业文件----
    std::cout << "[INFO] **********打开作业文件**********" << std::endl;
    ret = tl::job_open(socket_fd, jobName);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 打开作业文件失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 打开作业文件成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveJ----
    std::cout << "[INFO] **********作业文件插入MoveJ**********" << std::endl;
    int line = 1;
    tl::MoveCmd cmd;
    cmd.targetPosType  = tl::PosType::PType;    
    cmd.targetPosName  = "P0001";          
    cmd.coord          = tl::Coord::JOINT;                 
    cmd.velocity       = 40;               
    cmd.acc            = 10;               
    cmd.dec            = 10;              
    cmd.pl             = 0;                 
    cmd.targetPosValue = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ret = tl::job_insert_moveJ(socket_fd, line, cmd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveJ失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveJ成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveL----
    std::cout << "[INFO] **********作业文件插入MoveL**********" << std::endl;
    line = 2;
    cmd.targetPosName  = "P0002";
    cmd.coord          = tl::Coord::BASE;  
    cmd.targetPosValue = std::vector<double>{280, 0.0, 380, 3.14, 0.0, 0.0};
    ret = tl::job_insert_moveL(socket_fd, line, cmd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveL失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveL成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveC----
    std::cout << "[INFO] **********作业文件插入MoveC**********" << std::endl;
    line = 3;
    cmd.targetPosName  = "P0003";
    cmd.coord          = tl::Coord::TOOL;  
    cmd.targetPosValue = std::vector<double>{270, 0.0, 380, 3.14, 0.0, 0.0};
    ret = tl::job_insert_moveC(socket_fd, line, cmd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveC失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveC成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveC----
    std::cout << "[INFO] **********作业文件插入MoveC**********" << std::endl;
    line = 3;
    cmd.targetPosName  = "P0003";
    cmd.coord          = tl::Coord::TOOL;  
    cmd.targetPosValue = std::vector<double>{240, 0.0, 380, 3.14, 0.0, 0.0};
    ret = tl::job_insert_moveC(socket_fd, line, cmd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveC失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveC成功" << std::endl;
    std::cout << std::endl;

    // ----获取所有作业文件名----
    std::cout << "[INFO] **********获取所有作业文件名**********" << std::endl;
    std::vector<std::vector<std::string>> AllJobNames;
    ret = tl::job_get_all_jobfile_name(socket_fd, AllJobNames);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取作业文件名失败，程序退出!" << std::endl;
        return -1;
    }
    int i = 1;
    for (const auto& JobNames : AllJobNames) {
        for (const auto& name : JobNames) {
            std::cout << "[INFO] 作业文件 " << i++ << " 名称: " << name << std::endl;
        }
    }

    // ----切换为运行模式----
    // 运行作业文件需要在运行模式下执行（2=运行）
    std::cout << "--- 切换运行模式 ---" << std::endl;
    ret = tl::set_current_mode(socket_fd, 2);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 切换运行模式失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 已切换为运行模式" << std::endl;

    // ----运行作业文件----
    std::cout << "[INFO] **********运行作业文件**********" << std::endl;
    ret = tl::job_run(socket_fd, jobName);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 运行作业文件失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 运行作业文件成功" << std::endl;
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
