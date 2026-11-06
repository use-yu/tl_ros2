/**
 * ex_info_query.cpp
 * @brief 相关信息查询接口（包括获取 SDK 版本、控制器连接/断开、当前坐标、
 *        运行状态、指定关节在基坐标系中的位置、DH 参数、六维力传感器数据、关节软件版本号、
 *        碰撞防护等级、远程 IO 功能设置、电流环拖动示教灵敏度、工具坐标范围参数）
 * @attention 确保机械臂供电正常、网络通信正常，
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_info_query
 */

#include <iostream>
#include <iomanip>
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
    // 解析命令行参数: ./ex_info_query [-ip 192.168.1.13]，默认 192.168.1.13
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

    std::cout << std::endl;

    // ----获取机械臂当前坐标----
    std::cout << "[INFO] **********获取机械臂当前坐标**********" << std::endl;
    tl::Coord coord = tl::Coord::JOINT;
    std::vector<double> current_position;
    auto ret = tl::get_current_position(socket_fd, coord, current_position);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取机械臂当前坐标失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 机械臂当前坐标: ";
    for (const auto& pos : current_position) {
        std::cout << pos << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    // ----获取机械臂运行状态----
    std::cout << "[INFO] **********获取机械臂运行状态**********" << std::endl;
    int status = 0; 
    ret = tl::get_robot_running_state(socket_fd, status);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取机械臂运行状态失败，程序退出!" << std::endl;
        return -1;
    }
    switch (status)
    {
        case 0:
            std::cout << "[INFO] 机械臂处于停止状态 (status = " << status << ")" << std::endl;
            break;
        case 1:
            std::cout << "[INFO] 机械臂处于暂停状态 (status = " << status << ")" << std::endl;
            break;
        case 2:
            std::cout << "[INFO] 机械臂处于运行状态 (status = " << status << ")" << std::endl;
            break;
        default:
            break;
    }
    std::cout << std::endl;
    
    // ----获取指定关节在基坐标系中的位置（位姿）----
    std::cout << "[INFO] **********获取指定关节在基坐标系中的位置**********" << std::endl;
    int axisNum = 7;
    std::vector<double> joint_position;
    ret = tl::get_joint_position(socket_fd, axisNum, joint_position);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取" << axisNum << "关节在基坐标系中的位置失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 机械臂" << axisNum << "关节在基坐标系中的位置: ";
    for (const auto& joint_pos : joint_position) {
        std::cout << joint_pos << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;


    // 获取机械臂DH参数
    std::cout << "[INFO] **********获取机械臂DH参数**********" << std::endl;
    tl::RobotDHParam dh_params;
    ret = tl::get_robot_dh_param(socket_fd, dh_params);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取机械臂DH参数失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 机械臂DH参数: ";
    std::cout << "L1=" << dh_params.L1 << ", L2=" << dh_params.L2 << ", L3=" << dh_params.L3 
              << ", L4=" << dh_params.L4 << ", L5=" << dh_params.L5 << ", L6=" << dh_params.L6 
              << ", L7=" << dh_params.L7 << ", L8=" << dh_params.L8 << ", L9=" << dh_params.L9 
              << ", L10=" << dh_params.L10 << ", L11=" << dh_params.L11 << ", L12=" << dh_params.L12 
              << ", L13=" << dh_params.L13 << ", L14=" << dh_params.L14 << ", L15=" << dh_params.L15 
              << ", L16=" << dh_params.L16 << ", L17=" << dh_params.L17 << ", L18=" << dh_params.L18 
              << ", L19=" << dh_params.L19 << ", L20=" << dh_params.L20 
              << ", Couple_Coe_1_2=" << dh_params.Couple_Coe_1_2 
              << ", Couple_Coe_2_3=" << dh_params.Couple_Coe_2_3 
              << ", Couple_Coe_3_2=" << dh_params.Couple_Coe_3_2 
              << ", Couple_Coe_3_4=" << dh_params.Couple_Coe_3_4 
              << ", Couple_Coe_4_5=" << dh_params.Couple_Coe_4_5 
              << ", Couple_Coe_4_6=" << dh_params.Couple_Coe_4_6 
              << ", Couple_Coe_5_6=" << dh_params.Couple_Coe_5_6 
              << ", dynamicLimit_max=" << dh_params.dynamicLimit_max 
              << ", dynamicLimit_min=" << dh_params.dynamicLimit_min 
              << ", pitch=" << dh_params.pitch 
              << ", sliding_lead_value=" << dh_params.sliding_lead_value 
              << ", uplift_lead_value=" << dh_params.uplift_lead_value 
              << ", spray_distance=" << dh_params.spray_distance 
              << ", threeAxisDirection=" << dh_params.threeAxisDirection 
              << ", fiveAxisDirection=" << dh_params.fiveAxisDirection 
              << ", twoAxisConversionRatio=" << dh_params.twoAxisConversionRatio 
              << ", threeAxisConversionRatio=" << dh_params.threeAxisConversionRatio 
              << ", amplificationRatio=" << dh_params.amplificationRatio 
              << ", conversionratio_x=" << dh_params.conversionratio_x 
              << ", conversionratio_y=" << dh_params.conversionratio_y 
              << ", conversionratio_z=" << dh_params.conversionratio_z 
              << ", conversionratio_J1=" << dh_params.conversionratio_J1 
              << ", conversionratio_J2=" << dh_params.conversionratio_J2 
              << ", conversionratio_J3=" << dh_params.conversionratio_J3 
              << ", upsideDown=" << dh_params.upsideDown 
              << ", hanyu.PC=" << dh_params.hanyu.PC 
              << ", hanyu.SP[0]=" << dh_params.hanyu.SP[0] 
              << ", hanyu.SP[1]=" << dh_params.hanyu.SP[1] 
              << ", hanyu.SP[2]=" << dh_params.hanyu.SP[2] 
              << ", hanyu.TL[0]=" << dh_params.hanyu.TL[0] 
              << ", hanyu.TL[1]=" << dh_params.hanyu.TL[1] 
              << ", hanyu.TL[2]=" << dh_params.hanyu.TL[2] 
              << std::endl;
    std::cout << std::endl;

    // ----查询六维力传感器数据----
    std::cout << "[INFO] **********查询六维力传感器数据**********" << std::endl;
    {
        tl::Sensor6DData sensor_data;
        ret = tl::get_sensor_6d_data(socket_fd, sensor_data);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取六维力传感器数据失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 六维力传感器数据:" << std::endl;
            std::cout << "  传感器连接状态: " << (sensor_data.sensorConnected ? "已连接" : "未连接") << std::endl;
            std::cout << "  原始数据 - Fx=" << sensor_data.fxData << ", Fy=" << sensor_data.fyData
                      << ", Fz=" << sensor_data.fzData << std::endl;
            std::cout << "  原始数据 - Mx=" << sensor_data.mxData << ", My=" << sensor_data.myData
                      << ", Mz=" << sensor_data.mzData << std::endl;
            std::cout << "  去皮数据 - Fx=" << sensor_data.fxDataSubBase << ", Fy=" << sensor_data.fyDataSubBase
                      << ", Fz=" << sensor_data.fzDataSubBase << std::endl;
            std::cout << "  去皮数据 - Mx=" << sensor_data.mxDataSubBase << ", My=" << sensor_data.myDataSubBase
                      << ", Mz=" << sensor_data.mzDataSubBase << std::endl;
        }
    }
    std::cout << std::endl;

    // ----查询关节软件版本号----
    std::cout << "[INFO] **********查询关节软件版本号**********" << std::endl;
    {
        int query_axis = 1;  // 查询 J1 轴
        std::string joint_version;
        ret = tl::query_joint_software_version(socket_fd, query_axis, joint_version);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 查询关节" << query_axis << "软件版本号失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 关节" << query_axis << "软件版本号: " << joint_version << std::endl;
        }
    }
    std::cout << std::endl;

    // ----查询碰撞防护等级----
    std::cout << "[INFO] **********查询碰撞防护等级**********" << std::endl;
    {
        tl::CollisionPara collision_param;
        ret = tl::get_collision_detection_param(socket_fd, collision_param);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取碰撞防护参数失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 碰撞防护参数:" << std::endl;
            std::cout << "  指令位置响应时间: " << collision_param.position_delay_time_ms_value << " ms" << std::endl;
            std::cout << "  误差允许时间: " << collision_param.error_enable_time_ms_value << " ms" << std::endl;
            std::cout << "  机器人轴数: " << collision_param.axisum << std::endl;
            std::cout << "  碰撞检测阈值（点动）: ";
            for (const auto& val : collision_param.collisionDetection_teach) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
            std::cout << "  碰撞检测阈值（指令）: ";
            for (const auto& val : collision_param.collisionDetection_run) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    // ----查询远程IO功能设置----
    std::cout << "[INFO] **********查询远程IO功能设置**********" << std::endl;
    {
        int remote_num = 0;
        int remote_time = 0;
        tl::RemoteControl remote_general;
        std::vector<tl::RemoteProgram> remote_program;
        ret = tl::get_remote_function(socket_fd, 1, remote_num, remote_time, remote_general, remote_program);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取远程IO功能设置失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 远程IO功能设置:" << std::endl;
            std::cout << "  远程IO数量: " << remote_num << std::endl;
            std::cout << "  IO重复触发屏蔽时间: " << remote_time << " ms" << std::endl;
            std::cout << "  启动端口: " << remote_general.startPort
                      << " (有效值: " << remote_general.startValue << ")" << std::endl;
            std::cout << "  暂停端口: " << remote_general.pausePort
                      << " (有效值: " << remote_general.pauseValue << ")" << std::endl;
            std::cout << "  停止端口: " << remote_general.stopPort
                      << " (有效值: " << remote_general.stopValue << ")" << std::endl;
            std::cout << "  清除报警端口: " << remote_general.faultResetPort
                      << " (有效值: " << remote_general.faultResetValue << ")" << std::endl;
            std::cout << "  清除断电保持数据端口: " << remote_general.clearStashPort
                      << " (有效值: " << remote_general.clearStashValue << ")" << std::endl;
            std::cout << "  远程程序数量: " << remote_program.size() << std::endl;
            for (size_t i = 0; i < remote_program.size(); ++i) {
                std::cout << "    程序[" << i << "]: 端口=" << remote_program[i].port
                          << ", 值=" << remote_program[i].value << std::endl;
            }
        }
    }
    std::cout << std::endl;

    // ----查询电流环拖动示教灵敏度----
    std::cout << "[INFO] **********查询电流环拖动示教灵敏度**********" << std::endl;
    {
        std::vector<double> friction_offsets;
        ret = tl::get_current_teach_sensitivity(socket_fd, friction_offsets);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取电流环拖动示教灵敏度失败，错误码: " << ret << std::endl;
        } else {
            std::cout << "[INFO] 电流环拖动示教灵敏度 (J1~J6): ";
            for (const auto& val : friction_offsets) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;

    // ----查询工具坐标范围参数----
    std::cout << "[INFO] **********查询工具坐标范围参数**********" << std::endl;
    {
        int tool_number = 1;
        tl::ToolCoordinateRange range_param;
        ret = tl::get_tool_coordinate_range(socket_fd, tool_number, range_param);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取工具" << tool_number << "坐标范围参数失败，错误码: " << ret << std::endl;
        } else {
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "[INFO] 工具" << tool_number << "坐标范围参数:" << std::endl;
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
