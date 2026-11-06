/**
 * ex_user_coord_ik.cpp
 * @brief 在启用工具手和用户坐标的条件下，将用户坐标系下的目标位姿逆解为关节坐标，
 *        并用 MoveJ 下发运动到该关节角。
 *        演示流程: 选择 1 号工具手 → 选择 2 号用户坐标 → 给定用户坐标点位姿 → 逆解关节角 → 正解回验 → MoveJ 下发
 * @attention 运行前请确保:
 *            1. 机械臂供电正常、网络通信正常（默认 IP 192.168.1.13）
 *            2. 已标定 1 号工具手（TCP）与 2 号用户坐标，并已保存到控制器。标定方式二选一:
 *               - 推荐: 在示教器上手动标定，操作更快、可实时可视化确认标定结果
 *               - 代码标定: 工具手用 2 点/20 点法（tool_hand_2_or_20_point_calibrate +
 *                 tool_hand_2_or_20_point_calibrate_caculate）或 7 点法（tool_hand_7_point_calibrate +
 *                 tool_hand_7_point_calibrate_caculate）；用户坐标用 calculate_user_coordinate（三点 OXY 标定）
 *                 或 set_user_coordinate_data（直接写入坐标数据）
 *               本示例只使用控制器中已保存的标定结果，不执行标定流程
 * @warning 本示例会真实驱动机械臂运动到逆解点，运行前请确认机械臂周围无人员/障碍物
 * @note 运行步骤
 *       编译: cd examples/cpp && cmake --preset linux-native && cmake --build --preset linux-native
 *       运行: ./build/linux-native/ex_user_coord_ik [X Y Z Rx Ry Rz]
 *             （可选 6 个参数为目标点在 2 号用户坐标系下的位姿，位移单位 mm、姿态单位 rad；
 *               缺省时以当前位姿为基准，沿用户 X 轴正方向偏移 100mm 作为目标点）
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "tl_api.h"


#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif

// 弧度常量（避免依赖 POSIX 扩展宏 M_PI，保证 MSVC 可编译）
constexpr double kPi = 3.14159265358979323846;

// ============================================================
// 工具函数
// ============================================================

/// @brief 打印接口调用结果
void print_result(const std::string& msg, int ret)
{
    std::cout << "  [结果] " << msg << " -> " << ret << " (0=成功)" << std::endl;
}

/// @brief 打印分隔标题
void print_separator(const std::string& title)
{
    std::cout << "\n========== " << title << " ==========" << std::endl;
}

/// @brief 打印 double 向量
void print_vector(const std::string& name, const std::vector<double>& v)
{
    std::cout << "  " << name << " [";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i] << (i + 1 < v.size() ? ", " : "");
    }
    std::cout << "]" << std::endl;
}

// ============================================================
// 伺服上电
// ============================================================

/// @brief 清除错误并执行标准两步上电流程
/// @return true 上电成功（伺服状态=3 运行）
bool enable_servo(const int socket_fd)
{
    std::cout << "--- 伺服上电 ---" << std::endl;

    // 1. 先清错、下电复位，确保控制器状态同步
    tl::clear_error(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 2. 检查当前伺服状态
    int state = -1;
    auto ret = tl::get_servo_state(socket_fd, state);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
        return false;
    }
    std::cout << "  [信息] 当前伺服状态: " << state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

    if (state == 2) {
        std::cerr << "[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试" << std::endl;
        return false;
    }

    if (state == 3) {
        std::cout << "  [信息] 机械臂已在使能上电状态" << std::endl;
        return true;
    }

    // 3. 两步上电: 伺服就绪 -> 上电使能
    if (state == 0) {
        std::cout << "  [信息] 设置伺服就绪..." << std::endl;
        ret = tl::set_servo_state(socket_fd, 1);
        if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
            std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "  [信息] 上电使能..." << std::endl;
    ret = tl::set_servo_poweron(socket_fd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 4. 确认上电结果
    tl::get_servo_state(socket_fd, state);
    if (state == 3) {
        std::cout << "  [信息] 上电成功（servo_state = " << state << "）" << std::endl;
        return true;
    }
    std::cerr << "[ERROR] 上电失败（servo_state = " << state << "）" << std::endl;
    return false;
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // ---- 参数解析: [-ip 192.168.1.13] [X Y Z Rx Ry Rz]，单位 mm + rad ----
    std::string ip = "192.168.1.13";     // 控制器 IP，默认 192.168.1.13，可用 -ip 覆盖
    std::vector<double> user_target;   // 空 = 使用默认演示点
    int argi = 1;
    while (argi < argc) {
        std::string arg(argv[argi]);
        if (arg == "-ip" || arg == "--ip") {
            if (argi + 1 >= argc) {
                std::cerr << "[ERROR] 用法: " << argv[0]
                          << " [-ip 192.168.1.13] [X Y Z Rx Ry Rz]" << std::endl;
                return -1;
            }
            ip = argv[argi + 1];
            argi += 2;
        } else {
            break;   // 首个位置参数出现，剩余全部按位姿参数解析
        }
    }
    int pose_count = argc - argi;
    if (pose_count != 0 && pose_count != 6) {
        std::cerr << "[ERROR] 用法: " << argv[0]
                  << " [-ip 192.168.1.13] [X Y Z Rx Ry Rz]" << std::endl;
        std::cerr << "        位姿参数须为 6 个（X Y Z Rx Ry Rz），姿态单位为 rad；缺省时使用默认演示点" << std::endl;
        return -1;
    }
    for (int i = argi; i < argc; ++i) {
        try {
            user_target.push_back(std::stod(argv[i]));
        } catch (const std::exception&) {
            std::cerr << "[ERROR] 参数 " << i << " ('" << argv[i]
                      << "') 不是有效数字，请检查输入" << std::endl;
            return -1;
        }
    }

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    tl::SOCKETFD sock = tl::connect_robot(ip, "6001");
    if (sock < 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket=" << sock << ")" << std::endl;

    tl::Result ret;

    // ---- 切换示教模式（运动指令需要）----
    ret = tl::set_current_mode(sock, 0);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 切换示教模式失败: " << ret << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 上电 ----
    if (!enable_servo(sock)) {
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 1. 启用 1 号工具手（用户已标定）----
    // 标定说明: 优先在示教器上手动标定 1 号工具手（更快，实时可视化）；
    //           也可用代码标定: 先 set_tool_hand_number 选中该工具手，再采集标定点
    //           （tool_hand_2_or_20_point_calibrate 2/20 点法 或 tool_hand_7_point_calibrate 7 点法），
    //           最后 tool_hand_7_point_calibrate_caculate / tool_hand_2_or_20_point_calibrate_caculate 计算保存。
    //           本示例假设已标定完成，直接启用即可。
    print_separator("启用工具手");
    constexpr int kToolNum = 1;    // 用户标定的工具手编号
    ret = tl::set_tool_hand_number(sock, kToolNum);
    print_result("set_tool_hand_number(1)", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置工具手编号失败，请确认示教器上已标定 1 号工具手" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    // 控制器对 set 的写入是异步的，需延迟后才能正确读取（与 ex_tool_hand_number 约定一致）
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int cur_tool_num = -1;
    ret = tl::get_tool_hand_number(sock, cur_tool_num);
    print_result("get_tool_hand_number", ret);
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [信息] 当前工具手编号: " << cur_tool_num << std::endl;
    }

    // ---- 2. 选择 2 号用户坐标（用户已标定）----
    // 标定说明: 优先在示教器上手动标定 2 号用户坐标（更快，实时可视化）；
    //           也可用代码标定: calculate_user_coordinate 三点 OXY 标定计算，
    //           或 set_user_coordinate_data 直接写入已知的用户坐标数据。
    //           本示例假设已标定完成，直接启用即可。
    print_separator("选择用户坐标");
    constexpr int kUserNum = 2;    // 用户标定的用户坐标编号
    ret = tl::set_user_coord_number(sock, kUserNum);
    print_result("set_user_coord_number(2)", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置用户坐标编号失败，请确认示教器上已标定 2 号用户坐标" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    // 控制器对 set 的写入是异步的，需延迟后才能正确读取
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int cur_user_num = -1;
    ret = tl::get_user_coord_number(sock, cur_user_num);
    print_result("get_user_coord_number", ret);
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [信息] 当前用户坐标编号: " << cur_user_num << std::endl;
    }

    // ---- 3. 获取当前关节角（作为逆解参考点，选择最近形态）----
    print_separator("获取当前位姿");
    std::vector<double> current_joints;
    ret = tl::get_current_position(sock, tl::Coord::JOINT /* 关节坐标系 */, current_joints);
    print_result("get_current_position(关节)", ret);
    if (ret != tl::Result::SUCCESS || current_joints.size() < 6) {
        std::cerr << "[ERROR] 获取当前关节角失败，请确认伺服已上电" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    print_vector("当前关节角 [°]", current_joints);

    // 将当前关节角正解到 2 号用户坐标系，作为默认目标点的基准
    std::vector<double> current_user;
    bool fk_state = false;
    ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::JOINT /* 关节 */, current_joints,
                                           tl::Coord::USER /* 用户 */, current_user, fk_state);
    if (ret == tl::Result::SUCCESS && fk_state) {
        print_vector("当前位姿（2号用户坐标下） [mm, rad]", current_user);
    } else {
        std::cout << "  [信息] 当前位姿正解到用户坐标失败，将使用内置演示点" << std::endl;
        ret = tl::Result::PARAM_ERR;   // 标记正解失败，走兜底演示点分支
    }

    // ---- 4. 确定目标点（2 号用户坐标系下的位姿）----
    print_separator("目标位姿（2号用户坐标下）");
    std::vector<double> target_user;
    if (!user_target.empty()) {
        target_user = user_target;
    } else if (ret == tl::Result::SUCCESS && current_user.size() >= 6) {
        // 默认演示点: 当前位姿沿用户 X 轴正方向偏移 100mm，姿态B(Y轴)逆时针针旋转 0.4 弧度
        target_user = current_user;
        target_user[0] += 100.0;
        target_user[4] += 0.4;
        std::cout << "  [信息] 未指定目标点，使用默认演示点（当前位姿沿用户 X +100mm）" << std::endl;
    } else {
        // 无法获取当前位姿时的兜底演示点（需根据实际工况调整）
        target_user = {200.0, 0.0, 200.0, 0.0, 0.0, 0.0};
        std::cout << "  [信息] 使用内置演示点 {200, 0, 200, 0, 0, 0}" << std::endl;
    }
    if (target_user.size() < 7) {
        target_user.resize(7, 0.0);   // SDK 协议点位统一为 7 元素，不足补 0
    }
    std::cout << "  目标位姿 [X,Y,Z]=[" << target_user[0] << ", " << target_user[1] << ", " << target_user[2]
              << "]mm  [Rx,Ry,Rz]=[" << target_user[3] << ", " << target_user[4] << ", " << target_user[5]
              << "]rad" << std::endl;

    // ---- 5. 逆解: 用户坐标(3) -> 关节坐标(0) ----
    // 使用当前设置的工具手编号(1)与用户坐标编号(2)参与计算
    print_separator("逆解: 用户坐标 -> 关节坐标");
    std::vector<double> joint_solution;
    bool convert_state = false;
    // form=0 形态自适应；reference_pos 传入当前关节角，逆解时优先选择最近的形态
    ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::USER /* 用户 */, target_user,
                                           tl::Coord::JOINT /* 关节 */, joint_solution, convert_state,
                                           0 /* form */, current_joints /* reference_pos */);
    print_result("get_origin_coord_to_target_coord (用户->关节)", ret);
    if (ret != tl::Result::SUCCESS || !convert_state) {
        std::cerr << "[ERROR] 逆解失败（ret=" << ret << ", convert_state=" << convert_state << "）" << std::endl;
        std::cerr << "        请检查: 目标点是否在机械臂工作空间内 / 工具手与用户坐标是否标定正确" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    print_vector("逆解关节坐标 [°]", joint_solution);

    // ---- 6. 正解回验: 关节坐标(0) -> 用户坐标(3)，验证逆解一致性 ----
    print_separator("正解回验");
    std::vector<double> user_back;
    bool back_state = false;
    ret = tl::get_origin_coord_to_target_coord(sock, tl::Coord::JOINT /* 关节 */, joint_solution,
                                           tl::Coord::USER /* 用户 */, user_back, back_state);
    print_result("get_origin_coord_to_target_coord (关节->用户)", ret);
    if (ret == tl::Result::SUCCESS && back_state && user_back.size() >= 6) {
        print_vector("正解回验位姿（2号用户坐标下） [mm, rad]", user_back);
        double pos_err = 0.0, att_err = 0.0;
        for (int i = 0; i < 3; ++i) {
            pos_err += std::fabs(user_back[i] - target_user[i]);
        }
        for (int i = 3; i < 6; ++i) {
            // 姿态角相差 2π 等价，取最小角度差
            double d = std::fabs(user_back[i] - target_user[i]);
            d = std::fmod(d, 2.0 * kPi);
            if (d > kPi) d = 2.0 * kPi - d;
            att_err += d;
        }
        std::cout << "  位置误差 [mm]: " << pos_err << "（应接近 0）" << std::endl;
        std::cout << "  姿态误差 [rad]: " << att_err << "（应接近 0）" << std::endl;
    } else {
        std::cerr << "[ERROR] 正解回验失败（ret=" << ret << ", back_state=" << back_state
                  << ", size=" << user_back.size() << "），跳过误差对比" << std::endl;
    }

    // ---- 7. MoveJ 下发逆解关节角 ----
    print_separator("MoveJ 下发逆解关节角");
    std::cout << "  [信息] 即将运动到逆解点，请确认机械臂周围安全..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 设置全局速度倍率
    ret = tl::set_speed(sock, 30);
    print_result("set_speed(30)", ret);
    // 控制器对 set 的写入是异步的，延迟后再下发运动指令
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    tl::MoveCmd move_cmd;
    move_cmd.targetPosType  = tl::PosType::data;    // 直接使用数值点位
    move_cmd.coord          = tl::Coord::JOINT;     // 关节坐标系
    move_cmd.velocity       = 40;               // 关节速度 (1,100]（最大速度百分比）
    move_cmd.acc            = 10;               // 加速度 (1,100]（百分比）
    move_cmd.dec            = 10;               // 减速度 (1,100]（百分比）
    move_cmd.pl             = 0;                // 平滑参数 [0,5]
    // 逆解关节角 [°]：前 6 位为 6 轴本体关节，第 7 位为 7 轴本体槽（6 轴机型补 0 占位）
    move_cmd.targetPosValue = joint_solution;
    if (move_cmd.targetPosValue.size() < 7) {
        move_cmd.targetPosValue.resize(7, 0.0);
    }

    ret = tl::robot_movej(sock, move_cmd);
    print_result("robot_movej", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] MoveJ 下发失败，程序退出!" << std::endl;
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    // robot_movej 为非阻塞接口，轮询当前关节角直到接近目标（逐轴阈值 0.5°），超时 20s
    constexpr double kArriveTol = 0.5;          // 到位阈值 [°]
    constexpr int kWaitStepMs = 200;            // 轮询周期 [ms]
    constexpr int kWaitTimeoutMs = 20000;       // 到位超时 [ms]
    std::vector<double> cur_joints;
    bool arrived = false;
    for (int waited = 0; waited < kWaitTimeoutMs; waited += kWaitStepMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kWaitStepMs));
        ret = tl::get_current_position(sock, tl::Coord::JOINT, cur_joints);
        if (ret != tl::Result::SUCCESS) continue;
        // 逐轴判断: 所有关节最大偏差小于阈值视为到位（6 轴/7 轴通用，按实际返回轴数比较）
        size_t n = std::min(cur_joints.size(), joint_solution.size());
        if (n == 0) continue;
        double max_err = 0.0;
        for (size_t i = 0; i < n; ++i) {
            max_err = std::max(max_err, std::fabs(cur_joints[i] - joint_solution[i]));
        }
        if (max_err < kArriveTol) {
            arrived = true;
            break;
        }
    }
    if (arrived) {
        std::cout << "  [信息] MoveJ 运动到位" << std::endl;
        print_vector("到位关节角 [°]", cur_joints);
    } else {
        std::cerr << "[ERROR] MoveJ 运动超时（20s 内未到位），下电停止运动" << std::endl;
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 断开连接 ----
    print_separator("断开连接");
    tl::set_servo_poweroff(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    tl::disconnect_robot(sock);
    std::cout << "[信息] 已断开连接，机械臂切换到示教模式" << std::endl;

    std::cout << "\n[信息] 用户坐标逆解示例程序运行完毕" << std::endl;
    return 0;
}
