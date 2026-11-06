/**
 * ex_user_coord_compare.cpp
 * @brief 演示同一个坐标值在不同用户坐标系下运动到不同的实际位置:
 *        程序自动设置两个用户坐标系 —— 1 号与基坐标重合、2 号绕基坐标 Z 轴顺时针旋转 45°，
 *        上电后先回零，然后在两个用户坐标系下用【完全相同的坐标值】各执行一次 MoveJ
 *        （演示点 = 回零位姿沿用户坐标 1 的 X 正方向偏移 50mm），
 *        直观展示"同一坐标值在不同用户坐标系下代表不同的空间点"。
 *        坐标值原样传给控制器（coord=3 + userNum），用户系→基系转换与逆解由控制器完成。
 * @attention
 *   - 本示例会【覆盖控制器中已保存的 1 号、2 号用户坐标系数据】（演示用），
 *     如需保留原数据请先在示教器上记录备份。
 *   - 本示例会真实驱动机械臂运动（上电后先回零，再在用户坐标 1 下沿用户 X 正方向
 *     移动 50mm，最后在用户坐标 2 下运动到对应点），运行前请确认机械臂周围无人员/障碍物。
 *   - 每轮运动前会用 set_user_coord_number 激活对应的用户坐标系。
 *   - 用户坐标 2 的原点在基坐标原点，绕基坐标 Z 轴顺时针旋转 45°（Rz=-45°），
 *     其 X 轴指向基坐标 X 正方向与 -Y 之间的 45° 方向，
 *     因此第二次运动的目标点 = 第一次运动的目标点（回零位姿沿用户 X 正方向 +50mm）
 *     绕基坐标原点顺时针旋转 45° 后的位置。
 *     绕 Z 旋转不改变半径，第二轮目标与当前点同样可达；若仍因姿态限制超出工作空间，
 *     robot_movej 会返回失败，请调整初始位姿重试。
 *   - 本示例未启用工具手（toolNum=0，TCP 即法兰中心），如需带工具请修改 kToolNum。
 * @note 运行步骤
 *       编译: cd examples/cpp && cmake --preset linux-native && cmake --build --preset linux-native
 *       运行: ./build/linux-native/ex_user_coord_compare
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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

// 演示参数
constexpr int kUserNumBase   = 1;   // 用户坐标 1: 与基坐标重合
constexpr int kUserNumRotZ   = 2;   // 用户坐标 2: 绕基坐标 Z 轴顺时针转 45°
constexpr double kOffsetUserX = 50.0;  // 演示点沿用户 X 正方向的偏移量 [mm]

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

/// @brief 打印位姿（前 3 位移 mm，后 3 姿态 rad + 换算角度）
void print_pose(const std::string& name, const std::vector<double>& pose)
{
    if (pose.size() < 6) {
        std::cout << "  " << name << "（数据不足 6 元素，size=" << pose.size() << "）" << std::endl;
        return;
    }
    std::cout << "  " << name << ": X=" << pose[0] << "mm Y=" << pose[1] << "mm Z=" << pose[2] << "mm"
              << "  Rx=" << pose[3] << "rad Ry=" << pose[4] << "rad Rz=" << pose[5] << "rad"
              << "（" << pose[3] * 180.0 / kPi << "°, " << pose[4] * 180.0 / kPi << "°, "
              << pose[5] * 180.0 / kPi << "°）" << std::endl;
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
        tl::set_servo_poweroff(socket_fd);   // 避免伺服残留在就绪态
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
    tl::set_servo_poweroff(socket_fd);   // 避免伺服残留在就绪态
    return false;
}

// ============================================================
// 运动到位轮询
// ============================================================

/**
 * @brief 轮询等待运动完成（运行状态=停止 且 关节角连续两次读数逐轴稳定）
 * @param sock    socket 句柄
 * @param baseline 下发运动前记录的关节角快照（用于确认运动确实发生）
 * @return true 运动完成；false 超时或连接断开
 *
 * @note MoveJ 的目标以用户坐标系表达，关节目标值由控制器内部计算，
 *       因此这里不做目标比对，改用"运行状态 + 关节角稳定"判断到位。
 *       为避免虚假到位（运动尚未启动就判定完成，进而提前下电），
 *       必须观察到运动确实发生（关节角相对快照变化 > 0.1° 或运行状态非停止），
 *       之后才接受"停止 + 稳定"作为到位条件。
 */
bool wait_move_done(const tl::SOCKETFD sock, const std::vector<double>& baseline)
{
    constexpr double kMotionTol = 0.1;        // 判定"确实发生运动"的关节变化阈值 [°]
    constexpr double kJointStableTol = 0.05;  // 到位稳定阈值（逐轴最大差） [°]
    constexpr int kWaitStepMs = 200;          // 轮询周期 [ms]
    constexpr int kStartTimeoutMs = 3000;     // 运动启动超时 [ms]
    constexpr int kWaitTimeoutMs = 30000;     // 到位超时 [ms]

    bool has_baseline = baseline.size() >= 6;
    bool saw_motion = false;    // 已观察到运动发生（关节变化或运行状态非停止）
    bool saw_running = false;   // 已观察到运行状态非停止
    std::vector<double> prev_joints;
    bool got_prev = false;

    for (int waited = 0; waited < kWaitTimeoutMs; waited += kWaitStepMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kWaitStepMs));

        int run_state = -1;
        tl::Result ret_run = tl::get_robot_running_state(sock, run_state);
        if (ret_run == tl::Result::DISCONNECT) {
            return false;   // 连接已断开，无法继续轮询
        }
        if (run_state != 0 && run_state != -1) {
            saw_running = true;
        }

        std::vector<double> joints;
        if (tl::get_current_position(sock, tl::Coord::JOINT /* 关节 */, joints) != tl::Result::SUCCESS) continue;
        if (joints.size() < 6) continue;   // 关节数据不足，跳过本轮

        if (!saw_motion && has_baseline) {
            // 相对运动前快照，任一轴变化超过阈值即视为确实发生运动
            size_t n = std::min(joints.size(), baseline.size());
            for (size_t i = 0; i < n; ++i) {
                if (std::fabs(joints[i] - baseline[i]) > kMotionTol) {
                    saw_motion = true;
                    break;
                }
            }
        }
        if (saw_running) {
            saw_motion = true;
        }

        // 运动启动检测: 一段时间内未观察到运动发生，视为目标不可达未启动
        if (!saw_motion && waited >= kStartTimeoutMs) {
            return false;
        }

        // 到位判定: 必须观察过运动、运行状态已停止、且关节连续读数逐轴稳定
        if (saw_motion && run_state == 0 && got_prev) {
            bool stable = true;
            size_t n = std::min(joints.size(), prev_joints.size());
            for (size_t i = 0; i < n; ++i) {
                if (std::fabs(joints[i] - prev_joints[i]) > kJointStableTol) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                return true;
            }
        }

        prev_joints = joints;
        got_prev = true;
    }
    return false;
}

// ============================================================
// 单轮演示: 在指定用户坐标系下用同一坐标值运动
// ============================================================

/**
 * @brief 在指定用户坐标系下执行一轮演示运动
 * @param sock       socket 句柄
 * @param user_num   用户坐标编号
 * @param user_pose  演示点坐标值 [X,Y,Z,Rx,Ry,Rz]（用户坐标系下表达，位移 mm、姿态 rad）
 * @param label      演示标签（用于打印）
 * @return true 演示成功；false 失败
 *
 * @note 流程: 激活用户坐标 -> MoveJ 直接下发（coord=3 + userNum，坐标值原样传递，
 *       由控制器内部完成用户系→基系转换与逆解）-> 轮询运动完成
 */
bool run_demo_round(const tl::SOCKETFD sock, const int user_num,
                    const std::vector<double>& user_pose, const std::string& label)
{
    print_separator("演示轮次: " + label);

    // ---- 1. 激活用户坐标 ----
    // 切换控制器当前使用的用户坐标编号，保证后续坐标读取与运动都基于目标用户坐标系
    tl::Result ret = tl::set_user_coord_number(sock, user_num);
    print_result("set_user_coord_number(" + std::to_string(user_num) + ")", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 激活用户坐标 " << user_num << " 失败" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int active_num = -1;
    ret = tl::get_user_coord_number(sock, active_num);
    if (ret != tl::Result::SUCCESS || active_num != user_num) {
        std::cerr << "[ERROR] 回读用户坐标编号失败（ret=" << ret << "，当前=" << active_num << "）" << std::endl;
        return false;
    }
    std::cout << "  [信息] 用户坐标 " << user_num << " 已激活" << std::endl;

    // ---- 2. 用【同一个坐标值】直接下发 MoveJ（coord=3 用户坐标系）----
    // 坐标值原样传给控制器，用户系→基系转换与逆解由控制器完成（与示教器一致）
    tl::MoveCmd move_cmd;
    move_cmd.targetPosType  = tl::PosType::data;   // 直接使用数值点位
    move_cmd.coord          = tl::Coord::USER;   // 用户坐标系
    move_cmd.userNum        = user_num;        // 用户坐标编号（MoveCmd 独立字段）
    move_cmd.velocity       = 30;              // 关节速度百分比 (0,100]
    move_cmd.acc            = 20;              // 加速度百分比 (0,100]
    move_cmd.dec            = 20;              // 减速度百分比 (0,100]
    move_cmd.pl             = 0;               // 平滑参数 [0,5]
    move_cmd.targetPosValue = user_pose;       // 同一坐标值（前 6 位本体，第 7 位为 7 轴占位，6 轴机型补 0）
    if (move_cmd.targetPosValue.size() < 7) {
        move_cmd.targetPosValue.resize(7, 0.0);
    }

    // 下发前记录关节角快照（用于到位轮询确认运动确实发生，避免虚假到位）
    // 读取失败时向量为空，wait_move_done 会退化为仅靠运行状态检测运动，属可接受降级
    std::vector<double> pre_joints;
    tl::get_current_position(sock, tl::Coord::JOINT /* 关节 */, pre_joints);

    std::cout << "  [信息] 下发 MoveJ（coord=3, userNum=" << user_num << "）..." << std::endl;
    ret = tl::robot_movej(sock, move_cmd);
    print_result("robot_movej", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] MoveJ 下发失败，目标点可能超出工作空间" << std::endl;
        return false;
    }

    // ---- 3. 轮询运动完成（必须观察到运动发生 + 运行状态停止 + 关节角稳定）----
    if (!wait_move_done(sock, pre_joints)) {
        std::cerr << "[ERROR] MoveJ 运动超时（30s 内未完成）或连接断开" << std::endl;
        return false;
    }
    std::cout << "  [信息] MoveJ 运动完成" << std::endl;
    return true;
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    // 解析命令行参数: ./ex_user_coord_compare [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
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

    // ============================================================
    // 0. 回零（回到设定的零点，作为演示起点）
    // ============================================================
    print_separator("回零");
    // 回零前记录关节角快照，用于到位轮询确认运动确实发生
    // 读取失败时向量为空: wait_move_done 退化为仅靠运行状态检测运动；
    // 下方"已在零点"判定要求快照非空，快照缺失时不满足即报错
    std::vector<double> pre_home_joints;
    tl::get_current_position(sock, tl::Coord::JOINT /* 关节 */, pre_home_joints);

    ret = tl::robot_go_home(sock);
    print_result("robot_go_home", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 回零指令下发失败: " << ret << std::endl;
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    if (!wait_move_done(sock, pre_home_joints)) {
        // 未观察到运动: robot_go_home 已下发成功，若关节角相对快照无变化，
        // 说明机械臂本就在零点附近（不依赖"零点=全零关节角"的假定）
        std::vector<double> joints;
        tl::get_current_position(sock, tl::Coord::JOINT /* 关节 */, joints);
        bool at_home = joints.size() >= 6 && pre_home_joints.size() >= 6;
        for (size_t i = 0; i < std::min(joints.size(), pre_home_joints.size()); ++i) {
            if (std::fabs(joints[i] - pre_home_joints[i]) > 1.0) {   // 运动阈值 [°]
                at_home = false;
                break;
            }
        }
        if (!at_home) {
            std::cerr << "[ERROR] 回零运动超时或未完成" << std::endl;
            tl::set_servo_poweroff(sock);
            tl::disconnect_robot(sock);
            return -1;
        }
        std::cout << "  [信息] 机械臂已在零点附近，无需回零运动" << std::endl;
    } else {
        std::cout << "  [信息] 回零完成" << std::endl;
    }

    // ============================================================
    // 1. 设置两个用户坐标系
    // ============================================================
    print_separator("设置用户坐标系");
    std::cout << "  [注意] 将覆盖控制器中已保存的 " << kUserNumBase << " 号、"
              << kUserNumRotZ << " 号用户坐标数据" << std::endl;

    // 1 号用户坐标: 与基坐标重合（原点在基原点，姿态 0）
    const std::vector<double> user_base_para = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // 2 号用户坐标: 绕基坐标 Z 轴顺时针转 45°（顺时针 = 负方向，Rz = -π/4）
    const std::vector<double> user_rotz_para = {0.0, 0.0, 0.0, 0.0, 0.0, -kPi / 4.0, 0.0};

    struct UserCoordDef {
        int num;
        const std::vector<double>& para;
        const char* desc;
    };
    const std::array<UserCoordDef, 2> user_defs = {{
        {kUserNumBase, user_base_para, "与基坐标重合（原点在基原点，姿态 0）"},
        {kUserNumRotZ, user_rotz_para, "绕基坐标 Z 轴顺时针转 45°（Rz=-45°，原点在基原点）"},
    }};

    for (const auto& def : user_defs) {
        ret = tl::set_user_coordinate_data(sock, def.num, def.para);
        print_result("set_user_coordinate_data(" + std::to_string(def.num) + ")", ret);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 设置用户坐标 " << def.num << " 失败" << std::endl;
            tl::set_servo_poweroff(sock);
            tl::disconnect_robot(sock);
            return -1;
        }
        std::cout << "  [信息] 用户坐标 " << def.num << ": " << def.desc << std::endl;
    }

    // 控制器对 set 的写入是异步的，延迟后再回读验证
    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (const auto& def : user_defs) {
        std::vector<double> para_back;
        ret = tl::get_user_coord_para(sock, def.num, para_back);
        print_result("get_user_coord_para(" + std::to_string(def.num) + ") 回读验证", ret);
        if (ret == tl::Result::SUCCESS) {
            print_vector("  回读参数", para_back);
        } else {
            // 用户坐标系定义是运动方向依据且已覆盖控制器原数据，未确认写入成功不得继续运动
            std::cerr << "[ERROR] 回读用户坐标 " << def.num << " 参数失败，中止演示" << std::endl;
            tl::set_servo_poweroff(sock);
            tl::disconnect_robot(sock);
            return -1;
        }
    }

    // ============================================================
    // 2. 构造演示点（用户坐标系下的坐标值）
    // ============================================================
    print_separator("构造演示点");

    // 先激活用户坐标 1，使 get_current_position(coord=3) 读到的位姿
    // 是在用户坐标 1 下表达的（用户坐标 1 与基坐标重合，其 X 方向即基坐标 X 方向）
    ret = tl::set_user_coord_number(sock, kUserNumBase);
    print_result("set_user_coord_number(" + std::to_string(kUserNumBase) + ")", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 激活用户坐标 " << kUserNumBase << " 失败" << std::endl;
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    std::vector<double> current_user1;
    ret = tl::get_current_position(sock, tl::Coord::USER, current_user1);
    if (ret != tl::Result::SUCCESS || current_user1.size() < 6) {
        std::cerr << "[ERROR] 获取当前位姿（用户坐标 1 下）失败，请确认伺服已上电" << std::endl;
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    // 演示点 = 回零位姿沿【用户 X 正方向】偏移 50mm，姿态沿用当前 RPY
    // 注意: 该坐标值是"用户坐标系下"的表达，在两个用户坐标系下语义不同:
    //   - 用户坐标 1（=基系）下: 实际位置 = 回零位姿沿基 X 正方向 50mm
    //   - 用户坐标 2（绕 Z 顺时针 45°）下: 用户 X 轴指向基系 X 与 -Y 之间 45° 方向，
    //     实际位置 = 用户 1 下的到达点绕基原点顺时针转 45°
    // 用户坐标 2 相对基系只是绕 Z 旋转，绕 Z 旋转不改变半径，可保证第二轮
    // 目标点与当前点同样可达
    std::vector<double> demo_user = current_user1;
    demo_user[0] += kOffsetUserX;
    print_pose("演示点坐标值（用户坐标系下表达）", demo_user);

    // ============================================================
    // 3. 演示: 用户坐标 1（与基坐标重合）
    // ============================================================
    if (!run_demo_round(sock, kUserNumBase, demo_user, "用户坐标 1（与基坐标重合）")) {
        tl::set_servo_poweroff(sock);
        tl::disconnect_robot(sock);
        return -1;
    }

    // ============================================================
    // 4. 演示: 用户坐标 2（绕基坐标 Z 轴顺时针转 45°）
    // ============================================================
    if (!run_demo_round(sock, kUserNumRotZ, demo_user, "用户坐标 2（绕 Z 顺时针转 45°）")) {
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

    std::cout << "\n[信息] 用户坐标系对比示例程序运行完毕" << std::endl;
    return 0;
}
