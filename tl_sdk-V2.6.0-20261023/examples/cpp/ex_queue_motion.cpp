/**
 * ex_queue_motion.cpp
 * @brief 队列运动示例程序 —— 展示队列运动完整工作流
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 *   - 队列运动模式下，所有指令先在本地缓存，调用 send_to_controller 后一次性下发执行
 *   - 演示三种队列运动：纯 MoveJ、纯 MoveL、MoveJ+MoveL 混合
 *   - 运行模式: 队列模式（在queue_motion_set_status(true)之后，即可进入队列模式，在此之前建议set_current_mode(sock, 2)切换到运行模式）
 *   - 开始前需: connect_robot → 上电 → tl::queue_motion_set_status(true)
 *   - 退出前需: tl::queue_motion_set_status(false) → tl::set_current_mode(0) → set_servo_poweroff → disconnect_robot
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_queue_motion
 */

#include "tl_api.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <functional>

#ifdef _WIN32
#include <windows.h>
#endif
// ============================================================
// 工具函数
// ============================================================

void print_result(const std::string& msg, int ret)
{
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [成功] " << msg << std::endl;
    } else {
        std::cout << "  [失败] " << msg << "，错误码: " << ret << std::endl;
    }
}

void print_separator(const std::string& title)
{
    std::cout << "\n========== " << title << " ==========" << std::endl;
}

/**
 * @brief 构建 MoveCmd（关节坐标模式）
 * @param joints 6 个关节角度值，单位 °（度），顺序 [J1, J2, J3, J4, J5, J6]
 * @param vel    百分比 % (1,100]，最大关节速度的百分比，实际受全局速度限制
 * @param acc    百分比 % (1,100]
 * @param dec    百分比 % (1,100]
 * @param pl     平滑参数 [0,5]
 * @note 实际运动速度 = 全局速度 × (vel / 100)
 */
tl::MoveCmd make_movej_cmd(const std::vector<double>& joints,
                       double vel = 30, double acc = 50, double dec = 50, int pl = 0)
{
    tl::MoveCmd cmd;
    cmd.coord        = tl::Coord::JOINT; // 关节坐标系
    cmd.velocity     = vel;
    cmd.acc          = acc;
    cmd.dec          = dec;
    cmd.pl           = pl;
    cmd.targetPosValue.resize(14, 0.0);
    for (size_t i = 0; i < joints.size() && i < 7; ++i) {
        cmd.targetPosValue[i] = joints[i];
    }
    return cmd;
}

/**
 * @brief 构建 MoveCmd（基坐标模式）
 * @param pose 位姿 [X, Y, Z, Rx, Ry, Rz]，位置单位 mm，姿态单位 °
 * @param vel    百分比 % (1,1000]，最大 TCP 线速度的百分比
 * @param acc    百分比 % (1,100]
 * @param dec    百分比 % (1,100]
 * @param pl     平滑参数 [0,5]
 * @note 实际运动速度 = 全局速度 × (vel / 100)
 */
tl::MoveCmd make_movel_cmd(const std::vector<double>& pose,
                       double vel = 100, double acc = 50, double dec = 50, int pl = 0)
{
    tl::MoveCmd cmd;
    cmd.coord        = tl::Coord::BASE; // 基坐标系，可切换到 tl::Coord::TOOL / tl::Coord::USER
    cmd.velocity     = vel;
    cmd.acc          = acc;
    cmd.dec          = dec;
    cmd.pl           = pl;
    cmd.targetPosValue.resize(14, 0.0);
    for (size_t i = 0; i < pose.size() && i < 7; ++i) {
        cmd.targetPosValue[i] = pose[i];
    }
    return cmd;
}

// ============================================================
// 队列运动执行器
// ============================================================

/**
 * @brief 执行一轮队列运动
 * @param sock       socket 句柄
 * @param name       队列名称（用于打印）
 * @param fn_push    回调函数，在此函数中调用 queue_motion_push_back_*
 *                   回调会在队列模式开启后、发送前被调用
 * @return true 成功 / false 失败
 */
bool run_queue_motion(tl::SOCKETFD sock, const std::string& name,
                      std::function<bool(tl::SOCKETFD)> fn_push)
{
    print_separator("队列运动：" + name);

    // 1. 打开队列运动模式（会清空远端队列）
    tl::Result ret = tl::queue_motion_set_status(sock, true);
    print_result("tl::queue_motion_set_status(true)", ret);
    if (ret != tl::Result::SUCCESS) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 确认队列模式已开启
    {
        bool status = false;
        tl::queue_motion_get_status(sock, status);
        std::cout << "  [信息] 队列模式状态: " << (status ? "已开启" : "已关闭") << std::endl;
    }

    // 2. 插入运动指令
    std::cout << "  [信息] 插入运动指令..." << std::endl;
    if (!fn_push(sock)) {
        std::cerr << "[ERROR] 插入运动指令失败" << std::endl;
        tl::queue_motion_set_status(sock, false);
        return false;
    }

    // 3. 查询本地队列长度
    {
        int size = 0;
        ret = tl::queue_motion_size(sock, size);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [信息] 本地队列长度: " << size << " 条指令" << std::endl;
        }
    }

    // 4. 发送队列到控制器并等待执行完成
    std::cout << "  [信息] 发送队列到控制器..." << std::endl;
    ret = tl::queue_motion_send_to_controller(sock, 0, false);
    print_result("send_to_controller", ret);
    if (ret != tl::Result::SUCCESS) {
        tl::queue_motion_set_status(sock, false);
        return false;
    }

    std::cout << "  [信息] 等待运动执行完成..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    while (true) {
        int running_state = -1;
        ret = tl::get_robot_running_state(sock, running_state);
        if (ret == tl::Result::SUCCESS && running_state == 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "  [信息] 队列运动执行完毕" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 5. 关闭队列运动模式
    tl::queue_motion_set_status(sock, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
    // 解析命令行参数: ./ex_queue_motion [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    // ---- 获取 SDK 版本 ----
    {
        std::string version = tl::get_library_version();
        std::cout << "[信息] SDK 版本: " << version << std::endl;
    }

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    tl::SOCKETFD sock = tl::connect_robot(ip, "6001");
    if (sock < 0) {
        std::cerr << "[ERROR] 连接控制器失败" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket = " << sock << ")" << std::endl;

    // ---- 清错 ----
    tl::clear_error(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    tl::Result ret;

    // ---- 检查模式并确保伺服运行 ----
    {
        // 先获取当前模式
        int current_mode = -1;
        tl::Result ret = tl::get_current_mode(sock, current_mode);
        if (ret != tl::Result::SUCCESS || current_mode < 0) {
            std::cerr << "[ERROR] 获取当前模式失败: " << ret << std::endl;
            tl::disconnect_robot(sock);
            return -1;
        }
        std::cout << "  [信息] 当前模式: " << current_mode << " (0=示教 1=远程 2=运行)" << std::endl;

        if (current_mode == 0) {
            // 示教模式 → 先确保伺服就绪上电，再切换为运行模式
            int servo_state = -1;
            ret = tl::get_servo_state(sock, servo_state);
            if (ret != tl::Result::SUCCESS) {
                std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
                tl::disconnect_robot(sock);
                return -1;
            }
            std::cout << "  [信息] 伺服状态: " << servo_state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

            if (servo_state != 3) {
                // 伺服未运行 → 执行上电流程
                std::cout << "  [信息] 伺服未运行，执行上电流程..." << std::endl;
                ret = tl::set_servo_state(sock, 1);
                if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
                    std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
                    tl::disconnect_robot(sock);
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

            ret = tl::set_servo_poweron(sock);
            if (ret != tl::Result::SUCCESS) {
                std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
                tl::disconnect_robot(sock);
                return -1;
            }
            std::cout << "  [信息] 上电成功" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            } else {
                std::cout << "  [信息] 伺服已在运行状态" << std::endl;
            }

            // 切换为运行模式
            std::cout << "  [信息] 切换为运行模式..." << std::endl;
            ret = tl::set_current_mode(sock, 2);
            if (ret != tl::Result::SUCCESS) {
                std::cerr << "[ERROR] 切换运行模式失败: " << ret << std::endl;
                tl::disconnect_robot(sock);
                return -1;
            }
            std::cout << "  [信息] 已切换为运行模式" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else if (current_mode == 2) {
            // 已在运行模式，直接检查伺服状态
            int servo_state = -1;
            ret = tl::get_servo_state(sock, servo_state);
            if (ret != tl::Result::SUCCESS || servo_state != 3) {
                std::cerr << "[ERROR] 伺服未在运行状态 (servo_state=" << servo_state << ")" << std::endl;
                tl::disconnect_robot(sock);
                return -1;
            }
            std::cout << "  [信息] 已在运行模式，伺服运行中 (servo_state=3)" << std::endl;
        } else {
            std::cerr << "[ERROR] 不支持的模式: " << current_mode << "，期望示教(0)或运行(2)模式" << std::endl;
            tl::disconnect_robot(sock);
            return -1;
        }
    }

    // 延时 2 秒，保证状态完全就绪
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ---- 设置全局速度 ----
    // 控制器默认全局速度为 5%，设为 30% 提升运动效率
    print_separator("设置全局速度");
    ret = tl::set_speed(sock, 30);
    print_result("tl::set_speed(30)", ret);

    // ============================================================
    // 演示 1: 纯 MoveJ 队列运动
    // ============================================================
    run_queue_motion(sock, "纯 MoveJ", [](tl::SOCKETFD sock) -> bool {
        tl::Result r;

        std::cout << "  push_back_moveJ:  J1=30°" << std::endl;
        r = tl::queue_motion_push_back_moveJ(sock, make_movej_cmd({30, 0, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != tl::Result::SUCCESS) return false;

        std::cout << "  push_back_moveJ:  J1=30°, J2=20°" << std::endl;
        r = tl::queue_motion_push_back_moveJ(sock, make_movej_cmd({30, 20, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != tl::Result::SUCCESS) return false;

        std::cout << "  push_back_moveJ:  J1=0°, J2=20°" << std::endl;
        r = tl::queue_motion_push_back_moveJ(sock, make_movej_cmd({0, 20, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != tl::Result::SUCCESS) return false;

        std::cout << "  push_back_moveJ:  回零" << std::endl;
        r = tl::queue_motion_push_back_moveJ(sock, make_movej_cmd({0, 0, 0, 0, 0, 0}, 20));
        print_result("push_back_moveJ", r);
        if (r != tl::Result::SUCCESS) return false;

        return true;
    });

    // ============================================================
    // 演示 2: 纯 MoveL 队列运动
    // ============================================================
    run_queue_motion(sock, "纯 MoveL", [](tl::SOCKETFD sock) -> bool {
        tl::Result r;

        std::cout << "  push_back_moveL:  X=300, Y=0, Z=300" << std::endl;
        r = tl::queue_motion_push_back_moveL(sock, make_movel_cmd({300, 0, 300, 3.141, 0, 0}, 300));
        print_result("push_back_moveL", r);
        if (r != tl::Result::SUCCESS) return false;

        std::cout << "  push_back_moveL:  X=400, Y=100, Z=300" << std::endl;
        r = tl::queue_motion_push_back_moveL(sock, make_movel_cmd({400, 100, 300, 3.141, 0, 0}, 300));
        print_result("push_back_moveL", r);
        if (r != tl::Result::SUCCESS) return false;

        std::cout << "  push_back_moveL:  X=300, Y=200, Z=300" << std::endl;
        r = tl::queue_motion_push_back_moveL(sock, make_movel_cmd({300, 200, 300, 3.141, 0, 0}, 300));
        print_result("push_back_moveL", r);
        if (r != tl::Result::SUCCESS) return false;

        std::cout << "  push_back_moveL:  回起始点" << std::endl;
        r = tl::queue_motion_push_back_moveL(sock, make_movel_cmd({229.882, 0, 358.680, 3.141, 0, 0}, 200));
        print_result("push_back_moveL", r);
        if (r != tl::Result::SUCCESS) return false;

        return true;
    });

    // ============================================================
    // 演示 3: MoveJ + MoveL 混合队列运动
    // ============================================================
    run_queue_motion(sock, "MoveJ + MoveL 混合", [](tl::SOCKETFD sock) -> bool {
        tl::Result r;

        // 先用 MoveJ 调整姿态
        std::cout << "  push_back_moveJ:  J1=45°（调整关节姿态）" << std::endl;
        r = tl::queue_motion_push_back_moveJ(sock, make_movej_cmd({45, 0, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != tl::Result::SUCCESS) return false;

        // 再用 MoveL 走直线
        std::cout << "  push_back_moveL:  X=400, Y=0, Z=300" << std::endl;
        r = tl::queue_motion_push_back_moveL(sock, make_movel_cmd({400, 0, 300, 3.141, 0, 0}, 300));
        print_result("push_back_moveL", r);
        if (r != tl::Result::SUCCESS) return false;

        // MoveJ 换向
        std::cout << "  push_back_moveJ:  J1=-45°（换向）" << std::endl;
        r = tl::queue_motion_push_back_moveJ(sock, make_movej_cmd({-45, 0, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != tl::Result::SUCCESS) return false;

        // MoveL 走另一条直线
        std::cout << "  push_back_moveL:  X=300, Y=200, Z=300" << std::endl;
        r = tl::queue_motion_push_back_moveL(sock, make_movel_cmd({300, 200, 300, 3.141, 0, 0}, 300));
        print_result("push_back_moveL", r);
        if (r != tl::Result::SUCCESS) return false;

        // MoveJ 回零
        std::cout << "  push_back_moveJ:  回零" << std::endl;
        r = tl::queue_motion_push_back_moveJ(sock, make_movej_cmd({0, 0, 0, 0, 0, 0}, 20));
        print_result("push_back_moveJ", r);
        if (r != tl::Result::SUCCESS) return false;

        return true;
    });

    // ============================================================
    // 演示 4: 队列运动控制（暂停/恢复/停止保持上电）
    // ============================================================
    {
        print_separator("队列运动控制：暂停/恢复/停止");

        // 辅助：等待队列剩余数减少到 target_len
        auto wait_queue_len = [&](int target_len) {
            int len = -1;
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                tl::queue_motion_get_queuelen(sock, len);
            } while (len > target_len && len != -1);
        };

        // 1. 开启队列模式
        tl::Result r = tl::queue_motion_set_status(sock, true);
        print_result("tl::queue_motion_set_status(true)", r);
        if (r != tl::Result::SUCCESS) return -1;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 2. 插入 4 条 J5 摆动指令（简单清晰，便于观察控制效果）
        std::cout << "  [信息] 插入 4 条 MoveJ 指令（J5 摆动）" << std::endl;
        std::vector<std::vector<double>> targets = {
            {0, 0, 0, 0, -30, 0, 0},    // J5 → -30°
            {0, 0, 0, 0,   0, 0, 0},    // J5 → 0°
            {0, 0, 0, 0, -30, 0, 0},    // J5 → -30°
            {0, 0, 0, 0,   0, 0, 0},    // J5 → 0°
        };
        for (size_t i = 0; i < targets.size(); ++i) {
            auto cmd = make_movej_cmd(targets[i], 30);
            r = tl::queue_motion_push_back_moveJ(sock, cmd);
            std::cout << "  第" << (i + 1) << "条: push_back_moveJ (J5→" << targets[i][4] << "°)"
                      << (r == tl::Result::SUCCESS ? " 成功" : " 失败") << std::endl;
        }

        // 3. 发送队列到控制器
        std::cout << "  [信息] 发送队列到控制器..." << std::endl;
        r = tl::queue_motion_send_to_controller(sock, 0);
        print_result("send_to_controller", r);
        if (r != tl::Result::SUCCESS) return -1;

        // 4. 第 1 条执行中 → 暂停
        std::cout << "  [信息] 第 1 条执行中，等待 2 秒后暂停..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        r = tl::queue_motion_suspend(sock);
        print_result("暂停 (suspend)", r);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 5. 恢复
        r = tl::queue_motion_restart(sock);
        print_result("恢复 (restart)", r);
        std::cout << "  [信息] 等待第 1 条完成..." << std::endl;
        wait_queue_len(3);
        std::cout << "  [信息] 第 1 条完成" << std::endl;

        // 6. 等待第 2 条完成
        std::cout << "  [信息] 等待第 2 条完成..." << std::endl;
        wait_queue_len(2);
        std::cout << "  [信息] 第 2 条完成" << std::endl;

        // 7. 第 3 条执行中 → 暂停
        std::this_thread::sleep_for(std::chrono::seconds(1));
        r = tl::queue_motion_suspend(sock);
        print_result("再次暂停 (suspend)", r);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 8. 恢复
        r = tl::queue_motion_restart(sock);
        print_result("再次恢复 (restart)", r);
        std::cout << "  [信息] 等待第 3 条完成..." << std::endl;
        wait_queue_len(1);
        std::cout << "  [信息] 第 3 条完成" << std::endl;

        // 9. 第 4 条执行中 → 停止（保持上电）
        std::this_thread::sleep_for(std::chrono::seconds(2));
        r = tl::queue_motion_stop_not_power_off(sock);
        print_result("停止保持上电 (stop_not_power_off)", r);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 10. 关闭队列模式
        tl::queue_motion_set_status(sock, false);
        std::cout << "  [信息] 队列运动控制演示完毕" << std::endl;
    }

    // ---- 切回示教模式并断开连接 ----
    print_separator("切回示教模式 + 断开连接");
    ret = tl::set_current_mode(sock, 0);
    print_result("tl::set_current_mode(0) 切回示教模式", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    tl::disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 队列运动示例程序运行完毕" << std::endl;
    return 0;
}
