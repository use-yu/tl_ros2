/**
 * ex_servo.cpp
 * @brief 伺服状态与速度控制示例程序
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 可用命令行参数 -ip 指定控制器 IP（默认 192.168.1.13）
 *   - 运行模式：示教模式
 *   - 伺服状态机: 0=停止 → 1=就绪 → 3=运行（上电后）
 *   - tl::set_servo_state(1) 必须先于 set_servo_poweron 调用
 *   - set_servo_poweroff 只能在 state=3 时调用，成功后回到 state=1
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_servo
 */

#include "tl_api.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>

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

/// @brief 获取伺服状态文本描述
const char* servo_state_text(int state)
{
    switch (state) {
        case 0:  return "0=停止";
        case 1:  return "1=就绪";
        case 2:  return "2=报警";
        case 3:  return "3=运行";
        default: return "未知";
    }
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 解析命令行参数: ./ex_servo [-ip 192.168.1.13]，默认 192.168.1.13
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

    // 声明 ret 变量供后续使用
    tl::Result ret;

    // ============================================================
    // 1. 查询初始伺服状态
    // ============================================================
    print_separator("查询初始伺服状态");

    {
        int state = -1;
        ret = tl::get_servo_state(sock, state);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [信息] 初始伺服状态: " << servo_state_text(state) << std::endl;
        } else {
            print_result("get_servo_state", ret);
        }
    }

    // ============================================================
    // 2. 切换为示教模式
    // ============================================================
    // 伺服状态操作通常在示教模式下进行（0=示教）
    print_separator("切换示教模式");
    ret = tl::set_current_mode(sock, 0);
    print_result("tl::set_current_mode(0)", ret);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ============================================================
    // 3. 清错处理（在设置伺服状态前先清除控制器错误和报警）
    // ============================================================
    print_separator("清错");
    ret = tl::clear_error(sock);
    print_result("clear_error", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    ret = tl::set_servo_poweroff(sock);
    print_result("set_servo_poweroff", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 检查伺服状态
    {
        int state = -1;
        ret = tl::get_servo_state(sock, state);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
            tl::disconnect_robot(sock);
            return -1;
        }
        std::cout << "  [信息] 当前伺服状态: " << servo_state_text(state) << std::endl;

        if (state == 2) {
            // state=2 为报警状态，此时无法上电
            std::cerr << "[ERROR] 伺服处于报警状态，无法继续上电，请排查报警原因后重试" << std::endl;
            tl::disconnect_robot(sock);
            return -1;
        }
    }

    // ============================================================
    // 4. 设置伺服就绪 → 上电 → 验证状态变化
    // ============================================================
    print_separator("伺服上电流程");

    // 4a. 设置伺服就绪（state=1）
    std::cout << "[步骤 4a] 设置伺服就绪 (set_servo_state=1)" << std::endl;
    ret = tl::set_servo_state(sock, 1);
    if (ret == tl::Result::OPERATION_NOT_ALLOWED) {
        std::cout << "  [信息] 机械臂已处于就绪或运行状态，无需设置" << std::endl;
    } else {
        print_result("tl::set_servo_state(1)", ret);
        if (ret != tl::Result::SUCCESS) {
            tl::disconnect_robot(sock);
            return -1;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 验证当前状态
    {
        int state = -1;
        tl::get_servo_state(sock, state);
        std::cout << "  [信息] 当前伺服状态: " << servo_state_text(state) << std::endl;
    }

    // 4b. 上电使能（state=1 → state=3）
    std::cout << "\n[步骤 4b] 上电使能 (set_servo_poweron)" << std::endl;
    ret = tl::set_servo_poweron(sock);
    print_result("set_servo_poweron", ret);
    if (ret != tl::Result::SUCCESS) {
        tl::disconnect_robot(sock);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 验证上电成功后的状态应为 3=运行
    {
        int state = -1;
        ret = tl::get_servo_state(sock, state);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [信息] 上电后伺服状态: " << servo_state_text(state);
            if (state == 3) {
                std::cout << "（上电成功）";
            }
            std::cout << std::endl;
        }
    }

    // ============================================================
    // 4. 速度查询与设置
    // ============================================================
    print_separator("速度查询与设置");

    // 4a. 查询当前速度
    {
        int speed = -1;
        ret = tl::get_speed(sock, speed);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [信息] 当前全局速度: " << speed << "%" << std::endl;
        } else {
            print_result("get_speed", ret);
        }
    }

    // 4b. 设置速度并验证
    // set_speed 范围 0<speed≤100，设置后影响后续所有运动的速度上限
    int test_speeds[] = {50, 80, 30};
    for (int spd : test_speeds) {
        std::cout << "\n[步骤 4] 设置全局速度: " << spd << "%" << std::endl;
        ret = tl::set_speed(sock, spd);
        print_result("set_speed", ret);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        int current_speed = -1;
        ret = tl::get_speed(sock, current_speed);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [信息] 读取确认: 当前速度 = " << current_speed << "%" << std::endl;
        }
    }

    // ============================================================
    // 5. 下电 → 停止 → 验证状态变化
    // ============================================================
    print_separator("伺服下电流程");

    // 5a. 下电（state=3 → state=1）
    std::cout << "[步骤 5a] 下电 (set_servo_poweroff)" << std::endl;
    ret = tl::set_servo_poweroff(sock);
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [成功] set_servo_poweroff" << std::endl;
    } else if (ret == tl::Result::RECEIVE_FAILED) {
        // 下电后网络可能自动断开，-1 视为正常
        std::cout << "  [信息] set_servo_poweroff 返回 -1（网络可能已断开）" << std::endl;
    } else {
        print_result("set_servo_poweroff", ret);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 5b. 验证下电后的伺服状态
    {
        int state = -1;
        ret = tl::get_servo_state(sock, state);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [信息] 下电后伺服状态: " << servo_state_text(state) << std::endl;
        }
    }

    // 5c. 设置伺服停止（state=1 → state=0）
    std::cout << "\n[步骤 5c] 设置伺服停止 (set_servo_state=0)" << std::endl;
    ret = tl::set_servo_state(sock, 0);
    if (ret == tl::Result::OPERATION_NOT_ALLOWED) {
        std::cout << "  [信息] 当前状态不允许直接设为停止" << std::endl;
    } else {
        print_result("tl::set_servo_state(0)", ret);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 最终状态确认
    {
        int state = -1;
        ret = tl::get_servo_state(sock, state);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [信息] 最终伺服状态: " << servo_state_text(state) << std::endl;
        }
    }

    // // ⚠️ 以下功能不建议测试，请在厂家指导下使用
    // // ============================================================
    // // 6. 恢复机械臂默认 DH 参数
    // // ============================================================
    // print_separator("恢复机械臂默认 DH 参数");

    // ret = restore_default_param_DH(sock, 1);
    // print_result("restore_default_param_DH (robot=1)", ret);
    // std::cout << std::endl;

    // // ============================================================
    // // 7. 设置关节参数（遍历关节 1~6）
    // // ============================================================
    // print_separator("设置关节参数");

    // {
    //     RobotJointParam joint_param;
    //     joint_param.reducRatio = 1;          // 减速比
    //     joint_param.encoderResolution = 2;    // 编码器位数
    //     joint_param.posSWLimit = 3;           // 轴正限位
    //     joint_param.negSWLimit = -4;          // 轴反限位
    //     joint_param.ratedRotSpeed = 5;        // 电机额定正转速
    //     joint_param.ratedDeRotSpeed = -6;     // 电机额定反转速
    //     joint_param.maxRotSpeed = 2;          // 电机最大正转速
    //     joint_param.maxDeRotSpeed = -2;       // 电机最大反转速
    //     joint_param.ratedVel = 300;           // 额定正速度 (度/秒)
    //     joint_param.deRatedVel = -300;        // 额定反速度 (度/秒)
    //     joint_param.maxAcc = 2;               // 最大加速度 (度/秒^2)
    //     joint_param.maxDecel = -2;            // 最大减速度 (度/秒^2)
    //     joint_param.direction = -1;           // 模型方向: 1=正向, -1=反向

    //     // 遍历关节 1~6
    //     for (int joint_id = 1; joint_id <= 6; ++joint_id) {
    //         std::cout << "--- 关节 " << joint_id << " ---" << std::endl;
    //         ret = set_robot_joint_param(sock, joint_id, joint_param);
    //         if (ret == Result::SUCCESS) {
    //             std::cout << "  [成功] 设置关节 " << joint_id << " 参数" << std::endl;
    //         } else {
    //             std::cout << "  [失败] 设置关节 " << joint_id << " 参数，错误码: " << ret << std::endl;
    //         }
    //     }

    //     // 打印参数（仅供示意，TODO: 各关节应根据实际需求独立设置参数值）
    //     std::cout << "\n========== 机器人关节参数设置 (仅供参考) ==========" << std::endl;
    //     std::cout << "  减速比: " << joint_param.reducRatio << std::endl;
    //     std::cout << "  编码器位数: " << joint_param.encoderResolution << std::endl;
    //     std::cout << "  正限位: " << joint_param.posSWLimit << " 度" << std::endl;
    //     std::cout << "  反限位: " << joint_param.negSWLimit << " 度" << std::endl;
    //     std::cout << "  额定正转速: " << joint_param.ratedRotSpeed << " RPM" << std::endl;
    //     std::cout << "  额定反转速: " << joint_param.ratedDeRotSpeed << " RPM" << std::endl;
    //     std::cout << "  最大正转速: " << joint_param.maxRotSpeed << " RPM" << std::endl;
    //     std::cout << "  最大反转速: " << joint_param.maxDeRotSpeed << " RPM" << std::endl;
    //     std::cout << "  额定正速度: " << joint_param.ratedVel << " 度/秒" << std::endl;
    //     std::cout << "  额定反速度: " << joint_param.deRatedVel << " 度/秒" << std::endl;
    //     std::cout << "  最大加速度: " << joint_param.maxAcc << " 度/秒^2" << std::endl;
    //     std::cout << "  最大减速度: " << joint_param.maxDecel << " 度/秒^2" << std::endl;
    //     std::cout << "  模型方向: " << (joint_param.direction == 1 ? "正向" : "反向") << std::endl;
    //     std::cout << "================================================" << std::endl;
    // }
    // std::cout << std::endl;

    // // ============================================================
    // // 8. 设置笛卡尔参数
    // // ============================================================
    // print_separator("设置笛卡尔参数");

    // {
    //     CartesianParam cart_param;
    //     cart_param.maxSpeed = 500.0;         // 最大线速度 (mm/s)
    //     cart_param.maxAcc = 5.0;             // 最大线加速度 (mm/s^2)
    //     cart_param.maxDec = -5.0;            // 最大线减速度 (mm/s^2)
    //     cart_param.maxAttitudeVel = 180.0;   // 最大姿态角速度 (deg/s)
    //     cart_param.speedLimitMode = 1;       // 速度限制方式: 0=位姿, 1=位置

    //     std::cout << std::fixed << std::setprecision(2);
    //     ret = set_cartesian_params(sock, cart_param);
    //     if (ret == Result::SUCCESS) {
    //         std::cout << "[成功] 设置笛卡尔参数" << std::endl;
    //         std::cout << "  最大线速度: " << cart_param.maxSpeed << " mm/s" << std::endl;
    //         std::cout << "  最大线加速度: " << cart_param.maxAcc << " mm/s^2" << std::endl;
    //         std::cout << "  最大线减速度: " << cart_param.maxDec << " mm/s^2" << std::endl;
    //         std::cout << "  最大姿态角速度: " << cart_param.maxAttitudeVel << " deg/s" << std::endl;
    //         std::cout << "  速度限制方式: " << (cart_param.speedLimitMode == 0 ? "位姿" : "位置") << std::endl;
    //     } else {
    //         std::cerr << "[ERROR] 设置笛卡尔参数失败，错误码: " << ret << std::endl;
    //     }
    // }
    // std::cout << std::endl;

    // ============================================================
    // 9. 配置有线网口 IP 地址（危险操作，默认注释）
    // ============================================================
    // ⚠️  危险: 错误配置会导致控制器网络断连，仅能在已知网络参数时使用
    //    取消下面代码的注释前，请确认 IP/网关/DNS 参数正确
    //
    // print_separator("配置有线网口 IP 地址");
    // ret = set_controller_ip(sock, "eth0", "192.168.1.14", "192.168.1.1", "8.8.8.8");
    // print_result("set_controller_ip", ret);
    // print_separator("配置有线网口 IP 地址完成, 新IP: 192.168.1.14");
    // std::cout << std::endl;

    // ============================================================
    // 10. 恢复网络出厂设置（危险操作，默认注释）
    // ============================================================
    // ⚠️  危险: 恢复网络出厂设置会重置控制器 IP 为 192.168.1.13
    //    取消下面代码的注释前，请确认当前网络环境允许 IP 变更
    //
    // print_separator("恢复网络出厂设置");
    // ret = restore_network_factory_settings(sock);
    // print_result("restore_network_factory_settings", ret);
    // std::cout << std::endl;

    // ============================================================
    // 11. 断开连接
    // ============================================================
    print_separator("断开连接");
    tl::disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 伺服状态示例程序运行完毕" << std::endl;
    return 0;
}
