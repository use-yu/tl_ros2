/**
 * ex_collision_detect.cpp
 * @brief 碰撞检测演示示例: 使用 get_collision/set_collision 获取与设置碰撞防护参数 → 慢速运动并触发碰撞 → 检测碰撞 → 清除错误并自动恢复
 *        演示流程: 连接 → 注册错误回调 → 上电 → get_collision 查询碰撞参数 → set_collision 设置（系数=98、响应时间沿用原值）
 *                  → MoveJ 慢速运动 → 用户阻挡触发碰撞 → 检测到碰撞 → 清错自动恢复（clear_error → 下电 → 就绪 → 上电）
 * @attention 运行前请确保:
 *            1. 机械臂供电正常、网络通信正常（默认 IP 192.168.1.13）
 *            2. 碰撞检测功能需在示教器上开启，否则阻挡机械臂不会触发碰撞保护
 *            3. get_collision/set_collision 为 24.03 版本新增接口，旧版本固件会返回失败（示例仍可继续演示碰撞检测）
 *            4. 本示例 set_collision 将所有轴碰撞检测系数设为 98；伺服执行响应时间不建议手动设置
 *               （影响碰撞保护实时性），沿用查询到的原值；系数对应的灵敏方向请参照控制器说明
 * @warning 本示例会真实驱动机械臂运动，且需要人为阻挡机械臂来触发碰撞检测！
 *          碰撞瞬间机械臂会急停，请务必控制阻挡力度与运动速度（示例已用 10% 低速），
 *          并确认机械臂周围无人员/贵重设备，防止碰撞造成损伤
 * @note 运行步骤
 *       编译: cd examples/cpp && cmake --preset linux-native && cmake --build --preset linux-native
 *       运行: ./build/linux-native/ex_collision_detect
 *       运行中: 程序提示"请阻挡机械臂"后，用手或软质障碍物轻挡机械臂运动路径即可触发碰撞，
 *               碰撞触发后程序自动清除错误并重新上电
 */

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "tl_api.h"


#ifdef _WIN32
#include <windows.h>
#endif

// 控制器连接参数（与文件头运行说明一致）
static std::string g_robot_ip = "192.168.1.13";   // 默认 IP，可用命令行参数 -ip 覆盖
constexpr const char* kCtrlPort = "6001";

// ============================================================
// 全局碰撞/错误标志（回调在 SDK 内部线程触发，用原子变量保证跨线程可见）
// ============================================================

std::atomic<bool> g_error_received{false};   // 收到错误消息（messageType==1）

// ============================================================
// 错误/警告回调
// ============================================================

/// @brief 错误/警告消息回调（由 SDK 内部线程调用，不要做耗时操作）
/// @param messageType 消息类型（0=警告 1=错误）
/// @param message     消息内容
/// @param messageCode 消息码
void on_error_or_warning(int messageType, const char* message, int messageCode)
{
    if (messageType == 1) {
        g_error_received.store(true);
    }
    const char* type_str = (messageType == 0) ? "警告" : "错误";
    std::cout << "\n>>> [回调触发] " << type_str
              << " | code=" << messageCode
              << " | " << (message != nullptr ? message : "") << std::endl;
}

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

/// @brief 处理下电后的自动断连: 若 socket 已断开则重新连接控制器
/// @note 下电可能触发控制器自动断开连接（SDK 返回 -1 也视为下电成功），此后需重连才能继续查询/上电
/// @return true 连接有效
bool ensure_connected(tl::SOCKETFD& socket_fd)
{
    int state = -1;
    if (tl::get_servo_state(socket_fd, state) == tl::Result::SUCCESS) {
        return true;    // 连接仍有效
    }

    std::cout << "  [信息] 连接已断开（下电自动断连），重新连接控制器..." << std::endl;
    // 幂等清理旧 socket（死连接上 disconnect 返回 -1 视为成功），避免 fd 泄漏与控制器残留会话
    if (socket_fd >= 0) {
        tl::disconnect_robot(socket_fd);
    }
    // 用临时变量接收新 fd: 重连失败时保持原值，避免把负值传给下游接口
    tl::SOCKETFD new_fd = tl::connect_robot(g_robot_ip, kCtrlPort);
    if (new_fd < 0) {
        std::cerr << "[ERROR] 重连失败" << std::endl;
        return false;
    }
    socket_fd = new_fd;
    std::cout << "  [信息] 重连成功 (socket=" << socket_fd << ")" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // 注意: 重连后错误回调未重新注册，碰撞检测阶段（恢复后）仅靠轮询伺服状态兜底
    return true;
}

/// @brief 清除错误并执行标准两步上电流程
/// @note 入参为引用: 下电可能触发控制器自动断连，函数内部会重连并更新 socket
/// @return true 上电成功（伺服状态=3 运行）
bool enable_servo(tl::SOCKETFD& socket_fd)
{
    std::cout << "--- 伺服上电 ---" << std::endl;

    // 1. 先清错、下电复位，确保控制器状态同步
    tl::clear_error(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // 下电可能触发控制器自动断连，断连则重连后再查询状态
    if (!ensure_connected(socket_fd)) {
        return false;
    }

    // 2. 检查当前伺服状态
    int state = -1;
    auto ret = tl::get_servo_state(socket_fd, state);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
        return false;
    }
    std::cout << "  [信息] 当前伺服状态: " << state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

    if (state == 2) {
        // 报警状态: 尝试一次 clear_error；部分报警（如运动目标不可达）SDK 无法清除，需在示教器上确认清除
        std::cout << "  [信息] 伺服处于报警状态，尝试清除错误..." << std::endl;
        tl::clear_error(socket_fd);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ret = tl::get_servo_state(socket_fd, state);
        if (ret != tl::Result::SUCCESS) {
            if (!ensure_connected(socket_fd)) return false;
            ret = tl::get_servo_state(socket_fd, state);
        }
        if (ret == tl::Result::SUCCESS && state == 2) {
            std::cerr << "[ERROR] 报警无法通过 SDK 清除（servo_state=2），" << std::endl;
            std::cerr << "        请在示教器上确认并清除报警后重新运行本示例" << std::endl;
            return false;
        }
        std::cout << "  [信息] 报警已清除，伺服状态: " << state << std::endl;
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
// 碰撞后自动恢复
// ============================================================

/// @brief 碰撞触发后的自动恢复: 清错 → 下电 → 就绪 → 上电
/// @note 碰撞后控制器可能处于报警状态，必须先清错再下电复位，之后重新执行两步上电；
///       下电可能自动断连，函数内部会重连并更新 socket
/// @return true 恢复成功（伺服状态=3 运行）
bool recover_after_collision(tl::SOCKETFD& socket_fd)
{
    std::cout << "\n--- 自动恢复流程 ---" << std::endl;
    std::cout << "  [信息] 步骤 1/4: clear_error 清除碰撞报警..." << std::endl;
    auto ret = tl::clear_error(socket_fd);
    print_result("clear_error", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::cout << "  [信息] 步骤 2/4: set_servo_poweroff 下电复位（停止运动并解除报警锁定）..." << std::endl;
    ret = tl::set_servo_poweroff(socket_fd);
    print_result("set_servo_poweroff", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    // 下电可能自动断连（返回 -1 也视为下电成功），断连则重连后再查询/上电
    if (!ensure_connected(socket_fd)) {
        return false;
    }

    int state = -1;
    ret = tl::get_servo_state(socket_fd, state);
    print_result("get_servo_state", ret);
    std::cout << "  [信息] 下电后伺服状态: " << state << std::endl;

    if (state == 2) {
        std::cerr << "[ERROR] 下电后伺服仍处于报警状态（报警未清除），请排查碰撞原因后重试" << std::endl;
        return false;
    }

    std::cout << "  [信息] 步骤 3/4: set_servo_state(1) 伺服就绪..." << std::endl;
    if (state == 0) {
        ret = tl::set_servo_state(socket_fd, 1);
        if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
            std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "  [信息] 步骤 4/4: set_servo_poweron 上电使能..." << std::endl;
    ret = tl::set_servo_poweron(socket_fd);
    print_result("set_servo_poweron", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 确认恢复结果
    ret = tl::get_servo_state(socket_fd, state);
    print_result("get_servo_state", ret);
    if (ret == tl::Result::SUCCESS && state == 3) {
        std::cout << "  [信息] 碰撞后自动恢复成功，机械臂已重新上电（servo_state = " << state << "）" << std::endl;
        return true;
    }
    std::cerr << "[ERROR] 自动恢复失败（servo_state = " << state << "）" << std::endl;
    return false;
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // 解析命令行参数: ./ex_collision_detect [-ip 192.168.1.13]，默认 192.168.1.13
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            g_robot_ip = argv[i + 1];
            break;
        }
    }

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    tl::SOCKETFD sock = tl::connect_robot(g_robot_ip, kCtrlPort);
    if (sock < 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket=" << sock << ")" << std::endl;

    tl::Result ret;

    // ---- 切换示教模式 ----
    ret = tl::set_current_mode(sock, 0);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 切换示教模式失败: " << ret << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 注册错误/警告回调（用于判断碰撞触发）----
    print_separator("注册错误回调");
    ret = tl::set_receive_error_or_warnning_message_callback(sock, on_error_or_warning);
    print_result("set_receive_error_or_warnning_message_callback", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 注册回调失败" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }

    // ---- 上电 ----
    if (!enable_servo(sock)) {
        tl::disconnect_robot(sock);
        return -1;
    }

    // ============================================================
    // 1. 查询当前碰撞防护参数（get_collision）
    // ============================================================
    print_separator("查询当前碰撞参数");
    tl::CollisionSafeParam cur_param;
    ret = tl::get_collision(sock, cur_param);
    print_result("get_collision", ret);
    bool param_valid = false;
    if (ret == tl::Result::SUCCESS) {
        print_vector("  碰撞检测系数(每轴)", cur_param.safe_coeff);
        print_vector("  伺服执行响应时间 [ms](每轴)", cur_param.servo_execution_delay);
        if (cur_param.safe_coeff.empty() || cur_param.servo_execution_delay.empty()) {
            std::cout << "  [注意] 控制器返回参数为空，当前固件可能不支持 get_collision（24.03+ 版本支持），示例仍可继续" << std::endl;
        } else {
            param_valid = true;
        }
    }

    // ============================================================
    // 2. 设置碰撞防护参数（set_collision）
    // ============================================================
    print_separator("设置碰撞参数");
    std::cout << "  [信息] 本示例演示 set_collision：将所有轴碰撞检测系数（safe_coeff）设为 98，" << std::endl;
    std::cout << "        伺服执行响应时间（servo_execution_delay）不建议手动设置，沿用查询到的原值" << std::endl;
    std::cout << "  [注意] 系数大小对应的灵敏方向请参照控制器说明；设置后碰撞演示前请确认保护行为符合预期" << std::endl;
    if (param_valid) {
        tl::CollisionSafeParam set_param = cur_param;
        // 演示设置: 所有轴碰撞检测系数统一设为 98（系数无量纲）
        set_param.safe_coeff.assign(set_param.safe_coeff.size(), 98.0);
        // 不建议设置伺服执行响应时间（servo_execution_delay）: 该参数影响碰撞保护实时性，
        // 错误值可能导致保护响应延迟/失效，故沿用 get_collision 查询到的原值，由控制器管理
        ret = tl::set_collision(sock, set_param);
        print_result("set_collision(系数=98, 响应时间沿用原值)", ret);
        if (ret == tl::Result::SUCCESS) {
            // 读回验证: 再次 get_collision 对比设置值
            tl::CollisionSafeParam verify_param;
            ret = tl::get_collision(sock, verify_param);
            print_result("get_collision(读回验证)", ret);
            if (ret == tl::Result::SUCCESS) {
                bool same = (verify_param.safe_coeff == set_param.safe_coeff) &&
                            (verify_param.servo_execution_delay == set_param.servo_execution_delay);
                std::cout << "  [信息] 读回参数与设置值" << (same ? "一致" : "不一致") << std::endl;
                if (!same) {
                    std::cout << "  [信息] 不一致可能由固件内部量化/缩放处理导致，不一定是设置失败" << std::endl;
                    print_vector("  读回碰撞检测系数", verify_param.safe_coeff);
                    print_vector("  读回伺服执行响应时间 [ms]", verify_param.servo_execution_delay);
                }
            }
        } else {
            std::cout << "  [注意] set_collision 失败（当前固件可能不支持，24.03+ 版本支持），示例仍可继续" << std::endl;
        }
    } else {
        std::cout << "  [注意] 查询失败，跳过设置演示" << std::endl;
    }

    // ============================================================
    // 3. 慢速运动触发碰撞 + 碰撞检测 + 自动恢复
    // ============================================================
    print_separator("碰撞触发演示");
    std::cout << "  [信息] 机械臂将以 10% 速度沿基座 X 方向运动 100mm" << std::endl;
    std::cout << "  [信息] 请确认已在示教器上开启碰撞检测（若未开启，阻挡不会触发碰撞保护）" << std::endl;
    std::cout << "  [信息] 请用手或软质障碍物轻挡机械臂运动路径，触发碰撞检测（碰撞后机械臂会急停）" << std::endl;
    std::cout << "  [信息] 5 秒后开始运动..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 获取当前基坐标，作为运动起点
    std::vector<double> cur_cart;
    ret = tl::get_current_position(sock, tl::Coord::BASE /* 基坐标系 */, cur_cart);
    print_result("get_current_position(基坐标)", ret);
    if (ret != tl::Result::SUCCESS || cur_cart.size() < 6) {
        std::cerr << "[ERROR] 获取当前基坐标失败" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }
    print_vector("当前基坐标 [mm, rad]", cur_cart);

    // 目标点: 沿基座 X 方向 +100mm，姿态不变（距离过远可能超出工作空间导致目标不可达）
    tl::MoveCmd move_cmd;
    move_cmd.targetPosType  = tl::PosType::data;
    move_cmd.coord          = tl::Coord::BASE;   // 基坐标系
    move_cmd.velocity       = 10;               // 慢速（百分比），碰撞演示更安全
    move_cmd.acc            = 5;
    move_cmd.dec            = 5;
    move_cmd.pl             = 0;
    move_cmd.targetPosValue = cur_cart;
    move_cmd.targetPosValue[0] += 100.0;        // X 方向前进 100mm（基坐标下标 0=X）
    if (move_cmd.targetPosValue.size() < 7) {
        move_cmd.targetPosValue.resize(7, 0.0);
    }
    print_vector("目标基坐标 [mm, rad]", move_cmd.targetPosValue);

    ret = tl::robot_movej(sock, move_cmd);
    print_result("robot_movej(慢速前进)", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] MoveJ 下发失败" << std::endl;
        tl::disconnect_robot(sock);
        return -1;
    }

    // 复位碰撞标志: 此前阶段（上电/设置参数等）可能收到过错误消息，避免误判为本次碰撞
    g_error_received.store(false);

    // ---- 碰撞检测: 轮询伺服状态(2=报警) 或 等待错误回调触发，超时 60s ----
    constexpr int kDetectTimeoutMs = 60000;     // 碰撞检测超时 [ms]
    constexpr int kPollStepMs = 200;            // 轮询周期 [ms]
    bool collided = false;
    std::cout << "\n  [信息] 碰撞检测中（轮询伺服状态 + 错误回调）..." << std::endl;
    for (int waited = 0; waited < kDetectTimeoutMs; waited += kPollStepMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollStepMs));

        // 判断条件 1: 错误回调收到错误消息
        if (g_error_received.load()) {
            std::cout << "  [信息] 检测到碰撞: 错误回调已收到错误消息" << std::endl;
            collided = true;
            break;
        }

        // 判断条件 2: 伺服状态变为报警(2)
        int state = -1;
        if (tl::get_servo_state(sock, state) == tl::Result::SUCCESS && state == 2) {
            std::cout << "  [信息] 检测到碰撞: 伺服状态变为报警(" << state << ")" << std::endl;
            collided = true;
            break;
        }
    }

    if (collided) {
        std::cout << "\n========== 碰撞已触发 ==========" << std::endl;
        std::cout << "  [信息] 碰撞后机械臂已急停，开始清除错误并自动恢复..." << std::endl;

        // 自动恢复: 清错 → 下电 → 就绪 → 上电
        if (recover_after_collision(sock)) {
            std::cout << "  [信息] 恢复完成，机械臂可继续执行作业" << std::endl;

            // ---- 恢复后继续运行几秒: 再次下发 MoveJ，验证碰撞恢复后仍可正常运动 ----
            std::cout << "\n--- 恢复后继续运动 ---" << std::endl;
            std::cout << "  [信息] 碰撞恢复后，机械臂沿 X 方向再前进 50mm（运行约 5 秒）" << std::endl;
            std::vector<double> resume_pos;
            ret = tl::get_current_position(sock, tl::Coord::BASE /* 基坐标系 */, resume_pos);
            print_result("get_current_position(基坐标)", ret);
            if (ret == tl::Result::SUCCESS && resume_pos.size() >= 6) {
                if (resume_pos.size() < 7) {
                    resume_pos.resize(7, 0.0);
                }
                resume_pos[0] += 50.0;    // 沿基座 X 方向继续前进 50mm（短距离，避免超出工作空间）

                tl::MoveCmd resume_cmd;
                resume_cmd.targetPosType  = tl::PosType::data;
                resume_cmd.coord          = tl::Coord::BASE;   // 基坐标系
                resume_cmd.velocity       = 10;   // 慢速（百分比），与碰撞演示速度一致
                resume_cmd.acc            = 5;
                resume_cmd.dec            = 5;
                resume_cmd.pl             = 0;
                resume_cmd.targetPosValue = resume_pos;
                print_vector("恢复后目标基坐标 [mm, rad]", resume_cmd.targetPosValue);

                ret = tl::robot_movej(sock, resume_cmd);
                print_result("robot_movej(恢复后继续运动)", ret);
                if (ret == tl::Result::SUCCESS) {
                    std::cout << "  [信息] 继续运行 5 秒..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    // 运行期间若再次被阻挡，伺服会转报警（碰撞保护），给出提示
                    int state = -1;
                    if (tl::get_servo_state(sock, state) == tl::Result::SUCCESS && state == 2) {
                        std::cerr << "[ERROR] 恢复后运动期间再次触发碰撞（伺服报警），请移开障碍物后重新运行示例" << std::endl;
                    }
                } else {
                    std::cerr << "[ERROR] 恢复后运动下发失败，跳过" << std::endl;
                }
            } else {
                std::cerr << "[ERROR] 获取当前基坐标失败，跳过恢复后运动" << std::endl;
            }
        } else {
            std::cerr << "[ERROR] 自动恢复失败，请手动检查控制器报警" << std::endl;
        }
    } else {
        std::cout << "  [信息] 60s 内未检测到碰撞（可能未被阻挡或运动已到位），跳过恢复流程" << std::endl;
    }

    // ---- 断开连接 ----
    print_separator("断开连接");
    tl::set_servo_poweroff(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    tl::disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 碰撞检测示例程序运行完毕" << std::endl;
    return 0;
}
