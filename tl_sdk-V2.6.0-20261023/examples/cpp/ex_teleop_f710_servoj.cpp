/**
 * ex_teleop_f710_servoj.cpp
 * @brief F710 手柄遥操作控制 (SDL2) — 直接 set_servoJ_pos 方案
 *
 * 适用场景:
 *   - 使用罗技 F710 手柄进行连续、平滑的实时遥操作
 *   - 在循环内直接 IK + set_servoJ_pos，每 4ms 稳定输出一帧 (250Hz)
 *   - 笛卡尔速度积分模式，手柄松手即停
 *
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 【重要】运行前请先在下方"参数设定区"确认并修改 ROBOT_IP（控制器IP）和 NUM_JOINTS（轴数）
 *   - 需要双端口连接: 6001（控制）+ 7000（伺服高频透传）
 *   - 手柄需切到 DirectInput 模式，开关拨到 "D"
 *   - 运行前请先安装 SDL2: sudo apt install libsdl2-dev
 *   - 示例会连接真实机械臂，请确保工作区域安全
 *   - 首次使用请按 **Start** 键激活手柄控制，防止意外运动
 *   - Ctrl+C 退出程序；Back+Start 暂停/恢复机械臂
 *
 * @note 运行步骤
 *       编译: cd examples/cpp/build && cmake .. && make
 *       链接动态库: export LD_LIBRARY_PATH=lib/x86:$LD_LIBRARY_PATH
 *       运行: ./ex_teleop_f710_servoj
 *
 * @note 操作方式
 *       首次使用: 按 **Start** 键激活控制
 *       左摇杆 X/Y   → x / y 方向平移
 *       右摇杆 Y     → z 方向平移
 *       右摇杆 X     → rz 旋转（偏航，默认）
 *       LB + 右摇杆 X → rx 旋转（滚转）
 *       RB + 右摇杆 X → ry 旋转（俯仰）
 *       十字键上/下   → 速度倍率 ±5%
 *       A 键         → 回到零位
 *       Back+Start   → 暂停/恢复
 *
 * @note 启动流程
 *       双端口连接 → 伺服状态检查 → 上电 → 切换运行模式 → open_servoJ
 *       → 锁定当前关节角 → 按 Start 键激活 → 遥操作主循环 (250Hz)
 *       退出: close_servoJ → 下电 → 断开双端口
 */

// ============================================================
// 头文件
// ============================================================

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <csignal>
#include <SDL2/SDL.h>
#include "tl_api.h"
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif

// ============================================================
// 【参数设定区】—— 按需修改
// ============================================================

// ---------- 机械臂连接参数 ----------
/// 控制器 IP 地址（可用命令行参数 -ip 覆盖，默认 192.168.1.13）
static std::string ROBOT_IP = "192.168.1.13";

/// ========== 轴数 ==========
/// 6 轴机械臂设为 6，7 轴设为 7
static const int NUM_JOINTS = 7;

/// ========== ServoJ API 向量维度 ==========
/// 注意: SDK 底层协议要求 servoJ 相关向量固定为 7 元素，即使 6 轴机械臂也要传 7 个，
///       最后一个元素（外部轴）填 0.0。传 6 元素会导致控制器协议解析错乱并断开连接。
static const int SERVO_AXIS = 7;

/// ========== 零位关节角 ==========
/// [J1..JN]（度），N=NUM_JOINTS，程序启动时 FK 转为笛卡尔位姿
static const std::vector<double> HOME_JOINTS(NUM_JOINTS, 0.0);

// ---------- 控制周期 ----------
/// 控制周期（毫秒）——决定指令发送频率。ServoJ 最大支持 250Hz
static const int LOOP_MS = 4;   // 250Hz

/// ========== 运动速度（0-100） ==========
static const double SPEED_DEFAULT = 50.0;
static const double SPEED_MIN     =  5.0;
static const double SPEED_MAX     = 100.0;
static const double SPEED_STEP    = 5.0;

/// ========== 笛卡尔运动灵敏度（速度倍率 100% 时满推摇杆对应的速度）==========
/// 参考值: 真机 80 mm/s / 1.0 rad/s，仿真可适当提高
static const double POS_SENSITIVITY = 160.0;    // 平移 mm/s
static const double ROT_SENSITIVITY =  1.0;    // 旋转 rad/s

// （工作空间限位已移除 — 默认无限制，加长款机械臂可自由运动）

/// ========== 摇杆死区 ==========
static const double DEADZONE = 0.15;

/// ========== ServoJ 初始化参数（open_servoJ 传入控制器）==========
/// vmax 最大关节速度 (°/s)，最大 180；amax 加速度 (°/s²)；jmax 加加速度 (°/s³)
/// 参考值来自 ROS2 真机配置 tl_teleop_f710_6axis.yaml
static const double SERVO_VMAX = 300.0;       // °/s
static const double SERVO_AMAX = 3000.0;     // °/s²
static const double SERVO_JMAX = 50000.0;    // °/s³

/// ========== 归零速度（A 键回零时的关节插值速度）==========
/// 按最大关节位移自动计算插值步数，远点多走、近点少走，速度恒定
static const double HOME_SPEED = 30.0;        // °/s

/// ========== F710 轴映射（DirectInput 模式，开关拨到 "D"）==========
enum F710Axes {
    AXIS_LEFT_X  = 1,   // 左摇杆 X
    AXIS_LEFT_Y  = 0,   // 左摇杆 Y
    AXIS_RIGHT_X = 2,   // 右摇杆 X
    AXIS_RIGHT_Y = 3,   // 右摇杆 Y
    // 十字键用 SDL_JoystickGetHat(joystick, 0) 读取，非摇杆轴
};

/// ========== F710 按键映射 (DirectInput 模式) ==========
enum F710Buttons {
    BTN_X     = 0,
    BTN_A     = 1,
    BTN_B     = 2,
    BTN_Y     = 3,
    BTN_LB    = 4,
    BTN_RB    = 5,
    BTN_LT    = 6,
    BTN_RT    = 7,
    BTN_BACK  = 8,
    BTN_START = 9,
    NUM_BTNS  = 10,
};

/// ========== 摇杆 → 笛卡尔速度映射（默认模式，无 modifier 按键时）==========
/// 每个元素: {轴号, cart_index(0=x,1=y,2=z,3=rx,4=ry,5=rz), 方向(±1)}
/// LB 作为 modifier 时：右 X → rx（覆盖默认的 rz）
/// RB 作为 modifier 时：右 X → ry（覆盖默认的 rz）
struct AxisMap {
    int axis;
    int cart_idx;
    int sign;
};
static const AxisMap AXIS_MAP[] = {
    {AXIS_LEFT_X,  0,  -1},    // 左 X → x 平移
    {AXIS_LEFT_Y,  1,  -1},    // 左 Y → y 平移
    {AXIS_RIGHT_X, 5,  1},     // 右 X → rz 旋转（默认，LB 按住时改为 rx）
    {AXIS_RIGHT_Y, 2,  -1},    // 右 Y → z 平移  （默认，RB 按住时改为 ry）
};
static const int NUM_AXIS_MAPS = sizeof(AXIS_MAP) / sizeof(AXIS_MAP[0]);

// ============================================================
// 全局状态
// ============================================================

static std::atomic<bool> g_running{true};
static tl::SOCKETFD g_sock_tcp = -1;
static tl::SOCKETFD g_sock_udp = -1;

// ============================================================
// 信号处理
// ============================================================

static void signal_handler(int) {
    g_running.store(false);
}

// ============================================================
// 辅助函数
// ============================================================

/// SDL 轴值归一化，含死区
static double normalize_axis(Sint16 value, double deadzone) {
    double v = static_cast<double>(value) / 32767.0;
    if (v < -1.0) v = -1.0;
    if (std::abs(v) < deadzone) return 0.0;
    return (v > 0) ? (v - deadzone) / (1.0 - deadzone)
                   : (v + deadzone) / (1.0 - deadzone);
}

/// 打印当前笛卡尔位姿 + 实时延迟（单行覆盖刷新）
static void print_pose_state(const std::vector<double> &pose, double speed_pct,
                              double dt_ms, double t_sdl_ms, double t_ik_ms) {
    std::cout << "\r  [速度 " << std::setw(3) << static_cast<int>(speed_pct)
              << "%]  x=" << std::fixed << std::setprecision(1) << pose[0]
              << "  y=" << pose[1] << "  z=" << pose[2]
              << "  rx=" << std::setprecision(2) << pose[3]
              << "  ry=" << pose[4] << "  rz=" << pose[5]
              << "  dt=" << std::setprecision(1) << dt_ms << "ms"
              << " SDL=" << t_sdl_ms << "ms"
              << " IK=" << t_ik_ms << "ms  ";
    std::cout.flush();
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char *argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // 解析命令行参数: ./ex_xxx [-ip 192.168.1.13]，默认 192.168.1.13
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ROBOT_IP = argv[i + 1];
            break;
        }
    }

    // ---- 信号 ----
    signal(SIGINT, signal_handler);

    std::cout << "=== F710 手柄遥操作（直接 ServoJ，250Hz 稳定输出） ===\n\n";

    // ---- 打印参数 ----
    std::cout << "控制周期: " << LOOP_MS << " ms (" << (1000/LOOP_MS) << " Hz)\n";
    std::cout << "平移灵敏度: " << POS_SENSITIVITY << " mm/s\n";
    std::cout << "旋转灵敏度: " << ROT_SENSITIVITY << " rad/s\n";
    std::cout << "死区:   " << DEADZONE << "\n";
    std::cout << "\n手柄操作说明:\n";
    std::cout << "  左摇杆 X/Y   →  x / y 平移\n";
    std::cout << "  右摇杆 Y     →  z 平移\n";
    std::cout << "  右摇杆 X     →  rz 旋转\n";
    std::cout << "  LB + 右摇杆 X →  rx 旋转\n";
    std::cout << "  RB + 右摇杆 X →  ry 旋转\n";
    std::cout << "  十字键上/下   →  速度倍率\n";
    std::cout << "  A 键         →  回到零位\n";
    std::cout << "  Back+Start   →  暂停/恢复\n\n";

    // ============================================================
    // 初始化 SDL
    // ============================================================

    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "SDL 初始化失败: " << SDL_GetError() << "\n";
        std::cerr << "请安装 SDL2: sudo apt install libsdl2-dev\n";
        return -1;
    }

    SDL_Joystick *joystick = SDL_JoystickOpen(0);
    if (!joystick) {
        std::cerr << "未检测到手柄，请连接 F710 后重试\n";
        SDL_Quit();
        return -1;
    }
    std::cout << "  已连接手柄: " << SDL_JoystickName(joystick) << "\n";
    std::cout << "  轴数: " << SDL_JoystickNumAxes(joystick)
              << ", 按键数: " << SDL_JoystickNumButtons(joystick) << "\n\n";

    // ============================================================
    // 连接机械臂
    // ============================================================

    g_sock_tcp = tl::connect_robot(ROBOT_IP, "6001");
    g_sock_udp = tl::connect_robot(ROBOT_IP, "7000");
    if (g_sock_tcp < 0 || g_sock_udp < 0) {
        std::cerr << "连接机械臂失败\n";
        SDL_JoystickClose(joystick);
        SDL_Quit();
        return -1;
    }
    std::cout << "  已连接到机械臂\n";

    // ---- 伺服状态检查与上电 ----
    std::cout << "\n--- 伺服状态检查与上电 ---\n";
    tl::clear_error(g_sock_tcp);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(g_sock_tcp);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    {
        int servo_state = -1;
        auto ret_local = tl::get_servo_state(g_sock_tcp, servo_state);
        if (ret_local != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取伺服状态失败: " << ret_local << std::endl;
        }
        std::cout << "  当前伺服状态: " << servo_state
                  << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

        if (servo_state == 2) {
            std::cerr << "[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试" << std::endl;
        } else if (servo_state == 3) {
            std::cout << "  [信息] 伺服已运行，跳过上电" << std::endl;
        } else {
            // 确保伺服就绪
            if (servo_state == 0) {
                ret_local = tl::set_servo_state(g_sock_tcp, 1);
                if (ret_local != tl::Result::SUCCESS && ret_local != tl::Result::OPERATION_NOT_ALLOWED) {
                    std::cerr << "[ERROR] tl::set_servo_state(1) 失败，错误码: " << ret_local << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // 上电使能
            ret_local = tl::set_servo_poweron(g_sock_tcp);
            if (ret_local != tl::Result::SUCCESS) {
                std::cerr << "[ERROR] set_servo_poweron 失败，错误码: " << ret_local << std::endl;
            } else {
                std::cout << "  [信息] 上电成功" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // ---- 切换运行模式 & 设置速度 ----
    tl::set_current_mode(g_sock_tcp, 2);
    tl::set_speed(g_sock_tcp, 30);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "  当前模式已切换为运行模式\n";

    // ---- 开启 ServoJ ----
    std::cout << "\n--- 开启伺服跟踪模式 ---\n";
    // 速度/加速度/加加速度约束（SERVO_AXIS 元素，单位 °/s, °/s², °/s³）
    // 注意: SDK 协议固定要求 7 元素，6 轴机械臂多余元素置 0
    std::vector<double> vmax_servo(SERVO_AXIS, SERVO_VMAX);
    std::vector<double> amax_servo(SERVO_AXIS, SERVO_AMAX);
    std::vector<double> jmax_servo(SERVO_AXIS, SERVO_JMAX);
    tl::Result ret = tl::open_servoJ(g_sock_udp, vmax_servo, amax_servo, jmax_servo);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "open_servoJ 失败\n";
        tl::disconnect_robot(g_sock_udp);
        tl::disconnect_robot(g_sock_tcp);
        SDL_JoystickClose(joystick);
        SDL_Quit();
        return -1;
    }

    // ---- 获取当前关节角，立即发送稳住（避免 servoJ 空窗期）----
    std::vector<double> joint_cmd(NUM_JOINTS, 0.0);
    tl::get_current_position(g_sock_tcp, tl::Coord::JOINT, joint_cmd);
    // 补齐到 7 元素（6 轴机械臂外部轴置零）
    {
        auto servo_cmd = joint_cmd;
        servo_cmd.resize(SERVO_AXIS, 0.0);
        tl::set_servoJ_pos(g_sock_udp, servo_cmd);
    }
    std::cout << "  初始关节角已锁定" << std::endl;

    // ---- 用实际关节角作为 IK 参考点（避免全零导致解跳变）----
    std::vector<double> ref_pos = joint_cmd;

    // ---- FK 零位关节角 → 笛卡尔位姿（归零目标）----
    std::vector<double> home_cart(7, 0.0);
    {
        bool convert_state = false;
        ret = tl::get_origin_coord_to_target_coord(
            g_sock_tcp, tl::Coord::JOINT, HOME_JOINTS,
            tl::Coord::BASE, home_cart, convert_state, 0, ref_pos);
    }
    if (ret != tl::Result::SUCCESS || home_cart.size() < 6) {
        std::cerr << "FK 零位关节角失败\n";
        for (int i = 0; i < 6; ++i) home_cart[i] = 0.0;
    }

    // ---- 获取当前笛卡尔位姿作为初始目标 ---- 
    std::vector<double> target_pose(6, 0.0);
    std::vector<double> current_cart(7, 0.0);
    ret = tl::get_current_position(g_sock_tcp, tl::Coord::BASE, current_cart);
    if (ret == tl::Result::SUCCESS && current_cart.size() >= 6) {
        for (int i = 0; i < 6; ++i) target_pose[i] = current_cart[i];
    } else {
        for (int i = 0; i < 6; ++i) target_pose[i] = home_cart[i];
    }

    // ============================================================
    // Start 键等待 — 按 Start 后才开始响应手柄
    // ============================================================
    // 防止启动时误触导致机械臂意外运动
    std::cout << "\n=== 按 Start 键开始遥操作 (Ctrl+C 或 Back+Start 退出) ===\n\n";
    {
        bool start_ready = false;
        while (!start_ready && g_running.load()) {
            SDL_JoystickUpdate();
            // 检测 Start（单独按下）或 Back+Start（紧急退出）
            bool back = SDL_JoystickGetButton(joystick, BTN_BACK);
            bool start = SDL_JoystickGetButton(joystick, BTN_START);
            if (start && !back) {
                start_ready = true;
            }
            if (back && start) {
                std::cout << "\n  紧急停止！" << std::endl;
                g_running.store(false);
                break;
            }
            // 等待期间持续发送当前关节角，保持伺服锁定
            auto servo_cmd = joint_cmd;
            servo_cmd.resize(SERVO_AXIS, 0.0);
            tl::set_servoJ_pos(g_sock_udp, servo_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_MS));
        }
    }
    if (!g_running.load()) {
        // 用户在等待时退出了，直接跳到清理
    } else {
        // ============================================================
        // 主控制循环（250Hz 稳定输出，直接 set_servoJ_pos）
        // ============================================================

        std::cout << "\n=== 进入遥操作模式 (Back+Start 暂停/恢复) ===\n\n";

        double speed_pct = SPEED_DEFAULT;
        bool prev_dpad_up = false, prev_dpad_down = false;
        bool prev_a_btn = false;
        bool force_ik = false;
        // ---- 归零关节插值状态 ----
        bool home_active = false;
        int home_steps = 0, home_step = 0;
        std::vector<double> home_joint_start(NUM_JOINTS, 0.0);
        std::vector<double> home_cur_joint(NUM_JOINTS, 0.0);
        bool paused = false;               // Back+Start 切换暂停/恢复
        bool prev_bs = false;              // Back+Start 边沿检测
        bool btn_state[NUM_BTNS] = {false};

    const auto period = std::chrono::milliseconds(LOOP_MS);
    auto next_time = std::chrono::steady_clock::now();
    auto last_time = next_time;

    while (g_running.load()) {
        // ---- 帧开始时间（用于 dt）----
        double t_ik_ms = 0.0;

        // ============================================================
        // 1. 读取手柄（计时 SDL）
        // ============================================================

        auto t0_sdl = std::chrono::steady_clock::now();
        SDL_JoystickUpdate();

        Sint16 axis_raw[4];
        axis_raw[AXIS_LEFT_X]  = SDL_JoystickGetAxis(joystick, AXIS_LEFT_X);
        axis_raw[AXIS_LEFT_Y]  = SDL_JoystickGetAxis(joystick, AXIS_LEFT_Y);
        axis_raw[AXIS_RIGHT_X] = SDL_JoystickGetAxis(joystick, AXIS_RIGHT_X);
        axis_raw[AXIS_RIGHT_Y] = SDL_JoystickGetAxis(joystick, AXIS_RIGHT_Y);

        Uint8 hat = SDL_JoystickGetHat(joystick, 0);
        bool dpad_up   = (hat & SDL_HAT_UP)   != 0;
        bool dpad_down = (hat & SDL_HAT_DOWN) != 0;

        for (int b = 0; b < NUM_BTNS; ++b)
            btn_state[b] = SDL_JoystickGetButton(joystick, b);

        double t_sdl_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0_sdl).count();

        // ============================================================
        // 2. 暂停/恢复（Back+Start 切换）
        // ============================================================
        bool cur_bs = btn_state[BTN_BACK] && btn_state[BTN_START];
        if (cur_bs && !prev_bs) {
            paused = !paused;
            if (paused) {
                // 暂停时取消正在进行的归零，同步状态到当前插值位置
                if (home_active) {
                    joint_cmd = home_cur_joint;
                    ref_pos = joint_cmd;
                    home_active = false;
                    // FK 当前插值关节角 → 更新 target_pose，恢复后不跳变
                    std::vector<double> cur_cart(7, 0.0);
                    bool cs = false;
                    tl::get_origin_coord_to_target_coord(
                        g_sock_tcp, tl::Coord::JOINT, home_cur_joint,
                        tl::Coord::BASE, cur_cart, cs, 0, ref_pos);
                    for (int i = 0; i < 6; ++i) target_pose[i] = cur_cart[i];
                }
                std::cout << "\n  --- 已暂停 (Back+Start 恢复) ---\n";
            } else {
                std::cout << "\n  --- 已恢复 ---\n";
            }
        }
        prev_bs = cur_bs;

        // ============================================================
        // 3. 速度倍率（十字键 Hat 边沿触发）
        // ============================================================
        if (dpad_up && !prev_dpad_up)
            speed_pct = std::min(speed_pct + SPEED_STEP, SPEED_MAX);
        if (dpad_down && !prev_dpad_down)
            speed_pct = std::max(speed_pct - SPEED_STEP, SPEED_MIN);
        prev_dpad_up = dpad_up;
        prev_dpad_down = dpad_down;

        // ============================================================
        // 4. 回到零位（A 键边沿触发）— 关节空间插值
        // ============================================================
        // 读取当前关节角 → 找出最大位移 → 按 HOME_SPEED 计算步数
        // → 在主循环 step 7a 中逐帧发送插值关节角
        // 优点：速度精确（60°/s），远点多走近点少走，不影响 IK 机制
        if (btn_state[BTN_A] && !prev_a_btn) {
            std::cout << "\n  回到零位...\n";
            speed_pct = 50.0;
            // 获取当前关节角
            std::vector<double> cur_jnt(NUM_JOINTS, 0.0);
            tl::get_current_position(g_sock_tcp, tl::Coord::JOINT, cur_jnt);
            // 找最大关节位移
            double max_disp = 0.0;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                double d = std::abs(HOME_JOINTS[j] - cur_jnt[j]);
                if (d > max_disp) max_disp = d;
            }
            // 按 HOME_SPEED 计算插值步数（每帧约 LOOP_MS ms）
            double dt_frame = LOOP_MS / 1000.0;
            home_steps = std::max(5, static_cast<int>(max_disp / HOME_SPEED / dt_frame));
            home_joint_start = cur_jnt;
            home_cur_joint = cur_jnt;
            home_step = 0;
            home_active = true;
        }
        prev_a_btn = btn_state[BTN_A];

        // ============================================================
        // 5. 计算笛卡尔速度 → 累加目标位姿（归零/暂停中跳过）
        // ============================================================
        double cart_vel[6] = {0.0};  // 声明在块外，IK 段要读 has_input
        //
        // dt 使用实际帧间隔（而非固定 LOOP_MS），确保即使 IK 调用耗时较长，
        // 每帧积分量仍与真实经过时间匹配，速度感不受循环频率影响。
        // 帧间隔测量始终执行（不受归零/暂停影响），保证状态行 dt 持续更新。
        //
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;
        if (dt > 0.1) dt = LOOP_MS / 1000.0;

        if (!home_active && !paused) {
        double speed_scale = speed_pct / 100.0;

        // 5a. 摇杆默认映射
        for (int i = 0; i < NUM_AXIS_MAPS; ++i) {
            double raw = normalize_axis(axis_raw[AXIS_MAP[i].axis], DEADZONE);
            cart_vel[AXIS_MAP[i].cart_idx] += raw * AXIS_MAP[i].sign;
        }
        // 5b. Modifier 按键覆盖
        double rx_raw = normalize_axis(axis_raw[AXIS_RIGHT_X], DEADZONE);
        double ry_raw = normalize_axis(axis_raw[AXIS_RIGHT_Y], DEADZONE);
        if (btn_state[BTN_LB]) {
            cart_vel[3] += rx_raw * 1.0;   // rx
            cart_vel[5] -= rx_raw * 1.0;   // 取消默认 rz
        }
        if (btn_state[BTN_RB]) {
            // RB + 右 X → ry（取消默认的 rz）
            cart_vel[4] += rx_raw * 1.0;   // ry
            cart_vel[5] -= rx_raw * 1.0;   // 取消默认 rz
        }
        // 5c. 积分
        for (int j = 0; j < 3; ++j)
            target_pose[j] += cart_vel[j] * speed_scale * POS_SENSITIVITY * dt;
        for (int j = 3; j < 6; ++j)
            target_pose[j] += cart_vel[j] * speed_scale * ROT_SENSITIVITY * dt;
        }  // if (!home_active && !paused) — 归零/暂停中跳过摇杆积分

        // ============================================================
        // 6. 发送 & IK 更新
        // ============================================================
        //
        // 【关键设计】先发送、后 IK：
        //   1) tl::set_servoJ_pos(UDP，~1ms) 立即发出关节角，
        //      保证机械臂始终有指令流，不会因 IK 等待而断档。
        //   2) 归零时发送插值后的关节角，不调 IK。
        //   3) 正常模式下先发上一帧关节角，后做 IK。
        //   4) 无摇杆输入时跳过 IK（target_pose 未变，IK 结果相同）。
        //
        // 7a. 发送关节角
        if (home_active && !paused) {
            // ---- 归零插值模式：逐帧发送插值关节角 ----
            home_step++;
            double t = static_cast<double>(home_step) / home_steps;
            if (t >= 1.0) {
                t = 1.0;
                home_active = false;
                // 归零完成，同步状态变量
                home_cur_joint = HOME_JOINTS;
                joint_cmd = home_cur_joint;
                ref_pos = joint_cmd;
                for (int i = 0; i < 6; ++i) target_pose[i] = home_cart[i];
            }
            for (int j = 0; j < NUM_JOINTS; ++j) {
                home_cur_joint[j] = home_joint_start[j]
                    + (HOME_JOINTS[j] - home_joint_start[j]) * t;
            }
            auto servo_cmd = home_cur_joint;
            servo_cmd.resize(SERVO_AXIS, 0.0);
            tl::set_servoJ_pos(g_sock_udp, servo_cmd);
        } else {
            // ---- 正常模式：立即发送上一帧的关节角 ----
            auto servo_cmd = joint_cmd;
            servo_cmd.resize(SERVO_AXIS, 0.0);
            tl::set_servoJ_pos(g_sock_udp, servo_cmd);
        }

        // 7b. 有摇杆输入、强制 IK 或暂停中需要保持时才做 IK
        bool has_input = false;
        for (int i = 0; i < 6; ++i) {
            if (std::abs(cart_vel[i]) > 1e-6) { has_input = true; break; }
        }

        if (paused) {
            // 暂停中不调 IK，joint_cmd 保持原值，持续发送锁定位置
        } else if (has_input || force_ik) {
            auto t0_ik = std::chrono::steady_clock::now();
            force_ik = false;
            std::vector<double> new_joint(NUM_JOINTS, 0.0);
            bool convert_state = false;
            ret = tl::get_origin_coord_to_target_coord(
                g_sock_tcp, tl::Coord::BASE, target_pose,
                tl::Coord::JOINT, new_joint, convert_state, 0, ref_pos);
            t_ik_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0_ik).count();
            if (ret == tl::Result::SUCCESS) {
                joint_cmd.swap(new_joint);
                ref_pos = joint_cmd;
            }
            // IK 失败则 joint_cmd 保持原值（上一帧的有效值）
        }

        // ============================================================
        // 8. 打印状态
        // ============================================================
        if (paused) {
            std::cout << "\r  *** 已暂停 ***  " << std::flush;
        } else {
            double dt_ms = dt * 1000.0;
            print_pose_state(target_pose, speed_pct, dt_ms, t_sdl_ms, t_ik_ms);
        }

        // ============================================================
        // 9. 累加定时，250Hz 稳定输出
        // ============================================================
        next_time += period;
        // 如果处理耗时超过了周期，next_time 已落后于实际时间，
        // 此时直接进入下一帧而不等待，并将 next_time 重置到当前时间，
        // 避免累积漂移导致循环"空转"无等待
        if (next_time < std::chrono::steady_clock::now()) {
            next_time = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_until(next_time);
    }
    }  // else（Start 键已按下）

    // ============================================================
    // 清理退出
    // ============================================================

    std::cout << "\n\n--- 退出中... ---\n";

    tl::close_servoJ(g_sock_udp);
    tl::set_current_mode(g_sock_tcp, 0);
    tl::set_servo_poweroff(g_sock_tcp);

    tl::disconnect_robot(g_sock_udp);
    tl::disconnect_robot(g_sock_tcp);

    SDL_JoystickClose(joystick);
    SDL_Quit();

    std::cout << "=== F710 遥操作结束 ===\n";
    return 0;
}
