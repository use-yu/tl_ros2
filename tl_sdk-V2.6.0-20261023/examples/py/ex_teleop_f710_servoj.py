"""
ex_teleop_f710_servoj.py
@brief F710 手柄遥操作控制 (SDL2) — 直接 set_servoJ_pos 方案 (Python 版)

适用场景:
   - 使用罗技 F710 手柄进行连续、平滑的实时遥操作
   - 在循环内直接 IK + set_servoJ_pos，每 4ms 稳定输出一帧 (250Hz)
   - 笛卡尔速度积分模式，手柄松手即停

@attention
   - 确保机械臂供电正常、网络通信正常
   - 【重要】运行前请先在下方"参数设定区"确认并修改 ROBOT_IP（控制器IP）和 NUM_JOINTS（轴数）
   - 需要双端口连接: 6001（控制）+ 7000（伺服高频透传）
   - 手柄需切到 DirectInput 模式，开关拨到 "D"
   - 运行前请先安装 pysdl2: pip install pysdl2
   - 示例会连接真实机械臂，请确保工作区域安全
   - 首次使用请按 **Start** 键激活手柄控制，防止意外运动
   - Ctrl+C 退出程序；Back+Start 暂停/恢复机械臂

@note 运行步骤
       运行: export LD_LIBRARY_PATH=lib/x86:$LD_LIBRARY_PATH
            python3 examples/py/ex_teleop_f710_servoj.py

@note 操作方式
       首次使用: 按 **Start** 键激活控制
       左摇杆 X/Y   → x / y 方向平移
       右摇杆 Y     → z 方向平移
       右摇杆 X     → rz 旋转（偏航，默认）
       LB + 右摇杆 X → rx 旋转（滚转）
       RB + 右摇杆 X → ry 旋转（俯仰）
       十字键上/下   → 速度倍率 ±5%
       A 键         → 回到零位
       Back+Start   → 暂停/恢复

@note 启动流程
       双端口连接 → 伺服状态检查 → 上电 → 切换运行模式 → open_servoJ
       → 锁定当前关节角 → 按 Start 键激活 → 遥操作主循环 (250Hz)
       退出: close_servoJ → 下电 → 断开双端口
"""

import os
import platform
import sys
import time
import signal
import importlib.util
from typing import List

_script_dir = os.path.dirname(os.path.abspath(__file__))
# 脚本位于 examples/py/，项目根目录为 tl_sdk/
_project_root = os.path.dirname(os.path.dirname(_script_dir))

# 添加 SDK Python 绑定路径
_sdk_dir = os.path.join(_project_root, "lib", "linux", "x86", "python")
if os.path.isdir(_sdk_dir):
    sys.path.insert(0, _sdk_dir)
    # 修复 SWIG 模块名不匹配：_tl_host.so 内部模块名为 _nrc_host，
    # 但 tl_interface.py 中写死了 import _tl_host。
    # 先用正确内部名 _nrc_host 加载，再同时注册为 _tl_host 和 _nrc_host。
    _nrc_path = os.path.join(_sdk_dir, "_nrc_host.so")
    if os.path.isfile(_nrc_path):
        _spec = importlib.util.spec_from_file_location("_nrc_host", _nrc_path)
        _mod = importlib.util.module_from_spec(_spec)
        sys.modules["_nrc_host"] = _mod
        sys.modules["_tl_host"] = _mod  # tl_interface 需要 _tl_host
        _spec.loader.exec_module(_mod)

import sdl2

# 使用上面已定义的 _script_dir 构建 SDK Python 绑定的绝对路径
# 自动检测平台，添加对应 SDK Python 绑定路径
if platform.system() == "Windows":
    sys.path.append(os.path.join(_script_dir, "../../lib/windows/python"))
elif platform.machine() in ("aarch64", "arm64"):
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/arm64/python"))
else:
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/x86/python"))
from tl_interface import (
    SUCCESS, OPERATION_NOT_ALLOWED, VectorDouble,
    connect_robot, disconnect_robot,
    clear_error, get_servo_state,
    set_servo_state, set_servo_poweron, set_servo_poweroff,
    set_current_mode, set_speed,
    open_servoJ, close_servoJ, set_servoJ_pos,
    get_current_position,
    get_origin_coord_to_target_coord,
)

# ============================================================
# 【参数设定区】—— 按需修改
# ============================================================

# ---------- 机械臂连接参数 ----------
ROBOT_IP = "192.168.1.13"       # 控制器 IP（可用命令行参数 -ip 覆盖，默认 192.168.1.13）

# ========== 轴数 ==========
# 6 轴机械臂设为 6，7 轴设为 7
NUM_JOINTS = 6

# ========== ServoJ API 向量维度==========
# 注意: SDK 底层协议要求 servoJ 相关向量固定为 7 元素，即使 6 轴机械臂也要传 7 个，
#       最后一个元素（外部轴）填 0.0。传 6 元素会导致控制器协议解析错乱并断开连接。
SERVO_AXIS = 7

# ========== 零位关节角（度）==========
HOME_JOINTS = [0.0] * NUM_JOINTS

# ---------- 控制周期 ----------
LOOP_S = 0.004      # 4ms = 250Hz（ServoJ 最大支持 250Hz）

# ========== 运动速度（0-100）==========
SPEED_DEFAULT = 50.0
SPEED_MIN     =  5.0
SPEED_MAX     = 100.0
SPEED_STEP    =  5.0

# ========== 笛卡尔运动灵敏度（速度倍率 100% 时满推摇杆对应的速度）==========
# 参考值: 真机 80 mm/s / 1.0 rad/s，仿真可适当提高
POS_SENSITIVITY = 160.0     # 平移 mm/s
ROT_SENSITIVITY =  1.0     # 旋转 rad/s

# （工作空间限位已移除 — 默认无限制，加长款机械臂可自由运动）

# ========== 摇杆死区 ==========
DEADZONE = 0.15

# ========== ServoJ 初始化参数 ==========
# vmax 最大关节速度 (°/s)，最大 180；amax 加速度 (°/s²)；jmax 加加速度 (°/s³)
SERVO_VMAX = 300.0        # °/s
SERVO_AMAX = 3000.0      # °/s²
SERVO_JMAX = 50000.0     # °/s³

# ========== 归零速度（A 键回零时的关节插值速度）==========
# 按最大关节位移自动计算插值步数，远点多走、近点少走，速度恒定
HOME_SPEED = 30.0          # °/s

# ========== F710 轴映射（DirectInput 模式，开关拨到 "D"）==========
AXIS_LEFT_X  = 0   # 左摇杆 X
AXIS_LEFT_Y  = 1   # 左摇杆 Y
AXIS_RIGHT_X = 2   # 右摇杆 X
AXIS_RIGHT_Y = 3   # 右摇杆 Y

# ========== F710 按键映射 (DirectInput 模式) ==========
BTN_X     = 0
BTN_A     = 1
BTN_B     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_LT    = 6
BTN_RT    = 7
BTN_BACK  = 8
BTN_START = 9
NUM_BTNS  = 10

# ========== 摇杆 → 笛卡尔速度映射（默认模式）==========
# (轴号, cart_index(0=x,1=y,2=z,3=rx,4=ry,5=rz), 方向(±1))
AXIS_MAP = [
    (AXIS_LEFT_X,  1, -1),    # 左 X → y 平移
    (AXIS_LEFT_Y,  0, -1),    # 左 Y → x 平移
    (AXIS_RIGHT_X, 5,  1),    # 右 X → rz 旋转（默认，LB 按住时改为 rx）
    (AXIS_RIGHT_Y, 2, -1),    # 右 Y → z 平移
]
NUM_AXIS_MAPS = len(AXIS_MAP)

# ============================================================
# 全局状态
# ============================================================

g_running = True


def signal_handler(sig, frame):
    """Ctrl+C 信号处理"""
    global g_running
    g_running = False


def normalize_axis(value: int, deadzone: float) -> float:
    """SDL 轴值归一化，含死区（SDL2 返回 -32768 ~ 32767，归一化到 -1.0 ~ 1.0）"""
    v = value / 32767.0
    if v < -1.0:
        v = -1.0
    if abs(v) < deadzone:
        return 0.0
    if v > 0:
        return (v - deadzone) / (1.0 - deadzone)
    else:
        return (v + deadzone) / (1.0 - deadzone)


def print_pose_state(pose: List[float], speed_pct: float):
    """打印当前笛卡尔位姿（单行覆盖刷新）"""
    sys.stdout.write(
        f"\r  [速度 {int(speed_pct):3d}%]"
        f"  x={pose[0]:7.1f}  y={pose[1]:7.1f}  z={pose[2]:7.1f}"
        f"  rx={pose[3]:6.2f}  ry={pose[4]:6.2f}  rz={pose[5]:6.2f}  "
    )
    sys.stdout.flush()


# ============================================================
# 主函数
# ============================================================

def main():
    global g_running

    # 解析命令行参数: python3 ex_teleop_f710_servoj.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = ROBOT_IP
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    signal.signal(signal.SIGINT, signal_handler)

    print("=== F710 手柄遥操作（直接 ServoJ，250Hz 稳定输出）===\n")

    # ---- 打印参数 ----
    print(f"控制周期: {int(LOOP_S * 1000)} ms ({int(1 / LOOP_S)} Hz)")
    print(f"平移灵敏度: {POS_SENSITIVITY} mm/s")
    print(f"旋转灵敏度: {ROT_SENSITIVITY} rad/s")
    print(f"死区:   {DEADZONE}")
    print("\n手柄操作说明:")
    print("  左摇杆 X/Y   →  x / y 平移")
    print("  右摇杆 Y     →  z 平移")
    print("  右摇杆 X     →  rz 旋转")
    print("  LB + 右摇杆 X →  rx 旋转")
    print("  RB + 右摇杆 X →  ry 旋转")
    print("  十字键上/下   →  速度倍率")
    print("  A 键         →  回到零位")
    print("  Back+Start   →  紧急停止\n")

    # ============================================================
    # 初始化 SDL / 手柄
    # ============================================================

    if sdl2.SDL_Init(sdl2.SDL_INIT_GAMECONTROLLER | sdl2.SDL_INIT_JOYSTICK) < 0:
        print(f"SDL 初始化失败: {sdl2.SDL_GetError()}")
        print("请安装 SDL2: sudo apt install libsdl2-dev")
        return

    joystick = sdl2.SDL_JoystickOpen(0)
    if not joystick:
        print("未检测到手柄，请连接 F710 后重试")
        sdl2.SDL_Quit()
        return
    print(f"  已连接手柄: {sdl2.SDL_JoystickName(joystick)}")
    print(f"  轴数: {sdl2.SDL_JoystickNumAxes(joystick)}, "
          f"按键数: {sdl2.SDL_JoystickNumButtons(joystick)}\n")

    # ============================================================
    # 连接机械臂
    # ============================================================

    print("--- 连接机械臂 ---")
    sock_tcp = connect_robot(ip, "6001")
    sock_udp = connect_robot(ip, "7000")
    if sock_tcp < 0 or sock_udp < 0:
        print(f"连接失败: tcp={sock_tcp}, udp={sock_udp}")
        if sock_tcp >= 0:
            disconnect_robot(sock_tcp)
        if sock_udp >= 0:
            disconnect_robot(sock_udp)
        sdl2.SDL_JoystickClose(joystick)
        sdl2.SDL_Quit()
        return

    try:
        # ---- 伺服状态检查与上电 ----
        print("\n--- 伺服状态检查与上电 ---")
        clear_error(sock_tcp)
        time.sleep(0.3)
        set_servo_poweroff(sock_tcp)
        time.sleep(0.3)

        # SWIG 将 int& 输出参数以额外返回值形式返回：[result, status]
        _ret, servo_state = get_servo_state(sock_tcp, -1)
        if _ret != SUCCESS:
            print(f"[ERROR] 获取伺服状态失败: {_ret}")
        print(f"  当前伺服状态: {servo_state} (0=停止 1=就绪 2=报警 3=运行)")

        if servo_state == 2:
            print("[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试")
        elif servo_state == 3:
            print("  [信息] 伺服已运行，跳过上电")
        else:
            if servo_state == 0:
                print("  [信息] 伺服处于停止状态，设置就绪...")
                ret = set_servo_state(sock_tcp, 1)
                if ret != SUCCESS and ret != OPERATION_NOT_ALLOWED:
                    print(f"[ERROR] set_servo_state(1) 失败，错误码: {ret}")
                time.sleep(0.5)

            ret = set_servo_poweron(sock_tcp)
            if ret != SUCCESS:
                print(f"[ERROR] set_servo_poweron 失败，错误码: {ret}")
            else:
                print("  [信息] 上电成功")
            time.sleep(2)

        # ---- 切换运行模式 & 设置速度 ----
        set_current_mode(sock_tcp, 2)
        set_speed(sock_tcp, 30)

        # ---- 开启 ServoJ ----
        print("\n--- 开启伺服跟踪模式 ---")
        # 注意: SDK 协议固定要求 7 元素，6 轴机械臂多余元素置 0
        vmax_vec = VectorDouble([SERVO_VMAX] * SERVO_AXIS)
        amax_vec = VectorDouble([SERVO_AMAX] * SERVO_AXIS)
        jmax_vec = VectorDouble([SERVO_JMAX] * SERVO_AXIS)
        ret = open_servoJ(sock_udp, vmax_vec, amax_vec, jmax_vec)
        if ret != SUCCESS:
            print("open_servoJ 失败")
            return

        # ---- 获取当前关节角，立即发送稳住 ----
        joint_cmd = VectorDouble([0.0] * NUM_JOINTS)
        get_current_position(sock_tcp, Coord.JOINT, joint_cmd)
        # 补齐到 7 元素（6 轴机械臂外部轴置零）
        servo_cmd = list(joint_cmd)
        if len(servo_cmd) < SERVO_AXIS:
            servo_cmd += [0.0] * (SERVO_AXIS - len(servo_cmd))
        set_servoJ_pos(sock_udp, VectorDouble(servo_cmd))
        print("  初始关节角已锁定")

        # ---- 用实际关节角作为 IK 参考点 ----
        ref_pos = VectorDouble(list(joint_cmd))

        # ---- FK 零位关节角 → 笛卡尔位姿（归零目标）----
        home_cart = VectorDouble([0.0] * 7)
        home_joints_vd = VectorDouble(HOME_JOINTS + [0.0] * (7 - NUM_JOINTS))
        ret = get_origin_coord_to_target_coord(
            sock_tcp, Coord.JOINT, home_joints_vd,
            Coord.BASE, home_cart, 0, ref_pos)
        if ret != SUCCESS:
            print("FK 零位关节角失败，使用默认零位")
            for i in range(6):
                home_cart[i] = 0.0

        # ---- 获取当前笛卡尔位姿作为初始目标 ----
        target_pose = [0.0] * 6
        current_cart = VectorDouble([0.0] * 7)
        ret = get_current_position(sock_tcp, Coord.BASE, current_cart)
        if ret == SUCCESS:
            for i in range(6):
                target_pose[i] = current_cart[i]
        else:
            for i in range(6):
                target_pose[i] = home_cart[i]

        # ============================================================
        # Start 键等待 — 按 Start 后才开始响应手柄
        # ============================================================
        print("\n=== 按 Start 键开始遥操作 (Ctrl+C 或 Back+Start 退出) ===\n")
        start_ready = False
        while not start_ready and g_running:
            sdl2.SDL_JoystickUpdate()
            back = bool(sdl2.SDL_JoystickGetButton(joystick, BTN_BACK))
            start = bool(sdl2.SDL_JoystickGetButton(joystick, BTN_START))
            if start and not back:
                start_ready = True
            if back and start:
                print("\n  紧急停止！")
                g_running = False
                break
            servo_cmd = list(joint_cmd)
            if len(servo_cmd) < SERVO_AXIS:
                servo_cmd += [0.0] * (SERVO_AXIS - len(servo_cmd))
            set_servoJ_pos(sock_udp, VectorDouble(servo_cmd))
            time.sleep(LOOP_S)

        if not g_running:
            pass
        else:
            # ============================================================
            # 主控制循环（250Hz 稳定输出，直接 set_servoJ_pos）
            # ============================================================

            print("\n=== 进入遥操作模式 (Back+Start 暂停/恢复) ===\n")

            speed_pct = SPEED_DEFAULT
            prev_dpad_up = False
            prev_dpad_down = False
            prev_a_btn = False
            force_ik = False
            # ---- 归零关节插值状态 ----
            home_active = False
            home_steps = 0
            home_step = 0
            home_joint_start = [0.0] * NUM_JOINTS
            home_cur_joint = [0.0] * NUM_JOINTS
            paused = False
            prev_bs = False
            last_time = time.monotonic()
            next_time = last_time

            while g_running:
                # ============================================================
                # 1. 读取手柄
                # ============================================================
                sdl2.SDL_JoystickUpdate()

                axis_raw = [
                    sdl2.SDL_JoystickGetAxis(joystick, AXIS_LEFT_X),
                    sdl2.SDL_JoystickGetAxis(joystick, AXIS_LEFT_Y),
                    sdl2.SDL_JoystickGetAxis(joystick, AXIS_RIGHT_X),
                    sdl2.SDL_JoystickGetAxis(joystick, AXIS_RIGHT_Y),
                ]

                hat = sdl2.SDL_JoystickGetHat(joystick, 0)
                dpad_up   = bool(hat & sdl2.SDL_HAT_UP)
                dpad_down = bool(hat & sdl2.SDL_HAT_DOWN)

                btn = [bool(sdl2.SDL_JoystickGetButton(joystick, b)) for b in range(NUM_BTNS)]

                # ============================================================
                # 2. 暂停/恢复（Back+Start 切换）
                # ============================================================
                cur_bs = btn[BTN_BACK] and btn[BTN_START]
                if cur_bs and not prev_bs:
                    paused = not paused
                    if paused:
                        if home_active:
                            joint_cmd = VectorDouble(home_cur_joint)
                            ref_pos = VectorDouble(list(joint_cmd))
                            home_active = False
                            cur_cart = VectorDouble([0.0] * 7)
                            try:
                                get_origin_coord_to_target_coord(
                                    sock_tcp, Coord.JOINT, VectorDouble(home_cur_joint + [0.0] * (7 - NUM_JOINTS)),
                                    Coord.BASE, cur_cart, 0, ref_pos)
                            except TypeError:
                                pass
                            for i in range(6):
                                target_pose[i] = cur_cart[i]
                        print("\n  --- 已暂停 (Back+Start 恢复) ---")
                    else:
                        print("\n  --- 已恢复 ---")
                prev_bs = cur_bs

                # ============================================================
                # 3. 速度倍率（十字键 Hat 边沿触发）
                # ============================================================
                if dpad_up and not prev_dpad_up:
                    speed_pct = min(speed_pct + SPEED_STEP, SPEED_MAX)
                if dpad_down and not prev_dpad_down:
                    speed_pct = max(speed_pct - SPEED_STEP, SPEED_MIN)
                prev_dpad_up = dpad_up
                prev_dpad_down = dpad_down

                # ============================================================
                # 4. 回到零位（A 键边沿触发）— 关节空间插值
                # ============================================================
                if btn[BTN_A] and not prev_a_btn:
                    print("\n  回到零位...")
                    speed_pct = 50.0
                    cur_jnt = VectorDouble([0.0] * NUM_JOINTS)
                    get_current_position(sock_tcp, Coord.JOINT, cur_jnt)
                    max_disp = 0.0
                    for j in range(NUM_JOINTS):
                        d = abs(HOME_JOINTS[j] - cur_jnt[j])
                        if d > max_disp:
                            max_disp = d
                    dt_frame = LOOP_S
                    home_steps = max(5, int(max_disp / HOME_SPEED / dt_frame))
                    home_joint_start = list(cur_jnt)
                    home_cur_joint = list(cur_jnt)
                    home_step = 0
                    home_active = True
                prev_a_btn = btn[BTN_A]

                # ============================================================
                # 5. 计算笛卡尔速度 → 累加目标位姿（归零/暂停中跳过）
                # ============================================================
                cart_vel = [0.0] * 6
                if not home_active and not paused:
                    speed_scale = speed_pct / 100.0
                    now = time.monotonic()
                    dt = now - last_time
                    last_time = now
                    if dt > 0.1:
                        dt = LOOP_S

                    for axis_idx, cart_idx, sign in AXIS_MAP:
                        raw = normalize_axis(axis_raw[axis_idx], DEADZONE)
                        cart_vel[cart_idx] += raw * sign

                    rx_raw = normalize_axis(axis_raw[AXIS_RIGHT_X], DEADZONE)
                    if btn[BTN_LB]:
                        cart_vel[3] += rx_raw * 1.0
                        cart_vel[5] -= rx_raw * 1.0
                    if btn[BTN_RB]:
                        cart_vel[4] += rx_raw * 1.0
                        cart_vel[5] -= rx_raw * 1.0

                    for j in range(3):
                        target_pose[j] += cart_vel[j] * speed_scale * POS_SENSITIVITY * dt
                    for j in range(3, 6):
                        target_pose[j] += cart_vel[j] * speed_scale * ROT_SENSITIVITY * dt

                # ============================================================
                # 6. 发送 & IK 更新
                # ============================================================
                # 7a. 发送关节角
                if home_active and not paused:
                    home_step += 1
                    t = home_step / home_steps
                    if t >= 1.0:
                        t = 1.0
                        home_active = False
                        home_cur_joint = HOME_JOINTS[:]
                        joint_cmd = VectorDouble(home_cur_joint)
                        ref_pos = VectorDouble(list(joint_cmd))
                        for i in range(6):
                            target_pose[i] = home_cart[i]
                    for j in range(NUM_JOINTS):
                        home_cur_joint[j] = home_joint_start[j] \
                            + (HOME_JOINTS[j] - home_joint_start[j]) * t
                    servo_cmd = home_cur_joint[:]
                    if len(servo_cmd) < SERVO_AXIS:
                        servo_cmd += [0.0] * (SERVO_AXIS - len(servo_cmd))
                    set_servoJ_pos(sock_udp, VectorDouble(servo_cmd))
                else:
                    servo_cmd = list(joint_cmd)
                    if len(servo_cmd) < SERVO_AXIS:
                        servo_cmd += [0.0] * (SERVO_AXIS - len(servo_cmd))
                    set_servoJ_pos(sock_udp, VectorDouble(servo_cmd))

                # 7b. IK 更新
                has_input = any(abs(v) > 1e-6 for v in cart_vel)

                if paused:
                    pass
                elif has_input or force_ik:
                    force_ik = False
                    target_vd = VectorDouble(target_pose + [0.0])
                    new_joint = VectorDouble([0.0] * NUM_JOINTS)
                    try:
                        ret = get_origin_coord_to_target_coord(
                            sock_tcp, Coord.BASE, target_vd,
                            Coord.JOINT, new_joint, 0, ref_pos)
                    except TypeError:
                        ret = -5
                    if ret == SUCCESS:
                        joint_cmd = new_joint
                        ref_pos = VectorDouble(list(joint_cmd))

                # ============================================================
                # 8. 打印状态
                # ============================================================
                if paused:
                    sys.stdout.write("\r  *** 已暂停 ***  ")
                    sys.stdout.flush()
                else:
                    print_pose_state(target_pose, speed_pct)

                # ============================================================
                # 9. 累加定时，250Hz 稳定输出
                # ============================================================
                next_time += LOOP_S
                if next_time < time.monotonic():
                    next_time = time.monotonic()
                delay = next_time - time.monotonic()
                if delay > 0:
                    time.sleep(delay)

    finally:
        # ============================================================
        # 清理退出
        # ============================================================
        print("\n\n--- 退出中... ---\n")
        close_servoJ(sock_udp)
        set_current_mode(sock_tcp, 0)
        set_servo_poweroff(sock_tcp)
        disconnect_robot(sock_udp)
        disconnect_robot(sock_tcp)

    sdl2.SDL_JoystickClose(joystick)
    sdl2.SDL_Quit()

    print("=== F710 遥操作结束 ===")


if __name__ == "__main__":
    main()
