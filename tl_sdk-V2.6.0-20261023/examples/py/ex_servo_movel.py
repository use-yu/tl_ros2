"""
示例 3: Servo MoveL 笛卡尔空间直线伺服运动控制测试

流程:
   连接 → 清除错误 → 上电 → open_servo_movel → MoveL 到目标 → 暂停 → MoveL 回零
   → close_servo_movel → 下电 → 关闭

支持两种输入模式（通过 COORD 切换）：
   COORD=0  : target 为关节角度 [J1..J6] 或 [J1..J7]（单位：度），内部 FK 转笛卡尔做直线插值
   COORD=1/2/3: target 为笛卡尔位姿 [x, y, z, rx, ry, rz]（mm, rad），直接在笛卡尔空间直线插值

支持 6 轴和 7 轴机械臂（通过 NUM_JOINTS 切换）。

注意：
   servo_movel 在笛卡尔空间做直线插值 + 四元数 Slerp 姿态插值，
   每步通过 IK（逆运动学）转为关节角下发，轨迹更平滑。

依赖:
   - tl_interface（SDK Python 统一入口，from tl_interface import *）

使用:
   修改下方 NUM_JOINTS、COORD 和 TARGET 来设定测试目标。
"""

import os
import platform
import sys
import time

_script_dir = os.path.dirname(os.path.abspath(__file__))

# tl_interface 在 lib/<platform>/python/ 下（编译后自动同步）
if platform.system() == "Windows":
    sys.path.append(os.path.join(_script_dir, "../../lib/windows/python"))
elif platform.machine() in ("aarch64", "arm64"):
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/arm64/python"))
else:
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/x86/python"))

from tl_interface import *

# ============================================================
# 【用户设定区】—— 修改此处以改变测试目标
# ============================================================

# 机械臂 IP（可用命令行参数 -ip 覆盖，默认 192.168.1.13）
ROBOT_IP = "192.168.1.13"

# 通信端口（6001 = 命令通道 TCP，7000 = 伺服数据通道 UDP）
TCP_PORT = "6001"
UDP_PORT = "7000"

# 关节数: 6 或 7（根据机械臂型号设置）
# 6 轴时 TARGET 传 [J1..J6]；7 轴时传 [J1..J7]
NUM_JOINTS = 7

# 坐标系标志: 0=关节坐标, 1=基坐标系, 2=工具, 3=用户
# COORD=0 时 TARGET 为关节角度 [J1..JN]（度），内部 FK 转笛卡尔后做直线插值
# COORD=1/2/3 时 TARGET 为笛卡尔位姿 [x,y,z,rx,ry,rz]（mm, rad）
COORD = 1

# 目标位置（根据 COORD 自动解释为关节角或笛卡尔位姿）
# COORD=0 时填 NUM_JOINTS 个关节角度，COORD!=0 时前 6 个为笛卡尔位姿
TARGET = [
    285.0,   # x (mm)
    173.0,   # y (mm)
    251.0,   # z (mm)
     -3.14,   # rx (rad)
      0.0,   # ry (rad)
      0.0,   # rz (rad)
]

# ---- 关节角备选目标（COORD=0 时使用） ----
# TARGET = [
#     20.0,    # J1 (度)
#    -15.0,    # J2 (度)
#     30.0,    # J3 (度)
#    -10.0,    # J4 (度)
#     15.0,    # J5 (度)
#     -5.0,    # J6 (度)
#     45.0,    # J7（仅 7 轴有效）
# ]

# 到达目标后暂停时间（秒）
PAUSE_SECONDS = 3

# ServoJ 速度/加速度/加加速度百分比 (0-100)
# 与 C++ open_servo_movel 的百分比调速一致
VMAX_PCT = 10.0
AMAX_PCT = 80.0
JMAX_PCT = 10.0


def build_target(raw_target, num_joints, coord):
    """根据 NUM_JOINTS 和 coord 截取目标数组"""
    if coord == 0:
        return list(raw_target[:num_joints])
    else:
        # 笛卡尔模式固定取前 6 个元素
        return list(raw_target[:6])


def print_target(target, coord):
    """打印目标位置"""
    prefix = "[关节] " if coord == 0 else "[笛卡尔] "
    prec = 1 if coord == 0 else 3
    parts = [f"{v:.{prec}f}" for v in target]
    print(f"{prefix}[{', '.join(parts)}]")


def main():
    # 解析命令行参数: python3 ex_servo_movel.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = ROBOT_IP
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    print("=== Servo MoveL 笛卡尔空间直线伺服运动测试 ===\n")

    # ---- 打印测试参数 ----
    coord_name = "关节坐标" if COORD == 0 else "笛卡尔坐标"
    print(f"机械臂 IP: {ip}")
    print(f"关节数: {NUM_JOINTS} 轴")
    print(f"坐标系: coord={COORD} ({coord_name})")
    print(f"servo_movel 参数: vmax={VMAX_PCT}%, amax={AMAX_PCT}%, jmax={JMAX_PCT}%")

    target = build_target(TARGET, NUM_JOINTS, COORD)
    print("目标: ", end="")
    print_target(target, COORD)
    print()

    # ================================================================
    # 1. 连接机械臂
    # ================================================================
    print("--- 连接机械臂 ---")
    sock_tcp = connect_robot(ip, TCP_PORT)
    sock_udp = connect_robot(ip, UDP_PORT)
    if sock_tcp <= 0 or sock_udp <= 0:
        print(f"连接失败: tcp={sock_tcp}, udp={sock_udp}")
        if sock_tcp >= 0:
            disconnect_robot(sock_tcp)
        if sock_udp >= 0:
            disconnect_robot(sock_udp)
        return

    try:
        # ================================================================
        # 2. 伺服就绪（servoJ 不需要物理上电，只需伺服就绪 + 运行模式）
        # ================================================================
        print("\n--- 伺服就绪 ---")
        set_current_mode(sock_tcp, 0)           # 先切示教模式
        clear_error(sock_tcp)
        set_servo_poweroff(sock_tcp)            # 下电复位
        time.sleep(0.6)
        set_servo_state(sock_tcp, 1)            # 伺服就绪
        time.sleep(0.5)
        set_current_mode(sock_tcp, 2)           # 切换运行模式
        time.sleep(2)                           # 等待控制器状态稳定

        # ================================================================
        # 3. 开启 ServoJ（通过 open_servo_movel，百分比调速）
        # ================================================================
        print("\n========== 开启伺服跟踪模式 ==========")
        ret = open_servo_movel(sock_udp, VMAX_PCT, AMAX_PCT, JMAX_PCT)
        if ret != SUCCESS:
            print(f"open_servo_movel 失败, ret={ret}")
            return
        time.sleep(2)

        # ---- 构造零位（关节角全零） ----
        zero = [0.0] * NUM_JOINTS

        # ---- MoveL：零位 → 目标 ----
        print(f"\n========== MoveL（零位 → 目标, {NUM_JOINTS} 轴）==========")
        print("  目标: ", end="")
        print_target(target, COORD)

        ret = servo_movel(sock_tcp, sock_udp, target, coord=COORD)
        if ret != SUCCESS:
            print(f"servo_movel 正向运动失败, ret={ret}")

        wait_servo()
        print(f"\n  到达目标，暂停 {PAUSE_SECONDS} 秒...")
        time.sleep(PAUSE_SECONDS)

        # ---- MoveL：目标 → 零位（关节角全零）----
        print(f"\n========== MoveL（目标 → 零位, {NUM_JOINTS} 轴）==========")
        print("  目标: ", end="")
        print_target(zero, 0)

        ret = servo_movel(sock_tcp, sock_udp, zero, coord=0)
        if ret != SUCCESS:
            print(f"servo_movel 反向运动失败, ret={ret}")

        print(f"\n  回到零位，暂停 {PAUSE_SECONDS} 秒...")
        time.sleep(PAUSE_SECONDS)

        # ---- 打断测试 ----
        print("\n========== 打断测试（Cancel）==========")
        far_target = [520.0, 0.0, 300.0, -3.141, 0.0, 0.0]
        print("  启动远距离运动（目标: ", end="")
        print_target(far_target, 1)
        print("）...")

        ret = servo_movel(sock_tcp, sock_udp, far_target, coord=1)
        if ret != SUCCESS:
            print(f"servo_movel 入队失败, ret={ret}")

        time.sleep(0.8)
        print("  调用 cancel_servo() 打断运动")
        cancel_servo()
        time.sleep(0.2)

        busy = servo_busy()
        status = "⚠ 运动可能已完成" if busy else "✓ 打断成功"
        print(f"  servo_busy() = {busy}  → {status}")

    finally:
        # ================================================================
        # 4. 关闭 ServoJ
        # ================================================================
        print("\n--- 关闭伺服跟踪模式 ---")
        close_servo_movel(sock_udp)

        # ================================================================
        # 5. 伺服下电 + 断开连接
        # ================================================================
        print("\n--- 伺服下电 ---")
        set_current_mode(sock_tcp, 0)
        set_servo_poweroff(sock_tcp)

        print("--- 断开连接 ---\n")
        disconnect_robot(sock_udp)
        disconnect_robot(sock_tcp)

    print("=== Servo MoveL 测试完毕 ===")


if __name__ == "__main__":
    main()
