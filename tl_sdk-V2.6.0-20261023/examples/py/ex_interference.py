# /**
#  * ex_interference.py
#  * @brief 干涉区检测 + MoveJ 运动示例：
#  *   1. get_tool_trigger_interfer 判断工具手（带工具手的干涉区立方体）在某位置是否触发干涉区
#  *   2. robot_movej 遍历 TEST_POSES 逐个执行 MoveJ；运动前用 get_tool_trigger_interfer
#  *      判断是否触发干涉区，触发则不执行运动（使用工具手 2）
#  * @attention
#  *   - 干涉区接口为只读查询，不会让机器人运动，无需伺服上电
#  *   - MoveJ 测试需要伺服上电，且运动前请确保机器人周围无障碍、运动参数安全
#  *   - 运动前会自动调用 get_tool_trigger_interfer 判断干涉区，触发则该位姿不运动
#  *   - 默认使用工具手 2（set_tool_hand_number(2)）：工具手 x 偏移 -20、z 偏移 100
#  *   - 示例干涉范围设置为 x[-400 250] y[-999 999] z[-999 999] 立方体
#  *   - 工具手干涉区设置为 x[-20 20] y[-10 10] z[-100 100] 立方体
#  *   - 确保机械臂供电正常、网络通信正常
#  *   - 默认控制器 IP 为 192.168.1.13，可通过命令行参数指定
#  *   - 测试位姿可在下方 TEST_POSES 中修改
#  * @note 运行步骤
#  *       运行: python3 ex_interference.py [-ip 192.168.1.13]
#  */

import sys
import os
import platform
import time

# 基于脚本所在目录构建 SDK Python 绑定的绝对路径
_script_dir = os.path.dirname(os.path.abspath(__file__))
# 自动检测平台，添加对应 SDK Python 绑定路径
if platform.system() == "Windows":
    sys.path.append(os.path.join(_script_dir, "../../lib/windows/python"))
elif platform.machine() in ("aarch64", "arm64"):
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/arm64/python"))
else:
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/x86/python"))

from tl_interface import *


# ============================================================
# 测试数据配置（可按实际干涉区配置修改）
# ============================================================
# get_tool_trigger_interfer: 工具手在世界坐标系下的位姿 [x, y, z, a, b, c]（单位 mm / 弧度）
TEST_POSES = [
    [209.0, 0.0, 355.0, 3.14, 0.5, 0.0],
    [209.0, 0.0, 355.0, 3.14, 0.0, 0.0],
    [137.0, 157.0, 355.0, 3.14, 0.0, -0.853],
]


# ============================================================
# MoveJ 运动测试参数（请自行填写）
# ============================================================
# robot_movej 运动参数
TEST_MOVEJ = {
    "targetPosType": PosType_data,   # 点位类型: PosType_data(数值) / PosType_PType(变量) / PosType_GPType(全局)
    "targetPosName": "",             # 变量名（targetPosType 为 PType/GPType 时填写，如 "P0001" / "GP0001"）
    "coord": Coord.BASE,             # 坐标系: Coord.JOINT(关节) / Coord.BASE(基坐标) / Coord.TOOL(工具) / Coord.USER(用户)
    "velocity": 20,                  # 速度: 关节坐标 (1,100] %；直角坐标 (1,1000] mm/s
    "acc": 10,                       # 加速度 (1,100]
    "dec": 10,                       # 减速度 (1,100]
    "pl": 0,                         # 平滑参数 [0,5]
    "timeout_sec": 30,               # 等待运动到位超时（秒）
}


# ============================================================
# 工具函数
# ============================================================
def print_vec(v):
    """格式化打印向量"""
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"


def test_tool_trigger_interfer(socket_fd, pose):
    """判断工具手（带工具手的干涉区立方体）在某位置是否触发干涉区

    @return True 触发干涉区 / False 未触发干涉区 / None 查询失败
    """
    if len(pose) < 6:
        print(f"[跳过] get_tool_trigger_interfer：位姿坐标维度不足（{len(pose)} < 6）")
        return None
    ret, trigger = get_tool_trigger_interfer(socket_fd, pose, False)
    if ret == SUCCESS:
        print(f"[成功] get_tool_trigger_interfer  pose={print_vec(pose)}  -> "
              f"{'触发干涉区' if trigger else '未触发干涉区'}")
        return trigger
    else:
        print(f"[失败] get_tool_trigger_interfer  pose={print_vec(pose)}  错误码: {ret}")
        return None


# ---- 伺服上电（MoveJ 运动前需要使能） ----
def enable_servo(socket_fd):
    """使能上电 —— 先清错，再检查伺服状态，报警则报错退出，最后上电

    @param socket_fd 6001端口 socket
    """
    print("--- 检查伺服状态 ---")

    # 1. 先清除控制器错误和报警，再下电复位确保状态同步
    clear_error(socket_fd)
    time.sleep(0.3)
    set_servo_poweroff(socket_fd)
    time.sleep(0.3)

    # 2. 检查伺服状态
    state = -1
    ret, state = get_servo_state(socket_fd, state)
    if ret != SUCCESS:
        print(f"[ERROR] 获取伺服状态失败: {ret}")
        return
    print(f"  [信息] 当前伺服状态: {state} (0=停止 1=就绪 2=报警 3=运行)")

    if state == 2:
        print("[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试")
        return

    if state == 3:
        print("[INFO] 机械臂已在使能上电状态")
        return

    # 3. 上电流程
    if state == 0:
        print("  [信息] 伺服处于停止状态，设置就绪...")
        ret = set_servo_state(socket_fd, 1)
        if ret != SUCCESS and ret != OPERATION_NOT_ALLOWED:
            print(f"[ERROR] 设置伺服就绪失败: {ret}")
            return
        time.sleep(0.5)

    print("  [信息] 上电使能...")
    ret = set_servo_poweron(socket_fd)
    if ret != SUCCESS:
        print(f"[ERROR] 上电失败: {ret}")
        return
    time.sleep(2)

    # 4. 确认结果
    _, state = get_servo_state(socket_fd, state)
    if state == 3:
        print(f"[INFO] 机械臂使能上电成功（servo_state = {state}）")
    else:
        print(f"[ERROR] 机械臂使能上电失败（servo_state = {state}）")


def disable_servo(socket_fd):
    """使能下电

    @param socket_fd 6001端口 socket
    """
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print(f"[INFO] 机械臂使能下电成功（servo_state = {state}）")
    else:
        print(f"[INFO] 机械臂已使能下电（servo_state = {state}）")


def wait_motion_done(socket_fd, timeout_sec):
    """轮询 get_robot_running_state 判断运动是否到位

    先确认运动已开始（运行状态=2 运行），再等待运动结束（运行状态=0 停止），
    避免指令刚下发、运动尚未启动时误判为已到位。

    @return True 运动到位（运行状态=0 停止）/ False 超时或未检测到运动
    """
    deadline = time.time() + timeout_sec
    started = False   # 是否已观察到运动开始（状态=2 运行）
    time.sleep(0.1)
    while time.time() < deadline:
        ret, state = get_robot_running_state(socket_fd, -1)
        if ret != SUCCESS:
            print(f"  [WARN] get_robot_running_state 查询失败: {ret}")
        elif state == 2:
            if not started:
                print("  [信息] 检测到运动开始（运行状态=2 运行）")
            started = True
        elif state == 0 and started:
            print("  [信息] 运动到位（运行状态=0 停止）")
            return True
        # 尚未检测到运动开始且状态为停止：运动可能刚下发未执行，继续轮询
        time.sleep(0.2)
    if not started:
        print("  [WARN] 未检测到运动开始，可能指令未生效或目标点即当前位置")
    else:
        print(f"  [WARN] 等待运动到位超时 ({timeout_sec}s)")
    return False


# ---- 测试2: MoveJ 运动（运动前判断干涉区） ----
def test_movej(socket_fd, pose):
    """执行 MoveJ 运动到指定位姿，并用 get_robot_running_state 判断是否到位

    @param pose 目标位姿（直角坐标 [x, y, z, a, b, c]，单位 mm/弧度）
    @return True 指令下发成功且运动到位 / False 失败或超时
    """
    cfg = TEST_MOVEJ
    if cfg["velocity"] is None or cfg["acc"] is None or cfg["dec"] is None:
        print("[跳过] robot_movej：尚未填写 velocity / acc / dec，请先修改 TEST_MOVEJ 配置")
        return False

    print(f"[信息] 目标点位: {print_vec(pose)}  "
          f"速度={cfg['velocity']}  acc={cfg['acc']}  dec={cfg['dec']}  pl={cfg['pl']}")

    cmd = MoveCmd()
    cmd.targetPosType = cfg["targetPosType"]
    cmd.targetPosName = cfg["targetPosName"]
    cmd.coord = cfg["coord"]
    cmd.velocity = cfg["velocity"]
    cmd.acc = cfg["acc"]
    cmd.dec = cfg["dec"]
    cmd.pl = cfg["pl"]
    cmd.targetPosValue = VectorDouble(pose)

    ret = robot_movej(socket_fd, cmd)
    if ret != SUCCESS:
        print(f"[失败] robot_movej 指令下发失败  错误码: {ret}")
        return False

    # 使用 get_robot_running_state 轮询判断是否到位，到位后才返回继续下一个点位
    arrived = wait_motion_done(socket_fd, cfg["timeout_sec"])
    if not arrived:
        print(f"[失败] robot_movej 未在 {cfg['timeout_sec']}s 内到位: {print_vec(pose)}")
        return False
    print("[成功] robot_movej 运动到位")
    return True


# ============================================================
# 主流程
# ============================================================
def main():
    # 解析命令行参数: python3 ex_interference.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    port = "6001"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    print("===== 干涉区检测 + MoveJ 测试 =====")
    print(f"目标控制器: {ip}:{port}")

    # 连接控制器
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print(f"[ERROR] 连接控制器失败: {socket_fd}")
        return -1
    print("[信息] 连接控制器成功")

    # 设置工具手编号为 2
    print("\n--- 设置工具手编号 ---")
    ret = set_tool_hand_number(socket_fd, 2)
    if ret == SUCCESS:
        print("[成功] set_tool_hand_number -> 工具手 2")
    else:
        print(f"[失败] set_tool_hand_number  错误码: {ret}")

    # 测试1: 工具手位姿触发干涉区查询
    print("\n--- 测试1: get_tool_trigger_interfer（工具手位姿）---")
    for pose in TEST_POSES:
        test_tool_trigger_interfer(socket_fd, pose)

    # 测试2: MoveJ 运动（运动前先判断干涉区，触发则不运动）
    print("\n--- 测试2: robot_movej（MoveJ 运动，运动前判断干涉区）---")
    enable_servo(socket_fd)
    for pose in TEST_POSES:
        trigger = test_tool_trigger_interfer(socket_fd, pose)
        if trigger is None:
            print(f"[跳过] 位姿 {print_vec(pose)} 干涉区查询失败，不执行 MoveJ")
            continue
        if trigger:
            print(f"[跳过] 位姿 {print_vec(pose)} 触发干涉区，不执行 MoveJ")
            continue
        print(f"[信息] 位姿 {print_vec(pose)} 未触发干涉区，执行 MoveJ")
        test_movej(socket_fd, pose)
    disable_servo(socket_fd)

    # 断开连接
    disconnect_robot(socket_fd)
    print("\n===== 测试完成 =====")
    return 0


if __name__ == "__main__":
    exit(main())
