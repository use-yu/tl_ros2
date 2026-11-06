# /**
#  * ex_move_control.py
#  * @brief 运动控制相关接口（包括MoveJ、MoveL）
#  * @attention
#  *   - 运行模式：示教模式
#  *   - 开始前需: connect_robot → set_servo_state(1) → set_servo_poweron
#  *   - 退出前需: set_servo_poweroff → disconnect_robot
#  *   - 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 ex_move_control.py
#  */

import sys
import os
import platform
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
import time


# ============================================================
# 工具函数
# ============================================================

def print_result(msg, ret):
    """输出操作结果

    @param msg 操作描述
    @param ret 返回码（Result枚举值）
    """
    if ret == SUCCESS:
        print("  [成功] " + msg)
    else:
        print("  [失败] " + msg + "，错误码: " + str(ret))


def print_separator(title):
    """输出分隔标题"""
    print("\n========== " + title + " ==========")


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
        print("[INFO] 机械臂使能上电成功（servo_state = " + str(state) + "）")
    else:
        print("[ERROR] 机械臂使能上电失败（servo_state = " + str(state) + "）")


def disable_servo(socket_fd):
    """使能下电

    @param socket_fd 6001端口 socket
    """
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
    elif state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print("[INFO] 机械臂使能下电成功（servo_state = " + str(state) + "）")
        return

    print("[INFO] 机械臂使能下电失败（servo_state = " + str(state) + "）")


def wait_motion_done(socket_fd, timeout_sec):
    """轮询机器人运行状态等待运动完成，替代固定延时

    @param socket_fd TCP socket
    @param timeout_sec 超时秒数
    @return True 运动完成 / False 超时
    """
    deadline = time.time() + timeout_sec
    last_print = 0

    time.sleep(0.1)

    while time.time() < deadline:
        ret, state = get_robot_running_state(socket_fd, -1)
        if ret == SUCCESS and state == 0:
            print("  [信息] 运动完成")
            return True
        time.sleep(0.2)

        elapsed = int(time.time() - (deadline - timeout_sec))
        if elapsed - last_print >= 5:
            print(f"  [信息] 等待运动中... (已过 {elapsed}s)")
            last_print = elapsed

    print(f"  [WARN] 等待运动超时 ({timeout_sec}s)")
    return False


# ============================================================
# 主函数
# ============================================================

def main():
    # ---- 定义IP地址和端口 ----
    # 解析命令行参数: python3 ex_move_control.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]
    port = "6001"

    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[INFO] SDK 版本: " + version)

    # ---- 连接控制器 ----
    print("[INFO] 连接控制器...")
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print("[ERROR] 连接失败，程序退出!")
        return -1
    print("[INFO] 连接成功 (socket=" + str(socket_fd) + ")")

    # ---- 切换为示教模式 ----
    print("--- 切换示教模式 ---")
    ret = set_current_mode(socket_fd, 0)
    if ret != SUCCESS:
        print("[ERROR] 切换示教模式失败，程序退出!")
        return -1
    print("[INFO] 已切换为示教模式")
    print()

    # ---- 使能上电 ----
    enable_servo(socket_fd)

    # ---- 设置全局速度 ----
    ret = set_speed(socket_fd, 30)
    print_result("set_speed", ret)
    time.sleep(0.2)

    # ---- MoveJ运动控制 ----
    print("[INFO] **********MoveJ运动控制**********")
    cmd = MoveCmd()
    cmd.targetPosType = PosType_PType
    cmd.targetPosName = ""
    cmd.coord = Coord.JOINT          # 关节坐标系
    cmd.velocity = 40          # 关节速度百分比 (1,100]
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0                 # 平滑参数 [0,5]
    cmd.targetPosValue = VectorDouble([
        20.0, 10.0, -10.0,
        0.0, 0.0, 0.0, 0.0])  # 关节坐标下单位为度
    ret = robot_movej(socket_fd, cmd)
    if ret != SUCCESS:
        print("[ERROR] MoveJ运动控制失败，程序退出!")
        return -1
    # 等待运动到位
    wait_motion_done(socket_fd, 30)

    print("[INFO] MoveJ运动控制成功")
    print()

    # ---- MoveL运动控制 ----
    print("[INFO] **********MoveL运动控制**********")
    cmd.targetPosName = ""
    cmd.coord = Coord.BASE      # 基坐标系
    cmd.targetPosValue = VectorDouble([
        280, 90.0, 270,
        3.14, 0.0, 0.0])  # 基坐标下位移单位是mm，旋转单位是弧度
    ret = robot_movel(socket_fd, cmd)
    if ret != SUCCESS:
        print("[ERROR] MoveL运动控制失败，程序退出!")
        return -1
    # 等待运动到位
    wait_motion_done(socket_fd, 30)

    print("[INFO] MoveL运动控制成功")
    print()

    # ---- 使能下电 ----
    disable_servo(socket_fd)

    # ---- 断开控制器连接 ----
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        return -1
    print("[INFO] 断开连接成功")

    return 0


if __name__ == "__main__":
    main()
