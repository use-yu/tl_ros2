# /**
#  * ex_digital_io.py
#  * @brief  模式与 IO 控制接口（包括设置机械臂模式、设置数字 IO 输出、查询数字 IO 输出、获取所有 IO 输入状态）
#  * @attention 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 ex_digital_io.py
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


def enable_servo(socket_fd):
    """使能上电 —— 先清错，再检查伺服状态，报警则报错退出，最后上电"""
    print("--- 检查伺服状态 ---")

    # 1. 先清除控制器错误和报警，再下电复位确保状态同步
    ret = clear_error(socket_fd)
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
        print("[INFO] 机械臂使能上电成功（servo_state = %d）" % state)
    else:
        print("[ERROR] 机械臂使能上电失败（servo_state = %d）" % state)


def disable_servo(socket_fd):
    """@brief 使能下电"""
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
        return
    elif state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print("[INFO] 机械臂使能下电成功（servo_state = %d）" % state)
        return
    print("[INFO] 机械臂使能下电失败（servo_state = %d）" % state)


def demo_set_remote_function(socket_fd):
    """设置IO默认运行编号"""
    print("[INFO] **********设置IO默认运行编号**********")
    robot_num = 1
    general = RemoteControl()
    general.startPort = 1
    general.startValue = 1
    general.stopPort = 2
    general.stopValue = 1
    general.pausePort = 3
    general.pauseValue = 1
    general.clearStashPort = 4
    general.clearStashValue = 1
    general.faultResetPort = 5
    general.faultResetValue = 1
    program = VectorRemoteProgram()
    for i in range(10):
        p = RemoteProgram()
        p.port = i + 1
        p.value = 1
        program.append(p)
    ret = set_remote_function(socket_fd, robot_num, general, program)
    if ret != SUCCESS:
        print("[ERROR] 设置IO默认运行编号失败，错误码: %d" % ret)
        return
    print("[INFO] 设置IO默认运行编号成功")
    print()


def demo_get_robot_running_state(socket_fd):
    """查询流程图编程运行状态"""
    print("[INFO] **********查询流程图编程运行状态**********")
    status = 0
    ret, status = get_robot_running_state(socket_fd, status)
    if ret != SUCCESS:
        print("[ERROR] 查询流程图编程运行状态失败，错误码: %d" % ret)
        return
    if status == 0:
        print("[INFO] 机械臂处于停止状态 (status = %d)" % status)
    elif status == 1:
        print("[INFO] 机械臂处于暂停状态 (status = %d)" % status)
    elif status == 2:
        print("[INFO] 机械臂处于运行状态 (status = %d)" % status)
    print()


def main():
    # 定义IP地址和端口
    # 解析命令行参数: python3 ex_digital_io.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]
    port = "6001"

    # 获取 SDK 版本
    version = get_library_version()
    print("[INFO] SDK 版本: %s" % version)

    # 连接控制器
    print("[INFO] 连接控制器...")
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print("[ERROR] 连接失败，程序退出!")
        return -1
    print("[INFO] 连接成功 (socket=%d)" % socket_fd)

    # 使能上电
    enable_servo(socket_fd)

    # ----设置机器人模式----
    print("[INFO] **********设置机器人模式**********")
    mode = 1
    ret = set_current_mode(socket_fd, mode)
    if ret != SUCCESS:
        print("[ERROR] 设置机器人模式失败，程序退出!")
        return -1
    print("[INFO] 设置机器人模式成功")
    print()

    # ----设置数字 IO 输出----
    print("[INFO] **********设置数字 IO 输出**********")
    io_port = 1
    value = 0
    ret = set_digital_output(socket_fd, io_port, value)
    if ret != SUCCESS:
        print("[ERROR] 设置数字 IO 输出失败，程序退出!")
        return -1
    print("[INFO] 设置数字 IO 输出成功")
    print()

    # ----查询数字 IO 输出----
    print("[INFO] **********查询数字 IO 输出**********")
    outputValues = VectorInt()
    ret = get_digital_output(socket_fd, outputValues)
    if ret != SUCCESS:
        print("[ERROR] 查询数字 IO 输出失败，程序退出!")
        return -1
    print("[INFO] 查询数字 IO 输出成功，状态为：", end="")
    for state in outputValues:
        print("%d " % state, end="")
    print()

    # ----获取所有 IO 输入状态----
    print("[INFO] **********获取所有 IO 输入状态**********")
    inputValues = VectorInt()
    ret = get_digital_input(socket_fd, inputValues)
    if ret != SUCCESS:
        print("[ERROR] 查询数字 IO 输入失败，程序退出!")
        return -1
    print("[INFO] 查询数字 IO 输入成功，状态为：", end="")
    for state in inputValues:
        print("%d " % state, end="")
    print()

    # ----设置IO默认运行编号----
    demo_set_remote_function(socket_fd)

    # ----查询流程图编程运行状态----
    demo_get_robot_running_state(socket_fd)

    # 使能下电
    disable_servo(socket_fd)

    # 断开控制器连接
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        return -1
    print("[INFO] 断开连接成功")

    return 0


if __name__ == "__main__":
    main()
