# /**
#  * ex_coord_tool.py
#  * @brief 坐标系与工具管理相关接口（包括设置工具手参数、设置用户坐标系、设置用户坐标编号、获取当前工作坐标系、设置当前工作坐标系）
#  * @attention 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 ex_coord_tool.py
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


# ----工具函数----
def print_result(msg, ret):
    """打印操作结果"""
    if ret == SUCCESS:
        print("  [成功] " + msg)
    else:
        print("  [失败] " + msg + "，错误码: " + str(ret))


def print_separator(title):
    """打印分隔标题"""
    print("\n========== " + title + " ==========")


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
        print("[INFO] 机械臂使能上电成功（servo_state = " + str(state) + "）")
    else:
        print("[ERROR] 机械臂使能上电失败（servo_state = " + str(state) + "）")


def disable_servo(socket_fd):
    """使能下电 —— 安全关闭伺服"""
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
    elif state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print("[INFO] 机械臂使能下电成功（servo_state = " + str(state) + "）")
        return
    else:
        pass
    print("[INFO] 机械臂使能下电失败（servo_state = " + str(state) + "）")


# ============================================================
# 主程序
# ============================================================
if __name__ == "__main__":
    # 定义IP地址和端口
    # 解析命令行参数: python3 ex_coord_tool.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]
    port = "6001"

    # 获取 SDK 版本
    version = get_library_version()
    print("[INFO] SDK 版本: " + version)

    # 连接控制器
    print("[INFO] 连接控制器...")
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print("[ERROR] 连接失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 连接成功 (socket=" + str(socket_fd) + ")")

    # 使能上电
    enable_servo(socket_fd)

    # 注意: set_tool_hand_param/set_user_coordinate_data 涉及配置修改，
    #       错误的值可能导致路径偏差，请确认参数正确。
    # ----设置工具手参数---
    print("[INFO] **********设置工具手参数**********")
    tool_num = 1
    tool_param = ToolParam()
    tool_param.X = 10
    tool_param.Y = 10
    tool_param.Z = 10
    tool_param.A = 0
    tool_param.B = 0
    tool_param.C = 0
    ret = set_tool_hand_param(socket_fd, tool_num, tool_param)
    if ret != SUCCESS:
        print("[ERROR] 设置工具手参数失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 设置工具手参数成功")
    print()

    # ----配置用户坐标系----
    print("[INFO] **********配置用户坐标系**********")
    userNum = 1
    user_coord = [10, 20, 30, 0, 0, 0, 0]
    ret = set_user_coordinate_data(socket_fd, userNum, user_coord)
    if ret != SUCCESS:
        print("[ERROR] 配置用户坐标系失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 配置用户坐标系成功")
    print()

    # ----激活用户坐标编号 切换当前激活的用户坐标系编号----
    print("[INFO] **********激活用户坐标编号**********")
    userNum = 2
    ret = set_user_coord_number(socket_fd, userNum)
    if ret != SUCCESS:
        print("[ERROR] 激活用户坐标编号失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 激活用户坐标编号成功")
    print()

    # ----设置当前工作坐标系----
    print("[INFO] **********设置当前工作坐标系**********")
    coord = Coord.TOOL
    ret = set_current_coord(socket_fd, coord)
    if ret != SUCCESS:
        print("[ERROR] 设置当前工作坐标系失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 设置当前工作坐标系成功")
    print()

    # ----获取当前工作坐标系----
    print("[INFO] **********获取当前工作坐标系**********")
    coord = Coord.JOINT
    ret, coord = get_current_coord(socket_fd, coord)
    if ret != SUCCESS:
        print("[ERROR] 获取当前工作坐标系失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 当前坐标系编号为：" + str(int(coord)))
    print()

        # ----设置工具包络球参数----
    print("[INFO] **********设置工具包络球参数**********")
    tool_number = 1
    range_param = ToolCoordinateRange()
    range_param.max_range_x = 200
    range_param.min_range_x = -200
    range_param.max_range_y = 200
    range_param.min_range_y = -200
    range_param.max_range_z = 200
    range_param.min_range_z = -200
    range_param.tool_hand_method = 1
    ret = set_tool_coordinate_range(socket_fd, tool_number, range_param)
    if ret != SUCCESS:
        print("[ERROR] 设置工具包络球参数失败，错误码: " + str(ret))
        sys.exit(-1)
    print("[INFO] 设置工具包络球参数成功")
    print()

    # ----三点法自动设置用户坐标系----
    print("[INFO] **********三点法自动设置用户坐标系**********")
    print("[INFO] 请将机械臂移动到用户坐标系原点O后按Enter键继续...")
    input()
    ret = calibration_oxy(socket_fd, 1, "O")
    if ret != SUCCESS:
        print("[ERROR] 标定原点O失败，错误码: " + str(ret))
        sys.exit(-1)
    print("[INFO] 请将机械臂移动到X轴正方向上一点后按Enter键继续...")
    input()
    ret = calibration_oxy(socket_fd, 1, "X")
    if ret != SUCCESS:
        print("[ERROR] 标定X轴方向点失败，错误码: " + str(ret))
        sys.exit(-1)
    print("[INFO] 请将机械臂移动到XY平面内Y轴正方向上一点后按Enter键继续...")
    input()
    ret = calibration_oxy(socket_fd, 1, "Y")
    if ret != SUCCESS:
        print("[ERROR] 标定Y轴方向点失败，错误码: " + str(ret))
        sys.exit(-1)
    ret = calculate_user_coordinate(socket_fd, 1)
    if ret != SUCCESS:
        print("[ERROR] 计算用户坐标系失败，错误码: " + str(ret))
        sys.exit(-1)
    print("[INFO] 三点法标定用户坐标系成功")
    print()

    # ----设置机械臂末端参数为默认值----
    print("[INFO] **********设置机械臂末端参数为默认值**********")
    ret = set_default_cartesian_params(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 设置机械臂末端参数为默认值失败，错误码: " + str(ret))
        sys.exit(-1)
    print("[INFO] 设置机械臂末端参数为默认值成功")
    print()

    # 使能下电
    disable_servo(socket_fd)

    # 断开控制器连接
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 断开连接成功")
