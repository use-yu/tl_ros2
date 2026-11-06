#!/usr/bin/env python3
#
# ex_error_zero.py
# @brief 错误处理接口（清除错误）
# @attention 确保机械臂供电正常、网络通信正常
# @warning
#   set_axis_zero_position（设置关节零位）属于高级标定操作，
#   错误设置会导致机器人定位偏差甚至机械碰撞，默认已注释。
#   如需使用，请仔细阅读控制器手册并在专业人员指导下操作。
# @note 运行步骤
#       运行: python3 ex_error_zero.py
#

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
    """使能伺服下电"""
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


def main():
    # 定义IP地址和端口
    # 解析命令行参数: python3 ex_error_zero.py [-ip 192.168.1.13]，默认 192.168.1.13
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
        return -1
    print("[INFO] 连接成功 (socket=" + str(socket_fd) + ")")

    # ----清除错误----
    print("[INFO] **********清除错误**********")
    ret = clear_error(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 清除错误失败，程序退出!")
        return -1
    print("[INFO] 清除错误成功")
    print("")

    # 使能上电
    enable_servo(socket_fd)

    # ----设置指定关节零位（危险操作，默认注释）----
    # ⚠️  危险: set_axis_zero_position 会修改机器人关节零位标定，
    #    错误的值会导致所有后续运动出现定位偏差，严重时可能造成机械碰撞。
    #    仅在专业指导下、确认当前零位确需校正时使用。
    #    取消下面代码的注释前，请确认 axis 值正确且已理解全部后果。
    #
    # print("[INFO] **********设置指定关节零位**********")
    # axis = 1
    # ret = set_axis_zero_position(socket_fd, axis)
    # if ret != SUCCESS:
    #     print("[ERROR] 设置指定关节零位失败，程序退出!")
    #     return -1
    # print("[INFO] 设置指定关节零位成功")
    # print("")

    # ----标定六维力传感器零位/重心----
    # 操作前请确保:
    #   1. 机械臂末端无负载或已安装标准负载
    #   2. 机械臂处于竖直姿态（关节各轴在合理位置）
    #   3. 周围安全，标定过程中机械臂会缓慢运动
    print("[INFO] **********标定六维力传感器零位/重心**********")
    print("[INFO] 即将执行六维力传感器标定，请确认:")
    print("  1. 机械臂末端无负载（或已安装标准负载）")
    print("  2. 机械臂周围安全无人")
    confirm = input("  是否继续? (y/n): ")
    if confirm.lower() == 'y':
        success = False
        ret, success = mark_base_sensor(socket_fd, success)
        if ret != SUCCESS:
            print("[ERROR] 标定六维力传感器失败，错误码: " + str(ret))
        elif success:
            print("[INFO] 标定六维力传感器成功")

            # 查询标定后的基础参数
            base_param = SensorBaseParam()
            ret = get_sensor_base_param(socket_fd, base_param)
            if ret == SUCCESS:
                print("  传感器质量: " + str(base_param.sensorMass))
                print("  质心 - X: " + str(base_param.sensorMassCenterX)
                      + ", Y: " + str(base_param.sensorMassCenterY)
                      + ", Z: " + str(base_param.sensorMassCenterZ))
                print("  标零状态: " + ("已标零" if base_param.saveZero else "未标零"))
        else:
            print("[ERROR] 标定六维力传感器失败，标定结果为 false")
    else:
        print("  [信息] 用户取消标定")
    print("")

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
    exit(main())
