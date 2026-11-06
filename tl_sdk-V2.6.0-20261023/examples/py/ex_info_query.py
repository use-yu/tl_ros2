#!/usr/bin/env python3
#
# ex_info_query.py
# @brief 相关信息查询接口（包括获取 SDK 版本、控制器连接/断开、当前坐标、
#        运行状态、关节角度、DH 参数、六维力传感器数据、关节软件版本号、
#        碰撞防护等级、远程 IO 功能设置、电流环拖动示教灵敏度、工具坐标范围参数）
# @attention 确保机械臂供电正常、网络通信正常
# @note 运行步骤
#       运行: python3 ex_info_query.py
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
        print("[INFO] 机械臂使能上电成功（servo_state = " + str(state) + "）")
    else:
        print("[ERROR] 机械臂使能上电失败（servo_state = " + str(state) + "）")


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


def query_sensor_6d_data(socket_fd):
    """查询六维力传感器力信息"""
    print("[INFO] **********查询六维力传感器力信息**********")
    data = Sensor6DData()
    ret = get_sensor_6d_data(socket_fd, data)
    if ret != SUCCESS:
        print("[ERROR] 获取六维力传感器数据失败，错误码: " + str(ret))
        return
    print("[INFO] 传感器连接状态: " + ("已连接" if data.sensorConnected else "未连接"))
    print("[INFO] 原始数据: Fx=" + "{:.3f}".format(data.fxData) + ", Fy=" + "{:.3f}".format(data.fyData)
          + ", Fz=" + "{:.3f}".format(data.fzData) + ", Mx=" + "{:.3f}".format(data.mxData)
          + ", My=" + "{:.3f}".format(data.myData) + ", Mz=" + "{:.3f}".format(data.mzData))
    print("[INFO] 去皮数据: Fx=" + "{:.3f}".format(data.fxDataSubBase) + ", Fy=" + "{:.3f}".format(data.fyDataSubBase)
          + ", Fz=" + "{:.3f}".format(data.fzDataSubBase) + ", Mx=" + "{:.3f}".format(data.mxDataSubBase)
          + ", My=" + "{:.3f}".format(data.myDataSubBase) + ", Mz=" + "{:.3f}".format(data.mzDataSubBase))
    print("")


def show_joint_software_version(socket_fd):
    """查询关节软件版本号（query_joint_software_version）"""
    print("[INFO] **********查询关节软件版本号**********")
    for i in range(1, 8):
        ret, version = query_joint_software_version(socket_fd, i)
        if ret == SUCCESS:
            print("[INFO] 关节" + str(i) + " 软件版本: " + version)
        else:
            print("[ERROR] 查询关节" + str(i) + "软件版本失败，错误码: " + str(ret))
    print("")


def query_collision_detection_param(socket_fd):
    """查询碰撞防护等级"""
    print("[INFO] **********查询碰撞防护等级**********")
    param = CollisionPara()
    ret = get_collision_detection_param(socket_fd, param)
    if ret != SUCCESS:
        print("[ERROR] 获取碰撞防护等级失败，错误码: " + str(ret))
        return
    print("[INFO] 碰撞检测阈值（指令）: ", end="")
    for val in param.collisionDetection_run:
        print(str(val) + " ", end="")
    print("")
    print("[INFO] 碰撞检测阈值（点动）: ", end="")
    for val in param.collisionDetection_teach:
        print(str(val) + " ", end="")
    print("")
    print("[INFO] 指令位置响应时间: " + str(param.position_delay_time_ms_value) + " ms")
    print("[INFO] 误差允许时间: " + str(param.error_enable_time_ms_value) + " ms")
    print("[INFO] 机器人轴数: " + str(param.axisum))
    print("")


def query_remote_function(socket_fd):
    """查询远程IO功能设置（get_remote_function）"""
    print("[INFO] **********查询远程IO功能设置**********")
    robot_num = 1
    num = 0       # 出参：远程IO数量（22.07 版本无此参数，返回 -1）
    time = 0      # 出参：IO重复触发屏蔽时间
    general = RemoteControl()
    program = VectorRemoteProgram()  # 出参：远程程序参数，由接口填充
    # 绑定返回 (ret, num, time, general, program) 五元组，出参以返回值为准
    ret, num, time, general, program = get_remote_function(socket_fd, robot_num, num, time,
                                                           general, program)
    if ret != SUCCESS:
        print("[ERROR] 获取远程IO功能设置失败，错误码: " + str(ret))
        return
    print("[INFO] IO远程数量: " + str(num))
    print("[INFO] IO重复触发屏蔽时间: " + str(time) + " ms")
    print("[INFO] 通用功能远程IO参数:")
    print("[INFO]   启动端口: " + str(general.startPort) + ", 值: " + str(general.startValue))
    print("[INFO]   停止端口: " + str(general.stopPort) + ", 值: " + str(general.stopValue))
    print("[INFO]   暂停端口: " + str(general.pausePort) + ", 值: " + str(general.pauseValue))
    print("[INFO]   清错端口: " + str(general.clearStashPort) + ", 值: " + str(general.clearStashValue))
    print("[INFO]   复位端口: " + str(general.faultResetPort) + ", 值: " + str(general.faultResetValue))
    print("[INFO] 远程程序数量: " + str(len(program)))
    for i in range(len(program)):
        print("[INFO]   程序" + str(i + 1) + ": 端口=" + str(program[i].port) + ", 值=" + str(program[i].value))
    print("")


def query_current_teach_sensitivity(socket_fd):
    """获取电流环拖动示教灵敏度（get_current_teach_sensitivity）"""
    print("[INFO] **********获取电流环拖动示教灵敏度**********")
    friction_offsets = VectorDouble()
    ret = get_current_teach_sensitivity(socket_fd, friction_offsets)
    if ret != SUCCESS:
        print("[ERROR] 获取电流环拖动示教灵敏度失败，错误码: " + str(ret))
        return
    print("[INFO] 各关节电流环拖动示教灵敏度: ", end="")
    for i in range(len(friction_offsets)):
        print("J" + str(i + 1) + "=" + str(friction_offsets[i]) + " ", end="")
    print("")
    print("")


def query_tool_coordinate_range(socket_fd):
    """获取工具包络球参数（get_tool_coordinate_range）"""
    print("[INFO] **********获取工具包络球参数**********")
    tool_number = 1  # 工具编号，C++ 版使用 1
    range_param = ToolCoordinateRange()
    ret = get_tool_coordinate_range(socket_fd, tool_number, range_param)
    if ret != SUCCESS:
        print("[ERROR] 获取工具包络球参数失败，错误码: " + str(ret))
        return
    print("[INFO] X轴范围: [" + str(range_param.min_range_x) + ", " + str(range_param.max_range_x) + "]")
    print("[INFO] Y轴范围: [" + str(range_param.min_range_y) + ", " + str(range_param.max_range_y) + "]")
    print("[INFO] Z轴范围: [" + str(range_param.min_range_z) + ", " + str(range_param.max_range_z) + "]")
    print("[INFO] 工具握持方式: " + str(range_param.tool_hand_method))
    print("")


def main():
    # 定义IP地址和端口
    # 解析命令行参数: python3 ex_info_query.py [-ip 192.168.1.13]，默认 192.168.1.13
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

    # 使能上电
    enable_servo(socket_fd)

    print("")

    # ----获取机械臂当前坐标----
    print("[INFO] **********获取机械臂当前坐标**********")
    coord = Coord.JOINT
    current_position = VectorDouble()
    ret = get_current_position(socket_fd, coord, current_position)
    if ret != SUCCESS:
        print("[ERROR] 获取机械臂当前坐标失败，程序退出!")
        return -1
    print("[INFO] 机械臂当前坐标: ", end="")
    for pos in current_position:
        print(str(pos) + " ", end="")
    print("")
    print("")

    # ----获取机械臂运行状态----
    print("[INFO] **********获取机械臂运行状态**********")
    status = 0
    ret, status = get_robot_running_state(socket_fd, status)
    if ret != SUCCESS:
        print("[ERROR] 获取机械臂运行状态失败，程序退出!")
        return -1
    if status == 0:
        print("[INFO] 机械臂处于停止状态 (status = " + str(status) + ")")
    elif status == 1:
        print("[INFO] 机械臂处于暂停状态 (status = " + str(status) + ")")
    elif status == 2:
        print("[INFO] 机械臂处于运行状态 (status = " + str(status) + ")")
    print("")

    # ----获取指定关节在基坐标系中的位置----
    print("[INFO] **********获取指定关节在基坐标系中的位置**********")
    axis_num = 7
    joint_position = VectorDouble()
    ret = get_joint_position(socket_fd, axis_num, joint_position)
    if ret != SUCCESS:
        print(f"[ERROR] 获取{axis_num}关节在基坐标系中的位置失败，程序退出!")
        return -1
    print(f"[INFO] 机械臂{axis_num}关节在基坐标系中的位置: ", end="")
    for joint_pos in joint_position:
        print(str(joint_pos) + " ", end="")
    print("")
    print("")

    # ----获取机械臂DH参数----
    print("[INFO] **********获取机械臂DH参数**********")
    dh_params = RobotDHParam()
    ret = get_robot_dh_param(socket_fd, dh_params)
    if ret != SUCCESS:
        print("[ERROR] 获取机械臂DH参数失败，程序退出!")
        return -1
    print("[INFO] 机械臂DH参数: ", end="")
    print("L1=" + str(dh_params.L1) + ", L2=" + str(dh_params.L2) + ", L3=" + str(dh_params.L3)
          + ", L4=" + str(dh_params.L4) + ", L5=" + str(dh_params.L5) + ", L6=" + str(dh_params.L6)
          + ", L7=" + str(dh_params.L7) + ", L8=" + str(dh_params.L8) + ", L9=" + str(dh_params.L9)
          + ", L10=" + str(dh_params.L10) + ", L11=" + str(dh_params.L11) + ", L12=" + str(dh_params.L12)
          + ", L13=" + str(dh_params.L13) + ", L14=" + str(dh_params.L14) + ", L15=" + str(dh_params.L15)
          + ", L16=" + str(dh_params.L16) + ", L17=" + str(dh_params.L17) + ", L18=" + str(dh_params.L18)
          + ", L19=" + str(dh_params.L19) + ", L20=" + str(dh_params.L20)
          + ", Couple_Coe_1_2=" + str(dh_params.Couple_Coe_1_2)
          + ", Couple_Coe_2_3=" + str(dh_params.Couple_Coe_2_3)
          + ", Couple_Coe_3_2=" + str(dh_params.Couple_Coe_3_2)
          + ", Couple_Coe_3_4=" + str(dh_params.Couple_Coe_3_4)
          + ", Couple_Coe_4_5=" + str(dh_params.Couple_Coe_4_5)
          + ", Couple_Coe_4_6=" + str(dh_params.Couple_Coe_4_6)
          + ", Couple_Coe_5_6=" + str(dh_params.Couple_Coe_5_6)
          + ", dynamicLimit_max=" + str(dh_params.dynamicLimit_max)
          + ", dynamicLimit_min=" + str(dh_params.dynamicLimit_min)
          + ", pitch=" + str(dh_params.pitch)
          + ", sliding_lead_value=" + str(dh_params.sliding_lead_value)
          + ", uplift_lead_value=" + str(dh_params.uplift_lead_value)
          + ", spray_distance=" + str(dh_params.spray_distance)
          + ", threeAxisDirection=" + str(dh_params.threeAxisDirection)
          + ", fiveAxisDirection=" + str(dh_params.fiveAxisDirection)
          + ", twoAxisConversionRatio=" + str(dh_params.twoAxisConversionRatio)
          + ", threeAxisConversionRatio=" + str(dh_params.threeAxisConversionRatio)
          + ", amplificationRatio=" + str(dh_params.amplificationRatio)
          + ", conversionratio_x=" + str(dh_params.conversionratio_x)
          + ", conversionratio_y=" + str(dh_params.conversionratio_y)
          + ", conversionratio_z=" + str(dh_params.conversionratio_z)
          + ", conversionratio_J1=" + str(dh_params.conversionratio_J1)
          + ", conversionratio_J2=" + str(dh_params.conversionratio_J2)
          + ", conversionratio_J3=" + str(dh_params.conversionratio_J3)
          + ", upsideDown=" + str(dh_params.upsideDown)
          + ", hanyu.PC=" + str(dh_params.hanyu.PC)
          + ", hanyu.SP[0]=" + str(dh_params.hanyu.SP[0])
          + ", hanyu.SP[1]=" + str(dh_params.hanyu.SP[1])
          + ", hanyu.SP[2]=" + str(dh_params.hanyu.SP[2])
          + ", hanyu.TL[0]=" + str(dh_params.hanyu.TL[0])
          + ", hanyu.TL[1]=" + str(dh_params.hanyu.TL[1])
          + ", hanyu.TL[2]=" + str(dh_params.hanyu.TL[2]))
    print("")

    # ----查询六维力传感器力信息----
    query_sensor_6d_data(socket_fd)

    # ----查询关节软件版本号----
    show_joint_software_version(socket_fd)

    # ----查询碰撞防护等级----
    query_collision_detection_param(socket_fd)

    # ----获取IO默认运行编号----
    query_remote_function(socket_fd)

    # ----获取电流环拖动示教灵敏度----
    query_current_teach_sensitivity(socket_fd)

    # ----获取工具包络球参数----
    query_tool_coordinate_range(socket_fd)

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
