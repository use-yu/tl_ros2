# /**
#  * ex_error_callback.py
#  * @brief 错误/警告回调示例程序
#  * @attention
#  *   - 确保控制器网络通信正常
#  *   - 回调函数内不要做耗时操作或阻塞
#  *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#  * @note 运行步骤
#  *       运行: python3 ex_error_callback.py
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
# 回调函数（由用户实现，在收到错误/警告时被调用）
# ============================================================

def error_warning_callback(message_type: int, message: str, message_code: int):
    """
    处理接收到的错误和警告消息
    message_type: 消息类型
    message: 消息内容
    message_code: 消息代码
    """
    print(f"[回调] 类型={message_type}, 信息={message}, 代码={message_code}")


# ============================================================
# 工具函数
# ============================================================

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
        print_result("set_servo_state(1)", ret)
        time.sleep(0.5)

    print("  [信息] 上电使能...")
    ret = set_servo_poweron(socket_fd)
    print_result("set_servo_poweron", ret)
    if ret != SUCCESS:
        print(f"[ERROR] 上电失败: {ret}")
        return
    time.sleep(2)

    # 4. 确认结果
    _, state = get_servo_state(socket_fd, state)
    if state == 3:
        print("  [成功] 机械臂使能上电成功 (servo_state = " + str(state) + ")")
    else:
        print("  [失败] 机械臂使能上电失败 (servo_state = " + str(state) + ")")


# ============================================================
# 主程序
# ============================================================
if __name__ == "__main__":
    # 解析命令行参数: python3 ex_error_callback.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: " + version)

    # ---- 连接控制器 ----
    print_separator("连接控制器")
    sock = connect_robot(ip, "6001")
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        sys.exit(-1)
    print("[信息] 连接成功 (socket = " + str(sock) + ")")

    # ============================================================
    # 1. 注册错误/警告回调
    # ============================================================
    print_separator("注册回调")
    ret = set_receive_error_or_warnning_message_callback(sock, error_warning_callback)
    if ret == SUCCESS:
        print("  [成功] set_receive_error_or_warnning_message_callback")
    else:
        print("  [失败] set_receive_error_or_warnning_message_callback，错误码: " + str(ret))
        disconnect_robot(sock)
        sys.exit(-1)
    print("  [信息] 回调已注册，后续收到错误/警告时会自动触发")

    # ============================================================
    # 2. 切换示教模式
    # ============================================================
    print_separator("切换示教模式")
    ret = set_current_mode(sock, 0)
    print_result("set_current_mode(0=示教)", ret)

    # ============================================================
    # 3. 伺服上电
    # ============================================================
    print_separator("伺服上电")
    enable_servo(sock)

    # ============================================================
    # 4. 发送不可达运动指令，触发错误回调
    # ============================================================
    print_separator("发送不可达运动指令")

    print("--- 发送一个超出工作空间的关节角度，观察回调触发 ---")
    cmd = MoveCmd()
    cmd.targetPosType = PosType_data
    cmd.coord = Coord.JOINT     # 关节坐标系
    cmd.velocity = 20          # 低速，安全
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    # 14 维向量: 前7本体 + 后7外部轴，J1=999° 远超限位
    pos = VectorDouble([0.0] * 14)
    pos[0] = 999
    cmd.targetPosValue = pos
    ret = robot_movej(sock, cmd)
    print_result("robot_movej (不可达关节角度)", ret)
    if ret != SUCCESS:
        print("  [信息] 运动失败，控制器的错误消息应已通过回调输出")
    else:
        print("  [信息] 运动指令已发送（意外成功，等待反馈...）")

    # 等待回调处理完成
    time.sleep(1)

    # ============================================================
    # 5. 调用 get_origin_coord_to_target_coord 逆解失败，触发回调
    # ============================================================
    print_separator("逆解失败测试")

    print("--- 发送一个超出工作空间的目标坐标，触发逆解失败回调 ---")
    # 一个远超机器人工作空间的基坐标 (X=99999mm)
    unreachable_pose = VectorDouble()
    unreachable_pose.push_back(99999.0)  # X [mm]
    unreachable_pose.push_back(0.0)      # Y [mm]
    unreachable_pose.push_back(0.0)      # Z [mm]
    unreachable_pose.push_back(0.0)      # Rx [rad]
    unreachable_pose.push_back(0.0)      # Ry [rad]
    unreachable_pose.push_back(0.0)      # Rz [rad]
    unreachable_pose.push_back(0.0)      
    joint_result = VectorDouble()
    reference_pos = VectorDouble([0]*7)
    ret2 = get_origin_coord_to_target_coord(sock, Coord.BASE, unreachable_pose, Coord.JOINT, joint_result, 0, reference_pos)
    print_result("get_origin_coord_to_target_coord (不可达基坐标→关节逆解)", ret2)
    if ret2 != SUCCESS:
        print("  [信息] 逆解失败，期望控制器通过回调给出逆解失败的错误消息")
    else:
        print("  [信息] 逆解成功（意外），请确认坐标是否确实在工作空间内")

    time.sleep(1)

    # ============================================================
    # 5. 断开连接
    # ============================================================
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] 错误回调示例程序运行完毕")
