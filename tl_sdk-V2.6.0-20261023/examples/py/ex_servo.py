#!/usr/bin/env python3
#
# ex_servo.py
# @brief 伺服状态与速度控制示例程序
# @attention
#   - 确保机械臂供电正常、网络通信正常
#   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#   - 运行模式：示教模式
#   - 伺服状态机: 0=停止 → 1=就绪 → 3=运行（上电后）
#   - set_servo_state(1) 必须先于 set_servo_poweron 调用
#   - set_servo_poweroff 只能在 state=3 时调用，成功后回到 state=1
# @note 运行步骤
#       运行: python3 ex_servo.py
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


def servo_state_text(state):
    """获取伺服状态文本描述"""
    if state == 0:
        return "0=停止"
    elif state == 1:
        return "1=就绪"
    elif state == 2:
        return "2=报警"
    elif state == 3:
        return "3=运行"
    else:
        return "未知"


# ============================================================
# 参数设置函数（注：带 TODO 的数值需根据真实机械臂手册调整）
# ============================================================

def demo_restore_default_param_DH(sock):
    """恢复机械臂默认DH参数"""
    print_separator("恢复默认DH参数")
    robot_num = 1  # TODO: 根据实际机器人编号调整（1-4）
    ret = restore_default_param_DH(sock, robot_num)
    print_result("restore_default_param_DH", ret)
    print()


def demo_set_robot_joint_param(sock):
    """设置关节参数（最大速度/加速度/位置限位）"""
    print_separator("设置关节参数")
    print("  [信息] 遍历设置关节1~6的参数，以下数值需根据真实机械臂手册调整")
    for axis_id in range(1, 7):
        param = RobotJointParam()
        # --- 运动学参数（不可随意修改，需按实际减速比/编码器设置）---
        param.reducRatio = 100.0          # TODO: 减速比
        param.encoderResolution = 17      # TODO: 编码器位数
        param.direction = 1               # TODO: 方向（1=正向，-1=反向）
        # --- 位置限位（单位：度）---
        param.posSWLimit = 180.0          # TODO: 轴正限位(°)
        param.negSWLimit = -180.0         # TODO: 轴反限位(°)
        # --- 速度参数（单位：度/秒）---
        param.ratedRotSpeed = 100.0       # TODO: 电机额定正转速
        param.ratedDeRotSpeed = 100.0     # TODO: 电机额定反转速
        param.maxRotSpeed = 150.0         # TODO: 电机最大正转速
        param.maxDeRotSpeed = 150.0       # TODO: 电机最大反转速
        param.ratedVel = 80.0             # TODO: 额定正速度
        param.deRatedVel = 80.0           # TODO: 额定反速度
        # --- 加减速度 ---
        param.maxAcc = 100.0              # TODO: 最大加速度
        param.maxDecel = 100.0            # TODO: 最大减速度

        ret = set_robot_joint_param(sock, axis_id, param)
        print_result("set_robot_joint_param (关节%d)" % axis_id, ret)
    print()


def demo_set_cartesian_params(sock):
    """设置笛卡尔参数（最大线速度/线加速度/角速度）"""
    print_separator("设置笛卡尔参数")
    print("  [信息] 以下数值需根据真实机械臂手册调整")

    param = CartesianParam()
    param.maxSpeed = 500.0                # TODO: 最大线速度(mm/s)
    param.maxAcc = 1000.0                 # TODO: 最大线加速度(mm/s²)
    param.maxDec = 1000.0                 # TODO: 最大线减速度(mm/s²)
    param.maxAttitudeVel = 100.0          # TODO: 最大姿态角速度(°/s)
    param.speedLimitMode = 0              # TODO: 速度限制方式（0=位姿，1=位置）

    ret = set_cartesian_params(sock, param)
    print_result("set_cartesian_params", ret)
    print()


def demo_set_controller_ip(sock):
    """配置有线网口IP地址（危险操作，修改后控制器IP会立即变更导致断连）"""
    print_separator("配置控制器有线网口IP")
    print("  [警告] 修改控制器IP会立即生效并导致TCP断连！")
    print("  [警告] 请确认新IP与你的电脑在同一网段，否则无法重新连接！")
    # TODO: 根据实际网络环境修改以下参数
    name = "eth0"                         # TODO: 网口名称
    address = "192.168.1.13"              # TODO: 目标IP地址
    gateway = "192.168.1.1"               # TODO: 网关地址
    dns = "8.8.8.8"                       # TODO: DNS服务器
    ret = set_controller_ip(sock, name, address, gateway, dns)
    print_result("set_controller_ip", ret)
    print()


def demo_restore_network_factory_settings(sock):
    """恢复网络出厂设置（危险操作，会重置所有网络参数并可能断连）"""
    print_separator("恢复网络出厂设置")
    print("  [警告] 恢复网络出厂设置会重置控制器所有网络参数！")
    print("  [警告] 执行后需要重新配置网络才能连接！")
    ret = restore_network_factory_settings(sock)
    print_result("restore_network_factory_settings", ret)
    print()


# ============================================================
# 主函数
# ============================================================

def main():
    # 解析命令行参数: python3 ex_servo.py [-ip 192.168.1.13]，默认 192.168.1.13
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
        return -1
    print("[信息] 连接成功 (socket = " + str(sock) + ")")

    # ============================================================
    # 1. 查询初始伺服状态
    # ============================================================
    print_separator("查询初始伺服状态")

    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 初始伺服状态: " + servo_state_text(state))
    else:
        print_result("get_servo_state", ret)

    # ============================================================
    # 2. 切换为示教模式
    # ============================================================
    # 伺服状态操作通常在示教模式下进行（0=示教）
    print_separator("切换示教模式")
    ret = set_current_mode(sock, 0)
    print_result("set_current_mode(0)", ret)
    time.sleep(1)

    # ============================================================
    # 3. 清错处理（在设置伺服状态前先清除控制器错误和报警）
    # ============================================================
    print_separator("清错")
    ret = clear_error(sock)
    print_result("clear_error", ret)
    time.sleep(0.3)
    ret = set_servo_poweroff(sock)
    print_result("set_servo_poweroff", ret)
    time.sleep(0.3)

    # 检查伺服状态
    state = -1
    ret, state = get_servo_state(sock, state)
    if ret != SUCCESS:
        print(f"[ERROR] 获取伺服状态失败: {ret}")
        disconnect_robot(sock)
        return -1
    print(f"  [信息] 当前伺服状态: {servo_state_text(state)}")

    if state == 2:
        # state=2 为报警状态，此时无法上电
        print("[ERROR] 伺服处于报警状态，无法继续上电，请排查报警原因后重试")
        disconnect_robot(sock)
        return -1

    # ============================================================
    # 4. 设置伺服就绪 → 上电 → 验证状态变化
    # ============================================================
    print_separator("伺服上电流程")

    # 4a. 设置伺服就绪（state=1）
    print("[步骤 4a] 设置伺服就绪 (set_servo_state=1)")
    ret = set_servo_state(sock, 1)
    if ret == OPERATION_NOT_ALLOWED:
        print("  [信息] 机械臂已处于就绪或运行状态，无需设置")
    else:
        print_result("set_servo_state(1)", ret)
        if ret != SUCCESS:
            disconnect_robot(sock)
            return -1
    time.sleep(0.5)

    # 验证当前状态
    state = -1
    _, state = get_servo_state(sock, state)
    print("  [信息] 当前伺服状态: " + servo_state_text(state))

    # 4b. 上电使能（state=1 → state=3）
    print("\n[步骤 4b] 上电使能 (set_servo_poweron)")
    ret = set_servo_poweron(sock)
    print_result("set_servo_poweron", ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1
    time.sleep(2)

    # 验证上电成功后的状态应为 3=运行
    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 上电后伺服状态: " + servo_state_text(state), end="")
        if state == 3:
            print("（上电成功）")
        print("")

    # ============================================================
    # 5. 速度查询与设置
    # ============================================================
    print_separator("速度查询与设置")

    # 5a. 查询当前速度
    speed = -1
    ret, speed = get_speed(sock, speed)
    if ret == SUCCESS:
        print("  [信息] 当前全局速度: " + str(speed) + "%")
    else:
        print_result("get_speed", ret)

    # 5b. 设置速度并验证
    # set_speed 范围 0<speed≤100，设置后影响后续所有运动的速度上限
    test_speeds = [50, 80, 30]
    for spd in test_speeds:
        print("\n[步骤 5] 设置全局速度: " + str(spd) + "%")
        ret = set_speed(sock, spd)
        print_result("set_speed", ret)
        time.sleep(0.2)

        current_speed = -1
        ret, current_speed = get_speed(sock, current_speed)
        if ret == SUCCESS:
            print("  [信息] 读取确认: 当前速度 = " + str(current_speed) + "%")

    # ============================================================
    # 6. 其他参数设置（需上电状态，执行时请注意安全）
    # ============================================================
    print_separator("其他参数设置")

    # 6a. 恢复机械臂默认DH参数
    # demo_restore_default_param_DH(sock)

    # ⚠️ 不建议测试以下设置，可能会导致机械臂运动异常或损坏，请谨慎操作
    # # 6b. 设置关节参数（最大速度/加速度/位置限位）
    # demo_set_robot_joint_param(sock)

    # # 6c. 设置笛卡尔参数（最大线速度/线加速度/角速度）
    # demo_set_cartesian_params(sock)

    # ⚠️ 以下为危险操作，默认注释。取消注释前请确认对网络环境的影响！
    # demo_set_controller_ip(sock)
    # demo_restore_network_factory_settings(sock)

    # ============================================================
    # 7. 下电 → 停止 → 验证状态变化
    # ============================================================
    print_separator("伺服下电流程")

    # 7a. 下电（state=3 → state=1）
    print("[步骤 7a] 下电 (set_servo_poweroff)")
    ret = set_servo_poweroff(sock)
    if ret == SUCCESS:
        print("  [成功] set_servo_poweroff")
    elif ret == RECEIVE_FAILED:
        # 下电后网络可能自动断开，-1 视为正常
        print("  [信息] set_servo_poweroff 返回 -1（网络可能已断开）")
    else:
        print_result("set_servo_poweroff", ret)
    time.sleep(0.5)

    # 7b. 验证下电后的伺服状态
    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 下电后伺服状态: " + servo_state_text(state))

    # 7c. 设置伺服停止（state=1 → state=0）
    print("\n[步骤 7c] 设置伺服停止 (set_servo_state=0)")
    ret = set_servo_state(sock, 0)
    if ret == OPERATION_NOT_ALLOWED:
        print("  [信息] 当前状态不允许直接设为停止")
    else:
        print_result("set_servo_state(0)", ret)
    time.sleep(0.3)

    # 最终状态确认
    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 最终伺服状态: " + servo_state_text(state))

    # ============================================================
    # 8. 断开连接
    # ============================================================
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] 伺服状态示例程序运行完毕")
    return 0


if __name__ == "__main__":
    exit(main())
