# /**
#  * ex_servoj.py
#  * @brief 关节实时跟踪（servoJ）示例程序
#  * @attention
#  *   - 确保机械臂供电正常、网络通信正常
#  *   - 需要双端口连接: 6001（控制）+ 7000（伺服高频透传）
#  *   - servoJ 周期约 10ms，回调内不要做耗时操作
#  *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#  *   - 运行模式: 透传模式（实时的将关节数据下发给控制器，控制器直接响应。连接7000端口后，并调用open_servoJ即可进入透传模式。在此之前建议set_current_mode(sock, 2)切换到运行模式）
#  *   - 开始前需: 双端口连接(6001+7000) → 上电 → open_servoJ
#  *   - 退出前需: close_servoJ → set_current_mode(0) → disconnect_robot(双端口)
#  * @note 运行步骤
#  *       运行: python3 ex_servoj.py
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


# ============================================================
# 主函数
# ============================================================

def main():
    # 解析命令行参数: python3 ex_servoj.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: " + version)

    # ---- 双端口连接 ----
    print_separator("双端口连接")
    sock = connect_robot(ip, "6001")
    if sock <= 0:
        print("[ERROR] 连接 6001 端口失败")
        return -1
    sock_servo = connect_robot(ip, "7000")
    if sock_servo <= 0:
        print("[ERROR] 连接 7000 端口失败")
        disconnect_robot(sock)
        return -1
    print("[信息] 双端口连接成功 (6001=" + str(sock) + ", 7000=" + str(sock_servo) + ")")

    # ---- 清错 ----
    clear_error(sock)
    time.sleep(0.3)

    # ---- 下电复位 ----
    set_servo_poweroff(sock)
    time.sleep(0.3)

    # ---- 切换示教模式并设置伺服就绪 ----
    print_separator("切换示教模式")
    ret = set_current_mode(sock, 0)
    if ret != SUCCESS:
        print(f"[ERROR] 切换示教模式失败: {ret}")
        disconnect_robot(sock)
        disconnect_robot(sock_servo)
        return -1
    print("  [信息] 已切换为示教模式")
    time.sleep(0.3)

    print("  [信息] 设置伺服就绪...")
    ret = set_servo_state(sock, 1)
    if ret != SUCCESS and ret != OPERATION_NOT_ALLOWED:
        print(f"[ERROR] 设置伺服就绪失败: {ret}")
        disconnect_robot(sock)
        disconnect_robot(sock_servo)
        return -1
    print_result("set_servo_state(1)", ret)
    time.sleep(0.5)

    # ---- 切换为运行模式 ----
    print_separator("切换运行模式")
    ret = set_current_mode(sock, 2)
    print_result("set_current_mode(2)", ret)
    time.sleep(2)

    # ============================================================
    # 打开 servoJ 模式
    # ============================================================
    print_separator("servoJ 关节实时跟踪")

    # 速度/加速度/加加速度约束（7 元素，单位 °/s, °/s², °/s³）
    # 注意: SDK 底层协议要求 servoJ 相关向量固定为 7 元素，传 6 元素会导致
    #       控制器协议解析错乱并断开 7000 连接
    vmax = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
    amax = [60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
    jmax = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

    ret = open_servoJ(sock_servo, vmax, amax, jmax)
    print_result("open_servoJ", ret)
    if ret != SUCCESS:
        set_servo_poweroff(sock)
        disconnect_robot(sock)
        disconnect_robot(sock_servo)
        return -1

    # ---- 动作 1: J1 从 0° 平滑转到 30° ----
    print("\n--- 动作 1: J1 转到 30° ---")
    # q 必须为 7 元素，与 open_servoJ 一致
    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(200):
        q[0] += (30.0 / 200.0)       # 每次增加 0.15°
        set_servoJ_pos(sock_servo, q)
        time.sleep(0.01)              # 100Hz
    print("  [信息] J1=30° 到位")
    time.sleep(1.0)

    # ---- 动作 2: J1 回到 0°，同时 J2 转到 20° ----
    print("\n--- 动作 2: J1 回零, J2 转到 20° ---")
    # q 必须为 7 元素
    q = [30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(200):
        q[0] -= (30.0 / 200.0)       # J1 逐步回到 0°
        q[1] += (20.0 / 200.0)       # J2 逐步增加到 20°
        set_servoJ_pos(sock_servo, q)
        time.sleep(0.01)
    print("  [信息] J1=0°, J2=20° 到位")
    time.sleep(1.0)

    # ---- 动作 3: J1=0°, J2 回到 0°, J3 转到 15° ----
    print("\n--- 动作 3: J2 回零, J3 转到 15° ---")
    # q 必须为 7 元素
    q = [0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(200):
        q[1] -= (20.0 / 200.0)       # J2 逐步回到 0°
        q[2] += (15.0 / 200.0)       # J3 逐步增加到 15°
        set_servoJ_pos(sock_servo, q)
        time.sleep(0.01)
    print("  [信息] J2=0°, J3=15° 到位")
    time.sleep(1.0)

    # ---- 动作 4: 所有轴回到零位 ----
    print("\n--- 动作 4: 所有轴回零 ---")
    # q 必须为 7 元素
    q = [0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(150):
        q[2] -= (15.0 / 150.0)       # J3 逐步回到 0°
        set_servoJ_pos(sock_servo, q)
        time.sleep(0.01)
    print("  [信息] 所有轴回零到位")
    time.sleep(1.0)

    # ---- 关闭 servoJ 模式 ----
    ret = close_servoJ(sock_servo)
    print_result("close_servoJ", ret)

    print("\n  [信息] servoJ 运动全部完成")

    # ============================================================
    # 切回示教模式（自动下电）& 断开
    # ============================================================
    print_separator("切回示教模式")
    ret = set_current_mode(sock, 0)
    print_result("set_current_mode(0) 切回示教模式（自动下电）", ret)
    time.sleep(0.3)

    disconnect_robot(sock)
    disconnect_robot(sock_servo)
    print("[信息] 已断开连接")

    print("\n[信息] servoJ 示例程序运行完毕")
    return 0


if __name__ == "__main__":
    main()
