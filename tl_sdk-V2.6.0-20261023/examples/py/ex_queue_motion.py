# /**
#  * ex_queue_motion.py
#  * @brief 队列运动示例程序 —— 展示队列运动完整工作流
#  * @attention
#  *   - 确保机械臂供电正常、网络通信正常
#  *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#  *   - 队列运动模式下，所有指令先在本地缓存，调用 send_to_controller 后一次性下发执行
#  *   - 演示四种队列运动：纯 MoveJ、纯 MoveL、MoveJ+MoveL 混合、暂停/恢复/停止
#  *   - 运行模式: 队列模式（在queue_motion_set_status(true)之后，即可进入队列模式，在此之前建议set_current_mode(sock, 2)切换到运行模式）
#  *   - 开始前需: connect_robot → 上电 → queue_motion_set_status(true)
#  *   - 退出前需: queue_motion_set_status(false) → set_current_mode(0) → set_servo_poweroff → disconnect_robot
#  * @note 运行步骤
#  *       运行: python3 ex_queue_motion.py
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
# MoveCmd 构建函数
# ============================================================

# @brief 构建 MoveCmd（关节坐标模式）
# @param joints 6 个关节角度值，单位 °（度），顺序 [J1, J2, J3, J4, J5, J6]
# @param vel    百分比 % (1,100]，最大关节速度的百分比，实际受全局速度限制
# @param acc    百分比 % (1,100]
# @param dec    百分比 % (1,100]
# @param pl     平滑参数 [0,5]
# @note 实际运动速度 = 全局速度 × (vel / 100)
def make_movej_cmd(joints, vel=30, acc=50, dec=50, pl=0):
    cmd = MoveCmd()
    cmd.coord = Coord.JOINT     # 关节坐标系
    cmd.velocity = vel
    cmd.acc = acc
    cmd.dec = dec
    cmd.pl = pl
    # 填充 targetPosValue（前7位为关节值，后7位为外部轴，补零到14位）
    # 注意: SWIG 要求 VectorDouble 类型，不能用 Python list 直接赋值
    target = VectorDouble()
    for i in range(14):
        target.append(joints[i] if i < len(joints) else 0.0)
    cmd.targetPosValue = target
    return cmd


# @brief 构建 MoveCmd（基坐标模式）
# @param pose 位姿 [X, Y, Z, Rx, Ry, Rz]，位置单位 mm，姿态单位 °
# @param vel    百分比 % (1,1000]，最大 TCP 线速度的百分比
# @param acc    百分比 % (1,100]
# @param dec    百分比 % (1,100]
# @param pl     平滑参数 [0,5]
# @note 实际运动速度 = 全局速度 × (vel / 100)
def make_movel_cmd(pose, vel=100, acc=50, dec=50, pl=0):
    cmd = MoveCmd()
    cmd.coord = Coord.BASE  # 基坐标系，可切换到 2：工具坐标系，3：用户坐标系
    cmd.velocity = vel
    cmd.acc = acc
    cmd.dec = dec
    cmd.pl = pl
    # 填充 targetPosValue（前7位为位姿值，后7位为外部轴，补零到14位）
    target = VectorDouble()
    for i in range(14):
        target.append(pose[i] if i < len(pose) else 0.0)
    cmd.targetPosValue = target
    return cmd


# ============================================================
# 队列运动执行器
# ============================================================

# @brief 执行一轮队列运动
# @param sock       socket 句柄
# @param name       队列名称（用于打印）
# @param fn_push    回调函数，在此函数中调用 queue_motion_push_back_*
#                   回调会在队列模式开启后、发送前被调用
# @return True 成功 / False 失败
def run_queue_motion(sock, name, fn_push):
    print_separator("队列运动：" + name)

    # 1. 打开队列运动模式（会清空远端队列）
    ret = queue_motion_set_status(sock, True)
    print_result("queue_motion_set_status(true)", ret)
    if ret != SUCCESS:
        return False

    # 等待队列模式状态稳定
    time.sleep(2)

    # 确认队列模式已开启
    # 注意: status 为输出参数，SWIG Python 绑定中传递 bool 类型的可变对象
    status = False
    _, status = queue_motion_get_status(sock, status)
    print("  [信息] 队列模式状态: " + ("已开启" if status else "已关闭"))

    # 2. 插入运动指令
    print("  [信息] 插入运动指令...")
    if not fn_push(sock):
        print("[ERROR] 插入运动指令失败")
        queue_motion_set_status(sock, False)
        return False

    # 3. 查询本地队列长度
    # 注意: size 为输出参数
    size = 0
    ret, size = queue_motion_size(sock, size)
    if ret == SUCCESS:
        print("  [信息] 本地队列长度: " + str(size) + " 条指令")

    # 4. 发送队列到控制器并等待执行完成
    print("  [信息] 发送队列到控制器...")
    ret = queue_motion_send_to_controller(sock, 0, False)
    print_result("send_to_controller", ret)
    if ret != SUCCESS:
        queue_motion_set_status(sock, False)
        return False

    print("  [信息] 等待运动执行完成...")
    time.sleep(0.2)
    while True:
        # 注意: running_state 为输出参数
        running_state = -1
        ret, running_state = get_robot_running_state(sock, running_state)
        if ret == SUCCESS and running_state == 0:
            break
        time.sleep(0.5)
    print("  [信息] 队列运动执行完毕")
    time.sleep(1.0)

    # 5. 关闭队列运动模式
    queue_motion_set_status(sock, False)
    # 等待队列模式完全关闭
    time.sleep(0.2)
    return True


# ============================================================
# 主函数
# ============================================================

def main():
    # 解析命令行参数: python3 ex_queue_motion.py [-ip 192.168.1.13]，默认 192.168.1.13
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

    # ---- 清错 ----
    clear_error(sock)
    time.sleep(0.3)
    set_servo_poweroff(sock)
    time.sleep(0.3)

    # ---- 检查模式并确保伺服运行 ----
    # 先获取当前模式
    current_mode = -1
    ret, current_mode = get_current_mode(sock, current_mode)
    if ret != SUCCESS or current_mode < 0:
        print("[ERROR] 获取当前模式失败: " + str(ret))
        disconnect_robot(sock)
        return -1
    print("  [信息] 当前模式: " + str(current_mode) + " (0=示教 1=远程 2=运行)")

    if current_mode == 0:
        # 示教模式 → 先确保伺服就绪上电，再切换为运行模式
        servo_state = -1
        ret, servo_state = get_servo_state(sock, servo_state)
        if ret != SUCCESS:
            print("[ERROR] 获取伺服状态失败: " + str(ret))
            disconnect_robot(sock)
            return -1
        print("  [信息] 伺服状态: " + str(servo_state) + " (0=停止 1=就绪 2=报警 3=运行)")

        if servo_state != 3:
            # 伺服未运行 → 执行上电流程
            print("  [信息] 伺服未运行，执行上电流程...")
            ret = set_servo_state(sock, 1)
            if ret != SUCCESS and ret != OPERATION_NOT_ALLOWED:
                print("[ERROR] 设置伺服就绪失败: " + str(ret))
                disconnect_robot(sock)
                return -1
            time.sleep(0.5)

            ret = set_servo_poweron(sock)
            if ret != SUCCESS:
                print("[ERROR] 上电失败: " + str(ret))
                disconnect_robot(sock)
                return -1
            print("  [信息] 上电成功")
            time.sleep(2)
        else:
            print("  [信息] 伺服已在运行状态")

        # 切换为运行模式
        print("  [信息] 切换为运行模式...")
        ret = set_current_mode(sock, 2)
        if ret != SUCCESS:
            print("[ERROR] 切换运行模式失败: " + str(ret))
            disconnect_robot(sock)
            return -1
        print("  [信息] 已切换为运行模式")
        time.sleep(1.0)

    elif current_mode == 2:
        # 已在运行模式，直接检查伺服状态
        servo_state = -1
        ret, servo_state = get_servo_state(sock, servo_state)
        if ret != SUCCESS or servo_state != 3:
            print("[ERROR] 伺服未在运行状态 (servo_state=" + str(servo_state) + ")")
            disconnect_robot(sock)
            return -1
        print("  [信息] 已在运行模式，伺服运行中 (servo_state=3)")
    else:
        print("[ERROR] 不支持的模式: " + str(current_mode) + "，期望示教(0)或运行(2)模式")
        disconnect_robot(sock)
        return -1

    # 延时2秒，保证状态完全就绪
    time.sleep(2.0)

    # ---- 设置全局速度 ----
    # 控制器默认全局速度为 5%，设为 30% 提升运动效率
    print_separator("设置全局速度")
    ret = set_speed(sock, 30)
    print_result("set_speed(30)", ret)

    # ============================================================
    # 演示 1: 纯 MoveJ 队列运动
    # ============================================================
    def push_pure_movej(sock):
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd([30, 0, 0, 0, 0, 0], 30))
        print("  push_back_moveJ:  J1=30°")
        print_result("push_back_moveJ", r)
        if r != SUCCESS:
            return False

        r = queue_motion_push_back_moveJ(sock, make_movej_cmd([30, 20, 0, 0, 0, 0], 30))
        print("  push_back_moveJ:  J1=30°, J2=20°")
        print_result("push_back_moveJ", r)
        if r != SUCCESS:
            return False

        r = queue_motion_push_back_moveJ(sock, make_movej_cmd([0, 20, 0, 0, 0, 0], 30))
        print("  push_back_moveJ:  J1=0°, J2=20°")
        print_result("push_back_moveJ", r)
        if r != SUCCESS:
            return False

        r = queue_motion_push_back_moveJ(sock, make_movej_cmd([0, 0, 0, 0, 0, 0], 20))
        print("  push_back_moveJ:  回零")
        print_result("push_back_moveJ", r)
        if r != SUCCESS:
            return False

        return True

    run_queue_motion(sock, "纯 MoveJ", push_pure_movej)

    # ============================================================
    # 演示 2: 纯 MoveL 队列运动
    # ============================================================
    def push_pure_movel(sock):
        r = queue_motion_push_back_moveL(sock, make_movel_cmd([300, 0, 300, 3.141, 0, 0], 300))
        print("  push_back_moveL:  X=300, Y=0, Z=300")
        print_result("push_back_moveL", r)
        if r != SUCCESS:
            return False

        r = queue_motion_push_back_moveL(sock, make_movel_cmd([400, 100, 300, 3.141, 0, 0], 300))
        print("  push_back_moveL:  X=400, Y=100, Z=300")
        print_result("push_back_moveL", r)
        if r != SUCCESS:
            return False

        r = queue_motion_push_back_moveL(sock, make_movel_cmd([300, 200, 300, 3.141, 0, 0], 300))
        print("  push_back_moveL:  X=300, Y=200, Z=300")
        print_result("push_back_moveL", r)
        if r != SUCCESS:
            return False

        r = queue_motion_push_back_moveL(sock, make_movel_cmd([229.882, 0, 358.680, 3.141, 0, 0], 200))
        print("  push_back_moveL:  回起始点")
        print_result("push_back_moveL", r)
        if r != SUCCESS:
            return False

        return True

    run_queue_motion(sock, "纯 MoveL", push_pure_movel)

    # ============================================================
    # 演示 3: MoveJ + MoveL 混合队列运动
    # ============================================================
    def push_mixed(sock):
        # 先用 MoveJ 调整姿态
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd([45, 0, 0, 0, 0, 0], 30))
        print("  push_back_moveJ:  J1=45°（调整关节姿态）")
        print_result("push_back_moveJ", r)
        if r != SUCCESS:
            return False

        # 再用 MoveL 走直线
        r = queue_motion_push_back_moveL(sock, make_movel_cmd([400, 0, 300, 3.141, 0, 0], 300))
        print("  push_back_moveL:  X=400, Y=0, Z=300")
        print_result("push_back_moveL", r)
        if r != SUCCESS:
            return False

        # MoveJ 换向
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd([-45, 0, 0, 0, 0, 0], 30))
        print("  push_back_moveJ:  J1=-45°（换向）")
        print_result("push_back_moveJ", r)
        if r != SUCCESS:
            return False

        # MoveL 走另一条直线
        r = queue_motion_push_back_moveL(sock, make_movel_cmd([300, 200, 300, 3.141, 0, 0], 300))
        print("  push_back_moveL:  X=300, Y=200, Z=300")
        print_result("push_back_moveL", r)
        if r != SUCCESS:
            return False

        # MoveJ 回零
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd([0, 0, 0, 0, 0, 0], 20))
        print("  push_back_moveJ:  回零")
        print_result("push_back_moveJ", r)
        if r != SUCCESS:
            return False

        return True

    run_queue_motion(sock, "MoveJ + MoveL 混合", push_mixed)

    # ============================================================
    # 演示 4: 队列运动控制（暂停/恢复/停止保持上电）
    # ============================================================
    print_separator("队列运动控制：暂停/恢复/停止")

    # 辅助：等待队列剩余数减少到 target_len
    def wait_queue_len(sock, target_len):
        while True:
            time.sleep(0.2)
            length = -1
            _, length = queue_motion_get_queuelen(sock, length)
            if length <= target_len or length == -1:
                break

    # 1. 开启队列模式
    r = queue_motion_set_status(sock, True)
    print_result("queue_motion_set_status(true)", r)
    if r != SUCCESS:
        print("[ERROR] 开启队列模式失败")
        set_current_mode(sock, 0)
        disconnect_robot(sock)
        return -1
    time.sleep(2)

    # 2. 插入 4 条 J5 摆动指令（简单清晰，便于观察控制效果）
    print("  [信息] 插入 4 条 MoveJ 指令（J5 摆动）")
    targets = [
        [0, 0, 0, 0, -30, 0, 0],   # J5 → -30°
        [0, 0, 0, 0,   0, 0, 0],   # J5 → 0°
        [0, 0, 0, 0, -30, 0, 0],   # J5 → -30°
        [0, 0, 0, 0,   0, 0, 0],   # J5 → 0°
    ]
    for i, target in enumerate(targets):
        cmd = make_movej_cmd(target, 30)
        r = queue_motion_push_back_moveJ(sock, cmd)
        print("  第" + str(i + 1) + "条: push_back_moveJ (J5→" + str(target[4]) + "°)  " + ("成功" if r == SUCCESS else "失败"))

    # 3. 发送队列到控制器
    print("  [信息] 发送队列到控制器...")
    r = queue_motion_send_to_controller(sock, 0, False)
    print_result("send_to_controller", r)
    if r != SUCCESS:
        print("[ERROR] 发送队列失败")
        queue_motion_set_status(sock, False)
        set_current_mode(sock, 0)
        disconnect_robot(sock)
        return -1

    # 4. 第 1 条执行中 → 暂停
    print("  [信息] 第 1 条执行中，等待 2 秒后暂停...")
    time.sleep(2)
    r = queue_motion_suspend(sock)
    print_result("暂停 (suspend)", r)
    time.sleep(2)

    # 5. 恢复
    r = queue_motion_restart(sock)
    print_result("恢复 (restart)", r)
    print("  [信息] 等待第 1 条完成...")
    wait_queue_len(sock, 3)
    print("  [信息] 第 1 条完成")

    # 6. 等待第 2 条完成
    print("  [信息] 等待第 2 条完成...")
    wait_queue_len(sock, 2)
    print("  [信息] 第 2 条完成")

    # 7. 第 3 条执行中 → 暂停
    time.sleep(1)
    r = queue_motion_suspend(sock)
    print_result("再次暂停 (suspend)", r)
    time.sleep(2)

    # 8. 恢复
    r = queue_motion_restart(sock)
    print_result("再次恢复 (restart)", r)
    print("  [信息] 等待第 3 条完成...")
    wait_queue_len(sock, 1)
    print("  [信息] 第 3 条完成")

    # 9. 第 4 条执行中 → 停止（保持上电）
    time.sleep(2)
    r = queue_motion_stop_not_power_off(sock)
    print_result("停止保持上电 (stop_not_power_off)", r)
    time.sleep(1)

    # 10. 关闭队列模式
    queue_motion_set_status(sock, False)
    # 等待队列模式完全关闭（与 run_queue_motion 的关闭延时一致）
    time.sleep(0.2)
    print("  [信息] 队列运动控制演示完毕")

    # ---- 切回示教模式并断开连接 ----
    print_separator("切回示教模式 + 断开连接")
    ret = set_current_mode(sock, 0)
    print_result("set_current_mode(0) 切回示教模式", ret)
    time.sleep(0.3)

    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] 队列运动示例程序运行完毕")
    return 0


if __name__ == "__main__":
    main()
