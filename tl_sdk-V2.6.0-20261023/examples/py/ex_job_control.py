# /**
#  * ex_job_control.py
#  * @brief 作业控制相关接口（包括作业文件插入MoveJ、MoveL、MoveC，打开作业文件，创建作业文件，获取全部作业文件名，运行作业文件）
#  * @attention
#  *   - 插入作业之前需要先打开一个作业文件
#  *   - 运行一个作业文件需要切换到运行模式
#  *   - 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 ex_job_control.py
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


def main():
    # 定义IP地址和端口
    # 解析命令行参数: python3 ex_job_control.py [-ip 192.168.1.13]，默认 192.168.1.13
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

    # ----创建作业文件----
    jobName = "TEST_PY"
    print("[INFO] **********创建作业文件**********")
    ret = job_create(socket_fd, jobName)
    if ret != SUCCESS:
        print("[ERROR] 创建作业文件失败，程序退出!")
        return -1
    print("[INFO] 创建作业文件成功")
    print()

    # ----打开作业文件----
    print("[INFO] **********打开作业文件**********")
    ret = job_open(socket_fd, jobName)
    if ret != SUCCESS:
        print("[ERROR] 打开作业文件失败，程序退出!")
        return -1
    print("[INFO] 打开作业文件成功")
    print()

    # ----作业文件插入MoveJ----
    print("[INFO] **********作业文件插入MoveJ**********")
    line = 1
    cmd = MoveCmd()
    cmd.targetPosType  = PosType_PType
    cmd.targetPosName  = "P0001"
    cmd.coord          = Coord.JOINT
    cmd.velocity       = 40
    cmd.acc            = 10
    cmd.dec            = 10
    cmd.pl             = 0
    cmd.targetPosValue = VectorDouble([0.0] * 14)
    ret = job_insert_moveJ(socket_fd, line, cmd)
    if ret != SUCCESS:
        print("[ERROR] 作业文件插入MoveJ失败，程序退出!")
        return -1
    print("[INFO] 作业文件插入MoveJ成功")
    print()

    # ----作业文件插入MoveL----
    print("[INFO] **********作业文件插入MoveL**********")
    line = 2
    cmd.targetPosName  = "P0002"
    cmd.coord          = Coord.BASE
    cmd.targetPosValue = VectorDouble([280, 0.0, 380, 3.14, 0.0, 0.0])
    ret = job_insert_moveL(socket_fd, line, cmd)
    if ret != SUCCESS:
        print("[ERROR] 作业文件插入MoveL失败，程序退出!")
        return -1
    print("[INFO] 作业文件插入MoveL成功")
    print()

    # ----作业文件插入MoveC（姿态1）----
    print("[INFO] **********作业文件插入MoveC**********")
    line = 3
    cmd.targetPosName  = "P0003"
    cmd.coord          = Coord.TOOL
    cmd.targetPosValue = VectorDouble([270, 0.0, 380, 3.14, 0.0, 0.0])
    ret = job_insert_moveC(socket_fd, line, cmd)
    if ret != SUCCESS:
        print("[ERROR] 作业文件插入MoveC失败，程序退出!")
        return -1
    print("[INFO] 作业文件插入MoveC成功")
    print()

    # ----作业文件插入MoveC（姿态2）----
    print("[INFO] **********作业文件插入MoveC**********")
    line = 3
    cmd.targetPosName  = "P0003"
    cmd.coord          = Coord.TOOL
    cmd.targetPosValue = VectorDouble([240, 0.0, 380, 3.14, 0.0, 0.0])
    ret = job_insert_moveC(socket_fd, line, cmd)
    if ret != SUCCESS:
        print("[ERROR] 作业文件插入MoveC失败，程序退出!")
        return -1
    print("[INFO] 作业文件插入MoveC成功")
    print()

    # ----获取所有作业文件名----
    print("[INFO] **********获取所有作业文件名**********")
    AllJobNames = VectorVectorString()
    ret = job_get_all_jobfile_name(socket_fd, AllJobNames)
    if ret != SUCCESS:
        print("[ERROR] 获取作业文件名失败，程序退出!")
        return -1
    i = 1
    for JobNames in AllJobNames:
        for name in JobNames:
            print("[INFO] 作业文件 %d 名称: %s" % (i, name))
            i += 1

    # ----切换为运行模式----
    # 运行作业文件需要在运行模式下执行（2=运行）
    print("--- 切换运行模式 ---")
    ret = set_current_mode(socket_fd, 2)
    if ret != SUCCESS:
        print("[ERROR] 切换运行模式失败，程序退出!")
        return -1
    print("[INFO] 已切换为运行模式")

    # ----运行作业文件----
    print("[INFO] **********运行作业文件**********")
    ret = job_run(socket_fd, jobName)
    if ret != SUCCESS:
        print("[ERROR] 运行作业文件失败，程序退出!")
        return -1
    print("[INFO] 运行作业文件成功")
    print()

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
