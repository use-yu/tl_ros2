# /**
#  * ex_coord_transform.py
#  * @brief 坐标变换算法示例程序 —— 展示常用的坐标系转换接口
#  * @attention
#  *   - 需要控制器连接正常、已上电
#  *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#  * @note 运行步骤
#  *       运行: python3 ex_coord_transform.py
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
import math


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


def print_vector(name, v):
    """打印向量 —— 格式: name [size] = {v1, v2, ...}，保留4位小数"""
    formatted = ["{:.4f}".format(x) for x in v]
    print("  " + name + " [" + str(len(v)) + "] = {" + ", ".join(formatted) + "}")


def print_matrix(name, m, rows, cols):
    """打印矩阵（行主序），保留4位小数"""
    print("  " + name + " [" + str(len(m)) + "]:")
    for r in range(rows):
        line = "    "
        for c in range(cols):
            idx = r * cols + c
            if idx < len(m):
                line += "{:10.4f} ".format(m[idx])
        print(line)


# ============================================================
# 主程序
# ============================================================
if __name__ == "__main__":
    # 解析命令行参数: python3 ex_coord_transform.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: " + version)

    # ---- 双端口连接 ----
    print_separator("连接控制器")
    sock = connect_robot(ip, "6001")
    if sock < 0:
        print("[ERROR] 连接 6001 端口失败")
        sys.exit(-1)
    sock_servo = connect_robot(ip, "7000")
    if sock_servo < 0:
        print("[ERROR] 连接 7000 端口失败")
        disconnect_robot(sock)
        sys.exit(-1)
    print("[信息] 双端口连接成功 (6001=" + str(sock) + ", 7000=" + str(sock_servo) + ")")

    # ---- 上电（servoJ 需要上电状态）----
    print_separator("清错与上电")
    clear_error(sock)
    time.sleep(0.3)
    set_servo_poweroff(sock)
    time.sleep(0.3)

    servo_state = -1
    ret, servo_state = get_servo_state(sock, servo_state)
    if ret != SUCCESS:
        print(f"[ERROR] 获取伺服状态失败: {ret}")
        disconnect_robot(sock)
        disconnect_robot(sock_servo)
        sys.exit(-1)
    print(f"  [信息] 当前伺服状态: {servo_state} (0=停止 1=就绪 2=报警 3=运行)")

    if servo_state == 2:
        print("[ERROR] 伺服处于报警状态，无法上电")
        disconnect_robot(sock)
        disconnect_robot(sock_servo)
        sys.exit(-1)

    if servo_state != 3:
        print("  [信息] 伺服未运行，执行上电流程...")
        ret = set_servo_state(sock, 1)
        if ret != SUCCESS and ret != OPERATION_NOT_ALLOWED:
            print(f"[ERROR] 设置伺服就绪失败: {ret}")
            disconnect_robot(sock)
            disconnect_robot(sock_servo)
            sys.exit(-1)
        time.sleep(0.5)
        ret = set_servo_poweron(sock)
        if ret != SUCCESS:
            print(f"[ERROR] 上电失败: {ret}")
            disconnect_robot(sock)
            disconnect_robot(sock_servo)
            sys.exit(-1)
        time.sleep(2)
        print("  [信息] 上电成功")
    else:
        print("  [信息] 伺服已运行 (servo_state=3)")

    # ============================================================
    # 1. 四元数 ↔ 欧拉角 (RPY) — 正反验证
    # ============================================================
    # 单位说明:
    #   四元数 [x, y, z, w]      — 无量纲（归一化）
    #   欧拉角 [roll, pitch, yaw] — 弧度 rad
    #   旋转矩阵 (3×3 行主序)     — 无量纲
    print_separator("四元数 ↔ 欧拉角")

    # 1a. 欧拉角 → 四元数 → 欧拉角（正反验证）
    print("--- 1a. RPY ↔ 四元数 正反验证 ---")
    rpy_in = [0.2, 0.3, 0.5]  # rad
    quat = VectorDouble()
    rpy_out = VectorDouble()

    ret = get_rpy2quat(sock, rpy_in, quat)
    print_result("get_rpy2quat (RPY→四元数)", ret)
    if ret != SUCCESS:
        sys.exit(-1)
    print_vector("  输入欧拉角 [rad]", rpy_in)
    print_vector("  中间四元数 [x, y, z, w]", quat)

    ret = get_quat2rpy(sock, quat, rpy_out)
    print_result("get_quat2rpy (四元数→RPY)", ret)
    if ret != SUCCESS:
        sys.exit(-1)
    print_vector("  还原欧拉角 [rad]", rpy_out)

    if len(rpy_out) >= 3 and len(rpy_in) >= 3:
        err = abs(rpy_out[0] - rpy_in[0]) \
            + abs(rpy_out[1] - rpy_in[1]) \
            + abs(rpy_out[2] - rpy_in[2])
        print("  正反变换误差 (应接近0): " + str(err))

    # 1b. 欧拉角 → 旋转矩阵
    print("\n--- 1b. get_rpy2r: 欧拉角转旋转矩阵 ---")
    rpy = [0.0, 0.0, math.pi / 2]  # rad
    rot_mat = VectorDouble()

    ret = get_rpy2r(sock, rpy, rot_mat)
    print_result("get_rpy2r", ret)
    print_vector("输入欧拉角 [rad]", rpy)
    if ret == SUCCESS:
        print_matrix("输出旋转矩阵 (3x3, 行主序, 无量纲)", rot_mat, 3, 3)

    # ============================================================
    # 2. 位姿矩阵 ↔ 旋转矩阵 — 正反验证
    # ============================================================
    # 单位说明:
    #   旋转矩阵 R (3×3)     — 无量纲
    #   平移向量 T [Tx,Ty,Tz] — 毫米 mm
    #   位姿矩阵 TR (4×4)  — R 部分无量纲，T 部分单位 mm
    # 位姿矩阵 (行主序 16 元素):
    #   [Rxx Rxy Rxz Tx]  第 0 行
    #   [Ryx Ryy Ryz Ty]  第 1 行
    #   [Rzx Rzy Rzz Tz]  第 2 行
    #   [  0   0   0  1]  第 3 行
    print_separator("位姿矩阵 ↔ 旋转矩阵")

    # 2a. TR → R → TR（正反验证）
    print("--- 2a. TR ↔ R 正反验证 ---")
    # 构造一个绕 Z 转 90°、平移 (100, 200, 300) mm 的位姿矩阵
    cos90 = math.cos(math.pi / 2)
    sin90 = math.sin(math.pi / 2)
    tr_in = [
        cos90, -sin90, 0.0, 100.0,
        sin90,  cos90, 0.0, 200.0,
        0.0,    0.0,   1.0, 300.0,
        0.0,    0.0,   0.0, 1.0
    ]
    R = VectorDouble()
    tr_out = VectorDouble()

    ret = get_tr2r(sock, tr_in, R)
    print_result("get_tr2r (TR→R)", ret)
    if ret != SUCCESS:
        sys.exit(-1)
    print_matrix("输入位姿矩阵 (4x4, R无量纲, T:mm)", tr_in, 4, 4)
    print_matrix("中间旋转矩阵 (3x3, 无量纲)", R, 3, 3)

    ret = get_r2tr(sock, R, tr_out)
    print_result("get_r2tr (R→TR)", ret)
    if ret != SUCCESS:
        sys.exit(-1)
    print_matrix("还原位姿矩阵 (4x4, R无量纲, T:mm)", tr_out, 4, 4)

    # 说明: get_tr2r 只提取旋转矩阵，平移 T 在此丢弃；
    # get_r2tr 的输入不含平移，还原出的 TR 平移部分恒为 0。
    # SDK 无 "R+T 构造 TR" 接口，完整位姿矩阵需由调用方自行拼装 T。
    print("  注: get_tr2r 仅输出旋转矩阵，平移 T 不参与往返；")
    print("      还原位姿矩阵的平移部分为 0，需调用方另行拼装。")

    # 对比旋转部分（TR→R→TR 链路仅保证旋转信息无损）
    # 注意: 行主序 4x4 中旋转块位于 0,1,2, 4,5,6, 8,9,10，
    #       位置 3/7/11 是平移列，不能按连续 0..8 遍历。
    if len(tr_out) >= 16 and len(tr_in) >= 16:
        rot_idx = [0, 1, 2, 4, 5, 6, 8, 9, 10]
        r_err = 0.0
        for k in rot_idx:
            r_err += abs(tr_out[k] - tr_in[k])
        print("  旋转矩阵误差 (应接近0): " + str(r_err))

    # ============================================================
    # 3. 坐标系转换（获取机器人当前位姿在不同坐标系下的表示）
    # ============================================================
    # 单位说明:
    #   关节坐标 [J1..J6] — 度 °
    #   基坐标 [X,Y,Z,Rx,Ry,Rz] — 位移 mm，姿态 rad
    #   工具/用户坐标 — 同基坐标
    print_separator("坐标系转换")

    print("--- 3. get_origin_coord_to_target_coord ---")
    print("  坐标系: 0=关节(°)  1=基坐标(mm+rad)  2=工具(mm+rad)  3=用户(mm+rad)")

    # 从关节坐标系(0) 转换到基坐标系(1)
    # 关节角度: J1=10°, J2=20°, J3=30°, J4=0°, J5=0°, J6=0°
    joint_pos = VectorDouble()
    joint_pos.push_back(10.0)
    joint_pos.push_back(20.0)
    joint_pos.push_back(30.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    cartesian_pos = VectorDouble()
    origin_reference_pos = VectorDouble([0]*7)

    ret = get_origin_coord_to_target_coord(sock, Coord.JOINT, joint_pos, Coord.BASE, cartesian_pos, 0, origin_reference_pos)
    print_result("get_origin_coord_to_target_coord (关节→基坐标) 正解", ret)
    print_vector("输入关节角度 [°]", joint_pos)
    if ret == SUCCESS:
        print("  输出基坐标 — 前3位为位移[mm]，后3位为姿态[rad]:")
        print_vector("  基坐标", cartesian_pos)
        if len(cartesian_pos) >= 6:
            print("    位移 X=" + str(cartesian_pos[0]) + "mm, Y=" + str(cartesian_pos[1])
                  + "mm, Z=" + str(cartesian_pos[2]) + "mm")
            print("    姿态 Rx=" + str(cartesian_pos[3]) + "rad, Ry=" + str(cartesian_pos[4])
                  + "rad, Rz=" + str(cartesian_pos[5]) + "rad")
            print("    姿态换算(°): Rx=" + str(cartesian_pos[3] * 180.0 / math.pi)
                  + ", Ry=" + str(cartesian_pos[4] * 180.0 / math.pi)
                  + ", Rz=" + str(cartesian_pos[5] * 180.0 / math.pi))

    # 逆解: 基坐标系(1) → 关节坐标系(0)
    # 使用正解输出的基坐标作为输入，验证正反解一致性
    if ret == SUCCESS and len(cartesian_pos) >= 6:
        joint_back = VectorDouble()
        ret = get_origin_coord_to_target_coord(sock, Coord.BASE, cartesian_pos, Coord.JOINT, joint_back, 0, origin_reference_pos)
        print_result("get_origin_coord_to_target_coord (基坐标→关节) 逆解", ret)
        print("  输入基坐标:")
        print("    位移 [mm]: X=" + str(cartesian_pos[0]) + ", Y=" + str(cartesian_pos[1])
              + ", Z=" + str(cartesian_pos[2]))
        print("    姿态 [rad]: Rx=" + str(cartesian_pos[3]) + ", Ry=" + str(cartesian_pos[4])
              + ", Rz=" + str(cartesian_pos[5]))
        if ret == SUCCESS:
            print_vector("  逆解关节角度 [°]", joint_back)
            # 对比原始关节角度，计算误差
            err = 0.0
            for i in range(min(len(joint_pos), len(joint_back))):
                err += abs(joint_pos[i] - joint_back[i])
            print("  正反解误差 (应接近0): " + str(err))

    # ============================================================
    # 4. 逆解关节坐标 → servoJ 下发运动
    # ============================================================
    # 演示流程: 给定基坐标 → 逆解到关节角 → 通过 servoJ 发送到控制器
    print_separator("逆解 + servoJ 运动")

    # 取一个基坐标点，逆解为关节角
    target_pose = [300, 0, 100, 3.141, 0, 0]  # X=300mm, Y=0, Z=100mm
    target_joints = VectorDouble()
    origin_reference_pos = VectorDouble([0]*7)

    ret = get_origin_coord_to_target_coord(sock, Coord.BASE, target_pose, Coord.JOINT, target_joints, 0, origin_reference_pos)
    print_result("逆解: 基坐标→关节", ret)
    if ret == SUCCESS:
        print_vector("目标基坐标 [mm, rad]", target_pose)
        print_vector("逆解关节角度 [°]", target_joints)

        # 通过 servoJ 发送关节角度到控制器
        # servoJ 需要连接 7000 端口，已在开头连接
        print("\n  --- 通过 servoJ 下发逆解结果 ---")

        # servoJ 约束参数（6轴/7轴均传七个元素）
        vmax = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
        amax = [60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
        jmax = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

        ret = open_servoJ(sock_servo, vmax, amax, jmax)
        print_result("open_servoJ", ret)
        if ret == SUCCESS:
            # 逐步发送逆解出的关节角度，确保 servoJ 跟踪到位
            # 分 50 步从当前角度平滑过渡到目标角度
            # q 固定 7 元素，长度不够补 0（已在初始化时置零）
            axis_count = len(target_joints)
            current = [0.0] * axis_count
            steps = 50
            for i in range(steps):
                q = [0.0] * 7
                n = min(axis_count, len(q))
                for j in range(n):
                    q[j] = current[j] + (target_joints[j] - current[j]) * (i + 1) / steps
                set_servoJ_pos(sock_servo, q)
                time.sleep(0.01)
            # servoJ 是实时跟踪模式，等待固定时间确保运动到位
            print("  [信息] 等待 servoJ 运动到位...")
            time.sleep(5)

            close_servoJ(sock_servo)
            print_result("close_servoJ", ret)

    # ============================================================
    # 5. 不同坐标系转换（正解/逆解）— 多机器人版本
    # ============================================================
    print_separator("坐标系转换（多机器人版本）")

    print("--- 5. get_origin_coord_to_target_coord_robot ---")
    print("  坐标系: 0=关节(°)  1=基坐标(mm+rad)  2=工具(mm+rad)  3=用户(mm+rad)")

    # 正解: 关节坐标系(0) → 基坐标系(1)
    print("\n  --- 正解: 关节→基坐标 ---")
    joint_pos = VectorDouble()
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    joint_pos.push_back(0.0)
    cartesian_pos = VectorDouble()
    origin_reference_pos = VectorDouble([0]*7)

    robot_num = 1
    ret = get_origin_coord_to_target_coord(sock, Coord.JOINT, joint_pos, Coord.BASE, cartesian_pos, 0, origin_reference_pos)
    print_result("关节→基坐标 (robot_num=1) 正解", ret)
    print_vector("输入关节角度 [°]", joint_pos)
    if ret == SUCCESS:
        print_vector("输出基坐标", cartesian_pos)
        if len(cartesian_pos) >= 6:
            print("    位移 [mm]: X=" + str(cartesian_pos[0]) + ", Y=" + str(cartesian_pos[1])
                  + ", Z=" + str(cartesian_pos[2]))
            print("    姿态 [rad]: Rx=" + str(cartesian_pos[3]) + ", Ry=" + str(cartesian_pos[4])
                  + ", Rz=" + str(cartesian_pos[5]))

    # 逆解: 基坐标系(1) → 关节坐标系(0)
    if ret == SUCCESS and len(cartesian_pos) >= 6:
        print("\n  --- 逆解: 基坐标→关节 ---")
        joint_back = VectorDouble()
        ret = get_origin_coord_to_target_coord(sock, Coord.BASE, cartesian_pos, Coord.JOINT, joint_back, 0, origin_reference_pos)
        print_result("基坐标→关节 (robot_num=1) 逆解", ret)
        if ret == SUCCESS:
            print_vector("逆解关节角度 [°]", joint_back)
            err = 0.0
            for i in range(min(len(joint_pos), len(joint_back))):
                err += abs(joint_pos[i] - joint_back[i])
            print("  正反解误差 (应接近0): " + str(err))

    # ============================================================
    # 4. 逆解关节坐标 → servoJ 下发运动
    # ============================================================
    # 演示流程: 给定基坐标 → 逆解到关节角 → 通过 servoJ 发送到控制器
    print_separator("逆解 + servoJ 运动")

    # 取一个基坐标点，逆解为关节角
    target_pose = [300, 0, 100, 3.141, 0, 0]  # X=300mm, Y=0, Z=100mm
    target_joints = VectorDouble()
    origin_reference_pos = VectorDouble([0]*7)

    ret = get_origin_coord_to_target_coord(sock, Coord.BASE, target_pose, Coord.JOINT, target_joints, 0, origin_reference_pos)
    print_result("逆解: 基坐标→关节", ret)
    if ret == SUCCESS:
        print_vector("目标基坐标 [mm, rad]", target_pose)
        print_vector("逆解关节角度 [°]", target_joints)

        # 通过 servoJ 发送关节角度到控制器
        # servoJ 需要连接 7000 端口，已在开头连接
        print("\n  --- 通过 servoJ 下发逆解结果 ---")

        # servoJ 约束参数（7 元素: 6 本体轴 + 1 外部轴）
        # 注意: SDK 底层协议要求 servoJ 相关向量固定为 7 元素
        vmax = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
        amax = [60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
        jmax = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

        ret = open_servoJ(sock_servo, vmax, amax, jmax)
        print_result("open_servoJ", ret)
        if ret == SUCCESS:
            # 逐步发送逆解出的关节角度，确保 servoJ 跟踪到位
            # 分 50 步从当前角度平滑过渡到目标角度
            # q 固定 7 元素，长度不够补 0（已在初始化时置零）
            axis_count = len(target_joints)
            current = [0.0] * axis_count
            steps = 50
            for i in range(steps):
                q = [0.0] * 7
                n = min(axis_count, len(q))
                for j in range(n):
                    q[j] = current[j] + (target_joints[j] - current[j]) * (i + 1) / steps
                set_servoJ_pos(sock_servo, q)
                time.sleep(0.01)
            # servoJ 是实时跟踪模式，等待固定时间确保运动到位
            print("  [信息] 等待 servoJ 运动到位...")
            time.sleep(5)

            close_servoJ(sock_servo)
            print_result("close_servoJ", ret)

    # ---- 断开连接 ----
    print_separator("断开连接")
    disconnect_robot(sock)
    disconnect_robot(sock_servo)
    print("[信息] 已断开连接，机械臂保持运行模式")

    print("\n[信息] 坐标变换示例程序运行完毕")
