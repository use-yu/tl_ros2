# /**
#  * ex_six_axis_force_drag.py
#  * @brief 六维力传感器拖动示教示例程序（Python 版）
#  *
#  * 通过六维力传感器实现拖动示教：
#  *   1. 设置六维力传感器通讯参数（启用传感器拖拽 sensorDragEnable=true）
#  *   2. 读取传感器数据确认传感器连接正常
#  *   3. 设置拖拽模式（力矩模式），进入六维力拖动示教
#  *   4. 拖动过程中实时读取六维力数据并检测拖拽结束
#  *   5. 关闭拖拽、下电、断开连接
#  *
#  * @attention
#  *   - 运行前需保证六维力传感器已正确接线并完成通讯配置
#  *   - 拖动示教期间机械臂会随外力移动，请确保安全区域无人、无碰撞风险
#  *   - 本示例会直接使能伺服上电，请谨慎运行
#  * @note 运行步骤
#  *       运行: python3 ex_six_axis_force_drag.py
#  *       可调参数（控制器 IP、拖动时长、拖拽模式）见文件开头常量区，按需求修改即可
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
# ---- 可调参数（按需求修改即可） ----
# ============================================================
CONTROLLER_IP   = "192.168.1.13"  # 控制器 IP（可用命令行参数 -ip 覆盖，默认 192.168.1.13）
CONTROLLER_PORT = "6001"          # 控制/状态端口（六维力接口使用 6001）
DRAG_SECONDS    = 30              # 拖动示教时长（秒），到点自动结束
DRAG_MODE       = 2               # 拖拽模式：0=无 1=3D鼠标 2=力矩模式 3=位置
                                  # 六维力拖动示教使用力矩模式(2)


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


def print_six_dimensional_force_param(tag, p):
    """打印六维力传感器通讯参数"""
    print("========== [ " + tag + " ] SixDimensionalForceCommunicationParams ==========")
    print("  " + "sensorDragEnable".ljust(30) + " = " + str(p.sensorDragEnable))
    print("  " + "originDataInitialPara".ljust(30) + " = " + str(p.originDataInitialPara))
    print("  " + "sensorCommunicationType".ljust(30) + " = " + str(p.sensorCommunicationType)
          + "  (0=EtherCAT 1=ModbusRTU 2=ModbusTCP)")
    print("  " + "startupAutoConnectSensor".ljust(30) + " = " + str(p.startupAutoConnectSensor))
    print("  " + "YDirection".ljust(30) + " = " + str(p.YDirection))
    print("  " + "ZDirection".ljust(30) + " = " + str(p.ZDirection))
    print("  " + "etherCat_mapNum".ljust(30) + " = " + str(p.etherCat_mapNum))
    print("  -- Modbus RTU --")
    print("  " + "modbus_rtu_slaveID".ljust(30) + " = " + str(p.modbus_rtu_slaveID))
    print("  " + "modbus_rtu_port".ljust(30) + " = " + str(p.modbus_rtu_port))
    print("  " + "modbus_rtu_baudRate".ljust(30) + " = " + str(p.modbus_rtu_baudRate))
    print("  " + "modbus_rtu_addressType".ljust(30) + " = " + str(p.modbus_rtu_addressType))
    print("  " + "modbus_rtu_firstAddress".ljust(30) + " = " + str(p.modbus_rtu_firstAddress))
    print("  " + "modbus_rtu_addressNum".ljust(30) + " = " + str(p.modbus_rtu_addressNum))
    print("  " + "modbus_rtu_endian".ljust(30) + " = " + str(p.modbus_rtu_endian))
    print("  " + "modbus_rtu_checkBit".ljust(30) + " = " + str(p.modbus_rtu_checkBit))
    print("  " + "modbus_rtu_dataBit".ljust(30) + " = " + str(p.modbus_rtu_dataBit))
    print("  " + "modbus_rtu_stopBit".ljust(30) + " = " + str(p.modbus_rtu_stopBit))
    print("  -- Modbus TCP --")
    print("  " + "modbus_tcp_IP".ljust(30) + " = " + str(p.modbus_tcp_IP))
    print("  " + "modbus_tcp_port".ljust(30) + " = " + str(p.modbus_tcp_port))
    print("  " + "modbus_tcp_addressType".ljust(30) + " = " + str(p.modbus_tcp_addressType))
    print("  " + "modbus_tcp_firstAddress".ljust(30) + " = " + str(p.modbus_tcp_firstAddress))
    print("  " + "modbus_tcp_addressNum".ljust(30) + " = " + str(p.modbus_tcp_addressNum))
    print("  " + "modbus_tcp_endian".ljust(30) + " = " + str(p.modbus_tcp_endian))
    print("================================================================")


def print_sensor_6d_data(data):
    """打印六维力传感器数据（去皮后）"""
    print("  [传感器状态] " + ("已连接" if data.sensorConnected else "未连接"))
    print("  Fx={:8.2f}  Fy={:8.2f}  Fz={:8.2f} (N)   "
          "Mx={:8.2f}  My={:8.2f}  Mz={:8.2f} (N·m)".format(
              data.fxDataSubBase, data.fyDataSubBase, data.fzDataSubBase,
              data.mxDataSubBase, data.myDataSubBase, data.mzDataSubBase))


def setup_six_dimensional_force_comm(socket_fd):
    """设置六维力传感器通讯参数
    先 get 查询当前值，再基于原值修改关键字段，再 set 写回，避免覆盖无关配置
    @return True 设置成功
    """
    # ---------- Phase 1: 查询当前通讯参数 ----------
    orig_param = SixDimensionalForceCommunicationParams()
    ret = get_six_dimensional_force_communication_params(socket_fd, orig_param)
    if ret != SUCCESS:
        print("[ERROR] 查询六维力传感器通讯参数失败! ret = " + str(ret))
        return False
    print_six_dimensional_force_param("get(查询原值)", orig_param)

    # ---------- Phase 2: 基于原值修改拖动示教相关字段 ----------
    set_param = SixDimensionalForceCommunicationParams()
    set_param.sensorDragEnable = orig_param.sensorDragEnable
    set_param.originDataInitialPara = orig_param.originDataInitialPara
    set_param.sensorCommunicationType = orig_param.sensorCommunicationType
    set_param.startupAutoConnectSensor = orig_param.startupAutoConnectSensor
    set_param.YDirection = orig_param.YDirection
    set_param.ZDirection = orig_param.ZDirection
    set_param.etherCat_mapNum = orig_param.etherCat_mapNum
    set_param.modbus_rtu_slaveID = orig_param.modbus_rtu_slaveID
    set_param.modbus_rtu_port = orig_param.modbus_rtu_port
    set_param.modbus_rtu_baudRate = orig_param.modbus_rtu_baudRate
    set_param.modbus_rtu_addressType = orig_param.modbus_rtu_addressType
    set_param.modbus_rtu_firstAddress = orig_param.modbus_rtu_firstAddress
    set_param.modbus_rtu_addressNum = orig_param.modbus_rtu_addressNum
    set_param.modbus_rtu_endian = orig_param.modbus_rtu_endian
    set_param.modbus_rtu_checkBit = orig_param.modbus_rtu_checkBit
    set_param.modbus_rtu_dataBit = orig_param.modbus_rtu_dataBit
    set_param.modbus_rtu_stopBit = orig_param.modbus_rtu_stopBit
    set_param.modbus_tcp_IP = orig_param.modbus_tcp_IP
    set_param.modbus_tcp_port = orig_param.modbus_tcp_port
    set_param.modbus_tcp_addressType = orig_param.modbus_tcp_addressType
    set_param.modbus_tcp_firstAddress = orig_param.modbus_tcp_firstAddress
    set_param.modbus_tcp_addressNum = orig_param.modbus_tcp_addressNum
    set_param.modbus_tcp_endian = orig_param.modbus_tcp_endian

    # 修改拖动示教相关字段
    set_param.sensorDragEnable = True          # 启用传感器拖拽（六维力拖动示教关键开关）
    set_param.sensorCommunicationType = 0      # 通讯类型 0=EtherCAT（按实际传感器类型修改）
    set_param.startupAutoConnectSensor = True  # 启动自动连接传感器
    set_param.YDirection = -1                  # Y 轴方向（1=正，-1=反）
    set_param.ZDirection = -1                  # Z 轴方向（1=正，-1=反）
    print_six_dimensional_force_param("set(下发值)", set_param)

    # ---------- Phase 3: 写回 ----------
    ret = set_six_dimensional_force_communication_params(socket_fd, set_param)
    if ret != SUCCESS:
        print("[ERROR] 设置六维力传感器通讯参数失败! ret = " + str(ret))
        return False
    print("[INFO] 设置六维力传感器通讯参数成功（fire-and-forget，成败靠 get 读回验证）")

    # ---------- Phase 4: 读回校验 ----------
    time.sleep(0.3)
    get_param = SixDimensionalForceCommunicationParams()
    ret = get_six_dimensional_force_communication_params(socket_fd, get_param)
    if ret == SUCCESS:
        print_six_dimensional_force_param("get(读回值)", get_param)
    return True


def main():
    # 解析命令行参数: python3 ex_six_axis_force_drag.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = CONTROLLER_IP
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: " + version)

    # ---- 连接控制器 ----
    print_separator("连接控制器")
    sock = connect_robot(ip, CONTROLLER_PORT)
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        return -1
    print("[信息] 连接成功 (socket = " + str(sock) + ")")

    try:
        # ---- 使能伺服上电 ----
        enable_servo(sock)
        time.sleep(2)

        # ============================================================
        # 1. 设置六维力传感器通讯参数（启用传感器拖拽）
        # ============================================================
        print_separator("设置六维力传感器通讯参数")
        if not setup_six_dimensional_force_comm(sock):
            print("[ERROR] 通讯参数设置失败，程序退出!")
            return -1

        # ============================================================
        # 2. 读取传感器数据，确认传感器连接正常
        # ============================================================
        print_separator("读取传感器数据")
        sensor_data = Sensor6DData()
        ret = get_sensor_6d_data(sock, sensor_data)
        if ret != SUCCESS:
            print("[ERROR] 获取六维力传感器数据失败! ret = " + str(ret))
            return -1
        print_sensor_6d_data(sensor_data)
        if not sensor_data.sensorConnected:
            print("[WARN] 传感器未连接，拖动示教可能无法正常生效，请检查接线与配置")

        # ============================================================
        # 3. 设置拖拽模式（力矩模式 = 六维力拖动示教）
        # ============================================================
        print_separator("设置拖拽模式")
        print("[INFO] 拖拽模式 " + str(DRAG_MODE)
              + " (0=无 1=3D鼠标 2=力矩模式 3=位置)，六维力拖动示教使用力矩模式(2)")
        ret = set_darg_mode(sock, DRAG_MODE)
        if ret != SUCCESS:
            print("[ERROR] set_darg_mode(mode=" + str(DRAG_MODE) + ") 返回: " + str(ret))
            return -1
        print("[INFO] set_darg_mode(mode=" + str(DRAG_MODE) + ") 返回: " + str(ret)
              + "（fire-and-forget，成败靠实际拖动验证）")

        # ============================================================
        # 4. 六维力拖动示教：实时读取六维力数据，检测拖拽结束
        # ============================================================
        print_separator("开始六维力拖动示教（" + str(DRAG_SECONDS) + " 秒）")
        print("[INFO] 请握住末端工具，施加外力拖动机械臂到目标位置（安全第一）...")

        start_time = time.time()
        while True:
            elapsed = int(time.time() - start_time)

            # 达到设定时长，结束拖动
            if elapsed >= DRAG_SECONDS:
                print("\n[INFO] 拖动时长已到（" + str(DRAG_SECONDS) + " 秒），结束拖动")
                break

            # 查询拖拽线程是否结束（绑定返回 [ret, endFlag] 列表）
            ret, drag_finished = get_drag_thread_is_end(sock, False)
            if ret != SUCCESS:
                print("[ERROR] 查询拖拽结束状态失败! ret = " + str(ret))
                break
            if drag_finished:
                print("\n[INFO] 检测到拖拽已结束")
                break

            # 每 500ms 读取一次六维力数据
            sensor_data = Sensor6DData()
            ret = get_sensor_6d_data(sock, sensor_data)
            if ret == SUCCESS:
                print("  [剩余 " + str(DRAG_SECONDS - elapsed) + "s] ", end="")
                print_sensor_6d_data(sensor_data)

            time.sleep(0.5)

        # ============================================================
        # 5. 关闭拖拽模式
        # ============================================================
        print_separator("关闭拖拽模式")
        ret = set_darg_mode(sock, 0)   # 0 = 关闭拖拽
        print("[INFO] set_darg_mode(0) 返回: " + str(ret))

        return 0
    finally:
        # ---- 使能下电 ----
        disable_servo(sock)

        # ---- 断开控制器连接 ----
        print_separator("断开连接")
        ret = disconnect_robot(sock)
        if ret != SUCCESS:
            print("[ERROR] 断开连接失败（下电可能已自动断连，返回 -1 视为成功）")
        else:
            print("[INFO] 断开连接成功")

        print("\n[信息] 六维力拖动示教示例程序运行完毕")


if __name__ == "__main__":
    exit(main())
