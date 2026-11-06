# /**
#  * ex_six_axis_force.py
#  * @brief 六维力传感器示例程序 —— 设置通讯参数、读取传感器数据
#  * @attention
#  *   - 确保控制器网络通信正常
#  *   - 传感器需正确连接并配置
#  *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#  * @note 运行步骤
#  *       运行: python3 ex_six_axis_force.py
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


def print_result(msg, ret):
    """打印操作结果"""
    if ret == SUCCESS:
        print("  [成功] " + msg)
    else:
        print("  [失败] " + msg + "，错误码: " + str(ret))


def print_separator(title):
    """打印分隔标题"""
    print("\n========== " + title + " ==========")


def main():
    # 解析命令行参数: python3 ex_six_axis_force.py [-ip 192.168.1.13]，默认 192.168.1.13
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
    # 1. 设置六维力传感器通讯参数
    # ============================================================
    print_separator("设置传感器通讯参数")

    params = SixDimensionalForceCommunicationParams()
    params.sensorCommunicationType = 0       # 0: EtherCAT
    params.startupAutoConnectSensor = True   # 启动自动连接传感器
    params.sensorDragEnable = True           # 传感器拖拽使能
    params.etherCat_mapNum = 1               # EtherCAT 映射数量
    params.YDirection = -1                   # Y 方向（-1 反向）
    params.ZDirection = -1                   # Z 方向（-1 反向）

    ret = set_six_dimensional_force_communication_params(sock, params)
    print_result("set_six_dimensional_force_communication_params", ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1

    time.sleep(0.5)

    # ============================================================
    # 2. 获取六维力传感器数据
    # ============================================================
    print_separator("获取传感器数据")

    data = Sensor6DData()
    ret = get_sensor_6d_data(sock, data)
    if ret == SUCCESS:
        print("[信息] 传感器连接状态: " + ("已连接" if data.sensorConnected else "未连接"))

        print("\n--- 传感器原始数据 ---")
        print("  Fx = {:.3f} N".format(data.fxData))
        print("  Fy = {:.3f} N".format(data.fyData))
        print("  Fz = {:.3f} N".format(data.fzData))
        print("  Mx = {:.3f} N·m".format(data.mxData))
        print("  My = {:.3f} N·m".format(data.myData))
        print("  Mz = {:.3f} N·m".format(data.mzData))

        print("\n--- 去皮后数据 ---")
        print("  Fx = {:.3f} N".format(data.fxDataSubBase))
        print("  Fy = {:.3f} N".format(data.fyDataSubBase))
        print("  Fz = {:.3f} N".format(data.fzDataSubBase))
        print("  Mx = {:.3f} N·m".format(data.mxDataSubBase))
        print("  My = {:.3f} N·m".format(data.myDataSubBase))
        print("  Mz = {:.3f} N·m".format(data.mzDataSubBase))

        if data.torqueConvertData:
            print("\n--- 扭矩转换数据 ---")
            for i in range(len(data.torqueConvertData)):
                print("  J{} = {:.3f}".format(i + 1, data.torqueConvertData[i]))
    else:
        print_result("get_sensor_6d_data", ret)

    # ============================================================
    # 3. 断开连接
    # ============================================================
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] 六维力传感器示例程序运行完毕")
    return 0


if __name__ == "__main__":
    exit(main())
