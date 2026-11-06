# /**
#  * ex_inspire_claw.py
#  * @brief 因时电爪（Inspire EG2-4XX，RS485 / Modbus RTU）控制测试示例
#  * @attention
#  *   - 确保控制器网络通信正常，夹爪通过 RS485 串口连接控制器
#  *   - 主站参数（从站 ID、串口号、波特率等）请在下方"RTU 主站配置区"自行修改
#  *   - 开度范围 0~1000：0=闭合完，1000=张开完；写目标开度后夹爪立即动作，
#  *     需轮询实际开度寄存器等待到位
#  *   - 夹爪会真实动作：默认测试序列首位置 0 为全闭合，运行前请确保夹爪空闲、
#  *     指间无物体（有夹持需求时按需修改 CLAW_TEST_POS）
#  *   - startAddress=True 表示 0 起始地址、寄存器地址按手册编号直传；
#  *     若实机读写无响应，可将该字段改为 False 再试
#  * @note 运行步骤
#  *       运行: python3 ex_inspire_claw.py
#  */

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
# 控制器连接配置（按实际修改）
# ============================================================
ROBOT_IP        = "192.168.1.13"  # 机械臂 IP（可用命令行参数 -ip 覆盖，默认 192.168.1.13）
ROBOT_PORT      = "6001"            # 控制器端口

# ============================================================
# RTU 主站配置区（请根据实际接线自行修改）
# ============================================================
RTU_MASTER_ID   = 1               # 控制器侧主站编号
RTU_SLAVE_ID    = 1               # 夹爪从站 ID
RTU_PORT        = 2               # 控制器串口号（按接线修改）
RTU_BAUDRATE    = 115200          # 波特率（EG2 默认 115200）
RTU_CHECK_BIT   = "None"          # 校验位: None/Even/Odd
RTU_DATA_BIT    = 8               # 数据位
RTU_STOP_BIT    = 1               # 停止位

# ============================================================
# EG2-4C2 夹爪保持寄存器（控制器侧地址，EG2-4C2 固件实测）
# 注意: 通用手册（EG2-4XX V1.0.6）编号为 1010/1011/1012/1061/1064/1065，
#       本机 EG2-4C2 实测目标开度寄存器为地址 10（写 500 即张开到半开），
#       其余寄存器按同段顺延，若读数异常请按夹爪实际固件调整
# 功能码: 03H 读 / 06H 写单个 / 10H 写多个，CRC16-REV=0xA001
# ============================================================
REG_OPENLEN_SET = 10              # 目标开度 0~1000（0=闭合完 1000=张开完），写后立即动作
REG_SPEED_SET   = 11              # 速度 10~1000，写后不动作
REG_FORCE_SET   = 12              # 抓力 100~1000，写后不动作
REG_OPENLEN_ACT = 61              # 实际开度 0~1000（只读）
REG_ERRORCODE   = 64              # 错误码（只读，位0堵转 位1过温 位2过流 位3运行故障 位4内部通信故障）
REG_STATUS      = 65              # 状态码（只读，1张开到位 2闭合到位 3停止 4夹取中 5张开中 6夹取到位）
# 其它只读寄存器按需扩展: 60 实际抓力 / 62 电流 / 63 温度

CLAW_SPEED      = 300             # 夹爪动作速度
CLAW_FORCE      = 300             # 抓力阈值
# 测试开度序列: 0=闭合完, 1000=张开完；首位置 0 为全闭合，运行前确保指间无物体
CLAW_TEST_POS   = [0, 1000]


# ============================================================
# 工具函数
# ============================================================

def print_result(msg, ret):
    """@brief 打印操作结果"""
    if ret == SUCCESS:
        print("  [成功] %s" % msg)
    else:
        print("  [失败] %s，错误码: %d" % (msg, ret))


def print_separator(title):
    """@brief 打印分隔标题"""
    print("\n========== %s ==========" % title)


def read_with_retry(sock, master_id, addr, qty, desc):
    """@brief 带重试的读取：最多尝试 3 次，每次间隔 500ms"""
    for attempt in range(1, 4):
        data = VectorInt()
        ret = modbus_read_holding_registers(sock, master_id, addr, qty, data)
        if ret == SUCCESS:
            print("  [成功] %s（第%d次）" % (desc, attempt))
            return ret, data
        print("  [重试] %s 第%d次失败，500ms后重试..." % (desc, attempt))
        time.sleep(0.5)
    print("  [失败] %s，错误码: %d（重试3次均失败）" % (desc, ret))
    return ret, VectorInt()


def wait_openlen_reach(sock, master_id, target, timeout=10.0):
    """@brief 轮询实际开度寄存器（REG_OPENLEN_ACT）直到接近目标或超时"""
    deadline = time.time() + timeout
    last = -1
    read_ok = False
    while time.time() < deadline:
        data = VectorInt()
        ret = modbus_read_holding_registers(sock, master_id,
                                            REG_OPENLEN_ACT, 1, data)
        if ret == SUCCESS and len(data) > 0:
            read_ok = True
            last = data[0]
            print("  当前开度: %-4d（目标 %d）" % (last, target),
                  end="\r")
            if abs(last - target) <= 5:
                print()
                return True, last
        time.sleep(0.2)
    print()
    return False, last if read_ok else -1


# ============================================================
# 主函数
# ============================================================

def main():
    # 解析命令行参数: python3 ex_inspire_claw.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = ROBOT_IP
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: %s" % version)

    # ---- 连接控制器 ----
    print_separator("连接控制器")
    sock = connect_robot(ip, ROBOT_PORT)
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        return -1
    print("[信息] 连接成功 (socket = %d)" % sock)

    # ---- 配置 RTU 主站 ----
    print_separator("配置 RTU 主站")
    param = ModbusMasterParameter()
    param.type = "RTU"
    param.startAddress = True               # 起始地址 0
    param.RTU.slaveId = RTU_SLAVE_ID
    param.RTU.port = RTU_PORT
    param.RTU.baudrate = RTU_BAUDRATE
    param.RTU.checkBit = RTU_CHECK_BIT
    param.RTU.dataBit = RTU_DATA_BIT
    param.RTU.stopBit = RTU_STOP_BIT

    ret = modbus_set_master_parameter(sock, RTU_MASTER_ID, param)
    msg = "modbus_set_master_parameter (id=%d, RTU)" % RTU_MASTER_ID
    print_result(msg, ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1
    print("  从站ID: %d, 串口: %d, 波特率: %d"
          % (param.RTU.slaveId, param.RTU.port, param.RTU.baudrate))
    print("  校验: %s, 数据位: %d, 停止位: %d"
          % (param.RTU.checkBit, param.RTU.dataBit, param.RTU.stopBit))

    # ---- 打开 Modbus 主站 ----
    ret = modbus_open_master(sock, RTU_MASTER_ID)
    print_result("modbus_open_master (id=%d)" % RTU_MASTER_ID, ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1
    time.sleep(0.5)

    # ---- 03H 读当前开度 ----
    print_separator("读取夹爪当前开度")
    ret, data = read_with_retry(sock, RTU_MASTER_ID, REG_OPENLEN_ACT,
                                1, "读实际开度")
    if ret != SUCCESS:
        print("  [提示] 读取实际开度失败（地址 %d 为同段顺延假设，"
              "若持续失败请调整 REG_OPENLEN_ACT）" % REG_OPENLEN_ACT)
    else:
        print("  当前开度: %d（0=闭合完, 1000=张开完）" % data[0])

    # ---- 06H 设置速度与抓力（地址 11/12 为同段顺延，失败不阻塞动作） ----
    print_separator("设置速度与抓力")
    ret = modbus_write_signal_holding_registers(sock, RTU_MASTER_ID,
                                                REG_SPEED_SET, CLAW_SPEED)
    print_result("写速度 (addr=%d, val=%d)" % (REG_SPEED_SET, CLAW_SPEED),
                 ret)
    if ret != SUCCESS:
        print("  [提示] 速度寄存器可能不在此地址，跳过（不影响开度动作）")
    ret = modbus_write_signal_holding_registers(sock, RTU_MASTER_ID,
                                                REG_FORCE_SET, CLAW_FORCE)
    print_result("写抓力 (addr=%d, val=%d)" % (REG_FORCE_SET, CLAW_FORCE),
                 ret)
    if ret != SUCCESS:
        print("  [提示] 抓力寄存器可能不在此地址，跳过（不影响开度动作）")

    # ---- 06H 写目标开度（addr=10 为实测地址），并轮询实际开度到位 ----
    for i, target in enumerate(CLAW_TEST_POS):
        title = "测试位置 %d/%d: 目标开度=%d" % (
            i + 1, len(CLAW_TEST_POS), target)
        print_separator(title)
        ret = modbus_write_signal_holding_registers(
            sock, RTU_MASTER_ID, REG_OPENLEN_SET, target)
        msg = "modbus_write_signal_holding_registers (addr=%d, val=%d)" % (
            REG_OPENLEN_SET, target)
        print_result(msg, ret)
        if ret != SUCCESS:
            disconnect_robot(sock)
            return -1
        reached, last = wait_openlen_reach(sock, RTU_MASTER_ID, target)
        if not reached:
            print("  [提示] 未能确认到位（实际开度地址 %d 为同段顺延假设，"
                  "若读数异常请调整 REG_OPENLEN_ACT；开度命令已下发）" % REG_OPENLEN_ACT)
        else:
            print("  [到位] 开度 %d 到达目标 %d" % (last, target))
        time.sleep(0.3)

    # ---- 03H 读状态码与错误码 ----
    print_separator("读取夹爪状态")
    ret, data = read_with_retry(sock, RTU_MASTER_ID, REG_STATUS,
                                1, "读状态码")
    if ret == SUCCESS:
        status = data[0]
        status_map = {1: "张开到位", 2: "闭合到位", 3: "停止",
                      4: "夹取中", 5: "张开中", 6: "夹取到位"}
        status_desc = status_map.get(status, "未知")
        print("  状态码: %d（%s）" % (status, status_desc))
    ret, data = read_with_retry(sock, RTU_MASTER_ID, REG_ERRORCODE,
                                1, "读错误码")
    if ret != SUCCESS:
        print("  [提示] 读取错误码失败（地址 %d 为同段顺延假设）" % REG_ERRORCODE)
        return 0
    err = data[0]
    if err == 0:
        print("  错误码: 0（无错误）")
    else:
        bits = {0: "堵转", 1: "过温", 2: "过流", 3: "运行故障", 4: "内部通信故障"}
        faults = [desc for b, desc in bits.items() if err & (1 << b)]
        print("  错误码: %d（%s）" % (err, ", ".join(faults)))

    # ---- 断开连接 ----
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] 因时夹爪测试示例运行完毕")
    return 0


if __name__ == "__main__":
    main()
