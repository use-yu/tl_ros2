#!/usr/bin/env python3
#
# ex_omnipicker_claw.py
# @brief OmniPicker 夹爪（Modbus RTU 从站）控制示例
# @attention
#   - 夹爪需先在上位机把 modbus_mode 置为 True（Modbus RTU）并保存、重新上电，
#     拨码开关切到 RS485 或 UART；默认 CAN/CAN-FD 下不走 Modbus
#   - 夹爪经串口接控制器，主站参数（串口号、波特率、从站 ID）请按实际接线修改
#     下方"RTU 主站配置区"；从站地址复用夹爪 can_node_id
#   - 位置/速度/力矩/加(减)速度取值均为 0x00~0xFF，位置 0x00=夹紧 0xFF=完全张开
#   - 夹爪会真实动作：运行前请确保夹爪空闲、指间无物体
# @note 运行步骤
#       运行: python3 ex_omnipicker_claw.py
#       编译对照: ./ex_omnipicker_claw（C++ 版示例）
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
# 控制器连接配置（按实际修改）
# ============================================================
ROBOT_IP   = "192.168.1.13"  # 机械臂控制器 IP（可用命令行参数 -ip 覆盖）
ROBOT_PORT = "6001"          # 控制器端口

# ============================================================
# RTU 主站配置区（请根据实际接线自行修改）
# ============================================================
RTU_MASTER_ID = 1            # 控制器侧主站配方编号（1~9）
RTU_SLAVE_ID  = 1            # 夹爪从站地址，复用夹爪 can_node_id（上位机可查）
RTU_PORT      = 2            # 控制器串口号（按接线修改）
RTU_BAUDRATE  = 115200       # 波特率（固件 3.3.5 起可经上位机 uart_baudrate 修改）
RTU_CHECK_BIT = "None"       # 校验位: None/Even/Odd
RTU_DATA_BIT  = 8            # 数据位
RTU_STOP_BIT  = 1            # 停止位

# ============================================================
# OmniPicker 保持寄存器（地址按手册编号直传，startAddress=True 表示 0 起始）
# 功能码: 03H 读保持寄存器 / 10H 写多个保持寄存器
# ============================================================
REG_FIRMWARE_VERSION = 0     # 固件版本号（只读）
REG_TARGET_POS       = 10    # 目标位置 0x00~0xFF（0x00=夹紧 0xFF=完全张开）
REG_TARGET_SPEED     = 11    # 目标速度 0x00~0xFF（0xFF=最大速度）
REG_TARGET_FORCE     = 12    # 目标力矩 0x00~0xFF（0xFF=最大力矩）
REG_TARGET_ACC       = 13    # 目标加速度 0x00~0xFF（0xFF=最大加速度）
REG_TARGET_DEC       = 14    # 目标减速度 0x00~0xFF（0xFF=最大减速度）
REG_MOTION_TRIGGER   = 15    # 运动触发：写非零后按 10~14 运动，开始运动后自动清零
REG_FAULT_CODE       = 20    # 当前错误码（只读）
REG_STATE            = 21    # 当前状态（只读）
REG_CURRENT_POS      = 22    # 当前位置（只读）
REG_CURRENT_SPEED    = 23    # 当前速度（只读）
REG_CURRENT_FORCE    = 24    # 当前力矩（只读）

POS_OPEN      = 0xFF         # 完全张开
POS_HALF      = 0x7F         # 50% 行程
POS_CLOSE     = 0x00         # 夹紧
POS_TOLERANCE = 3            # 到位判定的位置容差（约 1.4mm，行程 120mm 对应 0x00~0xFF）
CMD_SPEED     = 0xFF         # 运动指令速度
CMD_FORCE     = 0xFF         # 运动指令力矩
CMD_ACC       = 0xFF         # 运动指令加速度
CMD_DEC       = 0xFF         # 运动指令减速度

# 演示位置序列（按需修改）
DEMO_POSITIONS = [POS_OPEN, POS_HALF, POS_CLOSE]

FAULT_DESC = {0: "无故障", 1: "过温警报", 2: "超速警报",
              3: "初始化故障警报", 4: "超限检测警报"}
STATE_DESC = {0: "已到达目标位置", 1: "夹爪移动中", 2: "夹爪堵转", 3: "物体掉落"}


# ============================================================
# 工具函数
# ============================================================

def print_result(msg, ret):
    """@brief 打印操作结果"""
    if ret == SUCCESS:
        print("  [成功] %s" % msg)
    else:
        # 错误码按整数打印：直接 str(ret) 会输出绑定的限定名而非可读错误码
        print("  [失败] %s，错误码: %d" % (msg, ret))


def print_separator(title):
    """@brief 打印分隔标题"""
    print("\n========== %s ==========" % title)


def read_holding_registers(sock, address, quantity, desc):
    """@brief 读保持寄存器（03H），失败返回 None"""
    data = VectorInt()
    ret = modbus_read_holding_registers(sock, RTU_MASTER_ID, address,
                                        quantity, data)
    print_result("%s（addr=%d, qty=%d）" % (desc, address, quantity), ret)
    if ret != SUCCESS or len(data) < quantity:
        return None
    return list(data)[:quantity]


def print_status(sock):
    """@brief 读 20~24 地址并打印错误码/状态/位置/速度/力矩"""
    data = read_holding_registers(sock, REG_FAULT_CODE, 5, "读夹爪状态")
    if data is None:
        print("  [提示] 状态读取失败，请检查接线与从站 ID (RTU_SLAVE_ID)")
        return None
    fault, state, pos, speed, force = data
    print("  错误码: %d（%s）" % (fault, FAULT_DESC.get(fault, "未知")))
    print("  状态: %d（%s）" % (state, STATE_DESC.get(state, "未知")))
    print("  位置: 0x%02X  速度: 0x%02X  力矩: 0x%02X"
          % (pos & 0xFF, speed & 0xFF, force & 0xFF))
    return data


def move_to(sock, pos):
    """@brief 写 10~14 目标参数 + 15 触发（10H 写 6 个寄存器）"""
    cmd = VectorInt([pos, CMD_SPEED, CMD_FORCE, CMD_ACC, CMD_DEC, 1])
    ret = modbus_write_multiple_holding_registers(sock, RTU_MASTER_ID,
                                                  REG_TARGET_POS, cmd)
    print_result("下发运动指令（pos=0x%02X, speed=0x%02X, force=0x%02X）"
                 % (pos, CMD_SPEED, CMD_FORCE), ret)
    return ret == SUCCESS


def wait_motion_done(sock, target_pos, timeout=10.0):
    """@brief 轮询状态与当前位置，直到到位（状态 0 且位置进入容差）或堵转（2）/掉落（3）或超时"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = VectorInt()
        # 同时读 21（状态）与 22（位置）：刚下发指令时读到的可能仍是上一帧的
        # 「已到达目标位置」，只看状态会在夹爪尚未起步时误判到位
        ret = modbus_read_holding_registers(sock, RTU_MASTER_ID,
                                            REG_STATE, 2, data)
        if ret == SUCCESS and len(data) >= 2:
            state = data[0]
            pos = data[1] & 0xFF
            if state == 0 and abs(pos - target_pos) <= POS_TOLERANCE:
                return True
            if state in (2, 3):
                print("  [警告] 运动异常: %s" % STATE_DESC[state])
                return False
        time.sleep(0.2)
    print("  [警告] 等待到位超时（%.1f s）" % timeout)
    return False


# ============================================================
# 主函数
# ============================================================

def main():
    # 解析命令行参数: python3 ex_omnipicker_claw.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = ROBOT_IP
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    # ---- 获取 SDK 版本 ----
    print("[信息] SDK 版本: %s" % get_library_version())

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
    param.startAddress = True               # 寄存器地址按手册编号直传（0 起始）
    param.RTU.slaveId = RTU_SLAVE_ID
    param.RTU.port = RTU_PORT
    param.RTU.baudrate = RTU_BAUDRATE
    param.RTU.checkBit = RTU_CHECK_BIT
    param.RTU.dataBit = RTU_DATA_BIT
    param.RTU.stopBit = RTU_STOP_BIT

    ret = modbus_set_master_parameter(sock, RTU_MASTER_ID, param)
    print_result("modbus_set_master_parameter (id=%d, RTU)" % RTU_MASTER_ID,
                 ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1
    print("  从站 ID: %d, 串口: %d, 波特率: %d" %
          (param.RTU.slaveId, param.RTU.port, param.RTU.baudrate))
    print("  校验: %s, 数据位: %d, 停止位: %d" %
          (param.RTU.checkBit, param.RTU.dataBit, param.RTU.stopBit))

    # ---- 打开 Modbus 主站 ----
    ret = modbus_open_master(sock, RTU_MASTER_ID)
    print_result("modbus_open_master (id=%d)" % RTU_MASTER_ID, ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1
    time.sleep(0.5)

    # ---- 读固件版本与当前状态 ----
    print_separator("读取固件版本与状态")
    data = read_holding_registers(sock, REG_FIRMWARE_VERSION, 1, "读固件版本号")
    if data is not None:
        print("  固件版本号: %d（0x%04X，Modbus RTU 需固件 3.3.5 及以上）"
              % (data[0], data[0]))
    print_status(sock)

    # ---- 逐位置下发运动指令并等待到位 ----
    for i, pos in enumerate(DEMO_POSITIONS):
        print_separator("测试位置 %d/%d: 目标位置=0x%02X"
                        % (i + 1, len(DEMO_POSITIONS), pos))
        if not move_to(sock, pos):
            continue
        if wait_motion_done(sock, pos):
            print("  [到位]")
        print_status(sock)

    # ---- 断开连接 ----
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] OmniPicker 夹爪示例运行完毕")
    return 0


if __name__ == "__main__":
    main()
