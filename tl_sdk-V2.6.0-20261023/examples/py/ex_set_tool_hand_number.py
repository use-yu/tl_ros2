#!/usr/bin/env python3
#
# ex_set_tool_hand_number.py
# @brief 测试 set_tool_hand_number / get_tool_hand_number 接口
# @attention
#   - 确保机械臂供电正常、网络通信正常
#   - 默认控制器 IP 为 192.168.1.13
#   - 建议在示教模式下操作
#   - 运行: python3 ex_set_tool_hand_number.py
#

import sys
import os
import platform
import time

_script_dir = os.path.dirname(os.path.abspath(__file__))
if platform.system() == "Windows":
    sys.path.append(os.path.join(_script_dir, "../../lib/windows/python"))
else:
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/x86/python"))

from tl_interface import *


# ============================================================
# 工具函数
# ============================================================

def print_ok(msg):
    print("  [OK] " + msg)

def print_fail(msg, ret):
    print("  [FAIL] " + msg + "，错误码: " + str(ret))

def print_title(title):
    print("\n" + "=" * 50)
    print("  " + title)
    print("=" * 50)


# ============================================================
# 主函数
# ============================================================

def main():
    # 解析命令行参数: python3 ex_set_tool_hand_number.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    print_title("set_tool_hand_number / get_tool_hand_number 接口测试")

    # ---- 连接 ----
    sock = connect_robot(ip, "6001")
    if sock <= 0:
        print("[ERROR] 连接失败")
        return -1
    print("[INFO] 连接成功 (socket=%d)" % sock)

    # ---- 清错 ----
    clear_error(sock)
    time.sleep(0.3)

    pass_count = 0
    fail_count = 0

    # ---- 读取当前工具手编号 ----
    print_title("1. 读取当前工具手编号")
    current_tool = 0
    ret, current_tool = get_tool_hand_number(sock, current_tool)
    if ret == SUCCESS:
        print_ok("当前工具手编号: %d" % current_tool)
        pass_count += 1
    else:
        print_fail("get_tool_hand_number", ret)
        fail_count += 1

    # ---- 测试 set + get 组合 ----
    print_title("2. 设置并验证工具手编号")

    test_cases = [
        (1,  "正常值"),
        (2,  "正常值"),
        (10, "较大值"),
        (0,  "边界值: 0"),
        (20, "边界值: 20"),
    ]

    for tool_num, desc in test_cases:
        print("\n  --- 设置 toolNum=%d (%s) ---" % (tool_num, desc))

        ret = set_tool_hand_number(sock, tool_num)
        if ret != SUCCESS:
            print_fail("set_tool_hand_number(%d)" % tool_num, ret)
            fail_count += 1
            continue

        time.sleep(0.1)

        # 读回验证
        readback = 0
        ret, readback = get_tool_hand_number(sock, readback)
        if ret != SUCCESS:
            print_fail("get_tool_hand_number (验证)" % tool_num, ret)
            fail_count += 1
            continue

        if readback == tool_num:
            print_ok("设置 %d → 读回 %d，一致" % (tool_num, readback))
            pass_count += 1
        else:
            print_fail("设置 %d → 读回 %d，不一致！" % (tool_num, readback), -1)
            fail_count += 1

    # ---- 恢复原始值 ----
    print_title("3. 恢复原始工具手编号")
    ret = set_tool_hand_number(sock, current_tool)
    if ret == SUCCESS:
        print_ok("已恢复为 %d" % current_tool)
        pass_count += 1
    else:
        print_fail("恢复失败", ret)
        fail_count += 1

    # ---- 断开 ----
    print_title("收尾")
    disconnect_robot(sock)
    print("[INFO] 已断开连接")

    # ---- 测试报告 ----
    print_title("测试报告")
    total = pass_count + fail_count
    print("  通过: %d, 失败: %d, 总计: %d" % (pass_count, fail_count, total))
    print("  结果: %d/%d 通过" % (pass_count, total))

    return 0 if fail_count == 0 else -1


if __name__ == "__main__":
    exit(main())
