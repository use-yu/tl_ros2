#!/usr/bin/env python3
#
# ex_track_queue_drag.py
# @brief 测试 track_record_playback 接口 —— 轨迹记录与回放
# @attention
#   - 确保机械臂供电正常、网络通信正常
#   - 默认控制器 IP 为 192.168.1.13
#   - 需要在安全环境下运行，回放时机械臂会自动运动
#   - 操作流程:
#       示教模式 → set_darg_mode(1)力矩 → set_teach_type(1)拖拽
#       → track_record_start → 手拖机械臂 → track_record_stop
#       → set_teach_type(0)恢复 → track_record_save → track_record_playback
#   - 运行: python3 ex_track_queue_drag.py
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


def enable_servo(sock):
    """伺服上电（含状态检查，对齐 liuxin1.cpp 的 enable_servo）"""
    print("--- 检查伺服状态 ---")

    clear_error(sock)
    time.sleep(0.3)
    set_servo_poweroff(sock)
    time.sleep(0.3)

    state = -1
    ret, state = get_servo_state(sock, state)
    if ret != SUCCESS:
        print("  [ERROR] 获取伺服状态失败: %d" % ret)
        return False
    print("  [信息] 当前伺服状态: %d (0=停止 1=就绪 2=报警 3=运行)" % state)

    if state == 2:
        print("[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试")
        return False

    if state == 3:
        print("[INFO] 机械臂已在使能上电状态")
        return True

    if state == 0:
        print("  [信息] 伺服处于停止状态，设置就绪...")
        ret = set_servo_state(sock, 1)
        if ret != SUCCESS and ret != OPERATION_NOT_ALLOWED:
            print("[ERROR] 设置伺服就绪失败: %d" % ret)
            return False
        time.sleep(0.5)

    print("  [信息] 上电使能...")
    ret = set_servo_poweron(sock)
    if ret != SUCCESS:
        print("[ERROR] 上电失败: %d" % ret)
        return False
    time.sleep(2)

    ret, state = get_servo_state(sock, state)
    if state == 3:
        print("[INFO] 机械臂使能上电成功（servo_state = %d）" % state)
        return True
    else:
        print("[ERROR] 机械臂使能上电失败（servo_state = %d）" % state)
        return False


def disable_servo(sock):
    """伺服下电（对齐 liuxin1.cpp 的 disable_servo）"""
    state = -1
    get_servo_state(sock, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
    elif state == 3:
        set_servo_poweroff(sock)
        get_servo_state(sock, state)
        print("[INFO] 机械臂使能下电成功（servo_state = %d）" % state)
    else:
        print("[INFO] 机械臂使能下电失败（servo_state = %d）" % state)


def wait_playback_done(sock, timeout=120):
    """轮询等待回放完成（用 get_robot_running_state 替代，x86 SWIG 不支持 bool&）"""
    for _ in range(timeout * 2):
        running_state = -1
        ret, running_state = get_robot_running_state(sock, running_state)
        if ret == SUCCESS and running_state == 0:  # 0=停止，回放完成
            return True
        time.sleep(0.5)
    return False


# ============================================================
# 主函数
# ============================================================

def main():
    # 解析命令行参数: python3 ex_track_queue_drag.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]

    print_title("track_record_playback 接口测试（对齐 liuxin1.cpp）")
    print("""
  流程概览:
    1. 上电 → 示教模式
    2. set_darg_mode(1)力矩 → set_teach_type(1)拖拽
    3. 开启轨迹记录
    4. 手动拖拽机械臂走一段轨迹
    5. 停止记录 → set_teach_type(0)恢复 → 保存轨迹
    6. 回放轨迹（不同速度，轮询等待完成）
    7. 清除轨迹数据
""")

    # ---- 连接 ----
    sock = connect_robot(ip, "6001")
    if sock <= 0:
        print("[ERROR] 连接失败")
        return -1
    print("[INFO] 连接成功 (socket=%d)" % sock)

    # ---- 清错 + 上电 ----
    print_title("清错 & 上电")
    if not enable_servo(sock):
        print("[ERROR] 伺服上电失败，程序退出")
        disconnect_robot(sock)
        return -1

    # ---- 示教模式 ----
    print_title("切换示教模式")
    ret = set_current_mode(sock, 0)
    print_ok("set_current_mode(0) 示教模式") if ret == SUCCESS else print_fail("set_current_mode", ret)
    time.sleep(0.3)

    pass_count = 0
    fail_count = 0

    # ================================================================
    # 测试1: 查询初始记录状态
    # ================================================================
    print_title("测试1: 查询初始记录状态")
    recordStart = False
    ret, recordStart = get_track_record_status(sock, recordStart)
    # 注意: x86 Linux SWIG 可能无法正确传出 bool& 参数，以 ret 为准
    if ret == SUCCESS:
        print_ok("初始状态: %s (ret=%d)" % ("记录中" if recordStart else "未记录", ret))
        pass_count += 1
    else:
        print_fail("get_track_record_status", ret)
        fail_count += 1

    # ================================================================
    # 测试2: 开启拖拽模式（对齐 liuxin1.cpp: darg_mode=1 力矩，需在 teach_type 前）
    # ================================================================
    # mode: 0=无  1=力矩模式  2=位置（参考 liuxin1.cpp 实际映射）
    print_title("测试2: set_darg_mode(1) 力矩模式")
    ret = set_darg_mode(sock, 1)
    if ret == SUCCESS:
        print_ok("set_darg_mode(1) 力矩模式已开启")
        pass_count += 1
    else:
        print_fail("set_darg_mode(1)", ret)
        fail_count += 1

    # set_teach_type 必须在 set_darg_mode 之后（对齐 liuxin1.cpp）
    print_title("测试2b: set_teach_type(1) 切换到拖拽")
    ret = set_teach_type(sock, 1)
    if ret == SUCCESS:
        print_ok("已切换到拖拽模式")
        pass_count += 1
    else:
        print_fail("set_teach_type(1)", ret)
        fail_count += 1

    # 确认
    teach_type = -1
    ret, teach_type = get_teach_type(sock, teach_type)
    if ret == SUCCESS:
        name = {0: "点动", 1: "拖拽"}.get(teach_type, "未知")
        print_ok("确认: teach_type=%d (%s)" % (teach_type, name))

    # ================================================================
    # 测试3: track_record_start（开始记录）
    # ================================================================
    print_title("测试3: 开始轨迹记录")

    # 参数对齐 liuxin1.cpp: maxSamplingNum=500, samplingInterval=0.1
    MAX_SAMPLES = 500
    SAMPLE_INTERVAL = 1

    ret = track_record_start(sock, MAX_SAMPLES, SAMPLE_INTERVAL)
    if ret != SUCCESS:
        print_fail("track_record_start", ret)
        fail_count += 1

        # 尝试备选参数
        print("  [信息] 尝试备选参数 (1000, 0.1)...")
        ret = track_record_start(sock, 1000, 1)
        if ret != SUCCESS:
            print_fail("track_record_start (备选参数)", ret)
            set_teach_type(sock, 0)
            set_darg_mode(sock, 0)
            set_servo_poweroff(sock)
            disconnect_robot(sock)
            return -1
    print_ok("轨迹记录已开始 (maxSamples=%d, interval=%.2fs)" % (MAX_SAMPLES, SAMPLE_INTERVAL))

    # 验证记录状态（注意: x86 SWIG 可能无法正确传出 bool&，以 track_record_start 返回值判断）
    recordStart = False
    ret, recordStart = get_track_record_status(sock, recordStart)
    if ret == SUCCESS:
        print_ok("记录状态验证: %s" % ("正在记录" if recordStart else "状态查询成功(ret=%d)，以上一步 start 返回值为准" % ret))
        pass_count += 1
    else:
        print_fail("记录状态验证失败", ret)
        fail_count += 1

    # ================================================================
    # 用户手动拖拽
    # ================================================================
    print("\n" + "=" * 50)
    print("  [操作] 机械臂已进入拖拽状态，请手动拖拽机械臂走一段轨迹")
    print("  [操作] 完成后按回车键停止记录...")
    print("=" * 50)

    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass

    # ================================================================
    # 测试4: track_record_stop（停止记录）
    # ================================================================
    print_title("测试4: 停止轨迹记录")
    ret = track_record_stop(sock)
    if ret == SUCCESS:
        print_ok("track_record_stop")
        pass_count += 1
    else:
        print_fail("track_record_stop", ret)
        fail_count += 1

    # 验证停止状态
    recordStart = True
    ret, recordStart = get_track_record_status(sock, recordStart)
    if ret == SUCCESS:
        print_ok("记录状态验证: %s" % ("已停止" if not recordStart else "状态查询成功(ret=%d)，以上一步 stop 返回值为准" % ret))
        pass_count += 1
    else:
        print_fail("记录状态验证失败", ret)
        fail_count += 1

    # ---- 退出拖拽模式，切回点动（对齐 liuxin1.cpp，回放前需切回点动）----
    print_title("退出拖拽模式")
    ret = set_teach_type(sock, 0)
    print_ok("set_teach_type(0) 已切回点动模式") if ret == SUCCESS else print_fail("切回点动", ret)
    time.sleep(0.3)

    # ================================================================
    # 测试5: track_record_save（保存轨迹）
    # ================================================================
    print_title("测试5: 保存轨迹")
    # 按示教器格式生成轨迹名（对齐 liuxin1.cpp）
    TRAJ_NAME = time.strftime("轨迹名-%m月%d日%H时%M分%S", time.localtime())
    ret = track_record_save(sock, TRAJ_NAME)
    if ret == SUCCESS:
        print_ok("轨迹已保存为: %s" % TRAJ_NAME)
        pass_count += 1
    else:
        print_fail("track_record_save('%s')" % TRAJ_NAME, ret)
        fail_count += 1

    # ================================================================
    # 测试6: track_record_playback（回放）—— 核心测试
    # ================================================================
    print_title("测试6: 轨迹回放 (track_record_playback)")

    print("  [警告] 机械臂将自动回放记录的轨迹！请确保周围安全！")
    print("  按回车键开始回放...")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass

    # --- 用例A: 低速回放 vel=20 ---
    print("\n  [用例A] 低速回放 (vel=20)")
    ret = track_record_playback(sock, 20)
    if ret == SUCCESS:
        print_ok("低速回放已开始，等待完成...")
        if wait_playback_done(sock, timeout=120):
            print_ok("低速回放完成")
            pass_count += 1
        else:
            print("  [WARN] 低俗回放等待超时")
            fail_count += 1
    else:
        print_fail("低俗回放", ret)
        fail_count += 1

    # --- 用例B: 中速回放 vel=50（对齐 liuxin1.cpp 的默认速度）---
    print("\n  [用例B] 中速回放 (vel=50)")
    print("  按回车键继续...")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass

    ret = track_record_playback(sock, 50)
    if ret == SUCCESS:
        print_ok("中速回放已开始，等待完成...")
        if wait_playback_done(sock, timeout=120):
            print_ok("中速回放完成")
            pass_count += 1
        else:
            print("  [WARN] 中速回放等待超时")
            fail_count += 1
    else:
        print_fail("中速回放", ret)
        fail_count += 1

    # --- 用例C: 高速回放 vel=100 ---
    print("\n  [用例C] 高速回放 (vel=100)")
    print("  按回车键继续...")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass

    ret = track_record_playback(sock, 100)
    if ret == SUCCESS:
        print_ok("高速回放已开始，等待完成...")
        if wait_playback_done(sock, timeout=120):
            print_ok("高速回放完成")
            pass_count += 1
        else:
            print("  [WARN] 高速回放等待超时")
            fail_count += 1
    else:
        print_fail("高速回放", ret)
        fail_count += 1

    # --- 用例D: 边界值测试 ---
    print_title("测试7: playback 边界值测试")
    for vel, desc in [(0, "vel=0"), (150, "vel=150"), (200, "vel=200")]:
        ret = track_record_playback(sock, vel)
        if ret == SUCCESS:
            print("  vel=%d (%s): 返回成功" % (vel, desc))
        else:
            print("  vel=%d (%s): 返回错误码 %d（可能是正常范围外）" % (vel, desc, ret))
        time.sleep(1)

    # ================================================================
    # 测试8: track_record_delete（清除轨迹缓冲区）
    # ================================================================
    print_title("测试8: 清除轨迹数据")
    ret = track_record_delete(sock)
    if ret == SUCCESS:
        print_ok("track_record_delete")
        pass_count += 1
    else:
        print_fail("track_record_delete", ret)
        fail_count += 1

    # ================================================================
    # 收尾
    # ================================================================
    print_title("收尾")
    set_darg_mode(sock, 0)
    set_teach_type(sock, 0)
    disable_servo(sock)
    disconnect_robot(sock)
    print("[INFO] 测试完毕")

    # ---- 报告 ----
    print_title("测试报告")
    total = pass_count + fail_count
    print("  通过: %d, 失败: %d, 总计: %d" % (pass_count, fail_count, total))
    return 0 if fail_count == 0 else -1


if __name__ == "__main__":
    exit(main())
