#!/usr/bin/env python3
#
# ex_log_download.py
# @brief 日志下载相关接口
# @attention 确保机械臂供电正常、网络通信正常，且日志保存路径有效
# @note 运行步骤
#       运行: python3 ex_log_download.py
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


def main():
    # 定义IP地址和端口
    # 解析命令行参数: python3 ex_log_download.py [-ip 192.168.1.13]，默认 192.168.1.13
    ip = "192.168.1.13"
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        ip = sys.argv[2]
    port = "6001"

    # 定义日志文件路径
    count = 10
    log_path = "./log"

    # 获取 SDK 版本
    version = get_library_version()
    print("[INFO] SDK 版本: " + version)

    # 连接控制器
    print("[INFO] 连接控制器...")
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print("[ERROR] 连接失败，程序退出!")
        return -1
    print("[INFO] 连接成功 (socket=" + str(socket_fd) + ")")

    # 下载日志
    print("[INFO] 开始下载日志...")
    ret = log_download_by_quantity(socket_fd, count, log_path)
    if ret != SUCCESS:
        print("[ERROR] 日志下载失败，程序退出!")
        return -1
    print("[INFO] 日志下载完成")

    # 断开控制器连接
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        return -1
    print("[INFO] 断开连接成功")

    return 0


if __name__ == "__main__":
    exit(main())
