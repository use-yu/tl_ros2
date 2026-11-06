#!/usr/bin/env python3
#
# ex_handeye_calibration.py
# @brief 基于 Intel D435i 的眼在手（Eye-in-Hand）手眼标定程序
#
# 实操使用说明链接：https://www.tlibot.com/arm/544.html
#
# 标定原理:
#   AX = XB  —  A = 机器人末端在基座下的位姿
#               B = 标定板在相机下的位姿（solvePnP）
#               X = 相机在机器人末端下的位姿（待求）
#
# @attention
#   - 确保机械臂供电正常、网络通信正常
#   - 需要连接 Intel D435i 相机
#   - 标定板需在相机视野内清晰可见
#   - 采集时请覆盖工作空间不同位置和姿态（建议 30° 以上朝向变化）
#   - 默认控制器 IP 为 192.168.1.13，可在 cal_config.yaml 中修改
#
# @note 运行步骤
#       安装依赖: pip install pyrealsense2 opencv-contrib-python numpy pyyaml
#       运行: python3 ex_handeye_calibration.py
#
# @note 操作键
#   S — 保存当前帧和机械臂位姿
#   C — 开始标定计算
#   R — 重置已采集数据
#   Q — 退出程序
#
# @note 生成文件 (均在 cal_data/ 目录下)
#   images/*.png          采集的标定图像
#   poses.csv             每张图像对应的机械臂末端位姿 (x,y,z,rx,ry,rz,qx,qy,qz,qw)
#   result.yaml           标定结果，包含:
#     camera_intrinsics     相机内参 (fx, fy, cx, cy)
#     distortion_coefficients  畸变系数
#     handeye_result:
#       T_cam2gripper        相机在末端下的位姿 (平移 + 旋转矩阵)
#       T_gripper2cam        末端在相机下的位姿
#     verification:
#       avg_solvePnP_reprojection_error_px  重投影误差 (px)
#       board_position_base_frame           标定板在基座下位置 (均值±标准差)
#       consistency_mm                      位置一致性 (越小越稳定)
#     configuration        标定配置参数
#
# @note 标定结果验证
#       运行 ex_handeye_verify.py 加载 result.yaml，
#       在实时画面中显示标定板在基座下的位置，
#       按 M 键运动到目标位姿确认标定精度。
#

import sys
import os
import time
import csv
import platform

# 优化 Qt 后端性能，消除 Wayland 下的卡顿
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import numpy as np
import cv2
import yaml

# ---- 添加 SDK Python 绑定路径 ----
_script_dir = os.path.dirname(os.path.abspath(__file__))
if platform.system() == "Windows":
    sys.path.append(os.path.join(_script_dir, "../../lib/windows/python"))
elif platform.machine() in ("aarch64", "arm64"):
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/arm64/python"))
else:
    sys.path.append(os.path.join(_script_dir, "../../lib/linux/x86/python"))
    
from tl_interface import *

# 尝试导入 pyrealsense2
try:
    import pyrealsense2 as rs
except ImportError:
    print("[ERROR] 请安装 pyrealsense2: pip install pyrealsense2")
    sys.exit(1)


# ============================================================
# 使能上电 / 下电
# ============================================================


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
    """使能伺服下电"""
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
    elif state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print("[INFO] 机械臂使能下电成功（servo_state = " + str(state) + "）")
        return
    print("[INFO] 机械臂使能下电失败（servo_state = " + str(state) + "）")


# ============================================================
# 相机初始化辅助函数
# ============================================================


def init_camera(cfg):
    """
    初始化 D435i 相机，返回 (pipeline, camera_matrix, dist_coeffs)
    """
    cam_cfg = cfg["camera"]
    pipeline = rs.pipeline()
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.color,
                            cam_cfg["width"],
                            cam_cfg["height"],
                            rs.format.bgr8,
                            cam_cfg["fps"])
    profile = pipeline.start(rs_config)
    color_profile = profile.get_stream(rs.stream.color)
    color_intrin = color_profile.as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([
        [color_intrin.fx, 0, color_intrin.ppx],
        [0, color_intrin.fy, color_intrin.ppy],
        [0, 0, 1]
    ], dtype=np.float64)
    dist_coeffs = np.array(color_intrin.coeffs, dtype=np.float64)
    print(f"[信息] D435i 已启动 ({cam_cfg['width']}x{cam_cfg['height']} @ {cam_cfg['fps']}fps)")
    print(f"  相机内参 fx={color_intrin.fx:.3f}, fy={color_intrin.fy:.3f}")
    print(f"  主点 cx={color_intrin.ppx:.3f}, cy={color_intrin.ppy:.3f}")
    return pipeline, camera_matrix, dist_coeffs


def poll_frame(pipeline):
    """
    非阻塞读取一帧彩色图，返回 numpy array 或 None
    """
    try:
        ok, frames = pipeline.try_wait_for_frames(timeout_ms=100)
        if ok:
            color = frames.get_color_frame()
            if color:
                return np.asanyarray(color.get_data())
    except RuntimeError:
        pass
    return None


# ============================================================
# 旋转矩阵 / 位姿转换工具
# ============================================================

def robot_xyz_to_matrix(x, y, z, rx, ry, rz):
    """
    将机器人位姿 (x,y,z,rx,ry,rz) 转为 4×4 齐次变换矩阵

    旋转顺序: R = Rx(roll) · Ry(pitch) · Rz(yaw)  (外旋 XYZ)
    等价于内旋 ZYX: 先绕自身 Z 转 rz, 再绕 Y 转 ry, 再绕 X 转 rx

    @return 4×4 numpy 数组, dtype=float64
    """
    cos, sin = np.cos, np.sin
    Rx = np.array([
        [1, 0, 0],
        [0, cos(rx), -sin(rx)],
        [0, sin(rx), cos(rx)]
    ])
    Ry = np.array([
        [cos(ry), 0, sin(ry)],
        [0, 1, 0],
        [-sin(ry), 0, cos(ry)]
    ])
    Rz = np.array([
        [cos(rz), -sin(rz), 0],
        [sin(rz), cos(rz), 0],
        [0, 0, 1]
    ])
    R = Rx @ Ry @ Rz
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_rpy(T):
    """
    从 4×4 齐次矩阵中提取 (x, y, z, rx, ry, rz)

    旋转顺序: R = Rx · Ry · Rz   (外旋 XYZ)
    """
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    R = T[:3, :3]
    # 从 R = Rx·Ry·Rz 中解出 rx, ry, rz
    # R[0,2] = sin(ry)
    sy = R[0, 2]
    # 避免 Gimbal Lock
    if np.abs(sy) < 1.0 - 1e-12:
        rx = np.arctan2(-R[1, 2], R[2, 2])
        ry = np.arcsin(sy)
        rz = np.arctan2(-R[0, 1], R[0, 0])
    else:
        # ry = ±90°, Gimbal Lock
        rx = np.arctan2(R[1, 0], R[1, 1])
        ry = np.pi / 2 * (1 if sy > 0 else -1)
        rz = 0.0
    return x, y, z, rx, ry, rz


def matrix_to_quat(T):
    """从 4×4 齐次矩阵中提取 (x, y, z, qx, qy, qz, qw)"""
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    R = T[:3, :3]
    qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return x, y, z, qx, qy, qz, qw


def get_robot_pose(sock, coord):
    """
    获取机械臂当前末端位姿

    @param sock: socket fd
    @param coord: Coord 枚举（如 Coord.BASE 基坐标系 / Coord.TOOL 工具）
    @return: (ret, T_4x4) 成功时 T_4x4 为 4×4 齐次矩阵
    """
    pos = VectorDouble()
    # 预分配 14 个元素，确保 SWIG 输出有足够空间
    for _ in range(14):
        pos.push_back(0.0)
    ret = get_current_position(sock, coord, pos)
    if ret != SUCCESS:
        return ret, None
    # pos[7..13] = x, y, z, rx, ry, rz
    T = robot_xyz_to_matrix(pos[0], pos[1], pos[2],
                            pos[3], pos[4], pos[5])
    # print(f"  [调试] 原始 pos: {[f'{v:.4f}' for v in pos]}")
    return ret, T


# ============================================================
# 棋盘格检测
# ============================================================

def detect_checkerboard(gray, pattern_size):
    """
    检测棋盘格角点

    @param gray: 灰度图
    @param pattern_size: (cols, rows) 内角点数
    @return: (found, corners) corners 为亚像素精度的角点坐标
    """
    found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    if found:
        # 亚像素精细化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return found, corners


def get_board_pose_cam(obj_points, corners, camera_matrix, dist_coeffs):
    """
    求解棋盘格在相机坐标系下的位姿 (solvePnP)

    @param obj_points:     棋盘格世界坐标 (N×3)
    @param corners:        检测到的图像角点
    @param camera_matrix:  相机内参 (3×3)
    @param dist_coeffs:    畸变系数
    @return: (ok, (x,y,z,rx,ry,rz), dist_mm)
             ok=True 时，元组为棋盘格相对相机的位姿（相机系下，单位 mm/rad），
             dist_mm 为相机光心到棋盘格原点的距离
    """
    ret, rvec, tvec = cv2.solvePnP(obj_points, corners,
                                   camera_matrix, dist_coeffs)
    if not ret:
        return False, None, 0.0
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    x, y, z, rx, ry, rz = matrix_to_rpy(T)
    dist_mm = float(np.linalg.norm(tvec))
    return True, (x, y, z, rx, ry, rz), dist_mm


# ============================================================
# 主程序
# ============================================================

def main():
    # ---- 加载配置 ----
    config_path = os.path.join(_script_dir, "config", "cal_config.yaml")
    if not os.path.isfile(config_path):
        print(f"[ERROR] 配置文件不存在: {config_path}")
        return
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    robot_cfg = cfg["robot"]
    # 命令行参数优先于 cal_config.yaml: python3 ex_handeye_calibration.py [-ip 192.168.1.13]
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        robot_cfg["ip"] = sys.argv[2]
    cal_cfg = cfg["calibration"]
    board_cfg = cfg["calibration_board"]

    pattern_size = tuple(board_cfg["pattern_size"])   # (cols, rows)
    square_size = board_cfg["square_size_mm"]
    coord_sys = robot_cfg["coord_system"]
    if coord_sys not in (1, 2):
        print(f"[ERROR] 配置的位姿坐标系不合法: coord_system={coord_sys}（支持 1=基坐标 / 2=工具）")
        sys.exit(-1)
    coord_name = {1: "基坐标系（直角坐标系）", 2: "工具坐标系"}.get(coord_sys, f"未知({coord_sys})")
    print(f"[信息] 位姿坐标系: coord_system={coord_sys} ({coord_name})")
    min_images = cal_cfg["min_images"]
    max_images = cal_cfg["max_images"]

    # ---- 创建输出目录 ----
    output_dir = os.path.join(_script_dir, cal_cfg["output_dir"])
    os.makedirs(output_dir, exist_ok=True)
    image_dir = os.path.join(output_dir, "images")
    os.makedirs(image_dir, exist_ok=True)

    # ---- 连接机械臂 ----
    print(f"[信息] 连接控制器 {robot_cfg['ip']}:{robot_cfg['port']}...")
    sock = connect_robot(robot_cfg["ip"], robot_cfg["port"])
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        return
    print(f"[信息] 连接成功 (socket={sock})")

    # ---- 使能上电 ----
    enable_servo(sock)

    # ---- 启动 D435i 相机 ----
    try:
        pipeline, camera_matrix, dist_coeffs = init_camera(cfg)
    except Exception as e:
        print(f"[ERROR] D435i 启动失败: {e}")
        print("  请检查相机连接: rs-enumerate-devices")
        disable_servo(sock)
        disconnect_robot(sock)
        return

    # ---- 初始化棋盘格世界坐标 ----
    # 标定板坐标系: Z=0 平面, 角点按行排列
    obj_points = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float64)
    obj_points[:, :2] = np.mgrid[0:pattern_size[0],
                                  0:pattern_size[1]].T.reshape(-1, 2)
    obj_points *= square_size

    # ---- 数据缓存 ----
    saved_images = []            # 已保存的图像文件名列表（用于排序）
    saved_poses = []             # 对应的机器人位姿 (4×4 矩阵)
    saved_count = 0
    calibrating = False
    reset_confirm = False

    # ---- 自动加载已保存的数据 ----
    csv_path = os.path.join(output_dir, "poses.csv")
    if os.path.isfile(csv_path) and os.path.isdir(image_dir):
        try:
            import csv as csv_mod
            with open(csv_path, "r") as f:
                reader = csv_mod.DictReader(f)
                for row in reader:
                    img_name = row["image"]
                    img_path = os.path.join(image_dir, img_name)
                    if not os.path.isfile(img_path):
                        continue
                    x = float(row["x"])
                    y = float(row["y"])
                    z = float(row["z"])
                    rx = float(row["rx"])
                    ry = float(row["ry"])
                    rz = float(row["rz"])
                    T = robot_xyz_to_matrix(x, y, z, rx, ry, rz)
                    saved_images.append(img_name)
                    saved_poses.append(T)
                    saved_count += 1
            if saved_count > 0:
                print(f"[信息] 从 {csv_path} 自动加载 {saved_count} 组数据，按 C 可直接标定")
        except Exception as e:
            print(f"[信息] 加载历史数据失败: {e}，将全新采集")

    # ---- 主循环 ----
    print(f"\n{'='*60}")
    print("  眼在手手眼标定 — 操作说明")
    print(f"{'='*60}")
    print("  S — 保存当前帧 + 机械臂位姿")
    print("  C — 开始标定计算")
    print("  R — 重置已采集数据（再按一次确认）")
    print("  Q — 退出程序")
    print(f"{'='*60}\n")

    cv2.namedWindow("Hand-Eye Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Hand-Eye Calibration", 960, 720)

    running = True
    while running:
        # ---- 获取最新帧（非阻塞）----
        frame = poll_frame(pipeline)

        if frame is None:
            cv2.waitKey(10)
            continue

        # ---- 检测棋盘格 ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = detect_checkerboard(gray, pattern_size)

        # ---- 绘制状态 ----
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, pattern_size, corners, found)
            status_text = f"Board detected | Captured: {saved_count}/{min_images}"
        else:
            status_text = f"No board | Captured: {saved_count}/{min_images}"

        if calibrating:
            status_text = "Calibrating..."

        cv2.putText(display, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if saved_count >= min_images:
            cv2.putText(display, "Press C to calibrate", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # ---- 画面叠加: 棋盘格相对相机位姿 + 距离（英文, 左上角状态文字下方）----
        if found:
            ok_pnp, pose_cam, dist_mm = get_board_pose_cam(obj_points, corners,
                                                           camera_matrix,
                                                           dist_coeffs)
            if ok_pnp:
                bx, by, bz, brx, bry, brz = pose_cam
                info_lines = [
                    f"Board-cam X={bx:.1f} Y={by:.1f} Z={bz:.1f} mm",
                    f"RX={brx:.3f} RY={bry:.3f} RZ={brz:.3f} rad",
                    f"Dist={dist_mm:.1f} mm",
                ]
                # 动态贴合上方文字: 有 "Press C" 提示(y=60)则从 y=82 起, 否则紧贴状态(y=30)从 y=52 起
                y0 = 82 if saved_count >= min_images else 52
                for i, line in enumerate(info_lines):
                    cv2.putText(display, line, (10, y0 + i * 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        cv2.imshow("Hand-Eye Calibration", display)

        # ---- 键盘事件 ----
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q') or key == ord('Q'):
            print("[信息] 用户退出")
            running = False
            break

        elif key == ord('r') or key == ord('R'):
            if reset_confirm:
                # 第二次按 R，确认重置
                saved_images.clear()
                saved_poses.clear()
                saved_count = 0
                reset_confirm = False
                print("[信息] 已重置所有采集数据")
            else:
                reset_confirm = True
                print("[信息] 再按一次 R 确认重置数据")

        elif key == ord('s') or key == ord('S'):
            if not found:
                print("[警告] 未检测到棋盘格，无法保存")
                continue
            if saved_count >= max_images:
                print(f"[警告] 已达最大采集数量 ({max_images})")
                continue

            # ---- 求解棋盘格相对相机位姿（供打印，同时校验 solvePnP）----
            ok_pnp, pose_cam, dist_mm = get_board_pose_cam(obj_points, corners,
                                                           camera_matrix,
                                                           dist_coeffs)
            if not ok_pnp:
                print("[警告] solvePnP 失败，无法解算棋盘格位姿，本次不保存")
                continue

            # ---- 保存原始图片（不带角点标注，保持画面干净）----
            ts = int(time.time() * 1000)
            img_name = f"cal_{ts:016d}.png"
            img_path = os.path.join(image_dir, img_name)
            cv2.imwrite(img_path, frame)
            print(f"  [保存] 图片: {img_name}")

            # ---- 打印棋盘格相对相机位姿 ----
            bx, by, bz, brx, bry, brz = pose_cam
            print(f"  [棋盘格->相机] x={bx:.1f}  y={by:.1f}  z={bz:.1f} mm  "
                  f"rx={brx:.3f}  ry={bry:.3f}  rz={brz:.3f} rad  "
                  f"距离={dist_mm:.1f} mm")

            # ---- 保存机械臂位姿 ----
            ret, T_pose = get_robot_pose(sock, Coord(coord_sys))
            if ret != SUCCESS or T_pose is None:
                print(f"  [ERROR] 获取机械臂位姿失败 (ret={ret})")
                continue

            saved_poses.append(T_pose)
            saved_images.append(img_name)
            saved_count += 1
            x, y, z, rx, ry, rz = matrix_to_rpy(T_pose)
            print(f"  [保存] 机械臂末端位姿: x={x:.1f}  y={y:.1f}  z={z:.1f}  "
                  f"rx={rx:.3f}  ry={ry:.3f}  rz={rz:.3f}")
            print(f"  [信息] 已采集 {saved_count}/{min_images}")

            # ---- 写入 CSV（实时追加） ----
            csv_path = os.path.join(output_dir, "poses.csv")
            write_header = not os.path.isfile(csv_path)
            with open(csv_path, "a", newline="") as f:
                writer = csv.writer(f)
                if write_header:
                    writer.writerow([
                        "image", "x", "y", "z", "rx", "ry", "rz",
                        "qx", "qy", "qz", "qw"
                    ])
                qx, qy, qz, qw = matrix_to_quat(T_pose)[3:]
                writer.writerow([img_name, f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                                 f"{rx:.6f}", f"{ry:.6f}", f"{rz:.6f}",
                                 f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}", f"{qw:.6f}"])

        elif key == ord('c') or key == ord('C'):
            if saved_count < min_images:
                print(f"[警告] 数据不足，需要至少 {min_images} 张，当前 {saved_count}")
                continue
            calibrating = True
            # ---- 执行标定（在主线程中运行）----
            try:
                run_calibration(cfg, output_dir, obj_points,
                                saved_images, saved_poses,
                                camera_matrix, dist_coeffs)
            except Exception as e:
                print(f"[ERROR] 标定失败: {e}")
                import traceback
                traceback.print_exc()
            calibrating = False
            print("[信息] 标定完成，程序退出")
            break

    # ---- 清理 ----
    pipeline.stop()
    cv2.destroyAllWindows()
    # 使能下电
    disable_servo(sock)
    disconnect_robot(sock)
    print("[信息] 程序已退出")


# ============================================================
# 标定计算模块
# ============================================================

def run_calibration(cfg, output_dir, obj_points,
                    saved_images, saved_poses,
                    camera_matrix, dist_coeffs):
    """
    执行手眼标定计算

    @param cfg: 完整配置字典
    @param output_dir: cal_data 输出目录
    @param obj_points: 棋盘格世界坐标 (N×3)
    @param saved_images: 图片文件名列表
    @param saved_poses: 机械臂位姿列表 (4×4 矩阵)
    @param camera_matrix: 相机内参矩阵 (3×3)
    @param dist_coeffs: 畸变系数
    """
    board_cfg = cfg["calibration_board"]
    pattern_size = tuple(board_cfg["pattern_size"])
    square_size = board_cfg["square_size_mm"]
    image_dir = os.path.join(output_dir, "images")

    print(f"\n{'='*60}")
    print("  手眼标定计算")
    print(f"{'='*60}")
    print(f"  图像数量: {len(saved_images)}")
    print(f"  棋盘格内角点: {pattern_size[0]}×{pattern_size[1]}")
    print(f"  方格边长: {square_size} mm")
    print()

    # ---- 读取所有图片，检测角点 ----
    img_points = []   # 图像角点
    valid_indices = []

    for i, img_name in enumerate(saved_images):
        img_path = os.path.join(image_dir, img_name)
        img = cv2.imread(img_path)
        if img is None:
            print(f"  [跳过] 无法读取: {img_name}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = detect_checkerboard(gray, pattern_size)
        if not found:
            print(f"  [跳过] 未检测到棋盘格: {img_name}")
            continue
        img_points.append(corners)
        valid_indices.append(i)

    if len(img_points) < 4:
        print("[ERROR] 有效图像不足 4 张，无法标定")
        return

    print(f"  有效图像: {len(img_points)}/{len(saved_images)}")

    # ---- 构建 B 矩阵: 标定板在相机下的位姿 (solvePnP) ----
    # 对每张有效图片调用 solvePnP
    rvecs = []
    tvecs = []
    reproj_errors = []

    for corners in img_points:
        ret, rvec, tvec = cv2.solvePnP(obj_points, corners,
                                        camera_matrix, dist_coeffs)
        if not ret:
            print("  [警告] solvePnP 失败")
            continue
        # 重投影误差
        proj, _ = cv2.projectPoints(obj_points, rvec, tvec,
                                     camera_matrix, dist_coeffs)
        err = np.mean(np.sqrt(np.sum((corners.reshape(-1, 2) - proj.reshape(-1, 2))**2, axis=1)))
        reproj_errors.append(err)

        rvecs.append(rvec)
        tvecs.append(tvec)

    avg_reproj = np.mean(reproj_errors)
    print(f"\n  solvePnP 平均重投影误差: {avg_reproj:.4f} px")
    print(f"  各帧误差: {', '.join(f'{e:.3f}' for e in reproj_errors)}")

    if len(rvecs) < 4:
        print("[ERROR] solvePnP 有效数据不足 4 组")
        return

    # ---- 构建 A 矩阵: 机器人末端在基座下的位姿 ----
    n = len(valid_indices)
    R_gripper2base = []
    t_gripper2base = []

    for idx in valid_indices[:n]:
        T = saved_poses[idx]
        R_gripper2base.append(T[:3, :3])
        t_gripper2base.append(T[:3, 3])

    # ---- 构建 B 矩阵: 标定板在相机下的位姿 ----
    R_target2cam = []
    t_target2cam = []

    for rvec, tvec in zip(rvecs[:n], tvecs[:n]):
        R, _ = cv2.Rodrigues(rvec)
        R_target2cam.append(R)
        t_target2cam.append(tvec.flatten())

    # ---- 手眼标定: AX = XB ----
    print("\n  正在执行手眼标定 (Park 方法)...")
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_PARK
    )

    # 相机在末端下的位姿 X (待求)
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

    # 末端在相机下的位姿 (逆)
    T_gripper2cam = np.linalg.inv(T_cam2gripper)

    print(f"\n  标定完成!")
    print(f"{'='*60}")
    print(f"  相机在机械臂末端下的位姿 (T_cam2gripper):")
    x, y, z, rx, ry, rz = matrix_to_rpy(T_cam2gripper)
    print(f"    平移: x={x:.3f} mm, y={y:.3f} mm, z={z:.3f} mm")
    print(f"    旋转: rx={rx:.6f} rad, ry={ry:.6f} rad, rz={rz:.6f} rad")

    # ---- 验证: 反算标定板在基座下的位姿 ----
    # 对每一组有效数据:
    #   T_board2base = T_gripper2base @ T_cam2gripper @ T_board2cam
    # 其中:
    #   T_gripper2base = A (机器人末端在基座下)
    #   T_cam2gripper  = X (相机在末端下, 标定结果)
    #   T_board2cam    = B (标定板在相机下, solvePnP)
    # 公式推导: T_board2base = T_cam2base @ T_board2cam
    #                   = (T_gripper2base @ T_cam2gripper) @ T_board2cam
    #                   = A @ X @ B
    print(f"\n{'='*60}")
    print("  验证: 标定板在机械臂基座下的位姿")
    print(f"{'='*60}")

    board_poses_base = []
    for i in range(n):
        T_g2b = saved_poses[valid_indices[i]]                   # A: 末端在基座下
        T_b2cam = np.eye(4)
        T_b2cam[:3, :3] = R_target2cam[i]
        T_b2cam[:3, 3] = t_target2cam[i]                        # B: 标定板在相机下

        # T_board2base = A @ X @ B
        T_board2base = T_g2b @ T_cam2gripper @ T_b2cam
        board_poses_base.append(T_board2base)

    # 计算标定板在基座下的均值与标准差
    positions = np.array([T[:3, 3] for T in board_poses_base])
    pos_mean = np.mean(positions, axis=0)
    pos_std = np.std(positions, axis=0)

    print(f"\n  标定板在基座下的位置 (均值 ± 标准差):")
    print(f"    X: {pos_mean[0]:.2f} ± {pos_std[0]:.2f} mm")
    print(f"    Y: {pos_mean[1]:.2f} ± {pos_std[1]:.2f} mm")
    print(f"    Z: {pos_mean[2]:.2f} ± {pos_std[2]:.2f} mm")
    print(f"\n  位置一致性 (标准差均值越小越稳定): {np.mean(pos_std):.3f} mm")

    # ---- 写出结果到 YAML ----
    result = {
        "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "camera_intrinsics": {
            "fx": float(camera_matrix[0, 0]),
            "fy": float(camera_matrix[1, 1]),
            "cx": float(camera_matrix[0, 2]),
            "cy": float(camera_matrix[1, 2]),
        },
        "distortion_coefficients": dist_coeffs.tolist(),
        "handeye_result": {
            "T_cam2gripper": {
                "translation_mm": {
                    "x": float(T_cam2gripper[0, 3]),
                    "y": float(T_cam2gripper[1, 3]),
                    "z": float(T_cam2gripper[2, 3]),
                },
                "rotation_rpy_rad": {
                    "rx": float(rx),
                    "ry": float(ry),
                    "rz": float(rz),
                },
                "rotation_matrix": T_cam2gripper[:3, :3].tolist(),
            },
            "T_gripper2cam": {
                "translation_mm": {
                    "x": float(T_gripper2cam[0, 3]),
                    "y": float(T_gripper2cam[1, 3]),
                    "z": float(T_gripper2cam[2, 3]),
                },
                "rotation_matrix": T_gripper2cam[:3, :3].tolist(),
            },
        },
        "verification": {
            "num_samples": n,
            "avg_solvePnP_reprojection_error_px": float(avg_reproj),
            "board_position_base_frame": {
                "mean_mm": {"x": float(pos_mean[0]), "y": float(pos_mean[1]), "z": float(pos_mean[2])},
                "std_mm":  {"x": float(pos_std[0]), "y": float(pos_std[1]), "z": float(pos_std[2])},
                "consistency_mm": float(np.mean(pos_std)),
            },
        },
        "configuration": {
            "checkerboard": {
                "pattern_size": list(pattern_size),
                "square_size_mm": square_size,
            },
            "robot_coord_system": cfg["robot"]["coord_system"],
            "rotation_order": cfg["robot"]["rotation_order"],
        }
    }

    result_path = os.path.join(output_dir, "result.yaml")
    with open(result_path, "w", encoding="utf-8") as f:
        yaml.dump(result, f, default_flow_style=False, allow_unicode=True,
                  sort_keys=False)
    print(f"\n[信息] 标定结果已保存: {result_path}")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
