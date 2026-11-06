#!/usr/bin/env python3
#
# ex_handeye_verify.py
# @brief 基于 Intel D435i 的眼在手（Eye-in-Hand）手眼标定验证程序
#
# 功能:
#   识别棋盘格 → 解算棋盘格中心在基座下的位姿 → 到达目标点
#   （目标点 = 棋盘格中心沿标定板平面法线方向向外偏移 offset_along_normal_mm）
#   用于验证 ex_handeye_calibration.py 的标定结果是否有效。
#
# 位姿推导（与标定程序一致）:
#   A = T_gripper2base   当前机械臂末端在基座下的位姿（get_current_position）
#   B = T_board2cam      标定板在相机下的位姿（solvePnP）
#   X = T_cam2gripper    相机在末端下的位姿（标定结果，result.yaml）
#   T_board2base = A · X · B
#   目标位置 = T_board2base · [中心x, 中心y, offset_along_normal_mm, 1]
#   目标姿态 = R_board2base · diag(1, -1, -1)  →  工具 Z 轴垂直板面朝下，正对棋盘格中心
#
# @attention
#   - 需先运行 ex_handeye_calibration.py 完成标定，生成 cal_data/result.yaml
#   - 需要连接 Intel D435i 相机，棋盘格需在视野内清晰可见
#   - 验证目标点的偏移量等参数在 config/cal_config.yaml 的 verify 段中配置
#   - 默认控制器 IP 为 192.168.1.13，可在 cal_config.yaml 中修改
#
# @note 运行步骤
#       安装依赖: pip install pyrealsense2 opencv-contrib-python numpy pyyaml
#       运行: python3 ex_handeye_verify.py
#
# @note 操作键
#   S — 识别棋盘格并解算目标位姿，打印后询问是否前往
#   Y — 确认后机械臂直线运动到目标位姿（仅在按 S 之后有效）
#   R — 机械臂返回零位，等待下一次识别任务
#   Q — 退出程序
#

import sys
import os
import time
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
    # 实测 pos[0..5] = x, y, z, rx, ry, rz
    T = robot_xyz_to_matrix(pos[0], pos[1], pos[2],
                            pos[3], pos[4], pos[5])
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


# ============================================================
# 手眼标定结果加载
# ============================================================

def load_handeye_result(result_path):
    """
    从 result.yaml 加载相机在机械臂末端下的位姿 X = T_cam2gripper

    @param result_path: result.yaml 路径
    @return: T_cam2gripper (4×4 numpy 矩阵)
    """
    with open(result_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    he = data["handeye_result"]["T_cam2gripper"]
    R = np.array(he["rotation_matrix"], dtype=np.float64)
    t = he["translation_mm"]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t["x"], t["y"], t["z"]]

    x, y, z, rx, ry, rz = matrix_to_rpy(T)
    print("  相机在末端下的位姿 (T_cam2gripper):")
    print(f"    平移: x={x:.3f} mm, y={y:.3f} mm, z={z:.3f} mm")
    print(f"    旋转: rx={rx:.6f} rad, ry={ry:.6f} rad, rz={rz:.6f} rad")
    return T


# ============================================================
# 目标位姿解算
# ============================================================

def compute_target_pose(T_gripper2base, T_cam2gripper, rvec, tvec,
                        pattern_size, square_size, offset_mm):
    """
    解算验证目标位姿

    @param T_gripper2base: 当前机械臂末端在基座下的位姿 A (4×4)
    @param T_cam2gripper:  相机在末端下的位姿 X (4×4)，标定结果
    @param rvec, tvec:     solvePnP 输出的标定板在相机下的位姿 B
    @param pattern_size:   棋盘格内角点数 (cols, rows)
    @param square_size:    方格边长 mm
    @param offset_mm:      目标点沿标定板法线方向的偏移量 mm（可调参数）
    @return: (x, y, z, rx, ry, rz) 基坐标目标位姿
    """
    # ---- B: 标定板在相机下的位姿 ----
    R_board2cam, _ = cv2.Rodrigues(rvec)
    T_board2cam = np.eye(4)
    T_board2cam[:3, :3] = R_board2cam
    T_board2cam[:3, 3] = tvec.flatten()

    # ---- 标定板在基座下的位姿: T_board2base = A · X · B ----
    T_board2base = T_gripper2base @ T_cam2gripper @ T_board2cam

    # ---- 棋盘格中心内角点（标定板坐标系下，Z=0 平面）----
    center_x = (pattern_size[0] - 1) / 2.0 * square_size
    center_y = (pattern_size[1] - 1) / 2.0 * square_size

    # ---- 目标位置: 中心沿标定板法线（板坐标 +Z 方向）向外偏移 offset_mm ----
    p_target = T_board2base @ np.array([center_x, center_y, offset_mm, 1.0])

    # ---- 目标姿态: 工具 Z 轴垂直板面朝下，正对棋盘格中心 ----
    # 注意: 本 SDK 中 solvePnP 的板坐标系 Z 轴沿相机光轴方向（指向相机前方）。
    # 相机朝下观测水平板时，板 Z 轴即朝下指向板面，因此目标工具姿态直接采用板姿态，
    # 使工具 Z 轴朝下指向棋盘格中心（实测板法线 nz≈-1、目标工具 Z 应≈(0,0,-1)）。
    R_board2base = T_board2base[:3, :3]
    R_target = R_board2base

    T_target = np.eye(4)
    T_target[:3, :3] = R_target
    T_target[:3, 3] = p_target[:3]

    return matrix_to_rpy(T_target)


# ============================================================
# 画面坐标系绘制
# ============================================================

def draw_coord_axes(display, rvec, tvec, camera_matrix, dist_coeffs,
                    pattern_size, square_size, offset_mm, axis_len=40.0):
    """
    在画面中叠加绘制坐标系，直观显示目标位置与姿态方向

    - 目标坐标系（红X / 绿Y / 蓝Z）: 原点 = 棋盘格中心沿标定板法线偏移 offset_mm 处；
      工具 Z 垂直板面朝下（与板 Z 反向），X 与板 X 同向，Y 与板 Y 反向
    - 棋盘格坐标系（灰色）: 原点 = 棋盘格中心，Z 沿板法线

    @param display     显示图像（就地绘制）
    @param rvec, tvec  solvePnP 输出（标定板在相机下的位姿）
    """
    R, _ = cv2.Rodrigues(rvec)
    T_board2cam = np.eye(4)
    T_board2cam[:3, :3] = R
    T_board2cam[:3, 3] = tvec.flatten()

    cx = (pattern_size[0] - 1) / 2.0 * square_size
    cy = (pattern_size[1] - 1) / 2.0 * square_size

    def project(px, py, pz):
        """标定板坐标系 3D 点 → 图像像素坐标"""
        p_cam = T_board2cam @ np.array([px, py, pz, 1.0])
        pts = np.array([[[p_cam[0], p_cam[1], p_cam[2]]]], dtype=np.float64)
        img_pts, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3),
                                       camera_matrix, dist_coeffs)
        return int(round(img_pts[0][0][0])), int(round(img_pts[0][0][1]))

    axis_colors = {"X": (0, 0, 255), "Y": (0, 255, 0), "Z": (255, 0, 0)}

    # ---- 目标坐标系（红X / 绿Y / 蓝Z）----
    # 目标姿态 = 板姿态（工具 Z 轴朝下指向板面），板坐标下轴方向:
    #   X=(1,0,0)  Y=(0,1,0)  Z=(0,0,1)
    t_origin = project(cx, cy, offset_mm)
    cv2.circle(display, t_origin, 6, (255, 255, 255), -1)
    cv2.putText(display, "Target", (t_origin[0] + 8, t_origin[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    for name, (dx, dy, dz) in {"X": (1, 0, 0),
                               "Y": (0, 1, 0),
                               "Z": (0, 0, 1)}.items():
        end = project(cx + dx * axis_len, cy + dy * axis_len,
                      offset_mm + dz * axis_len)
        cv2.line(display, t_origin, end, axis_colors[name], 2)
        cv2.putText(display, name, (end[0] + 3, end[1] - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, axis_colors[name], 1)

    # ---- 棋盘格坐标系（灰色，仅画轴不带字母）----
    g_origin = project(cx, cy, 0.0)
    cv2.circle(display, g_origin, 4, (200, 200, 200), -1)
    for dx, dy, dz in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        end = project(cx + dx * 25.0, cy + dy * 25.0, dz * 25.0)
        cv2.line(display, g_origin, end, (120, 120, 120), 1)


# ============================================================
# 运动控制辅助
# ============================================================

def wait_motion_done(socket_fd, timeout_sec):
    """轮询机器人运行状态等待运动完成

    @param socket_fd TCP socket
    @param timeout_sec 超时秒数
    @return True 运动完成 / False 超时
    """
    deadline = time.time() + timeout_sec
    last_print = 0
    time.sleep(0.1)
    while time.time() < deadline:
        ret, state = get_robot_running_state(socket_fd, -1)
        if ret == SUCCESS and state == 0:
            print("  [信息] 运动完成")
            return True
        time.sleep(0.2)
        elapsed = int(time.time() - (deadline - timeout_sec))
        if elapsed - last_print >= 5:
            print(f"  [信息] 等待运动中... (已过 {elapsed}s)")
            last_print = elapsed
    print(f"  [WARN] 等待运动超时 ({timeout_sec}s)")
    return False


def move_to_target(socket_fd, target, ver_cfg):
    """机械臂直线运动到目标位姿（基坐标系 movel）

    @param target: (x, y, z, rx, ry, rz)
    @param ver_cfg: verify 配置段
    @return: ret 返回码
    """
    x, y, z, rx, ry, rz = target

    # ---- 检查伺服状态（运动前确认已上电运行）----
    state = -1
    ret, state = get_servo_state(socket_fd, state)
    if ret != SUCCESS or state != 3:
        print(f"  [ERROR] 伺服未处于运行状态 (state={state})，无法运动")
        return -1
    print(f"  [信息] 伺服状态: {state} (3=运行)")

    cmd = MoveCmd()
    cmd.targetPosType = PosType_data
    cmd.targetPosName = ""
    cmd.coord = Coord.BASE              # 基坐标系
    cmd.velocity = ver_cfg["move_velocity"]
    cmd.acc = ver_cfg["move_acc"]
    cmd.dec = ver_cfg["move_dec"]
    cmd.pl = 0
    cmd.targetPosValue = VectorDouble([x, y, z, rx, ry, rz])
    print(f"  [运动] 前往目标位姿: x={x:.1f}  y={y:.1f}  z={z:.1f}  "
          f"rx={rx:.4f}  ry={ry:.4f}  rz={rz:.4f}")
    ret = robot_movel(socket_fd, cmd)
    if ret != SUCCESS:
        print(f"  [ERROR] MoveL 运动失败，错误码: {ret}")
        return ret
    wait_motion_done(socket_fd, ver_cfg["motion_timeout_s"])

    # ---- 到位后读取实际位姿，打印与目标的偏差（用于验证标定精度）----
    _, T_actual = get_robot_pose(socket_fd, Coord.BASE)
    if T_actual is not None:
        ax, ay, az, arx, ary, arz = matrix_to_rpy(T_actual)
        pos_err = np.linalg.norm(np.array([x - ax, y - ay, z - az]))
        print(f"  [到位] 实际位姿: x={ax:.2f}  y={ay:.2f}  z={az:.2f}  "
              f"rx={arx:.4f}  ry={ary:.4f}  rz={arz:.4f}")
        print(f"  [偏差] 目标与实际位置偏差: {pos_err:.2f} mm（越小说明标定越准）")
    return ret


def move_home(socket_fd, home_joints, ver_cfg):
    """机械臂返回零位（关节坐标系 movej）

    @param home_joints: 零位关节角列表（度）
    @param ver_cfg: verify 配置段
    @return: ret 返回码
    """
    cmd = MoveCmd()
    cmd.targetPosType = PosType_data
    cmd.targetPosName = ""
    cmd.coord = Coord.JOINT                  # 关节坐标系
    cmd.velocity = 40
    cmd.acc = 20
    cmd.dec = 20
    cmd.pl = 0
    cmd.targetPosValue = VectorDouble(list(home_joints))
    print(f"  [运动] 返回零位: {[f'{j:.1f}' for j in home_joints]}")
    ret = robot_movej(socket_fd, cmd)
    if ret != SUCCESS:
        print(f"  [ERROR] MoveJ 返回零位失败，错误码: {ret}")
        return ret
    wait_motion_done(socket_fd, ver_cfg["motion_timeout_s"])
    return ret


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
    # 命令行参数优先于 cal_config.yaml: python3 ex_handeye_verify.py [-ip 192.168.1.13]
    if len(sys.argv) >= 3 and sys.argv[1] in ("-ip", "--ip"):
        robot_cfg["ip"] = sys.argv[2]
    ver_cfg = cfg["verify"]
    board_cfg = cfg["calibration_board"]

    pattern_size = tuple(board_cfg["pattern_size"])   # (cols, rows)
    square_size = board_cfg["square_size_mm"]
    coord_sys = robot_cfg["coord_system"]
    if coord_sys not in (1, 2):
        print(f"[ERROR] 配置的位姿坐标系不合法: coord_system={coord_sys}（支持 1=基坐标 / 2=工具）")
        sys.exit(-1)

    # ★★★ 验证目标点偏移量 —— 需要调整时修改 config/cal_config.yaml ★★★
    offset_mm = ver_cfg["offset_along_normal_mm"]
    print(f"[信息] 验证目标点偏移量: 沿标定板法线方向向外 {offset_mm} mm")
    print(f"      （如需调整，修改 cal_config.yaml → verify → offset_along_normal_mm）")

    # ---- 加载标定结果 ----
    result_path = os.path.join(_script_dir, ver_cfg["result_path"])
    if not os.path.isfile(result_path):
        print(f"[ERROR] 标定结果不存在: {result_path}")
        print("       请先运行 ex_handeye_calibration.py 完成标定")
        return
    print("[信息] 加载标定结果:")
    T_cam2gripper = load_handeye_result(result_path)

    # ---- 连接机械臂 ----
    print(f"[信息] 连接控制器 {robot_cfg['ip']}:{robot_cfg['port']}...")
    sock = connect_robot(robot_cfg["ip"], robot_cfg["port"])
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        return
    print(f"[信息] 连接成功 (socket={sock})")

    # ---- 切换示教模式（运动前必须设置模式，参照官方运动示例 ex_move_control）----
    print("--- 切换示教模式 ---")
    ret = set_current_mode(sock, 0)
    if ret != SUCCESS:
        print(f"[ERROR] 切换示教模式失败: {ret}，程序退出!")
        disconnect_robot(sock)
        return
    print("[INFO] 已切换为示教模式")

    # ---- 使能上电 ----
    enable_servo(sock)

    # ---- 设置全局速度 ----
    ret = set_speed(sock, 30)
    if ret == SUCCESS:
        print("[信息] 已设置全局速度 30%")
    else:
        print(f"[警告] 设置全局速度失败: {ret}")
    time.sleep(0.2)

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

    # ---- 状态机 ----
    # IDLE    等待识别（按 S）
    # CONFIRM 已解算目标位姿，等待确认（按 Y 前往 / 按 R 放弃）
    # MOVING  运动中
    state = "IDLE"
    target_xyzrpy = None

    # ---- 主循环 ----
    print(f"\n{'='*60}")
    print("  手眼标定验证 — 操作说明")
    print(f"{'='*60}")
    print("  S — 识别棋盘格并解算目标位姿，询问是否前往")
    print("  Y — 确认后前往目标位姿（按 S 之后有效）")
    print("  R — 返回零位，等待下一次识别")
    print("  Q — 退出程序")
    print(f"{'='*60}\n")

    cv2.namedWindow("Hand-Eye Verify", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Hand-Eye Verify", 960, 720)

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

        # ---- 当前帧棋盘格位姿（用于叠加坐标系）----
        current_rvec = current_tvec = None
        if found:
            ret_pnp, crv, ctv = cv2.solvePnP(obj_points, corners,
                                             camera_matrix, dist_coeffs)
            if ret_pnp:
                current_rvec, current_tvec = crv, ctv

        # ---- 叠加棋盘格坐标系 + 目标坐标系（直观显示目标位置与姿态）----
        if current_rvec is not None:
            draw_coord_axes(display, current_rvec, current_tvec,
                            camera_matrix, dist_coeffs,
                            pattern_size, square_size, offset_mm)

        # ---- 状态提示文字（英文，避免 OpenCV 中文乱码）----
        if state == "IDLE":
            status_text = "IDLE | Board: " + ("OK" if found else "No board") + \
                          " | Press S to solve"
            color = (0, 255, 0)
        elif state == "CONFIRM":
            status_text = "CONFIRM | Press Y to move / R to home"
            color = (0, 255, 255)
            # 叠加目标位姿数值（基座基坐标）
            if target_xyzrpy is not None:
                tx, ty, tz, trx, try_, trz = target_xyzrpy
                cv2.putText(display,
                            f"TARGET X={tx:.1f} Y={ty:.1f} Z={tz:.1f} mm",
                            (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 0), 2)
                cv2.putText(display,
                            f"RX={trx:.3f} RY={try_:.3f} RZ={trz:.3f} rad",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 0), 2)
        else:  # MOVING
            status_text = "MOVING | Moving to target..."
            color = (255, 128, 0)
        cv2.putText(display, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        # 底部坐标系图例
        cv2.putText(display,
                    "Frame axes: X=red Y=green Z=blue | White dot=Target origin",
                    (10, display.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Hand-Eye Verify", display)

        # ---- 键盘事件 ----
        key = cv2.waitKey(30) & 0xFF

        # Q: 退出
        if key == ord('q') or key == ord('Q'):
            print("[信息] 用户退出")
            running = False
            break

        # S: 识别 + 解算目标位姿（仅在 IDLE 状态）
        elif (key == ord('s') or key == ord('S')) and state == "IDLE":
            if not found:
                print("[警告] 未检测到棋盘格，无法解算目标位姿")
                continue

            # 读取当前机械臂末端位姿 A
            ret, T_g2b = get_robot_pose(sock, Coord(coord_sys))
            if ret != SUCCESS or T_g2b is None:
                print(f"[ERROR] 获取机械臂位姿失败 (ret={ret})")
                continue

            # solvePnP 求标定板在相机下的位姿 B
            ret_pnp, rvec, tvec = cv2.solvePnP(obj_points, corners,
                                               camera_matrix, dist_coeffs)
            if not ret_pnp:
                print("[警告] solvePnP 失败，请调整棋盘格位置")
                continue

            # 解算目标位姿
            target_xyzrpy = compute_target_pose(T_g2b, T_cam2gripper,
                                                rvec, tvec,
                                                pattern_size, square_size,
                                                offset_mm)
            x, y, z, rx, ry, rz = target_xyzrpy

            # ---- 诊断: 板中心/板法线/目标工具Z（基座坐标系），用于确认姿态方向 ----
            R_b2cam_d, _ = cv2.Rodrigues(rvec)
            T_b2cam_d = np.eye(4)
            T_b2cam_d[:3, :3] = R_b2cam_d
            T_b2cam_d[:3, 3] = tvec.flatten()
            T_b2b_d = T_g2b @ T_cam2gripper @ T_b2cam_d
            cxx = (pattern_size[0] - 1) / 2.0 * square_size
            cyy = (pattern_size[1] - 1) / 2.0 * square_size
            center_base = (T_b2b_d @ np.array([cxx, cyy, 0.0, 1.0]))[:3]
            normal_base = T_b2b_d[:3, 2]
            tool_z = -normal_base
            print(f"\n[诊断] 板中心(基座): x={center_base[0]:.1f}  "
                  f"y={center_base[1]:.1f}  z={center_base[2]:.1f} mm")
            print(f"[诊断] 板法线(基座): nx={normal_base[0]:+.2f}  "
                  f"ny={normal_base[1]:+.2f}  nz={normal_base[2]:+.2f}  "
                  f"(相机朝下看水平板时应≈(0,0,1)朝上)")
            print(f"[诊断] 目标工具Z(基座): zx={tool_z[0]:+.2f}  "
                  f"zy={tool_z[1]:+.2f}  zz={tool_z[2]:+.2f}  "
                  f"(应≈(0,0,-1)朝下指向板)")

            print(f"\n[信息] 已解算目标位姿（棋盘格中心沿法线方向外偏移 {offset_mm} mm）:")
            print(f"  [目标] x={x:.2f}  y={y:.2f}  z={z:.2f}  "
                  f"rx={rx:.4f}  ry={ry:.4f}  rz={rz:.4f}")
            print("  [询问] 是否前往目标位姿？按 Y 确认前往，按 R 放弃并返回零位")
            state = "CONFIRM"

        # Y: 确认前往目标位姿（仅在 CONFIRM 状态）
        elif (key == ord('y') or key == ord('Y')) and state == "CONFIRM":
            if target_xyzrpy is None:
                continue
            state = "MOVING"
            move_to_target(sock, target_xyzrpy, ver_cfg)
            target_xyzrpy = None
            state = "IDLE"
            print("[信息] 已到达目标位姿。按 R 返回零位，或按 S 进行下一次识别\n")

        # R: 返回零位（IDLE / CONFIRM 状态均可）
        elif key == ord('r') or key == ord('R'):
            state = "MOVING"
            move_home(sock, ver_cfg["home_joints"], ver_cfg)
            target_xyzrpy = None
            state = "IDLE"
            print("[信息] 已返回零位，等待下一次识别（按 S）\n")

    # ---- 清理 ----
    pipeline.stop()
    cv2.destroyAllWindows()
    # 使能下电
    disable_servo(sock)
    disconnect_robot(sock)
    print("[信息] 程序已退出")


if __name__ == "__main__":
    main()
