#!/usr/bin/env python3
"""Read-only trajectory tracking monitor; end pose is derived from URDF FK."""

import argparse
import csv
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError("Joint axis must be nonzero")
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


class ForwardKinematics:
    def __init__(self, path, base, tip):
        root = ET.parse(path).getroot()
        links = {link.attrib["name"] for link in root.findall("link")}
        if base not in links or tip not in links:
            raise ValueError(f"Unknown base/tip: {base}, {tip}")
        joints = {j.find("child").attrib["link"]: j for j in root.findall("joint")}
        self.chain = []
        visited = set()
        while tip != base:
            if tip in visited or tip not in joints:
                raise ValueError("No valid chain from base to tip")
            visited.add(tip)
            joint = joints[tip]
            kind = joint.attrib["type"]
            if kind not in ("fixed", "revolute", "continuous", "prismatic"):
                raise ValueError(f"Unsupported joint type: {kind}")
            if joint.find("mimic") is not None:
                raise ValueError("Mimic joints are not supported")
            origin = joint.find("origin")
            attrs = {} if origin is None else origin.attrib
            xyz = np.array([float(v) for v in attrs.get("xyz", "0 0 0").split()])
            roll, pitch, yaw = map(float, attrs.get("rpy", "0 0 0").split())
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (
                rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
            )
            axis_node = joint.find("axis")
            axis = (
                [1.0, 0.0, 0.0]
                if axis_node is None
                else [float(v) for v in axis_node.attrib["xyz"].split()]
            )
            self.chain.append((joint.attrib["name"], kind, transform, axis))
            tip = joint.find("parent").attrib["link"]
        self.chain.reverse()

    def pose(self, positions):
        result = np.eye(4)
        for name, kind, origin, axis in self.chain:
            motion = np.eye(4)
            if kind in ("revolute", "continuous"):
                motion[:3, :3] = rotation(axis, positions[name])
            elif kind == "prismatic":
                vector = np.asarray(axis)
                motion[:3, 3] = vector / np.linalg.norm(vector) * positions[name]
            result = result @ origin @ motion
        return result


def pose_error(desired, actual):
    delta_mm = (desired[:3, 3] - actual[:3, 3]) * 1000
    relative = desired[:3, :3] @ actual[:3, :3].T
    angle_deg = np.degrees(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
    return np.r_[delta_mm, np.linalg.norm(delta_mm), angle_deg]


class TrackingMonitor(Node):
    def __init__(self, args, fk):
        super().__init__("tracking_monitor")
        self.args, self.fk = args, fk
        self.names = None
        self.count = 0
        self.last_received = None
        self.last_stamp = None
        self.start = time.monotonic()
        self.history = deque(maxlen=10000)
        self.output = Path(args.output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        # Refuse to silently overwrite an earlier measurement.
        self.stream = self.output.open("x", newline="")
        self.writer = csv.writer(self.stream)
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState, args.topic, self.receive, qos_profile_sensor_data
        )
        self.timer = self.create_timer(2.0, self.report)
        self.get_logger().info(f"Listening on {args.topic}; CSV: {self.output}")

    def receive(self, msg):
        desired = getattr(msg, "reference", None)
        actual = getattr(msg, "feedback", None)
        if desired is None or not desired.positions:
            desired = msg.desired
        if actual is None or not actual.positions:
            actual = msg.actual
        names = list(msg.joint_names)
        qd, qa = np.asarray(desired.positions), np.asarray(actual.positions)
        if (
            not names
            or len(set(names)) != len(names)
            or len(qd) != len(names)
            or len(qa) != len(names)
            or not np.isfinite(np.r_[qd, qa]).all()
        ):
            self.get_logger().warning(
                "Skipping invalid/incomplete controller state", throttle_duration_sec=5
            )
            return
        if self.names is not None and set(names) != set(self.names):
            self.get_logger().warning("Joint set changed; restart monitor", throttle_duration_sec=5)
            return
        try:
            target = self.fk.pose(dict(zip(names, qd)))
            feedback = self.fk.pose(dict(zip(names, qa)))
        except KeyError as exc:
            self.get_logger().warning(
                f"Missing URDF joint in feedback: {exc}", throttle_duration_sec=5
            )
            return
        if self.names is None:
            if any(kind == "prismatic" for _, kind, _, _ in self.fk.chain):
                raise ValueError("Degree plots require revolute/fixed arm joints")
            self.names = names
            self.labels = [f"{n}_error_deg" for n in names] + [
                "fk_dx_mm",
                "fk_dy_mm",
                "fk_dz_mm",
                "fk_position_error_mm",
                "fk_orientation_error_deg",
            ]
            self.writer.writerow(
                ["stamp_sec", "elapsed_sec"]
                + [f"{n}_desired_deg" for n in names]
                + [f"{n}_actual_deg" for n in names]
                + self.labels
            )
            self.sum_sq = np.zeros(len(self.labels))
            self.maximum = np.zeros(len(self.labels))
        order = [names.index(n) for n in self.names]
        qd, qa = np.degrees(qd[order]), np.degrees(qa[order])
        errors = np.r_[qd - qa, pose_error(target, feedback)]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.get_logger().warning(
                "Skipping duplicate/backward timestamp; restart after clock reset",
                throttle_duration_sec=5,
            )
            return
        self.last_stamp = stamp
        now = time.monotonic()
        elapsed = now - self.start
        self.last_received = now
        self.count += 1
        self.sum_sq += errors**2
        self.maximum = np.maximum(self.maximum, np.abs(errors))
        self.writer.writerow([f"{stamp:.9f}", elapsed, *qd, *qa, *errors])
        self.history.append((elapsed, qd, qa, errors))
        while self.history and elapsed - self.history[0][0] > self.args.window:
            self.history.popleft()

    def report(self):
        self.stream.flush()
        if self.last_received is None or time.monotonic() - self.last_received > 2:
            self.get_logger().warning(
                "No fresh controller state; check topic, controller and ROS domain"
            )
        else:
            error = self.history[-1][3]
            self.get_logger().info(
                f"samples={self.count}, joint max={np.max(np.abs(error[:len(self.names)])):.3f} deg, "
                f"FK position={error[-2]:.3f} mm, orientation={error[-1]:.3f} deg"
            )

    def finish(self):
        self.stream.close()
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        # Use a separate, non-GUI figure: saving also works after closing the live window.
        image_path = self.output.with_suffix(self.output.suffix + ".png")
        fig = Figure(figsize=(15, 12), layout="constrained")
        FigureCanvasAgg(fig)
        grid = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.2])
        axes = [fig.add_subplot(grid[row, col]) for row in range(2) for col in range(2)]
        stats = fig.add_subplot(grid[2, :])
        stats.axis("off")
        fig.suptitle(
            f"Tracking report: {self.count} samples | "
            f"{self.args.base_link} -> {self.args.tip_link}\n"
            "FK-derived end pose errors (not externally measured TCP accuracy)"
        )
        if self.count:
            # Read the entire recording, not just the rolling live-plot window.
            data = np.loadtxt(self.output, delimiter=",", skiprows=1, ndmin=2)
            count = len(self.names)
            times = data[:, 1]
            desired = data[:, 2 : 2 + count]
            actual = data[:, 2 + count : 2 + 2 * count]
            errors = data[:, 2 + 2 * count :]
            for i, name in enumerate(self.names):
                (line,) = axes[0].plot(times, actual[:, i], label=name)
                axes[0].plot(times, desired[:, i], "--", color=line.get_color())
                axes[1].plot(times, errors[:, i], label=name)
            axes[0].legend(ncol=4, fontsize=8)
            axes[1].legend(ncol=4, fontsize=8)
            axes[2].plot(times, errors[:, -2])
            axes[3].plot(times, errors[:, -1])
            table = stats.table(
                cellText=[
                    [name, f"{rms:.6f}", f"{peak:.6f}"]
                    for name, rms, peak in zip(
                        self.labels, np.sqrt(self.sum_sq / self.count), self.maximum
                    )
                ],
                colLabels=["Metric (unit in name)", "RMSE", "Max absolute error"],
                cellLoc="center",
                bbox=[0, 0.05, 1, 0.9],
            )
            table.auto_set_font_size(False)
            table.set_fontsize(9)
            stats.set_title("Full recording statistics (sample weighted, including idle periods)")
        else:
            stats.text(0.5, 0.5, "No valid controller samples received", ha="center")
        for ax, title in zip(
            axes,
            [
                "Joint angles (deg): actual solid / desired dashed",
                "Joint error: desired - actual (deg)",
                "FK position error norm (mm)",
                "FK orientation error (deg)",
            ],
        ):
            ax.set_title(title)
            ax.set_xlabel("Elapsed time (s)")
            ax.grid(True)
        # Exclusive creation avoids overwriting an existing report image.
        with image_path.open("xb") as stream:
            fig.savefig(stream, format="png", dpi=160)
        self.get_logger().info(
            f"Saved {self.count} samples; CSV: {self.output}; image: {image_path}"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/tcb_group_controller/controller_state")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "src/tl_description/urdf/tcb705l.urdf",
    )
    parser.add_argument("--base-link", default="link0")
    parser.add_argument("--tip-link", default="link7")
    parser.add_argument("--output", default=f"/tmp/tracking_{time.time_ns()}.csv")
    parser.add_argument("--duration", type=float, default=0, help="Seconds; 0 means until Ctrl+C")
    parser.add_argument("--window", type=float, default=20, help="Plot time window in seconds")
    parser.add_argument("--plot", action="store_true", help="Show live matplotlib plots")
    args = parser.parse_args()
    if not np.isfinite([args.duration, args.window]).all() or args.duration < 0 or args.window <= 0:
        parser.error("duration must be >= 0 and window must be > 0")
    fk = ForwardKinematics(args.urdf, args.base_link, args.tip_link)
    plt = None
    if args.plot:
        import matplotlib.pyplot as plt

        plt.ion()
        fig, axes = plt.subplots(2, 2, figsize=(13, 8))
        fig.suptitle("Joint tracking / FK-derived end pose errors")
    rclpy.init()
    monitor = None
    try:
        monitor = TrackingMonitor(args, fk)
        last_plot = 0
        while rclpy.ok():
            rclpy.spin_once(monitor, timeout_sec=0.05)
            now = time.monotonic()
            if args.duration and now - monitor.start >= args.duration:
                break
            if plt is not None:
                if not plt.fignum_exists(fig.number):
                    break
                if now - last_plot >= 0.2 and monitor.history:
                    last_plot = now
                    history = list(monitor.history)
                    times = [v[0] for v in history]
                    qd, qa, errors = [np.asarray([v[i] for v in history]) for i in (1, 2, 3)]
                    for ax in axes.flat:
                        ax.clear()
                        ax.grid(True)
                        ax.set_xlabel("Elapsed time (s)")
                    for i, name in enumerate(monitor.names):
                        (line,) = axes[0, 0].plot(times, qa[:, i], label=name)
                        axes[0, 0].plot(times, qd[:, i], "--", color=line.get_color())
                        axes[0, 1].plot(times, errors[:, i], label=name)
                    axes[0, 0].set_title("Angles (deg): actual solid / desired dashed")
                    axes[0, 1].set_title("Desired - actual (deg)")
                    axes[0, 0].legend(ncol=4, fontsize=8)
                    axes[0, 1].legend(ncol=4, fontsize=8)
                    axes[1, 0].plot(times, errors[:, -2])
                    axes[1, 0].set_title("FK position error norm (mm)")
                    axes[1, 1].plot(times, errors[:, -1])
                    axes[1, 1].set_title("FK orientation error (deg)")
                    fig.tight_layout()
                plt.pause(0.001)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if monitor is not None:
            monitor.finish()
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if plt is not None:
            plt.close("all")


if __name__ == "__main__":
    main()
