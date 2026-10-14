#!/usr/bin/env python3
"""Conservative Astral teleop smoke test — publishes joint_commands only.

Does NOT import astral_robot_sdk. Pair with astral_robot_control (or dry_run).

Usage:
  ros2 run astral_arm_teleop test_safe_teleop --ros-args \\
    -p arm_side:=left -p dry_run:=true -p solver_type:=urdf_numerical
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

from astral_arm_teleop.ik import make_ik_solver
from astral_arm_teleop.pose_processor import PoseProcessor
from astral_arm_teleop.safety_filter import SafetyFilter

try:
    from astral_robot_control.joint_layout import (
        LEFT_ARM_JOINT_NAMES,
        RIGHT_ARM_JOINT_NAMES,
    )
except ImportError:  # pragma: no cover
    LEFT_ARM_JOINT_NAMES = [f"left_j{i}" for i in range(7)]
    RIGHT_ARM_JOINT_NAMES = [f"right_j{i}" for i in range(7)]


def _qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class SafeTeleopTestNode(Node):
    def __init__(self) -> None:
        super().__init__("test_safe_teleop")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("solver_type", "analytic_dh")
        self.declare_parameter(
            "robot_world_to_base_rot",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("pos_smoothing", 0.6)
        self.declare_parameter("rot_smoothing", 0.8)
        self.declare_parameter("motion_scale", 0.5)
        self.declare_parameter("flip_pitch", False)
        self.declare_parameter("max_joint_vel", 2.0)
        self.declare_parameter("data_timeout", 1.5)
        self.declare_parameter("tcp_offset", [0.0] * 6)
        self.declare_parameter("dry_run", True)

        side = str(self.get_parameter("arm_side").value).lower()
        self.side = side
        self.arm_key = "L" if side == "left" else "R"
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.data_timeout = float(self.get_parameter("data_timeout").value)
        rate = float(self.get_parameter("control_rate").value)

        self.get_logger().warn("=" * 50)
        self.get_logger().warn("SAFE TELEOP TEST (joint_commands only)")
        self.get_logger().warn(f"  side={side} dry_run={self.dry_run}")
        self.get_logger().warn(
            f"  max_joint_vel={self.get_parameter('max_joint_vel').value} "
            f"scale={self.get_parameter('motion_scale').value}"
        )
        self.get_logger().warn("=" * 50)

        self.bridge = make_ik_solver(str(self.get_parameter("solver_type").value))
        lo, hi = self.bridge.joint_limits(self.arm_key)
        self.bridge.sync_state(np.zeros(14))
        T0 = self.bridge.fk(self.arm_key, np.zeros(7))
        _ = self.bridge.solve(self.arm_key, T0)
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()

        R = np.asarray(
            self.get_parameter("robot_world_to_base_rot").value, dtype=float
        ).reshape(3, 3)
        self.pose = PoseProcessor(
            robot_world_to_base_rot=R,
            pos_smoothing=float(self.get_parameter("pos_smoothing").value),
            rot_smoothing=float(self.get_parameter("rot_smoothing").value),
            motion_scale=float(self.get_parameter("motion_scale").value),
            flip_pitch=bool(self.get_parameter("flip_pitch").value),
        )
        tcp = list(self.get_parameter("tcp_offset").value)
        T_ft = np.eye(4)
        T_ft[:3, :3] = Rotation.from_euler("xyz", tcp[3:]).as_matrix()
        T_ft[:3, 3] = tcp[:3]
        self._T_tcp_to_flange = np.linalg.inv(T_ft)

        self.safety = SafetyFilter(
            joint_lower_limits=lo,
            joint_upper_limits=hi,
            max_joint_vel=float(self.get_parameter("max_joint_vel").value),
            workspace_radius=0.0,
        )
        self.safety.set_initial_state(np.zeros(7))

        self.names = (
            list(LEFT_ARM_JOINT_NAMES)
            if side == "left"
            else list(RIGHT_ARM_JOINT_NAMES)
        )
        self.pub = self.create_publisher(
            JointState, f"/{side}_arm/joint_commands", _qos()
        )
        self.create_subscription(
            PoseStamped, f"quest3/{side}_wrist_pose", self._vr_cb, 10
        )
        self.last_vr = 0.0
        self.vr_ok = False
        self._prev = time.monotonic()
        self.create_timer(1.0 / max(1.0, rate), self._loop)

    def _vr_cb(self, msg: PoseStamped) -> None:
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        quat = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        n = float(np.linalg.norm(quat))
        if n < 1e-9:
            return
        self.pose.update_vr_pose(pos, Rotation.from_quat(quat / n))
        self.last_vr = time.monotonic()
        self.vr_ok = True

    def _loop(self) -> None:
        now = time.monotonic()
        dt = max(1e-4, now - self._prev)
        self._prev = now
        if not self.vr_ok:
            return
        if now - self.last_vr > self.data_timeout:
            return
        dp, dr = self.pose.process(dt)
        T = self.pose.compute_target_pose(
            dp, dr, self.robot_init_pos, self.robot_init_rot
        )
        T_flange = T @ self._T_tcp_to_flange
        sol = self.bridge.solve(self.arm_key, T_flange)
        if sol is None:
            return
        safe, _ = self.safety.filter(sol, dt)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [float(x) for x in safe.tolist()]
        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] q={np.round(safe, 3).tolist()}",
                throttle_duration_sec=1.0,
            )
            return
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeTeleopTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
