#!/usr/bin/env python3
"""End-to-end dataflow test — NO hardware / no driver required.

Quest wrist (robot_world, convert_to_robot:=true) → PoseProcessor(vr_to_arm_rot)
→ IK → SafetyFilter.
Default ``vr_to_arm_rot = I``.

Usage:
  ros2 run astral_arm_teleop test_dataflow --ros-args \\
    -p arm_side:=right -p solver_type:=analytic_dh
  ros2 run astral_arm_teleop test_dataflow --ros-args \\
    -p arm_side:=right -p solver_type:=urdf_numerical
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from astral_arm_teleop.ik import make_ik_solver
from astral_arm_teleop.ik.factory import default_astral_urdf_path
from astral_arm_teleop.pose_processor import PoseProcessor
from astral_arm_teleop.safety_filter import SafetyFilter

_VR_TO_ARM_I = [
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
]

_INIT_Q_LEFT = [0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00]
_INIT_Q_RIGHT = [0.32, -0.11, 0.53, -0.80, 0.28, 0.00, 0.00]


class DataFlowTestNode(Node):
    def __init__(self) -> None:
        super().__init__("test_dataflow")
        self.declare_parameter("arm_side", "right")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("solver_type", "analytic_dh")
        self.declare_parameter("urdf_path", "")  # empty → astral_robot.pin.urdf
        self.declare_parameter("vr_to_arm_rot", [])
        self.declare_parameter("robot_world_to_base_rot", [])
        self.declare_parameter("pos_smoothing", 0.8)
        self.declare_parameter("rot_smoothing", 0.8)
        self.declare_parameter("motion_scale", 0.65)
        self.declare_parameter("flip_pitch", False)
        self.declare_parameter("max_joint_vel", 7.5)
        self.declare_parameter("tcp_offset", [0.0] * 6)
        self.declare_parameter("init_pose", [])

        self.arm_side = str(self.get_parameter("arm_side").value).lower()
        arm_key = "L" if self.arm_side == "left" else "R"
        rate = float(self.get_parameter("control_rate").value)

        init_q = list(self.get_parameter("init_pose").value)
        if len(init_q) != 7:
            init_q = list(_INIT_Q_LEFT if arm_key == "L" else _INIT_Q_RIGHT)
        init_q = np.asarray(init_q, dtype=float).reshape(7)

        st = str(self.get_parameter("solver_type").value)
        urdf = str(self.get_parameter("urdf_path").value).strip()
        if st.strip().lower() in ("urdf_numerical", "urdf", "numerical") and not urdf:
            urdf = default_astral_urdf_path()
        self.bridge = make_ik_solver(st, urdf_path=urdf)
        lo, hi = self.bridge.joint_limits(arm_key)
        init_q = np.clip(init_q, lo + 0.02, hi - 0.02)
        self.arm_key = arm_key

        q14 = np.zeros(14)
        if arm_key == "L":
            q14[0:7] = init_q
        else:
            q14[7:14] = init_q
        self.bridge.sync_state(q14)
        T0 = self.bridge.fk(arm_key, init_q)
        _ = self.bridge.solve(arm_key, T0)
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()

        R_flat = list(self.get_parameter("vr_to_arm_rot").value)
        if len(R_flat) != 9:
            R_flat = list(self.get_parameter("robot_world_to_base_rot").value)
        if len(R_flat) != 9:
            R_flat = list(_VR_TO_ARM_I)
        R = np.asarray(R_flat, dtype=float).reshape(3, 3)

        self.pose_proc = PoseProcessor(
            vr_to_arm_rot=R,
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
        self.safety.set_initial_state(init_q)

        self.frame_count = 0
        self.ik_ok = 0
        self.ik_fail = 0
        self.safety_hits = 0
        self.t0 = time.monotonic()
        self.t_active0 = None  # first frame with VR (for FPS excl. wait)
        self.last_print = self.t0
        self._last_safe = None
        self._last_T = None
        self._solve_ms: list[float] = []
        self._last_solve_ms = 0.0

        topic = f"quest3/{self.arm_side}_wrist_pose"
        self.create_subscription(PoseStamped, topic, self._vr_cb, 10)
        self.dt = 1.0 / max(1.0, rate)
        self.create_timer(self.dt, self._loop)
        self.get_logger().info(
            f"test_dataflow: {topic} solver={self.bridge.method_name} "
            f"EE0={np.round(self.robot_init_pos, 3).tolist()} "
            f"scale={self.pose_proc.motion_scale} "
            f"R={'I' if np.allclose(R, np.eye(3)) else 'custom'} "
            f"(prints IK solve_ms)"
        )

    def _vr_cb(self, msg: PoseStamped) -> None:
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            return
        self.pose_proc.update_vr_pose(pos, Rotation.from_quat(q))

    def _solve_stats(self) -> str:
        if not self._solve_ms:
            return "solve_ms=n/a"
        a = np.asarray(self._solve_ms, dtype=float)
        return (
            f"solve_ms last={self._last_solve_ms:.2f} "
            f"mean={a.mean():.2f} p50={np.percentile(a, 50):.2f} "
            f"p95={np.percentile(a, 95):.2f} max={a.max():.2f}"
        )

    def _loop(self) -> None:
        if not self.pose_proc.is_calibrated:
            return
        if self.t_active0 is None:
            self.t_active0 = time.monotonic()
        dp, dr = self.pose_proc.process(self.dt)
        T_tcp = self.pose_proc.compute_target_pose(
            dp, dr, self.robot_init_pos, self.robot_init_rot
        )
        T_flange = T_tcp @ self._T_tcp_to_flange
        t0 = time.perf_counter()
        sol = self.bridge.solve(self.arm_key, T_flange)
        self._last_solve_ms = (time.perf_counter() - t0) * 1000.0
        self._solve_ms.append(self._last_solve_ms)
        if len(self._solve_ms) > 2000:
            self._solve_ms = self._solve_ms[-1000:]

        self.frame_count += 1
        self._last_T = T_flange
        if sol is None:
            self.ik_fail += 1
        else:
            self.ik_ok += 1
            safe, info = self.safety.filter(sol, self.dt)
            self._last_safe = safe
            if info["clamped"] or info["velocity_limited"] or info["collision"]:
                self.safety_hits += 1

        now = time.monotonic()
        if now - self.last_print >= 2.0:
            fps_all = self.frame_count / max(1e-6, now - self.t0)
            fps_act = self.frame_count / max(
                1e-6, now - (self.t_active0 or self.t0)
            )
            rate = 100.0 * self.ik_ok / max(1, self.frame_count)
            self.get_logger().info(
                f"Frames={self.frame_count} FPS_active={fps_act:.1f} "
                f"FPS_incl_wait={fps_all:.1f} IK_OK={rate:.0f}% "
                f"fail={self.ik_fail} safety={self.safety_hits}"
            )
            self.get_logger().info(f"  {self._solve_stats()}")
            if self._last_safe is not None and self._last_T is not None:
                p = self._last_T[:3, 3]
                self.get_logger().info(
                    f"  EE=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) "
                    f"q_deg={np.degrees(self._last_safe).round(1).tolist()}"
                )
            self.last_print = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataFlowTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        elapsed = time.monotonic() - node.t0
        active = (
            time.monotonic() - node.t_active0
            if node.t_active0 is not None
            else 0.0
        )
        node.get_logger().info(
            f"SUMMARY t={elapsed:.1f}s active={active:.1f}s "
            f"frames={node.frame_count} ik_ok={node.ik_ok} fail={node.ik_fail} "
            f"vr={'YES' if node.pose_proc.is_calibrated else 'NO'}"
        )
        node.get_logger().info(f"SUMMARY {node._solve_stats()}")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
