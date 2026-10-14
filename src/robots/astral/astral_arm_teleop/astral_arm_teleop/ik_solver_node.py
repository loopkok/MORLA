#!/usr/bin/env python3
"""Dedicated single-arm analytic DH IK node (Nero-style).

Subscribes:
  ~/target_flange_pose (PoseStamped) — flange in that arm's *_base_link
Publishes:
  ~/joint_solution (Float64MultiArray) — 7 joints
  ~/ik_status (String)
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray, String

from astral_arm_teleop.ik.factory import make_single_arm_ik


class IKSolverNode(Node):
    def __init__(self) -> None:
        super().__init__("ik_solver_node")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("solver_type", "analytic_dh")

        side = str(self.get_parameter("arm_side").value).lower()
        st = str(self.get_parameter("solver_type").value).lower()
        if st != "analytic_dh":
            raise ValueError("ik_solver_node currently supports analytic_dh only")

        self.ik = make_single_arm_ik(side)
        self.get_logger().info(
            f"IK ready: arm={side} nq={self.ik.nq} frame={side}_base_link"
        )

        self._solve_count = 0
        self._fail_count = 0
        self._total_time = 0.0
        self._last_print = time.monotonic()

        self.create_subscription(
            PoseStamped, "~/target_flange_pose", self._pose_cb, 10
        )
        self.joint_pub = self.create_publisher(
            Float64MultiArray, "~/joint_solution", 10
        )
        self.status_pub = self.create_publisher(String, "~/ik_status", 10)

    def _pose_cb(self, msg: PoseStamped) -> None:
        t0 = time.perf_counter()
        T = np.eye(4, dtype=float)
        T[0, 3] = msg.pose.position.x
        T[1, 3] = msg.pose.position.y
        T[2, 3] = msg.pose.position.z
        T[:3, :3] = Rotation.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        ).as_matrix()

        sol = self.ik.solve(T)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self._solve_count += 1
        self._total_time += dt_ms

        out = Float64MultiArray()
        if sol is not None:
            out.data = sol.tolist()
            self.joint_pub.publish(out)
            status = f"OK dt={dt_ms:.2f}ms"
        else:
            out.data = [float("nan")] * 7
            self.joint_pub.publish(out)
            self._fail_count += 1
            status = "FAILED"
        self.status_pub.publish(String(data=status))

        now = time.monotonic()
        if now - self._last_print >= 10.0 and self._solve_count > 0:
            ok = self._solve_count - self._fail_count
            self.get_logger().info(
                f"Stats: {self._solve_count} solves | "
                f"avg {self._total_time / self._solve_count:.2f}ms | "
                f"OK {ok}/{self._solve_count} ({100.0 * ok / self._solve_count:.1f}%)"
            )
            self._last_print = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKSolverNode()
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
