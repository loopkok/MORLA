#!/usr/bin/env python3
"""Synthetic human arm publisher — Quest-free test of the elbow prior.

Publishes a minimal fake IOBT skeleton on the same topics the real
``quest3_udp_mocap`` uses (frame ``robot_body``, no Unity conversion needed):

  quest3/body_joint_names   String (latched JSON) — hips/chest/{side}-arm-*
  quest3/body_joints        PoseArray — shoulder/elbow positions
  quest3/{side}_wrist_pose  PoseStamped — wrist 6D

Scenarios (``scenario`` param):
  elbow_circle : wrist fixed, elbow plane circles the shoulder->wrist axis.
                 Watch: robot EE holds, robot elbow sweeps with the human's
                 (needs use_human_elbow:=true on the teleop node).
  wrist_spin   : elbow fixed, wrist slowly rolls. Watch: only the robot
                 wrist moves; shoulder/elbow stay put.
  combined     : both at once.

Run:
  ros2 run astral_arm_teleop body_joints_sim --ros-args -p side:=right
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scipy.spatial.transform import Rotation
from std_msgs.msg import String


class BodyJointsSim(Node):
    def __init__(self) -> None:
        super().__init__("body_joints_sim")
        self.declare_parameter("side", "right")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter(
            "scenario", "elbow_circle"
        )  # elbow_circle | wrist_spin | combined
        self.declare_parameter("sweep_rate", 0.5)  # rad/s elbow plane / wrist roll
        # Fake human in robot_body (≈ base_link with default vr_to_arm_rot=I).
        self.declare_parameter("shoulder_xyz", [-0.15, -0.05, -0.05])
        self.declare_parameter("wrist_xyz", [-0.42, -0.28, -0.30])
        self.declare_parameter("upper_arm_len", 0.28)
        self.declare_parameter("forearm_len", 0.26)

        self.side = str(self.get_parameter("side").value).lower()
        if self.side not in ("left", "right"):
            raise ValueError("side must be left|right")
        self.scenario = str(self.get_parameter("scenario").value)
        self.omega = float(self.get_parameter("sweep_rate").value)
        self.Sh = np.array(self.get_parameter("shoulder_xyz").value, dtype=float)
        self.Wh = np.array(self.get_parameter("wrist_xyz").value, dtype=float)
        self.l_u = float(self.get_parameter("upper_arm_len").value)
        self.l_f = float(self.get_parameter("forearm_len").value)

        # Human elbow circle about the shoulder->wrist axis (same triangle
        # math as the robot arm-angle circle).
        sw = self.Wh - self.Sh
        d = float(np.linalg.norm(sw))
        if d < 1e-6 or d >= self.l_u + self.l_f or d <= abs(self.l_u - self.l_f):
            raise ValueError(
                f"infeasible human triangle: |S-W|={d:.3f}, "
                f"l_u={self.l_u}, l_f={self.l_f}"
            )
        self.u = sw / d
        self.x0 = (self.l_u**2 - self.l_f**2 + d * d) / (2.0 * d)
        self.r0 = math.sqrt(max(0.0, self.l_u**2 - self.x0**2))
        t = np.cross(self.u, [1.0, 0.0, 0.0])
        if np.linalg.norm(t) < 1e-6:
            t = np.cross(self.u, [0.0, 1.0, 0.0])
        self.e1 = t / np.linalg.norm(t)
        self.e2 = np.cross(self.u, self.e1)

        self.names = [
            "hips",
            "chest",
            f"{self.side}-arm-upper",
            f"{self.side}-arm-lower",
        ]
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.names_pub = self.create_publisher(
            String, "quest3/body_joint_names", latched
        )
        self.body_pub = self.create_publisher(PoseArray, "quest3/body_joints", 10)
        self.wrist_pub = self.create_publisher(
            PoseStamped, f"quest3/{self.side}_wrist_pose", 10
        )

        nm = String()
        nm.data = json.dumps(self.names)
        self.names_pub.publish(nm)

        # wrist_spin pins the fake elbow; make it self-consistent with the
        # robot's actual arm angle (a real human's elbow always is), else the
        # prior would demand an arm angle where the roll is wrist-infeasible.
        # Delayed + non-blocking: the wrist stream must be live first so the
        # teleop arms and joint_states echo the settled init pose.
        self._fixed_elbow = self._elbow(0.6)
        self._js_holder: list = []
        self._matched = False
        if self.scenario in ("wrist_spin", "combined"):
            from sensor_msgs.msg import JointState

            self._js_sub = self.create_subscription(
                JointState,
                f"/{self.side}_arm/joint_states",
                lambda m: self._js_holder.append(m),
                10,
            )
            self._match_start = time.monotonic()
            self._match_timer = self.create_timer(0.5, self._match_once)

        self._t0 = self.get_clock().now()
        rate = float(self.get_parameter("rate").value)
        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f"fake human arm [{self.side}] scenario={self.scenario} "
            f"S={np.round(self.Sh, 2).tolist()} W={np.round(self.Wh, 2).tolist()} "
            f"circle r={self.r0:.3f} m"
        )

    def _match_once(self) -> None:
        """Periodic (0.5 s) trial until matched or 6 s deadline; self-cancels."""
        if self._matched:
            return
        if not self._js_holder:
            # Wait ~2.5 s for the teleop to arm and settle at init pose first.
            if time.monotonic() - self._match_start > 8.5:
                self._matched = True
                self._match_timer.cancel()
                self.get_logger().warn(
                    "no joint_states sample; fixed elbow stays at phi=0.6"
                )
            return
        if time.monotonic() - self._match_start < 2.5:
            return  # let the arm settle before sampling
        q = np.asarray(self._js_holder[-1].position[:7], dtype=float)
        self._js_holder.clear()
        if q.size < 7:
            self._matched = True
            self._match_timer.cancel()
            return
        self._matched = True
        self._match_timer.cancel()
        from astral_arm_teleop.ik.geometric import GeometricIKSolver, _axis_angle_rot

        solver = GeometricIKSolver(self.side, fast_mode=True)
        g = solver.geom
        T = np.eye(4)
        I3 = np.eye(3)
        for i in range(3):
            R = _axis_angle_rot(g.axes[i], float(q[i]))
            Ti = np.eye(4)
            Ti[:3, :3] = R
            Ti[:3, 3] = (I3 - R) @ g.points[i]
            T = T @ Ti
        E_robot = (T @ np.append(g.E0, 1.0))[:3]
        d = E_robot - g.S
        n = float(np.linalg.norm(d))
        if n < 1e-6:
            return
        self._fixed_elbow = self.Sh + self.l_u * (d / n)
        self.get_logger().info(
            f"fixed elbow matched to robot arm angle (dir "
            f"{np.round(d / n, 2).tolist()})"
        )

    def _elbow(self, phi: float) -> np.ndarray:
        return (
            self.Sh
            + self.x0 * self.u
            + self.r0 * (math.cos(phi) * self.e1 + math.sin(phi) * self.e2)
        )

    @staticmethod
    def _pose(p: np.ndarray, q_xyzw=(0.0, 0.0, 0.0, 1.0)) -> Pose:
        m = Pose()
        m.position.x, m.position.y, m.position.z = float(p[0]), float(p[1]), float(p[2])
        (
            m.orientation.x,
            m.orientation.y,
            m.orientation.z,
            m.orientation.w,
        ) = q_xyzw
        return m

    def _tick(self) -> None:
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        # Sinusoid: keeps the wrist roll inside the robot's wrist joint limits
        # (unbounded growth would force arm motion — kinematic necessity).
        roll = (
            0.6 * math.sin(self.omega * t)
            if self.scenario in ("wrist_spin", "combined")
            else 0.0
        )

        stamp = self.get_clock().now().to_msg()
        E = (
            self._elbow(self.omega * t)
            if self.scenario in ("elbow_circle", "combined")
            else self._fixed_elbow
        )

        body = PoseArray()
        body.header.stamp = stamp
        body.header.frame_id = "robot_body"
        chest = self.Sh * 0.5  # plausible midpoint; only arm joints are used
        body.poses = [
            self._pose(np.zeros(3)),  # hips = frame root
            self._pose(chest),
            self._pose(self.Sh),  # {side}-arm-upper == shoulder ball
            self._pose(E),  # {side}-arm-lower == elbow
        ]
        self.body_pub.publish(body)

        wrist = PoseStamped()
        wrist.header.stamp = stamp
        wrist.header.frame_id = "robot_body"
        q = Rotation.from_euler("x", roll).as_quat()
        wrist.pose = self._pose(self.Wh, tuple(float(v) for v in q))
        self.wrist_pub.publish(wrist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BodyJointsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
