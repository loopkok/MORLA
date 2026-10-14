#!/usr/bin/env python3
"""MuJoCo sim — subscribe Astral arm joint_commands, drive dual-arm MJCF.

Drop-in stand-in for ``astral_robot_control`` (do not run both on same topics).

Pipeline:
  quest3 → astral_arm_teleop_{left,right} → /{side}_arm/joint_commands
                                      → this node (viewer + optional joint_states)

Gripper: the MJCF has no gripper joint yet, so this node only subscribes
  /{left,right}_gripper/command (Float64 0–1) and /{side}_gripper/joint_commands
  (JointState rad) and echoes them to /{side}_gripper/joint_states — enough to
  verify the pinch→gripper chain in sim before a gripper joint is modeled.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from astral_mujoco_sim.fps_counter import FPSCounter
from astral_mujoco_sim.joint_names import (
    LEFT_ARM_JOINT_NAMES,
    LEFT_MJCF_JOINTS,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_MJCF_JOINTS,
    normalize_joint_name,
)
from astral_mujoco_sim.latency_meter import LatencyMeter, stamp_age_ms


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


# Mirror astral_robot_control gripper contract (no SDK / no MJCF joint here —
# the sim only echoes commands to /{side}_gripper/joint_states so the gripper
# teleop chain is observable before a gripper joint is added to the MJCF).
LEFT_GRIPPER_NAME = "left_gripper"
RIGHT_GRIPPER_NAME = "right_gripper"
# Head (mirror astral_robot_control HEAD_JOINT_NAMES = [head_yaw, head_pitch]).
# No MJCF head joint exists yet, so the sim only echoes /head/joint_commands →
# /head/joint_states so the thumbstick head-teleop chain is observable.
HEAD_NAMES = ["head_yaw", "head_pitch"]


def _spin_drain(node: Node, max_callbacks: int = 24) -> None:
    """Process pending ROS callbacks without blocking the sim loop.

    A single ``spin_once`` handles one event. At 150 Hz × 2 arms the executor
    would otherwise queue (and BEST_EFFORT-drop) commands.
    """
    for _ in range(max_callbacks):
        rclpy.spin_once(node, timeout_sec=0.0)


def _pack_arm(
    msg: JointState, expected: List[str]
) -> Optional[np.ndarray]:
    if not msg.position:
        return None
    q = np.asarray(msg.position, dtype=np.float64).reshape(-1)
    if msg.name and len(msg.name) == len(q):
        lut = {normalize_joint_name(n): float(v) for n, v in zip(msg.name, q)}
        matched = sum(1 for n in expected if n in lut)
        if matched >= max(1, len(expected) // 2):
            return np.array([lut.get(n, 0.0) for n in expected], dtype=np.float64)
    if q.size < 7:
        return None
    return q[:7].copy()


class AstralMujocoSimNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_mujoco_sim_node")
        self.declare_parameter("mjcf_path", "")
        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("realtime", True)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("state_rate", 50.0)
        self.declare_parameter("fps_print_interval", 5.0)
        self.declare_parameter("print_latency", True)
        self.declare_parameter("latency_print_interval", 2.0)
        self.declare_parameter(
            "init_pose_left",
            [0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0],
        )
        self.declare_parameter(
            "init_pose_right",
            [0.32, -0.11, 0.53, -0.80, 0.28, 0.0, 0.0],
        )
        # Gripper (mirror astral_robot_control; sim has no MJCF gripper joint,
        # so we only subscribe + echo joint_states — no qpos/ctrl drive).
        self.declare_parameter("left_gripper_ns", "left_gripper")
        self.declare_parameter("right_gripper_ns", "right_gripper")
        self.declare_parameter("enable_gripper_cmd", True)
        self.declare_parameter("left_gripper_open_rad", 0.8)
        self.declare_parameter("left_gripper_closed_rad", 0.0)
        self.declare_parameter("right_gripper_open_rad", 0.8)
        self.declare_parameter("right_gripper_closed_rad", 0.0)
        # Head echo (no MJCF head joint): subscribe /head/joint_commands,
        # echo to /head/joint_states + log. Mirrors astral_robot_control head.
        self.declare_parameter("head_ns", "head")
        self.declare_parameter("enable_head_cmd", True)

        mjcf_path = str(self.get_parameter("mjcf_path").value).strip()
        if not mjcf_path:
            share = Path(get_package_share_directory("astral_mujoco_sim"))
            mjcf_path = str(share / "assets" / "mjcf" / "astral_dual.xml")
        if not Path(mjcf_path).is_file():
            raise FileNotFoundError(f"MJCF not found: {mjcf_path}")

        try:
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError("pip install mujoco") from exc

        self._mujoco = mujoco
        self._viewer_mod = mujoco.viewer
        self.get_logger().info(f"Loading MJCF: {mjcf_path}")
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        self._qpos_addrs: Dict[str, np.ndarray] = {
            "left": self._addrs(LEFT_MJCF_JOINTS),
            "right": self._addrs(RIGHT_MJCF_JOINTS),
        }
        self._cmd_names = {
            "left": LEFT_ARM_JOINT_NAMES,
            "right": RIGHT_ARM_JOINT_NAMES,
        }

        q_l = np.asarray(
            self.get_parameter("init_pose_left").value, dtype=float
        ).reshape(7)
        q_r = np.asarray(
            self.get_parameter("init_pose_right").value, dtype=float
        ).reshape(7)
        self._latest = {"left": q_l.copy(), "right": q_r.copy()}
        self._apply_arms(q_l, q_r)

        self._lock = threading.Lock()
        self._cmd_count = {"left": 0, "right": 0}
        self._cmd_recv_mono = {"left": 0.0, "right": 0.0}
        self._cmd_pending = {"left": False, "right": False}
        interval = float(self.get_parameter("fps_print_interval").value)
        self._cmd_fps = {
            "left": FPSCounter(print_interval=interval),
            "right": FPSCounter(print_interval=interval),
        }
        self._sim_fps = FPSCounter(window=200, print_interval=interval)
        self._print_latency = bool(self.get_parameter("print_latency").value)
        self._lat = LatencyMeter(
            float(self.get_parameter("latency_print_interval").value)
        )

        qos = _sensor_qos()
        self.create_subscription(
            JointState, "/left_arm/joint_commands", self._on_left, qos
        )
        self.create_subscription(
            JointState, "/right_arm/joint_commands", self._on_right, qos
        )

        self._pub_state = bool(self.get_parameter("publish_joint_states").value)
        self._state_pubs = {}
        if self._pub_state:
            self._state_pubs["left"] = self.create_publisher(
                JointState, "/left_arm/joint_states", qos
            )
            self._state_pubs["right"] = self.create_publisher(
                JointState, "/right_arm/joint_states", qos
            )
            rate = float(self.get_parameter("state_rate").value)
            self.create_timer(1.0 / max(1.0, rate), self._publish_states)

        # Gripper: subscribe command topics, echo to joint_states. No MJCF
        # gripper joint exists yet, so nothing is driven in the viewer.
        self._grip_open_rad = {
            "left": float(self.get_parameter("left_gripper_open_rad").value),
            "right": float(self.get_parameter("right_gripper_open_rad").value),
        }
        self._grip_closed_rad = {
            "left": float(self.get_parameter("left_gripper_closed_rad").value),
            "right": float(self.get_parameter("right_gripper_closed_rad").value),
        }
        self._grip_rad = {
            "left": self._grip_open_rad["left"],
            "right": self._grip_open_rad["right"],
        }
        self._grip_name = {"left": LEFT_GRIPPER_NAME, "right": RIGHT_GRIPPER_NAME}
        self._grip_pubs = {}
        self._grip_log_t = {"left": 0.0, "right": 0.0}
        if bool(self.get_parameter("enable_gripper_cmd").value):
            for side, g_ns in (
                ("left", str(self.get_parameter("left_gripper_ns").value).strip("/")),
                ("right", str(self.get_parameter("right_gripper_ns").value).strip("/")),
            ):
                self.create_subscription(
                    JointState, f"/{g_ns}/joint_commands",
                    lambda msg, s=side: self._on_grip_js(s, msg), qos,
                )
                self.create_subscription(
                    Float64, f"/{g_ns}/command",
                    lambda msg, s=side: self._on_grip_ratio(s, msg), qos,
                )
                if self._pub_state:
                    self._grip_pubs[side] = self.create_publisher(
                        JointState, f"/{g_ns}/joint_states", qos
                    )

        # Head echo (no MJCF head joint): subscribe + echo + log.
        self._head_rad = [0.0, 0.0]  # [head_yaw, head_pitch]
        self._head_pub = None
        self._head_log_t = 0.0
        if bool(self.get_parameter("enable_head_cmd").value):
            h_ns = str(self.get_parameter("head_ns").value).strip("/")
            self.create_subscription(
                JointState, f"/{h_ns}/joint_commands", self._on_head_js, qos
            )
            if self._pub_state:
                self._head_pub = self.create_publisher(
                    JointState, f"/{h_ns}/joint_states", qos
                )

        self.enable_viewer = bool(self.get_parameter("enable_viewer").value)
        self.realtime = bool(self.get_parameter("realtime").value)
        grip_topics = (
            "/left_gripper|/right_gripper/(command|joint_commands)"
            if self._grip_pubs or bool(self.get_parameter("enable_gripper_cmd").value)
            else "gripper disabled"
        )
        head_topic = (
            "/head/joint_commands"
            if bool(self.get_parameter("enable_head_cmd").value)
            else "head disabled"
        )
        self.get_logger().info(
            f"Astral MuJoCo sim ready: nq={self.model.nq} viewer={self.enable_viewer} "
            f"arms=/left_arm|/right_arm/joint_commands gripper={grip_topics} "
            f"head={head_topic}"
        )

    def _addrs(self, joint_names: List[str]) -> np.ndarray:
        addrs = []
        for name in joint_names:
            jid = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if jid < 0:
                raise RuntimeError(f"MJCF missing joint '{name}'")
            addrs.append(int(self.model.jnt_qposadr[jid]))
        return np.asarray(addrs, dtype=int)

    def _on_left(self, msg: JointState) -> None:
        self._on_cmd("left", msg)

    def _on_right(self, msg: JointState) -> None:
        self._on_cmd("right", msg)

    def _on_grip_js(self, side: str, msg: JointState) -> None:
        if not msg.position:
            return
        self._set_grip_rad(side, float(msg.position[0]))

    def _on_grip_ratio(self, side: str, msg: Float64) -> None:
        r = max(0.0, min(1.0, float(msg.data)))
        lo = self._grip_open_rad[side]
        hi = self._grip_closed_rad[side]
        self._set_grip_rad(side, lo + r * (hi - lo))

    def _set_grip_rad(self, side: str, rad: float) -> None:
        with self._lock:
            self._grip_rad[side] = float(rad)
        now = time.monotonic()
        if now - self._grip_log_t[side] > 1.0:
            self._grip_log_t[side] = now
            self.get_logger().info(
                f"[Gripper][{side}] cmd rad={rad:.3f} "
                f"(open={self._grip_open_rad[side]:.3f} "
                f"closed={self._grip_closed_rad[side]:.3f})"
            )

    def _on_head_js(self, msg: JointState) -> None:
        if not msg.position:
            return
        d = dict(zip(msg.name, msg.position)) if msg.name else {}
        with self._lock:
            yaw = float(d.get("head_yaw", msg.position[0]))
            pitch = float(
                d.get("head_pitch", msg.position[1] if len(msg.position) > 1 else 0.0)
            )
            self._head_rad = [yaw, pitch]
        now = time.monotonic()
        if now - self._head_log_t > 1.0:
            self._head_log_t = now
            self.get_logger().info(f"[Head] cmd yaw={yaw:.3f} pitch={pitch:.3f}")

    def _on_cmd(self, side: str, msg: JointState) -> None:
        q = _pack_arm(msg, self._cmd_names[side])
        if q is None:
            return
        with self._lock:
            self._latest[side] = q
            self._cmd_count[side] += 1
            self._cmd_recv_mono[side] = time.monotonic()
            self._cmd_pending[side] = True
        if self._print_latency:
            e2e = stamp_age_ms(msg.header.stamp)
            if 0.0 <= e2e < 5000.0:
                self._lat.add(f"e2e.{side}", e2e)
        fps = self._cmd_fps[side].tick()
        if self._cmd_fps[side].should_print():
            self.get_logger().info(
                f"[Sim Cmd FPS][{side}] {fps:.0f} Hz total={self._cmd_count[side]}"
            )

    def _apply_arms(self, q_l: np.ndarray, q_r: np.ndarray) -> None:
        for i, a in enumerate(self._qpos_addrs["left"]):
            self.data.qpos[a] = float(q_l[i])
        for i, a in enumerate(self._qpos_addrs["right"]):
            self.data.qpos[a] = float(q_r[i])
        # Keep actuators in sync for passive viewer
        for i, name in enumerate(LEFT_MJCF_JOINTS):
            aid = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}"
            )
            if aid >= 0:
                self.data.ctrl[aid] = float(q_l[i])
        for i, name in enumerate(RIGHT_MJCF_JOINTS):
            aid = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}"
            )
            if aid >= 0:
                self.data.ctrl[aid] = float(q_r[i])
        self._mujoco.mj_forward(self.model, self.data)

    def _publish_states(self) -> None:
        with self._lock:
            ql = self._latest["left"].copy()
            qr = self._latest["right"].copy()
        now = self.get_clock().now().to_msg()
        for side, q, names in (
            ("left", ql, LEFT_ARM_JOINT_NAMES),
            ("right", qr, RIGHT_ARM_JOINT_NAMES),
        ):
            msg = JointState()
            msg.header.stamp = now
            msg.name = list(names)
            msg.position = [float(x) for x in q.tolist()]
            self._state_pubs[side].publish(msg)
        for side, pub in self._grip_pubs.items():
            with self._lock:
                rad = float(self._grip_rad[side])
            gmsg = JointState()
            gmsg.header.stamp = now
            gmsg.name = [self._grip_name[side]]
            gmsg.position = [rad]
            pub.publish(gmsg)
        if self._head_pub is not None:
            with self._lock:
                head = list(self._head_rad)
            hmsg = JointState()
            hmsg.header.stamp = now
            hmsg.name = list(HEAD_NAMES)
            hmsg.position = [float(head[0]), float(head[1])]
            self._head_pub.publish(hmsg)

    def run(self) -> None:
        viewer = None
        if self.enable_viewer:
            viewer = self._viewer_mod.launch_passive(self.model, self.data)
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -15
            viewer.cam.distance = 1.6
            viewer.cam.lookat[:] = [0.0, 0.0, 0.35]

        try:
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                _spin_drain(self)
                with self._lock:
                    ql = self._latest["left"].copy()
                    qr = self._latest["right"].copy()
                    recv = dict(self._cmd_recv_mono)
                    pending = dict(self._cmd_pending)
                    self._cmd_pending = {"left": False, "right": False}
                t_apply = time.monotonic()
                self._apply_arms(ql, qr)
                if self._print_latency:
                    for side in ("left", "right"):
                        if pending[side] and recv[side] > 0.0:
                            self._lat.add(
                                f"apply.{side}", (t_apply - recv[side]) * 1000.0
                            )
                    if self._lat.has_samples() and self._lat.should_print():
                        self.get_logger().info(
                            f"[Latency][sim] {self._lat.format_and_reset()}"
                        )
                if viewer is not None:
                    viewer.sync()
                sim_fps = self._sim_fps.tick()
                if self._sim_fps.should_print():
                    self.get_logger().info(
                        f"[Sim Loop FPS] {sim_fps:.0f} Hz "
                        f"cmds L={self._cmd_count['left']} R={self._cmd_count['right']}"
                    )
                if self.realtime:
                    time.sleep(self.model.opt.timestep)
        finally:
            if viewer is not None:
                viewer.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstralMujocoSimNode()
    try:
        node.run()
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
