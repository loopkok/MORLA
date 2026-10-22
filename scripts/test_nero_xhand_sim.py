#!/usr/bin/env python3
"""Drive the profile-based Nero/XHand sim and collector with synthetic ROS data."""

from __future__ import annotations

import argparse
from collections import Counter
from io import BytesIO
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from PIL import Image
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, String
from xhand_control_interfaces.msg import XHandStateArray

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src/robots/xhand/xhand_retargeting"))
from xhand_retargeting.preprocess import UNITY_TO_ROBOT  # noqa: E402
from xhand_retargeting.test_hand_publisher import make_open_hand  # noqa: E402


def stamp_message(node, header) -> None:
    header.stamp = node.get_clock().now().to_msg()


class SimIntegrationDriver(Node):
    def __init__(self, output_root: Path, duration_s: float) -> None:
        super().__init__("morla_sim_integration_driver")
        self.output_root = output_root
        self.duration_s = duration_s
        self.session = output_root / "nero_left_xhand" / "smoke"
        self.episode = self.session / "episode000000"
        self.counts: Counter[str] = Counter()
        self.latest_collect_state = {}
        self.errors: list[str] = []

        sensor_qos = qos_profile_sensor_data
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.landmarks_pub = self.create_publisher(
            PoseArray, "/hand_landmarks/left", sensor_qos
        )
        self.armed_pub = self.create_publisher(Bool, "/teleop/armed", reliable)
        self.arm_command_pub = self.create_publisher(
            JointState, "/left_arm/joint_commands", reliable
        )
        self.collect_control_pub = self.create_publisher(
            String, "/data_collect/control", reliable
        )
        camera_qos = sensor_qos
        self.camera_pubs = {
            name: self.create_publisher(
                CompressedImage,
                f"/quest3_video_streamer/collect/{name}",
                camera_qos,
            )
            for name in ("base", "left_wrist")
        }

        self.create_subscription(
            String, "/data_collect/state", self._on_collect_state, latched
        )
        self.create_subscription(
            JointState, "/left_arm/joint_commands",
            lambda _msg: self._count("arm_command"), sensor_qos,
        )
        self.create_subscription(
            JointState, "/left_arm/joint_states",
            lambda _msg: self._count("arm_state"), sensor_qos,
        )
        self.create_subscription(
            JointState, "/left_hand/joint_commands",
            self._on_hand_command, sensor_qos,
        )
        self.create_subscription(
            JointState, "/left_hand/joint_states",
            self._on_hand_state, sensor_qos,
        )
        self.create_subscription(
            XHandStateArray, "/left_hand/driver/xhand_state",
            self._on_native_hand_state, sensor_qos,
        )

        unity = make_open_hand()
        robot_raw = (UNITY_TO_ROBOT @ unity.T).T
        self.hand_points = robot_raw.copy()
        self.joint_names = [f"left_joint{i}" for i in range(1, 8)]
        self.arm_positions = [0.0] * 7
        self.jpeg = self._make_jpeg()
        self._last_emit = 0.0
        self._camera_frame = 0

    def _count(self, name: str) -> None:
        self.counts[name] += 1

    def _on_collect_state(self, msg: String) -> None:
        try:
            self.latest_collect_state = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.errors.append("collector published invalid JSON status")

    def _on_hand_command(self, msg: JointState) -> None:
        self._count("hand_command")
        if len(msg.name) != 12 or len(msg.position) != 12:
            self.errors.append("standard XHand command is not a named 12-vector")
        elif not np.isfinite(np.asarray(msg.position, dtype=float)).all():
            self.errors.append("standard XHand command contains non-finite values")

    def _on_hand_state(self, msg: JointState) -> None:
        self._count("hand_state")
        if len(msg.name) != 12 or len(msg.position) != 12:
            self.errors.append("adapted XHand state is not a named 12-vector")

    def _on_native_hand_state(self, msg: XHandStateArray) -> None:
        self._count("native_hand_state")
        if not msg.hand_states:
            self.errors.append("simulator native hand state is empty")
            return
        if msg.sensor_states and len(msg.sensor_states[0].finger_sensor_states) != 5:
            self.errors.append("native tactile state does not contain five fingers")
        if msg.sensor_states:
            for finger in msg.sensor_states[0].finger_sensor_states:
                if len(finger.raw_force) != 120 or len(finger.raw_temperature) != 20:
                    self.errors.append("native tactile finger shape is not 120 force/20 temperature")
                    break

    @staticmethod
    def _make_jpeg() -> bytes:
        image = Image.new("RGB", (64, 48), (48, 96, 160))
        buff = BytesIO()
        image.save(buff, format="JPEG", quality=85)
        return buff.getvalue()

    def _arm_command(self) -> JointState:
        msg = JointState()
        stamp_message(self, msg.header)
        msg.header.frame_id = "morla_sim_integration"
        msg.name = list(self.joint_names)
        msg.position = list(self.arm_positions)
        return msg

    def _landmarks(self) -> PoseArray:
        msg = PoseArray()
        stamp_message(self, msg.header)
        msg.header.frame_id = "left_hand"
        msg.poses = [
            Pose(
                position=Point(x=float(point[0]), y=float(point[1]), z=float(point[2])),
                orientation=Quaternion(w=1.0),
            )
            for point in self.hand_points
        ]
        return msg

    def _cameras(self) -> None:
        for index, publisher in enumerate(self.camera_pubs.values()):
            msg = CompressedImage()
            stamp_message(self, msg.header)
            msg.header.frame_id = f"synthetic_camera_{index}"
            msg.format = "jpeg"
            msg.data = self.jpeg
            publisher.publish(msg)
        self._camera_frame += 1
        self._count("camera_pair")

    def emit(self) -> None:
        self.armed_pub.publish(Bool(data=True))
        self.landmarks_pub.publish(self._landmarks())
        self.arm_command_pub.publish(self._arm_command())
        self._cameras()

    def spin_until(self, predicate, timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f"timed out waiting for {description}")

    def wait_for_graph(self) -> None:
        def connected() -> bool:
            return (
                self.landmarks_pub.get_subscription_count() > 0
                and self.arm_command_pub.get_subscription_count() > 0
                and self.collect_control_pub.get_subscription_count() > 0
                and all(pub.get_subscription_count() > 0 for pub in self.camera_pubs.values())
                and self.latest_collect_state.get("state") == "IDLE"
                and self.counts["native_hand_state"] > 0
                and self.counts["arm_state"] > 0
            )

        self.spin_until(connected, 45.0, "profile sim, collector and topic connections")

    def start_collection(self) -> None:
        self.collect_control_pub.publish(String(data="start"))
        self.spin_until(
            lambda: self.latest_collect_state.get("state") == "RECORDING",
            10.0,
            "collector RECORDING state",
        )

    def warm_up(self, duration_s: float = 1.5) -> None:
        """Settle simulated joints before recording the quality-checked segment."""
        deadline = time.monotonic() + duration_s
        next_emit = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_emit:
                self.emit()
                next_emit += 1.0 / 30.0
                if next_emit < now - 0.2:
                    next_emit = now + 1.0 / 30.0
            rclpy.spin_once(self, timeout_sec=0.005)

    def record(self) -> None:
        start_at = time.monotonic()
        next_emit = start_at
        while rclpy.ok() and time.monotonic() - start_at < self.duration_s:
            now = time.monotonic()
            if now >= next_emit:
                self.emit()
                next_emit += 1.0 / 30.0
                if next_emit < now - 0.2:
                    next_emit = now + 1.0 / 30.0
            rclpy.spin_once(self, timeout_sec=0.005)

        self.collect_control_pub.publish(String(data="stop"))
        self.spin_until(
            lambda: self.latest_collect_state.get("state") == "IDLE",
            15.0,
            "collector saved episode and returned to IDLE",
        )
        self.spin_until(lambda: self.episode.is_dir(), 5.0, "raw episode directory")

    def validate_evidence(self) -> dict:
        required = (
            "arm_command", "arm_state", "hand_command", "hand_state",
            "native_hand_state", "camera_pair",
        )
        missing = [name for name in required if self.counts[name] < 10]
        if missing:
            self.errors.append(f"insufficient live samples: {missing}")
        if self.errors:
            raise RuntimeError("; ".join(self.errors))
        meta_path = self.episode / "meta.json"
        metadata = json.loads(meta_path.read_text(encoding="utf-8"))
        if metadata.get("profile_id") != "nero_left_xhand":
            raise RuntimeError("collector did not freeze nero_left_xhand profile")
        if metadata.get("recording_mode") != "sim":
            raise RuntimeError("collector did not mark this synthetic session as sim")
        result = {
            "profile_id": metadata["profile_id"],
            "recording_mode": metadata["recording_mode"],
            "episode": str(self.episode),
            "live_topic_samples": dict(self.counts),
            "robot_data_h5": str(self.episode / "robot_data.h5"),
            "camera_data_h5": str(self.episode / "camera_data.h5"),
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=6.0)
    args = parser.parse_args()
    if args.duration < 4.0:
        parser.error("duration must be at least four seconds for offline alignment")
    target_session = args.output_root / "nero_left_xhand" / "smoke"
    if target_session.exists():
        parser.error(f"test session already exists: {target_session}")
    args.output_root.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = SimIntegrationDriver(args.output_root, args.duration)
    try:
        node.wait_for_graph()
        node.warm_up()
        node.start_collection()
        node.record()
        node.validate_evidence()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
