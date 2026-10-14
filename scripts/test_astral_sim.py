#!/usr/bin/env python3
"""Drive Astral Profile simulation and collection with synthetic ROS data."""
from __future__ import annotations

import argparse
from collections import Counter
from io import BytesIO
import json
from pathlib import Path
import time

from PIL import Image
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float64, String


class AstralIntegrationDriver(Node):
    def __init__(self, output_root: Path, duration: float) -> None:
        super().__init__("morla_astral_sim_integration_driver")
        self.episode = output_root / "astral_left_gripper" / "smoke" / "episode000000"
        self.duration = duration
        self.counts: Counter[str] = Counter()
        self.collect_state: dict = {}
        self.bridge_status: dict = {}
        self.errors: list[str] = []
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
        self.wrist_pub = self.create_publisher(
            PoseStamped, "/quest3/left_wrist_pose", qos_profile_sensor_data
        )
        self.start_pub = self.create_publisher(Bool, "/teleop/start", reliable)
        self.armed_pub = self.create_publisher(Bool, "/teleop/armed", reliable)
        self.gripper_pub = self.create_publisher(Float64, "/left_gripper/command", reliable)
        self.control_pub = self.create_publisher(String, "/data_collect/control", reliable)
        self.camera_pubs = {
            name: self.create_publisher(
                CompressedImage,
                f"/quest3_video_streamer/collect/{name}",
                qos_profile_sensor_data,
            )
            for name in ("base", "left_wrist")
        }
        self.create_subscription(
            String, "/data_collect/state", self._on_collect_state, latched
        )
        self.create_subscription(
            String, "/robot/kinematics/status", self._on_bridge_status, latched
        )
        self.create_subscription(
            JointState, "/left_arm/joint_states", lambda _msg: self._count("arm_state"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState, "/left_arm/joint_commands", lambda _msg: self._count("arm_command"),
            qos_profile_sensor_data,
        )
        self.jpeg = self._make_jpeg()

    @staticmethod
    def _make_jpeg() -> bytes:
        image = Image.new("RGB", (64, 48), (50, 96, 150))
        buffer = BytesIO()
        image.save(buffer, format="JPEG", quality=85)
        return buffer.getvalue()

    def _count(self, key: str) -> None:
        self.counts[key] += 1

    def _on_collect_state(self, msg: String) -> None:
        try:
            self.collect_state = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.errors.append("collector published invalid JSON state")

    def _on_bridge_status(self, msg: String) -> None:
        try:
            self.bridge_status = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.errors.append("kinematics bridge published invalid JSON status")

    def emit(self) -> None:
        wrist = PoseStamped()
        wrist.header.stamp = self.get_clock().now().to_msg()
        wrist.header.frame_id = "quest3_operator_body_canonical"
        wrist.pose.orientation.w = 1.0
        self.wrist_pub.publish(wrist)
        self.gripper_pub.publish(Float64(data=0.5))
        self._count("gripper_command")
        for index, (name, publisher) in enumerate(self.camera_pubs.items()):
            message = CompressedImage()
            message.header.stamp = wrist.header.stamp
            message.header.frame_id = f"synthetic_{name}"
            message.format = "jpeg"
            message.data = self.jpeg
            publisher.publish(message)
            self._count(f"camera_{index}")

    def spin_until(self, predicate, timeout: float, what: str) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(f"timed out waiting for {what}")

    def wait_for_graph(self) -> None:
        def ready() -> bool:
            return (
                self.wrist_pub.get_subscription_count() > 0
                and self.start_pub.get_subscription_count() > 0
                and self.armed_pub.get_subscription_count() > 0
                and self.gripper_pub.get_subscription_count() > 0
                and self.control_pub.get_subscription_count() > 0
                and all(pub.get_subscription_count() > 0 for pub in self.camera_pubs.values())
                and self.collect_state.get("state") == "IDLE"
                and self.bridge_status
            )

        self.spin_until(ready, 45.0, "Astral profile sim, collector and state bridge")

    def record(self) -> dict:
        self.control_pub.publish(String(data="start"))
        self.spin_until(
            lambda: self.collect_state.get("state") == "RECORDING",
            10.0,
            "collector RECORDING state",
        )
        warm_until = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < warm_until:
            self.emit()
            rclpy.spin_once(self, timeout_sec=0.01)
        # Match the controller start gate: open the teleop/hand arbitration
        # gate, then capture the current wrist pose as the teleop anchor.
        self.armed_pub.publish(Bool(data=True))
        self.start_pub.publish(Bool(data=True))
        self.spin_until(lambda: self.counts["arm_command"] >= 10, 5.0, "Astral teleop commands")
        start = time.monotonic()
        next_emit = start
        while rclpy.ok() and time.monotonic() - start < self.duration:
            now = time.monotonic()
            if now >= next_emit:
                self.emit()
                next_emit += 1.0 / 30.0
                if next_emit < now - 0.2:
                    next_emit = now + 1.0 / 30.0
            rclpy.spin_once(self, timeout_sec=0.005)
        self.control_pub.publish(String(data="stop"))
        self.spin_until(
            lambda: self.collect_state.get("state") == "IDLE",
            15.0,
            "collector save and return to IDLE",
        )
        self.spin_until(lambda: self.episode.is_dir(), 5.0, "raw Astral episode")
        missing = [key for key in (
            "arm_command", "gripper_command", "arm_state", "camera_0", "camera_1",
        ) if self.counts[key] < 10]
        if missing or self.errors:
            raise RuntimeError(f"missing samples={missing}; errors={self.errors}")
        metadata = json.loads((self.episode / "meta.json").read_text(encoding="utf-8"))
        if metadata.get("profile_id") != "astral_left_gripper":
            raise RuntimeError("collector did not freeze astral_left_gripper profile")
        if metadata.get("recording_mode") != "sim":
            raise RuntimeError("collector did not mark this synthetic session as sim")
        result = {
            "profile_id": metadata["profile_id"],
            "recording_mode": metadata["recording_mode"],
            "episode": str(self.episode),
            "live_topic_samples": dict(self.counts),
            "kinematics_status": self.bridge_status,
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
    session = args.output_root / "astral_left_gripper" / "smoke"
    if session.exists():
        parser.error(f"test session already exists: {session}")
    args.output_root.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = AstralIntegrationDriver(args.output_root, args.duration)
    try:
        node.wait_for_graph()
        node.record()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
