"""Forward measured component states to robot_state_publisher's joint_states."""
from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .profile_state import remap_joint_state_fields


class ProfileJointStateBridge(Node):
    """Map named measured state to URDF names without synthesizing feedback."""

    def __init__(self) -> None:
        super().__init__("profile_joint_state_bridge")
        self.declare_parameter("profile_id", "")
        self.declare_parameter("source_specs_json", "[]")
        self.declare_parameter("kinematics_hash", "")
        self.declare_parameter("coordinate_contract_version", 0)
        self.declare_parameter("robot_body_frame", "")
        self.declare_parameter("state_timeout_s", 1.0)
        self.profile_id = str(self.get_parameter("profile_id").value)
        self.kinematics_hash = str(self.get_parameter("kinematics_hash").value)
        self.coordinate_contract_version = int(
            self.get_parameter("coordinate_contract_version").value
        )
        self.robot_body_frame = str(self.get_parameter("robot_body_frame").value)
        self.state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        if not math.isfinite(self.state_timeout_s) or self.state_timeout_s <= 0.0:
            raise ValueError("state_timeout_s must be finite and positive")
        try:
            specs = json.loads(str(self.get_parameter("source_specs_json").value))
        except json.JSONDecodeError as exc:
            raise ValueError("source_specs_json must be valid JSON") from exc
        if not isinstance(specs, list) or not specs:
            raise ValueError("source_specs_json must contain at least one measured state source")

        self._specs: dict[str, dict[str, Any]] = {}
        self._last_seen: dict[str, float] = {}
        self._last_error: dict[str, str] = {}
        self._subscriptions = []
        self._joint_pub = self.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(String, "/robot/kinematics/status", status_qos)
        for spec in specs:
            if not isinstance(spec, dict):
                raise ValueError("each state source must be a mapping")
            component_id = str(spec.get("component_id", ""))
            topic = str(spec.get("topic", ""))
            source_names = spec.get("source_joint_names")
            urdf_names = spec.get("urdf_joint_names")
            if (not component_id or component_id in self._specs or not topic.endswith("/joint_states")
                    or not isinstance(source_names, list) or not source_names
                    or not isinstance(urdf_names, list) or len(source_names) != len(urdf_names)):
                raise ValueError(f"invalid joint-state source declaration: {spec!r}")
            self._specs[component_id] = spec
            self._subscriptions.append(self.create_subscription(
                JointState,
                topic,
                lambda msg, source=spec: self._on_state(msg, source),
                qos_profile_sensor_data,
            ))
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"profile joint-state bridge ready: profile={self.profile_id} "
            f"sources={len(self._specs)} contract={self.kinematics_hash[:12]} "
            "(measured state passthrough; commands are never used)"
        )

    def _on_state(self, msg: JointState, spec: dict[str, Any]) -> None:
        component_id = str(spec["component_id"])
        try:
            values = remap_joint_state_fields(
                list(msg.name), list(msg.position), list(msg.velocity), list(msg.effort),
                list(spec["source_joint_names"]), list(spec["urdf_joint_names"]),
            )
            if not all(math.isfinite(float(value)) for value in values["position"]):
                raise ValueError("JointState positions contain non-finite values")
            if any(not math.isfinite(float(value)) for value in values["velocity"]):
                raise ValueError("JointState velocities contain non-finite values")
            if any(not math.isfinite(float(value)) for value in values["effort"]):
                raise ValueError("JointState efforts contain non-finite values")
            output = JointState()
            output.header = msg.header  # retain source timestamp and frame semantics
            output.name = values["name"]
            output.position = [float(value) for value in values["position"]]
            output.velocity = [float(value) for value in values["velocity"]]
            output.effort = [float(value) for value in values["effort"]]
            self._joint_pub.publish(output)
            self._last_seen[component_id] = time.monotonic()
            self._last_error.pop(component_id, None)
        except (TypeError, ValueError) as exc:
            self._last_error[component_id] = str(exc)
            self.get_logger().warning(
                f"dropping invalid measured state from {component_id}: {exc}",
                throttle_duration_sec=2.0,
            )

    def _publish_status(self) -> None:
        now = time.monotonic()
        sources = {}
        for component_id in self._specs:
            seen = self._last_seen.get(component_id)
            age = None if seen is None else max(0.0, now - seen)
            sources[component_id] = {
                "state": "not_received" if seen is None else (
                    "fresh" if age <= self.state_timeout_s else "stale"
                ),
                "age_s": age,
                "last_error": self._last_error.get(component_id),
            }
        self._status_pub.publish(String(data=json.dumps({
            "profile_id": self.profile_id,
            "kinematics_hash": self.kinematics_hash,
            "coordinate_contract_version": self.coordinate_contract_version,
            "robot_body_frame": self.robot_body_frame,
            "sources": sources,
            "source_semantics": "measured_joint_state_passthrough",
            "sent_at": time.time(),
        }, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ProfileJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
