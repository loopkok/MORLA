#!/usr/bin/env python3
"""Single-side DexPilot node publishing standard named XHand commands."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from morla_core.ownership import PolicyLease
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from xhand_retargeting.preprocess import preprocess_xhand_landmarks


HAND_JOINT_NAMES = (
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2", "mid_joint1",
    "mid_joint2", "ring_joint1", "ring_joint2", "pinky_joint1", "pinky_joint2",
)


def urdf_values_to_hardware(joint_values, retargeter, side, index_bend_sign=None):
    """Map URDF target names to the vendor order and apply the legacy sign once."""
    side = str(side).strip().lower()
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    values = np.asarray(joint_values, dtype=float)
    names = tuple(str(name) for name in retargeter.joint_names)
    if values.shape != (len(names),) or len(set(names)) != len(names):
        raise ValueError("retargeter joint output and names do not align")
    if not np.isfinite(values).all():
        raise ValueError("retargeter produced non-finite joint values")
    prefix = f"{side}_hand_"
    expected = tuple(prefix + name for name in HAND_JOINT_NAMES)
    missing = sorted(set(expected) - set(names))
    if missing:
        raise ValueError(f"retargeter is missing configured XHand joints: {missing}")
    lut = dict(zip(names, values))
    result = np.asarray([lut[name] for name in expected], dtype=float)
    sign = (-1.0 if side == "right" else 1.0) if index_bend_sign is None else float(index_bend_sign)
    if sign not in (-1.0, 1.0):
        raise ValueError("index_bend_sign must be either -1 or 1")
    result[3] *= sign
    return result


class XHandDexRetargetingNode(Node):
    def __init__(self):
        super().__init__("xhand_dex_retargeting_node")
        self.declare_parameter("side", "left")
        self.declare_parameter("input_topic", "/hand_landmarks")
        self.declare_parameter("retarget_config_file", "")
        self.declare_parameter("description_model_id", "")
        self.declare_parameter("preprocess_mode", "legacy_xhand_pinky")
        self.declare_parameter("index_bend_sign", 1.0)
        self.declare_parameter("joint_lower_limits", [0.0])
        self.declare_parameter("joint_upper_limits", [0.0])
        self.declare_parameter("smoothing_alpha", 0.7)
        self.declare_parameter("enable_thumb_fix", True)
        self.declare_parameter("require_armed", True)
        self.declare_parameter("command_topic", "")
        self.declare_parameter("target_topic", "")

        self.side = str(self.get_parameter("side").value).strip().lower()
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.input_topic = str(self.get_parameter("input_topic").value).strip()
        self.preprocess_mode = str(self.get_parameter("preprocess_mode").value).strip()
        self.index_bend_sign = float(self.get_parameter("index_bend_sign").value)
        self.joint_lower_limits = np.asarray(
            self.get_parameter("joint_lower_limits").value, dtype=float
        )
        self.joint_upper_limits = np.asarray(
            self.get_parameter("joint_upper_limits").value, dtype=float
        )
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.enable_thumb_fix = bool(self.get_parameter("enable_thumb_fix").value)
        self.require_armed = bool(self.get_parameter("require_armed").value)
        if not 0.0 <= self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in [0, 1]")
        if (self.joint_lower_limits.shape != (12,) or self.joint_upper_limits.shape != (12,)
                or not np.isfinite(self.joint_lower_limits).all()
                or not np.isfinite(self.joint_upper_limits).all()
                or np.any(self.joint_lower_limits >= self.joint_upper_limits)):
            raise ValueError("joint limits must be finite profile-derived 12-vectors")
        if self.preprocess_mode not in ("legacy_xhand", "legacy_xhand_pinky"):
            raise ValueError("unsupported XHand preprocessing mode")

        from dex_retargeting.retargeting_config import RetargetingConfig
        config_path = self._config_path()
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["retargeting"]
        from morla_core.resource_locator import resolve_robot_asset
        model_id = str(self.get_parameter("description_model_id").value).strip() or f"xhand_{self.side}"
        config["urdf_path"] = str(resolve_robot_asset(model_id))
        self.retargeter = RetargetingConfig.from_dict(config).build()

        command_topic = str(self.get_parameter("command_topic").value).strip()
        target_topic = str(self.get_parameter("target_topic").value).strip()
        self.command_topic = command_topic or f"/{self.side}_hand/joint_commands"
        self.target_topic = target_topic or f"/{self.side}_hand/retarget_targets"
        self._command_pub = self.create_publisher(JointState, self.command_topic, 1)
        self._target_pub = self.create_publisher(JointState, self.target_topic, 1)
        self._status_pub = self.create_publisher(
            String, f"/teleop/writer_status/{self.side}_ee", 10
        )
        self._last_target = None
        self._armed = False
        self._policy_lease = PolicyLease()
        self._lease_sub, self._ack_pub = self._policy_lease.bind(
            self, writer=f"{self.side}_ee", callback=self._on_ownership
        )
        self.create_subscription(
            PoseArray, f"{self.input_topic}/{self.side}", self._on_landmarks,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bool, "/teleop/armed", self._on_armed, 10)
        self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)
        self._status_timer = self.create_timer(0.2, self._publish_runtime_status)
        self.get_logger().info(
            f"XHand DexPilot side={self.side}; input={self.input_topic}/{self.side}; "
            f"standard command={self.command_topic}; description={model_id}"
        )

    def _config_path(self) -> Path:
        supplied = str(self.get_parameter("retarget_config_file").value).strip()
        if supplied:
            path = Path(supplied)
        else:
            path = Path(get_package_share_directory("xhand_retargeting")) / "config" / (
                f"xhand_{self.side}_dexpilot.yml"
            )
        if not path.is_file():
            raise FileNotFoundError(f"XHand retarget config does not exist: {path}")
        return path

    def _on_ownership(self, msg):
        accepted = self._policy_lease.update(msg.data)
        if not self._policy_lease.manual_allowed:
            self._armed = False
        elif accepted and self._policy_lease.activity == "human":
            # Human takeover starts with new landmarks after the upstream start gate.
            self._armed = True
            self._last_target = None
        elif accepted:
            # A policy stop/release returns control to the explicit start gate.
            self._armed = False

    def _on_armed(self, msg: Bool):
        if bool(msg.data) and self._policy_lease.manual_allowed:
            self._armed = True
            self._last_target = None
        elif not msg.data:
            self._armed = False

    def _on_disarm(self, msg: Bool):
        if msg.data:
            self._armed = False

    def _publish_runtime_status(self):
        payload = {
            "writer_id": f"{self.side}_ee",
            "armed": bool(self._armed),
            "ownership": self._policy_lease.activity,
            "sent_at": time.time(),
        }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_landmarks(self, msg: PoseArray):
        activity = self._policy_lease.activity
        if not self._policy_lease.manual_allowed or activity in ("policy", "playback", "unknown"):
            return
        if self.require_armed and not self._armed:
            return
        if len(msg.poses) != 21:
            self.get_logger().warning(f"rejecting landmarks: expected 21, got {len(msg.poses)}")
            return
        points = np.asarray(
            [[pose.position.x, pose.position.y, pose.position.z] for pose in msg.poses],
            dtype=float,
        )
        if not np.isfinite(points).all():
            self.get_logger().warning("rejecting non-finite Quest landmarks")
            return
        try:
            robot_points = preprocess_xhand_landmarks(
                points, self.side, self.preprocess_mode
            )
            retargeter = self.retargeter
            retargeting_type = retargeter.optimizer.retargeting_type
            indices = retargeter.optimizer.target_link_human_indices
            if retargeting_type == "POSITION":
                references = robot_points[indices, :]
            else:
                origins, targets = indices[0, :], indices[1, :]
                references = robot_points[targets, :] - robot_points[origins, :]
            values = retargeter.retarget(references)
            values = urdf_values_to_hardware(
                values, retargeter, self.side, self.index_bend_sign
            )
            values = self._postprocess_thumb(robot_points, values)
            values = np.clip(values, self.joint_lower_limits, self.joint_upper_limits)
            if self._last_target is not None:
                values = (
                    self.smoothing_alpha * values
                    + (1.0 - self.smoothing_alpha) * self._last_target
                )
            self._last_target = values.copy()
        except (ValueError, KeyError, IndexError, RuntimeError) as exc:
            self.get_logger().error(f"retargeting rejected frame: {exc}")
            return

        now = self.get_clock().now().to_msg()
        target = JointState()
        target.header.stamp = now
        target.header.frame_id = f"{self.side}_hand"
        target.name = list(HAND_JOINT_NAMES)
        target.position = [float(value) for value in values]
        self._target_pub.publish(target)
        command = JointState()
        command.header = target.header
        command.name = target.name
        command.position = target.position
        self._command_pub.publish(command)

    def _postprocess_thumb(self, landmarks, values):
        if not self.enable_thumb_fix:
            return values
        mcp, ip, tip = landmarks[2], landmarks[3], landmarks[4]
        total = float(np.linalg.norm(tip - mcp))
        segments = float(np.linalg.norm(ip - mcp) + np.linalg.norm(tip - ip))
        straightness = total / max(segments, 0.001)
        if straightness > 0.85:
            factor = (straightness - 0.85) / 0.15 * 0.3
            values[2] *= 1.0 - factor
        return values


def main(args=None):
    rclpy.init(args=args)
    node = XHandDexRetargetingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
