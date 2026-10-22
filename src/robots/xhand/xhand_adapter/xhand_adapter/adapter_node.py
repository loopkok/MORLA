"""Per-hand standard JointState ↔ native XHand ROS adapter."""

from __future__ import annotations

import json
import time

import rclpy
from morla_core.ownership import PolicyLease, fresh_header
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from xhand_adapter.core import (
    HAND_JOINT_NAMES,
    XHandContractError,
    native_state_positions,
    reorder_named_positions,
    validate_joint_limits,
)


class XHandAdapterNode(Node):
    def __init__(self):
        super().__init__("xhand_adapter")
        self.declare_parameter("side", "left")
        self.declare_parameter("hand_id", 0)
        self.declare_parameter("state_topic", "")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("native_state_topic", "")
        self.declare_parameter("native_command_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("mode", 3)
        self.declare_parameter("kp", 80.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("effort_limit_ma", 400.0)
        self.declare_parameter("joint_lower_limits", [0.0])
        self.declare_parameter("joint_upper_limits", [0.0])
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("require_armed", True)

        self.side = str(self.get_parameter("side").value).strip().lower()
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.hand_id = int(self.get_parameter("hand_id").value)
        self.topics = {
            "state": self._topic("state_topic", f"/{self.side}_hand/joint_states"),
            "command": self._topic("command_topic", f"/{self.side}_hand/joint_commands"),
            "native_state": self._topic("native_state_topic", f"/{self.side}_hand/driver/xhand_state"),
            "native_command": self._topic("native_command_topic", f"/{self.side}_hand/driver/xhand_command"),
            "status": self._topic("status_topic", f"/{self.side}_hand/adapter/status"),
        }
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        if self.command_timeout_s <= 0:
            raise ValueError("command_timeout_s must be positive")
        self.require_armed = bool(self.get_parameter("require_armed").value)
        self._joint_lower = tuple(float(v) for v in self.get_parameter("joint_lower_limits").value)
        self._joint_upper = tuple(float(v) for v in self.get_parameter("joint_upper_limits").value)
        if len(self._joint_lower) != 12 or len(self._joint_upper) != 12:
            raise ValueError("joint_lower_limits and joint_upper_limits must come from the active 12-joint profile")
        validate_joint_limits([0.0] * 12, self._joint_lower, self._joint_upper)
        self._armed = False
        self._last_command_at = 0.0
        self._last_command_values = None
        self._last_feedback_at = 0.0
        self._rejections = 0
        self._lease = PolicyLease()
        self._ownership_sub, self._ack_pub = self._lease.bind(
            self, writer=f"{self.side}_ee", callback=self._on_ownership
        )

        from xhand_control_interfaces.msg import XHandCommand, XHandStateArray

        self._native_command_type = XHandCommand
        self._native_pub = self.create_publisher(
            XHandCommand, self.topics["native_command"], 1
        )
        # NEXUS uses best-effort state feedback; this reader also accepts the
        # original reliable publisher, avoiding back-pressure on live samples.
        native_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._standard_state_pub = self.create_publisher(JointState, self.topics["state"], 1)
        self._native_state_sub = self.create_subscription(
            XHandStateArray, self.topics["native_state"], self._on_native_state, native_qos
        )
        self._command_sub = self.create_subscription(
            JointState, self.topics["command"], self._on_standard_command, 1
        )
        self._armed_sub = self.create_subscription(Bool, "/teleop/armed", self._on_armed, 10)
        self._disarm_sub = self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)
        self._status_pub = self.create_publisher(String, self.topics["status"], 10)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"XHand adapter side={self.side} hand_id={self.hand_id}; "
            f"standard={self.topics['command']} -> native={self.topics['native_command']}"
        )

    def _topic(self, parameter, default):
        value = str(self.get_parameter(parameter).value).strip()
        return value or default

    def _on_ownership(self, msg):
        accepted = self._lease.update(msg.data)
        if accepted and self._lease.activity in ("policy", "playback"):
            # A policy epoch is its own explicit permit; manual armed state
            # cannot authorize commands while a policy owns the writer.
            self.get_logger().info(f"{self.side} hand ownership={self._lease.activity}")

    def _on_armed(self, msg: Bool):
        self._armed = bool(msg.data)

    def _on_disarm(self, msg: Bool):
        if msg.data:
            self._armed = False

    def _command_permitted(self) -> bool:
        activity = self._lease.activity
        if activity in ("policy", "playback"):
            return self._lease.fresh and not self._lease.payload.get("takeover_pending")
        if activity == "human":
            return self._lease.fresh and not self._lease.payload.get("takeover_pending")
        return self._lease.manual_allowed and (self._armed or not self.require_armed)

    def _on_standard_command(self, msg: JointState):
        if not self._feedback_fresh():
            self._reject("command rejected: fresh measured XHand feedback is required")
            return
        if not self._command_permitted():
            self._reject("command rejected: no active HUMAN arm or fresh POLICY/PLAYBACK lease")
            return
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        if not fresh_header(msg, now_ros, timeout=self.command_timeout_s):
            self._reject("command rejected: stale or future JointState header")
            return
        try:
            positions = validate_joint_limits(
                reorder_named_positions(msg.name, msg.position),
                self._joint_lower,
                self._joint_upper,
            )
            native = self._native_command_type()
            native.hand_id = self.hand_id
            native.name = list(HAND_JOINT_NAMES)
            native.position = list(positions)
            count = len(HAND_JOINT_NAMES)
            native.kp = [float(self.get_parameter("kp").value)] * count
            native.ki = [float(self.get_parameter("ki").value)] * count
            native.kd = [float(self.get_parameter("kd").value)] * count
            native.effort_limit = [float(self.get_parameter("effort_limit_ma").value)] * count
            native.mode = int(self.get_parameter("mode").value)
        except (XHandContractError, TypeError, ValueError, OverflowError) as exc:
            self._reject(f"command rejected: {exc}")
            return
        self._native_pub.publish(native)
        self._last_command_at = time.monotonic()
        self._last_command_values = list(positions)

    def _on_native_state(self, msg):
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        if not fresh_header(msg, now_ros, timeout=self.command_timeout_s):
            self._reject("feedback rejected: stale or future native state header")
            return
        try:
            positions = native_state_positions(msg, self.hand_id, self.side)
        except XHandContractError as exc:
            self._reject(f"feedback rejected: {exc}")
            return
        standard = JointState()
        standard.header = msg.header
        standard.name = list(HAND_JOINT_NAMES)
        standard.position = list(positions)
        self._standard_state_pub.publish(standard)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._last_feedback_at = time.monotonic() - (max(0.0, now_ros - stamp) if stamp else 0.0)

    def _feedback_fresh(self):
        return bool(self._last_feedback_at) and time.monotonic() - self._last_feedback_at <= self.command_timeout_s

    def _reject(self, reason):
        self._rejections += 1
        self.get_logger().warning(reason)

    def _publish_status(self):
        now = time.monotonic()
        state = {
            "side": self.side,
            "hand_id": self.hand_id,
            "armed": self._armed,
            "lease_activity": self._lease.activity,
            "lease_fresh": self._lease.fresh,
            "command_age_s": None if not self._last_command_at else max(0.0, now - self._last_command_at),
            "feedback_age_s": None if not self._last_feedback_at else max(0.0, now - self._last_feedback_at),
            "feedback_fresh": self._feedback_fresh(),
            "command_received": self._last_command_values is not None,
            "rejections": self._rejections,
            "sent_at": time.time(),
        }
        self._status_pub.publish(String(data=json.dumps(state, separators=(",", ":"))))


def main(args=None):
    rclpy.init(args=args)
    node = XHandAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
