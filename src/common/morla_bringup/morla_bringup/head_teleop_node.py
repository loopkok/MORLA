"""Right Touch controller thumbstick → Astral head yaw/pitch (absolute).

The thumbstick is spring-centred, so this uses ABSOLUTE position control
("摇杆到哪，头就到哪"), not incremental deltas:

  start signal  → capture the robot's current head angle as ``head_init``
                  (from /head/joint_states, or a fallback param in sim)
  armed         → head_target = head_init + stick * scale, published at 50 Hz
  stick centred → head returns to head_init (spring return → back to initial)
  disarm        → stop commanding; driver holds the last head position

Joy layout (morla_input): axes=[trigger, grip, stickX, stickY].
stickX: left=-1 right=+1 → yaw;  stickY: forward=+1 back=-1 → pitch.

Non-intrusive: only subscribes existing topics and publishes to the existing
``/head/joint_commands`` that astral_robot_control already consumes (driver is
the hardware authority). No changes to driver / arm teleop / gripper / hand.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool
from morla_data_collect.ownership import PolicyLease, fresh_header


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class HeadTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("head_teleop_node")

        self.declare_parameter("joy_topic", "quest3/right_controller_joy")
        self.declare_parameter("head_state_topic", "/head/joint_states")
        self.declare_parameter("head_command_topic", "/head/joint_commands")
        self.declare_parameter("start_topic", "/teleop/start")
        self.declare_parameter("disarm_topic", "/teleop/disarm")
        # Stick axes in Joy (see morla_input): stickX=2, stickY=3.
        self.declare_parameter("yaw_axis", 2)
        self.declare_parameter("pitch_axis", 3)
        # Signed rad-per-unit-stick; flip the sign if a direction is reversed
        # on the real head motors.
        self.declare_parameter("yaw_scale", 0.8)
        self.declare_parameter("pitch_scale", 0.4)
        # Absolute clamps on the commanded head angle (rad).
        self.declare_parameter("yaw_min", -0.9)
        self.declare_parameter("yaw_max", 0.9)
        self.declare_parameter("pitch_min", -0.5)
        self.declare_parameter("pitch_max", 0.5)
        self.declare_parameter("stick_deadzone", 0.05)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("input_timeout_s", 0.4)
        self.declare_parameter("require_start_signal", True)
        # Fallback initial head angle when /head/joint_states is unavailable
        # (e.g. sim, which has no head joint).
        self.declare_parameter("init_head_yaw", 0.0)
        self.declare_parameter("init_head_pitch", 0.0)

        gp = self.get_parameter
        self.joy_topic = str(gp("joy_topic").value).strip()
        self.head_state_topic = str(gp("head_state_topic").value).strip()
        self.head_command_topic = str(gp("head_command_topic").value).strip()
        self.yaw_axis = int(gp("yaw_axis").value)
        self.pitch_axis = int(gp("pitch_axis").value)
        self.yaw_scale = float(gp("yaw_scale").value)
        self.pitch_scale = float(gp("pitch_scale").value)
        self.yaw_min = float(gp("yaw_min").value)
        self.yaw_max = float(gp("yaw_max").value)
        self.pitch_min = float(gp("pitch_min").value)
        self.pitch_max = float(gp("pitch_max").value)
        self.deadzone = float(gp("stick_deadzone").value)
        self.input_timeout_s = float(gp("input_timeout_s").value)
        self._require_start = bool(gp("require_start_signal").value)
        self._init_fallback = (
            float(gp("init_head_yaw").value),
            float(gp("init_head_pitch").value),
        )

        qos = _sensor_qos()
        self._pub_cmd = self.create_publisher(JointState, self.head_command_topic, qos)
        # BEST_EFFORT matches the mocap/driver sensor streams. A RELIABLE
        # publisher (mocap Joy, CLI `ros2 topic pub`) is compatible with a
        # BEST_EFFORT subscription, so no dual-sub is needed; CLI echo of the
        # BEST_EFFORT state stream needs `--qos-reliability best_effort`.
        self.create_subscription(Joy, self.joy_topic, self._on_joy, qos)
        self.create_subscription(
            JointState, self.head_state_topic, self._on_head_state, qos
        )
        self.create_subscription(
            Bool, str(gp("start_topic").value), self._on_start, 10
        )
        self.create_subscription(
            Bool, str(gp("disarm_topic").value), self._on_disarm, 10
        )

        self._policy_lease = PolicyLease()
        self._policy_lease.bind(self, 'head', self._on_policy_state)
        self._armed = not self._require_start
        self._human_wait_for_joy = False
        self._head_init = list(self._init_fallback)  # [yaw, pitch]
        self._have_state = False
        self._head_state_t = 0.0
        self._last_joy_t = 0.0
        self._stick_yaw = 0.0   # latest stick deflection (-1..1)
        self._stick_pitch = 0.0

        rate = float(gp("publish_rate").value)
        self.create_timer(1.0 / max(1.0, rate), self._on_timer)

        self.get_logger().info(
            f"head_teleop joy={self.joy_topic} state={self.head_state_topic} "
            f"cmd={self.head_command_topic} yaw(axis{self.yaw_scale}x{self.yaw_axis}) "
            f"pitch(axis{self.pitch_scale}x{self.pitch_axis}) "
            f"require_start={self._require_start}"
        )
        if self._require_start:
            self.get_logger().warn(
                "require_start_signal=true: waiting for /teleop/start to capture "
                "head_init and arm head control (same gate as arm teleop)."
            )

    def _on_head_state(self, msg: JointState) -> None:
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        try:
            d = dict(zip(msg.name, msg.position))
            yaw = float(d.get("head_yaw", msg.position[0]))
            pitch = float(d.get("head_pitch", msg.position[1] if len(msg.position) > 1 else 0.0))
        except (ValueError, IndexError):
            return
        # Track the live head angle only while disarmed, so that on start we
        # capture the pre-start pose. After start, freeze head_init (otherwise
        # our own commands would drag the reference and the stick would drift).
        lease = getattr(self, '_policy_lease', None)
        if not self._armed or (lease is not None and not lease.manual_allowed):
            self._head_init[0] = yaw
            self._head_init[1] = pitch
        self._have_state = True
        self._head_state_t = time.monotonic()

    def _on_policy_state(self, msg) -> None:
        accepted = self._policy_lease.update(msg.data)
        if not self._policy_lease.manual_allowed:
            self._armed = False
            self._human_wait_for_joy = False
            return
        if not accepted:
            return
        if self._policy_lease.payload.get("activity") == "human":
            # Policy disarm has already frozen this writer. Wait for a new
            # controller sample after the HUMAN commit before resuming head IO.
            self._armed = False
            self._human_wait_for_joy = True
        else:
            self._human_wait_for_joy = False

    def _on_joy(self, msg: Joy) -> None:
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        ax = list(msg.axes) if msg.axes else []
        if max(self.yaw_axis, self.pitch_axis) >= len(ax):
            return
        self._stick_yaw = self._dz(float(ax[self.yaw_axis]))
        self._stick_pitch = self._dz(float(ax[self.pitch_axis]))
        self._last_joy_t = time.monotonic()
        if (self._human_wait_for_joy and self._policy_lease.manual_allowed
                and self._have_state and self._last_joy_t - self._head_state_t <= 0.5):
            self._armed = True
            self._human_wait_for_joy = False

    def _dz(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else max(-1.0, min(1.0, v))

    def _on_start(self, msg: Bool) -> None:
        if not msg.data or not self._policy_lease.manual_allowed:
            return
        # Capture head_init from the current head angle (already tracked from
        # /head/joint_states); fall back to the configured init if none seen.
        if not self._have_state:
            self._head_init = list(self._init_fallback)
            self.get_logger().warn(
                "start: no /head/joint_states yet; using init_head_* fallback "
                f"({self._head_init[0]:.2f},{self._head_init[1]:.2f})"
            )
        self._armed = True
        self.get_logger().info(
            f"start: head_init yaw={self._head_init[0]:.3f} "
            f"pitch={self._head_init[1]:.3f}; head control armed"
        )

    def _on_disarm(self, msg: Bool) -> None:
        # Level signal: only Bool(true) disarms (web pause = True, policy
        # re-opens with False on IDLE/HUMAN). A False must not disarm us.
        if not msg.data:
            return
        self._armed = False
        self._human_wait_for_joy = False
        self.get_logger().info("disarm: head control stopped (driver holds)")

    def _on_timer(self) -> None:
        if not self._policy_lease.manual_allowed:
            self._armed = False
            self._human_wait_for_joy = False
            return
        if not self._armed:
            return
        now = time.monotonic()
        joy_fresh = (
            self._last_joy_t > 0.0
            and (now - self._last_joy_t) <= self.input_timeout_s
        )
        if not joy_fresh:
            return  # hold last commanded head; do not snap on controller drop

        yaw = self._head_init[0] + self._stick_yaw * self.yaw_scale
        pitch = self._head_init[1] + self._stick_pitch * self.pitch_scale
        yaw = max(self.yaw_min, min(self.yaw_max, yaw))
        pitch = max(self.pitch_min, min(self.pitch_max, pitch))

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["head_yaw", "head_pitch"]
        js.position = [float(yaw), float(pitch)]
        self._pub_cmd.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
