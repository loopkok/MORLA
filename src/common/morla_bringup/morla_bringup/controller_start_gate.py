"""Touch controller button → teleop external start gate (inference-aware).

Subscribes to a ``sensor_msgs/Joy`` topic (default the left Touch controller)
and, on the rising edge of a chosen button, either:

* ``/teleop/start`` + ``/teleop/armed`` (Bool true) — the normal one-shot that
  astral_arm_teleop consumes (``require_start_signal:=true``) to capture
  ``vr_init`` and arm; or
* ``/policy_inference/cmd`` = "takeover" — when the policy node is active
  (``/policy_inference/state`` activity ∈ {policy, playback}), so grip becomes
  the HITL takeover trigger instead of arming teleop (which would race the
  policy writing ``joint_commands``).

Default button index 5 = grip click (middle finger). Re-pressing re-captures
the zero (re-center), matching the web "开始遥操" button and the CLI
``ros2 topic pub --once /teleop/start``.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from morla_bringup.start_gate_logic import (
    RestartRequiredGate,
    decide_release_action,
    decide_start_action,
)
from morla_core.ownership import PolicyLease, fresh_header


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ControllerStartGate(Node):
    def __init__(self) -> None:
        super().__init__("controller_start_gate")

        self.declare_parameter("joy_topic", "quest3/left_controller_joy")
        self.declare_parameter("button_index", 5)
        self.declare_parameter("start_topic", "/teleop/start")
        # 工作位/HOME/暂停发过 latched /teleop/disarm 会把夹爪 pinch 仲裁门
        # 关死，门只在 /teleop/armed=true 时重开——用本闸门（gripClick）启动
        # 遥操时必须一并发 armed=true，否则手柄对夹爪永远无响应。臂节点未
        # 校准会忽略 armed（无害），真正 arm 由 /teleop/start 完成。
        self.declare_parameter("armed_topic", "/teleop/armed")
        # 推理感知：策略活跃（policy/playback）时 grip 改为 HITL 接管。
        self.declare_parameter("policy_state_topic", "/policy_inference/state")
        self.declare_parameter("policy_cmd_topic", "/policy_inference/cmd")
        # 右手 A 在推理 HUMAN 时 = 释放（本门随遥操栈常驻，HITL 释放不依赖数采栈）。
        self.declare_parameter("right_joy_topic", "quest3/right_controller_joy")
        self.declare_parameter("input_epoch_topic", "quest3/input_epoch")
        self.declare_parameter("disarm_topic", "/teleop/disarm")

        joy_topic = str(self.get_parameter("joy_topic").value).strip()
        self.button_index = int(self.get_parameter("button_index").value)
        start_topic = str(self.get_parameter("start_topic").value).strip()
        armed_topic = str(self.get_parameter("armed_topic").value).strip()
        state_topic = str(self.get_parameter("policy_state_topic").value).strip()
        self._policy_cmd_topic = str(self.get_parameter("policy_cmd_topic").value).strip()
        right_joy_topic = str(self.get_parameter("right_joy_topic").value).strip()
        epoch_topic = str(self.get_parameter("input_epoch_topic").value).strip()
        disarm_topic = str(self.get_parameter("disarm_topic").value).strip()

        # RELIABLE + VOLATILE: one-shot, not latched (match monitor_node).
        self._pub = self.create_publisher(Bool, start_topic, 10)
        self._pub_armed = self.create_publisher(Bool, armed_topic, 10)
        self._policy_cmd_pub = self.create_publisher(String, self._policy_cmd_topic, 10)
        self._epoch_disarm_pub = self.create_publisher(Bool, disarm_topic, _latched_qos())
        # BEST_EFFORT sub is compatible with the mocap Joy publisher (RELIABLE
        # default) and CLI `ros2 topic pub` — no dual-sub needed.
        self.create_subscription(Joy, joy_topic, self._on_joy, _sensor_qos())
        self._prev_right_buttons: list[int] = [0] * 6
        self.create_subscription(
            Joy, right_joy_topic, self._on_right_joy, _sensor_qos()
        )
        # 策略节点状态（latched，新订阅者立即拿到当前值）；无人发布=None。
        self._activity: str | None = None
        self._policy_lease = PolicyLease()
        self._last_left_joy = self._last_right_joy = 0.0
        self._policy_state_topic = state_topic
        self.create_subscription(String, state_topic, self._on_policy_state, _latched_qos())
        self._prev_pressed = False
        self._restart_gate = RestartRequiredGate()
        self.create_subscription(String, epoch_topic, self._on_input_epoch, _latched_qos())

        self.get_logger().info(
            f"controller_start_gate joy={joy_topic} button={self.button_index} "
            f"(5=gripClick) → {start_topic} + {armed_topic} "
            f"(policy active → {self._policy_cmd_topic} 'takeover'; "
            f"right A in HUMAN → 'release')"
        )

    def _on_policy_state(self, msg: String) -> None:
        self._policy_lease.update(msg.data)
        self._activity = self._policy_lease.activity

    def _on_input_epoch(self, msg: String) -> None:
        """A new source session/frame stops writers until a fresh grip press."""
        self._restart_gate.reset()
        self._prev_pressed = False
        self._last_left_joy = 0.0
        # Stop policy/playback as well as human teleop so no writer can continue
        # using targets derived from the invalidated frame/session.
        self._policy_cmd_pub.publish(String(data="stop"))
        self._epoch_disarm_pub.publish(Bool(data=True))
        self._pub_armed.publish(Bool(data=False))
        self.get_logger().warning(
            f"input epoch changed ({msg.data}); stopped active writers; "
            "release then press grip to restart"
        )

    def _on_joy(self, msg: Joy) -> None:
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        btns = list(msg.buttons) if msg.buttons else []
        if self.button_index < 0 or self.button_index >= len(btns):
            return
        pressed = bool(btns[self.button_index])
        if not self._restart_gate.observe(pressed):
            self._prev_pressed = pressed
            self._last_left_joy = time.monotonic()
            return
        now = time.monotonic()
        if self._last_left_joy and now - self._last_left_joy > 1.5:
            self._prev_pressed = pressed
        self._last_left_joy = now
        lease = self._policy_lease
        if ((lease.seen and not lease.fresh and not lease.manual_allowed) or
                (not lease.seen and self.count_publishers(self._policy_state_topic))):
            self._prev_pressed = pressed
            return
        self._activity = lease.payload.get('activity') if lease.seen else None
        if lease.payload.get('takeover_pending') or lease.payload.get('policy_start_pending'):
            self._prev_pressed = pressed
            return
        # Rising edge only — hold does not spam start.
        if pressed and not self._prev_pressed:
            if decide_start_action(self._activity) == "takeover":
                self._policy_cmd_pub.publish(String(data="takeover"))
                self.get_logger().info(
                    f"controller button {self.button_index} pressed → "
                    f"{self._policy_cmd_topic} 'takeover' (policy active)"
                )
            else:
                self._pub.publish(Bool(data=True))
                self._pub_armed.publish(Bool(data=True))
                self.get_logger().info(
                    f"controller button {self.button_index} pressed → "
                    f"{self._pub.topic} + {self._pub_armed.topic}"
                )
        self._prev_pressed = pressed

    def _on_right_joy(self, msg: Joy) -> None:
        """右手 A 在推理 HUMAN 时 → /policy_inference/cmd 'release'。"""
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        cur = list(msg.buttons) if msg.buttons else []
        cur += [0] * (6 - len(cur))
        now = time.monotonic()
        if self._last_right_joy and now - self._last_right_joy > 1.5:
            self._prev_right_buttons = cur
        self._last_right_joy = now
        self._activity = self._policy_lease.activity
        if decide_release_action(self._activity, self._prev_right_buttons, cur):
            self._policy_cmd_pub.publish(String(data="release"))
            self.get_logger().info(
                f"right A pressed → {self._policy_cmd_topic} 'release' (HUMAN)"
            )
        self._prev_right_buttons = cur


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerStartGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
