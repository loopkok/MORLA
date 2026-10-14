#!/usr/bin/env python3
"""Quest3 hand_landmarks → /{side}_gripper/command (Float64: 0=open, 1=closed).

Single command stream on purpose: the node publishes ONLY the unitless close
ratio. The rad mapping (open_rad/closed_rad, CMD 0x97/0x98) lives solely in
astral_robot_control's driver config — the hardware authority. Never publish
JointState on /{side}_gripper/joint_commands as well: the driver subscribes to
both, and two interleaved streams with different rad values make the gripper
oscillate (抽搐) at control rate.

Swap this node later for a hardware adapter that publishes the same topic.
"""

from __future__ import annotations

import json
import time
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from morla_core.ownership import PolicyLease, fresh_header

from astral_gripper_teleop.pinch import (
    close_ratio_from_range,
    pinch_distance_m,
)


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class PinchGripperNode(Node):
    def __init__(self) -> None:
        super().__init__("pinch_gripper_node")

        self.declare_parameter("hand_side", "left")
        self.declare_parameter("landmark_topic", "")
        self.declare_parameter("command_topic", "")
        # Pinch distances in the same units as landmarks (Quest wrist-local meters).
        self.declare_parameter("open_dist_m", 0.08)
        self.declare_parameter("close_dist_m", 0.015)
        self.declare_parameter("ema_alpha", 0.4)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("input_timeout_s", 0.4)
        # hold = keep last ratio; open = force 0 on timeout
        self.declare_parameter("on_timeout", "hold")
        self.declare_parameter("log_interval_s", 0.0)
        # Auto-range: track the user's real pinch-distance envelope so the
        # gripper uses the full stroke even if open_dist_m/close_dist_m don't
        # match the actual hand range. open_dist_m/close_dist_m become priors.
        self.declare_parameter("auto_range", True)
        self.declare_parameter("auto_range_forget_s", 8.0)
        self.declare_parameter("auto_range_min_span_m", 0.01)
        # Touch controller trigger (analog) as an alternative gripper source.
        # When the controller Joy is fresh, trigger (axes[trigger_axis], 0=open
        # .. 1=pressed) drives the gripper; otherwise falls back to pinch. Same
        # Quest side only sends controller OR hand, so the two are complementary.
        self.declare_parameter("controller_joy_topic", "")
        self.declare_parameter("trigger_axis", 0)
        self.declare_parameter("trigger_deadzone", 0.05)
        self.declare_parameter("trigger_invert", False)
        # Trigger shaping: deadzone rescale → gamma → EMA. gamma > 1 gives
        # finer control at the start of the stroke (easier half-grasp).
        self.declare_parameter("trigger_gamma", 1.4)
        self.declare_parameter("trigger_ema_alpha", 0.4)
        # Slew limit on the merged published ratio (ratio/s); <=0 disables.
        # The gripper servo executes every absolute target at max speed, so
        # without this a fast trigger pull slams it shut despite the pipeline
        # being proportional end to end.
        self.declare_parameter("max_ratio_rate", 2.5)
        # Optional arbitration gate: when an external owner (e.g. policy
        # inference) publishes Bool true on this topic, this node stops writing
        # its ratio command topic (last-writer-wins collision). Empty = always
        # enabled (legacy behaviour).
        self.declare_parameter("disarm_topic", "/teleop/disarm")
        # Web monitor resume re-arms teleop by publishing Bool true on
        # /teleop/armed (it never publishes disarm=False), so an operator
        # pause→resume would otherwise leave this gate shut for the rest of
        # the node's life. Treat a fresh armed=true as "gate open" too.
        self.declare_parameter("arm_topic", "/teleop/armed")

        side = str(self.get_parameter("hand_side").value).strip().lower()
        if side not in ("left", "right"):
            raise ValueError(f"hand_side 须为 left|right，收到: {side}")
        self.side = side

        landmark_topic = str(self.get_parameter("landmark_topic").value).strip()
        command_topic = str(self.get_parameter("command_topic").value).strip()
        if not landmark_topic:
            landmark_topic = f"hand_landmarks/{side}"
        if not command_topic:
            command_topic = f"/{side}_gripper/command"

        self.landmark_topic = landmark_topic
        self.command_topic = command_topic
        self.open_dist = float(self.get_parameter("open_dist_m").value)
        self.close_dist = float(self.get_parameter("close_dist_m").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.on_timeout = str(self.get_parameter("on_timeout").value).strip().lower()
        self.log_interval_s = float(self.get_parameter("log_interval_s").value)
        self.auto_range = bool(self.get_parameter("auto_range").value)
        self._env_tau = float(self.get_parameter("auto_range_forget_s").value)
        self._env_min_span = float(self.get_parameter("auto_range_min_span_m").value)
        rate = float(self.get_parameter("publish_rate").value)

        if self.open_dist <= self.close_dist:
            raise ValueError("open_dist_m 必须大于 close_dist_m")

        qos = _sensor_qos()
        self._pub_cmd = self.create_publisher(Float64, self.command_topic, qos)
        self._status_pub = self.create_publisher(
            String, f"/teleop/writer_status/{self.side}_gripper", 10
        )
        self.create_subscription(
            PoseArray, self.landmark_topic, self._on_landmarks, qos
        )

        # Optional Touch controller trigger → gripper (merged with pinch).
        self.controller_joy_topic = str(
            self.get_parameter("controller_joy_topic").value
        ).strip()
        self.trigger_axis = int(self.get_parameter("trigger_axis").value)
        self.trigger_deadzone = float(self.get_parameter("trigger_deadzone").value)
        self.trigger_invert = bool(self.get_parameter("trigger_invert").value)
        self.trigger_gamma = max(
            0.05, float(self.get_parameter("trigger_gamma").value)
        )
        self.trigger_ema_alpha = float(self.get_parameter("trigger_ema_alpha").value)
        self.max_ratio_rate = float(self.get_parameter("max_ratio_rate").value)
        self._trigger_ratio = 0.0
        self._trigger_ema_ready = False
        self._last_joy_t = 0.0
        self._ratio_cmd = 0.0  # slew-limited published value
        self._last_pub_t = 0.0
        if self.controller_joy_topic:
            from sensor_msgs.msg import Joy
            self.create_subscription(
                Joy, self.controller_joy_topic, self._on_joy, qos
            )

        self._ratio: float = 0.0
        self._ema_ready = False
        self._last_lm_t = 0.0
        self._last_dist = 0.0
        self._last_log = time.monotonic()
        # Auto-range envelope (observed min/max pinch distance).
        self._emin: float | None = None
        self._emax: float | None = None
        self._last_env_t = 0.0

        self._gripper_disarmed = False
        self._policy_lease = PolicyLease()
        self._external_ratio = None
        self._soft_takeover = False
        self.declare_parameter('soft_takeover_tolerance', 0.05)
        self._takeover_tolerance = float(self.get_parameter('soft_takeover_tolerance').value)
        self.create_subscription(Float64, self.command_topic, self._on_executed_ratio, qos)
        self._policy_lease.bind(self, f'{self.side}_gripper', self._on_policy_state)
        disarm_topic = str(self.get_parameter("disarm_topic").value).strip()
        arm_topic = str(self.get_parameter("arm_topic").value).strip()
        if disarm_topic:
            from std_msgs.msg import Bool
            self.create_subscription(Bool, disarm_topic, self._on_disarm, 10)
        if arm_topic:
            from std_msgs.msg import Bool
            self.create_subscription(Bool, arm_topic, self._on_arm, 10)
        if disarm_topic or arm_topic:
            self.get_logger().info(
                f"gripper arbitration gate: disarm={disarm_topic or 'off'} "
                f"arm={arm_topic or 'off'}"
            )

        self.create_timer(1.0 / max(1.0, rate), self._on_timer)
        self._status_timer = self.create_timer(0.2, self._publish_runtime_status)
        self.get_logger().info(
            f"pinch→gripper[{side}] landmarks={self.landmark_topic} "
            f"cmd={self.command_topic} "
            f"pinch=[{self.close_dist:.3f},{self.open_dist:.3f}]m "
            f"auto_range={self.auto_range} forget={self._env_tau:.1f}s "
            f"joy={self.controller_joy_topic or 'off'} axis={self.trigger_axis} "
            f"gamma={self.trigger_gamma:.2f} trig_ema={self.trigger_ema_alpha:.2f} "
            f"rate={self.max_ratio_rate:.2f}/s "
            f"(ratio only; rad mapping in driver config)"
        )

    def _on_landmarks(self, msg: PoseArray) -> None:
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        if len(msg.poses) < 9:
            return
        pts = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in msg.poses],
            dtype=np.float64,
        )
        if not np.isfinite(pts[:9]).all():
            return
        if float(np.linalg.norm(pts[4])) < 1e-6 and float(np.linalg.norm(pts[8])) < 1e-6:
            return
        dist = pinch_distance_m(pts)
        now = time.monotonic()
        self._update_envelope(dist, now)
        lo = self._emin if self._emin is not None else self.close_dist
        hi = self._emax if self._emax is not None else self.open_dist
        raw = close_ratio_from_range(dist, lo, hi, self._env_min_span)
        if not self._ema_ready or self.ema_alpha >= 1.0:
            self._ratio = raw
            self._ema_ready = True
        else:
            a = max(0.0, min(1.0, self.ema_alpha))
            self._ratio = a * raw + (1.0 - a) * self._ratio
        self._last_dist = dist
        self._last_lm_t = now

    def _on_joy(self, msg) -> None:
        """Touch controller trigger (analog) → close ratio in [0,1].

        axes[trigger_axis]: 0 = released (open), 1 = fully pressed (closed).
        Shaping chain: deadzone + rescale (travel above the deadzone spans the
        full 0..1 stroke) → gamma curve → EMA against Joy jitter.
        ``trigger_invert`` flips polarity.
        """
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        ax = list(msg.axes) if msg.axes else []
        if self.trigger_axis < 0 or self.trigger_axis >= len(ax):
            return
        t = float(ax[self.trigger_axis])
        if self.trigger_invert:
            t = 1.0 - t
        t = (t - self.trigger_deadzone) / max(1e-3, 1.0 - self.trigger_deadzone)
        t = max(0.0, min(1.0, t))
        if self.trigger_gamma != 1.0:
            t = t ** self.trigger_gamma
        if not self._trigger_ema_ready or self.trigger_ema_alpha >= 1.0:
            self._trigger_ratio = t
            self._trigger_ema_ready = True
        else:
            a = max(0.0, min(1.0, self.trigger_ema_alpha))
            self._trigger_ratio = a * t + (1.0 - a) * self._trigger_ratio
        self._last_joy_t = time.monotonic()

    def _update_envelope(self, dist: float, now: float) -> None:
        """Track observed min/max pinch distance with exponential forget.

        When auto_range is off, falls back to the configured close/open dist.
        The envelope slowly leaks toward the current distance so stale
        extremes (an old deep pinch or wide open) fade over ``auto_range_forget_s``;
        a fresh extreme snaps immediately. A ``min_span`` floor keeps the
        denominator sane when the hand is still.
        """
        if not self.auto_range:
            self._emin = self.close_dist
            self._emax = self.open_dist
            return
        if self._emin is None or self._last_env_t <= 0.0:
            self._emin = dist
            self._emax = dist
            self._last_env_t = now
            return
        dt = max(1e-3, now - self._last_env_t)
        self._last_env_t = now
        k = 1.0 - math.exp(-dt / max(1e-3, self._env_tau))
        if dist < self._emin:
            self._emin = dist
        else:
            self._emin += k * (dist - self._emin)
        if dist > self._emax:
            self._emax = dist
        else:
            self._emax += k * (dist - self._emax)
        span = self._emax - self._emin
        if span < self._env_min_span:
            mid = 0.5 * (self._emax + self._emin)
            half = 0.5 * self._env_min_span
            self._emin = mid - half
            self._emax = mid + half

    def _set_gate(self, disarmed: bool) -> None:
        """仲裁门状态切换：只在状态**变化**时打一条日志（进/出 disarm 各一次），
        不再按固定周期刷屏——HOME/暂停期间门长时间关着，1s 一条的重复提示
        只污染日志。进入 disarm 打一条说明原因方向，重开（armed/门开放）打一条。"""
        if disarmed == self._gripper_disarmed:
            return
        self._gripper_disarmed = disarmed
        if disarmed:
            self.get_logger().info(
                f"[gripper {self.side}] disarmed by arbitration gate — "
                "not publishing (waiting for open)"
            )
        else:
            external = getattr(self, '_external_ratio', None)
            if external is not None:
                self._ratio_cmd = external
                self._last_pub_t = time.monotonic()
                self._soft_takeover = True
            # Require input observed after the gate opens; cached Joy/landmark
            # samples from the policy-owned interval must not trigger motion.
            self._last_lm_t = 0.0
            self._last_joy_t = 0.0
            self.get_logger().info(
                f"[gripper {self.side}] arbitration gate open — publishing resumed"
            )

    def _on_disarm(self, msg) -> None:
        """Arbitration gate: True = an external owner (policy / web pause)
        commands the gripper topic; stop publishing until the gate opens."""
        self._set_gate(bool(msg.data))

    def _on_arm(self, msg) -> None:
        """Web monitor resume publishes /teleop/armed=true (never disarm=false);
        treat a fresh arm signal as the gate opening."""
        lease = getattr(self, '_policy_lease', None)
        if msg.data and (lease is None or lease.manual_allowed):
            self._set_gate(False)

    def _on_policy_state(self, msg):
        self._policy_lease.update(msg.data)
        if not self._policy_lease.manual_allowed:
            self._set_gate(True)

    def _publish_runtime_status(self) -> None:
        payload = {
            "writer_id": f"{self.side}_gripper",
            "armed": bool(not self._gripper_disarmed and self._policy_lease.manual_allowed),
            "ownership": self._policy_lease.activity,
            "sent_at": time.time(),
        }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _on_executed_ratio(self, msg):
        if self._gripper_disarmed or not self._policy_lease.manual_allowed:
            value = float(msg.data)
            if math.isfinite(value):
                self._external_ratio = max(0.0, min(1.0, value))

    def _on_timer(self) -> None:
        now = time.monotonic()
        lease = getattr(self, '_policy_lease', None)
        if self._gripper_disarmed or (lease is not None and not lease.manual_allowed):
            return
        pinch_stale = (
            self._last_lm_t <= 0.0
            or (now - self._last_lm_t) > self.input_timeout_s
        )
        # Pinch contribution per on_timeout policy.
        if pinch_stale:
            if self.on_timeout == "open":
                pinch_ratio: float | None = 0.0
            elif self.on_timeout == "hold":
                pinch_ratio = self._ratio
            else:
                pinch_ratio = None
        else:
            pinch_ratio = self._ratio

        # Controller trigger takes priority when its Joy is fresh; falls back
        # to pinch when the controller is absent/stale (bare-hand mode).
        joy_fresh = (
            self.controller_joy_topic != ""
            and self._last_joy_t > 0.0
            and (now - self._last_joy_t) <= self.input_timeout_s
        )
        if joy_fresh:
            ratio = self._trigger_ratio
        elif pinch_ratio is not None:
            ratio = pinch_ratio
        else:
            return  # no input yet

        ratio = max(0.0, min(1.0, ratio))
        if getattr(self, '_soft_takeover', False):
            if abs(ratio - self._ratio_cmd) <= self._takeover_tolerance:
                self._soft_takeover = False
            else:
                ratio = self._ratio_cmd

        # Slew limit on the merged output. The driver sends an absolute angle
        # per tick and the servo goes at max speed, so a fast trigger pull (or
        # a pinch↔trigger source switch) would otherwise slam the gripper.
        # First publish snaps (avoids opening first when resuming mid-grasp).
        dt_pub = now - self._last_pub_t if self._last_pub_t > 0.0 else 0.0
        self._last_pub_t = now
        if self.max_ratio_rate > 0.0 and dt_pub > 0.0:
            max_d = self.max_ratio_rate * min(dt_pub, 0.1)
            d = ratio - self._ratio_cmd
            self._ratio_cmd += max(-max_d, min(max_d, d))
            ratio = self._ratio_cmd
        else:
            self._ratio_cmd = ratio

        cmd = Float64()
        cmd.data = float(ratio)
        self._pub_cmd.publish(cmd)

        if self.log_interval_s > 0 and (now - self._last_log) >= self.log_interval_s:
            self._last_log = now
            src = "trigger" if joy_fresh else "pinch"
            self.get_logger().info(
                f"[gripper {self.side}] src={src} dist={self._last_dist*1000:.0f}mm "
                f"range=[{(self._emin or 0.0)*1000:.0f},{(self._emax or 0.0)*1000:.0f}]mm "
                f"close={ratio:.2f} pinch_stale={pinch_stale}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PinchGripperNode()
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
