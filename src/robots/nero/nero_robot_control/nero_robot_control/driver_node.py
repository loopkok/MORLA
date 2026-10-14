"""Standard JointState bridge to the Nero SDK; only this node writes hardware."""

from __future__ import annotations

import json
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .driver_core import NeroDriverCore
from .sdk_adapter import FakeNeroSDK, PyAgxArmAdapter


LEFT_LIMITS = np.array(
    [
        [-2.705261, 2.705261],
        [-1.745330, 1.745330],
        [-2.757621, 2.757621],
        [-1.012291, 2.146755],
        [-2.757621, 2.757621],
        [-0.733039, 0.959932],
        [-1.570797, 1.570797],
    ],
    dtype=float,
)


def _latest_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class NeroRobotDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("nero_robot_driver")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("sdk_backend", "fake")  # fake | hardware
        self.declare_parameter("can_channel", "can_nero_left")
        self.declare_parameter("can_interface", "socketcan")
        self.declare_parameter("firmware_version", "default")
        # A nonempty floating sentinel makes rclpy infer DOUBLE_ARRAY rather
        # than BYTE_ARRAY; invalid/missing config is then rejected by shape.
        self.declare_parameter("initial_position", [0.0])
        self.declare_parameter("position_pose", [0.0])
        self.declare_parameter("position_speed_percent", 5)
        self.declare_parameter("max_joint_velocity_rad_s", 1.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("max_feedback_age_s", 0.5)
        self.declare_parameter("control_rate_hz", 125.0)
        self.declare_parameter("state_rate_hz", 20.0)
        self.declare_parameter("max_command_age_s", 0.5)
        self.declare_parameter("sdk_timeout_s", 2.0)
        self.declare_parameter("flange_frame_id", "")

        self.side = str(self.get_parameter("arm_side").value).strip().lower()
        self.backend = str(self.get_parameter("sdk_backend").value).strip().lower()
        if self.side not in ("left", "right"):
            raise ValueError("arm_side must be left or right")
        if self.backend not in ("fake", "hardware"):
            raise ValueError("sdk_backend must be fake or hardware")
        self.joint_names = [f"{self.side}_joint{i}" for i in range(1, 8)]
        self.base_frame = f"{self.side}_base_link"
        self.flange_frame_id = (
            str(self.get_parameter("flange_frame_id").value).strip()
            or f"{self.side}_gripper_flange"
        )
        self.command_topic = f"/{self.side}_arm/joint_commands"
        self.state_topic = f"/{self.side}_arm/joint_states"
        self.flange_topic = f"/{self.side}_arm/flange_pose"
        # Compatibility alias: historical topic name, but its payload is the
        # SDK's flange pose (not a separately calibrated hand/task TCP).
        self.tcp_topic = f"/{self.side}_arm/tcp_pose"
        self.position_pose = np.asarray(
            self.get_parameter("position_pose").value, dtype=float
        )
        seed = np.asarray(self.get_parameter("initial_position").value, dtype=float)
        if seed.shape != (7,) or not np.isfinite(seed).all():
            raise ValueError("initial_position must be an explicit finite 7-vector")
        if self.position_pose.size and (
            self.position_pose.shape != (7,) or not np.isfinite(self.position_pose).all()
        ):
            raise ValueError("position_pose must be empty or a finite 7-vector")

        limits = LEFT_LIMITS.copy()
        self.core = NeroDriverCore(
            self.side,
            limits,
            max_velocity_rad_s=float(
                self.get_parameter("max_joint_velocity_rad_s").value
            ),
            command_timeout_s=float(self.get_parameter("command_timeout_s").value),
            max_feedback_age_s=float(self.get_parameter("max_feedback_age_s").value),
            command_period_s=1.0 / float(self.get_parameter("control_rate_hz").value),
        )
        if self.backend == "fake":
            self.sdk = FakeNeroSDK(seed)
        else:
            self.sdk = PyAgxArmAdapter(
                str(self.get_parameter("can_channel").value),
                str(self.get_parameter("can_interface").value),
                str(self.get_parameter("firmware_version").value),
            )

        # All vendor SDK I/O is serialized off the ROS callback thread. Timer
        # callbacks never enqueue unbounded reads or motion commands.
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="nero-sdk")
        self._shutdown_requested = False
        self._connect_cleanup_lock = threading.Lock()
        self._connect_cleanup_claimed = False
        self._connect_started = time.monotonic()
        self._connect_future: Future = self._worker.submit(self._connect_and_configure)
        self._connect_future.add_done_callback(self._disconnect_if_shut_down)
        self._enable_future: Future | None = None
        self._estop_future: Future | None = None
        self._position_future: Future | None = None
        self._read_future: Future | None = None
        self._command_future: Future | None = None
        self._pending_command: np.ndarray | None = None
        self._pending_hold: np.ndarray | None = None
        self._last_feedback_schedule = 0.0
        self._last_error = ""
        self._connect_expired = False
        self._position_active = False
        self._position_seq = 0
        self._position_status = "idle"
        self._estop_seq = 0
        self._estop_status = "idle"
        self._enable_status = "idle"

        qos = _latest_qos()
        self._state_pub = self.create_publisher(JointState, self.state_topic, qos)
        self._flange_pub = self.create_publisher(PoseStamped, self.flange_topic, qos)
        self._legacy_tcp_pub = self.create_publisher(PoseStamped, self.tcp_topic, qos)
        self._status_pub = self.create_publisher(String, "~/status", qos)
        self.create_subscription(
            JointState, self.command_topic, self._on_command, qos
        )
        self.create_service(Trigger, "~/enable", self._on_enable)
        self.create_service(Trigger, "~/position", self._on_position)
        self.create_service(Trigger, "~/estop", self._on_estop)

        control_hz = float(self.get_parameter("control_rate_hz").value)
        state_hz = float(self.get_parameter("state_rate_hz").value)
        self._control_timer = self.create_timer(1.0 / control_hz, self._control_tick)
        self._state_timer = self.create_timer(1.0 / state_hz, self._state_tick)
        self._status_timer = self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f"Nero driver side={self.side} backend={self.backend} "
            f"command={self.command_topic} flange={self.flange_topic} "
            f"(frame={self.flange_frame_id}; legacy alias={self.tcp_topic}) "
            f"state_rate={state_hz:g}Hz; "
            "SDK enable is explicit and no home/position move starts automatically"
        )

    def _complete(self, future: Future | None) -> bool:
        return future is None or future.done()

    def _connect_and_configure(self) -> bool:
        connected = self.sdk.connect()
        if connected is False:
            return False
        try:
            self.sdk.configure_streaming()
        except Exception:
            try:
                self.sdk.disconnect()
            finally:
                raise
        return True

    def _disconnect_if_shut_down(self, future: Future) -> None:
        # A connect may finish after ROS teardown begins. Run cleanup on the
        # same SDK worker so a late connection cannot outlive its ROS owner.
        if not self._shutdown_requested:
            return
        try:
            connected = future.result() is True
        except Exception:
            connected = False
        if connected and self._claim_connect_cleanup():
            try:
                self.sdk.disconnect()
            except Exception:
                pass

    def _claim_connect_cleanup(self) -> bool:
        with self._connect_cleanup_lock:
            if self._connect_cleanup_claimed:
                return False
            self._connect_cleanup_claimed = True
            return True

    def _poll_connect(self) -> None:
        if self._connect_future is None:
            return
        if not self._connect_future.done():
            elapsed = time.monotonic() - self._connect_started
            if elapsed > float(self.get_parameter("sdk_timeout_s").value):
                self._connect_expired = True
                self.core.set_connected(False)
                self._last_error = "SDK connect exceeded sdk_timeout_s"
            return
        future, self._connect_future = self._connect_future, None
        try:
            result = future.result()
            if self._connect_expired:
                self._worker.submit(self.sdk.disconnect)
                return
            if result is False:
                raise RuntimeError("SDK connect returned false")
            self.core.set_connected(True)
            self._last_error = ""
            self.get_logger().info("Nero SDK connected; waiting for measured feedback")
        except Exception as exc:  # SDK exceptions surface through status, not callbacks
            self.core.set_connected(False)
            self._last_error = f"connect failed: {exc}"
            self.get_logger().error(self._last_error)

    def _poll_operation(self, name: str) -> None:
        attr = f"_{name}_future"
        future = getattr(self, attr)
        if future is None:
            return
        if not future.done():
            started = getattr(self, f"_{name}_started", None)
            if started is not None and time.monotonic() - started > float(
                self.get_parameter("sdk_timeout_s").value
            ):
                self._last_error = f"SDK {name} exceeded sdk_timeout_s"
                setattr(self, f"_{name}_expired", True)
                if name == "enable":
                    self.core.revoke()
                    self._enable_status = "timed_out"
                elif name == "estop":
                    self._estop_status = "timed_out"
                elif name == "position":
                    self._position_status = "timed_out"
            return
        setattr(self, attr, None)
        try:
            result = future.result()
            if name == "enable":
                if getattr(self, "_enable_expired", False):
                    self.core.revoke()
                    self._enable_status = "timed_out"
                    self._last_error = "late SDK enable result; motion remains revoked"
                    if result is True:
                        self._estop_future = self._worker.submit(self.sdk.estop)
                        self._estop_started = time.monotonic()
                    return
                if result is not True:
                    raise RuntimeError("SDK enable returned false")
                self.core.set_enabled(True)
                self._enable_status = "enabled"
                self._last_error = ""
                self.get_logger().info("Nero SDK explicitly enabled")
            elif name == "estop":
                self._estop_status = "sent"
                self._last_error = ""
                self.get_logger().error("Electronic emergency stop was sent")
            elif name == "position":
                if getattr(self, "_position_expired", False):
                    self._position_active = False
                    self._position_status = "timed_out"
                    self.core.revoke()
                    self._last_error = "position operation exceeded timeout; arrival is unknown"
                    return
                self._position_active = False
                self._position_status = "sent_arrival_unverified"
                self.core.revoke()
                self._pending_command = None
                self._last_error = "position request sent; verify measured arrival before re-enable"
                self.get_logger().warning(self._last_error)
            elif name == "command":
                self._last_error = ""
            elif name == "read":
                q, flange = result
                self.core.update_feedback(q)
                self._publish_feedback(q, flange)
                self._last_error = ""
        except Exception as exc:
            if name in ("enable", "estop", "position"):
                self.core.revoke()
            if name == "position":
                self._position_active = False
                self._position_status = "failed"
            elif name == "estop":
                self._estop_status = "failed"
            elif name == "enable":
                self._enable_status = "failed"
            self._last_error = f"SDK {name} failed: {exc}"
            self.get_logger().error(self._last_error)

    def _on_command(self, msg: JointState) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        now_ros_ns = self.get_clock().now().nanoseconds
        max_age = float(self.get_parameter("max_command_age_s").value)
        if stamp_ns <= 0 or now_ros_ns <= 0 or abs(now_ros_ns - stamp_ns) > max_age * 1e9:
            self.core.last_rejection = "command header timestamp is missing or stale"
            return
        if self._position_active:
            self.core.last_rejection = "driver is executing explicit position request"
            return
        try:
            accepted = self.core.accept_command(msg.name, msg.position)
            self._pending_command = accepted.target
            if accepted.rate_limited:
                self.get_logger().warning(
                    "joint command rad/s limit applied", throttle_duration_sec=2.0
                )
        except (ValueError, RuntimeError) as exc:
            self.core.last_rejection = str(exc)

    def _on_enable(self, _request, response):
        if not self.core.connected:
            response.success = False
            response.message = "SDK is not connected"
            return response
        if self._enable_future is not None:
            response.success = False
            response.message = "enable request is already pending"
            return response
        if self.core.feedback is None or self.core.feedback_age() > self.core.max_feedback_age_s:
            response.success = False
            response.message = "fresh measured seven-joint feedback is required before enable"
            return response
        self._enable_started = time.monotonic()
        self._enable_expired = False
        self._enable_status = "queued"
        self._enable_future = self._worker.submit(self._enable_and_configure)
        response.success = True
        response.message = "explicit enable request queued; verify ~/status before sending motion"
        return response

    def _enable_and_configure(self) -> bool:
        enabled = bool(self.sdk.enable())
        if enabled:
            self.sdk.configure_streaming()
        return enabled

    def _on_position(self, _request, response):
        if self.position_pose.shape != (7,):
            response.success = False
            response.message = "position_pose is not configured"
            return response
        if not self.core.enabled or self.core.feedback is None:
            response.success = False
            response.message = "driver must be enabled with measured feedback"
            return response
        if self.core.feedback_age() > self.core.max_feedback_age_s:
            response.success = False
            response.message = "measured feedback is stale"
            return response
        if self._position_future is not None:
            response.success = False
            response.message = "position request is already pending"
            return response
        if np.any(self.position_pose < self.core.limits[:, 0]) or np.any(
            self.position_pose > self.core.limits[:, 1]
        ):
            response.success = False
            response.message = "configured position is outside joint limits"
            return response
        self.core.revoke()
        self._pending_command = None
        self._position_active = True
        self._position_seq += 1
        self._position_status = "running"
        self._position_started = time.monotonic()
        self._position_expired = False
        speed = int(self.get_parameter("position_speed_percent").value)
        pose = self.position_pose.copy()
        self._position_future = self._worker.submit(self._position_sequence, pose, speed)
        response.success = True
        response.message = "explicit configured position request queued; no arrival is implied"
        return response

    def _position_sequence(self, pose: np.ndarray, speed: int) -> None:
        try:
            self.sdk.set_position_mode(speed)
            self.sdk.move_j(pose)
        finally:
            self.sdk.restore_streaming_mode()

    def _on_estop(self, _request, response):
        self.core.revoke()
        self._pending_command = None
        if self._estop_future is not None:
            response.success = False
            response.message = "emergency stop request is already pending"
            return response
        self._estop_started = time.monotonic()
        self._estop_expired = False
        self._estop_seq += 1
        self._estop_status = "queued"
        self._estop_future = self._worker.submit(self.sdk.estop)
        response.success = True
        response.message = "electronic emergency stop request queued"
        return response

    def _state_tick(self) -> None:
        self._poll_connect()
        self._poll_operation("enable")
        self._poll_operation("estop")
        self._poll_operation("position")
        self._poll_operation("read")
        now = time.monotonic()
        if self.core.connected and self._read_future is None and (
            now - self._last_feedback_schedule
            >= 1.0 / float(self.get_parameter("state_rate_hz").value)
        ):
            self._last_feedback_schedule = now
            self._read_future = self._worker.submit(self.sdk.read_state)

    def _control_tick(self) -> None:
        self._poll_operation("command")
        hold = self.core.watchdog_hold()
        if hold is not None:
            self.core.revoke()
            self._pending_command = None
            self._pending_hold = hold.copy()
            self.get_logger().error(
                "command_timeout: issuing one measured-seeded hold and revoking motion eligibility"
            )
        if self._command_future is not None:
            return
        watchdog_hold = self._pending_hold is not None
        target = self._pending_hold if watchdog_hold else self._pending_command
        self._pending_hold = None
        self._pending_command = None
        if target is not None and (self.core.enabled or watchdog_hold):
            self._command_future = self._worker.submit(self.sdk.move_js, target)

    def _publish_feedback(self, q, flange) -> None:
        q = np.asarray(q, dtype=float)
        flange = np.asarray(flange, dtype=float)
        stamp = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.header.frame_id = self.base_frame
        state.name = list(self.joint_names)
        state.position = [float(value) for value in q]
        self._state_pub.publish(state)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, flange[:3])
        quat = Rotation.from_euler("xyz", flange[3:]).as_quat()
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = map(float, quat)
        self._flange_pub.publish(pose)
        self._legacy_tcp_pub.publish(pose)

    def _publish_status(self) -> None:
        age = self.core.feedback_age()
        payload = {
            "side": self.side,
            "backend": self.backend,
            "flange_topic": self.flange_topic,
            "flange_frame_id": self.flange_frame_id,
            "flange_pose_source": "sdk_get_flange_pose_frame_mapping_unverified",
            "legacy_tcp_topic": self.tcp_topic,
            "legacy_tcp_topic_deprecated": True,
            "connected": self.core.connected,
            "enabled": self.core.enabled,
            "feedback_fresh": age is not None and age <= self.core.max_feedback_age_s,
            "feedback_age_s": age,
            "command_active": self.core.command_active,
            "position_active": self._position_active,
            "position_seq": self._position_seq,
            "position_status": self._position_status,
            "position_error_rad": (
                float(np.max(np.abs(self.core.feedback - self.position_pose)))
                if self.core.feedback is not None and self.position_pose.shape == (7,) else None
            ),
            "estop_seq": self._estop_seq,
            "estop_status": self._estop_status,
            "enable_status": self._enable_status,
            "last_rejection": self.core.last_rejection,
            "last_error": self._last_error,
        }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def destroy_node(self):
        self._shutdown_requested = True
        try:
            self.core.revoke()
            if self._connect_future is not None and self._connect_future.done():
                try:
                    connected = self._connect_future.result() is True
                except Exception:
                    connected = False
                if connected and self._claim_connect_cleanup():
                    self._worker.submit(self.sdk.disconnect)
            elif self.core.connected:
                self._worker.submit(self.sdk.disconnect)
        finally:
            # Preserve queued disconnect/e-stop cleanup behind any in-flight
            # vendor call; cancelling that task would leave the transport live.
            self._worker.shutdown(wait=False, cancel_futures=False)
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NeroRobotDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
