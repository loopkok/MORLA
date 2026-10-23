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
from rcl_interfaces.msg import SetParametersResult

from .driver_core import FEEDBACK_LIMIT_TOLERANCE_RAD, FeedbackLimitError, NeroDriverCore
from .home_trajectory import HOME_FINAL_POSE_DEG, home_targets
from .workpos_trajectory import workpos_targets
from .waypoint_trajectory import TRAJECTORY_REVISION, default_waypoints_deg
from .sdk_adapter import FakeNeroSDK, PyAgxArmAdapter


ARM_STATUS_REASONS = {
    0: "normal", 1: "emergency stop", 2: "no solution", 3: "singularity point",
    4: "target angle exceeds limit", 5: "joint communication exception",
    6: "joint brake not released", 7: "collision occurred", 8: "teaching overspeed",
    9: "abnormal joint status", 10: "other controller exception",
    11: "teaching record", 12: "teaching execution", 13: "teaching paused",
    14: "controller over-temperature", 15: "brake resistor over-temperature",
}


def _arm_status_reason(code: int) -> str:
    return ARM_STATUS_REASONS.get(int(code), "controller reported a non-normal arm status")


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
        self.declare_parameter("native_j7_first", False)
        self.declare_parameter("stream_position_mode", "js")
        self.declare_parameter("stop_on_shutdown", False)
        # A nonempty floating sentinel makes rclpy infer DOUBLE_ARRAY rather
        # than BYTE_ARRAY; invalid/missing config is then rejected by shape.
        self.declare_parameter("initial_position", [0.0])
        self.declare_parameter("position_pose", [0.0])
        self.declare_parameter("position_speed_percent", 5)
        # Zero inherits the existing work-position speed at startup (legacy behavior).
        self.declare_parameter("stream_j_speed_percent", 0)
        self.declare_parameter("position_timeout_s", 40.0)
        # Original profiles retain 0.02 rad. Piano opts into 3 degrees.
        self.declare_parameter("workpos_arrival_tolerance_deg", float(np.rad2deg(0.02)))
        self.declare_parameter("home_speed_percent", 5)
        self.declare_parameter("home_stage_timeout_s", 40.0)
        self.declare_parameter("max_joint_velocity_rad_s", 1.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("max_feedback_age_s", 0.5)
        self.declare_parameter("control_rate_hz", 125.0)
        self.declare_parameter("state_rate_hz", 20.0)
        self.declare_parameter("max_command_age_s", 0.5)
        self.declare_parameter("sdk_timeout_s", 2.0)
        self.declare_parameter("reset_timeout_s", 5.0)
        self.declare_parameter("can_feedback_timeout_s", 3.0)
        self.declare_parameter("flange_frame_id", "")

        self.side = str(self.get_parameter("arm_side").value).strip().lower()
        self.backend = str(self.get_parameter("sdk_backend").value).strip().lower()
        if self.side not in ("left", "right"):
            raise ValueError("arm_side must be left or right")
        if self.backend not in ("fake", "hardware"):
            raise ValueError("sdk_backend must be fake or hardware")
        self.declare_parameter("workpos_waypoints_deg", default_waypoints_deg(self.side).flatten().tolist())
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
        # The shared path is authoritative for both workpos and reverse HOME.
        self.position_pose = workpos_targets(self.side, self.get_parameter("workpos_waypoints_deg").value)[-1]
        position_timeout = float(self.get_parameter("position_timeout_s").value)
        if not np.isfinite(position_timeout) or position_timeout <= 0:
            raise ValueError("position_timeout_s must be finite and positive")
        home_timeout = float(self.get_parameter("home_stage_timeout_s").value)
        if not np.isfinite(home_timeout) or home_timeout <= 0:
            raise ValueError("home_stage_timeout_s must be finite and positive")
        can_timeout = float(self.get_parameter("can_feedback_timeout_s").value)
        if not np.isfinite(can_timeout) or can_timeout <= 0:
            raise ValueError("can_feedback_timeout_s must be finite and positive")
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
        stream_speed = self.get_parameter("stream_j_speed_percent").value
        if isinstance(stream_speed, bool) or not isinstance(stream_speed, int) or not 0 <= stream_speed <= 100:
            raise ValueError("stream_j_speed_percent must be an integer in [0,100]")
        stream_speed = stream_speed or int(self.get_parameter("position_speed_percent").value)
        if self.backend == "fake":
            self.sdk = FakeNeroSDK(seed)
            self.sdk.stream_j_speed_percent = stream_speed
            self.sdk.stream_position_mode = str(self.get_parameter("stream_position_mode").value)
        else:
            self.sdk = PyAgxArmAdapter(
                str(self.get_parameter("can_channel").value),
                str(self.get_parameter("can_interface").value),
                str(self.get_parameter("firmware_version").value),
                max_feedback_age_s=self.core.max_feedback_age_s,
                native_j7_first=bool(self.get_parameter("native_j7_first").value),
                stream_position_mode=str(self.get_parameter("stream_position_mode").value),
                stream_j_speed_percent=stream_speed,
            )

        # All vendor SDK I/O is serialized off the ROS callback thread. Timer
        # callbacks never enqueue unbounded reads or motion commands.
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="nero-sdk")
        self._shutdown_requested = False
        self._connect_cleanup_lock = threading.Lock()
        self._connect_cleanup_claimed = False
        self._connect_started = time.monotonic()
        self._connect_future: Future = self._worker.submit(self.sdk.connect)
        self._connect_future.add_done_callback(self._disconnect_if_shut_down)
        self._enable_future: Future | None = None
        self._can_init_future: Future | None = None
        self._can_init_status = "idle"
        self._can_init_seq = 0
        self._can_init_error = ""
        self._can_init_baseline = 0.0
        self._can_motor_enable_confirmed = False
        self._estop_future: Future | None = None
        self._reset_future: Future | None = None
        self._controller_status_future: Future | None = None
        self._controller_status: dict | None = None
        self._controller_status_received = 0.0
        self._controller_status_generation = 0
        self._controller_status_future_reset_seq = 0
        self._controller_status_last_reset_seq = 0
        self._controller_status_schedule = 0.0
        self._controller_status_error = ""
        self._reset_seq = 0
        self._reset_status = "idle"
        self._reset_error = ""
        self._reset_mode = "idle"
        self._reset_status_not_before = 0.0
        self._reset_baseline_generation = 0
        self._position_future: Future | None = None
        self._read_future: Future | None = None
        self._command_future: Future | None = None
        self._pending_command: np.ndarray | None = None
        self._pending_hold: np.ndarray | None = None
        self._last_feedback_schedule = 0.0
        self._last_feedback_read_started = 0.0
        self._feedback_read_ok = False
        self._feedback_read_error = ""
        self._last_feedback_limit_violation = None
        self._last_error = ""
        self._connect_expired = False
        self._position_active = False
        self._position_permitted = False
        self._position_seq = 0
        self._position_status = "idle"
        self._position_arrival_since: float | None = None
        self._position_arrival_tolerance_rad = 0.02
        self._position_kind = "workpos"
        self._position_targets = [self.position_pose.copy()]
        self._position_target = self.position_pose.copy()
        self._position_stage = 0
        self._position_feedback_baseline = 0.0
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
        self.create_service(Trigger, "~/initialize_can", self._on_initialize_can)
        self.create_service(Trigger, "~/position", self._on_position)
        for stage in range(2, 5):
            self.create_service(Trigger, f"~/position_point{stage}",
                                lambda req, res, stage=stage: self._advance_position("workpos", stage, res))
        self.create_service(Trigger, "~/home", self._on_home)
        for stage in range(2, 5):
            self.create_service(Trigger, f"~/home_point{5 - stage}",
                                lambda req, res, stage=stage: self._advance_position("home", stage, res))
        self.create_service(Trigger, "~/home_final_pose",
                            lambda req, res: self._advance_position("home", 5, res))
        self.create_service(Trigger, "~/estop", self._on_estop)

        control_hz = float(self.get_parameter("control_rate_hz").value)
        state_hz = float(self.get_parameter("state_rate_hz").value)
        self.add_on_set_parameters_callback(self._validate_stream_speed)
        self._control_timer = self.create_timer(1.0 / control_hz, self._control_tick)
        self._state_timer = self.create_timer(1.0 / state_hz, self._state_tick)
        self._status_timer = self.create_timer(0.2, self._publish_status)
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.get_logger().info(
            f"Nero driver side={self.side} backend={self.backend} "
            f"command={self.command_topic} flange={self.flange_topic} "
            f"(frame={self.flange_frame_id}; legacy alias={self.tcp_topic}) "
            f"state_rate={state_hz:g}Hz; "
            "SDK enable is explicit and no home/position move starts automatically"
        )

    def _validate_stream_speed(self, parameters):
        requested = [p.value for p in parameters if p.name == "stream_j_speed_percent"]
        if not requested:
            return SetParametersResult(successful=True)
        if (len(requested) != 1 or isinstance(requested[0], bool)
                or not isinstance(requested[0], int) or not 1 <= requested[0] <= 100):
            return SetParametersResult(successful=False, reason="stream speed must be an integer in [1,100]")
        if self._position_active or self._position_future is not None:
            return SetParametersResult(successful=False, reason="native waypoint operation is active")
        # SDK I/O occurs only in the serialized command worker. No enable/disable,
        # reset, target or mode transition is performed by this callback.
        return SetParametersResult(successful=True)

    def _complete(self, future: Future | None) -> bool:
        return future is None or future.done()

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
            timeout_parameter = "reset_timeout_s" if name == "reset" else "sdk_timeout_s"
            if started is not None and time.monotonic() - started > float(
                self.get_parameter(timeout_parameter).value
            ):
                self._last_error = f"SDK {name} exceeded sdk_timeout_s"
                setattr(self, f"_{name}_expired", True)
                if name == "enable":
                    self.core.revoke()
                    self._enable_status = "timed_out"
                elif name == "can_init":
                    if self._can_init_status != "timed_out":
                        self._fail_can_init("timed_out", self._last_error)
                elif name == "estop":
                    self._estop_status = "timed_out"
                elif name == "reset":
                    self._reset_status = "timed_out"
                    self._reset_error = self._last_error
                elif name == "position":
                    if self._position_active:
                        self._abort_position("timed_out", self._last_error)
            return
        setattr(self, attr, None)
        try:
            result = future.result()
            if name == "can_init":
                if getattr(self, "_can_init_expired", False):
                    return
                if result is not True:
                    if time.monotonic() - self._can_init_started < float(self.get_parameter("sdk_timeout_s").value):
                        self._can_init_future = self._worker.submit(self.sdk.initialize_can)
                    else:
                        self._fail_can_init("timed_out", "CAN cold-start motor enable was not confirmed before timeout; electronic stop requested")
                    return
                self._can_motor_enable_confirmed = True
                self._can_init_status = "waiting_feedback"
                # Require a read completed after the initialization call.
                self._can_init_baseline = time.monotonic()
            elif name == "enable":
                if getattr(self, "_enable_expired", False):
                    self.core.revoke()
                    self._enable_status = "timed_out"
                    self._last_error = "late SDK enable result; motion remains revoked"
                    if result is True:
                        self._estop_future = self._worker.submit(self.sdk.estop)
                        self._estop_started = time.monotonic()
                    return
                if result is not True:
                    # pyAgxArm sends the enable request and immediately checks
                    # asynchronous motor feedback. False can mean "not yet".
                    # Retry on later state ticks, leaving the SDK worker free
                    # for feedback reads and keeping the original deadline.
                    timeout = float(self.get_parameter("sdk_timeout_s").value)
                    if time.monotonic() - self._enable_started < timeout:
                        self._enable_status = "waiting_confirmation"
                        self._enable_future = self._worker.submit(self._enable_and_configure)
                        return
                    self.core.revoke()
                    self._enable_status = "timed_out"
                    self._last_error = f"SDK enable was not confirmed within {timeout:g}s"
                    self.get_logger().error(self._last_error)
                    return
                self.core.set_enabled(True)
                self._position_permitted = True
                self._enable_status = "enabled"
                self._last_error = ""
                self.get_logger().info("Nero SDK explicitly enabled")
            elif name == "estop":
                self.core.revoke()
                self._position_permitted = False
                self._estop_status = "sent"
                self._enable_status = "idle"
                if (self._position_status not in {"failed", "timed_out"}
                        and self._can_init_status not in {"failed", "timed_out"}):
                    self._last_error = ""
                self.get_logger().error("Electronic emergency stop was sent")
            elif name == "reset":
                self._position_permitted = False
                if getattr(self, "_reset_expired", False):
                    self._reset_status = "timed_out"
                    self._reset_error = "SDK reset completed after its timeout; normal controller status still needs verification"
                    return
                if result is False:
                    raise RuntimeError("controller reset returned false")
                self._reset_status = "waiting_status"
                self._reset_error = ""
                self._reset_started = time.monotonic()
                self._last_error = ""
                self.get_logger().warning(
                    "controller reset command returned; waiting for fresh normal arm status"
                )
            elif name == "position":
                if self._position_status == "cancelled":
                    return
                if getattr(self, "_position_expired", False):
                    return
                # move_j() only sends a target. Keep J mode and keep the
                # operation active until measured arrival has settled.
                self._position_status = "waiting_arrival"
                self._position_feedback_baseline = time.monotonic()
                self.core.revoke()
                self._pending_command = None
                self._last_error = ""
                self.get_logger().info("position target sent; holding J mode until measured arrival")
            elif name == "command":
                self._last_error = ""
            elif name == "read":
                q, flange = result
                self.core.update_feedback(q)
                self._feedback_read_ok = True
                self._feedback_read_error = ""
                excess = np.maximum(self.core.limits[:, 0] - q, q - self.core.limits[:, 1])
                if np.any(excess > 0):
                    joint = int(np.argmax(excess))
                    self.get_logger().warning(
                        f"J{joint + 1} measured={np.rad2deg(q[joint]):.4f} deg, "
                        f"boundary excess={np.rad2deg(excess[joint]):.4f} deg within 0.1 deg measurement margin; "
                        "raw feedback retained, command limits unchanged", throttle_duration_sec=5.0)
                self._last_feedback_read_started = self._last_feedback_schedule
                self._publish_feedback(q, flange)
                # A healthy read must not hide a failed enable/motion request.
                if self._last_error.startswith("SDK read failed:"):
                    self._last_error = ""
        except Exception as exc:
            if name in ("enable", "estop", "reset", "position"):
                self.core.revoke()
                self._position_permitted = False
            if name == "position":
                self._position_active = False
                self._position_status = "failed"
            elif name == "estop":
                self._estop_status = "failed"
            elif name == "reset":
                self._reset_status = "failed"
                self._reset_error = str(exc)
            elif name == "enable":
                self._enable_status = "failed"
            elif name == "can_init":
                self._can_init_status = "failed"
            # Keep the operation's root failure across repeated rejected reads
            # and later recovery; fresh read diagnostics have a separate field.
            if name != "read" or self._position_status not in {"failed", "timed_out"}:
                self._last_error = f"SDK {name} failed: {exc}"
            if name == "read":
                self._feedback_read_ok = False
                self._feedback_read_error = str(exc)
                self._position_arrival_since = None
                if self._position_status == "stage_arrived":
                    self._position_status = "waiting_arrival"
                if isinstance(exc, FeedbackLimitError):
                    self._last_feedback_limit_violation = dict(at=time.time(), joints=exc.details)
                    if self._position_active:
                        self._abort_position("failed", f"position feedback limit violation: {exc}; electronic stop requested")
            if name == "can_init":
                self._fail_can_init("failed", self._last_error)
            if name == "position" and self._position_status != "cancelled":
                self._abort_position("failed", self._last_error)
            if name == "read" and self.core.feedback is None and self._can_init_status in {"idle", "queued", "waiting_feedback"}:
                self.get_logger().warning(
                    "Waiting for first measured CAN feedback; use Web CAN 冷启动（含电机使能） before 检查连接/使能. "
                    + self._last_error,
                    throttle_duration_sec=5.0,
                )
            elif name == "read":
                self.get_logger().error(self._last_error, throttle_duration_sec=5.0)
            else:
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

    def _on_initialize_can(self, _request, response):
        if not self.core.connected:
            response.success = False
            response.message = "SDK transport is not connected"
            return response
        if self.backend == "hardware" and self._controller_status is not None:
            arm_status = int(self._controller_status.get("arm_status", -1))
            if arm_status == 1:
                response.success = False
                response.message = "Nero controller reports emergency stop (arm_status=1); explicitly reset it before CAN initialization"
                return response
            if arm_status not in {0, 6}:
                response.success = False
                response.message = f"Nero controller reports {_arm_status_reason(arm_status)} (arm_status={arm_status}); CAN initialization refused"
                return response
        if (self.core.enabled or self.core.command_active or self._position_active
                or self._position_future is not None or self._enable_future is not None
                or self._command_future is not None
                or self._estop_future is not None or self._reset_future is not None
                or self._reset_status in {"queued", "waiting_status"}
                or self._can_init_future is not None
                or self._can_init_status in {"queued", "waiting_feedback"}):
            response.success = False
            response.message = "CAN initialization requires disabled motion and no pending operation"
            return response
        self.core.revoke()
        self._pending_command = self._pending_hold = None
        self._can_init_started = time.monotonic()
        self._position_permitted = False
        self._can_init_expired = False
        self._can_init_seq += 1
        self._can_init_status = "queued"
        self._can_init_error = ""
        self._can_motor_enable_confirmed = False
        self._can_init_future = self._worker.submit(self.sdk.initialize_can)
        response.success = True
        response.message = "CAN cold start queued: motor enable requested; waiting for motor acknowledgement and fresh feedback; teleop permission remains revoked"
        return response

    def _fail_can_init(self, status, message):
        self._can_init_expired = True
        self._can_init_status = status
        self._can_init_error = message
        self._last_error = message
        self._can_motor_enable_confirmed = False
        # A failed cold startup may have partially enabled physical motors.
        # Request electronic stop; do not equate request acceptance with stop.
        self._on_estop(None, Trigger.Response())
        self.get_logger().error(message)

    def _check_can_feedback(self):
        if self._can_init_status != "waiting_feedback":
            return
        if (self.core.feedback_at is not None and self._last_feedback_read_started > self._can_init_baseline
                and self.core.feedback_age() <= self.core.max_feedback_age_s):
            self._can_init_status = "ready"
            self._can_init_error = ""
            self._last_error = ""
        elif time.monotonic() - self._can_init_started > float(self.get_parameter("can_feedback_timeout_s").value):
            self._fail_can_init("timed_out", "CAN initialization received no fresh joint/flange feedback before timeout; electronic stop requested")

    def _on_enable(self, _request, response):
        if self._reset_future is not None or self._reset_status in {"queued", "waiting_status"}:
            response.success = False
            response.message = "controller reset is pending; wait for confirmed normal status before enable"
            return response
        if self.backend == "hardware":
            if not self._controller_status_is_fresh():
                response.success = False
                response.message = "fresh Nero controller status is required before enable; wait for status polling"
                return response
            arm_status = int(self._controller_status.get("arm_status", -1))
            if arm_status == 1:
                response.success = False
                response.message = "Nero controller reports emergency stop (arm_status=1); reset it and verify normal status before enable"
                return response
            if arm_status not in {0, 6}:
                response.success = False
                response.message = f"Nero controller reports {_arm_status_reason(arm_status)} (arm_status={arm_status}); enable refused"
                return response
        if self._can_init_future is not None or self._can_init_status in {"queued", "waiting_feedback"}:
            response.success = False
            response.message = "CAN initialization is pending; wait for confirmed feedback"
            return response
        if self._position_active or self._position_future is not None or self._estop_future is not None:
            response.success = False
            response.message = "position/stop operation is pending; streaming mode cannot be enabled"
            return response
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
        if self.backend == "hardware":
            if not self._controller_status_is_fresh():
                response.success = False
                response.message = "fresh Nero controller status is required before enable; wait for status polling"
                return response
            # The nonzero case was rejected before the rest of the enable gate.
        self._enable_started = time.monotonic()
        self._position_permitted = False
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
        try:
            targets = workpos_targets(self.side, self.get_parameter("workpos_waypoints_deg").value)
        except (TypeError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        return self._start_position(targets, "workpos", "position_speed_percent", response)

    def _on_home(self, _request, response):
        try:
            targets = home_targets(self.side, self.get_parameter("workpos_waypoints_deg").value)
        except (TypeError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        return self._start_position(targets, "home", "home_speed_percent", response)

    def _motion_feedback_error(self) -> str | None:
        if not self._position_permitted:
            return "position motion permission was revoked; prepare/enable before moving"
        if self.core.feedback is None or self.core.feedback_age() > self.core.max_feedback_age_s:
            age = self.core.feedback_age()
            return (f"fresh measured seven-joint feedback is required (age_s={age}); "
                    f"latest SDK read error: {self._feedback_read_error or 'none'}")
        if self.backend == "hardware":
            if not self._controller_status_is_fresh():
                return "fresh Nero controller status is required"
            code = int(self._controller_status.get("arm_status", -1))
            if code != 0:
                return f"controller reports {_arm_status_reason(code)} (arm_status={code})"
        if not self._motor_status_is_fresh():
            return "fresh enable feedback for all seven motors is required"
        if self._motors_enabled() is not True:
            return "one or more motors are not enabled; prepare/enable before position motion"
        return None

    def _motor_status_is_fresh(self) -> bool:
        if self.backend == "fake":
            return self.core.connected
        if not self._controller_status_is_fresh() or not self._controller_status.get("motor_status_fresh"):
            return False
        try:
            stamp = float(self._controller_status["motor_status_timestamp"])
        except (KeyError, TypeError, ValueError):
            return False
        return np.isfinite(stamp) and -0.05 <= time.time() - stamp <= self.core.max_feedback_age_s

    def _motors_enabled(self) -> bool | None:
        if self.backend == "fake":
            return bool(self.sdk.enabled)
        return self._controller_status.get("motors_enabled") if self._controller_status else None

    def _start_position(self, targets, kind, speed_parameter, response):
        if (self._position_active or self._position_future is not None
                or self._estop_future is not None or self._enable_future is not None
                or self._reset_future is not None or self._reset_status in {"queued", "waiting_status"}
                or self._can_init_future is not None or self._can_init_status in {"queued", "waiting_feedback"}):
            response.success = False
            response.message = "position/stop operation is already pending"
            return response
        # core.enabled grants JS/teleop streaming, which explicit J motion
        # deliberately revokes. Chained J moves instead require current motor
        # feedback; never switch back to JS merely to run another J trajectory.
        if not self.core.connected or self.core.feedback is None:
            response.success = False
            response.message = "driver must be connected with measured feedback"
            return response
        feedback_error = self._motion_feedback_error()
        if not self._feedback_read_ok:
            feedback_error = feedback_error or f"latest feedback read is invalid: {self._feedback_read_error}"
        if feedback_error:
            response.success = False
            response.message = feedback_error
            return response
        for stage, target in enumerate(targets, 1):
            if np.asarray(target).shape != (7,) or not np.isfinite(target).all():
                response.success = False
                response.message = "every motion target must be a finite seven-joint vector"
                return response
            if np.any(target < self.core.limits[:, 0]) or np.any(target > self.core.limits[:, 1]):
                response.success = False
                joint = int(np.flatnonzero((target < self.core.limits[:, 0]) | (target > self.core.limits[:, 1]))[0])
                response.message = (f"{kind} stage {stage}: J{joint + 1} target "
                                    f"{np.rad2deg(target[joint]):.3f} deg is outside joint limits "
                                    f"{np.rad2deg(self.core.limits[joint]).tolist()}")
                return response
        speed = self.get_parameter(speed_parameter).value
        if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= 100:
            response.success = False
            response.message = f"{speed_parameter} must be an integer in [1, 100]"
            return response
        tolerance_deg = self.get_parameter("workpos_arrival_tolerance_deg").value if kind == "workpos" else float(np.rad2deg(.02))
        if (isinstance(tolerance_deg, bool) or not isinstance(tolerance_deg, (float, int))
                or not np.isfinite(tolerance_deg) or not .5 <= tolerance_deg <= 5.):
            response.success = False
            response.message = "workpos_arrival_tolerance_deg must be finite and in [0.5, 5]"
            return response
        self._position_arrival_tolerance_rad = float(np.deg2rad(tolerance_deg))
        self.core.revoke()
        self._pending_command = None
        self._pending_hold = None
        self._position_active = True
        self._position_kind = kind
        self._position_targets = [target.copy() for target in targets]
        self._position_speed = speed
        self._queue_position_stage(1)
        response.success = True
        response.message = f"{kind} stage 1/{len(targets)} queued; no arrival is implied"
        return response

    def _advance_home(self, stage, response):
        return self._advance_position("home", stage, response)

    def _advance_position(self, kind, stage, response):
        # Re-check timeouts and controller health even when the service lands
        # just before the next state timer callback.
        if self._position_active and self._position_kind == kind:
            self._check_position_progress()
        if (not 2 <= stage <= len(self._position_targets) or not self._position_active or self._position_kind != kind
                or self._position_status != "stage_arrived"
                or self._position_stage != stage - 1
                or self._position_future is not None or self._estop_future is not None):
            response.success = False
            response.message = f"{kind} stage {stage - 1} must arrive before stage {stage}"
            return response
        error = self._motion_feedback_error()
        if not self._feedback_read_ok:
            error = error or f"latest feedback read is invalid: {self._feedback_read_error}"
        if error or np.max(np.abs(self.core.feedback - self._position_target)) > self._position_arrival_tolerance_rad:
            self._abort_position("failed", error or f"{kind} left the confirmed stage target")
            response.success = False
            response.message = self._last_error
            return response
        self._queue_position_stage(stage)
        response.success = True
        response.message = f"{kind} stage {stage}/{len(self._position_targets)} queued; no arrival is implied"
        return response

    def _queue_position_stage(self, stage):
        self._position_stage = stage
        self._position_target = self._position_targets[stage - 1].copy()
        self._position_seq += 1
        self._position_status = "running"
        self._position_started = time.monotonic()
        self._position_expired = False
        self._position_arrival_since = None
        self._position_future = self._worker.submit(
            self._position_sequence, self._position_target.copy(), self._position_speed
        )

    def _position_sequence(self, pose: np.ndarray, speed: int) -> None:
        self.sdk.set_position_mode(speed)
        self.sdk.move_j(pose)
        # Never switch to JS here: move_j is nonblocking, and switching would
        # apply the distant target in unplanned instantaneous-response mode.
        # Explicit enable may restore streaming only after confirmed arrival.

    def _abort_position(self, status: str, message: str) -> None:
        self._position_permitted = False
        self._position_active = False
        self._position_expired = True
        self._position_arrival_since = None
        self._position_status = status
        self.core.revoke()
        self._pending_command = self._pending_hold = None
        self._last_error = message
        if self._estop_future is None:
            self._estop_started = time.monotonic()
            self._estop_expired = False
            self._estop_seq += 1
            self._estop_status = "queued"
            self._estop_future = self._worker.submit(self.sdk.estop)

    def _check_position_progress(self) -> None:
        if not self._position_active:
            return
        now = time.monotonic()
        timeout_parameter = "home_stage_timeout_s" if self._position_kind == "home" else "position_timeout_s"
        if now - self._position_started > float(self.get_parameter(timeout_parameter).value):
            self._abort_position("timed_out", "position arrival timed out; electronic stop requested")
            return
        feedback_error = self._motion_feedback_error()
        if feedback_error:
            self._abort_position("failed", f"position feedback/controller failed: {feedback_error}; electronic stop requested")
            return
        if not self._feedback_read_ok:
            self._position_arrival_since = None
            return  # A cached good sample cannot confirm arrival after a bad read.
        error = float(np.max(np.abs(self.core.feedback - self._position_target)))
        if self._position_status == "stage_arrived" and error > self._position_arrival_tolerance_rad:
            # A disturbed arm must settle again before the bilateral barrier
            # can advance; an old arrival flag cannot authorize a new stage.
            self._position_status = "waiting_arrival"
            self._position_arrival_since = None
        if (self._position_status != "waiting_arrival"
                or self._last_feedback_read_started <= self._position_feedback_baseline):
            return
        if error > self._position_arrival_tolerance_rad:
            self._position_arrival_since = None
            return
        if self._position_arrival_since is None:
            self._position_arrival_since = now
        elif now - self._position_arrival_since >= 0.2:
            final_stage = self._position_stage == len(self._position_targets)
            self._position_active = not final_stage
            self._position_status = "arrived" if final_stage else "stage_arrived"
            self._last_error = ""
            self.get_logger().info(
                f"{self._position_kind} stage {self._position_stage}/{len(self._position_targets)} arrival confirmed; "
                + ("J mode retained, explicit enable required" if final_stage
                   else "waiting for coordinator to confirm peer arms before next stage")
            )

    def _on_estop(self, _request, response):
        self._position_permitted = False
        # Emergency stop is always allowed to follow a reset request; the
        # single SDK worker serializes both commands in request order.
        if self._can_init_status in {"queued", "waiting_feedback"}:
            self._can_init_status = "cancelled"
            self._can_init_expired = True
            self._can_init_error = "CAN initialization cancelled by emergency stop"
        self._can_motor_enable_confirmed = False
        if self._position_active:
            self._position_active = False
            self._position_expired = True
            self._position_status = "cancelled"
            self._position_arrival_since = None
        self.core.revoke()
        self._pending_command = None
        self._pending_hold = None
        self._enable_status = "idle"
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

    def _controller_status_is_fresh(self) -> bool:
        if (self._controller_status is None
                or time.monotonic() - self._controller_status_received > 1.0):
            return False
        try:
            stamp = float(self._controller_status["timestamp"])
        except (KeyError, TypeError, ValueError):
            return False
        age = time.time() - stamp
        return np.isfinite(stamp) and -0.05 <= age <= self.core.max_feedback_age_s

    def _on_reset(self, _request, response):
        if self.backend != "hardware":
            response.success = False
            response.message = "controller emergency-stop reset is available only on real Nero hardware"
            return response
        if not self.core.connected:
            response.success = False
            response.message = "SDK transport is not connected"
            return response
        fresh_status = self._controller_status_is_fresh()
        arm_status = int(self._controller_status.get("arm_status", -1)) if fresh_status else None
        if fresh_status and arm_status not in {0, 1, 6}:
            response.success = False
            response.message = (f"controller reports {_arm_status_reason(arm_status)} "
                                f"(arm_status={arm_status}); emergency-stop reset cannot clear this fault")
            return response
        if (self.core.enabled or self.core.command_active or self._position_active
                or self._position_future is not None or self._enable_future is not None
                or self._command_future is not None or self._estop_future is not None
                or self._can_init_future is not None or self._reset_future is not None
                or self._can_init_status in {"queued", "waiting_feedback"}
                or self._reset_status in {"queued", "waiting_status"}):
            response.success = False
            response.message = "reset requires disabled motion and no pending driver operation"
            return response
        self.core.revoke()
        self._pending_command = self._pending_hold = None
        self._enable_status = "idle"
        self._reset_seq += 1
        self._position_permitted = False
        self._reset_mode = "unknown_status_recovery" if not fresh_status else "estop"
        self._reset_error = ""
        if arm_status in {0, 6}:
            # Allow a bilateral reset when this side already cleared its latch.
            # Never physically reset a normal/braked peer just to match the other side.
            self._reset_mode = "not_needed"
            self._reset_status = "cleared"
            response.success = True
            response.message = "fresh controller status already has no emergency stop; no reset command sent; prepare/enable remains separate"
            return response
        self._reset_status = "queued"
        self._reset_status_not_before = time.time()
        self._reset_started = time.monotonic()
        self._reset_expired = False
        self._reset_baseline_generation = self._controller_status_generation
        self._reset_future = self._worker.submit(self.sdk.reset)
        response.success = True
        response.message = ("explicit reset and CAN feedback recovery queued; waiting for fresh controller status; "
                            "motors are not enabled and no motion target is sent")
        return response

    def _poll_controller_status(self) -> None:
        future = self._controller_status_future
        if future is not None and future.done():
            self._controller_status_future = None
            try:
                sample = future.result()
                if not isinstance(sample, dict) or "arm_status" not in sample:
                    raise RuntimeError("SDK returned no controller arm status")
                status = dict(sample)
                status["arm_status"] = int(status["arm_status"])
                for key in ("motion_status", "ctrl_mode", "mode_feedback"):
                    if key in status and status[key] is not None:
                        status[key] = int(status[key])
                self._controller_status = status
                self._controller_status_received = time.monotonic()
                self._controller_status_generation += 1
                self._controller_status_last_reset_seq = self._controller_status_future_reset_seq
                self._controller_status_error = ""
            except Exception as exc:
                self._controller_status_error = str(exc)

        if (self._reset_status == "waiting_status"
                and self._controller_status_generation > self._reset_baseline_generation
                and self._controller_status_last_reset_seq == self._reset_seq
                and self._controller_status_is_fresh()
                and float(self._controller_status["timestamp"]) >= self._reset_status_not_before
                and int(self._controller_status.get("arm_status", -1)) in {0, 6}):
            self._reset_status = "cleared"
            self._reset_error = ""
            self.get_logger().warning(
                "Nero controller confirms emergency stop cleared (normal/brake-not-released); prepare/enable remains separate"
            )
        elif (self._reset_status == "waiting_status"
              and time.monotonic() - self._reset_started > float(self.get_parameter("reset_timeout_s").value)):
            self._reset_status = "timed_out"
            if not self._controller_status_is_fresh():
                self._reset_error = "reset was sent but no fresh controller status returned; verify arm power, USB-CAN and CAN wiring; motor enable remains blocked"
            else:
                code = self._controller_status.get("arm_status")
                self._reset_error = f"controller still reports arm_status={code} after reset; motor enable remains blocked"
            self._last_error = self._reset_error

        now = time.monotonic()
        if (self.backend == "hardware" and self.core.connected
                and self._controller_status_future is None
                and now - self._controller_status_schedule >= 0.25):
            self._controller_status_schedule = now
            reader = getattr(self.sdk, "read_controller_status", None)
            if reader is not None:
                self._controller_status_future_reset_seq = self._reset_seq
                self._controller_status_future = self._worker.submit(reader)

    def _state_tick(self) -> None:
        self._poll_connect()
        self._poll_operation("can_init")
        self._poll_operation("enable")
        self._poll_operation("estop")
        self._poll_operation("reset")
        self._poll_operation("position")
        self._poll_operation("read")
        self._poll_controller_status()
        self._check_can_feedback()
        self._check_position_progress()
        now = time.monotonic()
        if self.core.connected and self._read_future is None and (
            now - self._last_feedback_schedule
            >= 1.0 / float(self.get_parameter("state_rate_hz").value)
        ):
            self._last_feedback_schedule = now
            self._read_future = self._worker.submit(self.sdk.read_state)

    def _control_tick(self) -> None:
        requested = self.get_parameter("stream_j_speed_percent").value
        self.sdk.stream_j_speed_percent = requested or int(self.get_parameter("position_speed_percent").value)
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
            "streaming_enabled": self.core.enabled,
            "position_permitted": self._position_permitted,
            "motors_enabled": self._motors_enabled(),
            "motor_status_fresh": self._motor_status_is_fresh(),
            "joint_enable_status": (
                [bool(self.sdk.enabled)] * 7 if self.backend == "fake"
                else self._controller_status.get("joint_enable_status") if self._controller_status else None
            ),
            "motor_status_error": (
                self._controller_status.get("motor_status_error", "") if self._controller_status else ""
            ),
            "can_init_seq": self._can_init_seq,
            "can_init_status": self._can_init_status,
            "can_init_error": self._can_init_error,
            "can_motor_enable_confirmed": self._can_motor_enable_confirmed,
            "feedback_fresh": age is not None and age <= self.core.max_feedback_age_s,
            "feedback_age_s": age,
            "feedback_read_ok": self._feedback_read_ok,
            "feedback_read_error": self._feedback_read_error,
            "feedback_limit_tolerance_deg": float(np.rad2deg(FEEDBACK_LIMIT_TOLERANCE_RAD)),
            "last_feedback_limit_violation": self._last_feedback_limit_violation,
            "command_active": self.core.command_active,
            "position_active": self._position_active,
            "position_seq": self._position_seq,
            "position_status": self._position_status,
            "position_kind": self._position_kind,
            "position_stage": self._position_stage,
            "position_stage_count": len(self._position_targets),
            "position_target": self._position_target.tolist(),
            "native_j7_first": bool(self.get_parameter("native_j7_first").value),
            "stream_position_mode": str(self.get_parameter("stream_position_mode").value),
            "position_speed_percent": self.get_parameter("position_speed_percent").value,
            "stream_j_speed_percent": self.sdk.stream_j_speed_percent,
            "stream_applied_speed_percent": getattr(self.sdk, "_stream_applied_speed_percent", None),
            "position_arrival_tolerance_deg": float(np.rad2deg(self._position_arrival_tolerance_rad)),
            "home_speed_percent": self.get_parameter("home_speed_percent").value,
            "trajectory_revision": TRAJECTORY_REVISION,
            "home_final_pose_deg": list(HOME_FINAL_POSE_DEG),
            "workpos_waypoints_deg": np.asarray(self.get_parameter("workpos_waypoints_deg").value).reshape(4, 7).tolist(),
            "position_error_rad": (
                float(np.max(np.abs(self.core.feedback - self._position_target)))
                if self.core.feedback is not None and self._position_target.shape == (7,) else None
            ),
            "estop_seq": self._estop_seq,
            "estop_status": self._estop_status,
            "reset_seq": self._reset_seq,
            "reset_status": self._reset_status,
            "reset_error": self._reset_error,
            "reset_mode": self._reset_mode,
            "reset_unknown_status_supported": True,
            "controller_status_fresh": self._controller_status_is_fresh(),
            "controller_arm_status": (
                self._controller_status.get("arm_status") if self._controller_status else None
            ),
            "controller_motion_status": (
                self._controller_status.get("motion_status") if self._controller_status else None
            ),
            "controller_mode_feedback": (
                self._controller_status.get("mode_feedback") if self._controller_status else None
            ),
            "controller_status_age_s": (
                max(0.0, time.monotonic() - self._controller_status_received)
                if self._controller_status is not None else None
            ),
            "controller_status_error": self._controller_status_error,
            "enable_status": self._enable_status,
            "enable_started_monotonic_s": getattr(self, "_enable_started", None),
            "last_rejection": self.core.last_rejection,
            "last_error": self._last_error,
        }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def destroy_node(self):
        self._shutdown_requested = True
        try:
            # The exclusive piano launch can stop because a different device
            # failed. Disconnecting CAN alone does not cancel a native J path.
            stopping_motion = self.get_parameter("stop_on_shutdown").value and (
                self.core.enabled or self.core.command_active or self._position_active
                or self._position_future is not None or self._command_future is not None
                or self._enable_future is not None or self._can_init_future is not None)
            self.core.revoke()
            if stopping_motion and self.core.connected:
                # Serialize after any in-flight command and before disconnect;
                # never let a late SDK completion run after the final stop.
                self._worker.submit(self.sdk.estop)
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
