"""Lazy, serialized adapter around pyAgxArm's Nero API."""

from __future__ import annotations

import copy
import threading
import time

import numpy as np


class PyAgxArmAdapter:
    """The sole production layer that imports and writes through pyAgxArm."""

    def __init__(
        self,
        can_channel: str,
        can_interface: str = "socketcan",
        firmware_version: str = "default",
        max_feedback_age_s: float = 0.5,
        native_j7_first: bool = False,
        stream_position_mode: str = "js",
        stream_j_speed_percent: int = 5,
    ) -> None:
        # Do not import the vendor SDK for profile checks, simulation or tests.
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

        firmware = str(firmware_version).strip().lower()
        if firmware not in ("default", "v111"):
            raise ValueError("firmware_version must be 'default' or 'v111'")
        self.stream_position_mode = str(stream_position_mode).lower()
        if self.stream_position_mode not in {"js", "j"}:
            raise ValueError("stream_position_mode must be js or j")
        if isinstance(stream_j_speed_percent, bool) or not isinstance(stream_j_speed_percent, int) or not 1 <= stream_j_speed_percent <= 100:
            raise ValueError("stream_j_speed_percent must be an integer in [1,100]")
        self.stream_j_speed_percent = stream_j_speed_percent
        self._stream_goal_mdeg = None
        self.native_j7_first = bool(native_j7_first)
        self._native_move_lock = threading.RLock()
        transport_options = {"local_loopback": True} if self.native_j7_first and can_interface == "socketcan" else {}
        cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            # The vendor API currently spells this keyword ``firmeware_version``.
            firmeware_version=NeroFW.V111 if firmware == "v111" else NeroFW.DEFAULT,
            channel=str(can_channel),
            interface=str(can_interface),
            **transport_options,
        )
        self._robot = AgxArmFactory.create_arm(cfg)
        self._max_feedback_age_s = float(max_feedback_age_s)
        self._feedback_not_before = 0.0

    def connect(self) -> bool:
        self._robot.connect()
        return True

    def enable(self) -> bool:
        return bool(self._robot.enable())

    def initialize_can(self) -> bool:
        # Vendor cold startup includes motor enable. It grants no software
        # command permission and never selects JS or sends a joint target.
        self._feedback_not_before = time.time()
        previous = self._robot._msg_mode
        initialization = copy.copy(previous)
        initialization.ctrl_mode = 0x01
        initialization.move_mode = 0xFF
        initialization.mit_mode = 0x00
        self._robot._msg_mode = initialization
        try:
            enabled = bool(self._robot.enable())
            self._robot.set_normal_mode()
            return enabled
        finally:
            self._robot._msg_mode = previous

    def configure_streaming(self) -> None:
        self._stream_goal_mdeg = None
        if getattr(self, "stream_position_mode", "js") == "j":
            self.set_position_mode(self.stream_j_speed_percent)
        else:
            self._robot.set_motion_mode("js")
            self._robot.set_auto_set_motion_mode_enabled(False)

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        joints = self._robot.get_joint_angles()
        flange = self._robot.get_flange_pose()
        if joints is None or flange is None:
            raise RuntimeError(
                f"pyAgxArm returned incomplete feedback (joints={joints is not None}, flange={flange is not None})"
            )
        now = time.time()
        for label, sample in (("joints", joints), ("flange", flange)):
            stamp = float(sample.timestamp)
            if (not np.isfinite(stamp) or stamp < self._feedback_not_before
                    or not -0.05 <= now - stamp <= self._max_feedback_age_s):
                raise RuntimeError(f"pyAgxArm {label} feedback is stale or predates CAN initialization")
        q = np.asarray(joints.msg, dtype=float)
        pose = np.asarray(flange.msg, dtype=float)
        if q.shape != (7,) or pose.shape != (6,):
            raise RuntimeError(
                f"unexpected pyAgxArm feedback shapes joints={q.shape}, flange={pose.shape}"
            )
        if not np.isfinite(q).all() or not np.isfinite(pose).all():
            raise RuntimeError("pyAgxArm returned non-finite feedback")
        return q.copy(), pose.copy()  # xyz metres + xyz Euler radians

    def read_controller_status(self) -> dict | None:
        """Return the latest vendor arm-status sample without issuing motion."""
        sample = self._robot.get_arm_status()
        if sample is None:
            return None
        msg = sample.msg
        status = {
            "arm_status": int(msg.arm_status),
            "motion_status": int(msg.motion_status),
            "ctrl_mode": int(msg.ctrl_mode),
            "mode_feedback": int(msg.mode_feedback),
            "timestamp": float(sample.timestamp),
        }
        status.update(self._read_motor_enable_status())
        return status

    def _read_motor_enable_status(self) -> dict:
        """Read all seven enable bits with their own source timestamps.

        These are hardware feedback, independent of the ROS streaming gate.
        A cached enable() result or arm_status=0 alone is not motor evidence.
        """
        now = time.time()
        enabled, stamps, errors = [], [], []
        for joint in range(1, 8):
            try:
                sample = self._robot.get_driver_states(joint)
                if sample is None:
                    raise ValueError("missing feedback")
                stamp = float(sample.timestamp)
                bit = bool(sample.msg.foc_status.driver_enable_status)
            except (AttributeError, TypeError, ValueError) as exc:
                enabled.append(None)
                errors.append(f"J{joint}: {exc}")
                continue
            enabled.append(bit)
            stamps.append(stamp)
            if (not np.isfinite(stamp) or stamp < self._feedback_not_before
                    or not -0.05 <= now - stamp <= self._max_feedback_age_s):
                errors.append(f"J{joint}: stale motor enable feedback")
        return {
            "joint_enable_status": enabled,
            "motors_enabled": all(enabled) if all(bit is not None for bit in enabled) else None,
            "motor_status_fresh": not errors,
            "motor_status_timestamp": min(stamps) if len(stamps) == 7 and not errors else None,
            "motor_status_error": "; ".join(errors),
        }

    def move_js(self, q) -> None:
        values = np.asarray(q, dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError("move_js requires exactly seven finite joint values")
        with self._motion_lock():
            if getattr(self, "stream_position_mode", "js") == "j":
                # The piano is position-driven, not teleoperation. Use the
                # controller's J position loop and avoid replanning identical
                # holding goals on every browser/ROS heartbeat.
                encoded = tuple(round(float(np.rad2deg(v))*1000) for v in values)
                if encoded == getattr(self, "_stream_goal_mdeg", None):
                    return
                self.move_j(values)
                self._stream_goal_mdeg = encoded
            else:
                self._robot.move_js(values.tolist())

    def move_j(self, q) -> None:
        values = np.asarray(q, dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError("move_j requires exactly seven finite joint values")
        with self._motion_lock():
            if not getattr(self, "native_j7_first", False):
                self._robot.move_j(values.tolist())
                return
            # The observed one-stage J7 lag is consistent with a J trajectory
            # committing before 0x170; firmware internals are not verified.
            # Keep the public move_j API and its validation/encoding. Only
            # reorder its verified four-message batch, on this instance.
            robot = self._robot
            sender = robot._send_msgs
            had_override = "_send_msgs" in vars(robot)
            old_override = vars(robot).get("_send_msgs")
            expected = [round(float(np.rad2deg(v))*1000) for v in values]
            def send_j7_first(messages):
                fields = (("joint_1", "joint_2"), ("joint_3", "joint_4"),
                          ("joint_5", "joint_6"), ("joint_7",))
                if not isinstance(messages, list) or len(messages) != 4:
                    raise RuntimeError("unsupported Nero SDK J packet batch; no joint target sent")
                encoded = [getattr(message, field, None) for message, names in zip(messages, fields) for field in names]
                if encoded != expected:
                    raise RuntimeError("Nero SDK J packet fields/targets differ; no joint target sent")
                sender([messages[3]])
                # Give 0x170 time to leave the USB-CAN FIFO before the lower
                # CAN IDs of the six-axis batch can trigger the trajectory.
                time.sleep(.005)
                sender(messages[:3])
            robot._send_msgs = send_j7_first
            try:
                robot.move_j(values.tolist())
            finally:
                if had_override:
                    robot._send_msgs = old_override
                else:
                    del robot.__dict__["_send_msgs"]

    def _motion_lock(self):
        # __new__-constructed memory-only test adapters have no constructor.
        if not hasattr(self, "_native_move_lock"):
            self._native_move_lock = threading.RLock()
        return self._native_move_lock

    def set_position_mode(self, speed_percent: int) -> None:
        self._stream_goal_mdeg = None
        self._robot.set_speed_percent(int(speed_percent))
        self._robot.set_motion_mode("j")
        self._robot.set_auto_set_motion_mode_enabled(False)

    def restore_streaming_mode(self) -> None:
        self.configure_streaming()

    def estop(self) -> None:
        self._robot.electronic_emergency_stop()

    def reset(self) -> None:
        # Recovery must work even when CAN push stopped. Neither reset nor
        # requesting push is a motor-enable or joint-target instruction.
        self._feedback_not_before = time.time()
        self._robot.reset()
        previous = self._robot._msg_mode
        recovery = copy.copy(previous)
        recovery.ctrl_mode = 0x01
        recovery.move_mode = 0xFF
        recovery.mit_mode = 0x00
        self._robot._msg_mode = recovery
        try:
            self._robot.set_normal_mode()
        finally:
            self._robot._msg_mode = previous

    def disconnect(self) -> None:
        self._robot.disconnect()


class FakeNeroSDK:
    """Deterministic fake SDK for software tests; never opens a CAN device."""

    def __init__(
        self,
        initial_position,
        *,
        fail_connect: bool = False,
        enable_result: bool = True,
        connect_delay_s: float = 0.0,
        enable_delay_s: float = 0.0,
        read_error: bool = False,
        command_error: bool = False,
    ) -> None:
        q = np.asarray(initial_position, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("fake SDK needs an explicit finite seven-joint seed")
        self._lock = threading.Lock()
        self.q = q.copy()
        self.flange = np.zeros(6, dtype=float)
        self.connected = False
        self.enabled = False
        self.stopped = False
        self.arm_status = 0
        self.motion_mode = "js"
        self.fail_connect = fail_connect
        self.enable_result = enable_result
        self.connect_delay_s = float(connect_delay_s)
        self.enable_delay_s = float(enable_delay_s)
        self.read_error = read_error
        self.command_error = command_error
        self.calls: list[tuple] = []

    def connect(self) -> bool:
        if self.connect_delay_s:
            time.sleep(self.connect_delay_s)
        self.calls.append(("connect",))
        if self.fail_connect:
            raise RuntimeError("injected fake connect failure")
        self.connected = True
        return True

    def enable(self) -> bool:
        if self.enable_delay_s:
            time.sleep(self.enable_delay_s)
        self.calls.append(("enable",))
        if not self.connected or self.stopped:
            return False
        self.enabled = bool(self.enable_result)
        return self.enabled

    def configure_streaming(self) -> None:
        self.motion_mode = getattr(self, "stream_position_mode", "js")
        self.calls.append(("mode", self.motion_mode))

    def initialize_can(self) -> bool:
        self.calls.append(("initialize_can",))
        return self.enable()

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        self.calls.append(("read",))
        if self.read_error:
            raise RuntimeError("injected fake feedback failure")
        with self._lock:
            return self.q.copy(), self.flange.copy()

    def read_controller_status(self) -> dict:
        self.calls.append(("read_controller_status",))
        return {
            "arm_status": self.arm_status,
            "motion_status": 0,
            "ctrl_mode": 1,
            "mode_feedback": 1,
            "timestamp": time.time(),
            "joint_enable_status": [self.enabled] * 7,
            "motors_enabled": self.enabled,
            "motor_status_fresh": self.connected,
            "motor_status_timestamp": time.time(),
            "motor_status_error": "",
        }

    def move_js(self, q) -> None:
        self.calls.append(("move_js",))
        if self.command_error:
            raise RuntimeError("injected fake command failure")
        if not self.enabled or self.stopped:
            raise RuntimeError("fake SDK is not enabled")
        values = np.asarray(q, dtype=float)
        if values.shape != (7,):
            raise ValueError("fake move_js requires seven joints")
        with self._lock:
            self.q = values.copy()

    def move_j(self, q) -> None:
        self.calls.append(("move_j",))
        if not self.enabled or self.stopped:
            raise RuntimeError("fake SDK is not enabled")
        values = np.asarray(q, dtype=float)
        with self._lock:
            self.q = values.copy()

    def set_position_mode(self, _speed_percent: int) -> None:
        self.motion_mode = "j"
        self.calls.append(("mode", "j"))

    def restore_streaming_mode(self) -> None:
        self.configure_streaming()

    def estop(self) -> None:
        self.calls.append(("estop",))
        self.stopped = True
        self.enabled = False
        self.arm_status = 1

    def reset(self) -> None:
        self.calls.append(("reset",))
        self.stopped = False
        self.arm_status = 0

    def disconnect(self) -> None:
        self.connected = False
        self.enabled = False
