"""Lazy, serialized adapter around pyAgxArm's Nero API."""

from __future__ import annotations

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
    ) -> None:
        # Do not import the vendor SDK for profile checks, simulation or tests.
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

        firmware = str(firmware_version).strip().lower()
        if firmware not in ("default", "v111"):
            raise ValueError("firmware_version must be 'default' or 'v111'")
        cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            # The vendor API currently spells this keyword ``firmeware_version``.
            firmeware_version=NeroFW.V111 if firmware == "v111" else NeroFW.DEFAULT,
            channel=str(can_channel),
            interface=str(can_interface),
        )
        self._robot = AgxArmFactory.create_arm(cfg)

    def connect(self) -> bool:
        self._robot.connect()
        return True

    def enable(self) -> bool:
        return bool(self._robot.enable())

    def configure_streaming(self) -> None:
        self._robot.set_motion_mode("js")
        self._robot.set_auto_set_motion_mode_enabled(False)

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        joints = self._robot.get_joint_angles()
        flange = self._robot.get_flange_pose()
        if joints is None or flange is None:
            raise RuntimeError("pyAgxArm returned incomplete feedback")
        q = np.asarray(joints.msg, dtype=float)
        pose = np.asarray(flange.msg, dtype=float)
        if q.shape != (7,) or pose.shape != (6,):
            raise RuntimeError(
                f"unexpected pyAgxArm feedback shapes joints={q.shape}, flange={pose.shape}"
            )
        if not np.isfinite(q).all() or not np.isfinite(pose).all():
            raise RuntimeError("pyAgxArm returned non-finite feedback")
        return q.copy(), pose.copy()  # xyz metres + xyz Euler radians

    def move_js(self, q) -> None:
        values = np.asarray(q, dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError("move_js requires exactly seven finite joint values")
        self._robot.move_js(values.tolist())

    def move_j(self, q) -> None:
        values = np.asarray(q, dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError("move_j requires exactly seven finite joint values")
        self._robot.move_j(values.tolist())

    def set_position_mode(self, speed_percent: int) -> None:
        self._robot.set_speed_percent(int(speed_percent))
        self._robot.set_motion_mode("j")
        self._robot.set_auto_set_motion_mode_enabled(False)

    def restore_streaming_mode(self) -> None:
        self.configure_streaming()

    def estop(self) -> None:
        self._robot.electronic_emergency_stop()

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
        self.motion_mode = "js"
        self.calls.append(("mode", "js"))

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        self.calls.append(("read",))
        if self.read_error:
            raise RuntimeError("injected fake feedback failure")
        with self._lock:
            return self.q.copy(), self.flange.copy()

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
        self.motion_mode = "js"
        self.calls.append(("mode", "js"))

    def estop(self) -> None:
        self.calls.append(("estop",))
        self.stopped = True
        self.enabled = False

    def disconnect(self) -> None:
        self.connected = False
        self.enabled = False
