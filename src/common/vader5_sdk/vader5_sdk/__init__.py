"""Minimal Vader 5 Pro input SDK. GPL-3.0-or-later; see ../NOTICE."""
from __future__ import annotations

from dataclasses import dataclass
import math
import sys
import time

from ._vendor import protocol

BUTTON_NAMES = protocol.BUTTON_NAMES


@dataclass(frozen=True)
class State:
    """One received report. Axes retain the device's signs and coordinate frame."""

    received_at: float  # host monotonic seconds, not an IMU sampling timestamp
    buttons: frozenset[str]
    left_stick: tuple[float, float]  # normalized [-1, 1], no deadzone
    right_stick: tuple[float, float]
    triggers: tuple[float, float]  # left/right, [0, 1]
    gyro_rad_s: tuple[float, float, float]
    accel_m_s2: tuple[float, float, float]  # includes gravity
    raw: protocol.InputState

    def pressed(self, name: str) -> bool:
        key = name.upper()
        if key not in BUTTON_NAMES:
            raise ValueError(f"Unknown button {name!r}; valid names: {BUTTON_NAMES}")
        return key in self.buttons


def _normalize(value: int) -> float:
    return value / (32768.0 if value < 0 else 32767.0)


def _state(raw: protocol.InputState) -> State:
    return State(
        time.monotonic(), raw.buttons,
        tuple(_normalize(v) for v in raw.left_stick),
        tuple(_normalize(v) for v in raw.right_stick),
        (raw.left_trigger / 255.0, raw.right_trigger / 255.0),
        tuple(math.radians(v) for v in raw.gyro_dps),
        tuple(v * 9.80665 for v in raw.accel_g), raw,
    )


class Vader5:
    """Linux USB cable/receiver reader. read() returns a fresh report or raises.

    Use as a context manager; only one application should own the vendor stream.
    read() returns the newest report in each batch (intermediate edges may be
    skipped). Use read_batch() when recording all received button transitions.
    """

    def __init__(self, device: str | None = None):
        self.device = device
        self._controller = None

    def open(self):
        if self._controller is not None:
            return self
        if sys.platform != "linux":
            raise RuntimeError("This backend requires Linux hidraw; macOS/Windows are not supported.")
        from ._vendor.device import Controller
        controller = Controller(self.device)
        try:
            controller.open()
            controller.request_info()
        except BaseException:
            controller.close()
            raise
        self._controller = controller
        return self

    @property
    def info(self):
        """Firmware/model information, available after receiving its reply."""
        return self._controller.info if self._controller else None

    def read_batch(self, timeout: float = 1.0) -> list[State]:
        if self._controller is None:
            raise RuntimeError("Call open() first or use 'with Vader5() as pad'.")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("No fresh Vader 5 Pro input; check cable, mode, power and competing apps.")
            reports = self._controller.poll(min(remaining, 0.1))
            if self.info is not None and self.info.device_id != 130:
                raise RuntimeError(f"Expected Vader 5 Pro, got device ID {self.info.device_id}")
            if reports:
                return [_state(report) for report in reports]

    def read(self, timeout: float = 1.0) -> State:
        return self.read_batch(timeout)[-1]

    def close(self):
        controller, self._controller = self._controller, None
        if controller is not None:
            controller.close()

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
