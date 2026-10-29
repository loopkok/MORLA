"""Normalized input identity, freshness and reconnect fencing."""
from __future__ import annotations
from dataclasses import dataclass
import math
import secrets
import time
from typing import Any
from copy import deepcopy


class InputError(ValueError):
    pass


def finite(value):
    if isinstance(value, dict):
        return all(isinstance(k, str) and finite(v) for k, v in value.items())
    if isinstance(value, (list, tuple)):
        return len(value) <= 1024 and all(finite(v) for v in value)
    return (isinstance(value, (str, bool))
            or (isinstance(value, (int, float)) and math.isfinite(value)))


@dataclass(frozen=True)
class InputFrame:
    source_id: str
    source_epoch: str
    sequence: int
    received_ns: int
    frame_id: str
    ports: dict[str, Any]
    source_time_ns: int | None = None
    clock_domain: str = "host_monotonic"
    valid: bool = True
    port_metadata: dict | None = None
    synthetic: bool = False


class InputStream:
    def __init__(self, source_id, *, clock=time.monotonic_ns, max_age_ns=250_000_000):
        self.source_id, self.clock, self.max_age_ns = source_id, clock, max_age_ns
        self.epoch = secrets.token_hex(16)
        self.sequence = -1
        self.latest = None
        self.connected = False

    def reconnect(self):
        self.epoch = secrets.token_hex(16)
        self.sequence = -1
        self.latest = None
        self.connected = True
        return self.epoch

    def disconnect(self):
        self.connected = False
        self.latest = None

    def accept(self, ports, sequence, *, epoch, frame_id="input", source_time_ns=None):
        if not self.connected or epoch != self.epoch:
            raise InputError("unknown or retired input epoch")
        if type(sequence) is not int or sequence <= self.sequence:
            raise InputError("duplicate or out-of-order input sequence")
        if not isinstance(ports, dict) or len(ports) > 64 or not finite(ports):
            raise InputError("invalid input payload")
        if not isinstance(frame_id, str) or not 1 <= len(frame_id) <= 128:
            raise InputError("invalid frame identity")
        now = self.clock()
        # A remote client clock is deliberately not compared to the host clock.
        # Its clock-domain sample is metadata; local arrival drives the watchdog.
        if source_time_ns is not None and (type(source_time_ns) is not int or source_time_ns < 0):
            raise InputError("invalid source time")
        self.sequence = sequence
        self.latest = InputFrame(self.source_id, epoch, sequence, now, frame_id,
                                 deepcopy(ports), source_time_ns)
        return self.latest

    def read(self):
        if (not self.connected or self.latest is None
                or not 0 <= self.clock() - self.latest.received_ns <= self.max_age_ns):
            return None
        return self.latest


class GamepadAdapter(InputStream):
    AXES = ("left_x", "left_y", "right_x", "right_y", "left_trigger", "right_trigger")
    # SDL_GameController button enumeration, used by ROS game_controller_node.
    BUTTONS = ("a", "b", "x", "y", "back", "guide", "start", "left_stick",
               "right_stick", "left_shoulder", "right_shoulder", "dpad_up",
               "dpad_down", "dpad_left", "dpad_right", "misc1", "paddle1",
               "paddle2", "paddle3", "paddle4", "touchpad")

    def __init__(self, source_id, *, deadzone=0.08, **kwargs):
        super().__init__(source_id, **kwargs)
        if not 0 <= deadzone < 1:
            raise InputError("invalid deadzone")
        self.deadzone = deadzone

    def joy(self, axes, buttons):
        if len(axes) != len(self.AXES) or len(buttons) not in (15, len(self.BUTTONS)):
            raise InputError("SDL game_controller_node mapping required; unknown layout")
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) or abs(v) > 1 for v in axes):
            raise InputError("gamepad axes must be finite [-1,1]")
        if any(type(v) not in (bool, int) or v not in (0, 1) for v in buttons):
            raise InputError("invalid gamepad button")
        mapped = {name: 0.0 if abs(v) <= self.deadzone else float(v)
                  for name, v in zip(self.AXES, axes)}
        keys = dict(zip(self.BUTTONS, map(bool, buttons)))
        return self.accept({"axes": mapped, **keys}, self.sequence + 1, epoch=self.epoch)


class MobileAdapter(InputStream):
    def __init__(self, source_id, **kwargs):
        super().__init__(source_id, **kwargs)
        self.pairing_token = secrets.token_urlsafe(32)

    def authenticate(self, token):
        return isinstance(token, str) and secrets.compare_digest(token, self.pairing_token)

    def packet(self, raw):
        fields = {"epoch", "sequence", "axes", "scalar", "hold_to_control", "source_time_ns"}
        if (not isinstance(raw, dict) or set(raw) - fields
                or not {"epoch", "sequence", "hold_to_control"} <= set(raw)):
            raise InputError("invalid mobile packet fields")
        axes = raw.get("axes", {})
        if not isinstance(axes, dict) or set(axes) - {"x", "y", "z", "roll", "pitch", "yaw"}:
            raise InputError("invalid mobile axes")
        scalar = raw.get("scalar", 0.0)
        if type(raw["hold_to_control"]) is not bool:
            raise InputError("deadman must be boolean")
        for v in list(axes.values()) + [scalar]:
            if (isinstance(v, bool) or not isinstance(v, (int, float))
                    or not math.isfinite(v) or abs(v) > 1):
                raise InputError("mobile values must be finite [-1,1]")
        return self.accept({"axes": axes, "scalar": scalar,
                            "hold_to_control": raw["hold_to_control"]}, raw["sequence"],
                           epoch=raw["epoch"], source_time_ns=raw.get("source_time_ns"))


class PoseAdapter(InputStream):
    """Quest/glove bridge after their own device protocol and clock guards."""

    def pose(self, port, position, quaternion, *, frame_id, sequence, epoch):
        numbers = [*position, *quaternion]
        if (len(position) != 3 or len(quaternion) != 4
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in numbers)):
            raise InputError("invalid pose shape")
        norm = sum(v*v for v in quaternion)
        if not math.isclose(norm, 1.0, abs_tol=1e-3):
            raise InputError("quaternion must be normalized")
        return self.accept({port: {"position": list(position), "quaternion": list(quaternion)}},
                           sequence, epoch=epoch, frame_id=frame_id)
