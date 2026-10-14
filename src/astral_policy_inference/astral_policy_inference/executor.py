"""Command executor: safety layer between policy/replay actions and the robot.

Pure (no rclpy) so it is exhaustively unit-testable. Wraps a sequence of
:class:`CmdTarget` rows from :meth:`robot_io.ObsLayout.split_action` and:

* drops NaN/Inf (keeps last good per stream),
* clips joints to configured limits and gripper ratio to [0,1],
* slew-limits joint commands by ``max_joint_vel`` (per-control-step),
* can ``hold()`` the last good targets for pause modes.

Rate limiting is a *safety backstop*, not dynamics shaping: the same values
would otherwise go straight to the driver (teleop's driver applies its own
LPF). ``max_joint_vel<=0`` disables slewing.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from astral_policy_inference.robot_io import CmdTarget


@dataclass
class SafetyEvent:
    stream: str
    kind: str  # nan | clip | slew
    detail: str


@dataclass
class SafeExecutor:
    """Thread-unsafe; call from one control thread only."""

    max_joint_vel: float = 6.0        # rad/s; <=0 disables slew limit
    joint_limits: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    # ratio channel is dimensionless: slew limiting is disabled unless nonzero.
    max_gripper_rate: float = 0.0
    _last: dict[str, np.ndarray] = field(default_factory=dict, init=False)
    _events: list[SafetyEvent] = field(default_factory=list, init=False)

    def reset(self) -> None:
        self._last.clear()
        self._events.clear()

    def seed(self, targets: list[CmdTarget]) -> None:
        """Seed measured/last-executed positions before the first policy action."""
        self.reset()
        for target in targets:
            values = np.asarray(target.values, dtype=np.float64)
            if not np.isfinite(values).all():
                raise ValueError('non-finite executor seed')
            self._last[target.stream] = values.copy()

    @property
    def events(self) -> list[SafetyEvent]:
        out = self._events
        self._events = []
        return out

    def last_for(self, stream: str) -> np.ndarray | None:
        v = self._last.get(stream)
        return None if v is None else v.copy()

    def step(self, targets: list[CmdTarget], dt: float) -> list[CmdTarget]:
        """Safety-pass one set of targets; returns the rows actually to send."""
        dt = max(1e-4, float(dt))
        out: list[CmdTarget] = []
        for t in targets:
            v = np.asarray(t.values, dtype=np.float64)
            if not np.isfinite(v).all():
                last = self._last.get(t.stream)
                if last is not None:
                    self._events.append(
                        SafetyEvent(t.stream, "nan", "non-finite dropped, held last")
                    )
                    out.append(
                        CmdTarget(t.stream, t.topic, t.kind, last.copy())
                    )
                else:
                    self._events.append(
                        SafetyEvent(t.stream, "nan", "non-finite dropped (no last)")
                    )
                continue
            if t.kind == "ratio":
                clipped = np.clip(v, 0.0, 1.0)
                if not np.array_equal(clipped, v):
                    self._events.append(
                        SafetyEvent(
                            t.stream, "clip", f"ratio {np.round(v, 4)} -> [0,1]"
                        )
                    )
                prev = self._last.get(t.stream)
                if self.max_gripper_rate > 0.0 and prev is not None:
                    lim = self.max_gripper_rate * dt
                    delta = clipped - prev
                    delta = np.clip(delta, -lim, lim)
                    clipped = prev + delta
                v = clipped
            else:
                lo_hi = self.joint_limits.get(t.stream)
                if lo_hi is not None:
                    lo, hi = lo_hi
                    clipped = np.clip(v, lo, hi)
                    if not np.array_equal(clipped, v):
                        self._events.append(
                            SafetyEvent(t.stream, "clip", "joint clipped to limits")
                        )
                    v = clipped
                prev = self._last.get(t.stream)
                if self.max_joint_vel > 0.0 and prev is not None:
                    lim = self.max_joint_vel * dt
                    delta = np.clip(v - prev, -lim, lim)
                    if not np.allclose(v, prev + delta):
                        self._events.append(
                            SafetyEvent(
                                t.stream, "slew", f"step limited to {lim:.4f} rad"
                            )
                        )
                    v = prev + delta
            self._last[t.stream] = v.copy()
            out.append(CmdTarget(t.stream, t.topic, t.kind, v.copy()))
        return out
