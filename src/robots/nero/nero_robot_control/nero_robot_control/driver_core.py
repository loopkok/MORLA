"""ROS-free command validation and watchdog for one Nero arm."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np


# Measurement-only allowance for quantization and small following error at a
# commanded boundary. Command limits stay strict; raw feedback is never clipped.
FEEDBACK_LIMIT_TOLERANCE_RAD = float(np.deg2rad(0.1))


class FeedbackLimitError(ValueError):
    """Measured joints exceed the boundary margin; never masquerade as no CAN."""
    def __init__(self, positions, limits):
        excess = np.maximum(limits[:, 0] - positions, positions - limits[:, 1])
        indices = np.flatnonzero(excess > FEEDBACK_LIMIT_TOLERANCE_RAD)
        self.details = [dict(joint=int(i + 1), measured_deg=float(np.rad2deg(positions[i])),
                             lower_deg=float(np.rad2deg(limits[i, 0])),
                             upper_deg=float(np.rad2deg(limits[i, 1])),
                             excess_deg=float(np.rad2deg(excess[i]))) for i in indices]
        values = "; ".join(f"J{d['joint']} measured={d['measured_deg']:.4f} deg, "
                           f"limits=[{d['lower_deg']:.4f}, {d['upper_deg']:.4f}] deg, "
                           f"excess={d['excess_deg']:.4f} deg" for d in self.details)
        super().__init__(f"SDK feedback exceeds joint limits + 0.1 deg measurement margin: {values}")


@dataclass(frozen=True)
class CommandResult:
    target: np.ndarray
    rate_limited: bool


class NeroDriverCore:
    """Strict seven-joint contract with measured-state seeding and a watchdog."""

    def __init__(
        self,
        side: str,
        limits: np.ndarray,
        max_velocity_rad_s: float = 1.0,
        command_timeout_s: float = 0.5,
        max_feedback_age_s: float = 0.5,
        command_period_s: float = 1.0 / 125.0,
        clock=time.monotonic,
    ) -> None:
        self.side = str(side).strip().lower()
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.joint_names = tuple(f"{self.side}_joint{i}" for i in range(1, 8))
        self.limits = np.asarray(limits, dtype=float)
        if self.limits.shape != (7, 2) or not np.isfinite(self.limits).all():
            raise ValueError("limits must be a finite (7, 2) array")
        if np.any(self.limits[:, 0] >= self.limits[:, 1]):
            raise ValueError("every lower joint limit must be less than its upper limit")
        self.max_velocity_rad_s = float(max_velocity_rad_s)
        self.command_timeout_s = float(command_timeout_s)
        self.max_feedback_age_s = float(max_feedback_age_s)
        self.command_period_s = float(command_period_s)
        if min(
            self.max_velocity_rad_s,
            self.command_timeout_s,
            self.max_feedback_age_s,
            self.command_period_s,
        ) <= 0:
            raise ValueError("velocity and timeout/period values must be positive")
        self._clock = clock
        self.connected = False
        self.enabled = False
        self.feedback: np.ndarray | None = None
        self.feedback_at: float | None = None
        self.last_target: np.ndarray | None = None
        self.last_target_at: float | None = None
        self.command_active = False
        self._watchdog_reported = False
        self.last_rejection: str | None = None

    def set_connected(self, connected: bool) -> None:
        self.connected = bool(connected)
        if not self.connected:
            self.set_enabled(False)

    def set_enabled(self, enabled: bool) -> None:
        if enabled and not self.connected:
            raise RuntimeError("cannot enable a disconnected driver")
        if enabled and self.feedback is None:
            raise RuntimeError("fresh measured joint state is required before enable")
        if enabled and self.feedback_age() > self.max_feedback_age_s:
            raise RuntimeError("measured joint state is stale; enable rejected")
        self.enabled = bool(enabled) and self.connected
        # Every enable epoch starts from actual feedback, never a stale cache.
        self.last_target = np.clip(self.feedback, self.limits[:, 0], self.limits[:, 1]) if self.enabled and self.feedback is not None else None
        self.last_target_at = None
        self.command_active = False
        self._watchdog_reported = False

    def revoke(self) -> None:
        self.set_enabled(False)

    def update_feedback(self, positions, received_at: float | None = None) -> None:
        q = np.asarray(positions, dtype=float)
        if q.shape != (7,):
            raise ValueError(f"SDK feedback must contain exactly 7 joints, got {q.shape}")
        if not np.isfinite(q).all():
            raise ValueError("SDK feedback contains non-finite joint values")
        excess = np.maximum(self.limits[:, 0] - q, q - self.limits[:, 1])
        if np.any(excess > FEEDBACK_LIMIT_TOLERANCE_RAD):
            raise FeedbackLimitError(q, self.limits)
        self.feedback = q.copy()
        self.feedback_at = self._clock() if received_at is None else float(received_at)
        if self.enabled and self.last_target is None:
            self.last_target = np.clip(q, self.limits[:, 0], self.limits[:, 1])

    def reorder_command(self, names, positions) -> np.ndarray:
        names = tuple(str(name) for name in names)
        values = np.asarray(positions, dtype=float)
        if len(names) != 7 or len(set(names)) != 7 or values.shape != (7,):
            raise ValueError("command requires exactly 7 unique names and 7 positions")
        if set(names) != set(self.joint_names):
            missing = sorted(set(self.joint_names) - set(names))
            extra = sorted(set(names) - set(self.joint_names))
            raise ValueError(f"joint name mismatch; missing={missing}, extra={extra}")
        if not np.isfinite(values).all():
            raise ValueError("command contains non-finite positions")
        by_name = dict(zip(names, values))
        return np.asarray([by_name[name] for name in self.joint_names], dtype=float)

    def accept_command(self, names, positions, received_at: float | None = None) -> CommandResult:
        now = self._clock() if received_at is None else float(received_at)
        if not self.connected:
            raise RuntimeError("driver is not connected")
        if not self.enabled:
            raise RuntimeError("driver is not explicitly enabled")
        if self.feedback is None or self.feedback_at is None:
            raise RuntimeError("no measured joint state is available")
        if now - self.feedback_at > self.max_feedback_age_s:
            raise RuntimeError("measured joint state is stale")
        target = self.reorder_command(names, positions)
        if np.any(target < self.limits[:, 0]) or np.any(target > self.limits[:, 1]):
            raise ValueError("command is outside configured joint limits")

        if self.last_target is None:
            self.last_target = np.clip(self.feedback, self.limits[:, 0], self.limits[:, 1])
        period = (
            self.command_period_s
            if self.last_target_at is None
            else max(0.0, now - self.last_target_at)
        )
        max_step = self.max_velocity_rad_s * period
        delta = target - self.last_target
        limited = np.clip(delta, -max_step, max_step)
        output = self.last_target + limited
        self.last_target = output.copy()
        self.last_target_at = now
        self.command_active = True
        self._watchdog_reported = False
        self.last_rejection = None
        return CommandResult(output.copy(), bool(np.any(np.abs(delta) > max_step + 1e-12)))

    def watchdog_hold(self, now: float | None = None) -> np.ndarray | None:
        """Return one measured-seeded hold target after command timeout."""
        now = self._clock() if now is None else float(now)
        if not self.command_active or self.last_target_at is None:
            return None
        if now - self.last_target_at <= self.command_timeout_s:
            return None
        self.command_active = False
        if self._watchdog_reported:
            return None
        self._watchdog_reported = True
        return self.last_target.copy()

    def feedback_age(self, now: float | None = None) -> float | None:
        if self.feedback_at is None:
            return None
        now = self._clock() if now is None else float(now)
        return max(0.0, now - self.feedback_at)
