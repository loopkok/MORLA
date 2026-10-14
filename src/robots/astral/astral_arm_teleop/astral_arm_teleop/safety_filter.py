"""Joint-space safety filter (ported from nero_quest_teleop)."""

from __future__ import annotations

import numpy as np


class SafetyFilter:
    def __init__(
        self,
        joint_lower_limits: np.ndarray,
        joint_upper_limits: np.ndarray,
        max_joint_vel: float = 4.0,
        workspace_center: np.ndarray | None = None,
        workspace_radius: float = 0.0,
        workspace_z_min: float | None = None,
        workspace_z_max: float | None = None,
        workspace_x_min: float | None = None,
        workspace_x_max: float | None = None,
        workspace_y_min: float | None = None,
        workspace_y_max: float | None = None,
        collision_check_fn=None,
    ):
        self.joint_lower = np.asarray(joint_lower_limits, dtype=float).copy()
        self.joint_upper = np.asarray(joint_upper_limits, dtype=float).copy()
        # rad/s; per-tick cap is ``max_joint_vel * dt``
        self.max_joint_vel = float(max_joint_vel)
        self.workspace_center = (
            np.zeros(3)
            if workspace_center is None
            else np.asarray(workspace_center, dtype=float)
        )
        self.workspace_radius = float(workspace_radius)
        self.workspace_z_min = workspace_z_min
        self.workspace_z_max = workspace_z_max
        self.workspace_x_min = workspace_x_min
        self.workspace_x_max = workspace_x_max
        self.workspace_y_min = workspace_y_min
        self.workspace_y_max = workspace_y_max
        self.collision_check_fn = collision_check_fn
        self.prev_q = None

    def set_initial_state(self, q: np.ndarray, ee_pos: np.ndarray = None) -> None:
        self.prev_q = np.asarray(q, dtype=float).copy()

    def filter(self, q_target: np.ndarray, dt: float = 0.01):
        info = {
            "clamped": False,
            "velocity_limited": False,
            "collision": False,
            "invalid": False,
        }
        q = np.asarray(q_target, dtype=float)
        # NaN/Inf barrier: np.clip and the velocity gate below both pass NaN
        # through (NaN comparisons are False), and storing NaN into prev_q
        # would additionally disable the velocity limit on the *next* valid
        # tick (delta = valid - NaN = NaN → never "exceeded"). Hold the last
        # valid command and leave prev_q untouched instead.
        if not np.isfinite(q).all():
            info["invalid"] = True
            if self.prev_q is not None:
                return self.prev_q.copy(), info
            return np.zeros_like(q), info
        q = q.copy()
        clipped = np.clip(q, self.joint_lower, self.joint_upper)
        if not np.allclose(clipped, q):
            info["clamped"] = True
        q = clipped

        if self.prev_q is not None and dt > 0 and self.max_joint_vel > 0:
            delta = q - self.prev_q
            max_d = self.max_joint_vel * float(dt)
            exceeded = np.abs(delta) > max_d
            if np.any(exceeded):
                scale = np.ones_like(delta)
                scale[exceeded] = max_d / np.abs(delta[exceeded])
                q = self.prev_q + delta * float(np.min(scale))
                info["velocity_limited"] = True

        if self.collision_check_fn is not None:
            try:
                if self.collision_check_fn(q):
                    q = self.prev_q if self.prev_q is not None else q_target
                    info["collision"] = True
            except Exception:  # noqa: BLE001
                pass

        self.prev_q = q.copy()
        return q, info

    def check_workspace(self, ee_pos: np.ndarray) -> np.ndarray:
        pos = np.asarray(ee_pos, dtype=float).copy()
        if self.workspace_radius > 0:
            offset = pos - self.workspace_center
            dist = float(np.linalg.norm(offset))
            if dist > self.workspace_radius:
                pos = self.workspace_center + offset * (
                    self.workspace_radius / dist
                )
        if self.workspace_x_min is not None and self.workspace_x_max is not None:
            pos[0] = np.clip(pos[0], self.workspace_x_min, self.workspace_x_max)
        if self.workspace_y_min is not None and self.workspace_y_max is not None:
            pos[1] = np.clip(pos[1], self.workspace_y_min, self.workspace_y_max)
        if self.workspace_z_min is not None and self.workspace_z_max is not None:
            pos[2] = np.clip(pos[2], self.workspace_z_min, self.workspace_z_max)
        return pos
