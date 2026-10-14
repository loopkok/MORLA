import numpy as np


class SafetyFilter:
    """Multi-layer safety filter for joint-space commands."""

    def __init__(
        self,
        joint_lower_limits: np.ndarray,
        joint_upper_limits: np.ndarray,
        max_joint_vel: float = 0.15,
        max_ee_vel: float = 0.5,
        workspace_center: np.ndarray = None,
        workspace_radius: float = 0.58,
        workspace_z_min: float = 0.0,
        workspace_z_max: float = 0.8,
        workspace_x_min: float = None,
        workspace_x_max: float = None,
        workspace_y_min: float = None,
        workspace_y_max: float = None,
        collision_check_fn=None,
    ):
        self.joint_lower = joint_lower_limits.copy()
        self.joint_upper = joint_upper_limits.copy()
        self.max_joint_vel = max_joint_vel
        self.max_ee_vel = max_ee_vel
        self.workspace_center = (
            workspace_center if workspace_center is not None else np.zeros(3)
        )
        self.workspace_radius = workspace_radius
        self.workspace_z_min = workspace_z_min
        self.workspace_z_max = workspace_z_max
        # Per-axis box bounds (None = inactive)
        self.workspace_x_min = workspace_x_min
        self.workspace_x_max = workspace_x_max
        self.workspace_y_min = workspace_y_min
        self.workspace_y_max = workspace_y_max
        self.collision_check_fn = collision_check_fn

        self.prev_q = None
        self.prev_ee_pos = None
        self.prev_time = None

    def set_initial_state(self, q: np.ndarray, ee_pos: np.ndarray = None):
        self.prev_q = q.copy()
        if ee_pos is not None:
            self.prev_ee_pos = ee_pos.copy()

    def filter(self, q_target: np.ndarray, dt: float = 0.01) -> tuple:
        """Apply all safety filters.

        Args:
            q_target: Target joint angles (rad)
            dt: Time since last command (seconds)

        Returns:
            (filtered_q, info_dict)
        """
        info = {
            "clamped": False,
            "velocity_limited": False,
            "workspace_limited": False,
            "collision": False,
        }
        q = q_target.copy()

        # 1. Joint limit clamping
        q = np.clip(q, self.joint_lower, self.joint_upper)
        if not np.allclose(q, q_target):
            info["clamped"] = True

        # 2. Joint velocity limiting
        if self.prev_q is not None and dt > 0:
            delta_q = q - self.prev_q
            max_delta = self.max_joint_vel
            exceeded = np.abs(delta_q) > max_delta
            if np.any(exceeded):
                scale = np.ones_like(delta_q)
                scale[exceeded] = max_delta / np.abs(delta_q[exceeded])
                uniform_scale = np.min(scale)
                delta_q = delta_q * uniform_scale
                q = self.prev_q + delta_q
                info["velocity_limited"] = True

        # 3. Self-collision check
        if self.collision_check_fn is not None:
            try:
                if self.collision_check_fn(q):
                    q = self.prev_q if self.prev_q is not None else q_target
                    info["collision"] = True
            except Exception:
                pass

        self.prev_q = q.copy()
        return q, info

    def check_workspace(self, ee_pos: np.ndarray) -> np.ndarray:
        """Clamp EE position to workspace boundaries. Returns clamped position."""
        pos = ee_pos.copy()

        # Spherical boundary
        offset = pos - self.workspace_center
        dist = np.linalg.norm(offset)
        if dist > self.workspace_radius:
            pos = self.workspace_center + offset * (self.workspace_radius / dist)

        # Per-axis XYZ box bounds
        if self.workspace_x_min is not None:
            pos[0] = np.clip(pos[0], self.workspace_x_min, self.workspace_x_max)
        if self.workspace_y_min is not None:
            pos[1] = np.clip(pos[1], self.workspace_y_min, self.workspace_y_max)

        # Z bounds
        pos[2] = np.clip(pos[2], self.workspace_z_min, self.workspace_z_max)

        return pos
