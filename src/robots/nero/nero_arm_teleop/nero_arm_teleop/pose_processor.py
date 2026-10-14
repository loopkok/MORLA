import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class PoseProcessor:
    """VR wrist pose preprocessing: coordinate mapping, smoothing, scaling."""

    def __init__(
        self,
        vr_to_arm_rot: np.ndarray,
        pos_smoothing: float = 0.5,
        rot_smoothing: float = 0.7,
        motion_scale: float = 1.0,
        flip_pitch: bool = True,
    ):
        self.R_vr_to_arm = vr_to_arm_rot
        self.pos_smoothing = pos_smoothing
        self.rot_smoothing = rot_smoothing
        self.motion_scale = motion_scale
        self.flip_pitch = flip_pitch

        self.vr_init_pos = None
        self.vr_init_rot = None
        self.vr_current_pos = None
        self.vr_current_rot = None

        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()

    def set_vr_zero_point(self, pos: np.ndarray, rot: Rotation):
        self.vr_init_pos = pos.copy()
        self.vr_init_rot = rot
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()

    @property
    def is_calibrated(self) -> bool:
        return self.vr_init_pos is not None

    def reset(self):
        self.vr_init_pos = None
        self.vr_init_rot = None
        self.vr_current_pos = None
        self.vr_current_rot = None
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()

    def update_vr_pose(self, pos: np.ndarray, rot: Rotation):
        """Store the latest VR wrist pose. Calibrate on first receive."""
        if self.vr_init_pos is None:
            self.set_vr_zero_point(pos, rot)
        self.vr_current_pos = pos
        self.vr_current_rot = rot

    def process(self) -> tuple:
        """Compute smoothed delta pose from the latest VR data.

        Returns (delta_pos_arm, delta_rot_arm) in the robot arm base frame.
        Must be called after update_vr_pose().
        """
        if self.vr_current_pos is None or self.vr_init_pos is None:
            return np.zeros(3), Rotation.identity()

        # --- Delta in VR frame ---
        raw_delta_pos_vr = (self.vr_current_pos - self.vr_init_pos) * self.motion_scale
        raw_delta_rot_vr = self.vr_current_rot * self.vr_init_rot.inv()

        # Optionally flip pitch axis (VR vs robot convention)
        if self.flip_pitch:
            euler = raw_delta_rot_vr.as_euler("xyz", degrees=False)
            euler[0] = -euler[0]
            raw_delta_rot_vr = Rotation.from_euler("xyz", euler)

        # --- Transform to arm frame ---
        delta_pos_arm = self.R_vr_to_arm @ raw_delta_pos_vr
        delta_rot_arm = Rotation.from_matrix(
            self.R_vr_to_arm @ raw_delta_rot_vr.as_matrix() @ self.R_vr_to_arm.T
        )

        # --- EMA smoothing ---
        alpha_p = self.pos_smoothing
        self.smoothed_delta_pos = alpha_p * self.smoothed_delta_pos + (1 - alpha_p) * delta_pos_arm

        alpha_r = self.rot_smoothing
        try:
            slerp = Slerp([0, 1], Rotation.concatenate([self.smoothed_delta_rot, delta_rot_arm]))
            self.smoothed_delta_rot = slerp(1 - alpha_r)
        except Exception:
            self.smoothed_delta_rot = delta_rot_arm

        return self.smoothed_delta_pos.copy(), self.smoothed_delta_rot

    def compute_target_pose(self, delta_pos: np.ndarray, delta_rot: Rotation,
                            robot_init_pos: np.ndarray, robot_init_rot: np.ndarray) -> np.ndarray:
        """Compute the 4x4 target end-effector pose in the robot base frame.

        Args:
            delta_pos: Smoothed delta position (meters) in arm frame
            delta_rot: Smoothed delta rotation in arm frame
            robot_init_pos: Robot initial EE position [x,y,z] (meters)
            robot_init_rot: Robot initial EE rotation 3x3 matrix
        Returns:
            4x4 homogeneous transformation matrix
        """
        target_pos = robot_init_pos + delta_pos
        target_rot = delta_rot.as_matrix() @ robot_init_rot

        T = np.eye(4)
        T[:3, :3] = target_rot
        T[:3, 3] = target_pos
        return T
