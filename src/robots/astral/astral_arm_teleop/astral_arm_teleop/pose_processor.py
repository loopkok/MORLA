"""VR wrist pose preprocessing (Nero-compatible).

VR delta → optional pitch flip → ``vr_to_arm_rot`` → EMA/SLERP.

Smoothing: yaml ``pos_smoothing`` / ``rot_smoothing`` are 0–1 filter
coefficients (0 = follow immediately, 1 = hold). They are calibrated at
50 Hz and converted to a time constant, so changing ``control_rate`` does
not change how quickly the pose tracks.

For Astral the mapped frame is that arm's ``*_base_link``.
``robot_world_to_base_rot`` is kept as an alias of ``vr_to_arm_rot``.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# Legacy per-tick α=0.8 at 50 Hz → τ = -dt / ln(α)
_LEGACY_DT = 0.02


def _tau_from_legacy_alpha(alpha: float) -> float:
    a = float(alpha)
    if a <= 0.0:
        return 0.0
    if a >= 1.0:
        return 1e6
    return -_LEGACY_DT / math.log(a)


class PoseProcessor:
    """Incremental VR wrist → delta pose in the robot/IK frame."""

    def __init__(
        self,
        vr_to_arm_rot: np.ndarray | None = None,
        robot_world_to_base_rot: np.ndarray | None = None,
        pos_smoothing_tau: float | None = None,
        rot_smoothing_tau: float | None = None,
        pos_smoothing: float | None = None,
        rot_smoothing: float | None = None,
        motion_scale: float = 0.65,
        flip_pitch: bool = False,
        auto_calibrate: bool = True,
    ):
        if vr_to_arm_rot is not None:
            R = np.asarray(vr_to_arm_rot, dtype=float).reshape(3, 3)
        elif robot_world_to_base_rot is not None:
            R = np.asarray(robot_world_to_base_rot, dtype=float).reshape(3, 3)
        else:
            R = np.eye(3, dtype=float)
        self.R_vr_to_arm = R
        self.R_world_to_base = R  # alias
        # 0–1 filter coeff, calibrated at 50 Hz. Internally converted to a
        # time constant so the feel does not change with control_rate.
        if pos_smoothing is None:
            pos_smoothing = 0.8
        if rot_smoothing is None:
            rot_smoothing = 0.8
        self.pos_smoothing = float(np.clip(pos_smoothing, 0.0, 1.0))
        self.rot_smoothing = float(np.clip(rot_smoothing, 0.0, 1.0))
        if pos_smoothing_tau is not None:
            self.pos_smoothing_tau = float(pos_smoothing_tau)
        else:
            self.pos_smoothing_tau = _tau_from_legacy_alpha(self.pos_smoothing)
        if rot_smoothing_tau is not None:
            self.rot_smoothing_tau = float(rot_smoothing_tau)
        else:
            self.rot_smoothing_tau = _tau_from_legacy_alpha(self.rot_smoothing)
        self.motion_scale = float(motion_scale)
        self.flip_pitch = bool(flip_pitch)
        # When True (default, legacy behavior) the first received VR pose becomes
        # the vr_init zero point automatically. When False the node only tracks
        # vr_current and waits for an external calibrate_from_current() call —
        # used by require_start_signal so the user can place their hand at the
        # desired initial pose before the zero is captured.
        self.auto_calibrate = bool(auto_calibrate)

        self.vr_init_pos = None
        self.vr_init_rot = None
        self.vr_current_pos = None
        self.vr_current_rot = None
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()
        # Unfiltered arm-frame delta from the last process() (tune plots).
        self.last_raw_delta_pos = np.zeros(3)
        self.last_raw_delta_rot = Rotation.identity()

    def set_vr_zero_point(self, pos: np.ndarray, rot: Rotation) -> None:
        self.vr_init_pos = pos.copy()
        self.vr_init_rot = rot
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()
        self.last_raw_delta_pos = np.zeros(3)
        self.last_raw_delta_rot = Rotation.identity()

    def calibrate_from_current(self) -> bool:
        """Capture vr_init from the latest received VR pose.

        Returns False if no VR pose has been received yet. Used by the
        require_start_signal flow: the node tracks poses without auto-zeroing,
        then the user triggers this once their hand is at the desired initial
        pose.
        """
        if self.vr_current_pos is None or self.vr_current_rot is None:
            return False
        self.set_vr_zero_point(self.vr_current_pos, self.vr_current_rot)
        return True

    @property
    def is_calibrated(self) -> bool:
        return self.vr_init_pos is not None

    def reset(self) -> None:
        self.vr_init_pos = None
        self.vr_init_rot = None
        self.vr_current_pos = None
        self.vr_current_rot = None
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()
        self.last_raw_delta_pos = np.zeros(3)
        self.last_raw_delta_rot = Rotation.identity()

    def update_vr_pose(self, pos: np.ndarray, rot: Rotation) -> None:
        # A single non-finite pose would permanently poison the EMA state
        # (a*NaN + (1-a)*x = NaN forever); drop it at the entry point.
        if not np.isfinite(pos).all() or not np.isfinite(rot.as_quat()).all():
            return
        if self.auto_calibrate and self.vr_init_pos is None:
            self.set_vr_zero_point(pos, rot)
        self.vr_current_pos = pos
        self.vr_current_rot = rot

    @staticmethod
    def _alpha(tau: float, dt: float) -> float:
        if tau <= 0.0:
            return 0.0
        return float(math.exp(-max(1e-6, dt) / tau))

    def process(self, dt: float = 0.02):
        if self.vr_current_pos is None or self.vr_init_pos is None:
            return np.zeros(3), Rotation.identity()

        raw_delta_pos_vr = (self.vr_current_pos - self.vr_init_pos) * self.motion_scale
        raw_delta_rot_vr = self.vr_current_rot * self.vr_init_rot.inv()

        if self.flip_pitch:
            euler = raw_delta_rot_vr.as_euler("xyz", degrees=False)
            euler[0] = -euler[0]
            raw_delta_rot_vr = Rotation.from_euler("xyz", euler)

        R = self.R_vr_to_arm
        delta_pos_arm = R @ raw_delta_pos_vr
        delta_rot_arm = Rotation.from_matrix(
            R @ raw_delta_rot_vr.as_matrix() @ R.T
        )
        self.last_raw_delta_pos = delta_pos_arm.copy()
        self.last_raw_delta_rot = delta_rot_arm

        a_p = self._alpha(self.pos_smoothing_tau, dt)
        self.smoothed_delta_pos = (
            a_p * self.smoothed_delta_pos + (1.0 - a_p) * delta_pos_arm
        )
        a_r = self._alpha(self.rot_smoothing_tau, dt)
        try:
            slerp = Slerp(
                [0, 1],
                Rotation.concatenate([self.smoothed_delta_rot, delta_rot_arm]),
            )
            self.smoothed_delta_rot = slerp(1.0 - a_r)
        except Exception:  # noqa: BLE001
            self.smoothed_delta_rot = delta_rot_arm

        return self.smoothed_delta_pos.copy(), self.smoothed_delta_rot

    def set_pos_smoothing(self, alpha: float) -> None:
        self.pos_smoothing = float(np.clip(alpha, 0.0, 1.0))
        self.pos_smoothing_tau = _tau_from_legacy_alpha(self.pos_smoothing)

    def set_rot_smoothing(self, alpha: float) -> None:
        self.rot_smoothing = float(np.clip(alpha, 0.0, 1.0))
        self.rot_smoothing_tau = _tau_from_legacy_alpha(self.rot_smoothing)

    def set_motion_scale(self, scale: float) -> None:
        self.motion_scale = float(max(0.0, scale))

    def set_flip_pitch(self, flip: bool) -> None:
        self.flip_pitch = bool(flip)

    def compute_target_pose(
        self,
        delta_pos: np.ndarray,
        delta_rot: Rotation,
        robot_init_pos: np.ndarray,
        robot_init_rot: np.ndarray,
    ) -> np.ndarray:
        """4x4 EE target (same composition as Nero)."""
        T = np.eye(4)
        T[:3, 3] = robot_init_pos + delta_pos
        T[:3, :3] = delta_rot.as_matrix() @ robot_init_rot
        return T
