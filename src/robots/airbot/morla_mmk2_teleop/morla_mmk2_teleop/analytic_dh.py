"""Closed-form IK for the MMK2 six-axis arm and its spherical wrist."""

import math

import numpy as np
from morla_core.coordinates import validate_transform

from .urdf_ik import ArmKinematics, vector6


# Modified DH dimensions for airbot_mmk2.urdf, in metres. The wrist-to-TCP
# distance includes the joint6 origin and the fixed gripper-center offset.
BASE_HEIGHT = .1172
UPPER_ARM = .27009
FOREARM = .29015
WRIST_TO_TCP = .23645 + .02
SHOULDER_OFFSET = 2.7549
ELBOW_OFFSET = 1.5 * math.pi - SHOULDER_OFFSET
TCP_ROTATION = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
POSITION_TOLERANCE = 5e-5
ANGLE_TOLERANCE = 1e-4


def _rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


class AnalyticDhKinematics(ArmKinematics):
    method_name = 'mmk2_analytic_dh'
    method_label = 'DH 闭式解析求解'

    def _wrist_solutions(self, rotation, seed):
        sine = math.hypot(rotation[1, 0], rotation[1, 1])
        cosine = rotation[1, 2]
        if sine <= 5e-5 and cosine > 0.:
            # At J5 = 0, only the sum of the two wrist angles is observable.
            # Project the seed onto that sum while respecting both limits.
            total = math.atan2(-rotation[2, 0], rotation[0, 0]) + math.pi / 2
            (lo4, hi4), (lo6, hi6) = self.bounds[[3, 5]]
            for turn in (-math.tau, 0., math.tau):
                angle_sum = total + turn
                lower = max(lo4, angle_sum - hi6)
                upper = min(hi4, angle_sum - lo6)
                if lower <= upper:
                    q4 = np.clip((angle_sum + seed[3] - seed[5]) / 2, lower, upper)
                    yield q4, 0., angle_sum - q4
        if sine > 1e-12:
            for sign in (1, -1):
                yield (
                    math.atan2(sign * rotation[2, 2], -sign * rotation[0, 2]) + math.pi / 2,
                    math.atan2(sign * sine, cosine),
                    math.atan2(-sign * rotation[1, 1], sign * rotation[1, 0]),
                )

    def _candidates(self, target, seed):
        rotation = target[:3, :3] @ TCP_ROTATION.T
        x, y, z = target[:3, 3] - WRIST_TO_TCP * rotation[:, 2]
        radial = math.hypot(x, y)
        height = z - BASE_HEIGHT
        sine = (UPPER_ARM**2 + FOREARM**2 - radial**2 - height**2) / (2 * UPPER_ARM * FOREARM)
        if abs(sine) > 1. + 1e-5:
            return
        sine = float(np.clip(sine, -1., 1.))
        rx90 = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
        for shoulder in (1, -1):
            q1 = math.atan2(shoulder * y, shoulder * x) if radial > 1e-8 else seed[0]
            for elbow in (1, -1):
                cosine = elbow * math.sqrt(max(0., 1. - sine**2))
                theta3 = math.atan2(sine, cosine)
                along = UPPER_ARM - FOREARM * sine
                across = FOREARM * cosine
                theta2 = math.atan2(
                    along * height - across * shoulder * radial,
                    along * shoulder * radial + across * height,
                )
                arm_rotation = _rz(q1) @ rx90 @ _rz(theta2 + theta3)
                for wrist in self._wrist_solutions(arm_rotation.T @ rotation, seed):
                    joints = np.array([q1, theta2 - SHOULDER_OFFSET, theta3 - ELBOW_OFFSET, *wrist])
                    yield (joints + math.pi) % math.tau - math.pi

    def solve(self, target, seed=None, max_joint_step=None):
        began = self.clock()
        raw = validate_transform(target)
        q0 = vector6(self.q if seed is None else seed)
        lower, upper = self.bounds.T
        if np.any(q0 < lower - 1e-8) or np.any(q0 > upper + 1e-8):
            raise ValueError('MMK2 IK seed exceeds deployment joint limits')
        q0 = np.clip(q0, lower, upper)
        if max_joint_step is not None and (
                isinstance(max_joint_step, bool) or not np.isfinite(max_joint_step) or max_joint_step < 0):
            raise ValueError('Invalid MMK2 per-tick joint step')
        self.goal = self.raw_target = raw.copy()
        before = self.command_metrics(q0, raw, raw)
        result, metrics = q0, before
        converged = False
        candidates = sorted(self._candidates(raw, q0), key=lambda q: np.abs(q - q0).sum())
        for candidate in candidates:
            # URDF RPY values are rounded. Allow that numerical tolerance
            # when generating branches, then verify the clipped pose by FK.
            if np.any(candidate < lower - 2e-5) or np.any(candidate > upper + 2e-5):
                continue
            candidate = np.clip(candidate, lower, upper)
            if max_joint_step is not None and np.max(np.abs(candidate - q0)) > max_joint_step:
                continue
            trial = self.command_metrics(candidate, raw, raw)
            if (trial['position_error_m'] <= POSITION_TOLERANCE
                    and trial['angle_error_rad'] <= ANGLE_TOLERANCE):
                result, metrics, converged = candidate, trial, True
                break
        self.q = result.copy()
        step = float(np.max(np.abs(result - q0)))
        self.last_report = {
            'method': self.method_name, 'method_label': self.method_label,
            'reason': 'tracking' if converged else 'holding',
            'converged': converged, 'final_fk_verified': True, 'joint_step_rad': step,
            'cost_before': before['cost'], 'cost_after': metrics['cost'],
            'elapsed_s': self.clock() - began, **metrics,
        }
        return result.tolist()
