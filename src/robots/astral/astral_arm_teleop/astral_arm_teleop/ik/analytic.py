#!/usr/bin/env python3
"""Astral arm analytic IK — Modified DH geometric method (Nero algorithm).

Ported from ``nero_quest_teleop.ik_solver`` (pure numpy + scipy).
No Pinocchio / CasADI / numerical LM. DH parameters are Astral link lengths
with the same SRS / arm-angle structure as Nero.

Public API matches Nero: ``IKSolver.solve(T)`` / ``fk(q)`` / ``sync_state(q)``.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize_scalar


# ==============================================================================
# Utility functions
# ==============================================================================

def wrap_to_pi(x: np.ndarray) -> np.ndarray:
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def pose_error(T_cur: np.ndarray, T_des: np.ndarray) -> np.ndarray:
    dp = T_cur[:3, 3] - T_des[:3, 3]
    R = T_cur[:3, :3]
    Rd = T_des[:3, :3]
    re = 0.5 * (
        np.cross(R[:, 0], Rd[:, 0])
        + np.cross(R[:, 1], Rd[:, 1])
        + np.cross(R[:, 2], Rd[:, 2])
    )
    return np.hstack([dp, re])


def _rotz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _rotx(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def _dh_A(theta: float, d: float, a_prev: float, alpha_prev: float) -> np.ndarray:
    # Modified DH: T_i^{i-1} = RotX(alpha_{i-1}) * TransX(a_{i-1}) * RotZ(theta_i) * TransZ(d_i)
    T = np.eye(4)
    Rx = _rotx(alpha_prev)
    Rz = _rotz(theta)
    T[:3, :3] = Rx @ Rz
    T[:3, 3] = np.array(
        [a_prev, -math.sin(alpha_prev) * d, math.cos(alpha_prev) * d],
        dtype=float,
    )
    return T


def _invert_rigid_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Rt = R.T
    out = np.eye(4, dtype=float)
    out[:3, :3] = Rt
    out[:3, 3] = -(Rt @ t)
    return out


def _remove_post_transform(T07: np.ndarray, p: "AstralParams") -> np.ndarray:
    """Equivalent to T07 @ inv(T_post) where T_post = TransZ(post_transform_d8)."""
    T_chain = np.array(T07, dtype=float, copy=True)
    T_chain[:3, 3] = T07[:3, 3] - float(p.post_transform_d8) * T07[:3, 2]
    return T_chain


def _axis_angle_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    u = axis / max(np.linalg.norm(axis), 1e-12)
    ux, uy, uz = u
    K = np.array([[0.0, -uz, uy], [uz, 0.0, -ux], [-uy, ux, 0.0]], dtype=float)
    I = np.eye(3)
    return I + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


# ==============================================================================
# Parameter dataclasses
# ==============================================================================

@dataclass
class AstralParams:
    a_prev: np.ndarray
    alpha_prev: np.ndarray
    d_i: np.ndarray
    theta_offset: np.ndarray
    joint_limits: np.ndarray
    post_transform_d8: float
    theta0_coarse_divisor: int
    theta0_fine_count: int

    @staticmethod
    def _base(
        d_i: np.ndarray,
        theta_offset: np.ndarray,
        joint_limits: np.ndarray,
    ) -> "AstralParams":
        return AstralParams(
            a_prev=np.zeros(7, dtype=float),
            alpha_prev=np.deg2rad(
                np.array([0.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0], dtype=float)
            ),
            # Positive MDH (alpha = +90 deg spherical S-R-S). a_prev = 0 exactly:
            # the joint axes are spherical (verified via fit_dh_from_urdf); the
            # URDF link-origin lateral offsets are visual, not MDH a_prev.
            # The joint axes of the astral arm are "wound" with mixed alpha sign;
            # we flip the positive direction of joints 2,3,4 (left) / 2,4 (right)
            # in the URDF/MJCF so the arm is all +90 deg. theta_offset encodes the
            # neutral (elbow-forward) pose in that flipped convention.
            d_i=np.asarray(d_i, dtype=float),
            theta_offset=np.deg2rad(np.asarray(theta_offset, dtype=float)),
            joint_limits=np.asarray(joint_limits, dtype=float),
            post_transform_d8=0.0,
            theta0_coarse_divisor=3,
            theta0_fine_count=41,
        )

    @staticmethod
    def default() -> "AstralParams":
        return AstralParams.left_arm()

    @staticmethod
    def left_arm() -> "AstralParams":
        # Clean positive-MDH params (fit_dh_from_urdf ground truth), in the
        # joint-axis-flip convention: joints 2,3,4 axis positive direction flipped
        # so alpha = +90 deg. d_i[0] = -0.11655 (shoulder base along joint1 axis,
        # negative because joint1 axis = -X in the SolidWorks base frame).
        # Joint limits are the URDF limits negated on the flipped joints.
        return AstralParams._base(
            d_i=np.array(
                [-0.11655, 0.0, 0.246, 0.0, 0.2265, 0.0, 0.0036], dtype=float
            ),
            theta_offset=np.array(
                [180.0, 90.0, 90.0, 180.0, 180.0, -90.0, -90.0], dtype=float
            ),
            joint_limits=np.array(
                [
                    [-2.0, 2.0],
                    [-2.0, 2.0],
                    [-2.2689, 2.2689],
                    [-1.012, 2.26],
                    [-1.7802, 1.7802],
                    [-0.8, 0.85],
                    [-1.57, 1.57],
                ],
                dtype=float,
            ),
        )

    @staticmethod
    def right_arm() -> "AstralParams":
        # Mirrored elbow: joints 2,4 flipped (alpha = +90 deg), elbow theta4 in
        # [0, pi] with theta_offset[3] = 0 (neutral = straight arm). The upper
        # arm d2 is negative (mirror of left's positive) because joints 2,3,4
        # are NOT all flipped for the right arm.
        return AstralParams._base(
            d_i=np.array(
                [0.11655, 0.0, -0.246, 0.0, 0.2265, 0.0, 0.0036], dtype=float
            ),
            theta_offset=np.array(
                [180.0, -90.0, -90.0, 0.0, 180.0, -90.0, -90.0], dtype=float
            ),
            joint_limits=np.array(
                [
                    [-2.0, 2.0],
                    [-2.0, 2.0],
                    [-2.2689, 2.2689],
                    [-1.012, 2.26],
                    [-1.7802, 1.7802],
                    [-0.8, 0.85],
                    [-1.57, 1.57],
                ],
                dtype=float,
            ),
        )


# Fixed joint1 origin rpy of left/right_joint1 relative to *_base_link
# (astral_robot.pin.urdf). Applied outside the MDH chain so FK/IK are in arm base.
_ARM_BASE_FIXED_RPY = np.array([-0.5 * np.pi, 0.0, 0.5 * np.pi], dtype=float)


def arm_base_fixed_transform() -> np.ndarray:
    from scipy.spatial.transform import Rotation as _R

    T = np.eye(4, dtype=float)
    T[:3, :3] = _R.from_euler("xyz", _ARM_BASE_FIXED_RPY).as_matrix()
    return T


@dataclass
class ContinuityParams:
    # Local theta0 tracking window around previous selected theta0.
    local_theta0_window: float = 0.15
    local_theta0_count: int = 5
    # Candidate score: w_vel*|dq| + w_acc*|ddq| + w_pose*pose_err + w_theta0*|dtheta0|
    w_vel: float = 1.0
    w_acc: float = 0.25
    w_pose: float = 0.1
    w_theta0: float = 0.15
    # Human arm-angle prior (geometric solver only): when solve() receives a
    # psi_ref (e.g. from Quest shoulder/elbow tracking), the local psi window
    # is centered on it and the score gains w_psi_ref*|theta0 - psi_ref|.
    # Must dominate w_vel/w_theta0 so the elbow plane follows the human arm.
    w_psi_ref: float = 2.0
    # Hysteresis: keep locked branch unless significantly better candidate appears.
    hysteresis_margin: float = 0.03
    # Fallback to global scan when local window fails.
    enable_global_fallback: bool = True
    # Escape hysteresis (geometric solver): the local window can flicker empty
    # at the wrist-limit boundary during big end-effector rolls, making the
    # arm angle jump to a far feasible psi and snap back frame-to-frame
    # (visible shoulder twitch). Require the local windows to be empty for
    # this many consecutive frames before escaping to the global scan, and
    # (once escaped) the home window to stay feasible for this many frames
    # before returning. 1/1 reproduces the old immediate behavior.
    escape_after_frames: int = 4
    return_after_frames: int = 20
    # 1D QP weights
    w_qp_joint_inc: float = 1.0
    w_qp_pose_err: float = 0.5


@dataclass
class ContinuityRuntimeState:
    q_prev: np.ndarray
    q_prev2: Optional[np.ndarray] = None
    theta0_prev: Optional[float] = None
    q_lock: Optional[np.ndarray] = None
    # Escape-hysteresis state (geometric solver): whether the arm angle is
    # currently parked in a globally-escaped region, the "home" psi to probe
    # for return, and the debounce streaks.
    esc_active: bool = False
    esc_home_psi: Optional[float] = None
    local_fail_streak: int = 0
    home_ok_streak: int = 0


# ==============================================================================
# Forward kinematics
# ==============================================================================

def fk_all(q: np.ndarray, p: AstralParams) -> List[np.ndarray]:
    Ts = [np.eye(4)]
    T = np.eye(4)
    for i in range(7):
        theta = q[i] + p.theta_offset[i]
        T = T @ _dh_A(theta, p.d_i[i], p.a_prev[i], p.alpha_prev[i])
        Ts.append(T.copy())
    return Ts


def fk(q: np.ndarray, p: Optional[AstralParams] = None) -> np.ndarray:
    if p is None:
        p = AstralParams.default()
    T = fk_all(q, p)[-1]
    T_post = np.eye(4)
    T_post[2, 3] = p.post_transform_d8
    return T @ T_post


# ==============================================================================
# Analytic extraction of joint angles from rotation sub-matrices
# ==============================================================================

def _within_limits(q: np.ndarray, limits: np.ndarray) -> bool:
    return bool(np.all(q >= limits[:, 0] - 1e-8) and np.all(q <= limits[:, 1] + 1e-8))


def _extract_123_from_R03_paper(R03: np.ndarray) -> List[np.ndarray]:
    sols = []
    c2 = float(np.clip(R03[2, 2], -1.0, 1.0))
    for sgn in (1.0, -1.0):
        s2 = sgn * math.sqrt(max(0.0, 1.0 - c2 * c2))
        if abs(s2) < 1e-8:
            continue
        th2 = math.atan2(s2, c2)
        th1 = math.atan2(float(-R03[1, 2] / s2), float(-R03[0, 2] / s2))
        th3 = math.atan2(float(-R03[2, 1] / s2), float(R03[2, 0] / s2))
        sols.append(np.array([th1, th2, th3], dtype=float))
    return sols


def _extract_567_from_T47_paper(T47: np.ndarray) -> List[np.ndarray]:
    sols = []
    c6 = float(np.clip(T47[1, 2], -1.0, 1.0))
    for sgn in (1.0, -1.0):
        s6 = sgn * math.sqrt(max(0.0, 1.0 - c6 * c6))
        if abs(s6) < 1e-8:
            continue
        th6 = math.atan2(s6, c6)
        th5 = math.atan2(float(T47[2, 2] / s6), float(T47[0, 2] / s6))
        th7 = math.atan2(float(T47[1, 1] / s6), float(-T47[1, 0] / s6))
        sols.append(np.array([th5, th6, th7], dtype=float))
    return sols


# ==============================================================================
# S-W-E geometric primitives
# ==============================================================================

def _solve_q123_from_swe(
    E: np.ndarray, W: np.ndarray, q4: float, p: AstralParams
) -> List[np.ndarray]:
    """Solve shoulder joints from elbow E, wrist W and elbow angle q4 (raw).

    Returns FULL MDH angles (theta = q + theta_offset); the caller subtracts
    theta_offset uniformly. Derived for positive MDH (alpha = +90 deg) with
    shoulder point S = [0, 0, d0]:  cos(th2) = (d0 - Ez) / d2.
    """
    d0 = float(p.d_i[0])
    d2 = float(p.d_i[2])
    d4 = float(p.d_i[4])
    if abs(d2) < 1e-12 or abs(d4) < 1e-12:
        return []

    th4 = q4 + p.theta_offset[3]  # full elbow angle

    Ex, Ey, Ez = float(E[0]), float(E[1]), float(E[2])
    rho = math.hypot(Ex, Ey)
    c2 = (d0 - Ez) / d2
    if c2 < -1.0 - 1e-8 or c2 > 1.0 + 1e-8:
        return []
    c2 = float(np.clip(c2, -1.0, 1.0))
    if rho > abs(d2) + 1e-7:
        return []

    v = W - E
    n_v = float(np.linalg.norm(v))
    if n_v < 1e-10:
        return []
    u = v / n_v  # unit forearm direction (W - E)

    s4 = math.sin(th4)
    if abs(s4) < 1e-8:
        return []

    sols: List[np.ndarray] = []
    for s2 in (math.sqrt(max(0.0, 1.0 - c2 * c2)), -math.sqrt(max(0.0, 1.0 - c2 * c2))):
        if abs(s2) < 1e-10:
            continue
        # Ex = cos(th1) sin(th2) d2,  Ey = sin(th1) sin(th2) d2
        c1 = Ex / (d2 * s2)
        s1 = Ey / (d2 * s2)
        n1 = math.hypot(c1, s1)
        if n1 < 1e-12:
            continue
        c1 /= n1
        s1 /= n1
        th1 = math.atan2(s1, c1)
        th2 = math.atan2(s2, c2)

        # forearm direction: u = R_sh @ [cos(th3) sin(th4), sin(th3) sin(th4), -cos(th4)]
        R_sh = _rotz(th1) @ _rotx(0.5 * math.pi) @ _rotz(th2) @ _rotx(0.5 * math.pi)
        w = R_sh.T @ u
        th3 = math.atan2(w[1] / s4, w[0] / s4)
        sols.append(np.array([th1, th2, th3], dtype=float))
    return sols


def _solve_theta4_from_triangle(
    S: np.ndarray, W: np.ndarray, p: AstralParams
) -> Optional[float]:
    l_sw = float(np.linalg.norm(W - S))
    l_se = abs(float(p.d_i[2]))
    l_ew = abs(float(p.d_i[4]))
    if l_sw < 1e-10:
        return None
    c4 = (l_sw * l_sw - l_se * l_se - l_ew * l_ew) / (2.0 * l_se * l_ew)
    if c4 < -1.0 or c4 > 1.0:
        return None
    c4 = float(np.clip(c4, -1.0, 1.0))
    # Elbow bend phi = acos(c4). The full elbow angle theta4 is:
    #   left arm  (theta_offset[3] = pi): theta4 = pi - phi  -> raw q4 = -phi
    #   right arm (theta_offset[3] = 0) : theta4 = phi      -> raw q4 = +phi
    # The mirror is encoded by the sign of the shoulder-base offset d_i[0]
    # (negative left, positive right).
    if p.d_i[0] > 0.0:  # mirrored right arm
        return math.acos(c4)
    return -math.acos(c4)


def _compute_swe_from_target(
    T07: np.ndarray, p: AstralParams
) -> Tuple[np.ndarray, np.ndarray, Optional[float], np.ndarray]:
    R = T07[:3, :3]
    p_target = T07[:3, 3]
    z7 = R[:, 2]
    d6 = float(p.d_i[6])
    d1 = float(p.d_i[0])

    O7 = p_target - p.post_transform_d8 * z7
    W = O7 - d6 * z7
    S = np.array([0.0, 0.0, d1], dtype=float)

    q4_abs = _solve_theta4_from_triangle(S, W, p)
    v_sw = W - S
    n_sw = float(np.linalg.norm(v_sw))
    if n_sw < 1e-12:
        u_sw = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        u_sw = v_sw / n_sw
    return S, W, q4_abs, u_sw


def _elbow_from_arm_angle(
    S: np.ndarray, W: np.ndarray, theta0: float, p: AstralParams
) -> Optional[np.ndarray]:
    l_se = abs(float(p.d_i[2]))
    l_ew = abs(float(p.d_i[4]))
    sw = W - S
    l_sw = float(np.linalg.norm(sw))
    if l_sw < 1e-12:
        return None
    u_sw = sw / l_sw

    x = (l_se * l_se - l_ew * l_ew + l_sw * l_sw) / (2.0 * l_sw)
    r2 = l_se * l_se - x * x
    if r2 < -1e-10:
        return None
    r = math.sqrt(max(0.0, r2))
    C = S + x * u_sw

    os_vec = S.copy()
    t = np.cross(os_vec, u_sw)
    if np.linalg.norm(t) < 1e-10:
        t = np.cross(np.array([1.0, 0.0, 0.0], dtype=float), u_sw)
    if np.linalg.norm(t) < 1e-10:
        t = np.cross(np.array([0.0, 1.0, 0.0], dtype=float), u_sw)
    e1 = t / max(np.linalg.norm(t), 1e-12)
    e2 = np.cross(u_sw, e1)
    e2 = e2 / max(np.linalg.norm(e2), 1e-12)
    E = C + r * (math.cos(theta0) * e1 + math.sin(theta0) * e2)
    return E


# ==============================================================================
# Joint limit weighting and arm-angle feasibility
# ==============================================================================

def _weight_limits(q: float, q_min: float, q_max: float) -> float:
    span = q_max - q_min
    if span < 1e-12:
        return 1.0
    x = 2.0 * (q - (q_min + q_max) * 0.5) / span
    a = 2.38
    b = 2.28
    if x >= 0:
        if x >= 1.0:
            return 1e6
        den = math.exp(a * (1.0 - x)) - 1.0
        if abs(den) < 1e-12:
            return 1e6
        return b * x / den
    if x <= -1.0:
        return 1e6
    den = math.exp(a * (1.0 + x)) - 1.0
    if abs(den) < 1e-12:
        return 1e6
    return -b * x / den


def _get_theta0_feasible_region(
    T07: np.ndarray, p: AstralParams, step: float = 0.01
) -> List[float]:
    feasible: List[float] = []
    for theta0 in np.arange(-math.pi, math.pi, step):
        t = float(theta0)
        if _ik_one_arm_angle(T07, t, p):
            feasible.append(t)
    return feasible


def _optimal_theta0(
    feasible_theta0: List[float], T07: np.ndarray, p: AstralParams, q_prev: np.ndarray
) -> float:
    best_cost = float("inf")
    best_t = feasible_theta0[0]
    for t in feasible_theta0:
        sols = _ik_one_arm_angle(T07, float(t), p)
        for q_full in sols:
            q = q_full[:7]
            cost = 0.0
            for i in range(7):
                lo, hi = float(p.joint_limits[i, 0]), float(p.joint_limits[i, 1])
                w = _weight_limits(q[i], lo, hi)
                dq = abs(q[i] - q_prev[i])
                cost += w * dq * dq
            if cost < best_cost:
                best_cost = cost
                best_t = float(t)
    return best_t


def _scan_theta0_solutions(
    T07: np.ndarray, p: AstralParams, step: float
) -> Tuple[List[float], List[Tuple[float, List[np.ndarray]]]]:
    feasible_theta0: List[float] = []
    cached: List[Tuple[float, List[np.ndarray]]] = []
    for theta0 in np.arange(-math.pi, math.pi, step):
        t = float(theta0)
        sols = _ik_one_arm_angle(T07, t, p)
        if sols:
            feasible_theta0.append(t)
            cached.append((t, sols))
    return feasible_theta0, cached


def _best_weighted_from_cached(
    cached: List[Tuple[float, List[np.ndarray]]], p: AstralParams, q_prev: np.ndarray
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[float]]:
    best_cost = float("inf")
    q_best: Optional[np.ndarray] = None
    q_best_full: Optional[np.ndarray] = None
    theta0_best: Optional[float] = None
    for t, sols in cached:
        for q_full in sols:
            q = q_full[:7]
            cost = 0.0
            for i in range(7):
                lo, hi = float(p.joint_limits[i, 0]), float(p.joint_limits[i, 1])
                w = _weight_limits(q[i], lo, hi)
                dq = abs(q[i] - q_prev[i])
                cost += w * dq * dq
            if cost < best_cost:
                best_cost = cost
                q_best = q.copy()
                q_best_full = q_full
                theta0_best = t
    return q_best, q_best_full, theta0_best


def _best_q_at_theta0_weighted(
    theta0: float, T07: np.ndarray, p: AstralParams, q_prev: np.ndarray
) -> Optional[np.ndarray]:
    sols = _ik_one_arm_angle(T07, float(theta0), p)
    best_cost = float("inf")
    best_q: Optional[np.ndarray] = None
    for q_full in sols:
        q = q_full[:7]
        cost = 0.0
        for i in range(7):
            lo, hi = float(p.joint_limits[i, 0]), float(p.joint_limits[i, 1])
            w = _weight_limits(q[i], lo, hi)
            dq = abs(q[i] - q_prev[i])
            cost += w * dq * dq
        if cost < best_cost:
            best_cost = cost
            best_q = q.copy()
    return best_q


# ==============================================================================
# 1D QP post-optimization
# ==============================================================================

def _qp_1d_objective(
    dq: float,
    idx: int,
    q_base: np.ndarray,
    T_target: np.ndarray,
    p: AstralParams,
    continuity: ContinuityParams,
) -> float:
    q_new = q_base.copy()
    q_new[idx] = wrap_to_pi(q_new[idx] + dq)

    q_min, q_max = p.joint_limits[idx]
    if q_new[idx] < q_min or q_new[idx] > q_max:
        limit_penalty = 1e6 * abs(q_new[idx] - np.clip(q_new[idx], q_min, q_max))
    else:
        limit_penalty = 0.0

    inc_cost = continuity.w_qp_joint_inc * (dq ** 2)

    T_cur = fk(q_new, p)
    pose_err = np.linalg.norm(pose_error(T_cur, T_target))
    pose_cost = continuity.w_qp_pose_err * (pose_err ** 2)

    return inc_cost + pose_cost + limit_penalty


def _optimize_q_with_1d_qp(
    q_init: np.ndarray,
    T_target: np.ndarray,
    p: AstralParams,
    continuity: ContinuityParams,
) -> np.ndarray:
    q_opt = q_init.copy()
    for idx in range(7):
        q_min, q_max = p.joint_limits[idx]
        delta_max = min(0.05, q_max - q_opt[idx])
        delta_min = max(-0.05, q_min - q_opt[idx])

        res = minimize_scalar(
            _qp_1d_objective,
            bounds=(delta_min, delta_max),
            args=(idx, q_opt, T_target, p, continuity),
            method="bounded",
            options={"xatol": 0.005},
        )

        if res.success:
            q_opt[idx] = wrap_to_pi(q_opt[idx] + res.x)

    return q_opt


# ==============================================================================
# Core IK: single arm-angle solve and continuous multi-branch solver
# ==============================================================================

def _ik_one_arm_angle(
    T07: np.ndarray, theta0: float, p: AstralParams
) -> List[np.ndarray]:
    S, W_from_pose, q4_abs, _ = _compute_swe_from_target(T07, p)
    E_point = _elbow_from_arm_angle(S, W_from_pose, theta0, p)
    T_chain = _remove_post_transform(T07, p)
    return _ik_one_arm_angle_cached(S, W_from_pose, q4_abs, T_chain, theta0, E_point, p)


def _ik_one_arm_angle_cached(
    S: np.ndarray,
    W: np.ndarray,
    q4_abs: Optional[float],
    T_chain: np.ndarray,
    theta0: float,
    E_point: Optional[np.ndarray],
    p: AstralParams,
) -> List[np.ndarray]:
    """Optimized variant: S, W, q4_abs, T_chain precomputed once outside theta0 loop."""
    if q4_abs is None:
        return []

    q_solutions = []

    for q4 in (q4_abs, -q4_abs):
        if E_point is None:
            continue
        q123_sols = _solve_q123_from_swe(E_point, W, q4, p)
        for q123 in q123_sols:
            th1, th2, th3 = q123.tolist()  # already full MDH angles

            th4 = q4 + p.theta_offset[3]

            T04 = np.eye(4)
            th_raw = [th1, th2, th3, th4]
            for i in range(4):
                T04 = T04 @ _dh_A(th_raw[i], p.d_i[i], p.a_prev[i], p.alpha_prev[i])
            T47 = _invert_rigid_transform(T04) @ T_chain

            for th567 in _extract_567_from_T47_paper(T47):
                th5, th6, th7 = th567.tolist()
                theta_raw = np.array([th1, th2, th3, th4, th5, th6, th7], dtype=float)
                q = wrap_to_pi(theta_raw - p.theta_offset)
                if _within_limits(q, p.joint_limits):
                    extra = np.array(
                        [
                            theta0,
                            S[0], S[1], S[2],
                            W[0], W[1], W[2],
                            *(E_point.tolist() if E_point is not None else [np.nan, np.nan, np.nan]),
                        ],
                        dtype=float,
                    )
                    q = np.concatenate([q, extra])
                    q_solutions.append(q)
    return q_solutions


def _collect_unique_solutions_for_theta0_grid(
    T: np.ndarray, p: AstralParams, theta0_grid: np.ndarray
) -> List[np.ndarray]:
    # Precompute SWE primitives once (independent of theta0)
    S, W, q4_abs, _ = _compute_swe_from_target(T, p)
    T_chain = _remove_post_transform(T, p)

    all_solutions: List[np.ndarray] = []
    for theta0 in theta0_grid:
        t = float(wrap_to_pi(np.array([theta0]))[0])
        E_point = _elbow_from_arm_angle(S, W, t, p)
        all_solutions.extend(
            _ik_one_arm_angle_cached(S, W, q4_abs, T_chain, t, E_point, p)
        )
    unique_solutions: List[np.ndarray] = []
    for q in all_solutions:
        if not any(
            np.linalg.norm(wrap_to_pi(q[:7] - u[:7])) < 1e-4 for u in unique_solutions
        ):
            unique_solutions.append(q)
    return unique_solutions


def ik_arm_angle(
    T07: np.ndarray,
    p: Optional[AstralParams] = None,
    q_prev: Optional[np.ndarray] = None,
    n_psi: int = 181,
    skip_qp: bool = False,
) -> Tuple[Optional[np.ndarray], List[float]]:
    if p is None:
        p = AstralParams.default()
    T = np.array(T07, dtype=float)
    _, _, q4_abs, _ = _compute_swe_from_target(T, p)
    if q4_abs is None:
        return None, []
    if q_prev is None:
        q_prev = np.zeros(7, dtype=float)
    step = min(0.03, 2.0 * math.pi / max(1, n_psi))
    feasible_theta0 = _get_theta0_feasible_region(T, p, step=step)
    if len(feasible_theta0) == 0:
        return None, feasible_theta0
    best_t0 = _optimal_theta0(feasible_theta0, T, p, q_prev)
    q_best = _best_q_at_theta0_weighted(best_t0, T, p, q_prev)

    if q_best is not None and not skip_qp:
        continuity = ContinuityParams()
        q_best = _optimize_q_with_1d_qp(q_best, T, p, continuity)

    return q_best, feasible_theta0


def ik_arm_angle_with_report(
    T_target: np.ndarray,
    p: Optional[AstralParams] = None,
    q_prev: Optional[np.ndarray] = None,
    n_psi: int = 181,
    skip_qp: bool = False,
) -> Tuple[Optional[np.ndarray], List[np.ndarray], Dict[str, object]]:
    if p is None:
        p = AstralParams.default()
    if q_prev is None:
        q_prev = np.zeros(7, dtype=float)

    T = np.array(T_target, dtype=float)
    S_dbg, W_dbg, q4_dbg, _ = _compute_swe_from_target(T, p)
    step = min(0.03, 2.0 * math.pi / max(1, n_psi))
    feasible_theta0, cached = _scan_theta0_solutions(T, p, step=step)

    if len(feasible_theta0) == 0:
        return (
            None, [],
            {
                "method": "failed", "candidate_count": 0,
                "pose_err_best": None,
                "swe_debug": {
                    "S": S_dbg.tolist(), "W": W_dbg.tolist(),
                    "E": [float("nan"), float("nan"), float("nan")],
                    "q4_abs": None if q4_dbg is None else float(q4_dbg),
                },
            },
        )

    q_best, q_best_full, best_t0 = _best_weighted_from_cached(cached, p, q_prev)

    if q_best is not None and not skip_qp:
        continuity = ContinuityParams()
        q_best = _optimize_q_with_1d_qp(q_best, T, p, continuity)

    if q_best is None:
        return (
            None, [],
            {
                "method": "failed", "candidate_count": 0,
                "pose_err_best": None,
                "swe_debug": {
                    "S": S_dbg.tolist(), "W": W_dbg.tolist(),
                    "E": [float("nan"), float("nan"), float("nan")],
                    "q4_abs": None if q4_dbg is None else float(q4_dbg),
                },
            },
        )

    if q_best_full is None or best_t0 is None:
        q_best_full = np.concatenate([
            q_best,
            np.array([0.0, S_dbg[0], S_dbg[1], S_dbg[2],
                      W_dbg[0], W_dbg[1], W_dbg[2],
                      float("nan"), float("nan"), float("nan")], dtype=float),
        ])
    theta0_best = float(q_best_full[7] if best_t0 is None else best_t0)
    swe_debug = {
        "S": q_best_full[8:11].tolist(),
        "W": q_best_full[11:14].tolist(),
        "E": q_best_full[14:17].tolist(),
        "q4_abs": None if q4_dbg is None else float(q4_dbg),
    }
    e_best = float(np.linalg.norm(pose_error(fk(q_best, p), T)))
    all_solutions = [q_best_full]
    return (
        q_best, all_solutions,
        {
            "method": "feasible_region+1DQP",
            "candidate_count": len(feasible_theta0),
            "pose_err_best": e_best,
            "theta0_selected": theta0_best,
            "feasible_theta0_count": len(feasible_theta0),
            "swe_debug": swe_debug,
        },
    )


def solve_pose_continuous_with_state(
    T_target: np.ndarray,
    state: ContinuityRuntimeState,
    p: Optional[AstralParams] = None,
    n_psi: int = 181,
    continuity: Optional[ContinuityParams] = None,
    skip_qp: bool = False,
) -> Tuple[Optional[np.ndarray], Dict[str, object], ContinuityRuntimeState]:
    if p is None:
        p = AstralParams.default()
    if continuity is None:
        continuity = ContinuityParams()

    T = np.array(T_target, dtype=float)
    q_prev = state.q_prev
    q_prev2 = state.q_prev2
    theta0_prev = state.theta0_prev
    q_lock = state.q_lock

    all_solutions: List[np.ndarray] = []
    method = "continuous_local_theta0"
    if theta0_prev is not None:
        theta0_grid = np.linspace(
            theta0_prev - continuity.local_theta0_window,
            theta0_prev + continuity.local_theta0_window,
            max(5, continuity.local_theta0_count),
            endpoint=True,
        )
        all_solutions = _collect_unique_solutions_for_theta0_grid(T, p, theta0_grid)

    if not all_solutions and continuity.enable_global_fallback:
        q_global, all_full, _ = ik_arm_angle_with_report(
            T, p=p, q_prev=q_prev, n_psi=n_psi, skip_qp=skip_qp
        )
        all_solutions = all_full
        method = "continuous_global_fallback"
        if q_global is None:
            return (
                None,
                {"method": method, "candidate_count": 0,
                 "selected_by": "failed", "pose_err_best": None},
                state,
            )
    elif not all_solutions:
        return (
            None,
            {"method": method, "candidate_count": 0,
             "selected_by": "failed", "pose_err_best": None},
            state,
        )

    scored = []
    for cand in all_solutions:
        q = cand[:7]
        theta0 = float(cand[7]) if cand.shape[0] > 7 else 0.0
        dq = wrap_to_pi(q - q_prev)
        vel_cost = float(np.linalg.norm(dq))
        if q_prev2 is not None:
            ddq = wrap_to_pi(q - 2.0 * q_prev + q_prev2)
            acc_cost = float(np.linalg.norm(ddq))
        else:
            acc_cost = 0.0
        pose_cost = float(np.linalg.norm(pose_error(fk(q, p), T)))
        if theta0_prev is None:
            theta0_cost = 0.0
        else:
            theta0_cost = abs(float(wrap_to_pi(np.array([theta0 - theta0_prev]))[0]))
        score = (
            continuity.w_vel * vel_cost
            + continuity.w_acc * acc_cost
            + continuity.w_pose * pose_cost
            + continuity.w_theta0 * theta0_cost
        )
        scored.append((score, vel_cost, acc_cost, pose_cost, theta0_cost, cand))

    scored.sort(key=lambda x: x[0])
    best = scored[0]

    selected = best
    selected_by = "best_score"
    if q_lock is not None:
        lock_item = min(
            scored, key=lambda x: float(np.linalg.norm(wrap_to_pi(x[5][:7] - q_lock)))
        )
        if lock_item[0] <= best[0] + continuity.hysteresis_margin:
            selected = lock_item
            selected_by = "hysteresis_locked"

    q_best_full = selected[5]
    q_best = q_best_full[:7]
    theta0_best = float(q_best_full[7]) if q_best_full.shape[0] > 7 else None

    if not skip_qp:
        q_best = _optimize_q_with_1d_qp(q_best, T, p, continuity)

    pose_best = float(np.linalg.norm(pose_error(fk(q_best, p), T)))

    next_state = ContinuityRuntimeState(
        q_prev=q_best,
        q_prev2=q_prev.copy(),
        theta0_prev=theta0_best if theta0_best is not None else theta0_prev,
        q_lock=q_best,
    )
    report = {
        "method": f"{method}+1DQP",
        "candidate_count": len(all_solutions),
        "selected_by": selected_by,
        "score_best": float(selected[0]),
        "vel_cost_best": float(selected[1]),
        "acc_cost_best": float(selected[2]),
        "pose_cost_best": float(selected[3]),
        "theta0_cost_best": float(selected[4]),
        "pose_err_best": pose_best,
        "theta0_selected": theta0_best,
    }
    return q_best, report, next_state


def solve_trajectory(
    T_targets: List[np.ndarray],
    p: Optional[AstralParams] = None,
    q_init: Optional[np.ndarray] = None,
    n_psi: int = 181,
) -> Tuple[List[Optional[np.ndarray]], List[Dict[str, object]]]:
    if p is None:
        p = AstralParams.default()
    if q_init is None:
        q_prev = np.zeros(7, dtype=float)
    else:
        q_prev = np.array(q_init, dtype=float).reshape(7)

    q_list: List[Optional[np.ndarray]] = []
    reports: List[Dict[str, object]] = []
    for T in T_targets:
        q, _, report = ik_arm_angle_with_report(
            np.array(T, dtype=float), p=p, q_prev=q_prev, n_psi=n_psi
        )
        q_list.append(q)
        reports.append(report)
        if q is not None:
            q_prev = q
    return q_list, reports


def solve_trajectory_continuous(
    T_targets: List[np.ndarray],
    p: Optional[AstralParams] = None,
    q_init: Optional[np.ndarray] = None,
    n_psi: int = 181,
    continuity: Optional[ContinuityParams] = None,
) -> Tuple[List[Optional[np.ndarray]], List[Dict[str, object]]]:
    if p is None:
        p = AstralParams.default()
    if continuity is None:
        continuity = ContinuityParams()
    if q_init is None:
        q_prev = np.zeros(7, dtype=float)
    else:
        q_prev = np.array(q_init, dtype=float).reshape(7)
    state = ContinuityRuntimeState(q_prev=q_prev)

    q_list: List[Optional[np.ndarray]] = []
    reports: List[Dict[str, object]] = []

    for T_raw in T_targets:
        q_best, report, state = solve_pose_continuous_with_state(
            np.array(T_raw, dtype=float),
            state=state, p=p, n_psi=n_psi, continuity=continuity,
        )
        q_list.append(q_best)
        reports.append(report)

    return q_list, reports


# ==============================================================================
# IKSolver class — public API matching what the teleop project expects
# ==============================================================================

class IKSolver:
    """Analytic DH IK for one Astral arm in ``left/right_base_link``.

    Nero arm-angle algorithm. Pose frame = arm base (not torso body_link).
    ``fk``/``solve`` include the fixed joint1 origin rpy from the URDF.
    """

    def __init__(
        self,
        arm_params: Optional[AstralParams] = None,
        continuity_params: Optional[ContinuityParams] = None,
        fast_mode: bool = True,
        apply_base_fixed: bool = False,
    ):
        self.params = arm_params if arm_params is not None else AstralParams.default()
        self.continuity = continuity_params if continuity_params is not None else ContinuityParams()
        self.fast_mode = fast_mode
        # apply_base_fixed is a legacy hack for the old (pre-flip) URDF; the clean
        # params already express FK in *_base_link, so T_fixed = identity.
        self._T_fixed = arm_base_fixed_transform() if apply_base_fixed else np.eye(4)
        self._T_fixed_inv = np.eye(4, dtype=float)
        R = self._T_fixed[:3, :3]
        self._T_fixed_inv[:3, :3] = R.T
        self._state = ContinuityRuntimeState(
            q_prev=np.zeros(7, dtype=float),
            q_prev2=None,
            theta0_prev=None,
            q_lock=None,
        )

    # ---- Properties ----

    @property
    def method_name(self) -> str:
        return "analytic_dh_arm_angle"

    @property
    def nq(self) -> int:
        return 7

    @property
    def lower_limits(self) -> np.ndarray:
        return self.params.joint_limits[:, 0].copy()

    @property
    def upper_limits(self) -> np.ndarray:
        return self.params.joint_limits[:, 1].copy()

    # ---- Core methods ----

    def solve(self, T_target: np.ndarray) -> Optional[np.ndarray]:
        """Solve IK for flange pose in ``*_base_link`` (same frame as :meth:`fk`)."""
        T_arm = np.array(T_target, dtype=float)
        T_mdh = self._T_fixed_inv @ T_arm
        state_prev = self._state
        q_best, _report, new_state = solve_pose_continuous_with_state(
            T_mdh,
            state=self._state,
            p=self.params,
            n_psi=91,
            continuity=self.continuity,
            skip_qp=self.fast_mode,
        )
        if q_best is None:
            self._state = state_prev
            return None
        self._state = new_state
        return q_best.reshape(7).copy()

    def fk(self, q: np.ndarray) -> np.ndarray:
        """FK: joints → flange pose in ``left_base_link`` / ``right_base_link``."""
        return self._T_fixed @ fk(np.array(q, dtype=float).reshape(7), self.params)

    def sync_state(self, q: np.ndarray, reset_branch: bool = True):
        """Set warm-start joint state for the next solve() call.

        Called once after arm moves to init pose. For continuous teleop,
        solve() auto-updates internal state — no need to call this each frame.

        Args:
            q: 7-element joint angle array (rad).
            reset_branch: If True, reset theta0 tracking for fresh branch
                          selection. Set False to preserve branch continuity.
        """
        q_7 = np.array(q, dtype=float).reshape(7)
        self._state.q_prev2 = self._state.q_prev.copy()
        self._state.q_prev = q_7.copy()
        if reset_branch:
            self._state.theta0_prev = None
            self._state.q_lock = None

    def check_self_collision(self, q: np.ndarray) -> bool:
        """Self-collision check stub.

        The analytic DH solver has no collision geometry model.
        Returns False (no collision). Replace with capsule-based check later.
        """
        return False

    def active_joint_names(self) -> List[str]:
        return [f"Joint_{i}" for i in range(1, 8)]

    @staticmethod
    def find_agx_arm_package_dir(urdf_path: Optional[str] = None) -> str:
        """Compatibility stub — no URDF directory needed for analytic IK."""
        return "."


# Module-level alias for backward-compatible imports
find_agx_arm_package_dir = IKSolver.find_agx_arm_package_dir
