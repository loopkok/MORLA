#!/usr/bin/env python3
"""Astral arm geometric analytic IK — DH-free S/E/W arm-angle method.

All geometry (joint axes, shoulder/elbow/wrist centers, home flange pose,
joint limits) is extracted from the URDF at q=0 with Pinocchio and expressed
in the arm base frame (``left_base_link`` / ``right_base_link``). No DH table,
no theta offsets, no axis-flip convention: solutions come out directly in the
URDF/hardware joint sign convention (same frames as ``urdf_solver``).

Algorithm (SRS-type 7-DoF arm, POE + Paden-Kahan subproblems):

  1. wrist center ``W = p_t - R_t o`` (``o`` = home wrist->flange offset, exact)
  2. elbow ``q4`` from the S-W triangle: ``A cos q4 + B sin q4 = K`` (2 branches)
  3. elbow point ``E(psi)`` on the arm-angle circle around the S-W axis
  4. ``R03`` from two-vector alignment: ``R03 v_se = E - S`` and
     ``R03 R4 v_ew = W - E`` (angle between the pairs preserved by q4)
  5. decompose ``R03 = R1 R2 R3`` on the real home axes (PK2 + PK1, 2 branches)
  6. ``R47 = (R03 R4)^T R_t R_home^T`` -> decompose into q5..q7 (2 branches)
  7. joint-limit filter + continuity selection (local psi window, hysteresis,
     global fallback) — same scheme as the DH solver in ``analytic.py``.

FK is the POE product of exponentials ``T(q) = prod exp(xi_i q_i) T_home``,
exact against the URDF (no fitted parameters anywhere).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize_scalar

from astral_arm_teleop.ik.analytic import (
    ContinuityParams,
    ContinuityRuntimeState,
    pose_error,
    wrap_to_pi,
)

__all__ = [
    "ArmGeometry",
    "GeometricIKSolver",
    "extract_arm_geometry",
    "fk",
    "psi_from_elbow_dir",
    "solve_pose_continuous_with_state",
]


# ==============================================================================
# Small rotation helpers
# ==============================================================================

def _axis_angle_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation for a unit axis (axes from ArmGeometry are normalized)."""
    ux, uy, uz = float(axis[0]), float(axis[1]), float(axis[2])
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return np.array(
        [
            [t * ux * ux + c, t * ux * uy - s * uz, t * ux * uz + s * uy],
            [t * ux * uy + s * uz, t * uy * uy + c, t * uy * uz - s * ux],
            [t * ux * uz - s * uy, t * uy * uz + s * ux, t * uz * uz + c],
        ],
        dtype=float,
    )


def _angle_about_axis(R: np.ndarray, axis: np.ndarray) -> float:
    """Extract angle of a known-axis rotation: R = Rot(axis, theta)."""
    s = (
        (R[2, 1] - R[1, 2]) * axis[0]
        + (R[0, 2] - R[2, 0]) * axis[1]
        + (R[1, 0] - R[0, 1]) * axis[2]
    )
    return math.atan2(float(s), float(np.trace(R) - 1.0))


def _pk2(
    a1: np.ndarray, a2: np.ndarray, u: np.ndarray, v: np.ndarray
) -> List[Tuple[float, float]]:
    """Paden-Kahan subproblem 2 (rotation only): Rot(a1,t1) Rot(a2,t2) u = v.

    ``a1``/``a2`` are the (intersecting) home axes, ``u``/``v`` unit vectors.
    Returns up to two ``(t1, t2)`` branches; empty when v is inconsistent.
    Scalar math throughout — this is the hottest loop of the psi scan.
    """
    a1x, a1y, a1z = float(a1[0]), float(a1[1]), float(a1[2])
    a2x, a2y, a2z = float(a2[0]), float(a2[1]), float(a2[2])
    ux, uy, uz = float(u[0]), float(u[1]), float(u[2])
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])

    a1_a2 = a1x * a2x + a1y * a2y + a1z * a2z
    a2_u = a2x * ux + a2y * uy + a2z * uz
    a1_u = a1x * ux + a1y * uy + a1z * uz
    a1_v = a1x * vx + a1y * vy + a1z * vz
    # a2 x u
    cx = a2y * uz - a2z * uy
    cy = a2z * ux - a2x * uz
    cz = a2x * uy - a2y * ux

    A = a1_u - a1_a2 * a2_u
    B = a1x * cx + a1y * cy + a1z * cz
    C = a1_v - a1_a2 * a2_u
    h = math.hypot(A, B)
    if h < 1e-12:  # u parallel a2: t2 is a free spin, not a real branch
        return []
    r = C / h
    if r > 1.0 + 1e-9 or r < -1.0 - 1e-9:
        return []
    r = min(1.0, max(-1.0, r))
    base = math.atan2(B, A)
    d = math.acos(r)

    # v_perp = v - (a1.v) a1 (constant across branches)
    vpx = vx - a1_v * a1x
    vpy = vy - a1_v * a1y
    vpz = vz - a1_v * a1z

    out: List[Tuple[float, float]] = []
    for t2 in (base + d, base - d):
        c2 = math.cos(t2)
        s2 = math.sin(t2)
        k = a2_u * (1.0 - c2)
        # x = Rot(a2, t2) u = u c + (a2 x u) s + a2 (a2.u) (1-c)
        xx = ux * c2 + cx * s2 + a2x * k
        xy = uy * c2 + cy * s2 + a2y * k
        xz = uz * c2 + cz * s2 + a2z * k
        a1_x = a1x * xx + a1y * xy + a1z * xz
        xpx = xx - a1_x * a1x
        xpy = xy - a1_x * a1y
        xpz = xz - a1_x * a1z
        nx = math.sqrt(xpx * xpx + xpy * xpy + xpz * xpz)
        if nx < 1e-9:
            # x parallel a1: spherical singularity (e.g. q2=±90°), t1 free.
            continue
        # t1 = atan2(a1 . (xp x vp), xp . vp)
        scx = xpy * vpz - xpz * vpy
        scy = xpz * vpx - xpx * vpz
        scz = xpx * vpy - xpy * vpx
        t1 = math.atan2(
            a1x * scx + a1y * scy + a1z * scz, xpx * vpx + xpy * vpy + xpz * vpz
        )
        if not any(abs(t1 - s[0]) < 1e-9 and abs(t2 - s[1]) < 1e-9 for s in out):
            out.append((t1, t2))
    return out


def _decompose_spherical(
    R: np.ndarray, a1: np.ndarray, a2: np.ndarray, a3: np.ndarray
) -> List[np.ndarray]:
    """Decompose R = Rot(a1,q1) Rot(a2,q2) Rot(a3,q3) on concurrent home axes."""
    sols: List[np.ndarray] = []
    for q1, q2 in _pk2(a1, a2, a3, R @ a3):
        R12 = _axis_angle_rot(a1, q1) @ _axis_angle_rot(a2, q2)
        q3 = _angle_about_axis(R12.T @ R, a3)
        sols.append(np.array([q1, q2, q3], dtype=float))
    return sols


def _frame_from_two(a: np.ndarray, b: np.ndarray) -> Optional[np.ndarray]:
    """Right-handed orthonormal frame with e1=a and e2 in the a-b plane."""
    ax, ay, az = float(a[0]), float(a[1]), float(a[2])
    bx, by, bz = float(b[0]), float(b[1]), float(b[2])
    d = bx * ax + by * ay + bz * az
    e2x, e2y, e2z = bx - d * ax, by - d * ay, bz - d * az
    n = math.sqrt(e2x * e2x + e2y * e2y + e2z * e2z)
    if n < 1e-9:
        return None
    e2x, e2y, e2z = e2x / n, e2y / n, e2z / n
    # e3 = e1 x e2
    return np.array(
        [
            [ax, e2x, ay * e2z - az * e2y],
            [ay, e2y, az * e2x - ax * e2z],
            [az, e2z, ax * e2y - ay * e2x],
        ],
        dtype=float,
    )


def _rotation_two_pairs(
    u1_t: np.ndarray, u2_t: np.ndarray, u1_h: np.ndarray, u2_h: np.ndarray
) -> Optional[np.ndarray]:
    """Unique R mapping home unit-vector pair (u1_h, u2_h) to target pair."""
    Ft = _frame_from_two(u1_t, u2_t)
    Fh = _frame_from_two(u1_h, u2_h)
    if Ft is None or Fh is None:
        return None
    return Ft @ Fh.T


def _ls_axis_intersection(pts: np.ndarray, dirs: np.ndarray) -> Tuple[np.ndarray, float]:
    """Least-squares concurrency point of axis lines; returns (point, max dist)."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for p, d in zip(pts, dirs):
        M = np.eye(3) - np.outer(d, d)
        A += M
        b += M @ p
    x = np.linalg.solve(A, b)
    res = max(
        float(np.linalg.norm(np.cross(d, x - p))) for p, d in zip(pts, dirs)
    )
    return x, res


# ==============================================================================
# Geometry extraction from URDF (q=0, arm base frame)
# ==============================================================================

@dataclass
class ArmGeometry:
    """SRS arm geometry in the arm base frame, measured at q=0."""

    arm: str  # "L" | "R"
    axes: np.ndarray  # (7,3) home joint axes (unit)
    points: np.ndarray  # (7,3) home joint-frame origins (on each axis)
    S: np.ndarray  # shoulder center (concurrency of axes 1-3)
    E0: np.ndarray  # elbow center at home (joint4 origin)
    W0: np.ndarray  # wrist center at home (concurrency of axes 5-7)
    T_home: np.ndarray  # (4,4) flange pose at q=0
    flange_offset: np.ndarray  # R_home^T (F0 - W0): wrist->flange in flange frame
    v_se: np.ndarray  # E0 - S
    v_ew: np.ndarray  # W0 - E0
    v_se_hat: np.ndarray
    v_ew_hat: np.ndarray
    l_se: float
    l_ew: float
    q4_A: float  # v_se.Rot(a4,q4) v_ew = A cos q4 + B sin q4 + C
    q4_B: float
    q4_C: float
    psi_ref: np.ndarray  # reference vector defining arm angle psi = 0
    lower: np.ndarray  # (7,) joint limits
    upper: np.ndarray


def extract_arm_geometry(urdf_path: str, arm_side: str) -> ArmGeometry:
    """Measure S/E/W centers, home axes and home flange pose from the URDF."""
    import pinocchio as pin

    from astral_arm_teleop.ik.factory import default_astral_urdf_path
    from astral_arm_teleop.ik.urdf_solver import _package_dirs_for

    side = arm_side.strip().lower()
    is_left = side.startswith("l")
    prefix = "left" if is_left else "right"
    other = "right" if is_left else "left"

    urdf_path = str(urdf_path).strip() or default_astral_urdf_path()
    robot = pin.RobotWrapper.BuildFromURDF(
        urdf_path, package_dirs=_package_dirs_for(urdf_path)
    )
    if f"{prefix}_joint1" not in set(robot.model.names):
        raise ValueError(
            f"geometric IK expects the astral_robot URDF ({prefix}_joint*): {urdf_path}"
        )
    reduced = robot.buildReducedRobot(
        list_of_joints_to_lock=[f"{other}_joint{i}" for i in range(1, 8)],
        reference_configuration=np.zeros(robot.model.nq),
    )
    model = reduced.model
    data = model.createData()
    pin.forwardKinematics(model, data, np.zeros(model.nq))
    pin.updateFramePlacements(model, data)

    T_base_inv = data.oMf[model.getFrameId(f"{prefix}_base_link")].inverse()
    axes = np.zeros((7, 3))
    points = np.zeros((7, 3))
    for i in range(7):
        jid = model.getJointId(f"{prefix}_joint{i + 1}")
        if "RZ" not in model.joints[jid].shortname():
            raise ValueError(
                f"geometric IK expects revolute-Z joints, got "
                f"{model.joints[jid].shortname()} at {prefix}_joint{i + 1}"
            )
        p = T_base_inv * data.oMi[jid]
        points[i] = p.translation
        a = p.rotation @ np.array([0.0, 0.0, 1.0])
        axes[i] = a / np.linalg.norm(a)

    T_home = (T_base_inv * data.oMf[model.getFrameId(f"{prefix}_link7")]).homogeneous

    S, res_s = _ls_axis_intersection(points[0:3], axes[0:3])
    W0, res_w = _ls_axis_intersection(points[4:7], axes[4:7])
    if res_s > 2e-3 or res_w > 2e-3:
        raise ValueError(
            f"{prefix} arm is not SRS: shoulder concurrency residual "
            f"{res_s * 1e3:.2f} mm, wrist {res_w * 1e3:.2f} mm — "
            "geometric arm-angle IK not applicable"
        )
    E0 = points[3].copy()

    v_se = E0 - S
    v_ew = W0 - E0
    l_se = float(np.linalg.norm(v_se))
    l_ew = float(np.linalg.norm(v_ew))
    if l_se < 1e-6 or l_ew < 1e-6:
        raise ValueError(f"{prefix} arm degenerate link lengths: {l_se}, {l_ew}")

    a4 = axes[3]
    q4_A = float(v_se @ v_ew - (v_se @ a4) * (a4 @ v_ew))
    q4_B = float(v_se @ np.cross(a4, v_ew))
    q4_C = float((v_se @ a4) * (a4 @ v_ew))
    if math.hypot(q4_A, q4_B) < 1e-9:
        raise ValueError(f"{prefix} arm degenerate elbow axis (a4 parallel links)")

    return ArmGeometry(
        arm="L" if is_left else "R",
        axes=axes,
        points=points,
        S=S,
        E0=E0,
        W0=W0,
        T_home=T_home,
        flange_offset=T_home[:3, :3].T @ (T_home[:3, 3] - W0),
        v_se=v_se,
        v_ew=v_ew,
        v_se_hat=v_se / l_se,
        v_ew_hat=v_ew / l_ew,
        l_se=l_se,
        l_ew=l_ew,
        q4_A=q4_A,
        q4_B=q4_B,
        q4_C=q4_C,
        # Same psi=0 convention as the DH solver: reference vector = shoulder
        # point seen from the base origin.
        psi_ref=S.copy(),
        lower=np.asarray(model.lowerPositionLimit, dtype=float).copy(),
        upper=np.asarray(model.upperPositionLimit, dtype=float).copy(),
    )


# ==============================================================================
# Forward kinematics (POE)
# ==============================================================================

def fk(q: np.ndarray, g: ArmGeometry) -> np.ndarray:
    """POE FK: T(q) = prod_i exp(xi_i q_i) T_home, in the arm base frame."""
    q = np.asarray(q, dtype=float).reshape(7)
    T = np.eye(4)
    I3 = np.eye(3)
    for i in range(7):
        R = _axis_angle_rot(g.axes[i], float(q[i]))
        Ti = np.eye(4)
        Ti[:3, :3] = R
        Ti[:3, 3] = (I3 - R) @ g.points[i]
        T = T @ Ti
    return T @ g.T_home


# ==============================================================================
# Geometric IK primitives
# ==============================================================================

def _solve_q4(l_sw: float, g: ArmGeometry) -> List[float]:
    """Elbow angles from the S-W triangle (both branches, unsorted)."""
    h = math.hypot(g.q4_A, g.q4_B)
    K = 0.5 * (l_sw * l_sw - g.l_se * g.l_se - g.l_ew * g.l_ew)
    r = (K - g.q4_C) / h
    if r > 1.0 + 1e-9 or r < -1.0 - 1e-9:
        return []
    r = min(1.0, max(-1.0, r))
    base = math.atan2(g.q4_B, g.q4_A)
    d = math.acos(r)
    if d < 1e-12:
        return [base]
    return [base + d, base - d]


def _circle_basis_sw(
    S: np.ndarray, W: np.ndarray, g: ArmGeometry
) -> Optional[Tuple[float, ...]]:
    """Elbow-circle frame: (C, u, e1, e2, r) as flat scalars, None if degenerate.

    ``u`` = unit S->W axis, circle center ``C = S + x u``, radius ``r``, and
    (e1, e2) the in-plane basis with e1 from ``g.psi_ref`` (psi = 0 points the
    elbow along e1). Scalar math — hot path of the psi scan.
    """
    swx, swy, swz = float(W[0] - S[0]), float(W[1] - S[1]), float(W[2] - S[2])
    l_sw = math.sqrt(swx * swx + swy * swy + swz * swz)
    if l_sw < 1e-12:
        return None
    ux, uy, uz = swx / l_sw, swy / l_sw, swz / l_sw
    x = (g.l_se * g.l_se - g.l_ew * g.l_ew + l_sw * l_sw) / (2.0 * l_sw)
    r2 = g.l_se * g.l_se - x * x
    if r2 < -1e-10:
        return None
    r = math.sqrt(max(0.0, r2))
    Cx, Cy, Cz = S[0] + x * ux, S[1] + x * uy, S[2] + x * uz
    rx, ry, rz = float(g.psi_ref[0]), float(g.psi_ref[1]), float(g.psi_ref[2])
    # t = psi_ref x u
    tx, ty, tz = ry * uz - rz * uy, rz * ux - rx * uz, rx * uy - ry * ux
    n = math.sqrt(tx * tx + ty * ty + tz * tz)
    if n < 1e-10:  # u parallel x
        tx, ty, tz = 0.0, -uz, uy
        n = math.sqrt(tx * tx + ty * ty + tz * tz)
    if n < 1e-10:  # u parallel y
        tx, ty, tz = uz, 0.0, -ux
        n = math.sqrt(tx * tx + ty * ty + tz * tz)
    e1x, e1y, e1z = tx / n, ty / n, tz / n
    # e2 = u x e1
    e2x, e2y, e2z = uy * e1z - uz * e1y, uz * e1x - ux * e1z, ux * e1y - uy * e1x
    return (Cx, Cy, Cz, ux, uy, uz, e1x, e1y, e1z, e2x, e2y, e2z, r)


def _elbow_point(
    psi: float, S: np.ndarray, W: np.ndarray, g: ArmGeometry
) -> Optional[np.ndarray]:
    """Elbow center on the arm-angle circle around the S-W axis."""
    basis = _circle_basis_sw(S, W, g)
    if basis is None:
        return None
    Cx, Cy, Cz, _, _, _, e1x, e1y, e1z, e2x, e2y, e2z, r = basis
    cp, sp = r * math.cos(psi), r * math.sin(psi)
    return np.array(
        [Cx + cp * e1x + sp * e2x, Cy + cp * e1y + sp * e2y, Cz + cp * e1z + sp * e2z],
        dtype=float,
    )


def psi_from_elbow_dir(
    S: np.ndarray,
    W: np.ndarray,
    elbow_dir: np.ndarray,
    g: ArmGeometry,
    min_sin: float = 0.0,
) -> Optional[float]:
    """Arm angle psi whose elbow points along ``elbow_dir`` (arm base frame).

    ``elbow_dir`` is a measured upper-arm direction (shoulder -> elbow) already
    rotated into the arm base frame. Only its component perpendicular to the
    S-W axis enters, so human/robot arm-length and scale differences cancel:
    the robot elbow swings to the same side of the S-W axis as the human's.

    ``min_sin`` gates observability: when the human arm is nearly straight,
    the elbow sits ~on the S-W line and the perpendicular component is IOBT
    bias/noise, not signal — returning a psi then swivels the robot elbow to
    a garbage angle. Require sin(angle(elbow_dir, S-W)) >= min_sin.
    Returns None when degenerate or below the gate.
    """
    basis = _circle_basis_sw(S, W, g)
    if basis is None:
        return None
    _, _, _, ux, uy, uz, e1x, e1y, e1z, e2x, e2y, e2z, _ = basis
    dx, dy, dz = (
        float(elbow_dir[0]),
        float(elbow_dir[1]),
        float(elbow_dir[2]),
    )
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    if n < 1e-9:
        return None
    dx, dy, dz = dx / n, dy / n, dz / n
    du = dx * ux + dy * uy + dz * uz
    px, py, pz = dx - du * ux, dy - du * uy, dz - du * uz
    p2 = px * px + py * py + pz * pz  # = sin^2(angle(d, u)), d unit
    if p2 < max(1e-12, min_sin * min_sin):
        return None
    return math.atan2(px * e2x + py * e2y + pz * e2z, px * e1x + py * e1y + pz * e1z)


def _compute_sw(T: np.ndarray, g: ArmGeometry) -> Tuple[np.ndarray, np.ndarray, List[float]]:
    """Shoulder center, target wrist center, feasible elbow branches."""
    W = T[:3, 3] - T[:3, :3] @ g.flange_offset
    S = g.S
    l_sw = float(np.linalg.norm(W - S))
    if l_sw < 1e-9:
        return S, W, []
    return S, W, _solve_q4(l_sw, g)


def _ik_candidates_at_psi(
    T: np.ndarray,
    psi: float,
    g: ArmGeometry,
    S: np.ndarray,
    W: np.ndarray,
    q4_list: List[float],
    R07_rel: np.ndarray,
) -> List[np.ndarray]:
    """All branch solutions at one arm angle. Rows: [q(7), psi, S, W, E]."""
    E = _elbow_point(psi, S, W, g)
    if E is None:
        return []
    u_se_t = (E - S) / g.l_se
    u_ew_t = (W - E) / g.l_ew
    out: List[np.ndarray] = []
    for q4 in q4_list:
        R4 = _axis_angle_rot(g.axes[3], q4)
        R03 = _rotation_two_pairs(u_se_t, u_ew_t, g.v_se_hat, R4 @ g.v_ew_hat)
        if R03 is None:
            continue
        for q123 in _decompose_spherical(R03, g.axes[0], g.axes[1], g.axes[2]):
            R47 = (R03 @ R4).T @ R07_rel
            for q567 in _decompose_spherical(R47, g.axes[4], g.axes[5], g.axes[6]):
                q = wrap_to_pi(
                    np.array([q123[0], q123[1], q123[2], q4,
                              q567[0], q567[1], q567[2]], dtype=float)
                )
                if not (
                    np.all(q >= g.lower - 1e-9) and np.all(q <= g.upper + 1e-9)
                ):
                    continue
                # duplicates (double roots at singularities) are removed by the
                # dict dedupe in _collect_solutions
                out.append(np.concatenate([q, [psi], S, W, E]))
    return out


def _collect_solutions(
    T: np.ndarray,
    g: ArmGeometry,
    S: np.ndarray,
    W: np.ndarray,
    q4_list: List[float],
    psi_grid: np.ndarray,
) -> List[np.ndarray]:
    R07_rel = T[:3, :3] @ g.T_home[:3, :3].T
    seen: dict = {}
    for psi_raw in psi_grid:
        psi = float(wrap_to_pi(np.array([psi_raw]))[0])
        for cand in _ik_candidates_at_psi(T, psi, g, S, W, q4_list, R07_rel):
            # 2e-4 rad hash cells: near-duplicates (double roots) collapse,
            # genuinely distinct branches (>= ~1e-2 apart) never do.
            key = tuple(np.round(cand[:7] * 5000.0).astype(np.int64))
            if key not in seen:
                seen[key] = cand
    return list(seen.values())


# ==============================================================================
# 1D QP post-optimization (parity with the DH solver; exact FK makes it a no-op)
# ==============================================================================

def _qp_1d_objective(
    dq: float,
    idx: int,
    q_base: np.ndarray,
    T_target: np.ndarray,
    g: ArmGeometry,
    continuity: ContinuityParams,
) -> float:
    q_new = q_base.copy()
    q_new[idx] = wrap_to_pi(q_new[idx] + dq)
    q_min = float(g.lower[idx])
    q_max = float(g.upper[idx])
    if q_new[idx] < q_min or q_new[idx] > q_max:
        limit_penalty = 1e6 * abs(q_new[idx] - float(np.clip(q_new[idx], q_min, q_max)))
    else:
        limit_penalty = 0.0
    inc_cost = continuity.w_qp_joint_inc * (dq ** 2)
    pose_cost = continuity.w_qp_pose_err * (
        float(np.linalg.norm(pose_error(fk(q_new, g), T_target))) ** 2
    )
    return inc_cost + pose_cost + limit_penalty


def _optimize_q_with_1d_qp(
    q_init: np.ndarray,
    T_target: np.ndarray,
    g: ArmGeometry,
    continuity: ContinuityParams,
) -> np.ndarray:
    q_opt = q_init.copy()
    for idx in range(7):
        delta_max = min(0.05, float(g.upper[idx]) - q_opt[idx])
        delta_min = max(-0.05, float(g.lower[idx]) - q_opt[idx])
        res = minimize_scalar(
            _qp_1d_objective,
            bounds=(delta_min, delta_max),
            args=(idx, q_opt, T_target, g, continuity),
            method="bounded",
            options={"xatol": 0.005},
        )
        if res.success:
            q_opt[idx] = wrap_to_pi(q_opt[idx] + res.x)
    return q_opt


# ==============================================================================
# Continuous multi-branch solver (same selection scheme as analytic.py)
# ==============================================================================

def solve_pose_continuous_with_state(
    T_target: np.ndarray,
    state: ContinuityRuntimeState,
    g: ArmGeometry,
    n_psi: int = 181,
    continuity: Optional[ContinuityParams] = None,
    skip_qp: bool = False,
    psi_ref: Optional[float] = None,
    singular_sin_alpha: float = 0.05,
) -> Tuple[Optional[np.ndarray], dict, ContinuityRuntimeState]:
    """Continuous IK; optional ``psi_ref`` pins the arm angle to a human prior.

    When ``psi_ref`` (rad, e.g. from Quest shoulder/elbow tracking via
    ``psi_from_elbow_dir``) is given, the local psi window is centered on it
    instead of ``theta0_prev`` and the branch score gains
    ``w_psi_ref * |theta0 - psi_ref|`` — the elbow plane follows the human arm
    while w_vel/w_theta0/hysteresis still smooth out tracking noise.
    Fallback chain: psi_ref window -> theta0_prev window -> global scan, with
    escape hysteresis: the local windows must stay empty for
    ``continuity.escape_after_frames`` before going global, and once escaped
    the home window must stay feasible for ``continuity.return_after_frames``
    before returning (prevents shoulder twitch at the wrist-limit boundary
    during big end-effector rolls).
    """
    if continuity is None:
        continuity = ContinuityParams()

    T = np.array(T_target, dtype=float)
    q_prev = state.q_prev
    q_prev2 = state.q_prev2
    theta0_prev = state.theta0_prev
    q_lock = state.q_lock

    S, W, q4_list = _compute_sw(T, g)

    # Near the elbow-straight singularity (S, E, W ~collinear) the elbow
    # circle radius ~0, so the arm angle is numerically meaningless: every
    # psi gives ~the same elbow point and the scan only amplifies target
    # noise into shoulder motion. Collapse the grid to theta0_prev.
    swx, swy, swz = float(W[0] - S[0]), float(W[1] - S[1]), float(W[2] - S[2])
    l_sw = math.sqrt(swx * swx + swy * swy + swz * swz)
    singular = False
    if l_sw > 1e-12:
        x_c = (g.l_se * g.l_se - g.l_ew * g.l_ew + l_sw * l_sw) / (2.0 * l_sw)
        r2_c = g.l_se * g.l_se - x_c * x_c
        sin_alpha = math.sqrt(max(0.0, r2_c)) / g.l_se if g.l_se > 0.0 else 0.0
        singular = sin_alpha < singular_sin_alpha
    else:
        singular = True

    def _window_sols(center_psi: float, exact: Optional[float] = None) -> List[np.ndarray]:
        grid = np.linspace(
            center_psi - continuity.local_theta0_window,
            center_psi + continuity.local_theta0_window,
            max(5, continuity.local_theta0_count),
            endpoint=True,
        )
        if exact is not None:
            # Exact-psi candidate: w_psi_ref makes it win, so the elbow lands
            # on the human prior without grid quantization error.
            grid = np.append(grid, exact)
        return _collect_solutions(T, g, S, W, q4_list, grid)

    def _global_sols() -> List[np.ndarray]:
        step = min(0.03, 2.0 * math.pi / max(1, n_psi))
        psi_grid = np.arange(-math.pi, math.pi, step)
        return _collect_solutions(T, g, S, W, q4_list, psi_grid)

    def _failed(m: str):
        return (
            None,
            {"method": m, "candidate_count": 0,
             "selected_by": "failed", "pose_err_best": None},
            ContinuityRuntimeState(
                q_prev=state.q_prev,
                q_prev2=state.q_prev2,
                theta0_prev=state.theta0_prev,
                q_lock=state.q_lock,
                esc_active=esc_active,
                esc_home_psi=esc_home,
                local_fail_streak=fail_streak,
                home_ok_streak=ok_streak,
            ),
        )

    all_solutions: List[np.ndarray] = []
    method = "continuous_local_theta0" if psi_ref is None else "continuous_local_psi_ref"
    esc_active = state.esc_active
    esc_home = state.esc_home_psi
    fail_streak = state.local_fail_streak
    ok_streak = state.home_ok_streak

    if singular and q4_list:
        hold = theta0_prev if theta0_prev is not None else 0.0
        all_solutions = _collect_solutions(T, g, S, W, q4_list, np.array([hold]))
        method = "continuous_singular_hold"
        # psi is meaningless here; escapes neither help nor hurt. Keep state.
    elif esc_active:
        # Parked in a globally-escaped arm-angle region (the wrist-limit
        # boundary swallowed the local window, e.g. big end-effector roll).
        # Stay on the continuity window around theta0_prev and probe home
        # (psi_ref, else pre-escape theta0); return only after home stays
        # feasible for return_after_frames — prevents twitch-back oscillation.
        home = psi_ref if psi_ref is not None else esc_home
        probe: List[np.ndarray] = []
        if home is not None and q4_list:
            probe = _window_sols(
                home, exact=psi_ref if psi_ref is not None else None
            )
        ok_streak = ok_streak + 1 if probe else 0
        if probe and ok_streak >= continuity.return_after_frames:
            all_solutions = probe
            method = (
                "continuous_local_psi_ref"
                if psi_ref is not None
                else "continuous_local_theta0"
            )
            esc_active, esc_home, fail_streak, ok_streak = False, None, 0, 0
        else:
            if theta0_prev is not None and q4_list:
                all_solutions = _window_sols(theta0_prev)
                method = "continuous_escaped_hold"
            if not all_solutions and continuity.enable_global_fallback and q4_list:
                all_solutions = _global_sols()
                method = "continuous_global_fallback"
            if not all_solutions:
                return _failed(method)
    else:
        # Normal path: psi_ref window, then the theta0_prev continuity window.
        if psi_ref is not None and q4_list:
            all_solutions = _window_sols(psi_ref, exact=psi_ref)
        if (
            not all_solutions
            and theta0_prev is not None
            and q4_list
            and (
                psi_ref is None
                or abs(float(wrap_to_pi(np.array([psi_ref - theta0_prev]))[0]))
                > 0.5 * continuity.local_theta0_window
            )
        ):
            all_solutions = _window_sols(theta0_prev)
            method = "continuous_local_theta0"
        if all_solutions:
            fail_streak = 0
        else:
            fail_streak += 1
            ok_streak = 0
            # Escape to the global scan only after the local windows stay
            # empty for escape_after_frames (debounce against boundary
            # flicker). Cold start (no theta0_prev) escapes immediately.
            if (
                q4_list
                and continuity.enable_global_fallback
                and (theta0_prev is None or fail_streak >= continuity.escape_after_frames)
            ):
                all_solutions = _global_sols()
                method = "continuous_global_fallback"
                if all_solutions:
                    esc_active = True
                    esc_home = psi_ref if psi_ref is not None else theta0_prev
                    ok_streak = 0
            if not all_solutions:
                return _failed(method)

    if not all_solutions:
        return _failed(method)

    scored = []
    for cand in all_solutions:
        q = cand[:7]
        theta0 = float(cand[7])
        dq = wrap_to_pi(q - q_prev)
        vel_cost = float(np.linalg.norm(dq))
        if q_prev2 is not None:
            ddq = wrap_to_pi(q - 2.0 * q_prev + q_prev2)
            acc_cost = float(np.linalg.norm(ddq))
        else:
            acc_cost = 0.0
        pose_cost = float(np.linalg.norm(pose_error(fk(q, g), T)))
        if theta0_prev is None:
            theta0_cost = 0.0
        else:
            theta0_cost = abs(float(wrap_to_pi(np.array([theta0 - theta0_prev]))[0]))
        if psi_ref is None:
            psi_ref_cost = 0.0
        else:
            psi_ref_cost = abs(float(wrap_to_pi(np.array([theta0 - psi_ref]))[0]))
        score = (
            continuity.w_vel * vel_cost
            + continuity.w_acc * acc_cost
            + continuity.w_pose * pose_cost
            + continuity.w_theta0 * theta0_cost
            + continuity.w_psi_ref * psi_ref_cost
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
    theta0_best = float(q_best_full[7])

    if not skip_qp:
        q_best = _optimize_q_with_1d_qp(q_best, T, g, continuity)

    pose_best = float(np.linalg.norm(pose_error(fk(q_best, g), T)))

    next_state = ContinuityRuntimeState(
        q_prev=q_best,
        q_prev2=q_prev.copy(),
        theta0_prev=theta0_best,
        q_lock=q_best,
        esc_active=esc_active,
        esc_home_psi=esc_home,
        local_fail_streak=fail_streak,
        home_ok_streak=ok_streak,
    )
    report = {
        "method": f"{method}+1DQP" if not skip_qp else method,
        "candidate_count": len(all_solutions),
        "selected_by": selected_by,
        "score_best": float(selected[0]),
        "pose_err_best": pose_best,
        "theta0_selected": theta0_best,
        "psi_ref": None if psi_ref is None else float(psi_ref),
    }
    return q_best, report, next_state


# ==============================================================================
# GeometricIKSolver — drop-in for analytic.IKSolver (URDF/hardware convention)
# ==============================================================================

class GeometricIKSolver:
    """DH-free S/E/W arm-angle IK for one Astral arm in ``*_base_link``.

    Same public API as ``analytic.IKSolver`` (``solve`` / ``fk`` /
    ``sync_state`` / limits), but solutions are directly in the URDF/hardware
    joint sign convention — no flip_q anywhere.
    """

    def __init__(
        self,
        arm_side: str = "left",
        urdf_path: str = "",
        continuity_params: Optional[ContinuityParams] = None,
        fast_mode: bool = True,
        escape_after_frames: Optional[int] = None,
        return_after_frames: Optional[int] = None,
    ):
        side = arm_side.strip().lower()
        if side.startswith("l"):
            self.arm = "L"
            self._prefix = "left"
        elif side.startswith("r"):
            self.arm = "R"
            self._prefix = "right"
        else:
            raise ValueError("arm_side must be left|right")
        self.geom = extract_arm_geometry(urdf_path, arm_side)
        self.continuity = (
            continuity_params if continuity_params is not None else ContinuityParams()
        )
        if escape_after_frames is not None:
            self.continuity.escape_after_frames = int(escape_after_frames)
        if return_after_frames is not None:
            self.continuity.return_after_frames = int(return_after_frames)
        self.fast_mode = fast_mode
        self._state = ContinuityRuntimeState(
            q_prev=np.zeros(7, dtype=float),
            q_prev2=None,
            theta0_prev=None,
            q_lock=None,
        )

    # ---- Properties ----

    @property
    def method_name(self) -> str:
        return f"geometric_swe_arm_angle/{self._prefix}"

    @property
    def nq(self) -> int:
        return 7

    @property
    def lower_limits(self) -> np.ndarray:
        return self.geom.lower.copy()

    @property
    def upper_limits(self) -> np.ndarray:
        return self.geom.upper.copy()

    # ---- Core methods ----

    def solve(
        self, T_target: np.ndarray, psi_ref: Optional[float] = None
    ) -> Optional[np.ndarray]:
        """Solve IK for the flange pose in ``*_base_link`` (same frame as fk).

        ``psi_ref``: optional human arm-angle prior (rad) — see
        ``arm_angle_from_elbow_dir``; the elbow plane tracks it softly.
        """
        state_prev = self._state
        q_best, _report, new_state = solve_pose_continuous_with_state(
            np.array(T_target, dtype=float),
            state=self._state,
            g=self.geom,
            n_psi=91,
            continuity=self.continuity,
            skip_qp=self.fast_mode,
            psi_ref=psi_ref,
        )
        if q_best is None:
            # Keep the escape-hysteresis counters across failed frames (they
            # are the whole point of the debounce); pose/branch state stays.
            state_prev.esc_active = new_state.esc_active
            state_prev.esc_home_psi = new_state.esc_home_psi
            state_prev.local_fail_streak = new_state.local_fail_streak
            state_prev.home_ok_streak = new_state.home_ok_streak
            self._state = state_prev
            return None
        self._state = new_state
        return q_best.reshape(7).copy()

    def solve_hard(
        self,
        T_target: np.ndarray,
        psi: float,
        q_init: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """Deterministic IK at an *exact* arm angle ``psi`` — hard human-elbow
        mode (``human_elbow_mode: "hard"`` in the node).

        Unlike ``solve(psi_ref=...)`` — which treats psi as a soft prior
        blended with continuity and layers window/escape hysteresis on top —
        this resolves the arm exactly at ``psi`` (same convention as
        ``psi_from_elbow_dir`` / ``arm_angle_from_elbow_dir``) with only a
        minimal warm-start branch pick. Mirrors
        ``astral_pim_ik.geometry.GeometricArmAngleSolver.solve`` (kept here
        because teleop must not import astral_pim_ik — that package imports
        this one). No grid scan, no 1D QP, no escape hysteresis.

        Returns q (7,) or ``None`` when the pose is unreachable / out of
        limits / singular at this psi. On success the internal continuity
        state is synced (so a later soft solve warm-starts smoothly); on
        failure the state is left untouched for the soft path to take over.
        """
        T = np.array(T_target, dtype=float)
        psi = float(psi)
        q_prev = (
            np.asarray(q_init, dtype=float).reshape(7)
            if q_init is not None
            else np.array(self._state.q_prev, dtype=float).copy()
        )
        S, W, q4_list = _compute_sw(T, self.geom)
        if not q4_list:
            return None
        cands = _collect_solutions(T, self.geom, S, W, q4_list, np.array([psi]))
        if not cands:
            return None
        best = min(
            cands, key=lambda c: float(np.linalg.norm(wrap_to_pi(c[:7] - q_prev)))
        )
        q_best = np.asarray(best[:7], dtype=float).copy()
        # Sync continuity state so a fallback to the soft solve() later
        # warm-starts from what the arm is actually doing.
        st = self._state
        st.q_prev2 = np.array(st.q_prev, dtype=float).copy()
        st.q_prev = q_best.copy()
        st.theta0_prev = float(best[7])
        st.q_lock = q_best.copy()
        st.esc_active = False
        st.esc_home_psi = None
        st.local_fail_streak = 0
        st.home_ok_streak = 0
        return q_best

    def arm_angle_from_elbow_dir(
        self,
        T_target: np.ndarray,
        elbow_dir: np.ndarray,
        min_sin: float = 0.0,
    ) -> Optional[float]:
        """Human upper-arm direction -> arm-angle prior for this flange target.

        ``elbow_dir`` is the human shoulder->elbow direction expressed in the
        arm base frame (``*_base_link``; rotate from the VR frame with the
        same ``vr_to_arm_rot`` used for wrist deltas — direction only, no
        zero-point/scale). ``min_sin`` gates the human-arm straightness below
        which psi is unobservable (nearly straight arm; see
        ``psi_from_elbow_dir``). Returns psi (rad) for ``solve(psi_ref=...)``,
        or None when the target/direction is degenerate or gated.
        """
        T = np.array(T_target, dtype=float)
        S, W, q4_list = _compute_sw(T, self.geom)
        if not q4_list:
            return None
        d = np.asarray(elbow_dir, dtype=float).reshape(3)
        if not np.isfinite(d).all():
            return None
        return psi_from_elbow_dir(S, W, d, self.geom, min_sin=min_sin)

    def clamp_wrist_reach(
        self, p_wrist: np.ndarray, margin: float = 0.01
    ) -> Tuple[np.ndarray, bool]:
        """Radial soft wall at the elbow-straight singularity.

        Clamps the wrist-center target to
        ``|l_se - l_ew| + margin <= |p - S| <= l_se + l_ew - margin``.
        Past the outer boundary the elbow circle degenerates and the q4
        limit rejects every branch — solve() returns None, the node holds
        the last command, and the arm visibly catches when IK recovers.
        Direction from S is preserved, so the hand feels a soft wall.
        Returns ``(p_clamped, was_clipped)``.
        """
        p = np.asarray(p_wrist, dtype=float).reshape(3)
        S = np.asarray(self.geom.S, dtype=float)
        v = p - S
        d = float(np.linalg.norm(v))
        if d < 1e-9:
            return p.copy(), False
        m = max(0.0, float(margin))
        r_max = float(self.geom.l_se + self.geom.l_ew) - m
        r_min = abs(float(self.geom.l_se - self.geom.l_ew)) + m
        d_c = min(max(d, r_min), r_max)
        if d_c == d:
            return p.copy(), False
        return S + v * (d_c / d), True

    def fk(self, q: np.ndarray) -> np.ndarray:
        """FK: joints -> flange pose in ``left_base_link`` / ``right_base_link``."""
        return fk(np.asarray(q, dtype=float).reshape(7), self.geom)

    def sync_state(self, q: np.ndarray, reset_branch: bool = True) -> None:
        """Set warm-start joint state for the next solve() call."""
        q_7 = np.asarray(q, dtype=float).reshape(7)
        self._state.q_prev2 = self._state.q_prev.copy()
        self._state.q_prev = q_7.copy()
        if reset_branch:
            self._state.theta0_prev = None
            self._state.q_lock = None
            self._state.esc_active = False
            self._state.esc_home_psi = None
            self._state.local_fail_streak = 0
            self._state.home_ok_streak = 0

    def elbow_direction(self, q: np.ndarray) -> np.ndarray:
        """Upper-arm direction (shoulder → elbow, unit, arm base frame) of ``q``.

        ``u = R03 @ v_se_hat`` — the shoulder rotation from q1-q3 applied to
        the home upper-arm direction. Used to re-anchor the human-elbow
        reference (``_elbow_dir_vr``) to the arm's *current* configuration at
        HITL ``~/reanchor`` / ``_start_teleop``: the first ``solve_hard`` then
        keeps the current arm angle (no elbow snap), and the live human-elbow
        EMA blends in smoothly as the operator moves.
        """
        q_7 = np.asarray(q, dtype=float).reshape(7)
        g = self.geom
        R03 = (
            _axis_angle_rot(g.axes[0], q_7[0])
            @ _axis_angle_rot(g.axes[1], q_7[1])
            @ _axis_angle_rot(g.axes[2], q_7[2])
        )
        u = R03 @ g.v_se_hat
        n = float(np.linalg.norm(u))
        return u / n if n > 1e-9 else g.v_se_hat.copy()

    def check_self_collision(self, q: np.ndarray) -> bool:
        """Stub: no collision geometry model in the geometric solver."""
        return False

    def active_joint_names(self) -> List[str]:
        return [f"{self._prefix}_joint{i}" for i in range(1, 8)]
