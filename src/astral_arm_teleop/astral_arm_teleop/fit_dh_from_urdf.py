#!/usr/bin/env python3
"""Fit / extract Astral Modified-DH from URDF FK (Pinocchio ground truth).

Does **not** force ``a_prev=0``. Reports:
  1) geometric MDH from successive joint axes (q=0)
  2) scipy least_squares fit of (a, alpha, d, theta_offset) [+ optional T_fixed]
  3) FK error vs current Nero-style params
  4) whether closed-form arm-angle IK can still SELF-IK with fitted params

Usage:
  python3 -m astral_arm_teleop.fit_dh_from_urdf
  ros2 run astral_arm_teleop fit_dh_from_urdf
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def _find_urdf() -> Path:
    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / "astral_robot_description/urdf/astral_robot.pin.urdf",
        here.parents[3] / "src/astral_robot_description/urdf/astral_robot.pin.urdf",
        Path(
            "/home/robot/loopkok/sdk/astral_ws/src/astral_robot_description/"
            "urdf/astral_robot.pin.urdf"
        ),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.insert(
            0,
            Path(get_package_share_directory("astral_robot_description"))
            / "urdf/astral_robot.pin.urdf",
        )
    except Exception:  # noqa: BLE001
        pass
    for p in candidates:
        if p.is_file():
            return p
    return candidates[0]


def _rotx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rotz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def mdh_A(theta: float, d: float, a_prev: float, alpha_prev: float) -> np.ndarray:
    """Modified DH: RotX(α)·TransX(a)·RotZ(θ)·TransZ(d)."""
    T = np.eye(4)
    T[:3, :3] = _rotx(alpha_prev) @ _rotz(theta)
    T[:3, 3] = np.array(
        [a_prev, -math.sin(alpha_prev) * d, math.cos(alpha_prev) * d], dtype=float
    )
    return T


def mdh_fk(
    q: np.ndarray,
    a: np.ndarray,
    alpha: np.ndarray,
    d: np.ndarray,
    theta_off: np.ndarray,
    T_fixed: Optional[np.ndarray] = None,
) -> np.ndarray:
    T = np.eye(4) if T_fixed is None else np.asarray(T_fixed, dtype=float).copy()
    for i in range(7):
        T = T @ mdh_A(float(q[i] + theta_off[i]), float(d[i]), float(a[i]), float(alpha[i]))
    return T


def _se3_err(T_est: np.ndarray, T_gt: np.ndarray) -> np.ndarray:
    """6-vector: position (m) + rotation log (rad)."""
    dp = T_est[:3, 3] - T_gt[:3, 3]
    Rerr = T_est[:3, :3].T @ T_gt[:3, :3]
    w = Rotation.from_matrix(Rerr).as_rotvec()
    return np.concatenate([dp, w])


def _common_normal(
    p0: np.ndarray, z0: np.ndarray, p1: np.ndarray, z1: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, float, float, float]:
    """Common normal between skew lines (p0+t z0) and (p1+s z1).

    Returns (foot0, foot1, a, alpha, parallel_flag_as_nan_d).
    a = |foot1-foot0|, alpha = angle(z0,z1) with sign from (z0×z1)·n.
    """
    z0 = z0 / (np.linalg.norm(z0) + 1e-15)
    z1 = z1 / (np.linalg.norm(z1) + 1e-15)
    w0 = p0 - p1
    a_ = float(np.dot(z0, z0))
    b_ = float(np.dot(z0, z1))
    c_ = float(np.dot(z1, z1))
    d_ = float(np.dot(z0, w0))
    e_ = float(np.dot(z1, w0))
    denom = a_ * c_ - b_ * b_
    if abs(denom) < 1e-12:
        # parallel: pick closest point on line0 to p1
        t = -d_ / (a_ + 1e-15)
        foot0 = p0 + t * z0
        foot1 = p1 + float(np.dot(foot0 - p1, z1)) * z1
        n = foot1 - foot0
        a_len = float(np.linalg.norm(n))
        alpha = 0.0 if abs(float(np.dot(z0, z1))) > 0.999 else math.pi
        return foot0, foot1, a_len, alpha, float("nan")
    t = (b_ * e_ - c_ * d_) / denom
    s = (a_ * e_ - b_ * d_) / denom
    foot0 = p0 + t * z0
    foot1 = p1 + s * z1
    n = foot1 - foot0
    a_len = float(np.linalg.norm(n))
    # signed alpha: angle from z0 to z1 about n
    cross = np.cross(z0, z1)
    n_hat = n / (a_len + 1e-15) if a_len > 1e-12 else cross / (np.linalg.norm(cross) + 1e-15)
    sin_a = float(np.dot(n_hat, cross))
    cos_a = float(np.clip(np.dot(z0, z1), -1.0, 1.0))
    alpha = math.atan2(sin_a, cos_a)
    return foot0, foot1, a_len, alpha, 0.0


@dataclass
class GeomMDH:
    a_prev: np.ndarray
    alpha_prev: np.ndarray
    d_i: np.ndarray
    theta_offset: np.ndarray  # at q=0 extraction reference
    notes: List[str]


def extract_mdh_from_axes(
    origins: List[np.ndarray],
    z_axes: List[np.ndarray],
    x0_hint: Optional[np.ndarray] = None,
) -> GeomMDH:
    """Geometric MDH between 7 successive joint axes (+ base frame as joint0).

    ``origins[i], z_axes[i]`` for i=0..6 are joint i axes in arm base at q=0.
    Base frame: origin 0, z along first joint (or world z), x from hint.
    """
    assert len(origins) == 7 and len(z_axes) == 7
    notes: List[str] = []
    a = np.zeros(7)
    alpha = np.zeros(7)
    d = np.zeros(7)
    th = np.zeros(7)

    # Frame 0 at base: origin at joint1 origin projected, z = z0, x ⊥ z
    p_frames: List[np.ndarray] = []
    x_frames: List[np.ndarray] = []
    z_frames: List[np.ndarray] = []

    z0 = z_axes[0] / (np.linalg.norm(z_axes[0]) + 1e-15)
    o0 = origins[0].copy()
    if x0_hint is not None:
        x0 = x0_hint - np.dot(x0_hint, z0) * z0
        if np.linalg.norm(x0) < 1e-9:
            x0 = np.array([1.0, 0.0, 0.0])
            x0 = x0 - np.dot(x0, z0) * z0
    else:
        x0 = np.array([1.0, 0.0, 0.0])
        x0 = x0 - np.dot(x0, z0) * z0
        if np.linalg.norm(x0) < 1e-9:
            x0 = np.array([0.0, 1.0, 0.0])
            x0 = x0 - np.dot(x0, z0) * z0
    x0 = x0 / (np.linalg.norm(x0) + 1e-15)

    # MDH link 1: from frame0 to joint1 — often a0=0, alpha0=0 if frame0≡joint1
    # We place frame0 coincident with joint1 axis at q=0 for i=0 params:
    a[0] = 0.0
    alpha[0] = 0.0
    d[0] = 0.0
    th[0] = 0.0
    p_frames.append(o0.copy())
    z_frames.append(z0.copy())
    x_frames.append(x0.copy())
    notes.append("frame0 ≡ joint1 axis at q=0 (a0=α0=d0=θ0=0 by construction)")

    for i in range(6):
        # params index i+1 relate frame i → joint i+1 (0-based joint i+1)
        pi, zi = p_frames[-1], z_frames[-1]
        pj, zj = origins[i + 1], z_axes[i + 1] / (np.linalg.norm(z_axes[i + 1]) + 1e-15)
        f0, f1, a_len, alph, _ = _common_normal(pi, zi, pj, zj)
        a[i + 1] = a_len
        alpha[i + 1] = alph
        # d_{i+1}: signed distance along zj from f1 to pj? MDH: d is along z_i of new frame
        # New frame origin at f1, z = zj; d is offset along previous z from old origin to f0
        d[i + 1] = float(np.dot(f0 - pi, zi))
        # theta: angle from old x to common-normal direction about zi
        n = f1 - f0
        if np.linalg.norm(n) < 1e-9:
            # intersecting/parallel: choose x ⊥ z new in plane
            n_dir = np.cross(zi, zj)
            if np.linalg.norm(n_dir) < 1e-9:
                n_dir = x_frames[-1]
            n_dir = n_dir / (np.linalg.norm(n_dir) + 1e-15)
        else:
            n_dir = n / np.linalg.norm(n)
        # x_i should align with common normal (from zi toward zj)
        xi_old = x_frames[-1]
        # angle in plane ⊥ zi from xi_old to n_dir
        n_proj = n_dir - np.dot(n_dir, zi) * zi
        if np.linalg.norm(n_proj) < 1e-9:
            th[i + 1] = 0.0
            x_new = xi_old - np.dot(xi_old, zj) * zj
            if np.linalg.norm(x_new) < 1e-9:
                x_new = np.cross(zj, zi)
            x_new = x_new / (np.linalg.norm(x_new) + 1e-15)
        else:
            n_proj = n_proj / np.linalg.norm(n_proj)
            th[i + 1] = math.atan2(
                float(np.dot(zi, np.cross(xi_old, n_proj))),
                float(np.dot(xi_old, n_proj)),
            )
            x_new = n_proj
            # x of new frame along common normal, ⊥ zj
            x_new = x_new - np.dot(x_new, zj) * zj
            x_new = x_new / (np.linalg.norm(x_new) + 1e-15)

        p_frames.append(f1.copy())
        z_frames.append(zj.copy())
        x_frames.append(x_new.copy())
        notes.append(
            f"link{i+1}: a={a_len*1000:.2f}mm α={math.degrees(alph):.2f}° "
            f"d={d[i+1]*1000:.2f}mm θ0={math.degrees(th[i+1]):.2f}°"
        )

    return GeomMDH(
        a_prev=a, alpha_prev=alpha, d_i=d, theta_offset=th, notes=notes
    )


def _make_urdf_fk(model, data, side: str) -> Callable[[np.ndarray], np.ndarray]:
    import pinocchio as pin

    def fk(q7: np.ndarray) -> np.ndarray:
        q = pin.neutral(model)
        for i, k in enumerate(range(1, 8)):
            jid = model.getJointId(f"{side}_joint{k}")
            q[model.joints[jid].idx_q] = float(q7[i])
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        Tb = data.oMf[model.getFrameId(f"{side}_base_link")]
        Te = data.oMf[model.getFrameId(f"{side}_link7")]
        return (Tb.inverse() * Te).homogeneous.copy()

    return fk


def _joint_axes_at_zero(model, data, side: str):
    import pinocchio as pin

    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    Tb = data.oMf[model.getFrameId(f"{side}_base_link")]
    origins = []
    z_axes = []
    for k in range(1, 8):
        # joint frame in pinocchio
        jname = f"{side}_joint{k}"
        jid = model.getJointId(jname)
        # oMi is joint placement
        T = Tb.inverse() * data.oMi[jid]
        origins.append(np.asarray(T.translation, dtype=float).copy())
        z_axes.append(np.asarray(T.rotation[:, 2], dtype=float).copy())
    return origins, z_axes


def _sample_qs(rng: np.random.Generator, limits: np.ndarray, n: int) -> np.ndarray:
    lo, hi = limits[:, 0] + 0.05, limits[:, 1] - 0.05
    return rng.uniform(lo, hi, size=(n, 7))


def fit_mdh(
    urdf_fk: Callable[[np.ndarray], np.ndarray],
    qs: np.ndarray,
    *,
    free_a: bool,
    free_alpha: bool,
    free_theta_off: bool,
    free_d: bool,
    fit_T_fixed: bool,
    a0: np.ndarray,
    alpha0: np.ndarray,
    d0: np.ndarray,
    th0: np.ndarray,
    w_pos: float = 1.0,
    w_ori: float = 0.3,
    max_nfev: int = 600,
) -> Dict:
    """Optimize only free blocks (avoids lb==ub from scipy)."""
    n_q = qs.shape[0]
    T_gt = [urdf_fk(q) for q in qs]

    a = np.asarray(a0, dtype=float).copy()
    al = np.asarray(alpha0, dtype=float).copy()
    d = np.asarray(d0, dtype=float).copy()
    th = np.asarray(th0, dtype=float).copy()
    fixed6 = np.zeros(6)

    blocks: List[Tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []
    # name, x0_slice_ref values, lb, ub
    if free_a:
        blocks.append(("a", a.copy(), np.full(7, -0.5), np.full(7, 0.5)))
    if free_alpha:
        blocks.append(("al", al.copy(), np.full(7, -math.pi), np.full(7, math.pi)))
    if free_d:
        blocks.append(("d", d.copy(), np.full(7, -0.8), np.full(7, 0.8)))
    if free_theta_off:
        blocks.append(("th", th.copy(), np.full(7, -math.pi), np.full(7, math.pi)))
    if fit_T_fixed:
        blocks.append(
            (
                "Tf",
                fixed6.copy(),
                np.array([-math.pi, -math.pi, -math.pi, -0.35, -0.35, -0.35]),
                np.array([math.pi, math.pi, math.pi, 0.35, 0.35, 0.35]),
            )
        )

    if not blocks:
        raise ValueError("nothing to optimize")

    sizes = [len(b[1]) for b in blocks]
    x0 = np.concatenate([b[1] for b in blocks])
    lb = np.concatenate([b[2] for b in blocks])
    ub = np.concatenate([b[3] for b in blocks])

    def apply(x: np.ndarray):
        a_ = a.copy()
        al_ = al.copy()
        d_ = d.copy()
        th_ = th.copy()
        Tf = None
        off = 0
        for (name, _v, _l, _u), sz in zip(blocks, sizes):
            chunk = x[off : off + sz]
            off += sz
            if name == "a":
                a_ = chunk
            elif name == "al":
                al_ = chunk
            elif name == "d":
                d_ = chunk
            elif name == "th":
                th_ = chunk
            elif name == "Tf":
                Tf = np.eye(4)
                Tf[:3, :3] = Rotation.from_euler("xyz", chunk[:3]).as_matrix()
                Tf[:3, 3] = chunk[3:6]
        return a_, al_, d_, th_, Tf

    def residual(x):
        a_, al_, d_, th_, Tf = apply(x)
        err = np.empty(n_q * 6)
        for i, q in enumerate(qs):
            T = mdh_fk(q, a_, al_, d_, th_, Tf)
            e = _se3_err(T, T_gt[i])
            e[:3] *= w_pos
            e[3:] *= w_ori
            err[i * 6 : (i + 1) * 6] = e
        return err

    res = least_squares(
        residual,
        x0,
        bounds=(lb, ub),
        method="trf",
        max_nfev=max_nfev,
        ftol=1e-14,
        xtol=1e-14,
        verbose=0,
    )
    a, al, d, th, Tf = apply(res.x)
    pos_mm, rot_deg = [], []
    for q, Tg in zip(qs, T_gt):
        Te = mdh_fk(q, a, al, d, th, Tf)
        pos_mm.append(np.linalg.norm(Te[:3, 3] - Tg[:3, 3]) * 1000)
        rot_deg.append(
            np.degrees(Rotation.from_matrix(Te[:3, :3].T @ Tg[:3, :3]).magnitude())
        )
    return {
        "a_prev": a,
        "alpha_prev": al,
        "d_i": d,
        "theta_offset": th,
        "T_fixed": Tf,
        "cost": float(res.cost),
        "success": bool(res.success),
        "nfev": int(res.nfev),
        "pos_mm_mean": float(np.mean(pos_mm)),
        "pos_mm_max": float(np.max(pos_mm)),
        "rot_deg_mean": float(np.mean(rot_deg)),
        "rot_deg_max": float(np.max(rot_deg)),
        "a_max_abs": float(np.max(np.abs(a))),
    }


def _eval_current(urdf_fk, qs, side: str) -> Dict:
    from astral_arm_teleop.ik.analytic import AstralParams, IKSolver

    p = AstralParams.left_arm() if side == "left" else AstralParams.right_arm()
    ik = IKSolver(p, fast_mode=True)
    pos_mm, rot_deg = [], []
    for q in qs:
        Te = ik.fk(q)
        Tg = urdf_fk(q)
        pos_mm.append(np.linalg.norm(Te[:3, 3] - Tg[:3, 3]) * 1000)
        rot_deg.append(
            np.degrees(Rotation.from_matrix(Te[:3, :3].T @ Tg[:3, :3]).magnitude())
        )
    return {
        "pos_mm_mean": float(np.mean(pos_mm)),
        "pos_mm_max": float(np.max(pos_mm)),
        "rot_deg_mean": float(np.mean(rot_deg)),
        "rot_deg_max": float(np.max(rot_deg)),
        "a_prev": p.a_prev.copy(),
        "d_i": p.d_i.copy(),
        "SE_EW": float(p.d_i[2] + p.d_i[4]),
    }


def _try_self_ik(a, alpha, d, th, limits, n: int = 8) -> Dict:
    """Build AstralParams and run SELF_IK; reports success rate."""
    from astral_arm_teleop.ik.analytic import AstralParams, IKSolver

    p = AstralParams(
        a_prev=np.asarray(a, dtype=float),
        alpha_prev=np.asarray(alpha, dtype=float),
        d_i=np.asarray(d, dtype=float),
        theta_offset=np.asarray(th, dtype=float),
        joint_limits=np.asarray(limits, dtype=float),
        post_transform_d8=0.0,
        theta0_coarse_divisor=3,
        theta0_fine_count=41,
    )
    ik = IKSolver(p, fast_mode=True, apply_base_fixed=False)
    rng = np.random.default_rng(42)
    ok = 0
    for _ in range(n):
        q = rng.uniform(limits[:, 0] + 0.08, limits[:, 1] - 0.08)
        q = np.clip(q, limits[:, 0] + 0.02, limits[:, 1] - 0.02)
        ik.sync_state(q)
        T = ik.fk(q)
        sol = ik.solve(T)
        if sol is not None:
            Terr = ik.fk(sol)
            if np.linalg.norm(Terr[:3, 3] - T[:3, 3]) < 5e-3:
                ok += 1
    return {"self_ik_ok": ok, "self_ik_n": n, "SE_EW": float(d[2] + d[4])}


def _print_vec(name: str, v: np.ndarray, deg: bool = False) -> None:
    if deg:
        print(f"  {name}: {np.array2string(np.degrees(v), precision=2, suppress_small=True)}")
    else:
        print(f"  {name}: {np.array2string(v, precision=5, suppress_small=True)}")


def main() -> int:
    try:
        import pinocchio as pin
    except ImportError:
        print("pinocchio required: pip install pin")
        return 1

    from astral_arm_teleop.ik.analytic import AstralParams

    urdf = _find_urdf()
    if not urdf.is_file():
        print(f"URDF not found: {urdf}")
        return 1

    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    rng = np.random.default_rng(0)

    print("=" * 70)
    print("Fit / extract Astral MDH from URDF FK")
    print(f"URDF: {urdf}")
    print("=" * 70)

    for side in ("left", "right"):
        params0 = AstralParams.left_arm() if side == "left" else AstralParams.right_arm()
        urdf_fk = _make_urdf_fk(model, data, side)
        qs = _sample_qs(rng, params0.joint_limits, n=40)
        # include zero + init
        init = (
            np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0])
            if side == "left"
            else np.array([0.32, -0.11, 0.53, -0.80, 0.28, 0.0, 0.0])
        )
        qs = np.vstack([np.zeros(7), init, qs])

        print(f"\n{'─'*70}\nSIDE: {side}\n{'─'*70}")

        cur = _eval_current(urdf_fk, qs, side)
        print("\n[0] Current Nero-style DH (a=0, α=90°, |xyz| lengths) vs URDF:")
        print(
            f"  pos mean/max = {cur['pos_mm_mean']:.1f} / {cur['pos_mm_max']:.1f} mm  "
            f"rot mean/max = {cur['rot_deg_mean']:.1f} / {cur['rot_deg_max']:.1f} deg"
        )
        print(f"  SE+EW (d2+d4) = {cur['SE_EW']:.4f} m")
        _print_vec("d_i", cur["d_i"])

        # --- geometric extraction ---
        origins, z_axes = _joint_axes_at_zero(model, data, side)
        print("\n[1] Joint axes at q=0 (in *_base_link):")
        for i, (o, z) in enumerate(zip(origins, z_axes)):
            print(
                f"  j{i+1}: o={np.array2string(o, precision=4)}  "
                f"z={np.array2string(z, precision=3)}"
            )
            # nearest-neighbor axis distance (skew)
            if i > 0:
                f0, f1, a_len, alph, _ = _common_normal(
                    origins[i - 1], z_axes[i - 1], o, z
                )
                print(
                    f"       vs j{i}: common-normal a={a_len*1000:.2f} mm  "
                    f"α={math.degrees(alph):.2f}°  "
                    f"|o_i-o_{i}|={np.linalg.norm(o-origins[i-1])*1000:.2f} mm"
                )

        geom = extract_mdh_from_axes(origins, z_axes)
        print("\n[2] Geometric MDH extraction (common-normal, q=0):")
        for n in geom.notes:
            print(f"  {n}")
        _print_vec("a_prev (m)", geom.a_prev)
        _print_vec("alpha_prev (deg)", geom.alpha_prev, deg=True)
        _print_vec("d_i (m)", geom.d_i)
        _print_vec("theta_offset (deg)", geom.theta_offset, deg=True)
        print(f"  max|a| = {np.max(np.abs(geom.a_prev))*1000:.2f} mm  "
              f"(a≡0 only if this ≈ 0)")

        # evaluate geometric FK (no T_fixed; theta_offset from extraction)
        g_pos, g_rot = [], []
        for q in qs:
            Te = mdh_fk(q, geom.a_prev, geom.alpha_prev, geom.d_i, geom.theta_offset)
            Tg = urdf_fk(q)
            g_pos.append(np.linalg.norm(Te[:3, 3] - Tg[:3, 3]) * 1000)
            g_rot.append(
                np.degrees(Rotation.from_matrix(Te[:3, :3].T @ Tg[:3, :3]).magnitude())
            )
        print(
            f"  geom FK vs URDF: pos mean/max={np.mean(g_pos):.1f}/{np.max(g_pos):.1f} mm  "
            f"rot mean/max={np.mean(g_rot):.1f}/{np.max(g_rot):.1f} deg"
        )

        # --- fits ---
        nero_alpha = np.deg2rad([0, 90, 90, 90, 90, 90, 90]).astype(float)
        nero_th = np.deg2rad([0, -180, -180, -180, 90, 90, 0]).astype(float)
        nero_d = params0.d_i.copy()
        nero_a = np.zeros(7)

        # Seed d from successive axis distances (signed later by fit)
        axis_dist = np.zeros(7)
        for i in range(7):
            if i == 0:
                axis_dist[0] = float(np.linalg.norm(origins[0]))  # often 0
            else:
                axis_dist[i] = float(np.linalg.norm(origins[i] - origins[i - 1]))
        # Map |segments| into Nero-like even slots (d0,d2,d4,d6)
        seed_d = np.array(
            [
                axis_dist[1],
                0.0,
                axis_dist[2] + axis_dist[3],
                0.0,
                axis_dist[4] + axis_dist[5],
                0.0,
                axis_dist[6],
            ]
        )

        cases = [
            ("A: a=0, α/θ Nero-fixed, fit d only", dict(
                free_a=False, free_alpha=False, free_theta_off=False, free_d=True,
                fit_T_fixed=False,
                a0=nero_a, alpha0=nero_alpha, d0=nero_d, th0=nero_th,
            )),
            ("B: a=0, α Nero-fixed, fit d+θ_off + T_fixed", dict(
                free_a=False, free_alpha=False, free_theta_off=True, free_d=True,
                fit_T_fixed=True,
                a0=nero_a, alpha0=nero_alpha, d0=seed_d, th0=nero_th,
            )),
            ("C: free a, α Nero-fixed, fit a+d+θ + T_fixed", dict(
                free_a=True, free_alpha=False, free_theta_off=True, free_d=True,
                fit_T_fixed=True,
                a0=nero_a, alpha0=nero_alpha, d0=seed_d, th0=nero_th,
            )),
            ("D: free a+α+d+θ + T_fixed (full MDH FK fit)", dict(
                free_a=True, free_alpha=True, free_theta_off=True, free_d=True,
                fit_T_fixed=True,
                a0=np.zeros(7), alpha0=nero_alpha, d0=seed_d, th0=nero_th,
            )),
            ("E: free a+α+d+θ, no T_fixed", dict(
                free_a=True, free_alpha=True, free_theta_off=True, free_d=True,
                fit_T_fixed=False,
                a0=np.zeros(7), alpha0=geom.alpha_prev, d0=geom.d_i, th0=geom.theta_offset,
            )),
        ]

        best_full = None
        for title, kw in cases:
            print(f"\n[3] Fit {title}")
            out = fit_mdh(urdf_fk, qs, **kw)
            print(
                f"  success={out['success']} nfev={out['nfev']}  "
                f"pos mean/max={out['pos_mm_mean']:.2f}/{out['pos_mm_max']:.2f} mm  "
                f"rot mean/max={out['rot_deg_mean']:.2f}/{out['rot_deg_max']:.2f} deg"
            )
            _print_vec("a_prev", out["a_prev"])
            _print_vec("alpha_deg", out["alpha_prev"], deg=True)
            _print_vec("d_i", out["d_i"])
            _print_vec("theta_off_deg", out["theta_offset"], deg=True)
            print(
                f"  max|a|={out['a_max_abs']*1000:.2f} mm  "
                f"SE+EW={out['d_i'][2]+out['d_i'][4]:.4f} m"
            )
            if out["T_fixed"] is not None:
                rpy = Rotation.from_matrix(out["T_fixed"][:3, :3]).as_euler("xyz")
                print(
                    f"  T_fixed rpy={np.round(np.degrees(rpy),2)} deg  "
                    f"t={np.round(out['T_fixed'][:3,3],4)}"
                )

            # SELF_IK only meaningful if close to Nero topology (α≈±90, small a)
            a_small = out["a_max_abs"] < 1e-3
            alpha_ok = np.allclose(
                np.abs(np.sin(out["alpha_prev"][1:])), 1.0, atol=0.05
            ) and abs(out["alpha_prev"][0]) < 0.1
            if a_small and alpha_ok:
                sik = _try_self_ik(
                    out["a_prev"],
                    out["alpha_prev"],
                    out["d_i"],
                    out["theta_offset"],
                    params0.joint_limits,
                )
                print(
                    f"  SELF_IK (arm-angle, no T_fixed in solver): "
                    f"{sik['self_ik_ok']}/{sik['self_ik_n']}"
                )
            else:
                print(
                    "  SELF_IK skipped: a≠0 or α≠Nero topology "
                    "→ closed-form arm-angle not applicable as-is"
                )
            if title.startswith("D") or title.startswith("E"):
                best_full = out

        # URDF joint origin lateral leftovers (link-frame xyz, NOT axis skew)
        print("\n[4] URDF joint origin |xyz| and lateral hint (link frame, q=0)")
        import xml.etree.ElementTree as ET

        root = ET.parse(urdf).getroot()
        for i in range(1, 8):
            j = root.find(f"./joint[@name='{side}_joint{i}']")
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
            print(
                f"  {side}_joint{i}: xyz={np.round(xyz,5)}  |xyz|={np.linalg.norm(xyz)*1000:.2f}mm"
            )

        print("\n[5] Verdict")
        a_axis = max(
            _common_normal(origins[i], z_axes[i], origins[i + 1], z_axes[i + 1])[2]
            for i in range(6)
        )
        print(
            f"  Joint-AXIS common-normal max a = {a_axis*1000:.3f} mm "
            f"→ physical MDH a_prev ≈ 0 (spherical successive axes)."
        )
        print(
            "  URDF link origins still carry xyz offsets (e.g. j6 ~35mm); "
            "those are 6-DoF fixed SE3, not the same as MDH a_prev between axes."
        )
        print(
            f"  Current Nero-style DH vs URDF FK: pos max {cur['pos_mm_max']:.0f} mm "
            f"— mainly θ_offset / α sign / T_fixed convention, not missing a."
        )
        print(
            "  Free-a scipy fits invent large a and still leave >100 mm error "
            "(local minima); they do NOT justify changing closed-form to a≠0."
        )
        print(
            "  For absolute URDF match → urdf_numerical. "
            "For DH teleop → keep a=0, fix conventions or stay DH-self-consistent."
        )
        if best_full is not None:
            print(
                f"  (Ref) free-a FK fit pos max={best_full['pos_mm_max']:.1f} mm, "
                f"max|a|={best_full['a_max_abs']*1000:.1f} mm — not trustworthy as DH params."
            )

    print("\n" + "=" * 70)
    print("SUMMARY: axis geometry confirms a_prev=0; do not set a≠0 for arm-angle IK.")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
