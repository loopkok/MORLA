#!/usr/bin/env python3
"""Offline tests for geometric DH-free S/E/W arm-angle IK (``ik/geometric.py``).

Ground truth: Pinocchio FK via the production ``URDFNumericalIKSolver``.
The geometric solver must match URDF FK to float precision (no DH fit error)
and reproduce seed branches on FK->IK round-trips.

Usage:
  python3 -m astral_arm_teleop.test_geometric_ik
  ros2 run astral_arm_teleop test_geometric_ik
"""

from __future__ import annotations

import math
import sys
import time

import numpy as np

from astral_arm_teleop.ik.analytic import ContinuityParams
from astral_arm_teleop.ik.factory import (
    default_astral_urdf_path,
    make_ik_solver,
    make_single_arm_ik,
)
from astral_arm_teleop.ik.geometric import (
    GeometricIKSolver,
    _axis_angle_rot,
    _collect_solutions,
    _compute_sw,
    _elbow_point,
    psi_from_elbow_dir,
)
from astral_arm_teleop.ik.urdf_solver import URDFNumericalIKSolver

ARMS = ("L", "R")


def _elbow_fk(q: np.ndarray, g) -> np.ndarray:
    """Elbow center under joints 1-3 (q4 spins about E and cannot move it)."""
    T = np.eye(4)
    I3 = np.eye(3)
    for i in range(3):
        R = _axis_angle_rot(g.axes[i], float(q[i]))
        Ti = np.eye(4)
        Ti[:3, :3] = R
        Ti[:3, 3] = (I3 - R) @ g.points[i]
        T = T @ Ti
    return (T @ np.append(g.E0, 1.0))[:3]


def _wrap(x: np.ndarray) -> np.ndarray:
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _sample_q(rng, lower, upper, n, margin=0.05):
    """Random configs clear of generic SRS singularities.

    (shoulder PK2 double root at |q2|=pi/2, straight elbow q4=0, wrist q6=0 —
    no 7-DOF arm-angle solver is asserted exact at those.)
    """
    out = []
    lo = lower + margin
    hi = upper - margin
    while len(out) < n:
        q = rng.uniform(lo, hi)
        if abs(abs(q[1]) - 0.5 * np.pi) < 0.25:
            continue
        if q[3] > -0.30:
            continue
        if abs(q[5]) < 0.15:
            continue
        out.append(q)
    return out


class _GroundTruth:
    def __init__(self):
        self.solver = URDFNumericalIKSolver(urdf_path=default_astral_urdf_path())

    def fk(self, arm: str, q: np.ndarray) -> np.ndarray:
        return self.solver.fk_homogeneous(arm, q)


def _make(arm: str) -> GeometricIKSolver:
    return GeometricIKSolver("left" if arm == "L" else "right")


def test_fk_matches_urdf(gt: _GroundTruth) -> bool:
    print("\n  --- FK vs Pinocchio URDF (ground truth) ---")
    rng = np.random.default_rng(42)
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        max_pos = 0.0
        max_rot = 0.0
        for _ in range(100):
            q = rng.uniform(solver.lower_limits, solver.upper_limits)
            Tg = solver.fk(q)
            Tu = gt.fk(arm, q)
            max_pos = max(max_pos, float(np.linalg.norm(Tg[:3, 3] - Tu[:3, 3])))
            max_rot = max(max_rot, float(np.linalg.norm(Tg[:3, :3] - Tu[:3, :3])))
        ok = max_pos < 1e-8 and max_rot < 1e-8
        ok_all &= ok
        print(
            f"  arm {arm}: max pos err {max_pos * 1e9:.1f} nm, "
            f"max rot err {max_rot:.2e} -> {'PASS' if ok else 'FAIL'}"
        )
    return ok_all


def test_fk_ik_roundtrip() -> bool:
    print("\n  --- FK -> IK round-trip (both arms) ---")
    rng = np.random.default_rng(7)
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        seeds = _sample_q(rng, solver.lower_limits, solver.upper_limits, 8)
        fails = 0
        worst_pos_mm = 0.0
        worst_dq = 0.0
        for q_seed in seeds:
            solver.sync_state(q_seed)
            T = solver.fk(q_seed)
            sol = solver.solve(T)
            if sol is None:
                print(f"  arm {arm}: FAIL no solution for seed {np.round(q_seed, 2)}")
                fails += 1
                continue
            Terr = solver.fk(sol)
            pos_mm = float(np.linalg.norm(Terr[:3, 3] - T[:3, 3]) * 1000.0)
            rot_err = float(np.linalg.norm(Terr[:3, :3] - T[:3, :3]))
            dq = float(np.max(np.abs(_wrap(sol - q_seed))))
            worst_pos_mm = max(worst_pos_mm, pos_mm)
            worst_dq = max(worst_dq, dq)
            if pos_mm > 1.0 or rot_err > 0.02 or dq > 0.2:
                print(
                    f"  arm {arm}: FAIL seed {np.round(q_seed, 2)} "
                    f"pos={pos_mm:.3f}mm rot={rot_err:.4f} dq={dq:.3f}"
                )
                fails += 1
        ok = fails == 0
        ok_all &= ok
        print(
            f"  arm {arm}: {len(seeds)} seeds, worst pos {worst_pos_mm:.4f} mm, "
            f"worst |dq| {worst_dq:.4f} rad -> {'PASS' if ok else 'FAIL'}"
        )
    return ok_all


def test_solutions_within_limits() -> bool:
    print("\n  --- Solutions respect joint limits ---")
    rng = np.random.default_rng(11)
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        lo, hi = solver.lower_limits, solver.upper_limits
        seeds = _sample_q(rng, lo, hi, 5)
        ok = True
        for q_seed in seeds:
            solver.sync_state(q_seed)
            sol = solver.solve(solver.fk(q_seed))
            if sol is None:
                continue
            if not (np.all(sol >= lo - 1e-9) and np.all(sol <= hi + 1e-9)):
                print(f"  arm {arm}: FAIL out of limits {np.round(sol, 3)}")
                ok = False
        ok_all &= ok
        print(f"  arm {arm}: {'PASS' if ok else 'FAIL'}")
    return ok_all


def test_unreachable_returns_none() -> bool:
    print("\n  --- Unreachable target returns None ---")
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        solver.sync_state(np.zeros(7))
        T = np.eye(4)
        T[:3, 3] = np.array([1.5, 0.0, 0.0])  # reach ~0.48 m
        sol = solver.solve(T)
        ok = sol is None
        ok_all &= ok
        print(f"  arm {arm}: {'PASS' if ok else 'FAIL (returned a pose)'}")
    return ok_all


def test_smooth_trajectory() -> bool:
    print("\n  --- Smooth trajectory (continuity + timing) ---")
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        q0 = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00])
        q0 = np.clip(q0, solver.lower_limits + 0.02, solver.upper_limits - 0.02)
        solver.sync_state(q0)
        T0 = solver.fk(q0)
        center = T0[:3, 3].copy()
        base_rot = T0[:3, :3].copy()
        n = 60
        fails = 0
        times = []
        max_jump = 0.0
        q_prev = q0.copy()
        for step in range(n):
            ang = 2 * np.pi * step / n
            T = np.eye(4)
            T[:3, :3] = base_rot
            T[:3, 3] = center + 0.03 * np.array([np.cos(ang), np.sin(ang), 0.0])
            t0 = time.perf_counter()
            sol = solver.solve(T)
            times.append((time.perf_counter() - t0) * 1000)
            if sol is None:
                fails += 1
                continue
            max_jump = max(max_jump, float(np.max(np.abs(_wrap(sol - q_prev)))))
            q_prev = sol.copy()
        rate = (n - fails) / n * 100
        mean_t = float(np.mean(times[1:])) if len(times) > 1 else 0.0
        ok = rate >= 95.0 and max_jump < 0.3 and mean_t < 5.0
        ok_all &= ok
        print(
            f"  arm {arm}: rate={rate:.1f}% max_jump={max_jump:.3f} rad "
            f"mean_t={mean_t:.2f} ms -> {'PASS' if ok else 'FAIL'}"
        )
    return ok_all


def test_dual_arm_factory(gt: _GroundTruth) -> bool:
    print("\n  --- Dual-arm factory bridge ---")
    bridge = make_ik_solver("geometric")
    ok = True
    for arm in ARMS:
        Tg = bridge.fk(arm, np.zeros(7))
        Tu = gt.fk(arm, np.zeros(7))
        if float(np.linalg.norm(Tg - Tu)) > 1e-8:
            print(f"  arm {arm}: FAIL fk mismatch vs URDF")
            ok = False
    TL = bridge.fk("L", np.zeros(7))
    TR = bridge.fk("R", np.zeros(7))
    print(f"  L EE0={np.round(TL[:3, 3], 3).tolist()} R EE0={np.round(TR[:3, 3], 3).tolist()}")
    if not (TL[0, 3] > 0.05 and TR[0, 3] < -0.05):
        print("  FAIL: base_link arm separation wrong")
        ok = False
    # solve through the bridge on a bent seed
    for arm in ARMS:
        q0 = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0])
        lo, hi = bridge.joint_limits(arm)
        q0 = np.clip(q0, lo + 0.02, hi - 0.02)
        q14 = bridge.q_full
        q14[0:7 if arm == "L" else None] = 0.0  # placeholder, replaced below
        bridge.sync_state(np.concatenate([q0, np.zeros(7)]) if arm == "L" else np.concatenate([np.zeros(7), q0]))
        T = bridge.fk(arm, q0)
        sol = bridge.solve(arm, T)
        if sol is None or float(np.linalg.norm(bridge.fk(arm, sol)[:3, 3] - T[:3, 3])) > 1e-3:
            print(f"  arm {arm}: FAIL bridge solve")
            ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_single_arm_factory() -> bool:
    print("\n  --- Single-arm factory (teleop node API) ---")
    ok = True
    for side in ("left", "right"):
        s = make_single_arm_ik(side, "geometric")
        for attr in ("solve", "fk", "sync_state", "lower_limits", "upper_limits", "method_name"):
            if not hasattr(s, attr):
                print(f"  {side}: FAIL missing {attr}")
                ok = False
        if "geometric" not in str(s.method_name):
            print(f"  {side}: FAIL method_name={s.method_name}")
            ok = False
        if np.asarray(s.lower_limits).shape != (7,) or np.asarray(s.upper_limits).shape != (7,):
            print(f"  {side}: FAIL limits shape")
            ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_psi_ref_prior() -> bool:
    """Human elbow prior: psi_from_elbow_dir roundtrip + solve(psi_ref) tracking."""
    print("\n  --- Human elbow prior (psi_ref) ---")
    rng = np.random.default_rng(21)
    ok_all = True
    for arm in ARMS:
        solver = _make(arm)
        g = solver.geom
        ok = True
        seeds = _sample_q(rng, solver.lower_limits, solver.upper_limits, 6)

        # 1) algebraic roundtrip: psi -> E(psi) -> direction S->E -> psi
        worst_rt = 0.0
        for q_seed in seeds:
            T = solver.fk(q_seed)
            S, W, q4l = _compute_sw(T, g)
            if not q4l:
                continue
            for psi in np.linspace(-np.pi, np.pi, 25):
                E = _elbow_point(float(psi), S, W, g)
                if E is None:
                    continue
                got = psi_from_elbow_dir(S, W, E - S, g)
                err = abs(float(_wrap(np.array([got - psi]))[0]))
                worst_rt = max(worst_rt, err)
        if worst_rt > 1e-9:
            print(f"  arm {arm}: FAIL psi roundtrip err {worst_rt:.2e}")
            ok = False

        # 2) end-to-end: seed elbow direction -> psi_ref -> solve -> same arm angle
        worst_dpsi = 0.0
        for q_seed in seeds:
            T = solver.fk(q_seed)
            S, W, _ = _compute_sw(T, g)
            E_seed = _elbow_fk(q_seed, g)
            psi_ref = solver.arm_angle_from_elbow_dir(T, E_seed - S)
            if psi_ref is None:
                print(f"  arm {arm}: FAIL arm_angle_from_elbow_dir returned None")
                ok = False
                continue
            # cold-start from a zero seed: continuity alone would not pick
            # the seed's arm-angle branch — only psi_ref pulls it there.
            solver.sync_state(np.zeros(7), reset_branch=True)
            sol = solver.solve(T, psi_ref=psi_ref)
            if sol is None:
                print(f"  arm {arm}: FAIL no solution with psi_ref")
                ok = False
                continue
            pos_mm = float(np.linalg.norm(solver.fk(sol)[:3, 3] - T[:3, 3]) * 1000.0)
            psi_sol = psi_from_elbow_dir(S, W, _elbow_fk(sol, g) - S, g)
            dpsi = abs(float(_wrap(np.array([psi_sol - psi_ref]))[0]))
            worst_dpsi = max(worst_dpsi, dpsi)
            if dpsi > 0.02 or pos_mm > 1.0:
                print(
                    f"  arm {arm}: FAIL dpsi={np.degrees(dpsi):.2f} deg "
                    f"pos={pos_mm:.3f} mm"
                )
                ok = False

        # 3) psi_ref in a joint-limit-infeasible region: must still solve
        #    (global fallback + continuity), pose stays exact.
        q_seed = seeds[0]
        T = solver.fk(q_seed)
        solver.sync_state(q_seed, reset_branch=True)
        sol = solver.solve(T, psi_ref=0.0 if abs(0.0 - float(psi_ref)) > 0.5 else 2.0)
        # pick a psi_ref far from the seed's own arm angle: it may or may not
        # be feasible, but the solver must never return None or a bad pose.
        if sol is None:
            print(f"  arm {arm}: FAIL infeasible-side psi_ref returned None")
            ok = False
        else:
            pos_mm = float(np.linalg.norm(solver.fk(sol)[:3, 3] - T[:3, 3]) * 1000.0)
            if pos_mm > 1.0:
                print(f"  arm {arm}: FAIL fallback pose err {pos_mm:.3f} mm")
                ok = False

        # 4) psi_ref=None unchanged: continuity keeps the seed branch.
        solver.sync_state(q_seed)
        sol = solver.solve(T)
        dq = float(np.max(np.abs(_wrap(sol - q_seed)))) if sol is not None else 9.9
        if dq > 0.2:
            print(f"  arm {arm}: FAIL no-prior continuity dq={dq:.3f}")
            ok = False

        # 5) straightness gate: with the human arm nearly straight, the elbow
        #    direction is ~parallel to S-W and psi is unobservable — IOBT
        #    bias would otherwise swivel the robot elbow to a garbage angle.
        q_bent = max(seeds, key=lambda q: abs(float(q[3])))
        T = solver.fk(q_bent)
        S, W, _ = _compute_sw(T, g)
        u = (W - S) / float(np.linalg.norm(W - S))
        d_bent = _elbow_fk(q_bent, g) - S
        e_perp = d_bent - float(d_bent @ u) * u
        e_perp = e_perp / float(np.linalg.norm(e_perp))
        for s in (0.02, 0.05, 0.10):  # nearly-parallel dirs: gate must close
            d_str = u * math.sqrt(1.0 - s * s) + e_perp * s
            got = solver.arm_angle_from_elbow_dir(T, d_str, min_sin=0.15)
            if got is not None:
                print(f"  arm {arm}: FAIL gate open at sin={s}")
                ok = False
        got = solver.arm_angle_from_elbow_dir(T, d_bent, min_sin=0.15)
        if got is None:
            print(f"  arm {arm}: FAIL gate closed on a bent arm")
            ok = False

        ok_all &= ok
        print(
            f"  arm {arm}: roundtrip {worst_rt:.1e} rad, tracked dpsi "
            f"{np.degrees(worst_dpsi):.3f} deg -> {'PASS' if ok else 'FAIL'}"
        )
    return ok_all


def main() -> None:
    print("Astral Teleop — Geometric (DH-free) IK Test Suite")
    gt = _GroundTruth()
    results = {
        "fk_matches_urdf": test_fk_matches_urdf(gt),
        "fk_ik_roundtrip": test_fk_ik_roundtrip(),
        "within_limits": test_solutions_within_limits(),
        "unreachable_none": test_unreachable_returns_none(),
        "smooth_trajectory": test_smooth_trajectory(),
        "dual_arm_factory": test_dual_arm_factory(gt),
        "single_arm_factory": test_single_arm_factory(),
        "psi_ref_prior": test_psi_ref_prior(),
        "full_extension": test_full_extension(),
        "roll_escape_hysteresis": test_roll_escape_hysteresis(),
        "hard_elbow_follow": test_hard_elbow_follow(),
        "elbow_direction_reanchor_keeps_config": test_elbow_direction_reanchor_keeps_config(),
    }
    print("\n  SUMMARY:")
    for k, v in results.items():
        print(f"    {k}: {'PASS' if v else 'FAIL'}")
    ok = all(results.values())
    print(f"FINAL: {'ALL PASSED' if ok else 'SOME FAILED'}")
    if not ok:
        sys.exit(1)


def test_full_extension() -> bool:
    """Elbow-straight singularity: wrist target crossing the reach boundary.

    Without a reach clamp the q4 elbow limit (|q4| <= 2.88 rad < pi) rejects
    every branch in an outer band of the workspace, solve() returns None, the
    node holds the last command, and the arm visibly "catches" when IK
    recovers. With clamp_wrist_reach every step must solve smoothly.
    """
    print("\n  --- Full-extension reach boundary ---")
    solver = _make("R")
    g = solver.geom
    S = np.asarray(g.S, dtype=float)
    ok = True

    # Find a seed whose S->W ray stays feasible (fixed flange orientation)
    # essentially to the reach boundary, so the elbow is what runs out.
    rng = np.random.default_rng(3)
    r_max = g.l_se + g.l_ew
    best = None
    for cand in _sample_q(rng, solver.lower_limits, solver.upper_limits, 60):
        T_c = solver.fk(cand)
        r_c = float(np.linalg.norm(T_c[:3, 3] - S))
        if not (0.28 < r_c < 0.36):
            continue
        ray_c = (T_c[:3, 3] - S) / r_c
        solver.sync_state(cand, reset_branch=True)
        last = r_c
        for rr in np.arange(r_c, r_max + 0.02, 0.005):
            Tt = T_c.copy()
            Tt[:3, 3] = S + ray_c * rr
            q = solver.solve(Tt)
            if q is None:
                break
            solver.sync_state(q, reset_branch=False)
            last = rr
        if best is None or last > best[0]:
            best = (last, cand, T_c, ray_c, r_c)
    if best is None or best[0] < r_max - 0.005:
        print(f"  FAIL no ray feasible to boundary (best={best and best[0]})")
        return False
    _, q_seed, T_seed, ray, r_seed = best

    def _target(r: float) -> np.ndarray:
        T = T_seed.copy()
        T[:3, 3] = S + ray * r
        return T

    # 1) clamp_wrist_reach: beyond -> on the sphere, direction preserved.
    p_far = S + np.array([0.0, 0.0, -(g.l_se + g.l_ew + 0.05)])
    p_c, clipped = solver.clamp_wrist_reach(p_far, margin=0.01)
    if not clipped:
        print("  FAIL clamp did not report clip beyond reach")
        ok = False
    elif abs(float(np.linalg.norm(p_c - S)) - (g.l_se + g.l_ew - 0.01)) > 1e-9:
        print("  FAIL clamp radius wrong")
        ok = False
    elif not np.allclose(
        (p_far - S) / np.linalg.norm(p_far - S),
        (p_c - S) / np.linalg.norm(p_c - S),
        atol=1e-12,
    ):
        print("  FAIL clamp changed direction")
        ok = False
    p_in = S + np.array([0.30, 0.0, -0.20])
    p_c2, clipped2 = solver.clamp_wrist_reach(p_in, margin=0.01)
    if clipped2 or not np.allclose(p_c2, p_in):
        print("  FAIL clamp touched an in-reach target")
        ok = False

    # 2) raw sweep must actually fail somewhere (test exercises the band).
    radii = np.arange(r_seed, r_max + 0.03, 0.0005)
    solver.sync_state(q_seed, reset_branch=True)
    raw_failed = 0
    for r in radii:
        q = solver.solve(_target(float(r)))
        if q is None:
            raw_failed += 1
        else:
            solver.sync_state(q, reset_branch=False)
    if raw_failed == 0:
        print("  FAIL raw sweep never left the feasible region")
        ok = False

    # 3) clamped sweep: every step solvable, q4 in limits, motion smooth.
    # Uses the same 5 mm discretization as the seed-selection walk, which
    # proved this exact ray feasible to the boundary; finer steps can hit
    # narrow wrist-orientation pockets where a branch legitimately escapes
    # (that is the escape hysteresis working, not a clamp failure).
    radii = np.arange(r_seed, r_max + 0.03, 0.005)
    solver.sync_state(q_seed, reset_branch=True)
    q_prev = None
    max_step = 0.0
    n_fail = 0
    jump_log = []
    for r in radii:
        T = _target(float(r))
        T[:3, 3], _ = solver.clamp_wrist_reach(T[:3, 3], 0.01)
        q = solver.solve(T)
        if q is None:
            n_fail += 1
            continue
        if not (g.lower[3] - 1e-6 <= float(q[3]) <= g.upper[3] + 1e-6):
            print(f"  FAIL q4 limit at r={float(r):.4f}")
            ok = False
            break
        if q_prev is not None:
            step = float(np.max(np.abs(_wrap(q - q_prev))))
            if step > 0.15:
                jump_log.append((float(r), step, bool(solver._state.esc_active)))
            max_step = max(max_step, step)
        q_prev = q
    if n_fail:
        print(f"  FAIL clamped sweep had {n_fail} unsolved steps")
        ok = False
    # Jumps > 0.15 rad are allowed ONLY when they are debounced branch
    # escapes (esc_active): near full extension the fixed flange orientation
    # can become infeasible at the current arm angle, and the solver swings
    # to a feasible one — by design. The teleop velocity limiter smooths
    # that into motion, and psi_escape/WARN make it observable. A jump
    # WITHOUT an active escape is a genuine discontinuity.
    for (r_j, jump, esc) in jump_log:
        if jump > 0.15 and not esc:
            print(f"  FAIL per-frame jump {jump:.3f} rad at r={r_j:.4f} (no escape)")
            ok = False
    if jump_log:
        big = [j for _, j, _ in jump_log if j > 0.15]
        if big:
            print(
                f"  note: {len(big)} debounced escape(s), max "
                f"{max(big):.2f} rad (velocity-limited downstream)"
            )

    print(
        f"  raw fails={raw_failed}/{len(radii)}, clamped max step "
        f"{np.degrees(max_step):.2f} deg -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def _find_roll_case(solver):
    """(seed T, rolled T, home psi): rolled target infeasible near home psi
    but feasible globally — the real-machine shoulder-twitch trigger."""
    g = solver.geom
    rng = np.random.default_rng(11)
    for cand in _sample_q(rng, solver.lower_limits, solver.upper_limits, 80):
        if abs(float(cand[3])) < 0.8:  # want a well-bent elbow
            continue
        T0 = solver.fk(cand)
        solver.sync_state(cand, reset_branch=True)
        q0 = solver.solve(T0)
        if q0 is None or solver._state.theta0_prev is None:
            continue
        psi_home = float(solver._state.theta0_prev)
        for phi in (1.2, 1.6, 2.0, 2.4, 2.8, -1.2, -1.6, -2.0, -2.4, -2.8):
            T1 = T0.copy()
            T1[:3, :3] = T0[:3, :3] @ _axis_angle_rot(np.array([0.0, 0.0, 1.0]), phi)
            S, W, q4l = _compute_sw(T1, g)
            if not q4l:
                continue
            local = _collect_solutions(
                T1, g, S, W, q4l, np.linspace(psi_home - 0.15, psi_home + 0.15, 5)
            )
            if local:
                continue
            glob = _collect_solutions(
                T1, g, S, W, q4l, np.arange(-math.pi, math.pi, 0.03)
            )
            if not glob:
                continue
            # Require the nearest global escape to be FAR from home — a nearby
            # escape is invisible; the real shoulder twitch is a large swing.
            d_esc = min(
                abs(float(_wrap(np.array([float(c[7]) - psi_home]))[0]))
                for c in glob
            )
            if d_esc > 0.5:
                return T0, T1, psi_home
    return None


def _theta0_jumps(traj, thresh=0.3):
    return sum(
        1
        for a, b in zip(traj, traj[1:])
        if a is not None and b is not None
        and abs(float(_wrap(np.array([b - a]))[0])) > thresh
    )


def test_roll_escape_hysteresis() -> bool:
    """Big end-effector roll: local window goes empty at the wrist-limit
    boundary. Without hysteresis the arm angle escapes and snaps back
    frame-to-frame (shoulder twitch); with it, transitions are debounced."""
    print("\n  --- Roll escape hysteresis (shoulder twitch) ---")
    solver = _make("R")
    found = _find_roll_case(solver)
    if found is None:
        print("  FAIL no roll case found (locally infeasible, globally feasible)")
        return False
    T0, T1, psi_home = found
    ok = True

    def run(cont, frames):
        # psi_ref pinned at home: reproduces the real-machine setup
        # (use_human_elbow=true) where the window cannot drift with theta0_prev.
        solver.continuity = cont
        solver.sync_state(np.zeros(7), reset_branch=True)
        q = solver.solve(T0, psi_ref=psi_home)  # establish home branch
        if q is None:
            return None
        traj = []
        for T in frames:
            q = solver.solve(T, psi_ref=psi_home)
            traj.append(None if q is None else float(solver._state.theta0_prev))
        return traj

    # Sustained roll then sustained normal: new params -> exactly one escape
    # and one return; old params (1/1) may also pass here but must not regress.
    seq = [T1] * 30 + [T0] * 30
    traj_new = run(ContinuityParams(), seq)
    if traj_new is None:
        print("  FAIL could not establish home pose")
        return False
    jumps_new = _theta0_jumps(traj_new)
    holds = sum(1 for t in traj_new[:4] if t is None)
    if holds == 0:
        print("  FAIL debounce never held (expected None frames before escape)")
        ok = False
    if jumps_new > 2:
        print(f"  FAIL sustained case jumps={jumps_new} (>2)")
        ok = False
    final = traj_new[-1]
    if final is None or abs(float(_wrap(np.array([final - psi_home]))[0])) > 0.2:
        print(f"  FAIL did not return home (final={final}, home={psi_home:.3f})")
        ok = False

    # Boundary flicker: alternating feasible/infeasible every frame. Old
    # behavior oscillates (twitch); new behavior holds at home, zero jumps.
    flicker = [T1 if i % 2 else T0 for i in range(40)]
    traj_old = run(ContinuityParams(escape_after_frames=1, return_after_frames=1), flicker)
    traj_new2 = run(ContinuityParams(), flicker)
    jumps_old = _theta0_jumps(traj_old)
    jumps_new2 = _theta0_jumps(traj_new2)
    if jumps_new2 > 2:
        print(f"  FAIL flicker case jumps={jumps_new2} (>2)")
        ok = False
    if jumps_old < 4:
        # Without oscillation in the old behavior the test reproduces nothing.
        print(f"  FAIL old behavior did not oscillate (jumps={jumps_old})")
        ok = False

    print(
        f"  sustained jumps={jumps_new}, flicker jumps old={jumps_old} "
        f"new={jumps_new2}, returned home -> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def test_hard_elbow_follow() -> bool:
    """Hard human-elbow mode: ``solve_hard(T, psi)`` resolves the arm exactly at
    the requested arm angle (no soft blend / grid / hysteresis) so the robot
    elbow strictly follows the human's. The recording premise for astral_pim_ik:
    the q executed in hard mode always carries ``psi_from_config(q) == psi_human``
    (equivalently: the solved elbow direction equals the fed human direction).
    """
    print("\n  --- Hard human-elbow follow (solve_hard) ---")
    ok = True
    for arm in ARMS:
        s = GeometricIKSolver("left" if arm == "L" else "right")
        g = s.geom
        rng = np.random.default_rng(17 if arm == "L" else 23)
        worst_dir = 0.0
        worst_pos = 0.0
        n_solved = 0
        for q_seed in _sample_q(rng, s.lower_limits, s.upper_limits, 12):
            T = s.fk(q_seed)
            E = _elbow_fk(q_seed, g)
            d = E - g.S
            n = float(np.linalg.norm(d))
            if n < 1e-9:
                continue
            d = d / n
            psi = s.arm_angle_from_elbow_dir(T, d, min_sin=0.05)
            if psi is None:
                continue
            q_sol = s.solve_hard(T, psi, q_init=q_seed)
            if q_sol is None:
                continue
            n_solved += 1
            # Recording premise: elbow direction of the solved q == human dir.
            E_sol = _elbow_fk(q_sol, g)
            d_sol = (E_sol - g.S)
            d_sol = d_sol / np.linalg.norm(d_sol)
            worst_dir = max(
                worst_dir,
                float(np.degrees(np.arccos(np.clip(np.dot(d, d_sol), -1.0, 1.0)))),
            )
            worst_pos = max(
                worst_pos,
                float(np.linalg.norm(s.fk(q_sol)[:3, 3] - T[:3, 3]) * 1000.0),
            )
        # Determinism + unreachable -> None (the node's fallback trigger).
        Tfar = np.eye(4)
        Tfar[:3, 3] = [1.5, 0.0, 0.0]  # far beyond reach ~0.48 m
        if s.solve_hard(Tfar, 0.3, q_init=np.zeros(7)) is not None:
            ok = False
            print(f"  arm {arm}: FAIL solve_hard solved an unreachable pose")
        q0 = _sample_q(rng, s.lower_limits, s.upper_limits, 1)[0]
        T0 = s.fk(q0)
        p0 = s.arm_angle_from_elbow_dir(T0, _elbow_fk(q0, g) - g.S, min_sin=0.05)
        if p0 is not None and not np.allclose(
            s.solve_hard(T0, p0), s.solve_hard(T0, p0), atol=1e-12
        ):
            ok = False
            print(f"  arm {arm}: FAIL solve_hard not deterministic")
        ok_arm = worst_dir < 0.2 and worst_pos < 1.0 and n_solved > 0
        ok &= ok_arm
        print(
            f"  arm {arm}: solved={n_solved}, worst elbow-dir dev {worst_dir:.4f} deg, "
            f"worst pos {worst_pos:.4f} mm -> {'PASS' if ok_arm else 'FAIL'}"
        )
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_elbow_direction_reanchor_keeps_config() -> bool:
    """``elbow_direction(q)`` reproduces the arm angle of ``q``, so re-seeding
    the human-elbow reference from it (HITL ``~/reanchor`` / ``_start_teleop``)
    makes the first ``solve_hard`` keep the current configuration — no elbow
    snap at takeover (regression: reanchor 后 0.26-0.32 rad 肘部重构)."""
    print("\n  --- elbow_direction re-anchor keeps current config ---")
    ok = True
    for arm in ARMS:
        s = GeometricIKSolver("left" if arm == "L" else "right")
        g = s.geom
        rng = np.random.default_rng(41 if arm == "L" else 43)
        worst_keep = 0.0
        worst_dir = 0.0
        checked = 0
        for q in _sample_q(rng, s.lower_limits, s.upper_limits, 10):
            T = s.fk(q)
            d = s.elbow_direction(q)
            # (a) direction == actual upper-arm (shoulder -> elbow) direction
            E = _elbow_fk(q, g)
            d_true = E - g.S
            d_true = d_true / np.linalg.norm(d_true)
            worst_dir = max(
                worst_dir,
                float(np.degrees(np.arccos(np.clip(np.dot(d, d_true), -1.0, 1.0)))),
            )
            # (b) solve_hard at the psi from this direction keeps q (zero snap)
            psi = s.arm_angle_from_elbow_dir(T, d, min_sin=0.0)
            if psi is None:
                ok = False
                print(f"  arm {arm}: arm_angle_from_elbow_dir None for q={q}")
                continue
            q_sol = s.solve_hard(T, psi, q_init=q)
            if q_sol is None:
                continue
            worst_keep = max(worst_keep, float(np.max(np.abs(q_sol - q))))
            checked += 1
        ok_arm = checked > 0 and worst_dir < 0.2 and worst_keep < 0.02
        ok &= ok_arm
        print(
            f"  arm {arm}: checked={checked}, worst dir err {worst_dir:.4f} deg, "
            f"worst config move {worst_keep:.4f} rad -> {'PASS' if ok_arm else 'FAIL'}"
        )
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


if __name__ == "__main__":
    main()
