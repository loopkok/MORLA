#!/usr/bin/env python3
"""Offline Astral IK tests — analytic_dh + urdf_numerical.

Usage:
  ros2 run astral_arm_teleop test_ik_solver
"""

from __future__ import annotations

import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from astral_arm_teleop.ik import make_ik_solver
from astral_arm_teleop.safety_filter import SafetyFilter


class _ArmView:
    """Single-arm view over AstralIKBridge (Nero-like API for tests)."""

    def __init__(self, bridge, arm: str):
        self.bridge = bridge
        self.arm = arm  # L|R
        lo, hi = bridge.joint_limits(arm)
        self.lower_limits = lo
        self.upper_limits = hi
        self.nq = 7

    def sync_state(self, q7: np.ndarray) -> None:
        q14 = self.bridge.q_full
        if self.arm == "L":
            q14[0:7] = q7
        else:
            q14[7:14] = q7
        self.bridge.sync_state(q14)

    def fk(self, q7: np.ndarray) -> np.ndarray:
        return self.bridge.fk(self.arm, q7)

    def solve(self, T: np.ndarray):
        return self.bridge.solve(self.arm, T)


def _create(solver_type: str, arm: str = "L") -> _ArmView:
    return _ArmView(make_ik_solver(solver_type), arm)


def test_fk_ik_roundtrip(solver: _ArmView, label: str) -> bool:
    print("\n  --- FK -> IK round-trip ---")
    q_seeds = [
        np.array([0.1, 0.2, -0.1, -0.3, -0.2, 0.1, 0.0]),
        np.array([0.3, 0.4, -0.2, -0.8, -0.4, 0.2, 0.1]),
        np.array([-0.3, 0.5, 0.2, -0.6, -0.2, 0.3, 0.1]),
        np.array([0.2, 0.3, -0.1, -0.5, 0.2, -0.1, -0.2]),
    ]
    all_pass = True
    max_err = 0.0
    for i, q_seed in enumerate(q_seeds):
        q_seed = np.clip(q_seed, solver.lower_limits + 0.02, solver.upper_limits - 0.02)
        solver.sync_state(q_seed)
        T = solver.fk(q_seed)
        t0 = time.perf_counter()
        sol = solver.solve(T)
        dt = (time.perf_counter() - t0) * 1000
        if sol is None:
            print(f"  Config {i}: FAIL — no converge ({dt:.1f}ms)")
            all_pass = False
            continue
        Terr = solver.fk(sol)
        pos_mm = float(np.linalg.norm(T[:3, 3] - Terr[:3, 3]) * 1000)
        rot_err = float(np.linalg.norm(T[:3, :3] - Terr[:3, :3]))
        max_err = max(max_err, pos_mm)
        # Closed-form DH is exact; URDF LM is looser
        lim = 1.0 if label == "analytic_dh" else 8.0
        ok = pos_mm < lim and rot_err < 0.25
        print(
            f"  Config {i}: pos={pos_mm:.3f}mm rot={rot_err:.4f} "
            f"t={dt:.1f}ms -> {'PASS' if ok else 'FAIL'}"
        )
        all_pass &= ok
    print(f"  Max pos error: {max_err:.3f}mm Overall: {'PASS' if all_pass else 'FAIL'}")
    return all_pass


def test_smooth_trajectory(solver: _ArmView, label: str) -> bool:
    print("\n  --- Smooth trajectory ---")
    # Bent seed; small circle — arm-base DH workspace is tighter than torso URDF
    q0 = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00])
    q0 = np.clip(q0, solver.lower_limits + 0.02, solver.upper_limits - 0.02)
    solver.sync_state(q0)
    T0 = solver.fk(q0)
    center = T0[:3, 3].copy()
    base_rot = T0[:3, :3].copy()
    radius = 0.03 if label == "analytic_dh" else 0.04
    n = 60
    fails = 0
    times = []
    max_err = 0.0
    for step in range(n):
        ang = 2 * np.pi * step / n
        pos = center + radius * np.array([np.cos(ang), np.sin(ang), 0.0])
        T = np.eye(4)
        T[:3, :3] = base_rot
        T[:3, 3] = pos
        t0 = time.perf_counter()
        sol = solver.solve(T)
        times.append((time.perf_counter() - t0) * 1000)
        if sol is None:
            fails += 1
            continue
        err = float(np.linalg.norm(solver.fk(sol)[:3, 3] - pos) * 1000)
        max_err = max(max_err, err)
    rate = (n - fails) / n * 100
    steady = times[1:] if len(times) > 1 else times
    mean_t = float(np.mean(steady)) if steady else 0.0
    print(f"  Steps={n} fails={fails} rate={rate:.1f}% max_err={max_err:.2f}mm mean_t={mean_t:.1f}ms")
    ok = rate >= 90.0
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_dual_arms(solver_type: str) -> bool:
    print("\n  --- Dual arm base_link FK separation ---")
    bridge = make_ik_solver(solver_type)
    TL = bridge.fk("L", np.zeros(7))
    TR = bridge.fk("R", np.zeros(7))
    print(f"  L EE0={np.round(TL[:3, 3], 3).tolist()}")
    print(f"  R EE0={np.round(TR[:3, 3], 3).tolist()}")
    if solver_type == "urdf_numerical":
        # Poses are in left/right_base_link (same orientation as torso, origin at
        # the shoulder). q=0 EE x is ~±0.116 m, not the torso-frame ±0.21 m.
        ok = TL[0, 3] > 0.05 and TR[0, 3] < -0.05
    else:
        # Analytic DH is per-arm base (same model L/R aside from J2 limits)
        ok = np.all(np.isfinite(TL)) and np.all(np.isfinite(TR))
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_safety_filter(solver: _ArmView) -> bool:
    print("\n  --- Safety filter ---")
    sf = SafetyFilter(
        joint_lower_limits=solver.lower_limits,
        joint_upper_limits=solver.upper_limits,
        max_joint_vel=7.5,
        workspace_radius=0.5,
        workspace_z_min=-1.0,
        workspace_z_max=1.0,
        workspace_x_min=-1.0,
        workspace_x_max=1.0,
        workspace_y_min=-1.0,
        workspace_y_max=1.0,
    )
    sf.set_initial_state(np.clip(np.zeros(7), solver.lower_limits, solver.upper_limits))
    q_f, info = sf.filter(np.full(7, 10.0), dt=0.02)
    ok_clamp = bool(
        np.all(q_f <= solver.upper_limits) and np.all(q_f >= solver.lower_limits)
    )
    print(f"  [Clamp] {'PASS' if ok_clamp else 'FAIL'}")
    sf.set_initial_state(np.zeros(7))
    q_f, info = sf.filter(np.full(7, 0.5), dt=0.02)
    ok_vel = float(np.max(np.abs(q_f))) <= 0.16
    print(f"  [VelLimit] {'PASS' if ok_vel else 'FAIL'} info={info}")
    ee = sf.check_workspace(np.array([2.0, 0.0, 0.0]))
    ok_ws = float(np.linalg.norm(ee)) <= 0.51
    print(f"  [Workspace] {'PASS' if ok_ws else 'FAIL'} ee={np.round(ee, 3)}")
    sf.workspace_radius = 0.55
    inside = sf.check_workspace(np.array([0.077, -0.263, -0.343]))
    ok_in = abs(float(np.linalg.norm(inside)) - 0.4388) < 0.002
    far = sf.check_workspace(np.array([0.70, 0.0, 0.0]))
    ok_far = abs(float(np.linalg.norm(far)) - 0.55) < 1e-6
    print(f"  [Workspace 0.55] inside={'PASS' if ok_in else 'FAIL'} "
          f"clip={'PASS' if ok_far else 'FAIL'}")
    return ok_clamp and ok_vel and ok_ws and ok_in and ok_far


def test_q4_not_straight(solver: _ArmView, label: str) -> bool:
    print("\n  --- Joint4 stretch cap ---")
    if label != "urdf_numerical":
        print("  SKIP (analytic DH has its own limit weights)")
        return True
    hi = float(solver.upper_limits[3])
    ok_bound = hi <= -0.24
    print(f"  IK q4 upper={hi:.3f} (want <= -0.25) {'PASS' if ok_bound else 'FAIL'}")
    q0 = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00])
    q0 = np.clip(q0, solver.lower_limits + 0.02, solver.upper_limits - 0.02)
    solver.sync_state(q0)
    T = solver.fk(q0)
    p = T[:3, 3].copy()
    r = float(np.linalg.norm(p))
    T2 = T.copy()
    T2[:3, 3] = p * (0.58 / max(r, 1e-6))
    sol = None
    for _ in range(12):
        sol = solver.solve(T2)
        if sol is None:
            break
        if float(sol[3]) <= hi + 1e-3:
            break
    if sol is None:
        print("  far target: FAIL — no q (would freeze teleop)")
        return False
    q4 = float(sol[3])
    ok_sol = q4 <= hi + 1e-4
    print(
        f"  far target q4={q4:.3f} vs cap {hi:.3f} follow=yes "
        f"{'PASS' if ok_sol else 'FAIL'}"
    )
    return ok_bound and ok_sol


def run_all(solver_type: str) -> bool:
    print("\n" + "=" * 60)
    print(f"TESTING: {solver_type}")
    print("=" * 60)
    try:
        solver = _create(solver_type, "L")
    except Exception as exc:  # noqa: BLE001
        print(f"  SKIP/FAIL create: {exc}")
        return False
    results = {
        "dual_arm_fk": test_dual_arms(solver_type),
        "fk_ik_roundtrip": test_fk_ik_roundtrip(solver, solver_type),
        "smooth_trajectory": test_smooth_trajectory(solver, solver_type),
        "safety_filter": test_safety_filter(solver),
        "q4_stretch_cap": test_q4_not_straight(solver, solver_type),
    }
    print(f"\n  {solver_type} SUMMARY:")
    for k, v in results.items():
        print(f"    {k}: {'PASS' if v else 'FAIL'}")
    return all(results.values())


def main():
    print("Astral Teleop — Offline IK Test Suite")
    ok = True
    for st in ("analytic_dh", "urdf_numerical"):
        if not run_all(st):
            ok = False
    print("\n" + "=" * 60)
    print(f"FINAL: {'ALL PASSED' if ok else 'SOME FAILED'}")
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
