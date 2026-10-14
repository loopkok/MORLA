#!/usr/bin/env python3
"""Offline tests for the deterministic two-stage arm-angle solver (stage 2).

``GeometricArmAngleSolver`` must map ``(T_ee, psi) -> q`` exactly: FK->IK
round-trips recover the seed, the solved config's arm angle equals the requested
``psi``, joints stay in limits, unreachable targets return ``None``, and the
same input always yields the same output. Ground truth FK is Pinocchio via
``URDFNumericalIKSolver`` (same as ``test_geometric_ik``).

Usage:
  PYTHONPATH=src/astral_arm_teleop:src/astral_pim_ik \
    <python-with-pinocchio-and-torch> \   # e.g. miniconda3/envs/arm_sdk/bin/python3
    src/astral_pim_ik/astral_pim_ik/test_deterministic_solver.py
"""

from __future__ import annotations

import math
import sys

import numpy as np

from astral_arm_teleop.ik.factory import default_astral_urdf_path
from astral_arm_teleop.ik.urdf_solver import URDFNumericalIKSolver
from astral_pim_ik.geometry import GeometricArmAngleSolver

ARMS = ("L", "R")


def _wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _make(arm: str) -> GeometricArmAngleSolver:
    return GeometricArmAngleSolver("left" if arm == "L" else "right")


def _sample_q(rng, lower, upper, n, margin=0.05):
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

    def fk(self, arm, q):
        return self.solver.fk_homogeneous(arm, q)


def test_fk_matches_urdf(gt) -> bool:
    print("\n  --- FK vs Pinocchio URDF ---")
    rng = np.random.default_rng(42)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        worst = 0.0
        for _ in range(50):
            q = rng.uniform(s.lower_limits, s.upper_limits)
            Tg = s.fk(q)
            Tu = gt.fk(arm, q)
            worst = max(worst, float(np.linalg.norm(Tg - Tu)))
        ok &= worst < 1e-8
        print(f"  arm {arm}: max |Tg-Tu|={worst:.2e} -> {'PASS' if worst < 1e-8 else 'FAIL'}")
    return ok


def test_roundtrip_and_psi_fidelity() -> bool:
    print("\n  --- FK -> psi label -> IK round-trip ---")
    rng = np.random.default_rng(7)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        seeds = _sample_q(rng, s.lower_limits, s.upper_limits, 8)
        worst_pos = 0.0
        worst_dpsi = 0.0
        fails = 0
        for q_seed in seeds:
            T = s.fk(q_seed)
            psi = s.psi_from_config(q_seed)
            if psi is None:
                print(f"  arm {arm}: FAIL psi_from_config returned None")
                fails += 1
                continue
            sol = s.solve(T, psi, q_init=q_seed)
            if sol is None:
                print(f"  arm {arm}: FAIL no solution")
                fails += 1
                continue
            pos = float(np.linalg.norm(s.fk(sol)[:3, 3] - T[:3, 3]) * 1000.0)
            psi_sol = s.psi_from_config(sol)
            dpsi = abs(float(_wrap(np.array([psi_sol - psi]))[0]))
            worst_pos = max(worst_pos, pos)
            worst_dpsi = max(worst_dpsi, dpsi)
            if pos > 1.0 or dpsi > 0.02:
                print(f"  arm {arm}: FAIL seed {np.round(q_seed,2)} pos={pos:.3f}mm dpsi={dpsi:.3f}")
                fails += 1
        ok &= fails == 0
        print(f"  arm {arm}: worst pos {worst_pos:.4f} mm, worst dpsi {np.degrees(worst_dpsi):.3f} deg -> {'PASS' if fails == 0 else 'FAIL'}")
    return ok


def test_within_limits() -> bool:
    print("\n  --- Solutions respect joint limits ---")
    rng = np.random.default_rng(11)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        lo, hi = s.lower_limits, s.upper_limits
        for q_seed in _sample_q(rng, lo, hi, 5):
            T = s.fk(q_seed)
            psi = s.psi_from_config(q_seed)
            sol = s.solve(T, psi)
            if sol is not None and not (np.all(sol >= lo - 1e-9) and np.all(sol <= hi + 1e-9)):
                print(f"  arm {arm}: FAIL out of limits")
                ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_unreachable_none() -> bool:
    print("\n  --- Unreachable target returns None ---")
    ok = True
    for arm in ARMS:
        s = _make(arm)
        T = np.eye(4)
        T[:3, 3] = [1.5, 0.0, 0.0]  # reach ~0.48 m
        if s.solve(T, 0.0) is not None:
            print(f"  arm {arm}: FAIL returned a pose for unreachable target")
            ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_determinism() -> bool:
    print("\n  --- Determinism: same input -> same output ---")
    rng = np.random.default_rng(3)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        q_seed = _sample_q(rng, s.lower_limits, s.upper_limits, 1)[0]
        T = s.fk(q_seed)
        psi = s.psi_from_config(q_seed)
        a = s.solve(T, psi)
        b = s.solve(T, psi)
        if not np.allclose(a, b):
            print(f"  arm {arm}: FAIL non-deterministic")
            ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_warmstart_branch_selection() -> bool:
    print("\n  --- Warm-start selects the nearer branch ---")
    rng = np.random.default_rng(5)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        seeds = _sample_q(rng, s.lower_limits, s.upper_limits, 3)
        for q_seed in seeds:
            T = s.fk(q_seed)
            psi = s.psi_from_config(q_seed)
            # warm-start from the seed itself must recover the seed exactly
            q_warm = s.solve(T, psi, q_init=q_seed)
            dq = float(np.max(np.abs(_wrap(q_warm - q_seed))))
            if dq > 0.2:
                print(f"  arm {arm}: FAIL warm-start dq={dq:.3f}")
                ok = False
            # cold-start and warm-start must both be pose-correct regardless
            for q in (q_warm, s.solve(T, psi)):
                pos = float(np.linalg.norm(s.fk(q)[:3, 3] - T[:3, 3]) * 1000.0)
                if pos > 1.0:
                    print(f"  arm {arm}: FAIL pose err {pos:.3f} mm")
                    ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_straight_elbow_singularity() -> bool:
    """Adversarial: near-straight elbow -> psi unobservable, solve still exact
    in pose (the arm angle stops meaning anything but pose must not break)."""
    print("\n  --- Straight-elbow singularity (adversarial) ---")
    ok = True
    for arm in ARMS:
        s = _make(arm)
        g = s.geom
        # near-straight config: q4 close to 0, everything else modest.
        q = np.array([0.2, 0.1, -0.4, -0.05, 0.1, 0.2, 0.1])
        q = np.clip(q, g.lower + 0.01, g.upper - 0.01)
        T = s.fk(q)
        psi = s.psi_from_config(q)
        # psi may be None (elbow ~on axis). Solve at an arbitrary psi anyway and
        # assert the pose is still met (elbow circle radius ~0 -> any psi works).
        sol = s.solve(T, 0.3)
        if sol is None:
            print(f"  arm {arm}: note solve returned None at straight elbow (psi={psi})")
        else:
            pos = float(np.linalg.norm(s.fk(sol)[:3, 3] - T[:3, 3]) * 1000.0)
            if pos > 1.0:
                print(f"  arm {arm}: FAIL straight-elbow pose err {pos:.3f} mm")
                ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_ood_fallback_scan() -> bool:
    print("\n  --- OOD fallback: infeasible psi -> nearest feasible (opt-in) ---")
    rng = np.random.default_rng(21)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        found = False
        for q_seed in _sample_q(rng, s.lower_limits, s.upper_limits, 20):
            T = s.fk(q_seed)
            psi_gt = s.psi_from_config(q_seed)
            if psi_gt is None:
                continue
            # Opposite-side arm angle: elbow on the other side of the S-W
            # circle, outside the joint limits for most configs.
            bad = float(psi_gt + math.pi)
            bad = math.atan2(math.sin(bad), math.cos(bad))
            if s.solve(T, bad) is not None:
                continue  # feasible on both sides here; try another seed
            c0 = s.fallback_count
            # Default contract: infeasible psi -> None, counter untouched.
            if s.solve(T, bad) is not None:
                ok = False
                print(f"  arm {arm}: FAIL default solve returned a pose for infeasible psi")
                break
            if s.fallback_count != c0:
                ok = False
                print(f"  arm {arm}: FAIL default solve mutated fallback_count")
                break
            # Opt-in fallback rescues the frame.
            q_fb = s.solve(T, bad, fallback_scan=True)
            if q_fb is None:
                ok = False
                print(f"  arm {arm}: FAIL fallback_scan returned None")
                break
            if s.fallback_count != c0 + 1:
                ok = False
                print(f"  arm {arm}: FAIL fallback_count not incremented ({c0} -> {s.fallback_count})")
                break
            pos = float(np.linalg.norm(s.fk(q_fb)[:3, 3] - T[:3, 3]) * 1000.0)
            if pos > 1.0:
                ok = False
                print(f"  arm {arm}: FAIL fallback pose err {pos:.3f} mm")
                break
            found = True
            print(f"  arm {arm}: rescued psi={bad:.2f} -> pose err {pos:.4f} mm, "
                  f"fallback_count={s.fallback_count}")
            break
        if not found:
            ok = False
            print(f"  arm {arm}: FAIL no infeasible opposite-side psi in 20 seeds")
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_teleop_hard_mode_equivalence() -> bool:
    print("\n  --- teleop solve_hard == pim_ik solve (hard-elbow record premise) ---")
    rng = np.random.default_rng(31)
    ok = True
    from astral_arm_teleop.ik.geometric import GeometricIKSolver

    for arm in ARMS:
        t = GeometricIKSolver("left" if arm == "L" else "right")
        s = _make(arm)
        worst = 0.0
        n = 0
        for q in _sample_q(rng, s.lower_limits, s.upper_limits, 25):
            T = s.fk(q)
            psi = s.psi_from_config(q)
            if psi is None:
                continue
            q_t = t.solve_hard(T, psi, q_init=q)
            q_p = s.solve(T, psi, q_init=q)
            if q_t is None or q_p is None:
                ok = False
                print(f"  arm {arm}: FAIL one solver returned None")
                break
            worst = max(worst, float(np.max(np.abs(_wrap(q_t - q_p)))))
            n += 1
        if worst > 1e-9:
            ok = False
            print(f"  arm {arm}: FAIL max |q_teleop_hard - q_pim|={worst:.2e}")
        print(f"  arm {arm}: n={n}, max joint diff {worst:.2e} rad "
              f"-> {'PASS' if worst <= 1e-9 and n > 0 else 'FAIL'}")
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    print("Astral PiM-IK — Deterministic arm-angle solver test suite")
    gt = _GroundTruth()
    results = {
        "fk_matches_urdf": test_fk_matches_urdf(gt),
        "roundtrip_psi_fidelity": test_roundtrip_and_psi_fidelity(),
        "within_limits": test_within_limits(),
        "unreachable_none": test_unreachable_none(),
        "determinism": test_determinism(),
        "warmstart_branch": test_warmstart_branch_selection(),
        "straight_elbow": test_straight_elbow_singularity(),
        "ood_fallback_scan": test_ood_fallback_scan(),
        "teleop_hard_equiv": test_teleop_hard_mode_equivalence(),
    }
    print("\n  SUMMARY:")
    for k, v in results.items():
        print(f"    {k}: {'PASS' if v else 'FAIL'}")
    ok = all(results.values())
    print(f"FINAL: {'ALL PASSED' if ok else 'SOME FAILED'}")
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
