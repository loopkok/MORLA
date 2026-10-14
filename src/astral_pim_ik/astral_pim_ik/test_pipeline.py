#!/usr/bin/env python3
"""End-to-end test of the two-stage pipeline (stage 1 synthetic -> stage 2).

Uses ``SyntheticPsiPredictor`` (a perfect stage-1 stand-in) so the full
``predict psi -> geometric solve`` flow runs without torch. A joint-space
trajectory is generated from a seed config; for each frame the pipeline is
given ``(T_ee, psi)`` and must recover the config with sub-mm pose error and
no per-frame jumps (warm-start continuity).

Usage:
  PYTHONPATH=src/astral_arm_teleop:src/astral_pim_ik \
    <python-with-pinocchio-and-torch> \   # e.g. miniconda3/envs/arm_sdk/bin/python3
    src/astral_pim_ik/astral_pim_ik/test_pipeline.py
"""

from __future__ import annotations

import sys

import numpy as np

from astral_pim_ik.dataset import sample_configs
from astral_pim_ik.geometry import GeometricArmAngleSolver
from astral_pim_ik.pipeline import PiMIKPipeline, SyntheticPsiPredictor

ARMS = ("L", "R")


def _wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _make(arm: str) -> GeometricArmAngleSolver:
    return GeometricArmAngleSolver("left" if arm == "L" else "right")


def _smooth_joint_traj(s: GeometricArmAngleSolver, q0: np.ndarray, n: int) -> np.ndarray:
    """Joint-space drift of a seed config (stays clear of singularities)."""
    qs = []
    for k in range(n):
        q = q0.copy()
        q[0] += 0.05 * np.sin(2 * np.pi * k / n)
        q[2] += 0.03 * np.sin(2 * np.pi * k / n + 1.0)
        q = np.clip(q, s.lower_limits + 0.02, s.upper_limits - 0.02)
        qs.append(q)
    return np.asarray(qs, dtype=float)


def test_pipeline_roundtrip() -> bool:
    print("\n  --- Two-stage pipeline round-trip (joint-space trajectory) ---")
    rng = np.random.default_rng(9)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        q0 = sample_configs(s, 1, rng)[0]
        qs = _smooth_joint_traj(s, q0, 40)
        T_seq = np.asarray([s.fk(q) for q in qs])
        psi_labels = []
        for q in qs:
            psi = s.psi_from_config(q)
            if psi is None:
                print(f"  arm {arm}: FAIL unobservable psi in trajectory")
                ok = False
                psi_labels.append([1.0, 0.0])
            else:
                psi_labels.append([np.cos(psi), np.sin(psi)])

        pipe = PiMIKPipeline(SyntheticPsiPredictor(np.array(psi_labels)),
                             "left" if arm == "L" else "right")
        q_seq, psi_seq = pipe.solve_trajectory(T_seq, q_init=q0)
        ev = pipe.evaluate(T_seq, q_seq)
        n_none = sum(1 for q in q_seq if q is None)
        ok_arm = ev["solved"] == len(T_seq) and ev["max_pos_mm"] < 1.0 and n_none == 0
        ok &= ok_arm
        print(f"  arm {arm}: solved {ev['solved']}/{ev['total']}, "
              f"max pos {ev['max_pos_mm']:.4f} mm, none={n_none} -> {'PASS' if ok_arm else 'FAIL'}")
    return ok


def test_pipeline_warmstart_continuity() -> bool:
    print("\n  --- Per-frame warm-start continuity (no jumps) ---")
    rng = np.random.default_rng(13)
    ok = True
    for arm in ARMS:
        s = _make(arm)
        q0 = sample_configs(s, 1, rng)[0]
        qs = _smooth_joint_traj(s, q0, 30)
        T_seq = np.asarray([s.fk(q) for q in qs])
        psi_labels = []
        for q in qs:
            psi = s.psi_from_config(q)
            psi_labels.append([np.cos(psi), np.sin(psi)] if psi is not None else [1.0, 0.0])

        pipe = PiMIKPipeline(SyntheticPsiPredictor(np.array(psi_labels)),
                             "left" if arm == "L" else "right")
        q_seq, _ = pipe.solve_trajectory(T_seq, q_init=q0)
        max_jump = 0.0
        prev = q0
        for q in q_seq:
            if q is not None:
                max_jump = max(max_jump, float(np.max(np.abs(_wrap(q - prev)))))
                prev = q
        ok_arm = max_jump < 0.3
        ok &= ok_arm
        print(f"  arm {arm}: max_jump={max_jump:.4f} rad -> {'PASS' if ok_arm else 'FAIL'}")
    return ok


def main() -> None:
    print("Astral PiM-IK — Pipeline test suite")
    results = {
        "pipeline_roundtrip": test_pipeline_roundtrip(),
        "warmstart_continuity": test_pipeline_warmstart_continuity(),
    }
    print("\n  SUMMARY:")
    for k, v in results.items():
        print(f"    {k}: {'PASS' if v else 'FAIL'}")
    if not all(results.values()):
        sys.exit(1)
    print("FINAL: ALL PASSED")


if __name__ == "__main__":
    main()
