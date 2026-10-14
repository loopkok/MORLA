#!/usr/bin/env python3
"""Offline tests for dataset generation and the arm-angle label.

The label ``psi`` must round-trip through the deterministic solver: for every
generated sample, ``solve(T_ee, psi)`` recovers a config whose forward pose and
arm angle match. Also checks the .npz schema and the singular-sample mask.

Usage:
  PYTHONPATH=src/astral_arm_teleop:src/astral_pim_ik \
    <python-with-pinocchio-and-torch> \   # e.g. miniconda3/envs/arm_sdk/bin/python3
    src/astral_pim_ik/astral_pim_ik/test_dataset.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from astral_pim_ik.dataset import generate_dataset, generate_trajectory_dataset
from astral_pim_ik.geometry import GeometricArmAngleSolver

ARMS = ("L", "R")


def _wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def test_schema_and_labels() -> bool:
    print("\n  --- Dataset schema + psi label round-trip ---")
    ok = True
    for arm in ARMS:
        s = GeometricArmAngleSolver("left" if arm == "L" else "right")
        d = generate_dataset("left" if arm == "L" else "right", n=300, out_npz="", seed=0)
        for key in ("T_ee", "psi", "joint_positions", "L_upper", "L_lower", "is_valid", "q"):
            if key not in d:
                print(f"  arm {arm}: FAIL missing key {key}")
                ok = False
        if not (d["T_ee"].shape[1:] == (4, 4) and d["psi"].shape[1] == 2
                and d["joint_positions"].shape[1:] == (3, 3) and d["q"].shape[1] == 7):
            print(f"  arm {arm}: FAIL bad shapes")
            ok = False

        worst_pos = 0.0
        worst_dpsi = 0.0
        for i in range(len(d["T_ee"])):
            T = d["T_ee"][i]
            psi = float(np.arctan2(d["psi"][i, 1], d["psi"][i, 0]))
            q = s.solve(T, psi, q_init=d["q"][i])
            if q is None:
                print(f"  arm {arm}: FAIL solve None at {i}")
                ok = False
                continue
            worst_pos = max(worst_pos, float(np.linalg.norm(s.fk(q)[:3, 3] - T[:3, 3]) * 1000.0))
            psi_sol = s.psi_from_config(q)
            worst_dpsi = max(worst_dpsi, abs(float(_wrap(np.array([psi_sol - psi]))[0])))
        # elbow/wrist in joint_positions must match FK
        L_upper = float(d["L_upper"][0])
        L_lower = float(d["L_lower"][0])
        # link lengths are stored as float32 (same as the torch training path);
        # allow float32 rounding (~1e-7 relative), not a real geometry drift.
        if abs(L_upper - s.geom.l_se) > 1e-6 or abs(L_lower - s.geom.l_ew) > 1e-6:
            print(f"  arm {arm}: FAIL link lengths ({L_upper}, {L_lower}) vs ({s.geom.l_se}, {s.geom.l_ew})")
            ok = False

        print(f"  arm {arm}: n={len(d['T_ee'])} valid={int(d['is_valid'].sum())}, "
              f"worst pos {worst_pos:.4f} mm, worst dpsi {np.degrees(worst_dpsi):.3f} deg "
              f"-> {'PASS' if ok else 'FAIL'}")
    return ok


def test_npz_roundtrip() -> bool:
    print("\n  --- .npz write / read ---")
    import tempfile

    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "d.npz"
        generate_dataset("left", n=50, out_npz=str(p), seed=1)
        data = np.load(str(p))
        ok = set(data.keys()) >= {"T_ee", "psi", "joint_positions", "is_valid", "q"}
        print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_trajectory_dataset() -> bool:
    print("\n  --- Trajectory dataset: schema + temporal smoothness ---")
    ok = True
    s = GeometricArmAngleSolver("left")
    n_traj, traj_len = 4, 30
    d = generate_trajectory_dataset("left", n_traj=n_traj, traj_len=traj_len,
                                    out_npz="", seed=0)
    n = len(d["T_ee"])
    if n < n_traj * traj_len - 2:
        print(f"  FAIL too few frames: {n} < {n_traj * traj_len - 2}")
        ok = False
    for key in ("T_ee", "psi", "joint_positions", "L_upper", "L_lower", "is_valid", "q"):
        if key not in d:
            print(f"  FAIL missing key {key}")
            ok = False
    # Consecutive frames must be near each other in joint space (smooth motion),
    # unlike IID sampling — this is what lets the windowed net infer psi.
    # Inter-trajectory concatenation cuts jump like recorded-teleop session
    # boundaries, so exclude the largest n_traj-1 per-frame jumps.
    jumps = np.max(np.abs(_wrap(d["q"][1:] - d["q"][:-1])), axis=-1)
    if len(jumps) > n_traj - 1:
        max_inner = float(np.sort(jumps)[:-(n_traj - 1)][-1])
    else:
        max_inner = 0.0
    if max_inner > 0.35:
        print(f"  FAIL max in-trajectory joint jump {max_inner:.3f} rad (not smooth)")
        ok = False
    print(f"  n={n}, valid={int(d['is_valid'].sum())}, "
          f"max in-trajectory joint jump {max_inner:.3f} rad -> {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    print("Astral PiM-IK — Dataset test suite")
    results = {
        "schema_labels": test_schema_and_labels(),
        "npz_roundtrip": test_npz_roundtrip(),
        "trajectory_dataset": test_trajectory_dataset(),
    }
    print("\n  SUMMARY:")
    for k, v in results.items():
        print(f"    {k}: {'PASS' if v else 'FAIL'}")
    if not all(results.values()):
        sys.exit(1)
    print("FINAL: ALL PASSED")


if __name__ == "__main__":
    main()
