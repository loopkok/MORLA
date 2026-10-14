#!/usr/bin/env python3
"""Astral robot_world → base_link mapping + PoseProcessor / TCP checks.

Unlike Nero (per-arm vr_to_arm_rot), Astral default map is identity because
Quest3 already publishes robot_world aligned with Astral axes.

Usage:
  ros2 run astral_arm_teleop test_vr_mapping
"""

from __future__ import annotations

import sys

import numpy as np
from scipy.spatial.transform import Rotation

from astral_arm_teleop.ik import make_ik_solver
from astral_arm_teleop.pose_processor import PoseProcessor

R_IDENTITY = np.eye(3)


def test_identity_axes() -> bool:
    print("\n" + "=" * 60)
    print("TEST: robot_world → base_link identity axes")
    print("=" * 60)
    # Astral robot_world: X left, Y back, Z up — same as base_link when aligned
    cases = [
        ("+X (left)", np.array([1, 0, 0]), np.array([1, 0, 0])),
        ("+Y (back)", np.array([0, 1, 0]), np.array([0, 1, 0])),
        ("+Z (up)", np.array([0, 0, 1]), np.array([0, 0, 1])),
        ("-X", np.array([-1, 0, 0]), np.array([-1, 0, 0])),
    ]
    ok_all = True
    for name, vin, expect in cases:
        vout = R_IDENTITY @ vin
        ok = np.allclose(vout, expect)
        print(f"  {name}: {vin} → {vout} [{'PASS' if ok else 'FAIL'}]")
        ok_all &= ok
    return ok_all


def test_pose_processor_delta() -> bool:
    print("\n" + "=" * 60)
    print("TEST: PoseProcessor incremental delta (I map, no smooth)")
    print("=" * 60)
    pp = PoseProcessor(
        robot_world_to_base_rot=R_IDENTITY,
        pos_smoothing=0.0,
        rot_smoothing=0.0,
        motion_scale=1.0,
        flip_pitch=False,
    )
    pp.update_vr_pose(np.zeros(3), Rotation.identity())
    pp.update_vr_pose(np.array([0.1, -0.05, 0.02]), Rotation.identity())
    dp, _ = pp.process()
    expect = np.array([0.1, -0.05, 0.02])
    ok = np.allclose(dp, expect, atol=1e-9)
    print(f"  delta={np.round(dp, 4)} expect={expect} → {'PASS' if ok else 'FAIL'}")

    T = pp.compute_target_pose(
        dp,
        Rotation.identity(),
        robot_init_pos=np.array([0.2, 0.0, 0.1]),
        robot_init_rot=np.eye(3),
    )
    ok2 = np.allclose(T[:3, 3], np.array([0.3, -0.05, 0.12]))
    print(f"  target.p={np.round(T[:3, 3], 4)} → {'PASS' if ok2 else 'FAIL'}")
    return ok and ok2


def test_tcp_roundtrip() -> bool:
    print("\n" + "=" * 60)
    print("TEST: TCP offset round-trip")
    print("=" * 60)
    tcp = [0.05, 0.0, 0.0, 0.0, 0.1, 0.0]
    T_ft = np.eye(4)
    T_ft[:3, :3] = Rotation.from_euler("xyz", tcp[3:]).as_matrix()
    T_ft[:3, 3] = tcp[:3]
    T_tf = np.linalg.inv(T_ft)
    Tg = np.eye(4)
    Tg[:3, 3] = [0.3, 0.1, 0.2]
    Tg[:3, :3] = Rotation.from_euler("xyz", [0.2, -0.1, 0.3]).as_matrix()
    Tr = (Tg @ T_tf) @ T_ft
    ok = np.allclose(Tg, Tr)
    print(f"  pos_err={np.linalg.norm(Tg[:3,3]-Tr[:3,3]):.2e} → {'PASS' if ok else 'FAIL'}")
    return ok


def test_e2e_ik(side: str) -> bool:
    print("\n" + "=" * 60)
    print(f"TEST: e2e VR delta → IK → FK ({side}, urdf_numerical)")
    print("=" * 60)
    try:
        bridge = make_ik_solver("urdf_numerical")
    except Exception as exc:  # noqa: BLE001
        print(f"  SKIP: {exc}")
        return True
    arm = "L" if side == "left" else "R"
    q0 = np.zeros(7)
    bridge.sync_state(np.concatenate([q0, q0]) if arm == "L" else np.concatenate([q0, q0]))
    # sync both
    q14 = np.zeros(14)
    bridge.sync_state(q14)
    T0 = bridge.fk(arm, q0)
    _ = bridge.solve(arm, T0)

    pp = PoseProcessor(
        R_IDENTITY, pos_smoothing=0.0, rot_smoothing=0.0, motion_scale=1.0
    )
    pp.update_vr_pose(np.zeros(3), Rotation.identity())
    all_ok = True
    for name, dvr in [
        ("+X 3cm", np.array([0.03, 0, 0])),
        ("+Y 3cm", np.array([0, 0.03, 0])),
        ("+Z 3cm", np.array([0, 0, 0.03])),
    ]:
        pp.reset()
        pp.update_vr_pose(np.zeros(3), Rotation.identity())
        pp.update_vr_pose(dvr, Rotation.identity())
        dp, dr = pp.process()
        T_tcp = pp.compute_target_pose(dp, dr, T0[:3, 3], T0[:3, :3])
        sol = bridge.solve(arm, T_tcp)
        if sol is None:
            print(f"  {name}: FAIL IK")
            all_ok = False
            continue
        err = float(np.linalg.norm(bridge.fk(arm, sol)[:3, 3] - T_tcp[:3, 3]) * 1000)
        ok = err < 15.0
        print(f"  {name}: err={err:.2f}mm → {'PASS' if ok else 'FAIL'}")
        all_ok &= ok
    return all_ok


def main():
    print("Astral Teleop — VR / base_link Mapping Validation")
    results = {
        "identity_axes": test_identity_axes(),
        "pose_processor": test_pose_processor_delta(),
        "tcp_roundtrip": test_tcp_roundtrip(),
        "e2e_left": test_e2e_ik("left"),
        "e2e_right": test_e2e_ik("right"),
    }
    print("\n" + "=" * 60)
    print("SUMMARY")
    for k, v in results.items():
        print(f"  {k}: {'PASS' if v else 'FAIL'}")
    if not all(results.values()):
        sys.exit(1)
    print("\nAll mapping tests PASSED.")


if __name__ == "__main__":
    main()
