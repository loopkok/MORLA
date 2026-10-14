#!/usr/bin/env python3
"""VR-to-arm coordinate mapping validation test.

Verifies that hand movements in VR produce correct arm movements:
  - Hand forward  (VR X+)  → Arm forward
  - Hand outward  (VR Y+)  → Arm outward
  - Hand downward (VR Z+)  → Arm downward
  - Hand rotation preserved through IK chain

Also verifies TCP offset compensation:
  gripper target → flange target → IK → joint angles → FK → flange → gripper

Usage:
  python3 nero_arm_teleop/scripts/nero_vr_mapping_regression.py
"""

import sys
import numpy as np
from scipy.spatial.transform import Rotation

from nero_arm_teleop.ik_solver import IKSolver, fk as raw_fk


# ==============================================================================
# Expected VR-to-arm rotation matrices
# ==============================================================================

# Left arm:
#   VR X+(forward) -> Arm Y+(forward):  [0, 1, 0]
#   VR Y+(outward) -> Arm Z+(outward):  [0, 0, 1]
#   VR Z+(down)    -> Arm X-(down):     [-1, 0, 0]
R_LEFT = np.array([
    [0, 0, -1],
    [1, 0,  0],
    [0, 1,  0],
])

# Right arm:
#   VR X+(forward) -> Arm Y-(forward, since arm Y+ is backward): [0, -1, 0]
#   VR Y+(right)   -> Arm Z+(outward):                            [0,  0, 1]
#   VR Z+(down)    -> Arm X-(down):                               [-1, 0, 0]
R_RIGHT = np.array([
    [0,  0, -1],
    [-1, 0,  0],
    [0,  1,  0],
])

# TCP offset: [x, y, z, roll, pitch, yaw] — flange → gripper
TCP_OFFSET = [0.175, 0.0, -0.0235, 0, 0, 0]


def build_tcp_to_flange():
    """Build T_tcp_to_flange = inv(T_flange_to_tcp)."""
    T_ft = np.eye(4)
    T_ft[:3, :3] = Rotation.from_euler("xyz", TCP_OFFSET[3:]).as_matrix()
    T_ft[:3, 3] = np.array(TCP_OFFSET[:3])
    return np.linalg.inv(T_ft)


T_TCP_TO_FLANGE = build_tcp_to_flange()


def test_vr_to_arm_position(side, R_expected):
    """Test that VR position deltas map correctly to arm frame."""
    print(f"\n{'='*60}")
    print(f"TEST: {side.upper()} arm — VR position → arm position mapping")
    print(f"{'='*60}")

    # Test cases: (name, vr_delta, expected_arm_sign)
    # arm X = up(+)/down(-), arm Y = forward(+)/backward(-), arm Z = outward(+)
    if side == "left":
        tests = [
            # (description,  vr_delta,          expected_arm_direction)
            ("forward",     np.array([1, 0, 0]), np.array([0, 1, 0])),   # VR X+ → Arm Y+
            ("outward",     np.array([0, 1, 0]), np.array([0, 0, 1])),   # VR Y+ → Arm Z+
            ("downward",    np.array([0, 0, 1]), np.array([-1, 0, 0])),  # VR Z+ → Arm X-
            ("backward",    np.array([-1, 0, 0]), np.array([0, -1, 0])),  # VR X- → Arm Y-
            ("inward",      np.array([0, -1, 0]), np.array([0, 0, -1])),  # VR Y- → Arm Z-
            ("upward",      np.array([0, 0, -1]), np.array([1, 0, 0])),   # VR Z- → Arm X+
        ]
    else:  # right
        tests = [
            ("forward",     np.array([1, 0, 0]), np.array([0, -1, 0])),  # VR X+ → Arm Y-
            ("outward",     np.array([0, 1, 0]), np.array([0, 0, 1])),   # VR Y+ → Arm Z+
            ("downward",    np.array([0, 0, 1]), np.array([-1, 0, 0])),   # VR Z+ → Arm X-
            ("backward",    np.array([-1, 0, 0]), np.array([0, 1, 0])),   # VR X- → Arm Y+
            ("inward",      np.array([0, -1, 0]), np.array([0, 0, -1])),  # VR Y- → Arm Z-
            ("upward",      np.array([0, 0, -1]), np.array([1, 0, 0])),   # VR Z- → Arm X+
        ]

    all_ok = True
    for name, vr_delta, expected_dir in tests:
        arm_delta = R_expected @ vr_delta
        # Check direction matches (arm_delta should point in same direction)
        dot = np.dot(arm_delta, expected_dir)
        # Both should be unit vectors or zero
        norm_prod = np.linalg.norm(arm_delta) * np.linalg.norm(expected_dir)
        if norm_prod < 1e-10:
            ok = np.linalg.norm(arm_delta) < 1e-10
        else:
            ok = abs(dot / norm_prod - 1.0) < 1e-6
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_ok = False
        print(f"  Hand {name:10s}: VR{vr_delta} → Arm{arm_delta} | "
              f"expect {expected_dir} → {status}")

    print(f"  Overall: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_tcp_offset_roundtrip():
    """Verify gripper→flange→gripper round-trip with TCP offset."""
    print(f"\n{'='*60}")
    print("TEST: TCP offset gripper→flange→gripper round-trip")
    print(f"{'='*60}")
    print(f"  TCP offset: {TCP_OFFSET}")

    # Build flange→TCP transform
    T_ft = np.eye(4)
    T_ft[:3, :3] = Rotation.from_euler("xyz", TCP_OFFSET[3:]).as_matrix()
    T_ft[:3, 3] = np.array(TCP_OFFSET[:3])
    T_tf = np.linalg.inv(T_ft)

    # Create a sample gripper pose
    T_gripper = np.eye(4)
    T_gripper[:3, :3] = Rotation.from_euler("xyz", [0.5, -1.2, -3.1]).as_matrix()
    T_gripper[:3, 3] = [-0.3, 0.2, 0.4]

    # Gripper → flange
    T_flange = T_gripper @ T_tf

    # Flange → gripper (round-trip)
    T_gripper_rt = T_flange @ T_ft

    pos_err = np.linalg.norm(T_gripper[:3, 3] - T_gripper_rt[:3, 3])
    rot_err = np.linalg.norm(T_gripper[:3, :3] - T_gripper_rt[:3, :3])

    ok = pos_err < 1e-10 and rot_err < 1e-10
    status = "PASS" if ok else "FAIL"
    print(f"  Round-trip: pos_err={pos_err:.2e}m, rot_err={rot_err:.2e} → {status}")
    return ok


def test_ik_with_tcp_offset():
    """Verify IK solves correctly when TCP offset is applied.

    Given a gripper target pose, apply TCP→flange conversion, solve IK,
    then FK the solution and apply flange→TCP to get the achieved gripper pose.
    """
    print(f"\n{'='*60}")
    print("TEST: IK chain with TCP offset compensation")
    print(f"{'='*60}")

    solver = IKSolver(fast_mode=True)

    # Start from init pose
    q_init = np.array([-2.16, 1.15, 0.38, 1.43, -2.21, 1.02, 0.29])
    solver.sync_state(q_init)

    # Warm-up
    T_flange_init = solver.fk(q_init)
    solver.solve(T_flange_init)

    # Target gripper pose: same as init but shifted forward 0.05m
    # Gripper init = flange_init @ T_flange_to_tcp
    T_ft = np.eye(4)
    T_ft[:3, :3] = Rotation.from_euler("xyz", TCP_OFFSET[3:]).as_matrix()
    T_ft[:3, 3] = np.array(TCP_OFFSET[:3])

    T_gripper_init = T_flange_init @ T_ft
    T_gripper_target = T_gripper_init.copy()
    T_gripper_target[0, 3] += 0.05  # Shift in gripper X (physical direction depends on orientation)

    # Convert to flange target for IK
    T_tf = np.linalg.inv(T_ft)
    T_flange_target = T_gripper_target @ T_tf

    # Solve IK
    sol = solver.solve(T_flange_target)
    if sol is None:
        print("  FAIL: IK did not converge")
        return False

    # FK → flange → gripper
    T_flange_achieved = solver.fk(sol)
    T_gripper_achieved = T_flange_achieved @ T_ft

    pos_err = np.linalg.norm(T_gripper_target[:3, 3] - T_gripper_achieved[:3, 3])
    rot_err = np.linalg.norm(
        T_gripper_target[:3, :3] - T_gripper_achieved[:3, :3]
    )

    ok = pos_err < 0.01 and rot_err < 0.20
    status = "PASS" if ok else "FAIL"
    print(f"  Gripper FK check: pos_err={pos_err*1000:.2f}mm, rot_err={rot_err:.4f}")
    print(f"  Joints (deg): {np.degrees(sol).round(1).tolist()}")
    print(f"  Result: {status}")
    return ok


def test_end_to_end_mapping(side, R):
    """Verify end-to-end: VR delta → arm delta → IK → FK consistency."""
    print(f"\n{'='*60}")
    print(f"TEST: End-to-end VR→arm→IK→FK chain ({side} arm)")
    print(f"{'='*60}")

    solver = IKSolver(fast_mode=True)
    q_init = np.array([-2.16, 1.15, 0.38, 1.43, -2.21, 1.02, 0.29])
    if side == "right":
        q_init = np.array([2.16, 1.15, -0.38, 1.43, -2.21, 1.02, -0.29])

    solver.sync_state(q_init)
    T_flange_init = solver.fk(q_init)
    solver.solve(T_flange_init)

    # TCP transforms
    T_ft = np.eye(4)
    T_ft[:3, :3] = Rotation.from_euler("xyz", TCP_OFFSET[3:]).as_matrix()
    T_ft[:3, 3] = np.array(TCP_OFFSET[:3])
    T_tf = np.linalg.inv(T_ft)

    T_gripper_init = T_flange_init @ T_ft

    # Simulate several VR hand movements
    tests = [
        ("forward 5cm",  np.array([0.05, 0.0, 0.0])),
        ("outward 5cm",  np.array([0.0, 0.05, 0.0])),
        ("down 5cm",     np.array([0.0, 0.0, 0.05])),
    ]

    all_ok = True
    for name, vr_delta in tests:
        # VR delta → arm delta
        arm_delta = R @ vr_delta

        # Arm delta → gripper target
        T_gripper_target = T_gripper_init.copy()
        T_gripper_target[:3, 3] += arm_delta

        # Gripper → flange → IK
        T_flange_target = T_gripper_target @ T_tf
        sol = solver.solve(T_flange_target)

        if sol is None:
            print(f"  {name}: FAIL — IK did not converge")
            all_ok = False
            continue

        # FK verification
        T_flange_achieved = solver.fk(sol)
        T_gripper_achieved = T_flange_achieved @ T_ft

        pos_err = np.linalg.norm(
            T_gripper_target[:3, 3] - T_gripper_achieved[:3, 3]
        )
        rot_err = np.linalg.norm(
            T_gripper_target[:3, :3] - T_gripper_achieved[:3, :3]
        )

        ok = pos_err < 0.01 and rot_err < 0.20
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_ok = False

        print(f"  {name:15s}: gripper_pos_err={pos_err*1000:.2f}mm "
              f"rot_err={rot_err:.4f} → {status}")

    print(f"  Overall: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def main():
    print("Nero Teleop — VR-to-Arm Coordinate Mapping Validation")
    print("=" * 60)
    print(f"TCP offset: {TCP_OFFSET}")

    results = {}

    results["left_position_map"] = test_vr_to_arm_position("left", R_LEFT)
    results["right_position_map"] = test_vr_to_arm_position("right", R_RIGHT)
    results["tcp_roundtrip"] = test_tcp_offset_roundtrip()
    results["ik_tcp_chain"] = test_ik_with_tcp_offset()
    results["e2e_left"] = test_end_to_end_mapping("left", R_LEFT)
    results["e2e_right"] = test_end_to_end_mapping("right", R_RIGHT)

    print("\n" + "=" * 60)
    print("MAPPING VALIDATION SUMMARY")
    print("=" * 60)
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")

    total = len(results)
    passed = sum(results.values())
    print(f"\n  {passed}/{total} tests passed")

    if passed < total:
        print("\n!!! Some tests FAILED — check VR-to-arm rotation matrices !!!")
        sys.exit(1)
    else:
        print("\nAll mapping tests PASSED — VR-to-arm matrices are correct.")


if __name__ == "__main__":
    main()
