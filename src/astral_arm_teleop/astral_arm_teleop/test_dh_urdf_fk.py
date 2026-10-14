#!/usr/bin/env python3
"""Compare analytic DH FK vs astral_robot_description URDF (Pinocchio).

Stage-1 adaptation check: same q → position / orientation error in *_base_link.
Shows why Nero closed-form topology cannot fully match Astral URDF SE3.

Usage:
  ros2 run astral_arm_teleop test_dh_urdf_fk
  python3 -m astral_arm_teleop.test_dh_urdf_fk
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

def _find_urdf() -> Path:
    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / "astral_robot_description/urdf/astral_robot.pin.urdf",
        here.parents[3] / "src/astral_robot_description/urdf/astral_robot.pin.urdf",
        Path("/home/robot/loopkok/sdk/astral_ws/src/astral_robot_description/"
             "urdf/astral_robot.pin.urdf"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("astral_robot_description"))
        candidates.insert(0, share / "urdf/astral_robot.pin.urdf")
    except Exception:  # noqa: BLE001
        pass
    for p in candidates:
        if p.is_file():
            return p
    return candidates[0]
_INIT_L = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0])
_INIT_R = np.array([0.32, -0.11, 0.53, -0.80, 0.28, 0.0, 0.0])


def _rot_err_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    return float(np.degrees(Rotation.from_matrix(Ra.T @ Rb).magnitude()))


def main() -> int:
    try:
        import pinocchio as pin
    except ImportError:
        print("pinocchio required: pip install pin")
        return 1

    from astral_arm_teleop.ik.analytic import AstralParams, IKSolver

    urdf = _find_urdf()
    if not urdf.is_file():
        print(f"URDF not found: {urdf}")
        return 1

    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()

    def urdf_fk(side: str, q7: np.ndarray) -> np.ndarray:
        q = pin.neutral(model)
        for i, k in enumerate(range(1, 8)):
            jid = model.getJointId(f"{side}_joint{k}")
            q[model.joints[jid].idx_q] = float(q7[i])
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        Tb = data.oMf[model.getFrameId(f"{side}_base_link")]
        Te = data.oMf[model.getFrameId(f"{side}_link7")]
        return (Tb.inverse() * Te).homogeneous.copy()

    configs = {
        "zero": np.zeros(7),
        "init": None,  # filled per side
        "bent": None,
    }

    print("=" * 60)
    print("Astral DH (Nero structure) vs URDF FK")
    print(f"URDF: {urdf}")
    print("Note: large config-dependent error ⇒ topology mismatch,")
    print("      not a single T_fixed / theta_offset fix.")
    print("=" * 60)

    overall_ok = True
    for side in ("left", "right"):
        params = (
            AstralParams.left_arm() if side == "left" else AstralParams.right_arm()
        )
        ik = IKSolver(params)
        init_q = _INIT_L if side == "left" else _INIT_R
        bent = (
            np.array([0.0, 0.5, 0.0, -0.8, 0.0, 0.2, 0.0])
            if side == "left"
            else np.array([0.0, -0.5, 0.0, -0.8, 0.0, 0.2, 0.0])
        )
        samples = {
            "zero": np.zeros(7),
            "init": init_q,
            "bent": bent,
        }
        print(f"\n--- {side}_base_link ---")
        print(
            f"  d_i={np.round(params.d_i, 5).tolist()}  "
            f"T_fixed=joint1 rpy (-90,0,90) deg"
        )
        for name, q in samples.items():
            Tu = urdf_fk(side, q)
            Td = ik.fk(q)
            dp = float(np.linalg.norm(Tu[:3, 3] - Td[:3, 3]) * 1000)
            dr = _rot_err_deg(Td[:3, :3], Tu[:3, :3])
            print(
                f"  {name:5s} |dp|={dp:7.1f} mm  |dR|={dr:6.1f} deg  "
                f"URDF_p={np.round(Tu[:3, 3], 3).tolist()}  "
                f"DH_p={np.round(Td[:3, 3], 3).tolist()}"
            )
            # Absolute match is not expected; flag only if self-IK breaks.
        # Self-consistency of DH (must stay green)
        q = init_q
        T = ik.fk(q)
        ik.sync_state(q)
        sol = ik.solve(T)
        if sol is None:
            print("  SELF_IK: FAIL at init")
            overall_ok = False
        else:
            e = float(np.linalg.norm(ik.fk(sol)[:3, 3] - T[:3, 3]) * 1000)
            print(f"  SELF_IK: OK  pos_err={e:.3f} mm (DH-consistent teleop)")

    print("\n" + "=" * 60)
    print("Adaptation policy:")
    print("  - Keep Nero MDH + arm-angle IK (closed-form).")
    print("  - d_i / joint_limits from URDF segments; T_fixed = joint1 rpy.")
    print("  - Do NOT force full URDF SE3 into MDH (breaks arm-angle IK).")
    print("  - Teleop = incremental in DH frame (FK zero pose).")
    print("  - Absolute URDF match → solver_type:=urdf_numerical.")
    print("=" * 60)
    print("RESULT:", "SELF_IK OK" if overall_ok else "SELF_IK FAILED")
    return 0 if overall_ok else 2


if __name__ == "__main__":
    sys.exit(main())
