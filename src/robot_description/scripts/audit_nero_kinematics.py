#!/usr/bin/env python3
"""Compare hardware-coordinate Nero solver-tip and arm-flange FK across models.

Run from the MORLA checkout with ``python3 src/robot_description/scripts/audit_nero_kinematics.py``.
This is a numerical model check only; it does not connect to or move hardware.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/robots/nero/nero_arm_teleop"))
sys.path.insert(0, str(ROOT / "src/common/morla_core"))
from nero_arm_teleop.ik_solver import NeroParams, fk as nero_fk  # noqa: E402
from morla_core.coordinates import compose, fixed_transform_between_urdf  # noqa: E402
from morla_core.coordinate_contract import build_coordinate_contract  # noqa: E402


def _relative(T_world_base: np.ndarray, T_world_tip: np.ndarray) -> np.ndarray:
    T_base_world = np.eye(4)
    T_base_world[:3, :3] = T_world_base[:3, :3].T
    T_base_world[:3, 3] = -T_base_world[:3, :3] @ T_world_base[:3, 3]
    return T_base_world @ T_world_tip


def _errors(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    position = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    angle = float(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude())
    return position, angle


def _mjcf_geom_transform(mjcf_path: Path, body_name: str, mesh_name: str) -> np.ndarray:
    """Read a geom's authored frame from MJCF, before MuJoCo mesh recentering."""
    root = ET.parse(mjcf_path).getroot()
    body = next((node for node in root.iter("body") if node.get("name") == body_name), None)
    if body is None:
        raise RuntimeError(f"MJCF body {body_name!r} not found")
    geom = next((node for node in body.findall("geom") if node.get("mesh") == mesh_name), None)
    if geom is None:
        raise RuntimeError(f"MJCF geom using mesh {mesh_name!r} not found in {body_name}")
    position = [float(x) for x in geom.get("pos", "0 0 0").split()]
    quaternion = [float(x) for x in geom.get("quat", "1 0 0 0").split()]
    if len(quaternion) != 4:
        raise RuntimeError(f"MJCF geom {mesh_name!r} quaternion must have four values")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(
        [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    ).as_matrix()
    T[:3, 3] = position
    return T


def main() -> int:
    import mujoco
    import pinocchio as pin

    urdf = ROOT / "src/robot_description/urdf/nero/xhand_nero_description.urdf"
    mjcf = ROOT / "src/robot_description/mjcf/xnero/xnero_dual.xml"
    pin_model = pin.buildModelFromUrdf(str(urdf))
    pin_data = pin_model.createData()
    mj_model = mujoco.MjModel.from_xml_path(str(mjcf))
    mj_data = mujoco.MjData(mj_model)
    params = NeroParams.default()
    rng = np.random.default_rng(20261014)
    cases: list[np.ndarray] = [np.zeros(7)]
    for _ in range(500):
        cases.append(rng.uniform(params.joint_limits[:, 0], params.joint_limits[:, 1]))

    report: dict[str, object] = {
        "sample_count_per_side": len(cases),
        "joint_convention": "hardware_q_rad_no_offset",
        "solver_tip": "*_link7",
        "configured_arm_end": "*_link7",
        "hardware_or_live_simulation_used": False,
        "sides": {},
    }
    selected_profiles = {}
    profile_root = ROOT / "src/robots/nero/morla_nero_robot/config/profiles"
    for profile_path in sorted(profile_root.glob("nero_*.yaml")):
        values = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        profile_id = str(values.get("profile_id", profile_path.stem))
        if values.get("robot_id") != "nero":
            continue
        contract = build_coordinate_contract(values)
        selected_profiles[profile_id] = {}
        for side, arm in contract.arms.items():
            if arm["tool"]["frame"] != arm["solver_tip_frame"]:
                raise RuntimeError(
                    f"{profile_id}/{side} active tool is {arm['tool']['frame']!r}, "
                    f"expected solver tip {arm['solver_tip_frame']!r}"
                )
            T_tip_tool = contract.resolve_arm_tool_transform(side, str(urdf))
            selected_profiles[profile_id][side] = {
                "solver_tip_frame": arm["solver_tip_frame"],
                "physical_flange_frame": arm["physical_flange_frame"],
                "active_tool_id": arm["tool"]["id"],
                "active_tool_frame": arm["tool"]["frame"],
                "tool_calibration_status": arm["tool"]["calibration_status"],
                "T_solver_tip_from_active_tool": T_tip_tool.tolist(),
            }
    report["nero_profile_tool_selection"] = selected_profiles
    for side in ("left", "right"):
        joint_ids = [pin_model.getJointId(f"{side}_joint{i}") for i in range(1, 8)]
        if any(jid == 0 for jid in joint_ids):
            raise RuntimeError(f"Pinocchio joint lookup failed for {side}")
        qidx = [int(pin_model.idx_qs[jid]) for jid in joint_ids]
        base_fid = pin_model.getFrameId(f"{side}_base_link")
        tip_fid = pin_model.getFrameId(f"{side}_link7")
        flange_fid = pin_model.getFrameId(f"{side}_gripper_flange")
        if pin_model.frames[flange_fid].name != f"{side}_gripper_flange":
            raise RuntimeError(f"Pinocchio flange frame lookup failed for {side}")
        T_tip_flange = fixed_transform_between_urdf(
            urdf, f"{side}_link7", f"{side}_gripper_flange"
        )
        mj_joint_ids = [
            mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_joint{i}")
            for i in range(1, 8)
        ]
        if any(jid < 0 for jid in mj_joint_ids):
            raise RuntimeError(f"MuJoCo joint lookup failed for {side}")
        mj_qadr = [int(mj_model.jnt_qposadr[jid]) for jid in mj_joint_ids]
        # MJCF fuses the fixed base_link->link1 transform into the first arm
        # body, so *_base_link is intentionally absent from the MJCF body list.
        # The MJCF still preserves world placement; express that pose in the
        # URDF base frame, which is the IK solver's reference frame.
        mj_tip_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_link7")
        if mj_tip_id < 0:
            raise RuntimeError(f"MuJoCo link7 lookup failed for {side}")
        T_mj_tip_flange = _mjcf_geom_transform(mjcf, f"{side}_link7", "gripper_flange")
        flange_model_error = _errors(T_tip_flange, T_mj_tip_flange)
        if flange_model_error[0] > 5e-5 or flange_model_error[1] > 1e-4:
            raise RuntimeError(
                f"{side} URDF/MJCF link7→flange mismatch: "
                f"{flange_model_error[0]:.8g}m/{flange_model_error[1]:.8g}rad"
            )
        errors_tip_urdf = [[], []]
        errors_tip_mjcf = [[], []]
        errors_flange_urdf = [[], []]
        errors_flange_mjcf = [[], []]
        for q in cases:
            pin_q = pin.neutral(pin_model)
            pin_q[qidx] = q
            pin.forwardKinematics(pin_model, pin_data, pin_q)
            pin.updateFramePlacements(pin_model, pin_data)
            T_urdf = _relative(
                pin_data.oMf[base_fid].homogeneous,
                pin_data.oMf[tip_fid].homogeneous,
            )
            T_urdf_flange = _relative(
                pin_data.oMf[base_fid].homogeneous,
                pin_data.oMf[flange_fid].homogeneous,
            )

            mj_data.qpos[mj_qadr] = q
            mujoco.mj_forward(mj_model, mj_data)
            T_mj_tip = np.eye(4)
            T_mj_tip[:3, :3] = mj_data.xmat[mj_tip_id].reshape(3, 3)
            T_mj_tip[:3, 3] = mj_data.xpos[mj_tip_id]
            T_mjcf = _relative(pin_data.oMf[base_fid].homogeneous, T_mj_tip)
            T_mjcf_flange = compose(T_mjcf, T_mj_tip_flange)
            T_solver = nero_fk(q, params)
            T_solver_flange = compose(T_solver, T_tip_flange)
            for bucket, predicted, actual in (
                (errors_tip_urdf, T_solver, T_urdf),
                (errors_tip_mjcf, T_solver, T_mjcf),
                (errors_flange_urdf, T_solver_flange, T_urdf_flange),
                (errors_flange_mjcf, T_solver_flange, T_mjcf_flange),
            ):
                dp, dr = _errors(predicted, actual)
                bucket[0].append(dp)
                bucket[1].append(dr)
        report["sides"][side] = {
            "T_link7_from_gripper_flange": T_tip_flange.tolist(),
            "urdf_mjcf_fixed_flange_transform_max_position_m": flange_model_error[0],
            "urdf_mjcf_fixed_flange_transform_max_rotation_rad": flange_model_error[1],
            "solver_tip_vs_urdf_max_position_m": max(errors_tip_urdf[0]),
            "solver_tip_vs_urdf_max_rotation_rad": max(errors_tip_urdf[1]),
            "solver_tip_vs_mjcf_max_position_m": max(errors_tip_mjcf[0]),
            "solver_tip_vs_mjcf_max_rotation_rad": max(errors_tip_mjcf[1]),
            "solver_tip_plus_urdf_tool_vs_urdf_flange_max_position_m": max(errors_flange_urdf[0]),
            "solver_tip_plus_urdf_tool_vs_urdf_flange_max_rotation_rad": max(errors_flange_urdf[1]),
            "solver_tip_plus_urdf_tool_vs_mjcf_flange_max_position_m": max(errors_flange_mjcf[0]),
            "solver_tip_plus_urdf_tool_vs_mjcf_flange_max_rotation_rad": max(errors_flange_mjcf[1]),
        }
    for side, values in report["sides"].items():
        for key, limit in (
            ("solver_tip_vs_urdf_max_position_m", 5e-5),
            ("solver_tip_vs_urdf_max_rotation_rad", 1e-4),
            ("solver_tip_vs_mjcf_max_position_m", 5e-5),
            ("solver_tip_vs_mjcf_max_rotation_rad", 1e-4),
            ("solver_tip_plus_urdf_tool_vs_urdf_flange_max_position_m", 5e-5),
            ("solver_tip_plus_urdf_tool_vs_urdf_flange_max_rotation_rad", 1e-4),
            ("solver_tip_plus_urdf_tool_vs_mjcf_flange_max_position_m", 5e-5),
            ("solver_tip_plus_urdf_tool_vs_mjcf_flange_max_rotation_rad", 1e-4),
        ):
            if values[key] > limit:
                raise SystemExit(f"FAIL {side} {key}={values[key]:.8g} > {limit:g}")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
