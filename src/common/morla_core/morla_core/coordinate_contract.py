"""Validated, hashable coordinate/kinematics contracts derived from Profiles."""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Any

import numpy as np

from .coordinates import (
    CoordinateError,
    fixed_transform_between_urdf,
    validate_rotation,
)


def _canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def _matrix9(value: Any, label: str) -> list[float]:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape == (9,):
        matrix = matrix.reshape(3, 3)
    validate_rotation(matrix)
    return [float(x) for x in matrix.reshape(-1)]


@dataclass(frozen=True)
class CoordinateContract:
    """Effective input and kinematics identity for one enabled robot Profile."""

    version: int
    robot_id: str
    description_revision: str
    input_basis_id: str
    input_frame_policy: str
    robot_body_frame: str
    input_rotation_to_robot_body: list[float] | None
    input_rotation_source: str
    arms: dict[str, dict[str, Any]]
    kinematics_hash: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "robot_id": self.robot_id,
            "description_revision": self.description_revision,
            "input_basis_id": self.input_basis_id,
            "input_frame_policy": self.input_frame_policy,
            "robot_body_frame": self.robot_body_frame,
            "input_rotation_to_robot_body": self.input_rotation_to_robot_body,
            "input_rotation_source": self.input_rotation_source,
            "arms": self.arms,
            "kinematics_hash": self.kinematics_hash,
        }

    def arm(self, side: str) -> dict[str, Any]:
        try:
            return dict(self.arms[side])
        except KeyError as exc:
            raise CoordinateError(f"Profile has no enabled {side!r} arm coordinate contract") from exc

    def resolve_arm_input_rotation(self, side: str, urdf_path: str) -> np.ndarray:
        """Resolve input orientation into a solver base using the model mount."""
        arm = self.arm(side)
        direct = arm.get("input_rotation_to_solver_base")
        if direct is not None:
            return np.asarray(direct, dtype=float).reshape(3, 3)
        if self.input_rotation_to_robot_body is None:
            raise CoordinateError("contract has neither a common nor per-arm input alignment")
        T_body_base = fixed_transform_between_urdf(
            urdf_path, self.robot_body_frame, str(arm["solver_base_frame"])
        )
        R_base_body = T_body_base[:3, :3].T
        R_body_input = np.asarray(self.input_rotation_to_robot_body, dtype=float).reshape(3, 3)
        return validate_rotation(R_base_body @ R_body_input)

    def resolve_arm_tool_transform(self, side: str, urdf_path: str) -> np.ndarray:
        """Derive ``T_solver_tip_from_tool`` from the named URDF fixed chain.

        This deliberately fails if the selected tool is not connected to the
        solver tip by fixed joints. SDK flange semantics and physical tool
        calibration remain separate evidence gates in the tool declaration.
        """
        arm = self.arm(side)
        tool = arm["tool"]
        if (self.robot_id == "astral"
                and arm["solver_adapter_id"] == "astral_analytic_dh_legacy_v1"
                and str(tool["frame"]) != str(arm["solver_tip_frame"])):
            raise CoordinateError(
                "Astral analytic_dh does not use the centralized URDF link7 frame; "
                "a non-identity URDF tool chain requires the geometric or URDF solver adapter"
            )
        try:
            return fixed_transform_between_urdf(
                urdf_path, str(arm["solver_tip_frame"]), str(tool["frame"])
            )
        except CoordinateError as exc:
            raise CoordinateError(
                f"cannot resolve fixed tool chain {arm['solver_tip_frame']!r} "
                f"to {tool['frame']!r} for {side}: {exc}"
            ) from exc


def build_coordinate_contract(profile: dict[str, Any]) -> CoordinateContract:
    """Resolve explicit coordinate declarations, with a strict legacy adapter.

    For existing Profiles without ``coordinate_contract``, the legacy Nero
    matrix is read from the enabled arm component exactly once. Astral defaults
    to identity, matching its current teleop YAML. The result is then frozen in
    the resolved Profile and shared by launch, collection and policy lanes.
    """
    robot_id = str(profile.get("robot_id", ""))
    if not robot_id:
        raise CoordinateError("coordinate contract requires robot_id")
    explicit = profile.get("coordinate_contract") or {}
    if explicit and explicit.get("version") != 1:
        raise CoordinateError("unsupported coordinate_contract version")
    basis_id = str(explicit.get("input_basis_id", "quest3_canonical_legacy_v1"))
    frame_policy = str(explicit.get("input_frame_policy", "incremental_operator_aligned"))
    body_frame = str(explicit.get("robot_body_frame", "body_link" if robot_id == "astral" else "t_bracket_base"))
    common_alignment_raw = explicit.get("input_rotation_to_robot_body")
    common_alignment = None if common_alignment_raw is None else _matrix9(
        common_alignment_raw, "input_rotation_to_robot_body"
    )
    alignment_source = str(explicit.get("input_rotation_source", "legacy_component_matrix"))
    declared_arms = explicit.get("arms", {})
    if not isinstance(declared_arms, dict):
        raise CoordinateError("coordinate_contract.arms must be a mapping")

    arms: dict[str, dict[str, Any]] = {}
    components = [c for c in profile.get("components", []) if c.get("enabled")]
    for component in components:
        if component.get("type") not in {"arm", "nero_arm"}:
            continue
        side = str(component.get("side"))
        if side not in {"left", "right"}:
            raise CoordinateError(f"invalid arm side {side!r}")
        declaration = declared_arms.get(side, {})
        if not isinstance(declaration, dict):
            raise CoordinateError(f"coordinate_contract.arms.{side} must be a mapping")
        legacy_key = "vr_to_arm_rot_after_unity_to_robot" if robot_id == "nero" else "vr_to_arm_rot"
        raw_alignment = declaration.get("input_rotation_to_solver_base")
        if raw_alignment is None and common_alignment is None:
            raw_alignment = component.get(legacy_key, np.eye(3).reshape(-1))
        alignment = None if raw_alignment is None else _matrix9(
            raw_alignment, f"arms.{side}.input_rotation_to_solver_base"
        )
        solver_base = str(declaration.get("solver_base_frame", f"{side}_base_link"))
        solver_tip = str(declaration.get("solver_tip_frame", f"{side}_link7"))
        default_tool = {
            "id": "legacy_solver_tip",
            "frame": solver_tip,
            "source": "urdf_named_frame",
            "calibration_status": "physical_tool_unverified",
        }
        tool = declaration.get("tool", default_tool)
        if not isinstance(tool, dict) or not tool.get("id") or not tool.get("frame"):
            raise CoordinateError(f"arms.{side}.tool requires id and frame")
        arms[side] = {
            "solver_base_frame": solver_base,
            "solver_tip_frame": solver_tip,
            "solver_adapter_id": str(declaration.get(
                "solver_adapter_id",
                "nero_analytic_mdh_link7_v1" if robot_id == "nero"
                else "astral_geometric_poe_link7_v1",
            )),
            "physical_flange_frame": str(declaration.get("physical_flange_frame", solver_tip)),
            "sdk_flange_frame": str(declaration.get("sdk_flange_frame", "unverified")),
            "joint_convention": str(declaration.get("joint_convention", "hardware")),
            "tool": dict(tool),
            "description_model_id": component.get("description_model_id", ""),
        }
        if alignment is not None:
            arms[side]["input_rotation_to_solver_base"] = alignment
    if not arms:
        raise CoordinateError("coordinate contract requires at least one enabled arm")

    semantic_components = []
    for c in components:
        semantic_components.append({
            "type": c.get("type"), "side": c.get("side"),
            "description_model_id": c.get("description_model_id"),
            "joint_names": c.get("joint_names", []), "limits": c.get("limits", []),
        })
    identity = {
        "contract_version": 1,
        "robot_id": robot_id,
        "description_revision": str(profile.get("description_revision", "unresolved")),
        "input_basis_id": basis_id,
        "input_frame_policy": frame_policy,
        "robot_body_frame": body_frame,
        "input_rotation_to_robot_body": common_alignment,
        "input_rotation_source": alignment_source,
        "arms": arms,
        "components": semantic_components,
    }
    digest = hashlib.sha256(_canonical(identity).encode("utf-8")).hexdigest()
    return CoordinateContract(
        version=1, robot_id=robot_id,
        description_revision=identity["description_revision"],
        input_basis_id=basis_id, input_frame_policy=frame_policy,
        robot_body_frame=body_frame,
        input_rotation_to_robot_body=common_alignment,
        input_rotation_source=alignment_source,
        arms=arms, kinematics_hash=digest,
    )
