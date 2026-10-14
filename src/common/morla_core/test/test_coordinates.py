from __future__ import annotations

import numpy as np
import pytest

from morla_core.coordinates import (
    CoordinateError,
    UNITY_TO_QUEST3_CANONICAL,
    change_basis_rotation,
    change_basis_vector,
    compose,
    inverse,
    rpy_to_matrix,
    solver_tip_to_tcp,
    tcp_to_solver_tip,
    transform_from_xyz_rpy,
    validate_transform,
)
from morla_core.coordinate_contract import build_coordinate_contract
from morla_core.profile import ProfileRegistry, validate_profile
from morla_core.resource_locator import resolve_robot_asset


def test_unity_reflection_is_only_accepted_as_a_basis_change():
    assert np.linalg.det(UNITY_TO_QUEST3_CANONICAL) == pytest.approx(-1.0)
    np.testing.assert_allclose(
        change_basis_vector(UNITY_TO_QUEST3_CANONICAL, [1, 2, 3]), [-1, -3, 2]
    )
    orient = rpy_to_matrix([0.2, -0.3, 0.4])
    converted = change_basis_rotation(UNITY_TO_QUEST3_CANONICAL, orient)
    np.testing.assert_allclose(converted.T @ converted, np.eye(3), atol=1e-12)
    assert np.linalg.det(converted) == pytest.approx(1.0)
    bad = np.eye(4)
    bad[:3, :3] = UNITY_TO_QUEST3_CANONICAL
    with pytest.raises(CoordinateError, match="determinant"):
        validate_transform(bad)


def test_transform_composition_inverse_and_urdf_rpy_convention():
    a = transform_from_xyz_rpy([0.1, -0.2, 0.3], [0.4, 0.2, -0.3])
    b = transform_from_xyz_rpy([-0.2, 0.1, 0.05], [-0.1, 0.3, 0.5])
    identity = compose(a, b, inverse(b), inverse(a))
    np.testing.assert_allclose(identity, np.eye(4), atol=1e-12)
    legacy = rpy_to_matrix([0, np.pi / 2, -np.pi / 2])
    zero_offset = legacy @ rpy_to_matrix([0, 0, -np.pi / 2])
    vendor_origin = rpy_to_matrix([np.pi / 2, np.pi, 0])
    # Compare rotations rather than non-unique Euler strings.
    np.testing.assert_allclose(zero_offset, vendor_origin, atol=1e-12)


def test_nonzero_tool_anchor_and_inverse_preserve_zero_motion():
    T_solver = transform_from_xyz_rpy([0.25, -0.13, 0.42], [0.1, -0.3, 0.7])
    T_solver_tcp = transform_from_xyz_rpy([0.06, 0.01, -0.0235], [-0.4, 0.2, 0.8])
    anchor_tcp = solver_tip_to_tcp(T_solver, T_solver_tcp)
    recovered_solver_target = tcp_to_solver_tip(anchor_tcp, T_solver_tcp)
    np.testing.assert_allclose(recovered_solver_target, T_solver, atol=1e-12)

    # Rotating about the TCP changes the required solver-tip translation.
    moved_tcp = anchor_tcp.copy()
    moved_tcp[:3, :3] = anchor_tcp[:3, :3] @ rpy_to_matrix([0.0, 0.0, 0.3])
    moved_solver = tcp_to_solver_tip(moved_tcp, T_solver_tcp)
    assert np.linalg.norm(moved_solver[:3, 3] - T_solver[:3, 3]) > 1e-3


def test_profile_contract_is_explicit_hashable_and_sensitive_to_alignment():
    profile = ProfileRegistry.from_index().load("nero_dual_xhand")
    contract = build_coordinate_contract(profile.values)
    assert set(contract.arms) == {"left", "right"}
    assert contract.arm("left")["tool"]["id"] == "legacy_solver_tip"
    assert contract.arm("left")["tool"]["frame"] == "left_link7"
    assert len(contract.kinematics_hash) == 64

    changed = dict(profile.values)
    changed["coordinate_contract"] = {
        "version": 1,
        "input_basis_id": contract.input_basis_id,
        "input_frame_policy": contract.input_frame_policy,
        "robot_body_frame": contract.robot_body_frame,
        "arms": {side: dict(row) for side, row in contract.arms.items()},
    }
    changed["coordinate_contract"]["arms"]["left"]["input_rotation_to_solver_base"] = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    changed_contract = build_coordinate_contract(changed)
    assert changed_contract.kinematics_hash != contract.kinematics_hash

    resolved = validate_profile(changed)
    assert resolved.frozen_schema.kinematics_hash == changed_contract.kinematics_hash
    payload = resolved.frozen_schema.to_dict()
    assert payload["kinematics_hash"] == changed_contract.kinematics_hash


def test_all_registered_arm_profiles_resolve_proper_rotations_from_their_urdf():
    registry = ProfileRegistry.from_index()
    checked = 0
    for profile_id in registry.ids():
        profile = registry.load(profile_id)
        contract = build_coordinate_contract(profile.values)
        for component in profile.values["components"]:
            if not component.get("enabled") or component.get("type") not in {"arm", "nero_arm"}:
                continue
            model = resolve_robot_asset(str(component["description_model_id"]))
            alignment = contract.resolve_arm_input_rotation(component["side"], str(model))
            assert np.all(np.isfinite(alignment))
            np.testing.assert_allclose(alignment.T @ alignment, np.eye(3), atol=1e-7)
            assert np.linalg.det(alignment) == pytest.approx(1.0, abs=1e-7)
            checked += 1
    assert checked == 17  # eight Astral profiles and four Nero profiles by active arm side


def test_tool_frames_are_resolved_from_urdf_fixed_chain_not_profile_offsets():
    root = ProfileRegistry.from_index().load("nero_dual_xhand").values
    urdf = resolve_robot_asset("nero_left_arm")
    contract = build_coordinate_contract(root)
    active_tool = contract.resolve_arm_tool_transform("left", str(urdf))
    np.testing.assert_allclose(active_tool, np.eye(4), atol=1e-12)

    configured_flange = dict(root)
    configured_flange["coordinate_contract"] = {
        **root["coordinate_contract"],
        "arms": {
            **root["coordinate_contract"]["arms"],
            "left": {
                **root["coordinate_contract"]["arms"]["left"],
                "tool": {
                    "id": "nero_arm_flange",
                    "frame": "left_gripper_flange",
                    "source": "urdf_named_frame",
                    "calibration_status": "physical_tool_unverified",
                },
            },
        },
    }
    flange = build_coordinate_contract(configured_flange).resolve_arm_tool_transform(
        "left", str(urdf)
    )
    np.testing.assert_allclose(flange[:3, 3], [0.032, 0.0, -0.0235], atol=1e-12)
    np.testing.assert_allclose(
        flange[:3, :3], rpy_to_matrix([-1.5708, 0.0, -1.5708]), atol=1e-12
    )

    configured = dict(root)
    configured["coordinate_contract"] = {
        **root["coordinate_contract"],
        "arms": {
            **root["coordinate_contract"]["arms"],
            "left": {
                **root["coordinate_contract"]["arms"]["left"],
                "tool": {
                    "id": "xhand_mount_frame",
                    "frame": "left_hand_link",
                    "source": "urdf_named_frame",
                    "calibration_status": "physical_tool_unverified",
                },
            },
        },
    }
    derived = build_coordinate_contract(configured).resolve_arm_tool_transform(
        "left", str(urdf)
    )
    np.testing.assert_allclose(derived[:3, 3], [0.06, 0.0, -0.0235], atol=5e-5)
    bad_configured = dict(root)
    bad_configured["coordinate_contract"] = {
        **root["coordinate_contract"],
        "arms": {
            **root["coordinate_contract"]["arms"],
            "left": {
                **root["coordinate_contract"]["arms"]["left"],
                "tool": {"id": "movable", "frame": "left_link1"},
            },
        },
    }
    with pytest.raises(CoordinateError, match="cannot resolve fixed tool chain"):
        build_coordinate_contract(bad_configured).resolve_arm_tool_transform("left", str(urdf))


def test_every_nero_profile_keeps_link7_as_active_arm_end():
    registry = ProfileRegistry.from_index()
    checked = 0
    for profile_id in registry.ids():
        profile = registry.load(profile_id)
        if profile.robot_id != "nero":
            continue
        contract = build_coordinate_contract(profile.values)
        for side, arm in contract.arms.items():
            assert arm["solver_tip_frame"] == f"{side}_link7"
            assert arm["physical_flange_frame"] == f"{side}_gripper_flange"
            assert arm["tool"]["id"] == "legacy_solver_tip"
            assert arm["tool"]["frame"] == arm["solver_tip_frame"]
            model = resolve_robot_asset(str(arm["description_model_id"]))
            transform = contract.resolve_arm_tool_transform(side, str(model))
            np.testing.assert_allclose(transform, np.eye(4), atol=1e-12)
            checked += 1
    assert checked == 6  # four Nero Profiles: two dual-arm and two single-arm
