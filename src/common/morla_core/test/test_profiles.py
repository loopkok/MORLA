from __future__ import annotations

import copy
import json
import math
import sys
import time
from pathlib import Path

import pytest

from morla_core.joint_mapping import JointMappingError, validate_joint_positions
from morla_core.ownership import PolicyLease
from morla_core.profile import (
    ProfileRegistry,
    ProfileValidationError,
    validate_model_capacity,
    validate_profile,
)
from morla_core.resource_locator import ResourceLookupError, resolve_source_resource
from morla_core.schema import FrozenSchema, upgrade_v1_schema


MORLA_ROOT = Path(__file__).resolve().parents[4]
INDEX = MORLA_ROOT / "src/common/morla_bringup/config/robots.yaml"
EXPECTED_DIMS = {
    "astral_left_gripper": 8,
    "astral_left_gripper_3cam": 8,
    "astral_right_gripper": 8,
    "astral_dual_gripper": 16,
    "astral_dual_gripper_wuji": 35,
    "astral_left_wuji": 27,
    "astral_right_wuji": 27,
    "astral_dual_wuji": 54,
    "nero_left_xhand": 19,
    "nero_right_xhand": 19,
    "nero_dual_xhand": 38,
    "nero_dual_arms": 14,
}


def test_all_registered_profiles_have_expected_layout_and_primary_params():
    registry = ProfileRegistry.from_index(INDEX)
    assert set(registry.ids()) == set(EXPECTED_DIMS)
    package_dirs = {}
    for manifest in (MORLA_ROOT / "src").rglob("package.xml"):
        import xml.etree.ElementTree as ET

        package_dirs[ET.parse(manifest).getroot().findtext("name")] = manifest.parent

    for profile in registry.validate_all():
        schema = profile.frozen_schema
        assert schema.state_dim == schema.action_dim == EXPECTED_DIMS[profile.profile_id]
        assert schema.state_dim == len([n for b in schema.state_blocks for n in b.feature_names])
        assert schema.layout_hash and schema.schema_hash
        for component in profile.values["components"]:
            resource = package_dirs[component["package"]] / component["params_file"]
            assert resource.is_file(), f"missing {component['component_id']} params: {resource}"


def test_profile_materializes_shared_resolved_and_frozen_json(tmp_path):
    profile = ProfileRegistry.from_index(INDEX).load("nero_dual_xhand")
    resolved_path, schema_path = profile.write_resolved(tmp_path / profile.profile_id)
    resolved = json.loads(resolved_path.read_text(encoding="utf-8"))
    frozen = json.loads(schema_path.read_text(encoding="utf-8"))
    assert resolved["schema_hash"] == frozen["schema_hash"]
    assert resolved["layout_hash"] == frozen["layout_hash"]
    assert frozen["action_dim"] == 38


def test_nero_dual_xhand_has_fixed_left_arm_right_arm_left_hand_right_hand_order():
    profile = ProfileRegistry.from_index(INDEX).load("nero_dual_xhand")
    schema = profile.frozen_schema
    assert [b.block_id for b in schema.state_blocks] == [
        "left_arm", "right_arm", "left_ee", "right_ee"
    ]
    assert [b.state_indices for b in schema.state_blocks] == [
        tuple(range(0, 7)), tuple(range(7, 14)),
        tuple(range(14, 26)), tuple(range(26, 38)),
    ]
    assert schema.state_blocks[2].physical_joint_names[0] == "thumb_bend_joint"
    assert schema.state_blocks[3].physical_joint_names[-1] == "pinky_joint2"
    assert {s.stream_key for s in schema.streams if s.source_kind == "sensor"} == {
        "left_hand_tactile", "right_hand_tactile"
    }


def test_frozen_schema_roundtrip_detects_unknown_fields_and_hash_changes():
    original = ProfileRegistry.from_index(INDEX).load("astral_left_gripper").frozen_schema
    payload = json.loads(original.to_json())
    parsed = FrozenSchema.from_dict(payload)
    assert parsed.schema_hash == original.schema_hash
    assert parsed.layout_hash == original.layout_hash

    tampered = copy.deepcopy(payload)
    tampered["streams"][0]["unit"] = "degree"
    with pytest.raises(ValueError, match="layout_hash"):
        FrozenSchema.from_dict(tampered)
    unknown = copy.deepcopy(payload)
    unknown["unrecognized_contract"] = True
    with pytest.raises(ValueError, match="unknown"):
        FrozenSchema.from_dict(unknown)
    with pytest.raises(ValueError, match="unsupported schema_version"):
        FrozenSchema.from_dict({**payload, "schema_version": 99})


def test_v1_astral_upgrade_preserves_feature_names_and_explicit_semantics():
    upgraded = upgrade_v1_schema({
        "schema_version": 1,
        "profile_id": "historical_left_gripper",
        "arms": ["left"],
        "end_effector_left": "gripper",
        "end_effector_right": "none",
        "cameras": ["base", "left_wrist"],
        "action_source": "command",
        "state_dim": 8,
        "operator_note": "preserved metadata",
    })
    assert upgraded.state_dim == upgraded.action_dim == 8
    assert upgraded.state_blocks[0].feature_names == tuple(f"left_arm_{i}" for i in range(7))
    assert upgraded.state_blocks[1].feature_names == ("left_ee_0",)
    assert all(block.semantics == "command" for block in upgraded.action_blocks)
    assert upgraded.metadata["legacy_metadata"]["operator_note"] == "preserved metadata"
    assert upgraded.state_blocks[0].physical_joint_names == (
        "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_roll",
        "left_elbow_pitch", "left_forearm_roll", "left_wrist_pitch", "left_wrist_roll",
    )


def test_profile_rejects_topic_type_and_unit_conflicts_and_disabled_writer():
    source = ProfileRegistry.from_index(INDEX).load("nero_left_xhand").values
    bad_type = copy.deepcopy(source)
    bad_type["streams"][1]["topic"] = bad_type["streams"][0]["topic"]
    bad_type["streams"][1]["message_type"] = "std_msgs/msg/String"
    with pytest.raises(ProfileValidationError, match="conflicting types"):
        validate_profile(bad_type)

    bad_unit = copy.deepcopy(source)
    bad_unit["streams"][0]["unit"] = "degree"
    with pytest.raises(ProfileValidationError, match="unit"):
        validate_profile(bad_unit)

    disabled = copy.deepcopy(source)
    disabled["components"][0]["enabled"] = False
    with pytest.raises(ProfileValidationError, match="no enabled component"):
        validate_profile(disabled)


def test_model_capacity_is_a_backend_rule_not_a_schema_dimension_limit():
    registry = ProfileRegistry.from_index(INDEX)
    assert registry.load("nero_left_xhand").frozen_schema.action_dim == 19
    validate_model_capacity(registry.load("nero_left_xhand"), 32)
    for name in ("astral_dual_gripper_wuji", "nero_dual_xhand", "astral_dual_wuji"):
        profile = registry.load(name)
        assert profile.frozen_schema.action_dim > 32
        with pytest.raises(ProfileValidationError, match="exceeds model capacity"):
            validate_model_capacity(profile, 32)


def test_joint_mapping_requires_exact_names_and_finite_values():
    expected = ("left_joint1", "left_joint2")
    assert validate_joint_positions(
        ("left_joint2", "left_joint1"), (0.2, 0.1), expected
    ) == (0.1, 0.2)
    assert validate_joint_positions((), (0.1, 0.2), expected, allow_positional_legacy=True) == (0.1, 0.2)
    for names, values, options in [
        (("left_joint1",), (0.1,), {}),
        (("left_joint1", "left_joint1"), (0.1, 0.2), {}),
        (("left_joint1", "left_joint2"), (0.1, math.nan), {}),
        ((), (0.1,), {"allow_positional_legacy": True}),
        (("left_joint1", "left_joint2"), (0.1, 3.2), {"limits": ((-1, 1), (-1, 1))}),
    ]:
        with pytest.raises(JointMappingError):
            validate_joint_positions(names, values, expected, **options)


def test_lease_manual_ownership_and_lazy_ros_import():
    assert "rclpy" not in sys.modules
    lease = PolicyLease(timeout=2.0)
    assert lease.manual_allowed
    now = time.time()
    assert lease.update(json.dumps({"activity": "policy", "epoch": 1, "sequence": 1, "sent_at": now}))
    assert not lease.manual_allowed
    assert lease.update(json.dumps({"activity": "human", "epoch": 1, "sequence": 2, "sent_at": time.time()}))
    assert lease.manual_allowed
    assert not lease.update(json.dumps({"activity": "human", "epoch": 1, "sequence": 1, "sent_at": time.time()}))


def test_source_resource_locator_rejects_path_traversal(tmp_path):
    package = tmp_path / "pkg"
    package.mkdir()
    (package / "config.yaml").write_text("ok", encoding="utf-8")
    assert resolve_source_resource(package, "config.yaml").read_text() == "ok"
    with pytest.raises(ResourceLookupError, match="relative"):
        resolve_source_resource(package, "../escape")
