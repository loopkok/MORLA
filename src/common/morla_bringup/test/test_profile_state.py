from pathlib import Path

import numpy as np
import pytest
import yaml

from morla_bringup.profile_state import profile_state_sources, remap_joint_state_fields
from morla_core.profile import ProfileRegistry


def _manifest():
    root = Path(__file__).resolve().parents[4]
    return yaml.safe_load(
        (root / "src/robot_description/config/asset_manifest.yaml").read_text()
    )


def test_nero_measured_arm_and_hand_states_map_to_whole_robot_urdf():
    profile = ProfileRegistry.from_index().load("nero_dual_xhand")
    sources = profile_state_sources(profile.values, _manifest())
    assert {source["component_id"] for source in sources} == {
        "left_arm", "right_arm", "left_ee", "right_ee"
    }
    left_hand = next(source for source in sources if source["component_id"] == "left_ee")
    assert left_hand["topic"] == "/left_hand/joint_states"
    assert left_hand["source_joint_names"][0] == "thumb_bend_joint"
    assert left_hand["urdf_joint_names"][0] == "left_hand_thumb_bend_joint"
    left_arm = next(source for source in sources if source["component_id"] == "left_arm")
    assert left_arm["source_joint_names"] == left_arm["urdf_joint_names"]


def test_astral_scalar_gripper_command_is_not_exposed_as_measured_tf_state():
    profile = ProfileRegistry.from_index().load("astral_dual_gripper")
    sources = profile_state_sources(profile.values, _manifest())
    assert {source["component_id"] for source in sources} == {"left_arm", "right_arm"}
    assert all(source["topic"].endswith("/joint_states") for source in sources)


def test_joint_state_remapping_reorders_measurement_and_rejects_missing_joints():
    result = remap_joint_state_fields(
        ["joint_b", "joint_a"], [2.0, 1.0], [0.2, 0.1], [],
        ["joint_a", "joint_b"], ["urdf_a", "urdf_b"],
    )
    assert result == {
        "name": ["urdf_a", "urdf_b"],
        "position": [1.0, 2.0],
        "velocity": [0.1, 0.2],
        "effort": [],
    }
    with pytest.raises(ValueError, match="missing configured joints"):
        remap_joint_state_fields(
            ["joint_b"], [2.0], [], [], ["joint_a", "joint_b"], ["urdf_a", "urdf_b"]
        )
    assert np.isfinite(result["position"]).all()
