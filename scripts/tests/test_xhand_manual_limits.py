"""Check that every active command/model boundary follows the XHand manual."""
import importlib.util
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
HAND = ROOT / "src/robots/xhand/morla_xhand_hand"
SPEC = json.loads((HAND / "config/xhand1_manual_limits.json").read_text())
NAMES = [j["name"] for j in SPEC["joint_limits"]]
LIMITS = [[math.radians(j["lower"]), math.radians(j["upper"])] for j in SPEC["joint_limits"]]


def assert_limits(actual):
    assert len(actual) == 12
    for actual_pair, expected_pair in zip(actual, LIMITS):
        assert actual_pair == pytest.approx(expected_pair, rel=0, abs=1e-12)


def test_reference_matches_manual_joint_ids_and_degree_ranges():
    assert [j["id"] for j in SPEC["joint_limits"]] == list(range(12))
    assert [(j["lower"], j["upper"]) for j in SPEC["joint_limits"]] == [
        (0, 90), (-60, 90), (0, 90), (-5, 17), *[(0, 110)] * 8,
    ]
    assert (ROOT / SPEC["source_document"]).is_file()


@pytest.mark.parametrize("side", ["left", "right"])
def test_component_and_adapter_use_manual_native_limits(side):
    c = yaml.safe_load((HAND / f"config/components/xhand_{side}.yaml").read_text())
    assert c["joint_names"] == NAMES
    assert_limits(c["limits"])
    a = yaml.safe_load((HAND / f"config/xhand_adapter_{side}.yaml").read_text())
    p = next(iter(a.values()))["ros__parameters"]
    assert_limits(list(zip(p["joint_lower_limits"], p["joint_upper_limits"])))


@pytest.mark.parametrize("profile", ["nero_left_xhand", "nero_right_xhand", "nero_dual_xhand"])
def test_profiles_keep_all_hand_command_and_schema_limits_consistent(profile):
    p = yaml.safe_load((ROOT / f"src/robots/nero/morla_nero_robot/config/profiles/{profile}.yaml").read_text())
    count = 0
    def visit(value):
        nonlocal count
        if isinstance(value, dict):
            if (value.get("joint_names") == NAMES or value.get("physical_joint_names") == NAMES
                    or value.get("block_id") in ("left_ee", "right_ee")) and "limits" in value:
                assert_limits(value["limits"])
                count += 1
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)
    visit(p)
    assert count >= (6 if profile == "nero_dual_xhand" else 3)


@pytest.mark.parametrize("side", ["left", "right"])
def test_standalone_and_combined_urdf_match_manual_without_axis_changes(side):
    for relative in [f"urdf/xhand/xhand_{side}.urdf", "urdf/nero/xhand_nero_description.urdf"]:
        root = ET.parse(ROOT / "src/robot_description" / relative).getroot()
        actual = []
        for name in NAMES:
            joint = root.find(f"./joint[@name='{side}_hand_{name}']")
            limit = joint.find("limit")
            actual.append([float(limit.attrib[k]) for k in ("lower", "upper")])
        assert_limits(actual)


def test_external_piano_template_uses_same_manual_contract():
    project = Path(os.environ.get("MORLA_PIANO_PROJECT", str(ROOT.parent / "MORLA-piano")))
    assert project.is_dir(), "Set MORLA_PIANO_PROJECT to the external piano project"
    config = json.loads((project / "config/piano.placeholder.json").read_text())
    for hand in config["hands"].values():
        assert_limits(hand["hand_limits_rad"])


def test_adapter_accepts_endpoints_and_rejects_each_outside_target():
    path = ROOT / "src/robots/xhand/xhand_adapter/xhand_adapter/core.py"
    spec = importlib.util.spec_from_file_location("xhand_manual_contract_core", path)
    core = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(core)
    assert list(core.HAND_JOINT_NAMES) == NAMES
    lo, hi = zip(*LIMITS)
    for endpoint in (lo, hi):
        assert core.validate_joint_limits(endpoint, lo, hi) == tuple(endpoint)
    # Includes formerly accepted unsafe thumb targets and now-legal index 17°.
    for i in range(12):
        for value in (lo[i] - 1e-6, hi[i] + 1e-6):
            q = [0.] * 12
            q[i] = value
            with pytest.raises(core.XHandContractError, match="exceeds"):
                core.validate_joint_limits(q, lo, hi)
