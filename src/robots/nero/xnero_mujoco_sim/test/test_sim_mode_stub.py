"""The migrated simulator only exposes standard MORLA arm/native hand IO."""
from pathlib import Path

from xnero_mujoco_sim.hand_map import ARM_STATE_JOINT_NAMES


def test_simulator_does_not_reintroduce_legacy_teleop_driver():
    source = Path(__file__).resolve().parents[1] / "xnero_mujoco_sim" / "sim_node.py"
    text = source.read_text(encoding="utf-8")
    assert "nero_quest_teleop" not in text
    assert "pyAgxArm" not in text
    assert 'f"/{side}_arm/joint_commands"' in text
    assert 'f"/{side}_arm/joint_states"' in text
    assert 'f"/{side}_hand/driver/xhand_command"' in text
    assert 'f"/{side}_hand/driver/xhand_state"' in text


def test_arm_state_joint_names_are_fully_qualified():
    assert ARM_STATE_JOINT_NAMES["left"] == [f"left_joint{i}" for i in range(1, 8)]
    assert ARM_STATE_JOINT_NAMES["right"] == [f"right_joint{i}" for i in range(1, 8)]
