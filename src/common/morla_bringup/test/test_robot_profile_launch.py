from types import SimpleNamespace

import pytest

from morla_bringup.robot_profile_launch import select_robot_stack


def _profile(robot_id, profile_id, components):
    return SimpleNamespace(
        robot_id=robot_id,
        profile_id=profile_id,
        values={"components": components},
    )


def test_profile_selects_astral_and_nero_launches():
    astral = _profile("astral", "astral_left_gripper", [
        {"enabled": True, "type": "arm", "side": "left"},
        {"enabled": True, "type": "gripper", "side": "left"},
    ])
    nero = _profile("nero", "nero_dual_xhand", [
        {"enabled": True, "type": "nero_arm", "side": "left"},
        {"enabled": True, "type": "nero_arm", "side": "right"},
        {"enabled": True, "type": "xhand", "side": "left"},
        {"enabled": True, "type": "xhand", "side": "right"},
    ])

    astral_stack = select_robot_stack(astral, "sim")
    assert (astral_stack.package, astral_stack.launch_file) == (
        "astral_mujoco_sim", "astral_sim_pipeline.launch.py"
    )
    assert astral_stack.arguments["control_profile_id"] == "astral_left_gripper"
    assert astral_stack.arguments["gripper_sides"] == "left"

    nero_stack = select_robot_stack(nero, "sim")
    assert (nero_stack.package, nero_stack.launch_file) == (
        "morla_bringup", "nero_xhand_stack.launch.py"
    )
    assert nero_stack.arguments["profile_id"] == "nero_dual_xhand"
    assert nero_stack.arguments["mode"] == "sim"


def test_launch_dispatch_rejects_mode_backend_mismatch_and_unsupported_real_layout():
    astral_left_wuji = _profile("astral", "astral_left_wuji", [
        {"enabled": True, "type": "arm", "side": "left"},
        {"enabled": True, "type": "wuji_hand", "side": "left"},
    ])
    with pytest.raises(ValueError, match="requires sdk_backend=fake"):
        select_robot_stack(astral_left_wuji, "sim", "hardware")
    with pytest.raises(ValueError, match="one right-side Wuji"):
        select_robot_stack(astral_left_wuji, "real", "hardware")


def test_nero_real_dispatch_requires_hardware_backend():
    nero = _profile("nero", "nero_left_xhand", [
        {"enabled": True, "type": "nero_arm", "side": "left"},
        {"enabled": True, "type": "xhand", "side": "left"},
    ])
    with pytest.raises(ValueError, match="requires sdk_backend=hardware"):
        select_robot_stack(nero, "real", "fake")
    assert select_robot_stack(nero, "real", "hardware").arguments["sdk_backend"] == "hardware"
