"""Profile selection keeps every web lane on one validated robot layout."""

from __future__ import annotations

import asyncio

import pytest
from fastapi import HTTPException

from morla_web_manager import web_server
from morla_web_manager.schemas import ProfileSelectRequest


def test_profile_launch_builder_selects_n_values_for_n_modes():
    nero = web_server._profile_launch_preset("nero_left_xhand", "sim")
    assert nero.package == "morla_bringup"
    assert nero.launch == "robot.launch.py"
    assert nero.args["profile_id"] == "nero_left_xhand"
    assert nero.args["mode"] == "sim"

    astral = web_server._profile_launch_preset("astral_dual_gripper", "sim")
    assert astral.package == "morla_bringup"
    assert astral.launch == "robot.launch.py"
    assert astral.args["profile_id"] == "astral_dual_gripper"
    assert astral.args["mode"] == "sim"

    astral_wuji = web_server._profile_launch_preset("astral_dual_gripper_wuji", "sim")
    assert astral_wuji.package == "morla_bringup"
    assert astral_wuji.args["profile_id"] == "astral_dual_gripper_wuji"
    assert astral_wuji.launch == "robot.launch.py"


def test_profile_catalog_reports_action_dimensions_and_supported_launch_modes():
    result = asyncio.run(web_server.list_robot_profiles())
    data = result.data
    by_id = {row["profile_id"]: row for row in data["profiles"]}
    assert by_id["nero_left_xhand"]["action_dim"] == 19
    assert by_id["nero_left_xhand"]["hands"] == [
        {"side": "left", "type": "xhand", "dim": 12}
    ]
    assert by_id["astral_dual_gripper"]["launch_modes"] == ["sim", "real"]
    assert by_id["astral_dual_gripper_wuji"]["launch_modes"] == ["sim", "real"]


def test_profile_change_is_blocked_by_cli_writer(monkeypatch):
    original = (web_server._selected_profile_id, web_server._selected_operating_mode)
    web_server._selected_profile_id = "astral_dual_gripper_wuji"
    web_server._selected_operating_mode = "sim"

    class Node:
        def snapshot(self):
            return {"control": None, "data_collect": None, "infer": None}

        def robot_command_publishers(self):
            return {"/left_arm/joint_commands": 1}

    monkeypatch.setattr(web_server, "get_node", lambda: Node())
    try:
        with pytest.raises(HTTPException) as exc:
            asyncio.run(web_server.select_robot_profile(
                ProfileSelectRequest(profile_id="nero_left_xhand", operating_mode="sim")
            ))
        assert exc.value.status_code == 409
        assert "robot command writers are active" in exc.value.detail
        assert web_server._selected_profile_id == "astral_dual_gripper_wuji"
    finally:
        web_server._selected_profile_id, web_server._selected_operating_mode = original


def test_collector_lane_inherits_the_selected_profile(monkeypatch):
    original = (web_server._selected_profile_id, web_server._selected_operating_mode)
    web_server._selected_profile_id = "nero_dual_xhand"
    web_server._selected_operating_mode = "sim"
    monkeypatch.setattr(web_server, "get_node", lambda: None)
    try:
        preset = web_server._collect_preset()
        assert preset is not None
        assert preset.args["profile_id"] == "nero_dual_xhand"
        assert preset.args["runtime_mode"] == "sim"
        assert preset.args["keyboard"] == "false"
        assert preset.args["vr_control"] == "false"
    finally:
        web_server._selected_profile_id, web_server._selected_operating_mode = original


def test_live_cli_collector_profile_mismatch_blocks_launch_even_without_selection_change(monkeypatch):
    original = (web_server._selected_profile_id, web_server._selected_operating_mode)
    web_server._selected_profile_id = "nero_left_xhand"
    web_server._selected_operating_mode = "sim"

    class Node:
        def snapshot(self):
            return {
                "control": None,
                "data_collect": {
                    "stale": False, "node_count": 1,
                    "profile_id": "astral_dual_gripper", "runtime_mode": "sim",
                },
                "infer": None,
            }

        def robot_command_publishers(self):
            return {}

    monkeypatch.setattr(web_server, "get_node", lambda: Node())
    try:
        blockers = web_server._profile_switch_blockers("nero_left_xhand", "sim")
        assert any("collector is configured for astral_dual_gripper" in item for item in blockers)
    finally:
        web_server._selected_profile_id, web_server._selected_operating_mode = original


def test_policy_launch_args_carry_the_selected_profile():
    args = web_server.config.policy_launch_args({
        "profile_id": "nero_dual_xhand", "backend_type": "stub",
    })
    assert args["profile_id"] == "nero_dual_xhand"
    assert args["keyboard"] == "false"
