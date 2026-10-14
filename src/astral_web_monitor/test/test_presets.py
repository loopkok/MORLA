"""Presets.yaml is the web Start dropdown — left-only must not launch the right arm."""

from __future__ import annotations

from pathlib import Path

import yaml


def _presets() -> list[dict]:
    path = Path(__file__).resolve().parents[1] / "config" / "presets.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["presets"]


def test_left_arm_gripper_preset_skips_right_arm() -> None:
    names = [p["name"] for p in _presets()]
    assert "Left arm + left gripper (no right arm)" in names
    p = next(x for x in _presets() if x["name"].startswith("Left arm + left gripper"))
    assert p["package"] == "astral_teleop"
    assert p["launch"] == "full_teleop.launch.py"
    assert p["args"]["arm_side"] == "left"
    assert p["args"]["with_gripper"] == "true"
    assert p["args"]["right_hand_source"] == "none"
    assert p["args"]["with_arm_driver"] == "true"
    assert p["args"]["with_hand_driver"] == "false"
