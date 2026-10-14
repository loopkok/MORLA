"""遥操诊断日志开关：预设注入 log_dir 根目录（launch_manager.preset_with_teleop_log）。"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from astral_web_monitor.launch_manager import (  # noqa: E402
    Preset,
    preset_with_teleop_log,
)


def _preset(package: str, name: str = "p") -> Preset:
    return Preset(name=name, package=package, launch="x.launch.py", args={"a": "1"})


def test_injects_for_astral_teleop() -> None:
    p = preset_with_teleop_log(_preset("astral_teleop"), "/logs/teleop")
    assert p.args["log_dir"] == "/logs/teleop"
    assert p.args["a"] == "1"  # 原参数保留
    assert _preset("astral_teleop").args.get("log_dir") is None  # 源未改


def test_injects_for_astral_arm_teleop() -> None:
    p = preset_with_teleop_log(_preset("astral_arm_teleop"), "/logs/teleop")
    assert p.args["log_dir"] == "/logs/teleop"


def test_skips_non_whitelisted_package() -> None:
    p = preset_with_teleop_log(_preset("astral_mujoco_sim"), "/logs/teleop")
    assert p.args.get("log_dir") is None
    assert p.args["a"] == "1"


def test_empty_path_skips() -> None:
    p = preset_with_teleop_log(_preset("astral_teleop"), "")
    assert p.args.get("log_dir") is None
