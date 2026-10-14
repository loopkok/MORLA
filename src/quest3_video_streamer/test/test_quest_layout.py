"""Quest panel layout: D435i must not cover the left wrist stack.

Unity ``VideoPanelRenderer.ApplyFov``:

    worldWidth = 2 * distance * tan(fov_h/2) * size_multiplier

A panel at x with that width covers [x - half, x + half]. Overlap with the
wrist stack is exactly the "中间画面把左侧两个挡一半" failure.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from quest3_video_streamer.streamer_node import _scan_role_defaults

_DIST = 1.8
_CLEARANCE = 0.05  # metres of gap between panel edges


def _half_w(fov_h_deg: float, size_multiplier: float, distance: float = _DIST) -> float:
    return distance * math.tan(math.radians(fov_h_deg) * 0.5) * size_multiplier


def _gap(x1: float, hw1: float, x2: float, hw2: float) -> float:
    return abs(x1 - x2) - (hw1 + hw2)


def _params() -> dict:
    path = Path(__file__).resolve().parents[1] / "config" / "params.yaml"
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    return raw["quest3_video_streamer"]["ros__parameters"]


def test_yaml_base_does_not_cover_wrist_panels() -> None:
    p = _params()
    rs = p["base"]["layout"]
    rs_hw = _half_w(float(p["base"]["fov_h_deg"]), float(rs["size_multiplier"]))
    rs_x = float(rs["position"][0])
    for name in ("left_wrist", "right_wrist"):
        lay = p[name]["layout"]
        hw = _half_w(float(p[name]["fov_h_deg"]), float(lay["size_multiplier"]))
        wx = float(lay["position"][0])
        gap = _gap(rs_x, rs_hw, wx, hw)
        assert gap >= _CLEARANCE, (
            f"{name} covered by base: gap={gap:.3f} m "
            f"(base x={rs_x} hw={rs_hw:.3f}, {name} x={wx} hw={hw:.3f})"
        )


def test_yaml_base_auto_scan_block_matches_d435i() -> None:
    p = _params()
    d435i = p["d435i"]["layout"]
    base = p["base"]["layout"]
    assert base["position"] == d435i["position"]
    assert base["size_multiplier"] == d435i["size_multiplier"]


def test_scan_defaults_d435i_does_not_cover_wrists() -> None:
    rs_dev = {
        "fourcc": "YUYV",
        "label": "base",
        "sysfs_name": "Intel(R) RealSense(TM) Depth Camera 435i RGB",
    }
    w0 = {"fourcc": "MJPG", "label": "left_wrist", "sysfs_name": "USB Camera"}
    w2 = {"fourcc": "MJPG", "label": "right_wrist", "sysfs_name": "USB Camera"}
    found = [w0, w2, rs_dev]
    rs = _scan_role_defaults(rs_dev, found)
    rs_hw = _half_w(float(rs["fov_h_deg"]), float(rs["size_multiplier"]))
    rs_x = float(rs["position"][0])
    for wrist in (_scan_role_defaults(w0, found), _scan_role_defaults(w2, found)):
        hw = _half_w(float(wrist["fov_h_deg"]), float(wrist["size_multiplier"]))
        wx = float(wrist["position"][0])
        gap = _gap(rs_x, rs_hw, wx, hw)
        assert gap >= _CLEARANCE, (
            f"scan defaults overlap: gap={gap:.3f} m "
            f"(rs x={rs_x} hw={rs_hw:.3f}, wrist x={wx} hw={hw:.3f})"
        )
