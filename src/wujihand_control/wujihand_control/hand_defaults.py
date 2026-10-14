"""Load defaults from config/wujihand_control.yaml for launch files."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml
from ament_index_python.packages import get_package_share_directory


def _config_path() -> Path:
    try:
        share = Path(get_package_share_directory("wujihand_control"))
        path = share / "config" / "wujihand_control.yaml"
        if path.is_file():
            return path
    except Exception:
        pass
    # Source-tree fallback (before first install / unit tests)
    return Path(__file__).resolve().parents[1] / "config" / "wujihand_control.yaml"


def _load_config() -> Dict[str, Any]:
    path = _config_path()
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Invalid config: {path}")
    return data


_cfg = _load_config()

LEFT_HAND_SERIAL: str = str(_cfg.get("left_hand", {}).get("serial_number", "") or "")
RIGHT_HAND_SERIAL: str = str(_cfg.get("right_hand", {}).get("serial_number", "") or "")
LEFT_HAND_NAME: str = str(_cfg.get("left_hand", {}).get("name", "left_hand"))
RIGHT_HAND_NAME: str = str(_cfg.get("right_hand", {}).get("name", "right_hand"))

_driver = _cfg.get("driver", {}) or {}
DRIVER_PUBLISH_RATE: float = float(_driver.get("publish_rate", 1000.0))
DRIVER_FILTER_CUTOFF_FREQ: float = float(_driver.get("filter_cutoff_freq", 10.0))
DRIVER_DIAGNOSTICS_RATE: float = float(_driver.get("diagnostics_rate", 10.0))
