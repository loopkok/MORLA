"""Open-source wuji_retargeting backend (teleop AdaptiveOptimizer path).

Uses:
  from wuji_retargeting import Retargeter
  Retargeter.from_yaml(config, side).retarget(keypoints)

This is NOT wuji_sdk.RetargetSession. Default yaml copied from
wuji-hand-teleop retarget_wuji_glove_{left,right}.yaml.

Optional speed knobs (aligned with teleop / optimization notes):
  - nlopt_max_eval: lower = faster (teleop default often 25; 0 = library default)
  - disable_timing: turn off in-optimizer perf_counter overhead
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
from ament_index_python.packages import get_package_share_directory

from wujihand_retargeting.constants import NUM_JOINTS
from wujihand_retargeting.runtime_bootstrap import bootstrap_retarget_runtime


class WujiLibRetargetBackend:
    def __init__(
        self,
        hand_side: str,
        config_path: Optional[str] = None,
        nlopt_max_eval: int = 25,
        disable_timing: bool = True,
    ):
        bootstrap_retarget_runtime()
        try:
            from wuji_retargeting import Retargeter
        except ImportError as exc:
            raise RuntimeError(
                "未找到 wuji_retargeting。请安装开源包，例如：\n"
                "  pip install wuji-retargeting\n"
                "或从 wuji-hand-teleop 的 submodule  editable 安装。"
            ) from exc

        side = hand_side.lower()
        if side not in ("left", "right"):
            raise ValueError(f"hand_side 须为 left|right，收到: {hand_side}")

        if config_path is None or not str(config_path).strip():
            share = Path(get_package_share_directory("wujihand_retargeting"))
            config_path = str(share / "config" / f"retarget_wuji_lib_{side}.yaml")

        path = Path(config_path)
        if not path.is_file():
            raise FileNotFoundError(f"wuji_retargeting 配置不存在: {path}")

        self.hand_side = side
        self.config_path = str(path)
        self.nlopt_max_eval = int(nlopt_max_eval)
        self._retargeter = Retargeter.from_yaml(self.config_path, side)
        self._apply_runtime_tweaks(disable_timing=disable_timing)

    def _apply_runtime_tweaks(self, disable_timing: bool) -> None:
        opt = getattr(self._retargeter, "optimizer", None)
        if opt is None:
            return

        if disable_timing and hasattr(opt, "set_timing_enabled"):
            try:
                opt.set_timing_enabled(False)
            except Exception:  # noqa: BLE001
                pass

        if self.nlopt_max_eval > 0:
            # Prefer public helpers; fall back to nlopt handle if present.
            if hasattr(opt, "set_maxeval"):
                try:
                    opt.set_maxeval(self.nlopt_max_eval)
                    return
                except Exception:  # noqa: BLE001
                    pass
            raw = getattr(opt, "opt", None)
            if raw is not None and hasattr(raw, "set_maxeval"):
                try:
                    raw.set_maxeval(self.nlopt_max_eval)
                except Exception:  # noqa: BLE001
                    pass

    def retarget(self, keypoints: np.ndarray) -> np.ndarray:
        kp = np.asarray(keypoints, dtype=np.float32)
        if kp.shape == (63,):
            kp = kp.reshape(21, 3)
        if kp.shape != (21, 3):
            raise ValueError(f"keypoints shape 须为 (21,3)，收到 {kp.shape}")

        qpos = self._retargeter.retarget(kp)
        out = np.asarray(qpos, dtype=np.float64).reshape(-1)
        if out.size != NUM_JOINTS:
            raise RuntimeError(
                f"wuji_retargeting 输出维数 {out.size} != {NUM_JOINTS}"
            )
        return out

    def reset(self) -> None:
        if hasattr(self._retargeter, "reset"):
            self._retargeter.reset()
