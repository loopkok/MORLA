"""DexPilot backend adapted from xhand_retargeting for Wuji Hand 20-DoF.

Differences vs xhand_dex_retargeting_node:
  - 20 joints (5×4), not 12
  - URDF: palm_link + finger{{1..5}}_joint{{1..4}} / tip_link
  - No XHand index_bend sign flip
  - No XHAND short-name remap (command uses URDF index order = WUJI_JOINT_NAMES)
  - low_pass_alpha / project_dist tuned in yaml (see config/retarget_dexpilot_*.yml)
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from wujihand_retargeting.constants import WUJI_JOINT_NAMES


class DexPilotRetargetBackend:
    def __init__(
        self,
        hand_side: str,
        config_path: Optional[str] = None,
        urdf_path: Optional[str] = None,
    ):
        try:
            from dex_retargeting.retargeting_config import RetargetingConfig
        except ImportError as exc:
            raise RuntimeError(
                "未找到 dex_retargeting。请安装：pip install dex-retargeting"
            ) from exc

        side = hand_side.lower()
        if side not in ("left", "right"):
            raise ValueError(f"hand_side 须为 left|right，收到: {hand_side}")

        share = Path(get_package_share_directory("wujihand_retargeting"))
        if config_path is None:
            config_path = str(share / "config" / f"retarget_dexpilot_{side}.yml")
        if urdf_path is None:
            urdf_path = str(share / "urdf" / f"wujihand_{side}.urdf")

        with open(config_path, "r", encoding="utf-8") as f:
            yaml_config = yaml.load(f, Loader=yaml.FullLoader)
        cfg = dict(yaml_config["retargeting"])
        cfg["urdf_path"] = str(urdf_path)

        self._retargeter = RetargetingConfig.from_dict(cfg).build()
        self.hand_side = side
        self.config_path = config_path
        self.urdf_path = urdf_path

        # Precompute name → index for stable 20-DoF output order
        names = list(self._retargeter.joint_names)
        self._out_indices = []
        for name in WUJI_JOINT_NAMES:
            if name not in names:
                raise RuntimeError(
                    f"DexPilot retargeter 缺少关节 {name}；"
                    f"retargeter.joint_names={names}"
                )
            self._out_indices.append(names.index(name))

    def retarget(self, keypoints: np.ndarray) -> np.ndarray:
        pose_data = np.asarray(keypoints, dtype=np.float64)
        if pose_data.shape == (63,):
            pose_data = pose_data.reshape(21, 3)
        if pose_data.shape != (21, 3):
            raise ValueError(f"keypoints shape 须为 (21,3)，收到 {pose_data.shape}")

        retargeting_type = self._retargeter.optimizer.retargeting_type
        indices = self._retargeter.optimizer.target_link_human_indices

        if retargeting_type == "POSITION":
            reference_values = pose_data[indices, :]
        else:
            origin_indices = indices[0, :]
            task_indices = indices[1, :]
            reference_values = pose_data[task_indices, :] - pose_data[origin_indices, :]

        joint_values = np.asarray(
            self._retargeter.retarget(reference_values), dtype=np.float64
        )
        ordered = joint_values[self._out_indices]
        return ordered.reshape(-1)

    def reset(self) -> None:
        # dex-retargeting optimizer has internal LP state; rebuild if needed later
        pass
