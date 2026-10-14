"""Official backend: wuji_sdk RetargetSession (aligned with rob_station).

rob_station uses:
  GLOVE_HAND_MODEL ∈ {wuji_hand, wuji_hand_2}
  RetargetSession.for_hand(HandModel.WujiHand|WujiHand2, side)
  session.step(keypoints) → (20,) qpos

SDK API note (2026.8.x):
  RetargetSession / HandModel are top-level exports of wuji_sdk.
  Older docs / rob_station code still write ``from wuji_sdk import retargeting``;
  that submodule name is gone on current PyPI wheels — use top-level imports.

No AdaptiveOptimizer yaml here — that is wuji-hand-teleop's wuji_retargeting
path. This backend matches the already-tuned rob_station glove→hand pipeline.
"""

from __future__ import annotations

import numpy as np

from wujihand_retargeting.constants import DEFAULT_HAND_MODEL, HAND_MODEL_MAP
from wujihand_retargeting.runtime_bootstrap import bootstrap_retarget_runtime


def _load_retarget_api():
    """Return (Handedness, HandModel, RetargetSession) for old or new wuji_sdk."""
    try:
        from wuji_sdk import Handedness
    except ImportError as exc:
        raise RuntimeError(
            "未找到 wuji_sdk。请安装：/usr/bin/python3 -m pip install --user wuji-sdk"
        ) from exc

    # New API (2026.8.3+): top-level RetargetSession / HandModel
    try:
        from wuji_sdk import HandModel, RetargetSession

        return Handedness, HandModel, RetargetSession
    except ImportError:
        pass

    # Legacy API: wuji_sdk.retargeting submodule (older wheels / [retarget] extra)
    try:
        from wuji_sdk import retargeting

        return Handedness, retargeting.HandModel, retargeting.RetargetSession
    except ImportError as exc:
        raise RuntimeError(
            "已安装 wuji_sdk，但找不到 RetargetSession。"
            "请升级：/usr/bin/python3 -m pip install --user -U wuji-sdk"
        ) from exc


class OfficialRetargetBackend:
    def __init__(self, hand_side: str, hand_model: str = DEFAULT_HAND_MODEL):
        bootstrap_retarget_runtime()
        Handedness, HandModel, RetargetSession = _load_retarget_api()

        side = hand_side.lower()
        if side not in ("left", "right"):
            raise ValueError(f"hand_side 须为 left|right，收到: {hand_side}")

        model_key = (hand_model or DEFAULT_HAND_MODEL).strip().lower()
        model_attr = HAND_MODEL_MAP.get(model_key)
        if model_attr is None:
            raise ValueError(
                f"未知 hand_model={hand_model!r}，可选: {list(HAND_MODEL_MAP)}"
            )

        model = getattr(HandModel, model_attr)
        handedness = Handedness.Right if side == "right" else Handedness.Left
        self._session = RetargetSession.for_hand(model, side=handedness)
        self.hand_side = side
        self.hand_model = model_key

    def retarget(self, keypoints: np.ndarray) -> np.ndarray:
        kp = np.asarray(keypoints, dtype=np.float32)
        if kp.shape == (63,):
            kp = kp.reshape(21, 3)
        if kp.shape != (21, 3):
            raise ValueError(f"keypoints shape 须为 (21,3)，收到 {kp.shape}")
        qpos = self._session.step(kp)
        return np.asarray(qpos, dtype=np.float64).reshape(-1)

    def reset(self) -> None:
        if hasattr(self._session, "reset"):
            self._session.reset()
