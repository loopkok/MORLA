"""Retarget backend protocol."""

from __future__ import annotations

from typing import Protocol

import numpy as np


class RetargetBackend(Protocol):
    def retarget(self, keypoints: np.ndarray) -> np.ndarray:
        """(21, 3) MediaPipe keypoints → (20,) joint angles [rad]."""
        ...

    def reset(self) -> None:
        ...
