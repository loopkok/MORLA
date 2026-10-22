"""Native joint-angle mirroring for the current human-like Nero installation.

See docs/nero_joint_conventions.md. Apply once when deriving a mirrored target,
not to joint feedback or targets already expressed for their own arm.
"""
from __future__ import annotations

import numpy as np


NERO_BILATERAL_SIGNS = (-1, 1, -1, 1, -1, -1, 1)


def mirror_joint_positions(positions) -> np.ndarray:
    """Convert a native right-arm target to left, or vice versa (same units)."""
    q = np.asarray(positions, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("Nero mirroring requires seven finite native joint angles")
    return q * np.asarray(NERO_BILATERAL_SIGNS)
