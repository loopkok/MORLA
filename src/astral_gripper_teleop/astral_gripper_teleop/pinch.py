"""Pinch distance → gripper close ratio / actuator radians.

MediaPipe / Quest 21-point layout (wrist-local):
  4 = thumb tip, 8 = index tip.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

THUMB_TIP = 4
INDEX_TIP = 8


def pinch_distance_m(landmarks: Sequence[Sequence[float]]) -> float:
    pts = np.asarray(landmarks, dtype=np.float64)
    if pts.shape[0] < 9 or pts.shape[1] < 3:
        raise ValueError(f"need (21,3) landmarks, got {pts.shape}")
    delta = pts[THUMB_TIP, :3] - pts[INDEX_TIP, :3]
    return float(np.linalg.norm(delta))


def close_ratio_from_pinch(
    dist_m: float,
    open_dist_m: float,
    close_dist_m: float,
) -> float:
    """Map tip distance to close ratio in [0, 1].

    0 = fully open (fingers apart), 1 = fully closed (pinch).
    """
    span = float(open_dist_m) - float(close_dist_m)
    if abs(span) < 1e-9:
        return 1.0 if dist_m <= close_dist_m else 0.0
    t = (float(dist_m) - float(close_dist_m)) / span
    open_amt = max(0.0, min(1.0, t))
    return 1.0 - open_amt


def close_ratio_from_range(
    dist_m: float,
    lo_m: float,
    hi_m: float,
    min_span_m: float = 0.01,
) -> float:
    """Close ratio from an (auto-tracked) distance envelope.

    lo_m = observed min distance (full pinch), hi_m = observed max (full open).
    Span is clamped to ``min_span_m`` so a still hand does not blow up the
    denominator. 0 = open, 1 = pinched/closed.
    """
    span = float(hi_m) - float(lo_m)
    if span < float(min_span_m):
        span = float(min_span_m)
    t = (float(dist_m) - float(lo_m)) / span
    open_amt = max(0.0, min(1.0, t))
    return 1.0 - open_amt


def ratio_to_rad(close_ratio: float, open_rad: float, closed_rad: float) -> float:
    r = max(0.0, min(1.0, float(close_ratio)))
    return float(open_rad) + r * (float(closed_rad) - float(open_rad))
