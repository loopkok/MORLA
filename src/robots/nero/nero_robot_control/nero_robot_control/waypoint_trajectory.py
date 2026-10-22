"""Native Nero four-point path; angles in degrees at the configuration boundary."""
from __future__ import annotations
import numpy as np
from .joint_conventions import NERO_BILATERAL_SIGNS

TRAJECTORY_REVISION = "nero_four_points_home85_v2"
RIGHT_WAYPOINTS_DEG = (
    (0., 25., 0., 123., 0., 0., -60.),
    (0., 25., 0., 123., -100., 42., 0.),
    (0., 25., 0., 123., -100., 42., -70.),
    (29., 31., 42., 103., -120., 24., -10.),
)


def default_waypoints_deg(side: str) -> np.ndarray:
    if side not in {"left", "right"}:
        raise ValueError("trajectory side must be left or right")
    points = np.array(RIGHT_WAYPOINTS_DEG, dtype=float)
    return points * np.array(NERO_BILATERAL_SIGNS) if side == "left" else points


def waypoint_targets(side: str, waypoints_deg=None, *, reverse=False) -> list[np.ndarray]:
    """Return independent radian targets: 1,2,3,4 or 4,3,2,1.

    Custom points are already native for their side and never mirrored again.
    This helper returns the four waypoints only; home_targets appends its final pose.
    The driver validates ALL targets against its joint limits before sending any.
    """
    default = default_waypoints_deg(side)
    points = default if waypoints_deg is None else np.asarray(waypoints_deg, dtype=float)
    if points.shape == (28,):
        points = points.reshape(4, 7)
    if points.shape != (4, 7) or not np.isfinite(points).all():
        raise ValueError("trajectory requires exactly four finite seven-joint waypoints in degrees")
    radians = np.deg2rad(points)
    return [q.copy() for q in (radians[::-1] if reverse else radians)]
