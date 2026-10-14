"""XHand-only Quest landmark conversion kept out of the shared input layer.

The Quest receiver publishes wrist-local points after the Unity-to-robot
rotation ``M``. The legacy XNero receiver instead flipped Unity X with ``F``
before MANO conversion. ``legacy_xhand_pinky`` reconstructs that exact input
and preserves the original numeric behavior for migrated profiles.
"""

from __future__ import annotations

import numpy as np

UNITY_TO_ROBOT = np.array(
    [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]], dtype=np.float64
)
LEGACY_X_FLIP = np.diag([-1.0, 1.0, 1.0])

OPERATOR2MANO_RIGHT = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)
OPERATOR2MANO_LEFT = np.array(
    [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)


def _landmark_array(landmarks: np.ndarray) -> np.ndarray:
    points = np.asarray(landmarks, dtype=np.float64)
    if points.shape != (21, 3):
        raise ValueError(f"expected landmarks with shape (21, 3), got {points.shape}")
    if not np.isfinite(points).all():
        raise ValueError("landmarks must contain only finite values")
    return points.copy()


def unity_points_from_robot_raw(landmarks: np.ndarray) -> np.ndarray:
    """Invert Quest ``M`` for points already expressed in robot axes."""
    points = _landmark_array(landmarks)
    return (UNITY_TO_ROBOT.T @ points.T).T


def estimate_frame_from_hand_points(points: np.ndarray) -> np.ndarray:
    """Return the legacy MANO wrist frame from wrist/index/middle landmarks."""
    points = _landmark_array(points)
    selected = points[[0, 5, 9], :]
    x_vector = selected[0] - selected[2]
    selected = selected - np.mean(selected, axis=0, keepdims=True)
    _, _, v = np.linalg.svd(selected)
    normal = v[2, :]
    x = x_vector - np.sum(x_vector * normal) * normal
    norm = np.linalg.norm(x)
    if norm < 1e-12:
        raise ValueError("degenerate hand landmarks cannot define a MANO frame")
    x = x / norm
    z = np.cross(x, normal)
    if np.sum(z * (selected[1] - selected[2])) < 0:
        normal *= -1
        z *= -1
    return np.stack([x, normal, z], axis=1)


def adaptive_retargeting_xhand(landmarks: np.ndarray) -> np.ndarray:
    """Legacy sequential pinky stretch from XNero Quest preprocessing."""
    points = _landmark_array(landmarks)
    mcp, pip, dip, tip = 17, 18, 19, 20
    extension = np.linalg.norm(points[tip] - points[mcp])
    ratio = np.clip((extension - 0.03) / (0.10 - 0.03), 0.0, 1.0)
    scale = 1.2 + (2.2 - 1.2) * ratio
    vector = points[pip] - points[mcp]
    points[pip] = points[mcp] + vector * scale
    vector = points[dip] - points[pip]
    points[dip] = points[pip] + vector * scale
    vector = points[tip] - points[dip]
    points[tip] = points[dip] + vector * scale
    return points


def preprocess_xhand_landmarks(
    robot_raw_landmarks: np.ndarray,
    side: str,
    mode: str = "legacy_xhand_pinky",
) -> np.ndarray:
    """Convert unified robot-axis raw landmarks to the legacy XHand MANO input.

    ``mode`` is one of ``legacy_xhand`` (MANO only) or
    ``legacy_xhand_pinky`` (MANO plus the sequential legacy pinky stretch).
    Input is never modified in place.
    """
    side = str(side).strip().lower()
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    if mode not in ("legacy_xhand", "legacy_xhand_pinky"):
        raise ValueError("mode must be 'legacy_xhand' or 'legacy_xhand_pinky'")

    raw = _landmark_array(robot_raw_landmarks)
    # Column-vector equation from the migration specification:
    # old = F @ M.T @ new_raw. Quest points are wrist-local.
    legacy_axes = (LEGACY_X_FLIP @ UNITY_TO_ROBOT.T @ raw.T).T
    operator2mano = OPERATOR2MANO_RIGHT if side == "right" else OPERATOR2MANO_LEFT
    centered = legacy_axes * 1.05
    centered -= centered[0:1, :]
    frame = estimate_frame_from_hand_points(centered)
    joint_positions = centered @ frame @ operator2mano
    if mode == "legacy_xhand_pinky":
        joint_positions = adaptive_retargeting_xhand(joint_positions)
    return joint_positions
