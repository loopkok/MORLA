"""XHand-only Quest landmark conversion kept out of the shared input layer.

The Quest receiver now publishes raw Unity wrist-local points, so the XHand
convention is expressed directly on Unity input: the legacy XNero path flipped
Unity X with ``F = diag([-1, 1, 1])``. That is still the default here
(``F @ u == F @ Mᵀ @ (M @ u)``), so ``legacy_xhand_pinky`` keeps its original
numbers; a different Unity-to-hand basis can be passed in ``input_basis``.
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


def _basis_matrix(value, label: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape == (9,):
        matrix = matrix.reshape(3, 3)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError(f"{label} must be a finite 3x3 matrix")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6):
        raise ValueError(f"{label} must be orthonormal")
    return matrix


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
    input_basis=None,
) -> np.ndarray:
    """Convert raw Unity landmarks to the legacy XHand MANO input.

    ``mode`` is one of ``legacy_xhand`` (MANO only) or
    ``legacy_xhand_pinky`` (MANO plus the sequential legacy pinky stretch).
    ``input_basis`` is this hand's Unity -> hand-frame matrix; ``None`` keeps
    the historical XNero X flip. Input is never modified in place.
    """
    side = str(side).strip().lower()
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    if mode not in ("legacy_xhand", "legacy_xhand_pinky"):
        raise ValueError("mode must be 'legacy_xhand' or 'legacy_xhand_pinky'")

    raw = _landmark_array(robot_raw_landmarks)
    basis = LEGACY_X_FLIP if input_basis is None else _basis_matrix(input_basis, "input_basis")
    # Quest points are wrist-local and raw Unity, so the legacy XNero axes are
    # simply the hand's own basis applied to them.
    legacy_axes = (basis @ raw.T).T
    operator2mano = OPERATOR2MANO_RIGHT if side == "right" else OPERATOR2MANO_LEFT
    centered = legacy_axes * 1.05
    centered -= centered[0:1, :]
    frame = estimate_frame_from_hand_points(centered)
    joint_positions = centered @ frame @ operator2mano
    if mode == "legacy_xhand_pinky":
        joint_positions = adaptive_retargeting_xhand(joint_positions)
    return joint_positions
