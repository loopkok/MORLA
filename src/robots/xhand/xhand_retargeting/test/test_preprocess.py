import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from xhand_retargeting.preprocess import (  # noqa: E402
    preprocess_xhand_landmarks,
)


M = np.array([[-1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
F = np.diag([-1.0, 1.0, 1.0])
OP_RIGHT = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
OP_LEFT = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=float)


def old_xnero_reference(unity_points, side, pinky=True):
    """Literal numeric reference for the source XNero Quest path."""
    points = unity_points.copy()
    points[:, 0] = -points[:, 0]
    points *= 1.05
    points -= points[0:1, :]
    selected = points[[0, 5, 9], :]
    x_vector = selected[0] - selected[2]
    selected = selected - np.mean(selected, axis=0, keepdims=True)
    _, _, v = np.linalg.svd(selected)
    normal = v[2, :]
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / np.linalg.norm(x)
    z = np.cross(x, normal)
    if np.sum(z * (selected[1] - selected[2])) < 0:
        normal *= -1
        z *= -1
    frame = np.stack([x, normal, z], axis=1)
    operator2mano = OP_RIGHT if side == "right" else OP_LEFT
    result = points @ frame @ operator2mano
    if pinky:
        result = result.copy()
        mcp, pip, dip, tip = 17, 18, 19, 20
        extension = np.linalg.norm(result[tip] - result[mcp])
        ratio = np.clip((extension - 0.03) / (0.10 - 0.03), 0.0, 1.0)
        scale = 1.2 + (2.2 - 1.2) * ratio
        vector = result[pip] - result[mcp]
        result[pip] = result[mcp] + vector * scale
        vector = result[dip] - result[pip]
        result[dip] = result[pip] + vector * scale
        vector = result[tip] - result[dip]
        result[tip] = result[dip] + vector * scale
    return result


def make_unity_hand(seed):
    rng = np.random.default_rng(seed)
    points = rng.normal(0.0, 0.04, (21, 3))
    points[0] = [0.0, 0.0, 0.0]
    points[5] = [-0.04, 0.03, 0.005]
    points[9] = [0.0, 0.06, 0.0]
    return points


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("seed", range(20))
def test_legacy_pinky_path_matches_xnero_source(side, seed):
    unity = make_unity_hand(seed)
    robot_raw = unity @ M.T
    actual = preprocess_xhand_landmarks(robot_raw, side, "legacy_xhand_pinky")
    expected = old_xnero_reference(unity, side, pinky=True)
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-12)


@pytest.mark.parametrize("side", ["left", "right"])
def test_mano_only_mode_matches_source_before_pinky(side):
    unity = make_unity_hand(77)
    robot_raw = unity @ M.T
    actual = preprocess_xhand_landmarks(robot_raw, side, "legacy_xhand")
    expected = old_xnero_reference(unity, side, pinky=False)
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-12)


def test_side_specific_mano_frames_are_preserved_and_input_is_not_mutated():
    raw = make_unity_hand(3) @ M.T
    before = raw.copy()
    left = preprocess_xhand_landmarks(raw, "left")
    right = preprocess_xhand_landmarks(raw, "right")
    assert not np.allclose(left, right)
    np.testing.assert_array_equal(raw, before)


@pytest.mark.parametrize(
    "points,side,mode",
    [
        (np.zeros((20, 3)), "left", "legacy_xhand_pinky"),
        (np.full((21, 3), np.nan), "right", "legacy_xhand_pinky"),
        (np.zeros((21, 3)), "left", "legacy_xhand_pinky"),
        (np.zeros((21, 3)), "middle", "legacy_xhand_pinky"),
        (np.zeros((21, 3)), "left", "raw"),
    ],
)
def test_rejects_bad_shape_nonfinite_degenerate_side_and_mode(points, side, mode):
    with pytest.raises(ValueError):
        preprocess_xhand_landmarks(points, side, mode)
