"""Strict conversion rules between MORLA and RobotEra XHand contracts."""

from __future__ import annotations

import math
import numpy as np


HAND_JOINT_NAMES = (
    "thumb_bend_joint",
    "thumb_rota_joint1",
    "thumb_rota_joint2",
    "index_bend_joint",
    "index_joint1",
    "index_joint2",
    "mid_joint1",
    "mid_joint2",
    "ring_joint1",
    "ring_joint2",
    "pinky_joint1",
    "pinky_joint2",
)


class XHandContractError(ValueError):
    """Raised when a standard or native XHand message violates its contract."""


def reorder_named_positions(names, positions, expected_names=HAND_JOINT_NAMES):
    """Return values in ``expected_names`` order, rejecting ambiguous layouts."""
    names = tuple(str(name) for name in names)
    values = tuple(float(value) for value in positions)
    expected = tuple(str(name) for name in expected_names)
    if len(expected) != 12 or len(set(expected)) != 12:
        raise XHandContractError("expected_names must contain 12 unique joint names")
    if len(names) != 12 or len(set(names)) != 12 or len(values) != 12:
        raise XHandContractError("XHand command requires exactly 12 unique names and positions")
    missing = sorted(set(expected) - set(names))
    extra = sorted(set(names) - set(expected))
    if missing or extra:
        raise XHandContractError(f"joint name mismatch; missing={missing}, extra={extra}")
    if not all(math.isfinite(value) for value in values):
        raise XHandContractError("XHand command contains non-finite positions")
    by_name = dict(zip(names, values))
    return tuple(by_name[name] for name in expected)


def validate_joint_limits(positions, lower, upper):
    """Reject finite 12-vectors that exceed the profile's radian limits."""
    values = np.asarray(positions, dtype=float)
    lo = np.asarray(lower, dtype=float)
    hi = np.asarray(upper, dtype=float)
    if values.shape != (12,) or lo.shape != (12,) or hi.shape != (12,):
        raise XHandContractError("XHand positions and lower/upper limits must have 12 entries")
    if not np.isfinite(values).all() or not np.isfinite(lo).all() or not np.isfinite(hi).all():
        raise XHandContractError("XHand positions and limits must be finite")
    if np.any(lo >= hi):
        raise XHandContractError("every XHand lower limit must be less than its upper limit")
    if np.any(values < lo) or np.any(values > hi):
        bad = np.flatnonzero((values < lo) | (values > hi)).tolist()
        raise XHandContractError(f"XHand command exceeds profile joint limits at indices {bad}")
    return tuple(float(value) for value in values)


def native_state_positions(state_array, hand_id, side):
    """Select the requested hand by ID and return its 12 named positions.

    The vendor message stores IDs and states in parallel arrays. Some SDK
    versions prefix state names with ``left_hand_`` / ``right_hand_``; both
    spellings are accepted only after the hand ID has matched.
    """
    ids = tuple(int(value) for value in state_array.hand_id)
    if len(set(ids)) != len(ids):
        raise XHandContractError("native state contains duplicate hand_id values")
    if len(state_array.hand_states) != len(ids):
        raise XHandContractError("native hand_id and hand_states arrays are not aligned")
    try:
        index = ids.index(int(hand_id))
    except ValueError as exc:
        raise XHandContractError(f"native state does not contain hand_id={hand_id}") from exc
    state = state_array.hand_states[index]
    names = tuple(str(name) for name in state.name)
    prefix = f"{str(side).strip().lower()}_hand_"
    if names and all(name.startswith(prefix) for name in names):
        names = tuple(name[len(prefix):] for name in names)
    return reorder_named_positions(names, state.position)
