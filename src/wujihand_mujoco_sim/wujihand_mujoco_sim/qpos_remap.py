"""Qpos order remap helpers (from wuji-retargeting/example/utils/config_paths.py)."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


def qpos_reorder_perm(
    src_joint_names: Sequence[str],
    dst_joint_names: Sequence[str],
) -> Optional[np.ndarray]:
    """Index array ``perm`` so that ``qpos[perm]`` reorders src→dst.

    Returns None when names cannot be aligned (caller may try other strategies).
    """
    if not dst_joint_names or not src_joint_names:
        return None
    idx = {n: i for i, n in enumerate(src_joint_names)}
    try:
        perm = np.array([idx[n] for n in dst_joint_names], dtype=int)
    except KeyError:
        return None
    if len(perm) != len(src_joint_names):
        return None
    return perm


def strip_side_prefix(name: str, side: str) -> str:
    """right_finger1_joint1 → finger1_joint1."""
    prefix = f"{side}_"
    if name.startswith(prefix):
        return name[len(prefix) :]
    return name


def build_cmd_to_actuator_perm(
    cmd_joint_names: Sequence[str],
    actuator_joint_names: Sequence[str],
    hand_side: str,
) -> Optional[np.ndarray]:
    """Map command order (finger1_joint1…) to MJCF actuator joint order.

    MJCF from wuji-description uses ``{side}_finger*_joint*`` names.
    """
    # Exact match first
    perm = qpos_reorder_perm(cmd_joint_names, actuator_joint_names)
    if perm is not None:
        return perm

    # Strip side prefix on actuator joints
    stripped = [strip_side_prefix(n or "", hand_side) for n in actuator_joint_names]
    perm = qpos_reorder_perm(cmd_joint_names, stripped)
    if perm is not None:
        return perm

    # Add side prefix to command names
    prefixed = [f"{hand_side}_{n}" for n in cmd_joint_names]
    return qpos_reorder_perm(prefixed, actuator_joint_names)
