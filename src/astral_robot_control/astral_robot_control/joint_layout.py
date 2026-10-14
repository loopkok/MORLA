"""Astral Robot ROS2 control — joint name / DOF contract.

Aligned with ``astral_robot_sdk.api.robot_options``:

  18-DoF order = left_arm(7) + right_arm(7) + waist(2) + head(2)

Mechanical grippers use CMD 0x97/0x98 (``set_gripper_angle``), not 0x31/0x32.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

# Prefer SDK constants when installed; fall back to a frozen copy so the
# package still imports for docs / dry-run without the native SDK.
try:
    from astral_robot_sdk import ROBOT_JOINT_NAMES as _SDK_NAMES
    from astral_robot_sdk import LEFT_ARM_IDS, RIGHT_ARM_IDS  # noqa: F401

    ROBOT_JOINT_NAMES: List[str] = list(_SDK_NAMES)
except ImportError:  # pragma: no cover
    ROBOT_JOINT_NAMES = [
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_elbow_roll",
        "left_elbow_pitch",
        "left_forearm_roll",
        "left_wrist_pitch",
        "left_wrist_roll",
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_elbow_roll",
        "right_elbow_pitch",
        "right_forearm_roll",
        "right_wrist_pitch",
        "right_wrist_roll",
        "waist_front",
        "waist_side",
        "head_yaw",
        "head_pitch",
    ]

NUM_JOINTS = 18
NUM_ARM_JOINTS = 7

LEFT_ARM_JOINT_NAMES: List[str] = ROBOT_JOINT_NAMES[0:7]
RIGHT_ARM_JOINT_NAMES: List[str] = ROBOT_JOINT_NAMES[7:14]
WAIST_JOINT_NAMES: List[str] = ROBOT_JOINT_NAMES[14:16]
HEAD_JOINT_NAMES: List[str] = ROBOT_JOINT_NAMES[16:18]
# 兼容旧导入名（曾误称夹爪）
GRIPPER_JOINT_NAMES = HEAD_JOINT_NAMES

# Default ROS topic contract (absolute names).
LEFT_ARM_NS = "left_arm"
RIGHT_ARM_NS = "right_arm"
ASTRAL_NS = "astral"
HEAD_NS = "head"
LEFT_GRIPPER_NS = "left_gripper"
RIGHT_GRIPPER_NS = "right_gripper"
CMD_RATIO_SUFFIX = "command"

CMD_SUFFIX = "joint_commands"
STATE_SUFFIX = "joint_states"


def pack_named_positions(
    names: Sequence[str],
    positions: Sequence[float],
    expected_names: Sequence[str],
) -> List[float]:
    """Map a JointState (names+positions) into ``expected_names`` order.

    If ``names`` is empty or length mismatch, treat ``positions`` as already
    ordered (positional). Missing names keep 0.0.
    """
    expected = list(expected_names)
    n_exp = len(expected)
    pos = [float(x) for x in positions]

    if not names or len(names) != len(pos):
        if len(pos) < n_exp:
            return pos + [0.0] * (n_exp - len(pos))
        return pos[:n_exp]

    lut = {str(n): float(v) for n, v in zip(names, pos)}
    matched = sum(1 for n in expected if n in lut)
    if matched < max(1, n_exp // 2):
        # Few name hits → positional fallback
        if len(pos) < n_exp:
            return pos + [0.0] * (n_exp - len(pos))
        return pos[:n_exp]
    return [lut.get(n, 0.0) for n in expected]


def split_full_q(q18: Sequence[float]) -> Tuple[List[float], List[float], List[float], List[float]]:
    """Split 18-DoF vector into left, right, waist, head."""
    q = list(q18)
    if len(q) < NUM_JOINTS:
        q = q + [0.0] * (NUM_JOINTS - len(q))
    return q[0:7], q[7:14], q[14:16], q[16:18]
