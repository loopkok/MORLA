"""ROS SDK joint names ↔ MJCF ``left_jointN`` / ``right_jointN``."""

from __future__ import annotations

from typing import List

LEFT_ARM_JOINT_NAMES: List[str] = [
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_elbow_roll",
    "left_elbow_pitch",
    "left_forearm_roll",
    "left_wrist_pitch",
    "left_wrist_roll",
]

RIGHT_ARM_JOINT_NAMES: List[str] = [
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_elbow_roll",
    "right_elbow_pitch",
    "right_forearm_roll",
    "right_wrist_pitch",
    "right_wrist_roll",
]

# MJCF hinge names from astral_robot_description conversion
LEFT_MJCF_JOINTS: List[str] = [f"left_joint{i}" for i in range(1, 8)]
RIGHT_MJCF_JOINTS: List[str] = [f"right_joint{i}" for i in range(1, 8)]

# Accept teleop legacy alias for J7
_NAME_ALIASES = {
    "left_wrist_yaw": "left_wrist_roll",
    "right_wrist_yaw": "right_wrist_roll",
}


def normalize_joint_name(name: str) -> str:
    return _NAME_ALIASES.get(str(name), str(name))
