"""ROS2 thin control layer for Astral RobotMain via astral_robot_sdk."""

from .joint_layout import (
    LEFT_ARM_JOINT_NAMES,
    NUM_ARM_JOINTS,
    NUM_JOINTS,
    RIGHT_ARM_JOINT_NAMES,
    ROBOT_JOINT_NAMES,
)

__all__ = [
    "ROBOT_JOINT_NAMES",
    "LEFT_ARM_JOINT_NAMES",
    "RIGHT_ARM_JOINT_NAMES",
    "NUM_JOINTS",
    "NUM_ARM_JOINTS",
]
