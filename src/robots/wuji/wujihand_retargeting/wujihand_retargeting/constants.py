"""Shared constants for Wuji Hand 20-DoF command layout.

Joint order matches rob_station / wujihandpy: reshape(5, 4) as
finger1..finger5 × joint1..joint4 (thumb = finger1).
"""

from __future__ import annotations

NUM_JOINTS = 20
NUM_FINGERS = 5
JOINTS_PER_FINGER = 4

# URDF revolute joints in command order (index order for joint_commands).
WUJI_JOINT_NAMES = [
    f"finger{f}_joint{j}"
    for f in range(1, NUM_FINGERS + 1)
    for j in range(1, JOINTS_PER_FINGER + 1)
]

WUJI_TIP_LINK_NAMES = [f"finger{f}_tip_link" for f in range(1, NUM_FINGERS + 1)]

WRIST_LINK_NAME = "palm_link"

# rob_station GLOVE_HAND_MODEL → wuji_sdk.retargeting.HandModel attribute
HAND_MODEL_MAP = {
    "wuji_hand": "WujiHand",
    "wuji_hand_2": "WujiHand2",
}

DEFAULT_HAND_MODEL = "wuji_hand"  # rob_station default
