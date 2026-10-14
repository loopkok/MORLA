"""手部关节名 / 执行器名 / 触觉 geom 名的唯一映射源。

命名约定（务必与真机驱动、URDF、MJCF 三边一致）：
  URDF/MJCF 关节 : {side}_hand_{name}          name ∈ HAND_JOINT_NAMES
  执行器         : act_{side}_hand_{name}
  触觉碰撞 geom  : tac_{side}_{finger}_p{001..120}   （= SDK raw_force[k] 的 k+1）

XHandCommand.position 的数组顺序就是 HAND_JOINT_NAMES 的顺序（无前缀），
与 xhand_retargeting:XHAND_JOINT_NAMES、真机驱动 finger_joint_names_ 一致。
"""

from __future__ import annotations

import re

FINGERS = ("thumb", "index", "mid", "ring", "pinky")
SIDES = ("left", "right")

HAND_JOINT_NAMES = [
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2",
    "mid_joint1", "mid_joint2",
    "ring_joint1", "ring_joint2",
    "pinky_joint1", "pinky_joint2",
]
NUM_HAND_JOINTS = len(HAND_JOINT_NAMES)

# 真机驱动 (xhand_control_ros2) 里 finger_sensor_states[].location 用的是这些字符串，
# 注意中指是 "middle" 而 URDF/link 名是 "mid" —— 为了与真机契约一致，对外发 "middle"
LOCATION_LABEL = {"thumb": "thumb", "index": "index", "mid": "middle",
                  "ring": "ring", "pinky": "pinky"}

_TAC_RE = re.compile(r"^tac_(left|right)_(thumb|index|mid|ring|pinky)_p(\d{3})$")


def arm_joint_names(side: str) -> list[str]:
    """臂的 7 个关节名（URDF/MJCF 里的写法）。"""
    return [f"{side}_joint{i}" for i in range(1, 8)]


# Standard MORLA arm JointState names include the side prefix.
ARM_STATE_JOINT_NAMES = {side: [f"{side}_joint{i}" for i in range(1, 8)] for side in SIDES}


def mjcf_hand_joint(side: str, name: str) -> str:
    return f"{side}_hand_{name}"


def actuator_name(side: str, name: str) -> str:
    return f"act_{side}_hand_{name}"


def tactile_geom(side: str, finger: str, point: int) -> str:
    """point 为 1..120（= SDK raw_force 下标 + 1）。"""
    return f"tac_{side}_{finger}_p{point:03d}"


def parse_tactile_geom(name: str):
    """'tac_left_index_p001' → ('left', 'index', 1)；非触觉 geom 返回 None。"""
    m = _TAC_RE.match(name or "")
    if not m:
        return None
    return (m.group(1), m.group(2), int(m.group(3)))
