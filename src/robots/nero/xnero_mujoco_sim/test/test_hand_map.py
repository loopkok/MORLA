"""手部映射的单元测试（纯 Python，不需要 ROS 运行）。"""
from xnero_mujoco_sim.hand_map import (
    FINGERS,
    HAND_JOINT_NAMES,
    actuator_name,
    arm_joint_names,
    mjcf_hand_joint,
    parse_tactile_geom,
    tactile_geom,
)


def test_hand_joint_names_are_12_and_ordered():
    """顺序必须与 XHandCommand.position 一致，否则采集数据会错位。"""
    assert len(HAND_JOINT_NAMES) == 12
    assert HAND_JOINT_NAMES[0] == "thumb_bend_joint"
    assert HAND_JOINT_NAMES[5] == "index_joint2"
    assert HAND_JOINT_NAMES[-1] == "pinky_joint2"


def test_prefixing():
    assert mjcf_hand_joint("left", "index_joint1") == "left_hand_index_joint1"
    assert actuator_name("right", "mid_joint2") == "act_right_hand_mid_joint2"


def test_arm_joint_names():
    assert arm_joint_names("left") == [f"left_joint{i}" for i in range(1, 8)]
    assert arm_joint_names("right")[6] == "right_joint7"


def test_tactile_geom_roundtrip():
    n = tactile_geom("left", "index", 1)
    assert n == "tac_left_index_p001"
    assert parse_tactile_geom(n) == ("left", "index", 1)
    assert parse_tactile_geom("tac_right_thumb_p120") == ("right", "thumb", 120)


def test_parse_rejects_non_tactile():
    for bad in ("right_hand_index_rota_link2", "", "tac_left_index_p12",
                "tac_left_index_p1200", "tac_middle_index_p001"):
        assert parse_tactile_geom(bad) is None


def test_fingers_tuple():
    assert FINGERS == ("thumb", "index", "mid", "ring", "pinky")
