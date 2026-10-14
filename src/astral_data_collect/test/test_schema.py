"""schema 布局与维度单元测试（纯 python，无需 ROS）。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.schema import (  # noqa: E402
    EE_GRIPPER,
    EE_NONE,
    EE_WUJI,
    CollectSchema,
)


def test_dual_gripper_dim():
    s = CollectSchema(end_effector_left=EE_GRIPPER, end_effector_right=EE_GRIPPER)
    # 双臂 14 + 双夹爪 2 = 16
    assert s.state_dim == 16
    assert [b.name for b in s.state_blocks()] == [
        "left_arm", "right_arm", "left_ee", "right_ee",
    ]


def test_single_left_arm():
    s = CollectSchema(arms=["left"], end_effector_left=EE_GRIPPER,
                      end_effector_right=EE_WUJI)  # 右侧设置不生效
    # 左臂 7 + 左夹爪 1 = 8
    assert s.state_dim == 8
    assert [b.name for b in s.state_blocks()] == ["left_arm", "left_ee"]
    streams = s.required_streams()
    assert "left_arm_state" in streams and "left_gripper_ratio" in streams
    assert not any(k.startswith("right_") for k in streams)


def test_single_right_arm_wuji_with_waist_head():
    s = CollectSchema(arms=["right"], end_effector_right=EE_WUJI,
                      include_waist=True, include_head=True)
    # 右臂 7 + wuji 20 + 腰 2 + 头 2 = 31
    assert s.state_dim == 31
    assert [b.name for b in s.state_blocks()] == [
        "right_arm", "right_ee", "waist", "head",
    ]


def test_arms_validation():
    import pytest

    with pytest.raises(ValueError):
        CollectSchema(arms=[])
    with pytest.raises(ValueError):
        CollectSchema(arms=["middle"])
    with pytest.raises(ValueError):
        CollectSchema(arms=["left", "left"])
    # 字符串大小写/空白宽容
    assert CollectSchema(arms=[" Left "]).arms == ["left"]


def test_gripper_wuji_head_dim():
    s = CollectSchema(
        end_effector_left=EE_GRIPPER, end_effector_right=EE_WUJI, include_head=True,
    )
    # 14 + 1 + 20 + 2 = 37
    assert s.state_dim == 37
    assert [b.name for b in s.state_blocks()] == [
        "left_arm", "right_arm", "left_ee", "right_ee", "head",
    ]


def test_dual_wuji_waist_head_dim():
    s = CollectSchema(
        end_effector_left=EE_WUJI, end_effector_right=EE_WUJI,
        include_waist=True, include_head=True,
    )
    # 14 + 20 + 20 + 2 + 2 = 58
    assert s.state_dim == 58


def test_none_end_effector_skips_block():
    s = CollectSchema(end_effector_left=EE_NONE, end_effector_right=EE_NONE)
    assert s.state_dim == 14
    assert not any(b.name.endswith("_ee") for b in s.state_blocks())


def test_required_streams_match_ee():
    s = CollectSchema(end_effector_left=EE_GRIPPER, end_effector_right=EE_WUJI)
    streams = s.required_streams()
    assert "left_gripper_ratio" in streams
    assert "left_hand_state" not in streams
    assert "right_hand_state" in streams
    assert streams["right_hand_state"] == 20
    assert "body_state" not in streams  # 未 include waist/head


def test_waist_head_use_body_state():
    s = CollectSchema(include_waist=True, include_head=True)
    streams = s.required_streams()
    assert streams["body_state"] == 18
    waist = [b for b in s.state_blocks() if b.name == "waist"][0]
    assert waist.body_slice == (14, 16)
    head = [b for b in s.state_blocks() if b.name == "head"][0]
    assert head.body_slice == (16, 18)


def test_command_mode_fallback_for_waist_head():
    s = CollectSchema(include_head=True, action_source="command")
    head = [b for b in s.state_blocks() if b.name == "head"][0]
    # 头/腰无采集侧指令话题 → 回退 state 流（next_state 语义）
    assert s.action_block_stream(head) == "body_state"
    arm = [b for b in s.state_blocks() if b.name == "left_arm"][0]
    assert s.action_block_stream(arm) == "left_arm_cmd"


def test_state_names_unique_and_sized():
    s = CollectSchema(end_effector_right=EE_WUJI, include_head=True)
    names = s.state_names()
    assert len(names) == s.state_dim
    assert len(set(names)) == len(names)
    assert names[0] == "left_arm_0"
    assert "right_ee_19" in names


def test_roundtrip_json():
    s = CollectSchema(
        end_effector_left=EE_GRIPPER, end_effector_right=EE_WUJI,
        include_waist=True, cameras=["a", "b", "c"], dataset_fps=15,
        action_source="command", hold_frames=5,
    )
    s2 = CollectSchema.from_json(s.to_json())
    assert s2.state_dim == s.state_dim
    assert s2.cameras == ["a", "b", "c"]
    assert s2.dataset_fps == 15
    assert s2.action_source == "command"
    assert s2.hold_frames == 5


def test_invalid_config_raises():
    import pytest

    with pytest.raises(ValueError):
        CollectSchema(end_effector_left="bad")
    with pytest.raises(ValueError):
        CollectSchema(action_source="bad")
    with pytest.raises(ValueError):
        CollectSchema(cameras=[])
