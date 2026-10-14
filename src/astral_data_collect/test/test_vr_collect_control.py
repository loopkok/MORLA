"""VR 采集控制键位逻辑单元测试（纯 python，无需 ROS）。

覆盖 vr_collect_logic.decide_vr_command：上升沿触发、状态门控、
采集节点未运行（state=None）、长按不重复、缺位 buttons 不越界。

当前映射：A=start（仅 IDLE）、B=stop（录制中）、摇杆按下=discard（录制中）。
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.vr_collect_logic import (  # noqa: E402
    BUTTON_A,
    BUTTON_B,
    BUTTON_STICK_PRESS,
    decide_release_or_collect,
    decide_vr_command,
)

# 便捷构造：6 位 buttons 向量
_ZERO = [0, 0, 0, 0, 0, 0]


def _press(idx, prev=None):
    cur = _ZERO.copy()
    cur[idx] = 1
    return _ZERO if prev is None else prev, cur


def test_a_in_idle_starts():
    prev, cur = _press(BUTTON_A)
    assert decide_vr_command("IDLE", prev, cur) == "start"


def test_a_while_recording_gated():
    prev, cur = _press(BUTTON_A)
    assert decide_vr_command("RECORDING", prev, cur) is None
    assert decide_vr_command("PAUSED", prev, cur) is None


def test_a_hold_is_not_repeat():
    # 长按：连续两帧都为 1 → 无上升沿 → 不重复 start
    prev, cur = _press(BUTTON_A)
    assert decide_vr_command("IDLE", prev, cur) == "start"
    assert decide_vr_command("IDLE", cur, cur) is None


def test_release_then_press_again_fires_again():
    prev, cur = _press(BUTTON_A)
    assert decide_vr_command("IDLE", prev, cur) == "start"
    # 松开再按 → 新的上升沿 → 再次 start（同一段内被门控，无副作用）
    assert decide_vr_command("IDLE", cur, prev) is None
    assert decide_vr_command("IDLE", prev, cur) == "start"


def test_stick_press_while_recording_discards():
    prev, cur = _press(BUTTON_STICK_PRESS)
    assert decide_vr_command("RECORDING", prev, cur) == "discard"
    assert decide_vr_command("PAUSED", prev, cur) == "discard"


def test_stick_press_in_idle_gated():
    # 没有正在录的段可丢：IDLE 下摇杆按下不动作
    prev, cur = _press(BUTTON_STICK_PRESS)
    assert decide_vr_command("IDLE", prev, cur) is None


def test_b_in_recording_stop():
    prev, cur = _press(BUTTON_B)
    assert decide_vr_command("RECORDING", prev, cur) == "stop"
    assert decide_vr_command("PAUSED", prev, cur) == "stop"


def test_b_in_idle_gated():
    prev, cur = _press(BUTTON_B)
    assert decide_vr_command("IDLE", prev, cur) is None


def test_state_unknown_no_action():
    # 尚未收到 /data_collect/state（采集节点没跑）：任何键都不动作
    for idx in (BUTTON_STICK_PRESS, BUTTON_A, BUTTON_B):
        prev, cur = _press(idx)
        assert decide_vr_command(None, prev, cur) is None


def test_simultaneous_press_takes_lowest_valid_index():
    # A+B 同帧按下（IDLE）：A 合法（start）优先
    cur = _ZERO.copy()
    cur[BUTTON_A] = 1
    cur[BUTTON_B] = 1
    assert decide_vr_command("IDLE", _ZERO, cur) == "start"
    # A+B 同帧（RECORDING）：A 不合法跳过 → B 生效（stop）
    assert decide_vr_command("RECORDING", _ZERO, cur) == "stop"
    # 摇杆+ A 同帧（RECORDING）：A 不合法跳过 → 摇杆生效（discard）
    cur2 = _ZERO.copy()
    cur2[BUTTON_A] = 1
    cur2[BUTTON_STICK_PRESS] = 1
    assert decide_vr_command("RECORDING", _ZERO, cur2) == "discard"


def test_short_buttons_array_no_crash():
    # mocap 帧异常/缺位：不越界；可判读的位按正常逻辑走（短帧仍含真实按键位）
    assert decide_vr_command("IDLE", [], [1]) == "start"  # A 上升沿且 IDLE 合法
    assert decide_vr_command("RECORDING", [0, 1], [1, 1]) is None  # A 上升沿但录制中 → 门控
    assert decide_vr_command("IDLE", [0, 1], [1, 1]) == "start"  # A/B 上升沿，A 先命中


def test_other_buttons_ignored():
    prev, cur = _press(3)  # menu
    assert decide_vr_command("IDLE", prev, cur) is None
    prev, cur = _press(5)  # gripClick
    assert decide_vr_command("RECORDING", prev, cur) is None


# ---------------- decide_release_or_collect：推理 HUMAN 时 A=release ----------------

def test_a_in_human_releases():
    prev, cur = _press(BUTTON_A)
    assert decide_release_or_collect("human", "IDLE", prev, cur) == "release"
    # 采集节点未跑也照样 release（release 属于推理域，不依赖采集状态）
    assert decide_release_or_collect("human", None, prev, cur) == "release"


def test_a_not_in_human_stays_collection():
    prev, cur = _press(BUTTON_A)
    # 推理活跃（policy）但非 HUMAN → A 仍是采集 start（仅 IDLE 合法）
    assert decide_release_or_collect("policy", "IDLE", prev, cur) == "start"
    assert decide_release_or_collect("playback", "IDLE", prev, cur) == "start"
    # 推理 HUMAN 但 A 在录制中 → 仍 release（HUMAN 时 A 归推理域）
    assert decide_release_or_collect("human", "RECORDING", prev, cur) == "release"
    # 无策略节点（activity=None）→ 纯采集路由
    assert decide_release_or_collect(None, "IDLE", prev, cur) == "start"
    assert decide_release_or_collect(None, "RECORDING", prev, cur) is None


def test_a_hold_in_human_not_repeated():
    prev, cur = _press(BUTTON_A)
    assert decide_release_or_collect("human", "IDLE", prev, cur) == "release"
    assert decide_release_or_collect("human", "IDLE", cur, cur) is None


def test_b_and_stick_unchanged_in_human():
    # HUMAN 只劫持 A；B/摇杆仍走采集路由（stop/discard）
    prev, cur = _press(BUTTON_B)
    assert decide_release_or_collect("human", "RECORDING", prev, cur) == "stop"
    prev, cur = _press(BUTTON_STICK_PRESS)
    assert decide_release_or_collect("human", "RECORDING", prev, cur) == "discard"


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-q", "-p", "no:anyio"]))
