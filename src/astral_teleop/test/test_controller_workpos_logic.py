"""controller_workpos_logic 纯逻辑单元测试（纯 python，无需 ROS）。

覆盖 decide_workpos：上升沿触发、长按不重复、录制中（RECORDING/PAUSED/SAVING）
忽略、无数采节点（state=None）放行、短 buttons 数组不越界、非 X 键不触发。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_teleop.controller_workpos_logic import (  # noqa: E402
    BUTTON_X,
    decide_workpos,
)

# 便捷构造：6 位 buttons 向量，X=index 0
_ZERO = [0, 0, 0, 0, 0, 0]


def _press(idx: int, prev: list[int] | None = None) -> tuple[list[int], list[int]]:
    cur = _ZERO.copy()
    cur[idx] = 1
    return (_ZERO if prev is None else prev), cur


def test_x_rising_edge_triggers() -> None:
    prev, cur = _press(BUTTON_X)
    assert decide_workpos("IDLE", prev, cur) is True


def test_x_hold_does_not_repeat() -> None:
    prev, cur = _press(BUTTON_X)
    # 长按不重复：prev 已按下，cur 仍按下 → 不触发
    assert decide_workpos("IDLE", cur, cur) is False


def test_release_then_press_again_triggers() -> None:
    # 抬起后再按下 → 触发（下一段循环可再次使用）
    prev, cur = _press(BUTTON_X)
    assert decide_workpos("IDLE", cur, prev) is False  # 抬起沿不触发
    assert decide_workpos("IDLE", prev, cur) is True  # 再按下触发


def test_x_while_recording_ignored() -> None:
    prev, cur = _press(BUTTON_X)
    assert decide_workpos("RECORDING", prev, cur) is False


def test_x_while_paused_ignored() -> None:
    prev, cur = _press(BUTTON_X)
    assert decide_workpos("PAUSED", prev, cur) is False


def test_x_while_saving_ignored() -> None:
    prev, cur = _press(BUTTON_X)
    assert decide_workpos("SAVING", prev, cur) is False


def test_x_with_no_collect_running_allowed() -> None:
    # 无数采节点运行（state=None）→ 纯遥操场景 X 照常生效
    prev, cur = _press(BUTTON_X)
    assert decide_workpos(None, prev, cur) is True


def test_short_buttons_array_no_index_error() -> None:
    # buttons 数组比约定短（缺位）按 0 处理，不越界、不崩
    assert decide_workpos("IDLE", [0], [1]) is True   # cur=[1] → X 按下
    assert decide_workpos("IDLE", [], []) is False     # 空数组 → 无按键
    assert decide_workpos("RECORDING", [0], [1]) is False  # 录制中仍忽略


def test_non_x_button_does_not_trigger() -> None:
    # Y 键（index 1）按下，X=0 未按 → 不触发
    prev, cur = _press(1)
    assert decide_workpos("IDLE", prev, cur) is False


def test_out_of_range_button_index_false() -> None:
    prev, cur = _press(BUTTON_X)
    assert decide_workpos("IDLE", prev, cur, button_index=99) is False
