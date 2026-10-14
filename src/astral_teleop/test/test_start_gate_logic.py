"""controller_start_gate 纯路由逻辑单元测试（纯 python，无需 ROS）。

覆盖 decide_start_action：推理活跃（policy/playback）→ "takeover"（grip 改
HITL 接管，避免与策略双写 joint_commands）；其余（HUMAN/IDLE/无策略节点）→
"teleop_start"（现状 grip 启动遥操）。以及 decide_release_action：推理 HUMAN
时右手 A 上升沿 → "release"（交还控制权）。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_teleop.start_gate_logic import (  # noqa: E402
    RIGHT_A,
    decide_release_action,
    decide_start_action,
)

_ZERO = [0, 0, 0, 0, 0, 0]


def test_policy_active_takeover() -> None:
    assert decide_start_action("policy") == "takeover"
    assert decide_start_action("playback") == "takeover"


def test_paused_policy_still_takeover() -> None:
    # 暂停态 activity 仍为 policy（FSM 保持 activity 语义）→ 仍是接管
    assert decide_start_action("policy") == "takeover"


def test_human_stays_teleop_start() -> None:
    # HUMAN 时 grip 维持现状（重标定），不劫持
    assert decide_start_action("human") == "teleop_start"


def test_idle_and_none_stay_teleop_start() -> None:
    assert decide_start_action(None) == "teleop_start"  # 策略节点未跑/无状态
    assert decide_start_action("idle") == "teleop_start"


# ---------------- decide_release_action：右手 A 在 HUMAN 时 = release ----------------

def test_a_in_human_releases() -> None:
    cur = _ZERO.copy()
    cur[RIGHT_A] = 1
    assert decide_release_action("human", _ZERO, cur) is True


def test_a_hold_in_human_not_repeated() -> None:
    cur = _ZERO.copy()
    cur[RIGHT_A] = 1
    assert decide_release_action("human", _ZERO, cur) is True
    assert decide_release_action("human", cur, cur) is False  # 长按不重复


def test_a_not_in_human_no_release() -> None:
    cur = _ZERO.copy()
    cur[RIGHT_A] = 1
    for act in ("policy", "playback", "idle", None, "HUMAN"):
        assert decide_release_action(act, _ZERO, cur) is False


def test_other_right_buttons_ignored() -> None:
    for idx in (1, 2, 3, 5):  # B/摇杆/menu/gripClick
        cur = _ZERO.copy()
        cur[idx] = 1
        assert decide_release_action("human", _ZERO, cur) is False


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-q", "-p", "no:anyio"]))
