"""左手 X 键「段间回位」闸门的纯决策逻辑。

Quest Touch 左手柄 X 键 = primary（buttons[0]，与右手柄 A 键同一位，但左右手柄
各自独立发布）。按下 X = 停止跟随 VR 遥操并回到工作位（发 /teleop/disarm +
/teleop/init）。单独成模块以便离线单测；节点在 ``controller_workpos_gate.py``
里订阅 Joy / /data_collect/state 后调用。

无 ROS 依赖。语义与 ``vr_collect_logic`` 一致：上升沿触发、状态门控、缺位不越界。
"""

from __future__ import annotations

from typing import Optional, Sequence

# 左手柄 X 键位（左控制器 primary；右控制器 primary 是 A 键，本闸门只订左手）。
BUTTON_X = 0
# 数采节点这些状态下按 X 无效：手臂回位会毁掉正在录的 episode。
# IDLE 以及 state=None（无数采节点运行 / 纯遥操）放行。
BLOCK_STATES: tuple[str, ...] = ("RECORDING", "PAUSED", "SAVING")


def decide_workpos(
    collect_state: Optional[str],
    prev_buttons: Sequence[int],
    cur_buttons: Sequence[int],
    button_index: int = BUTTON_X,
) -> bool:
    """上升沿 + 状态门控 → 是否发布回位信号（disarm + init）。

    - ``collect_state=None``（尚未收到 /data_collect/state，采集节点没跑）→
      放行（纯遥操场景 X 照常生效）；
    - 长按不重复：只认 0→1 上升沿；
    - buttons 数组比约定短（缺位）按 0 处理，不越界；button_index 越界返回 False。
    """
    prev = list(prev_buttons) + [0] * 6
    cur = list(cur_buttons) + [0] * 6
    if button_index < 0 or button_index >= len(cur):
        return False
    if collect_state in BLOCK_STATES:
        return False
    return (not prev[button_index]) and bool(cur[button_index])
