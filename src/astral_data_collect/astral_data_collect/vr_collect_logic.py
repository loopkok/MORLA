"""VR 采集控制——右控制器按键 → /data_collect/control 命令的纯决策逻辑。

无 ROS 依赖（节点在 ``vr_collect_control.py`` 里订阅 Joy / 状态话题后调用），
单独成模块以便离线单测。按键语义与 keyboard_controller 一致：
  s=start  q=stop&save  d=discard

Quest Touch 右手柄按键位（quest3_hand_mocap 的 Joy buttons，6 位 mask）：
  [0]=primary(A)  [1]=secondary(B)  [2]=stickPress(摇杆按下)  ...
"""

from __future__ import annotations

from typing import Optional, Sequence

# 右手柄按键映射：A=开始录制，B=停止保存，摇杆按下=丢弃当前段。
# 值 = (命令, 合法状态集合)。start 仅 IDLE 有效；stop/discard 仅录制中有效
# （与 data_collect_node 状态机一致：start 在非 IDLE、stop/discard 在 IDLE 会
# 被忽略并告警，桥先在本地门控，避免无效命令刷屏）。
BUTTON_STICK_PRESS = 2
BUTTON_A = 0  # primary（右手柄是 A 键；左手柄是 X 键，本桥只订右手）
BUTTON_B = 1  # secondary（右手柄是 B 键）

_RIGHT_CMDS: dict[int, tuple[str, tuple[str, ...]]] = {
    BUTTON_A: ("start", ("IDLE",)),
    BUTTON_B: ("stop", ("RECORDING", "PAUSED")),
    BUTTON_STICK_PRESS: ("discard", ("RECORDING", "PAUSED")),
}


def decide_vr_command(
    state: Optional[str],
    prev_buttons: Sequence[int],
    cur_buttons: Sequence[int],
) -> Optional[str]:
    """上升沿 + 状态门控 → 要发布的 /data_collect/control 命令；无动作返回 None。

    - ``state=None``（尚未收到 /data_collect/state，采集节点可能没跑）→ 一律
      不动作；
    - 长按不重复：只认 0→1 上升沿；
    - 同帧多个键同时按下：按键号小的优先（A > B > 摇杆按下，按键号升序遍历，
      已命中的直接返回；若低号键在当状态不合法则跳过继续判下一个）；
    - buttons 数组比约定短（缺位）按 0 处理，不越界。
    """
    if state is None:
        return None
    prev = list(prev_buttons) + [0] * 6
    cur = list(cur_buttons) + [0] * 6
    for idx in sorted(_RIGHT_CMDS):
        cmd, valid_states = _RIGHT_CMDS[idx]
        if not prev[idx] and cur[idx] and state in valid_states:
            return cmd
    return None


def decide_release_or_collect(
    activity: Optional[str],
    collect_state: Optional[str],
    prev_buttons: Sequence[int],
    cur_buttons: Sequence[int],
) -> Optional[str]:
    """右手 A 键路由：推理 HUMAN → "release"，否则走采集路由（start）。

    - ``activity`` 来自 /policy_inference/state（None=策略节点未跑/无状态）；
    - 推理 HUMAN 时 A 键不再发采集 start，改为发给 /policy_inference/cmd 的
      "release"（把控制权交还策略/回放）；
    - 其余（含推理其它状态、无策略节点）完全走 ``decide_vr_command``：
      A=start（仅 IDLE）、B=stop、摇杆=discard。
    """
    prev = list(prev_buttons) + [0] * 6
    cur = list(cur_buttons) + [0] * 6
    if activity == "human" and not prev[BUTTON_A] and cur[BUTTON_A]:
        return "release"
    return decide_vr_command(collect_state, prev_buttons, cur_buttons)
