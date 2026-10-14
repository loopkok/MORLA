"""controller_workpos_gate 节点集成测试（rclpy；需 ROS 环境，同 test_node_guards）。

覆盖真实话题契约：/data_collect/state（latched）驱动门控、
quest3/left_controller_joy X 键上升沿 → /teleop/disarm + /teleop/init_direct。
需 ROS 源环境运行；无 ROS 时整文件 skip（sys.exit(0)）。

当前映射：X（左手柄 primary, buttons[0]）= 段间回位（disarm + **直达**
init_direct，不经 init_waypoints）；录制中（RECORDING/PAUSED/SAVING）忽略；
无数采节点（state=None）放行。

测试装置要点（对抗性审查沉淀）：
- 捕获订阅放**独立 client 节点**（非闸门同节点），贴近生产"闸门 vs 臂节点
  跨进程"布局；
- Joy 用**直接驱动 _on_joy**（非真实话题）：闸门在 spin 回调内发布时同进程
  intra-process 会双投（intra 快路径 + DDS 环回各一次）——从测试主线程直接
  调用等价于"回调外发布"，单投稳定；真实 DDS 全路径由跨进程冒烟背书
  （ros2 run 独立进程 + rclpy 客户端，见 astral_ws/CHANGELOG）；
- 断言 disarm 必须先于 init（0.1s 定时器时序）。
"""

from __future__ import annotations

import os
import sys

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from astral_teleop.controller_workpos_gate import ControllerWorkposGate  # noqa: E402
from astral_teleop.controller_workpos_logic import BUTTON_X  # noqa: E402

# 桥订 /data_collect/state 用 TRANSIENT_LOCAL（对齐采集节点 latched 发布）
_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.fixture()
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _joy(btn: int) -> Joy:
    msg = Joy()
    msg.buttons = [0] * 6
    msg.buttons[btn] = 1
    return msg


def _release() -> Joy:
    msg = Joy()
    msg.buttons = [0] * 6
    return msg


class _Harness:
    """闸门节点 + 独立 client 捕获节点（贴近生产跨进程布局）。"""

    def __init__(self):
        self.gate = ControllerWorkposGate()
        self.client = Node("workpos_test_client")
        self.state_pub = self.gate.create_publisher(
            String, "/data_collect/state", _LATCHED
        )
        self.init_msgs: list[Bool] = []
        self.disarm_msgs: list[Bool] = []
        # 到达顺序（断言 disarm 必须先于 init——时序是闸门正确性的关键）
        self.order: list[str] = []
        self.client.create_subscription(
            Bool, "/teleop/init_direct", self._on_init, 10
        )
        self.client.create_subscription(
            Bool, "/teleop/disarm", self._on_disarm, 10
        )
        # 发现预热：让 client 捕获订阅与闸门发布建立匹配，避免 latched 补投歧义
        self._spin(0.6)

    def _on_init(self, msg: Bool) -> None:
        self.init_msgs.append(msg)
        self.order.append("init")

    def _on_disarm(self, msg: Bool) -> None:
        self.disarm_msgs.append(msg)
        self.order.append("disarm")

    def _spin(self, duration: float) -> None:
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.gate, timeout_sec=0.005)
            rclpy.spin_once(self.client, timeout_sec=0.005)

    def _spin_until(self, predicate, timeout_s: float = 2.0) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self.gate, timeout_sec=0.005)
            rclpy.spin_once(self.client, timeout_sec=0.005)

    def set_state(self, state: str) -> None:
        import json

        self.state_pub.publish(String(data=json.dumps({"state": state})))
        self._spin(0.3)  # latched state 送达闸门订阅回调

    def press_x(self, hold: bool = False) -> None:
        """直接驱动 _on_joy 触发一次 X（默认松手）；hold=True 长按两帧。

        注意不走真实 Joy 话题：闸门在 spin 回调内发布 disarm/init 时，同进程
        intra-process 会双投（intra 快路径 + DDS 环回各一次）——从测试主线程
        直接调用 _on_joy 等价于"从回调外发布"，单投稳定。真实 DDS 全路径由
        跨进程冒烟（ros2 run 独立进程）背书，见 CHANGELOG。
        """
        self.gate._on_joy(_release())
        self.gate._on_joy(_joy(BUTTON_X))
        if hold:
            self.gate._on_joy(_joy(BUTTON_X))
        self.gate._on_joy(_release())
        # 等 0.1s 一次性定时器触发 init（disarm 即时、init 延迟）全部送达
        self._spin_until(lambda: len(self.init_msgs) >= 1)

    def destroy(self) -> None:
        self.client.destroy_node()
        self.gate.destroy_node()


def test_idle_x_publishes_disarm_and_init(ros_context):
    h = _Harness()
    try:
        h.set_state("IDLE")
        h.press_x()
        assert len(h.disarm_msgs) == 1
        assert len(h.init_msgs) == 1
        # disarm 必须先于 init（背靠背连发会让臂节点先收 init 启动回位、
        # 再收 disarm 取消回位——闸门必须镜像 web workpos 的时序）
        assert h.order == ["disarm", "init"]
    finally:
        h.destroy()


def test_recording_x_ignored(ros_context):
    h = _Harness()
    try:
        h.set_state("RECORDING")
        h.press_x()
        assert h.disarm_msgs == []
        assert h.init_msgs == []
    finally:
        h.destroy()


def test_paused_and_saving_x_ignored(ros_context):
    for state in ("PAUSED", "SAVING"):
        h = _Harness()
        try:
            h.set_state(state)
            h.press_x()
            assert h.disarm_msgs == [], f"state={state} 不应 disarm"
            assert h.init_msgs == [], f"state={state} 不应 init"
        finally:
            h.destroy()


def test_hold_does_not_repeat(ros_context):
    h = _Harness()
    try:
        h.set_state("IDLE")
        h.press_x(hold=True)
        assert len(h.disarm_msgs) == 1
        assert len(h.init_msgs) == 1
    finally:
        h.destroy()


def test_repeat_x_before_init_fires_cancels_pending(ros_context):
    """连按两次 X（第一次的 init 定时器尚未触发）→ 旧 init 被取消，只发一次。

    帧级精确控制直接驱动 _on_joy：depth-1 真实话题连发会被最新帧覆盖，表达
    不了"两次上升沿"（这正是本场景的本质），故不走 topic 路径。
    """
    h = _Harness()
    try:
        h.set_state("IDLE")
        h.gate._on_joy(_release())
        h.gate._on_joy(_joy(BUTTON_X))  # fire #1 → disarm + timer1
        h.gate._on_joy(_release())
        h.gate._on_joy(_joy(BUTTON_X))  # fire #2 → disarm + 取消 timer1 + timer2
        h.gate._on_joy(_release())
        h._spin_until(lambda: len(h.init_msgs) >= 1)
        assert len(h.disarm_msgs) == 2
        assert len(h.init_msgs) == 1  # timer1 被取消，只有 timer2 的 init 发出
        assert h.order == ["disarm", "disarm", "init"]
    finally:
        h.destroy()


def test_no_collect_state_x_allowed(ros_context):
    """无数采节点运行（state=None）→ 纯遥操场景 X 照常回位。"""
    h = _Harness()
    try:
        h.press_x()
        assert len(h.disarm_msgs) == 1
        assert len(h.init_msgs) == 1
    finally:
        h.destroy()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q", "-p", "no:anyio"]))
