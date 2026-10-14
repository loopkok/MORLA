"""VR 采集控制节点集成测试（rclpy；需 ROS 环境，同 test_node_guards）。

覆盖真实话题契约：/data_collect/state（latched）驱动门控、
quest3/right_controller_joy → /data_collect/control 命令序列。
需 ROS 源环境运行；无 ROS 时整文件 skip。

当前映射：A=start（仅 IDLE）、B=stop&save（录制中）、摇杆按下=discard（录制中）。
"""

import os
import sys

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from astral_data_collect.vr_collect_control import VrCollectControl  # noqa: E402
from astral_data_collect.vr_collect_logic import (  # noqa: E402
    BUTTON_A,
    BUTTON_B,
    BUTTON_STICK_PRESS,
)

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
    """起 vr 节点 + 状态发布 + 控制订阅，喂 Joy、收命令。"""

    def __init__(self):
        self.node = VrCollectControl()
        self.state_pub = self.node.create_publisher(
            String, "/data_collect/state", _LATCHED
        )
        self.received: list[str] = []
        self._sub = self.node.create_subscription(
            String, "/data_collect/control", self._on_cmd, 10
        )

    def _on_cmd(self, msg: String) -> None:
        self.received.append(msg.data)

    def set_state(self, state: str) -> None:
        import json

        self.state_pub.publish(String(data=json.dumps({"state": state})))
        for _ in range(50):  # latched state 送达节点订阅回调
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def press(self, btn: int, hold=False) -> None:
        """按一次（默认松手）；hold=True 表示长按两帧（验证不重复）。"""
        self.node._on_joy(_release())
        self.node._on_joy(_joy(btn))
        if hold:
            self.node._on_joy(_joy(btn))
        self.node._on_joy(_release())
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def destroy(self) -> None:
        self.node.destroy_node()


def test_idle_a_starts_b_and_stick_gated(ros_context):
    h = _Harness()
    try:
        h.set_state("IDLE")
        h.press(BUTTON_A)
        assert h.received == ["start"]
        # IDLE 下 B/摇杆不发命令（无段可停/可丢）
        h.press(BUTTON_B)
        h.press(BUTTON_STICK_PRESS)
        assert h.received == ["start"]
    finally:
        h.destroy()


def test_recording_b_stop_stick_discard_a_gated(ros_context):
    h = _Harness()
    try:
        h.set_state("IDLE")
        h.press(BUTTON_A)
        assert h.received == ["start"]
        h.set_state("RECORDING")
        h.press(BUTTON_STICK_PRESS)
        h.press(BUTTON_B)
        assert h.received == ["start", "discard", "stop"]
        # 录制中 A 键被门控（不重复 start）
        h.press(BUTTON_A)
        assert h.received == ["start", "discard", "stop"]
    finally:
        h.destroy()


def test_hold_does_not_repeat(ros_context):
    h = _Harness()
    try:
        h.set_state("IDLE")
        h.press(BUTTON_A, hold=True)
        assert h.received == ["start"]
    finally:
        h.destroy()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q", "-p", "no:anyio"]))
