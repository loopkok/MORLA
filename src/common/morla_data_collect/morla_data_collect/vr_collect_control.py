"""VR 采集控制：右手柄按键 → /data_collect/control 命令（推理感知）。

单人数据采集时手不离手柄即可控制录制（键盘控制器的手动替代）：
  Quest 右手柄  A 键      → start（开始录制，仅 IDLE 有效）
                B 键      → stop（结束并保存当前段，仅录制中有效）
                摇杆按下  → discard（丢弃当前段，仅录制中有效）

推理感知（HITL）：/policy_inference/state 的 activity==human 时，A 键改发
/policy_inference/cmd = "release"（把控制权交还策略/回放），不再发采集 start。

上升沿触发（长按不重复）；本地按 /data_collect/state 做状态门控——非法状态或
采集节点未运行（未收到 state）时按键静默忽略（info 日志），不给采集节点发
空命令刷 warning。与 web 数采卡片 / 键盘控制器并存：都发离散一次性命令，
采集节点状态机幂等。键位 → 命令的纯决策在 ``vr_collect_logic``（可离线单测）。

用法：
  ros2 run morla_data_collect vr_collect_control        # 随 data_collect.launch.py 同启
"""

from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from morla_data_collect.vr_collect_logic import (
    BUTTON_A,
    BUTTON_B,
    BUTTON_STICK_PRESS,
    decide_release_or_collect,
    decide_vr_command,
)
from morla_data_collect.ownership import PolicyLease

_CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# mocap 的 Joy 是 BEST_EFFORT 流（只留最新帧）；订阅方须兼容（不能 RELIABLE）。
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 命令 → 触发键名（start/stop/discard 与三个键一一对应，见 vr_collect_logic）
_CMD_BTN_NAMES = {
    "start": "A 键",
    "stop": "B 键",
    "discard": "摇杆按下",
    "release": "A 键",
}
_BTN_NAMES = {
    BUTTON_STICK_PRESS: "摇杆按下",
    BUTTON_A: "A 键",
    BUTTON_B: "B 键",
}


class VrCollectControl(Node):
    def __init__(self) -> None:
        super().__init__("data_collect_vr_control")
        self.declare_parameter("policy_state_topic", "/policy_inference/state")
        self.declare_parameter("policy_cmd_topic", "/policy_inference/cmd")
        self._ctrl_pub = self.create_publisher(
            String, "/data_collect/control", _CONTROL_QOS
        )
        self._state: str | None = None  # None = 尚未收到采集节点状态
        self.create_subscription(
            String, "/data_collect/state", self._on_state, _LATCHED_QOS
        )
        # 推理 HUMAN 时 A 键 → release。
        self._activity: str | None = None
        self._policy_lease = PolicyLease()
        self._policy_cmd_topic = str(self.get_parameter("policy_cmd_topic").value)
        self._policy_cmd_pub = self.create_publisher(
            String, self._policy_cmd_topic, _CONTROL_QOS
        )
        self.create_subscription(
            String,
            str(self.get_parameter("policy_state_topic").value),
            self._on_policy_state,
            _LATCHED_QOS,
        )
        self.create_subscription(
            Joy, "quest3/right_controller_joy", self._on_joy, _SENSOR_QOS
        )
        self._prev_buttons: list[int] = [0] * 6
        self.get_logger().info(
            "VR 采集控制就绪：右手柄 A=start/release  B=stop&save  摇杆按下=discard"
            "（订 quest3/right_controller_joy + /data_collect/state + "
            f"{str(self.get_parameter('policy_state_topic').value)}）"
        )

    def _on_state(self, msg: String) -> None:
        try:
            self._state = json.loads(msg.data).get("state")
        except Exception:  # noqa: BLE001
            self._state = None
        self.get_logger().info(f"data_collect state = {self._state}")

    def _on_policy_state(self, msg: String) -> None:
        self._policy_lease.update(msg.data)
        self._activity = self._policy_lease.activity

    def _send(self, cmd: str, to_policy: bool = False) -> None:
        msg = String()
        msg.data = cmd
        (self._policy_cmd_pub if to_policy else self._ctrl_pub).publish(msg)
        self.get_logger().info(
            f"-> {cmd}（{_CMD_BTN_NAMES.get(cmd, cmd)}，state={self._state}）"
        )

    def _on_joy(self, msg: Joy) -> None:
        cur = list(msg.buttons) if msg.buttons else []
        cur += [0] * (6 - len(cur))
        cmd = decide_release_or_collect(
            self._activity, self._state, self._prev_buttons, cur
        )
        if cmd == "release":
            # The always-running controller_start_gate is the sole release
            # publisher. This bridge only suppresses A=start during HUMAN.
            pass
        elif cmd is not None:
            self._send(cmd)
        else:
            # 有键按下但被门控 / 采集节点未运行：只打 info 不上发——采集节点
            # 状态机已会拒绝非法命令并 warning，桥先本地过滤避免刷屏。
            pressed = [
                b for b in (BUTTON_STICK_PRESS, BUTTON_A, BUTTON_B)
                if not self._prev_buttons[b] and cur[b]
            ]
            if pressed:
                names = "/".join(_BTN_NAMES[b] for b in pressed)
                reason = (
                    "采集节点未运行？" if self._state is None
                    else f"state={self._state} 不合法"
                )
                self.get_logger().info(f"[vr] 忽略 {names}：{reason}")
        self._prev_buttons = cur


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VrCollectControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
