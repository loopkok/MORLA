"""Touch 左手柄 X 键 → 段间回位闸门（停止跟随 VR + 直达回到工作位）。

单人数采时，段与段之间需要把手从手柄上解放出来去重新摆放物品：按左手柄
X 键（primary, buttons[0]）→ 发布 ``/teleop/disarm``（停止跟随 VR）+ 一次
``/teleop/init_direct``（臂节点自 disarm 并**不经 init_waypoints**、直接关节
空间插补到 init_pose——即 web 数采卡片「段间回位」按钮的信号序列；系统 tab
「工作位」按钮仍走途经点路径 /teleop/init）。与 ``controller_start_gate``
（grip → start）对称，随遥操栈启动，纯遥操场景同样生效。

录制保护：订阅 /data_collect/state（latched），录制中（RECORDING/PAUSED/
SAVING）按 X 忽略并节流告警——防止手臂回位毁掉正在录的 episode。采集节点
未运行（state=None）时放行。键位 → 是否下发的纯决策在
``controller_workpos_logic``（可离线单测）。
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from morla_bringup.controller_workpos_logic import (
    BLOCK_STATES,
    BUTTON_X,
    decide_workpos,
)
from morla_core.ownership import PolicyLease, fresh_header

# /teleop/disarm 用 latched（对齐 monitor_node：晚启动臂节点也能收到）；
# /teleop/init 用 VOLATILE 一次性（对齐 monitor_node：晚启动节点不得被历史
# 信号误触发回位——与 /teleop/start 同契约）。
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
# mocap 的 Joy 是 BEST_EFFORT 流（只留最新帧）；订阅方须兼容（不能 RELIABLE）。
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 录制中按 X 的告警节流间隔（Joy 帧率 ~30Hz，避免刷屏）。
_WARN_THROTTLE_S = 5.0

# disarm → init 的间隔。不能连发：/teleop/disarm 会取消进行中的 homing，而
# /teleop/init 在 homing 中会被拒绝（_go_init 的 "homing/park already in
# progress"）——DDS 跨话题无保序，直接连发可能让臂节点先收 init 启动回位、
# 再收 disarm 取消回位（X 键静默失效）。镜像 web workpos 端点的
# disarm → sleep(0.1) → init 时序。
_INIT_DELAY_S = 0.1


class ControllerWorkposGate(Node):
    def __init__(self) -> None:
        super().__init__("controller_workpos_gate")

        self.declare_parameter("joy_topic", "quest3/left_controller_joy")
        self.declare_parameter("button_index", BUTTON_X)
        self.declare_parameter("disarm_topic", "/teleop/disarm")
        # 段间回位 = 直达（不经 init_waypoints）；系统 tab「工作位」仍走
        # /teleop/init（途经点）。X 的语义是"段间快速回工作位"→ 直达。
        self.declare_parameter("init_topic", "/teleop/init_direct")
        self.declare_parameter("collect_state_topic", "/data_collect/state")
        self.declare_parameter("use_control_coordinator", False)

        joy_topic = str(self.get_parameter("joy_topic").value).strip()
        self.button_index = int(self.get_parameter("button_index").value)
        disarm_topic = str(self.get_parameter("disarm_topic").value).strip()
        init_topic = str(self.get_parameter("init_topic").value).strip()
        state_topic = str(self.get_parameter("collect_state_topic").value).strip()
        self._coordinator_enabled = bool(self.get_parameter("use_control_coordinator").value)
        self._control_client = None
        self._control_pending = False

        self._pub_disarm = self.create_publisher(Bool, disarm_topic, _LATCHED_QOS)
        self._pub_init = self.create_publisher(Bool, init_topic, _VOLATILE_QOS)
        self.create_subscription(Joy, joy_topic, self._on_joy, _SENSOR_QOS)
        self._collect_state: str | None = None  # None = 未收到采集节点状态
        if state_topic:
            self.create_subscription(
                String, state_topic, self._on_state, _LATCHED_QOS
            )
        self._prev_buttons: list[int] = [0] * 6
        self._last_blocked_warn = 0.0
        self._init_timer = None  # 一次性定时器：disarm 后 0.1s 发 init（见 _INIT_DELAY_S）
        self._policy_lease = PolicyLease()
        self.create_subscription(String, '/policy_inference/state', self._on_policy_state, _LATCHED_QOS)

    def _on_policy_state(self, msg):
        self._policy_lease.update(msg.data)
        if not self._policy_lease.manual_allowed and self._init_timer is not None:
            self._init_timer.cancel()
            self._init_timer = None

    def _on_state(self, msg: String) -> None:
        try:
            self._collect_state = json.loads(msg.data).get("state")
        except Exception:  # noqa: BLE001
            self._collect_state = None

    def _on_joy(self, msg: Joy) -> None:
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        cur = list(msg.buttons) if msg.buttons else []
        cur += [0] * (6 - len(cur))
        fire = decide_workpos(
            self._collect_state, self._prev_buttons, cur, self.button_index
        )
        lease = getattr(self, '_policy_lease', None)
        if lease is not None and not lease.manual_allowed:
            fire = False
        if fire:
            if self._coordinator_enabled:
                self._request_coordinated_workpos()
            else:
                self._pub_disarm.publish(Bool(data=True))
                # 先 disarm、延迟后再 init（时序说明见 _INIT_DELAY_S）：重复按 X
                # 时取消上一个待发 init（每次 X 都以最新一次 disarm 为基准）。
                if self._init_timer is not None:
                    self._init_timer.cancel()
                self._init_timer = self.create_timer(_INIT_DELAY_S, self._fire_init)
                self.get_logger().info(
                    f"X 段间回位 → /teleop/disarm（{_INIT_DELAY_S}s 后 /teleop/init）"
                    f"（state={self._collect_state}）"
                )
        else:
            rising = (
                0 <= self.button_index < len(cur)
                and not self._prev_buttons[self.button_index]
                and bool(cur[self.button_index])
            )
            if rising and self._collect_state in BLOCK_STATES:
                now = time.time()
                if now - self._last_blocked_warn >= _WARN_THROTTLE_S:
                    self._last_blocked_warn = now
                    self.get_logger().warn(
                        f"X 段间回位被忽略：data_collect 状态 {self._collect_state}"
                        "（录制中/保存中，防手臂回位毁段；先 B 停止保存再按 X）"
                    )
        self._prev_buttons = cur

    def _request_coordinated_workpos(self) -> None:
        if self._control_pending:
            return
        if self._control_client is None:
            self._control_client = self.create_client(Trigger, "/morla/control/workpos_direct")
        if not self._control_client.service_is_ready():
            self.get_logger().warning(
                "MORLA control coordinator is not ready; X request was not sent"
            )
            return
        self._control_pending = True
        future = self._control_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_coordinated_workpos_result)

    def _on_coordinated_workpos_result(self, future) -> None:
        self._control_pending = False
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"X workpos accepted: {response.message}")
            else:
                self.get_logger().warning(f"X workpos rejected: {response.message}")
        except Exception as exc:  # service disconnect / callback failure
            self.get_logger().error(f"X workpos service failed: {exc}")

    def _fire_init(self) -> None:
        """一次性定时器回调：disarm 已先发出，现在发 /teleop/init 触发回位。"""
        if self._init_timer is not None:
            self._init_timer.cancel()
            self._init_timer = None
        lease = getattr(self, '_policy_lease', None)
        if (self._collect_state in BLOCK_STATES
                or (lease is not None and not lease.manual_allowed)):
            return
        self._pub_init.publish(Bool(data=True))
        self.get_logger().info("X 段间回位 → /teleop/init（工作位轨迹启动）")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ControllerWorkposGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
