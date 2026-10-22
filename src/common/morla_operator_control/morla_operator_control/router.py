"""Normalize input intent, then use the existing coordinator/collector APIs."""
import json
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .commands import SeenRequests, validate_request

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TOPIC = "/morla/operator/command"
EVENTS = "/morla/operator/events"


class OperatorRouter(Node):
    def __init__(self):
        super().__init__("morla_operator_control")
        if self.count_publishers("/morla/operator/state"):
            raise RuntimeError("another operator router is already running")
        self._control = self._collect = {}
        self._control_at = self._collect_at = 0.0
        self._seen = SeenRequests()
        self._pending = None
        self._saving = None
        self._discard = None
        self._service_clients = {}
        self._last_event = None
        self._event_pub = self.create_publisher(String, EVENTS, 20)
        self._state_pub = self.create_publisher(String, "/morla/operator/state", LATCHED)
        self._collect_pub = self.create_publisher(String, "/data_collect/control", 10)
        self.create_subscription(String, TOPIC, self._on_command, 10)
        self.create_subscription(String, "/morla/control/state", self._on_control, LATCHED)
        self.create_subscription(String, "/data_collect/state", self._on_collect, LATCHED)
        self.create_subscription(String, "/data_collect/control_ack", self._on_collect_ack, 10)
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_state)
        self._publish_state()

    def _event(self, row, phase, message, **extra):
        event = dict(request_id=row.get("request_id"), source=row.get("source"),
                     command=row.get("command"), phase=phase, message=message,
                     sent_at=time.time(), **extra)
        self._last_event = event
        self._event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
        self.get_logger().info(f"operator {event['source']} {event['command']} {phase}: {message}")
        return event

    def _publish_state(self):
        self._state_pub.publish(String(data=json.dumps({
            "pending": self._pending["row"] if self._pending else None,
            "last_event": self._last_event, "sent_at": time.time(),
        }, ensure_ascii=False)))

    def _control_live(self):
        return (time.monotonic() - self._control_at <= 2.5
                and self.count_publishers("/morla/control/state") > 0)

    def _collect_live(self):
        return (time.monotonic() - self._collect_at <= 2.5
                and self.count_publishers("/data_collect/state") > 0)

    def _on_control(self, msg):
        try:
            row = json.loads(msg.data)
            if isinstance(row, dict):
                self._control = row
                self._control_at = time.monotonic()
                self._confirm_control()
        except (ValueError, TypeError):
            pass

    def _on_collect(self, msg):
        try:
            row = json.loads(msg.data)
            if isinstance(row, dict):
                self._collect = row
                self._collect_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _on_command(self, msg):
        try:
            row = validate_request(msg.data)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.get_logger().warning(f"invalid operator request: {exc}")
            return
        now = time.monotonic()
        if not self._seen.accept(row, now):
            return
        command = row["command"]
        if command == "cancel":
            self._discard = None
            self._event(row, "confirmed", "已取消待确认的丢弃操作；已执行的操作不会撤销")
            return
        if self._pending:
            if command == "teleop_pause" and self._pending["kind"] == "collect":
                self._saving, self._pending = self._pending, None
            elif command == "teleop_pause" and self._pending.get("operation") == "start":
                self._event(self._pending["row"], "cancelled", "已被暂停请求取消")
                self._pending = None
            else:
                self._event(row, "rejected", "上一操作尚未完成，请等待结果")
                return
        if not self._control_live():
            self._event(row, "rejected", "机器人控制服务未上线，请先启动 Profile")
            return
        if command == "teleop_toggle":
            command = "teleop_pause" if self._control.get("teleop_armed") else "teleop_start"
        if command in {"teleop_start", "teleop_pause", "release"}:
            if command == "teleop_start" and self._control.get("teleop_armed") and row["source"] != "quest_controller":
                self._event(row, "confirmed", "已在跟随，不重复重锚定")
                return
            self._send_control(row, {"teleop_start": "start", "teleop_pause": "pause", "release": "release"}[command])
            return
        if not self._collect_live():
            self._event(row, "rejected", "采集节点未运行或状态过期")
            return
        if self._saving:
            self._event(row, "rejected", "上一采集请求尚未完成，请等待结果")
            return
        if (self._collect.get("profile_id") != self._control.get("profile_id")
                or self._collect.get("runtime_mode") != self._control.get("operating_mode")):
            self._event(row, "rejected", "采集与机器人 Profile 或运行模式不一致")
            return
        state = self._collect.get("state")
        if command == "record_toggle":
            command = "record_start" if state == "IDLE" else "record_stop"
        if command == "confirm_discard":
            pending = self._discard
            if (not pending or now > pending["deadline"] or pending["episode"] != self._collect.get("episode_index")
                    or pending["profile"] != self._control.get("profile_id")):
                self._discard = None
                self._event(row, "rejected", "没有待确认的当前段丢弃操作")
                return
            command = "record_discard"
        valid = {"record_start": {"IDLE"}, "record_stop": {"RECORDING", "PAUSED"},
                 "record_discard": {"RECORDING", "PAUSED"}, "record_pause": {"RECORDING"},
                 "record_resume": {"PAUSED"}}
        if state not in valid.get(command, set()):
            self._event(row, "rejected", f"当前采集状态 {state} 不允许此操作")
            return
        if command in {"record_start", "record_resume"} and not self._control.get("teleop_armed"):
            self._event(row, "rejected", "请先开始跟随，等待遥操确认后再采集")
            return
        if command == "record_discard" and row["command"] != "confirm_discard" and not (
            row["source"] == "quest_controller" or row["source"] == "pedal" and row.get("long_press") is True
        ):
            self._discard = dict(deadline=now + 5.0, episode=self._collect.get("episode_index"), profile=self._control.get("profile_id"))
            self._event(row, "confirmation_required", "请在五秒内说唤醒词加确认丢弃，或取消操作")
            return
        self._discard = None
        short = command.removeprefix("record_")
        self._pending = dict(row=row, kind="collect", deadline=now + (120.0 if short in {"stop", "discard"} else 10.0))
        payload = dict(command=short, request_id=row["request_id"], source=row["source"],
                       sent_at=time.time(),
                       expected_state=state, expected_episode_index=self._collect.get("episode_index"),
                       profile_id=self._control["profile_id"])
        self._collect_pub.publish(String(data=json.dumps(payload)))
        self._event(row, "submitted", "采集请求已提交，等待执行结果")

    def _send_control(self, row, operation):
        client = self._service_clients.get(operation)
        if client is None:
            client = self.create_client(Trigger, f"/morla/control/{operation}")
            self._service_clients[operation] = client
        if not client.service_is_ready():
            self._event(row, "rejected", "机器人控制服务不可用")
            return
        pending = dict(row=row, kind="control", operation=operation, deadline=time.monotonic() + 10.0)
        self._pending = pending
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._on_control_response(pending, done))
        self._event(row, "submitted", "遥操请求已提交，等待状态确认")

    def _on_control_response(self, pending, future):
        if self._pending is not pending:
            return
        try:
            response = future.result()
            match = re.search(r"operation_id=([a-zA-Z0-9]+)", response.message)
            if not response.success or not match:
                raise ValueError(response.message)
            pending["operation_id"] = match.group(1)
            self._confirm_control()
        except Exception as exc:
            self._event(pending["row"], "failed", f"遥操请求失败：{exc}")
            self._pending = None

    def _confirm_control(self):
        pending = self._pending
        if not pending or pending["kind"] != "control" or not pending.get("operation_id"):
            return
        state = self._control
        if state.get("operation_id") != pending["operation_id"]:
            return
        phase = state.get("phase")
        if phase in {"completed", "failed", "cancelled"}:
            labels = {"start": "遥操已开始", "pause": "跟随已暂停", "release": "控制权已交还"}
            message = labels[pending["operation"]] if phase == "completed" else state.get("message", "操作失败")
            self._event(pending["row"], "confirmed" if phase == "completed" else "failed", message)
            self._pending = None

    def _on_collect_ack(self, msg):
        try:
            ack = json.loads(msg.data)
            pending = next((item for item in (self._pending, self._saving)
                            if item and item["kind"] == "collect" and item["row"]["request_id"] == ack.get("request_id")), None)
            if not pending:
                return
            self._event(pending["row"], "confirmed" if ack.get("success") is True else "failed", ack.get("message", "采集操作完成"))
            if self._pending is pending:
                self._pending = None
            if self._saving is pending:
                self._saving = None
        except (ValueError, TypeError):
            pass

    def _tick(self):
        if self._pending and time.monotonic() > self._pending["deadline"]:
            self._event(self._pending["row"], "failed", "操作结果确认超时，请检查实际状态；没有自动重试")
            self._pending = None
        if self._saving and time.monotonic() > self._saving["deadline"]:
            self._event(self._saving["row"], "failed", "采集结果确认超时，请检查实际状态")
            self._saving = None


def main(args=None):
    rclpy.init(args=args)
    node = OperatorRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
