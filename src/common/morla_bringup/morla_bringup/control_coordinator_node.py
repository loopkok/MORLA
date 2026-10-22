"""Profile-driven asynchronous control API shared by web and robot tools."""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from morla_core.profile import ProfileRegistry
from morla_bringup.coordinator_logic import (
    MOTION_OPERATIONS, OPERATIONS, OperationRecord, nero_home_blockers, nero_start_blockers, operation_guard, preparation_steps,
    writer_statuses_match, writers_have_manual_ownership,
)


_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)

NERO_HOME_SERVICES = ("home", "home_point3", "home_point2", "home_point1", "home_final_pose")
NERO_HOME_LABELS = ("点位 4（工作位）", "返回点位 3", "返回点位 2", "返回点位 1", "最终 HOME：J2 到 85°，其余关节到 0°")
NERO_WORKPOS_SERVICES = ("position", "position_point2", "position_point3", "position_point4")
NERO_WORKPOS_LABELS = ("移至点位 1", "移至点位 2", "移至点位 3", "移至点位 4（工作位）")
NERO_POSITION_OPERATIONS = {"home", "position", "workpos", "workpos_direct"}


def nero_position_plan(operation):
    if operation == "home":
        return "home", "home_stage", NERO_HOME_SERVICES, NERO_HOME_LABELS
    return "workpos", "workpos_stage", NERO_WORKPOS_SERVICES, NERO_WORKPOS_LABELS



class ControlCoordinator(Node):
    """Accept control requests, route them through existing robot interfaces.

    Trigger.success means the request was accepted for asynchronous execution.
    Final completion or failure is published on ``/morla/control/state`` with
    the same operation_id. No hardware success is inferred from topic publish.
    """

    def __init__(self):
        super().__init__("morla_control_coordinator")
        self.declare_parameter("profile_id", "")
        self.declare_parameter("operating_mode", "real")
        self.declare_parameter("freeze_timeout_s", 2.0)
        self.declare_parameter("operation_timeout_s", 45.0)
        self.declare_parameter("policy_state_topic", "/policy_inference/state")
        self.declare_parameter("policy_cmd_topic", "/policy_inference/cmd")
        self.declare_parameter("collector_state_topic", "/data_collect/state")
        self.declare_parameter("state_topic", "/morla/control/state")
        profile_id = str(self.get_parameter("profile_id").value).strip()
        if not profile_id:
            raise ValueError("profile_id is required for the MORLA control coordinator")
        self.profile = ProfileRegistry.from_index().load(profile_id)
        self.profile_id = profile_id
        self.operating_mode = str(self.get_parameter("operating_mode").value).strip().lower()
        if self.operating_mode not in {"real", "sim"}:
            raise ValueError("operating_mode must be 'real' or 'sim'")
        self.capabilities = dict(self.profile.values.get("capabilities", {}))
        self.capabilities["initialize_can"] = self.profile.robot_id == "nero" and self.operating_mode == "real"
        self.capabilities["reset"] = self.profile.robot_id == "nero" and self.operating_mode == "real"
        if self.operating_mode == "sim":
            for capability in ("ready", "enable", "reset", "position", "damping", "estop"):
                self.capabilities[capability] = False
            if self.profile.robot_id == "nero":
                # The Nero simulator accepts standard joint targets but has no
                # SDK position service for the profile's configured work pose.
                self.capabilities["workpos"] = False
                self.capabilities["home"] = False
        self.capabilities["workpos_requires_reenable"] = self.profile.robot_id == "nero" and self.operating_mode == "real"
        self.capabilities["prepare"] = self.operating_mode == "real" and bool(self.capabilities.get("enable"))
        self.arms = [
            component for component in self.profile.values["components"]
            if component.get("enabled") and component.get("type") in {"arm", "nero_arm"}
        ]
        if not self.arms:
            raise ValueError(f"profile {profile_id!r} has no enabled arm components")

        self._collector_state: str | None = None
        self._collector_received = 0.0
        self._collector_state_topic = str(self.get_parameter("collector_state_topic").value)
        self._started_monotonic = time.monotonic()
        self._policy: dict | None = None
        self._policy_received = 0.0
        self._driver_status: dict[str, dict] = {}
        self._driver_received: dict[str, float] = {}
        self._teleop_status: dict[str, dict] = {}
        self._teleop_received: dict[str, float] = {}
        self.hands = [
            component for component in self.profile.values["components"]
            if component.get("enabled") and component.get("type") == "xhand"
        ]
        self._hand_status: dict[str, dict] = {}
        self._hand_received: dict[str, float] = {}
        self._writer_ids = [
            str(row["writer_id"]) for row in self.profile.values.get("writers", [])
            if row.get("writer_id")
        ]
        self._arm_writer_ids: dict[str, str] = {}
        self._writer_status: dict[str, dict] = {}
        self._writer_received: dict[str, float] = {}
        self._active: OperationRecord | None = None
        self._execution: dict | None = None
        # Node._clients is rclpy's own list; create_client() appends to it.
        self._robot_service_clients: dict[tuple[str, str], object] = {}
        self._state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), _STATE_QOS
        )
        self._policy_cmd = self.create_publisher(
            String, str(self.get_parameter("policy_cmd_topic").value), 10
        )
        self._teleop_start = self.create_publisher(Bool, "/teleop/start", _STATE_QOS)
        self._teleop_armed = self.create_publisher(Bool, "/teleop/armed", _STATE_QOS)
        self._teleop_disarm = self.create_publisher(Bool, "/teleop/disarm", _STATE_QOS)
        self._teleop_home = self.create_publisher(Bool, "/teleop/home", 10)
        self._teleop_init = self.create_publisher(Bool, "/teleop/init", 10)
        self._teleop_init_direct = self.create_publisher(Bool, "/teleop/init_direct", 10)
        self.create_subscription(
            String, self._collector_state_topic,
            self._on_collector_state, _STATE_QOS,
        )
        self.create_subscription(
            String, str(self.get_parameter("policy_state_topic").value),
            self._on_policy_state, _STATE_QOS,
        )
        self._setup_component_status()
        self._control_services = {
            op: self.create_service(Trigger, f"/morla/control/{op}",
                                    lambda req, res, operation=op: self._on_request(operation, req, res))
            for op in sorted(OPERATIONS)
        }
        self._timer = self.create_timer(0.05, self._tick)
        # Keep the latched state fresh while idle as well as during an
        # operation. Consumers use its receive age to distinguish a live
        # coordinator from a stale transient-local sample after process exit.
        self._state_timer = self.create_timer(1.0, self._publish_state)
        self._publish_state()
        self.get_logger().info(
            f"MORLA control coordinator profile={profile_id} arms="
            f"{[item['side'] for item in self.arms]} services={sorted(OPERATIONS)}"
        )

    def _setup_component_status(self):
        writers = self.profile.values.get("writers", [])
        component_writer = {
            str(row.get("component_id")): str(row.get("writer_id"))
            for row in writers if row.get("component_id") and row.get("writer_id")
        }
        for component in self.arms:
            side = component["side"]
            writer_id = component_writer.get(component.get("component_id"), f"{side}_arm")
            self._arm_writer_ids[side] = writer_id
            if component.get("type") == "nero_arm":
                driver_ns = f"/{side}_arm/nero_robot_driver"
                teleop_status = f"/{side}_arm/nero_arm_teleop/status"
            else:
                driver_ns = "/astral_robot_driver"
                teleop_status = f"/teleop/arm_status/{side}"
            self.create_subscription(
                String, f"{driver_ns}/status",
                lambda msg, s=side: self._on_driver_status(s, msg), 10,
            )
            self.create_subscription(
                String, teleop_status,
                lambda msg, s=side, w=writer_id: self._on_arm_teleop_status(s, w, msg), 10,
            )
        arm_writer_ids = set(self._arm_writer_ids.values())
        for writer_id in self._writer_ids:
            if writer_id in arm_writer_ids:
                continue
            self.create_subscription(
                String, f"/teleop/writer_status/{writer_id}",
                lambda msg, w=writer_id: self._on_writer_status(w, msg), 10,
            )
        for hand in self.hands:
            side = hand["side"]
            self.create_subscription(
                String, f"/{side}_hand/adapter/status",
                lambda msg, s=side: self._on_hand_status(s, msg), 10,
            )

    def _on_hand_status(self, side: str, msg: String):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict) and value.get("side") == side:
                self._hand_status[side] = value
                self._hand_received[side] = time.monotonic()
        except (TypeError, ValueError):
            pass

    def _on_arm_teleop_status(self, side: str, writer_id: str, msg: String):
        self._on_status(self._teleop_status, side, msg)
        if side in self._teleop_status:
            self._writer_status[writer_id] = self._teleop_status[side]
            self._writer_received[writer_id] = self._teleop_received[side]

    def _on_writer_status(self, writer_id: str, msg: String):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict) and value.get("writer_id") == writer_id:
                self._writer_status[writer_id] = value
                self._writer_received[writer_id] = time.monotonic()
        except (TypeError, ValueError):
            pass

    def _on_status(self, target: dict, side: str, msg: String):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                target[side] = value
                if target is self._teleop_status:
                    self._teleop_received[side] = time.monotonic()
                if target is self._teleop_status and self._active is not None:
                    if value.get("homing"):
                        self._active.components.setdefault(side, {})["homing_seen"] = True
        except (TypeError, ValueError):
            pass

    def _on_driver_status(self, side: str, msg: String):
        self._on_status(self._driver_status, side, msg)
        if side in self._driver_status:
            self._driver_received[side] = time.monotonic()

    def _fresh_driver_status(self, side: str) -> dict:
        if time.monotonic() - self._driver_received.get(side, 0.0) > 1.0:
            return {}
        return self._driver_status.get(side, {})

    def _on_collector_state(self, msg: String):
        try:
            value = json.loads(msg.data)
            state = value.get("state") if isinstance(value, dict) else None
            self._collector_state = str(state) if state is not None else None
            self._collector_received = time.monotonic()
        except (TypeError, ValueError):
            self._collector_state = None
            self._collector_received = 0.0

    def _on_policy_state(self, msg: String):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self._policy = value
                self._policy_received = time.monotonic()
        except (TypeError, ValueError):
            self._policy = None

    def _effective_collector_state(self) -> str | None:
        age = time.monotonic() - self._collector_received
        if self._collector_state is not None and age <= 2.5:
            return self._collector_state
        # No collector is a legitimate teleop-only configuration. Permit it
        # only after graph discovery has settled and no publisher has ever
        # reported an active recording state; stale RECORDING/PAUSED/SAVING
        # remains fail-closed after a collector crash.
        if time.monotonic() - self._started_monotonic < 2.0:
            return None
        if self.count_publishers(self._collector_state_topic) != 0:
            return None
        if self._collector_state in {"RECORDING", "PAUSED", "SAVING"}:
            return self._collector_state
        return "NO_COLLECTOR"

    def _on_request(self, operation: str, _request, response):
        collector_state = self._effective_collector_state()
        allowed, reason = operation_guard(
            operation, self.capabilities, collector_state
        )
        if not allowed:
            response.success = False
            response.message = reason
            return response
        if self._active is not None and self._active.phase not in {"completed", "failed", "cancelled"}:
            pause_start = operation == "pause" and self._active.operation == "start"
            if operation != "estop" and not pause_start:
                response.success = False
                response.message = f"operation {self._active.operation_id} is still {self._active.phase}"
                return response
            self._active.update("cancelled", "preempted by pause" if pause_start else "preempted by emergency stop", time.time(), result=False)
            self._publish_state()
            self._execution = None

        record = OperationRecord.create(operation, time.time())
        if operation in {"prepare", "initialize_can", "reset", "position", "workpos", "workpos_direct", "home", "estop", "enable", "damping"}:
            record.components = {
                row["side"]: {"result": "pending"} for row in self.arms
            }
        self._active = record
        self._execution = {
            "operation": operation,
            "phase": "accepted",
            "deadline": time.monotonic() + float(self.get_parameter("operation_timeout_s").value),
            "futures": [],
            "responses_seen": False,
            "status_baseline": {},
            "freeze_sent": False,
            "service_started": False,
            "homing_seen": set(),
            "position_seen": set(),
        }
        self._publish_state()
        response.success = True
        response.message = f"accepted operation_id={record.operation_id}; poll /morla/control/state"
        return response

    def _publish_state(self):
        if self._active is None:
            payload = {"profile_id": self.profile_id, "phase": "idle", "operation_id": None}
        else:
            payload = self._active.to_dict(self.profile_id)
        payload["operating_mode"] = self.operating_mode
        payload["capabilities"] = dict(self.capabilities)
        payload["teleop_armed"] = bool(self._writers_armed())
        if self.profile.robot_id == "nero":
            blockers = self._start_blockers()
            payload["start_readiness"] = {"ready": not blockers, "blockers": blockers}
        payload["driver_status"] = {
            row["side"]: {
                key: status.get(key)
                for key in (
                    "connected", "feedback_fresh", "enabled", "enable_status",
                    "streaming_enabled", "motors_enabled", "motor_status_fresh",
                    "position_permitted",
                    "motor_status_error", "joint_enable_status",
                    "controller_status_fresh", "controller_arm_status",
                    "controller_motion_status", "reset_seq", "reset_status",
                    "reset_error", "reset_mode", "reset_unknown_status_supported", "position_active", "command_active", "estop_status", "estop_seq", "last_error",
                    "position_kind", "position_stage", "position_stage_count",
                    "feedback_read_ok", "feedback_read_error", "last_feedback_limit_violation",
                    "position_status", "position_error_rad", "home_speed_percent", "trajectory_revision", "home_final_pose_deg", "workpos_waypoints_deg", "position_speed_percent",
                )
            }
            for row in self.arms
            if (status := self._fresh_driver_status(row["side"]))
        }
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _set_phase(self, phase: str, message: str, *, result=None):
        if self._active is None:
            return
        self._active.update(phase, message, time.time(), result=result)
        self._publish_state()

    def _policy_fresh(self) -> bool:
        return self._policy is not None and time.monotonic() - self._policy_received <= 2.0

    def _policy_activity(self):
        return self._policy.get("activity") if self._policy_fresh() else None

    def _writers_frozen(self):
        return writer_statuses_match(
            self._writer_ids, self._writer_status, self._writer_received,
            "armed", False, now=time.monotonic(),
        )

    def _writers_armed(self):
        return writer_statuses_match(
            self._writer_ids, self._writer_status, self._writer_received,
            "armed", True, now=time.monotonic(),
        )

    def _start_blockers(self):
        """Only Nero has this measured streaming contract; preserve other stacks."""
        if self.profile.robot_id != "nero":
            return []
        reasons = []
        now = time.monotonic()
        for arm in self.arms:
            side = arm["side"]
            label = "左臂" if side == "left" else "右臂"
            if self.operating_mode == "real":
                reasons.extend(f"{label}：{item}" for item in nero_start_blockers(self._fresh_driver_status(side)))
            teleop = self._teleop_status.get(side, {})
            if now - self._teleop_received.get(side, 0.0) > 1.0:
                reasons.append(f"{label}：遥操状态缺失或过期")
            elif not (teleop.get("wrist_fresh") and teleop.get("feedback_fresh")):
                reasons.append(f"{label}：需要新鲜的 Quest 腕位姿和实测关节反馈")
        for hand in self.hands:
            side = hand["side"]
            label = "左手" if side == "left" else "右手"
            status = self._hand_status.get(side, {})
            if now - self._hand_received.get(side, 0.0) > 1.0 or status.get("feedback_fresh") is not True:
                reasons.append(f"{label}：XHand 实测反馈缺失或过期，请检查串口与 hand_id")
            writer = next((str(row["writer_id"]) for row in self.profile.values.get("writers", [])
                           if row.get("component_id") == hand.get("component_id")), f"{side}_ee")
            if (now - self._writer_received.get(writer, 0.0) > 1.0
                    or self._writer_status.get(writer, {}).get("input_fresh") is not True):
                reasons.append(f"{label}：Quest 手部关键点缺失或过期，请开启手部追踪")
        return reasons

    def _all_writers_manual_or_idle(self):
        return writers_have_manual_ownership(
            self._writer_ids, self._writer_status, self._writer_received,
            now=time.monotonic(),
        )

    def _dispatch_service(self, side: str, operation: str):
        component = next(row for row in self.arms if row["side"] == side)
        kind = component.get("type")
        if kind == "nero_arm":
            service = {"initialize_can": "initialize_can", "enable": "enable", "position": "position", "workpos": "position",
                       "workpos_direct": "position", "estop": "estop", "reset": "reset",
                       **{stage: stage for stage in (*NERO_HOME_SERVICES, *NERO_WORKPOS_SERVICES)}}.get(operation)
            name = f"/{side}_arm/nero_robot_driver/{service}" if service else None
        else:
            service = {"ready": "ready", "enable": "enable", "position": "position",
                       "damping": "damping", "estop": "estop", "home": "home",
                       "workpos": "init", "workpos_direct": "init_direct"}.get(operation)
            node = "/astral_robot_driver" if operation in {"ready", "enable", "position", "damping", "estop"} else f"/astral_arm_teleop_{side}"
            name = f"{node}/{service}" if service else None
        if not name:
            return None
        cache_key = (side, name)
        if cache_key not in self._robot_service_clients:
            self._robot_service_clients[cache_key] = self.create_client(Trigger, name)
        return self._robot_service_clients[cache_key]

    def _operation_clients(self, operation: str):
        """Return the configured service targets, de-duplicating shared Astral driver."""
        if self.profile.robot_id == "astral" and operation in {
            "ready", "enable", "position", "damping", "estop"
        }:
            client = self._dispatch_service(self.arms[0]["side"], operation)
            return [("robot", client)] if client is not None else []
        return [
            (row["side"], self._dispatch_service(row["side"], operation))
            for row in self.arms
        ]

    def _fail(self, message: str):
        if self._active is not None and self._active.operation == "start":
            self._teleop_armed.publish(Bool(data=False))
            self._teleop_disarm.publish(Bool(data=True))
            if self._execution and self._execution.get("start_dispatched_at") and self._policy_fresh():
                self._policy_cmd.publish(String(data="stop"))
        fail_stop_operations = {
            "initialize_can", "prepare", "position", "workpos", "workpos_direct", "home",
        }
        if (self._active is not None and self._active.operation in fail_stop_operations
                and self._execution is not None
                and (self._execution.get("futures") or self._execution.get("service_started"))):
            # A partially failed physical operation must not leave its peer
            # moving or make an accepted service response look like arrival.
            # Request both stops; never present the request as confirmation.
            stop_messages = {}
            for side, client in self._operation_clients("estop"):
                if client is not None and client.service_is_ready():
                    client.call_async(Trigger.Request())
                    stop_message = "operation aborted; electronic stop requested, verify driver status"
                else:
                    stop_message = "operation aborted; stop service unavailable, physical stop unconfirmed"
                stop_messages[side] = stop_message
            for side, component in self._active.components.items():
                if component.get("result") in {"pending", "accepted"}:
                    component.update(result="cancelled", message=stop_messages.get(side, stop_messages.get("robot", "stop unconfirmed")))
            message += "; electronic stop requested for all configured arms where available; verify driver status"
        self._set_phase("failed", message, result=False)
        self._execution = None

    def _complete(self, message: str):
        execution = self._execution
        if (execution is not None and execution["operation"] in NERO_POSITION_OPERATIONS
                and self.profile.robot_id == "nero"):
            kind, key, services, labels = nero_position_plan(execution["operation"])
            stage = execution[key]
            name = "HOME" if kind == "home" else "工作位"
            if stage < len(services):
                execution.update({key: stage + 1, "futures": [], "responses_seen": False,
                                  "status_baseline": {}})
                execution["deadline"] = time.monotonic() + float(self.get_parameter("operation_timeout_s").value)
                for component in self._active.components.values():
                    component.update(result="pending", message=f"{name} {stage}/{len(services)} 双臂到位，准备下一段")
                self._dispatch(execution["operation"])
                return
            message = f"{name} {stage}/{len(services)}：所有配置手臂实测到位，保持 J 模式；遥操前请再次准备并使能"
            for component in self._active.components.values():
                component.update(result="completed", message=f"{name} 实测到位完成")
        if execution is not None and len(execution.get("prepare_steps", [])) > 1:
            execution["prepare_steps"].pop(0)
            next_op = execution["prepare_steps"][0]
            execution.update(operation=next_op, futures=[], responses_seen=False, status_baseline={})
            execution["deadline"] = time.monotonic() + float(self.get_parameter("operation_timeout_s").value)
            for component in self._active.components.values():
                component.update(result="pending", message="feedback channel ready; enabling robot control")
            self._dispatch(next_op)
            return
        self._set_phase("completed", message, result=True)
        self._execution = None

    def _tick(self):
        execution = self._execution
        if execution is None or self._active is None:
            return
        op = execution["operation"]
        now = time.monotonic()
        if now > execution["deadline"]:
            self._fail(f"{op} did not reach confirmed completion before timeout")
            return
        if execution["phase"] == "accepted":
            if op == "estop":
                if self._policy_fresh() and self._policy_activity() is not None:
                    self._policy_cmd.publish(String(data="stop"))
                self._teleop_disarm.publish(Bool(data=True))
                self._dispatch(op)
                return
            if op in {"pause", "resume", "start", "release"}:
                self._dispatch_mode(op)
                return
            if op in MOTION_OPERATIONS:
                execution["phase"] = "freezing"
                execution["freeze_deadline"] = now + float(self.get_parameter("freeze_timeout_s").value)
                if self._policy_fresh() and (self._policy_activity() is not None
                        or self._policy.get("takeover_pending") or self._policy.get("policy_start_pending")):
                    self._policy_cmd.publish(String(data="stop"))
                self._teleop_disarm.publish(Bool(data=True))
                self._set_phase("freezing", "stopping policy/manual writers before robot operation")
                return
        if execution["phase"] == "freezing":
            if self._policy_fresh():
                policy_safe = (self._policy_activity() is None
                               and not self._policy.get("takeover_pending")
                               and not self._policy.get("policy_start_pending"))
            else:
                policy_safe = self._all_writers_manual_or_idle()
            writers_safe = self._writers_frozen()
            if policy_safe and writers_safe:
                self._dispatch(op)
            elif now > execution["freeze_deadline"]:
                self._fail("writer freeze was not confirmed within two seconds")
            return
        if execution["phase"] == "waiting_services_ready":
            clients = execution["clients"]
            if all(client is not None and client.service_is_ready() for _, client in clients):
                self._call_services(clients)
            return
        if execution["phase"] == "waiting_services":
            futures = execution["futures"]
            if not futures or not all(future.done() for _, future in futures):
                return
            failures = []
            for side, future in futures:
                try:
                    result = future.result()
                    success = bool(result.success)
                    message = str(result.message)
                except Exception as exc:  # service disconnected or callback failure
                    success, message = False, str(exc)
                self._active.components.setdefault(side, {}).update(
                    {"result": "accepted" if success else "failed", "message": message}
                )
                if not success:
                    failures.append(f"{side}: {message}")
            if failures:
                self._fail("; ".join(failures))
                return
            execution["responses_seen"] = True
            if self._service_response_is_final(op):
                for component in self._active.components.values():
                    component.update(result="completed")
                self._complete("configured robot controller completed the operation")
            elif op in {"initialize_can", "enable", "reset", "position", "workpos", "workpos_direct", "home", "estop", "ready"}:
                execution["phase"] = "executing"
                execution["deadline"] = now + float(self.get_parameter("operation_timeout_s").value)
                message = "controller accepted request; waiting for measured/status confirmation"
                if op in NERO_POSITION_OPERATIONS and self.profile.robot_id == "nero":
                    kind, key, services, labels = nero_position_plan(op)
                    stage = execution[key]
                    message = f"{kind} {stage}/{len(services)}：{labels[stage - 1]}；等待所有配置手臂实测到位"
                self._set_phase("executing", message)
            else:
                self._complete("all configured controllers returned success")
            return
        if execution["phase"] == "executing":
            if op == "start":
                blockers = self._start_blockers()
                if blockers:
                    self._fail("开始遥操检查失败；已冻结输出：" + "; ".join(blockers))
                    return
            if op == "initialize_can" and self.profile.robot_id == "nero":
                failures = []
                for row in self.arms:
                    status = self._fresh_driver_status(row["side"])
                    if (int(status.get("can_init_seq", 0))
                            > int(execution["status_baseline"].get(row["side"], -1))
                            and status.get("can_init_status") in {"failed", "timed_out", "cancelled"}):
                        message = status.get("can_init_error") or status.get("last_error") or status["can_init_status"]
                        self._active.components.setdefault(row["side"], {}).update(result="failed", message=message)
                        failures.append(f"{row['side']}: {message}")
                if failures:
                    self._fail("; ".join(failures))
                    return
            if self.profile.robot_id == "nero" and op in {"position", "workpos", "workpos_direct", "home"}:
                failures = []
                for row in self.arms:
                    status = self._fresh_driver_status(row["side"])
                    if (int(status.get("position_seq", 0))
                            > int(execution["status_baseline"].get(row["side"], -1))
                            and status.get("position_status") in {"failed", "timed_out", "cancelled"}):
                        message = status.get("last_error") or status["position_status"]
                        self._active.components.setdefault(row["side"], {}).update(
                            result="failed", message=message)
                        failures.append(f"{row['side']}: {message}")
                if failures:
                    self._fail("; ".join(failures))
                    return
            if self.profile.robot_id == "nero" and op == "reset":
                failures = []
                for row in self.arms:
                    status = self._fresh_driver_status(row["side"])
                    if (int(status.get("reset_seq", 0))
                            > int(execution["status_baseline"].get(row["side"], -1))
                            and status.get("reset_status") in {"failed", "timed_out"}):
                        message = status.get("reset_error") or status.get("last_error") or status["reset_status"]
                        self._active.components.setdefault(row["side"], {}).update(result="failed", message=message)
                        failures.append(f"{row['side']}: {message}")
                if failures:
                    self._fail("; ".join(failures))
                    return
            if self._operation_confirmed(op, execution):
                self._complete(f"{op} confirmed by controller status/feedback")

    def _dispatch_mode(self, op: str):
        execution = self._execution
        assert execution is not None
        # The accepted phase is only a dispatch gate. Move the internal state
        # before publishing requests so the 20 Hz timer cannot resend them.
        execution["phase"] = "executing"
        policy_state = self._policy if self._policy_fresh() else None
        activity = policy_state.get("activity") if policy_state else None
        if op == "start":
            blockers = self._start_blockers()
            if blockers:
                self._fail("尚不能开始遥操：" + "; ".join(blockers))
                return
            execution["start_dispatched_at"] = time.monotonic()
            if activity in {"policy", "playback"}:
                self._policy_cmd.publish(String(data="takeover"))
                self._set_phase("executing", "requested policy-to-human re-anchor; waiting for ownership commit")
                return
            if activity == "human":
                if self._writers_armed():
                    self._complete("human writers are already armed")
                else:
                    self._teleop_armed.publish(Bool(data=True))
                    self._teleop_start.publish(Bool(data=True))
                    self._set_phase("executing", "manual start signal published; waiting for enabled writers")
                    execution["deadline"] = time.monotonic() + 3.0
                return
            self._teleop_armed.publish(Bool(data=True))
            self._teleop_start.publish(Bool(data=True))
            self._set_phase("executing", "start signal published; waiting for enabled teleop writers")
            execution["deadline"] = time.monotonic() + 3.0
            return
        if op == "pause":
            if activity in {"policy", "playback"}:
                self._policy_cmd.publish(String(data="pause"))
                self._set_phase("executing", "pause requested; waiting for policy state")
            else:
                self._teleop_disarm.publish(Bool(data=True))
                self._set_phase("executing", "manual writers disarmed; waiting for status")
            execution["deadline"] = time.monotonic() + 3.0
            return
        if op == "release":
            if activity != "human":
                self._fail("release requires HUMAN ownership")
                return
            self._policy_cmd.publish(String(data="release"))
            self._set_phase("executing", "release requested; waiting for POLICY ownership commit")
            execution["deadline"] = time.monotonic() + 3.0
            return
        # resume
        if activity in {"policy", "playback"} and policy_state.get("paused"):
            self._policy_cmd.publish(String(data="resume"))
            self._set_phase("executing", "resume requested; waiting for policy state")
            execution["deadline"] = time.monotonic() + 3.0
        elif activity in (None, "idle", "human"):
            self._teleop_armed.publish(Bool(data=True))
            self._teleop_start.publish(Bool(data=True))
            self._set_phase("executing", "manual resume signal published; waiting for enabled writers")
            execution["deadline"] = time.monotonic() + 3.0
        else:
            self._fail("resume requires paused policy/playback or an idle/manual lease")

    def _dispatch(self, op: str):
        execution = self._execution
        assert execution is not None
        if op == "prepare":
            feedback_ready = all(
                self._fresh_driver_status(row["side"]).get("connected")
                and self._fresh_driver_status(row["side"]).get("feedback_fresh")
                for row in self.arms
            )
            steps = preparation_steps(self.capabilities, feedback_ready=feedback_ready)
            execution["prepare_initialize_sides"] = [
                row["side"] for row in self.arms
                if not (self._fresh_driver_status(row["side"]).get("connected")
                        and self._fresh_driver_status(row["side"]).get("feedback_fresh"))
            ]
            execution.update(prepare_steps=steps, operation=steps[0])
            self._dispatch(steps[0])
            return
        if op == "ready" and self.profile.robot_id == "nero":
            if all(self._fresh_driver_status(row["side"]).get("connected")
                   and self._fresh_driver_status(row["side"]).get("feedback_fresh") for row in self.arms):
                self._complete("all enabled arm drivers have fresh feedback")
            else:
                self._fail("one or more Nero arm drivers are disconnected or have stale feedback")
            return
        if op == "initialize_can" and "prepare_initialize_sides" in execution:
            clients = [(side, self._dispatch_service(side, op)) for side in execution["prepare_initialize_sides"]]
        elif op in NERO_POSITION_OPERATIONS and self.profile.robot_id == "nero":
            kind, key, services, labels = nero_position_plan(op)
            stage = execution.setdefault(key, 1)
            blockers = {
                row["side"]: nero_home_blockers(
                    self._fresh_driver_status(row["side"])
                ) for row in self.arms
            }
            if any(blockers.values()):
                messages = []
                for side, reasons in blockers.items():
                    label = "左臂" if side == "left" else "右臂"
                    message = "；".join(reasons) if reasons else f"另一侧未就绪，本侧不执行 {kind}"
                    self._active.components.setdefault(side, {}).update(
                        result="failed" if reasons else "cancelled", message=message
                    )
                    if reasons:
                        messages.append(f"{label}：{message}")
                self._fail(f"{'HOME' if kind == 'home' else '工作位'} 未开始：" + "；".join(messages))
                return
            # Keep a bilateral barrier: no arm receives the next stage until
            # every configured arm has confirmed the previous stage in feedback.
            clients = self._operation_clients(services[stage - 1])
        else:
            clients = self._operation_clients(op)
        if not clients or any(client is None for _, client in clients):
            self._fail(f"no configured {op} service target for profile {self.profile_id}")
            return
        for row in self.arms:
            side = row["side"]
            if op == "initialize_can" and self.profile.robot_id == "nero":
                execution["status_baseline"][side] = int(self._fresh_driver_status(side).get("can_init_seq", 0))
            elif op in {"position", "workpos", "workpos_direct", "home"} and self.profile.robot_id == "nero":
                execution["status_baseline"][side] = int(self._fresh_driver_status(side).get("position_seq", 0))
            elif op in {"workpos", "workpos_direct", "home"} and self.profile.robot_id == "astral":
                current = self._teleop_status.get(side, {})
                execution["status_baseline"][side] = {
                    "homing": bool(current.get("homing", False)),
                    "at_init_pose": bool(current.get("at_init_pose", False)),
                    "at_zero": bool(current.get("at_zero", False)),
                }
            elif op == "estop" and self.profile.robot_id == "nero":
                execution["status_baseline"][side] = int(self._fresh_driver_status(side).get("estop_seq", 0))
            elif op == "reset" and self.profile.robot_id == "nero":
                execution["status_baseline"][side] = int(self._fresh_driver_status(side).get("reset_seq", 0))
        if not all(client.service_is_ready() for _, client in clients):
            execution["clients"] = clients
            execution["phase"] = "waiting_services_ready"
            self._set_phase("executing", f"waiting for configured {op} service discovery")
            return
        self._call_services(clients)

    def _call_services(self, clients):
        execution = self._execution
        if execution is None:
            return
        execution["service_started"] = True
        futures = []
        for target, client in clients:
            futures.append((target, client.call_async(Trigger.Request())))
        execution["futures"] = futures
        execution["phase"] = "waiting_services"
        self._set_phase("executing", f"calling configured {execution['operation']} controller services")

    def _service_response_is_final(self, operation: str) -> bool:
        # Astral driver Trigger callbacks perform the hardware mode change before
        # returning. Teleop home/workpos callbacks only start an asynchronous
        # trajectory and must be confirmed from fresh measured status below.
        return self.profile.robot_id == "astral" and operation in {
            "ready", "enable", "position", "damping", "estop"
        }

    def _operation_confirmed(self, op: str, execution: dict) -> bool:
        if op == "initialize_can":
            return self.profile.robot_id == "nero" and all(
                int(self._fresh_driver_status(row["side"]).get("can_init_seq", 0))
                > int(execution["status_baseline"].get(row["side"], -1))
                and self._fresh_driver_status(row["side"]).get("can_init_status") == "ready"
                and self._fresh_driver_status(row["side"]).get("feedback_fresh")
                for row in self.arms
                if "prepare_initialize_sides" not in execution or row["side"] in execution["prepare_initialize_sides"]
            )
        if op == "start":
            if self._start_blockers():
                return False
            if self.profile.robot_id == "nero" and any(
                self._writer_received.get(writer, 0.0) < execution.get("start_dispatched_at", float("inf"))
                for writer in self._writer_ids
            ):
                return False
            if self._policy_fresh() and self._policy_activity() == "human":
                return (
                    not self._policy.get("takeover_pending")
                    and not self._policy.get("policy_start_pending")
                    and self._writers_armed()
                )
            return self._writers_armed()
        if op == "pause":
            if self._policy_activity() in {"policy", "playback"}:
                return bool(self._policy.get("paused")) and self._writers_frozen()
            return self._writers_frozen()
        if op == "resume":
            return self._policy_activity() in {"policy", "playback"} and not bool(self._policy.get("paused"))
        if op == "release":
            return (
                self._policy_fresh()
                and self._policy_activity() in {"policy", "playback"}
                and not self._policy.get("takeover_pending")
                and self._writers_frozen()
            )
        if op == "enable":
            if self.profile.robot_id == "astral":
                return True
            return all(
                self._fresh_driver_status(row["side"]).get("enabled")
                and self._fresh_driver_status(row["side"]).get("controller_status_fresh")
                and self._fresh_driver_status(row["side"]).get("controller_arm_status") == 0
                and self._fresh_driver_status(row["side"]).get("motor_status_fresh") is True
                and self._fresh_driver_status(row["side"]).get("motors_enabled") is True
                and self._fresh_driver_status(row["side"]).get("position_permitted") is True
                for row in self.arms
            )
        if op == "reset":
            return self.profile.robot_id == "nero" and all(
                int(self._fresh_driver_status(row["side"]).get("reset_seq", 0))
                > int(execution["status_baseline"].get(row["side"], -1))
                and self._fresh_driver_status(row["side"]).get("reset_status") == "cleared"
                and self._fresh_driver_status(row["side"]).get("controller_status_fresh")
                and self._fresh_driver_status(row["side"]).get("controller_arm_status") in {0, 6}
                and not self._fresh_driver_status(row["side"]).get("enabled")
                for row in self.arms
            )
        if op == "estop":
            if self.profile.robot_id == "nero":
                return all(
                    int(self._fresh_driver_status(row["side"]).get("estop_seq", 0))
                    > int(execution["status_baseline"].get(row["side"], -1))
                    and self._fresh_driver_status(row["side"]).get("estop_status") == "sent"
                    for row in self.arms
                )
            return True  # Astral Trigger is synchronous and reports the hardware result.
        if op in NERO_POSITION_OPERATIONS and self.profile.robot_id == "nero":
            kind, key, services, labels = nero_position_plan(op)
            stage = execution[key]
            return all(
                int(status.get("position_seq", 0)) > int(execution["status_baseline"].get(row["side"], -1))
                and status.get("position_kind") == kind
                and status.get("position_stage") == stage
                and status.get("position_stage_count") == len(services)
                and status.get("position_status") == ("arrived" if stage == len(services) else "stage_arrived")
                and status.get("position_active") is (stage < len(services))
                and status.get("feedback_fresh")
                and status.get("feedback_read_ok", True)
                and status.get("position_error_rad") is not None
                and status["position_error_rad"] <= 0.02
                and status.get("controller_status_fresh")
                and status.get("controller_arm_status") == 0
                and status.get("motor_status_fresh") is True
                and status.get("motors_enabled") is True
                and status.get("position_permitted") is True
                for row in self.arms
                for status in (self._fresh_driver_status(row["side"]),)
            )
        if op in {"workpos", "workpos_direct", "home"} and self.profile.robot_id == "astral":
            key = "at_zero" if op == "home" else "at_init_pose"
            return all(
                time.monotonic() - self._teleop_received.get(row["side"], 0.0) <= 1.0
                and self._teleop_status.get(row["side"], {}).get("feedback_fresh")
                and self._teleop_status.get(row["side"], {}).get(key)
                and not self._teleop_status.get(row["side"], {}).get("homing", True)
                for row in self.arms
            )
        if op == "ready":
            return all(self._fresh_driver_status(row["side"]).get("connected")
                       and self._fresh_driver_status(row["side"]).get("feedback_fresh") for row in self.arms)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ControlCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
