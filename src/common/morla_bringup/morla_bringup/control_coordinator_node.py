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
    MOTION_OPERATIONS, OPERATIONS, OperationRecord, operation_guard,
    writer_statuses_match, writers_have_manual_ownership,
)


_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)


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
        if self.operating_mode == "sim":
            for capability in ("ready", "enable", "position", "damping", "estop"):
                self.capabilities[capability] = False
            if self.profile.robot_id == "nero":
                # The Nero simulator accepts standard joint targets but has no
                # SDK position service for the profile's configured work pose.
                self.capabilities["workpos"] = False
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
        self._writer_ids = [
            str(row["writer_id"]) for row in self.profile.values.get("writers", [])
            if row.get("writer_id")
        ]
        self._arm_writer_ids: dict[str, str] = {}
        self._writer_status: dict[str, dict] = {}
        self._writer_received: dict[str, float] = {}
        self._active: OperationRecord | None = None
        self._execution: dict | None = None
        self._clients: dict[tuple[str, str], object] = {}
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
            if operation != "estop":
                response.success = False
                response.message = f"operation {self._active.operation_id} is still {self._active.phase}"
                return response
            self._active.update("cancelled", "preempted by emergency stop", time.time(), result=False)
            self._publish_state()
            self._execution = None

        record = OperationRecord.create(operation, time.time())
        if operation in {"position", "workpos", "workpos_direct", "home", "estop", "enable", "damping"}:
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

    def _all_writers_manual_or_idle(self):
        return writers_have_manual_ownership(
            self._writer_ids, self._writer_status, self._writer_received,
            now=time.monotonic(),
        )

    def _dispatch_service(self, side: str, operation: str):
        component = next(row for row in self.arms if row["side"] == side)
        kind = component.get("type")
        if kind == "nero_arm":
            service = {"enable": "enable", "position": "position", "workpos": "position",
                       "workpos_direct": "position", "estop": "estop"}.get(operation)
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
        if cache_key not in self._clients:
            self._clients[cache_key] = self.create_client(Trigger, name)
        return self._clients[cache_key]

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
        self._set_phase("failed", message, result=False)
        self._execution = None

    def _complete(self, message: str):
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
            elif op in {"enable", "position", "workpos", "workpos_direct", "home", "estop", "ready"}:
                execution["phase"] = "executing"
                execution["deadline"] = now + float(self.get_parameter("operation_timeout_s").value)
                self._set_phase("executing", "controller accepted request; waiting for measured/status confirmation")
            else:
                self._complete("all configured controllers returned success")
            return
        if execution["phase"] == "executing":
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
        if op == "ready" and self.profile.robot_id == "nero":
            if all(self._fresh_driver_status(row["side"]).get("connected")
                   and self._fresh_driver_status(row["side"]).get("feedback_fresh") for row in self.arms):
                self._complete("all enabled arm drivers have fresh feedback")
            else:
                self._fail("one or more Nero arm drivers are disconnected or have stale feedback")
            return
        clients = self._operation_clients(op)
        if not clients or any(client is None for _, client in clients):
            self._fail(f"no configured {op} service target for profile {self.profile_id}")
            return
        for row in self.arms:
            side = row["side"]
            if op == "position" and self.profile.robot_id == "nero":
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
        if op == "start":
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
            return all(self._fresh_driver_status(row["side"]).get("enabled") for row in self.arms)
        if op == "estop":
            if self.profile.robot_id == "nero":
                return all(
                    int(self._fresh_driver_status(row["side"]).get("estop_seq", 0))
                    > int(execution["status_baseline"].get(row["side"], -1))
                    and self._fresh_driver_status(row["side"]).get("estop_status") == "sent"
                    for row in self.arms
                )
            return True  # Astral Trigger is synchronous and reports the hardware result.
        if op in {"position", "workpos", "workpos_direct"} and self.profile.robot_id == "nero":
            return all(
                int(self._fresh_driver_status(row["side"]).get("position_seq", 0))
                > int(execution["status_baseline"].get(row["side"], -1))
                and not self._fresh_driver_status(row["side"]).get("position_active", True)
                and self._fresh_driver_status(row["side"]).get("position_status") == "sent_arrival_unverified"
                and self._fresh_driver_status(row["side"]).get("position_error_rad") is not None
                and self._fresh_driver_status(row["side"])["position_error_rad"] <= 0.05
                for row in self.arms
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
