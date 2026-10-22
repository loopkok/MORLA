"""Pure policy and lifecycle rules for the MORLA control coordinator."""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field


MOTION_OPERATIONS = frozenset({
    "prepare", "ready", "initialize_can", "enable", "reset", "position", "damping", "home", "workpos", "workpos_direct",
})
OPERATIONS = frozenset({
    "pause", "resume", "start", "release", *MOTION_OPERATIONS, "estop",
})
RECORDING_STATES = frozenset({"RECORDING", "PAUSED", "SAVING"})


def nero_home_blockers(status: dict) -> list[str]:
    """Gate planned J motion using measured motors, not streaming permission."""
    if not status:
        return ["驱动状态缺失或过期，请检查节点与连接"]
    reasons = []
    if not status.get("connected"):
        reasons.append("驱动未连接")
    if not status.get("feedback_fresh"):
        reasons.append("关节反馈缺失或过期")
    code = status.get("controller_arm_status")
    if not status.get("controller_status_fresh"):
        reasons.append("控制器状态缺失或过期")
    elif code == 1:
        reasons.append("控制器仍处于急停（状态 1），请先解除并复位急停")
    elif code == 6:
        reasons.append("关节抱闸未打开（状态 6），请先点击“准备并使能”并等待完成")
    elif code != 0:
        reasons.append(f"控制器异常（状态 {code}），请先排查故障")
    if status.get("motor_status_fresh") is None:
        reasons.append("驱动尚未提供电机使能反馈，请重启机器人 Profile 加载新驱动")
    elif status.get("motor_status_fresh") is not True:
        reasons.append("七关节电机使能反馈缺失或过期，请检查反馈与连接")
    elif status.get("motors_enabled") is not True and code not in {1, 6}:
        reasons.append("电机未全部使能，请先点击“准备并使能”并等待完成")
    elif status.get("position_permitted") is not True and code not in {1, 6}:
        reasons.append("位置动作许可已撤销，请先点击“准备并使能”并等待完成")
    return reasons


def nero_start_blockers(status: dict) -> list[str]:
    """Streaming requires live measured hardware, independently of writer armed."""
    if not status:
        return ["驱动状态缺失或过期，请检查连接并重启机器人 Profile"]
    reasons = []
    if status.get("connected") is not True:
        reasons.append("驱动未连接")
    if status.get("feedback_fresh") is not True or status.get("feedback_read_ok") is False:
        reasons.append("实测关节反馈缺失或过期")
    if status.get("controller_status_fresh") is not True:
        reasons.append("控制器状态缺失或过期")
    elif status.get("controller_arm_status") != 0:
        reasons.append("控制器未就绪，请检查急停或故障并准备使能")
    if status.get("motor_status_fresh") is not True:
        reasons.append("电机使能反馈缺失或过期")
    elif status.get("motors_enabled") is not True:
        reasons.append("电机未全部使能，请先准备并使能")
    if status.get("streaming_enabled") is not True:
        reasons.append("遥操流式许可未开启，请先点击“准备并使能”并等待完成")
    if status.get("position_active"):
        reasons.append("位置轨迹尚未结束")
    return reasons


def operation_guard(
    operation: str, capabilities: dict, collector_state: str | None,
) -> tuple[bool, str]:
    """Return whether a request may be accepted and its concrete rejection."""
    if operation not in OPERATIONS:
        return False, f"unknown control operation {operation!r}"
    capability = "workpos" if operation == "workpos_direct" else operation
    if operation not in {"pause", "resume", "start", "release"} and not capabilities.get(capability, False):
        return False, f"profile capability {capability!r} is disabled"
    if operation in MOTION_OPERATIONS:
        if collector_state == "NO_COLLECTOR":
            return True, ""
        if collector_state is None:
            return False, "data collection state is unknown; motion operation was not started"
        if collector_state in RECORDING_STATES:
            return False, f"motion operation is blocked while data collection is {collector_state}"
        if collector_state != "IDLE":
            return False, f"motion operation requires data collection IDLE, got {collector_state}"
    return True, ""


def writer_statuses_match(
    writer_ids: list[str], statuses: dict[str, dict], received: dict[str, float],
    key: str, expected, *, now: float, max_age_s: float = 1.0,
) -> bool:
    """Fail closed if any configured writer is missing, stale, or mismatched."""
    if not writer_ids:
        return False
    return all(
        writer_id in statuses
        and now - received.get(writer_id, 0.0) <= max_age_s
        and statuses[writer_id].get(key) == expected
        for writer_id in writer_ids
    )


def writers_have_manual_ownership(
    writer_ids: list[str], statuses: dict[str, dict], received: dict[str, float],
    *, now: float, max_age_s: float = 1.0,
) -> bool:
    """Require fresh status for every writer and reject policy/unknown ownership."""
    if not writer_ids:
        return False
    return all(
        writer_id in statuses
        and now - received.get(writer_id, 0.0) <= max_age_s
        and statuses[writer_id].get("ownership") in (None, "human")
        for writer_id in writer_ids
    )


@dataclass
class OperationRecord:
    operation_id: str
    operation: str
    phase: str = "accepted"
    started_at: float = 0.0
    updated_at: float = 0.0
    result: bool | None = None
    message: str = "accepted"
    components: dict[str, dict] = field(default_factory=dict)

    @classmethod
    def create(cls, operation: str, now: float) -> "OperationRecord":
        return cls(uuid.uuid4().hex, operation, started_at=now, updated_at=now)

    def update(self, phase: str, message: str, now: float, *, result=None) -> None:
        self.phase = str(phase)
        self.message = str(message)
        self.updated_at = float(now)
        self.result = result

    def to_dict(self, profile_id: str) -> dict:
        return {
            "operation_id": self.operation_id,
            "operation": self.operation,
            "phase": self.phase,
            "result": self.result,
            "message": self.message,
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "profile_id": profile_id,
            "components": self.components,
        }


def preparation_steps(capabilities: dict, *, feedback_ready: bool) -> list[str]:
    """One preparation entry point, using declared capabilities, without homing."""
    if not capabilities.get("enable", False):
        raise ValueError("this runtime does not support hardware enable")
    steps = []
    if capabilities.get("initialize_can", False) and not feedback_ready:
        steps.append("initialize_can")
    steps.append("enable")
    return steps
