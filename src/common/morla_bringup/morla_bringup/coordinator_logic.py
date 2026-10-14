"""Pure policy and lifecycle rules for the MORLA control coordinator."""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field


MOTION_OPERATIONS = frozenset({
    "ready", "enable", "position", "damping", "home", "workpos", "workpos_direct",
})
OPERATIONS = frozenset({
    "pause", "resume", "start", "release", *MOTION_OPERATIONS, "estop",
})
RECORDING_STATES = frozenset({"RECORDING", "PAUSED", "SAVING"})


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
