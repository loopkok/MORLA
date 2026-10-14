"""Pure operation guards and operation records for the shared coordinator."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from morla_bringup.coordinator_logic import (  # noqa: E402
    OPERATIONS,
    OperationRecord,
    operation_guard,
    writer_statuses_match,
    writers_have_manual_ownership,
)


def test_motion_requires_fresh_idle_collector_state():
    caps = {"workpos": True}
    for state in (None, "RECORDING", "PAUSED", "SAVING", "STOPPED"):
        allowed, reason = operation_guard("workpos", caps, state)
        assert not allowed
        assert reason
    assert operation_guard("workpos", caps, "IDLE") == (True, "")
    assert operation_guard("workpos", caps, "NO_COLLECTOR") == (True, "")


def test_profile_capability_is_authoritative():
    assert operation_guard("workpos", {"workpos": False}, "IDLE")[0] is False
    assert operation_guard("damping", {"damping": True}, "IDLE")[0] is True


def test_estop_remains_available_during_recording():
    assert operation_guard("estop", {"estop": True}, "RECORDING") == (True, "")
    assert operation_guard("estop", {"estop": False}, "IDLE")[0] is False


def test_lifecycle_controls_do_not_require_motion_capabilities():
    caps = {}
    for operation in ("pause", "resume", "start", "release"):
        assert operation_guard(operation, caps, None) == (True, "")


def test_unknown_control_is_rejected():
    assert "bogus" not in OPERATIONS
    allowed, reason = operation_guard("bogus", {}, "IDLE")
    assert not allowed
    assert "unknown" in reason


def test_operation_record_has_correlatable_lifecycle_fields():
    record = OperationRecord.create("workpos", 12.5)
    assert record.operation_id
    record.update("completed", "measured target reached", 13.0, result=True)
    state = record.to_dict("astral_left_gripper")
    assert state["operation_id"] == record.operation_id
    assert state["profile_id"] == "astral_left_gripper"
    assert state["phase"] == "completed"
    assert state["result"] is True


def test_writer_freeze_requires_every_writer_fresh_and_disarmed():
    writer_ids = ["left_arm", "left_ee"]
    status = {"left_arm": {"armed": False}, "left_ee": {"armed": False}}
    received = {"left_arm": 9.5, "left_ee": 9.5}
    assert writer_statuses_match(writer_ids, status, received, "armed", False, now=10.0)
    assert not writer_statuses_match(writer_ids, status, received, "armed", True, now=10.0)
    assert not writer_statuses_match(writer_ids, {"left_arm": status["left_arm"]}, received,
                                     "armed", False, now=10.0)
    assert not writer_statuses_match(writer_ids, status, {"left_arm": 9.5, "left_ee": 8.0},
                                     "armed", False, now=10.0)


def test_policy_owned_writer_blocks_manual_freeze_assumption():
    ids = ["left_arm", "left_ee"]
    received = {"left_arm": 10.0, "left_ee": 10.0}
    status = {
        "left_arm": {"ownership": "human"},
        "left_ee": {"ownership": "policy"},
    }
    assert not writers_have_manual_ownership(ids, status, received, now=10.1)
    status["left_ee"]["ownership"] = None
    assert writers_have_manual_ownership(ids, status, received, now=10.1)
