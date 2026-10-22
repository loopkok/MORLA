"""Adversarial storage/session tests; the software worker opens no hardware."""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import sys
import threading
import time

import pytest

from morla_web_manager.piano_manager import PianoManager
from morla_web_manager.piano_store import PianoConflict, PianoError, PianoStore

PROJECT = Path(os.environ.get("MORLA_PIANO_PROJECT", str(Path(__file__).resolve().parents[5] / "MORLA-piano")))
if not PROJECT.is_dir():
    PROJECT = Path.home() / "Desktop/MORLA-piano"


@pytest.fixture
def store(tmp_path):
    if not PROJECT.is_dir():
        pytest.fail("Set MORLA_PIANO_PROJECT to the independent project to run piano integration tests")
    return PianoStore(PROJECT, tmp_path)


def wait_until(predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        result = predicate()
        if result:
            return result
        time.sleep(.02)
    raise AssertionError("condition timed out")


@pytest.fixture
def manager(store):
    record = store.config()
    cfg = record["config"]
    cfg["timing"].update(prepare_duration_s=1, lead_in_s=.2, max_lateness_s=.1)
    store.save_config(cfg, record["revision"])
    instance = PianoManager(store, python=sys.executable)
    yield instance
    instance.close()


def open_software(manager):
    result = manager.open("software", manager.store.config()["revision"])
    wait_until(lambda: manager.snapshot()["state"] == "CONNECTED")
    return result["token"]


def wait_job(manager, job, token):
    def completed():
        manager.heartbeat(token)
        state = manager.snapshot()
        result = state.get("result")
        if result and result.get("job_id") == job["id"]:
            return result
    return wait_until(completed)


def test_placeholder_and_seeded_score_preflight(store):
    assert store.config()["config"]["calibrated"] is False
    store.seed()
    assert len(store.list_scores()) == 2
    for score in store.list_scores():
        _, plan = store.plan(score["id"])
        assert plan["notes"] > 0


def test_config_compare_and_swap_prevents_lost_update(store):
    a, b = store.config(), store.config()
    a["config"]["timing"]["lead_in_s"] = 3
    store.save_config(a["config"], a["revision"])
    with pytest.raises(PianoConflict):
        store.save_config(b["config"], b["revision"])


def test_edit_invalidates_calibration_unless_explicitly_certified(store):
    record = store.config()
    certified = store.save_config(record["config"], record["revision"], True)
    assert certified["config"]["calibrated"] is True
    cfg = certified["config"]
    cfg["timing"]["lead_in_s"] = 3
    assert store.save_config(cfg, certified["revision"])["config"]["calibrated"] is False


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -.1, 2])
def test_velocity_cannot_disable_safety_limits(store, value):
    cfg = store.config()["config"]
    cfg["timing"]["max_arm_velocity_rad_s"] = value
    with pytest.raises((PianoError, ValueError)):
        store.validate(cfg)


def test_hard_joint_limits_cannot_be_widened(store):
    cfg = store.config()["config"]
    cfg["hands"]["left"]["arm_limits_rad"][0] = [-100, 100]
    with pytest.raises(PianoError, match="放宽"):
        store.validate(cfg)


def test_thumb_cannot_be_selected_as_press_joint(store):
    cfg = store.config()["config"]
    cfg["hands"]["right"]["press_rad"]["index"]["thumb_bend_joint"] = .2
    with pytest.raises(ValueError, match="cannot command"):
        store.validate(cfg)


@pytest.mark.parametrize("score_id", ["../config", "/etc/passwd", "x; touch /tmp/pwned", "0" * 33, ""])
def test_score_ids_do_not_become_filesystem_paths(store, score_id):
    with pytest.raises(PianoError):
        store.score(score_id)


def test_nonfinite_json_and_oversized_import_rejected(store):
    with pytest.raises(PianoError):
        store.import_score("bad", '{"tempo_bpm": NaN, "events": []}')
    with pytest.raises(PianoError):
        store.import_score("large", "x" * 512001)


def test_invalid_chord_cannot_reach_worker(store):
    score = {"tempo_bpm": 60, "events": [{"start": 0, "duration": 1, "hand": hand, "note": note} for hand, note in (("left", "C3"), ("right", "C4"))]}
    imported = store.import_score("chord", json.dumps(score))
    with pytest.raises(ValueError, match="overlapping"):
        store.plan(imported["id"])


def test_session_is_exclusive_and_editing_is_locked(manager):
    token = open_software(manager)
    with pytest.raises(PianoConflict):
        manager.open("software", manager.store.config()["revision"])
    with pytest.raises(PianoConflict):
        manager.editable()
    with pytest.raises(PianoConflict):
        manager.command("wrong-token", "prepare")
    assert wait_job(manager, manager.command(token, "capture"), token)["ok"]


def test_full_software_prepare_finger_shift_return_and_play(manager):
    token = open_software(manager)
    score = {"tempo_bpm": 120, "gate": .5, "events": [
        {"start": 0, "duration": 1, "hand": "left", "note": "C3"},
        {"start": 1, "duration": 1, "hand": "right", "note": "C4"}]}
    imported = manager.store.import_score("short", json.dumps(score))
    for operation, args in (("prepare", {}), ("finger", {"side": "left", "finger": "pinky"}),
                            ("shift", {"side": "right", "shift_white": 1}), ("prepare", {}),
                            ("play", {"score_id": imported["id"]})):
        result = wait_job(manager, manager.command(token, operation, **args), token)
        assert result["ok"], (result, manager.snapshot())
    assert manager.snapshot()["state"] == "FINISHED"
    assert manager.snapshot()["pressed"] is None


def test_parallel_operations_and_stale_ready_are_rejected(manager):
    token = open_software(manager)
    job = manager.command(token, "prepare")
    with pytest.raises(PianoConflict, match="正在执行"):
        manager.command(token, "capture")
    assert wait_job(manager, job, token)["ok"]
    moved = manager.command(token, "shift", side="left", shift_white=1)
    assert wait_job(manager, moved, token)["ok"]
    manager.store.seed()
    with pytest.raises(PianoConflict, match="初始位"):
        manager.command(token, "play", score_id=manager.store.list_scores()[0]["id"])


def test_invalid_worker_operation_reports_error_without_crashing(manager):
    token = open_software(manager)
    result = wait_job(manager, manager.command(token, "finger", side="left", finger="index"), token)
    assert result["ok"] is False
    assert manager.proc.poll() is None
    assert wait_job(manager, manager.command(token, "capture"), token)["ok"]


def test_stop_during_prepare_does_not_continue_queued_motion(manager):
    token = open_software(manager)
    manager.command(token, "prepare")
    manager.stop()
    wait_until(lambda: manager.snapshot().get("state") in {"STOPPED", "CONNECTED"} and not manager.snapshot()["busy"])
    assert manager.proc.poll() is None


def test_browser_lease_loss_reaps_owned_process(manager):
    manager.LEASE_SECONDS = .25
    manager.IDLE_LEASE_SECONDS = .25
    open_software(manager)
    proc = manager.proc
    wait_until(lambda: not manager.active)
    assert proc.poll() is not None


def test_failed_spawn_does_not_leave_a_reserved_lane(manager, monkeypatch):
    def fail(*args, **kwargs):
        raise FileNotFoundError("no interpreter")
    monkeypatch.setattr(manager, "_spawn", fail)
    with pytest.raises(FileNotFoundError):
        manager.open("software", manager.store.config()["revision"])
    assert manager.active is False


def test_two_threads_cannot_acquire_same_session(manager):
    results = []
    def acquire():
        try:
            results.append(manager.open("software", manager.store.config()["revision"]))
        except PianoConflict:
            results.append(None)
    a, b = threading.Thread(target=acquire), threading.Thread(target=acquire)
    a.start(); b.start(); a.join(); b.join()
    assert sum(result is not None for result in results) == 1


def test_separate_stores_share_compare_and_swap_file_lock(store):
    other = PianoStore(store.project, store.root)
    record = store.config()
    results = []
    def save(instance, seconds):
        cfg = copy.deepcopy(record['config'])
        cfg['timing']['lead_in_s'] = seconds
        try:
            instance.save_config(cfg, record['revision'])
            results.append(True)
        except PianoConflict:
            results.append(False)
    a = threading.Thread(target=save, args=(store, 3))
    b = threading.Thread(target=save, args=(other, 4))
    a.start(); b.start(); a.join(); b.join()
    assert results.count(True) == 1


def test_separate_web_instances_cannot_share_ros_domain(manager, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '212')
    other = PianoManager(manager.store)
    manager._resource_lock('ros_fake')
    try:
        with pytest.raises(PianoConflict):
            other._resource_lock('ros_fake')
    finally:
        manager._release_resource_lock()
        other._release_resource_lock()


def test_physical_lock_is_shared_across_ros_domains(manager, monkeypatch):
    other = PianoManager(manager.store)
    monkeypatch.setenv('ROS_DOMAIN_ID', '213')
    manager._resource_lock('hardware')
    try:
        monkeypatch.setenv('ROS_DOMAIN_ID', '214')
        with pytest.raises(PianoConflict):
            other._resource_lock('hardware')
    finally:
        manager._release_resource_lock()
        other._release_resource_lock()


def test_worker_crash_releases_exclusive_lane(manager):
    open_software(manager)
    proc = manager.proc
    proc.kill()
    wait_until(lambda: not manager.active)
    assert proc.poll() is not None
    assert manager.proc is None


def test_press_shorter_than_feedback_window_fails_preflight(store):
    score = {"tempo_bpm": 300, "gate": .1, "events": [{"start": 0, "duration": .1, "hand": "left", "note": "C3"}]}
    imported = store.import_score("too short", json.dumps(score))
    with pytest.raises(PianoError, match="太短"):
        store.plan(imported['id'])
