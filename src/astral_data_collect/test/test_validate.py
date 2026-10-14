"""validate_data 单元测试：健康段通过、各类缺陷被对应规则捕获、隔离生效。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.pipeline.validate_data import (  # noqa: E402
    QUARANTINE_DIR,
    validate_episode,
    validate_session,
)
from conftest import write_raw_episode  # noqa: E402


def test_healthy_episode_passes(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"))
    rep = validate_episode(ep)
    assert rep.status == "pass", rep.issues


def test_nan_fails_f3(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"), nan_in="left_arm_state")
    rep = validate_episode(ep)
    assert rep.status == "fail"
    assert any(i["rule"] == "F3" for i in rep.issues)


def test_missing_stream_fails_f2(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"), drop_stream="right_arm_cmd")
    rep = validate_episode(ep)
    assert rep.status == "fail"
    assert any(i["rule"] == "F2" for i in rep.issues)


def test_short_episode_fails_f5(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"), duration_s=1.0)
    rep = validate_episode(ep, min_duration_s=2.0)
    assert rep.status == "fail"
    assert any(i["rule"] == "F5" for i in rep.issues)


def test_gap_warns_w2(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"), gap_in="left_arm_state")
    rep = validate_episode(ep)
    assert rep.status == "warn"
    assert any(i["rule"] == "W2" for i in rep.issues)


def test_disarmed_warns_w5(tmp_path):
    ep = write_raw_episode(
        str(tmp_path / "episode000000"),
        duration_s=3.0,
        events=[
            {"t": 1_700_000_000.0, "name": "episode_start", "data": True},
            {"t": 1_700_000_000.0, "name": "teleop_disarmed", "data": True},
            {"t": 1_700_000_003.0, "name": "episode_end", "data": True},
        ],
    )
    rep = validate_episode(ep)
    assert any(i["rule"] == "W5" for i in rep.issues)


def test_missing_files_fail_f1(tmp_path):
    ep = tmp_path / "episode000099"
    os.makedirs(ep)
    rep = validate_episode(str(ep))
    assert rep.status == "fail"
    assert any(i["rule"] == "F1" for i in rep.issues)


def test_session_quarantine_apply(tmp_path):
    root = tmp_path / "session"
    write_raw_episode(str(root / "episode000000"))
    write_raw_episode(str(root / "episode000001"), nan_in="right_arm_state")

    summary = validate_session(str(root), apply=True, log=lambda *a: None)
    assert summary["pass"] == 1
    assert summary["fail"] == 1
    assert summary["quarantined"] == 1
    # fail 段被移入 quarantine/，pass 段原处保留
    assert os.path.isdir(root / QUARANTINE_DIR / "episode000001")
    assert os.path.isdir(root / "episode000000")
    assert not os.path.isdir(root / "episode000001")
    assert os.path.exists(root / "validation_report.json")


def test_session_no_apply_keeps_dirs(tmp_path):
    root = tmp_path / "session"
    write_raw_episode(str(root / "episode000000"), duration_s=1.0)
    summary = validate_session(str(root), apply=False, log=lambda *a: None)
    assert summary["fail"] == 1
    assert os.path.isdir(root / "episode000000")
    assert not (root / QUARANTINE_DIR).exists()
