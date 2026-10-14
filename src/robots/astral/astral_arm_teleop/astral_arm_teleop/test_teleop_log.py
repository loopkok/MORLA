#!/usr/bin/env python3
"""Teleop JSONL diagnostics log helpers (teleop_log.py) + LatencyMeter.snapshot.

Pure-logic tests, no ROS / no hardware: verify the per-side path splitting,
append-only JSONL writing (incl. write-failure safety) and the non-destructive
LatencyMeter window snapshot used by the kind=metrics records.

Usage:
  /usr/bin/python3 -m pytest test_teleop_log.py -q
  ros2 run astral_arm_teleop test_teleop_log
"""

from __future__ import annotations

import json
import os
import sys
import tempfile

from astral_arm_teleop.latency_meter import LatencyMeter
from astral_arm_teleop.teleop_log import TeleopJsonlLog, side_log_path


def test_side_log_path() -> bool:
    print("\n" + "=" * 60)
    print("TEST: side_log_path per-side split")
    print("=" * 60)
    ok = True
    cases = [
        # base, side, expected
        ("/tmp/teleop_teleop.jsonl", "left", "/tmp/teleop_teleop_left.jsonl"),
        ("/tmp/teleop_teleop.jsonl", "right", "/tmp/teleop_teleop_right.jsonl"),
        # already per-side → pass through unchanged
        ("/tmp/teleop_teleop_left.jsonl", "right", "/tmp/teleop_teleop_left.jsonl"),
        ("/tmp/teleop_teleop_right.jsonl", "left", "/tmp/teleop_teleop_right.jsonl"),
        # no .jsonl suffix → append
        ("/tmp/teleop", "left", "/tmp/teleop_left.jsonl"),
        # empty base → disabled
        ("", "left", ""),
        ("  ", "right", ""),
    ]
    for base, side, expect in cases:
        got = side_log_path(base, side)
        good = got == expect
        print(f"  side_log_path({base!r}, {side!r}) → {got!r} "
              f"[{'PASS' if good else 'FAIL'}]")
        ok &= good
    return ok


def test_teleop_jsonl_log() -> bool:
    print("\n" + "=" * 60)
    print("TEST: TeleopJsonlLog append-only JSONL write")
    print("=" * 60)
    ok = True
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "diag.jsonl")
        lg = TeleopJsonlLog(path)
        ok &= lg.enabled
        ok &= lg.path == path
        lg.write({"kind": "loop", "t": 1.0, "q": [0.0] * 7})
        lg.write({"kind": "wrist", "t": 1.5, "pos": [0.1, 0.2, 0.3]})
        lg.close()
        ok &= not lg.enabled
        with open(path) as f:
            lines = [json.loads(x) for x in f if x.strip()]
        good = len(lines) == 2 and lines[0]["kind"] == "loop" and lines[1]["kind"] == "wrist"
        print(f"  wrote 2 records, read back {len(lines)}: "
              f"[{'PASS' if good else 'FAIL'}]")
        ok &= good
    # empty path → disabled, no file created
    lg2 = TeleopJsonlLog("")
    good = (not lg2.enabled) and lg2.path == ""
    print(f"  empty path disabled, no file: [{'PASS' if good else 'FAIL'}]")
    ok &= good
    return ok


def test_teleop_log_write_failure_safe() -> bool:
    print("\n" + "=" * 60)
    print("TEST: write failure must not raise / corrupt control flow")
    print("=" * 60)
    ok = True
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "diag.jsonl")
        lg = TeleopJsonlLog(path)
        # non-JSON-serialisable value → json.dumps raises inside write() → swallowed
        bad = {"kind": "loop", "payload": object()}
        try:
            lg.write(bad)
            good = True
        except Exception:  # noqa: BLE001
            good = False
        print(f"  non-serialisable record did not raise: [{'PASS' if good else 'FAIL'}]")
        ok &= good
        # the writer must still work for a valid record afterwards
        lg.write({"kind": "loop", "ok": 1})
        lg.close()
        with open(path) as f:
            lines = [x for x in f if x.strip()]
        good = len(lines) == 1
        print(f"  file kept only valid records ({len(lines)}): "
              f"[{'PASS' if good else 'FAIL'}]")
        ok &= good
    return ok


def test_latency_meter_snapshot() -> bool:
    print("\n" + "=" * 60)
    print("TEST: LatencyMeter.snapshot non-destructive")
    print("=" * 60)
    ok = True
    m = LatencyMeter(print_interval=999.0)
    m.add("ik", 1.0)
    m.add("ik", 3.0)
    m.count("ik_fail", 2)
    snap = m.snapshot()
    good = snap["ms"]["ik"]["mean"] == 2.0 and snap["counts"]["ik_fail"] == 2
    print(f"  snapshot stats={snap} [{'PASS' if good else 'FAIL'}]")
    ok &= good
    # non-destructive: format_and_reset still sees the same samples
    fmt = m.format_and_reset()
    good = "ik=2.0ms" in fmt and "ik_fail=2" in fmt
    print(f"  after snapshot, format_and_reset={fmt!r} [{'PASS' if good else 'FAIL'}]")
    ok &= good
    # empty meter → empty snapshot
    good = LatencyMeter().snapshot() == {"ms": {}, "counts": {}}
    print(f"  empty meter snapshot: [{'PASS' if good else 'FAIL'}]")
    ok &= good
    return ok


def main():
    print("Astral Teleop — JSONL diagnostics log validation")
    results = {
        "side_log_path": test_side_log_path(),
        "teleop_jsonl_log": test_teleop_jsonl_log(),
        "write_failure_safe": test_teleop_log_write_failure_safe(),
        "latency_snapshot": test_latency_meter_snapshot(),
    }
    print("\n" + "=" * 60)
    print("SUMMARY")
    for k, v in results.items():
        print(f"  {k}: {'PASS' if v else 'FAIL'}")
    if not all(results.values()):
        sys.exit(1)
    print("\nAll teleop log tests PASSED.")


if __name__ == "__main__":
    main()
