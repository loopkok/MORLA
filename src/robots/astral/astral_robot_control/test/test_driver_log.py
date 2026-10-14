#!/usr/bin/env python3
"""Pure tests for driver_log.py (DriverJsonlLog + spike_mrad) — no ROS needed.

Covers the append-only JSONL writer semantics and the per-tick joint-jump
spike detector used by the driver diagnostics log (抓"电机抽一下").
"""

from __future__ import annotations

import json
import os
import sys
import tempfile

import numpy as np

from astral_robot_control.driver_log import DriverJsonlLog, spike_mrad


def test_jsonl_write() -> None:
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "driver.jsonl")
        lg = DriverJsonlLog(path)
        assert lg.enabled and lg.path == path
        lg.write({"kind": "send", "t": 1.0, "left": [0.1, 0.2]})
        lg.write({"kind": "spike", "t": 1.5, "jump_mrad": 100.0})
        lg.close()
        assert not lg.enabled
        with open(path) as f:
            lines = [json.loads(x) for x in f if x.strip()]
        assert len(lines) == 2 and lines[0]["kind"] == "send" and lines[1]["jump_mrad"] == 100.0


def test_jsonl_empty_disabled() -> None:
    lg = DriverJsonlLog("")
    assert not lg.enabled and lg.path == ""
    lg.write({"kind": "x"})  # no-op, no raise


def test_jsonl_write_failure_safe() -> None:
    with tempfile.TemporaryDirectory() as d:
        lg = DriverJsonlLog(os.path.join(d, "driver.jsonl"))
        lg.write({"kind": "spike", "bad": object()})  # json.dumps raises inside → swallowed
        lg.write({"kind": "spike", "jump_mrad": 5.0})
        lg.close()
        with open(os.path.join(d, "driver.jsonl")) as f:
            lines = [x for x in f if x.strip()]
        assert len(lines) == 1  # only the valid record survived


def test_spike_mrad_basic() -> None:
    assert spike_mrad(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]), 30.0) == 0.0
    # 0.3 rad = 300 mrad jump on one joint → spike
    assert spike_mrad(np.array([0.0, 0.3, 0.0]), np.array([0.0, 0.0, 0.0]), 30.0) == 300.0
    # below threshold → 0
    assert spike_mrad(np.array([0.0, 0.02, 0.0]), np.array([0.0, 0.0, 0.0]), 30.0) == 0.0
    # None inputs → 0 (no judgement)
    assert spike_mrad(None, np.zeros(3), 30.0) == 0.0
    assert spike_mrad(np.zeros(3), None, 30.0) == 0.0


def test_spike_threshold_boundary() -> None:
    # exactly at threshold → report (>) vs just below → 0
    assert spike_mrad(np.array([0.03]), np.array([0.0]), 30.0) == 0.0  # 30 mrad not > 30
    assert spike_mrad(np.array([0.031]), np.array([0.0]), 30.0) == 31.0


if __name__ == "__main__":
    import traceback

    for fn in (test_jsonl_write, test_jsonl_empty_disabled, test_jsonl_write_failure_safe,
               test_spike_mrad_basic, test_spike_threshold_boundary):
        try:
            fn()
            print(f"  {fn.__name__}: PASS")
        except Exception:
            traceback.print_exc()
            print(f"  {fn.__name__}: FAIL")
            sys.exit(1)
    print("all driver_log tests PASS")
