"""Launch 日志：环形缓冲 8000、WS 实时推最后 800 行。"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from astral_web_monitor.config import (  # noqa: E402
    WS_LOG_PUSH_LINES,
    WS_LOG_TAIL_LINES,
    log_tail_for_push,
)


def test_ring_and_push_sizes() -> None:
    assert WS_LOG_TAIL_LINES == 8000
    assert WS_LOG_PUSH_LINES == 800


def test_push_keeps_last_800() -> None:
    lines = [str(i) for i in range(2000)]
    out = log_tail_for_push(lines)
    assert len(out) == 800
    assert out[0] == "1200"
    assert out[-1] == "1999"


def test_push_short_list_unchanged() -> None:
    lines = ["a", "b"]
    assert log_tail_for_push(lines) == ["a", "b"]
