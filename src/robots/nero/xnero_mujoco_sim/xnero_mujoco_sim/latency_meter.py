"""Rolling latency stats, printed every interval."""

from __future__ import annotations

import time
from collections import defaultdict
from typing import Dict, List


class LatencyMeter:
    def __init__(self, print_interval: float = 2.0) -> None:
        self.print_interval = float(print_interval)
        self._last_print = time.monotonic()
        self._ms: Dict[str, List[float]] = defaultdict(list)
        self._counts: Dict[str, int] = defaultdict(int)

    def add(self, name: str, ms: float) -> None:
        if ms != ms:  # NaN
            return
        self._ms[name].append(float(ms))

    def count(self, name: str, n: int = 1) -> None:
        self._counts[name] += int(n)

    def should_print(self) -> bool:
        now = time.monotonic()
        if now - self._last_print >= self.print_interval:
            self._last_print = now
            return True
        return False

    def has_samples(self) -> bool:
        return bool(self._ms) or bool(self._counts)

    def format_and_reset(self) -> str:
        parts: List[str] = []
        for name in self._ms:
            xs = self._ms[name]
            if not xs:
                continue
            n = len(xs)
            mean = sum(xs) / n
            xs_sorted = sorted(xs)
            p95 = xs_sorted[min(n - 1, int(n * 0.95))]
            parts.append(
                f"{name}={mean:.1f}ms(p95={p95:.1f},max={xs_sorted[-1]:.1f},n={n})"
            )
        for name in self._counts:
            parts.append(f"{name}={self._counts[name]}")
        self._ms.clear()
        self._counts.clear()
        return " ".join(parts) if parts else "(no samples)"


def stamp_age_ms(stamp, now_wall: float | None = None) -> float:
    """Age of a ROS stamp that was filled from ``time.time()``."""
    sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if sec <= 0.0:
        return float("nan")
    now = time.time() if now_wall is None else now_wall
    return (now - sec) * 1000.0
