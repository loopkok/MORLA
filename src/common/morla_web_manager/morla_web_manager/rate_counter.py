"""EWMA-based publish-rate estimator for command topics.

Each command topic gets one RateCounter. Every arriving message calls tick();
hz() returns the exponentially-weighted moving average of the inter-arrival
period. This mirrors the sliding-window Hz estimation used in the wuji
teleop monitor's joint panel.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from .config import RATE_EWMA_ALPHA


@dataclass
class RateCounter:
    name: str
    _alpha: float = RATE_EWMA_ALPHA
    _last_ts: float = 0.0
    _ewma_period: float = 0.0
    _count: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def tick(self, ts: float | None = None) -> None:
        ts = ts if ts is not None else time.time()
        with self._lock:
            if self._last_ts > 0.0:
                dt = ts - self._last_ts
                if dt > 0.0:
                    if self._ewma_period == 0.0:
                        self._ewma_period = dt
                    else:
                        self._ewma_period = (
                            self._alpha * dt + (1.0 - self._alpha) * self._ewma_period
                        )
            self._last_ts = ts
            self._count += 1

    def hz(self) -> float:
        with self._lock:
            if self._ewma_period <= 0.0:
                return 0.0
            return 1.0 / self._ewma_period

    def count(self) -> int:
        with self._lock:
            return self._count

    def reset(self) -> None:
        with self._lock:
            self._last_ts = 0.0
            self._ewma_period = 0.0
            self._count = 0


class RateRegistry:
    """Named collection of RateCounters, built from the TOPICS config."""

    def __init__(self) -> None:
        from .config import TOPICS
        self._counters: dict[str, RateCounter] = {}
        for entity, pair in TOPICS.items():
            if pair.command:
                self._counters[f"{entity}_cmd"] = RateCounter(f"{entity}_cmd")

    @classmethod
    def for_state_topics(cls) -> "RateRegistry":
        """Registry that ticks on state topics (one counter per entity)."""
        from .config import TOPICS
        reg = cls.__new__(cls)
        reg._counters = {
            f"{entity}_state": RateCounter(f"{entity}_state")
            for entity in TOPICS
        }
        return reg

    def get(self, key: str) -> RateCounter | None:
        return self._counters.get(key)

    def tick(self, key: str, ts: float | None = None) -> None:
        c = self._counters.get(key)
        if c is not None:
            c.tick(ts)

    def snapshot(self) -> dict[str, float]:
        return {k: c.hz() for k, c in self._counters.items()}

    def reset_all(self) -> None:
        for c in self._counters.values():
            c.reset()
