"""Simple sliding-window FPS counter."""

from __future__ import annotations

import time
from collections import deque


class FPSCounter:
    def __init__(self, window: int = 100, print_interval: float = 5.0):
        self._ts: deque = deque(maxlen=max(2, int(window)))
        self.print_interval = float(print_interval)
        self._last_print = 0.0

    def tick(self) -> float:
        now = time.monotonic()
        self._ts.append(now)
        if len(self._ts) < 2:
            return 0.0
        dt = self._ts[-1] - self._ts[0]
        return (len(self._ts) - 1) / dt if dt > 1e-9 else 0.0

    def should_print(self) -> bool:
        now = time.monotonic()
        if now - self._last_print >= self.print_interval:
            self._last_print = now
            return True
        return False
