"""Simple sliding-window FPS counter for real-time monitoring."""
import time


class FPSCounter:
    def __init__(self, window: int = 50, print_interval: float = 5.0):
        self._window = window
        self._print_interval = print_interval
        self._times = []
        self._last_print = time.time()

    def tick(self) -> float:
        """Call on each frame. Returns current FPS."""
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / max(1e-9, self._times[-1] - self._times[0])

    def should_print(self) -> bool:
        now = time.time()
        if now - self._last_print >= self._print_interval:
            self._last_print = now
            return True
        return False

    def get_fps(self) -> float:
        return self.tick()
