"""Motion stream liveness; callers serialize access with their parser lock."""

import math


class StreamHealth:
    def __init__(self, timeout_s=0.5):
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("input_timeout_s must be finite and positive")
        self.streams = {}

    def clear(self):
        self.streams.clear()

    def observe(self, payload, peer, now):
        # Buttons may be event-driven. Only pose/landmark streams renew motion.
        stream = payload.split(":", 1)[0].split("|", 1)[0].strip().lower()
        if "buttons" in stream:
            return False
        if not (stream in {"left", "right", "head", "hmd", "body"}
                or "wrist" in stream or "controller" in stream or "landmarks" in stream):
            return False
        previous = self.streams.get(stream)
        changed = previous is not None and (
            previous[0] != peer or now - previous[1] > self.timeout_s
        )
        self.streams[stream] = (peer, now)
        return changed

    def expired(self, now):
        return [stream for stream, (_, received) in self.streams.items()
                if now - received > self.timeout_s]

    def disconnected(self, peer):
        # Quest uses independent sockets for hands, controllers, head and body.
        # Closing an obsolete socket must not invalidate its replacement.
        return [stream for stream, (owner, _) in self.streams.items() if owner == peer]
