"""Quest source-clock/sequence envelope; shared by ROS and standalone receivers."""
import math
import threading
import time


class PacketGuard:
    def __init__(self, max_age=0.5, allow_legacy=False):
        self.max_age = max_age
        self.allow_legacy = allow_legacy
        self._sequences = {}
        self._sessions = {}
        self._retired = set()
        self._lock = threading.Lock()
        self.rejected = 0

    def accept(self, line, received=None):
        received = time.time() if received is None else received
        if not line.startswith('@q3 '):
            if self.allow_legacy: return line, received
            self.rejected += 1
            return None
        try:
            header, payload = line.split('|', 1)
            _, session, sequence, stamp = header.split()
            sequence, stamp = int(sequence), float(stamp) / 1000.0
            if not math.isfinite(stamp) or not -0.05 <= received - stamp <= self.max_age:
                raise ValueError('stale packet or clocks not synchronized')
            stream = payload.split(':', 1)[0].split('|', 1)[0].strip().lower()
            if len(stream) > 256 or len(session) > 64 or sequence < 0:
                raise ValueError('invalid envelope')
            with self._lock:
                if session in self._retired: raise ValueError('retired session')
                old = self._sessions.get(stream)
                if old and old != session:
                    self._retired.add(old)
                self._sessions[stream] = session
                key = (session, stream)
                if sequence <= self._sequences.get(key, -1): raise ValueError('out of order')
                self._sequences[key] = sequence
                if len(self._sequences) > 256:
                    raise ValueError('too many sources; restart receiver')
            return payload, stamp
        except (ValueError, TypeError):
            self.rejected += 1
            return None
