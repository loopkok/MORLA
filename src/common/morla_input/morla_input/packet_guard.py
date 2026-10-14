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
        self._active_session = None
        # Set only when an already-established producer session changes. The
        # receiver consumes this edge to invalidate pose anchors and disarm.
        self._thread_state = threading.local()
        self._lock = threading.Lock()
        self.rejected = 0

    def accept(self, line, received=None):
        self._thread_state.session_change = None
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
            session_change = None
            with self._lock:
                if session in self._retired: raise ValueError('retired session')
                if self._active_session is not None and self._active_session != session:
                    self._retired.add(self._active_session)
                    session_change = {
                        'old_session': self._active_session,
                        'new_session': session,
                        'stream': stream,
                    }
                    self._sequences.clear()
                self._active_session = session
                self._sessions[stream] = session
                key = (session, stream)
                if sequence <= self._sequences.get(key, -1): raise ValueError('out of order')
                self._sequences[key] = sequence
                if len(self._sequences) > 256:
                    raise ValueError('too many sources; restart receiver')
            self._thread_state.session_change = session_change
            return payload, stamp
        except (ValueError, TypeError):
            self.rejected += 1
            return None

    @property
    def last_session_change(self):
        """Session transition for the most recent accept call on this thread."""
        return getattr(self._thread_state, 'session_change', None)
