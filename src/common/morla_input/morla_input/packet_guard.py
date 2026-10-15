"""Quest session, ordering, and freshness checks using monotonic clock sync."""

from collections import defaultdict, deque
import secrets
import threading
import time


class PacketGuard:
    """
    Validate Quest stream identity, order, and freshness.

    Quest and host wall clocks need not agree. The receiver estimates the
    offset between their monotonic clocks from nonce-protected four-timestamp
    challenge/response exchanges, then converts an accepted source timestamp
    back into the host wall-clock domain for existing ROS message stamps.
    """

    def __init__(self, max_age=0.5, allow_legacy=False, sync_interval_s=2.0):
        """Initialize freshness limits and synchronized-session state."""
        self.max_age = float(max_age)
        self.allow_legacy = bool(allow_legacy)
        self.sync_interval_ns = int(float(sync_interval_s) * 1e9)
        self._sequences = {}
        self._sessions = {}
        self._retired = set()
        self._active_session = None
        self._sync_samples = defaultdict(lambda: deque(maxlen=5))
        self._pending_sync = {}
        self._last_sync_challenge = {}
        # Keep the epoch-change edge local to each networking thread, as UDP
        # and each TCP peer may be processed concurrently.
        self._thread_state = threading.local()
        self._lock = threading.Lock()
        self.rejected = 0

    def make_sync_challenge(self, peer_key, now_ns=None, force=False):
        """Create a challenge for one TCP peer or UDP endpoint when due."""
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._lock:
            last = self._last_sync_challenge.get(peer_key)
            if not force and last is not None and now_ns - last < self.sync_interval_ns:
                return None
            nonce = secrets.token_hex(12)
            self._pending_sync[nonce] = (peer_key, now_ns)
            self._last_sync_challenge[peer_key] = now_ns
            # Bound stale challenges when peers disappear without responding.
            if len(self._pending_sync) > 256:
                cutoff = now_ns - 30_000_000_000
                self._pending_sync = {
                    key: value for key, value in self._pending_sync.items()
                    if value[1] >= cutoff
                }
            return f"@q3sync {nonce} {now_ns}\n"

    def handle_sync_ack(self, line, peer_key, received_ns=None):
        """Consume a challenge reply and update the recent clock mapping."""
        received_ns = time.monotonic_ns() if received_ns is None else int(received_ns)
        try:
            fields = line.strip().split()
            if len(fields) != 5 or fields[0] != "@q3sync_ack":
                raise ValueError("invalid sync response")
            _, session, nonce, quest_receive_us, quest_send_us = fields
            quest_receive_ns = int(quest_receive_us) * 1000
            quest_send_ns = int(quest_send_us) * 1000
            if not session or len(session) > 64 or quest_send_ns < quest_receive_ns:
                raise ValueError("invalid sync response values")
            with self._lock:
                pending = self._pending_sync.get(nonce)
                if pending is None or pending[0] != peer_key or session in self._retired:
                    raise ValueError("unknown/expired sync challenge")
                del self._pending_sync[nonce]
                sent_ns = pending[1]
                if received_ns < sent_ns:
                    raise ValueError("receiver monotonic clock moved backwards")
                network_rtt_ns = (received_ns - sent_ns) - (quest_send_ns - quest_receive_ns)
                if network_rtt_ns < 0 or network_rtt_ns > 2_000_000_000:
                    raise ValueError("sync round trip outside 0..2 seconds")
                # Offset is PC_monotonic - Quest_monotonic (four-timestamp NTP formula).
                offset_ns = ((sent_ns - quest_receive_ns) + (received_ns - quest_send_ns)) / 2.0
                samples = self._sync_samples[session]
                cutoff = received_ns - 10_000_000_000
                while samples and samples[0][3] < cutoff:
                    samples.popleft()
                samples.append((offset_ns, network_rtt_ns / 2.0, network_rtt_ns, received_ns))
                return True
        except (ValueError, TypeError, OverflowError):
            self.rejected += 1
            return False

    def is_synced(self, session, now_ns=None):
        """Return whether a session has a recent monotonic clock sample."""
        if not session:
            return False
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._lock:
            samples = self._sync_samples.get(session)
            if not samples:
                return False
            latest = max(sample[3] for sample in samples)
            return now_ns - latest <= 10_000_000_000

    @staticmethod
    def session_from_packet(line):
        """Extract the session ID from a new-format Quest packet."""
        if not line.startswith("@q3m "):
            return None
        try:
            return line.split("|", 1)[0].split()[1]
        except (IndexError, ValueError):
            return None

    def accept(self, line, received=None, received_monotonic=None):
        """
        Return payload and estimated host-wall source time, or ``None``.

        Explicit legacy mode accepts only unwrapped payload lines. It does not
        accept the retired ``@q3`` UTC-clock envelope, which would bypass the
        new synchronized freshness checks while appearing to be legacy data.
        """
        self._thread_state.session_change = None
        received_wall = time.time() if received is None else float(received)
        received_mono_ns = (
            time.monotonic_ns() if received_monotonic is None
            else int(float(received_monotonic) * 1e9)
        )
        if not line.startswith("@q3m "):
            if self.allow_legacy and not line.startswith("@q3 "):
                return line, received_wall
            self.rejected += 1
            return None
        try:
            header, payload = line.split("|", 1)
            _, session, sequence_text, source_us_text = header.split()
            sequence = int(sequence_text)
            source_us = int(source_us_text)
            if not session or len(session) > 64 or sequence < 0 or source_us < 0:
                raise ValueError("invalid envelope")
            stream = payload.split(":", 1)[0].split("|", 1)[0].strip().lower()
            if not stream or len(stream) > 256:
                raise ValueError("invalid stream")

            with self._lock:
                samples = self._sync_samples.get(session)
                if not samples or session in self._retired:
                    raise ValueError("Quest clock not synchronized")
                # The lowest-delay recent exchange minimizes offset error.
                offset_ns, uncertainty_ns, _, sample_ns = min(
                    samples, key=lambda sample: sample[2]
                )
                if received_mono_ns - sample_ns > 10_000_000_000:
                    raise ValueError("Quest clock synchronization expired")
                age_ns = received_mono_ns - (source_us * 1000 + offset_ns)
                # Reject conservatively: the packet must fit the age budget at
                # the upper edge of the clock-offset uncertainty.
                upper_age_ns = age_ns + uncertainty_ns
                if upper_age_ns < -50_000_000 or upper_age_ns > self.max_age * 1e9:
                    raise ValueError("stale/future packet")

                session_change = None
                if self._active_session is not None and self._active_session != session:
                    old_session = self._active_session
                    self._retired.add(old_session)
                    session_change = {
                        "old_session": old_session,
                        "new_session": session,
                        "stream": stream,
                    }
                    self._sequences.clear()
                self._active_session = session
                self._sessions[stream] = session
                key = (session, stream)
                if sequence <= self._sequences.get(key, -1):
                    raise ValueError("duplicate/out-of-order packet")
                self._sequences[key] = sequence
                if len(self._sequences) > 256:
                    raise ValueError("too many sources; restart receiver")

            self._thread_state.session_change = session_change
            estimated_age_s = max(0.0, age_ns / 1e9)
            return payload, received_wall - estimated_age_s
        except (ValueError, TypeError, OverflowError):
            self.rejected += 1
            return None

    @property
    def last_session_change(self):
        """Session transition for the most recent accept call on this thread."""
        return getattr(self._thread_state, "session_change", None)
