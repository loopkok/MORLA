"""Runtime push gate for the Quest 3 video streamer.

The configured camera set (and therefore the negotiated WebRTC track set) is
fixed at startup — changing it would require SDP renegotiation.  The gate
instead decides, per track and at frame granularity, whether a track sends
camera frames or 2 fps black frames ("muted": near-zero bandwidth; the Quest
app shrinks the panel to an edge chip — or hides it — on `track_visibility`,
and un-muting resumes instantly with no reconnect).

Control surface (handled by the ROS node, see streamer_node.py):
  * ``push_enabled`` master switch — false mutes every track.
  * ``active`` label set — ``None`` means "all configured cameras stream";
    otherwise only the listed labels stream.

Both the rclpy spin thread (service/topic callbacks) and the asyncio WebRTC
thread (frame reads) touch the gate, so all state is behind a lock.
"""

from __future__ import annotations

import json
import threading
from typing import Any


class StreamGate:
    """Thread-safe runtime gate: master push switch + per-label active set.

    ``on_change``: optional zero-arg callback invoked (outside the lock) after
    any state change. Used to push track visibility to the Quest app. Set by
    the service once the asyncio loop exists; may be None at startup.
    """

    def __init__(
        self,
        *,
        all_labels: list[str],
        push_enabled: bool = True,
        active: list[str] | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._all = list(all_labels)
        self._push = bool(push_enabled)
        # None = all configured cameras; otherwise only these labels stream.
        self._active: set[str] | None = set(active) if active else None
        self.on_change: Any = None

    def _notify(self) -> None:
        cb = self.on_change
        if cb is not None:
            try:
                cb()
            except Exception:
                pass

    def is_enabled(self, label: str) -> bool:
        with self._lock:
            return self._push and (self._active is None or label in self._active)

    def set_push(self, enabled: bool) -> None:
        with self._lock:
            self._push = bool(enabled)
        self._notify()

    def set_active(self, labels: list[str] | None) -> list[str]:
        """Restrict streaming to ``labels`` (unknown labels ignored).

        Empty / None resets to "all configured cameras".  Returns the
        effective active list (or all labels when unrestricted).
        """
        with self._lock:
            if not labels:
                self._active = None
            else:
                wanted = {str(x).strip() for x in labels if str(x).strip()}
                self._active = {x for x in wanted if x in self._all}
            # Compute inline: active_labels() would re-acquire the (non-reentrant) lock.
            effective = [x for x in self._all if self._active is None or x in self._active]
        self._notify()
        return effective

    def active_labels(self) -> list[str]:
        with self._lock:
            return [x for x in self._all if self._active is None or x in self._active]

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "push_enabled": self._push,
                "configured": list(self._all),
                "active": [x for x in self._all if self._active is None or x in self._active],
            }

    def snapshot_json(self) -> str:
        return json.dumps(self.snapshot(), ensure_ascii=False)
