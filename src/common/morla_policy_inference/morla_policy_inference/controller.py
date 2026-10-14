"""Mode state machine for policy / human / playback with pause & takeover.

Pure FSM (no ROS). States:

    IDLE  POLICY  POLICY_PAUSED  HUMAN  PLAYBACK  PLAYBACK_PAUSED

* ``pause``   POLICY|PLAYBACK → *_PAUSED (executor holds last target).
* ``resume``  *_PAUSED → active (executor starts moving again).
* ``takeover`` → HUMAN. Only the *node* decides whether the VR re-anchor
  actually succeeded; on failure the node calls :meth:`revert` to restore the
  previous state (the FSM itself has no I/O).
* ``release`` HUMAN → whichever activity was interrupted (or → IDLE if the
  node called :meth:`request` with ``resume=None``).
* ``policy`` / ``playback`` from IDLE|HUMAN start that activity.
* ``stop`` → IDLE from anywhere (drops queued context).

Each transition is validated by an explicit table so illegal command sequences
fail loudly instead of silently corrupting arbitration state.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar


class InvalidTransition(RuntimeError):
    pass


@dataclass
class Controller:
    _state: str = "IDLE"
    _activity: str | None = None        # "policy" | "playback"
    _paused: bool = False
    _prev_snapshot: tuple | None = field(default=None)
    _interrupted: tuple | None = field(default=None)
    # (state, activity, paused) captured right before a takeover; release()
    # restores it. _prev_snapshot additionally carries the _interrupted of the
    # moment before the last verb so a failed request (revert) does not lose
    # the "what did takeover interrupt" context.

    # allowed source states per verb
    _TABLE: ClassVar[dict[str, set[str]]] = {
        "policy": {"IDLE", "HUMAN", "POLICY_PAUSED", "PLAYBACK_PAUSED", "PLAYBACK"},
        "playback": {"IDLE", "HUMAN", "POLICY_PAUSED", "PLAYBACK_PAUSED", "POLICY"},
        "pause": {"POLICY", "PLAYBACK"},
        "resume": {"POLICY_PAUSED", "PLAYBACK_PAUSED"},
        "takeover": {"POLICY", "POLICY_PAUSED", "PLAYBACK", "PLAYBACK_PAUSED"},
        "release": {"HUMAN"},
        "stop": {"IDLE", "POLICY", "POLICY_PAUSED", "HUMAN", "PLAYBACK", "PLAYBACK_PAUSED"},
    }

    @property
    def state(self) -> str:
        return self._state

    @property
    def activity(self) -> str | None:
        return self._activity

    def _label(self, activity: str | None, paused: bool) -> str:
        if activity is None:
            return "IDLE"
        if activity == "human":
            return "HUMAN"
        name = activity.upper()
        return f"{name}_PAUSED" if paused else name

    def request(self, verb: str) -> str:
        """Validate + apply a command, returning the new state string."""
        allowed = self._TABLE.get(verb)
        if allowed is None:
            raise InvalidTransition(f"unknown verb {verb!r}")
        if self._state not in allowed:
            raise InvalidTransition(
                f"{verb!r} not allowed from state {self._state}"
            )
        self._prev_snapshot = (
            self._state, self._activity, self._paused, self._interrupted,
        )
        if verb == "stop":
            self._state, self._activity, self._paused = "IDLE", None, False
            self._interrupted = None
            return self._state
        if verb == "pause":
            self._paused = True
        elif verb == "resume":
            self._paused = False
        elif verb == "takeover":
            self._interrupted = (self._state, self._activity, self._paused)
            self._activity = "human"
            self._paused = False
        elif verb == "release":
            # back to the activity that takeover interrupted (keeps pause flag)
            snap = self._interrupted
            self._interrupted = None
            if snap is None:
                self._state, self._activity, self._paused = "IDLE", None, False
                return self._state
            _prev_state, prev_activity, prev_paused = snap
            if prev_activity is None:
                self._state, self._activity, self._paused = "IDLE", None, False
                return self._state
            self._activity = prev_activity
            self._paused = prev_paused
        elif verb in ("policy", "playback"):
            self._activity = verb
            self._paused = False
            self._interrupted = None
        self._state = self._label(self._activity, self._paused)
        return self._state

    def revert(self) -> str:
        """Roll back the last transition (failed takeover/policy start)."""
        if self._prev_snapshot is None:
            return self._state
        state, activity, paused, interrupted = self._prev_snapshot
        self._state, self._activity, self._paused = state, activity, paused
        self._interrupted = interrupted
        self._prev_snapshot = None
        return self._state

    def snapshot(self) -> dict:
        return {
            "state": self._state,
            "activity": self._activity,
            "paused": self._paused,
        }
