"""ROS-free command contract shared by all input sources."""
import json
import math
import re
import time
import uuid
from collections import OrderedDict

COMMANDS = frozenset({
    "teleop_start", "teleop_pause", "teleop_toggle", "release",
    "record_start", "record_stop", "record_pause", "record_resume", "record_toggle",
    "record_discard", "confirm_discard", "cancel",
})
PHRASES = {
    "开始跟随": "teleop_start", "开始遥操": "teleop_start",
    "暂停跟随": "teleop_pause", "暂停遥操": "teleop_pause",
    "开始采集": "record_start", "开始录制": "record_start",
    "结束采集": "record_stop", "停止采集": "record_stop", "结束保存": "record_stop",
    "暂停采集": "record_pause", "继续采集": "record_resume",
    "丢弃本段": "record_discard", "确认丢弃": "confirm_discard",
    "取消操作": "cancel", "交还控制": "release",
}


def voice_command(text, confidence, *, wake_word="机器人", min_confidence=0.75):
    try:
        confidence = float(confidence)
    except (TypeError, ValueError):
        return None
    if not isinstance(text, str) or not math.isfinite(confidence) or confidence < min_confidence:
        return None
    # Chinese ASR inserts spaces between words. Match the entire utterance;
    # substrings, negation, partial hypotheses and fuzzy matching cannot execute.
    compact = re.sub(r"[\s，。！？,.!?]", "", text)
    if not compact.startswith(wake_word):
        return None
    return PHRASES.get(compact[len(wake_word):])


def request(command, source, **extra):
    return dict(version=1, request_id=uuid.uuid4().hex, command=command,
                source=source, sent_at=time.time(), **extra)


def validate_request(text, *, now=None):
    row = json.loads(text)
    if not isinstance(row, dict) or row.get("version") != 1 or row.get("command") not in COMMANDS:
        raise ValueError("unsupported operator command/version")
    for key in ("request_id", "source"):
        if not isinstance(row.get(key), str) or not 1 <= len(row[key]) <= 128:
            raise ValueError(f"invalid {key}")
    sent = float(row["sent_at"])
    age = (time.time() if now is None else now) - sent
    if not math.isfinite(sent) or not -0.1 <= age <= 2.0:
        raise ValueError("operator request is stale or future-dated")
    return row


class SeenRequests:
    def __init__(self):
        self.seen = OrderedDict()
        self.voice = {}

    def accept(self, row, now):
        key = row["request_id"]
        if key in self.seen:
            return False
        self.seen[key] = now
        while len(self.seen) > 256:
            self.seen.popitem(last=False)
        # Both microphones may hear the same utterance. Never turn one spoken
        # instruction into two actions, including a stateful pedal-style toggle.
        if row["source"].startswith("voice_"):
            command = row["command"]
            previous = self.voice.get(command, -float("inf"))
            self.voice[command] = now
            if now - previous < 2.0:
                return False
        return True


class PedalEdges:
    """Linux EV_KEY values: 1=down, 0=up, 2=autorepeat (ignored)."""
    def __init__(self, bindings, long_press_s=1.5):
        self.bindings = bindings
        self.long_press_s = long_press_s
        self.down = {}
        self.released = set()
        self.last = {}

    def reset(self):
        self.down.clear()
        self.released.clear()

    def update(self, code, value, now):
        command = self.bindings.get(code)
        if not command or value == 2:
            return None
        if value == 0:
            pressed_at = self.down.pop(code, None)
            self.released.add(code)
            if command == "record_discard" and pressed_at is not None and now - pressed_at >= self.long_press_s:
                return command
        elif value == 1 and code in self.released and code not in self.down:
            self.down[code] = now
            if now - self.last.get(code, -float("inf")) < 0.3:
                return None
            self.last[code] = now
            if command != "record_discard":
                return command
        return None
