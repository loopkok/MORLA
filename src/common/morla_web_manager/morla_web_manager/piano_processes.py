"""Reap only a launch's tagged descendants, including reparented process groups.

ROS launch children and vendor SDKs may outlive their launch parent. A PGID is
not an ownership boundary. The random environment tag is inherited by children;
Linux PID start times prevent signalling a PID that has since been reused.
"""
from pathlib import Path
import os
import signal
import time

SCOPE_ENV = "MORLA_PIANO_PROCESS_SCOPE"


def owned_processes(scope):
    if not scope or not Path("/proc/self/stat").exists():
        return {}
    marker = (SCOPE_ENV + "=" + scope).encode()
    found = {}
    for path in Path("/proc").iterdir():
        if not path.name.isdigit() or int(path.name) == os.getpid():
            continue
        try:
            if path.stat().st_uid != os.getuid():
                continue
            fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
            if fields[0] == "Z":  # no code, device descriptors or DDS endpoints remain
                continue
            if marker in (path / "environ").read_bytes().split(b"\0"):
                found[int(path.name)] = fields[19]  # /proc stat field 22
        except (OSError, ValueError, IndexError):
            continue
    return found


def _signal(pid, start, sig):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        if fields[19] == start:
            os.kill(pid, sig)
    except (ProcessLookupError, FileNotFoundError):
        pass


def stop_owned(scope, leader_pid=None, *, grace=3.0, term=2.0, kill=1.0):
    """Return still-live owned PIDs. Never signal by executable/node name."""
    for sig, duration in ((signal.SIGINT, grace), (signal.SIGTERM, term), (signal.SIGKILL, kill)):
        members = owned_processes(scope)
        if not members:
            return []
        # First let launch forward SIGINT and reap children itself. If launch
        # has already died, signal the remaining tagged children directly.
        targets = {leader_pid: members[leader_pid]} if sig == signal.SIGINT and leader_pid in members else members
        for pid, start in targets.items():
            _signal(pid, start, sig)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if not owned_processes(scope):
                return []
            time.sleep(.05)
    return sorted(owned_processes(scope))
