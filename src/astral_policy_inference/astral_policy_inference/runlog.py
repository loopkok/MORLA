"""Per-run log directory helper for ``policy_inference.launch.py``.

Mirrors ``astral_arm_teleop.teleop_log.run_log_dir`` (the two packages have no
shared dependency, so the ~10-line helper is duplicated here): a launch-time run
root + optional event tag → a stamped directory ``{root}/{YYYYMMDD-HHMMSS}[_tag]``
that scopes one recording session's metrics / joint-stream JSONL files instead of
appending to a shared ``/tmp`` file that is lost on reboot.
"""

from __future__ import annotations

import os
import time


def run_log_dir(root: str, tag: str = "") -> str:
    """Create and return ``{root}/{stamp}[_tag]``; empty ``root`` → ``""``."""
    root = (root or "").strip()
    if not root:
        return ""
    tag = (tag or "").strip().replace("/", "_").replace(" ", "_")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    name = f"{stamp}_{tag}" if tag else stamp
    d = os.path.join(root, name)
    os.makedirs(d, exist_ok=True)
    return d
