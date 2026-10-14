"""runlog.run_log_dir：每次 launch 自动建 {root}/{stamp}[_tag]/ 运行目录。"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from astral_policy_inference.runlog import run_log_dir  # noqa: E402

_STAMP_RE = re.compile(r"^\d{8}-\d{6}(_[A-Za-z0-9_]+)?$")


def test_empty_root_disables() -> None:
    assert run_log_dir("") == ""
    assert run_log_dir("   ") == ""


def test_creates_stamped_dir(tmp_path) -> None:
    d = run_log_dir(str(tmp_path))
    assert os.path.isdir(d)
    assert os.path.dirname(d) == str(tmp_path)
    assert _STAMP_RE.match(os.path.basename(d))


def test_tag_appended_and_sanitized(tmp_path) -> None:
    d = run_log_dir(str(tmp_path), "pick/place test 7")
    assert os.path.isdir(d)
    assert os.path.basename(d).endswith("_pick_place_test_7")


def test_separate_calls_exist(tmp_path) -> None:
    a, b = run_log_dir(str(tmp_path)), run_log_dir(str(tmp_path))
    # 秒级戳：同一秒内可能同名，但目录都存在且互不影响
    assert os.path.isdir(a) and os.path.isdir(b)
