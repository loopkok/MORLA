"""replay_rerun 冒烟：回放一段对齐 episode 到 .rrd 文件（headless，不开 viewer）。"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.pipeline.align_data import align_episode  # noqa: E402
from conftest import write_raw_episode  # noqa: E402


def test_replay_to_rrd(tmp_path):
    rr = pytest.importorskip("rerun")  # 无 rerun-sdk 时跳过（函数级，不影响收集）

    ep = write_raw_episode(str(tmp_path / "episode000000"), duration_s=1.0)
    assert align_episode(ep) is not None

    rrd = tmp_path / "replay.rrd"
    rr.init("astral_test_replay", spawn=False)
    rr.save(str(rrd))

    from astral_data_collect.pipeline.replay_rerun import log_episode

    log_episode(ep, log=lambda *a: None)

    import time

    for _ in range(50):  # rr.save 异步落盘
        if rrd.exists() and rrd.stat().st_size > 1024:
            break
        time.sleep(0.1)
    assert rrd.exists() and rrd.stat().st_size > 1024
