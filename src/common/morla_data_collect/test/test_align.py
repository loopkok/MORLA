"""align_data 单元测试：网格、最近邻、action 语义、hold 帧。"""

import json
import os
import sys

import h5py
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from morla_data_collect.pipeline.align_data import (  # noqa: E402
    ALIGNED_H5,
    FLAG_STATE_GAP,
    align_episode,
    find_nearest_idx,
)
from morla_data_collect.schema import CollectSchema  # noqa: E402
from conftest import write_raw_episode  # noqa: E402


def test_find_nearest_idx():
    ts = np.array([0.0, 0.1, 0.2, 0.3])
    assert find_nearest_idx(ts, 0.0) == 0
    assert find_nearest_idx(ts, 0.149) == 1   # 0.149 距 0.1 (0.049) 比 0.2 (0.051) 近
    assert find_nearest_idx(ts, 0.151) == 2
    assert find_nearest_idx(ts, 0.999) == 3


def test_align_basic(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"), duration_s=2.0)
    stats = align_episode(ep)
    assert stats is not None
    assert stats["missing_blocks"] == []

    with h5py.File(os.path.join(ep, ALIGNED_H5), "r") as f:
        state = f["observation/state"][:]
        action = f["action"][:]
        ts = f["timestamps"][:]
        n = len(ts)
        fps = f.attrs["fps"]

        # 维度：双臂 14 + 双夹爪 2 = 16
        assert state.shape == (n, 16)
        assert action.shape == (n, 16)
        # 网格严格 1/fps 等距
        dt = np.diff(ts)
        assert np.allclose(dt, 1.0 / fps, atol=1e-9)
        # 无 NaN（所有流齐全）
        assert np.all(np.isfinite(state))
        # next_state 语义：action[i] == state[i+1]，末帧 hold
        assert np.allclose(action[:-1], state[1:], atol=1e-5)
        assert np.allclose(action[-1], state[-1], atol=1e-6)
        # 图像帧数与网格一致
        for cam in ("wrist_left", "wrist_right"):
            assert f[f"{cam}/images"].shape[0] == n


def test_align_hold_frames(tmp_path):
    schema = CollectSchema(cameras=["wrist_left", "wrist_right"], hold_frames=7)
    ep = write_raw_episode(
        str(tmp_path / "episode000000"), duration_s=2.0, schema=schema,
    )
    stats = align_episode(ep)
    assert stats is not None
    with h5py.File(os.path.join(ep, ALIGNED_H5), "r") as f:
        n = f["timestamps"].shape[0]
        state = f["observation/state"][:]
        # 基础帧数 ~2s*30+1（首尾包含），hold 帧追加在后
        assert n == stats["frames"]
        # 末 7 帧 state 全部等于 hold 前末帧
        assert np.allclose(state[-7:], state[-8], atol=1e-6)


def test_align_command_mode(tmp_path):
    schema = CollectSchema(
        cameras=["wrist_left", "wrist_right"], action_source="command",
    )
    ep = write_raw_episode(
        str(tmp_path / "episode000000"), duration_s=2.0, schema=schema,
    )
    stats = align_episode(ep)
    assert stats is not None
    with h5py.File(os.path.join(ep, ALIGNED_H5), "r") as f:
        state = f["observation/state"][:]
        action = f["action"][:]
        # Commands must come from the latest sample at or before the grid time.
        with h5py.File(os.path.join(ep, "robot_data.h5"), "r") as raw:
            stream = raw["streams/left_arm_cmd"]
            times = stream["timestamps"][:]
            grid = f["timestamps"][:]
            count = len(grid) - schema.hold_frames
            indices = np.clip(np.searchsorted(times, grid[:count], side="right") - 1,
                              0, len(times) - 1)
            np.testing.assert_array_equal(action[:count, :7], stream["values"][:][indices])
        assert not np.allclose(action[:-1], state[1:], atol=1e-6)


def test_align_gap_flagged(tmp_path):
    ep = write_raw_episode(
        str(tmp_path / "episode000000"), duration_s=2.0, gap_in="left_arm_state",
    )
    stats = align_episode(ep)
    assert stats is not None
    assert stats["state_gap_frames"] > 0
    with h5py.File(os.path.join(ep, ALIGNED_H5), "r") as f:
        quality = f["quality"][:]
        assert np.count_nonzero(quality & FLAG_STATE_GAP) > 0


def test_align_missing_stream_nan_and_flag(tmp_path):
    ep = write_raw_episode(
        str(tmp_path / "episode000000"), duration_s=2.0, drop_stream="right_arm_state",
    )
    stats = align_episode(ep)
    assert stats is not None
    assert "right_arm" in stats["missing_blocks"]
    with h5py.File(os.path.join(ep, ALIGNED_H5), "r") as f:
        state = f["observation/state"][:]
        # 缺失块 → 对应列 NaN
        assert np.all(np.isnan(state[:, 7:14]))
        assert np.all(np.isfinite(state[:, :7]))
        assert json.loads(f.attrs["missing_blocks"]) == ["right_arm"]


def test_align_idempotent_skip(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"))
    assert align_episode(ep) is not None
    assert align_episode(ep) is None  # 已存在，默认跳过
    assert align_episode(ep, force=True) is not None


def test_unsigned_aligned_cache_requires_explicit_rebuild(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"))
    align_episode(ep)
    with h5py.File(os.path.join(ep, ALIGNED_H5), "r+") as f:
        del f.attrs["raw_signature"]
    import pytest
    with pytest.raises(ValueError, match="no raw_signature"):
        align_episode(ep)
    assert align_episode(ep, force=True) is not None


def test_signed_aligned_cache_detects_changed_raw_source(tmp_path):
    ep = write_raw_episode(str(tmp_path / "episode000000"))
    align_episode(ep)
    with h5py.File(os.path.join(ep, "robot_data.h5"), "r+") as raw:
        raw["streams/left_arm_state/values"][0, 0] += 0.1
    import pytest
    with pytest.raises(ValueError, match="raw source changed"):
        align_episode(ep)
    assert align_episode(ep, force=True) is not None


def test_align_single_arm_schema(tmp_path):
    """单臂 schema 全链路：raw → aligned，布局与维度正确。"""
    schema = CollectSchema(
        arms=["right"], end_effector_right="wuji",
        cameras=["wrist_right"], dataset_fps=30,
    )
    ep = write_raw_episode(str(tmp_path / "episode000000"), duration_s=2.0,
                           schema=schema)
    stats = align_episode(ep)
    assert stats is not None
    with h5py.File(os.path.join(ep, ALIGNED_H5), "r") as f:
        # 右臂 7 + wuji 20 = 27
        assert f["observation/state"].shape[1] == 27
        assert f["action"].shape[1] == 27
        assert np.all(np.isfinite(f["observation/state"][:]))
        names = json.loads(f.attrs["state_names"])
        assert names[0] == "right_arm_0" and "right_ee_19" in names
        assert "wrist_right/images" in f
