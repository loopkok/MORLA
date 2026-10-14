"""convert_to_lerobot 单元测试：v2.1 布局逐字段校验 + 视频/parquet 内容校验。

格式真值：VLA/lerobot @0cf86487（OpenPI pyproject 钉死的 lerobot 提交）。
"""

import json
import os
import sys

import h5py
import numpy as np
import pyarrow.parquet as pq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from morla_data_collect.pipeline.align_data import align_episode  # noqa: E402
from morla_data_collect.pipeline.convert_to_lerobot import (  # noqa: E402
    _letterbox,
    aggregate_stats,
    convert_session,
    get_video_info,
)
from morla_data_collect.schema import CollectSchema  # noqa: E402
from conftest import write_raw_episode  # noqa: E402

SCHEMA = CollectSchema(
    end_effector_left="gripper", end_effector_right="gripper",
    cameras=["wrist_left", "wrist_right"], dataset_fps=30, hold_frames=0,
)


def _build_dataset(tmp_path):
    root = tmp_path / "session"
    write_raw_episode(str(root / "episode000000"), schema=SCHEMA, duration_s=1.0,
                      task="pick up the cube")
    write_raw_episode(str(root / "episode000001"), schema=SCHEMA, duration_s=1.0,
                      task="pick up the cube")
    write_raw_episode(str(root / "episode000002"), schema=SCHEMA, duration_s=1.0,
                      task="put down the cube")
    for i in range(3):
        assert align_episode(str(root / f"episode00000{i}")) is not None
    out = tmp_path / "lerobot"
    # image_size=None：保留 64x48 原尺寸，让既有尺寸断言不受影响
    summary = convert_session(
        str(root), str(out), codec="h264", image_size=None, log=lambda *a: None
    )
    return str(out), summary


def test_directory_layout(tmp_path):
    out, summary = _build_dataset(tmp_path)
    assert summary["episodes"] == 3
    for rel in [
        "meta/info.json", "meta/tasks.jsonl", "meta/episodes.jsonl",
        "meta/episodes_stats.jsonl", "meta/stats.json",
        "data/chunk-000/episode_000000.parquet",
        "data/chunk-000/episode_000002.parquet",
        "videos/chunk-000/observation.images.wrist_left/episode_000000.mp4",
        "videos/chunk-000/observation.images.wrist_right/episode_000002.mp4",
    ]:
        assert os.path.exists(os.path.join(out, rel)), rel


def test_info_json_fields(tmp_path):
    out, _ = _build_dataset(tmp_path)
    with open(os.path.join(out, "meta/info.json")) as f:
        info = json.load(f)

    # 与 create_empty_dataset_info @0cf86487 同构
    assert info["codebase_version"] == "v2.1"
    assert info["robot_type"] == "astral_dual_arm"
    assert info["total_episodes"] == 3
    assert info["total_tasks"] == 2
    assert info["total_videos"] == 6
    assert info["total_chunks"] == 1
    assert info["chunks_size"] == 1000
    assert info["fps"] == 30
    assert info["splits"] == {"train": "0:3"}
    assert info["data_path"] == "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
    assert info["video_path"] == "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
    assert info["total_frames"] > 0

    feats = info["features"]
    assert feats["observation.state"] == {
        "dtype": "float32", "shape": [16], "names": feats["observation.state"]["names"],
    }
    assert len(feats["observation.state"]["names"]) == 16
    assert feats["action"]["shape"] == [16]
    for key, default in (
        ("timestamp", "float32"), ("frame_index", "int64"),
        ("episode_index", "int64"), ("index", "int64"), ("task_index", "int64"),
    ):
        assert feats[key]["dtype"] == default
        assert feats[key]["shape"] == [1]
    for cam in ("wrist_left", "wrist_right"):
        v = feats[f"observation.images.{cam}"]
        assert v["dtype"] == "video"
        assert v["shape"] == [48, 64, 3]
        assert v["names"] == ["height", "width", "channels"]
        assert v["info"]["video.fps"] == 30
        assert v["info"]["has_audio"] is False


def test_tasks_and_episodes_jsonl(tmp_path):
    out, _ = _build_dataset(tmp_path)
    tasks = [json.loads(l) for l in open(os.path.join(out, "meta/tasks.jsonl"))]
    assert tasks == [
        {"task_index": 0, "task": "pick up the cube"},
        {"task_index": 1, "task": "put down the cube"},
    ]
    episodes = [json.loads(l) for l in open(os.path.join(out, "meta/episodes.jsonl"))]
    assert [e["episode_index"] for e in episodes] == [0, 1, 2]
    assert all(set(e.keys()) == {"episode_index", "tasks", "length"} for e in episodes)
    assert episodes[2]["tasks"] == ["put down the cube"]


def test_parquet_schema_and_content(tmp_path):
    out, _ = _build_dataset(tmp_path)
    pf = pq.ParquetFile(os.path.join(out, "data/chunk-000/episode_000001.parquet"))
    names = pf.schema_arrow.names
    # 列集与 v2.1 一致：parquet 只含非视频列（视频帧由读取侧用 timestamp +
    # meta.video_path 模板解析；struct 列会让 openpi 锁定的旧版 lerobot 崩溃）
    for col in (
        "observation.state", "action", "timestamp", "frame_index",
        "episode_index", "index", "task_index",
    ):
        assert col in names, col
    # 回归：不得出现视频 struct 列
    assert not any(c.startswith("observation.images.") for c in names), names

    table = pf.read()
    n = table.num_rows
    assert n > 0
    # timestamp = i/fps 严格等距
    ts = table.column("timestamp").to_numpy()
    assert np.allclose(np.diff(ts), 1.0 / 30.0, atol=1e-6)
    assert ts[0] == np.float32(0.0)
    # frame_index 0..n-1；episode_index 恒为 1；task_index 映射正确
    assert table.column("frame_index").to_pylist() == list(range(n))
    assert set(table.column("episode_index").to_pylist()) == {1}
    assert set(table.column("task_index").to_pylist()) == {0}
    # state 维度与值
    state = np.array(table.column("observation.state").to_pylist(), dtype=np.float32)
    assert state.shape == (n, 16)
    assert np.all(np.isfinite(state))


def test_global_index_continuity(tmp_path):
    out, _ = _build_dataset(tmp_path)
    idx = []
    for i in range(3):
        t = pq.read_table(os.path.join(out, f"data/chunk-000/episode_00000{i}.parquet"))
        idx.extend(t.column("index").to_pylist())
    assert idx == list(range(len(idx)))  # 跨 episode 全局连续


def test_video_decodable_and_fps(tmp_path):
    import av

    out, _ = _build_dataset(tmp_path)
    mp4 = os.path.join(
        out, "videos/chunk-000/observation.images.wrist_left/episode_000000.mp4"
    )
    info = get_video_info(mp4)
    assert info["video.fps"] == 30
    assert info["video.width"] == 64
    assert info["video.height"] == 48
    assert info["has_audio"] is False

    with av.open(mp4) as container:
        frames = [f for f in container.decode(video=0)]
    assert len(frames) == 30  # 1s @ 30fps
    # 视频帧时间戳与 i/fps 对齐（check_timestamps_sync 容差 1e-4）
    for i, frame in enumerate(frames[:5]):
        t = float(frame.pts * frame.time_base)
        assert abs(t - i / 30.0) < 1e-4
    # 解码内容接近注入的纯色（wrist_left 注入 BGR(30,100,200) → RGB(200,100,30)；
    # JPEG+H264 双重有损 + yuv420 色度半分辨率，容差放宽）
    img = frames[0].to_ndarray(format="rgb24")
    assert img.shape == (48, 64, 3)
    assert abs(int(img[..., 0].mean()) - 200) < 45
    assert abs(int(img[..., 2].mean()) - 30) < 45


def test_episode_stats_and_aggregate(tmp_path):
    out, _ = _build_dataset(tmp_path)
    ep_stats = [json.loads(l) for l in open(os.path.join(out, "meta/episodes_stats.jsonl"))]
    assert len(ep_stats) == 3
    s0 = ep_stats[0]["stats"]
    for key in ("observation.state", "action"):
        assert set(s0[key].keys()) == {"min", "max", "mean", "std", "count"}
        # 与上游 get_feature_stats(keepdims=True) 一致：(1, D)
        assert np.asarray(s0[key]["mean"]).shape == (1, 16)
    img_stats = s0["observation.images.wrist_left"]
    assert np.asarray(img_stats["mean"]).shape == (3, 1, 1)
    assert 0.0 <= float(np.mean(img_stats["mean"])) <= 1.0  # /255 归一化

    with open(os.path.join(out, "meta/stats.json")) as f:
        gs = json.load(f)
    counts = [s["stats"]["observation.state"]["count"][0] for s in ep_stats]
    assert gs["observation.state"]["count"][0] == sum(counts)
    # 全局 mean 是 episode 加权 mean
    wmean = sum(
        np.asarray(s["stats"]["observation.state"]["mean"]) * s["stats"]["observation.state"]["count"][0]
        for s in ep_stats
    ) / sum(counts)
    assert np.allclose(gs["observation.state"]["mean"], wmean, atol=1e-6)


def test_image_stats_accumulator_matches_stacked():
    """流式累加器与 np.stack 后计算数值等价（OOM 修复的回归锚点）。"""
    from morla_data_collect.pipeline.convert_to_lerobot import (
        ImageStatsAccumulator,
        compute_episode_stats,
    )

    rng = np.random.default_rng(0)
    frames = rng.integers(0, 256, size=(17, 9, 13, 3), dtype=np.uint8)
    state = rng.normal(size=(17, 4)).astype(np.float32)
    action = rng.normal(size=(17, 4)).astype(np.float32)

    ref = compute_episode_stats(state, action, {"cam": frames})["cam"]
    acc = ImageStatsAccumulator()
    for i in range(len(frames)):
        acc.update(frames[i])
    got = compute_episode_stats(state, action, {"cam": acc.stats()})["cam"]

    assert got["count"][0] == ref["count"][0] == 17
    for k in ("min", "max", "mean", "std"):
        assert np.asarray(got[k]).shape == (3, 1, 1)
        assert np.allclose(got[k], ref[k], atol=1e-6), k


def test_image_stats_accumulator_empty():
    from morla_data_collect.pipeline.convert_to_lerobot import ImageStatsAccumulator

    assert ImageStatsAccumulator().stats() is None


def test_aggregate_stats_math():
    a = {"x": {"min": np.array([0.0]), "max": np.array([2.0]),
               "mean": np.array([1.0]), "std": np.array([0.5]), "count": np.array([4])}}
    b = {"x": {"min": np.array([1.0]), "max": np.array([5.0]),
               "mean": np.array([3.0]), "std": np.array([1.0]), "count": np.array([6])}}
    agg = aggregate_stats([a, b])["x"]
    assert agg["count"][0] == 10
    assert agg["min"][0] == 0.0 and agg["max"][0] == 5.0
    assert abs(agg["mean"][0] - 2.2) < 1e-9
    # 并行方差: (4*(0.25+1) + 6*(1+9))/10 - 2.2^2 = (5+60)/10 - 4.84 = 1.66
    assert abs(agg["std"][0] - np.sqrt(1.66)) < 1e-9


def test_missing_aligned_raises(tmp_path):
    root = tmp_path / "session"
    write_raw_episode(str(root / "episode000000"), schema=SCHEMA)
    import pytest

    with pytest.raises(RuntimeError):
        convert_session(str(root), str(tmp_path / "out"), log=lambda *a: None)


def test_convert_single_arm_layout(tmp_path):
    """单臂+wuji schema 的导出：state/action 维度跟随 schema。"""
    schema = CollectSchema(
        arms=["right"], end_effector_right="wuji",
        cameras=["wrist_right"], dataset_fps=30, hold_frames=0,
    )
    root = tmp_path / "session"
    write_raw_episode(str(root / "episode000000"), schema=schema, duration_s=1.0)
    assert align_episode(str(root / "episode000000")) is not None
    out = tmp_path / "lerobot"
    summary = convert_session(
        str(root), str(out), codec="h264", image_size=None, log=lambda *a: None
    )
    assert summary["state_dim"] == 27
    with open(os.path.join(str(out), "meta/info.json")) as f:
        info = json.load(f)
    assert info["features"]["observation.state"]["shape"] == [27]
    assert len(info["features"]["observation.state"]["names"]) == 27
    assert list(info["features"].keys()).count("observation.images.wrist_right") == 1
    assert "observation.images.wrist_left" not in info["features"]


def test_letterbox_matches_openpi_geometry():
    """letterbox 几何与 openpi resize_with_pad 一致：等比 + 对称黑边（余数归下/右）。"""
    # 1080p -> 224x126 + 上下各 49 黑边
    img = np.full((1080, 1920, 3), 200, dtype=np.uint8)
    out = _letterbox(img, 224)
    assert out.shape == (224, 224, 3)
    assert out[0].max() == 0 and out[-1].max() == 0  # 黑边
    assert out[49].max() == 200 and out[49 + 126 - 1].max() == 200
    # 竖高输入（200x100）-> 112x224 + 左右各 56 黑边
    tall = np.full((200, 100, 3), 128, dtype=np.uint8)
    out_tall = _letterbox(tall, 224)
    assert out_tall.shape == (224, 224, 3)
    assert out_tall[:, 0].max() == 0 and out_tall[:, -1].max() == 0
    assert out_tall[:, 56].max() == 128
    # 方形输入等比拉满，无黑边
    sq = np.full((100, 100, 3), 128, dtype=np.uint8)
    out_sq = _letterbox(sq, 224)
    assert out_sq.shape == (224, 224, 3)
    assert out_sq[0].max() == 128 and out_sq[-1].max() == 128
    # 已是目标尺寸：恒等
    done = np.full((224, 224, 3), 7, dtype=np.uint8)
    assert np.array_equal(_letterbox(done, 224), done)


def test_convert_letterbox_default_224(tmp_path):
    """默认 image_size=224：视频与 meta 均为 224x224，解码帧带黑边。"""
    root = tmp_path / "session"
    write_raw_episode(str(root / "episode000000"), schema=SCHEMA, duration_s=0.5,
                      task="t")
    assert align_episode(str(root / "episode000000")) is not None
    out = tmp_path / "lerobot"
    convert_session(str(root), str(out), codec="h264", log=lambda *a: None)
    with open(os.path.join(str(out), "meta/info.json")) as f:
        info = json.load(f)
    v = info["features"]["observation.images.wrist_left"]
    assert v["shape"] == [224, 224, 3], v["shape"]
    vinfo = get_video_info(
        os.path.join(
            str(out),
            "videos/chunk-000/observation.images.wrist_left/episode_000000.mp4",
        )
    )
    assert vinfo["video.width"] == 224 and vinfo["video.height"] == 224
    import cv2

    cap = cv2.VideoCapture(
        os.path.join(
            str(out),
            "videos/chunk-000/observation.images.wrist_left/episode_000000.mp4",
        )
    )
    ok, frame = cap.read()
    cap.release()
    assert ok and frame.shape == (224, 224, 3)
    # 64x48 源 -> 224x168 内容 + 上下 28px 黑边
    assert frame[0].max() == 0 and frame[-1].max() == 0
    assert frame[28:196].max() > 0


def test_svt_av1_params_env_override(monkeypatch):
    """ASTRAL_AV1_LP 环境变量控制 AV1 并行度：默认 2（内存红线），放开、非法回退。"""
    from morla_data_collect.pipeline.convert_to_lerobot import _svt_av1_params

    monkeypatch.delenv("ASTRAL_AV1_LP", raising=False)
    assert _svt_av1_params() == "lp=2:lookahead=16"   # 默认（内存红线）

    monkeypatch.setenv("ASTRAL_AV1_LP", "8")
    assert _svt_av1_params() == "lp=8:lookahead=16"   # 20 核机器放开

    for bad in ("abc", "-1", "0", "  "):
        monkeypatch.setenv("ASTRAL_AV1_LP", bad)
        assert _svt_av1_params() == "lp=2:lookahead=16"  # 非法回退默认
