"""convert_to_lerobot_v3 单元测试：v2.1→v3.0 升版逐文件镜像官方转换语义。

格式真值：VLA/lerobot scripts/convert_dataset_v21_to_v30.py。冒烟验证过的等价
判据（目录布局 / tasks.parquet / 数据 parquet 行数与列 / 视频帧数与时间戳叠加 /
stats 原样拷贝 / 源目录只读）在此固化为回归。
"""

import glob
import json
import os
import shutil
import sys
import tempfile

import pytest
import pyarrow.parquet as pq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.pipeline.align_data import align_episode  # noqa: E402
from astral_data_collect.pipeline.convert_to_lerobot import convert_session  # noqa: E402
from astral_data_collect.pipeline.convert_to_lerobot_v3 import (  # noqa: E402
    convert_dataset_v21_to_v30,
)
from astral_data_collect.schema import CollectSchema  # noqa: E402
from conftest import write_raw_episode  # noqa: E402

SCHEMA = CollectSchema(
    end_effector_left="gripper", end_effector_right="gripper",
    cameras=["wrist_left", "wrist_right"], dataset_fps=30, hold_frames=0,
)


def _build_v21(tmp_path, n=3):
    """造一个 v2.1 源数据集（h264 视频），返回其目录。"""
    root = tmp_path / "session"
    tasks = ["pick up the cube", "put down the cube", "pick up the cube"]
    for i in range(n):
        write_raw_episode(
            str(root / f"episode00000{i}"),
            schema=SCHEMA, duration_s=1.0, task=tasks[i],
        )
    for i in range(n):
        assert align_episode(str(root / f"episode00000{i}")) is not None
    out = tmp_path / "lerobot_v21"
    convert_session(str(root), str(out), codec="h264",
                    image_size=None, log=lambda *a: None)
    return str(out)


def _upgrade(tmp_path):
    v21 = _build_v21(tmp_path)
    v30 = tmp_path / "lerobot_v30"
    convert_dataset_v21_to_v30(v21, str(v30), log=lambda *a: None)
    return v21, str(v30)


def _mp4_frame_count(path: str) -> int:
    import av

    with av.open(path) as c:
        return sum(1 for _ in c.decode(c.streams.video[0]))


def _v2_cams(v21: str) -> list[str]:
    with open(os.path.join(v21, "meta", "info.json")) as f:
        return sorted(
            k for k, ft in json.load(f)["features"].items()
            if ft.get("dtype") == "video"
        )


def test_upgrade_layout_and_info(tmp_path):
    v21, v30 = _upgrade(tmp_path)
    with open(os.path.join(v21, "meta", "info.json")) as f:
        info21 = json.load(f)
    with open(os.path.join(v30, "meta", "info.json")) as f:
        info = json.load(f)

    assert info["codebase_version"] == "v3.0"
    cams = _v2_cams(v21)
    for rel in [
        "meta/tasks.parquet", "meta/stats.json",
        "meta/episodes/chunk-000/file-000.parquet",
        "data/chunk-000/file-000.parquet",
    ] + [
        f"videos/{cam}/chunk-000/file-000.mp4" for cam in cams
    ]:
        assert os.path.exists(os.path.join(v30, rel)), rel

    # v2.1 专用字段清掉 / 换成 v3.0 模板
    assert "total_chunks" not in info and "total_videos" not in info
    assert info["data_path"] == "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
    assert info["video_path"] == (
        "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
    )
    assert info["data_files_size_in_mb"] == 100
    assert info["video_files_size_in_mb"] == 200
    assert isinstance(info["fps"], int)
    # 非视频 feature 补 fps、视频 feature 原样保留
    for key, ft in info["features"].items():
        if ft.get("dtype") == "video":
            assert key in info21["features"]
        else:
            assert ft["fps"] == info["fps"]
    # 源 v2.1 目录保持只读（未原地改写）
    assert os.path.exists(os.path.join(v21, "meta", "tasks.jsonl"))
    assert os.path.exists(os.path.join(v21, "meta", "info.json"))


def test_tasks_parquet(tmp_path):
    v21, v30 = _upgrade(tmp_path)
    with open(os.path.join(v21, "meta", "tasks.jsonl")) as f:
        tasks21 = sorted(
            (int(e["task_index"]), str(e["task"]))
            for e in (json.loads(line) for line in f if line.strip())
        )
    df = pq.read_table(os.path.join(v30, "meta", "tasks.parquet")).to_pydict()
    # 官方转换：按 task_index 升序，task 作索引（pyarrow 侧即 "task" 列）
    assert df["task_index"] == [t for t, _ in tasks21]
    assert df["task"] == [task for _, task in tasks21]


def test_data_parquet_rows_and_columns(tmp_path):
    v21, v30 = _upgrade(tmp_path)

    v2_rows = sum(
        pq.read_metadata(os.path.join(root, name)).num_rows
        for root, _dirs, files in os.walk(os.path.join(v21, "data"))
        for name in files if name.endswith(".parquet")
    )
    data = pq.read_table(os.path.join(v30, "data", "chunk-000", "file-000.parquet"))
    assert data.num_rows == v2_rows

    with open(os.path.join(v21, "meta", "info.json")) as f:
        v21_cols = {k for k, ft in json.load(f)["features"].items()
                    if ft.get("dtype") != "video"}
    v3_cols = set(data.column_names)
    # v3 数据 parquet 仍只含非视频列，且与 v2.1 一致（升版不丢列）
    assert v3_cols == v21_cols
    assert not any(c.startswith("observation.images.") for c in v3_cols)


def test_episodes_meta_rows(tmp_path):
    v21, v30 = _upgrade(tmp_path)
    with open(os.path.join(v21, "meta", "episodes.jsonl")) as f:
        tasks21 = {
            int(e["episode_index"]): e.get("tasks", [])
            for e in (json.loads(line) for line in f if line.strip())
        }

    eps = pq.read_table(
        os.path.join(v30, "meta", "episodes", "chunk-000", "file-000.parquet")
    ).to_pylist()
    assert len(eps) == len(tasks21)
    for row in eps:
        assert row["tasks"] == tasks21[row["episode_index"]]
        assert row["length"] == row["dataset_to_index"] - row["dataset_from_index"]
        assert row["meta/episodes/chunk_index"] == 0
        assert row["meta/episodes/file_index"] == 0
        for cam in _v2_cams(v21):
            assert f"videos/{cam}/file_index" in row
            assert (
                row[f"videos/{cam}/to_timestamp"]
                >= row[f"videos/{cam}/from_timestamp"]
            )

    # dataset 全局帧区间首尾相接、无空洞
    for a, b in zip(eps, eps[1:]):
        assert a["dataset_to_index"] == b["dataset_from_index"]
    assert eps[0]["dataset_from_index"] == 0


def test_videos_concat_frames_and_timestamps(tmp_path):
    v21, v30 = _upgrade(tmp_path)
    eps = pq.read_table(
        os.path.join(v30, "meta", "episodes", "chunk-000", "file-000.parquet")
    ).to_pylist()

    for cam in _v2_cams(v21):
        src_frames = sum(
            _mp4_frame_count(os.path.join(root, name))
            for root, _dirs, files in os.walk(os.path.join(v21, "videos"))
            for name in files
            if name.endswith(".mp4") and os.path.basename(root) == cam
        )
        merged = os.path.join(v30, "videos", cam, "chunk-000", "file-000.mp4")
        assert _mp4_frame_count(merged) == src_frames, cam

        import av

        with av.open(merged) as c:
            vstream = c.streams.video[0]
            dur = float(vstream.duration * vstream.time_base)
        acc = eps[-1][f"videos/{cam}/to_timestamp"]
        # 合成容器时长 vs 元数据时间窗叠加：容许 encoder 时间基量化误差
        assert abs(dur - acc) < 0.2, (cam, dur, acc)
        # 每段时间窗首尾相接
        for a, b in zip(eps, eps[1:]):
            assert (
                a[f"videos/{cam}/to_timestamp"]
                == b[f"videos/{cam}/from_timestamp"]
            )


def test_stats_copied_and_guards(tmp_path):
    v21, v30 = _upgrade(tmp_path)
    with open(os.path.join(v21, "meta", "stats.json"), "rb") as a, \
            open(os.path.join(v30, "meta", "stats.json"), "rb") as b:
        assert a.read() == b.read()

    # 输出已存在且不 overwrite -> 拒绝
    import pytest

    with pytest.raises(FileExistsError):
        convert_dataset_v21_to_v30(v21, v30, log=lambda *a: None)
    # overwrite 重建成功
    convert_dataset_v21_to_v30(v21, v30, overwrite=True, log=lambda *a: None)
    assert os.path.exists(os.path.join(v30, "meta", "info.json"))

    # 非 v2.1 源目录 -> 清晰报错
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(FileNotFoundError):
        convert_dataset_v21_to_v30(str(empty), str(tmp_path / "x"),
                                   log=lambda *a: None)

    # 输出不能是源目录或其子目录（源只读）
    with pytest.raises(ValueError):
        convert_dataset_v21_to_v30(v21, os.path.join(v21, "sub"),
                                   log=lambda *a: None)


def test_source_readonly_across_upgrades(tmp_path):
    v21, _ = _upgrade(tmp_path)
    before = sorted(
        os.path.relpath(os.path.join(r, n), v21)
        for r, _d, fs in os.walk(v21) for n in fs
    )
    with tempfile.TemporaryDirectory() as td:
        out2 = os.path.join(td, "out")
        convert_dataset_v21_to_v30(v21, out2, log=lambda *a: None)
    after = sorted(
        os.path.relpath(os.path.join(r, n), v21)
        for r, _d, fs in os.walk(v21) for n in fs
    )
    assert after == before


# --------------------------------------------------------------------------
# 改采集配置后重跑：旧 episode 文件不得残留污染
# --------------------------------------------------------------------------

_DUAL = CollectSchema(
    arms=["left", "right"], end_effector_left="gripper",
    end_effector_right="gripper",
    cameras=["wrist_left", "wrist_right"], dataset_fps=30,
)


def _align_session(root: str, schema, tasks: list[str]) -> None:
    for i, t in enumerate(tasks):
        write_raw_episode(
            os.path.join(root, f"episode{i:06d}"), schema=schema,
            duration_s=1.0, task=t,
        )
        assert align_episode(os.path.join(root, f"episode{i:06d}")) is not None


def test_reconvert_same_output_cleans_stale(tmp_path):
    """5 段单臂 → 同目录重转 3 段双臂：data/videos 无残留，v3 升版一致性不受影响。"""
    ses = tmp_path / "session"
    out = tmp_path / "ds"
    _align_session(str(ses), SCHEMA, ["a", "b", "c", "d", "e"])
    convert_session(str(ses), str(out), codec="h264", image_size=None,
                    log=lambda *a: None)
    # 换配置：双 gripper 双臂（16 维），且段数变少
    shutil.rmtree(str(ses))
    _align_session(str(ses), _DUAL, ["x", "y", "z"])
    convert_session(str(ses), str(out), codec="h264", image_size=None,
                    log=lambda *a: None)

    # v2.1 目录只留 3 段（旧 episode_3/4 被清掉，而非静默残留）
    data = sorted(os.path.basename(p)
                  for p in glob.glob(os.path.join(str(out), "data", "chunk-*", "*.parquet")))
    assert data == ["episode_000000.parquet", "episode_000001.parquet",
                    "episode_000002.parquet"], data
    for cam in ("observation.images.wrist_left", "observation.images.wrist_right"):
        vids = glob.glob(os.path.join(str(out), "videos", "chunk-*", cam, "*.mp4"))
        assert len(vids) == 3, (cam, len(vids))
    with open(os.path.join(str(out), "meta", "info.json")) as f:
        assert json.load(f)["features"]["observation.state"]["shape"] == [16]

    # v3 升版：3 段、16 维，与 v2.1 信息一致
    v3 = tmp_path / "ds_v3"
    summary = convert_dataset_v21_to_v30(str(out), str(v3), log=lambda *a: None)
    assert summary["episodes"] == 3
    eps = pq.read_table(
        os.path.join(str(v3), "meta", "episodes", "chunk-000", "file-000.parquet")
    ).to_pylist()
    assert [e["episode_index"] for e in eps] == [0, 1, 2]
    data_v3 = pq.read_table(
        os.path.join(str(v3), "data", "chunk-000", "file-000.parquet")
    )
    assert data_v3.num_rows == sum(
        pq.read_metadata(os.path.join(r, n)).num_rows
        for r, _d, fs in os.walk(str(out)) for n in fs
        if n.endswith(".parquet") and os.path.basename(n).startswith("episode_")
    )
    with open(os.path.join(str(v3), "meta", "info.json")) as f:
        assert json.load(f)["total_episodes"] == 3


def test_v3_rejects_dirty_source(tmp_path):
    """手工往 v2.1 数据/视频目录塞多余 episode 文件 → 升版必须响亮拒绝。"""
    out = tmp_path / "ds"
    _align_session(str(tmp_path / "session"), SCHEMA, ["a", "b", "c"])
    convert_session(str(tmp_path / "session"), str(out), codec="h264",
                    image_size=None, log=lambda *a: None)
    # 伪造残留：episode_3 的数据与两相机视频各复制一份（episodes.jsonl 无此段）
    for src, dst in [
        (os.path.join(str(out), "data", "chunk-000", "episode_000002.parquet"),
         os.path.join(str(out), "data", "chunk-000", "episode_000003.parquet")),
    ]:
        shutil.copy2(src, dst)
    for cam in ("observation.images.wrist_left", "observation.images.wrist_right"):
        shutil.copy2(
            os.path.join(str(out), "videos", "chunk-000", cam, "episode_000002.mp4"),
            os.path.join(str(out), "videos", "chunk-000", cam, "episode_000003.mp4"),
        )
    with pytest.raises(ValueError, match="残留"):
        convert_dataset_v21_to_v30(str(out), str(tmp_path / "v3_bad"),
                                   log=lambda *a: None)
