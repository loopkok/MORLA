"""convert_to_act 专属测试：raw session → ACT 数据集（官方 v3 布局）的结构级自检金标准。

覆盖：完整链路跑通（对齐→v2.1→v3→自检）、结构级自检对 stats 缺失的负例、
深度自检源码可编译、输出目录存在时拒绝覆盖。
"""
from __future__ import annotations

import json
import os

import pytest

from astral_data_collect.pipeline.convert_to_act import (
    deep_check_source,
    run_pipeline,
    structural_check,
)
from astral_data_collect.schema import CollectSchema
from conftest import write_raw_episode


def _schema():
    return CollectSchema(
        arms=["left"], end_effector_left="gripper", end_effector_right="none",
        cameras=["base", "left_wrist", "right_wrist"], dataset_fps=30,
    )


def _make_raw_session(root: str, n: int = 3, duration: float = 3.0) -> None:
    os.makedirs(root, exist_ok=True)
    for i in range(n):
        write_raw_episode(
            os.path.join(root, f"episode{i:06d}"),
            schema=_schema(), duration_s=duration, task=f"task_{i}",
        )


def test_convert_act_pipeline_passes_structural_check(tmp_path):
    """raw session 全链路转 ACT 数据集：结构级自检 0 issue，产物布局正确。"""
    raw = str(tmp_path / "raw")
    _make_raw_session(raw)
    out = str(tmp_path / "act")

    summary = run_pipeline(out, session_dir=raw, image_size=64)

    assert summary["output"] == out
    # 官方 v3 布局关键件
    for p in (
        "meta/info.json", "meta/stats.json", "meta/tasks.parquet",
        "data/chunk-000/file-000.parquet",
        "videos/observation.images.base/chunk-000/file-000.mp4",
        "videos/observation.images.left_wrist/chunk-000/file-000.mp4",
        "videos/observation.images.right_wrist/chunk-000/file-000.mp4",
    ):
        assert os.path.exists(os.path.join(out, p)), f"缺 {p}"
    # 结构级自检通过
    assert structural_check(out) == []
    # 深度自检脚本本身可编译（真实执行需现代 lerobot 环境）
    compile(deep_check_source(), "<deep-check>", "exec")


def test_structural_check_rejects_missing_image_stats(tmp_path):
    """stats.json 缺图像统计 → 结构级自检必须报错（ACT VISUAL MEAN_STD 硬需求）。"""
    raw = str(tmp_path / "raw")
    _make_raw_session(raw, n=2)
    out = str(tmp_path / "act")
    run_pipeline(out, session_dir=raw, image_size=64)

    sp = os.path.join(out, "meta", "stats.json")
    with open(sp, encoding="utf-8") as f:
        stats = json.load(f)
    for k in list(stats):
        if k.startswith("observation.images"):
            del stats[k]
    with open(sp, "w", encoding="utf-8") as f:
        json.dump(stats, f)

    issues = structural_check(out)
    assert any("observation.images" in i and "MEAN_STD" in i for i in issues)


def test_structural_check_rejects_empty_tasks(tmp_path):
    """tasks.parquet 为空 → 结构级自检必须报错（ACT 需要任务文本）。"""
    import pyarrow as pa
    import pyarrow.parquet as pq

    raw = str(tmp_path / "raw")
    _make_raw_session(raw, n=2)
    out = str(tmp_path / "act")
    run_pipeline(out, session_dir=raw, image_size=64)

    tp = os.path.join(out, "meta", "tasks.parquet")
    empty = pa.table({"task_index": pa.array([], type=pa.int64()),
                      "task": pa.array([], type=pa.string())})
    pq.write_table(empty, tp)

    issues = structural_check(out)
    assert any("tasks.parquet" in i for i in issues)


def test_run_pipeline_rejects_existing_output_without_overwrite(tmp_path):
    """输出目录已存在且未 --overwrite → 拒绝（防止误盖已训数据集）。"""
    raw = str(tmp_path / "raw")
    _make_raw_session(raw, n=2)
    out = str(tmp_path / "act")
    os.makedirs(out)
    with pytest.raises(FileExistsError, match="--overwrite"):
        run_pipeline(out, session_dir=raw, image_size=64)


def test_run_pipeline_from_v21_root(tmp_path):
    """从已有 v2.1 直接升版（openpi/ACT 共享中间层）：跳过对齐/重编码，自检仍通过。"""
    raw = str(tmp_path / "raw")
    _make_raw_session(raw, n=2)
    v21 = str(tmp_path / "v21")
    out = str(tmp_path / "act")
    run_pipeline(out, session_dir=raw, v21_root=None, image_size=64,
                 keep_v21=v21)  # 先用 --session 出 v2.1 并保留
    assert os.path.isdir(os.path.join(v21, "meta"))
    # 清掉 v2.1 里的 aligned 产物不影响 v21-root 路径（它不碰 raw）
    out2 = str(tmp_path / "act2")
    summary = run_pipeline(out2, v21_root=v21)
    assert structural_check(out2) == []
    assert summary["source"] == v21


def test_run_pipeline_requires_exactly_one_source(tmp_path):
    with pytest.raises(ValueError, match="--session 或 --v21-root"):
        run_pipeline(str(tmp_path / "out"), image_size=64)
    with pytest.raises(ValueError, match="--session 或 --v21-root"):
        run_pipeline(str(tmp_path / "out"), session_dir="x", v21_root="y", image_size=64)


def test_native_mode_rejects_mismatched_camera_shapes(tmp_path):
    """原生模式（--image-size 0）下各相机分辨率不一致 → 提前报错并给出改用
    224/480/720 的指引（ACT 要求全部相机同 shape）。"""
    import h5py
    import numpy as np

    from conftest import make_jpeg

    raw = str(tmp_path / "raw")
    _make_raw_session(raw, n=2)
    # 把 left_wrist 的图像改成 32x32，base 保持 64x48 → 原生尺寸不一致
    for ep in os.listdir(raw):
        h5 = os.path.join(raw, ep, "camera_data.h5")
        with h5py.File(h5, "a") as f:
            ds = f["left_wrist"]["images"]
            small = np.frombuffer(make_jpeg(color=(90, 90, 90), w=32, h=32), dtype=np.uint8)
            for i in range(len(ds)):
                ds[i] = small
    with pytest.raises(ValueError, match="同 shape"):
        run_pipeline(str(tmp_path / "act"), session_dir=raw, image_size=0)


def test_image_size_480_end_to_end(tmp_path):
    """480 分辨率端到端：结构自检通过，产物视频确为 480×480。"""
    import av
    import glob

    raw = str(tmp_path / "raw")
    _make_raw_session(raw, n=2)
    out = str(tmp_path / "act480")
    run_pipeline(out, session_dir=raw, image_size=480)
    assert structural_check(out) == []
    v = sorted(glob.glob(os.path.join(out, "videos", "**", "*.mp4"), recursive=True))[0]
    with av.open(v) as c:
        sv = c.streams.video[0]
        assert (sv.codec_context.width, sv.codec_context.height) == (480, 480)
