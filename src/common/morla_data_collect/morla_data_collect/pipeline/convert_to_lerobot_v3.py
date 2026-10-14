"""LeRobot v2.1 数据集 → v3.0 数据集（现代 lerobot ACT 等策略直接可读）。

背景：本包已有的 convert_to_lerobot.py 产出 codebase_version=v2.1 数据集，专供
OpenPI pi0.5（其 pyproject 钉死的旧版 lerobot 0.1.0 只认 v2.1）。而现代 lerobot
（>=0.6，如 VLA/lerobot）的数据集 codebase_version 已是 v3.0：读取侧
check_version_compatibility 会对 v2.1 数据直接 raise BackwardCompatibilityError，
必须先升版。本模块就是把 convert_to_lerobot.py 的 v2.1 产物原样升版为 v3.0——

  v2.1                                  v3.0
  meta/{episodes,episodes_stats,tasks}.jsonl
                                        meta/tasks.parquet
                                        meta/episodes/chunk-000/file-000.parquet
  meta/stats.json                       meta/stats.json（原样沿用）
  meta/info.json(codebase v2.1)         meta/info.json(codebase v3.0)
  data/chunk-000/episode_*.parquet      data/chunk-000/file-000.parquet（同 chunk 合并）
  videos/chunk-000/{cam}/episode_*.mp4  videos/{cam}/chunk-000/file-000.mp4（同 chunk 串接）

布局/数值语义逐字段镜像 VLA/lerobot scripts/convert_dataset_v21_to_v30.py：
  * 非视频列 parquet 逐段拼接（分段阈值 data_files_size_in_mb，默认 100MB）；
  * 同相机 episode mp4 用 PyAV ffconcat 流拷贝串接（不重编码），episode 在合成
    文件里的 [from_timestamp, to_timestamp) 由各段实测时长累加（与官方一致）；
  * episodes 元数据 parquet 只写读取侧实际消费的列（官方还内嵌 stats/* 扁平列，
    load_episodes 一律 strip，这里省略以避开 HF datasets 的序列化细节）；
  * v3.0 的 tasks.parquet / info.json 关键字段（data_path、video_path、
    data_files_size_in_mb 等）照抄官方写出。

**源 v2.1 目录只读不改**：输出写进独立的 output_root（默认不覆盖已有目录，需
--overwrite 才会重建）。ACT 训练需要的数据就是 v3.0 数据集本身（现代 lerobot 的
ACT 与其它策略共用同一读取管线），转换完成后可用 conda lerobot 环境直接
LeRobotDataset(...). 加载训练。

用法：
  python3 -m morla_data_collect.pipeline.convert_to_lerobot_v3 \
      --v21-root <v2.1 数据集目录> --output <v3.0 数据集目录>
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import tempfile
from typing import Any, Iterable

V21 = "v2.1"
V30 = "v3.0"

CHUNK_SIZE = 1000
DEFAULT_DATA_FILE_SIZE_IN_MB = 100
DEFAULT_VIDEO_FILE_SIZE_IN_MB = 200

DATA_PATH_TEMPLATE = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
VIDEO_PATH_TEMPLATE = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
EPISODES_PATH_TEMPLATE = "meta/episodes/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
TASKS_PATH = "meta/tasks.parquet"
INFO_PATH = "meta/info.json"
STATS_PATH = "meta/stats.json"


# ---------------------------------------------------------------- 小工具


def _load_episode_indices(v21_root: str) -> list[int]:
    """读 v2.1 meta/episodes.jsonl → 升序 episode_index 清单。

    该清单是源数据集里"实际有数"的 episode 的唯一权威（convert_session 会把
    NaN/Inf 段跳过不写，因此不能按 info.total_episodes 或目录文件数猜）。
    数据/视频转换只处理清单内的段，多出来的文件 = 残留污染，直接报错。
    """
    ep_path = os.path.join(v21_root, "meta", "episodes.jsonl")
    if not os.path.exists(ep_path):
        raise FileNotFoundError(
            f"{ep_path} 缺失：v2.1 数据集必须带 episodes 清单"
            "（用 morla_data_collect.pipeline.convert_to_lerobot 生成）"
        )
    indices: list[int] = []
    with open(ep_path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            e = json.loads(line)
            indices.append(int(e["episode_index"]))
    if not indices:
        raise ValueError(f"{ep_path} 为空：源数据集没有任何 episode")
    indices.sort()
    return indices


def _episode_index_from(name: str) -> int | None:
    m = re.fullmatch(r"episode_(\d{6})\.[^.]+", os.path.basename(name))
    return int(m.group(1)) if m else None


def _check_episode_set(
    v21_root: str, kind: str, found: list[int], expected: list[int]
) -> None:
    """found（目录里扫到的 episode 下标）必须 == expected（episodes 清单）。

    残留文件（上次转换遗留、或外源补齐过）会让下游混入旧布局/旧数据行，
    静默跳过会掩盖数据损坏，这里宁可响亮失败。
    """
    if found == expected:
        return
    extra = sorted(set(found) - set(expected))
    missing = sorted(set(expected) - set(found))
    raise ValueError(
        f"{v21_root} 的 {kind} 与 meta/episodes.jsonl 不一致："
        f"清单 {len(expected)} 段，目录 {len(found)} 个文件"
        + (f"，多余(疑似上次残留): {extra}" if extra else "")
        + (f"，缺失: {missing}" if missing else "")
        + "。若刚改过采集配置/重跑过转换，请把 v2.1 输出目录换成全新目录"
        "或删除旧 data/videos 后再转。"
    )


def _load_info(v21_root: str) -> dict:
    path = os.path.join(v21_root, INFO_PATH)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} 不存在：不是 LeRobot v2.1 数据集目录")
    with open(path, encoding="utf-8") as f:
        info = json.load(f)
    if info.get("codebase_version") != V21:
        raise ValueError(
            f"源数据集 codebase_version={info.get('codebase_version')!r}，"
            f"本转换只接受 {V21}（先跑 convert_to_lerobot.py 生成 v2.1）"
        )
    return info


def _next_chunk_file(chunk: int, file: int) -> tuple[int, int]:
    """与上游 update_chunk_file_indices 一致：每 chunk 至多 CHUNK_SIZE 个文件。"""
    if file == CHUNK_SIZE - 1:
        return chunk + 1, 0
    return chunk, file + 1


def _concat_videos(paths: list[str], out_path: str) -> None:
    """PyAV ffconcat 流拷贝串接（不重编码），等价上游 concatenate_video_files。

    要求所有输入同 codec/分辨率/fps——本管线的 v2.1 mp4 由同一 encoder 配置产出，
    满足约束。用 av 而非 ffmpeg 二进制，避免外部依赖。
    """
    import av

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".ffconcat", delete=False) as tf:
        tf.write("ffconcat version 1.0\n")
        for p in paths:
            tf.write(f"file '{os.path.abspath(p)}'\n")
        tf.flush()
        concat_path = tf.name
    tmp_out = out_path + ".tmp.mp4"
    try:
        in_container = av.open(concat_path, mode="r", format="concat", options={"safe": "0"})
        try:
            out_container = av.open(tmp_out, mode="w", options={"movflags": "faststart"})
            try:
                stream_map: dict[int, Any] = {}
                for istream in in_container.streams:
                    if istream.type in ("video", "audio", "subtitle"):
                        ostream = out_container.add_stream_from_template(
                            template=istream, opaque=True
                        )
                        ostream.time_base = istream.time_base
                        stream_map[istream.index] = ostream
                for packet in in_container.demux():
                    if packet.stream.index not in stream_map or packet.dts is None:
                        continue
                    packet.stream = stream_map[packet.stream.index]
                    out_container.mux(packet)
            finally:
                out_container.close()
        finally:
            in_container.close()
    finally:
        os.unlink(concat_path)
        if os.path.exists(tmp_out):
            shutil.move(tmp_out, out_path)


def _video_duration_s(path: str) -> float:
    """与上游 get_video_duration_in_s 一致（PyAV）。"""
    import av

    with av.open(path) as container:
        vstream = container.streams.video[0]
        if vstream.duration is not None:
            return float(vstream.duration * vstream.time_base)
        return float(container.duration / av.time_base)


def _parquet_num_rows(path: str) -> int:
    import pyarrow.parquet as pq

    return pq.read_metadata(path).num_rows


# ---------------------------------------------------------------- 各段转换


def _convert_tasks(v21_root: str, out_root: str) -> None:
    import pandas as pd

    tasks_path = os.path.join(v21_root, "meta", "tasks.jsonl")
    entries = []
    if os.path.exists(tasks_path):
        with open(tasks_path, encoding="utf-8") as f:
            entries = [json.loads(line) for line in f if line.strip()]
    entries.sort(key=lambda e: int(e["task_index"]))
    tasks = [str(e["task"]) for e in entries]
    task_indices = [int(e["task_index"]) for e in entries]
    df = pd.DataFrame({"task_index": task_indices}, index=pd.Index(tasks, name="task"))
    out_path = os.path.join(out_root, TASKS_PATH)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    df.to_parquet(out_path)


def _convert_data(
    v21_root: str,
    out_root: str,
    data_file_size_in_mb: int,
    episode_indices: list[int],
    log=print,
) -> list[dict]:
    """v2.1 每段一个 parquet → v3.0 同 chunk 合并分段 file-*.parquet。

    返回每段的 episodes 数据元数据（episode_index/data 下标/dataset 全局帧区间）。
    文件集以 meta/episodes.jsonl 为权威：只处理清单内下标，残留文件直接报错。
    """
    import pandas as pd
    import pyarrow as pa
    import pyarrow.parquet as pq

    ep_paths = sorted(
        p for p in _walk(v21_root, "data") if os.path.basename(p).endswith(".parquet")
    )
    found: list[int] = []
    for p in ep_paths:
        idx = _episode_index_from(p)
        if idx is None:
            raise ValueError(
                f"{v21_root} 的 data 目录含非 episode 命名文件 {os.path.basename(p)}"
                "（疑似混入其它版本产物），请换全新输出目录后重转"
            )
        found.append(idx)
    found.sort()
    _check_episode_set(v21_root, "data", found, episode_indices)
    ep_metadata: list[dict] = []
    chunk_idx = file_idx = 0
    acc_mb = 0.0
    pending: list[str] = []
    dataset_frames = 0

    def _flush() -> None:
        nonlocal chunk_idx, file_idx, acc_mb
        if not pending:
            return
        rel = DATA_PATH_TEMPLATE.format(chunk_index=chunk_idx, file_index=file_idx)
        out_path = os.path.join(out_root, rel)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        frames = [pd.read_parquet(p) for p in pending]
        table = pa.Table.from_pandas(pd.concat(frames, ignore_index=True))
        pq.write_table(table, out_path)
        if file_idx == 0:
            log(f"  data chunk-{chunk_idx:03d}/file-{file_idx:03d}: {len(pending)} 段")
        pending.clear()
        chunk_idx, file_idx = _next_chunk_file(chunk_idx, file_idx)
        acc_mb = 0.0

    ep_paths = sorted(p for p in ep_paths if _episode_index_from(p) in set(episode_indices))
    for path in ep_paths:
        ep_idx = _episode_index_from(path)
        n_rows = _parquet_num_rows(path)
        size_mb = _parquet_size_mb(path)
        if acc_mb + size_mb >= data_file_size_in_mb and pending:
            _flush()
        ep_metadata.append(
            {
                "episode_index": ep_idx,
                "data/chunk_index": chunk_idx,
                "data/file_index": file_idx,
                "dataset_from_index": dataset_frames,
                "dataset_to_index": dataset_frames + n_rows,
            }
        )
        pending.append(path)
        dataset_frames += n_rows
        acc_mb += size_mb
    _flush()
    return ep_metadata


def _walk(root: str, subdir: str) -> Iterable[str]:
    base = os.path.join(root, subdir)
    if not os.path.isdir(base):
        return
    for dirpath, _dirnames, filenames in os.walk(base):
        for name in filenames:
            yield os.path.join(dirpath, name)


def _parquet_size_mb(path: str) -> float:
    import pyarrow.parquet as pq

    md = pq.read_metadata(path)
    total = 0
    for rg in range(md.num_row_groups):
        rg_meta = md.row_group(rg)
        for col in range(rg_meta.num_columns):
            total += rg_meta.column(col).total_uncompressed_size
    return total / (1024**2)


def _convert_videos_of_camera(
    v21_root: str,
    out_root: str,
    video_key: str,
    video_file_size_in_mb: int,
    episode_indices: list[int],
) -> list[dict]:
    """把某相机所有 episode mp4 按阈值分段串接为 videos/{key}/chunk/file.mp4。

    返回每段该相机视频元数据（chunk/file 下标 + from/to_timestamp）。文件集同样
    以 meta/episodes.jsonl 为权威（残留旧段 mp4 会污染时间窗，直接报错）。
    """
    # v2.1 布局：videos/chunk-{c:03d}/{video_key}/episode_*.mp4 —— 相机是文件父目录名
    ep_paths = [
        p
        for p in _walk(v21_root, "videos")
        if os.path.basename(p).endswith(".mp4")
        and os.path.basename(os.path.dirname(p)) == video_key
    ]
    found: list[int] = []
    for p in ep_paths:
        idx = _episode_index_from(p)
        if idx is None:
            raise ValueError(
                f"{v21_root} 的 videos/{video_key} 含非 episode 命名文件 "
                f"{os.path.basename(p)}，疑似混入其它版本产物"
            )
        found.append(idx)
    found.sort()
    _check_episode_set(v21_root, f"videos/{video_key}", found, episode_indices)
    ep_paths.sort()
    cam_meta: list[dict] = []
    chunk_idx = file_idx = 0
    acc_mb = 0.0
    acc_duration = 0.0
    pending: list[str] = []

    def _flush() -> None:
        nonlocal chunk_idx, file_idx, acc_mb, acc_duration
        if not pending:
            return
        rel = VIDEO_PATH_TEMPLATE.format(
            video_key=video_key, chunk_index=chunk_idx, file_index=file_idx
        )
        out_path = os.path.join(out_root, rel)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        _concat_videos(pending, out_path)
        pending.clear()
        chunk_idx, file_idx = _next_chunk_file(chunk_idx, file_idx)
        acc_mb = 0.0
        acc_duration = 0.0

    for path in ep_paths:
        duration = _video_duration_s(path)
        size_mb = os.path.getsize(path) / (1024**2)
        if acc_mb + size_mb >= video_file_size_in_mb and pending:
            _flush()
        cam_meta.append(
            {
                f"videos/{video_key}/chunk_index": chunk_idx,
                f"videos/{video_key}/file_index": file_idx,
                f"videos/{video_key}/from_timestamp": acc_duration,
                f"videos/{video_key}/to_timestamp": acc_duration + duration,
            }
        )
        pending.append(path)
        acc_mb += size_mb
        acc_duration += duration
    _flush()
    return cam_meta


def _convert_info(v21_root: str, out_root: str, info: dict) -> None:
    out = dict(info)
    out["codebase_version"] = V30
    out.pop("total_chunks", None)
    out.pop("total_videos", None)
    out["data_files_size_in_mb"] = DEFAULT_DATA_FILE_SIZE_IN_MB
    out["video_files_size_in_mb"] = DEFAULT_VIDEO_FILE_SIZE_IN_MB
    out["data_path"] = DATA_PATH_TEMPLATE
    out["video_path"] = (
        VIDEO_PATH_TEMPLATE if out.get("video_path") is not None else None
    )
    out["fps"] = int(out["fps"])
    for key, ft in out["features"].items():
        if ft.get("dtype") == "video":
            continue
        ft["fps"] = out["fps"]
    out_path = os.path.join(out_root, INFO_PATH)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)


# ---------------------------------------------------------------- 主入口


def convert_dataset_v21_to_v30(v21_root: str, output_root: str, *, overwrite: bool = False,
                               log=print) -> dict[str, Any]:
    from morla_data_collect.pipeline.artifacts import atomic_directory, reject_overlap
    v21_root = os.path.abspath(os.path.expanduser(v21_root))
    output_root = os.path.abspath(os.path.expanduser(output_root))
    reject_overlap(v21_root, output_root)
    if os.path.exists(output_root) and not overwrite:
        raise FileExistsError(f"output exists: {output_root}")
    with atomic_directory(output_root) as staged:
        summary = _convert_v30_impl(v21_root, staged, overwrite=True, log=log)
        for name in ("processing_manifest.json", "quality"):
            source = os.path.join(v21_root, "meta", name)
            if os.path.isdir(source):
                shutil.copytree(source, os.path.join(staged, "meta", name))
            elif os.path.isfile(source):
                shutil.copy2(source, os.path.join(staged, "meta", name))
    summary["output"] = output_root
    return summary


def _convert_v30_impl(
    v21_root: str,
    output_root: str,
    *,
    overwrite: bool = False,
    log=print,
) -> dict[str, Any]:
    """把 v2.1 数据集原地升版为 v3.0（源目录只读），产出可被现代 lerobot
    （>=0.6，含 ACT）直接加载的数据集。返回摘要 dict。"""
    v21_root = os.path.abspath(os.path.expanduser(v21_root))
    output_root = os.path.abspath(os.path.expanduser(output_root))
    if not os.path.isdir(v21_root):
        raise FileNotFoundError(f"v2.1 数据集目录不存在: {v21_root}")
    if v21_root == output_root or output_root.startswith(v21_root + os.sep):
        raise ValueError("输出目录不能是 v2.1 源目录或其子目录（源只读）")
    if os.path.exists(output_root):
        if not overwrite:
            raise FileExistsError(
                f"输出目录已存在: {output_root}（加 --overwrite 重建）"
            )
        shutil.rmtree(output_root)
    os.makedirs(output_root)

    info = _load_info(v21_root)
    features = info["features"]
    video_keys = sorted(
        key for key, ft in features.items() if ft.get("dtype") == "video"
    )
    episode_indices = _load_episode_indices(v21_root)

    log(f"升版 v2.1 → {V30}: {v21_root}")
    log(f"  {len(episode_indices)} episodes, cams={video_keys or '无'}")

    _convert_tasks(v21_root, output_root)
    log("  tasks -> meta/tasks.parquet")

    ep_data = _convert_data(
        v21_root, output_root, DEFAULT_DATA_FILE_SIZE_IN_MB,
        episode_indices, log=log,
    )

    cam_meta_list = []
    for vkey in video_keys:
        cam_meta = _convert_videos_of_camera(
            v21_root, output_root, vkey, DEFAULT_VIDEO_FILE_SIZE_IN_MB,
            episode_indices,
        )
        if len(cam_meta) != len(ep_data):
            raise RuntimeError(
                f"{vkey}: {len(cam_meta)} 段视频 ≠ {len(ep_data)} 段数据，"
                "v2.1 源数据集损坏或由别处补齐过"
            )
        cam_meta_list.append(cam_meta)
    if cam_meta_list:
        log("  videos -> 串接为 chunk/file.mp4")

    _write_episodes_meta(v21_root, output_root, ep_data, cam_meta_list)
    _copy_stats(v21_root, output_root)
    _convert_info(v21_root, output_root, info)

    total_frames = sum(
        e["dataset_to_index"] - e["dataset_from_index"] for e in ep_data
    )
    summary = {
        "source": v21_root,
        "output": output_root,
        "codebase_version": V30,
        "episodes": len(ep_data),
        "frames": total_frames,
        "video_keys": video_keys,
    }
    log(
        f"LeRobot {V30} dataset -> {output_root}\n"
        f"  {len(ep_data)} episodes / {total_frames} frames, "
        f"cams={len(video_keys)}"
    )
    return summary


def _write_episodes_meta(
    v21_root: str,
    out_root: str,
    ep_data: list[dict],
    cam_meta_list: list[list[dict]],
) -> None:
    """写 meta/episodes/chunk-000/file-000.parquet（每段一行）。

    列集 = 读取侧消费的元数据列（episode_index / data 下标 / dataset 全局区间 /
    各相机视频下标与时间窗 / tasks / length / meta/episodes 下标）。官方转换还会
    内嵌 stats/* 扁平列，load_episodes 读取时统一 select 掉，这里不生成以避免
    HF datasets 序列化细节泄漏到本包。
    """
    import pyarrow as pa
    import pyarrow.parquet as pq

    tasks_per_episode = _episode_tasks(v21_root)
    rows: list[dict] = []
    for i, ep in enumerate(ep_data):
        row: dict[str, Any] = {}
        row.update(ep)
        for cam_meta in cam_meta_list:
            row.update(cam_meta[i])
        # 原 v2.1 episodes.jsonl 有每段 tasks（list[str]）与 length
        row["tasks"] = tasks_per_episode.get(i, [])
        row["length"] = ep["dataset_to_index"] - ep["dataset_from_index"]
        row["meta/episodes/chunk_index"] = 0
        row["meta/episodes/file_index"] = 0
        rows.append(row)

    rel = EPISODES_PATH_TEMPLATE.format(chunk_index=0, file_index=0)
    out_path = os.path.join(out_root, rel)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    table = pa.Table.from_pylist(rows)
    pq.write_table(table, out_path, compression="snappy")


def _episode_tasks(v21_root: str) -> dict[int, list[str]]:
    """读 v2.1 meta/episodes.jsonl → {episode_index: [task, ...]}。

    v3.0 的 meta/episodes parquet 仍带每段 tasks 列（读取侧 episodes 抽样/过滤会
    用到），需要从 v2.1 的 episodes.jsonl 取回。返回空 dict 表示无该文件。
    """
    ep_path = os.path.join(v21_root, "meta", "episodes.jsonl")
    out: dict[int, list[str]] = {}
    if not os.path.exists(ep_path):
        return out
    with open(ep_path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            e = json.loads(line)
            out[int(e["episode_index"])] = [str(t) for t in e.get("tasks", [])]
    return out


def _copy_stats(v21_root: str, out_root: str) -> None:
    src = os.path.join(v21_root, STATS_PATH)
    dst = os.path.join(out_root, STATS_PATH)
    if not os.path.exists(src):
        raise FileNotFoundError(
            f"{src} 缺失：v2.1 数据集必须有聚合统计（转换器会写）"
        )
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(
        description="把 LeRobot v2.1 数据集升版为 v3.0（现代 lerobot ACT 直接可读）"
    )
    ap.add_argument("--v21-root", required=True, help="convert_to_lerobot.py 的 v2.1 输出目录")
    ap.add_argument("--output", required=True, help="v3.0 数据集输出目录（独立于源目录）")
    ap.add_argument(
        "--overwrite",
        action="store_true",
        help="输出目录已存在时删除重建（默认拒绝）",
    )
    args = ap.parse_args(argv)
    convert_dataset_v21_to_v30(
        args.v21_root, args.output, overwrite=args.overwrite
    )


if __name__ == "__main__":
    main(sys.argv[1:])
