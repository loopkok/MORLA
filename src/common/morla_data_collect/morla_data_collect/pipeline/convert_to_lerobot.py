"""aligned_data.h5 → LeRobot v2.1 数据集（OpenPI pi0.5 直接可读）。

不依赖 lerobot 包，纯 pyarrow + av 手写 v2.1 布局；格式真值逐字段对齐
VLA/lerobot @0cf86487（OpenPI pyproject 钉死的提交）：

  meta/info.json            create_empty_dataset_info() 同构
  meta/tasks.jsonl          {"task_index", "task"}
  meta/episodes.jsonl       {"episode_index", "tasks", "length"}
  meta/episodes_stats.jsonl {"episode_index", "stats": {key: {min,max,mean,std,count}}}
  meta/stats.json           全部 episode 的加权聚合
  data/chunk-000/episode_{i:06d}.parquet
  videos/chunk-000/{video_key}/episode_{i:06d}.mp4

parquet 不含视频列（v2.1 规范：视频帧由读取侧用 timestamp 列 + meta 的
video_path 模板解析；写 struct 列会让旧版 lerobot 加载崩溃）。

用法：
  python3 convert_to_lerobot.py --session ~/astral_data/pick_place \
      --output ~/astral_data/lerobot/pick_place [--robot-type astral_dual_arm]
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from fractions import Fraction
from typing import Any, Iterable

import h5py
import numpy as np

from morla_data_collect.pipeline.align_data import ALIGNED_H5
from morla_data_collect.schema import pipeline_schema_from_dict

CODEBASE_VERSION = "v2.1"
CHUNK_SIZE = 1000  # 与上游 DEFAULT_CHUNK_SIZE 一致
DATA_PATH_TEMPLATE = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
VIDEO_PATH_TEMPLATE = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"

_IMAGE_STAT_MAX_SAMPLES = 100  # 与上游 estimate_num_samples 上限一致


# ---------------------------------------------------------------- 视频编码


def _svt_av1_params() -> str:
    """SVT-AV1 编码参数：lp 并行度可经 ASTRAL_AV1_LP 环境变量放开（默认 2）。

    默认 lp=2 是内存红线（15GB 小内存机 3 段 1080p 曾峰值 RSS 6.3GB 被 OOM 杀，
    见 CLAUDE.md 不变式 6）；核数多的机器（如 20 核）可在内存充裕时
    `ASTRAL_AV1_LP=8` 加速。lookahead=16 限前瞻深度，保持不变。
    """
    raw = os.environ.get("ASTRAL_AV1_LP", "2").strip()
    try:
        lp = int(raw)
        if lp < 1:
            raise ValueError
    except ValueError:
        lp = 2
    if lp > 2:
        print(
            f"[convert] ASTRAL_AV1_LP={lp}：AV1 并行度已放开（默认 2 是内存红线），"
            f"预计峰值内存 ~{0.6 * lp:.0f}GB，请确保内存充裕",
            file=sys.stderr,
        )
    return f"lp={lp}:lookahead=16"


def encode_video(
    frames: Iterable[np.ndarray],
    out_path: str,
    *,
    fps: int,
    width: int,
    height: int,
    codec: str = "libsvtav1",
    crf: int = 30,
    gop: int = 2,
) -> str:
    """把 uint8 RGB 帧序列写成 mp4；返回实际使用的 codec。

    默认 libsvtav1（上游 v2.1 默认），不可用时回退 h264。
    """
    import av

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    def _open(codec_name: str, extra: dict[str, str] | None = None):
        container = av.open(out_path, "w")
        try:
            opts = {"crf": str(crf), "g": str(gop)}
            if extra:
                opts.update(extra)
            stream = container.add_stream(codec_name, rate=fps, options=opts)
        except Exception:
            container.close()  # 回退路径：不泄漏容器/残留文件句柄
            raise
        stream.pix_fmt = "yuv420p"
        stream.width = width - (width % 2)  # yuv420p 要求偶数
        stream.height = height - (height % 2)
        return container, stream

    # SVT-AV1 默认按核数并行 + 深 lookahead：1080p 时编码器内部缓冲可达
    # 数 GB（实测 15GB 小内存机转换 3 段 1080p 数据峰值 RSS 6.3GB 被 OOM
    # 杀掉）。lookahead=16 限前瞻深度、lp 默认 2 限并行度（内存 ~1GB 量级）；
    # 核数多的机器可 ASTRAL_AV1_LP=8 放开（见 _svt_av1_params）。
    # 参数名不被旧版 libsvtav1 包装识别时退化为无参数打开，再不行回退 h264。
    if codec == "libsvtav1":
        try:
            container, stream = _open(
                codec, {"svtav1-params": _svt_av1_params()}
            )
        except Exception:
            try:
                container, stream = _open(codec)
            except Exception:
                codec = "h264"
                container, stream = _open(codec)
    else:
        try:
            container, stream = _open(codec)
        except Exception:
            codec = "h264"
            container, stream = _open(codec)

    time_base = Fraction(1, fps)
    try:
        for i, img in enumerate(frames):
            frame = av.VideoFrame.from_ndarray(img, format="rgb24")
            frame.pts = i
            frame.time_base = time_base
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    finally:
        container.close()
    if not os.path.exists(out_path):
        raise OSError(f"video encode failed: {out_path}")
    return codec


def get_video_info(video_path: str) -> dict[str, Any]:
    """与上游 get_video_info 同构（写进 features[key].info）。"""
    import av

    info: dict[str, Any] = {}
    with av.open(video_path, "r") as f:
        stream = f.streams.video[0]
        info["video.height"] = int(stream.height)
        info["video.width"] = int(stream.width)
        info["video.codec"] = str(stream.codec.canonical_name or stream.codec.name)
        info["video.pix_fmt"] = str(stream.pix_fmt)
        info["video.is_depth_map"] = False
        info["video.fps"] = int(stream.base_rate)
        pix = str(stream.pix_fmt)
        info["video.channels"] = (
            1 if ("gray" in pix or "mono" in pix) else (4 if "a" in pix[-2:] else 3)
        )
        info["has_audio"] = len(f.streams.audio) > 0
    return info


# ---------------------------------------------------------------- 统计


def _feature_stats(arr: np.ndarray, axis, keepdims=True) -> dict[str, np.ndarray]:
    return {
        "min": np.min(arr, axis=axis, keepdims=keepdims),
        "max": np.max(arr, axis=axis, keepdims=keepdims),
        "mean": np.mean(arr, axis=axis, keepdims=keepdims),
        "std": np.std(arr, axis=axis, keepdims=keepdims),
        "count": np.array([arr.shape[0] if arr.ndim else 1]),
    }


def _to_jsonable(v: Any) -> Any:
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    return v


class ImageStatsAccumulator:
    """流式图像统计：输出与 compute_episode_stats 的图像分支同构，内存 O(1)。

    旧实现 np.stack(≤100 帧) 在 1080p 下 ≈620MB/相机、3 相机 >1GB，与
    SVT-AV1 编码器内部缓冲叠加后在小内存机上 OOM。这里只维护每通道
    sum/sumsq/min/max（float64 标量组），数值与堆叠后计算等价。
    """

    def __init__(self) -> None:
        self._frames = 0
        self._pixels = 0
        self._sum: np.ndarray | None = None
        self._sumsq: np.ndarray | None = None
        self._min: np.ndarray | None = None
        self._max: np.ndarray | None = None

    def update(self, img: np.ndarray) -> None:
        c = img.astype(np.float64).reshape(-1, 3) / 255.0
        s = c.sum(axis=0)
        sq = (c * c).sum(axis=0)
        mn = c.min(axis=0)
        mx = c.max(axis=0)
        if self._sum is None:
            self._sum, self._sumsq, self._min, self._max = s, sq, mn, mx
        else:
            self._sum += s
            self._sumsq += sq
            self._min = np.minimum(self._min, mn)
            self._max = np.maximum(self._max, mx)
        self._frames += 1
        self._pixels += c.shape[0]

    def stats(self) -> dict[str, np.ndarray] | None:
        if self._frames == 0:
            return None
        mean = self._sum / self._pixels
        var = self._sumsq / self._pixels - mean**2

        def _c(v: np.ndarray) -> np.ndarray:
            return np.asarray(v, dtype=np.float64).reshape(3, 1, 1)

        return {
            "min": _c(self._min),
            "max": _c(self._max),
            "mean": _c(mean),
            "std": _c(np.sqrt(np.clip(var, 0.0, None))),
            "count": np.array([self._frames]),  # 与上游一致：按帧计数（聚合权重）
        }


def compute_episode_stats(
    state: np.ndarray,
    action: np.ndarray,
    image_samples: dict[str, Any],  # cam → (N,H,W,C) uint8 数组，或预算好的 stats dict
) -> dict[str, dict]:
    """与上游 compute_episode_stats 同构：数值全量，图像抽样 /255。"""
    stats: dict[str, dict] = {
        "observation.state": _feature_stats(state, axis=0),
        "action": _feature_stats(action, axis=0),
    }
    for key, imgs in image_samples.items():
        if isinstance(imgs, dict):
            stats[key] = imgs  # ImageStatsAccumulator 预算结果
            continue
        if len(imgs) == 0:
            continue
        arr = imgs.astype(np.float32) / 255.0  # (N,H,W,C)
        arr = np.transpose(arr, (0, 3, 1, 2))  # (N,C,H,W)
        s = _feature_stats(arr, axis=(0, 2, 3), keepdims=True)
        stats[key] = {
            k: (v if k == "count" else np.squeeze(v, axis=0))  # (C,1,1)
            for k, v in s.items()
        }
    return stats


def aggregate_stats(all_stats: list[dict[str, dict]]) -> dict[str, dict]:
    """上游 aggregate_stats 同构：加权 mean、并行方差、min/max/count。"""
    if not all_stats:
        return {}
    keys = all_stats[0].keys()
    out: dict[str, dict] = {}
    for key in keys:
        stats_list = [s[key] for s in all_stats if key in s]
        if not stats_list:
            continue
        counts = np.concatenate(
            [np.asarray(s["count"], dtype=np.float64).reshape(-1) for s in stats_list]
        )
        total = counts.sum()
        means = np.stack([np.asarray(s["mean"], dtype=np.float64) for s in stats_list])
        stds = np.stack([np.asarray(s["std"], dtype=np.float64) for s in stats_list])
        w = counts.reshape((-1,) + (1,) * (means.ndim - 1))
        mean = (means * w).sum(axis=0) / total
        var = (((stds ** 2 + means ** 2) * w).sum(axis=0) / total) - mean ** 2
        out[key] = {
            "min": np.min(
                np.stack([np.asarray(s["min"], dtype=np.float64) for s in stats_list]),
                axis=0,
            ),
            "max": np.max(
                np.stack([np.asarray(s["max"], dtype=np.float64) for s in stats_list]),
                axis=0,
            ),
            "mean": mean,
            "std": np.sqrt(np.clip(var, 0.0, None)),
            "count": np.array([int(total)]),
        }
    return out


# ---------------------------------------------------------------- parquet


def write_episode_parquet(
    path: str,
    *,
    state: np.ndarray,
    action: np.ndarray,
    fps: int,
    episode_index: int,
    index_start: int,
    task_index: int,
    labels: dict | None = None,
) -> None:
    import pyarrow as pa
    import pyarrow.parquet as pq

    n = state.shape[0]
    columns: dict[str, Any] = {
        "observation.state": pa.array(
            [state[i].tolist() for i in range(n)], type=pa.list_(pa.float32())
        ),
        "action": pa.array(
            [action[i].tolist() for i in range(n)], type=pa.list_(pa.float32())
        ),
    }
    for key, values in (labels or {}).items():
        dtype = pa.bool_() if key != 'control_owner' else pa.int64()
        # LeRobot maps shape=[1] to a scalar feature. Writing [value] here
        # creates list<bool>/list<int64>, which v3's datasets.Features rejects.
        columns[key] = pa.array(values.tolist(), type=dtype)
    # 注意：parquet 不写视频 struct{path,timestamp} 列。v2.1 规范的 parquet 只含
    # 非视频列（视频帧由读取侧用 timestamp 列 + meta 里的 video_path 模板解析）；
    # 多余的 struct 列会让旧版 lerobot（openpi 锁定的 0.1.0）hf_transform_to_torch
    # 在 torch.tensor(dict) 处直接崩溃。
    columns["timestamp"] = pa.array(
        [np.float32(i / fps) for i in range(n)], type=pa.float32()
    )
    columns["frame_index"] = pa.array(np.arange(n, dtype=np.int64))
    columns["episode_index"] = pa.array(np.full(n, episode_index, dtype=np.int64))
    columns["index"] = pa.array(
        np.arange(index_start, index_start + n, dtype=np.int64)
    )
    columns["task_index"] = pa.array(np.full(n, task_index, dtype=np.int64))

    os.makedirs(os.path.dirname(path), exist_ok=True)
    pq.write_table(pa.table(columns), path)


# ---------------------------------------------------------------- 主流程


def _decode_jpeg(jpeg: np.ndarray) -> np.ndarray | None:
    import cv2

    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def _letterbox(img: np.ndarray, size: int) -> np.ndarray:
    """等比缩放 + 黑边补齐到 size×size（复刻 openpi resize_with_pad 的几何约定：
    ratio = max(w,h)/size、对称黑边、余数归下/右）。转换期就把它做掉，
    训练时 openpi 的 ResizeImages(224,224) 对已是 size×size 的输入是恒等操作，
    与全分辨率入库再在线 letterbox 的模型输入一致，同时省掉加载侧解码+缩放开销。
    """
    import cv2

    h, w = img.shape[:2]
    if (h, w) == (size, size):
        return img
    ratio = max(w / size, h / size)
    new_w, new_h = int(w / ratio), int(h / ratio)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    pad_h0 = (size - new_h) // 2
    pad_h1 = size - new_h - pad_h0
    pad_w0 = (size - new_w) // 2
    pad_w1 = size - new_w - pad_w0
    return cv2.copyMakeBorder(
        resized, pad_h0, pad_h1, pad_w0, pad_w1, cv2.BORDER_CONSTANT, value=0
    )


def convert_session(session_dir: str, output_dir: str, *, robot_type: str = "astral_dual_arm",
                    codec: str = "libsvtav1", image_size: int | None = 224,
                    model_capacity: int | None = None,
                    log=print) -> dict[str, Any]:
    from morla_data_collect.pipeline.artifacts import atomic_directory, reject_overlap
    from morla_data_collect.pipeline.validate_data import ensure_training_ready
    session_dir = os.path.abspath(os.path.expanduser(session_dir))
    output_dir = os.path.abspath(os.path.expanduser(output_dir))
    reject_overlap(session_dir, output_dir)
    reports = ensure_training_ready(session_dir)
    with atomic_directory(output_dir) as staged:
        summary = _convert_session_impl(session_dir, staged, robot_type=robot_type,
                                        codec=codec, image_size=image_size,
                                        model_capacity=model_capacity, log=log)
        with open(os.path.join(staged, "meta", "processing_manifest.json"), "w", encoding="utf-8") as f:
            json.dump({"version": "export_v2", "source": session_dir, "output": output_dir,
                       "summary": {**summary, "output": output_dir}, "validation": reports},
                      f, ensure_ascii=False, indent=2)
    summary["output"] = output_dir
    return summary


def _convert_session_impl(
    session_dir: str,
    output_dir: str,
    *,
    robot_type: str = "astral_dual_arm",
    codec: str = "libsvtav1",
    image_size: int | None = 224,
    model_capacity: int | None = None,
    log=print,
) -> dict[str, Any]:
    """image_size: 视频落盘前等比缩放+黑边补齐到该边长（默认 224，与 openpi
    ResizeImages 约定一致）；None 保留原始分辨率。"""
    """把 session 下所有已对齐 episode 导出为一个 LeRobot v2.1 数据集。"""
    import cv2  # noqa: F401  (JPEG 解码依赖)

    session_dir = os.path.abspath(os.path.expanduser(session_dir))
    output_dir = os.path.abspath(os.path.expanduser(output_dir))
    episode_dirs = sorted(
        os.path.join(session_dir, d)
        for d in os.listdir(session_dir)
        if d.startswith("episode")
        and os.path.isdir(os.path.join(session_dir, d))
        and os.path.exists(os.path.join(session_dir, d, ALIGNED_H5))
    )
    if not episode_dirs:
        raise RuntimeError(f"no aligned episodes under {session_dir} (run align_data first)")

    # 输出目录 = 一个数据集的专属目录：重跑（改 schema / 换 codec / 段数变少）
    # 必须清掉上次的 data/videos，否则旧 episode 文件残留——后续升版 v3.0 按
    # 目录遍历时会混入旧布局的行（见 convert_to_lerobot_v3 的严格校验）。
    # meta/*.jsonl 本就是"w"截断重写，无需单独清理。
    for _sub in ("data", "videos"):
        _old = os.path.join(output_dir, _sub)
        if os.path.isdir(_old):
            shutil.rmtree(_old)
    os.makedirs(os.path.join(output_dir, "meta"), exist_ok=True)

    # -- 第一遍：读 schema / task / 尺寸，建 tasks 表 -----------------------------
    schemas: list[CollectSchema] = []
    tasks: list[str] = []
    metas: list[dict] = []
    shapes: dict[str, tuple[int, int]] = {}
    for ep in episode_dirs:
        with open(os.path.join(ep, "meta.json"), encoding="utf-8") as f:
            meta = json.load(f)
        metas.append(meta)
        schemas.append(pipeline_schema_from_dict(meta["schema"]))
        task = str(meta.get("task", ""))
        if task not in tasks:
            tasks.append(task)
    schema = schemas[0]
    for s in schemas[1:]:
        from morla_data_collect.pipeline.validate_data import training_contract
        if training_contract(s) != training_contract(schema):
            raise RuntimeError("episode 间 schema 不一致，无法合并导出（分 session 重录）")
    fps = int(schema.dataset_fps)
    state_dim = schema.state_dim
    state_names = schema.state_names()
    if model_capacity is not None and state_dim > model_capacity:
        raise ValueError(
            f"layout state_dim={state_dim} exceeds explicitly selected model capacity={model_capacity}; "
            "generic LeRobot export supports this layout, but the selected model does not"
        )
    video_keys = [f"observation.images.{cam}" for cam in schema.cameras]

    # -- 第二遍：逐 episode 写 parquet + mp4 + meta -------------------------------
    tasks_f = open(os.path.join(output_dir, "meta", "tasks.jsonl"), "w", encoding="utf-8")
    episodes_f = open(os.path.join(output_dir, "meta", "episodes.jsonl"), "w", encoding="utf-8")
    ep_stats_f = open(os.path.join(output_dir, "meta", "episodes_stats.jsonl"), "w", encoding="utf-8")
    for ti, task in enumerate(tasks):
        tasks_f.write(json.dumps({"task_index": ti, "task": task}, ensure_ascii=False) + "\n")
    tasks_f.close()

    all_stats: list[dict[str, dict]] = []
    total_frames = 0
    video_info: dict[str, dict] = {}
    used_codec: str | None = None
    ep_index = 0  # LeRobot 要求 0..N-1 连续：跳过坏段时用独立计数器重编号

    for ep_dir, meta in zip(episode_dirs, metas):
        chunk = ep_index // CHUNK_SIZE
        task = str(meta.get("task", ""))
        task_index = tasks.index(task)

        with h5py.File(os.path.join(ep_dir, ALIGNED_H5), "r") as fa:
            state = fa["observation/state"][:]
            action = fa["action"][:]
            labels = {k: fa[k][:] if k in fa else np.zeros(len(state), dtype=np.int64 if k == "control_owner" else bool)
                      for k in ("intervention", "intervention_valid", "control_owner")}
            if not (np.all(np.isfinite(state)) and np.all(np.isfinite(action))):
                raise ValueError(f"{ep_dir}: state/action contains NaN/Inf")
            n = state.shape[0]

            if "sensors" in fa and len(fa["sensors"]):
                sensor_dir = os.path.join(output_dir, "meta", "sensors")
                os.makedirs(sensor_dir, exist_ok=True)
                sidecar_path = os.path.join(sensor_dir, f"episode_{ep_index:06d}.h5")
                with h5py.File(sidecar_path, "w") as sidecar:
                    sidecar.attrs["episode_index"] = ep_index
                    sidecar.attrs["sensor_export_policy"] = "sidecar_only; not a LeRobot policy feature"
                    sidecar.attrs["profile_id"] = str(meta.get("profile_id") or "legacy")
                    for key, source_group in fa["sensors"].items():
                        target_group = sidecar.create_group(key)
                        for attr, value in source_group.attrs.items():
                            target_group.attrs[attr] = value
                        for field, dataset in source_group.items():
                            target_group.create_dataset(
                                field, data=dataset[:],
                                compression="lzf" if field in {"force_raw", "temperature_raw"} else None,
                            )

            # -- 视频：JPEG → RGB 帧 → mp4 ------------------------------------------
            rel_paths: dict[str, str] = {}
            img_stats: dict[str, dict] = {}
            for cam, vkey in zip(schema.cameras, video_keys):
                rel = VIDEO_PATH_TEMPLATE.format(
                    episode_chunk=chunk, video_key=vkey, episode_index=ep_index
                )
                rel_paths[vkey] = rel
                mp4_path = os.path.join(output_dir, rel)

                jpegs = fa[f"{cam}/images"]
                first = _decode_jpeg(jpegs[0])
                if first is None:
                    raise RuntimeError(f"{ep_dir}: {cam} frame 0 undecodable")
                if image_size is not None:
                    first = _letterbox(first, image_size)
                h, w = first.shape[:2]
                shapes[vkey] = (h, w)

                def _frames(cam=cam, n=n, jpegs=jpegs):
                    last = None
                    for i in range(n):
                        img = _decode_jpeg(jpegs[i])
                        if img is None:
                            raise ValueError(f"{ep_dir}: {cam} frame {i} JPEG undecodable")
                        elif image_size is not None:
                            img = _letterbox(img, image_size)
                        last = img
                        yield img

                used = encode_video(
                    _frames(), mp4_path, fps=fps, width=w, height=h, codec=codec
                )
                used_codec = used

                # 图像统计抽样（均匀 ≤100 帧，剔除解码失败帧）——流式累加，
                # 不堆叠帧数组（1080p×100 帧 ≈620MB/相机，小内存机会 OOM）。
                # 统计口径与落盘一致：letterbox 后的帧。
                acc = ImageStatsAccumulator()
                step = max(1, n // _IMAGE_STAT_MAX_SAMPLES)
                for i in range(0, n, step):
                    img = _decode_jpeg(jpegs[i])
                    if img is not None:
                        if image_size is not None:
                            img = _letterbox(img, image_size)
                        acc.update(img)
                st = acc.stats()
                if st is not None:
                    img_stats[vkey] = st

            if ep_index == 0:
                for vkey in video_keys:
                    video_info[vkey] = get_video_info(
                        os.path.join(output_dir, rel_paths[vkey])
                    )

            # -- parquet --------------------------------------------------------------
            pq_rel = DATA_PATH_TEMPLATE.format(
                episode_chunk=chunk, episode_index=ep_index
            )
            write_episode_parquet(
                os.path.join(output_dir, pq_rel),
                state=state,
                action=action,
                fps=fps,
                episode_index=ep_index,
                index_start=total_frames,
                task_index=task_index,
                labels=labels,
            )

            quality_report = {"source_episode": os.path.abspath(ep_dir),
                              "frames": int(n), "flagged_frames": int(np.count_nonzero(fa["quality"][:] & 3)),
                              "synthetic_hold_frames": int(np.count_nonzero(fa["quality"][:] & 4)),
                              "camera_repeated_source_frames": {}, "visual_samples": {}}
            for cam in schema.cameras:
                camera_ts = fa[f"{cam}/src_timestamps"][:]
                quality_report["camera_repeated_source_frames"][cam] = int(np.count_nonzero(np.diff(camera_ts) == 0))
                samples = []
                for i in range(0, n, max(1, n // 100)):
                    image = _decode_jpeg(fa[f"{cam}/images"][i])
                    if image_size is not None:
                        image = _letterbox(image, image_size)
                    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                    samples.append((float(gray.mean()), float(cv2.Laplacian(gray, cv2.CV_64F).var())))
                quality_report["visual_samples"][cam] = {"count": len(samples),
                    "mean_brightness": float(np.mean([v[0] for v in samples])),
                    "median_laplacian_variance": float(np.median([v[1] for v in samples]))}
            os.makedirs(os.path.join(output_dir, "meta", "quality"), exist_ok=True)
            with open(os.path.join(output_dir, "meta", "quality", f"episode_{ep_index:06d}.json"), "w") as f:
                json.dump(quality_report, f, indent=2)
            ep_stats = compute_episode_stats(state, action, img_stats)
            all_stats.append(ep_stats)
            episodes_f.write(
                json.dumps(
                    {"episode_index": ep_index, "tasks": [task], "length": int(n)},
                    ensure_ascii=False,
                )
                + "\n"
            )
            ep_stats_f.write(
                json.dumps(
                    {
                        "episode_index": ep_index,
                        "stats": {
                            k: {kk: _to_jsonable(vv) for kk, vv in v.items()}
                            for k, v in ep_stats.items()
                        },
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            total_frames += n
            log(f"  episode {ep_index:06d}: {n} frames task={task!r}")
            ep_index += 1

    episodes_f.close()
    ep_stats_f.close()

    # -- stats.json（全局聚合） ------------------------------------------------------
    global_stats = aggregate_stats(all_stats)
    with open(os.path.join(output_dir, "meta", "stats.json"), "w", encoding="utf-8") as f:
        json.dump(
            {k: {kk: _to_jsonable(vv) for kk, vv in v.items()} for k, v in global_stats.items()},
            f, ensure_ascii=False, indent=2,
        )

    # -- info.json ---------------------------------------------------------------------
    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32", "shape": [state_dim], "names": state_names,
        },
        "action": {
            "dtype": "float32", "shape": [state_dim], "names": state_names,
        },
    }
    features.update({key: {"dtype": "int64" if key == "control_owner" else "bool",
                           "shape": [1], "names": None}
                     for key in ("intervention", "intervention_valid", "control_owner")})
    for vkey in video_keys:
        h, w = shapes[vkey]
        features[vkey] = {
            "dtype": "video",
            "shape": [h, w, 3],
            "names": ["height", "width", "channels"],
            "info": video_info.get(vkey, {}),
        }
    features.update({
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
    })

    n_ep = ep_index  # 实际写入的段数（NaN 段已跳过）
    if n_ep == 0:
        raise RuntimeError("没有可导出的健康 episode（全部含 NaN/Inf）")
    info = {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": robot_type,
        "total_episodes": n_ep,
        "total_frames": total_frames,
        "total_tasks": len(tasks),
        "total_videos": n_ep * len(video_keys),
        "total_chunks": (n_ep + CHUNK_SIZE - 1) // CHUNK_SIZE if n_ep else 0,
        "chunks_size": CHUNK_SIZE,
        "fps": fps,
        "splits": {"train": f"0:{n_ep}"},
        "data_path": DATA_PATH_TEMPLATE,
        "video_path": VIDEO_PATH_TEMPLATE,
        "features": features,
        "control_owner_codes": {"unknown": 0, "policy": 1, "human": 2, "playback": 3, "pending": 4, "idle": 5},
        "morla": {
            "profile_id": getattr(schema, "profile_id", None),
            "robot_id": getattr(schema, "robot_id", None),
            "schema_hash": getattr(schema, "schema_hash", None),
            "layout_hash": getattr(schema, "layout_hash", None),
            "kinematics_hash": getattr(schema, "kinematics_hash", None),
            "coordinate_contract": getattr(schema, "metadata", {}).get("coordinate_contract", {}),
            "sensor_export_policy": "sidecar_only; not a LeRobot policy feature",
            "sensor_sidecars": "meta/sensors/episode_{episode_index:06d}.h5",
            "legacy_validity_unknown": any(bool(meta.get("legacy_validity_unknown")) for meta in metas),
        },
    }
    with open(os.path.join(output_dir, "meta", "info.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, ensure_ascii=False, indent=2)

    contract = {
        "schema": schema.to_dict(),
        "profile_id": getattr(schema, "profile_id", None),
        "robot_id": getattr(schema, "robot_id", None),
        "schema_hash": getattr(schema, "schema_hash", None),
        "layout_hash": getattr(schema, "layout_hash", None),
        "kinematics_hash": getattr(schema, "kinematics_hash", None),
        "coordinate_contract": getattr(schema, "metadata", {}).get("coordinate_contract", {}),
        "state_features": state_names,
        "action_features": state_names,
        "state_dim": state_dim,
        "action_dim": schema.action_dim,
        "fps": fps,
        "action_source": schema.action_source,
        "sensor_policy": {
            "mode": "sidecar_only",
            "policy_features_enabled": False,
            "excluded_from_policy": [dict(sensor) for sensor in getattr(schema, "sensor_specs", ())],
            "reason": "The selected LeRobot model feature contract does not declare tactile inputs.",
        },
        "model_capacity_check": {"capacity": model_capacity, "enforced": model_capacity is not None},
        "episodes": [
            {"episode_index": i, "source_episode": os.path.abspath(ep_dir),
             "legacy_validity_unknown": bool(meta.get("legacy_validity_unknown")),
             "provenance": meta.get("provenance", {})}
            for i, (ep_dir, meta) in enumerate(zip(episode_dirs, metas))
        ],
    }
    with open(os.path.join(output_dir, "meta", "morla_schema.json"), "w", encoding="utf-8") as f:
        json.dump(contract, f, ensure_ascii=False, indent=2)

    summary = {
        "output": output_dir,
        "episodes": n_ep,
        "frames": total_frames,
        "fps": fps,
        "state_dim": state_dim,
        "video_keys": video_keys,
        "codec": used_codec,
        "tasks": tasks,
    }
    log(
        f"LeRobot v2.1 dataset -> {output_dir}\n"
        f"  {n_ep} episodes / {total_frames} frames @ {fps}Hz, "
        f"state_dim={state_dim}, cams={len(video_keys)}, codec={used_codec}"
    )
    return summary


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session", required=True, help="session 目录（含已对齐 episode*）")
    ap.add_argument("--output", required=True, help="LeRobot 数据集输出目录")
    ap.add_argument("--robot-type", default="astral_dual_arm")
    ap.add_argument("--model-capacity", type=int, default=None,
                    help="可选目标模型动作/状态维数；通用LeRobot导出默认不限制维数")
    ap.add_argument("--codec", default="libsvtav1", help="libsvtav1（默认）| h264，不可用自动回退")
    ap.add_argument(
        "--image-size",
        type=int,
        default=224,
        help="视频落盘前 letterbox 到该边长（默认 224，与 openpi 输入一致）；0 = 保留原分辨率",
    )
    args = ap.parse_args(argv)
    convert_session(
        args.session,
        args.output,
        robot_type=args.robot_type,
        codec=args.codec,
        image_size=args.image_size or None,
        model_capacity=args.model_capacity,
    )


if __name__ == "__main__":
    main(sys.argv[1:])
