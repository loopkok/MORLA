"""离线对齐：raw HDF5 → 固定 fps 时间网格上的逐帧 (state, action)。

网格取严格等距 t0 + i/fps（LeRobot v2.1 check_timestamps_sync 容差 1e-4
的硬要求）；每个网格点取参考时刻最近的相机帧与数值流采样。

action 语义（schema.action_source）：
  next_state : action[t] = state[t+1]，末帧复制自身（hold）
  command    : action[t] = 指令流在 t 时刻的最近采样
               （腰/头无采集侧指令话题，仍回退 next_state 语义）

用法：
  python3 align_data.py --session ~/astral_data/pick_place [--force]
  ros2 run morla_data_collect data_pipeline align --session ~/astral_data/pick_place
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any

import h5py
import numpy as np

from morla_data_collect.pipeline.artifacts import (
    atomic_file,
    require_source_signature,
    source_signature,
)
from morla_data_collect.data_writer import CAMERA_H5, META_JSON, ROBOT_H5
from morla_core.schema import ACTION_COMMAND
from morla_data_collect.schema import pipeline_schema_from_dict

ALIGNED_H5 = "aligned_data.h5"

# 每帧质量位（写入 /quality u8）
FLAG_IMAGE_FAR = 0x01    # 最近图像帧偏差 > 1.5/fps
FLAG_SYNTHETIC_HOLD = 0x04
FLAG_STATE_GAP = 0x02    # 某数值流最近采样间隔 > max_gap_ms
SENSOR_FIELDS = ("force_raw", "force_sum", "temperature_raw", "temperature_mean", "valid", "temperature_valid")


def find_nearest_idx(sorted_ts: np.ndarray, value: float) -> int:
    """已排序时间数组中最近值下标（二分）。"""
    idx = int(np.searchsorted(sorted_ts, value, side="left"))
    if idx > 0 and (
        idx == len(sorted_ts)
        or abs(value - sorted_ts[idx - 1]) < abs(value - sorted_ts[idx])
    ):
        return idx - 1
    return idx


def _valid_timestamps(ts: np.ndarray, name: str) -> None:
    if ts.ndim != 1 or not np.all(np.isfinite(ts)) or np.any(np.diff(ts) <= 0):
        raise ValueError(f"{name}: timestamps must be finite and strictly increasing")


def _reject_pause_intervals(events: list[dict], t0: float, t1: float) -> None:
    paused = None
    for event in sorted(events, key=lambda e: e.get("t", 0)):
        if event.get("name") != "pause":
            continue
        t = float(event["t"])
        if event.get("data"):
            paused = t
        elif paused is not None:
            if min(t, t1) - max(paused, t0) > 1e-6:
                raise ValueError("pause interval inside episode: split the recording before alignment")
            paused = None
    if paused is not None and t1 - max(paused, t0) > 1e-6:
        raise ValueError("pause interval inside episode: split the recording before alignment")


def load_meta(episode_dir: str) -> dict[str, Any]:
    with open(os.path.join(episode_dir, META_JSON), encoding="utf-8") as f:
        return json.load(f)


def _resolve_block_source(
    block_name: str, stream_name: str, body_slice, stream_counts: dict[str, int]
) -> tuple[str, tuple[int, int] | None] | None:
    """决定块的实际数据源；返回 (流名, 切片) 或 None（无可用源）。"""
    if body_slice is not None:
        if stream_counts.get("body_state", 0) > 0:
            return ("body_state", body_slice)
        if block_name == "head" and stream_counts.get("head_state", 0) > 0:
            return ("head_state", None)  # 仿真等无 body_state 场景的回退
        return None
    if stream_counts.get(stream_name, 0) > 0:
        return (stream_name, None)
    return None


def align_episode(
    episode_dir: str,
    *,
    force: bool = False,
    log=print,
) -> dict[str, Any] | None:
    """对齐单段 episode，输出 aligned_data.h5；返回统计信息（失败返回 None）。"""
    out_path = os.path.join(episode_dir, ALIGNED_H5)
    meta = load_meta(episode_dir)
    if os.path.exists(out_path) and not force:
        from morla_data_collect.pipeline.validate_data import _Report, _check_aligned
        rep = _Report(episode_dir)
        with h5py.File(out_path, "r") as f:
            _check_aligned(rep, f, pipeline_schema_from_dict(meta["schema"]))
            try:
                require_source_signature(f, episode_dir)
            except ValueError as exc:
                raise ValueError(f"{exc}; use --force to rebuild alignment") from exc
        if rep.status == "fail":
            raise ValueError(f"invalid existing aligned file; use --force: {rep.issues}")
        log(f"  skip (verified aligned exists): {episode_dir}")
        return None

    schema = pipeline_schema_from_dict(meta["schema"])
    fps = int(schema.dataset_fps)

    robot_h5 = os.path.join(episode_dir, ROBOT_H5)
    camera_h5 = os.path.join(episode_dir, CAMERA_H5)
    if not (os.path.exists(robot_h5) and os.path.exists(camera_h5)):
        log(f"  skip (missing h5): {episode_dir}")
        return None

    with h5py.File(robot_h5, "r") as fr, h5py.File(camera_h5, "r") as fc:
        # -- 数值流加载（只加载非空流） ------------------------------------------
        streams: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for name in fr["streams"]:
            ts = fr[f"streams/{name}/timestamps"][:]
            if len(ts) > 0:
                _valid_timestamps(ts, name)
                values = fr[f"streams/{name}/values"][:]
                if values.ndim != 2 or len(values) != len(ts):
                    raise ValueError(f"{name}: value/timestamp shape mismatch")
                streams[name] = (ts, values)
        stream_counts = {name: len(ts) for name, (ts, _) in streams.items()}

        # -- 相机流加载 ------------------------------------------------------------
        cams: dict[str, tuple[np.ndarray, Any]] = {}
        for cam in schema.cameras:
            if cam in fc and len(fc[f"{cam}/timestamps"]) > 0:
                cams[cam] = (fc[f"{cam}/timestamps"][:], fc[f"{cam}/images"])
        if set(cams) != set(schema.cameras):
            raise ValueError("missing configured cameras")
        for cam, (ts, images) in cams.items():
            _valid_timestamps(ts, cam)
            if len(images) != len(ts):
                raise ValueError(f"{cam}: image/timestamp count mismatch")

        # High-dimensional tactile data is a separate sensor sidecar, never
        # concatenated into the model state/action vector.
        raw_sensors: dict[str, dict[str, np.ndarray]] = {}
        for sensor in getattr(schema, "sensor_specs", ()):
            key = str(sensor.get("sensor_key", ""))
            path = f"sensors/{key}"
            if path not in fr:
                if sensor.get("required", False):
                    raise ValueError(f"required sensor {key!r} is missing")
                continue
            group = fr[path]
            ts = group["timestamps"][:] if "timestamps" in group else np.empty(0)
            if len(ts) == 0:
                if sensor.get("required", False):
                    raise ValueError(f"required sensor {key!r} is empty")
                continue
            _valid_timestamps(ts, f"sensor:{key}")
            raw_sensors[key] = {name: group[name][:] for name in group.keys()}
            raw_sensors[key]["timestamps"] = ts

        # -- 时间网格：参考相机 = cameras 列表里第一个有数据的 -----------------------
        ref_cam = schema.cameras[0]
        ref_ts = cams[ref_cam][0]
        t0 = float(ref_ts[0])
        t_end = float(ref_ts[-1])
        _reject_pause_intervals(meta.get("events", []), t0, t_end)
        n_frames = int(np.floor((t_end - t0) * fps)) + 1
        if n_frames < 2:
            log(f"  skip (too short: {n_frames} frames): {episode_dir}")
            return None
        grid = t0 + np.arange(n_frames, dtype=np.float64) / fps

        blocks = schema.state_blocks()
        state_dim = schema.state_dim

        # -- 解析每块数据源 ----------------------------------------------------------
        state_src: list[tuple[str, tuple[int, int] | None] | None] = [
            _resolve_block_source(b.name, b.state_stream, b.body_slice, stream_counts)
            for b in blocks
        ]
        missing = [b.name for b, s in zip(blocks, state_src) if s is None]

        # -- 逐网格点采样 ----------------------------------------------------------
        state = np.full((n_frames, state_dim), np.nan, dtype=np.float32)
        quality = np.zeros(n_frames, dtype=np.uint8)
        max_gap_s = float(schema.max_gap_ms) / 1000.0
        img_far_s = 1.5 / fps

        for bi, (block, src) in enumerate(zip(blocks, state_src)):
            if src is None:
                continue
            sname, sl = src
            ts, values = streams[sname]
            col = sum(b.dim for b in blocks[:bi])
            for i, t in enumerate(grid):
                j = (max(0, int(np.searchsorted(ts, t, side="right")) - 1)
                     if "gripper_ratio" in sname else find_nearest_idx(ts, float(t)))
                if abs(float(ts[j]) - t) > max_gap_s:
                    quality[i] |= FLAG_STATE_GAP
                row = values[j]
                if sl is not None:
                    state[i, col:col + block.dim] = row[sl[0]:sl[1]]
                elif getattr(block, "stream_indices", ()):
                    state[i, col:col + block.dim] = row[list(block.stream_indices)]
                else:
                    state[i, col:col + block.dim] = row[:block.dim]

        aligned_sensors: dict[str, dict[str, np.ndarray]] = {}
        for sensor in getattr(schema, "sensor_specs", ()):
            key = str(sensor.get("sensor_key", ""))
            source = raw_sensors.get(key)
            if source is None:
                continue
            source_ts = source["timestamps"]
            n_fingers = len(sensor.get("finger_order", ("thumb", "index", "middle", "ring", "pinky")))
            points = int(sensor.get("point_count_per_finger", 120))
            n_source = len(source_ts)
            expected_shapes = {
                "force_raw": (n_fingers, points, 3), "force_sum": (n_fingers, 3),
                "temperature_raw": (n_fingers, 20), "temperature_mean": (n_fingers,),
                "valid": (n_fingers,), "temperature_valid": (n_fingers,),
            }
            for name, shape in expected_shapes.items():
                if name not in source or source[name].shape != (n_source,) + shape:
                    raise ValueError(f"sensor {key}/{name}: expected shape {(n_source,) + shape}")
            source_index = np.full(n_frames, -1, dtype=np.int64)
            offsets = np.zeros(n_frames, dtype=np.float64)
            for i, t in enumerate(grid):
                j = find_nearest_idx(source_ts, float(t))
                delta = float(source_ts[j]) - float(t)
                offsets[i] = delta
                if abs(delta) <= max_gap_s:
                    source_index[i] = j
            result = {name: np.zeros((n_frames,) + expected_shapes[name], dtype=source[name].dtype)
                      for name in SENSOR_FIELDS}
            result.update({"source_index": source_index,
                           "source_timestamps": np.zeros(n_frames, dtype=np.float64),
                           "source_offsets": offsets})
            valid_rows = source_index >= 0
            result["source_timestamps"][valid_rows] = source_ts[source_index[valid_rows]]
            for name in SENSOR_FIELDS:
                result[name][valid_rows] = source[name][source_index[valid_rows]]
            result["valid"][~valid_rows] = False
            result["temperature_valid"][~valid_rows] = False
            if sensor.get("required", False):
                quality[np.any(~result["valid"], axis=1)] |= FLAG_STATE_GAP
            aligned_sensors[key] = result

        # -- action ------------------------------------------------------------------
        if schema.action_source == ACTION_COMMAND:
            action = np.full((n_frames, state_dim), np.nan, dtype=np.float32)
            for bi, block in enumerate(blocks):
                col = sum(b.dim for b in blocks[:bi])
                if block.cmd_stream is None:
                    action[:-1, col:col + block.dim] = state[1:, col:col + block.dim]
                    action[-1, col:col + block.dim] = state[-1, col:col + block.dim]
                    continue
                cmd_stream = schema.action_block_stream(block)
                src = _resolve_block_source(
                    block.name, cmd_stream,
                    block.body_slice if cmd_stream == "body_state" else None,
                    stream_counts,
                )
                if src is None:
                    continue
                sname, sl = src
                ts, values = streams[sname]
                col = sum(b.dim for b in blocks[:bi])
                for i, t in enumerate(grid):
                    j = max(0, int(np.searchsorted(ts, t, side="right")) - 1)
                    if t < ts[0] or abs(float(ts[j]) - t) > max_gap_s:
                        quality[i] |= FLAG_STATE_GAP
                    if sl is not None:
                        action[i, col:col + block.dim] = values[j][sl[0]:sl[1]]
                    elif getattr(block, "stream_indices", ()):
                        action[i, col:col + block.dim] = values[j][list(block.stream_indices)]
                    else:
                        action[i, col:col + block.dim] = values[j][:block.dim]
        else:  # next_state
            action = np.empty_like(state)
            action[:-1] = state[1:]
            action[-1] = state[-1]  # 末帧 hold

        # -- 图像：每网格点取最近帧（保留 JPEG 字节） ---------------------------------
        cam_idx: dict[str, np.ndarray] = {}
        cam_off: dict[str, np.ndarray] = {}
        for cam, (ts, _imgs) in cams.items():
            idx = np.empty(n_frames, dtype=np.int64)
            off = np.empty(n_frames, dtype=np.float64)
            for i, t in enumerate(grid):
                j = find_nearest_idx(ts, float(t))
                idx[i] = j
                off[i] = float(ts[j]) - t
            far = np.abs(off) > img_far_s
            quality[far] |= FLAG_IMAGE_FAR
            cam_idx[cam] = idx
            cam_off[cam] = off

        # -- hold 帧（episode 末追加） ------------------------------------------------
        hold = int(schema.hold_frames)
        if hold > 0:
            dt = 1.0 / fps
            grid = np.concatenate([grid, grid[-1] + dt * np.arange(1, hold + 1)])
            state = np.concatenate([state, np.repeat(state[-1:], hold, axis=0)])
            action = np.concatenate([action, np.repeat(action[-1:], hold, axis=0)])
            quality = np.concatenate([quality, np.full(hold, FLAG_SYNTHETIC_HOLD, dtype=np.uint8)])
            for cam in cams:
                cam_idx[cam] = np.concatenate(
                    [cam_idx[cam], np.full(hold, cam_idx[cam][-1], dtype=np.int64)]
                )
                cam_off[cam] = np.concatenate([cam_off[cam],
                    cams[cam][0][cam_idx[cam][-1]] - grid[-hold:]])
            for result in aligned_sensors.values():
                source_rows = np.flatnonzero(result["source_index"] >= 0)
                carry_row = int(source_rows[-1]) if len(source_rows) else len(result["source_index"]) - 1
                carried = {name: result[name][carry_row:carry_row + 1].copy()
                           for name in SENSOR_FIELDS}
                last_index = int(result["source_index"][carry_row])
                last_source_ts = float(result["source_timestamps"][carry_row])
                for name in SENSOR_FIELDS:
                    result[name] = np.concatenate([result[name], np.repeat(carried[name], hold, axis=0)])
                result["valid"][-hold:] = False
                result["temperature_valid"][-hold:] = False
                result["source_index"] = np.concatenate([
                    result["source_index"], np.full(hold, last_index, dtype=np.int64)
                ])
                result["source_timestamps"] = np.concatenate([
                    result["source_timestamps"], np.full(hold, last_source_ts, dtype=np.float64)
                ])
                result["source_offsets"] = np.concatenate([
                    result["source_offsets"], last_source_ts - grid[-hold:]
                ])

        total_frames = len(grid)

        # -- 写 aligned_data.h5 --------------------------------------------------------
        vlen_u8 = h5py.special_dtype(vlen=np.dtype("uint8"))
        with atomic_file(out_path) as staged, h5py.File(staged, "w") as fo:
            fo.create_dataset("observation/state", data=state)
            fo.create_dataset("action", data=action)
            fo.create_dataset("timestamps", data=grid)
            fo.create_dataset("quality", data=quality)
            from .intervention import labels_at
            labels = labels_at(meta, grid)
            synthetic = (quality & FLAG_SYNTHETIC_HOLD) != 0
            for values in labels.values(): values[synthetic] = 0
            for key, values in labels.items():
                fo.create_dataset(key, data=values)
            fo.attrs['control_owner_codes'] = json.dumps(
                {'unknown': 0, 'policy': 1, 'human': 2, 'playback': 3, 'pending': 4, 'idle': 5})
            for cam, (ts, imgs) in cams.items():
                g = fo.create_group(cam)
                ds = g.create_dataset(
                    "images", shape=(total_frames,), dtype=vlen_u8
                )
                for i in range(total_frames):
                    ds[i] = imgs[int(cam_idx[cam][i])]
                g.create_dataset("src_timestamps", data=ts[cam_idx[cam]])
                g.create_dataset("src_offsets", data=cam_off[cam])
            for key, result in aligned_sensors.items():
                group = fo.create_group(f"sensors/{key}")
                for name, value in result.items():
                    group.create_dataset(name, data=value, compression="lzf" if name in {"force_raw", "temperature_raw"} else None)
                group.attrs["alignment"] = "nearest_source_sample_with_max_gap; no_interpolation"
                group.attrs["source_kind"] = meta.get("sensors", {}).get(key, {}).get("source_kind", "unknown")
                group.attrs["force_unit"] = meta.get("sensors", {}).get(key, {}).get("force_unit", "unknown")
                group.attrs["force_frame"] = meta.get("sensors", {}).get(key, {}).get("force_frame", "unknown")
                group.attrs["temperature_source_kind"] = meta.get("sensors", {}).get(key, {}).get("temperature_source_kind", "unknown")

            fo.create_dataset("source_frame_index", data=np.arange(total_frames, dtype=np.int64))
            fo.create_dataset("source_grid_timestamps", data=grid)
            fo.attrs["processing_version"] = "align_v2"
            fo.attrs["raw_signature"] = source_signature(episode_dir)
            fo.attrs["hold_frames"] = hold
            fo.attrs["fps"] = fps
            fo.attrs["num_frames"] = total_frames
            fo.attrs["state_dim"] = state_dim
            fo.attrs["state_names"] = json.dumps(schema.state_names())
            fo.attrs["schema"] = schema.to_json()
            fo.attrs["task"] = meta.get("task", "")
            fo.attrs["episode_index"] = int(meta.get("episode_index", -1))
            fo.attrs["missing_blocks"] = json.dumps(missing)
            fo.attrs["description"] = (
                "Aligned on a uniform 1/fps grid. action semantics: "
                f"{schema.action_source}."
            )

    stats = {
        "episode_dir": episode_dir,
        "frames": total_frames,
        "duration_s": round(total_frames / fps, 2),
        "missing_blocks": missing,
        "state_gap_frames": int(np.count_nonzero(quality & FLAG_STATE_GAP)),
        "image_far_frames": int(np.count_nonzero(quality & FLAG_IMAGE_FAR)),
        "max_img_offset_ms": {
            cam: round(float(np.max(np.abs(off))) * 1000.0, 2)
            for cam, off in cam_off.items()
        },
        "sensors": {key: {"samples": int(len(value["source_index"])),
                           "valid_samples": int(np.count_nonzero(value["valid"])),
                           "missing_frames": int(np.count_nonzero(value["source_index"] < 0))}
                    for key, value in aligned_sensors.items()},
    }
    log(
        f"  aligned {os.path.basename(episode_dir)}: {total_frames} frames "
        f"({stats['duration_s']}s), missing={missing or '[]'}, "
        f"state_gap={stats['state_gap_frames']}, img_far={stats['image_far_frames']}"
    )
    return stats


def align_session(session_dir: str, *, force: bool = False, log=print) -> list[dict]:
    """批量对齐一个 session 下的全部 episode。"""
    episode_dirs = sorted(
        os.path.join(session_dir, d)
        for d in os.listdir(session_dir)
        if d.startswith("episode")
        and os.path.isdir(os.path.join(session_dir, d))
    )
    if not episode_dirs:
        log(f"no episode* dirs under {session_dir}")
        return []
    log(f"aligning {len(episode_dirs)} episodes in {session_dir}")
    out = []
    errors = []
    for ep in episode_dirs:
        try:
            r = align_episode(ep, force=force, log=log)
            if r is not None:
                out.append(r)
        except Exception as exc:
            log(f"  ERROR {ep}: {exc}")
            errors.append(f"{os.path.basename(ep)}: {exc}")
    if errors:
        raise RuntimeError("alignment failed: " + "; ".join(errors))
    log(f"done: {len(out)}/{len(episode_dirs)} aligned")
    return out


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session", required=True, help="session 目录（含 episode*）")
    ap.add_argument("--force", action="store_true", help="覆盖已有 aligned_data.h5")
    args = ap.parse_args(argv)
    align_session(os.path.expanduser(args.session), force=args.force)


if __name__ == "__main__":
    main(sys.argv[1:])
