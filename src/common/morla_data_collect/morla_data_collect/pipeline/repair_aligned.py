#!/usr/bin/env python3
"""对齐数据完整修复：两种模式——**natural 保时序摩擦移除**（默认）/ **uniformize 弧长匀速化**。

目的：用 command/state 差异识别候选摩擦停顿，保守处理慢速遥操示范。
停顿也可能是操作意图；仅凭轨迹不能证明真机卡点的成因。本脚本在 **aligned_data.h5 层**用
**选帧**处理。**选帧**（子集）保证 state 与图像**严格同源**、零错位；插值会破坏
这一点，故不用。输出训练时间均匀重定时，同时保留原始采样时间与来源索引。

--mode natural（默认，推荐）：**保时序摩擦移除**——删静摩擦"卡住"帧（全臂停住、
  夹爪没在动、非意图停顿、连续 ≥ min_hold 帧），其余帧**原样保留**，每个保留帧 →
  一个输出帧，不统一运动段速度；删除帧的连接处可能改变速度，输出检查新增速度峰值。
  夹爪闭合/释放过渡 ±5 帧也保留（抓取上下文不丢，不会"猛合猛放"）；意图停顿
  （默认 --keep-intent）时长保留；无 command 证据不删除。

--mode uniformize（可选，工程化提速的数据烘焙）：**弧长匀速化**——臂运动段按
  --speed-ref 重定时为统一速度，训练"更快/匀速执行"的模型。**夹爪与意图停顿受保护**
  （1:1 保留，不再像旧 v2 那样压没 → 猛合）。speed-ref=0 自动取 session 臂维弧长均值
  （总时长不变）；更高值更快、更低更慢。

处理对象：session 下 episode*/aligned_data.h5（已对齐的数据）。输出新 session 的
aligned_data.h5（+ 原样复制 robot/camera/meta，供后续 data_pipeline 直接导出——
align 检测到 aligned 已存在会跳过）。

用法:
  # natural（默认）：
  bash MORLA/scripts/data_pipeline.sh repair \
      --session ~/astral_data/raw/pick_place_merged \
      --out-session ~/astral_data/raw/pick_place_merged_repaired \
      [--fps 30] [--flat-v 0.02] [--min-hold 2] [--keep-intent] [--dry-run]
  # uniformize（训练更快执行；速度由 --speed-ref 控）：
  ... 同参数 + --mode uniformize [--speed-ref 0.05]

参数:
  --mode natural|uniformize   保时序（默认）或 弧长匀速化
  --speed-ref rad/帧          仅 uniformize：目标每帧弧长（0=自动取臂维均值）
  --fps                      输出时间戳网格（默认源 schema 的 fps；不允许与源不一致）
  --flat-v rad/s             判定"停住"的臂速度阈值（默认 0.02；越低只删越彻底的停顿）
  --min-hold 帧              连续 ≥ 该帧数的摩擦停顿才删（默认 2；更短当噪声保留）
  --keep-intent              默认启用；--remove-all-stalls 显式关闭意图保护
  --dry-run                  只报告不写文件
退出码 0=成功。
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys

import h5py
import numpy as np

from morla_data_collect.pipeline.quantify_cmd_state import _cross_lag
from morla_data_collect.pipeline.artifacts import (
    atomic_directory,
    reject_overlap,
    require_source_signature,
    source_signature,
)
from morla_data_collect.schema import pipeline_schema_from_dict


CAM_KEYS = None  # 运行时探测 aligned 里的 camera 组名（left_wrist/base/...）


def _block_masks(ep_dir: str, n_dim: int) -> tuple[np.ndarray | None, np.ndarray | None]:
    """返回 (臂维掩码, 末端掩码)。臂维参与停顿判定；末端(夹爪/wuji)在动则帧保留。

    读 meta.json 的 schema.state_blocks：name 含 "arm" → 臂；含 "ee" → 末端（gripper
    的 ratio 无量纲，不参与停顿判定，但其变化必须保留——夹爪闭合/释放不能压掉）。
    无 meta/schema 时回退 (None, None)=臂全维、无末端保护（历史行为）。
    """
    try:
        with open(os.path.join(ep_dir, "meta.json"), encoding="utf-8") as f:
            meta = json.load(f)
        arm, ee = [], []
        schema = pipeline_schema_from_dict(meta["schema"])
        for b in schema.state_blocks():
            arm += ["arm" in b.name] * b.dim
            ee += ["ee" in b.name] * b.dim
        if len(arm) == n_dim:
            return np.asarray(arm, dtype=bool), np.asarray(ee, dtype=bool)
    except Exception:  # noqa: BLE001
        pass
    return None, None


def _camera_groups(f) -> list[str]:
    return [k for k in f.keys()
            if isinstance(f[k], h5py.Group) and "images" in f[k]]


def _load_cmd(ep_dir: str, arm_name: str | None = None) -> tuple[np.ndarray, np.ndarray] | None:
    """读 robot_data.h5 的臂 cmd 流（values, timestamps）；无则 None。

    arm_name（如 "left_arm"）给则优先取 {arm_name}_cmd（未来双臂时不会拿错侧），
    否则取第一个 *_cmd 流。
    """
    p = os.path.join(ep_dir, "robot_data.h5")
    if not os.path.exists(p):
        return None
    with h5py.File(p, "r") as f:
        if "streams" not in f:
            return None
        names = [k for k in f["streams"] if k.endswith("_cmd")]
        if not names:
            return None
        name = names[0]
        if arm_name and (arm_name + "_cmd") in f["streams"]:
            name = arm_name + "_cmd"
        g = f[f"streams/{name}"]
        return (np.asarray(g["values"], dtype=np.float64),
                np.asarray(g["timestamps"], dtype=np.float64))


def _intent_hold_mask(state: np.ndarray, ts: np.ndarray,
                      cmd: np.ndarray, cmd_ts: np.ndarray,
                      arm_mask: np.ndarray | None,
                      flat_v: float = 0.02, min_pause: float = 0.15,
                      intent_frac: float = 0.5) -> np.ndarray:
    """aligned 帧级意图停顿掩码（True=操作者有意停顿，repair 应保留其时长）。

    判据：state 全臂停住（每步 max|Δarm|/dt < flat_v）的停顿段里，cmd 对齐后同步也停
    （窗口内 cmd 平段占比 >= intent_frac）→ 意图型（保留）；cmd 在动而 state 停 → 摩擦
    型（不保留，照常压缩）。cmd 与 state 时间戳语义不同（~200ms 恒偏移），先经
    _cross_lag（带方差/相关度守卫）对齐。cmd 值在 state 时刻 t 处取 cmd_ts ≈ t + lag。
    """
    st = state[:, arm_mask] if arm_mask is not None else state
    n = len(st)
    mask = np.zeros(n, dtype=bool)
    if n < 4 or cmd is None or len(cmd) < 4 or len(ts) < 4:
        return mask
    lag = _cross_lag(st.mean(axis=1)[:, None], ts,
                     cmd.mean(axis=1)[:, None], cmd_ts)
    c_idx = np.clip(np.searchsorted(cmd_ts, ts + lag, side="right") - 1,
                    0, len(cmd_ts) - 1)
    c_arm = cmd[c_idx]   # cmd 流只有臂关节维，无需 arm_mask（无 ee/waist 污染）
    dt = np.maximum(np.diff(ts), 1e-6)
    cflat = np.abs(np.diff(c_arm, axis=0)).max(axis=1) / dt < flat_v
    sflat = np.abs(np.diff(st, axis=0)).max(axis=1) / dt < flat_v
    i, L = 0, len(sflat)
    while i < L:
        if sflat[i]:
            j = i
            while j < L and sflat[j]:
                j += 1
            if ts[j] - ts[i] >= min_pause:
                frac = float(cflat[i:j].mean()) if j > i else 0.0
                if frac >= intent_frac:
                    mask[i:j + 1] = True
            i = j
        else:
            i += 1
    return mask


def resample_frames(state: np.ndarray, quality: np.ndarray,
                    cam_images: dict, cam_offsets: dict, cam_ts: dict,
                    fps: int, state_ts0: float = 0.0,
                    arm_mask: np.ndarray | None = None,
                    ee_mask: np.ndarray | None = None,
                    intent_hold_mask: np.ndarray | None = None,
                    flat_v: float = 0.02, min_hold: int = 2):
    """摩擦停顿移除（**保时序**，选帧零错位）：删掉静摩擦"卡住"的帧，其余保留。

    与旧"弧长均匀化"的本质区别：**不做速度统一**。每个保留的原始帧 → 一个输出帧，
    速度 = 自然速度（慢速精密保持慢、快速保持快）；只删**摩擦型停顿**——全臂停住
    （max|Δarm|/dt < flat_v）且全末端没在动（夹爪闭合/释放的帧保留）、非意图停顿、
    且连续 ≥ min_hold 帧。这样：
      * 慢速段不再被提速（旧均匀化把 p10 速度 0.003→0.012，4x，回放"整体加快"）；
      * 夹爪闭合/释放帧 + 周围抓取停顿保留 → 不再"猛合猛放"；
      * 意图停顿（keep-intent）完整保留其时长。

    选帧（子集）保证 state/图像严格同源、零错位。保留帧 < 2 的退化 episode 返回 None
    （调用方原样复制）。
    """
    st = state[:, arm_mask] if arm_mask is not None else state
    ee = state[:, ee_mask] if (ee_mask is not None and np.any(ee_mask)) else None
    n = len(state)
    if n < 3:
        return None
    dt = 1.0 / fps
    arm_stuck = np.abs(np.diff(st, axis=0)).max(axis=1) / dt < flat_v
    if ee is not None:
        ee_moving = np.abs(np.diff(ee, axis=0)).max(axis=1) / dt >= flat_v
        # 夹爪活动保护窗：过渡（闭合/释放）前后 ±5 帧也保留——夹爪 ratio 是指令回显，
        # 过渡本身只 1-2 帧，但抓取保持/释放的上下文（操作者停臂让夹爪完成动作）必须留，
        # 否则回放表现为"猛合猛放"。意图分类没抓到的抓取停顿靠这里兜底。
        if ee_moving.any():
            prot = np.zeros(len(ee_moving), dtype=bool)
            for k in np.where(ee_moving)[0]:
                prot[max(0, k - 5):min(len(ee_moving), k + 6)] = True
            ee_moving = prot
        friction = arm_stuck & ~ee_moving
    else:
        friction = arm_stuck
    if intent_hold_mask is not None:
        inside = intent_hold_mask[1:] & intent_hold_mask[:-1]
        friction = friction & ~inside
    # 摩擦段 < min_hold 帧 → 视为噪声不删
    i, L = 0, len(friction)
    while i < L:
        if friction[i]:
            j = i
            while j < L and friction[j]:
                j += 1
            if j - i < min_hold:
                friction[i:j] = False
            i = j
        else:
            i += 1
    # 帧级保留：帧 0 恒保留；帧 f≥1 保留当且仅当步 f-1 非摩擦
    keep = np.ones(n, dtype=bool)
    if n > 1:
        keep[1:] = ~friction
    idx = np.where(keep)[0]
    if len(idx) < 2:
        return None

    state_new = state[idx]                                  # 原始帧值（非插值）
    quality_new = quality[idx]
    cam_new = {c: cam_images[c][idx] for c in cam_images}
    off_new = {c: cam_offsets[c][idx] for c in cam_offsets}
    ts_cam_new = {c: cam_ts[c][idx] for c in cam_ts}

    # 均匀时间戳（帧间隔 = 1/fps 秒；严格单调）
    timestamps_new = float(state_ts0) + np.arange(len(idx)) / fps
    # next-state action
    action_new = np.vstack([state_new[1:], state_new[-1]])

    return state_new, action_new, quality_new, timestamps_new, idx, cam_new, off_new, ts_cam_new


def session_speed_ref(session: str, eps: list, arm_mask: np.ndarray | None) -> float:
    """session 全局 arm 弧长均值（uniformize 模式的自动 speed-ref）。

    均值 = 总弧长/总帧数：均匀化后总帧数≈原始（总时长不变）。要更快/更匀速可显式
    给更高的 --speed-ref。
    """
    ds = []
    for ep in eps:
        p = os.path.join(session, ep, "aligned_data.h5")
        if not os.path.exists(p):
            continue
        try:
            with h5py.File(p, "r") as f:
                require_source_signature(f, os.path.join(session, ep))
                st = np.asarray(f["observation/state"], dtype=np.float64)
                if arm_mask is not None:
                    st = st[:, arm_mask]
            if len(st) < 3:
                continue
            d = np.abs(np.diff(st, axis=0)).sum(axis=1)
            ds.append(d)
        except ValueError:
            raise
        except Exception:  # noqa: BLE001
            continue
    if not ds:
        return 0.0
    all_d = np.concatenate(ds)
    return float(all_d.mean())


def resample_frames_uniform(state: np.ndarray, quality: np.ndarray,
                            cam_images: dict, cam_offsets: dict, cam_ts: dict,
                            speed_ref: float, fps: int, state_ts0: float = 0.0,
                            arm_mask: np.ndarray | None = None,
                            ee_mask: np.ndarray | None = None,
                            intent_hold_mask: np.ndarray | None = None,
                            flat_v: float = 0.02, min_hold: int = 2):
    """弧长匀速化（旧 v2 修好版）：臂按 --speed-ref 重定时，夹爪/意图停顿保护不压没。

    与 natural（保时序）的本质区别：**臂运动段按 speed-ref 统一重定时**——所有段都
    变成均匀速度，总帧数 ≈ 总弧长/speed-ref。适用：训练"更快/匀速执行"的模型
    （工程化提速的**数据烘焙**方案）。与旧 v2 的三处修复：
      * **夹爪保护**：夹爪闭合/释放过渡 ±5 帧按 1:1 保留（旧 v2 压没 → 猛合猛放）；
      * **意图停顿保护**：--keep-intent 的停顿帧 1:1 保留时长；
      * **摩擦移除**：摩擦型停顿帧照常删（d=0）。

    选帧保证 state/图像零错位。退化（speed_ref<=0 或总弧长 0）返回 None。
    """
    st = state[:, arm_mask] if arm_mask is not None else state
    ee = state[:, ee_mask] if (ee_mask is not None and np.any(ee_mask)) else None
    n = len(state)
    if n < 3 or speed_ref <= 0:
        return None
    dt = 1.0 / fps
    arm_arc = np.abs(np.diff(st, axis=0)).sum(axis=1)
    arm_stuck = np.abs(np.diff(st, axis=0)).max(axis=1) / dt < flat_v
    protected = np.zeros(len(arm_arc), dtype=bool)
    if ee is not None:
        ee_moving = np.abs(np.diff(ee, axis=0)).max(axis=1) / dt >= flat_v
        if ee_moving.any():
            prot = np.zeros(len(ee_moving), dtype=bool)
            for k in np.where(ee_moving)[0]:
                prot[max(0, k - 5):min(len(ee_moving), k + 6)] = True
            ee_moving = prot
        protected |= ee_moving
    if intent_hold_mask is not None:
        inside = intent_hold_mask[1:] & intent_hold_mask[:-1]
        protected |= inside
    friction = arm_stuck & ~protected
    i, L = 0, len(friction)
    while i < L:
        if friction[i]:
            j = i
            while j < L and friction[j]:
                j += 1
            if j - i < min_hold:
                friction[i:j] = False
            i = j
        else:
            i += 1
    # 度量：臂弧长；保护帧至少 1 输出帧（=speed_ref）；摩擦帧 0
    d = np.where(protected, np.maximum(arm_arc, speed_ref), arm_arc)
    d = np.where(friction, 0.0, d)
    S_total = d.sum()
    if S_total <= 0:
        return None
    N_new = max(2, int(round(S_total / speed_ref)))
    S = np.concatenate([[0.0], np.cumsum(d)])
    s_grid = np.linspace(0.0, S_total, N_new)
    idx = np.array([int(np.argmin(np.abs(S - g))) for g in s_grid])
    # 快速帧伪影去重（意图停顿帧豁免——其重复 = 时长正确表示）
    if len(idx) > 1:
        keep = np.concatenate([[True], idx[1:] != idx[:-1]])
        if intent_hold_mask is not None:
            keep = keep | intent_hold_mask[idx]
        idx = idx[keep]
    state_new = state[idx]
    quality_new = quality[idx]
    cam_new = {c: cam_images[c][idx] for c in cam_images}
    off_new = {c: cam_offsets[c][idx] for c in cam_offsets}
    ts_cam_new = {c: cam_ts[c][idx] for c in cam_ts}
    timestamps_new = float(state_ts0) + np.arange(len(idx)) / fps
    action_new = np.vstack([state_new[1:], state_new[-1]])
    return state_new, action_new, quality_new, timestamps_new, idx, cam_new, off_new, ts_cam_new


def repair_episode(ep_dir: str, out_ep_dir: str, fps: int | None, dry_run: bool,
                   keep_intent: bool = True, flat_v: float = 0.02,
                   min_hold: int = 2, mode: str = "natural", speed_ref: float = 0.0,
                   camera_map: dict[str, str] | None = None) -> dict:
    src = os.path.join(ep_dir, "aligned_data.h5")
    if not os.path.exists(src):
        raise ValueError(f"missing aligned file: {ep_dir}")
    if not dry_run:
        reject_overlap(ep_dir, out_ep_dir)
    with open(os.path.join(ep_dir, "meta.json"), encoding="utf-8") as mf:
        meta = json.load(mf)
    source_schema = pipeline_schema_from_dict(meta["schema"])
    schema = pipeline_schema_from_dict(meta["schema"])
    if camera_map:
        if hasattr(schema, "map_cameras"):
            schema.map_cameras(camera_map)
        else:
            schema.cameras = [camera_map.get(c, c) for c in schema.cameras]
            schema.__post_init__()
    fps = schema.dataset_fps if fps is None else fps
    if fps != schema.dataset_fps:
        raise ValueError("repair fps must match source schema")
    with h5py.File(src, "r") as f:
        require_source_signature(f, ep_dir)
        attrs = dict(f.attrs)
        labels = {k: f[k][:] for k in ('intervention', 'intervention_valid', 'control_owner') if k in f}
        state = f["observation/state"][:]
        original_action = f["action"][:]
        quality = f["quality"][:]
        ts_all = f["timestamps"][:]
        source_idx = f["source_frame_index"][:] if "source_frame_index" in f else np.arange(len(state))
        source_grid = f["source_grid_timestamps"][:] if "source_grid_timestamps" in f else ts_all
        camera_pairs = [(camera, (camera_map or {}).get(camera, camera))
                        for camera in source_schema.cameras]
        cams = [target for _source, target in camera_pairs]
        cam_images = {} if dry_run else {target: f[f"{source}/images"][:]
                                         for source, target in camera_pairs}
        cam_ts = {target: f[f"{source}/src_timestamps"][:]
                  for source, target in camera_pairs}
        cam_offsets = {c: cam_ts[c] - source_grid for c in cams}
        aligned_sensors = {}
        if "sensors" in f:
            for key, group in f["sensors"].items():
                fields = {name: ds[:] for name, ds in group.items() if isinstance(ds, h5py.Dataset)}
                bad = [name for name, value in fields.items() if value.ndim == 0 or len(value) != len(state)]
                if bad:
                    raise ValueError(f"aligned sensor {key} frame count mismatch: {bad}")
                aligned_sensors[key] = (fields, dict(group.attrs))
    if len(state) < 2 or not np.all(np.isfinite(state)) or not np.all(np.isfinite(original_action)):
        raise ValueError("repair requires finite state/action with at least two frames")
    if any(np.any(t <= 0) or not np.all(np.isfinite(t)) for t in cam_ts.values()):
        raise ValueError("source camera timestamps lost; rebuild aligned file from raw")
    arm_mask, _ = _block_masks(ep_dir, state.shape[1])
    if arm_mask is None:
        raise ValueError("repair requires valid state block mapping")
    # Waist/head movements protect frames just like end-effectors.
    ee_mask = ~arm_mask
    intent_mask = np.zeros(len(state), dtype=bool)
    hold = int(attrs.get("hold_frames", schema.hold_frames))
    if hold > 0:
        intent_mask[max(0, len(state) - hold - 1):] = True
    if keep_intent:
        col = 0
        for block in schema.state_blocks():
            if "arm" in block.name:
                cmd = _load_cmd(ep_dir, block.name)
                mask = np.zeros(state.shape[1], dtype=bool)
                mask[col:col+block.dim] = True
                if cmd is None:
                    intent_mask[:] = True  # Unknown intent must not be deleted.
                else:
                    intent_mask |= _intent_hold_mask(state, ts_all, cmd[0], cmd[1], mask)
            col += block.dim
    args = (state, quality, cam_images, cam_offsets, cam_ts)
    if mode == "uniformize":
        r = resample_frames_uniform(*args, speed_ref, fps, float(ts_all[0]),
                                    arm_mask, ee_mask, intent_mask, flat_v, min_hold)
    else:
        r = resample_frames(*args, fps, float(ts_all[0]), arm_mask, ee_mask,
                            intent_mask, flat_v, min_hold)
    if r is None:
        idx = np.arange(len(state))
        r = (state, original_action, quality, ts_all, idx, cam_images, cam_offsets, cam_ts)
    state_new, action_new, quality_new, ts_new, idx, cam_new, _, ts_cam_new = r
    if schema.action_source == "command":
        action_new = original_action[idx].copy()
        col = 0
        for block in schema.state_blocks():
            if block.cmd_stream is None:
                action_new[:-1, col:col+block.dim] = state_new[1:, col:col+block.dim]
                action_new[-1, col:col+block.dim] = state_new[-1, col:col+block.dim]
            col += block.dim
    before = np.abs(np.diff(state, axis=0)).max(axis=0) * fps
    after = np.abs(np.diff(state_new, axis=0)).max(axis=0) * fps
    if np.any(after > np.maximum(before, 4.0) + 1e-6):
        raise ValueError("repair would introduce excessive velocity; lower speed-ref")
    if not dry_run:
        with atomic_directory(out_ep_dir) as staged:
            with h5py.File(os.path.join(staged, "aligned_data.h5"), "w") as fo:
                for key, value in attrs.items():
                    fo.attrs[key] = value
                fo.attrs.update({"fps": fps, "num_frames": len(state_new),
                                 "state_dim": schema.state_dim,
                                 "state_names": json.dumps(schema.state_names()),
                                 "schema": schema.to_json(), "task": meta.get("task", ""),
                                 "episode_index": meta.get("episode_index", -1),
                                 "processing_version": "repair_v4", "repair_mode": mode,
                                 "hold_frames": hold})
                for key, value in [("observation/state", state_new), ("action", action_new),
                                   ("quality", quality_new), ("timestamps", ts_new),
                                   ("source_frame_index", source_idx[idx]),
                                   ("source_grid_timestamps", source_grid[idx])]:
                    fo.create_dataset(key, data=value)
                dtype = h5py.special_dtype(vlen=np.dtype("uint8"))
                for key, values in labels.items():
                    fo.create_dataset(key, data=values[idx])
                for c in cams:
                    cg = fo.create_group(c)
                    ds = cg.create_dataset("images", shape=(len(idx),), dtype=dtype)
                    for k, image in enumerate(cam_new[c]):
                        ds[k] = image
                    cg.create_dataset("src_timestamps", data=ts_cam_new[c])
                    cg.create_dataset("src_offsets", data=ts_cam_new[c] - ts_new)
                    cg.create_dataset("source_alignment_offsets", data=ts_cam_new[c] - source_grid[idx])
                for key, (fields, sensor_attrs) in aligned_sensors.items():
                    sg = fo.create_group(f"sensors/{key}")
                    for attr, value in sensor_attrs.items():
                        sg.attrs[attr] = value
                    for name, values in fields.items():
                        sg.create_dataset(name, data=values[idx],
                                          compression="lzf" if name in {"force_raw", "temperature_raw"} else None)
            for fname in ("robot_data.h5", "camera_data.h5", "meta.json"):
                shutil.copy2(os.path.join(ep_dir, fname), os.path.join(staged, fname))
            if camera_map:
                with h5py.File(os.path.join(staged, "camera_data.h5"), "r+") as f:
                    for old, new in camera_map.items():
                        if old == new or old not in f:
                            continue
                        if new in f:
                            raise ValueError(f"camera name collision {old}={new}")
                        f.move(old, new)
                meta["schema"] = schema.to_dict()
                if isinstance(meta.get("cameras"), dict):
                    meta["cameras"] = {camera_map.get(c, c): value
                                        for c, value in meta["cameras"].items()}
                with open(os.path.join(staged, "meta.json"), "w", encoding="utf-8") as f:
                    json.dump(meta, f, ensure_ascii=False, indent=2)
            with h5py.File(os.path.join(staged, "aligned_data.h5"), "r+") as f:
                f.attrs["raw_signature"] = source_signature(staged)
            with open(os.path.join(staged, "repair_report.json"), "w", encoding="utf-8") as f:
                json.dump({"source": os.path.abspath(ep_dir), "version": "repair_v4",
                           "mode": mode, "keep_intent": keep_intent, "fps": fps,
                           "flat_v": flat_v, "min_hold": min_hold, "speed_ref": speed_ref,
                           "source_frames": len(state), "output_frames": len(state_new)}, f, indent=2)
    speed_before = np.abs(np.diff(state, axis=0)).max(axis=1)
    speed_after = np.abs(np.diff(state_new, axis=0)).max(axis=1)
    return {"episode": os.path.basename(ep_dir), "frames": len(state), "kept": len(state_new),
            "pct": (len(state)-len(state_new))/len(state)*100,
            "speed_p50_before": float(np.percentile(speed_before, 50)),
            "speed_p50_after": float(np.percentile(speed_after, 50)),
            "speed_std_before": float(speed_before.std()), "speed_std_after": float(speed_after.std())}


def _self_test() -> int:
    """合成数据自测：保时序摩擦移除——摩擦停删、夹爪动保留、意图停保留、速度不统一。"""
    hz, T = 30.0, 5.0
    n = int(T * hz)
    t = np.arange(n) / hz
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")
        ok = ok and cond

    def longest_run(vals):
        best = cur = 1
        for i in range(1, len(vals)):
            if abs(vals[i] - vals[i - 1]) < 1e-9:
                cur += 1
                best = max(best, cur)
            else:
                cur = 1
        return best

    vel = 0.04                                   # 每步位移 rad/步（≈1.2 rad/s，>> flat_v=0.02）
    cmd = vel * np.arange(n)                     # 匀速斜坡
    cmd[(t >= 1.0) & (t < 1.5)] = cmd[int(1.0 * hz)]      # 意图停顿：cmd 也停 0.5s
    arm_state = cmd.copy()
    i0, i1 = int(3.0 * hz), int(3.6 * hz)                 # 摩擦停顿：cmd 动、arm 卡 0.6s
    arm_state[i0:i1] = arm_state[i0]
    gripper = np.zeros(n)
    gc0, gc1 = int(2.0 * hz), int(2.2 * hz)               # 夹爪闭合：arm 停但 gripper 动 0.2s
    arm_state[gc0:gc1] = arm_state[gc0]
    gripper[gc0:gc1] = 1.0
    st8 = np.stack([arm_state, gripper], axis=1)          # 8 维（1 臂 + 1 夹爪）
    arm, ee = np.array([True, False]), np.array([False, True])

    intent = _intent_hold_mask(st8, t, cmd[:, None], t, arm, flat_v=0.02, min_pause=0.1)
    check("意图停顿被识别", intent[int(1.0 * hz):int(1.5 * hz)].mean() > 0.8,
          f"覆盖={intent[int(1.0*hz):int(1.5*hz)].mean():.2f}")
    check("摩擦停顿不当意图", intent[int(3.0 * hz):int(3.6 * hz)].mean() < 0.2,
          f"覆盖={intent[int(3.0*hz):int(3.6*hz)].mean():.2f}")

    r = resample_frames(st8, np.zeros(n, np.uint8), {}, {}, {},
                        hz, 0.0, arm, ee, intent)
    stn, idx = r[0], r[4]
    # 1) 摩擦段(3.0-3.6s, 帧90-107)被删：输出仅留 1 帧（到达位）
    kept_friction = sum(1 for k in idx if 90 <= k < 108)
    check("摩擦停顿删除(18→≤2帧)", kept_friction <= 2, f"保留={kept_friction}")
    # 2) 夹爪闭合段(2.0-2.2s, 帧60-65)保留：gripper 动 → 帧全留
    kept_grip = sum(1 for k in idx if 60 <= k < 66)
    check("夹爪闭合帧保留(6帧)", kept_grip >= 5, f"保留={kept_grip}")
    # 3) 意图段(1.0-1.5s)保留时长：输出恒值 run ≈ 15 帧
    check("意图停顿保留时长(~15帧)", longest_run(stn[:, 0]) >= 12,
          f"最长恒值run={longest_run(stn[:, 0])}")
    # 4) 速度不统一：运动段 1:1 保留（0-1.0s 的 30 个斜坡帧几乎全留）
    kept_ramp = sum(1 for k in idx if k < 30)
    check("运动段保时序(30→≥28帧)", kept_ramp >= 28, f"保留={kept_ramp}")
    # 5) 零错位
    z = all((np.abs(st8 - row).max(axis=1) <= 1e-9).any() for row in stn)
    check("零错位(state∈原始帧)", z)

    # --- uniformize 模式（修好版 v2）：摩擦删、夹爪/意图保护、臂匀速化 ---
    ru = resample_frames_uniform(st8, np.zeros(n, np.uint8), {}, {}, {},
                                 vel, hz, 0.0, arm, ee, intent)
    stu, idxu = ru[0], ru[4]
    kept_friction_u = sum(1 for k in idxu if 90 <= k < 108)
    check("uniformize: 摩擦段删除(≤2帧)", kept_friction_u <= 2, f"保留={kept_friction_u}")
    kept_grip_u = sum(1 for k in idxu if 60 <= k < 66)
    check("uniformize: 夹爪闭合保留(≥5帧)", kept_grip_u >= 5, f"保留={kept_grip_u}")
    check("uniformize: 意图停顿保留", longest_run(stu[:, 0]) >= 12,
          f"run={longest_run(stu[:, 0])}")
    kept_ramp_u = sum(1 for k in idxu if k < 30)
    check("uniformize: 匀速斜坡保帧(≥28)", kept_ramp_u >= 28, f"保留={kept_ramp_u}")
    zu = all((np.abs(st8 - row).max(axis=1) <= 1e-9).any() for row in stu)
    check("uniformize: 零错位", zu)
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", help="含 episode*/aligned_data.h5 的 session")
    ap.add_argument("--out-session", help="输出新 session 目录")
    ap.add_argument("--mode", choices=["natural", "uniformize"], default="natural",
                    help="natural=保时序摩擦移除（默认，速度=自然，推荐）；"
                         "uniformize=弧长匀速化（臂按 --speed-ref 重定时，训练更快执行用，夹爪受保护）")
    ap.add_argument("--speed-ref", type=float, default=0.0,
                    help="仅 uniformize 模式：目标每帧弧长 rad/帧（0=自动取 session 臂维弧长均值；"
                         "想更快给更高值如 0.07，慢给更低如 0.03）")
    ap.add_argument("--fps", type=int, default=None, help="default: source schema fps")
    ap.add_argument("--flat-v", type=float, default=0.02,
                    help="判定'停住'的臂速度阈值 rad/s（默认 0.02；越低只删越彻底的停顿）")
    ap.add_argument("--min-hold", type=int, default=2,
                    help="连续 ≥ 该帧数(默认2)的摩擦停顿才删（更短当噪声保留）")
    ap.add_argument("--keep-intent", action="store_true", default=True,
                    help="意图感知：用 raw *_cmd 流分型停顿，操作者有意停顿（cmd 同步停）保留其"
                         "时长，只删摩擦型停顿（cmd 在动 state 被静摩擦卡住）")
    ap.add_argument("--remove-all-stalls", dest="keep_intent", action="store_false",
                    help="allow deleting stationary intervals without intent protection")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--camera-map", default="",
                    help="explicit schema camera rename, e.g. video8=base,video0=left_wrist")
    ap.add_argument("--self-test", action="store_true", help="合成数据自测")
    args = ap.parse_args()
    try:
        args.camera_map = dict(pair.split("=", 1) for pair in args.camera_map.split(",") if pair)
        if any(not a or not b for a, b in args.camera_map.items()):
            raise ValueError("empty camera name")
    except ValueError:
        ap.error("--camera-map must be comma-separated old=new pairs")

    if args.self_test:
        return _self_test()
    if not args.session:
        ap.error("--session 必填（--self-test 除外）")
    if not args.dry_run and not args.out_session:
        ap.error("--out-session 必填（--dry-run 只需 --session）")

    if not os.path.isdir(args.session):
        print(f"session 不存在: {args.session}", file=sys.stderr)
        return 1
    eps = sorted(d for d in os.listdir(args.session)
                 if d.startswith("episode") and os.path.isdir(os.path.join(args.session, d)))
    if not eps:
        print("无 episode", file=sys.stderr)
        return 1
    out_root = args.session if args.dry_run else args.out_session
    if not args.dry_run:
        reject_overlap(args.session, out_root)
        if os.path.exists(out_root):
            raise ValueError("out-session must be a new directory")
        # The entire session is replaced only after all episodes succeed.

    # uniformize 模式：speed-ref 未显式给 → 自动取 session 臂维弧长均值（总时长不变）
    if args.mode == "uniformize" and args.speed_ref <= 0:
        arm_mask0 = None
        for ep0 in eps:
            ap_ = os.path.join(args.session, ep0, "aligned_data.h5")
            if os.path.exists(ap_):
                with h5py.File(ap_) as f:
                    n_dim = f["observation/state"].shape[1]
                am, _ = _block_masks(os.path.join(args.session, ep0), n_dim)
                arm_mask0 = am
                break
        args.speed_ref = session_speed_ref(args.session, eps, arm_mask0)

    if args.dry_run:
        return _run_session(args, eps, out_root)
    with atomic_directory(out_root) as staged:
        return _run_session(args, eps, staged)


def _run_session(args, eps, out_root):
    mode_name = "保时序摩擦移除" if args.mode == "natural" else "弧长匀速化"
    print(f"修复: {mode_name} flat_v={args.flat_v} min_hold={args.min_hold} "
          f"{'speed_ref=' + ('%.4f(自动)' % args.speed_ref if args.mode == 'uniformize' else '')} "
          f"{'意图感知(保留有意停顿)' if args.keep_intent else ''} "
          f"{'[DRY-RUN]' if args.dry_run else ''}")
    tot0 = tot1 = 0
    for ep in eps:
        st = repair_episode(os.path.join(args.session, ep), os.path.join(out_root, ep),
                            args.fps, args.dry_run, args.keep_intent,
                            args.flat_v, args.min_hold, args.mode, args.speed_ref,
                            camera_map=args.camera_map or None)
        tot0 += st.get("frames", 0); tot1 += st.get("kept", 0)
        if "error" in st:
            print(f"  {ep}: {st['error']}")
            continue
        print(f"  {ep}: {st['frames']}→{st['kept']} 帧 ({st['pct']:.0f}%)  "
              f"速度std {st['speed_std_before']:.3f}→{st['speed_std_after']:.3f}")
    if tot0:
        print(f"\n合计: {tot0}→{tot1} 帧 (-{(tot0-tot1)/tot0*100:.1f}%)")
    print(f"{'输出到 ' + args.out_session if not args.dry_run else 'dry-run 结束，未写文件'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
