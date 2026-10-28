"""清洗校验：逐 episode 体检，产出报告并可隔离问题段。

规则（级别：fail=必须处理，warn=人工过目）：
  F1 fail  文件缺失/不可读（robot_data.h5 / camera_data.h5 / meta.json）
  F2 fail  schema 要求的流缺失或为空（配置与硬件不一致）
  F3 fail  数值流含 NaN/Inf
  F4 fail  时间戳非严格递增
  F5 fail  episode 时长 < min_duration_s（默认 2s）
  F6 fail  raw/aligned 维数、长度、schema 或数值完整性错误
  F7 fail  严重对齐缺口（超过 20% 或连续超过 1s）
  W1 warn  流实测频率低于期望下限（arm 45Hz / hand 100Hz / gripper 5Hz / 图像 0.8*fps）
  W2 warn  采样空洞 > max_gap_ms（按次数报告）
  W3 warn  关节相邻帧跳变 |Δ| > 0.5 rad（遥操限速 4rad/s@30fps → 0.5 为硬异常）
  W4 warn  图像帧 JPEG 解码失败（首/中/尾抽检）
  W5 warn  armed 覆盖率 < 50%（按事件时间线推算；无事件则跳过）
  W6 warn  对齐帧图像时间偏差 p95 > 1.5/fps（需已跑 align）
  W7 warn  旧版 armed/disarm 快照矛盾，覆盖率无法可靠计算

用法：
  python3 validate_data.py --session ~/astral_data/pick_place [--apply]
  --apply：把 fail 的 episode 移入 session/quarantine/（移动，不删除）
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from typing import Any

import h5py
import numpy as np

from morla_data_collect.pipeline.align_data import ALIGNED_H5
from morla_data_collect.data_writer import CAMERA_H5, META_JSON, ROBOT_H5
from morla_data_collect.schema import pipeline_schema_from_dict, pipeline_schema_from_json

QUARANTINE_DIR = "quarantine"
REPORT_JSON = "validation_report.json"

# 各流实测频率下限（Hz）——低于则 W1
EXPECTED_MIN_HZ = {
    "body_state": 45.0,
    "left_arm_state": 45.0, "right_arm_state": 45.0,
    "left_arm_cmd": 45.0, "right_arm_cmd": 45.0,
    "left_hand_state": 100.0, "right_hand_state": 100.0,
    "left_hand_cmd": 100.0, "right_hand_cmd": 100.0,
    "left_gripper_ratio": 5.0, "right_gripper_ratio": 5.0,
    "left_gripper_rad": 5.0, "right_gripper_rad": 5.0,
    "head_state": 20.0, "head_cmd": 20.0,
}
MAX_JUMP_RAD = 0.5
MIN_DURATION_S = 2.0


class _Report:
    def __init__(self, episode_dir: str) -> None:
        self.episode_dir = episode_dir
        self.issues: list[dict[str, str]] = []

    def fail(self, rule: str, msg: str) -> None:
        self.issues.append({"level": "fail", "rule": rule, "msg": msg})

    def warn(self, rule: str, msg: str) -> None:
        self.issues.append({"level": "warn", "rule": rule, "msg": msg})

    @property
    def status(self) -> str:
        levels = {i["level"] for i in self.issues}
        return "fail" if "fail" in levels else ("warn" if "warn" in levels else "pass")

    def to_dict(self) -> dict[str, Any]:
        return {
            "episode": os.path.basename(self.episode_dir),
            "status": self.status,
            "issues": self.issues,
        }


def _stream_duration(ts: np.ndarray) -> float:
    return float(ts[-1] - ts[0]) if len(ts) > 1 else 0.0


def _check_stream(rep: _Report, name: str, ts: np.ndarray, values: np.ndarray,
                  max_gap_s: float) -> None:
    if len(ts) == 0:
        rep.fail("F2", f"stream {name} empty")
        return
    if ts.ndim != 1 or values.ndim != 2 or len(values) != len(ts):
        rep.fail("F6", f"stream {name} timestamps/value shape mismatch")
        return
    if np.any(~np.isfinite(ts)):
        rep.fail("F4", f"stream {name} timestamps contain NaN/Inf")
        return
    if np.any(~np.isfinite(values)):
        rep.fail("F3", f"stream {name} contains NaN/Inf")
    if len(ts) > 1 and np.any(np.diff(ts) <= 0):
        rep.fail("F4", f"stream {name} timestamps not strictly increasing")
    dur = _stream_duration(ts)
    if dur > 0.5:
        hz = len(ts) / dur
        low = EXPECTED_MIN_HZ.get(name)
        if low is not None and hz < low:
            rep.warn("W1", f"stream {name} rate {hz:.1f}Hz < {low}Hz")
        if len(ts) > 1:
            gaps = np.diff(ts)
            n_gap = int(np.count_nonzero(gaps > max_gap_s))
            if n_gap:
                rep.warn(
                    "W2",
                    f"stream {name}: {n_gap} gaps > {max_gap_s * 1000:.0f}ms "
                    f"(max {gaps.max() * 1000:.0f}ms)",
                )
        if len(ts) > 2 and values.ndim == 2 and values.shape[1] > 0:
            jump = np.max(np.abs(np.diff(values, axis=0)))
            if np.isfinite(jump) and jump > MAX_JUMP_RAD:
                rep.warn("W3", f"stream {name} max frame jump {jump:.2f} rad")


def _armed_coverage(events: list[dict], t0: float, t1: float) -> float | None:
    """从事件时间线推 teleop_armed 覆盖比例；无事件返回 None。"""
    ev = sorted(
        (e for e in events if e.get("name") in ("teleop_armed", "teleop_disarmed")),
        key=lambda e: e["t"],
    )
    if not ev:
        return None
    initial = [e for e in ev if abs(float(e["t"]) - t0) < 0.01 and bool(e.get("data"))]
    if {e["name"] for e in initial} == {"teleop_armed", "teleop_disarmed"}:
        return None  # Legacy snapshots contain two conflicting persistent flags.
    armed = False
    last = t0
    acc = 0.0
    for e in ev:
        t = min(max(float(e["t"]), t0), t1)
        if armed:
            acc += t - last
        armed = bool(e["data"]) if e["name"] == "teleop_armed" else not bool(e["data"])
        # teleop_armed False / teleop_disarmed True 都表示撤防
        if e["name"] == "teleop_armed":
            armed = bool(e["data"])
        else:
            armed = not bool(e["data"])
        last = t
    if armed:
        acc += t1 - last
    total = max(t1 - t0, 1e-9)
    return acc / total


def _check_aligned(rep: _Report, f: h5py.File, schema) -> None:
    required = ["observation/state", "action", "timestamps", "quality"]
    missing = [k for k in required if k not in f]
    if missing:
        rep.fail("F6", f"aligned missing datasets: {missing}")
        return
    ts, state, action = f["timestamps"][:], f["observation/state"][:], f["action"][:]
    n = len(ts)
    label_keys = ('control_owner', 'intervention', 'intervention_valid')
    if any(key in f for key in label_keys):
        if not all(key in f and f[key].shape == (n,) for key in label_keys):
            rep.fail('F6', 'incomplete or mismatched intervention labels')
        else:
            owner = f['control_owner'][:]
            intervention = f['intervention'][:]
            valid = f['intervention_valid'][:]
            if (not np.isin(owner, range(6)).all()
                    or not np.array_equal(intervention, owner == 2)
                    or not np.array_equal(valid, np.isin(owner, [1, 2, 3, 5]))):
                rep.fail('F6', 'invalid intervention/owner labels')
    if n < 2 or ts.ndim != 1 or not np.all(np.isfinite(ts)):
        rep.fail("F6", "aligned timestamps invalid/too short")
    elif not np.allclose(np.diff(ts), 1 / schema.dataset_fps, rtol=0, atol=1e-5):
        rep.fail("F6", "aligned timestamps are not a uniform fps grid")
    for key, value in [("state", state), ("action", action)]:
        if value.shape != (n, schema.state_dim if key == "state" else schema.action_dim) or not np.all(np.isfinite(value)):
            rep.fail("F6", f"aligned {key} invalid shape or NaN/Inf")
    for attr in ("fps", "schema", "state_names"):
        if attr not in f.attrs:
            rep.fail("F6", f"aligned missing attribute {attr}; rebuild from raw")
    if "fps" in f.attrs and f.attrs["fps"] != schema.dataset_fps:
        rep.fail("F6", "aligned fps differs from meta schema")
    if "schema" in f.attrs:
        other = pipeline_schema_from_json(f.attrs["schema"])
        if training_contract(other) != training_contract(schema):
            rep.fail("F6", "aligned schema differs from meta schema")
    if state.shape == (n, schema.state_dim) and action.shape == (n, schema.action_dim) and n > 0:
        from morla_data_collect.schema import rebuild_actions
        expected = rebuild_actions(schema, state, action)
        if not np.allclose(action, expected, atol=1e-6, rtol=0, equal_nan=False):
            rep.fail("F6", "aligned next_state action relationship or terminal hold is broken")
    q = f["quality"][:]
    if q.shape != (n,):
        rep.fail("F6", "aligned quality length mismatch")
    else:
        bad = (q & 3) != 0  # Synthetic terminal hold is a separate bit.
        longest = current = 0
        for value in bad:
            current = current + 1 if value else 0
            longest = max(longest, current)
        if bad.mean() > 0.2 or longest / schema.dataset_fps > 1.0:
            rep.fail("F7", f"aligned severe gaps: {bad.mean():.1%}, longest {longest/schema.dataset_fps:.2f}s")
    for cam in schema.cameras:
        for key in ("images", "src_timestamps", "src_offsets"):
            full = f"{cam}/{key}"
            if full not in f or len(f[full]) != n:
                rep.fail("F6", f"aligned {full} missing/count mismatch")
        if f"{cam}/src_timestamps" in f:
            source = f[f"{cam}/src_timestamps"][:]
            if not np.all(np.isfinite(source)) or np.any(source <= 0):
                rep.fail("F6", f"aligned {cam} source timestamps invalid")
        if f"{cam}/src_offsets" in f and not np.all(np.isfinite(f[f"{cam}/src_offsets"][:])):
            rep.fail("F6", f"aligned {cam} source offsets invalid")
    for sensor in getattr(schema, "sensor_specs", ()):
        key = str(sensor.get("sensor_key", ""))
        required = bool(sensor.get("required", False))
        sensor_root = f.get("sensors")
        if sensor_root is None or key not in sensor_root:
            if required:
                rep.fail("F2", f"aligned required sensor {key} missing")
            else:
                rep.warn("W9", f"optional aligned sensor {key} missing; tactile features are unavailable")
            continue
        group = sensor_root[key]
        npoints = int(sensor.get("point_count_per_finger", 120))
        n_fingers = len(sensor.get("finger_order", ("thumb", "index", "middle", "ring", "pinky")))
        from morla_data_collect.codecs import sensor_fields
        expected = {name: (n,) + shape for name, (shape, dtype) in sensor_fields(sensor).items()}
        expected.update({"source_index": (n,), "source_timestamps": (n,), "source_offsets": (n,)})
        for name, shape in expected.items():
            if name not in group or group[name].shape != shape:
                rep.fail("F6", f"aligned sensor {key}/{name} missing/shape mismatch")
        if "valid" in group:
            valid = group["valid"][:].astype(bool)
            temp_valid = (group["temperature_valid"][:].astype(bool)
                          if "temperature_valid" in group else np.zeros_like(valid))
            for field in ("force_raw", "force_sum", "temperature_raw", "temperature_mean"):
                if field in group and np.issubdtype(group[field].dtype, np.number):
                    values = group[field][:]
                    mask = temp_valid if field.startswith("temperature") else valid
                    while mask.ndim < values.ndim:
                        mask = mask[..., None]
                    if values.size and np.any(~np.isfinite(values[np.broadcast_to(mask, values.shape)])):
                        rep.fail("F3", f"aligned sensor {full} has NaN/Inf in valid samples")
                        break


def training_contract(schema) -> dict:
    frozen = getattr(schema, "frozen", None)
    if frozen is not None:
        return {"profile_id": frozen.profile_id, "layout_hash": frozen.layout_hash,
                "schema_hash": frozen.schema_hash, "names": schema.state_names(),
                "fps": schema.dataset_fps, "cameras": schema.cameras,
                "action_source": schema.action_source}
    return {"names": schema.state_names(), "fps": schema.dataset_fps,
            "cameras": schema.cameras, "action_source": schema.action_source}


def ensure_training_ready(session_dir: str) -> list[dict]:
    """Read-only hard gate shared by every exporter; no implicit dropped episodes."""
    reports = [validate_episode(os.path.join(session_dir, name), min_duration_s=0,
                                require_aligned=True).to_dict()
               for name in sorted(os.listdir(session_dir))
               if name.startswith("episode") and os.path.isdir(os.path.join(session_dir, name))]
    if not reports:
        raise RuntimeError("no aligned episodes available")
    failures = [r for r in reports if r["status"] == "fail"]
    if failures:
        raise RuntimeError("training validation failed: " + "; ".join(
            r["episode"] + ": " + ", ".join(i["msg"] for i in r["issues"] if i["level"] == "fail")
            for r in failures))
    return reports


def validate_episode(
    episode_dir: str,
    *,
    min_duration_s: float = MIN_DURATION_S,
    check_aligned: bool = True,
    require_aligned: bool = False,
) -> _Report:
    rep = _Report(episode_dir)

    robot_h5 = os.path.join(episode_dir, ROBOT_H5)
    camera_h5 = os.path.join(episode_dir, CAMERA_H5)
    meta_json = os.path.join(episode_dir, META_JSON)
    for p in (robot_h5, camera_h5, meta_json):
        if not os.path.exists(p):
            rep.fail("F1", f"missing {os.path.basename(p)}")
    if any(i["rule"] == "F1" for i in rep.issues):
        return rep

    try:
        with open(meta_json, encoding="utf-8") as f:
            meta = json.load(f)
        schema = pipeline_schema_from_dict(meta["schema"])
    except Exception as exc:
        rep.fail("F1", f"meta.json unreadable: {exc}")
        return rep

    if meta.get("recording_status") not in {None, "complete"}:
        rep.fail("F1", f"recording is not complete: {meta.get('recording_status')}")

    if meta.get("legacy_validity_unknown"):
        rep.warn("W10", "legacy XNero cache validity/command timing is unknown; review provenance before training")

    max_gap_s = float(schema.max_gap_ms) / 1000.0

    try:
        with h5py.File(robot_h5, "r") as fr, h5py.File(camera_h5, "r") as fc:
            # -- 数值流 ------------------------------------------------------------
            for name, dim in schema.required_streams().items():
                alternative = (
                    (name == "body_state" and getattr(schema, "include_head", False)
                     and not getattr(schema, "include_waist", False)
                     and "head_state" in fr["streams"] and len(fr["streams/head_state/timestamps"]) > 0)
                    or (name in ("head_state", "head_cmd") and "body_state" in fr["streams"]
                        and len(fr["streams/body_state/timestamps"]) > 0)
                )
                if alternative and (name not in fr["streams"] or len(fr[f"streams/{name}/timestamps"]) == 0):
                    continue
                if name not in fr["streams"]:
                    rep.fail("F2", f"stream {name} missing")
                    continue
                ts = fr[f"streams/{name}/timestamps"][:]
                values = fr[f"streams/{name}/values"][:]
                if values.ndim == 2 and values.shape[1] != dim and len(ts) > 0:
                    rep.fail("F6", f"stream {name} dim {values.shape[1]} != schema {dim}")
                _check_stream(rep, name, ts, values, max_gap_s)
                group = fr[f"streams/{name}"]
                masks = {key: group[key][:] for key in ('valid', 'measured', 'synthetic', 'command_echo') if key in group}
                if any(mask.shape != (len(ts),) for mask in masks.values()):
                    rep.fail('F6', f'stream {name}: provenance count mismatch')
                elif masks:
                    measured = masks.get('measured', np.zeros(len(ts), bool))
                    synthetic = masks.get('synthetic', np.zeros(len(ts), bool))
                    echo = masks.get('command_echo', np.zeros(len(ts), bool))
                    if np.any(measured & (synthetic | echo)):
                        rep.fail('F6', f'stream {name}: contradictory measured provenance')
                    if 'valid' in masks and len(ts) and not masks['valid'].all():
                        rep.fail('F7', f'stream {name}: invalid required samples in provenance')


            # Non-joint profile streams have dedicated sidecar storage and never
            # participate in the model state/action vector.
            for sensor in getattr(schema, "sensor_specs", ()):
                key = str(sensor.get("sensor_key", ""))
                required = bool(sensor.get("required", False))
                path = f"sensors/{key}"
                if path not in fr:
                    if required:
                        rep.fail("F2", f"required sensor {key} missing")
                    else:
                        rep.warn("W9", f"optional sensor {key} missing; tactile features are unavailable")
                    continue
                group = fr[path]
                fingers = len(sensor.get("finger_order", ("thumb", "index", "middle", "ring", "pinky")))
                points = int(sensor.get("point_count_per_finger", 120))
                from morla_data_collect.codecs import sensor_fields
                expected = {name: shape for name, (shape, dtype) in sensor_fields(sensor).items()}
                ts = group["timestamps"][:] if "timestamps" in group else np.empty(0)
                for field, shape in expected.items():
                    if field not in group or group[field].shape != (len(ts),) + shape:
                        rep.fail("F6", f"sensor {key}/{field} missing/shape mismatch")
                if "arrival_timestamps" not in group or group["arrival_timestamps"].shape != (len(ts),):
                    rep.fail("F6", f"sensor {key}/arrival_timestamps missing/count mismatch")
                if len(ts) == 0 and required:
                    rep.fail("F2", f"required sensor {key} empty")
                elif len(ts) == 0:
                    rep.warn("W9", f"optional sensor {key} has no samples; tactile features are unavailable")
                if len(ts) and (np.any(~np.isfinite(ts)) or np.any(np.diff(ts) <= 0)):
                    rep.fail("F4", f"sensor {key} timestamps invalid/not strictly increasing")
                if len(ts) > 1 and np.isfinite(ts).all():
                    gaps = np.diff(ts)
                    if np.count_nonzero(gaps > max_gap_s):
                        rep.warn("W2", f"sensor {key}: {np.count_nonzero(gaps > max_gap_s)} gaps > {max_gap_s * 1000:.0f}ms")
                if "valid" in group:
                    valid = group["valid"][:].astype(bool)
                    for field in ("force_raw", "force_sum", "temperature_raw", "temperature_mean"):
                        if field not in group:
                            continue
                        values = group[field][:]
                        mask = (group["temperature_valid"][:].astype(bool)
                                if field.startswith("temperature") and "temperature_valid" in group
                                else valid)
                        while mask.ndim < values.ndim:
                            mask = mask[..., None]
                        if values.size and np.any(~np.isfinite(values[np.broadcast_to(mask, values.shape)])):
                            rep.fail("F3", f"sensor {key}/{field} has NaN/Inf in valid samples")

            # -- 相机流 ------------------------------------------------------------
            for cam in schema.cameras:
                if cam not in fc or len(fc[f"{cam}/timestamps"]) == 0:
                    rep.fail("F2", f"camera {cam} missing/empty")
                    continue
                ts = fc[f"{cam}/timestamps"][:]
                if np.any(~np.isfinite(ts)):
                    rep.fail("F4", f"camera {cam} timestamps contain NaN/Inf")
                    continue
                if len(fc[f"{cam}/images"]) != len(ts):
                    rep.fail("F6", f"camera {cam} frame/timestamp count mismatch")
                    continue
                if len(ts) > 1 and np.any(np.diff(ts) <= 0):
                    rep.fail("F4", f"camera {cam} timestamps not strictly increasing")
                dur = _stream_duration(ts)
                if dur > 0.5:
                    hz = len(ts) / dur
                    if hz < 0.8 * schema.dataset_fps:
                        rep.warn(
                            "W1",
                            f"camera {cam} rate {hz:.1f}Hz < {0.8 * schema.dataset_fps:.1f}Hz",
                        )
                # JPEG 解码抽检（首/中/尾）
                try:
                    import cv2

                    n = len(ts)
                    for i in {0, n // 2, n - 1}:
                        raw = fc[f"{cam}/images"][int(i)]
                        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                        if img is None:
                            rep.warn("W4", f"camera {cam} frame {i} JPEG undecodable")
                except ImportError:
                    pass  # 无 cv2 时跳过抽检

            # -- 时长 ---------------------------------------------------------------
            dur = float(meta.get("duration_s", 0.0))
            if dur < min_duration_s:
                rep.fail("F5", f"duration {dur:.2f}s < {min_duration_s}s")

            # -- armed 覆盖率 ---------------------------------------------------------
            cov = _armed_coverage(
                meta.get("events", []),
                float(meta.get("start_time_wall", 0.0)),
                float(meta.get("end_time_wall", 0.0)),
            )
            initial = [e for e in meta.get("events", [])
                       if abs(float(e.get("t", 0)) - float(meta.get("start_time_wall", 0))) < 0.01
                       and bool(e.get("data"))]
            if cov is None and {e.get("name") for e in initial}.issuperset(
                    {"teleop_armed", "teleop_disarmed"}):
                rep.warn("W7", "legacy armed/disarm snapshot conflicts; coverage unknown")
            if cov is not None and cov < 0.5:
                rep.warn("W5", f"teleop armed coverage {cov * 100:.0f}% < 50%")

        # -- 对齐产物（可选） ---------------------------------------------------------
        aligned = os.path.join(episode_dir, ALIGNED_H5)
        if require_aligned and not os.path.exists(aligned):
            rep.fail("F6", "missing aligned_data.h5")
        if check_aligned and os.path.exists(aligned):
            with h5py.File(aligned, "r") as fa:
                _check_aligned(rep, fa, schema)
                from morla_data_collect.pipeline.artifacts import require_source_signature
                try:
                    require_source_signature(fa, episode_dir)
                except ValueError as exc:
                    rep.fail("F6", str(exc))
                state = fa["observation/state"][:]
                if np.any(~np.isfinite(state)):
                    missing = json.loads(fa.attrs.get("missing_blocks", "[]"))
                    if missing:
                        rep.fail("F3", f"aligned state NaN (missing blocks: {missing})")
                    else:
                        rep.fail("F3", "aligned state contains NaN/Inf")
                fps = float(fa.attrs.get("fps", schema.dataset_fps))
                for cam in schema.cameras:
                    key = f"{cam}/source_alignment_offsets" if f"{cam}/source_alignment_offsets" in fa else f"{cam}/src_offsets"
                    if key in fa:
                        off = np.abs(fa[key][:])
                        if "quality" in fa and fa["quality"].shape == off.shape:
                            off = off[(fa["quality"][:] & 4) == 0]
                        if len(off):
                            p95 = float(np.percentile(off, 95))
                            if p95 > 1.5 / fps:
                                rep.warn(
                                    "W6",
                                    f"camera {cam} align offset p95 {p95 * 1000:.1f}ms "
                                    f"> {1.5 / fps * 1000:.1f}ms",
                                )
    except Exception as exc:
        rep.fail("F1", f"h5 read error: {exc}")
    return rep


def validate_session(
    session_dir: str,
    *,
    apply: bool = False,
    min_duration_s: float = MIN_DURATION_S,
    log=print,
) -> dict[str, Any]:
    session_dir = os.path.abspath(session_dir)
    episode_dirs = sorted(
        os.path.join(session_dir, d)
        for d in os.listdir(session_dir)
        if d.startswith("episode")
        and os.path.isdir(os.path.join(session_dir, d))
    )
    reports = []
    for ep in episode_dirs:
        rep = validate_episode(ep, min_duration_s=min_duration_s)
        reports.append(rep)
        line = f"  [{rep.status.upper():4s}] {os.path.basename(ep)}"
        if rep.issues:
            line += ": " + "; ".join(
                f"{i['rule']} {i['msg']}" for i in rep.issues[:4]
            )
            if len(rep.issues) > 4:
                line += f" … (+{len(rep.issues) - 4})"
        log(line)

    n_fail = sum(1 for r in reports if r.status == "fail")
    n_warn = sum(1 for r in reports if r.status == "warn")
    n_pass = sum(1 for r in reports if r.status == "pass")

    if apply:
        qdir = os.path.join(session_dir, QUARANTINE_DIR)
        for rep in reports:
            if rep.status != "fail":
                continue
            os.makedirs(qdir, exist_ok=True)
            dst = os.path.join(qdir, os.path.basename(rep.episode_dir))
            shutil.move(rep.episode_dir, dst)
            log(f"  quarantined -> {dst}")

    summary = {
        "session": session_dir,
        "total": len(reports),
        "pass": n_pass,
        "warn": n_warn,
        "fail": n_fail,
        "quarantined": n_fail if apply else 0,
        "episodes": [r.to_dict() for r in reports],
    }
    with open(os.path.join(session_dir, REPORT_JSON), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    log(
        f"summary: {n_pass} pass / {n_warn} warn / {n_fail} fail"
        + (f" (fail moved to {QUARANTINE_DIR}/)" if apply else "")
        + f" — report: {os.path.join(session_dir, REPORT_JSON)}"
    )
    return summary


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session", required=True, help="session 目录（含 episode*）")
    ap.add_argument("--apply", action="store_true",
                    help="把 fail 的 episode 移入 quarantine/（移动不删除）")
    ap.add_argument("--min-duration", type=float, default=MIN_DURATION_S)
    args = ap.parse_args(argv)
    summary = validate_session(
        os.path.expanduser(args.session),
        apply=args.apply,
        min_duration_s=args.min_duration,
    )
    sys.exit(1 if summary["fail"] and not args.apply else 0)


if __name__ == "__main__":
    main(sys.argv[1:])
