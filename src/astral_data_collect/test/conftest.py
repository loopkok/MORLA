"""测试公共工具：构造合成的 raw episode（robot_data.h5 + camera_data.h5 + meta.json）。"""

import json
import os
import sys

import cv2
import h5py
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.schema import CollectSchema  # noqa: E402


def make_jpeg(color=(32, 128, 224), w=64, h=48):
    img = np.full((h, w, 3), color, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


def write_raw_episode(
    episode_dir: str,
    *,
    schema: CollectSchema | None = None,
    duration_s: float = 3.0,
    fps: int = 30,
    joint_hz: float = 100.0,
    task: str = "pick up the cube",
    nan_in: str | None = None,
    gap_in: str | None = None,
    drop_stream: str | None = None,
    events: list | None = None,
) -> str:
    """生成一段结构完整的 raw episode；可用参数注入各类缺陷。"""
    schema = schema or CollectSchema(
        end_effector_left="gripper", end_effector_right="gripper",
        cameras=["wrist_left", "wrist_right"], dataset_fps=fps,
    )
    os.makedirs(episode_dir, exist_ok=True)
    t0 = 1_700_000_000.0

    # --- camera_data.h5：严格 fps 帧率 -----------------------------------------
    n_cam = int(duration_s * fps)
    with h5py.File(os.path.join(episode_dir, "camera_data.h5"), "w") as fc:
        vlen_u8 = h5py.special_dtype(vlen=np.dtype("uint8"))
        for ci, cam in enumerate(schema.cameras):
            g = fc.create_group(cam)
            ds = g.create_dataset("images", shape=(n_cam,), dtype=vlen_u8)
            color = (30 + 60 * ci, 100, 200 - 30 * ci)
            jpeg = make_jpeg(color=color)
            for i in range(n_cam):
                ds[i] = np.frombuffer(jpeg, dtype=np.uint8)
            g.create_dataset(
                "timestamps",
                data=t0 + np.arange(n_cam, dtype=np.float64) / fps,
            )

    # --- robot_data.h5 -----------------------------------------------------------
    streams = schema.required_streams()
    with h5py.File(os.path.join(episode_dir, "robot_data.h5"), "w") as fr:
        grp = fr.create_group("streams")
        for name, dim in streams.items():
            if name == drop_stream:
                continue
            hz = 1000.0 if "hand" in name else joint_hz
            n = int(duration_s * hz)
            ts = t0 + np.arange(n, dtype=np.float64) / hz
            if gap_in == name and n > 30:
                ts[20:] += 0.5  # 注入 0.5s 空洞
            values = np.zeros((n, dim), dtype=np.float32)
            # 缓变正弦，便于对齐后验证插值正确性
            for d in range(dim):
                values[:, d] = 0.1 * d + np.sin(
                    2 * np.pi * 0.5 * (ts - t0) + d
                ).astype(np.float32)
            if nan_in == name and n > 5:
                values[5, 0] = np.nan
            g = grp.create_group(name)
            g.create_dataset("values", data=values)
            g.create_dataset("timestamps", data=ts)

    meta = {
        "package": "astral_data_collect",
        "format": "raw_hdf5_v1",
        "task": task,
        "schema": schema.to_dict(),
        "episode_index": int(episode_dir.rstrip("/").split("episode")[-1] or 0),
        "start_time_wall": t0,
        "end_time_wall": t0 + duration_s,
        "duration_s": duration_s,
        "events": events if events is not None else [
            {"t": t0, "name": "episode_start", "data": True},
            {"t": t0, "name": "teleop_armed", "data": True},
            {"t": t0 + duration_s, "name": "episode_end", "data": True},
        ],
        "streams": {k: {"dim": d, "count": 100} for k, d in streams.items()},
        "cameras": {c: {"count": n_cam} for c in schema.cameras},
        "dropped": {},
    }
    with open(os.path.join(episode_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f)
    return episode_dir


@pytest.fixture()
def session_dir(tmp_path):
    """含两段健康 episode 的 session 目录。"""
    root = tmp_path / "session"
    write_raw_episode(str(root / "episode000000"))
    write_raw_episode(str(root / "episode000001"), task="put down the cube")
    return str(root)
