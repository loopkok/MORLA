"""Rerun 可视化回放：加载 aligned_data.h5，图像 + 关节曲线 + 时间轴拖动。

用法：
  python3 replay_rerun.py --session ~/astral_data/pick_place            # 全部段
  python3 replay_rerun.py --session ~/astral_data/pick_place --episode 3
  python3 replay_rerun.py --episode-dir ~/astral_data/pick_place/episode000003

依赖：pip install rerun-sdk（仅回放需要，采集/对齐/导出不依赖）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any

import h5py
import numpy as np

from morla_data_collect.pipeline.align_data import ALIGNED_H5
from morla_data_collect.pipeline.artifacts import require_source_signature


def _decode(jpeg: np.ndarray) -> np.ndarray | None:
    import cv2

    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    return None if img is None else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def log_episode(episode_dir: str, *, log=print) -> None:
    import rerun as rr

    aligned = os.path.join(episode_dir, ALIGNED_H5)
    if not os.path.exists(aligned):
        log(f"  skip (no {ALIGNED_H5}, run align_data first): {episode_dir}")
        return
    with open(os.path.join(episode_dir, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)

    # 注意：图像按帧惰性读取，文件句柄必须活到逐帧循环结束
    with h5py.File(aligned, "r") as f:
        require_source_signature(f, episode_dir)
        state = f["observation/state"][:]
        action = f["action"][:]
        ts = f["timestamps"][:]
        quality = f["quality"][:]
        fps = float(f.attrs["fps"])
        ep_index = int(f.attrs.get("episode_index", meta.get("episode_index", 0)))
        state_names = json.loads(f.attrs.get("state_names", "[]"))
        schema = json.loads(f.attrs.get("schema", "{}"))
        cams = [c for c in schema.get("cameras", []) if f"{c}/images" in f]
        cam_imgs = {c: f[f"{c}/images"] for c in cams}

        root = f"episode_{ep_index:06d}"
        task = str(meta.get("task", ""))
        n = len(ts)
        t0 = float(ts[0]) if n else 0.0

        from morla_data_collect.schema import pipeline_schema_from_dict, action_blocks, action_names
        view = pipeline_schema_from_dict(schema)
        layouts = {"state": view.state_blocks(), "action": action_blocks(view)}
        names_by_kind = {"state": view.state_names(), "action": action_names(view)}
        columns = {}
        for kind, blocks in layouts.items():
            offset = 0
            columns[kind] = []
            for block in blocks:
                indices = list(getattr(block, "indices", ()) or range(offset, offset + block.dim))
                columns[kind].append((block.name, indices))
                names = [names_by_kind[kind][j] for j in indices]
                rr.log(f"{root}/{kind}/{block.name}", rr.SeriesLines(names=names), static=True)
                offset += block.dim
        rr.log(f"{root}/task", rr.TextDocument(f"# episode {ep_index:06d}\n\n{task}"), static=True)
        log(f"  logging {root}: {n} frames @ {fps}Hz, cams={cams}, task={task!r}")

        for i in range(n):
            # rerun >= 0.24: set_time_seconds 废弃，改用 set_time(timestamp=秒)
            rr.set_time("episode_time", timestamp=float(ts[i]) - t0)
            for cam, imgs in cam_imgs.items():
                img = _decode(imgs[i])
                if img is not None:
                    rr.log(f"{root}/camera/{cam}", rr.Image(img))
            for kind, values in (("state", state), ("action", action)):
                for name, indices in columns[kind]:
                    rr.log(f"{root}/{kind}/{name}", rr.Scalars(values[i, indices].tolist()))
            if quality[i]:
                rr.log(
                    f"{root}/quality",
                    rr.TextLog(f"frame {i}: flags=0x{int(quality[i]):02x}", level="WARN"),
                )


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--session", help="session 目录（回放全部或指定段）")
    src.add_argument("--episode-dir", help="单个 episode 目录")
    ap.add_argument("--episode", type=int, default=None, help="只回放指定段号")
    ap.add_argument("--save", default="", help="另存为 .rrd 文件（不 spawn viewer）")
    args = ap.parse_args(argv)

    try:
        import rerun as rr
    except ImportError:
        print("rerun-sdk 未安装：/usr/bin/python3 -m pip install rerun-sdk", file=sys.stderr)
        sys.exit(2)

    rr.init("astral_data_replay", spawn=not bool(args.save))
    if args.save:
        rr.save(args.save)

    if args.episode_dir:
        dirs = [os.path.expanduser(args.episode_dir)]
    else:
        session = os.path.expanduser(args.session)
        dirs = sorted(
            os.path.join(session, d)
            for d in os.listdir(session)
            if d.startswith("episode") and os.path.isdir(os.path.join(session, d))
        )
        if args.episode is not None:
            dirs = [d for d in dirs if d.endswith(f"{args.episode:06d}")]
            if not dirs:
                print(f"episode {args.episode} not found under {session}", file=sys.stderr)
                sys.exit(1)

    for d in dirs:
        log_episode(d)
    print("done. 在 viewer 时间轴（episode_time）上拖动回放。")


if __name__ == "__main__":
    main(sys.argv[1:])
