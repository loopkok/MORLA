#!/usr/bin/env python3
"""迁移采集数据相机键名：video8/video0/video2 → base/left_wrist/right_wrist（语义名）。

旧数据（streamer label_aliases 语义化之前录的）用内核 videoN 作相机键；改名后与新配置
（quest3_video_streamer label_aliases → base/left_wrist/right_wrist）一致，可被新
camera_map（openpi / policy_inference）直接消费。

处理的格式（parquet 无图像列，图像全在 videos 目录 + meta/info.json features）：
  * raw   <session>/episode*/camera_data.h5    组名 video0/video8
  * raw   <session>/episode*/aligned_data.h5   顶层键 video0/video8
  * v2.1  <dir>/videos/{chunk}/observation.images.{label} + meta/info.json features
  * v3    <dir>/videos/observation.images.{label}/{chunk} + meta/info.json features

用法：
  # raw 迁移（逐个 episode 的 camera_data.h5 + aligned_data.h5）
  bash MORLA/scripts/data_pipeline.sh migrate-cameras --raw ~/astral_data/raw/pick_place_merged
  # v2.1 / v3 数据集迁移
  bash MORLA/scripts/data_pipeline.sh migrate-cameras \
      --v21 ~/astral_data/pi/pick_place_merged --v3 ~/astral_data/act/pick_place_merged
  # 全部 + 干跑预览（不实际改名）
  bash MORLA/scripts/data_pipeline.sh migrate-cameras \
      --raw ... --v21 ... --v3 ... --dry-run
"""
from __future__ import annotations

import argparse
import json
import os
import sys

# 旧名 -> 语义名
RENAMES = [("video8", "base"), ("video0", "left_wrist"), ("video2", "right_wrist")]


def _apply_renames(name: str) -> str | None:
    """子串替换；命中任一旧名返回新名，否则 None。"""
    out = name
    hit = False
    for old, new in RENAMES:
        if old in out:
            out = out.replace(old, new)
            hit = True
    return out if hit else None


def rename_h5_keys(h5_path: str, dry: bool) -> int:
    """h5 顶层键名重命名（组名），返回改名数。"""
    import h5py

    n = 0
    with h5py.File(h5_path, "r+") as f:
        for key in list(f.keys()):
            new = _apply_renames(key)
            if new and new != key and new not in f:
                if dry:
                    print(f"    [dry] {h5_path}: {key} -> {new}")
                else:
                    f.move(key, new)
                n += 1
    return n


def rename_info_json(dataset: str, dry: bool) -> int:
    """meta/info.json 的 features 键重命名，返回改名数。"""
    path = os.path.join(dataset, "meta", "info.json")
    if not os.path.isfile(path):
        print(f"  ! 无 {path}，跳过")
        return 0
    with open(path, encoding="utf-8") as f:
        info = json.load(f)
    features = info.get("features", {})
    n = 0
    for key in list(features.keys()):
        new = _apply_renames(key)
        if new and new != key and new not in features:
            if dry:
                print(f"    [dry] info.json: {key} -> {new}")
            else:
                features[new] = features.pop(key)
            n += 1
    if n and not dry:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)
            f.write("\n")
    return n


def rename_videos(dataset: str, v3: bool, dry: bool) -> int:
    """videos 目录里的 observation.images.{label} 路径改名（v2.1 与 v3 层级不同）。"""
    videos = os.path.join(dataset, "videos")
    if not os.path.isdir(videos):
        print(f"  ! 无 {videos}，跳过")
        return 0
    n = 0
    if v3:
        # v3: videos/observation.images.{label}/{chunk}/
        for name in os.listdir(videos):
            new = _apply_renames(name)
            if new and new != name:
                src, dst = os.path.join(videos, name), os.path.join(videos, new)
                if dry:
                    print(f"    [dry] {src} -> {dst}")
                else:
                    os.rename(src, dst)
                n += 1
    else:
        # v2.1: videos/{chunk}/observation.images.{label}/
        for chunk in os.listdir(videos):
            chunk_dir = os.path.join(videos, chunk)
            if not os.path.isdir(chunk_dir):
                continue
            for name in os.listdir(chunk_dir):
                new = _apply_renames(name)
                if new and new != name:
                    src, dst = os.path.join(chunk_dir, name), os.path.join(chunk_dir, new)
                    if dry:
                        print(f"    [dry] {src} -> {dst}")
                    else:
                        os.rename(src, dst)
                    n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", help="raw session 目录（episode*/camera_data.h5 + aligned_data.h5）")
    ap.add_argument("--v21", help="LeRobot v2.1 数据集目录")
    ap.add_argument("--v3", help="ACT v3 数据集目录")
    ap.add_argument("--dry-run", action="store_true", help="只打印不改名")
    args = ap.parse_args()

    if not (args.raw or args.v21 or args.v3):
        ap.error("need --raw and/or --v21 and/or --v3")

    total = 0
    if args.raw:
        import glob

        episodes = sorted(glob.glob(os.path.join(args.raw, "episode*")))
        print(f"== raw {args.raw}: {len(episodes)} 个 episode ==")
        for ep in episodes:
            for h5 in ("camera_data.h5", "aligned_data.h5"):
                p = os.path.join(ep, h5)
                if os.path.isfile(p):
                    total += rename_h5_keys(p, args.dry_run)

    if args.v21:
        print(f"== v2.1 {args.v21} ==")
        total += rename_info_json(args.v21, args.dry_run)
        total += rename_videos(args.v21, v3=False, dry=args.dry_run)

    if args.v3:
        print(f"== v3 {args.v3} ==")
        total += rename_info_json(args.v3, args.dry_run)
        total += rename_videos(args.v3, v3=True, dry=args.dry_run)

    print(f"\n共 {total} 处改名" + ("（dry-run，未实际执行）" if args.dry_run else "完成"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
