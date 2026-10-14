#!/usr/bin/env python3
"""真实数据推理正确性验证（py3.12 lerobot 环境，无 ROS）。

从 ACT v3 数据集读真实 episode 的观测（observation.state + base/left_wrist 图像），
喂给训练好的 ACT checkpoint，把模型预测的绝对动作与录制的 action 逐帧对比。

验证点：
  * 模型加载 + 真实观测推理不报错、输出有限、维度正确、夹爪在 [0,1]；
  * 预测与录制的对齐偏移（predicted[0] ≈ action[t]？经验判定 -1/0/+1）；
  * 每维 MAE + 整体 MAE（数值上「推理结果是否正确」的直接证据）。

用法（py3.12 环境）：
  PYTHONPATH=astral_ws/src/astral_policy_inference:astral_ws/src/astral_data_collect \
    /home/robot/miniconda3/envs/lerobot/bin/python astral_ws/scripts/run_act_correctness.py \
      --checkpoint-dir astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model \
      --dataset-dir astral_data/act/pick_up_and_place_480（空格注意引号）

退出码 0=通过；非 0=失败。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")

import numpy as np

DEFAULT_TASK = "Pick up the red-capped liquid container and place it into the box"


def load_episode(dataset_dir: str, episode_index: int, max_frames: int = 80, stride: int = 4):
    """读一段 episode 的 (state, action) 行范围 + 视频路径 + task。"""
    import pyarrow.parquet as pq

    info = json.load(open(os.path.join(dataset_dir, "meta", "info.json")))
    episodes = pq.read_table(
        os.path.join(dataset_dir, "meta", "episodes", "chunk-000", "file-000.parquet")
    ).to_pydict()
    n_ep = len(episodes["episode_index"])
    if episode_index >= n_ep:
        raise ValueError(f"episode {episode_index} >= {n_ep}")
    row_from = int(episodes["dataset_from_index"][episode_index])
    row_to = int(episodes["dataset_to_index"][episode_index])
    length = int(episodes["length"][episode_index])
    task = episodes["tasks"][episode_index]

    data = pq.read_table(
        os.path.join(dataset_dir, "data", "chunk-000", "file-000.parquet"),
        columns=["observation.state", "action"],
    ).slice(row_from, length)
    states = np.asarray(data.column("observation.state").to_pylist(), dtype=np.float64)
    actions = np.asarray(data.column("action").to_pylist(), dtype=np.float64)

    # 视频文件按 chunk 合并了多段；episode 内帧号 t → 视频绝对帧号 = from_timestamp*fps + t
    video_files = {}
    video_offsets = {}
    for cam in ("base", "left_wrist"):
        vchunk = int(episodes[f"videos/observation.images.{cam}/chunk_index"][episode_index])
        vfile = int(episodes[f"videos/observation.images.{cam}/file_index"][episode_index])
        from_ts = float(episodes[f"videos/observation.images.{cam}/from_timestamp"][episode_index])
        path = os.path.join(
            dataset_dir, "videos", f"observation.images.{cam}",
            f"chunk-{vchunk:03d}", f"file-{vfile:03d}.mp4")
        if not os.path.isfile(path):
            raise FileNotFoundError(f"{cam} video not found: {path}")
        video_files[cam] = path
        video_offsets[cam] = int(round(from_ts * int(info.get("fps", 30))))

    idxs = list(range(0, length, stride))[:max_frames]
    print(f"  episode {episode_index}: {length} frames, rows {row_from}..{row_to}, "
          f"task={task!r}, sampling {len(idxs)} frames (stride {stride}), "
          f"video_offsets={video_offsets}")
    return states, actions, video_files, video_offsets, task, idxs


def decode_frame(video_path: str, frame_idx: int, fps: int = 30) -> np.ndarray:
    """解码 mp4 指定帧为 HxWx3 uint8 RGB。

    seek 到目标帧前最近的关键帧再向前解几帧（内存 O(1)）。**禁止整段解码**：
    单个 file-*.mp4 含整个 chunk 的所有 episode（~1 万帧 ≈ 15GB/相机），
    全量解码会 OOM 卡死（已踩过）。
    """
    import av

    with av.open(video_path) as c:
        stream = c.streams.video[0]
        tb = stream.time_base  # Fraction, e.g. 1/15360
        target_pts = int(round((frame_idx / fps) / float(tb)))
        c.seek(max(0, target_pts - 1), stream=stream, backward=True)
        for frame in c.decode(stream):
            if frame.pts is None:
                continue
            idx = round(frame.pts * float(tb) * fps)
            if idx >= frame_idx:
                # to_ndarray("rgb24") 已是 HxWx3 uint8 RGB，勿再转置
                return frame.to_ndarray(format="rgb24")
    raise RuntimeError(f"could not decode frame {frame_idx}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint-dir", required=True)
    ap.add_argument("--dataset-dir", required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--max-frames", type=int, default=80)
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--action-dim", type=int, default=8)
    args = ap.parse_args()

    from astral_policy_inference.backend import InprocBackend, ObsBatch

    states, actions, video_files, video_offsets, task, idxs = load_episode(
        args.dataset_dir, args.episode, args.max_frames, args.stride)

    back = InprocBackend(
        checkpoint_dir=args.checkpoint_dir,
        action_dim=args.action_dim,
        image_keys={"base": "base", "left_wrist": "left_wrist"},
        default_prompt=task or DEFAULT_TASK,
    )
    back.open()
    print(f"  model loaded (action_dim={back.action_dim})")

    preds: list[np.ndarray] = []
    for i, t in enumerate(idxs):
        # select_action 内部维护 50 行 action 队列：不清队列的话，后续 infer 只会
        # 吐上一 chunk 的缓存行（对新观测不重新推理）。逐帧评估必须每帧 reset。
        back.reset()
        images = {cam: decode_frame(video_files[cam], video_offsets[cam] + t)
                  for cam in video_files}
        obs = ObsBatch(state=states[t], images=images, prompt=task or DEFAULT_TASK)
        out = np.asarray(back.infer(obs), dtype=np.float64)
        preds.append(out[0])  # ACT 首行 = 该步应执行的动作
    preds = np.asarray(preds)  # (n, action_dim)
    targets = np.asarray([actions[t] for t in idxs])

    fails = []
    # 对齐判定：predicted[0] 应 ≈ action[t]（next-state 语义）；也试 ±1 偏移
    offsets = []
    for off in (-1, 0, 1):
        tgt_off = np.asarray([actions[t + off] for t in idxs if 0 <= t + off < len(actions)])
        pr_off = preds[: len(tgt_off)]
        offsets.append((off, np.abs(pr_off - tgt_off).mean()))
    best_off, best_mae = min(offsets, key=lambda x: x[1])

    names = states.shape[1] and [f"d{i}" for i in range(actions.shape[1])]
    arm_mae = np.abs(preds - targets).mean(axis=0)
    print("\n  offset vs recorded action (MAE rad): "
          + ", ".join(f"{off:+d}: {mae:.4f}" for off, mae in offsets))
    print(f"  best alignment offset = {best_off:+d} (MAE {best_mae:.4f})")
    print("  per-dim MAE (predicted[0] vs action[t+offset]):")
    for i, m in enumerate(arm_mae):
        print(f"    {names[i]:>8}: {m:.4f}  (recorded last= "
              f"{np.round(targets[-1][i],4) if len(targets) else '?'})")
    gripper_mae = arm_mae[-1]
    check = (best_mae < 0.15) and gripper_mae < 0.1 and bool(np.isfinite(preds).all()) \
            and bool((preds[:, -1] >= -0.01).all()) and bool((preds[:, -1] <= 1.01).all())
    print("\nRESULT", "ALL PASS" if check else "FAIL",
          f"(overall MAE {best_mae:.4f} rad, gripper MAE {gripper_mae:.4f})")
    return 0 if check else 1


if __name__ == "__main__":
    sys.exit(main())
