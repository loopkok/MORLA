#!/usr/bin/env python3
"""真实 ACT checkpoint 的模型侧集成验证（无 ROS，需 py3.12 + lerobot + CUDA）。

把 /tmp 时代散落的后端/引擎验证收拢为可复用脚本。验证三段：
  1. 后端：InprocBackend 加载 checkpoint + 合成观测推理（get_policy_class 路径，
     即 backend.py 修复的回归点）；
  2. 引擎：ActionEngine 接真实后端，50 tick 产出 50 行平滑绝对动作；ACT 内部队列语义
     （50 行只真推理 1 次，其余缓存弹出）；reset 后从新观测重规划；
  3. 输出语义：夹爪维绝对闭合比、相邻行不跳变、维度 = action_dim。

用法（在 py3.12 lerobot 环境）：
  PYTHONPATH=astral_ws/src/astral_policy_inference:astral_ws/src/astral_data_collect \
    /home/robot/miniconda3/envs/lerobot/bin/python astral_ws/scripts/run_act_integration.py \
      --checkpoint-dir astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model

退出码 0=通过；非 0=失败（打印 FAIL 行）。
"""
from __future__ import annotations

import argparse
import sys

sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")

import numpy as np

from astral_policy_inference.backend import InprocBackend, ObsBatch
from astral_policy_inference.engine import ActionEngine

DEFAULT_TASK = "Pick up the red-capped liquid container and place it into the box"

fails: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")
    if not cond:
        fails.append(name)


def build_obs(dim: int, cam_size: int, rng: np.random.Generator) -> ObsBatch:
    # 用接近真实的关节位姿（含离开零位的 d3≈-1.9，取自真实数据），保证引擎绝对语义
    # 守卫在真实模型上通过。合成 OOD 状态（如 normal(0,0.5) 偶发 |state|>1）会让 ACT
    # 预测向训练均值 → arm_scale 变小 → 守卫误报（部署中状态永远在分布内，不会出现）。
    state = np.array([-0.35, 0.19, -0.004, -1.89, -0.20, -0.001, 0.014, 0.5])[:dim]
    return ObsBatch(
        state=state,
        images={
            "base": rng.integers(0, 256, (cam_size, cam_size, 3), dtype=np.uint8),
            "left_wrist": rng.integers(0, 256, (cam_size, cam_size, 3), dtype=np.uint8),
        },
        prompt=DEFAULT_TASK,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint-dir", required=True,
                    help="lerobot ACT checkpoint dir (config.json + model.safetensors ...)")
    ap.add_argument("--action-dim", type=int, default=8)
    ap.add_argument("--camera-size", type=int, default=480,
                    help="模型 preprocessor 要求的图像边长（本模型 480，无 resize）")
    ap.add_argument("--n-action-steps", type=int, default=50,
                    help="ACT chunk 长度（config.json n_action_steps）")
    ap.add_argument("--device", default=None)
    args = ap.parse_args()

    # ---- 1. 后端：加载 + 推理 ----
    back = InprocBackend(
        checkpoint_dir=args.checkpoint_dir,
        action_dim=args.action_dim,
        image_keys={"base": "base", "left_wrist": "left_wrist"},
        device=args.device,
        default_prompt=DEFAULT_TASK,
    )
    rng = np.random.default_rng(0)
    back.open()
    check("open(): checkpoint + pre/post load", back._policy is not None)
    obs = build_obs(args.action_dim, args.camera_size, rng)
    action = back.infer(obs)
    arr = np.asarray(action)
    check("infer(): (n, action_dim)", arr.ndim == 2 and arr.shape[1] == args.action_dim,
          f"shape={arr.shape}")
    check("infer(): finite", bool(np.isfinite(arr).all()),
          f"min={np.nanmin(arr):.4f} max={np.nanmax(arr):.4f}")
    check("infer(): gripper dim absolute in [0,1.2]",
          bool((arr[:, -1] >= 0).all() and (arr[:, -1] <= 1.2).all()),
          f"gripper [{arr[:, -1].min():.4f},{arr[:, -1].max():.4f}]")
    # 方案 A：infer() 一次返回完整 chunk（n_action_steps 行），引擎拿回分块权。
    # 老行为（select_action）每次只回 1 行 → 该断言在改动前 FAIL。
    check("infer(): returns full chunk (n_action_steps rows)",
          arr.shape[0] == args.n_action_steps,
          f"got {arr.shape[0]} (期望 {args.n_action_steps}; select_action 老行为=1)")

    # ---- 2. 引擎：按 chunk 分块 + 边界重规划 + reset 重规划 ----
    back2 = InprocBackend(
        checkpoint_dir=args.checkpoint_dir,
        action_dim=args.action_dim,
        image_keys={"base": "base", "left_wrist": "left_wrist"},
        device=args.device,
        default_prompt=DEFAULT_TASK,
    )
    _orig = back2.infer
    _durations = []

    def _wrapped(o):
        import time
        t0 = time.perf_counter()
        out = _orig(o)
        _durations.append(time.perf_counter() - t0)
        return out

    back2.infer = _wrapped
    eng = ActionEngine(back2, mode="queue_sync", action_dim=args.action_dim,
                       chunk=args.n_action_steps, policy_fps=30, autostart=False)
    eng.feed_obs(build_obs(args.action_dim, args.camera_size, rng))
    eng.start()
    check("engine start(): full chunk installed (one real plan)",
          eng.stats["plans"] == 1 and eng.remaining == args.n_action_steps,
          f"plans={eng.stats['plans']} remaining={eng.remaining}")
    rows = []
    for _ in range(args.n_action_steps):
        r = eng.tick()
        if r is None:
            break
        rows.append(r)
    arr = np.asarray(rows)
    check("tick() x n_action_steps yields that many rows",
          len(rows) == args.n_action_steps, f"got {len(rows)}")
    check("rows finite 8-dim",
          arr.shape == (args.n_action_steps, args.action_dim) and bool(np.isfinite(arr).all()),
          f"shape={arr.shape}")
    check("single plan covers full chunk (no re-infer mid-chunk)",
          eng.stats["plans"] == 1, f"plans={eng.stats['plans']}")
    d = np.linalg.norm(np.diff(arr, axis=0), axis=1)
    check("consecutive rows smooth (no cliff)", bool(np.all(d < 5.0)),
          f"max step={d.max():.4f} rad")
    # chunk 边界 → 阻塞重填（queue_sync）→ 一次新推理
    r = eng.tick()
    check("tick at chunk boundary re-plans",
          r is not None and eng.stats["plans"] == 2,
          f"plans={eng.stats['plans']}")
    # reset 清引擎 chunk → 下一次 tick 从新观测重新规划
    eng.reset()
    check("reset clears engine chunk", eng.remaining == 0)
    plans_before = eng.stats["plans"]
    eng.feed_obs(build_obs(args.action_dim, args.camera_size, rng))
    r = eng.tick()
    check("after reset, tick() re-plans fresh chunk",
          r is not None
          and eng.stats["plans"] == plans_before + 1
          and eng.remaining == args.n_action_steps - 1,
          f"plans {plans_before}->{eng.stats['plans']} remaining={eng.remaining}")
    eng.stop()

    if _durations:
        print(f"  info: infer durations — mean {sum(_durations)/len(_durations)*1000:.1f}ms, "
              f"max {max(_durations)*1000:.0f}ms, {len(_durations)} calls "
              f"(= {len(_durations)} real inferences, one per chunk)")

    print("\nRESULT", "ALL PASS" if not fails else "FAILURES: " + ",".join(fails))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
