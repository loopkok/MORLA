#!/usr/bin/env python3
"""推理时间曲线诊断：从 joint_stream_log 画 state/action 曲线，算错位时间与动作平滑度。

用法（先跑推理，节点 joint_stream_log_file:=/tmp/pi_cmds.jsonl 已记录；
可再给 --metrics /tmp/pi_metrics.jsonl 做换 chunk 边界关联）：
  /usr/bin/python3 astral_ws/scripts/plot_inference_curves.py \
      --log /tmp/pi_cmds.jsonl --out /tmp/pi_curves.png [--metrics /tmp/pi_metrics.jsonl]

输出：
  * PNG：7 个臂关节 state(蓝实) vs command(红虚) 时间曲线 + 动作步长(平滑度) + 夹爪；
        步长面板红点 = >阈值尖峰、绿竖虚线 = 换 chunk（重规划）边界；
  * 终端：state↔command 错位时间（逐关节互相关，+ = command 领先）、动作步长分布/尖峰，
        每个尖峰标：距最近重规划时刻（判定是否换 chunk 边界）+ 类型
        （收敛拉回 = command 一步追向 state；模型突变 = 主动跳离 state）。

日志每行 JSON：{"t": epoch, "/left_arm/joint_commands": [7], "/left_gripper/command": [1],
"state": [8]}（state 为节点新加的观测向量，旧日志可能没有 → 跳过错位与尖峰分类）。
metrics 用节点 metrics_log_file 产物（engine.plans 递增处 = 重规划/换 chunk 时刻）。
退出码 0=成功。
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def load(path: str) -> list[dict]:
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            if "/left_arm/joint_commands" in d:
                rows.append(d)
    return rows


def load_plans_times(metrics_path: str) -> np.ndarray | None:
    """读 metrics_log_file 产物，返回 ``engine.plans`` 递增处的绝对时间戳数组。

    每次 plans 递增 = 引擎换 chunk（重规划）——用于把尖峰关联到换 chunk 边界。
    """
    out: list[float] = []
    last: int | None = None
    try:
        with open(metrics_path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    d = json.loads(line)
                except Exception:  # noqa: BLE001
                    continue
                p = (d.get("engine") or {}).get("plans")
                if p is not None and p != last:
                    out.append(float(d["t"]))
                    last = p
    except OSError as exc:
        print(f"读 metrics 失败: {exc}", file=sys.stderr)
        return None
    return np.array(out) if out else None


def analyze_spikes(
    t_abs: np.ndarray,
    cmd: np.ndarray,
    st: np.ndarray | None,
    plans_t: np.ndarray | None,
    thresh: float = 0.1,
    chunk_window: float = 0.5,
) -> list[dict]:
    """尖峰分类：换 chunk 边界关联 + 收敛拉回 vs 模型突变。

    对每个 >thresh 的步长尖峰：
      * on_chunk：尖峰时刻距最近重规划（plans 递增）<= chunk_window 秒；
      * kind：跳变后 command 是否贴向 state——跳完距 state < 0.6×步长 = 收敛拉回
        （旧 command 漂移、新预测猛追回真实位置）；否则模型突变（主动跳离 state，
        可能是真实策略行为）。state 缺失 → 未知。
    """
    if len(cmd) < 2:
        return []
    dcmd = np.abs(np.diff(cmd, axis=0)).max(axis=1)
    spikes: list[dict] = []
    for i in np.where(dcmd > thresh)[0]:
        idx = int(i + 1)
        d = cmd[idx] - cmd[i]
        j = int(np.argmax(np.abs(d)))
        rec: dict = {
            "row": idx,
            "t": float(t_abs[idx]),
            "step": float(dcmd[i]),
            "joint": j,
            "d_new_state": None,
            "near_plans": None,
            "on_chunk": False,
            "kind": "?",
        }
        if st is not None and st.shape[1] > j:
            rec["d_new_state"] = float(abs(cmd[idx, j] - st[idx, j]))
        if plans_t is not None and len(plans_t):
            prev_p = plans_t[plans_t <= t_abs[idx]]
            next_p = plans_t[plans_t > t_abs[idx]]
            dtp = 1e9
            if len(prev_p):
                dtp = min(dtp, t_abs[idx] - prev_p[-1])
            if len(next_p):
                dtp = min(dtp, next_p[0] - t_abs[idx])
            rec["near_plans"] = float(dtp)
            rec["on_chunk"] = dtp <= chunk_window
        if rec["d_new_state"] is not None:
            rec["kind"] = (
                "收敛拉回" if rec["d_new_state"] < 0.6 * rec["step"] else "模型突变"
            )
        else:
            rec["kind"] = "未知(无state)"
        spikes.append(rec)
    return spikes


def _self_test() -> int:
    """合成数据验证 analyze_spikes 分类逻辑（换 chunk 关联 + 收敛拉回/模型突变）。"""
    rng = np.random.default_rng(7)
    n = 60
    t0 = 1_000_000.0
    t = t0 + np.arange(n) / 30.0
    base = 0.3 * np.sin(np.arange(n) / 12.0)
    cmd = np.stack([base + rng.normal(0, 0.01, n) for _ in range(7)], axis=1)
    st = np.stack([base + rng.normal(0, 0.005, n) for _ in range(7)], axis=1)

    # 收敛拉回：旧 command 漂移，换 chunk 处猛追回 state（在行 20 附近重规划）
    st[20] = cmd[20] + 0.0
    cmd[19, 3] = st[19, 3] + 0.4          # 旧 command 漂移 +0.4
    cmd[20, 3] = st[20, 3] + 0.01         # 换 chunk 追回 state（一步 0.39）
    # 模型突变：在行 50 主动跳离 state（远离唯一重规划行 20，非换 chunk）
    cmd[50, 5] = cmd[49, 5] + 0.25        # 跳 0.25 到远离 state 的地方
    plans_t = np.array([t0 + 20 / 30.0])  # 重规划只在行 20（行 50 距它 1.0s）

    spikes = analyze_spikes(t, cmd, st, plans_t, thresh=0.1, chunk_window=0.5)
    by_row = {s["row"]: s for s in spikes}
    ok = True
    # 行20：换chunk + 收敛拉回
    s = by_row.get(20)
    ok &= s is not None and s["on_chunk"] and s["kind"] == "收敛拉回"
    # 行50：非换chunk + 模型突变
    s = by_row.get(50)
    ok &= s is not None and not s["on_chunk"] and s["kind"] == "模型突变"
    print(f"self-test: 收敛拉回@行20(换chunk)={'PASS' if by_row.get(20) and by_row[20]['kind']=='收敛拉回' and by_row[20]['on_chunk'] else 'FAIL'}"
          f"  模型突变@行50(非换chunk)={'PASS' if by_row.get(50) and by_row[50]['kind']=='模型突变' and not by_row[50]['on_chunk'] else 'FAIL'}")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--log", help="joint_stream_log_file 产物 (jsonl；--self-test 时可不给)")
    ap.add_argument("--out", default="/tmp/pi_curves.png", help="PNG 输出路径")
    ap.add_argument("--metrics", default=None,
                    help="metrics_log_file 产物 (jsonl)；给则用 engine.plans 递增时刻标换 chunk 边界")
    ap.add_argument("--lag-max", type=int, default=50, help="互相关搜索最大滞后（行）")
    ap.add_argument("--smooth-thresh", type=float, default=0.1, help="动作尖峰阈值 rad/步")
    ap.add_argument("--chunk-window", type=float, default=0.5,
                    help="距最近重规划 <= 此秒数视为换 chunk 边界（需 --metrics）")
    ap.add_argument("--self-test", action="store_true", help="合成数据自测分类逻辑后退出")
    args = ap.parse_args()

    if args.self_test:
        return _self_test()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover
        print(f"需要 matplotlib：{exc}（可用 /home/robot/miniconda3/envs/lerobot/bin/python）",
              file=sys.stderr)
        return 1

    rows = load(args.log)
    if not rows:
        print("日志无有效行", file=sys.stderr)
        return 1

    t = np.array([r["t"] for r in rows])           # 绝对时间戳（与 plans_t 同轴）
    cmd = np.array([r["/left_arm/joint_commands"] for r in rows], dtype=float)  # (N,7)
    grip = np.array([
        (r.get("/left_gripper/command") or [np.nan])[0] for r in rows
    ], dtype=float)
    has_state = all(r.get("state") is not None for r in rows)
    st = np.array([r["state"] for r in rows], dtype=float) if has_state else None  # (N,8)

    t_rel = t - t[0]
    hz = len(t) / max(t_rel[-1], 1e-6)
    NJ = cmd.shape[1]
    print(f"{len(rows)} 行, {t_rel[-1]:.1f}s, ~{hz:.1f}Hz, state={'有' if has_state else '无'}")

    # ---------------- 动作平滑度：每行最大|Δ| + 尖峰分类 ----------------
    dcmd = np.abs(np.diff(cmd, axis=0)).max(axis=1) if len(cmd) > 1 else np.zeros(0)
    dt = np.diff(t_rel)
    plans_t = load_plans_times(args.metrics) if args.metrics else None
    if plans_t is not None:
        print(f"重规划(换chunk)时刻: {len(plans_t)} 次 (~{len(plans_t)/max(t_rel[-1],1e-6):.2f}Hz)")
    if len(dcmd):
        print("动作步长(rad/步): "
              + " ".join(f"p{p}={np.percentile(dcmd, p):.4f}" for p in (50, 90, 99))
              + f"  max={dcmd.max():.4f}")
        spikes = analyze_spikes(t, cmd, st, plans_t, args.smooth_thresh, args.chunk_window)
        print(f"步长>{args.smooth_thresh} 尖峰: {len(spikes)} 个 ({(len(spikes) / len(dcmd) * 100):.1f}%)")
        for s in spikes:
            kind = s["kind"]
            ck = "★换chunk" if s["on_chunk"] else "  非chunk"
            extra = (f"  cmd-state={s['d_new_state']:.3f}"
                     if s["d_new_state"] is not None else "")
            near = (f"  距重规划{s['near_plans']:.2f}s" if s["near_plans"] is not None else "")
            vel = s["step"] / max(dt[s["row"] - 1], 1e-6)
            print(f"  t={t_rel[s['row']]:6.2f}s  步长 {s['step']:.3f} rad  "
                  f"(~{vel:.1f} rad/s)  关节j{s['joint']}  {ck}{near}  {kind}{extra}")
        n_chunk = sum(1 for s in spikes if s["on_chunk"])
        n_conv = sum(1 for s in spikes if s["kind"] == "收敛拉回")
        if spikes:
            print(f"  -- 换chunk边界 {n_chunk}/{len(spikes)}；收敛拉回 {n_conv}/{len(spikes)}")

    # ---------------- state ↔ command 错位时间（逐关节互相关） ----------------
    lags: dict[int, tuple[int, float]] = {}
    if st is not None and len(st) > 4 * args.lag_max:
        n = len(st)
        L = min(args.lag_max, n // 4)
        for j in range(NJ):
            x = st[:, j] - st[:, j].mean()
            y = cmd[:, j] - cmd[:, j].mean()
            best_s, best_c = 0, -2.0
            for s in range(-L, L + 1):
                if s >= 0:
                    a, b = x[: n - s], y[s:]      # state[i] vs command[i+s]
                else:
                    a, b = x[-s:], y[: n + s]     # state[i-s] vs command[i]
                if len(a) < 4:
                    continue
                c = float(np.corrcoef(a, b)[0, 1])
                if c > best_c:
                    best_c, best_s = c, s
            lags[j] = (best_s, best_c)
        mean_s = np.mean([v[0] for v in lags.values()])
        # 符号约定：best_s<0 时 state[i]≈command[i-|s|] → command 领先 state。
        direction = "state 领先 command" if mean_s > 0 else "command 领先 state"
        print(f"state↔command 错位：")
        for j, (s, c) in lags.items():
            print(f"  joint{j}: {s:+d} 行 ({s / hz * 1000:+.0f}ms)  corr={c:.3f}")
        print(f"  平均 {abs(mean_s) / hz * 1000:.0f}ms ({direction}, {abs(mean_s):.1f} 行 @{hz:.0f}Hz)")
    else:
        print("state 数据不足或旧日志无 state，跳过错位分析")

    # ---------------- 绘图 ----------------
    fig, axes = plt.subplots(3, 3, figsize=(16, 12))
    order = [0, 1, 2, 3, 4, 5, 6]
    for k, j in enumerate(order):
        ax = axes[k // 3][k % 3]
        if st is not None:
            ax.plot(t_rel, st[:, j], "b-", lw=0.9, label="state")
        ax.plot(t_rel, cmd[:, j], "r--", lw=0.8, label="command")
        if plans_t is not None:
            for pt in plans_t:
                ax.axvline(pt - t[0], color="g", ls=":", lw=0.6, alpha=0.5)
        ax.set_title(f"joint{j}")
        ax.set_xlabel("t (s)")
        ax.legend(fontsize=7)
    # 平滑度面板
    ax = axes[2][1]
    ax.plot(t_rel[1:], dcmd, "k-", lw=0.6)
    ax.axhline(args.smooth_thresh, color="r", ls=":", lw=0.8, label="thresh")
    for s in (spikes if len(dcmd) else []):
        color = "#d62728" if s["kind"] == "模型突变" else "#ff7f0e"
        lbl = "converge" if s["kind"] == "收敛拉回" else ("model-shift" if s["kind"] == "模型突变" else None)
        ax.plot(t_rel[s["row"]], s["step"], "o", color=color, ms=5,
                label=lbl if (lbl and s["row"] == (spikes[0]["row"] if spikes else -1)) else "")
    if plans_t is not None:
        for pt in plans_t:
            ax.axvline(pt - t[0], color="g", ls=":", lw=0.8, alpha=0.6)
    ax.set_title("action step size (max|Δ|/step); orange=converge red=model-shift green:replan")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("rad/step")
    ax.legend(fontsize=7)
    # 夹爪 + 统计面板
    ax = axes[2][2]
    ax.plot(t_rel, grip, "g-", lw=0.9, label="grip cmd")
    if st is not None:
        ax.plot(t_rel, st[:, -1], "b--", lw=0.8, label="grip state")
    ax.set_title("gripper")
    ax.set_xlabel("t (s)")
    ax.legend(fontsize=7)
    txt = [f"{len(rows)} rows, ~{hz:.0f}Hz"]
    if lags:
        ms = mean_s / hz * 1000
        txt.append(f"state->cmd offset: {ms:+.0f}ms ({direction})")
    if len(dcmd):
        txt.append(f"step p50/p90/max: "
                   f"{np.percentile(dcmd, 50):.3f}/{np.percentile(dcmd, 90):.3f}/{dcmd.max():.3f} rad")
        if spikes:
            txt.append(f"spikes {len(spikes)}: chunk-boundary {n_chunk}, converge {n_conv}")
    ax.text(0.02, 0.98, "\n".join(txt), transform=ax.transAxes, va="top",
            family="monospace", fontsize=9)
    fig.suptitle("Inference curves: state vs action, offset & smoothness", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(args.out, dpi=120)
    print(f"图已存: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
