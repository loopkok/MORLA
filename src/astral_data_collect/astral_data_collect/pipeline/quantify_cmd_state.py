#!/usr/bin/env python3
"""量化采集数据的"操作意图 vs 摩擦伪影"（cmd vs state）。

背景：遥操粘滞/静摩擦 → 慢速时实际 state"粘-滑"（平段 + 突跳）。next-state 语义
action[t]=state[t+1] 会把摩擦伪影直接做成回归目标 → 模型学到"停-跳"节奏。本脚本
从采集数据量化 cmd（操作意图）与 state（实际位置）的差异，把每个停顿标成：

  * 意图型（intent）：cmd 也停住 → 操作者/任务本身要停（真实行为，训练该保留）
  * 摩擦型（friction）：cmd 在平滑移动、state 却平段+突跳 → 摩擦伪影（训练该去掉）
  * 混合（mixed）：两者都有

用法：
  # raw robot_data.h5（含 /streams/*_cmd + /*_state）→ 完整对比 + 停顿分类
  bash astral_ws/scripts/data_pipeline.sh quantify \
      --h5 <episode>/robot_data.h5 [--h5 <更多>] [--out res.json] [--plot res.png]
  # 传 episode 目录 / session 目录自动找 robot_data.h5
  bash astral_ws/scripts/data_pipeline.sh quantify --h5 ~/astral_data/raw/pick_place/

  # aligned_data.h5（仅 /observation/state + /action）→ next-state 动作零膨胀统计
  bash astral_ws/scripts/data_pipeline.sh quantify --aligned <aligned_data.h5>

输出：终端逐关节表（state/cmd 平段占比、粘滑频率、突跳幅度分布、停顿数与意图/摩擦分型）+
可选 JSON（全明细）+ 可选 PNG（state vs cmd 曲线 + 平段/分型着色）。

--self-test 用合成数据自测分型逻辑（先写失败用例再修的模式）。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .artifacts import require_source_signature

# ------------------------------------------------------------------ 工具


def _load_raw(h5_path: str) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """读 raw robot_data.h5，返回 {流名: (values(N,D), timestamps(N,))}。"""
    import h5py

    streams: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    with h5py.File(h5_path, "r") as f:
        if "streams" not in f:
            return streams
        for name in f["streams"]:
            g = f[f"streams/{name}"]
            streams[name] = (np.asarray(g["values"], dtype=np.float64),
                             np.asarray(g["timestamps"], dtype=np.float64))
    return streams


def _resample(x: np.ndarray, ts: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """最近邻重采样到 grid（对齐数据流的哲学：逐帧最近邻取帧）。"""
    if x.ndim == 1:
        x = x[:, None]
    idx = np.searchsorted(ts, grid, side="right") - 1
    idx = np.clip(idx, 0, len(ts) - 1)
    return x[idx]


def _distinct(x: np.ndarray, ts: np.ndarray, grid: np.ndarray):
    """最近邻重采样到 grid 后**折叠重复源样本**：返回去重样本值 + 各自真实采样时刻。

    若网格比源采样率细（或源采样有抖动间隙），最近邻会重复同一源样本到多个网格点——
    这些重复点在网格上的速度=0，会被误判成"平段"。折叠后速度在**真实采样间隔**上算。
    """
    if x.ndim == 1:
        x = x[:, None]
    idx = np.clip(np.searchsorted(ts, grid, side="right") - 1, 0, len(ts) - 1)
    new = np.concatenate([[True], idx[1:] != idx[:-1]])
    return x[idx][new], ts[idx][new]


def _grid_from(ts_cmd: np.ndarray, ts_state: np.ndarray, hz: int) -> np.ndarray:
    t0 = max(float(ts_cmd[0]), float(ts_state[0]))
    t1 = min(float(ts_cmd[-1]), float(ts_state[-1]))
    n = max(2, int((t1 - t0) * hz))
    return np.linspace(t0, t1, n)


def _native_hz(ts: np.ndarray) -> float:
    d = np.diff(np.sort(ts))
    med = float(np.median(d[d > 0])) if np.any(d > 0) else 0.0
    return 1.0 / med if med > 0 else 0.0


def _cross_lag(x: np.ndarray, tsx: np.ndarray, y: np.ndarray, tsy: np.ndarray,
               max_ms: int = 400, hz: int = 200) -> float:
    """两去重序列在公共时间轴上的互相关滞后（秒）。

    返回 lag_s = **state 领先 cmd** 的量：cmd 时间轴减 lag_s 即与 state 物理对齐。
    由每条流的 timestamp 语义差异产生（cmd=采集节点到达时刻，state=driver header.stamp），
    实测约 0.2s 恒定偏移；分型前必须对齐，否则 state 停顿窗口看的 cmd 错位、短停顿被错分。
    用每关节自身序列算（手腕关节滞后更大）。
    """
    t0, t1 = max(float(tsx[0]), float(tsy[0])), min(float(tsx[-1]), float(tsy[-1]))
    if t1 - t0 < 0.5 or len(x) < 20 or len(y) < 20:
        return 0.0
    gi = np.linspace(t0, t1, max(100, int((t1 - t0) * hz)))
    X = np.interp(gi, tsx, x[:, 0])
    Y = np.interp(gi, tsy, y[:, 0])
    # 守卫：任一方几乎不动（近恒定）时互相关无意义，任何 shift 都能"匹配"——返回 0，
    # 避免把静止关节的 cmd 平段标志错移、分型错分。
    if X.std() < 1e-4 or Y.std() < 1e-4:
        return 0.0
    step = 1.0 / hz
    best, bestc = 0, -2.0
    for k in range(-int(max_ms * hz / 1000), int(max_ms * hz / 1000) + 1):
        if k == 0:
            a, b = X, Y
        elif k < 0:
            a, b = X[:k], Y[-k:]     # cmd(t_i) vs state(t_{i+|k|}) → k<0 = state 领先
        else:
            a, b = X[k:], Y[:-k]
        if len(a) < 40 or len(b) < 40:
            continue
        ca = a - a.mean()
        cb = b - b.mean()
        c = float(np.dot(ca, cb) / (np.linalg.norm(ca) * np.linalg.norm(cb) + 1e-9))
        if c > bestc:
            best, bestc = k, c
    # 相关度太低 → 两序列没有可信对齐，返回 0（别用噪声 shift 错移窗口）
    if bestc < 0.5:
        return 0.0
    return -best * step


def _signal_stats(xs: np.ndarray, ts: np.ndarray, flat_v: float, min_pause: float):
    """单条信号（去重样本，xs/ts 对齐）的平段/突跳统计。

    "停住"按**速度**判定（flat_v rad/s，默认 0.02 ≈ 基本静止）：dx/dt < flat_v，速度在
    **真实采样间隔**上算。最近邻网格上的重复点（比源采样率更细的网格 / 采样抖动间隙）
    必须先经 `_distinct` 折叠，否则每个重复点速度=0 会把慢速移动误判成平段。
    """
    n = len(xs)
    if n < 3:
        return None
    dx = np.abs(np.diff(xs, axis=0)).max(axis=1)        # 样本间最大关节步长 (N-1,)
    dt = np.diff(ts)                                    # 真实采样间隔
    vel = dx / np.maximum(dt, 1e-6)                      # rad/s
    flat = vel < flat_v                                  # 该步"停住"
    # 平段游程（flat[i] 对应 xs[i]→xs[i+1]，时长为 ts[i]..ts[j]）
    runs: list[tuple[int, int]] = []
    i = 0
    while i < len(flat):
        if flat[i]:
            j = i
            while j < len(flat) and flat[j]:
                j += 1
            if ts[j] - ts[i] >= min_pause:
                runs.append((i, j))
            i = j
        else:
            i += 1
    slips = int((~flat[1:] & flat[:-1]).sum())           # 平→动上升沿 = 粘滑一次
    moving = dx[~flat]
    span = float(ts[-1] - ts[0])
    return {
        "flat_prop": float(flat.mean()),                  # 平段占比
        "flat_runs": len(runs),                            # 停顿次数
        "flat_run_mean_s": float(np.mean([ts[j] - ts[i] for i, j in runs])) if runs else 0.0,
        "flat_run_max_s": float(np.max([ts[j] - ts[i] for i, j in runs])) if runs else 0.0,
        "slip_hz": float(slips / span) if span > 0 else 0.0,  # 粘滑频率
        "step_p50": float(np.percentile(moving, 50)) if len(moving) else 0.0,
        "step_p90": float(np.percentile(moving, 90)) if len(moving) else 0.0,
        "step_max": float(moving.max()) if len(moving) else 0.0,
        "_runs": runs,                                   # 停顿游程 (i,j)
        "_flat": flat,                                   # 每步平段标志（供分型）
        "_ts": ts,                                       # 样本时刻（供分型）
    }


def classify_pause(frac: float, intent_frac: float = 0.5, friction_frac: float = 0.2) -> str:
    """state 停顿窗口内 cmd 平段占比 → 意图型/摩擦型/混合。"""
    if frac >= intent_frac:
        return "intent"
    if frac <= friction_frac:
        return "friction"
    return "mixed"


def analyze_cmd_state(cmd: np.ndarray, ts_cmd: np.ndarray,
                      state: np.ndarray, ts_state: np.ndarray, *,
                      hz: int = 100, flat_v: float = 0.02,
                      min_pause: float = 0.15, align: bool = True,
                      intent_frac: float = 0.5, friction_frac: float = 0.2) -> dict:
    """一条机器人数据流（如 left_arm）：逐关节 cmd vs state 对比 + 停顿分型。

    align=True（默认）：分型前用互相关把 cmd 时间轴对齐到 state（减掉 state 领先 cmd 的
    恒定量，~200ms）——两流时间戳语义不同（cmd=到达时刻/state=header.stamp），不齐则
    state 停顿窗口看的 cmd 错位、短停顿被错分。
    """
    # 网格分辨率 ≥ 两流原生采样率（否则 _distinct 漏采样）；速度在去重样本的真实
    # 采样间隔上算，网格分辨率只影响对齐窗口的精度。
    r_cmd, r_state = _native_hz(ts_cmd), _native_hz(ts_state)
    grid_hz = max(int(hz), int(r_cmd), int(r_state))
    grid = _grid_from(ts_cmd, ts_state, grid_hz)
    if len(grid) < 5:
        return {"error": "overlap too short"}
    Cs, Ct = _distinct(cmd, ts_cmd, grid)       # cmd（意图）
    Ss, St = _distinct(state, ts_state, grid)   # state（实际）
    d = min(Cs.shape[1], Ss.shape[1])
    joints: dict[str, dict] = {}
    for j in range(d):
        c = _signal_stats(Cs[:, j:j + 1], Ct, flat_v, min_pause)
        s = _signal_stats(Ss[:, j:j + 1], St, flat_v, min_pause)
        if c is None or s is None:
            continue
        # cmd 每步平段标志 + 步起点时刻（供停顿分型）
        cflat, cts = c["_flat"], Ct[:-1]
        lag = 0.0
        if align:
            # 对齐：cmd 时间轴减 lag_s（state 领先 cmd 的恒定量），消除两流时间戳语义
            # 差异（cmd=到达时刻 / state=header.stamp）对分型窗口的错位。
            lag = _cross_lag(Ss[:, j:j + 1], St, Cs[:, j:j + 1], Ct)
            cts = cts - lag
        pauses = []
        for i, jj in s["_runs"]:
            t0, t1 = St[i], St[jj]
            inw = (cts >= t0) & (cts < t1)
            frac = float(cflat[inw].mean()) if inw.any() else 1.0
            kind = classify_pause(frac, intent_frac, friction_frac)
            pauses.append({
                "t0": round(float(t0) - float(St[0]), 3),
                "t1": round(float(t1) - float(St[0]), 3),
                "dur_s": round(float(t1 - t0), 3),
                "kind": kind,
            })
        joints[f"j{j}"] = {
            "state_flat_prop": s["flat_prop"],
            "cmd_flat_prop": c["flat_prop"],
            "slip_hz": s["slip_hz"],
            "state_step_p50": s["step_p50"], "state_step_p90": s["step_p90"],
            "state_step_max": s["step_max"],
            "cmd_step_p50": c["step_p50"], "cmd_step_p90": c["step_p90"],
            "cmd_step_max": c["step_max"],
            "lag_s": round(lag, 4),   # state 领先 cmd 的对齐量（秒），供绘图对齐 cmd 曲线
            "pauses": pauses,
            "n_intent": sum(1 for p in pauses if p["kind"] == "intent"),
            "n_friction": sum(1 for p in pauses if p["kind"] == "friction"),
            "n_mixed": sum(1 for p in pauses if p["kind"] == "mixed"),
        }
    return {"joints": joints, "grid_hz": grid_hz, "grid_dt_ms": (1.0 / grid_hz) * 1000,
            "t0_origin": float(St[0])}


def analyze_aligned(action: np.ndarray, state: np.ndarray | None, hz: int = 30,
                    flat_v: float = 0.02, min_pause: float = 0.15) -> dict:
    """aligned_data.h5 的 /action（next-state）→ 零膨胀/突跳统计。

    zero_prop 用 **delta = action - state**（真·没动）判定；旧实现量 |action| 绝对位
    近 0（"关节在零位"，对 gripper=开位，语义错）。
    """
    joints: dict[str, dict] = {}
    d = action.shape[1] if action.ndim == 2 else 1
    A = action if action.ndim == 2 else action[:, None]
    t = np.arange(len(A)) / hz           # 均匀网格时间戳（30Hz）
    zero_th = flat_v * (1.0 / hz)        # 每步"基本没动"的位置阈值
    has_state = state is not None and state.shape[0] == A.shape[0]
    D = (A - state) if has_state else None
    for j in range(d):
        s = _signal_stats(A[:, j:j + 1], t, flat_v, min_pause)
        if has_state:
            zero_prop = float((np.abs(D[:, j]) < zero_th).mean())
        else:
            zero_prop = s["flat_prop"]   # 无 state 时退化用动作平段
        joints[f"j{j}"] = {
            "action_flat_prop": s["flat_prop"],
            "action_zero_prop": zero_prop,
            "action_step_p50": s["step_p50"], "action_step_p90": s["step_p90"],
            "action_step_max": s["step_max"],
            "n_zero_runs": s["flat_runs"],
            "slip_hz": s["slip_hz"],
        }
    return {"joints": joints, "grid_hz": hz}


# ------------------------------------------------------------------ CLI

def _collect_raw(paths: list[str]) -> list[dict]:
    """展开输入为 raw h5 列表（目录→递归找 robot_data.h5）。"""
    import glob
    import os

    found: list[str] = []
    for p in paths:
        p = os.path.expanduser(p)
        if os.path.isdir(p):
            found.extend(glob.glob(os.path.join(p, "**", "robot_data.h5"), recursive=True))
        else:
            found.append(p)
    out = []
    for h5 in sorted(found):
        streams = _load_raw(h5)
        arm = [k for k in streams if k.endswith("_state")]
        for sname in arm:
            base = sname[:-len("_state")]
            cname = base + "_cmd"
            if cname not in streams:
                continue
            out.append({
                "h5": h5,
                "stream": base,
                "cmd": streams[cname][0], "cmd_ts": streams[cname][1],
                "state": streams[sname][0], "state_ts": streams[sname][1],
            })
    return out


def _print_report(rows: list[dict], min_peak_v: float) -> None:
    for r in rows:
        res = r["res"]
        first = next(iter(res.get("joints", {})), None)
        if first is None or "error" in res:
            print(f"{r['h5']}/{r['stream']}: {res.get('error', 'no joints')}")
            continue
        if "action_flat_prop" in res["joints"][first]:
            _print_aligned_row(r)
        else:
            _print_cmd_state_row(r, min_peak_v)


def _print_aligned_row(r: dict) -> None:
    """aligned_data.h5 /action（next-state）零膨胀表。"""
    res = r["res"]
    n = len(res["joints"])
    agg = {k: 0 for k in ("action_flat_prop", "action_zero_prop", "slip_hz",
                          "action_step_p90", "action_step_max")}
    for jj in res["joints"].values():
        for k in agg:
            agg[k] += jj[k]
    for k in agg:
        agg[k] /= n
    print(f"{r['h5']} :: {r['stream']}  (next-state 动作，fps={res['grid_hz']})")
    print(f"  平段占比 {agg['action_flat_prop']*100:5.1f}%   零步占比 "
          f"{agg['action_zero_prop']*100:5.1f}%   粘滑Hz {agg['slip_hz']:5.1f}   "
          f"突跳p90 {agg['action_step_p90']*1000:5.0f}mrad   max {agg['action_step_max']*1000:5.0f}mrad")
    for jname, jj in res["joints"].items():
        print(f"    {jname}: 平段 {jj['action_flat_prop']*100:5.1f}%  零步 "
              f"{jj['action_zero_prop']*100:5.1f}%  停顿段 {jj['n_zero_runs']}  粘滑Hz {jj['slip_hz']:.1f}")


def _print_cmd_state_row(r: dict, min_peak_v: float) -> None:
    res = r["res"]
    joints = res["joints"]
    gz = res.get("grid_hz", 100)
    # 活跃关节 = 峰值 cmd 速度 ≥ min_peak_v（有实际任务运动）。否则全程低于 flat_v
    # 的慢关节恒判"平段"、恒产生"意图"停顿，污染聚合与分型。
    active = {k for k, jj in joints.items() if jj["cmd_step_max"] * gz >= min_peak_v}
    sel = {k: joints[k] for k in joints if k in active} or joints
    agg = {k: 0 for k in ("state_flat_prop", "cmd_flat_prop", "slip_hz",
                          "state_step_p90", "n_intent", "n_friction", "n_mixed")}
    for jj in sel.values():
        for k in agg:
            agg[k] += jj[k]
    n = len(sel)
    for k in ("state_flat_prop", "cmd_flat_prop", "slip_hz", "state_step_p90"):
        agg[k] /= n
    tag = f" [活跃关节 {sorted(active)}]" if len(active) < len(joints) else ""
    print(f"{r['h5'].split('/')[-2] + '/' + r['h5'].split('/')[-1] + '/' + r['stream']:<44}"
          f"state平段 {agg['state_flat_prop']*100:5.1f}%  cmd平段 {agg['cmd_flat_prop']*100:5.1f}%  "
          f"粘滑 {agg['slip_hz']:5.1f}Hz  突跳p90 {agg['state_step_p90']*1000:5.0f}mrad  "
          f"停顿 {agg['n_intent']+agg['n_friction']+agg['n_mixed']}"
          f"(意图{agg['n_intent']}/摩擦{agg['n_friction']}/混合{agg['n_mixed']}){tag}")
    for k, jj in sorted(joints.items()):
        print(f"    {k}: state平段 {jj['state_flat_prop']*100:5.1f}%  "
              f"cmd平段 {jj['cmd_flat_prop']*100:5.1f}%  粘滑 {jj['slip_hz']:4.1f}Hz  "
              f"突跳p90 {jj['state_step_p90']*1000:4.0f}mrad  停顿 "
              f"{jj['n_intent']+jj['n_friction']+jj['n_mixed']}"
              f"(意图{jj['n_intent']}/摩擦{jj['n_friction']}/混合{jj['n_mixed']})"
              + ("  ★活跃" if k in active else ""))
    # 摩擦型停顿明细（供"标签平滑/切 command-source"决策）
    for k, jj in joints.items():
        fri = [p for p in jj["pauses"] if p["kind"] == "friction"]
        if fri:
            print(f"    {k} 摩擦型停顿 {len(fri)} 个: "
                  + "; ".join(f"t={p['t0']:.2f}-{p['t1']:.2f}s({p['dur_s']:.2f}s)"
                              for p in fri[:6]) + (" ..." if len(fri) > 6 else ""))


def _self_test() -> int:
    """合成数据自测：cmd 平滑移动 + state 粘-滑 → 摩擦型；cmd 也停 → 意图型。"""
    hz, T = 100.0, 3.0
    grid = np.arange(0, T, 1 / hz)
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")
        ok = ok and cond

    vel = 0.15  # rad/s，命令匀速移动（> flat_v=0.02，判"在动"）

    # 场景1：cmd 匀速斜坡（意图平滑）；state = 粘-滑（每 120ms 粘住、1 格滑一步）
    cmd = vel * grid
    state = np.zeros_like(grid)
    pos = 0.0
    hold_frames = int(0.12 * hz)
    for i, t in enumerate(grid):
        if int(t * hz) % (2 * hold_frames) < hold_frames:  # 粘住
            state[i] = pos
        else:
            pos += vel * 0.12
            state[i] = pos
    res = analyze_cmd_state(cmd[:, None], grid, state[:, None], grid,
                            hz=hz, flat_v=0.02, min_pause=0.1)
    j = res["joints"]["j0"]
    check("state 平段占比 ~50%（粘滑）", abs(j["state_flat_prop"] - 0.5) < 0.12,
          f"={j['state_flat_prop']:.2f}")
    check("cmd 平段占比 ≈ 0（意图平滑）", j["cmd_flat_prop"] < 0.05,
          f"={j['cmd_flat_prop']:.2f}")
    check("停顿全为摩擦型", j["n_friction"] >= 10 and j["n_intent"] == 0,
          f"intent={j['n_intent']} friction={j['n_friction']}")

    # 场景2：cmd 和 state 都停 1s（操作者有意停顿）
    cmd2 = np.where((grid > 1.0) & (grid < 2.0), 0.0, vel * grid)
    state2 = cmd2.copy()
    res2 = analyze_cmd_state(cmd2[:, None], grid, state2[:, None], grid,
                             hz=hz, flat_v=0.02, min_pause=0.1)
    j2 = res2["joints"]["j0"]
    pauses = [p for p in j2["pauses"] if p["dur_s"] > 0.8]
    check("cmd+state 同停 → 意图型", len(pauses) == 1 and pauses[0]["kind"] == "intent",
          f"{pauses}")

    # 场景3：aligned next-state 零膨胀 —— 零步必须量 delta=action-state（真·没动），
    # 不是 |action| 绝对位近 0。state=台阶（停 4 拍跳 0.1），action=next-state → delta 80% 为 0。
    st3 = np.zeros(grid.shape)
    st3[:] = 0.1 * (np.arange(len(grid)) // 5)   # 每 5 拍跳 0.1，中间 80% 停住
    ac3 = np.empty_like(st3)
    ac3[:-1] = st3[1:]
    ac3[-1] = st3[-1]                            # action[t] = state[t+1]
    ra = analyze_aligned(ac3[:, None], st3[:, None], hz=hz, flat_v=0.02)
    check("next-state 零占比（delta 语义）", abs(ra["joints"]["j0"]["action_zero_prop"] - 0.8) < 0.05,
          f"={ra['joints']['j0']['action_zero_prop']:.2f}")

    # 场景4：cmd 短有意停顿(0.3s)，但 cmd 时间戳被推迟 0.2s（两流时间戳语义差异）——
    # 对齐后应分回意图；不对齐会被错分（窗口看的 cmd 已恢复移动）。
    cmd4 = np.where((grid > 1.0) & (grid < 1.3), 0.0, vel * grid)
    state4 = cmd4.copy()
    cmd_ts4 = grid + 0.2
    r4 = analyze_cmd_state(cmd4[:, None], cmd_ts4, state4[:, None], grid,
                           hz=hz, flat_v=0.02, min_pause=0.1, align=True)
    p4 = [p for p in r4["joints"]["j0"]["pauses"] if p["dur_s"] > 0.2]
    check("对齐后 cmd 短停顿 → 意图型", any(p["kind"] == "intent" for p in p4), f"{p4}")
    r4b = analyze_cmd_state(cmd4[:, None], cmd_ts4, state4[:, None], grid,
                            hz=hz, flat_v=0.02, min_pause=0.1, align=False)
    p4b = [p for p in r4b["joints"]["j0"]["pauses"] if p["dur_s"] > 0.2]
    check("不对齐 → 该停顿被错分（非意图）",
          not any(p["kind"] == "intent" for p in p4b), f"{p4b}")

    # 场景5：静止关节（cmd/state 都恒定）→ _cross_lag 守卫必须返回 0，不能给噪声 shift
    # （旧实现近恒定序列任何 shift 都"相关"，会错移 cmd 平段标志 → 分型错分）
    const = np.full_like(grid, 1.7)
    lag0 = _cross_lag(const[:, None], grid, const[:, None], grid)
    check("静止关节 → lag=0（守卫）", lag0 == 0.0, f"lag={lag0}")
    r5 = analyze_cmd_state(const[:, None], grid, const[:, None], grid,
                           hz=hz, flat_v=0.02, min_pause=0.1, align=True)
    # 全程恒定 → 平段≈100%，停顿应判意图（cmd 也停）而非被错移成摩擦
    j5 = r5["joints"]["j0"]
    check("静止关节分型不被错移", j5["n_friction"] == 0, f"friction={j5['n_friction']}")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--h5", nargs="+", help="raw robot_data.h5 / episode 目录 / session 目录")
    ap.add_argument("--aligned", help="aligned_data.h5（仅 state+action，next-state 零膨胀）")
    ap.add_argument("--hz", type=int, default=100, help="重采样网格频率（默认 100）")
    ap.add_argument("--flat-v", type=float, default=0.02,
                    help="判定'停住'的速度阈值 rad/s（默认 0.02 ≈ 基本静止）")
    ap.add_argument("--min-pause", type=float, default=0.15, help="最小停顿时长 s（默认 0.15）")
    ap.add_argument("--intent-thresh", type=float, default=0.5,
                    help="state 停顿窗口内 cmd 平段占比 ≥ 此值判'意图型'（默认 0.5）")
    ap.add_argument("--friction-thresh", type=float, default=0.2,
                    help="cmd 平段占比 ≤ 此值判'摩擦型'（默认 0.2）")
    ap.add_argument("--min-peak-v", type=float, default=0.1,
                    help="聚合只算峰值 cmd 速度 ≥ 该值(rad/s) 的'活跃关节'，"
                         "避免全程低速的小关节污染平段/分型（默认 0.1）")
    ap.add_argument("--no-align", action="store_true",
                    help="分型前不做 cmd→state 互相关对齐（默认对齐：消除两流时间戳语义"
                         "差异 ~200ms 造成的错分）")
    ap.add_argument("--out", help="JSON 输出路径")
    ap.add_argument("--plot", help="PNG 输出路径（state vs cmd + 平段/分型着色）")
    ap.add_argument("--self-test", action="store_true", help="合成数据自测")
    args = ap.parse_args()

    if args.self_test:
        return _self_test()

    rows = []
    if args.h5:
        for r in _collect_raw(args.h5):
            r["res"] = analyze_cmd_state(r["cmd"], r["cmd_ts"], r["state"], r["state_ts"],
                                         hz=args.hz, flat_v=args.flat_v,
                                         min_pause=args.min_pause, align=not args.no_align,
                                         intent_frac=args.intent_thresh,
                                         friction_frac=args.friction_thresh)
            rows.append(r)
    if args.aligned:
        import h5py

        with h5py.File(args.aligned, "r") as f:
            require_source_signature(f, str(Path(args.aligned).resolve().parent))
            action = np.asarray(f["action"], dtype=np.float64)
            state = (np.asarray(f["observation/state"], dtype=np.float64)
                     if "observation/state" in f else None)
            fps = int(f.attrs.get("fps", 30))
        rows.append({"h5": args.aligned, "stream": "action(next-state)",
                     "res": analyze_aligned(action, state, hz=fps, flat_v=args.flat_v,
                                            min_pause=args.min_pause)})

    if not rows:
        ap.error("need --h5 and/or --aligned")

    _print_report(rows, args.min_peak_v)
    if args.out:
        with open(args.out, "w") as f:
            json.dump([{"h5": r["h5"], "stream": r["stream"], "result": r["res"]} for r in rows],
                      f, indent=1, ensure_ascii=False)
        print(f"JSON -> {args.out}")
    if args.plot:
        try:
            _plot(rows, args)
        except ImportError:
            print("--plot 需要 matplotlib，跳过")
    return 0


def _plot(rows, args) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(rows)
    fig, axes = plt.subplots(n, 1, figsize=(14, 3.2 * n), squeeze=False)
    for ri, r in enumerate(rows):
        ax = axes[ri][0]
        res = r["res"]
        grid = _grid_from(r["cmd_ts"], r["state_ts"], args.hz)
        S = np.interp(grid, r["state_ts"], r["state"][:, 0])
        # cmd 曲线按分型时的对齐量平移（state 领先 cmd 的恒定量）——否则图上 state 仍
        # 显示领先 cmd ~200ms，与分型/摩擦特征错位。
        lag = res.get("joints", {}).get("j0", {}).get("lag_s", 0.0) if not args.no_align else 0.0
        C = np.interp(grid + lag, r["cmd_ts"], r["cmd"][:, 0])
        # 时间原点 = 分析时的 state 流起点（pauses 的 t0/t1 相对它）
        origin = res.get("t0_origin", grid[0])
        t = grid - origin
        ax.plot(t, S, "b-", lw=0.9, label="state")
        ax.plot(t, C, "r--", lw=0.8, label=f"cmd (对齐 {lag*1000:.0f}ms)")
        for jname, jj in res.get("joints", {}).items():
            if jname != "j0":
                continue
            for p in jj["pauses"]:
                color = {"intent": "green", "friction": "orange", "mixed": "gray"}[p["kind"]]
                ax.axvspan(p["t0"], p["t1"], color=color, alpha=0.15)
        ax.set_title(f"{r['h5']} :: {r['stream']}")
        ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(args.plot, dpi=110)
    print(f"plot -> {args.plot}")


if __name__ == "__main__":
    sys.exit(main())
