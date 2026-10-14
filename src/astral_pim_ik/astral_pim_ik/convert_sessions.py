#!/usr/bin/env python3
"""Convert recorded teleop sessions (astral_data_collect robot_data.h5) into
the astral_pim_ik training/val .npz schema.

A session directory is ``~/astral_data/<task>/episodeNNNNNN/{meta.json,
robot_data.h5}``. For one arm side this script reads the executed joint
stream ``streams/{side}_arm_state`` and derives, per frame:

    q (measured joints) --FK--> T_ee (4x4, the network input)
                          --psi_from_config--> psi label [cos, sin]
                          (same convention as the geometric solver, so a
                          hard-elbow teleop session yields labels that are
                          exactly the fed human arm angle)

Episodes are cut into continuous segments at meta pause/resume events and at
large per-frame joint jumps; frames with unobservable psi are marked
``is_valid = 0`` (the loss masks them) instead of being dropped, so window
contiguity is preserved. Train/val split is by whole session directory
(``--val_sessions``), never by time inside an episode.

Usage:
  PYTHONPATH=src/astral_arm_teleop:src/astral_pim_ik python3 \
      astral_pim_ik/convert_sessions.py --sessions ~/astral_data/default_task \
      --side left --out_dir ~/pimik_data \
      [--val_sessions verify_80741] [--min_frames 100]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import h5py
import numpy as np

from astral_arm_teleop.ik.geometric import _circle_basis_sw, _compute_sw
from astral_pim_ik.geometry import GeometricArmAngleSolver, elbow_position

# A per-frame joint jump above this (rad) cuts the segment — a teleop drop /
# hold-frame burst / reset, not arm motion. Real motion at 30 fps stays far
# below it; pim_ik's smoothness test uses the same order of threshold.
JUMP_TOL = 0.35


def _wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _running_windows(events: List[dict], t0: float, t1: float):
    """(start, end) timestamp windows inside which recording was not paused."""
    windows: List[Tuple[float, float]] = []
    running = True
    win_start = t0
    for e in events:
        name = str(e.get("name", ""))
        if name == "pause":
            if running:
                windows.append((win_start, float(e["t"])))
            running = False
        elif name == "resume":
            if not running:
                running = True
                win_start = float(e["t"])
    if running:
        windows.append((win_start, t1))
    return windows


def _segments(
    solver: GeometricArmAngleSolver,
    qs: np.ndarray,
    ts: np.ndarray,
    windows: List[Tuple[float, float]],
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Split frames into continuous segments (event windows + jump cuts)."""
    lo = solver.lower_limits - 1e-4
    hi = solver.upper_limits + 1e-4
    segs: List[Tuple[np.ndarray, np.ndarray]] = []
    for (a, b) in windows:
        idx = np.where((ts >= a - 1e-9) & (ts <= b + 1e-9))[0]
        if len(idx) == 0:
            continue
        # cut inside the window at large per-frame jumps / junk frames
        cut = [idx[0]]
        for i in range(1, len(idx)):
            prev, cur = idx[i - 1], idx[i]
            if (not np.isfinite(qs[cur]).all()
                    or np.any(qs[cur] < lo) or np.any(qs[cur] > hi)):
                # NaN / out-of-URDF-limits frame: real teleop never executes
                # these; cut it out of both neighboring segments.
                cut.append(cur)
                cut.append(cur)
                continue
            if float(np.max(np.abs(_wrap(qs[cur] - qs[prev])))) > JUMP_TOL:
                cut.append(cur)  # ends previous segment, starts next
        cut.append(idx[-1] + 1)
        for i in range(len(cut) - 1):
            s, e = int(cut[i]), int(cut[i + 1])
            if e - s >= 2:
                segs.append((qs[s:e].copy(), ts[s:e].copy()))
    return segs


def _derive(
    solver: GeometricArmAngleSolver,
    g,
    q_seg: np.ndarray,
) -> Dict[str, np.ndarray]:
    n = len(q_seg)
    T_ee = np.empty((n, 4, 4), dtype=np.float32)
    psi = np.empty((n, 2), dtype=np.float32)
    jp = np.empty((n, 3, 3), dtype=np.float32)
    iv = np.zeros(n, dtype=np.float32)
    q_out = np.empty((n, 7), dtype=np.float32)

    for i, q in enumerate(q_seg):
        q_out[i] = q.astype(np.float32)
        T = solver.fk(q)
        T_ee[i] = T.astype(np.float32)
        S, W, q4_list = _compute_sw(T, g)
        if not q4_list:
            continue
        basis = _circle_basis_sw(S, W, g)
        if basis is None:
            continue
        E = elbow_position(q, g)
        jp[i, 0] = S
        jp[i, 1] = E
        jp[i, 2] = W
        p = solver.psi_from_config(q)
        if p is None:
            continue
        r = basis[12]
        sin_alpha = r / g.l_se if g.l_se > 0 else 0.0
        psi[i] = (math.cos(p), math.sin(p))
        iv[i] = 1.0 if sin_alpha >= 0.05 else 0.0
    return {
        "T_ee": T_ee, "psi": psi, "joint_positions": jp,
        "L_upper": np.full(n, g.l_se, dtype=np.float32),
        "L_lower": np.full(n, g.l_ew, dtype=np.float32),
        "is_valid": iv, "q": q_out,
    }


def collect_episodes(session_dirs: List[Path]) -> List[Path]:
    eps = []
    for d in session_dirs:
        for ep in sorted(d.glob("episode*")):
            if (ep / "robot_data.h5").is_file():
                eps.append(ep)
    return eps


def convert_episode(ep: Path, side: str, solver) -> Optional[Dict[str, np.ndarray]]:
    stream = f"streams/{side}_arm_state"
    with h5py.File(ep / "robot_data.h5", "r") as f:
        if stream not in f:
            raise ValueError(
                f"{ep}: no stream {stream!r}; available: "
                + ", ".join(sorted(k for k in f if k.startswith("streams/")))
            )
        ts = np.asarray(f[f"{stream}/timestamps"], dtype=float)
        qs = np.asarray(f[f"{stream}/values"], dtype=float)
    meta = json.loads((ep / "meta.json").read_text())
    events = meta.get("events", [])
    t0 = float(ts[0]) if len(ts) else float(meta.get("start_time_wall", 0.0))
    t1 = float(ts[-1]) if len(ts) else float(meta.get("end_time_wall", t0))
    segs = _segments(solver, qs, ts, _running_windows(events, t0, t1))
    parts = []
    for q_seg, _t in segs:
        parts.append(_derive(solver, solver.geom, q_seg))
    return _concat(parts)


def _concat(parts: List[Dict[str, np.ndarray]]) -> Optional[Dict[str, np.ndarray]]:
    if not parts:
        return None
    out = {}
    for k in parts[0]:
        out[k] = np.concatenate([p[k] for p in parts], axis=0)
    return out


def _stack(ep_data: List[Optional[Dict[str, np.ndarray]]]):
    eps = [d for d in ep_data if d is not None and len(d["T_ee"]) > 0]
    if not eps:
        return None
    out = {}
    for k in eps[0]:
        out[k] = np.concatenate([e[k] for e in eps], axis=0)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=__doc__.split("Usage:")[0],
    )
    ap.add_argument("--sessions", nargs="+", required=True,
                    help="session dir(s) (each containing episode*/robot_data.h5)")
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--out_dir", type=str, required=True)
    ap.add_argument("--val_sessions", nargs="*", default=[],
                    help="session names whose episodes go to val.npz (whole-session hold-out)")
    ap.add_argument("--min_frames", type=int, default=100,
                    help="drop episodes shorter than this")
    args = ap.parse_args()

    solver = GeometricArmAngleSolver(args.side)
    session_dirs = []
    for s in args.sessions:
        p = Path(s).expanduser()
        session_dirs.append(p)
    episodes = collect_episodes(session_dirs)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    train_eps, val_eps = [], []
    for ep in episodes:
        owner = ep.parent.name
        (val_eps if owner in args.val_sessions else train_eps).append(ep)

    def convert_all(eps: List[Path]) -> Optional[Dict[str, np.ndarray]]:
        datas = []
        for ep in eps:
            d = convert_episode(ep, args.side, solver)
            if d is None or len(d["T_ee"]) < args.min_frames:
                print(f"  {ep.parent.name}/{ep.name}: skipped "
                      f"({0 if d is None else len(d['T_ee'])} frames < {args.min_frames})")
                continue
            print(f"  {ep.parent.name}/{ep.name}: {len(d['T_ee'])} frames, "
                  f"valid {int(d['is_valid'].sum())}")
            datas.append(d)
        return _stack(datas)

    for tag, eps in (("train", train_eps), ("val", val_eps)):
        if not eps:
            print(f"[{tag}] no episodes")
            continue
        d = convert_all(eps)
        if d is None:
            print(f"[{tag}] nothing after filtering")
            continue
        out_path = out_dir / f"{args.side}_{tag}.npz"
        np.savez(out_path, **d)
        print(f"[{tag}] {len(d['T_ee'])} frames -> {out_path}")


if __name__ == "__main__":
    main()
