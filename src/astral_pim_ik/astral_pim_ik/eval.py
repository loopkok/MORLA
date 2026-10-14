#!/usr/bin/env python3
"""Evaluate a trained PiM_IK_Net against the deterministic stage-2 solver.

Loads a checkpoint and the dataset ``.npz``, predicts the arm angle ``psi`` on
the val split (the last 5% of frames, matching ``SwivelSequenceDataset``),
resolves the 7 joints with ``GeometricArmAngleSolver`` frame-by-frame
(warm-started from the ground-truth previous frame), and reports:

  - psi angular error (deg)
  - elbow error (mm, via the same circle basis the loss supervises)
  - end-to-end pose error (mm) and joint MAE (deg) through the IK
  - success rate (solve != None and pose error < 1 mm)

An "oracle psi" baseline (solve with the ground-truth label instead of the
prediction) is reported alongside, so the psi-prediction contribution is
separated from the IK's own residual error. Requires torch + pinocchio (e.g.
the lerobot env).

Usage:
  PYTHONPATH=src/astral_pim_ik:src/astral_arm_teleop python3 astral_pim_ik/eval.py \
      --checkpoint checkpoints/best_transformer_L4_w15.pth --data_path d.npz
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from astral_pim_ik.geometry import GeometricArmAngleSolver  # noqa: E402
from astral_pim_ik.network import PiM_IK_Net  # noqa: E402


def _wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--checkpoint", type=str, required=True)
    p.add_argument("--data_path", type=str, required=True)
    p.add_argument("--arm", type=str, default="left", choices=["left", "right"])
    p.add_argument("--window_size", type=int, default=None,
                   help="defaults to the checkpoint's training window_size")
    p.add_argument("--no_split", action="store_true",
                   help="evaluate the whole npz as val (for session-held-out npz "
                        "produced by convert_sessions.py); default is the last-5%% time split")
    p.add_argument("--device", type=str, default="cuda")
    return p.parse_args()


def load_model(ckpt, device) -> PiM_IK_Net:
    a = ckpt.get("args", {})
    model = PiM_IK_Net(
        d_model=a.get("d_model", 256),
        num_layers=a.get("num_layers", 4),
        backbone_type=a.get("backbone", "transformer"),
        dropout=a.get("dropout", 0.1),
    ).to(device)
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()
    return model


def solve_window(
    solver: GeometricArmAngleSolver,
    T_ee: np.ndarray,
    psi_units: np.ndarray,
    psi_labels: np.ndarray,
    q: np.ndarray,
    E_gt: np.ndarray,
    W: int,
) -> Dict[str, float]:
    """Solve each window's last frame with ``psi_units``; collect metrics."""
    nwin = len(psi_units)
    dpsi_deg: List[float] = []
    elbow_mm: List[float] = []
    pos_mm: List[float] = []
    joint_deg: List[float] = []
    solved = 0
    pose_ok = 0
    for i in range(nwin):
        j = i + W - 1  # val-frame index of the window's last frame
        psi_pred = float(np.arctan2(psi_units[i, 1], psi_units[i, 0]))
        psi_gt = float(np.arctan2(psi_labels[j, 1], psi_labels[j, 0]))
        dpsi_deg.append(abs(float(_wrap(np.array([psi_pred - psi_gt]))[0])) * 180.0 / math.pi)

        E_pred = solver.elbow_from_psi(T_ee[j], psi_pred)
        if E_pred is not None:
            elbow_mm.append(float(np.linalg.norm(E_pred - E_gt[j]) * 1000.0))

        q_sol = solver.solve(T_ee[j], psi_pred, q_init=q[j - 1])
        if q_sol is None:
            continue
        solved += 1
        pos = float(np.linalg.norm(solver.fk(q_sol)[:3, 3] - T_ee[j][:3, 3]) * 1000.0)
        pos_mm.append(pos)
        if pos < 1.0:
            pose_ok += 1
        joint_deg.append(float(np.mean(np.abs(_wrap(q_sol - q[j])))) * 180.0 / math.pi)

    def mean(xs):
        return float(np.mean(xs)) if xs else float("nan")

    return {
        "n": float(nwin),
        "psi_err_deg": mean(dpsi_deg),
        "elbow_mm": mean(elbow_mm),
        "pose_mm_mean": mean(pos_mm),
        "pose_mm_max": float(np.max(pos_mm)) if pos_mm else float("nan"),
        "joint_mae_deg": mean(joint_deg),
        "solve_rate": solved / nwin if nwin else float("nan"),
        "pose_ok_rate": pose_ok / nwin if nwin else float("nan"),
    }


def main() -> None:
    args = parse_args()
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    ckpt = torch.load(args.checkpoint, map_location=device, weights_only=False)
    W = args.window_size or int(ckpt.get("args", {}).get("window_size", 15))

    data = np.load(args.data_path, allow_pickle=True)
    T_ee = data["T_ee"].astype(np.float32)
    psi = data["psi"].astype(np.float32)
    q = data["q"].astype(np.float32)
    E_gt = data["joint_positions"][:, 1, :].astype(np.float32)

    split = 0 if args.no_split else int(len(T_ee) * 0.95)
    T_ee, psi, q, E_gt = T_ee[split:], psi[split:], q[split:], E_gt[split:]
    T = len(T_ee)
    nwin = T - W + 1

    model = load_model(ckpt, device)
    solver = GeometricArmAngleSolver(args.arm)

    windows = np.stack([T_ee[i:i + W] for i in range(nwin)])  # (nwin, W, 4, 4)
    with torch.no_grad():
        pred = model(torch.from_numpy(windows).to(device))[:, -1].cpu().numpy()  # (nwin, 2)

    pred_m = solve_window(solver, T_ee, pred, psi, q, E_gt, W)
    orac_m = solve_window(solver, T_ee, psi[W - 1:], psi, q, E_gt, W)  # label psi (oracle)

    print("Astral PiM-IK — eval summary")
    print(f"  arm={args.arm} backbone={ckpt.get('args', {}).get('backbone')} "
          f"window={W} val_frames={T} windows={nwin}")
    print(f"  {'metric':<16}{'predicted':>12}{'oracle':>12}")
    for key, label in [
        ("psi_err_deg", "psi err (deg)"),
        ("elbow_mm", "elbow (mm)"),
        ("pose_mm_mean", "pose mean (mm)"),
        ("pose_mm_max", "pose max (mm)"),
        ("joint_mae_deg", "joint MAE (deg)"),
        ("solve_rate", "solve rate"),
        ("pose_ok_rate", "pose<1mm rate"),
    ]:
        print(f"  {label:<16}{pred_m[key]:>12.4f}{orac_m[key]:>12.4f}")


if __name__ == "__main__":
    main()
