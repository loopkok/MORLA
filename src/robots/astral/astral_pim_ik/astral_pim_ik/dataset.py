"""Dataset generation (numpy) + sliding-window torch Dataset for the Astral arm.

``generate_dataset`` samples joint configs, runs forward kinematics, and writes
the arm-angle label ``psi`` (plus shoulder/elbow/wrist and link lengths) to a
``.npz`` in the same spirit as the PiM-IK reference dataset. ``SwivelSequenceDataset``
is the torch sliding-window reader used by ``train.py``.

The label ``psi`` is computed with the *same* convention as the geometric
solver (``psi_from_elbow_dir`` on the S-W axis), so stage 1 and stage 2 agree.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Optional

import numpy as np

try:
    import torch
    from torch.utils.data import Dataset as _TorchDataset

    _TORCH = True
except ImportError:  # pragma: no cover - torch optional
    torch = None
    _TorchDataset = object
    _TORCH = False

from astral_arm_teleop.ik.geometric import _circle_basis_sw, _compute_sw
from astral_pim_ik.geometry import GeometricArmAngleSolver, elbow_position, psi_from_config

__all__ = [
    "generate_dataset",
    "generate_trajectory_dataset",
    "SwivelSequenceDataset",
    "sample_configs",
]


def sample_configs(
    solver: GeometricArmAngleSolver,
    n: int,
    rng: np.random.Generator,
    margin: float = 0.05,
) -> np.ndarray:
    """Random configs clear of generic SRS singularities.

    Rejects the shoulder PK2 double-root (|q2| ~ pi/2), a near-straight elbow
    (q4 > -0.30, so the elbow is observably bent), and the wrist PK1 double-root
    (|q6| ~ 0). Same acceptance rule as ``test_geometric_ik._sample_q``.
    """
    lo = solver.lower_limits + margin
    hi = solver.upper_limits - margin
    out: list = []
    while len(out) < n:
        q = rng.uniform(lo, hi)
        if abs(abs(q[1]) - 0.5 * math.pi) < 0.25:
            continue
        if q[3] > -0.30:
            continue
        if abs(q[5]) < 0.15:
            continue
        out.append(q)
    return np.asarray(out, dtype=float)


def generate_dataset(
    arm_side: str = "left",
    urdf_path: str = "",
    n: int = 20000,
    out_npz: str = "astral_arm_angle_dataset.npz",
    seed: int = 0,
    min_sin_alpha: float = 0.05,
) -> Dict[str, np.ndarray]:
    """Sample configs, FK, and write the arm-angle dataset.

    Arrays written (all float32):
      T_ee           (N, 4, 4)  flange pose in ``*_base_link``
      psi            (N, 2)     arm-angle label [cos psi, sin psi]
      joint_positions(N, 3, 3)  [shoulder, elbow, wrist] in ``*_base_link``
      L_upper        (N,)       l_se (upper-arm length)
      L_lower        (N,)       l_ew (forearm length)
      is_valid       (N,)       1.0 when psi is observable, else 0.0
      q              (N, 7)     the source joint config (debug / round-trip)

    ``min_sin_alpha`` marks the elbow-straight singularity (orbit-circle radius
    ~0 -> psi numerically meaningless) as invalid, mirroring PiM-IK's mask.
    """
    solver = GeometricArmAngleSolver(arm_side, urdf_path)
    g = solver.geom
    rng = np.random.default_rng(seed)

    qs = sample_configs(solver, n, rng)
    T_ee = np.empty((n, 4, 4), dtype=np.float32)
    psi_arr = np.empty((n, 2), dtype=np.float32)
    joint_pos = np.empty((n, 3, 3), dtype=np.float32)
    is_valid = np.zeros(n, dtype=np.float32)

    kept = 0
    for q in qs:
        T = solver.fk(q)
        S, W, q4_list = _compute_sw(T, g)
        if not q4_list:
            continue
        basis = _circle_basis_sw(S, W, g)
        if basis is None:
            continue
        r = basis[12]
        sin_alpha = r / g.l_se if g.l_se > 0 else 0.0
        psi = psi_from_config(q, g)
        if psi is None:
            continue
        E = elbow_position(q, g)
        T_ee[kept] = T.astype(np.float32)
        psi_arr[kept] = np.array([math.cos(psi), math.sin(psi)], dtype=np.float32)
        joint_pos[kept, 0] = S
        joint_pos[kept, 1] = E
        joint_pos[kept, 2] = W
        is_valid[kept] = 1.0 if sin_alpha >= min_sin_alpha else 0.0
        kept += 1
        if kept >= n:
            break

    out = {
        "T_ee": T_ee[:kept],
        "psi": psi_arr[:kept],
        "joint_positions": joint_pos[:kept],
        "L_upper": np.full(kept, g.l_se, dtype=np.float32),
        "L_lower": np.full(kept, g.l_ew, dtype=np.float32),
        "is_valid": is_valid[:kept],
        "q": qs[:kept].astype(np.float32),
    }
    if out_npz:
        Path(out_npz).parent.mkdir(parents=True, exist_ok=True)
        np.savez(out_npz, **out)
    return out


def generate_trajectory_dataset(
    arm_side: str = "left",
    urdf_path: str = "",
    n_traj: int = 200,
    traj_len: int = 100,
    out_npz: str = "astral_arm_angle_traj_dataset.npz",
    seed: int = 0,
    min_sin_alpha: float = 0.05,
    travel: float = 0.35,
    wobble: float = 0.08,
) -> Dict[str, np.ndarray]:
    """Temporally-coherent smooth trajectories — the dataset for stage-1 training.

    Stage-1 training needs time coherence: a single ``T_ee`` does not determine
    ``psi`` (the elbow is free on the S-W orbit circle — any feasible ``psi``
    reaches the same wrist pose), and the windowed network disambiguates the
    elbow from *how the wrist moves across the window* (mirroring PiM-IK's
    recorded-teleop data). ``generate_dataset`` draws IID random configs, whose
    windows carry no such signal — measured: psi err stuck at ~53 deg on IID no
    matter the loss, while coherent smooth drifts let the same model reach
    ~20-40 deg at 5k frames / 30 epochs (residual error tracks trajectory
    speed; keep ``travel`` modest like real teleop motion). Use this function
    for stage-1 training; ``generate_dataset`` remains for stage-2 label tests.

    Each trajectory starts at an accepted random config and drifts by a bounded
    linear term (per joint ``+-travel`` of the joint span) plus a slow sine
    wobble (``wobble`` of the span), clipped to the joint limits — locally
    smooth like real arm motion, concatenated like teleop sessions. Same .npz
    schema as ``generate_dataset``.
    """
    solver = GeometricArmAngleSolver(arm_side, urdf_path)
    g = solver.geom
    rng = np.random.default_rng(seed)
    lo = solver.lower_limits + 0.05
    hi = solver.upper_limits - 0.05
    span = hi - lo

    frames_est = n_traj * traj_len
    T_ee = np.empty((frames_est, 4, 4), dtype=np.float32)
    psi_arr = np.empty((frames_est, 2), dtype=np.float32)
    joint_pos = np.empty((frames_est, 3, 3), dtype=np.float32)
    is_valid = np.zeros(frames_est, dtype=np.float32)
    q_arr = np.empty((frames_est, 7), dtype=np.float32)

    kept = 0
    for _ in range(n_traj):
        q0 = sample_configs(solver, 1, rng)[0]   # accepted (clear of SRS)
        drift = rng.uniform(-1.0, 1.0, 7) * (travel * span)
        freq = rng.uniform(0.05, 0.5, 7)          # sine cycles over the traj
        phase = rng.uniform(0.0, 2.0 * math.pi, 7)
        amp = wobble * span
        for t in np.linspace(0.0, 1.0, traj_len):
            q = q0 + drift * t + amp * np.sin(2.0 * math.pi * freq * t + phase)
            q = np.clip(q, lo, hi)
            T = solver.fk(q)
            S, W, q4_list = _compute_sw(T, g)
            if not q4_list:
                continue
            basis = _circle_basis_sw(S, W, g)
            if basis is None:
                continue
            psi = psi_from_config(q, g)
            if psi is None:
                continue
            E = elbow_position(q, g)
            T_ee[kept] = T.astype(np.float32)
            psi_arr[kept] = np.array([math.cos(psi), math.sin(psi)], dtype=np.float32)
            joint_pos[kept, 0] = S
            joint_pos[kept, 1] = E
            joint_pos[kept, 2] = W
            is_valid[kept] = 1.0 if (basis[12] / g.l_se if g.l_se > 0 else 0.0) >= min_sin_alpha else 0.0
            q_arr[kept] = q.astype(np.float32)
            kept += 1

    out = {
        "T_ee": T_ee[:kept],
        "psi": psi_arr[:kept],
        "joint_positions": joint_pos[:kept],
        "L_upper": np.full(kept, g.l_se, dtype=np.float32),
        "L_lower": np.full(kept, g.l_ew, dtype=np.float32),
        "is_valid": is_valid[:kept],
        "q": q_arr[:kept],
    }
    if out_npz:
        Path(out_npz).parent.mkdir(parents=True, exist_ok=True)
        np.savez(out_npz, **out)
    return out


class SwivelSequenceDataset(_TorchDataset):
    """Sliding-window torch Dataset over a generated ``.npz`` (PiM-IK style).

    Train/val split is by time (first 95% train, last 5% val, no shuffle across
    the boundary) to avoid temporal leakage, matching the reference trainer.
    """

    def __init__(
        self,
        npz_path: str,
        window_size: int = 30,
        train: bool = True,
        stride: int = 1,
    ):
        if not _TORCH:
            raise ImportError("SwivelSequenceDataset requires torch")
        self.window_size = window_size
        self.train = train
        self.stride = stride

        data = np.load(npz_path, allow_pickle=True)
        self.T_ee = data["T_ee"].astype(np.float32)
        self.psi = data["psi"].astype(np.float32)
        self.joint_positions = data["joint_positions"].astype(np.float32)
        self.L_upper = data["L_upper"].astype(np.float32)
        self.L_lower = data["L_lower"].astype(np.float32)
        self.is_valid = data["is_valid"].astype(np.float32)

        self.total_frames = len(self.T_ee)
        split = int(self.total_frames * 0.95)
        if train:
            self.start_idx, self.end_idx = 0, split
        else:
            self.start_idx, self.end_idx = split, self.total_frames

        available = self.end_idx - self.start_idx - window_size
        self.num_samples = max(0, available) // self.stride + 1

    def __len__(self) -> int:
        return self.num_samples

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        start = self.start_idx + idx * self.stride
        end = start + self.window_size
        jp = self.joint_positions[start:end]
        return {
            "T_ee": torch.from_numpy(self.T_ee[start:end]),
            "gt_psi": torch.from_numpy(self.psi[start:end]),
            "p_s": torch.from_numpy(jp[:, 0, :]),
            "p_e_gt": torch.from_numpy(jp[:, 1, :]),
            "p_w": torch.from_numpy(jp[:, 2, :]),
            "L_upper": torch.from_numpy(self.L_upper[start:end]),
            "L_lower": torch.from_numpy(self.L_lower[start:end]),
            "is_valid": torch.from_numpy(self.is_valid[start:end]),
        }
