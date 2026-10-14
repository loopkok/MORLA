"""Two-stage PiM-IK pipeline for the Astral arm.

Stage 1 (a ``psi`` predictor, torch or synthetic) maps end-effector poses to
the arm angle ``psi``; stage 2 (``GeometricArmAngleSolver``) resolves the 7
joints deterministically. The pipeline is torch-agnostic: any object exposing
``predict(T_ee_seq) -> (T, 2)`` in numpy works, so the full two-stage flow can
be exercised end-to-end without torch via ``SyntheticPsiPredictor``.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from astral_pim_ik.geometry import GeometricArmAngleSolver

__all__ = ["PiMIKPipeline", "SyntheticPsiPredictor", "TorchPsiPredictor"]


class SyntheticPsiPredictor:
    """A perfect-stage-1 stand-in: returns stored ``psi`` labels.

    Lets the deterministic stage-2 path be tested and demonstrated without
    torch. ``predict`` returns the labels for the first ``T`` frames.
    """

    def __init__(self, psi_labels: np.ndarray):
        self.psi = np.asarray(psi_labels, dtype=float).reshape(-1, 2)

    def predict(self, T_ee_seq: np.ndarray) -> np.ndarray:
        T = np.asarray(T_ee_seq).shape[0]
        return self.psi[:T].copy()


class TorchPsiPredictor:
    """Wraps a torch ``PiM_IK_Net`` behind the numpy ``predict`` interface."""

    def __init__(self, model, device: str = "cpu"):
        import torch  # deferred so importing this class needs torch

        self._torch = torch
        self.model = model
        self.device = device
        self.model.eval()

    def predict(self, T_ee_seq: np.ndarray) -> np.ndarray:
        torch = self._torch
        T = np.asarray(T_ee_seq, dtype=np.float32)
        with torch.no_grad():
            x = torch.from_numpy(T).unsqueeze(0).to(self.device)  # (1, T, 4, 4)
            psi = self.model(x)[0].cpu().numpy()                   # (T, 2)
        return psi


class PiMIKPipeline:
    """Two-stage IK: ``psi = predict(T_ee_seq)`` then ``q = solve(T_ee, psi)``.

    Frame-by-frame the geometric solve is warm-started from the previous frame's
    solution (mirrors PiM-IK's ``HierarchicalIKSolver`` warm-start).
    """

    def __init__(
        self,
        predictor,
        arm_side: str = "left",
        urdf_path: str = "",
    ):
        self.predictor = predictor
        self.solver = GeometricArmAngleSolver(arm_side, urdf_path)

    def solve_frame(
        self,
        T_ee: np.ndarray,
        psi: float,
        q_init: Optional[np.ndarray] = None,
        fallback_scan: bool = False,
        n_scan: int = 36,
    ) -> Optional[np.ndarray]:
        return self.solver.solve(
            np.asarray(T_ee, dtype=float), float(psi), q_init,
            fallback_scan=fallback_scan, n_scan=n_scan,
        )

    def solve_trajectory(
        self,
        T_ee_seq: np.ndarray,
        q_init: Optional[np.ndarray] = None,
        fallback_scan: bool = False,
        n_scan: int = 36,
    ) -> Tuple[List[Optional[np.ndarray]], np.ndarray]:
        """Solve a trajectory.

        Args:
            T_ee_seq: (T, 4, 4) flange poses in ``*_base_link``.
            q_init: (7,) optional warm-start for the first frame.
            fallback_scan: rescue an infeasible predicted ``psi`` by scanning
                for the nearest feasible arm angle (see ``GeometricArmAngleSolver.solve``).
                Off by default; rescues are counted in ``self.solver.fallback_count``.
            n_scan: grid size of the fallback scan.

        Returns:
            (q_seq, psi_seq): q_seq is a list of (7,) arrays or ``None``;
            psi_seq is the (T, 2) predicted arm-angle unit vector.
        """
        T_ee_seq = np.asarray(T_ee_seq, dtype=float)
        psi_seq = np.asarray(self.predictor.predict(T_ee_seq), dtype=float)
        q_seq: List[Optional[np.ndarray]] = []
        prev = np.asarray(q_init, dtype=float) if q_init is not None else None
        for i in range(len(T_ee_seq)):
            psi = float(np.arctan2(psi_seq[i, 1], psi_seq[i, 0]))
            q = self.solver.solve(
                T_ee_seq[i], psi, q_init=prev,
                fallback_scan=fallback_scan, n_scan=n_scan,
            )
            q_seq.append(q)
            if q is not None:
                prev = q
        return q_seq, psi_seq

    def evaluate(
        self, T_ee_seq: np.ndarray, q_seq: Sequence[Optional[np.ndarray]]
    ) -> dict:
        """Pose-error summary of a solved trajectory (for tests / reporting)."""
        pos_mm = []
        for T, q in zip(T_ee_seq, q_seq):
            if q is None:
                continue
            T_sol = self.solver.fk(q)
            pos_mm.append(float(np.linalg.norm(T_sol[:3, 3] - np.asarray(T)[:3, 3]) * 1000.0))
        if not pos_mm:
            return {"solved": 0, "total": len(q_seq), "max_pos_mm": float("nan")}
        return {
            "solved": len(pos_mm),
            "total": len(q_seq),
            "max_pos_mm": max(pos_mm),
            "mean_pos_mm": float(np.mean(pos_mm)),
        }
