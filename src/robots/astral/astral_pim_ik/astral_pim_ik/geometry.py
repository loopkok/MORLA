"""Deterministic arm-angle IK for the Astral arm — PiM-IK stage 2.

Thin wrapper over the DH-free S/E/W arm-angle solver in
``astral_arm_teleop.ik.geometric``. That solver already contains a clean
closed-form mapping ``(T_ee, psi) -> q`` via POE + Paden-Kahan; the teleop
solver ``GeometricIKSolver`` layers a lot of VR-teleop machinery on top of it
(escape hysteresis, soft ``psi_ref`` prior, singular-hold, 1D QP, weighted
continuity scoring, reach clamp). This module keeps only the *deterministic*
core: given the flange pose and the arm angle ``psi``, return the joint
vector ``q``, filtered by hard joint limits, with branch disambiguation done
by a minimal warm-start (nearest previous ``q``) — mirroring the warm-start in
PiM-IK's ``HierarchicalIKSolver``, not the teleop continuity scoring.

``psi`` is the elbow's angle on the orbit circle around the shoulder->wrist
axis, with the same ``psi = 0`` convention as ``geometric._circle_basis_sw``
(``psi_ref = S``). This convention is shared by the dataset label, the
differentiable kinematics layer, and ``psi_from_elbow_dir`` — do not redefine
it anywhere else or the network and the solver will disagree silently.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from astral_arm_teleop.ik.analytic import wrap_to_pi
from astral_arm_teleop.ik.geometric import (
    ArmGeometry,
    _axis_angle_rot,
    _collect_solutions,
    _compute_sw,
    _elbow_point,
    extract_arm_geometry,
    fk as _fk_poe,
    psi_from_elbow_dir,
)

__all__ = [
    "GeometricArmAngleSolver",
    "elbow_position",
    "psi_from_config",
]


def elbow_position(q: np.ndarray, g: ArmGeometry) -> np.ndarray:
    """Elbow center under joints 1-3 (q4 spins about E and cannot move it)."""
    q = np.asarray(q, dtype=float).reshape(7)
    T = np.eye(4)
    I3 = np.eye(3)
    for i in range(3):
        R = _axis_angle_rot(g.axes[i], float(q[i]))
        Ti = np.eye(4)
        Ti[:3, :3] = R
        Ti[:3, 3] = (I3 - R) @ g.points[i]
        T = T @ Ti
    return (T @ np.append(g.E0, 1.0))[:3]


def psi_from_config(q: np.ndarray, g: ArmGeometry) -> Optional[float]:
    """Arm angle ``psi`` of a joint configuration (the dataset label).

    Returns ``None`` when the target is unreachable or ``psi`` is unobservable
    (elbow ~on the S-W axis).
    """
    q = np.asarray(q, dtype=float).reshape(7)
    T = _fk_poe(q, g)
    S, W, q4_list = _compute_sw(T, g)
    if not q4_list:
        return None
    E = elbow_position(q, g)
    return psi_from_elbow_dir(S, W, E - S, g)


class GeometricArmAngleSolver:
    """Deterministic ``(T_ee, psi) -> q`` for one Astral arm in ``*_base_link``.

    Same ``solve`` / ``fk`` / ``limits`` shape as ``GeometricIKSolver`` so it
    can be dropped in later as a ``solver_type``, but with no internal state:
    ``solve`` is a pure function of ``(T_ee, psi, q_init)``.
    """

    def __init__(self, arm_side: str = "left", urdf_path: str = ""):
        side = arm_side.strip().lower()
        if side.startswith("l"):
            self.arm = "L"
            self._prefix = "left"
        elif side.startswith("r"):
            self.arm = "R"
            self._prefix = "right"
        else:
            raise ValueError("arm_side must be left|right")
        self.geom = extract_arm_geometry(urdf_path, arm_side)
        # Observability counter: how many times the opt-in fallback scan
        # rescued an infeasible psi. Mutated by solve() only; never affects
        # the returned joints.
        self.fallback_count = 0

    # ---- Properties ----

    @property
    def method_name(self) -> str:
        return f"pim_ik_geometric/{self._prefix}"

    @property
    def nq(self) -> int:
        return 7

    @property
    def lower_limits(self) -> np.ndarray:
        return self.geom.lower.copy()

    @property
    def upper_limits(self) -> np.ndarray:
        return self.geom.upper.copy()

    def active_joint_names(self) -> List[str]:
        return [f"{self._prefix}_joint{i}" for i in range(1, 8)]

    # ---- Core methods ----

    def fk(self, q: np.ndarray) -> np.ndarray:
        """FK: joints -> flange pose in ``left_base_link`` / ``right_base_link``."""
        return _fk_poe(np.asarray(q, dtype=float).reshape(7), self.geom)

    def candidates(self, T_ee: np.ndarray, psi: float) -> List[np.ndarray]:
        """All joint-limit-feasible solutions at an exact arm angle ``psi``.

        Each row is ``[q(7), psi, S(3), W(3), E(3)]`` (the geometric solver's
        internal layout). Empty when unreachable, out of limits, or ``psi`` is
        singular.
        """
        T = np.asarray(T_ee, dtype=float)
        S, W, q4_list = _compute_sw(T, self.geom)
        if not q4_list:
            return []
        return _collect_solutions(T, self.geom, S, W, q4_list, np.array([float(psi)]))

    def _nearest_feasible(
        self, T_ee: np.ndarray, psi: float, n_scan: int
    ) -> Tuple[List[np.ndarray], Optional[float]]:
        """Scan the arm-angle circle for the feasible ``psi`` nearest ``psi``.

        Returns ``(cands, nearest_psi)`` — ``([], None)`` when no grid point is
        feasible. Deterministic: circular distance via ``wrap_to_pi`` and the
        first grid point wins ties (grid order).
        """
        T = np.asarray(T_ee, dtype=float)
        grid = np.linspace(-np.pi, np.pi, n_scan, endpoint=False)
        best_d: Optional[float] = None
        best_cands: List[np.ndarray] = []
        best_psi: Optional[float] = None
        for psi_g in grid:
            cands = self.candidates(T, float(psi_g))
            if not cands:
                continue
            d = abs(float(wrap_to_pi(psi_g - psi)))
            if best_d is None or d < best_d:
                best_d = d
                best_cands = cands
                best_psi = float(psi_g)
        return best_cands, best_psi

    def solve(
        self,
        T_ee: np.ndarray,
        psi: float,
        q_init: Optional[np.ndarray] = None,
        fallback_scan: bool = False,
        n_scan: int = 36,
    ) -> Optional[np.ndarray]:
        """Deterministic IK: flange pose + arm angle -> 7 joints.

        Args:
            T_ee: (4, 4) flange pose in ``*_base_link``.
            psi: arm angle (rad) — the NN's prediction (or a label).
            q_init: (7,) previous joint state for branch disambiguation
                (warm-start, mirrors PiM-IK's ``q_init``). When ``None`` the
                most-bent-elbow branch (min ``q4``) is chosen deterministically.
            fallback_scan: when ``True`` and the exact ``psi`` is infeasible
                (out of joint limits / singular), scan the arm-angle circle for
                the nearest feasible ``psi`` and solve there instead of failing.
                Off by default to keep the strict ``None``-on-infeasible
                contract; a rescue is observable via ``self.fallback_count``.
            n_scan: grid size of the fallback scan over ``[-pi, pi)``.

        Returns:
            q (7,) or ``None`` when unreachable / out of limits / singular
            (and no feasible ``psi`` exists even under ``fallback_scan``).
        """
        cands = self.candidates(T_ee, psi)
        if not cands and fallback_scan:
            cands, _ = self._nearest_feasible(T_ee, psi, n_scan)
            if cands:
                self.fallback_count += 1
        if not cands:
            return None
        if q_init is not None:
            q_prev = np.asarray(q_init, dtype=float).reshape(7)
            best = min(cands, key=lambda c: float(np.linalg.norm(wrap_to_pi(c[:7] - q_prev))))
        else:
            # Deterministic default: elbow-most-bent (most negative q4). This
            # is the natural hanging posture and avoids the straight-elbow
            # singularity; documented, not a teleop constraint.
            best = min(cands, key=lambda c: float(c[3]))
        return np.asarray(best[:7], dtype=float).copy()

    def psi_from_config(self, q: np.ndarray) -> Optional[float]:
        """Arm angle of a config — the dataset-label / ground-truth helper."""
        return psi_from_config(np.asarray(q, dtype=float).reshape(7), self.geom)

    def elbow_from_psi(
        self, T_ee: np.ndarray, psi: float
    ) -> Optional[np.ndarray]:
        """Elbow center implied by ``(T_ee, psi)`` (same formula as stage 1's
        differentiable kinematics layer — kept here for consistency checks)."""
        T = np.asarray(T_ee, dtype=float)
        S, W, q4_list = _compute_sw(T, self.geom)
        if not q4_list:
            return None
        return _elbow_point(float(psi), S, W, self.geom)
