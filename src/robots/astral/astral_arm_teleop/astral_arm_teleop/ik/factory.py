"""IK factory — per-arm analytic DH (arm base) + optional URDF numerical."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

_PKG_DIR = Path(__file__).resolve().parents[1]
_DEFAULT_URDF = _PKG_DIR / "robot" / "urdf" / "RobotMain_URDF.pin.urdf"


def default_urdf_path() -> str:
    return str(_DEFAULT_URDF)


def default_astral_urdf_path() -> str:
    """Prefer installed astral_robot.pin.urdf (matches MuJoCo sim)."""
    try:
        from ament_index_python.packages import get_package_share_directory

        p = (
            Path(get_package_share_directory("astral_robot_description"))
            / "urdf"
            / "astral_robot.pin.urdf"
        )
        if p.is_file():
            return str(p)
    except Exception:  # noqa: BLE001
        pass
    # source-tree fallback
    src = (
        _PKG_DIR.parents[1]
        / "astral_robot_description"
        / "urdf"
        / "astral_robot.pin.urdf"
    )
    if src.is_file():
        return str(src)
    return default_urdf_path()


class AstralIKBridge:
    """Adapter for teleop / tests.

    ``analytic_dh`` / ``geometric``: two per-arm solver instances in
    ``left_base_link`` / ``right_base_link`` (same ``IKSolver``-style API).
    ``urdf_numerical``: Pinocchio LM; dual solve via ``solve_dual`` (parallel).
    """

    def __init__(self, solver: Any, method: str):
        self._solver = solver
        self.method = method
        from astral_arm_teleop.ik.analytic import AstralParams

        self._params = {"L": AstralParams.left_arm(), "R": AstralParams.right_arm()}
        # Per-arm dict solvers (analytic_dh, geometric) share the same API.
        self._dh: Dict[str, Any] = solver if isinstance(solver, dict) else {}

    @property
    def method_name(self) -> str:
        if self._dh:
            return str(self._dh["L"].method_name)
        return getattr(self._solver, "method_name", self.method)

    @property
    def q_full(self) -> np.ndarray:
        if self._dh:
            q = np.zeros(14, dtype=float)
            q[0:7] = self._dh["L"]._state.q_prev  # noqa: SLF001
            q[7:14] = self._dh["R"]._state.q_prev  # noqa: SLF001
            return q
        return np.asarray(self._solver.q_full, dtype=float).copy()

    def sync_state(self, q14: Sequence[float]) -> None:
        q = np.asarray(q14, dtype=float).reshape(-1)
        if q.size < 14:
            q = np.pad(q, (0, 14 - q.size))
        q = q[:14]
        if self._dh:
            self._dh["L"].sync_state(q[0:7])
            self._dh["R"].sync_state(q[7:14])
            return
        self._solver._q_full = q.copy()  # noqa: SLF001
        if hasattr(self._solver, "_q_prev_L"):
            self._solver._q_prev_L = q[:7].copy()  # noqa: SLF001
            self._solver._q_prev_R = q[7:14].copy()  # noqa: SLF001

    def joint_limits(self, arm: str) -> tuple:
        key = "L" if arm.upper().startswith("L") else "R"
        if self._dh:
            s = self._dh[key]
            return s.lower_limits, s.upper_limits
        if self.method == "urdf_numerical" and hasattr(self._solver, "bounds_for"):
            return self._solver.bounds_for(key == "L")
        if self.method == "urdf_numerical" and hasattr(self._solver, "_model_L"):
            model = self._solver._model_L if key == "L" else self._solver._model_R
            return (
                np.asarray(model.lowerPositionLimit, dtype=float).copy(),
                np.asarray(model.upperPositionLimit, dtype=float).copy(),
            )
        lim = np.asarray(self._params[key].joint_limits, dtype=float)
        return lim[:, 0].copy(), lim[:, 1].copy()

    def fk(self, arm: str, q7: Sequence[float]) -> np.ndarray:
        key = "L" if arm.upper().startswith("L") else "R"
        q = np.asarray(q7, dtype=float).reshape(7)
        if self._dh:
            return self._dh[key].fk(q)
        if self.method == "urdf_numerical" and hasattr(self._solver, "fk_homogeneous"):
            return self._solver.fk_homogeneous(key, q)
        if self.method == "urdf_numerical" and hasattr(self._solver, "_model_L"):
            import pinocchio as pin

            model = self._solver._model_L if key == "L" else self._solver._model_R
            data = model.createData()
            ee = (
                getattr(self._solver, "_ee_L", "Link_la_7")
                if key == "L"
                else getattr(self._solver, "_ee_R", "Link_ra_7")
            )
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            return data.oMf[model.getFrameId(ee)].homogeneous.copy()
        raise RuntimeError("FK unavailable")

    def solve(self, arm: str, T_ee: np.ndarray) -> Optional[np.ndarray]:
        key = "L" if arm.upper().startswith("L") else "R"
        if self._dh:
            return self._dh[key].solve(T_ee)
        pos = T_ee[:3, 3].tolist()
        rpy = Rotation.from_matrix(T_ee[:3, :3]).as_euler("xyz").tolist()
        res = self._solver.solve_ik(arm=key, pos=pos, rpy=rpy)
        if res.get("q7") is not None:
            return np.asarray(res["q7"], dtype=float).reshape(7)
        return None

    def solve_dual(
        self, targets: Mapping[str, np.ndarray]
    ) -> Dict[str, Optional[np.ndarray]]:
        """Solve multiple arms; URDF / DH both fan out to 2 workers when needed."""
        items = [
            ("L" if k.upper().startswith("L") else "R", np.asarray(T, dtype=float))
            for k, T in targets.items()
        ]
        out: Dict[str, Optional[np.ndarray]] = {k: None for k, _ in items}
        if not items:
            return out

        if self._dh:
            if len(items) == 1:
                k, T = items[0]
                out[k] = self._dh[k].solve(T)
                return out

            def _one(pair):
                k, T = pair
                return k, self._dh[k].solve(T)

            with ThreadPoolExecutor(max_workers=2) as pool:
                for k, sol in pool.map(_one, items):
                    out[k] = sol
            return out

        jobs = []
        keys = []
        for k, T in items:
            keys.append(k)
            jobs.append(
                (
                    k,
                    T[:3, 3].tolist(),
                    Rotation.from_matrix(T[:3, :3]).as_euler("xyz").tolist(),
                )
            )
        if hasattr(self._solver, "solve_ik_both"):
            results = self._solver.solve_ik_both(jobs)
        else:
            results = [
                self._solver.solve_ik(arm=a, pos=p, rpy=r) for a, p, r in jobs
            ]
        for k, res in zip(keys, results):
            if res.get("q7") is not None:
                out[k] = np.asarray(res["q7"], dtype=float).reshape(7)
            else:
                out[k] = None
        return out


class SingleArmIKAdapter:
    """Nero-style single-arm API over ``AstralIKBridge`` (DH or URDF)."""

    def __init__(self, bridge: AstralIKBridge, arm_side: str):
        self.bridge = bridge
        side = arm_side.strip().lower()
        if side.startswith("l"):
            self.arm = "L"
        elif side.startswith("r"):
            self.arm = "R"
        else:
            raise ValueError("arm_side must be left|right")
        lo, hi = bridge.joint_limits(self.arm)
        self.lower_limits = np.asarray(lo, dtype=float)
        self.upper_limits = np.asarray(hi, dtype=float)

    @property
    def method_name(self) -> str:
        return self.bridge.method_name

    def sync_state(self, q7: Sequence[float], reset_branch: bool = True) -> None:
        del reset_branch  # URDF LM has no arm-angle branch state
        q = np.asarray(q7, dtype=float).reshape(7)
        q14 = self.bridge.q_full
        if self.arm == "L":
            q14[0:7] = q
        else:
            q14[7:14] = q
        self.bridge.sync_state(q14)

    def fk(self, q7: Sequence[float]) -> np.ndarray:
        return self.bridge.fk(self.arm, q7)

    def solve(self, T_target: np.ndarray) -> Optional[np.ndarray]:
        return self.bridge.solve(self.arm, T_target)

    def set_lm_params(self, **kwargs) -> None:
        solver = getattr(self.bridge, "_solver", None)
        if solver is not None and hasattr(solver, "set_lm_params"):
            solver.set_lm_params(**kwargs)
        lo, hi = self.bridge.joint_limits(self.arm)
        self.lower_limits = np.asarray(lo, dtype=float)
        self.upper_limits = np.asarray(hi, dtype=float)


def make_ik_solver(
    solver_type: str,
    *,
    urdf_path: str = "",
    ik_max_iter: int = 20,
    ik_tol: float = 1e-8,
    ik_w_pos: float = 1.0,
    ik_w_ori: float = 0.3,
    ik_w_reg: float = 1e-4,
    ik_q4_max: float = -0.45,
    ik_w_limit: float = 0.12,
    ik_dq_max: float = 0.0,
    ik_w_pref: float = 0.0,
    ik_w_fold: float = 0.015,
    ik_q4_fold: float = -1.20,
) -> AstralIKBridge:
    st = solver_type.strip().lower()

    if st in ("analytic_dh", "analytic", "dh"):
        from astral_arm_teleop.ik.analytic import AstralParams, IKSolver

        return AstralIKBridge(
            {
                "L": IKSolver(AstralParams.left_arm(), fast_mode=True),
                "R": IKSolver(AstralParams.right_arm(), fast_mode=True),
            },
            "analytic_dh",
        )

    if st in ("geometric", "swe", "poe", "geometric_arm_angle"):
        from astral_arm_teleop.ik.geometric import GeometricIKSolver

        path = urdf_path.strip() or default_astral_urdf_path()
        return AstralIKBridge(
            {
                "L": GeometricIKSolver("left", urdf_path=path, fast_mode=True),
                "R": GeometricIKSolver("right", urdf_path=path, fast_mode=True),
            },
            "geometric",
        )

    if st in ("urdf_numerical", "urdf", "numerical"):
        from astral_arm_teleop.ik.urdf_solver import URDFNumericalIKSolver

        path = urdf_path.strip() or default_urdf_path()
        if not Path(path).is_file():
            raise FileNotFoundError(f"Astral URDF not found: {path}")
        solver = URDFNumericalIKSolver(
            urdf_path=path,
            max_iter=int(ik_max_iter),
            tol=float(ik_tol),
            w_pos=float(ik_w_pos),
            w_ori=float(ik_w_ori),
            w_reg=float(ik_w_reg),
            q4_max=float(ik_q4_max),
            w_limit=float(ik_w_limit),
            dq_max=float(ik_dq_max),
            w_pref=float(ik_w_pref),
            w_fold=float(ik_w_fold),
            q4_fold=float(ik_q4_fold),
        )
        return AstralIKBridge(solver, "urdf_numerical")

    raise ValueError(
        f"Unknown solver_type={solver_type!r}; "
        "use analytic_dh, geometric or urdf_numerical"
    )


def make_single_arm_ik(
    arm_side: str,
    solver_type: str = "analytic_dh",
    *,
    urdf_path: str = "",
    ik_max_iter: int = 20,
    ik_tol: float = 1e-8,
    ik_w_pos: float = 1.0,
    ik_w_ori: float = 0.3,
    ik_w_reg: float = 1e-4,
    ik_q4_max: float = -0.45,
    ik_w_limit: float = 0.12,
    ik_dq_max: float = 0.0,
    ik_w_pref: float = 0.0,
    ik_w_fold: float = 0.015,
    ik_q4_fold: float = -1.20,
    escape_after_frames: int | None = None,
    return_after_frames: int | None = None,
):
    """One-arm solver for ``astral_arm_teleop_node`` (DH native or URDF adapter)."""
    st = solver_type.strip().lower()
    if st in ("analytic_dh", "analytic", "dh"):
        from astral_arm_teleop.ik.analytic import AstralParams, IKSolver

        side = arm_side.strip().lower()
        if side.startswith("l"):
            return IKSolver(AstralParams.left_arm(), fast_mode=True)
        if side.startswith("r"):
            return IKSolver(AstralParams.right_arm(), fast_mode=True)
        raise ValueError("arm_side must be left|right")

    if st in ("geometric", "swe", "poe", "geometric_arm_angle"):
        from astral_arm_teleop.ik.geometric import GeometricIKSolver

        path = urdf_path.strip() or default_astral_urdf_path()
        return GeometricIKSolver(
            arm_side,
            urdf_path=path,
            fast_mode=True,
            escape_after_frames=escape_after_frames,
            return_after_frames=return_after_frames,
        )

    if st in ("urdf_numerical", "urdf", "numerical"):
        path = urdf_path.strip() or default_astral_urdf_path()
        bridge = make_ik_solver(
            "urdf_numerical",
            urdf_path=path,
            ik_max_iter=ik_max_iter,
            ik_tol=ik_tol,
            ik_w_pos=ik_w_pos,
            ik_w_ori=ik_w_ori,
            ik_w_reg=ik_w_reg,
            ik_q4_max=ik_q4_max,
            ik_w_limit=ik_w_limit,
            ik_dq_max=ik_dq_max,
            ik_w_pref=ik_w_pref,
            ik_w_fold=ik_w_fold,
            ik_q4_fold=ik_q4_fold,
        )
        return SingleArmIKAdapter(bridge, arm_side)

    raise ValueError(
        f"Unknown solver_type={solver_type!r}; "
        "use analytic_dh, geometric or urdf_numerical"
    )
