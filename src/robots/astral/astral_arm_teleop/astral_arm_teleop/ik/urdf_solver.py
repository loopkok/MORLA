#!/usr/bin/env python3
"""Astral / RobotMain URDF numerical IK — Pinocchio FK + scipy LM.

Astral scheme (``astral_robot.pin.urdf``): FK/IK poses are in
``left_base_link`` / ``right_base_link`` (shoulder mounts), not universe.

Supports:
  - RobotMain_URDF (Joint_la_* / Joint_ra_*) — poses in universe
  - astral_robot.pin.urdf (left_joint* / right_joint*)

Dual-arm ``solve_ik_both`` runs left/right LM in a thread pool (separate
Pinocchio models/data per arm — safe to parallelize).
"""

from __future__ import annotations

import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover
    raise ImportError("URDF IK requires pinocchio. pip install pin") from exc

from astral_arm_teleop.ik.base import IKSolverBase

_ArmJob = Tuple[str, Sequence[float], Sequence[float]]


def _package_dirs_for(urdf_path: str) -> List[str]:
    """Parents that contain ``<pkg_name>/meshes/...`` for package:// resolution."""
    urdf_dir = os.path.dirname(urdf_path)
    pkg_root = os.path.dirname(urdf_dir)  # .../<pkg>/urdf → .../<pkg>
    dirs = [
        os.path.dirname(pkg_root),  # source: .../src  (→ src/<pkg>/meshes)
        pkg_root,
        urdf_dir,
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        for pkg in ("astral_robot_description", "RobotMain_URDF"):
            try:
                share = get_package_share_directory(pkg)
                # install: .../share/<pkg> → parent .../share
                dirs.append(os.path.dirname(share))
                dirs.append(share)
            except Exception:  # noqa: BLE001
                pass
    except Exception:  # noqa: BLE001
        pass
    out: List[str] = []
    for d in dirs:
        if d and d not in out and os.path.isdir(d):
            out.append(d)
    return out


class URDFNumericalIKSolver(IKSolverBase):
    """Pinocchio FK + scipy LM — dual reduced models, parallel dual solve."""

    def __init__(
        self,
        urdf_path: str,
        max_iter: int = 20,
        tol: float = 1e-8,
        w_pos: float = 1.0,
        w_ori: float = 0.3,
        w_reg: float = 1e-4,
        q4_max: float = -0.45,
        w_limit: float = 0.12,
        limit_margin: float = 0.18,
        dq_max: float = 0.0,
        w_pref: float = 0.0,
        q_pref: Sequence[float] | None = None,
        w_fold: float = 0.015,
        q4_fold: float = -1.20,
    ):
        urdf_path = str(Path(urdf_path).resolve())
        self._urdf_path = urdf_path
        package_dirs = _package_dirs_for(urdf_path)

        robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs=package_dirs)
        names = set(robot.model.names)

        if "Joint_la_1" in names and "Joint_ra_1" in names:
            self._scheme = "robotmain"
            lock_R = [f"Joint_ra_{i}" for i in range(1, 8)]
            lock_L = [f"Joint_la_{i}" for i in range(1, 8)]
            self._ee_L = "Link_la_7"
            self._ee_R = "Link_ra_7"
            tip_L, tip_R = "Joint_la_7", "Joint_ra_7"
            self._base_L = None
            self._base_R = None
        elif "left_joint1" in names and "right_joint1" in names:
            self._scheme = "astral"
            lock_R = [f"right_joint{i}" for i in range(1, 8)]
            lock_L = [f"left_joint{i}" for i in range(1, 8)]
            self._ee_L = "left_link7"
            self._ee_R = "right_link7"
            tip_L, tip_R = "left_joint7", "right_joint7"
            self._base_L = "left_base_link"
            self._base_R = "right_base_link"
        else:
            raise ValueError(
                f"Unrecognized dual-arm URDF (need Joint_la_* or left_joint*): {urdf_path}"
            )

        self._model_L = self._build_reduced(robot, lock_R, self._ee_L, tip_L)
        self._nq_L = self._model_L.nq
        self._data_L = self._model_L.createData()

        self._model_R = self._build_reduced(robot, lock_L, self._ee_R, tip_R)
        self._nq_R = self._model_R.nq
        self._data_R = self._model_R.createData()

        for model, base, ee in (
            (self._model_L, self._base_L, self._ee_L),
            (self._model_R, self._base_R, self._ee_R),
        ):
            if base and not model.existFrame(base):
                raise ValueError(
                    f"URDF IK base frame {base!r} missing after reduce "
                    f"(ee={ee}, urdf={urdf_path})"
                )

        self._joint_names = [
            n
            for n in robot.model.names
            if n != "universe" and robot.model.getJointId(n) >= 1
        ][:14]

        self._max_nfev = max(max_iter * 15, 80)
        self._xtol = tol
        self._ftol = min(tol, 1e-12)
        self._gtol = 1e-10
        self._w_pos = w_pos
        self._w_ori = w_ori
        self._w_reg = w_reg
        # Joint4 URDF upper is 0 (fully stretched). That is an elbow
        # singularity: LM parks on the bound and cannot fold back.
        # q4_max < 0 shrinks the IK box; >= 0 keeps the URDF limit.
        self._q4_max = float(q4_max)
        self._w_limit = float(max(0.0, w_limit))
        self._limit_margin = float(max(1e-3, limit_margin))
        self._dq_max = float(max(0.0, dq_max))
        self._w_pref = float(max(0.0, w_pref))
        # One-sided: penalize an elbow straighter than q4_fold so J3 keeps
        # a lever arm and the hand can come in. Position still wins for reach.
        self._w_fold = float(max(0.0, w_fold))
        self._q4_fold = float(q4_fold)
        # Proximal joints (esp. shoulder roll) cause the "weird" teleop poses.
        self._reg_scale = np.array([1.0, 1.6, 1.4, 1.3, 0.7, 0.5, 0.5], dtype=float)
        self._pref_scale = np.array([1.0, 1.4, 1.2, 1.3, 0.4, 0.3, 0.3], dtype=float)
        self._q_pref = None
        if q_pref is not None:
            self._q_pref = np.asarray(q_pref, dtype=float).reshape(-1)

        self._q_full = np.zeros(14, dtype=float)
        self._q_prev_L = np.zeros(self._nq_L, dtype=float)
        self._q_prev_R = np.zeros(self._nq_R, dtype=float)

        self._solve_times: List[float] = []
        self._solve_stamp: List[float] = []
        self._perf_window = 100
        self._last_solve_ms = 0.0
        self._stats_lock = threading.Lock()
        self._q_lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="urdf_ik")

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)

    def get_perf_stats(self) -> Dict[str, Any]:
        with self._stats_lock:
            times = list(self._solve_times[-self._perf_window :])
            stamps = list(self._solve_stamp[-self._perf_window :])
            last_ms = float(self._last_solve_ms)
            total_count = len(self._solve_times)
        if not times:
            return {
                "count": 0,
                "last_ms": 0.0,
                "avg_ms": 0.0,
                "p95_ms": 0.0,
                "max_ms": 0.0,
                "fps": 0.0,
            }
        arr = np.asarray(times, dtype=float)
        fps = 0.0
        if len(stamps) >= 2:
            span = stamps[-1] - stamps[0]
            if span > 1e-6:
                fps = (len(stamps) - 1) / span
        return {
            "count": total_count,
            "last_ms": round(last_ms, 2),
            "avg_ms": round(float(np.mean(arr)), 2),
            "p95_ms": round(float(np.percentile(arr, 95)), 2),
            "max_ms": round(float(np.max(arr)), 2),
            "fps": round(fps, 1),
        }

    def _record_solve_ms(self, dt_ms: float) -> None:
        with self._stats_lock:
            self._last_solve_ms = dt_ms
            self._solve_times.append(dt_ms)
            self._solve_stamp.append(time.perf_counter())
            if len(self._solve_times) > self._perf_window * 2:
                self._solve_times = self._solve_times[-self._perf_window :]
                self._solve_stamp = self._solve_stamp[-self._perf_window :]

    @staticmethod
    def _build_reduced(
        robot,
        locked_names: List[str],
        ee_name: str,
        tip_joint: str,
    ) -> pin.Model:
        reduced = robot.buildReducedRobot(
            list_of_joints_to_lock=locked_names,
            reference_configuration=np.zeros(robot.model.nq),
        )
        model = reduced.model
        if model.existFrame(ee_name):
            return model
        if model.existJointName(tip_joint):
            ee_parent = model.getJointId(tip_joint)
        else:
            ee_parent = model.njoints - 1
        model.addFrame(
            pin.Frame(ee_name, ee_parent, pin.SE3.Identity(), pin.FrameType.OP_FRAME)
        )
        return model

    @staticmethod
    def _ee_in_base(model, data, ee_id: int, base_name: str | None):
        """EE pose in ``*_base_link`` when set, otherwise universe."""
        Tee = data.oMf[ee_id]
        if not base_name:
            return Tee
        return data.oMf[model.getFrameId(base_name)].inverse() * Tee

    def fk_homogeneous(self, arm: str, q7: Sequence[float]) -> np.ndarray:
        is_left = str(arm).upper().startswith("L")
        model = self._model_L if is_left else self._model_R
        data = model.createData()
        q = np.asarray(q7, dtype=float).reshape(-1)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        ee_name = self._ee_L if is_left else self._ee_R
        base_name = self._base_L if is_left else self._base_R
        T = self._ee_in_base(model, data, model.getFrameId(ee_name), base_name)
        return T.homogeneous.copy()

    @property
    def method_name(self) -> str:
        return f"urdf_numerical_lm/{self._scheme}"

    def set_lm_params(
        self,
        max_iter: int | None = None,
        tol: float | None = None,
        w_pos: float | None = None,
        w_ori: float | None = None,
        w_reg: float | None = None,
        q4_max: float | None = None,
        w_limit: float | None = None,
        dq_max: float | None = None,
        w_pref: float | None = None,
        q_pref: Sequence[float] | None = None,
        w_fold: float | None = None,
        q4_fold: float | None = None,
    ) -> None:
        if max_iter is not None:
            self._max_nfev = max(int(max_iter) * 15, 80)
        if tol is not None:
            t = float(tol)
            self._xtol = t
            self._ftol = min(t, 1e-12)
        if w_pos is not None:
            self._w_pos = float(max(0.0, w_pos))
        if w_ori is not None:
            self._w_ori = float(max(0.0, w_ori))
        if w_reg is not None:
            self._w_reg = float(max(0.0, w_reg))
        if q4_max is not None:
            self._q4_max = float(q4_max)
        if w_limit is not None:
            self._w_limit = float(max(0.0, w_limit))
        if dq_max is not None:
            self._dq_max = float(max(0.0, dq_max))
        if w_pref is not None:
            self._w_pref = float(max(0.0, w_pref))
        if q_pref is not None:
            self._q_pref = np.asarray(q_pref, dtype=float).reshape(-1)
        if w_fold is not None:
            self._w_fold = float(max(0.0, w_fold))
        if q4_fold is not None:
            self._q4_fold = float(q4_fold)

    def bounds_for(self, is_left: bool) -> Tuple[np.ndarray, np.ndarray]:
        model = self._model_L if is_left else self._model_R
        return self._ik_bounds(model)

    def _ik_bounds(self, model) -> Tuple[np.ndarray, np.ndarray]:
        lower = np.asarray(model.lowerPositionLimit, dtype=float).copy()
        upper = np.asarray(model.upperPositionLimit, dtype=float).copy()
        if lower.size > 3 and self._q4_max < upper[3]:
            upper[3] = float(self._q4_max)
        upper = np.maximum(upper, lower + 1e-3)
        return lower, upper

    @property
    def q_full(self) -> List[float]:
        with self._q_lock:
            return self._q_full.tolist()

    def reset(self) -> None:
        with self._q_lock:
            self._q_full[:] = 0.0
            self._q_prev_L[:] = 0.0
            self._q_prev_R[:] = 0.0

    def get_state(self) -> Dict[str, Any]:
        return {
            "type": "state",
            "q": self.q_full,
            "joint_names": list(self._joint_names),
            "nq": 14,
            "ik_method": self.method_name,
            "scheme": self._scheme,
        }

    def solve_ik(
        self,
        arm: str,
        pos: List[float],
        rpy: List[float],
        lock_joint7: bool = False,
    ) -> Dict[str, Any]:
        del lock_joint7  # reserved
        t0 = time.perf_counter()

        is_left = arm.upper().startswith("L")
        model = self._model_L if is_left else self._model_R
        # Per-call Data so concurrent L/R (and future same-arm) stay safe.
        data = model.createData()
        nq = self._nq_L if is_left else self._nq_R
        ee_name = self._ee_L if is_left else self._ee_R
        ee_id = model.getFrameId(ee_name)
        base_name = self._base_L if is_left else self._base_R
        arm_base = 0 if is_left else 7

        with self._q_lock:
            q_prev_arm = (
                self._q_prev_L.copy() if is_left else self._q_prev_R.copy()
            )

        lower_j, upper_j = self._ik_bounds(model)
        q0 = np.clip(q_prev_arm.copy(), lower_j, upper_j)
        lower, upper = lower_j, upper_j
        if self._dq_max > 0.0:
            lower = np.maximum(lower_j, q0 - self._dq_max)
            upper = np.minimum(upper_j, q0 + self._dq_max)
            upper = np.maximum(upper, lower + 1e-4)

        T_tgt_mat = np.eye(4)
        T_tgt_mat[:3, 3] = np.asarray(pos, dtype=float)
        T_tgt_mat[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        T_tgt = pin.SE3(T_tgt_mat)

        w_pos, w_ori, w_reg = self._w_pos, self._w_ori, self._w_reg
        w_limit, margin = self._w_limit, self._limit_margin
        n_lim = nq if w_limit > 0.0 else 0
        w_pref = self._w_pref
        q_pref = self._q_pref
        n_pref = nq if (w_pref > 0.0 and q_pref is not None and q_pref.size >= nq) else 0
        w_fold, q4_fold = self._w_fold, self._q4_fold
        n_fold = 1 if (w_fold > 0.0 and nq > 3) else 0
        rs = self._reg_scale[:nq]
        ps = self._pref_scale[:nq]
        s_reg = np.sqrt(w_reg) * rs
        s_pref = np.sqrt(w_pref) * ps if n_pref else None
        q_pref_n = q_pref[:nq] if n_pref else None
        # Don't spend most of a short-side travel (left J2 inward is 0.3 rad)
        # inside the hinge. Scale margin by room from q=0 (clipped to bounds).
        ref0 = np.clip(np.zeros(nq), lower_j, upper_j)
        room_lo = np.maximum(ref0 - lower_j, 1e-3)
        room_hi = np.maximum(upper_j - ref0, 1e-3)
        m_lo = np.minimum(margin, 0.30 * room_lo)
        m_hi = np.minimum(margin, 0.30 * room_hi)

        def _cost(q):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            Tc = self._ee_in_base(model, data, ee_id, base_name)
            err = pin.log(Tc.inverse() * T_tgt).vector
            out = np.empty(6 + nq + n_lim + n_pref + n_fold)
            out[:3] = w_pos * err[:3]
            out[3:6] = w_ori * err[3:]
            out[6 : 6 + nq] = s_reg * (q - q0)
            off = 6 + nq
            if n_lim:
                lo_pen = np.maximum(0.0, m_lo - (q - lower_j))
                hi_pen = np.maximum(0.0, m_hi - (upper_j - q))
                out[off : off + nq] = w_limit * (lo_pen + hi_pen)
                off += nq
            if n_pref:
                out[off : off + nq] = s_pref * (q - q_pref_n)
                off += nq
            if n_fold:
                out[off] = w_fold * max(0.0, float(q[3]) - q4_fold)
            # At the elbow cap, drop orientation so the other 6 joints
            # can still slide the EE along the reach surface.
            if nq > 3 and (upper_j[3] - q[3]) < 0.04:
                out[3:6] *= 0.2
            return out

        try:
            res = least_squares(
                _cost,
                q0,
                bounds=(lower, upper),
                method="trf",
                xtol=self._xtol,
                ftol=self._ftol,
                gtol=self._gtol,
                max_nfev=self._max_nfev,
                verbose=0,
            )
        except Exception:
            dt = (time.perf_counter() - t0) * 1000
            self._record_solve_ms(dt)
            return {
                "ik_ok": False,
                "ik_err": float("inf"),
                "arm": "L" if is_left else "R",
                "ik_method": self.method_name,
                "detail": {"exception": True, "solve_ms": round(dt, 2)},
            }

        dt = (time.perf_counter() - t0) * 1000
        self._record_solve_ms(dt)

        q_out = np.asarray(res.x, dtype=float).reshape(-1)
        if not np.isfinite(q_out).all():
            dt = (time.perf_counter() - t0) * 1000
            self._record_solve_ms(dt)
            return {
                "ik_ok": False,
                "ik_err": float("inf"),
                "arm": "L" if is_left else "R",
                "ik_method": self.method_name,
                "q7": None,
                "detail": {"nonfinite": True, "solve_ms": round(dt, 2)},
            }
        q_out = np.clip(q_out, lower, upper)
        pin.forwardKinematics(model, data, q_out)
        pin.updateFramePlacements(model, data)
        Tc = self._ee_in_base(model, data, ee_id, base_name)
        pos_err = float(np.linalg.norm(Tc.translation - T_tgt.translation))
        rot_err = float(np.linalg.norm(pin.log(Tc.inverse() * T_tgt).vector[3:]))
        err_norm = float(np.sqrt(pos_err * pos_err + rot_err * rot_err))
        ik_ok = pos_err < 8e-3 and rot_err < 0.25

        # Always keep the bounded LM pose as warm-start / teleop command.
        # Unreachable targets (elbow at ik_q4_max) must still track along
        # the reach surface instead of returning None and freezing q_cmd.
        with self._q_lock:
            self._q_full[arm_base : arm_base + nq] = q_out
            if is_left:
                self._q_prev_L = q_out.copy()
            else:
                self._q_prev_R = q_out.copy()

        return {
            "ik_ok": ik_ok,
            "ik_err": err_norm,
            "arm": "L" if is_left else "R",
            "ik_method": self.method_name,
            "q7": q_out.copy(),
            "detail": {
                "pos_err_mm": round(pos_err * 1000, 3),
                "rot_err_rad": round(rot_err, 4),
                "solve_ms": round(dt, 2),
                "nfev": int(getattr(res, "nfev", 0)),
                "saturated": (not ik_ok),
            },
        }

    def solve_ik_both(
        self, jobs: Sequence[_ArmJob]
    ) -> List[Dict[str, Any]]:
        """Solve several arms; 2+ jobs run left/right in parallel."""
        jobs = list(jobs)
        if len(jobs) <= 1:
            return [
                self.solve_ik(arm, list(pos), list(rpy)) for arm, pos, rpy in jobs
            ]

        futs = [
            self._pool.submit(self.solve_ik, arm, list(pos), list(rpy))
            for arm, pos, rpy in jobs
        ]
        return [f.result() for f in futs]

    @property
    def avg_solve_time_ms(self) -> float:
        return float(self.get_perf_stats()["avg_ms"])
