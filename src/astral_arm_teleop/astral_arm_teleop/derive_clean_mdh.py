#!/usr/bin/env python3
"""Derive a clean positive-alpha Modified-DH for the Astral arm.

Fixes a=0, alpha=+90 deg, and d from the S/E/W geometry (verified), then fits
the 7 theta_offset values against the URDF flange pose (Pinocchio) over random
q. Verifies the result by re-running FK. Clean base frame (Z = joint1 axis) is
reported as an SE3 from the *old* base_link.

Run:  python3 -m astral_arm_teleop.derive_clean_mdh
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def _rotx(a): return np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
def _rotz(t): return np.array([[math.cos(t),-math.sin(t),0],[math.sin(t),math.cos(t),0],[0,0,1]])


def mdh_A(theta, d, alpha_prev):
    T = np.eye(4)
    T[:3,:3] = _rotx(alpha_prev) @ _rotz(theta)
    T[:3,3] = np.array([0.0, -math.sin(alpha_prev)*d, math.cos(alpha_prev)*d])
    return T


def mdh_fk(q, d, th_off, alpha):
    T = np.eye(4)
    for i in range(7):
        T = T @ mdh_A(float(q[i] + th_off[i]), float(d[i]), float(alpha[i]))
    return T


def _find_urdf() -> Path:
    here = Path(__file__).resolve()
    for p in [
        here.parents[2] / "astral_robot_description/urdf/astral_robot.pin.urdf",
        here.parents[3] / "src/astral_robot_description/urdf/astral_robot.pin.urdf",
        Path("/home/robot/loopkok/sdk/astral_ws/src/astral_robot_description/urdf/astral_robot.pin.urdf"),
    ]:
        if p.is_file():
            return p
    return here.parents[2] / "astral_robot_description/urdf/astral_robot.pin.urdf"


def _norm(v): return v / (np.linalg.norm(v) + 1e-15)


def _intersect(p0, z0, p1, z1):
    z0, z1 = _norm(z0), _norm(z1)
    w = p0 - p1
    a = float(z0@z0); b = float(z0@z1); c = float(z1@z1)
    d = float(z0@w); e = float(z1@w)
    den = a*c - b*b
    if abs(den) < 1e-12:
        return (p0 + p1) / 2.0
    t = (b*e - c*d) / den
    s = (a*e - b*d) / den
    return (p0 + t*z0 + p1 + s*z1) / 2.0


def main() -> int:
    import pinocchio as pin
    urdf = _find_urdf()
    print(f"URDF: {urdf}\n")
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()

    for side in ("left", "right"):
        # link frames at q=0 (old base) + joint axes
        q0 = pin.neutral(model)
        pin.forwardKinematics(model, data, q0)
        pin.updateFramePlacements(model, data)
        Tb = data.oMf[model.getFrameId(f"{side}_base_link")]
        pl, zl = [], []
        for k in range(1, 8):
            T = np.asarray((Tb.inverse()*data.oMf[model.getFrameId(f"{side}_link{k}")]).homogeneous)
            pl.append(T[:3,3].copy()); zl.append(T[:3,:3][:,2].copy())

        # Flip positive direction of joints 2,3,4 so the arm becomes all +90 deg
        # alpha (spherical "right-handed" winding for the closed-form IK).
        FLIP = [False, True, True, True, False, False, False]
        for i in range(7):
            if FLIP[i]:
                zl[i] = -zl[i]

        S = _intersect(pl[0], zl[0], pl[1], zl[1]); S = _intersect(S, zl[0], pl[2], zl[2])
        E = _intersect(pl[2], zl[2], pl[3], zl[3]); E = _intersect(E, zl[3], pl[4], zl[4])
        W = _intersect(pl[4], zl[4], pl[5], zl[5]); W = _intersect(W, zl[5], pl[6], zl[6])

        # clean base: Z = joint1 axis (sign -> d1>0), X/Y arbitrary
        zb = _norm(zl[0])
        if float((S - pl[0]) @ zb) < 0: zb = -zb
        ref = np.array([0.,0.,1.])
        if abs(float(ref @ zb)) > 0.9: ref = np.array([1.,0.,0.])
        xb = _norm(np.cross(ref, zb)); yb = np.cross(zb, xb)
        Rb = np.stack([xb, yb, zb], axis=1)   # old = Rb @ new
        O_new = pl[0].copy()
        T_base = np.eye(4); T_base[:3,:3] = Rb; T_base[:3,3] = O_new
        Tb_inv = np.linalg.inv(T_base)

        d1 = float((S - pl[0]) @ zb)
        d2 = float(np.linalg.norm(E - S))
        d4 = float(np.linalg.norm(W - E))
        # d6: distance W -> link7 along joint7 axis
        z7c = (Tb_inv[:3,:3] @ zl[6])
        Wc = Tb_inv[:3,:3] @ (W - O_new)
        p7c = Tb_inv[:3,:3] @ (pl[6] - O_new)
        d6 = float((p7c - Wc) @ z7c)
        d = np.array([d1, 0.0, d2, 0.0, d4, 0.0, d6])
        alpha = np.deg2rad([0,90,90,90,90,90,90])

        # URDF flange FK in clean base. Input q is in the FLIPPED convention
        # (joints 2,3,4 negated); map back to the original URDF convention.
        def urdf_fk(q7):
            import pinocchio as pin2
            qq = pin2.neutral(model)
            for i,k in enumerate(range(1,8)):
                jid = model.getJointId(f"{side}_joint{k}")
                qq[model.joints[jid].idx_q] = float(q7[i] if not FLIP[i] else -q7[i])
            pin2.forwardKinematics(model, data, qq)
            pin2.updateFramePlacements(model, data)
            T = Tb.inverse()*data.oMf[model.getFrameId(f"{side}_link7")]
            return np.asarray((Tb_inv @ T.homogeneous))

        rng = np.random.default_rng(0)
        q_samples = [np.zeros(7)] + [rng.uniform(-1.5, 1.5, 7) for _ in range(5)]
        T_gt = [urdf_fk(q) for q in q_samples]

        def residual(x):
            errs = []
            for q, Tg in zip(q_samples, T_gt):
                T = mdh_fk(q, d, x, alpha)
                # orientation only: lateral link offsets make position unmatchable with a=0
                errs.append(Rotation.from_matrix(T[:3,:3].T @ Tg[:3,:3]).as_rotvec())
            return np.concatenate(errs)

        res = least_squares(residual, np.zeros(7), method="lm", max_nfev=50000,
                            ftol=1e-14, xtol=1e-14, gtol=1e-14)
        th = res.x
        print(f"{'='*70}\nSIDE {side}\n{'='*70}")
        print(f"  d_i (m)           = {np.round(d,5).tolist()}")
        print(f"  theta_offset (deg)= {np.round(np.degrees(th),3).tolist()}")
        rpy = Rotation.from_matrix(Rb).as_euler("xyz")
        print(f"  clean base R_old_from_new rpy(xyz) = {np.round(np.degrees(rpy),3)} deg  O_new={np.round(O_new,4)}")

        # verify over fresh random q
        worst_p = worst_r = 0.0
        for _ in range(200):
            q = rng.uniform(-1.5, 1.5, 7)
            Tm = mdh_fk(q, d, th, alpha)
            Tg = urdf_fk(q)
            worst_p = max(worst_p, np.linalg.norm(Tm[:3,3]-Tg[:3,3])*1000)
            worst_r = max(worst_r, np.degrees(Rotation.from_matrix(Tm[:3,:3].T@Tg[:3,:3]).magnitude()))
        print(f"  VERIFY (200 q): flange pos max {worst_p:.2f} mm, rot max {worst_r:.3f} deg")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
