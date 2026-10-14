#!/usr/bin/env python3
"""Generate a clean MJCF (astral_dual_clean.xml) with positive-MDH joints.

Rewrites the body pos/quat (joint origins), geom pos/quat (visual offsets) and
joint range (flipped limits) to match the clean URDF / DH. The mesh files and
inertial blocks are reused unchanged from astral_mujoco_sim/assets/mjcf/astral_dual.xml.

Run: python3 -m astral_arm_teleop.generate_clean_mjcf
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation


def _rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def _rotz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def mdh_A(theta, d, alpha_prev):
    T = np.eye(4)
    T[:3, :3] = _rotx(alpha_prev) @ _rotz(theta)
    T[:3, 3] = [0.0, -math.sin(alpha_prev) * d, math.cos(alpha_prev) * d]
    return T


PARAMS = {
    "left": {"d": [-0.11655, 0, 0.246, 0, 0.2265, 0, 0.0036],
             "th": np.deg2rad([180, 90, 90, 180, 180, -90, -90])},
    "right": {"d": [0.11655, 0, -0.246, 0, 0.2265, 0, 0.0036],
              "th": np.deg2rad([180, -90, -90, 0, 180, -90, -90])},
}
ALPHA = np.deg2rad([0, 90, 90, 90, 90, 90, 90])

# flipped joint limits (negate on flipped joints). Flipped joints: left 2,3,4; right 2,4.
_LIM = {
    "left": [(-2.0, 2.0), (-2.0, 0.3), (-1.57, 1.57), (0.0, 2.26),
             (-1.57, 1.57), (-0.8, 0.85), (-1.57, 1.57)],
    "right": [(-2.0, 2.0), (-0.3, 2.0), (-1.57, 1.57), (0.0, 2.26),
              (-1.57, 1.57), (-0.8, 0.85), (-1.57, 1.57)],
}

_SRC_ROOT = Path(__file__).resolve().parents[2]
_SRC_URDF = _SRC_ROOT / "astral_robot_description/urdf/astral_robot.pin.urdf"
_SRC_MJCF = _SRC_ROOT / "astral_mujoco_sim/assets/mjcf/astral_dual.xml"
_DST_MJCF = _SRC_ROOT / "astral_mujoco_sim/assets/mjcf/astral_dual_clean.xml"


def _q_str(q):
    # scipy quat = [x, y, z, w]; MJCF wants "w x y z"
    return " ".join(f"{v:.6g}" for v in [q[3], q[0], q[1], q[2]])


def _p_str(p):
    return " ".join(f"{v:.6g}" for v in p)


def main() -> int:
    model = pin.buildModelFromUrdf(str(_SRC_URDF))
    data = model.createData()

    # compute per-arm: base quat, body pos/quat (joint origin), geom pos/quat (offset)
    arm = {}
    for side in ("left", "right"):
        prm = PARAMS[side]
        d = np.asarray(prm["d"], float)
        th = prm["th"]
        q0 = pin.neutral(model)
        pin.forwardKinematics(model, data, q0)
        pin.updateFramePlacements(model, data)
        Tb = data.oMf[model.getFrameId(f"{side}_base_link")]
        z1 = np.asarray((Tb.inverse() * data.oMi[model.getJointId(f"{side}_joint1")]).rotation[:, 2])
        zb = z1
        xb = np.array([0.0, 0.0, 1.0]); xb = xb - float(xb @ zb) * zb
        if np.linalg.norm(xb) < 1e-9:
            xb = np.array([1.0, 0.0, 0.0]); xb = xb - float(xb @ zb) * zb
        xb /= np.linalg.norm(xb); yb = np.cross(zb, xb)
        R = np.stack([xb, yb, zb], axis=1)
        T_base = np.eye(4); T_base[:3, :3] = R
        base_q = Rotation.from_matrix(R).as_quat()

        Tc = np.eye(4)
        frames_old_base = []
        body = []  # (pos, quat) per joint
        geom = []  # (pos, quat) per link
        for i in range(7):
            Ai = mdh_A(float(th[i]), float(d[i]), float(ALPHA[i]))
            Tc = Tc @ Ai
            frames_old_base.append((T_base @ Tc).copy())
            body.append((Ai[:3, 3].copy(), Rotation.from_matrix(Ai[:3, :3]).as_quat()))
        for i in range(7):
            T_old = np.asarray((Tb.inverse() * data.oMf[model.getFrameId(f"{side}_link{i+1}")]).homogeneous)
            Tv = np.linalg.inv(frames_old_base[i]) @ T_old
            geom.append((Tv[:3, 3].copy(), Rotation.from_matrix(Tv[:3, :3]).as_quat()))
        arm[side] = {"base_q": base_q, "body": body, "geom": geom}

    # rewrite MJCF
    tree = ET.parse(_SRC_MJCF)
    root = tree.getroot()
    for side in ("left", "right"):
        a = arm[side]
        # base_link body quat
        base = root.find(f"./worldbody/body/body[@name='{side}_base_link']")
        if base is not None:
            base.set("quat", _q_str(a["base_q"]))
        for i in range(1, 8):
            ln = f"{side}_link{i}"
            body_el = root.find(f"./body[@name='{ln}']")
            if body_el is None:
                body_el = root.find(f".//body[@name='{ln}']")
            if body_el is not None:
                body_el.set("pos", _p_str(a["body"][i - 1][0]))
                body_el.set("quat", _q_str(a["body"][i - 1][1]))
            # geom (first mesh geom inside this body)
            if body_el is not None:
                geom_el = body_el.find("./geom[@type='mesh']")
                if geom_el is not None:
                    geom_el.set("pos", _p_str(a["geom"][i - 1][0]))
                    geom_el.set("quat", _q_str(a["geom"][i - 1][1]))
            # joint range
            jn = f"{side}_joint{i}"
            joint_el = root.find(f".//joint[@name='{jn}']")
            if joint_el is not None:
                lo, hi = _LIM[side][i - 1]
                joint_el.set("range", f"{lo} {hi}")
            # actuator ctrlrange
            act = root.find(f".//position[@joint='{jn}']")
            if act is not None:
                lo, hi = _LIM[side][i - 1]
                act.set("ctrlrange", f"{lo} {hi}")

    tree.write(_DST_MJCF, xml_declaration=True, encoding="utf-8")
    print(f"wrote {_DST_MJCF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
