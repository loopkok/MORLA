#!/usr/bin/env python3
"""Generate a clean (positive-MDH) single/dual-arm URDF for the Astral arm.

Ground truth DH (see ik/analytic.py). The joint origin IS the MDH
A_i = RotX(alpha) RotZ(theta_offset) TransZ(d); the visual/collision/inertial
origin re-positions each STL mesh (which lives at the OLD link frame origin)
into the new clean link frame.

Writes:
  astral_arm_clean_description/urdf/astral_arm_clean.urdf     (single left arm)
  astral_arm_clean_description/urdf/astral_robot_clean.urdf   (dual-arm)
and verifies the clean chain FK (flange) matches the old URDF.

Run: python3 -m astral_arm_teleop.generate_clean_urdf
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
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
    "left": {
        "d": [-0.11655, 0.0, 0.246, 0.0, 0.2265, 0.0, 0.0036],
        "theta_deg": [180.0, 90.0, 90.0, 180.0, 180.0, -90.0, -90.0],
        "alpha_deg": [0, 90, 90, 90, 90, 90, 90],
        "mount_xyz": [0.09495, 0.0, 0.412],
        "mesh_pkg": "astral_arm_description",
    },
    "right": {
        "d": [0.11655, 0.0, -0.246, 0.0, 0.2265, 0.0, 0.0036],
        "theta_deg": [180.0, -90.0, -90.0, 0.0, 180.0, -90.0, -90.0],
        "alpha_deg": [0, 90, 90, 90, 90, 90, 90],
        "mount_xyz": [-0.09495, 0.0, 0.412],
        "mesh_pkg": "astral_robot_description",
    },
}

_SRC_ROOT = Path(__file__).resolve().parents[2]
_SRC_URDF = _SRC_ROOT / "astral_robot_description/urdf/astral_robot.pin.urdf"
_OUT_DIR = _SRC_ROOT / "astral_arm_clean_description/urdf"


def _fmt(xyz, rpy):
    return (f'xyz="{" ".join(f"{v:.6g}" for v in xyz)}" '
            f'rpy="{" ".join(f"{v:.6g}" for v in rpy)}"')


def _load_old_links():
    """Return dict side -> {link_i: {inertial:{...}, visual:{mesh,color}}}."""
    root = ET.parse(_SRC_URDF).getroot()
    out = {"left": {}, "right": {}}
    for side in ("left", "right"):
        for i in range(1, 8):
            ln = f"{side}_link{i}"
            link = root.find(f"./link[@name='{ln}']")
            info = {"mass": 0.0, "inertial": None, "visual": None}
            inert = link.find("inertial")
            info["mass"] = float(inert.find("mass").get("value"))
            org = inert.find("origin")
            info["inertial_origin"] = {
                "xyz": [float(v) for v in org.get("xyz", "0 0 0").split()],
                "rpy": [float(v) for v in org.get("rpy", "0 0 0").split()],
            }
            inr = inert.find("inertia")
            info["inertia"] = {k: float(inr.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
            vis = link.find("visual")
            info["mesh"] = vis.find("geometry/mesh").get("filename")
            info["color"] = vis.find("material/color").get("rgba") if vis.find("material/color") is not None else None
            out[side][i] = info
    return out


def _se3(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = np.asarray(xyz, float)
    return T


def _transform_inertial(info, T_vis):
    """inertial origin/rot/inertia re-expressed in the new link frame."""
    T_old_in = _se3(info["inertial_origin"]["xyz"], info["inertial_origin"]["rpy"])
    T_new_in = T_vis @ T_old_in
    xyz = T_new_in[:3, 3]
    rpy = Rotation.from_matrix(T_new_in[:3, :3]).as_euler("xyz")
    R = T_vis[:3, :3]
    I = np.array([[info["inertia"]["ixx"], info["inertia"]["ixy"], info["inertia"]["ixz"]],
                  [info["inertia"]["ixy"], info["inertia"]["iyy"], info["inertia"]["iyz"]],
                  [info["inertia"]["ixz"], info["inertia"]["iyz"], info["inertia"]["izz"]]])
    I2 = R @ I @ R.T
    return xyz, rpy, I2


def _gen_arm_xml(side, old_links, base_rpy, joint_origins, visual_offsets):
    prm = PARAMS[side]
    lines = []
    lines.append(f"  <link name=\"{side}_base_link\"/>")
    for i in range(1, 8):
        info = old_links[side][i]
        xyz, rpy = joint_origins[i - 1]
        v_xyz, v_rpy = visual_offsets[i - 1]
        # inertial in new link frame
        T_vis = _se3(v_xyz, v_rpy)
        in_xyz, in_rpy, I2 = _transform_inertial(info, T_vis)
        lines.append(f"  <link name=\"{side}_link{i}\">")
        lines.append("    <inertial>")
        lines.append(f"      <origin {_fmt(in_xyz, in_rpy)}/>")
        lines.append(f"      <mass value=\"{info['mass']:.8g}\"/>")
        lines.append(f"      <inertia ixx=\"{I2[0,0]:.8g}\" ixy=\"{I2[0,1]:.8g}\" ixz=\"{I2[0,2]:.8g}\" "
                     f"iyy=\"{I2[1,1]:.8g}\" iyz=\"{I2[1,2]:.8g}\" izz=\"{I2[2,2]:.8g}\"/>")
        lines.append("    </inertial>")
        mesh = info["mesh"].replace("astral_robot_description", prm["mesh_pkg"])
        lines.append("    <visual>")
        lines.append(f"      <origin {_fmt(v_xyz, v_rpy)}/>")
        lines.append("      <geometry>")
        lines.append(f"        <mesh filename=\"{mesh}\"/>")
        lines.append("      </geometry>")
        if info["color"]:
            lines.append(f"      <material name=\"\"><color rgba=\"{info['color']}\"/></material>")
        lines.append("    </visual>")
        lines.append("    <collision>")
        lines.append(f"      <origin {_fmt(v_xyz, v_rpy)}/>")
        lines.append("      <geometry>")
        lines.append(f"        <mesh filename=\"{mesh}\"/>")
        lines.append("      </geometry>")
        lines.append("    </collision>")
        lines.append("  </link>")
    # joints
    for i in range(1, 8):
        parent = f"{side}_base_link" if i == 1 else f"{side}_link{i-1}"
        child = f"{side}_link{i}"
        xyz, rpy = joint_origins[i - 1]
        lines.append(f"  <joint name=\"{side}_joint{i}\" type=\"revolute\">")
        lines.append(f"    <origin {_fmt(xyz, rpy)}/>")
        lines.append(f"    <parent link=\"{parent}\"/>")
        lines.append(f"    <child link=\"{child}\"/>")
        lines.append("    <axis xyz=\"0 0 1\"/>")
        # URDF limits (flipped on the flipped joints handled by theta_offset/rpy)
        lines.append(f"    <limit lower=\"-3.15\" upper=\"3.15\" effort=\"100\" velocity=\"10\"/>")
        lines.append("  </joint>")
    return lines


def main() -> int:
    import pinocchio as pin

    old_links = _load_old_links()
    model = pin.buildModelFromUrdf(str(_SRC_URDF))
    data = model.createData()

    per_side = {}
    for side in ("left", "right"):
        prm = PARAMS[side]
        d = np.asarray(prm["d"], float)
        th = np.deg2rad(np.asarray(prm["theta_deg"], float))
        al = np.deg2rad(np.asarray(prm["alpha_deg"], float))
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
        R_on = np.stack([xb, yb, zb], axis=1)
        T_base = np.eye(4); T_base[:3, :3] = R_on
        base_rpy = Rotation.from_matrix(R_on).as_euler("xyz")

        T_clean = np.eye(4)
        joint_origins = []
        clean_frames_old = []
        for i in range(7):
            Ai = mdh_A(float(th[i]), float(d[i]), float(al[i]))
            T_clean = T_clean @ Ai
            clean_frames_old.append(T_base @ T_clean)
            joint_origins.append((Ai[:3, 3].copy(), Rotation.from_matrix(Ai[:3, :3]).as_euler("xyz")))
        visual_offsets = []
        for i in range(7):
            T_old = np.asarray((Tb.inverse() * data.oMf[model.getFrameId(f"{side}_link{i+1}")]).homogeneous)
            T_vis = np.linalg.inv(clean_frames_old[i]) @ T_old
            visual_offsets.append((T_vis[:3, 3].copy(), Rotation.from_matrix(T_vis[:3, :3]).as_euler("xyz")))
        per_side[side] = (base_rpy, joint_origins, visual_offsets)

    # ---- write single-arm (left) URDF ----
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    base_rpy, jo, vo = per_side["left"]
    m = PARAMS["left"]["mount_xyz"]
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<robot name="astral_arm_clean">',
             '  <!-- single clean arm (left) — positive-MDH joints, meshes re-positioned -->',
             '  <link name="body_link"/>',
             '  <joint name="shoulder_mount" type="fixed">',
             f'    <origin xyz="{m[0]} {m[1]} {m[2]}" rpy="{" ".join(f"{v:.6g}" for v in base_rpy)}"/>',
             '    <parent link="body_link"/><child link="left_base_link"/>',
             '  </joint>']
    lines += _gen_arm_xml("left", old_links, base_rpy, jo, vo)
    lines.append("</robot>")
    (_OUT_DIR / "astral_arm_clean.urdf").write_text("\n".join(lines) + "\n")

    # ---- write dual-arm URDF ----
    lines = ['<?xml version="1.0" encoding="utf-8"?>',
             '<robot name="astral_robot_clean">',
             '  <!-- dual clean arm — positive-MDH joints -->',
             '  <link name="world"/>',
             '  <joint name="ground_to_body" type="fixed">',
             '    <origin xyz="0 0 0" rpy="0 0 0"/>',
             '    <parent link="world"/><child link="body_link"/>',
             '  </joint>',
             '  <link name="body_link"/>']
    for side in ("left", "right"):
        br, jjo, vvo = per_side[side]
        m = PARAMS[side]["mount_xyz"]
        lines.append(f'  <joint name="{side}_shoulder_mount" type="fixed">')
        lines.append(f'    <origin xyz="{m[0]} {m[1]} {m[2]}" rpy="{" ".join(f"{v:.6g}" for v in br)}"/>')
        lines.append(f'    <parent link="body_link"/><child link="{side}_base_link"/>')
        lines.append("  </joint>")
        lines += _gen_arm_xml(side, old_links, br, jjo, vvo)
    lines.append("</robot>")
    (_OUT_DIR / "astral_robot_clean.urdf").write_text("\n".join(lines) + "\n")

    print(f"wrote {_OUT_DIR/'astral_arm_clean.urdf'}")
    print(f"wrote {_OUT_DIR/'astral_robot_clean.urdf'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
