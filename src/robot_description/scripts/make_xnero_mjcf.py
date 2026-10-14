#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert the centralized Nero/XHand URDF into its tactile MuJoCo MJCF.

为什么要"转"而不是直接喂 URDF：
  - MuJoCo 不认 `package://` 路径；
  - 厂商 URDF 有 34 个 link 的惯量是退化张量，MuJoCo 直接拒收；
  - 触觉点与指尖 mesh 共面，要让逐点力有意义必须关掉那 5 个掌侧 link2 的 mesh 碰撞；
  - 仿真还需要 URDF 表达不了的东西：地面、执行器、`site`+`touch` 传感器、接触参数。
前三条由 `prep_xnero_sim_urdf.py` 负责，本脚本负责转换与后处理。

产物：`robot_description/mjcf/xnero/xnero_dual.xml`
  - 手部 24 个位置执行器 `act_{side}_hand_{joint}`（ctrlrange 取自 URDF 限位）
  - 10 个触觉 touch 传感器 `ft_{side}_{finger}`（site 位置/半径来自实测标定）
  - 1200 个 `tac_*` 碰撞球，带接触参数
  - 地面 + 光照 + option/visual
  - meshdir 写成**相对路径**（源码树可直接加载）；ROS 节点加载时会覆盖成
    `get_package_share_directory('robot_description')/meshes` 的绝对路径。

用法：
  python3 src/robot_description/scripts/make_xnero_mjcf.py  # 更新集中模型
  python3 scripts/make_mjcf.py --timestep 0.001 --kp 8 --force 0.8
  python3 scripts/make_mjcf.py --out /tmp/x.xml --no-floor
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]        # robot_description
DEFAULT_OUT = PKG_ROOT / "mjcf" / "xnero" / "xnero_dual.xml"
PREP = PKG_ROOT / "scripts" / "prep_xnero_sim_urdf.py"

# 装了触觉阵列的 5 个掌侧指尖 link2（精确匹配，别用 .*link2$ —— 会误伤臂与手背）
PAD_LINK2_RE = r"^(left|right)_hand_(thumb_rota|index_rota|mid|ring|pinky)_link2$"

FINGERS = ("thumb", "index", "mid", "ring", "pinky")
HAND_JOINT_NAMES = [
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2",
    "mid_joint1", "mid_joint2", "ring_joint1", "ring_joint2",
    "pinky_joint1", "pinky_joint2",
]

# 指尖 link2 body 名（tactile 点所在的坐标系）
PAD_BODY = {
    "thumb": "{side}_hand_thumb_rota_link2",
    "index": "{side}_hand_index_rota_link2",
    "mid": "{side}_hand_mid_link2",
    "ring": "{side}_hand_ring_link2",
    "pinky": "{side}_hand_pinky_link2",
}

# touch sensor 的 site：= 120 点的质心，半径 = 覆盖半径 + 2mm 余量
# （实测验证：与逐点求和逐档一致，多指同压不串味；见 docs/xhand1-tactile-sim-params.md §5.2b）
FOUR_FINGERS_SITE = ("0.00487 0 0.03077", "0.01782")
PAD_SITE = {
    ("left", "thumb"): ("0 -0.03508 0.00269", "0.02384"),
    ("right", "thumb"): ("0 0.03508 0.00347", "0.02384"),
}
for _side in ("left", "right"):
    for _f in ("index", "mid", "ring", "pinky"):
        PAD_SITE[(_side, _f)] = FOUR_FINGERS_SITE

# 触觉探针的接触参数（见 docs/xhand1-tactile-sim-params.md §5.4）
TAC_SOLREF = "0.004 1"
TAC_SOLIMP = "0.9 0.95 0.001"
TAC_FRICTION = "1 0.05 0.001"


def _run_prep(workdir: Path, urdf: Path) -> Path:
    cmd = [sys.executable, str(PREP), "--urdf", str(urdf), "--out", str(workdir), "--no-collision", PAD_LINK2_RE]
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout.rstrip())
    if r.returncode:
        print(r.stderr, file=sys.stderr)
        raise SystemExit("prep_sim_urdf.py 失败")
    return workdir / "robot_sim.urdf"


def _urdf_joint_limits(urdf: Path) -> dict[str, tuple[float, float]]:
    """从 URDF 读非 fixed 关节的 (lower, upper)。"""
    root = ET.parse(urdf).getroot()
    out = {}
    for j in root.findall("joint"):
        if j.get("type") == "fixed":
            continue
        lim = j.find("limit")
        if lim is None:
            continue
        out[j.get("name")] = (float(lim.get("lower", 0.0)), float(lim.get("upper", 0.0)))
    return out


def _fmt(v: float) -> str:
    s = f"{v:g}"
    return "0" if s == "-0" else s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="URDF → MuJoCo MJCF")
    ap.add_argument("--urdf", type=Path, default=PKG_ROOT / "urdf" / "nero" / "xhand_nero_description.urdf")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--timestep", type=float, default=0.002)
    ap.add_argument("--kp", type=float, default=5.0, help="手部位置执行器 kp")
    ap.add_argument("--kv", type=float, default=0.1, help="手部位置执行器 kv")
    ap.add_argument("--force", type=float, default=0.5, help="手部执行器力矩上限 N·m")
    ap.add_argument("--no-floor", action="store_true")
    args = ap.parse_args(argv)

    try:
        import mujoco
    except ImportError:
        raise SystemExit("需要 mujoco：/usr/bin/python3 -m pip install mujoco")

    urdf_md5 = hashlib.md5(args.urdf.read_bytes()).hexdigest()[:12]
    limits = _urdf_joint_limits(args.urdf)

    with tempfile.TemporaryDirectory(prefix="make_mjcf_") as td:
        sim_urdf = _run_prep(Path(td), args.urdf)
        model = mujoco.MjModel.from_xml_path(str(sim_urdf))
        raw = Path(td) / "raw.xml"
        mujoco.mj_saveLastXML(str(raw), model)
        print(f"MuJoCo 编译：nq={model.nq} nu={model.nu} ngeom={model.ngeom}")
        text = raw.read_text(encoding="utf-8")

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(text, parser=parser)

    # ---- compiler：加 meshdir，并把 mesh 的绝对路径改成相对 meshes 根 ----
    # meshdir 相对「本 XML 所在目录」(robot_description/mjcf/xnero/)：
    #   ../ = assets/ , ../../ = xnero_mujoco_sim/ , ../../../ = src/
    # 源码树可直接加载；ROS 节点加载时会覆盖成绝对路径（安装后层级不同）。
    abs_mesh_prefix = str((PKG_ROOT / "meshes").resolve()) + "/"
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", os.path.relpath((PKG_ROOT / "meshes").resolve(), args.out.parent.resolve()))
    n_mesh = 0
    for mesh in root.iter("mesh"):
        f = mesh.get("file") or ""
        if f.startswith(abs_mesh_prefix):
            mesh.set("file", f[len(abs_mesh_prefix):])
            n_mesh += 1
        elif f.startswith("/"):
            raise SystemExit(f"!! mesh 路径不在 meshes 根下，无法相对化：{f}")
    print(f"mesh：{n_mesh} 个改成相对路径 + meshdir")

    # ---- option / visual ----
    opt = root.find("option")
    if opt is None:
        opt = ET.Element("option")
        root.insert(list(root).index(compiler) + 1, opt)
    opt.set("timestep", _fmt(args.timestep))
    opt.set("integrator", "implicitfast")
    opt.set("gravity", "0 0 -9.81")

    vis = ET.Element("visual")
    ET.SubElement(vis, "headlight", {"diffuse": "0.8 0.8 0.8", "ambient": "0.35 0.35 0.35"})
    ET.SubElement(vis, "map", {"znear": "0.01", "zfar": "20"})
    root.insert(list(root).index(opt) + 1, vis)

    # ---- worldbody：地面 + 光照；指尖加 pad site ----
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise SystemExit("!! 产物里没有 <worldbody>")
    if not args.no_floor:
        ET.SubElement(worldbody, "light", {"pos": "1 1 2", "dir": "-0.3 -0.3 -1"})
        ET.SubElement(worldbody, "geom", {
            "name": "floor", "type": "plane", "size": "2 2 0.05",
            "rgba": "0.2 0.2 0.25 1",
        })
        print("worldbody：加了地面与光照")

    bodies = {b.get("name"): b for b in worldbody.iter("body")}
    n_site = 0
    for side in ("left", "right"):
        for finger in FINGERS:
            bname = PAD_BODY[finger].format(side=side)
            b = bodies.get(bname)
            if b is None:
                raise SystemExit(f"!! 找不到指尖 body {bname}")
            pos, size = PAD_SITE[(side, finger)]
            ET.SubElement(b, "site", {
                "name": f"pad_{side}_{finger}", "pos": pos, "size": size,
                "rgba": "0 0.8 1 0.25",
            })
            n_site += 1
    print(f"指尖 site：{n_site} 个")

    # ---- 触觉 geom 接触参数 ----
    n_tac = 0
    for g in root.iter("geom"):
        if (g.get("name") or "").startswith("tac_"):
            g.set("solref", TAC_SOLREF)
            g.set("solimp", TAC_SOLIMP)
            g.set("friction", TAC_FRICTION)
            n_tac += 1
    print(f"触觉 geom：{n_tac} 个加了接触参数")
    if n_tac != 1200:
        raise SystemExit(f"!! 期望 1200 个触觉 geom，实际 {n_tac}")

    # ---- actuator：手部 24 个位置执行器 ----
    act = ET.Element("actuator")
    n_act = 0
    for side in ("left", "right"):
        for name in HAND_JOINT_NAMES:
            jname = f"{side}_hand_{name}"
            lo, hi = limits.get(jname, (0.0, 0.0))
            ET.SubElement(act, "position", {
                "name": f"act_{jname}", "joint": jname,
                "kp": _fmt(args.kp), "kv": _fmt(args.kv),
                "ctrlrange": f"{_fmt(lo)} {_fmt(hi)}",
                "forcerange": f"{-args.force:g} {args.force:g}",
            })
            n_act += 1
    root.append(act)
    print(f"执行器：{n_act} 个（kp={args.kp} kv={args.kv} forcerange=±{args.force}N·m）")

    # ---- sensor：10 个 touch ----
    sensor = ET.Element("sensor")
    for side in ("left", "right"):
        for finger in FINGERS:
            ET.SubElement(sensor, "touch", {
                "name": f"ft_{side}_{finger}", "site": f"pad_{side}_{finger}",
            })
    root.append(sensor)
    print("传感器：10 个 touch（指尖合力，法向标量）")

    # ---- 写出（带生成头）----
    body_xml = ET.tostring(root, encoding="unicode")
    header = (
        "<!-- 本文件由 robot_description/scripts/make_xnero_mjcf.py 生成，勿手改。\n"
        f"     生成时间 : {_dt.datetime.now().isoformat(timespec='seconds')}\n"
        f"     源 URDF  : {args.urdf.name} (md5 {urdf_md5})\n"
        "     重新生成 : python3 scripts/make_mjcf.py\n"
        "     meshdir 为相对路径（源码树可直接加载）；ROS 节点加载时会覆盖为\n"
        "     robot_description share 下的相对 mesh 路径。 -->\n"
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + header + body_xml + "\n",
                        encoding="utf-8")
    print(f"写出：{args.out}（{args.out.stat().st_size} 字节）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
