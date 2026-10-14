#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""prep_sim_urdf.py — 把厂商 URDF 处理成仿真器能读的副本（不改原文件）。

解决三件事，都是仿真器侧的要求（厂商 URDF 本身在 RViz 里没问题）：

1. `package://` 路径 —— MuJoCo 不认，必须换成文件系统路径。
   （Isaac Sim 认 package://，但如果用 Isaac Lab 的 UrdfConverter 走文件系统，
     也可以直接用处理后的副本。）

2. **退化惯量张量** —— 厂商把 34 个 link 的六个惯量分量（含非对角项）全写成同一个值
   `1e-11`。那是秩 1 矩阵，特征值 (3e-11, 0, 0)，MuJoCo 直接报
   `inertia must have positive eigenvalues` 拒收整个模型；PhysX 也会告警。
   修法：保留对角量级（对 1e-5 kg 的指节，1e-11 kg·m² 正是合理量级），非对角置 0
   —— 得到各向同性、正定的有效张量。

3. **可选：关掉指定 link 的 collision** —— 触觉探针球与指尖 mesh 共面，要让「逐点力」
   有意义，需要让探针而不是 mesh 去承受接触。用 `--no-collision <link 名或正则>` 关掉
   （此选项只影响副本，厂商文件不动）。

用法
----
  python3 scripts/prep_sim_urdf.py                          # 默认输出 /tmp/xhand_sim/
  python3 scripts/prep_sim_urdf.py --out build/sim          # 指定输出目录
  python3 scripts/prep_sim_urdf.py --no-collision '.*_link2$'   # 指尖 link2 不再参与碰撞
  python3 scripts/prep_sim_urdf.py --report                 # 只体检，不输出
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

PKG_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = PKG_ROOT / "urdf" / "nero" / "xhand_nero_description.urdf"


def fmt(v: float) -> str:
    s = f"{v:.6g}"
    return "0" if s in ("-0", "-0.0") else s


def inertia_eigs(ixx, iyy, izz, ixy, ixz, iyz):
    """3x3 对称阵的特征值（不依赖 numpy，方便在纯 ROS 环境里跑）。"""
    tr = ixx + iyy + izz
    # 解析解：对称阵特征值
    p1 = ixy * ixy + ixz * ixz + iyz * iyz
    if p1 == 0:
        return sorted([ixx, iyy, izz])
    q = tr / 3.0
    p2 = (ixx - q) ** 2 + (iyy - q) ** 2 + (izz - q) ** 2 + 2 * p1
    p = (p2 / 6.0) ** 0.5
    if p == 0:
        return [q, q, q]
    b = [[(ixx - q) / p, ixy / p, ixz / p],
         [ixy / p, (iyy - q) / p, iyz / p],
         [ixz / p, iyz / p, (izz - q) / p]]
    det = (b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1])
           - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0])
           + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]))
    r = max(-1.0, min(1.0, det / 2.0))
    import math
    phi = math.acos(r) / 3.0
    e1 = q + 2 * p * math.cos(phi)
    e3 = q + 2 * p * math.cos(phi + 2 * math.pi / 3)
    e2 = 3 * q - e1 - e3
    return sorted([e1, e2, e3])


def main(argv=None):
    ap = argparse.ArgumentParser(description="URDF → 仿真器可读副本")
    ap.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    ap.add_argument("--out", type=Path, default=Path("/tmp/xhand_sim"))
    ap.add_argument("--no-collision", action="append", default=[],
                    metavar="REGEX", help="关掉名字匹配该正则的 link 的 collision（可多次）")
    ap.add_argument("--keep-mesh", action="store_true",
                    help="保留 package:// 写法（Isaac Sim 认；MuJoCo 不认）")
    ap.add_argument("--report", action="store_true", help="只体检，不写文件")
    args = ap.parse_args(argv)

    text = args.urdf.read_text(encoding="utf-8")
    # 保留注释（TACTILE 标记、厂商抬头说明都要留住，便于人工核对）
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(text, parser=parser)

    # ---- 体检：退化惯量 ----
    bad = []
    for l in root.findall("link"):
        ine = l.find("inertial")
        if ine is None or ine.find("inertia") is None:
            continue
        I = ine.find("inertia")
        v = {k: float(I.get(k, 0)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        ev = inertia_eigs(**{k: v[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")})
        if ev[-1] <= 0 or ev[0] / ev[-1] < 1e-3:
            bad.append((l.get("name"), v))

    print(f"体检：退化惯量 link {len(bad)} 个 / 总 link {len(root.findall('link'))} 个")
    for n, v in bad[:3]:
        print(f"   {n}: ixx={v['ixx']:g} ixy={v['ixy']:g} …（六分量同值 → 秩 1）")
    if len(bad) > 3:
        print(f"   …其余 {len(bad) - 3} 个签名相同")
    if args.report:
        return 0

    # ---- 修复 1：退化惯量 → 对角各向同性 ----
    fixed = 0
    for l in root.findall("link"):
        ine = l.find("inertial")
        if ine is None or ine.find("inertia") is None:
            continue
        I = ine.find("inertia")
        v = {k: float(I.get(k, 0)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        ev = inertia_eigs(**v)
        if ev[-1] <= 0 or ev[0] / ev[-1] < 1e-3:
            I.set("ixx", fmt(v["ixx"])); I.set("iyy", fmt(v["iyy"])); I.set("izz", fmt(v["izz"]))
            for k in ("ixy", "ixz", "iyz"):
                I.set(k, "0")
            fixed += 1
    print(f"修复：{fixed} 个退化惯量 → 对角各向同性（量级不变）")

    # ---- 修复 2：package:// 路径 ----
    out_text = ET.tostring(root, encoding="unicode")
    out_text = '<?xml version="1.0" encoding="utf-8"?>\n' + out_text + "\n"

    if not args.keep_mesh:
        out_text = out_text.replace("package://robot_description/",
                                    str(PKG_ROOT.resolve()) + "/")
        print(f"路径：package:// → {PKG_ROOT.resolve()}/")

    # ---- 修复 3：关掉指定 link 的 collision ----
    for pat in args.no_collision:
        rx = re.compile(pat)
        n = 0
        for l in root.findall("link"):
            if not rx.search(l.get("name") or ""):
                continue
            for c in list(l.findall("collision")):
                l.remove(c)
                n += 1
        print(f"碰撞：link 匹配 /{pat}/ 的 collision 关掉 {n} 个")
        out_text = ('<?xml version="1.0" encoding="utf-8"?>\n'
                    + ET.tostring(root, encoding="unicode") + "\n")
        if not args.keep_mesh:
            out_text = out_text.replace("package://robot_description/",
                                        str(PKG_ROOT.resolve()) + "/")

    out = args.out / "robot_sim.urdf"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(out_text, encoding="utf-8")
    print(f"输出：{out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
