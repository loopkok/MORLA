#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_tactile_mujoco.py — MuJoCo 端触觉阵列自检：压一下指尖，看逐点力出不出来。

做三件事：
  1. 调 prep_sim_urdf.py 生成仿真副本（修 package:// + 退化惯量，可选关 mesh 碰撞）
  2. 让 MuJoCo 加载，核对 1200 个触觉 geom 是否都在、名字对不对
  3. 用一个 mocap 探针球沿外法线压进指定测点，打印逐点法向力

关键结论（本脚本会实测复现）：**指尖 mesh 的 collision 必须关掉**，否则它会把
30~40% 的接触力吃掉，而触觉阵列看不见这部分 —— 逐点力会系统性偏小且分布失真。

用法
----
  python3 scripts/test_tactile_mujoco.py                    # 默认压左手食指 p100
  python3 scripts/test_tactile_mujoco.py --point tac_right_thumb_p060 --pen 1.0
  python3 scripts/test_tactile_mujoco.py --with-mesh        # 对照：保留 mesh 碰撞
  python3 scripts/test_tactile_mujoco.py --sweep            # 扫压入深度
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = PKG_ROOT / "urdf" / "nero" / "xhand_nero_description.urdf"
PREP = PKG_ROOT / "scripts" / "prep_xnero_sim_urdf.py"

# 装了触觉阵列的那 5 个指尖掌侧 link2（每手 5 个）——精确匹配，不用通配
PAD_LINK2_RE = r"^(left|right)_hand_(thumb_rota|index_rota|mid|ring|pinky)_link2$"


def build_sim_urdf(urdf: Path, workdir: Path, keep_mesh: bool) -> Path:
    cmd = [sys.executable, str(PREP), "--urdf", str(urdf), "--out", str(workdir)]
    if not keep_mesh:
        # 只关 5 个「掌侧指尖 link2」的 mesh —— 注意别用 .*link2$，那会连带关掉
        # nero 臂的 left_link2/right_link2 和手背的 *_rotaback_link2
        cmd += ["--no-collision", PAD_LINK2_RE]
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout.strip())
    if r.returncode:
        print(r.stderr); raise SystemExit("prep_sim_urdf.py 失败")
    return workdir / "robot_sim.urdf"


def main(argv=None):
    ap = argparse.ArgumentParser(description="MuJoCo 触觉阵列自检")
    ap.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    ap.add_argument("--point", default="tac_left_index_p100", help="被压的触觉 collision 名")
    ap.add_argument("--ball-radius", type=float, default=2.0, help="探针球半径 mm")
    ap.add_argument("--pen", type=float, default=0.8, help="压入深度 mm")
    ap.add_argument("--with-mesh", action="store_true", help="对照组：保留指尖 mesh 碰撞")
    ap.add_argument("--sweep", action="store_true", help="扫压入深度")
    args = ap.parse_args(argv)

    import mujoco   # 放在参数解析之后，--help 不需要 mujoco

    tmp = Path(tempfile.mkdtemp(prefix="xhand_tac_"))
    sim_urdf = build_sim_urdf(args.urdf, tmp, args.with_mesh)

    spec = mujoco.MjSpec.from_file(str(sim_urdf))
    probe = spec.worldbody.add_body(name="probe", mocap=True)
    probe.add_geom(name="probe_ball", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                   size=[args.ball_radius / 1000.0, 0, 0], rgba=[0.2, 0.8, 1, 0.6])
    m = spec.compile()
    d = mujoco.MjData(m)
    print(f"\n模型：nbody={m.nbody} ngeom={m.ngeom} nq={m.nq} 总质量={m.body_mass.sum():.3f} kg")

    # --- 1) geom 点名 ---
    gname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
    tac = [i for i in range(m.ngeom) if gname(i).startswith("tac_")]
    print(f"触觉 geom：{len(tac)} / 1200 {'✓' if len(tac) == 1200 else '✗'}")
    if len(tac) != 1200:
        return 1
    missing = []
    for side in ("left", "right"):
        for fing in ("thumb", "index", "mid", "ring", "pinky"):
            for p in range(1, 121):
                if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM,
                                     f"tac_{side}_{fing}_p{p:03d}") < 0:
                    missing.append(f"tac_{side}_{fing}_p{p:03d}")
    print(f"命名完整性：{'全部 1200 个名字都能查到 ✓' if not missing else f'缺 {len(missing)} 个 ✗'}")

    # --- 2) 压测 ---
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, args.point)
    if gid < 0:
        raise SystemExit(f"找不到 geom {args.point}")
    bid = int(m.geom_bodyid[gid])
    mujoco.mj_forward(m, d)
    p = d.geom_xpos[gid].copy()
    out = p - d.xpos[bid]
    out /= np.linalg.norm(out)
    ball = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "probe_ball")

    def press(pen_mm):
        d.mocap_pos[0] = p - out * (args.ball_radius / 1000.0 + 0.0005 - pen_mm / 1000.0)
        mujoco.mj_forward(m, d)
        res = np.zeros(6)
        per, mesh = {}, 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if ball not in (g1, g2):
                continue
            other = g2 if g1 == ball else g1
            mujoco.mj_contactForce(m, d, i, res)
            fn = abs(float(res[0]))
            nm = gname(other)
            if nm.startswith("tac_"):
                per[nm] = per.get(nm, 0.0) + fn
            else:
                mesh += fn
        return per, mesh

    print(f"\n压测点：{args.point}（body={mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bid)}）")
    if args.sweep:
        print(f"{'压入(mm)':>9} {'触觉点':>7} {'触觉合力(N)':>12} {'mesh 拿走(N)':>13}")
        for pen in (0.2, 0.5, 0.8, 1.5, 3.0):
            per, mesh = press(pen)
            print(f"{pen:>9.1f} {len(per):>7} {sum(per.values()):>12.4f} {mesh:>13.4f}")
        return 0

    per, mesh = press(args.pen)
    tot = sum(per.values()) + mesh
    print(f"  压入 {args.pen} mm")
    for k, v in sorted(per.items()):
        print(f"    {k:26s} {v:.4f} N")
    if mesh:
        print(f"    指尖 mesh（触觉看不见）      {mesh:.4f} N  ← {mesh/tot*100:.0f}% 的力被它吃掉")
    print(f"  触觉合力 {sum(per.values()):.4f} N / 总 {tot:.4f} N "
          f"= {sum(per.values())/tot*100:.1f}%")
    ok = sum(per.values()) > 0 and (args.with_mesh or mesh == 0)
    print(f"\n{'✓ 通过：逐点力正常' if ok else '✗ 不通过：触觉探针没吃到力'}"
          + ("" if args.with_mesh else "（且 mesh 未抢力）"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
