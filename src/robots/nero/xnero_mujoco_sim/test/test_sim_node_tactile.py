"""触觉注入：xhand_state.sensor_states 的逐点力必须与 MuJoCo 接触力一致。

做法：生成一份「在原模型上焊一个探针球、精确压在左手食指尖」的测试 MJCF
（确定性，不依赖地面/重力），让 sim 节点加载它，再把手指压上去。
"""
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("mujoco")

from xnero_mujoco_sim.hand_map import FINGERS, HAND_JOINT_NAMES  # noqa: E402

CURL = 1.9          # 手指屈曲命令（rad）
# T16 的 120 点是列优先网格（每列 12 点沿手指方向），p060 = 第 5 列第 12 行
# 正好落在指尖关节 z=42.25mm 处，会被融进同 body 的 *_rota_tip 碰撞 mesh 压住。
# 取第 3 列第 6 行（pad 中段）才是干净的触觉接触。
PROBE_POINT = 30


@pytest.fixture(scope="module")
def probe_assets(tmp_path_factory):
    """返回 (测试 MJCF 路径, 预期指尖 world 坐标)。"""
    import mujoco
    from xnero_mujoco_sim.sim_node import default_mjcf_path, rewrite_meshdir
    from morla_core.resource_locator import package_share

    meshdir = str(package_share("robot_description") / "meshes")
    xml = rewrite_meshdir(default_mjcf_path().read_text(encoding="utf-8"), meshdir)

    # 1) 先算出「食指屈曲后，p060 这个测点在世界的哪儿、外法线朝哪」
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for jn in ("left_hand_index_joint1", "left_hand_index_joint2"):
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + jn)
        d.ctrl[a] = CURL
    # 必须像节点那样把臂锁在 init_pose（全 0）—— 否则臂在重力下垂，
    # 算出来的指尖位置与节点运行时的姿态对不上，球就焊歪了
    arm_qadr, arm_vadr = [], []
    for side in ("left", "right"):
        for i in range(1, 8):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_joint{i}")
            arm_qadr.append(int(m.jnt_qposadr[j]))
            arm_vadr.append(int(m.jnt_dofadr[j]))
    for _ in range(1500):
        d.qpos[arm_qadr] = 0.0
        d.qvel[arm_vadr] = 0.0
        mujoco.mj_step(m, d)
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM,
                            f"tac_left_index_p{PROBE_POINT:03d}")
    p = d.geom_xpos[gid].copy()
    bid = int(m.geom_bodyid[gid])
    outward = p - d.xpos[bid]
    outward /= np.linalg.norm(outward)
    # 球必须够小：pad 所在的 link2 body 里还融着 *_rota_tip / *_rotaback_link2 的
    # 碰撞 mesh（fixed 子链被 fusestatic 并进来了），4mm 球会连带顶到它们。
    ball_r, probe_r, pen = 0.001, 0.0005, 0.0003
    c = p + outward * (ball_r + probe_r - pen)

    # 2) 把球焊进一份测试 MJCF
    ball = (f'<geom name="probe_ball" type="sphere" size="{ball_r}" '
            f'pos="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" rgba="0.2 0.8 1 0.6"/>')
    out = xml.replace("<worldbody>", "<worldbody>\n" + ball, 1)
    pth = tmp_path_factory.mktemp("tac") / "probe.xml"
    pth.write_text(out, encoding="utf-8")
    return pth, p


@pytest.fixture(scope="module", autouse=True)
def _ros():
    """rclpy 只能 init 一次 —— 整个模块统一管理，别在用例里反复 init/shutdown。"""
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _node(mjcf: Path):
    from xnero_mujoco_sim.sim_node import XneroMujocoSimNode
    return XneroMujocoSimNode(
        viewer=False,
        parameter_overrides={"mjcf_path": str(mjcf), "enable_viewer": False})


def _curl_and_step(sim, n=400):
    import mujoco
    for jn in ("left_hand_index_joint1", "left_hand_index_joint2"):
        a = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + jn)
        sim.data.ctrl[a] = CURL
    for _ in range(n):
        sim.step_once()
        # step_once 会用 _hand_target 覆盖 ctrl；测完再补一次
        for jn in ("left_hand_index_joint1", "left_hand_index_joint2"):
            a = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + jn)
            sim.data.ctrl[a] = CURL


def test_collect_tactile_shape_and_consistency(probe_assets):
    import mujoco
    mjcf, _ = probe_assets
    sim = _node(mjcf)
    try:
        _curl_and_step(sim, 600)
        tac = sim.collect_tactile("left")
        assert set(tac) == set(FINGERS)
        for f in FINGERS:
            assert tac[f]["raw"].shape == (120, 3)
            assert tac[f]["calc"].shape == (3,)

        # 先确认接触对象就是触觉探针（否则说明球压到了指尖/指背 mesh，测试无效）
        hit = []
        for i in range(sim.data.ncon):
            for g in (int(sim.data.contact[i].geom1), int(sim.data.contact[i].geom2)):
                nm = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}"
                hit.append(nm)
        assert any(h.startswith("tac_left_index") for h in hit), \
            f"球没压在触觉探针上，接触对象={sorted(set(hit))}"

        idx = tac["index"]
        nz = int((np.abs(idx["raw"]).sum(axis=1) > 0).sum())
        assert nz > 0, "食指没有任何测点吃到力"
        # 逐点之和 == calc_force
        assert np.allclose(idx["raw"].sum(axis=0), idx["calc"], atol=1e-12)
        # 两条**原理性**不变量（别拿 raw 的某个分量去等于接触法向 —— 那只是巧合）：
        #   a) 非零点数 == 该指触觉 geom 参与的接触数（逐点视图没有漏报/多报）
        #   b) touch 传感器读数 == 同一批接触的 |接触法向| 之和
        #      （touch = site zone 内接触法向之和，用它反验我们看到的接触是完整的）
        m, d = sim.model, sim.data
        n_contact_idx, normal_sum = 0, 0.0
        res = np.zeros(6)
        for i in range(d.ncon):
            g1, g2 = int(d.contact[i].geom1), int(d.contact[i].geom2)
            names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in (g1, g2)]
            if not any(n.startswith("tac_left_index") for n in names):
                continue
            n_contact_idx += 1
            mujoco.mj_contactForce(m, d, i, res)
            normal_sum += abs(float(res[0]))
        assert n_contact_idx > 0, "本案应该至少有 1 个食指接触"
        assert nz == n_contact_idx, f"逐点非零 {nz} != 实际接触数 {n_contact_idx}"

        ft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "ft_left_index")
        sensor_val = float(d.sensordata[m.sensor_adr[ft]])
        assert sensor_val == pytest.approx(normal_sum, abs=1e-6), \
            f"touch 传感器 {sensor_val} != 逐接触法向之和 {normal_sum}"
        # 各点力的**幅值之和** >= 法向之和（每点还含切向分量），量级自洽。
        # 注意不能写成 |矢量和| —— 多个力方向不同会相互抵消，|ΣF| 可以小于 Σ|F_n|。
        total_mag = float(np.linalg.norm(idx["raw"], axis=1).sum())
        assert total_mag >= sensor_val - 1e-9, \
            f"逐点幅值和 {total_mag} 竟小于法向之和 {sensor_val}"

        # 其他指不该被误记
        for f in ("mid", "ring", "pinky", "thumb"):
            assert np.abs(tac[f]["raw"]).sum() == 0.0, f"{f} 被误记了力"
    finally:
        sim.destroy_node()


def test_xhand_state_carries_tactile(probe_assets):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from xhand_control_interfaces.msg import XHandStateArray

    mjcf, _ = probe_assets
    sim = _node(mjcf)
    probe = Node("probe_tac")
    got = {}
    probe.create_subscription(XHandStateArray, "/left_hand/driver/xhand_state",
                              lambda msg: got.__setitem__("m", msg), qos_profile_sensor_data)
    try:
        _curl_and_step(sim, 400)
        for _ in range(20):
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert got, "没收到状态"
        m = got["m"]
        assert len(m.sensor_states) == 1
        fss = m.sensor_states[0].finger_sensor_states
        assert [x.location for x in fss] == ["thumb", "index", "middle", "ring", "pinky"]
        for x in fss:
            assert len(x.raw_force) == 120
            assert len(x.raw_temperature) == 20
        forces = {x.location: sum(abs(v) for p in x.raw_force for v in (p.x, p.y, p.z))
                  for x in fss}
        assert forces["index"] > 0.0, f"食指触觉为 0：{forces}"
        for loc in ("thumb", "middle", "ring", "pinky"):
            assert forces[loc] == 0.0, f"{loc} 被误记：{forces}"
    finally:
        sim.destroy_node()
        probe.destroy_node()
