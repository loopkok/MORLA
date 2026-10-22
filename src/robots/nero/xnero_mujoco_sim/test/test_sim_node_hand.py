"""sim 节点：手部命令打包 + meshdir 解析 + 端到端手部跟踪。"""
import numpy as np
import pytest

from xnero_mujoco_sim.hand_map import HAND_JOINT_NAMES

pytest.importorskip("rclpy")
pytest.importorskip("mujoco")


def _cmd(names, positions):
    from xhand_control_interfaces.msg import XHandCommand
    m = XHandCommand()
    m.name = list(names)
    m.position = [float(v) for v in positions]
    m.kp = [80.0] * 12
    m.ki = [0.0] * 12
    m.kd = [0.0] * 12
    m.effort_limit = [400.0] * 12
    m.mode = 3
    return m


def test_pack_hand_command_by_name():
    """命令里 name 是乱序时，必须按名字对上，不能按数组顺序硬套。"""
    from xnero_mujoco_sim.sim_node import pack_hand_command
    rev = list(reversed(HAND_JOINT_NAMES))
    out = pack_hand_command(_cmd(rev, list(range(12))[::-1]), "left")
    assert set(out) == {f"left_hand_{n}" for n in HAND_JOINT_NAMES}
    assert out["left_hand_thumb_bend_joint"] == 0.0
    assert out["left_hand_pinky_joint2"] == 11.0


def test_pack_hand_command_without_names_is_rejected():
    from xnero_mujoco_sim.sim_node import pack_hand_command
    assert pack_hand_command(_cmd([], [0.1] * 12), "right") is None


def test_pack_hand_command_rejects_short():
    from xnero_mujoco_sim.sim_node import pack_hand_command
    assert pack_hand_command(_cmd([], [0.1] * 5), "left") is None


def test_rewrite_meshdir():
    from xnero_mujoco_sim.sim_node import rewrite_meshdir
    src = '<mujoco><compiler meshdir="../../rel"/><asset/></mujoco>'
    out = rewrite_meshdir(src, "/abs/meshes")
    assert 'meshdir="/abs/meshes"' in out
    # 没有 meshdir 时要插进去
    out2 = rewrite_meshdir('<mujoco><compiler angle="radian"/></mujoco>', "/abs/meshes")
    assert 'meshdir="/abs/meshes"' in out2


def test_hand_tracking_end_to_end(sim_node):
    """发 0.6 rad 命令，手关节应跟上去（真物理 + 位置执行器）。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from xhand_control_interfaces.msg import XHandStateArray

    sim = sim_node
    probe = Node("probe_hand")
    got = {}
    # 记「最后一条」而不是第一条 —— 第一条是瞬态快照，采样它会把收敛前的中间态当结果
    probe.create_subscription(
        XHandStateArray, "/left_hand/driver/xhand_state",
        lambda m: got.__setitem__("pos", list(m.hand_states[0].position)), qos_profile_sensor_data)
    pub = probe.create_publisher(
        __import__("xhand_control_interfaces.msg", fromlist=["XHandCommand"]).XHandCommand,
        "/left_hand/driver/xhand_command", 10)
    cmd = _cmd(HAND_JOINT_NAMES, [0.6] * 12)
    try:
        for _ in range(500):
            pub.publish(cmd)
            sim.step_once()
            rclpy.spin_once(probe, timeout_sec=0.0)
        # 再空跑一段把状态抽干，确保拿到收敛后的稳态
        for _ in range(300):
            sim.step_once()
            rclpy.spin_once(probe, timeout_sec=0.0)
        for _ in range(20):
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert got, "没收到 /left_hand/driver/xhand_state"
        arr = np.array(got["pos"])
        assert arr.shape == (12,)
        # index_bend_joint 会贴当前模型的上限；其余 11 个应跟到 ~0.6。
        # 旧 ±0.174 断言属于 legacy MJCF，当前模型是 -5°..17°。
        others = np.delete(arr, 3)
        assert others.mean() > 0.55, f"手没跟上命令：{np.round(arr, 3)}"
        assert np.all(others > 0.4), f"有点没动：{np.round(arr, 3)}"
        joint = "left_hand_index_bend_joint"
        lower, upper = sim.model.joint(joint).range
        # 目标必须截在精确限位；MuJoCo 软约束实测允许 0.002rad 数值偏差。
        assert sim.data.ctrl[sim._hand_act[joint]] == pytest.approx(upper)
        assert lower - 0.002 <= arr[3] <= upper + 0.002, "index_bend 超出当前模型限位"
    finally:
        probe.destroy_node()
