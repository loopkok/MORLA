"""臂：运动学覆盖 —— 命令必须精确等于 qpos，且速度被清零。"""
import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("mujoco")

from xnero_mujoco_sim.hand_map import ARM_STATE_JOINT_NAMES, arm_joint_names  # noqa: E402


def _arm_cmd(side: str, values) -> "JointState":
    from sensor_msgs.msg import JointState
    js = JointState()
    js.name = arm_joint_names(side)
    js.position = [float(v) for v in values]
    return js


def test_arm_is_kinematically_overridden(sim_node):
    """发命令 → qpos 精确等于命令（不是"跟到附近"），qvel 恒为 0。"""
    import rclpy
    from rclpy.node import Node

    sim = sim_node
    probe = Node("probe_arm")
    pub = probe.create_publisher(
        __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState,
        "/left_arm/joint_commands", 10)
    want = np.array([0.3, -0.2, 0.4, -0.5, 0.1, 0.6, -0.3])
    try:
        for _ in range(50):
            pub.publish(_arm_cmd("left", want))
            sim.step_once()
            rclpy.spin_once(probe, timeout_sec=0.0)
        q = sim.data.qpos[sim._arm_qadr["left"]]
        v = sim.data.qvel[sim._arm_vadr["left"]]
        assert np.allclose(q, want, atol=1e-9), f"qpos={q} 命令={want}"
        assert np.allclose(v, 0.0, atol=1e-12), f"qvel 没清零：{v}"
    finally:
        probe.destroy_node()


def test_arm_by_name_matching_is_order_independent(sim_node):
    """命令里 name 乱序也要对上（真机 teleop 会带 name）。"""
    import rclpy
    from rclpy.node import Node

    sim = sim_node
    probe = Node("probe_arm2")
    pub = probe.create_publisher(
        __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState,
        "/right_arm/joint_commands", 10)
    names = list(reversed(arm_joint_names("right")))
    vals = [0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77]   # 对应 names 的顺序
    msg = __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState()
    msg.name = names
    msg.position = vals
    try:
        for _ in range(50):
            pub.publish(msg)
            sim.step_once()
            rclpy.spin_once(probe, timeout_sec=0.0)
        got = sim.data.qpos[sim._arm_qadr["right"]]
        want = np.array(vals)[::-1]      # names 反序 → 值也要反着对上
        assert np.allclose(got, want, atol=1e-9), f"got={got} want={want}"
    finally:
        probe.destroy_node()


def test_arm_state_published(sim_node):
    """/{side}_arm/joint_states publishes standard measured state."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    sim = sim_node
    probe = Node("probe_arm3")
    got = {}
    probe.create_subscription(JointState, "/left_arm/joint_states",
                              lambda m: got.__setitem__("m", m), 10)
    pub = probe.create_publisher(JointState, "/left_arm/joint_commands", 10)
    want = np.array([0.1] * 7)
    try:
        for _ in range(120):
            pub.publish(_arm_cmd("left", want))
            sim.step_once()
            rclpy.spin_once(probe, timeout_sec=0.0)
        for _ in range(20):
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert got, "没收到 /left_arm/joint_states"
        m = got["m"]
        assert list(m.name) == ARM_STATE_JOINT_NAMES["left"]
        assert np.allclose(m.position, want, atol=1e-9)
    finally:
        probe.destroy_node()


def test_arm_timeout_holds_last_value(sim_node):
    """超时后保持最后值，不许归零（否则臂会突然弹回）。"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    sim = sim_node
    probe = Node("probe_arm4")
    pub = probe.create_publisher(JointState, "/left_arm/joint_commands", 10)
    want = np.array([0.5] * 7)
    try:
        for _ in range(30):
            pub.publish(_arm_cmd("left", want))
            sim.step_once()
            rclpy.spin_once(probe, timeout_sec=0.0)
        sim._arm_cmd_mono["left"] = 0.0          # 伪造"很久没收到命令"
        for _ in range(30):
            sim.step_once()
        got = sim.data.qpos[sim._arm_qadr["left"]]
        assert np.allclose(got, want, atol=1e-9), f"超时后没保持：{got}"
    finally:
        probe.destroy_node()
