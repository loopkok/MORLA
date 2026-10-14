"""共享 fixture 与 import 路径设置。"""
import sys
from pathlib import Path

import pytest

# 有些兄弟包（如 nero_quest_teleop）可能还没 colcon 构建过；直接从源码树加路径，
# 这样单测不必依赖整个工作空间构建完成。
_SRC = Path(__file__).resolve().parents[2]
for _pkg in ("nero_quest_teleop",):
    _p = _SRC / _pkg
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


@pytest.fixture(scope="session", autouse=True)
def _ros_context():
    rclpy = pytest.importorskip("rclpy")
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def sim_node():
    """一个关掉 viewer 的 sim 节点，用例结束自动销毁。"""
    pytest.importorskip("mujoco")
    from xnero_mujoco_sim.sim_node import XneroMujocoSimNode
    node = XneroMujocoSimNode(viewer=False)
    yield node
    node.destroy_node()
