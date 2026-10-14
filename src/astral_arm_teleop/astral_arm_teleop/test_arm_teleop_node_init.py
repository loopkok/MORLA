"""astral_arm_teleop_node 的 _go_init(direct) 路径构建测试（rclpy，需 ROS）。

段间回位直达（direct=True，/teleop/init_direct）：_homing_path 应**只有**
init_pose 一个点（不经 init_waypoints）；工作位（direct=False，/teleop/init）：
init_waypoints 正序 → init_pose。两条路径共用同一 homing 机（disarm /
_homing / _homing_mode="init" / 到点保持语义相同）。

装置说明：节点默认 move_to_init_pose=true 会在构造时**自动启动一次 init 归位**
（启动归位行为，非本测试关注点）——fixture 复位到"空闲未归位"基线，并注入一个
真实途经点让"途经点 vs 直达"路径差异可测。无 ROS 时整文件 skip（sys.exit(0)）。
"""

from __future__ import annotations

import os
import sys

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from std_msgs.msg import Bool  # noqa: E402

from astral_arm_teleop.astral_arm_teleop_node import (  # noqa: E402
    AstralTeleopArmNode,
)


@pytest.fixture()
def arm():
    # 构造期注入：move_to_init_pose=false 关掉"构造即自动归位"，给一个真实途经点
    # 让"途经点 vs 直达"路径差异可测（init_waypoints 是只读参数，须构造期注入）。
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "move_to_init_pose:=false",
            "-p", "init_waypoints:=[-1.6,0.2,0.0,-1.92,-0.2,0.0,0.0]",
        ]
    )
    node = AstralTeleopArmNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_go_init_waypoints_path_includes_waypoints(arm) -> None:
    ok, _ = arm._go_init(direct=False)
    assert ok
    # 工作位：途经点正序 → init_pose（此处 1 途经点 + init_pose = 2 点）
    assert len(arm._homing_path) == 2
    assert np.allclose(arm._homing_path[-1], arm._init_q_hw)
    assert arm._homing_mode == "init"
    assert arm._homing is True
    assert arm._armed is False


def test_go_init_direct_path_is_single_init_pose(arm) -> None:
    ok, msg = arm._go_init(direct=True)
    assert ok
    # 段间回位直达：路径 = 只有 init_pose 一个点（不经 init_waypoints）
    assert len(arm._homing_path) == 1
    assert np.allclose(arm._homing_path[0], arm._init_q_hw)
    assert arm._homing_mode == "init"
    assert arm._homing is True
    assert arm._armed is False
    assert "直达" in msg


def test_on_init_direct_triggers_direct_path(arm) -> None:
    arm._on_init_direct(Bool(data=True))
    assert len(arm._homing_path) == 1
    assert np.allclose(arm._homing_path[0], arm._init_q_hw)


def test_on_init_triggers_waypoints_path(arm) -> None:
    arm._on_init(Bool(data=True))
    assert len(arm._homing_path) == 2
    assert np.allclose(arm._homing_path[-1], arm._init_q_hw)


def test_init_direct_level_guard_ignores_false(arm) -> None:
    arm._on_init_direct(Bool(data=False))
    assert arm._homing is False  # 未触发


def test_go_init_rejects_while_homing(arm) -> None:
    arm._go_init(direct=True)
    # homing 进行中，再次触发（无论是否 direct）应被拒
    ok, msg = arm._go_init(direct=False)
    assert not ok
    assert "in progress" in msg
    ok, msg = arm._go_init(direct=True)
    assert not ok
    assert "in progress" in msg


def test_idle_baseline_not_homing(arm) -> None:
    # move_to_init_pose=false 构造 → 不自动归位（空闲，未在回位中）
    assert arm._homing is False


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q", "-p", "no:anyio"]))
