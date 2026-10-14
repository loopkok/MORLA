#!/usr/bin/env python3
"""Regression tests for driver Trigger services on the enable path.

``~/enable`` must be idempotent w.r.t. the board power state:
  * robot already powered (auto_ready / one_click_ready ran) → success WITHOUT
    re-sending the WORK→POSITION→enable sequence (web「HOME」实机出现已使能后
    再 enable 误报"电机未使能"，根因=重复使能 + 板端 robot_powered 位不确认);
  * power bit not observed but board online → still success ("enable issued"),
    mirroring the SDK demos' "enable 未确认 robot_powered 仍继续" handling;
  * only a genuinely offline board yields a hard failure.

Instantiate the node via ``__new__`` (no rclpy spin) and stub ``_robot``,
matching the script-style tests in astral_arm_teleop.

Runs only where rclpy is importable (ROS-sourced system python); otherwise
the whole suite skips with exit 0.
"""

import sys
import threading
import time

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

from astral_robot_control.driver_node import AstralRobotDriverNode  # noqa: E402
from astral_robot_control.joint_layout import (  # noqa: E402
    LEFT_ARM_IDS,
    RIGHT_ARM_IDS,
)


class _FakeRes:
    def __init__(self):
        self.success = None
        self.message = ""


class _FakeRobot:
    def __init__(self, powered=False, motion_mode=1, online=True, enable_ok=False):
        self._robot_powered = powered
        self._motion_mode = motion_mode
        self.is_online = online
        self._enable_ok = enable_ok
        self.calls = []

    def set_system_mode(self, mode):
        self.calls.append(("set_system_mode", mode))

    def set_motion_mode(self, mode):
        self._motion_mode = mode
        self.calls.append(("set_motion_mode", mode))

    def enable(self, enable_timeout_s=2.0):
        self.calls.append(("enable", enable_timeout_s))
        return self._enable_ok

    def set_all_joints_zero(self):
        self.calls.append(("set_all_joints_zero",))

    def move_arm_js(self, left, right):
        self.calls.append(("move_arm_js", tuple(left), tuple(right)))

    def set_target_positions(self, targets: dict):
        self.calls.append(("set_target_positions", dict(targets)))

    def e_stop(self):
        self.calls.append(("e_stop",))

    def one_click_ready(self, enable_timeout_s=3.0, seed_from_current=False):
        self.calls.append(("one_click_ready", enable_timeout_s, seed_from_current))
        return True


def _make_node(robot=None, dry_run=False, q18=None):
    node = AstralRobotDriverNode.__new__(AstralRobotDriverNode)
    node.dry_run = dry_run
    node._robot = robot
    node._apply_lpf = lambda: None
    # 实测关节角（阻尼中被拖拽后的当前位置）。默认全零，测试按需注入。
    node._read_q18 = lambda: ([0.0] * 18 if q18 is None else list(q18))
    node._lock = threading.Lock()
    node._left_cmd = None
    node._right_cmd = None
    node._full_cmd = None
    node._head_cmd = None
    node._left_cmd_t = 0.0
    node._right_cmd_t = 0.0
    node._full_cmd_t = 0.0
    node._head_cmd_t = 0.0
    node._use_full_priority = False
    return node


def _seed_cache(node):
    """模拟遥操/启动归位刚发过的"阻尼前的陈旧目标"（仍处新鲜窗口）。"""
    with node._lock:
        node._left_cmd = [0.1] * 7
        node._left_cmd_t = 1234.0
        node._right_cmd = [0.2] * 7
        node._right_cmd_t = 1234.0
        node._full_cmd = None
        node._full_cmd_t = 0.0
        node._head_cmd = [0.0, 0.0]
        node._head_cmd_t = 1234.0


def _cache_empty(node):
    with node._lock:
        return all(
            v is None
            for v in (node._left_cmd, node._right_cmd, node._full_cmd, node._head_cmd)
        )


def test_dry_run_skips():
    node = _make_node(robot=None, dry_run=True)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "dry_run" in res.message


def test_no_robot_fails():
    node = _make_node(robot=None)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is False
    assert res.message == "robot not connected"


def test_already_powered_skips_sequence():
    robot = _FakeRobot(powered=True, motion_mode=1)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "already enabled" in res.message
    assert robot.calls == []  # 已上电绝不重复下发 WORK/POSITION/enable


def test_already_powered_but_damping_returns_to_position():
    robot = _FakeRobot(powered=True, motion_mode=0)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert robot.calls == [("set_motion_mode", 1)]


def test_enable_confirmed_ok():
    robot = _FakeRobot(powered=False, online=True, enable_ok=True)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "enable OK" in res.message
    assert ("set_system_mode", "work") in robot.calls
    assert ("enable", 3.0) in robot.calls


def test_power_bit_unconfirmed_but_online_is_success():
    # 板端在线、电源位未在窗口确认：命令已送达，算下发成功（对齐 SDK demo
    # 「enable 未确认 robot_powered 仍继续」）。否则 web HOME 会误报电机未使能。
    robot = _FakeRobot(powered=False, online=True, enable_ok=False)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "enable issued" in res.message


def test_offline_board_is_hard_failure():
    robot = _FakeRobot(powered=False, online=False, enable_ok=False)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is False
    assert "not confirmed" in res.message


# --- 阻尼/归零/一键就绪：缓存清理 + 阻尼中归零先切 POSITION ------------------

def test_home_in_damping_restores_position_then_zero_and_clears_cache():
    # 阻尼（motion=0）下板端忽略位置目标 → home 必须先切回 POSITION 再归零，
    # 且清掉"阻尼前的陈旧目标"，防止 100Hz 重发把它顶回去。
    robot = _FakeRobot(powered=True, motion_mode=0)
    node = _make_node(robot=robot)
    _seed_cache(node)
    res = _FakeRes()
    node._srv_home(None, res)
    assert res.success is True
    assert robot._motion_mode == 1
    assert ("set_motion_mode", 1) in robot.calls
    assert ("set_all_joints_zero",) in robot.calls
    assert _cache_empty(node)


def test_home_in_position_skips_motion_mode_switch():
    robot = _FakeRobot(powered=True, motion_mode=1)
    node = _make_node(robot=robot)
    _seed_cache(node)
    res = _FakeRes()
    node._srv_home(None, res)
    assert res.success is True
    assert [c for c in robot.calls if c[0] == "set_motion_mode"] == []
    assert ("set_all_joints_zero",) in robot.calls
    assert _cache_empty(node)


def test_home_unpowered_fails_with_ready_hint():
    # 急停下电后单靠 home 归不了零（没上电位置环不生效）：给明确提示而非静默。
    robot = _FakeRobot(powered=False, motion_mode=1)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_home(None, res)
    assert res.success is False
    assert "先 ~/ready" in res.message
    assert robot.calls == []


def test_ready_clears_cached_target_before_zero():
    # 一键就绪前清缓存：one_click_ready 的归零是显式目标，随后 100Hz 重发
    # 只该复读零位，不能把阻尼前的陈旧位姿重新顶上来（症状：回到阻尼前位姿）。
    # 且 ready 走 SDK one_click_ready 时传 seed_from_current=True：阻尼切回
    # POSITION 的瞬间先用实测关节角重写板端目标，避免板端追踪内部保存的旧位姿。
    robot = _FakeRobot(powered=False, online=True, enable_ok=True)
    node = _make_node(robot=robot)
    _seed_cache(node)
    res = _FakeRes()
    node._srv_ready(None, res)
    assert res.success is True
    assert ("one_click_ready", 3.0, True) in robot.calls
    assert _cache_empty(node)


def test_damping_clears_cached_target():
    robot = _FakeRobot(powered=True, motion_mode=1)
    node = _make_node(robot=robot)
    _seed_cache(node)
    res = _FakeRes()
    node._srv_damping(None, res)
    assert res.success is True
    assert robot._motion_mode == 0
    assert _cache_empty(node)


def test_estop_clears_cached_target():
    robot = _FakeRobot(powered=True, motion_mode=1)
    node = _make_node(robot=robot)
    _seed_cache(node)
    res = _FakeRes()
    node._srv_estop(None, res)
    assert res.success is True
    assert ("e_stop",) in robot.calls
    assert _cache_empty(node)


def test_home_after_damping_seeds_target_from_current_before_zero():
    # 阻尼（motion=0）下板端内部仍保存"阻尼前位姿"目标。切回 POSITION 的瞬间
    # 板端会追踪该陈旧目标 → 臂抽回旧位姿。home 必须在 set_motion_mode(1) 之后、
    # set_all_joints_zero 之前，先用实测关节角重写板端目标（move_arm_js），
    # 让 POSITION 进入即保持当前位置，再下发归零。
    q18 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
           1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7,
           0.0, 0.0, 0.0, 0.0]
    robot = _FakeRobot(powered=True, motion_mode=0)
    node = _make_node(robot=robot, q18=q18)
    _seed_cache(node)
    res = _FakeRes()
    node._srv_home(None, res)
    assert res.success is True
    # 顺序：先切 POSITION → 用实测位姿重写目标 → 再归零。
    kinds = [c[0] for c in robot.calls]
    i_mode = kinds.index("set_motion_mode")
    i_seed = kinds.index("move_arm_js")
    i_zero = kinds.index("set_all_joints_zero")
    assert i_mode < i_seed < i_zero
    seed = robot.calls[i_seed]
    assert seed[1] == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)  # left = q[0:7]
    assert seed[2] == (1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7)  # right = q[7:14]
    assert _cache_empty(node)


def test_position_after_damping_seeds_target_from_current():
    # 「位置保持」语义 = 在当前位置保持。阻尼切回 POSITION 时若只发
    # set_motion_mode(1)，板端会追踪内部保存的旧位姿；必须随后用实测关节角
    # 重写目标，才能真正保持在被拖拽后的当前位置。
    q18 = [0.3] * 18
    robot = _FakeRobot(powered=True, motion_mode=0)
    node = _make_node(robot=robot, q18=q18)
    res = _FakeRes()
    node._srv_position(None, res)
    assert res.success is True
    assert robot._motion_mode == 1
    assert ("set_motion_mode", 1) in robot.calls
    assert any(c[0] == "move_arm_js" for c in robot.calls)
    seed = [c for c in robot.calls if c[0] == "move_arm_js"][0]
    assert seed[1] == (0.3,) * 7
    assert seed[2] == (0.3,) * 7


def test_position_in_position_mode_still_seeds_target():
    # 已在 POSITION 再点「位置保持」也应重写目标为当前实测位姿（幂等、无害，
    # 且能顺带把任何陈旧目标校正回当前位置）。
    robot = _FakeRobot(powered=True, motion_mode=1)
    node = _make_node(robot=robot, q18=[0.0] * 18)
    res = _FakeRes()
    node._srv_position(None, res)
    assert res.success is True
    assert any(c[0] == "move_arm_js" for c in robot.calls)




# --- 控制定时器：单臂预设缺侧不补零（no-right-arm 预设点工作位右臂抽一下的根因）----

def _ctrl_node(left=None, right=None):
    """构造可直接调 _on_control_timer 的节点：左/右缓存可选注入（新鲜）。"""
    robot = _FakeRobot()
    node = _make_node(robot=robot)
    node.command_timeout_s = 1.5
    now = time.monotonic()
    with node._lock:
        if left is not None:
            node._left_cmd = list(left)
            node._left_cmd_t = now
        if right is not None:
            node._right_cmd = list(right)
            node._right_cmd_t = now
    node._send_head = lambda: None
    node._send_grippers = lambda: None
    return node, robot


def test_control_timer_left_only_does_not_zero_right():
    # 单臂预设（no right arm）：只左臂有新鲜指令时，右臂**不得**被补零下发
    # （否则停在任何位姿的实体右臂会被 100Hz 零目标拽向零位——症状：工作位/
    # HOME/遥操一发流右臂抽一下）。缺席侧不命令 = 板端位置保持原位。
    node, robot = _ctrl_node(left=[0.3] * 7)
    node._on_control_timer()
    stp = [c for c in robot.calls if c[0] == "set_target_positions"]
    assert len(stp) == 1
    assert sorted(stp[0][1]) == sorted(LEFT_ARM_IDS)  # 只左臂电机
    assert all(v == 0.3 for v in stp[0][1].values())
    assert not [c for c in robot.calls if c[0] == "move_arm_js"]


def test_control_timer_right_only_sends_right():
    node, robot = _ctrl_node(right=[-0.5] * 7)
    node._on_control_timer()
    stp = [c for c in robot.calls if c[0] == "set_target_positions"]
    assert len(stp) == 1
    assert sorted(stp[0][1]) == sorted(RIGHT_ARM_IDS)
    assert not [c for c in robot.calls if c[0] == "move_arm_js"]


def test_control_timer_both_fresh_still_sends_both_together():
    # 双臂都新鲜：维持原路径 move_arm_js(左, 右)（一次下发两臂，行为不变）。
    node, robot = _ctrl_node(left=[0.3] * 7, right=[-0.5] * 7)
    node._on_control_timer()
    mj = [c for c in robot.calls if c[0] == "move_arm_js"]
    assert len(mj) == 1
    assert mj[0][1] == (0.3,) * 7
    assert mj[0][2] == (-0.5,) * 7
    assert not [c for c in robot.calls if c[0] == "set_target_positions"]


def test_control_timer_no_fresh_side_sends_nothing():
    node, robot = _ctrl_node()
    node._on_control_timer()
    assert robot.calls == []


def _run_all():
    tests = [
        test_dry_run_skips,
        test_no_robot_fails,
        test_already_powered_skips_sequence,
        test_already_powered_but_damping_returns_to_position,
        test_enable_confirmed_ok,
        test_power_bit_unconfirmed_but_online_is_success,
        test_offline_board_is_hard_failure,
        test_home_in_damping_restores_position_then_zero_and_clears_cache,
        test_home_in_position_skips_motion_mode_switch,
        test_home_unpowered_fails_with_ready_hint,
        test_ready_clears_cached_target_before_zero,
        test_damping_clears_cached_target,
        test_estop_clears_cached_target,
        test_home_after_damping_seeds_target_from_current_before_zero,
        test_position_after_damping_seeds_target_from_current,
        test_position_in_position_mode_still_seeds_target,
        test_control_timer_left_only_does_not_zero_right,
        test_control_timer_right_only_sends_right,
        test_control_timer_both_fresh_still_sends_both_together,
        test_control_timer_no_fresh_side_sends_nothing,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {t.__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return failed


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
