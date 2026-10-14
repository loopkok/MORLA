#!/usr/bin/env python3
"""MuJoCo 端到端冒烟：astral_mujoco_sim + policy_node(stub) + 本驱动。

真实 ROS 话题全链路（不经进程内 mock）：
  sim  echo /left_arm/joint_states            → policy_node 观测
  本驱动 发 /left_gripper/command (Float64 0.5)  → policy_node 夹爪反馈
  policy_node 发 /left_arm/joint_commands + /left_gripper/command → sim/本驱动接收

场景：
  1. policy   —— 启动后持续收到策略指令（stub 漂移值应变化）→ 断言收到命令且非全零；
  2. pause    —— 夹持：收到的 joint 指令应停止变化（值稳定）；
  3. resume   —— 恢复变化；
  4. playback —— 回放 h5 绝对值轨迹（斜波）→ 断言命令值与回放第 k 帧一致，播完自动 IDLE。

退出码 0=通过；非 0=失败（附 FAIL 行原因）。需在 source 后运行：
  ros2 launch astral_mujoco_sim astral_mujoco_sim.launch.py enable_viewer:=false &
  ros2 run astral_policy_inference policy_node ... &
  /usr/bin/python3 scripts/run_inference_sim_smoke.py --replay <h5> --run-s 12
"""

from __future__ import annotations

import argparse
import json
import time

import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

FPS = 30


def _sensor_qos():
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class E2eDriver(Node):
    def __init__(self, replay: str):
        super().__init__("inference_e2e_driver")
        self.cmd_pub = self.create_publisher(String, "/policy_inference/cmd", 10)
        self._pub_grip = self.create_publisher(Float64, "/left_gripper/command", _sensor_qos())

        self.js_seen: list[np.ndarray] = []
        self.grip_seen: list[float] = []
        self.state = "IDLE"
        # Mirror the policy arbitration gate (level latch): the pump below is the
        # "gripper teleop" writer and must go silent while policy owns the topic.
        self.grip_gate_closed = False
        self._sub_js = self.create_subscription(
            JointState, "/left_arm/joint_commands", self._on_js, _sensor_qos()
        )
        self._sub_grip = self.create_subscription(
            Float64, "/left_gripper/command", self._on_grip, _sensor_qos()
        )
        self._sub_disarm = self.create_subscription(
            Bool, "/teleop/disarm", self._on_disarm, 10
        )
        self.create_subscription(
            String, "/policy_inference/state", self._on_state, 10
        )
        # gripper ratio feedback publisher (mirrors real teleop pinch stream)
        self.create_timer(1.0 / 50.0, self._pump_gripper)
        self.replay = replay

    def _on_js(self, msg: JointState) -> None:
        if msg.position:
            self.js_seen.append(np.asarray(msg.position[:7], dtype=np.float64))

    def _on_grip(self, msg: Float64) -> None:
        self.grip_seen.append(float(msg.data))

    def _on_disarm(self, msg: Bool) -> None:
        self.grip_gate_closed = bool(msg.data)

    def _on_state(self, msg: String) -> None:
        try:
            self.state = json.loads(msg.data).get("state", "?")
        except Exception:
            pass

    def _pump_gripper(self) -> None:
        if self.grip_gate_closed:
            return  # policy owns the gripper topic — do not collide
        m = Float64()
        m.data = 0.5
        self._pub_grip.publish(m)

    def send(self, verb: str) -> None:
        m = String()
        m.data = verb
        self.cmd_pub.publish(m)

    def wait_state(self, want: str, timeout_s: float = 6.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state == want:
                return True
        return self.state == want

    def wait_true(self, fn, what: str, timeout_s: float = 6.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fn():
                return True
        return bool(fn())

    def drain(self, seconds: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def last_js(self) -> np.ndarray | None:
        return self.js_seen[-1] if self.js_seen else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--replay", required=True, help="aligned_data.h5 for playback leg")
    ap.add_argument("--run-s", type=float, default=14.0)
    args = ap.parse_args()

    rclpy.init()
    node = E2eDriver(args.replay)
    fails: list[str] = []

    def check(name: str, cond: bool, detail: str = "") -> None:
        print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")
        if not cond:
            fails.append(name)

    # warm-up: gripper feedback + sim arm state flowing
    node.drain(3.0)
    check("feedback topics seen", len(node.grip_seen) > 0, f"grip_msgs={len(node.grip_seen)}")

    # --- 1. policy leg ------------------------------------------------------
    node.send("policy")
    if not node.wait_state("POLICY", 8.0):
        check("policy enters POLICY", False, f"state={node.state}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    check("policy enters POLICY", True)
    # Once the disarm latch reaches us the pump goes silent, so every message on
    # /left_gripper/command is the policy's own ratio output.
    check(
        "policy closes gripper arbitration gate",
        node.wait_true(lambda: node.grip_gate_closed, "disarm latch", timeout_s=4.0),
    )
    n0 = len(node.js_seen)
    g0 = len(node.grip_seen)
    node.drain(2.0)
    js_vals = node.js_seen[n0:]
    vals = [v for v in js_vals if v is not None]
    distinct = len({tuple(np.round(v, 3)) for v in vals}) if vals else 0
    check(
        "policy drives arm commands",
        len(js_vals) > 3 and distinct > 1,
        f"cmd_msgs={len(js_vals)} distinct={distinct}",
    )
    grip = node.grip_seen[g0:]
    grip_distinct = len({round(v, 3) for v in grip})
    check(
        "policy streams gripper ratio (gate shut → no 0.5 echo)",
        len(grip) > 3 and grip_distinct > 1 and not any(abs(v - 0.5) < 1e-6 for v in grip),
        f"grip_msgs={len(grip)} distinct={grip_distinct}",
    )
    if vals and np.allclose(vals[-1], 0.0, atol=1e-3):
        fails.append("policy commands all-zero (stub drift expected)")

    # --- 2. pause holds -----------------------------------------------------
    node.send("pause")
    check("pause → POLICY_PAUSED", node.wait_state("POLICY_PAUSED"))
    node.drain(1.0)
    a = node.last_js()
    node.drain(1.2)
    b = node.last_js()
    stable = a is not None and b is not None and np.allclose(a, b, atol=1e-4)
    check("pause holds joint target", stable,
          f"Δ={0 if stable else np.max(np.abs(a - b)) if a is not None and b is not None else float('nan'):.5f}")

    # --- 3. resume -----------------------------------------------------------
    node.send("resume")
    check("resume → POLICY", node.wait_state("POLICY"))
    base = len(node.js_seen)
    node.drain(3.0)
    post = node.js_seen[base:]
    post_vals = [v for v in post if v is not None]
    post_distinct = len({tuple(np.round(v, 3)) for v in post_vals}) if post_vals else 0
    check(
        "resume replans (keeps commanding new targets)",
        len(post_vals) > 3 and post_distinct > 1,
        f"new_msgs={len(post_vals)} distinct={post_distinct}",
    )

    # --- 4. playback leg -----------------------------------------------------
    node.send("stop")
    check("stop → IDLE", node.wait_state("IDLE"))
    # stop re-opens the arbitration gate (latched disarm=False), so the gripper
    # "teleop" feedback pump must be publishing again before the next leg.
    ok_open = node.wait_true(lambda: not node.grip_gate_closed, "gate open", timeout_s=4.0)
    check("gate re-opens at IDLE (gripper teleop resumes)", ok_open)
    g_mark = len(node.grip_seen)
    node.drain(0.6)
    resumed = any(abs(v - 0.5) < 1e-6 for v in node.grip_seen[g_mark:])
    check("feedback pump writing again after policy stop", resumed)

    with h5py.File(args.replay, "r") as fh:
        acts = np.asarray(fh["action"], dtype=np.float64)
    final_row = acts[-1]

    node.send(f"playback:{args.replay}")
    if not node.wait_state("PLAYBACK", 8.0):
        check("playback enters PLAYBACK", False, f"state={node.state}")
    else:
        check("playback enters PLAYBACK", True)
        base = len(node.js_seen)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and node.state != "IDLE":
            node.drain(0.2)
        check("playback auto-returns IDLE after hold", node.state == "IDLE",
              f"final_state={node.state}")
        play = node.js_seen[base:]
        if play:
            ramp0 = [v[0] for v in play]
            last = play[-1]
            # last held command == last recorded row; saw the ramp start near 0
            check(
                "playback replays recorded trajectory",
                min(ramp0) <= 0.06
                and abs(last[0] - float(final_row[0])) < 0.05
                and abs(last[1] - float(final_row[1])) < 0.05,
                f"msgs={len(play)} first0={min(ramp0):.3f} last0={last[0]:.3f}"
                f" expect0={float(final_row[0]):.3f}",
            )
        else:
            check("playback replays recorded trajectory", False, "no commands seen")

    print(f"RESULT {'ALL PASS' if not fails else 'FAILURES: ' + ','.join(fails)}")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if not fails else 1)


if __name__ == "__main__":
    main()
