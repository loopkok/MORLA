#!/usr/bin/env python3
"""Functional tests for the policy node (in-process, no real robot/backend).

Drives PolicyNode state machine + control loop with a *stub* backend and
synthetic joint/ratio/image feedback; stands up a real Trigger service to
exercise the HUMAN takeover path (effect → commit ordering).
"""

import json
import os
import tempfile
import threading
import time
import unittest

import h5py
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from morla_data_collect.schema import CollectSchema
from morla_policy_inference.node import PolicyNode
from morla_policy_inference.image_codec import encode_jpeg

FPS = 100

def params(**kw):
    p = dict(
        arms=["left"],
        end_effector_left="gripper",
        end_effector_right="none",
        include_waist=False,
        include_head=False,
        cameras=["base"],
        camera_map='{"cam0": "base"}',
        dataset_fps=FPS,
        control_interp=1,
        backend_type="stub",
        ownership_ack_required=False,
        engine_mode="queue_sync",
        action_chunk=8,
        obs_timeout_s=5.0,
        max_joint_vel=0.0,
        image_required=False,
        teleop_reanchor_services=["/pi_test/reanchor"],
        replay_hold_s=0.0,
        cmd_topic="/pi_test/cmd",
        state_topic="/pi_test/state",
        task_topic="/pi_test/task",
    )
    p.update(kw)
    return [Parameter(k, value=v) for k, v in p.items()]


def make_h5(frames=30, dim=8, fps=FPS):
    d = tempfile.mkdtemp()
    path = os.path.join(d, "aligned_data.h5")
    acts = np.zeros((frames, dim), dtype=np.float64)
    acts[:, 0] = np.arange(frames) / fps  # small smooth drift
    with h5py.File(path, "w") as f:
        f.create_dataset("action", data=acts)
        f.attrs["fps"] = fps
    return path


def make_h5_with_schema(frames=12, fps=FPS):
    """Recorded on a *right*-arm robot → same total dim (7+1) as our left-arm
    robot but a different state layout, so the name check must refuse it."""
    d = tempfile.mkdtemp()
    path = os.path.join(d, "aligned_data.h5")
    acts = np.zeros((frames, 8), dtype=np.float64)
    with h5py.File(path, "w") as f:
        f.create_dataset("action", data=acts)
        schema = CollectSchema(
            arms=["right"],
            end_effector_left="none",
            end_effector_right="gripper",
            include_waist=False,
            include_head=False,
            cameras=["base"],
            dataset_fps=fps,
        )
        f.attrs["fps"] = fps
        f.attrs["schema"] = schema.to_json()
    return path


def feed_state(node, q=None):
    js = JointState()
    js.position = list(np.zeros(7) if q is None else q)
    node._on_joints(js, "left_arm_state", 7)
    node._on_ratio(Float64(data=0.5), "left_gripper_ratio")


class TriggerServer(Node):
    def __init__(self):
        super().__init__("pi_test_trigger")
        self.srv = self.create_service(
            Trigger, "/pi_test/prepare_reanchor", self._cb
        )
        self.armed = 0

    def _cb(self, req, resp):
        self.armed += 1
        resp.success = True
        resp.message = json.dumps({"token": "test-token"})
        return resp


class RejectTriggerServer(Node):
    """Re-anchor service that always refuses (e.g. VR not present)."""

    def __init__(self):
        super().__init__("pi_test_reject_trigger")
        self.srv = self.create_service(
            Trigger, "/pi_test/prepare_reanchor", self._cb
        )
        self.calls = 0

    def _cb(self, req, resp):
        self.calls += 1
        resp.success = False
        resp.message = "VR not tracking"
        return resp


class PolicyNodeFlowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = PolicyNode(parameter_overrides=params())

    def tearDown(self):
        self._stop_server()
        self._stop_node_spin()
        if getattr(self, "node", None) is not None and self.node.context.ok():
            self.node.destroy_node()
        self.node = None

    def _start_node_spin(self):
        self._node_exec = SingleThreadedExecutor()
        self._node_exec.add_node(self.node)
        self._node_stop = threading.Event()
        self._node_thread = threading.Thread(
            target=self._spin_node, daemon=True
        )
        self._node_thread.start()
        time.sleep(0.1)

    def _spin_node(self):
        while not self._node_stop.is_set():
            self._node_exec.spin_once(timeout_sec=0.05)

    def _stop_node_spin(self):
        if not hasattr(self, "_node_exec") or self._node_exec is None:
            return
        self._node_stop.set()
        self._node_thread.join(timeout=2.0)
        self._node_exec.shutdown()
        self._node_exec = None

    def _start_server(self):
        self.server = TriggerServer()
        self._srv_exec = SingleThreadedExecutor()
        self._srv_exec.add_node(self.server)
        self._srv_stop = threading.Event()
        self._server_thread = threading.Thread(
            target=self._spin_server, daemon=True
        )
        self._server_thread.start()
        time.sleep(0.5)

    def _spin_server(self):
        while not self._srv_stop.is_set():
            self._srv_exec.spin_once(timeout_sec=0.1)

    def _stop_server(self):
        srv = getattr(self, "server", None)
        if srv is None:
            return
        if hasattr(self, "_server_thread"):
            self._srv_stop.set()
            self._server_thread.join(timeout=2.0)
        self._srv_exec.remove_node(srv)
        self._srv_exec.shutdown()
        srv.destroy_node()
        self.server = None

    def _cmd(self, text):
        self.node._handle_cmd(text)
        deadline = time.monotonic()+3
        while self.node._policy_start_pending is not None and time.monotonic()<deadline:
            for key in self.node._stamps:
                self.node._stamps[key] = time.monotonic()
            self.node._tick()
            time.sleep(.001)

    def _wait_for(self, fn, what, timeout=3.0):
        """Poll until fn() is truthy (spin helper keeps ROS callbacks alive)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if fn():
                return True
            time.sleep(0.02)
        self.fail(f"timed out waiting for {what}")

    def test_blocked_without_obs(self):
        self._cmd("policy")
        self.assertEqual(self.node.controller.state, "IDLE")
        self.assertIsNone(self.node._engine)

    def test_mute_cameras_omits_image_and_skips_missing(self):
        """mute_cameras：被静音相机从请求省略（serve 补零且 mask=False），
        base 原样保留，且 image_required 不再把缺失腕部图当作阻断条件。"""
        node = PolicyNode(parameter_overrides=params(
            cameras=["base", "left_wrist"],
            camera_map='{"base_0_rgb":"base",'
                       '"left_wrist_0_rgb":"left_wrist"}',
            mute_cameras='["left_wrist"]',
            image_required=True,
        ))
        self.node.destroy_node()
        self.node = node
        # 喂关节状态（否则 state 组装失败，与图像无关）
        feed_state(node)
        # 只喂 base，不喂 left_wrist：被 mute 的腕部相机不应进入 images_missing。
        base = np.full((224, 224, 3), 137, dtype=np.uint8)
        node._images["base"] = base
        node._image_stamps["base"] = time.monotonic()
        state, missing = node._state_ok()
        self.assertIsNotNone(state)
        self.assertFalse(any(m.startswith("image:") for m in missing),
                         f"muted camera leaked into missing: {missing}")
        # _obs_batch 只保留 base；RemoteBackend 会省略 left_wrist 传输键。
        obs = node._obs_batch(np.zeros(8))
        self.assertIn("base", obs.images)
        self.assertTrue(np.array_equal(obs.images["base"], base))
        self.assertNotIn("left_wrist", obs.images)

    def test_stop_cancels_slow_first_plan(self):
        ready = threading.Event()
        retired = threading.Event()
        class SlowEngine:
            enabled = True
            def feed_obs(self, obs): pass
            def start(self): ready.wait(2.0)
            def set_enabled(self, value): self.enabled = value
            def stop(self): retired.set()
        candidate = SlowEngine()
        feed_state(self.node)
        self.node._new_engine = lambda: candidate
        self.node._handle_cmd('policy')
        self.node._tick()
        self.assertIsNotNone(self.node._policy_start_pending)
        self.node._on_cmd(String(data='stop'))
        begin = time.monotonic()
        self.node._tick()
        self.assertLess(time.monotonic()-begin, .2)
        self.assertEqual(self.node.controller.state, 'IDLE')
        ready.set()
        self.assertTrue(retired.wait(1.0))
        self.assertIsNone(self.node._engine)

    def test_missing_ownership_ack_never_emits(self):
        self.node.set_parameters([Parameter('ownership_ack_required', value=True)])
        emitted = []
        self.node._publish_targets = lambda targets: emitted.extend(targets)
        feed_state(self.node)
        self.node._handle_cmd('policy')
        self.node._tick()
        self.assertEqual(emitted, [])
        self.assertIsNone(self.node._engine)
        self.node._policy_start_pending['deadline'] = time.monotonic()-1
        self.node._tick()
        self.assertEqual(self.node.controller.state, 'IDLE')

    def test_only_matching_writer_acks_release_first_plan(self):
        node = self.node
        node.set_parameters([Parameter('ownership_ack_required', value=True)])
        feed_state(node)
        node._handle_cmd('policy')
        node._on_ownership_ack(String(data=json.dumps({'writer': 'left_arm', 'transition_id': 'old'})))
        node._tick()
        self.assertEqual(node._policy_start_pending['phase'], 'ack')
        for writer in ('left_arm', 'left_gripper'):
            node._on_ownership_ack(String(data=json.dumps({'writer': writer, 'transition_id': node._transition_id})))
        deadline = time.monotonic()+1
        while node._policy_start_pending is not None and time.monotonic()<deadline:
            feed_state(node)
            node._tick()
            time.sleep(.001)
        self.assertIsNotNone(node._engine)
        self.assertIsNone(node._policy_start_pending)

    def test_prepare_request_exception_leaves_safe_pause(self):
        self._cmd('policy') if self.node._values else None
        if self.node._engine is None:
            feed_state(self.node)
            self._cmd('policy')
        from types import SimpleNamespace
        def reject(request): raise RuntimeError('send failed')
        self.node._reanchor_clients = {'bad': SimpleNamespace(call_async=reject)}
        self.node._handle_cmd('takeover')
        self.assertIsNone(self.node._takeover_pending)
        self.assertEqual(self.node.controller.state, 'POLICY_PAUSED')
        self.assertFalse(self.node._engine.enabled)

    def test_policy_runs_ticks_and_stops(self):
        feed_state(self.node)
        self._cmd("policy")
        self.assertEqual(self.node.controller.state, "POLICY")
        self.assertIsNotNone(self.node._engine)
        feed_state(self.node)
        self.node._policy_tick(1.0 / FPS)
        self.assertIsNotNone(self.node._seg_cur)
        self.assertEqual(self.node._seg_cur.shape, (8,))
        self.assertTrue(self.node._last_cmds)
        self._cmd("pause")
        self.assertEqual(self.node.controller.state, "POLICY_PAUSED")
        feed_state(self.node)
        self._cmd("resume")
        self.assertEqual(self.node.controller.state, "POLICY")
        self.assertIsNotNone(self.node._engine)  # rebuilt with a fresh chunk
        feed_state(self.node)
        self.node._policy_tick(1.0 / FPS)
        self.assertIsNotNone(self.node._seg_cur)
        self._cmd("stop")
        self.assertEqual(self.node.controller.state, "IDLE")
        self.assertIsNone(self.node._engine)

    def test_policy_entry_seeds_gripper_from_driver_echo(self):
        """首次进 POLICY 且 /left_gripper/command 从未出现：用 driver 的
        /{side}_gripper/joint_states（last-commanded rad 回显）线性换算
        ratio 种子（open=0.8/closed=0.0 → rad 0.4 = ratio 0.5）。"""
        js = JointState()
        js.position = [0.4]
        self.node._on_gripper_rad(js, "left")
        arm = JointState()
        arm.position = list(np.zeros(7))
        self.node._on_joints(arm, "left_arm_state", 7)
        state, missing = self.node._state_ok()
        self.assertIsNotNone(state)
        self.assertNotIn("left_gripper_ratio", missing)
        self.assertAlmostEqual(float(state[7]), 0.5, places=6)
        # 完整进 POLICY（stub 后端）不再被夹爪缺失挡住
        self.node._handle_cmd("policy")
        self.assertEqual(self.node.controller.state, "POLICY")

    def test_policy_entry_blocked_without_any_gripper_source(self):
        """无 ratio、无 driver 回显 → 仍拒绝进 POLICY（原保护不回退）。"""
        arm = JointState()
        arm.position = list(np.zeros(7))
        self.node._on_joints(arm, "left_arm_state", 7)
        state, missing = self.node._state_ok()
        self.assertIsNone(state)
        self.assertIn("left_gripper_ratio", missing)
        self.node._handle_cmd("policy")
        self.assertEqual(self.node.controller.state, "IDLE")  # reverted

    def test_playback_runs_to_idle(self):
        h5 = make_h5(12)
        self.node.destroy_node()
        self.node = PolicyNode(parameter_overrides=params(replay_source=h5))
        feed_state(self.node)
        self._cmd("playback")
        self.assertEqual(self.node.controller.state, "PLAYBACK")
        self.assertIsNotNone(self.node._playback)
        for _ in range(20):
            self.node._playback_tick(1.0 / FPS)
            if self.node.controller.state == "IDLE":
                break
        self.assertEqual(self.node.controller.state, "IDLE")
        # auto-stop on episode end clears the session and last targets
        self.assertIsNone(self.node._playback)

    def test_takeover_failure_keeps_policy(self):
        feed_state(self.node)
        self._cmd("policy")
        self._cmd("takeover")  # no server running -> reanchor not ready
        self.assertEqual(self.node.controller.state, "POLICY")
        self.assertIsNotNone(self.node._engine)
        self._cmd("stop")

    def test_takeover_and_release_roundtrip(self):
        feed_state(self.node)
        self._cmd("policy")
        self.node.set_parameters([Parameter('ownership_ack_required', value=True)])
        self._start_server()
        self._start_node_spin()  # node executor completes the reanchor futures
        self.node._handle_cmd("takeover")
        pending = self.node._takeover_pending
        self.assertIsNotNone(pending)
        self._wait_for(lambda: all(f.done() for f in pending.values()),
                       "prepare_reanchor responses")
        self.assertEqual(self.node.controller.state, "POLICY")
        transition = self.node._transition_id
        # Re-anchor success alone is insufficient: stale/wrong or partial
        # writer acknowledgements must keep the policy frozen and uncommitted.
        self.node._on_ownership_ack(String(data=json.dumps({
            'writer': 'left_arm', 'transition_id': 'old-transition',
        })))
        self.node._on_ownership_ack(String(data=json.dumps({
            'writer': 'left_arm', 'transition_id': transition,
        })))
        self.assertEqual(self.node.controller.state, "POLICY")
        self.assertIsNotNone(self.node._takeover_pending)
        self.node._on_ownership_ack(String(data=json.dumps({
            'writer': 'left_gripper', 'transition_id': transition,
        })))
        self._wait_for(
            lambda: self.node.controller.state == "HUMAN",
            "HUMAN after takeover",
        )
        self.assertIsNone(self.node._engine)
        self.node.set_parameters([Parameter('ownership_ack_required', value=False)])
        feed_state(self.node)
        self._cmd("release")
        self._wait_for(
            lambda: self.node.controller.state == "POLICY"
            and self.node._engine is not None,
            "POLICY rebuilt after release",
        )
        self._cmd("stop")

    def test_playback_takeover_and_release_roundtrip(self):
        """A recorded real-robot episode can be interrupted by HITL and resumed."""
        h5 = make_h5(200)
        self.node.destroy_node()
        self.node = PolicyNode(parameter_overrides=params(replay_source=h5))
        feed_state(self.node)
        self._cmd("playback")
        self.assertEqual(self.node.controller.state, "PLAYBACK")
        for _ in range(5):
            self.node._playback_tick(1.0 / FPS)
        session = self.node._playback
        self.assertIsNotNone(session)
        self.assertGreater(session.idx, 0)

        self._start_server()
        self._start_node_spin()
        feed_state(self.node)
        self.node._handle_cmd("takeover")
        self._wait_for(
            lambda: self.node.controller.state == "HUMAN",
            "HUMAN takeover from playback",
        )
        self.assertIs(self.node._playback, session)
        human_idx = session.idx
        time.sleep(0.1)
        self.assertEqual(session.idx, human_idx)  # playback is frozen under HUMAN

        feed_state(self.node)
        self.node._handle_cmd("release")
        self._wait_for(
            lambda: self.node.controller.state == "PLAYBACK"
            and self.node._policy_start_pending is None,
            "playback resumed after HITL release",
        )
        self.assertIs(self.node._playback, session)
        self.assertGreaterEqual(session.idx, human_idx)
        self._cmd("stop")

    def test_pause_resume_playback_reanchors_session(self):
        h5 = make_h5(20)
        self.node.destroy_node()
        self.node = PolicyNode(parameter_overrides=params(replay_source=h5))
        feed_state(self.node)
        self._cmd("playback")
        for _ in range(5):
            self.node._playback_tick(1.0 / FPS)
        self.assertEqual(self.node._playback.idx, 5)
        self._cmd("pause")
        self.assertEqual(self.node.controller.state, "PLAYBACK_PAUSED")
        feed_state(self.node)  # robot was moved by hand while paused
        self._cmd("resume")
        self.assertEqual(self.node.controller.state, "PLAYBACK")
        self.assertEqual(self.node._playback.idx, 5)  # session preserved
        # resume re-anchored the offset, so the next target continues from here
        self.node._playback_tick(1.0 / FPS)
        self.assertTrue(self.node._last_cmds)
        self._cmd("stop")

    def test_invalid_transition_ignored(self):
        self._cmd("takeover")  # IDLE -> takeover is invalid
        self.assertEqual(self.node.controller.state, "IDLE")

    # --- adversarial-review regressions --------------------------------------

    def test_obs_loss_midrun_autopauses(self):
        """F6: losing joint states mid-POLICY must auto-pause, not keep
        re-inferring/emitting a blind plan."""
        node = PolicyNode(parameter_overrides=params(obs_stale_stop_s=0.05))
        self.node.destroy_node()
        self.node = node
        feed_state(node)
        self._cmd("policy")
        self.assertEqual(node.controller.state, "POLICY")
        feed_state(node)
        node._policy_tick(1.0 / FPS)
        self.assertIsNotNone(node._last_cmds)
        # make the joint-state stamp stale, then keep ticking past the grace
        for key in node._stamps:
            node._stamps[key] -= 10.0
        deadline = time.monotonic() + 2.0
        while node.controller.state != "POLICY_PAUSED" and time.monotonic() < deadline:
            node._policy_tick(1.0 / FPS)
            time.sleep(0.02)
        self.assertEqual(node.controller.state, "POLICY_PAUSED")
        self.assertFalse(node._engine.enabled)

    def test_stop_reopens_arbitration_gate(self):
        """F2: stop → IDLE must publish disarm=False (level latch) so the
        gripper teleop gate reopens; entering POLICY publishes True first."""
        published = []
        node = self.node
        node._teleop_disarm.publish = lambda m: published.append(bool(m.data))
        feed_state(node)
        self._cmd("policy")
        self.assertEqual(published[-1], True)
        node._handle_cmd("stop")
        self.assertEqual(published[-1], False)
        self.assertEqual(node.controller.state, "IDLE")

    def test_takeover_success_opens_gate_abort_recloses(self):
        """F1/F8: successful takeover opens the gate (False); a failed re-anchor
        (rejected service) aborts and re-disarms (True) so a partially-armed
        teleop node cannot race the policy."""
        published = []
        node = self.node
        node._teleop_disarm.publish = lambda m: published.append(bool(m.data))
        feed_state(node)
        self._cmd("policy")
        published.clear()
        # failing service first
        self.server = RejectTriggerServer()
        self._srv_exec = SingleThreadedExecutor()
        self._srv_exec.add_node(self.server)
        self._srv_stop = threading.Event()
        self._server_thread = threading.Thread(target=self._spin_server, daemon=True)
        self._server_thread.start()
        time.sleep(0.4)
        self._start_node_spin()
        node._handle_cmd("takeover")
        self._wait_for(
            lambda: node._takeover_pending is None, "takeover abort resolution"
        )
        self.assertEqual(node.controller.state, "POLICY_PAUSED")  # old plan cannot resume after timeout
        self.assertIsNotNone(node._engine)
        self.assertEqual(published[-1], True)  # re-disarmed (F8)
        # success now: switch server to accepting
        self._stop_server()
        self._start_server()
        feed_state(node)
        node._handle_cmd("takeover")
        self._wait_for(
            lambda: node.controller.state == "HUMAN", "HUMAN takeover success"
        )
        self.assertEqual(published[-1], False)  # gate opened on commit

    def test_playback_refuses_schema_layout_mismatch(self):
        """F11: same action_dim but a different recorded state layout must be
        refused up front (no FSM change, no silent wrong replay)."""
        h5 = make_h5_with_schema()
        node = self.node
        node._handle_cmd(f"playback:{h5}")
        self.assertEqual(node.controller.state, "IDLE")
        self.assertIsNone(node._playback)

    def test_hold_end_reset_between_playback_runs(self):
        """F10: a stale _hold_end from a previous playback must not skip the
        end-of-episode hold of the next run (regression: run #2 auto-stopped
        the moment its last frame was sent)."""
        h5 = make_h5(8)
        node = PolicyNode(parameter_overrides=params(
            replay_source=h5, replay_hold_s=0.3,
        ))
        self.node.destroy_node()
        self.node = node
        feed_state(node)

        def run_to_hold_or_idle():
            for _ in range(40):
                if node.controller.state != "PLAYBACK":
                    return
                node._playback_tick(1.0 / FPS)

        # run #1: play out, hold 0.3s, auto-stop to IDLE (clears _hold_end)
        self._cmd("playback")
        run_to_hold_or_idle()
        self.assertEqual(node.controller.state, "PLAYBACK")
        self.assertIsNotNone(node._hold_end)
        time.sleep(0.35)
        for _ in range(5):
            if node.controller.state == "IDLE":
                break
            node._playback_tick(1.0 / FPS)
        self.assertEqual(node.controller.state, "IDLE")
        self.assertIsNone(node._hold_end)
        # run #2 must hold again at its end instead of stopping instantly
        feed_state(node)
        self._cmd("playback")
        self.assertEqual(node.controller.state, "PLAYBACK")
        self.assertIsNone(node._hold_end)
        run_to_hold_or_idle()
        self.assertEqual(node.controller.state, "PLAYBACK")
        self.assertIsNotNone(node._hold_end)
        node._handle_cmd("stop")

    def test_metrics_log_writes_file(self):
        """metrics_log_file 非空时，每次 _publish_state 追加一行带 t 的 state JSON。"""
        self._stop_server()
        self._stop_node_spin()
        if self.node.context.ok():
            self.node.destroy_node()
        d = tempfile.mkdtemp()
        path = os.path.join(d, "pi_metrics.jsonl")
        self.node = PolicyNode(parameter_overrides=params(metrics_log_file=path))
        feed_state(self.node)
        self.node._publish_state()
        self.node._publish_state()
        with open(path) as f:
            lines = f.read().splitlines()
        self.assertEqual(len(lines), 2)  # 每次发布一行
        rec = json.loads(lines[0])
        self.assertIn("t", rec)
        self.assertIn("state", rec)
        self.assertIn("latency_ms", rec)
        self.assertIn("engine", rec)

    def test_joint_stream_log_writes_commanded_values(self):
        """joint_stream_log_file 非空时，每次实际下发（_policy_tick）把带时间戳的
        各话题指令值记录成 JSON 行——排障卡顿用的指令流日志。"""
        self._stop_server()
        self._stop_node_spin()
        if self.node.context.ok():
            self.node.destroy_node()
        d = tempfile.mkdtemp()
        path = os.path.join(d, "pi_cmds.jsonl")
        self.node = PolicyNode(parameter_overrides=params(joint_stream_log_file=path))
        feed_state(self.node)  # 先给观测，policy 门才放行（与现有 POLICY 用例同序）
        self._cmd("policy")
        self.assertEqual(self.node.controller.state, "POLICY")
        self.node._policy_tick(1.0 / FPS)
        self.node._policy_tick(1.0 / FPS)
        self._cmd("stop")
        with open(path) as f:
            lines = [l for l in f.read().splitlines() if l.strip()]
        self.assertGreaterEqual(len(lines), 1, "下发后指令流文件应有内容")
        rec = json.loads(lines[0])
        self.assertIn("t", rec)
        self.assertIn("/left_arm/joint_commands", rec, "应记录臂指令话题")
        self.assertEqual(len(rec["/left_arm/joint_commands"]), 7)
        self.assertTrue(all(isinstance(v, float) for v in rec["/left_arm/joint_commands"]))
        self.assertIn("raw_targets", rec, "须可比对模型原始目标与安全后指令")
        self.assertIn("/left_arm/joint_commands", rec["raw_targets"])
        self.assertEqual(len(rec["raw_targets"]["/left_arm/joint_commands"]), 7)
        self.assertIn("state", rec, "应同时记录观测 state（供绘图对比）")
        self.assertEqual(len(rec["state"]), 8)
        self.assertEqual(rec["send_kind"], "new_target")

    def test_control_diagnostics_records_stale_state_and_resend_reason(self):
        """逐 tick 日志必须能区分 state gate 拒绝和 resend_last hold。"""
        self._stop_server()
        self._stop_node_spin()
        if self.node.context.ok():
            self.node.destroy_node()
        d = tempfile.mkdtemp()
        control_path = os.path.join(d, "pi_control.jsonl")
        cmds_path = os.path.join(d, "pi_cmds.jsonl")
        self.node = PolicyNode(parameter_overrides=params(
            control_diagnostics_log_file=control_path,
            joint_stream_log_file=cmds_path,
            obs_timeout_s=0.01,
        ))
        feed_state(self.node)
        self._cmd("policy")
        self.assertEqual(self.node.controller.state, "POLICY")

        # 先产生一个真实 target，确保后续 hold 有 last_cmd 可重发。
        self.node._tick()
        # 强制关节反馈过期：下一 tick 应被 _state_ok gate 静默拒绝，但诊断日志有证据。
        arm_keys = [
            key for key, src in self.node._layout.sources.items()
            if src.kind != "ratio"
        ]
        for key in arm_keys:
            self.node._stamps[key] = time.monotonic() - 0.1
        self.node._tick()

        # 恢复观测并暂停：paused tick 必须记录 resend_last 及明确原因。
        feed_state(self.node)
        self._cmd("pause")
        self.node._tick()

        with open(control_path) as f:
            control = [json.loads(line) for line in f if line.strip()]
        stale = [r for r in control if r["action"] == "state_rejected"]
        self.assertEqual(len(stale), 1)
        self.assertIn("left_arm_state", stale[0]["missing"])
        obs = stale[0]["observation"]
        self.assertEqual(obs["source_status"]["left_arm_state"], "stale")
        self.assertGreater(obs["joint_age_max_ms"], obs["obs_timeout_ms"])
        self.assertIn("tick_interval_ms", stale[0])
        self.assertIn("callback_ms", stale[0])

        holds = [r for r in control if r["action"] == "resend_last"]
        self.assertTrue(holds)
        self.assertEqual(holds[-1]["hold_reason"], "policy_paused")
        with open(cmds_path) as f:
            commands = [json.loads(line) for line in f if line.strip()]
        resend = [r for r in commands if r["send_kind"] == "resend_last"]
        self.assertTrue(resend)
        self.assertEqual(resend[-1]["hold_reason"], "policy_paused")
        self.assertIn("control_seq", resend[-1])

    def test_camera_diagnostics_combines_policy_rx_and_streamer_stats(self):
        """log_dir 的第四条流应自动包含逐帧接收和上游 capture/tap 统计。"""
        self._stop_server()
        self._stop_node_spin()
        if self.node.context.ok():
            self.node.destroy_node()
        d = tempfile.mkdtemp()
        path = os.path.join(d, "camera_diagnostics.jsonl")
        self.node = PolicyNode(parameter_overrides=params(
            camera_diagnostics_log_file=path,
            camera_image_size=8,
        ))
        msg = CompressedImage()
        msg.data = encode_jpeg(np.zeros((8, 8, 3), dtype=np.uint8))
        self.node._on_image(msg, "base")
        self.node._on_image(msg, "base")
        self.node._on_streamer_diagnostics(String(data=json.dumps({
            "t": 123.0,
            "stage": "capture",
            "label": "base",
            "fps": 30.0,
        })))
        self.node._camera_diag_queue.join()

        with open(path) as f:
            rows = [json.loads(line) for line in f if line.strip()]
        rx = [r for r in rows if r.get("stage") == "policy_rx"]
        self.assertEqual(len(rx), 2)
        self.assertIsNone(rx[0]["gap_ms"])
        self.assertIsInstance(rx[1]["gap_ms"], float)
        self.assertTrue(rx[0]["decoded"])
        self.assertGreater(rx[0]["payload_bytes"], 0)
        self.assertIn("decode_ms", rx[0])
        upstream = [r for r in rows if r.get("stage") == "capture"]
        self.assertEqual(len(upstream), 1)
        self.assertEqual(upstream[0]["fps"], 30.0)
        self.assertIn("policy_received_t", upstream[0])

    def test_command_callback_isolated_and_traced_in_control_log(self):
        """高频图像不能再与 cmd/tick 共用默认串行 callback group。

        回归 2026-09-22：base 30fps 解码时 left_wrist 仅 2.86fps，连可靠的
        ``policy`` 命令也始终未被 callback 调度。这里锁定每相机独立组、命令
        专用组，以及 cmd_rx -> cmd_execute 的控制日志证据。
        """
        self._stop_server()
        self._stop_node_spin()
        if self.node.context.ok():
            self.node.destroy_node()
        d = tempfile.mkdtemp()
        control_path = os.path.join(d, "pi_control.jsonl")
        self.node = PolicyNode(parameter_overrides=params(
            cameras=["base", "left_wrist"],
            camera_map='{"base_0_rgb":"base","left_wrist_0_rgb":"left_wrist"}',
            control_diagnostics_log_file=control_path,
        ))
        self.assertIs(
            self.node._cmd_sub.callback_group,
            self.node._command_callback_group,
        )
        self.assertIsNot(
            self.node._cmd_sub.callback_group,
            self.node._timer.callback_group,
        )
        self.assertIsNot(
            self.node._image_callback_groups["base"],
            self.node._image_callback_groups["left_wrist"],
        )

        feed_state(self.node)
        self.node._on_cmd(String(data="policy"))
        self.node._tick()
        self.assertEqual(self.node.controller.state, "POLICY")
        with open(control_path) as f:
            records = [json.loads(line) for line in f if line.strip()]
        rec = records[-1]
        self.assertIn("cmd_execute", rec["events"])
        self.assertEqual(rec["cmd"], "policy")
        self.assertEqual(rec["cmd_rx_seq"], 1)
        self.assertGreaterEqual(rec["cmd_queue_delay_ms"], 0.0)

    def test_async_prefetch_ahead_reaches_engine(self):
        """async_prefetch_ahead 参数应透传到引擎（控制重规划/融合频率）。"""
        self._stop_server()
        self._stop_node_spin()
        if self.node.context.ok():
            self.node.destroy_node()
        self.node = PolicyNode(parameter_overrides=params(async_prefetch_ahead=10))
        feed_state(self.node)
        self._cmd("policy")
        self.assertEqual(self.node.controller.state, "POLICY")
        self.assertIsNotNone(self.node._engine)
        self.assertEqual(self.node._engine.async_prefetch_ahead, 10)
        self._cmd("stop")


if __name__ == "__main__":
    unittest.main(verbosity=2)
