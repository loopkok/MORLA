#!/usr/bin/env python3
"""Tests for the three action engines (threading included)."""

import threading
import time
import unittest

import numpy as np

from astral_policy_inference.backend import ObsBatch, PolicyError, StubBackend
from astral_policy_inference.engine import ActionEngine, EngineStateError

DIM = 8


def make_engine(mode="queue_sync", **kw):
    backend = StubBackend(action_dim=DIM, camera_map={})
    kwargs = dict(mode=mode, action_dim=DIM, chunk=kw.pop("chunk", 10),
                  autostart=False)
    kwargs.update(kw)
    return ActionEngine(backend, **kwargs), backend


class FailingBackend(StubBackend):
    def infer(self, obs):
        raise PolicyError("server down")


class TestQueueSync(unittest.TestCase):
    def test_plan_trace_contains_snapshot_chunk_and_alignment(self):
        rows = []
        eng, _ = make_engine(
            "queue_sync", chunk=4, plan_trace_hook=rows.append,
        )
        eng.feed_obs(ObsBatch(
            state=np.zeros(DIM, dtype=np.float64), images={}, prompt="pick",
            metadata={"image_age_ms": {"base": 12.5}},
        ))
        eng.set_enabled(True)
        eng.start()
        self.assertEqual(len(rows), 1)
        rec = rows[0]
        self.assertEqual(rec["event"], "plan_installed")
        self.assertEqual(rec["snapshot"]["state"], [0.0] * DIM)
        self.assertEqual(rec["snapshot"]["metadata"]["image_age_ms"]["base"], 12.5)
        self.assertEqual(len(rec["server_chunk"]), 4)
        self.assertEqual(rec["alignment"]["resume_i"], 0)
        self.assertIn("selected_row", rec["alignment"])
        eng.stop()

    def test_ticks_consume_and_refill(self):
        eng, _ = make_engine("queue_sync", chunk=3)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertEqual(eng.stats["plans"], 1)
        rows = [eng.tick() for _ in range(9)]
        self.assertTrue(all(r is not None for r in rows))
        self.assertGreaterEqual(eng.stats["plans"], 3)

    def test_failed_infer_raises_engine_state_error(self):
        eng = ActionEngine(FailingBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=5,
                           autostart=False)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        with self.assertRaises(EngineStateError):
            eng.start()

    def test_disabled_returns_none(self):
        eng, _ = make_engine("queue_sync")
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(False)
        self.assertIsNone(eng.tick())

    def test_malformed_or_empty_chunks_are_rejected_cleanly(self):
        """Bad backend shapes become diagnostic rejection, never index errors."""
        eng, _ = make_engine("queue_sync")
        for bad in (
            np.asarray(1.0),
            np.zeros((1, DIM, 1)),
            np.zeros((0, DIM)),
        ):
            self.assertIsNone(eng._install(bad, 0.0))
            self.assertIsNotNone(eng.last_error)


class TestQueueAsync(unittest.TestCase):
    def test_queue_async_refill_logic(self):
        # Deterministic (no planner thread): exercise exactly what the planner
        # thread runs on refill — swap the chunk at the boundary and resume.
        eng, _ = make_engine("queue_async", chunk=10, autostart=False,
                             async_prefetch_ahead=1)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()  # sync initial plan (no thread with autostart=False)
        self.assertEqual(eng.stats["plans"], 1)
        pops = 0
        while eng.tick() is not None:
            pops += 1
        self.assertEqual(eng.remaining, 0)
        eng._run_plan()  # == what _planner_loop does when not idle
        self.assertGreater(eng.remaining, 0)
        self.assertGreaterEqual(eng.stats["plans"], 2)
        self.assertIsNotNone(eng.tick())
        eng.stop()

    def test_reset_then_refill(self):
        # deterministic: no planner thread in queue_sync
        eng, _ = make_engine("queue_sync", chunk=10)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertEqual(eng.remaining, 4)  # stub chunk is 4 rows
        eng.reset()
        self.assertEqual(eng.remaining, 0)
        self.assertIsNone(eng.last_error)
        row = eng.tick()  # queue_sync refills synchronously
        self.assertIsNotNone(row)
        eng.stop()


class TestRtc(unittest.TestCase):
    def test_rolling_replan_swaps_unexecuted_tail(self):
        # Deterministic: mimic the planner's rolling decision — after the min
        # tail has executed, a fresh plan swaps the tail of the running chunk.
        eng, _ = make_engine("rtc", chunk=10, autostart=False,
                             rtc_replan_interval_s=0.0, rtc_min_tail=2)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertEqual(eng.stats["plans"], 1)
        for _ in range(2):
            self.assertIsNotNone(eng.tick())  # execute the min tail
        self.assertEqual(eng.stats["plans"], 1)
        self.assertGreater(eng.remaining, 0)
        eng._run_plan()  # rolling replan while rows remain
        self.assertGreaterEqual(eng.stats["plans"], 2)
        self.assertGreater(eng.remaining, 0)
        eng.stop()


class TestThreadingSafety(unittest.TestCase):
    def test_engine_stop_joins_thread(self):
        eng, _ = make_engine("queue_async", autostart=True)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.start()
        self.assertIsNotNone(eng._thread)
        alive = True
        eng.stop()
        # thread references are cleared; verify no dangling writer
        self.assertIsNone(eng._thread)

    def test_start_raises_on_no_obs(self):
        eng, _ = make_engine("queue_sync")
        eng.set_enabled(True)
        with self.assertRaises(EngineStateError):
            eng.start()  # _plan_blocking silently no-ops without obs

    def test_queue_async_control_thread_not_starved_by_planner(self):
        """回归：queue_async 空闲时 planner 不得持锁饿死控制线程。

        根因：`_planner_loop` 把 `self._stop.wait(0.005)` 写在 `with self._lock`
        内，空闲时 planner 几乎 100% 持有引擎锁（5ms 等待+立即重获），`tick()`
        在锁上饿死 → 即使瞬时推理也只有 ~15Hz。修复：wait 移出锁外。
        被 plan A（后端返回完整 chunk → planner 进入空闲路径）暴露。
        """
        class FullChunkBackend(StubBackend):
            def infer(self, obs):
                return np.zeros((50, DIM), dtype=np.float64)

        eng = ActionEngine(
            FullChunkBackend(action_dim=DIM, camera_map={}),
            mode="queue_async", action_dim=DIM, chunk=50, policy_fps=30,
            autostart=True,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.start()
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < 0.5:
            if eng.tick() is not None:
                n += 1
        eng.stop()
        # 修复前 ~7 pop/0.5s；修复后 >1000。200 为 10× 余量的宽松阈值。
        self.assertGreater(n, 200, f"control thread starved: {n} pops in 0.5s")

    def test_unexpected_planner_failure_is_traced_and_surfaced_after_tail(self):
        """Unexpected planner bugs must not become an infinite last-command hold.

        The accepted tail remains usable, but once exhausted tick() raises so
        PolicyNode can stop.  The daemon thread also leaves an auditable trace.
        """
        class ExplodingPlannerEngine(ActionEngine):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.plan_calls = 0

            def _run_plan(self):
                self.plan_calls += 1
                if self.plan_calls >= 2:
                    raise RuntimeError("injected install bug")
                return super()._run_plan()

        traces = []
        eng = ExplodingPlannerEngine(
            StubBackend(action_dim=DIM, camera_map={}),
            mode="queue_async", action_dim=DIM, chunk=10,
            autostart=True, async_prefetch_ahead=4,
            plan_trace_hook=traces.append,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()  # first synchronous plan succeeds; background replan explodes

        deadline = time.monotonic() + 1.0
        while eng.last_error is None and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertIn("injected install bug", eng.last_error or "")
        self.assertTrue(any(r.get("event") == "planner_failed" for r in traces))

        # StubBackend returns four rows.  A planner fault must not discard that
        # already accepted tail or interrupt it halfway through.
        for _ in range(4):
            self.assertIsNotNone(eng.tick())
        with self.assertRaisesRegex(EngineStateError, "injected install bug"):
            eng.tick()
        eng.stop()


class TestAbsoluteSemanticsGuard(unittest.TestCase):
    """回归：后端返回 delta 语义（远离当前 state 的小值）时引擎拒绝 chunk。

    后端契约是绝对动作；openpi server 若漏 AbsoluteActions、或错 checkpoint，
    会把 delta 当绝对返回 → 首行≈小 delta，在 |state|>1 的维上 |action-state|≈|state|
    超阈值 → 引擎报错（节点据此安全 stop，不静默跳到错误目标）。
    """

    def _away_state(self):
        st = np.zeros(DIM)
        st[0], st[3] = 1.5, -1.8  # 臂维明显离开零位
        return st

    def test_rejects_delta_like_actions(self):
        class DeltaBackend(StubBackend):
            def infer(self, obs):
                return np.full((4, DIM), 0.03, dtype=np.float64)  # 小 delta

        eng = ActionEngine(DeltaBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=50,
                           policy_fps=30, autostart=False, abs_action_min_scale=0.5)
        eng.feed_obs(ObsBatch(state=self._away_state(), images={}, prompt=""))
        with self.assertRaises(EngineStateError):
            eng.start()  # 首次 plan 的绝对语义检查失败 → engine error

    def test_accepts_absolute_actions(self):
        class AbsBackend(StubBackend):
            def infer(self, obs):
                out = np.zeros((4, DIM))
                out[0] = obs.state  # 绝对 next-state ≈ 当前 state
                return out

        eng = ActionEngine(AbsBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=50,
                           policy_fps=30, autostart=False, abs_action_min_scale=0.5)
        eng.feed_obs(ObsBatch(state=self._away_state(), images={}, prompt=""))
        eng.start()  # 不应 raise
        self.assertIsNotNone(eng._chunk)

    def test_disabled_when_threshold_nonpositive(self):
        class DeltaBackend(StubBackend):
            def infer(self, obs):
                return np.full((4, DIM), 0.03, dtype=np.float64)

        eng = ActionEngine(DeltaBackend(action_dim=DIM, camera_map={}),
                           mode="queue_sync", action_dim=DIM, chunk=50,
                           policy_fps=30, autostart=False, abs_action_min_scale=0.0)
        eng.feed_obs(ObsBatch(state=self._away_state(), images={}, prompt=""))
        eng.start()  # 关闭时不拦截
        self.assertIsNotNone(eng._chunk)


class JumpBackend(StubBackend):
    """第 2 次起每次推理都返回一个全 1 的 (10, dim) chunk（第 1 次全 0）。
    故意制造"换 chunk 时 0→1 的硬跳变"，用于验证时序融合把边界步压下去。"""

    def __init__(self, dim=DIM):
        super().__init__(action_dim=dim, camera_map={})
        self.plans = 0

    def infer(self, obs):
        self.plans += 1
        val = 1.0 if self.plans >= 2 else 0.0
        return np.full((10, self.action_dim), val)


class TestTemporalEnsembling(unittest.TestCase):
    """引擎级 ACT 时序融合：换 chunk 时旧尾段与新头部按权重平均，边界不再硬跳。"""

    def _drive(self, coeff, chunk=10, prefetch=5, ticks=40):
        eng = ActionEngine(
            JumpBackend(), mode="queue_async", action_dim=DIM, chunk=chunk,
            autostart=False, async_prefetch_ahead=prefetch,
            temporal_ensemble_coeff=coeff,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()  # 首块（plans=1, 全 0）
        rows = []
        for _ in range(ticks):
            # 模拟 _planner_loop 判定：还有 >0 行可消费且剩余 <= prefetch → 重规划
            if 0 < eng.remaining <= prefetch:
                eng._run_plan()
            r = eng.tick()
            if r is None:  # 意外耗尽（async 无线程时兜底）
                eng._run_plan()
                r = eng.tick()
            if r is not None:
                rows.append(r)
        eng.stop()
        return np.asarray(rows)

    def test_ensembling_bounds_boundary_step(self):
        raw = self._drive(coeff=0.0)   # 无融合：0→1 硬跳
        ens = self._drive(coeff=0.01)  # 融合：边界步被压
        raw_steps = np.abs(np.diff(raw, axis=0)).max(axis=1)
        ens_steps = np.abs(np.diff(ens, axis=0)).max(axis=1)
        self.assertGreater(raw_steps.max(), 0.9, "无融合时应出现 ~1.0 的硬跳变")
        self.assertLess(ens_steps.max(), 0.9, "融合后边界步必须显著小于硬跳变")
        self.assertLess(ens_steps.max(), raw_steps.max(), "融合必须实际降低最大步长")
        self.assertEqual(ens.shape[1], DIM)
        self.assertTrue(np.isfinite(ens).all())

    def test_ensembling_first_plan_adopts_and_reset_clears(self):
        eng = ActionEngine(
            JumpBackend(), mode="queue_async", action_dim=DIM, chunk=10,
            autostart=False, temporal_ensemble_coeff=0.01,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertIsNotNone(eng._ensembler)
        self.assertIsNotNone(eng._ensembler.counts, "首块后计数应初始化")
        eng.reset()
        self.assertIsNone(eng._ensembler.counts, "reset 必须清空融合计数")
        self.assertEqual(eng.remaining, 0)
        eng.stop()

    def test_ensembling_short_chunk_does_not_crash(self):
        """后端返回行数 < chunk_size（StubBackend 4 行、截断响应）时不得越界。

        对抗性回归：update() 里 r 若只按 chunk_size 与 n 取小，会用 old[i+k]
        (k 到 r-1) 越出旧缓冲长度 → IndexError。r 必须同时受 len(old)-i 约束。
        """
        class ShortBackend(StubBackend):
            def __init__(self, dim=DIM):
                super().__init__(action_dim=dim, camera_map={})

            def infer(self, obs):
                return np.full((4, DIM), 0.5, dtype=np.float64)

        eng = ActionEngine(
            ShortBackend(), mode="queue_async", action_dim=DIM, chunk=10,
            autostart=False, async_prefetch_ahead=2, temporal_ensemble_coeff=0.01,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        for _ in range(30):
            if 0 < eng.remaining <= 2:
                eng._run_plan()  # 剩余 2 行时重规划 → 旧缓冲仅 4 行、已消费 2 行
            r = eng.tick()
            if r is None:
                eng._run_plan()
                r = eng.tick()
            self.assertIsNotNone(r, "短 chunk + 融合不应断流")
        eng.stop()


class TestLockFreeInfer(unittest.TestCase):
    """锁外推理回归：慢推理（模拟远程 100ms+）不得堵控制线程、不得破坏续播对齐。

    修复前 _run_plan 锁跨 infer() 持有 → tick() 被堵整个推理时长（每次重规划一个
    空档 + 滞后 i0 超前跳）；修复后推理在锁外，控制线程照常消费，续播用实测 consumed。
    """

    def test_slow_infer_does_not_block_control_tick(self):
        """慢推理 200ms 期间，tick() 必须立刻返回，不被堵接近推理时长。"""
        class SlowBackend(StubBackend):
            def __init__(self, dim=DIM, delay=0.2):
                super().__init__(action_dim=dim, camera_map={})
                self.delay = delay

            def infer(self, obs):
                time.sleep(self.delay)
                return np.full((10, DIM), 0.1, dtype=np.float64)

        eng = ActionEngine(SlowBackend(delay=0.2), mode="queue_async", action_dim=DIM,
                           chunk=10, autostart=False, async_prefetch_ahead=1)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()  # 首块阻塞 ~200ms
        t = threading.Thread(target=eng._run_plan, daemon=True)
        t.start()
        time.sleep(0.05)  # 确保已进入 infer 的 sleep
        t0 = time.perf_counter()
        row = eng.tick()  # 不应被 200ms 推理堵住
        waited = time.perf_counter() - t0
        t.join(timeout=1.0)
        self.assertIsNotNone(row)
        self.assertLess(waited, 0.1,
                        f"tick 被慢推理堵了 {waited*1000:.0f}ms（修复前锁跨 infer 持有会 ~200ms）")
        eng.stop()

    def test_slow_replan_resumes_at_consumed(self):
        """慢推理期间控制线程消费 consumed 行，重规划后必须从 new[consumed] 续播
        （_i == consumed），而不是从 new[0] 重来或按 latency_ms 估算超前跳。"""
        class RampBackend(StubBackend):
            def __init__(self, dim=DIM, delay=0.15):
                super().__init__(action_dim=dim, camera_map={})
                self.delay = delay
                self.plans = 0

            def infer(self, obs):
                time.sleep(self.delay)
                self.plans += 1
                base = float(1000 * self.plans)
                return (base + np.arange(10, dtype=np.float64))[:, None] * np.ones((1, DIM))

        eng = ActionEngine(RampBackend(delay=0.15), mode="queue_async", action_dim=DIM,
                           chunk=10, autostart=False, async_prefetch_ahead=1)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()  # plan1 = [1000..1009], _i=0
        self.assertIsNotNone(eng.tick())  # 1000
        self.assertIsNotNone(eng.tick())  # 1001 → _i=2
        t = threading.Thread(target=eng._run_plan, daemon=True)
        t.start()
        time.sleep(0.05)  # infer 进行中（sleep 150ms），控制线程继续消费
        self.assertIsNotNone(eng.tick())
        self.assertIsNotNone(eng.tick())  # _i 2→4
        t.join(timeout=1.0)
        consumed = 2  # 推理期间实际消费 = 4 - 2
        self.assertEqual(eng._i, consumed,
                         f"重规划后应从 new[consumed={consumed}] 续播，实际 _i={eng._i}")
        self.assertEqual(eng.remaining, eng.chunk - consumed)
        row = eng.tick()
        self.assertIsNotNone(row)
        self.assertAlmostEqual(row[0], 1000 * 2 + consumed,
                               msg="应从第 2 块 new[consumed] 续播（修复前按 latency_ms 估算会跳 5 行）")
        eng.stop()

    def test_lockfree_replan_with_ensembling_resumes_at_consumed(self):
        """时序融合 + 锁外推理：融合用锚点 snap_i 对齐，续播仍从 new[consumed]，不崩、有限。"""
        class RampBackend(StubBackend):
            def __init__(self, dim=DIM, delay=0.15):
                super().__init__(action_dim=dim, camera_map={})
                self.delay = delay
                self.plans = 0

            def infer(self, obs):
                time.sleep(self.delay)
                self.plans += 1
                base = float(1000 * self.plans)
                return (base + np.arange(10, dtype=np.float64))[:, None] * np.ones((1, DIM))

        eng = ActionEngine(RampBackend(delay=0.15), mode="queue_async", action_dim=DIM,
                           chunk=10, autostart=False, async_prefetch_ahead=1,
                           temporal_ensemble_coeff=0.01)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng.set_enabled(True)
        eng.start()
        self.assertIsNotNone(eng.tick())
        self.assertIsNotNone(eng.tick())  # _i=2
        t = threading.Thread(target=eng._run_plan, daemon=True)
        t.start()
        time.sleep(0.05)
        self.assertIsNotNone(eng.tick())
        self.assertIsNotNone(eng.tick())  # _i 2→4
        t.join(timeout=1.0)
        consumed = 2
        self.assertEqual(eng._i, consumed)
        row = eng.tick()
        self.assertIsNotNone(row)
        self.assertTrue(np.isfinite(row).all())
        # 融合行应介于"旧流续行 old[4]=1004"与"新 new[2]=2002"之间（权重平均），
        # 而不是从 new[0]=2000 起跳、也不是回到旧流 1004——验证锚点对齐没被破坏
        lo, hi = 1004.0, 2002.0
        self.assertGreaterEqual(row[0], lo - 1)
        self.assertLessEqual(row[0], hi + 1)
        eng.stop()


class TestChunkAnchor(unittest.TestCase):
    """换 chunk 切换平滑（chunk_anchor_tol>0）：续播起点偏离"正在执行的旧 command"
    >tol 时，前 blend 行从旧值平滑过渡到新轨迹——消除收敛拉回/模型突变两类切换尖峰。
    起点必须是旧 command 当前行（不是实测 state）：收敛拉回型跳后新 chunk[i0]≈state、
    dev<tol 不触发，真正跳的是旧 command 漂移量。"""

    def _engine(self, tol=0.05, blend=4):
        eng, _ = make_engine(mode="queue_async", chunk=50,
                             chunk_anchor_tol=tol, chunk_anchor_blend=blend)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng._install(np.zeros((50, DIM)), 0, snap_i=0, consumed=0)  # 首次 chunk（全零）
        eng._i = 20                                                 # 已消费 20 行
        return eng

    def test_blends_from_old_command_when_switch_jumps(self):
        """旧 command 漂移 0.5 vs 新 chunk[i0]=0.8（切换差 0.3）：首拍≈旧值平滑过渡，
        不产生一步跳；blend 之外恢复新 chunk 原值。锚 = 最后**已发出**的行
        _chunk[_i-1]=_chunk[19]（机器人正在执行它），不是 _chunk[20]（还没发）。"""
        eng = self._engine()
        eng._chunk[19, 0] = 0.5            # 旧 command 最后已发行为 0.5（_i=20 → 19）
        new = np.zeros((50, DIM)); new[:, 0] = 0.8
        eng._install(new, 0.1, snap_i=20, consumed=3)   # i0=3 → chunk[3,0]=0.8
        # 首拍 blend：w=1/(4+1)=0.2 → chunk[3] = 0.5 + 0.2×0.3 = 0.56（几乎旧值，不跳）
        self.assertAlmostEqual(eng._chunk[3, 0], 0.56, places=6)
        # 第 4 行收尾：w=4/5 → 0.74
        self.assertAlmostEqual(eng._chunk[6, 0], 0.74, places=6)
        # blend 之外恢复新 chunk 原值
        self.assertAlmostEqual(eng._chunk[7, 0], 0.8, places=9)
        self.assertEqual(eng._i, 3)
        eng.stop()

    def test_noop_within_tol(self):
        """切换差 0.02 < tol 0.05：正常跟随差不触发，新 chunk 原样安装。"""
        eng = self._engine()
        eng._chunk[19, 0] = 0.78
        new = np.zeros((50, DIM)); new[:, 0] = 0.8
        eng._install(new, 0.1, snap_i=20, consumed=3)
        self.assertAlmostEqual(eng._chunk[3, 0], 0.8, places=9)  # 未 blend
        eng.stop()

    def test_disabled_when_tol_nonpositive(self):
        """tol<=0 关闭：即使切换差大也不 blend（硬切换，行为不变）。"""
        eng = self._engine(tol=0.0)
        eng._chunk[19, 0] = 0.5
        new = np.zeros((50, DIM)); new[:, 0] = 0.8
        eng._install(new, 0.1, snap_i=20, consumed=3)
        self.assertAlmostEqual(eng._chunk[3, 0], 0.8, places=9)
        eng.stop()

    def test_readonly_remote_chunk_is_copied_before_anchor_blend(self):
        """msgpack decodes ndarray data from immutable bytes; anchor may mutate it.

        This reproduces TEST_0922-1626 exactly: coeff=0 leaves the remote array
        untouched, then anchor blending needs to write the replacement chunk.
        """
        eng = self._engine()
        eng._chunk[19, 0] = 0.5
        payload = np.zeros((50, DIM), dtype=np.float64)
        payload[:, 0] = 0.8
        readonly = np.frombuffer(payload.tobytes(), dtype=np.float64).reshape(50, DIM)
        before = readonly.copy()
        self.assertFalse(readonly.flags.writeable)

        install = eng._install(readonly, 0.1, snap_i=20, consumed=3)

        self.assertIsNotNone(install)
        self.assertTrue(eng._chunk.flags.writeable)
        self.assertEqual(install["anchor_blended_rows"], 4)
        self.assertAlmostEqual(eng._chunk[3, 0], 0.56, places=6)
        np.testing.assert_array_equal(readonly, before)  # backend response not mutated
        eng.stop()

    def test_readonly_chunk_copy_preserves_disabled_anchor_behavior(self):
        """The ownership copy must not numerically change the legacy tol=0 path."""
        eng = self._engine(tol=0.0)
        payload = np.arange(50 * DIM, dtype=np.float64).reshape(50, DIM)
        readonly = np.frombuffer(payload.tobytes(), dtype=np.float64).reshape(50, DIM)
        install = eng._install(readonly, 0.1, snap_i=20, consumed=3)
        self.assertIsNotNone(install)
        self.assertTrue(eng._chunk.flags.writeable)
        np.testing.assert_array_equal(eng._chunk, readonly)
        self.assertEqual(eng._i, 3)
        eng.stop()

    def test_readonly_chunk_with_temporal_ensemble_and_anchor(self):
        """Defense-in-depth for the combined ACT-style ensemble + anchor path."""
        eng, _ = make_engine(
            mode="queue_async", chunk=50, temporal_ensemble_coeff=0.01,
            chunk_anchor_tol=0.05, chunk_anchor_blend=4,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng._install(np.zeros((50, DIM)), 0, snap_i=0, consumed=0)
        eng._i = 20
        eng._chunk[19, 0] = 0.5
        payload = np.zeros((50, DIM), dtype=np.float64)
        payload[:, 0] = 1.0
        readonly = np.frombuffer(payload.tobytes(), dtype=np.float64).reshape(50, DIM)
        install = eng._install(readonly, 0.1, snap_i=20, consumed=3)
        self.assertIsNotNone(install)
        self.assertTrue(eng._chunk.flags.writeable)
        self.assertTrue(np.isfinite(eng._chunk).all())
        eng.stop()

    def test_failed_postprocess_rolls_back_chunk_index_and_ensemble_state(self):
        """Chunk installation is atomic even if a future postprocessor raises."""
        eng, _ = make_engine(
            mode="queue_async", chunk=50, temporal_ensemble_coeff=0.01,
            chunk_anchor_tol=0.05,
        )
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng._install(np.zeros((50, DIM)), 0, snap_i=0, consumed=0)
        eng._i = 20
        old_chunk = eng._chunk.copy()
        old_counts = eng._ensembler.counts.copy()
        old_i = eng._i
        real_update = eng._ensembler.update

        def update_then_fail(*args, **kwargs):
            real_update(*args, **kwargs)  # deliberately mutates counts first
            raise RuntimeError("injected postprocess failure")

        eng._ensembler.update = update_then_fail
        with self.assertRaisesRegex(RuntimeError, "injected postprocess failure"):
            eng._install(np.ones((50, DIM)), 0.1, snap_i=20, consumed=3)
        np.testing.assert_array_equal(eng._chunk, old_chunk)
        np.testing.assert_array_equal(eng._ensembler.counts, old_counts)
        self.assertEqual(eng._i, old_i)
        eng.stop()


class ServerTimingBackend(StubBackend):
    """带 last_server_timing 的 remote 风格 backend（serve.py 随响应返回分项）。"""

    def infer(self, obs):
        self.last_server_timing = {
            "prep_ms": 1.0, "pre_ms": 0.2, "infer_ms": 3.0,
            "post_ms": 0.1, "total_ms": 4.3,
        }
        return super().infer(obs)


class TestServerTiming(unittest.TestCase):
    def test_propagates_to_stats(self):
        """服务端分项 server_timing 应透传到 engine.stats（进 state/metrics），
        从而可拆"网络+序列化 vs 服务端推理"（RTT−server_total）。"""
        backend = ServerTimingBackend(action_dim=DIM, camera_map={})
        eng = ActionEngine(backend, mode="queue_sync", action_dim=DIM, chunk=10,
                           autostart=False)
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng._run_plan()
        st = eng.stats
        self.assertEqual(st["server_timing"]["total_ms"], 4.3)
        self.assertIn("prep_ms", st["server_timing"])
        eng.stop()

    def test_none_when_backend_has_no_timing(self):
        """无 server_timing 的 backend（Stub/Inproc）→ None，不崩。"""
        eng, _ = make_engine(mode="queue_sync")
        eng.feed_obs(ObsBatch(state=np.zeros(DIM), images={}, prompt=""))
        eng._run_plan()
        self.assertIsNone(eng.stats["server_timing"])
        eng.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
