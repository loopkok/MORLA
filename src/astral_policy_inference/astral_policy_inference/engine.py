"""Action-chunk execution engines: queue_sync / queue_async / rtc.

The engine turns a backend's *chunks* into a stream of single absolute action
rows at the policy rate. pi0.5/ACT outputs are absolute joint targets. Guided
RTC also sends the unexecuted tail into OpenPI's denoiser so the next chunk is
conditioned before it replaces the current one.

Modes:
* ``queue_sync``  — refill blocks the calling (control) thread at chunk end.
* ``queue_async`` — background planner pre-fetches the next chunk once the
  current one is within ``async_prefetch_ahead`` of empty; the control thread
  never blocks after the initial plan.
* ``rtc``         — rolling re-plan. Guided OpenPI pi05 starts inference when
  remaining rows reach ``rtc_queue_threshold`` and conditions generation on
  that tail; the legacy unconditioned path uses ``rtc_replan_interval_s`` and
  ``rtc_min_tail``.

No ROS state here; backend is injected, so the engine is unit-testable with a
fake backend. Threading is optional (``autostart=False``) for deterministic
tests.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

import numpy as np

from astral_policy_inference.backend import ObsBatch, PolicyBackend, PolicyError


class EngineStateError(RuntimeError):
    """Engine has no usable chunk (planner failed / no observation yet)."""


class TemporalEnsembler:
    """ACT 时序融合——借鉴 lerobot ``ACTTemporalEnsembler``（ACT 论文 Algorithm 2）。

    lerobot 版在 ``temporal_ensemble_coeff`` 非空时**每 tick 都重规划**，新 chunk 与
    历史预测做在线指数加权平均，权重 ``w_i = exp(-coeff*i)``（coeff>0 → 旧预测权重更高），
    于是单个新 chunk 无法瞬间跳变。我们的引擎每 ~chunk/2 行才重规划一次，这里把
    ``update`` 推广到**部分消费后重规划**：新 chunk 的头部与缓冲里未消费的尾段（时间
    对齐，都是"从现在起的后续行"）按 ACT 权重平均，超出重叠的新尾部直接追加。

    权重语义：某时步被预测 c 次时，新值权重 = ``w_c / Σ_{0..c} w_i``；c=1 时 ≈1/2
    （边界步直接减半），之后指数衰减（坏 chunk 的影响被逐步稀释）。
    """

    def __init__(self, coeff: float, chunk_size: int) -> None:
        self.coeff = float(coeff)
        self.chunk_size = int(chunk_size)
        self.weights = np.exp(-self.coeff * np.arange(self.chunk_size))
        self.weights_cumsum = np.cumsum(self.weights)
        self.reset()

    def reset(self) -> None:
        self.counts: Optional[np.ndarray] = None

    def update(
        self,
        new: np.ndarray,
        old: Optional[np.ndarray] = None,
        i: int = 0,
    ) -> np.ndarray:
        """并入新 chunk，返回融合后的 (n, dim) 缓冲。

        ``new``：新 chunk（(n, dim)，从"当前"起预测）；``old`` + ``i``：旧缓冲（已消费
        i 行，未消费尾段与 ``new`` 头部时间对齐）。``old`` 为 None 或首次调用 → 直接采用。
        """
        new = np.asarray(new, dtype=np.float64)
        n = new.shape[0]
        out = np.array(new, copy=True)
        if old is None or self.counts is None:
            self.counts = np.ones(n, dtype=np.int64)
            return out
        M = self.chunk_size
        # 重叠行数 = 旧缓冲剩余未消费行数 ∩ 新 chunk 长度。只按 min(M-i, n) 取会在
        # 旧缓冲短于 chunk_size（后端返回行数 < chunk，如 StubBackend 4 行/截断响应）
        # 时越界（old[i+k]/counts[i+k]）。对抗性回归 test_ensembling_short_chunk_*。
        r = int(min(max(0, len(old) - i), n))
        if r > 0:
            for k in range(r):
                c = int(self.counts[i + k])  # 该时步已被预测次数（本次融合前）
                ci = min(c, M - 1)
                w_new = float(self.weights[ci])
                w_old = float(self.weights_cumsum[ci - 1]) if ci >= 1 else 0.0
                out[k] = (old[i + k] * w_old + new[k] * w_new) / (w_old + w_new)
            if n > r:
                out[r:] = new[r:]
        self.counts = np.concatenate(
            [
                self.counts[i : i + r] + 1 if r > 0 else np.zeros(0, dtype=np.int64),
                np.ones(max(0, n - r), dtype=np.int64),
            ]
        )
        return out


class ActionEngine:
    def __init__(
        self,
        backend: PolicyBackend,
        *,
        mode: str = "queue_async",
        action_dim: int,
        chunk: int = 50,
        policy_fps: int = 30,
        async_prefetch_ahead: Optional[int] = None,
        rtc_replan_interval_s: Optional[float] = None,
        rtc_min_tail: Optional[int] = None,
        rtc_guided: bool = False,
        rtc_queue_threshold: int = 30,
        rtc_execution_horizon: int = 10,
        rtc_max_guidance_weight: float = 10.0,
        rtc_prefix_schedule: str = "linear",
        queue_min_replan_interval_s: float = 0.05,
        autostart: bool = True,
        abs_action_min_scale: float = 0.5,
        temporal_ensemble_coeff: float = 0.0,
        chunk_anchor_tol: float = 0.0,
        chunk_anchor_blend: int = 4,
        plan_trace_hook: Optional[Callable[[dict], None]] = None,
    ):
        if mode not in ("queue_sync", "queue_async", "rtc"):
            raise ValueError(f"mode={mode!r} not in queue_sync|queue_async|rtc")
        self.backend = backend
        self.mode = mode
        self.action_dim = int(action_dim)
        self.chunk = int(chunk)
        self.policy_fps = int(policy_fps)
        # 绝对动作语义守卫：机器人离开零位时，chunk 首行目标值须保持量级；
        # <=0 关闭。见 _check_absolute_semantics。
        self.abs_action_min_scale = float(abs_action_min_scale)
        # ACT 时序融合系数：>0 时换 chunk 用 TemporalEnsembler 加权平均（借鉴 lerobot
        # ACTTemporalEnsembler），边界不再硬跳；0 = 关闭（默认，行为不变）。ACT 推荐 0.01。
        self.temporal_ensemble_coeff = float(temporal_ensemble_coeff)
        self._ensembler: Optional[TemporalEnsembler] = (
            TemporalEnsembler(self.temporal_ensemble_coeff, self.chunk)
            if self.temporal_ensemble_coeff > 0
            else None
        )
        # 换 chunk 切换平滑：安装时若续播起点（融合后 chunk[i0]）偏离正在执行的旧
        # command >chunk_anchor_tol rad，把前 chunk_anchor_blend 行从旧值平滑过渡到
        # 新轨迹——消除"观测过时/预测漂移 → 换 chunk 一步追向实测"的收敛跳变。
        # <=0 关闭（硬切换）；blend=过渡行数（@30Hz 每行 33ms）。
        self.chunk_anchor_tol = float(chunk_anchor_tol)
        self.chunk_anchor_blend = int(max(1, chunk_anchor_blend))
        # Hook must be non-blocking.  PolicyNode feeds it to a dedicated JSONL
        # writer, so detailed tracing cannot delay the planner/control threads.
        self._plan_trace_hook = plan_trace_hook
        self.async_prefetch_ahead = (
            int(async_prefetch_ahead)
            if async_prefetch_ahead is not None
            else max(1, self.chunk // 2)
        )
        self.rtc_replan_interval_s = (
            float(rtc_replan_interval_s)
            if rtc_replan_interval_s is not None
            else min(1.0, self.chunk / (2.0 * self.policy_fps))
        )
        self.rtc_min_tail = (
            int(rtc_min_tail) if rtc_min_tail is not None else max(1, self.chunk // 5)
        )
        self.rtc_guided = bool(rtc_guided)
        self.rtc_queue_threshold = int(rtc_queue_threshold)
        self.rtc_execution_horizon = int(rtc_execution_horizon)
        self.rtc_max_guidance_weight = float(rtc_max_guidance_weight)
        self.rtc_prefix_schedule = str(rtc_prefix_schedule).lower()
        if self.rtc_guided and mode != "rtc":
            raise ValueError("rtc_guided requires engine mode='rtc'")
        if self.rtc_guided and not 0 <= self.rtc_queue_threshold < self.chunk:
            raise ValueError("rtc_queue_threshold must be in [0, action_chunk)")
        if self.rtc_guided and (self.rtc_execution_horizon < 1 or self.rtc_execution_horizon > self.chunk):
            raise ValueError("rtc_execution_horizon must be in [1, action_chunk]")
        if not math.isfinite(self.rtc_max_guidance_weight) or self.rtc_max_guidance_weight <= 0:
            raise ValueError("rtc_max_guidance_weight must be positive")
        if self.rtc_prefix_schedule not in ("zeros", "ones", "linear", "exp"):
            raise ValueError("unknown RTC prefix attention schedule")
        self._rtc_max_latency_ms = 0.0
        self.queue_min_replan_interval_s = float(queue_min_replan_interval_s)
        self._autostart = autostart

        self._lock = threading.RLock()
        self._chunk: Optional[np.ndarray] = None
        self._i = 0                       # next row to emit from current chunk
        self._pops_since_install = 0
        self._obs: Optional[ObsBatch] = None
        self._planning = False
        self._last_plan_ms = 0.0
        self._last_server_timing: Optional[dict] = None
        self._last_accept_t = 0.0
        self._plans = 0
        self._pops = 0
        self._last_error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.enabled = False

    # ------------------------------------------------------------------ obs

    def feed_obs(self, obs: ObsBatch) -> None:
        with self._lock:
            self._obs = ObsBatch(
                state=np.array(obs.state, copy=True),
                images={
                    k: np.array(v, copy=True) for k, v in (obs.images or {}).items()
                },
                prompt=str(obs.prompt or ""),
                metadata=dict(obs.metadata or {}),
            )

    @property
    def has_obs(self) -> bool:
        with self._lock:
            return self._obs is not None

    # ------------------------------------------------------------- lifecycle

    def start(self) -> None:
        """Open backend; block until an initial chunk exists."""
        self.backend.open()
        self.enabled = True
        if self._chunk is None:
            self._plan_blocking()  # synchronous first plan → POLICY can move at once
        if self._chunk is None:
            err = self.last_error or "policy returned no chunk"
            raise EngineStateError(err)
        if self._autostart and self.mode != "queue_sync" and self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._planner_loop, name="policy-planner", daemon=True
            )
            self._thread.start()

    def set_enabled(self, enabled: bool) -> None:
        """Gate planning/ticking while the node is not in POLICY."""
        self.enabled = bool(enabled)

    def stop(self) -> None:
        self.enabled = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        # close under the lock so a queue_sync refill that is mid-infer (control
        # thread, holding the lock during _plan_blocking) can never race it
        try:
            with self._lock:
                self.backend.close()
        except Exception:  # noqa: BLE001
            pass

    def reset(self) -> None:
        """Drop queued chunk + policy-side context (used on human takeover)."""
        with self._lock:
            self._chunk = None
            self._i = 0
            self._pops_since_install = 0
            self._last_error = None
            self._rtc_max_latency_ms = 0.0
            if self._ensembler is not None:
                # 必须在锁内：planner 线程的 _install 也在锁内改融合计数，锁外清会竞态
                self._ensembler.reset()
        try:
            self.backend.reset()
        except Exception:  # noqa: BLE001
            pass

    # --------------------------------------------------------------- ticks

    def tick(self) -> Optional[np.ndarray]:
        """One absolute action row for the current policy tick, or None."""
        if not self.enabled:
            return None
        while True:
            with self._lock:
                if self._chunk is not None and self._i < len(self._chunk):
                    row = self._chunk[self._i].copy()
                    self._i += 1
                    self._pops += 1
                    self._pops_since_install += 1
                    return row
                if self._chunk is not None and self._last_error is not None:
                    err = self._last_error
                    self._last_error = None
                    raise EngineStateError(err)
            if self.mode == "queue_sync":
                if not self.has_obs:
                    return None
                if self._chunk is None and self._last_error is not None:
                    err = self._last_error
                    self._last_error = None
                    raise EngineStateError(err)
                self._plan_blocking()
                continue
            # async/rtc: the planner thread is responsible for refills. If it
            # already failed hard, surface that instead of holding forever.
            err = self.last_error
            if err:
                raise EngineStateError(err)
            return None

    @property
    def need_refill(self) -> bool:
        with self._lock:
            return self._chunk is None or self._i >= len(self._chunk)

    @property
    def remaining(self) -> int:
        with self._lock:
            if self._chunk is None:
                return 0
            return max(0, len(self._chunk) - self._i)

    @property
    def last_error(self) -> Optional[str]:
        with self._lock:
            return self._last_error

    def clear_error(self) -> None:
        with self._lock:
            self._last_error = None

    @property
    def stats(self) -> dict:
        with self._lock:
            return {
                "mode": self.mode,
                "chunk": self.chunk,
                "plans": self._plans,
                "pops": self._pops,
                "last_plan_ms": self._last_plan_ms,
                "server_timing": self._last_server_timing,
                "remaining": self.remaining,
                "error": self._last_error,
            }

    # -------------------------------------------------------------- planning

    def _snapshot_obs(self) -> ObsBatch:
        with self._lock:
            return ObsBatch(
                state=np.array(self._obs.state, copy=True),
                images={
                    k: np.array(v, copy=True)
                    for k, v in (self._obs.images or {}).items()
                },
                prompt=self._obs.prompt,
                metadata=dict(self._obs.metadata or {}),
            )

    @staticmethod
    def _trace_observation(obs: ObsBatch) -> dict:
        """A serialisable request-side view; deliberately excludes image pixels."""
        return {
            "state": np.asarray(obs.state, dtype=np.float64).tolist(),
            "prompt": str(obs.prompt or ""),
            "image_labels": sorted((obs.images or {}).keys()),
            "metadata": dict(obs.metadata or {}),
        }

    def _emit_plan_trace(self, rec: dict) -> None:
        hook = self._plan_trace_hook
        if hook is None:
            return
        try:
            hook(rec)
        except Exception:  # noqa: BLE001 - diagnostic tracing must be fail-open
            pass

    def _plan_blocking(self) -> None:
        """Blocking plan (first chunk / queue_sync refill).

        Holds the engine lock across inference so ``stop()``/teardown that
        close the backend on another thread cannot race a running infer.
        """
        with self._lock:
            if self._obs is None:
                return
            self._planning = True
            try:
                self._run_plan()
            finally:
                self._planning = False

    def _run_plan(self) -> bool:
        """Infer one chunk and (re)install it. Returns success.

        推理在引擎锁**外**执行。之前锁跨 ``infer()`` 持有，慢推理（远程网络 ~100ms+）
        会堵住控制线程的 ``tick()`` 整个推理时长 → 每次重规划都产生指令空档
        （"hold → lunge"）。现在：锁内只做观测快照 + 记录 ``_i_snap``（融合对齐锚点），
        释放锁推理，再取锁安装；续播索引用**实测** ``consumed``（推理期间控制线程
        实际消费的行数），比 ``latency_ms`` 估算更准——旧估算在"控制线程被锁堵、机器人
        实际没动"时还会超前跳。

        并发安全：queue_sync 走 ``_plan_blocking``（外层仍持锁），控制线程重填保持原子；
        queue_async 的 planner 线程锁外推理，``stop()`` 先 join planner 再锁内关后端，
        都不会与在飞推理竞态。
        """
        with self._lock:
            if self._obs is None:
                return False
            obs = self._snapshot_obs()
            snap_i = self._i  # 观测时刻的消费位置（时间对齐锚点）
            rtc = None
            if self.rtc_guided and self._chunk is not None and snap_i < len(self._chunk):
                # Prefix is the still-committed absolute command trajectory.
                # The OpenPI server maps it through Astral's DeltaActions and
                # Normalize transforms into the model's action space.
                remaining = self._chunk[snap_i : snap_i + self.rtc_execution_horizon]
                valid = len(remaining)
                prefix = np.repeat(remaining[-1:, :], self.rtc_execution_horizon, axis=0)
                prefix[:valid] = remaining
                estimated_delay = math.ceil(self._rtc_max_latency_ms * self.policy_fps / 1000.0)
                rtc = {
                    "prefix": prefix.astype(np.float32),
                    "valid": valid,
                    "delay": min(estimated_delay, valid),
                    "max_guidance_weight": self.rtc_max_guidance_weight,
                    "schedule": self.rtc_prefix_schedule,
                }
        request_t = time.time()
        t0 = time.perf_counter()
        try:
            full = np.asarray(
                self.backend.infer_rtc(obs, rtc) if rtc is not None else self.backend.infer(obs),
                dtype=np.float64,
            )
        except PolicyError as exc:
            with self._lock:
                self._last_error = str(exc)
            self._emit_plan_trace({
                "schema": "pi_plan_trace/v1", "event": "infer_failed",
                "t": round(time.time(), 6), "mode": self.mode,
                "snapshot": self._trace_observation(obs),
                "inference": {"request_t": round(request_t, 6),
                              "rtt_ms": round((time.perf_counter() - t0) * 1000.0, 3)},
                "error": str(exc),
            })
            return False
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._last_error = f"infer failed: {exc}"
            self._emit_plan_trace({
                "schema": "pi_plan_trace/v1", "event": "infer_failed",
                "t": round(time.time(), 6), "mode": self.mode,
                "snapshot": self._trace_observation(obs),
                "inference": {"request_t": round(request_t, 6),
                              "rtt_ms": round((time.perf_counter() - t0) * 1000.0, 3)},
                "error": f"infer failed: {exc}",
            })
            return False
        ms = (time.perf_counter() - t0) * 1000.0
        trace_rec = None
        with self._lock:
            # 服务端分项（serve.py 返回 prep/pre/infer/post/total）——从 backend 带回，
            # 进 state/metrics 后能直接拆"网络+序列化 vs 服务端推理"（RTT−total）。
            self._last_server_timing = getattr(
                self.backend, "last_server_timing", None
            )
            # 推理期间控制线程继续消费，机器人确实前进了 consumed 行 → 从这里续播
            consumed = max(0, self._i - snap_i)
            try:
                self._check_absolute_semantics(full, obs.state)
            except PolicyError as exc:
                self._last_error = str(exc)
                trace_rec = {
                    "schema": "pi_plan_trace/v1", "event": "plan_rejected",
                    "t": round(time.time(), 6), "mode": self.mode,
                    "snapshot": self._trace_observation(obs),
                    "inference": {"request_t": round(request_t, 6), "rtt_ms": round(ms, 3), "server_timing": self._last_server_timing},
                    "alignment": {"snap_i": snap_i, "i_at_install": self._i,
                                  "consumed": consumed},
                    "server_chunk": full.tolist(), "error": str(exc),
                }
            else:
                install = self._install(full, latency_ms=ms, snap_i=snap_i, consumed=consumed)
                if install is not None:
                    if self.rtc_guided:
                        self._rtc_max_latency_ms = max(self._rtc_max_latency_ms, ms)
                    trace_rec = {
                        "schema": "pi_plan_trace/v1", "event": "plan_installed",
                        "t": round(self._last_accept_t, 6), "mode": self.mode,
                        "plan": self._plans,
                        "snapshot": self._trace_observation(obs),
                        "inference": {"request_t": round(request_t, 6), "rtt_ms": round(ms, 3), "server_timing": self._last_server_timing},
                        "alignment": {**install, "rtc": {
                            "guided": self.rtc_guided,
                            "conditioned_delay": rtc["delay"] if rtc is not None else 0,
                            "prefix_valid": rtc["valid"] if rtc is not None else 0,
                        }},
                        # This is the complete action chunk received from the remote
                        # policy before any local ensembling/anchor processing.
                        "server_chunk": full.tolist(),
                        "installed_chunk": self._chunk.tolist() if self._chunk is not None else [],
                        "postprocess": {
                            "temporal_ensemble_coeff": self.temporal_ensemble_coeff,
                            "chunk_anchor_tol": self.chunk_anchor_tol,
                            "chunk_anchor_blend": self.chunk_anchor_blend,
                        },
                    }
                else:
                    trace_rec = {
                        "schema": "pi_plan_trace/v1", "event": "plan_rejected",
                        "t": round(time.time(), 6), "mode": self.mode,
                        "snapshot": self._trace_observation(obs),
                        "inference": {"request_t": round(request_t, 6), "rtt_ms": round(ms, 3), "server_timing": self._last_server_timing},
                        "alignment": {"snap_i": snap_i, "i_at_install": self._i,
                                      "consumed": consumed},
                        "server_chunk": full.tolist(),
                        "error": self._last_error or "chunk installation failed",
                    }
        if trace_rec is not None:
            self._emit_plan_trace(trace_rec)
        return trace_rec is not None and trace_rec["event"] == "plan_installed"

    def _check_absolute_semantics(self, full: np.ndarray, state: np.ndarray) -> None:
        """绝对动作语义守卫：机器人明显离开零位时，chunk 首行目标值不得全是小值。

        后端契约是绝对动作（next-state ≈ 当前 state，量级一致）。若某后端把 delta
        当绝对返回（openpi server 漏 AbsoluteActions / 错 checkpoint），首行≈小 delta
        （~0.03 rad）；而当前 state 离开零位（max|state|>1）时，绝对目标不可能对离开
        零位的关节给近零值 → 拒绝。只查 |state|>1 的维（臂关节；夹爪 state∈[0,1] 自动
        排除），机器人接近零位时 fail-open（绝对与 delta 目标都近零，危害可忽略）。
        不做「|action-state| 必须小」的假设（对 OOD 输入/激进模型会误报）。
        abs_action_min_scale<=0 关闭。
        """
        if self.abs_action_min_scale <= 0 or full is None or len(full) == 0:
            return
        row = np.asarray(full[0], dtype=np.float64).reshape(-1)
        state = np.asarray(state, dtype=np.float64).reshape(-1)
        if row.shape[0] != state.shape[0]:
            return  # 维度不一致交给 action_dim 校验
        informative = np.abs(state) > 1.0
        if not informative.any():
            return
        arm_scale = np.abs(row)[informative].max()
        if arm_scale < self.abs_action_min_scale:
            raise PolicyError(
                f"action does not look absolute: on dims where |state|>1.0, chunk "
                f"first row max|action|={arm_scale:.3f} < "
                f"{self.abs_action_min_scale} (delta-as-absolute?). Check backend "
                f"config (openpi server missing AbsoluteActions?) or "
                f"abs_action_min_scale."
            )

    def _install(
        self,
        full: np.ndarray,
        latency_ms: float,
        snap_i: Optional[int] = None,
        consumed: int = 0,
    ) -> Optional[dict]:
        """Swap the chunk and resume at the right index.

        ``snap_i`` = 观测捕获时的消费位置（融合对齐锚点）；``consumed`` = 推理期间
        控制线程实际消费的行数（续播索引）。锁外推理时二者必须显式传入——用当前
        ``_i`` 会因推理期间 ``_i`` 已前移而把 ``old[当前_i]`` 与 ``new[0]`` 错位对齐；
        用 ``latency_ms`` 估算则会在"控制线程被锁堵、机器人实际没动"时超前跳。
        """
        full = np.asarray(full, dtype=np.float64)
        if full.ndim == 1:
            full = full.reshape(1, -1)
        if full.ndim != 2:
            self._last_error = (
                f"chunk must be 2-D (rows, action_dim), got shape {full.shape}"
            )
            return None
        if full.shape[1] != self.action_dim:
            self._last_error = (
                f"chunk action_dim {full.shape[1]} != {self.action_dim}"
            )
            return None
        rows = min(self.chunk, len(full))
        if rows <= 0:
            self._last_error = "policy returned an empty action chunk"
            return None
        if self.rtc_guided and consumed >= rows:
            self._last_error = "RTC inference latency exceeded the returned action horizon"
            return None
        # The websocket/msgpack decoder reconstructs ndarrays on top of an
        # immutable ``bytes`` buffer.  ``np.asarray(..., dtype=float64)`` keeps
        # that read-only storage when no dtype conversion is needed.  The
        # installed chunk is engine-owned and anchor blending mutates its first
        # rows, so take one explicit writable copy at this ownership boundary.
        # This also prevents a backend from changing an executing chunk after
        # ``infer()`` returns.  The copy is tiny (normally 50 x 8 float64).
        tail = np.array(full[:rows], dtype=np.float64, copy=True)
        anchor = snap_i if snap_i is not None else self._i
        was_moving = self._chunk is not None and anchor < len(self._chunk)
        # 安装前"正在执行的旧 command 当前行"——blend 起点。切换尖峰的本质是
        # 旧 command（开环 chunk 内漂移/过时）→ 新 chunk[i0] 的瞬间差；从旧值起步
        # 过渡到新轨迹，把切换差摊到 nblend 行，直接消除指令流不连续。
        # 锚 = 最后**已发出**的行 _chunk[_i-1]（机器人正在执行它）；_chunk[_i] 是
        # 下一行、还没发给机器人，用它起步会超前一行。
        old_row = (
            self._chunk[self._i - 1].copy()
            if self._chunk is not None and 1 <= self._i <= len(self._chunk)
            else None
        )
        old_next_row = (
            self._chunk[self._i].copy()
            if self._chunk is not None and self._i < len(self._chunk)
            else None
        )
        old_at_snapshot = (
            self._chunk[anchor].copy()
            if self._chunk is not None and 0 <= anchor < len(self._chunk)
            else None
        )
        if self.mode != "queue_sync" and was_moving:
            # 续播索引 = 推理期间实际消费的行数（对齐到机器人真实位置）
            i0 = int(np.clip(consumed, 0, max(0, rows - 1)))
        else:
            i0 = 0
        # Build the replacement transactionally.  Until all post-processing
        # succeeds, ``self._chunk`` remains the previously accepted trajectory.
        # TemporalEnsembler owns mutable counters, so snapshot and roll those
        # back too if a future post-processor unexpectedly raises.
        ensembler_counts_before = (
            None
            if self._ensembler is None or self._ensembler.counts is None
            else self._ensembler.counts.copy()
        )
        try:
            if self._ensembler is not None:
                # ACT 时序融合：用锚点 anchor（=snap_i）对齐 old[anchor+k] ↔ new[k]，
                # 保证时间语义；前 consumed 行是推理期间已执行的 stale 混合，由 i0 跳过。
                candidate = self._ensembler.update(
                    tail,
                    old=self._chunk if was_moving else None,
                    i=anchor if was_moving else 0,
                )
            else:
                candidate = tail
            # 换 chunk 实测对齐（切换平滑）：续播起点 candidate[i0] 偏离
            # "正在执行的旧 command" >tol 时，前 blend 行从旧值平滑过渡到新轨迹。
            anchor_deviation = None
            anchor_blended_rows = 0
            if (
                self.chunk_anchor_tol > 0
                and was_moving
                and old_row is not None
                and candidate.shape[0] > i0
                and candidate.shape[1] == old_row.shape[0]
            ):
                dev = float(np.abs(candidate[i0] - old_row).max())
                anchor_deviation = dev
                if dev > self.chunk_anchor_tol:
                    nblend = int(min(self.chunk_anchor_blend, candidate.shape[0] - i0))
                    for k in range(1, nblend + 1):
                        w = k / (nblend + 1)
                        candidate[i0 + k - 1] = (
                            old_row + w * (candidate[i0 + k - 1] - old_row)
                        )
                    anchor_blended_rows = nblend
            selected_row = candidate[i0].copy() if candidate.shape[0] > i0 else None
        except Exception:
            if self._ensembler is not None:
                self._ensembler.counts = ensembler_counts_before
            raise

        # Atomic commit: indices/statistics change only after candidate is valid.
        self._chunk = candidate
        self._i = i0
        self._pops_since_install = 0
        self._plans += 1
        self._last_plan_ms = latency_ms
        self._last_error = None
        self._last_accept_t = time.time()
        return {
            "snap_i": int(snap_i) if snap_i is not None else None,
            "i_at_install": int(self._i),
            "consumed": int(consumed), "resume_i": int(i0), "rows": int(rows),
            "was_moving": bool(was_moving),
            "old_last_sent": old_row.tolist() if old_row is not None else None,
            "old_next": old_next_row.tolist() if old_next_row is not None else None,
            "old_at_snapshot": old_at_snapshot.tolist() if old_at_snapshot is not None else None,
            "selected_row": selected_row.tolist() if selected_row is not None else None,
            "anchor_deviation_max": anchor_deviation,
            "anchor_blended_rows": anchor_blended_rows,
        }

    # ------------------------------------------------------- planner thread

    def _planner_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                if not self.enabled or self._obs is None or self._planning:
                    idle = True
                elif self._chunk is None or self._i >= len(self._chunk):
                    idle = False          # boundary: urgent refill
                elif self.mode == "queue_async":
                    since = time.time() - self._last_accept_t
                    idle = (
                        self.remaining > self.async_prefetch_ahead
                        or since < self.queue_min_replan_interval_s
                    )
                elif self.mode == "rtc":
                    if self.rtc_guided:
                        # LeRobot RTCInferenceEngine starts a new conditioned
                        # inference when the action queue reaches its threshold.
                        idle = self.remaining > self.rtc_queue_threshold
                    else:
                        due = (time.time() - self._last_accept_t) >= self.rtc_replan_interval_s
                        idle = not (due and self._pops_since_install >= self.rtc_min_tail)
                else:
                    idle = True
                if not idle:
                    self._planning = True
            # 关键：Event.wait 必须在锁外。此前写在 with self._lock 内，空闲时
            # planner 几乎 100% 持有引擎锁（5ms 等待 + 立即重获），控制线程 tick()
            # 在锁上饿死 → queue_async 即使瞬时推理也只有 ~15Hz（被 plan A 暴露）。
            if idle:
                self._stop.wait(0.005)
                continue
            try:
                self._run_plan()
            except Exception as exc:  # noqa: BLE001
                # An unexpected post-processing/programming error must never
                # kill the daemon planner silently.  Preserve the current
                # chunk so its already accepted rows can finish, then make the
                # failure visible through stats/trace; tick() raises as soon as
                # that chunk is exhausted and PolicyNode performs a safe stop.
                error = f"planner failed: {type(exc).__name__}: {exc}"
                with self._lock:
                    self._last_error = error
                self._emit_plan_trace({
                    "schema": "pi_plan_trace/v1",
                    "event": "planner_failed",
                    "t": round(time.time(), 6),
                    "mode": self.mode,
                    "error": error,
                })
                return
            finally:
                with self._lock:
                    self._planning = False
