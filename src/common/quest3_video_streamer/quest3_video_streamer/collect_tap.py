"""Full-quality frame tap for offline data collection (JPEG over CompressedImage).

Companion to ``preview.py``: same thread model, but full resolution at (near)
native frame rate, for consumption by ``morla_data_collect`` recording nodes.

Key differences from the web preview:

  * No downscale and high JPEG quality (default q90) — pixels are training data.
  * Encode-on-demand: frames are only queued when the topic has at least one
    subscriber, so a teleop-only run pays exactly zero encode cost.
  * Rate limit is a ceiling, not a throttle target: default 30 fps matches the
    dataset grid; set higher/lower per deployment.

Latency isolation (same contract as preview):

  * ``submit()`` runs in the producer thread (capture thread / rclpy
    callback). It only does subscriber + rate checks and a ``put_nowait``
    on a maxsize-1 queue — never blocks, drops when busy.
  * JPEG encoding runs in a dedicated daemon thread, NEVER in the capture
    thread and NEVER in the asyncio loop.
  * Publishing is decoupled from the runtime push gate: data collection
    works whether or not anyone is watching (Quest / web preview off).
    The only admission condition is "topic has a subscriber" — the
    recorder decides what to keep by its own state machine.
"""

from __future__ import annotations

import array
import logging
import queue
import threading
import time
from typing import Any

_LOG = logging.getLogger("quest3_video_streamer.collect_tap")


class CollectTapPublisher:
    """Per-camera full-res JPEG publisher on ``~/collect/{label}``."""

    def __init__(
        self,
        *,
        node: Any,
        label: str,
        max_fps: float = 30.0,
        quality: int = 90,
        diagnostic_hook: Any = None,
    ) -> None:
        from sensor_msgs.msg import CompressedImage
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self._node = node
        self._label = label
        self._max_fps = max(0.5, max_fps)
        self._quality = int(quality)
        self._diagnostic_hook = diagnostic_hook
        # 令牌桶限流（容量 2）：源帧率==目标时抖动被吸收、长跑 100% 透传；
        # 源更快时仍限在目标附近。固定间隔硬卡会在源≈目标时误杀早到帧
        # （实机：30/s 到达只放行 19-28/s）。
        self._tokens = 2.0
        self._last_refill = time.monotonic()
        # 边界插桩：帧在哪一段丢的（准入/限流/队列/编码/发布），5s 窗口 INFO。
        # 实测事故定位用：driver 侧 30fps 而 collect 话题 3fps 时，看这里。
        self._n_submitted = 0
        self._n_rate_skip = 0
        self._n_queue_full = 0
        self._n_encoded = 0
        self._n_published = 0
        self._encode_ms_sum = 0.0
        self._publish_ms_sum = 0.0
        self._last_submit_t: float | None = None
        self._max_submit_gap_ms = 0.0
        self._last_publish_t: float | None = None
        self._max_publish_gap_ms = 0.0
        self._stat_t0 = time.monotonic()
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._msg_type = CompressedImage
        self._pub = node.create_publisher(
            CompressedImage, f"~/collect/{label}", qos
        )
        self._thread = threading.Thread(
            target=self._run, name=f"collect-tap-{label}", daemon=True
        )
        self._thread.start()

    def submit(self, frame: Any, *, is_rgb: bool, timestamp_ns: int | None = None) -> None:
        """Offer a frame (numpy HxWx3). Non-blocking; drops when busy.

        No push-gate check on purpose: collection must not depend on the
        Quest/web "push" switch.  Encode cost is already on-demand via the
        subscriber-count check below.
        """
        if self._pub.get_subscription_count() == 0:
            return
        self._n_submitted += 1
        now = time.monotonic()
        if self._last_submit_t is not None:
            self._max_submit_gap_ms = max(
                self._max_submit_gap_ms,
                (now - self._last_submit_t) * 1000.0,
            )
        self._last_submit_t = now
        self._tokens = min(2.0, self._tokens + (now - self._last_refill) * self._max_fps)
        self._last_refill = now
        if self._tokens < 1.0:
            self._n_rate_skip += 1
            return
        self._tokens -= 1.0
        try:
            if timestamp_ns is None:
                timestamp_ns = self._node.get_clock().now().nanoseconds
            self._queue.put_nowait((frame, is_rgb, timestamp_ns))
        except queue.Full:
            self._n_queue_full += 1

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_publisher(self._pub)
        except Exception:
            pass

    def _log_stats(self) -> None:
        """5s 窗口边界计数——帧到底丢在准入/限流/队列/编码/发布哪一段。"""
        now = time.monotonic()
        dt = now - self._stat_t0
        if dt < 5.0:
            return
        enc = self._encode_ms_sum / self._n_encoded if self._n_encoded else 0.0
        pub = self._publish_ms_sum / self._n_published if self._n_published else 0.0
        _LOG.info(
            "[tap %s] submit=%.1f/s rate_skip=%d queue_full=%d "
            "encoded=%.1f/s published=%.1f/s encode=%.1fms publish=%.1fms",
            self._label,
            self._n_submitted / dt,
            self._n_rate_skip,
            self._n_queue_full,
            self._n_encoded / dt,
            self._n_published / dt,
            enc,
            pub,
        )
        if self._diagnostic_hook is not None:
            try:
                self._diagnostic_hook({
                    "stage": "collect_tap",
                    "label": self._label,
                    "window_s": round(dt, 3),
                    "submitted_fps": round(self._n_submitted / dt, 3),
                    "rate_skip": self._n_rate_skip,
                    "queue_full": self._n_queue_full,
                    "encoded_fps": round(self._n_encoded / dt, 3),
                    "published_fps": round(self._n_published / dt, 3),
                    "max_submit_gap_ms": round(self._max_submit_gap_ms, 3),
                    "max_publish_gap_ms": round(self._max_publish_gap_ms, 3),
                    "encode_ms": round(enc, 3),
                    "publish_ms": round(pub, 3),
                })
            except Exception:
                pass
        self._n_submitted = self._n_rate_skip = self._n_queue_full = 0
        self._n_encoded = self._n_published = 0
        self._encode_ms_sum = self._publish_ms_sum = 0.0
        self._max_submit_gap_ms = self._max_publish_gap_ms = 0.0
        self._stat_t0 = now

    # -- encoder thread ------------------------------------------------------

    def _run(self) -> None:
        import cv2

        while not self._stop.is_set():
            try:
                frame, is_rgb, timestamp_ns = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if is_rgb:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                t0 = time.monotonic()
                ok, jpg = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
                )
                if not ok:
                    continue
                self._n_encoded += 1
                self._encode_ms_sum += (time.monotonic() - t0) * 1000.0
                msg = self._msg_type()
                msg.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
                msg.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
                msg.header.frame_id = self._label
                msg.format = "jpeg"
                # 必须走 array.array 快路径：rosidl 的 data setter 对 bytes 会
                # 用 Python genexpr 逐字节校验两遍（200KB JPEG ≈ 40 万次迭代，
                # 持 GIL 数百 ms），py-spy 实锤三路 tap 全卡在这里拖垮全进程。
                msg.data = array.array("B", jpg.tobytes())
                t0 = time.monotonic()
                self._pub.publish(msg)
                self._n_published += 1
                published_t = time.monotonic()
                if self._last_publish_t is not None:
                    self._max_publish_gap_ms = max(
                        self._max_publish_gap_ms,
                        (published_t - self._last_publish_t) * 1000.0,
                    )
                self._last_publish_t = published_t
                self._publish_ms_sum += (time.monotonic() - t0) * 1000.0
            except Exception:
                continue
            self._log_stats()
