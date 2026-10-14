"""Low-cost web preview publisher (JPEG over CompressedImage).

Taps frames that the capture path already produces and republishes them as
``sensor_msgs/CompressedImage`` at a throttled rate so the web monitor can
show live previews alongside the Quest3 WebRTC feed.

Latency isolation (the whole point):

  * ``submit()`` runs in the producer thread (capture thread / rclpy
    callback). It only does a gate check + rate check + ``put_nowait`` on a
    maxsize-1 queue — never blocks, drops when busy.
  * JPEG encoding runs in a dedicated daemon thread, NEVER in the capture
    thread (would stretch the capture period) and NEVER in the asyncio loop
    (would jitter the Quest RTP pacing).
  * Publishing follows the runtime gate: a muted camera publishes nothing.

Default cost: ~10 fps x 640-wide q65 JPEG ≈ a few % of one core per camera.
"""

from __future__ import annotations

import array
import logging
import queue
import threading
import time
from typing import Any

_LOG = logging.getLogger("quest3_video_streamer.preview")


class PreviewPublisher:
    """Per-camera throttled JPEG publisher on ``~/preview/{label}``."""

    def __init__(
        self,
        *,
        node: Any,
        label: str,
        gate: Any = None,
        fps: float = 10.0,
        width: int = 640,
        quality: int = 65,
    ) -> None:
        from sensor_msgs.msg import CompressedImage
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self._node = node
        self._label = label
        self._gate = gate
        self._period = 1.0 / max(0.5, fps)
        self._width = max(64, width)
        self._quality = int(quality)
        self._last_submit = 0.0
        self._worker_error_reported = False
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._msg_type = CompressedImage
        self._pub = node.create_publisher(
            CompressedImage, f"~/preview/{label}", qos
        )
        self._thread = threading.Thread(
            target=self._run, name=f"preview-{label}", daemon=True
        )
        self._thread.start()

    def submit(
        self,
        frame: Any,
        *,
        is_rgb: bool,
        timestamp_ns: int | None = None,
    ) -> None:
        """Offer a frame (numpy HxWx3). Non-blocking; drops when busy."""
        if self._gate is not None and not self._gate.is_enabled(self._label):
            return
        now = time.monotonic()
        if now - self._last_submit < self._period:
            return
        self._last_submit = now
        try:
            if timestamp_ns is None:
                timestamp_ns = self._node.get_clock().now().nanoseconds
            self._queue.put_nowait((frame, is_rgb, int(timestamp_ns)))
        except queue.Full:
            pass

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_publisher(self._pub)
        except Exception:
            pass

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
                h, w = frame.shape[:2]
                if w > self._width:
                    scale = self._width / float(w)
                    frame = cv2.resize(
                        frame, (self._width, max(1, int(h * scale))),
                        interpolation=cv2.INTER_AREA,
                    )
                ok, jpg = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
                )
                if not ok:
                    continue
                msg = self._msg_type()
                msg.header.stamp.sec = timestamp_ns // 1_000_000_000
                msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
                msg.header.frame_id = self._label
                msg.format = "jpeg"
                # array.array 快路径，原因见 collect_tap（bytes 会触发
                # rosidl setter 的逐字节 Python 校验，持 GIL 数百 ms/帧）。
                msg.data = array.array("B", jpg.tobytes())
                self._pub.publish(msg)
            except Exception as exc:
                if not self._worker_error_reported:
                    _LOG.warning("preview encode/publish failed for %s: %s", self._label, exc)
                    self._worker_error_reported = True
