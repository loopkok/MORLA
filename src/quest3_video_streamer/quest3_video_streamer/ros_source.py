"""ROS ``sensor_msgs/Image`` source adapter.

Bridges a ROS 2 image topic (push/callback based) into the pull-based
``next_frame()`` API used by the WebRTC sender. The rclpy subscription runs in
the rclpy executor thread; frames are handed to the asyncio loop thread-safely
via ``loop.call_soon_threadsafe``.

No ``cv_bridge`` dependency (it has a numpy ABI mismatch on this system); we
convert ``sensor_msgs/Image`` -> ``numpy`` -> ``av.VideoFrame`` by hand.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

from quest3_video_streamer.source_base import VideoFormat, VideoSourceAdapter

_LOG = logging.getLogger("quest3_video_streamer.ros_source")


class RosImageSourceAdapter(VideoSourceAdapter):
    """Subscribe to a sensor_msgs/Image topic and feed frames to WebRTC."""

    def __init__(
        self,
        *,
        node: Any,
        topic: str,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        fov_h_deg: float = 69.0,
        label: str = "ros_camera",
    ) -> None:
        self._format = VideoFormat(
            width=width, height=height, fps=fps,
            fov_h_deg=fov_h_deg, label=label,
        )
        self._loop: asyncio.AbstractEventLoop | None = None
        self._queue: asyncio.Queue[Any] = asyncio.Queue(maxsize=1)
        self._latest: Any = None
        self._node = node
        self._topic = topic
        self._sub = None
        self._frame_count = 0
        # Optional tap for the web preview pipeline: callable(rgb_frame, timestamp_ns).
        # Called from the rclpy executor thread; must be non-blocking.
        self.preview_hook: Any = None
        self._preview_hook_error_reported = False
        # Optional tap for the data-collection pipeline: callable(rgb_frame, timestamp_ns).
        # Same contract as preview_hook (executor thread, must not block).
        self.collect_hook: Any = None
        self.diagnostic_hook: Any = None
        self._diag_t0 = time.monotonic()
        self._diag_frames = 0
        self._diag_last_frame: float | None = None
        self._diag_max_gap_ms = 0.0

    async def start(self) -> None:
        if self._sub is not None:
            return  # 幂等：eager start 后 track 的 lazy open 不再重复订阅
        # Called from the asyncio loop thread: capture the loop so the rclpy
        # callback (running in a different thread) can schedule puts safely.
        self._loop = asyncio.get_running_loop()
        # Defer the rclpy import/subscription creation to here so the adapter
        # can be constructed before rclpy is fully spun.
        from sensor_msgs.msg import Image as ImageMsg
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        # Camera drivers commonly publish best-effort; use a sensor-data style
        # QoS but allow reliable if the driver prefers it.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        try:
            self._sub = self._node.create_subscription(
                ImageMsg, self._topic, self._on_image, qos
            )
        except Exception:
            # Fall back to the default sensor data QoS.
            from rclpy.qos import qos_profile_sensor_data
            self._sub = self._node.create_subscription(
                ImageMsg, self._topic, self._on_image, qos_profile_sensor_data
            )

    async def stop(self) -> None:
        if self._sub is not None:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None

    def get_format(self) -> VideoFormat:
        return self._format

    async def next_frame(self) -> Any:
        import av

        frame = await self._queue.get()
        if frame is None:
            return None
        # frame is a contiguous uint8 numpy array in RGB order, sized to the
        # target format.
        return av.VideoFrame.from_ndarray(frame, format="rgb24")

    # -- rclpy callback (runs in the rclpy executor thread) -----------------

    def _on_image(self, msg: Any) -> None:
        source_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if source_ns <= 0:
            source_ns = time.time_ns()
        try:
            rgb = self._convert(msg)
        except Exception:
            return
        if rgb is None:
            return
        self._diag_frames += 1
        now = time.monotonic()
        if self._diag_last_frame is not None:
            self._diag_max_gap_ms = max(
                self._diag_max_gap_ms,
                (now - self._diag_last_frame) * 1000.0,
            )
        self._diag_last_frame = now
        dt = now - self._diag_t0
        if dt >= 5.0:
            diagnostic_hook = self.diagnostic_hook
            if diagnostic_hook is not None:
                try:
                    diagnostic_hook({
                        "stage": "capture",
                        "label": self._format.label,
                        "source": "ros",
                        "window_s": round(dt, 3),
                        "frames": self._diag_frames,
                        "fps": round(self._diag_frames / dt, 3),
                        "read_failures": 0,
                        "max_frame_gap_ms": round(self._diag_max_gap_ms, 3),
                    })
                except Exception:
                    pass
            self._diag_t0 = now
            self._diag_frames = 0
            self._diag_max_gap_ms = 0.0
        hook = self.preview_hook
        if hook is not None:
            try:
                hook(rgb, source_ns)
            except Exception as exc:
                if not self._preview_hook_error_reported:
                    _LOG.warning("preview hook failed for %s: %s", self._topic, exc)
                    self._preview_hook_error_reported = True
        collect_hook = self.collect_hook
        if collect_hook is not None:
            try:
                collect_hook(rgb, source_ns)
            except Exception:
                pass
        if self._loop is None:
            return
        self._frame_count += 1
        # Keep only the freshest frame: schedule a put on the asyncio loop.
        self._latest = rgb
        self._loop.call_soon_threadsafe(self._threadsafe_put, rgb)

    def _threadsafe_put(self, rgb: Any) -> None:
        # Runs on the asyncio loop thread -> asyncio.Queue ops are safe.
        try:
            self._queue.put_nowait(rgb)
        except asyncio.QueueFull:
            # Drop the oldest to make room for the newest (low-latency over
            # completeness for live teleop video).
            try:
                self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            try:
                self._queue.put_nowait(rgb)
            except asyncio.QueueFull:
                pass

    def _convert(self, msg: Any) -> Any:
        import numpy as np

        h = int(msg.height)
        w = int(msg.width)
        enc = str(msg.encoding)
        # msg.data may be a memoryview/array.array; frombuffer gives a view,
        # .copy() makes it writable for av/cv2.
        arr = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ("rgb8", "RGB8"):
            rgb = arr.reshape(h, w, 3)
        elif enc in ("bgr8", "BGR8"):
            rgb = arr.reshape(h, w, 3)[:, :, ::-1]
        elif enc in ("rgba8", "RGBA8"):
            rgb = arr.reshape(h, w, 4)[:, :, :3]
        elif enc in ("bgra8", "BGRA8"):
            rgb = arr.reshape(h, w, 4)[:, :, :3][:, :, ::-1]
        elif enc in ("yuyv", "YUYV", "uyvy", "UYVY"):
            import cv2

            yuv = arr.reshape(h, w, 2)
            code = cv2.COLOR_YUV2RGB_YUYV if enc.lower() == "yuyv" else cv2.COLOR_YUV2RGB_UYVY
            rgb = cv2.cvtColor(yuv, code)
        elif enc in ("mjpeg", "MJPEG"):
            import cv2

            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return None
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        elif enc in ("mono8", "MONO8"):
            mono = arr.reshape(h, w, 1)
            rgb = np.repeat(mono, 3, axis=2)
        else:
            # Best-effort: assume 3-channel packed.
            try:
                rgb = arr.reshape(h, w, 3)
            except Exception:
                return None

        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)

        tw, th = self._format.width, self._format.height
        if (w, h) != (tw, th):
            import cv2

            rgb = cv2.resize(rgb, (tw, th), interpolation=cv2.INTER_LINEAR)
            rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        return rgb
