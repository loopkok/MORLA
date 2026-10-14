"""Video source adapters for the Quest 3 WebRTC streamer.

Self-contained. Two concrete sources:

* ``RosImageSourceAdapter`` — subscribes to a ``sensor_msgs/Image`` topic
  (e.g. RealSense D435i color stream from ``realsense2_camera``, or a USB
  camera published by ``usb_cam`` / ``v4l2_camera``).
* ``WebcamSourceAdapter`` — reads a V4L2 USB camera directly with OpenCV
  (fallback when no ROS camera driver is desired / available).

Both expose the same async ``VideoSourceAdapter`` protocol consumed by
``VideoWebRTCSender``.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class VideoFormat:
    width: int
    height: int
    fps: int
    # Horizontal field of view of the source camera, in degrees. The Quest uses
    # this to size the display panel so the streamed image appears at natural
    # scale (1:1) instead of magnified. D435i color ~69°, typical USB webcam ~60°.
    fov_h_deg: float = 69.0
    # Human-readable label identifying this source (e.g. "d435i", "wrist_left").
    label: str = "camera"


class VideoSourceAdapter:
    """Structural base for source adapters consumed by the WebRTC sender."""

    async def start(self) -> None: ...
    async def stop(self) -> None: ...
    async def next_frame(self) -> Any: ...
    def get_format(self) -> VideoFormat: ...


