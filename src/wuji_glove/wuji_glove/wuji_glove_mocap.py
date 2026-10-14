#!/usr/bin/env python3
"""Wuji Glove → hand_landmarks/{left,right} (PoseArray).

Output contract matches quest3_hand_mocap so retarget nodes can switch inputs
without topic changes.

Parameter names align with rob_station env vars where applicable:

  ROS param              rob_station env           notes
  ---------------------  ------------------------  -------------------------
  hand_side              GLOVE_HAND_SIDE           left|right|both (rob_station: left|right)
  sn                     GLOVE_HAND_SN             single-hand SN; empty = auto UDP scan
  device_name            (CLI --device-name)       default "glove"
  left_sn / right_sn     —                         both 模式按侧 SN（优先于 sn）
  publish_rate           (loop-fps≈50)             default 50.0 Hz
  ema_alpha              —                         与 quest3 一致，默认 0.7

Hand-control-only rob_station vars are NOT used here (GLOVE_HAND_MODEL,
GLOVE_HAND_SERIAL, GLOVE_HAND_FILTER_HZ) — those belong to the hand driver /
retarget packages.
"""

from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from wuji_glove.fps_counter import FPSCounter

# Reconnect if no skeleton for this long (same idea as wuji-hand-teleop).
_RECV_TIMEOUT_SEC = 2.0


def _env_or_default(name: str, default: str) -> str:
    val = os.environ.get(name)
    if val is None or str(val).strip() == "":
        return default
    return str(val).strip()


def _extract_keypoints(skeleton) -> Optional[np.ndarray]:
    joints = skeleton.joints
    if len(joints) != 21:
        return None
    kp = np.array([j.pose.position for j in joints], dtype=np.float32)
    if kp.shape != (21, 3):
        return None
    return kp


class _GloveSideSession:
    """One glove connection + skeleton subscription for a single hand side."""

    def __init__(self, side: str, sn: str, device_name: str, logger):
        self.side = side
        self.sn = sn.strip() if sn else ""
        self.device_name = device_name or f"{side}_glove"
        self.logger = logger
        self.device = None
        self.sub = None
        self.last_recv_time = 0.0
        self.reconnect_attempts = 0

    def connect(self) -> bool:
        from wuji_sdk import ConnectOptions, SdkManager

        self.release()
        manager = SdkManager.instance()
        opts = ConnectOptions(enable_bridge=False)
        try:
            if self.sn:
                device = manager.connect(
                    sn=self.sn, device_name=self.device_name, options=opts
                )
            else:
                scanned = manager.scan()
                udp_sns = [
                    d.sn
                    for d in scanned
                    if "udp" in str(d.transport_type).lower()
                ]
                if not udp_sns:
                    raise RuntimeError(
                        "未扫描到 UDP 手套。请开机并完成 Studio 标定，"
                        f"或设置 sn / GLOVE_HAND_SN。scan="
                        f"{[(d.sn, str(d.transport_type)) for d in scanned]}"
                    )
                device = manager.connect(
                    sn=udp_sns[0], device_name=self.device_name, options=opts
                )
                self.sn = udp_sns[0]
        except Exception as exc:  # noqa: BLE001
            self.reconnect_attempts += 1
            if self.reconnect_attempts == 1 or self.reconnect_attempts % 10 == 0:
                self.logger.warn(
                    f"[{self.side}] wuji_sdk connect #{self.reconnect_attempts} "
                    f"failed: {exc}"
                )
            return False

        actual_side = device.hand_side().get().lower()
        if actual_side != self.side:
            self.release()
            raise RuntimeError(
                f"[{self.side}] SN={self.sn} reports hand_side={actual_side}; "
                f"请交换 left_sn/right_sn 或检查 GLOVE_HAND_SIDE。"
            )

        self.device = device
        self.sub = device.hand_skeleton().subscribe()
        self.last_recv_time = time.monotonic()
        was_retry = self.reconnect_attempts > 0
        self.reconnect_attempts = 0
        self.logger.info(
            f"[{self.side}] wuji_sdk {'re' if was_retry else ''}connected "
            f"SN={self.sn} device_name={self.device_name}"
        )
        return True

    def poll_keypoints(self) -> Optional[np.ndarray]:
        """Drain skeleton queue; reconnect on timeout. Returns (21,3) or None."""
        now = time.monotonic()
        if self.sub is None:
            self.connect()
            return None

        skeleton = self.sub.recv()
        if skeleton is None:
            if now - self.last_recv_time > _RECV_TIMEOUT_SEC:
                self.logger.warn(
                    f"[{self.side}] no skeleton for "
                    f"{now - self.last_recv_time:.1f}s, reconnecting..."
                )
                self.connect()
            return None

        self.last_recv_time = now
        while True:
            newer = self.sub.recv()
            if newer is None:
                break
            skeleton = newer

        return _extract_keypoints(skeleton)

    def release(self) -> None:
        self.sub = None
        if self.device is not None:
            try:
                self.device.disconnect()
            except Exception:  # noqa: BLE001
                pass
        self.device = None


class WujiGloveMocap(Node):
    def __init__(self):
        super().__init__("wuji_glove_mocap")

        # ---- params (defaults follow rob_station / quest3) ----
        default_side = _env_or_default("GLOVE_HAND_SIDE", "right")
        default_sn = _env_or_default("GLOVE_HAND_SN", "")

        self.declare_parameter("hand_side", default_side)  # left|right|both
        self.declare_parameter("sn", default_sn)
        self.declare_parameter("device_name", "glove")
        self.declare_parameter("left_sn", "")
        self.declare_parameter("right_sn", "")
        self.declare_parameter("left_device_name", "left_glove")
        self.declare_parameter("right_device_name", "right_glove")
        self.declare_parameter("publish_rate", 50.0)  # rob_station loop-fps
        self.declare_parameter("ema_alpha", 0.7)
        self.declare_parameter("viz", False)
        self.declare_parameter("output_topic", "hand_landmarks")
        self.declare_parameter("fps_print_interval", 5.0)

        self.hand_side = str(self.get_parameter("hand_side").value).lower()
        if self.hand_side not in ("left", "right", "both"):
            raise ValueError(
                f"hand_side 须为 left|right|both，收到: {self.hand_side}"
            )

        self.sn = str(self.get_parameter("sn").value).strip()
        self.device_name = str(self.get_parameter("device_name").value).strip() or "glove"
        self.left_sn = str(self.get_parameter("left_sn").value).strip()
        self.right_sn = str(self.get_parameter("right_sn").value).strip()
        self.left_device_name = (
            str(self.get_parameter("left_device_name").value).strip() or "left_glove"
        )
        self.right_device_name = (
            str(self.get_parameter("right_device_name").value).strip() or "right_glove"
        )
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.viz = bool(self.get_parameter("viz").value)
        self.output_topic = str(self.get_parameter("output_topic").value).rstrip("/")

        if self.publish_rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0, got {self.publish_rate}")

        self.get_logger().info(
            f"Wuji Glove Mocap: hand_side={self.hand_side}, "
            f"rate={self.publish_rate:.1f}Hz, topic={self.output_topic}/{{left,right}}, "
            f"sn={self.sn or '(auto/per-side)'}, ema_alpha={self.ema_alpha}"
        )

        # Publishers — same names as quest3_hand_mocap
        self.pub: Dict[str, Any] = {
            "right": self.create_publisher(
                PoseArray, f"{self.output_topic}/right", 10
            ),
            "left": self.create_publisher(
                PoseArray, f"{self.output_topic}/left", 10
            ),
        }
        self.marker_pub: Dict[str, Any] = {}
        if self.viz:
            self.marker_pub["right"] = self.create_publisher(
                MarkerArray, "wuji_glove/right_hand_markers", 10
            )
            self.marker_pub["left"] = self.create_publisher(
                MarkerArray, "wuji_glove/left_hand_markers", 10
            )

        self._ema_cache = {
            "left": np.zeros((21, 3), dtype=np.float64),
            "right": np.zeros((21, 3), dtype=np.float64),
        }
        self._ema_initialized = {"left": False, "right": False}

        fps_interval = float(self.get_parameter("fps_print_interval").value)
        self._fps: Dict[str, FPSCounter] = {
            "left": FPSCounter(window=100, print_interval=fps_interval),
            "right": FPSCounter(window=100, print_interval=fps_interval),
        }

        self._sessions: Dict[str, _GloveSideSession] = {}
        sides = ("left", "right") if self.hand_side == "both" else (self.hand_side,)
        for side in sides:
            sn = self._resolve_sn(side)
            dev = self._resolve_device_name(side)
            self._sessions[side] = _GloveSideSession(
                side=side, sn=sn, device_name=dev, logger=self.get_logger()
            )

        # First connect (non-fatal if offline — timer keeps retrying)
        for side, sess in self._sessions.items():
            try:
                sess.connect()
            except RuntimeError as exc:
                # Config error (side mismatch) should fail fast
                self.get_logger().error(str(exc))
                raise

        self.create_timer(1.0 / self.publish_rate, self._tick)

    def _resolve_sn(self, side: str) -> str:
        if side == "left" and self.left_sn:
            return self.left_sn
        if side == "right" and self.right_sn:
            return self.right_sn
        # Single-hand mode: use sn / GLOVE_HAND_SN
        if self.hand_side != "both":
            return self.sn
        # both + empty per-side: leave empty → auto scan (first UDP each connect)
        return self.sn if self.sn else ""

    def _resolve_device_name(self, side: str) -> str:
        if self.hand_side == "both":
            return self.left_device_name if side == "left" else self.right_device_name
        # rob_station default device_name="glove"
        return self.device_name

    def _tick(self) -> None:
        for side, sess in self._sessions.items():
            try:
                kp = sess.poll_keypoints()
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                continue
            if kp is None:
                continue
            self._publish_landmarks(kp, side)

    def _publish_landmarks(self, landmarks: np.ndarray, side: str) -> None:
        lm = np.asarray(landmarks, dtype=np.float64)
        if self.ema_alpha < 1.0:
            if not self._ema_initialized[side]:
                self._ema_cache[side] = lm.copy()
                self._ema_initialized[side] = True
            else:
                a = self.ema_alpha
                self._ema_cache[side] = a * lm + (1.0 - a) * self._ema_cache[side]
            lm = self._ema_cache[side]

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"hand_{side}"
        msg.poses = [
            Pose(
                position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            for p in lm
        ]
        self.pub[side].publish(msg)

        fps = self._fps[side].tick()
        if self._fps[side].should_print():
            self.get_logger().info(f"[Glove FPS][{side}] {fps:.0f} Hz")

        if self.viz and side in self.marker_pub:
            self._publish_markers(lm, side)

    def _publish_markers(self, landmarks: np.ndarray, side: str) -> None:
        ma = MarkerArray()
        for i, p in enumerate(landmarks):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = f"hand_{side}"
            m.ns = f"wuji_glove_{side}"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(p[0])
            m.pose.position.y = float(p[1])
            m.pose.position.z = float(p[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.008
            m.color.a = 1.0
            if side == "right":
                m.color.r, m.color.g, m.color.b = 0.2, 0.8, 1.0
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.2
            ma.markers.append(m)
        self.marker_pub[side].publish(ma)

    def destroy_node(self):
        for sess in self._sessions.values():
            sess.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WujiGloveMocap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
