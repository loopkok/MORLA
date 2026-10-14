"""High-level video service orchestration: signaling + sender + stats loop.

Self-contained. The video source is injected (built by the caller/launch),
unlike the upstream SDK which builds it internally. This lets the same
service push a ROS image topic, a direct webcam, or any other
``VideoSourceAdapter``.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from quest3_video_streamer.schemas import SignalingMessage, make_signaling_message
from quest3_video_streamer.signaling import SignalingConnection, VideoSignalingServer
from quest3_video_streamer.source_base import VideoSourceAdapter
from quest3_video_streamer.webrtc_sender import VideoSenderStats, VideoWebRTCSender


@dataclass(frozen=True, slots=True)
class VideoServiceConfig:
    signaling_host: str = "0.0.0.0"
    signaling_port: int = 8765
    preset: str = "720p30"
    stats_interval_s: float = 1.0
    server_version: str = "0.1.0"
    verbose: bool = False
    log_hook: Callable[[str], None] | None = None
    # 回传降载：0 = 不降载。push_max_width 等比缩分辨率（偶数取整），
    # push_fps 降发送帧率；只影响 WebRTC 软编码负载，采集抽头不受影响。
    push_max_width: int = 0
    push_fps: int = 0
    # 启动即打开所有相机源（不等 Quest 连接）：采集/web 预览不再依赖
    # Quest 视频会话，采集可与推流完全解耦。
    eager_start_sources: bool = False


_PRESET_MAP: dict[str, tuple[int, int, int]] = {
    "480p": (640, 480, 60),
    "480p30": (640, 480, 30),
    "720p": (1280, 720, 60),
    "720p30": (1280, 720, 30),
    "1080p": (1920, 1080, 60),
    "1080p30": (1920, 1080, 30),
}


def parse_preset(preset: str) -> tuple[int, int, int]:
    result = _PRESET_MAP.get(preset.lower())
    if result is None:
        raise ValueError(
            f"Unknown preset {preset!r}; expected one of {list(_PRESET_MAP)}"
        )
    return result


class Quest3VideoService:
    """Owns signaling + sender + source lifecycle for one video session.

    Supports one or more video sources (one outbound WebRTC track each). When
    multiple sources are configured, the Quest must create one recv-only video
    transceiver per source; the ``video_config`` message tells the Quest how
    many tracks to expect and how to lay them out in 3D space.
    """

    def __init__(
        self,
        *,
        sources: list[VideoSourceAdapter],
        layouts: list[dict[str, Any]] | None = None,
        config: VideoServiceConfig,
        gate: Any = None,
    ) -> None:
        if not sources:
            raise ValueError("At least one video source is required.")
        self._sources = list(sources)
        # Optional runtime gate (StreamGate) forwarded to every sender's tracks.
        self._gate = gate
        # Optional per-source display layout (parallel list to sources).
        # Each entry may contain: position [x,y,z], distance, size_multiplier.
        self._layouts = layouts if layouts is not None else [{} for _ in sources]
        self._config = config
        self._signaling = VideoSignalingServer(
            host=config.signaling_host,
            port=config.signaling_port,
            on_message=self._on_message,
            on_connect=self._on_connect,
            on_disconnect=self._on_disconnect,
        )
        self._sender: VideoWebRTCSender | None = None
        self._active_connection: SignalingConnection | None = None
        self._active_session_id: str | None = None
        self._stats_task: asyncio.Task[None] | None = None

    async def start(self) -> None:
        await self._signaling.start()
        self._log(
            f"signaling server listening host={self._config.signaling_host} "
            f"port={self._config.signaling_port}"
        )
        if self._config.eager_start_sources:
            for src in self._sources:
                try:
                    await src.start()
                    self._log(f"eager start: {src.get_format().label} opened")
                except Exception as exc:
                    self._log(f"eager start failed for {src.get_format().label}: {exc}")

    async def stop(self) -> None:
        await self._stop_sender()
        if self._stats_task is not None:
            self._stats_task.cancel()
            self._stats_task = None
        await self._signaling.stop()
        self._active_connection = None
        self._active_session_id = None
        self._log("video service stopped")

    # -- message dispatch --------------------------------------------------

    async def _on_message(self, connection: SignalingConnection, message: SignalingMessage) -> None:
        session_id = message.session_id
        message_type = message.type
        self._log(f"recv type={message_type} session={session_id}")

        if message_type == "hello":
            await self._signaling.send(
                connection,
                make_signaling_message(
                    type="hello_ack",
                    session_id=session_id,
                    payload={"server_version": self._config.server_version},
                ),
            )
            self._log(f"sent hello_ack session={session_id}")
            # Tell the Quest the geometry + layout of every source track so it
            # can size each panel to match the real FOV (fixes magnification)
            # and position multiple cameras in 3D space.
            try:
                tracks_payload = []
                for idx, src in enumerate(self._sources):
                    fmt = src.get_format()
                    layout = self._layouts[idx] if idx < len(self._layouts) else {}
                    tracks_payload.append({
                        "index": idx,
                        "label": fmt.label,
                        "width": fmt.width,
                        "height": fmt.height,
                        "fps": fmt.fps,
                        "fov_h_deg": fmt.fov_h_deg,
                        "layout": {
                            "position": layout.get("position", [0.0, -0.1, 1.8]),
                            "distance": layout.get("distance", 1.8),
                            "size_multiplier": layout.get("size_multiplier", 1.0),
                        },
                    })
                await self._signaling.send(
                    connection,
                    make_signaling_message(
                        type="video_config",
                        session_id=session_id,
                        payload={"tracks": tracks_payload},
                    ),
                )
                self._log(
                    f"sent video_config session={session_id} tracks={len(tracks_payload)} "
                    f"labels={[t['label'] for t in tracks_payload]}"
                )
            except Exception as exc:
                self._log(f"video_config send failed: {exc}")
            return

        if message_type == "ping":
            await self._signaling.send(
                connection,
                make_signaling_message(type="pong", session_id=session_id, payload=message.payload),
            )
            return

        if message_type == "start_video":
            self._active_connection = connection
            self._active_session_id = session_id
            await self._send_video_state(connection, session_id=session_id, state="connecting")
            return

        if message_type == "stop_video":
            await self._stop_sender()
            await self._send_video_state(connection, session_id=session_id, state="stopped")
            return

        if message_type == "offer":
            await self._handle_offer(connection, message)
            return

        if message_type == "ice_candidate":
            await self._handle_remote_ice_candidate(connection, message)
            return

        await self._signaling.send(
            connection,
            make_signaling_message(
                type="error",
                session_id=session_id,
                payload={"code": "unsupported_message", "message": f"Unsupported: {message_type}"},
            ),
        )

    async def _handle_offer(self, connection: SignalingConnection, message: SignalingMessage) -> None:
        session_id = message.session_id
        sdp = str(message.payload.get("sdp", ""))
        if not sdp:
            await self._emit_error(connection, session_id, "missing_offer", "Offer missing 'sdp'.")
            return
        if "m=video" not in sdp:
            await self._emit_error(
                connection, session_id, "invalid_offer",
                "Offer missing m=video; Quest must create a recv-only video transceiver.",
            )
            return
        self._log(f"offer received session={session_id} sdp_len={len(sdp)}")

        if self._sender is None:
            try:
                self._sender = VideoWebRTCSender(
                    sources=self._sources,
                    on_local_ice_candidate=self._make_ice_callback(session_id),
                    log_hook=lambda msg: self._log(f"[sender] {msg}"),
                    gate=self._gate,
                    push_fps=self._config.push_fps,
                    push_max_width=self._config.push_max_width,
                )
                await self._sender.start()
                self._log(f"sender started sources={len(self._sources)} preset={self._config.preset}")
            except Exception as exc:
                await self._emit_error(connection, session_id, "sender_start_failed", str(exc))
                await self._send_video_state(connection, session_id=session_id, state="error", reason="sender_start_failed")
                self._log(f"sender start failed: {exc}")
                return

        try:
            answer_sdp = await self._sender.apply_offer(sdp_offer=sdp)
        except Exception as exc:
            await self._emit_error(connection, session_id, "offer_failed", str(exc))
            await self._send_video_state(connection, session_id=session_id, state="error", reason="offer_failed")
            await self._stop_sender()
            return

        await self._signaling.send(
            connection,
            make_signaling_message(type="answer", session_id=session_id, payload={"sdp": answer_sdp}),
        )
        self._log(f"answer sent session={session_id} sdp_len={len(answer_sdp)}")
        await self._send_video_state(connection, session_id=session_id, state="playing")
        await self.send_track_visibility()
        self._start_stats_loop_if_needed()

    async def _handle_remote_ice_candidate(self, connection: SignalingConnection, message: SignalingMessage) -> None:
        if self._sender is None:
            return
        try:
            await self._sender.add_remote_ice_candidate(
                candidate=str(message.payload.get("candidate", "")),
                sdp_mid=self._to_optional_str(message.payload.get("sdpMid")),
                sdp_mline_index=self._to_optional_int(message.payload.get("sdpMLineIndex")),
            )
            self._log(f"remote ICE applied session={message.session_id}")
        except Exception as exc:
            self._log(f"remote ICE failed: {exc}")

    async def _on_connect(self, connection: SignalingConnection) -> None:
        remote = getattr(connection.websocket, "remote_address", None)
        self._log(f"client connected remote={remote}")

    async def _on_disconnect(self, _: SignalingConnection) -> None:
        await self._stop_sender()
        self._log("client disconnected; sender stopped")

    # -- helpers -----------------------------------------------------------

    def _make_ice_callback(self, session_id: str) -> Callable[[dict[str, Any]], Awaitable[None]]:
        async def _on_candidate(payload: dict[str, Any]) -> None:
            if self._active_connection is None:
                return
            await self._signaling.send(
                self._active_connection,
                make_signaling_message(type="ice_candidate", session_id=session_id, payload=payload),
            )
        return _on_candidate

    def _start_stats_loop_if_needed(self) -> None:
        if self._stats_task is None or self._stats_task.done():
            self._stats_task = asyncio.create_task(self._stats_loop())

    async def _stats_loop(self) -> None:
        while True:
            await asyncio.sleep(self._config.stats_interval_s)
            if (
                self._sender is None
                or self._active_connection is None
                or self._active_session_id is None
            ):
                continue
            try:
                stats = await self._sender.get_stats()
                await self._emit_stats(self._active_connection, self._active_session_id, stats)
                if stats.tracks:
                    parts = " ".join(
                        f"[{t.index}:{t.label} fps={t.fps:.1f} "
                        f"{t.bitrate_kbps:.0f}kbps]" for t in stats.tracks
                    )
                    self._log(
                        f"stats session={self._active_session_id} "
                        f"total_fps={stats.fps:.1f} total_kbps={stats.bitrate_kbps:.0f} "
                        f"rtt_ms={stats.rtt_ms} drops={stats.frame_drops} | {parts}"
                    )
                else:
                    self._log(
                        f"stats session={self._active_session_id} "
                        f"fps={stats.fps:.1f} bitrate_kbps={stats.bitrate_kbps:.1f} "
                        f"drops={stats.frame_drops} rtt_ms={stats.rtt_ms}"
                    )
            except Exception:
                continue

    async def _emit_stats(self, connection: SignalingConnection, session_id: str, stats: VideoSenderStats) -> None:
        await self._signaling.send(
            connection,
            make_signaling_message(
                type="stats",
                session_id=session_id,
                payload={
                    "fps": round(stats.fps, 2),
                    "bitrate_kbps": round(stats.bitrate_kbps, 2),
                    "frame_drops": stats.frame_drops,
                    "rtt_ms": None if stats.rtt_ms is None else round(stats.rtt_ms, 2),
                    "tracks": [
                        {
                            "index": t.index,
                            "label": t.label,
                            "fps": round(t.fps, 2),
                            "bitrate_kbps": round(t.bitrate_kbps, 2),
                        }
                        for t in (stats.tracks or [])
                    ],
                },
            ),
        )

    async def _emit_error(self, connection: SignalingConnection, session_id: str, code: str, message: str) -> None:
        await self._signaling.send(
            connection,
            make_signaling_message(
                type="error", session_id=session_id, payload={"code": code, "message": message},
            ),
        )
        self._log(f"error session={session_id} code={code} message={message}")

    def hook_gate_visibility(self) -> None:
        """Register a thread-safe gate-change hook that re-sends visibility.

        Called by the node from the asyncio thread once the service exists;
        gate changes arrive from the rclpy spin thread, so schedule onto the
        running loop.
        """
        if self._gate is None:
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return

        def _schedule() -> None:
            loop.call_soon_threadsafe(
                lambda: asyncio.ensure_future(self.send_track_visibility())
            )

        self._gate.on_change = _schedule

    async def send_track_visibility(self) -> None:
        """Tell the Quest app which track labels are currently un-muted.

        The app hides panels whose label is absent (muted panels otherwise
        render as black panels). Backward compatible: older app builds log
        and ignore the unknown message type.
        """
        if (
            self._gate is None
            or self._active_connection is None
            or self._active_session_id is None
        ):
            return
        try:
            snap = self._gate.snapshot()
            enabled = list(snap["active"]) if snap["push_enabled"] else []
            await self._signaling.send(
                self._active_connection,
                make_signaling_message(
                    type="track_visibility",
                    session_id=self._active_session_id,
                    payload={"enabled": enabled},
                ),
            )
            self._log(f"track_visibility sent enabled={enabled}")
        except Exception as exc:
            self._log(f"track_visibility send failed: {exc}")

    async def _send_video_state(self, connection: SignalingConnection, *, session_id: str, state: str, reason: str | None = None) -> None:
        payload: dict[str, Any] = {"state": state}
        if reason is not None:
            payload["reason"] = reason
        await self._signaling.send(
            connection,
            make_signaling_message(type="video_state", session_id=session_id, payload=payload),
        )

    async def _stop_sender(self) -> None:
        if self._sender is not None:
            await self._sender.stop()
            self._sender = None
            self._log("sender stopped")

    @staticmethod
    def _to_optional_str(value: Any) -> str | None:
        return None if value is None else str(value)

    @staticmethod
    def _to_optional_int(value: Any) -> int | None:
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _log(self, message: str) -> None:
        if self._config.log_hook is not None:
            self._config.log_hook(message)
