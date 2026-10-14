"""Host-side WebRTC sender for one outbound H.264/VP8 video track.

Self-contained (vendored, no hand-tracking-sdk dependency).

Includes a runtime patch that raises aiortc's H264/VP8 encoder bitrate floor so
that the Quest receiver's REMB feedback cannot drag the bitrate down to the
default ~0.5-1 Mbps (which makes 720p/1080p look blocky).
"""

from __future__ import annotations

import asyncio
import fractions
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from time import monotonic
from typing import Any


# ---------------------------------------------------------------------------
# Runtime bitrate patch.
#
# aiortc's H264 encoder defaults to 1 Mbps (DEFAULT_BITRATE), VP8 to 500 kbps,
# and both clamp to low ceilings (H264: 3 Mbps, VP8: 1.5 Mbps).  On a LAN the
# Quest's REMB feedback would otherwise cap the encoder at ~1.5 Mbps, making the
# image blocky.  We raise the floor and ceiling so the encoder stays crisp.
# ---------------------------------------------------------------------------
_BITRATE_MIN = 3_000_000      # 3 Mbps floor (720p clear)
_BITRATE_DEFAULT = 5_000_000  # 5 Mbps start (1080p clear)
_BITRATE_MAX = 12_000_000     # 12 Mbps ceiling (1080p crisp)

try:
    import aiortc.codecs.h264 as _h264
    _h264.DEFAULT_BITRATE = _BITRATE_DEFAULT
    _h264.MIN_BITRATE = _BITRATE_MIN
    _h264.MAX_BITRATE = _BITRATE_MAX
except Exception:
    pass

try:
    import aiortc.codecs.vpx as _vpx
    _vpx.DEFAULT_BITRATE = _BITRATE_DEFAULT
    _vpx.MIN_BITRATE = _BITRATE_MIN
    _vpx.MAX_BITRATE = _BITRATE_MAX
except Exception:
    pass


# ---------------------------------------------------------------------------
# x264 preset patch.
#
# aiortc's H264Encoder sets tune=zerolatency but never sets x264 `preset`,
# so libx264 runs its default "medium" — on Jetson-class ARM CPUs three
# tracks at medium starve the whole process (实测：采集抽头被挤到个位数 fps，
# 相机捕获线程同步掉速）。preset 只影响 CPU/压缩率权衡，不影响解码兼容性；
# LAN + 3-12 Mbps 码率下 veryfast 与 medium 画质差异不可察觉。
# 做法与上面的码率补丁一致：包一层 _encode_frame，codec 未建时先按
# aiortc 原逻辑建 codec 但追加 preset。aiortc 内部结构若变则静默回退。
# ---------------------------------------------------------------------------
_X264_PRESET = "veryfast"
_X264_THREADS = "2"
_H264_PATCHED = False
_VP8_PATCHED = False


def encoder_patch_status() -> str:
    """启动时打一行自证——补丁静默回退时（aiortc 结构变化）能立刻看出来。"""
    return f"h264_preset={'on' if _H264_PATCHED else 'OFF'} vp8_cpu_used={'on' if _VP8_PATCHED else 'OFF'}"


try:
    import aiortc.codecs.h264 as _h264_pres

    _orig_h264_encode_frame = _h264_pres.H264Encoder._encode_frame

    def _h264_encode_frame_with_preset(self, frame, force_keyframe=False):
        if self.codec is None:
            import fractions as _fractions

            import av as _av

            self.codec = _av.CodecContext.create("libx264", "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = self.target_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.framerate = _fractions.Fraction(_h264_pres.MAX_FRAME_RATE, 1)
            self.codec.time_base = _fractions.Fraction(1, _h264_pres.MAX_FRAME_RATE)
            self.codec.options = {
                "level": "31",
                "tune": "zerolatency",
                "preset": _X264_PRESET,
                # x264 默认 threads=1.5×核数：12 核 Jetson 上每编码器 18 线程，
                # 3 路 = 54 线程在弱 ARM 核上convoy（实机：流从 ~3fps 塌缩到 0）。
                # 每路限 2 线程，3 路共 6，给捕获/采集线程留核。
                "threads": _X264_THREADS,
            }
            self.codec.profile = "Baseline"
        return _orig_h264_encode_frame(self, frame, force_keyframe)

    if getattr(_h264_pres.H264Encoder._encode_frame, "__name__", "") != "_h264_encode_frame_with_preset":
        _h264_pres.H264Encoder._encode_frame = _h264_encode_frame_with_preset
    _H264_PATCHED = True
except Exception:
    pass


# ---------------------------------------------------------------------------
# VP8 cpu-used patch.
#
# 实机：Quest(Unity WebRTC) 的 offer 不含可用 H264 → 实际协商 VP8。aiortc 给
# libvpx 设 cpu-used="-6"——负值比默认更慢（画质向），桌面无感，Jetson 上
# 三路 540p 直接吃掉所有核（实测每路发送仅 1-2fps，采集/捕获线程全被饿死）。
# realtime 模式下 cpu-used 0..16 越大越快；8 是速度/画质平衡点。
# 手法同 H264 补丁：codec 未建时先按原逻辑建（镜像 1.15 的 options，
# 只改 cpu-used），结构不符则静默回退到原实现。
# ---------------------------------------------------------------------------
_VP8_CPU_USED = "8"

try:
    import aiortc.codecs.vpx as _vpx_pres

    _orig_vp8_encode = _vpx_pres.Vp8Encoder.encode

    def _vp8_encode_fast(self, frame, force_keyframe=False):
        if self.codec is None:
            import av as _av

            self.codec = _av.CodecContext.create("libvpx", "w")
            self.codec.width = frame.width
            self.codec.height = frame.height
            self.codec.bit_rate = self.target_bitrate
            self.codec.pix_fmt = "yuv420p"
            self.codec.gop_size = 3000  # kf_max_dist
            self.codec.qmin = 2  # rc_min_quantizer
            self.codec.qmax = 56  # rc_max_quantizer
            self.codec.options = {
                "bufsize": str(self.target_bitrate),
                "cpu-used": _VP8_CPU_USED,
                "deadline": "realtime",
                "lag-in-frames": "0",
                "minrate": str(self.target_bitrate),
                "maxrate": str(self.target_bitrate),
                "noise-sensitivity": "4",
                "overshoot-pct": "15",
                "partitions": "0",  # VP8_ONE_TOKENPARTITION
                "static-thresh": "1",
                "undershoot-pct": "100",
            }
            try:
                self.codec.thread_count = _vpx_pres.number_of_threads(
                    frame.width * frame.height,
                    _vpx_pres.multiprocessing.cpu_count(),
                )
            except Exception:
                pass
        return _orig_vp8_encode(self, frame, force_keyframe)

    if getattr(_vpx_pres.Vp8Encoder.encode, "__name__", "") != "_vp8_encode_fast":
        _vpx_pres.Vp8Encoder.encode = _vp8_encode_fast
    _VP8_PATCHED = True
except Exception:
    pass


def sdp_video_codecs(sdp: str) -> list[str]:
    """SDP 里 m=video 段的编码名列表（去重，保序）——看对端到底报了什么。"""
    names: list[str] = []
    in_video = False
    for line in sdp.splitlines():
        if line.startswith("m=video"):
            in_video = True
            continue
        if line.startswith("m="):
            in_video = False
        if in_video and line.startswith("a=rtpmap:"):
            enc = line.split(":", 1)[1].split(" ", 1)[1].split("/", 1)[0].strip()
            if enc and enc not in names:
                names.append(enc)
    return names


def install_bitrate_diagnostics(verbose: bool = False) -> None:
    """Optionally log encoder target_bitrate changes and the first codec.bit_rate.

    Only call this when you want the diagnostic lines; it wraps the encoder
    setter/encode so each REMB-driven change is visible.  Idempotent-ish: it
    replaces class attributes, so call once at startup.
    """
    if not verbose:
        return
    import logging

    _log = logging.getLogger("quest3_video_streamer")

    def _patch(cls, label):
        if cls is None or not hasattr(cls, "target_bitrate"):
            return
        prop = cls.target_bitrate
        if not isinstance(prop, property):
            return
        _orig_set = prop.fset

        def _logged_set(self, bitrate):
            _log.warning(f"[bitrate-diag][{label}] target_bitrate set to {bitrate}")
            if _orig_set is not None:
                _orig_set(self, bitrate)

        cls.target_bitrate = property(prop.fget, _logged_set)
        _orig_encode = cls.encode
        _done = {"v": False}

        def _logged_encode(self, frame, force_keyframe=False):
            codec = getattr(self, "codec", None)
            if not _done["v"] and codec is not None:
                _log.warning(
                    f"[bitrate-diag][{label}] first encode: codec.bit_rate="
                    f"{codec.bit_rate} target={self.target_bitrate} "
                    f"{codec.width}x{codec.height}"
                )
                _done["v"] = True
            return _orig_encode(self, frame, force_keyframe)

        cls.encode = _logged_encode

    try:
        _patch(__import__("aiortc.codecs.h264", fromlist=["H264Encoder"]).H264Encoder, "h264")
    except Exception:
        pass
    try:
        _patch(__import__("aiortc.codecs.vpx", fromlist=["Vp8Encoder"]).Vp8Encoder, "vp8")
    except Exception:
        pass


@dataclass(frozen=True, slots=True)
class TrackStats:
    index: int
    label: str
    fps: float
    bitrate_kbps: float
    frame_drops: int


@dataclass(frozen=True, slots=True)
class VideoSenderStats:
    fps: float
    bitrate_kbps: float
    frame_drops: int
    rtt_ms: float | None
    tracks: list[TrackStats] = None  # type: ignore[assignment]


def scaled_size(src_w: int, src_h: int, max_width: int) -> tuple[int, int]:
    """按 max_width 等比缩到偶数尺寸（yuv420p 要求）；不放大、不升级。"""
    if max_width <= 0 or src_w <= max_width:
        return src_w, src_h
    w = max_width - (max_width % 2)
    h = int(round(src_h * (w / src_w)))
    h -= h % 2
    return w, max(2, h)


class _AdapterVideoTrack:
    """Bridges a source adapter into the aiortc track API.

    Optional runtime gate: while ``gate.is_enabled(label)`` is false the track
    sends 2 fps black frames instead of camera frames.  This keeps the RTP
    stream (and the Quest panel) alive at negligible bandwidth without SDP
    renegotiation, and un-muting resumes the live feed instantly.

    Optional push throttle (``push_fps`` / ``push_max_width``): 回传规格与
    采集规格解耦——采集（collect tap）在 capture 线程侧拿全帧全速，而
    WebRTC 软编码是 Jetson 上最大的 CPU 负载；给 Quest 看的画面可以降
    分辨率/帧率而不影响录进数据集的内容。黑帧同样按降载尺寸生成，
    保证整轨分辨率恒定（x264 context 不重建）。

    Sources are opened lazily: a track only calls ``source.start()`` on its
    first un-muted frame, so cameras that are gated off (or whose device is
    missing) are never opened.  Open/read failures degrade to black frames
    with a 1 s retry backoff instead of killing the whole sender.
    """

    kind = "video"

    _MUTED_FPS = 2.0
    _RETRY_BACKOFF_S = 1.0

    def __init__(
        self,
        source: "VideoSourceAdapter",
        fps: int,
        gate: Any = None,
        label: str = "",
        log: Callable[[str], None] | None = None,
        push_fps: int = 0,
        push_max_width: int = 0,
    ) -> None:
        self._source = source
        self._fps = max(1, min(fps, push_fps) if push_fps > 0 else fps)
        self._gate = gate
        self._label = label
        self._log = log or (lambda _msg: None)
        fmt = source.get_format()
        self._push_w, self._push_h = scaled_size(fmt.width, fmt.height, push_max_width)
        self._pts = 0
        self._time_base = fractions.Fraction(1, self._fps)
        self._started = False
        self._muted = False

    async def recv(self) -> Any:
        if self._gate is not None and not self._gate.is_enabled(self._label):
            await asyncio.sleep(1.0 / self._MUTED_FPS)
            frame = self._new_black_frame()
            self._muted = True
        else:
            self._muted = False
            frame = await self._live_frame()
        frame.pts = self._pts
        frame.time_base = self._time_base
        self._pts += 1
        return frame

    async def _live_frame(self) -> Any:
        if not self._started:
            try:
                await self._source.start()
                self._started = True
                self._log(f"[track {self._label}] source started (lazy open)")
            except Exception as exc:
                self._log(f"[track {self._label}] source open failed: {exc}; sending black")
                await asyncio.sleep(self._RETRY_BACKOFF_S)
                return self._new_black_frame()
        try:
            frame = await self._source.next_frame()
            if (frame.width, frame.height) != (self._push_w, self._push_h):
                frame = frame.reformat(width=self._push_w, height=self._push_h)
            return frame
        except Exception as exc:
            self._log(f"[track {self._label}] next_frame failed: {exc}; sending black")
            try:
                await self._source.stop()
            except Exception:
                pass
            self._started = False  # retry open on the next recv
            await asyncio.sleep(self._RETRY_BACKOFF_S)
            return self._new_black_frame()

    def _new_black_frame(self) -> Any:
        """Fresh black frame at the push resolution (2 fps → cheap)."""
        import av  # lazy: only needed once a track is actually muted

        frame = av.VideoFrame(self._push_w, self._push_h, "yuv420p")
        # Limited-range black: Y=16, U=V=128.
        frame.planes[0].update(bytes([16]) * (self._push_w * self._push_h))
        cw, ch = self._push_w // 2, self._push_h // 2
        frame.planes[1].update(bytes([128]) * (cw * ch))
        frame.planes[2].update(bytes([128]) * (cw * ch))
        return frame


class VideoWebRTCSender:
    """One-to-one sender peer for host->Quest video.

    Supports one or more outbound video tracks (one per source adapter) so a
    single peer connection can carry multiple cameras (e.g. D435i + 2 wrist
    USB cams). The Quest offer must contain one recv-only video transceiver per
    source; the answer then carries one send track per source, in order.
    """

    def __init__(
        self,
        *,
        sources: "list[VideoSourceAdapter]",
        on_local_ice_candidate: Callable[[dict[str, Any]], Awaitable[None]] | None = None,
        log_hook: Callable[[str], None] | None = None,
        gate: Any = None,
        push_fps: int = 0,
        push_max_width: int = 0,
    ) -> None:
        if not sources:
            raise ValueError("At least one video source is required.")
        self._sources = list(sources)
        self._on_local_ice_candidate = on_local_ice_candidate
        self._log_hook = log_hook
        # Optional runtime gate (StreamGate): muted tracks send 2 fps black.
        self._gate = gate
        # 回传降载旋钮：0 = 不降载（保持源分辨率/帧率）。
        self._push_fps = int(push_fps)
        self._push_max_width = int(push_max_width)
        self._pc: Any = None
        self._created_at = monotonic()
        self._frames_sent = 0
        self._bytes_sent = 0
        self._frame_drops = 0
        self._last_stats_lock = asyncio.Lock()
        self._h264_forced = False
        # Per-track accounting for per-camera bandwidth/fps stats.
        # track_id -> {index, label}; last_bytes/last_frames keyed by track_id.
        self._track_info: list[dict[str, Any]] = []
        self._last_bytes: dict[str, int] = {}
        self._last_frames: dict[str, int] = {}
        self._last_stats_time: float = monotonic()
        # Per-track frame counters incremented in AdapterTrack.recv(); aiortc's
        # outbound-rtp stats have no framesSent field, so fps is derived here.
        self._track_frame_counts: list[int] = []

    async def start(self) -> None:
        # Sources are NOT started here: each track lazily opens its source on
        # the first un-muted frame (see _AdapterVideoTrack), so gated-off or
        # missing cameras are never opened.
        self._pc = self._new_peer_connection()
        self._add_video_tracks()
        self._wire_ice_callbacks()

    async def stop(self) -> None:
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
        for src in self._sources:
            try:
                await src.stop()  # adapters no-op when never started
            except Exception:
                pass

    async def apply_offer(self, *, sdp_offer: str) -> str:
        if self._pc is None:
            raise RuntimeError("Video sender not started.")
        rtc_session_description = self._import_aiortc_symbol("RTCSessionDescription")
        offer = rtc_session_description(sdp=sdp_offer, type="offer")
        self._log(f"offer video codecs: {sdp_video_codecs(sdp_offer)}")
        await self._pc.setRemoteDescription(offer)
        self._force_h264_codec_if_possible()
        answer = await self._pc.createAnswer()
        await self._pc.setLocalDescription(answer)
        answer_sdp = str(self._pc.localDescription.sdp)
        try:
            video_codecs: list[str] = []
            in_video = False
            for line in answer_sdp.splitlines():
                if line.startswith("m=video"):
                    in_video = True
                    video_codecs.append("unknown")
                    continue
                if line.startswith("m="):
                    in_video = False
                if in_video and line.startswith("a=rtpmap:") and video_codecs and video_codecs[-1] == "unknown":
                    payload = line.split(":", 1)[1]
                    enc = payload.split(" ", 1)[1].split("/", 1)[0] if " " in payload else ""
                    if enc:
                        video_codecs[-1] = enc
            self._log(f"negotiated video codecs: {video_codecs} (h264_forced={self._h264_forced})")
        except Exception:
            pass
        for t in self._pc.getTransceivers():
            self._log(
                f"transceiver kind={t.kind} direction={t.direction} "
                f"currentDirection={t.currentDirection}"
            )
        return answer_sdp

    async def add_remote_ice_candidate(
        self,
        *,
        candidate: str,
        sdp_mid: str | None,
        sdp_mline_index: int | None,
    ) -> None:
        if self._pc is None or not candidate:
            return
        candidate_from_sdp = self._import_aiortc_sdp_symbol("candidate_from_sdp")
        parsed = candidate_from_sdp(candidate)
        parsed.sdpMid = sdp_mid
        parsed.sdpMLineIndex = sdp_mline_index
        await self._pc.addIceCandidate(parsed)

    async def get_stats(self) -> VideoSenderStats:
        if self._pc is None:
            return VideoSenderStats(fps=0.0, bitrate_kbps=0.0, frame_drops=0, rtt_ms=None, tracks=[])
        async with self._last_stats_lock:
            try:
                report = await self._pc.getStats()
            except Exception:
                report = {}
            now = monotonic()
            dt = max(0.001, now - self._last_stats_time)
            self._last_stats_time = now

            rtt_ms: float | None = None
            # aiortc's outbound-rtp stats only carry bytesSent (no framesSent),
            # so bitrate comes from bytesSent and fps from our own per-track
            # frame counters (incremented in AdapterTrack.recv). Collect bytes
            # per outbound stream; order is stable across calls for index mapping.
            ob_bytes: list[int] = []
            for stat in (report.values() if hasattr(report, "values") else []):
                stat_type = getattr(stat, "type", "")
                if stat_type == "outbound-rtp":
                    ob_bytes.append(int(getattr(stat, "bytesSent", 0)))
                if stat_type == "candidate-pair":
                    current_rtt = getattr(stat, "currentRoundTripTime", None)
                    if current_rtt is not None:
                        rtt_ms = float(current_rtt) * 1000.0

            n = len(self._track_info)
            # Per-track bitrate from bytesSent deltas (order-mapped).
            track_stats: list[TrackStats] = []
            total_bitrate_kbps = 0.0
            total_fps = 0.0
            for i in range(n):
                b = ob_bytes[i] if i < len(ob_bytes) else 0
                prev_b = self._last_bytes.get(str(i), 0)
                db = max(0, b - prev_b)
                self._last_bytes[str(i)] = b
                bitrate_kbps = (db * 8.0 / dt) / 1000.0
                # fps from our own frame counter.
                cur_f = self._track_frame_counts[i] if i < len(self._track_frame_counts) else 0
                prev_f = self._last_frames.get(str(i), 0)
                df = max(0, cur_f - prev_f)
                self._last_frames[str(i)] = cur_f
                fps = df / dt
                total_bitrate_kbps += bitrate_kbps
                total_fps += fps
                track_stats.append(TrackStats(
                    index=i, label=self._track_info[i]["label"],
                    fps=fps, bitrate_kbps=bitrate_kbps, frame_drops=0,
                ))

            return VideoSenderStats(
                fps=total_fps,
                bitrate_kbps=total_bitrate_kbps,
                frame_drops=self._frame_drops,
                rtt_ms=rtt_ms,
                tracks=track_stats,
            )

    # -- internals --------------------------------------------------------

    def _new_peer_connection(self) -> Any:
        rtc_peer_connection = self._import_aiortc_symbol("RTCPeerConnection")
        rtc_configuration = self._import_aiortc_symbol("RTCConfiguration")
        rtc_ice_server = self._import_aiortc_symbol("RTCIceServer")
        config = rtc_configuration(
            iceServers=[rtc_ice_server(urls=["stun:stun.l.google.com:19302"])]
        )
        pc = rtc_peer_connection(config)
        self._wire_connection_state(pc)
        return pc

    def _wire_connection_state(self, pc: Any) -> None:
        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            self._log(f"connection state: {pc.connectionState}")

        @pc.on("iceconnectionstatechange")
        async def _on_ice_state() -> None:
            self._log(f"ICE connection state: {pc.iceConnectionState}")

        @pc.on("icegatheringstatechange")
        async def _on_ice_gathering() -> None:
            self._log(f"ICE gathering state: {pc.iceGatheringState}")

    def _log(self, message: str) -> None:
        if self._log_hook is not None:
            self._log_hook(message)

    def _add_video_tracks(self) -> None:
        if self._pc is None:
            raise RuntimeError("Peer connection not created.")
        video_stream_track = self._import_aiortc_symbol("VideoStreamTrack")

        class AdapterTrack(video_stream_track):  # type: ignore[misc, valid-type]
            def __init__(self, adapter: _AdapterVideoTrack, sender: VideoWebRTCSender, idx: int) -> None:
                super().__init__()
                self._adapter = adapter
                self._sender = sender
                self._idx = idx

            async def recv(self) -> Any:
                try:
                    frame = await self._adapter.recv()
                except Exception as exc:
                    import traceback

                    self._sender._log(f"recv() error: {exc}\n{''.join(traceback.format_exc())}")
                    raise
                self._sender._frames_sent += 1
                if frame is not None:
                    # Count actual frames delivered for per-track fps stats.
                    self._sender._track_frame_counts[self._idx] += 1
                else:
                    self._sender._frame_drops += 1
                return frame

        self._track_frame_counts = [0] * len(self._sources)
        for idx, source in enumerate(self._sources):
            video_format = source.get_format()
            adapter = _AdapterVideoTrack(
                source, fps=video_format.fps,
                gate=self._gate, label=video_format.label,
                log=self._log,
                push_fps=self._push_fps, push_max_width=self._push_max_width,
            )
            track = AdapterTrack(adapter, self, idx)
            self._pc.addTrack(track)
            self._track_info.append({"index": idx, "label": video_format.label, "id": getattr(track, "id", str(idx))})
            self._log(
                f"added video track #{idx} label={video_format.label} "
                f"{video_format.width}x{video_format.height}@{video_format.fps} "
                f"-> push {adapter._push_w}x{adapter._push_h}@{adapter._fps} "
                f"fov_h={video_format.fov_h_deg}"
            )

    def _wire_ice_callbacks(self) -> None:
        if self._pc is None or self._on_local_ice_candidate is None:
            return

        @self._pc.on("icecandidate")
        async def _on_icecandidate(candidate: Any) -> None:
            if candidate is None:
                return
            candidate_str = str(getattr(candidate, "candidate", ""))
            self._log(f"local ICE candidate: {candidate_str[:80]}")
            await self._on_local_ice_candidate(
                {
                    "candidate": candidate_str,
                    "sdpMid": getattr(candidate, "sdpMid", None),
                    "sdpMLineIndex": getattr(candidate, "sdpMLineIndex", None),
                }
            )

    def _force_h264_codec_if_possible(self) -> None:
        """把协商公共集里的 H264 提为每路 video transceiver 的首优编码。

        必须在 setRemoteDescription 之后、createAnswer 之前调用：aiortc 的
        createAnswer 直接用 setRemoteDescription 时算好的 transceiver._codecs
        （offer 顺序，VP8 在前），setCodecPreferences 对应答无效（实测：
        Quest offer 含 H264 却协商出 VP8）。发送端用 _codecs[0] 建编码器，
        故直接重排该列表。条目来自公共集（offer 参数的深拷贝），无 fmtp
        参数不匹配风险。
        """
        if self._pc is None:
            return
        try:
            for transceiver in self._pc.getTransceivers():
                if getattr(transceiver, "kind", "") != "video":
                    continue
                codecs = getattr(transceiver, "_codecs", None)
                if not codecs:
                    continue
                h264_pts = {
                    int(getattr(c, "payloadType"))
                    for c in codecs
                    if str(getattr(c, "mimeType", "")).lower() == "video/h264"
                }
                if not h264_pts:
                    self._log("公共编码集无 H264（对端 offer 未提供可兼容项），保持默认")
                    continue

                def _rank(c: Any) -> int:
                    mt = str(getattr(c, "mimeType", "")).lower()
                    if mt == "video/h264":
                        return 0
                    if mt == "video/rtx":
                        apt = getattr(c, "parameters", {}).get("apt")
                        try:
                            if apt is not None and int(apt) in h264_pts:
                                return 1  # H264 的 RTX 重传伴随包
                        except (TypeError, ValueError):
                            pass
                    return 2

                first_before = getattr(codecs[0], "mimeType", "?")
                codecs.sort(key=_rank)  # sort 稳定：同档内保持 offer 原序
                self._h264_forced = True
                self._log(
                    f"codec 首优 {first_before} -> {getattr(codecs[0], 'mimeType', '?')}"
                )
        except Exception:
            return

    def _import_aiortc_symbol(self, symbol: str) -> Any:
        try:
            return getattr(__import__("aiortc", fromlist=[symbol]), symbol)
        except Exception as exc:
            raise RuntimeError(
                "aiortc is required for video streaming. "
                "Install with: python3 -m pip install aiortc av"
            ) from exc

    def _import_aiortc_sdp_symbol(self, symbol: str) -> Any:
        try:
            return getattr(__import__("aiortc.sdp", fromlist=[symbol]), symbol)
        except Exception as exc:
            raise RuntimeError("aiortc.sdp unavailable.") from exc


# Forward declaration for type hints (the real base lives in ros_source.py /
# webcam_source.py and is duck-typed here).
class VideoSourceAdapter:  # pragma: no cover - structural protocol
    async def start(self) -> None: ...
    async def stop(self) -> None: ...
    async def next_frame(self) -> Any: ...
    def get_format(self) -> "VideoFormat": ...


