"""Push-throttle (回传降载) 的纯逻辑测试：scaled_size 与 track fps 上限。

不触相机/网络：_AdapterVideoTrack 构造只调 source.get_format()。
"""

from __future__ import annotations

from quest3_video_streamer.source_base import VideoFormat
from quest3_video_streamer.webrtc_sender import (
    _AdapterVideoTrack,
    scaled_size,
    sdp_video_codecs,
)


def _fmt(w: int, h: int, fps: int) -> VideoFormat:
    return VideoFormat(width=w, height=h, fps=fps, fov_h_deg=60.0, label="t")


class _Src:
    def __init__(self, fmt: VideoFormat) -> None:
        self._fmt = fmt

    def get_format(self) -> VideoFormat:
        return self._fmt


def test_scaled_size_disabled_when_max_width_zero():
    assert scaled_size(1920, 1080, 0) == (1920, 1080)


def test_scaled_size_no_upscale():
    assert scaled_size(1280, 720, 1920) == (1280, 720)


def test_scaled_size_keeps_aspect_and_even():
    w, h = scaled_size(1920, 1080, 960)
    assert (w, h) == (960, 540)
    assert w % 2 == 0 and h % 2 == 0


def test_scaled_size_odd_max_width_rounds_even():
    w, h = scaled_size(1920, 1080, 961)
    assert w % 2 == 0 and h % 2 == 0
    assert w <= 960


def test_track_fps_capped_by_push_fps():
    t = _AdapterVideoTrack(_Src(_fmt(1920, 1080, 30)), fps=30, push_fps=15)
    assert t._fps == 15
    assert t._time_base.numerator == 1 and t._time_base.denominator == 15


def test_track_fps_untouched_when_push_fps_zero_or_higher():
    assert _AdapterVideoTrack(_Src(_fmt(1280, 720, 30)), fps=30, push_fps=0)._fps == 30
    assert _AdapterVideoTrack(_Src(_fmt(1280, 720, 30)), fps=30, push_fps=60)._fps == 30


def test_track_push_size_defaults_to_source_when_disabled():
    t = _AdapterVideoTrack(_Src(_fmt(1920, 1080, 30)), fps=30, push_max_width=0)
    assert (t._push_w, t._push_h) == (1920, 1080)


def test_track_push_size_scaled():
    t = _AdapterVideoTrack(_Src(_fmt(1920, 1080, 30)), fps=30, push_max_width=960)
    assert (t._push_w, t._push_h) == (960, 540)


_SDP = """v=0\r
m=audio 9 UDP/TLS/RTP/SAVPF 111\r
a=rtpmap:111 opus/48000/2\r
m=video 9 UDP/TLS/RTP/SAVPF 96 97 98\r
a=rtpmap:96 VP8/90000\r
a=rtpmap:97 VP9/90000\r
a=rtpmap:98 H264/90000\r
a=fmtp:98 profile-level-id=42e01f\r
m=application 9 UDP/DTLS/SCTP webrtc-datachannel\r
"""


def test_sdp_video_codecs_only_video_section():
    assert sdp_video_codecs(_SDP) == ["VP8", "VP9", "H264"]


def test_sdp_video_codecs_vp8_only_offer():
    sdp = _SDP.replace(" 96 97 98", " 96").replace(
        "a=rtpmap:97 VP9/90000\r\n", ""
    ).replace("a=rtpmap:98 H264/90000\r\n", "")
    assert sdp_video_codecs(sdp) == ["VP8"]


def test_encoder_speed_patches_applied_when_aiortc_present():
    """aiortc 存在时 x264/VP8 补丁必须已挂上（静默回退会让本测试失败）。"""
    aiortc = __import__("aiortc", fromlist=["__version__"])
    import aiortc.codecs.h264 as h264_mod
    import aiortc.codecs.vpx as vpx_mod

    assert aiortc.__version__
    assert h264_mod.H264Encoder._encode_frame.__name__ == "_h264_encode_frame_with_preset"
    assert vpx_mod.Vp8Encoder.encode.__name__ == "_vp8_encode_fast"


# --- H264 首优重排（createAnswer 不吃 setCodecPreferences，只能直接排 _codecs）---

from types import SimpleNamespace

from quest3_video_streamer.webrtc_sender import VideoWebRTCSender


class _FakeCodec:
    def __init__(self, mime: str, pt: int, apt: int | None = None) -> None:
        self.mimeType = mime
        self.payloadType = pt
        self.parameters = {} if apt is None else {"apt": apt}


def _fake_sender(codecs: list) -> SimpleNamespace:
    tr = SimpleNamespace(kind="video", _codecs=codecs)
    logs: list[str] = []
    return SimpleNamespace(
        _pc=SimpleNamespace(getTransceivers=lambda: [tr]),
        _log=logs.append,
        _h264_forced=False,
        _tr=tr,
        _logs=logs,
    )


def test_force_h264_reorders_and_keeps_rtx_companion():
    # offer 顺序：VP8(96)+其rtx(97)、H264(102)+其rtx(103)
    codecs = [
        _FakeCodec("video/VP8", 96), _FakeCodec("video/rtx", 97, apt=96),
        _FakeCodec("video/H264", 102), _FakeCodec("video/rtx", 103, apt=102),
    ]
    s = _fake_sender(codecs)
    VideoWebRTCSender._force_h264_codec_if_possible(s)
    assert s._h264_forced is True
    assert [c.payloadType for c in codecs] == [102, 103, 96, 97]
    assert codecs[0].mimeType == "video/H264"


def test_force_h264_noop_when_offer_lacks_h264():
    codecs = [_FakeCodec("video/VP8", 96), _FakeCodec("video/rtx", 97, apt=96)]
    s = _fake_sender(codecs)
    VideoWebRTCSender._force_h264_codec_if_possible(s)
    assert s._h264_forced is False
    assert [c.payloadType for c in codecs] == [96, 97]
    assert any("无 H264" in m for m in s._logs)
