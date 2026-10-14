"""Preview tap accepts and preserves source timestamps."""
import queue
from types import SimpleNamespace

from quest3_video_streamer.preview import PreviewPublisher


def _publisher(clock_ns=1234567890):
    publisher = object.__new__(PreviewPublisher)
    publisher._node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=clock_ns)
        )
    )
    publisher._label = "base"
    publisher._gate = None
    publisher._period = 0.0
    publisher._last_submit = 0.0
    publisher._queue = queue.Queue(maxsize=1)
    return publisher


def test_submit_accepts_source_timestamp():
    publisher = _publisher()
    frame = object()
    source_ns = 1700000000123456789

    publisher.submit(frame, is_rgb=True, timestamp_ns=source_ns)

    queued_frame, is_rgb, queued_ns = publisher._queue.get_nowait()
    assert queued_frame is frame
    assert is_rgb is True
    assert queued_ns == source_ns


def test_submit_uses_node_clock_when_timestamp_missing():
    publisher = _publisher(clock_ns=9876543210)

    publisher.submit(object(), is_rgb=False)

    assert publisher._queue.get_nowait()[2] == 9876543210
