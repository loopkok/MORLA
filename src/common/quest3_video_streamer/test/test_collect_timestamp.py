"""JPEG encoding must not overwrite the source frame timestamp."""
import queue
import threading
import time
from types import SimpleNamespace
import numpy as np
from quest3_video_streamer.collect_tap import CollectTapPublisher


def test_source_timestamp_survives_encoding():
    tap = object.__new__(CollectTapPublisher)
    tap._label = 'base'
    tap._quality = 90
    tap._queue = queue.Queue(maxsize=1)
    tap._stop = threading.Event()
    tap._max_fps = 30
    tap._tokens = 2
    tap._last_refill = time.monotonic()
    tap._last_submit_t = None
    tap._last_publish_t = None
    tap._stat_t0 = time.monotonic()
    tap._diagnostic_hook = None
    for name in ['_n_submitted', '_n_rate_skip', '_n_queue_full', '_n_encoded', '_n_published',
                 '_encode_ms_sum', '_publish_ms_sum', '_max_submit_gap_ms', '_max_publish_gap_ms']:
        setattr(tap, name, 0)
    messages = []
    def publish(message):
        messages.append(message)
        tap._stop.set()
    tap._pub = SimpleNamespace(get_subscription_count=lambda: 1, publish=publish)
    # Access to the node clock would mean the original timestamp was lost.
    tap._node = None
    tap._msg_type = lambda: SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))
    source_ns = 1700000000123456789
    tap.submit(np.zeros((16, 16, 3), dtype=np.uint8), is_rgb=False, timestamp_ns=source_ns)
    time.sleep(0.01)
    tap._run()
    assert len(messages) == 1
    stamp = messages[0].header.stamp
    assert stamp.sec * 10**9 + stamp.nanosec == source_ns
