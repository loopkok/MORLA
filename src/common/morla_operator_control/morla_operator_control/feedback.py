"""Speak only confirmed results/explicit failures; never echo wake phrases."""
import io
import json
import math
import queue
import shutil
import struct
import subprocess
import threading
import time
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .router import EVENTS


class OperatorFeedback(Node):
    def __init__(self):
        super().__init__("morla_operator_feedback")
        self._queue = queue.Queue(maxsize=8)
        self._stop = threading.Event()
        self.create_subscription(String, EVENTS, self._on_event, 10)
        self._thread = threading.Thread(target=self._play, daemon=True)
        self._thread.start()

    def _on_event(self, msg):
        try:
            row = json.loads(msg.data)
            if (row.get("phase") in {"confirmed", "failed", "rejected", "confirmation_required"}
                    and abs(time.time() - row["sent_at"]) <= 2.0):
                self._queue.put_nowait(row)
        except (ValueError, TypeError, KeyError, queue.Full):
            pass

    def _play(self):
        while not self._stop.is_set():
            try:
                row = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if time.time() - row["sent_at"] > 3.0:
                continue
            text = row.get("message", "")[:160]
            try:
                if shutil.which("espeak-ng"):
                    # A result must not itself contain the wake phrase.
                    text = text.replace("机器人", "设备")
                    subprocess.run(["espeak-ng", "-v", "cmn", text], timeout=15, check=False)
                elif shutil.which("aplay"):
                    frequency = 880 if row["phase"] == "confirmed" else 330
                    data = b"".join(struct.pack("<h", int(4000 * math.sin(2 * math.pi * frequency * n / 16000)))
                                    for n in range(2400))
                    output = io.BytesIO()
                    with wave.open(output, "wb") as wav:
                        wav.setnchannels(1); wav.setsampwidth(2); wav.setframerate(16000); wav.writeframes(data)
                    subprocess.run(["aplay", "-q"], input=output.getvalue(), timeout=3, check=False)
            except (OSError, subprocess.TimeoutExpired):
                pass

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OperatorFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
