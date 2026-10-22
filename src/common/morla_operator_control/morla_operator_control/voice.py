"""Host microphone or timestamped Quest PCM -> offline ASR -> shared commands."""
import json
import math
import queue
import socket
import shutil
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from morla_input.packet_guard import PacketGuard

from .asr import OfflineRecognizer, load_optional_dependencies
from .audio_protocol import decode_pcm
from .commands import request, voice_command
from .router import TOPIC, EVENTS


class VoiceInput(Node):
    def __init__(self):
        super().__init__("morla_voice_input")
        self.declare_parameter("source", "host")
        self.declare_parameter("model_path", "~/.local/share/morla/models/vosk-model-small-cn-0.22")
        self.declare_parameter("microphone_device", "")
        self.declare_parameter("quest_bind", "127.0.0.1")
        self.declare_parameter("quest_port", 8001)
        self.declare_parameter("wake_word", "机器人")
        self.declare_parameter("min_confidence", 0.75)
        self.declare_parameter("dry_run", False)
        self.source = str(self.get_parameter("source").value)
        if self.source not in {"host", "quest"}:
            raise ValueError("source must be host or quest")
        confidence = float(self.get_parameter("min_confidence").value)
        if not 0.5 <= confidence <= 1.0 or not math.isfinite(confidence):
            raise ValueError("min_confidence must be between 0.5 and 1.0")
        if not str(self.get_parameter("wake_word").value).strip():
            raise ValueError("wake_word must not be empty")
        self.tag = "voice_" + self.source
        self._pub = self.create_publisher(String, TOPIC, 10)
        self._transcripts = self.create_publisher(String, "/morla/operator/transcripts", 10)
        self._source_pub = self.create_publisher(String, "/morla/operator/sources", 10)
        self.create_subscription(String, EVENTS, self._on_feedback, 10)
        self._audio = queue.Queue(maxsize=8)
        self._generation = 0
        self._stop = threading.Event()
        self._network = None
        self._server = None
        self._network_lock = threading.Lock()
        self._status = "initializing"
        self._detail = ""
        self.create_timer(1.0, self._publish_source)
        self.recognizer = OfflineRecognizer(str(self.get_parameter("model_path").value))
        self._worker = threading.Thread(target=self._recognize, daemon=True)
        self._worker.start()
        if self.source == "host":
            try:
                self._open_microphone()
            except Exception as exc:
                self._status, self._detail = "error", str(exc)
                self.get_logger().error(f"Host microphone unavailable: {exc}")
        else:
            self._network_thread = threading.Thread(target=self._listen, daemon=True)
            self._network_thread.start()

    def _publish_source(self):
        self._source_pub.publish(String(data=json.dumps(dict(
            source=self.tag, state=self._status, detail=self._detail, sent_at=time.time()))))

    def _invalidate(self):
        self._generation += 1
        while True:
            try:
                self._audio.get_nowait()
            except queue.Empty:
                return

    def _enqueue(self, data, received):
        try:
            self._audio.put_nowait((self._generation, received, data))
        except queue.Full:
            self._invalidate()

    def _open_microphone(self):
        load_optional_dependencies()
        device = str(self.get_parameter("microphone_device").value).strip()
        if device.startswith("alsa:"):
            self._open_alsa(device[5:] or "default")
            return
        try:
            import sounddevice as sd
        except (ImportError, OSError):
            if not device and shutil.which("arecord"):
                self._open_alsa("default")
                return
            raise
        device = int(device) if device.isdecimal() else device or None
        info = sd.query_devices(device, "input")
        if info["max_input_channels"] < 1 or "monitor" in info["name"].lower():
            raise ValueError("select a physical input microphone, not a speaker monitor")
        def audio_callback(indata, frames, timing, status):
            if status:
                self._invalidate()
                return
            self._enqueue(bytes(indata), time.monotonic())
        self._capture = sd.RawInputStream(device=device, samplerate=16000, channels=1,
                                         dtype="int16", blocksize=640, callback=audio_callback)
        self._capture.start()
        self._status, self._detail = "listening", str(info["name"])

    def _open_alsa(self, device):
        import fcntl
        import os
        import select
        self._alsa_error = tempfile.TemporaryFile()
        self._alsa = subprocess.Popen(["arecord", "-q", "-D", device, "-f", "S16_LE", "-r", "16000",
                                       "-c", "1", "--buffer-time=100000", "-t", "raw"],
                                       stdout=subprocess.PIPE, stderr=self._alsa_error, bufsize=0)
        fd = self._alsa.stdout.fileno()
        fcntl.fcntl(fd, fcntl.F_SETPIPE_SZ, 4096)
        def read():
            self._status, self._detail = "connecting", "ALSA " + device
            previous = time.monotonic()
            while not self._stop.is_set():
                # Raw FileIO may return short reads. Accumulate exactly one frame.
                data = b""
                while len(data) < 1280 and not self._stop.is_set():
                    part = self._alsa.stdout.read(1280 - len(data))
                    if not part:
                        break
                    data += part
                if len(data) != 1280:
                    self._invalidate()
                    self._alsa_error.seek(0)
                    self._status, self._detail = "error", self._alsa_error.read(1024).decode(errors="replace")
                    return
                now = time.monotonic()
                if now - previous > 0.2:
                    self._invalidate()
                    for _ in range(16):
                        if not select.select([fd], [], [], 0)[0]:
                            break
                        os.read(fd, 4096)
                else:
                    self._status = "listening"
                    self._enqueue(data, now)
                previous = now
        self._alsa_reader = threading.Thread(target=read, daemon=True)
        self._alsa_reader.start()

    def _recognize(self):
        generation, previous = -1, 0.0
        while not self._stop.is_set():
            try:
                current, received, data = self._audio.get(timeout=0.1)
            except queue.Empty:
                continue
            now = time.monotonic()
            if current != generation or now - previous > 0.5:
                self.recognizer.reset()
                generation = current
            previous = now
            if current != self._generation or now - received > 0.5:
                self.recognizer.reset()
                continue
            result = self.recognizer.feed(data)
            if result and not self._stop.is_set() and current == self._generation and time.monotonic() - received <= 0.5:
                text, confidence = result
                command = voice_command(text, confidence,
                    wake_word=str(self.get_parameter("wake_word").value),
                    min_confidence=float(self.get_parameter("min_confidence").value))
                self._transcripts.publish(String(data=json.dumps(dict(
                    source=self.tag, text=text, confidence=confidence, command=command,
                    dry_run=bool(self.get_parameter("dry_run").value), sent_at=time.time()), ensure_ascii=False)))
                if command and not self.get_parameter("dry_run").value:
                    self._pub.publish(String(data=json.dumps(request(command, self.tag))))

    def _listen(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                self._server = server
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((str(self.get_parameter("quest_bind").value), int(self.get_parameter("quest_port").value)))
                server.listen(2)
                server.settimeout(0.2)
                self._status, self._detail = "waiting", "Quest PCM :8001, enable microphone in Quest app"
                while not self._stop.is_set():
                    try:
                        connection, _ = server.accept()
                    except socket.timeout:
                        continue
                    self._handle_connection(connection)
        except OSError as exc:
            if not self._stop.is_set():
                self._status, self._detail = "error", str(exc)

    def _handle_connection(self, connection):
        guard, peer = PacketGuard(max_age=0.5), "quest_voice"
        self._invalidate()
        self._network = connection
        self._status = "connected"
        buffer, last_audio = b"", time.monotonic()
        try:
            with connection:
                connection.settimeout(0.2)
                with self._network_lock:
                    connection.sendall(guard.make_sync_challenge(peer, force=True).encode())
                while not self._stop.is_set():
                    challenge = guard.make_sync_challenge(peer)
                    if challenge:
                        with self._network_lock:
                            connection.sendall(challenge.encode())
                    try:
                        data = connection.recv(8192)
                    except socket.timeout:
                        if time.monotonic() - last_audio > 1.0:
                            raise OSError("Quest microphone stream timed out")
                        continue
                    if not data:
                        break
                    buffer += data
                    if len(buffer) > 32768:
                        raise ValueError("oversized Quest audio frame")
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        line = line.decode("ascii").strip()
                        now_mono, now_wall = time.monotonic(), time.time()
                        if line.startswith("@q3sync_ack "):
                            guard.handle_sync_ack(line, peer)
                            continue
                        accepted = guard.accept(line, received=now_wall, received_monotonic=now_mono)
                        if accepted:
                            if guard.last_session_change:
                                self._invalidate()
                            pcm = decode_pcm(accepted[0])
                            last_audio = now_mono
                            self._status = "listening"
                            self._enqueue(pcm, now_mono - max(0.0, now_wall - accepted[1]))
        except (OSError, ValueError, UnicodeError):
            pass
        finally:
            self._network = None
            self._invalidate()
            self._status = "waiting"

    def _on_feedback(self, msg):
        try:
            event = json.loads(msg.data)
            if event.get("phase") not in {"confirmed", "failed", "rejected", "confirmation_required"}:
                return
            if self._network:
                wire = json.dumps(dict(type="feedback", phase=event["phase"], message=event["message"]), ensure_ascii=False)
                with self._network_lock:
                    self._network.sendall((wire + "\n").encode())
        except (ValueError, KeyError, OSError):
            pass

    def destroy_node(self):
        self._stop.set()
        self._invalidate()
        capture = getattr(self, "_capture", None)
        if capture:
            capture.close()
        alsa = getattr(self, "_alsa", None)
        if alsa:
            alsa.terminate()
            try:
                alsa.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                alsa.kill()
            self._alsa_reader.join(timeout=1.0)
            alsa.stdout.close()
            self._alsa_error.close()
        if self._network:
            self._network.close()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
