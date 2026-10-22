"""Real local TCP handshake/framing, fake ASR: never opens a microphone."""
import base64
import json
import socket
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from morla_operator_control.voice import VoiceInput


class Recognizer:
    def __init__(self, path):
        self.frames = 0
        self.resets = 0
    def reset(self):
        self.resets += 1
    def feed(self, data):
        self.frames += 1
        return ("机器人开始跟随", .99)


@pytest.fixture
def voice(monkeypatch):
    monkeypatch.setattr("morla_operator_control.voice.OfflineRecognizer", Recognizer)
    rclpy.init(args=["--ros-args", "-p", "source:=quest", "-p", "quest_port:=0"])
    node = VoiceInput()
    node.commands = []
    from types import SimpleNamespace
    node._pub = SimpleNamespace(publish=lambda msg: node.commands.append(json.loads(msg.data)))
    deadline = time.monotonic() + 2
    while node._status != "waiting" and time.monotonic() < deadline:
        time.sleep(.01)
    assert node._status == "waiting"
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def connect(node):
    client = socket.create_connection(node._server.getsockname(), timeout=2)
    reader = client.makefile("rb")
    challenge = reader.readline().decode().strip().split()
    source_us = time.monotonic_ns() // 1000
    client.sendall(f"@q3sync_ack test {challenge[1]} {source_us} {source_us}\n".encode())
    return client, reader


def packet(seq=1, age=0):
    pcm = base64.b64encode(bytes(1280)).decode()
    return f"@q3m test {seq} {time.monotonic_ns() // 1000 - int(age * 1e6)}|VoicePCM:16000:{pcm}\n".encode()


def until(condition):
    end = time.monotonic() + 2
    while not condition() and time.monotonic() < end:
        time.sleep(.01)
    assert condition()


def test_quest_pcm_handshake_split_frame_and_normalized_voice_command(voice):
    client, reader = connect(voice)
    try:
        data = packet()
        client.sendall(data[:20]); client.sendall(data[20:])
        until(lambda: voice.commands)
        assert voice.commands[0]["source"] == "voice_quest"
        assert voice.commands[0]["command"] == "teleop_start"
    finally:
        reader.close(); client.close()
    until(lambda: voice._status == "waiting")


def test_stale_audio_and_replayed_sequence_do_not_reach_recognizer(voice):
    client, reader = connect(voice)
    try:
        client.sendall(packet(age=2))
        time.sleep(.05)
        assert voice.recognizer.frames == 0
        first = packet(seq=2)
        client.sendall(first)
        until(lambda: voice.recognizer.frames == 1)
        client.sendall(first)
        time.sleep(.05)
        assert voice.recognizer.frames == 1
    finally:
        reader.close(); client.close()


def test_disconnect_invalidates_in_flight_recognition(voice):
    entered, release = threading.Event(), threading.Event()
    def delayed(data):
        entered.set(); release.wait(2)
        return "机器人开始跟随", .99
    voice.recognizer.feed = delayed
    client, reader = connect(voice)
    client.sendall(packet())
    assert entered.wait(1)
    reader.close(); client.close()
    until(lambda: voice._status == "waiting")
    release.set()
    time.sleep(.05)
    assert not voice.commands


def test_host_alsa_pipe_feeds_same_command_contract_without_microphone(monkeypatch):
    import subprocess
    import sys
    from types import SimpleNamespace
    original = subprocess.Popen
    def generated_audio(args, **kwargs):
        assert args[0] == "arecord"
        script = "import sys,time; time.sleep(.1); " + "sys.stdout.buffer.write(bytes(1280));sys.stdout.buffer.flush();time.sleep(.04);" * 5
        return original([sys.executable, "-c", script], **kwargs)
    monkeypatch.setattr("morla_operator_control.voice.OfflineRecognizer", Recognizer)
    monkeypatch.setattr("morla_operator_control.voice.subprocess.Popen", generated_audio)
    monkeypatch.setattr(VoiceInput, "_open_microphone", lambda node: node._open_alsa("fake-device"))
    rclpy.init(args=["--ros-args", "-p", "source:=host"])
    node = VoiceInput()
    rows = []
    node._pub = SimpleNamespace(publish=lambda msg: rows.append(json.loads(msg.data)))
    try:
        until(lambda: rows)
        assert rows[0]["source"] == "voice_host" and rows[0]["command"] == "teleop_start"
    finally:
        node.destroy_node()
        rclpy.shutdown()
