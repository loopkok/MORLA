"""Exercise port ownership and handoff with isolated dummy TCP processes."""
import socket
import subprocess
import sys
import time

import pytest

from morla_web_manager.quest_preview import QuestPreview


SCRIPT = """
import signal,socket,sys,time
signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind(('127.0.0.1',int(sys.argv[1])));s.listen()
print('READY',flush=True)
while True:time.sleep(.05)
"""


def wait_for(condition):
    deadline = time.monotonic() + 4
    while not condition() and time.monotonic() < deadline:
        time.sleep(.02)
    assert condition()


@pytest.fixture
def preview():
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        port = reserve.getsockname()[1]
    node = QuestPreview(True, port, lambda: [sys.executable, "-u", "-c", SCRIPT, str(port)])
    try:
        yield node
    finally:
        node.close()


def test_preview_starts_without_a_profile_and_close_releases_owned_port(preview):
    preview.tick(False)
    wait_for(lambda: not preview._port_free())
    preview.tick(False)
    assert preview.snapshot()["state"] == "listening"
    process = preview._proc
    preview.close()
    assert process.poll() == 0 and preview._port_free()
    preview.tick(False)
    assert preview.snapshot()["pid"] is None


def test_profile_takes_port_only_after_preview_exits_and_preview_returns_after_stop(preview):
    preview.tick(False)
    wait_for(lambda: not preview._port_free())
    old = preview._proc
    profile = None
    def start():
        nonlocal profile
        assert old.poll() == 0 and preview._port_free()
        profile = subprocess.Popen(preview._command_factory(), stdout=subprocess.DEVNULL)
        wait_for(lambda: not preview._port_free())
        return True, "accepted"
    try:
        assert preview.start_stack(start)[0]
        preview.tick(True)
        assert profile.poll() is None and preview.snapshot()["state"] == "profile"
    finally:
        if profile:
            profile.terminate(); profile.wait(timeout=3)
    preview.tick(False)
    wait_for(lambda: not preview._port_free())
    assert preview._proc.pid != old.pid


def test_external_listener_is_never_killed_or_claimed(preview):
    with socket.socket() as external:
        external.bind(("127.0.0.1", preview.port));external.listen()
        preview.tick(False)
        assert preview.snapshot()["state"] == "external_port_owner"
        assert preview._proc is None
        called = []
        assert not preview.start_stack(lambda: called.append(True))[0]
        assert not called
        preview.close()
        assert external.fileno() >= 0


def test_failed_profile_start_restores_preview_and_active_stack_blocks_start(preview):
    preview.tick(True)
    assert preview._proc is None
    assert preview.start_stack(lambda: (False, "invalid preset")) == (False, "invalid preset")
    preview.tick(False)
    wait_for(lambda: not preview._port_free())


def test_profile_api_releases_preview_before_accepting_launch(preview, monkeypatch):
    pytest.importorskip("rclpy")
    import asyncio
    from types import SimpleNamespace
    from morla_web_manager import web_server
    from morla_web_manager.schemas import ProfileLaunchRequest
    preview.tick(False)
    wait_for(lambda: not preview._port_free())
    old = preview._proc
    launched = []
    def start(preset):
        assert old.poll() == 0 and preview._port_free()
        launched.append(preset.args["with_input"])
        return True, "accepted"
    monkeypatch.setattr(web_server, "_quest_preview", preview)
    monkeypatch.setattr(web_server, "_launch_mgr", SimpleNamespace(start=start))
    monkeypatch.setattr(web_server, "get_node", lambda: None)
    monkeypatch.setattr(web_server, "_profile_switch_blockers", lambda *args: [])
    monkeypatch.setattr(web_server, "_selected_profile_id", web_server._selected_profile_id)
    monkeypatch.setattr(web_server, "_selected_operating_mode", web_server._selected_operating_mode)
    response = asyncio.run(web_server.launch_robot_profile(
        ProfileLaunchRequest(profile_id="nero_dual_xhand", operating_mode="sim")))
    assert response.ok and launched == ["true"]
