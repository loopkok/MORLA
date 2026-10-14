"""Launch lane state transitions do not depend on log chatter."""

from types import SimpleNamespace

from morla_web_manager import launch_manager


def test_quiet_live_process_promotes_after_warmup(monkeypatch):
    monkeypatch.setattr(launch_manager, "STARTUP_WARMUP_S", 0.0)
    manager = launch_manager.LaunchManager()
    process = SimpleNamespace(poll=lambda: None)
    manager._proc = process
    manager._state = launch_manager.STARTING

    manager._promote_after_warmup(process)

    assert manager.state == launch_manager.RUNNING


def test_exited_process_is_not_promoted_as_running(monkeypatch):
    monkeypatch.setattr(launch_manager, "STARTUP_WARMUP_S", 0.0)
    manager = launch_manager.LaunchManager()
    process = SimpleNamespace(poll=lambda: 1)
    manager._proc = process
    manager._state = launch_manager.STARTING

    manager._promote_after_warmup(process)

    assert manager.state == launch_manager.STARTING
