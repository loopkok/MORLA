"""Background-idle grace must never extend an active motion watchdog."""
import time

import pytest

from morla_web_manager.piano_store import PianoConflict
from test_piano_lane import store,manager,open_software,wait_until


def test_idle_can_renew_after_short_browser_throttle_without_reopening(manager):
    token=open_software(manager)
    proc=manager.proc
    with manager.lock:manager._lease_seen-=10
    time.sleep(.3)
    assert manager.active and manager.proc is proc
    state=manager.snapshot()
    assert state['browser_heartbeat_timeout_s'] is None
    assert state['browser_heartbeat_age_s']>=10
    with pytest.raises(PianoConflict,match='恢复页面心跳'):
        manager.command(token,'prepare')
    with pytest.raises(PianoConflict,match='恢复页面心跳'):
        manager.command(token,'refresh')
    manager.heartbeat(token)
    assert manager.snapshot()['browser_heartbeat_age_s']<1
    assert manager.snapshot()['state']=='CONNECTED' and manager._job is None


def test_idle_can_renew_after_long_background_suspension(manager):
    token=open_software(manager)
    with manager.lock:
        manager._lease_seen-=86400
    time.sleep(.3)
    assert manager.active
    manager.heartbeat(token)
    assert manager.snapshot()['browser_heartbeat_age_s']<1
    assert manager._job is None


def test_active_motion_keeps_short_watchdog_even_with_long_idle_grace(manager):
    token=open_software(manager)
    manager.LEASE_SECONDS=.25
    manager.IDLE_LEASE_SECONDS=None
    manager.heartbeat(token)
    manager.command(token,'prepare')
    proc=manager.proc
    assert manager.snapshot()['browser_heartbeat_timeout_s']==.25
    wait_until(lambda:not manager.active)
    assert proc.poll() is not None


def test_job_and_unsettled_state_cannot_use_idle_grace(manager):
    manager._state={'state':'READY'}
    manager._job=None
    assert manager._lease_seconds() is None
    manager._job={'operation':'work_pose'}
    assert manager._lease_seconds()==6
    manager._job=None
    for state in ('PLAYING','PREPARING','CALIBRATING','STARTING','CLOSING'):
        manager._state={'state':state}
        assert manager._lease_seconds()==6
    manager._lease_seen=time.monotonic()-7
    assert manager._lease_expired()
