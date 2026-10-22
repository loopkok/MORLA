"""Actual OS process trees: these cases cannot be covered by instant fake hands."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading

import pytest

from morla_web_manager.piano_manager import PianoManager
from morla_web_manager.piano_processes import owned_processes, stop_owned
from test_piano_lane import store, manager, wait_until, open_software

pytestmark = pytest.mark.skipif(not Path('/proc/self/stat').exists(), reason='Linux /proc ownership')


def test_reap_reparented_new_session_child_and_preserve_unrelated_process(tmp_path):
    child_file = tmp_path / 'child'
    # A vendor child ignores graceful signals, detaches from the launch group,
    # and survives its parent's exit. Only its inherited random tag links it.
    child = "import os,signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); open(%r,'w').write(str(os.getpid())); time.sleep(60)" % str(child_file)
    parent = "import subprocess,sys; subprocess.Popen([sys.executable,'-c',%r],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)" % child
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], start_new_session=True)
    launch = PianoManager._spawn([sys.executable, '-c', parent])
    try:
        launch.wait(timeout=3)
        deadline = time.monotonic() + 3
        while not child_file.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        pid = int(child_file.read_text())
        assert pid in owned_processes(launch.piano_scope)
        assert os.getpgid(pid) != launch.pid
        assert unrelated.pid not in owned_processes(launch.piano_scope)
        assert stop_owned(launch.piano_scope, launch.pid, grace=.1, term=.1, kill=1) == []
        assert unrelated.poll() is None
    finally:
        stop_owned(launch.piano_scope, launch.pid, grace=.05, term=.05, kill=.5)
        unrelated.kill()
        unrelated.wait(timeout=2)


def test_manager_waits_for_slow_graceful_sdk_cleanup(tmp_path):
    ready, closed = tmp_path/'ready', tmp_path/'closed'
    program = """
import signal,time
def close(*args):
    time.sleep(.5)
    open(%r,'w').write('closed')
    raise SystemExit
signal.signal(signal.SIGINT,close)
open(%r,'w').write('ready')
time.sleep(60)
""" % (str(closed), str(ready))
    proc = PianoManager._spawn([sys.executable, '-c', program])
    try:
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert ready.exists()
        assert PianoManager._kill_group(proc) == []
        assert closed.read_text() == 'closed'
        assert proc.returncode == 0
    finally:
        PianoManager._kill_group(proc)


def test_serial_aliases_cannot_open_the_same_hand_twice(tmp_path):
    import pty
    master, slave = pty.openpty()
    try:
        a, b = tmp_path/'left', tmp_path/'right'
        a.symlink_to(os.ttyname(slave))
        b.symlink_to(os.ttyname(slave))
        with pytest.raises(ValueError, match='同一设备'):
            PianoManager._check_devices(dict(left_serial_port=str(a),right_serial_port=str(b)))
    finally:
        os.close(master)
        os.close(slave)


def test_cleanup_retains_exclusivity_until_sdk_processes_exit(manager, monkeypatch):
    gone = threading.Event()
    manager._active = True
    manager._resource_lock('ros_fake')
    monkeypatch.setattr(manager, '_kill_group', lambda proc: [] if gone.is_set() else [12345])
    thread = threading.Thread(target=manager._finish_cleanup,args=(None,None,manager._generation,False,None))
    thread.start()
    try:
        wait_until(lambda:manager.snapshot()['state']=='CLEANUP_FAILED')
        assert manager.active and manager._resource_fds
        with pytest.raises(ValueError): manager.editable()
        gone.set()
        thread.join(timeout=4)
        assert not thread.is_alive()
        assert not manager.active and not manager._resource_fds
    finally:
        gone.set()
        thread.join(timeout=4)


def test_frozen_worker_cannot_keep_showing_fresh_feedback(manager):
    manager.STATUS_SECONDS = .4
    manager.CLOSE_SECONDS = .2
    manager.LEASE_SECONDS = 30
    open_software(manager)
    proc = manager.proc
    os.kill(proc.pid, signal.SIGSTOP)
    try:
        wait_until(lambda:manager.snapshot()['closing'])
        assert manager.snapshot()['connected'] is False
        wait_until(lambda:not manager.active, timeout=12)
        assert '反馈心跳中断' in manager.snapshot()['error']
        assert proc.poll() is not None
    finally:
        try: os.kill(proc.pid,signal.SIGCONT)
        except ProcessLookupError: pass


def test_feedback_age_keeps_advancing_without_new_worker_snapshot(manager):
    manager._state={'feedback_age_s': {'left_arm': .02}}
    manager._snapshot_seen=time.monotonic()-.5
    assert manager.snapshot()['feedback_age_s']['left_arm'] >= .52
