"""Real subprocess/ROS fake watchdog; explicitly isolated from hardware domain."""
import os
from pathlib import Path
import signal
import time

import pytest

pytestmark=pytest.mark.skipif(os.environ.get('MORLA_TEST_PIANO_ROS')!='1',reason='explicit ROS fake test')


def test_frozen_ros_worker_gets_stop_only_recovery_and_all_drivers_exit(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='78' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    from morla_web_manager.piano_manager import PianoManager
    from morla_web_manager.piano_store import PianoStore
    from morla_web_manager.piano_processes import owned_processes
    store=PianoStore(Path(os.environ['MORLA_PIANO_PROJECT']),tmp_path)
    manager=PianoManager(store)
    manager.STATUS_SECONDS=1.
    token=None
    def wait(condition,timeout=18):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if token and not manager.snapshot()['closing'] and manager.active: manager.heartbeat(token)
            state=manager.snapshot()
            if condition(state): return state
            time.sleep(.05)
        raise AssertionError(manager.snapshot())
    try:
        opened=manager.open('ros_fake',store.config()['revision'])
        token=opened['token']
        wait(lambda s:s['state']=='CONNECTED')
        job=manager.command(token,'prepare')
        wait(lambda s:(s.get('result') or {}).get('job_id')==job['id'] and s['result']['ok'])
        worker,drivers=manager.proc,manager.drivers
        os.kill(worker.pid,signal.SIGSTOP)
        state=wait(lambda s:not s['active'])
        assert state['recovery']['exit_code']==0
        assert '反馈心跳中断' in state['error']
        assert not owned_processes(drivers.piano_scope)
        log=next(tmp_path.glob('sessions/*/drivers.log')).read_text()
        assert log.count('Electronic emergency stop was sent')>=2,log[-6000:]
    finally:
        manager.close()
