import os
import signal
import subprocess
import sys
import time
import pytest
from morla_runtime import AdapterWorker, ProcessExecutor


class HungSDK:
    def read(self):
        time.sleep(30)


def test_sdk_worker_timeout_fences_and_terminates_only_worker():
    worker = AdapterWorker(HungSDK, timeout=0.4)
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    try:
        began = time.monotonic()
        with pytest.raises(TimeoutError): worker.call('read')
        assert time.monotonic() - began < 2
        assert not worker.process.is_alive()
        assert unrelated.poll() is None
        with pytest.raises(RuntimeError, match='WORKER_FENCED'): worker.call('read')
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=2)
        worker.close()


def test_executor_never_interprets_shell_metacharacters(tmp_path):
    marker = tmp_path / 'unexpected'
    executor = ProcessExecutor()
    with pytest.raises(ValueError): executor.start(['true'])
    child = executor.start([sys.executable, '-c', 'import sys; assert sys.argv[1].startswith("$(")',
                            '$(touch ' + str(marker) + ')'], trusted=True)
    assert child.wait(timeout=2) == 0
    assert not marker.exists()
    executor.close()
