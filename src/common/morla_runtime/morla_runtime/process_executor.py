"""No shells; bounded, identity-scoped subprocess and SDK containment."""
import multiprocessing
import os
import signal
import subprocess
import threading


class ProcessExecutor:
    def __init__(self):
        self.children = []

    def start(self, argv, *, trusted=False, env=None, pass_fds=()):
        if not trusted or not isinstance(argv, list) or not argv or not all(isinstance(x, str) and '\x00' not in x for x in argv):
            raise ValueError('UNTRUSTED_ARGV')
        child = subprocess.Popen(argv, shell=False, start_new_session=True, env=env, pass_fds=pass_fds)
        self.children.append(child)
        return child

    def close(self, timeout=1.0):
        for child in reversed(self.children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=timeout)
        self.children.clear()


def _serve(connection, factory, args):
    adapter = factory(*args)
    while True:
        try:
            method, args = connection.recv()
        except EOFError:
            break
        try:
            connection.send((True, getattr(adapter, method)(*args)))
        except Exception as exc:
            connection.send((False, type(exc).__name__ + ': ' + str(exc)))


class AdapterWorker:
    """SDK calls run in an owned process; timeout kills and permanently fences it.

    Killing a process is not a hardware stop. Real adapters still require a
    separately verified device/board watchdog before activation is permitted.
    """
    def __init__(self, factory, *args, timeout=0.25):
        ctx = multiprocessing.get_context('spawn')
        self.connection, remote = ctx.Pipe()
        self.process = ctx.Process(target=_serve, args=(remote, factory, args), daemon=True)
        self.process.start()
        remote.close()
        self.timeout = timeout
        self.lock = threading.Lock()
        self.failed = False

    def call(self, method, *args):
        with self.lock:
            if self.failed:
                raise RuntimeError('WORKER_FENCED')
            self.connection.send((method, args))
            if not self.connection.poll(self.timeout):
                self.failed = True
                self.close()
                raise TimeoutError('SDK_TIMEOUT: ' + method)
            ok, value = self.connection.recv()
            if not ok:
                raise RuntimeError(value)
            return value

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=self.timeout)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=self.timeout)
        self.connection.close()
