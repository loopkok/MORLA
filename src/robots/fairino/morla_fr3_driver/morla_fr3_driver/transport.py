"""Bounded process RPC: vendor reconnect/retry loops cannot block the host forever."""
import multiprocessing
import threading

from .contract import validate_parameters


def _worker(pipe, parameters):
    backend = None
    try:
        from .sdk_backend import SDKBackend
        backend = SDKBackend(parameters)
        pipe.send((True, None))
        while True:
            name, args = pipe.recv()
            if name not in {'read', 'write', 'activate', 'stop', 'close', 'forward_kinematics'}:
                raise ValueError('unsupported SDK operation')
            try:
                value = getattr(backend, name)(*args)
                pipe.send((True, value))
            except Exception as exc:
                pipe.send((False, f'{type(exc).__name__}: {exc}'))
            if name == 'close':
                backend = None
                return
    except EOFError:
        pass
    except Exception as exc:
        try:
            pipe.send((False, f'{type(exc).__name__}: {exc}'))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if backend is not None:
            backend.close()
        pipe.close()


class FR3Transport:
    def __init__(self, parameters, *, worker=_worker):
        validate_parameters(parameters, real=True)
        self.parameters = parameters
        self.lock = threading.Lock()
        self.closed = False
        context = multiprocessing.get_context('spawn')
        self.pipe, child = context.Pipe()
        self.process = context.Process(target=worker, args=(child, parameters), daemon=True)
        try:
            self.process.start()
            child.close()
            self._receive('connect', parameters['startup_timeout_s'])
        except Exception:
            child.close()
            self._terminate()
            raise

    def _terminate(self):
        self.closed = True
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=.5)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=.5)
        self.pipe.close()

    def _receive(self, operation, timeout):
        if not self.pipe.poll(timeout):
            self._terminate()
            raise TimeoutError(f'FR3 {self.parameters["host"]} SDK {operation} timed out; '
                               'worker terminated, physical stop UNCONFIRMED')
        try:
            ok, value = self.pipe.recv()
        except (EOFError, OSError) as exc:
            self._terminate()
            raise ConnectionError('FR3 SDK process exited; physical stop UNCONFIRMED') from exc
        if not ok:
            raise RuntimeError('FR3 SDK: ' + value)
        return value

    def _call(self, operation, *args):
        with self.lock:
            if self.closed:
                raise RuntimeError('FR3 transport closed; start a new session to reconnect')
            timeout = self.parameters['operation_timeout_s'] if operation in {
                'activate', 'stop', 'close'} else self.parameters['rpc_timeout_s']
            try:
                self.pipe.send((operation, args))
            except (BrokenPipeError, EOFError, OSError) as exc:
                self._terminate()
                raise ConnectionError('FR3 SDK process unavailable; physical stop UNCONFIRMED') from exc
            return self._receive(operation, timeout)

    def read(self):
        return self._call('read')

    def write(self, q):
        return self._call('write', q)

    def activate(self):
        return self._call('activate')

    def stop(self):
        return self._call('stop')

    def forward_kinematics(self, q):
        return self._call('forward_kinematics', q)

    def close(self):
        if self.closed:
            return
        try:
            self._call('close')
        finally:
            self.process.join(timeout=.5)
            self._terminate()
