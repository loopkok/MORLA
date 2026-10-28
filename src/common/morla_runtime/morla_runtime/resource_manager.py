"""Host-local advisory locks, held by live file descriptors, never by TTL."""
import fcntl
import hashlib
import os
from pathlib import Path


class ResourceBusy(RuntimeError):
    pass


class ResourceManager:
    def __init__(self, root=None):
        self.root = Path(root or '/tmp/morla-runtime-locks')
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        info = self.root.stat()
        if info.st_uid != os.getuid() or info.st_mode & 0o022:
            raise PermissionError('UNSAFE_RESOURCE_LOCK_DIRECTORY')
        self.handles = []
        self.lane_fd = None

    def acquire(self, resources):
        try:
            for resource in sorted(set(resources)):
                key = 'active-system.lock' if resource == 'morla:active-system' else hashlib.sha256(resource.encode()).hexdigest()
                fd = os.open(str(self.root / key), os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | getattr(os, 'O_NOFOLLOW', 0), 0o600)
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    os.close(fd)
                    raise ResourceBusy('RESOURCE_BUSY: ' + resource)
                self.handles.append(fd)
                if resource == 'morla:active-system':
                    self.lane_fd = fd
        except BaseException:
            self.release()
            raise

    def release(self):
        for fd in reversed(self.handles):
            os.close(fd)
        self.handles.clear()
        self.lane_fd = None
