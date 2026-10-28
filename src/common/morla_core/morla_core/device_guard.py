"""Host-wide legacy/composable exclusion before any hardware connection.

Legacy nodes share the lane. A runtime holds the lane exclusively and delegates
its existing open file description to owned children via pass_fds; environment
strings alone cannot bypass the inode check. This prevents accidental parallel
launches, not malicious same-user processes (ROS security is separate).
"""
import fcntl
import os
from pathlib import Path

LOCK_NAME = 'active-system.lock'
DEFAULT_ROOT = '/tmp/morla-runtime-locks'


class LegacyDeviceGuard:
    def __init__(self, root=None):
        self.fd = None
        self.root = Path(root or os.environ.get('MORLA_RESOURCE_ROOT', DEFAULT_ROOT))
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        info = self.root.stat()
        if info.st_uid != os.getuid() or info.st_mode & 0o022:
            raise PermissionError('UNSAFE_RESOURCE_LOCK_DIRECTORY')
        path = self.root / LOCK_NAME
        inherited = os.environ.get('MORLA_RUNTIME_LOCK_FD')
        if inherited is not None:
            try:
                descriptor = int(inherited)
                a, b = os.fstat(descriptor), path.stat()
                if (a.st_dev, a.st_ino) != (b.st_dev, b.st_ino):
                    raise ValueError('lock inode mismatch')
                # Reassert ownership on the inherited open-file description.
                # This cannot acquire against another runtime's exclusive lock.
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.fd = os.dup(descriptor)
                return
            except (ValueError, OSError) as exc:
                raise RuntimeError('INVALID_RUNTIME_LOCK_DELEGATION') from exc
        descriptor = os.open(str(path), os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except OSError as exc:
            os.close(descriptor)
            raise RuntimeError('COMPOSABLE_RUNTIME_OWNS_DEVICES') from exc
        self.fd = descriptor

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __del__(self):
        self.close()
