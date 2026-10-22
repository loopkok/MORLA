"""Own a Quest-only receiver until a Web-managed robot stack needs its port."""
from collections import deque
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time


class QuestPreview:
    def __init__(self, enabled=False, port=8000, command_factory=None):
        self.enabled = enabled
        self.port = port
        self._command_factory = command_factory or self._command
        self._lock = threading.RLock()
        self._proc = None
        self._closed = False
        self._next_attempt = 0.0
        self._state = "disabled" if not enabled else "waiting"
        self._log = deque(maxlen=30)
        self._log_lock = threading.Lock()

    def _command(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        prefix = Path(get_package_prefix("morla_input"))
        config = Path(get_package_share_directory("morla_input")) / "config/quest3_mocap.yaml"
        return [str(prefix / "lib/morla_input/quest3_udp_mocap"), "--ros-args",
                "--params-file", str(config), "-p", "protocol:=tcp_wired",
                "-p", f"tcp_port:={self.port}", "-p", "arm_side:=both", "-p", "viz:=false"]

    def _port_free(self):
        # Bind instead of connecting: a probe must not create a Quest session.
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", self.port))
                return True
            except OSError:
                return False

    def snapshot(self):
        with self._lock, self._log_lock:
            return dict(enabled=self.enabled, state=self._state, port=self.port,
                        pid=self._proc.pid if self._proc and self._proc.poll() is None else None,
                        log_tail=list(self._log))

    def _read_output(self, process):
        for line in process.stdout:
            with self._log_lock:
                self._log.append(line.rstrip())
        process.stdout.close()

    def _stop_owned(self):
        process, self._proc = self._proc, None
        if not process:
            return
        for sig, seconds in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=seconds)
                return
            except subprocess.TimeoutExpired:
                continue
        self._proc = process
        raise RuntimeError("Quest preview did not exit; refusing to start a conflicting Profile")

    def tick(self, stack_active):
        with self._lock:
            if not self.enabled or self._closed:
                return
            if stack_active:
                self._stop_owned()
                self._state = "profile"
                return
            if self._proc and self._proc.poll() is None:
                self._state = "starting" if self._port_free() else "listening"
                return
            if time.monotonic() < self._next_attempt:
                return
            self._next_attempt = time.monotonic() + 3.0
            if not self._port_free():
                # An independently started receiver belongs to its operator.
                self._state = "external_port_owner"
                return
            try:
                self._proc = subprocess.Popen(self._command_factory(), stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
                self._state = "starting"
                threading.Thread(target=self._read_output, args=(self._proc,), daemon=True).start()
            except (OSError, LookupError) as exc:
                self._state = "failed"
                with self._log_lock:
                    self._log.append(str(exc))

    def start_stack(self, start):
        """Serialize handoff against the supervisor; never touch another owner."""
        with self._lock:
            if self.enabled:
                self._stop_owned()
                self._state = "handoff"
                if not self._port_free():
                    self._state = "external_port_owner"
                    return False, f"Quest port {self.port} belongs to an independent receiver; stop it before launching Profile"
            result = start()
            if self.enabled:
                self._state = "profile" if result[0] else "waiting"
                self._next_attempt = 0.0
            return result

    def close(self):
        with self._lock:
            self._closed = True
            self._stop_owned()
            self._state = "stopped"
