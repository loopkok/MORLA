"""Exclusive, supervised subprocess lane for the external piano engine."""
from __future__ import annotations

from collections import deque
import copy
import fcntl
import json
import os
from pathlib import Path
import secrets
import signal
import stat
import subprocess
import sys
import threading
import time
import uuid

from .piano_store import PianoConflict, PianoError, PianoStore, canonical
from .piano_calibration import MANUAL_OPERATIONS, validate_command
from .piano_maintenance import validate_maintenance
from .piano_processes import SCOPE_ENV, stop_owned


OPERATIONS = {"prepare", "home", "play", "capture", "finger", "shift", "refresh", "initialize_can", "reset", "start_hand_feedback"} | MANUAL_OPERATIONS


class PianoManager:
    LEASE_SECONDS = 6.0
    IDLE_LEASE_SECONDS = None
    IDLE_STATES = {"CONNECTED", "WAITING_FEEDBACK", "READY", "CALIBRATION_IDLE", "STOPPED", "FINISHED", "FAULT"}
    STATUS_SECONDS = 2.5
    CLOSE_SECONDS = 10.0

    def __init__(self, store: PianoStore, python=None):
        self.store = store
        self.python = python or os.environ.get("MORLA_PIANO_PYTHON", sys.executable)
        self.lock = threading.RLock()
        self._write_lock = threading.Lock()
        self.proc = self.drivers = None
        self._active = False
        self._token = None
        self._lease_seen = 0.0
        self._state = {"state": "DISCONNECTED", "error": None}
        self.mode = None
        self._config = None
        self._directory = None
        self._job = None
        self._result = None
        self._logs = deque(maxlen=100)
        self._generation = 0
        self._closing = False
        self._thread = None
        self._resource_fds = []
        self._worker_env = None
        self._recovery = None
        self._snapshot_seen = None
        self._started_at = 0.0

    @property
    def active(self):
        with self.lock:
            return self._active

    def snapshot(self):
        with self.lock:
            value = copy.deepcopy({**self._state, "active": self._active, "mode": self.mode,
                                  "session_id": self._directory.name if self._directory else None,
                                  "busy": self._job is not None, "job": self._job, "result": self._result,
                                  "closing": self._closing, "logs": list(self._logs),
                                  "recovery": self._recovery,
                                  "config_revision": self._config and self._config["revision"]})
            value["browser_heartbeat_age_s"] = max(0., time.monotonic() - self._lease_seen) if self._active else None
            value["browser_heartbeat_timeout_s"] = self._lease_seconds()
            if self._snapshot_seen is not None:
                elapsed = max(0., time.monotonic() - self._snapshot_seen)
                value["feedback_age_s"] = {k: age + elapsed if age is not None else None
                                          for k, age in value.get("feedback_age_s", {}).items()}
                if self._active and elapsed > self.STATUS_SECONDS:
                    value.update(connected=False, maintenance_allowed=False)
            return value

    def editable(self):
        if self.active:
            raise PianoConflict("请先断开钢琴会话，再修改配置或曲谱；旧准备状态不可复用")

    def _lease_seconds(self):
        # Browsers can suspend background tabs indefinitely. An idle session
        # belongs to the server until explicitly closed; executing commands
        # still require the short motion watchdog and fresh browser ownership.
        return (self.IDLE_LEASE_SECONDS if self._job is None and self._state.get("state") in self.IDLE_STATES
                else self.LEASE_SECONDS)

    def _lease_expired(self):
        timeout = self._lease_seconds()
        return timeout is not None and time.monotonic() - self._lease_seen > timeout

    def authorize(self, token):
        if not self._active or not isinstance(token, str) or not self._token or not secrets.compare_digest(token, self._token):
            raise PianoConflict("钢琴会话由另一页面持有，或会话已失效")
        if self._closing or self._lease_expired():
            raise PianoConflict("钢琴会话正在关闭或浏览器心跳已过期")

    def heartbeat(self, token):
        with self.lock:
            self.authorize(token)
            self._lease_seen = time.monotonic()

    def _write(self, message):
        with self._write_lock:
            proc = self.proc
            if not proc or proc.poll() is not None or proc.stdin is None:
                raise PianoConflict("钢琴执行进程已退出")
            try:
                proc.stdin.write(canonical(message) + "\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError) as exc:
                raise PianoConflict("钢琴执行进程连接已断开") from exc

    @staticmethod
    def _spawn(command, *, env=None, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, pass_fds=()):
        # No bash, command interpolation or inherited ignored SIGINT.
        child_env = dict(os.environ if env is None else env)
        scope = uuid.uuid4().hex
        child_env[SCOPE_ENV] = scope
        proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=stdout, stderr=stderr,
                                text=True, bufsize=1, start_new_session=True, env=child_env, pass_fds=pass_fds)
        proc.piano_scope = scope
        return proc

    def _resource_lock(self, mode):
        if mode == "software":
            return
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if not domain.isdigit() or not 0 <= int(domain) <= 232:
            raise PianoError("ROS_DOMAIN_ID 必须为 0 至 232 的整数")
        keys = ["domain-" + domain]
        if mode == "hardware":
            keys.append("physical-hardware")
        try:
            for key in keys:
                path = Path("/tmp") / f"morla-piano-{os.getuid()}-{key}.lock"
                fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    os.close(fd)
                    raise PianoConflict("其他 Web/钢琴进程已占用 ROS 域或实体机器人")
                self._resource_fds.append(fd)
        except Exception:
            self._release_resource_lock()
            raise

    def _release_resource_lock(self):
        for fd in self._resource_fds:
            os.close(fd)
        self._resource_fds.clear()

    @staticmethod
    def _check_devices(options):
        ports = [Path(options[s + "_serial_port"]).resolve(strict=True) for s in ("left", "right")]
        if ports[0] == ports[1] or ports[0].stat().st_rdev == ports[1].stat().st_rdev:
            raise PianoError("左右手串口实际指向同一设备；请核对 by-id 路径")
        for port in ports:
            if not stat.S_ISCHR(port.stat().st_mode) or not os.access(port, os.R_OK | os.W_OK):
                raise PianoError(f"串口不是可读写的字符设备：{port}")
        for side in ("left", "right"):
            interface = Path("/sys/class/net") / options[side + "_can"]
            if (not interface.exists() or (interface / "type").read_text().strip() != "280"
                    or not int((interface / "flags").read_text().strip(), 16) & 1):
                raise PianoError(f"{side} CAN 接口不存在、类型错误或未启用；请先检查 CAN 接口")

    def open(self, mode, expected_revision, options=None):
        options = options or {}
        if not isinstance(options, dict):
            raise PianoError("设备参数必须为对象")
        with self.lock:
            self.editable()
            if mode not in ("software", "ros_fake", "hardware"):
                raise PianoError("无效运行模式")
            current = self.store.config()
            if current["revision"] != expected_revision:
                raise PianoConflict("配置版本已改变，请重新载入")
            self.store.model()
            self._active, self._closing = True, False
            self._generation += 1
            generation = self._generation
            self.mode, self._config = mode, current
            self._state = {"state": "STARTING", "error": None}
            self._snapshot_seen = None
            self._started_at = time.monotonic()
            self._job, self._result = None, None
            self._recovery = None
            self._logs.clear()
            self._token, self._lease_seen = secrets.token_urlsafe(32), time.monotonic()
            self._directory = self.store.root / "sessions" / uuid.uuid4().hex
            try:
                self._resource_lock(mode)
                self.store.atomic(self._directory / "config.json", current["config"])
                env = dict(os.environ)
                # Installed and source builds both find this worker without changing global PYTHONPATH.
                package_root = str(Path(__file__).resolve().parent.parent)
                env["PYTHONPATH"] = package_root + os.pathsep + env.get("PYTHONPATH", "")
                self._worker_env = env
                if mode in ("ros_fake", "hardware"):
                    launch = self.store.project / "launch/piano_drivers.launch.py"
                    if not launch.is_file():
                        raise PianoError("外部项目缺少 piano_drivers.launch.py")
                    command = ["ros2", "launch", str(launch), "mode:=" + ("fake" if mode == "ros_fake" else "hardware"),
                               "config:=" + str(self._directory / "config.json")]
                    if mode == "hardware":
                        for key in ("left_serial_port", "right_serial_port", "left_can", "right_can"):
                            value = options.get(key, "")
                            if not isinstance(value, str) or not value or len(value) > 256 or any(char in value for char in "\n\r\x00"):
                                raise PianoError(f"请填写 {key}")
                            if key.endswith("serial_port") and (not value.startswith("/dev/") or ".." in Path(value).parts):
                                raise PianoError("串口必须为 /dev/ 下的设备")
                            if key.endswith("can") and (len(value) > 15 or not all(c.isalnum() or c in "_-" for c in value)):
                                raise PianoError("无效 CAN 接口")
                            command.append(key + ":=" + value)
                        if options["left_serial_port"] == options["right_serial_port"] or options["left_can"] == options["right_can"]:
                            raise PianoError("左右手串口及左右臂 CAN 必须分别使用不同设备")
                        self._check_devices(options)
                    self.drivers = self._spawn(command, env=env, pass_fds=tuple(self._resource_fds))
                    threading.Thread(target=self._read_driver_logs, args=(self.drivers, generation), daemon=True).start()
                worker = [self.python, "-m", "morla_web_manager.piano_worker", "--project", str(self.store.project),
                          "--config", str(self._directory / "config.json"), "--mode", mode,
                          "--log", str(self._directory / "events.jsonl")]
                if self.drivers:
                    worker.extend(["--driver-pgid", str(self.drivers.pid)])
                    worker.extend(["--driver-scope", self.drivers.piano_scope])
                self.proc = self._spawn(worker, env=env, stderr=subprocess.PIPE, pass_fds=tuple(self._resource_fds))
                threading.Thread(target=self._read_worker, args=(self.proc, generation), daemon=True).start()
                threading.Thread(target=self._read_stderr, args=(self.proc, generation), daemon=True).start()
                self._thread = threading.Thread(target=self._supervise, args=(self.proc, self.drivers, generation), daemon=True)
                self._thread.start()
            except Exception:
                remaining = (self._kill_group(self.proc) or []) + (self._kill_group(self.drivers) or [])
                if remaining:
                    self._closing = True
                    self._state.update(state="CLEANUP_FAILED", error=f"启动失败，驱动尚未释放：{remaining}")
                    self._thread = threading.Thread(target=self._finish_cleanup,
                        args=(self.proc, self.drivers, generation, False, "启动失败，驱动已退出"), daemon=True)
                    self._thread.start()
                    raise
                self.proc = self.drivers = None
                self._release_resource_lock()
                self._active, self._token = False, None
                raise
            return {"token": self._token, "session": self.snapshot()}

    def command(self, token, operation, **arguments):
        with self.lock:
            self.authorize(token)
            if operation not in OPERATIONS:
                raise PianoError("无效操作")
            if time.monotonic() - self._lease_seen > self.LEASE_SECONDS:
                raise PianoConflict("请先恢复页面心跳，再发起操作；不会自动恢复旧动作")
            if self._job is not None:
                raise PianoConflict("已有钢琴操作正在执行；停止始终可用")
            message = {"operation": operation}
            if operation in ("reset", "initialize_can", "start_hand_feedback"):
                validate_maintenance(self.mode, self._state.get("state"), self._state.get("motion_claimed"), arguments.get("calibration_confirmed"))
            if operation in MANUAL_OPERATIONS:
                message.update(validate_command(operation, arguments, self._config["config"]))
            if operation == "play":
                prepare_first = arguments.get("prepare_first") is True
                if prepare_first:
                    if self._state.get("state") not in {"FINISHED", "STOPPED"}:
                        raise PianoConflict("重新准备并演奏仅用于演奏结束或已停止状态")
                    if self._state.get("replay_direct_allowed") is not True:
                        raise PianoConflict("当前位置不能直接重新演奏，请先移动到初始位")
                elif self._state.get("state") != "READY" or any((self._state.get("wrist_shifts") or {}).values()):
                    raise PianoConflict("演奏前先准备到初始位")
                if self.mode == "hardware" and not self._config["config"]["calibrated"]:
                    raise PianoConflict("未认证的配置只能进行标定试动")
                score, _ = self.store.plan(arguments.get("score_id", ""), arguments.get("tempo_bpm"), self._config["config"])
                score_path = self._directory / (uuid.uuid4().hex + ".json")
                self.store.atomic(score_path, score)
                message["score_path"] = str(score_path)
                message["prepare_first"] = prepare_first
            if operation in ("finger", "shift"):
                if arguments.get("side") not in ("left", "right"):
                    raise PianoError("请选择左手或右手")
                message["side"] = arguments["side"]
            if operation == "finger":
                if arguments.get("finger") not in ("index", "middle", "ring", "pinky"):
                    raise PianoError("只允许四指，不使用拇指")
                message["finger"] = arguments["finger"]
            if operation == "shift":
                shift = arguments.get("shift_white")
                if isinstance(shift, bool) or not isinstance(shift, int):
                    raise PianoError("移位必须为整数")
                self.store.model().arm_target(self._config["config"], arguments["side"], shift)
                message["shift_white"] = shift
            message["calibration_confirmed"] = arguments.get("calibration_confirmed") is True
            message["job_id"] = uuid.uuid4().hex
            self._job = {"id": message["job_id"], "operation": operation, "started_at": time.time()}
            self._result = None
            try:
                self._write(message)
            except Exception:
                self._job = None
                raise
            return copy.deepcopy(self._job)

    def stop(self, *, emergency=False):
        with self.lock:
            if not self._active:
                return
            self._write({"operation": "estop" if emergency else "stop", "job_id": uuid.uuid4().hex})

    def disconnect(self, token=None, *, force=False):
        with self.lock:
            if not self._active:
                return
            if not force:
                self.authorize(token)
            self._closing = True
            self._state["state"] = "CLOSING"
            try:
                self._write({"operation": "shutdown", "job_id": uuid.uuid4().hex})
            except PianoConflict:
                pass

    @staticmethod
    def _kill_group(proc):
        if not proc:
            return []
        if getattr(proc, "piano_scope", None) and Path("/proc/self/stat").exists():
            remaining = stop_owned(proc.piano_scope, proc.pid)
            try:
                proc.wait(timeout=.1)
            except subprocess.TimeoutExpired:
                pass
            return remaining
        # A process group can outlive the launch parent. Do not skip it on poll().
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                if sig == signal.SIGINT and proc.poll() is None:
                    proc.send_signal(sig)  # ROS launch forwards it once to each child.
                else:
                    os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                continue
            # SIGINT normally lets ROS launch reap its children.
            if sig == signal.SIGINT:
                try:
                    os.killpg(proc.pid, 0)
                except ProcessLookupError:
                    break

    def _read_worker(self, proc, generation):
        for line in proc.stdout:
            try:
                payload = json.loads(line)
            except ValueError:
                with self.lock:
                    self._logs.append(line.rstrip()[:1000])
                continue
            with self.lock:
                if generation != self._generation:
                    return
                if payload.get("kind") == "snapshot":
                    self._snapshot_seen = time.monotonic()
                    self._state = {key: value for key, value in payload.items() if key != "kind"}
                elif payload.get("kind") == "result":
                    self._result = payload
                    if self._job and payload.get("job_id") == self._job["id"]:
                        self._job = None
                    if payload.get("error"):
                        self._logs.append(payload["error"])

    def _read_stderr(self, proc, generation):
        for line in proc.stderr:
            with self.lock:
                if generation != self._generation:
                    return
                self._logs.append(line.rstrip()[:1000])

    def _read_driver_logs(self, proc, generation):
        path = self._directory / "drivers.log"
        with path.open("a", encoding="utf-8") as log:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                with self.lock:
                    if generation != self._generation:
                        return
                    self._logs.append(line.rstrip()[:1000])

    def _supervise(self, proc, drivers, generation):
        closing_at = None
        shutdown_reason = None
        worker_unresponsive = False
        while proc.poll() is None:
            with self.lock:
                if generation != self._generation:
                    return
                expired = self._lease_expired()
                driver_failed = drivers is not None and drivers.poll() is not None
                worker_stale = (time.monotonic() - self._snapshot_seen > self.STATUS_SECONDS
                                if self._snapshot_seen is not None else time.monotonic() - self._started_at > 15)
                if (expired or driver_failed or worker_stale) and not self._closing:
                    shutdown_reason = ("浏览器心跳过期，已停止并释放钢琴会话" if expired else
                                       "钢琴驱动已退出，会话已关闭" if driver_failed else
                                       "执行进程反馈心跳中断，已停止并关闭会话")
                    self._state["error"] = shutdown_reason
                    self.disconnect(force=True)
                if worker_stale:
                    worker_unresponsive = True
                    break  # It cannot process shutdown or cancel a native J path.
                if self._closing:
                    if closing_at is None:
                        closing_at = time.monotonic()
                    elif time.monotonic() - closing_at > self.CLOSE_SECONDS:
                        break
                try:
                    self._write({"operation": "heartbeat"})
                except PianoConflict:
                    break
            time.sleep(.2)
        # Worker gets a chance to release fingers/request estop before its drivers disappear.
        unexpected_exit = not self._closing
        if (unexpected_exit or worker_unresponsive) and self.mode != "software" and drivers is not None and drivers.poll() is None:
            command = [self.python, "-m", "morla_web_manager.piano_worker", "--project", str(self.store.project),
                       "--config", str(self._directory / "config.json"), "--mode", self.mode,
                       "--log", str(self._directory / "events.jsonl"), "--failsafe"]
            try:
                recovery = self._spawn(command, env=self._worker_env, pass_fds=tuple(self._resource_fds))
                try:
                    output, _ = recovery.communicate(timeout=3)
                    with self.lock:
                        self._recovery = {"requested": True, "exit_code": recovery.returncode}
                        self._logs.append("执行进程异常退出，已执行仅急停恢复；请核验实体状态")
                        if output:
                            self._logs.append(output[-2000:])
                finally:
                    self._kill_group(recovery)
            except Exception as exc:
                with self.lock:
                    self._recovery = {"requested": True, "error": str(exc)}
                    self._logs.append("执行进程异常退出，急停恢复失败：" + str(exc))
        self._finish_cleanup(proc, drivers, generation, unexpected_exit, shutdown_reason)

    def _finish_cleanup(self, proc, drivers, generation, unexpected_exit, shutdown_reason):
        remaining = self._kill_group(proc) or []
        remaining += self._kill_group(drivers) or []
        # Do not announce a released lane or drop locks while SDK descendants
        # still hold devices. Retrying is safe and confined to this session.
        while remaining:
            with self.lock:
                self._closing = True
                self._state.update(state="CLEANUP_FAILED", connected=False, motion_claimed=False,
                                   maintenance_allowed=False, error=f"钢琴驱动尚未退出，保持设备占用锁：{remaining}")
            time.sleep(1)
            remaining = (self._kill_group(proc) or []) + (self._kill_group(drivers) or [])
        with self.lock:
            if generation == self._generation:
                self._release_resource_lock()
                if self._job is not None:
                    self._result = {"ok": False, "error": "执行进程退出，操作未完成", "job_id": self._job["id"]}
                self._job = None
                self._active, self._token, self._closing = False, None, False
                self._state["state"] = "DISCONNECTED"
                self._state["connected"] = False
                self._state["motion_claimed"] = False
                self._state["maintenance_allowed"] = False
                if unexpected_exit:
                    self._state["error"] = "执行进程异常退出，已释放会话；请核验设备状态及急停恢复记录"
                elif shutdown_reason:
                    self._state["error"] = shutdown_reason
                self.proc = self.drivers = None

    def close(self):
        self.disconnect(force=True)
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=24)
