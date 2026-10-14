"""Subprocess lifecycle manager for `ros2 launch`.

The monitor never imports or starts ROS nodes of the teleop stack directly.
Starting a preset spawns `bash -c "exec ros2 launch <pkg> <file> <args>"`;
stopping sends SIGINT with a grace timeout then SIGKILL — the same flow an
operator gets by pressing Ctrl-C in a terminal.

Pause/resume is handled by the caller (MonitorNode) publishing to the
existing /teleop/disarm and /teleop/armed topics; LaunchManager only tracks
the resulting state label so the UI reflects it.
"""
from __future__ import annotations

import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Callable

import yaml

from .config import (
    BACKEND_MATCH,
    STARTUP_WARMUP_S,
    STOP_SIGINT_TIMEOUT_S,
    STOP_SIGKILL_GRACE_S,
    WS_LOG_TAIL_LINES,
    _presets_path,
)


# Teleop state machine labels.
STOPPED = "stopped"
STARTING = "starting"
RUNNING = "running"
STOPPING = "stopping"
PAUSED = "paused"
START_FAILED = "start_failed"


@dataclass
class Preset:
    name: str
    package: str
    launch: str
    args: dict[str, str] = field(default_factory=dict)
    description: str = ""

    def command(self) -> str:
        arg_str = " ".join(f"{k}:={v}" for k, v in self.args.items())
        tail = f" {arg_str}" if arg_str else ""
        return f"exec ros2 launch {self.package} {self.launch}{tail}"


def load_presets() -> dict[str, Preset]:
    path = _presets_path()
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    out: dict[str, Preset] = {}
    for item in data.get("presets", []):
        p = Preset(
            name=item["name"],
            package=item["package"],
            launch=item["launch"],
            args=dict(item.get("args", {}) or {}),
            description=item.get("description", ""),
        )
        out[p.name] = p
    return out


# 遥操诊断日志注入白名单：只有这些包的 launch 声明了 `log_dir` 参数。
# 其余预设（如 MuJoCo sim）不注入——未声明参数传给 `ros2 launch` 会启动报错。
_TELEOP_LOG_PACKAGES = ("astral_teleop", "astral_arm_teleop")


def preset_with_teleop_log(preset: Preset, log_dir: str) -> Preset:
    """Return a copy of ``preset`` with ``log_dir`` injected (whitelisted only).

    非白名单预设原样返回；白名单预设用 ``dataclasses.replace`` 生成副本，不改
    presets.yaml 源。log_dir 空串也原样返回（等价于不记录）。launch 层收到
    ``log_dir`` 后每次自动建 ``{log_dir}/{stamp}[_tag]/`` 运行子目录，把遥操
    JSONL（按侧 _left/_right）落进去——不复用同一个 /tmp 文件。
    """
    if not log_dir or preset.package not in _TELEOP_LOG_PACKAGES:
        return preset
    return replace(preset, args={**preset.args, "log_dir": log_dir})


class LaunchManager:
    """Owns the single ros2 launch subprocess and its state label."""

    def __init__(self, on_log: Callable[[str], None] | None = None, lane_name: str = "遥操作") -> None:
        self._lane_name = lane_name
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._state = STOPPED
        self._preset_name = ""
        self._started_at = 0.0
        self._log_lines: deque[str] = deque(maxlen=WS_LOG_TAIL_LINES)
        self._on_log = on_log
        self._watcher: threading.Thread | None = None

    # --- state accessors ---------------------------------------------------
    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def preset(self) -> str:
        with self._lock:
            return self._preset_name

    @property
    def started_at(self) -> float:
        with self._lock:
            return self._started_at

    @property
    def pid(self) -> int | None:
        with self._lock:
            return self._proc.pid if self._proc and self._proc.poll() is None else None

    def uptime_s(self) -> float:
        with self._lock:
            if not self._started_at:
                return 0.0
            return time.time() - self._started_at

    def log_tail(self) -> list[str]:
        with self._lock:
            return list(self._log_lines)

    # --- control ------------------------------------------------------------
    def start(self, preset: Preset, *, check_orphan: bool = True) -> tuple[bool, str]:
        """启动预设进程。

        check_orphan=False 用于数采这类纯订阅者泳道：它不与任何遥操栈抢
        joint_commands，无需因别的 ros2 launch 在跑而拒绝启动。
        """
        with self._lock:
            if self._proc and self._proc.poll() is None:
                return False, f"已有{self._lane_name}进程在运行"
            # Orphan detection: refuse if a stray ros2 launch matches our backend.
            if check_orphan and _find_orphan():
                return False, "检测到残留的 ros2 launch 进程，请先停止"
            try:
                self._proc = subprocess.Popen(
                    ["bash", "-c", preset.command()],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    preexec_fn=os.setsid,
                )
            except OSError as e:
                self._state = START_FAILED
                return False, f"启动失败: {e}"
            self._preset_name = preset.name
            self._state = STARTING
            self._started_at = time.time()
            self._log_lines.clear()
        self._watcher = threading.Thread(target=self._watch, daemon=True)
        self._watcher.start()
        return True, f"启动预设: {preset.name}"

    def stop(self) -> tuple[bool, str]:
        with self._lock:
            proc = self._proc
            if not proc or proc.poll() is not None:
                self._state = STOPPED
                return False, "未运行"
            self._state = STOPPING
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        # Wait up to the SIGINT timeout in a worker so the REST call returns fast.
        threading.Thread(target=self._await_exit, args=(proc,), daemon=True).start()
        return True, "已发送停止信号 (SIGINT)"

    def mark_paused(self) -> None:
        with self._lock:
            if self._state == RUNNING:
                self._state = PAUSED

    def mark_resumed(self) -> None:
        with self._lock:
            if self._state == PAUSED:
                self._state = RUNNING

    # --- internal ----------------------------------------------------------
    def _watch(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        proc = self._proc
        warmup_deadline = time.time() + STARTUP_WARMUP_S
        for line in proc.stdout:
            line = line.rstrip()
            with self._lock:
                self._log_lines.append(line)
            if self._on_log:
                try:
                    self._on_log(line)
                except Exception:
                    pass
            # Promote starting -> running after the warm-up window.
            if self._state == STARTING and time.time() >= warmup_deadline:
                with self._lock:
                    if self._state == STARTING:
                        self._state = RUNNING
        # stdout closed: process exited.
        rc = proc.poll()
        with self._lock:
            self._state = STOPPED if rc in (None, 0, -2) else START_FAILED
            self._proc = None

    def _await_exit(self, proc: subprocess.Popen) -> None:
        try:
            proc.wait(timeout=STOP_SIGINT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=STOP_SIGKILL_GRACE_S)
            except subprocess.TimeoutExpired:
                pass
        with self._lock:
            self._state = STOPPED
            self._proc = None


def _find_orphan() -> bool:
    """Return True if a ros2 launch process exists that we did not start.

    astral_data_collect 的 launch 是监控自己的数采泳道（纯订阅者，永不与
    遥操栈冲突），显式豁免——否则数采泳道运行时会反过来挡住遥操预设启动。
    """
    pattern = re.compile(BACKEND_MATCH)
    try:
        out = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            capture_output=True, text=True, timeout=2.0,
        ).stdout
    except Exception:
        return False
    for line in out.splitlines():
        if not pattern.search(line):
            continue
        if "astral_web_monitor" in line or "astral_data_collect" in line:
            continue
        return True
    return False
