#!/usr/bin/env python3
"""Stop local MORLA services and explicit Web-port owners, never broad pkill.

Called by nero_web_start.sh with sudo. --dry-run is strictly read-only.
PID start times are checked before every signal to avoid PID reuse.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


@dataclass(frozen=True)
class Process:
    pid: int
    ppid: int
    start: str
    args: tuple[str, ...]
    exe: str = ''
    cwd: str = ''
    cgroup: str = ''


def read_process(pid: int) -> Process | None:
    try:
        directory = Path('/proc') / str(pid)
        fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        args = tuple(s.decode(errors='replace') for s in (directory / 'cmdline').read_bytes().split(b'\0') if s)
        def link(name):
            try:
                return os.readlink(directory / name)
            except OSError:
                return ''
        return Process(pid, int(fields[1]), fields[19], args,
                       link('exe'), link('cwd'), (directory / 'cgroup').read_text())
    except (OSError, ValueError, IndexError):
        return None


def snapshot() -> dict[int, Process]:
    return {int(p.name): process for p in Path('/proc').iterdir() if p.name.isdigit()
            if (process := read_process(int(p.name))) is not None}


def ancestors(processes: dict[int, Process], seeds: set[int]) -> set[int]:
    protected = {1, *seeds}
    for seed in seeds:
        current = processes.get(seed)
        while current and current.ppid not in protected:
            protected.add(current.ppid)
            current = processes.get(current.ppid)
    return protected


def is_morla_process(process: Process, workspace: str) -> bool:
    workspace = workspace.rstrip('/')
    tokens = (*process.args, process.exe)
    # Installed Python/C++ nodes remain identifiable even after launch has died.
    if any(t.startswith((workspace + '/install/', workspace + '/build/')) for t in tokens):
        return True
    args = process.args
    for index, token in enumerate(args):
        if Path(token).name == 'ros2' and index + 2 < len(args) and args[index + 1] in {'launch', 'run'}:
            package = args[index + 2]
            if package.startswith(('morla_', 'nero_', 'xnero_', 'astral_', 'quest3_', 'xhand_', 'wujihand_', 'pinch_gripper')):
                return True
            if package.startswith(workspace + '/'):
                return True
    known = {'morla_web_manager', 'nero_robot_driver', 'nero_arm_teleop',
             'morla_control_coordinator', 'astral_robot_driver', 'astral_arm_teleop',
             'quest3_hand_mocap', 'quest3_udp_mocap', 'quest3_video_streamer',
             'xhand_driver', 'xhand_driver_node'}
    if any(Path(t).name.removesuffix('.py') in known for t in tokens):
        return True
    in_workspace = process.cwd == workspace or process.cwd.startswith(workspace + '/')
    source_node = any(t.startswith(workspace + '/src/') for t in args)
    if source_node and ('--ros-args' in args or any('morla_web_manager' in t for t in args)):
        return True
    # Frontend dev/preview servers may listen on 5173, 4173, or another port.
    if in_workspace and any(Path(t).name in {'vite', 'vite.js', 'vite.mjs'} for t in args):
        return True
    if in_workspace and any(Path(t).name in {'npm', 'npm-cli.js', 'pnpm', 'yarn'} for t in args):
        return any(t in {'dev', 'preview', 'start'} for t in args)
    return False


def select_targets(processes, workspace, port_pids, protected):
    selected = {pid for pid, p in processes.items()
                if pid not in protected and (pid in port_pids or is_morla_process(p, workspace))}
    # Include generic ROS children (robot_state_publisher etc.) by ancestry.
    while True:
        children = {pid for pid, p in processes.items() if p.ppid in selected and pid not in protected}
        if children <= selected:
            return {pid: processes[pid] for pid in selected}
        selected |= children


def port_owners(port: int) -> tuple[set[int], str]:
    result = subprocess.run(['ss', '-H', '-ltnp', f'( sport = :{port} )'],
                            check=True, capture_output=True, text=True)
    return {int(p) for p in re.findall(r'pid=(\d+)', result.stdout)}, result.stdout.strip()


def same_process(process: Process) -> bool:
    live = read_process(process.pid)
    return live is not None and live.start == process.start


def signal_process(process: Process, sig: int):
    if same_process(process):
        try:
            os.kill(process.pid, sig)
        except ProcessLookupError:
            pass


def wait_for_exit(targets: dict[int, Process], seconds: float) -> dict[int, Process]:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        targets = {pid: p for pid, p in targets.items() if same_process(p)}
        if not targets:
            return {}
        time.sleep(.1)
    return {pid: p for pid, p in targets.items() if same_process(p)}


def stop_supervisors(targets):
    units = set()
    for process in targets.values():
        for line in process.cgroup.splitlines():
            if '/system.slice/' in line:
                unit = line.rsplit('/', 1)[-1]
                if (unit.endswith('.service') and any(marker in unit.lower() for marker in
                    ('morla', 'nero', 'astral', 'quest', 'rob-station'))):
                    units.add(unit)
    for unit in sorted(units):
        print(f'停止自动重启单元：{unit}', flush=True)
        subprocess.run(['systemctl', 'stop', '--no-block', unit], check=True, timeout=8)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--web-port', type=int, default=8080)
    parser.add_argument('--exclude-pid', type=int, action='append', default=[])
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    if not 1 <= args.web_port <= 65535:
        parser.error('invalid port')
    workspace = str(Path(args.workspace).resolve())
    processes = snapshot()
    protected = ancestors(processes, {os.getpid(), *args.exclude_pid})
    owners, port_text = port_owners(args.web_port)
    if owners & protected:
        raise RuntimeError(f'端口 {args.web_port} 属于启动脚本的父进程，拒绝终止自身执行链')
    if port_text and not owners:
        print(port_text, flush=True)
        if not args.dry_run:
            raise RuntimeError('无法识别端口占用 PID；请使用 sudo 运行清理助手')
    targets = select_targets(processes, workspace, owners, protected)
    if not targets:
        print(f'未发现旧 MORLA 进程；端口 {args.web_port}：' + ('有占用（只读模式无 PID 权限）' if port_text else '空闲'), flush=True)
    for pid, process in sorted(targets.items()):
        reason = f'端口 {args.web_port}' if pid in owners else 'MORLA 栈/子进程'
        print(f'{"[预览] " if args.dry_run else ""}停止 PID {pid} [{reason}] {" ".join(process.args)[:400]}', flush=True)
    if args.dry_run:
        return 0
    stop_supervisors(targets)
    roots = [p for p in targets.values() if p.ppid not in targets]
    for process in roots:
        signal_process(process, signal.SIGINT)
    remaining = wait_for_exit(targets, 1.)
    for sig, timeout in [(signal.SIGINT, 35.), (signal.SIGTERM, 5.), (signal.SIGKILL, 2.)]:
        for process in remaining.values():
            signal_process(process, sig)
        remaining = wait_for_exit(remaining, timeout)
        if not remaining:
            break
    if remaining:
        raise RuntimeError(f'进程未退出：{sorted(remaining)}')
    # Verify actual listeners and respawned roots instead of trusting kill status.
    time.sleep(.3)
    owners, port_text = port_owners(args.web_port)
    live = snapshot()
    leftovers = select_targets(live, workspace, owners, ancestors(live, protected))
    if port_text or leftovers:
        raise RuntimeError(f'清理后仍有占用/服务自动重启：PIDs={sorted(leftovers)}, port={port_text}')
    print(f'MORLA 旧栈已停止，端口 {args.web_port} 空闲。', flush=True)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'清理失败：{exc}', file=sys.stderr)
        sys.exit(1)
