"""把一次运行的 trace 压成可随身带走的小体积证据（CSV + summary.json）。

用法:
    /usr/bin/python3 scripts/export.py <run_dir> <out_dir>

只读 trace-*.jsonl，输出:
  snapshots.csv       每个位姿快照一行：哪只手、APK 时间戳、MORLA 收到时刻
                      （直接证明"每个快照只装一只手、左右轮流"）
  commands.csv        每条下发指令一行：步长、上限、解算耗时、candidate age、σ_min、阻尼
  rates_per_second.csv  每秒的帧数/左右样本数/左右指令数，便于画图
  summary.json        本次运行的关键百分位（report.py 用它生成报告）

一份运行通常压到几百 KB，换台电脑不带 trace 也能核对。
"""
import collections
import csv
import glob
import json
import os
import sys

import numpy as np


def percentile(values, q):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, q)) if len(values) else None


def number(source, *names):
    for name in names:
        if isinstance(source, dict) and source.get(name) is not None:
            try:
                return float(source[name])
            except (TypeError, ValueError):
                return None
    return None


def scan(run):
    """input 记录里，某只手的 received_ns 变化了，就说明这一帧装的是这只手的新快照。

    单臂运行时 metadata 里只有一侧的端口，所以逐端口比较，不能假设两侧都在。
    """
    frames, snapshots = [], []
    commands = collections.defaultdict(list)
    previous = {}
    for path in sorted(glob.glob(run.rstrip('/') + '/trace-*.jsonl')):
        for line in open(path):
            if '"kind":"input"' in line:
                row = json.loads(line)
                data = row.get('data') or {}
                metadata = data.get('port_metadata') or {}
                frames.append(row['monotonic_ns'])
                for side in ('left', 'right'):
                    meta = metadata.get(side + '_pose')
                    if not meta:
                        continue
                    received = meta.get('received_ns')
                    if received is not None and received != previous.get(side):
                        snapshots.append((row['monotonic_ns'], side,
                                          meta.get('source_time_ns'), received,
                                          meta.get('sample_sequence')))
                    previous[side] = received
            elif '"kind":"control"' in line:
                row = json.loads(line)
                data = row['data']
                port = data.get('source_port') or ''
                if not (port.startswith('left') or port.startswith('right')):
                    continue
                for report in (data.get('reports') or {}).values():
                    if 'solver_report' in report:
                        report = report['solver_report']
                    commands[port].append((row['monotonic_ns'], report, data.get('target_values')))
    return frames, snapshots, commands


def export(run, out):
    os.makedirs(out, exist_ok=True)
    frames, snapshots, commands = scan(run)
    with open(os.path.join(out, 'snapshots.csv'), 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['frame_ns', 'side', 'arrival_wall_ns', 'received_ns', 'sample_sequence'])
        writer.writerows(snapshots)

    summary = {'run': run.rstrip('/').split('/')[-1], 'port': {}}
    frame_seconds = collections.Counter(ns // 1_000_000_000 for ns in frames)
    sample_seconds = collections.defaultdict(lambda: collections.Counter())
    command_seconds = collections.defaultdict(lambda: collections.Counter())
    for _, side, _, received, _ in snapshots:
        sample_seconds[side][received // 1_000_000_000] += 1
    with open(os.path.join(out, 'commands.csv'), 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['tick_ns', 'port', 'output_step_rad', 'output_step_limit_rad',
                         'joint_step_rad', 'max_step_rad', 'candidate_age_ms', 'solve_ms',
                         'sigma_min', 'damping', 'raw_position_error_m'])
        for port, entries in sorted(commands.items()):
            previous_tick, previous_target = None, None
            pairs, jumps = [], []
            gaps, steps, solver, limits, ages, solves, sigmas, dampings, errors = ([] for _ in range(9))
            for tick, report, target in entries:
                age_source = report.get('output_candidate_ns') or report.get('sample_ns')
                age = (tick - age_source) / 1e6 if age_source else None
                solve_start, solve_end = number(report, 'sample_ns'), number(report, 'completed_ns')
                solve = (solve_end - solve_start) / 1e6 if solve_start and solve_end else None
                values = (tick, port, number(report, 'output_step_rad'),
                          number(report, 'output_step_limit_rad'), number(report, 'joint_step_rad'),
                          number(report, 'max_step_rad'), age, solve, number(report, 'sigma_min'),
                          number(report, 'damping'), number(report, 'raw_position_error_m'))
                writer.writerow(['' if value is None else value for value in values])
                if previous_tick is not None:
                    gaps.append((tick - previous_tick) / 1e9)
                previous_tick = tick
                command_seconds[port][tick // 1_000_000_000] += 1
                for bucket, value in zip((steps, solver, limits, ages, solves, sigmas, dampings, errors),
                                         (values[2], values[4], values[5], age, solve, values[8], values[9], values[10])):
                    if value is not None:
                        bucket.append(value)
                if values[2] is not None and values[3] is not None:
                    pairs.append((values[2], values[3]))
                if target is not None and previous_target is not None and len(target) == len(previous_target):
                    jumps.append(max(abs(b - a) for a, b in zip(previous_target, target)))
                previous_target = target
            step = np.asarray(steps, float)
            step_at_cap = np.asarray([step_value for step_value, _ in pairs], float)
            cap = np.asarray([cap_value for _, cap_value in pairs], float)
            summary['port'][port] = {
                'ticks': len(entries),
                'rate_hz': (len(entries) - 1) / sum(gaps) if gaps else None,
                'gap_p50_ms': percentile(np.asarray(gaps) * 1000, 50),
                'gap_p95_ms': percentile(np.asarray(gaps) * 1000, 95),
                'gap_max_ms': max(gaps) * 1000 if gaps else None,
                'gap_over_40ms_pct': 100 * float(np.mean(np.asarray(gaps) > .04)) if gaps else None,
                'output_step_p50': percentile(step, 50),
                'output_step_p95': percentile(step, 95),
                'at_cap_pct': 100 * float(np.mean(step_at_cap > .95 * cap)) if len(cap) else None,
                'zero_step_pct': 100 * float(np.nanmean(step <= 1e-12)),
                'joint_jump_p50': percentile(jumps, 50),
                'joint_jump_p95': percentile(jumps, 95),
                'solver_step_p50': percentile(solver, 50),
                'solver_step_p95': percentile(solver, 95),
                'solver_saturated_pct': (100 * float(np.nanmean(np.asarray(solver) > .999 * np.asarray(limits)))
                                         if len(limits) == len(solver) else None),
                'candidate_age_p50_ms': percentile(ages, 50),
                'candidate_age_p95_ms': percentile(ages, 95),
                'candidate_age_max_ms': float(np.max(ages)) if ages else None,
                'solve_p50_ms': percentile(solves, 50),
                'solve_p95_ms': percentile(solves, 95),
                'sigma_min_p50': percentile(sigmas, 50),
                'sigma_min_p05': percentile(sigmas, 5),
                'damping_p50': percentile(dampings, 50),
                'damping_p95': percentile(dampings, 95),
                'raw_position_error_p50_m': percentile(errors, 50),
                'raw_position_error_p95_m': percentile(errors, 95),
            }

    for port_side in ('left', 'right'):
        entries = [(t, s) for (t, side, _, _, s) in snapshots if side == port_side and s is not None]
        if len(entries) < 2:
            continue
        span = (entries[-1][0] - entries[0][0]) / 1e9
        pose_hz = (len(entries) - 1) / span
        sequence_hz = (entries[-1][1] - entries[0][1]) / span
        summary['port'].setdefault(port_side + '_pose', {}).update({
            'pose_hz': pose_hz, 'sequence_per_s': sequence_hz, 'seq_per_pose': sequence_hz / pose_hz})
    if len(snapshots) > 1:
        order = [side for _, side, _, _, _ in snapshots]
        runs = []
        for side in order:
            if runs and runs[-1][0] == side:
                runs[-1][1] += 1
            else:
                runs.append([side, 1])
        summary['interleave'] = {'%s x%d' % (side, length): count
                                 for (side, length), count in sorted(collections.Counter(
                                     (side, length) for side, length in runs).items())}
        ticks = [ns for ns, _, _, _, _ in snapshots]
        intervals = np.diff(np.asarray(ticks, float)) / 1e6
        summary['snapshot_rate_hz'] = (len(ticks) - 1) / ((ticks[-1] - ticks[0]) / 1e9)
        summary['snapshot_interval_p50_ms'] = float(np.median(intervals))
        summary['snapshot_interval_p95_ms'] = float(np.percentile(intervals, 95))

    with open(os.path.join(out, 'rates_per_second.csv'), 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['second', 'frames', 'left_samples', 'right_samples', 'left_commands', 'right_commands'])
        seconds = sorted(set(frame_seconds) | set(sample_seconds['left']) | set(sample_seconds['right']))
        for second in seconds:
            writer.writerow([second, frame_seconds[second], sample_seconds['left'][second],
                             sample_seconds['right'][second],
                             command_seconds.get('left_pose', {}).get(second, 0),
                             command_seconds.get('right_pose', {}).get(second, 0)])
    summary['frames'] = len(frames)
    if len(frames) > 1:
        summary['frame_rate_hz'] = (len(frames) - 1) / ((frames[-1] - frames[0]) / 1e9)
    with open(os.path.join(out, 'summary.json'), 'w') as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
    print('%s -> %s' % (run.rstrip('/').split('/')[-1], out))


if __name__ == '__main__':
    export(sys.argv[1], sys.argv[2])
