"""控制级指标：指令节奏、每拍步长、IK 延迟、奇异程度。

用法:
    PYTHONPATH=src/common/morla_core:src/common/morla_teleop:src/robots/airbot/morla_mmk2_teleop \
      /usr/bin/python3 scripts/control.py [--tcp] ~/.local/share/morla/logs/<run_dir> [...]

PYTHONPATH 只在需要 --tcp（用 MMK2 URDF 把指令关节角正解成末端跳变）时必需；
缺了也照样输出其余指标。FR3/CR5 的 trace 没有 MMK2 的 URDF，--tcp 只对左右臂端口生效。

每个 source_port 输出:
  ticks / rate            下发的指令条数和频率
  dt                     相邻指令间隔（p50/p95/max，>40 ms 占比）
  output step             每拍实际下发的关节步长（p50/p95），==cap 的比例，零步长比例
  solver step             求解器内部步长（p50）及其饱和比例
  candidate age           本条指令所用的解，对应的是多久以前的那次请求
  solve                   IK 求解耗时（p50/p95）
  joint jump              相邻两条指令之间的最大关节差（p50/p95）
  tcp jump                同一差值在末端空间的距离（需要 --tcp）
  sigma / damping         最小奇异值、自适应阻尼（p50/p95）
"""
import collections
import glob
import json
import sys

import numpy as np


def percentile(values, q):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, q)) if len(values) else float('nan')


def load(run):
    rows = collections.defaultdict(list)
    for path in sorted(glob.glob(run.rstrip('/') + '/trace-*.jsonl')):
        for line in open(path):
            if '"kind":"control"' not in line:
                continue
            row = json.loads(line)
            data = row['data']
            for name, report in (data.get('reports') or {}).items():
                if 'solver_report' in report:
                    report = report['solver_report']
                rows[data.get('source_port') or name].append((row['monotonic_ns'], data, report))
    return rows


def kinematics(port):
    try:
        from morla_mmk2_teleop.urdf_ik import create_kinematics
    except Exception:
        return None
    for side in ('left', 'right'):
        if port.startswith(side):
            try:
                return create_kinematics({'plugin_id': 'morla.airbot.mmk2.%s_arm' % side})
            except Exception:
                return None
    return None


def report(run, tcp=False):
    print(run.rstrip('/').split('/')[-1])
    for port, entries in sorted(load(run).items()):
        if len(entries) < 20:
            continue
        t = np.asarray([entry[0] for entry in entries], float) / 1e9
        dt = np.diff(t)
        reports = [entry[2] for entry in entries]
        step = [_get(r, 'output_step_rad') for r in reports]
        cap = [_get(r, 'output_step_limit_rad', 'max_step_rad') for r in reports]
        solver = [_get(r, 'joint_step_rad') for r in reports]
        maxstep = [_get(r, 'max_step_rad') for r in reports]
        age = [(entry[0] - (entry[2].get('output_candidate_ns') or entry[2].get('sample_ns'))) / 1e6
               for entry in entries]
        solve = [(_get(r, 'completed_ns', default=float('nan')) - _get(r, 'sample_ns', default=float('nan'))) / 1e6
                 for r in reports]
        sigma = [_get(r, 'sigma_min') for r in reports]
        damping = [_get(r, 'damping') for r in reports]
        error = [_get(r, 'raw_position_error_m') for r in reports]
        targets = [entry[1].get('target_values') for entry in entries]
        jumps = [np.max(np.abs(np.subtract(b, a))) for a, b in zip(targets, targets[1:])
                 if a is not None and b is not None]
        print('  --- %s   %d ticks   %.1f Hz' % (port, len(entries), (len(t) - 1) / (t[-1] - t[0])))
        print('      dt            p50 %5.1f  p95 %5.1f  max %6.1f ms   >40ms %2.0f%%'
              % (1000 * np.median(dt), 1000 * percentile(dt, 95), 1000 * dt.max(), 100 * np.mean(dt > .04)))
        if any('output_step_rad' in r for r in reports):
            print('      output step   p50 %.4f  p95 %.4f   ==cap %2.0f%%   zero %2.0f%%'
                  % (percentile(step, 50), percentile(step, 95),
                     100 * np.nanmean(np.asarray(step, float) > .95 * np.asarray(cap, float)),
                     100 * np.nanmean(np.asarray(step, float) <= 1e-12)))
        if np.isfinite(np.asarray(maxstep, float)).any():
            print('      solver step   p50 %.4f  p95 %.4f   saturated %2.0f%%   max_step p50 %.4f'
                  % (percentile(solver, 50), percentile(solver, 95),
                     100 * np.nanmean(np.asarray(solver, float) > .999 * np.asarray(maxstep, float)),
                     percentile(maxstep, 50)))
        else:
            # 求解器不回报内部步长上限（例如 FR3 用绑定里的 max_joint_step 夹）
            print('      solver step   p50 %.4f  p95 %.4f' % (percentile(solver, 50), percentile(solver, 95)))
        print('      candidate age p50 %5.1f  p95 %5.1f  max %6.1f ms'
              % (percentile(age, 50), percentile(age, 95), np.nanmax(age)))
        print('      solve         p50 %5.1f  p95 %5.1f ms'
              % (percentile(solve, 50), percentile(solve, 95)))
        if jumps:
            print('      joint jump    p50 %.4f  p95 %.4f rad' % (np.median(jumps), np.percentile(jumps, 95)))
        print('      sigma         p50 %.4f  p05 %.4f      damping p50 %.3f  p95 %.3f'
              % (percentile(sigma, 50), percentile(sigma, 5), percentile(damping, 50), percentile(damping, 95)))
        print('      raw pos err   p50 %.4f  p95 %.4f m' % (percentile(error, 50), percentile(error, 95)))
        if tcp and 'mmk2' in run:
            solver = kinematics(port)
            if solver is not None:
                points = [solver.fk(np.asarray(q, float))[:3, 3] for q in targets if q is not None]
                distances = np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1)
                print('      tcp jump      p50 %.1f  p95 %.1f  max %.1f mm'
                      % (1000 * np.median(distances), 1000 * np.percentile(distances, 95),
                         1000 * distances.max()))
    print()


def _get(source, *names, default=float('nan')):
    for name in names:
        if isinstance(source, dict) and source.get(name) is not None:
            return float(source[name])
    return default


if __name__ == '__main__':
    arguments = sys.argv[1:]
    tcp = '--tcp' in arguments
    for argument in [a for a in arguments if a != '--tcp']:
        report(argument, tcp)
