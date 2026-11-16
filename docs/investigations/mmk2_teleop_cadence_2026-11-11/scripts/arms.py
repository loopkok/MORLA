"""逐臂统计一轮运行的遥操节奏与跟随质量（按 trace 记录的 component_id 分组）。

用法:
    /usr/bin/python3 scripts/arms.py ~/.local/share/morla/logs/<run_dir> [...]

输出每个组件（left_arm / right_arm / gripper / lift / head）的:
  control   控制帧数、时长、Hz、帧间隔 p50/p95/max、超 40ms 比例
  produced  produced_target 比例、dispatch_acknowledged 比例
  step      关节步长 p50/p95、触顶(max_step_rad)比例、零步长比例
  solver    求解耗时 p50/p95/max、sigma_min p50、raw_position_error p50/p95
  reason    reports 里的 reason 分布（tracking / deadline_hold / ...）
  gate      该绑定被 control_gate 拦下的原因分布
输入侧另给: 输入帧 Hz、每侧 pose/grip/controller_state 端口出现次数、每 pose 的平均帧数。
"""
import glob
import json
import os
import statistics
import sys

ARMS = ('left_arm', 'right_arm')


def pct(values, q):
    if not values:
        return float('nan')
    values = sorted(values)
    if len(values) == 1:
        return float(values[0])
    pos = (len(values) - 1) * q
    lo, hi = int(pos), min(int(pos) + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (pos - lo)


def ms(ns):
    return ns / 1e6


def summary(values, scale=1.0):
    if not values:
        return 'n/a'
    s = sorted(values)
    return 'p50=%.2f p95=%.2f max=%.2f' % (pct(s, .5) * scale, pct(s, .95) * scale, s[-1] * scale)


def load(run):
    rows = []
    for path in sorted(glob.glob(os.path.join(run, 'trace-*.jsonl'))):
        with open(path) as handle:
            for line in handle:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
    return rows


def analyse(run):
    rows = load(run)
    inputs = [r for r in rows if r['kind'] == 'input']
    controls = [r for r in rows if r['kind'] == 'control']
    gates = [r for r in rows if r['kind'] == 'control_gate']
    handoffs = [r for r in rows if r['kind'] == 'handoff']
    print('=' * 100)
    print(os.path.basename(run.rstrip('/')))

    # 输入侧
    if inputs:
        span = inputs[-1]['monotonic_ns'] - inputs[0]['monotonic_ns']
        poses = {side: sum(1 for r in inputs if (side + '_pose') in r['data'].get('ports', {}))
                 for side in ('left', 'right')}
        holds = {side: sum(1 for r in inputs if r['data'].get('ports', {}).get(side + '_hold') is True)
                 for side in ('left', 'right')}
        print('input   %d 帧 / %.1fs = %.1f Hz   帧间隔 %s'
              % (len(inputs), span / 1e9, len(inputs) / (span / 1e9),
                 summary([b['monotonic_ns'] - a['monotonic_ns'] for a, b in zip(inputs, inputs[1:])])))
        print('        pose 帧: left=%d right=%d (每 pose %.1f 个输入帧)   hold=true: left=%d right=%d'
              % (poses['left'], poses['right'],
                 len(inputs) / max(1, max(poses.values())), holds['left'], holds['right']))
        seqs = [r['data'].get('sequence') for r in inputs]
        print('        sequence 跨度 %s -> %s（%d 个输入帧，%.2f 帧/序号）'
              % (seqs[0], seqs[-1], len(inputs), len(inputs) / max(1, (seqs[-1] or 0) - (seqs[0] or 0) + 1)))

    if handoffs:
        print('handoff ' + ', '.join(
            '%s@%.1fs(%s->%s)' % (h['data'].get('phase'), (h['monotonic_ns'] - rows[0]['monotonic_ns']) / 1e9,
                                  h['data'].get('source'), h['data'].get('destination')) for h in handoffs))

    gate_counts = {}
    for gate in gates:
        key = gate.get('binding_id')
        gate_counts.setdefault(key, {})
        reason = gate['data'].get('reason')
        gate_counts[key][reason] = gate_counts[key].get(reason, 0) + 1

    by_component = {}
    for control in controls:
        by_component.setdefault(control.get('component_id'), []).append(control)

    for component in sorted(by_component):
        items = by_component[component]
        items.sort(key=lambda r: r['monotonic_ns'])
        span = items[-1]['monotonic_ns'] - items[0]['monotonic_ns']
        gaps = [b['monotonic_ns'] - a['monotonic_ns'] for a, b in zip(items, items[1:])]
        bindings = sorted({r.get('binding_id') for r in items})
        print('-' * 100)
        print('%s  (binding %s)  %d 帧 / %.1fs = %.1f Hz'
              % (component, ','.join(str(b) for b in bindings), len(items), span / 1e9,
                 len(items) / (span / 1e9) if span else float('nan')))
        print('  control  帧间隔(ms) %s   >40ms=%.0f%%'
              % (summary(gaps, 1e-6), 100.0 * sum(1 for g in gaps if g > 40e6) / max(1, len(gaps))))
        produced = sum(1 for r in items if r['data'].get('produced_target'))
        acked = sum(1 for r in items if r['data'].get('dispatch_acknowledged'))
        print('  produced %d/%d (%.0f%%)   dispatch_ack=%d (%.0f%%)'
              % (produced, len(items), 100.0 * produced / len(items), acked, 100.0 * acked / len(items)))
        print('  elapsed(ms) %s' % summary([r['data'].get('elapsed_ns') or 0 for r in items], 1e-6))
        lead = [r['monotonic_ns'] - r['data']['input_received_ns'] for r in items
                if r['data'].get('input_received_ns')]
        if lead:
            print('  input->control 延迟(ms) %s' % summary(lead, 1e-6))

        reports = [r['data'].get('reports') or {} for r in items]
        plugin = next((name for rep in reports for name in rep), None)
        if plugin:
            plug = [rep.get(plugin) or {} for rep in reports]
            reasons = {}
            for rec in plug:
                reasons[rec.get('reason')] = reasons.get(rec.get('reason'), 0) + 1
            print('  reason    ' + ', '.join('%s=%d' % (k, v) for k, v in
                                             sorted(reasons.items(), key=lambda kv: -kv[1])))
            steps = [r.get('joint_step_rad') for r in plug if isinstance(r.get('joint_step_rad'), (int, float))]
            caps = [r.get('max_step_rad') for r in plug if isinstance(r.get('max_step_rad'), (int, float))]
            cap = caps[-1] if caps else None
            if steps:
                print('  solver_step(rad) %s  at_solver_cap=%.0f%%  zero=%.0f%%'
                      % (summary(steps), 100.0 * sum(1 for s in steps if cap and s >= cap * .999) / len(steps),
                         100.0 * sum(1 for s in steps if s == 0) / len(steps)))
            out = [r.get('output_step_rad') for r in plug if isinstance(r.get('output_step_rad'), (int, float))]
            outcap = [r.get('output_step_limit_rad') for r in plug if isinstance(r.get('output_step_limit_rad'), (int, float))]
            scale = [r.get('output_step_scale') for r in plug if isinstance(r.get('output_step_scale'), (int, float))]
            if out:
                limit = outcap[-1] if outcap else None
                print('  output_step(rad) %s  limit=%.4f  at_out_cap=%.0f%%  zero_step=%.0f%%'
                      % (summary(out), limit or float('nan'),
                         100.0 * sum(1 for s in out if limit and s >= limit * .999) / len(out),
                         100.0 * sum(1 for s in out if s == 0) / len(out)))
            if scale:
                print('  output_step_scale %s  scale==0=%.0f%%' % (summary(scale), 100.0 * sum(1 for s in scale if s == 0) / len(scale)))
            err = [r.get('raw_position_error_m') for r in plug
                   if isinstance(r.get('raw_position_error_m'), (int, float))]
            if err:
                print('  raw_err(m) %s' % summary(err))
            out_err = [r.get('output_position_error_m') for r in plug
                       if isinstance(r.get('output_position_error_m'), (int, float))]
            if out_err:
                print('  out_err(m) %s' % summary(out_err))
            solve = [r.get('elapsed_s') for r in plug if isinstance(r.get('elapsed_s'), (int, float))]
            if solve:
                print('  solve(s)  %s' % summary(solve))
            sigma = [r.get('sigma_min') for r in plug if isinstance(r.get('sigma_min'), (int, float))]
            if sigma:
                print('  sigma_min %s' % summary(sigma))
            age = []
            for rec in plug:
                if isinstance(rec.get('sample_ns'), int) and isinstance(rec.get('completed_ns'), int):
                    age.append(rec['completed_ns'] - rec['sample_ns'])
            if age:
                print('  候选龄期(ms) %s' % summary(age, 1e-6))

    if gate_counts:
        print('-' * 100)
        for binding in sorted(gate_counts):
            print('gate %-10s %s' % (binding, ', '.join('%s=%d' % (k, v) for k, v in
                                                        sorted(gate_counts[binding].items(), key=lambda kv: -kv[1]))))


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        analyse(argument)
