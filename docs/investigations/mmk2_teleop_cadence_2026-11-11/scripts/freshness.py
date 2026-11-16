"""输入侧新鲜度：每侧 pose 的原始采样率、被控制帧消费的比例、以及序号跳变。

用法:
    /usr/bin/python3 scripts/freshness.py ~/.local/share/morla/logs/<run_dir> [...]
"""
import glob
import json
import os
import sys

def pct(values, q):
    values = sorted(values)
    if not values:
        return float('nan')
    if len(values) == 1:
        return float(values[0])
    pos = (len(values) - 1) * q
    lo, hi = int(pos), min(int(pos) + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (pos - lo)


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
    print('=' * 100)
    print(os.path.basename(run.rstrip('/')))
    span = (inputs[-1]['monotonic_ns'] - inputs[0]['monotonic_ns']) / 1e9
    steps = [b['data']['sequence'] - a['data']['sequence'] for a, b in zip(inputs, inputs[1:])]
    print('input 帧 %d / %.1fs = %.1f Hz；sequence 增量 p50=%.1f'
          % (len(inputs), span, len(inputs) / span, pct(steps, .5)))
    for side in ('left', 'right'):
        samples = []
        last = None
        for row in inputs:
            meta = (row['data'].get('port_metadata') or {}).get(side + '_pose')
            if not meta:
                continue
            key = (meta.get('sample_sequence'), meta.get('received_ns'))
            if key != last:
                samples.append(meta['received_ns'])
                last = key
        if len(samples) < 3:
            print('  %-5s pose 新鲜采样 %d（不足）' % (side, len(samples)))
            continue
        gaps = [b - a for a, b in zip(samples, samples[1:])]
        print('  %-5s pose 新鲜采样 %d / %.1fs = %.1f Hz  间隔(ms) %s  >40ms=%.0f%%'
              % (side, len(samples), (samples[-1] - samples[0]) / 1e9,
                 len(samples) / ((samples[-1] - samples[0]) / 1e9), summary(gaps, 1e-6),
                 100.0 * sum(1 for g in gaps if g > 40e6) / len(gaps)))
    for component in ('left_arm', 'right_arm'):
        items = sorted((r for r in controls if r.get('component_id') == component),
                       key=lambda r: r['monotonic_ns'])
        if not items:
            continue
        seqs = [r['data'].get('input_sequence') for r in items if r['data'].get('input_sequence') is not None]
        recv = [r['data'].get('input_received_ns') for r in items if r['data'].get('input_received_ns')]
        print('  %-9s 控制帧 %d  输入序号跨度 %s->%s（%.2f 控制帧/序号）  输入到达间隔(ms) %s'
              % (component, len(items), seqs[0] if seqs else None, seqs[-1] if seqs else None,
                 len(items) / max(1, (seqs[-1] - seqs[0] + 1)) if seqs else float('nan'),
                 summary([b - a for a, b in zip(recv, recv[1:])], 1e-6)))
    # 求解 worker 的实际完成节奏
    for component in ('left_arm', 'right_arm'):
        solves = sorted((r for r in rows if r['kind'] == 'controller' and r.get('component_id') == component),
                        key=lambda r: r['monotonic_ns'])
        if not solves:
            continue
        gaps = [b['monotonic_ns'] - a['monotonic_ns'] for a, b in zip(solves, solves[1:])]
        seconds = (solves[-1]['monotonic_ns'] - solves[0]['monotonic_ns']) / 1e9
        print('  %-9s worker 求解 %d 帧 / %.1fs = %.1f Hz  间隔(ms) %s'
              % (component, len(solves), seconds, len(solves) / seconds if seconds else float('nan'),
                 summary(gaps, 1e-6)))


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        analyse(argument)
