"""逐帧归因：新鲜 pose 的输入帧有多少最终变成该臂的 control 行，其余去哪了。

用法:
    /usr/bin/python3 scripts/frames.py ~/.local/share/morla/logs/<run_dir> [...]
"""
import glob
import json
import os
import sys

SIDES = {'left': 'left_arm', 'right': 'right_arm'}


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
    rows = [r for r in load(run) if 'record_sequence' in r]
    rows.sort(key=lambda r: r['record_sequence'])
    print('=' * 100)
    print(os.path.basename(run.rstrip('/')))
    controls = {}
    gates = {}
    for row in rows:
        if row['kind'] == 'control':
            controls.setdefault(row['data'].get('input_sequence'), set()).add(row.get('component_id'))
        elif row['kind'] == 'control_gate':
            gates.setdefault(row['data'].get('input_sequence'), set()).add(row.get('binding_id'))
    fresh = {'left': [], 'right': []}
    seen = {'left': None, 'right': None}
    for row in rows:
        if row['kind'] != 'input':
            continue
        data = row['data']
        for side in fresh:
            meta = (data.get('port_metadata') or {}).get(side + '_pose')
            if not meta:
                continue
            key = (meta.get('sample_sequence'), meta.get('received_ns'))
            if key != seen[side]:
                seen[side] = key
                fresh[side].append((data['sequence'], data.get('ports', {}).get(side + '_hold')))
    for side, component in SIDES.items():
        items = fresh[side]
        if not items:
            continue
        controlled = sum(1 for seq, _ in items if component in controls.get(seq, ()))
        held = sum(1 for seq, hold in items if hold is True)
        held_controlled = sum(1 for seq, hold in items if hold is True and component in controls.get(seq, ()))
        gated = sum(1 for seq, _ in items if component and any(
            b == ('left_vr' if side == 'left' else 'right_vr') for b in gates.get(seq, ())))
        print('%-5s 新鲜 pose %4d 帧：hold=true %4d，产出 control %4d（其中 hold=true %4d），'
              '被 control_gate 拦 %4d'
              % (side, len(items), held, controlled, held_controlled, gated))
        missing = [(seq, hold) for seq, hold in items
                   if hold is True and component not in controls.get(seq, ())
                   and not any(b == ('left_vr' if side == 'left' else 'right_vr') for b in gates.get(seq, ()))]
        print('       hold=true 但既无 control 也无 gate 记录：%d 帧' % len(missing))
    # 同一输入帧里两只臂都被控的次数（双命中）
    dual = sum(1 for seq, comps in controls.items()
               if {'left_arm', 'right_arm'} <= comps)
    print('同一输入帧内左右臂都出 control 的次数：%d' % dual)


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        analyse(argument)
