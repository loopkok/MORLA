"""输入/指令节奏：接收端样本、引擎消费、下发指令的每一跳。

用法:
    /usr/bin/python3 scripts/rates.py ~/.local/share/morla/logs/<run_dir> [...]

每个 run 只读 trace-*.jsonl，逐端口输出:
  pose          引擎实际收到的位姿样本 / 秒（trace 里 received_ns 变化的次数）
  sequence      接收器给该手分配的样本序号，每秒前进多少
                （quest_snapshot.py 里每次 update() +1：位姿一次、landmarks 一次）
  seq/pose      平均每个"到达引擎的位姿"对应多少次接收端更新
                =2 表示到位姿全部进了引擎；≈4.6 表示约一半以上在到达引擎前被丢掉
  command       kind=control 中该 source_port 的记录 / 秒（真正下发的指令）
  interleave    位姿快照的左右手排列与快照间隔

读数要点：sequence 由接收器（quest3_udp_mocap）在解析完 APK 数据后自增，
所以它衡量的是"APK 交给接收器多少"，与引擎消费快慢无关。seq/pose 才是
"从接收器发布到引擎拿到"这一跳有没有丢。
"""
import collections
import glob
import json
import sys

import numpy as np


def advance(entries):
    """(time, counter) 对的计数器前进速度 / 秒。"""
    if len(entries) < 2 or None in (entries[0][1], entries[-1][1]):
        return float('nan')
    return (entries[-1][1] - entries[0][1]) / ((entries[-1][0] - entries[0][0]) / 1e9)


def rate(timestamps):
    t = sorted(set(timestamps))
    return (len(t) - 1) / ((t[-1] - t[0]) / 1e9) if len(t) > 1 else float('nan')


def scan(run):
    frames, received, commands = [], collections.defaultdict(list), collections.defaultdict(list)
    sequence = collections.defaultdict(list)
    order, order_ns = [], []
    previous = {}
    for path in sorted(glob.glob(run.rstrip('/') + '/trace-*.jsonl')):
        for line in open(path):
            if '"kind":"input"' in line:
                row = json.loads(line)
                data = row.get('data') or {}
                metadata = data.get('port_metadata') or {}
                frames.append(row['monotonic_ns'])
                changed = False
                for port, side in (('left_pose', 'L'), ('right_pose', 'R')):
                    meta = metadata.get(port)
                    if not meta:
                        continue
                    now = meta.get('received_ns')
                    if now is not None and now != previous.get(port):
                        received[port].append(now)
                        sequence[port].append((row['monotonic_ns'], meta.get('sample_sequence')))
                        order.append(side)
                        changed = True
                    previous[port] = now
                if changed:
                    order_ns.append(row['monotonic_ns'])
            elif '"kind":"control"' in line:
                row = json.loads(line)
                commands[row['data'].get('source_port')].append(row['monotonic_ns'])
    return frames, received, sequence, commands, order, order_ns


def interleave(order):
    if not order:
        return None
    runs = []
    for side in order:
        if runs and runs[-1][0] == side:
            runs[-1][1] += 1
        else:
            runs.append([side, 1])
    return collections.Counter((side, n) for side, n in runs)


def report(run):
    frames, received, sequence, commands, order, order_ns = scan(run)
    print(run.rstrip('/').split('/')[-1])
    print('  frames                  %7.1f Hz' % rate(frames))
    for port in ('left_pose', 'right_pose'):
        samples = received.get(port) or []
        if len(samples) < 3:
            continue
        pose = rate(samples)
        seq = advance(sequence[port])
        line = '  %-15s pose %6.1f Hz   sequence %6.1f /s   seq/pose %5.2f' % (port, pose, seq, seq / pose)
        if len(commands.get(port) or []) > 20:
            line += '   command %6.1f Hz' % rate(commands[port])
        print(line)
    runs = interleave(order)
    if runs and len(order_ns) > 1:
        t = np.asarray(order_ns, float) / 1e9
        dt = np.diff(t) * 1000
        print('  pose snapshots          %7.1f Hz   interval p50 %.1f p95 %.1f ms'
              % ((len(t) - 1) / (t[-1] - t[0]), np.median(dt), np.percentile(dt, 95)))
        print('  interleave              %s' % (' '.join('%sx%d=%d' % (s, n, c)
                                                        for (s, n), c in sorted(runs.items()))))
    print()


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        report(argument)
