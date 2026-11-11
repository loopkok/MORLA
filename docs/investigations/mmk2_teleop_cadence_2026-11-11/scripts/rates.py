"""输入/指令节奏：Quest 快照到机械臂指令的每一跳频率。

用法:
    /usr/bin/python3 scripts/rates.py ~/.local/share/morla/logs/<run_dir> [...]

每个 run 只读 trace-*.jsonl，输出:
  frames          kind=input 的不同帧数 / 秒（引擎入口）
  <port> APK      port_metadata.<port>.source_time_ns 的不同值 / 秒
                  这是头显 APK 自己写的时间戳（quest_events.py 里的 sample['sent_ns']），
                  即上游采样节奏，与 MORLA 无关
  <port> recv     port_metadata.<port>.received_ns / 秒（MORLA 实际收到的节奏）
  <port> cmd      kind=control 中该 source_port 的记录 / 秒（真正下发的指令）
  interleave      位姿快照的左右手排列：连续同侧的最大长度、快照间隔
"""
import collections
import glob
import json
import sys

import numpy as np


def rate(timestamps):
    t = sorted(set(timestamps))
    return (len(t) - 1) / ((t[-1] - t[0]) / 1e9) if len(t) > 1 else float('nan')


def scan(run):
    frames, sent, received = [], collections.defaultdict(list), collections.defaultdict(list)
    commands = collections.defaultdict(list)
    order, order_ns = [], []
    previous = None
    for path in sorted(glob.glob(run.rstrip('/') + '/trace-*.jsonl')):
        for line in open(path):
            if '"kind":"input"' in line:
                row = json.loads(line)
                data = row.get('data') or {}
                metadata = data.get('port_metadata') or {}
                frames.append(row['monotonic_ns'])
                for port, meta in metadata.items():
                    if 'source_time_ns' in meta:
                        sent[port].append(meta['source_time_ns'])
                    if 'received_ns' in meta:
                        received[port].append(meta['received_ns'])
                current = (metadata.get('left_pose', {}).get('received_ns'),
                           metadata.get('right_pose', {}).get('received_ns'))
                if previous is None:
                    previous = current
                elif current != previous:
                    order.append('L' if current[0] != previous[0] else 'R')
                    order_ns.append(row['monotonic_ns'])
                    previous = current
            elif '"kind":"control"' in line:
                row = json.loads(line)
                commands[row['data'].get('source_port')].append(row['monotonic_ns'])
    return frames, sent, received, commands, order, order_ns


def report(run):
    frames, sent, received, commands, order, order_ns = scan(run)
    print(run.rstrip('/').split('/')[-1])
    print('  frames                  %7.1f Hz' % rate(frames))
    for port in ('left_pose', 'right_pose'):
        if sent.get(port):
            print('  %-11s APK           %7.1f Hz' % (port, rate(sent[port])))
            print('  %-11s received      %7.1f Hz' % ('', rate(received[port])))
            if len(commands.get(port) or []) > 20:
                print('  %-11s command       %7.1f Hz' % ('', rate(commands[port])))
    if order:
        runs = []
        for side in order:
            if runs and runs[-1][0] == side:
                runs[-1][1] += 1
            else:
                runs.append([side, 1])
        lengths = collections.Counter((side, n) for side, n in runs)
        t = np.asarray(order_ns, float) / 1e9
        dt = np.diff(t) * 1000
        print('  pose snapshots          %7.1f Hz   interval p50 %.1f p95 %.1f ms'
              % ((len(t) - 1) / (t[-1] - t[0]), np.median(dt), np.percentile(dt, 95)))
        print('  interleave              %s' % (' '.join('%sx%d=%d' % (s, n, c)
                                                        for (s, n), c in sorted(lengths.items()))))
    print()


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        report(argument)
