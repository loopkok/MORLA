"""一帧输入在摄取线程上到底花在哪：控制器、设备提交、整帧间隔。

用法:
    /usr/bin/python3 scripts/frame_cost.py ~/.local/share/morla/logs/<run_dir> [...]

从 trace 里取三段（都在同一条摄取线程上，以 trace 的 monotonic_ns 为准）:
  step     kind=control 的 elapsed_ns —— engine 调 controller.step 的同步耗时
  submit   kind=command_request → 紧随其后的 command_receipt —— 提交到设备/仿真
  frame    相邻 kind=input 的间隔；按该秒有没有指令分成"空转"和"遥操"两段

读数要点：MMK2 的 IK 不在这一跳上（AsyncCartesianController 把它放进
worker 线程），所以 step 只有毫秒级；真正贵的是 step 之后的设备提交。
"""
import collections
import glob
import json
import sys

import numpy as np


def percentiles(values, label):
    a = np.asarray(values, float)
    return '%-24s n=%5d  p50 %7.2f  p95 %7.2f  max %7.1f' % (
        label, len(a), np.percentile(a, 50), np.percentile(a, 95), a.max())


def report(run):
    step = collections.defaultdict(list)
    submit = []
    frames = []
    frame_seconds = []
    commands_per_second = collections.Counter()
    request_ns = None
    for path in sorted(glob.glob(run.rstrip('/') + '/trace-*.jsonl')):
        for line in open(path):
            if '"kind":"input"' in line:
                ns = json.loads(line)['monotonic_ns']
                frames.append(ns)
                frame_seconds.append(ns // 1_000_000_000)
            elif '"kind":"control"' in line:
                row = json.loads(line)
                value = row['data'].get('elapsed_ns')
                if value is not None:
                    step[row['data'].get('source_port')].append(value / 1e6)
                commands_per_second[row['monotonic_ns'] // 1_000_000_000] += 1
            elif '"kind":"command_request"' in line:
                request_ns = json.loads(line)['monotonic_ns']
            elif '"kind":"command_receipt"' in line and request_ns is not None:
                submit.append((json.loads(line)['monotonic_ns'] - request_ns) / 1e6)
                request_ns = None

    print(run.rstrip('/').split('/')[-1])
    for port, values in sorted(step.items()):
        if port and port.endswith('_pose'):
            print('  ' + percentiles(values, 'controller.step %s' % port) + ' ms')
    if submit:
        print('  ' + percentiles(submit, 'adapter submit') + ' ms')
    if len(frames) > 1:
        gaps = np.diff(np.asarray(frames, float))
        idle = [g / 1e6 for g, second in zip(gaps, frame_seconds[1:])
                if not commands_per_second.get(second)]
        busy = [g / 1e6 for g, second in zip(gaps, frame_seconds[1:])
                if commands_per_second.get(second)]
        if idle:
            print('  ' + percentiles(idle, '帧间隔（该秒无指令）') + ' ms')
        if busy:
            print('  ' + percentiles(busy, '帧间隔（该秒有指令）') + ' ms')
        print('  整轮输入 %d 帧 %.1f Hz' % (len(frames), (len(frames) - 1) / ((frames[-1] - frames[0]) / 1e9)))
    print()


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        report(argument)
