#!/usr/bin/env python3
"""quest3_pose_listener.py — 记录 Quest 3 位姿与事件，按模式/运动分段对比手柄与纯手。

只订阅、不发布，不改变接收器与会话状态；会话是否 ACTIVE 都不影响。

用法（ROS_DOMAIN_ID 必须与接收器一致，接收器现在是 91 + LOCALHOST_ONLY）：
  source /opt/ros/humble/setup.bash && source install/setup.bash
  export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
  /usr/bin/python3 scripts/quest3_pose_listener.py --side right \
      --out ~/quest3_capture/compare_$(date +%Y%m%d_%H%M%S)

Ctrl-C 结束。运行中每秒打印一行状态，切换手柄/纯手时打印 [MODE] 切换；
退出时写 summary.json：
  - 每个流（wrist/controller/head/hips）的条数、实际到达 Hz、frame_id；
  - 模式窗口（controller / hand / none），由 teleop_events 的 source 判定；
  - 运动分段：静止 → 运动 → 静止的每一段给出位移、旋转轴角（Unity 世界系），
    这是对比“手柄 vs 纯手 方向对不对”的直接证据；
  - controller 与 wrist 的镜像一致性（接收器 controller_as_wrist=true 时两者应逐条相同）；
  - epoch 事件（motion_stream_timeout / reconnect / frame 切换），它们会让下游撤权重锚。
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

STREAM_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=50)
EVENT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=50)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

MODE_BY_TOKEN = {'ctrl': 'controller', 'hand': 'hand', 'none': 'none'}


def quat_matrix(q):
    """xyzw -> 3x3 旋转矩阵；零四元数返回 None。"""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return None
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([[1 - yy - zz, xy - wz, xz + wy],
                     [xy + wz, 1 - xx - zz, yz - wx],
                     [xz - wy, yz + wx, 1 - xx - yy]])


def rot_axis_angle(q0, q1):
    """世界系旋转差 R1·R0ᵀ 的轴角（重标定后 ΔR = R_t·R_anchorᵀ，与常量偏移无关）。"""
    r0, r1 = quat_matrix(q0), quat_matrix(q1)
    if r0 is None or r1 is None:
        return None, 0.0
    d = r1 @ r0.T
    cos = max(-1.0, min(1.0, (np.trace(d) - 1.0) / 2.0))
    angle = float(np.arccos(cos))
    if angle < 1e-9:
        return np.zeros(3), 0.0
    axis = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]])
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.zeros(3), angle
    return axis / norm, angle


def q_to_list(msg):
    o = msg.pose.orientation
    return [float(o.x), float(o.y), float(o.z), float(o.w)]


def p_to_list(msg):
    p = msg.pose.position
    return [float(p.x), float(p.y), float(p.z)]


class Quest3Listener(Node):
    def __init__(self, args):
        super().__init__('quest3_pose_listener')
        self.side = args.side
        self.out = Path(os.path.expanduser(args.out))
        self.out.mkdir(parents=True, exist_ok=True)
        self.landmarks = args.landmarks
        self.speed_threshold = args.speed_threshold
        self.angle_threshold = args.angle_threshold
        self.rest_seconds = args.rest_seconds
        self.min_segment = args.min_segment

        self.samples = (self.out / 'samples.jsonl').open('w', buffering=1)
        self.events = (self.out / 'events.jsonl').open('w', buffering=1)

        self.started_ns = time.time_ns()
        self.started_wall = time.strftime('%Y-%m-%d %H:%M:%S')
        self.mode = 'unknown'
        self.mode_since = self.started_ns
        self.mode_windows = []
        self.streams = {}      # name -> ring of recent recv_ns, latest row
        self.rows = {'wrist': [], 'controller': []}  # (recv, p, q, mode)
        self.counts = {}
        self.last = {}
        self.gap_max = {'wrist': 0.0, 'controller': 0.0}
        self.gap_count = {'wrist': 0, 'controller': 0}
        self.last_recv = {}
        self._tick_counts = {}
        self._tick_time = time.monotonic()
        self.epoch_events = []
        self.mode_events = []
        self.joy_last = None

        side = self.side
        self._subscribe_pose(f'quest3/{side}_wrist_pose', 'wrist')
        self._subscribe_pose(f'quest3/{side}_controller_pose', 'controller')
        self._subscribe_pose('quest3/head_pose', 'head')
        self._subscribe_pose('quest3/hips_pose', 'hips')
        self.create_subscription(String, f'quest3/{side}_teleop_events', self.on_teleop, EVENT_QOS)
        self.create_subscription(String, 'quest3/input_mix', self.on_mix, LATCHED_QOS)
        self.create_subscription(String, 'quest3/input_epoch', self.on_epoch, LATCHED_QOS)
        self.create_subscription(Joy, f'quest3/{side}_controller_joy', self.on_joy, STREAM_QOS)
        if self.landmarks:
            self.create_subscription(PoseArray, f'hand_landmarks/{side}', self.on_landmarks, STREAM_QOS)

        self.create_timer(args.status_period, self.status)
        self.get_logger().info(
            f'listening right={side} mode={self.mode} out={self.out} '
            f'(sidestep: teleop events + poses; ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID")})')

    # ---- subscriptions -------------------------------------------------
    def _subscribe_pose(self, topic, name):
        self.create_subscription(
            PoseStamped, topic,
            lambda msg, n=name: self.on_pose(n, msg), STREAM_QOS)

    def on_pose(self, stream, msg):
        recv = time.time_ns()
        p, q = p_to_list(msg), q_to_list(msg)
        self.increment(stream)
        # wrist 流在手柄模式下是 controller 的镜像（controller_as_wrist=true），
        # 这里按“发布时刻的模式”标注，分段分析才能把两轮分开。
        mode = 'controller' if stream == 'controller' else self.mode
        self.record({'recv_ns': recv, 'topic': stream, 'mode': mode,
                     'stamp_ns': stamp_ns(msg), 'frame': msg.header.frame_id,
                     'p': p, 'q': q})
        if stream in self.rows:
            self.rows[stream].append({'recv': recv, 'stamp': stamp_ns(msg),
                                      'p': tuple(p), 'q': tuple(q), 'mode': mode})
            prev = self.last_recv.get(stream)
            if prev:
                gap = (recv - prev) / 1e9
                if gap > self.gap_max[stream]:
                    self.gap_max[stream] = gap
                if gap > 0.1:
                    self.gap_count[stream] += 1
            self.last_recv[stream] = recv
        self.last[stream] = {'recv_ns': recv, 'frame': msg.header.frame_id, 'p': p, 'q': q}

    def on_teleop(self, msg):
        try:
            row = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        recv = time.time_ns()
        kind = row.get('kind')
        src = row.get('source')
        values = row.get('values')
        record = {'recv_ns': recv, 'event': 'teleop', 'kind': kind, 'source': src,
                  'epoch': row.get('epoch'), 'sequence': row.get('sequence'),
                  'frame_id': row.get('frame_id'), 'basis': row.get('basis')}
        if kind == 'landmarks':
            record['n_landmarks'] = len(values or [])
        else:
            record['values'] = values
        self.record(record, events=True)
        if kind == 'pose' and src in ('controller', 'hand') and src != self.mode:
            self.set_mode(src, 'teleop_source', recv)

    def on_mix(self, msg):
        recv = time.time_ns()
        self.record({'recv_ns': recv, 'event': 'mix', 'data': msg.data}, events=True)
        for part in msg.data.split():
            if part.startswith(self.side + '='):
                mode = MODE_BY_TOKEN.get(part.split('=', 1)[1], 'unknown')
                if mode not in ('unknown',) and mode != self.mode:
                    self.set_mode(mode, 'input_mix', recv)

    def on_epoch(self, msg):
        self.epoch_events.append({'recv_ns': time.time_ns(), 'data': msg.data})
        self.record({'recv_ns': time.time_ns(), 'event': 'epoch', 'data': msg.data}, events=True)

    def on_joy(self, msg):
        self.joy_last = {'recv_ns': time.time_ns(),
                         'axes': [round(float(a), 4) for a in msg.axes],
                         'buttons': [int(b) for b in msg.buttons]}
        self.increment('joy')

    def on_landmarks(self, msg):
        recv = time.time_ns()
        self.increment('landmarks')
        self.record({'recv_ns': recv, 'topic': 'landmarks', 'mode': self.mode,
                     'frame': msg.header.frame_id,
                     'p': [p_to_list(ps) for ps in msg.poses],
                     'q': [q_to_list(ps) for ps in msg.poses]})

    # ---- bookkeeping ---------------------------------------------------
    def increment(self, name):
        self.counts[name] = self.counts.get(name, 0) + 1
        self._tick_counts[name] = self._tick_counts.get(name, 0) + 1

    def record(self, row, events=False):
        (self.events if events else self.samples).write(json.dumps(row) + '\n')

    def set_mode(self, mode, reason, recv):
        end = recv
        if self.mode != 'unknown':
            self.mode_windows.append({'mode': self.mode, 'start_ns': self.mode_since,
                                      'end_ns': end, 'reason': reason})
        self.mode_events.append({'mode': mode, 'reason': reason, 'recv_ns': recv})
        self.record({'recv_ns': recv, 'event': 'mode', 'mode': mode, 'reason': reason},
                    events=True)
        self.get_logger().info(f'[MODE] {self.mode} -> {mode} ({reason})')
        self.mode = mode
        self.mode_since = recv

    def status(self):
        now = time.monotonic()
        dt = max(1e-6, now - self._tick_time)
        counts = self._tick_counts
        self._tick_counts = {}
        self._tick_time = now
        parts = [f'[{ (time.time_ns() - self.started_ns) / 1e9:6.1f}s] mode={self.mode}']
        for name, n in sorted(counts.items()):
            parts.append(f'{name}={n / dt:5.1f}Hz')
        parts.append(f'gaps(wrist>{0.1}s)={self.gap_count["wrist"]}')
        if self.gap_max['wrist'] > 0.1:
            parts.append(f'wrist_gap_max={self.gap_max["wrist"] * 1e3:.0f}ms')
        if self.epoch_events:
            parts.append(f'epochs={len(self.epoch_events)}')
        self.get_logger().info(' '.join(parts))

    # ---- analysis ------------------------------------------------------
    def motion_segments(self, stream):
        rows = self.rows[stream]
        segs = []
        if len(rows) < 3:
            return segs
        moving = [False] * len(rows)
        speeds = [0.0] * len(rows)
        rates = [0.0] * len(rows)
        prev = rows[0]
        for i in range(1, len(rows)):
            cur = rows[i]
            dt = (cur['recv'] - prev['recv']) / 1e9
            if dt > 0:
                speeds[i] = float(np.linalg.norm(np.subtract(cur['p'], prev['p']))) / dt
                _, ang = rot_axis_angle(prev['q'], cur['q'])
                rates[i] = ang / dt
                moving[i] = speeds[i] > self.speed_threshold or rates[i] > self.angle_threshold
            prev = cur

        start = None
        last_moving = None
        run = 0
        for i, flag in enumerate(moving):
            # A controller<->hand switch is a new segment even without a pause:
            # the two rounds must never be merged into one statistic.
            if i > 0 and rows[i]['mode'] != rows[i - 1]['mode']:
                if start is not None:
                    seg = self._segment(rows, start, last_moving, speeds, rates)
                    if seg and seg['duration_s'] >= self.min_segment:
                        segs.append(seg)
                start = None
                run = 0
                continue
            if flag:
                run += 1
                if start is None and run >= 2:
                    start = i - 1
                last_moving = i
            else:
                if start is not None and (rows[i]['recv'] - rows[last_moving]['recv']) / 1e9 >= self.rest_seconds:
                    seg = self._segment(rows, start, last_moving, speeds, rates)
                    if seg and seg['duration_s'] >= self.min_segment:
                        segs.append(seg)
                    start = None
                    run = 0
        if start is not None:
            seg = self._segment(rows, start, last_moving, speeds, rates)
            if seg and seg['duration_s'] >= self.min_segment:
                segs.append(seg)
        return segs

    def _segment(self, rows, a, b, speeds, rates):
        if b is None or b <= a:
            return None
        p0, q0 = rows[a]['p'], rows[a]['q']
        p1, q1 = rows[b]['p'], rows[b]['q']
        axis, angle = rot_axis_angle(q0, q1)
        path = 0.0
        for i in range(a + 1, b + 1):
            path += float(np.linalg.norm(np.subtract(rows[i]['p'], rows[i - 1]['p'])))
        d = np.subtract(p1, p0)
        window = slice(a + 1, b + 1)
        return {
            'mode': rows[a]['mode'],
            'start_ns': rows[a]['recv'], 'end_ns': rows[b]['recv'],
            'duration_s': round((rows[b]['recv'] - rows[a]['recv']) / 1e9, 3),
            'samples': b - a + 1,
            'displacement_m': [round(float(v), 4) for v in d],
            'distance_m': round(float(np.linalg.norm(d)), 4),
            'path_m': round(path, 4),
            'peak_speed_m_s': round(max(speeds[window] or [0.0]), 3),
            'peak_rate_deg_s': round(float(np.degrees(max(rates[window] or [0.0]))), 1),
            'rotation_axis': None if axis is None else [round(float(v), 3) for v in axis],
            'rotation_deg': round(float(np.degrees(angle)), 2),
        }

    def mirror_check(self):
        """controller_as_wrist=true 时 wrist 在手柄模式下就是 controller 的镜像。

        接收器把同一个 PoseStamped 发到两个话题，所以按 header.stamp 对齐；两条流
        的记录时刻不同，不能按接收时刻配对。
        """
        ctrl = {r['stamp']: r for r in self.rows['controller']}
        compared = 0
        max_p = 0.0
        max_q = 0.0
        for r in self.rows['wrist']:
            c = ctrl.get(r['stamp'])
            if c is None:
                continue
            compared += 1
            max_p = max(max_p, float(np.linalg.norm(np.subtract(r['p'], c['p']))))
            max_q = max(max_q, max(abs(a - b) for a, b in zip(r['q'], c['q'])))
        return {'compared': compared,
                'max_position_diff_m': round(max_p, 6),
                'max_quaternion_diff': round(max_q, 6)}

    def summarize(self):
        now = time.time_ns()
        windows = list(self.mode_windows)
        if self.mode != 'unknown':
            windows.append({'mode': self.mode, 'start_ns': self.mode_since,
                            'end_ns': now, 'reason': 'shutdown'})
        duration = (now - self.started_ns) / 1e9
        summary = {
            'side': self.side,
            'out': str(self.out),
            'started': self.started_wall,
            'ended': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration_s': round(duration, 2),
            'mode': self.mode,
            'streams': {},
            'mode_windows': [
                {'mode': w['mode'],
                 'start_s': round((w['start_ns'] - self.started_ns) / 1e9, 2),
                 'end_s': round((w['end_ns'] - self.started_ns) / 1e9, 2),
                 'reason': w['reason']}
                for w in windows],
            'mode_events': len(self.mode_events),
            'epoch_events': [
                {'at_s': round((e['recv_ns'] - self.started_ns) / 1e9, 2), 'data': e['data']}
                for e in self.epoch_events],
            'gaps': {k: {'max_s': round(v, 4), 'count_over_100ms': self.gap_count[k]}
                     for k, v in self.gap_max.items()},
            'motion_segments': {k: self.motion_segments(k) for k in ('wrist', 'controller')},
            'wrist_controller_mirror': self.mirror_check(),
        }
        for name, count in sorted(self.counts.items()):
            last = self.last.get(name, {})
            summary['streams'][name] = {
                'count': count,
                'hz': round(count / duration, 1) if duration > 0 else 0.0,
                'frame': last.get('frame'),
                'last_p': last.get('p'),
                'last_q': last.get('q'),
            }
        return summary

    def close(self):
        for f in (self.samples, self.events):
            try:
                f.close()
            except OSError:
                pass


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side', choices=('right', 'left'), default='right')
    ap.add_argument('--out', default=f'~/quest3_capture/{time.strftime("%Y%m%d_%H%M%S")}')
    ap.add_argument('--landmarks', action='store_true', help='同时记录 hand_landmarks（文件会大很多）')
    ap.add_argument('--status-period', type=float, default=1.0)
    ap.add_argument('--duration', type=float, default=0.0, help='秒；0 表示 Ctrl-C 结束')
    ap.add_argument('--speed-threshold', type=float, default=0.02, help='m/s，运动分段阈值')
    ap.add_argument('--angle-threshold', type=float, default=0.15, help='rad/s，运动分段阈值')
    ap.add_argument('--rest-seconds', type=float, default=0.35, help='静止多久算一段结束')
    ap.add_argument('--min-segment', type=float, default=0.3, help='短于此的分段丢弃（秒）')
    args = ap.parse_args(argv)

    rclpy.init()
    node = Quest3Listener(args)
    stopping = {'flag': False}

    def stop(*_):
        stopping['flag'] = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok() and not stopping['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            if deadline and time.monotonic() >= deadline:
                break
    finally:
        summary = node.summarize()
        (node.out / 'summary.json').write_text(json.dumps(summary, indent=2))
        node.get_logger().info(
            f'capture done: {summary["duration_s"]}s, '
            f'{len(summary["mode_windows"])} mode windows, '
            f'{sum(len(v) for v in summary["motion_segments"].values())} motion segments '
            f'-> {node.out / "summary.json"}')
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
