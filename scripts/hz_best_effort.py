#!/usr/bin/env python3
"""hz_best_effort.py — 对 BEST_EFFORT 话题测实际到达率/间隔（py3.10 + rclpy）。

ros2 topic hz 在 Humble 没有 QoS 选项，默认 RELIABLE 订阅，匹配不上 BEST_EFFORT
发布者（如 /left_arm/joint_commands），永远 0 条。本脚本用 BEST_EFFORT depth=1
订阅，滚动打印：到达率、相邻消息间隔的 min/mean/max/std；可同时按固定周期
写 JSONL，供相机超时测试与 policy 控制日志对齐。

用法（py3.10 + ROS 环境）：
  PYTHONPATH=astral_ws/src/astral_policy_inference:astral_ws/src/astral_data_collect \
    /usr/bin/python3 astral_ws/scripts/hz_best_effort.py \
      /left_arm/joint_commands --msg js
  /usr/bin/python3 astral_ws/scripts/hz_best_effort.py \
      /quest3_video_streamer/collect/left_wrist --msg cimg --window 150 \
      --log-file /tmp/left_wrist_hz.jsonl
  # --msg: js(JointState) | cimg(CompressedImage) | float(Float64) |
  #        str(String)，按话题选
  # --window N：滚动窗口消息数（默认 50）；Ctrl-C 退出
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument(
        "--msg", choices=["js", "cimg", "float", "str"], default="js"
    )
    ap.add_argument("--window", type=int, default=50)
    ap.add_argument(
        "--log-file", default="",
        help="可选 JSONL 输出；每 --report-s 秒记录一次，检测到长 gap 时立即记录",
    )
    ap.add_argument("--report-s", type=float, default=1.0)
    ap.add_argument("--gap-ms", type=float, default=500.0)
    args = ap.parse_args()

    from sensor_msgs.msg import CompressedImage, JointState
    from std_msgs.msg import Float64, String

    msg_type = {
        "js": JointState,
        "cimg": CompressedImage,
        "float": Float64,
        "str": String,
    }[args.msg]
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    rclpy.init()
    node = Node("hz_best_effort")
    stamps: list[float] = []
    intervals: list[float] = []
    count = 0
    last_report = 0.0
    log_fh = None
    if args.log_file:
        parent = os.path.dirname(os.path.abspath(args.log_file))
        os.makedirs(parent, exist_ok=True)
        log_fh = open(args.log_file, "a", buffering=1)

    def cb(_msg) -> None:
        nonlocal count, last_report
        t = time.monotonic()
        wall_t = time.time()
        gap = t - stamps[-1] if stamps else None
        if stamps:
            intervals.append(gap)
            if len(intervals) > args.window:
                intervals.pop(0)
        stamps.append(t)
        count += 1
        if len(stamps) > args.window:
            stamps.pop(0)
        if len(stamps) >= 2:
            span = stamps[-1] - stamps[0]
            rate = (len(stamps) - 1) / span if span > 0 else 0.0
            iv = intervals[-min(len(intervals), 50):]
            s = ""
            if iv:
                s = (f" interval[min={min(iv)*1e3:.1f} mean={statistics.mean(iv)*1e3:.1f}"
                     f" max={max(iv)*1e3:.1f} std={statistics.pstdev(iv)*1e3:.1f} ms]")
            print(f"\rrate={rate:.1f} Hz{s}   ", end="", flush=True)
            gap_event = gap is not None and gap * 1000.0 >= args.gap_ms
            if log_fh is not None and (
                wall_t - last_report >= max(0.05, args.report_s) or gap_event
            ):
                rec = {
                    "t": round(wall_t, 4),
                    "topic": args.topic,
                    "msg": args.msg,
                    "count": count,
                    "rate_hz": round(rate, 3),
                    "last_gap_ms": round(gap * 1000.0, 3) if gap is not None else None,
                    "gap_event": gap_event,
                    "interval_ms": {
                        "min": round(min(iv) * 1000.0, 3),
                        "mean": round(statistics.mean(iv) * 1000.0, 3),
                        "max": round(max(iv) * 1000.0, 3),
                        "std": round(statistics.pstdev(iv) * 1000.0, 3),
                    },
                }
                log_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
                last_report = wall_t

    node.create_subscription(msg_type, args.topic, cb, qos)
    print(f"subscribing {args.topic} (BEST_EFFORT depth=1), --window {args.window} ...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log_fh is not None:
            log_fh.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
