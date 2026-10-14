#!/usr/bin/env python3
"""远程推理链路完整指标基准（py3.10，连 serve.py）。

覆盖全部链路指标：
  * 握手：建立 websocket 会话耗时；
  * RTT 分布（p50/p95/p99/max/std）——整体 + 真推理/缓存弹出分界；
  * 服务端分项耗时 server_timing（prep/pre/infer/post，随响应返回）；
  * 网络 vs 模型分离：RTT − server_total；
  * 本地序列化耗时（vendored msgpack pack/unpack 载荷）；
  * 线缆探测：小消息（畸形请求被快速拒绝）的往返，作原始消息 RTT 下限；
  * GPU 利用率/显存（推理期间采样）；
  * 吞吐（requests/s 与 MB/s）。

用法（先起 serve.py）：
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 astral_ws/scripts/run_act_benchmark.py --host 127.0.0.1 --port 8001

退出码 0=链路正常；非 0=异常。
"""
from __future__ import annotations

import argparse
import statistics
import subprocess
import sys
import threading
import time

import numpy as np


def pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    return float(np.percentile(sorted(xs), p))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--requests", type=int, default=250)
    ap.add_argument("--camera-size", type=int, default=480)
    ap.add_argument("--n-action-steps", type=int, default=50,
                    help="ACT chunk 长度：每次 infer 产出的动作行数")
    ap.add_argument("--jpeg", action="store_true",
                    help="上行 camera 槽位编码成 JPEG（对比 RGB 的带宽/延迟收益）")
    args = ap.parse_args()

    sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
    sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")
    from astral_policy_inference import protocol as _proto
    from astral_policy_inference.backend import ObsBatch, RemoteBackend

    rng = np.random.default_rng(1)
    cam = args.camera_size
    obs = ObsBatch(
        state=rng.normal(0, 0.5, 8),
        images={"base": rng.integers(0, 256, (cam, cam, 3), np.uint8),
                "left_wrist": rng.integers(0, 256, (cam, cam, 3), np.uint8)},
        prompt="benchmark",
    )
    if args.jpeg:
        from astral_policy_inference.image_codec import encode_jpeg

        payload = {
            "observation/state": obs.state.astype(np.float32),
            "observation/camera/base_0_rgb": encode_jpeg(obs.images["base"]),
            "observation/camera/left_wrist_0_rgb": encode_jpeg(obs.images["left_wrist"]),
            "prompt": obs.prompt,
            "image_format": "jpeg",
        }
    else:
        payload = {
            "observation/state": obs.state.astype(np.float32),
            "observation/camera/base_0_rgb": obs.images["base"],
            "observation/camera/left_wrist_0_rgb": obs.images["left_wrist"],
            "prompt": obs.prompt,
        }
    nbytes = sum(
        len(v) if isinstance(v, (bytes, bytearray))
        else (v.nbytes if isinstance(v, np.ndarray) else 0)
        for v in payload.values()
    )
    print(f"payload: {nbytes/1e6:.2f} MB/request ({'JPEG' if args.jpeg else 'RGB'})")

    # ---- 握手耗时 ----
    bk = RemoteBackend(
        host=args.host, port=args.port, action_dim=8,
        slot_keys={"base_0_rgb": "base", "left_wrist_0_rgb": "left_wrist"},
        jpeg_transport=args.jpeg,
    )
    t_h = time.perf_counter()
    bk.open()
    handshake_ms = (time.perf_counter() - t_h) * 1000.0
    print(f"connected (handshake {handshake_ms:.1f}ms)")

    # ---- 本地序列化探测（包内 vendored msgpack pack/unpack 载荷）----
    _pk = _proto.Packer()
    t0 = time.perf_counter()
    for _ in range(10):
        blob = _pk.pack(payload)
    pack_ms = (time.perf_counter() - t0) / 10 * 1000.0
    t0 = time.perf_counter()
    for _ in range(10):
        _proto.unpackb(blob)
    unpack_ms = (time.perf_counter() - t0) / 10 * 1000.0
    print(f"local serialization: pack {pack_ms:.2f}ms, unpack {unpack_ms:.2f}ms "
          f"(blob {len(blob)/1e6:.2f}MB)")

    # ---- 突发请求：RTT + server_timing 采集 ----
    gpu_utils: list[float] = []
    stop = threading.Event()

    def sample_gpu():
        while not stop.is_set():
            try:
                out = subprocess.run(
                    ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used",
                     "--format=csv,noheader,nounits"],
                    capture_output=True, text=True, timeout=3).stdout.strip()
                u, mem = out.split(",")
                gpu_utils.append((float(u), float(mem)))
            except Exception:
                pass
            time.sleep(0.4)

    t_gpu = threading.Thread(target=sample_gpu, daemon=True)
    t_gpu.start()

    rtts: list[float] = []
    net_ms: list[float] = []           # RTT − server_total
    timing_agg: dict[str, list[float]] = {"prep_ms": [], "pre_ms": [],
                                          "infer_ms": [], "post_ms": [], "total_ms": []}
    fails = 0
    t0 = time.perf_counter()
    for i in range(args.requests):
        s = time.perf_counter()
        try:
            out = np.asarray(bk.infer(obs))
            if out.ndim != 2 or out.shape[1] != 8 or not np.isfinite(out).all():
                fails += 1
            elif out.shape[0] < 1:
                fails += 1
        except Exception:  # noqa: BLE001
            fails += 1
            rtts.append(-1.0)
            continue
        rtt = (time.perf_counter() - s) * 1000.0
        rtts.append(rtt)
        st = bk.last_server_timing or {}
        for k, v in st.items():
            if k in timing_agg:
                timing_agg[k].append(float(v))
        tot = st.get("total_ms")
        if tot:
            net_ms.append(rtt - tot)
    total = time.perf_counter() - t0
    stop.set()
    t_gpu.join(timeout=1)

    ok = [r for r in rtts if r >= 0]
    print(f"\n=== 结果（{args.requests} 请求, 突发 {total:.2f}s）===")
    print(f"失败/异常: {fails}")
    print(f"每次 infer 产出一个 {args.n_action_steps} 行 chunk（方案 A，无缓存弹出）")
    print(f"吞吐: {len(ok)/total:.1f} chunks/s = {len(ok)*args.n_action_steps/total:.0f} "
          f"actions/s ({len(ok)*nbytes/1e6/total:.0f} MB/s 上行)")
    print(f"RTT 全分布: p50 {pct(ok,50):.1f}  p95 {pct(ok,95):.1f}  "
          f"p99 {pct(ok,99):.1f}  max {max(ok):.1f}  std {statistics.pstdev(ok):.1f} ms")
    print(f"RTT 抖动: std {statistics.pstdev(ok):.2f} ms, "
          f"max-min 跨度 {max(ok)-min(ok):.1f} ms")
    print(f"30Hz 控制率需要的 chunk/s: 0.60（每 chunk 覆盖 50 行/30Hz = 1.67s）→ "
          f"余量 {len(ok)/total/0.6:.0f}×")

    # ---- 服务端分项（server_timing）----
    print("\n服务端分项 (server_timing, mean ms):")
    for k in ("prep_ms", "pre_ms", "infer_ms", "post_ms", "total_ms"):
        xs = timing_agg[k]
        if xs:
            print(f"  {k:>9}: {statistics.mean(xs):6.2f}  (median {pct(xs,50):.2f})")

    # ---- 网络 vs 模型分离 ----
    if net_ms:
        print(f"网络+序列化 (RTT−server_total): p50 {pct(net_ms,50):.2f}  "
              f"p95 {pct(net_ms,95):.2f}  max {max(net_ms):.2f} ms")

    # ---- 线缆探测：小消息往返（畸形请求被快速拒绝）----
    from astral_policy_inference.client import WebsocketClient
    try:
        raw = WebsocketClient(args.host, args.port)
        t0 = time.perf_counter()
        try:
            raw.infer({"bad": 1})  # 缺 observation/state → server 快速回错误串
        except RuntimeError:
            pass
        tiny_ms = (time.perf_counter() - t0) * 1000.0
        print(f"线缆探测（小消息 RTT 下限）: {tiny_ms:.2f} ms")
        raw.close()
    except Exception as exc:  # noqa: BLE001
        print(f"线缆探测失败: {exc}")

    if gpu_utils:
        us = [u for u, _ in gpu_utils]
        mems = [m for _, m in gpu_utils]
        print(f"GPU 推理期间: util mean {statistics.mean(us):.0f}% max {max(us):.0f}%, "
              f"mem mean {statistics.mean(mems):.0f}MiB")

    bk.close()
    ok_final = fails == 0 and len(ok) > 0
    print("\nRESULT", "ALL PASS" if ok_final else "FAIL", "(0 失败 + 全为真 chunk 推理)")
    return 0 if ok_final else 1


if __name__ == "__main__":
    raise SystemExit(main())
