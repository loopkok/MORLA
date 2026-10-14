#!/usr/bin/env python3
"""完整节点端到端（真实 ACT via serve.py）——合成观测驱动 policy_node。

前提（外部已启动）：
  * serve.py（py3.12 lerobot 环境）加载真实 checkpoint；
  * policy_node（py3.10 + ROS）backend_type=remote，host/port 指向 serve.py，
    camera_image_size=480（与模型 preprocessor 一致）。

本脚本：
  1.（可选 --check-server）用真实的 RemoteBackend 直连 serve.py 做一次推理往返，
     快速定位「节点连不上」是 server 问题还是节点配置问题；
  2. 发布合成观测（关节 + 夹爪比值 + base/left_wrist 的 480×480 JPEG），
     发 policy 命令，验证真实 ACT 驱动指令流 → pause 夹持 → resume 重规划 → stop。

用法（py3.10 + ROS，用 ros2 conda env python 或 /usr/bin/python3，需 cv2/PIL 之一）：
  source /opt/ros/humble/setup.bash
  /home/robot/miniconda3/envs/ros2/bin/python astral_ws/scripts/run_act_e2e.py \
      --check-server --host 127.0.0.1 --port 8001

退出码 0=通过；非 0=失败。
"""
from __future__ import annotations

import argparse
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64, String

SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

fails: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    print(f"[{'PASS' if cond else 'FAIL'}] {name} {detail}")
    if not cond:
        fails.append(name)


def make_jpeg(size: int = 480) -> bytes:
    y, x = np.mgrid[0:size, 0:size]
    img = np.stack([(x / size * 255).astype(np.uint8),
                    (y / size * 255).astype(np.uint8),
                    np.full((size, size), 128, np.uint8)], axis=-1)
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes()
    except ImportError:
        from PIL import Image
        import io
        b = io.BytesIO()
        Image.fromarray(img).save(b, format="JPEG", quality=90)
        return b.getvalue()


def check_server(host: str, port: int, jpeg: bool = False) -> None:
    """用真实 RemoteBackend 直连 serve.py 做一次推理往返。"""
    import sys
    sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_policy_inference")
    sys.path.insert(0, "/home/robot/loopkok/sdk/astral_ws/src/astral_data_collect")
    from astral_policy_inference.backend import ObsBatch, RemoteBackend

    rng = np.random.default_rng(7)
    bk = RemoteBackend(
        host=host, port=port, action_dim=8,
        slot_keys={"base_0_rgb": "base", "left_wrist_0_rgb": "left_wrist"},
        jpeg_transport=jpeg,
    )
    bk.open()
    obs = ObsBatch(
        state=rng.normal(0, 0.5, 8),
        images={"base": rng.integers(0, 256, (480, 480, 3), np.uint8),
                "left_wrist": rng.integers(0, 256, (480, 480, 3), np.uint8)},
        prompt="round trip",
    )
    out = np.asarray(bk.infer(obs))
    check("preflight: RemoteBackend -> serve.py round trip",
          out.ndim == 2 and out.shape[1] == 8 and bool(np.isfinite(out).all()),
          f"shape={out.shape} infer_ms={bk.last_infer_s*1000:.0f}")
    bk.close()


class Driver(Node):
    def __init__(self):
        super().__init__("act_e2e_driver")
        self.cmd_pub = self.create_publisher(String, "/policy_inference/cmd", 10)
        self.js_pub = self.create_publisher(JointState, "/left_arm/joint_states", 1)
        self.grip_pub = self.create_publisher(Float64, "/left_gripper/command", 1)
        self.img_pubs = {
            label: self.create_publisher(
                CompressedImage, f"/quest3_video_streamer/collect/{label}", SENSOR)
            for label in ("base", "left_wrist")
        }
        self.jpg = make_jpeg()
        self.cmd_seen: list[np.ndarray] = []
        self.grip_seen: list[float] = []
        self.state = "?"
        self.last_payload: dict | None = None
        self.create_subscription(
            JointState, "/left_arm/joint_commands",
            lambda m: self.cmd_seen.append(np.asarray(m.position[:7])), SENSOR,
        )
        self.create_subscription(Float64, "/left_gripper/command", self._on_grip, SENSOR)
        self.create_subscription(String, "/policy_inference/state", self._on_state, 10)
        self._stop = threading.Event()

    def _on_state(self, msg: String) -> None:
        try:
            p = json.loads(msg.data)
            self.state = p.get("state", "?")
            self.last_payload = p
        except Exception:
            pass

    def _on_grip(self, msg: Float64) -> None:
        self.grip_seen.append(float(msg.data))

    def start_pump(self) -> None:
        def _pump():
            # 真实量级的关节位姿（含离开零位的 d3≈-1.9），让绝对语义守卫被真实模型
            # 验证通过（合成近零位姿会 fail-open，测不到守卫）
            home = np.zeros(7)
            home[3] = -1.8
            while not self._stop.is_set():
                m = JointState()
                m.header.stamp = self.get_clock().now().to_msg()
                m.position = [float(v) for v in home]
                self.js_pub.publish(m)
                g = Float64()
                g.data = 0.5
                self.grip_pub.publish(g)
                for label, pub in self.img_pubs.items():
                    c = CompressedImage()
                    c.header.stamp = self.get_clock().now().to_msg()
                    c.format = "jpeg"
                    c.data = self.jpg
                    pub.publish(c)
                time.sleep(0.03)

        threading.Thread(target=_pump, daemon=True).start()

    def stop_pump(self) -> None:
        self._stop.set()

    def send(self, verb: str) -> None:
        m = String()
        m.data = verb
        self.cmd_pub.publish(m)

    def wait_state(self, want: str, timeout_s: float = 20.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state == want:
                return True
        return self.state == want

    def drain(self, s: float) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.03)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--jpeg", action="store_true",
                    help="上行 camera 槽位走 JPEG（匹配 yaml jpeg_transport:true 的真实部署路径）")
    ap.add_argument("--check-server", action="store_true",
                    help="先直连 serve.py 做一次推理往返，快速定位 server 是否正常")
    args = ap.parse_args()

    if args.check_server:
        try:
            check_server(args.host, args.port, jpeg=args.jpeg)
            if fails:
                print("preflight 失败，中止节点流程")
                return 1
        except Exception as exc:  # noqa: BLE001
            check("preflight: connect serve.py", False, f"{type(exc).__name__}: {exc}")
            return 1

    rclpy.init()
    d = Driver()
    d.start_pump()
    d.drain(3.0)
    check("state latch + schema",
          d.state != "?" and d.last_payload is not None and d.last_payload.get("state_dim") == 8,
          f"state={d.state} dim={d.last_payload.get('state_dim') if d.last_payload else None}")

    d.send("policy")
    check("policy cmd -> POLICY (real ACT engine build)", d.wait_state("POLICY"),
          f"state={d.state}")
    n0, g0 = len(d.cmd_seen), len(d.grip_seen)
    eng_p0 = d.last_payload.get("engine", {}).get("pops", 0) if d.last_payload else 0
    t_rate0 = time.monotonic()
    d.drain(4.0)
    eng_p1 = d.last_payload.get("engine", {}).get("pops", 0) if d.last_payload else 0
    t_rate1 = time.monotonic()
    rate = (eng_p1 - eng_p0) / (t_rate1 - t_rate0) if t_rate1 > t_rate0 else 0.0
    new, ng = d.cmd_seen[n0:], d.grip_seen[g0:]
    distinct = len({tuple(np.round(v, 3)) for v in new}) if new else 0
    check("ACT drives arm commands", len(new) > 3 and distinct > 1,
          f"msgs={len(new)} distinct={distinct}")
    check("arm cmds finite", bool(np.isfinite(np.asarray(new)).all()),
          f"range [{np.nanmin(new):.3f},{np.nanmax(new):.3f}]")
    check("ACT streams gripper ratio in [0,1]",
          len(ng) > 2 and all(0 <= v <= 1 for v in ng),
          f"grip_msgs={len(ng)} range [{min(ng) if ng else '-'},{max(ng) if ng else '-'}]")
    check("achieved policy tick rate ~30Hz (engine pops/s)",
          25 <= rate <= 35, f"{rate:.1f} Hz (engine pops {eng_p0}->{eng_p1})")
    eng = d.last_payload.get("engine", {}) if d.last_payload else {}
    print(f"  info: engine stats {eng}")
    lat = d.last_payload.get("latency_ms", {}) if d.last_payload else {}
    if lat.get("loop") or lat.get("obs_age"):
        print(f"  info: node latency_ms {lat}")
    check("engine plans happened", bool(eng.get("plans", 0) >= 1), f"plans={eng.get('plans')}")

    d.send("pause")
    check("pause -> POLICY_PAUSED", d.wait_state("POLICY_PAUSED"), f"state={d.state}")
    d.drain(1.0)
    a = d.cmd_seen[-1] if d.cmd_seen else None
    d.drain(1.0)
    b = d.cmd_seen[-1] if d.cmd_seen else None
    stable = a is not None and b is not None and np.allclose(a, b, atol=1e-4)
    check("pause holds ACT target", stable,
          f"Δ={0 if stable else np.max(np.abs(a-b)) if a is not None and b is not None else float('nan'):.5f}")

    d.send("resume")
    check("resume -> POLICY (fresh replan)", d.wait_state("POLICY"), f"state={d.state}")
    d.send("stop")
    check("stop -> IDLE", d.wait_state("IDLE"), f"state={d.state}")

    d.stop_pump()
    d.destroy_node()
    rclpy.shutdown()
    print("RESULT", "ALL PASS" if not fails else "FAILURES: " + ",".join(fails))
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
