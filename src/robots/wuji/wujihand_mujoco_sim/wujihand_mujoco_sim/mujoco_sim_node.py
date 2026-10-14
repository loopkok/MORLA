#!/usr/bin/env python3
"""MuJoCo sim node — subscribe joint_commands, drive Wuji Hand MJCF.

Mirrors the control/visual loop in wuji-retargeting/example/teleop_sim.py,
but takes ROS2 JointState commands instead of running retarget in-process.

Typical pipeline (no real hand):
  wuji_glove / quest3 → wujihand_retargeting → /{hand}/joint_commands
                                              → this node (MuJoCo viewer)
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def _sensor_data_qos() -> QoSProfile:
    """Match wujihand_retargeting / wujihand_driver joint_commands QoS."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

from wujihand_mujoco_sim.fps_counter import FPSCounter
from wujihand_mujoco_sim.qpos_remap import build_cmd_to_actuator_perm

# Default command joint order (same as wujihand_retargeting.constants)
_DEFAULT_CMD_NAMES = [
    f"finger{f}_joint{j}" for f in range(1, 6) for j in range(1, 5)
]


def _env_or_default(name: str, default: str) -> str:
    val = os.environ.get(name)
    if val is None or str(val).strip() == "":
        return default
    return str(val).strip()


class WujiHandMujocoSimNode(Node):
    def __init__(self):
        super().__init__("wujihand_mujoco_sim_node")

        default_side = _env_or_default("GLOVE_HAND_SIDE", "right")
        self.declare_parameter("hand_side", default_side)
        self.declare_parameter("hand_name", "")  # default: {side}_hand
        self.declare_parameter("mjcf_path", "")  # empty → package assets
        self.declare_parameter("cmd_topic", "")  # empty → /{hand_name}/joint_commands
        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("realtime", True)  # sleep model.opt.timestep
        self.declare_parameter("fps_print_interval", 5.0)

        self.hand_side = str(self.get_parameter("hand_side").value).lower()
        if self.hand_side not in ("left", "right"):
            raise ValueError("hand_side must be left|right")

        hand_name = str(self.get_parameter("hand_name").value).strip()
        if not hand_name:
            hand_name = f"{self.hand_side}_hand"
        self.hand_name = hand_name

        cmd_topic = str(self.get_parameter("cmd_topic").value).strip()
        if not cmd_topic:
            cmd_topic = f"/{self.hand_name}/joint_commands"
        self.cmd_topic = cmd_topic

        self.enable_viewer = bool(self.get_parameter("enable_viewer").value)
        self.realtime = bool(self.get_parameter("realtime").value)

        mjcf_path = str(self.get_parameter("mjcf_path").value).strip()
        if not mjcf_path:
            share = Path(get_package_share_directory("wujihand_mujoco_sim"))
            mjcf_path = str(share / "assets" / "mjcf" / f"{self.hand_side}.xml")
        self.mjcf_path = mjcf_path
        if not Path(self.mjcf_path).is_file():
            raise FileNotFoundError(
                f"MJCF not found: {self.mjcf_path}. "
                "Ensure package assets were installed (meshes + mjcf)."
            )

        try:
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError(
                "未找到 mujoco。请安装：pip install mujoco"
            ) from exc

        self._mujoco = mujoco
        self._viewer_mod = mujoco.viewer

        self.get_logger().info(f"Loading MJCF: {self.mjcf_path}")
        self.model = mujoco.MjModel.from_xml_path(self.mjcf_path)
        self.data = mujoco.MjData(self.model)

        # Actuator → joint name order
        act_joint_names = []
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i, 0]
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            act_joint_names.append(name)
        self._act_joint_names = act_joint_names

        self._perm = build_cmd_to_actuator_perm(
            _DEFAULT_CMD_NAMES, act_joint_names, self.hand_side
        )
        if self._perm is None:
            self.get_logger().warn(
                "Could not align cmd joint names to MJCF actuators; "
                "using positional index mapping (may be wrong for Hand2)."
            )
        else:
            self.get_logger().info(
                f"Cmd→actuator remap OK (nu={self.model.nu})"
            )

        self._qpos_addrs = self._build_cmd_qpos_addrs(
            _DEFAULT_CMD_NAMES, self.hand_side
        )
        # Open hand (lower ctrl limits), not mid-range clenched pose.
        for i in range(self.model.nu):
            if self.model.actuator_ctrllimited[i]:
                lo, _hi = self.model.actuator_ctrlrange[i]
                self.data.ctrl[i] = float(lo)
            else:
                self.data.ctrl[i] = 0.0
            jid = self.model.actuator_trnid[i, 0]
            self.data.qpos[int(self.model.jnt_qposadr[jid])] = self.data.ctrl[i]
        mujoco.mj_forward(self.model, self.data)

        self._latest_qpos: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._cmd_count = 0
        fps_interval = float(self.get_parameter("fps_print_interval").value)
        self._cmd_fps = FPSCounter(window=100, print_interval=fps_interval)
        self._sim_fps = FPSCounter(window=200, print_interval=fps_interval)

        self.create_subscription(
            JointState, self.cmd_topic, self._on_cmd, _sensor_data_qos()
        )
        self.get_logger().info(
            f"MuJoCo sim ready: side={self.hand_side}, topic={self.cmd_topic}, "
            f"viewer={self.enable_viewer}"
        )

    def _build_cmd_qpos_addrs(self, cmd_names: List[str], hand_side: str) -> np.ndarray:
        addrs = []
        for name in cmd_names:
            jid = -1
            for candidate in (name, f"{hand_side}_{name}"):
                jid = self._mujoco.mj_name2id(
                    self.model, self._mujoco.mjtObj.mjOBJ_JOINT, candidate
                )
                if jid >= 0:
                    break
            if jid < 0:
                raise RuntimeError(f"MJCF missing joint for command name '{name}'")
            addrs.append(int(self.model.jnt_qposadr[jid]))
        return np.asarray(addrs, dtype=int)

    def _on_cmd(self, msg: JointState) -> None:
        if not msg.position:
            return
        q = np.asarray(msg.position, dtype=np.float64).reshape(-1)
        # Prefer name-aligned packing into default cmd order when names present.
        if msg.name and len(msg.name) == len(q):
            name_to_q = {n: float(v) for n, v in zip(msg.name, q)}
            packed = np.array(
                [name_to_q.get(n, 0.0) for n in _DEFAULT_CMD_NAMES],
                dtype=np.float64,
            )
            # If few names matched, fall back to positional.
            matched = sum(1 for n in _DEFAULT_CMD_NAMES if n in name_to_q)
            if matched >= 16:
                q = packed
        if q.size < 20:
            self.get_logger().warning(
                f"joint_commands len={q.size} < 20，忽略"
            )
            return
        with self._lock:
            self._latest_qpos = q[:20].copy()
            self._cmd_count += 1
        fps = self._cmd_fps.tick()
        if self._cmd_fps.should_print():
            self.get_logger().info(
                f"[Sim Cmd FPS][{self.hand_side}] {fps:.0f} Hz "
                f"(topic={self.cmd_topic})"
            )

    def _apply_ctrl(self, qpos: np.ndarray) -> None:
        """Write qpos directly (viz teleop). Soft MJCF kp makes ctrl-only lag badly."""
        q = np.asarray(qpos, dtype=np.float64).reshape(-1)
        n = min(len(q), len(self._qpos_addrs))
        for i in range(n):
            self.data.qpos[self._qpos_addrs[i]] = q[i]
        if self._perm is not None:
            ctrl = q[self._perm]
        else:
            ctrl = q
        nu = self.model.nu
        self.data.ctrl[: min(len(ctrl), nu)] = ctrl[:nu]
        self._mujoco.mj_forward(self.model, self.data)
    def run(self) -> None:
        """Blocking sim loop (call instead of rclpy.spin)."""
        viewer = None
        if self.enable_viewer:
            viewer = self._viewer_mod.launch_passive(self.model, self.data)
            viewer.cam.azimuth = 180
            viewer.cam.elevation = -20
            viewer.cam.distance = 0.5
            viewer.cam.lookat[:] = [0, 0, 0.05]

        try:
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                rclpy.spin_once(self, timeout_sec=0.0)

                with self._lock:
                    qpos = None if self._latest_qpos is None else self._latest_qpos.copy()

                if qpos is not None:
                    self._apply_ctrl(qpos)  # sets qpos + mj_forward
                else:
                    self._mujoco.mj_step(self.model, self.data)

                if viewer is not None:
                    viewer.sync()

                sim_fps = self._sim_fps.tick()
                if self._sim_fps.should_print():
                    self.get_logger().info(
                        f"[Sim Loop FPS][{self.hand_side}] {sim_fps:.0f} Hz "
                        f"(cmds_total={self._cmd_count})"
                    )

                if self.realtime:
                    time.sleep(self.model.opt.timestep)
        finally:
            if viewer is not None:
                viewer.close()


def main(args=None):
    rclpy.init(args=args)
    node = WujiHandMujocoSimNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
