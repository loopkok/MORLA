#!/usr/bin/env python3
"""ROS2 tuning / live-retarget viewer for Wuji Hand.

Input: hand_landmarks/{side} (PoseArray) — from wuji_glove or quest3_hand_mocap.
Hand model: Wuji MJCF in this package.
Retarget backends (param retarget_backend):
  wuji_retargeting — TuningViewer (orange/cyan/white + YAML hot-reload)
  official         — wuji_sdk RetargetSession → MuJoCo mesh
  dexpilot         — dex-retargeting → MuJoCo mesh

Only wuji_retargeting has the three-layer skeleton tuning UI (upstream
tuning_tool). official/dexpilot still drive the hand live so you can compare
backends with the same landmarks stream.
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from wujihand_mujoco_sim.fps_counter import FPSCounter
from wujihand_mujoco_sim.qpos_remap import build_cmd_to_actuator_perm


def _sensor_data_qos() -> QoSProfile:
    """Match wujihand_driver joint_commands subscription (SensorDataQoS)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

_DEFAULT_CMD_NAMES = [
    f"finger{f}_joint{j}" for f in range(1, 6) for j in range(1, 5)
]


def _env_or_default(name: str, default: str) -> str:
    val = os.environ.get(name)
    if val is None or str(val).strip() == "":
        return default
    return str(val).strip()


def _pose_array_to_keypoints(msg: PoseArray) -> Optional[np.ndarray]:
    if len(msg.poses) < 21:
        return None
    kp = np.array(
        [[p.position.x, p.position.y, p.position.z] for p in msg.poses[:21]],
        dtype=np.float32,
    )
    if kp.shape != (21, 3):
        return None
    return kp


class WujiHandTuningNode(Node):
    def __init__(self):
        super().__init__("wujihand_tuning_node")

        default_side = _env_or_default("GLOVE_HAND_SIDE", "right")
        default_model = _env_or_default("GLOVE_HAND_MODEL", "wuji_hand")

        self.declare_parameter("hand_side", default_side)
        self.declare_parameter("retarget_backend", "wuji_retargeting")
        self.declare_parameter("hand_model", default_model)
        self.declare_parameter("landmarks_topic", "")
        self.declare_parameter("retarget_config", "")  # wuji_lib yaml override
        self.declare_parameter("dexpilot_config", "")
        self.declare_parameter("viz_config", "")
        self.declare_parameter("mjcf_path", "")
        self.declare_parameter("publish_joint_commands", True)
        self.declare_parameter("cmd_topic", "")
        self.declare_parameter("nlopt_max_eval", 25)
        self.declare_parameter("fps_print_interval", 5.0)

        self.hand_side = str(self.get_parameter("hand_side").value).lower()
        if self.hand_side not in ("left", "right"):
            raise ValueError("hand_side must be left|right")

        self.backend = str(self.get_parameter("retarget_backend").value).lower()
        if self.backend not in ("official", "wuji_retargeting", "dexpilot"):
            raise ValueError(
                "retarget_backend must be official|wuji_retargeting|dexpilot"
            )
        self.hand_model = str(self.get_parameter("hand_model").value).strip()

        landmarks_topic = str(self.get_parameter("landmarks_topic").value).strip()
        if not landmarks_topic:
            landmarks_topic = f"hand_landmarks/{self.hand_side}"
        self.landmarks_topic = landmarks_topic

        publish_cmds = bool(self.get_parameter("publish_joint_commands").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value).strip()
        if not cmd_topic:
            cmd_topic = f"/{self.hand_side}_hand/joint_commands"
        self.cmd_topic = cmd_topic
        self._cmd_pub = (
            self.create_publisher(JointState, self.cmd_topic, _sensor_data_qos())
            if publish_cmds
            else None
        )

        sim_share = Path(get_package_share_directory("wujihand_mujoco_sim"))
        mjcf_path = str(self.get_parameter("mjcf_path").value).strip()
        if not mjcf_path:
            mjcf_path = str(sim_share / "assets" / "mjcf" / f"{self.hand_side}.xml")
        self.mjcf_path = mjcf_path
        if not Path(self.mjcf_path).is_file():
            raise FileNotFoundError(f"MJCF not found: {self.mjcf_path}")

        viz_config = str(self.get_parameter("viz_config").value).strip()
        if not viz_config:
            viz_config = str(sim_share / "config" / "tuning_viz.yaml")
        self.viz_config = viz_config

        self._latest_kp: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._frame_count = 0
        self.viewer = None  # TuningViewer when wuji_retargeting
        self._backend = None  # official / dexpilot / also wuji_lib fallback
        self.model = None
        self.data = None
        self._perm = None
        fps_interval = float(self.get_parameter("fps_print_interval").value)
        self._input_fps = FPSCounter(window=100, print_interval=fps_interval)
        self._retarget_fps = FPSCounter(window=100, print_interval=fps_interval)
        self._loop_fps = FPSCounter(window=200, print_interval=fps_interval)

        import mujoco

        self._mujoco = mujoco

        if self.backend == "wuji_retargeting":
            self._init_tuning_viewer()
        else:
            self._init_mesh_backend()

        self.create_subscription(
            PoseArray, self.landmarks_topic, self._on_landmarks, 10
        )
        self.get_logger().info(
            f"Tuning ready: input={self.landmarks_topic}, "
            f"backend={self.backend}, side={self.hand_side}, mjcf={self.mjcf_path}"
        )

    def _init_tuning_viewer(self) -> None:
        retarget_config = str(self.get_parameter("retarget_config").value).strip()
        if not retarget_config:
            retarget_share = Path(
                get_package_share_directory("wujihand_retargeting")
            )
            retarget_config = str(
                retarget_share / "config" / f"retarget_wuji_lib_{self.hand_side}.yaml"
            )
        self.retarget_config = retarget_config
        if not Path(self.retarget_config).is_file():
            raise FileNotFoundError(f"retarget config not found: {self.retarget_config}")

        try:
            from wuji_retargeting.viz import TuningViewer
        except ImportError as exc:
            raise RuntimeError(
                "未找到 wuji_retargeting.viz.TuningViewer。请安装 wuji-retargeting。"
            ) from exc

        self.viewer = TuningViewer(
            hand_side=self.hand_side,
            retarget_config_path=self.retarget_config,
            viz_config_path=self.viz_config if Path(self.viz_config).is_file() else None,
            mjcf_path=self.mjcf_path,
        )
        self.model = self.viewer.model
        self.data = self.viewer.data
        self._midrange_init(self.model, self.data)
        self.get_logger().info(
            "3-layer TuningViewer: Orange=input Cyan=scaled White=FK. "
            f"Hot-reload: {Path(self.retarget_config).name}"
        )

    def _init_mesh_backend(self) -> None:
        """official / dexpilot: retarget backends + plain MuJoCo mesh (no 3-layer)."""
        if self.backend == "official":
            from wujihand_retargeting.backends.official import OfficialRetargetBackend

            self._backend = OfficialRetargetBackend(
                hand_side=self.hand_side, hand_model=self.hand_model
            )
            self.get_logger().info(
                "official backend: no YAML 3-layer tuning; compare pose visually. "
                "Params via hand_model / SDK builtin."
            )
        else:
            from wujihand_retargeting.backends.dexpilot import DexPilotRetargetBackend

            cfg = str(self.get_parameter("dexpilot_config").value).strip() or None
            self._backend = DexPilotRetargetBackend(
                hand_side=self.hand_side, config_path=cfg
            )
            self.get_logger().info(
                "dexpilot backend: mesh-only viewer. "
                "Edit retarget_dexpilot_*.yml then restart node to apply."
            )

        self.model = self._mujoco.MjModel.from_xml_path(self.mjcf_path)
        self.data = self._mujoco.MjData(self.model)

        act_names = []
        for i in range(self.model.nu):
            jid = self.model.actuator_trnid[i, 0]
            act_names.append(
                self._mujoco.mj_id2name(
                    self.model, self._mujoco.mjtObj.mjOBJ_JOINT, jid
                )
            )
        self._perm = build_cmd_to_actuator_perm(
            _DEFAULT_CMD_NAMES, act_names, self.hand_side
        )
        # Map command-order qpos → MJCF qpos slots (same idea as TuningViewer).
        self._qpos_addrs = self._build_cmd_qpos_addrs(
            self.model, _DEFAULT_CMD_NAMES, self.hand_side
        )
        self._open_hand_init(self.model, self.data)

    def _build_cmd_qpos_addrs(
        self, model, cmd_names: list, hand_side: str
    ) -> np.ndarray:
        addrs = []
        for name in cmd_names:
            jid = -1
            for candidate in (name, f"{hand_side}_{name}"):
                jid = self._mujoco.mj_name2id(
                    model, self._mujoco.mjtObj.mjOBJ_JOINT, candidate
                )
                if jid >= 0:
                    break
            if jid < 0:
                raise RuntimeError(f"MJCF missing joint for command name '{name}'")
            addrs.append(int(model.jnt_qposadr[jid]))
        return np.asarray(addrs, dtype=int)

    def _open_hand_init(self, model, data) -> None:
        """Pose near joint lower limits (open), not mid-range (looks clenched)."""
        for i in range(model.nu):
            if model.actuator_ctrllimited[i]:
                lo, _hi = model.actuator_ctrlrange[i]
                data.ctrl[i] = float(lo)
            else:
                data.ctrl[i] = 0.0
        # Mirror into qpos so the first viewer frame is open.
        for i in range(model.nu):
            jid = model.actuator_trnid[i, 0]
            qadr = int(model.jnt_qposadr[jid])
            data.qpos[qadr] = data.ctrl[i]
        self._mujoco.mj_forward(model, data)

    def _on_landmarks(self, msg: PoseArray) -> None:
        kp = _pose_array_to_keypoints(msg)
        if kp is None or np.allclose(kp, 0):
            return
        with self._lock:
            self._latest_kp = kp
            self._frame_count += 1
        fps = self._input_fps.tick()
        if self._input_fps.should_print():
            self.get_logger().info(
                f"[Tuning Input FPS][{self.hand_side}] {fps:.0f} Hz "
                f"(topic={self.landmarks_topic})"
            )

    def _publish_cmds(self, qpos: np.ndarray) -> None:
        if self._cmd_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(_DEFAULT_CMD_NAMES)
        q = np.asarray(qpos, dtype=np.float64).reshape(-1)
        msg.position = [float(x) for x in q[:20]]
        self._cmd_pub.publish(msg)

    def _apply_mesh_qpos(self, qpos: np.ndarray) -> None:
        """Drive mesh by writing qpos (instant), not soft position actuators.

        MJCF kp≈0.2–0.4; one ``mj_step`` on ``ctrl`` barely moves the hand, so
        dexpilot/official looked stuck in a clenched mid-range pose.
        """
        q = np.asarray(qpos, dtype=np.float64).reshape(-1)
        n = min(len(q), len(self._qpos_addrs))
        for i in range(n):
            self.data.qpos[self._qpos_addrs[i]] = q[i]
        # Keep ctrl in sync so any later mj_step does not yank joints back.
        if self._perm is not None:
            ctrl = q[self._perm]
        else:
            ctrl = q
        nu = self.model.nu
        self.data.ctrl[: min(len(ctrl), nu)] = ctrl[:nu]
        self._mujoco.mj_forward(self.model, self.data)

    def run(self) -> None:
        import mujoco.viewer

        if self.backend == "wuji_retargeting":
            self._run_tuning_viewer(mujoco.viewer)
        else:
            self._run_mesh_viewer(mujoco.viewer)

    def _run_tuning_viewer(self, viewer_mod) -> None:
        last_result = None

        with viewer_mod.launch_passive(self.model, self.data) as mj_viewer:
            self.viewer._set_camera(mj_viewer)
            while rclpy.ok() and mj_viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0.0)

                changed, new_config = self.viewer.config_watcher.check()
                if changed:
                    self.viewer._reload_retargeter(new_config, [])
                    self.viewer.retargeter.reset()
                    self.get_logger().info("Retarget config hot-reloaded")

                self.viewer._check_highlight_timeout()

                with self._lock:
                    kp = None if self._latest_kp is None else self._latest_kp.copy()

                if kp is not None:
                    try:
                        last_result = self.viewer._process_frame(kp)
                        qpos = last_result["qpos"]
                        self.data.qpos[:] = qpos[self.viewer._qpos_perm]
                        self._mujoco.mj_forward(self.model, self.data)
                        self._publish_cmds(qpos)
                        r_fps = self._retarget_fps.tick()
                        if self._retarget_fps.should_print():
                            self.get_logger().info(
                                f"[Tuning Retarget FPS][{self.hand_side}]"
                                f"[{self.backend}] {r_fps:.0f} Hz"
                            )
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warning(f"tuning frame failed: {exc}")

                if last_result is not None:
                    with mj_viewer.lock():
                        self.viewer.drawer.draw(
                            mj_viewer.user_scn,
                            mediapipe_kp=last_result["mediapipe_kp"],
                            scaled_kp=last_result["scaled_kp"],
                            pinch_alphas=last_result.get("pinch_alphas"),
                        )

                mj_viewer.sync()
                loop_fps = self._loop_fps.tick()
                if self._loop_fps.should_print():
                    self.get_logger().info(
                        f"[Tuning Loop FPS][{self.hand_side}] {loop_fps:.0f} Hz"
                    )
                time.sleep(0.01)

    def _run_mesh_viewer(self, viewer_mod) -> None:
        with viewer_mod.launch_passive(self.model, self.data) as mj_viewer:
            mj_viewer.cam.azimuth = 135
            mj_viewer.cam.elevation = -20
            mj_viewer.cam.distance = 0.5
            mj_viewer.cam.lookat[:] = [0, 0, 0.05]

            while rclpy.ok() and mj_viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0.0)

                with self._lock:
                    kp = None if self._latest_kp is None else self._latest_kp.copy()

                if kp is not None:
                    try:
                        qpos = self._backend.retarget(kp)
                        self._apply_mesh_qpos(qpos)
                        self._publish_cmds(qpos)
                        r_fps = self._retarget_fps.tick()
                        if self._retarget_fps.should_print():
                            self.get_logger().info(
                                f"[Tuning Retarget FPS][{self.hand_side}]"
                                f"[{self.backend}] {r_fps:.0f} Hz"
                            )
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warning(f"retarget failed: {exc}")
                else:
                    self._mujoco.mj_step(self.model, self.data)

                mj_viewer.sync()
                loop_fps = self._loop_fps.tick()
                if self._loop_fps.should_print():
                    self.get_logger().info(
                        f"[Tuning Loop FPS][{self.hand_side}] {loop_fps:.0f} Hz"
                    )
                time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = WujiHandTuningNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
