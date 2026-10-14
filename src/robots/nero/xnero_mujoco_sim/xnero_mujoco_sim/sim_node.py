#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""xnero MuJoCo 仿真节点 —— 硬件驱动的替身。

顶替 Nero SDK 与 XHand 串口驱动：标准臂命令进入 `/{side}_arm/joint_commands`；
XHand adapter 将标准手命令转到 `/{side}_hand/driver/xhand_command`。仿真分别发布
标准臂实测状态和原生手状态/逐点触觉。

物理模型（见 spec）：
  - **手**：真物理。`<position>` 执行器 + `mj_step`，接触真实、触觉力真实。
  - **臂**：运动学覆盖。每步在 `mj_step` 之前把臂关节 `qpos` 写成命令值、`qvel` 清零，
    语义是"无限刚 Kinematic 体"——能推动物体，自己不被推动。
"""

from __future__ import annotations

import re
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from morla_core.resource_locator import package_share, resolve_robot_asset

from xnero_mujoco_sim.fps_counter import FPSCounter
from xnero_mujoco_sim.hand_map import (
    ARM_STATE_JOINT_NAMES,
    FINGERS,
    HAND_JOINT_NAMES,
    LOCATION_LABEL,
    NUM_HAND_JOINTS,
    actuator_name,
    arm_joint_names,
    mjcf_hand_joint,
    parse_tactile_geom,
)
from xnero_mujoco_sim.latency_meter import LatencyMeter, stamp_age_ms

# 与真机两端对齐：命令发布方用默认（RELIABLE depth 10），驱动发状态用 QoS(1)
QOS_CMD = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
QOS_STATE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)

DESC_PKG = "robot_description"


# --------------------------------------------------------------------------- #
# 可单测的纯函数
# --------------------------------------------------------------------------- #
def pack_hand_command(msg, side: str) -> Optional[Dict[str, float]]:
    """XHandCommand → {MJCF 关节名: 目标角}。

    优先按 `name` 匹配（真机驱动的做法）；`name` 为空或对不上时按顺序取前 12 个。
    点数不足 12 → 返回 None（丢弃，避免把半个命令写进去）。
    """
    pos = [float(v) for v in msg.position]
    if len(pos) != NUM_HAND_JOINTS or not np.isfinite(pos).all():
        return None
    names = list(msg.name) if msg.name else []
    if len(names) != NUM_HAND_JOINTS or len(set(names)) != NUM_HAND_JOINTS:
        return None
    by_name = {n: v for n, v in zip(names, pos)}
    if set(by_name) != set(HAND_JOINT_NAMES):
        return None
    return {mjcf_hand_joint(side, n): by_name[n] for n in HAND_JOINT_NAMES}


def rewrite_meshdir(text: str, abs_meshdir: str) -> str:
    """把 MJCF 里的 meshdir 换成绝对路径。

    产物里写的是相对路径（源码树可直接加载），但安装后层级不同，
    所以节点加载前统一覆盖成 robot_description share 下的 meshes。
    """
    if 'meshdir="' in text:
        return re.sub(r'meshdir="[^"]*"', f'meshdir="{abs_meshdir}"', text, count=1)
    return text.replace("<compiler ", f'<compiler meshdir="{abs_meshdir}" ', 1)


def default_mjcf_path() -> Path:
    return resolve_robot_asset("xnero_dual_sim")


def _spin_drain(node: Node, max_callbacks: int = 32) -> None:
    """非阻塞抽干回调队列，避免 rclpy 阻塞物理循环。"""
    for _ in range(max_callbacks):
        rclpy.spin_once(node, timeout_sec=0.0)


# --------------------------------------------------------------------------- #
class XneroMujocoSimNode(Node):
    def __init__(self, *, viewer: Optional[bool] = None,
                 parameter_overrides: Optional[dict] = None) -> None:
        """parameter_overrides: {参数名: 值}。rclpy 的 Node 要的是 Parameter 列表，
        这里收 dict 更顺手（测试里常用来指一份临时 MJCF）。"""
        overrides = None
        if parameter_overrides:
            from rclpy.parameter import Parameter  # noqa: PLC0415
            overrides = [Parameter(k, value=v) for k, v in parameter_overrides.items()]
        super().__init__("xnero_mujoco_sim_node", parameter_overrides=overrides)

        self.declare_parameter("mjcf_path", "")
        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("realtime", True)
        self.declare_parameter("state_rate", 50.0)
        self.declare_parameter("hand_kp", 5.0)
        self.declare_parameter("hand_kv", 0.1)
        self.declare_parameter("hand_torque_limit", 0.5)
        self.declare_parameter("publish_tactile", True)
        self.declare_parameter("tactile_rate", 50.0)
        self.declare_parameter("arm_command_timeout", 0.5)
        self.declare_parameter("init_pose_left", [0.0] * 7)
        self.declare_parameter("init_pose_right", [0.0] * 7)
        self.declare_parameter("hand_id_left", 0)
        self.declare_parameter("hand_id_right", 0)
        self.declare_parameter("fps_print_interval", 5.0)
        self.declare_parameter("print_latency", True)
        self.declare_parameter("latency_print_interval", 2.0)

        try:
            import mujoco
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("需要 mujoco：/usr/bin/python3 -m pip install mujoco") from exc

        self._mj = mujoco
        p = str(self.get_parameter("mjcf_path").value).strip()
        mjcf_path = Path(p) if p else default_mjcf_path()
        if not mjcf_path.is_file():
            raise FileNotFoundError(f"MJCF 不存在：{mjcf_path}")
        meshdir = str(package_share(DESC_PKG) / "meshes")
        xml = rewrite_meshdir(mjcf_path.read_text(encoding="utf-8"), meshdir)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.mjcf_path = mjcf_path

        # ---- 索引表 ----
        # 一律按 MJCF 全名（{side}_hand_{joint}）索引 —— pack_hand_command 返回的也是全名
        self._hand_act: Dict[str, int] = {}
        self._hand_qadr: Dict[str, int] = {}
        self._hand_ctrlrange: Dict[str, tuple] = {}
        for side in ("left", "right"):
            for name in HAND_JOINT_NAMES:
                jn = mjcf_hand_joint(side, name)
                an = actuator_name(side, name)
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, an)
                if aid < 0:
                    raise RuntimeError(f"MJCF 缺执行器 {an}（先跑 make_mjcf.py）")
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid < 0:
                    raise RuntimeError(f"MJCF 缺关节 {jn}")
                self._hand_act[jn] = int(aid)
                self._hand_qadr[jn] = int(self.model.jnt_qposadr[jid])
                self._hand_ctrlrange[jn] = tuple(
                    float(v) for v in self.model.actuator_ctrlrange[aid])

        self._arm_joints = {"left": arm_joint_names("left"),
                            "right": arm_joint_names("right")}
        self._arm_qadr: Dict[str, np.ndarray] = {}
        self._arm_vadr: Dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                    for n in self._arm_joints[side]]
            self._arm_qadr[side] = np.array(
                [int(self.model.jnt_qposadr[j]) for j in jids], dtype=int)
            self._arm_vadr[side] = np.array(
                [int(self.model.jnt_dofadr[j]) for j in jids], dtype=int)

        # ---- 运行态 ----
        self._lock = threading.Lock()
        self._hand_target: Dict[str, Dict[str, float]] = {"left": {}, "right": {}}
        self._arm_cmd: Dict[str, np.ndarray] = {
            side: np.array(self.get_parameter(f"init_pose_{side}").value, dtype=float).reshape(7)
            for side in ("left", "right")
        }
        self._arm_cmd_mono = {"left": 0.0, "right": 0.0}
        self._cmd_count = {"left": 0, "right": 0}
        self._tac_geom: Dict[str, Dict[str, int]] = {"left": {}, "right": {}}
        for gid in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            parsed = parse_tactile_geom(name or "")
            if parsed:
                side, finger, point = parsed
                self._tac_geom.setdefault(side, {}).setdefault(finger, {})[gid] = point

        # gid → (side, finger, point) 反查表（接触遍历的热路径，别在循环里解析名字）
        self._tac_by_gid: Dict[int, tuple] = {}
        for side_, fingers_ in self._tac_geom.items():
            for finger_, pts_ in fingers_.items():
                for gid_, point_ in pts_.items():
                    self._tac_by_gid[gid_] = (side_, finger_, point_)

        self._publish_tactile = bool(self.get_parameter("publish_tactile").value)
        self._state_rate = float(self.get_parameter("state_rate").value)
        self._tactile_rate = float(self.get_parameter("tactile_rate").value)
        self._arm_timeout = float(self.get_parameter("arm_command_timeout").value)
        self._print_latency = bool(self.get_parameter("print_latency").value)
        self._lat = LatencyMeter(float(self.get_parameter("latency_print_interval").value))
        interval = float(self.get_parameter("fps_print_interval").value)
        self._cmd_fps = {s: FPSCounter(window=200, print_interval=interval)
                         for s in ("left", "right")}
        self._sim_fps = FPSCounter(window=200, print_interval=interval)

        # ---- ROS 接口 ----
        self._state_pubs = {}
        self._arm_state_pubs = {}
        for side in ("left", "right"):
            self.create_subscription(
                # XHandCommand 在运行时才 import，避免无接口包时整个节点起不来
                _xhand_msg("XHandCommand"), f"/{side}_hand/driver/xhand_command",
                lambda m, s=side: self._on_hand_cmd(s, m), QOS_CMD)
            self.create_subscription(
                JointState, f"/{side}_arm/joint_commands",
                lambda m, s=side: self._on_arm_cmd(s, m), QOS_CMD)
            self._state_pubs[side] = self.create_publisher(
                _xhand_msg("XHandStateArray"), f"/{side}_hand/driver/xhand_state", QOS_STATE)
            self._arm_state_pubs[side] = self.create_publisher(
                JointState, f"/{side}_arm/joint_states", QOS_STATE)

        self._enable_viewer = bool(self.get_parameter("enable_viewer").value) \
            if viewer is None else bool(viewer)
        self._realtime = bool(self.get_parameter("realtime").value)
        self._t_next_state = 0.0
        self._t_next_tactile = 0.0
        self.get_logger().info(
            f"xnero MuJoCo sim 就绪：MJCF={mjcf_path.name} nq={self.model.nq} "
            f"nu={self.model.nu} 触觉 geom="
            f"{sum(len(p) for fs in self._tac_geom.values() for p in fs.values())} "
            f"viewer={self._enable_viewer}")

    # ------------------------------------------------------------------ #
    def _on_hand_cmd(self, side: str, msg) -> None:
        target = pack_hand_command(msg, side)
        if target is None:
            return
        with self._lock:
            self._hand_target[side] = target
            self._cmd_count[side] += 1
        fps = self._cmd_fps[side].tick()
        if self._cmd_fps[side].should_print():
            self.get_logger().info(
                f"[手命令][{side}] {fps:.0f} Hz total={self._cmd_count[side]}")

    def _on_arm_cmd(self, side: str, msg: JointState) -> None:
        if len(msg.position) != 7 or len(msg.name) != 7 or len(set(msg.name)) != 7:
            return
        q = np.asarray(msg.position, dtype=float).reshape(-1)
        if len(q) != 7 or not np.isfinite(q).all():
            return
        names = list(msg.name)
        if set(names) != set(self._arm_joints[side]):
            return
        lut = {n: v for n, v in zip(names, q)}
        q = np.array([lut[n] for n in self._arm_joints[side]])
        with self._lock:
            self._arm_cmd[side] = q[:7].copy()
            self._arm_cmd_mono[side] = time.monotonic()

    # ------------------------------------------------------------------ #
    def _apply_hands(self) -> None:
        m, d = self.model, self.data
        with self._lock:
            targets = {s: dict(self._hand_target[s]) for s in ("left", "right")}
        for side in ("left", "right"):
            for jn, val in targets[side].items():      # jn 是 MJCF 全名
                aid = self._hand_act[jn]
                lo, hi = self._hand_ctrlrange[jn]
                d.ctrl[aid] = float(np.clip(val, lo, hi))

    def _apply_arms(self) -> None:
        """运动学覆盖：直接把命令写进 qpos，并把速度清零。

        必须在 mj_step 之前调用，臂才会精确跟命令且不受重力/接触影响。
        """
        d = self.data
        with self._lock:
            cmds = {s: self._arm_cmd[s].copy() for s in ("left", "right")}
        for side in ("left", "right"):
            d.qpos[self._arm_qadr[side]] = cmds[side]
            d.qvel[self._arm_vadr[side]] = 0.0

    # ------------------------------------------------------------------ #
    def _fill_hand_state(self, side: str, msg) -> None:
        from xhand_control_interfaces.msg import XHandState  # noqa: PLC0415
        st = XHandState()
        st.name = [mjcf_hand_joint(side, n) for n in HAND_JOINT_NAMES]
        st.position = []
        st.effort = []
        st.temperature = []
        st.error_code = []
        for n in HAND_JOINT_NAMES:
            jn = mjcf_hand_joint(side, n)
            st.position.append(float(self.data.qpos[self._hand_qadr[jn]]))
            st.effort.append(float(self.data.actuator_force[self._hand_act[jn]]))
            st.temperature.append(25.0)
            st.error_code.append(0)
        msg.hand_id = [int(self.get_parameter(f"hand_id_{side}").value)]
        msg.hand_name = [f"{side}_hand"]
        msg.hand_type = ["XHAND1"]
        msg.hand_states = [st]

    def collect_tactile(self, side: str) -> Dict[str, Dict[str, np.ndarray]]:
        """逐点接触力，返回 {finger: {"raw": (120,3), "calc": (3,)}}。

        力的表达：**该指尖 link2 坐标系**下的三分量（触觉球无自身旋转，所以 geom 系
        就是 link2 系）。这样 120 个点在同一坐标系里，可以直接求和、看方向。
        `raw[k]` 对应 SDK `raw_force[k]`。

        注意：真机传感器给的是**传感器自身三轴**的分量，轴约定与这里不同 ——
        幅值可比，分量方向需按安装姿态换算（见 README 的说明）。
        """
        m, d = self.model, self.data
        out = {f: {"raw": np.zeros((120, 3)), "calc": np.zeros(3)} for f in FINGERS}
        if d.ncon == 0:
            return out
        res = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            e1 = self._tac_by_gid.get(g1)
            e2 = self._tac_by_gid.get(g2)
            if e2 is not None:
                entry, tac_gid = e2, g2          # 触觉 geom 是 geom2
            elif e1 is not None:
                entry, tac_gid = e1, g1          # 触觉 geom 是 geom1
            else:
                continue
            s_, finger_, point_ = entry
            if s_ != side:
                continue
            self._mj.mj_contactForce(m, d, i, res)  # 接触系下的 6 维力/力矩
            frame = np.asarray(c.frame, dtype=float).reshape(3, 3)   # 接触系三轴（世界）
            f_world = frame @ np.asarray(res[:3], dtype=float)
            xmat = np.asarray(d.geom_xmat[tac_gid], dtype=float).reshape(3, 3)
            f_local = xmat.T @ f_world        # 世界 → link2 系
            # mj_contactForce 的符号约定是「作用在 geom2 上」；统一成「作用在触觉点上」
            if tac_gid == g1:
                f_local = -f_local
            out[finger_]["raw"][point_ - 1] += f_local
            out[finger_]["calc"] += f_local
        return out

    def _fill_tactile(self, side: str, msg) -> None:
        """把逐点接触力填进 sensor_states（与真机驱动的字段结构一致）。"""
        from geometry_msgs.msg import Vector3  # noqa: PLC0415
        from xhand_control_interfaces.msg import (  # noqa: PLC0415
            FingerSensorState, XHandSensorState)

        tac = self.collect_tactile(side)
        ss = XHandSensorState()
        for finger in FINGERS:
            raw, calc = tac[finger]["raw"], tac[finger]["calc"]
            fss = FingerSensorState()
            fss.location = LOCATION_LABEL[finger]
            fss.raw_force = [Vector3(x=float(a), y=float(b), z=float(c))
                             for a, b, c in raw]
            fss.calc_force = Vector3(x=float(calc[0]), y=float(calc[1]), z=float(calc[2]))
            fss.raw_temperature = [0] * 20
            fss.calc_temperature = 0
            ss.finger_sensor_states.append(fss)
        msg.sensor_states = [ss]

    def _publish_states(self) -> None:
        from xhand_control_interfaces.msg import XHandStateArray  # noqa: PLC0415
        now_msg = self.get_clock().now().to_msg()
        for side in ("left", "right"):
            msg = XHandStateArray()
            msg.header.stamp = now_msg
            self._fill_hand_state(side, msg)
            if self._publish_tactile:
                self._fill_tactile(side, msg)
            self._state_pubs[side].publish(msg)
            js = JointState()
            js.header.stamp = now_msg
            js.name = list(ARM_STATE_JOINT_NAMES[side])
            js.position = [float(self.data.qpos[a]) for a in self._arm_qadr[side]]
            self._arm_state_pubs[side].publish(js)

    # ------------------------------------------------------------------ #
    def step_once(self) -> None:
        """跑一轮：抽干回调 → 写输入 → 物理步进 → 按需发布。"""
        _spin_drain(self, 32)
        t0 = time.monotonic()
        self._apply_arms()
        self._apply_hands()
        self._mj.mj_step(self.model, self.data)
        # 步进会把臂的 qpos 积分走一点（重力/接触加速度 × dt²），
        # 而运动学体要求「对外看到的姿态就是命令值」——所以步进后再写一次。
        self._apply_arms()
        if self._viewer is not None:
            self._viewer.sync()
        if self._print_latency:
            self._lat.add("step", (time.monotonic() - t0) * 1000.0)

        now = time.monotonic()
        state_period = 1.0 / max(1.0, self._state_rate)
        if self._t_next_state <= 0.0:
            self._t_next_state = now
        if now >= self._t_next_state:
            # Advance against a fixed schedule instead of resetting to
            # ``now + period``. The latter drifts down to one publish every
            # five 220-Hz MuJoCo steps (about 44 Hz for a requested 50 Hz).
            self._t_next_state += state_period
            if self._t_next_state <= now:
                self._t_next_state = now + state_period
            self._publish_states()
        fps = self._sim_fps.tick()
        if self._sim_fps.should_print():
            self.get_logger().info(
                f"[仿真循环] {fps:.0f} Hz 手命令 L={self._cmd_count['left']} "
                f"R={self._cmd_count['right']}")
        if self._print_latency and self._lat.has_samples() and self._lat.should_print():
            self.get_logger().info(f"[Latency][sim] {self._lat.format_and_reset()}")

    def run(self) -> None:
        self._viewer = None
        if self._enable_viewer:
            import mujoco.viewer  # noqa: PLC0415
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._viewer.cam.azimuth = 130
            self._viewer.cam.elevation = -10
            self._viewer.cam.distance = 1.4
            self._viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
        try:
            while rclpy.ok() and (self._viewer is None or self._viewer.is_running()):
                self.step_once()
                if self._realtime:
                    time.sleep(self.model.opt.timestep)
        finally:
            if self._viewer is not None:
                self._viewer.close()

    _viewer = None


def _xhand_msg(name: str):
    """延迟 import 接口消息（缺接口包时报错更清楚）。"""
    import xhand_control_interfaces.msg as m  # noqa: PLC0415
    return getattr(m, name)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XneroMujocoSimNode()
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
