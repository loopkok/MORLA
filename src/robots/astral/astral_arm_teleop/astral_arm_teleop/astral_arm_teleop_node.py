#!/usr/bin/env python3
"""Single-arm Astral teleop (Nero layout).

Quest wrist → PoseProcessor → IK (``geometric`` default; ``analytic_dh`` /
``urdf_numerical`` selectable) → SafetyFilter → ``/{side}_arm/joint_commands``.

With ``use_human_elbow`` (default on, geometric only), the Quest IOBT
shoulder/elbow from ``quest3/body_joints`` pins the arm angle psi_ref so the
robot elbow plane follows the human's.

Use two instances (left + right) for dual-arm (process-level parallel).
"""

from __future__ import annotations

import json
import math
import time
import uuid
from collections import deque
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from astral_arm_teleop.ik.factory import make_single_arm_ik
from astral_arm_teleop.latency_meter import LatencyMeter, stamp_age_ms
from astral_arm_teleop.pose_processor import PoseProcessor
from astral_arm_teleop.safety_filter import SafetyFilter
from astral_arm_teleop.teleop_log import TeleopJsonlLog, teleop_event_record
from morla_core.coordinates import (
    solver_tip_to_tcp, tcp_to_solver_tip, validate_rotation, validate_transform,
)
from morla_core.ownership import PolicyLease, fresh_header

_LEFT_NAMES = [
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_elbow_roll",
    "left_elbow_pitch",
    "left_forearm_roll",
    "left_wrist_pitch",
    "left_wrist_roll",
]
_RIGHT_NAMES = [
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_elbow_roll",
    "right_elbow_pitch",
    "right_forearm_roll",
    "right_wrist_pitch",
    "right_wrist_roll",
]

# Joints whose positive direction was flipped to make the arm alpha = +90 deg
# (see ik/analytic.py AstralParams). The DH/IK works in this "flipped" convention;
# the real hardware / MJCF firmware uses the original (SolidWorks) convention, so
# the teleop node flips these joint signs at the IK boundary.
_JOINT_FLIP = {
    "left": [False, True, True, True, False, False, False],
    "right": [False, True, False, True, False, False, False],
}

# Rotation that maps a vector from the original base frame (SolidWorks torso,
# joint1 axis = -X) into the analytic-DH "clean base" frame (joint1 axis = +Z).
# Only the analytic DH solver works in the clean base; the URDF numerical solver
# stays in the original base frame, so this rotation is applied only when
# solver_type == analytic_dh.
_R_BASE_T = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=float,
)


def _sensor_qos() -> QoSProfile:
    """腕位 / 关节 / 指令：只留最新一帧，避免 IK 跟不上时把旧样本排队。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


class AstralTeleopArmNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_arm_teleop_arm")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("solver_type", "geometric")
        self.declare_parameter("urdf_path", "")  # empty → astral_robot.pin.urdf
        self.declare_parameter("ik_max_iter", 20)
        self.declare_parameter("ik_tol", 1e-8)
        self.declare_parameter("ik_w_pos", 1.0)
        self.declare_parameter("ik_w_ori", 0.3)
        self.declare_parameter("ik_w_reg", 1e-4)
        # Joint4 URDF upper=0 is fully stretched (elbow singularity).
        # <0 caps IK/safety below that; >=0 keeps the URDF limit.
        self.declare_parameter("ik_q4_max", -0.45)
        self.declare_parameter("ik_w_limit", 0.12)
        # Per-solve joint step box around q_prev (rad). 0 = off. Blocks IK branch jumps.
        self.declare_parameter("ik_dq_max", 0.0)
        # Pull toward init_pose when redundant. 0 = off.
        self.declare_parameter("ik_w_pref", 0.0)
        # One-sided elbow fold: penalize q4 straighter than ik_q4_fold.
        self.declare_parameter("ik_w_fold", 0.015)
        self.declare_parameter("ik_q4_fold", -1.20)
        self.declare_parameter(
            "vr_to_arm_rot",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("pos_smoothing", 0.8)
        self.declare_parameter("rot_smoothing", 0.8)
        self.declare_parameter("motion_scale", 0.65)
        self.declare_parameter("flip_pitch", False)
        self.declare_parameter("tcp_offset", [0.0] * 6)
        self.declare_parameter(
            "solver_tip_to_tcp_transform",
            [1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("use_solver_tip_to_tcp_transform", False)
        self.declare_parameter("coordinate_contract_version", 0)
        self.declare_parameter("kinematics_hash", "")
        self.declare_parameter("solver_base_frame", "")
        self.declare_parameter("solver_tip_frame", "")
        self.declare_parameter("tool_frame", "")
        self.declare_parameter("solver_adapter_id", "astral_geometric_poe_link7_v1")
        self.declare_parameter("max_joint_vel", 4.0)
        self.declare_parameter("workspace_radius", 0.0)
        # Radial soft wall at the elbow-straight singularity (geometric
        # solver only): the wrist target is clamped to l_se + l_ew -
        # reach_margin from the shoulder center. At full extension the
        # elbow circle degenerates and q4 hits its limit, so without the
        # clamp IK returns None and the arm visibly catches. <=0 disables.
        self.declare_parameter("reach_margin", 0.01)
        # Watchdog: disarm when VR data is older than this. 1.5s tolerates
        # brief VR link jitter; while teleop keeps publishing the frozen
        # target the driver keeps tracking it, and only after teleop stops
        # does the driver's own command_timeout_s (1.5) engage the hold.
        # <= 0 disables the watchdog (not recommended).
        self.declare_parameter("data_timeout", 1.5)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("require_clench_to_start", False)
        self.declare_parameter("auto_arm_on_start", True)
        self.declare_parameter(
            "require_start_signal",
            False,
        )
        self.declare_parameter("use_joint_state_seed", True)
        # HOME park 的反馈只做"跟随便用"（不每拍从实测位姿迈步——那会把平滑
        # 位置控制变成"等实测挪一步才挪一步"的阶梯，实机表现 = 一卡一卡+慢）。
        # park 与启动 init 一样纯开环匀速推进；若实测落后指令超过
        # homing_follow_tol（电机失能/堵转没在跟随），冻结轨迹不超前跑，
        # 等实测追近后自动继续——领先被钳在 tol 内，恢复时不会猛扑。
        # 启动 init 归位不受此守卫影响（保持改前纯开环）。
        self.declare_parameter("homing_track_state", True)
        self.declare_parameter("homing_follow_tol", 0.25)  # rad；正常跟随稳态滞后远小于此
        # park 途经点"实体到位门限"：命令（q_cmd 纯开环）到途经点 ≠ 实体到。
        # 命令一到点立即反向会把实体"切角"切在 waypoint 之前（HOME 出了但没到
        # init_waypoints 就掉头去零）。推进下一段前：命令钉在途经点重发，等实测
        # 进入 homing_via_tol 且**连续稳定 homing_via_settle_s**（实体真的停下、
        # 到位）才放行；homing_via_hold_s 是总等待上限（防实测长期不到位卡死
        # 归零），实测流不可用时按纯命令驻留 settle 时长后放行。
        self.declare_parameter("homing_via_tol", 0.12)      # rad
        self.declare_parameter("homing_via_settle_s", 0.5)  # s；实测进入 tol 后需稳定时长
        self.declare_parameter("homing_via_hold_s", 2.0)    # s；途经点总等待上限
        # 工作位路径必须确认实体到达每个途经点和最终目标；超时在当前点
        # 停止，不得像 HOME 的兼容回退那样越过未到达的安全途经点。
        self.declare_parameter("homing_init_arrive_timeout", 12.0)  # s per target
        self.declare_parameter(
            "init_pose", [0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00]
        )
        # Flat 7*N joint-space via points (hardware convention), visited in
        # order before init_pose. Default [0.0] is a typed DOUBLE_ARRAY
        # placeholder (empty [] would be inferred as BYTE_ARRAY in rclpy).
        # Length < 7 means "no via points".
        self.declare_parameter(
            "init_waypoints",
            [0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )
        self.declare_parameter("move_to_init_pose", True)
        self.declare_parameter("init_speed_percent", 10)  # of max_joint_vel
        self.declare_parameter("init_arrive_tol", 0.05)  # rad
        self.declare_parameter("init_timeout", 15.0)
        self.declare_parameter("print_latency", True)
        self.declare_parameter("latency_print_interval", 2.0)
        # 遥操诊断 JSONL 日志（kind=loop/wrist/state/body/metrics 记录）。
        # 空串 = 关闭（零开销）；非空 = 行缓冲追加写，写失败不影响控制流。
        self.declare_parameter("teleop_log_file", "")
        # 电机指令最小步长地板（mrad/拍，0=关）：电机死区 ~1 mrad，慢速时
        # geometric 精确解把指令压到死区下 → 电机不执行 → 一顿一顿。
        # >0 时慢速运动（VR 速度 > cmd_deadband_min_vel）中把 0<|dq|<floor 的
        # 关节步抬到 floor（方向保持），让电机每拍都有足够的步长去跟。
        # 副作用：慢速时关节带恒定微动（轻微"蠕"），手停时应关闭（min_vel 门控）。
        self.declare_parameter("cmd_deadband_mrad", 0.0)
        self.declare_parameter("cmd_deadband_min_vel", 10.0)  # mm/s
        self.declare_parameter("publish_tune", True)
        # Human arm-angle prior from Quest body tracking (geometric solver
        # only): body_joints {side}-arm-upper/-lower give the human upper-arm
        # direction -> psi_ref, so the robot elbow plane follows the human's.
        # Falls back to pure continuity when the body stream is stale.
        self.declare_parameter("use_human_elbow", True)
        self.declare_parameter("human_elbow_mode", "soft")
        self.declare_parameter("human_elbow_weight", 2.0)
        self.declare_parameter("human_elbow_timeout", 0.3)
        self.declare_parameter("human_elbow_smoothing_tau", 0.15)
        # HITL re-anchor 臂角保持（方案B）：reanchor 后肘保持接管时刻的臂角，直到
        # 操作者手臂方向相对 reanchor 时刻变化超过 reanchor_elbow_release_thresh
        # 才切回人肘硬跟随——避免"接管后肘自行摆动到操作者手臂角度"（真机实测
        # 方案A 的 EMA 过渡被感知为"卡一下/臂自己在动"，0.07-0.42 rad/1s）。
        # <=0 关闭保持（回方案A 的即时 EMA 过渡）。
        self.declare_parameter("reanchor_elbow_hold", True)
        self.declare_parameter("reanchor_elbow_release_thresh", 0.2)
        # Straightness gate: when the human arm is nearly straight (elbow
        # within ~sin*upper-arm-length of the shoulder-wrist line), psi is
        # unobservable and IOBT bias would swivel the robot elbow to a
        # garbage angle. 0.15 ~= elbow 4 cm off the line on a 28 cm upper
        # arm; normal bent-arm gestures have sin >= 0.5.
        self.declare_parameter("human_elbow_min_sin", 0.15)
        # 逃逸迟滞（仅 geometric）：局部窗连续空 esc 帧才全局逃逸，逃逸后
        # home 窗连续可行 ret 帧才返回。roll 大角度腕限位边界闪空时防肩抽。
        self.declare_parameter("ik_escape_after_frames", 4)
        self.declare_parameter("ik_return_after_frames", 20)

        self.side = str(self.get_parameter("arm_side").value).lower()
        if self.side not in ("left", "right"):
            raise ValueError("arm_side must be left|right")
        self.names = _LEFT_NAMES if self.side == "left" else _RIGHT_NAMES
        self.coordinate_contract_version = int(self.get_parameter("coordinate_contract_version").value)
        self.kinematics_hash = str(self.get_parameter("kinematics_hash").value)
        self.solver_base_frame = str(self.get_parameter("solver_base_frame").value).strip() or f"{self.side}_base_link"
        self.solver_tip_frame = str(self.get_parameter("solver_tip_frame").value).strip() or f"{self.side}_link7"
        self.tool_frame = str(self.get_parameter("tool_frame").value).strip() or self.solver_tip_frame
        self.solver_adapter_id = str(self.get_parameter("solver_adapter_id").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.data_timeout = float(self.get_parameter("data_timeout").value)
        rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / max(1.0, rate)

        st = str(self.get_parameter("solver_type").value).strip().lower()
        # The joint-sign flip is only meaningful for the analytic DH solver, which
        # works in the "flipped" (alpha=+90) convention. The URDF numerical solver
        # reads the original-convention URDF and already returns hardware-convention q.
        self._flip_needed = st in ("analytic_dh", "analytic", "dh")
        self._urdf_path = str(self.get_parameter("urdf_path").value)
        self._vr_to_arm_yaml = np.asarray(
            self.get_parameter("vr_to_arm_rot").value, dtype=float
        ).reshape(3, 3)
        self._vr_to_arm_yaml = validate_rotation(self._vr_to_arm_yaml)
        self.ik = make_single_arm_ik(
            self.side,
            st,
            urdf_path=self._urdf_path,
            ik_max_iter=int(self.get_parameter("ik_max_iter").value),
            ik_tol=float(self.get_parameter("ik_tol").value),
            ik_w_pos=float(self.get_parameter("ik_w_pos").value),
            ik_w_ori=float(self.get_parameter("ik_w_ori").value),
            ik_w_reg=float(self.get_parameter("ik_w_reg").value),
            ik_q4_max=float(self.get_parameter("ik_q4_max").value),
            ik_w_limit=float(self.get_parameter("ik_w_limit").value),
            ik_dq_max=float(self.get_parameter("ik_dq_max").value),
            ik_w_pref=float(self.get_parameter("ik_w_pref").value),
            ik_w_fold=float(self.get_parameter("ik_w_fold").value),
            ik_q4_fold=float(self.get_parameter("ik_q4_fold").value),
        )
        self.reach_margin = float(self.get_parameter("reach_margin").value)
        self._use_human_elbow = bool(self.get_parameter("use_human_elbow").value)
        # "soft": psi_ref is a weighted prior blended with continuity (default,
        # w_psi_ref vs w_vel). "hard": when the human psi is fresh and feasible
        # the arm resolves exactly at it (GeometricIKSolver.solve_hard) and the
        # elbow strictly follows the human; infeasible/stale psi falls back to
        # the soft path. Only meaningful for the geometric solver.
        self._human_elbow_mode = str(
            self.get_parameter("human_elbow_mode").value
        ).strip().lower()
        if self._human_elbow_mode not in ("soft", "hard"):
            raise ValueError(
                f"human_elbow_mode must be soft|hard, got {self._human_elbow_mode!r}"
            )
        self._human_elbow_weight = float(self.get_parameter("human_elbow_weight").value)
        self._human_elbow_timeout = float(
            self.get_parameter("human_elbow_timeout").value
        )
        self._human_elbow_tau = float(
            self.get_parameter("human_elbow_smoothing_tau").value
        )
        self._human_elbow_min_sin = float(
            self.get_parameter("human_elbow_min_sin").value
        )
        self._apply_human_elbow_cfg()
        # Body-tracking prior state (Quest IOBT operator hips frame, not robot body).
        self._body_names: Optional[List[str]] = None
        self._elbow_dir_vr: Optional[np.ndarray] = None  # EMA-smoothed unit dir
        self._elbow_dir_t: float = 0.0
        self._body_subs = None
        # 方案B：reanchor 后保持臂角直到操作者手臂方向变化超阈值。
        self._reanchor_elbow_hold = bool(
            self.get_parameter("reanchor_elbow_hold").value
        )
        self._reanchor_elbow_release_thresh = float(
            self.get_parameter("reanchor_elbow_release_thresh").value
        )
        self._elbow_held = False
        self._elbow_reanchor_dir: Optional[np.ndarray] = None
        # Arm-angle escape watcher state (geometric solver; see main loop).
        self._esc_prev_active = False
        init_q_old = np.asarray(
            self.get_parameter("init_pose").value, dtype=float
        ).reshape(7)  # hardware/original convention (config)
        init_q = self._flip_q(init_q_old)  # DH/flipped convention (feed IK)
        init_q = np.clip(
            init_q, self.ik.lower_limits + 0.02, self.ik.upper_limits - 0.02
        )
        if hasattr(self.ik, "set_lm_params"):
            self.ik.set_lm_params(q_pref=init_q)
        self.ik.sync_state(init_q)
        tcp = np.asarray(self.get_parameter("tcp_offset").value, dtype=float)
        if tcp.shape != (6,) or not np.isfinite(tcp).all():
            raise ValueError("tcp_offset must contain [x,y,z,roll,pitch,yaw]")
        tool_transform = np.asarray(
            self.get_parameter("solver_tip_to_tcp_transform").value, dtype=float
        )
        if bool(self.get_parameter("use_solver_tip_to_tcp_transform").value):
            if tool_transform.size != 16:
                raise ValueError("solver_tip_to_tcp_transform must contain 16 row-major values")
            self._T_solver_tip_to_tcp = validate_transform(tool_transform.reshape(4, 4))
        else:
            self._T_solver_tip_to_tcp = np.eye(4)
            self._T_solver_tip_to_tcp[:3, :3] = Rotation.from_euler("xyz", tcp[3:]).as_matrix()
            self._T_solver_tip_to_tcp[:3, 3] = tcp[:3]
        T0 = self._fk_tcp(init_q)
        _ = self.ik.solve(tcp_to_solver_tip(T0, self._T_solver_tip_to_tcp))
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()
        self.q_cmd = self._flip_q(init_q)  # back to hardware convention
        self._init_q_hw = self.q_cmd.copy()
        self._homing_path = self._parse_init_waypoints() + [self._init_q_hw.copy()]
        self._homing_i = 0
        self.state_q = self.q_cmd.copy()
        self._got_state = False
        self._homing = bool(self.get_parameter("move_to_init_pose").value) and (
            not self.dry_run
        )
        vmax = float(self.get_parameter("max_joint_vel").value)
        pct = max(1.0, float(self.get_parameter("init_speed_percent").value))
        self._init_joint_vel = vmax * (pct / 100.0)
        self._init_arrive_tol = float(self.get_parameter("init_arrive_tol").value)
        self._init_timeout = float(self.get_parameter("init_timeout").value)
        self._homing_t0 = 0.0
        self._homing_last_log = 0.0
        self._homing_last_base = None
        self._homing_started = False
        self._homing_seeded = False
        # "init" = 启动归位 init_waypoints→init_pose；"park" = HOME 按钮
        # init_pose→init_waypoints→零位。决定 _finish_homing 的终态目标。
        self._homing_mode = "init"
        self._track_homing_state = bool(
            self.get_parameter("homing_track_state").value
        )
        self._homing_follow_tol = float(
            self.get_parameter("homing_follow_tol").value
        )
        self._park_frozen = False
        self._homing_via_tol = float(self.get_parameter("homing_via_tol").value)
        self._homing_via_settle_s = float(
            self.get_parameter("homing_via_settle_s").value
        )
        self._homing_via_hold_s = float(self.get_parameter("homing_via_hold_s").value)
        self._homing_init_arrive_timeout = float(
            self.get_parameter("homing_init_arrive_timeout").value
        )
        self._via_wait_t0 = None  # 途经点等待计时起点（见 _homing_tick）
        self._via_in_tol_since = None  # 实测首次进入 tol 的时刻（连续计时）
        # 节点是否已"到达初始工作位"（init 归位 arrived）或仍锚在启动位。
        # 启动不再自动归位（move_to_init_pose=false 由 web「工作位」触发）后，
        # 臂可能从任意位直接 /teleop/start——此时须先把机器人原点重锚到当前
        # 实测，否则首帧会向启动 init 锚点跳变（见 _start_teleop 守卫）。
        self._at_init_pose = False

        R = self._vr_to_arm_yaml.copy()
        if self._flip_needed:
            # analytic DH works in the clean base (joint1 axis = +Z); map the VR
            # delta from the original torso base into the clean base frame.
            R = _R_BASE_T @ R
        self._require_start = bool(self.get_parameter("require_start_signal").value)
        self.pose = PoseProcessor(
            vr_to_arm_rot=R,
            pos_smoothing=float(self.get_parameter("pos_smoothing").value),
            rot_smoothing=float(self.get_parameter("rot_smoothing").value),
            motion_scale=float(self.get_parameter("motion_scale").value),
            flip_pitch=bool(self.get_parameter("flip_pitch").value),
            auto_calibrate=not self._require_start,
        )
        self.safety = SafetyFilter(
            joint_lower_limits=self.ik.lower_limits,
            joint_upper_limits=self.ik.upper_limits,
            max_joint_vel=float(self.get_parameter("max_joint_vel").value),
            workspace_radius=float(self.get_parameter("workspace_radius").value),
        )
        self.safety.set_initial_state(init_q, self.robot_init_pos)

        qos = _sensor_qos()
        self.cmd_pub = self.create_publisher(
            JointState, f"/{self.side}_arm/joint_commands", qos
        )
        self._publish_tune = bool(self.get_parameter("publish_tune").value)
        self._tune_pubs = {}
        if self._publish_tune:
            tune_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
            )
            prefix = f"/teleop/{self.side}/tune"
            self._tune_pubs = {
                "vr": self.create_publisher(PoseStamped, f"{prefix}/ee_vr", tune_qos),
                "filt": self.create_publisher(
                    PoseStamped, f"{prefix}/ee_filt", tune_qos
                ),
                "cmd": self.create_publisher(PoseStamped, f"{prefix}/ee_cmd", tune_qos),
                "xyz": self.create_publisher(
                    Float64MultiArray, f"{prefix}/xyz", tune_qos
                ),
            }
            self.get_logger().info(
                f"tune topics: /teleop/{self.side}/tune/ee_{{vr,filt,cmd}} + xyz"
            )
        self.create_subscription(
            PoseStamped, f"quest3/{self.side}_wrist_pose", self._on_wrist, qos
        )
        self._ensure_body_subs()
        if self._homing or bool(self.get_parameter("use_joint_state_seed").value):
            self.create_subscription(
                JointState,
                f"/{self.side}_arm/joint_states",
                self._on_state,
                qos,
            )

        require = bool(self.get_parameter("require_clench_to_start").value)
        auto = bool(self.get_parameter("auto_arm_on_start").value)
        if self._require_start:
            # Wait for /teleop/start (or ~/start): do not arm or auto-zero on the
            # first VR pose. The user triggers start once their hand is placed.
            self._armed = False
        else:
            self._armed = (not require) or auto
        # Why teleop was last disarmed: None = never/freshly started,
        # "operator" = web/CLI pause (resume via /teleop/armed allowed),
        # "fault" = VR watchdog (must re-start to re-capture vr_init).
        self._disarm_reason: Optional[str] = None
        self.create_subscription(Bool, "/teleop/armed", self._on_armed, 10)
        self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)
        self.create_subscription(Bool, "/teleop/start", self._on_start, 10)
        self.create_service(Trigger, "~/start", self._start_srv)
        # HITL takeover: re-anchor the robot origin to the current measured
        # joint state AND the VR zero to the current hand pose, then arm.
        self.create_service(Trigger, "~/reanchor", self._reanchor_srv)
        self.create_service(Trigger, "~/prepare_reanchor", self._prepare_reanchor_srv)
        self._policy_lease = PolicyLease()
        self._prepared_token = None
        self._policy_lease.bind(self, f'{self.side}_arm', self._on_policy_state)
        # HOME / park-to-zero: init_pose → init_waypoints → 零位（双臂同启用
        # /teleop/home 一次性信号，单臂用 ~/home 服务）。VOLATILE 语义同
        # /teleop/start：晚启动节点不得被历史 HOME 信号误触发。
        self.create_subscription(Bool, "/teleop/home", self._on_home, 10)
        self.create_service(Trigger, "~/home", self._home_srv)
        # 工作位 / go-to-init：启动不再自动归位（web「工作位」按钮手动触发），
        # 从当前位姿沿 init_waypoints → init_pose 走启动同款 init 轨迹，到点
        # 保持并锚 VR 原点。入口与 HOME 对称：/teleop/init 全局一次性信号 +
        # ~/init 单臂服务。VOLATILE：晚启动节点不得被历史信号误触发。
        self.create_subscription(Bool, "/teleop/init", self._on_init, 10)
        self.create_service(Trigger, "~/init", self._init_srv)
        # 段间回位直达：/teleop/init_direct（左 X / 数采卡片「段间回位」）→
        # **不经 init_waypoints**、直接关节空间直线插补到 init_pose（数采段间
        # 快速回工作位）。入口与 /teleop/init 对称（全局一次性 + ~/init_direct
        # 单臂服务），VOLATILE 语义相同。
        self.create_subscription(Bool, "/teleop/init_direct", self._on_init_direct, 10)
        self.create_service(Trigger, "~/init_direct", self._init_direct_srv)

        self._state_t: Optional[float] = None
        self._last_vr_t = 0.0
        self._last_vr_stamp = None
        self._prev_t = time.monotonic()
        self._print_latency = bool(self.get_parameter("print_latency").value)
        self._lat = LatencyMeter(
            float(self.get_parameter("latency_print_interval").value)
        )
        self._tlog = TeleopJsonlLog(
            str(self.get_parameter("teleop_log_file").value or "")
        )
        # 跟手/慢速跟踪指标状态：上一拍 vr 目标 + 窗口聚合缓存（供 kind=metrics 汇总）。
        self._prev_vr_tgt: Optional[np.ndarray] = None
        self._track_buf = deque(maxlen=400)  # ~4s @100Hz；metrics 窗口 2s
        # 电机指令最小步长地板（见参数声明；热改走 _on_set_parameters）
        self._cmd_db_mrad = float(self.get_parameter("cmd_deadband_mrad").value)
        self._cmd_db_min_vel = float(self.get_parameter("cmd_deadband_min_vel").value)
        if self._tlog.enabled:
            self.get_logger().info(
                f"[{self.side}] teleop jsonl log -> {self._tlog.path}"
            )
        self._runtime_status_pub = self.create_publisher(
            String, f"/teleop/arm_status/{self.side}", 10
        )
        self.create_timer(0.2, self._publish_runtime_status)
        self.create_timer(self.dt, self._loop)
        solver_name = getattr(self.ik, "method_name", type(self.ik).__name__)
        n_via = max(0, len(self._homing_path) - 1)
        home_msg = (
            f"homing {n_via} via + init at {self._init_joint_vel:.2f} rad/s "
            f"({self.get_parameter('init_speed_percent').value}%)"
            if self._homing
            else "homing off"
        )
        ee0_r = float(np.linalg.norm(self.robot_init_pos))
        ws_r = float(self.safety.workspace_radius)
        self.get_logger().info(
            f"Astral arm teleop: {self.side} solver={solver_name} "
            f"EE0={np.round(self.robot_init_pos, 3).tolist()} "
            f"r={ee0_r:.3f}m ws_r={ws_r:.3f}m "
            f"scale={self.pose.motion_scale} dry_run={self.dry_run} {home_msg}"
        )
        if self._require_start:
            self.get_logger().warn(
                f"[{self.side}] require_start_signal=true: waiting for "
                "/teleop/start (Bool true) or ~/start service. Place your hand "
                "at the initial pose, then send start to capture vr_init and arm."
            )
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, params):
        result = SetParametersResult(successful=True)
        solver_req = None
        lm = {}
        try:
            for p in params:
                name = p.name
                if name == "pos_smoothing":
                    self.pose.set_pos_smoothing(float(p.value))
                elif name == "rot_smoothing":
                    self.pose.set_rot_smoothing(float(p.value))
                elif name == "motion_scale":
                    self.pose.set_motion_scale(float(p.value))
                elif name == "flip_pitch":
                    self.pose.set_flip_pitch(bool(p.value))
                elif name == "max_joint_vel":
                    self.safety.max_joint_vel = float(p.value)
                elif name == "workspace_radius":
                    self.safety.workspace_radius = float(p.value)
                elif name == "reach_margin":
                    self.reach_margin = float(p.value)
                elif name == "cmd_deadband_mrad":
                    self._cmd_db_mrad = float(p.value)
                elif name == "cmd_deadband_min_vel":
                    self._cmd_db_min_vel = float(p.value)
                elif name == "data_timeout":
                    self.data_timeout = float(p.value)
                elif name == "dry_run":
                    self.dry_run = bool(p.value)
                elif name == "print_latency":
                    self._print_latency = bool(p.value)
                elif name == "solver_type":
                    solver_req = str(p.value).strip().lower()
                elif name == "ik_max_iter":
                    lm["max_iter"] = int(p.value)
                elif name == "ik_tol":
                    lm["tol"] = float(p.value)
                elif name == "ik_w_pos":
                    lm["w_pos"] = float(p.value)
                elif name == "ik_w_ori":
                    lm["w_ori"] = float(p.value)
                elif name == "ik_w_reg":
                    lm["w_reg"] = float(p.value)
                elif name == "ik_dq_max":
                    lm["dq_max"] = float(p.value)
                elif name == "ik_w_pref":
                    lm["w_pref"] = float(p.value)
                elif name == "ik_w_fold":
                    lm["w_fold"] = float(p.value)
                elif name == "ik_q4_fold":
                    lm["q4_fold"] = float(p.value)
                elif name == "ik_q4_max":
                    lm["q4_max"] = float(p.value)
                elif name == "ik_w_limit":
                    lm["w_limit"] = float(p.value)
                elif name == "use_human_elbow":
                    self._use_human_elbow = bool(p.value)
                    self._ensure_body_subs()
                elif name == "human_elbow_mode":
                    mode = str(p.value).strip().lower()
                    if mode not in ("soft", "hard"):
                        self.get_logger().warn(
                            f"[{self.side}] human_elbow_mode must be soft|hard, "
                            f"ignoring {mode!r}"
                        )
                    else:
                        self._human_elbow_mode = mode
                elif name == "human_elbow_weight":
                    self._human_elbow_weight = float(p.value)
                    self._apply_human_elbow_cfg()
                elif name == "human_elbow_timeout":
                    self._human_elbow_timeout = float(p.value)
                elif name == "human_elbow_smoothing_tau":
                    self._human_elbow_tau = float(p.value)
                elif name == "human_elbow_min_sin":
                    self._human_elbow_min_sin = float(p.value)
                elif name in ("ik_escape_after_frames", "ik_return_after_frames"):
                    cont = getattr(self.ik, "continuity", None)
                    if cont is not None:
                        attr = (
                            "escape_after_frames"
                            if name == "ik_escape_after_frames"
                            else "return_after_frames"
                        )
                        setattr(cont, attr, max(1, int(p.value)))
                elif name in (
                    "arm_side",
                    "urdf_path",
                    "init_pose",
                    "init_waypoints",
                    "vr_to_arm_rot",
                    "tcp_offset",
                    "control_rate",
                    "move_to_init_pose",
                    "init_speed_percent",
                    "teleop_log_file",
                ):
                    result.successful = False
                    result.reason = f"{name} cannot be changed at runtime"
                    return result
            if solver_req is not None:
                self._rebuild_solver(solver_req)
            if lm:
                self._apply_ik_lm(lm)
        except Exception as exc:  # noqa: BLE001
            result.successful = False
            result.reason = str(exc)
            return result
        return result

    def _apply_ik_lm(self, lm: dict) -> None:
        ik = self.ik
        if hasattr(ik, "set_lm_params"):
            ik.set_lm_params(**lm)
        if hasattr(ik, "lower_limits") and hasattr(ik, "upper_limits"):
            self.safety.joint_lower = np.asarray(ik.lower_limits, dtype=float).copy()
            self.safety.joint_upper = np.asarray(ik.upper_limits, dtype=float).copy()
        # DH closed-form: LM weights do not apply.

    def _rebuild_solver(self, solver_type: str) -> None:
        st = solver_type.strip().lower()
        if st in ("analytic", "dh"):
            st = "analytic_dh"
        if st in ("urdf", "numerical"):
            st = "urdf_numerical"
        if st in ("swe", "poe", "geometric_arm_angle"):
            st = "geometric"
        if st not in ("analytic_dh", "geometric", "urdf_numerical"):
            raise ValueError(
                f"solver_type must be analytic_dh|geometric|urdf_numerical, got {st}"
            )
        q_hw = np.asarray(self.q_cmd, dtype=float).reshape(7)
        self._flip_needed = st == "analytic_dh"
        self.ik = make_single_arm_ik(
            self.side,
            st,
            urdf_path=self._urdf_path,
            ik_max_iter=int(self.get_parameter("ik_max_iter").value),
            ik_tol=float(self.get_parameter("ik_tol").value),
            ik_w_pos=float(self.get_parameter("ik_w_pos").value),
            ik_w_ori=float(self.get_parameter("ik_w_ori").value),
            ik_w_reg=float(self.get_parameter("ik_w_reg").value),
            ik_q4_max=float(self.get_parameter("ik_q4_max").value),
            ik_w_limit=float(self.get_parameter("ik_w_limit").value),
            ik_dq_max=float(self.get_parameter("ik_dq_max").value),
            ik_w_pref=float(self.get_parameter("ik_w_pref").value),
            ik_w_fold=float(self.get_parameter("ik_w_fold").value),
            ik_q4_fold=float(self.get_parameter("ik_q4_fold").value),
            escape_after_frames=int(
                self.get_parameter("ik_escape_after_frames").value
            ),
            return_after_frames=int(
                self.get_parameter("ik_return_after_frames").value
            ),
        )
        self._apply_human_elbow_cfg()
        if self._use_human_elbow and not hasattr(self.ik, "arm_angle_from_elbow_dir"):
            self.get_logger().warn(
                f"[{self.side}] human elbow prior requires the geometric solver; "
                f"disabled for {st}"
            )
        R = self._vr_to_arm_yaml.copy()
        if self._flip_needed:
            R = _R_BASE_T @ R
        self.pose.R_vr_to_arm = R
        self.pose.R_world_to_base = R
        q_ik = self._flip_q(q_hw)
        q_ik = np.clip(
            q_ik, self.ik.lower_limits + 0.02, self.ik.upper_limits - 0.02
        )
        if hasattr(self.ik, "set_lm_params"):
            self.ik.set_lm_params(q_pref=self._flip_q(self._init_q_hw))
        self.ik.sync_state(q_ik)
        T0 = self._fk_tcp(q_ik)
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()
        self.q_cmd = q_hw
        self.safety.joint_lower = np.asarray(self.ik.lower_limits, dtype=float).copy()
        self.safety.joint_upper = np.asarray(self.ik.upper_limits, dtype=float).copy()
        self.safety.set_initial_state(q_ik, self.robot_init_pos)
        if self.pose.vr_current_pos is not None and self.pose.vr_current_rot is not None:
            self.pose.set_vr_zero_point(
                self.pose.vr_current_pos, self.pose.vr_current_rot
            )
        else:
            self.pose.reset()
        name = getattr(self.ik, "method_name", type(self.ik).__name__)
        self.get_logger().info(
            f"switched solver → {name}; VR zero reset, hold pose then move"
        )

    def _flip_q(self, q):
        """Map joint angles between the DH (flipped) and hardware (original) convention.

        Self-inverse: the flipped joints (per _JOINT_FLIP) have their sign negated,
        so applying it twice returns the original vector. For the URDF numerical
        solver this is a no-op (it already works in the hardware convention).
        """
        q = np.asarray(q, dtype=float).reshape(7)
        if not self._flip_needed:
            return q
        out = q.copy()
        for i, f in enumerate(_JOINT_FLIP[self.side]):
            if f:
                out[i] = -out[i]
        return out

    def _fk_tcp(self, q_solver: np.ndarray) -> np.ndarray:
        """Return the configured TCP pose from the active solver-tip FK."""
        return solver_tip_to_tcp(
            self.ik.fk(np.asarray(q_solver, dtype=float)), self._T_solver_tip_to_tcp
        )

    def _parse_init_waypoints(self) -> list:
        """Hardware-convention via points from the flat ``init_waypoints`` array."""
        raw = np.asarray(
            self.get_parameter("init_waypoints").value, dtype=float
        ).ravel()
        if raw.size < 7:
            return []
        if raw.size % 7 != 0:
            raise ValueError(
                f"init_waypoints length {raw.size} is not a multiple of 7"
            )
        out = []
        lo = self.ik.lower_limits + 0.02
        hi = self.ik.upper_limits - 0.02
        for row in raw.reshape(-1, 7):
            q_ik = np.clip(self._flip_q(row), lo, hi)
            q_hw = self._flip_q(q_ik)
            if not np.allclose(q_hw, row, atol=1e-3):
                self.get_logger().warn(
                    f"[{self.side}] init_waypoints clipped to joint limits: "
                    f"{np.round(row, 3).tolist()} → {np.round(q_hw, 3).tolist()}"
                )
            out.append(q_hw)
        return out

    def _homing_target(self) -> np.ndarray:
        return self._homing_path[self._homing_i]

    def _tlog_event(self, event: str, **fields) -> None:
        """Write a ``kind=event`` lifecycle record (armed/disarm/reanchor/start/fault).

        Zero-overhead when the JSONL log is off (the disabled writer is a no-op).
        Lets a single teleop jsonl timeline reconstruct the full HITL takeover
        sequence (who armed/disarmed/re-anchored the arm and when).
        """
        tlog = getattr(self, "_tlog", None)
        if tlog is not None and tlog.enabled:
            tlog.write(teleop_event_record(
                event, self.side,
                coordinate_contract_version=self.coordinate_contract_version,
                kinematics_hash=self.kinematics_hash,
                solver_base_frame=self.solver_base_frame,
                solver_tip_frame=self.solver_tip_frame,
                solver_adapter_id=self.solver_adapter_id,
                tool_frame=self.tool_frame,
                **fields,
            ))

    def _on_armed(self, msg: Bool) -> None:
        # Only Bool(true) arms; a stray/default Bool(false) must not arm.
        if not msg.data:
            return
        if not self._manual_allowed():
            return
        if not self.pose.is_calibrated:
            # Also covers the web monitor's latched /teleop/armed residue
            # reaching a freshly started (uncalibrated) node.
            self.get_logger().warn(
                f"[{self.side}] armed ignored: not calibrated — "
                "send /teleop/start to capture vr_init first"
            )
            return
        if self._disarm_reason == "fault":
            self.get_logger().warn(
                f"[{self.side}] armed ignored: disarmed by VR watchdog — "
                "re-send /teleop/start to re-calibrate and resume"
            )
            return
        self._armed = True
        self._disarm_reason = None
        self._tlog_event("armed")
        self.get_logger().info(f"[{self.side}] armed via /teleop/armed")

    def _on_disarm(self, msg: Bool) -> None:
        # /teleop/disarm is a *level* signal: only Bool(true) disarms. Several
        # owners publish to it (web pause = True, morla_policy_inference opens
        # the gate with False on IDLE/HUMAN), so a False must not disarm an
        # armed teleop that a re-anchor / ~/start just armed.
        if not msg.data:
            return
        self._armed = False
        # A fault reason sticks until a proper /teleop/start; an operator
        # pause must not downgrade it (pause→resume would bypass re-centering).
        if self._disarm_reason != "fault":
            self._disarm_reason = "operator"
        was_homing = bool(self._homing)
        if was_homing:
            # Operator disarm while a homing/park is running must cancel it:
            # otherwise the node keeps streaming the stale joint trajectory
            # (e.g. the slow startup init move), which would override a later
            # driver ~/home / ~/ready zero once motion mode returns to POSITION
            # (症状: 阻尼释放后点一键就绪，臂回到阻尼前位姿). The arm simply
            # holds its last commanded pose; a later /teleop/start re-captures.
            self._homing = False
            self._homing_started = False
            self.get_logger().warn(
                f"[{self.side}] homing cancelled by disarm — hold current pose"
            )
        self._tlog_event(
            "disarm", disarm_reason=self._disarm_reason, homing_cancelled=was_homing
        )

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            self._start_teleop()

    def _start_srv(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, message = self._start_teleop()
        resp.success = ok
        resp.message = message
        return resp

    def _start_teleop(self):
        """Capture vr_init from the current VR pose and arm teleop.

        Triggered by /teleop/start (global, both arms) or ~/start (per arm).
        Re-sending re-captures the zero from the current pose (re-center).
        """
        if not self._manual_allowed():
            return False, 'policy owns output; use takeover'
        if self._homing:
            msg = "homing in progress; wait for init pose, then send start"
            self.get_logger().warn(f"[{self.side}] start: {msg}")
            return False, msg
        if self.pose.vr_current_pos is None:
            msg = "no VR wrist pose yet; start Quest stream, place hand, then send start"
            self.get_logger().warn(f"[{self.side}] start: {msg}")
            return False, msg
        if not self.pose.calibrate_from_current():
            msg = "calibrate failed (no VR pose)"
            self.get_logger().warn(f"[{self.side}] start: {msg}")
            return False, msg
        reanchored = not self._at_init_pose
        if reanchored:
            # 启动不再自动归位（工作位改由 web「工作位」/~/init 触发）后，臂
            # 可能从任意位姿直接 start：若不重锚，遥操增量目标相对启动位 FK
            # 锚点计算，首帧会整体向旧 init 锚点跳变。把原点重锚到当前实测，
            # 遥操从实际位姿纯增量开始（与 HITL ~/reanchor 同一重锚逻辑）。
            src = self._anchor_origin_to_measured()
            self.get_logger().warn(
                f"[{self.side}] START: 臂不在初始工作位——原点重锚到 {src}，"
                "遥操从当前位姿纯增量开始；建议先用「工作位」移到 init_pose"
            )
        self._armed = True
        self._disarm_reason = None
        self._tlog_event("start", reanchored=reanchored)
        self.get_logger().warn(
            f"[{self.side}] START: vr_init captured from current pose, teleop armed"
        )
        return True, "started"

    def _on_home(self, msg: Bool) -> None:
        # One-shot level guard (mirrors /teleop/start): only Bool(true) triggers.
        if msg.data:
            self._go_home()

    def _home_srv(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self._go_home()
        resp.success = ok
        resp.message = message
        return resp

    def _go_home(self):
        """HOME / park-to-zero：从当前位姿慢速走 init_pose → init_waypoints（**倒序**）
        → 零位（启动 init 路径的反向，硬件约定），到零位后保持（关节目标=0）。

        用于把遥操（或急停恢复后）的臂安全收回零位。动作前先 disarm（若在
        遥操/armed），期间忽略 VR/start；结束后要再遥操需重新 /teleop/start。
        与启动归位共用同一条慢速 joint-space 轨迹机（_homing_*），仅终态与
        轨迹点不同。
        """
        if not self._manual_allowed():
            return False, 'policy owns output; stop policy before HOME'
        if self._homing:
            msg = "homing/park already in progress; wait until it finishes"
            self.get_logger().warn(f"[{self.side}] home: {msg}")
            return False, msg
        self._armed = False
        if self._disarm_reason != "fault":
            self._disarm_reason = "operator"
        self._homing_mode = "park"
        # park 路径 = 启动 init 路径的**反向**：启动按 init_waypoints 正序
        # （从零/低位 抬向 init_pose），收回须按**倒序**逐点退回（先回离
        # init_pose 最近的途经点再逐级放下），否则多途经点时 HOME 走的是
        # 顺向延伸而非原路返回（症状：经过位置与启动移动不成反向关系）。
        # 到达最后一个安全途经点后，先保持其余关节、单独将 J4 (q[3]) 归零，
        # 再回全零。该中间点只属于 HOME，不改变启动/工作位路径。
        park_waypoints = self._parse_init_waypoints()[::-1]
        self._homing_path = [self._init_q_hw.copy()] + park_waypoints
        if park_waypoints:
            j4_zero = park_waypoints[-1].copy()
            j4_zero[3] = 0.0
            if not np.allclose(j4_zero, park_waypoints[-1]):
                self._homing_path.append(j4_zero)
        self._homing_path.append(np.zeros(7, dtype=float))
        self._homing_i = 0
        self._homing_started = False
        self._homing_seeded = False
        self._homing = True
        self._park_frozen = False
        self._via_wait_t0 = None
        self._via_in_tol_since = None
        self._homing_last_log = 0.0
        self._homing_last_base = None
        self.pose.reset()
        self.get_logger().warn(
            f"[{self.side}] HOME: disarm + park to zero via init_pose → "
            f"{len(park_waypoints)} via → J4=0 → zero "
            f"({self._init_joint_vel:.2f} rad/s)"
        )
        return True, "HOME 已启动 (init_pose → init_waypoints → J4归零 → 全零位)"

    def _on_init(self, msg: Bool) -> None:
        # One-shot level guard (mirrors /teleop/home): only Bool(true) triggers.
        if msg.data:
            self._go_init()

    def _init_srv(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self._go_init()
        resp.success = ok
        resp.message = message
        return resp

    def _on_init_direct(self, msg: Bool) -> None:
        # One-shot level guard：段间回位直达（不经 init_waypoints，直接到
        # init_pose）。与 /teleop/init 同契约：只认 Bool(true)。
        if msg.data:
            self._go_init(direct=True)

    def _init_direct_srv(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self._go_init(direct=True)
        resp.success = ok
        resp.message = message
        return resp

    def _go_init(self, direct: bool = False):
        """工作位 / go-to-init：从当前位姿慢速走到 init_pose。

        direct=False（默认，web「工作位」按钮 / /teleop/init / ~/init）：
        沿 init_waypoints 正序 → init_pose，走启动同款 init 轨迹（途经点）。
        direct=True（段间回位，左 X / 数采卡片「段间回位」/ /teleop/init_direct
        / ~/init_direct）：**直接**关节空间直线插补到 init_pose，不经途经点——
        数采段与段之间快速回工作位（真机实测确认直接路径安全）。

        两条路径共用同一 homing 慢速轨迹机（init_speed_percent 限速、关节限位
        裁剪、到点判定相同），仅 _homing_path 不同。动作前先 disarm（若在
        遥操/armed），期间忽略 VR/start；到 init_pose 后保持并把 _at_init_pose
        置位，之后可 /teleop/start 开始遥操。与启动自动归位共用 _homing_*
        机器，不加 park 的途经点门控/冻结守卫（实测没动不能卡住去工作位——
        与启动 init 一致）。
        """
        if not self._manual_allowed():
            return False, 'policy owns output; stop policy before init'
        if self._homing:
            msg = "homing/park already in progress; wait until it finishes"
            self.get_logger().warn(f"[{self.side}] init: {msg}")
            return False, msg
        self._armed = False
        if self._disarm_reason != "fault":
            self._disarm_reason = "operator"
        self._homing_mode = "init"
        if direct:
            # 段间回位直达：单点路径 = init_pose（关节空间直线插补）。
            self._homing_path = [self._init_q_hw.copy()]
        else:
            # 工作位：init_waypoints 正序 → init_pose（与启动自动归位同款）。
            self._homing_path = self._parse_init_waypoints() + [
                self._init_q_hw.copy()
            ]
        self._homing_i = 0
        self._homing_started = False
        self._homing_seeded = False
        self._homing = True
        self._park_frozen = False
        self._via_wait_t0 = None
        self._via_in_tol_since = None
        self._homing_last_log = 0.0
        self._homing_last_base = None
        self.pose.reset()
        n_wp = 0 if direct else len(self._parse_init_waypoints())
        self.get_logger().warn(
            f"[{self.side}] WORKPOS{'（直达）' if direct else ''}: disarm + "
            f"move to init_pose via {n_wp} waypoint(s) "
            f"({self._init_joint_vel:.2f} rad/s)"
        )
        return True, (
            "工作位已启动（直达，不经途径点）" if direct
            else "工作位已启动 (init_waypoints → init_pose)"
        )

    def _anchor_origin_to_measured(self) -> str:
        """把机器人原点（robot_init_pos/rot + ik/safety 初始位）重锚到当前实测。

        返回源描述（"measured joint state" / "last commanded q"）。用于：
        HOME park 后、或启动未自动归位时从任意位姿直接 /teleop/start 的守卫
        ——避免遥操增量目标相对启动位 FK 锚点算，首帧向旧锚点跳变。
        """
        state_fresh = self._got_state and (
            self.data_timeout <= 0.0
            or (
                self._state_t is not None
                and time.monotonic() - self._state_t <= self.data_timeout
            )
        )
        q_hw = np.asarray(
            self.state_q if state_fresh else self.q_cmd, dtype=float
        ).reshape(7)
        q_ik = np.clip(
            self._flip_q(q_hw), self.ik.lower_limits + 0.02, self.ik.upper_limits - 0.02
        )
        self.ik.sync_state(q_ik)
        T0 = self._fk_tcp(q_ik)
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()
        self.q_cmd = q_hw
        self.safety.set_initial_state(q_ik, self.robot_init_pos)
        # HITL/任意位姿 re-anchor 时把**臂角参考**一并重锚到当前配置：人肘
        # EMA 重置为当前上臂方向，首个 solve_hard 解在当前臂角（肘不跳，
        # 否则 human_elbow_mode=hard 会立刻把臂角摆到操作者手臂角度——
        # 实测 reanchor 后 0.7s 内关节重构 0.26-0.32 rad）。
        # 方案B（reanchor_elbow_hold，默认开）：随后**保持**当前臂角，直到
        # 操作者手臂方向相对 reanchor 时刻变化超过阈值才切回人肘跟随——
        # 避免方案A 的 EMA 过渡被感知为"接管后肘自行摆动"（0.07-0.42 rad/1s）。
        try:
            u_se = self.ik.elbow_direction(q_ik)
            self._elbow_dir_vr = self.pose.R_vr_to_arm.T @ u_se
            self._elbow_dir_t = time.monotonic()
            self._elbow_held = self._reanchor_elbow_hold
            self._elbow_reanchor_dir = self._elbow_dir_vr.copy()
        except Exception as exc:  # noqa: BLE001 非 geometric 求解器无该方法（无害）；
            if hasattr(self.ik, "solve_hard"):  # 但 hard 模式缺重锚 = 肘会跳
                self.get_logger().warn(
                    f"[{self.side}] elbow re-anchor failed: {exc}"
                )
        return "measured joint state" if state_fresh else "last commanded q"

    def _on_state(self, msg: JointState) -> None:
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        if len(msg.position) >= 7:
            self.state_q = np.asarray(msg.position[:7], dtype=float)
            self._got_state = True
            self._state_t = time.monotonic()
            if self._tlog.enabled:
                # 实测关节事件：命令→实体执行链分析（到达率 + 每帧实测 q）。
                self._tlog.write(
                    {
                        "kind": "state",
                        "t": time.time(),
                        "side": self.side,
                        "q": [float(v) for v in msg.position[:7]],
                    }
                )

    def _reanchor_srv(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self._reanchor_teleop()
        resp.success = ok
        resp.message = message
        return resp

    def _manual_allowed(self):
        lease = getattr(self, '_policy_lease', None)
        return lease is None or lease.manual_allowed

    def _on_policy_state(self, msg: String):
        accepted = self._policy_lease.update(msg.data)
        if not self._policy_lease.manual_allowed:
            if self._armed:
                self._armed = False
                self._disarm_reason = "policy"
                self._tlog_event("disarm", disarm_reason="policy_ownership")
            return
        if not accepted:
            return
        payload = self._policy_lease.payload
        if not self._policy_lease.manual_allowed:
            self._homing = False
        elif payload.get('activity') == 'human':
            token = self._prepared_token
            if token and token in payload.get('reanchor_tokens', []):
                now = time.monotonic()
                freshness = self.data_timeout if self.data_timeout > 0 else 0.5
                if (now - self._last_vr_t <= freshness
                        and self._got_state and self._state_t is not None
                        and now - self._state_t <= freshness):
                    self._armed = True
                    self._disarm_reason = None
                self._prepared_token = None
        else:
            self._prepared_token = None

    def _prepare_reanchor_srv(self, _req, resp):
        # Preparation NEVER grants output ownership. Late or cancelled service
        # responses cannot arm a teleop writer after rollback.
        lease = self._policy_lease
        if lease.seen and (not lease.fresh or lease.activity == 'human'):
            resp.success, resp.message = False, 'ownership changed or expired'
            return resp
        ok, message = self._reanchor_teleop(arm=False)
        self._prepared_token = uuid.uuid4().hex if ok else None
        resp.success = ok
        resp.message = json.dumps({'token': self._prepared_token}) if ok else message
        return resp

    def _reanchor_teleop(self, arm=True):
        """Re-anchor robot origin + VR zero to the *current* state/pose, then arm.

        Used for HITL takeover: after a policy has moved the arm away from the
        startup pose, ``_start_teleop`` alone would command toward the stale
        startup ``robot_init`` and jump. This re-derives ``robot_init_pos/rot``
        from the latest measured joint state (fallback: last commanded q) and
        captures ``vr_init`` from the current VR pose, so subsequent motion is
        purely incremental from where the robot actually is.

        Returns ``(ok, message)``; on failure nothing is re-anchored/armed.
        """
        if arm and not self._manual_allowed():
            return False, 'policy owns output; prepare first'
        if self._homing:
            msg = "homing in progress; wait for init pose, then re-anchor"
            self.get_logger().warn(f"[{self.side}] reanchor: {msg}")
            self._tlog_event("reanchor", ok=False, reason=msg)
            return False, msg
        if self.pose.vr_current_pos is None or self.pose.vr_current_rot is None:
            msg = "no VR wrist pose yet; start Quest stream, place hand, then re-anchor"
            self.get_logger().warn(f"[{self.side}] reanchor: {msg}")
            self._tlog_event("reanchor", ok=False, reason=msg)
            return False, msg
        now = time.monotonic()
        freshness = self.data_timeout if self.data_timeout > 0 else 0.5
        if (now - getattr(self, '_last_vr_t', 0.0) > freshness
                or not self._got_state or self._state_t is None
                or now - self._state_t > freshness):
            return False, 'fresh wrist and measured joints required for reanchor'
        src = self._anchor_origin_to_measured()
        if not self.pose.calibrate_from_current():
            msg = "re-anchor failed (no VR pose for zero capture)"
            self.get_logger().warn(f"[{self.side}] reanchor: {msg}")
            self._tlog_event("reanchor", ok=False, reason=msg)
            return False, msg
        self._armed = bool(arm)
        self._disarm_reason = None
        self._tlog_event("reanchor", ok=True, armed=bool(arm))
        state = "armed" if arm else "prepared; output remains gated"
        self.get_logger().warn(
            f"[{self.side}] REANCHOR: robot origin ← FK({src}) "
            f"{np.round(self.robot_init_pos, 3).tolist()}, vr_init ← current pose, {state}"
        )
        return True, "re-anchored + armed" if arm else "re-anchored; prepared only"


    def _on_wrist(self, msg: PoseStamped) -> None:
        if self._homing:
            return
        if not fresh_header(msg, self.get_clock().now().nanoseconds * 1e-9):
            return
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        # Reject non-finite input first: NaN passes the norm check below
        # (NaN > 0.1 is False) and would permanently poison the EMA in
        # PoseProcessor and the SafetyFilter state downstream.
        if not np.isfinite(pos).all() or not np.isfinite(q).all():
            return
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            return
        self.pose.update_vr_pose(pos, Rotation.from_quat(q))
        self._last_vr_t = time.monotonic()
        self._last_vr_stamp = msg.header.stamp
        if self._print_latency:
            age = stamp_age_ms(msg.header.stamp)
            if 0.0 <= age < 5000.0:
                self._lat.add("vr_rx", age)
        if self._tlog.enabled:
            # 原始 VR 腕位事件：分析上游阶梯/停帧/到达率用（与 loop 记录对齐 wall 时钟）。
            self._tlog.write(
                {
                    "kind": "wrist",
                    "t": time.time(),
                    "side": self.side,
                    "pos": pos.tolist(),
                    "quat": [float(v) for v in q],
                    "age_ms": round(stamp_age_ms(msg.header.stamp), 2),
                    "frame": msg.header.frame_id,
                }
            )

    # ---- Human elbow prior (Quest body tracking -> geometric arm angle) ----

    def _apply_human_elbow_cfg(self) -> None:
        cont = getattr(self.ik, "continuity", None)
        if cont is not None and hasattr(cont, "w_psi_ref"):
            cont.w_psi_ref = self._human_elbow_weight

    def _ensure_body_subs(self) -> None:
        if not self._use_human_elbow or self._body_subs is not None:
            return
        if not hasattr(self.ik, "arm_angle_from_elbow_dir"):
            self.get_logger().warn(
                f"[{self.side}] use_human_elbow=true but solver "
                f"{getattr(self.ik, 'method_name', '?')} has no arm-angle prior "
                "support (geometric only) — ignoring"
            )
            self._use_human_elbow = False
            return
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._body_subs = (
            self.create_subscription(
                String, "quest3/body_joint_names", self._on_body_names, latched
            ),
            self.create_subscription(
                PoseArray, "quest3/body_joints", self._on_body_joints, _sensor_qos()
            ),
        )
        self.get_logger().info(
            f"[{self.side}] human elbow prior on: quest3/body_joints "
            f"({self.side}-arm-upper/-lower), w={self._human_elbow_weight}"
        )

    def _on_body_names(self, msg: String) -> None:
        try:
            names = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(names, list) and names:
            self._body_names = [str(n) for n in names]

    def _on_body_joints(self, msg: PoseArray) -> None:
        names = self._body_names
        if not names or len(msg.poses) != len(names):
            return
        # {side}-arm-upper joint sits at the shoulder ball, {side}-arm-lower
        # at the elbow (Meta IOBT skeleton; aliases tolerated).
        i_up = i_lo = -1
        for alias_up, alias_lo in (
            (f"{self.side}-arm-upper", f"{self.side}-arm-lower"),
            (f"{self.side}-upper-arm", f"{self.side}-lower-arm"),
            (f"{self.side}-shoulder", f"{self.side}-elbow"),
        ):
            if alias_up in names and alias_lo in names:
                i_up, i_lo = names.index(alias_up), names.index(alias_lo)
                break
        if i_up < 0:
            return
        pu, pl = msg.poses[i_up].position, msg.poses[i_lo].position
        d = np.array([pl.x - pu.x, pl.y - pu.y, pl.z - pu.z], dtype=float)
        if not np.isfinite(d).all():
            return
        n = float(np.linalg.norm(d))
        if n < 0.05 or n > 1.0:  # implausible human upper-arm length
            return
        d /= n
        now = time.monotonic()
        if self._elbow_dir_vr is None or self._elbow_dir_t <= 0.0:
            self._elbow_dir_vr = d
        elif self._elbow_held and self._elbow_reanchor_dir is not None:
            # 方案B：接管后保持 reanchor 时刻的臂角，直到操作者手臂方向相对
            # 该时刻变化超过阈值才切回人肘 EMA 跟随——肘不自行摆动（"卡/怪"）。
            ang = float(np.arccos(np.clip(np.dot(self._elbow_reanchor_dir, d), -1.0, 1.0)))
            if ang > self._reanchor_elbow_release_thresh:
                self._elbow_held = False
                if self._tlog.enabled:
                    self._tlog.write(
                        {
                            "kind": "event",
                            "t": time.time(),
                            "side": self.side,
                            "event": "elbow_hold_release",
                            "angle": round(ang, 4),
                        }
                    )
            else:
                # 保持接管时刻方向（不向操作者方向混合）
                self._elbow_dir_vr = self._elbow_reanchor_dir.copy()
        else:
            dt_e = min(0.5, max(1e-3, now - self._elbow_dir_t))
            a = math.exp(-dt_e / max(1e-3, self._human_elbow_tau))
            v = a * self._elbow_dir_vr + (1.0 - a) * d
            nv = float(np.linalg.norm(v))
            if nv > 1e-6:  # keep last good dir if EMA degenerates
                self._elbow_dir_vr = v / nv
        self._elbow_dir_t = now
        if self._tlog.enabled:
            # 人肘方向事件：臂角 psi 源（到达率 + 原始/EMA 方向，供 psi 台阶分析）。
            self._tlog.write(
                {
                    "kind": "body",
                    "t": time.time(),
                    "side": self.side,
                    "elbow_dir": [float(v) for v in d],
                    "elbow_dir_ema": (
                        [float(v) for v in self._elbow_dir_vr]
                        if self._elbow_dir_vr is not None
                        else None
                    ),
                }
            )

    def _human_psi_ref(self, T_flange: np.ndarray) -> Optional[float]:
        """Arm-angle prior (rad) from the latest human elbow direction."""
        fn = getattr(self.ik, "arm_angle_from_elbow_dir", None)
        d = self._elbow_dir_vr
        if fn is None or d is None or self._elbow_dir_t <= 0.0:
            return None
        if (time.monotonic() - self._elbow_dir_t) > self._human_elbow_timeout:
            return None
        # Same rotation as wrist deltas (direction only: no zero-point/scale).
        d_arm = self.pose.R_vr_to_arm @ d
        try:
            psi = fn(T_flange, d_arm, min_sin=self._human_elbow_min_sin)
        except Exception:  # noqa: BLE001 - never let the prior break teleop
            return None
        if psi is None and self._print_latency:
            # Fresh body data but prior gated (nearly straight human arm or
            # degenerate target) — visible in the [Latency] line.
            self._lat.count("psi_off")
        return psi

    def _loop(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._prev_t)
        self._prev_t = now
        if not self._manual_allowed():
            if self._armed:
                self._armed = False
                self._disarm_reason = "policy"
                self._tlog_event("disarm", disarm_reason="policy_lease_expired")
            self._maybe_log_latency()
            return
        if self._homing:
            self._homing_tick(now, dt)
            return
        if not self._armed or not self.pose.is_calibrated:
            self._maybe_log_latency()
            return
        # VR stream stale: disarm instead of silently holding the last pose.
        # Resuming requires a fresh /teleop/start, which also re-captures
        # vr_init and resets the EMA (avoids a stale-state jump on reconnect).
        # data_timeout <= 0 disables the watchdog. Logs once per event: after
        # disarm the loop exits at the not-armed check above.
        if self.data_timeout > 0 and (
            self._last_vr_t <= 0.0 or (now - self._last_vr_t) > self.data_timeout
        ):
            self._armed = False
            self._disarm_reason = "fault"
            self._tlog_event("fault", timeout_s=self.data_timeout)
            self.get_logger().error(
                f"[{self.side}] VR data timeout (>{self.data_timeout:.2f}s) — "
                "disarmed; re-send /teleop/start to resume"
            )
            self._maybe_log_latency()
            return

        t_loop = time.perf_counter()
        if self._print_latency and self._last_vr_t > 0:
            self._lat.add("vr_age", (now - self._last_vr_t) * 1000.0)
        vr_age_ms = (now - self._last_vr_t) * 1000.0 if self._last_vr_t > 0 else None

        dp, dr = self.pose.process(dt)
        # VR 目标速度（mm/s）：死区地板门控 + 慢/快打标用（rec 复用）
        vr_tgt = self.robot_init_pos + self.pose.last_raw_delta_pos
        if self._prev_vr_tgt is not None:
            vr_vel = (
                float(np.linalg.norm(vr_tgt - self._prev_vr_tgt))
                / max(dt, 1e-3)
                * 1000.0
            )
        else:
            vr_vel = 0.0
        self._prev_vr_tgt = vr_tgt.copy()
        T_tcp = self.pose.compute_target_pose(
            dp, dr, self.robot_init_pos, self.robot_init_rot
        )
        # SafetyFilter's radial workspace is centered at the solver base and
        # has historically constrained the solver-tip position.
        T_flange = tcp_to_solver_tip(T_tcp, self._T_solver_tip_to_tcp)
        p_req = T_flange[:3, 3].copy()
        r_req = float(np.linalg.norm(p_req))
        if self._print_latency:
            self._lat.add("ee_r", r_req * 1000.0, unit="mm")
        clip = {"ws_clip": 0, "reach_clip": 0}
        p_ws = self.safety.check_workspace(p_req)
        if not np.allclose(p_ws, p_req, atol=1e-9):
            T_flange[:3, 3] = p_ws
            T_tcp = solver_tip_to_tcp(T_flange, self._T_solver_tip_to_tcp)
            p_req = p_ws.copy()
            clip["ws_clip"] = 1
            if self._print_latency:
                self._lat.count("ws_clip")
        # Soft wall at full elbow extension: clamp the wrist target to the
        # reachable sphere around the shoulder center. Beyond it the elbow
        # circle degenerates, q4 hits its limit, IK fails, and the arm
        # catches; clamping keeps IK well-posed (hand feels a soft wall).
        clamp_reach = getattr(self.ik, "clamp_wrist_reach", None)
        if clamp_reach is not None and self.reach_margin > 0.0:
            p_clamped, clipped = clamp_reach(p_req, self.reach_margin)
            if clipped:
                T_flange[:3, 3] = p_clamped
                T_tcp = solver_tip_to_tcp(T_flange, self._T_solver_tip_to_tcp)
                p_req = p_clamped
                clip["reach_clip"] = 1
                if self._print_latency:
                    self._lat.count("reach_clip")
        # Warm-start from last *commanded* q (after vel limit), not raw IK jump.
        try:
            self.ik.sync_state(self._flip_q(self.q_cmd), reset_branch=False)
        except TypeError:
            self.ik.sync_state(self._flip_q(self.q_cmd))
        t_ik = time.perf_counter()
        psi_ref = self._human_psi_ref(T_flange) if self._use_human_elbow else None
        hard_fb = 0
        sol = None
        if (
            psi_ref is not None
            and self._human_elbow_mode == "hard"
            and hasattr(self.ik, "solve_hard")
        ):
            # Hard follow: resolve exactly at the human arm angle (elbow
            # strictly on the human's plane). solve_hard syncs the solver
            # state itself; on infeasible/singular psi it returns None and we
            # fall through to the soft-prior path (smooth degradation).
            sol = self.ik.solve_hard(T_flange, psi_ref)
            if self._print_latency:
                self._lat.count("hard_follow" if sol is not None else "hard_fallback")
            if sol is None:
                hard_fb = 1
        if sol is None:
            if psi_ref is None:
                sol = self.ik.solve(T_flange)
            else:
                sol = self.ik.solve(T_flange, psi_ref=psi_ref)
        if self._print_latency:
            self._lat.add("ik", (time.perf_counter() - t_ik) * 1000.0)
        ik_ms = (time.perf_counter() - t_ik) * 1000.0
        # Arm-angle escape: the local psi window went empty for
        # ik_escape_after_frames (wrist-limit boundary during a big roll) and
        # the solver jumped to a globally feasible psi — the visible shoulder
        # swing. Count it and log once per escape; return is debounced by
        # ik_return_after_frames.
        esc_state = getattr(self.ik, "_state", None)
        if esc_state is not None and esc_state.esc_active:
            if self._esc_prev_active is not True:
                self._esc_prev_active = True
                if self._print_latency:
                    self._lat.count("psi_escape")
                self.get_logger().warn(
                    f"[{self.side}] arm-angle escape: wrist limit swallowed the "
                    f"local psi window during a big roll; shoulder swings "
                    f"once, returns after ik_return_after_frames of feasible "
                    f"home psi"
                )
        else:
            self._esc_prev_active = False
        # jsonl 诊断记录（kind=loop）：VR 原始 / 滤波目标 / 命令 FK / q / 计数。
        # 仅在 teleop_log_file 开启时构建与写盘（零开销路径不碰这里）。
        rec = None
        if self._tlog.enabled:
            rec = {
                "kind": "loop",
                "t": time.time(),
                "side": self.side,
                "armed": True,
                # raw VR 目标（TCP 位，与 /teleop/{side}/tune/ee_vr 同源）
                "vr": (self.robot_init_pos + self.pose.last_raw_delta_pos).tolist(),
                "filt": T_tcp[:3, 3].tolist(),
                "q_state": self.state_q.tolist() if self._got_state else None,
                "psi_ref": psi_ref,
                "vr_age_ms": None if vr_age_ms is None else round(vr_age_ms, 2),
                "ik_ms": round(ik_ms, 2),
                "loop_ms": round((time.perf_counter() - t_loop) * 1000.0, 2),
                "hard_fallback": hard_fb,
                **clip,
            }
        if sol is None:
            if self._print_latency:
                self._lat.count("ik_fail")
            if rec is not None:
                rec["cmd"] = None
                rec["q"] = None
                rec["ik_fail"] = 1
                rec["ik_sat"] = 0
                self._tlog.write(rec)
            self._publish_tune_poses(T_tcp)
            self._maybe_log_latency()
            return
        ik_sat = 0
        if rec is not None or self._print_latency:
            Terr = self.ik.fk(sol)
            sat = float(np.linalg.norm(Terr[:3, 3] - T_flange[:3, 3]))
            ik_sat = 1 if sat > 0.008 else 0
            if self._print_latency and sat > 0.008:
                self._lat.count("ik_sat")
        prev_safe = self.safety.prev_q  # filter 前捕获（filter 内部会把 prev_q 更新为当前）
        safe, _info = self.safety.filter(sol, dt)
        # 电机指令最小步长地板（0=关）：慢速时把 0<|dq|<floor 的关节步抬到 floor，
        # 让电机每拍都有步长去跟（治"一顿一顿"；副作用=慢速微蠕，手停门控关闭）。
        db_nudge = False
        if (
            self._cmd_db_mrad > 0.0
            and vr_vel >= self._cmd_db_min_vel
            and prev_safe is not None
        ):
            dq_safe = safe - prev_safe
            floor = self._cmd_db_mrad * 1e-3
            mask = (np.abs(dq_safe) > 1e-9) & (np.abs(dq_safe) < floor)
            if np.any(mask):
                safe = prev_safe + np.where(mask, np.sign(dq_safe) * floor, dq_safe)
                self.safety.prev_q = safe.copy()
                db_nudge = True
                if self._print_latency:
                    self._lat.count("db_nudge", int(np.sum(mask)))
        # IK/safety work in the flipped convention; convert back to hardware.
        self.q_cmd = self._flip_q(safe)
        if self._print_latency:
            self._lat.add("loop", (time.perf_counter() - t_loop) * 1000.0)
        if rec is not None:
            T_cmd = self._fk_tcp(self._flip_q(self.q_cmd))
            rec["cmd"] = T_cmd[:3, 3].tolist()
            rec["q"] = self.q_cmd.tolist()
            rec["ik_fail"] = 0
            rec["ik_sat"] = ik_sat
            # ---- 跟手/慢速跟踪指标（量化 solver 差异的核心字段）----
            # vr_vel：VR 原始目标速度（mm/s），用于给慢/快帧打标（在 process 后已算）
            rec["vr_vel"] = round(vr_vel, 1)
            # db_nudge：本拍是否触发指令最小步长地板（死区交互可观测）
            rec["db_nudge"] = 1 if db_nudge else 0
            # pos_err/ori_err：命令 FK 与滤波目标的跟踪误差（跟手性核心）
            #   geometric 精确解析 → ori_err≈0；urdf_numerical 姿态权重 0.3 → 较大
            pos_err = float(np.linalg.norm(T_cmd[:3, 3] - T_tcp[:3, 3])) * 1000.0
            dR = T_cmd[:3, :3] @ T_tcp[:3, :3].T
            cos_a = float(np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0))
            ori_err = math.degrees(math.acos(cos_a))
            rec["pos_err"] = round(pos_err, 3)
            rec["ori_err"] = round(ori_err, 3)
            # psi_err：臂角跟随误差 |θ0_sel − psi_ref|（仅人肘先验有效；
            #   numeric/DH 无 psi_ref → null，表示压根没追人肘 = 肘乱跑的直接证据）
            theta0 = getattr(getattr(self.ik, "_state", None), "theta0_prev", None)
            psi_err = None
            if psi_ref is not None and theta0 is not None:
                psi_err = abs(float(psi_ref - theta0))
            rec["psi_err"] = None if psi_err is None else round(psi_err, 4)
            if vr_vel is not None:
                self._track_buf.append((vr_vel, pos_err, ori_err))
            self._tlog.write(rec)
        self._publish_tune_poses(T_tcp)
        if self.dry_run:
            self._maybe_log_latency()
            return
        self._publish_q()
        self._maybe_log_latency()

    def _publish_q(self) -> None:
        msg = JointState()
        msg.header.stamp = (
            self._last_vr_stamp
            if self._last_vr_stamp is not None
            else self.get_clock().now().to_msg()
        )
        msg.name = list(self.names)
        msg.position = self.q_cmd.tolist()
        self.cmd_pub.publish(msg)

    def _finish_homing(self, now: float, reason: str) -> None:
        mode = self._homing_mode
        self._homing_mode = "init"
        if mode == "park":
            if reason == "arrived":
                # 到零位：终态 q=0，并把机器人原点锚到零位 FK（后续 start/
                # re-anchor 从实际位姿重新标定前不会用旧 init 目标）
                self.q_cmd = np.zeros(7, dtype=float)
                T0 = self._fk_tcp(self._flip_q(self.q_cmd))
                self.robot_init_pos = T0[:3, 3].copy()
                self.robot_init_rot = T0[:3, :3].copy()
            # reason=timeout → 保持当前 q_cmd（未到零位绝不能硬发零目标）
            self.safety.set_initial_state(
                self._flip_q(self.q_cmd), self.robot_init_pos
            )
            self.pose.reset()
            self._homing = False
            elapsed = now - self._homing_t0 if self._homing_t0 else 0.0
            verb = "Parked at zero" if reason == "arrived" else "Park interrupted"
            self.get_logger().warn(
                f"[{self.side}] {verb} ({reason}, {elapsed:.1f}s). "
                "Arms hold; send /teleop/start to resume teleop"
            )
            return
        self.q_cmd = self._init_q_hw.copy()
        self.safety.set_initial_state(self._flip_q(self.q_cmd), self.robot_init_pos)
        self.pose.reset()
        self._homing = False
        if reason == "arrived":
            # 实体真到工作位才置位：电机未使能时命令"内部空跑"到 init_pose 但
            # 实体没动——只凭命令到达置位会让后续 /teleop/start 误用启动锚点
            # 跳变。要求实测新鲜且距 init_pose 在 homing_follow_tol 内。
            state_fresh = self._got_state and self._state_t is not None and (
                self.data_timeout <= 0.0
                or (
                    time.monotonic() - self._state_t
                    <= max(self.data_timeout, 1.0)
                )
            )
            if state_fresh and float(
                np.max(np.abs(self.state_q - self._init_q_hw))
            ) <= max(self._homing_follow_tol, 0.05):
                self._at_init_pose = True
            else:
                self._at_init_pose = False
                self.get_logger().warn(
                    f"[{self.side}] init 命令已到 init_pose 但实测不在"
                    f"（电机未使能？）——/teleop/start 将把原点重锚到当前实测"
                )
        elapsed = now - self._homing_t0 if self._homing_t0 else 0.0
        self.get_logger().warn(
            f"[{self.side}] Initial pose reached ({reason}, {elapsed:.1f}s). "
            "Hold VR still, then move."
        )

    def _abort_init_homing(
        self, now: float, target: np.ndarray, measured_error: float | None
    ) -> None:
        """Stop a work-position path at its current waypoint if feedback never arrives."""
        self._homing = False
        self._homing_mode = "init"
        self._at_init_pose = False
        self.safety.set_initial_state(self._flip_q(self.q_cmd), self.robot_init_pos)
        self.pose.reset()
        elapsed = now - self._homing_t0 if self._homing_t0 else 0.0
        measured = "unavailable" if measured_error is None else f"{measured_error:.3f} rad"
        self.get_logger().warn(
            f"[{self.side}] WORKPOS stopped at target {np.round(target, 3).tolist()}: "
            f"measured error={measured} after {elapsed:.1f}s; holding current command, "
            "init_pose unchanged. Check motor enable and joint-state feedback before retrying."
        )

    def _homing_tick(self, now: float, dt: float) -> None:
        """Slow joint-space approach: init_waypoints in order, then init_pose."""
        if not self._homing_started:
            self._homing_started = True
            self._homing_t0 = now
            self._homing_last_log = now
            self._homing_i = 0
            n = len(self._homing_path)
            self.get_logger().warn(
                f"[{self.side}] Homing {n} segment(s) at "
                f"{self._init_joint_vel:.2f} rad/s; first target="
                f"{np.round(self._homing_target(), 3).tolist()}"
            )

        elapsed = now - self._homing_t0
        dt = min(float(dt), 0.05)
        if not self._homing_seeded:
            if not self._got_state:
                if elapsed < 3.0:
                    return
                self.get_logger().warn(
                    f"[{self.side}] No joint_states after 3s; homing from zeros"
                )
                self.q_cmd = np.zeros(7, dtype=float)
            else:
                self.q_cmd = self.state_q.copy()
            self.safety.set_initial_state(
                self._flip_q(self.q_cmd), self.robot_init_pos
            )
            self._homing_seeded = True

        # 纯开环逐拍推进（base = q_cmd）——启动 init 与 HOME park 一致。HOME
        # park 的反馈只做"跟随便用"：实测落后指令超过 homing_follow_tol 说明
        # 电机没在跟随（失能/堵转），冻结轨迹不让 q_cmd 超前跑，等实测追近后
        # 自动继续。**不做**"每拍从实测位姿迈步"——那会把平滑位置控制变成等
        # 实测挪一步才挪一步的阶梯采样（命令永远只领先实测一个 tick 步长），
        # 实机 HOME 表现 = 慢慢的一卡一卡 + 突然抽一下。
        base = self.q_cmd
        if self._homing_mode == "park" and self._track_homing_state:
            follow_checked = False
            if self._got_state and self._state_t is not None:
                state_fresh = self.data_timeout <= 0.0 or (
                    now - self._state_t <= max(self.data_timeout, 1.0)
                )
                if state_fresh:
                    follow_checked = True
                    gap = float(np.max(np.abs(self.state_q - self.q_cmd)))
                    if gap > self._homing_follow_tol:
                        if not self._park_frozen:
                            self._park_frozen = True
                            self.get_logger().warn(
                                f"[{self.side}] HOME: 实测落后指令 {gap:.3f} rad "
                                f"(>{self._homing_follow_tol:.2f}) — 电机未使能/堵转？"
                                "轨迹冻结，等实测跟上来再继续"
                            )
                    else:
                        self._park_frozen = False
            if not follow_checked:
                # 实测流不可用：守卫无从判断，按纯开环继续（与 init 行为一致）。
                self._park_frozen = False

        target = self._homing_target()
        last = self._homing_i >= len(self._homing_path) - 1
        err = target - base
        max_abs = float(np.max(np.abs(err)))

        # park 途经点"实体到位门限"：
        # 命令到途经点 ≠ 实体到途经点——q_cmd 纯开环领先实测（MIT 跟踪误差 +
        # 链路延迟），命令一到点立即反向会被"切角"在 waypoint 前（症状：HOME
        # 伸出但没到 init_waypoints 就转去零位）。放行条件 = 命令到点 **且**：
        #   ① 实测进入 homing_via_tol 后**连续稳定 homing_via_settle_s**（实体
        #      真正到位停下，不是运动中擦过——运动中进入 tol 时 err 还会再出
        #      tol，连续计时会重置）；或
        #   ② 实测流不可用 → 按纯命令驻留 settle 时长后放行（无从判断实体）；
        #   ③ 等待超 homing_via_hold_s 总上限 → 放行（防长期不到位卡死归零）。
        # 等待期间命令钉在途经点重发，实体在位置保持下自然到位。
        via_waiting = False
        if (
            self._homing_mode == "park"
            and not last
            and max_abs < self._init_arrive_tol
            and self._homing_via_hold_s > 0.0
        ):
            if self._via_wait_t0 is None:
                self._via_wait_t0 = now
                self._via_in_tol_since = None
            state_fresh = self._got_state and self._state_t is not None and (
                self.data_timeout <= 0.0
                or (now - self._state_t <= max(self.data_timeout, 1.0))
            )
            if state_fresh:
                via_err = float(np.max(np.abs(self.state_q - target)))
                if via_err <= self._homing_via_tol:
                    if self._via_in_tol_since is None:
                        self._via_in_tol_since = now
                else:
                    self._via_in_tol_since = None  # 又出 tol：连续计时清零
            wait_elapsed = now - self._via_wait_t0
            if state_fresh:
                settled = (
                    self._via_in_tol_since is not None
                    and (now - self._via_in_tol_since) >= self._homing_via_settle_s
                )
            else:
                settled = wait_elapsed >= self._homing_via_settle_s
            via_waiting = not settled and wait_elapsed < self._homing_via_hold_s

        # 工作位（/teleop/init）要求实体确实到达每一个途经点以及最终目标。
        # 之前仅凭 q_cmd 到点就切段，左臂实测落后 4.7 rad 时仍继续向 init_pose
        # 发命令，视觉上就像跳过了途经点。命令到点后固定重发当前目标，等实测
        # 进入容差并稳定；超时则原地停止，绝不越过未确认的安全点。direct 路径
        # 只有最终目标，因此同样会确认实体到位，但不会增加中间途经点。
        init_waiting = False
        init_measured_error = None
        if self._homing_mode == "init" and max_abs < self._init_arrive_tol:
            if self._via_wait_t0 is None:
                self._via_wait_t0 = now
                self._via_in_tol_since = None
            state_fresh = self._got_state and self._state_t is not None and (
                self.data_timeout <= 0.0
                or now - self._state_t <= max(self.data_timeout, 1.0)
            )
            settled = False
            if state_fresh:
                init_measured_error = float(np.max(np.abs(self.state_q - target)))
                if init_measured_error <= self._homing_via_tol:
                    if self._via_in_tol_since is None:
                        self._via_in_tol_since = now
                    settled = (
                        now - self._via_in_tol_since >= self._homing_via_settle_s
                    )
                else:
                    self._via_in_tol_since = None
            else:
                self._via_in_tol_since = None
            init_waiting = not settled
            wait_elapsed = now - self._via_wait_t0
            if init_waiting and wait_elapsed >= self._homing_init_arrive_timeout:
                self._abort_init_homing(now, target, init_measured_error)
                self._publish_q()
                return

        if now - self._homing_last_log >= 2.0:
            if self._homing_mode == "park" and last:
                kind = "zero"
            elif self._homing_mode == "park":
                kind = f"park[{self._homing_i}]"
            else:
                kind = "init" if last else f"via[{self._homing_i}]"
            stuck = ""
            if self._park_frozen:
                stuck = " — 实测落后，轨迹冻结（电机未使能/堵转？）"
            elif via_waiting:
                stuck = " — 途经点驻留：等实体到位（防切角）"
            elif init_waiting:
                error_text = (
                    "等待关节反馈" if init_measured_error is None
                    else f"实测误差 {init_measured_error:.3f} rad"
                )
                stuck = f" — 工作位等待实体到点：{error_text}"
            self.get_logger().info(
                f"[{self.side}] Homing {kind}: error={max_abs:.3f} rad, "
                f"elapsed={elapsed:.1f}s{stuck}"
            )
            self._homing_last_log = now
            self._homing_last_base = base.copy()

        if max_abs < self._init_arrive_tol:
            if init_waiting:
                # 包括最后一个 init_pose 目标：未确认实体到位时不能报告完成。
                self._publish_q()
                return
            self.q_cmd = target.copy()
            if last:
                self._finish_homing(now, "arrived")
                self._publish_q()
                return
            if via_waiting:
                # 命令已到途经点但实体还没到位停稳：钉住重发，等实体到位。
                self._publish_q()
                return
            # 放行日志附实测距途经点的实际误差（数字裁决"实体到没到过途经点"；
            # target 此刻仍是刚放行的途经点）。
            meas_note = ""
            state_fresh = self._got_state and self._state_t is not None and (
                self.data_timeout <= 0.0
                or (now - self._state_t <= max(self.data_timeout, 1.0))
            )
            if state_fresh:
                meas_note = (
                    f", 实测距途经点 "
                    f"{float(np.max(np.abs(self.state_q - target))):.3f} rad"
                )
            self._homing_i += 1
            self._via_wait_t0 = None
            self._via_in_tol_since = None
            self.get_logger().warn(
                f"[{self.side}] Via {self._homing_i}/{len(self._homing_path)-1} "
                f"reached; next={np.round(self._homing_target(), 3).tolist()}"
                f"{meas_note}"
            )
            self._publish_q()
            return
        if elapsed >= self._init_timeout:
            if self._homing_mode == "park":
                self.get_logger().warn(
                    f"[{self.side}] Home timeout ({self._init_timeout:.0f}s), "
                    f"error={max_abs:.3f} rad — hold at current q"
                )
            else:
                # 启动 init 归位保持改前措辞与语义：超时即用当前 q 作 VR 零点。
                self.get_logger().warn(
                    f"[{self.side}] Init pose timeout ({self._init_timeout:.0f}s), "
                    f"error={max_abs:.3f} rad — VR zero uses current q"
                )
            T0 = self._fk_tcp(self._flip_q(self.q_cmd))
            self.robot_init_pos = T0[:3, 3].copy()
            self.robot_init_rot = T0[:3, :3].copy()
            self._init_q_hw = self.q_cmd.copy()
            self._finish_homing(now, "timeout")
            return

        if self._park_frozen:
            # 冻结：不推进 q_cmd（领先被钳在 follow_tol 内），重发当前目标保持，
            # 等实测追近（电机恢复跟随）后自然解除，继续走完收回轨迹。
            self._publish_q()
            return

        max_d = self._init_joint_vel * dt
        self.q_cmd = base + np.clip(err, -max_d, max_d)
        self._publish_q()

    def _pose_msg(self, T: np.ndarray, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{self.side}_ik"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(T[0, 3]),
            float(T[1, 3]),
            float(T[2, 3]),
        )
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        return msg

    def _publish_tune_poses(self, T_tcp: np.ndarray) -> None:
        if not self._publish_tune or not self._tune_pubs:
            return
        stamp = (
            self._last_vr_stamp
            if self._last_vr_stamp is not None
            else self.get_clock().now().to_msg()
        )
        T_vr = self.pose.compute_target_pose(
            self.pose.last_raw_delta_pos,
            self.pose.last_raw_delta_rot,
            self.robot_init_pos,
            self.robot_init_rot,
        )
        T_cmd = self._fk_tcp(self._flip_q(self.q_cmd))
        self._tune_pubs["vr"].publish(self._pose_msg(T_vr, stamp))
        self._tune_pubs["filt"].publish(self._pose_msg(T_tcp, stamp))
        self._tune_pubs["cmd"].publish(self._pose_msg(T_cmd, stamp))
        q_vr = Rotation.from_matrix(T_vr[:3, :3]).as_quat()
        q_fi = Rotation.from_matrix(T_tcp[:3, :3]).as_quat()
        q_cmd = Rotation.from_matrix(T_cmd[:3, :3]).as_quat()
        packed = Float64MultiArray()
        packed.data = [
            float(T_vr[0, 3]),
            float(T_vr[1, 3]),
            float(T_vr[2, 3]),
            float(T_tcp[0, 3]),
            float(T_tcp[1, 3]),
            float(T_tcp[2, 3]),
            float(T_cmd[0, 3]),
            float(T_cmd[1, 3]),
            float(T_cmd[2, 3]),
            *map(float, q_vr),
            *map(float, q_fi),
            *map(float, q_cmd),
        ]
        self._tune_pubs["xyz"].publish(packed)

    def _publish_runtime_status(self) -> None:
        now = time.monotonic()
        feedback_fresh = bool(
            self._got_state and self._state_t is not None
            and now - self._state_t <= max(self.data_timeout, 1.0)
        )
        payload = {
            "side": self.side,
            "armed": bool(self._armed),
            "homing": bool(self._homing),
            "homing_mode": self._homing_mode,
            "at_init_pose": bool(self._at_init_pose),
            "at_zero": bool(feedback_fresh and np.max(np.abs(self.state_q)) < 0.05),
            "feedback_fresh": feedback_fresh,
            "ownership": getattr(getattr(self, "_policy_lease", None), "activity", "unknown"),
            "sent_at": time.time(),
        }
        self._runtime_status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _track_summary(self) -> dict:
        """慢/快段跟踪质量窗口汇总（供 kind=metrics 的 track 字段）。

        按 vr_vel（mm/s）分慢（<40）/快（≥40）段，输出 pos_err/ori_err 的 p95
        与慢帧占比。用于对比 solver（geometric vs urdf_numerical）的跟手性与
        慢速跟踪：numeric 姿态权重低 → ori_err 更大，慢速段更明显。
        """
        buf = list(self._track_buf)
        self._track_buf.clear()
        if len(buf) < 10:
            return {}
        vr_vel = np.array([b[0] for b in buf])
        pe = np.array([b[1] for b in buf])
        oe = np.array([b[2] for b in buf])
        slow = vr_vel < 40.0
        out = {"slow_frac": round(float(np.mean(slow)), 3)}
        if slow.sum() >= 5:
            out["pos_err_slow_p95"] = round(float(np.percentile(pe[slow], 95)), 2)
            out["ori_err_slow_p95"] = round(float(np.percentile(oe[slow], 95)), 2)
        if (~slow).sum() >= 5:
            out["pos_err_fast_p95"] = round(float(np.percentile(pe[~slow], 95)), 2)
            out["ori_err_fast_p95"] = round(float(np.percentile(oe[~slow], 95)), 2)
        return out

    def _maybe_log_latency(self) -> None:
        if not self._print_latency or not self._lat.has_samples():
            return
        if not self._lat.should_print():
            return
        if self._tlog.enabled:
            # kind=metrics：LatencyMeter 窗口汇总（ms 统计 + 计数）+ 慢/快段跟踪质量，
            # 随 [Latency] 周期 ~2s 落一条；非破坏性 snapshot，format_and_reset 照常清窗。
            track = self._track_summary()
            self._tlog.write(
                {
                    "kind": "metrics",
                    "t": time.time(),
                    "side": self.side,
                    "armed": self._armed,
                    "homing": self._homing,
                    **self._lat.snapshot(),
                    **({"track": track} if track else {}),
                }
            )
        self.get_logger().info(
            f"[Latency][{self.side}] {self._lat.format_and_reset()}"
        )

    def destroy_node(self) -> bool:
        if getattr(self, "_tlog", None) is not None:
            self._tlog.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstralTeleopArmNode()
    try:
        rclpy.spin(node)
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
