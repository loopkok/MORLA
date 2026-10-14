# astral_arm_teleop

Quest3 腕部 → 双臂 IK（DH / 几何闭式 / URDF 数值）→ `/left_arm|/right_arm/joint_commands`。  
**不** import `astral_robot_sdk`；硬件由 `astral_robot_control` 负责。

当前架构是 **B：模型不动，控制层换算**。SolidWorks 的 `astral_robot_description` URDF、旧 MJCF、真机关节符号都保持原约定；闭式 DH 在「翻转轴 + 干净 MDH 基座」里求解，teleop 边界用 `flip_q` 和 `R_baseᵀ` 对接。

## 数据流（2× 单臂节点）

```text
quest3_udp_mocap (convert_to_robot:=true)
  Mixed → robot_world (X left, Y back, Z up)
  IOBT  → robot_body  (hips 系；+X右/+Y上/+Z前)
  # mocap 保证整段流 frame_id 稳定（sticky latch + hold gate）：
  # 见过 body 包即不回退世界系；启动后 head/wrist/controller 在帧确定前不发布。
  # 本节点按 "当前 - vr_init" 算增量；稳定系下增量有界、IK 可达。
  # IOBT 下增量即"手相对躯干"的位移，正是以髋为父、手身解耦的语义。

astral_arm_teleop_{left|right}                 # 进程并行
  PoseProcessor × vr_to_arm_rot
      analytic_dh: yaml(I) 再左乘 R_baseᵀ → 干净 MDH 基座
      urdf_numerical / geometric: yaml(I) 原样 → SW `left_base_link` / `right_base_link`
        （Pinocchio `T_base⁻¹ T_ee`，不再用 universe/躯干）
  → workspace_radius 球约束（IK 帧 *_base_link 原点；≤0 关闭）
  → IK
      analytic_dh: 翻转约定 q_dh（α=+90 闭式）
      geometric: 硬件约定 q_hw（URDF 几何 + POE/PK 闭式）
      urdf_numerical: 硬件约定 q_hw（Pinocchio LM）
  → SafetyFilter（关节限位 / 速度；与求解器同一约定）
  → flip_q（仅 analytic_dh）→ q_hw
  → /{side}_arm/joint_commands             # 始终旧约定；BEST_EFFORT depth=1（只留最新帧）
      → 真机 astral_robot_control
      → 或仿真 astral_mujoco_sim（旧 MJCF，无 flip）
```

```bash
ros2 run quest3_hand_mocap quest3_udp_mocap --ros-args \
  -p convert_to_robot:=true -p arm_side:=both -p protocol:=tcp_wired
```

## 两套约定

| | DH / analytic IK | 真机 / 旧 MJCF / URDF 数值 |
|--|--|--|
| 基座 | 干净 MDH：\(z_0 \parallel\) joint1 轴 | SolidWorks `*_base_link`（joint1 轴 ≈ −X） |
| 关节符号 | 左翻 2/3/4，右翻 2/4，使 \(\alpha=+90^\circ\) | URDF `axis xyz="0 0 1"` 原方向 |
| 谁转换 | teleop：`R_baseᵀ` + `flip_q` | 仿真/驱动直接吃 `joint_commands` |

yaml 里 `init_pose` / `vr_to_arm_rot` 都按 **旧约定 / robot_world** 写。  
单臂节点在 `solver_type=analytic_dh` 时自动：`R = R_baseᵀ @ yaml`，并对 init / seed / 输出做 `flip_q`。

## 双 IK（包内 `ik/`）

```text
astral_arm_teleop/
  ik/
    analytic.py         ← 闭式臂角 DH（AstralParams + 公式修复）
    geometric.py        ← 免 DH 臂角闭式（S/E/W 空间几何，POE + Paden-Kahan）
    urdf_solver.py      ← Pinocchio LM
    factory.py          ← make_ik_solver / make_single_arm_ik
  astral_arm_teleop_node.py   ← 自动 R_baseᵀ + flip_q
```

| `solver_type` | 实现 | 位姿帧 | 输出 q | 默认模型 |
|---------------|------|--------|--------|----------|
| `analytic_dh` | Nero 臂角闭式，硬编码 MDH | 干净 MDH 基座 | 翻转约定，发布前 flip | 无 URDF |
| `geometric`（yaml 默认） | 免 DH 臂角闭式：URDF 提取肩/肘/腕中心与关节轴，POE + Paden-Kahan；支持 `use_human_elbow` 人臂肘先验 | SW `*_base_link` | 已是硬件约定，不 flip | `astral_robot_description/urdf/astral_robot.pin.urdf` |
| `urdf_numerical` | Pinocchio + scipy LM | SW `*_base_link` | 已是硬件约定，不 flip | `astral_robot_description/urdf/astral_robot.pin.urdf` |

单臂节点 `urdf_path` 为空 → `default_astral_urdf_path()` = **`astral_robot.pin.urdf`**（与 MuJoCo 同源）。  
支持 RobotMain 命名（`Joint_la_*`）或 Astral 命名（`left_joint*`）。

### 人臂肘先验（`use_human_elbow`，仅 `geometric`）

Quest IOBT 身体追踪经 `quest3/body_joints` 发布 `{side}-arm-upper`（肩球头）/
`{side}-arm-lower`（肘）关节位置（`robot_body` 系，推算但可用）。节点取大臂
方向 `d = elbow − shoulder`，用与腕部增量相同的 `vr_to_arm_rot` 旋到机械臂基座系
（纯方向，与 zero point / motion_scale / 人臂长无关），投影到机器人 S–W 轴垂面
得到人臂角 `psi_ref`，作为软先验传入 `solve(T, psi_ref)`：

- 局部 ψ 扫描窗口以 `psi_ref` 为中心，评分加 `w_psi_ref·|ψ − psi_ref|`
  （`human_elbow_weight`，默认 2.0，须主导 w_vel=1.0 / w_theta0=0.15）；
- 效果：人转腕带动肘平面转 → 机器人肘跟转；人只转腕 → 肩肘不动；
- 鲁棒：`human_elbow_timeout`（0.3 s）外或关节名缺失自动回退纯连续性；
  肘方向 EMA（`human_elbow_smoothing_tau` 0.15 s）抑制 IOBT 噪声；
  `psi_ref` 落在关节限位不可行区时走全局扫描 + 连续性，不会卡死；
- **伸直度门控**（`human_elbow_min_sin`，默认 0.15）：人臂接近伸直时肘尖
  几乎在肩腕连线上，ψ 不可观测 —— IOBT 骨架的系统性肘偏置会被归一化
  放大成满权重先验，把机器人肘拧到固定错误角度（典型症状：手臂放下时
  肘部固定偏 ~90°）。低于门限自动关闭先验回退纯连续性，被门控的帧计入
  `[Latency]` 行的 `psi_off`。0.15 ≈ 肘尖离线 4 cm；正常弯曲手势
  sin ≥ 0.5，远离门限。

**硬跟肘模式（`human_elbow_mode: "hard"`，当前 yaml 默认）**：软先验是
weight 混合（肘半跟人半跟惯性，观感偏"妥协"）。`"hard"` 下只要人肘方向
新鲜且门控通过，IK 直接**精确解在人臂角上**（`GeometricIKSolver.solve_hard`：
`_collect_solutions(T, ψ_human)` + 最小 warm-start 选支，无网格/1D QP/逃逸
迟滞）——机器人肘严格贴人臂；ψ_human 不可行/奇异/超龄期时自动回退软先验
路径（`[Latency]` 行 `hard_follow`/`hard_fallback` 计数可观测），不会卡死。
**录制前提**：硬模式下执行的 q 的臂角 == 喂入的人臂角（已测肘方向偏差
<0.02°），因此遥操录制的关节流离线 `psi_from_config(q)` 即得人臂角标签——
这是 astral_pim_ik 训练"复现人手姿态"数据集的来源（`convert_sessions.py`）。
模式可运行时热改（参数回调）；`"soft"` 为历史默认，保持旧行为。

```bash
# 求解器见 astral_arm_teleop_{left,right}.yaml（默认 geometric + use_human_elbow + human_elbow_mode=hard）
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py dry_run:=true
# 一次性覆盖：solver_type:=analytic_dh
```

### 肘伸直奇异（`reach_margin`，仅 `geometric`）

肘完全伸直时 S/E/W 共线：肘圆半径 → 0，臂角 ψ 失去定义，且 q4（本机
`[-2.26, 0]`，0 = 伸直）正好撞限位 —— IK 在可达边界带内无解，节点保持上一帧
指令，恢复时臂会"卡一下"。两层防护：

- **软墙**：腕目标径向钳制在肩心 `l_se + l_ew - reach_margin`（当前 yaml **0.001 m**——软墙近放平、
  逼近 URDF 伸直极限才挡；节点代码默认 0.01 m）内，
  方向不变，手感到的是一堵软墙而不是卡顿；`reach_margin <= 0` 关闭，
  可热改，钳制次数计入 `[Latency]` 行的 `reach_clip`；
- **ψ 冻结**：`sin α < 0.05`（肘圆半径 < ~1.2 cm）时 ψ 在数值上无意义
  （此时解本就与 ψ 无关），扫描网格塌缩为 `θ0_prev` 单候选，杜绝肩部噪声跳变。

### 大幅 roll 时的肩部抽搐（逃逸迟滞，仅 `geometric`）

末端 roll 幅度超过腕关节行程（q5/q6/q7 各 ±100°，纯 roll ~180° 需多关节
分担）时，**当前臂角下**腕部可能翻不到目标姿态：局部窗候选全撞限位被弃，
旧行为同帧全局逃逸到远处可行 ψ（肩部猛甩），下一帧 `psi_ref` 窗恢复可行
又拉回 —— 在可行边界逐帧振荡，真机上表现为肩部"抽搐一下又拉回来"。

逃逸迟滞（`ContinuityParams.escape_after_frames` 默认 4、
`return_after_frames` 默认 20，~150 Hz 下 ≈ 27 ms / 133 ms）：

- 局部窗**连续空 4 帧**才允许全局逃逸，边界单帧闪烁不再甩肩；
- 逃逸后驻留在逃逸区（连续性窗跟随，期间 `w_psi_ref` 把臂角向人臂方向
  缓慢牵回），探针确认人臂角**连续 20 帧**可行才正式返回 —— 一次大 roll
  最多甩一次、回一次，且都被速度限幅摊成平滑过渡；
- 冷启动（无 `θ0_prev`）不受影响，立即全局求解；`1/1` 可复现旧行为。
- 两个帧数已暴露为节点参数（可热改）：`ik_escape_after_frames`（默认 4，
  调大 = 越不容易甩肩、但腕姿态"卡住不跟"的窗口越长）、
  `ik_return_after_frames`（默认 20，调小 = roll 结束后更快回到人臂角）。
  逃逸发生时打 WARN 日志一次并计入 `[Latency]` 行的 `psi_escape` 计数，
  真机看到肩部突动先查这个。

### 按求解器切换的 `vr_to_arm_rot` 与 `flip_q`

推荐路径 **`astral_arm_teleop_node`**（左右 yaml 里 `vr_to_arm_rot` 仍是 **I**）：

| | `analytic_dh` | `urdf_numerical` / `geometric` |
|--|--|--|
| 有效 `vr_to_arm_rot` | `_R_BASE_T @ yaml` | yaml（默认 I） |
| `flip_q` | 左 2/3/4、右 2/4 取反（自反） | 恒等 |
| Safety 限位 | DH 限位 | URDF 限位 |

`_R_BASE_T`（\(R_\text{base}^\top\)，行优先）：

```text
[ 0  0  1 ]
[ 0  1  0 ]
[-1  0  0 ]
```

把 `robot_world` / SW `*_base_link` 下的增量转到「joint1 轴 = +Z」的干净基座。与关节翻转无关。

`flip_q` 边界（仅 DH）：yaml `init_pose`（旧约定）→ flip 入 IK；warm-start `flip(q_cmd)`；输出 `flip(safe)` 再发 `joint_commands`。

## DH 参数与闭式公式（`ik/analytic.py`）

闭式解要求 SRS + \(a_i=0\) + \(\alpha=+90^\circ\)。Astral CAD 轴符号混杂，用翻转约定把链统一成 +90°，而不是改 URDF。

### 参数（相对早期照抄 Nero）

| | 改前 | 改后 |
|--|------|------|
| 左 `d_i` | `[0.11655, 0, 0.246, 0, 0.23058, 0, 0.03474]` | `[-0.11655, 0, 0.246, 0, 0.2265, 0, 0.0036]` |
| 右 `d_i` | 与左相同 | `[0.11655, 0, -0.246, 0, 0.2265, 0, 0.0036]` |
| 左 \(\theta_\text{offset}\)（deg） | Nero `[0,-180,-180,-180,90,90,0]` | `[180,90,90,180,180,-90,-90]` |
| 右 \(\theta_\text{offset}\) | 左臂别名 | `[180,-90,-90,0,180,-90,-90]` |
| `apply_base_fixed` | 默认 True（joint1 rpy hack） | **False**（干净基座由 `R_baseᵀ` 承担） |

`d4=0.2265` 是前臂长度；旧 `0.23058` 量错。`d6=0.0036` 是沿腕轴；旧 `0.03474` 把横向视觉偏置误当 MDH \(d\)。

### 公式修复（与 `_dh_A` 的 \(\alpha=+90^\circ\) 对齐）

1. **`_solve_q123_from_swe`**：`c2=(d0-Ez)/d2`；`c1/s1` 去负号；`th3=atan2(w[1]/s4, w[0]/s4)`；返回满角。Nero 曾靠 offset=−180° 碰巧盖住 \(\alpha\) 符号错配。
2. **`_solve_theta4_from_triangle`**：按 `d_i[0]` 符号分左右——左 `-acos(c4)`，右 `+acos(c4)`（镜像肘）。

在「±q + 干净基座」映射下，DH FK 与 URDF FK 约 0.05 mm / 0.001°；SELF_IK 自洽。直接在 SW `*_base_link` 同 q 对比会差数百 mm，那是约定不一致，不是杆长算错。

### DH 限位（翻转约定，已放宽）

左右臂统一：

| J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|----|----|----|----|----|----|----|
| [-2, 2] | [-2, 2] | [-1.57, 1.57] | [-1.012, 2.26] | [-1.57, 1.57] | [-0.8, 0.85] | [-1.57, 1.57] |

`init_pose` 仍为旧约定，例如左 `[0.32, 0.11, -0.53, -0.80, 0.28, 0, 0]`。  
注意：DH 限位 flip 到硬件后，J2/J4 可能宽于旧 URDF/MJCF（左 J2 `[-0.3,2]`、J4 `[-2.26,0]`）。Safety 只在 DH 空间钳位。仿真/真机仍受旧 `ctrlrange` 约束。

## 仿真 / URDF / MJCF —— **未改运行时模型**

| 文件 | 是否改控制闭环 |
|------|----------------|
| `astral_robot_description`（`astral_robot.pin.urdf`） | **否**，仍 SW 原轴 |
| `astral_mujoco_sim` 默认 `assets/mjcf/astral_dual.xml` | **否** |
| `astral_mujoco_sim.yaml` 的 `init_pose_*` | **否**（旧约定，与 teleop yaml 一致） |

仿真吃的是 flip 之后的 `q_hw`，自己不再 flip。  
**不要**把默认 `mjcf_path` 指到 `astral_dual_clean.xml`，除非整条链都改成干净约定。

## 可选包与工具（不影响默认闭环）

| | 作用 |
|--|--|
| `astral_arm_clean_description` | 干净单臂/双臂 URDF（关节原点=MDH，mesh 用 visual origin 重定位）。架构 B 下 **不接入** teleop/仿真 |
| `astral_mujoco_sim/assets/mjcf/astral_dual_clean.xml` | 对应干净 MJCF；默认不用 |
| `derive_clean_mdh.py` | 从 URDF 抽 MDH 诊断 |
| `generate_clean_urdf.py` | 生成干净 URDF |
| `generate_clean_mjcf.py` | 生成干净 MJCF |
| `fit_dh_from_urdf.py` | DH 拟合审计 |

```bash
python3 -m astral_arm_teleop.fit_dh_from_urdf
python3 -m astral_arm_teleop.derive_clean_mdh
python3 -m astral_arm_teleop.generate_clean_urdf
```

## 依赖

```bash
# URDF IK:
# pip install pin

cd /home/robot/loopkok/sdk/astral_ws
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_robot_description astral_robot_control astral_arm_teleop --symlink-install
source install/setup.bash
```

## 启动

```bash
# 2× 单臂 teleop（自动 R_baseᵀ + flip_q）
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py dry_run:=true
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py \
  with_driver:=true control_board_ip:=192.168.10.2
# 单左臂：arm_side:=left（right 对称）。整机左臂+夹爪见 astral_teleop arm_side:=left
# 纯臂：不含夹爪 / 灵巧手。整机编排见 astral_teleop/full_teleop.launch.py

# 仿真管线（solver / protocol 见 yaml，默认 geometric + tcp_wired）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py
# 一次性覆盖：solver_type:=analytic_dh  protocol:=udp
```

Quest3 有线：`adb reverse tcp:8000 tcp:8000`。  
**不要**与 `astral_robot_control` 和 `astral_mujoco_sim` 同时订同一 `joint_commands`。

左手夹爪（独立包 `astral_gripper_teleop`）：`hand_landmarks/left` 拇指–食指距离 → `/left_gripper/command`（0 开 1 合）。真机由驱动 `set_gripper_angle` 下发。本包不再 include 夹爪；整机编排见 `astral_teleop`，单独跑夹爪用 `ros2 launch astral_gripper_teleop gripper_teleop.launch.py`。

## 遥操诊断 JSONL 日志（`teleop_log_file`）

排查卡顿（如慢速平移"停一下走一下"）时把链路逐级数据落盘，照 `astral_policy_inference` 的
`metrics_log_file`/`joint_stream_log_file` 模式。参数 `teleop_log_file`（空=关，零开销）非空则
行缓冲追加写，写失败不打断控制流。同一文件 `kind` 区分六类记录：

| kind | 触发 | 关键字段 |
|------|------|---------|
| `event` | 每次生命周期/接管事件 | `event`=armed/disarm/reanchor/start/fault、`disarm_reason`、`ok`、`timeout_s`（一条 jsonl 即可还原 HITL takeover 全流程） |
| `loop` | 每 armed 控制拍 | `vr`/`filt`/`cmd` TCP 位、`q`/`q_state`、`psi_ref`、`vr_age_ms`/`ik_ms`/`loop_ms`、`ik_fail`/`ik_sat`/`hard_fallback`/`ws_clip`/`reach_clip`、**`pos_err`/`ori_err`**（命令 FK vs 滤波目标）、**`psi_err`**（臂角跟随误差，无人肘先验=null）、**`vr_vel`**（VR 目标速度 mm/s）、**`db_nudge`**（本拍是否触发指令最小步长地板） |
| `wrist` | 每腕位事件 | `pos`/`quat`/`age_ms`/`frame`（upstream 阶梯/到达率证据） |
| `state` | 每实测关节事件 | `q`（命令→实体执行链） |
| `body` | 每 body_joints | `elbow_dir`/`elbow_dir_ema`（臂角 psi 源） |
| `metrics` | ~2s | LatencyMeter 窗口 `ms` 统计 + `counts` + **`track`**（`slow_frac` 慢帧占比 + 慢/快段 `pos_err`/`ori_err` p95） |

**对比 solver（geometric vs urdf_numerical）用 `pos_err`/`ori_err`/`psi_err`/`track`**：
geometric 精确解析 → `pos_err`≈0、`ori_err`≈0、`psi_err`≈0（硬跟人肘）；urdf_numerical
姿态权重 0.3 → `ori_err` 明显更大、`psi_err`=null（不追人肘）、肘乱跑但位置跟得上。
同一段遥操分别用两种 `solver_type` 各录一份，对比 `track` 的慢/快段 p95 即量化跟手性与
慢速差异。

### 电机指令最小步长地板（`cmd_deadband_mrad`）

**背景**：电机最小可靠步长 ~1 mrad（机械/固件死区，实机验证调增益无效）。geometric 精确解
+ 最小关节速度优化，慢速时把指令压到死区下（实测慢速段 24% 拍整臂 <1 mrad、单关节
51~69%）→ 电机不执行 → 一顿一顿；urdf 的"浪费型"关节运动反而每拍超死区 → 平滑。

**参数**（可热改 `ros2 param set`）：
- `cmd_deadband_mrad`（0=关）：>0 时，VR 目标速度 ≥ `cmd_deadband_min_vel` 的运动中，
  把 `0<|dq|<floor` 的关节步抬到 floor（方向保持），让电机每拍都有步长去跟。
- `cmd_deadband_min_vel`（默认 10 mm/s）：手停时不抬（避免静止微蠕）。

**副作用**：慢速时关节带恒定微动（轻微"蠕"）、TCP 可能略超目标速度（电机被强迫每拍动）。
真机 A/B 建议从 1.0 起试，观察慢速是否从"一顿一顿"变成"平滑微蠕"，再调 1.2/1.5 找手感。

### 求解器对比测试（geometric vs urdf_numerical）

详细测试记录（现象/方法/指标/数据/量化结论）见
`doc/2026-09-17-ik-solver-comparison.md`；测试数据与可复用分析脚本归档在
`astral_test_logs/2026-09-17_ik_solver_comparison/`（sdk 根，独立于 git 仓库）。
**一句话结论**：geometric 慢速"一顿一顿" = 电机死区(~1mrad)与 geometric"最小关节速度"
优化的交互（慢速指令压在死区下、整臂全<1mrad 占 ~24-37%，urdf 仅 ~7-9%）；urdf 平滑但
肘乱跑、位置误差 1.3mm。

```bash
# CLI：log_dir 模式（推荐）——每次 launch 自动建 {log_dir}/{YYYYMMDD-HHMMSS}[_tag]/
# 运行子目录并按侧拆 _left/_right：区分每次记录、持久化、不复用同一文件（旧 /tmp
# 模式重启即丢、多跑互相追加）。
ros2 launch astral_teleop full_teleop.launch.py \
  with_arm_driver:=true right_hand_source:=none with_gripper:=true arm_side:=left \
  log_dir:=<ws>/inference_test_logs/teleop log_tag:=pick_place_test7
#   → <ws>/inference_test_logs/teleop/20260916-153012_pick_place_test7/teleop_teleop_{left,right}.jsonl
# 精确路径模式（旧）：teleop_log_file:=/tmp/teleop_teleop.jsonl（显式路径优先于 log_dir）
# web：System tab 预设启动旁勾选「记录遥操日志」→ 自动落 <ws>/inference_test_logs/teleop/...
```

分析：按 `kind` 过滤 + `t` 对齐；`vr`→`filt`→`cmd` 逐级求速度可定位阶梯产生/抹平在哪一级，
`wrist` 的 `age_ms`/事件间隔看上游是否停帧，`counts.ik_fail`/`hard_fallback` 看 IK 是否偶发失败。

## 遥操不平滑（慢速平移"抖"）排障全记录

**症状**：慢速平移机械臂"停一下走一下"地抖、快速流畅；**不在肘伸直奇异位置**也抖。
**结论**：根因在**驱动层**——电机低速静摩擦粘滑（speed 环 kp=0.04 微弱），与遥操/IK/指令
精度无关。从发现到根因的完整链路（2026-09-14~16），每一步都是可复现的：

### 第 1 步：建遥操 JSONL 诊断日志（本包能力）

`teleop_log_file` 参数 + web「记录遥操日志」开关（见上一节）。真实会话 117.7s 落盘
40K 行，`kind` 区分五类记录。**这一步让"抖"从感觉变成可量化的数据**——否则只能靠
"好像有点抖"猜。

### 第 2 步：真实数据定位（排除上游）

慢速段逐级分析：
- `vr`（原始 VR）36% 时间零速、步长 0.1mm 整数倍 → **Quest App 用 `ToString("F4")` 发位姿**
  （0.1mm 量化）。已修 F4→F7（`astral-tracking`），但**这不是主因**——0.1mm 精度本身够细，
  残留抖动只有 ~10mm/s 高频纹波。
- 实测关节（`kind=state`）30~56% 时间停帧、12~19Hz 粘滑、突发峰值 >1 rad/s，而指令每拍
  都在动 → **指向驱动层**。
- IK/安全/人肘计数器干净（仅 `hard_follow`，无 `ik_fail`/`ik_sat`/`hard_fallback`）→ 排除。

### 第 3 步：SDK 直连复现（驱动层实锤）

`astral_robot_sdk/demos/repro_stickslip.py`：绕过 ROS/IK，从当前位姿用 SDK 直接发
**干净恒定低速三角波**（无任何量化/纹波）→ 实测关节仍 **70% 停帧、21Hz 粘滑、滞后 3.4°**。
pos(0x90) vs pv(0x95 速度前馈) 对照几乎一样（70.7% vs 69.7%）→ **速度前馈无效**，修复路线排除。

### 第 4 步：定位根因（板卡/电机配置）

`read_gains.py` 读电机内部四环 PID 级联：**position(35)→speed(0.04)→iq(2)**，
speed 环 kp=0.04 微弱 → 慢速产不出扭矩/阻尼 → 静摩擦粘滑。板卡 CFG 无可调电流/力矩限制，
只有 MIT_KP/KD（0x0010/11）与摩擦补偿（0x0014/15）。

### 第 5 步：解决方案（⚠ 待实机验证）

| 手段 | 工具 | 说明 |
|---|---|---|
| **主修**：调 speed 环 kp | `set_pid.py --joint 3 --spd-kp 0.2` | 0.1→0.2→0.4 从低往高扫，太高会振荡（电流尖峰/嘎嘎响）；改后重跑 repro 看"低频真实滞后/去量化平段" |
| 补充：静摩擦补偿 | `calib_friction.py --write` | 0xC4 自测→set_friction（0~1.0），先下电放安全位 |
| 补充：MIT 增益 | `repro --kp/--kd` | 下电写，扫位置环增益 |

**验证状态：未实机确认**。调参后的重跑（17:31 stickslip_pos.csv 74.8% 平段 vs 基线 70.7%）
无改善——必须 `set_pid.py` 改完 speed 环后重跑 repro 对照，平段占比显著下降才算修好。

### 工具速查

| 工具 | 位置 | 用途 |
|---|---|---|
| 遥操 jsonl | 本包 `teleop_log_file` | 链路逐级数据（vr/filt/cmd/q/counters） |
| `repro_stickslip.py` | `astral_robot_sdk/demos/` | 驱动层粘滑复现 + pos/pv 对照 + 扫 MIT 增益 |
| `read_gains.py` | 同上 | 读电机四环 PID 级联 + MIT + 摩擦（在线可读） |
| `set_pid.py` | 同上 | 写 speed/position 环 PID（下电） |
| `calib_friction.py` | 同上 | 0xC4 静摩擦自测 + set_friction 补偿（下电） |

**排障顺序建议**：先 jsonl 看 `state` 是否停帧（驱动层嫌疑）→ SDK `repro --mode pos` 复现实锤
→ `read_gains.py` 看 speed 环 → `set_pid.py` 调 → 重跑 repro 对照。不要一开始就调遥操平滑参数
（pos_smoothing 只是掩盖驱动层问题，且加滞后）。

## 外部启动闸门（`require_start_signal`）

**问题**：Quest3 端点 "start stream" 时手得抬起来点按钮，推流一来第一帧腕姿就在按钮位置，旧逻辑把第一帧当 `vr_init`（零点），于是零点错位、机器人一上手就偏。

**新流程**（`require_start_signal:=true`）：

1. 节点启动后**不**自动记 `vr_init`、**不** arm，只跟踪 `vr_current`（PoseProcessor `auto_calibrate=false`）。
2. homing 走完后机器人停在 `init_pose`；你把手摆到与机器人一致的初始位姿。
3. **外部发一个 start 信号** → 节点用**当前** `vr_current` 记 `vr_init` 并 arm，遥操开始。

信号入口（任选）：

```bash
# 全局，双臂同启（推荐）
ros2 topic pub --once /teleop/start std_msgs/msg/Bool '{data: true}'

# 单臂 + 带反馈
ros2 service call /astral_arm_teleop_left/start  std_srvs/srv/Trigger
ros2 service call /astral_arm_teleop_right/start std_srvs/srv/Trigger
```

再发一次 `/teleop/start` = 用当前 pose **重新记零点**（re-center，不解除 arm）。`/teleop/disarm` 仍可暂停，`/teleop/armed` 只 arm 不重记零点（未标定时仍不发指令）。

启动时若仍在 homing 或还没收到腕姿，start 会被拒并告警（等 homing 完成、Quest 推流后再发）。

```bash
# 真机整机：外部启动闸门（yaml 已默认 true，此处 require_start_signal:=true 可省略）
ros2 launch astral_teleop full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true \
  right_hand_source:=quest3
# 手摆好后：
ros2 topic pub --once /teleop/start std_msgs/msg/Bool '{data: true}'
```

默认值：**真机 yaml `require_start_signal: true`**（启动推流后手摆好再发 start）；**sim 同样默认 true**（launch arg 默认 `""` 透传 yaml）。yaml `require_start_signal` 或 launch arg 都可覆盖（launch arg 非空时覆盖 yaml）。

> **启动不再自动归位**（左右 yaml `move_to_init_pose: false`，2026-09-04 起）：节点启动只把
> 控制环拉起来，**不**自动走 init_waypoints → init_pose。要回初始工作位请用 web「工作位」
> 按钮（或 `/teleop/init` / `~/init`，见下节）。置 `move_to_init_pose: true`（或 CLI launch
> 透传覆盖）恢复启动自动归位。
>
> **直接从任意位姿 `/teleop/start` 也安全**：臂节点知道自己没到过工作位（`_at_init_pose`
> 标志），start 时会把机器人原点**重锚到当前实测关节角**（与 `~/prepare_reanchor` 共用重锚逻辑），
> 遥操从实际位姿纯增量开始，不会向启动位 FK 锚点跳变。
>
> **重锚同时重锚臂角参考（2026-09-17 起，方案B）**：HITL 的 `~/prepare_reanchor` 与任意位姿 start
> 的重锚除了 robot_init，还把**人肘参考 `_elbow_dir_vr` 重灌为当前上臂方向**——否则
> `human_elbow_mode=hard` 的第一拍 `solve_hard` 会立刻把臂角摆到操作者当前手臂角度
> （真机实测 reanchor 后 0.7s 内肘部关节重构 0.26-0.32 rad、TCP 不动，即"接管时肘突然
> 换姿势"）。**重锚后默认保持该臂角**（`reanchor_elbow_hold=true`），直到操作者手臂方向
> 相对 reanchor 时刻变化超 `reanchor_elbow_release_thresh`(0.2 rad) 才切回人肘 EMA 跟随——
> 肘不自行摆动（实测方案A 的 EMA 过渡被感知为"卡一下/臂自己在动"）。<0 关闭保持（回方案A）。

## 工作位（去初始位 / go-to-init）

把双臂从**当前位姿**慢速走到**初始工作位 init_pose**，路径：**init_waypoints → init_pose**
（与启动自动归位同一条 init 轨迹机）。动作前先 disarm；命令到每个途经点或最终目标后，
会保持该目标，等待新鲜关节实测进入 `homing_via_tol` 并连续稳定 `homing_via_settle_s`，
确认实体到位后才走下一段或报告完成。等待超过 `homing_init_arrive_timeout`（默认 12 秒）时，
轨迹停在当前目标并保留原 `init_pose`，不会越过未确认的途经点。到 init_pose 后保持，之后可
`/teleop/start` 开始遥操。入口（任选）：

```bash
# 全局，双臂同启（web「工作位」按钮即此）
ros2 topic pub --once /teleop/init std_msgs/msg/Bool '{data: true}'

# 单臂 + 带反馈
ros2 service call /astral_arm_teleop_left/init  std_srvs/srv/Trigger
ros2 service call /astral_arm_teleop_right/init std_srvs/srv/Trigger
```

> 轨迹要求电机上电（位置模式才生效）；与 HOME 不同，本按钮**不**自动调 `~/enable`
> （web 端设计如此）——冷启动/急停后请先「一键就绪」再按工作位。

## HOME（归位到零 / park-to-zero）

把双臂从**当前位姿**安全收回零位，路径：**init_pose → init_waypoints（倒序逐点）→ 零位**，
与启动归位共用同一条慢速 joint-space 轨迹机。动作前先 disarm；期间忽略 VR/`start`；到零位后
保持（关节目标=0）并把机器人原点锚到 FK(零)。之后要再遥操需重新 `/teleop/start`。

> **途经点防切角（实体到位 + 稳定驻留）**：park 轨迹纯开环，命令（q_cmd）会领先实测（板端
> 跟踪误差 + 链路延迟）。若命令一到途经点就立刻反向去下一段，**实体会被"切角"在 waypoint
> 之前掉头**——现象就是 HOME 时臂"伸向 init_waypoints 但没到位就转去零位"（waypoint 是防刮桌
> 的安全走廊位姿，切角 = 走廊失效）。因此 park 在**命令到途经点后钉住重发**，等**实测进入
> `homing_via_tol`（默认 0.12 rad）且连续稳定 `homing_via_settle_s`（默认 0.5 s）**——实体真正
> 停下到位（运动中擦过 tol 会清零计时重来），而不是刚到附近就反向。等待超 `homing_via_hold_s`
> （默认 2 s 总上限）或实测流不可用（按纯命令驻留 settle 时长）则放行。段间运动仍纯开环——
> 不退回"每拍从实测迈步"的阶梯采样。工作位/启动 init 使用相同的到点稳定判定，但超时会停在
> 当前目标，不会像 HOME 的兼容回退那样继续越过途经点。

信号入口（任选）：

```bash
# 全局，双臂同启（推荐）
ros2 topic pub --once /teleop/home std_msgs/msg/Bool '{data: true}'

# 单臂 + 带反馈
ros2 service call /astral_arm_teleop_left/home  std_srvs/srv/Trigger
ros2 service call /astral_arm_teleop_right/home std_srvs/srv/Trigger
```

> 归位需电机上电。急停（真断电）后请先用 driver `~/enable`（使能**不回零**，避免与轨迹目标抢）
> 再发 HOME——web 端 **HOME 按钮**（`POST /api/v1/teleop/home`）已自动做这一步（先 enable 后
> disarm+home）。`~/enable` **幂等**：启动 auto_ready 已使能时直接跳过重复下发，已上电不会误报
> "电机未使能"。注意 web「启动/重启」只拉栈、不再自动 enable（曾导致 503/臂不动，已回退）。

web 端操作：系统页预设管理区 **HOME** 按钮（或 `POST /api/v1/teleop/home`）。

### 手动硬件模式 vs 命令源（阻尼 → 归零/一键就绪的"抢占"坑）

归零/一键就绪是 **driver 一次性目标**，而有两种"持续命令源"会在臂切回 POSITION 后立刻把
**阻尼释放前的旧目标 P** 重新压上来、覆盖归零：

1. driver 100Hz 控制定时器在 `command_timeout_s` 新鲜窗口内重发缓存目标（`astral_robot_driver`
   已在 `~/ready`/`~/home`/`~/damping`/`~/estop`/`~/enable` 内清缓存修复）；
2. 本节点仍在 armed 或**启动归位 homing 进行中**持续发流——本节点已修复：收到 operator
   **disarm（web 暂停 / 急停 / 阻尼 / 就绪 / 归零 / 位置保持都会先发）即取消进行中的
   homing/park**，停止下发陈旧轨迹，臂保持当前目标，等新的 `/teleop/start` / `/teleop/home`。

因此手动拖臂/阻尼后的标准操作是：**web 点「阻尼释放」（自动 disarm）→ 手动拖回 → 「归零」或
「一键就绪」**；之后要用遥操重新点「开始遥操」。CLI 手调请先 `ros2 topic pub --once
/teleop/disarm std_msgs/msg/Bool '{data: true}'` 再调 driver 服务。

   316|## 频率

默认遥操 / 驱动均为 **50 Hz**。有效落板频率 ≈ `min(teleop, driver)`。

## 策略接管

策略节点先调用 `~/prepare_reanchor`：该服务用新鲜 VR 腕姿与实测关节建立锚点并返回一次性 token，
不会开始发控制指令。只有策略状态切到 HUMAN 且携带该节点 token 时，本节点才武装输出；提交时关节或
VR 输入已过期就保持 disarm。策略所有权过期、进入 pending 或收到非法状态时，本节点停止发指令；
重新开放时需要新的 `/teleop/start` 或有效接管 token，避免旧 VR 锚点自动恢复。

## 测试

| 命令 | 作用 | 依赖 |
|------|------|------|
| `ros2 run astral_arm_teleop test_ik_solver` | DH/URDF 离线 FK↔IK、双臂、安全滤波 | 无（URDF 需 pin） |
| `ros2 run astral_arm_teleop test_geometric_ik` | 几何闭式 vs Pinocchio FK、回环、限位、工厂 | pinocchio |
| `ros2 run astral_arm_teleop test_dh_urdf_fk` | 同 q、SW `*_base_link` 下 DH vs URDF（未映射会很大） | pinocchio |
| `ros2 run astral_arm_teleop fit_dh_from_urdf` | 轴几何 / MDH 拟合 | pinocchio |
| `ros2 run astral_arm_teleop test_vr_mapping` | PoseProcessor、TCP | 无 |
| `ros2 run astral_arm_teleop test_dataflow` | 订腕姿跑管线，FPS/IK 统计 | Quest 或键盘 |
| `ros2 run astral_arm_teleop test_safe_teleop` | 保守参数发 `joint_commands` | 可选驱动 |
| `ros2 run astral_arm_teleop keyboard_vr_sim` | 键盘发 `quest3/*_wrist_pose` | 无 |
| `ros2 run astral_arm_teleop body_joints_sim` | 假人臂：发 `quest3/body_joints`+腕姿，测 `use_human_elbow`（scenario: elbow_circle / wrist_spin / combined） | 无 |

对齐 DH 与 URDF 时必须带：**关节 ±q（左 2/3/4、右 2/4）** 和 **干净基座 `Tbinv`**，不能只跑 `test_dh_urdf_fk` 的同 q 直比。

```bash
ros2 run astral_arm_teleop test_ik_solver
ros2 run astral_arm_teleop keyboard_vr_sim --ros-args -p arm_side:=left
ros2 run astral_arm_teleop test_dataflow --ros-args -p arm_side:=left
```

## 小范围跟手调参（曲线）

遥操节点默认发 `/teleop/{left|right}/tune/`：

| 话题 | 含义 |
|------|------|
| `ee_vr` | 手：VR 增量 × `motion_scale`，**未**平滑 |
| `ee_filt` | 滤波后的 IK 目标 |
| `ee_cmd` | 指令关节的 FK 末端 |
| `xyz` | 上面三个位姿打成一条（绘图用） |

仿真起来后另开终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run astral_arm_teleop teleop_tune_plot --ros-args -p arm_side:=right
```

窗口里：**蓝=手、橙虚线=滤波、绿=末端**。先静止按 `z` 清零，再做 1–2 cm 来回。

左下角 **DH analytic / URDF LM** 可热切换（切完重定 VR 零点，臂停在当前姿态）。  
左侧滑条两套 IK 都生效：`pos_smooth` / `rot_smooth` / `motion_scale` / `max_vel` / `workspace_radius`。  
右侧滑条仅 URDF LM：`ik_w_pos` / `ik_w_ori` / `ik_max_iter` / `ik_tol` / `ik_w_reg` / `ik_q4_max` / `ik_w_limit`。DH 是闭式，改右侧无效。

| 曲线 | 说明 |
|------|------|
| 绿贴蓝 | 跟手 |
| 静持时绿抖、蓝平 | IK/控制抖 |
| 蓝自己抖 | Quest 噪声 |
| 绿圆滑但落后蓝 | `pos_smoothing` 太大 |
| 蓝幅值对、绿跟不上 | 限速 / IK |
| URDF 位置跟、姿态飘 | 加大 `ik_w_ori` |
| URDF 整臂拧出怪姿势 | 减小 `ik_w_ori`，或先确认 `ik_w_reg` 够大 |
| URDF 发黏、小动作被吃 | 减小 `ik_w_reg` |
| URDF 构型乱跳、姿势怪 | 加大 `ik_w_reg`（默认 1e-4；贴上一帧时曾用 0.02） |

右侧数字：`pos err RMS/p95`、静持 `hold jitter`、`lag`。`s` 存 CSV 到 `/tmp`。yaml 只是下次启动的初值。

也可用 PlotJuggler 订 `ee_vr` / `ee_cmd` 的 `pose.position.{x,y,z}`。

## 参数

单臂：`config/astral_arm_teleop_{left,right}.yaml`。

常用：`solver_type`、`urdf_path`、`motion_scale`（VR 位姿增量缩放，yaml 当前 1.0=1:1）、`vr_to_arm_rot`（yaml 默认 I；DH 时节点自动乘 `R_baseᵀ`）、`init_pose`（旧约定）、`init_waypoints`（扁平 7×N 途经点，硬件约定，先按序走到再去 `init_pose`；空=直接到 init）、`move_to_init_pose`（启动低速走到 `init_pose`，默认开）、`init_speed_percent`（默认 10，相对 `max_joint_vel`）、`max_joint_vel`（rad/s）、`workspace_radius`（末端相对 `*_base_link` 原点的球半径，m；yaml 当前 0.65，≤0 关闭；可热改）、`reach_margin`（肘伸直软墙，腕目标钳在肩心 l_se+l_ew−margin，yaml 当前 0.001 m，节点代码默认 0.01，≤0 关闭；可热改）、`ik_q4_max`（URDF IK 肘关节上限，默认 −0.45；URDF 硬件上限是 0=伸直奇异，≥0 关闭此帽）、`ik_w_fold` / `ik_q4_fold`（单向折肘，比 fold 更直才罚）、`ik_w_limit`（近限位软约束；短边按到 0 的余量缩 margin）、`ik_w_ori`（姿态权重，默认 0.3）、`ik_w_reg`（贴上一帧，默认 1e-4）、`ik_dq_max`（每拍关节步进盒 rad，默认 0=关）、`ik_w_pref`（冗余时往 init_pose 靠，默认 0=关）、`pos_smoothing` / `rot_smoothing`（0–1，按 50 Hz 标定，与 `control_rate` 无关）、`require_start_signal`（真机 yaml 默认 true、sim launch 默认 false；true 时等外部 `/teleop/start` 或 `~/start` 服务记 `vr_init` 并 arm，见上节）。

`use_joint_state_seed`：单臂节点会订 `joint_states` 但控制环目前仍用 `q_cmd` 做 warm-start（开环种子）。数值 IK 同样用上一帧 `q` 作 LM 初值。

`homing_track_state`（默认 true）+ `homing_follow_tol`（默认 0.25 rad）：**HOME park** 段间
纯开环匀速推进**（这是唯一被实机验证的平滑路径）——**不再**"每拍从实测位姿迈步"，那会把平滑位置
控制变成"等实测挪一步才挪一步"的阶梯采样（命令永远只领先实测一个 tick 步长），实机表现 = 慢慢
一卡一卡 + 突然抽一下。反馈只做**跟随便用**：park 运行时实测落后指令超过 `homing_follow_tol`
（电机失能/堵转、没在跟随）→ **冻结轨迹**，q_cmd 不"内部空跑"一路领先到零位；等实测追近
（电机恢复跟随）后自动解除继续收回——领先被钳在 tol 内，不会猛扑。正常跟随稳态滞后远小于
tol，守卫完全不介入。启动 init 归位不受这个逐拍跟随守卫影响，但到每个途经点/最终目标仍须实测
确认到位。关掉
`homing_track_state` 则 park 也完全退化为纯开环（无冻结守卫）。

`homing_via_tol`（默认 0.12 rad）+ `homing_via_settle_s`（默认 0.5 s）+ `homing_via_hold_s`
（默认 2.0 s）：HOME park **途经点防切角门限**（见上节）。命令到途经点后钉住重发，等**实测
进入 `homing_via_tol` 且连续稳定 `homing_via_settle_s`**（实体真到位停下）才推进下一段——
运动中擦过 tol 会清零重计；等待超 `homing_via_hold_s` 总上限、或实测流不可用（按纯命令驻留
settle 时长）则放行。改小 tol / 改大 settle_s = 途经点贴得更死（更慢、更"到过位姿"）；
改大 tol / 改小 settle_s = 更宽松（更快）。正常跟随且到位即停时门限只加 ~settle_s 的驻留。

`homing_init_arrive_timeout`（默认 12 s）：工作位/启动 init 命令到目标后，等待关节实测到位的
单目标上限。超时就保持在该途经点并终止本次路径，不把实体尚未到达的点当成已通过；重试前检查
电机使能和关节状态反馈。HOME 的 `homing_via_hold_s` 仍保持原有超时放行语义。
