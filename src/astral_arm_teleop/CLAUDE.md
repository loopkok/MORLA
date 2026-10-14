# CLAUDE.md — astral_arm_teleop IK 工程

Quest3 腕部 → 双臂 IK → `/left_arm|/right_arm/joint_commands`。本文件面向新会话/新接手者：先读这里，再进 `README.md`（完整细节）和 `CHANGELOG.md`（按天演进史，含每个问题的症状→根因→修法）。

> **求解器对比测试**（geometric vs urdf_numerical 慢速"一顿一顿"根因）详细记录见
> `doc/2026-09-17-ik-solver-comparison.md`，数据/脚本在 `astral_test_logs/2026-09-17_ik_solver_comparison/`。
> 一句话：电机死区(~1mrad)×geometric 最小关节速度优化 → 慢速指令压死区下 → 一顿一顿；
> `cmd_deadband_mrad`（指令最小步长地板，0=关）可缓解。
> **独立整理版**：慢速平移"抖"排障全记录（五步链路 + 数值 + 工具）→ `astral_ws/docs/teleop-stick-slip-investigation.md`；
> 驱动层"电机突然自己动"定责 → `astral_ws/docs/motor-control-pitfalls.md`。

## 当前状态（2026-09-03）

- **默认求解器 = `geometric`**（免 DH 臂角闭式 IK），左右 yaml 与节点默认一致；
- **`use_human_elbow = true`** 默认开：Quest IOBT 肩/肘数据 → 人臂角 psi_ref，机器人臂姿跟人；
- **`human_elbow_mode = "hard"`（2026-09-04 起 yaml 默认）**：人肘方向新鲜且门控通过时
  `solve_hard` **精确解在 psi_human 上**（肘严格贴人，无软混合/网格/逃逸）；不可行/奇异/
  超龄期自动回退软先验（`hard_fallback` 计数）。硬模式是给 astral_pim_ik 录训练数据的
  前提——执行的 q 的臂角 == 喂入的人臂角（肘方向偏差 <0.02° 已测）。
- 仿真（MuJoCo）与真机走同一套 teleop 代码，只差下游（sim node vs astral_robot_control）；
- 关节限位（URDF 双文件 + MJCF + analytic.py 四处一致）：
  J1 ±2.0 / J2 [−0.3, 2.0]（右臂镜像） / **J3 ±2.2689(±130°)** / J4 [−2.26, 0] / **J5 ±1.7802(±102°)** / J6 [−0.8, 0.85] / J7 ±1.57。

## 文件地图（ik/ + 节点）

```text
ik/geometric.py        ← 核心。extract_arm_geometry（URDF→S/E/W 几何+POE）+ 臂角闭式解
ik/analytic.py         ← DH 闭式（旧路径，硬编码 MDH + flip_q 约定）+ ContinuityParams/State 共享
ik/urdf_solver.py      ← Pinocchio LM 数值解（旧默认，仍可切）
ik/factory.py          ← make_single_arm_ik：solver_type → 求解器；所有参数从节点透传
astral_arm_teleop_node.py  ← 150Hz 控制环：pose 映射→先验→IK→safety→发布；参数回调热改
pose_processor.py      ← VR delta→EMA/SLERP 滤波（τ=-0.02/ln(α)，与控制率无关）
safety_filter.py       ← NaN 屏障/限位 clip/速度限幅/工作空间球
body_joints_sim.py     ← 假人臂数据源（测 use_human_elbow，scenario: elbow_circle/wrist_spin/combined）
teleop_tune_plot.py    ← 热调 GUI（曲线看跟手性）
test_geometric_ik.py   ← geometric 全量回归（10 例）
test_ik_solver.py      ← DH 套件（改 analytic.py 后必跑）
```

## 架构不变量（改动前先确认不破坏这些）

1. **URDF 是唯一几何真源**。`extract_arm_geometry()` 从 `astral_robot_description/urdf/astral_robot.pin.urdf` 现场提取轴/点/限位，SRS 残差 >2mm 直接 raise。改几何只改 URDF；所有下游（IK/safety/sim）自动跟随。别在代码里写死几何（曾评估过 baked 常量方案，明确要求配防漂移测试才可行，暂未做）。
2. **geometric 输出即硬件约定**，无 flip_q；只有 `analytic_dh` 需要 flip（节点内自动）。
3. **限位改动必须四处同步**：`astral_robot{,.pin}.urdf` + `astral_arm{,.pin}.urdf` + `astral_dual.xml`(range+ctrlrange) + `analytic.py` joint_limits。漏一处就会出现"仿真正常、DH 切换后怪异"类问题。
4. **QoS：腕位/关节/指令流全部 BEST_EFFORT + depth=1**（`_sensor_qos`）。Jetson 实测 depth=10 时队列积满导致 IK 恒解 ~110ms 前的旧帧（vr_rx≈105ms = 10×11.1ms 帧间隔的整数倍，这是指纹）。仿真 PC 上 loop 快、队列积不起来，depth 无感——**别因为"仿真没问题"回退 QoS**。
5. **评分权重序**：`w_psi_ref=2.0` 必须压过 `w_vel=1.0`/`w_theta0=0.15`，否则人肘先验失效。改权重前想清楚对抗关系。**`human_elbow_mode="hard"` 时绕过评分混合**（精确解在 psi_human），权重只在软回退时生效。

## 已解决的坑（症状 → 根因 → 现有防护）

| 症状 | 根因 | 防护/参数 |
|---|---|---|
| 肘伸直时整臂"卡一下" | 伸直奇异 ψ 无定义 + q4 撞 0 限位 + 目标超可达域→IK 无解→hold→跳变 | `reach_margin` 软墙（钳到 l_se+l_ew−margin）+ ψ 冻结（sinα<0.05 时网格塌缩为 θ0_prev）；计数 `reach_clip` |
| 大 roll 时肩"抽搐又拉回"（振荡） | 腕限位边界处局部窗逐帧空→同帧全局逃逸→下一帧先验窗恢复又拉回 | 逃逸迟滞：`ik_escape_after_frames=4` / `ik_return_after_frames=20`（节点参数可热改）；单次逃逸仍会"抽一下+平滑拉回"，这是设计行为，`psi_escape` 计数+WARN 可观测 |
| 手臂放下时肘拧 90° | 人臂近伸直时 IOBT 肘偏置被归一化成满权重错误先验 | 伸直度门控 `human_elbow_min_sin=0.15`（sin<d 方向与 SW 轴即关先验）；计数 `psi_off` |
| 人转腕肩也跟着乱转（最早的问题） | 冗余自由度无姿态约束 | use_human_elbow 整套方案 |
| 真机 e2e 278ms、vr_rx≈105ms 恒定 | Jetson loop 8ms>150Hz 预算 6.7ms→订阅回调积压+depth=10 读旧帧 | QoS depth=1 + （建议）Jetson 上 control_rate 降 100 |
| sim e2e 好真机差 | PC loop 2ms 预算充裕，问题全被掩盖 | 看 `[Latency]` 各分项定位：e2e 高+ik/loop 低 → 查 vr_rx/vr_age（上游 Quest/WiFi） |
| `--params-file` 不生效 | ros2 run 节点名与 yaml key 不匹配 | 加 `-r __node:=astral_arm_teleop_left` |
| import rclpy 失败 | miniconda python 抢占 | 显式 `/usr/bin/python3` |
| 慢速平移"抖抖的"、快速流畅 | 驱动层静摩擦粘滑（电机 speed 环 kp=0.04 微弱，慢速产不出扭矩/阻尼）；非遥操/IK 上游 | 排障工具链见下节：遥操 `teleop_log_file` jsonl → SDK 直连 `repro_stickslip.py` 复现 → `read_gains.py` 定位 speed 环 → `set_pid.py` 调 spd_kp / `calib_friction.py` 摩擦补偿 |

## 低速粘滑排障（慢速平移"抖"）全记录

**症状**：慢速平移机械臂"停一下走一下"地抖，快速流畅；不在肘伸直奇异位置也抖。
**结论**：根因在**驱动层**（电机低速静摩擦粘滑，speed 环 kp 微弱），与遥操/IK/指令精度无关。
从发现到根因的完整链路（2026-09-14~16）：

1. **建遥操 JSONL 诊断日志**（本包）：`teleop_log_file` 参数 + web「记录遥操日志」开关，
   kind=loop/wrist/state/body/metrics 五类记录（详见 README「遥操诊断 JSONL 日志」节）。
2. **真实数据定位**：慢速段实测关节 30~56% 时间停帧、12~19Hz 粘滑、突发峰值 >1 rad/s，
   IK/安全/人肘计数器干净 → 排除上游，指向驱动层。顺带发现 VR 位姿 0.1mm 量化
   （Quest App `HandLandmarkStreamer` `F4`）并修 F4→F7（精度瑕疵，非主因）。
3. **SDK 直连复现**（`astral_robot_sdk/demos/repro_stickslip.py`）：绕过 ROS/IK 从当前
   位姿发干净恒定低速三角波 → 实测关节仍 70% 停帧、21Hz 粘滑、滞后 3.4° → **驱动层实锤**；
   pos(0x90) vs pv(0x95) 对照 → 速度前馈**无效**（70.7% vs 69.7%）。
4. **定位根因**（`read_gains.py`）：电机四环 PID 级联 position(35)→speed(0.04)→iq(2)，
   **speed 环 kp=0.04 微弱** → 慢速产不出扭矩/阻尼 → 粘滑。板卡 CFG 无可调电流/力矩限制，
   只有 MIT_KP/KD 与摩擦补偿（0x0010/11、0x0014/15）。
5. **解决方案**（`set_pid.py`）：调 speed 环 kp（0.1→0.2→0.4 从低往高扫，太高振荡）；
   补充 `calib_friction.py`（0xC4 静摩擦自测→set_friction）与 `repro --kp/--kd`（MIT 增益）。
   **⚠ 验证状态：未实机确认**——调参后重跑 repro 的"去量化平段/低频滞后"下降才算修好
   （17:31 的 pos 重跑 74.8% 平段无改善，待 set_pid 后重跑对照）。

**工具速查**：

| 工具 | 位置 | 用途 |
|---|---|---|
| 遥操 jsonl | 本包 `teleop_log_file` | 链路逐级数据（vr/filt/cmd/q/counters） |
| `repro_stickslip.py` | `astral_robot_sdk/demos/` | SDK 直连驱动层粘滑复现 + pos/pv 对照 + 扫 MIT 增益 |
| `read_gains.py` | 同上 | 读电机四环 PID 级联 + MIT + 摩擦（在线可读） |
| `set_pid.py` | 同上 | 写 speed/position 环 PID（下电） |
| `calib_friction.py` | 同上 | 0xC4 静摩擦自测 + set_friction 补偿（下电） |

## 延迟指标速查（`[Latency]` 行）

`vr_rx`=Quest→teleop 传输龄期（正常 ~1ms，恒定 100ms+ = 时钟未对齐或积压）；`vr_age`=控制环取数时数据龄；`ik`/`loop`=求解/整环耗时（PC 1-3/2-4ms，Jetson 6.9/8ms）；sim 侧 `e2e`=teleop 盖章→sim 收到，`apply`=sim 内排队。计数器：`ws_clip`/`reach_clip`/`ik_fail`/`ik_sat`/`psi_off`/`psi_escape`。

## 测试（改 ik/ 后必须全绿）

```bash
cd /home/robot/loopkok/sdk/astral_ws
PYTHONPATH=src/astral_arm_teleop /usr/bin/python3 src/astral_arm_teleop/astral_arm_teleop/test_geometric_ik.py   # 10 例
PYTHONPATH=src/astral_arm_teleop /usr/bin/python3 src/astral_arm_teleop/astral_arm_teleop/test_ik_solver.py      # DH 套件
# sim 冒烟: ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py
```

测试风格：脚本式函数 + main() 汇总表 + sys.exit(1)，不用 unittest class。新增能力必须带新测试段（先写失败用例复现 bug，再修）。

## 修改流程约定（从历史 commit 沉淀）

1. **先诊断后动手**：把症状在测试里复现（如 test_full_extension 复现伸直卡顿、test_roll_escape_hysteresis 复现 roll 抽搐——用随机种子找可行用例，别手工构造），修复后该用例转正为回归。
2. **防护优先做成"平滑降级"**：不卡死、不跳变、可观测（计数器进 `[Latency]` 行）。任何新防护至少给一个可热改参数 + 一个日志/计数。
3. **参数化路径**：求解器内部常量若需真机调，提成节点参数（工厂透传），如 escape_after_frames；`ContinuityParams` 是 DH/geometric 共享 dataclass，加字段给默认值保持 DH 侧行为不变。
4. **每次实质改动更新** `astral_ws/CHANGELOG.md`（当日日期加粗标题——改动——动机——数值——验证结果）+ 对应包 README 段落。README 的"症状"文档要写用户可感知的表象（"肩部抽搐"），不是内部术语。
5. **数值给实测**：测试输出贴具体数字（如 "flicker jumps old=39 new=0"），不写"测试通过"了事。
6. 用户重视**姿态跟手性**（臂姿像人）与**真机可观测性**；性能优化先看 `[Latency]` 分项再动手，别猜。

## 遗留事项 / 下一步候选

- 固件板端 `lim.min/lim.max`（CfgKey 0x0012/0x0013）与 URDF 新限位（J3/J5 放宽）**尚未核对同步**——板端不放宽的话超出段仍被夹；
- `body_joints_sim` 的 wrist_spin 场景依赖订阅 joint_states 自匹配肘（非阻塞重试），改 teleop 发布名时要同步；
- `test_dataflow`/`keyboard_vr_sim` 依赖 ROS 环境跑，CI 外手动验证；
- geometric 的 `local_theta0_window`(0.15)/`local_theta0_count`(5) 仍是 ContinuityParams 代码级常量，未提成 ROS 参数。
