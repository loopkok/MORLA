# 遥操慢速"抖"（低速粘滑）排障全记录

> 2026-09-14~16 真机实测排障，2026-09-22 整理归档。
> 给新会话/接手者：理解"慢速平移一卡一卡"这类现象**为什么根因在驱动层**，
> 以及正确的排障顺序（先量化、再逐层排除、最后实锤）。
> 母本：`src/robots/astral/astral_arm_teleop/CLAUDE.md`「低速粘滑排障全记录」+ `README.md`「遥操不平滑排障全记录」+
> `inference_test_logs/teleop/SUMMARY.md`（原始数据）。与
> `src/robots/astral/astral_arm_teleop/doc/2026-09-17-ik-solver-comparison.md`（求解器对比，死区交互）互补。

## TL;DR（一分钟结论）

**慢速平移"停一下走一下"地抖、快速流畅，根因在驱动层**——电机低速静摩擦粘滑
（speed 环 kp=0.04 微弱，慢速产不出扭矩/阻尼），与遥操/IK/指令精度**无关**；
速度前馈（0x95 PV）无效。**⚠ 修复（调 speed 环 kp）尚未实机验证**。

| 层 | 证据 | 结论 |
|---|---|---|
| 上游 VR 精度 | 36% 零速、0.1mm 量化（Quest App `F4`，已修 F7） | 真实缺陷但非主因 |
| 遥操/IK | IK/安全/人肘计数器干净 | 排除上游 |
| **驱动层（实锤）** | SDK 直连干净三角波仍 70% 停帧、21Hz 粘滑、滞后 3.4° | **根因** |
| 速度前馈 | pos(0x90) vs pv(0x95) 70.7% vs 69.7% | 无效，排除修复路线 |
| 根因细节 | `read_gains.py`：position(35)→**speed(0.04)**→iq(2) | speed 环 kp 微弱 |

## 五步排障链路（每一步都可复现）

### 第 1 步：建遥操 JSONL 诊断日志（把"抖"变成数据）

`teleop_log_file` 参数 + web「记录遥操日志」开关。真实会话 117.7s 落盘 40K 行，
`kind` 区分 `loop/wrist/state/body/metrics`（详见 `astral_arm_teleop/README.md`「遥操诊断 JSONL 日志」节）。
**没有这步只能靠"好像有点抖"猜。**

### 第 2 步：真实数据定位（排除上游）

- `vr`（原始 VR）**36% 时间零速**、步长 0.1mm 整数倍 → Quest App `HandLandmarkStreamer` 用
  `ToString("F4")` 发位姿（0.1mm 量化）。已修 F4→F7（`astral-tracking`），**但非主因**
  （0.1mm 精度本身够细，残留仅 ~10mm/s 高频纹波）。
- 实测关节（`kind=state`）**30~56% 时间停帧、12~19Hz 粘滑、突发峰值 >1 rad/s**，而指令每拍都在动
  → **指向驱动层**。
- IK/安全/人肘计数器干净（仅 `hard_follow`，无 `ik_fail`/`ik_sat`/`hard_fallback`）→ 排除上游。

### 第 3 步：SDK 直连复现（驱动层实锤）

`astral_robot_sdk/demos/repro_stickslip.py`：绕过 ROS/IK，从当前位姿发**干净恒定低速三角波**
（无任何量化/纹波）→ 实测关节仍 **70% 停帧、21Hz 粘滑、滞后 3.4°** → 驱动层实锤。
pos(0x90) vs pv(0x95 速度前馈) 对照几乎一样（70.7% vs 69.7%）→ **速度前馈无效**，修复路线排除。

### 第 4 步：定位根因（板卡/电机配置）

`read_gains.py` 读电机内部四环 PID 级联：**position(35)→speed(0.04)→iq(2)**——speed 环
kp=0.04 微弱 → 慢速产不出扭矩/阻尼 → 静摩擦粘滑。板卡 CFG **无可调电流/力矩限制**，
只有 MIT_KP/KD（0x0010/11）与摩擦补偿（0x0014/15）。

### 第 5 步：解决方案（⚠ 待实机验证）

| 手段 | 工具 | 说明 |
|---|---|---|
| **主修**：调 speed 环 kp | `set_pid.py --joint 3 --spd-kp 0.2` | 0.1→0.2→0.4 从低往高扫，太高振荡（电流尖峰/嘎嘎响）；改后重跑 repro 看平段 |
| 补充：静摩擦补偿 | `calib_friction.py --write` | 0xC4 自测→set_friction（0~1.0），先下电放安全位 |
| 补充：MIT 增益 | `repro --kp/--kd` | 下电写，扫位置环增益 |

**验证状态：未实机确认**。17:31 重跑 `stickslip_pos.csv` 平段 74.8% vs 基线 70.7% 无改善
——必须 `set_pid.py` 改完 speed 环后重跑 repro 对照，**平段占比显著下降（如 <30%）、粘滑频率
明显降低**才算修好。

## 关键数字表

| 文件/指标 | 值 | 含义 |
|---|---|---|
| stickslip_pos.csv（14:53） | 平段 70.7%、21.1Hz、err p95 59mrad | 基线 |
| stickslip_pv.csv（14:53） | 平段 69.7%、20.4Hz、err p95 58mrad | 速度前馈无效 |
| stickslip_pos.csv（17:31） | 平段 74.8%、17.6Hz、err p95 87mrad | 调参未验证 |
| teleop JSONL | VR 76Hz、raw 36% 零速、cmd 10.7mm/s 抖动 | F4 量化残留 |

复现参数：joint=3（左肘pitch）、amp=0.12 rad、vel=0.05 rad/s、tri 三角波、100Hz。
上升/下降段平段对称（70%），顶部（肘伸直、重力矩最大）叠加 1.5s 冻住、电流顶满（iq≈0.05）
→ 重力+扭矩余量薄。

## 排障顺序建议（checklist）

1. 先开 `teleop_log_file` 看 `kind=state` 是否停帧（驱动层嫌疑）；
2. 是 → SDK `repro_stickslip.py --mode pos` 复现实锤；
3. `read_gains.py` 看 speed 环 kp；
4. `set_pid.py` 调 → 重跑 repro 对照；
5. **不要一开始就调遥操平滑参数**（pos_smoothing 只是掩盖驱动层问题，且加滞后）。

## 工具速查

| 工具 | 位置 | 用途 |
|---|---|---|
| 遥操 jsonl | `astral_arm_teleop` `teleop_log_file` | 链路逐级数据（vr/filt/cmd/q/counters） |
| `repro_stickslip.py` | `astral_robot_sdk/demos/` | 驱动层粘滑复现 + pos/pv 对照 + 扫 MIT 增益 |
| `read_gains.py` | 同上 | 读电机四环 PID 级联 + MIT + 摩擦（在线可读） |
| `set_pid.py` | 同上 | 写 speed/position 环 PID（下电） |
| `calib_friction.py` | 同上 | 0xC4 静摩擦自测 + set_friction 补偿（下电） |
| `quantify_cmd_state.py` | `MORLA/scripts/` | 训练数据"意图 vs 摩擦"量化（raw cmd vs state 分型） |

## 下游影响：训练数据平段以摩擦为主（2026-09-16 实测）

对 `astral_data/raw/pick_place_merged`（100 episode）跑 `quantify_cmd_state.py`——把驱动层
粘滑的影响量化到训练数据上（这正是推理侧"固定卡点"的数据源，详见
[data-quality-investigation.md](data-quality-investigation.md)）：

| 指标 | 中位 | 含义 |
|---|---|---|
| **state 平段** | **34.9%** | 实际位置 ~35% 时间"停住" |
| **cmd 平段** | **9.3%** | 操作意图只有 ~9% 时间停顿 |
| **摩擦份额**（state−cmd） | **25.6pp** | ~1/4 轨迹是"被命令在动、实际却粘住" |
| 粘滑频率 | 11.7Hz | 与驱动层复现（12-21Hz）一致 |
| **停顿分型** | 1316 个 = 意图 **13%** / **摩擦 38%** / 混合 49% | 摩擦仍是主导 |
| **分型前对齐** | `--align`（默认开），cmd 领先 state ~120ms | 不对齐时短意图停顿被错分 |

**结论**：next-state 动作标签里 ~25pp 是摩擦导致的假停顿（action≈0），不是操作者犹豫。
→ 标签平滑 / 切 command-source 可捞回 ~25pp "丢失动作"；13% 意图停顿是任务真实节奏，应保留
（repair `--keep-intent`）。

## 关联

- **求解器死区交互**：`src/robots/astral/astral_arm_teleop/doc/2026-09-17-ik-solver-comparison.md`——
  geometric 精确解+最小关节速度优化，慢速时把指令压到电机死区（~1mrad）下 → 停-跳
  （24% 拍整臂 7 关节全 <1mrad）；urdf 的"浪费型"运动反而平滑。`cmd_deadband_mrad` 实测无效。
- **数据修复**：`astral_ws/src/common/morla_data_collect/morla_data_collect/pipeline/repair_aligned.py`（保时序摩擦移除 / 弧长匀速化），见
  [data-quality-investigation.md](data-quality-investigation.md)。
