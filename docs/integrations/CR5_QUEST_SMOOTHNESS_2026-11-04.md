# CR5 Quest 遥操顿挫：三轮真机量化结果与指令积分修复

日期：2026-11-04。范围：Quest 右手柄连续跟随真机 CR5（`cr5_real_quest` 装配）。本文只记录数据、口径和修复结论；实机手感验收仍需现场复测。

## 结论（先读）

1. **24 Hz 指令阶梯已消除。** 新反馈样本落地那一拍与其余拍的平均步长比从 **3.85×** 降到 **0.98×**；步长信号中 20–26.5 Hz 频段能量占比从 **46.4%** 降到 **1.70%**；步长 FFT 主峰从 **22.4–22.5 Hz**（30004 反馈频率）变为**无主峰**（0.0–0.1 Hz）。
2. **微分 IK 不再每拍打满。** 单拍关节更新中位从 **2.292°（= `differential_step_rad` 0.04 rad 上限）** 降到 **0.476°**，只有 p99 = 2.292°；`position_lead_limited` 7.8% → **0.0%**，`rotation_lead_limited` 11.8% → **0.1%**，`absorbed_position_m` 中位 24.2 mm → **0.0 mm**；IK 判定 converged 的拍从 4.7% 升到 **77.9%**。
3. **新增的防 windup 护栏没有参与正常工作。** 指令领先实测最大 **13.49°**，`command_lead_rad = 0.3 rad (17.2°)` 的夹紧触发 **0 / 7615 拍**。也就是说它不会把 24 Hz 抖动带回积分基准，只在臂被挡或伺服不跟时兜底。
4. **延迟没有变化。** 指令→实测的最优延迟三轮都是 **约 195–208 ms**（本次 206–208 ms，六轴一致），由 [transport.py](../../src/robots/dobot/morla_cr5_driver/morla_cr5_driver/transport.py) 里写死的 `ServoJ(..., t=0.100000, lookahead_time=50.000000, gain=500.000000)` 决定，不在本次修复范围。

## 数据来源

| 轮次 | 日志目录（`~/.local/share/morla/logs/`） | 参数要点 | 时长 | 指令数 | 指令频率 | 设备反馈 |
|---|---|---|---|---|---|---|
| run1 | `20261104T044318Z_cr5_real_quest_a21ee3d34…` | `max_joint_velocity 0.3`、`smoothing_alpha 0.35`、`position_scale 0.8` | 72.2 s | 4927 | 68.3 Hz | ~23.8 Hz |
| run2 | `20261104T055945Z_cr5_real_quest_9070f2953…` | `max_joint_velocity 0.8`、其余同上 | 95.4 s | 6904 | 72.4 Hz | ~23.2 Hz |
| run3 | `20261104T064042Z_cr5_real_quest_2e789513…` | `max_joint_velocity 1.0`、`smoothing_alpha 0.6`、`position_scale 0.8`、`command_lead_rad` 用默认 0.3；装配 revision 3 | 93.5 s | 7619 | 81.5 Hz | ~22.7 Hz |

run1/run2 为修复前，run3 为修复后（含 `command_seed` 诊断字段）。run1→run2 只改了速度上限，run2→run3 同时包含修复与速度/滤波参数变化，因此**绝对步长不可直接横比**，可比的是比值、频谱、限幅命中率这类与速度无关的指标。

## 统计口径

- 一条「指令」= `command_request` 中 `producer == "human_mobile"` 的 `commands[0].values`（弧度），与 `control` 拍一一对应。
- 「新样本拍」= 该控制拍使用的 `feedback.sample_ns` 与上一拍不同（`control.feedback.sample_ns`）。异步 worker 使第 i 拍消费的新样本先进入第 i 拍的解、在第 i+1 拍下发，因此 `step[i] = |cmd[i+1] − cmd[i]|∞` 归因到第 i 拍。
- 剔除 `dt ≥ 50 ms` 的指令断档（run3 有 1 处 8.9 s，run1/2 各有 2–3 处）。
- 频段能量：取最长连续段，步长序列去均值后做 FFT，`20–26.5 Hz` 能量 / 全部 AC 能量。
- 延迟：把指令线性插值到实测时间戳，扫描 −60…+400 ms，取 RMSE 最小处；同时给出 0 ms 处的 RMSE 作为对照。
- 领先量（仅 run3 有该字段）：`lead = max|command_seed − feedback_seed|`，单位 rad/deg。

## 三轮量化对比

### 阶梯特征

| 指标 | run1 (v=0.3, 修复前) | run2 (v=0.8, 修复前) | run3 (v=1.0, 修复后) |
|---|---|---|---|
| 新样本拍步长 均值 / 中位 | 0.166° / 0.187° | **0.356° / 0.382°** | **0.293° / 0.244°** |
| 其余拍步长 均值 / 中位 | 0.104° / 0.085° | 0.092° / 0.049° | 0.298° / 0.245° |
| 新样本拍 / 其余拍 均值比 | **1.59×** | **3.85×** | **0.98×** |
| 新样本拍占总行程 | 35% | 56% | 25%（≈拍数占比） |
| 步长自相关 lag1 / lag2 / lag3 / lag4 | 0.02 / −0.02 / 0.01 / 0.05 | 0.01 / −0.11 / −0.06 / **0.77** | 0.89 / 0.81 / 0.75 / **0.70**（平滑衰减） |
| 步长 FFT 主峰 | 22.4 / 22.4 / 22.3 Hz | 22.4 / 22.5 / 22.5 Hz | 0.0 / 0.1 / 0.0 Hz |
| 20–26.5 Hz 频段能量占比 | 17.0% | **46.4%** | **1.70%** |
| p99 / p50 步长比 | 1.75 | 6.54 | 2.93 |
| 指令加速度 RMS（各轴平均） | 0.164°/拍² | 0.153°/拍² | **0.078°/拍²** |

run1 的比值较小是速度上限把大部分跳变摊平了（39.2% 的拍顶在上限），但 22.4 Hz 主峰仍在；run2 放开上限后同样的跳变整段落在一拍里，所以阶梯最明显。run3 在更高速度下已经没有 4 拍周期性：自相关只是随速度缓慢衰减，频段能量降到 1.7%。

### 指令与 IK

| 指标 | run1 | run2 | run3 |
|---|---|---|---|
| 关节步长 中位 / p90 / p99 / max | 0.127 / 0.200 / 0.222 / 0.355° | 0.083 / 0.436 / 0.540 / 0.629° | 0.245 / 0.626 / 0.717 / 0.921° |
| 顶到速度上限的拍 | 39.2% | 6.3% | 15.9% |
| IK 单拍关节更新 中位 / p99 | 2.292 / 2.292° | 2.292 / 2.292° | **0.476 / 2.292°** |
| `position_lead_limited` | 8.9% | 7.8% | **0.0%** |
| `rotation_lead_limited` | 10.3% | 11.8% | **0.1%** |
| `absorbed_position_m` 中位 / p90 | 40.5 / 125.5 mm | 24.2 / 41.9 mm | **0.0 / 0.0 mm** |
| IK reason：converged / tracking / absorbed-hold | 11.1% / 81.1% / 7.7% | 4.7% / 91.3% / 3.9% | **77.9% / 7.9% / 14.1%** |
| IK 解算 sample→completed 中位 / p90 / max | 2.9 / 3.5 / 7.8 ms | 2.9 / 3.5 / 8.6 ms | 3.0 / 3.6 / 9.8 ms |

修复前 IK 每拍都被 `differential_step_rad` 顶满（中位数就等于上限），说明每一步都在"从旧实测追一整个 2.29°"；修复后中位 0.476°，吸收窗口也不再经常触发。

### 领先量与防 windup 护栏（run3 新增诊断）

| 指标 | run3 |
|---|---|
| `command_seed − feedback_seed` 中位 / p90 / p99 / max | 3.77° / 9.81° / 12.60° / **13.49°** |
| 触及 `command_lead_rad = 0.3 rad (17.2°)` 夹紧的拍 | **0 / 7615（0.00%）** |

默认值依据：修复前指令领先实测的最大值在三轮里为 0.125 rad（7.18°），run3 在 1.0 rad/s 上限下也只有 0.235 rad（13.49°）；0.3 rad 留有余量，正常跟随时不触发，只有臂被挡或伺服明显不跟时才会生效。

### 延迟与跟踪残差

| 轴 | run1 最优延迟 / RMSE | run2 最优延迟 / RMSE | run3 最优延迟 / RMSE | run3 0 ms RMSE |
|---|---|---|---|---|
| J1 | +220 ms / 0.41° | +196 ms / 0.51° | +208 ms / 0.27° | 1.28° |
| J2 | +220 ms / 0.11° | +210 ms / 0.27° | +206 ms / 0.32° | 1.18° |
| J3 | +220 ms / 0.56° | +220 ms / 0.43° | +208 ms / 0.28° | 2.21° |
| J4 | +210 ms / 0.47° | +220 ms / 0.24° | +208 ms / 0.71° | 4.12° |
| J5 | +210 ms / 1.13° | +194 ms / 0.96° | +206 ms / 0.41° | 3.44° |
| J6 | +218 ms / 0.40° | +194 ms / 0.30° | +208 ms / 0.93° | 3.79° |

最优延迟三轮都在 194–220 ms，与 ServoJ `t=0.1` 的设定一致；残差量级相同（0.1–1.1°），说明本轮修复改变的是指令形状，不是伺服跟随特性。

## 根因与修复

- 根因：`DifferentialIKSolver` 每拍都用 30004 实测重新播种。反馈 ~23–24 Hz、控制循环 ~90 Hz，两次样本之间指令冻在旧实测上，样本落地时实测与 `current` 一起跳，解就跳一步。速度上限越高，原本被摊平的跳变越会整段落在一拍里，所以 0.3→0.8 提速反而让顿挫更明显。
- Astral 的先例：`astral_arm_teleop_node` 的 `ik.sync_state(self._flip_q(self.q_cmd), reset_branch=False)`（“Warm-start from last *commanded* q (after vel limit), not raw IK jump”）。
- 修法（详见 [CHANGELOG](../../CHANGELOG.md) 2026-11-04 条）：
  - `DifferentialIKSolver.solve(target, seed=None)`：显式积分基准；
  - `CartesianController.__call__(..., seed=None)`：用 seed 播种/求解，实测只用于真实位姿与诊断；
  - `AsyncCartesianController`：每拍快照 `integration_seed(measured, last_output, command_lead_rad)`，并新增 `command_seed` / `integration_seed` 诊断。

## 验证

- 单元/回归：`morla_cr5_driver` 全部 + `morla_teleop` 的 `test_kinematics.py`/`test_engine.py` **78 passed**；新增用例覆盖显式 seed、陈旧反馈不重置积分基准、`command_lead_rad` 限幅与参数校验。
- 闭环仿真（真实 `DifferentialIKSolver`，22.5 Hz 反馈 / 90 Hz 循环）：修复版「新样本拍」与其余拍平均步长比 **1.00**，旧播种 **0.43**。
- 真机：本文上表（run3）。

## 未完成 / 下一步

1. **ServoJ 延迟**：`t=0.100000` 仍是 ~0.2 s 延迟的主因。参数现已可通过 arm 部署参数 `servo_t` / `servo_lookahead_time` / `servo_gain` 覆盖（默认值与 Dobot 官方 `dobot_api.py` 一致，行为未变）；计划把 `t` 调到 0.02–0.03、`lookahead_time` 20–30，`gain` 先保持 500。改前仍未确认现场固件对这些参数的接受范围与实机稳定性，需按低速小幅、单参数逐轮实测。
2. **手感验收**：本次只有数据层面的结论，操作员尚未在 run3 之后给出主观评价。
3. **`command_lead_rad` 未进模板**：当前用代码默认 0.3；若要现场可调，需要加进绑定参数与 Web 参数表（会改变解析快照，需重新保存装配）。

## 复现

分析口径对应的脚本逻辑：按 `command_request(producer=human_mobile)` 与 `control` 记录按时间排序后一一对齐；用 `control.feedback.sample_ns` 的变化标记新样本拍；剔除 `dt ≥ 50 ms`；FFT 取最长连续段。任何自写脚本都应先复现 run2 的 3.85× 与 22.4 Hz 主峰，再去看 run3 的 0.98× 与 1.70%——否则说明口径不一致（例如把归因错位到下一拍，会得到 0.47× 的相反结论）。
