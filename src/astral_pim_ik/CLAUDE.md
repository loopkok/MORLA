# CLAUDE.md — astral_pim_ik 两段式臂角 IK

把 Astral 机械臂接入 PiM-IK 两段式管线：**神经网络预测臂角 ψ → geometric 免 DH 闭式 IK 解 7 关节**。本文件面向新会话/新机器接手：先读这里，再看 `README.md`（快速开始）和 `docs/adversarial_review.md`（对抗性发现，含实测复现）。

## 1. 项目定位（一句话）

PiM-IK 参考项目（`pim-ik-*/`，G1 专用）用「NN 预测 swivel angle φ → 数值/解析 IK 解关节」。本项目把**第二段替换成 Astral 的 geometric 免 DH 臂角闭式 IK**（`astral_arm_teleop.ik.geometric`），并把 geometric 里为 VR 遥操作而设的一堆约束**全部剥掉**，只留「硬关节限位过滤 + 最小 warm-start 分支选择」的确定性核心。输出是一个独立 ROS 包，不接入 teleop 节点。

## 2. 设计思路

### 2.1 为什么是「两段式」而不是直接回归 7 关节

7-DOF 机械臂有 1 个冗余自由度。直接让网络回归 7 个关节角：难训练、多解歧义、关节限位难保证。PiM-IK 的关键洞察是把冗余自由度**单独拎出来**——「末端位姿 + 臂角」在非奇异时唯一确定整条臂构型，所以网络只需学一个 **2 维输出** `[cos ψ, sin ψ]`，关节角交给闭式/数值 IK 精确解出（位姿误差 <0.001 mm，网络预测的微小误差只影响臂姿不影响末端精度）。

### 2.2 为什么「臂角 ψ」用 2D 单位向量而不是角度

角度有环绕问题（−π 和 +π 是同一角度，直接回归有 discontinuity）。`[cos ψ, sin ψ]` 连续、无环绕，网络最后 L2 归一化天然满足单位圆约束。

### 2.3 为什么复用 geometric（免 DH）而不是 analytic（DH）

- `geometric.py` 的臂角 `psi`（`psi_ref`）与 PiM-IK 的 swivel φ 是**同一个概念**：肘部在肩→腕轴周围轨道圆上的角度。
- geometric 是 **DH-free**：`extract_arm_geometry()` 直接从 URDF 现场提取肩/肘/腕中心、关节轴、home 位姿、限位，用 POE + Paden-Kahan 闭式解，**无 DH 表、无 theta offset、无 flip_q 约定**，输出直接就是 URDF/硬件关节符号约定（与 `urdf_solver` 同框）。`analytic.py` 是旧 DH 路径，硬编码 MDH + 翻转约定，仅作为对比保留。
- 架构不变量（继承自 `astral_arm_teleop/CLAUDE.md`）：**URDF 是唯一几何真源**，改几何只改 URDF。

### 2.4 关键正确性不变量：ψ=0 约定

臂角 ψ 的定义依赖 `geometric._circle_basis_sw` 里的 `psi_ref = S`（肩心向量，指向「肘平面」的零位参考）。**四处必须用同一套约定**，否则 NN 学到的 ψ 与求解器消费的 ψ 对不上（静默错误，比崩溃更危险）：

1. 求解器（`geometric._circle_basis_sw` / `_elbow_point` / `_collect_solutions`）
2. 数据集标签（`geometry.psi_from_config` → `geometric.psi_from_elbow_dir`）
3. 可微运动学层（`kinematics.circle_basis`，torch 向量化重写）
4. 先验（本包暂不使用人类肘先验，故第 4 处当前空）

**做法**：1/2 直接复用 geometric 原语保证一致；3 是 torch 重写，与 numpy 版**必须逐位一致**（`test_network.py::test_loss_and_convention` 有 numpy-vs-torch 肘点一致性断言，**已实测 err=9.58e-07 m，F2 关闭**）。改 3 的几何公式时，同步核对 `geometric._circle_basis_sw`。

### 2.5 为什么 stage-1 训练数据必须是「时间相干」的轨迹（2026-09-04 实测发现）

给定单个 `T_ee`，ψ **不可观测**：肘在 S-W 轨道圆上自由，任一可行 ψ 都达到同一腕部位姿。窗口网络只能从「腕部在窗口内的运动模式」反推肘——前提是数据里帧间真有运动关联。`generate_dataset` 的 IID 随机采样**没有**这种关联，实测：IID 数据上 ψ 误差卡在 ~53° 无论怎么训（loss 平台 ~1.15）；换成平滑漂移轨迹（`generate_trajectory_dataset`，ψ 每帧仅漂 ~0.5°）同一模型即可显著下降。**训练必须用 `generate_trajectory_dataset`（或真机遥操录制的轨迹数据）；`generate_dataset` 只用于 stage-2 标签测试。**

## 3. 核心架构 & 数据流

```text
Stage 1 (NN, torch)                Stage 2 (deterministic, numpy/pinocchio)
T_ee (B,W,4,4)                     (T_ee, psi)
  └ transform_to_9d → (B,W,9)        └ _compute_sw → S, W, q4 分支
  └ Stem(Linear+Conv1d+GELU)         └ _collect_solutions(T, g, S, W, q4l, [psi])
  └ Backbone(transformer/mamba/lstm)   └ 硬限位过滤 + min-q4/warm-start 选支
  └ Head(MLP) → L2 归一化              └ q (7,)
  = [cos ψ, sin ψ]
```

- 帧间：`PiMIKPipeline.solve_trajectory` 逐帧 `solve(T_k, ψ_k, q_init=上一帧解)`，warm-start 保证轨迹连续（等价 PiM-IK `HierarchicalIKSolver` 的 warm-start，**不是** teleop 的加权连续性打分）。
- 训练标签：`generate_trajectory_dataset` 采样平滑关节漂移轨迹 → FK 得 T_ee → `psi_from_config(q)` 得 ψ → 存 .npz；`PhysicsInformedLoss` = `w_psi·L_psi(L1) + w_elbow·L_elbow(RMSE 经可微层) + w_smooth·L_smooth(Jerk)`，乘 `is_valid` mask。
- OOD 兜底（F1，2026-09-04 实现）：`solve(..., fallback_scan=True)` 在请求 ψ 不可行时网格扫描最近可行 ψ；**默认关闭**，救援计入 `solver.fallback_count`。

## 4. 文件地图

```text
astral_pim_ik/                    ← ROS 包根（package.xml 在 src/astral_pim_ik/，标准布局）
  geometry.py    ← 核心第二段。GeometricArmAngleSolver（确定性 (T_ee,ψ)→q）+ psi_from_config/elbow_position + F1 fallback_scan
  network.py     ← PiM_IK_Net 移植（torch；9D 位姿→[cosψ,sinψ]，backbone 默认 transformer）
  kinematics.py  ← DifferentiableKinematicsLayer + PhysicsInformedLoss（Astral ψ=S 约定）
  dataset.py     ← generate_dataset（IID，仅 stage-2 标签测试）+ generate_trajectory_dataset（时间相干，stage-1 训练用）+ SwivelSequenceDataset
  pipeline.py    ← PiMIKPipeline（两段式推理）+ SyntheticPsiPredictor（免 torch 测 e2e）+ TorchPsiPredictor
  train.py       ← DDP 训练脚本（torchrun；镜像 PiM-IK trainer）
  eval.py        ← 训练后评估：滑窗预测 ψ → 几何 IK → ψ/肘部/位姿/Joint MAE/成功率（NN vs oracle；--no_split 整段评估）
  convert_sessions.py ← 遥操录制 robot_data.h5 → 训练/val npz（整段留出；读 {side}_arm_state 实测 q）
  test_deterministic_solver.py / test_dataset.py / test_pipeline.py / test_network.py
  config/astral_pim_ik.yaml   ← 默认超参（不参与 ROS 运行时，仅文档）
  docs/adversarial_review.md  ← 对抗性发现（F1..F8 + 2026-09-04 新增 3 条）
```

**依赖关系（移植到新机器必读）**：本包 `import astral_arm_teleop.ik.geometric`（`extract_arm_geometry`/`_circle_basis_sw`/`_elbow_point`/`_collect_solutions`/`_compute_sw`/`psi_from_elbow_dir`/`_axis_angle_rot`/`fk`）和 `astral_arm_teleop.ik.analytic.wrap_to_pi`。**所以 `astral_arm_teleop` 包必须同机可 import**（PYTHONPATH 或 colcon install 后），且 `extract_arm_geometry` 需要 pinocchio + 能解析到 URDF（`factory.default_astral_urdf_path()` → `astral_robot_description/urdf/astral_robot.pin.urdf`）。

## 5. 已完成的 & 已测试的（截至 2026-09-04，本机环境）

- 离线测试：`/home/robot/miniconda3/envs/arm_sdk`（pinocchio 3.8.0 / numpy 2.2.6 / torch 2.13.0+cpu）
- 训练/评估：`/home/robot/miniconda3/envs/lerobot`（pinocchio 3.4.0 / torch 2.11.0+cu130 / RTX 4090D）

| 测试 | 结果 | 覆盖 |
|------|------|------|
| `test_deterministic_solver.py` | **9/9 PASS** | FK 匹配 Pinocchio URDF(<1e-8)、FK→ψ→IK 回环、ψ 保真、限位、不可达→None、确定性、warm-start 选支、直肘奇异、**F1 OOD fallback（默认关/计数/位姿<1mm）**、**teleop solve_hard ≡ 本求解器（逐位 0.00e+00，双臂 25 样本）** |
| `test_dataset.py` | **3/3 PASS** | schema + ψ 标签回环(位姿<0.07mm)、npz 读写、**generate_trajectory_dataset 时间平滑性** |
| `test_pipeline.py` | **2/2 PASS** | 两段式 e2e 回环(位姿<0.05mm)、warm-start 连续性(max_jump<0.03 rad) |
| `test_network.py` | **4/4 PASS** | 9D 表征 round-trip、网络 forward/L2/梯度、**numpy-vs-torch ψ 约定一致性（9.58e-07 m，F2 关闭）**、**L_elbow 真实肘误差回归（防 W 遮蔽 bug）** |
| `astral_arm_teleop/test_geometric_ik.py` | **10/10 PASS**（回归） | 被复用的 geometric 原语未被破坏 |

### 5.1 首次真实训练闭环实测（2026-09-04，lerobot env / 4090D）

三组对照实验（同一 transformer W=15，5k/20k 帧，30-40 epochs，均 ~1 分钟内）：

| 数据 | ψ 误差(deg) | Joint MAE(deg) | solve 率 | 结论 |
|---|---|---|---|---|
| IID `generate_dataset` 5k | **52.8**（loss 平台 ~1.15） | 18.4 | 70% | ψ 不可学——单帧 T_ee 不决定 ψ，窗口无时间信号 |
| 轨迹 seed5 `travel=0.35` 5k | **15.2** | 5.4 | 88% | 时间相干后可学，seed 间方差大（val 仅 2.5 条轨迹） |
| 轨迹 seed0 `travel=0.35` **20k** | **27.1**（median 19） | 10.0 | 74% | 正式小闭环：预测 std 41.7° vs 真实 43.1°、mean cos=0.826——模型在跟踪，残差是合成数据宽肘先验下的信息瓶颈，非代码缺陷 |
| oracle（用真值 ψ 解 IK） | 0.0 | 0.008 | 100% | stage-2 几何 IK 精确（位姿 <0.06mm） |

**推论**：pose→ψ 的不可观测性（冗余自由度）是核心约束。真机遥操录制数据（人类肘先验 + 慢速自然运动，`astral_data_collect` 链路已能录）预计显著优于合成数据。合成数据上要收紧须先收紧肘先验（如分层采样 ψ 路径而非 q 路径）。

复现命令（产物在 /tmp/pimik_run，可随时再生成）：
```bash
# 数据 → 训练 → 评估
$P -c "from astral_pim_ik.dataset import generate_trajectory_dataset as g; g('left', n_traj=200, traj_len=100, out_npz='/tmp/pimik_run/left_20k.npz')"
torchrun --nproc_per_node=1 astral_pim_ik/train.py --data_path /tmp/pimik_run/left_20k.npz \
    --window_size 15 --batch_size 256 --epochs 40 --save_dir /tmp/pimik_run/ckpt_20k
python3 astral_pim_ik/eval.py --checkpoint /tmp/pimik_run/ckpt_20k/best_transformer_L4_w15.pth \
    --data_path /tmp/pimik_run/left_20k.npz
```

## 6. 未完成内容（接手者先看这里，别当成已完成）

1. ~~torch 代码从未执行~~ → **已执行**：`test_network.py` 4/4；执行即暴露 3 个真 bug（见 §5.2），均已修 + 回归测试。
2. ~~F2 numpy-vs-torch 断言未跑~~ → **已跑并关闭**：肘点一致性 9.58e-07 m < 1e-5。
3. ~~从未真实训练~~ → **已小规模闭环**：见 §5.1。全量/真机数据训练待做。
4. **未接入 teleop 节点**：`GeometricArmAngleSolver` 保持了 `solve/fk/lower_limits/upper_limits/nq` 同形 API，但**没有**加进 `astral_arm_teleop_node` 的 `solver_type`（用户明确本次范围是「仅独立管线 + 测试 + 审查」）。
5. **人类肘先验未接**：PiM-IK 参考里 ψ 也可来自 Quest 肩/肘跟踪（`use_human_elbow`）；本包 Stage 1 只有「NN 从 T_ee 预测 ψ」这一条路，`SyntheticPsiPredictor` 只是测试桩。

### 5.2 首次真实执行暴露并已修复的 bug（记录在案）

| bug | 症状 | 根因 | 修复 + 回归 |
|---|---|---|---|
| 9D 编码行列序错误 | `test_transform_9d` 恒等位姿还原出退化旋转阵 | `transform_to_9d` 把 (3,2) 旋转块行主序拉平得 `[r00,r01,...]`，与 docstring/`rotation_6d_to_matrix` 的列主序 `[r00,r10,r20,r01,r11,r21]` 不符 | `network.py` 拉平前 `transpose`；测试补非恒等旋转 round-trip |
| L_elbow 变量遮蔽 | 训练 loss ~14 且不降，网络几乎不学 ψ | `PhysicsInformedLoss.forward` 里 `B, W, _ = pred_psi.shape` 把形参 `W`（腕部张量）盖成窗口长度 int，肘误差对 int 广播 ~7-13m | `kinematics.py` 改 `B, T, _`；新增回归测试「perfect ψ → L_elbow≈0」（实测 1e-4 m） |
| IID 数据学不动 ψ | IID 数据 ψ 误差 ~53°（loss 平台） | 单帧 T_ee 不决定 ψ，IID 窗口无时间信号 | 新增 `generate_trajectory_dataset`（§2.5）；测试断言轨迹内帧间平滑 |

## 7. 后续计划（按优先级）

1. **真机遥操录制 → 训练（runbook，2026-09-04 链路已就绪）**：
   teleop 已加 `human_elbow_mode: "hard"`（yaml 默认）——硬模式下执行的 q 的臂角
   == 喂入的 ψ_human（肘方向偏差 <0.02° 已测），所以**标签 = 录的 q 离线算**，零额外录制流：
   ```bash
   # 机器人侧：hard 模式遥操，用 astral_data_collect 录 N 段（建议 ≥20 段覆盖不同动作）
   # 4090D 侧：会话 → npz（--val_sessions 整段留出）→ 训练 → 整段评估
   PY=.../envs/lerobot/bin/python3
   $PY astral_pim_ik/convert_sessions.py --sessions ~/astral_data/<task> \
       --side left --out_dir ~/pimik_data --val_sessions <留出会话名>
   torchrun --nproc_per_node=1 astral_pim_ik/train.py --data_path ~/pimik_data/left_train.npz ...
   $PY astral_pim_ik/eval.py --checkpoint ... --data_path ~/pimik_data/left_val.npz --no_split
   ```
   剩余工作 = 数据本身（真机遥操）。NN 无肘跟踪部署 = 常规 `TorchPsiPredictor` 接几何 IK。
2. **OOD 兜底已在（F1 fallback_scan，默认关）**：接真机时打开并观察 `fallback_count`。
3. **接入 teleop 节点（部分前置已落地）**：硬跟肘模式已在 teleop 端实现（`solve_hard`，
   与 `GeometricArmAngleSolver.solve` 逐位一致——见 §5 交叉测试）；把 NN 输出 ψ 作为
   ψ_human 来源接进该模式（策略/回放直接发位姿时），仍是后续项，注意与 `flip_q`、
   `use_human_elbow` 的边界。
4. **人类肘先验（可选）**：PiM-IK 的 `use_human_elbow` 路线在遥操端已活（Quest IOBT），
   训练标签即来自它；本包 Stage 1 是否再接"肘先验输入"通道待部署形态定。
5. **对抗性审查 F3/F4/F5/F6** 长期观察项（见报告，当前均「记录在案」不需改）。

## 8. 对抗性审查发现速查（详见 docs/adversarial_review.md）

| # | 位置 | 严重度 | 状态 |
|---|------|--------|------|
| F1 | `geometry.py: solve` | 高 | 不可行 ψ → None；**已加 fallback_scan（默认关）+ fallback_count** |
| F2 | `kinematics.py` vs `geometric.py` | 高 | ψ=0 约定漂移；**已实测一致 9.58e-07 m，关闭** |
| F3 | 复用 `geometric.py: psi_ref=S` | 中 | 肩心靠近基座原点时 ψ=0 参考退化（本机实测 \|S\|≈0.12 m，两级 fallback 覆盖，全套测试通过）。记录在案 |
| F4 | 复用 `geometric.py: _pk2` | 中 | q2≈±90° 球形奇异静默丢分支。记录在案 |
| F5 | 复用 `geometric.py: _collect_solutions` 去重 | 中 | 2e-4 rad 哈希格。记录在案 |
| F6 | `dataset.py` 直肘门控 | 中 | 近直肘 ψ 弱定义，靠 is_valid 屏蔽。记录在案 |
| F7 | `geometry.py` 冷启动分支 | 低 | min-q4 任意但确定。接受 |
| F8 | 参考 `inference.py` 面板计时 | 低 | 未移植 |
| F9 | `network.py: transform_to_9d` | 高（已修） | 6D 旋转行/列主序错误 → 恒等位姿退化 |
| F10 | `kinematics.py: L_elbow` | 高（已修） | 形参 W 被窗口长度遮蔽 → 假 ~13m 肘误差 |
| F11 | `dataset.py` IID 采样 | 高（已修） | 单帧 T_ee 不决定 ψ → stage-1 需时间相干轨迹 |

## 9. 测试命令（本机）

```bash
cd /home/robot/loopkok/sdk/astral_ws
export PYTHONPATH=$PWD/src/astral_arm_teleop:$PWD/src/astral_pim_ik
PY=/home/robot/miniconda3/envs/arm_sdk/bin/python3    # 离线测试（pinocchio+torch 都齐）
$PY src/astral_pim_ik/astral_pim_ik/test_deterministic_solver.py
$PY src/astral_pim_ik/astral_pim_ik/test_dataset.py
$PY src/astral_pim_ik/astral_pim_ik/test_pipeline.py
$PY src/astral_pim_ik/astral_pim_ik/test_network.py
$PY src/astral_arm_teleop/astral_arm_teleop/test_geometric_ik.py   # 回归，必须 10/10
```

测试风格沿用仓库约定（`astral_arm_teleop`）：脚本式函数 + `main()` 汇总表 + `sys.exit(1)`，不用 unittest class；新增能力必须带新测试段，先写失败用例复现问题再修。

## 10. 修改流程约定（继承自 astral_arm_teleop/CLAUDE.md）

1. **先诊断后动手**：在测试里复现问题再改；数值给实测（如「IID 52.8° vs 轨迹 15-27°」），不写「测试通过」了事。
2. **防护优先「平滑降级」**：不卡死、不跳变、可观测。F1 fallback 是可选参数 + 可观测计数（`fallback_count`）。
3. **改动几何公式必须回到 ψ 约定**：动 `kinematics.circle_basis` 前先对照 `geometric._circle_basis_sw`，并在有 torch 时跑一致性测试。
4. **URDF 是唯一几何真源**：不要在任何地方写死 l_se/l_ew/轴/限位，都从 `extract_arm_geometry` 拿。
5. **torch 形参命名防遮蔽**：`PhysicsInformedLoss.forward` 的形参 `W` 是腕部张量，解包形状别用 `W` 命名（已用 `T`）；同类影子变量 bug 已咬过一次，新 torch 代码注意。
6. **每次实质改动更新本文件 + README +（如涉及发现）adversarial_review.md + astral_ws/CHANGELOG.md**。

## 11. 环境依赖

- 必装：`numpy`、`scipy`。
- 第二段/数据集需要：`pinocchio`（经 `astral_arm_teleop.ik.geometric`），以及可解析的 `astral_robot.pin.urdf`。
- Stage 1 需要：`torch`（`network.py`/`kinematics.py`/`train.py`）；`mamba_ssm` 仅在 `backbone="mamba"` 时。
- 本机：`arm_sdk` env（pinocchio 3.8.0 + torch 2.13.0+cpu）跑全部离线测试；`lerobot` env（pinocchio 3.4.0 + torch 2.11.0+cu130，4090D）跑训练/评估。注意早期文档引用的 `/opt/anaconda3/envs/MujocoSim` 在本机不存在。
