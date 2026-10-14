# astral_pim_ik — PiM-IK 两段式臂角 IK（Astral 机械臂）

按 PiM-IK 的思路（`pim-ik-*/` 参考项目），把 Astral 机械臂接入「神经网络预测臂角 → 免 DH 几何闭式 IK 解关节」的两段式管线：

1. **Stage 1（NN）**：`PiM_IK_Net` 把末端位姿 `T_ee` 的滑动窗口（9D 连续表征）映射成臂角 `ψ` 的单位向量 `[cos ψ, sin ψ]`；
2. **Stage 2（确定性闭式 IK）**：`GeometricArmAngleSolver` 复用 `astral_arm_teleop.ik.geometric` 的 POE + Paden-Kahan 臂角解，由 `(T_ee, ψ)` 解出 7 关节。

与 PiM-IK 参考项目唯一的实质差异是**第二段**：用 Astral 的 **geometric（免 DH）** 臂角闭式替换了 G1 专属的数值/解析 IK，并**剥掉**了 geometric 里为 VR 遥操作而设的一堆约束（逃逸迟滞、`psi_ref` 软先验、伸直冻结、1D QP、连续性打分、reach 软墙），只保留「硬关节限位过滤 + 最小 warm-start 分支选择」的确定性核心。可选 OOD 兜底（`fallback_scan`，默认关闭）在 ψ 不可行时网格扫描最近可行臂角并计入 `fallback_count`。

## 关键不变量：ψ=0 约定

臂角 `ψ` 的定义依赖 `geometric._circle_basis_sw` 里的 `psi_ref = S`（肩心向量）。**四处必须用同一套约定**，否则 NN 学到的 ψ 与求解器消费的 ψ 对不上（静默错误）：①求解器、②数据集标签、③可微运动学层（损失）、④`psi_from_elbow_dir`。本包通过**直接复用** `geometric` 里的 `_circle_basis_sw` / `_elbow_point` / `psi_from_elbow_dir` / `_collect_solutions` 来保证 ①②④ 一致，③ 是 torch 向量化重写并已有 numpy-vs-torch 一致性测试（实测 err 9.58e-07 m，关闭）。

## 关键不变量：训练数据必须时间相干

给定单个 `T_ee`，ψ 不可观测（肘在 S-W 轨道圆上自由）——窗口网络只能从帧间运动反推肘。**训练请用 `generate_trajectory_dataset`（平滑关节漂移轨迹）；`generate_dataset`（IID 随机位形）只能用于 stage-2 标签测试**（实测 IID 上 ψ 误差卡 ~53° 学不动，轨迹数据同模型降到 15-27°）。

## 模块

```text
astral_pim_ik/
  geometry.py    GeometricArmAngleSolver（确定性第二段，含 fallback_scan）+ psi 标签辅助
  network.py     PiM_IK_Net 移植（torch；backbone: transformer 默认 / mamba / lstm）
  kinematics.py  DifferentiableKinematicsLayer + PhysicsInformedLoss（Astral ψ 几何）
  dataset.py     generate_dataset（IID）+ generate_trajectory_dataset（时间相干）+ SwivelSequenceDataset
  pipeline.py    PiMIKPipeline（两段式推理）+ SyntheticPsiPredictor（免 torch 测 e2e）+ TorchPsiPredictor
  train.py       DDP 训练脚本（torchrun）
  eval.py        训练后评估（滑窗预测 ψ → 几何 IK → ψ/肘/位姿/Joint MAE/成功率；--no_split 整段评估）
  convert_sessions.py  遥操录制 robot_data.h5 → 训练/val npz（整段留出）
```

## 快速开始

```python
from astral_pim_ik.geometry import GeometricArmAngleSolver
import numpy as np

solver = GeometricArmAngleSolver("left")
T_ee = solver.fk(np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0]))
psi = solver.psi_from_config(np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0]))
q = solver.solve(T_ee, psi)          # 确定性 (T_ee, psi) -> q
q = solver.solve(T_ee, -2.6, fallback_scan=True)   # OOD 兜底（可选，默认关）
print(solver.fallback_count)         # 兜底救援次数（可观测）
```

```python
from astral_pim_ik.pipeline import PiMIKPipeline, SyntheticPsiPredictor

# 免 torch：用已知 ψ 标签走通两段式
pipe = PiMIKPipeline(SyntheticPsiPredictor(psi_labels), "left")
q_seq, psi_seq = pipe.solve_trajectory(T_ee_seq, q_init=q0)
```

## 生成数据 + 训练 + 评估

```bash
cd /home/robot/loopkok/sdk/astral_ws
export PYTHONPATH=$PWD/src/astral_arm_teleop:$PWD/src/astral_pim_ik
P=/home/robot/miniconda3/envs/lerobot/bin/python3     # 有 torch+pinocchio+4090D

# 1) 时间相干轨迹数据（stage-1 训练用；IID 学不动 ψ）
$P -c 'from astral_pim_ik.dataset import generate_trajectory_dataset as g; g("left", n_traj=200, traj_len=100, out_npz="d.npz")'
# 2) 训练（DDP 单卡即可）
/home/robot/miniconda3/envs/lerobot/bin/torchrun --nproc_per_node=1 astral_pim_ik/train.py \
    --data_path d.npz --window_size 15 --batch_size 256 --epochs 40
# 3) 评估：滑窗预测 ψ → 几何 IK → ψ 角误差/肘误差(mm)/位姿(mm)/Joint MAE(deg)/成功率
$P astral_pim_ik/eval.py --checkpoint checkpoints/best_transformer_L4_w15.pth --data_path d.npz
```

实测（合成轨迹 20k 帧 / 40 epochs / 4090D）：ψ 误差 27°（median 19°）、Joint MAE 10°、solve 率 74%；oracle（真值 ψ）位姿 <0.06 mm、solve 100%——stage-2 精确，残差来自合成数据的宽肘先验。

**真机数据（推荐路径）**：遥操已支持 `human_elbow_mode: "hard"`（yaml 默认）——机器人肘
严格贴 Quest 人肘（执行的 q 的臂角 == 人臂角），此时遥操录制的关节流离线算标签即得
**人手姿态的 ψ**：

```bash
# 1) 机器人侧遥操（hard 模式）用 astral_data_collect 录 N 段 → 2) 转 npz（整段留出）
python3 astral_pim_ik/convert_sessions.py --sessions ~/astral_data/<task> \
    --side left --out_dir ~/pimik_data --val_sessions <留出会话>
# 3) 训练 → 4) 整段评估（--no_split）
torchrun --nproc_per_node=1 astral_pim_ik/train.py --data_path ~/pimik_data/left_train.npz ...
python3 astral_pim_ik/eval.py --checkpoint ... --data_path ~/pimik_data/left_val.npz --no_split
```

## 测试

```bash
cd /home/robot/loopkok/sdk/astral_ws
export PYTHONPATH=$PWD/src/astral_arm_teleop:$PWD/src/astral_pim_ik
PY=/home/robot/miniconda3/envs/arm_sdk/bin/python3     # pinocchio 3.8 + torch 2.13(cpu)

$PY src/astral_pim_ik/astral_pim_ik/test_deterministic_solver.py   # 8/8：FK↔IK 回环/ψ 保真/限位/确定性/直肘/OOD fallback
$PY src/astral_pim_ik/astral_pim_ik/test_dataset.py                # 3/3：schema/标签回环/轨迹时间平滑
$PY src/astral_pim_ik/astral_pim_ik/test_pipeline.py               # 2/2：两段式 e2e + warm-start 连续性
$PY src/astral_pim_ik/astral_pim_ik/test_network.py                # 4/4：9D 表征/forward/L2/损失 + ψ 约定一致性 + L_elbow 回归
$PY src/astral_arm_teleop/astral_arm_teleop/test_geometric_ik.py   # 10/10：回归
```

依赖：`numpy` / `scipy`（必装）；`pinocchio`（`geometry.py`/`dataset.py` 需要，通过 `astral_arm_teleop.ik.geometric`）；`torch`（仅 `network.py`/`kinematics.py`/`train.py`/`eval.py`，可选）。

## 对抗性审查

见 [docs/adversarial_review.md](docs/adversarial_review.md) — 覆盖不可行 ψ、ψ=0 约定一致性、球形/直肘奇异、分支确定性等发现（F1-F11，其中 F2 已关闭，F1 已实现兜底，F9/F10/F11 为首次 torch 执行暴露并已修复的真 bug）。
