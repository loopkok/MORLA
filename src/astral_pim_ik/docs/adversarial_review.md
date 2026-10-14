# 对抗性审查 — astral_pim_ik 两段式臂角 IK

审查范围：本包新代码（`geometry.py` / `dataset.py` / `network.py` / `kinematics.py` / `pipeline.py` / `train.py` / `eval.py`）、被复用的现有代码（`astral_arm_teleop/ik/geometric.py`、`analytic.py`）、以及 PiM-IK 参考项目（`pim-ik-*/core/*`）。每条给出「触发条件 → 错误行为 → 复现/缓解」，按严重度排序。文末附可复现命令与实测数字。

> 2026-09-04 更新：F1 已实现可选兜底、F2 已在有 torch 的机器上实测关闭、新增 F9-F11（首次真实执行 torch 路径暴露的三个真 bug，均已修复 + 回归测试）。见各节。

## 严重度分级结论

| # | 位置 | 严重度 | 一句话 | 状态 |
|---|------|--------|--------|------|
| F1 | 新 `geometry.py: solve` | 高 | 不可行 ψ → `solve` 返回 `None` → 管线停摆（OOD 主风险） | **已实现 `fallback_scan`（默认关）+ `fallback_count`** |
| F2 | 新 `kinematics.py` vs `geometric.py` | 高 | ψ=0 约定若漂移 = 静默错误（损失监督错肘点） | **已实测一致 9.58e-07 m，关闭** |
| F9 | 新 `network.py: transform_to_9d` | 高 | 6D 旋转行/列主序错误 → 编码自洽性破坏 | **已修**（非恒等 round-trip 回归） |
| F10 | 新 `kinematics.py: PhysicsInformedLoss` | 高 | 形参 `W`（腕部）被窗口长度遮蔽 → L_elbow 对 int 广播 ~13m | **已修**（perfect-ψ→L_elbow≈0 回归） |
| F11 | 新 `dataset.py: generate_dataset` | 高 | IID 采样无时间信号，单帧 T_ee 不决定 ψ → stage-1 学不动 | **已修**（新增 `generate_trajectory_dataset`） |
| F3 | 复用 `geometric.py: psi_ref=S` | 中 | 肩心靠近基座原点时 ψ=0 参考向量退化 | 记录在案（本机 \|S\|≈0.12 m，fallback 覆盖） |
| F4 | 复用 `geometric.py: _pk2` 球形奇异 | 中 | q2≈±90° 分支被静默丢弃 → 可能无解 | 记录在案 |
| F5 | 复用 `geometric.py: _collect_solutions` 去重 | 中 | 2e-4 rad 哈希格，依赖"真分支 ≥1e-2 才不塌缩" | 记录在案 |
| F6 | 新 `dataset.py` 直肘门控 | 中 | 近直肘 ψ 弱定义，标签噪声靠 `is_valid` 屏蔽 | 记录在案 |
| F7 | 新 `geometry.py` 冷启动分支 | 低 | 无 warm-start 时 min-q4 是任意但确定的 tiebreak | 接受 |
| F8 | 参考 `inference.py` 面板计时 | 低 | NN 时间由 total−IK 反推，量级误导（未移植） | 记录在案 |

---

## F1 — 不可行 ψ 导致 `solve` 返回 `None`（高）

**触发条件**：网络对某个可达位姿预测了一个「当前关节限位下不可行」的臂角 ψ（例如预测到肘圆上被限位排除的那一侧）。

**错误行为**：`GeometricArmAngleSolver.solve` 是纯函数，`_collect_solutions(T, g, S, W, q4l, [ψ])` 在硬限位过滤后为空 → 返回 `None`。真实系统里下游会「保持上一帧指令」，表现为手臂在可达位姿处卡住不跟。

**实测复现**（左臂，q=[0.32,0.11,-0.53,-0.80,0.28,0,0]，ψ_gt=0.534 rad）：
```
opposite ψ = ψ_gt + π = -2.608  →  candidates=0  →  solve()=False（None）
gt ψ       = 0.534            →  candidates=1  →  solve()=True
```

**关键量化**：对**分布内**数据（从工作空间采样配置、ψ 取其真值）不可行率为 **0 / 176**。即「网络学到的可行 ψ」与「求解器消费的 ψ」在分布内一致，不可行只出现在 OOD。这正对应 PiM-IK README 里的「VR 遥操作 OOD」风险，但**在确定性求解器里它从"软先验+全局回退"变成了硬失败**——因为我们按要求剥掉了遥操的全局扫描回退。

**缓解**（均不破坏"确定性"约束）：
- 训练数据覆盖完整工作空间 + 只用可行 ψ 作标签（已如此）；
- 运行时对 `None` 做「保持上一帧」或「最近可行 ψ 的一次性扫描」——后者应作为可选、显式的 fallback，而不是默认行为。

**2026-09-04 已实现**：`solve(T, ψ, fallback_scan=True, n_scan=36)` 在 `candidates` 为空时按 `wrap_to_pi` 圆距网格扫描最近可行 ψ（网格序确定 tie-break，保持确定性），成功救援计入 `solver.fallback_count`（可观测、不影响返回）。默认 `fallback_scan=False` 行为与修复前完全一致。`PiMIKPipeline.solve_trajectory` 同步透传。回归测试 `test_ood_fallback_scan`：双臂各救回一个 ψ+π 不可行样本，位姿 <1mm、计数 +1、默认路径不碰计数。

---

## F2 — ψ=0 约定必须在四处锁死（高）

**触发条件**：四处的 ψ=0 参考不一致——①求解器（`geometric._circle_basis_sw`）、②数据集标签（`psi_from_elbow_dir`）、③可微运动学层（`kinematics.circle_basis`）、④先验。

**错误行为**：若 `kinematics.py` 的 `circle_basis` 与 `geometric._circle_basis_sw` 参考向量不同，训练时 `L_elbow` 监督的肘点与求解器实际解出的肘点不一致 → 网络学到的 ψ 语义错位，**端到端位姿仍可能对，但臂角整体偏一个固定量**（静默错误，比崩溃更危险）。

**现状**：①③④ 复用/复刻同一公式；③ 是 torch 向量化重写，**必须**与 numpy 版逐位一致。`test_network.py::test_loss_and_convention` 专门做了 numpy-vs-torch 肘点一致性断言（err < 1e-5 m）。

**2026-09-04 已实测关闭**：在 arm_sdk env（torch 2.13）跑 `test_network.py`，肘点一致性 **9.58e-07 m**（< 1e-5 阈值）。顺带暴露并修复了 F9/F10 两个 torch 路径真 bug（见下），`test_network.py` 现 4/4 必跑（无 torch 仍优雅 skip，但本机两套环境都齐）。

**缓解**：把该一致性测试作为 CI 必跑项；`kinematics.circle_basis` 里的参考向量写死为 `S`（肩心），并在此处加注释锚定 `geometric._circle_basis_sw`。

---

## F9 — `transform_to_9d` 6D 旋转行列序错误（高，已修）

**触发条件**：任何用 9D 表征训练/推理的路径（首次有 torch 的机器跑 `test_transform_9d` 即暴露）。

**错误行为**：`T_ee[:, :3, :2]` 是 (3,2) 块，`.view(...,6)` 按**行主序**拉平成 `[r00,r01,r10,r11,r20,r21]`，但 docstring 与 `rotation_6d_to_matrix` 期望的 Zhou et al. 6D 是**列主序** `[r00,r10,r20,r01,r11,r21]`（=[col0; col1]）。恒等位姿下编码成 a1=a2=[1,0,0]，Gram-Schmidt 还原出退化矩阵 `[[1,0,0],[0,0,0],[0,0,0]]`——旋转信息以错位顺序进入网络，验证函数无法还原。

**实测**：`torch.eye(4).repeat(3,5,1,1)` → `x9[:,0,3:]` = `[1,0,0,1,0,0]`（应为 `[1,0,0,0,1,0]`）。

**修复**：拉平前 `transpose`（dim==3 用 `(0,1)`，dim==4 用 `(-1,-2)`）；`test_transform_9d` 补「非恒等 SO(3) encode→decode round-trip」断言（修复前必失败，修复后 1e-5 内还原）。

---

## F10 — `PhysicsInformedLoss.forward` 形参 W 被遮蔽（高，已修）

**触发条件**：任何一次真实训练（首次跑 `train.py` 即暴露——loss 卡 ~14 且网络几乎不学）。

**错误行为**：`B, W, _ = pred_psi.shape` 把形参 `W`（**腕部位置张量**）重绑定为窗口长度 int。`self.kinematics_layer(pred_psi, S, W=15, ...)` 里 `circle_basis` 算 `sw = 15 - S` → 肘圆中心被推到 ~7-13 m 外 → `L_elbow` 是 ~13 m 的假误差，淹没 L_psi（~1），网络只学到无意义的梯度。训练 loss ~14 且 val 平台，正是这个假项的指纹。

**实测**（修复前）：同一批真值数据，手动肘误差 max 0.33 m / mean 0.11 m，而 `loss_fn` 报 `L_elbow=12.97`。

**修复**：解包改 `B, T, _ = pred_psi.shape`，`if T >= 3` 用 `T`。回归测试 `test_elbow_loss_measures_elbow`：perfect ψ（=标签）在 solver 一致几何上必须 `L_elbow < 1e-3`（实测 1e-4 m）；遮蔽 bug 下此值 ~13 m 必失败。**教训：torch 形参命名别和解包变量撞名。**

---

## F11 — `generate_dataset` IID 采样对 stage-1 不可学（高，已修）

**触发条件**：用 IID 随机位形数据训练 stage-1（首次真实训练即暴露——loss 平台、ψ 误差 ~53°）。

**错误行为**：给定单个 `T_ee`，ψ 在 S-W 轨道圆上**不可观测**（任一可行 ψ 同一腕部位姿）。IID 数据帧间无运动关联，窗口拿不到「肘如何动」的信号，网络只能回归先验众数 → ψ 误差 ~随机。

**实测**：IID 5k 帧 ψ 误差 52.8°（loss 平台 ~1.15）；同一模型换平滑漂移轨迹（`generate_trajectory_dataset`，帧间 ψ 漂 ~0.5°/帧）训后 ψ 误差 15-27°（loss 降到 0.4）。oracle（真值 ψ）始终 100% solve、位姿 <0.06 mm——问题在数据形态不在 IK。

**修复**：新增 `generate_trajectory_dataset`（n_traj × traj_len 平滑关节漂移，`travel`/`wobble` 控制速度，schema 与 `generate_dataset` 一致）；`test_dataset.py` 断言轨迹内帧间平滑。**文档明确：训练必须用轨迹数据（或真机遥操录制），`generate_dataset` 只留作 stage-2 标签测试。**

**注意**：合成轨迹数据仍有一个宽肘先验——ψ 路径是 q 路径的隐函数，无自然「人类肘」约束；实测残差 27° 主要是这个先验宽度，不是代码缺陷。上真机前用 `astral_data_collect` 录的真实动作重训（见 `CLAUDE.md` §7）。

---

## F3 — `psi_ref = S` 在肩心靠近基座原点时退化（中，复用代码）

**触发条件**：`extract_arm_geometry` 用 `psi_ref = S.copy()`（肩心向量）作 ψ=0 参考。若肩心 S 到基座原点距离很小，`t = S × u` 对大多数 u 都接近零，`_circle_basis_sw` 连续两级 fallback 后 e1 方向不稳定。

**错误行为**：ψ=0 的基准方向随 u 剧烈变化，数据集标签与求解器仍一致（同一套公式），所以**内部仍自洽**；但 ψ 的物理含义（肘平面朝向）变得对肩心坐标敏感，换 URDF 若移动肩原点会整体改写 ψ 分布，需重新生成数据。

**现状**：Astral 肩心 S≈[0.117, −0.004, −0.472]，|S|≈0.48 m，远离退化。**记录在案，不处理**。

---

## F4 — `_pk2` 球形奇异静默丢分支（中，复用代码）

**触发条件**：肩部球形关节 q2≈±90° 时，`_pk2` 里 x ∥ a1，`nx < 1e-9` 走 `continue`，该分支被丢弃。

**错误行为**：在该奇异附近一个本可求解的 (T, ψ) 可能返回 `candidates=[]` → `solve=None`。采样器 `sample_configs` 已排除 |q2|≈π/2，但 OOD 目标靠近该奇异仍会失败。

**现状**：与现有 `test_geometric_ik` 行为一致（那里也明确不测该奇异）。**记录在案**。

---

## F5 — `_collect_solutions` 去重哈希（中，复用代码）

**触发条件**：`key = round(cand[:7] * 5000)`，哈希格宽 2e-4 rad。注释声称「真分支间隔 ≥~1e-2，永不塌缩」。

**错误行为**：若两个真分支距离 < 2e-4 rad（极罕见，仅在双重根附近），会被误判为重复而塌缩，丢失一个候选。当前臂与限位下未观察到。**记录在案**，属可接受启发式。

---

## F6 — 直肘奇异：ψ 弱定义（中，新代码）

**触发条件**：肘接近伸直（S/E/W 共线），肘圆半径 r→0，ψ 数值上无意义。

**错误行为**：数据集里 `sin_alpha = r/l_se` 低（<0.05）的帧 `is_valid=0`，`L_elbow`/`L_psi` 被 mask 掉，正确；但网络仍会对这些帧输出一个 ψ，推理时该 ψ 是噪声。求解器侧：半径 6mm 时仍能解出位姿（实测 `solve(T, ψ=任意)` 均 True、位姿精确），只是臂角由 warm-start 连续性兜底，不会跳。

**实测**：直肘配置 q4≈−0.05，肘圆半径 0.0059 m，`solve(T, ψ∈{-1,0,0.3,1})` 全部可解、cands=1。

**缓解**：数据门控 `min_sin_alpha=0.05`（默认）+ 推理 warm-start 连续性，已足够。**记录在案**。

---

## F7 — 冷启动分支选择（低，新代码）

**触发条件**：`solve(T, ψ, q_init=None)` 时，同 ψ 下最多 2(q4)×2(q123)×2(q567)=8 候选，取 `min q4`（最弯肘）作 tiebreak。

**错误行为**：仅第一帧无 warm-start 时命中，选择是**任意但确定**的；若首帧 ψ 恰好对应"肘朝上"分支而默认选了"肘朝下"，首帧臂姿可能与直觉不符，但位姿正确、后续被 warm-start 平滑。文档已说明。**接受**。

---

## F8 — PiM-IK 参考 `inference.py` 面板计时（低，未移植）

`print_results` 里 NN 时间用 `(total_time − avg_time*T)/T` 反推，量级会误导（NN 与 IK 之外的开销被算进 NN）。未移植到本包。**记录在案**。

---

## 附带发现（对 PiM-IK 参考代码的提示）

- `pim_ik_net.py::PiM_IK_Net.forward` 的 Transformer 分支同时传 `mask=causal_mask` 和 `is_causal=True`，在部分 PyTorch 版本会冲突/告警；本包移植时已改为只传显式 `mask`。
- `pim_ik_kinematics.py::DifferentiableKinematicsLayer` 用 `v_ref=[-1,0,0]`（G1 约定）作参考向量——**换机器人必须改**，本包已改用 Astral 的 `psi_ref=S`。这提醒：参考向量是"每台臂一个约定"，不可照抄。

---

## 复现命令与实测数字

```bash
# 本机（astral_ws 根目录）——原文档的 /opt/anaconda3/envs/MujocoSim 在本机不存在
PY=/home/robot/miniconda3/envs/arm_sdk/bin/python3     # pinocchio 3.8 / torch 2.13 (cpu)
export PYTHONPATH=src/astral_arm_teleop:src/astral_pim_ik

$PY src/astral_pim_ik/astral_pim_ik/test_deterministic_solver.py   # 8/8 PASS（含 F1 fallback）
$PY src/astral_pim_ik/astral_pim_ik/test_dataset.py                # 3/3 PASS（含 F11 轨迹模式）
$PY src/astral_pim_ik/astral_pim_ik/test_pipeline.py               # 2/2 PASS
$PY src/astral_pim_ik/astral_pim_ik/test_network.py                # 4/4 PASS（F2 关闭 + F9/F10 回归）
$PY src/astral_arm_teleop/astral_arm_teleop/test_geometric_ik.py   # 10/10 PASS（回归）
```

关键实测（F1）：`gt ψ=0.534 → candidates=1, solve=True`；`ψ+π=-2.608 → candidates=0, solve=None`；`fallback_scan=True` 救回、位姿 <0.03 mm、计数 +1。分布内可行率 0/176 不可行；直肘半径 0.0059 m 仍可解。

关键实测（F2/F9/F10/F11）：
```
torch/numpy 肘点一致性 err = 9.58e-07 m（<1e-5，F2 关闭）
transform_to_9d 修复前恒等位姿编码 = [1,0,0,1,0,0]（退化）；修复后非恒等旋转 round-trip 1e-5 内还原
L_elbow 遮蔽 bug：手动肘误差 mean 0.11 m vs loss_fn 报 12.97；修复后 perfect-ψ → 1e-4 m
训练对照：IID ψ err 52.8°（loss 平台 ~1.15）；轨迹 20k/40ep ψ err 27.1°、Joint MAE 10.0°、solve 74%
         轨迹 5k seed5 ψ err 15.2°；oracle 位姿 <0.06 mm、solve 100%
```

## 交付限制

- torch / 训练路径在本机已真实跑通（arm_sdk 跑单测、lerobot/4090D 跑训练评估）；首次执行即暴露并修复 F9/F10/F11 三个真 bug，现均有回归测试。
- stage-1 在**合成轨迹数据**上的 ψ 残差 ~27° 主要受合成数据的宽肘先验限制；真机遥操录制数据是生产路径（见 `CLAUDE.md` §5.1/§7）。
- teleop 节点接入（`solver_type: "pim_ik"`）与人类肘先验仍不在范围（用户明确）。
