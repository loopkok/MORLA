# ACT 推理卡顿排查记录（症状 → 根因 → 修法 → 数值）

> 2026-09-14 真机实录（test1–5），2026-09-22 整理归档。
> 给新会话/接手者：理解**为什么**有这些引擎参数与机制，以及排查"推理卡顿"的正确顺序。
> 完整逐日记录见 `astral_ws/CHANGELOG.md`；本文件是推理包 `src/common/morla_policy_inference/CLAUDE.md`
> "推理优化历程"章节的独立展开版，两者内容同源。

## TL;DR（一分钟结论）

真机 ACT 推理"一卡一卡"是**四层独立原因叠加**，按排查顺序：

| 层 | 症状 | 根因 | 修法 | 实测 |
|---|---|---|---|---|
| **0** | 改了参数运动却不见变化 | yaml 顶层键 ≠ 节点名，整份参数被静默丢弃 | 顶层键改 `policy_node:` + 节点自报生效参数 | 连续 4 轮测试全被此坑吞掉 |
| **1** | 换 chunk 时"冲一下"（hold→lunge） | `_run_plan` 锁跨推理，堵住控制线程 ~120ms | 推理移出引擎锁 + 实测 consumed 续播 | 空档≈0 |
| **2** | 换 chunk 后 2-3 行 0.1-0.2 rad 尖峰 | 旧 chunk 开环漂移 + 观测过时 → 收敛拉回 | `chunk_anchor_tol` 切换平滑 | 7 尖峰 → ≤0.04 rad |
| **3** | 任务固定卡点（4 个） | 训练数据走走停停（20.6% 慢帧） | **治本在数据**（repair_aligned） | 引擎只能平滑不能消除 |
| **4** | RTT ~140ms 主项 | 上行 1.38MB 原始 RGB | `jpeg_transport` | RTT 140→56ms |

---

## 第 0 层：参数根本没生效（最隐蔽、最贵的教训）

**症状**：真机连续 4 次测试（test1-4）"改了 yaml 参数（coeff/tol/interp/jpeg）运动却不见
对应变化"；test4 实测指令流仍 30Hz（`control_interp: 2` 未生效）、尖峰仍在。

**根因**：yaml 顶层键是 `morla_policy_inference:`，而 launch 节点名是 `policy_node`——
rclpy 按节点名匹配 `--params-file` 的段，键不匹配**整份参数被静默丢弃**（Humble 无单键回退），
节点回落到 `_declare_params` 代码默认值：`control_interp=1`、`temporal_ensemble_coeff=0.0`、
`chunk_anchor_tol=0.0`、`jpeg_transport=false`。即**此前所有真机测试的时序融合 / 切换平滑 /
60Hz 插值 / JPEG 全都没生效**（host/port、camera_image_size、metrics/joint_stream 日志因
launch 显式传参而侥幸正确）。

对照 data_collect：其 yaml 顶层键 = `data_collect` = 节点名，所以从未踩坑。

**修法**：
1. yaml 顶层键 `morla_policy_inference:` → `policy_node:`（+ 注释警示）；
2. node 启动行扩展为**生效参数自报**（`ctrl/coeff/anchor_tol/anchor_blend/prefetch/jpeg/
   backend/host:port`），未来任何 launch 一眼可见实际参数、防复发。

**验证**：launch 默认 → `ctrl=60.0Hz coeff=0.01 anchor_tol=0.05 anchor_blend=4 prefetch=40
jpeg=True`（此前是 interp=1/coeff=0/tol=0/jpeg=false）；launch 传 `control_interp:=2` 仍
60Hz（显式覆盖照常生效）；全套件 108 例全绿。

**教训**：改参数后先看节点启动行确认生效，不要假设 yaml 被加载（Humble 无单键回退，静默丢）。

---

## 第 1 层：换 chunk 指令断流（hold→lunge）

**症状**：真机换 chunk 时"冲一下"——`pi_cmds` 实测 106~149ms 空档后接 0.1~0.15 rad 步。

**根因**：`_run_plan` 锁跨 `backend.infer()`（远程网络 ~120ms）持有，慢推理堵住控制线程
`tick()` → 指令断流；且 `i0=round(latency_ms×fps)` 按"机器人前进了"估算，但锁内推理时机器人
实际没动 → 超前跳。

**修法**（3 例回归）：
- **推理移出引擎锁**（锁内快照 obs + `_i_snap` 锚点 → 锁外 infer → 锁内安装）；
- **续播索引用实测 `consumed = _i − _i_snap`**（替代 latency_ms 估算）；
- 时序融合锚点用 `_i_snap`。queue_sync 的 `_plan_blocking` 外层仍持锁（阻塞语义不变）。

---

## 第 2 层：换 chunk 收敛拉回尖峰

**症状**：时序融合已开，换 chunk 后 2-3 行仍出现 0.1-0.2 rad 尖峰（`pi_cmds` 实测 7 个全在
换 chunk 边界）。

**根因**：时序融合只平滑"新旧预测"连续性，不平滑"预测 vs 执行"。机制：旧 chunk 开环漂移 +
观测过时（请求→推理→回传 ~130ms 期间机器人已前进 4-5 行）→ 新预测一步追回实测
（收敛拉回，跳后新 chunk[i0]≈state、cmd-state 0.003~0.055）；起点用"实测"的 blend 对收敛
拉回无效（dev<tol）。

**修法**：**`chunk_anchor_tol` 切换平滑**——安装时续播起点（最后已发出的行 `_i-1`，不是
下一行 `_i`）偏离**正在执行的旧 command** >tol（默认 0.05，≈稳态跟随差，正常不触发）→ 前
`chunk_anchor_blend`（默认 4）行从旧值线性过渡到新轨迹。

**验证**：真实数据离线模拟 7 尖峰 0.1-0.2 → 全部 ≤0.04 rad。3 例单测（旧值起步 blend /
tol 内不触发 / 关闭）。

**关键认知**：起点必须用"最后已发出的行 `_i-1`"而非"下一行 `_i`"（后者超前一行，blend 起点错）。

---

## 第 3 层：任务固定卡点 = 训练数据节奏（治本在数据）

**症状**：尖峰清零后肉眼仍见 4 个**固定**卡点（到目标前 / 夹取后 / 放置前 / 释放后）。与换
chunk 无关（跨 2-3 个 chunk）。

**根因**：模型在对应任务状态输出低速轨迹——因为**训练数据本身走走停停**：`pick_place_merged`
实测 **20.6% 帧速度 <0.008 rad/帧、208 个慢速段遍布**。操作员在任务阶段转换处的停顿/减速被
模型忠实复现进 next-state 动作。

**结论**：引擎参数只能平滑不能消除（卡点是"模型预测"），**治本在数据**。

**修法**（数据层，见 `scripts/`）：
- `repair_aligned.py`：弧长均匀**选帧**（严格零 state-图像错位），`natural`（保时序摩擦移除）/
  `uniformize`（弧长匀速化）；
- `compress_pauses.py`（raw 层删停顿帧，早期方案）；
- 重训后对比真机固定卡点。

---

## 第 4 层：网络带宽（RTT 主项）

**症状**：`engine.last_plan_ms` 端到端 ~140ms，`server_timing.total` 服务端推理仅 ~10ms——
差值是上行 1.38MB 原始 RGB 在受限链路的传输（WiFi ~106ms / 直连 ~14ms）。

**修法**：**`jpeg_transport`**（camera 槽位编码 JPEG，quality 92）——载荷 1.38MB→0.45MB
（随机噪声最差；真实相机图 ~0.1-0.2MB，~7×），RTT 140→56ms。节点保持"解码→letterbox→RGB"
不变（像素无改），只在 RemoteBackend 发送前编码，serve 端解码还原（ACT/pi05 两分支照旧消费
RGB）。`false` = 现状 RGB（兼容直连 openpi 官方 serve 的退路）。

---

## 另两个 ACT 专属机制

### A. ACT 后端一次 infer 只回 1 行

**症状**：ACT 部署控制率只有 13Hz（应 30Hz）。

**根因**：两层：① ACT 后端返回 1 行 chunk（`select_action` 内部 50 行 action 队列逐行吐）→
引擎分块/预取/网络全浪费；② `_planner_loop` 锁内 `Event.wait` 饿死控制线程。

**修法**：**方案 A**——改 `predict_action_chunk` 一次返回完整 chunk（`temporal_ensemble_coeff`
非 None 回退 select_action）；图像上传从每行一次变每 chunk 一次（-96%），queue_async 达 30Hz，
loop 7.9→1.26ms。锁修复：`Event.wait` 移出锁外（空闲判定在锁内、等待在锁外）。

### B. 任务粘滞 / 摩擦伪影（数据层）

遥操 stick-slip（速度环 kp 弱）+ 摩擦卡点被模型学进 next-state 标签。量化脚本
`src/common/morla_data_collect/morla_data_collect/pipeline/quantify_cmd_state.py` 区分"操作意图停顿 vs 摩擦伪影"；摩擦主导 → `repair_aligned.py`
修复。

---

## 排查"推理卡顿"的标准顺序（checklist）

1. **先确认参数生效**：看节点启动行 `ctrl=... coeff=... anchor_tol=...`——第 0 层坑一票否决；
2. **抓 pi_cmds + pi_metrics**：`metrics_log_file:=/tmp/pi_metrics.jsonl` +
   `joint_stream_log_file:=/tmp/pi_cmds.jsonl`（或 `log_dir` 自动归档）；
3. **分类尖峰**：`scripts/plot_inference_curves.py --log pi_cmds --metrics pi_metrics`
   ——尖峰在换 chunk 边界（第 1/2 层）还是固定任务阶段（第 3 层）；
4. **换 chunk 边界尖峰** → 检查 `chunk_anchor_tol` / `temporal_ensemble_coeff` 是否生效
   （第 2 层）；空档 + 冲一下 → 推理是否仍持锁（第 1 层）；
5. **固定任务卡点** → 数据层（第 3 层），引擎参数救不了；
6. **RTT 高** → `engine.last_plan_ms − server_timing.total` = 网络+序列化，开 `jpeg_transport`；
7. **控制率低** → ACT 是否方案 A（完整 chunk，第 4 层 A）。

## 诊断资产（沉淀成工具）

- `metrics_log_file` / `joint_stream_log_file`：state 指标 + 关节指令流落盘（JSONL）
- `plot_inference_curves.py`：尖峰换 chunk 关联 + 收敛拉回/模型突变分类（`--self-test` 自测）
- `engine.server_timing` 透传：state/metrics 直接可拆"网络+序列化 vs 服务端推理"
- `pi_control.jsonl`（`control_diagnostics_log_file`）：逐控制 tick 调度/回调/state-gate/
  hold 原因（2026-09-21 起）

## 参考

- 推理包：`src/common/morla_policy_inference/CLAUDE.md`（已解决坑表 + 优化历程 + 环境约束 + 验证清单）
- 数据修复：`src/common/morla_data_collect/morla_data_collect/pipeline/repair_aligned.py`（README 数据层）、`scripts/compress_pauses.py`
- 推理测试归档：`inference_test_logs/inference/`
