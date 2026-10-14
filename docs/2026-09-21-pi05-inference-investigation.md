# pi0.5 真机推理问题记录

> 建立日期：2026-09-21  
> 状态：调查中  
> 目的：记录 Astral 上 pi0.5 推理异常的客观证据、实验变量、结果与后续结论，避免把模型、视觉域偏移和执行层问题混在一起。

## 1. 当前环境

- 模型：Astral 数据微调的 pi0.5（需继续确认是 `pi05_astral` 全量还是 `pi05_astral_lora`）。
- GPU 推理主机：`192.168.1.249`。
- GPU 启动脚本：`~/loopkok/VLA/astral_ws/scripts/` 下的 `server29999` / `sever29999` 脚本（准确文件名待确认）。
- 训练数据：`astral_data/pi/pick_place_merged_repaired_v3`。
- 本次推理日志：`astral_ws/inference_test_logs/inference/20260921-143151_pi0.5_single+camera`。
- 本次推理录像：`astral_data/pi/pi0_infer_test`。
- 训练集规模：100 episodes、20,883 frames、30 Hz，平均每段约 6.96 秒。
- 训练任务：红盖/绿盖液体容器放入盒子，各 50 episodes。

## 2. 已观察到的异常

1. Web 端开始策略后，即使桌面上没有目标物体，机械臂/夹爪仍会缓慢下降并继续执行类似抓取流程。
2. 有目标物体时，抓取位置和放置位置不准确。
3. 模型会把桌面上的条形胶带等干扰物当成目标去抓。
4. 同样约 100 条数据训练的 ACT 没有明显出现以上问题。

## 3. 已确认的客观因素

### 3.1 左腕相机视角在训练后发生变化

当前左腕相机大致安装位置不变，但视野比训练数据明显更宽。按相同或相近机械臂关节姿态比较后，可见：

- 训练画面通常只出现两侧夹爪尖端，目标在画面中占比较大；
- 当前画面能看到完整夹爪横梁、更多桌面、盒子和蓝色支架；
- 目标瓶身和瓶盖在 224×224 输入中的像素面积明显减小；
- 除视场角变化外，还存在一定俯仰、裁剪和相机到夹爪外参变化；
- base 相机图像统计基本稳定，明显变化集中在 left wrist。

图像统计对比：

| 图像 | RGB 均值变化 | 标准差变化 | 初步判断 |
|---|---:|---:|---|
| base | 约 0.002～0.008 | 约 2%～4% | 分布基本一致 |
| left wrist | 约 0.026～0.095 | 约 17%～19% | 存在明显视觉域偏移 |

影响判断：该变化足以影响目标识别、像素位置到机器人动作的隐式映射，以及最终抓取/放置精度；很可能是误抓胶带和精细定位失败的重要因素之一，但不能单独解释全部异常。

### 3.2 本次推理 prompt 为空

本次 `pi_metrics.jsonl` 中所有记录的 `prompt` 都为空，推理录制数据的 task 也是空字符串。训练集只有两条明确任务文本，没有训练过空 prompt 或泛化任务文本。

后续实验必须在启动策略前设置与训练完全一致的任务文本，例如：

```text
Pick up the red-capped liquid container and place it into the box
```

或：

```text
Pick up the green-capped liquid container and place it into the box
```

实验开始后必须从 `pi_metrics.jsonl` 再次确认 prompt 非空。

### 3.3 当前执行时间轴与训练时间轴不一致

本次日志统计结果：

| 指标 | 预期 | 实际 |
|---|---:|---:|
| 策略动作行消费频率 | 30 Hz | 约 11.6 Hz |
| `control_interp=3` 后指令频率 | 90 Hz | 约 34.9 Hz |
| 50 行 chunk 时长 | 1.67 秒 | 约 4.3 秒 |
| 重规划间隔 | 应明显低于 1 秒用于精细闭环 | 约 3.9～4.0 秒 |
| 服务端模型推理 | — | 约 7 ms |
| 完整请求 RTT | — | 约 96～114 ms |

服务端推理速度正常，慢动作主要来自机器人侧控制 timer 未达到目标频率，以及 `_tick()` 使用固定名义 `dt` 而非实际 elapsed time。

### 3.4 当前 rollout 远长于训练 episode

- 训练 episode 平均约 7 秒；
- 本次 POLICY 连续运行约 130 秒；
- 训练集中每段夹爪只经历一次闭合和一次打开（阈值跨越 2 次）；
- 本次推理录像中阈值跨越 14 次。

说明完成/失败后策略仍持续运行，并重复产生抓取阶段。后续单次实验暂定最多运行 8～10 秒。

## 4. 实验原则

每轮实验只改变一个主要变量，其余条件保持一致：

- 使用同一 checkpoint、同一服务端配置和同一目标物；
- 目标物初始位置尽量固定；
- 任务文本与训练文本逐字一致；
- 单轮最长 8～10 秒；
- 每组至少重复 10 次后再比较；
- 保存推理 metrics、joint command 日志和同步录像；
- 记录目标是否存在、胶带等干扰物是否存在，不混用场景；
- 首轮测试不要叠加 ACT temporal ensemble 或 chunk anchor 平滑。

## 5. 实验一：建立干净的 pi0.5 双相机基线

保持训练时的两个视觉输入：base + 当前 left wrist。使用以下参数：

```yaml
model: "pi05"
engine_mode: "queue_async"
action_chunk: 50
async_prefetch_ahead: 40
control_interp: 1
temporal_ensemble_coeff: 0.0
chunk_anchor_tol: 0.0
camera_image_size: 224
image_required: true
```

参数目的：

- `control_interp=1`：直接按训练数据的 30 Hz 执行动作，避免请求 90 Hz 后实际只运行约 35 Hz；
- `async_prefetch_ahead=40`：每执行约 10 行动作重新规划，而不是执行约 45 行后才重新看图；
- 关闭 temporal ensemble 和 anchor：先观察未经 ACT 后处理修改的 pi0.5 行为；
- `image_required=true`：任何已配置相机缺失或过期时不继续用旧画面规划。

### 实验一验收指标

- `pi_cmds.jsonl` 实际发送频率接近 30 Hz；
- engine `pops` 增长速度接近 30 rows/s；
- `plans` 约每 0.3～0.5 秒增长一次；
- prompt 非空且与目标颜色一致；
- 无 `EngineStateError`、相机超时或 action dimension 错误；
- 分别记录接近、抓取、抬起、放置是否成功；
- 记录是否仍误抓胶带，以及误抓发生时两路相机中的目标位置。

## 6. 实验二：base-only A/B 测试

实验一完成后，测试不使用当前左腕相机：

```yaml
cameras: ["base"]
camera_map: '{"base_0_rgb": "base"}'
```

其他参数与实验一完全一致。

### 重要前置条件

已实现端到端 `mute_cameras`：ROS 节点省略被静音相机的传输 key，修改后的 OpenPI
`AstralInputs` 允许推理请求临时缺少配置中的相机，并对左腕固定槽执行：

```text
zero image + image_mask=False
```

GPU 主机 `lukang@192.168.1.249` 的实际 checkout
`~/loopkok/VLA/openpi_astral/src/openpi/policies/astral_policy.py` 已同步新版；远端虚拟环境冒烟
验证得到 `{base_0_rgb: True, left_wrist_0_rgb: False, right_wrist_0_rgb: False}`。部署时服务未运行、
8001 端口空闲，下次按 `serve_29999.env` 启动即加载新版。不得发送一张固定旧图、全黑图却仍将
mask 设为 true，这会产生另一种训练分布外输入。

### 实验二判定

- 若 base-only 明显减少误抓和方向性偏差，说明新左腕视角正在负向干扰模型；
- 若粗定位改善但最终抓取/放置变差，说明腕相机仍然必要，应恢复旧安装视角或把新画面裁剪/透视变换到旧视角；
- 若 base-only 与双相机同样失败，则继续优先排查 prompt、服务端配置/checkpoint、动作时间轴及训练数据覆盖；
- base-only 仅作为诊断和临时退路，不直接视为最终部署方案。

## 7. 后续候选实验

按优先级排列：

1. 将当前宽视野左腕图像固定裁剪、缩放、必要时旋转，使夹爪尖端和目标尺度接近训练图像；
2. 恢复训练时的相机安装位置和焦距；
3. 使用当前相机位置重新采集数据并微调；
4. 加入空桌面/no-op、胶带等干扰物以及目标位置变化数据；
5. 对服务端原始 action chunk 与执行后的 command 分别记录，区分模型错误与执行层修改；
6. 修复控制 timer，使用真实 elapsed time，并增加实际 control/policy Hz 告警；
7. 增加单次 rollout 超时、目标存在检查和任务完成停止条件。

## 8. GPU 服务端待确认项

当前尚未读取到 `192.168.1.249` 上的实际启动脚本。开始正式 A/B 前需记录：

```bash
grep -nE 'checkpoint-dir|policy-config|default-prompt|capture-dir' \
  ~/loopkok/VLA/astral_ws/scripts/*29999*
```

确认：

- LoRA checkpoint 使用 `--policy-config pi05_astral_lora`；
- 全量 checkpoint 使用 `--policy-config pi05_astral`；
- checkpoint 确实来自 `pick_place_merged_repaired_v3`；
- checkpoint 内含对应的 `assets/astral/astral_teleop/norm_stats.json`；
- 服务端相机槽配置与本次实验一致。

## 9. 单轮实验记录模板

| 字段 | 记录值 |
|---|---|
| 日期/时间 | |
| checkpoint / step | |
| policy config | |
| cameras / camera_map | |
| prompt | |
| 目标颜色及初始位置 | |
| 是否有胶带等干扰物 | |
| control 实测 Hz | |
| policy pops 实测 Hz | |
| plan 间隔 | |
| RTT / server infer | |
| 接近是否正确 | |
| 抓取是否成功 | |
| 放置是否成功 | |
| 是否误抓干扰物 | |
| 日志目录 | |
| 录像目录 | |
| 备注/异常 | |

## 10. 实验结果日志

### 实验一：双相机、30 Hz、10-step 重规划、关闭 ACT 后处理

- 状态：待测试。
- 结果：待补充。

### 实验二：base-only

- 状态：待服务端 base-only 配置就绪后测试。
- 结果：待补充。

### 中间测试：准确 prompt + `prefetch=25` + 保留 ACT 时序融合

- 日志：`astral_ws/inference_test_logs/inference/20260921-155133_pi05_sing_no_act`。
- 用户确认：保留 ACT temporal ensemble，`async_prefetch_ahead=25`；关闭后运动主观上更卡。
- prompt：已正确设置为 `Pick up the red-capped liquid container and place it into the box`。
- 主观结果：启动后仍先缓慢下移；抓取点系统性偏向试管下方，有时落到试管下方桌面；运动仍明显卡顿。

客观日志结果：

| 指标 | 本次结果 | 判断 |
|---|---:|---|
| POLICY 时长 | 约 66.7 秒 | 仍远超训练 episode 的约 7 秒 |
| action rows | 1,745 | — |
| 实际 action/command 平均频率 | 约 26.5 Hz | 未稳定达到训练 30 Hz |
| plans | 70 | 重规划频率已明显提高 |
| 重规划间隔中位数 | 约 1.02 秒 | 与 `prefetch=25` 基本相符，但精细视觉闭环仍偏慢 |
| 服务端 infer | 约 6.9～7.0 ms | GPU 模型推理不是瓶颈 |
| 请求 RTT | 约 91～114 ms | 网络/序列化正常 |
| >80 ms 动作日志空档 | 22 次 | 明显 stop-and-go 来源 |
| 最大空档 | 约 918 ms | 足以被肉眼感知为卡住 |
| >80 ms 空档累计 | 约 6.86 秒 | 占本轮有效运行时间约 10% |
| 单步最大关节变化 p50 / p90 | 0.0129 / 0.0338 rad | 与训练轨迹 p50 / p90（0.014 / 0.033 rad）接近 |
| 单步最大关节变化 p95 / p99 | 0.0685 / 0.2000 rad | 尾部明显高于训练（0.042 / 0.060 rad） |
| 命中 slew 上限 | 最大值恰为 0.2 rad | 存在模型/换轨迹大步，被 `max_joint_vel=6` 限制 |
| 夹爪 0.5 阈值跨越 | 23 次 | 策略在长 rollout 中重复抓放，已离开单任务分布 |

初步结论：

1. 准确 prompt 生效后，系统性抓向试管下方仍存在，因此空 prompt 不是该方向性偏差的唯一原因；结合已确认的腕相机外参/视场变化，应优先进行 base-only 或旧视角裁剪测试。
2. ACT temporal ensemble 主要平滑旧/new chunk 的重叠预测；日志中的大步并未明显集中在 plan 边界，因此它不能解决 chunk 内离散动作、控制停顿或视觉外参偏差。
3. 典型动作步长与训练数据接近，`control_interp=1` 会直接暴露训练数据原有的 30 Hz 阶梯；但 p95/p99 异常增大，说明还叠加了 OOD 预测或错误闭环修正。
4. 多次长空档发生时 engine 仍有约 25～40 行 remaining，且服务端推理正常，因此不是“等待新 chunk”。可能是 joint state 暂时不满足 freshness 门限、ROS timer/回调调度停顿，或进入 `_resend_last()` hold；当前日志没有记录 hold reason，需增加诊断字段才能定责。

### 控制卡顿诊断日志（已实现）

`policy_node` 已新增 `control_diagnostics_log_file`。使用 `log_dir` 启动时，每次测试目录会自动
生成 `pi_control.jsonl`，并与 `pi_metrics.jsonl`、`pi_cmds.jsonl` 使用同一时间轴：

- `tick_interval_ms` / `tick_late_ms`：控制 timer 两次进入的间隔及相对标称周期的超期量；
- `callback_ms`：单次 `_tick()` 自身执行耗时；
- `action=state_rejected` + `missing`：本拍 `_state_ok()` 未通过；
- `observation.source_age_ms/source_status`：逐 state 源的龄期及 `fresh/stale/missing` 状态；
- `action=resend_last` + `hold_reason`：重复旧指令及原因；
- `engine.pops/plans/remaining`：判断停顿时动作队列是否仍有余量。

`pi_cmds.jsonl` 同时新增 `send_kind=new_target|resend_last`、`hold_reason` 和关联用的
`control_seq`，因此现在能识别“控制回调正常但只在 hold”，而不是把重复指令误判成没有下发。

归因规则：`state_rejected` 且 state age 超门限表示观测过期；`tick_interval_ms` 很大而
`callback_ms` 正常表示 timer/executor 未及时调度；`callback_ms` 本身很大表示回调内部阻塞；
`resend_last` 则直接查看 `hold_reason`。这四类证据可覆盖本轮待验证的四个假设。
5. 本轮仍连续运行约 67 秒，远超训练 episode。前 8～10 秒后的动作不应再用于评价一次标准 pick-place 的成功率，长时间重复运行会把状态推到训练范围外。

下一步保持优先级：

1. 完成 base-only（正确 `mask=False`）测试，判断下抓偏差是否来自新腕相机视角；
2. 单轮限制为 8～10 秒，禁止用一分钟长 rollout 判断单次任务；
3. 将 `async_prefetch_ahead` 从 25 提到 40，目标约每 10 行/0.33 秒重规划；
4. 增加 `actual_control_hz`、`actual_policy_hz`、`hold_reason`、state/image age/skew 和每次 raw action chunk 首行日志；
5. 卡顿定责前不要继续只调 temporal ensemble 系数，因为现有空档不是由 plan 边界主导。

## 11. 下一轮严格 A/B：新增诊断日志后双相机 vs 左腕 mask

目的分两步：先用与 `20260921-155133` 相同的推理条件采集新版三日志，定位卡顿；然后只改变
左腕相机 mask，判断新腕部视角是否造成系统性向目标下方抓取。

两轮共同固定：

```yaml
model: "pi05"
engine_mode: "queue_async"
action_chunk: 50
async_prefetch_ahead: 25
control_interp: 1
temporal_ensemble_coeff: 0.05
chunk_anchor_tol: 0.05
chunk_anchor_blend: 4
camera_image_size: 224
image_required: true
```

同时固定同一个 checkpoint（29999）、policy config、prompt、目标物体及胶带等干扰物位置、机械臂
初始关节位姿、夹爪初始状态、base/left-wrist 相机物理位置。每次只评估第一轮 pick-place，建议
10 秒左右停止；若比较成功率，两组各重复至少 3 次且每次恢复相同初态。

### A：双相机日志基线

```yaml
mute_cameras: "[]"
```

建议 `log_tag=pi05_prefetch25_dualcam_diag`。确认运行目录包含：

- `pi_metrics.jsonl`
- `pi_cmds.jsonl`
- `pi_control.jsonl`

本轮首先回答卡顿来自 `state_rejected`、timer/executor 延迟、callback 内部阻塞，还是
`resend_last`，并作为相机实验的行为基线。

#### A 轮结果：`inference_test_logs/inference/tetsA`

本轮三份日志已经能够定责。策略从 `IDLE` 进入 `POLICY` 后运行约 7.38 秒，共经历 211 个
控制 tick：153 次正常发出新 target，58 次 `state_rejected`。58 次拒绝的 `missing` 全部是
`image:left_wrist`；base 图和 joint state 在全部 211 个 tick 中均为 fresh。左腕图像至少出现
三段超过 `obs_timeout_s=1.0` 的间歇，第一次约在策略开始后 0.97 秒越过门限，最后一次从约
6.39 秒持续到本轮结束，并触发超过 `obs_stale_stop_s` 后自动进入 `POLICY_PAUSED`。

因此，本轮肉眼观察到的间歇/卡顿主要不是模型推理或 action queue 见底，而是左腕相机流呈
突发式到达，配合 `image_required=true` 使控制 tick 被 freshness gate 拒绝。证据如下：

- `joint`：211/211 fresh；`base`：211/211 fresh；`left_wrist`：153 fresh、58 stale；
- engine 在结束时仍有约 47 行 remaining，策略阶段没有 `engine_no_row`；
- 服务端 infer 约 6.7 ms，7 个 plan 均正常完成；
- timer 间隔 p50 约 33.9 ms、p95 约 46.4 ms；仅策略首 tick 有一次约 182.5 ms callback，
  会造成一次启动顿挫，但解释不了反复停顿和最终自动暂停；
- 日志中的 427 次 `resend_last` 均发生在自动暂停之后，原因是 `policy_paused`，不是策略运行时
  队列耗尽。

本轮还暴露了独立的 Web 控制竞态：Web 启动推理节点后立即点击“开始策略”时，节点可保持
`IDLE` 约 30 秒；终端向同一话题发送命令后才进入 `POLICY`。原 Web 实现对 VOLATILE 命令只
发布一次，未等待 DDS 发现订阅者，却立即向页面返回成功，和该现象完全一致。现已保持
VOLATILE 安全语义（避免重启后重放旧 `policy` 自动运动），改为发送前最多等待 2 秒发现订阅者；
未发现则返回 503，前端在推理状态离线时禁用控制按钮。

这份日志只能判断控制连续性，不能单凭 action/state JSONL 判断抓取落点是否改善；方向性抓偏
仍需结合录像与下面 B 轮相同初态实验。

### B：只屏蔽左腕

除下面一行外不得改变其他条件：

```yaml
mute_cameras: '["left_wrist"]'
```

建议 `log_tag=pi05_prefetch25_baseonly_diag`。此时客户端不发送左腕 key；GPU OpenPI 服务端已部署
缺槽兼容，实际输入为 left-wrist 零图且 `image_mask=False`，base 槽仍为 `True`。

除视觉消融外，这一轮也会绕过已在 A 轮确认的左腕 freshness gate。若 base 流保持 fresh，
`state_rejected missing=[image:left_wrist]` 应降为 0；因此 B 轮应同时观察运动连续性和抓取方向，
不要把“不卡了”和“视觉定位变准了”混成同一个结论。

#### B 轮结果：`inference_test_logs/inference/20260921-171159_pi05_muteleft_testB`

实机表现为无法抓到试管，并持续卡顿/抽搐。日志把“卡顿”进一步拆成了两类：

1. **A 轮的断流型停顿已经消失**：34.62 秒 POLICY 内 1015/1015 tick 全部 `policy_emit`，
   没有 `state_rejected`、`resend_last` 或 `engine_no_row`；base 1015/1015 fresh，左腕 1015/1015
   muted，joint 1015/1015 fresh。实际指令约 29.45 Hz，最大相邻发送间隔 78 ms。engine 完成
   41 次 plan，结束仍有 35 行，服务端 infer 约 6.85 ms。因此 B 中肉眼所见抽搐不是 ROS 指令
   空档，也不是 GPU/queue 卡顿。
2. **连续指令轨迹本身存在周期性大跳**：动作步长 p50 约 0.010 rad、p99 约 0.187 rad，
   共有 28 次超过 0.1 rad/步，最大值被 `max_joint_vel=6 rad/s` 的安全层截在 0.2 rad/步。
   28 次大跳全部落在某次 chunk 安装后的第 22～23 tick（约 0.75～0.81 秒）；反而 chunk
   真正安装的当拍最大跳变仅 0.042 rad。该位置恰好是 `async_prefetch_ahead=25` 时，旧 chunk
   剩余 25 行与新 chunk 前 25 行的 temporal ensemble 重叠区末端：`new[0:25]` 被融合，
   `new[25:]` 未融合，执行从融合行 24 进入纯新预测行 25 时形成内部接缝。当前 anchor 只平滑
   chunk 安装起点的 4 行，无法处理约 22 tick 后的这个接缝。回查 A 轮的 4 个 >0.1 rad
   尖峰，它们同样全部位于 chunk 安装后第 22～24 tick，说明该接缝并非 mute 左腕才产生，
   B 的长 rollout 只是把规律暴露得更充分。

抓取失败说明 base-only 对这个双相机训练的 checkpoint 不足，左腕视觉即使视角发生变化仍携带
关键近场信息；“屏蔽后更差”不能反推新左腕视角无影响，只能说明永久移除左腕不是解决方案。
最终应恢复训练时视角/裁剪，或使用新视角数据微调，而不是用 mask 作为部署配置。

### 判定

1. B 的第一次接近/抓取落点明显上移并靠近试管，而 A 仍偏向试管下方或桌面：腕部视角变化是
   主要方向性偏差来源。
2. A/B 都在相同时间发生控制停顿，且 `pi_control` 原因相同：卡顿属于公共控制链路，与左腕图无关。
3. B 粗定位改善但精抓/放置下降：base 足以定位目标，但腕部近场视觉仍必要；最终方案应恢复旧
   腕部视角或把新画面裁剪/标定到训练视角，而不是永久移除腕部相机。
4. A/B 都抓向相同错误位置：继续检查 raw action、归一化统计、state/action 顺序和 checkpoint
   数据覆盖，不能再把主要原因归到腕部相机。

## 12. C 轮：恢复正常双相机，定位 `image:left_wrist` 超时发生在哪一层

C 轮在 B 轮完成并保存日志后再做。恢复 `mute_cameras: "[]"`，其余 checkpoint、prompt、物体、
初始位姿和 A/B 参数保持不变。C 轮不再用于比较模型行为，目标仅是定位左腕帧在以下哪一段中断：

```text
V4L2/USB capture → collect tap 准入/编码/发布 → DDS 图像话题
                → policy_node 图像 callback → freshness gate
```

### C0/C1：先 IDLE、再正常推理，同步采集三层证据

先只启动双相机 streamer 和 policy 节点，**不要点击开始策略，保持 IDLE 30 秒**。policy 在 IDLE
同样持续订阅/解码图像，`pi_control.jsonl` 也逐 tick 记录 image age/status，因此 C0 可以在机械臂
完全不运动时验证相机链路。若 C0 已出现 `image:left_wrist` stale，问题与模型推理无关；若 C0
始终正常而点击开始策略后才出现，才说明 POLICY 阶段新增的推理、传输或 CPU 调度负载参与了超时。
鉴于 A/B 已发现周期性动作尖峰，C1 的运动阶段限制为 8～10 秒。

新版 streamer 会把每 5 秒的边界统计发布到 `/quest3_video_streamer/diagnostics`，开启 Web
“记录推理日志”后由 policy 自动合并进 `camera_diagnostics.jsonl`：

- `[capture left_wrist] driver-side N fps`：低或长时间不再出现，优先怀疑相机、USB、V4L2
  `read()` 或捕获线程；JSON 中的 `max_frame_gap_ms` 会保留该 5 秒窗口内最长的一次帧间隔，
  可直接与第一次 stale 的时间比对；
- `[tap left_wrist] submit=... queue_full=... encoded=... published=... encode=... publish=...`：
  capture 正常但 submit/published 低，说明问题位于抽头、JPEG 编码或 DDS 发布；JSON 中的
  `max_submit_gap_ms` / `max_publish_gap_ms` 能定位间隔是发生在送入抽头前还是 DDS 发布后；
- `queue_full` 高且 `encode` 高：JPEG 编码跟不上；`publish` 显著升高：DDS publish 阻塞或主机负载。

开启 Web 完整推理日志，建议 `log_tag=pi05_left_wrist_timeout_diag`。记录第一次
`image:left_wrist` stale 的墙钟时间；相机诊断使用同一 epoch 时间轴，不再需要另开测速终端。
不要在 C1 改 `image_timeout_s`、`image_required`、分辨率、JPEG quality 或推流设置，否则只能掩盖
超时，无法定位丢帧边界。

勾选 Web“记录推理日志”后，一份完整 C 轮目录会自动包含：

```text
pi_metrics.jsonl
pi_cmds.jsonl
pi_control.jsonl
camera_diagnostics.jsonl
```

`camera_diagnostics.jsonl` 的 `stage` 分为 `capture`（V4L2/ROS 源）、`collect_tap`
（JPEG/队列/DDS 发布）和 `policy_rx`（每帧到达 gap、payload、JPEG decode 耗时/结果）。文件由
独立后台线程写入，图像 callback 只做非阻塞入队，避免诊断本身制造超时。

### 2026-09-22：命令 IDLE 与 left_wrist 低接收率的回调组修复

`TEST_922-1101` 证明 streamer 两路均约 30fps、queue full 为 0，但 policy 端 base 收到
28.7fps、left_wrist 仅 2.86fps（最长 gap 2.37 秒），`pi_cmds.jsonl` 为 0 行且全部控制 tick 为
IDLE。原因不是 GPU server 或相机，而是所有高频 image、control timer 和 `/policy_inference/cmd`
都落在 Node 默认的 `MutuallyExclusiveCallbackGroup`；即使使用 4 线程 executor，实际仍被串行化。
现已改为每相机一个串行 group、轻量 state 回调 reentrant、命令专用 group、控制/status 专用 group。
新一轮 launch 应显示 `callbacks=image-per-camera+command+control`；点击策略时必须先出现
`cmd_rx`、再出现 `cmd_exec`，对应 `pi_control.jsonl` 的 `cmd_execute` 事件。若 `cmd_rx` 都没有，
再查 ROS domain/topic；若 `cmd_rx` 有而 `cmd_exec` 没有，查 control callback 阻塞。

### 2026-09-22：C 轮 `TEST——0922-1137` 结论

该轮已部署回调组修复，`cmd_rx seq=1` 后 29.0ms 出现 `cmd_exec seq=1 state=POLICY`，
`pi_cmds.jsonl` 有 579 条实际指令；因此“Web 点开始策略仍 IDLE”已解决。两路 policy 接收也恢复：
base 为 28.98fps、left_wrist 为 29.77fps，二者 gap p95 分别为 40.46/38.93ms，图像年龄 p95
分别为 34.50/34.27ms，无 image stale。结论：当前卡顿**不是**左腕相机采集、DDS 接收或观测超时。

控制正常阶段 30Hz 指令间隔中位数为 33.30ms、p95 为 37.60ms；仅开始策略的首个同步 plan 使一次
control callback 达 306.75ms（首 plan 端到端 130ms），随后 23 次后台 replan 为约 93--137ms，
未造成持续断流。当前运行参数仍是 `coeff=0.05`、`anchor_tol=0.05`、`prefetch=25`，并非计划中的
pi0.5 无 ACT 融合对照，故下一轮应只改 `temporal_ensemble_coeff=0`、`chunk_anchor_tol=0`，其余保持。

运动语义仍不正确不能归因于图像时序：训练集 action 第 8 维确为 0..1 的 gripper ratio，C 轮实际
下发的臂关节相对 state 偏差最大达到 0.44rad，且 launch 摘要出现 `clip:left_ee_cmd` 与
`slew:left_arm_cmd`。旧日志仅保留 safety 后的指令，不能判断是模型原始 action、时序融合还是 safety
层造成；现已在新日志加入 `raw_targets` 和 `safety_events`。下一轮需要检查它们，且不要凭现有
最终指令值断言模型输出越界。

### C1 判定矩阵

| 同一时刻的证据 | 结论/下一步 |
|---|---|
| capture 左腕先掉到接近 0，base 正常 | 左腕设备/USB/V4L2 捕获层；查内核 USB reset/disconnect、线缆和供电 |
| capture 正常，tap `queue_full` 上升或 published 掉速 | streamer JPEG/抽头线程或 CPU 调度瓶颈 |
| tap published 正常，但 `policy_rx` 出现约 1 秒 gap | DDS 传输/接收调度；检查 CPU 饥饿和 ROS executor |
| `policy_rx` gap 正常但 `decode_ms` 暴涨/decoded=false | policy JPEG 解码异常或 callback 计算阻塞 |
| base 与左腕同时出现 gap | 公共 CPU/USB 总线/streamer 进程问题，不是单个左腕设备 |
| 只有左腕 gap | 左腕设备、物理 USB 路径或该路编码线程问题 |

若捕获层异常，在测试后立即保存 `journalctl -k`/`dmesg` 中同一时间附近的 `usb`、`uvcvideo`、
`reset`、`disconnect` 信息；不要在相机正被 streamer 占用时另跑 `v4l2-ctl --stream-*`，避免引入
第二个设备消费者。若 C1 证明 capture 正常而 policy callback stale，再做 C2：保持推理参数不变，
仅关闭 WebRTC 视频会话/浏览器预览，比较 gap 是否消失，以验证同进程编码或 CPU 争用。

## 13. D 轮：单变量关闭稀疏 temporal ensemble，验证周期性抽搐

C 轮完成后再做，恢复正常双相机，并保持 `async_prefetch_ahead=25`、`chunk_anchor_tol=0.05`、
`chunk_anchor_blend=4`、`control_interp=1` 等参数不变，只改：

```yaml
temporal_ensemble_coeff: 0.0
```

单轮限制 8～10 秒。比较 B 中高度规律的“每次 chunk 安装后第 22～23 tick、步长 >0.1 rad”是否
消失。若消失，说明抽搐来自当前稀疏重规划下 temporal ensemble 重叠区结束时的权重断崖，而非
模型推理卡住；若仍在相同 chunk 内索引出现，则需记录服务端完整 raw chunk，确认 pi0.5 原始动作
本身在约第 25 行是否不连续。`control_interp=2` 或降低 `max_joint_vel` 只能降低冲击，不应用来替代
这个单变量定责实验。

### 2026-09-22：`TEST_0922-1352` — 原始动作越界；本轮 YAML 参数仍为 ACT 值

启动行仍为 `coeff=0.05 anchor_tol=0.05`，所以该轮不能称为 D 对照；这些参数仅由
`policy_inference.yaml` 管理，下一轮应在 YAML 修改后重启节点，并以此启动行核验。该轮图像、状态、控制节奏仍正常（base/left_wrist 28.48/29.97fps；图像年龄
p95 34.81/34.36ms；845 条指令 dt p95 38.10ms），故“抖动/脱离范围”不是超时、断流或 left_wrist。

新 `raw_targets` 表明模型/融合器给出的臂关节目标在 845 帧中有 688 帧超出训练集 action min/max；
例如 j0 到 -1.748rad（训练下限 -0.477rad）、j3 到 -2.879rad（训练下限 -2.073rad）、j5 到
-1.029rad（训练下限 -0.503rad）。安全层只修正夹爪 ratio 越界 253 次和臂关节瞬时 slew 6 次，
不负责把臂关节钳回训练集范围，因此会忠实驱动机械臂去异常位置。chunk 边界最大 raw 跳变仅 0.096rad，
小于非边界最大 0.503rad，说明本轮主要不是 chunk 接缝跳变，而是连续的异常轨迹/视觉分布偏移或模型
训练质量问题。必须先跑参数真正为 0 的 D2 对照，再决定是否从数据与 checkpoint 排查。

### 2026-09-22：D2 `TEST_0922-1403` 的边界证据与下一轮逐 plan trace

这里需要纠正上一节容易造成的误读：**训练集动作没有越过机械臂硬件范围**。训练集
`pick_place_merged_repaired_v3/meta/stats.json` 的 action min/max 均在 URDF 关节范围内；D2 中
出现的低于 URDF 下限的值，是 pi0.5 在当前实机观测上返回的 *inference raw action*，不是训练样本
“超限”。因此不能以“训练动作不正常”解释当前现象。

此前 C1/`tetsA` 确认过另一种真实故障：`image_required=true` 且 left_wrist 偶发旧帧会触发
`image:left_wrist` gate，控制保持旧指令。回调组修复后的 D2 不再复现该**单路 freshness timeout**：
base/left_wrist policy 端接收约 28/30fps，图像龄期 p95 约 35/37ms，control tick p95 约 37ms。
这只能说明“某一路图像连续一秒未到”已不再是 D2 的主因，**不能**证明每一次送入模型的 state 与两张
图是同一物理时刻的原子快照，也不能证明请求返回时观测仍足够新。

D2（`coeff=0`、`anchor_tol=0`、`prefetch=25`）的 remote plan RTT 中位约 95ms（约 3 个 30Hz
control tick）。客户端当前会在请求前取 latest observation，并在返回后以实际 `consumed` 跳过已执行
行；设计上这是正确的延迟补偿，但旧日志没有保存每一次的 `snapshot → consumed → resume_i` 映射，无法
验证它在每个边界都用到了正确的 state/image 对。最大一次 raw 边界跳变约 0.518rad，恰发生在新 plan
把旧开环 chunk 中 j3≈-2.687 拉回接近实测 j3≈-2.23 的时刻；该 plan RTT 约 93ms，并不比中位 RTT 高。
所以“网络某次特别慢”不是这次最大跳变的充分解释；更合理的待验证链是：旧 chunk 在约 0.8 秒开环执行
中已偏离实测，而新的、可能时间不一致的视觉/state 观测要求回正。模型在新相机视角下的分布偏移和每次
flow-matching 采样差异也会放大该闭环校正，但不应为了掩盖它而固定 pi0.5 的采样噪声。

现已新增 `pi_plan_trace.jsonl`。启用 Web 的“记录推理日志”（即 launch 有 `log_dir`）后，每个运行目录
自动生成该文件；无需另开终端，也不会修改 YAML 的 `temporal_ensemble_coeff` 或 `chunk_anchor_tol`。
每个 `plan_installed` 一行记录：

- 请求快照的完整 state、prompt、实际发送的相机 label，及各 state source / image 的龄期、monotonic
  timestamp；不写图像像素，避免日志暴涨；
- 请求 RTT 和 server timing、`snap_i`、推理期间实际 `consumed`、最终 `resume_i`；
- server 返回的完整 raw action chunk、客户端最终安装的 chunk、实际将续播的 `selected_row`；
- 旧 chunk 的 `old_last_sent`、`old_next`、`old_at_snapshot`、是否在运动，以及 temporal/anchor 的
  后处理参数和实际 blend 信息。

下一轮保持当前 checkpoint、prompt、相机、初始位姿和 YAML 不变，只运行一次 8--10 秒 pick-place。
分析时按 plan id 把 `pi_plan_trace.jsonl` 与 `pi_cmds.jsonl`/`pi_control.jsonl` 对齐：若 snapshot 的
state/image age 或二者时间差异常，先定责观测原子性；若 `consumed` 与 RTT×30Hz 不匹配，查控制调度；若
二者正常而 `old_last_sent → selected_row` 或 server chunk 自身仍大跳，才将证据指向旧 chunk 开环漂移、
模型 OOD 或新采样 chunk 的不连续。这样才可以把“图像观测旧”与“模型原始动作异常”区分开。

## 14. ACT 历史卡顿的交叉对照（必须作为 pi0.5 主对照）

ACT 不是“无关的旧模型”，而是相同机器人、驱动、ROS state/image 缓存、ActionEngine 和 action-chunk
执行架构下最有价值的对照组。回查
`inference_test_logs/inference/SUMMARY.md` 及对应 `raw_logs/pi_cmds*_test*.jsonl` 后，已经能确认的
**模型无关共性**如下：

| 证据 | ACT 历史结果 | 当前 pi0.5 D2 对应含义 |
|---|---|---|
| action 与实测 state 的相位差 | 各 ACT 轮稳定约 194--214ms，手腕更大 | D2 RTT 中位约 95ms，连同相机/state 龄期、next-state 语义和驱动跟踪，同样存在“新观测返回时旧 chunk 已执行多步”的条件 |
| 大步位置 | 未开启平滑的 base--test4 有 5--17 个 >0.1rad 尖峰；全部在换 chunk 附近，已分类的 7 个中 6 个是收敛拉回 | D2 最大边界跳变同样是旧 j3 command 已偏离实测后，新 plan 拉回实测附近；这是同一类开环尾段→闭环校正模式 |
| 网络是不是唯一根因 | ACT 直连后 RTT 从约 116ms 降到约 21ms，卡顿仍可见 | 不应把 pi0.5 的约 95ms RTT 当唯一根因；它会放大错位，但不是单独的充分解释 |
| 有效处理 | ACT test5 真正启用 anchor、temporal ensemble、60Hz interpolation、JPEG 后，大步尖峰为 0 | pi0.5 D2 故意设 `coeff=0`、`anchor_tol=0` 做原始基线，不能拿它与“ACT 全部平滑生效”的 test5 直接比较优劣 |

因此当前优先级应调整为：**先验证共同的 chunk/观测/执行时间轴问题，再讨论 pi0.5 特异模型问题**。
此前 C1 的 `image:left_wrist` timeout 是已证实的公共输入链路风险，但老 ACT 日志没有逐帧 image age，
不能倒推“ACT 的每次卡顿也是左腕旧帧”。D2 的常规 freshness 统计已正常，也不能反向证明 request
snapshot 原子。新 `pi_plan_trace.jsonl` 正是补齐这个 ACT 旧日志缺失的证据：它会显示一次新 chunk 的
`old_last_sent → selected_row` 差异究竟是否随 state/image timestamp skew 或 `consumed` 异常增大。

在共同原因之外，pi0.5 仍有两项 ACT 对照不能解释的叠加风险：左腕视野已与训练数据显著不同，以及
pi0.5 raw action 在当前观测下会离开训练 action 分布。它们可解释“为什么 pi0.5 抓向试管下方/胶带、
为什么同一收敛拉回的幅度更大”，但不应取代对共用时间轴的定责。下一轮 trace 若显示时间轴正常而
server chunk 自身仍在返回相互矛盾的大动作，才有充分证据把主因进一步收敛到 pi0.5 的视觉分布、
归一化/动作语义或在线采样输出。

## 15. `TEST_0922-1505`：逐 plan trace 的首次定责结果

本轮 prompt 为训练任务文本，POLICY 期间在完成任务后立即 stop。有效动作窗口为 10.23 秒，300/300
control tick 都是 `policy_emit`；无 `state_rejected`、`resend_last`、queue empty 或运行中 image stale。
300 条实际 arm 指令的间隔中位 33.3ms、p95 37.7ms、最大 41.9ms。唯一 312ms control callback 是
进入 POLICY 时的首个同步 plan；随后 callback p95 7.58ms。因此“运动中的卡顿”不是 ROS timer、
图像 callback、state gate 或指令断流。

### 输入新鲜且近似原子；RTT 补偿也实际生效

13 次 plan 的 remote RTT 中位 98.50ms、p95 101.13ms（首块冷启动 118.30ms）；服务端 infer 中位
6.89ms，说明约 90ms 是网络/编码传输，但没有异常长尾。每次 request 之前的 snapshot state 龄期中位
4.07ms、p95 12.33ms；base/left_wrist 图像龄期中位为 22.88/16.29ms、p95 为 32.08/19.17ms；两路
image stamp 相差中位 7.31ms、p95 14.98ms。除首 plan 的启动调度间隔 175ms 外，snapshot 到 request
的间隔中位 4.84ms、p95 9.17ms。

9 个运行中 replan 实际 `consumed=3`、另 2 个为 2，恰与约 95--101ms / 30Hz 相符；首次发出的
`selected_row` 与**该行实际下发时** state 的最大差通常为 0.011--0.063rad。换言之，客户端没有把
约 100ms 旧观测直接送成错误的 action；`consumed/resume_i` 延迟补偿在这一轮工作正常。最后 plan 13
在 stop 指令之后才返回，未产生任何 command，随后引擎完整 teardown，不会带入下一次策略。

### 已证实与 ACT 相同的“旧 chunk 漂移 → 新 plan 收敛拉回”

各边界应比较“旧 chunk 的最后已发行”与新 row 真正下发时的 state，不能只比较 snapshot。结果如下：

| plan | 旧行 → 当前 state 最大偏差 | 新 selected row → 当前 state 最大偏差 | 边界最大跳变 | 解释 |
|---:|---:|---:|---|
| 6 | 0.164rad（j3） | 0.063rad（j3） | 0.101rad | 新 plan 纠正旧尾段漂移 |
| 7 | 0.124rad（j5） | 0.059rad（j3） | 0.163rad（j2） | 最大 arm 跳变；不是 RTT 或 image stale 异常 |
| 8 | 0.148rad（j5） | 0.027rad（j5） | 0.120rad | 新行明显更接近当前 state |
| 12 | 0.118rad（j5） | 0.021rad（j0） | 0.120rad | 同一模式再次出现 |

plan 5 的 0.655 差异仅发生在 gripper ratio（旧尾段 0.661 → 新行 0.007），对应抓取阶段闭合；arm
边界仅约 0.061rad，不能与 arm 抽搐混为一谈。所有 arm 大步的 `raw_targets` 与实际下发值一致，安全层
只做了 69 次很小的 gripper 负值 clip，**没有** arm slew/clip，因此也不是安全层制造了抖动。

这与 ACT 历史的 6/7 “收敛拉回”完全一致：`prefetch=25` 时，新 observation 到来前旧 chunk 已开环
运行约 25--28 行（约 0.8--0.93 秒），新 plan 基于当前 state 给出更接近实测的路径，因 `coeff=0`、
`anchor_tol=0` 而硬切换。server raw chunk 内也有步长（p95 0.083rad，最大 0.105rad），但本轮最大 arm
步长 0.163rad 发生在 chunk 边界，主导视觉上的“抽一下”。

### 遥操/driver 对照

POLICY 窗口内 driver 收到 297 条 `/left_arm/joint_commands`，逐值与 policy 的 297 条可匹配 arm
command 完全相同；teleop 在策略开始时已有 `disarm_reason=operator`，日志中没有并发的 teleop arm
command。故本轮不存在遥操与 policy 同时覆盖机械臂指令的证据。stop 后第一拍 IDLE 在末条 policy
command 后约 32ms 出现，符合“完成即 stop”的预期。

### 后续实验（先不改变模型采样）

本轮已完成 `coeff=0`、`anchor_tol=0` 原始基线的定责；不应再把主要精力放在图像超时或固定 pi0.5
flow-matching noise。下一轮只把 YAML 的 `async_prefetch_ahead` 从 25 改为 40，其余保持不变；这会把
重新看 state/image 的周期从约 25 行缩至约 10 行，同时仍不使用 temporal/anchor 平滑。若 trace 中
`old_last_sent → 当前 state` 及边界 jump 显著下降，即可确认开环 chunk 年龄是主要可控项；若 server
chunk 的内部跳变反而成为主导，再单独诊断 pi0.5 online sampling/视觉输入。

## 16. 收敛拉回：机制、数据口径与固定测试计划

本节把 §14--15 的结论整理为可复现的排查项，后续每轮均以本节口径比较。

### 机制：旧预测并非没有执行，而是开环 target 与实机脱节

在当前 `action_chunk=50`、`async_prefetch_ahead=25`、30Hz、RTT 约 100ms 的实现中：

```text
旧 chunk: [old[0] ... old[24] | old[25] old[26] old[27] | old[28] ... old[49]]
                                  ↑ 已发起请求、RTT 内继续执行  ↑ 被新 chunk 丢弃
新 chunk: [new[0] new[1] new[2] | new[3] new[4] ... new[49]]
                                            ↑ 返回后实际续播行
```

因此旧 chunk 的前约 28 行确实被逐行下发；`pi_cmds`、driver command 和 `raw_targets` 已证明 arm
没有被安全层或遥操改写。问题是 `old[25:27]` 是约 0.83--0.93 秒前、由当时单次 state/image 观测预测的
绝对位置 target。实机不保证每 33ms 到达上一个 target：驱动跟踪、惯性/摩擦、负载、接触以及模型未来
轨迹预测误差都会使实际 state 与旧预测逐渐不同。新 plan 依据当前 state 重算，通常比旧尾段更接近实机；
在 `temporal_ensemble_coeff=0`、`chunk_anchor_tol=0` 时直接 `old[27] → new[3]`，就是可见的收敛拉回。

这里的约 0.2s command↔state 相位差是历史 ACT 曲线的互相关结果，**不是**本轮 image/state 输入旧了
0.2s。位置差只可近似理解为 `位置差 ≈ 轨迹速度 × 相位/预测时间误差`：例如 0.3/0.5/0.8rad/s 对应
约 0.06/0.10/0.16rad。它在匀速时不会无限累加；在加减速、反向、接触或视觉判断改变时，则会变成
不同关节、不同方向的轨迹分叉，直到下一次 replan 才被纠正。

### 如何判定为收敛拉回，而非图像超时、网络卡顿或安全层

判定必须在同一个 plan 的三个时刻比较：

```text
1. request snapshot：检查 state/image age、双相机 skew
2. response/install：检查 RTT、consumed、resume_i
3. first selected row 真正下发时：比较 old_last_sent / selected_row 与该拍 state
```

`TEST_0922-1505` 已满足以下完整证据链：

- 300/300 tick `policy_emit`，cmd dt p95 37.7ms；无 `state_rejected`、hold、queue empty；
- state age p95 12.33ms，base/left_wrist image age p95 32.08/19.17ms，双相机 skew p95 14.98ms；
- RTT p50/p95 98.50/101.13ms，`consumed=2--3` 正好对应 RTT×30Hz；新行下发时与 state 通常仅差
  0.011--0.063rad；
- 反而 plan 6/7/8/12 的旧最后行与同拍 state 已相差 0.164/0.124/0.148/0.118rad，新行更接近 state，
  边界 arm jump 为 0.101/0.163/0.120/0.120rad；
- raw arm target 与下发 arm target 完全相同，安全层只有 69 次微小 gripper clip；driver 与 policy arm
  command 逐值一致，teleop 已 disarm。

因此本轮的“抽一下”归因于旧开环轨迹被新闭环轨迹纠正。这与 ACT base--test4 的换 chunk 尖峰及其中
6/7 收敛拉回相同；不是 pi0.5 flow-matching 特有故障，也不能由“左腕当前帧过期”解释。

### 处理原则

1. **先缩短开环年龄。** 这才会减少旧 target 相对实机的偏离；不要用更多 `resume_i` 跳步来伪补偿，
   因为 action 是绝对位置 target，盲目多跳只会使目标更远。
2. **再平滑已经发生的边界。** `chunk_anchor_tol` 直接比较旧最后 command 与新续播行；超过阈值时在
   `chunk_anchor_blend` 行内过渡。它降低冲击，但也会把真正的状态纠正延后约 `blend/30Hz`，不等于提高
   视觉抓取精度。
3. **temporal ensemble 留到后续。** 它融合旧/新预测但不直接利用真实 state。此前 `prefetch=25`
   只覆盖前 25 行融合、后半为纯新预测，可能产生内部接缝；应在更早 replan 的基础上单独验证。
4. **长期解决项。** 用 driver feedback 标定各关节 command→state 延迟，审计训练数据的
   state/image/action 时间语义，并在需要时重对齐标签或引入带时间戳的低层短轨迹控制。Ruking 等
   插值/轨迹重定时方法属于后续候选，使用前必须确认其处理的是绝对位置 target 还是速度/时间参数化轨迹，
   并继续用 plan trace 验证，不能直接套用。

### 已确定的两轮 YAML 实验

每轮均保持 checkpoint、prompt、物体位置、初始位姿、双相机、224 输入和 `control_interp=1` 不变；
任务完成即 stop，并保存 Web 自动生成的五份 JSONL（尤其 `pi_plan_trace.jsonl`）。

```yaml
# E1：验证缩短开环年龄，不使用任何平滑
async_prefetch_ahead: 40
temporal_ensemble_coeff: 0.0
chunk_anchor_tol: 0.0

# E2：只在 E1 基础上增加直接的 chunk 边界过渡
async_prefetch_ahead: 40
temporal_ensemble_coeff: 0.0
chunk_anchor_tol: 0.05
chunk_anchor_blend: 4
```

E1 的成功指标是 `old_last_sent→当前 state` 与 raw boundary jump 下降，证明较短开环年龄有效；E2 的
成功指标是 raw server chunk 不变或相近时，**applied** boundary jump 降低且不持续增加
`selected_row→当前 state`。两轮完成后再决定是否引入 temporal ensemble 或插值/轨迹重定时方案。

## 17. `TEST_0922-1626`：anchor 测试被只读 chunk 安装崩溃中断

测试参数为 `prefetch=40, coeff=0, anchor_tol=0.05, anchor_blend=4, control_interp=2`。现象是机械臂
执行到试管附近后保持不动。该轮不能用于评价 anchor、prefetch 或插值效果，因为第二个 chunk 没有成功
安装：policy launch 在 `_install()` 的 anchor 原地赋值处明确报
`ValueError: assignment destination is read-only`，随后 planner daemon 退出。

时间线与各日志完全一致：首 plan RTT 115.19ms（server infer 15.21ms），只产生 1 条
`plan_installed` trace；队列从 50 行消费到 `pops=50, remaining=0, plans=1`，16:26:09.2949 起至 stop
持续保持末指令约 4.27s。POLICY 的 358 个控制 tick 中 joint/image stale 均为 0；排除启动首拍后，
callback 平均 3.44ms、p95 7.84ms、最大 15.85ms，因此这次停止不是相机、关节观测或远端延迟导致。
`control_interp=2` 使耗尽后的 policy-rate tick 记录 `resend_last`，中间插值 tick 仍把最后 `_seg_cur`
记为 `policy_emit`；两者都是 hold，不是新模型动作。

根因是 vendored msgpack 协议用不可变 `bytes` 作为 ndarray buffer；dtype 已为 float64 时，
`np.asarray(..., dtype=float64)` 不复制，故 `coeff=0` 路径把只读 view 直接作为 `_chunk`。anchor 开启且
边界偏差超过 tol 后首次写入即崩溃；以前测试使用可写的 `np.zeros`，没有覆盖真实远端所有权语义。

修复采用三层防护：

1. `_install()` 在所有权边界用 `np.array(..., copy=True)` 取得可写且不与 backend 共享的有效行副本；
2. 新 chunk 在局部 candidate 上完成 temporal ensemble 与 anchor，成功后原子提交；失败则恢复
   ensemble counts，并保留旧 chunk、`_i` 和统计值；
3. planner 捕获所有未预期异常，记录 `planner_failed`，保留已接受尾段；尾段耗尽后 `tick()` 抛出
   `EngineStateError`，PolicyNode 执行现有安全 stop，而不是永久重发旧指令。

负面影响审计：复制不改变 dtype、shape、行值、`resume_i`、融合权重或 anchor 公式；anchor 关闭路径有
逐值回归，ACT ensemble+anchor 组合、短 chunk、慢推理续播、三种 engine mode 均由现有/新增测试覆盖。
典型 50×8 float64 仅 3.2KB，实测复制约 0.25µs/plan，相对约 100ms 远端 RTT 可忽略。完整推理包
119 tests passed，语法编译与 diff whitespace 检查通过。下一轮真机应从原定 E1/E2 单变量矩阵重跑；
本轮结果标记为“基础设施故障，无算法对比效力”。
