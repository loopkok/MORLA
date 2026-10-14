# morla_policy_inference — 策略部署 / 数据真机回放 / 人在环路（HITL）

一个 ROS 2 Humble 功能包，通过 `morla_core` robot profile 与 `FrozenSchema`，将外部策略输入、OpenPI/ACT推理、数据回放和HITL接入标准机器人命令流。Astral历史模型兼容适配仍由策略后端负责，运行时布局始终由Profile及其layout hash确定：

- **策略部署**：VLA 策略跑在真实机器人/仿真上，模型无关换权重（后端抽象）。
- **推理节奏**：同步 `queue_sync`、异步 `queue_async`、实时分块 `rtc` 三种引擎模式。
- **全链路适配**：按与数采相同的不可变 `FrozenSchema` 订阅观测、构造state、验证模型容量与layout handshake，并具名拆分动作；换机器人只需选择同一个 `profile_id`。
- **数据真机回放**：已采集 episode（`aligned_data.h5` 或 LeRobot v2.1 目录）绝对动作回放，
  训练好的模型和采集的数据走同一条执行链。
- **人在环路**：策略暂停 → VR 遥操**增量接管**（自动重新记录机器人原点/VR 位姿）→
  控制权返回策略/回放，全程可重复。

```text
              ┌──────────── 数采/训练上游（本包不生成数据）─────────────┐
   astral_arm_teleop  + morla_data_collect  ──►  LeRobot v2.1 / aligned_data.h5
              ▲            │                                    │
              │ ~/prepare_reanchor + ownership ACK               ▼
              │ (增量接管)                              ┌──────────────────┐
              │                                        │  morla_policy_inference │
              │          joint_states / gripper_ratio  │  policy_node       │
   robot ◄────┼────────────────────────────────────────│      │ keyboard    │
   (arm driver│  /left_arm/joint_commands ...          └──┬──────────┬─────┘
    / sim) ◄──┘                                           │          │
                                   PolicyBackend ◄────────┘          │
                          openpi websocket | lerobot ACT | stub     replay
```

## 目录 / 包内部结构

```text
morla_policy_inference/
├── robot_io.py     ObsLayout：由 FrozenSchema 生成订阅表 / 观测向量 / 指令拆分
├── executor.py     SafeExecutor：NaN/限位/速率安全层（纯逻辑，可单测）
├── backend.py      PolicyBackend：RemoteBackend(remote) / InprocBackend(inproc) / StubBackend
├── client.py       自包含 websocket client + vendored __ndarray__ 序列化（无 openpi_client）
├── engine.py       ActionEngine：queue_sync / queue_async / rtc 三种分块引擎
├── controller.py   Controller：IDLE/POLICY(_PAUSED)/PLAYBACK(_PAUSED)/HUMAN FSM
├── replay.py       读取 aligned_data.h5 / LeRobot v2.1 目录，PlaybackSession 步进
├── node.py         policy_node：装配以上全部 + HITL 仲裁 + 控制定时器
├── keyboard.py     键盘驱动（s/y/空格/n/h/g/x）
├── config/policy_inference.yaml
└── launch/policy_inference.launch.py
```

## 训练入口（一键脚本 `MORLA/scripts/act_train.sh`）

ACT 三种训练模式一个脚本搞定：`--mode from_scratch|resume|finetune`
（从头训 / 同数据续步数 / 已有模型上加数据微调），参数见文件头；`--dry-run` 先看命令。

## 快速开始

### 1) 运行节点

```bash
cd /home/robot/loopkok/sdk/MORLA
ros2 launch morla_policy_inference policy_inference.launch.py \
    backend_type:=stub \
    profile_id:=nero_left_xhand \
    keyboard:=true
```

不带 `keyboard:=true` 时，可用以下任意途径发命令：

> ⚠️ **yaml 参数加载**：`config/policy_inference.yaml` 顶层键必须是 `policy_node`（= launch
> 节点名）。rclpy 按节点名匹配 `--params-file` 的段，键不匹配**整份参数被静默丢弃**（节点
> 落回代码默认值，无任何报错）。曾因此 `control_interp: 2`/`temporal_ensemble_coeff`/
> `chunk_anchor_tol`/`jpeg_transport` 全部没生效、真机一直跑 30Hz 裸引擎。确认方法：节点
> 启动行现在会自报生效参数（`ctrl=60.0Hz coeff=0.01 anchor_tol=0.05 ... jpeg=True`），
> 看到 `ctrl=30.0Hz`/`coeff=0.0` 即 yaml 没加载。要临时覆盖 yaml 用 launch 参数（如
> `control_interp:=2`）。

```bash
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'policy'" -1
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'playback'" -1
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'pause'" -1
```

节点状态 JSON 发布在 `/policy_inference/state`（latch）：

```bash
ros2 topic echo /policy_inference/state
```

### 2) 换模型 = 只改 backend 参数（其余配置零改动）

`backend_type` = **传输方式**：`remote`（连 serve.py）/ `inproc`（进程内）/ `stub`（冒烟）。
`model` = **模型族**：`act`（lerobot）/ `pi05`（openpi）/ 后续扩展——决定 serve/inproc 加载路径。
远程连接与单次响应分别受 `server_connect_timeout_s`（默认 5 秒）和
`server_infer_timeout_s`（默认 3 秒）限制。服务未启动或推理无响应时会报错，
不会无限等待。当前服务握手包含 `action_dim`；客户端也兼容只发送 `model`
的旧服务，以及独立 pi0.5 服务的 `server_ms` 耗时字段。

| 传输 | model | 改什么 | 说明 |
|---|---|---|---|
| `remote` | `act`/`pi05`/… | GPU 主机上换 `serve.py --model <模型> --checkpoint-dir` 加载的 checkpoint | 机器人端只填 `host/port`，**不碰**；server 换模型即可 |
| `inproc` | `act`（lerobot） | `checkpoint_dir` 指向新 checkpoint | 本地进程内：按 checkpoint `config.json` 的 `type` 经 `get_policy_class` 解析具体类加载；换权重/归一化/分块即换目录。**图像尺寸须与模型 preprocessor 一致**（本模型 480×480 → `camera_image_size: 480`） |
| `stub` | — | — | 无网络冒烟 / 开发用，勿上真机 |

**远程部署（推荐，解决 rclpy py3.10 与 lerobot py3.12 同进程冲突）**：模型（lerobot/pi05）在
GPU 主机起统一 `scripts/serve.py`，节点 `backend_type=remote` 连它（同一 websocket 协议，
包内 self-contained client，**无需 openpi_client 安装**）：

```bash
# GPU 主机（py3.12 lerobot 环境，serve ACT）：
<lerobot-env>/bin/python MORLA/src/common/morla_policy_inference/scripts/serve.py \
  --model act --checkpoint-dir <act_checkpoint> --port 8001
# 或直接用运维脚本（参数外置 serve_policy.env，启动前查端口冲突，--status/--stop 配套）：
MORLA/scripts/serve_policy.sh            # 启动（改参编辑 serve_policy.env，不动脚本）
MORLA/scripts/serve_policy.sh --status   # 查看端口/pidfile 状态
MORLA/scripts/serve_policy.sh --stop     # 优雅停止本脚本起的 serve
# pi05（需 openpi env + openpi 格式 checkpoint）：
<openpi-env>/bin/python .../scripts/serve.py --model pi05 --checkpoint-dir <openpi_ckpt> --port 8001
# 机器人侧节点（py3.10 + ROS）——三个参数必传，engine_mode 默认 queue_async 即可：
ros2 launch morla_policy_inference policy_inference.launch.py \
  backend_type:=remote host:=127.0.0.1 port:=8001 \
  camera_image_size:=480
```

### OpenPI π0.5 guided RTC

普通 OpenPI π0.5 checkpoint 使用 LeRobot 的 `guided` RTC 路径：旧块尚未执行的绝对
目标随请求送到服务端，经过 Astral 的 `DeltaActions`、归一化和维度填充后，在每个
flow-matching 去噪步约束新块前缀。节点继续执行旧块，收到新块后按推理期间实际发出的
策略行数续播。这个路径不要求 RTC 专项训练 checkpoint。

```bash
# GPU 主机；--rtc-guided 会同时预编译普通和 guided 两条推理路径
PYTHONPATH=<sdk>/VLA/openpi/src <openpi-env>/bin/python \
  MORLA/src/common/morla_policy_inference/scripts/serve.py \
  --model pi05 --checkpoint-dir <openpi_ckpt> --policy-config pi05_astral \
  --action-dim 8 --port 8001 --rtc-guided --rtc-execution-horizon 10

# 机器人侧；model 和 engine_mode 都要指定
ros2 launch morla_policy_inference policy_inference.launch.py \
  backend_type:=remote model:=pi05 engine_mode:=rtc \
  host:=<gpu-host> port:=8001
```

`rtc_execution_horizon` 必须与服务端 `--rtc-execution-horizon` 相同，握手时检查；
`rtc_queue_threshold`（默认剩余 30 行）、`rtc_max_guidance_weight`（默认 10）和
`rtc_prefix_schedule`（默认 linear）在 YAML 中配置。guided RTC 下节点自动关闭
引擎的时序融合和换块 anchor 平滑，避免给模型引导结果再次施加轨迹混合。
服务端没有 `--rtc-guided` 或模型不匹配时，节点在建立 POLICY 会话时直接报错。

**pi0/pi05 消融诊断：`mute_cameras`（yaml，JSON 数组）**——指定 collect label 不进入推理请求，
例如 `mute_cameras: '["left_wrist"]'` 会让客户端只发送 base。配套 OpenPI `AstralInputs` 对本次
缺失但配置中存在的固定槽补零，并设置 `image_mask=False`；这不同于 mask 为真的全黑 OOD 图像。
被静音相机也会从 `_state_ok` 的 images_missing 中排除，`image_required` 不会误挡。空 `[]` =
全部相机原样发送。GPU 服务端必须同步包含此兼容逻辑的 `openpi/policies/astral_policy.py` 并重启。

- `camera_image_size` 必须与模型 preprocessor 输入一致（本机 pickup_act_480=480；默认 224 崩）；
- 后端一次返回**完整 chunk**（方案 A），引擎按 50 行分块/预取，默认 `queue_async` 即 30Hz、
  图像上传每 chunk 一次（网络 -96%）、节点 loop ~1.3ms；
- **`jpeg_transport`（上行 JPEG，yaml 默认 true）**：camera 槽位在节点 letterbox 后编码成
  JPEG 字节（载荷 1.38MB→~0.2MB，WiFi 上行 ~106ms→~15ms），serve 端 `image_codec` 解码
  还原——像素 = 节点 RGB 再编码（二次 JPEG 有损，训练数据本身是 JPEG-90 解码，模型鲁棒）。
  `false` = 现状发 RGB（兼容直连 openpi 官方 serve 的退路）。benchmark/e2e 加 `--jpeg` 测真实路径。
- **`temporal_ensemble_coeff`（引擎级 ACT 时序融合，借鉴 lerobot ACTTemporalEnsembler）**：
  真机换 chunk 时把旧尾段与新头部按 `exp(-coeff·i)` 权重平均，消除切换跳变（缓解一卡一卡）。
  yaml 默认 `0.01`（ACT 推荐）；`0` = 关闭（硬切换，A/B 对比用）。A/B：
  `temporal_ensemble_coeff:=0.01` vs `:=0.0`。
- **`chunk_anchor_tol`（换 chunk 切换平滑）**：时序融合只平滑"新旧预测"，不平滑"预测 vs
  执行"——真机换 chunk 仍会"冲一下"（pi_cmds 实测 0.1-0.2 rad 尖峰，全在换 chunk 后 2-3
  行，多为收敛拉回：旧 command 开环漂移、新预测一步追向实测）。`chunk_anchor_tol>0` 时
  安装检测续播起点偏离"正在执行的旧 command" >tol → 前 `chunk_anchor_blend`（默认 4）行
  从旧值线性过渡到新轨迹，切换差被摊平。yaml 默认 `0.05`（≈稳态跟随差，正常不触发）；
  `0` = 关闭。A/B：`chunk_anchor_tol:=0.05` vs `:=0.0`。真实数据离线模拟：7 个尖峰
  0.1-0.2 rad → blend 后全部 ≤0.04 rad。
- **换 chunk 不再断流**：`_run_plan` 推理已移出引擎锁（慢推理/网络不再堵控制线程），
  续播用实测 consumed 对齐——换块空档≈0。若真机仍觉快段"跳"，把 `control_interp` 从 1 调
  到 2/3（节点把 30Hz 数据线性插值到 60/90Hz 下发，恢复采集时的细粒度）。
- 序列化用包内 vendored `protocol`（`__ndarray__`），client/serve/node 三端一致。

**真机指标自动记录**：推荐 `log_dir:=<root> log_tag:=<事件>`——每次 launch 自动建
`{log_dir}/{YYYYMMDD-HHMMSS}[_tag]/` 运行目录，把 `pi_metrics.jsonl`（指标）+
`pi_cmds.jsonl`（实际下发指令）+ `pi_control.jsonl`（逐控制 tick 诊断）+
`camera_diagnostics.jsonl`（streamer capture/tap + policy 图像 rx/decode）一起落进去：
区分每次记录、持久化到测试归档（不复用同一 /tmp 文件、重启即丢）。
也可用精确路径：`metrics_log_file:=/tmp/pi_metrics.jsonl`（或 yaml 配置），节点
每次发布 state（1Hz + 状态变化）把带时间戳的 JSON（含 `latency_ms.loop/obs_age`、engine
`pops/plans/last_plan_ms/server_timing/remaining`、`exec_events`）追加写该文件，并在终端打印
一行 `[MET]` 摘要（含 `plan_ms` 与 `srv`=服务端推理 total）。
**时间口径**：`engine.last_plan_ms` = 节点发起 infer（发送观测）→ 收到 action 的**端到端
往返**（remote 含网络上行+序列化+服务端推理+下行，本机实测 avg ~140ms）；`engine.server_timing`
= 服务端分项（prep/pre/infer/post/total），`last_plan_ms − server_timing.total_ms` = 网络+序列化
开销（实测 ~130ms）。
排障卡顿可分别指定 `joint_stream_log_file:=/tmp/pi_cmds.jsonl` 和
`control_diagnostics_log_file:=/tmp/pi_control.jsonl`。前者记录**每次实际下发**的关节指令值、
同轴观测 state、`send_kind=new_target|resend_last`；重复保持还会记录 `hold_reason`。后者每个
控制回调一行，记录 `tick_interval_ms/tick_late_ms/callback_ms`、`action`、state/image 各源龄期、
fresh/stale/missing 状态及 engine 队列快照。因此即使 `_state_ok()` 拒绝后完全没有下发指令，
也不会成为日志空洞。`camera_diagnostics_log_file:=/tmp/camera_diagnostics.jsonl` 把相机管线分成
`capture`、`collect_tap`、`policy_rx` 三层；使用 `log_dir` 时自动开启，无需额外测速终端。
命令也有完整留痕：launch 日志依次出现 `cmd_rx`（DDS callback 已收）、`cmd_exec`（控制 tick 已执行）
及其 queue delay；同一控制行的 `events` 包含 `cmd_execute`、`cmd`、`cmd_rx_seq` 与
`cmd_queue_delay_ms`。图像回调按相机分组并行，命令与控制定时器使用独立 callback group，避免 base
JPEG 解码饿死 left_wrist 或可靠命令订阅。
当某一帧经过安全层修正时，`pi_cmds.jsonl` 还会同时保存 `raw_targets`（模型/融合器给出的原始
目标）与 `safety_events`，而同 control tick 也会保存同名 `safety_events`；可直接区分模型输出越界、
关节限速与实际下发动作，不能只根据最终的 0/1 夹爪值推断模型行为。
跑完用绘图脚本直接看曲线、错位时间与平滑度：

```bash
/usr/bin/python3 MORLA/scripts/plot_inference_curves.py \
    --log /tmp/pi_cmds.jsonl --out /tmp/pi_curves.png \
    --metrics /tmp/pi_metrics.jsonl   # 可选：engine.plans 递增处 = 换 chunk 边界
# 输出：7 臂关节 state vs command 曲线 + 动作步长 + 夹爪 PNG（步长面板红点=尖峰、
#   绿虚线=换 chunk）；终端：state↔command 错位时间（互相关）、动作步长分布与
#   >阈值尖峰，每个尖峰标「距最近重规划」（判换 chunk）+ 类型（收敛拉回 = command
#   一步追向 state；模型突变 = 主动跳离）。--self-test 合成数据自测分类逻辑。
```
另可用（传感器话题**必须**加 QoS flag，否则收 0 条）：

```bash
ros2 topic echo /policy_inference/state --qos-reliability reliable
ros2 topic echo /left_arm/joint_commands --qos-reliability best_effort --qos-depth 1
```

换**机器人配置**（加右臂/换灵巧手/加腰头）时，改 `config/policy_inference.yaml` 顶部 robot
段，使其与采集当时的 `data_collect.yaml` 一致——观测布局/指令拆分自动跟随。

### 3) 键盘操作一览

| 键 | 动作 |
|---|---|
| `s` | 启动策略（需新鲜观测；缺图会告警不阻断） |
| `y` | 启动回放（用 `replay_source`/`replay_episode` 参数）；`y <path>[:<ep>]` 指定来源 |
| `t <文本>` | 设置语言指令 |
| `空格` | 暂停（机器人保持最后指令） |
| `n` | 恢复（策略：按当前实况重新规划；回放：重锚定偏移后续播） |
| `h` | 人在环路接管（对每个臂节点调 `~/prepare_reanchor`，全部成功后切 HUMAN） |
| `g` | 交还控制权（回策略/回放继续） |
| `x` | 停止 → IDLE |
| `q` | 退出键盘 |

**接管语义**：`h` 先调用每个臂节点的 `~/prepare_reanchor`（机器人原点 ← FK(当前关节)，VR 零点 ← 当前 VR 位姿），
遥操以增量方式从当前姿态接管——不会跳变。准备服务只建立锚点并返回一次性 token，不会提前武装；
任一节点准备失败则**不进入** HUMAN，策略进入 `POLICY_PAUSED`，不会续发可能已过期的旧 chunk。
全部准备成功后才提交 HUMAN；臂节点收到带匹配
token 的状态且关节/VR 输入仍新鲜时才开始发指令。中途失败会 disarm 所有遥操节点并暂停旧策略规划，
避免旧动作接着执行。`g` 返回：若之前是策略 →
按当前实况重新规划续跑；若是回放 → 从实际位姿重锚定 offset 继续剩余帧。

**VR 键位（推理活跃时，2026-09-17 起）**：`controller_start_gate`（左手 gripClick + 右手 A，
随遥操栈常驻）会订 `/policy_inference/state` 按 `activity` 路由——

| 按键 | 推理活跃（policy/playback） | 推理 HUMAN | 其它 |
|---|---|---|---|
| 左 grip | **HUMAN 接管**（`/policy_inference/cmd`="takeover"） | /teleop/start（重标定，现状） | /teleop/start（现状） |
| 右 A | 无动作（采集 start 走数采栈） | **释放**（`/policy_inference/cmd`="release"） | /data_collect/control="start"（数采栈） |

**关键**：策略活跃时 grip **不再**发 `/teleop/start`——否则会重新武装遥操、与策略双写
`joint_commands`。推理时想接管 = 按 grip；想交还 = 右 A（HUMAN 期间）。右 A 的释放路由
在 `controller_start_gate`（遥操栈常驻），**不依赖数采栈**；数采栈的 `vr_collect_control`
在非 HUMAN 时仍把 A 映射为采集 start。

**所有权握手**：`/policy_inference/state` 每次状态变更都带 `epoch`、单调 `sequence`、`sent_at`、
`transition_id` 和 pending 标志；遥操/夹爪/手部写入端拒绝乱序、过期或非法状态。进入策略、回放或
HUMAN 前，当前 schema 需要的每个写入端都必须回传同一 `transition_id` 的
`/teleop/ownership_ack`。默认 `ownership_ack_required:=true`；2 秒内未确认就中止切换并按状态回滚，
未确认的写入端不会获得新控制权。策略/回放确认期限为 2 秒；接管的重锚与所有写入端确认共用 3 秒期限。
首个策略动作也等新鲜关节反馈及异步首轮规划完成后才提交；
`stop` 在等待期间仍可立即处理。
`ownership_ack_required:=false` 只用于不连硬件的隔离测试。

**接管时的肘部行为（方案B）**：`~/prepare_reanchor` 使用同一重锚逻辑，把**臂角参考**重锚到当前配置并**保持**
（`astral_arm_teleop` 2026-09-17），HUMAN 后肘不自行摆动（实测方案A 的 EMA 过渡被感知
为"卡一下/臂自己在动"，0.07-0.42 rad/1s）；操作者手臂方向变化超 `reanchor_elbow_release_thresh`
才切回人肘跟随。**排障**：若接管后 web 显示 HUMAN 有 ~1s 延迟，看 policy_node 终端
`[takeover]` 三行时间戳（`send prepare` → `all prepare done` → `committed HUMAN`）定位是
响应慢还是提交/拆引擎慢。

**仲裁 / disarm 电平语义**：`/teleop/disarm` 是**电平闩锁**——`true` = 外部所有者（策略/Web 暂停）
占用指令流，`false` = 放行。policy_node 进入 POLICY/PLAYBACK 发 `true`，进入 HUMAN 或停止回
IDLE 发 `false`。arm/head 遥操与 web 都只在 `true` 时 disarm（`false` 不会误伤刚 re-anchor
武装的遥操），夹爪遥操门控同时监听 `disarm`(true 停发) 与 `/teleop/armed`(true 恢复)，因此
Web 的 pause/resume 与策略接管/交还互不踩踏。

**夹爪仲裁**：policy 运行期间若夹爪遥操（`astral_gripper_teleop`）也在跑，其
`gripper_teleop.yaml` 默认已设 `disarm_topic: "/teleop/disarm"` + `arm_topic: "/teleop/armed"`
——policy 持有指令话题时夹爪遥操停发；进入 HUMAN/IDLE 后 policy_node 发 disarm=false 放行
真人捏合/扳机，Web 恢复（armed=true）同样放行。

**Web 控制面（等价键盘，推荐）**：`morla_web_manager` 监控 tab 数采卡片下方有**推理模块**——
启动/重启/停止节点（配置 GPU 主机 IP/端口/图像尺寸/引擎模式/模型族/记录日志开关）+ 上表全部
命令按钮 + 任务输入，走 `/policy_inference/cmd`+`/task` 纯话题（与 `policy_keyboard` 等价，
CLI 启动的节点同样可控）+ `/policy_inference/state` 实时镜像。启动参数经 shell 元字符消毒
（防注入）。详见 `morla_web_manager/README.md`。

### 4) 回放采集数据

```bash
# LeRobot v2.1 数据集目录（语法 playback:<source>[:<episode>]）
ros2 topic pub /policy_inference/cmd std_msgs/msg/String \
    "data: 'playback:/data/datasets/xxx:0'" -1
# 或 aligned_data.h5 单文件（对齐流水线产物）
ros2 topic pub /policy_inference/cmd std_msgs/msg/String \
    "data: 'playback:/data/aligned/aligned_data.h5'" -1
```

回放帧率 = `dataset_fps`；播完保持末位 `replay_hold_s` 秒后自动回 IDLE。进入回放前会先校验
`action_dim` 与（h5/lerobot meta 带 schema 时）state 布局命名顺序，不一致直接拒绝并保持
当前活动——不会出现“总维数相同但块顺序不同”的静默错放。

**真机回放中的人在环测试**：回放运行时按键盘 `h`（或向 `/policy_inference/cmd` 发 `takeover`）可
冻结数据回放并切到 Quest 人工遥操；按 `g`（或发 `release`）后，会以当前实测机器人状态重锚并从
被中断的帧继续回放。也可以直接发送命令：

```bash
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'takeover'" -1
ros2 topic pub /policy_inference/cmd std_msgs/msg/String "data: 'release'" -1
```

这条路径适合在通过 schema 校验的数据集回放中验证 HUMAN 接管与续播；自动回归覆盖 HDF5 回放、
接管期间冻结帧索引，以及交还后恢复同一回放会话。

### 5) MuJoCo 真话题端到端冒烟（无需模型/相机）

`astral_mujoco_sim` + `policy_node`（stub 后端）全链路验证仲裁与回放（覆盖 policy 驱动、
pause 夹持、resume 恢复、h5 回放逐帧一致与播完自动 IDLE）：

```bash
cd /home/robot/loopkok/sdk/MORLA
source /opt/ros/humble/setup.bash && source install/local_setup.bash

# 先做一次 /tmp/e2e/replay.h5（h5py 写 action + attrs[fps]=30 即可，参考脚本注释）
ros2 launch astral_mujoco_sim astral_mujoco_sim.launch.py enable_viewer:=false &
install/morla_policy_inference/bin/policy_node --ros-args \
    --params-file install/morla_policy_inference/share/morla_policy_inference/config/policy_inference.yaml \
    -p backend_type:=stub -p engine_mode:=queue_sync &
/usr/bin/python3 scripts/run_inference_sim_smoke.py --replay /tmp/e2e/replay.h5
```

`ros2 run morla_policy_inference policy_node` 若报 “No executable found”，是该 colcon 布局把
console script 放进 `install/morla_policy_inference/bin/` 而无 resource index 所致，直接跑
上面 `bin/policy_node` 路径即可。

## 关键设计

- **观测/动作布局 = profile导出的 `morla_core.schema.FrozenSchema`**。state 向量按 `meta.json`
  冻结顺序拼装；`split_action` 把策略输出的绝对动作行拆回 arm/gripper/head 独立话题；
  waist 块无 v1 指令通道 → 显式 `UnsupportedActionBlockError`，拒绝猜测。
- **绝对动作行**。pi0.5/ACT 输出与采集的 next-state 同为绝对关节目标，因此 chunk 无缝替换
  无需混合；`engine` 只在“仍在执行中换新 chunk”时做延迟补偿跳行。
- **安全兜底独立于策略**：`SafeExecutor` 拦截 NaN/Inf、夹爪比值 [0,1]、关节限位与
  `max_joint_vel` 速率限制后才发话题（`max_joint_vel<=0` 关闭限速）。
- **绝对动作语义守卫**（`abs_action_min_scale`，默认 0.5）：机器人离开零位时 chunk 首行量级
  不得塌缩（检测后端把 delta 当绝对返回），违例即安全 stop。
- **模型无关**：`ObsBatch`（state+images+prompt）为中性输入；后端只负责输出与
  `state_dim` 同维的绝对 chunk。
- **仲裁先效应后提交（事件驱动）**：进入 HUMAN 前先对全部臂节点发起 `~/prepare_reanchor`（效应），
  它只建立锚点并返回 token，不提前武装；控制定时器轮询全部成功后提交状态机，臂节点收到匹配 token
  且反馈新鲜才输出。任一失败会 disarm 已武装节点并让原策略/回放进入暂停，要求重新规划，不恢复旧块。
  `~/cmd` 经队列由控制定时器串行消费，与 `_tick` 共享一把
  RLock——仲裁回调与控制循环不可能交错双写。
- **运行期观测门控**：POLICY 运行中 state 反馈缺失超过 `obs_stale_stop_s` 自动暂停（保持当前
  目标），而不是继续用陈旧观测盲推盲发。

## 推理质量优化（真机卡顿 → 流畅）

从真机推理"一卡一卡 / 任务固定卡点 / 换 chunk 冲一下"到基本流畅的完整过程：参数体系、
诊断工作流与实测数据。核心结论：**多数"卡顿"不是引擎 bug，而是 ①参数没生效（yaml 命名
空间）、②模型忠实复现了训练数据里的停顿节奏、③网络带宽限制换 chunk 频率**。

### 症状画像（按真机/日志可复现性）

| 症状 | 特征 | 根因层 |
|---|---|---|
| 换 chunk"冲一下" | 固定间隔，`pi_cmds` 实测 106~149ms 指令空档后接 0.1~0.15 rad 步 | 引擎锁跨推理（已修） |
| 换 chunk 后 2-3 行尖峰 | 0.1-0.2 rad 单关节步，全在 `engine.plans` 递增边界 | 收敛拉回（anchor_tol/coeff 修） |
| 任务固定卡点 | 到目标前/夹取后/放置前/释放后"停一下"，与换 chunk 无关 | **训练数据节奏**（数据修复） |
| 慢速平移"停一下走一下" | 低速段骤停再走 | 训练数据 + 链路延迟混合 |

### 根因链（按排查顺序）

1. **参数从未生效（最隐蔽）**：yaml 顶层键 `morla_policy_inference:` ≠ 节点名 `policy_node` →
   rclpy 按节点名匹配 `--params-file` 段，键不匹配**整份参数静默丢弃**，节点落回代码默认值
   （interp=1/coeff=0/tol=0/jpeg=false）。此前所有真机"调参没反应"都因此。**防复发**：节点
   启动行自报生效参数（`ctrl=..Hz coeff=.. anchor_tol=.. ... jpeg=..`），看到默认值即 yaml 没加载。
2. **换 chunk 指令断流**：`_run_plan` 锁跨 `backend.infer()` → 慢推理（远程 ~120ms）堵控制线程
   `tick()` → 空档。→ 推理移出引擎锁（锁内快照 + 锁外 infer + 锁内安装）。
3. **换 chunk 尖峰（收敛拉回）**：旧 chunk 开环预测漂移 + 观测过时（请求→推理→回传期间机器人
   已前进 4-5 行），重规划时新预测一步追回实测。时序融合（coeff）只平滑"新旧预测"、不平滑
   "预测 vs 执行"→ 叠加切换平滑（anchor_tol）。
4. **数据节奏（卡点根因）**：训练数据本身走走停停（pick_place_merged 实测 20.6% 帧速度
   <0.008 rad/帧），ACT 忠实学进停顿 → 真机在对应任务状态复现。→ 数据修复（见下）。

### 参数体系（policy_inference.yaml，全可 launch 覆盖，真机 A/B）

| 参数 | 默认 | 机制 | 调优方向 |
|---|---|---|---|
| `temporal_ensemble_coeff` | 0.05 | 时序融合：换 chunk 新旧预测指数加权（借鉴 lerobot ACTTemporalEnsembler） | 大=更平滑但"肉"；0=关 |
| `chunk_anchor_tol` | 0.05 | 切换平滑：安装时续播起点偏离**正在执行的旧 command** >tol → 前 `chunk_anchor_blend`(4) 行从旧值线性过渡 | 0=硬切换；真机仍跳可调大 |
| `control_interp` | 2 | 控制率 = dataset_fps×N（2=60Hz 插值下发，把 30Hz 大步拆半；3=90Hz） | 拆小大步、缓解肉眼卡顿 |
| `async_prefetch_ahead` | 25 | 每 (chunk−N) 行重规划并融合（小=重规划更勤、观测更新鲜，但推理频率高） | 直连网线小值；带宽紧大值 |
| `jpeg_transport` | true | 上行 camera 槽位发 JPEG（载荷 1.38MB→~0.2MB，RTT 主项是带宽×载荷） | false=原始 RGB（兼容官方 serve） |
| `engine_mode` | queue_async | 后台预取整 chunk（方案 A 后默认即 30Hz） | 时序融合 checkpoint 才需 queue_sync |

### 诊断工作流（真机排障三板斧）

```bash
# ① 落盘：state 指标 + 关节指令流（推荐 log_dir 模式：每次 launch 自动建运行子目录
#    区分记录、持久化；--metrics 供换 chunk 关联）
ros2 launch morla_policy_inference policy_inference.launch.py ... \
    log_dir:=<ws>/inference_test_logs/inference log_tag:=pick_place_test7
#    → <ws>/inference_test_logs/inference/20260916-153012_pick_place_test7/{pi_metrics,pi_cmds}.jsonl
# 精确路径模式：metrics_log_file:=/tmp/pi_metrics.jsonl \
#   joint_stream_log_file:=/tmp/pi_cmds.jsonl \
#   control_diagnostics_log_file:=/tmp/pi_control.jsonl
# ② 分析：尖峰换 chunk 关联 + 收敛拉回/模型突变分类（--self-test 先自测）
/usr/bin/python3 MORLA/scripts/plot_inference_curves.py \
    --log /tmp/pi_cmds.jsonl --metrics /tmp/pi_metrics.jsonl --out /tmp/pi_curves.png
# ③ 拆延迟：engine.last_plan_ms（端到端 RTT，state/metrics 均带）− server_timing.total = 网络+序列化
```

关键判读：`engine.last_plan_ms` = 发送观测→收到 action 的端到端（remote 实测 p50 41ms，
含 1.38MB/0.2MB 上行 + 服务端推理 ~10ms）；`latency_ms.obs_age` = 观测龄期（~23ms，观测本身
新鲜）；`latency_ms.loop` = 节点处理（~3ms）。尖峰分类：**收敛拉回** = command 一步追向 state
（旧 chunk 漂移 → 换 chunk 纠正，anchor_tol 对症）；**模型突变** = 主动跳离 state（真策略行为，
需看是否该动作本就快）。

### 实测（test4 → test5，同一任务两段录制）

| 指标 | test4（参数未生效） | test5（参数生效） |
|---|---|---|
| \>0.1 rad 尖峰 | 7 个 | **0 个** |
| 最大步长 | 0.200 rad | 0.084 rad |
| 控制率 | ~30Hz | ~50Hz（interp=2） |
| 端到端 RTT | ~140ms | ~56ms（jpeg） |

### 数据层（卡点治本）

模型复现的是训练数据节奏，引擎参数只能平滑、不能消除"模型在固定状态输出低速"。**治本**：
- `src/common/morla_data_collect/morla_data_collect/pipeline/repair_aligned.py`（aligned 层**弧长均匀选帧**，-27% 帧、平段 11.3%→0.6%、速度更匀
  且不加速、每帧跳变不放大；选帧而非插值 = 零图像-关节错位；`--speed-ref` 控速度/压缩比，
  默认臂维弧长均值=保留总时长，想更快给 0.07、别超 0.07 否则跳变放大回归）
- 修完重新 `data_pipeline process-act` → `act_train.sh` 训练 → 真机对比卡点是否消失

## 测试

```bash
cd /home/robot/loopkok/sdk/MORLA
src/common/morla_policy_inference/scripts/run_policy_inference_tests.sh
```

覆盖：robot_io（布局/组装/拆分/图像）、executor（NaN/限位/限速/reset）、backend（openpi
payload、错误处理、工厂）、engine（三种模式、后台规划、RTC 尾切、线程安全）、controller
（FSM 全部合法/非法迁移）、replay（读取两来源、PlaybackSession 步进/重锚）、node_flow
（进程内 mock 机器人 + stub 后端 + 真实 Trigger 服务的端到端状态流、HITL 成功/失败路径）。

仿真冒烟（无需模型/相机，stub 后端 + sim 提供 joint_states/命令消费即可）。

### 真实 ACT checkpoint 验证（部署前必跑，需 GPU + lerobot + 真实 checkpoint）

单测不加载真实模型，部署前用 `MORLA/scripts/` 下两个退出码脚本验证：

```bash
# A) 模型侧集成（py3.12 lerobot 环境，无 ROS）——后端 get_policy_class 加载 + 推理 +
#    引擎分块/ACT 队列/reset 重规划：
/home/robot/miniconda3/envs/lerobot/bin/python MORLA/scripts/run_act_integration.py \
    --checkpoint-dir <act_checkpoint>

# B) 完整节点端到端（py3.10 + ROS，需先启动 serve.py + policy_node）——
#    真实 ACT 驱动指令流 + pause/resume/stop，--check-server 先做推理往返预检：
/home/robot/miniconda3/envs/ros2/bin/python MORLA/scripts/run_act_e2e.py --check-server

# C) 推理正确性（py3.12 环境，真实数据集 vs 模型预测）：逐帧喂 episode 观测对比录制动作
/home/robot/miniconda3/envs/lerobot/bin/python MORLA/scripts/run_act_correctness.py \
    --checkpoint-dir <act_checkpoint> --dataset-dir "<act_dataset>"

# D) 远程链路基准（py3.10，先起 serve.py）：RTT 分布/真推理 vs 缓存/吞吐/GPU
/usr/bin/python3 MORLA/scripts/run_act_benchmark.py
#    含：握手、服务端分项 server_timing（prep/pre/infer/post）、网络vs模型分离、
#        本地序列化、线缆探测、RTT 抖动
```

- A 对应「模型能不能被这个包加载并产出有效动作」，B 对应「整条 ROS 链路 + 仲裁 +
  控制率」，C 对应「推理结果对不对」（真实数据 MAE 基线 0.004~0.005 rad），D 对应
  「远程推理快不快」（本机基线：稳态 RTT 4.3ms、吞吐 198 req/s、冷启动真推理 201ms）；
- **C 的两条硬规则**：每帧先 `backend.reset()`（否则 `select_action` 队列吐缓存行，
  MAE 被污染 0.20→0.004）；视频用 PyAV seek 按需解码（整段解码 OOM）；
- 真机部署流程：起 `serve.py --model <act|pi05>`（GPU 机）→ 起 `policy_node`
  （backend_type=remote，`camera_image_size` 与模型 preprocessor 一致）→ 跑 B 冒烟 → 接真话题。
