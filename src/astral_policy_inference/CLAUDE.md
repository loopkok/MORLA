# CLAUDE.md — astral_policy_inference 策略推理工程

策略部署（pi0.5/ACT）+ 采集数据真机回放 + 人在环路仲裁。先读本文件 → `README.md`
（用法）→ `docs/specs/2026-09-03-astral-policy-inference-design.md`（设计决策）→
`astral_ws/CHANGELOG.md`（症状→根因→修法）。改动本包前请确认不破坏下述不变量。

## 架构不变量

1. **CollectSchema 是唯一布局真源**。`robot_io.ObsLayout` 从 `astral_data_collect.schema`
   生成：订阅表、state 向量顺序、`split_action` 指令拆分。改机器人配置（臂/手/腰头/相机）
   只能通过给到 policy_node 的 schema 参数发生——别在推理侧写死维度或话题名。
2. **动作/状态都是“绝对量”**（arm rad + gripper 0..1，与采集 next-state 同序）。chunk
   换尾因此无需混合：`ActionEngine._install` 仅在“替换仍在执行中的 chunk”时按
   `latency_ms×policy_fps` 跳行补偿。回放同样发绝对动作，中断后续播用 offset 重锚定。
3. **仲裁先效应后提交，且事件驱动**。`~/cmd` 只入队，由控制定时器串行消费；所有 `_cmd_*`
   与 `_tick` 共享一把 `RLock`（仲裁与控制不可能交错）。策略/回放切换必须等当前 schema
   所需的每个 writer ACK 同一 `transition_id`；`takeover` 先异步调用所有
   `~/prepare_reanchor`（只准备锚点和一次性 token，不武装），定时器轮询全部成功才提交 HUMAN，
   writer 收到匹配 token 后才可发指令。失败/超时时 disarm writer，旧 policy chunk 过期，必须留在
   `POLICY_PAUSED` 等显式重规划，不能恢复旧块。任何分支都禁止“半接管”。
4. **暂停≠拆引擎，恢复必须重规划**。暂停只是控制器转 `*_PAUSED` 并 hold 最后指令；但
   暂停期间策略的“未来行”按旧观测已过期（机器人原地没跟），因此恢复（POLICY）一律
   `_start_policy()` 重建 chunk；恢复（PLAYBACK）一律按当前实况 `reanchor(offset)`。
5. **发布即仲裁权 + disarm 电平语义**：仅在 POLICY/PLAYBACK 及其暂停态发送指令
   （`_last_cmds`），进入时 `_disarm_teleop()`（true）；`/teleop/disarm` 是电平闩锁——
   进 HUMAN 或停止回 IDLE 发 false 放行遥操/夹爪。arm/head 遥操 `_on_disarm` 只认
   `Bool(true)`（false 不 disarm 刚 re-anchor 武装的节点），夹爪门控同时认 `armed=true`
   恢复（web pause/resume 不踩踏）。HUMAN 期间 policy_node 静默。
6. **QoS 与上游一致**：状态/图像/指令话题 `BEST_EFFORT depth=1`（`_SENSOR_QOS`）；
   遥操 disarm、state、task 用 RELIABLE+TRANSIENT_LOCAL（`_LATCHED_QOS`）保证新订阅者
   拿到当前模式/指令。

## 已解决的坑（症状 → 根因 → 防护）

| 症状 | 根因 | 防护 |
|---|---|---|
| takeover 卡死/首策略推理阻塞 stop | 同步服务或后端启动占用控制线程；旧实现没有 pending 所有权态 | 服务 `call_async` + 3s 超时；状态转 pending 后该周期不 dispatch；首轮策略规划在后台运行；stop 可打断等待。失败暂停旧策略并要求重新规划，不恢复旧 chunk |
| 节点一启动 `_backend_cfg` 空 → make_backend 报缺参 | `__init__` 在 `_load_robot_params()` 之后又把 `_backend_cfg={}` 清空 | 删掉重复赋值；构造顺序：declare → load(填充 cfg) |
| rclpy 直接声明 dict 参数报 “not one of allowed types” | Humble rclpy 无 struct 参数类型，Python 侧默认值只能标量/数组 | `camera_map` 改 JSON 字符串参数（`_parse_camera_map`），yaml 里写 JSON 文本 |
| pause 后 resume，策略行陈旧造成跳变 | 暂停期间机器人 hold，未消费的旧 chunk 行按旧观测过期 | `_cmd_resume` 对 POLICY 重建引擎、对 PLAYBACK reanchor |
| 回放中 pause→takeover 顺序 | 状态机允许从 `*_PAUSED` 直接 takeover（FSM 语义） | `release` 会恢复 paused 标志；节点 hold 当前姿态等 `resume` |
| HUMAN 后遥操臂“假接管”或迟到服务重新武装 | re-anchor 服务直接 arm，旧状态/迟到请求没有 transition 身份 | `~/prepare_reanchor` 只返回 token；HUMAN 状态携带匹配 token 才武装，且臂关节、VR、头部输入必须新鲜 |
| policy stop → IDLE 后夹爪/遥操再也发不了 | `_disarm_teleop` 是 TRANSIENT_LOCAL 闩锁，没人再发 false | `_cmd_stop` 也 `_release_teleop_gate()`（false）开闸；HUMAN/IDLE 同样开闸 |
| pinch 门控回归 web pause→resume | pinch 把 disarm 当电平门，web resume 只发 `/teleop/armed`=true 从不发 disarm=false | pinch 新增 `arm_topic`（armed=true 恢复）；yaml 默认同时设 disarm+arm |
| `_cmd_playback` 先拆引擎后加载失败 → 状态“POLICY”引擎为 None 静默冻结 | 拆除/teardown 在 load+校验之前 | 先 load + `action_dim`/`state_names` 校验，失败原地保持；全部通过才 request→disarm→teardown→装 session |
| 首轮策略规划失败后恢复旧引擎继续发旧动作 | 在异步首轮规划失败时直接恢复冻结的引擎 | 首轮规划成功前保持 hold；失败退回之前的 HUMAN/PAUSED 状态并关闭旧策略，要求操作者显式重规划 |
| 运行中丢 joint_states，策略继续用陈旧观测盲推盲发 | obs 门只在“进入策略”时检查，运行期无门 | POLICY 中 state 连续缺失 > `obs_stale_stop_s` → 自动 `_cmd_pause()`（保持+停引擎） |
| `_on_cmd`(MultiThreaded) 与 `_tick` 并发改 FSM/引擎 | 无共享锁，`_tick` 可能在 stop/takeover 置 `_engine=None` 后解引用 | `~/cmd` 入队 → 定时器 `_tick` 持 `RLock` 串行 drain+dispatch；传感器回调保持无锁只写缓冲 |
| stop/接管瞬间恰逢 queue_sync 边界推理 | 另一线程 `stop()` 直接 close 后端，与正在 `infer` 的线程并发 | `_run_plan` 整段持锁推理；`stop()` 先 join 后台线程再**锁内** close 后端 |
| 第二次连续回放不保持末帧 | `_hold_end` 只在 `_start_policy` 清，播完自动 stop 后残留旧时间戳 | `_cmd_playback` 与 `_cmd_stop` 都置 `_hold_end=None` |
| 回放总维数相同但块序不同会静默错放 | 只校验 `action_dim` | 带 schema（h5/lerobot meta）时比对 `ep.schema.state_names()` 与机器人 schema，不符拒绝 |
| 策略与夹爪遥操抢写 `/left_gripper/command` | `astral_gripper_teleop` 原来不看 disarm 一直发 pinch 比值 | pinch 节点 `disarm_topic`+`arm_topic` 仲裁门；policy 进 POLICY/PLAYBACK 发 disarm=true、进 HUMAN/IDLE 发 false |
| ACT 后端加载真实 checkpoint 报 "Can't instantiate abstract class PreTrainedPolicy" | lerobot 0.6.2 里 `PreTrainedPolicy` 是抽象基类，`from_pretrained` 直接调用无法实例化（两个 checkout 行为一致）；`test_backend.py` 从没覆盖过 `InprocBackend.open()` | `open()` 读 `config.json["type"]` → `get_policy_class` 解析具体类（ACTPolicy）再 from_pretrained；补 2 例 hermetic 回归（sys.modules 伪造 lerobot，无 torch） |
| 逐帧评估 ACT 正确性 MAE 高达 0.20 rad（实际模型只有 0.004） | `InprocBackend.infer()` 走 `select_action` 内部 50 行 action 队列：后续 infer 只吐上一 chunk 缓存行、对新观测不重新推理 | 逐帧评估每帧先 `backend.reset()` 清队列（节点运行时此行为正确=ACT 自管节奏，勿"修"）；正确性脚本已内置 |
| 正确性脚本把整段视频解码进内存 OOM 卡死 | 单个 `file-*.mp4` 含整个 chunk 所有 episode（~1 万帧 ≈15GB/相机） | PyAV `seek` 按需解码目标帧附近（O(1) 内存） |
| HITL 接管/重启后重新进 POLICY 重放 ~1.7s 陈旧动作（危险跳变） | serve.py 的 ACT 后端常驻，`select_action` 50 行队列跨客户端连接残留；节点新会话第一推理拿到的是上一会话缓存行 | serve.py 每个新连接 `backend.reset()` 清队列（openpi 协议无 reset 消息）；节点每次 POLICY 会话正好一个新连接 |
| ACT 部署控制率只有 13Hz（应 30Hz） | 两层：① ACT 后端返回 1 行 chunk（select_action 队列）→ 引擎分块失效；② `_planner_loop` 锁内 Event.wait 饿死控制线程 | ① 方案 A：`predict_action_chunk` 返回完整 chunk；② 锁修复（wait 移出锁外）。默认 queue_async 即 30Hz |
| `ros2 run`/`ros2 launch` 报 "No executable found" | 缺 `setup.cfg`，console script 装进 `bin/` 而非 ament 的 `lib/<pkg>/`（data_collect 有 setup.cfg 所以正常） | 补 `setup.cfg`（`[install] install_scripts=$base/lib/<pkg>`） |
| queue_async 控制线程饿死（瞬时推理也只有 ~15Hz；真实 ACT 0.5Hz） | `_planner_loop` 把 `self._stop.wait(0.005)` 写在 `with self._lock` 内——planner 空闲时几乎 100% 持锁，`tick()` 在锁上饿死。此前被 select_action 1 行 chunk（planner 一直重填）掩盖，方案 A 暴露 | `Event.wait` 移出锁外（空闲判定在锁内、等待在锁外）；回归 `test_queue_async_control_thread_not_starved_by_planner` |
| ACT 后端一次 infer 只回 1 行，引擎分块/预取/网络全浪费 | `InprocBackend.infer` 走 `select_action`（内部 50 行队列逐行吐），引擎拿不到完整 chunk | **方案 A**：改 `predict_action_chunk` 一次返回完整 chunk（`temporal_ensemble_coeff` 非 None 回退 select_action）；图像上传从每行一次变每 chunk 一次（-96%），queue_async 达 30Hz，loop 7.9→1.26ms |
| 后端把 delta 当绝对返回（openpi server 漏 AbsoluteActions/错 checkpoint），Jetson 静默跳到错误目标 | 节点信任 backend「返回绝对」契约，无语义断言；安全层 `joint_limits` 也未填充（只有 slew 限速） | **绝对语义守卫** `abs_action_min_scale`：机器人离开零位（|state|>1 臂维）时 chunk 首行量级不得塌缩（arm_scale≥0.5），否则 PolicyError→安全 stop；3 例单测 |
| 真机换 chunk 时"冲一下"（固定时刻 hold→lunge，pi_cmds 实测 106~149ms 空档后接 0.1~0.15 rad 步） | `_run_plan` 锁跨 `backend.infer()` 持有，慢推理（远程网络 ~120ms）堵住控制线程 `tick()` → 指令断流；且 `i0=round(latency_ms×fps)` 按"机器人前进了"估算，但锁内推理时机器人实际没动 → 超前跳 | **推理移出引擎锁**（锁内快照 obs + `_i_snap` 锚点 → 锁外 infer → 锁内安装）；续播索引用**实测 `consumed = _i − _i_snap`**；时序融合锚点用 `_i_snap`。queue_sync 的 `_plan_blocking` 外层仍持锁（阻塞语义不变）。3 例回归：慢推理不堵 tick、从 new[consumed] 续播、融合锚点对齐 |
| 换 chunk 后 2-3 行仍出现 0.1-0.2 rad 尖峰（temporal_ensemble 已开，pi_cmds 实测 7 个全在换 chunk 边界） | 时序融合只平滑"新旧预测"，不平滑"预测 vs 执行"——旧 command 开环 chunk 内漂移（观测过时 + 预测漂移），重规划时新预测一步追向实测（收敛拉回，跳后新 chunk[i0]≈state、cmd-state 0.003~0.055）；起点用"实测"的 blend 对收敛拉回无效（dev<tol） | **`chunk_anchor_tol` 切换平滑**（默认 0.05，yaml 开）：安装时续播起点偏离**正在执行的旧 command** >tol → 前 `chunk_anchor_blend`(4) 行从旧值线性过渡到新轨迹。真实数据离线模拟：7 尖峰 0.1-0.2 → ≤0.04 rad。3 例单测（旧值起步 blend / tol 内不触发 / 关闭） |

## 推理优化历程（2026-09-14 实录，按时间顺序）

> 给新会话/接手者：理解**为什么**有这些参数和机制（坑表是"症状→防护"，这里是"决策顺序 + 数据 +
> 教训"）。完整逐日记录见 `astral_ws/CHANGELOG.md`。
> **独立整理版（症状→根因→修法→数值 + 排查 checklist）**：`docs/act-inference-stutter-investigation.md`；
> **部署环境/后端/协议类坑**（跑不起来、回传不对、静默退化）：`docs/inference-deploy-pitfalls.md`；
> 全主题索引：`docs/README.md`。

**第 0 层：参数根本没生效（最隐蔽、最贵的教训）**
yaml 顶层键 `astral_policy_inference:` 而节点名是 `policy_node`——rclpy 按节点名匹配
`--params-file` 段，键不匹配**整份参数静默丢弃**，节点落回代码默认值。连续 4 轮真机"改了
coeff/tol/interp/jpeg 运动却不见变化"的根因全在这。**修法**：yaml 顶层键改 `policy_node:` +
节点启动自报生效参数（`ctrl=60Hz coeff=0.05 anchor_tol=0.05 ... jpeg=True`）。**教训**：
改参数后先看节点启动行确认生效，不要假设 yaml 被加载（Humble 无单键回退，静默丢）。

**第 1 层：换 chunk 指令断流（hold→lunge）**
`_run_plan` 锁跨 `backend.infer()`（远程 ~120ms）堵控制线程 → 每次重规划 ~120ms 指令空档。
**修法**：推理移出引擎锁（锁内快照 obs + `_i_snap` → 锁外 infer → 锁内安装），续播索引用实测
`consumed = _i − _i_snap`（替代 latency_ms 估算，后者在"控制线程被堵、机器人没动"时超前跳）。

**第 2 层：换 chunk 收敛拉回尖峰**
锁移出后空档消失，但仍有 0.1-0.2 rad 尖峰全在换 chunk 边界（`plot_inference_curves --metrics`
关联 `engine.plans` 递增确认）。机制：旧 chunk 开环漂移 + 观测过时（请求→推理→回传 ~130ms 期间
机器人已前进 4-5 行）→ 新预测一步追回实测。时序融合（`temporal_ensemble_coeff`，借鉴 lerobot
ACTTemporalEnsembler）只平滑"新旧预测"连续性，不平滑"预测 vs 执行"（旧预测已漂移）。**修法**：
`chunk_anchor_tol` 切换平滑——安装时续播起点偏离**正在执行的旧 command** >tol 则前 blend 行
从旧值过渡。实测 7 尖峰 0.1-0.2 → ≤0.04。**关键认知**：起点必须用"最后已发出的行 `_i-1`"而非
"下一行 `_i`"（后者超前一行，blend 起点错）。

**第 3 层：任务固定卡点 = 训练数据节奏**
尖峰清零后肉眼仍见 4 个**固定**卡点（到目标前/夹取后/放置前/释放后）。与换 chunk 无关（跨 2-3
chunk），是模型在对应任务状态输出低速轨迹——因为训练数据本身走走停停（pick_place_merged 实测
20.6% 帧速度 <0.008 rad/帧、208 个慢速段遍布）。**结论**：引擎参数只能平滑不能消除（卡点是
"模型预测"），治本在数据（`src/astral_data_collect/astral_data_collect/pipeline/repair_aligned.py`，见 README 数据层）。

**第 4 层：网络带宽（RTT 主项）**
`engine.last_plan_ms` 端到端 ~140ms，`server_timing.total` 服务端推理仅 ~10ms——差值是上行
1.38MB 原始 RGB 在受限链路的传输（WiFi 约百 ms 级）。**修法**：`jpeg_transport`（camera 槽位
编码 JPEG，载荷 ~7× 小，RTT 140→56ms）。

**顺手的诊断资产**（都沉淀成工具）：
- `metrics_log_file` / `joint_stream_log_file`：state 指标 + 关节指令流落盘（JSONL）
- `plot_inference_curves.py`：尖峰换 chunk 关联 + 收敛拉回/模型突变分类（`--self-test` 自测）
- `engine.server_timing` 透传：state/metrics 直接可拆"网络+序列化 vs 服务端推理"

**数据修复设计决策**：`repair_aligned.py` 从"插值"改成"选帧"——关节插值必然造成关节 vs 图像
错位（图像只能取离散帧），**选帧（弧长均匀取原始帧）零错位**；`--speed-ref` 控压缩比（小=更
温和更多帧）。

## 代码路径速查

```text
node.py      policy_node：参数/schema → 布局/后端 → 引擎 → 100Hz 控制定时器 _tick
             _policy_tick / _playback_tick / _emit_target(插值) / _send_targets_from(安全层)
             _cmd_{policy,playback,pause,resume,takeover,release,stop}  仲裁入口
engine.py    queue_sync 阻塞重填；queue_async 后台预取；rtc 后台滚切尾段（min_tail）
             TemporalEnsembler：ACT 时序融合（借鉴 lerobot ACTTemporalEnsembler），
             参数 temporal_ensemble_coeff>0 时换 chunk 加权平均消切换跳变；0=关
             chunk_anchor_tol：切换平滑（续播起点偏离旧 command>tol 时前 blend 行
             从旧值过渡到新轨迹，消收敛拉回/模型突变尖峰）；0=关
backend.py   make_backend(backend_type=remote|inproc|stub, model=act|pi05|...)；
             RemoteBackend(远程 client) / InprocBackend(本地进程内) / StubBackend
client.py    自包含 websocket client + vendored __ndarray__ 序列化（无 openpi_client 依赖）
replay.py    PlaybackSession：target()/advance()/reanchor(offset)/done
controller.py 纯 FSM：request/revert/snapshot；_TABLE 显式迁移表
scripts/serve.py   统一 serve：--model act(lerobot)|pi05(openpi)，同一 websocket 协议，
             替代 serve_act/serve_policy 分离入口；节点 backend_type=remote 连它
```

## 环境约束（改环境相关代码前必读）

- **rclpy 只有 py3.10**（ROS Humble），**lerobot 0.6.2 要求 py3.12**（代码用 PEP 695 泛型，
  py3.10 解析期 SyntaxError，无法 shim）——本机没有任何单解释器能同时跑节点 + 进程内策略。
- **传输/模型双维度命名**：`backend_type`=remote|inproc|stub（传输方式）；`model`=act|pi05|…
  （模型族，决定 serve/inproc 加载路径）。**不留** "openpi"/"act" 旧名。
- **remote 部署**：GPU 主机起 `scripts/serve.py --model <act|pi05> --checkpoint-dir <ckpt>`，
  节点 `backend_type=remote` + host/port 连它；pi05 需 openpi env + openpi 格式 checkpoint
  （含 params/ 或 model.safetensors + assets/<asset_id>/norm_stats.json）。
- **线协议自包含**：client/serve/node 三端都用包内 `protocol.py`（vendored `__ndarray__`，
  与 pip 版 `nd` 不互通但已统一）——**Jetson 无需再装 openpi_client**。
- **真机部署 launch 必传**：`backend_type:=remote host:=<gpu> port:=8001
  camera_image_size:=<模型输入>`。`camera_image_size` 不传静默 224（ACT 480 崩）；
  `engine_mode` 默认 queue_async 即 30Hz（方案 A 后无需改）。
- **指标排障**：launch 传 `metrics_log_file:=/tmp/pi_metrics.jsonl` → 节点把带时间戳的 state
  JSON（latency/engine/exec_events）追加写文件 + 终端打 `[MET]` 摘要行（node.py
  `_write_metrics`）。抓关节指令流用 `ros2 topic echo` 且**必须**加
  `--qos-reliability best_effort --qos-depth 1`（节点发布的 cmd/gripper 是 BEST_EFFORT depth=1，
  默认 RELIABLE 的 echo 匹配不上、收 0 条）。

## 测试与验证（改后必须全绿）

```bash
cd /home/robot/loopkok/sdk/astral_ws
src/astral_policy_inference/scripts/run_policy_inference_tests.sh   # pytest 收集包内全部 test_*.py
# 单跑节点流（需 ROS 环境）：
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 src/astral_policy_inference/scripts/run_policy_inference_tests.sh
```

- `test_engine.py` 刻意用 `autostart=False` + 直接 `_run_plan()` 把后台线程时序改成确定性，
  别把新用例写回 `time.sleep` 赌调度。
- 改推理控制流必须补 node_flow 用例（进程内 stub 后端 + 真实 Trigger 服务，
  覆盖成功/失败接管、暂停恢复重规划、回放重锚）。
- `astral_arm_teleop` 的 `~/prepare_reanchor` 与 legacy manual-only `~/reanchor` 都在
  `astral_arm_teleop/test_reanchor_teleop.py` 有单测。

### 真实 checkpoint 集成/端到端（部署前冒烟，需 GPU + lerobot + checkpoint）

单测不覆盖「真实 ACT 模型加载 + 推理」路径（那需要 checkpoint/GPU/lerobot 环境），
用 `astral_ws/scripts/` 下两个退出码脚本补齐（对齐 `run_inference_sim_smoke.py` 模式）：

```bash
# A) 模型侧集成（py3.12 lerobot 环境，无 ROS）：后端加载 + 推理 + 引擎分块/队列/reset
/home/robot/miniconda3/envs/lerobot/bin/python astral_ws/scripts/run_act_integration.py \
    --checkpoint-dir astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model

# B) 完整节点端到端（py3.10 + ROS，需先外部启动 serve.py + policy_node）：
#    ① serve.py --model act（py3.12 环境）② policy_node（backend_type=remote, camera_image_size=480）
source /opt/ros/humble/setup.bash
/home/robot/miniconda3/envs/ros2/bin/python astral_ws/scripts/run_act_e2e.py --check-server
```

- `run_act_integration.py` 对应后端 `get_policy_class` 修复的回归点 + 引擎队列语义
  （50 行 1 次真推理、reset 重规划），全部 PASS 才能说「模型能用」；
- `run_act_e2e.py --check-server` 先直连 serve.py 做推理往返（快速区分 server 问题 vs
  节点配置），再发 policy/pause/resume/stop 验证真实 ACT 驱动指令流 + 实测控制率 ~30Hz；
- 改 backend/engine/serve.py 后跑 A；改节点仲裁/话题/配置后跑 B。

### 真实数据推理正确性 + 远程链路基准（评估「结果对不对 + 快不快」）

```bash
# C) 正确性：喂真实 episode 观测给模型，对比录制的 next-state 动作（py3.12 环境）
/home/robot/miniconda3/envs/lerobot/bin/python astral_ws/scripts/run_act_correctness.py \
    --checkpoint-dir astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model \
    --dataset-dir "astral_data/act/pick up and place_480" --episode 0

# D) 链路基准：serve.py 远程推理的 RTT 分布/真推理 vs 缓存/吞吐/GPU（py3.10，先起 serve.py）
/usr/bin/python3 astral_ws/scripts/run_act_benchmark.py --requests 250
```

**C 的两条硬规则**（都踩过）：
1. **每帧必须 `backend.reset()`**——`select_action` 内部 50 行 action 队列，不清队列时后续
   infer 只吐上一 chunk 缓存行、不重新推理，逐帧 MAE 会被污染（0.20→0.004 rad 的差距）。
   节点运行时此行为是**正确**的（ACT 自管节奏），只有逐帧评估要 reset；
2. **视频用 PyAV seek 按需解码**——单个 file-*.mp4 含整个 chunk 所有 episode（~1 万帧
   ≈15GB/相机），整段解码会 OOM 卡死。
实测基线：ep0/5/20/46 全 PASS，整体 MAE 0.004~0.005 rad；链路 250 请求 0 失败、吞吐
198 req/s、稳态 RTT 4.3ms、冷启动真推理 201ms、GPU ~957MiB、节点端到端控制率 29.6Hz。

### 可观测指标清单（远程链路全层）

| 层 | 指标 | 获取方式 | 本机基线 |
|---|---|---|---|
| 载荷 | 1.38MB/请求 (8×4B + 2×480²×3B) | 计算 | 1.38MB |
| 网络 | RTT p50/p95/p99/std | `run_act_benchmark.py` | 3.9/5.1/12.1ms, std 11 |
| 网络 | 网络+序列化 = RTT−server_total | 同 | p50 1.94ms |
| 网络 | 线缆下限（小消息 RTT） | 同 | 0.14ms |
| 网络 | 本地序列化 pack/unpack | 同 | 0.26/0.04ms |
| 服务端 | server_timing prep/pre/infer/post | serve.py 响应 `server_timing` | 1.68/0.22/1.16/0.07ms |
| 服务端 | GPU util / mem | nvidia-smi（benchmark 采样） | ~6-10% / ~1GiB |
| 节点 | engine plans/pops/last_plan_ms/remaining | `/policy_inference/state` | 30Hz 时 4-7ms |
| 节点 | engine `server_timing`（serve 分项，随 state 透传） | 同 | total ~3-10ms；RTT−total = 网络+序列化 |
| 节点 | 控制率 pops/s | 同（增量） | queue_async 30Hz |
| 节点 | `latency_ms.loop` 新观测→指令处理耗时 | 同 | avg 1.3ms |
| 节点 | `latency_ms.obs_age` 真实关节反馈龄期 | 同 | avg ~16ms |
| 节点 | `exec_events` 安全层拦截（nan/clip/slew） | 同 | 空=无拦截 |
| 正确性 | 预测 vs 录制动作 MAE | `run_act_correctness.py` | 0.004~0.005 rad |
| 稳定性 | serve.py 连接计数/断线 | serve.py 日志 `connection #N` | 按会话数 |

埋点位置：backend `last_timing`（InprocBackend）/ `last_server_timing`（RemoteBackend）、
engine stats `last_plan_ms`（端到端 RTT）+ `server_timing`（服务端分项，state/metrics 均带，
可拆"网络+序列化 vs 服务端推理"）、node `latency_ms`（`_policy_tick` 测，排除夹爪自回显）、
serve.py `server_timing`+连接计数。

## 修改约定

1. 新能力先写失败用例复现，再修复转正（同 astral_arm_teleop 风格）。
2. 安全相关默认“平滑降级 + 可观测”：宁可 keep-last/hold，不跳变不静默。
3. 每个实质改动同步更新 `astral_ws/CHANGELOG.md`（当日日期下加粗条目：改动—动机—
   数值—验证）+ 本包 README 相关段落。
4. 显式 `/usr/bin/python3` + `source /opt/ros/humble/setup.bash`（miniconda python 会
   抢 import rclpy）。

## 遗留事项

- `queue_sync` 的阻塞重填发生在控制线程（同步模式下换 chunk 会有一次整块阻塞）；
  ACT 本地推理够快通常无感，超长推理建议 queue_async/rtc。
- 后端 `infer()` 尚无独立超时：openpi server 若“接受连接但永不应答”，首块规划/queue_sync
  重填会一直阻塞该调用方（async/rtc 只阻塞后台 planner 线程，控制定时器不受影响）。真机
  部署前可在 backend 加 deadline 并抛 `PolicyError`。
- openpi server 端机器人零改动，但 serve 进程与 checkpoint 加载在 GPU 主机（无断线自动重连
  前的“server 重启需重启节点”仍是操作注意事项）。
- 未有真实相机/驱动参与的本机联调；真机前先在 MuJoCo sim 用 stub→真后端过一遍
  astral_sim_pipeline（topic 仲裁、disarm、takeover 全链路）。
