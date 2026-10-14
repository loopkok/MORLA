# Astral Policy Inference 推理功能包 — 设计文档

日期：2026-09-03 · 状态：approved（用户已批准设计）

## 1. 目标（Goal）

在 `astral_ws` 中新增 ROS 2 功能包 `astral_policy_inference`，面向已完成的遥操数采 + 数据收集 + 训练链路，提供：

1. **策略部署**：换 checkpoint（openpi pi0.5 或 lerobot ACT）即可换模型，不改其它配置。
2. **同步 / 异步 / 实时分块（RTC）** 三种执行引擎（节点内，参考 lerobot rollout 形态）。
3. **真机/仿真回放**：把采集的数据（绝对动作轨迹）按节奏回放到机器人；回放与策略推理共用同一执行器与模式机。
4. **人在环路控制**：推理/回放过程中支持策略暂停 → VR 接管（增量控制，重新记录机器人原点与 VR 位姿零点）→ 控制权交还策略。
5. 与 `astral_data_collect`（schema 真源）、`astral_arm_teleop`（VR 遥操/接管）、`astral_robot_control` / `astral_mujoco_sim`（机器人执行侧）适配；全链路 schema 驱动，配置变更自动跟随。

## 2. 设计决策（用户已确认）

| 决策点 | 结论 |
|---|---|
| 后端 | openpi pi0.5（远程 websocket server）+ lerobot ACT（进程内 `from_pretrained`），统一 `PolicyBackend` 抽象 |
| 拓扑 | 同一套代码支持桌面+MuJoCo 仿真 / Jetson 真机两类主机，launch 参数切换；pi0.5 在 4090 主机远程服务 |
| 引擎 | 节点内执行引擎：`queue_sync` / `queue_async` / `rtc`（动作队列 + 后台重规划） |
| 回放 | v1 仅真机/仿真回放记录的绝对动作轨迹（含夹爪），支持暂停/接管；不含"录好观测喂模型离线评测" |
| 接管 | 扩展 `astral_arm_teleop` 增加 `~/reanchor` 服务；仲裁由推理包统一负责 |
| 触发 | ROS 服务/话题 + 键盘热键；v1 不做 web 集成 |

## 3. 架构

分层：ROS 层薄，核心逻辑纯 Python 可离线测试。

```
astral_ws/src/astral_policy_inference/
  astral_policy_inference/
    robot_io.py      观测/动作适配（纯逻辑，复用 astral_data_collect.schema.CollectSchema）
    executor.py      指令执行器（限速/NaN/限位防护，动作向量→分块写入值）
    backend.py       PolicyBackend 抽象 + OpenPiServerBackend + LerobotActBackend + factory
    engine.py        执行引擎 queue_sync / queue_async / rtc + 线性插值
    controller.py    模式状态机（IDLE/POLICY/HUMAN/PLAYBACK + PAUSED），回调注入
    replay.py        PlaybackSource：读 v2.1 / aligned_data.h5 单段 → 帧迭代器
    node.py          ROS 节点：yaml→参数、订阅/发布/服务接线、~/state 发布
    keyboard.py      热键节点
  config/policy_inference.yaml
  launch/policy_inference.launch.py
  package.xml / setup.py
  test/*.py
  README.md / CLAUDE.md
```

配套改动：
- `astral_arm_teleop/astral_arm_teleop_node.py`：新增 `~/reanchor` Trigger 服务——从当前 `joint_states` FK 重锚 `robot_init_pos/rot` + 用当前 VR 位姿重设 `vr_init` 零点并 arm（复用求解器切换时的重锚写法），保证接管无跳变。
- 变更记录：astral_ws/CHANGELOG.md、新包 README/CLAUDE。

## 4. 观测 / 动作适配（robot_io.py）

- 复用 `CollectSchema`（`astral_data_collect`）：推理包加载一份与 `data_collect.yaml` 同构的 robot config，构建同一 `state_blocks()/state_names()`。换右臂/换手/加腰头/换相机 → 只改这一份 yaml，obs 布局、动作拆分、topic 对应全部自动跟随。
- 观测源（在线实时）：
  - 臂 `{side}_arm` ← `/{side}_arm/joint_states`（rad）。
  - 末端 gripper `{side}_ee` ← 跟踪 `/{side}_gripper/command`（Float64 闭合比 0..1；HITL 交还后自动同步人捏的值）。
  - 末端 wuji（可选）`{side}_ee` ← `/{side}_hand/joint_states` 20 维。
  - 腰/头 ← `/astral/joint_states` 18 维 body 切片（`BODY_WAIST_SLICE`/`BODY_HEAD_SLICE`）；头在仿真无 body 时回退 `/head/joint_states`（对齐 align 的可用性选择逻辑）。
  - 相机 ← `/quest3_video_streamer/collect/{label}`（`sensor_msgs/CompressedImage`，JPEG），解码 + `resize_with_pad` 到槽位尺寸。`camera_map` 参数映射模型槽→相机 label，默认 `base_0_rgb←base`、`left_wrist_0_rgb←left_wrist`（右腕槽 3 相机模型启用后可加 `right_wrist_0_rgb←right_wrist`）。
- 动作写：
  - `{side}_arm` → `/{side}_arm/joint_commands`（JointState.position[7]；BEST_EFFORT depth=1，与 teleop QoS 一致）。
  - gripper `{side}_ee` → `/{side}_gripper/command`（Float64 闭合比）。
  - wuji `{side}_ee` → `/{side}_hand/joint_commands`。
  - 腰/头：v1 推理/回放仅支持具有独立指令话题的块；若 schema 含腰且策略动作含腰（v1 不含），启动时报不支持错误（防静默错写）。头块如有独立话题则写入 `/head/joint_commands`。
- 加载时校验：obs 向量 `names` 与真实块布局逐一核对，错配拒绝启动。

## 5. 后端（backend.py）与执行引擎（engine.py）

### 后端抽象

```python
class PolicyBackend(Protocol):
    def infer(self, obs: dict) -> np.ndarray: ...   # (T, A) 绝对动作，T=chunk, A=action_dim
    def reset(self) -> None: ...
    def close(self) -> None: ...
```

- `OpenPiServerBackend`：`openpi_client.WebsocketClientPolicy(host, port)`；发送键 `observation/state` + `observation/camera/{slot}`（uint8 HxWx3）+ `prompt`；返回 `actions[:T]`（`(50, action_dim)`）。权重在 server 端（`scripts/serve_policy.py`），机器人端零改动。
- `LerobotActBackend`：`PreTrainedPolicy.from_pretrained(checkpoint_dir)` + `make_pre_post_processors`；每步 `prepare_observation_for_inference` → preprocessor → `policy.select_action`（内含 ACT 动作队列/时间集成）→ postprocessor 绝对动作。换目录即换模型。
- `factory`：由 `backend.type` + checkpoint/URL 创建；缺依赖（lerobot/torch/openpi_client）报明确错误。

### 执行引擎

- `queue_sync`：控制环每 tick 从队列取 1 步；队列空则阻塞 `infer` 取整块。
- `queue_async`：后台线程预填队列，tick 不阻塞。
- `rtc`：后台连续重规划 + 延迟感知 chunk 替换。因 pi0.5 与 ACT 输出**绝对位置目标**，新块直接替换剩余段无跳变。
- 支持控制率 = N×`dataset_fps` 的线性插值（复刻 lerobot `ActionInterpolator`）。
- `engine_mode` 参数选择；pi0.5 建议 `queue_sync`/`rtc`，ACT 可用 `queue_async`。

## 6. 模式机与人在环路（controller.py）

状态：`IDLE → POLICY / HUMAN / PLAYBACK`（各可 `PAUSED` 冻结保持）。

仲裁原则：`/{side}_arm/joint_commands` 为最后写者赢；任意时刻仅一个写者。

- 进入 POLICY/PLAYBACK：先发 `/teleop/disarm`（true→false 边沿）停 teleop，再开始发布。
- 接管（→HUMAN）：
  1. 清空动作队列 + `backend.reset()`（丢弃按接管前旧姿态规划的绝对块）；
  2. 对每个启用臂侧的 teleop 节点调用 `~/reanchor`；
  3. teleop 从当前机器人姿态 + 当前 VR 位姿开始增量控制。
- 交还（HUMAN→POLICY）：发 `/teleop/disarm` 停 teleop；恢复后首个 tick 以当前实时 state 重新 `infer`（openpi delta 掩码以请求 state 为锚 → 新块自然衔接）。
- 触发：`~/cmd`（`std_msgs/String`，语义同 data_collect 的 control topic）+ `policy_keyboard` 热键；`~/state` latched JSON 发布模式/计数/task/安全字段。

### 接管时序伪码（供实现锁定）

```
HUMAN_entry:
  engine.flush(); backend.reset()
  for side in schema.arms:  call f"/astral_arm_teleop_{side}/reanchor" (Trigger)
HUMAN_exit:
  pub /teleop/disarm: Bool(True); Bool(False)   # 停 teleop
  engine.resume_from_live_state = True
```

## 7. 真机回放（replay.py）

- 源：LeRobot v2.1 本地数据集（repo 目录 + episode idx）读 `action` 列；或 `aligned_data.h5` 的 `action`。两者均为绝对目标（与 driver 直接消费格式一致）。
- 按 `dataset_fps` 节奏帧迭代；走同一 executor + 模式机（可暂停/接管/交还）。
- v1 不含离线评测模式。

## 8. 安全与观测（executor + controller 横切）

- 每步 delta 限速（`max_joint_vel`）、限位 clip、NaN/Inf 屏障、末帧保持。
- 后端超时/失败 → 停发 → `IDLE`；driver `command_timeout_s` 兜底保持。
- 参考相机低帧率告警、joint_states 缺失告警。
- `~/state` 输出模式、计数、task、安全事件。

## 9. 测试与对抗审查

- 离线单测（uv openpi venv / pytest）：obs 适配×schema 变化、executor 拆分与防护、engine 三态 + 插值、controller 全模式转移、replay 节奏/列校验；lerobot/torch `importorskip`。
- 节点层（系统 `/usr/bin/python3` + mock，仿 `test_node_guards.py`）：yaml 加载、话题/服务接线、`~/cmd` 转移。
- 集成冒烟：astral_mujoco_sim + 本节点回放真实采集段。
- 对抗审查：schema 突变全链、错相机 key、缺话题、后端不可达、checkpoint 维度不符、RTC 延迟边界、无 VR 接管被拒。

## 10. 非目标（v1）

- web monitor 集成（只发 `~/state` 留好口子）。
- 腰/头纳入策略动作（采集侧可选，推理侧 v1 报不支持）。
- "录好的观测喂模型离线评测"模式。
- wuji 手策略推理端到端（obs/write 抽象已留，端到端待真机 wuji 配置再验）。

## 11. 实现状态（2026-09-03，代码完成）

本设计已按上述决策实现，新增/变更与初稿的差异如下（详见包内 CLAUDE.md/README.md 与 astral_ws/CHANGELOG.md）：

- `camera_map` 采用 **JSON 字符串** 参数（Humble rclpy 无 struct 参数类型），运行时由
  `PolicyNode._parse_camera_map` 解析成 `{模型槽位: quest3 collect label}`；键是模型侧相机名，
  值是采集/数据集侧 label（训练数据 `observation.images.<label>` 也按 label 命名）。
- 新增 `stub` 后端（`backend_type: stub`），供无 GPU/相机的本机与 MuJoCo 冒烟联调用。
- **恢复语义**（实现期修正）：暂停后 resume，POLICY 一律按当前实况重建 chunk（暂停 hold 期间
  旧 chunk 余行按旧观测过期），PLAYBACK 一律按当前实况 `reanchor(offset)` 续播；`release`
  交还时同理（策略重规划 / 回放重锚）。
- **接管采用“事件驱动先效应后提交”**：`~/cmd` 只入队，由控制定时器持 `RLock` 串行消费；
  takeover 先异步发起全部 `~/reanchor`（冻结策略发射防双写），定时器轮询到全部成功才原子提交
  HUMAN；任一失败/3s 超时事务性回滚：重新 disarm 已武装的遥操节点、恢复被冻结的引擎，就地留
  在原状态。takeover 前要求新鲜 joint state（拒绝陈旧 q_cmd 锚点）。
- **`/teleop/disarm` 电平语义（对抗审查 critical 修复）**：true=外部所有者占用指令流、
  false=放行。policy 进 POLICY/PLAYBACK 发 true、进 HUMAN/停止回 IDLE 发 false；
  arm/head 遥操 `_on_disarm` 只认 `Bool(true)`；pinch 夹爪门控同时认 `/teleop/armed`=true
  恢复（web pause/resume 不踩踏）。
- **运行期观测门控**：POLICY 中 state 反馈缺失超过 `obs_stale_stop_s` 自动暂停（保持），
  不继续用陈旧观测盲推盲发。
- **回放入口校验前置**：先 load + `action_dim` 与（带 schema 时）`state_names` 布局顺序校验，
  失败原地保持；`_start_policy` 先验证 obs→冻结旧引擎→新引擎成功后才 disarm+换新，失败
  `revert()` 恢复资源（Controller 快照含 `_interrupted`）。
- 测试面：79 例单元/进程内功能测试（含 node_flow 的真实 Trigger 服务接管成功/拒绝回滚路径、
  运行期观测丢失自动暂停、disarm 电平顺序、回放 schema 拒绝、连续回放末帧保持）全绿；
  `astral_arm_teleop/test_reanchor_teleop.py` 覆盖 `~/reanchor`。
\n
