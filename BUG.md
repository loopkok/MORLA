# BUG：Quest 位姿事件在输入订阅端被整批丢弃

- **状态**：已定位，未修复（本文件只记录问题与证据，不含代码改动）
- **发现版本**：工作树 `accf9c1`（`fix(input): keep the Quest controller event JSON-serializable`）
- **首次现象**：2026-11-02 23:14 的 `quest_right_cr5` 会话
- **影响面**：任何使用 `morla.input.quest` 事件流、且绑定带时效（`max_age_ms`）的装配

## 1. 现象

点击开始遥操时 Web 报：

```
重锚失败 control_1: reanchor requires fresh bound input
```

绑定 `control_1`：`source: quest.right_pose`、`target: arm.position_target`、
`controller: morla.cr5.cartesian_delta`、`max_age_ms: 250`。
即：重锚执行的那一刻，绑定要求的 `right_pose` 端口必须不超过 250 ms 龄期。

## 2. 现场证据（会话 trace）

会话目录：`~/.local/share/morla/logs/20261102T151434Z_quest_right_cr5_be88a7e068e345618683f7fc2ad92d59`

| 端口 | 更新频率 | 间隔中位 | p90 | 最大空档 | >250 ms 空档 |
|---|---|---|---|---|---|
| `right_pose`（位姿） | ≈14 Hz（16.7 s 内 241 次） | 21.9 ms | 167 ms | **1.022 s** | **17 段** |
| `right_hold`（握把） | 90 Hz | 11.2 ms | 13.1 ms | 89 ms | 0 |
| `right_axes`（摇杆） | ≈190 Hz | 5.3 ms | 9.5 ms | 87 ms | 0 |

- 5 次重锚全部失败并回滚，失败原因均为 `reanchor requires fresh bound input`。
- 失败那一刻最近一帧位姿的龄期：**4 次该端口根本不存在**（>250 ms 未更新），1 次为 109 ms 的旧快照。
- 全程 `input_error` 为 0、`input_connection` 只有 3 条 → 不是校验拒绝，也不是反复重连。
- 按键/摇杆同期满速 → 不是整体丢包，也不是连接断开。

## 3. 排除输入侧（头显与链路实测）

方法：独立 TCP 探针直接监听 `8000`（只说 `@q3sync` 握手、记录原始字节，不经 ROS），
在设备上已安装的构建上抓取头显实际发出的数据。

设备状态：`com.loopkok.morlaxr`、`versionCode=8`、`1.1.0`、
`lastUpdateTime=2026-11-02 15:41:35`（即含 App 侧「整帧一条消息」修复的构建）。

```
stream                 lines      Hz  gap_med  gap_p90  gap_max  >250ms  >100ms
body                     959    90.1     11.2     12.1     40.3       0       0
head                     959    90.1     11.2     12.0     29.4       0       0
left_buttons             959    90.1     11.2     12.1     40.1       0       0
left_controller          959    90.1     11.2     12.1     40.1       0       0
right_buttons            959    90.1     11.2     12.1     40.1       0       0
right_controller         959    90.1     11.2     12.1     40.1       0       0

right: 959 条位姿，959 条与按键行在 2 ms 内成对到达（100%）
left:  959 条位姿，959 条与按键行在 2 ms 内成对到达（100%）
```

结论：头显以 **90.1 Hz** 发送全部六路数据，间隔中位 11.2 ms、p90 12.1 ms、最大 40 ms，
**零个 >100 ms 空档**；每帧的 `controller` 与 `buttons` **100% 同一次 TCP 写入成对到达**。
App 侧与 USB / `adb reverse` 链路均无问题。

## 4. 复现（不依赖头显）

前提：接收器 `quest3_udp_mocap` 每帧向同一话题 `/quest3/{side}_teleop_events` 连发两条事件，
先 `pose`、后 `controller_state`；而 `morla_input/builtin_sources.py` 为事件流建立的订阅是
`BEST_EFFORT + KEEP_LAST depth=1`。

**实验一**：合成发布与该话题同格式的事件，90 Hz × 2 条（10 s：pose 900 + controller_state 900），
两个订阅者同进程、同时接收：

| 订阅深度 | pose | controller_state |
|---|---|---|
| depth 1 | **0.1 /s** | 64.2 /s |
| depth 20 | 64.2 /s | 64.2 /s |

**实验二**：接真实 `RosInputManager` + `QuestEventSource`（与 Web 输入栈同一套代码），
订阅同一条合成事件流 10 s：

| 订阅深度 | 收到帧数 | 带 `right_pose` 的帧 | 断连 |
|---|---|---|---|
| depth 1 | **0 帧** | 0 | 1 |
| depth 16 | 1292 帧（129 /s） | **1292（100%）** | 1 |

depth 1 一帧都出不来的原因：`QuestEventSource` 只在收到 `pose` 事件时才确定该手的
`controller` / `hand` 模式，位姿进不来时模式始终为 `None`，随后的 `controller_state`
一律被 `row['source'] != self.mode[side]` 丢弃。

## 5. 根因

`depth=1` 的 `KEEP_LAST` 队列只保留最新一条样本。每帧先发的 `pose` 都会被后发的
`controller_state` 覆盖，消费端能取到的几乎永远是后者：

- 位姿在 Web 侧退化为 1–23 Hz，且分布与按键的 90 Hz 完全脱钩（现场 §2、复现 §4 一致）；
- 绑定的 `max_age_ms: 250` 因此频繁失配，重锚/开始遥操被拒。

该现象与 2026-11-02 修复的 App 侧问题（共用一个 `TelemetryChannel`、每帧提交四条消息、
通道只留最新一条导致右手柄 9–20 Hz）**症状同型但层次不同**：App 侧已修复并通过本次实测
（§3 成对到达 100%）；本节问题位于接收器 → Web 的订阅环节。

## 6. 未采用的候选方向

以下都只是候选，没有一条落地：

1. 事件订阅深度 1 → 16（`KEEP_LAST` 16 的最坏积压为 16 条 = 8 帧 = 89 ms，
   小于 `QuestEventSource` 的 250 ms `max_age_ns`）；
2. 接收器侧把 `controller_state` 改为「变化时发 + 低频心跳」，降低每帧事件条数；
3. 位姿改由状态话题 `/quest3/{side}_controller_pose`（`depth=1`、永远最新）供给，
   事件流只负责按键/握把；
4. 事件流拆成 pose / state 两条话题，各自 `depth=1`；
5. 提高消费端吞吐（多线程执行器 / 更轻的回调）。

选择时必须保持 `QuestEventSource` 的时效语义：任何缓冲方案的积压都必须显著小于
`max_age_ns`（250 ms），否则缓冲事件会变成 `Queued/stale Quest event` 并触发断连。

## 7. 验收方法

修复后在现场重启工作站，确认：

1. 开始遥操不再报 `重锚失败 control_1`；
2. 行走/转身过程中机械臂不出现走走停停；
3. 新会话 `trace-*.jsonl` 中 `right_pose` 与 `right_hold` 的更新频率同量级（均接近 90 Hz）。

## 8. 附录：本次使用的临时工具

均在仓库外，未纳入版本库：

| 工具 | 用途 |
|---|---|
| `quest_wire_probe.py` | 监听 `8000`，只做 `@q3sync` 握手，记录每个源的频率/间隔/配对 |
| `analyze_wire.py` | 统计与源、间隔分布，判定位姿与按键是否同帧成对 |
| `fake_event_pub.py` | 按接收器节奏合成 `pose` + `controller_state` 事件 |
| `ros_topic_probe.py` | 同一话题上并排挂 depth 1 / depth 20 订阅者做对照 |
| `verify_input_frames.py` | 用真实 `RosInputManager` + `QuestEventSource` 统计收到的帧与端口 |
