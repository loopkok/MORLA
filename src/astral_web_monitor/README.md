# astral_web_monitor

非侵入式 Web 监控面板：通过浏览器启动/暂停/停止遥操作栈，实时查看关节状态、指令频率、健康巡检与实时图表。

**不修改任何现有功能包的控制面**（只调用/订阅各包自己暴露的接口）。通过已有接口执行以下工作：

1. **只读订阅**现有关节话题（`joint_states` / `joint_commands`）
2. **发布到已有控制话题** `/teleop/armed`、`/teleop/disarm`（暂停/恢复/急停）、`/teleop/start`（外部启动闸门）、`/teleop/home`（HOME 归位到零）、`/teleop/init`（工作位，途经点）、`/teleop/init_direct`（段间回位，直达）
3. **subprocess 启停** `ros2 launch`（与命令行操作等价）
4. **只读健康巡检**：估算各源状态流/指令流频率与新鲜度，不下发任何命令
5. **视频回传门控**：调用 `quest3_video_streamer` 自己暴露的 `~/set_push_enabled` 服务 / `~/active_cameras` 话题（选路），并镜像其 `~/gate_state` 状态
6. **离线数据处理**：在独立工作副本调用 `astral_data_collect.pipeline`，从 raw 检查、对齐、可选清洗到 OpenPI/ACT 导出，显示日志、报告和图像预览。

```text
┌─────────────── astral_ws 现有功能包（不修改）──────────────┐
│  quest3_mocap → arm_teleop → /{side}_arm/joint_commands   │
│                          → /teleop/armed ◄──┐              │
│                          → /teleop/disarm ◄─┤  (已有话题)   │
│  pinch_gripper → /left_gripper/command     │              │
│  wujihand_retarget → /right_hand/joint_commands            │
│  driver → /{side}_arm/joint_states, /astral/joint_states  │
└────────────────────────────────────────────────────────────┘
                        │ 订阅(只读)        ▲ 发布(disarm/arm)
                        ▼                  │
┌─────────────── astral_web_monitor (本包) ──────────────────┐
│  monitor_node (rclpy)                                      │
│   ├─ Sub: joint_states ×5, joint_commands ×4              │
│   ├─ Pub: /teleop/armed, /teleop/disarm  (仅暂停/恢复时发)  │
│   ├─ RateCounter: EWMA 指令 Hz 估计                        │
│   └─ LaunchManager: subprocess ros2 launch ...             │
│         │                                                  │
│         ▼  WebSocket 30Hz                                  │
│  web_server (FastAPI + uvicorn :8080)                     │
│   ├─ REST /api/v1/*  → 启停/暂停/恢复/预设/健康              │
│   ├─ WS   /ws/telemetry → 实时推送关节/HZ/日志              │
│   └─ Static / → web/dist (React SPA)                      │
└────────────────────────────────────────────────────────────┘
                        │ WebSocket
                        ▼
                  浏览器 React UI
```

## 架构

| 层 | 技术 | 说明 |
|----|------|------|
| ROS 节点 | rclpy（后台线程） | 只读订阅 + 线程安全快照 + pause/resume 发布 |
| Web 后端 | FastAPI + uvicorn | REST 控制 + WebSocket 实时推送 + SPA 静态托管 |
| Web 前端 | React 18 + Vite | 单例 WebSocket + `useSyncExternalStore`（30Hz 不穿透重渲染） |
| 进程模型 | 单进程 | rclpy spin 线程 + uvicorn 主线程，无需多进程 IPC |

## 话题契约

### 只读订阅（不干扰数据流）

| 话题 | 类型 | 用途 |
|------|------|------|
| `/left_arm/joint_states` | JointState | 左臂 7-DoF 显示 |
| `/right_arm/joint_states` | JointState | 右臂 7-DoF 显示 |
| `/left_gripper/joint_states` | JointState | 左夹爪状态（driver 回显的指令角） |
| `/right_hand/joint_states` | JointState | 右灵巧手 20-DoF |
| `/head/joint_states` | JointState | 头部 yaw/pitch（摇杆遥操回显/真机 OBS） |
| `/astral/joint_states` | JointState | 全身 18-DoF（含腰） |
| `/left_arm/joint_commands` | JointState | 左臂指令 Hz |
| `/right_arm/joint_commands` | JointState | 右臂指令 Hz |
| `/left_gripper/command` | Float64 | 左夹爪闭合比指令 Hz（rad 映射在 driver） |
| `/right_hand/joint_commands` | JointState | 右手指令 Hz |
| `/head/joint_commands` | JointState | 头部指令 Hz（head_teleop_node） |

- QoS：**BEST_EFFORT**（SensorData）
- 灵巧手话题名可配置（`ASTRAL_WEB_MONITOR_HAND_NAME`，默认 `right_hand`）

### 发布（暂停/恢复/外部启动，已有话题）

| 话题 | 类型 | 触发 |
|------|------|------|
| `/teleop/disarm` | Bool(True) | 点"暂停" |
| `/teleop/armed` | Bool(True) | 点"恢复"（仅"暂停"后可恢复；VR 看门狗 disarm 后会被臂节点拒绝，需点"开始遥操"重标定） |
| `/teleop/start` | Bool(True) | 点"开始遥操"（一次性，记录 `vr_init` 并 arm） |
| `/teleop/home` | Bool(True) | 点"HOME"（一次性，双臂沿 init_pose → init_waypoints → 零位 收回并 disarm） |
| `/teleop/init` | Bool(True) | 点"工作位"（一次性，双臂沿 init_waypoints → init_pose 走到初始工作位并 disarm；**途经点路径**；启动自动归位关闭后的手动替代） |
| `/teleop/init_direct` | Bool(True) | 点"段间回位"（一次性，双臂**不经 init_waypoints**、直接回到 init_pose 并 disarm；**直达路径**，数采段间快速回位） |

- QoS：`/teleop/armed`、`/teleop/disarm` 为 **RELIABLE + TRANSIENT_LOCAL**（latched，晚启动的臂节点也能收到）
- `/teleop/start`、`/teleop/home` 为 **RELIABLE + VOLATILE**（**非** latched 一次性触发，避免晚加入的臂节点收到旧信号自动开始/自动归位）
- 视频门控：`/quest3_video_streamer/active_cameras`（String，**latched**）发布选路；`/quest3_video_streamer/gate_state`（String JSON，latched）只读订阅镜像状态；`/quest3_video_streamer/set_push_enabled`（SetBool）服务调用总开关
- **暂停只影响臂**：`/teleop/disarm` 只作用于 `astral_arm_teleop_node`，夹爪和灵巧手节点无 disarm 接口，继续运行
- **开始遥操**：配合 `astral_arm_teleop` 的 `require_start_signal:=true`——启动预设后臂节点只跟踪 `vr_current` 不记零点；手摆好初始位姿后点此按钮，臂节点用当前 pose 记 `vr_init` 并 arm。再点一次 = 重新记零点（re-center）
- **无条件发送**：`/api/v1/teleop/start` 始终发布 `/teleop/start`，**不**检查 launch 是否经本监控启动。臂节点是唯一裁判：homing 中或无 VR pose 时会忽略并告警。因此无论遥操由本监控的预设启动还是从外部 CLI 启动，此按钮均可用
- **HOME（归位到零）**：发布 `/teleop/home` 前先 disarm，臂节点沿 init_pose → init_waypoints → 零位 慢速收回并停在零位；仅当前预设 RUNNING/PAUSED 时可用。与 driver `~/home`（全关节零位）不同，走的是遥操慢速轨迹，用于把臂从任意位姿安全收回到零位

## REST API

所有响应信封：`{ok, message, data}`，冲突返回 HTTP 409 + `{"detail": "..."}`。

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/v1/health` | ROS 节点状态、launch 状态、PID、运行时长、WS 客户数、health 汇总 |
| GET | `/api/v1/presets` | 启动预设列表 |
| GET | `/api/v1/state` | 当前快照（同步，含 health + state_rates_hz） |
| GET | `/api/v1/logs` | 遥操、数采、推理三条 Launch 环形缓冲全量（默认各 8000 行；WS `log_tail` 只推尾 800） |
| POST | `/api/v1/start` | `{preset: "...", log?: bool}` 启动指定预设（只把遥操栈拉起来；使能交给 driver auto_ready/一键就绪——曾自动等 `~/enable` 导致启动即 503，已回退）。`log:true` 且预设属 astral_teleop/astral_arm_teleop 时注入 `log_dir=<inference_test_logs>/teleop`（launch 每次自动建 `{stamp}[_tag]/` 运行子目录、按侧拆 `_left/_right`；`ASTRAL_WEB_MONITOR_LOG_ROOT` 可改根目录） |
| POST | `/api/v1/stop` | SIGINT 停止 launch（30s 超时 SIGKILL） |
| POST | `/api/v1/pause` | 发 `/teleop/disarm`（软暂停，节点保持运行） |
| POST | `/api/v1/resume` | 发 `/teleop/armed`（恢复） |
| POST | `/api/v1/teleop/start` | 发 `/teleop/start`（一次性，记录 `vr_init` 并 arm；配合 `require_start_signal`；无条件发送，臂节点自行判断有效性） |
| POST | `/api/v1/teleop/home` | HOME 归位：先 `~/enable` 使能（真机预设），再 disarm + 发 `/teleop/home`（双臂沿 init_pose → init_waypoints → 零位 收回；仅 RUNNING/PAUSED 可用） |
| POST | `/api/v1/teleop/workpos` | 工作位：disarm + 发 `/teleop/init`（双臂沿 init_waypoints → init_pose 走到初始工作位，**途经点路径**；不调 `~/enable`；仅 RUNNING/PAUSED 可用） |
| POST | `/api/v1/teleop/workpos/direct` | 段间回位：disarm + 发 `/teleop/init_direct`（双臂**不经 init_waypoints**、直接回 init_pose，**直达路径**；与左 X 同功能；仅 RUNNING/PAUSED 可用） |
| POST | `/api/v1/restart` | 重启当前预设（停止后重新启动；同样只拉栈不自动使能；仅对经本监控启动的预设有效） |
| POST | `/api/v1/robot/ready` | 先自动 disarm 遥操，再调 driver `~/ready`（one_click_ready，上电+零位） |
| POST | `/api/v1/robot/home` | 先自动 disarm 遥操，再调 driver `~/home`（归零：阻尼中自动先切回 POSITION） |
| POST | `/api/v1/robot/position` | 先自动 disarm 遥操，再调 driver `~/position`（motion_mode=1，位置保持） |
| POST | `/api/v1/robot/damping` | 先自动 disarm 遥操，再调 driver `~/damping`（motion_mode=0，阻尼释放，可手动拖拽） |
| POST | `/api/v1/robot/estop` | 先自动 disarm 遥操，再调 driver `~/estop` → SDK `e_stop()`/`disable()` 断电，臂失去保持力 |
| GET | `/api/v1/video/status` | 视频回传状态：在线时 = streamer 自动扫描的相机列表（label/device/实时在线/sysfs 名）+ 门控状态；离线时 = monitor 自行扫描主机设备（供预选） |
| POST | `/api/v1/video/push` | `{enabled: bool}` 调 streamer `~/set_push_enabled` 总开关（关 = 全轨 2fps 黑帧静音） |
| POST | `/api/v1/video/cameras` | `{cameras: [...]}` 发 latched `~/active_cameras`（label 子集，空数组 = 全部配置相机） |
| GET | `/api/v1/video/feed/{label}` | 该路相机的 MJPEG 实时预览流（转发 streamer `~/preview/{label}`，浏览器 `<img>` 可直接用） |
| GET | `/api/v1/video/snapshot/{label}` | 该路最新一帧预览 JPEG（404 = 未知相机或暂无帧） |

## WebSocket

端点：`ws://<host>:8080/ws/telemetry`

- 推送频率：**30Hz**（`ASTRAL_WEB_MONITOR_PUSH_HZ`）
- 无客户端时跳过广播
- 连接后立即发一帧快照
- 支持 `ping` → 回 `{"type":"pong","ts":...}`

```json
{
  "type": "ui_state",
  "ts": 1692878400.123,
  "teleop": {"state": "running", "preset": "Full real (Quest)", "uptime_s": 120.5, "pid": 12345},
  "joints": {
    "left_arm":      {"values": [0.1, ...7], "ts": 1692878400.1, "stale": false},
    "right_arm":     {"values": [0.1, ...7], "ts": 1692878400.1, "stale": false},
    "left_gripper":  {"values": [0.35],     "ts": 1692878400.1, "stale": false},
    "right_hand":    {"values": [...20],    "ts": 1692878400.0, "stale": true},
    "head":          {"values": [0.0, 0.0], "ts": 1692878400.1, "stale": false}
  },
  "rates_hz": {
    "left_arm_cmd": 149.8, "right_arm_cmd": 149.9,
    "left_gripper_cmd": 50.0, "right_hand_cmd": 30.1, "head_cmd": 50.0
  },
  "state_rates_hz": {
    "left_arm_state": 200.0, "right_arm_state": 200.0,
    "left_gripper_state": 100.0, "right_hand_state": 100.0,
    "head_state": 100.0,
    "full_body_state": 100.0
  },
  "health": {
    "overall": "ok",
    "entities": {
      "left_arm": {"stale": false, "state_hz": 200.0, "cmd_hz": 149.8, "expected_hz": 30.0, "slow": false, "status": "ok"}
    }
  },
  "video_gate": {"push_enabled": true, "configured": ["wrist_left", "wrist_right"], "active": ["wrist_left"]},
  "log_tail": ["[INFO] mocap connected ...", "..."]
}
```

- `video_gate`：quest3_video_streamer 的实时门控镜像（`~/gate_state` latched JSON）；streamer 未运行时为 `null`。

## 状态机

```
stopped ──start──► starting ──2s暖机──► running
   ▲                                       │
   │                                  pause │ resume
   │                                       ▼
   │                                    paused
   │                                       │
   └────────── stop(SIGINT) ◄──────────────┘
```

## Web UI 功能

顶栏：标题 + WS/ROS/数据 三色状态徽标 + ControlBar（急停[真断电] / 阻尼释放 / 开始遥操 / 暂停 / 恢复）。
下方三 Tab：

| Tab | 内容 |
|-----|------|
| 监控 | **数据采集卡片**（录制控制 + 任务文本 + 实时状态，见下）+ 4 个关节面板（左/右臂、左夹爪、右灵巧手）+ 指令频率 chips + 实时折线图（指令 Hz、臂关节0 角度） |
| 数据处理 | 选择 session / 段号 / 相机映射，完整流程或单步，任务进度、日志、质量报告、图像和状态曲线预览 |
| 健康 | 总体徽标 + 每实体卡片（状态流 Hz / 指令流 Hz / 期望 / 数据龄期 / ok·slow·stale） |
| 系统 | 预设管理（启动/停止/重启/**HOME** + **「记录遥操日志」勾选框**→ 启动遥操预设时落 `/tmp/teleop_teleop_{left,right}.jsonl` 诊断 JSONL）+ **机器人模式**（一键就绪/归零/位置保持/阻尼释放/急停断电）+ **管线延迟**（mocap stamp / IK 求解 / VR→指令端到端，左右臂）+ **视频回传**（总开关/路数下拉/逐路勾选/设备在线点/**实时画面预览**）+ Launch 日志控制台（70vh，复制/下载全量缓冲） |

- **视频回传卡片**：调 `quest3_video_streamer` 的运行时门控（`~/set_push_enabled` + latched `~/active_cameras`）。总开关关掉后所有轨发 2fps 黑帧（几乎不占带宽，Quest 面板变黑）；逐路勾选决定哪些相机推流；「路数」下拉是快捷选择（选 n = 勾前 n 路，逐路勾选后显示"自定义"）。**相机列表不写死**：streamer 默认 `auto_scan` 自动扫描主机采集设备（label = `videoN`），卡片在线时显示其扫描结果（含 `/dev/videoN` 与 sysfs 设备名，未接置灰）；**离线时卡片自行扫描主机设备，可预选**——latched 话题会在 streamer 启动后生效（总开关服务需在线）。在线判定看 `~/gate_state` 是否还有活发布者，streamer 死掉会正确显示"离线"。启动后才插入的相机需重启栈进入 track 集合。**实时画面**：勾选「实时画面」后，在线且勾选的每路相机显示 MJPEG 实时预览（streamer 抽帧 10fps/640宽/q65 JPEG，独立线程编码经 `~/preview/{label}` 转发，延迟约 0.2~0.4s，仅供监控；取消勾选的路预览同步停止，不勾不耗资源）

- **急停（真断电）**：红色常驻按钮，确认后**先自动 disarm 遥操**再调 driver `~/estop` → SDK `e_stop()`/`disable()`，臂失去保持力；恢复需重新「一键就绪」。区别于「暂停」（软 disarm，臂仍上电保持位姿）
- **启动/重启只拉栈**：`/start`、`/restart` 恢复改前语义——只把遥操栈拉起来，**不**自动调 `~/enable`（曾导致启动即 503 + 臂不动，已回退）。电机使能由 driver `auto_ready`/「一键就绪」负责；**急停 → 停止 → 启动**后的恢复收敛到 HOME 按钮
- **HOME（归位到零）**：预设管理区紫色按钮，确认后调 `POST /api/v1/teleop/home`——先 `~/enable` 使能（仅此端点使用，上电**不回零**），再 disarm + 发 `/teleop/home`，双臂沿 init_pose → init_waypoints → 零位 慢速收回并停在零位；之后需「一键就绪/开始遥操」才能继续。仅遥操 RUNNING/PAUSED 时可用。`~/enable` **幂等**：已上电（启动 auto_ready 已使能）直接成功跳过，不会重复使能；板端在线但电源位未确认也算下发成功——不会再把"其实已使能"误报成"电机未使能"（见 robot_control README）
- **五个硬件模式按钮先自动 disarm**：急停/阻尼释放/一键就绪/归零/位置保持 每次点击都**先发布 `/teleop/disarm`** 再调 driver 服务（与 HOME 端点同款）——把臂节点带出 armed/启动归位(homing) 发流。否则 driver 显式下发的一次性目标（归零/位置保持）会被仍在运行的陈旧命令流下一帧覆盖（症状：阻尼后手动拖臂再点一键就绪，臂回到阻尼前的位姿；阻尼中归零无效）。之后要用遥操重新点「开始遥操」
- **阻尼释放**：调 driver `~/damping` → `motion_mode=0`，电机仍上电、关节可手动拖拽。典型流程：遥操中 → **阻尼释放（自动 disarm）**（手动拖回 home）→ 位置保持/归零

> **机器人模式按钮的可用时机（重要）**：急停/阻尼释放/位置保持/一键就绪/归零 这五个按钮调的都是 **driver 节点（astral_robot_driver）的 ROS 服务**，driver 随遥操栈启停：
>
> | 遥操栈状态 | 按钮表现 |
> |-----------|---------|
> | 运行中（启动后） | ✅ 可用 |
> | 已暂停（软 disarm） | ✅ 可用（driver 还活着，只是不收遥操指令） |
> | 已停止 | ❌ 503「driver service 未就绪」——driver 进程已随栈退出，属**预期行为**，先点「启动」再操作 |
>
> 即：**先启动栈，再点机器人模式按钮**；停止栈之后任何机器人按钮都不会生效。若运行中仍 503，说明 driver 没起来（`with_arm_driver:=false` 的预设/sim）或 Jetson 端 driver 是旧代码——在 Jetson 上 `ros2 service list \| grep astral_robot_driver` 应列出 6 个 Trigger 服务（ready/enable/home/estop/damping/position）。

- **数据采集卡片**（监控 tab 顶部）：`astral_data_collect` 的完整控制面，分两层：
  - **节点进程（独立泳道）**：卡片右上角「启动/重启/停止节点」，走专属 `LaunchManager` 泳道（`POST /api/v1/collect/launch/start|stop|restart`），与遥操预设**生命周期完全解耦、可并存**——数采泳道是纯订阅者，启动跳过孤儿检测；遥操侧孤儿检测对 `astral_data_collect` 命令行有对称豁免。泳道状态（运行中/启动中/已停止 + pid）随 ui_state 的 `collect_launch` 字段推送。启动命令来源仍是 `presets.yaml` 中 `package: astral_data_collect` 的条目（该条目不再出现在「系统」tab 遥操预设下拉中，避免占用主泳道）。schema 硬件配置改 `astral_data_collect/config/data_collect.yaml` 后点「重启节点」生效。
  - **录制控制（纯话题桥接）**：按钮组（开始录制/停止保存/下一段/暂停继续/丢弃[确认后删文件]）+ **段间回位**（与 VR 左手柄 X 同功能：发 `/teleop/disarm`+`/teleop/init_direct` 停止跟随 VR 并**直接**回到工作位——**不经 init_waypoints**，供段与段之间摆放物品；仅遥操 RUNNING/PAUSED 且**非录制中**可用——录制保护与 X 键闸门一致，确认后下发）+ 下一段任务文本输入 + 实时状态徽标（IDLE/录制中/已暂停/保存中，录制中红色）+ 状态行（session、段号、时长、state 维度、各流实测频率、掉帧红 chip）。接口为 `POST /api/v1/collect/control {cmd}`（白名单 start/stop/discard/next/pause/resume）与 `POST /api/v1/collect/task {text}`（latched，应用于下一段）、`POST /api/v1/collect/session {text}`（latched，仅 IDLE 生效的录制目录切换）；状态来自 `/data_collect/state` latched JSON 镜像（带 stale 龄期标记，采集节点退出后可识别）。CLI 启动的采集节点同样可控（此时泳道显示"已停止"但录制按钮照常可用）。
  - **「本次采集数据」实时下拉窗口**：设定任务/目录输入下方可折叠面板（点标题展开/收起），实时显示——**文件夹**（完整路径）、本次 session 累计磁盘占用、当前段（时长 / 精确样本数 / 每相机帧数 / 磁盘占用）。数据由采集节点 1Hz 发布（`/data_collect/state` 新增 `folder`/`episode_counts`/`camera_counts`/`episode_bytes`/`session_bytes`）；**样本/帧数为精确数据量，磁盘占用含 HDF5 chunk 预分配**（"占了多少磁盘"口径）。
  - **推理模块（`astral_policy_inference`）**：数采卡片下方独立卡片，两层——
    **配置化泳道**：GPU 主机 IP / 端口 / 图像尺寸（默认 **224**）/ 引擎模式（queue_async|queue_sync|rtc）/ 模型族（默认 **pi0.5**）/ **记录日志开关**（开启 → `pi_metrics.jsonl` + `pi_cmds.jsonl` + `pi_control.jsonl` + `camera_diagnostics.jsonl`；最后一份自动合并 streamer capture/tap 与 policy 图像 rx/decode，无需另开测速终端），「启动/重启/停止节点」走专属 `LaunchManager`（`POST /api/v1/infer/launch/*`，启动跳过孤儿检测——推理节点与遥操共存）；卡片内的**推理 Launch 日志**实时显示 stdout/stderr 尾部，可复制，下载会获取完整环形缓冲；**控制面（纯话题桥）**：命令按钮（开始策略/回放[可选源路径]/暂停/恢复/接管 HUMAN/释放/停止，对应 `policy_keyboard` 全部按键）+ 任务输入，发 `/policy_inference/cmd`+`/task`（`POST /api/v1/infer/cmd|task`，白名单校验；`playback:<path>[:<ep>]` 带源），**CLI 启动的推理节点同样可控**；推理状态离线时按钮禁用，后端发送 VOLATILE 命令前最多等待 2 秒发现订阅者，超时明确返回 503，避免节点刚启动时 DDS 尚未完成发现却假报“已发送”（命令保持 VOLATILE，防止节点重启后重放旧 `policy` 自动运动）；状态显示 `/policy_inference/state` 镜像（模式徽标/engine 指标/延迟/exec_events/error，带 stale 标记）。**启动参数经 shell 元字符消毒**（对抗性审查：host 等进 `bash -c` 拼 launch 命令，白名单清洗防注入）。
- **Toast 通知**：操作成功/失败以右上角浮窗提示（替代 alert），自动消失
- **实时图表**：手写 SVG 折线（无第三方图表库），环形缓冲 200 样本（≈6.7s @ 30Hz）
- **头部通道**：监视 tab 显示「头部 (yaw/pitch)」面板 + yaw/pitch 实时折线；健康巡检含头部（仅新鲜度判断——头部指令在启动前/手柄掉线时合法为 0Hz，故不设 cmd 频率下限）
- **健康巡检**：`expected_hz` 阈值经 `ASTRAL_WEB_MONITOR_EXPECTED_HZ` 配置（默认 30Hz），只读估算

## 启动预设

见 `config/presets.yaml`，可直接编辑无需改代码：

| 预设 | 包 | launch | 说明 |
|------|----|--------|------|
| Arms only (sim, no driver) | astral_arm_teleop | astral_dual_arm_teleop | 双臂+Quest，无真机 |
| Dual arms + left gripper (no right hand) | astral_teleop | full_teleop | 双臂+左夹爪，无右手 |
| Left arm + left gripper (no right arm) | astral_teleop | full_teleop | **单左臂+左夹爪**，不启右臂遥操 |
| Dual arms + dual grippers (no dexterous hand) | astral_teleop | full_teleop | 双臂+双夹爪 |
| Full real (Quest right hand) | astral_teleop | full_teleop | 双臂+左夹爪+右手灵巧手，Quest |
| Full real (Glove right hand) | astral_teleop | full_teleop | 同上，右手来自 Wuji Glove |
| MuJoCo sim pipeline | astral_mujoco_sim | astral_sim_pipeline | MuJoCo 仿真全链路 |
| Data collect (VLA recording) | astral_data_collect | data_collect | VLA 数据采集节点（独立泳道：由监控页数采卡片启停，不在遥操预设下拉中） |

## 依赖

```bash
# Python 依赖（用系统 python3，不是 conda）
python3 -m pip install fastapi uvicorn

# 构建
cd /path/to/astral_ws
export PATH=/usr/bin:$PATH          # 避免 conda python 抢占
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_web_monitor --symlink-install
source install/setup.bash
```

## 启动

```bash
# 生产模式：后端托管已构建前端
# 注意：ros2 pkg prefix 返回 install/<pkg>，需 ../../ 回到工作区根再进 src
ASTRAL_WEB_MONITOR_DIST=$(ros2 pkg prefix astral_web_monitor)/../../src/astral_web_monitor/web/dist \
  ros2 launch astral_web_monitor web_monitor.launch.py

# 浏览器访问 http://<host>:8080
# 若 8080 被占（如 Cursor IDE），换端口：
# ASTRAL_WEB_MONITOR_PORT=8090 ros2 launch astral_web_monitor web_monitor.launch.py
```

开发模式（前端热更新）：

```bash
# 终端1：后端
ros2 launch astral_web_monitor web_monitor.launch.py

# 终端2：前端 dev server（:5173，代理 /api /ws → :8080）
cd src/astral_web_monitor/web && npm install && npm run dev

# 浏览器访问 http://localhost:5173
```

## 完整数采流程（web + VR 键位，权威版见 `astral_teleop/README.md`）

1. **启动 web 监控**（上）→ **系统 tab 选预设启动遥操**（当前采集配置用
   「Left arm + left gripper (no right arm)」）→ **数采卡片「启动节点」**拉数采泳道。
2. 数采卡片设**「录到目录」**（session）与**「下一段任务文本」**；点**「工作位」**（系统 tab）让双臂就位。
3. 戴 Quest3 → 左手 **grip** 开始遥操（重标定）→ 右手 **A** 开始录制 → 执行任务 →
   右手 **B** 停止保存 → 左手 **X**（或卡片**「段间回位」**）停止跟随+**直接**回工作位（不经途径点）→ 摆物品 → 循环。

- **顺序要求**：先 grip 再 A（否则新段 armed 覆盖率 <50% 触发 validate W5）；B 后等状态回 IDLE
  再按 X（SAVING 期 X/「段间回位」被拦截）；回位走完再 grip（homing 中 `/teleop/start` 被拒）。
- **「段间回位」按钮**（监控 tab 数采卡片，青色）：与左手 X 同功能（发 `/teleop/disarm`+`/teleop/init_direct`，
  **直接**回 init_pose、不经途径点），仅遥操 RUNNING/PAUSED 且非录制中可用（录制保护与 X 闸门一致），
  确认后下发。区别于系统 tab「工作位」（`/teleop/init`，途经点路径）。
- 键位/按钮等价表见 `astral_teleop/README.md`「键位速查」。

## 参数 / 环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `ASTRAL_WEB_MONITOR_HOST` | `127.0.0.1` | 绑定地址；仅本机可访问。需要局域网访问时显式设置 `0.0.0.0` |
| `ASTRAL_WEB_MONITOR_PORT` | `8080` | 端口 |
| `ASTRAL_WEB_MONITOR_DIST` | `""` | 前端构建产物路径（空时自动定位源码或安装包中的 web/dist） |
| `ASTRAL_WEB_MONITOR_PUSH_HZ` | `30` | WebSocket 推送频率 |
| `ASTRAL_WEB_MONITOR_STALE_S` | `2.0` | 关节陈旧阈值（秒） |
| `ASTRAL_WEB_MONITOR_WARMUP_S` | `2.0` | 启动暖机时间 |
| `ASTRAL_WEB_MONITOR_STOP_TIMEOUT_S` | `30` | SIGINT 超时 |
| `ASTRAL_WEB_MONITOR_HAND_NAME` | `right_hand` | 灵巧手话题命名空间 |

## 前端构建

```bash
cd src/astral_web_monitor/web
npm install
npm run build    # → web/dist/
```

## 非侵入性保证

| 交互 | 机制 | 修改现有包？ |
|------|------|-------------|
| 监控关节 | 订阅现有话题 | 否 |
| 指令 Hz | 订阅现有指令话题 | 否 |
| 启动 | subprocess `ros2 launch` | 否 |
| 停止 | SIGINT subprocess | 否 |
| 暂停 | 发布 `/teleop/disarm`（已有话题） | 否 |
| 恢复 | 发布 `/teleop/armed`（已有话题） | 否 |
| 开始遥操 | 发布 `/teleop/start`（已有话题） | 否 |
| 重启 | stop + start 同一预设（subprocess） | 否 |
| 急停（真断电） | 先发 `/teleop/disarm`，再调 driver 已有服务 `~/estop`（SDK `e_stop`） | 否（仅调用 driver 已暴露的服务 + 发布已有 disarm 话题） |
| 阻尼释放 | 先发 `/teleop/disarm`，再调 driver 已有服务 `~/damping`（`motion_mode=0`） | 否 |
| 位置保持 | 先发 `/teleop/disarm`，再调 driver 已有服务 `~/position`（`motion_mode=1`） | 否 |
| 一键就绪 / 归零 | 先发 `/teleop/disarm`，再调 driver 已有服务 `~/ready` / `~/home` | 否 |
| 健康巡检 | 订阅现有 state/command 话题估算频率 | 否 |
| 视频门控 | 调/发 streamer 已暴露的 `~/set_push_enabled` / `~/active_cameras` / `~/gate_state` | 否 |
| 日志 | 采集 subprocess stdout | 否 |

> 注：机器人模式按钮调用 `astral_robot_control` driver **已暴露**的 Trigger 服务（`~/ready`/`~/home`/`~/position`/`~/damping`/`~/estop`），调用前先发布已有的 `/teleop/disarm`（与 HOME 端点同款，防止仍在运行的遥操/归位命令流覆盖一次性目标）。driver 是硬件权威，monitor 仅作为服务客户端调用——不新增硬件写入路径、不修改 driver 逻辑。若 driver 未启动，端点返回 503。

## 包结构

```
astral_web_monitor/
├── astral_web_monitor/
│   ├── monitor_node.py      # rclpy 节点：只读订阅 + 快照 + pause/resume
│   ├── launch_manager.py    # subprocess 启停 + 孤儿检测
│   ├── rate_counter.py      # EWMA Hz 估计
│   ├── web_server.py        # FastAPI + WebSocket + 静态托管
│   ├── schemas.py           # Pydantic 模型
│   └── config.py            # 话题名/阈值/端口
├── config/presets.yaml      # 启动预设
├── launch/web_monitor.launch.py
└── web/                     # React + Vite 前端
    ├── src/
    │   ├── hooks/useRealtime.ts   # 单例 WebSocket + 连接状态
    │   ├── hooks/useToast.ts      # Toast 通知 store
    │   ├── hooks/historyStore.ts  # 图表环形缓冲
    │   ├── components/            # ControlBar, MonitorTab, HealthPanel,
    │   │                          # SystemTab, VideoCard, ChartPanel, ToastHost,
    │   │                          # Tabs, JointPanel, LogConsole, StatusBadge
    │   └── lib/mapUiState.ts      # 数据归一化
    └── dist/                # 构建产物
```

## Web 数据处理

新增「数据处理」Tab。停止当前 session 录制后，选择已有 raw session，选择完整流程与导出目标（OpenPI v2.1 / ACT v3 / 两者），点击「开始处理」。任务在后台执行，页面关闭不终止任务；同一服务一次只执行一个数据任务。

完整流程：建立工作副本 → raw 校验 → 可选 cmd/state 诊断 → 对齐 → 对齐校验 → 清洗前预览 → 可选保守清洗、校验和来源验证 → 导出。保守清洗默认关闭，需要勾选；若要先评审清洗方案，选择单步「清洗预览」。清洗前后生成的 HTML 显示每段首/中/末图像及状态曲线，属于抽查，不能代替完整任务评审。单步还提供 raw/ aligned 校验、诊断、对齐、来源验证、导出与 v2.1 升 v3。

每个任务目录是 `${ASTRAL_DATA_ROOT}/web_jobs/<时间-任务ID>/`。working 保存真实副本（不是硬链接），cleaned 保存清洗结果，openpi / act 保存训练输入；preview_before/preview_after 保存浏览器预览。不会修改原始 raw，也不会自动隔离或删除失败段；遇到失败立即中止后续步骤。默认不可覆盖已有任务输出。点击「用工作结果继续处理」可将结果作为下一任务输入。

高级选项：段号 `0,24,49,74,99`（空=全部），历史相机映射 `video8=base,video0=left_wrist`（只处理工作副本），图像边长 224（0=原生）。正在录制/暂停/保存的同一 session 拒绝提交；复制过程中检测原始文件变化也会失败。当前 `pick_place_merged` 历史数据需要上述映射。

结果区显示逐步状态、通过/告警/失败段数、报告、完整日志、预览和服务器输出路径。停止任务使用独立进程组 SIGINT，8 秒未退出则 SIGKILL，已完成产物保留；服务退出停止活动任务，异常重启后的历史活动任务标为中断，不自动续跑。任务历史和 job.json 持久保存。训练及归一化统计仍使用对应训练工具。

| 接口 | 行为 |
| --- | --- |
| GET /api/v1/data/sessions | 数据目录列表、处理环境与 ACT 深度检查环境状态 |
| GET /api/v1/data/jobs | 当前任务及历史摘要（与 ui_state.data_process 相同） |
| POST /api/v1/data/jobs | 提交 action/source/target/image_size/camera_map/episodes/quantify/repair/reference |
| POST /api/v1/data/jobs/cancel | 停止当前任务 |
| GET /api/v1/data/jobs/{id} | 任务详情、步骤、结果与日志尾部 |
| GET /api/v1/data/jobs/{id}/files/{relative} | 读取该任务内的报告/预览/图像/日志；拒绝越界和 HDF5 下载 |

实时状态随现有 `ui_state` 的 `data_process` 字段推送；完整日志与报告按需 REST 读取。后端不导入转换依赖，使用独立 OpenPI Python 调用统一 CLI。目录输入限定在数据根目录内；使用参数数组启动子进程，不拼接用户 shell 命令。

| 环境变量 | 默认值／用途 |
| --- | --- |
| ASTRAL_SDK_ROOT | 从源码/安装目录向上定位 sdk，可显式覆盖 |
| ASTRAL_DATA_ROOT | sdk/astral_data |
| ASTRAL_DATA_PYTHON | sdk/VLA/openpi/.venv/bin/python |
| ASTRAL_DATA_CHECK_PYTHON | ~/miniconda3/envs/lerobot/bin/python；存在时 ACT 真加载与预处理检查 |
| ASTRAL_WEB_MONITOR_DIST | 空时优先源码 web/dist，否则使用安装包 share/astral_web_monitor/web/dist |

前端变更必须 npm run build；dist 会随 colcon 安装。已有 Web 进程需要重启以加载新后端，浏览器刷新加载新页面。

2026-10-10 冒烟：独立本地测试服务（未初始化 ROS，不连接硬件）通过 REST 提交 pick_place_merged 的 0/24/49/74/99 段，显式映射相机，完整流程开启清洗并导出两种格式。13 步通过；5 段清洗后 1066 帧；现代 LeRobot/ACT 预处理通过；报告、HTML 和 JPEG 读取通过；原始 HDF5/meta SHA256 前后不变。结果：`astral_data/web_jobs/20261010-160354-20b6d236/`。清洗结果作为单步校验输入通过，WebSocket 推送状态通过。Web 回归 23 项通过，数据离线回归 84 项通过，前端构建及 colcon 安装构建通过。当前工具环境没有可用浏览器，未进行交互式视觉验收。
