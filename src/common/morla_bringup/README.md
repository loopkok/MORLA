# morla_bringup

`morla_bringup` also owns MORLA's shared asynchronous control coordinator. Every profile-driven launch starts one `control_coordinator_node`; Web buttons and controller gates call the same `/morla/control/*` services. An accepted `Trigger` request is not a completed movement: follow `/morla/control/state` by `operation_id` until `completed` or `failed`. The coordinator freezes policy and configured manual writers and waits for fresh ownership/prepare acknowledgements before forwarding mode changes.

```bash
# Example software-only Nero + dual XHand profile. Real mode must explicitly
# select hardware backends and configured CAN/serial devices.
ros2 launch morla_bringup nero_xhand_stack.launch.py \
  profile_id:=nero_dual_xhand mode:=sim sdk_backend:=fake enable_viewer:=false

# Astral dual arm + left gripper + right Wuji simulation (from Web, select the
# same profile and Sim mode, or use the profile-driven Astral launch arguments).
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py \
  control_profile_id:=astral_dual_gripper_wuji arm_side:=both gripper_sides:=left \
  enable_viewer:=false
```

The Web manager's “pause/resume/start/release/ready/enable/position/damping/home/workpos/workpos_direct/estop” operations route through this coordinator when a live profile coordinator is present. Direct legacy teleop topic controls remain for standalone compatibility; profile-driven UI and gates use the shared service so writer freeze and completion status stay consistent.

## Legacy Astral assembly notes

整机遥操 **只拼现有包**，这里没有 IK / 夹爪 / retarget 源码。

```text
Quest3 mocap（仅一份）
  wrist ×2     → astral_arm_teleop → 双臂
  landmarks/L  → astral_gripper_teleop → 左夹爪
  landmarks/R  → wujihand_retargeting → 右 Wuji   （right_hand_source:=quest3）
手套            → 同一 hand_landmarks/right         （right_hand_source:=glove）
  landmarks/R  → astral_gripper_teleop → 右夹爪     （right_hand_source:=gripper）
右手柄摇杆      → head_teleop_node → /head/joint_commands → 头(yaw/pitch)
机器人相机      → quest3_video_streamer → WebRTC → Quest 面板  （with_video:=true 默认）
```

`right_hand_source` 四选一：
- `quest3` — 右灵巧手，来自 Quest landmarks（默认）
- `glove` — 右灵巧手，来自 Wuji Glove（关掉 Quest 的 `hand_landmarks/right`，右腕 `wrist_pose` 仍给右臂 IK）
- `gripper` — 右捏合 → 右夹爪（不起 Wuji，右手用裸手做 wrist+pinch）
- `none` — 无右手设备

```bash
# 真机：Quest 双臂 + 左捏合夹爪 + Quest 右手 Wuji
ros2 launch morla_bringup full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true \
  right_hand_source:=quest3

# 右手改手套（关 Studio，勿与 Quest 右手 landmark 同时发）
ros2 launch morla_bringup full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true \
  right_hand_source:=glove

# 双夹爪：左/右捏合 → 左/右夹爪（无灵巧手）
ros2 launch morla_bringup full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=false \
  right_hand_source:=gripper with_gripper:=true

# 单左臂 + 左夹爪（不启动右臂遥操）
ros2 launch morla_bringup full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=false \
  right_hand_source:=none with_gripper:=true arm_side:=left
```

不要再单独起 `astral_dual_arm_teleop` / `wujihand_real_pipeline`（会抢 mocap 或 `joint_commands`）。

## 完整遥操启动（web / CLI 两条路径）

### 路径 A：Web 监控（推荐，真机 / 数采）

1. **启动 web 监控**（生产模式，托管已构建前端）：
   ```bash
   MORLA_WEB_MANAGER_DIST=$(ros2 pkg prefix morla_web_manager)/../../src/common/morla_web_manager/web/dist \
     ros2 launch morla_web_manager web_monitor.launch.py
   ```
   浏览器访问 `http://<host>:8080`（8080 被占换 `MORLA_WEB_MANAGER_PORT=8090`）。
2. **系统 tab → 预设列表 → 选预设 → 启动**：启动只把遥操栈拉起来，**不自动使能电机**——
   使能由 driver `auto_ready` 或「一键就绪」按钮负责。当前采集配置（单左臂+左夹爪）对应
   预设 **「Left arm + left gripper (no right arm)」**。
3. （数采工作流）**监控 tab 数据采集卡片**：右上角「启动节点」拉数采独立泳道（与遥操预设
   生命周期解耦），再设「录到目录」（session）与「下一段任务文本」。
4. 点**「工作位」**（系统 tab）让双臂沿 init_waypoints → init_pose 走到初始工作位——启动不再
   自动归位（`move_to_init_pose=false`），回工作位靠此按钮（途经点路径）。**段间回位（左 X）不走
   途经点、直接回工作位**（见下）。

### 路径 B：CLI 直接启动

```bash
# 单左臂 + 左夹爪（当前数采配置；对应 web 预设 "Left arm + left gripper"）
ros2 launch morla_bringup full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=false \
  right_hand_source:=none with_gripper:=true arm_side:=left

# 完整真机：双臂 + 左夹爪 + 右灵巧手（Quest 右手，默认）
ros2 launch morla_bringup full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true right_hand_source:=quest3

# 双夹爪：左/右捏合 → 左/右夹爪（无灵巧手）
ros2 launch morla_bringup full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=false right_hand_source:=gripper with_gripper:=true

# 仿真：MuJoCo 全链路
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py

# 数采节点（独立泳道，可随后台/命令行启动）
ros2 launch morla_data_collect data_collect.launch.py session:=pick_place
```

> CLI 启动的遥操栈，web 监控同样可监控/暂停/机器人模式（只读 + 发布已有话题）；
> CLI 启动的数采节点，web 数采卡片同样可控（纯话题桥接）。

## 遥操操作步骤（数采 episode 循环）

**准备阶段**（每次新 session）：
1. 启动 web 监控 + 遥操栈（路径 A 或 B）→ 启动数采节点 → 设 session 目录 + 任务文本。
2. 点**「工作位」**让双臂就位（或确认已在 init_pose）。
3. 戴 Quest3 进入软件（要视频回传则开 video feed）。

**每段 episode 循环**（VR 键位与 web 按钮等价，可混用）：

| 步骤 | 操作 | 说明 |
|---|---|---|
| ① 开始遥操 | 左手 **grip**（web「开始遥操」） | 记 `vr_init`（重标定）+ armed；再按一次 = 重新记零点 |
| ② 开始录制 | 右手 **A**（web「开始录制」） | 仅 IDLE 有效 |
| ③ 执行任务 | — | VR 跟随，臂 + 夹爪 |
| ④ 停止保存 | 右手 **B**（web「停止保存」） | 仅录制中有效，保存后段号自动接续 |
| ⑤ 段间回位 | 左手 **X**（web 数采卡片「段间回位」） | **停止跟随 VR + 直接回到工作位**（不经途径点）；等状态回 IDLE 再按 |
| ⑥ 摆放物品 | — | 手臂已回工作位、不跟随，人离开手柄布置桌面 |
| ⑦ 回到 ① | 左手 **grip** | 重标定 + armed，开始下一段 |

**推理活跃时 grip 语义改变（2026-09-17 起）**：`controller_start_gate` 会订
`/policy_inference/state`——当 activity∈{policy,playback}（策略/回放运行中），左 **grip**
改发 `/policy_inference/cmd`="takeover"（HITL 接管），**不再**发 `/teleop/start`（否则会
重新武装遥操、与策略双写 `joint_commands`）。**HUMAN 时右手 A 上升沿** → `/policy_inference/cmd`
="release"（交还控制权；本门随遥操栈常驻，HITL 释放不依赖数采栈）。HUMAN/IDLE/无数采时
grip 仍为原「开始遥操」。

**关键顺序要求**（影响数据质量，不损坏数据）：
- **先 grip 再按 A**：段间回位后臂是 disarmed，若先 A 后 grip，新段 armed 覆盖率统计不足
  → validate **W5**（armed <50%）警告。
- **B 保存后等状态回 IDLE 再按 X**：SAVING 期按 X 会被闸门拦截（安全但无反应），需再按一次。
- **回位走完再 grip**：homing 中 `/teleop/start` 被臂节点拒绝（须等臂停在工作位）。
- **先启动栈再点机器人模式按钮**（急停/阻尼/就绪/归零/位置保持）：driver 随栈退出，栈停止后
  按钮 503 属预期。

**安全/收尾**：
- **急停（真断电）**：web 顶栏红色急停（确认后先 disarm 再 driver `~/estop`，臂失去保持力；
  恢复需重新「一键就绪」）。
- **暂停/恢复**：web「暂停」= 软 disarm（节点保持运行，夹爪/手/头继续）；VR 看门狗
  （1.5s 无腕姿）disarm 后须**重新 grip 重标定**才能恢复。
- **HOME（归零）**：web 系统 tab「HOME」（先使能电机 → disarm → 沿 init_pose → init_waypoints →
  零位慢速收回）。

### 键位速查（VR / web / 键盘 / CLI 四端等价）

| 动作 | VR 手柄 | Web 按钮 | 数采键盘 | CLI 等价 |
|---|---|---|---|---|
| 开始遥操（重标定） | 左 **grip** | 系统tab「开始遥操」 | — | `ros2 topic pub --once /teleop/armed std_msgs/Bool "{data: true}"` + 同法 `/teleop/start` |
| 回到工作位·**直达**（段间回位） | 左 **X** | 数采卡片「段间回位」 | — | `/teleop/disarm`(True) + `/teleop/init_direct`(True) |
| 回到工作位·途经点 | — | 系统tab「工作位」 | — | `/teleop/disarm`(True) + `/teleop/init`(True) |
| 开始录制 | 右 **A** | 数采卡片「开始录制」 | `s` | `ros2 topic pub --once /data_collect/control std_msgs/String "{data: 'start'}"` |
| 停止保存 | 右 **B** | 数采卡片「停止保存」 | `q` | `/data_collect/control` `"stop"` |
| 丢弃 | 摇杆按下 | 数采卡片「丢弃」（确认删文件） | `d` | `/data_collect/control` `"discard"` |
| 保存并开新段 | — | 数采卡片「下一段」 | `n` | `/data_collect/control` `"next"` |
| 暂停/继续录制 | — | 数采卡片「暂停/继续」 | `p` | `/data_collect/control` `"pause"`/`"resume"` |
| 任务文本 | — | 数采卡片「设定任务」 | `t` | `ros2 topic pub --once /data_collect/task std_msgs/String "{data: '...'}"` |

## 手柄集成

`full_teleop.launch.py` 默认起 `controller_start_gate` 节点：

- **左手柄 grip 键（中指，mask bit 5）→ `/teleop/start`**：按下沿（rising edge）发一次性启动信号，等价于 web「开始遥操」或 `ros2 topic pub --once /teleop/start`。配合 `require_start_signal:=true`：手摆好初始位姿后按左 grip 即开始遥操；再按一次 = 重新记零点（re-center）。
- **左手柄 X 键（primary，mask bit 0）→ 段间回位（直达）**：`controller_workpos_gate` 按下沿发
  `/teleop/disarm` + `/teleop/init_direct`——停止跟随 VR 并**不经 init_waypoints**、直接关节空间
  直线插补回到 init_pose 工作位（数采段与段之间快速回位；区别于 web「工作位」按钮的途经点路径
  `/teleop/init`）。订阅 `/data_collect/state` 做录制保护：**录制中（RECORDING/PAUSED/SAVING）按 X
  忽略**（防手臂回位毁段），无数采节点运行（纯遥操）时照常生效。sim 预设同启（`with_start_gate:=true`）。

`pinch_gripper_node` 同时订阅 `quest3/{side}_controller_joy`：

- **左右手柄 trigger 模拟量 → 左右夹爪开合**：`axes[0]`（0=开 … 1=合）驱动夹爪，与手部捏合并存（手柄 Joy 新鲜时优先 trigger，否则回退 pinch）。握手柄控臂时用 trigger 控夹爪，裸手时用捏合控夹爪。

手柄位姿本身经 `controller_as_wrist` 镜像到 `wrist_pose` 驱动臂 IK（既有）。

### 右手柄摇杆 → 头部（`head_teleop_node`，默认开启）

`full_teleop.launch.py` 还默认起 `head_teleop_node`（`with_head_teleop:=true`，可关）：

- **右手柄摇杆 → 头 yaw/pitch（绝对位置，非增量）**：`axes[2]`=stickX（左=-1 右=+1）→ yaw，`axes[3]`=stickY（前=+1 后=-1）→ pitch。摇杆弹簧回中 → 头回初始位。
- **启动时记 `head_init`**：与臂遥操同一闸门 `/teleop/start`（`require_start_signal:=true`）。按下 start 时读 `/head/joint_states` 记当前头角为基准（无该话题时回退 `init_head_*`，默认 0,0），避免上电跳变；之后 `head_target = head_init + stick*scale`。
- **非侵入**：只订阅已有 `quest3/right_controller_joy`、`/head/joint_states`、`/teleop/start|disarm`，只发到 `astral_robot_control` 已在订的 `/head/joint_commands`——driver 是唯一硬件权威。全部 BEST_EFFORT（与 mocap/driver 传感流一致；CLI `ros2 topic pub` 默认 RELIABLE 发布与之兼容，可直接注入；但 `ros2 topic echo` 看 BEST_EFFORT 话题需加 `--qos-reliability best_effort`）。
- 参数在 `config/head_teleop.yaml`：`yaw_scale`/`pitch_scale`（带符号，方向反了改符号）、`yaw/pitch_min/max`（限幅）、`stick_deadzone`、`publish_rate`、`input_timeout_s`。默认量程保守（yaw ±0.8rad、pitch ±0.4rad），**真机首测请确认"摇杆右推头右转"，反了把对应 scale 加负号**。

关掉头部遥操：`with_head_teleop:=false`。

策略切到 HUMAN 后，头部写入端会先等待策略 disarm，再等待一条新鲜右手柄 Joy；头部关节状态也必须
新鲜才会恢复输出。策略或输入租约过期时头部保持 disarm，避免旧摇杆值在交接后继续生效。

## 视频回传（`with_video:=true`，默认开启）

`full_teleop.launch.py` 默认拉起 `quest3_video_streamer`（机器人相机 → Quest 3D 面板）：

- **链路前提**：USB 线连 Quest + `adb reverse tcp:8000 tcp:8000`（mocap）+ `adb reverse tcp:8765 tcp:8765`（视频信令）——launch 启动时会**自动执行**这两条 reverse（adb 不存在/无设备只打 WARN，不阻塞启动）；然后**在 Quest 端 app 开启 video feed**，链路才通。
- **推哪些相机**：`quest3_video_streamer/config/params.yaml` 的 `cameras` 列表（默认 `wrist_left + wrist_right` 两路 USB，无 RealSense；接回 D435i 把 `"d435i"` 加回）；临时覆盖用 `video_cameras:=wrist_left,wrist_right`。
- **运行时开关/选路**：不用重启——streamer 暴露 `~/set_push_enabled` 服务与 latched `~/active_cameras` 话题（被关的轨发 2fps 黑帧静音），web 监控"系统"页有对应卡片；详见 `quest3_video_streamer/README.md`「运行时推流门控」。
- 关掉视频：`with_video:=false`。

## 测试

```bash
cd /home/robot/loopkok/sdk/MORLA/src/common/morla_bringup
# 纯逻辑（无 ROS）：
/usr/bin/python3 -m pytest test/test_controller_workpos_logic.py -q -p no:anyio
# 节点集成（需 ROS 源环境；本机 rclpy 缺失自动 skip，机器人侧跑）：
/usr/bin/python3 -m pytest test/test_controller_workpos_node.py -q -p no:anyio
```
