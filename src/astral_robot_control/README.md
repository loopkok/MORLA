# astral_robot_control

ROS2 驱动包：把 [`astral_robot_sdk`](../../../astral_robot_sdk) 包成 Wuji/XHand 风格的
`joint_commands` / `joint_states` 话题，供后续 `astral_arm_teleop` 使用。

**本包不做 IK / Quest**；只负责连接控制板、收指令、发反馈。

## 话题契约

18 轴顺序与 SDK `ROBOT_JOINT_NAMES` 一致：

```text
[ left_arm×7 | right_arm×7 | waist×2 | head×2 ]
```

真机：`0x21/0x22`=腰，`0x31/0x32`=头。机械夹爪走 `0x97`/`0x98`（`set_gripper_angle`），不是 0x31/0x32。

| 话题 | 方向 | 内容 |
|------|------|------|
| `/left_arm/joint_commands` | sub | `JointState.position[7]` → `move_arm_js` |
| `/right_arm/joint_commands` | sub | `JointState.position[7]` → `move_arm_js` |
| `/astral/joint_commands` | sub | `JointState.position[18]` → `move_js`（有新数据时优先） |
| `/head/joint_commands` | sub | `[head_yaw, head_pitch]` → `move_head_js` |
| `/left_gripper/command` | sub | `Float64` 0=开 1=合 → `set_gripper_angle`（0x97）。**遥操唯一发布流**，rad 经本节点 `left/right_gripper_open_rad` 映射 |
| `/left_gripper/joint_commands` | sub | `JointState` 弧度 → `set_gripper_angle`（保留给直接发弧度的适配器；**勿与 /command 同时发**——两流覆盖同一目标会抽搐） |
| `/left_arm/joint_states` | pub | 左臂 7 |
| `/right_arm/joint_states` | pub | 右臂 7 |
| `/astral/joint_states` | pub | 全身 18（含腰+头） |
| `/head/joint_states` | pub | 头 2 |
| `/left_gripper/joint_states` | pub | 左夹爪（最近指令） |

- QoS：**BEST_EFFORT**（SensorData）
- 关节名可选；无名时按位置顺序；有名时按 `joint_layout.py` 对齐
- `command_timeout_s`（默认 1.5）：超时不再下发，避免僵持旧指令
- **单臂预设缺侧不补零**：只有一侧有新鲜 `joint_commands` 时，只向该侧电机下发目标
  （`set_target_positions` 单侧子集），**不**给缺席侧补零——否则单臂模式（如 no-right-arm）
  下发工作位/HOME/遥操轨迹时，停在任意位姿的另一侧实体臂会被拽向零位。缺席侧不发命令 =
  板端位置保持维持原位。双臂都有新鲜指令时仍走 `move_arm_js` 一次下发两臂。

左臂关节名：`left_shoulder_pitch` … `left_wrist_roll`  
右臂：`right_shoulder_pitch` … `right_wrist_roll`

## 服务

| 服务 | 说明 |
|------|------|
| `/astral_robot_driver/ready` | `one_click_ready`（WORK→POSITION→enable→zero）。**执行前清空缓存目标**——归零后 100Hz 重发只复读零位，不把阻尼前的陈旧位姿顶回来；**切回 POSITION 时用实测关节角重写板端目标**（`seed_from_current=True`），避免板端追踪其内部保存的"阻尼前位姿"导致臂抽回旧位姿 |
| `/astral_robot_driver/enable` | WORK→POSITION→enable，**不回零**（HOME/急停恢复用，避免抢 teleop 轨迹目标）。**幂等**：已上电直接成功跳过重复下发；板端在线但 `robot_powered` 位未确认也算下发成功（该位在此板子常不置位，仅真正离线才失败） |
| `/astral_robot_driver/home` | 归零：**已在阻尼先切回 POSITION**（motion_mode=1）→ **用实测关节角重写板端目标**（防回跳）→ `set_all_joints_zero`，并清缓存；未上电直接报"先 `~/ready`"，不静默 |
| `/astral_robot_driver/estop` | **真急停**：`e_stop` / disable（断电，臂失去保持力）。同时清缓存，避免重新上电瞬间 100Hz 重发旧位姿 |
| `/astral_robot_driver/damping` | 阻尼释放：`motion_mode=0`（电机仍上电、关节可手动拖拽）。同时清缓存 |
| `/astral_robot_driver/position` | 位置保持：`motion_mode=1`，**并用实测关节角重写板端目标**——阻尼拖拽后切回时保持在被拖拽后的当前位置，不回跳 |

> 典型遥操收尾流程：遥操中 → `damping`（**web 会自动先 disarm 遥操**，可手动拖回 home）→ `position` 或 `home`。
> 真急停 `estop` 会断电，恢复需重新 `ready`（遥操恢复则靠 HOME 端点先 `~/enable` 再 disarm+收回零位，
> 见 astral_arm_teleop 与 astral_web_monitor；web 启动/重启只拉栈、不自动 enable）。
> **手动硬件模式（ready/home/damping/estop/position）前务必先让遥操停止发流**——否则归零等一次性
> 目标会被仍在运行的 joint_commands 流（armed 遥操 / 启动归位 homing）下一帧覆盖。web 端点已自动
> 先发 `/teleop/disarm`；CLI 手调时请自行 `ros2 topic pub --once /teleop/disarm std_msgs/msg/Bool "data: true"`。
>
> **板端目标寄存器（阻尼回跳根因）**：阻尼（`motion_mode=0`）只让板端**忽略** 0x90 位置指令，
> 并不清空其内部保存的"上一次目标"（= 阻尼前位姿）；手动拖臂只改实测、不改板端目标。因此
> `ready`/`home`/`position` 一切回 POSITION，板端会立刻重新追踪旧目标把臂抽回——这三条服务已在
> `set_motion_mode(1)` 之后、显式目标之前用实测关节角重写板端目标（`_seed_target_from_current`）
> 兜住这一点。CLI 直接用 SDK 时，请给 `one_click_ready(..., seed_from_current=True)`。

## 依赖

```bash
# SDK（含 native）
cd /home/robot/loopkok/sdk/astral_robot_sdk
pip install -e .

cd /home/robot/loopkok/sdk/astral_ws
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_robot_control --symlink-install
source install/setup.bash
```

## 启动

```bash
# 无硬件冒烟（只打日志）
ros2 launch astral_robot_control astral_drivers.launch.py dry_run:=true

# 真机
ros2 launch astral_robot_control astral_drivers.launch.py \
  control_board_ip:=192.168.10.2 local_port:=8081
```

手动发指令：

```bash
ros2 topic pub --once /left_arm/joint_commands sensor_msgs/msg/JointState \
  "{name: [left_shoulder_pitch,left_shoulder_roll,left_elbow_roll,left_elbow_pitch,left_forearm_roll,left_wrist_pitch,left_wrist_roll],
    position: [0.1,0,0,-0.1,0,0,0]}"
```

看反馈：

```bash
ros2 topic hz /astral/joint_states
ros2 topic echo /left_arm/joint_states --once
```

## 参数

见 `config/astral_robot.yaml`。常用：`control_board_ip`、`local_port`、`dry_run`、`auto_ready`、`obs_hz` / `ctrl_hz`（板卡 SESSION）、`control_rate` / `state_publish_rate`（ROS 定时器；建议与板卡同档）。yaml 默认 **100 Hz**（遥操 IK 150 Hz；UDP 稳可再升到 150）。`lpf_enable` / `lpf_alpha` 经 SDK `set_lpf` 写 SESSION 目标低通（connect / `~/ready` 后下发；`one_click_ready` 进 WORK 可能重置，故每次 ready 会重写）。夹爪开合角度在此配置：`left/right_gripper_open_rad`（当前 **2.5** 实测全开 / `closed_rad` 0.0 合拢；若开到最大又弹回说明超机械行程，回调 ~2.0）——遥操侧（pinch/trigger）只发 0..1 闭合比，rad 以此处为唯一权威。上电瞬态说明：`one_click_ready` 会把夹爪归零(0)，遥操第一条指令随即开到 open_rad，看起来像"启动抽一下"，属正常。

## 驱动层诊断日志（`driver_log_file`）

> 驱动层/电机控制坑的完整排障历史（陈旧命令流/板端陈旧目标/使能/HOME 归位/夹爪竞态/死区）：
> `astral_ws/docs/motor-control-pitfalls.md`。

排"电机抽一下"（遥操/没遥操时臂/夹爪/头偶发突动）用。参数 `driver_log_file`（空=关）+ `driver_spike_mrad`（默认 30）。JSONL `kind` 区分：

| kind | 触发 | 用途 |
|---|---|---|
| `cmd` | 每个到达的 joint_commands（臂/全/头/夹爪） | 排"上游谁发坏指令" |
| `send` | 每控制拍实际下发 + fresh 标志 | 排"driver 陈旧重发" |
| `state` | 每实测关节发布 | 物理臂实际位置 |
| `srv` | 6 服务 + cache_clear + seed_from_current | 运动模式切换/重播种=抽动高危点 |
| `spike` | 命令/实测单拍跳变 > 阈值 | 电机"抽一下"直接记录（含跳前/跳后值） |

```bash
ros2 launch astral_robot_control astral_drivers.launch.py driver_log_file:=/tmp/driver.jsonl
# web 勾选「记录遥操日志」→ log_dir 模式自动落 {run_dir}/driver.jsonl
# 定位：spike 时间对齐 cmd（上游到）/ send（driver 发）/ srv（服务调用）
```

## 与遥操

全链路见 [`astral_arm_teleop`](../astral_arm_teleop/README.md)：

```bash
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py \
  with_driver:=true control_board_ip:=192.168.10.2
```

```text
quest3 / IK  →  /{left,right}_arm/joint_commands
pinch_gripper →  /left_gripper/command
                     ↓
              astral_robot_driver  →  astral_robot_sdk  →  板
                     ↓
              /{left,right,astral,left_gripper}/joint_states
```
