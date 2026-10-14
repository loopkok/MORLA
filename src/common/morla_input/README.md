# morla_input

Quest3 VR 手部 / 手柄 / 头部 / IOBT 身体数据接收节点。支持 UDP / TCP 有线 / TCP 无线。

对接 Quest 端 **astral-tracking**：Mixed 时手柄和手可同时有数据（通常左右各一种）；IOBT 开启时额外发全身关节，且头/手/手柄已在 **hips 身体系**。

---

## 功能

接收 Quest3 推流并发布 ROS2 话题：

| 线协议 | 内容 |
|--------|------|
| `Left/Right wrist:` | 人手腕 6DoF |
| `Left/Right landmarks:` | 21×3 腕局部关键点 |
| `Left/Right controller:` | Touch 手柄 6DoF |
| `Head pose:` | 头显 |
| `body iobt \| fid=...:` | IOBT 全身关节（hips 世界系进包，其余关节已为 hips 系） |

**下游（MORLA 工作空间）：**

- **Astral 臂**：订 `quest3/{side}_wrist_pose`。默认 `controller_as_wrist:=true`，握柄时手柄位姿会写到同一话题，臂 IK 不用改。
- **Wuji 手**：接收 `raw`（Unity→机器人轴、腕局部、21×3），由 `wujihand_retargeting` 执行自己的 MANO 映射。
- **XHand**：统一输入不做 MANO/手指补偿；`xhand_retargeting` 从 raw 恢复并执行旧 XNero MANO 与可选 `legacy_xhand_pinky` 兼容处理。

`morla_input` 的 ROS 启动只发布 wrist-local raw landmarks；手型变换只由末端适配包负责。

**坐标系：**

```text
IOBT 关：
  robot_world|vr_world
   ├── head                 ← quest3/head_pose
   └── wrist | controller   ← quest3/{side}_wrist_pose（手柄默认也写这里）
        └── landmarks       ← hand_landmarks/{side}（腕局部 robot axes + raw + EMA）

IOBT 开（Quest 已把头/手/手柄及 body 关节变到 hips 躯干系；hips 仍为世界系）：
  robot_world|vr_world
   └── hips                 ← quest3/hips_pose（人在房间里的位置）
  robot_body|vr_body        ← hips 为原点（+X右/+Y上/+Z前）
   ├── head / wrist|controller / body_joints   ← 均已 hips 相对，本节点只做轴约定转换
   └── landmarks            ← 仍是腕局部
```

`frame_id` 由 IOBT 状态决定，且做了两道稳定性处理，避免下游（`astral_arm_teleop` 的 `PoseProcessor`）跨系做 delta 导致 IK 失败：

- **sticky latch**：一旦收到过 body 包（`_body_ever_seen`），即使 body 包瞬时丢包（>0.4 s）也保持 `robot_body`，不再回退世界系——消除运行中 `frame_id` 抖动。
- **hold gate**：启动后 head/wrist/controller 在"帧确定"前不发布——收到首个 body 包即定躯干系；1 s 内无 body 包即定世界系（`_WRIST_SETTLE_S`）。从源头消除启动期 world→body 翻转，下游第一帧 wrist 即在正确系，`vr_init` 不会被错系污染。landmarks 为腕局部系、与参考系无关，不受闸门影响。

臂 IK 若仍把 `wrist_pose` 当世界目标：IOBT 开启后实际是相对躯干，走路不会把整条臂拖走。

**Unity → 机器人轴（`convert_to_robot:=true` 时）：**

| Unity (LH) | 机器人 |
|------------|--------|
| X 右 | **X 左** = `-X_u` |
| Y 上 | **Z 上** = `Y_u` |
| Z 前 | **Y 后** = `-Z_u` |

即 \((x,y,z)_r = (-x_u,\ -z_u,\ y_u)\)，姿态用 \(R_r = M R_u M^\top\)。

> 建议 HTS 开启 head tracking（若要用头控底盘）；腕不再依赖 head，无 head 也可发腕。

## 数据传输方式

| 模式 | 参数值 | Quest3 端配置 | PC 端 | 延迟 | 需要 WiFi 同网 |
|------|--------|-------------|-------|------|:---:|
| UDP | `udp` | UDP, 端口 9000, PC_IP | 无额外操作 | 低 | 是 |
| TCP 有线 | `tcp_wired`（yaml 默认） | TCP, localhost:8000 | `adb reverse tcp:8000 tcp:8000` | 最低 | 否 |
| TCP 无线 | `tcp_wireless` | TCP, PC_IP:8000 | 无额外操作 | 中 | 是 |

### Quest 数据包时效与去重

当前接收端要求每条 Quest 数据行带 `@q3 <session> <sequence> <utc_ms>|` 包头。UDP/TCP 共用检查器会拒绝
过期、未来时间戳、重复/乱序序号以及已退役 session 的迟到包；默认最大年龄 0.5 秒。Quest 与接收机系统时钟
需要同步。仓库中的独立脚本使用同一规则。

旧 APK 不带此包头，常规配置默认拒绝。只有明确运行旧设备时才可通过
`allow_legacy_packets:=true` 开启兼容；此时不具备源端时效、序号和 session 保护。
收到新 session 或运行中从 world 切换到 IOBT body frame 时，输入发布 `quest3/input_epoch`，清空接收端滤波缓存，
控制闸门停止活动写入并要求先松开、再按 grip 重新启动。

### TCP 有线（推荐）

1. Quest3 USB 连 PC，允许 USB 调试（`adb devices` 为 `device`）
2. `adb reverse tcp:8000 tcp:8000`
3. HTS 选 TCP，地址 `localhost`，端口 `8000`，并开启 head tracking
4. PC：`protocol:=tcp_wired`

## 相对 VR 原始数据的处理

| 数据 | VR 原始 | 本节点处理 | 最终 `frame_id` |
|------|---------|------------|-----------------|
| hips | Unity 世界（Quest 已重定向为躯干系朝向） | 可选 Unity→机器人轴 | `robot_world` / `vr_world` |
| body joints | Quest 已转 hips 相对（非 hips 关节） | 可选 Unity→机器人轴（不再做 pose_in_parent_frame） | `robot_body` / `vr_body` |
| head / wrist / controller | IOBT 关：世界；开：hips 系（Quest 已转） | 可选 Unity→机器人轴 | 关=`robot_world`；开=`robot_body` |
| landmarks | Unity 腕局部 21×3 | Unity→机器人轴、wrist-local raw、EMA | `hand_{side}` |

## 数据流

```text
Quest3 astral-tracking (Unity LH)
  ├── body iobt  → hips 世界(躯干系朝向) + body_joints（Quest 已转 hips 系）
  ├── head       → [convert_to_robot?] → quest3/head_pose
  ├── wrist      → [convert_to_robot?] → quest3/{side}_wrist_pose
  ├── controller → 同上，并默认镜像到 wrist_pose（臂 IK）
  └── landmarks
        → Unity→robot → wrist-centered raw 21×3
             → EMA → hand_landmarks/{left,right}
```

Mixed：左右可一边 `controller` 一边 `hand`。同侧 Quest 只发一种（手柄优先）。Meta 不支持 IOBT + Mixed 同时开。

## 话题

| 话题 | 类型 | `frame_id` | 说明 |
|------|------|------------|------|
| `hand_landmarks/left` | PoseArray | `hand_left` | 左手 21 点（腕局部） |
| `hand_landmarks/right` | PoseArray | `hand_right` | 右手 21 点（腕局部） |
| `quest3/left_wrist_pose` | PoseStamped | 世界或身体 | 左手腕；握柄时也可是手柄 |
| `quest3/right_wrist_pose` | PoseStamped | 世界或身体 | 右手腕 |
| `quest3/left_controller_pose` | PoseStamped | 世界或身体 | 左手柄（仅手柄行） |
| `quest3/right_controller_pose` | PoseStamped | 世界或身体 | 右手柄 |
| `quest3/left_controller_joy` | Joy | 世界或身体 | 左手柄按键/摇杆：axes=[trigger,grip,stickX,stickY]，buttons=[primary,secondary,stickPress,menu,triggerClick,gripClick] |
| `quest3/right_controller_joy` | Joy | 世界或身体 | 右手柄按键/摇杆（同上） |
| `quest3/head_pose` | PoseStamped | 世界或身体 | HMD |
| `quest3/hips_pose` | PoseStamped | `robot_world`/`vr_world` | IOBT hips，房间系 |
| `quest3/body_joints` | PoseArray | `robot_body`/`vr_body` | IOBT 关节，hips 系，顺序见 names |
| `quest3/body_joint_names` | String | latch | JSON 字符串数组，与 PoseArray 对齐 |
| `quest3/input_mix` | String | latch | 如 `left=ctrl right=hand` |
| `quest3/{side}_hand_markers` | MarkerArray | `world` | RViz（可选） |

### 手柄 Joy 量程与符号约定（实测）

`quest3/{side}_controller_joy`（`sensor_msgs/Joy`）的 axes/buttons：

| 索引 | 字段 | 范围 | 符号约定 |
|------|------|------|----------|
| axes[0] | trigger | [0, 1] | 0=松开，1=按到底（模拟量） |
| axes[1] | grip | [0, 1] | 0=松开，1=按到底（模拟量） |
| axes[2] | stickX | [−1, 1] | **左 −1 / 右 +1**，回中 0 |
| axes[3] | stickY | [−1, 1] | **后 −1 / 前 +1**，回中 0 |

buttons（6 位 mask，0/1）：`[primary(X/A), secondary(Y/B), stickPress, menu, triggerClick, gripClick]`。

实测（2026-08-26，UDP，双手柄）：左右手 stickX/stickY 均可达 ±1.0 满量程，回中为干净 0.000，无零点漂移。下游做差速/速度控制建议死区 0.05~0.1。

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `protocol` | `tcp_wired` | `udp` / `tcp_wired` / `tcp_wireless` |
| `udp_port` | `9000` | UDP 端口 |
| `tcp_port` | `8000` | TCP 端口 |
| `arm_side` | `right` | `left` / `right` / `both` |
| `ema_alpha` | `0.7` | landmarks EMA 平滑 |
| `viz` | `true` | RViz markers |
| `landmark_preprocess` | `raw` | ROS 接收器只输出 raw；MANO 由机器人末端包处理 |
| `allow_legacy_packets` | `false` | 明确兼容旧 Quest APK；关闭包时效/序号/session 防护 |
| `convert_to_robot` | `true` | `true`：Unity→机器人轴（X左 Y后 Z上）；`false`：保留 Unity 轴（landmarks 仅旧版翻 X） |
| `controller_as_wrist` | `true` | 手柄 6DoF 同时写到 `quest3/{side}_wrist_pose`，臂 IK 跟柄 |

Wuji 与 XHand 可从同一个 Quest 输入并行接收不同侧数据；profile 选择 landmark 来源与末端类型，
不得让 Quest 与 glove 同时发布同侧 `hand_landmarks/{side}`。

## 启动

### ROS2 节点

```bash
# UDP
ros2 run morla_input quest3_udp_mocap --ros-args -p arm_side:=both -p protocol:=udp

# TCP 有线
ros2 run morla_input quest3_udp_mocap --ros-args \
  -p arm_side:=both -p protocol:=tcp_wired

# Wuji 兼容
ros2 run morla_input quest3_udp_mocap --ros-args \
  -p arm_side:=both -p protocol:=tcp_wired \
  -p landmark_preprocess:=raw \
  -p convert_to_robot:=true

# 关闭机器人轴转换（保持 Unity 轴）
ros2 run morla_input quest3_udp_mocap --ros-args \
  -p arm_side:=both -p protocol:=tcp_wired -p convert_to_robot:=false
```

### 无 ROS 独立脚本

与本包处理链路一致（含可选 Unity→机器人轴），不依赖 `rclpy`。仅需 `numpy`；`--viz` 可选 `matplotlib`。

先准备 TCP 有线：

```bash
adb reverse tcp:8000 tcp:8000
# HTS：TCP + localhost:8000，开启 head tracking
```

**推荐：TCP 有线 + Wuji(raw) + 机器人轴转换 + 可视化 + 落盘**

```bash
cd /home/robot/loopkok/quest3/morla_input
python3 scripts/quest3_mocap_standalone.py \
  --protocol tcp_wired \
  --arm-side both \
  --landmark-preprocess raw \
  --convert-to-robot \
  --viz \
  --jsonl /tmp/mocap.jsonl
```

关闭轴转换时加 `--no-convert-to-robot`。

其他示例：

```bash
# 仅 TCP 有线打印（默认已开启 convert_to_robot）
python3 scripts/quest3_mocap_standalone.py --protocol tcp_wired --arm-side both

# 保持 Unity 轴
python3 scripts/quest3_mocap_standalone.py --protocol tcp_wired --no-convert-to-robot

# UDP + 可视化
python3 scripts/quest3_mocap_standalone.py --protocol udp --port 9000 --viz
```

也可在代码里直接用（把 `scripts` 加入 `PYTHONPATH`，或按文件路径导入）：

```python
from quest3_mocap_standalone import MocapConfig, Quest3MocapStandalone

cfg = MocapConfig(protocol="tcp_wired", arm_side="both", landmark_preprocess="raw")
rx = Quest3MocapStandalone(cfg, on_wrist=lambda side, pose: print(side, pose.position))
rx.start()
# ...
rx.stop()
```

## 在系统中的位置

```text
quest3_udp_mocap
  ├── hips_pose (世界系) + body_joints (hips 系)
  ├── head_pose（IOBT 关：世界；开：身体系）
  ├── wrist_pose / controller_pose（同上；默认手柄镜像到 wrist）
  ├── XHand:  raw landmarks → xhand_retargeting (local MANO/pinky profile)
  └── Wuji:   raw landmarks → wujihand_retargeting
```

## Changelog / Bug fixes

| 日期 | 项 | 说明 |
|------|----|------|
| 2026-10-12 | Quest 包时效 | ROS 接收端与独立脚本共用 session/sequence/UTC 时间戳检查，默认拒绝旧 APK 的无包头数据；TCP 行缓冲限长 |
| 2026-08-26 | 手柄按键流 | 解析 Quest 端新增 `{side} buttons:` 行 → `sensor_msgs/Joy`，发布 `quest3/{side}_controller_joy`（axes=[trigger,grip,stickX,stickY]，buttons=[primary,secondary,stickPress,menu,triggerClick,gripClick]），`package.xml` 增 `sensor_msgs` 依赖 |
| 2026-08-25 | IOBT 帧稳定 | sticky latch（见过 body 包即不回退世界系）+ hold gate（启动后 head/wrist/controller 在帧确定前不发布，`_WRIST_SETTLE_S=1.0`），从源头消除 `frame_id` 在 `robot_world`↔`robot_body` 间切换，修复下游 `astral_arm_teleop` 跨系做 delta 导致的 IK 失败；独立脚本同步 sticky latch |
| 2026-08-21 | body 关节不再二次转换 | Quest 端已把非 hips 关节转成 hips 相对；本节点 `_process_body_line` 移除 `pose_in_parent_frame`，hips 发世界、其余关节直接 `unity_pose_to_robot`，PoseArray 中 hips 为 identity 根。独立脚本同步 |
| 2026-08-21 | hips 躯干系对齐 | Quest 端用 `_hipsBoneToTorsoFix` 把 FullBody_Hips 骨头系(+X下/+Y前/+Z左)重定向为躯干系(+X右/+Y上/+Z前)；本节点无需改，轴映射 `unity_pose_to_robot` 因此从"用错轴"变为正确 |
| 2026-08-20 | Mixed + IOBT | 解析 `controller` / `body iobt`；IOBT 时 head/wrist 用 `robot_body`；手柄默认同写 wrist_pose |
| 2026-08 | 腕改回世界系 | 取消 wrist 相对 head；head 与 wrist 同属 `robot_world`/`vr_world` |
| 2026-08 | convert_to_robot | 启动可选 Unity→机器人轴 |
详见顶层 [README](../../README.md) 与 [CHANGELOG.md](../../CHANGELOG.md)。
