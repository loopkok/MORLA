# wujihand_retargeting

`hand_landmarks/{side}`（21 点 PoseArray）→ `/{side}_hand/joint_commands`
（`sensor_msgs/JointState`，20 维位置，按 index 填充）。

## 后端

| `retarget_backend` | 实现 | 配置 |
|--------------------|------|------|
| `official` | `wuji_sdk.retargeting.RetargetSession`（对齐 rob_station） | `hand_model:=wuji_hand\|wuji_hand_2` |
| `wuji_retargeting` | 开源 `wuji_retargeting.Retargeter`（AdaptiveOptimizer） | `retarget_wuji_lib_*.yaml` / Quest3 专用 yaml |
| `dexpilot` | `dex-retargeting` + Wuji URDF | `retarget_dexpilot_*.yml` |

## 话题契约

```text
订阅  hand_landmarks/{left,right}     PoseArray
发布  /{left,right}_hand/joint_commands   JointState.position[20]
QoS   BEST_EFFORT（SensorDataQoS，对齐 wujihand_driver）
```

## YAML（`wuji_retargeting` 后端）

| 文件 | 用途 |
|------|------|
| `config/retarget_wuji_lib_right.yaml` / `_left.yaml` | **手套**调参（含 `mediapipe_rotation`） |
| `config/retarget_wuji_lib_quest3_right.yaml` / `_left.yaml` | **Quest3**（`snap_mcp_to_robot`、旋转清零） |

`input_source:=quest3` 时，tuning / sim / real pipeline 会自动选用 quest3 yaml。
也可手动：

```bash
ros2 launch wujihand_retargeting wujihand_retarget.launch.py \
  hand_side:=right retarget_backend:=wuji_retargeting \
  wuji_lib_config_right:=/path/to/retarget_wuji_lib_quest3_right.yaml
```

启动日志示例：`wuji_retargeting[right] yaml=.../retarget_wuji_lib_quest3_right.yaml`。

## 启动

```bash
ros2 launch wujihand_retargeting wujihand_retarget.launch.py \
  hand_side:=right retarget_backend:=official

ros2 launch wujihand_retargeting wujihand_retarget.launch.py \
  hand_side:=right retarget_backend:=wuji_retargeting
```

常用参数：`nlopt_max_eval`（默认 25）、`smoothing_alpha`（默认 1.0=关；开源后端已有 `lp_alpha`）。

## 依赖

- `official`：`wuji-sdk`
- `wuji_retargeting`：可编辑安装的 [`wuji-retargeting`](https://github.com/wuji-technology/wuji-retargeting)（本仓库旁路常见路径 `../wuji-retargeting`）
- `dexpilot`：`dex-retargeting` + torch

## 策略所有权

当 `/policy_inference/state` 表明策略拥有手部输出时，节点停止 retarget 和 HOME 写入，并用实际执行关节状态
跟踪当前手型。HUMAN 接管后从该位置限速恢复；只消费最新、header 时间新鲜的 PoseArray，避免排队旧手势在
所有权切换后突然执行。

## Changelog / Bug fixes

| 日期 | 项 | 说明 |
|------|----|------|
| 2026-10-12 | 策略所有权门控 | 策略活动时禁止 retarget/HOME 双写；切回人工从实测手型限速接管，只保留最新鲜的关键点帧 |
| 2026-08 | QoS | `joint_commands` 改为 BEST_EFFORT，与 `wujihand_driver` 一致，避免真机收不到指令 |
| 2026-08 | Quest3 yaml | 新增 `retarget_wuji_lib_quest3_*.yaml`；`snap_mcp_to_robot` 对齐掌部 MCP 与 URDF `link1` |
| 2026-08 | Launch 透传 | `wuji_lib_config_{left,right}` 可从 sim/real pipeline 传入 |
| 2026-08 | 日志 | 启动时打印实际加载的 wuji_lib yaml 路径 |
