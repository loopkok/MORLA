# wujihand_mujoco_sim

Wuji Hand MuJoCo 仿真 + **三层调参可视化**（TuningViewer）。

## 两种用法

| Launch | 作用 |
|--------|------|
| `wujihand_tuning.launch.py` | 进程内 retarget + 橙/青/白骨架 + 网格；**热重载** retarget yaml |
| `wujihand_sim_pipeline.launch.py` | 输入 → `wujihand_retargeting` → MuJoCo 跟 `joint_commands`（无真机） |

**不要**与 `wujihand_control` 真机 pipeline 同时订阅同一条 `joint_commands`。

## Tuning 三层颜色

| 层 | 颜色 | 含义 |
|----|------|------|
| MediaPipe 输入 | 橙 | 变换后的输入关键点 |
| Scaled target | 青 | 应用 `segment_scaling` 后的目标（仅 scaling≠1 时与橙分离） |
| Robot FK | 白 | 优化后的关节角做 FK |

网格透明度：`config/tuning_viz.yaml` → `robot_mesh.alpha`（设 `0.0` 可隐藏网格）。

## 启动

```bash
# Quest3 调参（自动 landmark_preprocess:=raw + quest3 yaml）
ros2 launch wujihand_mujoco_sim wujihand_tuning.launch.py \
  input_source:=quest3 retarget_backend:=wuji_retargeting

# 手套调参
ros2 launch wujihand_mujoco_sim wujihand_tuning.launch.py \
  input_source:=glove retarget_backend:=wuji_retargeting

# 离线仿真管线
ros2 launch wujihand_mujoco_sim wujihand_sim_pipeline.launch.py \
  input_source:=quest3 retarget_backend:=wuji_retargeting
```

Quest3 有线：`adb reverse tcp:8000 tcp:8000`，HTS 填 `localhost:8000`。

启动后应看到：`Hot-reload: retarget_wuji_lib_quest3_right.yaml`（Quest3）或手套对应 yaml。

## 常用参数

| 参数 | 说明 |
|------|------|
| `input_source` | `glove` / `quest3` / `none` |
| `retarget_backend` | `wuji_retargeting` / `official` / `dexpilot` |
| `hand_side` | `left` / `right` |
| `retarget_config` | 覆盖 wuji_lib yaml；空 + quest3 → 自动 quest3 yaml |
| `viz_config` | 默认 `config/tuning_viz.yaml` |

## Changelog / Bug fixes

| 日期 | 项 | 说明 |
|------|----|------|
| 2026-08 | Quest3 默认 | tuning/sim：`landmark_preprocess:=raw`，关掉 XHand 小指自适应 |
| 2026-08 | Quest3 yaml | `input_source:=quest3` 自动加载 `retarget_wuji_lib_quest3_*.yaml` |
| 2026-08 | QoS | sim 订阅 `joint_commands` 使用 BEST_EFFORT |
| 2026-08 | 可视化说明 | 橙/白 MCP 不一致在未 snap 时属掌宽差异；quest3 yaml 启用 `snap_mcp_to_robot` |
| 2026-08 | dexpilot/official 手握拳不动 | MJCF `kp` 很软，只写 `ctrl`+`mj_step` 几乎不动，又停在 mid-range；改为写 `qpos`+`mj_forward`，初始张开 |
