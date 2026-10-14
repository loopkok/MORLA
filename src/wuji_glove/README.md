# wuji_glove

Wuji Glove → `hand_landmarks/{left,right}`（`PoseArray`，21 点）。

与 `quest3_hand_mocap` 共用同一下游契约，可无缝切换到 `wujihand_retargeting` /
`wujihand_mujoco_sim` / `wujihand_control`。

## 数据流

```text
Wuji Glove (UDP / wuji_sdk)
        │  SDK 直接给出 21×3 MediaPipe keypoints
        ▼
wuji_glove_mocap
        │
        ├── hand_landmarks/left|right   (PoseArray)
        └── （可选）RViz MarkerArray
```

手套侧 **不做** MANO / OPERATOR2MANO；由 `wuji_retargeting.Retargeter` 统一做一次坐标变换（与 Quest3 `landmark_preprocess:=raw` 对齐）。

## 依赖

- `wuji-sdk`（含手套连接与 `hand_skeleton`）
- ROS 2 Humble + `geometry_msgs`

## 参数（`config/wuji_glove.yaml`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `hand_side` | `both` | `left` / `right` / `both` |
| `sn` | `""` | 手套序列号；空=自动 |
| `publish_rate` | `50.0` | 发布频率 Hz |
| `output_topic` | `hand_landmarks` | 话题前缀 |
| `viz` | `false` | 是否发 Marker |

## 启动

```bash
ros2 launch wuji_glove wuji_glove_mocap.launch.py hand_side:=right

# 或并入调参 / 真机 pipeline
ros2 launch wujihand_mujoco_sim wujihand_tuning.launch.py input_source:=glove
ros2 launch wujihand_control wujihand_real_pipeline.launch.py input_source:=glove
```

## 验证

```bash
ros2 topic hz /hand_landmarks/right
ros2 topic echo /hand_landmarks/right --once
```

## Changelog / Bug fixes

| 日期 | 项 | 说明 |
|------|----|------|
| 2026-08 | 新建 | 对齐 `quest3` 话题契约；发布 QoS 与下游 BEST_EFFORT 兼容 |
| 2026-08 | 坐标系 | 不在手套节点做 MANO，避免与 `wuji_retargeting` 双重变换 |
