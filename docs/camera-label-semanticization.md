# 相机 label 语义化（base / left_wrist / right_wrist）— 使用 · 迁移 · 部署

2026-09-18 重构：相机 label 从内核编号 `videoN` 改为**语义位置名** `base` / `left_wrist` /
`right_wrist`，全流水线（采集/转换/训练/推理）同步。本文档覆盖：约定、旧数据迁移、
新数据采集、训练与推理的配置对应、部署步骤。

## 1. 约定：label = 角色，跟硬件指纹走

```
base         realsense D435i 主视角（第三视角，对齐参考，首位）
left_wrist   左腕 USB 相机
right_wrist  右腕 USB 相机（2026-09-18 起启用；口位 0:1.3:1.0 已于 2026-09-20 实机 scan 填）
```

- `quest3_video_streamer` 的 `label_aliases`（params.yaml）按**硬件指纹**把设备钉成语义名：
  - realsense → **by-path 物理口位 + `-video-index0`**（2026-09-20 从 by-id 改来）。D435i 是
    复合 USB 设备（彩色+IR+depth 多接口），内核编 by-id 时 **index 串位**：实测 by-id 的
    video-index0 落到 0:2.1:1.0 的深度节点（scan 用不了），而真正的彩色（YUYV，0:2.1:1.3）
    在 by-id 里无条目 → by-id 序列号对复合相机不可靠，只能按口位钉；换口后按实机 scan 重填；
  - 左/右腕 USB → **by-path 物理口位**（廉价相机 by-id 序列号是假的，只能按端口钉）。
- 因此 `data_collect.yaml` / `policy_inference.yaml` / openpi `config.py` 里的 label
  **永不需要跟随内核 videoN 编号变**。换口/重插/换机只需改 streamer 的 alias 规则。
- ⚠ 模型槽名 `base_0_rgb` / `left_wrist_0_rgb` / `right_wrist_0_rgb` 是 **pi0.5 固定输入**，
  **不随 label 改名**——改的是"槽 → label"的映射，不是槽名。

## 2. 旧数据迁移（`src/common/morla_data_collect/morla_data_collect/pipeline/migrate_camera_labels.py`）

旧数据（语义化之前录的）用 videoN 作相机键。`migrate_camera_labels.py` 改名
`video8→base`、`video0→left_wrist`、`video2→right_wrist`：

```bash
# raw session（逐个 episode 的 camera_data.h5 + aligned_data.h5）
bash scripts/data_pipeline.sh migrate-cameras --raw ~/astral_data/raw/pick_place_merged
# v2.1（pi）与 v3（act）数据集（info.json features + videos 目录；parquet 无图像列不用动）
bash scripts/data_pipeline.sh migrate-cameras \
    --v21 ~/astral_data/pi/pick_place_merged --v3 ~/astral_data/act/pick_place_merged
# 全部 + 干跑预览
bash scripts/data_pipeline.sh migrate-cameras --raw ... --v21 ... --v3 ... --dry-run
```

实测：pick_place_merged 迁移 **408 处**（raw 100 段 × 2 h5 + pi + act）。改名在 h5/目录内
原子完成，可逆（反向跑同名工具或手动改名）。

## 3. 新数据采集（3 相机）

`data_collect.yaml` 已配 `cameras: ["base", "left_wrist", "right_wrist"]`（3 路全录）。
- 对齐参考 = cameras[0] = base；
- 采集前 web 视频卡片核对 base/left_wrist/right_wrist 三路有图；
- 旧数据（2 相机）与新数据（3 相机）**不能混在同一数据集**（列名/槽位不同，见下）。

## 4. 训练配置对应（openpi）

`VLA/openpi/src/openpi/training/config.py` 提供 4 个配置：

| 配置 | camera_map | 适用 |
|---|---|---|
| `pi05_astral` / `pi05_astral_lora` | 2 槽（base, left_wrist） | **旧数据**（迁移后 2 相机）；右腕槽零填充 mask=False |
| `pi05_astral_3cam` / `pi05_astral_lora_3cam` | 3 槽（+ right_wrist） | **新数据**（3 相机） |

```bash
uv run scripts/train.py pi05_astral_lora        --exp-name=old_data   # 旧数据 2 槽
uv run scripts/train.py pi05_astral_lora_3cam   --exp-name=new_data   # 新数据 3 槽
```

## 5. 推理配置对应（policy_inference）

`policy_inference.yaml` 当前 `camera_map: {base_0_rgb→base, left_wrist_0_rgb→left_wrist}`
（2 槽，匹配**已部署的 2 相机模型**；`right_wrist_0_rgb` 缺省 → serve 端零填充）。
**待 3 相机模型部署后**：`cameras` 加 `right_wrist`、`camera_map` 加
`"right_wrist_0_rgb": "right_wrist"`。

> ⚠ **关键区分：节点 camera_map 的值 = collect label；serve `--slot-map` 的值 = 模型特征键**。
> 节点订阅 `collect/{label}`（语义名 base/left_wrist），发送 `observation/camera/{槽名}`；
> serve 把图像喂到 `observation.images.<slot-map 值>` 键下，**值必须 = 模型的
> `input_features` 图像键**（旧模型=video8/video0，新模型=base/left_wrist）——两者在
> label 语义化后不再相等。`serve_policy.env` 的 `SLOT_MAP` 默认按旧模型写 video8/video0；
> 部署新模型时同步改成 base/left_wrist（+right_wrist）。

## 6. 部署步骤

```bash
# 机器人侧（nvidia@nvidia-desktop，~/loopkok/astral_ws）
git pull                                  # 拉到 2026-09-18 的改动
colcon build --packages-select quest3_video_streamer morla_data_collect morla_policy_inference
# 重启 streamer → 日志 "alias ... matched ..." 是命中提示非错误
# web 视频卡片应显示 base=realsense、left_wrist=左腕、right_wrist=右腕
# 验证：ros2 topic hz /quest3_video_streamer/collect/base
```

⚠ 非 symlink 构建时 yaml 是构建期拷贝，**必须重新 colcon build** 才生效。

## 7. 换相机 / 换口时的唯一维护点

只改 `quest3_video_streamer/config/params.yaml` 的 `label_aliases`：

```yaml
label_aliases:
  - "platform-3610000.usb-usb-0:2.1:1.3-video-index0=base"  # realsense：换口改 by-path（彩色口位）
  - "platform-3610000.usb-usb-0:2.2:1.0-video-index0=left_wrist"   # 左腕：换口改 by-path
  - "platform-3610000.usb-usb-0:1.3:1.0-video-index0=right_wrist"  # 右腕：换口改 by-path（实机 scan 填）
```

改完重启 streamer 即可；下游 data_collect / openpi / policy_inference 的 label 引用零改动。
