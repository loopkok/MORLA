# astral_data_collect 设计文档

日期：2026-08-28
状态：已获批（用户选定方案 A 及各决策项）
参考：xnero_ws-main/src/nero_dual_data_collect（架构模式）、VLA/lerobot @0cf86487（LeRobot v2.1 格式真值）、VLA/openpi（训练侧消费方）

## 1. 目标与范围

为 astral_ws 遥操作系统新增数据采集能力，产出可直接训练 OpenPI pi0.5 的数据集：

1. 采集：与遥操作并行录制全部可用流（关节反馈/指令、图像、任务文本、事件）
2. 对齐：离线统一到固定 fps 时间网格，生成逐帧 (state, action)
3. 清洗校验：规则化体检 + 隔离问题 episode
4. 导出：LeRobot **v2.1** 格式（OpenPI 钉死 lerobot==0cf86487 直接可读）
5. 回放：Rerun 可视化（不回放到真机/仿真）

非目标：在线对齐、自动切分技能、云端上传、真机回放。

## 2. 已确认决策

| 项 | 决策 |
|---|---|
| action/state 布局 | 采集前配置：左/右末端独立选 gripper/wuji/none；腰、头可选纳入 |
| 图像来源 | quest3_video_streamer 内部加高清抽头（与 WebRTC 同源零延迟差） |
| 回放范围 | 仅 Rerun 可视化 |
| 录制控制 | 键盘热键 + ROS 话题双通道 |
| 任务文本 | 每段 episode 一条，录制中可改，作用于下一段 |
| action 语义 | 默认 `action[t]=state[t+1]`（下一帧观测，可复现真值），可切指令流 |
| 架构 | 方案 A：采集期只写 raw HDF5；对齐/清洗/导出全部离线 |

## 3. 架构

```
采集期（与遥操作同机）:
  各驱动/遥操话题 ──┐
                    ├─► astral_data_collect_node ──► episodeN/{robot_data.h5, camera_data.h5, meta.json}
  streamer 抽头 ────┘        ▲ /data_collect/control, /data_collect/task
                    keyboard_controller（热键→话题）

离线（可换机）:
  raw ─► align_data.py ─► aligned_data.h5（含 JPEG 帧）
     ─► validate_data.py ─► 报告 + quarantine/
     ─► convert_to_lerobot.py ─► lerobot v2.1 目录
     ─► replay_rerun.py ─► rerun 可视化
```

### 3.1 streamer 高清抽头（collect_tap）

- 新模块 `quest3_video_streamer/collect_tap.py`：`CollectTapPublisher`，发布 `~/collect/{label}`（`sensor_msgs/CompressedImage`，BEST_EFFORT，depth=1）
- 复用 preview 的线程模型：producer 线程只做 put_nowait(maxsize=1)，JPEG 编码在独立 daemon 线程，绝不阻塞采集/WebRTC
- **按需编码**：仅当 `get_subscription_count() > 0` 才入队编码 —— 无人订阅时零成本，teleop-only 场景无感知
- 全分辨率、不降采样（fps 跟随源原生帧率，可配上限）、JPEG 质量默认 90
- 遵循 runtime gate：被静音的相机不出流
- 两个 source adapter 各加 `collect_hook` 属性（与 preview_hook 并列）：webcam 源回调 BGR、ros 源回调 RGB

### 3.2 采集节点 data_collect_node

订阅（全部 BEST_EFFORT sensor QoS；控制面 RELIABLE+TRANSIENT_LOCAL）：

| 流 | 话题 | 维度 | 条件 |
|---|---|---|---|
| body_state | /astral/joint_states | 18 | 真机时存在；含腰2+头2 |
| left/right_arm_state | /{side}_arm/joint_states | 7 | 恒有 |
| left/right_arm_cmd | /{side}_arm/joint_commands | 7 | 恒有 |
| left/right_hand_state | /{side}_hand/joint_states | 20 | 末端=wuji |
| left/right_hand_cmd | /{side}_hand/joint_commands | 20 | 末端=wuji |
| left/right_gripper_cmd | /{side}_gripper/command | 1 | 末端=gripper（闭合比 0..1） |
| left/right_gripper_rad | /{side}_gripper/joint_states | 1 | 末端=gripper（rad 回显） |
| head_state/cmd | /head/joint_states, /head/joint_commands | 2 | include_head |
| cam_{i} | /quest3_video_streamer/collect/{label} | JPEG | 按 cameras 配置 |
| 事件 | /teleop/armed, /teleop/disarm, /teleop/start（latched Bool） | - | 记入 meta.json events |

线程模型：rclpy 回调只入队（deque + Lock）；写盘线程批量 drain → HDF5 增量写（chunk 预分配 1000 行，关闭时裁剪）。丢弃 = 关闭后删文件，不落盘即无垃圾。

状态机：`IDLE →(start)→ RECORDING →(stop)→ SAVING → IDLE`；`discard` 任何时刻终止当前段并删除；`pause/resume` 暂停/继续写盘（同一段内）；`next` = stop+start。话题命令与键盘完全等价。

状态发布：`/data_collect/state`（latched JSON：状态/段号/时长/各流帧率/task 文本），web 监控可接。

### 3.3 schema（可配置布局）

yaml 配置（录制时冻结进 meta.json，离线全程使用）：

```yaml
arms: [left, right]          # 启用臂侧：[left] | [right] | 双臂；末端块跟随所属臂
end_effector_left: gripper   # gripper | wuji | none
end_effector_right: wuji
include_waist: false
include_head: true
cameras: [d435i, wrist_left, wrist_right]   # label 对应 streamer；第一个=对齐参考相机
dataset_fps: 30
action_source: next_state    # next_state | command
```

state 向量（顺序固定，仅含启用项）：`[left_arm(7)?, right_arm(7)?, left_ee?, right_ee?, waist(2)?, head(2)?]`
- `arms` 决定录哪侧臂（单臂/双臂）；该侧 arm 块与其 ee 块一起纳入或剔除
- 末端=gripper：1 维闭合比（0=开 1=合，同 pi0.5 dim6 约定），state 用最近一次指令比（夹爪无反馈，README 明示）
- 末端=wuji：state 用 20 维关节反馈
- action：默认 = t+1 帧 state（同布局）；`command` 模式 = 对应 *_cmd 流在 t 时刻的采样

### 3.4 raw 存储格式（每段一个目录）

```
{save_root}/{session}/episode{N:06d}/
├── robot_data.h5    /streams/{name}/values (N,D) f4 + timestamps (N,) f8
├── camera_data.h5   /cam_{i}/images vlen u8(JPEG) + timestamps f8
└── meta.json        schema 快照、task 文本、events、录制起止、包版本
```

时间戳统一 PC 墙钟秒（float64）：消息 header.stamp 有效（sec>0）用之，否则取到达时刻。所有话题同机同源，天然同钟。

### 3.5 align_data.py（离线对齐）

- 时间网格：均匀网格 `t0 + i/fps`，t0=参考相机首帧；**严格 1/fps 等距**（LeRobot check_timestamps_sync 容差 1e-4 的硬要求）
- 图像：每网格点取参考时刻最近帧（偏差 > 1.5/fps 记 warning），保留 JPEG 字节进 aligned h5
- 数值流：searchsorted 最近邻；采样空洞 > `max_gap_ms`（默认 100ms）→ 该段标记 quality_flags
- action=next_state：末帧无 t+1 → 复制末帧 state（hold）；另可配置追加 hold 帧（默认 10）
- 输出 `aligned_data.h5`：`/observation/state (T,D) f4`、`/action (T,D) f4`、`/timestamps f8`、`/cam_{i}/{images,src_timestamps}`、attrs（fps、schema json、quality flags）

### 3.6 validate_data.py（清洗校验）

逐 episode 规则（pass/warn/fail + JSON 报告）：

- 文件可读、各流 shape/长度自洽、时间戳单调
- NaN/Inf 检查（全部数值流）
- 各流实测频率 vs 期望（如 arm_state ≥45Hz、image ≥24Hz@30）
- 时间空洞 > 阈值计数；对齐帧图像时间偏差统计
- episode 时长下限（默认 2s）
- 关节跳变：相邻帧 |Δ| > 0.5 rad（遥操限速 4rad/s@30fps → 上限 0.133，0.5 为硬异常）
- 图像 JPEG 可解码抽检（首/中/尾）
- armed 覆盖率（事件时间线推算，未接管段标 warn）

`--apply` 把 fail 段移入 `quarantine/`（移动不删除）。

### 3.7 convert_to_lerobot.py（LeRobot v2.1 导出，OpenPI 兼容）

**不依赖 lerobot 包**，纯 pyarrow+av 手写 v2.1 布局（真值来源：VLA/lerobot @0cf86487）：

```
meta/info.json            codebase_version=v2.1, fps, robot_type, features, splits,
                          data_path=data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet
                          video_path=videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4
meta/tasks.jsonl          {"task_index", "task"}
meta/episodes.jsonl       {"episode_index", "tasks", "length"}
meta/episodes_stats.jsonl {"episode_index", "stats": {key: {min,max,mean,std,count}}}
meta/stats.json           全局聚合（加权 mean/std、min/max、count）
data/chunk-000/episode_XXXXXX.parquet
videos/chunk-000/observation.images.{cam}/episode_XXXXXX.mp4
```

- parquet 列：`observation.state`(list\<f32\>)、`action`(list\<f32\>)、`timestamp`(f32)、`frame_index`/`episode_index`/`index`/`task_index`(i64) + 每个 video key 一列 `struct{path: string, timestamp: f32}`（VideoFrame 结构）
- 视频：PyAV 编码，pts=i（time_base=1/fps），默认尝试 libsvtav1(crf=30,g=2,pix_fmt=yuv420p)，缺编码器回退 h264；写后回读 get_video_info 填 features[key].info
- 图像 stats：抽 ≤100 帧缩样 /255 计算（同上游 compute_episode_stats）
- state/action 维度：按 schema 实维（如双臂+gripper+wuji+头 = 7+7+1+20+2=37），**padding 到 32/64 由 openpi 侧配置决定**，导出器如实写维数并在 README 给出 openpi 配置示例
- `--verify`：若环境装了 lerobot@0cf86487，用 LeRobotDataset 实际加载抽帧比对

### 3.8 replay_rerun.py

加载 aligned_data.h5：逐相机 `rr.Image`（JPEG 解码）、state/action 各维 `rr.Scalars`、时间轴 `rr.set_time_nanos`，viewer 内拖动时间轴回放；支持 `--episode` 指定或连续多段（entity 路径分 episode）。

### 3.9 键盘控制器 keyboard_controller

独立 rclpy 节点，stdin 读键（无需焦点窗口）→ 发布 `/data_collect/control`：

| 键 | 命令 |
|---|---|
| s | start 新段 |
| q | stop 保存 |
| d | discard 丢弃当前段 |
| n | next（保存并开新段） |
| p | pause/resume |
| t | 输入任务文本（作用于下一段） |
| ESC | 退出 |

### 3.10 文件清单

新增 `src/astral_data_collect/`：package.xml、setup.py/cfg、config/data_collect.yaml、launch/data_collect.launch.py、模块 {schema, data_writer, data_collect_node, keyboard_controller, align_data, validate_data, convert_to_lerobot, replay_rerun}、test/（4 单测 + 1 采集冒烟）、README.md。

改动 `src/quest3_video_streamer/`：+collect_tap.py；webcam_source.py / ros_source.py 各加 collect_hook（2 行级改动）；streamer_node.py 加 `_setup_collect_taps`（仿 _setup_previews）；params.yaml 加 collect_tap 参数；README/CHANGELOG 更新。

## 4. 验证计划

- pytest（系统 python3.10）：schema 布局/维度、align（合成流验证最近邻+网格+hold）、validate（构造坏 episode 触发各规则）、convert（写 2 段小数据集 → pyarrow 回读校验 info/parquet/video 结构与 v2.1 真值逐字段比对）
- ROS 冒烟：进程内起 collect 节点 + 假发布者，start/stop 走话题，校验 HDF5 内容与 meta.json
- colcon build 通过；`ros2 run` 入口可用
- 代码审查：bugbot 全 diff 审查并修复发现
