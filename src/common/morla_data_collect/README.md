# morla_data_collect — VLA 数据采集包

## MORLA统一profile模式

遥操/仿真、策略和数采都从同一个 `morla_core` robot profile 读取冻结的 `FrozenSchema`；数采不会再根据独立的 Astral arms/end-effector 参数拼造 Nero/XHand 布局。录制开始时将 `profile_id`、`runtime_mode`（`sim`/`real`）、schema/layout hash 与源资产版本写入 `meta.json`。数据保存到 `{save_root}/{profile_id}/{session}/episodeNNNNNN/`，默认根目录受 `MORLA_DATA_ROOT` 控制。

```bash
# 先启动同profile的遥操/仿真，再启动数采。mode 必须写明，避免把合成反馈标为实测。
ros2 launch morla_data_collect data_collect.launch.py \
  profile_id:=nero_left_xhand runtime_mode:=sim session:=pick_place \
  keyboard:=false vr_control:=false

# 从 MORLA 工作区运行统一离线工具
scripts/data_pipeline.sh align --session /path/to/raw/nero_left_xhand/pick_place
scripts/data_pipeline.sh validate --session /path/to/raw/nero_left_xhand/pick_place
scripts/data_pipeline.sh export-openpi \
  --session /path/to/raw/nero_left_xhand/pick_place \
  --output /path/to/lerobot/nero_left_xhand --codec h264
```

profile布局由注册表统一维护：Astral左臂+夹爪为8维，Nero单臂+XHand为19维，Astral双臂+左夹爪+右Wuji为35维，Nero双臂+双XHand为38维；其它活动Profile见 `morla-profile validate-all`。XHand力/温度等传感器独立保留在raw/aligned sidecar，不拼进state/action。仿真可缺少某些可选传感器，报告会明确给warning；必需流缺失则录制校验失败。

为 OpenPI pi0.5 等 VLA 模型采集训练数据。与遥操作链路并行运行，四段式：
**采集（raw HDF5）→ 离线对齐（固定 fps 网格）→ 清洗校验 → LeRobot v2.1 导出**，
外加 Rerun 可视化回放。设计文档见 `docs/superpowers/specs/2026-08-28-astral-data-collect-design.md`。

```
遥操话题 ──┐
           ├─► data_collect_node ─► ~/astral_data/{session}/episode{N:06d}/
streamer 抽头 ┘     ▲                    robot_data.h5 + camera_data.h5 + meta.json
        keyboard_controller（热键→ /data_collect/control）

离线：align_data → validate_data → convert_to_lerobot → replay_rerun
```

## 1. 前置条件

1. 遥操作链路运行中（`morla_bringup` / 真机或仿真）
2. `quest3_video_streamer` 运行中且 `collect_tap: true`（默认开）——
   抽头向 `/quest3_video_streamer/collect/{label}` 发全分辨率 JPEG，
   **仅在有订阅者时编码**，不采集时零开销
3. 依赖：`pip install h5py pyarrow`（采集只需 h5py；导出需 pyarrow+av；回放需 rerun-sdk）

## 2. 采集前必查（schema 冻结进每段 meta.json）

本节的 `arms/end_effector/include_waist/include_head` 参数只用于兼容的无 `profile_id` Astral schema。统一 MORLA 采集应显式传 `profile_id`；该profile内的state/action/stream布局不可由这些旧参数覆盖。

**参数来源约定**：`config/data_collect.yaml` 是所有参数的唯一默认值来源。
launch 参数默认空串，**只在显式传入时**（CLI `xxx:=` 或 web 预设 args）
覆盖 yaml 同名字段——改 schema 一律改 yaml，改完重启节点生效
（web 卡片「重启节点」或重 launch）。

> 旧版 launch 曾用自身默认值静默盖掉 yaml（改了 yaml 却采出旧 schema），已修复。

可配项：

| 参数 | 默认 | 说明 |
|---|---|---|
| `arms` | `[left, right]` | 启用臂侧：`["left"]` 仅左臂 / `["right"]` 仅右臂 / 双臂；launch 里用逗号串 `arms:=left` |
| `end_effector_left/right` | `gripper` | 左右末端：`gripper` / `wuji` / `none`（跟随所属臂，臂未启用不生效） |
| `include_waist` | `false` | 腰 2 维（来自 `/astral/joint_states[14:16]`） |
| `include_head` | `false` | 头 2 维（body_state[16:18]，无 body 时回退 `/head/joint_states`） |
| `cameras` | `[d435i, wrist_left, wrist_right]` | 相机 label，须与 streamer 实际生效值一致；**第一个 = 对齐参考相机**（episode 区间由其首尾帧界定）。auto_scan 下 label 是 `videoN`——直接把 d435i 那路的 videoN 写第一个即可；**videoN 编号不稳定（重启/重插会重排），每次开录前在 web 视频卡片（sysfs 名含 RealSense）或 `gate_state` 核对一次** |
| `dataset_fps` | `30` | 对齐网格与 LeRobot fps |
| `action_source` | `next_state` | `next_state`：action[t]=state[t+1]；`command`：指令流采样 |
| `hold_frames` | `10` | 对齐时末尾追加的保持帧 |
| `save_root` / `session` | `~/astral_data/raw` / `default_task` | 存储位置；目录约定见 §4（raw/ pi/ act/ 三文件夹） |

state 向量布局（顺序固定，仅含启用项）：`[left_arm(7)?, right_arm(7)?, left_ee?, right_ee?, waist(2)?, head(2)?]`
- `arms` 决定录哪些臂（单臂/双臂）；末端块跟随所属臂
- 末端=gripper：1 维闭合比（0=开 1=合，同 pi0.5 dim6 约定）。**夹爪无反馈，state 用指令回显**。
- 末端=wuji：20 维关节反馈；command 模式下 action 用 `/left|right_hand/joint_commands`。
- 腰/头在采集侧无独立指令话题，`command` 模式下这两块仍是 next_state 语义。

维度速查：双 gripper=16；gripper+wuji+头=37；双 wuji+腰+头=58；单臂 gripper=8；单臂 wuji+腰+头=31。

## 3. 采集操作

```bash
# 终端 1：采集节点
ros2 launch morla_data_collect data_collect.launch.py \
    session:=pick_place end_effector_right:=wuji include_head:=true

# 终端 2：键盘（也可只发话题，见下）
ros2 run morla_data_collect keyboard_controller
```

**VR 手柄控制（默认随 data_collect.launch.py 同启，`vr_control:=false` 可关）**：
单人采集手不离 Quest 手柄即可控制录制——**右手柄 A 键=开始录制**（仅 IDLE 有效）、
**B 键=停止保存**（仅录制中有效）、**摇杆按下=丢弃当前段**（删除文件并回 IDLE，
仅录制中有效，误按即丢——需要确认的破坏性操作请走 web 卡片）。按键上升沿触发
（长按不重复）；采集节点未运行或状态不合法时按键静默忽略（节点 info 日志说明）。
需遥操链路 `morla_input` 在发布 `quest3/right_controller_joy`（mocap 没跑则无动作）。

**推理 HUMAN 时 A 键 = HUMAN 释放（2026-09-17 起）**：主路由在**常驻遥操栈**的
`controller_start_gate`（订 `/policy_inference/state`，activity==human 时右手 A 发
`/policy_inference/cmd`="release"），不依赖数采栈。`vr_collect_control` 也保留同路由
（数采栈运行时 HUMAN 下 A 归推理域）；其余状态 A 仍为采集 start。B/摇杆语义不变。

**左手柄 X 键 = 段间回位**（teleop 侧 `controller_workpos_gate`，随 `full_teleop` 同启）：
停止跟随 VR + **直接**回到 init_pose 工作位（**不经 init_waypoints**，`/teleop/init_direct`），
供段与段之间摆放物品。区别于系统 tab「工作位」的途经点路径（`/teleop/init`）。
**录制中/暂停/保存中按 X 无效**（闸门按 `/data_collect/state` 拦截，防毁段）；无数采节点
运行时（纯遥操）照常生效。与 web 数采卡片「段间回位」按钮同功能。**顺序要求**：B 保存后
等状态回 IDLE 再按 X；先 grip（重标定 armed）再 A 开始下一段（否则新段 armed 覆盖率不足
触发 validate W5）。完整流程见 `morla_bringup/README.md`「遥操操作步骤」。

**web 端操作（推荐）**：「监控」tab 顶部数据采集卡片右上角**启动/重启/停止节点**
（独立泳道，与遥操预设解耦可并存；CLI 启动的节点同样受控），卡片本体提供
开始/停止保存/下一段/暂停继续/丢弃 + 下一段任务文本 + 录制目录（session）切换
+ 实时状态徽标与流率 + **「本次采集数据」实时下拉**（完整文件夹路径、当前段
精确样本数/每相机帧数、session 累计磁盘占用——采集节点 1Hz 发布，见下）。
四种控制面（web 卡片 / 键盘 / VR 手柄 / 话题）完全等价，可混用。卡片带
**动态状态提示行**（当前状态能做什么、保存中按键会被忽略并计入「忽略指令」）
与**操作键位/门控小抄**（VR/键盘键位、各自合法状态、非法按键被静默忽略）。

**防护**（对应实测双开事故的三层根因）：
- **单例锁**：`/data_collect/control` 是全局控制面，谁订阅谁开录。节点启动时
  对 `/tmp/morla_data_collect_domain{ROS_DOMAIN_ID}.lock` 取排他锁，第二个
  节点（**哪怕 session 不同**）构造即拒绝。注意：老版本残留进程不持锁，
  锁管不住它——靠下两条兜底。
- **段号原子占位**：`mkdir` 抢号，撞号让位——即使残留老节点在写，新节点也
  不会和它撞 episode 编号互写（此前一个 start 在两个节点分别写出
  000000/000001）。
- **多节点检测**：web 后端按 `/data_collect/state` 发布者计数，>1 时数采卡片
  红条提示有残留节点（这是发现老进程的手段）。
- **低帧率告警**：录制中参考相机（`cameras[0]`）实率低于 `dataset_fps` 一半时，
  state JSON 带 `low_fps_warning`、节点日志 5s 节流 WARN、web 卡片红条提示——
  开录后瞄一眼卡片即可发现相机链路异常。
- **空录告警**：录制启动约 2s 数值/图像仍全 0 → state JSON `empty_warning` +
  节点 `EMPTY-REC` WARN + web 红条。low_fps 只管参考相机半速，管不到"源未就绪
  全 0"的空录（忘了 armed/teleop 没发布/抽头没开）；每段独立判定、段结束清空。
- **吞指令留痕**：状态不合法被忽略的指令（如 SAVING 期按 start）计数入 state JSON
  `ignored` + web 卡片 chip——web 按钮有 disabled 视觉，键盘/VR 没有，靠它发现
  "以为开了实际没录"。

热键：`s` 开始 / `q` 停止保存 / `d` 丢弃当前段 / `n` 保存并开新段 /
`p` 暂停继续 / `t` 输入下一段任务文本 / `ESC` 退出键盘（不影响采集）。

等价话题接口（UI/脚本可接）：

```bash
ros2 topic pub --once /data_collect/control std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /data_collect/task std_msgs/msg/String "{data: '把方块放进盒子'}"
ros2 topic pub --once /data_collect/session std_msgs/msg/String "{data: 'pick_place'}"   # 切换录制目录（仅 IDLE 生效）
ros2 topic echo /data_collect/state   # latched JSON：状态/段号/各流频率/丢弃计数/
                                       #   本次采集量（folder/episode_counts/camera_counts/
                                       #   episode_bytes/session_bytes，1Hz 刷新）
```

## 4. 离线流水线

**停止保存后可直接回放 raw 录制段**：采集格式仍为 `robot_data.h5`、`camera_data.h5` 和
`meta.json`。录制停止并保存后，可在 `morla_web_manager` 推理卡点击“浏览…”选择 episode，
或给 `morla_policy_inference` 传 episode/session 路径；无需先运行对齐、修复或 LeRobot 导出。
回放只读机器人状态流并按 profile 布局重采样，不读取相机或改变原始文件。策略训练仍走下方
的对齐、质量校验和导出流程；回放规则、安全执行和 HITL 接管详见
[`morla_policy_inference`](../morla_policy_inference/README.md) 的“回放采集数据”。

**统一实现目录**：[`morla_data_collect/pipeline/`](morla_data_collect/pipeline/README.md)。对齐、校验、诊断、清洗、导出、回放和验证都在这里；旧脚本和父包离线模块已删除。统一入口：

```bash
# 从 sdk 目录运行
bash MORLA/scripts/data_pipeline.sh --help
bash MORLA/scripts/data_pipeline.sh align --session astral_data/work/pick_place
bash MORLA/scripts/data_pipeline.sh repair --session astral_data/work/pick_place --dry-run
bash MORLA/scripts/data_pipeline.sh process-openpi astral_data/work/pick_place astral_data/pi/pick_place
```

完整步骤和文件地图见上述目录说明。一键流程不自动执行 quantify/repair；采集、schema、写盘代码留在父包。

**目录约定（三文件夹，按 session 组织）**：

```text
astral_data/
├── raw/<session>/      ← 采集原始数据（save_root 默认已指向这里）
├── pi/<session>/       ← openpi 数据集（v2.1）
└── act/<session>/      ← ACT 数据集（v3 + 自检）
```

同一 raw session 跑两条一键脚本，各进各的文件夹。以下示例以 `pick_place` 为 session：

```bash
# ① 对齐（raw → aligned_data.h5，严格 1/fps 网格）
ros2 run morla_data_collect data_pipeline align --session ~/astral_data/raw/pick_place

# ② 校验（规则体检 + 报告；--apply 把 fail 段移入 quarantine/，移动不删除）
ros2 run morla_data_collect data_pipeline validate --session ~/astral_data/raw/pick_place --apply

# ③ 导出 LeRobot v2.1（OpenPI 直接可读）
ros2 run morla_data_collect data_pipeline export-openpi \
    --session ~/astral_data/raw/pick_place --output ~/astral_data/pi/pick_place

# ④ 回放（Rerun：图像 + 关节曲线 + 时间轴）
ros2 run morla_data_collect data_pipeline replay --session ~/astral_data/raw/pick_place --episode 0
```

一键脚本（openpi 与 ACT 从**同一 raw 目录**各取所需、输出独立文件夹）：
`bash MORLA/scripts/data_pipeline.sh process-openpi <raw> <pi输出>`（对齐→校验→v2.1）与
`bash MORLA/scripts/data_pipeline.sh process-act <raw> <act输出>`（对齐→校验→v3+自检）。**默认把校验失败段隔离到 `session/quarantine/`**
（移动不删除，可逆）；隔离后仍有 fail 段会中止转换——防坏段进训练集。
`--no-quarantine` 只关闭自动隔离；导出前的完整性检查始终生效，失败段必须先修复。

校验规则（fail 必须处理 / warn 人工过目）：F1 文件缺失、F2 流缺失或为空、
F3 NaN/Inf、F4 时间戳非有限或非递增、F5 时长 <2s、F6 维数/长度/schema 不一致、F7 严重对齐缺口；W1 频率不足、W2 采样空洞、
W3 关节跳变 >0.5rad、W4 JPEG 不可解码、W5 armed 覆盖率 <50%、W6 对齐偏差大、W7 旧版 armed 事件矛盾导致覆盖率未知。导出时全量解码 JPEG，坏帧拒绝导出。
报告写在 `{session}/validation_report.json`。

## 5. LeRobot v2.1 导出格式

布局与 `VLA/lerobot @0cf86487`（OpenPI pyproject 钉死提交）逐字段一致：
`meta/info.json`（codebase_version=v2.1、features、data/video_path 模板）、
`meta/{tasks,episodes,episodes_stats}.jsonl`、`meta/stats.json`、
`data/chunk-000/episode_XXXXXX.parquet`、`videos/chunk-000/{key}/episode_XXXXXX.mp4`。

- **parquet 只含非视频列**（v2.1 规范）：视频帧由读取侧按 `timestamp` 列 +
  meta 的 `video_path` 模板解析。写 `struct{path,timestamp}` 视频列会让 openpi
  锁定的 lerobot 0.1.0 在 `torch.tensor(dict)` 处崩（有回归断言禁止该列）。
- 视频编码默认 libsvtav1（crf=30, g=2, yuv420p；限内存参数 `lp=2:lookahead=16`），
  不可用自动回退 h264；同相机各段 mp4 时间基/分辨率一致（后续升版按此假设串接）。
  核数多的机器可在内存充裕时 `ASTRAL_AV1_LP=8` 放开 AV1 并行（默认 2 是内存红线；
  实测 20 核双核 AV1-224 ≈78 帧/秒，720 ≈8 帧/秒）。
- 图像默认在**转换期** letterbox 到 224×224（等比+对称黑边，复刻 openpi
  `resize_with_pad` 几何），`--image-size 0` 保留原分辨率；训练侧
  `ResizeImages(224,224)` 变恒等操作，解码后不再缩放。

OpenPI 侧配置示例（state/action 维度 = 你的 schema 实维，padding 在 openpi config 里做）：

```python
# src/openpi/training/config.py 你的 TrainConfig：
data=LeRobotAlohaDataConfig(
    repo_id="<local path or pushed repo>",
    default_prompt="pick up the cube",
    # repack/transform 里把 state/action pad 到模型 max_dim（如 32）；
    # 若 schema 维度 > max_dim（如双 wuji 58 维），需调大模型 max_action_dim。
),
```

验证装载（在 openpi 环境内，金标准检查）：

```bash
cd VLA/openpi && uv run python -c "
from openpi.training.data_loader import create_data_loader
... # 或直接用 lerobot LeRobotDataset(root=...) 抽帧比对
"
```

## 5b. ACT 目标：专属转换 `convert_to_act.py`（推荐，内置自检）

**从 raw 会话目录直接产出 ACT 可训数据集**（官方 lerobot v3 布局），把 ACT 训练器的
需求做成硬保证 + 两级自检，不依赖外部自觉：

```bash
python3 -m morla_data_collect.pipeline.convert_to_act \
    --session ~/astral_data/raw/pick_place \
    --output ~/astral_data/act/pick_place \
    --image-size 224                 # letterbox 边长：224(默认)/480/720 或任意正整数；0=原生
    --check-python ~/miniconda3/envs/lerobot/bin/python   # 给则跑深度自检（金标准）
    # --keep-v21 <dir> 保留 v2.1 中间产物 | --overwrite 允许覆盖输出 | --force-align 重对齐

# 或从已有 v2.1 直接升版（openpi/ACT 共享 v2.1 中间层，不重编码，秒级）：
python3 -m morla_data_collect.pipeline.convert_to_act \
    --v21-root ~/astral_data_lerobot --output ~/astral_data_act
```

分辨率可选 224/480/720/原生：原生（0）会先探测各相机原生尺寸，不一致即报错
（ACT 要求全部相机同 shape，如 base=1080p、left_wrist=720p 需 letterbox 统一）。

链路 = 对齐 → v2.1（临时中间产物）→ v3 → **自检**：
- **结构级**（脚本内强制，不过即退出）：`stats.json` 含 `observation.images.{每路}` +
  `observation.state` + `action` 的 mean/std（ACT `VISUAL/STATE/ACTION→MEAN_STD` 硬需求）、
  各相机**同 shape**（ACT 只支持同 shape）、`tasks` 非空、parquet/视频可读可解码；
- **深度级**（`--check-python` 指向现代 lerobot 环境）：用该解释器真装载
  `LeRobotDataset` + 构建 ACT 预处理管线 + 逐帧解码——这是"无缝衔接官方 ACT"的实测证明。

训练命令脚本结尾直接打印。**训练侧数值口径**：`/255` 归一化由 lerobot 装载器自动做，
`mean/std` 用本数据集 stats.json（图像统计按 /255 抽样，与装载刻度一致），**无 resize**
（224×224 同 shape 由转换期 letterbox 保证）。

### 5b-底层工具：v2.1 → v3.0 升版 `convert_to_lerobot_v3.py`
`convert_to_act.py` 内部复用此通用升版（镜像官方 `convert_dataset_v21_to_v30`），
也可单独用于把已有 v2.1 升版给其它现代 lerobot 策略：

OpenPI（pinned lerobot 0.1.0）只认 v2.1；现代 lerobot（`VLA/lerobot`，>=0.6，
ACT 等策略共用同一读取管线）的数据集 `codebase_version` 已是 **v3.0**，读取侧对
v2.1 直接 `raise BackwardCompatibilityError`。`convert_to_lerobot_v3.py` 把上面
的 v2.1 产物**原样升版**为 v3.0（源目录只读，输出独立目录）：

```bash
# 用法：--output 已存在需 --overwrite（不会覆盖 --v21-root）
python3 -m morla_data_collect.pipeline.convert_to_lerobot_v3 \
    --v21-root ~/astral_data/lerobot/pick_place \
    --output   ~/astral_data/lerobot_v3/pick_place
```

升版后即现代 lerobot 数据集：`videos/{cam}/chunk-*/file-*.mp4`（同 chunk 的段
用 PyAV ffconcat **流拷贝串接、不重编码**，元数据时间窗按各段实测时长叠加）、
`data/chunk-*/file-*.parquet`、`meta/{tasks.parquet, episodes/chunk-*/file-*.parquet,
info.json, stats.json}`（stats 原样沿用，无需重算）。

ACT 训练（conda lerobot 环境，GPU）。`--dataset.root` **直接指向升版产出的数据集
目录**（给 root 后即按本地加载，`--dataset.repo_id` 仅作本地标识名）：

```bash
HF_LEROBOT_HOME=~/lerobot_home lerobot-train \
    --dataset.root=~/astral_data/lerobot_v3/pick_place \
    --dataset.repo_id=astral/pick_place \
    --policy.type=act --steps=100000 --batch_size=64 --job_name=astral_act
```

已用真实 v2.1 数据验证：产物与官方转换脚本逐文件一致、现代 `LeRobotDataset`
加载并逐帧解码成功、`lerobot-train` ACT 冒烟（2 步）真实跑通并落 checkpoint。
状态/动作语义与 v2.1 完全相同（`action` 绝对关节角 + `_ee_` 末端绝对）；
若想训 delta action，属**训练前数据变换**范畴，改数据处理而非本转换。

### 5c. 数据质量优化（采集 → 训练）

从"慢速遥操抖抖的 / 训练后真机固定卡点"到质量可控的完整过程。核心结论：**数据质量决定
训练质量**——采集链路（上游精度、实时防线）、转换（自检）、**训练数据节奏**（示范停顿
耦合）三层都要管；引擎参数只能平滑推理表现，治本在数据。

**症状画像（怎么发现问题）**

| 症状 | 根因 | 防线 / 修法 |
|---|---|---|
| 慢速平移"抖抖的"（遥操日志 VR 零速占比 36%、cmd 高频抖动 ~10mm/s） | 上游 `astral-tracking` 位姿序列化 F4/F3 → 0.1mm/0.001 量化阶梯，下游 EMA 滤不净 | 上游改 F7/F6（1e-7/1e-6，~float 原生精度），协议行结构不变、解析无需改 |
| 录制中相机只有 3fps 没人知道 | 抽头链路异常（限流器/编码器/GIL） | `low_fps_warning`：参考相机实率 < fps/2 → state JSON + 节点 WARN + web 红条 |
| 空录（忘 armed / teleop 没发布 / 抽头没开） | 源未就绪数值+图像全 0 | `EMPTY-REC` 空录告警（启动 ~2s 全 0 → 三档文案 + web 红条，段级判定） |
| "以为开了实际没录"（SAVING 期按 start 被吞） | 键盘/VR 无 disabled 视觉，非法态按键静默 | 吞指令计数 `ignored` 入 state JSON + web chip（pause/resume 原静默 no-op 也计数） |
| 坏段进训练集 | 短段/体检差段未被隔离，validate 默认只出报告 | `data_pipeline process-*` validate 默认 `--apply`（fail 段移 `quarantine/`，可逆），隔离后仍有 fail 即 `exit 1` 中止；`--no-quarantine` 仅关闭自动隔离，导出硬检查仍生效 |
| 训练后真机**固定卡点**（到目标前/夹取后/放置前/释放后"停一下"） | 训练数据本身走走停停（pick_place_merged 实测 **20.6% 帧速度 <0.008 rad/帧**、208 个慢速段）——ACT 忠实学进示范节奏 | 数据修复（见下） |

**防线体系（三层，各司其职）**

- **实时（采集时）**：`low_fps_warning` / 空录告警 / 吞指令留痕 / `session` 溯源（每段 meta 记
  目录，切目录不重启节点）
- **离线（转换前）**：`validate_data` F1-F6 fail / W1-W6 warn + 默认隔离 fail 段 + 隔离后中止
- **转换（ACT 训练数据）**：`convert_to_act.py` **两级自检**——结构级（stats mean/std 齐全 /
  相机同 shape / tasks 非空 / parquet+视频可读可解码，不过即退出）+ 深度级（`--check-python`
  用现代 lerobot 真装载 `LeRobotDataset` + ACT 预处理管线 + 逐帧解码）

**训练数据节奏修复（治本）**

模型复现的是示范节奏——示范停顿 → 真机卡点。一个脚本（`MORLA/scripts/`）：

| 脚本 | 处理层 | 方法 | 效果 |
|---|---|---|---|
| `repair_aligned.py` | aligned_data.h5 | 默认 natural + 意图保护：只删有 command 证据的摩擦停顿；无 command 保留。保护夹爪、腰、头、末端位姿及末尾 hold。command action 保留原值，next_state 按清洗后的下一帧重建；保留真实相机时间戳和来源索引。uniformize 为显式可选重定时，并检查速度上限 | 2026-10-10 全量 100 段 dry-run：21321→21313 帧；该数据几乎没有可安全删除的停顿，详见验证报告 |

修完 `data_pipeline process-act`（align 检测 aligned 存在即跳过）→ `act_train.sh` 重新训练 → 真机
对比卡点是否消失。`--speed-ref` 调速度/压缩比（默认臂维弧长均值=保留总时长不加速；想更快给
0.07，**别超 0.07** 否则跳变放大回归；更低 0.035~0.045 更贴近原速）。

## 6. 数据格式（raw）

```
episode{N:06d}/
├── robot_data.h5   /streams/{name}/values (N,D) f4 + timestamps (N,) f8
├── camera_data.h5  /{cam}/images (N,) vlen u8 JPEG + timestamps f8
└── meta.json       schema 快照 / task / session / events（armed 跃迁、pause）/ 流统计 / 丢弃计数
```

时间戳统一 PC 墙钟秒：`header.stamp` 有效（sec>0）优先，否则到达时刻。
所有话题同机同源同钟，跨流对齐误差 ≪ 1/fps。

aligned_data.h5：`observation/state (T,D)`、`action (T,D)`、`timestamps`、
`quality`（bit0=图像偏远 bit1=数值空洞）、`{cam}/{images,src_timestamps,src_offsets}`。

## 7. 测试

```bash
cd src/common/morla_data_collect/test
# 纯离线模块（无 ROS 依赖；在 VLA/openpi uv venv 跑，行数/命令细节见 CLAUDE.md「测试」）
uv run --project ../../../../../VLA/openpi pytest test_schema.py test_align.py \
    test_validate.py test_convert_lerobot.py test_convert_lerobot_v3.py \
    test_adversarial_configs.py -q -p no:anyio
# 节点类（需 ROS 环境，系统 3.10；rclpy 只在 /usr/bin/python3 有）
/usr/bin/python3 -m pytest test_node_guards.py test_collect_smoke.py \
    test_replay.py -q -p no:anyio
```

conda python3.13 无法 import rclpy，故节点测试用 `/usr/bin/python3`（系统 3.10）。

### 2026-10-10 采集与清洗约束

- 暂停后继续会保存当前 episode 并开启新 episode，避免跨暂停插值或重复补帧。任务文本在开始录制时冻结，录制中修改只影响下一段。
- 相机 tap 使用提交时的源时间戳：ROS 使用有效 header，webcam 使用 read 返回后的主机时间；编码延迟不改变它。raw 相机另外保存 arrival_timestamps。仍需真机测量相机曝光与机器人时钟的偏差。
- command 按过去最近值对齐，夹爪比例采用零阶保持；缺失配置相机、非有限/非递增时间戳、历史 episode 内暂停空洞直接报错。raw/meta 更改后已有 aligned 会拒绝复用，须 --force 重建。
- 末尾合成 hold 通过 quality_flags bit 4 单独标记。清洗后的训练时间可重定时；source_grid_timestamps、source_frame_index、source_alignment_offsets 用于追溯原始采样，不能把重定时后的 src_offsets 当作传感器同步误差。
- 导出要求每段 aligned 存在且 state/action/timestamps 有限、维数正确、schema 的 fps/cameras/action_source 一致。输出采用临时文件/目录完成后替换；中途失败保留已有产物。meta/quality 与 processing_manifest.json 保存质量与处理信息。
- 历史相机别名只能显式指定，例如 repair_aligned.py --camera-map video8=base,video0=left_wrist；不会自动猜测不同相机是否相同。

真实数据验证（输出目录必须不存在，原始 raw 不修改）：

```bash
bash MORLA/scripts/data_pipeline.sh verify-existing \
  --session astral_data/raw/pick_place_merged \
  --output astral_data/qa/new_run \
  --camera-map video8=base,video0=left_wrist \
  --check-python /home/robot/miniconda3/envs/lerobot/bin/python
```

本次结果与局限见 [修复验证报告](../../docs/data-pipeline-fixes-2026-10-10.md)。
