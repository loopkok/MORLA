# CLAUDE.md — morla_data_collect 数采工程

为 OpenPI pi0.5 等 VLA 模型采集训练数据的 ROS 2 包。与遥操作链路并行运行，
四段式离线管线：**采集（raw HDF5）→ 对齐（fps 网格）→ 校验清洗 → LeRobot v2.1 导出**
（另有 **v2.1 → v3.0 升版**，供现代 lerobot 的 ACT 等策略直接训练），
外加 Rerun 回放。本文件面向新会话/新接手者：先读这里，再进 `README.md`（操作手册/API 契约）
和 `astral_ws/CHANGELOG.md`（按天演进史，每个问题都有 症状→根因→修法 三段）。

## 当前状态（2026-09-03）

- **当前采集配置 = 单左臂 + 左夹爪**（`config/data_collect.yaml`，双臂注释保留），
  8 维 state（7 关节 + 闭合比），相机 `["base", "left_wrist"]`（不录 right_wrist；openpi camera_map 本就只用 base+left_wrist）；
- 默认 `action_source: next_state`（`action[t] = state[t+1]`，绝对关节角）——
  openpi 训练侧再转 delta（见下游约定，别在采集侧改）；
- 图像**转换期 letterbox 到 224×224**（等比+黑边，复刻 openpi `resize_with_pad` 几何），
  `--image-size 0` 保留原分辨率；
- 上游 `quest3_video_streamer` 已有 `label_aliases`（按硬件指纹把 videoN 改写为
  稳定名）——`cameras` 里的 label 对应稳定名，换口/重插不再需要改本包配置。
- session 目录可运行期切换：web 卡片「设定目录」或 /data_collect/session 话题
  （仅 IDLE；本段 meta.json 记 session 溯源）；重启节点回落 yaml/launch 初始值。
- 三目录约定：raw 采集落 `~/astral_data/raw/{session}`（yaml save_root 默认已指向），
  openpi 转换 → `~/astral_data/pi/{session}`，ACT 转换 → `~/astral_data/act/{session}`。

## 数采完整流程（episode 循环，与遥操联动）

1. 启动 web 监控 → 系统 tab 预设启动遥操（当前配置：**「Left arm + left gripper (no right arm)」**）
   → 数采卡片「启动节点」拉数采泳道 → 设 session 目录 + 下段任务文本 → 点「工作位」。
2. 戴 Quest3 → 左 **grip** 开始遥操（重标定）→ 右 **A** 开始录制 → 执行任务 → 右 **B** 停止保存
   → 左 **X**（或卡片「段间回位」）停止跟随+**直接**回工作位（不经途径点）→ 摆物品 → 左 grip 开始下一段。
3. 完整命令/键位表见 `morla_bringup/README.md`「完整遥操启动 / 键位速查」。
   **X=直达**（`/teleop/init_direct`），系统 tab「工作位」=途经点（`/teleop/init`）。

**与数据的关系**：X 段间回位发生在 IDLE（B 已保存后），`_accepting()` 门控把回位期间所有流丢弃，
不入任何段；段前 `dq.clear()` 保证干净起点；X 在 RECORDING/PAUSED/SAVING 被闸门拦截。
**顺序要求**：先 grip 再 A（否则新段 armed 覆盖率 <50% → validate W5）；B 后等状态回 IDLE 再按 X
（SAVING 期 X 被拦）；回位走完再 grip（homing 中 `/teleop/start` 被臂节点拒）。

## 文件地图

离线处理实现统一在 `morla_data_collect/pipeline/`；文件/命令映射、完整流程见
`morla_data_collect/pipeline/README.md`。统一源码入口为 `MORLA/scripts/data_pipeline.sh`，
安装入口为 `data_pipeline`；旧 Python 模块、旧 scripts/ 数据脚本及旧 ROS 离线入口已删除。

```text
morla_data_collect/
├── schema.py                         # 采集/处理共同的布局真源
├── data_collect_node.py, data_writer.py # 在线采集与 raw 写盘
├── keyboard_controller.py, vr_collect_* # 采集控制
└── pipeline/                         # 全部离线实现
    ├── cli.py, __main__.py, process.sh # 统一分发和一键编排
    ├── align_data.py, validate_data.py
    ├── quantify_cmd_state.py, repair_aligned.py
    ├── convert_to_lerobot.py, convert_to_lerobot_v3.py, convert_to_act.py
    ├── replay_rerun.py
    ├── verify_aligned.py, verify_existing_data_pipeline.py
    ├── migrate_camera_labels.py      # 历史兼容工具
    └── artifacts.py, README.md
```

新增离线功能放在 pipeline/ 并注册 cli.COMMANDS，禁止在 scripts/ 另写业务实现。
测试仍统一在包的 test/；ROS 配置和 launch 位置不变。

## 架构不变量（改动前先确认不破坏这些）

1. **schema.py 是唯一布局真源**。state/action 维度、块顺序、流名、相机 key
   全部从 `CollectSchema` 推导；openpi 侧 `LeRobotAstralDataConfig.create()`
   运行时读数据集 `meta/info.json` 的 `names` 反推 delta 掩码（含 `_ee_` 的维
   保持绝对，其余转 delta）。**加右臂/换 wuji/加腰头都不用改任何代码**：
   改 `data_collect.yaml` → 重采 → 重转 → 重算 norm stats。
2. **硬上限：总 state 维 ≤ 32**（pi05_base action_dim）。当前单臂+夹爪=8 ✓；
   双臂=16 ✓；单臂+wuji=27/31 ✓；双臂+双 wuji=38 ✗（openpi 侧 create() 会
   明确 raise，不会静默错训——别绕过这个检查）。
3. **`*_cmd` 流时间戳 = 到达时刻（time.time()），state 流 = header.stamp**。
   根因：teleop 把上游 VR 戳打进 joint_commands，IK 60Hz > VR 30Hz 时同戳重复，
   F4 全灭。改时间戳语义前先看 `test_cmd_stream_uses_arrival_time`。
4. **对齐参考相机 = cameras[0]**：episode 区间由其首尾帧界定。换参考相机
   等于换 episode 边界定义，已采数据不可比。
5. **parquet 只含非视频列**（v2.1 规范）：视频帧由读取侧按 timestamp +
   meta 的 video_path 模板解析。struct{path,timestamp} 视频列会让 openpi
   锁定的 lerobot 0.1.0 在 `torch.tensor(dict)` 处崩（已踩过，有回归测试）。
6. **内存红线：转换在小内存机（15GB）跑**。SVT-AV1 默认 `lp=2:lookahead=16`
   （深 lookahead 会吃数 GB）；图像统计必须走 `ImageStatsAccumulator` 流式
   累加（np.stack 100 帧 1080p ≈620MB/相机）。这两个优化是 exit 137 换来的，
   别"简化"回去。**核数多的机器（20 核）可在内存充裕时 `ASTRAL_AV1_LP=8` 放开
   AV1 并行（~0.6GB/lp 峰值；本机当前 IDE 占 10GB 时 lp≤4 才安全）**。
7. **QoS 与门控**：抽头话题仅在有订阅者时编码（无订阅零开销）；采集开关只
   管"录"，不碰 streamer 的推送门控（"看"与"录"正交，别再耦合）。

## 已解决的坑（症状 → 根因 → 现有防护）

| 症状 | 根因 | 防护 |
|---|---|---|
| 两个采集节点各录一份（双开事故） | `/data_collect/control` 全局控制面，谁订阅谁执行；老版本 launch 默认值还盖 yaml | 三层：domain 级排他锁（`/tmp/morla_data_collect_domain{N}.lock`，**不同 session 也不许双开**）+ episode 原子 mkdir 占号（对无锁残留也互斥）+ web 后端发布者计数红条 |
| 改了 yaml 却采出旧 schema | launch 自身默认值静默盖 yaml | launch 默认空串、显式传参才覆盖；schema 冻结进每段 meta.json 可事后核对 |
| 录制中相机只有 3fps 没人知道 | 抽头链路异常（限流器/编码器/GIL 问题史） | low_fps_warning：参考相机实率 < fps/2 时 state JSON 标记 + 节点 WARN（5s 节流）+ web 红条 |
| 转换 exit 137（OOM）、管道里还"只出 1 段、退出码 0" | SVT-AV1 默认按核数并行+深 lookahead；np.stack 统计堆 620MB/相机 | `svtav1-params lp=2:lookahead=16`（不识别再退化 h264）；流式 ImageStatsAccumulator（输出与堆叠数值等价，有回归）；**该命令勿接 `\| tail`，要看退出码** |
| openpi 加载数据集崩 "Could not infer dtype of dict" | parquet 多写了视频 struct 列（旧版 lerobot 崩） | 转换器删列；测试断言 parquet **不得**出现 `observation.images.*` 列 |
| validate F4 全灭：cmd 戳非递增 | teleop 用 VR 输入戳打 joint_commands，IK 比 VR 快时同戳复用 | 采集端 `*_cmd` 一律到达时刻；`test_state_stream_keeps_header_stamp` 保 state 侧不回退 |
| 对齐后帧数 ≠ fps×时长 | 对齐网格含边界帧/保持帧 | **不要硬编码帧数算 episode 偏移**，从 parquet 读真实段长（测试里踩过） |
| 改采集配置后同目录重转，v3 升版混入旧布局脏行 | `convert_session` 只截断 meta jsonl、不删旧 `data/videos`；v3 升版按目录遍历文件 → 旧段（另一 schema 的维数/内容）被当新段并进 parquet | ①`convert_session` 重跑前清空 `data/`+`videos/`（输出目录=数据集专属，重跑即替换）；②`convert_to_lerobot_v3` 以 `meta/episodes.jsonl` 为权威，数据/视频文件集与清单不符即报错（列出多余/缺失下标）。`test_reconvert_same_output_cleans_stale` / `test_v3_rejects_dirty_source` 双回归 |

## 数据质量优化历程（2026-09-14 前后，按时间顺序）

> 给新会话/接手者：理解数采链路质量认知的演进顺序（坑表是"症状→防护"，这里是"决策顺序 +
> 数据 + 教训"）。完整逐日记录见 `astral_ws/CHANGELOG.md`。
> **独立整理版**（五层防线 + 全部数值 + 工具表）：`astral_ws/docs/data-quality-investigation.md`。

**第 1 层：采集链路精度（上游量化阶梯）**
慢速遥操"抖抖的"——上游 `astral-tracking` 位姿序列化 F4/F3（0.1mm/0.001）量化成台阶，下游
EMA 滤不净（遥操日志实测 VR 零速占比 36%、cmd 高频抖动 ~10mm/s）。**修法**：上游改 F7/F6
（1e-7/1e-6 ≈ float 原生精度），协议行结构不变、`morla_input` 解析零改动。**教训**：
采集质量的最上游是精度，量化台阶在慢速段最显眼。

**第 2 层：实时防线（采集时就知道坏了）**
之前"改了数据才发现问题"。逐步补齐采集时可见的信号：`low_fps_warning`（参考相机半速 → 红条）、
**空录告警**（启动 ~2s 数值+图像全 0 → `EMPTY-REC` 三档文案，管"源未就绪全 0"这种 low_fps 管
不到的）、**吞指令留痕**（非法态被忽略的按键计数进 state JSON——键盘/VR 无 disabled 视觉，
"以为开了实际没录"）、`session` 溯源（每段 meta 记目录）。**原则**：数据坏了要"当时"知道，
不是离线复盘才知道。

**第 3 层：控制面与目录（单人采集不打断节奏）**
VR 手柄控制录制（A=start/B=stop&save/摇杆=discard，上升沿+状态门控，纯决策模块离线单测）、
session 运行期切换（换目录不重启节点，仅 IDLE、目录名安全校验）、三目录约定（raw/pi/act 各进
各的）+ 剔 right_wrist（openpi camera_map 本就只用 base+left_wrist）。**目的**：让"录到好数据"这件事
本身顺手、不易出错。

**第 4 层：转换自检（坏段隔离 + ACT 两级自检）**
`validate` 默认隔离 fail 段到 `quarantine/`（移动不删除可逆），隔离后仍有 fail 即中止转换；
`convert_to_act.py` 结构级自检（stats/相机同 shape/tasks/可读可解码，不过即退出）+ 深度级
（`--check-python` 现代 lerobot 真装载）。**目的**：堵坏数据进训练集的通路，宁可中止不静默。

**第 5 层：训练数据节奏（治本，卡点根因）**
采集质量修好后，真机推理仍**固定卡点**（到目标前/夹取后/放置前/释放后）。定位到：**训练数据
本身走走停停**——ACT 忠实学进示范节奏，真机在对应任务状态复现停顿。**修法**：
`pipeline/repair_aligned.py`（aligned 层，**选帧**零错位）。**关键设计决策**：repair 从"插值"
改"选帧"——关节插值必然造成关节 vs 图像错位（图像只能取离散帧），**选帧（取原始帧子集）
零错位**。**09-17 重写为双模式**：`--mode natural`（默认）=**保时序摩擦移除**——只删摩擦型
停顿帧（全臂+夹爪都停、非意图、连续≥2帧），其余帧 1:1 保留、**速度=自然速度**（不再统一
速度：旧均匀化把慢速段 p10 提速 4 倍、夹爪被压没 → 回放"整体加快/猛合猛放"）；夹爪过渡
±5 帧保护；`--keep-intent` 保留有意停顿时长。`--mode uniformize`=弧长匀速化（供"更快执行"
训练，夹爪/意图同样保护）。实测 natural：-2.1% 帧、臂速度逐位不变、夹爪时序全保、平段
→0.5%、零错位对抗全过。**教训**：模型是示范的镜子——要流畅的动作，先要有流畅的示范（或
修复数据）；"复现操作者"用保时序，"更快执行"才用匀速化，别混。

## 下游约定（openpi v2.1 / lerobot ACT v3.0，改采集必须连带核对）

- 两条下游：**openpi（pinned lerobot 0.1.0）直接消费 v2.1**；**ACT 走专属
  `convert_to_act.py`**（raw session → 官方 v3 布局 + 内置两级自检，推荐）——
  也可经通用 `convert_to_lerobot_v3.py` 把 v2.1 升版 v3.0 给其它现代 lerobot
  策略（state/action 数值语义原样，只动布局；源只读）。ACT 训练侧 `/255` +
  数据集 stats.json 的 mean/std 归一化、无 resize（224×224 同 shape 由转换期
  letterbox 保证）。
- 数据集位置：`${HF_LEROBOT_HOME:-~/.cache/huggingface/lerobot}/astral/morla_bringup`
  （软链即可）。pinned lerobot 0.1.0 只认 `HF_LEROBOT_HOME`，设旧名
  `LEROBOT_HOME` 会直接 raise。
- 相机槽位映射在 openpi `LeRobotAstralDataConfig.camera_map`（默认
  base→base_0_rgb，left_wrist→left_wrist_0_rgb，右腕槽零填充 mask=False）。
  改 cameras 列表/换 label 名 → openpi camera_map 一行联动。
- 空 task 段靠 openpi `default_prompt` 兜底（仅冒烟可用）；正式训练的数据
  **每段必须填 task**（web 卡片或热键 t）。
- 一键脚本（openpi 与 ACT 从同一 raw 目录各取所需、输出独立）：
  `bash MORLA/scripts/data_pipeline.sh process-openpi <raw> <openpi输出>`（→ v2.1）与
  `bash MORLA/scripts/data_pipeline.sh process-act <raw> <act输出>`（→ v3 + 两级自检）；
  `openpi_train.sh`（norm stats→训练）。建议输出命名 `<session>_openpi` / `<session>_act`。
- 换 schema 后的动作序列：重转 → **重算 norm stats**
  （`uv run scripts/compute_norm_stats.py --config-name pi05_astral_lora`）→
  再训练。norm stats 不含图像（letterbox 与否都**不用**重算）。

## 测试（pytest，改任何模块后必须全绿）

```bash
cd astral_ws/src/common/morla_data_collect/test
# 纯离线模块（无 ROS 依赖；可在 openpi uv venv 跑）:
uv run --project ../../../../../VLA/openpi pytest test_schema.py test_align.py \
    test_validate.py test_convert_lerobot.py test_convert_lerobot_v3.py \
    test_adversarial_configs.py \
    -q -p no:anyio
# 对抗配置 8 例依赖 openpi（HF_LEROBOT_HOME 自动隔离到临时目录，不碰真实数据）
# 节点类（需 ROS 环境，系统 3.10）:
/usr/bin/python3 -m pytest test_node_guards.py test_collect_smoke.py \
    test_replay.py -q -p no:anyio
```

- **python 环境分界**：rclpy 只在系统 `/usr/bin/python3`（3.10）有；miniconda
  python 3.13 没有。离线模块用 openpi uv venv（pyarrow/cv2/av/h5py 齐）。
- 测试风格：pytest 函数式 + `conftest.write_raw_episode()` 造合成 session
  （支持注入 NaN/空洞/丢流缺陷）；改 schema/对齐语义先在这里加失败用例。
- `test_adversarial_configs.py` 是**换机器人配置的黄金回归**：双臂/灵巧手/
  腰头/15fps/NaN 隔离/norm stats 全链 + 三个负例（>32 维、错相机名、缺 meta
  必须报错）。动 schema↔openpi 接口必跑。

## 修改流程约定（从历史 commit 沉淀）

1. **先诊断后动手**：bug 先在测试里复现（write_raw_episode 注入缺陷），修完
   转正为回归；对抗测试模式 = 每种配置变更跑完整链条 + 在 openpi 侧逐维验证
   语义（delta 掩码逐位、相机颜色指纹、截断维），不是只看 shape。
2. **schema 变更=一次写清三件套**：data_collect.yaml 注释 + README 参数表 +
   维度速查行（"单臂 gripper=8；单臂 wuji+腰+头=31"这种）。
3. **数值给实测**：CHANGELOG 里写具体数字（"峰值 RSS 6.3GB→1.14GB"、
   "283MB→20MB -93%"），不写"已优化"了事。
4. **每次实质改动更新** `astral_ws/CHANGELOG.md`（当日日期加粗标题——改动——
   动机——数值——验证结果）+ 对应 README 段落。症状描述写用户可感知表象。
5. 用户重视**端到端可验证**：每个管线阶段都要有金标准检查（如 openpi 环境
   实际加载 LeRobotDataset 逐字段比对），而不是"转换器说转换完了"。

## 遗留事项 / 下一步候选

- `meta.json` 的 `streams[].count` 恒为 100（占位值），未写真实样本数——
  校验只依赖实际数据，不影响功能，但字段有误导性；
- 双臂+双 wuji（38 维）超 pi05_base 上限，真要训需动作降维或改模型投影层
  （openpi 侧 create() 已挡住，无静默风险）；
- `command` action_source 路径近期没有真实数据验证（当前全用 next_state）；
- web 卡片的任务文本输入与热键 t 并存，任务名拼写无校验（空 task 靠 openpi
  default_prompt 兜底，正式数据靠自觉）。
