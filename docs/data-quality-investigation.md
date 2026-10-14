# 数采数据质量排障（采集 → 训练：从"当时知道坏了"到"卡点治本"）

> 2026-09-14 前后真机/离线排障，2026-09-22 整理归档。
> 给新会话/接手者：理解数采链路质量认知的**五层演进**（坑表是"症状→防护"，这里是"决策顺序 +
> 数据 + 教训"）。母本：`src/common/morla_data_collect/CLAUDE.md`「数据质量优化历程」+ `README.md` §5c
> + `astral_ws/CHANGELOG.md`（09-14~09-18）。
> 关联：训练后真机卡点的数据侧根因详见 [teleop-stick-slip-investigation.md](teleop-stick-slip-investigation.md)
> （摩擦伪影来源）；推理侧表现见 [act-inference-stutter-investigation.md](act-inference-stutter-investigation.md) 第 3 层。

## TL;DR（一分钟结论）

数据质量问题分五层，从采集到训练层层设防；**最贵的教训是"训练后真机固定卡点"的根因在数据**：

| 层 | 防线 | 一句话 |
|---|---|---|
| 1 上游精度 | Quest App 位姿量化 F4→F7 | 量化台阶在慢速段最显眼 |
| 2 实时防线 | low_fps / 空录告警 / 吞指令留痕 | **数据坏了要"当时"知道** |
| 3 控制面与目录 | VR 手柄控制录制、session 运行期切换 | 让"录到好数据"顺手 |
| 4 转换自检 | validate 默认隔离 + convert_to_act 两级自检 | 堵坏数据进训练集 |
| 5 训练数据节奏 | `repair_aligned.py` 选帧零错位 | **模型是示范的镜子，治本在数据** |

## 第 1 层：采集链路精度（上游量化阶梯）

慢速遥操"抖抖的"——上游 `astral-tracking` 位姿序列化 F4/F3（0.1mm/0.001）量化成台阶，
下游 EMA 滤不净（遥操日志实测 VR 零速占比 36%、cmd 高频抖动 ~10mm/s）。
**修法**：上游改 F7/F6（1e-7/1e-6 ≈ float 原生精度），协议行结构不变、`morla_input`
解析零改动。**教训**：采集质量的最上游是精度；量化台阶在慢速段最显眼
（详见 [teleop-stick-slip-investigation.md](teleop-stick-slip-investigation.md)）。

## 第 2 层：实时防线（采集时就知道坏了）

之前"改了数据才发现问题"。逐步补齐采集时可见的信号：

- **`low_fps_warning`**：录制中参考相机实率 < dataset_fps/2 → state JSON + 5s 节流 WARN + web 红条
  （管"相机链路异常"——相机 3.3fps 无人察觉的教训）；
- **空录告警（EMPTY-REC）**：启动 ~2s 数值+图像全 0 → 三档文案（未收到任何样本 / teleop 未发布 /
  抽头未发布）——管 `low_fps` 管不到的"源未就绪全 0"；
- **吞指令留痕（`ignored`）**：非法态被忽略的按键计数进 state JSON——键盘/VR 无 disabled 视觉，
  "以为开了实际没录"；
- **`session` 溯源**：每段 meta 记目录（换目录不重启节点）。

**原则**：数据坏了要"当时"知道，不是离线复盘才知道。

## 第 3 层：控制面与目录（单人采集不打断节奏）

VR 手柄控制录制（A=start / B=stop&save / 摇杆=discard，上升沿+状态门控，纯决策模块离线单测）、
session 运行期切换（换目录不重启节点，仅 IDLE、目录名安全校验）、三目录约定（raw/pi/act 各进各的）。
**目的**：让"录到好数据"这件事本身顺手、不易出错。

## 第 4 层：转换自检（坏段隔离 + ACT 两级自检）

- `validate` 默认隔离 fail 段到 `quarantine/`（移动不删除可逆），隔离后仍有 fail 即 `exit 1` 中止转换；
- `convert_to_act.py` **结构级自检**（stats/相机同 shape/tasks/可读可解码，不过即退出）+ **深度级**
  （`--check-python` 现代 lerobot 真装载 + 逐帧解码）。
- 同目录重转防污染：重转前清旧 `data/`+`videos/`；v3 升版以 `meta/episodes.jsonl` 为权威 episode
  清单，文件集与清单不一致即报错。

**目的**：堵坏数据进训练集的通路，宁可中止不静默。

## 第 5 层：训练数据节奏（治本，固定卡点的根因）

采集质量修好后，真机推理仍**固定卡点**（到目标前/夹取后/放置前/释放后）。定位到：**训练数据
本身走走停停**——pick_place_merged 实测 **20.6% 帧速度 <0.008 rad/帧、208 个慢速段遍布**。
ACT 忠实学进示范节奏，真机在对应任务状态复现停顿（推理侧 test5 慢速段与训练数据阶段一一对应）。

### 意图 vs 摩擦量化（先诊断再修）

`src/common/morla_data_collect/morla_data_collect/pipeline/quantify_cmd_state.py` 读 raw `*_cmd` + `*_state`（互相关对齐后）把每个停顿分型：
**意图型**（cmd 也停=操作者/任务真实停顿，训练该保留）vs **摩擦型**（cmd 平滑移动、state 平段
突跳=驱动层摩擦伪影，训练该去掉）。实测（pick_place_merged 100 episode，`--align` 对齐后）：

- state 平段中位 **34.9%** vs cmd 平段 **9.3%** → **摩擦份额 ~25.6pp**；
- 1316 个停顿分型 = 意图 **13%** / 摩擦 **38%** / 混合 **49%**；
- `--align` 实测 cmd 领先 state **~120ms**（物理跟踪滞后）——不对齐时 state 停顿窗口看的 cmd
  错位一个跟踪滞后，短意图停顿会被错分。

### repair_aligned.py 演进（关键设计决策：选帧 vs 插值）

| 版本 | 方法 | 效果 | 问题 |
|---|---|---|---|
| `compress_pauses.py`（raw 删帧，已删） | 删停顿帧 | 帧 -34.9%，但 | 相邻帧位移变大、时间戳 gap（W2×100） |
| resample（插值）v1 | 弧长插值 | 帧 -50.8%，但 | 关节插值 vs 图像离散帧 **错位 p90 6° / max 12.7°**（argmin 修复后 p90 2.7°） |
| v2 选帧 | 弧长均匀**选帧** | -50.9% | 无错位；但速度被 p90 提速 4 倍、夹爪猛合 |
| **v3 双模式** | `natural`（保时序摩擦移除，默认）/-2.1% 帧、速度逐位不变、夹爪全保、平段→0.5%；`uniformize`（匀速化）/-13% | — | `--keep-intent` 保留意图停顿时长 |

**核心设计决策**：repair 从"插值"改"选帧"——关节插值必然造成关节 vs 图像错位（图像只能取
离散帧），**选帧（取原始帧子集）零错位**（`verify_aligned.py` 断言 state 每行精确等于原始某帧、
图像 bytes ∈ 原始帧集合）。**教训**：模型是示范的镜子——要流畅的动作，先要有流畅的示范（或修复
数据）；"复现操作者"用 natural（保时序），"更快执行"才用 uniformize，别混。

## 采集/转换层的坑（症状 → 根因 → 防护）

| 症状 | 根因 | 防护/参数 |
|---|---|---|
| 两个采集节点各录一份（双开事故） | `/data_collect/control` 全局控制面，谁订阅谁执行；残留节点+再启动 | 三层：domain 排他锁（`/tmp/morla_data_collect_domain{N}.lock`）+ episode 原子 mkdir 占号 + web 发布者计数红条 |
| 改了 yaml 却采出旧 schema | launch 自身默认值静默盖 yaml | launch 默认空串、显式传参才覆盖；schema 冻结进每段 meta.json |
| 转换 exit 137（OOM），管道里还"只出 1 段、退出码 0" | SVT-AV1 默认按核数并行+深 lookahead；np.stack 统计堆 620MB/相机 | `svtav1-params lp=2:lookahead=16`（不识别退化 h264）+ `ImageStatsAccumulator` 流式；**该命令勿接 `\| tail`，要看退出码**；峰值 RSS 6.3→1.14GB |
| openpi 加载数据集崩 "Could not infer dtype of dict" | parquet 多写了视频 struct 列 | 转换器删列；测试断言**不得**出现 `observation.images.*` 列 |
| validate F4 全灭：cmd 戳非递增 | teleop 用 VR 输入戳打 joint_commands，IK 比 VR 快时同戳复用 | 采集端 `*_cmd` 一律到达时刻；state 流仍用 header.stamp（两种时钟语义不同） |
| 对齐后帧数 ≠ fps×时长 | 对齐网格含边界帧/保持帧 | 不硬编码帧数，从 parquet 读真实段长 |
| 改配置后同目录重转混入旧布局脏行 | 转换器只截断 meta jsonl 不删旧文件 | 重转清 data/videos + v3 以 episode 清单为权威 |

## 工具速查

| 工具 | 用途 |
|---|---|
| `src/common/morla_data_collect/morla_data_collect/pipeline/quantify_cmd_state.py` | 采集数据"意图 vs 摩擦"量化（停顿分型 + 平段/粘滑/突跳统计），`--self-test` 合成自测 |
| `src/common/morla_data_collect/morla_data_collect/pipeline/repair_aligned.py` | aligned 层数据节奏修复（`--mode natural\|uniformize`、`--keep-intent`、`--dry-run`；输出新目录绝不在原地改） |
| `src/common/morla_data_collect/morla_data_collect/pipeline/verify_aligned.py` | 修复输出对抗验证（state=原始帧/图像同源/时间戳单调/action=next-state，零错位） |
| `scripts/data_pipeline.sh process-openpi` / `data_pipeline process-act` | 同一 raw 各进各的输出目录（openpi v2.1 / ACT v3+两级自检） |
| `src/common/morla_data_collect/morla_data_collect/pipeline/migrate_camera_labels.py` | 旧数据相机键 videoN→语义名迁移（注意：不迁 meta schema.cameras，见遗留） |

## 遗留 / 注意

- `migrate_camera_labels.py` 未同步 meta.json 的 `schema.cameras`——存量 raw 源段仍有
  "文件=base/left_wrist、meta=video8/video0"不一致态（修复输出已自洽）；
- 夹爪 ratio 是指令回显（无真实反馈），物理闭合滞后数据层无法改；
- `command` action_source 路径近期没有真实数据验证（当前全用 next_state）。
