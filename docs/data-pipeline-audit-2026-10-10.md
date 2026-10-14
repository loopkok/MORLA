# astral_ws 数据采集、处理与清洗审查

日期：2026-10-10。范围：morla_data_collect、quest3_video_streamer 采集抽头、repair_aligned、OpenPI/ACT 处理入口，以及 sdk/astral_data/raw 的现有数据。此次只审查、复现及生成报告，没有修改业务代码或采集数据。

## 结论

原始采集与 LeRobot 导出已有扎实基础：schema 驱动布局、独立写盘线程、缓冲溢出计数、单例锁、原子段号、失败段隔离、流式图像统计、v2.1/v3 分离。这些部分适合继续保留。

清洗尚不能作为可靠的训练准入门禁。优先修复 repair 的溯源丢失、armed 状态统计、最终产物验证、暂停时间处理及采集时间戳。引入复杂学习式筛选应排在这些问题之后。

## 验证

- 离线回归：70 passed，覆盖 schema、align、validate、v2.1/v3、ACT、对抗配置。
- source /opt/ros/humble/setup.bash 后节点/VR/回放相关回归：37 passed；1 条 rclpy Executor 析构 warning。
- repair --self-test：12 个检查全部通过。这个测试直接调用重采样函数，未覆盖 repair_episode 的 HDF5 属性与相机字段读取。
- 临时合成复现：/tmp/astral-data-audit/reproduce.py；结果：reproductions.json。
- 现有数据只读检查：/tmp/astral-data-audit/inspect_existing.py；结果：existing_data.json。
- 没有连接真机进行新采集，没有执行训练或完整解码所有真实视频。以下实测数据为结构、数值和时间元数据检查；任务是否完成、物体是否滑落等尚未人工评审。

## 优先修复的问题

### 1. P1：repair 丢失时间溯源和文件属性，已影响现有数据

位置：scripts/repair_aligned.py:351、352、394；replay_rerun.py:48。

读取代码在 f[c] 这个 camera group 内查询 c/src_offsets、c/src_timestamps，正确的组内相对名称应为 src_offsets、src_timestamps。现有数据通常因此走 zeros 分支。repair 新建 aligned 文件时还没有复制 fps、schema、state_names 等属性。

三个 repaired 会话各 100 段，300 段全部缺失 aligned 属性且时间字段不自洽。抽查 repaired_v3/episode000000 两路相机 src_timestamps 为零。回放器直接访问 f.attrs['fps']，这些产物无法按原接口正常回放。图像与 state 的同帧选取本身不因此失效，但无法再用这些字段验证其原始时间关系。

建议：修正组内路径；保留文件属性；记录 source_frame_index、source_grid_timestamp、source_camera_timestamp、training_timestamp；原始对齐误差和重定时后的时间差分别命名；增加文件级端到端测试。

### 2. P1：armed/disarm 被当成两个独立永久状态，覆盖率失真

位置：data_collect_node.py:351、519；validate_data.py 的 _armed_coverage。

事件回调各自保存 armed、disarmed 的布尔值，并忽略重复同值事件。一次 disarm=True 后重新 armed=True，不会清除旧 disarmed=True；段开始可能把两者同时写入快照，覆盖率计算最后又判为未 armed。重复的下一次 disarm=True 还可能被当成“无跃迁”忽略。

现有 merged 100 段的报告全部为 W5/armed 0%。抽查 episode000000 开始快照确实同时含 teleop_armed=True 和 teleop_disarmed=True。因此这个数字不能直接证明操作者没有使能遥操。

建议：维护单一有效 armed 状态；arm/disarm 是更新该状态的输入事件；每段冻结一个初始有效状态。测试至少包含 disarm→arm→start→disarm→arm 和重复 disarm。

### 3. P1：校验只覆盖部分 raw/state，没有验证最终训练契约

位置：validate_data.py:83、171、225；convert_to_act.py:227。

实测：aligned action[10,0]=NaN 时 validate 返回 pass；raw timestamps[10]=NaN 时也返回 pass；缺失 aligned 文件仍可通过 raw 校验。

原因：时间戳没有 finite 检查；aligned 检查只读取 state、不读取 action；没有要求 aligned 一定存在，也没有完整检查各数组长度、维度、时间网格、相机集合及质量位。raw 维度不匹配仅记 warn。align_session 捕获单段异常后仍继续，CLI 没有失败退出；转换器只枚举已经存在 aligned 的段，因此对齐失败段可能被漏导出。action NaN 当前会在转换器被跳过，并非保证进入训练，但校验报告和实际导出清单不一致。

直接调用 convert_to_act --session 也没有执行 validate；shell 包装器才提供隔离步骤。

建议：区分 raw 检查与 training-ready 检查；所有入口复用同一个强制准入函数；最终 action/state/timestamp/shape/camera/frame-count 一律硬校验；输出包含成功、隔离、失败、跳过原因的完整 manifest；遇到未解决错误非零退出。

### 4. P1：暂停区间被对齐成训练补帧，严重空洞仅为警告

位置：data_collect_node.py 的 pause/resume；align_data.py:117；validate_data.py:99。

采集暂停后不接收样本，但恢复后沿用墙钟。align 根据首尾相机时刻建立连续网格，未使用 pause 事件，于是用最近样本填充暂停区间。

合成实测：3 秒采集、中间暂停 5 秒，对齐得到 250 帧（含保持帧），144 帧 FLAG_STATE_GAP；validate 只返回 warn，默认隔离只处理 fail。

建议：暂停时切段；或按活动区间对齐并显式标记重定时，禁止制造跨暂停的 next_state 动作。严重同步空洞应按持续时间/占比/关键操作区间升级为 fail 或 needs_review，未经复核不可训练。

### 5. P1：相机时间戳记录编码完成时间，无法证明物理采集同步

位置：quest3_video_streamer/collect_tap.py:95、196；webcam_source.py 的 collect_hook；ros_source.py 的 collect_hook。

抽头队列仅传 frame/is_rgb，编码线程完成 JPEG 后才取 ROS now 作为 header.stamp。采集时刻和源 ROS stamp 没有被传递保留。排队与编码延迟会进入图像时间戳，不同相机的延迟还可能不同；最近邻 offset 很小并不能证明曝光与关节采样同步。机器人驱动也主要采用读取后发布时间。

建议：尽可能保留设备采样时间；在 capture/read 返回时记录 host monotonic/wall time；另存 source、receive、encode、publish 时间与 clock_domain。先测编码与同步偏差，再决定是否需要时钟校准。真实错位幅度尚未量化。

### 6. P1：repair 无条件重建 next_state action，command 契约被悄悄改写

位置：repair_aligned.py:224、329；align_data.py:155。

合成 command episode 修复后 action 变为下一帧 state，而复制的 meta 仍写 action_source=command。默认 next_state 数据不受这个特定问题影响。

此外 command 模式的腰/头分支实际采样当前 state，而文档写的是 next_state；command 采样不检查独立的时间空洞。

建议：repair 明确支持的 action_source；command 保留原 action 的选帧映射，或拒绝处理，禁止无声明改语义。统一腰/头语义与文档；为每个动作块记录来源、单位、采样方法及时间偏差。

## 进一步值得优化的问题

### 7. P2：停顿删除的语义保护不足

natural 默认未开启 keep-intent，因此“臂静止、末端静止”即可能被删除，不能仅凭此证明是静摩擦。接触等待、物体稳定、视觉确认可能都是必要停顿。腰/头运动不进入 arm/ee 判据，也可能被删。双臂 keep-intent 只读取第一侧 cmd，却与多臂 state 汇总比较。

选帧保持单帧 state/image 对应关系，但删除帧并改为 1/fps 时间网格仍改变时间过程，低速累积运动在删除边界可能变成更大步长。“所有速度保持原样”不应成为无条件保证。

建议：默认只报告候选停顿；明确允许删的区间；保护任务阶段、末端接触、腰头运动、最终保持段；删除后检查逐关节速度/加速度、action chunk 边界；做原始/筛段/删帧三组闭环评估。

### 8. P2：跨 episode schema 合并校验不足

位置：convert_to_lerobot.py:428。

只比较 state_blocks，未强制比较 fps、camera 集合/顺序、action_source、单位；输出统一采用首段 fps。因此布局相同但帧率或动作语义不同的会话可能混入同一数据集。repair --fps 默认 30，也未同步复制的 meta/schema。

建议：冻结并核对完整训练契约；允许不同的 task，但 fps/action_source/units/camera 需明确规范化或分数据集输出。

### 9. P2：JPEG 损坏会被静默替换，质量信息在导出时丢失

位置：validate_data.py 的三帧抽检；convert_to_lerobot.py:489。

只抽首/中/尾 JPEG；导出遇到坏帧复用 last，没有坏帧掩码或替换计数；quality 位也没有成为训练过滤条件。视频最终可解码不代表原始观察有效。

建议：在编码时统计全部解码错误、重复源帧、最长冻结、黑帧/曝光/模糊；输出报告；对持续故障设置准入阈值。不要将正常任务静止等同于相机冻结。

### 10. P2：ACT “深度自检”覆盖低于文档描述

位置：convert_to_act.py:141、173、178。

结构检查只解码每相机第一个视频的首帧；深度脚本只取 ds[0]，不是逐帧检查。LeRobotDataset(root) 把路径传给 repo_id，与当前本地 LeRobotDataset(repo_id, root=...) 接口不一致。这项为代码核对，未在现代训练环境运行深度脚本。

建议：显式 repo_id/root；至少抽每段首/中/尾、v3 视频拼接边界、最后一帧；检查张量内容和 timestamps，必要时提供完整视频解码模式。

### 11. P2：任务文本未在段开始冻结

位置：data_collect_node.py:368、571。

_on_task 更新“下一段任务”时，当前段保存也使用同一个 _pending_task。录制中提前设置下段任务，会把本段标签改掉。

建议：_begin_episode 冻结 episode_task，保存用冻结值；pending_task 仅影响下段。空 task/拼写差异进入任务字典检查。

### 12. P2：导出更新缺少事务式完成标记

位置：convert_to_lerobot.py:410；convert_to_lerobot_v3.py:434。

先删除已有产物再转换，新转换中途失败可能留下半成品；aligned 也直接写最终文件。重跑只检查 aligned 存在，没有版本、源 hash 或完整性判断，可能跳过半写文件或过期产物。

建议：临时文件/临时数据集目录写完并校验后原子替换；保存处理版本、参数、源 manifest 与 completion marker；输出目录拒绝与 raw/v2.1 源目录重叠。

## 现有数据结构检查

| 会话 | 段数 | aligned 帧数 | 有质量标记帧 | 缺失属性段数 |
|---|---:|---:|---:|---:|
| default_task | 5 | 1391 | 0 | 0 |
| pick up and place | 50 | 10962 | 13 | 0 |
| pick up and place 2 | 50 | 10359 | 0 | 0 |
| pick_place_merged | 100 | 21321 | 13 | 0 |
| pick_place_merged_repaired | 100 | 15503 | 1 | 100 |
| pick_place_merged_repaired_v2 | 100 | 16075 | 3 | 100 |
| pick_place_merged_repaired_v3 | 100 | 20883 | 7 | 100 |
| pick_place_merged_smooth | 100 | 20816 | 4794 | 0 |

这些会话包含复制/派生数据，不能把 605 段视为 605 段独立采集。检查未发现已有 aligned action NaN/Inf，也未发现空任务文本。smooth 有 4794/20816≈23.0% 帧携带质量位，建议先拆解 bit 类别及产生原因，不能直接按此判为坏数据。现有 validation_report 属于历史报告，未重新覆盖写入。

## 与开源项目的比较及采用建议

| 项目 | 与 astral 的差异 | 建议采用方式 |
|---|---|---|
| [lerobot-doctor](https://github.com/jashshah999/lerobot-doctor) | 更广泛的最终数据集完整性、重复 episode、泄漏、归一化和 URDF 限制检查 | 导出后作为独立诊断；先人工校准规则，稳定后纳入 CI。不能替代原始采集时钟检查 |
| [Robot Data Studio](https://github.com/Peter-cuhk/robot-data-studio) | 提供视觉质量、运动学检查、人工复核、报告和筛选导出闭环 | 用已经导出的 v3 作为审查输入，或为 Astral raw 写 adapter；优先复用 UI/复核记录与视觉检测，不迁移采集核心 |
| [embodied-sync](https://github.com/anicut-ai/embodied-sync) | 时钟域、延迟、漂移、缺失、对齐策略与报告更明确；目前 alpha | 借鉴时间模型和质量报告，先保留真实源时间再做小样本对齐验证；无法从当前已丢失的曝光戳恢复真实同步 |
| [ARES](https://github.com/jacobphillips99/ares) | VLM 标注、任务成功评估、语义与轨迹检索 | 后期用于成功/失败复核和任务覆盖；现在优先修基本数据契约 |
| [DemInf](https://github.com/jhejna/demonstration-information) | 按示范状态—动作信息估计排序，处理混合质量数据 | 可作为低成本研究基线，需适配 loader，验证是否误删慢速精细操作 |
| [CUPID](https://github.com/agiachris/cupid) | 按闭环策略表现估计示范影响；有 π0 后训练实验 | 具备稳定真机评估集后再试，需要训练策略与评估 rollout |

本地 robot-data-studio 代码也做了抽查：quality/time_sync.py 主要检查输出时间网格与视频边界，无法凭已经规整的 timestamps 发现上游曝光时刻丢失。因此接入第三方工具不是修好采集同步的替代方案。

## 实施顺序

1. 修 repair 路径/属性、armed 状态、冻结 task；加入文件级复现回归。
2. 将 raw→aligned→export 准入契约统一，硬检查 action/时间戳/shape/缺失文件；修暂停处理、command 语义和混合 schema；记录完整输出 manifest。
3. 保留采集时间与时钟域、测同步误差；处理原始质量位和坏 JPEG，采用事务式产物生成。
4. 增加 URDF 限制、冻结/黑帧/模糊、重复数据/泄漏检测；接 lerobot-doctor 与 RDS 人工复核。
5. 固定评估初始条件并与训练数据隔离，比较原始、规则筛选、保守删帧的真机成功率、完成时间和失败类型，再评估 DemInf/CUPID。

应先选小批原始数据在修复后的管线生成新版本，确认兼容回放与训练读取，再迁移全部派生数据。当前原始数据仍保留，可重建溯源；不能依据旧 repaired 文件的全零 source timestamps 判断其物理同步。
