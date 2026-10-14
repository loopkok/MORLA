# 2026-10-12 数据管线与 Web 数据处理提交日志

## 提交范围与验证状态

本次整理并提交 `db7d8f1` 之后的工作区更新：采集语义与相机时间戳、离线数据处理目录与统一入口、保守清洗及导出准入、Web 数据处理后台和前端，以及对应文档、测试文件和现有前端 dist。

按用户要求，本次提交整理未运行新内容测试，未执行 pytest、构建、数据处理、训练或真机操作。只读取代码差异、整理日志并检查 Git 提交状态。10 月 10 日文档内的测试数量与真实数据结果是已有历史记录，本次没有重新验证，不应视作本次执行结果。

## 1. 采集分段、任务与使能状态

- `data_collect_node.py` 将 arm/disarm 输入归并为一个有效 armed 状态，记录实际跃迁，段开始记录明确的初始状态；meta 标注 `event_format=effective_armed_v2`。
- 暂停时落盘缓冲并记录暂停开始时间；继续时保存旧段并创建新 episode，避免离线对齐把暂停空洞转换为连续动作。段时长不包含末尾暂停等待。
- 段开始冻结任务文本到 `_episode_task`，保存使用冻结值，避免录制过程中修改待用任务后覆盖当前段标签。
- 相机缓冲与 HDF5 写盘分开保存源采样时间和接收时间，新增 `arrival_timestamps`；meta 明确 camera、camera_arrival、command、state 的时间戳语义。
- schema 加强 fps、相机名称、max_gap_ms 与 hold_frames 的输入检查。

## 2. 相机采样时间传递

- `webcam_source.py` 在主机 `read()` 返回时记录时间；`ros_source.py` 保留消息 header 时间，无有效源时间时使用主机时间。
- `collect_hook` 和 `CollectTapPublisher.submit` 传递 `timestamp_ns`，编码队列同时保存该值，发布 JPEG 时使用提交时的时间，避免把编码完成时间当作采样时间。
- streamer 的抽头与预览绑定适配可选时间参数；新增的时间戳回归文件随此次更新提交，但本次没有运行。
- webcam 时间仍是主机读取时间，不能替代曝光时间，也不能据此证明跨设备物理同步。

## 3. 离线工具统一入口与迁移

实现集中到 `src/astral_data_collect/astral_data_collect/pipeline/`，采集节点、写盘和共享 schema 保留在父包。统一入口包括源码脚本 `scripts/data_pipeline.sh`、安装后的 `data_pipeline` 与 `python -m astral_data_collect.pipeline`。

| 命令 | 功能 |
| --- | --- |
| prepare / preview | 新建真实工作副本、段号选择及相机映射；浏览器抽帧与状态曲线 |
| quantify / align / validate | 指令与状态诊断、时间对齐、质量检查 |
| repair / replay | 可选保守清洗、Rerun 回放 |
| export-openpi / export-act / upgrade-v3 | LeRobot v2.1、ACT v3 导出及独立版本升级 |
| verify-aligned / verify-existing | 逐帧来源核对、已有 raw 的隔离样本验证 |
| migrate-cameras | 显式历史相机迁移，保留原地修改语义，先 dry-run |
| process-openpi / process-act | 共用 process.sh 的一键编排 |

`cli.py` 按需加载各命令依赖；脚本按自身位置推导 SDK 路径，支持 `ASTRAL_DATA_PYTHON`。打包包含 pipeline 的 shell 与 README。OpenPI 训练提示、策略回放导入、各包说明和排障文档同步新路径。

以本次相对基线的差异为准，删除 6 个旧 scripts 文件（migrate_camera_labels、quantify_cmd_state、repair_aligned、verify_aligned、vla_process_act、vla_process_openpi），迁移 6 个父包离线模块（align_data、convert_to_act、convert_to_lerobot、convert_to_lerobot_v3、replay_rerun、validate_data）。历史文档中的阶段性数量不作为此次最终差异统计。

旧 ROS 离线 console entry `align_data`、`validate_data`、`convert_to_lerobot`、`replay_rerun` 由 `data_pipeline` 替代；使用旧导入和命令的调用方需要同步迁移。

## 4. 对齐、清洗、质量检查与导出

- 对齐使用过去最近的 command，夹爪零阶保持；拒绝缺相机、非法时间戳、历史暂停空洞及失效缓存复用。aligned 增加来源网格、帧索引和源签名；合成末尾 hold 单独标记。
- 清洗修复相机组内时间字段读取及 HDF5 属性保留；默认保护操作意图与非臂动作，保留 command action，按 next_state 契约重建相应动作；历史相机别名要求显式映射。
- 分开记录训练时间与原始采样时间，保留帧来源，并检查选帧后产生的速度峰值。
- 校验与导出检查 aligned 的 state/action/时间戳、维度、schema、相机集合及严重缺口；逐段准入、全量 JPEG 解码，失败不静默漏段。
- `artifacts.py` 提供路径重叠拒绝、源签名、暂存文件和目录提交；替换目录失败时恢复既有产物。导出保存质量报告与处理清单；ACT 自检支持实际加载和 CPU 预处理检查。
- `verify-existing` 移除本机源路径硬编码，拒绝空输入；完整文件说明见 pipeline README。

## 5. Web 数据处理后台与界面

- 新增 `DataProcessManager`，以独立 Python 子进程调用统一 CLI。支持完整流程或单步、段号与历史相机映射、可选诊断/保守清洗、OpenPI/ACT/双格式导出，以及 v2.1→v3 升级。
- session 处理先复制到任务目录的 working；Web 不自动隔离失败段。原始输入与工作产物分开，失败即停止后续步骤，保留已有结果。
- 后台只允许一个活动数据任务；保存步骤、退出码、起止时间、日志与产物；取消向进程组发 SIGINT，超时后终止；服务重启将未结束任务标记 interrupted，恢复持久历史。
- REST 新增 session 列表、任务列表/提交、取消、任务详情及产物读取；WebSocket ui_state 增加 data_process。正在录制、暂停或保存的源 session 拒绝提交。
- 输入限定在配置数据根目录内，产物读取限定任务目录及允许的文件后缀；相机映射、段号与图像尺寸在提交前检查。
- 新增 `DataProcessTab.tsx`，展示任务参数、步骤状态、日志、质量报告、图像/状态预览、导出产物及历史；API client、类型与状态映射同步。
- 提交已有 dist：替换旧 JS 资源并更新 index.html；setup 安装 dist，服务支持源码默认位置和 ROS share 回退。本次未重新构建或做浏览器验收。

环境配置：`ASTRAL_SDK_ROOT`、`ASTRAL_DATA_ROOT`、`ASTRAL_DATA_PYTHON`、`ASTRAL_DATA_CHECK_PYTHON`；前端可用 `ASTRAL_WEB_MONITOR_DIST` 覆盖默认路径。部署时需配置实际存在的数据处理与 ACT 检查 Python 环境，并重新安装/colcon 构建以更新入口和静态资源。

## 6. 文档与已有验证记录

- 新增 [10 月 10 日审查报告](data-pipeline-audit-2026-10-10.md) 和 [修复与历史验证记录](data-pipeline-fixes-2026-10-10.md)，保留审查时的旧路径和问题背景。
- 更新采集、推理与 Web 包 README/CLAUDE、相关排障文档及 CHANGELOG；现有测试导入迁移到 pipeline，补充统一入口、质量、节点守卫、Web 任务和相机时间戳用例。
- 历史记录涵盖 100 段只读检查、5 段 1066 帧导出与来源核对、LeRobot/ACT 加载及 Web REST 链路；各阶段测试数量不同，详见原文。本次只归档这些记录，没有复跑或确认本地历史产物。
- 新真机采集、训练成功率对比及交互式视觉验收仍不能从此次提交推出；历史 armed 矛盾和物理曝光同步的限制保留在修复说明中。
