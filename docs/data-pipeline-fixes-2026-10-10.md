# astral_ws 数据管线修复与验证

日期：2026-10-10。对应审查： [审查报告](data-pipeline-audit-2026-10-10.md)。代码已修改，原始采集数据未修改。

## 已完成

1. 采集：armed/disarm 更新同一有效状态；任务在开始时冻结；暂停继续保存旧段并新建 episode，避免跨暂停造动作。相机编码保留提交前的源时间戳，raw 另存 arrival_timestamps。
2. 对齐：command 使用过去最近指令，夹爪零阶保持；缺相机、非法时间戳和历史暂停空洞报错。aligned 记录来源网格/帧索引和 raw 签名，源文件更改后拒绝复用。合成末尾 hold 单独标记 bit 4，不计入传感器缺口。
3. 清洗：修复 HDF5 相机组读取路径和属性丢失；默认保留意图停顿，无 command 不删除；保护夹爪、腰、头、末端动作及 hold；command action 保留原值，next_state 重建。训练时间和原始采样时间分开记录，检查新增速度峰值。相机别名要求显式映射。
4. 导出：每段 aligned 必须存在，检查 state/action/时间戳/维数/schema/严重缺口；全量 JPEG 解码，坏帧直接失败；不静默跳过 episode。原子写文件与暂存目录提交，失败保留已有产物。质量报告与处理清单进入 meta。
5. ACT 检查：正确传 repo_id/root，按每段首/中/尾加载图像，用实际数据特征构建 CPU ACT 预处理器并检查归一化后的有限值；结构检查覆盖全部 parquet 与视频帧。

## 回归测试

- 离线模块（schema/align/validate/LeRobot v2.1/v3/ACT/OpenPI 对抗配置/清洗）：81 passed。
- ROS 采集、控制、回放相关：39 passed。
- 视频模块及采样时间戳回归：40 passed。
- 清洗合成自测：12 项通过。
- 最后 command 因果取样与 ACT 检查调整后的专项回归：16 passed。
- git diff --check 无错误。

离线测试使用 OpenPI Python 3.11；ROS 测试使用 source /opt/ros/humble/setup.bash 后的系统 Python 3.10；实际 ACT 加载使用 conda lerobot 环境。测试未连接真机。

## 已有数据实测

源：`astral_data/raw/pick_place_merged`，100 段。原始相机组是 base/left_wrist，但 meta schema 仍是 video8/video0，直接准入检查 100 段均失败；这是配置不一致，不代表 100 段图像/轨迹全部不可用。

提供显式映射 video8=base,video0=left_wrist 后，全量清洗预览 21321→21313 帧，只删除 8 帧。该数据不支持大量去停顿。选取 episode 0、24、49、74、99，在独立副本修正 schema，并从 raw 重建 aligned。

| 段 | 对齐帧数 | 清洗后帧数 | 检查 |
| --- | ---: | ---: | --- |
| 0 | 213 | 207 | warn，仅 W7 |
| 24 | 222 | 222 | warn，仅 W7 |
| 49 | 239 | 239 | warn，仅 W7 |
| 74 | 217 | 217 | warn，仅 W7 |
| 99 | 181 | 181 | warn，仅 W7 |

W7 是旧版相互矛盾的 armed/disarm 快照，旧日志不能可靠恢复实际使能覆盖率，因此保留告警而非伪造 0% 或 100%。

5 段共 1066 帧，30fps，state_dim=8，两路相机；导出 H264 224px 的 LeRobot v2.1，再升级 v3。逐帧核对来源索引、state/action、真实相机时间戳、offset 和全部 JPEG SHA256；全量输出视频解码及现代 LeRobot 加载通过。每段首/中/尾实际执行 ACT 预处理通过。源文件大小/mtime 和 meta SHA256 前后一致；验证只读 raw，未移动或覆写原始文件。

产物目录：`astral_data/qa/2026-10-10-fixed/`。其中：

- verification_report.json：汇总及来源未变检查。
- source_validation.json / repair_preview.json：100 段检查与预览。
- sample_raw / sample_repaired：5 段独立派生文件（sample_raw 两个 raw HDF5 为只读使用的硬链接，避免重复占空间，勿编辑它们）。
- lerobot_v21 / lerobot_v3：可加载导出样本，含质量/处理信息。
- act_preprocessing_check.txt：增强 ACT 预处理检查结果。

复现入口：`src/astral_data_collect/astral_data_collect/pipeline/verify_existing_data_pipeline.py`，参数见采集包 README。输出目录必须新建；当前只完整重建与导出 5 段，未批量迁移旧 repaired 目录。

## 优化选择与边界

借鉴 LeRobot 的数据契约与加载验证，以及 UMI 等采集流程的时间与来源溯源，优先实现可追溯的质量准入、保守筛选和可回滚输出。没有引入学习式筛选器，也没有用删除帧数量衡量清洗效果。

尚未进行新真机采集、训练或成功率对比；这些改动证明管线完整性与接口兼容性，不能证明任务成功率提高。webcam 时间是主机 read 返回时间；曝光时间、跨设备时钟和物理动作延迟仍需硬件测量。历史 armed 状态无法凭现有日志完全恢复。uniformize 会改变执行节奏，应显式启用并单独评估。

## 离线工具目录统一（同日后续）

实现集中到 `src/astral_data_collect/astral_data_collect/pipeline/`，统一入口 `scripts/data_pipeline.sh` / 安装后的 `data_pipeline`。完整文件地图见包内 `pipeline/README.md`。旧根模块、旧 scripts/ 数据工具及旧 ROS 离线入口已删除；一键 OpenPI/ACT 共用 `process.sh`。来源验证不再写死本机目录，并拒绝空 session。

回归：91 项离线测试，含实际统一入口的诊断、对齐、清洗、来源检查、OpenPI/ACT 一键导出及带空格路径；39 项 ROS 回归（36+3）通过。wheel 构建和临时安装通过，包内 process.sh、README 和 console entry 可用；旧入口现已移除，统一使用 data_pipeline。

从 /tmp 通过绝对路径调用统一 verify-existing：100 段只读检查与预览，5 段 1066 帧完整导出、现代 LeRobot/ACT 预处理验证通过，源签名未变。新结果在 `astral_data/qa/2026-10-10-unified/verification_report.json`。此次仅整理代码/命令入口，不批量迁移历史 raw，也未运行训练。

用户要求不保留旧入口后，已删除 14 个旧脚本/模块及 4 个旧 ROS 离线命令，安装目录同步清理并 colcon 重建。移除旧入口兼容测试后，84 项离线测试、39 项 ROS 测试通过；真实样本 1066 帧来源检查通过。当前离线命令只保留 data_pipeline，父包仅保留采集与共享 schema。
