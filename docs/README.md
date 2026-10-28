# 排查经验索引（Troubleshooting & Investigation）

可组合架构重构的执行入口：[完整计划](COMPOSABLE_ARCHITECTURE_PLAN.md)、[agent 分阶段任务](architecture_refactor/EXECUTION.md)、[验收与对抗性审查](architecture_refactor/VALIDATION.md)。状态为计划已完成、重构未实施；旧排障和硬件验收证据继续有效。

> 给新会话/接手者：真机/数据/推理出问题先来这里找对应主题的**症状 → 根因 → 修法 → 数值**。
> 本目录按"排查主题"聚合散落在各包 `CLAUDE.md` 坑表、`README` 排障章节与
> `astral_ws/CHANGELOG.md`（按天演进史）里的经验；每个主题给出独立整理版 + 权威源指针。
> 一句话原则：**改参数先看生效、查问题先留日志、数据坏了要当时知道、数值给实测**（见 §方法论）。

## 一、主题 → 文档 → 一句话结论

| 主题 | 独立文档 | 一句话结论 | 权威源 |
|---|---|---|---|
| 坐标、零位与工具链优化 | [coordinate_kinematics_optimization_plan.md](coordinate_kinematics_optimization_plan.md) | 软件实施与仿真/数据闭环已完成；实机 flange、connector、TCP 与安装标定待设备验收 | [审查与实施证据](coordinate_kinematics_audit_2026-10-14.json) |
| MORLA 后续 TODO | [TODO.md](TODO.md) | 声明式、高自由度的机器人与执行器组合装配规划；当前尚未实现 | `morla_bringup/robot_profile_launch.py` |
| ACT 推理卡顿 | [act-inference-stutter-investigation.md](act-inference-stutter-investigation.md) | 真机"一卡一卡"= 四层独立原因叠加（参数没生效/换 chunk 断流/收敛拉回尖峰/数据节奏/网络），按层排查 | `src/common/morla_policy_inference/CLAUDE.md`「推理优化历程」 |
| pi0.5 推理问题调查（进行中） | [2026-09-21-pi05-inference-investigation.md](2026-09-21-pi05-inference-investigation.md) | 抓偏/误抓/卡顿的 A/B/C/D 轮实验设计与结论；左腕视角变化 + 时间轴定责 | `pi05_infer_test_log/`、CHANGELOG 09-21/22 |
| 推理部署环境/后端坑 | [inference-deploy-pitfalls.md](inference-deploy-pitfalls.md) | py3.12 lerobot vs py3.10 rclpy 硬墙、ACT 1 行 chunk、锁饥饿、陈旧队列、绝对语义守卫等 | `src/common/morla_policy_inference/CLAUDE.md` 坑表 |
| 遥操慢速"抖"（低速粘滑） | [teleop-stick-slip-investigation.md](teleop-stick-slip-investigation.md) | 慢速"停一下走一下"根因在**驱动层**（电机 speed 环 kp=0.04 微弱），与遥操/IK 无关；速度前馈无效 | `src/robots/astral/astral_arm_teleop/CLAUDE.md`「低速粘滑排障全记录」、`inference_test_logs/teleop/SUMMARY.md` |
| 逆解求解器对比 | [src/robots/astral/astral_arm_teleop/doc/2026-09-17-ik-solver-comparison.md](../src/robots/astral/astral_arm_teleop/doc/2026-09-17-ik-solver-comparison.md) | geometric 慢速一顿一顿 = 电机死区(~1mrad)×最小关节速度优化；`cmd_deadband_mrad` 实测无效 | `astral_test_logs/2026-09-17_ik_solver_comparison/` |
| 驱动层/电机控制坑 | [motor-control-pitfalls.md](motor-control-pitfalls.md) | 陈旧命令流/板端陈旧目标/使能确认位/HOME 归位/夹爪竞态/死区 一整套"电机突然自己动"的定责路径 | `src/robots/astral/astral_robot_control/README.md`、CHANGELOG 08-09 月 |
| 数采数据质量（卡点治本） | [data-quality-investigation.md](data-quality-investigation.md) | 训练后真机**固定卡点**= 数据走走停停被模型学走；治本在数据修复（repair_aligned 选帧零错位） | `src/common/morla_data_collect/CLAUDE.md`「质量优化历程」 |
| 相机/视频回传性能 | [camera-video-performance-investigation.md](camera-video-performance-investigation.md) | 三路相机拖垮进程=rosidl setter 逐字节校验吃 GIL（630× 加速）；编码器线程/限流/设备指纹各有一串坑 | `src/common/quest3_video_streamer/README.md`、CHANGELOG 08-09 月 |
| 相机 USB3 坏帧/120fps | [camera-usb3-120fps-corruption-investigation.md](camera-usb3-120fps-corruption-investigation.md) | ffmpeg 三类错误分清真伪；坏帧=SS 120fps 编码器过载（帧率 vs 端口四格表实证）；固件更新+迁 USB2@30 根治；USB2 线强制 HS；`pkill -9` 会脏 xHCI | 本会话 Jetson 实机、CHANGELOG 09-23/24 |
| 相机 label 语义化/设备指纹 | [camera-label-semanticization.md](camera-label-semanticization.md) | videoN 是"内核号"不是"角色"；label_aliases 按硬件指纹钉死，换口/重插不漂移 | streamer params.yaml、CHANGELOG 09-18 |
| openpi 训练部署 | [openpi-training-deploy.md](openpi-training-deploy.md) | pi05_base 权重 11.6GB 需 RAM≥32GB（本机 3 次 OOM）；迁移三件套 + 5 个坑 | CHANGELOG 09-11 |
| NN 臂角 IK（piM-IK）对抗性审查 | [src/robots/astral/astral_pim_ik/docs/adversarial_review.md](../src/robots/astral/astral_pim_ik/docs/adversarial_review.md) | F1-F11 发现：ψ=0 约定一致性、单帧 T_ee 不决定 ψ（训练须时间相干轨迹）、已修 3 个真 bug（9D 行列序/L_elbow 遮蔽/IID 不可学） | `src/robots/astral/astral_pim_ik/CLAUDE.md` §5/§8 |
| Quest 3 人在环接管与介入标签 | [审查记录](quest3-hitl-audit-2026-10-12.md) / [修复与验证](quest3-hitl-fixes-2026-10-12.md) | 租约与 ACK 确认、重锚 token、输入限龄、Quest 包防重放及逐帧 HUMAN 标签 | `CHANGELOG.md` 2026-10-12、policy/teleop 与数据管线回归记录 |
| Quest mocap 坐标系/腕位精度 | 记录在 CHANGELOG 08-25 与 `morla_input/README.md` | IOBT 开启后 frame_id 在 world↔body 间切换 → IK 米级跳变（sticky latch + hold gate 修复）；位姿 F4 0.1mm 量化→F7 | CHANGELOG 08-25、teleop-stick-slip 第 2 步 |

## 数据管线近期记录

- [2026-10-12 详细提交日志](update-log-2026-10-12.md)：最终变更范围、旧入口迁移、Web 数据处理、部署配置；本次未运行新内容测试。
- [2026-10-10 数据管线审查](data-pipeline-audit-2026-10-10.md)：采集、对齐、清洗与导出的原始问题和审查证据。
- [2026-10-10 修复与历史验证](data-pipeline-fixes-2026-10-10.md)：修复说明、已有数据来源验证及各阶段测试记录。

## 二、方法论（跨主题沉淀的可复用教训）

这些不是某个 bug 的修法，而是这个项目反复踩、值得每次都先想的模式：

1. **参数生效先验证，不要假设 yaml 被加载**。rclpy 按节点名匹配 `--params-file` 段，顶层键 ≠ 节点名 → 整份参数静默丢弃（推理第 0 层，4 轮测试被吞）；launch 默认值会静默盖 yaml（数采不变量 1）。防护形态 = **launch 默认空串 + 节点启动行自报生效参数**。改任何参数后先看启动行。
2. **查问题先留日志，把"感觉"变成可量化数据**。所有链路都沉淀了 JSONL 诊断：遥操 `teleop_log_file`、推理 `pi_metrics/pi_cmds/pi_control/pi_plan_trace`、驱动 `driver_log_file`、相机 `camera_diagnostics.jsonl`。卡顿/抽动第一件事是开日志拿数据，而不是调参数。
3. **数据坏了要"当时"知道**。low_fps_warning、空录告警（EMPTY-REC）、吞指令留痕（ignored）、参考相机半速红条——采集时可见的信号，别等离线复盘。
4. **模型是示范的镜子**。推理固定卡点/粘滞动作 = 训练数据把操作员的停顿/摩擦学走了。引擎参数只能平滑不能消除，治本在数据（`repair_aligned.py`）。
5. **选帧零错位 vs 插值必然错位**。关节插值必然造成关节 vs 图像错位（图像只能取离散帧），修复数据用"选帧（取原始帧子集）"。
6. **嵌入式 CPU：每帧 Python 开销过 GIL 这根弦**。rosidl setter 逐字节校验、JPEG 编码器线程数、软编码 cpu-used 都是"看着无害、实际拖垮全进程"的坑；瓶颈判断用 py-spy 插桩实证，别猜。
7. **数值给实测、验证给状态**。修性能类问题要有前后数字（"6.3→1.14GB"、"10.1→0.016ms"、"尖峰 7→0"）；未实机确认的修法必须标注 ⚠（如 speed 环 kp 调参）。
8. **防护做成"平滑降级 + 可观测"**：不卡死、不跳变、有计数/红条。单例锁、冻结守卫、逃逸迟滞都是这个形态。
9. **测试复现先行**：bug 先在测试里复现（conftest 注入 NaN/空洞/缺流缺陷），修完转正回归；换 schema/布局/相机配置用 `test_adversarial_configs.py`（openpi 训练侧金标准）把关。

## 三、原始记录归档（要证据链看这里）

| 位置 | 内容 |
|---|---|
| `astral_ws/CHANGELOG.md` | 按天演进史，每个问题的 症状→根因→修法→数值（本目录文档的母本） |
| `inference_test_logs/inference/SUMMARY.md` | 推理真机测试总表（test1-5 实际生效参数/尖峰/错位）+ 诊断结论 |
| `inference_test_logs/teleop/SUMMARY.md` | 遥操粘滑排障 + 训练数据"意图 vs 摩擦"量化结果 |
| `pi05_infer_test_log/` | pi0.5 推理逐轮实验目录（TEST_9xx-xxxx，含三/四份 JSONL） |
| `astral_test_logs/`（sdk 根） | 大测试记录文件夹（独立于 git 仓库），`2026-09-17_ik_solver_comparison/` 含数据+分析脚本 |
| 各包 `CLAUDE.md`「已解决的坑」表 | 包级症状→根因→防护速查（比本目录更贴代码） |

## 四、给"排查一个新症状"的入口

1. 先开对应链路的 JSONL 诊断日志（§方法论 2），拿可量化数据；
2. 到本目录对应主题文档对症状（多数已收录）；
3. 对不上就到 `CHANGELOG.md` 按日期 grep 关键词；
4. 判断是参数/配置类 → 先查"参数是否生效"（§方法论 1），多半是第 0 层坑；
5. 是运动异常 → 判断上游（遥操/IK）还是下游（driver/电机），用 driver_log_file 的 spike 对齐 cmd/send/srv 定"谁先跳"。
