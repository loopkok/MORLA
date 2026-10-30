# MORLA 主机编译与真机测试报告模板

复制本模板到现场证据目录填写；测试入口见[主机测试手册](host_build_and_hardware_acceptance.md)。空项默认为NOT_RUN，不预填PASS。

## 本轮身份

- 开始/结束时间与时区：
- 操作员/复核人：
- 主机/CPU架构/GPU/OS/ROS/RMW/系统Python：
- 仓库完整SHA/分支/工作区dirty diff附件：
- 开发或普通安装树、实际Python模块/share路径：
- SDK版本/路径/native库与依赖freeze附件：
- morla_native_bridge 安装路径与四类 entrypoint 清单（adapter/input/process/kinematics）：
- process_mode、实际 PID/退出码、共享 DeviceSession 与 external 所有者：
- Runtime stop / Web stop / 宿主退出后的输入进程差异与遗留 PID：
- 标准输入端口、axis_names、左右 scalar、单位/范围与滤波参数：
- 设备型号/资产编号/固件/USB、CAN、串口或网络身份：
- Assembly/Deployment/revision/Resolved哈希与标定证据：
- 输入模式、模型/checkpoint/后端版本（适用时）：
- 物理急停/空载支撑/工作范围与首次停止验证记录（真机适用）：
- 证据根目录：

## 单用例记录（每个ID复制一次）

- ID及目标：例如B03、Q02-右手柄、Q02-右手追踪、V01-USB、V01-2.4G。
- 前置条件与所依赖的已通过ID：
- 环境层：源码pure / 安装 / ROS fake / 真实输入+fake / 真实只读 / 真实运动。
- 步骤/完整命令/配置附件：
- 执行开始、结束、退出码/实际运行数量/skip数量与逐条原因：
- 预期结果与预先约定的误差/时效门槛：
- 实际结果（关节/单位/来源/时效/回执/物理行为等）：
- 性能：目标频率、源样本/发布/dispatch实际频率、p50/p95/p99/max、CPU/RSS/队列/丢帧：
- 原始日志/rosbag/视频/截图/数据集及SHA256位置：
- 结果：PASS / FAIL / NOT_RUN / BLOCKED_ENV / BLOCKED_CAPABILITY / NOT_APPLICABLE。
- 阻塞或不适用依据：
- 缺陷/修复提交/复测命令与证据：
- 操作员/复核人及时间：

## 本轮覆盖索引

逐项填写结果及证据，合并执行时仍保留子用例差异：

- H01：环境、版本、证据。
- B01/B02/B03/B04/B05：依赖、开发编译、普通安装、模型资产、colcon/CI。
- T01/T02/T03/T04：pure、fake/SDK、sim、离线数据。
- R01/R02/R03/R04：DDS独立驱动、正负命令、互斥清理、组合故障。
- C01/C02/C03/C04：CR5只读、物理对齐、独立运动、停止/失效。
- Q01/Q02/Q03：Quest+fake、Quest+real（手柄/裸手分列）、录制恢复。
- L01/L02/L03：LoopHand fake、真实只读、真实运动拒绝/能力阻塞。
- V01/V02/V03/V04：Vader输入（USB/2.4G分列）、fake执行器、失效/策略、real执行器。
- E01/E02/E03：EPG只读、状态/初始化、停止及真实控制。
- N01/N02：Nero、Astral及所装末端的新旧路径回归。
- S01/S02/S03：相机、其他输入、传感器与跨源语义。
- D01/D02/D03：实际录制、处理/导出、真实loader/训练/推理/回放。
- W01/W02：前端/浏览器、设备链路人工检查。
- P01/P02/P03：性能、长稳、恢复/发布/独立扩展。
- U01–U08：插件安装、无 Web 双臂、配置替换、进程共享/回收、滤波等价、Web/数据、新代码主机复验、独立审查。
- VALIDATION补充编号及未覆盖项：

## 本轮结论与限制

- 可以使用的确切设备/输入/模式/固件/安装版本：
- 仅只读、fake或mock通过的范围：
- 未通过/未执行/能力阻塞项及责任人：
- Nero模型哈希、LoopHand停止/watchdog、Astral夹爪反馈等已知问题的现场结果：
- 回退版本、配置和执行方式：
- 是否满足本轮限定范围的发布条件及依据：

归档时保留原始数据和原始错误；关闭问题需复测证据，不能用修改验证标志代替。

## 本轮代码与历史证据分离

- 本轮离线参考：统一路径证据中的 202 passed/4 skipped、fake23、sim7、浏览器/构建通过，仅为参考，不预填现场 PASS。
- 历史主机/EPG 台架证据 SHA、实际范围与未关闭 FAIL：
- 新增包后本轮实际 colcon 包数/失败数，勿沿用历史41包：
- 单执行器 ROS fake 脚本与 U02 多组件输入流程分别填写：
- 第二位 reviewer 姓名/意见/复测；若仅自动检查则填 NOT_RUN：
