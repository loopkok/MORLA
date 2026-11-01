# Vader 5 Pro 输入组件接入记录

> 2026-10-30 更新：当前统一路径：Vader 整链 launch 不再单独包含 input launch，公共 session 按 auto/managed/external 管理采集；真实输入需 device 和仅输入 real 的部署覆盖。morla.scalar_map 是直接映射，未执行旧文中的变化率限制；需要该处理时显式选择 trigger_filtered。最新实现/证据见 [统一路径记录](../architecture_refactor/UNIFIED_IMPLEMENTATION.md)。

日期：2026-10-29。SDK位置为 `src/common/vader5_sdk`，与ROS包 `src/common/morla_vader5_input` 同级。软件接入目标是将其作为与Quest/手机/手套同级的输入，复用MORLA的遥操、数采、模型握手和runtime命令网关。

## 交付与复用

独立输入包：[morla_vader5_input](../../src/common/morla_vader5_input/README.md)，组件 `morla.input.vader5@1.0.0`。包含静态descriptor、ament注册、选中加载的ROS输入源entry point、ROS SDK reader、Joy/Imu诊断、配置、launch、来源/许可与测试。

复用关系：

- 原SDK的Vader5、read_batch及HID transport/parser原样使用；未复制解码器、模拟虚拟手柄或引入完整tux-invader服务。
- 归一化继承公共GamepadAdapter/InputStream；Vader特有层仅处理扩展按钮、设备事件身份、映射与操作边沿。
- RosInputManager增加品牌无关 `morla.ros_inputs` 选中插件接线，无Vader品牌分支；配置发现不导入SDK/设备。
- 动作调用现有ComposableService的activate/disarm/estop/recording API，并复用runtime operation去重与回执；不另发旧Profile操作话题。
- 机器人移动复用CartesianController、原Nero/Astral后端及CR5现有Pinocchio求解器。CR5新增jog入口但共用同一个异步worker、限速和代际隔离；回退后标定/验证声明只作记录，不再是激活门槛。
- 原CR5专用会话宿主提取为公共morla-composable-session，CR5旧控制页/命令保持可用，原默认store保留；Vader launch只提供配置。
- 夹爪复用scalar_map，补齐一维scalar到一关节joint_targets能力检查及可选目标单位速度限制；拒绝将单标量误连多关节。

SDK提供摇杆、扳机、按钮、IMU角速度和含重力加速度，未提供绝对6DoF定位。本轮实现六轴笛卡尔速度点动，没有把IMU积分伪装成VR位置或已校准姿态。raw轴/按钮/IMU/报告与长度均可录制；方向在现场确认。

SDK作为独立Python包迁至ROS包旁边，使用 `python3 -m pip install ./src/common/vader5_sdk` 安装；COLCON_IGNORE使colcon跳过SDK目录。迁移更新README安装路径与来源记录，协议、传输、公开接口及测试源码保持原样。本包采用GPL-3.0-or-later并保存LICENSE/PROVENANCE，SDK来源及NOTICE继续有效。

## 统一控制与按键

默认M1保持（作为输入意图）、M2切换平移/旋转，LT精细倍率、RT闭合比例，START激活、B撤权、Y开始录制、X结束保存。START+SELECT 软件急停已随回退删除，急停改用 Web/服务操作或外部硬件急停；M1 的 `hold_to_control` 仍随事件发布，但 runtime 不再要求“按住才动”，启动时是否已按住也不再影响激活。HOME/TURBO/模拟扳机默认不绑定为操作或保持键，回退后误配置也不会被拒绝。

映射可在输入实例参数中配置、轴符号可显式对齐；回退后不再校验 deadman/模式/动作按钮的角色冲突，`action_buttons` 可置空以关闭离散操作。每一帧仍按按钮边沿处理操作/释放，再按control_tick派发运动，避免同包按钮行以极小dt覆盖目标。SDK批次内每个按钮变化被保留，录制另记input_buttons事件，不依赖30 FPS是否命中短按。

事件有epoch、sequence、源主机单调解码时间、backend与连接状态；旧会话/重放/过期/后端不匹配拒绝。断开或重连清锚，无自动恢复旧跟随；回退后 runtime 也不再因松手或断流自动撤权，需要显式 stop/revoke。SDK硬件采样时间未知；读批次限128、SDK读超时最大0.2 s，不能据此声明硬实时或证明hidraw队列内采样年龄。

ROS reader独立持有规范hidraw路径的transport锁；runtime持有会话与逻辑输入资源锁。该锁不强制阻止Steam/厂商程序读取，应在现场关闭竞争进程。ROS事件入口不是不可信DDS参与者的安全认证边界。

回退后策略与人工输入之间没有运行时互斥或租约：同时驱动时后写入者生效，需要上层自律；B 仍可显式撤权。人工、策略、回放使用同一robot state/action契约。默认手柄数据为auxiliary；若训练需要手柄意图作为模型条件，显式调整capture.state/feature contract并重新训练/握手。没有新增专用训练或模型执行通道。

## 初始参数与部署

50 Hz目标连续输入采样加即时按钮变化；默认deadzone0.08、端口时效250 ms；原IMU不额外滤波或姿态融合。实际源采样与端到端频率未测。

CR5示例：线速度0.02 m/s、角速度0.15 rad/s、关节速度0.3 rad/s。点动采用10 µm/0.0001 rad求解接受精度，避免LT精细模式每拍小增量被0.2 mm VR接受容差吞掉；VR模式原默认值保持。EPG扳机示例用公共 scalar_map 做0.05至0m直接映射；变化率处理须显式选择通用滤波控制器。

包内有纯fake装配、仅真实手柄+fake执行器部署覆盖、完整CR5 real模板、EPG fake以及Nero/Astral公共jog示例。真实模板保留身份占位和false声明；不假定基座、TCP、方向或停止已验证。不自动添加相机、末端安装变换、双臂同步或启动真实模型推理。

构建、权限、启动、每个按钮/轴与ROS接口详见包README。

## 验证与边界

macOS、独立Python3.13，无ROS/手柄/实体机器人。证据位于 [本轮证据目录](evidence/2026-10-29-vader5/)。

- 现有SDK的5项unittest：协议/27按钮独立位、单位、错误报文、超时不复用旧数据、错误型号拒绝；使用原SDK源码，没有连接设备。
- 新输入/整链测试：公共归一化、模式/精细倍率、序号/epoch/源时效、批次边沿、选中ROS插件接线（mock ROS）、fake provenance、配置/契约、采集HDF5保存与边沿事件、同契约假预测器/回放、CR5实际本地求解小步响应。按钮冲突、启动释放、START+SELECT 急停和策略持权互斥相关用例已随 2026-10-30 回退删除。
- 独立input wheel验证：未安装SDK或其他机器人包也能静态发现并加载fake源；实际entry point→source→Service操作→runtime目标→反馈→撤权/锁清理。仅ament索引查询替代，公共框架从源码加载，不等于ROS/colcon验收。

本轮11项新输入/整链测试已包含在172项pure通过中；fake集成23项通过，原SDK另有5项unittest通过。SDK移至同级目录后，再从新路径复跑5项SDK测试与11项输入/整链测试。各pytest层有一条现有Starlette/AnyIO弃用警告。原始输出、wheel和文件哈希见证据manifest与日志；wheel对应迁移前构建，本次仅变更SDK布局与文档，迁移后的源码哈希另列manifest。历史日志不因本轮测试改变实机状态。

NOT_RUN：Linux Humble colcon/IDL与真实ROS传输、Vader USB/2.4G实物、固件/按键/方向/IMU核验、硬件采样/端到端延迟、Nero/Astral/CR5真实跟随、夹爪停止/设备watchdog、真实训练/模型服务、相机同步、碰撞与长稳。本轮未改动原厂SDK或自动连接任何硬件。

## 2026-10-30 USB 真实手柄台架（补记）

环境：Ubuntu 22.04 / ROS 2 Humble / `vader5_sdk 0.1.0`；Vader 5 Pro USB 有线，固件 **7.1.5.0**（SDK 上游记录为 7.1.4.0，键位可能随批次不同），配置接口 `/dev/hidraw5`，厂商 37d7。会话经 `morla-composable-session`（`ROS_DOMAIN_ID=91`、`ROS_LOCALHOST_ONLY=1`）。证据：`$HOME/morla-acceptance/vader-20261030-171327/VADER_BENCH_REPORT.md` 及同目录 JSONL/日志。

- 原始输入 **PASS**：两轮合计 24/27 键（缺 FN/TURBO/HOME，操作员在实物上找不到，记 NOT_RUN）；摇杆四轴 [-1,1]、扳机 0→1；最大 5 键并发、边沿无丢失；IMU 角速度与含重力加速度正常。ROS 事件率 47.9–48.8 Hz（SDK 内部约 493 Hz 批处理）。
- fake CR5 jog **PASS（有缺陷）**：真手柄可驱动六轴，START/B/Y/X 动作生效；但摇杆接近满行程时 fake CR5 报 `IK unreachable or limit failure`，运行时将其升级为 `input_errors.pad` 并 `group_stopped/input_or_controller_failed`，整会话撤到 READY_DISARMED，一轮 54 s 内需 22 次重新激活。
- fake EPG **PASS**：RT 0↔1 ↔ 宽度 0.05↔0 m，两轮平滑往返。
- 断流/重连 **PASS（行为记录）**：杀掉受管 reader 后进程记为 `exited/1`，但会话仍 `ACTIVE`、`connected=true`、`fresh=false`、无 `input_errors`、不重启也不撤权；新 reader 上线产生新 epoch 后触发 `group_stopped/input_disconnect` → READY_DISARMED，旧锚点清除，需显式 `activate` 恢复。即：断流本身不感知，靠新 epoch 才撤权。
- 真实 EPG **PASS**：真手柄 → `scalar_map` → 真实 EPG50-060，激活时 RT 松开即 0.0173→0.0500 m；RT 4 次全行程均在约 3.1 s 内到达 0.0000/0.0500 m，全程 ACTIVE、无故障、无输入错误；`stop` 为 `operator_stop`，串口与端口正常释放。

文档对齐的已知行为（代码为准，非缺陷宣称）：

- `/vader5_input/status` 只在断开/出错时发布，没有初始消息。
- `start_recording()` 不检查会话状态，READY_DISARMED 时按 Y 也能开始录制。
- LT 精细倍率只乘六轴 jog；`scalar` 直接取 RT，因此对 EPG 宽度无影响。
- M1 `hold_to_control` 仅为输入意图，不是运动门禁；V04 中未按 M1、仅 RT 即驱动真 EPG。
- 会话启动期 DDS 残留的旧 reader 事件以 stale/foreign-clock 拒绝并打印日志。

仍 NOT_RUN：2.4G/无线、FN/TURBO/HOME 键位与功能、真实 CR5 跟随、物理拔插 USB、LT 在 jog 上的定量减速比、长时连续操作与硬件采样/端到端时延。

设备身份（2026-11-01 复核）：USB `37d7:2401`，USB iSerial 与 HID_UNIQ 均返回产品字符串 “Flydigi Vader 5 Pro”，SDK `ControllerInfo.device_id=130` 为型号常量，**不存在唯一硬件序列号**。真实部署只能以 VID:PID + 资产标签标识，见 `src/common/morla_vader5_input/config/input.real.yaml`；同型号多台必须使用不同标签。
