# Vader 5 Pro 输入组件接入记录

日期：2026-10-29。SDK位置为 `src/common/vader5_sdk`，与ROS包 `src/common/morla_vader5_input` 同级。软件接入目标是将其作为与Quest/手机/手套同级的输入，复用MORLA的遥操、数采、模型握手和runtime命令网关。

## 交付与复用

独立输入包：[morla_vader5_input](../../src/common/morla_vader5_input/README.md)，组件 `morla.input.vader5@1.0.0`。包含静态descriptor、ament注册、选中加载的ROS输入源entry point、ROS SDK reader、Joy/Imu诊断、配置、launch、来源/许可与测试。

复用关系：

- 原SDK的Vader5、read_batch及HID transport/parser原样使用；未复制解码器、模拟虚拟手柄或引入完整tux-invader服务。
- 归一化继承公共GamepadAdapter/InputStream；Vader特有层仅处理扩展按钮、设备事件身份、映射与操作边沿。
- RosInputManager增加品牌无关 `morla.ros_inputs` 选中插件接线，无Vader品牌分支；配置发现不导入SDK/设备。
- 动作调用现有ComposableService的activate/disarm/estop/recording API，并复用runtime operation去重与回执；不另发旧Profile操作话题。
- 机器人移动复用CartesianController、原Nero/Astral后端及CR5现有Pinocchio求解器。CR5新增jog入口但共用同一个异步worker、限速、代际隔离和标定门槛。
- 原CR5专用会话宿主提取为公共morla-composable-session，CR5旧控制页/命令保持可用，原默认store保留；Vader launch只提供配置。
- 夹爪复用scalar_map，补齐一维scalar到一关节joint_targets能力检查及可选目标单位速度限制；拒绝将单标量误连多关节。

SDK提供摇杆、扳机、按钮、IMU角速度和含重力加速度，未提供绝对6DoF定位。本轮实现六轴笛卡尔速度点动，没有把IMU积分伪装成VR位置或已校准姿态。raw轴/按钮/IMU/报告与长度均可录制；方向在现场确认。

SDK作为独立Python包迁至ROS包旁边，使用 `python3 -m pip install ./src/common/vader5_sdk` 安装；COLCON_IGNORE使colcon跳过SDK目录。迁移更新README安装路径与来源记录，协议、传输、公开接口及测试源码保持原样。本包采用GPL-3.0-or-later并保存LICENSE/PROVENANCE，SDK来源及NOTICE继续有效。

## 统一控制与按键

默认M1保持、M2切换平移/旋转，LT精细倍率、RT闭合比例，START激活、B撤权、Y开始录制、X结束保存、START+SELECT软件急停。启动已按住的操作键需先释放；组合急停优先且不会在放开SELECT时补发激活。HOME/TURBO/模拟扳机不兼任操作或保持键。

映射可在输入实例参数中配置、轴符号可显式对齐；接线时校验deadman/模式/动作冲突。每一帧先处理操作/释放，再按control_tick派发运动，避免同包按钮行以极小dt覆盖目标。SDK批次内每个按钮变化被保留，录制另记input_buttons事件，不依赖30 FPS是否命中短按。

事件有epoch、sequence、源主机单调解码时间、backend与连接状态；旧会话/重放/过期/后端不匹配拒绝。断开或重连清锚/释放门控，无自动恢复旧跟随。SDK硬件采样时间未知；读批次限128、SDK读超时最大0.2 s，不能据此声明硬实时或证明hidraw队列内采样年龄。

ROS reader独立持有规范hidraw路径的transport锁；runtime持有会话与逻辑输入资源锁。该锁不强制阻止Steam/厂商程序读取，应在现场关闭竞争进程。ROS事件入口不是不可信DDS参与者的安全认证边界。

策略持权时普通输入只观察/录制，不执行IK/人工目标、不自动抢权；B/急停为显式操作。人工、策略、回放使用同一robot state/action契约。默认手柄数据为auxiliary；若训练需要手柄意图作为模型条件，显式调整capture.state/feature contract并重新训练/握手。没有新增专用训练或模型执行通道。

## 初始参数与部署

50 Hz目标连续输入采样加即时按钮变化；默认deadzone0.08、端口时效250 ms；原IMU不额外滤波或姿态融合。实际源采样与端到端频率未测。

CR5示例：线速度0.02 m/s、角速度0.15 rad/s、关节速度0.3 rad/s。点动采用10 µm/0.0001 rad求解接受精度，避免LT精细模式每拍小增量被0.2 mm VR接受容差吞掉；VR模式原默认值保持。EPG扳机示例用公共scalar_map的0.05 m/s目标变化率限制，首次保持帧dt=0不跳目标。

包内有纯fake装配、仅真实手柄+fake执行器部署覆盖、完整CR5 real模板、EPG fake以及Nero/Astral公共jog示例。真实模板保留身份占位和false声明；不假定基座、TCP、方向或停止已验证。不自动添加相机、末端安装变换、双臂同步或启动真实模型推理。

构建、权限、启动、每个按钮/轴与ROS接口详见包README。

## 验证与边界

macOS、独立Python3.13，无ROS/手柄/实体机器人。证据位于 [本轮证据目录](evidence/2026-10-29-vader5/)。

- 现有SDK的5项unittest：协议/27按钮独立位、单位、错误报文、超时不复用旧数据、错误型号拒绝；使用原SDK源码，没有连接设备。
- 新输入/整链测试：公共归一化、模式/精细倍率、按钮冲突与启动释放、组合急停、序号/epoch/源时效、批次边沿、选中ROS插件接线（mock ROS）、fake provenance、配置/契约、采集HDF5保存与边沿事件、策略持权不被人工输入覆盖、同契约假预测器/回放、CR5实际本地求解小步响应。
- 独立input wheel验证：未安装SDK或其他机器人包也能静态发现并加载fake源；实际entry point→source→Service操作→runtime目标→反馈→撤权/锁清理。仅ament索引查询替代，公共框架从源码加载，不等于ROS/colcon验收。

本轮11项新输入/整链测试已包含在172项pure通过中；fake集成23项通过，原SDK另有5项unittest通过。SDK移至同级目录后，再从新路径复跑5项SDK测试与11项输入/整链测试。各pytest层有一条现有Starlette/AnyIO弃用警告。原始输出、wheel和文件哈希见证据manifest与日志；wheel对应迁移前构建，本次仅变更SDK布局与文档，迁移后的源码哈希另列manifest。历史日志不因本轮测试改变实机状态。

NOT_RUN：Linux Humble colcon/IDL与真实ROS传输、Vader USB/2.4G实物、固件/按键/方向/IMU核验、硬件采样/端到端延迟、Nero/Astral/CR5真实跟随、夹爪停止/设备watchdog、真实训练/模型服务、相机同步、碰撞与长稳。本轮未改动原厂SDK或自动连接任何硬件。
