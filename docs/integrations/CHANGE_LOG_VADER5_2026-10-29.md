# Vader 5 Pro 输入接入与 SDK 目录调整详细提交日志

日期：2026-10-29，时区 Asia/Shanghai。仓库 loopkok/MORLA，分支 main。提交前基线为 `5154808ad3110aaa9a807bfaec98296c28dbc4c9`。本轮按用户要求同步最新更新、归档详细日志并提交推送。

本文件记录最终范围及已取得的软件证据；主内容实际SHA和推送结果在成功后另作回执，回执自身SHA由Git历史记录。

## 输入组件与 SDK

新增独立ROS 2包 `src/common/morla_vader5_input`，组件 `morla.input.vader5@1.0.0`、包版本0.1.0。与Quest、手机、手套一样属于kind=input；不依赖具体机器人包。提供descriptor、ament静态注册、`morla.ros_inputs`选中插件、ROS reader、launch、配置、来源/许可和测试。

使用用户提供的morla-vader5-sdk 0.1.0。按后续目录要求，将SDK从工具目录移至 `src/common/vader5_sdk`，与ROS包同级；添加COLCON_IGNORE，由pip单独安装。SDK运行时代码、测试、构建元数据、LICENSE和NOTICE保持原字节，仅更新SDK README安装路径。没有在ROS包内再次复制HID协议或建立另一套解码器。公开导入仍为 `from vader5_sdk import Vader5`，不靠修改sys.path加载SDK。

SDK及本输入包为GPL-3.0-or-later，保留完整许可证与NOTICE；协议/传输来源为TJLawInOrbit/tux-invader，原始内容哈希、当前目录与README原哈希见PROVENANCE.json。没有宣称上游main来源已冻结到一个未提供的commit。

已核对并写入接入指南的既有布局：CR5只在驱动vendor中保留Dobot反馈dtype；LoopHand在驱动vendor中保留所需SDK模块；Nero源码在third_party/pyAgxArm；Astral使用外部安装的astral_robot_sdk，仓库没有该源码。本轮不将这几款标成已完成同级目录迁移。

## 最终变更范围

1. ROS入口 `vader5_input`发布 `/vader5/events` JSON来源事件，以及命名空间内Joy、Imu和status；明确Joy为SDK布局，不与SDL入口重复订阅。支持真实/fake后端，SDK和ROS均延迟导入。
2. 复用SDK的Vader5/read_batch和Linux hidraw传输；真实设备显式指定厂商接口或稳定符号链接，并持有规范设备路径的独占transport锁。读超时/异常发断开事件并结束reader，无自动重连或旧指令重放。每批连续运动取最新报告，按钮边沿逐份保留，超过128份拒绝积压。
3. 继承公共GamepadAdapter/InputStream，输出六轴意图、RT标量、保持键、完整27按钮、raw轴、gyro、accel、未知位、原始报告和长度。IMU单位为rad/s与m/s²（含重力），无姿态融合或绝对6DoF位置；Imu显式标记orientation未知。
4. 默认M1为deadman、M2切换平移/旋转、LT倍率1.0至0.2、RT闭合比例；START激活、B撤权、Y开始录制、X结束保存、START+SELECT软件急停。校验按钮角色冲突，启动已按住操作键须先释放；急停优先，HOME/TURBO/模拟扳机不兼任操作或保持键。映射通过冻结实例参数配置。
5. 事件包含epoch/sequence、源主机单调解码时间与backend；拒绝过期、重放、退休epoch及后端不一致。ROS到达时间不刷新源年龄，部署要求reader与组合宿主使用同一主机单调时钟。源切换/释放清理控制锚点与授权。
6. 公共RosInputManager新增按plugin_id选中加载的morla.ros_inputs机制，先分发所有有效事件，再按control_event处理运动，避免公共宿主硬编码Vader品牌。InputFrame增加兼容默认的synthetic字段，Ingress保留标量端口来源元数据。
7. ComposableService复用现有activate/disarm/estop/recording操作，以输入epoch/sequence/action关联去重及结果回执；录制额外保存input_buttons边沿事件。策略/回放持权时普通人工输入只观察/录制，在IK前拒绝人工目标，不自动抢权。
8. 提取公共 `morla-composable-session` 宿主，新增只读 `/api/v2/active-session`。CR5原VR会话入口仍使用原控制页和默认store，通过共享宿主启动；Vader launch提供配置并拉起reader。
9. CR5新增morla.cr5.cartesian_jog，复用同一URDF/Pinocchio求解器、CartesianController、异步worker及限位/速度/时效/标定门槛。仅jog采用10 µm/0.0001 rad接受容差，避免精细小步被容差吞掉；VR原0.2 mm/0.002 rad默认值保留。没有复制另一套IK。
10. 公共scalar_map增加可选max_velocity（目标单位/秒），并允许一维scalar连接一维joint_targets；编译器拒绝误连多关节。真实公共gamepad jog增加源新鲜度要求。
11. 交付CR5/EPG fake、真实手柄覆盖、CR5 real占位模板及Nero/Astral示例；默认state/action沿用机器人契约，手柄数据为auxiliary。训练、策略与回放复用既有冻结契约、数据路径和runtime网关；没有新建手柄专属模型或执行通道。
12. 更新CHANGELOG、入口README、接入指南、SDK扩展说明、分层测试脚本，保存来源、原始测试和安装验证证据。普通Python SDK与ROS包同级、由pip安装且供colcon跳过的目录约定已写入接入规范。
13. 应用户后续要求新增主机编译/真机测试手册、报告模板及仅fake的实际ROS网关检查入口；覆盖开发/普通安装、IDL/C++/DDS、各设备/输入链、数据/模型、Web、长稳及回退。手册列明现有证据与未做/未实现项，Nero资产哈希、LoopHand real只读和Astral夹爪来源限制继续保留。重构progress更新提交状态与最新测试交接入口。

## 默认频率与主要参数

ROS连续运动目标50 Hz，同时发送按钮变化；deadzone=0.08、输入max_age_ms=250、SDK读超时0.1 s（允许0.01至0.2 s）。SDK的received_at为主机解码时间，不是设备硬件采样时间；上游约500 Hz说明不是本轮实测结果。

CR5 fake示例线速度0.02 m/s、角速度0.15 rad/s、关节速度0.3 rad/s、最大关节步长0.15 rad。EPG示例RT映射为0.05至0 m开口宽度，软件目标变化率0.05 m/s，首次保持帧dt=0。参数和坐标方向均需现场按设备配置，不以模板替代真实标定。

## 已执行验证与提交前核对

环境为macOS、独立Python3.13，无ROS或实体设备。

- pure **172 passed**（8.10 s），包含11项新增输入/整链测试；fake集成 **23 passed**（3.33 s）。两层各有一条现有Starlette/AnyIO弃用警告，无跳过。
- SDK离线unittest **5 passed**，覆盖协议/27按钮独立位、单位、异常报告、超时不复用旧状态、错误型号拒绝。迁至新路径后再次 **5 passed**；输入专项再次 **11 passed**（1.40 s），不与pure总数重复累加。
- 新增输入/整链用例覆盖映射/倍率、按钮冲突/启动释放、组合急停、源时效/epoch/序号、批次边沿、fake来源、配置/契约、HDF5保存及边沿事件、策略不被输入抢权、同契约假预测器/回放、真实CR5数值求解小步响应、mock ROS插件接线。
- 独立input wheel验证静态发现、实际选中entry point、归一化bridge、Service操作、runtime目标/反馈及撤权清理；没有安装SDK和其他机器人包。仅替代ament索引查询、公共框架从源码加载，不等于colcon验收。记录的wheel及其哈希对应SDK目录/文档调整前构建；manifest的files列对应当前源文件，明确区分。
- SDK迁移前10个原文件哈希匹配；迁移后运行时代码、测试、pyproject、LICENSE和NOTICE哈希保持。README路径和COLCON_IGNORE变化已记入来源及迁移证据。
- 提交日志编辑前复核证据manifest的65个文件与SDK来源的11个文件哈希全部一致；代码与通过测试的版本一致，因此复用已通过回归，没有重复运行相同代码的全量测试。
- 新增日志/索引后核查最终AST、JSON/YAML/XML、相对文档链接、清单哈希和Git空白。提交只包含源码、配置、许可证和文本证据，不含venv、缓存、构建目录、wheel或硬件数据。
- 新增fake ROS检查脚本在本机仅做AST、--help及快照门禁检查；实际rclpy/DDS调用按主机手册执行，不能算入172项或宣称ROS实测通过。测试文档内Bash/Python命令块只做语法核对，不在本机执行构建或实体设备操作。

复验命令（MORLA根目录、匹配依赖的Python）：

```bash
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer pure
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer fake
PYTHONPATH=src/common/vader5_sdk /tmp/morla-cr5-vr-tests/bin/python -m unittest discover -s src/common/vader5_sdk/tests -v
```

完整操作见[输入包README](../../src/common/morla_vader5_input/README.md)，实现和边界见[接入记录](vader5.md)，原始证据见[证据目录](evidence/2026-10-29-vader5/)。manifest不包含自身哈希。

## 未执行与已知边界

NOT_RUN：Linux Humble colcon/IDL/真实ROS传输、USB/2.4G实体与固件、各按钮/轴/IMU方向、硬件采样与端到端延迟、Nero/Astral/CR5真实跟随、夹爪物理停止/设备watchdog、真实模型训练/推理服务、相机同步、碰撞与长稳。fake/mock及本地假预测器通过不等于这些项目已验收。Nero既有模型清单问题仍保持原状态。

## Git与推送方式

提交前通过此前已验证的SSH443及本机已有代理fetch远端main，退出0；HEAD和origin/main均为基线，左右计数0/0。永久origin保持原HTTPS地址，使用现有Git身份与SSH密钥，不修改全局配置或关闭主机密钥校验。

主内容按当前main普通提交及fast-forward推送，不force或重写历史；成功后fetch并读回远端ref。随后补交实际推送回执并再次推送，最终核对HEAD、origin/main和远端main一致及工作区干净。实际结果以回执和最终命令返回为准。

主内容 `90fcddafead1095b2db2ce80724fc2ed1346e819` 已成功推送并fetch读回，实际命令结果、统计与时间见[推送回执](PUSH_LOG_VADER5_2026-10-29.md)。
