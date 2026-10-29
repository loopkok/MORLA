# MORLA 设备移植与接入指南

> 适用基线：2026-10-28，架构重构提交 `040a965`，本次核对工作区 HEAD 为 `b91211a`。本文面向后续机械臂、夹爪、灵巧手、VR、手套、各类手柄、手机、外骨骼、相机和传感器接入。
>
> 本文规定接入工作及交付要求，不授予任何设备“已通过实机验收”的状态。后续开发应先核对本文与当时源码；存在差异时，以实际接口、测试和部署证据为依据，同时更新本文。文中的“必须”是接入要求，不表示当前程序已自动检查所有要求。

## 目录

1. [架构与接入边界](#1-架构与接入边界)
2. [接入前必须收齐的资料](#2-接入前必须收齐的资料)
3. [所有设备共用的契约](#3-所有设备共用的契约)
4. [组件包、注册与 Adapter 接口](#4-组件包注册与-adapter-接口)
5. [机械臂、夹爪与灵巧手](#5-机械臂夹爪与灵巧手)
6. [VR、手套、手柄、手机与外骨骼](#6-vr手套手柄手机与外骨骼)
7. [相机](#7-相机)
8. [通用传感器、USB 脚踏板与操作事件](#8-通用传感器usb-脚踏板与操作事件)
9. [数据、训练、推理与回放](#9-数据训练推理与回放)
10. [配置与离线接入样例](#10-配置与离线接入样例)
11. [验收步骤和完成条件](#11-验收步骤和完成条件)
12. [已知接线限制与扩展位置](#12-已知接线限制与扩展位置)
13. [每次接入的交付模板](#13-每次接入的交付模板)

## 1. 架构与接入边界

### 1.1 新接入的主线

MORLA 的核心是把设备能力、物理安装、控制权、输入意图和数据身份分开管理，再用同一份编译快照连接遥操、采集、推理与回放。

```text
静态 ComponentDescriptor（组件能力，不连接设备）
  + AssemblySpec（实例、机械连接、输入绑定、采集布局）
  + DeploymentBinding（设备身份、总线、安装标定、验证记录）
    → compile_system()
    → resolved_system.json + capture_schema（SchemaV3）+ 身份哈希
        ├─ RuntimeSession：资源锁 → configure → 准备 → 显式激活
        ├─ 输入源 → InputFrame → BindingController / IK / retarget
        │                         → 带租约和时效的 CommandEnvelope
        ├─ 策略 / 回放 ────────────→ 同一个 runtime 命令网关
        │                           → 单组件 target → Adapter → SDK / 驱动
        └─ Adapter.read / health ← 实测反馈、图像、传感器
                                    → recorder → raw → 对齐/验证/导出
```

职责落点：

- `morla_core`：静态目录、配置校验、装配编译、具名关节、坐标及数据契约；不应为了发现设备而 import 厂商 SDK。
- `morla_runtime`：生命周期、资源占用、控制权、命令校验、停止、设备后端；提供唯一的运动写入边界。
- `morla_input`：设备协议接收、标准化、输入来源/序号/时效；不决定机器人关节怎么动。
- `morla_teleop`：将输入意图映射成目标关节；处理增量锚点、IK、手部重定向。
- `morla_data_collect`：按冻结契约录制和处理数据，不解析每一家厂商的私有报文。
- `morla_learning`、`morla_policy_inference`：特征/几何/归一化握手、模型桥、推理和回放。
- `morla_web_manager`：编译保存、会话操作、输入接线、采集、策略与交互；实际写入仍交给 runtime。
- `morla_operator_control`：现有脚踏、语音、手柄按钮的离散操作路由，主要连接旧 Profile 控制/采集链。
- `src/robot_description`：URDF/MJCF、安装 frame、模型资源、单场景装配。

主要源码：[compiler.py](../src/common/morla_core/morla_core/composition/compiler.py)、[session.py](../src/common/morla_runtime/morla_runtime/session.py)、[composable_service.py](../src/common/morla_web_manager/morla_web_manager/composable_service.py)。

### 1.2 新旧链路不能混用

旧入口 `morla_bringup/launch/robot.launch.py` 以 Profile 选择 Astral/Nero 栈；旧 `FrozenSchema`/Schema v2 及旧 ROS 命令话题仍用于兼容。

新入口是 `system.launch.py` / `morla-runtime`，以及组合 Web 的 `/api/v2`。新设备原则上先实现组件契约和组合链路；确有旧 Profile 使用需求时，再单独实现兼容接线。不要仅往 `robots.yaml` 添加名字，就认为新组合系统已接入。

`system.launch.py` 当前只启动 runtime ROS 网关，**不会自动启动完整输入源、绑定控制器、录制和 Web 服务**。`RuntimeSession` 本身也不会执行 `resolved.bindings`；当前完整输入映射由 `ComposableService` 驱动。无 Web 部署必须提供等价的输入/控制调度宿主，并持续调用 `tick()`。

重构后的代码已存在；旧 [项目导览](project_architecture_guide.md) 的 2026-10-19 结构说明、重构计划和历史日志不能当作当前 API。实现/验收状态另见 [重构提交记录](architecture_refactor/CHANGE_LOG_2026-10-28.md)、[验收规格](architecture_refactor/VALIDATION.md)。

### 1.3 按能力拆组件，不按品牌拆主程序

- 机械臂、夹爪、灵巧手、主动外骨骼驱动属于 `actuator`。
- VR、手套、手柄、手机、被动外骨骼编码器的操作意图属于 `input`。
- RGB 相机、力/力矩、触觉、IMU、温度等属于 `sensor`；相机没有单独的 `kind: camera`。
- IK、重定向、比例映射属于 controller；不要藏在相机或硬件驱动里。
- 多个逻辑执行器可以共用一个物理 `device_session`，但必须明确共享总线/控制板的操作范围。
- 同一设备可同时提供不同能力。例如灵巧手输出关节反馈和触觉；外骨骼既是输入又施加力时，应拆分输入与执行能力，共享会话但分别授权。
- 来自同一示例项目、同时用于一次抓取任务的机械臂和手，仍分别建立独立设备包。每包独立持有 descriptor、Adapter、SDK/协议依赖、配置、启动与测试；组合关系只通过 Assembly/Deployment 表达，不建立绑定两种设备的联合驱动包。可复用的品牌无关机制放公共层，不能让一个设备包 import 另一个设备包来获取通用功能。
- 脚踏的“录制开始”是离散操作事件；“踩住才能运动”是连续 deadman；模拟踏板压力是数值传感器。三者需要不同接线，不能只用一个 `pressed` 字段混代。

## 2. 接入前必须收齐的资料

先完成第 13 节的接入记录，再写真实驱动。缺少资料时明确标记未知，并保持对应能力不可用。

### 2.1 所有设备

- 厂商、型号、硬件版本、固件、SDK/驱动版本、依赖来源和许可证；目标 OS/CPU/ROS/Python 版本。
- 稳定身份：序列号、USB VID/PID 与接口号、CAN 总线及节点 ID、串口身份、网络地址与设备指纹。地址与设备身份分别记录。
- 连接方式、权限、供电、带宽、实际采样/控制频率、最大消息大小、超时行为、断连/重连行为。
- 最小厂商收发示例、有效/无效报文样本、错误码、是否阻塞以及 SDK 线程安全要求。
- 时间戳来自哪里、单位是什么、是否回绕/复位、主机接收时间如何取得。
- 标定文件、版本、内容摘要、安装照片/尺寸及可复现的验证方法。

### 2.2 执行器增加

- 每个可控关节名称、类型、正方向、零点、单位、硬限位/软限位、速度/加速度/力矩限制和来源。
- SDK 的目标模式是位置、速度、力矩还是阻抗；是否缓存旧目标；使能是否立即执行旧目标。
- 测量值与命令回显的区分，反馈有效性、反馈频率、状态/错误读取方式。
- 停止、保持、失能、制动、急停的实际作用；通信中断后设备如何停止，停止是否影响同板其他关节。
- URDF/MJCF、关节到 SDK/模型的映射、基座/法兰/TCP、载荷与安装偏置；HOME/工作位是否涉及碰撞风险。

### 2.3 输入与传感器增加

- 轴/按钮/关键点拓扑、左/右侧、标称范围、中立值、死区、噪声、饱和、置信度、掉点处理。
- 坐标系左右手性、轴方向、位置单位、四元数顺序、位姿相对哪个 frame。
- 配对、重置原点、退出应用、失焦、熄屏、蓝牙掉线等状态是否有明确事件。
- 相机的编码、分辨率、曝光/增益、实际帧率、内外参、畸变、光学坐标、是否硬件同步。
- 力/触觉的单位、零偏、温漂、量程、通道顺序、有效掩码；原始 ADC/count 值不能直接标成 N 或 N·m。

## 3. 所有设备共用的契约

### 3.1 五类身份必须分开

1. `plugin_id + plugin_version`：设备类型/实现版本，例如 `vendor.arm`、`1.0.0`，不是一台硬件的序列号。
2. `instance_id`：本装配中的逻辑角色，例如 `left_arm`、`wrist_camera`；稳定小写标识，影响特征名称。
3. Deployment 的 `device_id`、资源键和 DeviceSession 的 `session_id`：物理设备、独占资源及共享连接身份。
4. RuntimeSession 的 `session_id`：每次运行新生成的会话身份；不能与 DeviceSession ID 混为一谈。
5. 输入 `source_id/source_epoch/sequence` 与控制租约 `producer/token/epoch/sequence`：分别保护输入重连和命令控制权，二者不可互换。

同一物理设备的资源键必须规范一致。不能用串口别名、不同 CAN 名称或不同 IP 字符串创建两套锁后同时控制一块板。共享连接使用 `session_family`、`sharing: session` 和一致的 DeviceSession；不同协议/不同后端不能强行共享。

### 3.2 端口与数值语义

端口 `direction` 是**相对组件**：输入设备输出 pose/axes；执行器输入 target、输出 feedback。端口应声明 `semantic`、`dim`、`unit`、`dtype`、`codec`，必要时声明 frame、shape、更新范围和有效性。

- 关节角用 `rad`，角速度用 `rad/s`；平移明确采用 `m`/`m/s`。
- 闭合比例统一明确为 `ratio`，通常 `0=完全打开，1=完全闭合`。夹爪开口宽度 `m` 与比例不能共用一个无说明字段。
- 位置、速度、力矩、增量、绝对值分别声明；不因数值维度相同就连接。
- 每个向量有完整、固定的名称和顺序。Adapter 收到 target 时必须严格遵守 descriptor 的 canonical 关节顺序；底层返回乱序名称时先校验集合，再重排。
- `NaN/Inf`、重复/缺失/未知关节、单位不匹配和维度错误应拒绝。缺失反馈不能补零伪装正常。
- 只读传感器无需提供动作端口；`state_dim` 与 `action_dim` 可以不同，甚至不含相机。

编译器目前会把不少 numeric stream 标成 `sensor_msgs/msg/JointState`。这不意味着任意传感器实际必须发 JointState，也不意味着已实现该 ROS 订阅；组合 recorder 主要读取 Adapter 的 `ports` 字典。传输与存储契约需分别验证。

### 3.3 坐标、零位和几何对齐

通用数学约定见 [coordinates.py](../src/common/morla_core/morla_core/coordinates.py)：列向量；`T_target_from_source` 表示把 source 坐标转换到 target；URDF RPY 为 `Rz(yaw) Ry(pitch) Rx(roll)`。

接入时必须给出：

- `world/workcell → robot base → flange → connector/mount → tool/TCP` 的方向、单位与变换。
- 输入设备 frame → 选定遥操参考 frame → 机器人基座/求解器参考 frame 的变换。
- canonical joint ↔ SDK joint ↔ URDF/MJCF joint 的双向名称、符号、零位偏置和单位映射。
- FK 的末端究竟是 `solver_tip`、法兰还是任务 TCP；IK 的目标必须先转成求解器需要的末端。

建议在接入记录中明确写出单关节公式，例如 `q_canonical = sign * scale * q_driver + offset`，以及反变换；标出 scale 是否含度转弧度。可使用 [joint_mapping.py](../src/common/morla_core/morla_core/joint_mapping.py) 做严格具名映射，但比例单位换算及异名映射仍需显式实现。正反变换、速度符号、限位转换都要测试。

当前 pose 控制器按 `position=[x,y,z]`、`quaternion=[qx,qy,qz,qw]` 读取；mount 的七元素格式同样为 `[x,y,z,qx,qy,qz,qw]`。MJCF 使用自身的四元数排列，进入模型时需要转换。编译器接受十六元素 mount，但当前单场景 builder 只接受七元素；不要把“编译通过”当作十六元素矩阵已能执行。自建矩阵消费者时明确 4×4 展平顺序并转换到已支持格式。不要将四元数数组当欧拉角，也不要通过逐项取负猜测镜像。

已有 Quest 的 canonical 基变换为 `(-x_unity,-z_unity,y_unity)`，它属于特定输入基变换，**不是所有新设备都必须先套一次的通用公式**。左手系到右手系的反射要在输入基转换层完成；`input_rotation` 必须是行列式 +1 的真正旋转。

IOBT 的 hips 参考系是操作者躯干坐标，不等于机器人 body/base。重置 VR 原点、由 world 切换 body、更换手套标定或左右手切换时，必须更新输入 epoch/参考 frame、清除滤波并重新建立锚点。

`mounts`/标定进入 `geometry_hash`，但当前编译器不会自动把安装图解析成全部 IK 参数。`input_rotation`、`solver_tip_from_tool` 等要从同一标定推导并显式接线，核验 FK、TF、模型与硬件一致。只填写标定文件名不能让文件内容自动进入 hash；应把已解析数值和内容摘要放入冻结契约。

### 3.4 时间、有效性与来源

- runtime 的 deadline、lease、命令 `issued_ns/ttl_ns` 使用运行主机 `monotonic_ns`，不使用 ROS `/clock` 或手机本地时间。
- 设备时间、ROS stamp、主机接收时间、录制时间分别保留，明确 clock domain 和同步不确定度。不同主机 monotonic 不能直接相减。
- `health.sample_ns` 必须是最近一次有效采样的主机单调时间。读缓存时不能每次刷新成当前时间掩盖断流。
- 输入每次重连生成新 epoch；序号严格递增。按钮更新不能刷新另一端口陈旧 pose/landmarks 的时效。
- 来源至少区分实测、估计、命令回显、fake、sim、合成。fake 必须 `measured=false, synthetic=true`；command echo 不能冒充实际位置。
- 实测但无效的值应携带 invalid/fault；同一设备局部通道失效需要保留有效掩码。

[SampleEnvelope](../src/common/morla_core/morla_core/contracts/samples.py) 定义了较完整的样本元数据，但当前各传输/录制路径并未全面采用它。开发者必须检查这些字段是否真正经过接线并写到目标文件，不能仅因存在 dataclass 就宣称已保留。

### 3.5 控制权、停止与运行生命周期

正常顺序是编译验证 → 占有资源 → `configure` → 等待新鲜反馈 → `READY_DISARMED` → prepare → 显式 activate → 获得 lease → submit。import、目录发现、构造函数、预检和启动不得自动 HOME/使能运动。

命令只能通过公共 runtime 网关。其 envelope 包含 `session_id, producer, token, epoch, sequence, issued_ns, ttl_ns, commands`；每个 target 包含 `component_id, names, values, mode, units`。Adapter 只接收已校验的单个 target，不自行生成授权。

- 不新增绕过网关的公开 ROS target 写入口；旧驱动应使用会话私有 topic 或等价隔离及已有 DeviceGuard。
- prepare/activate/操作可能异步返回 RUNNING；按 request_id 查询结果，区分接纳、发送、物理完成。
- prepare 不保证只是读取；现有 native bridge 的 prepare 可调用 enable。真实设备必须说明 prepare 的副作用，把它作为显式设备操作验收。
- 停止必须有设备/驱动层可核验行为，不能发送全零关节目标冒充停止。
- 断连、过期、SDK 阻塞、部分写失败、急停后要撤权、清除旧目标并按作用范围停止。
- reset 不能恢复旧动作；重连必须重新验证反馈，并在观察到 deadman 释放后才允许再次控制。
- 进程被杀不等于硬件停止。真实执行器需要设备 watchdog/停止证据和异常掉电/重连行为记录。

当前网关检查名称、单位、有限值、关节位置范围等，但**不是通用速度/加速度/碰撞保护器**。需在 controller/adapter/设备层实现并验证速率、跳变、速度、加速度和工作空间约束。descriptor 写了 `max_velocity` 不表示网关自动执行了它。

### 3.6 ROS 传输、频率与状态可观测性

默认 `runtime_node` 的私有接口展开为 `/morla_runtime/commands`、`receipts`、`health`、`state`、`operation` 和 `authority`；重命名或加 namespace 后按实际节点路径解析。commands 使用 `morla_interfaces/msg/CommandEnvelope`，operation/authority 使用对应 srv，不能向它发送旧 Profile 的 JointState 命令。

当前 runtime 命令订阅 depth=1，ROS 输入和相机订阅使用 `qos_profile_sensor_data`。新增驱动需要核对发布/订阅的 reliability、durability、history/depth 和速率，避免把 QoS 不匹配造成的无数据误判为设备掉线。厂商原始 topic 可以保留作诊断，运动写入口仍由受信 Adapter 管理。

分别记录设备采样频率、输入更新频率、controller tick、SDK 目标刷新频率、dataset_fps 和模型规划频率；它们可以不同。每层报告最后有效样本年龄、实际速率、丢包/丢帧/队列溢出、SDK 错误、控制权及停止原因。低频传感器不能通过刷新缓存时间冒充高频采样，高频输入也不能无限排队到机器人。

## 4. 组件包、注册与 Adapter 接口

### 4.1 推荐交付目录

以下为建议目录，不是当前强制路径：

```text
src/robots/<vendor>/morla_<device>/       # 执行器；输入/传感器也可在 src/common 独立包
  package.xml / setup.py / setup.cfg
  morla_<device>/
    adapter.py                          # MORLA ABI 与厂商接口转换
    driver.py                           # 有界 I/O、SDK、缓存和错误状态
    descriptor.json                     # 静态能力
  resource/morla_<device>
  resource/components                   # ament 静态 manifest 清单（若用 ament）
  config/assembly.example.yaml
  config/deployment.example.yaml
  test/                                 # 契约、映射、故障、fake/仿真回归
  README.md                             # 安装、配置、操作、限制与实测证据入口
```

模型放 `src/robot_description` 或可定位的受信安装包；现场标定、序列号、认证信息通过 Deployment 绑定，示例中使用占位符。原始数据与现场文件另行保存，不在更新代码时覆盖。

独立厂商SDK与对应ROS 2包放在同一父目录，不放入 `tools`。例如输入设备使用 `src/common/vader5_sdk/` 与 `src/common/morla_vader5_input/`，执行器使用 `src/robots/<vendor>/<device>_sdk/` 与 `src/robots/<vendor>/morla_<device>_driver/`。SDK保留自身构建、版本、许可和来源；普通Python SDK通过pip安装，并放置COLCON_IGNORE供colcon跳过。ROS驱动使用已安装SDK的公开接口，不靠临时sys.path或相对目录加载。无再分发许可时仍由部署环境单独安装，不能为满足目录布局而擅自复制原厂二进制。

现有布局尚未全部统一：CR5仅在驱动内vendor保留Dobot反馈报文布局，LoopHand在驱动内vendor保留所需SDK模块；Nero源码在 `third_party/pyAgxArm`，Astral依赖外部安装的 `astral_robot_sdk`，当前仓库不含该SDK。它们不能视为已符合上述同级目录约定；后续迁移应同时更新导入、安装脚本、来源记录和部署验证。

### 4.2 Descriptor 和发现

完整参照 [三自由度 descriptor](../examples/composable_plugin/descriptor.json) 与 [registry.py](../src/common/morla_core/morla_core/plugins/registry.py)。

必须落实：

1. `descriptor_version: 1`、当前 `api_version`、唯一稳定 `plugin_id`、精确 `major.minor.patch`。
2. `kind`、输出/输入端口、关节列表、`command_modes`、限位来源；sensor/input 不需要伪造 joints。
3. `backend_variants` 分别声明 fake/sim/real 的 available、依赖和验证等级。存在 factory 不等于 real 已验收。
4. `resources` 的 session_family/sharing/stop_policy；`operations` 的 moves 与 scope。
5. `geometry` 的 frame/模型映射及标定需求；不同左右手可使用 `joint_variants`。
6. `configuration_schema` 和 defaults：支持什么参数就明确声明，推荐拒绝未知参数；单位、范围、默认值有说明。
7. `source`、`validation_artifacts` 和版本记录。更改关节语义/顺序、坐标、单位、停止行为后更新版本和验收。

静态发现有两种方式：

- ament resource 类型 `morla_components`；resource 文件每行指向安装 prefix 内一个 manifest 的相对路径。`setup.py` 要安装 manifest 和 ament index 条目。接入包不得使用绝对路径、路径穿越或逃逸 symlink；当前 ament loader 主要校验解析后仍在 prefix 内，Python manifest loader 另有相对路径检查。
- Python distribution 元数据 group `morla.component_manifests`；value 是分发目录内 JSON/YAML 的相对路径，**不是 Python import reference**。检查实际安装的 `entry_points.txt` 和包数据，不能只写一个源码文件就假定发现成功。

示意安装元数据：

```ini
[morla.component_manifests]
custom_arm = morla_custom_arm/descriptor.json

[morla.adapters]
vendor.arm:real = morla_custom_arm.adapter:create_adapter
vendor.arm:fake = morla_custom_arm.fake:create_adapter
```

adapter entry point 使用标准 Python factory。manifest group 是 MORLA 自己读取的静态数据格式，打包工具需要能原样保留该相对路径；不能不经验证就套用普通对象 entry point 的约束。相同 ID/version 重复注册应报错，不能依赖安装顺序选取。

### 4.3 实际 Adapter ABI

权威定义：[contracts/component.py](../src/common/morla_core/morla_core/contracts/component.py)。

```text
# 接口签名示意，非可直接执行的 Python
# 模块级 factory；仅选中组件且 runtime 已持有资源锁后加载
create_adapter(instance: dict, context: dict) -> adapter

# 所有托管组件
adapter.configure(context: dict) -> None
adapter.health(now_ns: int) -> dict
adapter.close() -> None

# 提供可录制采样的组件
adapter.read(now_ns: int) -> dict

# actuator 另外实现
adapter.prepare() -> bool | dict
adapter.activate() -> bool | dict
adapter.submit(target: dict) -> dict
adapter.stop() -> bool | dict

# 可选；只实现 descriptor 声明的操作
adapter.operate(operation: str) -> bool | dict
```

factory context 含 `session_id, clock, resource_root, lane_fd, shared_device_sessions, device_session`。后续 `configure` 收到的是 `{'session_id': ..., 'instance': instance_snapshot}`，不是同一份 factory context。共享连接及 clock 要在 factory 中妥善保留。

执行器 health/read 示例形状（值和时间来自实际缓存，不能照抄为固定常量）：

```python
health = {
    "ready": True,
    "sample_ns": last_feedback_received_monotonic_ns,
    "source": "hardware", "measured": True, "synthetic": False,
    "values": ordered_joint_positions,
}
sample = {
    **health,
    "ports": {
        "position": {"values": ordered_joint_positions, "valid": True},
        "velocity": {"values": ordered_joint_velocities, "valid": True},
    },
}
target = {
    "component_id": "arm_a",
    "names": ["joint1", "joint2", "joint3"],
    "values": [0.1, 0.2, 0.3],
    "mode": "position", "units": ["rad", "rad", "rad"],
}
```

没有测得速度就不要输出假的 velocity；异步多端口须分别保留采样时间。当前组合采集主要读取顶层 `sample_ns` 与端口 `received_ns`，仅写端口 `sample_ns` 不足以证明它会被用于时效判断。

真实执行器还需要可验证的 `deployment_verified` 和 `watchdog_verified` 属性。它们应来自设备对应的证据和配置，不能为了通过 activate 而无条件设 True。编译后实例的 deployment_verified 当前由该实例 Deployment 的 `verified && stop_verified` 产生。

实现约束：

- `health/read` 返回缓存快照并保持短时、非阻塞；SDK 访问放有界 worker/拥有的驱动进程。
- `submit` 快速校验、入队或派发，返回明确 `accepted`；失败返回 False/`accepted:false` 或抛错，不能默认成功。
- prepare/activate 失败使用明确 False 或异常；当前运行时不会把任意 `{'ok': false}` 都识别成失败。成功语义应有测试覆盖。
- `completed:false` 表示只完成派发、仍有待确认项；真实激活不能拿“已发送使能”冒充已经确认使能。
- `stop` 有界、可重复、可确认；`close` 释放线程、进程、SDK 和总线。清理不确定时不得宣称资源已释放。
- 同板停止只有确实覆盖整板时才能标 `device_scoped_stop=True`，否则每个选中逻辑组件均需处理。

参考 [native_ros.py](../src/common/morla_runtime/morla_runtime/native_ros.py)、[process_executor.py](../src/common/morla_runtime/morla_runtime/process_executor.py)、[cameras.py](../src/common/morla_runtime/morla_runtime/cameras.py)。现有 native bridge 有品牌分支，不会因安装新 descriptor 就自动了解新品牌的 SDK。

### 4.4 三层兼容性检查

- `validate_adapter_contract(adapter, instance)`：只检查必需方法及可检查的 Python 签名，不访问硬件。
- `exercise_fake_adapter(adapter, instance)`：只允许明确 fake 的协议行为测试；不能把 real 改名 fake 后调用。
- 真实行为、停止、反馈、总线断连和重连：在对应部署环境单独验收。

CLI 见 [conformance.py](../src/common/morla_core/morla_core/plugins/conformance.py) 和 [SDK.md](architecture_refactor/SDK.md)。外部扩展方法无法 inspect signature 时会报告 `uninspectable_methods`，这不是行为已验证。

## 5. 机械臂、夹爪与灵巧手

### 5.1 机械臂接入顺序

1. 独立验证厂商 SDK 的身份读取、状态读取、错误查询、停止；记录实测反馈来源。
2. 建立 canonical 关节清单和双向映射。用单关节小幅变化核对正方向、零位、限位，禁止凭外观猜左右镜像。
3. 实现 real Adapter、资源锁、watchdog、使能和停止；先确保“只读且 disarmed”可持续工作。
4. 实现 fake；若支持 sim，提供模型及其关节/actuator 映射。不要把 fake 的目标回显称为物理仿真。
5. 验证 FK：模型多个姿态的末端与 SDK/实测对比，误差门槛由设备和任务提前规定并留记录。
6. 验证 IK：以实测状态为种子，包含不可达、限位、奇异附近、解跳变和计算超时；失败要停止本次输出并清除失效锚点。
7. 接遥操输入，验证相对控制和重锚；再接录制/回放，最后接模型。

现有 `solver_for()` 只内置 Nero/Astral。新机械臂可以提供自己的受信 controller 插件；若复用 `CartesianController`，solver 至少实现 `sync_state(q)`、`fk(q)`、`solve(T_target)`，并保证种子、FK、IK 的关节顺序/单位与 canonical 一致。详见 [kinematics.py](../src/common/morla_teleop/morla_teleop/kinematics.py)。

MORLA 的笛卡尔遥操采用相对位移：首次有效输入将人手位姿与机器人实测 TCP 建锚；之后映射位移/旋转增量。松开、断连、frame/epoch 改变后重新建锚，不能把远端绝对世界位姿直接作为机器人目标。

同一控制板上的双臂、头、腰必须说明 HOME/disable/stop 的板级作用范围。只选左臂时，不得误使能或回位右臂/头/腰；未选择关节是 disabled 还是 held 需要实际证明。

### 5.2 夹爪

先确定控制量：开口宽度、单关节角、闭合比例、力或接触状态。推荐把“运动控制量”和“压力/电流/接触观测”分开端口。

比例式例子：`closure` 输入为 ratio，`position` 输出为已测闭合比例，`scalar_map` 将 trigger 范围映射到 `[0,1]`。若夹爪物理开口为 `w`，比例与宽度的转换要写明最大/最小开口、方向、非线性及饱和规则；限位和速度也要转换。

必须验证：打开/闭合方向、空行程、接触物体后的反馈、目标未到达、堵转/过流、断连停止、初始化是否夹合。不能把“发送了 0.7”直接作为“已闭合 70%”的实测状态。

现有 Astral gripper 的 real bridge 将命令角度回显转换为比例，并标 `command_echo / measured=false / ready=false`；该组合不能据此通过真实 ACTIVE 门禁。新增夹爪应提供真实反馈；无法提供时，需设计并验证明确的降级契约，不能沿用虚假的 measured 标记。

### 5.3 灵巧手

需要独立交付三条链：

- 硬件链：canonical 关节 ↔ SDK、限位/速度、电流/故障/停止、左右手身份。
- 重定向链：输入拓扑与局部坐标 → 人手标定/预处理 → 重定向算法 → 完整具名关节目标。
- 传感链：触觉、温度、接触、力等独立采样和有效掩码；不要默认拼进模型动作向量。

已有 XHand 为 12 关节、Wuji 为 20 关节，现有重定向接受完整有限的 `21×3` 或明确展平的 63 元素输入。新的手不应复制这些维度，也不应复制 XHand 右手 `index_bend_sign` 等设备专用修正。

必须记录 21 点的索引语义、指序、腕局部/世界系、长度单位、人手尺度及左右手；“恰好 21 点”不等于拓扑相同。厂商只提供弯曲度时，可选择显式关节映射 controller，不能复制标量伪造完整骨架。算法若生成估计骨架，应标 estimated，并单独验证。

重定向 controller 输出 descriptor 中的完整 canonical 顺序，不包含未声明关节。滤波/上一帧优化状态必须在 reset 清除。检查张手、握拳、对指、外展、左右镜像、掉点、重连和触觉有效性。参考 [retargeting.py](../src/common/morla_teleop/morla_teleop/retargeting.py)。

### 5.4 仿真接入

新机器人仅放入 URDF/MJCF 不会自动加入组合仿真。当前 [composable_scene.py](../src/robot_description/scripts/composable_scene.py) 的受信资产表和组件子树抽取逻辑需扩展，或提供经过隔离验证的仿真后端。

要求：canonical→MJCF 的 `mjcf_name` 一一对应；模型 actuator/sensor 的名称及控制单位正确；装配 mount 有实际变换；多设备进入同一场景而非各跑独立世界。惯量、碰撞、重力、接触和夹爪 mimic/tendon 关系分别验证。

现有部分 legacy mesh 关闭碰撞，自动生成的位置伺服也不代表真实设备动力学。仿真验证必须说明覆盖的是运动学、动力学还是碰撞，不能从“能动”推断所有层都成立。sim 相机缺图形环境时应报告不可用，不以 fake 图像替换。

## 6. VR、手套、手柄、手机与外骨骼

### 6.1 标准化输入与接线

权威源码：[normalized.py](../src/common/morla_input/morla_input/adapters/normalized.py)、[ros_sources.py](../src/common/morla_input/morla_input/ros_sources.py)、[controllers.py](../src/common/morla_teleop/morla_teleop/controllers.py)。

`InputFrame` 当前包含：

```python
InputFrame(
    source_id="operator_input",          # 必须等于装配中的 input instance_id
    source_epoch=current_epoch,          # 当前 Python 输入链为字符串
    sequence=sequence,                    # 每连接严格递增
    received_ns=host_receive_ns,
    frame_id="calibrated_input_frame",
    ports={
        "pose": {"position": [x, y, z], "quaternion": [qx, qy, qz, qw]},
        "deadman": pressed,              # Python bool，不能用 1.0 代替
    },
    source_time_ns=device_source_ns,
    clock_domain="device_clock",
    valid=True,
    port_metadata={
        "pose": {"received_ns": pose_receive_ns, "frame_id": "calibrated_input_frame"},
        "deadman": {"received_ns": button_receive_ns},
    },
)
```

这是形状示意；变量须由真实协议和主机接收过程生成。`InputStream.accept()` 将远端时间当元数据，主机到达时间驱动基础 watchdog；设备网关还须防止缓存积压报文被当作新鲜输入。

每个输入接入必须完成四层：

1. 设备层：HID/蓝牙/USB/UDP/TCP/SDK 的实际读取、身份选择、断流检测。
2. 标准化层：产生 InputFrame，轴/骨架/pose/按钮转换及 per-port 时间。
3. 宿主接线：把 frame 送入当前会话的 `ComposableService.ingest_frame()` 或等价受信宿主；管理 source 的 read/disconnect 生命周期。
4. 绑定层：Assembly 的 source、controller、target、同源 deadman；最终经 runtime.submit。

只实现 `read()` 或只发 `InputEnvelope` ROS 消息不会自动驱动绑定。现有 `InputEnvelope.msg` 的 `source_epoch` 是 uint64，而 Python InputFrame 使用字符串；当前 runtime ROS bridge 未订阅 InputEnvelope。新增 ROS 输入通用桥时需显式解决类型转换、会话认证、epoch 和接线，不能直接假定两个类型等价。

deadman 当前必须来自同一个 source 实例，且值严格为 `True/False`。第一次启动、重连、切换参考系后，先观察到 False，再 True 才允许运动。手套+脚踏、双设备组合按钮须实现显式多源聚合器：每源独立时效、共同 epoch、任何必需源失联撤权，再输出一个聚合 InputFrame；不能直接跨 source 填 deadman 绕过编译器。

### 6.2 新 controller 的接口

现有 controller 能力包括 joint jog、scalar map、笛卡尔增量 IK、手柄笛卡尔 jog、XHand/Wuji retarget。编译器中出现某 controller 名不表示运行时一定安装了实现，例如 identity 仍需实际可调用实现。

外部 controller 需要静态 `kind: controller` descriptor，`controller_types.inputs/outputs` 声明接受/输出的端口 semantic；同时安装：

```ini
[morla.controllers]
vendor.arm_pose = morla_custom_arm.controllers:create_controller
```

factory 接收 context（当前组合宿主含 instance 和 binding），返回 callable：

```text
controller(value, measured, dt, parameters) -> ordered_target_values
controller.reset()  # 有锚点/滤波/积分状态时实现
```

输出是值数组，名字/units/mode 由宿主按目标实例组装。controller 不直接调用设备、不自取控制权。新插件应明确超时、限位、空输入、不可达和重置行为；当前 built-in IK 的求解超时检查在 solve 返回后执行，无法中断卡死的 solver，阻塞算法需额外隔离。

### 6.3 其他 VR / 6DoF 追踪器

- 提供位置 m、规范化四元数 xyzw、tracking validity、handedness、参考 frame、重定位事件、抓握/保持按钮。
- 明确坐标是否来自 world、stage、local、head、hips，明确设备 recenter 后发生什么。
- 位姿与按钮分开保留时间，不能由新按钮维持旧位姿有效。
- 若同时有手骨架和控制器，应显式选择每侧来源，不能让两个源同时发布相同侧的控制流。
- 若设备同时提供头、眼、身体、腕，分别声明端口；当前 ROS 输入桥只接了 Quest 的部分腕/按钮/landmarks，其他端口要新增接线。
- 验证轴向六个方向、旋转方向、静止漂移、追踪遮挡、重定位和重连；首帧保持现有机器人姿态。

现有 Quest 协议中的 packet guard/frame latch 可以参考，但新 VR 不必冒充 Quest 文本报文；应在自己的设备适配层生成等价标准输入。

### 6.4 其他手套

骨架型手套需提供点/骨拓扑、腕局部坐标、长度单位、左右手和标定；弯曲传感器型需提供校准后的通道语义、零位/满量程、串扰及有效位，再选专用 controller。

手套没有位置追踪时，只能提供手型，不能凭空作为机械臂 6DoF 控制源。若腕姿态来自 VR/追踪器，则多源同步、frame 对齐和断连策略是接入的一部分。Wuji glove 的 ROS 输出只能在它的拓扑/预处理契约一致时复用。

### 6.5 Xbox、PS、NS、国产及通用手柄

优先统一到现有 `GamepadAdapter` 认可的 SDL `game_controller_node` 布局；不能假定普通 `joy_node`、XInput、DInput 或浏览器 Gamepad 原始索引天然一致。

当前六轴顺序固定为：`left_x, left_y, right_x, right_y, left_trigger, right_trigger`。轴须有限且在 `[-1,1]`。按钮接受 15 或 21 个；前 15 个按 `a,b,x,y,back,guide,start,left_stick,right_stick,left_shoulder,right_shoulder,dpad_up,dpad_down,dpad_left,dpad_right`，其后为 `misc1,paddle1..4,touchpad`。

- Xbox：核验摇杆 Y 正负、trigger 松开值/按下值、RB 等实际按钮映射。
- PS：将 Cross/Circle/Square/Triangle、L1/R1/L2/R2 等明确映射；触摸板、陀螺仪是额外能力，需要新增声明和读取。
- NS：A/B/X/Y 标识与物理位置的差别、单 Joy-Con 还是组合 Joy-Con、触发键是否只有数字开关，都要记录；左右 Joy-Con 聚合需要共同失联规则。
- 国产手柄：记录当前 XInput/DInput/NS/Android 模式、USB/蓝牙差异；切模式后视为布局/身份改变，重新校验。
- 其他 HID：做 VID/PID/序列号到明确布局的映射，未知布局拒绝；禁止套用“按钮 0 大概是 A”。

交付每个支持模式的中立/满量程/逐按钮实测记录，测量死区与漂移，设置回中后零输出。trigger 可能是 `[-1,1]` 或 `[0,1]`，先规范化再映射夹爪，不能直接假设松开为 0。

当前 `RosInputManager` 通过 `/joy`（可配置 input_topic）接收，不负责启动手柄驱动；`device_name/deadzone` 等 descriptor 参数也不是都已经传到实际适配器。要检查消费者是否真正读取参数，详见第 12 节。

### 6.6 手机

现有 mobile 路径提供 axes/scalar/hold-to-control，不是自动支持手机 AR 的 6DoF：

1. 组合会话中的 input 实例具备 `axes/scalar/hold_to_control` 端口。
2. 调用 `/api/v2/sessions/{session_id}/inputs/{input_id}/pair` 取得配对信息。
3. 按 [composable_api.py](../src/common/morla_web_manager/morla_web_manager/composable_api.py) 的 WebSocket 协议连接，token 认证；使用**连接返回的新 epoch**，不是缓存的旧 epoch。
4. 每次收到服务端 challenge 后，在时限内回传相同 challenge、sequence、epoch、bool hold_to_control，以及可选 scalar/axes/source_time_ns；当前服务端校验 challenge 往返不超过 250 ms。不能只周期发送不带 challenge 的 JSON。
5. 断开、退后台、熄屏、触摸取消和失焦时及时释放；主机仍须依据时效独立撤权。

axes 允许命名 `x,y,z,roll,pitch,yaw`，值范围 `[-1,1]`；scalar 同样是 `[-1,1]`，若表示夹爪应明确映射到 `[0,1]`。为完整录制 axes，建议六轴齐全，未使用轴填 0；当前 ingress 录制会跳过不完整六轴字典。

添加 AR/IMU 位置控制要新增 pose 协议、坐标/标定/漂移/重定位处理及标准输入桥，不能把角度直接塞入归一化摇杆字段。远程网络的认证和传输部署另行实现；不要把当前本机服务假定为已经具备公网遥操部署能力。

### 6.7 外骨骼、主从示教器

- 被动编码器输入：只读，提供每关节名称、单位、零点、方向、有效位。异构主从应使用具名比例/偏置/限位映射或 FK→目标 TCP→机器人 IK。
- 身体/手部外骨骼：明确操作员尺寸标定与机器人几何的区别，不按关节编号直接一对一复制；自由度不足时说明不可表达的方向。
- 主动力反馈外骨骼：额外是 actuator，需要独立的执行器资源、授权、力/速度/功率边界、停止、watchdog 和穿戴验收。`InputFeedbackProvider.feedback(event)` 只是接口声明，当前没有完整通用触觉闭环。
- 位置映射可以先使用现有 position 链；双向力反馈、力矩/阻抗和高频同步需要版本化扩展 controller、网关、Adapter、采集动作语义和训练桥。

当前 schema 能表达 velocity/effort 等名称，不代表端到端已支持这些执行模式；不能把力矩值放入 position target 或使用位置限位校验力矩。

## 7. 相机

### 7.1 可直接复用的路径

- `morla.camera.uvc`：OpenCV 工作进程采集，要求已存在的 `/dev/v4l/by-id/...`；Deployment `device_id` 必须等于该路径 basename。
- `morla.camera.ros_image`：订阅显式 image_topic；原始 Image 当前只接受 `rgb8/bgr8/mono8`，CompressedImage 只接受 JPEG/JPG。
- `morla.camera`：通过 camera_backend 选 ros_image 或 uvc。

外部厂商相机如果能通过受控驱动发布上述格式，可以复用 ROS image Adapter；否则提供专用相机 Adapter。厂商驱动进程的启动、身份检查、重连和退出也必须交付。

当前 RGB 采集不依赖浏览器预览。预览正常不能证明 recorder 正常；关掉预览后仍必须稳定采样。相机读取、压缩和队列必须有界，慢消费者应丢旧帧并报告，不能拖住控制线程。

### 7.2 相机输出与对齐

`read(now_ns)` 的 `ports.rgb` 提供 JPEG bytes `data`、有效性、来源、sequence 和时间；顶层提供新鲜度时间。ImageCache 拒绝源时间倒退，忽略重复源时间。图像内容要做真实解码/尺寸验收，仅有 JPEG 起止字节不等于图像完好。

接入必须记录：

- 设备序列号/USB 拓扑及 image_topic→设备的映射证据；topic 名字不是设备指纹。
- `capture.images` 的语义键，例如 `overview/left_wrist`，不使用不稳定的 video0/video1 作为场景语义。
- 宽高、通道、RGB/BGR 转换、JPEG 质量、实际帧率、丢帧与编码延迟。
- 内参、畸变、光学 frame、外参、手眼标定；安装变化更新 geometry 身份。
- 曝光时刻、驱动时间、到达时刻与时钟域；没有曝光时间时明确记录近似来源。

UVC 当前 `source_ns` 取主机完成 read 后的 monotonic，不能称为曝光硬件时间。ROS image 保留 header stamp 于相机缓存，但当前组合 Web 录制 `append_camera(label,jpeg,timestamp)` 不完整传递缓存里的全部 source_ns/fingerprint 元数据；需要严格相机同步的数据集必须先扩展接线并检查 HDF5，不能只在 Adapter 中添加字段。

### 7.3 深度、双目、红外、热成像、事件相机

现有 RGB/JPEG 链不能直接承载这些物理量。

- 双目分别声明 left/right 图像、外参和同步关系；不能仅水平拼接后不记录来源。
- 深度应保留深度单位、scale、无效值/置信度和无损数组，不能转 JPEG 后当原始深度。
- 红外/热成像需保留位深、温度/辐射/原始计数语义；可另生成 RGB 预览，但预览不是测量数据。
- 点云/事件流通常可变长，需要新 codec、schema、writer、对齐/导出和读取链；现有固定 shape tensor 不能未经设计容纳。
- 深度/RGB 对齐、硬件 trigger、时钟同步、标定都需要实际 SDK/驱动支持和数据证据。

先选择“仅辅助落盘”或“进入模型观测”；后者还须补模型数据加载、normalization、ModelManifest、训练和推理验证。

## 8. 通用传感器、USB 脚踏板与操作事件

### 8.1 通用传感器

固定维度数值传感器可提供 `kind: sensor`、输出 numeric ports 及 read；用 `capture.sensors` 保存带 shape/dtype/valid 的张量，用 `capture.auxiliary` 保存不进入模型的普通流。

- 六维力/力矩：推荐 force `[Fx,Fy,Fz]` N 与 torque `[Tx,Ty,Tz]` 明确单位的两个端口；标出坐标、零偏、载荷补偿和校准。不要用单一 `unit: N` 描述混合六维量。
- IMU：加速度、角速度、姿态分开；说明重力是否扣除、rad/s 与 deg/s、机体轴及协方差/置信度。
- 触觉阵列：逐指/逐点/逐轴顺序、shape、单位、无效通道及温度有效位独立。XHand 历史形状只是该设备的契约，不能用于其他手。
- 数字开关/接近传感器：bool、事件边沿、去抖、采样时间、断线状态；“没收到”不等于 False。
- 编码器：计数转角度/位移比例、零点、环绕与多圈；掉电后绝对位置是否仍可信。

参考 [codecs.py](../src/common/morla_data_collect/morla_data_collect/codecs.py) 的 numeric/tensor_sensor 和 `sensor_fields()`。复杂多字段传感器可通过 recorder.append_sensor 写入，但当前 Web capture_tick 只自动组装 `values + 标量 valid`；需要扩展采集宿主才能保存温度/协方差/逐点 mask 等额外字段。

### 8.2 脚踏和离散操作入口

当前实现见 [pedal.py](../src/common/morla_operator_control/morla_operator_control/pedal.py)、[commands.py](../src/common/morla_operator_control/morla_operator_control/commands.py) 与 [operator_control.md](operator_control.md)。

现有 USB 脚踏使用显式 `/dev/input/by-id/...-event-kbd`，不扫描全部键盘。F13 默认切换跟随，F14 切换录制，F15 长踩 1.5 秒后释放丢弃；EV_KEY value=2 自动重复忽略，启动/重连已按下时需先释放。断开已连接踏板会发送暂停跟随请求。

新开关设备只控制旧操作路由时，可生成：

```python
import json
from morla_operator_control.commands import request
payload = json.dumps(request("record_start", "custom_pedal"))
# 发布为 std_msgs/msg/String 到 /morla/operator/command
```

请求包含 version/request_id/source/command/sent_at；sent_at 是主机 Unix 秒，**与 runtime 命令 monotonic ns 不同**。当前请求仅接受约两秒内的时间，重复 request_id 去重。结果从 `/morla/operator/events` 关联，submitted 不等于完成；保存/丢弃按实际段号和最终回执验收。

新增映射必须说明短按、长按、释放、重复、抖动、启动按住、热插拔和队列溢出行为。优先生成明确 start/stop，而不是网络重试会翻转的 toggle。丢弃等有副作用操作须保留 request_id、当前段身份及已有确认语义。

### 8.3 接到新组合会话时还缺什么

现有 operator router 调用旧 `/morla/control/*` 与 `/data_collect/control`，并未自动对接 `/api/v2` 的 ComposableService。新设备若要操作组合会话，必须新增明确的会话路由桥：

1. 核对 active session_id 和会话类型，不能同时转发给新旧两套系统。
2. 将 pause/stop/release 映射到该会话的 runtime 撤权或操作；录制调用当前会话 recorder。
3. 为 start 定义显式激活与 deadman 的关系；不把脚踏按钮直接变成不受约束的 activate。
4. 按 request_id 关联操作终态/录制落盘状态，正确返回失败或不支持。
5. pause/resume/discard 等旧能力在新 Web API 没有等价入口时，先补语义与测试，不静默当成功。

如果踏板作为持续 deadman，走第 6.1 节的输入及多源聚合契约；如果记录模拟压力，走 sensor 端口。软件脚踏暂停不等于独立硬件急停。

## 9. 数据、训练、推理与回放

### 9.1 先冻结再录制

Assembly `capture` 明确声明：

- `state`：模型观测，允许 position、velocity 等不同输出；排列由显式列表决定。
- `action`：动作端口及顺序；需要完整对应可写实例的 canonical 目标。
- `images`：数据集语义键 → 相机端口。
- `sensors`：独立结构化传感器；`auxiliary`：辅助输入/诊断流，不自动变成学习特征。
- dataset_fps、action_source、hold_frames、max_gap_ms 等对齐要求。

例如三关节臂的位置+速度是 6 维 state，位置动作只有 3 维；增加脚踏事件不应无故改变模型特征。采集过程中不能更改关节顺序、相机语义或标定后继续写同一冻结 episode。

raw 各流有独立时间轴，不能按第 N 行直接配图像和关节。先验证时钟与来源，再使用 pipeline 对齐；state/action/camera/传感器掩码和控制事件必须同步保留。离线 repair 处理工作副本，不能覆盖原始证据。

### 9.2 动作标签与实际完成

请求动作、获授权动作、派发动作、实测后继状态是不同数据。当前组合 Web 的 capture_command 记录网关已接纳并派发的命令，不能称为物理执行完成。

SchemaV3 接受多种 action_source 字符串，但必须检查实际生产者和导出链是否支持。新路径优先验证已有 command/dispatched 行为；不能仅把配置改成 requested/authorized 就认为中间过程已采集。next_state 需有可匹配的 measured position，且要说明它与命令监督的差异。

原始数据中应保留：冻结 resolved、组件/部署/模型版本、事件、来源/valid、丢帧/队列溢出、缺流和失败状态。检查实际 HDF5/meta 内容；当前通路未完整落盘的字段要补齐后再声明支持。

### 9.3 哈希与模型契约

- `assembly_hash`：装配源配置身份；`deployment_hash`：部署源配置身份。
- `feature_contract_hash`：有序 state/action 特征与相机数据键等；实例/端口/单位/语义变化会改变它。
- `geometry_hash`：非 input 实例的 joints/geometry、mounts 和 Deployment calibrations；外部文件内容必须显式纳入才能被追踪。
- `input_mapping_hash`：当前对 bindings 求摘要，**不是全部输入固件/参数/标定的完整摘要**；输入实例参数变化还需检查 assembly/resolved/插件版本。
- `resolved_hash`：冻结快照；运行前 `verify_resolved()` 对 trusted catalog 重新编译验证。

更换输入且模型特征不变，可以保持 feature 身份；换 TCP、手型/关节语义、相机安装、模型槽位和归一化不能假定兼容。SchemaV2 与 V3 的 layout/schema hash 定义不同，禁止直接互认；旧数据通过显式 `to_v3` 保留来源。

训练交付 TrainingSpec，模型交付 ModelManifest：后端 revision、数据指纹、特征、geometry_hash、normalization_hash、checkpoint_hash、camera_map、capacity 与验证状态。模型容量按实际 state/action 检查，不靠静默截断/补零。相机映射方向是“模型槽位 → 数据集键”，必须一一覆盖。

当前策略/遥操主要执行绝对位置目标。新力矩、速度、阻抗、深度/触觉观测接入时，除了硬件适配，还需修改相应动作语义、特征装配、模型输入输出、网关验证和回放。真实 loader、短程训练、checkpoint 加载和预测需各自验证；stub/fake 成功不等于模型已可部署。

## 10. 配置与离线接入样例

### 10.1 可编译的 fake 装配

以下完整 YAML 使用已内置组件，可保存为 `/tmp/morla_integration_example.yaml` 验证。它展示独立 state/action、输入绑定、相机和传感器布局；**fake input 不会自己采集实体手柄**，命令输入还需第 6 节宿主接线。

```yaml
assembly_version: 1
assembly_id: integration_example
revision: 1
instances:
  arm:
    plugin: morla.fake.arm
    version: '1.0.0'
    backend: fake
  pad:
    plugin: morla.fake.input
    version: '1.0.0'
    backend: fake
  camera:
    plugin: morla.fake.camera
    version: '1.0.0'
    backend: fake
  force:
    plugin: morla.fake.sensor
    version: '1.0.0'
    backend: fake
bindings:
  - id: pad_arm
    source: pad.axes
    target: arm.position_target
    controller: morla.joint_jog
    deadman: pad.right_shoulder
    parameters:
      axes_map: [left_x, left_y, right_x, right_y, null, null, null]
      speed: 0.10
capture:
  state: [arm.position, arm.velocity]
  action: [arm.position_target]
  images:
    overview: camera.rgb
  sensors: [force.values]
  auxiliary: [pad.axes]
  dataset_fps: 30
  action_source: command
  hold_frames: 0
  max_gap_ms: 100
```

编译预期：state_dim=14、action_dim=7、相机键 overview、sensor_key=force__values。这是协议样例，不是任何实机限位或控制速度建议。

在已安装/source MORLA、具备配置依赖的环境执行：

```bash
python -m morla_core.composition.cli validate /tmp/morla_integration_example.yaml
python -m morla_core.composition.cli compile /tmp/morla_integration_example.yaml \
  --output /tmp/morla_integration_resolved.json
python -m morla_runtime.cli /tmp/morla_integration_resolved.json --preflight
# 仅对上面的全 fake 样例：启动 disarmed 后退出，不调用 activate
python -m morla_runtime.cli /tmp/morla_integration_resolved.json --once
```

自定义三关节组件从 `examples/composable_plugin` 起步，先注册静态 descriptor，再实现选中后加载的 factory。未安装示例组件前不能直接让默认 catalog 识别 `lab.reference.arm`；隔离验证可使用 `Catalog.register()`，正式运行要验证安装发现。

### 10.2 真实设备 Deployment 模板

以下仅为字段模板，`arm_a` 必须与自己的 Assembly 对应；插件配置 schema 也必须声明示例中的参数。禁止把占位符当真实部署启动。

```yaml
deployment_version: 1
deployment_id: bench_a
instances:
  arm_a:
    backend: real
    device_id: REPLACE_WITH_VERIFIED_SERIAL
    session_id: vendor_board_a
    resource_key: 'can:bench_a:device_a'
    resources: ['can:bench_a:device_a']
    parameters:
      can_channel: can0
      side: left
    verified: false
    stop_verified: false
calibrations: {}
metadata:
  acceptance_record: 'REPLACE_WITH_EVIDENCE_REFERENCE'
```

最初 verified/stop_verified 均为 false；取得对应安装、反馈、停止证据后，才更新指定实例。顶层 `verified` 不代替实例验证。`address/serial/bus/secret_ref` 等元数据字段存在，不代表所有 Adapter 都会使用；驱动实际读取的字段必须核对。

安装关系写入 Assembly `mounts`，完整校准值/摘要写 Deployment `calibrations`；例如连接器和 TCP 不是同一点时分别建模，不能用默认 identity 省略未知偏置。

## 11. 验收步骤和完成条件

### 11.1 分阶段执行

1. **静态/纯逻辑**：descriptor 发现、参数 schema、编译与哈希稳定、禁未知字段、重复资源拒绝、错维度/单位/关节/端口拒绝。导入与编译无硬件副作用。
2. **fake 协议闭环**：configure→health/read→prepare→activate→授权→submit→stop→close；全链路来源明确为 fake。验证外部插件安装后能被默认 catalog、运行时和组合 Web 看见。
3. **输入独立验收**：不用电机，显示并记录各轴/按钮/pose/骨架，测试重连、长按、旧包、乱序、丢包、frame 变化、手机后台和 per-port 过期。
4. **模型/仿真**：具名映射、FK/IK、安装/TCP、左右手、场景一致性、相机渲染；明确未验证的动力学/碰撞范围。
5. **ROS/目标主机**：Humble colcon/IDL/C++ 构建、真实 message/QoS、所有权锁和进程回收；安装 SDK 后再做真实只读反馈。
6. **实机有限动作**：设备停止先验收；逐关节低幅受限动作、方向/零位、跟踪误差、停止时延、丢线/进程退出/总线故障。数值阈值按设备任务先规定，不能拿 fake 默认范围作硬件依据。
7. **遥操整链**：deadman 首次释放、重锚无跳变、反馈变旧撤权、输入/策略/回放互斥、多组件部分失败停机与未选关节保护。
8. **数据闭环**：录制一段→检查 raw 与 meta→align→validate→导出→目标 loader 读取。核对 names/units/图像颜色、时间偏差、来源、掩码和动作标签；缺必需流应失败。
9. **模型/回放**：冻结契约握手、过期模型/动作拒绝、容量和 camera_map、短时回放/推理；具体模型/设备组合另附结果。
10. **稳定性和交付**：重启恢复、带宽/CPU/内存/队列、实际 p50/p95/p99 延迟、超时/丢帧计数；按发布门槛做长稳（现有重构规格要求 8 小时），归档日志与未完成项。

### 11.2 必测反例

- 两个实例占同一物理资源；新旧入口同时启动；SDK 子进程退出后锁/设备是否仍占有。
- 旧 session、旧 token/epoch、重复序号、过期/未来命令、越权组件、只到一半的多组件写入。
- 关节乱序/漏项/重复、单位错误、NaN/Inf、超范围、目标突变、错误模式。
- health 不更新、源时间倒退、设备时钟重置、SDK 长阻塞、stop 未确认、迟到 enable。
- 启动时按钮已经按下、deadman 释放丢失、pose 冻结但按钮持续更新、重连复用旧锚点。
- 手柄切模式、Joy-Con 一半断开、手套掉关键点、VR recenter、手机退后台。
- 相机换 USB 口/重插/序号改变、错 topic、重复 stamp、坏 JPEG、缺帧、分辨率被驱动拒绝。
- sensor mask 局部失效、传感器量程溢出、整型截断、required sensor 无样本。
- 数据盘满、writer 异常、缓冲溢出、停止录制时队列未排空、保存失败却返回成功。

### 11.3 可复用验证入口

在合适的隔离 Python 环境准备依赖，参考 `dependencies/composable-test.txt`；不要在 ROS 系统环境里盲装/升级整份离线测试依赖。真实 ROS 构建使用与 Humble 匹配的系统 Python。

```bash
bash scripts/test_composable.sh --layer pure
bash scripts/test_composable.sh --layer fake
bash scripts/test_composable.sh --layer learning
# 有 MuJoCo/模型/渲染条件时
bash scripts/test_composable.sh --layer sim
# 在对应 ROS Humble 环境，使用项目构建/ROS 验证脚本
bash scripts/build.sh --mode symlink
bash scripts/test_ros.sh
```

分层测试存在重叠，不把各历史数字相加充当唯一用例数。插件自己的测试加入对应测试层；文档/配置小改动只执行相关检查，不必无理由启动实机。

完成状态逐项填 `PASS / FAIL / NOT_RUN / BLOCKED_ENV`，同时写命令、版本、设备、结果文件。只有已有证据的层才能标 PASS；“能编译”“fake 通过”“sim 可运行”均不等于“实机可用”。

## 12. 已知接线限制与扩展位置

以下是对本文基线源码的核对结果，后续接入必须逐项判断是否涉及。它们不是要求所有设备一次性补完全部功能。

1. **输入不是通用自动插件宿主。** ComposableService.start 对所有 input 注入 InputIngressAdapter；安装 `morla.adapters` 的新 input factory 不会在该路径自动替换它。新输入必须接 ingest_frame 及 source 生命周期，或扩展受信 input source 注册/宿主。
2. **ROS 输入驱动未自动拉起。** connect_inputs 启动订阅器；RosInputManager 有 gamepad/xbox、Quest、glove 和来源明确的 quest_right 分支，并支持按morla.ros_inputs entry point加载所选外部源（Vader接入使用此机制），Quest/hand topics 多为固定路径。新 VR/外骨骼、source_namespace、驱动启动/退出需要额外接线。
3. **元数据参数不保证生效。** 当前旧SDL GamepadAdapter分支默认deadzone=0.08，RosInputManager未把其实例deadzone/device_name传入；Vader插件使用自己冻结的输入参数传入公共归一化器；BindingController 默认 250 ms；CR5 VR 接入时已将 binding.max_age_ms 接入组合宿主。runtime 默认反馈新鲜度为 500 ms，不能假定端口 max_age_ms 自动改变它。按现场需求补接并做参数生效测试，不直接修改冻结 snapshot 绕过校验。
4. **插件预检仅提供元数据结论。** CR5/LoopHand 接入时已补充外部 real adapter entry point 检查，不再把直接 TCP/串口插件一律按内置 ROS 品牌处理；该分支仍不加载插件、不验证设备或其全部依赖，报告 dependencies_unchecked。输入 descriptor 的 real available 也不能当作完整输入可用性判断。安装后应验证实际加载、依赖和设备反馈，更不能靠改 backend=fake 绕过。
5. **资源声明与实际锁需核验。** 编译器检查 resources 列表；当前 RuntimeSession.start 主要按 DeviceSession.resource_key 获取设备锁，并持有独占 host lane。一个插件涉及多个物理资源时要核实实际覆盖，不能只写 resources 列表就宣称全部锁住。主机锁也不能替代远端控制器/其他主机的访问协调。
6. **IK 与仿真资产有内置分支。** 新臂需要 controller/solver；新 MJCF 需要资产/场景构建接线。只添加 URDF、descriptor 或 controller 名不足以自动生效。
7. **安装图不自动生成全部控制参数。** geometry_hash、mounts、calibrations 会冻结，但 IK 参数推导和现场验证仍需完成。七/十六元素数据通过编译不等于安装物理正确。
8. **通用力控/触觉闭环尚未完成。** 组合宿主发 position，编译器 capture action 主要生成 absolute_position/closure_ratio，网关按关节单位及位置范围检查。新增模式应贯穿完整链路，不能只增加 command_modes。
9. **同步与控制组仍有限。** 当前编译只支持 best_effort 和 stop_group；runtime 只有一份主要 lease，故障停止可能覆盖整会话执行器。ACTIVE 时还会检查所有 Adapter 的 health；辅助相机/传感器过期也可能导致停止，capture 的 required 标志不能当成运行时故障隔离策略。不能宣称多独立操作者并行控制或严格跨设备同步；控制组与部分失效实际作用域需要验收。
10. **复杂数据元数据未全面贯通。** 相机源时间、输入 frame/source time、复杂传感器多字段/逐通道 mask、requested/authorized 标签均需逐路径检查。Web capture 按 dataset_fps 轮询 latest cache，高频采样若要原速保存，需要单独有界采样写入链，不能把 1 kHz 传感器误称为已 1 kHz 录制。
11. **操作事件仍需新旧桥接。** 现有 pedal/router 不是组合 runtime 的默认操作源。旧 pause/resume/discard 与新采集 API 不能未经设计直接对照；详见第 8.3 节。
12. **真实部署与模型验收未由重构软件回归证明。** ROS 构建、实体设备、训练/模型、长稳及最终发布门禁以证据为准；特别保留 Astral gripper 回显限制。

新增品牌优先在独立包、静态 descriptor、外部 adapter/controller 中扩展。确实触及上述公共能力时，修改最小必要公共层，补覆盖该能力的回归，并更新本节。不要在 Web、数采、策略等多个包反复添加同一品牌判断。

## 13. 每次接入的交付模板

目标主机编译、安装、ROS fake与逐设备真机验收按[主机测试手册](testing/host_build_and_hardware_acceptance.md)执行，使用[报告模板](testing/host_acceptance_report_template.md)留证；该手册区分当前已验证与待执行范围，不代替本指南的接入接口要求。

已按本指南独立接入：[Vader 5 Pro输入](integrations/vader5.md)、[Jodell EPG50-060](integrations/epg50_060.md)、[CR5](integrations/cr5.md)、[Aero Hand Open](integrations/loophand.md)；CR5 另有 [右手柄/右手 VR 链路](integrations/cr5_vr.md)，使用独立 URDF / Pinocchio 本地 IK，纯本体资源位于统一 robot_description；软件已接通、实机待验收；[组合示例](../examples/cr5_loophand/README.md) 只引用两包的组件。验收记录保留当前真实手只读及停止/watchdog 限制。

建议每台/类新设备在 `docs/integrations/<device>.md` 留下以下记录；该目录是建议交付位置，按实际项目组织创建。

```markdown
# <设备/版本> 接入记录

## 基线与范围
- MORLA commit：
- 本指南版本/日期：
- 设备型号/固件/SDK/OS/ROS：
- plugin_id / plugin_version / instance_id：
- 支持能力与不支持能力：
- fake / sim / real 各自验证状态：

## 接口契约
- descriptor、adapter、controller、模型文件位置：
- 端口、名称顺序、shape/dtype、单位、语义、采样频率：
- canonical↔SDK↔模型的名称、符号、比例、零位：
- base/flange/TCP/input/optical frame 和变换方向：
- 时间戳/时钟同步/有效掩码/来源：
- command/feedback/stop/enable/错误处理的实际行为：

## 部署与接线
- 稳定设备身份、资源锁、共享 DeviceSession、未选关节策略：
- Assembly / Deployment / 标定及摘要：
- 参数消费者及生效证据：
- 启动/停止/重连/操作和输入宿主：
- Web、无 Web、旧 Profile 分别支持哪些路径：

## 数据与模型
- state/action 名称与维度，images/sensors/auxiliary：
- action_source、dataset_fps、对齐及时间误差门槛：
- frozen resolved 与哈希、实际 raw/meta 检查结果：
- 导出/loader/回放/模型握手与验证结果：

## 验收与交付
- 每项 PASS/FAIL/NOT_RUN/BLOCKED_ENV、日志/数据路径：
- 实机停止/断连/重连、零位/TCP、控制范围证据：
- 延迟/丢帧/长期运行结果、限制与未完成工作：
- 安装与运行步骤、回退步骤、维护者：
```

交付时同时提供可安装插件包、可编译 Assembly、脱敏 Deployment、所需模型/标定格式、测试与证据、操作说明及本指南涉及部分的更新。后续接入以“本指南要求 + 当时源码契约 + 本设备验收记录”共同作为依据。

## 本文核验记录

本文根据 `b91211a` 工作区的 compiler、runtime、输入、controller/IK/retarget、相机、脚踏路由、录制、学习及组合 Web 代码交叉核对。2026-10-28 使用本机已有 Python 3.13.5 离线环境完成：

- 从本文提取 fake YAML，实际调用 validate/compile CLI、verify_resolved；确认 state_dim=14、action_dim=7、overview 相机与 force__values 传感器。
- 使用临时锁目录执行全 fake preflight、disarmed 启动、prepare、activate、授权、命令提交、重复命令拒绝、stop 与资源释放。
- 现有三自由度参考插件的 descriptor/assembly 编译、结构 ABI 与 fake 行为符合性通过。
- 本文手柄绑定的释放→按下→joint jog 数值验证通过。
- 两个 YAML 块解析、可解析 Python 示例语法和 28 个本地源码/文档链接存在性检查通过；Deployment 占位模板未作为真实部署运行。

本文不改变设备运行逻辑；没有连接实机、执行 ROS 构建或证明真实模型效果。
