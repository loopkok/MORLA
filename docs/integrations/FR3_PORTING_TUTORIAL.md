# 从 FR3 实例学习：如何把新机械臂移植到 MORLA

适用代码：`063168d`（2026-11-05）。本文是成员操作教程；接口定义以当前源码为准。它与[设备接入总指南](../device_integration_guide.md)配合使用：总指南描述契约，本教程展示如何把契约落实为文件、插件、配置和测试。

**本实例达到的结果：独立 ROS 2 驱动 + MORLA 执行器插件 + FR3 URDF 严格/微分 IK + 公共遥操 + Web 配置/控制 + fake 录制 + Linux ROS/DDS/浏览器验收。** 没有完成实体机械臂/Quest、真实固件通信、物理停止/断网、真实数据训练，现已补齐 FR3 独立仿真入口与公共 MuJoCo 的运动学/参考动力学模式，尚未完成碰撞及实机参数标定。不要以“移植完成”一个词代替能力与验收清单。

## 1. 先确定你在移植哪一种能力

新机械臂不必一次写完所有功能。按下列里程碑推进，每一步都应能独立交付：

1. **M0，静态设备定义。** 组件可发现、配置可编译，发现时不加载 SDK、不联网。
2. **M1，关节执行器。** fake/real Adapter、反馈、显式激活、命令校验和停止；无 IK 也能经统一网关接收关节目标。
3. **M2，ROS 传输。** 复用公共 ROS 网关，实际验证 DDS 命令、回执、服务和具名关节反馈。
4. **M3，笛卡尔遥操。** FK/IK provider + 现有输入组件 + controller binding；先用合成输入和 fake 机械臂验证。
5. **M4，Web 与数据。** 页面选型/配置/生命周期/诊断/录制可用，检查真实输出文件。
6. **M5，物理仿真，可选。** 单独提供 URDF/MJCF/碰撞与仿真后端、状态映射和场景接线。M1 的 fake 不算 M5。
7. **M6，实机验收。** 逐轴方向、零位、TCP、固件、停止/断网、实体输入和长稳，逐项留证。

FR3 当前已完成 M0–M4 软件路径和 M5 的参考仿真接入；M1 中 real 实体行为、M6 未验收，M5 的碰撞及实机动力学标定也未验收。FR3 遥操现使用与 CR5 同策略的严格/微分 IK。独立包、公共层接线、Web 与实际测试见 [FR3 仿真移植实例](FR3_SIMULATION.md)。

### “多机器人”需要区分三件事

- **支持多种机型**：不同设备包实现相同 ABI，公共输入、ROS、数据和 Web 复用。本次 FR3 实例主要证明这一层。
- **一个装配中有多台设备**：用多个 instance_id 表达，每台绑定自己的设备身份、资源键和 controller。FR3 的每设备 SDK 子进程有助于隔离厂商全局状态，但本轮只验收了一台 fake FR3，未验收双 FR3 同时控制或跨品牌同步。
- **多个独立操作者/会话同时控制**：不能从前两层推导支持。当前 runtime 有独占 host lane，Web 主要管理一个活跃组合会话；控制组同步与故障作用域仍有限。把多台机器人放进一个 Assembly，不等于已有硬实时同步或多会话并行调度。

团队后续应分别给这三种能力设置测试和文档，不用一个“支持多机器人”标签覆盖全部含义。

## 2. 先读懂一次命令怎么流动

```text
组件描述 descriptor + 装配 Assembly + 设备部署 Deployment
                  │ compile_system
                  ▼
             frozen resolved
                  │
       ┌──────────┴───────────┐
       ▼                      ▼
RuntimeSession           ControlHost / Web 宿主
资源锁、生命周期          输入接收 → BindingEngine
       ▲                      │
       │                controller / FK / IK
       │                      │
       └──── CommandEnvelope ─┘
                  │
          设备 Adapter.submit
                  │
         有界 transport / SDK
                  │
             设备状态反馈
                  │
       Adapter.read / health → ROS / Web / recorder
```

注意两条部署路径：

- **独立 ROS driver**：本实例的 `fr3_driver` 只是公共 ROS 网关的薄入口。厂商相关操作在 Adapter/SDK 边界完成，并非再起一个品牌 ROS 节点，然后由 Web 转发到它。
- **Web 工作站**：公共 Web 宿主在自身 RuntimeSession 中加载同一个 Adapter，同时托管输入、绑定和录制。它无需额外启动一份独立 FR3 driver。

二者不要同时占用同一台设备。运行了 ROS driver，不意味着自动运行了完整输入映射；配置中存在 bindings，也不意味着裸 RuntimeSession 会主动执行它们。需要 ControlHost/ComposableService 这样的调度宿主。

## 3. 本次到底做了哪些处理

### 3.1 核对项目主线、资料和能力边界

先阅读总指南、CR5 接入与实现、Nero 接口和测试手册，再核对 compiler/runtime/Web 实际源码。选定新组合架构，而非只往旧 `robots.yaml` 加一个名字。

对用户提供的 SDK 进行代码检查，发现：

- 标准法奥 FR3 是六轴；不使用 Franka FR3 或其他 FR3 子型号的参数。
- SDK J1..J6 使用度，框架使用 rad。
- 初次接入曾使用 V2.2.8 / Robot V4.0.1；当前按现场版本改用 V2.1.5 / Robot V3.8.5，使用 XML-RPC 20003 与实时反馈 20004，见 [迁移记录](FR3_SDK_2_1_5_MIGRATION.md)。
- SDK 存在类级共享状态、后台线程、断网循环重试；其所谓 timeout 装饰器不等于有界超时。
- 有些 getter 读取的是缓存；返回成功码不能证明拿到了新反馈。
- 启动、使能、进入伺服、保持、停止、失能分别需要说明。

**成员要产出的不是 SDK 函数名列表，而是行为说明：会不会阻塞、会不会自动重连/重播、有没有新样本依据、停止怎么确认。** 对未知的实机行为标 NOT_RUN，不能凭 SDK 文档推定已验证。

来源记录放在 [PROVENANCE.json](../../src/robots/fairino/morla_fr3_driver/PROVENANCE.json)，保留源码/手册摘要与许可证。不要从开发者桌面路径直接 import SDK。

### 3.2 确定统一关节契约

本实例在 [contract.py](../../src/robots/fairino/morla_fr3_driver/morla_fr3_driver/contract.py) 中落实：

- canonical 名称和顺序为 `joint1..joint6`，模式为 position，单位为 rad。
- SDK 边界转换 `q_sdk_deg = q_rad × 180 / π`，没有擅自增加符号或零点偏移。
- 手册软限位作为外层边界；部署只能收紧。
- 目标 component_id、名称顺序、模式、单位、维度、有限值、位置范围均检查。
- 真机配置必须提供明确 IP；占位符在连接前拒绝。

新设备必须重新确认名称、顺序、符号、零位、单位和限位。不能把六轴名称、FR3 的 DH 或限位复制给另一台机器人。SDK 乱序反馈应在边界重排，不要求所有下游处理私有轴序。

**当前公共 Runtime 没有统一执行目标名称、单位、TTL、序号和位置范围校验，也没有统一上游命令 watchdog。** 因此，descriptor 写了范围不等于真实命令已被保护；设备 Adapter 必须落实其实际检查，并清楚说明上游停发时的行为。

### 3.3 建立真正可安装的独立包

实际目录：[morla_fr3_driver](../../src/robots/fairino/morla_fr3_driver/README.md)。文件职责如下：

```text
src/robots/fairino/morla_fr3_driver/
  package.xml / setup.py / setup.cfg   ROS 包、安装数据、Python entry points
  descriptor.json                    静态组件能力与参数定义
  resource/
    morla_fr3_driver                  ROS ament 包标记
    morla_fr3_driver_components       静态组件描述清单
  morla_fr3_driver/
    contract.py                      canonical 与部署校验
    sdk_backend.py                   厂商调用、完整反馈帧、状态确认
    transport.py                     隔离进程与有界请求
    adapter.py                       生命周期、缓存、目标调度
    kinematics.py                    本机 FK/Jacobian/IK
    node.py                          公共 ROS 网关薄入口
    check.py                         只读现场诊断命令
    vendor/Robot.py                   本实例固定的原始 SDK
  config/                            单臂、遥操和真实部署示例
  launch/                            独立 ROS / 可选 Web 入口
  test/                              SDK 边界、契约、运行时、Web 测试
  README.md / PROVENANCE.json / LICENSE*
```

SDK 布局是本实例的现实取舍：FR3 将许可允许分发的完整源文件固定在包内 vendor 中，未修改原文件，适配逻辑写在外部。**总指南更推荐 SDK 作为独立同级库安装；FR3 的 vendor 布局并非所有设备的强制模板。** 遇到闭源/不可再分发 SDK，应由部署环境单独安装并声明版本，不能复制进仓库。

### 3.4 完成两类注册，不能只完成一种

**第一类：静态描述发现。** 本实例的 ament resource 清单内容为：

```text
share/morla_fr3_driver/descriptor.json
```

setup.py 必须把清单安装到 `share/ament_index/resource_index/morla_components`，并安装它引用的 descriptor。descriptor 定义插件 ID/version、actuator 类型、position 输出与 position_target 输入、关节、后端能力、参数 schema/defaults、资源策略和来源。

**第二类：实际实现加载。** 本实例注册：

```ini
[morla.adapters]
morla.fairino.fr3:fake = morla_fr3_driver.adapter:create_adapter
morla.fairino.fr3:real = morla_fr3_driver.adapter:create_adapter

[morla.kinematics]
morla.fairino.fr3 = morla_fr3_driver.kinematics:create_kinematics
```

该片段是实际 setup.py 中 entry_points 的说明，不是另建一份重复配置。adapter 工厂匹配的是 `plugin_id:backend`，运动学工厂匹配的是 plugin_id。添加入口后需要重新安装/构建并 source，不是只保存 Python 文件。

发现与实现加载分别参照 [manifest_loader.py](../../src/common/morla_core/morla_core/plugins/manifest_loader.py) 和 [runtime/plugins.py](../../src/common/morla_runtime/morla_runtime/plugins.py)。发现应只读元数据，不导入 SDK。不要让同一 ID/version 通过两条路径重复注册。

**验收重点：** catalog 能找到设备，不能证明实际使用了你写的 Adapter。公共 Runtime 对缺失实现的 fake 设备可能退回通用 FakeAdapter；测试应像 FR3 一样检查实际类为 `FR3FakeAdapter`，并验证它执行了自定义契约。

### 3.5 把厂商 SDK 包成可控的 I/O 边界

本实例没有把 SDK 直接塞进 ROS 回调或 Web 请求线程：

- [sdk_backend.py](../../src/robots/fairino/morla_fr3_driver/morla_fr3_driver/sdk_backend.py)：调用 SDK，确认返回码、使能状态、伺服状态和完整 20004 反馈帧。
- [transport.py](../../src/robots/fairino/morla_fr3_driver/morla_fr3_driver/transport.py)：每个设备一个 spawn 子进程；启动、普通调用和生命周期操作分别有超时。超时终止进程，不自动重连和重放迟到目标。
- 原始 SDK 解码器外增加观察子类：仅完整帧解码成功后复制状态；重复帧不更新时间，帧号回绕允许，解码异常锁存。

不要为了“每个机器人一套结构”而强制所有 SDK 都用进程。若 SDK 本身具备可靠超时、明确线程安全和可关闭的连接，可用线程或同步 transport；**需要统一的是接口、时间和错误语义，不是实现技术。** FR3 进程隔离是针对无限重试与类级状态作出的设备适配决定，目前还不是公共框架自动提供的服务。

进程超时退出只能证明主机调用有界，不能证明实体机械臂停止。通信失效后的物理状态必须保留“停止未确认”，并由现场验收补齐。

### 3.6 实现 Adapter 生命周期，而不是只提供 write(q)

[adapter.py](../../src/robots/fairino/morla_fr3_driver/morla_fr3_driver/adapter.py) 的实际职责：

- `configure(context)`：校验配置、创建 transport、启动后台 I/O；不自动使能、不回零。
- `health(now_ns)` / `read(now_ns)`：提供最近反馈及原始采样时间、有效性、来源、关节值和 ports。
- `prepare()`：要求有新鲜反馈；FR3 中没有设备运动副作用。
- `activate()`：显式使能，必须用请求之后的新反馈确认使能，再进入 ServoMoveStart。
- `submit(target)`：检查当前激活/反馈状态及完整目标契约，写入单槽 pending；接纳不等于发送或到达。
- I/O 循环：取最新目标，默认 125 Hz 按速度上限生成发送点，限制相对实测关节的领先量，迟到不补发历史队列。
- `release()`：公共软暂停，保持最后已发送点；不能把尚未执行的 pending 目标当成保持点。
- `stop()`：清空目标并请求 StopMotion、ServoMoveEnd、RobotEnable(0)，等待新的失能反馈。
- `close()`：关闭线程/连接；软暂停后仍开启的伺服也要清理。从未激活的只读连接关闭时不发送运动/失能操作。

动态装载器对 actuator 检查 configure/health/close/prepare/activate/submit/stop。read/release、运动学、诊断等能力仍要按所选公共宿主的实际消费者实现，不能只满足最低方法列表。

FR3 的反馈示意（字段是实际接口的缩略，不是可直接运行的驱动）：

```python
{
    'ready': True,
    'sample_ns': last_complete_frame_monotonic_ns,
    'source': 'hardware', 'measured': True, 'synthetic': False,
    'values': joint_positions_rad,
    'ports': {'position': {
        'values': joint_positions_rad, 'valid': True,
        'received_ns': last_complete_frame_monotonic_ns,
    }},
    'device_status': {'enabled': 1, 'main_code': 0},
}
```

读取缓存时不要重写 sample_ns。故障后即使保留上一组 values 用于诊断，也必须将 ready/valid 置为 false。fake 不可标为实测；输入监听器的就绪与机械臂实测反馈也不是同一种来源。

### 3.7 复用公共 ROS 网关，保持单一写入边界

[node.py](../../src/robots/fairino/morla_fr3_driver/morla_fr3_driver/node.py) 的核心只有：

```python
from morla_runtime.ros_bridge import main as run
run(args, node_name='fr3_driver', required_plugin='morla.fairino.fr3',
    publish_joint_states=True)
```

因此 FR3 自动复用了 CommandEnvelope/CommandReceipt、RuntimeOperation、ControlAuthority、state/health 及具名 JointState 的 ROS 接线。独立节点下实际接口为：

- `/fr3_driver/commands` 与 `/fr3_driver/receipts`；
- `/fr3_driver/operation` 与 `/fr3_driver/authority`；
- `/fr3_driver/state`、`/fr3_driver/health`；
- `/fr3_driver/arm/joint_states`（本例 instance_id 为 arm）。

新设备不要另外建立一个无授权边界的公开 JointState 写入话题。已有外部 ROS 驱动可以做一个受信桥接 Adapter，但也应保留唯一设备拥有者，明确反馈、生命周期和锁的实际覆盖。

### 3.8 运动学、遥操和输入分别接

FR3 的 [kinematics.py](../../src/robots/fairino/morla_fr3_driver/morla_fr3_driver/kinematics.py) 提供：

```python
solver.sync_state(q_measured)
T_base_from_flange = solver.fk(q)
q_target = solver.solve(T_base_from_flange_target)  # 失败返回 None
```

当前版本按独立 `fr3_description` 中 fairino3_v6 URDF 计算 FK/Jacobian，提供 Pinocchio 严格 DLS 与微分 IK，不向控制器请求 IK。旧 DH 实现仅保留为迁移测试的独立几何参考。所有输入输出关节遵守 canonical 的顺序与 rad；位姿平移使用 m。TCP 偏置由公共 controller 的 `solver_tip_from_tool` 处理。

新绑定使用 `morla.fr3.cartesian_delta` / `morla.fr3.cartesian_jog`，默认微分 IK，可选 strict。旧 `morla.cartesian_delta_ik` 通过运动学插件继续走严格 URDF IK。公共 CartesianController 已负责相对位姿建锚、输入基变换、TCP 转换和调用 solver；不需要重新编写一套 Quest 接收器。重锚经 `reanchor()` 协议确认后才提交控制。

不要混淆四个部分：

- **模型**：DH/URDF 决定几何和参考坐标系。
- **求解算法**：解析解、严格迭代 DLS、连续微分 IK 等。
- **controller**：把输入目标、锚点与测量转成关节目标。
- **设备伺服**：Adapter/SDK 如何调度与执行这些关节目标。

此次 FR3 已按 CR5 移植 Pinocchio/URDF 连续微分 IK、目标吸收、奇异性相关阻尼、指令积分连续性和异步调度，同时保留严格模式。算法同策略不等于真机遥操质量相同：FR3 的模型、SDK、反馈率和硬件约束仍分别验证。具体资产提取、坐标/限位、插件与 Web 接线、验证方法见 [URDF/IK 迁移实例](FR3_URDF_IK_MIGRATION.md)。

### 3.9 用 Assembly 表达组合，用 Deployment 表达具体设备

先编译 [assembly.fake.yaml](../../src/robots/fairino/morla_fr3_driver/config/assembly.fake.yaml)，再使用 [quest.fake.yaml](../../src/robots/fairino/morla_fr3_driver/config/quest.fake.yaml) 加入输入和绑定。前者只有执行器和六维 state/action；后者加入右侧 Quest、公共 controller、deadman、输入时效、坐标变换和辅助采集通道。

绑定的关键部分：

```yaml
source: quest.right_pose
target: arm.position_target
controller: morla.fr3.cartesian_delta
deadman: quest.right_hold
max_age_ms: 250
input_basis_id: unity_left_handed_v1
```

实际 YAML 中还有 input_rotation、solver_tip_from_tool、速度/步长等参数。FR3 示例基变换假定操作者站位，不是所有机械臂的默认安装标定；Unity 左手系到机器人右手系可以包含反射，不要当普通旋转随意改符号。

[deployment.real.example.yaml](../../src/robots/fairino/morla_fr3_driver/config/deployment.real.example.yaml) 只绑定实体身份、唯一资源键、地址和现场参数。机械臂与输入后端可以独立选择，例如真实 Quest + fake 机械臂。不要新建“FR3+某灵巧手”联合驱动，组合关系写在 Assembly 中。

当前 verified/stop_verified/kinematics_verified 等字段是记录，不构成完整运行门禁。填 true 不能替代现场检查。DDS 域隔离也不隔离 TCP/CAN/串口设备，更不能代替唯一资源键。

### 3.10 Web 接线：当前已改为插件声明驱动

2026-11-06 框架改造后，下面的手工修改记录仅供代码考古，不再是新设备接入步骤。FR3 的显示名、模板、默认控制器、能力限制和仿真资源现在均在设备包 descriptor 中声明。Web 与编译器读取同一控制器能力，场景构建器读取 `geometry.simulation`，无需再添加品牌分支。

新接入按 [插件声明驱动规范](PLUGIN_DRIVEN_PORTING.md) 操作。已有配置须重新编译并保存新 revision，保留原有标定和历史数据哈希。

首次 FR3 接入曾修改以下位置（已由后续框架改造替代）：

- [composable_api.py](../../src/common/morla_web_manager/morla_web_manager/composable_api.py)：登记 `fr3-fake` 模板。
- [workcell.ts](../../src/common/morla_web_manager/web/src/lib/workcell.ts)：设备显示名、FR3 专属 controller 的兼容性与默认值，以及旧公共笛卡尔 controller 的机器人白名单。
- [ComposableTab.tsx](../../src/common/morla_web_manager/web/src/components/ComposableTab.tsx)：预设加载列表/选项与 FR3 绑定默认值。
- 公共报告展示：支持通用 controller 的嵌套 solver_report，过滤未使用的空报告；这是公共修复，不应复制进每个机器人页面。

**只安装 Adapter 工厂仍不足以提供模板与求解器。** 同时安装静态描述符、声明能力和包内模板后，Web 自动发现；实际工厂、模型和运行能力仍须逐项验收。

新增设备通常无需修改 recorder、策略特征布局、Quest 线协议和公共 ROS 消息。若你准备在这些包里再次增加品牌判断，先检查是否遗漏了 ports、joint metadata、插件注册或绑定配置。

## 4. 跟着 FR3 在目标机复现实例

以下从 MORLA 根目录运行。它们针对已有 FR3 例子，不是另一个未实现设备可直接运行的命令。先退出 conda，使用与 Humble 一致的系统 Python。

### A. 包识别与构建

```bash
source /opt/ros/humble/setup.bash
colcon list --packages-select morla_fr3_driver
# 必须识别为 ros.ament_python；若显示 python，先检查 package.xml。
colcon build --symlink-install --packages-up-to morla_fr3_driver morla_web_manager \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 pkg prefix morla_fr3_driver
```

真实 SDK 导入依赖按包 README 安装。fake 的目录发现、配置编译与运行不应以厂商 SDK 已安装为前提。检查 `setup.cfg` 的 console script 安装位置与 setup.py 的 data_files，不要只看 colcon 退出码。

### B. 确认发现了描述和实现

```bash
/usr/bin/python3 - <<'PY'
from importlib.metadata import entry_points
from morla_core.plugins.registry import default_catalog
print(default_catalog().get('morla.fairino.fr3')['plugin_version'])
for group in ('morla.adapters', 'morla.kinematics'):
    print(group, [(e.name, e.value) for e in entry_points(group=group)
                  if e.name.startswith('morla.fairino.fr3')])
PY
```

如果改了 descriptor/entry points，却只重启页面，安装目录可能仍是旧内容。先重新构建/source，再重新编译、保存新的装配快照。

### C. 独立 fake ROS 驱动与 DDS

终端 1：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=126 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 -m morla_core.composition.cli compile \
  src/robots/fairino/morla_fr3_driver/config/assembly.fake.yaml \
  --output /tmp/fr3-tutorial.resolved.json
ros2 launch morla_fr3_driver fr3_driver.launch.py \
  resolved_path:=/tmp/fr3-tutorial.resolved.json
```

终端 2（source 相同环境，设置相同域）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=126 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 docs/testing/fake_ros_gateway_smoke.py \
  --resolved /tmp/fr3-tutorial.resolved.json \
  --node /fr3_driver --component arm --require-joint-state
```

域 126 仅为示例，先确认没有其他该域会话；不要通过清除别人的资源锁来获得测试环境。只对 fresh、单执行器、明确 fake 的快照运行此脚本。它会激活 fake、发送小目标并做拒绝测试；不是实体机械臂验收脚本。完成后结束终端 1 的驱动，确认可再次启动并释放资源。

应看到六轴 JointState、有效命令被接纳，重复/未知组件、急停锁存、复位未激活命令被拒绝。读到了 topic 名字不等于完成 DDS 验收。

### D. Web 与遥操

结束独立 ROS 驱动、确认没有另一个工作站占用设备后，在已有 Web 服务中载入 FR3 预设，或启动独立示例：

```bash
ros2 launch morla_fr3_driver fr3_web.launch.py
```

页面默认 fake，启动为待使能。依次验证预设、配置检查、依赖检查、保存、启动、准备、激活、暂停和停止。

fake Quest 不自动产生人手位姿。没有新鲜输入时拒绝激活是正确行为。要验证端到端遥操，使用已有 FR3 Web 专项测试中的合成输入，或像[主机验收记录](FR3_HOST_ACCEPTANCE_2026-11-05.md)那样在隔离 DDS 域发布合成 Quest v3 帧；实体 Quest 的接收和连接另行验收。不要把跳过输入门控当成接通。

### E. 数据与普通安装

从页面开始/停止录制后检查 meta.json、robot_data.h5：任务、完成状态、state/action 名称、维度、来源和时间字段必须与冻结快照一致。accepted/dispatched command 不等于实体已到达，也不等于 Adapter 每个中间伺服点。

FR3 本轮六维状态 1183 条、六维动作 2450 条，说明两类流写入成功，不证明训练前的对齐质量。没有实际回放/模型实验，就不能将“可以复用统一网关”写成“模型效果通过”。

再使用独立 build/install 目录做一次非 symlink 的 FR3 包构建，切换到 `/tmp` 验证模块和 package share 来自安装目录；不要靠源码 cwd 或临时 PYTHONPATH 掩盖缺文件。具体主机结果见验收报告。

## 5. 测试要分别证明什么

- **契约测试**：乱序/缺失名称、NaN/Inf、错误模式/单位、超限、非法部署；在设备连接前拒绝非法配置。
- **SDK 边界测试**：返回码、短帧、重复帧、回绕、旧样本、通信错误、阻塞调用超时；测试替身必须标明替代范围。
- **生命周期测试**：默认 disarmed、prepare 副作用、激活新反馈确认、清除旧目标、release 与 stop 区别、软暂停后 close、资源复用。
- **运动学测试**：已知 FK 位形、Jacobian 有限差分、小范围 IK、不可达/限位/近奇异、耗时和跳变；与 SDK/实机比较另列。
- **组合/Web 测试**：注册实际工厂、冻结 6 维契约、输入→controller→Adapter、无输入拒绝、录制、停止与重启。
- **ROS 安装/DDS 测试**：不能用 Python mock 代替。必须在目标 ROS 主机确认包识别、IDL、服务、消息/QoS和退出清理。
- **浏览器测试**：不能用 HTTP 200 代替目视/按钮操作。尤其检查字段只读、正确 controller 选项、错误提示和真实诊断数值。
- **实机测试**：停止/断网、方向/零位/TCP、负载、实体输入、长稳等，不从 fake 推导通过。

本实例入口：`bash scripts/test_fr3.sh`、`bash scripts/test_composable.sh --layer pure`、`bash scripts/test_composable.sh --layer fake`，以及 Web 目录的 `npm run build` / `npm run test:workcell`。本轮主机最终 pure 338 项、fake 28 项、前端 18 项通过；数字有包含关系，不是新设备的固定通过指标。

## 6. 本次踩坑：新成员应怎样定位

- **构建成功却 ros2 找不到包**：检查 `colcon list` 的包类型，再用 catkin_pkg 校验 package.xml。本轮是非法维护者邮箱；不要只增加 AMENT_PREFIX_PATH 绕过问题。
- **catalog 有设备但实际未加载自定义驱动**：检查 adapter entry point、实际工厂类和安装 metadata，避免通用 fake 回退掩盖注册错误。
- **Web launch 报 unrecognized --ros-args**：普通 argparse 程序用 ExecuteProcess；真正 rclpy 节点才用 Node。空可选参数不要生成只有参数名而无值的命令。
- **fake 模板已启动却不能遥操**：先看是否有新鲜输入、是否已完成重锚、deadman 是否配置/按住，而不是改机器人的 IK。
- **SDK 每次 read 都成功却断流不报错**：检查实际帧时间是否变化；缓存读取的当前时间不能充当采样时间。
- **有解但页面显示等待目标**：检查诊断结构。FR3 公共 controller 是嵌套 solver_report，CR5 历史报告为扁平；先统一公共展示，不复制品牌特例。
- **Ctrl+C 出现 shutdown 错误**：区分功能异常、context 关闭竞态与后台 shell 的信号行为。保留退出码/日志，验证资源释放和再次启动，不能只删除 traceback。
- **脚本测通就宣称支持仿真/实机**：按第 1 节逐能力登记，sim 未提供就 unavailable，不把 fake 或 FK 数学模型当成物理仿真。

## 7. 从“这次接通”到“方便别人移植”，框架还应改进什么

以下保留最初的改进建议。第 1 项已于 2026-11-06 落实，同时完成共享仿真资源声明和后端目标能力校验；其余事项仍需独立设计，不能视为本次已完成。具体范围见 [改造规范](PLUGIN_DRIVEN_PORTING.md)。

1. **优先去掉 Web 品牌白名单和散落的预设登记。** 设备包应能声明显示名、可用 controller/kinematics 能力、模板及默认绑定；Web 根据已安装能力发现，不再每加一台机械臂就修改前后端多个列表。只靠 ports 语义还不够，需要证明存在对应 provider。
2. **提供统一的“插件安装自检”。** 同时验证 ROS 包类型、静态描述、Adapter 实际类、kinematics、安装资源和编译示例；明确指出 fake 是否回退，发现阶段是否导入 SDK。它比仅检查 catalog 更接近成员需要的成功标准。
3. **提供机械臂脚手架与契约测试套件。** 生成包结构、注册、fake 示例和测试入口；成员只填写关节、通信与生命周期。进程隔离/线程通信作为可选实现，不自动套用 FR3 SDK 私有 API。
4. **收敛公共诊断与生命周期规范。** 固定报告层次、有效性、错误/停止确认语义，保留扩展字段；避免每个设备都让页面适配一次。通用进程 worker 若抽取，先用 CR5/FR3 等多种 I/O 行为验证。
5. **明确运动学的可替换层。** 模型供应、算法策略、遥操调度分别注册。可以共享 URDF/Pinocchio 或通用 DLS 的机制，但不能把 CR5 参数当作另一机器人的模型，也不能把基础 DLS 与成熟微分控制策略混称“通用 IK”。
6. **单独设计多实例与多会话验收。** 先验证同品牌多实例的 SDK 状态隔离、资源键、输入绑定和分别停止，再验证跨品牌组合、控制组故障范围；独立会话并发需要先明确 host lane 与授权模型，而不是删除锁来证明“多机器人”。
7. **将验收状态按能力呈现。** 目录显示“可发现、可编译、fake 已测、sim 已测、ROS 已测、实机已测”等证据，而不是一个 available=true 表示全部可用。

判断设计是否便于多机器人接入，可以用一个具体目标：**下一位成员完成执行器包、注册和配置后，不应该还要在输入、录制、学习、Web 页面里逐处寻找品牌分支。** 当前 Web 和共享仿真已提供声明式扩展路径；复杂模式、多会话、硬件校验仍有独立边界。

## 8. 成员交付与维护

复制[移植任务与验收模板](ROBOT_PORTING_WORKSHEET.md)建立自己设备的工作单。评审时至少提供：

- 一个能安装/发现的独立设备包，SDK 来源/许可证和版本固定方式；
- canonical→SDK→模型的映射、时间/来源契约、明确的生命周期和停止限制；
- 最小 fake Assembly、分离的真实 Deployment 占位模板；
- 按需要提供运动学/遥操、Web、sim 能力，逐项注明未支持或未验收；
- 可复现测试、ROS 主机证据、浏览器证据、真实数据文件检查结果；
- 当前提交与安装版本、启动/停止/更新/回退说明，以及下一步现场验收项。

代码考古建议从 `6fa0507`（首次 FR3 实现）和 `063168d`（目标机与浏览器修复）分别看 diff。厂商原 SDK 和自动生成的前端 dist 占较多行，评审应先看手写边界、注册和测试，再核对来源摘要，不把新增行数当成移植工作量。

### FR3 当前平移方向更新

2026-11-06 按操作要求确定 Quest 平移映射为右移→FR3 −X、上移→+Z、前推→−Y（`input_position_rotation: [[-1,0,0],[0,0,-1],[0,1,0]]`，det −1；同日早先的 +X/+Y/+Z 版本已被取代）。姿态 `input_rotation` 必须写成**同一个矩阵**：旋转是物理量，只有用同一基变换搬运（`ΔR_base = P·ΔR_quest·Pᵀ`）才与平移一致；早先沿用的 `[[0,0,-1],[1,0,0],[0,1,0]]` 会让手腕俯仰变成工具自转（现场表现为 yaw 对、pitch/roll 互换），已更正。配置和旧 revision 升级见 [仿真说明](FR3_SIMULATION.md#quest--fr3-平移与姿态映射2026-11-06)。
