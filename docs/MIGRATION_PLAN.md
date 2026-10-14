# MORLA 统一机器人平台迁移执行计划

版本：1.0；编写日期：2026-10-13（Asia/Shanghai）。

本文是基于本机源码核对形成的执行规格。后续执行者应直接按本文的接口、文件地图、阶段与验收条件实施；不需要重新通读两个项目。实施时仍须阅读正在修改的函数并检查当前版本差异，不得把本文中的“目标实现”误认为已存在。

本文定义迁移目标、源码基线、包边界、ROS 数据契约和分阶段验收。执行进度与本次实测证据见 [`migration_progress.md`](migration_progress.md)；计划里的阶段状态以进度文件为准，历史分析段落描述的是建立计划时的源仓库状态。

## 1. 目标、范围与已经确定的决策

1. 在 `/home/robot/loopkok/sdk/MORLA` 新建独立 ROS 2 工作空间，先复制当前 `astral_ws` 的完整源码、配置、脚本和文档，再在副本上改造并迁入 `xnero_ws` 的 Nero、XHand 与仿真能力。两个原项目保持原样。
2. 大写 `MORLA` 是项目/工作空间名称；ROS 包使用小写下划线名称。MORLA 不是一个装载所有源文件的单体 ROS 包。
3. 统一采集包为 `morla_data_collect`，以 `astral_data_collect` 为唯一实现基础；不并行保留两套在线采集器和两套转换管线。
4. 统一上层输入包为 `morla_input`，以 Astral 版 `quest3_hand_mocap` 为基础。Quest、手套等来源经它的统一输入契约进入下游；机器人专属手型修正放在机器人适配侧。
5. 统一网页管理包为 `morla_web_manager`，以 `astral_web_monitor` 为基础，管理启动、控制、采集、数据处理、策略、回放和监控。网页调用 ROS 接口，不直接调用机器人 SDK。
6. 统一外部策略接口为 `morla_policy_inference`，以实际存在的 `astral_policy_inference` 为基础。用户所写 `astral_polciy_inference` 对应此包，不创建拼写错误的另一个包。继续支持 remote / inproc / stub，以及 policy / playback / HUMAN。
7. 新建一个实际 ROS 资源包 `robot_description`，统一安装 URDF、MJCF、mesh、RViz、描述生成工具及资产清单。机器人专属运动学参数、硬件配置仍属于各机器人包。
8. 所有机器人专属包放在 `src/robots/`，Astral、Nero、Wuji、XHand 各有独立配置/入口包；共享包放在 `src/common/`。已有成熟的 Astral/Wuji 驱动与 IK 包保留原包名，只改路径、描述引用和共享依赖。
9. 增加共享纯逻辑包 `morla_core`，承载冻结 schema、配置校验、关节映射和控制权租约。采集和推理共用一份布局；遥操不能再依赖“数据采集包才能获得控制权逻辑”。
10. `morla_bringup` 从 `astral_teleop` 改造，负责统一装配。机器人接入采用 profile 注册；不让 Web、采集、推理分别维护机器人判断分支。
11. 第一版只允许一个 ROS domain 中运行一个活动 robot profile。多机器人同时控制另设后续项目，不能在本次迁移中混用相同全局话题。
12. 支持现有 Astral 配置组合与 Nero/XHand 单侧、双侧配置。完整维度可采集、可回放；能否推理由具体模型的布局和维度声明决定。
13. 本次不训练新模型、不改原工作空间、不修改外部 OpenPI/LeRobot 仓库、不自动构建/安装 Quest APK、不把 XNero 仿真升级为全动力学机器人。训练侧支持扩展在 MORLA 内通过新配置/适配器完成。

## 2. 源码基线和真实项目对比

### 2.1 基线

源码根：`/home/robot/loopkok/sdk`，下文所有源路径都相对于此目录。

| 项目 | 基线提交 | 提交时间 |
| --- | --- | --- |
| `astral_ws` | `3a57d722d3c4257670d9f90526127bed77063e66` | 2026-10-12 18:24:03 +08:00 |
| `xnero_ws` | `e6a0100429024f04e18238f1a42ee968bb18967a` | 2026-10-12 17:56:51 +08:00 |

分析时两仓库 `git status --short` 均无输出；ROS 安装为 Humble。配套 `MORLA_SOURCE_BASELINE.json` 列出包清单、关键源码 SHA256 和定位符。本文以源码优先，README/CLAUDE.md 中历史段落不能覆盖当前实现。

### 2.2 架构对比

| 项目维度 | Astral 当前实现 | XNero 当前实现 | 迁移结论 |
| --- | --- | --- | --- |
| 机械臂 | Astral 单侧 7 关节，整机反馈 18 维；独立 `astral_robot_control` 驱动 | Nero 单侧 7 关节，`nero_teleop_node.py` 直接调用 `pyAgxArm` | 新建 Nero 独立驱动，遥操只发统一命令 |
| 臂 IK | `geometric`、`analytic_dh`、`urdf_numerical`，不同关节/基座约定 | 自有 Modified DH 解析 IK；数值 URDF 路径部分指向不存在的 `agx_arm_description` | 保留各自 solver，不用 Astral solver 替换 Nero 几何 |
| 手 | Wuji 20 关节，标准 `JointState` command/state | XHand 12 关节，`XHandCommand` / `XHandStateArray` | 增加 XHand 标准接口适配，底层原生消息保留 |
| 输入 | mixed/IOBT、手柄 Joy、frame latch、包序号/时间保护、raw/MANO 两模式 | 较旧接收器；腕位姿保留 Unity 坐标，landmarks 做 X 翻转、MANO 和小指缩放 | 只迁入 Astral 接收器，重新适配 Nero 坐标和 XHand 预处理 |
| 启停 | start/reanchor、disarm、HOME、工作位、ACK/租约/HITL token | 握拳触发全局 armed；disarm 会让臂回初始位，退出还会自动收臂和手 HOME | 采用 Astral 控制语义；删除迁入路径中的隐式回位 |
| 数采 | 各原始流异步独立时间戳，schema 冻结，离线 pipeline、质量验证、介入标签 | 50Hz 抽样 latest cache，缺源填零，固定臂/手布局，独立相机进程 | 使用 Astral 采集架构，扩展 XHand/触觉/TCP，旧数据单独导入 |
| 网页 | FastAPI + rclpy 线程 + React/Vite；已有遥操/数采/推理独立泳道与离线 job | 没有等价统一管理包 | 改造 Astral Web 的静态话题/设备/服务表 |
| 策略 | 共用 CollectSchema，后端抽象、动作块拆分、绝对动作执行、HITL | 没有等价框架；旧 `replay_episode.py` 可直驱 CAN | 只用统一策略/回放执行链，旧直驱回放不作为 MORLA 入口 |
| 仿真 | Astral 臂运动学、Wuji 手独立仿真 | XNero 手 `mj_step` 物理 + 臂 qpos 运动学覆盖，1200 个触觉测点 | 迁入 XNero sim，换统一上游接口；保留其物理语义 |
| 描述资源 | 多 description 包，teleop/retarget/sim 内也重复放模型 | `xhand_nero_description` 加 retarget 和 sim 内资产 | 归并到一个 `robot_description`，逐一改安装和引用 |
| 配置接入 | 各节点 yaml + launch 参数、Web presets、采集/推理 schema 参数 | CAN/固件/串口/左右姿态在多个 launch 与 yaml 中 | profile 统筹；底层参数名、已校准值与节点名匹配规则保留 |

### 2.3 已核实的差异与不可照抄的地方

**A. 触觉尚未被 XNero 采集。** `xnero_mujoco_sim` README 写“采集一行不改可录触觉”，但 `nero_dual_data_collect/data_collect_node.py::_hand_state_callback` 只读 `hand_states[0].position[:12]`，`data_writer.py` 只有关节、TCP 和 timestamps，没有 `sensor_states` 数据集。因此触觉落盘必须作为新增实现和验收项，不能宣称已经有。

**B. 维度要按代码计算。** 现有 schema 的块顺序是：启用的臂侧（按 arms 列表顺序）→ 对应末端 → waist → head。按此规则：

| 配置（不含腰头） | state/action 维度 | 标准 pi0.5 32 维容量 |
| --- | --- | --- |
| Astral 单臂 + 夹爪 | 8 | 可容纳 |
| Astral 双臂 + 双夹爪 | 16 | 可容纳 |
| Astral 单臂 + Wuji | 27 | 可容纳 |
| Astral 双臂 + 左夹爪 + 右 Wuji | 35 | 不可直接容纳 |
| Astral 双臂 + 双 Wuji | 54 | 不可直接容纳 |
| Nero 单臂 + XHand | 19 | 可容纳 |
| Nero 双臂，无手 | 14 | 可容纳 |
| Nero 双臂 + 双 XHand | 38 | 不可直接容纳 |

旧文档把“双臂+双 Wuji”写成 38，实际为 `7*2+20*2=54`。腰/头各另加 2。数据层不设置 32 维上限；推理连接和模型导出适配器应明确拒绝超出模型容量的配置。不得截去最后 6 维以凑 32；完整 XNero 采用支持 38 维的模型，或单独显式定义降维布局/模型，不在本次默认实施中猜测降维。

**C. 默认配置并不天然相同。** 当前 Astral 数采 yaml 为 `arms=[left]`、左夹爪、`cameras=[base,left_wrist,right_wrist]`；推理 yaml 是左臂夹爪和两相机 `[base,left_wrist]`。full_teleop 的常见整机配置为双臂+左夹爪+右 Wuji。旧 CLAUDE.md 有两相机的历史叙述。MORLA 必须提供不同命名 profile，并让三条泳道都读同一 profile，不能复制三个 yaml 后认为它们已一致。

**D. 有硬件二进制限制。** 当前 XHand SDK `xhand_control/lib/libxhand_control.so` 是 x86-64 ELF。ARM64/Jetson 部署必须获得/构建匹配的 SDK 库并检查 ABI、动态依赖与 RPATH；没有库时该目标硬件验收记为未完成，不能通过改路径声称支持。

**E. Quest 的防重放兼容例外仍开启。** Astral 接收器有 `PacketGuard`，但当前 yaml `allow_legacy_packets=true`，旧 APK 模式没有源时间、sequence/session 保护。迁移保留接收能力，但将旧包模式做成显式兼容 profile/开关并在 Web 标示降级；常规新 profile 默认 false。没有新 APK 时可以用显式 legacy 模式实测，不能报告严格包保护已在设备端验收。

**F. 同名 `joint_states` 未必是真实反馈。** XHand retarget 的 `viz=true` 会在 `/{side}_hand/joint_states` 发布计算目标；它不等于驱动反馈。迁移后该话题只能由适配器从驱动/仿真实际状态生成，目标可视化另用 `/{side}_hand/retarget_targets`。

**G. 策略目前依赖弱位置约定。** 当前 policy `_publish_targets` 不填 `JointState.name`，采集回调直接按 `position` 截取，Astral driver `pack_named_positions` 对缺名/错名有填零和位置回退。MORLA 的新通道必须显式名字、严格维度、拒绝坏映射；历史兼容只在具名适配器内开启，不扩散到通用控制层。

**H. 现有 LeRobot 回放会猜错手类型。** `astral_policy_inference/replay.py::_parse_schema_from_info` 见到任何 `*_ee_*` 名字都按 gripper 重建schema，并未按20维还原Wuji。新导出必须保存完整冻结schema，回放优先读取它；历史数据须按明确schema或可验证的旧格式升级，禁止把12/20维手误读为1维夹爪。

## 3. 目标结构和包迁移表

### 3.1 目录结构

```text
MORLA/
  README.md
  CHANGELOG.md
  docs/
    MIGRATION_PLAN.md
    SOURCE_BASELINE.json
    migration_progress.md
    interfaces.md
    profile_reference.md
    hardware_acceptance.md
  scripts/
    data_pipeline.sh
    serve_policy.sh
    build.sh
    check_profiles.py
    check_runtime_graph.py
  tests/integration/
  src/
    common/
      morla_core/                  # 纯逻辑 schema/配置/关节/租约，无 SDK、无模型重依赖
      morla_input/                 # Astral Quest 接收器 + 输入来源编排
      morla_bringup/               # 总 launch + common controller gates
      morla_data_collect/          # 在线录制 + 唯一离线 pipeline
      morla_policy_inference/      # backend/engine/executor/回放/HITL
      morla_web_manager/           # FastAPI/ROS 桥/React
      quest3_video_streamer/       # 保留包名，统一相机所有者
      wuji_glove/                  # 保留 SDK 接入包；由 morla_input 装配
    robot_description/            # 单个 ament_cmake 包，内部不再嵌套其他 ROS 包
      urdf/{astral,nero,xhand,wuji}/
      meshes/{astral,nero,xhand,wuji}/
      mjcf/{astral,xnero,wuji}/
      rviz/
      launch/
      scripts/
      config/asset_manifest.yaml
      docs/
    robots/
      astral/
        morla_astral_robot/        # profile/机器人入口/能力声明；独立 ROS 包
        astral_arm_teleop/
        astral_gripper_teleop/
        astral_robot_control/
        astral_mujoco_sim/
        astral_pim_ik/
      nero/
        morla_nero_robot/          # profile/机器人入口/能力声明
        nero_arm_teleop/           # 从 nero_quest_teleop 拆分，保留 Nero IK
        nero_robot_control/       # 新驱动包，唯一 pyAgxArm 硬件写入者
        xnero_mujoco_sim/
      wuji/
        morla_wuji_hand/           # 手组件元配置和能力声明
        wujihand_retargeting/
        wujihand_control/
        wujihand_mujoco_sim/
        wujihandros2/              # driver/msgs/bringup 三个已有子包
      xhand/
        morla_xhand_hand/          # 手组件元配置和能力声明
        xhand_retargeting/
        xhand_adapter/            # 新包，标准 JointState ↔ 原生 XHand 消息
        xhand_control_ros2/        # 原供应商目录内两个现有 ROS 子包
  third_party/
    pyAgxArm/                     # Python SDK，不伪造 ROS package.xml
  dependencies/                   # 安装说明、版本和架构锁定清单
```

`robots/astral` 等只是分类目录，不放 package.xml。其下各实际包是兄弟目录；禁止在一个有 package.xml 的父包里再埋其他包，防止 colcon 漏发现。`robot_description` 内只放资源，旧 description 包的 manifest/CMake 不作为子包复制进去。`pi05_standalone` 等非 ROS 辅助内容复制基线时保留，后面移入 `tools/legacy/` 并标记 `COLCON_IGNORE`，不成为另一套在线策略入口。

### 3.2 明确的源包 → 目标映射

| 来源 | 目标 | 处理方式 |
| --- | --- | --- |
| Astral `quest3_hand_mocap` | `common/morla_input` | 重命名目录、Python module、resource、setup、manifest、导入和 launch；保留已有 packet/frame 防护 |
| Astral `astral_data_collect` | `common/morla_data_collect` | 重命名；schema/ownership 逻辑抽到 core；扩展流描述与传感输入 |
| Astral `astral_web_monitor` | `common/morla_web_manager` | 重命名；保留 REST/WS 与 job 管理；改 profile 驱动的 UI/ROS/config |
| Astral `astral_policy_inference` | `common/morla_policy_inference` | 重命名；layout 读 core；扩展握手/名字/可写块/动态 writer |
| Astral `astral_teleop` | `common/morla_bringup` | 控制闸门移为 common，头部节点保留并由 Astral profile 条件启动；重构 full_teleop |
| Astral arm/control/gripper/sim/piM-IK | `robots/astral/` 同名包 | 保留算法、SDK、硬件参数；更改路径/描述解析/core 依赖 |
| Astral Wuji retarget/control/sim/vendor | `robots/wuji/` 同名包 | 保留 20 关节契约；所有权抽 core；迁资产引用 |
| Astral `quest3_video_streamer`、`wuji_glove` | `common/` 同名包 | 保留视频/相机契约；统一输入控制从 morla_input 装配 |
| 三个 Astral description 包 | `robot_description` | 分目录存模型，保留原模型区别，不互相覆盖 |
| Astral teleop 内 `robot/`、retarget 内 `urdf/`、sim `assets/` | `robot_description` | 去运行时多份模型；更改默认查找和安装方式 |
| XNero `nero_quest_teleop` | `robots/nero/nero_arm_teleop` + 新 `nero_robot_control` | IK/pose/safety 与 SDK 生命周期拆分；不是只重命名 |
| XNero `xhand_retargeting` | `robots/xhand/xhand_retargeting` | 保留 DexPilot、左右名表/符号处理；标准命令输出、输入局部预处理、租约 gate |
| XNero `xhand_control_ros2` | `robots/xhand/xhand_control_ros2` | 保留接口包名和供应商驱动；原生命令改内部话题；显式控制动态库架构 |
| XNero `xnero_mujoco_sim` | `robots/nero/xnero_mujoco_sim` | 保留物理/触觉模型，臂标准接口、手原生内部接口；迁资产引用 |
| XNero `xhand_nero_description` + XHand retarget URDF + sim MJCF | `robot_description` | 分目录聚合，改生成工具与相对 mesh 路径 |
| XNero `pyAgxArm` | `third_party/pyAgxArm` | 保留源码/许可证，显式 pip editable 安装，不通过 sys.path 偷找旧仓库 |
| XNero `quest3_hand_mocap` | 不引入第二份 | 它的 MANO/小指处理作为数值兼容参考，迁到 XHand 适配函数 |
| XNero `nero_dual_data_collect` | 不引入在线实现 | 只提取旧格式导入映射和参考算法；新增 pipeline/import_xnero.py |
| XNero `参考文件` | `docs/legacy/xnero/` 中必要说明 | 不参与编译，不把旧调试/硬件回放脚本加入生产入口 |

重命名是完整原子任务：必须覆盖 `package.xml/<name>`、Python目录与 import、resource 文件名、setup.py/name/data_files/entry_points、setup.cfg 安装路径、launch 中 package/executable/get_package_share_directory、测试、脚本、环境变量、前端文字和 README。节点运行名与 ROS 话题不等于包名；不要用全仓字符串替换顺便改坏 yaml 节点匹配键。

迁移后的旧名允许出现在历史文档、SOURCE_BASELINE 和明确的旧 schema 读取器；不得出现在活动 import、launch package、默认模型路径、离线 CLI module、Web 子进程路径中。

## 4. 统一配置接入：保留 Astral 已有参数，用 profile 消除多份布局

### 4.1 所有权和依赖方向

```text
morla_core ← morla_input / morla_data_collect / morla_policy_inference
           ← 各机器人 teleop / driver / adapter / bringup / web_manager
robot_description ← 各机器人包（ament share 资源查询）
机器人 profile ← morla_bringup / data_collect / policy / web 的共同入口
driver/adapter → 实际 SDK；Web/采集/策略不得直接 import SDK
```

core 不 import rclpy、硬件 SDK、torch、OpenPI 或 ROS 自定义消息。`PolicyLease.bind` 目前内部 import ROS；抽取时把纯 lease 与 ROS bind 桥分开（桥放 core 的独立可选模块，import 纯逻辑时不加载它）。保留现有行为测试。

### 4.2 profile 文件的唯一职责

新增 `morla_core/profile.py`、`schema.py`、`joint_mapping.py`、`ownership.py`、`resource_locator.py`。

机器人 profile 位于 `morla_astral_robot/config/profiles/*.yaml` 和 `morla_nero_robot/config/profiles/*.yaml`；手组件模板位于 `morla_wuji_hand`/`morla_xhand_hand`。注册索引为 `morla_bringup/config/robots.yaml`，每项只记录 profile_id、所属 ROS 包、资源相对路径、标题。通过 ament share 定位，不扫描旧仓库。

组件模板提供名字/映射/限位/资产默认值；profile 组合组件，写明输入、采集与策略布局、相机、控制 capability 和启动依赖。解析后必须生成一份完整 `resolved_profile.json` 与 `frozen_schema.json`；所有泳道共用解析结果，不在各节点再次推断组件类型。

必须提供这些初始 profile：

| ID | 组合 | 目的 |
| --- | --- | --- |
| `astral_left_gripper` | 单左臂+左夹爪；base/left_wrist；8 维 | 默认可推理配置，对齐现有单臂能力 |
| `astral_left_gripper_3cam` | 同上，显式三相机 | 保留当前数采 yaml 的实际组合 |
| `astral_right_gripper` | 单右臂+右夹爪 | 右侧对称接入 |
| `astral_dual_gripper` | 双臂+双夹爪；16 维 | 保留现有预设 |
| `astral_dual_gripper_wuji` | 双臂+左夹爪+右 Wuji；35 维 | 整机遥操/完整采集/适配模型推理 |
| `astral_left_wuji` / `astral_right_wuji` | 单臂+同侧 Wuji；27 维 | 通用末端配置与回归 |
| `astral_dual_wuji` | 双臂+双 Wuji；54 维 | 大维度 schema 回归，硬件按可用性验收 |
| `nero_left_xhand` / `nero_right_xhand` | 单侧 Nero+XHand；19 维 | Nero 首批验收配置 |
| `nero_dual_xhand` | 双 Nero+双 XHand；38 维 | 完整 XNero 迁入 |
| `nero_dual_arms` | 双 Nero，无手；14 维 | 臂独立测试与隔离故障 |

`mode=real|sim|dry_run`、`input_source`、`with_video` 是启动选择，不复制三份相同布局。Quest strict/legacy 是明确输入兼容开关。Glove 初版保留 Astral/Wuji 支持，XHand glove 配置只有经过数值测试后才注册；不因为同样 21 点就默认可用。

### 4.3 resolved profile 的强制字段

| 字段 | 类型/约定 |
| --- | --- |
| `profile_version`、`profile_id`、`robot_id` | 初版版本为 1；稳定 ID；显示名不作为 ID |
| `components` | 每项含 component_id、enabled、side、type、ROS/node 参数文件和资产引用 |
| `streams` | 每项 stream_key、topic、message_type、shape、dtype、joint_names、unit、QoS、timestamp_mode、required、source_kind |
| `state_blocks` | 显式有序块，含 block_id、stream_key、joint_names/feature_names、dim、unit、state_indices |
| `action_blocks` | 显式有序可写块，含 action_indices、command_stream、语义、限位/限速、writer_id；与 state 可分开 |
| `cameras` | 有序 label；参考相机为第一项；硬件指纹/slot/image_size 在同一配置内显式绑定 |
| `capabilities` | ready/enable/position/damping/estop/home/workpos/reanchor/tactile 等是否支持和具体路由 |
| `writers` | 活动遥操写入者 ID、所属 component、命令通道、ACK、prepare_reanchor 服务（适用时） |
| `backend_requirements` | 模型名、模型布局 ID/版本、维度容量、camera_map、absolute 语义、可选传感器 |
| `data` | dataset_fps、action_source、hold_frames、max_gap_ms、数据根目录 |
| `hardware` | Astral 板卡、Nero CAN/固件、Wuji 序列号、XHand 串口/hand_id；不进入训练布局 hash |

启动前校验：ID 唯一、topic 类型唯一、每块维数和索引覆盖一致、名字无重复/缺失、末端属于启用侧、流 unit/shape 一致、action 全可写、资产可加载、相机槽名存在、依赖及模式一致。head/waist 在 Nero 不允许注册为空假设备。

配置优先级：组件默认值 → profile → 显式非空 launch 覆盖。UI 调整硬件地址/任务不改变布局；改变 arms/末端/相机/单位/块顺序必须切换 profile，停止活动泳道并重新解析。所有节点日志输出 profile_id、schema_hash、实际参数与配置来源。

节点参数 yaml 顶层键须与实际节点名一致。第一轮重命名保留 `policy_node`、`data_collect`、`quest3_udp_mocap` 等节点名；Web 节点可改为 `morla_web_manager` 并同步相关键。旧 Astral/Nero 已校准参数文件作为配置事实迁入，不用 README 的默认值覆盖它们。

### 4.4 schema 版本与兼容

从现有 `CollectSchema` 迁为 `FrozenSchema v2`；公开兼容入口 `CollectSchema` 可保留在 core 中。

- v2 保存 robot/profile、显式 streams/state_blocks/action_blocks、每块语义、物理关节名、稳定 feature_names、limits、传感描述、相机与时间契约。
- 读取 v1 历史 Astral meta 时用 `upgrade_v1_schema()`，保留原 arms 顺序、所有逐维名字（如 `left_arm_0`、`left_ee_0`）、单位、相机、next_state/command 含义，不在历史文件上原地改。
- 新同构 Astral 数据继续沿用旧逐维 feature_names；physical_joint_names 是单独字段。现有 OpenPI 通过 `_ee_` 识别末端 delta mask，不能在迁移中偷偷把 `left_ee_0` 改成另一名字。
- hash 使用 canonical JSON/SHA256。`schema_hash` 覆盖冻结的流/布局/采集语义；另有 `layout_hash` 只覆盖模型相关的顺序、名字、单位、动作语义与传感定义，避免仅硬件地址或数据根变化导致误拒绝。
- 记录 state 与 action 的独立维数/名字；默认本次各臂+手配置两者同序同维。腰等只读块不得被默认塞进 policy action。历史带不可写腰块的回放必须报 `UnsupportedActionBlockError`，不能绕过。
- `next_state`：`action[t]`使用同一对齐网格的`state[t+1]`对应目标块；终帧/合成 hold 延续既有管线规则。遇无效下一帧标记质量失败，不能偷偷跨空洞跳到更远“有效帧”。`command`：从各块实际命令流取样。只读块或无 cmd 块的历史 fallback 必须逐块标记 `next_state`，不能宣称整向量全为 command。
- `from_dict` 不得像旧实现一样默默丢掉新字段；未知版本/必填字段/单位错误必须拒绝。旧字段只通过具名升级器支持。

### 4.5 首批 Nero 双 XHand 布局的固定示例

`nero_dual_xhand` 的 state/action 默认均为下表顺序，索引为左闭右开；只改变硬件地址、固件或端口不会改变布局：

| block_id | state_indices / action_indices | state_stream | command_stream | dim / unit |
| --- | --- | --- | --- | --- |
| `left_arm` | `[0,7)` | `left_arm_state` | `left_arm_cmd` | 7 / rad |
| `right_arm` | `[7,14)` | `right_arm_state` | `right_arm_cmd` | 7 / rad |
| `left_ee` | `[14,26)` | `left_hand_state` | `left_hand_cmd` | 12 / rad |
| `right_ee` | `[26,38)` | `right_hand_state` | `right_hand_cmd` | 12 / rad |

臂名字分别为`left_joint1..7`和`right_joint1..7`，手名字为第6.3节完整12名表；feature_names依次为`left_arm_0..6`、`right_arm_0..6`、`left_ee_0..11`、`right_ee_0..11`。topic按第5.1节映射，无body/head/waist/gripper。传感附加`left_hand_tactile`、`right_hand_tactile`，不占这38维。

各block的limits、source_kind、writer和timestamp规则来自组件声明；profile解析后必须物化为完整值，不在运行中再猜测。`nero_left_xhand`删掉右侧block并重新排索引为arm[0,7)、ee[7,19)。Astral同构布局保持旧CollectSchema生成顺序。

### 4.6 配置与能力元包的实施边界

`morla_astral_robot`、`morla_nero_robot`、`morla_wuji_hand`、`morla_xhand_hand`为薄配置包：安装profiles/component模板和机器人入口launch，声明真实依赖，没有另写一套IK或数采。机器人硬件yaml通过组件模板引用，profile仅覆盖必要值；冻结结果物化全部值供溯源。

`morla_bringup`新增`control_coordinator_node.py`、`coordinator_logic.py`，提供第5.4节协调接口。core仅提供状态决策，不成为ROS节点；Web、手柄闸门和CLI统一调用协调入口。

## 5. ROS 接口与数据上下游契约

### 5.1 保留现有 Astral 话题路径，改为配置来源

第一次统一不批量改 ROS topic。所有组件以 resolved profile 中的绝对名为准；以后可 remap。

| 通道 | ROS 类型 | 约定 |
| --- | --- | --- |
| `/quest3/{side}_wrist_pose`、`/quest3/{side}_controller_pose`、`/quest3/head_pose`、`/quest3/hips_pose` | PoseStamped | m；quaternion xyzw；源戳；robot_world/robot_body 稳定父系 |
| `/hand_landmarks/{side}` | PoseArray | 21 点、腕局部、m，统一 raw 机器人轴；不含 MANO/小指手型修正 |
| `/quest3/{side}_controller_joy` | Joy | axes=[trigger,grip,stickX,stickY]；buttons=mask bit0..5 |
| `/quest3/body_joints`、`/quest3/body_joint_names`、`/quest3/input_mix` | PoseArray、String、String | IOBT/来源状态；body names/input_mix为latched |
| `/{side}_arm/joint_commands` | JointState | 本机器人硬件关节约定的绝对 rad；完整 name+position |
| `/{side}_arm/joint_states` | JointState | 驱动/仿真实测反馈；完整 name+position；源戳 |
| `/{side}_hand/joint_commands` | JointState | Wuji 20 / XHand 12 维，由 profile 指定名字/单位 |
| `/{side}_hand/joint_states` | JointState | Wuji driver 或 XHand adapter 实际反馈，非 retarget 目标回显 |
| `/{side}_gripper/command` | Float64 | ratio [0,1]，0=开、1=合；driver 管 rad 映射 |
| `/{side}_gripper/joint_states` | JointState | 当前 Astral 是指令 rad 回显；source_kind=command_echo，不宣称传感真值 |
| `/head/joint_commands`、`/head/joint_states` | JointState | 仅配置有头部时启用；2 rad |
| `/astral/joint_states` | JointState | 保留 Astral 18 维设备内部反馈，不强迫 Nero 发布假 18 维 |
| `/quest3_video_streamer/collect/{label}` | CompressedImage | 原 JPEG 采集 tap，label 来自同一 profile |
| `/data_collect/control`、`task`、`session`、`state` | String | 保留现有控制命令和状态 JSON |
| `/policy_inference/cmd`、`task`、`state` | String | 保留 policy/playback/pause/resume/takeover/release/stop 与状态 JSON |
| `/teleop/ownership_ack` | String JSON | writer_id + transition_id；沿用当前 ACK 格式后扩展 profile 校验 |

`side` 表示 left/right，数据块的末端字段沿用 `left_ee` / `right_ee`。不得按向量长度推断“1=夹爪、其它=手”，必须按 block type/command_kind。

### 5.2 QoS、名字与时间

- 标准实时 command/state、腕/Joy/landmarks/camera tap：BEST_EFFORT + VOLATILE + KEEP_LAST；命令和低延迟输入 depth=1。传感器写盘可使用有界接收队列，计 dropped，不无限缓冲。
- 一次性 `/teleop/start|home|init|init_direct`、数采 control、policy cmd：RELIABLE + VOLATILE。不能 latch 一次性启动/运动命令。
- `/teleop/armed|disarm` 保留现有电平接口的 latched QoS；policy state、配置/任务/状态镜像按现有 RELIABLE+TRANSIENT_LOCAL，收到后仍检查 epoch/sequence/有效期。
- XHand 底层原生通道当前是 rclcpp QoS(1)，默认 RELIABLE。adapter 原生命令 publisher 必须 RELIABLE，与 C++订阅者兼容。标准 BEST_EFFORT 命令不能直接 remap 到 RELIABLE 原生订阅；类型也不同。
- 所有新标准关节命令均写完整 `name`；接收者按配置名字重排，只接受 exact-name-set。重复/缺失/未知/非有限/错长度均拒绝并留诊断，绝不补零驱动机器人。兼容无名旧消息须显式 `allow_positional_legacy=true` 且精确维数，仅特定适配入口可开。
- 采集 command 流继续用 host arrival wall-clock 时间，避免遥操沿用 VR 戳产生重复 cmd 时间；state 和 camera 保留有效源 header.stamp。分别保存 `source_timestamp` 与 `arrival_timestamp`（新增可选数据列）以及 `timestamp_mode`，不能混淆。
- 看门狗用 monotonic；磁盘对齐用同一 wall-clock 域。真机/Quest/ROS host 必须时钟一致；仿真如使用 `/clock`，将整个采集会话显式声明为 sim 时间域，禁止把 ROS sim stamp 与 Unix wall-clock 混入同一次对齐。初版 sim smoke 可继续统一 host 时间。

### 5.3 控制动作语义

| 动作 | MORLA 含义 |
| --- | --- |
| pause/disarm | 撤销遥操输出资格，保持当前目标/驱动保持策略；不回 init，不归零，不 HOME 手 |
| resume | 只有合法暂停且输入/反馈新鲜才恢复；watchdog 后要求显式 start/reanchor |
| start | 使用当前新鲜输入和实际反馈重锚，合法所有权下启遥操 |
| init/workpos | 显式沿 profile 的已配置路径回工作位 |
| init_direct | 显式直接回工作位，仅 profile 允许且不在录制/保存中 |
| home | 显式执行该机器人 HOME 路径，与阻尼/急停不同 |
| stop stack | 撤销输出、清命令缓存、停止节点；不自动执行回零轨迹 |
| estop | 调机器人实际支持的急停接口；按 capability 报告完成/失败，不把暂停包装成真急停 |

所有硬件模式与 HOME 类操作都必须先通过共享控制协调入口冻结/停止 policy 和遥操并确认状态，然后才调用 driver。当前 Web “先发 disarm 后调用 driver”不足以暂停策略输出，迁移必须改为等待策略停止与相关 writer ACK，超时拒绝模式切换；急停走最高优先级通道，立即取消所有待提交 transition 并清缓存。

### 5.4 共享操作协调接口

在`morla_bringup`提供`/morla/control/{pause,resume,start,release,home,workpos,workpos_direct,ready,enable,position,damping,estop}`，类型统一`std_srvs/Trigger`。操作通过resolved profile的capability映射到原teleop话题/driver服务；不支持的capability立即失败，不发布运动话题。

服务response的`success=true`只表示请求已接纳，message携带生成的operation_id；完成结果通过`/morla/control/state`（String JSON、RELIABLE/TRANSIENT_LOCAL）公布，字段包含profile_id、operation_id、operation、phase（idle/accepted/freezing/waiting_services_ready/waiting_services/executing/completed/failed/cancelled）、每component结果、message、result、started_at、updated_at。Web必须区分“已接纳”和“已完成”。同一时刻只允许一个改变运动模式的协调操作；estop可抢占并取消它。异步调用/状态机轮询，不在ROS回调里sleep等归位。

pause在POLICY/PLAYBACK时转发policy pause并确认，在manual时disarm；resume按当前owner恢复，不只一律发armed。start在policy活动时转换为takeover；HUMAN/合法IDLE时才重锚manual。硬件ready/home/position/damping/workpos先停止策略、确认无pending、冻结manual，再执行；录制中拒绝home/workpos/模式切换，estop始终允许。原`/teleop/init*|home`消费者也检查所有权及录制保护，拒绝策略占有期间绕过协调入口的直接运动触发。

冻结确认复用policy transition/ACK和manual writer状态；新的协调状态须确认实际停止输出及新鲜反馈，不能只用`ros2 publisher count`（暂停writer的publisher仍在）判断。初版冻结超时2秒，运动超时由profile声明（默认15秒）；超时安全失败。driver底层模式服务亦检查当前所有权/活动操作，拒绝未冻结状态下的非急停模式改动。

## 6. 上层输入统一与坐标适配

### 6.1 morla_input 的职责

保留 Astral 接收器的 UDP/TCP_wired/TCP_wireless、多连接、`PacketGuard`、IOBT 启动 hold/frame latch、controller_as_wrist、Joy、body names/input_mix、延迟诊断和来源限龄。

统一默认 `convert_to_robot=true`、`landmark_preprocess=raw`、`enable_xhand_pinky_adapt=false`。`raw` 的真实含义是已完成 Unity→robot 轴转换并以 wrist 为原点，仍为 21 点，尚未做 MANO 或机器人指长补偿；不能误认为“原 Unity 数据”。mocap 的新 packet/session/frame 消息沿用原实现。

input_source 的 Quest/glove 来源选择由 morla_input launch 编排：同侧同一种统一 landmarks 只有一个来源发布；Glove 占用右 landmarks 时关闭 Quest 的对应 landmarks，不关闭该侧 wrist/Joy。输入设备不会决定末端是 Wuji/XHand/夹爪；profile 分开记录 `input_source` 与 `end_effector_type`。

控制按钮的事件闸门放 `morla_bringup`，不由 retarget 输出全局 armed。保留左 grip（start 或 takeover）、右 A（录制或 HUMAN release）、右 B（保存）、左 X（段间回位）与新鲜度/上升沿/录制保护规则；右 A 的 release 由常驻闸门负责，不依赖数采节点。

### 6.2 Nero 腕位姿矩阵的确定迁移

旧 XNero 腕直接使用 Unity pose；Astral 输入使用

```text
p_robot = M p_unity
M = [[-1,0,0], [0,0,-1], [0,1,0]]
R_robot = M R_unity M^T
```

旧 Nero yaml 的 `vr_to_arm_rot` 不能原样用于新输入。按 `A_new=A_old*M^T`，初始迁移值为：

```yaml
# nero 左侧
vr_to_arm_rot: [0.0, 0.0, 1.0,
                0.0,-1.0, 0.0,
                1.0, 0.0, 0.0]
# nero 右侧
vr_to_arm_rot: [0.0, 0.0, 1.0,
                0.0, 1.0, 0.0,
               -1.0, 0.0, 0.0]
```

两个旧配置均 `flip_pitch=false`。保留其状态；对任意 Unity 位姿增量，新输入+新矩阵须与旧输入+旧矩阵的末端平移/旋转增量等价。用六方向平移、绕三轴旋转、随机单位四元数、TCP 非零偏移分别验证，纯变换误差目标 `1e-8`（float64）；不能只测试平移。IOBT 则在 body 稳定系建立新锚，不能拿 world 初始 pose 与 body 后续 pose 做 delta。

frame/session 改变：disarm、清滤波/旧锚并要求重新 start；不把“frame 名字不同”当作可忽略。Nero 原自动第一帧校准改为和 Astral 一样显式 start/prepare_reanchor。

### 6.3 XHand 预处理及关节约定

把旧 Quest 的 MANO 预处理迁到 `xhand_retargeting/preprocess.py`，其输入是统一 raw，不再要求整个 mocap 全局切 MANO。Wuji 仍使用现有 raw 后端，不再被 XHand 小指算法污染；左右末端类型不同可同时运行。

必须保留：21 点顺序、1.05 缩放、以腕居中、SVD 帧估计、左右 `OPERATOR2MANO`、现有小指 stretch 的可配置兼容行为、滤波顺序。旧 landmarks 从 Unity 只做 `F=diag(-1,1,1)`，新 raw 做 M；兼容预处理先按 `old_points=F*M^T*new_raw_points` 重建旧输入轴，再执行旧算法。这样先做到同输入数值等价，再注册新的改进手型参数。

原小指算法逐级使用已移动的上游点，可能产生折线；初版用 profile 显式选择 `legacy_xhand_pinky`，默认 XHand 兼容 profile 保留原效果，Wuji 永不开。新的修正算法另配版本，不与框架迁移混在一次变更里。

XHand 硬件顺序固定为：

```text
thumb_bend_joint, thumb_rota_joint1, thumb_rota_joint2,
index_bend_joint, index_joint1, index_joint2,
mid_joint1, mid_joint2,
ring_joint1, ring_joint2,
pinky_joint1, pinky_joint2
```

标准 XHand ROS name 使用上述硬件名字，topic 命名空间区分左右；URDF/MJCF 名为 `{side}_hand_{name}`。右手 `index_bend` 在旧 retarget 中取反，左手不取反。标准 command/state 使用硬件数值约定，retarget 的 URDF→硬件符号只执行一次，不能 adapter 再翻一次。当前sim的`pack_hand_command`是数值identity+名称转换，没有额外取反；初版保留这个事实，若模型校验发现必须修正则在hand_map中具名定义双向映射并独立记录，不能先假设要再翻符号。反馈以驱动真实名字/hand_id 重排。手 id 不再一律硬编码 0。

## 7. Nero 独立驱动与机器人适配

### 7.1 nero_robot_control

从 `nero_teleop_node.py` 提取 SDK 连接、enable、motion mode、move_js、回读、急停、断开。新 `driver_node.py` 为唯一硬件写入者；`sdk_adapter.py` 包装已存在的 API 并可注入 fake SDK。

现有真实 API 调用事实：

```python
cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V111 if fw_version == "v111" else NeroFW.DEFAULT,
    channel=can_channel,
    interface=can_interface,
)
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
robot.enable()
robot.set_motion_mode("js")
robot.set_auto_set_motion_mode_enabled(False)
robot.move_js(q7)
robot.get_joint_angles()       # 数据容器 .msg
robot.get_flange_pose()       # xyz + rpy，不是 quaternion
robot.electronic_emergency_stop()
```

SDK 参数 `firmeware_version` 是现有拼写，禁止擅自“纠正”而使调用失效。

左右 CAN 名保留 `can_nero_left` / `can_nero_right`，profile 显式只连接启用侧。SDK 以 pip editable 安装 MORLA/third_party/pyAgxArm；driver 使用 lazy import，纯测试/仿真不用加载硬件 SDK。

初始命令与第一次 policy 输出都用新鲜实测 q 播种，禁止回读缺失时填零继续启动。connect/enable/首反馈必须有界超时和失败状态，移除旧 `while not enable()` 无限循环。状态发布独立于 IK/遥操，停遥操后仍可给推理和 Web 提供真实 q。需要可配置 control/state rate；先保留源 125Hz 遥操频率和稳定的反馈实现，再实测驱动频率，不能把旧 20Hz反馈伪报成125Hz。

Nero 标准关节名为 `left_joint1..7` / `right_joint1..7`，driver 转成 SDK 的有序7数组；旧 `joint1..7` 只用于历史映射。URDF 名表通过 manifest 核验。TCP 状态改为 `/{side}_arm/tcp_pose` PoseStamped，位置m，rpy转换为xyzw；header.frame_id 为实际 arm base。原采集把6维 xyz+rpy补成7维的路径不得迁入。

新 driver 提供 `~/enable`、`~/position`、`~/estop` 和状态/诊断；`ready/home/damping` 只有 SDK 和机械意义已核实后才提供。可统一路由层按左右 driver 分派 Trigger，不用假成功填齐按钮。一次操作双侧中一侧失败时返回整体失败和每侧结果，保留显式部分完成状态。

源Nero解析IK的限位（rad）为：`[-2.705261,2.705261]`、`[-1.745330,1.745330]`、`[-2.757621,2.757621]`、`[-1.012291,2.146755]`、`[-2.757621,2.757621]`、`[-0.733039,0.959932]`、`[-1.570797,1.570797]`。profile初值从该NeroParams生成并核对SDK/URDF，不用Astral限位。左工作位保留`[-0.405,1.281,-0.957,1.311,2.682,-0.314,-0.163]`；右为`[0.405,1.281,0.957,1.311,-2.682,0.314,-0.163]`。这些是源配置值，真实到点/安全路径仍须P11确认。

命令验收：精确7维、名字映射、有限值、限位、rad/s限速、反馈新鲜度、command_timeout。无命令/超时保持最近安全姿态并撤销移动资格，不回零。stop/急停/模式切换清除待发缓存，重使能从实况播种；SDK I/O 有专属串行通道，回调不阻塞等待归位。

### 7.2 nero_arm_teleop

保留 Nero `ik_solver.py`、`urdf_ik_solver.py`、`pose_processor.py`、`safety_filter.py` 与数学测试；替换 Node 的硬件依赖为标准反馈订阅和标准命令发布。

Node 增加/适配：`/teleop/start|armed|disarm|home|init|init_direct`、core lease、`~/prepare_reanchor`/manual `~/reanchor`、新鲜 wrist/真实 q、frame/session 重置、ownership ACK。`prepare_reanchor` 只准备，返回一次性 token，不立即武装。HUMAN 提交匹配 token 后才开始发 q。

旧 `disarm` 回位、destroy_node 的三段自动收臂和手 HOME 删除。显式 HOME/工作位轨迹用非阻塞状态机实现、从反馈判断到点；不能把“已发送 move_j”当作“已到位”。旧 Nero shutdown 末态是 J2=pi/2 并非全零；原轨迹移入待验证的 `park_pose/park_waypoints` 参数，不统一硬编码零位，也不默认自动执行。

Nero 原 `max_joint_vel=0.065` 是 rad/step，125Hz 理论对应8.125rad/s，不等于0.065rad/s。迁移先保留原算法参数和语义（可内部改名 `max_joint_delta_rad`），独立驱动/策略新安全限速统一rad/s并显式配置。不能把数字原封不动复制到不同单位的安全层。PoseProcessor alpha 的旧代码是 `alpha*old+(1-alpha)*new`，也不能按 README 的“1=无平滑”解释重写。

### 7.3 xhand_adapter 与 retarget

adapter 每侧一份，职责：

1. 订标准 `JointState /{side}_hand/joint_commands`，严格校验后转原生 `XHandCommand`。
2. 原生内部话题设 `/{side}_hand/driver/xhand_command`、`/{side}_hand/driver/xhand_state`；vendor driver 通过 namespace/remap 接入。迁移后的 retarget、policy、Web 都不直接发布原生命令。
3. 原生数组选择 profile 指定 hand_id，按 name 提取实际12关节；发布标准 state，保留原生诊断/tactile topic供采集与监控。
4. 默认原生 mode=3、kp=80、ki=0、kd=0、effort_limit=400（mA）来自原retarget；将它们放手组件yaml，不能从通用 rad/s或MuJoCo kp推导。
5. 超时/缺反馈/坏消息清缓存并停止接受陈旧目标；恢复从实际q播种。能力诊断表明“安全保持/断电/急停”实际是否可用，不能声称仅停发串口命令就是硬件急停。

retarget改发标准 command；原 `viz`改发独立目标话题。去掉自身发布 `/teleop/armed` 的握拳全局控制，改共用 start gate；握拳可作为可选 `morla_bringup` 输入事件，不成为第二控制权系统。

所有 retarget 输出和HOME请求先检查 PolicyLease；POLICY/PLAYBACK/pending/过期租约无遥操写入。人工恢复以实测手q为起点、限速、等待新鲜landmark，不能重放接管前缓存。HOME变成可验证动作状态，不沿用立即 `success=true` 的伪到点。

## 8. robot_description 资产集中管理

### 8.1 具体归集路径

| 来源资源 | 目标资源位置 |
| --- | --- |
| `astral_robot_description/urdf/astral_robot.pin.urdf` 及同包其他URDF/mesh | `urdf/astral/robot/`、`meshes/astral/robot/` |
| `astral_arm_description` | `urdf/astral/arm/`、`meshes/astral/arm/` |
| `astral_arm_clean_description` | `urdf/astral/arm_clean/`，保留其MDH用途，不成为默认真机模型 |
| `astral_arm_teleop/astral_arm_teleop/robot/` | `urdf/astral/legacy_robotmain/`、对应mesh，记录与当前robot模型是否重复 |
| `astral_mujoco_sim/assets/{mjcf,meshes}` | `mjcf/astral/`、`meshes/astral/sim/` |
| `wujihand_retargeting/urdf/wujihand_{left,right}.urdf` | `urdf/wuji/` |
| `wujihand_mujoco_sim/assets/{mjcf,meshes}` | `mjcf/wuji/`、`meshes/wuji/` |
| `xhand_nero_description/urdf/xhand_nero_description.urdf` 和完整meshes | `urdf/nero/`、`meshes/nero/`（手/连接件子目录可保留） |
| `xhand_retargeting/urdf/xhand_{left,right}.urdf` 及meshes | `urdf/xhand/`、`meshes/xhand/` |
| `xnero_mujoco_sim/assets/mjcf/xnero_dual.xml` | `mjcf/xnero/xnero_dual.xml` |
| description 中launch/RViz/docs及make_mjcf.py | `launch/`、`rviz/`、`docs/`、`scripts/make_xnero_mjcf.py` |

同名不同内容不可覆盖；完全相同资产可按SHA去重，但先建立来源manifest再调整引用。外部 `wuji-retargeting` 自有模型仅复制实际运行需要的资源闭包并保留许可证，或显式声明外部依赖；不能让MORLA里声称已集中管理的模型仍默认从旧项目路径读取。

### 8.2 manifest 和安装要求

每个模型ID记录：源路径/hash、目标相对路径/hash、用途（IK/visual/sim/retarget）、机器人/侧、base/end/tip link、可动joint列表、ROS↔URDF名字与符号/offset、mesh闭包、单位/许可证、生成脚本及参数、是否默认。

迁移机械意义保持：不同时改 joint axis、origin、joint limit、惯量、mesh scale、TCP、DH flip。若发现模型与硬件参数不一致，保存两份并将不一致作为模型差异验证项，不凭“清理”统一成某一份。

资源查询统一 `get_package_share_directory('robot_description')` + manifest相对路径；URDF `package://旧包/...`改为`package://robot_description/...`；MJCF meshdir/include/texture路径相对于安装后的文件可解析。URDF/Pinocchio/retarget/MuJoCo对packageURI支持不同，loader按各库能力解析为可用路径，不能只让RViz加载成功就算通过。

`CMakeLists.txt`安装urdf/meshes/mjcf/rviz/config/docs/scripts/launch的完整子目录。从源码和非symlink安装两种运行方式验证。移除旧 description活动package.xml及teleop/retarget/sim内运行时资产副本；原仓库和baseline里保留来源。

### 8.3 仿真适配

- Astral sim改用集中模型，保持原运动学模式与夹爪/头支持；Wuji仿真保留独立组件入口。
- XNero sim臂订标准`/{side}_arm/joint_commands`，发标准state；不再依赖`SimArmStub`从teleop发私有节点话题。手原生内部通道与real一致，通过同一个xhand_adapter公开标准接口。
- real与sim的同侧反馈发布者/硬件写入者严格互斥。mode=sim不得加载CAN/串口real driver；dry_run不得因SDK import而碰硬件。
- XNero仍是手物理+臂运动学；不能凭臂qpos精确跟随认定真实碰撞停机能力。当前sim不渲染相机；完整采集/策略验收须提供带时间戳的相机夹具或真实streamer，不能把空camera填黑后当完整验收。
- 集中`make_xnero_mjcf.py`从集中URDF生成集中MJCF，所有source/output路径显式参数；记录meshdir和生成hash。复用原sim的触觉一致性测试。

## 9. 统一采集、触觉和历史数据迁入

### 9.1 morla_data_collect 必须保留的行为

保留现有raw异步录制、独立写盘线程/有界队列、IDLE/PAUSED不接样本、SAVING同步、episode原子占号、domain级单例锁、掉帧计数、空录/低fps警告、任务/session控制、冻结schema/meta、所有权事件、图像arrival时间、pipeline来源签名和暂存提交保护。

使用`FrozenSchema.streams`建立订阅与writer，不再使用全局`STREAM_TOPICS/ARM_DIM/HAND_DIM/BODY_STATE_DIM`决定所有机器人。Astral18维反馈切片只在Astral profile里声明。head和waist可各自引用反馈块；Nero无此能力时显式禁用，不填零。

数据根统一默认`${MORLA_DATA_ROOT:-<SDK_ROOT>/morla_data}`；目录为`raw/{profile_id}/{session}/episodeNNNNNN`、`work/`、`clean/`、`pi/`、`act/`、`web_jobs/`。不同profile不会误混在同一session；显示可只展示session名，但job和meta保留完整profile路径。活动录制、schema冻结后禁止换profile。

raw numeric维持`robot_data.h5:/streams/<key>/{values,timestamps}`，camera维持独立camera_data.h5与现有meta相机映射。数值流shape可由registry给出，普通关节(N,D)，高维传感放专门组，避免全局把所有数值拼进action。

meta新增：schema_version、profile_id、robot_id、schema_hash/layout_hash、完整resolved schema、source revisions、description/calibration hash、硬件标识、单位、timestamp_mode、source_kind、实际count/drop、传感能力和录制降级说明。保留task/session/pause/teleop/control_owner等现有字段，计数使用writer实际值。

### 9.2 XHand 触觉数据规范

采集订内部原生`XHandStateArray`（RELIABLE源，订阅可BEST_EFFORT），通过`hand_id`找到对应`sensor_states`，手指按location匹配，不假定数组位置。标准顺序：`thumb,index,middle,ring,pinky`；不把middle写成mid。

新`robot_data.h5:/sensors/<side>_hand_tactile/`保存：

| 数据集 | shape / 单位 |
| --- | --- |
| `force_raw` | (N,5,120,3)，float32；真机单位按SDK标定声明，仿真为N |
| `force_sum` | (N,5,3)，float32；对应calc_force |
| `temperature_raw` | (N,5,20)，保留SDK整数语义；温标/比例记meta |
| `temperature_mean` | (N,5)，对应calc_temperature |
| `timestamps`、`arrival_timestamps` | (N,) float64，同会话时间域 |
| `valid` | (N,5) bool，缺指/错点数可见，不用零冒充有效无接触 |

记录每指frame、point IDs（p001..p120）、sensor calibration版本、来源real/sim、实际单位/比例。真机是传感器自身三轴，sim是指尖link2三轴；未经标定变换不得将二者标成相同方向/单位。sim温度为合成0，应source_kind=synthetic并标为不具物理温度有效性。

触觉默认为observation-only，可独立启停/降频/压缩，不改变关节state/action维数；启用required tactile时缺数据报失败，可选触觉则报降级。关节与触觉要来自同一hand_id，而不是只拿`hand_states[0]`。

对齐使用离散最近有效样本与max_gap约束，保留raw索引/valid，触觉不默认插值。repair按相同source_frame_indices选择相机/状态/触觉/介入标签。保持帧传感沿用末样本时标synthetic，不伪装新测量。

导出必须显式选择传感策略：支持触觉的模型按声明feature导出；不支持的模型保留旁路sidecar+manifest并记录exclusion，policy握手声明未使用触觉。完整数采验收包括raw、aligned、sidecar/导出及逐元素source索引核对，不能只检查ROS有触觉topic。

### 9.3 pipeline和下游模型接口

所有离线实现继续只放`morla_data_collect/pipeline/`，由统一`cli.COMMANDS`分发：保留prepare、preview、quantify、align、validate、repair、replay、export-openpi、export-act、upgrade-v3、verify-aligned、verify-existing、migrate-cameras、process-openpi/process-act，新增`import-xnero`。

默认使用工作副本；来源raw不改；失败中止后续；无显式覆盖不覆盖输出；quarantine仍显式操作。prepare使用真实副本，不能为方便写入而硬链接原数据。raw_signature/缓存兼容检查、视频列/统计/内存约束与现有测试保持。

处理脚本改为按自身位置或已安装模块定位；Web config里`SDK_ROOT/astral_ws/src/astral_data_collect`的硬路径必须改掉。新增`MORLA_DATA_PYTHON`、`MORLA_DATA_CHECK_PYTHON`、`MORLA_SDK_ROOT`，如提供旧ASTRAL环境变量兼容，打印deprecated和实际生效值，并限定过渡期。

OpenPI v2.1和ACT/v3的实际加载测试均保留。现有OpenPI训练适配有Astral命名/delta mask/32维容量假设；在MORLA内给外部训练工具输出model_layout.json、camera_map、delta_mask、dimension/capacity检查和配置示例。对同构旧Astral布局保持兼容；Nero新布局在对应模型适配未验证前不宣称旧`LeRobotAstralDataConfig`直接支持38维。

数据格式容量和具体模型容量分开：通用LeRobot v2.1/v3向量导出可以保留38/54维；只有显式目标模型为标准pi05_base的训练bundle生成/加载和推理握手才拒绝大于32维。不得把模型限制变成raw或通用导出器的全局限制。

### 9.4 旧 XNero 数据导入器

当前旧格式：`robot_data.h5`有`left_arm/joints(7)`、`right_arm/joints(7)`、`left_hand/joints(12)`、`right_hand/joints(12)`、`left_arm/tcp_pose(7)`、`right_arm/tcp_pose(7)`与公共`timestamps`。camera文件每cam_id有JPEG/images和timestamps。采集器将缺源填零、50Hz重复latest cache；旧TCP有效值是xyz+rpy六维、第七维补0，不是xyzw。

`import-xnero --source --out --profile --camera-map --task ...`要求明确配置，输出新的raw工作副本：

- 映射臂/手块与名字，保留原sample时间（timestamp_mode=legacy_host_tick），不把重复latest采样冒充硬件原生频率。
- 使用明确active-side声明；零数组无法区分缺源/真实零，不从零值自动猜禁用侧。标记legacy_validity_unknown，对训练验收提出警告/拒绝规则。
- TCP六维按xyz+rpy转换Pose/诊断流，第七占位丢弃并记provenance；如果文件布局与此不符拒绝，要求显式格式版本。
- camera_map、task和坐标/calibration来源必须明确；缺meta信息保留unknown，不编造。
- 旧数据没有触觉、独立cmd时间或ownership事件，不能恢复出来。next_state可按旧数据生成，command导出必须拒绝缺cmd流，intervention标unknown/invalid。
- 源文件SHA256前后完全一致；导入产物经过align/validate/verify/export实际加载才算迁入完成。

## 10. 统一外部策略接入和控制权交接

### 10.1 延续现有架构，替换布局依赖

保留`backend.py/client.py/protocol.py/engine.py/controller.py/executor.py/replay.py/node.py/runlog.py/image_codec.py`模块职责和当前测试。

`robot_io.py`从core读取schema，按stream声明订阅、按state块组装、按action块拆分；移除固定EE_WUJI/HAND_DIM/ARM_DIM及`/astral`全身切片假设。`CmdTarget`增加joint_names/units/command_kind；`_publish_targets`填完整name。SafetyExecutor从profile加载每通道真实limits、rad/s、ratio rate，不能只构造空joint_limits。

LeRobot导出的`meta/info.json`保留现有标准字段，额外`morla`字段或独立`meta/morla_schema.json`存完整FrozenSchema及hash；v2.1→v3升级和ACT导出必须复制这份契约。replay优先读取它，不再从`*_ee_*`猜gripper；历史Astral缺schema时只接受显式可验证的v1布局绑定。

replay检查完整layout_hash/feature_names/units/action semantics/robot模型映射，不能只校验总维数。旧v1 schema升级后比对；裸无schema数据不直接真机播放，必须显式具名导入。暂停恢复/HUMAN释放以当前实况重规划或重新锚定，保留已有无陈旧chunk设计。

### 10.2 外部策略线协议

继续使用现有WebSocket+包内`protocol.py`的`__ndarray__` msgpack序列化，不用不兼容pip msgpack-numpy的nd格式。

已有request字段：`observation/state`、`observation/camera/<slot>`、`prompt`、可选JPEG image_format和guided RTC字段。已有response：`actions`形状(K,action_dim)、可选server_timing。ROS侧收到的是逆归一化、恢复绝对语义后的关节rad/夹爪ratio，不是网络内部delta或归一化数。

若模型显式声明触觉输入，`ObsBatch`增加`sensors`字典并用`observation/sensors/<sensor_key>/<field>`传array，例如`observation/sensors/left_hand_tactile/force_raw`形状(5,120,3)、`.../valid`形状(5,)。server metadata声明字段、shape、unit和frame要求；未声明传感能力的原ACT/pi05不发送这些字段，完整触觉仍采集并保留sidecar，不能无提示拼到state里。

新server metadata/模型布局文件必须声明：protocol_version、model、robot/layout支持、layout_hash、state_dim/action_dim、feature_names/units、action_semantics=absolute、camera slots/image_size、optional sensor支持/required、model capacity、rtc能力。`serve.py`加载指定model_layout.json，并与checkpoint维度核验；node在规划前验证。

已有第三方OpenPI server可能只提供旧metadata。允许显式`legacy_server_contract=<model_layout.json>`绑定并打印兼容模式；仍校验维数和本地布局，未知语义直接拒绝。不能“只要actions长度正确就开机”。

外部策略新增接入方式：实现PolicyBackend或按上述协议服务端返回chunk，无需修改机器人驱动。原remote/inproc/stub均保留；纯外部推理为主要部署路径，不新建一个能绕过仲裁直接发低层topic的external_action订阅者。

端口固定区分：mocap TCP8000、视频信令8765、policy默认8001、Web8080；旧policy yaml8000改profile默认8001以避免在同主机抢mocap端口。端口可配置并启动预检。

### 10.3 所有权和HITL必须覆盖所有实际写入者

状态机继续IDLE、POLICY、POLICY_PAUSED、PLAYBACK、PLAYBACK_PAUSED、HUMAN；pending保留在payload中。保留epoch、sequence、sent_at、transition_id、prepare token、租约超时和stop取消pending。

迁移要修`node.py::_required_writers`：目前它遍历左右末端字段，有可能等未启用侧的ACK。新逻辑来自resolved profile的活动writer集合，必须覆盖同命令通道的全部遥操writer；如果head或其它组件仍在运行却不在模型action里，也必须纳入冻结/路由策略。不能仅按schema中有几块生成不完整writer表。

没有启动遥操writer的纯策略部署，profile明确登记writer集合为空且不要求reanchor；有teleop writer必须等待其ACK，不能全局把`ownership_ack_required=false`规避Nero缺ACK。

进入POLICY/PLAYBACK：冻结所有相关manual writer → 等同transition ACK → 从新鲜实况seed安全层 → 首次规划准备 → 提交 → 后续tick才发。接管：冻结policy输出 → 所有臂prepare_reanchor返回token → 全部准备成功并确认 → 提交HUMAN → 匹配token激活臂，手/夹爪等也从反馈与新鲜输入启动。部分失败或超时：撤销token、保持暂停、清旧chunk，不半接管。

旧policy进程重启、旧epoch晚包、租约过期、帧丢失、service迟到，都不能让旧锚/旧q自动恢复。数采保留control_owner/intervention/intervention_valid逐帧，repair/export完整保留；历史unknown保持unknown。

保留引擎网络/推理移出控制锁、实测consumed续播、时序融合/chunk anchor、后台首规划、ROS callback groups和timeout，不在迁移中简化回阻塞实现。

## 11. 网页管理统一

### 11.1 后端改造

`config.py`的TOPICS、EXPECTED_RATES、DRIVER_SRV、MOCAP/IK、DATA_PIPELINE_PACKAGE、硬编码包名全部来自resolved profile或应用配置。

保留现有REST响应`{ok,message,data}`、冲突409、服务不可用503、`/ws/telemetry`、独立teleop/collect/infer launch泳道、job进程组取消/历史持久化/原始数据保护、完整日志按需读取和30Hz轻量快照。

新增profile查询/切换接口：GET `/api/v1/robots`、GET `/api/v1/profile`、POST `/api/v1/profile/select`。select只在遥操/采集/推理停止且无待提交运动时生效；旧泳道停干净、清topic与快照缓存、加载新subscription和services、发布新profile/schema hash。CLI起的活动节点根据runtime状态纳入冲突检测，不能只看本Web启动的PID。

robot操作由capability映射至一或多个driver服务；不支持返回明确unsupported并禁用按钮，仿真服务状态标sim，不能假显示已硬件断电。流程协调走第5章共享控制规则。

LaunchManager改为参数数组+明确环境启动，避免把Web输入拼进`bash -c`；需要source ROS时使用固定wrapper脚本，将用户参数作为argv传入。加入profile_id/mode/schema_hash，日志路径/包名判断改用泳道元数据，不再用字符串含astral识别。

### 11.2 前端和监控

实体表从后端profile动态渲染：左右臂、双手/夹爪、头/腰、12/20维关节名字、不同维度图表与健康阈值。缺能力隐藏或disabled，不把Nero缺18维body判故障。健康按模式和状态区分“不启用/等待输入/冻结/失联/慢流”，策略控制时遥操cmd0Hz是正常冻结。

显示robot/profile/mode、layout hash摘要、控制owner/pending/ACK、反馈source_kind、新鲜度、输入strict/legacy、各采集流rate/drop、相机label/设备指纹、触觉每指合力/有效性/来源frame/单位。完整1200点触觉不30Hz塞入通用ui_state；用降采样预览或按需专用接口，避免GIL/JSON带宽拖慢控制。

Web的数据采集/推理启动统一profile，不再让infer launch单独写arms/end_effector。模型不匹配在按钮操作前显示服务握手错误；修改camera_map也要校验profile。

保留既有视频runtime gate服务、active_cameras与preview；录制tap独立于头显视频推流门控。两路只“看”的相机不必成为采集必需相机，profile明确数据相机集合。

前端变化后`npm ci && npm run build`，dist随colcon安装；验证安装share中静态文件可托管，禁止默认依赖旧源码`astral_web_monitor/web/dist`。

## 12. 逐文件执行地图

源路径均以对应`<workspace>/src/<package>`为根；目标包路径按第3章。下表可直接作为修改清单。

| 源文件/定位符 | 目标任务 |
| --- | --- |
| Astral collect `schema.py:CollectSchema/StateBlock/STREAM_TOPICS` | 迁core，设计FrozenSchema/StreamSpec/ActionBlock和v1升级，维度/顺序唯一真源 |
| Astral collect `ownership.py:PolicyLease/fresh_header/bind` | 纯lease迁core，ROS bind分模块；保留序号/时效/IDLE与HUMAN规则 |
| Astral collect `data_collect_node.py:_mk_subscriptions/_mk_joint_cb/_begin_episode/_end_episode` | registry订阅、严格关节名、XHand传感、schema/meta/provenance |
| Astral collect `data_writer.py:StreamDataWriter/CameraDataWriter` | 保留writer/thread模型，加传感writer/source-arrival字段 |
| Astral collect `pipeline/{align_data,validate_data,repair_aligned,verify_aligned}.py` | v2布局/传感支持、source索引同步、validity/单位校验 |
| Astral collect `pipeline/{convert_to_lerobot,convert_to_lerobot_v3,convert_to_act,artifacts}.py` | metadata/layout/传感sidecar、旧数据兼容、actual reader验证 |
| Astral collect `pipeline/cli.py`、`process.sh` | 注册import-xnero，保留单入口和退出码 |
| Astral `scripts/data_pipeline.sh` | 新包module/位置/环境名，不走旧路径 |
| Astral Quest `quest3_udp_mocap.py:process_landmarks/process_line/_process_network_line` | morla_input，raw默认、保留packet/frame/时间/来源，XHand专属修正抽出 |
| Astral Quest `packet_guard.py`、`test/*` | 完整搬迁并重命名import；strict/legacy验收 |
| Astral `astral_teleop/launch/full_teleop.launch.py` | morla_bringup profile装配，只起一份mocap/streamer/gates |
| 新 `morla_bringup/control_coordinator_node.py`、`control_coordinator_logic.py` | 共享操作状态机、profile服务路由、冻结确认、录制/所有权互锁、完成结果 |
| Astral teleop `controller_start_gate.py/controller_workpos_gate.py/*logic.py` | common gates，writer/profile/新鲜度/录制/HITL保持 |
| Astral `head_teleop_node.py` | 作为profile可选Astral节点，core lease、全writer冻结策略 |
| Astral arm `astral_arm_teleop_node.py` | 保留既有IK/reanchor/HOME；core import与profile参数/资产定位 |
| Astral arm `ik/factory.py:default_*_urdf_path` | 改robot_description manifest查询，去旧源码兜底 |
| Astral control `joint_layout.py:pack_named_positions`、`driver_node.py:_on_*_cmd/_clear_cmd_cache` | profile严格映射；禁隐式填零，模式/超时行为回归 |
| Astral Wuji `wujihand_retarget_node.py`、`backends/*` | core lease/asset URI；保持20关节和raw预处理语义 |
| XNero teleop `nero_teleop_node.py:__init__/_feedback_thread/_control_loop/_disarm_callback/destroy_node` | SDK逻辑拆driver，遥操纯ROS，移除隐式回位/关闭时HOME |
| XNero teleop `ik_solver.py:NeroParams/IKSolver`、`pose_processor.py`、`safety_filter.py` | 保留数值实现；变换矩阵与单位显式验收 |
| XNero teleop `urdf_ik_solver.py`、`config/nero_teleop_*.yaml` | 资产迁集中，不再依赖缺失agx_arm_description；数值IK作为可选能力验证 |
| XNero teleop `sim_arm_stub.py` | 不作为生产路径，仿真从统一标准IO接入；必要测试fake留tests |
| XNero hand `xhand_dex_retargeting_node.py:make_config/_resolve_urdf_path/_urdf_to_xhand_joint_values/_build_command_msg/_process_pose` | standard命令、局部preprocess、core gate、反馈播种、HOME协调 |
| XNero vendor `src/xhand_control_ros2/xhand_control_ros2.cpp`、CMake | 通过remap接内部原生topic；必要错误诊断/库安装修复，保留vendor ABI |
| XNero sim `sim_node.py/hand_map.py`、launch/config | arm标准IO、hand内部IO、集中资源、触觉规则 |
| XNero description `scripts/make_mjcf.py` | 集中化生成器、明确输入/输出/meshdir、复用原生成测试 |
| Astral policy `robot_io.py:ObsLayout/CmdTarget` | core有序schema/动态维数/动作块/名字，无EE长度猜测 |
| Astral policy `node.py:_load_robot_params/_setup_io/_required_writers/_publish_targets/_cmd_*` | profile布局、动态writer、名字、limits、HITL与硬件操作互锁 |
| Astral policy `backend.py:RemoteBackend/open/infer`、`client.py`、`scripts/serve.py` | metadata布局握手、absolute/容量/传感契约，兼容旧server显式绑定 |
| Astral policy `replay.py`、`executor.py` | 真机layout校验、反馈seed、profile限位/限速与回放重锚 |
| Astral Web `config.py`、`schemas.py`、`monitor_node.py` | 动态实体/服务/profile管理/传感监控，清旧路径 |
| Astral Web `launch_manager.py`、`web_server.py` | profile传所有泳道、硬件动作协调、argv启动、CLI活动检测 |
| Astral Web `data_process_manager.py` | 新CLI/env/目录schema选择；保留副本/job路径隔离 |
| Astral Web `web/src/{types,App}.tsx/ts`、`api/client.ts`及组件 | 动态关节/能力/profile/错误/触觉与不支持模型显示 |
| 所有 `setup.py/setup.cfg/package.xml/resource/launch/config` | 重命名、依赖core/description、递归资源安装、yaml节点名回归 |
| Astral `scripts/serve_policy.sh/serve_policy.env/jetson_teleop_start.sh/act_train.sh/openpi_train.sh` | 新包路径/profile/env/端口/模型layout，机器本地值不当通用配置 |

## 13. 按依赖顺序实施的阶段与完成条件

每阶段先更新`docs/migration_progress.md`，记录输入/变更/验证命令/结果/未验证条件；通过后保存独立commit或可回退快照。不能跳过前面接口阶段直接把xnero整目录塞入最终src。

### P0：创建副本、冻结源与环境（无硬件操作）

执行者先核对baseline中的commit/hash；如果源码有后续变化，仅审阅改变的关键文件并记录差异，禁止reset或清理原仓库。

从SDK根执行示例：

```bash
test ! -e MORLA
mkdir MORLA
rsync -a --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
  --exclude='__pycache__' --exclude='.pytest_cache' --exclude='node_modules' \
  --exclude='*.egg-info' --exclude='inference_test_logs' \
  --exclude='pi05_infer_test_log' --exclude='pi05_infer_test_logs' \
  astral_ws/ MORLA/
cp docs/MORLA_MIGRATION_PLAN.md MORLA/docs/MIGRATION_PLAN.md
cp docs/MORLA_SOURCE_BASELINE.json MORLA/docs/SOURCE_BASELINE.json
```

以上命令必须逐条检查成功；`test`失败时不能继续mkdir/rsync。MORLA若已有内容先按progress判断是否是本迁移副本，恢复已有阶段，不盲目覆盖。复制前枚举其它大数据/运行日志目录，仅排除生成数据，不漏源码资源；不要带旧install内指向原项目的symlink。复制后核验源码/配置/描述hash，验证symlink无运行时指向原ws；禁止跟随外部symlink把大数据复制进项目。

创建MORLA独立git（若已有则复用），保存“原Astral源代码复制”基线commit，保留许可证与供应商信息。复制来的本地.env与硬件地址只作为local config、加入适当gitignore；不把checkpoint/dataset/secrets提交。

记录ROS/Python、colcon、numpy/scipy/h5py/cv2/mujoco/dex-retargeting、SDK版本、外部库架构、离线/ACT/OpenPI环境和npm lockfile。ROS在线用系统Python3.10，模型/离线环境隔离；不使用conda Python去生成ROS消息。新shell只source`/opt/ros/humble`和MORLA安装，不能继承astral/xnero overlay。

**完成条件：** 新项目可定位、基线可回退、完整源代码hash可比、原两仓库hash/status未变化、环境缺项已列出。未构建不等于阶段失败；依赖安装下一阶段处理。

### P1：重命名共享包并迁文件夹，先恢复Astral基线

按第3章移动/重命名common与robots/astral/wuji；新增四个机器人/手元包先提供manifest/config框架；旧description此时暂留以便对照。改所有import/resource/setup/launch/test/scripts/env，节点名和现有topic先保留。core可先迁schema/lease兼容实现，暂不同时改语义。

运行colcon list确认所有实际包且无同名冲突；package.xml依赖全闭包；前端构建与重命名后的纯逻辑/ROS测试。使用stub/dry-run仅检验源路径/启动参数，禁止初始化真实SDK。

**完成条件：** Astral原能力在新包名/目录下可构建与调用；不依赖旧源路径；无重复Quest包/collector；旧测试数值保持。保存package_map.json（旧名、新名、路径、可执行名、节点名）。

### P2：实现core/profile冻结规格与Astral注册

实现第4章版本/schema/profile/resource locator；将Astral所有现有配置注册到profiles，并将三个泳道对齐到同源。保留旧v1解析及稳定features名。

增加profile验证CLI（`python -m morla_core.profile validate-all`）与纯逻辑黄金测试；重复/缺名、维数、只读动作、未知版本、相机、active-side/末端冲突、布局hash比较均有负例。验证8/16/27/35/54等实际维数及原block顺序。driver/teleop继续通过参数兼容层接收原参数，而不是一次性重写算法。

**完成条件：** 任何消费者只拿profile_id就得到同一schema/layout；Web/采集/推理schema hash一致；原Astral历史meta可读且逐维等价。无硬件仅profile校验不触发SDK。

### P3：集中description并验证安装路径

按第8章归集资源、manifest、所有loader与生成器，之后移除活动旧description包和重复资产目录。使用Pinocchio/MuJoCo/retarget实际加载、FK对比，RViz可作为额外验证。测试symlink和普通安装。

**完成条件：** 全模型及mesh闭包解析，生成器可重跑，原关节/机械意义不改变，没有旧绝对路径或缺失agx_arm_description依赖；Astral仿真/IK原回归仍通过。

### P4：迁入统一输入和Nero/XHand局部变换

morla_input只一个接收器；统一raw，抽XHand预处理，提供per-side source选择。注册Nero的矩阵/初始姿态/固件/CAN参数，不接硬件。完成包Guard、frame/session、IOBT、mixed/Joy/Glove冲突与旧→新数值对比。

**完成条件：** 合成相同Quest数据，经旧/新Nero链的平移/姿态增量等价；旧/新XHand预处理等价；Wuji回归不受影响；QoS实际匹配；严格输入拒过期/乱序包，legacy显式标记。

### P5：拆Nero驱动与遥操，使用fake SDK先通过

实现第7章两个Nero包、profile wiring，删除production SimArmStub和teleop SDK调用。fake SDK覆盖connect失败/enable超时/反馈缺失/错误joint/命令断流/stop-estop-mode/cache/左右独立。复用Nero IK与VR mapping数学用例。

**完成条件：** 不运行teleop也能提供反馈；policy或标准命令能经driver SDK适配下发；teleop只发标准ROS；disarm/stop不HOME；无首帧填零、无无限while enable；驱动和所有权日志可审查。到此仍不得把未实测真实CAN称为已完成硬件适配。

### P6：XHand adapter/retarget与XNero仿真接通

迁vendor两包、SDK、xhand retarget，建立标准↔原生适配、hand_id选择、符号映射、core lease。仿真改标准arm/内部hand IO与集中资源；新增headless全链路launch。

**完成条件：** 使用合成21点输入→XHand标准命令→adapter原生命令→sim→标准反馈；每侧12维/名字/单位/符号正确，retarget无伪joint_states；50Hz触觉源有有效样本；real/sim不能同时启动。保留原sim测试功能覆盖并记录实际测试数，不复制旧“31例”充当新结果。

### P7：统一采集、传感、pipeline与旧数据导入

动态streams/sensor writer/metadata、source时间、XHand force/温度/valid、不同profile维数；离线aligned/repair/export/verify支持；实现import-xnero。

**完成条件：** 所有首批profile的合成raw→aligned→validate→可选repair→export→真实reader可执行；raw与source不改；触觉raw/aligned/输出逐元素对应；关节与介入/图像索引无错位；缺关键流/坏相机/非有限/模型容量不符明确失败；旧XNero数据导入不补假信息。

### P8：统一policy外部接口与HITL适配

重构layout与具名target、配置limits、握手、active writer注册、Nero prepare/token/ACK、XHand冻结/反馈seed，保留引擎优化。增加Nero+XHand的stub策略/回放/人工接管集成测试与38维假远程服务测试。

同时完成第5.4节共享协调节点，将Web/手柄动作转为调用此入口，统一“策略活动时HOME/工作位/模式操作”与暂停/恢复语义；只读输入仍直接订阅标准topic。

**完成条件：** 19/38维backend可正确拆到标准topic并驱动sim；标准32维pi05对38维在发命令前拒绝；同维不同layout也拒绝；ACK缺失/prepare部分失败/迟到token/epoch过期/暂停恢复/stop pending全无双写与旧chunk恢复；所有实际manual writer纳入冻结。

### P9：Web profile、动态监控与操作管理

改Web配置/ROS桥/argv启动/三个泳道/profile select/capability/frontend，保持数据job与API旧行为。加入simulation/hardware/legacy状态、touch预览；安装dist。

**完成条件：** 从Web选择Astral和Nero，同profile起遥操/采集/推理并监控；左右手12/20维正确；不支持动作禁用且后端拒绝；CLI启动的节点同样可控且参与profile切换阻塞；录制中不可回位/换profile；ROS服务失败不假报成功。前端浏览器交互验收+后端测试+build全部记录。

### P10：端到端仿真与Astral回归

执行第14章矩阵，至少完整走一遍：选择profile→输入→遥操→采集→保存→离线校验/导出→stub/假远程策略→HITL→回放→停止。覆盖Astral8维基线、Astral35维配置、Nero单侧19维和完整38维。

**完成条件：** 关键运行topic和schema一致，原Astral可用组合不退化；集中description普通安装可运行；原两仓库未改；源copy/新schema/旧导入/new38维链路有可复用证据与自动命令。

### P11：真机与真实模型部署验收

在有对应硬件/固件/SDK/APK/模型的现场按最小侧依次验证：只读反馈→dry-run标准命令诊断→显式enable与低速单关节/六方向→单臂遥操→单手→双侧→数采→真实remote策略→接管/释放→急停/恢复。HOME/工作位/阻尼等逐能力验证后再启按钮；不自动沿旧shutdown轨迹运行。

保留Astral原低通/机械关节映射与实测夹爪2.5/0.0配置；验证真实硬件反馈和夹爪command_echo语义；Nero先验证CAN名称和NeroFW，XHand先确认串口/hand_id/ABI/实际力单位。旧APK只验兼容模式；严格包验证需要新APK+时钟同步。

**完成条件：** hardware_acceptance逐项记设备/固件/模型/layout/限速/实际结果；没有设备/38维模型/ARM64SDK时对应项标“未验收/缺条件”，软件迁移与硬件可用性分别报告，不把测试skip写PASS。用户若只要求软件迁移，交付软件完成及清楚的硬件待验收列表。

## 14. 验收矩阵、命令和证据

### 14.1 必须覆盖的测试

| 类别 | 正例 | 必须失败/保护的负例 |
| --- | --- | --- |
| 构建/安装 | colcon识别全部新包；symlink及普通安装；console entries | 重复Quest包、嵌套遗漏包、缺resource、旧ws兜底路径 |
| profile/schema | 8/16/19/27/35/38/54维；单侧禁用；旧v1数据 | 同维换序、unknown版本、缺关节、错单位、inactive侧ACK |
| input | 6方向+3轴旋转、21点、mixed/IOBT、glove替代 | 旧frame delta、双来源同topic、旧session重放、坏Quaternion |
| driver | fake SDK按名字下发；实况seed；mode切换 | 无限enable、NaN/短数组、缺名填零、陈旧cache重启、stop自动HOME |
| XHand | 左右12维、right index符号一次、hand_id选择、实际state | adapter二次取反、vendor QoS不匹配、retarget目标伪装反馈 |
| sim | arm standard IO、hand物理、触觉一致、headless | real+sim同侧双driver、仿真空图当完整数据 |
| collect | 分流源时间、独立arrival、drop/count、record/pause/save | 缺必需流、空录、源中途变schema、录制时换profile/回位 |
| tactile | 5*120*3、finger顺序/valid、raw→aligned→sidecar | 缺指补零当valid、real/sim单位/frame混标、repair传感错位 |
| pipeline | raw来源签名、选帧索引、v2.1/v3真实reader加载 | 旧cache无签名、dirty输出、失败仍继续、修改原始raw |
| policy | 19/38维stub/假服务、absolute目标、limits | 32容量接38维、同维错layout、delta当绝对、timeout旧chunk |
| HITL | 所有writer ACK；prepare全成功才HUMAN；暂停恢复重规划 | 部分prepare失败、service迟到、坏token/epoch、policy重启、freeze遗漏头/手 |
| Web | CLI与Web一致；动态实体；job隔离；能力禁用 | 只看WebPID切profile、模式切换policy仍发、拼shell、伪service成功 |

算法/资产纯数值回归对源码保留实现要求无实质偏移。机械IK收敛精度按现有数学tests保持，位置方向/坐标纯变换用第6章阈值。实时控制率按profile目标测量，10秒稳态均值≥目标90%，报告p50/p95/max间隔、反馈龄期、drop和异常；这属于软件调度验收，真实安全与跟踪精度单独硬件验收。触觉重流开启/关闭都测Web与控制延迟，不能只证明文件能写。

### 14.2 计划中的统一验证入口

实施时创建以下脚本，使后续执行者无需记散落的旧命令：

```bash
# 每个命令在干净shell中执行，只加载Humble与MORLA
cd /home/robot/loopkok/sdk/MORLA
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
colcon list
bash scripts/build.sh --mode symlink
source install/setup.bash
python3 -m morla_core.profile validate-all
bash scripts/test_ros.sh                 # 系统Python，域隔离91+ROS_LOCALHOST_ONLY
bash scripts/test_offline.sh             # MORLA_DATA_PYTHON
bash scripts/test_policy.sh              # 保留完整原policy suite及新node flow
bash scripts/test_sim_integration.sh     # headless，合成输入/相机，不接硬件
```

build.sh默认完整依赖闭包，不沿用旧README的手写packages-select漏掉Web/policy。ROS Python/CMake构建用系统解释器；外部pip SDK锁版本/路径；C++接口先构建。测试子集与环境不足要在报告中区分；不能因原项目曾测试通过而跳过新路径回归。

生产操作目标入口：

```bash
ros2 launch morla_bringup robot.launch.py profile_id:=astral_left_gripper mode:=sim
ros2 launch morla_bringup robot.launch.py profile_id:=nero_dual_xhand mode:=sim
ros2 launch morla_data_collect data_collect.launch.py profile_id:=nero_dual_xhand session:=smoke
ros2 launch morla_policy_inference policy_inference.launch.py \
  profile_id:=nero_dual_xhand backend_type:=stub
ros2 launch morla_web_manager web_manager.launch.py
bash scripts/data_pipeline.sh --help
```

这些入口是要求实现的目标，不应现在在原项目执行。真实remote启动必须选择匹配model_layout并完成握手；stub仅在sim/dry-run默认可用，不自动进入真实运动。

### 14.3 验收产物

每阶段至少输出：变更commit、包映射、resolved profile/schema、接口topic/service清单、测试/构建命令与退出码、测试摘要（pass/fail/skip）、golden原始数据签名、转换source-index校验、仿真运行日志、策略所有权时间轴。

最终`docs/migration_progress.md`必须区分已实现、软件已验证、硬件未验证、外部模型未验证。最少附一个新Astral session和一个Nero/XHand仿真session（含触觉）的全链路报告；文件过大放data根目录并记录hash/路径，不提交数据本体。

## 15. 风险、边界与恢复方式

| 风险 | 已确定处理 |
| --- | --- |
| 名称改了，但运行仍从旧workspace导入 | 干净shell、安装入口验证、rg活动引用、模块__file__/ament prefix日志、symlink扫描 |
| 配置多份相互覆盖 | 单一profile解析；launch非空覆盖；所有泳道schema/hash自报并启动前核验 |
| Nero disarm触发归位，与策略并发 | 删除隐式回位；policy冻结/ACK后提交；所有运动动作走协调入口 |
| XHand坐标/右index双翻导致反向 | 旧新合成数据数值等价；硬件/模型约定分界只转换一次 |
| 触觉看得到录不到，或错手/错单位 | hand_id/location映射、raw/aligned/sidecar端到端元素对照、unit/frame/valid字段 |
| 38/54维被截断为32 | dataset不限维；model容量/布局握手拒绝；完整采集和模型支持分别验收 |
| 供应商动态库无法在Jetson运行 | 架构/ABI检查，设备目标分开验收；未获得库不宣称ARM64可用 |
| 旧schema与历史数据混在一起 | v1显式升级、profile目录隔离、hash/版本、旧XNero独立只读导入器 |
| 资产路径迁移改变IK/仿真模型 | 来源SHA与manifest，逐模型joint/FK/MJCF加载；不混同clean/legacy/current模型 |
| 新epoch/未ACKwriter导致旧遥操恢复 | core lease共用、动态active writer表、迟到token/timeout/pending负例 |
| 源项目损坏/回退成本高 | 仅改MORLA，阶段commit；原Astral/XNero可独立运行，source hash/status复核 |

阶段失败只回退MORLA的该阶段commit/快照，不删源仓库、不重置原开发状态、不删除原始数据。构建缓存清理由新workspace或专用build目录处理；禁止拿旧README里的`rm -rf build install log`对SDK根或原仓库执行。

## 16. 给下一位执行模型的直接任务说明

> 在 `/home/robot/loopkok/sdk` 使用 `docs/MORLA_MIGRATION_PLAN.md` 与 `MORLA/docs/SOURCE_BASELINE.json`。若 `MORLA/` 已存在，先读 `MORLA/docs/migration_progress.md` 和当前 Git 状态并从最近未完成阶段续做；不得重新复制或覆盖工作区。若尚无 `MORLA/`，才按P0建立独立副本，然后只修改MORLA。既定共享包名、目录、profile/schema、ROS契约和验收标准遵循本文，不重新设计一套无兼容关系的框架。优先保留Astral已有配置、算法、HITL和离线保护；将Nero硬件逻辑拆独立driver，将XHand接到标准JointState并真实反馈，触觉实现落盘/对齐/旁路导出。每阶段在progress记录实际修改与验证后保存commit；测试缺环境写skip/未验收，不写PASS。硬件、APK、模型或架构库缺失只阻止对应真实部署验收，不阻止独立的软件阶段。不要改动原工作空间，不自动驱动真机归位，不通过关闭ACK/截维数/填零/旧路径兜底让测试表面通过。最终交付可构建的MORLA、配置与接口文档、迁移映射、全链路软件证据及真实部署待验收项。
