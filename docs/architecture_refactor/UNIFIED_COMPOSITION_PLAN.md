# MORLA 组件与绑定统一重构计划

2026-10-30；以 c1896c6 为实施起点。目标：更换输入、单/双臂、末端和传感器组合时，主要改变组件与绑定配置。本文替代“增加一套设备组合 launch 即完成接入”的判断；与项目实际代码共同作为实施依据。

## 一、目标边界

厂商协议和运动学允许各自实现；输入适配器只产生标准意图，执行器只接受标准目标。中间 BindingEngine 按能力选择 controller。公共层不得根据输入与机器人品牌配对分支。一个系统入口管理所选组件及子进程，Web 与无 Web 调用复用相同执行路径。

保持当前资源互斥、显式启动/激活/停止/急停与错误报告；不恢复此前移除的租约门禁、部署验证 flag 或重复确认。数据结构/单位错误应清楚报错，不通过填零或猜测映射继续执行。

## 二、当前问题与迁移原则

1. Quest、QuestRight、Vader 的传输不同且能力不一致；QuestRight 缺 scalar，旧 Quest 有 scalar。统一标准意图端口，原始话题留在适配器边界内。
2. Web 中创建输入空壳并执行控制映射；裸 Runtime 不具备同等输入执行路径。抽取可脱离 Web 使用的 BindingEngine，再使两种入口复用。
3. 输入采集进程由专用 launch 启动；native_plan 有机器人分支；新驱动可能进程内直连 SDK。提供通用托管进程工厂接口，adapter 注册自身生命周期，公共编排不认识厂商命令行。
4. Astral pinch_gripper 直接写专用话题。迁移为标量输入→通用映射→Astral adapter；保留旧入口用于兼容，不把旧入口称为新系统已迁移。
5. 几何/IK 和灵巧手重定向保留插件实现；同一设备的不同上游应复用同一运动学与驱动。

## 三、实施阶段与交付

### U1：公共绑定执行内核

抽出 controller 创建、绑定实例化、目标构造、生命周期与重置；输入是 InputFrame，反馈由 adapter.health 提供，输出统一 ComponentTarget。Web 调用该内核，非 Web 可以直接调用。支持一个输入映射多个执行器及多个输入映射不同执行器；同一执行器的多源仲裁必须显式配置后再扩展，当前保持单 binding。

测试：同一 scalar 控制米制夹爪/ratio 夹爪；同一组具名 axes 控制两个不同 plugin_id 的机械臂；同一执行器换输入实例；未知 controller 明确失败；插件 close/reset 被调用。

### U2：输入能力与传输归一

标准能力：pose（m、xyzw）、axes（具名连续轴）、scalar（声明范围）、joint_targets（具名关节）、hand_landmarks（明确骨架）、按钮事件；每个源提供 source_id/epoch/sequence/本机接收时间/源时间与 frame。允许设备原始协议不同，消费方不读取厂商包。

先补齐 QuestRight 扳机数据发布和 scalar 端口，保留原 right_pose/right_hold；Vader 复用已存在 scalar。新增通用输入源注册与统一标准帧 ROS 入口；迁移旧 Quest/glove 分支为插件。左右手使用实例身份与配置表达，不继续添加 quest_right_epg 等组合。

测试：真实格式 Quest 原始数据→scalar；VR/Vader/mobile 同值经公共映射输出相同开口；断连/模式变化不会把其他端口刷新成新数据；不同传输的时间字段含义一致。

### U3：组件与设备进程统一生命周期

新增 ProcessSupervisor 与选中组件 process-factory 注册；工厂返回固定 argv、环境、是否外部托管和就绪说明。编排按组件清单启动并在失败/停止时回收，SDK 进程内 adapter 可声明无需子进程。共享控制板只启动一次。外部启动模式仍可用并在状态中显示。进程退出须显示错误；不通过轮询无关 ROS 节点猜就绪。

集成 Runtime/输入管理，不再要求用户为设备组合挑专用 launch；system/session 入口支持同一份 assembly/deployment。专用 launch 最终只作为指向公共入口的兼容薄封装。

测试：两种 input 替换仅配置变化；输入+双臂+两末端启动一次；共享设备去重；子进程失败回收；external 进程不被误杀；fake 模式不启动真实设备。

### U4：旧设备链路迁移

Astral/Nero/CR5 采用统一 position 目标，Cartesian 使用运动学插件；Astral 夹爪采用 scalar_map，EPG 保持米制开口；XHand/Wuji/其他灵巧手用手部重定向插件。相机/传感器沿用 read 端口。删除公共业务层的品牌条件，设备专属行为留在插件中。

测试：配置矩阵 VR/Vader/mobile × EPG/Astral夹爪，pose/axes × Nero/Astral/CR5，骨架 × XHand/Wuji；不要求每种都真实硬件存在，但 fake 必须逐元素验证。实机结果分别记录。

### U5：Web 与数据联动

Web 根据能力筛选 source/controller/target，不根据品牌生成逻辑；输入启动、设备准备、激活、采集、停止都走同一 session。state/action/camera/sensors 从 frozen schema 生成。展示托管进程和启动失败，明确 external。

浏览器验收：空配置创建混合双臂/双末端；替换输入只编辑 binding；替换夹爪同时变更单位映射；未安装插件可解释；停止后无自有进程残留；数据特征与配置一致。

### U6：审查、扩展考试与验收

审查必须检查：是否仍有组合专用代码、Web/CLI 路径是否分叉、是否按话题名猜设备、是否把旧专用链路误标已迁移、启动与回收是否成对、传感数据是否改变模型动作布局。用未在内置列表中的两个虚构机器人插件和一个输入插件做接入考试；仅新增插件包与配置应可工作。

按 pure、fake、ROS、浏览器、实机分开报告。当前主机无 ROS/设备时，不把协议测试称为 ROS 或硬件通过。计划完成条件是 U1–U6 全部有相应软件证据，实机缺项逐项列出。

## 四、实施记录

以代码与实际测试更新，不预填完成状态。尚未迁移的旧入口继续明确标为 legacy；不能据 U1 完成推断整套统一架构完成。

### 首批实际落地

- U1：新增 morla_teleop.engine.BindingEngine，Web 复用同一目标生成器；新增跨输入、跨夹爪及混合双臂配置测试。裸 Runtime 的完整自动输入驱动仍待集成。
- U2：QuestRight 增加 trigger 事件和 scalar 端口；增加 morla.input.standard JSON/ROS 桥。旧发布者尚未全部迁入标准协议。
- U3：新增 ProcessSupervisor、morla.processes 工厂入口、QuestRight/Vader 采集进程计划，接入 connect_inputs/close 并显示进程状态。默认 external 保留现有操作，managed 显式选择；所有执行器/共享会话统一编排尚未完成。
- U4–U6：旧 Astral pinch 链路迁移、所有标准能力端口、通用系统入口、完整 Web 表单/设备组合矩阵和最终独立审查尚未完成。
- 首轮 conda 离线 pure：177 passed、4 skipped；fake：23 passed。跳过的厂商 SDK/物理环境不算通过。无 ROS/实机启动验证。本轮未新增租约/deadman/部署验证门禁。

使用方法与协议示例见 [统一组合示例](../../examples/unified_composition/README.md)。

### 后续本轮实现与证据

公共 ControlHost、ROS/CLI 输入连接、输入插件边界、进程自动托管/共享去重、兼容 launch 收敛、pinch_scalar、轴名能力描述、Web 动态控制器与进程选项、实际浏览器及已安装插件扩展考试已落地。详细文件与复验见 [UNIFIED_IMPLEMENTATION.md](UNIFIED_IMPLEMENTATION.md)。

阶段状态：U1 软件路径及 U2 标准入口已实现；U3 已接系统入口和输入托管，但原生设备启动代码完全下沉设备包仍待迁移；U4 公共夹爪/双臂绑定可用，但旧 pinch 滤波/自动范围的等价迁移未完成；U5 当前浏览器流程通过，完整配置矩阵尚需 ROS；U6 扩展考试与实现者检查已执行，独立最终审查未完成。整计划不得标为 COMPLETE。


### 剩余软件迁移收敛记录

- U1：公共 BindingEngine/ControlHost 与动作路由已接入 Web、CLI、ROS 入口；补充缓存端口去重与关闭异常回收。
- U2：标准意图桥、内置传输模块化、QuestRight scalar/骨架、手柄端口轴名和进程选项已实现。源协议差异留在适配器内。
- U3：原生 ROS 桥与设备启动计划已迁入 morla_native_bridge；Runtime 使用注册工厂，跨逻辑组件共享一次物理会话。已实际构建并隔离安装插件验证。目标环境的真实进程启动仍 BLOCKED_ENV。
- U4：旧 pinch 的自动范围、EMA、扳机 shaping 与输出 slew 已提取并由旧/新路径复用；冻结旧算法 trace 逐元素通过。Nero/Astral/CR5 的 IK 可通过设备注册工厂选择。旧入口继续标记 legacy。
- U5：能力/维度/单位筛选、进程选项与状态、采集联动及 Edge 浏览器流程通过。
- U6：未知组件扩展考试、失败反例、静态层边界检查和隔离 wheel 验证通过。最终记录见 reviews/UNIFIED_FINAL_REVIEW.md；检查由同一实现者加隔离解释器完成，没有第二位 reviewer。

软件实现收敛，当前复验 pure 202 passed/4 skipped、fake 23 passed、sim 7 passed，前端构建与浏览器通过。不得将此状态等同于全部部署验收 COMPLETE；ROS/实机矩阵与第二 reviewer 仍未执行。上述更新覆盖早期“尚待迁移”的阶段记录，但保留早期记录供追溯。

文档复核开放项 UR11：无 Web ROS stop 与 Web stop 的输入回收时机不同，见 UNIFIED_IMPLEMENTATION.md。U3 的“stop 即完整回收”仍需修复该入口并在 ROS 复验；本轮文档更新未改变生产代码。
