# CR5 + LoopHand 配置组合与验证记录

日期：2026-10-28。依据 [设备接入指南](../device_integration_guide.md)。两类设备来自同一个桌面示例项目，但在 MORLA 中按设备能力独立接入。

## 最终结构

- [CR5 独立驱动包](../../src/robots/dobot/morla_cr5_driver/README.md)：`morla_cr5_driver`；[接入记录](cr5.md)。
- [LoopHand 独立驱动包](../../src/robots/loophand/morla_loophand_driver/README.md)：`morla_loophand_driver`；[接入记录](loophand.md)。
- [组合示例](../../examples/cr5_loophand/README.md)：仅 Assembly/Deployment 和组合回归，不是 ROS 包，不包含设备驱动。
- 通用 worker 与 ROS 网关属于 `morla_runtime`，不依赖或识别任何具体品牌。

初版将二者放在 morla_cr5_loophand 一个包中，导致协议、依赖和安装耦合。当前已删除这个联合包及 cr5_loophand_driver 联合入口，拆分为互不依赖的两包；四个插件 entry point 分别归各自组件所有，原 plugin_id 保留。每包拥有独立描述、参数 schema、单设备启动配置、测试与许可证。旧冻结 resolved 的插件来源/描述摘要已改变，应重新编译；组合 Web 需保存新 revision，不继续使用旧快照。

## 如何组合

装配引用 `morla.dobot.cr5` 与 `morla.loophand.compact`，由公共 runtime 调度两个已安装的组件。两包可分别构建/测试，并不强制绑定彼此。需要两台设备同时工作时，使用一份组合快照和一个 RuntimeSession；当前 MORLA 的 host lane 仍独占，不能同时开两个 standalone runtime 来代替装配。

默认 fake 组合为 state_dim=13/action_dim=13；每个单设备分别为 6/6 与 7/7。组合启动使用 `morla_runtime joint_state_runtime_node`；独立设备入口为各自包中的 cr5_driver、loophand_driver。也可让组合 Web 管理同一装配，不能并行启动第二份 runtime。

组合示例没有假定已标定的机械安装、TCP 或相机；不是预装配的真实机器人模型。真实手仍缺确认停止/watchdog，只读门禁保持，因此双 real 装配全组件 activate 不能通过。其他品牌臂/末端可以通过更换 Assembly 引用加入，禁止写品牌专用联合驱动。

## 拆包验证

实际验证结果：

1. 各包只出现在自己的源码 PYTHONPATH 下分别运行测试：CR5 4 passed；LoopHand 3 passed，不需要另一驱动。
2. 各包独立构建 wheel；在独立临时安装目录中确认另一包不可 import，并从已打包资源执行 ament descriptor 发现、选中 factory 加载、ABI 和单组件 fake 生命周期：6/6 与 7/7 均通过。
3. 两包同时安装时通过通用 catalog 编译/运行组合，13/13 契约与资源释放通过；组合不依赖联合包或联合入口。
4. MORLA pure 回归：112 passed、1 skipped；包含两包测试和独立的组合配置测试，跳过项不计通过。静态 Python/XML、文档链接及旧联合包引用检查通过。

macOS 没有 ROS Humble；安装资源验证仅替换 ament 索引查询层，读取真实 wheel 中的资源和 entry point，不替代目标主机 colcon/IDL/ROS 验收。所有实体设备、真实 stop/watchdog、标定、长稳、真实采集/模型效果仍 NOT_RUN。

原始协议/SDK 没有调用带自动使能、自动清错或自动 homing 的脚本；未修改桌面原项目。回退先停止会话并确认设备状态，保留现场数据/标定，不删除活动锁。
