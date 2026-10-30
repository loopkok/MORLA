# LoopHand / Aero Hand Open 独立组件接入记录

> 2026-10-30 更新：当前公共路径已收敛为标准输入、绑定和注册执行器工厂。接入接口更新见 [设备指南](../device_integration_guide.md)。本轮没有改变 LoopHand 的真实只读边界或固件能力，不能据公共框架测试宣称真实运动可用。

日期：2026-10-28；MORLA 基线 b91211a，当前工作区实现。依据 [设备接入指南](../device_integration_guide.md)。来源为桌面 cr5_loophand_graspnet/loophand/aero_open_sdk，原项目未修改。

## 包边界

独立包：[morla_loophand_driver](../../src/robots/loophand/morla_loophand_driver/README.md)，组件 `morla.loophand.compact@1.0.0`。拥有自己的 package.xml/setup.py、Python 模块、descriptor、ament manifest、adapter entry points、loophand_driver ROS 入口、单设备 launch、7 维 fake Assembly、real Deployment、来源/许可证及测试。

不依赖 CR5 或其他机械臂包。可接任何满足 MORLA 契约的机械臂，也可独立运行手。只有 Assembly 决定与谁组合；通用 worker/ROS 网关来自公共 morla_runtime。

## 数据与控制约定

紧凑手型为 thumb_cmc_abd、thumb_cmc_flex、thumb_flex、index_flex、middle_flex、ring_flex、pinky_flex，位置 rad。7 维目标经上游模型展开为 16 个耦合运动学关节，再换算执行器位置；不能当作 16 个独立可控/可测关节。

串口 921600、稳定 /dev/serial/by-id 身份，协议 `<2B7H`。GET_POS 读实际执行器编码器，模型反解的关节反馈标 source=estimated、measured=false，不是 command echo。JointState 无 provenance，必须结合 state/health 判断。fake 的所有合成来源仍明确标记。

## 真实手运动限制

真实手当前只读反馈。上游 CTRL_TOR 零力矩请求缺少执行 ACK，也无已验证通信 watchdog。Adapter 不允许通过配置声明绕过门禁；stop 请求后报告 STOP_UNCONFIRMED，不能宣称停止完成或通过删锁绕过。

下一步需固件/驱动提供可确认停止/禁能及设备 watchdog，再补掉线、SDK 阻塞、进程退出、重新上电和目标恢复验收。启动不会自动 homing、设置力矩或运行原项目的抓取线程。触觉总线、导纳抓取、输入重定向、URDF/仿真还需独立接入。

构建/接口/单包测试见包 README。ROS Humble/实体手/标定/长稳/数据及模型验收均 NOT_RUN。独立构建与安装验证结果集中记录于 [本轮拆包验证](cr5_loophand.md#拆包验证)。
