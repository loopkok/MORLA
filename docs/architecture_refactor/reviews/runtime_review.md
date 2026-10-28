# Runtime / Core 独立对抗性审查

审查日期：2026-10-28。审查者：data_pipeline agent；被审查代码作者：runtime agent / core_composition agent。
审查者参与 R6 数据模块实现，因此这里只对非本人编写的 runtime/core/native bridge/共享设备锁声明独立审查。
方法：先根据 VALIDATION.md 的 S01–S16 构造反例，再检查作者测试；真实反例先报作者修复，之后独立运行复核。
不启动 ROS、不连接 SDK、不发送真实机器人命令。

## 已复现的 P1 及修复复核

1. **缺失 resolved_hash 可绕过编译产物校验（S12/S14）**。
   旧 RuntimeSession 仅在字段存在时 verify_resolved。删除该字段、修改关节上限后仍可由 CLI/ROS 的 JSON 路径构造 runtime。
   作者改为默认拒绝缺失哈希；仅测试用显式 allow_unverified_test_fixture 可接受裸 fixture。
   独立复核：得到 RESOLVED_HASH_REQUIRED，反例关闭。
2. **失败的 native stop ACK 被 enabled=False 擦除（S03/S05/S09）**。
   旧 NativeDeviceSession 在 transport.operation 返回 False 时仍清掉 enabled；第二次 stop 直接返回 True，根本不调用 transport。
   作者改为只有成功 ACK 才清 enabled/active_ports。
   独立复核：连续两次 transport=False 均返回 False，调用次数 2、enabled 仍 True，反例关闭。
3. **Astral gripper 命令回显被标为实测，并把 rad 当 ratio（S04/S11/S13）**。
   driver_node.py 发布的是缓存 gripper 指令角，桥接原来无条件标记 measured=True/source=hardware/unit=rad，而组件契约使用 ratio。
   作者显式转换 rad→closure_ratio，标记 source=command_echo/measured=False/valid=False/ready=False。
   独立复核：开位 .8rad 得到 ratio 0，且不能通过 measured readiness。
   该真实夹爪组合在缺少实测反馈方案前保持不可激活；此限制是安全门槛，不是真机验收已完成。
4. **reset 可在 stop 从未成功时清急停（S09）**。
   构造持续 stop=False、health.ready=True 的适配器：start→activate→estop→reset 原来得到 SUCCEEDED/READY_DISARMED。
   作者在 reset 期间重新要求所有 stop ACK 成功。
   独立复核：FAILED/STOP_UNCONFIRMED，状态保留 ESTOP_LATCHED，反例关闭。
5. **旧 Nero stop 状态可充当新请求 ACK（S03）**。
   Bridge 缓存上次 estop_status=sent（仍在 500ms 内），新 Trigger 请求只返回 queued。
   旧实现立即返回 True，没有等待新 estop_seq，SDK 即便尚未执行本次 stop 也会被认定已停止。
   作者为真实 Nero 驱动 enable/estop 添加返回序号，bridge 要求请求返回相关序号、对应状态序号及请求开始之后的状态帧。
   独立复核：未关联的旧 queued/sent 回执被拒绝，runtime_review_repros.py 七项检查均 True（含旧 generation 拒绝、新 generation 接受）；反例关闭。

## 可重复证据

本目录 `runtime_review_repros.py` 使用 mock transport 和 fake adapter；每条输出为 True 才算该反例关闭。
最终独立结果保存在 `runtime_review_results.json`：7/7 True。
独立执行 runtime/test 与 morla_core/test/contracts：**68 passed（1.92s）**。
R6 单独回归：**104 passed, 1 deselected（25.91s）**；deselected 为无 rclpy 环境无法执行的在线 ROS 采样用例。
源码修改后必须重新运行，不能沿用之前的结果。

共享锁另做跨语言独立验证：将只构造 LegacyDeviceGuard 的 C++17 程序编译后，Python ResourceManager 持有独占 lane：

- 普通 C++ 子进程退出码 7（拒绝）；
- 通过 pass_fds 传入真实 lane fd 的自有子进程退出码 0；
- 伪造 fd=-1 退出码 7（拒绝）；
- runtime 释放锁后普通 C++ 子进程退出码 0。

该检查覆盖 Python/C++ 的 inode 与 flock 协议；不代表 ROS launch 已编译或硬件已连接。

## 覆盖判断与明确边界

S01/S08：Python 与 C++ 共享 lane 的冲突/授权 fd 通过；原 ROS public driver remap 通过纯 native_plan 检查。
S02/S04/S06/S09：runtime 单测检查 token/session/epoch/sequence/TTL、错关节/单位/限位、部分派发失败和启动清理。
S03：主动发现并修复失败 ACK/旧 ACK 反例；完整异步生命周期与迟到 SDK 返回以 runtime 回归为补充。
S07：native_plan 使用单 Astral 进程、managed sides、关闭 full-body/head 自动路径；实际板级影响范围仍须硬件台架验证。
S10：SDK worker timeout 和独立子进程清理能离线检查；设备/板端 watchdog 的物理停止上界仍未实测。
S11/S13：本审查对比 native gripper 的单位/来源；模型/数据闭环另见 integration review 与 R6 canary。
S12/S14：缺 hash、修改哈希载荷、shell argv、路径/描述符检查可离线执行；安装包/插件文件更新后需要重新编译锁定产物。
S15：Web 限流由 integration review 覆盖，不在这里以 runtime 单测替代。
S16：native 来源时钟回退/重复样本被拒绝；板端重启、断电、真实网络分区及 watchdog 必须另作台架验收。

**阻塞的外部验证**：当前 macOS 环境没有 ROS 2 Humble/rclpy/vendor SDK/机器人。不能宣称 native 桥接完整 ROS build、真机 stop 时延、Astral 板级隔离、断电恢复或模型真训练已通过。真实部署的 verified 标志需要对应的外部证据。
