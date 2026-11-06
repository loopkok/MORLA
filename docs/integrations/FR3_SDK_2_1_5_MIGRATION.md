# FR3 SDK V2.1.5 / Robot V3.8.5 本地迁移

日期：2026-11-06；起点提交 `017e2d5`。用户确认现场 Web 版本为 V3.8.5，要求使用桌面 `fairino-python-sdk-v2.1.5_robot3.8.5`，本轮仅修改本地 MORLA，没有 SSH、没有同步/构建 ROS 主机、没有连接实体机械臂。

## 固定的 SDK 与协议

- 替换 `morla_fr3_driver/vendor/Robot.py` 为新目录的 `linux/fairino/Robot.py`，原始字节保持一致；SHA-256 和旧 SDK 来源记录在 `PROVENANCE.json`。
- 新源码 `GetSDKVersion()` 声明 `SDK:V2.1.5 / Robot:V3.8.5`；这是源码声明，不是本轮实机读取。
- 使用 XML-RPC TCP **20003** 发送命令，TCP **20004** 接收旧版固定结构实时状态。删除 CNDE 20005、UDP 20007、TLS/mbedtls 的运行依赖与适配；不按端口连通情况自动切换 SDK。
- ROS 单臂入口、公共 Web 宿主继续加载同一 Adapter 和隔离进程，不另建第二条控制路径。预设 ID、ROS 消息、关节单位/顺序、URDF、IK、限速、按住跟随、Quest 前向 +X 映射保持原契约。

手册 4.0.1 仍是已有软限位和法兰补充的历史资料来源；换 SDK 不等于这些参数已经由 3.8.5 实物验收。现场须重新核对模型、零位、方向、TCP、限位与停止。

## 为什么不能只换 Robot.py

V2.1.5 没有 `FRCNDEClient`、`_cnde_client`、`is_connect` 和 `SetReConnectParam`；旧标志拼写是 `is_conect`。旧 SDK 使用 `RobotStatePkg` ctypes 结构，内置接收器会自动重连，分包及校验失败处理也不能提供 MORLA 所需的有效样本保证。

新增 `realtime_state.py`，在隔离进程内生成 SDK RPC 子类：

1. 以原 SDK 的 packed `RobotStatePkg` 为布局，目前 `ctypes.sizeof` 为 **958 字节**，不复制另一版固件的数据结构。
2. 检查 `0x5A5A` 帧头、little-endian `data_len + 7` 总长、完整帧 uint16 累加校验和；通过后才解码并原子发布样本。
3. 保留 TCP 半帧，支持任意分包和多帧粘包。接收完整包含校验和的结构，避免旧接收逻辑把不完整结构交给 ctypes。
4. 坏帧、长度不符或非有限关节值锁存故障，不把错误数据改成零，不刷新采样时间。重复 8 位帧号不刷新时间，255→0 回绕正常。
5. 单独管理实际接收线程和 socket；EOF、流超时、重复帧失效后不重连，关闭时终止接收线程。
6. 用有界 XML-RPC transport 和一次请求代理替代默认代理，既禁止标准库断连重试，又把网络异常转换为旧 SDK 重试循环不会吞掉的错误。使能/停止使用 operation_timeout，普通调用用 rpc_timeout，外层进程期限保留。

SDK 原文件不改写；只有上述接收/连接/关闭/请求边界由 MORLA 子类与代理接管。启动不使能、不切模式、不自动清错；`prepare` 只检查反馈。`activate` 仍确认使能后进入 ServoMoveStart，ServoJ 仍在 SDK 边界把 rad 转为 degree。

停止仍依次尝试 StopMotion、ServoMoveEnd、RobotEnable(0)，等待新的失能反馈。通信断开、进程被终止均不能证明物理停止；现场停止/断网验收仍必须执行。

## 配置与 Web 升级

- SDK V2.1.5 不需要额外 `python-mbedtls`；旧环境即使安装该库也不会因此自动获得旧协议兼容。
- descriptor 的 real 依赖改为目标版本和 20003/20004，真实连接错误和只读报告更新为当前协议。
- `fr3_check` 的 stamp_domain 为 `host_monotonic_ns_on_validated_TCP_20004_frame`；这是主机接收完整有效帧的时间，不是硬件时钟。
- descriptor 摘要变化后，已有真实配置需要在部署新代码、重启 Web 后重新检查并保存新 revision；IP、身份、映射和现场参数由原 Assembly/Deployment 保留，禁止直接修改旧 resolved 哈希。
- 本轮没有部署主机。后续需要在主机同步并构建 `morla_fr3_driver`，已有 Web 进程也需要重启；本地源码修改不会改变当前运行服务。
- 用只读检查确认持续反馈和多位形 FK，再按无输入使能/停止、小步运动、Quest 的顺序测试。不能把本地模拟器通过当成 3.8.5 实机通过。

## 本地验证结果

- FR3 定向 **67 passed**。新增测试使用真实 V2.1.5 SDK 和本地 XML-RPC/TCP 控制器模拟器，覆盖构造只读、实时反馈、SDK FK 返回格式、使能、新反馈确认、ServoJ、停止、EOF、无自动重连、关闭线程。
- 原始 ctypes 布局造帧覆盖逐字节分包、粘包、帧号回绕、重复帧、错误帧头/长度/校验和及 NaN。网络异常断言只调用一次。
- 公共核心、Runtime、遥操、Web、仿真及 FR3 相关回归 **260 passed / 1 skipped**（可选依赖）；FastAPI 依赖产生一项已有弃用警告。
- 在临时目录构建 wheel 成功；从 wheel 导入真实 vendor SDK 与接收器成功，版本为 V2.1.5 / V3.8.5，未导入 mbedtls。没有在本机声称完成 ROS/colcon 构建。
- [测试证据](evidence/2026-11-06-fr3-sdk215-local/)。

厂商原始源码包含行尾空白；为保留来源摘要未格式化。手写变更单独做 whitespace 检查。
