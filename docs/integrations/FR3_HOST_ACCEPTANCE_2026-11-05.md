# FR3 ROS 2 主机与浏览器验收 — 2026-11-05

本轮使用用户指定的 Ubuntu 22.04.5 x86_64 / ROS 2 Humble 主机，项目 `/home/loopkok/MORLA`。起始代码为 `40a69cbe2c8fff765e8fdd1a425ef538a89955a0`，工作区最初干净；验收期间修复了下述部署问题。使用 `/usr/bin/python3`，退出 conda 后构建和测试。

已完成 FR3 相关 ROS 编译、安装、fake DDS、SDK Linux 导入、不可达本机地址负例，以及浏览器实际交互。实体机械臂和实体 Quest 未使用；没有向实体设备发送连接、使能或运动命令。未实施新的动力学仿真、碰撞模型或真实数据训练。

## 主机与隔离

- 主机：用户指定的 `192.168.50.5`，Ubuntu 22.04.5、Humble、系统 Python 3.10。
- DDS 测试环境：`ROS_DOMAIN_ID=126`、`ROS_LOCALHOST_ONLY=1`。所有被测执行器均为 fake；没有修改用户终端的持久环境配置。
- Web 测试监听目标机回环地址，端口 18084，经 SSH 本地转发访问；退出测试后关闭服务器与合成输入源。
- 原始证据目录：`/home/loopkok/morla-acceptance/fr3-20261105-223209/`。录制及运行诊断保留在其中的 `web-store/`。
- 安装依赖：`python-mbedtls 2.10.1`（SDK 即使不用 TLS 也导入），以及仓库测试依赖中指定的 `hypothesis 6.135.26`，均安装至系统 Python 的用户 site。

## 发现并修复的问题

1. **colcon 将 FR3 当作普通 Python 包。** `robot@localhost` 被 catkin_pkg 判为非法邮箱，ROS 包识别失败。首次构建虽返回 0，但没有生成 ROS local_setup，目录发现与 launch 失败。将包元数据及 setup.py 维护者邮箱改为有效地址后，`colcon list` 显示 `ros.ament_python`，重新构建后可正常发现和启动。
2. **Web launch 传入不支持的 ROS 参数。** 普通 CLI 使用 argparse，不接受 launch_ros.Node 自动附加的 `--ros-args`；空 deployment 也可能丢失参数值。改为 OpaqueFunction 构建 ExecuteProcess 命令，仅当 deployment 非空才附加该参数。实际 fake Web launch 已启动并通过浏览器验收。
3. **Humble 信号关闭竞态。** 公共 ROS 网关停止时重复调用 shutdown，或在 context 已失效时抛 wait-set RCLError。改为 try_shutdown，处理 KeyboardInterrupt/ExternalShutdownException；仅在 context 已关闭时接受该 RCLError，活跃上下文中的错误仍继续抛出。重新执行 driver/Web SIGINT 验证，两个 launch 均退出 0，无信号升级，分别约 0.165 / 0.414 秒。
4. **通用 IK 数值未显示。** CartesianController 将设备残差放在 solver_report 中，页面原先只识别 CR5 的扁平报告，导致已求解仍显示“等待目标”，并展示一个空控制器卡片。统一展示嵌套位置/旋转误差、求解状态并过滤空报告；新增前端回归，实页已出现“已达到求解精度”和误差数值。

退出边界：一次最终 driver SIGINT 日志仍出现 Humble `Future.__del__` 中被忽略的 KeyboardInterrupt 提示，进程退出码为 0，未升级至 SIGTERM；没有隐去该日志或将 traceback 字段改为 false。早期通过交互 shell 后台启动的测试也曾等待 5 秒后由 launch 升级 SIGTERM；改用独立进程会话的最终信号测试未发生升级。见 `shutdown-check.json` 及主机原始 `driver-signal-test.log`。

## 实际验证结果

- **PASS：34 个相关包 symlink 编译。** `colcon build --symlink-install --packages-up-to morla_fr3_driver morla_web_manager --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`，修复后的输出 `34 packages finished`。
- **PASS：普通安装。** FR3 单包使用独立 build/install 目录完成非 symlink 构建。在 `/tmp` 中从普通安装导入 FR3，ament 资源发现、fake 配置编译均成功，state/action 各六维；发现过程中未加载厂商 Robot 模块。公共依赖来自已构建工作区，这不是整个 MORLA 的全量普通安装验收。
- **PASS：IDL/DDS。** 已读取生成的 CommandEnvelope 接口，实际 ROS Python 消息和服务穿过 DDS：prepare、activate、acquire/renew、有效目标与回执、反馈、急停和 reset。重复/未知组件、急停锁存、复位后未激活目标均正确拒绝。
- **PASS：JointState。** 增加 smoke 脚本可选 `--require-joint-state`，在 fresh fake 驱动上实际收到 `joint1..joint6`、六个有限 rad 值和非零 ROS 时间戳。首次在 reset 后追加 CLI echo 超时，随后还遇到旧 ros2 daemon context 错误；最终使用直接 rclpy DDS 订阅在初始状态验证，结果保留于 `dds-final.log`。
- **PASS：资源互斥。** 第一驱动占用 fake:arm 后，第二个 ROS 驱动退出 1，报告 `RESOURCE_BUSY: fake:arm`。停止后后续驱动与 Web 能重新获得资源。
- **PASS：Linux SDK。** 真正安装 python-mbedtls 后导入原始 FAIRINO Robot.py 成功。另对本机回环且未监听的 20003/20005 端口执行只读检查，按预期快速返回连接错误和退出 1；没有构造实体控制器连接。该负例不代表真实固件兼容。
- **PASS：FR3 专项 34 项。** 目标机系统 Python 执行 `bash scripts/test_fr3.sh`。
- **PASS：全量 pure 338 项，无跳过。** 首轮 334 passed / 1 skipped（Hypothesis 未安装）；安装仓库锁定版本后完成 338 passed。
- **PASS：fake 数据/策略/Web 28 项。** `scripts/test_composable.sh --layer fake`。既有 FastAPI 弃用和 pandas 可选加速库版本警告保留，不影响本轮通过。
- **PASS：前端。** TypeScript/Vite 构建通过；18 项交互模型测试通过，dist 已同步到目标机并重新构建 Web 包。

## 浏览器实际交互

访问的是 ROS 主机 Web，经 SSH 转发至本机 `http://127.0.0.1:18084/`。本轮内置浏览器可用，完成了：

1. 发现六轴 FAIRINO FR3，确认 arm 与 quest 均为测试数据后端，运行时配置只读。
2. 准备设备；没有输入时点击开始，明确显示“重锚缺少新鲜输入：right_vr”。
3. 在隔离 DDS 域发布合成 Quest v3 帧，重新点击开始，完成重锚和激活；观测 fake 关节偏离初始位置，状态始终 `source=fake / measured=false / synthetic=true`。
4. 点击开始记录、结束并保存；检查 HDF5：六维状态 **1183 条**、六维动作 **2450 条**，任务 `fr3_ros_host_browser_acceptance`，`recording_status=complete`。动作是接受的目标，不是物理完成标签，状态/动作采样频率不同，记录数不同不是对齐证明。
5. 点击暂停、恢复；修复前端后目视确认一个有效 IK 卡片显示数值，空报告不再出现。
6. 停止并释放工作站；重新载入 FR3 预设、检查配置/依赖、保存 v2、启动，确认仍为待使能，再停止并恢复可编辑状态。

合成输入来自主机证据目录的 `quest_fixture.py`，不是实体 Quest。它使用真实 DDS 发布并经过公共 ROS 输入接收/BindingEngine/IK/Runtime。浏览器验收不替代真实输入设备验收。

截图与主要机器记录在 [证据目录](evidence/2026-11-05-fr3-host/)。完整诊断约十余分钟，未开展小时级长稳、性能基准、实物相机或真实模型效果测试。

## 后续运行

主机已安装修正后的 symlink 包和 SDK 依赖。正常新终端中执行：

```bash
cd ~/MORLA
# 若处于 conda，先 conda deactivate
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch morla_fr3_driver fr3_web.launch.py
```

默认为 fake + fake，地址 `http://127.0.0.1:8080/`。本轮临时 18084/18085 服务器与合成输入源均已退出。若从另一台电脑浏览，按现有安全访问方式配置 SSH 转发或已有 Web 的监听地址；不需要修改设备后端为 real。

仍需现场执行的部分为：实体 FR3/Quest、控制器固件和网络兼容、逐轴方向/零位、多位形 FK 与 TCP、实体停止/断网 watchdog、负载与真实控制频率/长稳。FR3 MuJoCo/碰撞模型当前未提供，未将其标为通过。
