# FAIRINO FR3 接入与验收记录

2026-11-06 当前通信基线已改为 **SDK V2.1.5 / Robot V3.8.5，20003 + 20004**，仅完成本地修改与离线验证，未同步 ROS 主机。旧 V2.2.8/CNDE 主机记录保留历史含义，详见 [SDK 迁移记录](FR3_SDK_2_1_5_MIGRATION.md)。

日期：2026-11-05。本地实现基线：`70ee93fd45b3983992f0eb438c80cf060066dcdd`；目标机验收基线：`40a69cbe2c8fff765e8fdd1a425ef538a89955a0` 加本轮修复。

教学入口：[FR3 移植实战教程](FR3_PORTING_TUTORIAL.md)；成员可复制[移植工作单](ROBOT_PORTING_WORKSHEET.md)开展新设备接入。

## 当前仿真更新

已新增独立 `fr3_mujoco_sim` 入口和公共 MuJoCo 场景适配，支持运动学/参考动力学、Web/local/both、FR3 IK 和场景相机。用法与本轮证据见 [FR3 仿真说明](FR3_SIMULATION.md)。下面 DH 与 URDF 阶段的验收数字均保留历史口径。

## URDF / IK 更新

现已提取独立 `fr3_description`（仅 fairino3_v6），FR3 使用与 CR5 同策略的严格 URDF DLS、微分 IK 与异步 controller；新预设默认微分模式。此次模型/控制链变更的证据见 [URDF/IK 移植记录](FR3_URDF_IK_MIGRATION.md)。下面主机/浏览器统计属于此前 DH 版本，不能替代新版本验证。

## 此前驱动阶段结论

已在 Ubuntu 22.04 / ROS 2 Humble 主机完成 FR3 相关 34 包编译、独立普通安装、真实 DDS fake 命令/回执/六轴 JointState/服务、资源互斥及浏览器交互验收。目标机 pure **338 passed**、fake **28 passed**，前端 **18 passed**；浏览器完成配置保存、激活、录制、暂停与停止，录制 HDF5 已核验。详见 [ROS 主机与浏览器验收报告](FR3_HOST_ACCEPTANCE_2026-11-05.md)。

实体机械臂/Quest、真实控制器通信、物理停止/断网、TCP 标定及动力学/碰撞仍未验收。fake 不是 MuJoCo。下文及“本轮软件验证”中的 Mac 阻塞是此前阶段记录，已补齐的主机/浏览器项目以新报告为准。

## 此前 Mac 实现阶段结论

标准法奥 FR3 的独立驱动、组件注册、本地运动学、公共 ROS 网关入口、Web 选型/预设与
fake 输入→控制→录制链路已实现。当前环境是 macOS，没有 ROS 2 / colcon，没有已提供的真实
控制器地址/固件/目标主机，因此 **ROS 构建、DDS 通信、实体 FR3、实体 Quest、停止/断网、TCP 标定均未验收**。
浏览器自动化因 Mac 锁屏及内置浏览器初始化超时受阻；前端构建、交互模型测试及实际 HTTP 服务验证通过。
没有连接真实机械臂、没有发出实机使能或运动命令。fake 不是物理仿真。
用户已确认当前未接机械臂，后续自行执行真机测试，ROS 主机/IP 稍后提供；本轮交付范围为实现与离线验证。

## 项目实际情况与接入取舍

依据 [设备指南](../device_integration_guide.md)、[CR5 接入](cr5.md)、
[Nero 接口](../nero_arm_interfaces.md) 和当前源码核对：

- 主线是独立包 descriptor + `morla.adapters` / `morla.kinematics` 插件 + Assembly/Deployment。
  旧 Profile 和 `robots.yaml` 不是新设备的主接入口。FR3 不修改旧 Nero/CR5 的运动逻辑。
- 当前 Runtime 不统一检查命令名称/单位/TTL/序号，也没有统一命令 watchdog；FR3 在自己的 adapter
  严格检查六轴名称顺序、有限数、rad、模式、部署软限位和目标领先，并在发送端限速。
- CR5 的公共 worker 可复用接口思想，但不能直接照搬：FR SDK 有无限重试及类级共享状态，
  FR3 使用独占 SDK 子进程和明确超时；故障清理覆盖软暂停后仍开启的伺服。
- Nero 的实测反馈、重锚与坐标约定经验适用；七轴几何、J2 零位和镜像规则不适用 FR3。
  FR3 采用自身 fairino3_v6 URDF 模型；手册 DH 仅作独立测试参考。Quest 基变换、TCP 偏置仍在公共绑定中明确配置。
- 目录发现只读静态元数据，不能 import SDK 或连接设备；Python wheel/ament 注册实际安装验证通过。
- 公共 Web 有机器人控制器白名单，新增 FR3 标签与通用 IK 匹配，否则只注册 Python 插件会导致页面
  无法选择位姿控制。实际发现并补齐了这个接线点。
- 旧 Web 手机回归错误地假定显式 deadman 被删除；从基线提取的测试也失败。本轮只更新该测试断言，
  与当前“显式绑定时松开不输出”的实现一致，不修改任何既有运动门控。

## 设备、SDK 与模型来源

- 包：[morla_fr3_driver](../../src/robots/fairino/morla_fr3_driver/README.md)。组件 `morla.fairino.fr3@1.0.0`。
- 用户本地 `FR机械臂资料/fairino-python-sdk-v2.1.5_robot3.8.5/linux/fairino/Robot.py` 原样随包安装。
  SDK 的 `GetSDKVersion()` 自报 **SDK V2.1.5 / Robot V3.8.5**；这是源码声明，不是已读取的实机版本。
  源码 SHA-256、手册 SHA-256、Apache-2.0 许可证见包内 `PROVENANCE.json`。
- 本地手册 v4.0.1 表 2.1-1（PDF 第 121 页、印刷 35 页）的 FR3 软限位已目视核对；
  表 2.4-1（PDF 第 126 页、印刷 40 页）的标准 DH 参数已提取。没有套用其他 FR3 子型号。
- [官方 Python SDK 文档](https://manual.fairino.support/latest/SDKManual/python_intro.html)
  已对照；实现以固定的本地源码为准，不默认任意旧固件兼容。
- `sim` 现使用公共 MuJoCo 后端；保留 URDF 惯量与显示网格，但未启用机器人碰撞，参考位置伺服动力学未完成实机辨识。fake 与 sim 分别登记。
  本地 FK/IK 有数学自一致测试，但仍须用多组 SDK FK 和实体位形核对模型约定。

## 接口与行为

Canonical 与 SDK 同序 J1..J6，位置统一 rad；SDK 只在传输边界转成 degree。
URDF 几何单位为 m；FK 为 base_link→flange，TCP 在绑定 `solver_tip_from_tool` 中表达。

当前 SDK V2.1.5 使用 XML-RPC 20003 + TCP 实时状态 20004；运动命令采用 XML-RPC。
不再依赖 CNDE/TLS/UDP 接收线程，不保留自动跨版本回退。未支持扩展轴、力控、夹爪或 HOME 自动运动。

20004 状态按固定 SDK ctypes 布局检查完整长度、帧头与校验和后原子复制；`sample_ns` 来自主机接收完成时的 `monotonic_ns`，
不是机器人硬件时间。重复帧不更新时间，8 位帧号回绕允许；短帧、NaN/Inf、解码异常和通信失效被标故障。
这不等于跨机时钟同步，也不证明传输层没有网络缓冲延迟。

导出的 position 状态带 `hardware/measured=true` 或 `fake/synthetic=true`；故障后的端口 valid=false。
设备状态含使能、主/子错误码、急停、保护停止、碰撞、帧计数及本地递增序号。
ROS JointState 复用公共网关按采样时间发布，不为重复缓存制造新时间戳。

prepare 只检查反馈，activate 才使能并确认新反馈，再进入 ServoMoveStart；不会自动清错或切模式。
发送是单槽最新目标，默认 125 Hz，`cmdT=0.008`，默认速度上限 0.3 rad/s，实测领先上限 0.1 rad。
这只是软件调度目标，不是硬实时承诺。暂停保持最后已发送目标，stop 清空待发目标并请求停运动、
结束伺服、失能，须等新失能反馈。SDK 卡死时终止隔离进程并记录 **physical stop UNCONFIRMED**。
设备自身断网停止机制仍必须实测。

公共控制层释放 grip 只停止生成新目标；已接受目标仍可能收敛，不能当作物理急停。
Web 全局暂停会调用 adapter.release 并保持最后已发点。SDK 超时和上游命令停发是两类问题：
后者沿用现框架保持目标语义，不新增全局 TTL watchdog。

录制动作标签是公共网关已接受的请求目标，**不是每个 125 Hz 中间伺服点或物理完成证明**。
状态来自独立反馈。数据维度固定 state=6/action=6；当前验证的是 fake raw HDF5 和冻结 SchemaV3，
未进行训练、模型效果、实机数据质量或 LeRobot 全管线验收。

## 启动与分阶段现场验证

完整构建、ROS 话题与部署说明见包 README。便捷 Web 入口（需同时安装 morla_web_manager）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch morla_fr3_driver fr3_web.launch.py
```

默认 FR3 + Quest 均为 fake，不连接实体设备，打开 `http://127.0.0.1:8080/`。
已有统一 Web 时，在“设备预设”载入 FR3 即可，不需要再启动第二个服务器。
独立 ROS driver 与 Web 会话二选一拥有设备，不得同时运行并使用不同资源键绕开锁。

建议依次留证：

1. **目标机安装**：Ubuntu 22.04 / ROS Humble 系统 Python 构建，V2.1.5 SDK 无额外 TLS 依赖，
   验证目录、adapter 和 kinematics entry points。不要将 Mac Python 测试结果当作 colcon 成功。
2. **ROS fake**：编译 `assembly.fake.yaml`，运行 `fr3_driver.launch.py`，查 JointState 六轴顺序/rad、
   私有服务、启动 disarmed、激活/回执/停止与资源释放。
3. **只读真实设备**：确认标准 FR3、序列号/IP、固件与所供 SDK 兼容；人工选择正确模式/负载，
   在控制器端确认没有其他程序占用。运行 `fr3_check --host <实际IP> --output /tmp/fr3-readonly.json`。
   检查变化帧序号、SDK 错误与本地 FK 残差。该检查不使能、不切模式、不运动。
4. **使能与停止**：现场操作员就位后，通过真实 Deployment 显式使能，先不发送运动目标。
   记录新使能反馈、stop 后新失能反馈、SDK 错误和控制器日志。断网/进程退出须单独验证实体停止。
5. **逐轴小幅运动**：从实测关节值出发，对每轴约 0.01 rad 增量、低速验证方向/零位，
   检查被拒绝目标、反馈断流与再次激活不会重播旧点。收紧现场限位。
6. **真实输入 + fake FR3**：先核对 Quest 的右侧、基变换、按住/释放、重锚和输入断线，避免几何和网络问题混淆。
7. **真实 FR3 + Quest**：核对多组 FK、TCP 与输入方向后再开展；保留诊断日志，检查暂停/停止、
   目标领先、实际采样/发送率、迟到与长稳。远程代码测试不能代替本步骤。
8. **采集/回放/模型**：再验证状态/动作语义与时间、控制权切换、HDF5/导出、长稳和模型效果。

需要补充的信息：控制器 IP、实际固件、ROS 主机/系统及连接方式、末端工具与安装朝向。

## 本轮软件验证

可复现入口：

```bash
bash scripts/test_fr3.sh
bash scripts/test_composable.sh --layer web
cd src/common/morla_web_manager/web
npm ci
npm run build
npm run test:workcell
```

本机使用已有 `/tmp/morla-cr5-vr-tests/bin/python` 和随应用提供的 Node；这些路径不是部署要求。

以下是初次 V2.2.8 接入的历史结果，当前 V2.1.5 测试以迁移记录为准。

- **PASS：34 项 FR3 专项**。六维编译、插件 ABI、Runtime 生命周期、目标名称/单位/有限值/范围、
  限速、暂停保持、停止、故障锁存、软暂停期间断流、SDK 阻塞进程超时、真实 SDK 的 CNDE 解码、
  帧号回绕、重复帧、短包、SDK 返回码/单位、使能新反馈确认、FK/Jacobian/IK、Web 输入→控制→录制。
  CNDE 解码测试仅 stub 了可选 TLS 导入；没有伪称测试 TLS 网络连接。
- **PASS：252 项相关回归，1 项跳过**。范围：core/contracts、runtime、teleop、CR5 全包、FR3 全包。
  本轮没有修改 CR5/Nero 驱动代码。
- **PASS：10 项公共 Web + FR3 Web 回归**。其中一项旧 deadman 断言修正已说明。
- **PASS：TypeScript/Vite 构建、17 项 Web 交互模型测试**。已同步仓库跟踪的 dist。
- **PASS：实际安装**。驱动 wheel 构建及临时目录安装；使用官方 ament_index_python 的 Humble 分支
  读取真实 resource index，目录发现未导入 SDK，adapter/kinematics entry points 均存在。
  最终 wheel 独立加载两份 fake 配置并完成 Runtime 启动/停止，安装了 driver/Web 两个 launch。
- **PASS：实际 localhost HTTP 冒烟**。`/components` 发现六轴 FR3，载入模板，保存编译，启动为
  READY_DISARMED，反馈 source=fake/measured=false/synthetic=true，prepare/stop 成功。
  本轮临时记录 `/tmp/fr3-http-acceptance.json`；未占用用户生产配置目录。
- **BLOCKED_ENV：全量 pure 脚本**。既有 `test_teleop_events.py` 在收集时直接 import rclpy，当前 Mac 没有 ROS。
  252 项回归使用明确选择的子集，不能将其记为整个 pure 脚本通过。
- **BLOCKED_ENV：浏览器目视交互**。锁屏及浏览器初始化超时，未声称截图/点击验收通过。
- **NOT_RUN：ROS colcon/IDL/DDS、真实 SDK 网络、实体 FR3/Quest、控制器固件适配、
  物理急停/断网 watchdog、长稳、动力学/碰撞与模型训练效果**。

以上计数存在包含关系，不能相加作为独立用例总数。提交与推送结果见 [本轮发布日志](FR3_PUBLISH_LOG_2026-11-05.md)。
