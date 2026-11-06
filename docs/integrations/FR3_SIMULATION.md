# FR3 独立仿真包与公共 MuJoCo 接入

2026-11-06 SDK 更新：真实驱动改用 V2.1.5 / Robot V3.8.5 的 20004 反馈协议；fake、URDF/IK 与共享仿真仍使用原模型和算法。见 [迁移记录](FR3_SDK_2_1_5_MIGRATION.md)。

后续框架改造：模板、控制器与仿真资产接线已迁至描述符，见 [插件声明驱动规范](PLUGIN_DRIVEN_PORTING.md)。本文原仿真验收结果保留其执行时含义，最新公共层回归另见 [验证日志](PLUGIN_PORTING_VALIDATION_2026-11-06.md)。

日期：2026-11-06；实现基于 MORLA `06939d1`，参考同版本 CR5 的独立包与公共场景实现。仅 fairino3_v6，不连接实体 FR3。

## 两种入口对应什么

`src/robots/fairino/fr3_mujoco_sim` 是机器人拥有的独立 ament_python 仿真包，含 launch、CLI、默认装配和 README。它与 CR5 一样，调用 `morla_web_manager.session_cli` 和公共 `sim_launch.description()`，不复制物理循环。

`src/robots/fairino/morla_fr3_driver/launch/fr3_sim.launch.py` 是接入公共层的兼容入口，使用驱动内 `config/quest.sim.yaml`；普通 Web 也可加载 `fr3-sim` 模板。独立包的 `config/composable.sim.yaml` 与该模板内容相同，并由测试防止漂移。二者最终都通过 `RuntimeSession → MujocoDeviceSession → MujocoAdapter`，同一场景拥有状态、控制、相机与仿真时间，不是两套模拟器。

## 资产如何建立

`fr3_description/urdf/fairino3_v6.urdf` 继续作为 IK 几何基准，摘要不变；`scripts/make_fr3_mjcf.py` 借鉴 CR5 生成器，在同一个独立描述包生成 `mjcf/fr3.xml`。七个 STL 不重复复制到公共 robot_description，也不依赖桌面供应商目录。

保留源关节原点/轴、质量/惯量、mesh 姿态与显式 flange 固定帧（腕 3 Z 向 100 mm）。生成器区分固定/活动关节，允许无惯量的固定 flange；原始惯量坐标旋转为零，若以后变成非零会拒绝生成，避免静默忽略旋转。J3 继续取手册 ±150° 与原 URDF 的交集，不扩大到约 ±162°。

基础参数参考 CR5：timestep=0.002 s、implicitfast、joint damping=5、armature=0.05、position kp=1200/kv=120、forcerange ±300。来源和 URDF/MJCF/生成器摘要见 `fr3_description/SIMULATION.json`。它们是参考位置伺服参数，没有 FR3 电机/减速器/控制器辨识结论。

机器人网格仅显示，contype/conaffinity=0；碰撞声明没有被当作已验收碰撞模型。模型代码与入口的 Apache-2.0 不覆盖上游许可未声明的 URDF/STL；来源按描述包 NOTICE 保留。

## 公共层与设备契约适配

- 场景构建器新增受信 FR3 包资源解析、base_link 子树与 flange frame，装配不能传任意 MJCF 路径。
- 对外仍为 joint1..joint6 / rad，descriptor 用 mjcf_name 明确映射到 j1..j6；组合时加 instance 前缀，两台 FR3 可共用一个场景。
- 部署收紧限位写入 MJCF joint range 和 position ctrlrange。公共模拟器在初始化和提交时按模型范围检查有限六轴目标，错误输入不会进入物理线程。
- 目前 FR3 sim 只接受原生符号全 +1 / 零偏全 0，非原生映射显式拒绝；真实后端的轴映射配置不应直接复制给仿真以免 qpos/IK 含义不同。
- FR3 严格/微分 controller 增加 sim 后端支持，仍使用同一 URDF solver、异步 worker、速度与领先约束。没有调用 SDK IK 或厂商连接。
- 状态为 source=simulation、synthetic=true、measured=false，含 simulation_time_s / simulation_mode；图像为同场景 JPEG，不能作为实机测量。

## 启动与 Web

```bash
source /opt/ros/humble/setup.bash
# 使用兼容 rclpy 的系统 Python，同时安装项目 Web 和 Pinocchio 依赖
python3 -m pip install 'mujoco>=3.2,<4' Pillow
colcon build --symlink-install --packages-up-to fr3_mujoco_sim
source install/setup.bash
ros2 launch fr3_mujoco_sim fr3_mujoco_sim.launch.py
# 或：ros2 launch morla_fr3_driver fr3_sim.launch.py
```

默认 kinematic/web，fake Quest；打开 [http://127.0.0.1:8080/](http://127.0.0.1:8080/)。独立入口自动保存并启动至 READY_DISARMED，普通 Web 选“FAIRINO FR3 / Quest（MuJoCo 仿真）”后检查、保存、启动。已有会话恢复时显示对应 FR3 仿真预设，不误显示 CR5。新增 sim 能力和 MJCF 映射改变了 descriptor 冻结摘要，旧 resolved 应从原 Assembly/Deployment 重新检查并保存/编译。

`sim_mode:=physics` / `sim_display:=web|local|both` / `input_backend:=real` 等参数和 CR5 共用。运动学/动力学与 strict/differential IK 是独立选项。切换动力学模式须撤销许可并停止录制/策略；模式切换保持当前姿态、清待处理目标与速度。显示切换共用状态与时钟，local 窗口在服务主机打开，不在远程浏览器电脑打开。Linux 离屏可使用 `MUJOCO_GL=egl`；本地窗口另需图形桌面的 DISPLAY/XAUTHORITY。macOS local 需要同环境 mjpython。

模板保留 FR3 的 right_hold deadman、0.25 位移倍率、0.2 rad/s 关节速度与 0.08 rad 指令领先。fake Quest 不产生真实手部输入，无新鲜输入时“开始遥操”仍会拒绝。真实 Quest 可只切换输入为 real，机械臂保持 sim；先确认接收协议、坐标标定和 TCP。

相机预览支持已有旋转/平移/缩放/复位，采集为 state=6/action=6/image=1。physics 用 mj_step 产生含重力静差与跟踪延迟的反馈；kinematic 直接写 qpos 并 forward。两者均没有启用机器人网格碰撞，不能据此做避障安全结论。

只需单臂 ROS JointState 时，可编译驱动内 `config/assembly.sim.yaml`，运行 `fr3_driver.launch.py`；该路径使用公共 ROS 网关。独立 Web 仿真入口本身与 CR5 一样，不额外创建另一套 ROS 控制循环。

## 验证方法

`scripts/test_fr3.sh` 包含资产可复现、62 组位形下 MuJoCo/Pinocchio 法兰 FK（1e-10 数值容差，含 mount）、两台 FR3 的命名与映射、模式切换、参考动力学跟踪、独立反馈时间、停用保持与关闭、收紧限位/无效目标/非原生映射拒绝、strict/differential sim factory，以及真实 MORLA HTTP→Session→输入→IK→MuJoCo→HDF5 链（运动学和物理模式）。自动 Web 集成测试不渲染相机，相机用目标主机实际图形上下文另验，避免把测试替身当作渲染成功。

`scripts/test_composable.sh --layer sim` 也包含 FR3 模型/仿真专项；公共回归另覆盖 CR5、Nero、Astral 的仿真行为。ROS 主机上还验证独立 launch、安装资源、离屏相机、Web 模式与画面、本地窗口和 ROS 仿真反馈，结果与原始日志见下节。

真机轴方向/零位、SDK 兼容、TCP、运动停止、碰撞模型、动力学辨识、接触任务和 sim2real 效果未在本次验收。物理模式是仿真能力，不能替代上述现场工作。


## 本轮实际结果与证据

主机：用户指定 `loopkok@192.168.50.5`，Ubuntu 22.04 / ROS Humble / 系统 Python 3.10；本轮实际导入 MuJoCo **3.17.0**、Pinocchio **3.9.0**。Mac 的 MuJoCo 为 3.12.0、Pinocchio 为 4.1.0，结果分别记录，没有沿用上一轮版本号。

- 独立包依赖构建与最终构建均为 **36 packages finished**；安装后的 launch 启动至 READY_DISARMED，`ros2 run fr3_mujoco_sim fr3_mujoco_sim_node --help` 正常。首次包 README 尚未补齐时构建失败，原日志单独保留，最终重建通过。
- FR3 专项现为 **60 项**；FR3 + 公共仿真/模式 + CR5/场景的相关回归在 Mac 和 ROS 主机各 **83 passed**。主机完整 pure **366 passed**，fake 数据/策略/Web **28 passed**；新增 sim 层入口在 Mac **21 passed**。最终前端 **20 passed**，TypeScript/Vite 构建通过。
- 同一仿真资源被独立 Web 入口占用时，ROS driver 以 RESOURCE_BUSY 拒绝第二个拥有者。停止 Web 会话后，ROS sim 网关实际 DDS 验证六轴 JointState、0.001 rad 目标与反馈、命令/回执、重复/未知组件拒绝、急停和复位后未激活拒绝，最终 probe exit 0、driver finished cleanly。现有 smoke 脚本新增显式 `--allow-kinematic-sim`，仍拒绝 physics/real；其原假驱动错误码断言已按 SIM_DISARMED 分支适配，首次断言失败日志保留。
- 目标主机使用 EGL 实际渲染 640×480 JPEG，浏览器目视看到了 FR3 模型；界面完成 kinematic→physics、录制/停止、刷新恢复 FR3 仿真预设。local viewer 在主机 `DISPLAY=:1` 的既有桌面打开，3 秒后仍运行/error=null，切回 web 后关闭；model/时间仍由同一 Runtime 拥有，vendor SDK 未加载。
- 无 Quest 输入时，浏览器测试片段得到 **474×6 状态、470 帧图像、0 条动作**，元数据按缺少必需通道正确标 failed。这不是有效训练片段；发现页面原先无条件显示已保存后，已改为按返回 status/failure 提示完整性失败，并增加前端回归。
- 用真实 renderer 和合成标准 Quest 帧补跑完整链路：**120 帧输入被接受、60×6 状态、120×6 动作、60 帧场景图像**，meta.status=complete；SDK 未加载。该程序不使用测试替身或假相机：

```bash
MUJOCO_GL=egl python3 docs/testing/fr3_sim_camera_acceptance.py \
  --output-root /tmp/fr3-sim-camera-new-run
```

每次使用新的输出目录以保留先前证据。该验证使用实际已安装插件和 TestClient 调用同一 Web/Runtime 服务，输入是明确标记的合成 Quest，不是实体 Quest 的验收。

[证据目录](evidence/2026-11-06-fr3-sim/) 保留构建、相关/pure/fake、ROS/DDS（含预期资源拒绝与首次 probe 断言）、viewer、相机完整链路、空输入片段摘要、最终 STOPPED 状态和浏览器截图。入库文本日志仅去除行尾空格，原始主机日志保留。原始 HDF5 位于主机 `/tmp/fr3-sim-acceptance/` 下对应 episode 目录；入库的是摘要与日志。Web 测试会话、ROS driver 和临时本地窗口均已停止；目标主机继续保留编译安装结果。

## Quest 前向平移改为 FR3 +X（2026-11-06）

FR3 随包 fake/sim 模板的平移映射更新为 Quest +X→FR3 +Y、+Y→+Z、+Z→+X；位移倍率仍为 0.25。新增绑定参数 `input_position_rotation: [[0,0,1],[1,0,0],[0,1,0]]`，仅作用于位置增量。原 `input_rotation` 继续负责姿态增量的基变换，避免仅翻转前向平移时意外改变旋转跟随方向。

该操作映射与坐标系手性转换分开：前者允许正交的行列式 +1/-1 矩阵；原 Quest Unity 姿态基变换的 det=-1 校验保留。未声明新参数的历史装配沿用原有平移行为。已有保存配置需在绑定参数中增加该矩阵并保存新 revision，或重新加载 FR3 预设；不会静默覆盖用户标定。
