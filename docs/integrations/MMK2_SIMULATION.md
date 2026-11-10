# AIRBOT MMK2 整机仿真移植

日期：2026-11-09；2026-11-10 补齐实际 Web 验收与整机七组控制，同日晚上统一仿真/实机的微分 IK 与限位保持策略。实现基于设备接入指南、PLUGIN_DRIVEN_PORTING、FR3_PORTING_TUTORIAL，以及 FR3/CR5 公共场景经验。新增 `src/robots/airbot/mmk2_description` 和 `mmk2_mujoco_sim`，提交与推送结果见 [发布日志](MMK2_PUBLISH_LOG_2026-11-10.md)。

可运行入口与端口语义见 [仿真包 README](../../src/robots/airbot/mmk2_mujoco_sim/README.md)，模型来源见 [description README](../../src/robots/airbot/mmk2_description/README.md)。

## 资源选择

用户提供的 airbot_mmk2_description 有完整 URDF、视觉和碰撞网格。SDK 0.0.4.dev5 的 MuJoCo XML 引用 `models/meshes`，但所供源码没有此目录；SDK 的 head_pitch 符号、arm 命名、夹爪 tendon 和底盘参数也与 URDF 不同。因此运行模型从 URDF 确定性生成，SDK XML 原样保留作为参考，绝不把缺失资源静默换成别的型号。

URDF 的原包名 URI 在运行副本归一为 mmk2_description；原件保存于 `urdf/airbot_mmk2.source.urdf`。保留 MIT 和 SDK Apache-2.0 许可，PROVENANCE.json 记录 SHA-256。模型包含 24 个标量自由度、22 个执行器及 4 个相机。七个公开执行组件合计 19 维：底盘 2、升降 1、头部 2、每臂 6、每夹爪 1。其余自由度为底盘平面 x/y/yaw 与两个 mimic 手指。

## 公共层与机器人自身的边界

- 静态 descriptor、ament `morla_components` 索引提供九类设备组件、一个底盘控制器和三个 Web 模板；目录发现不导入 SDK。MMK2 只有 sim 后端。
- 公共层新增品牌无关 `morla.simulation_scenes` 入口扩展。MORLA 保留资源锁、会话、显式 prepare/activate、命令网关、停止、ROS 私有桥、Web 和录制职责。
- MMK2 provider 拥有模型、异构执行器、差速平面底盘、mimic、机载相机、IMU、里程计和激光测距。所有组件共用一个 MuJoCo 时钟/模型/采样线程；相机使用同一姿态快照进行离屏渲染，本地 viewer 只消费快照。
- 指令严格校验名称顺序、模式、单位、维数、有限值和范围；支持公共绑定发送的逐关节单位列表。底盘停止归零速度，位置组保持最后目标；底盘另有默认 .25 s 轮速刷新超时。停用后切模式原子清除待处理命令；轮角不会变成轮速目标。
- 公共采集优先使用端口 sample_ns，保留低频激光的真实采样时间，防止读缓存制造新帧。
- 公共编译器按速度单位将 base.velocity_target 标为 velocity，去除轮角→速度的错误 state 映射。默认 schema state/action 均为 19 维，辅助流保留传感器。当前公共策略/回放只支持位置和闭合比例，不能直接执行这个包含轮速的整机 action 布局；双臂位置策略可另建不含底盘动作的 capture 布局。
- 双臂提供 `morla.kinematics` FK/DLS IK，使用各臂基座到 gripper_center。Quest 模板各侧 Unity 基变换由 base→slide→arm 完整固定旋转链推导，保持机器人 body 下的前/左/上操作方向；未将 FR3 安装坐标复制过来。默认 fake 输入，grip deadman，显式激活。

## 遥操初始位姿与单拍限幅（2026-11-10）

2026-11-10 的一段 112 s 整机仿真录制暴露出“平移基本动不了、一动就撞关节限位”。逐帧核对后分成两件事：

- **平移链路本身是对的**：`position_scale=0.25`，实测位移满足 `Δp_base = 0.25·U·Δp_unity`，拟合残差约 1e-16，方向没有问题；手感上“移不动”来自 0.25 的增益和松手重锚的语义，不是坐标矩阵错了。
- **真正的故障是初始位姿落在腕部奇异点**：当时工作位为左右 `[0,-1.11,1.85,±1.53,0,0]`，q5=0 时第 4、6 轴共线，Jacobian 最小奇异值 σ_min≈1e-6、条件数约 1.6e6。DLS 解在等效分支 `(q4−π, −q5, q6+π)` 之间来回切换，左臂 q4 有 37% 的帧被顶在 +2.96 rad 限位，单拍关节跳最大 1.54 rad。

当前推荐工作位：左 `[0,-0.8,0.8,1.64,0.8,0]`、右 `[0,-0.8,0.8,-1.64,-0.8,0]`。按求解器同样的 0.3 旋转权重，该姿态 σ_min=0.057、条件数 16（旋转权 1.0 时为 0.089/21.2），各关节距模型限位至少 0.887 rad，TCP 位于臂基座系 `(0.3676, ∓0.0127, 0.5511)` m，不奇异。工作位在 Web「工作位」里填写后保存为新 revision；仓库侧模板默认仍是 `[0,-0.6,1.2,0,0.3,0]`（条件数 31，同样不奇异）。

同期修复参数问题：MMK2 绑定里一直写着 `max_joint_step: 0.02`、`max_joint_velocity: 0.2`，但公共 `morla.cartesian_delta_ik` 从未读取，整位姿 DLS 因此没有被按拍限幅。现在 controller 在 solver 返回后把命令夹到 `min(0.02, 0.2·dt)`（50 Hz 下 0.004 rad/拍），并在报告里输出 `max_joint_step_rad` / `joint_step_clamped`。参数不会因为写在模板里就生效，这个问题和自查方法见[设备接入指南](../../device_integration_guide.md) §5.1。

## 腕部到限位时的保持策略（2026-11-10 晚）

第二轮整机录制里右臂仍有 137 拍（3.3%）被丢弃、最长一次冻结 1.86 s，q4/q5 贴住限位。原因不在坐标链，而在**仿真 solver 与实机驱动用了两套策略**：

- 旧仿真 `mmk2_mujoco_sim.kinematics` 迭代到 `1e-4 m / 1e-3 rad` 容差，每轮把 q 硬夹到关节范围，35 ms 预算到期就返回 `None`。到限位后残差不再下降，循环空转到超时，公共 controller 把该拍记为 `held` 丢弃；操作员继续转手，目标继续积累，于是松手前一直冻结。
- 实机驱动 `morla_mmk2_driver.kinematics` 与 CR5/FR3 的微分 solver 早已是另一套：每拍只做一次带阻尼的 DLS 更新，把有效目标吸收在 `position_lead_m` / `rotation_lead_rad` 之内，用 alpha 回溯（1、0.5、0.25、0.125）找能降低代价的步长，夹到关节范围；没有能改进的候选就**保持当前关节**并报告 `limit_pressure_rad`。它从不返回 `None`，因为“这一拍动不了”本来就是遥操作的有效答案。

现在仿真与实机共用同一套策略，也共用同一组实例参数（descriptor 默认值与原字面量一致，行为不变）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `position_lead_m` | 0.02 | 每拍有效平移目标相对当前 TCP 的最大超前 |
| `rotation_lead_rad` | 0.15 | 每拍有效旋转目标相对当前姿态的最大超前 |
| `damping_min` | 0.005 | 远离奇异时的 DLS 阻尼 |
| `damping_max` | 0.145 | 奇异点的 DLS 阻尼 |
| `singularity_threshold` | 0.05 | `σ_min` 低于该值时阻尼开始增加 |

吸收式领先的另一个作用：超出的需求被丢弃而不是累积，`previous_raw` 仍保存操作员的原始目标，所以在限位处一反向，下一拍立刻见效，不需要先把多余的目标“退回去”。

同轮排查发现公共 `BindingController` 的一个真实缺陷，与 IK 策略无关但会让双臂同时丢帧：新鲜度是用**处理到该绑定那一刻**的时钟判断的。第一帧要为左臂惰性构建 MuJoCo 模型（本机实测约 0.34 s），轮到右臂时同一帧已被判成超过 `max_age_ms: 250` 而过期，于是右臂整帧丢弃。`test_dual_quest_binding_uses_public_ik_and_units` 在 HEAD 上就稳定失败，与本次 solver 改动无关。现在 `BindingEngine._targets` 在帧进入时取一次时间，deadman 与 `controller.step` 都用它判断新鲜度：过期指“样本到达时就已过期”，不包含我们自己的构建/求解耗时。

这是“平滑饱和并保持”，不是“到达目标”。工作位仍应避开限位，腕部到限位时会停住直到反向，这是设计内行为。

## 按下 Grip 时双臂先“降一下”（2026-11-10 夜）

操作员报告：工作站启动、恢复发送后按住两个手柄的 Grip，仿真的两个臂先整体降一下才开始遥操；FR3/CR5 按住 Grip 不会突变。追踪最新一轮录制（`20261110T145258Z_…_a2f7885`）后确认**不是双臂 IK 引起的**：

- 第一个 `left_arm` 控制拍的目标 TCP 与实测 TCP 只差 0.1 mm，之后 0.6 s 内关节变化 ≤0.05 rad、TCP ≈0.2 mm，全段无跳变；
- `handoff` 在 55.51 s，55.552 s 第一拍升降命令就是 `[0.1]`，55.573 s 升降反馈从 `0.0` 变成 `0.1`。

原因是整机模板把 `quest.left_scalar`（扳机）同时接到 `left_gripper`（`[0.04, 0]`）和 `lift`（`[0.1, 0.2]`），而 `morla.scalar_map` 写的是**绝对位置**：扳机静止为 0，所以按下 Grip、deadman 生效的那一拍，升降目标直接是 `0.1 m`。厂商 URDF 的 `slide_joint` 是 `origin rpy="3.1416 0 1.5708"`、`axis 0 0 1`，旋转后世界方向是 **−z**，也就是正值向下；实测 `slide=0.1` 时 `slide_link` 与左右 gripper_center 的世界 z 都下降 0.10 m。所以操作员看到的“两个臂降一下”是躯干整体下移 10 cm，双臂相对躯干没有动。

修法：新增 `morla.scalar_anchored`（Web 标签“线性映射（锚定）”）。它在 engage 时锁存实测位置与扳机值，只下发之后的变化量，松开再按重新锁存；`output_range` 由“绝对位置”变成“整段输入对应的相对行程”，因此按下 Grip 不产生任何阶跃。整机模板与 SDK 驱动的示例配置已改为该 controller；夹爪仍保留 `scalar_map`（开合是绝对状态）。已有的保存装配不会自动迁移，需要在 Web 里把升降绑定的 controller 改成“线性映射（锚定）”并保存新 revision，或直接删掉该绑定。

## 本机验证

测试脚本：

```bash
MORLA_PYTHON=/path/to/python bash scripts/test_mmk2.sh
```

本次使用 macOS / Python 3.13 / MuJoCo 3.12.0，使用本机现有 Python 入口安装仿真包 wheel，并使用已有官方 ament_index_python 1.4.1 读取真实索引。没有声称 ROS 2 已在 Mac 运行。已验证从 site-packages 加载 provider，并在按 CMake 安装规则暂存的 description share 中解析全部资产后启动 12 个实例、四路相机；这属于安装布局冒烟，不等于 colcon 构建。

- MMK2 24 项：模型/19 维契约、同场景身份、生命周期、名称/单位/模式/有限数/限位拒绝、底盘移动和停止、模式门控与轮速清零、物理伺服、双指联动、传感器维度、左右 FK/IK 与独立 URDF 变换相符、吸收式领先与限位保持、真实安装工厂发现、Web 目录/模板/编译/保存/启动/HDF5 录制、四视角实际 JPEG/float32 深度渲染、双手 InputStream→公共 BindingEngine/IK→Runtime 命令、provider 缺失/冲突拒绝、初始化失败资源释放、轮速绑定/超时停止、完整坐标链及头部镜头位置。
- FR3 70 项专项回归通过。
- 新一轮联合回归涵盖 MMK2、FR3/CR5、公共 runtime/core/contracts/teleop、Web composable/operator/plugin：**355 passed、1 skipped**。跳过项缺少可选 Hypothesis。完整结果见 [Web 阶段回归日志](evidence/2026-11-09-mmk2-web/regression.log)，计数不能与专项结果相加。
- 实际四相机 RGB 与深度图均有限且非空。macOS OpenGL 报告 ARB_clip_control 不可用，远端深度精度受限；这不是传感器标定证据。

完整输出见 [联合测试日志](evidence/2026-11-09-mmk2-local/tests.log)、[wheel 安装日志](evidence/2026-11-09-mmk2-local/wheel-install.log)、[安装布局冒烟](evidence/2026-11-09-mmk2-local/installed-scene.log)。

## 尚未验收的内容

本地实际浏览器已完成配置、检查、启动、七组遥操、录制、Grip 释放、暂停、急停/复位、模式切换及停止，见 [Web 验收报告](MMK2_WEB_ACCEPTANCE_2026-11-10.md)。Ubuntu/ROS 2 Humble 的 colcon 编译和真实 DDS 尚未在目标机执行，本地浏览器验证不等于目标机验收。

底盘是平面约束速度伺服模型，半径 .08401 m 根据 URDF 轮轴高度假定、轮距 .3265 m 来自关节安装位置，未标定真实差速参数，也未模拟轮地接触/滑移。机器人视觉网格不参与碰撞。IMU/相机/激光外参、惯量和伺服参数未做实机辨识，不能据此宣称避碰、接触动力学或 sim2real。

默认采集三路机载 RGB，深度由 read 的 depth 端口返回 32FC1 原始米制数据，尚未接通公共深度数据集导出。激光只扫描世界 group 0 几何，10 Hz、180 点、10 m 上限，不含噪声模型。

该交付是整机仿真接入，不包含真实 MMK2 SDK 通信、使能或实机运动；SDK 与 URDF 差异必须在后续真机移植中逐项核验。
