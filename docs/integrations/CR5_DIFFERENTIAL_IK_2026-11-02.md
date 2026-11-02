# CR5 连续微分 IK 优化记录

日期：2026-11-02。在当前未提交的公共仿真模式/Web 改动之上继续实现，保留这些工作。设计借鉴桌面 vr-teleop-kit（本地 HEAD `8d37f3d`）的自适应阻尼、目标领先限制与超量吸收；新代码使用 MORLA 自己的 CR5 URDF/Pinocchio 完整六维模型，没有移植 DK1 的 3+3 分组、硬编码腕部锚点或工具几何，也没有修改该桌面项目。

## 算法与控制链

新增 `morla_cr5_driver/differential_ik.py`，继承现有 IKSolver 的模型绑定、硬件关节 signs/offsets、FK 和部署限位交集。

- 一次输入执行完整六维 SE(3) Jacobian 的单步 DLS 更新；六轴共同满足位置和方向，保留 CR5 的腕部位置耦合。
- 使用加权 Jacobian 最小奇异值计算接近度，并将阻尼从 damping_min 平滑提高到 damping_max。接近奇异点时限制弱方向更新，不保证能穿越或逃离精确奇异位形。
- 求解前限制有效目标领先当前反馈位置、方向的幅度。分别保存原始目标和有效目标，只把下一帧的原始目标增量累计到有效目标；越界超量被吸收，反向操作可立即减少有效需求。
- 更新受 differential_step_rad、关节限位和线搜索约束，只接受降低加权位姿误差的步。没有改善或超时则返回当前关节保持；非有限输入、模型/数值错误仍是异常，不伪装成正常跟随。
- 沿用异步单工作线程、最新帧覆盖、输入平滑、反馈播种、TCP/输入坐标映射、generation 重置和最终按 dt 的关节限速。没有增加控制器远程 IK、硬件写入通道或碰撞规避。
- 重锚时在工作线程中清除原始目标/有效目标和诊断，旧 generation 的结果仍不得回流。

现有 `urdf_ik.IKSolver.solve()` 严格迭代算法保持。CR5 专用 `morla.cr5.cartesian_delta` / `morla.cr5.cartesian_jog` 默认选择 differential；可显式选择 strict。通用 morla.kinematics provider、独立 kinematics_check 仍按原有实现工作，不自动改变成近似求解器。

## 参数与切换

在绑定 parameters 中配置：

```yaml
ik_mode: differential       # strict 保留原精确位姿求解
position_lead_m: 0.05       # 有效目标最大位置领先，最大允许 0.25 m
rotation_lead_rad: 0.35     # 有效目标最大方向领先，最大允许 1 rad
singularity_threshold: 0.05 # 加权 Jacobian 最小奇异值的阻尼起始阈值
# 阻尼是上述加权任务下的算法参数，不是已辨识的实机参数。
damping_min: 0.005
damping_max: 0.15
differential_step_rad: 0.04 # 内部单次更新上限，最大允许 0.15 rad
max_joint_velocity: 0.3     # 最终下发速度仍按 dt 限制
```

内部更新上限同时受 max_joint_step 限制；damping_min 不得大于 damping_max。strict 模式不使用这些微分模式参数，保留原信赖域/容差/求解预算。Quest、Vader 的 fake/sim 模板显式注明 differential，CR5 独立 sim 包配置同步。

Web「组合配置」的 CR5 控制绑定新增「IK 模式」选择。选择后保存下一版本，停止旧会话并启动新版本生效；不在线替换运行中的求解器。IK 模式与仿真的 kinematic/physics 是独立选择：两种 IK 均可用于两种仿真。

目标吸收刻意牺牲部分手柄与末端的绝对对应。快速操作、不可达或物理跟踪滞后时，都可能吸收位移/旋转；手停后不会追赶已丢弃的超量。当前 Quest 连续跟随绑定不靠握把重锚，需要显式撤销/重新激活，或按已有输入断流重锚机制重新建立对应。该策略适合遥操，不等同于精确点位执行。

## 诊断

AsyncCartesianController 发布线程安全的报告快照。统一 Web session state 增加 controllers[binding_id][plugin_id]，控制页展示候选关节 FK 的原始目标残差、奇异点接近度、阻尼、已吸收位移与报告龄期。独立 CR5 VR 控制页也在诊断 JSON 中包含该字段。

- accepted 表示得到有限、限位内的候选更新/保持，不代表目标已到达。
- converged 针对原始目标；bounded_target_converged 针对限制后的有效目标。两者分开，避免把吸收目标后的保持误报为原始目标已达标。
- position_error_m / angle_error_rad 是相对有效目标的候选 FK 残差；raw_* 是相对原始目标的残差。不是实机实际 TCP 测量误差。
- absorbed_position_m / absorbed_angle_rad 是原始目标与有效目标的差。
- reason 区分 tracking、holding、deadline_hold、absorbed_target_hold、converged；error 保留真正异常。
- 奇异点接近度是加权 Jacobian 阈值下的算法指标，不能当作物理安全距离。

## 本地验证

相关回归：**67 passed，1 deselected，7.30 s**；排除项仍是前一轮已在 HEAD 基线复现的手机输入 `test_mobile_input_and_recording`（release_or_reanchor_required），没有为此次优化修改其控制语义。

新增 8 项差分 IK 用例，覆盖：平滑 30° 腕部转动的位置/方向跟踪及前三轴补偿；5 m 远目标吸收后反向 1 cm 立即作用；170° 方向需求的限制与回退；奇异位形自适应阻尼及无输入静止；不可达/关节限位/超时保持/NaN 拒绝；Quest、jog 的 strict/differential 选择及非法参数拒绝；非单位硬件 signs/offsets 和重锚清零。既有 Quest 组合控制测试额外校验 Web state 中的求解器诊断。

测试范围包括：CR5 driver 全部测试、公共 CartesianController、公共仿真模式、公共物理运行、CR5/多机器人 MJCF、Web API。前端 TypeScript/Vite 构建通过，更新 web/dist。

物理闭环短时冒烟：CR5 初始准备姿态，physics 模式；异步控制器以 50 Hz 发送 160 帧、5 mm 平移输入，最终输出限速 0.3 rad/s。每帧命令增量均不超过 0.006 rad，状态有限、worker_error=null；末帧候选 FK 对原始目标的位置残差约 0.0181 mm。该值不是实体设备精度或长稳结果。

同一 URDF 的简单离线对照：250 帧，前 100 帧平滑增加 10 mm / 5 mm 平移及 X 轴 10° 转动，再保持；两种模式均使用相同 50 Hz、1 rad/s 输出限制。两者均无 None、微分模式没有触发目标领先限制。一次本机测量：strict 求解 P95 约 0.104 ms，differential 约 0.241 ms；最终位置残差分别约 0.000475 mm 和 0.000000036 mm。小幅可达输入下 strict 可以快速通过容差检查，微分模式额外计算 SVD/诊断，并非所有场景更快。此对照只作本机检查，没有构成实机性能基准。

## 未验收

未执行实体 Quest/Vader→真实 CR5、本轮 ROS Humble colcon/launch、长时负载或碰撞/物理参数验收。微分模式持续返回近似更新不等于位姿已达标；严格模式也不提供碰撞规避。不得将 kit 的按帧速度上限或 DK1 几何参数直接用于 CR5。
