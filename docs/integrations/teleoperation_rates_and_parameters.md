# 遥操频率与参数核对（2026-10-29）

以下为当前源码与仓库配置，不是现场测频结果。Profile、launch 参数和现场部署可覆盖；定时器频率、硬件采样率、有效新目标率和网页刷新率必须分别判断。本轮只核对并记录，没有修改 Nero/Astral 的频率或调参。

## 机械臂链路

- Nero 旧专用节点：遥操循环 `control_rate_hz=125`（8 ms），目标不变时复用解；驱动调度125 Hz，`state_rate_hz=20`（50 ms），遥操报告2 Hz、驱动报告5 Hz。SDK 单工作线程/调用耗时和待处理目标会影响实际发送率。依据 `nero_arm_teleop/config/nero_teleop_{left,right}.yaml`、`nero_robot_control/config/nero_driver_{left,right}.yaml`。
- Astral 旧专用节点：YAML 覆盖遥操 `control_rate=150`（约6.67 ms），当前 geometric；驱动定时器100 Hz、板卡 `ctrl_hz=100`、`obs_hz=100`、ROS状态100 Hz，遥操报告5 Hz。源码中的50 Hz默认值不是加载当前YAML后的值。依据 `astral_arm_teleop/config/astral_arm_teleop_{left,right}.yaml`、`astral_robot_control/config/astral_robot.yaml`。
- CR5 新组合链：输入 pose 事件触发 IK，没有固定IK定时器；后台单槽保留最新任务，后续 pose 回调读取已完成解，可能额外等待一次输入间隔。VR Assembly 的 `rate_hz=50` 控制 Adapter I/O；实现为写/读处理后等待20 ms，实际周期包含I/O耗时，并非固定50 Hz。控制器二进制反馈独立接收，源频率未实测；缓存更新和源反馈频率也不相同。普通独立驱动 descriptor 默认20 Hz，VR Assembly覆盖50 Hz。
- 公共组合 CartesianController：随输入调用；绑定 Nero/Astral 不会自动启动旧专用节点的125/150 Hz定时器。FK通常与所在求解调用共用调度，没有另设固定频率。

## 输入、手、显示与数采

- Quest：接收事件驱动，健康检查20 Hz。仓库接收器不能证明APK在固定90 Hz推流；需测源与ROS到达率。
- XHand：重定向随关键点输入；Nero/XHand启动链覆盖原生 `update_rate=50`，原生源码默认10；Adapter状态报告2 Hz。
- Wuji：重定向随输入；关节状态发布配置1000 Hz，诊断10 Hz；不代表1000个独立硬件新样本，也不代表重定向1000 Hz。
- Astral夹爪：目标50 Hz、状态报告5 Hz，再经Astral驱动下发。
- LoopHand：Adapter I/O默认20 Hz；real仍受停止/watchdog验收限制。
- 公共runtime ROS bridge：tick与状态/健康定时器50 Hz。Web宿主每轮处理后等待20 ms，实际低于或接近50 Hz。
- WebSocket状态每轮约100 ms；CR5页面请求完成后等待200 ms，显示低于或接近5 Hz；均不是控制频率。
- 数采常见30 FPS，独立于控制周期。组合宿主用tick检查采样时机、没有严格定周期补帧，不能保证实际30 FPS。相机源频率由选中设备和参数决定。
- 语音/脚踏意图路由检查20 Hz，来源状态报告1 Hz，连续关节运动不由它生成。
- 旧Nero/Astral仿真状态常见50 Hz，物理积分由模型timestep决定，不代表真机频率。

## 主要遥操参数

位置/坐标：Nero/Astral的 `motion_scale`、输入到基座旋转、`flip_pitch`、TCP变换；CR5的 `position_scale=0.25`、`input_rotation`、`solver_tip_from_tool`、`urdf_joint_signs/offsets`。建锚、切换来源和实测播种影响起步连续性。

滤波系数不能跨包直接比较：

- Nero `pos_smoothing/rot_smoothing=0.8`，越大越迟缓，按控制拍更新，改变频率会改变滤波时间特性。
- Astral左臂配置位置0.25、旋转0.35，越大越平滑；内部转为时间常数。板卡 `lpf_enable=true/lpf_alpha=0.85` 则越大越直接。
- CR5 `smoothing_alpha=0.35` 越大越直接，按worker处理样本更新，没有固定时间常数换算。
- Quest landmarks `ema_alpha=0.7`；CR5捏合保持使用原始关键点。Wuji命令滤波截止10 Hz；手重定向另有平滑参数。

速度/步长：Nero遥操 `max_joint_delta_rad_per_step=0.065`，驱动 `max_joint_velocity_rad_s=1.0`，另有 `stream_position_mode/stream_j_speed_percent`。Astral左臂 `max_joint_vel=6.0 rad/s`。CR5 controller速度0.3 rad/s、步长0.15 rad，Adapter相对实测步长0.05 rad；ServoJ当前字面参数 `t=0.1,lookahead_time=50,gain=500` 固定在代码，固件兼容/实际效果待验证。各层关节限位与工作空间共同约束最终动作。

IK：Nero/Astral解析/几何路径的局部搜索窗口、连续性评分、分支滞回/回退影响跳解和耗时；Astral另有 `reach_margin`、人肘模式/权重/时效/平滑和escape/return帧数。`ik_max_iter/ik_tol/ik_w_pos/ik_w_ori/ik_w_reg/ik_dq_max` 等主要作用于Astral urdf_numerical后端，当前geometric不靠这些LM参数调手感。

CR5数值IK：100次迭代、50 ms预算、阻尼与线搜索、±0.3 rad局部域、0.06 rad迭代步长、0.2 mm/0.002 rad FK接受门槛，多数为代码常量。公共CartesianController默认求解门槛50 ms；CR5外层完整任务250 ms、结果年龄300 ms。超时门槛不是控制周期。

时效：Nero输入/驱动命令/反馈常见0.5 s；Astral输入/驱动命令1.5 s，人肘0.3 s。CR5位姿/保持端口分别250 ms，命令watchdog0.3 s，缓存反馈0.5 s，网络超时0.2 s。QoS、队列、TCP排队、SDK串行和调度负载同样影响延迟。

## 本地计算开销观察

开发机macOS、Python3.13、Pinocchio4.1的单次观察：2000次FK中位数0.0037 ms/P95 0.0040 ms；300组邻近目标完整IK（含检查）中位数0.0998 ms/P95 0.1357 ms、最大1.0430 ms，0失败。该数据不是机器人主机或Quest端到端延迟，也不是实时保证；原始采样未单独归档，不作为验收基准。

现场应同时测：源位姿率、有效IK结果率、真实命令下发率、独立硬件反馈率、丢帧/年龄、端到端延迟。不要仅提高某一层Hz或删除一次本地FK检查就判断跟手问题已解决。
