# MMK2 整机仿真

完整整机模型与公共 MORLA Runtime/Web/采集接入。七个执行组件为 base、lift、head、left_arm、right_arm、left_gripper、right_gripper；三台机载 RGB-D 相机加 scene 相机、里程计、IMU 和激光测距共用唯一 MuJoCo 场景。只声明 sim 后端，不连接实机 SDK。

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install 'mujoco>=3.2,<4' Pillow
colcon build --symlink-install --packages-up-to mmk2_mujoco_sim
source install/setup.bash
ros2 launch mmk2_mujoco_sim mmk2_mujoco_sim.launch.py
# physics 为参考伺服动力学；both 同时启用本地窗口
ros2 launch mmk2_mujoco_sim mmk2_mujoco_sim.launch.py sim_mode:=physics sim_display:=both
```

打开 http://127.0.0.1:8080；独立入口 `ros2 run mmk2_mujoco_sim mmk2_mujoco_sim_node` 复用公共 session_cli。公共 Web 通过静态 descriptor 发现 `mmk2-sim` 预设，模板启动为 READY_DISARMED，显式 prepare/activate 后才接受公共网关命令。现有 `system.launch.py` 可加载本模板编译出的 resolved 快照提供 ROS 私有网关。

公共层新增品牌无关 `morla.simulation_scenes` 工厂扩展，由 descriptor 的 `geometry.simulation.provider` 选择已安装入口。MMK2 的异构执行器和传感器由自己的 scene provider 实现，公共层仍负责资源锁、生命周期、控制权、录制、Web 和 ROS 网关。没有第二条公开运动写入口。输入后端参数仅对装配中已有输入生效；默认整机模板不含输入，不会自动运动。

接口：

- 双臂与头部 position_target：固定具名顺序、rad、position；升降与夹爪为 m。夹爪数值为每侧指行程 0..0.04 m，总开口约为 2 倍行程；不是闭合比例。
- base.velocity_target：`[lft_wheel_joint,rgt_wheel_joint]`，rad/s，velocity，±8 rad/s。base.position 反馈为无界轮角 rad，velocity 为轮速 rad/s；descriptor 的 ±8 表示命令速度范围，不表示轮角限位。正速度前进，右轮快于左轮时逆时针转向。
- 底盘差速控制器 `morla.mmk2.differential_drive` 接受具名 x/y 摇杆输入：x 右转、y 前进，默认线速度 .08 m/s、转向 .3 rad/s。轮速命令超过 `command_timeout_s`（默认 .25 s）未刷新时归零，松开 Grip 后不会无限保留移动速度。
- 底盘 stop 清待处理轮速并设速度目标为 0；位置组件 stop 保持最后目标。physics 制动具有伺服减速过程。模式切换要求撤销运动许可，不重放旧速度。
- IMU gyro 为 rad/s，acceleration 为 m/s²（含加速度计重力响应），odometry 为 `[x,y,z,qx,qy,qz,qw]`。lidar 为 -π 起每 2° 一点，10 Hz，上限 10 m，无回波取 10；同一帧缓存保留原采样时间。
- rgb 为 JPEG；depth 为小端 float32 原始深度、m、编码 32FC1，包含 width/height。默认采集模板录制三路 RGB、19 维 state/action 及辅助传感器；depth 可由 Adapter.read 取得，尚未接入公共深度数据集导出。
- 所有反馈 simulation/synthetic=true、measured=false。固定机载相机与 scene 相机用于采集，Web 拖动不改采集外参；自由观察用本地窗口。

公共 SchemaV3 将底盘动作标为 velocity；现有公共策略/回放执行器仅支持位置/闭合比例，默认整机 action 布局尚不能直接用于策略推理。

Quest 双臂默认使用 `morla.mmk2.cartesian_delta` 专属异步控制器；旧版公共 IK 绑定共用新的 URDF 求解器。FK/IK 均以各臂自身 `left_base_link/right_base_link` 为参考，以 gripper_center 为末端。`config/quest.sim.yaml` 提供双臂示例；`config/quest.whole.sim.yaml` 是从 Web 逐步配置得到的整机七组映射，预设名为 **AIRBOT MMK2 整机七组 / Quest（仿真）**。双臂 Unity 基变换包含 base→slide→arm 的完整固定旋转链，现场方向/工具偏置仍未标定。

双臂微分 IK 与实机驱动同策略：每拍一次带阻尼的 DLS，把需求吸收在 `position_lead_m`（默认 0.02 m）/ `rotation_lead_rad`（默认 0.35 rad，与 FR3/CR5 一致）之内，alpha 回溯找下降步长，无法改进时保持当前关节并报告 `limit_pressure_rad`，不返回 `None`。阻尼参数为 `damping_min` 0.005、`damping_max` 0.145，`singularity_threshold` 0.05；五个值都是实例参数，未配置时用 descriptor 默认值，其中 `rotation_lead_rad` 在设备页可改。绑定侧另有 `smoothing_alpha`（输入低通，1=关闭，模板 0.35）和 `rotation_scale`（旋转增量倍率，默认 1.0=1:1）。
有效目标每拍都从原始目标重新推导（`goal = raw` 后再做领先截断），从不跨拍积分：手不动时增量为零，积分写法会把目标冻在“上一拍手臂位置 + 领先距离”，手臂永久停在离手 2 cm / 0.35 rad 的偏置上，误差再也不回零。
贴限位的关节用 active-set 处理：把仍在向外推的贴限位关节锁住重解，其余关节承担步长，被拒绝的向外需求记入 `limit_pressure_rad`；否则 DLS 把步长花在动不了的关节上，回溯全部失败而楔死。腕部贴限位时平滑停住直到反向，不是收敛到目标。
Quest 模板的输出上限为 0.05 rad / 2.0 rad/s，专属求解器的差分步长默认上限为 0.02 rad，可通过绑定的 `differential_step_rad` 配置。

整机模板的升降绑定用 `morla.scalar_anchored` 而不是 `morla.scalar_map`：扳机是相对量，按下 Grip 时锁存当前升降高度，只下发之后的行程。用绝对映射时，扳机静止为 0、映射起点为 `0.1 m`，deadman 一生效躯干就瞬降 10 cm（厂商 URDF 的 `slide_joint` 正值朝下），双臂看起来“先降一下”才开始遥操。夹爪仍是 `scalar_map`（开合是绝对状态）。
锚定映射的 `output_range` 是**扳机满行程对应的位移**，不是绝对位置，所以必须显式给出：Web 切换控制器时会丢失未在 descriptor 里声明的参数，丢掉量程后默认 `[0,1]` 就是 1 m 行程，按住扳机直接把躯干开到底。现在两个 scalar 控制器都在 `controllers.json` 声明 `input_range`/`output_range`（切换时保留、编辑器始终显示），`AnchoredScalarController` 缺量程直接报错，编译器以 `ANCHORED_OUTPUT_RANGE` 拒绝。升降建议 `input_range [0,1]`、`output_range [0.1,0.2]`。

Quest 测试后端本身不会主动生成动作。要复现本地 Web 验证，从仓库根目录运行 `python3 scripts/run_mmk2_web_acceptance.py --store /tmp/mmk2-web-demo`，在 Web 载入上述预设并逐步检查、保存、启动及激活。该独立测试宿主只向已经由页面创建的 MMK2 会话中的 fake Quest 注入 30 Hz 合成输入，不自动激活，不连接硬件。测试开关保存在该 store 下的 quest_fixture.json；hold=false 模拟松开 Grip，enabled=false 模拟输入停发。

本地 Web 验证记录见 [逐步验收说明](../../../../docs/integrations/MMK2_WEB_ACCEPTANCE_2026-11-10.md)。普通启动入口不会隐式启用测试数据生成器。

模型与限制见 mmk2_description/README.md。SDK 原模型无法直接加载，未声称兼容真实 MMK2 控制器；底盘为平面约束参考模型，视觉 mesh 无机器人接触，传感器外参为假定，均非 sim2real 验收。macOS 本地窗口需要同环境 mjpython；Linux 网页离屏可设置 MUJOCO_GL=egl。

测试前需完成包安装并 source 安装环境，以验证真实入口工厂；仅添加 PYTHONPATH 不会注册 entry points。验证入口：`MORLA_PYTHON=/path/to/python bash scripts/test_mmk2.sh`。本机测试和 ROS/目标机验收范围见 docs/integrations/MMK2_SIMULATION.md。
