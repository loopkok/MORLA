# Vader 5 Pro ROS 2 与 MORLA 输入组件

组件 `morla.input.vader5@1.0.0`，位于输入层，与Quest/手机/手套同为kind=input。使用用户提供的 `src/common/vader5_sdk`（morla-vader5-sdk 0.1.0），不再实现HID协议、IMU解码、虚拟手柄或另一套机器人控制/数采框架。

本轮完成软件接线与离线验证；Linux ROS 2、USB/2.4G实体、固件、坐标方向、按键映射、端到端时延和真实机器人尚未验收。

## 能力与边界

- 摇杆/扳机、27个已解码按钮、角速度rad/s、含重力加速度m/s²、原始报告及未知位。
- ROS发布命名空间内Joy/Imu诊断和来源明确的事件流；Imu无姿态，orientation_covariance[0]=-1。没有把加速度二次积分成位置，也没有宣称它具有VR绝对6DoF定位。
- 默认是六轴笛卡尔速度点动：摇杆中立停止增量；M1（`hold_to_control`）仍作为输入意图/auxiliary发布，但runtime不再强制“按住才动”。也可用公共joint jog/scalar map绑定其他执行器。
- 默认位姿求解沿用机器人自己的IK：Nero/Astral用既有公共gamepad jog，CR5使用同一个本地URDF/Pinocchio后端及异步控制器的jog模式。输入包不依赖这些机器人包。
- 数据、训练、策略和回放继续使用相同的冻结state/action契约；不新增手柄专属模型或直接SDK运动通道。

SDK与本封装采用GPL-3.0-or-later；保留LICENSE，来源及文件哈希见PROVENANCE.json与[SDK NOTICE](../vader5_sdk/NOTICE)。SDK位于 `src/common/vader5_sdk`，与本ROS包同级，以独立Python包安装。迁移只调整目录和README安装路径，并添加COLCON_IGNORE，协议、传输与公开Python接口保持原样。

```text
src/common/
  vader5_sdk/          # 独立Python SDK，COLCON_IGNORE；由pip安装
  morla_vader5_input/  # ROS 2输入驱动与MORLA组件，由colcon构建
```

## 安装与启动

在机器人Linux/ROS 2 Humble主机、MORLA根目录，使用ROS对应Python：

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install ./src/common/vader5_sdk
colcon build --symlink-install --packages-up-to morla_vader5_input morla_cr5_driver morla_web_manager
source install/setup.bash
python3 -m morla_core.composition.cli catalog --json
```

只诊断输入不需要CR5/Web：仅构建 `--packages-up-to morla_vader5_input`。Linux hidraw后端需要USB数据线、PC模式；SDK也保留2.4G接收器路径，但本轮均未实测。不依赖完整tux-invader服务，不会创建虚拟手柄或自动卸载xpad驱动。

权限按SDK README的厂商hidraw规则配置，SSH/无桌面账号另行授予精确设备权限。关闭Steam、飞智配置工具和旧SDK读取进程。明确指定正确厂商接口，多个设备不自动猜测；可使用稳定udev符号链接，解析后必须是/dev/hidrawN。

```bash
ros2 launch morla_vader5_input vader5_input.launch.py device:=/dev/hidrawN
```

独立驱动发布：`/vader5/events`（String JSON），`/vader5_input/joy`、`/vader5_input/imu`、`/vader5_input/status`。Joy为本SDK四摇杆轴+两扳机和27按钮顺序，**不是SDL game_controller_node布局**，不能再接公共SDL /joy分支，否则丢失背键且可能重复写入。组合宿主只订阅来源事件。

默认运动采样目标50 Hz，同时发送每个按钮变化；SDK用read_batch保留批次内按下/松开，不用只取最新报告的read。每批连续运动只取最后一份，按钮边沿仍逐份处理；超过128份报告拒绝并断开，不回放积压。读超时0.1 s后发布断开并退出reader，不自动重连；重新启动产生新epoch，必须重新激活和释放deadman。

SDK的received_at是主机解码时间，并非硬件采样时间。事件流保存这个单调时钟，ROS排队不刷新年龄；本方案要求reader和宿主在同一主机。hidraw内未带可靠时间戳的排队时延仍需实测，不能用此软件声明硬实时。

## 先用真实手柄控制fake机械臂

纯fake演练（driver持续发布中立状态，不自动模拟按键或运动）：

```bash
ros2 launch morla_vader5_input vader5_teleop.launch.py
```

真实手柄、fake CR5（推荐首轮核对）：先复制 `config/input.real.example.yaml` 并填写资产身份，再运行：

```bash
ros2 launch morla_vader5_input vader5_teleop.launch.py \
  device:=/dev/hidrawN fake_input:=false \
  deployment:=/absolute/path/to/input.real.yaml
```

同一launch拉起reader及公共 `morla-composable-session`，默认装配 `config/cr5.fake.yaml`，页面http://127.0.0.1:8080/；终端打印session_id。API `GET /api/v2/active-session` 可查看活动会话与操作回执。使用现有组合Web界面/API进行prepare、激活、录制等，不依赖旧Nero启动脚本。driver和宿主都必须source当前安装环境。

`port:=8082`、`store:=/absolute/path`可覆盖；Web默认本机监听。connect_inputs只启动订阅器；本launch负责同时启动SDK driver。单独用现有Web管理器时，先启动独立reader，再编译保存Assembly、开始会话、connect_inputs。

## 默认按键与复用规则

所有分配在pad实例参数中冻结，可通过Assembly/Deployment修改，修改后重编译保存revision。配置接线时检查冲突；不通过改变设备上板载映射来实现软件配置。

- **M1保持**：人工运动deadman，激活后先释放再按下。可共同授权同源机械臂与夹爪绑定。
- **M2保持**：旋转模式；未按M2为平移模式。该键不能同时绑定采集/会话命令。
- 平移：左摇杆Y→x、左摇杆X→y、右摇杆Y→z。
- 旋转：左摇杆X→roll、左摇杆Y→pitch、右摇杆X→yaw；另一组三轴置零，避免无意同时平移。
- LT模拟扳机：精细倍率，从1.0逐渐降到0.2。RT模拟扳机：独立夹爪闭合比例0..1，只有绑定夹爪时使用。
- **START单按**：请求激活；不替代prepare/标定/实机门槛。启动时已经按住不触发，需先释放。
- **B**：暂停/撤权，显式停止当前人工或策略会话控制。
- **Y**：开始录制；**X**：结束并保存。采集动作与激活分开，长按不重复触发。
- **START+SELECT** 组合急停已随重构删除；急停请用Web/服务操作或外部硬件急停。
- A、C/Z、其余背键、LB/RB等未绑定操作时只作原始输入记录。HOME和TURBO不绑定动作，TURBO是脉冲，不可当保持键。

重构后不再检查 `deadman_button`、`rotation_button`、`action_buttons` 的角色冲突，也不再保留SELECT保留键；配置写错不会被拒绝，需自行核对。action_buttons可设为空以关闭所有离散操作。数据命令只走组合Service现有API，不同时再发旧 `/morla/operator/command`。操作request仍按输入epoch/sequence记录，但runtime不据此拒绝，submitted或RUNNING不等于已完成。

SDK坐标保持原方向；`axis_signs`按x,y,z,roll,pitch,yaw提供六个±1软件方向；`input_rotation`由机器人binding把增量对齐基座。现场逐轴验证后再运动，不默认把手柄轴当成机器人轴。

## 机器人绑定与参数

包内配置：CR5 fake整链、仅输入real覆盖、完整CR5 real模板、EPG fake整链，以及Nero/Astral公共gamepad jog示例。更换执行器只改Assembly/Deployment和controller，不修改SDK/输入驱动。

CR5默认线速度0.02 m/s、角速度0.15 rad/s、输出关节速度0.3 rad/s；LT倍率在此前提下降速。jog复用公共CartesianController及现有异步worker，采用10 µm/0.0001 rad接受精度，避免精细模式每拍小增量被原VR的0.2 mm接受容差吞掉；VR模式原默认精度不变。它仍受关节限位/步长、求解超时和硬件反馈约束。

EPG示例用公共scalar_map把RT 0..1映射到0.05..0 m。重构后scalar_map不再执行 `max_velocity` 上限，EPG驱动也没有5 mm步长限制；一次扳机大跳变会直接成为大目标跳变，需要操作上避免。

真实CR5仍需joint_lower/upper、身份/IP、输入基座/TCP变换等正确配置；`kinematics_verified`/`calibration_verified` 和部署/停止/watchdog声明不再参与门禁，只作记录。真实EPG同样不再有初始化/位置语义/停止/watchdog门禁。Nero/Astral示例只验证装配契约，真实运行需各自驱动、模型资源及部署；不声称这些示例已实机测试。

## 数采、训练、推理与回放

默认state/action仍是机器人实测/请求关节目标；手柄axes、scalar、deadman、完整按钮、IMU、raw报告和长度作为auxiliary。输入采样不冒充机器人状态，fake样本保留synthetic=true；录制时按钮变化另存input_buttons事件，短按不依赖30 FPS采样是否恰好命中。

使用既有SchemaV3录制、对齐/导出及ACT/OpenPI训练入口，冻结机器人feature/geometry和动作单位。默认auxiliary不会自动成为模型条件输入；若要把手柄意图作为训练观测，显式修改capture.state/特征契约并重做模型握手。更改按钮/轴配置会改变Assembly/Resolved身份，应重新编译；不假定input_mapping_hash单独涵盖全部输入参数。

策略/回放输出继续走同一runtime命令接口，但重构后不再有策略与人工输入的运行时互斥或租约强制；同时驱动时后写入者生效，需要上层自律。B仍可显式暂停/撤权。真实训练/模型推理服务未执行，本轮只验证本地假预测器/回放的同契约控制链。

## 验证

代码与软件验证记录见仓库 `docs/integrations/vader5.md`。Linux ROS2、实物USB/2.4G、固件/映射/IMU方向、相机同步、机器人停止/碰撞及端到端时延仍待现场验收。SDK上游约500 Hz的说明不是本机已测频率。
