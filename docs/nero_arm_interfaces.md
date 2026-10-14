# Nero 软件接口

此接口文档覆盖 P5 的 Nero 手臂 driver 与遥操作。其 ROS 话题使用统一具名接口；`pyAgxArm` 只由 `nero_robot_control` 的硬件适配器加载。ROS 构建、fake 运行和 profile 检查不要求导入该 SDK。

| 用途 | 每侧接口 | 类型/语义 |
| --- | --- | --- |
| 手臂命令 | `/{side}_arm/joint_commands` | `sensor_msgs/JointState`；必须提供严格有序契约中的 7 个 `name`，position 单位为 rad |
| 实测关节 | `/{side}_arm/joint_states` | `sensor_msgs/JointState`；SDK 回读，名字为 `{side}_joint1..7` |
| 实测末端 | `/{side}_arm/tcp_pose` | `geometry_msgs/PoseStamped`；SDK flange xyz+rpy 转换为 Pose 四元数，位置 m、姿态 xyzw |
| driver 状态 | `/{side}_arm/nero_robot_driver/status` | `std_msgs/String` JSON；backend、连接/使能、反馈新鲜度、拒绝原因和最近错误 |
| 显式使能 | `/{side}_arm/nero_robot_driver/enable` | `std_srvs/Trigger`；只在连接且七关节实测反馈新鲜时接受 |
| 显式位置动作 | `/{side}_arm/nero_robot_driver/position` | `std_srvs/Trigger`；仅在对应配置存在 `position_pose` 时可用，返回“已排队”不代表已到位 |
| 急停请求 | `/{side}_arm/nero_robot_driver/estop` | `std_srvs/Trigger`；异步请求 SDK electronic emergency stop |
| re-anchor 准备/提交 | `/{side}_arm/nero_arm_teleop/prepare_reanchor`、`.../reanchor` | `std_srvs/Trigger`；prepare 只捕获新鲜实测关节与 Quest 姿态并返回一次性 token；HUMAN 所有权消息携带 token 后才开启遥操作输出 |
| 遥操作状态 | `/{side}_arm/nero_arm_teleop/status` | `std_msgs/String` JSON；armed、所有者、新鲜度、输入 frame 和 solver |

`side` 为 `left` 或 `right`。默认 launch backend 为 `fake`；fake backend 使用配置中的显式 7 维初值，不打开 CAN。接入硬件时需显式传 `sdk_backend:=hardware`，并先按设备现场验收完成 CAN、固件、限速和运动方向检查。不得将 fake 初值当作机器人的实测姿态。

工作区内保留 SDK 源码于 `third_party/pyAgxArm`。只有在具备对应系统依赖的硬件部署环境中才运行 `scripts/install_pyagxarm_editable.sh`；它会将源码以 editable 方式安装到当前 `python3`。本次软件迁移没有安装供应商 SDK、没有初始化 CAN，也没有执行位置动作。

