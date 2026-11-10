# AIRBOT MMK2 整机仿真移植

日期：2026-11-09；2026-11-10 补齐实际 Web 验收与整机七组控制。实现基于设备接入指南、PLUGIN_DRIVEN_PORTING、FR3_PORTING_TUTORIAL，以及 FR3/CR5 公共场景经验。新增 `src/robots/airbot/mmk2_description` 和 `mmk2_mujoco_sim`，提交与推送结果见 [发布日志](MMK2_PUBLISH_LOG_2026-11-10.md)。

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

## 本机验证

测试脚本：

```bash
MORLA_PYTHON=/path/to/python bash scripts/test_mmk2.sh
```

本次使用 macOS / Python 3.13 / MuJoCo 3.12.0，使用本机现有 Python 入口安装仿真包 wheel，并使用已有官方 ament_index_python 1.4.1 读取真实索引。没有声称 ROS 2 已在 Mac 运行。已验证从 site-packages 加载 provider，并在按 CMake 安装规则暂存的 description share 中解析全部资产后启动 12 个实例、四路相机；这属于安装布局冒烟，不等于 colcon 构建。

- MMK2 21 项：模型/19 维契约、同场景身份、生命周期、名称/单位/模式/有限数/限位拒绝、底盘移动和停止、模式门控与轮速清零、物理伺服、双指联动、传感器维度、左右 FK/IK 与独立 URDF 变换相符、真实安装工厂发现、Web 目录/模板/编译/保存/启动/HDF5 录制、四视角实际 JPEG/float32 深度渲染、双手 InputStream→公共 BindingEngine/IK→Runtime 命令、provider 缺失/冲突拒绝、初始化失败资源释放、轮速绑定/超时停止、完整坐标链及头部镜头位置。
- FR3 70 项专项回归通过。
- 新一轮联合回归涵盖 MMK2、FR3/CR5、公共 runtime/core/contracts/teleop、Web composable/operator/plugin：**355 passed、1 skipped**。跳过项缺少可选 Hypothesis。完整结果见 [Web 阶段回归日志](evidence/2026-11-09-mmk2-web/regression.log)，计数不能与专项结果相加。
- 实际四相机 RGB 与深度图均有限且非空。macOS OpenGL 报告 ARB_clip_control 不可用，远端深度精度受限；这不是传感器标定证据。

完整输出见 [联合测试日志](evidence/2026-11-09-mmk2-local/tests.log)、[wheel 安装日志](evidence/2026-11-09-mmk2-local/wheel-install.log)、[安装布局冒烟](evidence/2026-11-09-mmk2-local/installed-scene.log)。

## 尚未验收的内容

本地实际浏览器已完成配置、检查、启动、七组遥操、录制、Grip 释放、暂停、急停/复位、模式切换及停止，见 [Web 验收报告](MMK2_WEB_ACCEPTANCE_2026-11-10.md)。Ubuntu/ROS 2 Humble 的 colcon 编译和真实 DDS 尚未在目标机执行，本地浏览器验证不等于目标机验收。

底盘是平面约束速度伺服模型，半径 .08401 m 根据 URDF 轮轴高度假定、轮距 .3265 m 来自关节安装位置，未标定真实差速参数，也未模拟轮地接触/滑移。机器人视觉网格不参与碰撞。IMU/相机/激光外参、惯量和伺服参数未做实机辨识，不能据此宣称避碰、接触动力学或 sim2real。

默认采集三路机载 RGB，深度由 read 的 depth 端口返回 32FC1 原始米制数据，尚未接通公共深度数据集导出。激光只扫描世界 group 0 几何，10 Hz、180 点、10 m 上限，不含噪声模型。

该交付是整机仿真接入，不包含真实 MMK2 SDK 通信、使能或实机运动；SDK 与 URDF 差异必须在后续真机移植中逐项核验。
