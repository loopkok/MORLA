# Native ROS actuator plugins

本包承载 Nero、Astral、XHand、Wuji 的既有原生 ROS 桥与启动计划；公共 morla_runtime 不再内置品牌启动分支。

安装后通过 morla.adapters 的 plugin_id:real 工厂被选中加载。工厂从 context.instances 读取本次装配，从 context.shared_device_sessions 按物理 session_id 复用连接。Astral 臂与夹爪共享一个控制板进程。configure 显式建立连接，prepare/activate/submit/stop/close 沿用同一 adapter ABI。

`release()` 是撤销运动许可时的软停止：只停发目标，不调用驱动的 `~/driver/estop`（Nero/Astral 的 estop 会断电机电源，机械臂在重力下垂，下一次 activate 就会在掉下去的姿态上重新使能）。软释放后由驱动自身的命令看门狗（`command_timeout_s`）发一次以实测位姿为种子的保持目标并撤销流式许可，电机保持使能。急停、停止会话、复位与失败补偿仍走 `stop()` → `estop`。

在 ROS 工作区执行 colcon build --symlink-install --packages-up-to morla_bringup 并 source install/setup.bash。也可单独构建本包；真实驱动、SDK 仍需另外安装。Python wheel 的安装成功只验证元数据和包布局，不验证 ROS 或硬件。

旧 morla_runtime.native_ros 仅保留兼容导入；自定义机器人应注册自己的 morla.adapters 工厂，不向本包追加上游输入或品牌组合分支。进程内 SDK 也使用同一 configure/close 生命周期，无须强制包成 ROS 节点。
