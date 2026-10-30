# 公共输入与绑定路径

`vr_epg.fake.yaml` 描述 QuestRight scalar→EPG 米制开口，复用公共 BindingEngine/scalar_map。必须安装新版 morla_input 与 morla_epg_driver 的组件描述后编译；源码直接编译可在隔离 Catalog 注册两份 descriptor。

改成 Vader：将 vr.plugin 改为 morla.input.vader5、参数改成该描述允许的值，保留 scalar binding；改成手机则选 morla.input.mobile 并使用现有配对入口。换成 Astral 夹爪需同时改 target 端口为 closure，并把 output_range 改成 [0,1]。不同单位不能仅替换 plugin。

fake 不产生真实 VR 包；本配置的编译成功不等于真实 VR→EPG 已运行。

## 任意上游的统一 ROS 意图入口

安装新版 morla_input 后选择 morla.input.standard，设置 parameters.input_topic，例如 /cell/operator/intent。消息类型 std_msgs/msg/String，JSON 内容：

```json
{"epoch":"connection-1","sequence":0,"frame_id":"operator","ports":{"scalar":0.5,"axes":{"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0}}}
```

每个连接用新的 epoch，sequence 递增；connected:false 表示断开。ports 按组件声明提供；pose 为 position[3] 与 quaternion[4]（xyzw）。source_time_ns 可选，作为源时间记录，本机接收时间由入口生成。消费方统一读 InputFrame，不读上游品牌报文。

此入口是新标准协议，原 Quest/Vader 专用报文桥暂时保留兼容；尚未迁移全部原始发布者。

## 输入进程托管

QuestRight/Vader 的 parameters.process_mode 可选 auto（默认，已注册工厂则托管）、external（已有进程）或 managed。connect_inputs 使用 ProcessSupervisor 加载 morla.processes 同名 entrypoint，启动所选输入组件的采集进程；close 回收自己创建的进程。Web state.input_processes 显示 PID/退出码；external 不会被回收。

扩展工厂签名 factory(instance)->list[argv | spec]，spec 可包含 argv/key/env/readiness，内部生成固定 argv。注册在组件包 setup.py 的 morla.processes，name 等于 plugin_id。SDK 进程内执行器不必为了接口统一额外启动 ROS 节点。

公共输入宿主、共享进程去重、系统入口和原生插件桥已接通。UR11 仍开放：无 Web ROS stop 服务暂不关闭输入管理器，需退出宿主；Web stop 则主动关闭。ROS/实机与第二 reviewer 结论仍未取得，示例不作为完整部署验收。

完整本轮状态见 [实现记录](../../docs/architecture_refactor/UNIFIED_IMPLEMENTATION.md)，尚未通过的迁移/审查项按记录继续执行。

## 双臂、双夹爪与滤波

旧 Quest 组件现在提供 left_scalar/right_scalar、left_axes/right_axes，以及原有左右 pose/landmarks。双侧绑定使用这些明确端口，避免使用兼容 scalar 别名（它代表最近一次控制器事件）。部分帧仅更新出现的源端口，不要求两只手同时发送。

QuestRight 的 scalar 可由扳机或裸手捏合产生，pinch_range_m=[close,open] 配置裸手米制范围。需要旧自动范围和滤波时选择 morla.pinch_filtered，扳机 shaping 选择 morla.trigger_filtered；旧 Astral Node 与这些控制器共用纯数值实现。

使用旧原生臂/手时安装 morla_native_bridge；公共运行时通过注册工厂加载，不再内置品牌启动计划。
