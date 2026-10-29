# MORLA 可组合组件接入 SDK

完整设备移植流程、各类设备的对齐要求、当前接线限制及验收模板见 [设备移植与接入指南](../device_integration_guide.md)。本文保留简明 ABI 参考。

本接口以实际测试代码为准：纯配置入口位于 `morla_core.composition`，适配 ABI 位于 `morla_core.contracts.component`。`schema.py` 的旧 v2 身份保持不变；新系统使用 `SchemaV3`。

## 一、组件包最小内容

1. 提供静态 descriptor，声明版本、kind、端口方向/语义/维度/单位、具名关节/限位、配置 schema、后端、物理会话共享方式与停止行为。参考 `examples/composable_plugin/descriptor.json` 的三自由度组件。该样例限位与反馈来自 fake，仅用于协议集成。
2. 提供选中后才加载的 adapter factory；参考 `examples/composable_plugin/adapter.py`。不要在模块 import、目录发现或构造函数里 enable/home。
3. 为真实组件提供模型、显式 canonical→driver/MJCF 名称映射、安装/TCP 标定与部署验收证据。已有 Nero/XHand/Wuji 的描述保留原 YAML 限位来源；Astral 的旧声明限位仍标注未做现场确认。
4. 各能力实现自己需要的方法。sensor 不必实现 submit/home；输入设备采样与 robot actuator 相互独立。引入新物理能力需要版本化契约扩展。

## 二、静态发现与选中后加载

目录发现支持 ament resource 类型 `morla_components`。resource 内容每行是 prefix 内的 manifest 相对路径；绝对路径、路径穿越和逃逸 symlink 拒绝。

纯 Python 分发元数据使用 `morla.component_manifests` group；entry value 是**分发目录内的 JSON/YAML 相对文件路径**，不是 Python import reference。例如最终 wheel 的 `entry_points.txt`：

```ini
[morla.component_manifests]
reference_arm = morla_reference/descriptor.json

[morla.adapters]
lab.reference.arm:fake = morla_reference.adapter:create_adapter
```

前一 group 由 MORLA 作为静态数据元数据读取，不调用 EntryPoint.load；打包器应将相对资源路径原样写入已安装的分发元数据。后一 group 使用标准 Python entry-point factory，仅当精确 `plugin_id:backend` 被选中且 runtime 已占有资源锁时 load。安装时验证包来源，AssemblySpec 不接受 Python 路径或 shell。相同 ID/version 多重注册报歧义；不会按安装顺序任选其一。

`default_catalog()` 自动发现安装的静态 manifest。`Catalog.register()` 可用于隔离测试；Web 与 runtime 必须使用同一 trusted catalog。盲接回归在 `integration_repros.py::test_ir07_installed_blind_arm_web_gateway_recording_canary` 中创建独立分发元数据，实际完成自动发现、编译、启动、网关与落盘。

## 三、实际 adapter ABI

factory：`factory(instance: dict, context: AdapterFactoryContext) -> adapter`。

factory context 的字段如下：

- `session_id`：本次会话唯一身份。
- `clock`：返回 host monotonic 纳秒的函数；deadline/TTL 使用它。
- `resource_root`、`lane_fd`：runtime 已持有的主机资源锁上下文。
- `shared_device_sessions`：当前装配内共享物理会话对象缓存；由同一 trusted 组件族协调使用。
- `device_session`：编译快照中该实例对应的 DeviceSession 描述；包含 session_id、device_id、resources、部署参数及逻辑实例列表。

runtime 随后调用 `configure({'session_id': ..., 'instance': instance_snapshot})`，不是把 factory context 再原样传入。factory 可保留已分配的共享物理会话引用。

所有托管 adapter：

```python
configure(context: dict) -> None
health(now_ns: int) -> dict
close() -> None
```

actuator 另外实现：

```python
prepare() -> bool | dict
activate() -> bool | dict
submit(target: dict) -> dict
stop() -> bool | dict
```

公共网关校验完整 CommandEnvelope 后，`submit` 收到单个 target，键为 `component_id / names / values / mode / units`。`activate` 没有 permit 参数；授权只由公共 runtime 生成并校验。不要在 adapter 内另造一个接收任意公开 ROS target 的写入口。

读取能力提供 `read(now_ns: int) -> dict`；可选操作能力提供 `operate(operation: str) -> bool | dict`，只允许 descriptor 声明的动作。操作幂等、取消状态、超时由 runtime 管理；不可中断 SDK 必须有独立停止/设备 watchdog 证据。

`health` 至少返回 `ready / sample_ns / source / measured / synthetic`；actuator 返回具名顺序对应的 `values`。read 返回 `ports[port_id]`，样本含 `values` 或 JPEG `data`、源时间/有效性/来源。`fake` 不能标 measured；缓存 command echo 不能标实际反馈。传感器与图像原 payload 通过专用流落盘。

## 四、符合性测试

`morla_core.plugins.conformance.validate_adapter_contract(adapter,instance)` 检查必需方法和可调用签名，不调用任何方法、不打开设备。对于缺少 Python signature 的扩展方法，报告 `uninspectable_methods`，不能宣称已验证行为。

`exercise_fake_adapter(adapter,instance)` 执行 configure、health、prepare/activate、单次 submit、stop、close，仅接受明确 fake adapter/instance。real/sim 的行为测试在独立部署环境执行，不能用此 harness 自动触发设备。

独立运行：`python -m morla_core.plugins.conformance assembly.yaml --instance custom_arm --exercise-fake`。CLI 拒绝任何包含非 fake 后端的装配，使用 runtime 申请资源锁后只加载该装配选中的已安装 adapter；不从 Assembly 解析代码路径。

必测负例：重复命令/旧 session/旧 epoch/过期 TTL、缺关节/错单位/非有限值、资源重复占用、断连停止、非预期作用范围、SDK 阻塞、部分写失败与旧 writer 旁路。新真实部署必须有 feedback、stop/watchdog、安装/工具/载荷和选中关节作用范围记录。

## 五、控制器与重定向

安装的控制器通过 `morla.controllers` entrypoint 注册；name 是 descriptor 的 controller plugin ID。factory `factory(context: dict) -> callable`，callable 签名 `(value, measured, dt, parameters) -> ordered targets`，有内部滤波时提供 `reset()`。`install_controller_plugin` 仅加载所选 name；静态 descriptor 的 controller_types 定义源/目标能力。

`install_retargeting(registry,instance)` 注册已有 XHand/Wuji 映射：XHand 复用原 Quest→MANO preprocess、DexPilot YAML、具名关节重排和左右 index-bend 规则；Wuji 复用 official 或 DexPilot 后端。输入必须是完整 21×3（或明确 63 元素）有限骨架，输出完整 12/20 个具名关节，不通过复制标量假造手型。可选依赖或安装模型缺失时 preflight 明确 unavailable；stub 数学回归不等同真实手 SDK 已验收。

## 六、编译与数据身份

`compile_system(assembly,deployment=None,registry=None)` 纯逻辑返回 detached JSON snapshot，不启动设备。`verify_resolved(snapshot,registry=None)` 校验快照 hash 与 trusted catalog 重新编译结果。运行/采集使用同一快照；旧数据通过显式 `to_v3` 转换保留来源版本/hash。

Schema v3 独立保存 state/action 布局、任意相机数据键、raw auxiliary 与 sensor_specs。修改输入而模型特征不变时，feature_contract_hash 保持；几何/标定内容变化更新 geometry_hash；输入映射另有 input_mapping_hash。相机模型槽位映射、归一化、feature 名称/单位/语义都必须通过学习握手。

## 七、异步笛卡尔 controller 扩展示例

CR5 的 `morla.cr5.cartesian_delta` 通过同一 morla.controllers 机制注册，静态 controller descriptor 随编译锁文件冻结。组合宿主在显式激活前加载所选 controller，factory context 除 instance/binding 外可包含 `clock` 和 Adapter 可选的只读 `kinematics_provider`；未提供该能力的 Adapter 不受影响。CR5 当前改用独立 URDF / Pinocchio 本地求解器，不使用该可选 provider，也不回退控制器内置 IK。

controller 的 `reset()` 必须隔离迟到的异步结果；持有线程的 controller 实现 `close()`，宿主在会话退出时回收。绑定的 max_age_ms 已传给 BindingController。参考 [CR5 VR 操作与实现](../integrations/cr5_vr.md)，该例不能替代其他设备自己的运动学和实机验收。

可选 controller 标记 `requires_fresh_input=True` 时，宿主在激活前及 ACTIVE 的人工控制/等待阶段调用 BindingController.input_fresh，检查源、位姿和 bool deadman 的独立时效；缺输入不会先使能再等命令。原 controller 默认不启用该附加要求，CR5 真实 VR 后端显式启用。


### 可插拔ROS输入源

输入设备可注册 `morla.ros_inputs` entry point（名称为plugin_id）。RosInputManager只加载当前选中的factory，传入instance、clock与on_transition；返回bridge包含source（InputStream）、topic、consume(JSON)、actions及control_event(frame)。订阅器将所有有效帧交给input_event观察/操作，再按control_event派发运动；未知旧会话、断连或坏包撤销旧输入权。Vader实现位于morla_vader5_input，不在公共宿主硬编码品牌。

动作名复用teleop_start/teleop_pause/record_start/record_stop，另有estop；组合Service调用现有生命周期/数采API并以输入epoch/sequence去重，不能再同时转发旧Profile操作话题。动作回执不等于硬件完成；新设备必须给出按钮冲突与启动已按住处理策略。

InputFrame.synthetic用于明确fake源，InputIngressAdapter保留标量与向量端口的来源时间；录制按钮边沿另存事件。scalar_map可连接一维joint_targets并可用max_velocity（目标单位/秒）限速，多关节不允许标量误连。已有不传max_velocity的映射行为保持。
