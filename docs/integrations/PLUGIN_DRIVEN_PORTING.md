# 插件声明驱动的机器人接入

2026-11-06。本文替代早期 FR3 教程中“在 Web 前后端增加品牌分支”的步骤；硬件 SDK、生命周期、标定和实机验收要求不变。

## 本次修复的框架问题

- Web 模板 ID、标题和文件路径分别写死在前后端：改为已发现 descriptor 的 `web.templates`，由 `GET /api/v2/templates` 列表驱动页面，原 `GET /api/v2/templates/{id}` 保留。未安装的可选模板资源不展示，重复 ID 明确报错，不静默覆盖。
- 控制器在编译器和 Web 各维护一份能力表，机器人限制只有前端执行：公共 controller 也进入 catalog；两端消费同一 `controller_types`。编译器检查目标插件、目标能力、维数、单位及 backend，controller descriptor 摘要进入 lockfile。
- 显示名、设备字段、品牌默认值、Nero/Astral 左右臂绑定参数散落于 TypeScript：迁至 descriptor 的 `web`、`defaults` 和 `configuration_schema`。CR5、FR3、Nero、Astral、Wuji、XHand、LoopHand、EPG 及现有输入/相机都迁移至同一读取路径。
- 共享仿真按品牌选择 MJCF 路径和模型子树：改为 `geometry.simulation`。CR5、FR3、Nero、Astral、XHand、Wuji 使用同一个通用组合器；FR3 原生映射与部署限位检查保留。独立仿真包和公共入口仍复用同一仿真循环。
- 相机预览接口与共享仿真只识别 `morla.camera*` 名称：改为 sensor 的 `rgb` image 端口能力，Web 预览也读取部署覆盖后的 backend。

这不意味着发现描述符即完成硬件或模型验证。声明能力后仍须安装实际 Adapter/controller/kinematics 工厂，并完成独立模型、停止与通信测试。原生 ROS 桥的设备进程配置仍由其插件负责；本次未重写 SDK 或扩大实机验收结论。

## 新设备包怎么声明

设备和控制器继续通过 `morla_components` ament 索引或 `morla.component_manifests` 静态资源注册。发现不导入厂商 SDK。已有运行工厂接口不变：`morla.adapters`、`morla.controllers`、`morla.kinematics`。纯 Python 安装必须同时安装静态 manifest 与相关资源，不能只有工厂 entry point。

设备 descriptor 示例片段（合并到完整 descriptor，不能单独当作完整文件）：

```json
{
  "web": {
    "label": "示例机械臂",
    "fields": ["rate_hz", "host", "initial_positions"],
    "connection_fields": ["host"],
    "field_backends": {"initial_positions": ["fake", "sim"]},
    "preferred_controllers": {"pose6d": "example.arm.follow"},
    "templates": [{"id": "example-arm-sim", "title": "示例机械臂仿真",
      "package": "example_arm_driver", "path": "config/quest.sim.yaml"}]
  }
}
```

- `fields` 可省略：展示配置 schema 中的字段。字段标题、类型、枚举、范围放在 `configuration_schema.properties`，无需增加前端中文词典。
- `connection_fields` 仅在 real 的连接区显示；其他字段在设备设置区显示。`field_backends` 限定字段后端；`field_conditions` 例如 `{"tcp_port":{"protocol":["tcp_wired","tcp_wireless"]}}` 表示所有条件满足才显示。
- 模板只能引用已声明包内的相对 YAML/JSON，禁止绝对路径和 `..`；安装资源缺失不会从源码副本悄悄补齐。开发 checkout 只有包未安装时才按 package.xml 查找同名源码包。安装包须在 setup.py/CMake 中安装模板；模板拥有者须依赖其引用包。
- 模板中的安装位姿、输入基变换、TCP、deadman、速度参数必须按设备填写。不要把“自动发现模板”理解成“自动产生现场标定”。
- `binding_presets` 可声明按输入插件和目标参数选择的默认绑定，例如 `{ "controller":"morla.cartesian_delta_ik", "source_plugin":"morla.input.quest", "parameters":{"side":"left"}, "defaults":{...} }`。左右臂示例见内置 `web_metadata.json`；新设备在自己的 descriptor 中声明。
- 已保存配置仍保留自身参数，不因插件默认值变动而覆盖用户标定。设备预设选择器按模板 `assembly_id` 恢复；自定义名称显示未选择预设。

控制器 descriptor 片段：

```json
{
  "controller_types": {
    "inputs": ["pose6d"], "outputs": ["joint_targets"],
    "target_plugins": ["example.arm"]
  },
  "defaults": {"position_scale": 0.25, "max_joint_velocity": 0.2},
  "web": {"label": "示例末端跟随", "control_family": "cartesian_pose",
    "ik_modes": ["differential", "strict"]}
}
```

`target_plugins` 用于专属求解器；通用求解器可使用 `target_capability`。当前公共 Cartesian 控制器要求目标 `geometry.controller_capabilities` 包含 `cartesian_ik`，且运行时存在对应 kinematics 工厂；新机器人不需要加入公共品牌白名单。`target_dim` 限制输出维数；`equal_dimensions_units` 要求输入/输出维数及单位一致。`backend_variants` 只声明已实现后端。

`control_family` 可选 `cartesian_pose`、`cartesian_jog`、`joint_jog`，启用对应坐标/TCP/摇杆表单；不声明时使用通用参数表单。实际参数默认值使用 `defaults`，参数说明/类型使用 `configuration_schema`。Web 建议不是授权，后端编译与 runtime 检查必须保留。

## 新仿真模型怎么声明

资产保存在机器人自己的 description 包。设备 descriptor 的 `geometry.simulation`：

```json
{
  "package": "example_arm_description",
  "path": "mjcf/arm.xml",
  "root_body": "base_link",
  "flange_body": "flange",
  "native_mapping_only": true,
  "deployment_limits": true
}
```

`root_body` 是要抽取的固定根子树；省略或 null 使用 worldbody 的首个 body。`flange_body` 必须位于该子树；每个 canonical joint 的 `mjcf_name` 必须对应唯一模型关节。`path/root_body/flange_body` 可含 `{side}`，side 只允许 left/right。文件包界限由资源解析器检查，Assembly 不能覆盖 descriptor 的模型路径。

`native_mapping_only` 拒绝非 +1 方向/非零偏置的映射。`deployment_limits` 校验 `joint_lower/upper` 不能放宽 descriptor 关节范围，并与 MJCF 原生范围取交集，同时收紧 joint range 与 actuator ctrlrange。复杂的非线性传动、非原生映射不在这个通用模型格式中；不要伪装为已支持。

Nero/XHand 的旧组合模型需要抽取/排除子树，使用 `exclude_body_contains`，或 `extract_children` 加 `body_contains/geom_mesh_contains/wrapper_name`；新模型优先直接提供独立完整根子树。安装包必须包含 meshes、MJCF、元数据，并完成多实例命名空间、FK 和限位测试。

## 现有配置升级

本次 descriptor 摘要和公共 controller lockfile 内容改变。停止旧运行、重新构建/source、重启 Web，打开原 Assembly/Deployment 后检查并保存**新 revision**。不要修改旧 snapshot 的 hash，不覆盖历史数据元信息。旧接口模板 ID `cr5-sim`、`fr3-fake`、`fr3-sim`、`nero-sim`、`astral-sim` 保留。

内置历史设备目前仍由 morla_core 的静态目录生成，展示元数据位于 `plugins/data/web_metadata.json`；这是对现存内置 descriptor 的兼容整理，不要求新厂商往该文件加记录。CR5/FR3/LoopHand/EPG 等独立包在自己的 descriptor 持有元数据。

## 验收

必须检查“安装后”结果，而不只在源码路径运行：

1. `/api/v2/components` 有设备和控制器；`/api/v2/templates` 有正确模板，加载不导入 SDK。
2. 浏览器可以发现预设、切换后端、编辑专属参数、选择兼容控制器，保存后重新打开不会丢参数。
3. 直接 HTTP/CLI 提交错误机器人/controller 组合仍被拒绝，不能绕过 UI 限制。
4. 仿真单/多实例模型能构建，FK 与机器人 URDF 一致，限位及原有停止门控保留。
5. fake/sim/ROS/真实硬件分别记录结果；本次不做真机测试。

新增回归 `test_plugin_discovery.py` 使用未出现在品牌表里的 `example.new_arm`，验证目录、动态模板、编译、共享模型构建、错误绑定拒绝、重复模板和路径越界。前端另测 `example.arm/example.follow` 的显示、参数、兼容性和默认控制器。历史机器人使用原回归；本轮执行记录见 [验证日志](PLUGIN_PORTING_VALIDATION_2026-11-06.md)。

## 整机耦合仿真扩展（2026-11-09）

对于移动底盘、混合位置/速度控制、mimic/传动及非图像传感器，固定子树 builder 并不足够。
设备 descriptor 可声明 `geometry.simulation.provider`，例如 `morla.airbot.mmk2`；
设备包在 `morla.simulation_scenes` 注册同名可信工厂 `factory(runtime) -> {instance_id: adapter}`。
公共 runtime 要求整个场景只能选择一个 provider，并要求工厂覆盖全部 sim 实例；未安装、重复入口、provider 冲突、缺少实例均拒绝。
该字段来自静态 descriptor，不接受 Assembly 参数指定 Python 导入路径。工厂自行核对混合后端、组件范围与模型映射，持有一个共享场景，仍通过 RuntimeSession 网关接受命令。
普通固定机器人未声明 provider 时继续走原公共 builder，不改变 FR3/CR5 的组织方式。

MMK2 的完整实例见 [MMK2 仿真移植](MMK2_SIMULATION.md)。速度动作在公共 SchemaV3 中标为 velocity，不能当作 absolute_position，也不将轮角索引冒充速度目标的 next-state 映射。当前公共策略执行器尚不支持速度动作，整机默认模板不能直接用于现有位置策略/回放。
