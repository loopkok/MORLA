# 新机械臂移植工作单（复制后填写）

配套教程：[从 FR3 实例学习移植](FR3_PORTING_TUTORIAL.md)。这是记录模板，不是通过声明；勾选时必须同时给出证据。给不适用项写原因，未运行项写 NOT_RUN，缺环境写 BLOCKED_ENV。

## 1. 身份与范围

- 设备厂商 / 精确型号 / 子型号：
- 固件 / SDK 版本及实际读取方式：
- 目标 OS / CPU / ROS / Python：
- MORLA Git SHA / 工作区补丁 / 安装版本：
- 包名 / plugin_id / plugin_version：
- 逻辑 instance_id / 实体 device_id / 唯一 resource_key：
- 多机器人范围：多机型支持 / 单装配多实例 / 独立会话并发；哪些已测试：
- 本次范围：关节驱动 / FK / IK / 输入绑定 / Web / 数据 / fake / sim / 实机：
- 未实现能力与未验收能力（分别填写）：

## 2. 资料与通信审计

- SDK 来源 / 许可证 / 是否允许再分发 / 文件摘要：
- SDK 是独立安装库，还是固定 vendor 源文件；选择原因：
- 端口 / 总线 / 地址 / 权限：
- 构造、read、write、enable、stop、close 各自是否阻塞：
- 返回码及异常语义、后台线程、全局状态和线程安全：
- 自动重连 / 旧命令缓存 / 重连后重放行为：
- 新反馈依据（设备时间 / 帧序号 / 接收回调等）：
- 停止、保持、失能、制动、急停的实际区别：
- watchdog 所在层及证据；未知时明确写未知：
- 只读诊断命令是否有连接之外的副作用：

## 3. 关节、坐标和时间契约

- canonical 完整名称、顺序、单位、模式：
- canonical↔SDK 双向公式（符号、比例、偏移、轴序）：
- canonical↔模型双向映射：
- 限位 / 速度 / 加速度来源与部署可收紧范围：
- base / flange / solver_tip / mount / TCP 的定义及变换方向：
- 输入坐标约定、基变换、操作者站位及标定摘要：
- sample_ns 的实际生成点、时钟域及有效期：
- 重复帧 / 回绕 / 断流 / 非法反馈的处理：
- hardware / fake / simulation / command_echo 的来源标签：

## 4. 包、注册与安装

- [ ] package.xml 通过解析，colcon 识别为 ros.ament_python 或相应 ROS 包类型。
- [ ] setup.py / setup.cfg 安装 console scripts、描述、配置、launch 和许可资源。
- [ ] ament 包标记、morla_components 清单及清单引用路径一致。
- [ ] descriptor 的 ports/joints/backends/schema/defaults 与实现一致。
- [ ] 已声明的 fake/real adapter entry points 能加载，且没有同 ID/version 重复注册。
- [ ] 若支持公共笛卡尔控制，kinematics entry point 能加载。
- [ ] 发现和 compile 不导入 SDK、不探测实体设备。
- [ ] 确认实际 Adapter 类，没有把通用 fake 回退误认为自定义插件成功。
- [ ] 普通安装后脱离源码 cwd 仍能发现模块和资源。

证据命令 / 日志 / 结果：

## 5. Adapter 与控制行为

分别填写实际行为，不只填写是否存在同名方法：

- configure：
- health/read 及 ports 有效性：
- prepare（是否有副作用）：
- activate（如何确认实体已就绪）：
- submit（检查内容、接纳/发送/完成的区别）：
- I/O 调度（更新频率、队列上限、限速、目标领先）：
- release（软暂停、保持依据）：
- stop（操作序列、确认依据、失败报告）：
- close（从未激活、已激活、软暂停、SDK 故障四种情况）：
- 重连后如何防止旧命令重新执行：
- 同一板卡未选关节/末端是否受影响：

必测反例及结果：错轴序 / 错单位 / 非有限数 / 超限 / 缺反馈 / 缓存陈旧 / SDK 卡死 / 部分激活失败 / 软暂停后故障 / stop 未确认 / 资源争用。

## 6. 按需增加能力

### 运动学与遥操

- 模型来源、摘要、末端定义；DH/URDF 不等于算法名称：
- FK / Jacobian 核验方法：
- IK 方法、误差定义、阻尼、种子、搜索范围、限位、迭代/耗时和失败策略：
- 使用公共 controller，还是独立 controller 插件；选择原因：
- 输入来源、freshness、deadman、epoch 切换和重锚行为：
- 实际验证的是数学自一致 / 合成输入 / 实体输入 / 实体机械臂中的哪些：

### Web

- 显示名 / controller 兼容条件 / 参数表单 / 模板 / 默认绑定的实际登记位置：
- 是否新增品牌分支；能否改由已有元数据/插件表示：
- 配置、启动、无输入拒绝、激活、暂停、停止、诊断、重启测试结果：
- 前端构建与部署版本；实际页面/截图证据：

### 数据

- state/action 特征名称、维度、单位、label_source：
- 相机/传感器/输入辅助通道：
- 冻结 resolved/capture schema 与哈希：
- 原始文件来源标签、有效性、采样时间、完成状态检查：
- 对齐 / 导出 / loader / 回放 / 模型各自结果；不要把共用接口写成实测通过：

### 仿真

- 若不支持，descriptor 是否明确 sim unavailable：
- 若支持：URDF/MJCF、惯量/碰撞、驱动映射、场景接线和显示资源：
- kinematic / physics / fake 分别验证了什么：

## 7. 分层验收记录

每条写 `PASS / FAIL / NOT_RUN / BLOCKED_ENV / BLOCKED_CAPABILITY / NOT_APPLICABLE`，附代码身份、命令、日志路径和原因：

- 静态发现与 Assembly compile：
- 实际 factory 与 fake Runtime：
- 契约 / SDK 边界 / 生命周期故障注入：
- FK / IK 数学验证：
- ROS 构建 / IDL / DDS / QoS / JointState / 服务：
- 设备锁 / 同时启动 / 退出清理 / 重启：
- 同品牌多实例 SDK 隔离 / 跨品牌组合 / 控制组故障范围 / 多会话限制：
- 目标机 SDK 导入及只读负例：
- 合成输入→公共控制→fake 执行器：
- Web 实际按钮与目视交互：
- 原始录制文件及数据来源：
- 普通安装 / 脱离源码路径：
- 物理仿真：
- 实体输入 / 实体执行器 / 物理停止 / 断网 / TCP / 长稳：
- skipped 用例和原因：

## 8. 交付

- 维护人 / 评审人：
- 安装、更新、启动、停止、卸载/回退说明：
- 正式示例与现场私有 Deployment 的存放方式：
- CHANGELOG、教程/操作手册与证据索引：
- 提交 SHA / 发布方式 / 远端核验结果：
- 剩余问题、限制及下一次验收负责人：

评审最后检查：一位未参与开发的成员，是否能仅凭文档在没有实体机械臂的机器上完成发现、fake 编译/运行和错误测试，并清楚说明哪些能力尚不能声称通过。
