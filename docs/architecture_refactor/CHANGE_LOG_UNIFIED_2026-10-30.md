# 统一组件与绑定重构更新日志

基线 c1896c6；本文件记录本轮主内容提交；实际 SHA 与推送结果见 PUSH_LOG_UNIFIED_2026-10-30.md。原有2026-10-28重构、2026-10-29设备接入及2026-10-30旧主机/EPG台架记录保留各自身份，不改写历史结论。

## 实现变化

- 公共 BindingEngine/ControlHost 与操作事件路由供 Web/CLI/ROS 复用；新增标准意图桥、左右 scalar/axes、具名轴、部分帧和每端口去重。
- 通过 morla.processes 托管选择的输入进程，支持 auto/managed/external、共享配置去重、退出状态、失败回收；兼容 Vader/CR5 launch 移除重复输入节点。
- 原生 Nero/Astral/XHand/Wuji 桥迁入 morla_native_bridge，通过 adapter 工厂共享物理会话；Nero/Astral/CR5 运动学由注册提供者选择。
- 旧 pinch 自动范围/EMA、扳机曲线和输出 slew 提取到通用处理器，旧 Node 与新控制器复用；新夹爪通过单位/范围映射接入。
- Web 根据端口语义、自由度、单位与已安装控制器能力匹配；采集继续使用同一冻结契约。未恢复租约/deadman/部署验证 flag 门禁。

## 本轮文档同步

更新根 README/CHANGELOG、公共 runtime/input/bringup/Web/data/policy README、Vader与相关驱动 README、Astral旧夹爪入口、相机预览说明及组合示例；补充新插件依赖、标准入口、legacy 边界与测试链接。新主机手册 U01–U08、报告模板、单执行器 fake 冒烟脚本说明及设备接入指南同步。

供应商 SDK、第三方发布日志和历史推送回执保留原始事实；不把上层框架变化写成厂商协议或固件版本更新。

## 验证与开放项

最近一次代码验证：pure202 passed/4 skipped、fake23、sim7、TypeScript/Vite与Edge通过；插件 wheel 实际安装后用 mock transport 验证共享会话。证据见 evidence/unified-20261030/。

文档更新只检查代码块语法、链接、CLI帮助与脚本fake-only读取路径；没有因为文字更新重跑硬件或虚报测试数量。新版 ROS/实机、真实模型和第二 reviewer 未执行。

UR11 尚未关闭：Web stop 主动关闭输入管理器；无 Web ROS Runtime stop 服务只停止执行器，输入在宿主退出时关闭。主机手册要求分别记录 stop 后与退出后 PID，不能据 STOPPED 推断完整输入回收。后续代码修复及 ROS 复验需单独留证。


## 交付文件与接口

1. morla_teleop：engine.py、session.py、actions.py 为共享执行/输入动作路径；gripper_processing.py 保存通用数值处理，旧 Astral Node 调用同一实现。controllers/kinematics 扩展 identity、滤波与运动学注册。
2. morla_input：标准描述/桥、QuestRight scalar/骨架、左右独立 scalar/axes、per-port 时戳和采集工厂；旧 ROS 设备分支隔离到 builtin_sources。
3. morla_runtime：supervisor、进程状态/清理、ROS/CLI connect_inputs、adapter context.instances；native_ros 只保留兼容导入，原生实现移入 src/robots/native/morla_native_bridge。
4. 设备包：原生适配工厂及共享 DeviceSession、Nero/Astral/CR5 运动学注册；Vader 与 CR5 VR 的兼容 launch 收敛到公共 session。
5. Web：共享绑定/事件路由、控制器能力/维度/单位匹配、axis_names、进程模式与状态；前端 dist 与源码重新构建一致。
6. 测试：跨输入/夹爪/双臂矩阵、安装元数据扩展考试、共享进程/错误清理/维度/缓存/部分帧反例、冻结旧 pinch trace、静态层边界；fake ROS脚本维持单执行器范围。
7. 文档：根与相关包 README、主 CHANGELOG、详细实施/审查/SDK、设备指南、操作说明、主机手册 U01–U08 与报告模板、组合示例及实际证据。

接口变化：新增 morla.processes 与 morla.kinematics 注册；输入标准端口与可选 axis_names；adapter 工厂 context 新增 instances。原 morla.adapters/ROS commands 基本签名保留。旧原生设备需安装 morla_native_bridge；不要只升级 morla_runtime 后沿用缺失插件的安装树。

## 提交前实际命令与环境

本机 macOS；纯逻辑/场景使用已有 conda Python 3.13，fake 联测使用独立 Python3.12及 /tmp/morla-composable-deps。ROS 构建不使用该 conda 环境；目标主机仍须系统 Python/Humble 单独验收。

- MORLA_PYTHON=/opt/anaconda3/bin/python bash scripts/test_composable.sh --layer pure -rs：202 passed，4 skipped，退出0；具体skip原因见提交日志。
- Python3.12 + /tmp/morla-composable-deps 执行 --layer fake：23 passed、1条Starlette/AnyIO弃用警告，退出0。
- conda 执行 --layer sim：7 passed，退出0。
- Node 执行 tsc -b 与 vite build：退出0；61 modules，index-rnwb7kYG.js，282.36kB/gzip96.66kB。
- python reviews/unified_review.py：边界检查通过；fake_ros_gateway_smoke.py --help：通过，未执行DDS。
- 之前同轮实际 Edge、隔离 wheel/factory 及文档检查保留各自证据；本次提交前未重新执行真实浏览器/硬件。
- git diff --check、暂存检查及最终包/源文件语法检查以提交回执记录为准。

当前证据目录 evidence/unified-20261030/。测试有重叠，禁止将不同层/历史通过数相加充当唯一用例数。没有复制厂商二进制wheel、现场标定、SDK凭据、视频、缓存或未追踪设备运行数据。

## 提交、推送与回退

先提交完整主内容，再正常快进推送 main；成功并重新获取远端核对后，单独提交推送回执，避免自引用 SHA。不会强制推送。回退应先停止设备会话并退出输入宿主、确认资源释放，再 git revert 主内容提交；不删活动锁、不覆盖原始数据/标定或将 v3 原地降级。

提交前 skip 明细：conda 中缺 Hypothesis（parser properties 1项）与 FastAPI（CR5 VR 1项），缺 jodellSdk（EPG SDK 2项）。独立 Python3.12 fake API 层23项已通过，但不据此把这些 skip 改成 PASS。
