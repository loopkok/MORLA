# 设备接入规范与独立驱动：2026-10-28 详细提交记录

记录时区：Asia/Shanghai。工作区：MORLA；本轮基线 `b91211a9184067a42d7b8090d3dddca138ac80e4`。提交目标为已配置 origin/main；本轮不修改桌面 cr5_loophand_graspnet、MORLA-piano、morla-xr 或任何实体设备。

## 最终交付范围

### 设备移植规范

新增 [device_integration_guide.md](../device_integration_guide.md)，核对重构后的 ComponentDescriptor / Assembly / Deployment / RuntimeSession / SchemaV3 / 输入与 controller / 数据链实现，明确接口 ABI、插件发现、状态/动作布局、资源与控制权、坐标/关节/单位/TCP/时间、来源/有效掩码以及各类设备的实现与验收工作。包含可编译 fake 样例、real 占位模板、必测反例和每次接入记录模板。

README、docs 索引、简明 SDK 和历史项目导览补充当前入口，纠正“重构尚未实施”的过时表述，历史基线和验收边界继续保留。

### CR5 独立接入

`src/robots/dobot/morla_cr5_driver` 提供独立 ROS Python 包、`morla.dobot.cr5@1.0.0` 描述、ament manifest、fake/real factory、单设备 launch/配置、测试与许可证/来源记录。不依赖手部包或串口 SDK。

TCP 29999/30003/30004 分离控制、ServoJ 与二进制反馈；解析 1440 字节布局，核验长度/魔数/设备时钟，转换 degree/rad；使能/停止等待新鲜 enable_status，提供实测播种步长限制、部署关节范围、有界 I/O 与主机命令 watchdog。设备层 watchdog、实际限位、固件兼容、零位/方向、载荷/工具及实机停止尚未证明。

### LoopHand 独立接入

`src/robots/loophand/morla_loophand_driver` 独立提供 `morla.loophand.compact@1.0.0`、串口协议、上游耦合模型、Adapter、发现/加载/ROS 入口、单设备配置与测试。不依赖 CR5 或其他机械臂包。

7 维紧凑手型映射到 16 维耦合运动学关节和 7 个执行器；真实 GET_POS 编码器经模型反解的反馈为 estimated、measured=false，不冒充 16 个独立关节测量。real 运动保持门禁：CTRL_TOR 零力矩请求没有执行 ACK，设备 watchdog 未验收；不能靠 deployment 标志绕过。stop 未确认明确报告错误，不能静默当作停止完成。

来源及原文件 SHA-256 分别归各自 PROVENANCE.json，保留 Apache-2.0 / Dobot MIT 许可。上游 aero_hand_constants.py 仅规范化行尾空白，已在来源记录中说明；原摘要始终表示来源原文件，不表示修改后的文件摘要。

### 公共 runtime 与配置组合

- `cached_actuator.py` 提供品牌无关缓存 I/O、latest target、反馈来源、目标守卫和主机 watchdog；具体设备只提供 transport、配置校验及能力声明。
- `ros_bridge.py` 保留原 runtime_node 默认行为，可选发布每个执行器的具名 JointState，按真实 sample_ns 转换时间且不重复刷新缓存样本；独立驱动入口限制为对应单设备快照。
- 新 `joint_state_runtime_node` 为通用多组件入口。命令、操作和控制权始终经过原 runtime 网关。
- real 外部插件的预检只读已安装 entry-point 元数据；不导入设备代码，不证明 SDK 依赖和硬件就绪。
- `examples/cr5_loophand` 只有 Assembly/Deployment、说明和组合回归。没有联合 ROS 包、联合驱动或两包相互依赖；可分别搭配其他臂/末端。

初版在同一个未提交包中同时封装 CR5 与手；根据用户指出的包边界问题完成拆分。最终仓库只提交独立两包；旧联合包没有历史迁移包或兼容 shim。plugin_id 保留，但描述来源/插件摘要已改变，旧 resolved 必须重新编译，Web 保存新 revision。

## 公共 IK 与 CR5 VR 的范围说明

本轮阅读并明确了现有公共层与专用算法的关系，没有实现新 IK 算法：

- 公共 CartesianController 处理 VR 增量、建锚、工具偏置、求解调用和结果检查。
- 实际 FK/IK 仍由 Nero / Astral 各自求解器承担；当前 solver_for 只认识这两类机器人，新组合路径选择 Astral geometric。
- 旧 Profile 遥操与新组合输入/controller 不是完全相同的执行链；旧 Nero 验证不能替代新链路验证。
- CR5 还缺运动学后端及加载接线、坐标/TCP 对齐、Quest 绑定和实机验收。本轮不宣称已经实现 CR5 VR 跟随、CR5 URDF/IK、手重定向、触觉抓取、相机或仿真模型。

## 实际验证与证据

使用本机已有 `/opt/anaconda3/bin/python`（Python 3.13.5），未使用 ROS 或连接真实设备。原始结果在 [evidence/2026-10-28](evidence/2026-10-28/manifest.json)。

1. `MORLA_PYTHON=/opt/anaconda3/bin/python bash scripts/test_composable.sh --layer pure -rs`：112 passed、1 skipped。跳过来自缺少 Hypothesis 的属性测试，不记为通过。
2. 分别只将一个驱动目录加入 PYTHONPATH：CR5 4 passed；LoopHand 3 passed。对应测试已纳入 pure，不能再相加冒充额外唯一用例。
3. 两包分别构建 wheel：在各自临时安装目录确认另一包不可 import，通过实际打包资源发现 descriptor、加载选中 factory、校验 ABI 和 fake 生命周期。单臂 6/6、单手 7/7、组合 13/13 状态/动作维度均符合契约；停止后锁释放。
4. 安装验证只替换 ROS-free 主机上的 ament 索引查询层，实际读取 wheel manifest 和 entry-point；未伪造实体反馈。过程脚本与结果一并归档，不等于 Linux colcon/ROS 验收。
5. Python 语法、package.xml、资源清单、跨设备依赖、旧联合包移除和 52 个本地文档链接检查通过；Git 暂存区空白检查随最终提交核验。

本轮没有改动前端、训练算法或仿真资产，未重复宣称上轮 Web/训练/仿真测试为本轮结果。尚未执行 Linux ROS Humble colcon/IDL/消息服务联调、真实设备/输入、实体停止/watchdog、安装标定、8 小时长稳、真实录制导出和训练/模型推理。

## 提交、推送与回退策略

先提交包含最终源码、配置、文档、CHANGELOG 和证据的主内容；正常推送当前 main，禁止 force 或重写既有提交。若远端出现新提交，先核对并协调，不覆盖远端历史。

主内容实际推送并核对远端 SHA 后，再以独立文档提交记录真实提交 ID、推送返回及远端结果。回执自身 SHA 由 Git 历史记录，避免文档自引用。没有预先声明推送成功。

回退先停止会话并确认物理设备状态，再以 git revert 撤销相应提交；保留原数据、标定、设备身份和验收证据，不删除活动锁绕过未确认停止。
