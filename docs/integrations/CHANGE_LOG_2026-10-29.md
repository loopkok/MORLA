# CR5 本地 IK / VR 遥操详细提交日志（2026-10-29）

记录时区 Asia/Shanghai；仓库 loopkok/MORLA，分支 main。提交前基线 `c7dbec01d48641cf7c47029116edf130b6e9281b`。用户授权同步最新更新、编写详细提交推送日志并提交推送。

本文件记录本次待提交的最终内容及已经执行的验证，不把提交前的计划写成已推送。实际提交SHA、推送输出与远端核对结果随后单独归档，回执提交自身由Git历史标识，避免自引用SHA。

## 最终功能与行为

1. **纯 CR5 描述资源**：迁入统一robot_description中的 `urdf/cr5`、`meshes/cr5`、补充模型清单和MIT来源/哈希。固定基座、六转动关节、7 link、7视觉模型、12碰撞模型、26个网格/材质文件。排除夹爪、场景、ros2_control、OCS2和旧launch/config。原主资源清单与12份旧Profile逐字保留；公共查询/资产检查支持补充清单。
2. **独立本地数值IK**：CR5 `urdf_ik.IKSolver` 使用Pinocchio SE(3)误差、阻尼最小二乘和线搜索；实测播种、正负号/零偏映射、URDF/部署限位交集、局部求解域、时间/迭代边界、FK硬回验。fake/real VR均选此求解器，无控制器IK回退；原控制器/合成求解辅助仅保留早期协议回归。模型身份冻结到geometry契约并在加载时检查。
3. **右手柄与裸手整链**：可选Quest右侧来源事件、静态输入descriptor、序号/epoch/源时效保护、右握把或原始21点捏合deadman；位姿驱动运动、释放立即撤权，来源/会话/frame切换清锚。异步单槽worker隔离旧generation结果、限速与过期拒绝，目标经runtime到独立CR5 ServoJ驱动。
4. **部署与操作入口**：fake Assembly、保留占位/false的real Deployment模板、Quest接收器+会话launch、本地控制页、录制入口、只读本地运动学探针及可选controller FK诊断。启动disarmed，真实激活检查新鲜输入与标定声明，不自动使能/恢复旧跟随。
5. **公共宿主与生命周期**：选中controller在激活前加载、descriptor纳入编译锁、binding时效生效、输入/控制异常撤权、异步controller关闭回收。命令watchdog从首次授权目标开始，真实输入在等待首次目标期间也检查；关闭报错不会跳过停止。
6. **文档与依赖**：更新入口、SDK、设备接入指南、CR5文档、CHANGELOG；归档当前频率/调参解释。增加Pinocchio/robot_description依赖及测试依赖；旧控制器IK阶段明确标为已替代，历史验证记录保留。

未修改Nero/Astral的专用IK算法、频率/速度配置或旧模型；CR5与LoopHand仍分别接入，没有联合驱动。没有连接实机、部署到机器人服务器或运行运动命令。

## 关键参数与现状

- 本地IK：100次迭代、50 ms预算、种子附近±0.3 rad局部域、单次迭代0.06 rad、FK位置0.2 mm/旋转0.002 rad接受门槛。
- VR初始配置：位置比例0.25、平滑新样本权重0.35、关节速度0.3 rad/s、controller步长0.15 rad、Adapter实测步长0.05 rad、pose/deadman分别250 ms时效。
- CR5 IK是输入驱动；I/O `rate_hz=50` 为每轮处理后等待20 ms，不是严格50 Hz定周期。反馈源频率与实际ServoJ时延待现场测量。控制页约5 Hz显示不代表控制频率。
- ServoJ字面参数t=0.1/lookahead_time=50/gain=500目前写在代码中；固件兼容与实际跟手效果未验收。

## 本次提交前复验

解释器 `/tmp/morla-cr5-vr-tests/bin/python`，Python3.13、Pinocchio4.1.0，macOS，无ROS/实体设备。

```bash
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer pure
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer fake
```

- 最新复验pure **144 passed**（7.45 s）、fake **23 passed**（2.87 s）；每层一条Starlette/AnyIO弃用警告，无跳过。原始输出位于 [提交前证据目录](evidence/2026-10-29-cr5-commit/)。不将两层与其子测试重复相加。
- 实际CR5数学/整链测试包括独立XML FK对照、100组邻近位姿、60帧连续轨迹、映射/限位、奇异保持、不可达/超时/坏模型、安装资源解析、真实factory/probe不使用控制器IK，以及手柄/裸手运动和撤权。
- 模型源等价、26个mesh/material逐字节对照、目标哈希闭包、实际Pinocchio几何加载通过。
- 输入/驱动最终wheel独立构建；临时share布局下实际加载URDF、controller和HTTP页面，验证手柄运动、释放和资源清理。公共源码与受控ament索引替代仍参与，不等于完整colcon安装。
- Python3.10测试依赖解析成功，未声称该平台已执行；静态AST/XML/JSON/YAML/URDF、JS语法与diff检查通过。
- 详细模型/wheel/数学证据见 [URDF实现记录](CR5_URDF_IK_2026-10-29.md) 与其证据目录；提交前文件清单/哈希另见本次证据manifest。

## 已知问题与验收边界

全库资产审计发现既有 `nero_left_arm -> urdf/nero/xhand_nero_description.urdf` 与旧清单目标哈希不符，本次未修改相关模型或旧清单，保留失败结果。CR5针对性模型/网格/材质哈希及真实几何加载通过。

NOT_RUN：ROS Humble colcon/IDL、真实ROS topic、Quest/CR5实物、模型与硬件零位/方向/限位、VR到base_link、Link6到TCP标定、停止/watchdog/断网/主机掉电、碰撞与长稳。LoopHand real原有停止确认/watchdog限制保留。软件通过和推送成功均不表示现场验收完成。

## 提交与推送策略

在当前main提交主内容，采用详细Git提交正文；先fetch核对远端基线，普通fast-forward推送，不force、不重写历史。永久origin仍为原HTTPS地址，使用既有SSH443与本机代理通道；凭据不入库。主内容成功推送并读回核对后，另提交实际回执并再次推送，最终核对HEAD、origin/main及远端refs/heads/main一致、工作区干净。

操作说明：[CR5 VR](cr5_vr.md)；频率/参数：[核对记录](teleoperation_rates_and_parameters.md)。
