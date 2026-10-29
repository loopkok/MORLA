# CR5 VR 遥操实现与验证记录

> 历史阶段记录：控制器内置 IK 和合成 fake 求解已由后续 [URDF / Pinocchio 本地 IK](CR5_URDF_IK_2026-10-29.md) 替代。下述测试结果仅对应当时实现。

日期：2026-10-29。对应需求：参考 Nero 已有 VR 相对位姿遥操方式，用右手柄或右手追踪控制独立 CR5。

## 交付内容

软件链为 Quest 接收器 → 来源明确的右侧事件 → `QuestRightSource` → 公共 `BindingController` → CR5 异步 Cartesian controller → runtime 网关 → CR5 Adapter。启动、编译、操作和真实部署步骤见 [CR5 VR 操作说明](cr5_vr.md)。

- `morla_input` 新增静态组件 `morla.input.quest_right`，输出 `right_pose` 与 `right_hold`。接收器事件流默认关闭，CR5 launch 显式开启，不替换原 Nero 使用的话题。
- 右手柄按握把保持；裸手以手腕位姿控制、拇指尖与食指尖相对掌宽捏合保持。释放立即撤权，来源/会话/frame 改变清锚，重新激活后必须先释放再按下。按钮/关键点只更新保持状态，运动由 pose 帧驱动，防止同包按钮行覆盖运动目标。
- `morla_cr5_driver` 新增 `morla.cr5.cartesian_delta` 插件、fake Assembly、真实 Deployment 模板、launch、本地操作页和只读运动学探针。运动学调用由 Adapter 持有的同一 Dashboard 串行执行，不增开绕过 runtime 的运动通道。
- 复用公共相对锚点和 TCP 算法；真实 FK/IK 来自 CR5 V3 控制器，显式换算 mm/degree 与 m/rad，带实测关节参考，验证求解结果的 FK 误差。fake 合成模型仅验证协议。
- 后台 worker 采用一个待处理槽，generation 隔离旧任务；失败锁存至 reset、过期拒绝、输出限速。真实激活要求新鲜输入及显式标定声明，首次目标前断流仍撤权。停止会话关闭 worker，关闭报错不会跳过 runtime 停止。
- 公共编译锁包含选中的 controller descriptor；公共 Web 应用支持可选专用页面。CR5 设备包不依赖 Nero、Astral 或 LoopHand；专用 VR 宿主可选使用现有公共 Web 管理应用。

本轮没有修改 Nero/Astral 专用 IK，也没有增加 CR5 与 LoopHand 联合驱动。操作页启动后保持 disarmed，不自动恢复旧控制权。

## 已执行验证

在 macOS、无 ROS/无实体硬件环境执行；解释器为独立 `/tmp/morla-cr5-vr-tests/bin/python`（Python 3.13）。未改变系统 Python。可复验命令：

```bash
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer pure
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer fake
```

- 纯软件层：**138 passed**，见 [pure.txt](evidence/2026-10-29-cr5-vr/pure.txt)。包含坐标单位、V3 命令、FK 回验、限位/参数、源时效、手柄/裸手切换、捏合中间区、独立 deadman 年龄、异步失败/迟到解、真实输入激活门槛和首次目标前失联撤权。
- fake 集成层：**23 passed**，见 [fake.txt](evidence/2026-10-29-cr5-vr/fake.txt)。覆盖现有数据/策略/Web 回归。
- wheel 验证：分别构建最终 `morla_input` 与 `morla_cr5_driver` wheel，解包到独立临时安装前缀，验证静态发现、controller entry point、安装后的 HTML HTTP、右手柄动作、释放撤权和关闭清理。输入与 CR5 从 wheel 加载，其他公共框架从仓库源码加载；禁止依赖其他机器人 Python 包。仅替代 ament 索引查询，未替代实际 runtime/controller/Adapter。见 [installed.txt](evidence/2026-10-29-cr5-vr/installed.txt) 及 [复验脚本](evidence/2026-10-29-cr5-vr/verify_wheels.py)。它不是完整 ROS 安装测试。
- 新增/修改 Python AST、package XML、JSON/YAML、页面内联 JavaScript 语法与 diff 空白检查结果见 [static.txt](evidence/2026-10-29-cr5-vr/static.txt)。证据与本轮源码 SHA-256 见 [manifest.json](evidence/2026-10-29-cr5-vr/manifest.json)。

pytest 各层有一条 Starlette/AnyIO 弃用警告，不影响通过结果。旧 Python 3.12 临时环境缺 pandas 且 SciPy 二进制不可加载，未将该环境中的失败计为通过；最终以上述独立环境重跑结果为准。

## 尚未执行的现场验收

ROS Humble colcon/IDL 与真实 ROS topic 传输、Quest APK/手追踪、CR5 V3 固件实际命令响应及耗时、关节方向/零点/限位、base/TCP 与 VR 旋转标定、使能/停止/断网/watchdog、碰撞及长稳均为 **NOT_RUN**。只读 FK→IK→FK 通过也不能替代物理标定。

真实示例保留 false 声明和占位身份，没有伪填现场参数。现场补齐与验证后才能按操作说明启用 real。此记录不声称已获得此前 Nero 实机遥操同等级的验证。

本轮为工作区实现记录；没有为本轮新增提交或推送回执，历史推送记录保持原含义。
