# 公共工作站本地更新集中实施日志

日期：2026-11-02，Asia/Shanghai。仓库：loopkok/MORLA，分支：main，实施基线：`1e90a89`。本次将此前连续实施、尚未提交的功能与检查文档集中归档；提交前已 fetch origin，确认 HEAD 与 origin/main 无分叉、无远端新增提交。实际提交 SHA 和推送结果由后续推送回执记录。

## 范围与结果

本次包含公共仿真后端、CR5 IK、Web 配置与操作流程、独立运行诊断、统一输入操作、公共工作站人在环内切换，以及 Astral 功能对照报告。保留此前本地实现，没有为提交而扩大功能范围；Astral 对照报告中的待办仅记录，不在本次宣称实现。

### 公共 MuJoCo 与机器人包入口

- 公共 Runtime 支持 kinematic / physics。前者直接应用关节目标并执行 mj_forward；后者使用执行器目标和 mj_step。切换模式需停止运动输出并满足采集/策略约束，保持当前姿态、清理旧目标与速度。
- 新增独立 `cr5_mujoco_sim` ament_python 包，包含配置、launch 和 CLI。Astral/Nero 保留原包、原仿真入口，并增加 `composable_sim.launch.py` 和公共装配模板。
- Web 与 CLI/launch 支持 web / local / both。独立本地 viewer 读取公共场景快照，不另起控制器或物理时钟；关闭窗口不停止仿真，停止会话负责清理进程。
- 新模板默认 kinematic/web；未声明模式的旧公共配置保持 physics/web。运行模式进入状态和采集来源事件。物理计算不代表已完成碰撞验证或 sim2real 参数辨识。
- 细节：[公共仿真日志](SHARED_MUJOCO_2026-11-02.md)。

### CR5 连续微分 IK

- 新增 CR5 自身 URDF/Pinocchio 六维 Jacobian 单步 DLS，结合自适应阻尼、目标领先限制、超量吸收、关节限位和有界线搜索；保留严格迭代求解模式。
- CR5 专用 Quest/jog 控制器默认 differential；绑定可选择 strict，保存并重启会话生效。IK 模式与仿真物理模式独立。
- 保留异步工作线程、最新帧覆盖、反馈播种、generation 隔离、输入平滑与最终速度限制；重锚清除旧解和内部目标。
- Web/诊断区分原始目标和有效目标残差、候选接受与收敛状态，显示奇异点接近度、阻尼、吸收量和报告龄期。候选 FK 残差不等于真实 TCP 测量精度。
- 细节：[CR5 IK 日志](CR5_DIFFERENTIAL_IK_2026-11-02.md)。

### Web 工作站配置与流程

- 将组合配置组织为设备选型、安装与标定、遥操作设置、数据记录、检查与运行，调整中文术语、页签顺序和工程控制台样式。
- 安装位姿、TCP、输入轴与控制映射、硬件连接参数、控制组和采集投影由表单生成配置；支持按组件 schema 配置附加参数并保留未知字段。
- 运行期间配置只读，导入导出放入可选工具；修正切换控制器遗留不兼容参数，增加工作位与插补速度表单及键盘启用入口。
- 提交包含 web/dist 的 JS/CSS 与 index.html，避免部署使用旧页面；本轮重建产物为 `index-B0pAyV6U.js` / `index-N_1VrTr9.css`。
- 细节：[Web 表单日志](WORKCELL_WEB_FORMS_2026-11-02.md)。

### 独立公共诊断日志

- Runtime 按次运行默认保存独立诊断，不以启动数据采集为前提；记录输入身份、门控、映射输出、命令回执、反馈、生命周期及异常。
- 公共控制器/CR5 异步工作线程提供计算报告与实际对应的输入帧，避免旧结果误标为当前输入。
- 使用有界快照、非阻塞队列、后台分段写盘、容量限制、脱敏和丢弃/故障计数；日志错误不改变运动许可。
- Web 提供日志参数、历史运行、最近记录和下载；CLI/launch/ROS 提供目录入口。正常停止写摘要，缺摘要的异常退出运行标为不完整。
- 细节：[公共诊断日志](COMMON_TELEOP_DIAGNOSTICS_2026-11-02.md)。

### 最终操作分配与采集协调

以下是本次提交的最终公共工作站默认分配，替代同日实施历史中的 Z/C 等旧默认键位：

- Quest 右 grip / 键盘 S / Vader M3：开始遥操、记录跟随零点；策略或回放中用于人工接管及 hold。
- Quest 右 A / 键盘 A / Vader A：普通模式开始录制；人工接管状态下交还策略或回放。
- Quest 右 B / 键盘 B / Vader B：结束当前录制并保存。
- Quest 右摇杆按下 / 键盘 N / Vader R3：丢弃当前录制段。
- Quest 左 X / 键盘 A+N / Vader X：停止跟随并直接回工作位。

Web 按钮使用「开始遥操／暂停遥操」。A/N 单键释放执行、组合键抑制单键；键盘有输入框、失焦、hold 心跳与超时保护。已发行的完整 Vader 默认映射可识别升级，显式自定义映射保留。工作位需要显式配置，未配置不自动回零；丢弃不删除已保存录制段。

细节：[统一操作日志](UNIFIED_WORKSTATION_OPERATORS_2026-11-02.md)。

### HITL 对齐与公共命令仲裁

- 接管采用冻结输出、设备完成确认、逐映射实测 FK/输入重锚、全部 ACK 后原子提交 HUMAN。失败/超时回滚，取消与迟到 ACK 不恢复旧代次权限。
- 交还策略先获取当前观测的新预测；失败保留人工。回放按实测姿态重锚剩余轨迹 offset，检查后续限位，保留游标及原暂停状态。
- 暂停保留自动控制上下文；显式继续恢复运行。停止、急停、复位和回位清理交还上下文。
- 公共工作站给 Runtime 安装分发前 command_guard，检查 producer、token、epoch、租约、资源和冻结/暂停状态。独立 Runtime 的兼容路径及绕过 Runtime 的 SDK/原生话题不自动获得该仲裁。
- 该协议对按接口接入的机器人复用；不代表 Astral 肘部先验等品牌运动学行为自动公共化，也不代表 Headless 入口已自动组装全部协调器。
- 细节：[HITL 对齐日志](WORKSTATION_HITL_ASTRAL_ALIGNMENT_2026-11-02.md)。

### Astral 对照检查

对照桌面 astral_ws 与当前 MORLA 的实现和实际调用链，整理 18 组待补齐/公共化能力（8 组专用路径保留、9 组部分公共化、1 个独立工具未检出），另列 7 组公共能力和 1 处文档不一致，包含 75 处代码位置。这些数字是检查分组，不是覆盖率。

优先待办包括统一运动输出约束、采集任务文本一致性、在线质量状态、完整推理调度及模型服务入口。特别记录公共操作键开始采集仍固定使用 `operator demonstration` 的现状。本次不修改这些审计发现，不把“已发现”写成“已修复”。Canvas 位于本机 IDE 管理目录，不属于仓库提交；可移植的完整检查结果保存为 [审计报告](ASTRAL_MORLA_PUBLIC_PARITY_AUDIT_2026-11-02.md)。

## 本轮提交前验证

本轮重新运行的结果与各阶段日志的历史验证分开记录：

- 核心合同、Runtime、遥操、学习、CR5/LoopHand/EPG 驱动、组合示例、Quest 事件源、标准输入、Vader：**265 passed**，10.37 s。使用 Python 3.13 的 `/tmp/morla-cr5-vr-tests` 环境，具备 Pinocchio 4.1.0、MuJoCo 3.12.0，实际运行微分 IK 和仿真测试。
- 采集、公共策略、Web API、工作站操作/HITL：**45 passed、1 deselected**，6.01 s。
- robot_description 模型/公共场景：**12 passed**，3.92 s。
- 前端工作站和键盘逻辑：**13 passed**；`tsc -b` 通过；Vite 5.4.21 构建通过，66 modules，JS 331.45 kB / gzip 114.79 kB，CSS 15.84 kB。
- 以上合计 **322 项 Python 测试通过**，不包含下列排除、未运行或失败尝试。出现 Starlette/httpx 弃用警告，未将其解释为产品测试失败。

异常与限制：

- `test_mobile_input_and_recording` 返回 `release_or_reanchor_required`。在临时隔离目录恢复 HEAD 的 Web API/service/CLI 后同样失败，确认既有基线问题；最终套件显式排除此项，没有修改输入控制语义规避。
- 当前 macOS 环境无 `rclpy`，直接运行完整 pure 入口在收集 `test_teleop_events.py` 时失败。最终核心测试清单未包含该文件的 2 项 ROS 事件发布测试；没有用 mock 的结果冒充 ROS 验收。
- 首次 fake 测试误用 `--import-mode=importlib`，导致原采集测试里的 `from conftest import make_jpeg` 无法导入；恢复项目默认导入方式后该测试通过。最终命令不使用该选项。
- 最初使用系统 Anaconda 环境缺少可选 Pinocchio，随后改用已有 IK 测试环境运行核心套件，最终 265 项无可选依赖跳过。

### 可复现命令

在仓库根目录执行，先选择已安装相应依赖的 Python。下列路径处理与仓库测试脚本一致，不要求 `/tmp` 内的私有辅助脚本：

```bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
for package_dir in "$PWD"/src/common/* "$PWD"/src/robots/*/*; do
  if test -f "$package_dir/package.xml"; then
    export PYTHONPATH="$package_dir${PYTHONPATH:+:$PYTHONPATH}"
  fi
done
python -m pytest -q \
  src/common/morla_core/test/contracts src/common/morla_runtime/test \
  src/common/morla_teleop/test src/common/morla_learning/test \
  src/robots/dobot/morla_cr5_driver/test src/robots/loophand/morla_loophand_driver/test \
  src/robots/jodell/morla_epg_driver/test examples/cr5_loophand/test \
  src/common/morla_input/test/test_quest_events.py \
  src/common/morla_input/test/test_standard_source.py src/common/morla_vader5_input/test
python -m pytest -q \
  src/common/morla_data_collect/test/test_composable_data.py \
  src/common/morla_policy_inference/tests/test_composable.py \
  src/common/morla_web_manager/test/test_composable_api.py \
  src/common/morla_web_manager/test/test_workstation_operator.py \
  -k 'not test_mobile_input_and_recording'
python -m pytest -q src/robot_description/test
```

ROS 环境完整时，另外运行 `scripts/test_composable.sh --layer pure`，包含上述未运行的 ROS 测试。前端在 `src/common/morla_web_manager/web` 执行 `npm run test:workcell` 和 `npm run build`；本轮本机缺少 npm 命令，使用同一 Node 直接调用测试、TypeScript 与 Vite 的对应入口，均成功。

## 部署与验收边界

更新后重启 Web/输入进程并刷新页面。装配模板不会无条件覆盖已保存配置；需要检查当前配置、保存新版本并启动。IK 等绑定参数保存后重启生效；运行时仿真显示/模式修改只作用当前会话，要持久化需更新保存配置。

历史阶段日志中记录了 macOS viewer、短时物理闭环与 Edge 交互检查；本轮提交前没有重复进行这些操作，也没有启动实际机器人或真实模型服务。ROS Humble colcon/launch、实体 Quest/Vader、实机长稳、碰撞和物理参数验收仍未完成。远程 CI 结果不由本地测试推断。

不提交运行数据、临时截图、个人 Canvas、虚拟环境、日志采集目录或依赖缓存。仓库原本跟踪的 web/dist 随源码更新保留。
