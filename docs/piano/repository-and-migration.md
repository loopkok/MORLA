# 钢琴项目：仓库提交与异机恢复说明

检查日期：2026-10-22。依据本地 MORLA、同级 MORLA-piano 源码和现有部署记录；初次检查没有连接机器人服务器，也没有进行 ROS/真机验证。随后按用户要求提交源码，后续入库信息见下文。

## 本次入库基线

MORLA-piano 已初始化独立仓库并提交至 main，远端为 https://github.com/loopkok/MORLA-piano.git，配套引擎 commit 为 `1dcd032665cfa2876ab92667fb0ff3e0c3dc4f3c`。MORLA 本次同样使用现有 main 提交；部署这次版本时，将包含本段说明的 MORLA commit 与上述引擎 commit 配对。下文创建分支、初始化仓库的步骤是首次准备时的操作示例，无需在已经初始化的仓库重复执行。实际推送结果以 Git 远端为准。

本次入库检查：独立引擎 unittest 26 项通过；Web 前端 node 测试 18 项通过；pnpm frozen-lockfile 安装及 TypeScript/Vite 构建通过；MORLA 新增/修改 Python 文件 97 个通过语法解析。修正 pnpm-workspace.yaml 中 esbuild 构建许可的占位文本。

在本机 macOS Python 3.9 临时环境运行的 10 个 Nero/钢琴定向测试文件结果为 171 通过、6 失败、1 跳过：test_piano_lane 的完整演奏和并发准备用例，以及 test_piano_calibration 的软件 worker 用例等待超时；另外两个 arrival_timeout 用例期待 fault 事件、一个 3.01° 超时用例期待后端急停，与当前实际结果不符。未为本次入库更改运动代码或放宽测试；这些失败尚待专门核对。未运行 ROS 2、原生 C++ 驱动或实体机器人验收。

## 1. 应保存两个项目和一份现场数据

建议保持目前的双仓库结构，同步发布两个仓库的明确版本：

```text
~/robot-projects/
  MORLA/                    # ROS 工作区、Web、驱动、SDK、说明文档
  MORLA-piano/              # 独立演奏引擎、谱表、专用 launch
```

现场配置和曲谱另行备份，默认运行路径是 `MORLA/.morla_piano/`。若只需要离线规划和 fake 演奏，可以只下载 MORLA-piano；要使用目前的钢琴 Web 和双臂双手硬件，需要两个项目。

检查时 MORLA 在 main 分支，origin 为 https://github.com/loopkok/MORLA，存在大量未提交修改与新增文件；MORLA-piano 目录没有 `.git`。现有远端内容没有在本次检查中 fetch 或核实。不要将 `git push` 理解为上传当前整个文件夹：必须先把需要的文件 add 并 commit。

## 2. MORLA-piano 提交清单

以下均提交，保留目录层级：

- `morla_piano/__init__.py`、`__main__.py`、`cli.py`：包入口、命令行。
- `morla_piano/model.py`：配置、关节约束、曲谱解析和动作规划。
- `morla_piano/engine.py`：演奏状态机、时序和停止处理。
- `morla_piano/ros_backend.py`：ROS 状态读取、命令、控制权。
- `launch/piano_drivers.launch.py`：双臂、双手驱动及适配器启动；包含当前钢琴专用 J7 顺序和 J 模式参数。
- `config/piano.placeholder.json`、`config/position.example.json`：示例配置。
- `scores/demo.json`、`scores/template.csv`、`scores/ode_to_joy_C3_C5_40bpm.csv`：源码内谱表。
- `scripts/fake_xhands.py`、`scripts/ros_smoke.py`、`scripts/ros_smoke.sh`：软件后端和隔离联调。
- `tests/test_piano.py`：独立引擎验证。
- `pyproject.toml`、`.gitignore`、`README.md`、`CHANGELOG.md`、`VALIDATION.md`、`docs/DEBUG_LOG_2026-10-20_22.md`。

不提交 `logs/`、`__pycache__/`、`.pytest_cache/`、虚拟环境、构建输出。`config/piano.calibrated.json` 已被忽略，需要另行保存或使用有说明的配置快照。

注意：pyproject 的 Python 包发现仅包含 `morla_piano*`；不要只备份 wheel 或 site-packages。Web 按项目路径读取 `config/`，并直接使用外部 `launch/` 和 `scripts/`，应完整 clone 源码仓库。

## 3. MORLA 中相关项目和文件

为完整复现现有 Web，建议保存整个 MORLA 源码仓库，不按文件名只挑 `piano`。多个公共文件已参与这次功能，单独抽取容易漏掉依赖。

直接相关部分：

- `src/common/morla_web_manager/` 整包：`piano_manager.py`、`piano_worker.py`、`piano_store.py`、`piano_calibration.py`、`piano_maintenance.py`、`piano_processes.py`；同时保留 `web_server.py`、`config.py`、其他公共模块、launch、setup 和 package.xml。
- 同包的 `web/`：整个前端源码和配置，特别是 `PianoTab.tsx`、`PianoManual.tsx`、`piano.css`、`api/piano*.ts`、`pianoTypes.ts`、`App.tsx` 和公共 API/样式；提交 `package.json` 和选定构建流程的锁文件。
- 同包的 `test/` 和 `web/test/`：钢琴 API、心跳、反馈、标定、进程、回放等回归测试。
- `src/robots/nero/nero_robot_control/` 整包：驱动节点、SDK 适配、HOME/工作位轨迹、关节约定、左右配置及测试。现有修改不是上游默认驱动可以直接替换的。
- `src/robots/nero/nero_arm_teleop/`：钢琴笛卡尔标定会导入 `ik_solver.py`，因此不用遥操也不能简单删掉此包。
- `src/robots/xhand/xhand_adapter/`：JointState 与原生 XHand 消息转换、控制权及反馈检查。
- `src/robots/xhand/xhand_control_ros2/`：原生 C++ 驱动、`xhand_control_interfaces` 消息与随附 XHand SDK。原生驱动含本次钢琴反馈与诊断修改。
- `src/common/morla_core/`：公共工具和 Profile 注册；`src/common/morla_bringup/`：Web 依赖的启动和协调模块。
- `src/robot_description/`、`src/robots/nero/morla_nero_robot/`、`src/robots/xhand/morla_xhand_hand/`：模型、关节限位、组件及 Profile 配置；保留配套资产。
- `src/common/morla_operator_control/`、`src/common/morla_input/` 等公共包：现有 Web launch 和 package.xml 引用；不开启输入不意味着可以不经分析就删除包。
- `third_party/pyAgxArm/`：本地收录的 Nero SDK，版本文件标为 1.0.0；clone 后仍须安装到 ROS 使用的 Python。
- `scripts/build.sh`、`install_pyagxarm_editable.sh`、`nero_web_start.sh`、`morla_stop_processes.py`、`test_piano_web_ros.py`、`piano_watch_shift.py` 和相关 `scripts/tests/`。
- `README.md`、`CHANGELOG.md`、`docs/piano/` 全部、`docs/nero_web_start.md`、关节约定等说明。

完整 MORLA 的 manifest 还包含 Astral、Wuji、采集、策略、视频和仿真包。这些并非专用钢琴 launch 的直接运动链路；保留源码可避免破坏现有整库构建与 Web。裁剪成最小发行版需要另做依赖清理和验证，本次没有把上述清单宣称为已验证的最小包集合。

## 4. 提交与排除规则

应提交 changelog、人工整理的 Markdown 调试记录、配置归档与部署哈希清单。它们是文档，不应因为含有“日志”二字就删掉。

默认不提交 `build/`、`install/`、`log/`、`.pnpm-store/`、`.pytest_cache/`、`node_modules/`、虚拟环境、原始运行日志、采集数据和 `.morla_piano/`。MORLA 当前忽略规则缺少 `.pnpm-store/` 和 `.pytest_cache/`；MORLA-piano 缺少 `.pytest_cache/`、`.DS_Store`，推送前补齐。

例外：XHand `xhand_control/lib/libxhand_control.so` 是供应商运行依赖，已在 Git 跟踪，不是 colcon 生成文件；不要新增 `*.so` 通配忽略而误删它。其 include、share/CMake 配置也需保留。对外分发前确认供应商许可；当前 ROS package.xml 的 license 为 TODO，不能据此假定可公开再分发。可先使用私有仓库保存工程。

前端当前已有 dist 被跟踪，建议这次继续随源码一起提交构建后的完整 `web/dist/`，包括旧 hash 资源删除、新资源加入和 index.html 更新。以后若决定不提交 dist，需要统一移除跟踪并在部署流程里强制构建。本次不要混用两种策略。

当前同时有 npm 与 pnpm 锁文件，现有 README 使用 `npm ci && npm run build`。下文沿用 npm；需要实际执行成功，才能确认 package-lock 与当前 package.json 一致，不要未经验证删除另一个锁文件。

## 5. 当前迁移缺口

1. `scripts/nero_web_start.sh` 必须读取 `scripts/can_muti_activate.sh`，但本地没有此文件。需从实际主机取回并审查 USB-CAN 绑定，或者按新主机建立对应脚本。发现 CAN 的脚本可回退到内置 pyAgxArm，但激活脚本没有同样回退。未补齐时一键入口会直接退出。
2. 现有 `libxhand_control.so` 为 ELF Linux x86-64。目标应优先采用 Ubuntu 22.04 x86-64、ROS 2 Humble、系统 Python 3.10；ARM/Jetson 需要匹配架构的供应商 SDK，不能直接用当前库。
3. 本机没有运行目录 `.morla_piano/`。部署记录中配置、Web 曲谱和会话在服务器 `/home/loopkok/MORLA/.morla_piano/`，本次没有验证其当前内容。
4. `docs/piano/snapshots/config-2026-10-22.json` 是历史归档，不是实时入口；文档已记录若干尚待验证事项。不能用历史快照或占位配置自动覆盖当前现场配置。
5. 双仓库要记录成对的 Git commit SHA。已有 deployment manifest 是当时文件哈希，明确不是 tagged release，也不覆盖所有工程依赖。

## 6. 如何推送

以下是供执行的命令，本次未执行。先完成忽略规则、CAN 脚本补齐及前端构建。示例使用新的 MORLA 分支保存现有整套修改；不要在未看暂存差异时把所有修改当成已审查过的钢琴改动。

给 MORLA 的 `.gitignore` 补充：

```gitignore
.pnpm-store/
.pytest_cache/
```

给 MORLA-piano 的 `.gitignore` 补充：

```gitignore
.pytest_cache/
.DS_Store
```

MORLA 提交和推送到现有远端：

```bash
cd /Users/a1234/Desktop/MORLA
git switch -c codex/piano-portable
cd src/common/morla_web_manager/web
npm ci
npm run build
cd /Users/a1234/Desktop/MORLA
git status --short
git add -n -A
# 确认预览没有缓存、数据、凭据；完整保存现有工作区时：
git add -A
git diff --cached --stat
git diff --cached --check
git diff --cached
# 审查完成后执行：
git commit -m "Add piano workbench and Nero XHand integration"
git push -u origin codex/piano-portable
git rev-parse HEAD
```

若只想提交钢琴功能而保留其他未提交工作，应按路径/差异逐项暂存；上述 `git add -A` 会包含所有未被忽略的现有修改。

在 GitHub 创建一个空的 `MORLA-piano` 仓库，先不生成 README/License/gitignore。将下面 `REMOTE_URL` 替换为实际新仓库地址：

```bash
cd /Users/a1234/Desktop/MORLA-piano
git init -b main
git add .gitignore README.md CHANGELOG.md VALIDATION.md pyproject.toml
git add morla_piano config launch scores scripts tests docs
git diff --cached --stat
git diff --cached --check
git diff --cached
git commit -m "Initial piano engine and ROS integration"
git remote add origin REMOTE_URL
git push -u origin main
git rev-parse HEAD
```

将两次 HEAD 输出记录在部署说明或 release 中，部署时检出这两个确定版本。`git push -u` 负责推送分支并建立 upstream；遇到远端已有独立提交，应先 fetch 审查并整合，不使用强推。

## 7. 现场数据怎样带走

从实际机器人主机备份 `.morla_piano/config.json` 与整个 `scores/`。若需要保留故障证据，也备份 `sessions/`、`diagnostics/`，但它们不是启动必需条件。停用钢琴会话并停止 Web 后再复制，避免跨文件快照不一致。

下面命令在旧机器人主机执行，先确认目录是当前运行数据路径：

```bash
cd /home/loopkok/MORLA
tar -czf "$HOME/morla-piano-data-20261022.tar.gz" \
  .morla_piano/config.json .morla_piano/scores
sha256sum "$HOME/morla-piano-data-20261022.tar.gz"
```

通过 scp 等复制到新主机，校验摘要后在新 MORLA 根目录解包；仅对新目录或已另行备份的数据执行恢复。归档不要放进源码仓库。若换了机器人、安装位置或钢琴，旧姿态只能作为参考，必须重新确认设备绑定与实体标定。

## 8. 在新主机拉取和构建

前置环境：安装与 ROS Humble 匹配的 Ubuntu/ROS、Git、系统 Python/pip、colcon、rosdep、C++ 编译工具，以及能满足当前前端 Vite 要求的 Node/npm。以下假定 ROS 已安装且 rosdep 已初始化。`PIANO_REMOTE_URL`、`MORLA_COMMIT_SHA`、`PIANO_COMMIT_SHA` 均替换为自己的实际值。

```bash
mkdir -p ~/robot-projects
cd ~/robot-projects
git clone https://github.com/loopkok/MORLA.git
git clone PIANO_REMOTE_URL MORLA-piano
git -C MORLA checkout MORLA_COMMIT_SHA
git -C MORLA-piano checkout PIANO_COMMIT_SHA

source /opt/ros/humble/setup.bash
cd ~/robot-projects/MORLA
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
# 使用 ROS 的系统 Python，避免 conda：
PATH=/usr/bin:/bin:$PATH bash scripts/install_pyagxarm_editable.sh

cd src/common/morla_web_manager/web
npm ci
npm run build
cd ~/robot-projects/MORLA
bash scripts/build.sh --mode symlink
source install/setup.bash
python3 -m morla_core.profile validate-all
```

rosdep 读取的是整个多机器人工作区依赖；外部 Python 包、供应商库和可选模型仍可能需要按各包说明安装。这些命令是基于源码的恢复路径，本次未在干净 Ubuntu 主机验证，不能当作已经通过的全自动安装器。

独立引擎在源码目录即可运行，Web 也会注入项目源码路径，通常无须另行 pip 安装 MORLA-piano：

```bash
cd ~/robot-projects/MORLA-piano
python3 -m unittest discover -s tests -v
python3 -m morla_piano plan scores/demo.json --out /tmp/piano-plan.json
python3 -m morla_piano run scores/demo.json --fast

# ROS 软件链路验证，不连接实体硬件：
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
bash scripts/ros_smoke.sh
```

## 9. Web 与硬件恢复

纯钢琴先不接 Quest/语音。新终端中加载环境，并明确路径：

```bash
source /opt/ros/humble/setup.bash
source ~/robot-projects/MORLA/install/setup.bash
export MORLA_WS="$HOME/robot-projects/MORLA"
export MORLA_PIANO_PROJECT="$HOME/robot-projects/MORLA-piano"
export MORLA_PIANO_DATA="$MORLA_WS/.morla_piano"
export MORLA_PIANO_PYTHON=/usr/bin/python3
export MORLA_WEB_MANAGER_DIST="$MORLA_WS/src/common/morla_web_manager/web/dist"
ros2 launch morla_web_manager web_manager.launch.py web_host:=0.0.0.0 web_port:=8080
```

先在浏览器的钢琴工作台选择纯软件或 ROS fake 模式验证；硬件模式前恢复/核对数据、CAN 名称和串口。CAN 工具包括 `ip`、`ethtool`、`candump`；现有脚本约定 `can_nero_left`、`can_nero_right`，1 Mbit/s，USB 绑定必须按新机器实际连接配置。XHand 应使用 `/dev/serial/by-id/` 稳定路径和串口权限，而不是照搬旧 ttyUSB 序号。

CAN 激活脚本补齐且硬件已连接后，才使用一键入口：

```bash
cd ~/robot-projects/MORLA
bash scripts/nero_web_start.sh --no-quest --dry-run
bash scripts/nero_web_start.sh --no-quest
```

dry-run 本身也要求硬件/文件存在。正式入口会激活 CAN、修改串口权限并清理旧 MORLA 进程；它不是通用的无硬件 Web 启动器。之后由操作员在页面建立会话，不将启动成功等同于标定或实体演奏验收。

## 10. 现有文档如何使用

- 两边的 CHANGELOG：说明版本改了什么。
- 两边的 DEBUG_LOG：保存诊断经过、现场结果和未验证边界。
- MORLA `docs/piano/snapshots/`：历史配置及文件哈希，保留追溯。
- 本文件：说明提交哪些内容、缺哪些外部条件、如何推送和在新主机恢复。

Git 行为参考：[git push](https://git-scm.com/docs/git-push)、[gitignore](https://git-scm.com/docs/gitignore)。注意 `.gitignore` 不会自动取消已经跟踪的文件。
