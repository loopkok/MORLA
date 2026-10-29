# 验收收尾修复推送回执：2026-10-29

记录时区：Asia/Shanghai。仓库为 <https://github.com/loopkok/MORLA>，分支 `main`；未使用 force，未重写既有提交。起点为 `ddcb974d`（`docs: record new-host piano deployment, CAN map and push receipt`），其上的 CI 已绿。

## 提交内容

| 提交 | 主题 | 范围 |
| --- | --- | --- |
| `e1dae63` | `fix(piano): report the final state before the job result` | `piano_worker.py`、3 个钢琴测试文件 |
| `e6b395b` | `fix(packaging): drop invalid ament_python and vendor-only rosdep keys` | 15 个 `package.xml`、`VENDORED_IN_XNERO.md` |
| `a6d09da` | `docs(changelog): record acceptance closeout for piano, R04 and rosdep` | `CHANGELOG.md` |
| `e29c46f` | `docs(changelog): restore the section heading displaced by the new entry` | `CHANGELOG.md` |

变更统计（`ddcb974..e29c46f`）：21 files changed、88 insertions、31 deletions。

### 钢琴集成

- `piano_worker.py`：原先在收尾 `snapshot` 之前发 `result`，管理器收到 `result` 即清空当前作业，调用方会在作业“完成”的瞬间看到作业前状态并拒绝紧随其后的命令（全量套件中 `test_finished_shifted_performance_can_explicitly_prepare_and_replay` 偶发“演奏前先准备到初始位”）。改为先清空 `active_job`、发收尾 `snapshot`、再发 `result`，并注释顺序约束。
- 测试对齐既有引擎语义（外部工程 `MORLA-piano` `b68a4b5` 未修改）：管理器/worker 用例使用注入 `Clock = VirtualClock` 的项目副本，保留一项真实时钟、等待 180 s 的完整轨迹用例；`MotionConfirmationError` 断言改为 `arrival_timeout` → `motion_unconfirmed`、无 `fault`、`STOPPED` 且不断电。

### 包清单

- 13 个第一方 `package.xml` 删除无效的 `<buildtool_depend>ament_python</buildtool_depend>`（`ament_python` 是 build type，不是 rosdep key）；`<export><build_type>ament_python</build_type></export>` 保留。
- `wujihand_driver`、`wujihand_bringup`、`morla_epg_driver` 移除无 rosdep 键的 `wujihandcpp`、`wuji_description`、`python3-modbus-tk` 声明，就地注明来源；`VENDORED_IN_XNERO.md` 说明 `WUJIHANDCPP_PREFIX`/`~/.local/opt`/源码树查找与 vendor 安装要求。

## 提交前实际验证

- `xmllint --noout` 通过全部改动清单；`git diff --check` 无空白错误。
- `colcon`：symlink 与 regular 两种模式各 40 包 `finished`，退出 0。
- 源码层：pure **170 passed, 2 skipped**（`pure-final.txt`）；fake **23 passed**、sim **7 passed**（`layers-final.txt`）。
- Web manager 全量（ROS Humble 系统 Python + 同级 `MORLA-piano`）：修复后连续两轮 **288 passed, 2 skipped**（约 118 s/轮，`web-manager-tests-final.txt`、`web-manager-tests-final-2.txt`）；钢琴子集 **228 passed**。
- R04 故障注入：`cr5-loophand`、`cr5-epg` 两个 fake 装配各 **11/11** 项通过，`r04-faults.json` 为 `passed:true`、`failures:[]`（`r04-faults.py` 为一次性证据脚本，位于验收证据目录，未纳入仓库；同组部分失败停止的常驻回归由 `morla_runtime/test/test_runtime.py::test_partial_dispatch_stops_all` 覆盖）。
- `rosdep check`：未知键清零，仅剩 10 个真实 apt 包（fastapi、uvicorn、pydantic、msgpack、serial、aiortc、pandas、realsense2-camera、usb-cam、rosbag2-storage-mcap），见 `b01-rosdep-after-fix.txt`；本机无 sudo，未执行 `rosdep install`。

证据目录：`/home/loopkok/morla-acceptance/20261029-hostcheck/`，其中 `report.md` 的 T01、R04、B01 与钢琴集成条目已按本轮结果更新。

## 实际推送与核对

1. 推送前 `git fetch origin main` 后 `origin/main` 为 `ddcb974d`，本地领先 3 个提交。
2. `git push origin main` 退出 0：`ddcb974..a6d09da main -> main`（origin 为 SSH `git@github.com:loopkok/MORLA.git`）。
3. 发现的 CHANGELOG 标题被新小节挤掉后，补 `e29c46f` 并再次推送：`a6d09da..e29c46f main -> main`。
4. 推送后 `git fetch origin main`，`git rev-parse HEAD origin/main` 两行均为 `e29c46f38afea743e52859a7fcdca71c87862d81`。

## CI

- 工作流：`Composable contracts and fake integration`（`.github/workflows/composable.yml`）。
- `a6d09da`：run `40886013330`，`contracts (3.10)`、`contracts (3.12)`、`web` 均成功（已被后续文档提交取代）。
- `e29c46f`（当前 HEAD）：run `40886129886`，三个任务均成功：
  - `contracts (3.10)` job `123490214268`；
  - `contracts (3.12)` job `123490214505`；
  - `web` job `123490214141`。

复验命令：

```bash
git fetch origin main
git rev-parse HEAD origin/main
git status --short --branch
```

## 边界

本回执不改变硬件验收结论：无 CR5/EPG/LoopHand/Vader5/Quest/相机硬件接入，C/Q/L/V/E/N/S/D(real)/P 组仍为 BLOCKED_ENV，L03 真实运动为 BLOCKED_CAPABILITY，R04 的注入为 fake+源码层、不等于真机时序。本文件与 `CHANGELOG.md` 的收尾说明另作一个文档提交推送，其自身 SHA 由 Git 历史记录，推送结果以实际命令返回和远端 SHA 核对为准。

## 补记：B01 依赖关闭（2026-10-29 22:11）

操作员以 sudo 直接 apt 安装（rosdep 在 root 下因 `raw.githubusercontent.com` 超时未能 `update`，改用已知包名）：

```bash
sudo apt install -y python3-fastapi python3-uvicorn python3-pydantic python3-msgpack \
  python3-serial python3-aiortc python3-pandas \
  ros-humble-realsense2-camera ros-humble-usb-cam ros-humble-rosbag2-storage-mcap
```

- 安装版本：aiortc 1.3.0-3、fastapi 0.63.0-2ubuntu0.1、msgpack 1.0.3-1build1、pandas 1.3.5+dfsg-3、pydantic 1.8.2-1、serial 3.5-1、uvicorn 0.15.0-2ubuntu1、realsense2-camera 4.58.4、usb-cam 0.8.1、rosbag2-storage-mcap 0.15.17。
- `rosdep check --from-paths src --ignore-src --rosdistro humble` 退出 0：`All system dependencies have been satisfied`（`b01-rosdep-pass.txt`）。
- 运行时仍使用用户 site 的 pip 版本（fastapi 0.141.1、uvicorn 0.54.0、pydantic 2.13.4、msgpack 1.2.4、aiortc 1.15.0、pandas 2.3.3），apt 版本仅满足 rosdep 归属；仅 `pandas` 出现 numexpr/bottleneck 版本告警（自动回退，不影响功能）。
- 能力核对：`rosbag2_py.get_registered_writers()` 含 `mcap`，`ros2 pkg executables usb_cam` 有 `usb_cam_node_exe`，`quest3_video_streamer` 42 passed，web manager 全量 288 passed/2 skipped（无回归）。真实相机/串流设备仍未接入，S01/S02/Q 组硬件结论不变。

## 补记：EPG 原厂 SDK（2026-10-29 22:23）

- 用户提供 `/home/loopkok/JodellTool-0.1.5-py3-none-any.whl`，sha256 `b651a7f25e5b8ce97433a62f362a83ded93d6dd3105ebc0f512d72127c036963`，与 `src/robots/jodell/morla_epg_driver/PROVENANCE.json` 记录的原文件一致。
- `python3 -m pip install --user` 安装成功，附带 `modbus-tk 1.1.5`；`jodellSdk.jodellSdkDemo.EpgClawControl` 与 `modbus_tk` 均可导入。
- `morla_epg_driver` 测试 **17 passed**，无 skip：此前因缺 SDK 跳过的两项真实 SDK 契约测试现实际执行（真实 `EpgClawControl` + 记录型 Modbus master 的寄存器序列；real 打开路径的只读/超时/独占参数）。
- 边界：真实固件、RS485 接线、物理停止（SDK `0x03FF` 与手册 `0x03E8` 位差异）和掉线行为仍需实物验证，E01/E02/E03 维持 BLOCKED_ENV。证据：`epg-sdk-install.txt`。
