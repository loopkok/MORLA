# 新主机部署脚本与 CAN 映射推送回执：2026-10-29

记录时区：Asia/Shanghai。仓库为 <https://github.com/loopkok/MORLA> ，分支 `main`；未使用 force，未重写既有提交。

## 提交内容

- 推送前远端基线：`9bb354b2f21ccb1de1bc1b1d68c1723dfb1b84a9`（`test(xhand): exclude upstream-owned lint checks for the vendor package`）。
- 本轮代码提交：`1563387e010143183caec46ce946858547ec4f03`。
- 主题：`fix(deploy): bind new-host CAN map and XHand serial ports explicitly`。
- 变更统计：8 files changed、268 insertions、7 deletions。
- 范围：
  - `scripts/nero_can_mapping.py`：只读校验新主机左右 USB-CAN 映射；bitrate 必须为 1000000、左右不得同口、bus-info 必须唯一匹配且目标接口未被占用；任一条件不满足即退出，不修改接口。
  - `scripts/can_muti_activate.sh`：改用 `config/nero_can_ports.local.json`，保留旧机控制主体，增加重复目标名检查，激活失败以非零码回传给一键启动脚本。
  - `config/nero_can_ports.local.json`：登记新主机映射，左 `1-2.2:1.0`、右 `1-2.1:1.0`。
  - `scripts/nero_web_start.sh`：过滤 `ip -j link show type can` 中缺少 `ifname` 的行。
  - `scripts/setup_operator_audio.sh`：下载 Vosk 模型时输出 URL、进度和阶段信息；校验和不匹配仍拒绝安装。
  - `scripts/nero_web_start.sh`、`nero_dual_xhand.yaml`：左右 XHand RS485 串口固定为新机 by-id 路径（`FTAEFQWY` 左、`FTAAUYUK` 右）。
  - `test/fixtures/legacy_profiles/index.json`：仅刷新 `nero_dual_xhand` 的 `source_sha256`；schema/layout 哈希未变。
  - `.gitignore`：忽略运行时 `src/common/morla_web_manager/web/browser-api.pid`。

## 提交前实际验证

- `bash -n` 通过：`scripts/can_muti_activate.sh`、`scripts/nero_web_start.sh`、`scripts/setup_operator_audio.sh`；`py_compile` 通过：`scripts/nero_can_mapping.py`。
- `config/nero_can_ports.local.json` 通过 JSON 解析；`nero_can_mapping.py --status` 退出 0，输出配置映射及当前检测到的 CAN 接口（本机为空列表）。
- 本机无 USB-CAN 硬件时执行映射校验（非 `--status`）退出 1，提示左右端口未唯一匹配且未修改任何接口；未执行 `modprobe`、未重命名接口、未设置 bitrate。
- 旧 Profile 兼容测试 `test_legacy_golden_hashes_unchanged` 通过（1 passed），确认 12 个 profile 的 schema/layout golden 与源文件哈希一致。
- 隔离虚拟环境 pure 套件：50 passed、3 failed；3 项均为 `test_coordinates.py` 中依赖 ament 索引定位 `robot_description` 的用例，在该 venv 未加载 ament 索引时失败，与本次改动无关，不计为通过。
- `install_regular` 与 symlink 安装中的 `nero_dual_xhand.yaml` 均已包含上述串口路径，与工作区一致。
- 本次未执行 ROS 运行、实机 CAN 激活、控制器复位、使能或运动指令；192.168.0.84 的硬件验收结论不变，仍以部署记录中的现场核对为准。

## 实际推送与核对

1. 推送前 `git fetch origin main`，`git rev-parse origin/main` 为 `9bb354b2f21ccb1de1bc1b1d68c1723dfb1b84a9`，与本地已知基线一致。
2. `git push origin main` 退出 0：`9bb354b..1563387 main -> main`。origin 使用 SSH（`git@github.com:loopkok/MORLA.git`），本轮未使用 HTTPS 凭据通道。
3. 推送后再次 `git fetch origin main`，`git rev-parse HEAD origin/main` 两行均为 `1563387e010143183caec46ce946858547ec4f03`。

复验命令：

```bash
git fetch origin main
git rev-parse HEAD origin/main
git status --short --branch
```

## 回执提交与边界

本文件与 CHANGELOG、部署记录另作一个文档提交推送；该提交自身 SHA 由 Git 历史记录，避免自引用，其推送结果以实际命令返回和远端 SHA 核对为准。

本轮不表示新主机硬件验收完成：CAN 接口是否 UP、是否处于 BUS-OFF、左右臂与串口对应关系仍需在 192.168.0.84 现场核对。配置中的 bus-info 来自部署记录的现场确认，本次推送前未在本机复验。
