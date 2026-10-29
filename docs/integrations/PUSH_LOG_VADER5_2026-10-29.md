# Vader5接入、SDK目录与主机验收交接提交推送回执

日期：2026-10-29，时区Asia/Shanghai。仓库 https://github.com/loopkok/MORLA ，分支main。本回执覆盖Vader5输入接入、同级SDK布局及本轮新增主机测试手册，不覆盖同日EPG/CR5此前提交的历史回执。

## 主内容提交

- 提交前基线：`5154808ad3110aaa9a807bfaec98296c28dbc4c9`。
- 主内容SHA：`90fcddafead1095b2db2ce80724fc2ed1346e819`。
- 主题：`feat: 接入 Vader5 输入并补齐主机编译与真机验收手册`。
- Git提交时间：2026-10-29 17:50:43 +0800。
- 统计：72 files changed、4778 insertions、74 deletions。
- 使用此前现有Git自动身份，未修改全局配置、amend或重写历史。

最终范围见[详细提交日志](CHANGE_LOG_VADER5_2026-10-29.md)。包括独立morla_vader5_input、同级src/common/vader5_sdk、ROS输入插件/公共宿主、按钮操作去重与来源时效、原控制器/IK/数据策略契约复用、CR5 jog、配置示例和文本证据。SDK运行时代码与GPL来源/NOTICE保持。

新增[主机编译与真机手册](../testing/host_build_and_hardware_acceptance.md)、[报告模板](../testing/host_acceptance_report_template.md)和[仅fake的ROS网关检查](../testing/fake_ros_gateway_smoke.py)，覆盖编译/普通安装/IDL/DDS、设备迁移/遥操、其他输入/相机、数据/真实模型、Web/性能长稳及回退。

未入库构建目录、wheel、venv、缓存、原厂EPG二进制或硬件数据。其余Nero/Astral/LoopHand驱动、Nero模型与主资源清单本轮未修改；没有将它们标成已完成SDK同级目录迁移。

## 验证依据与限制

- pure172（含输入专项11）、fake23、SDK5已有原始日志；SDK迁移后输入11、SDK5再次通过。
- 代码哈希与已通过证据匹配，后续仅增加测试交接文档及主机检查工具，没有重复运行同一runtime代码的全套回归。
- 提交前最终manifest覆盖71个源/配置/证据文件，自身除外；静态解析31个Python、4个JSON、6个YAML、1个XML、1个TOML及11份Markdown的相对链接通过。
- 主机手册27个Bash命令块和嵌入Python语法通过；新fake检查入口仅验证AST、--help和快照门禁，实际DDS仍NOT_RUN。
- 暂存区git diff --check通过，已知Nero哈希、LoopHand真实运动禁止、Astral夹爪回显等限制均保留。

没有执行Linux Humble完整构建/IDL/真实DDS、Quest/Vader实物、实体机器人/夹爪/相机、物理停止/watchdog、真实训练推理或8小时长稳。手册是主机待执行交接，不是已经完成这些项目的测试报告。

本回执提交只更新文档索引/实际结果及清单哈希；manifest随之增加回执并刷新文档摘要，运行时代码和通过测试的证据保持不变。

## 实际推送与读回

1. 主内容提交前两次SSH443 fetch退出0，HEAD与origin/main均为基线，左右计数0/0。
2. 主内容commit成功后，普通fast-forward push退出0：

```text
To ssh.github.com:loopkok/MORLA.git
   5154808..90fcdda  main -> main
```

3. 同通道fetch退出0：

```text
From ssh.github.com:loopkok/MORLA
   5154808..90fcdda  main       -> origin/main
```

4. 当时HEAD和origin/main均为`90fcddafead1095b2db2ce80724fc2ed1346e819`，工作区干净，状态`## main...origin/main`。

使用现有SSH密钥、SSH443和本机已有SOCKS代理；永久origin保留HTTPS。没有force、关闭主机密钥校验或新增HTTPS认证失败尝试；凭据未写入仓库。

复验命令：

```bash
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=15 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' push git@ssh.github.com:loopkok/MORLA.git main:main
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=15 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' fetch git@ssh.github.com:loopkok/MORLA.git refs/heads/main:refs/remotes/origin/main
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=15 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' ls-remote git@ssh.github.com:loopkok/MORLA.git refs/heads/main
git rev-parse HEAD origin/main
git status --short --branch
```

## 回执自身提交

本回执、CHANGELOG链接和manifest刷新随后另作文档提交并普通推送。回执自身SHA由Git历史识别，避免自引用；最终结果以第二次push返回及HEAD、origin/main、远端refs/heads/main实际读回一致为准。
