# EPG50-060 独立驱动提交与推送回执

日期：2026-10-29，时区 Asia/Shanghai。仓库 https://github.com/loopkok/MORLA ，分支main。此回执对应EPG夹爪接入，不覆盖同日CR5/VR或之前驱动的历史回执。

## 主内容提交

- 提交前基线：`0eb3110ba8ba9bda6e4d60d7ac95c80815f8f926`。
- 主内容SHA：`c7f2ba2db1f2c51cb7f0d4c6b9c882f7a4c88027`。
- 主题：`feat: 独立接入 Jodell EPG50-060 夹爪 ROS2 驱动`。
- Git提交时间：2026-10-29 15:04:44 +0800。
- 统计：34 files changed、1501 insertions、2 deletions。
- 范围：独立morla_epg_driver组件、SDK包装、ROS网关/launch/probe、fake/real配置、状态与单位契约、生命周期/watchdog/确认停止、仅配置CR5组合、测试、文档、CHANGELOG与来源/证据。
- 未入库原厂SDK、PDF/Word手册、Windows工具、wheel/egg-info/pyc或运行缓存；未改其他机器人算法/控制参数。

细节见 [详细提交日志](CHANGE_LOG_EPG_2026-10-29.md)、[接入验证](epg50_060.md) 和 [驱动说明](../../src/robots/jodell/morla_epg_driver/README.md)。

## 验证依据

EPG专项17项、pure总回归161项、fake集成23项通过，17项已包含在161项中。原厂SDK配录制Modbus master与mock串口测试、独立wheel发现/实际factory/runtime辅助反馈及清理通过。提交前源码和最终wheel哈希与已有通过证据匹配；随后只新增日志/索引并重做静态和暂存空白检查，没有重复运行相同代码的全量回归。

提交代码保持真实模式的现场验证门槛。ROS Humble/真实RS485/完整型号接口固件、寻程/位置反馈语义、物理停止/watchdog/抱闸、夹持力/TCP、碰撞和长稳NOT_RUN。资料中SDK停止地址与手册主寄存器差异、通信丢失后物理行为未知等均保留，没有将软件测试或推送当作实机验收。

## 实际推送与核对

1. 提交前SSH443 fetch成功；HEAD与origin/main均为基线，`HEAD...origin/main`计数0/0。
2. 主内容提交成功，沿用既有Git自动身份，没有修改全局配置或amend历史。
3. 通过既有SSH密钥、SSH443及本机SOCKS代理普通推送，退出0：

```text
To ssh.github.com:loopkok/MORLA.git
   0eb3110..c7f2ba2  main -> main
```

4. 同通道fetch，退出0：

```text
From ssh.github.com:loopkok/MORLA
   0eb3110..c7f2ba2  main       -> origin/main
```

5. 读回后HEAD和origin/main均为 `c7f2ba2db1f2c51cb7f0d4c6b9c882f7a4c88027`；当时工作区干净，状态为 `## main...origin/main`。

没有force、重写历史、关闭主机密钥校验或新增HTTPS失败尝试。永久origin仍为原HTTPS地址，认证凭据未写入仓库。复验命令（代理地址为当前主机已有配置）：

```bash
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=15 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' push git@ssh.github.com:loopkok/MORLA.git main:main
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=15 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' fetch git@ssh.github.com:loopkok/MORLA.git refs/heads/main:refs/remotes/origin/main
git rev-parse HEAD origin/main
git status --short --branch
```

## 回执提交

本文件与CHANGELOG回执链接随后另作文档提交并普通推送。回执自身SHA由Git历史记录，避免自引用；最终推送成功以命令返回和HEAD、origin/main、远端refs/heads/main读回一致为准。
