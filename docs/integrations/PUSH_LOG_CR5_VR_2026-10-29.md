# CR5 本地 IK / VR 遥操提交与推送回执（2026-10-29）

时区 Asia/Shanghai；仓库 https://github.com/loopkok/MORLA ，分支 main。此回执对应本地URDF/Pinocchio IK和右侧VR整链，区别于同日既有独立设备驱动回执 `PUSH_LOG_2026-10-29.md`。

## 主内容提交

- 基线：`c7dbec01d48641cf7c47029116edf130b6e9281b`。
- 提交：`937a66a8ca31a69aa1cfb2f1f514c7e3e112d9f8`。
- 主题：`feat: 接通 CR5 本地 URDF IK 与右侧 VR 遥操链路`。
- Git提交时间：2026-10-29 13:02:40 +0800。
- 统计：97 files changed、165572 insertions、43 deletions；大量新增文本行来自原CR5视觉OBJ，另含二进制碰撞STL。没有加入wheel、egg-info、pyc或缓存。
- 内容：纯CR5模型/网格/许可、本地数值IK、右手柄/裸手来源输入与deadman、异步控制和runtime/驱动链、控制页/launch/配置/只读probe、公共生命周期与补充模型清单、测试和文档。
- 详细范围与测试见 [提交日志](CHANGE_LOG_2026-10-29.md)、[URDF实现记录](CR5_URDF_IK_2026-10-29.md)、[操作指南](cr5_vr.md) 和 [频率参数核对](teleoperation_rates_and_parameters.md)。

## 提交前复验

pure 144 passed、fake 23 passed，每层一条弃用警告。CR5源模型等价、哈希闭包/实际几何加载、独立wheel/share布局加载和静态检查通过。原始测试及提交前文件摘要见 [证据目录](evidence/2026-10-29-cr5-commit/)。

保留既有Nero URDF全库资产哈希失败；没有声称全库资产检查通过。ROS Humble/Quest/CR5实体、方向/零位/TCP标定、停止/设备watchdog/断网、碰撞与长稳NOT_RUN。推送不等于实机部署。

## 实际推送与读回

1. 提交前通过既有SSH443通道fetch，退出0；当时HEAD与origin/main均为基线，确认没有远端分叉。
2. 主内容提交成功。Git提示身份来自现有用户名/主机自动配置，沿用现有身份；没有修改全局Git身份或amend重写历史。
3. 使用既有SSH密钥及本机SOCKS代理普通推送，退出0：

```text
To ssh.github.com:loopkok/MORLA.git
   c7dbec0..937a66a  main -> main
```

4. 通过同一通道fetch，退出0：

```text
From ssh.github.com:loopkok/MORLA
   c7dbec0..937a66a  main       -> origin/main
```

5. 主内容读回后，HEAD与origin/main均为 `937a66a8ca31a69aa1cfb2f1f514c7e3e112d9f8`；工作区当时干净，状态 `## main...origin/main`。

本轮直接复用此前已验证的SSH通道，没有新增HTTPS失败尝试。永久origin HTTPS URL保持原样；没有force、关闭主机密钥校验或写入认证凭据。

复验命令：

```bash
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=15 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' push git@ssh.github.com:loopkok/MORLA.git main:main
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=15 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' fetch git@ssh.github.com:loopkok/MORLA.git refs/heads/main:refs/remotes/origin/main
git rev-parse HEAD origin/main
git status --short --branch
```

## 回执提交

本文件与CHANGELOG回执链接随后另作文档提交并普通推送。回执自身SHA由Git历史标识，避免文档自引用；最终成功以该次推送返回以及HEAD、origin/main、远端refs/heads/main读回一致为准。
