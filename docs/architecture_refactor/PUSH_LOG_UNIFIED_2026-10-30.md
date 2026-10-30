# 统一组件重构提交与推送回执

记录时间：2026-10-30T15:39:37+08:00（Asia/Shanghai）。主内容推送成功并重新 fetch 核对后编写；范围见 [详细日志](CHANGE_LOG_UNIFIED_2026-10-30.md)。

## 主内容提交

- 仓库：https://github.com/loopkok/MORLA 。
- 目标分支：main；正常快进，无 force，无历史重写。
- 推送前远端：c1896c6e3114758c04514e9936ded812210a6a25。
- 主内容：480e3d5caead580b873783467662ad18b08eed1c。
- 主题：refactor: 统一组件绑定与设备插件并同步测试文档。
- 范围：115 files changed、2743 insertions、697 deletions。包含代码/插件、测试/样例、dist、README/CHANGELOG、实施与审查/主机文档及证据；不包含厂商二进制wheel、缓存或现场标定。

## 提交前验证

- pure -rs：202 passed、4 skipped，退出0。skip 为 conda 缺 Hypothesis 1项、FastAPI 1项、jodellSdk 2项，逐条见 evidence/unified-20261030/pure.log。
- 独立 Python3.12 fake层：23 passed、1条弃用警告，退出0。
- sim：7 passed，退出0。
- TypeScript/Vite：退出0，61 modules；index-rnwb7kYG.js，282.36kB/gzip96.66kB。
- 静态边界检查、Python/XML解析、fake ROS脚本 --help、git diff --cached --check：通过。fake ROS脚本的DDS测试未执行。
- 同轮此前 Edge/隔离安装/文档检查分别留证；本次未重新执行实机、真实模型或第二 reviewer。

## 实际推送过程

1. 提交前 git fetch origin 成功，HEAD 与 origin/main 均为 c1896c6e3114758c04514e9936ded812210a6a25，没有待合并远端提交。
2. git push origin main 使用 HTTPS 失败，退出128：could not read Username for https://github.com；该次没有推送。
3. 使用本机既有SSH密钥、github.com已知主机公钥、SSH443及已配置本地SOCKS代理重新推送，退出0：`c1896c6..480e3d5 main -> main`。
4. 同一通道 fetch main 到 origin/main，退出0；git rev-parse HEAD origin/main 均为 480e3d5caead580b873783467662ad18b08eed1c，核对时工作区干净。

永久 origin HTTPS URL 未修改，未改全局Git配置，没有保存密钥/令牌/密码。复验命令：

```bash
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=20 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' push git@ssh.github.com:loopkok/MORLA.git main:main
git -c core.sshCommand='ssh -p 443 -o HostKeyAlias=github.com -o BatchMode=yes -o ConnectTimeout=20 -o "ProxyCommand=nc -X 5 -x 127.0.0.1:7897 %h %p"' fetch git@ssh.github.com:loopkok/MORLA.git refs/heads/main:refs/remotes/origin/main
git rev-parse HEAD origin/main
git status --short --branch
```

## 保留的未闭环项

UR11：无 Web ROS Runtime stop 服务只停止执行器，输入在宿主退出时关闭；Web stop 主动关闭输入。新版工作区 ROS/实机矩阵、真实模型/长稳及第二 reviewer 仍未验收。推送成功不代表这些项目通过，历史EPG断链FAIL/豁免继续保留。

本回执随后单独提交并推送；自身 SHA 以Git历史为准，避免自引用。其推送结果以实际执行返回和最终远端核对为准。
