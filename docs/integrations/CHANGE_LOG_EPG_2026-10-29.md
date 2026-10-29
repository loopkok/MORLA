# EPG50-060 独立驱动详细提交日志

日期：2026-10-29，时区 Asia/Shanghai。仓库 loopkok/MORLA，分支 main，提交前基线 `0eb3110ba8ba9bda6e4d60d7ac95c80815f8f926`。用户授权同步最新更新、记录详细提交/推送日志并提交推送。

本文件记录最终变更与已取得的验证证据；提交SHA和推送读回结果随后另作回执，避免把提交前计划写为已推送。回执自身提交由Git历史标识。

## 设备与资料

完整目标型号：`EPG50-060-0PF-L200-C7-N-A-P40-S00`。依据用户提供的EPG系列A.2手册、Python SDK说明和JodellTool 0.1.5 wheel。通用资料确认EPG50-060族的50 mm名义行程和RS485/Modbus RTU协议；未推断完整后缀的定制接口/固件含义。

原PDF、Word说明和SDK文件留在原位置，路径/哈希保存在包的PROVENANCE.json。原wheel没有明确再分发许可元数据，故未将SDK/手册/Windows工具或二进制库复制入仓库；运行时使用用户原wheel外部安装。MORLA封装源码采用Apache-2.0。

## 最终提交范围

1. 新增独立 `src/robots/jodell/morla_epg_driver`，组件 `morla.jodell.epg50_060@1.0.0`。包含descriptor、ament注册、fake/real选中加载entry point、ROS节点、launch、只读probe、Assembly/Deployment、文档、来源和测试。不依赖机械臂、其他手、IK或Web。
2. 封装原厂EpgClawControl，延迟导入SDK；有界串口读写超时、稳定by-id路径和独占打开，严格检查SDK错误字符串、ACK和四个uint16状态寄存器。未使用SDK硬编码超时的serialOperation入口。
3. canonical目标为opening_width，单位m：0闭合、0.05打开；SDK raw0打开/raw255闭合显式转换。辅助输出闭合比例、速度设定回读、电流标幺、温度、电压、原始状态。默认不将有歧义的位置字段宣称为独立实测，也不将电流标幺换算成N。
4. 单线程I/O、最新目标槽、反馈时效、相对反馈步长、首次授权目标后的host watchdog；激活取消/迟到enable隔离，故障锁存，无自动重连/旧目标重放。
5. prepare/只读probe/从未激活的关闭不写控制寄存器。activate显式检查模式，拒绝IO模式而不自动切换；使能可能开合寻程，descriptor明确moves=true，并等待新读状态确认。
6. 停止使用手册主控制寄存器0x03E8的rSTOP，即0x0005；不采用SDK.stopClaw的0x03FF地址。ACK之后仍读取gACT/gSTA/gGTO与故障确认，不以速度raw0或gOBJ位替代停止证据。不自动松闸、复位、清故障或解除急停继续旧动作。
7. ROS命令复用MORLA CommandEnvelope/ControlAuthority/RuntimeOperation网关，保留授权、epoch、时效、单位及关节检查；JointState输出opening_width（m）。单设备入口拒绝含其他机器人的快照。
8. 新增 `examples/cr5_epg` 仅配置组合，state/action 7维（CR5六关节rad、夹爪一维m），没有联合驱动、安装变换、TCP、URDF或碰撞模型。
9. 更新CHANGELOG、入口README、设备接入指南、分层测试脚本和EPG接入文档；保存原始测试、安装验证脚本和文件/wheel摘要。

没有修改CR5/Nero/Astral现有求解器、控制频率或旧模型；没有修改桌面外部SDK/说明资料，也没有连接或操作实体夹爪。

## 默认参数与真实运动门槛

I/O目标轮询20 Hz，串口超时0.1 s，反馈新鲜度0.5 s，命令watchdog0.3 s，目标相对反馈最大步长5 mm，速度/力raw默认32。raw0表示最低速度/最低力，不是停止或零力。轮询参数不代表已测硬件频率。

real模板保留身份/串口占位与false声明。只有现场验证后才能声明Deployment verified/stop_verified及activation_motion_verified、position_feedback_verified、device_watchdog_verified。资料仅记载1秒通信丢失故障，未证明物理停止；host watchdog不能替代设备watchdog。SDK停止地址差异与位置反馈语义均需在本机固件验证。

## 已执行验证

macOS、无ROS/实体设备；独立Python3.13环境 `/tmp/morla-cr5-vr-tests`。实际安装用户原JodellTool 0.1.5、pyserial3.5、modbus-tk1.1.5，不连接真实串口。

- EPG专项 **17 passed**：含宽度/状态/字节序/故障/温度、SDK错误返回、真实原厂SDK配录制Modbus master、mock串口超时/exclusive/只读打开、IO拒绝、未验证激活门控、迟到使能取消、host watchdog、停止状态确认、通信失败锁存、runtime授权/单位/TTL、CR5组合。
- 最新pure回归 **161 passed**（7.82 s），fake集成 **23 passed**（3.04 s）；EPG17项已包含在161项中，不重复累加。各层一条现有Starlette/AnyIO弃用警告，无跳过。
- 独立构建EPG wheel：静态发现不导入SDK，实际选中factory、米单位runtime目标、辅助反馈、停止/锁清理通过；不导入其他机器人包。公共框架从源码加载、ament查询受控替代，不等于完整colcon安装。
- AST/JSON/YAML/XML、文档链接、无厂商二进制再分发及diff空白检查通过。
- 此次提交前核对已有manifest的32个文件哈希和最终wheel哈希全部一致，因此复用同一源码的已通过测试，没有重复运行不必要的回归。新增提交日志/索引编辑后重新检查静态文档与暂存区；更新本轮manifest覆盖最终内容。

复验命令：

```bash
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer pure
MORLA_PYTHON=/tmp/morla-cr5-vr-tests/bin/python bash scripts/test_composable.sh --layer fake
```

原始输出及安装验证见 [证据目录](evidence/2026-10-29-epg/)，[接入记录](epg50_060.md) 和 [安装操作](../../src/robots/jodell/morla_epg_driver/README.md)。没有外部SDK时，两项真实SDK测试会明确skip，不可计作通过。

## 未执行与已知边界

NOT_RUN：ROS2 Humble colcon/IDL/launch、真实RS485、完整型号接口/固件、初始化行程、物理位置回传、停止/断网/设备watchdog/抱闸、力标定、装配TCP、碰撞和长稳。当前Nero资源清单问题不在本轮修改范围内。

软件测试与推送成功均不等于夹爪实机验收。real默认只读，不为提交或演示将现场验证标志伪填为true。

## Git与推送

提交前fetch核对远端main；在当前main以详细提交正文保存主内容，普通fast-forward推送，不force或重写历史。复用此前SSH443、主机密钥校验与本机已有代理，永久origin保持HTTPS；凭据不入库。主内容成功推送并fetch读回后另提交回执，再推送并核对HEAD、origin/main、远端main一致及工作区干净。
