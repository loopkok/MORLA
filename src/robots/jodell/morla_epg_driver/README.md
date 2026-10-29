# Jodell EPG50-060 独立 ROS 2 驱动

支持目标型号：**EPG50-060-0PF-L200-C7-N-A-P40-S00**。组件 `morla.jodell.epg50_060@1.0.0`，基于用户提供的JodellTool 0.1.5 Python SDK和EPG A.2手册，采用标准RS485 / Modbus RTU。完整后缀未在所给通用资料中解码，不能据此推断定制接口/固件；需核对实物接线。

包不依赖任何机械臂、其他手、IK或Web；不包含厂商SDK、副本手册、Windows工具或二进制库。源码只实现MORLA封装，许可与来源见 PROVENANCE.json。实际SDK需单独安装。

## 安装与构建

在机器人Linux/ROS 2 Humble主机，MORLA根目录，用ROS对应的Python环境：

```bash
source /opt/ros/humble/setup.bash
# 使用用户提供的原厂wheel；pip自动安装pyserial和modbus-tk依赖。
python3 -m pip install '/absolute/path/to/JodellTool-0.1.5-py3-none-any.whl'
colcon build --symlink-install --packages-up-to morla_epg_driver
source install/setup.bash
python3 -c 'from jodellSdk.jodellSdkDemo import EpgClawControl'
python3 -m morla_core.composition.cli catalog --json
```

不要安装同名非原厂包替代，也不要在SDK示例中独立开第二个串口。优先使用 `/dev/serial/by-id/…`，运行用户须有串口权限；RS485 A/B、公共地和24 V供电按手册接线。115200/8N1为手册默认，示例站号9只是示例，不能当作已确认出厂站号。

## fake启动和ROS接口

```bash
python3 -m morla_core.composition.cli compile \
  src/robots/jodell/morla_epg_driver/config/assembly.fake.yaml \
  --output /tmp/epg.resolved.json
ros2 launch morla_epg_driver epg_driver.launch.py resolved_path:=/tmp/epg.resolved.json
```

节点 `/epg_driver` 启动disarmed，不自动激活。该入口只接受一台EPG：

- `~/state`：JSON状态，包含反馈来源、时间、辅助ports与设备状态。
- `~/gripper/joint_states`：`opening_width`，单位m；只在样本变新时发布。
- `~/health`、`~/receipts`：组件健康和命令回执。
- `~/operation`：`morla_interfaces/srv/RuntimeOperation`，prepare/activate/status/estop/reset/stop。
- `~/authority`：`morla_interfaces/srv/ControlAuthority`，acquire/renew/revoke。
- `~/commands`：`morla_interfaces/msg/CommandEnvelope`，必须含当前session、token、epoch、递增sequence、主机单调时间issued_ns和TTL。目标names=`[opening_width]`、values=`[宽度m]`、units=`[m]`、mode=position。

操作顺序：从state取得session_id → prepare → activate并轮询operation结果 → acquire(资源gripper) → 连续发送有效命令/续租 → revoke或stop。ROS服务返回RUNNING只表示操作已派发，不是已使能。不能直接publish普通JointState绕过授权；stop结束会话，estop锁存，reset不会自动恢复运动。

fake只有协议行为，不模拟夹持力或运动动力学。完整fake生命周期/过期目标测试见test/。若要带Web采集，交给现有ComposableService宿主；此单设备launch不启动Web、VR或录制。

## 真实连接与只读检查

```bash
ros2 run morla_epg_driver epg_probe \
  --port /dev/serial/by-id/YOUR_USB_RS485 \
  --device-id YOUR_EPG_ASSET_ID --slave-id YOUR_ID --baudrate 115200
```

probe先获得runtime/设备锁，只读0x07D0起的四个寄存器，不写使能、停止、模式、回零、抱闸、ID或波特率；退出只读会话不发送停止命令。

复制 `config/deployment.real.example.yaml`，填真实资产身份、规范资源key、稳定串口与站号。若设备没有可读序列号，device_id由现场资产标签提供，不能冒充设备自动认证。编译并启动：

```bash
python3 -m morla_core.composition.cli compile \
  src/robots/jodell/morla_epg_driver/config/assembly.fake.yaml \
  --deployment /absolute/path/to/epg.deployment.yaml --output /tmp/epg.real.json
ros2 launch morla_epg_driver epg_driver.launch.py resolved_path:=/tmp/epg.real.json
```

模板默认只读。现场验证完成后才可设置Deployment `verified/stop_verified`及参数 `activation_motion_verified/position_feedback_verified/device_watchdog_verified`。这些声明不由fake测试自动获得。真实activate先只读检查0x03FD，IO模式时拒绝而不自动改模式；随后显式发使能，并等待新读取的gACT=1、gSTA=3、gGTO=0、无故障；**使能可能开合寻程**，不是无动作准备。prepare仅检查状态，不寻程、不改参数模式。

## 数值与协议约定

- 标准EPG50-060：名义总行程50 mm，手册单指夹持力10–60 N；无定制指尖CAD/安装标定，未制作URDF或碰撞模型。
- SDK位置0=打开、255=闭合；`opening_width=0.05*(1-raw/255)`，分辨率约0.196 mm。`closure_ratio=raw/255`，0打开、1闭合。50 mm为本体名义行程，不证明定制指尖绝对开口。
- 默认速度/力设置raw32（范围0–255）。**raw0是最低速度/最低力，不是停止/零力**。力设置只作为设备电流上限相关参数，不提供未经标定的N换算。
- 位置/速度状态描述有歧义：手册称其为当前设定值。默认position输出 `device_reported_unverified_position/measured=false`；验证实测语义后才可显式声明position_feedback_verified。速度始终标记设定回读，不冒充m/s。
- motor_current是电机电流标幺ratio，不是A、N或N·m；voltage是V；temperature按int8摄氏度；status为四个uint16原始寄存器。gOBJ仅在gGTO=1时有效，不能只据目标检测位宣告停止。
- 控制采用SDK `runWithParam`，写0x03E8起的三个寄存器；不调用未确认的预置动作/自动回零。
- 手册主控制寄存器rSTOP是0x03E8 bit2。SDK.stopClaw却写0x03FF，通用手册主表没有该地址；本封装使用0x03E8=0x0005（保留rACT、rSTOP置位、rGTO清零），并轮询状态确认，未实机验证不宣称物理停止已证明。不会强制打开抱闸、自动清故障或发解除急停继续旧动作命令。
- 1秒通信丢失故障有记载，但资料未明确证明掉线后物理停止。device_watchdog_verified保持false直到现场证据齐全；主机watchdog不能替代设备watchdog。

## 频率与故障

I/O目标轮询20 Hz（1–50可配），单线程串行SDK，单槽最新目标；该周期不是已测设备采样频率。串口读写超时0.1 s、反馈新鲜度0.5 s、首次授权目标之后命令watchdog0.3 s；默认每目标相对反馈步长5 mm。ROS网关tick50 Hz，状态样本时间不会因重复发布被刷新。

通信、故障、过期命令导致清空目标并尝试rSTOP，失败保留stop_error；不自动重连或重放旧命令。取消/停止隔离迟到的初始化结果。停止保持使能，不通过去使能或松闸释放工件。软件停止不能替代现场掉落防护或物理急停。

## 验证边界

本轮在无ROS/实体设备的macOS验证Python SDK字节序、状态解析、假设备I/O生命周期、使能取消、停止确认、watchdog及runtime契约。ROS Humble/IDL、真实串口、完整型号固件适配、位置实测语义、抱闸、断网停止、夹持力与长稳NOT_RUN。原厂SDK未提供可再分发许可信息，故外部安装，不入库。

独立源码测试：

```bash
PYTHONPATH=src/common/morla_core:src/common/morla_runtime:src/robots/jodell/morla_epg_driver \
  python3 -m pytest -q src/robots/jodell/morla_epg_driver/test
```

未安装外部SDK时，原厂SDK录制总线测试明确skip，其余mock/fake可运行；不能把跳过记作真实SDK验证。CR5组合仅配置见examples/cr5_epg，不自动假定夹爪安装变换或VR控制方式。
