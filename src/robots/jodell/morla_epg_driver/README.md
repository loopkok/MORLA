# Jodell EPG50-060 独立 ROS 2 驱动

> 2026-10-30 统一路径更新：EPG 是单自由度米制开口执行器；VR/Vader/mobile/标准源通过 scalar_map 或显式滤波控制器绑定 position_target。不会因新增上游输入而增加 EPG 专用遥操节点。历史断链 FAIL/操作者豁免不因公共框架测试改变。 当前接口、安装与测试见[设备指南](../../../../docs/device_integration_guide.md)和[实现记录](../../../../docs/architecture_refactor/UNIFIED_IMPLEMENTATION.md)。下文独立驱动入口仍可单用；不能与另一个会话同时占用设备。

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
- `~/commands`：`morla_interfaces/msg/CommandEnvelope`，字段仍保留session_id、producer、token、epoch、sequence、issued_ns、ttl_ns。**当前runtime不校验授权、序号或时效**，只检查急停锁存、组件是否属于本会话、以及适配器是否接受。目标names=`[opening_width]`、values=`[宽度m]`、units=`[m]`、mode=position。

操作顺序：从state取得session_id → prepare → activate并轮询operation结果 → acquire(资源gripper，仅登记租约) → 连续发送命令 → revoke或stop。ROS服务返回RUNNING只表示操作已派发，不是已使能。节点不订阅普通JointState，命令必须走CommandEnvelope；stop结束会话，estop锁存，reset只清锁存、不会恢复运动。

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

模板默认只读。Deployment `verified/stop_verified` 及参数 `activation_motion_verified/position_feedback_verified/device_watchdog_verified` **现在只是记录字段，不参与编译或运行门禁**；填 true 不会放开任何检查，填 false 也不会阻止activate。真实activate先只读检查0x03FD，IO模式时拒绝而不自动改模式；随后显式发使能，并等待新读取的gACT=1、gSTA=3、gGTO=0、无故障；**使能可能开合寻程**，不是无动作准备。prepare仅检查状态，不寻程、不改参数模式。

## 真实台架检查脚本

`scripts/epg_real_check.py` 通过已安装的ROS服务/话题驱动一个real EPG，不直接调用SDK；必须先 `source install/setup.bash`（插件注册来自安装树）。

```bash
# 只读：核对节点使用的快照、打印状态/宽度、执行prepare，不使能、不写寄存器
python3 src/robots/jodell/morla_epg_driver/scripts/epg_real_check.py \
  --resolved /tmp/epg.real.json

# 运动：需显式确认与目标宽度；目标与当前反馈之差不得超过5 mm
python3 src/robots/jodell/morla_epg_driver/scripts/epg_real_check.py \
  --resolved /tmp/epg.real.json --confirm-motion --target-width 0.045
```

脚本不再核对 `deployment_verified`、`watchdog_verified` 或任何验证声明——这些字段只作记录。只有 `--confirm-motion` 才会 activate/acquire/发命令。命令以10 Hz流式发送只是测试节奏，**适配器已没有主机命令watchdog**；结束后执行`stop`，异常时尝试`stop`再撤权，但脚本被`SIGKILL`或节点崩溃时夹爪会保持最后目标。`--target-width 0`为闭合、`0.05`为全开；没有5 mm步长限制，单次给定全行程目标即可。

首次使能本身可能产生寻程动作：确认夹爪空载、行程内无工件、24 V电源开关或物理急停在手边。mcap录制非本脚本职责。

## 数值与协议约定

- 标准EPG50-060：名义总行程50 mm，手册单指夹持力10–60 N；无定制指尖CAD/安装标定，未制作URDF或碰撞模型。
- SDK位置0=打开、255=闭合；`opening_width=0.05*(1-raw/255)`，分辨率约0.196 mm。`closure_ratio=raw/255`，0打开、1闭合。50 mm为本体名义行程，不证明定制指尖绝对开口。
- 默认速度/力设置raw32（范围0–255）。**raw0是最低速度/最低力，不是停止/零力**。力设置只作为设备电流上限相关参数，不提供未经标定的N换算。
- 位置/速度状态描述有歧义：手册称其为当前设定值。默认position输出 `device_reported_unverified_position/measured=false`；`position_feedback_verified` 等声明字段仅作记录，驱动不再据此拒绝整定或运动。未使能时位置寄存器是默认值（物理闭合时实测见过回读0和254），驱动要求 `enabled && gSTA=3` 才发布有效宽度，否则端口 `valid=false`、顶层 `values=[]`；避免把默认值显示成 0.05 m 的开口。速度始终标记设定回读，不冒充m/s。
- motor_current是电机电流标幺ratio，不是A、N或N·m；voltage是V；temperature按int8摄氏度；status为四个uint16原始寄存器。gOBJ仅在gGTO=1时有效，不能只据目标检测位宣告停止。
- 控制采用SDK `runWithParam`，写0x03E8起的三个寄存器；不调用未确认的预置动作/自动回零。
- 手册主控制寄存器rSTOP是0x03E8 bit2。SDK.stopClaw却写0x03FF，通用手册主表没有该地址；本封装使用0x03E8=0x0005（保留rACT、rSTOP置位、rGTO清零），并轮询状态确认。2026-10-30 台架实测该写入能停住运动并保持在停止位置、无故障；软件停止仍不替代物理急停或断电防护。不会强制打开抱闸、自动清故障或发解除急停继续旧动作命令。
- 1秒通信丢失故障有记载，但资料未明确证明掉线后物理停止。台架实测断链 **FAIL**：切断Modbus 2 s期间夹爪继续运动，只有重连后的stop才停住。包默认 `device_watchdog_verified=false`；本地台架按操作者豁免置true并记录“实测FAIL”，该字段不参与运行门禁，也不代表watchdog已验证。主机没有命令watchdog，不能替代设备侧行为。

## 频率与故障

I/O目标轮询20 Hz（1–50可配），单线程串行SDK，单槽最新目标；该周期不是已测设备采样频率。串口读写超时0.1 s、反馈新鲜度0.5 s。descriptor里仍保留 `max_step_m`、`command_timeout_s` 字段以兼容旧配置，但适配器**不执行目标步长限制，也没有主机命令超时**（`command_timeout_s`只在configure中做范围检查）。ROS网关tick50 Hz，状态样本时间不会因重复发布被刷新。

通信错误或适配器故障会清空目标并尝试rSTOP，失败保留stop_error；不自动重连或重放旧命令。主机不再判定“过期命令”。取消/停止隔离迟到的初始化结果。停止保持使能，不通过去使能或松闸释放工件。软件停止不能替代现场掉落防护或物理急停。

## 验证边界

早期在无ROS/实体设备时验证了Python SDK字节序、状态解析、假设备I/O生命周期、使能取消与停止确认。2026-10-30 在 ROS 2 Humble + 实物 EPG 上完成：只读检查、0↔0.05 m 全行程、5轮重复性（最大误差0.392 mm）、5分钟/150次往返短时连续运行（温度31→32 °C）、以及统一管理层（ComposableService）只读 prepare/estop/reset/stop。断网停止仍为 **FAIL（操作者豁免）**；夹持力/N换算、小时级长稳、抱闸与位置计量学标定 NOT_RUN。原厂SDK未提供可再分发许可信息，故外部安装，不入库。

独立源码测试：

```bash
PYTHONPATH=src/common/morla_core:src/common/morla_runtime:src/robots/jodell/morla_epg_driver \
  python3 -m pytest -q src/robots/jodell/morla_epg_driver/test
```

未安装外部SDK时，原厂SDK录制总线测试明确skip，其余mock/fake可运行；不能把跳过记作真实SDK验证。CR5组合仅配置见examples/cr5_epg，不自动假定夹爪安装变换或VR控制方式。
