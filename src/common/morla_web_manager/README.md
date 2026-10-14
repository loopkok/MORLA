# morla_web_manager — MORLA统一操作与监控

`morla_web_manager` 是 MORLA 的网页管理包。它以 robot profile 为入口，统一展示系统、启动仿真/真机遥操、管理数采与策略推理泳道、查看动态关节和流率、运行离线数据任务，并通过 `morla_bringup` 的控制协调器执行需要冻结/确认的机器人操作。

## 启动与构建

```bash
cd /home/robot/loopkok/sdk/MORLA/src/common/morla_web_manager/web
npm ci && npm run build

source /opt/ros/humble/setup.bash
source /home/robot/loopkok/sdk/MORLA/install/setup.bash
ros2 launch morla_web_manager web_manager.launch.py
```

浏览器默认访问 `http://127.0.0.1:8080`。局域网访问可用 `web_host:=0.0.0.0`。环境变量前缀统一为 `MORLA_WEB_MANAGER_`，包括 `PORT`、`HOST`、`DIST` 和数据/日志根目录；旧 `ASTRAL_WEB_MONITOR_*` 只作为兼容回退。symlink-install 会从源码 `web/dist` 服务静态文件，普通安装使用 package share 下的 `web/dist`。

## Profile和运行泳道

在“系统”页选择同一 robot profile 与 `sim`/`real` 模式。Profile目录由 `morla_core` 解析，Web列出state/action维度、机械臂/末端、相机和能力，并据此启动对应ROS launch。当前支持Astral与Nero配置；Astral MuJoCo支持Wuji手仿真，Nero/XHand支持MuJoCo仿真。profile选择会被`control_profile_id`/`profile_id`传递给遥操、采集和策略泳道。

“遥操/仿真”“数据采集”“策略推理”是独立启停泳道，可并行监控；启动参数均使用ROS argv，不经shell拼接。Web也会监视CLI启动的节点。profile/mode切换前端和后端同时检查活动launch、collector、policy、control coordinator及ROS command publishers；旧profile数据或writer仍活动时返回409，不会只凭Web自己的PID判断空闲。

模式和控制动作以ROS协调器结果为准。`/morla/control/*`服务成功表示操作已接纳，页面继续根据`/morla/control/state`的`operation_id`显示`accepted/freezing/waiting_services/executing/completed/failed/cancelled`。超时、writer ACK缺失、collector状态未知或服务失败都显示为失败，不把topic publish当作硬件动作成功。仿真不提供电机使能、阻尼、急停等真机能力，界面按profile禁用。

数据采集卡片可独立启动/停止collector，且会传入所选`profile_id`和`runtime_mode`。录制中禁止切换profile或执行回位。策略卡片控制policy/playback/HITL并展示profile/layout hash。监控实体来自当前profile和ROS实际发现的话题；关节名、维度和流率不再固定为Astral布局。摄像头与传感器缺失会按required/optional契约反映在状态和数据质量报告中。

## 主要API

响应统一为`{ok, message, data}`；状态冲突返回HTTP 409。

| 方法 | 路径 | 用途 |
|---|---|---|
| `GET` | `/api/v1/health`, `/api/v1/state` | Web、ROS、launch泳道、profile、动态关节/流率和健康状态 |
| `GET` | `/api/v1/profiles` | Profile能力与state/action维度目录 |
| `POST` | `/api/v1/profile/select` | 选择profile及`sim`/`real`模式；活动writer/泳道阻止切换 |
| `POST` | `/api/v1/profile/launch` | 启动选定profile的遥操/仿真栈 |
| `POST` | `/api/v1/control/{operation}` | 统一控制协调：pause/resume/start/release/ready/enable/position/damping/home/workpos/workpos_direct/estop |
| `POST` | `/api/v1/collect/launch/start`, `/stop` | 数采泳道启停，自动沿用选定profile/mode |
| `POST` | `/api/v1/collect/control` | start/stop/pause/resume/next/discard等录制控制 |
| `POST` | `/api/v1/infer/launch/start`, `/stop`, `/api/v1/infer/cmd` | 策略泳道及policy/playback/HITL控制 |
| `POST` | `/api/v1/data/jobs` | 提交profile感知的对齐、校验、导出等离线任务 |
| `WS` | `/ws/telemetry` | 推送统一UI快照、动态streams与日志尾部 |

OpenAPI文档由FastAPI提供在`/docs`。控制接口与ROS原生topic/service契约详见[`morla_bringup`](../morla_bringup/README.md)和[`morla_policy_inference`](../morla_policy_inference/README.md)。
