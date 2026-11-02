# 从空白保存库开始的 Web 遥操操作手册

核对日期：2026-11-02；对应提交：`27f2bbc`。目标：真实 Vader/Quest 输入控制 MuJoCo 内的 Nero/CR5，在网页画面和本地 MuJoCo viewer 间切换。这里的 Nero 双手按 XHand 解释。

不需要用户预先编写 YAML/JSON。Web 内置预设由软件提供，载入后用表单修改、保存；第一次没有“已保存的工作站”也能使用。页面没有六种场景各一个的一键按钮，需要按下面修改预设。

## 使用主机与一次性准备

实体 Vader 的现有 SDK 使用 Linux hidraw。以下实体输入流程按有图形桌面的 Ubuntu/ROS 2 Humble 主机说明。当前开发机器 `/Users/a1234/Desktop/MORLA` 是 macOS，不能原样运行 `/opt/ros/humble` 和 Linux Vader SDK；不能把 macOS 仿真测试当作六种实体输入链路已在此机器可用。

Web 和本地 viewer 都运行在启动服务的主机上。远程浏览器选择本地窗口时，窗口仍在服务主机桌面，浏览器电脑不会自动弹窗。

假定主机已经安装 ROS 2 Humble、colcon 和 MORLA 基础依赖。将下面仓库路径换成实际路径；不是要创建这个路径：

```bash
cd /实际路径/MORLA
source /opt/ros/humble/setup.bash
python3 -m pip install --user ./src/common/vader5_sdk
colcon build --symlink-install --packages-up-to \
  morla_web_manager morla_vader5_input morla_input \
  morla_cr5_driver cr5_mujoco_sim xnero_mujoco_sim \
  morla_native_bridge nero_arm_teleop xhand_retargeting
source install/setup.bash
```

检查当前 ROS 使用的 Python 环境，必须能导入 Web、模型、IK 和手势重定向依赖：

```bash
python3 -c 'import rclpy, mujoco, PIL, pinocchio, numpy, scipy, fastapi, uvicorn, yaml, jsonschema, dex_retargeting; print("依赖导入成功")'
```

若缺少 pip 依赖，在同一个 ROS Python 环境补齐后重新检查；Pinocchio 的 pip 发行名是 `pin`，导入名是 `pinocchio`：

```bash
python3 -m pip install --user mujoco Pillow pin dex-retargeting fastapi uvicorn PyYAML jsonschema
```

`rclpy` 来自 ROS，不能靠上面的 pip 命令替代 ROS 安装。已有环境导入成功时无需重新安装。XHand 还要求构建后的 `xhand_retargeting` 和 `robot_description` 资源可发现。

仓库已包含 web/dist，正常使用不必每次 npm 构建。修改过前端源码才在 `src/common/morla_web_manager/web` 执行 `npm ci`、`npm run build`，然后返回仓库根目录重新构建安装。

## 启动一个不依赖装配文件的 Web

新终端，仍在上述 Linux 主机：

```bash
cd /实际路径/MORLA
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
export MORLA_WEB_MANAGER_HOST=127.0.0.1
export MORLA_WEB_MANAGER_PORT=8080
ros2 run morla_web_manager morla-composable-web
```

保持终端运行，浏览器打开 `http://127.0.0.1:8080`，选择“工作站”。这是公共 Web 入口，无需 `--assembly`，也不提前启动任何机器人。不要同时再启动另外一个仿真整链或占用 8080 的 Web。

已有完整 Linux 安装也可在仓库根目录执行 `bash tools/start_morla_web.sh`，它会加载 ROS/工作区环境、打开完整控制台并自动尝试打开浏览器；与上面的启动方式二选一。脚本提示已更新为当前工作站流程，不需要配置文件。`OPEN_BROWSER=0 bash tools/start_morla_web.sh` 仅启动服务；`PORT=9090 bash tools/start_morla_web.sh` 使用其他端口。检测到旧服务时只复用，不会自动重启更新它。

## 每个场景共用的网页流程

设备选型 → 安装与标定 → 遥操作设置 → 数据记录 → 检查与运行。

1. 顶部“仿真预设”选择机器人，点击“载入预设”。填写独立配置名称，例如本手册的六个名称，避免覆盖另一个场景。
2. 真实 Vader/Quest 的“运行方式”选“真实设备”；机械臂、XHand 和场景相机均选“MuJoCo 仿真”。输入是真设备不等于机械臂是真设备。
3. 在“安装与标定 → 硬件连接”为输入填写“设备标识”，例如 `vader_usb_bench` 或 `quest3_usb_bench`。Vader 的“设备路径参数 device”另填 `/dev/hidrawN`；设备标识不是设备路径。
4. 按对应场景调整控制映射；保留预设相机及机器人安装关系。初次不采集也可以直接保留“数据记录”默认设置。
5. “检查与运行 → 仿真模式”选“运动学仿真 · 验证 IK 与映射”。需要动力学时再切换，不必为两种显示方式切换物理模式。
6. “仿真显示”先选“浏览器画面”。点击“检查配置 → 检查运行依赖 → 保存配置 → 启动工作站”。检查失败时看详情补齐对应项；修改后重新检查、保存。真实输入确认框是因为要连接手柄/头显。
7. 点击“连接输入”，确认输入已连接且新鲜、执行器反馈就绪；点击“准备设备”。手柄/手保持中立或舒适初始姿态，再用对应启动操作进入遥操。等待零点确认结束，不要在切换确认阶段移动输入。
8. 先在网页观察；需要本地 MuJoCo 时，点“暂停遥操”，将“仿真显示”改为“本机 MuJoCo 窗口”或“浏览器与本机窗口”，再开始遥操。这里只切换显示，本来不要求停止当前人工遥操；暂停是为了方便观察切换。
9. 网页左键拖动旋转、右键/Shift 拖动平移、滚轮缩放，可“复位视角”。本地 viewer 可调整相机，窗口内的模型扰动不回写公共仿真。关闭 viewer 不停止会话。
10. 换下一个机器人/输入场景时，“暂停遥操 → 停止工作站”，再载入另一预设。运行中配置只读。无需重启 Web，也无需单独启动另一份 MuJoCo。

有策略/录制时先停止策略/录制再改显示。物理模式需在非 ACTIVE 且无策略/录制时更改。运行时改显示/模式只影响本次会话；要保留默认值，停止后修改并保存配置。

## Vader 的 USB 准备

手柄使用已验证的 USB 有线 PC 模式，关闭其他读取厂商数据流的程序。先确定实际 hidraw 节点：

```bash
for device in /dev/hidraw*; do
  printf '%s\n' "$device"
  udevadm info --query=property --name="$device" | rg 'ID_VENDOR_ID|ID_MODEL_ID|ID_MODEL='
done
```

如未安装 ripgrep，可用 `grep -E` 替换 `rg`。飞智厂商 ID 是 `37d7`；同厂商多个接口仍需通过 SDK 读取验证，不能随便挑一个节点。

权限不足时按 SDK 文档配置本机活动桌面用户访问，再重新插拔：

```bash
sudo tee /etc/udev/rules.d/70-vader5-sdk.rules <<'EOF'
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="37d7", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
python3 -m vader5_sdk --device /dev/hidrawN
```

最后一行是可选读取检查；看到摇杆/按键报告后 Ctrl+C 退出，释放设备。SSH 服务账号的权限需管理员另行配置，uaccess 不自动授权所有账号。

Web 添加 Vader 后，输入进程 `process_mode` 选 `managed` 或 `auto`，运行参数 `device` 填实际节点。点击“连接输入”会启动 reader，无需另开 `vader5_input.launch.py`。若确实在外部运行了 reader，页面改选 `external`，同一设备不要起两个 reader。

## 场景 A：Vader → Nero 双臂

- 载入“Nero 双臂 / Quest”，配置名称改为 `vader_nero_dual_sim`。
- “设备选型”删除 `quest`，对应旧映射和辅助记录会一并移除；添加“Vader 5 手柄”，默认实例名 `pad`。
- `pad` 设真实设备、managed/auto；填写上述身份与 `device`。`left_arm`、`right_arm`、`scene_camera` 保持 sim，保留预设初始关节位置和安装关系。
- “遥操作设置”添加两条映射：`pad.axes → left_arm.position_target` 和 `pad.axes → right_arm.position_target`。
- 两条都手动选择“末端速度控制”（`morla.gamepad_cartesian_jog`）。Nero 的 axes 输入可能自动建议“关节点动”，必须改成这里的笛卡尔速度控制。
- 每条“摇杆轴分配”：X←x、Y←y、Z←z、绕 X←roll、绕 Y←pitch、绕 Z←yaw。不要把摇杆原始轴 `raw_axes` 当作这组命名六轴。
- 每条“运动使能信号”选 `pad.hold_to_control`。线速度设 `0.02 m/s`，角速度设 `0.15 rad/s`；初次使用默认单位方向和零 TCP 偏置，并逐轴确认两臂的局部基座方向。不要把 Quest 的 Unity 输入变换照抄给 Vader。
- 按共用流程启动：松开 M3 后再按下 M3 开始；保持 M3 并推摇杆运动。松开 M3 停止这两条映射的运动增量，全局会话仍可保持激活；Web“暂停遥操”执行全局暂停。
- 不按 M2 为平移：左摇杆 Y→x、左摇杆 X→y、右摇杆 Y→z。按住 M2 为旋转：左摇杆 X→roll、左摇杆 Y→pitch、右摇杆 X→yaw。LT 是精细倍率。
- 按共用显示流程分别观察网页和 MuJoCo viewer。

一只 Vader 的这两条映射共享同一组六轴，两臂会同时响应，因各自基座不同可能表现为镜像方向；这不是两套独立六维输入。当前没有内置左右臂切换按钮。若想先分别验证单臂，可停止后仅保留一条映射另存配置，再对另一侧重复；同时独立控制双臂需要另行设计输入分配/切换策略。

## 场景 B：Vader → CR5

- 停止 A，载入“CR5 / Quest”，命名 `vader_cr5_sim`。
- 删除 `quest`，添加真实 Vader `pad`，填写身份、device 与进程模式；保留 sim 的 `arm` 和 `scene_camera`。
- 添加 `pad.axes → arm.position_target`，控制方式“末端速度控制 · CR5”（`morla.cr5.cartesian_jog`）。
- “运动使能信号”选 `pad.hold_to_control`；六轴分配与 A 相同。IK 选连续微分，线速度 `0.02 m/s`、角速度 `0.15 rad/s`、关节速度上限 `0.3 rad/s`。输入方向默认单位对应，TCP 偏置零。
- 按共用流程启动，M3 和 M2 的操作同 A，再分别选择网页和本机窗口观察。

## Quest 的连接准备

使用与仓库接收协议匹配的 astral-tracking/HTS 应用；旧的无新包头 APK 不能默认互通。USB 连接并允许调试，在另一个同主机终端执行：

```bash
adb devices
adb reverse tcp:8000 tcp:8000
```

设备应为 `device`，不是 `unauthorized`。重新插拔或重启 ADB 后可能需要重新执行 reverse。

头显应用设置 TCP、`localhost:8000`、开启头部跟踪及当前场景需要的手柄/手追踪发送。Web 中 Quest 设真实设备、`process_mode=managed`、`protocol=tcp_wired`、TCP 端口 8000，并填写设备标识。点击“连接输入”后先确认接收器已启动，再连接/重连头显应用推流。

Web 托管接收器已传入 `publish_teleop_events=true` 和 `controller_as_wrist=true`，不需要另开接收器或手动改 Unity→机器人轴转换开关。输入的 `side=both` 表示双侧接收，不代表“手柄模式”；手柄/裸手来自头显当前实际发送的来源，不是在 Web 选 left/right/both 来切换。

## 场景 C：Quest 双手柄 → Nero 双臂

- 停止 B，载入“Nero 双臂 / Quest”，命名 `quest_controller_nero_dual_sim`。
- `quest` 按上述设 real、both、managed、tcp_wired，填写硬件身份；机械臂与相机仍 sim。
- 保留预设两条映射：`quest.left_pose → left_arm.position_target`、`quest.right_pose → right_arm.position_target`，控制器均为“末端位姿跟随”（`morla.cartesian_delta_ik`）。保留预设左右不同的输入标定和 TCP。
- 保持预设无按住使能（页面显示连续跟随选项）。两只手柄先放在舒适位置、右 grip 松开一次；准备后按右 grip 启动并记录双侧零点。移动左手柄控制左臂、右手柄控制右臂，位置和姿态均跟随。
- 在当前连续跟随配置下，松开 grip 不会暂停跟随；再次按右 grip 会重新记录跟随零点。暂停使用 Web“暂停遥操”。
- 若自行为映射选择 `left_hold/right_hold`，则改为按住模式；公共 operator hold 可使能其他映射，不能据此宣称左右 grip 是完全独立的离合。此手册使用预设连续跟随，避免混淆。
- 按共用显示流程观察两种可视化。

## 场景 D：Quest 裸双手 → Nero 双臂 + 双 XHand

- 停止 C，重新载入“Nero 双臂 / Quest”，命名 `quest_hands_nero_xhand_sim`；先完成与 C 相同的真实 Quest 连接设置。
- “机器人与末端执行器”添加两个“XHand 灵巧手”，通常实例 ID 是 `hand`、`hand_2`。显示名称分别填左 XHand、右 XHand；两者选 MuJoCo 仿真，设备参数 side 分别 left、right。按实际生成的 ID 操作，显示名称不会改 ID。
- “安装与标定”：将 `hand.base` 的父坐标系改为 `left_arm.flange`；`hand_2.base` 的父坐标系改为 `right_arm.flange`。两处安装 XYZ/RPY 都为 0，复用模型内已有安装几何，不能继续把手放在 world 原点。
- 保留双臂位姿映射，额外添加 `quest.left_landmarks → hand.position_target` 和 `quest.right_landmarks → hand_2.position_target`，均选“XHand 手势重定向”（`morla.xhand.retarget`）。也可选择同义的 joint_target，但各执行器只保留一条映射。
- 手部预处理保持既有默认 `legacy_xhand_pinky`；不需要给手部填机械臂的 TCP。右手 index_bend_sign 默认 -1，左手默认 +1；未明确配置时控制器会按 side 取这些默认值。
- 手部及双臂先保持连续跟随、不选 pinch/hold 使能，避免张开手指时意外停掉整条手部重定向。依赖检查需通过 dex_retargeting 与手模型资源；只有手能显示，不代表手势算法已就绪。
- 检查摘要应为 **38 维观测、38 维动作**（14 臂 + 24 手），四条映射。启动后将 Quest 切为裸手追踪，确认双侧位姿、21 点关键点持续新鲜。
- 裸手没有手柄 grip，使用 Web“开始遥操”，或勾选“启用本页键盘控制”后在页面有焦点且不编辑输入框时按 S。先等待双臂和双手映射确认，再移动手腕/弯曲手指。
- 腕部位姿控制双臂，关键点控制双 XHand；捏合默认不自动发出“开始遥操”。暂停后保持舒适手势，重新开始可重记零点。分别切换网页与本地 viewer。

## 场景 E：Quest 右手柄 → CR5

- 停止 D，载入“CR5 / Quest”，命名 `quest_controller_cr5_sim`。
- Quest real、side=right、managed、tcp_wired，填写身份；CR5 与场景相机保留 sim。
- 保留 `quest.right_pose → arm.position_target`、“末端位姿跟随 · CR5”（`morla.cr5.cartesian_delta`）、连续微分及模板输入方向/TCP，运动使能保持连续跟随。
- 启动、连接、准备，头显切手柄追踪。右 grip 先松开，再按下开始和记录零点；松开后仍连续跟随，Web 可暂停。右 grip 再按下重新记录零点。
- 先用右手柄小幅移动和转腕，分别切换网页和 MuJoCo viewer。

## 场景 F：Quest 裸右手 → CR5

- 停止 E，重新载入“CR5 / Quest”，命名 `quest_hand_cr5_sim`，Quest 仍 real/right/managed/tcp_wired。
- 映射仍为 `quest.right_pose → arm.position_target`，控制方式和 CR5 标定不变。CR5 无灵巧手，本场景不添加 XHand，也不需要 landmarks→手部关节映射。
- 头显切为裸手追踪，确认右腕位姿持续新鲜。启动、连接、准备后，右手处于舒适零点姿态，点击 Web“开始遥操”或启用键盘后按 S。
- 保持连续跟随：右手腕位移/旋转控制 CR5，手指弯曲不控制机械臂关节；捏合不自动启动全局遥操。
- 如果希望“捏住才动”，停止后将该映射的“运动使能信号”改成 `quest.right_hold`，重新检查保存启动。全局仍需 Web/S 开始，再松开/捏合建立跟随；松开捏合停止该映射并丢弃锚点，重新捏合按当前姿态重建。不要与连续跟随操作混用。
- 分别选择网页与本地窗口观察。

## 结束、再次启动与诊断

完成一个场景后先暂停，再停止工作站；录制中先结束保存或明确丢弃。可以在同一个 Web 服务中依次运行六个配置，不能同时启动六个占用资源的会话。

下次只需启动 Web，从“已保存的工作站”选择对应名称，检查设置后启动并连接输入。不需要自己管理配置文件。关闭服务前先停止工作站，再在终端 Ctrl+C。

公共诊断默认写入 Web store 的 logs 目录，不依赖数据采集；页面可以查看历史和下载。没有画面先看 scene_camera 是否 sim、显示是否误选 local、渲染错误；输入不动先看 connected/fresh、身份/设备路径、Quest 头显来源、切换确认错误和是否仍暂停。不能靠持续点击“开始遥操”修复缺失输入。

## 本手册核对范围

已对六种等价配置进行内存编译和 MuJoCo 模型加载检查：Vader/Nero、Quest 手柄/Nero 各 14 维；Quest 裸手/Nero+双 XHand 为 38 维；三种 CR5 场景各 6 维。没有产生必须由用户提前准备的配置文件，也未启动实际输入、机器人或完整 XHand 优化器验收。物理控制响应和输入方向仍需在目标 Linux 主机逐项确认。
