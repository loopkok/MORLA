# MMK2 整机 Web 逐步验收

2026-11-09 开始，2026-11-10 完成。实际使用本机浏览器操作 `http://127.0.0.1:8080/`，不是仅调用接口测试。服务和配置保留在 `/tmp/morla-mmk2-web-20261109`；最终工作空间 `mmk2_quest_whole_sim` v2，工作站已停止并释放仿真资源，Web 服务仍可打开。

## 操作过程

1. 从空工作空间进入「系统编排 → 导入 / 打开」，选择「AIRBOT MMK2 双臂 / Quest（仿真）」。页面显示 13 个实例：7 个执行部件、4 个相机、1 个综合传感器、1 个 Quest。
2. 核对 Quest 后端为「测试数据」、侧别为「双侧」，MMK2 全部为 MuJoCo 仿真。沿用左右手位姿→对应机械臂的 IK 映射。
3. 在页面逐条建立额外映射：左/右 scalar→对应夹爪，输出范围 `.04→0 m`；左 scalar→升降，范围 `.1→.2 m`；左 axes→头部两轴，速度 `.1 rad/s`；右 axes→底盘差速轮速，默认 `.08 m/s`、`.3 rad/s`。七条映射均配置按住跟随。
4. 验证连接、保存工作空间，进入「运行前检查」，确认 13 个组件依赖可用，19 维观测、19 维动作、3 路采集图像。点击「保存并进入运行 → 启动工作站」；状态先为「就绪 · 待使能」，不会自行开始运动。
5. 本地测试宿主只为页面已创建会话中的 fake Quest 提供 30 Hz 合成 InputFrame。点击「开始遥操」，公共控制权切换确认七条映射后进入人工遥操；实际回执包含七组组件，页面显示变化的关节反馈与 MuJoCo 画面。
6. 在「数据采集」填写任务名并点击「开始录制 → 结束并保存」。第一段运动学录制完整保存。
7. 根据实际画面修正头部镜头在外壳内的问题；复核双臂基变换，将升降安装旋转纳入 base→slide→arm 链。页面产生的七组映射整理为可安装的 `quest.whole.sim.yaml`，通过新预设「AIRBOT MMK2 整机七组 / Quest（仿真）」重新载入。
8. 在页面「完整运行控制 → 运行设置」选择动力学仿真，重新检查保存为 v2，启动、激活，录制第二段整机数据。页面明确反馈「录制已结束并保存」。
9. 将测试 Grip 置为释放：公共绑定不再生成目标，底盘自身 .25 s 轮速刷新超时归零。页面观察轮角稳定；日志中释放 600 ms 后的最大轮速残差为 `2.67e-6 rad/s`。位置组保持末次目标。
10. 点击「暂停遥操」，状态回到就绪；点击「软件急停」并确认，验证急停锁存；通过「复位故障」返回就绪。停用后在页面从动力学切回运动学成功，没有旧轮速重放。
11. 点击「结束工作站」并确认；页面显示「已停止」，最终日志为 STOPPED、停止操作 SUCCEEDED、清理错误为空。

## 录制核验

实际文件经 `scripts/check_mmk2_web_recordings.py` 只读核验，详细数值见 [recordings-audit.json](evidence/2026-11-09-mmk2-web/recordings-audit.json)。

- 运动学片段约 26.4 秒：每组 342 个状态样本、579 个动作样本，三路相机分别 159/160/159 帧。
- 动力学片段约 46.7 秒：每组 595 个状态样本、987 个动作样本，三路相机分别 364/364/365 帧。
- 两段均为 complete；七组状态均有实际变化，数值有限且时间戳递增，来源明确为 simulation/synthetic、measured=false。轮速动作标为 velocity/rad/s。
- 里程计、陀螺仪、加速度计、180 点激光均有数据，激光保留独立采样时间。三路 JPEG 首尾帧均成功解码为 320×240。
- 最终动力学会话收到 1560 次七组完整命令回执；诊断日志无丢弃、无写盘错误，停止无清理错误。

相机本机实际录制约 6–8 fps，低于配置的 15 fps；四视角 RGB-D 离屏渲染和录制在本机没有达到目标帧率。本次验证功能与数据完整性，不将配置帧率写成实测性能。

原始数据位置：`/tmp/morla-mmk2-web-20261109/recordings/`。运行记录位置：同一 store 的 `logs/`。冻结配置副本见 [saved-system.json](evidence/2026-11-09-mmk2-web/saved-system.json)，原始大体积日志不复制入仓库。

## 此次发现和修复

- 新增底盘差速控制器，公共 BindingEngine 按目标端口生成 velocity/rad/s 命令，同时保留旧设备/测试夹具缺少 ports 时的兼容行为。
- 底盘增加独立轮速超时；公共绑定 Grip 释放只停止生成目标，因此不能仅依赖绑定门控来停止移动底盘。
- 头部相机按 URDF 实际连杆轴向设置，镜头移至外壳前方并向下俯视 .3 rad；场景补足环境照明。
- Quest→双臂基变换包含升降安装旋转。模型 SHA-256 进入设备 geometry，配置更新后重新检查保存，不篡改旧版本。
- 补齐本机 WebSocket 运行依赖，以支持实际浏览器状态流。

## 复现

安装/构建 MMK2 包后，在仓库根目录使用与依赖一致的 Python：

```bash
python3 scripts/run_mmk2_web_acceptance.py --store /tmp/mmk2-web-demo
```

打开 Web，按上面的操作顺序配置；测试宿主不自动载入工作空间、不自动激活、不绕过 Runtime 网关。`quest_fixture.json` 中 `hold=false` 模拟 Grip 释放，`enabled=false` 模拟输入停发。已有服务占用 8080 时，使用 `--port` 指定另一个端口。

核验原始记录：

```bash
python3 scripts/check_mmk2_web_recordings.py --store /tmp/morla-mmk2-web-20261109
MORLA_PYTHON=/path/to/python bash scripts/test_mmk2.sh
```

截图：[运行录制](evidence/2026-11-09-mmk2-web/02-physics-recording.jpg)、[停止状态](evidence/2026-11-09-mmk2-web/03-stopped.jpg)。回归结果见 [regression.log](evidence/2026-11-09-mmk2-web/regression.log)。

这是合成 Quest 输入和参考整机仿真的浏览器验收；未接真实 Quest/机器人，未验证 ROS DDS、轮地接触、碰撞、实机标定和整机速度动作的策略/回放执行。
