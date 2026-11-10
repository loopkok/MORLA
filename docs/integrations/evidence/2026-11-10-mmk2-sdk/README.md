# MMK2 SDK 本地验证证据

所有网络协议测试连接本机 127.0.0.1 的测试 gRPC 服务；浏览器使用 SDK fake 与 Quest 合成输入。没有连接 172.25.14.1，没有 ROS/DDS 或实机验收。

- `regression.log`：驱动与公共 core/runtime/teleop、FR3/CR5 联合回归，309 passed、1 skipped。
- `web-regression.log`：公共 Web、录制、策略相关 37 passed。
- `simulation-regression.log`：现有 MMK2 MuJoCo 21 passed。
- `installed.json`：独立安装包的入口、共享连接、19 维状态/动作与停止检查。
- `web-audit.json`：两段完整录制的逐流样本/变化量、JPEG 检查与各会话的拒绝/停止记录；失败过程也保留。
- `web-recording.png`、`web-stopped.png`：真实浏览器测试画面（灰色相机为 16×16 合成图像）。

全量原始文件在本机 `/tmp/morla-mmk2-sdk-web-20261110`。审计脚本为 `scripts/check_mmk2_sdk_recordings.py`。具体边界与已知录制收尾现象见 MMK2_SDK_DRIVER.md；不要将最后 STOPPED 等同于整个过程中从未发生故障。
