# 组合系统本地操作

实现范围与未验收项见 progress.md，扩展组件见 SDK.md，运行时接口见 ../../src/common/morla_runtime/README.md。

## 环境与验证

在独立 Python 环境安装 dependencies/composable-test.txt，再从仓库根目录运行：

```bash
bash scripts/test_composable.sh --layer pure
bash scripts/test_composable.sh --layer fake
bash scripts/test_composable.sh --layer learning
bash scripts/test_composable.sh --layer sim
```

sim 另需 MuJoCo；real/input ROS 路径需要安装并 source ROS Humble 及本工作区。测试依赖不能代替设备 SDK 和标定。

安装各 ROS Python 包后，`morla-composable-web` 默认监听 127.0.0.1:8080。开发时按测试脚本设置源码 PYTHONPATH 后调用 `morla_web_manager.composable_api.main()`。持久化目录由 MORLA_SYSTEM_STORE 指定，默认 ~/.local/share/morla；组件版本和部署改变时必须重新编译、保存 revision。

## 操作顺序

1. Web「组合」页添加组件，分别选择 backend、安装侧、输入、controller、deadman、观测/动作端口和相机键。配置机械连接、TCP 与部署资源。
2. 编译并预检；预检只检查元数据，不自动访问设备，也不代表真实部署已验收。
3. 保存 revision，然后启动对应已保存 hash。启动保持 disarmed。准备和显式激活后，输入仍须先释放再按下 deadman。
4. 手机使用会话配对 token；ROS 输入通过 connect_inputs 连接。失联、时效超限或资源争用会撤销运动权。
5. 采集冻结本会话契约。没有必须的动作或相机样本时，结束结果应为 failed，不能作为完整 episode 用于训练。
6. 停止采集、disarm/stop。故障或急停复位不恢复旧动作，重新准备并确认反馈。

## 策略与回放 API

`POST /api/v2/sessions/{id}/policy/start` 接收 manifest、normalization_hash，可附 backend_revision、camera_map、prompt。仅连接本机 8001 策略服务，服务必须公布与选中模型完全一致的 manifest，运行时还检查当前系统契约与 authority。`policy/stop` 撤销策略租约。网络服务存在并不等于模型经过真实验证。

`POST /api/v2/sessions/{id}/replay/step` 接收 frozen_resolved 与命名契约对应的 action，仅执行显式单步并释放 playback 权限。完整 episode 的时间调度需由调用方驱动 RuntimeReplay；不要通过循环 HTTP 快速发送代替数据集时间轴。

训练使用 morla_learning 的 TrainingSpec 和 CLI，ACT/OpenPI wrapper 的 --spec 分支使用冻结配置。后端仓库、Python、revision 和数据指纹必须匹配；真实训练命令执行前先检查生成配置。当前测试未证明任何真实模型的训练效果。

## 回退

停止组合会话并确认资源释放后才能启动旧入口；保留原始数据与标定文件。旧 Profile 维度/hash 对照位于 test/fixtures/legacy_profiles。禁止原地把 v3 数据伪装成 v2，禁止以删除锁文件解除仍活动的 writer。
