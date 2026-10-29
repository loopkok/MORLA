# 黑武士 5 Pro 最小输入 SDK（Ubuntu / Linux）

Python 3.9+，运行时无第三方依赖。USB 数据线连接、手柄处于 PC 模式。
也沿用上游的 2.4G 接收器路径；本项目尚未进行实机验证。
不需要安装完整 tux-invader，不启动服务、GUI 或虚拟手柄。

## 运行

在本目录执行：

```bash
python3 -m vader5_sdk
```

持续打印每份报告的 JSON，Ctrl+C 关闭输入流。首次使用请关闭 Steam、飞智配置软件及其他读取厂商数据流的程序。
如提示权限不足，可为飞智 hidraw 接口创建 udev 规则（仅对该厂商接口生效）：

```bash
sudo tee /etc/udev/rules.d/70-vader5-sdk.rules <<'EOF'
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="37d7", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
```

然后重新插拔 USB。`uaccess` 用于本机活动桌面用户；SSH/无桌面部署需要管理员为运行账号配置对应设备的读写权限。
如果插入多个飞智设备，可指定 `python3 -m vader5_sdk --device /dev/hidrawN`。

## 程序接口

在本目录可以直接 import；其他项目可先 `python3 -m pip install -e /实际路径/src/common/vader5_sdk`。

```python
from vader5_sdk import Vader5

with Vader5() as pad:
    while True:
        s = pad.read(timeout=1.0)
        print(s.pressed("M1"), s.pressed("A"))
        print(s.left_stick, s.right_stick)  # 各轴 [-1, 1]
        print(s.triggers)                  # LT / RT，各 [0, 1]
        print(s.gyro_rad_s)                # 三轴角速度，rad/s
        print(s.accel_m_s2)                # 三轴加速度，m/s²，含重力
        print(s.raw)                      # 原始整数、数据包、未识别按键位
```

`read()` 返回当前批次最新状态；`read_batch()` 返回当前批次全部报告，记录短按/松开时应使用后者。
超时抛出 `TimeoutError`，断开连接抛出异常，不以旧状态冒充新数据。不自动重连。
`received_at` 是主机解码时的单调时钟，不是手柄硬件采样时间。

按键名称：UP、RIGHT、DOWN、LEFT、A、B、X、Y、SELECT、START、LB、RB、LT、RT、L3、R3、C、Z、M1–M4、LM、RM、FN、TURBO、HOME。
这些名称来自上游已解码位；TURBO 据上游记录是约 50 ms 的脉冲，不代表持续按住状态。
机械开关/拨杆是否有独立状态，不在该映射的保证范围内。

## 准确性与验证范围

协议与 Linux 传输直接复用 tux-invader（来源与文件哈希见 NOTICE），上游记录验证固件为 7.1.4.0。
本封装仅通过离线协议及接口测试，尚未接入实物，因此不能承诺不同固件下所有按键、IMU 均准确。
没有施加摇杆死区、滤波、IMU 零偏校准、重力扣除或姿态融合。IMU 是角速度与加速度，不是姿态角/四元数。
轴序、正负号沿用手柄报告；用于机器人坐标系前需要实际转动各轴确认方向。

首次实测请逐个按下/松开每个按键、推动摇杆与扳机检查完整行程，并静置/分别绕三个方向转动手柄核对 IMU。
若异常请保留固件版本和原始报告。上游的正常退出流程会关闭测试模式；被强制结束时可重新插拔手柄恢复。

离线检查：`python3 -m unittest discover -s tests -v`。

许可证：GPL-3.0-or-later，保留上游 LICENSE 与 NOTICE。此目录独立于 MORLA 运行流程。
