# CR5 与 EPG 独立组件组合

本目录只有装配配置，不是联合驱动。CR5与EPG分别由各自包持有连接、descriptor和Adapter。

`assembly.fake.yaml`验证7维state/action的组合：CR5为六关节rad，EPG为一个opening_width（m，0闭合、0.05打开）。auxiliary保留夹爪状态与电流标幺值。没有填造安装变换、URDF或碰撞模型，因此不是已安装末端或仿真场景。

安装两个包后可编译：

```bash
python3 -m morla_core.composition.cli compile examples/cr5_epg/assembly.fake.yaml --output /tmp/cr5_epg.resolved.json
```

多组件快照交给通用runtime/Web宿主；单设备epg_driver入口会拒绝含机械臂的快照。真实接入需分别填写身份、验证声明和物理安装/TCP；此配置没有自动添加VR绑定，也不改变CR5裸手捏合deadman的作用。
