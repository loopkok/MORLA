# MMK2 description

`urdf/airbot_mmk2.urdf` 与 `meshes/` 来自用户提供的 airbot_mmk2_description，原始 URDF 另存为 airbot_mmk2.source.urdf，MIT 许可见 LICENSE。
`mjcf/mmk2.xml` 由 `python3 scripts/make_mmk2_mjcf.py` 生成；保留 URDF 的连杆、坐标、惯量、关节范围和双指 mimic。
运行 URDF 的 package:// URI 已归一到 mmk2_description，原始 URDF 供资料核对；MJCF 使用包内相对资源。

底盘添加 world 平面 x/y/yaw 三自由度，以轮速和 URDF 轮半径 .08401 m、轮距 .3265 m 驱动；右轮 URDF 的 ±π 被修正为连续旋转。物理模式仍为平面速度伺服参考，不模拟轮胎接触/打滑。机器人视觉 mesh 不参与接触。

腕部相机按 link +X 向前、+Z 向上定义。头部连杆实际 -Y 向上、-Z 向右；镜头位于 head_pitch_link 的 (.11,-.1855,0) m 并朝下 .3 rad，以免落入外壳。此为仿真外参假定，不是硬件标定。IMU 安装在 base_link 的 z=.25 m；激光扫描在 z=.25 m，180 点、360°、10 m 上限，只扫描世界 group 0 几何。

`sdk_reference_mjcf/` 原样保留 SDK 0.0.4.dev5 的 XML（Apache-2.0，见 LICENSE-SDK），仅作为对照。SDK 缺失 meshes，且 head_pitch、臂/夹爪命名和传动与所供 URDF 不一致，因此未作为运行模型。SDK 的 .0838/.189 底盘参数未套用于此 URDF 模型。
来源与逐文件 SHA-256 见 PROVENANCE.json。

2026-11-11 核对：原始 URDF 与用户提供的 airbot_mmk2_description 逐字节相同，37 个网格文件的 SHA-256 全部一致。MJCF 转换补齐 URDF 的 mesh scale，保留右驱动轮及左右各一片手指的负缩放镜像。
