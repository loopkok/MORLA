# 统一组件路径最终代码检查与对抗回归

2026-10-30；检查对象是 c1896c6 后本地工作区，不代表远端 CI 或实机认证。方法为实现者源码复核、独立解释器静态检查、隔离安装及行为反例；未使用第二位人工/agent reviewer，不能将本报告引用为其独立背书。

## 发现、修复与复验

- UR01：公共 Runtime 原有原生设备品牌表和启动计划。现移到独立 morla_native_bridge 插件包；正常 morla.adapters 工厂注册，Runtime 仅调用选中工厂。native_ros 旧路径仅兼容导入，kinematics 也由设备包提供注册工厂。静态边界检查通过。
- UR02：自动托管与 Vader/CR5 VR 专用 launch 可能重复启动输入节点。兼容 launch 移除输入节点，只调用公共 session；原始单设备 launch 保留 external 运维用途。静态检查确认整链 wrapper 不再创建 Node/IncludeLaunchDescription。
- UR03：一条新输入事件可能重用另一端口缓存，导致 pose 或标量重复进入 IK/EMA。BindingEngine 对显式同源端口时间与 frame/epoch 去重；断连/重置会清除该状态。新回归覆盖缓存 scalar、epoch/reset，旧 CR5 hand/controller 切换数值测试继续通过。QuestRight 现在将手部 scalar/landmarks 和控制器 trigger 都转发给公共层，由绑定按端口处理。
- UR04：ControlHost 输入关闭异常可能跳过 controller 关闭。已保证继续释放 controller，记录关闭错误；进程退出与 killpg 间的竞态也不会中断其余子进程回收。
- UR05：同一个显式共享进程 key 配上不同 argv/env 时会错误复用。现在报告配置冲突并清理已启动进程；相同 key/配置只启动一次。external 不启动、不关闭已有外部进程。
- UR06：自定义控制器可返回与执行器自由度不一致的向量。目标构造处报告维度错误，不填零、不截断。新失败用例通过。
- UR07：旧 pinch 算法与新通用路径数值可能分叉。迁移前以 AST 提取旧 Node 数值方法生成固定 trace，含源 SHA；新 PinchFilter/TriggerFilter 逐元素对照通过。旧 Node 直接调用同一纯处理器，保留其原有 ROS 仲裁、输入优先级与计时行为；输出 slew 也共用同一函数。
- UR08：原生桥从源目录可导入不等于已安装可发现。实际构建/安装 wheel，使用真实 entrypoint 元数据创建两个逻辑组件，以 mock transport 验证单个物理会话和显式激活；没有 ROS、SDK 或硬件访问。
- UR09：无 Web 输入按钮原先缺少动作路由。ControlHost 与 Web 共用 dispatch_input_action；activate/disarm/estop 一致，录制可注入处理器。未配置录制处理器的纯控制 host 明确报错；带采集的公共 session 服务提供该处理器，不静默丢弃。

- UR10：单个源用于双臂/双夹爪时，缺少某个源端口的部分帧可能阻塞全部绑定。公共引擎现在只处理帧中存在的绑定源；旧 Quest 提供 left_scalar/right_scalar 和具名左右摇杆，时间戳分别保留，兼容 scalar 别名仍在。新用例验证左右手互不覆盖和单侧帧不阻塞另一执行器。

## 实际结果与范围

- pure：202 passed、4 skipped。包含已安装未知插件扩展考试、混合双臂、输入/夹爪矩阵、真实本机子进程、数值 trace、缺失插件/冲突/清理反例。不同 fixture 中的 fake/solver stub 不代表品牌实机性能。
- fake 联测：23 passed；MuJoCo 场景：7 passed。
- TypeScript/Vite：通过；Edge 浏览器：通过，pageErrors=[]。
- 静态边界检查：全部通过；Python 语法与修改的 package.xml 解析、git diff --check 通过。
- wheel：构建与安装成功；实际元数据工厂加载/共享会话验证通过，physical_transport=mock。

证据见 ../evidence/unified-20261030/。独立运行 `python docs/architecture_refactor/reviews/unified_review.py` 可复验层边界；完整行为回归使用 scripts/test_composable.sh。

## 仍不能据此声称通过的项目

当前主机无 ROS/rclpy。新版工作区 colcon 构建、真实 topics/QoS、驱动子进程启动、实体 VR/机械臂/夹爪/灵巧手/相机组合、部署中的停止反馈、实际 IK 性能仍须目标环境验证。历史远端主机的构建/EPG台架证据不能替代本轮代码验收。第二位 reviewer 未执行，这一点与自动化对抗回归分开记录。
