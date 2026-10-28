# MORLA 重构验收与对抗性审查规格

日期：2026-10-27。配套 [主规格](../COMPOSABLE_ARCHITECTURE_PLAN.md) 和 [任务包](EXECUTION.md)。下列为验收规格，已执行范围见 [progress.md](progress.md) 和 evidence/；本清单本身不是测试通过报告。

## 1. 不变量与门禁

I-A：同一物理命令接口在同一 epoch 只有一个被授权 producer；旧会话/旧序号/旧 deadline 不可使新目标下发。

I-B：设备真实状态、命令回显、估计、仿真和缺失值不得混淆；validity、来源与时间必须传到数据集。

I-C：启动/重连/保存配置不授予运动权；新激活需要完整前提，停止与失效在设备侧仍可执行。

I-D：同一 ResolvedSystem 在 CLI、Web、collector、policy 中解析出相同数据与控制契约；episode 中不可变。

I-E：按特征身份和单位映射，不能根据“都是 7 维/都是左臂/第 0 个相机”推断兼容。

I-F：训练观测和动作各自有独立布局；新增输入或传感器不隐式改变模型动作。

I-G：fake、物理 sim、real 与模型兼容分别验收；部分能力不可用时准确拒绝，不降级冒充成功。

I-H：已有原始数据、现场标定、关节约定和安全保护可追溯、可回退，不静默覆盖。

每一阶段只有适用门禁通过才能标 COMPLETE；软件 COMPLETE 不代表 HARDWARE_ACCEPTED。结果枚举为 PASS、FAIL、NOT_RUN、BLOCKED_ENV、NOT_APPLICABLE，最后一项必须解释依据。禁止用 skip 数量充当通过数。

## 2. 测试环境层级

- **T0 纯逻辑**：Python 3.10 和 3.12，完全无 rclpy/SDK/网络设备。schema、compiler、hash、状态机、映射、时间注入、property tests。
- **T1 协议与 fake**：Linux ROS Humble、mock SDK、stub input/model、虚拟相机；统计实际 gateway/driver 调用而非只查发布日志。
- **T2 集成仿真**：单场景 MuJoCo、组合 robot_description、实际 ROS topics/QoS、录制导出/loader/replay；明确资产仿真等级。
- **T3 Web/学习**：浏览器 E2E、真实前后端；锁定 ACT/LeRobot/OpenPI 环境的 CPU loader 和少步 GPU 验证，分开结果。
- **T4 真实部署**：真实输入设备、各驱动/执行器/相机/固件/标定/装配；现场人员能触及硬件急停并控制负载/工作空间，逐级低速验收。

测试 ROS domain 与生产 domain 分离，但这不能代替设备资源隔离。CI 默认不具备硬件设备权限，real adapter 在 CI 必须拒绝无显式配置的设备访问。每层环境固定依赖与机器信息，不能拿 Mac 无 ROS 的结果代表 Jetson 真机。

## 3. 旧基线与兼容用例

- **B01**：12 个 Profile 记录源 YAML/hash、resolved/hash、state/action 名称/维度和旧 launch 选择；相同源码重复导出一致。
- **B02**：旧 Quest 包、关节命名、零位/镜像、工具链、HITL ACK trace 在新适配器上重放；数值与事件差异在允许清单内。
- **B03**：旧 raw/aligned/LeRobot/ACT fixture 只读加载不修改源文件；源文件 hash 前后相同。
- **B04**：旧/新在同硬件、同负载、同数据下各跑性能与长稳；报告分位数和异常次数，不只报均值。
- **L01**：12 Profile 逐个比较 legacy view 的 v2 hash 不变；新 v3 身份单独保存。启动行为的有意改变（例如禁止自动使能）需显式说明，不能以 golden 一致为由保留危险默认。
- **L02**：legacy 与 composable 同时启动争用同一设备，后启动者在打开设备前被拒绝。
- **L03**：回退代码/配置 revision 后旧数据和标定仍可读；不执行原地 schema 降级。
- **L04**：旧 v1/v2 未包含的信息保持 unknown；迁移不虚构时间同步、真实反馈或 TCP 标定证据。
- **L05**：piano 的进程直连/维护路径与新 runtime 资源互斥，关闭会话后不遗留活动 writer。

现有 Profile 的预期数值维度（来自当前 test_profiles.py，R0 需实际校验）：

- Astral：left_gripper 8、left_gripper_3cam 8、right_gripper 8、dual_gripper 16、dual_gripper_wuji 35、left_wuji 27、right_wuji 27、dual_wuji 54。
- Nero：left_xhand 19、right_xhand 19、dual_xhand 38、dual_arms 14。

以上不是新架构的硬编码上限，也不是当前 real 支持矩阵。

## 4. 配置、契约与编译器

- **C01**：非法/重复 instance/feature/camera ID、重复 YAML key、未知字段/版本 → 启动前结构化拒绝，路径精确到字段。
- **C02**：NaN/Inf、负频率、非法 quaternion、重复关节、dim 与 names 不等 → 拒绝；不通过 `int()` 截断浮点维度。
- **C03**：m/rad、degree/rad、closure/opening 方向、移动/转动关节混用 → 仅显式转换器可接受，转换测试用已知数值。
- **C04**：输入端口为 axes、目标 controller 要求 landmarks → 拒绝并指出缺转换能力；不能自动补零。
- **C05**：随机打乱 YAML map 顺序不改输出；显式 feature 顺序变化必须改变相应 contract hash。
- **C06**：保留原特征语义仅变 namespace/部署地址，模型 feature hash 不变、部署 hash 改变；旧 v2 hash 仍按旧规则计算。
- **C07**：输入设备变化只改变 input mapping/hash；当输入纳入模型观测时才同步改变 feature contract。
- **C08**：同插件 ID/版本冲突、codec 未注册、依赖缺失/ABI 错误 → 目录给可诊断状态，不能启动后才 ImportError。
- **C09**：安装树环、多个父节点、frame 丢失、标定引用过期 → compile/preflight 分类拒绝；0 臂传感器配置无需伪造 base/arm。
- **C10**：启动依赖有环被拒；合法控制反馈回路可运行，不错误地禁掉闭环。
- **C11**：相同物理设备以不同别名、serial symlink、IP/port 组合重复声明 → canonical resource key 检出。
- **C12**：Astral 两逻辑臂或同协议总线多地址 → 合并预期会话，不重复连接；不支持共享的设备正确拒绝。
- **C13**：dry-run/catalog/validate 期间 monkeypatch 所有 SDK open/connect/enable 为抛错 → 无调用，仍能列目录/编译。
- **C14**：路径穿越、shell 字符、未受信插件入口、巨量 YAML alias/消息 → 拒绝或按文本处理，无代码执行/越界读写。
- **C15**：preview 后依赖被替换或 assembly revision 改变 → launch hash 校验失败；没有校验/执行竞态。
- **C16**：artifact 写一半时杀进程 → 原活动 revision 仍完整，未完成目录不会被当作有效版本。

Property-based 测试（例如 Hypothesis，锁版本）生成 N=0…6 个实例、不同维度/命名/排列/单位；检查命名空间注入、layout packing/unpacking 双射、hash 确定性、resource claims 无重叠。保存失败 seed 和最小化样例，不以随机“跑了很多次没错”代替不变量。

## 5. 适配器与几何

- **A01**：describe/configure 不产生使能/HOME/位置写入；日志和 fake SDK call recorder 双重断言。
- **A02**：只读 sensor 无 write capability；input 无 robot actuator capability；错误调用返回 Unsupported。
- **A03**：驱动返回空/缺名/乱名/重复名/非有限反馈 → gateway/collector 标 invalid；合法乱序可按名重排。
- **A04**：API 同步失败、异步超时、迟到成功、取消不支持分别返回可辨状态；不得重试非幂等操作两次。
- **A05**：只选 Astral 右臂，fake SDK 全量记录证明未对左臂/头腰发送新目标；板级 enable 作用范围必须在状态中展示。
- **A06**：Nero cold-start、enable feedback、J7、position/home/workpos 现有测试全保留；过期反馈不得被 readiness 放行。
- **A07**：双 XHand 调换 hand_id/顺序、复用设备接口 → 命令和反馈仍按实例正确分配；触觉缺失保持 invalid。
- **A08**：夹爪 ratio→rad/m 与反馈逆变换端点、方向、限速可验证；command echo 不标 measured。
- **A09**：相机 USB 重插/编号改变、同型号两台设备 → 语义角色固定到正确指纹，不按扫描顺序交换。
- **A10**：相机预览阻塞/断开、WebRTC 关闭 → 采集时钟/帧数在规定阈值内保持；相机句柄不被两个消费者重复打开。
- **G01**：随机合法关节 FK/IK/FK、named mapping round-trip，单组件沿用已有容差；新容差须有数值依据。
- **G02**：Nero J2/镜像约定和已有 link7 工具链数值 golden，不允许 double conversion。
- **G03**：混合臂的 base 不同、工具有非零偏移和旋转 → 目标在各自 solver frame 一致，不把一臂的基座用于另一臂。
- **G04**：相机 optical frame 与图像/CameraInfo ID 一致，calibration hash 与对应序列号匹配。
- **G05**：实例重命名/实例复制时 URDF/MJCF joint/link/actuator/sensor 不碰撞，所有映射覆盖且一一对应。
- **G06**：单场景组合 sim 里双臂/手/相机同步更新；两套独立整机 sim 不计此项通过。
- **G07**：缺 collision/inertia/TCP 证据时降级或禁止相应 real 操作，不能自动设单位阵“修复”。
- **G08**：工具安装或零位有意变化 → geometry hash 变化，旧模型/回放被兼容检查拦下；重新验收后才可建立映射。

## 6. Runtime 与命令权

- **R01**：随机 lifecycle 事件序列不出现未授权 ACTIVE；启动失败逆序释放已持有资源。
- **R02**：两个 Web/CLI 进程同时启动同设备 → 恰好一个 owner，另一个在开硬件前拒绝。
- **R03**：某 SDK read/write 卡死 → 其他独立设备状态/停止通道不被同一事件循环卡住；卡死设备触发规定停止/故障。
- **R04**：输入/设备/策略心跳停止 → 期限内不再接受新动作；重复回放最后位置是否允许由对应 StopPolicy 决定。
- **R05**：旧 session、旧 epoch、重复 sequence、乱序、超期、未来超界命令 → 设备调用数为 0；reject reason 可定位。
- **R06**：HUMAN 接管中丢一个 ACK/prepare_reanchor 失败 → 不授新权、不恢复旧权，group 保持停写。
- **R07**：相同 request_id 重发 enable/home/start → 返回原 operation，SDK 只执行一次。
- **R08**：一臂已发另一臂下发失败 → group stop，记录部分执行，禁止宣布原子完成。
- **R09**：kill -9 runtime/worker、主机进程重启 → 板端/驱动 watchdog 生效；重启无自动 replay/activate。
- **R10**：Web 断连、刷新、两个浏览器同时操作 → 命令权仍由 runtime 持有，前端不能成为单点安全闸。
- **R11**：wall clock 跳变、仿真 /clock 暂停/倒退/加速 → 真实 TTL 看门狗不失效；sim 回退产生新 epoch。
- **R12**：积压图像/录盘/网络推理任务 → 高频 mailbox 有界、丢弃旧值留计数，不排队执行陈旧动作。

调度测试必须观测“设备适配器实际 dispatch”的时间与参数，不能只测 publisher.publish 返回速度。测量调用后再验证反馈年龄和设备确认，明确信号链哪段有证据。

## 7. 输入与人机切换

- **I01**：Quest strict packet/session/sequence/time-sync 原测试保留；legacy 无包头仅显式兼容模式可用，UI 标明保护降级。
- **I02**：Quest world/body 变化或输入 session 更新 → 清缓存/锚点并要求 release 后重新 deadman，不产生跳变目标。
- **I03**：Xbox 轴方向/触发器范围/按钮映射实测；设备型号变化不能无说明沿用不匹配映射。
- **I04**：deadman 松开、手柄拔线、无线断连 → 对应 group 撤权；重连时扳机仍按住也不自动恢复。
- **I05**：Xbox/手机切换控制臂时保持轴非零 → 新目标保持 disarmed，必须回中/重按才能运动。
- **I06**：手机 touchcancel/锁屏/切后台/浏览器崩溃/网络重连 → 服务端 timeout 独立有效；客户端 release 只是加速手段。
- **I07**：手机重复包、伪造会话、超大消息、旧配对 token → 拒绝且不影响 runtime 的停止通道。
- **I08**：输入频率由 120 Hz 降到 5 Hz/突发积压 → 超时或显式降级，不把旧输入积分为大位移。
- **I09**：手套与 Quest 同时宣称同端口/资源 writer → 显式选择或拒绝，不能最后写入者覆盖。
- **I10**：手势 preset/synergy 只能控制声明的自由度；记录其版本/模式，不能标作全关节直接示教。
- **I11**：语音与脚踏重复 record_start/stop、多麦克风同一命令 → 幂等、ACK 对应唯一 operation，确认删除流程保留。
- **I12**：无 VR 进程时 Xbox/手机完成遥操与录制，确认没有隐藏的 Quest topic/按钮依赖。

## 8. 数据、学习与推理

- **D01**：观测 11 维、动作 8 维，动作顺序与观测不同，含只读速度 → raw/aligned/export/loader/replay 的形状、名称、每个值独立正确。
- **D02**：0 相机数值数据可录制和处理；需要图像的模型收到明确 missing modality，不能制造黑图凑数。
- **D03**：camera keys 为 `scene_top/wrist_tool_a/aux_depth_rgb`，设备顺序随机 → 数据键/模型槽位显式映射不漂移。
- **D04**：38/54 维及非对称多臂数据无截断；32 容量模型拒绝，但采集/兼容导出仍可独立完成。
- **D05**：源时间与接收时间分离，时钟偏移/漂移/重新同步、包乱序、sequence 回绕边界 → 对齐报告可追溯、未同步数据不假标同步。
- **D06**：慢/缺/重复相机帧、传感器缺口、joint NaN → validity/reason/source index 一直保留；清洗不按 frame 重新错位配对。
- **D07**：next_state 只投影到对应 action features；末帧合成 hold 有标记，不能把触觉/速度整个复制成位置命令。
- **D08**：requested 超限被裁剪 → requested、authorized、dispatched、feedback 分开；训练选源与 metadata 一致。
- **D09**：录制中修改装配/键名/TCP/fps → 活动 episode 不变，修改只进入下版本，启动新录制需新 revision。
- **D10**：磁盘满、writer 崩溃、取消保存、数据目录重名 → 原数据不覆盖；不完整 episode 不进入默认训练索引。
- **D11**：触觉来自 native/sim/missing 三种输入 → 数据来源和温度/力有效掩码正确；新增不同 sensor codec 无 collector 品牌改动。
- **D12**：export 的 state/action norm stats 分别对齐具名特征；换 dataset fingerprint 必须重算或验证统计，不能复用错误 norm。
- **D13**：同 feature dim 但 unit/顺序/相机 preprocessing/action mode 不同的两数据集合并 → 默认拒绝，显式转换另存 provenance。
- **D14**：旧 v1/v2 显式迁移到 v3 后用具名值对比；hash 变化报告明确，无原地修改。
- **P01**：ModelManifest 与 runtime 完全匹配 → 正常 stub/真实 backend 握手，实际输出通过 gateway。
- **P02**：仅长度相同但特征/单位/动作模式不同 → 启动推理前拒绝。
- **P03**：相机缺失/颜色空间/resize/槽位错误 → 明确诊断，不自动改模型约定。
- **P04**：norm/checkpoint/几何 hash 不符或 manifest 缺失 → 默认不取得 motion permit；旧模型用显式可信迁移工具产生说明。
- **P05**：过期 chunk、策略断线、推理线程阻塞、恢复后旧队列 → 不发送过期动作，停止/等待策略符合设备契约。
- **P06**：HUMAN→policy→playback 切换伴随输入 epoch 变化 → 旧 chunk 清空、重新锚定、owner/介入标签与实际命令对应。
- **P07**：训练配置生成后 ACT/OpenPI 数据 loader 读同一 fixture → names/shape/action semantics 与导出一致。
- **P08**：锁版本 backend 少步训练产生 checkpoint → ModelManifest 写入正确；再加载完成一次 inference，不能只证明 CLI 有参数。
- **P09**：replay 使用历史冻结契约，当前同名 assembly 已升级 → 不跟随当前配置变动；无损兼容才准执行。
- **P10**：外部训练 Python 环境无 rclpy、ROS 环境无 torch/训练 SDK → 两侧独立正常；进程协议版本不匹配明确拒绝。

逐元素“金丝雀”样本：让每个 source/feature/time 有可唯一识别的值（数值控制在 fake 限位内），而非全部为 0。随机排列字段后，从 raw 到 loader 再到 fake dispatch 对照期望 ID，能直接检出左右互换、截维、索引错位与 off-by-one 时间错位。

## 9. Web 用例

- **W01**：只安装一个测试插件，组件目录自动出现；Web 公共代码无需品牌分支。
- **W02**：从空配置创建混合 arm/tool/input/camera，保存、重开、导出 YAML、CLI 编译 → 同一 hash。
- **W03**：非法 binding、缺 driver、缺标定、模型容量不足 → 字段/组件级原因与后端一致。
- **W04**：两个浏览器编辑同一 revision → 后提交者收到冲突与 diff，不覆盖先提交版本。
- **W05**：连点 launch/prepare/HOME 或重发网络请求 → 一个 operation、一个 device session。
- **W06**：预览后修改配置或依赖 → 启动拒绝旧 preview token；用户看到实际版本。
- **W07**：录制/推理中保存新配置 → 当前 contract 不变；应用按钮按真实 runtime 状态限制。
- **W08**：浏览器刷新/事件断流/服务重启 → 从 runtime 快照恢复，不显示虚假已激活或已停止。
- **W09**：动态显示第三臂、0 相机、20 关节手、任意数据键；图表无固定左右/7 维假设。
- **W10**：旧 v1 控制接口与 piano 测试回归，不能借旧接口绕过新资源/租约检查。

## 10. 最小组合矩阵

矩阵是必须覆盖的场景，不是宣称所有真机当前可用。每个场景分别记录 compile/fake/physics/real/model 状态。

- **M01**：现有 12 Profile，逐一 legacy/new 等价；包含 8/14/16/19/27/35/38/54 等维度。
- **M02**：左 Nero + 右 Astral + 两个经描述的 gripper，验证跨品牌会话、数据与操作影响范围。无 Nero 夹爪真实 backend 时 real 明确不可用。
- **M03**：左 Nero+XHand，右 Astral+Wuji，验证不同手 DOF、不同 retarget 与组合场景。
- **M04**：双臂双 XHand / 双臂双 Wuji，分别验证 38/54 维和模型容量拒绝。
- **M05**：双臂一手一夹爪，与 M02/M04 对比只改实例和 binding，无公共 launch 改动。
- **M06**：只选 Astral 右臂但物理会话是整板，测试未选关节与全板操作限制。
- **M07**：同型号三条 fake arm，角色为 `tool_a/tool_b/aux`，验证不依赖 left/right。
- **M08**：纯相机/纯只读 sensor，0 action、0 arm，采集和预览正常；动作模型明确拒绝无动作配置。
- **M09**：单臂无相机，数值控制/采集正常；视觉模型不可用有明确提示。
- **M10**：混合 real/fake actuator 同 group，默认拒绝；显式隔离 HIL 模式必须有来源标签。
- **M11**：Quest pose + 手套手部，同时独立绑定不同端口。
- **M12**：仅 Xbox 控制单臂+夹爪，环境中不启动任何 Quest 节点。
- **M13**：仅手机控制单臂或夹爪，锁屏/后台/掉网反例完整。
- **M14**：Quest 控左臂，Xbox 控右臂，手机 scalar 控夹爪，命令 scope 不冲突。
- **M15**：一个手柄显式切换臂/末端，轴非零切换时必须保持 disarmed。
- **M16**：同一 actuator 被两个连续输入同时绑定，compile 或激活阶段明确拒绝，除非声明并实现了仲裁 controller。

所有类型组合不必穷举笛卡尔积；先全覆盖上述关键矩阵，再对 brand/DOF/input/camera/backend/故障做 pairwise 组合并固定种子。任何新增 capability 必须补一项正例、一项不兼容负例与一项故障例。

## 11. 对抗性审查专项

### 11.1 审查时点与角色

第一次：R1 契约冻结前，挑战“是否仍然按左右/品牌隐含建模”“输入是否被强行当机器人动作”。第二次：R3 gateway 完成后，攻击单写者与生命周期。第三次：R6/R7，攻击数值与模型语义。第四次：R9 完整集成与硬件部署。

作者准备变更/设计、可重复输入、环境与初始状态；独立 reviewer 先基于规范构造预期，不只读取作者测试。reviewer 给最小反例；作者修复并补回归；reviewer 在独立运行中确认关闭。若只有一个 agent，当轮结果明确标“作者自审”，不能等同独立审查。

### 11.2 高风险攻击场景

- **S01**：让旧 teleop 和新 policy 同时对 driver 写；通过底层 fake call recorder 确认旧路径失效，而非仅看新日志。
- **S02**：获取租约后杀 owner、复制旧 token、重启复用旧 session；旧命令全拒绝。
- **S03**：控制权交接在每个步骤随机中断，尤其一个 ACK 迟到、旧 ACK 重放；不得误完成新事务。
- **S04**：租约合法但 command features 缺一个手指/重复一个关节/错单位；整个事务拒绝，不能部分拼接缓存旧值。
- **S05**：软件 stop 时 SDK 正卡在写操作，验证独立设备/板端 watchdog 与声明的停止上界；无法证明时 real 部署不通过。
- **S06**：部分启动成功后最后设备失败，之前启动的执行器不能留在 active；锁和子进程清理完整。
- **S07**：Astral 只选右臂，发 enable/home/full-body 操作；试图误动未选择的物理关节，检查实际 SDK 参数与硬件影响范围。
- **S08**：资源名字不同但底层设备相同；跨旧入口/new/maintenance lane 重复开设备必须被挡。
- **S09**：急停状态未解除、反馈陈旧、SDK 迟到返回成功；不能从 FAULTED 自动回 ACTIVE。
- **S10**：切断相机/录盘/模型服务并制造 CPU 压力；控制线程仍符合预算，缺观测模型立即按策略停/拒绝。
- **S11**：same-dim 模型交换左右关节、ratio 方向反转、norm 用另一数据集；握手必须识别，而非仅检查容量。
- **S12**：回放旧 assembly ID 的新版本、旧输入帧、旧几何；使用冻结版本且明确拒绝不兼容。
- **S13**：插入无效 sensor 值和重复图像，把质量标签改为有效，验证校验器可发现矛盾；不能只相信 meta 标志。
- **S14**：配置注入进 shell/import/path、替换 symlink/插件 artifact；执行使用锁定的受控资源，攻击没有副作用。
- **S15**：手机高频消息/错误消息淹没网络入口；限流/有界队列生效，stop/health 不被饥饿。
- **S16**：硬件断电/网络分区/板端重启后发送旧缓存目标；必须新会话/重新准备/实测锚定，不能自动追旧目标。

### 11.3 缺陷分级与报告格式

P0：可导致未授权运动/停止机制失效/不可恢复原始数据损坏。P1：有效配置错误路由、静默数据错位/截断、模型语义错配、关键隔离失效。P2：明确报错但合法流程不可用/监控错误/性能门槛未满足。P3：文档/提示/可维护性问题。

P0/P1 未关闭不得通过相关门禁；关键用户流程的 P2 也必须关闭或降级发布并禁用对应功能。不能以“评审认为风险可接受”替代实际复现修复。

每条报告包含：ID、级别、源码版本、目标/不变量、最小配置与输入、运行命令、时序/日志、实际 vs 期望、影响范围、修复建议、回归测试、复测结果。关闭依赖复测 evidence，不依赖作者口头确认。

建议用 mutation testing 抽查关键保护：删除 epoch 判断、反转夹爪方向、让 expired command 可过、将 action_dim 改回 state_dim。对应测试必须失败，否则测试虽然绿也没有证明所需保护。

## 12. 性能与时效验收

以下是**拟定工程门槛**，不是现有系统实测结果，也不是通用物理安全停止时间。R0 固定测试机器与负载后可用书面 ADR 调整，但不得通过放宽安全 deadline 使失败消失。

- 参考负载：双臂 100 Hz（使用设备支持频率）、双手各自原频率、3 路 640×480@30fps、30fps 采集、stub/remote policy，同时开 Web 预览。若部署实际分辨率/频率不同，增设真实负载档。
- 同一主机新增 gateway/路由处理开销目标 p99 ≤ 2ms（100Hz 档），或 ≤ 对应执行周期的 20%；报告两种门槛，取更严格者作为该档预算。若 SDK 无法达到其声明周期，调整能力声明或优化，不造假样本频率。
- 新路径端到端输入→dispatch p99 相对等价旧路径增幅目标 ≤ 10%，同时必须小于绑定的最大允许年龄；记录 p50/p95/p99/max 与超期拒绝次数。
- 固定负载 1h 热身后量测至少 1h 稳态：队列上界不增长，RSS 无持续上升趋势；稳态 RSS 增长目标 < 10%，说明缓存峰值与允许波动。
- 软件集成长稳至少 8h，周期性输入断连/模型暂停/预览开关/录制段切换；所有 unauthorized dispatch 计数为 0，无无人解释的数据缺口。
- 多臂同步任务先声明可接受的 dispatch skew 与设备执行 skew。fake 同步测试可采用 10ms 预算；真实硬件阈值由任务/设备实测固化，不能以 fake 值替代现场证据。
- 失效响应上界按 `检测期限 + 调度裕量 + 后端停止时限` 分解测试；TTL、反馈最大年龄、板端 watchdog 各有版本化配置。任何一项没有有限上界，需显式记录并限制 real 激活。
- CLI compile 小型装配（≤20 实例）目标 < 2s；Web validate p95 < 500ms（纯逻辑缓存依赖），不访问硬件。此项服务体验不与驱动 deadline 混算。

CPU、RSS、队列长度、网络吞吐、图像重复率、输入年龄、feedback age、dispatch delay、拒绝原因、采集丢帧、磁盘队列均由 session_id/instance_id 关联。测试报告必须给原始 artifact 位置与采样方法。

## 13. 目标环境运行入口

以下是已有脚本，在配置好的 Linux/ROS Humble workspace 运行，R0 先确认参数与依赖；不在当前 Mac 冒执行真机脚本：

```bash
bash scripts/test_offline.sh
bash scripts/test_ros.sh
bash scripts/test_policy.sh
bash scripts/test_sim_integration.sh
bash scripts/test_astral_sim_integration.sh
```

先阅读 sim 脚本的输出目录和 backend 选择；使用独立 `/tmp` 输出和 ROS domain，不覆盖既有数据。前端在对应 web 目录执行 `npm ci`、`npm run build`，已有 piano `.cjs` 用 node test runner；R8 增加实际交互测试命令。

计划新增统一入口 `scripts/test_composable.sh`，支持 `--layer pure|fake|sim|web|learning`、`--case ID`、`--seed`、`--output`。硬件测试独立 `test_deployment.sh` 且不作为默认 CI 目标。每层先运行相关用例，再运行跨层回归；不要每次文档/样式修改重复全套硬件测试。

CI 按 pure、ROS fake、sim、web、learning-loader 分 job；GPU/硬件由有对应环境的 runner 执行并报告身份。缓存和数据 artifact 按依赖 hash 隔离。仓库当前没有根 `.github/` 测试工作流，R0 应明确采用 GitHub Actions 还是本地等效 runner，不能假定已接 CI。

## 14. 扩展性最终考试

- **E01**：新品牌 6-DoF fake arm + 自定义名字 + 新输入；只增加 plugin package/descriptor/assembly 即跑通 Web→控制→录制→导出→replay，公共包无新增品牌条件。
- **E02**：加入 6 轴力传感器与不等维动作、0 相机配置；采集器/训练配置器无供应商专用修改，视觉模型合理拒绝。
- **E03**：由未参与实现的人按 SDK 文档完成上述接入，记录必须询问/修改核心的每一步。若必须改公共源码才能实现已有 capability，架构验收失败；如果是全新 capability，需按正式扩展流程版本化，而非隐藏特例。

## 15. 本计划编写阶段的设计自审

以下是作者在源码核查和计划复读中识别并已写入规格的设计风险，**不是已完成的独立红队测试**：

1. 只抽象 actuator 忽略 input → 分出 InputAdapter/Binding/Controller，并明确 Xbox 与手机参考实现。
2. 每个逻辑组件启动独立 SDK → 增加 DeviceSession/资源 canonical key，覆盖 Astral 整板与共享总线。
3. Profile 支持不同维就认为数据管线支持 → 指出 align/export 仍按 state 维分配，增加 D01 金丝雀与独立 ActionSpec。
4. 统一 JointState 就统一所有单位 → 明确 prismatic/ratio/velocity 与显式单位转换。
5. “换 hash 定义解决兼容” → 冻结 v2 身份，新 v3 与几何/输入/特征身份拆分。
6. 把一条中央 Python loop 当调度框架 → 高频驱动隔离、worker/有界邮箱、多频率与 ros2_control 可选后端。
7. API 接受即设备完成 → Receipt 分级、实测 readiness、超时/迟到/取消语义。
8. stop 一律断力/置零 → 设备模式 StopPolicy 与板端 watchdog，识别载荷与夹持风险。
9. 可组合 sim 等于并排跑两个旧 sim → 要求同一场景及 namespace/几何/actuator 映射证据。
10. 数据导出等于训练支持 → 增加 TrainingSpec/ModelManifest/loader/少步训练与容量负例。
11. Web 保存与活动配置同一份 → 不可变 revision、原子产物、预览/执行 hash 与并发编辑检查。
12. 只迁移 teleop/policy 忘记 HOME/piano/CLI → 全部 producer 纳入权限或独占 legacy lane。
13. 没相机/没臂被旧 schema 拒绝 → v3 支持数值/只读系统，模型按需要求模态。
14. 自动重连恢复运动、旧 lease 未见即 manual_allowed → 新 runtime 默认禁止，重连新 epoch 重新准备。
15. 声称物理动作原子同步 → 区分资源原子授予与实际部分执行，要求 skew 量测和 group stop。
16. 手机只有协议无用户可用客户端 → 明确触屏 MVP、实际手机端验收和 AR 后续边界。

后续 reviewer 应主动尝试推翻上述设计，而不是把这份自审当作结论。
