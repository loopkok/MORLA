# 操作页精简与日志页验证

日期：2026-11-09。沿用本轮尚未提交的参考图 UI 改造。

- 工作空间创建、打开、导入与编辑集中到系统编排。遥操及诊断页只显示当前工作空间；没有已配置工作空间时保留遥操页，提供「配置工作空间」按钮；不自动跳转。
- 遥操页检查与启动操作粘附于顶栏下方，运行概览中不再重复配置入口；启动与开始遥操仍为独立动作。初始化相关界面文案统一为启动。
- 控制链的添加控制映射按钮置于列表/参数之前，并在属性区滚动时保持可达。
- 会话与诊断只显示日志文件配置与实时日志。诊断参数保存至现有 assembly.metadata.diagnostic_logging；运行时仍锁定配置，下一次启动生效。
- 工作站日志按顺序轮询读取已有 v2 tail；launch 日志优先订阅已有共享 WebSocket，未连接时顺序读取 v1/logs；离开页面取消等待。支持来源切换、关键词、暂停、跟随滚动、下载。
- 本地独立 v2 预览不含 launch 接口，此时明确显示完整 Web 服务未提供的原因；真正 launch 实时日志需在完整 Web/ROS 服务环境验证。

## 验证

- TypeScript 与 Vite 生产构建通过，dist 已重建。
- 42 项已有前端回归通过，见[输出](evidence/workstation-readability-2026-11-09/tests.txt)。本轮布局调整采用浏览器验证。
- 隔离预览打开 CR5 fake 旧配置后，遥操工作空间表单数为 0、顶层启动按钮为 1；诊断页工作空间表单与检查/启动按钮均为 0。
- 实际读取已有工作站诊断 tail，显示 controller/input/feedback/operation 等记录及原始日志下载；未触发硬件连接或运动。
- 添加控制映射已出现在属性区最前部。侧栏最终计算字号 16px、字重 700，移除运行/配置/资源分组文字。
- 1920px 视口 document.scrollWidth=1920，内容区宽 1680；390px 视口 document.scrollWidth=390。

[遥操页](evidence/workstation-readability-2026-11-09/teleop.png) · [诊断页](evidence/workstation-readability-2026-11-09/diagnostics.png) · [控制链顶部入口](evidence/workstation-readability-2026-11-09/chain.png)。遥操截图对应最终构建，其余截图对应本轮功能验证构建。

## 操作反馈修正

- 取消点击遥操控制及恢复结束时的强制跳转，空配置在本页显示引导。浏览器验证从系统编排切回后标题保持遥操控制。
- 组件属性约占分栏的 55%，画布约占 45%；图形根据实际容器宽度缩放，多字段在宽屏使用两列。
- 1280px 视口实际测量画布 433px、属性 529px，页面 scrollWidth=1280。
- 本次 tsc/Vite 构建通过；布局与导航通过实际浏览器验证。

[扩大后的属性区](evidence/workstation-readability-2026-11-09/expanded-properties.png)。

## 左右拖拽分栏

- 中间分隔线可向两侧拖动；分隔线具有可聚焦 separator 语义，方向键微调，Home/End 到两侧边界，双击恢复默认。比例保存在本机 UI 偏好中，不写入工作空间或部署参数。
- 宽度变化时图形重新适配，表单字体不缩放；属性区使用容器查询决定单双列，窄屏改为上下排列。
- 实际指针拖动验证：1280px 下左/右从约 434/530px 调整为 334/630px，输入字号仍为 15px。
- 键盘边界验证：End 为 604/360px；Home 为 300/664px。document.scrollWidth=1280，无页面横向溢出。
- TypeScript 与 Vite 生产构建通过，dist 已更新。

[可拖拽属性区](evidence/workstation-readability-2026-11-09/resizable-properties.png)。

## 分类设备库与简化分栏

- 分隔线没有白色手柄、三个点或竖条。透明区域仍支持拖动，键盘聚焦时在整个容器显示轮廓。实际拖动验证画布由 300px 变为 351px。
- 添加组件与组件库/驱动共用设备分类面板，初始不列全部设备；分类为输入设备、机械臂、末端、整机、相机、传感器。搜索仍可按名称发现组件。整机分类也收纳现有头/腰本体模块，显示其真实组件名，不宣称它们是完整机器人预设。
- 组件可通过 web.device_category 声明分类，现有元数据采用 kind、image/closure 端口和通用组件名回退；不修改实际插件、配置或后端能力。
- 浏览器实测：初始设备卡片 0；选择机械臂显示 6 个臂组件；侧栏 14px、名称 17px、后端说明 11px；统一 7 个 SVG 导航图标。分隔线文本为空、伪元素 display:none；1280px 下页面无横向溢出。
- 44 项前端回归通过（新增分类测试），TypeScript 与 Vite 构建通过。

[分类设备库](evidence/workstation-readability-2026-11-09/classified-library.png) · [回归输出](evidence/workstation-readability-2026-11-09/library-tests.txt)。

## 大屏比例调整

- 各页内容区最大 1480px，在侧栏右侧的可用空间内居中，减少超宽铺满造成的扁平视觉。
- 遥操/采集主画面使用 4:3 比例并增加最小高度；图像继续 object-fit:contain，不裁切视频。窄屏改为单列。
- 编排画布高度为 clamp(660px,70dvh,900px)，属性区保留纵向滚动。中间恢复 1px 淡线，保留拖动和键盘功能，没有三个点或白色手柄。
- 关键词、标题换行，表单/数值数组按容器宽度换列，操作条允许折行，避免字段越过面板。
- 实测 2290×1276 下内容区 1480px，主画面约 776×582px；画布高度约 893px。展开 FR3 专用参数后，超出属性区边界的输入/选择框数为 0。
- 1440、1024、390px 下 scrollWidth 与视口相同。TypeScript 检查及 Vite 生产构建通过，dist 更新。

[收窄增高后的采集页](evidence/workstation-readability-2026-11-09/balanced-capture.png) · [连接画布与组件属性](evidence/workstation-readability-2026-11-09/balanced-composition.png)。

## 内容宽度回调

按最新反馈取消 1480px 最大宽度与额外居中留白，恢复内容铺满侧栏右侧。保留画面和画布增高、淡分隔线及字段自适应布局。TypeScript 与 Vite 构建通过。

## 紧凑运行概览与右侧机器人状态

- 顶部工作空间/状态合并，1280px 遥操摘要高度 64px；当前会话移除动作维度和记录频率，两项导航放标题行。启动步骤完成后默认收起，可展开查看。
- 右侧按设备库类别列出实际配置的机器人，可逐台展开。左侧画面下方上下排列输入与控制权、会话与记录。分栏独立增长，采集设置和原有录制控制保留。
- 在隔离 18089 服务中打开既有 fr3_quest_fake v1，启动测试会话，确认页面显示测试数据来源、六关节反馈、125Hz 配置频率；实测频率保持暂无数据。开始 fake 遥操后可读取真实控制器报告及求解耗时/报告龄期，未伪造硬件反馈或 E2E；测试结束后通过原有停止确认释放会话。
- 反馈频率使用硬件序号差/采样时间差，覆盖序号回退、重复帧、无计数、故障、过期、测试及仿真数据测试。连接异常时不显示反馈在线。硬件实测频率尚未接真机验证；现有接口无完整 E2E 时间，因此显示暂无数据。
- 47 项前端回归全部通过；TypeScript 与 Vite 生产构建通过，dist 已更新。默认 1280px、1024px、390px 的 scrollWidth 等于视口，390px 展开反馈详情没有越界。

[遥操与设备反馈](evidence/workstation-readability-2026-11-09/compact-overview.png) · [采集概览](evidence/workstation-readability-2026-11-09/compact-capture.png) · [窄屏详情](evidence/workstation-readability-2026-11-09/compact-mobile.png) · [无设备状态](evidence/workstation-readability-2026-11-09/compact-empty.png)
