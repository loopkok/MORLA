# 工作站前端重构软件验收记录

日期：2026-11-09（Asia/Shanghai）。源码基线：main / d3e0ab3；实现提交以本记录的 Git 历史为准。对应[实施计划](../plans/WEB_WORKSTATION_UX_REDESIGN_2026-11-04.md)。

## 已交付

六个页面、右侧设备库和单设备编辑、窄屏抽屉、独立检查与运行页已实施。配置状态集中持有；切换顶层标签仍保留草稿和活动会话。设备字段及控制器仍由 descriptor 驱动。

补齐了部署覆盖来源与编辑、后端切换保留连接信息、复制后的独立身份、删除引用预览、安装树、控制链切换差异确认、FR3 独立平移轴映射、旧字段清理预览、记录通道和诊断设置。

检查调用一次 preflight，保留编译问题、依赖结果及原始详情；错误可定位并聚焦字段。草稿代次和取消信号防止过期检查覆盖；保存使用 expected_revision，409 保留草稿并刷新版本目录，支持另存为。

初始化分别跟踪建会话、输入接收、设备就绪；开始遥操及策略交接依据 transition/state 确认。共享状态请求顺序轮询并合并并发读取，操作后的确认读取等待旧读取结束；断网和超时显示结果待确认，不自动重放 POST。暂停、急停、停止在普通操作忙碌时仍可请求。

## 自动验证

- `npm run test:workcell` 等价命令：40 项通过，0 失败。原有 21 项与新增 19 项涵盖配置保留、控制器兼容、未知插件、来源/上限、旧字段、JSON/400/409/500、异步失败、404 未登记、取消检查、响应丢失、transition 回滚与 Toast 快照。见[测试输出](evidence/workstation-ux-2026-11-09/frontend-tests.txt)。
- `npm run build` 等价的 `tsc -b` 与 Vite 构建通过，已重建并交付 `web/dist`。
- `test_workstation_operator.py`：28 项通过。覆盖录制保存/丢弃、回工作位、策略/回放接管交还、键盘心跳、暂停中断、迟到回执、无位置但健康的设备使能与故障反馈。出现一条现有 FastAPI/Starlette TestClient 弃用提示，不影响结果。
- 本机未在 PATH 提供 npm，执行时使用 Codex 缓存中的 Node，直接运行仓库 TypeScript、Vite 和 node:test；没有更改 package-lock 或系统依赖。

## 浏览器与实际 v2 服务验证

服务运行在 `127.0.0.1:18089`，使用 `/tmp/morla-ux-preview-store`、独立资源锁和诊断目录；FR3、CR5、Vader 插件安装到 `/tmp/morla-ux-plugins`。Mac 无 ament 索引，预览手动注册仓库插件清单。所有执行器使用 fake 后端；有效输入由预览脚本在全 fake 会话中注入固定 Quest 测试帧。脚本位于 /tmp，不属于产品代码。

1. FR3 动态预设 → 检查（观测/动作各 6 维）→ 保存 v1 → 初始化 → 等待输入。无输入时开始遥操明确显示「重锚缺少新鲜输入」，没有显示成功。
2. 有效测试输入下 FR3 完成异步遥操切换，显示人工遥操和会话运行中；暂停后回到 READY_DISARMED。急停后可复位；刷新恢复原 session_id，没有重建会话。测试结束后停止会话。
3. CR5 fake 配置带设备速度 .25、部署速度 .1 和连接地址 192.0.2.2，检查和保存/加载保留部署覆盖。并发保存得到 HTTP 409；页面保留「保留本地编辑测试」名称，另存为新配置成功。随后检查 → 保存 v2 → 初始化 → 遥操 → 停止成功。
4. 带 missing.pose 错误引用的草稿检查返回 422。问题按钮跳到 right_vr 控制链，聚焦「right_vr输入信号」，字段旁保留错误。
5. 测试录制完整性失败时显示「记录未通过完整性检查」，保留检查提示。后续修正使此类业务失败也记为未完成并打开错误详情。
6. 十台设备时仅渲染一个设备编辑卡片。1440、1024、650、390px 下 document.scrollWidth 均等于视口宽度；650/390 使用可关闭的设备抽屉，点击已选设备也会关闭并返回触发按钮。最终截图已保存。

API 保存/加载、并发冲突和特征契约的[实际响应摘要](evidence/workstation-ux-2026-11-09/api-checks.json)保存在证据目录。

## 截图

- [1440px 十台设备](evidence/workstation-ux-2026-11-09/1440-devices.png)、[1024px](evidence/workstation-ux-2026-11-09/1024-devices.png)。
- [650px](evidence/workstation-ux-2026-11-09/650-devices.png)、[390px](evidence/workstation-ux-2026-11-09/390-devices.png)、[设备抽屉](evidence/workstation-ux-2026-11-09/drawer.png)。
- [保存冲突详情](evidence/workstation-ux-2026-11-09/conflict.png)、[无输入失败](evidence/workstation-ux-2026-11-09/error.png)、[假设备运行](evidence/workstation-ux-2026-11-09/run.png)。运行/错误截图记录对应验证当时的构建；尺寸截图来自最终构建。

## 现场验收边界

软件实施和上述自动/假设备验证已完成。真机连接与运动、ROS 输入采集进程、真实 Quest/Vader 数据、手机现场配对、所有品牌实物、MuJoCo 渲染/本机 viewer、物理相机及实际日志下载仍需在具备设备和依赖的主机复验。这些入口已迁移保留，不能把代码核对或 fake 结果称为实机验收。

计划中的未勾选项保留为尚未完整复验的场景，不代表自动把硬件现场结果置为通过。没有新增基于空位置数组的使能门禁，也没有改变后端驱动、IK、控制权或录制契约。
