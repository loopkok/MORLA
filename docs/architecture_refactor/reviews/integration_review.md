# 独立集成对抗审查

日期：2026-10-28（Asia/Shanghai）。审查者：core/composition 实施 agent；审查对象主要由其他 agent 编写。编译器的后续修复属于审查者自己修改，不能作为独立作者隔离的证明。仅软件与模拟数据，无设备连接。

执行环境：应用附带 Python 3.12、`/tmp/morla-composable-deps` 以及所有 MORLA 源码包路径。复现脚本：`docs/architecture_refactor/reviews/integration_repros.py`。首次原始实现运行结果 **7 passed**：前六个用例以“观察到缺陷行为”为断言，表示缺陷成功复现，不表示功能验收通过；最后一个为同维异名/异单位模型拒绝的正向保护检查。

## IR01 — P1：保存中断可永久卡住下一 revision

位置：`AssemblyStore.save`。revision 文件使用最终文件名直接 `open('x')`，先写完 revision 再更新 latest。注入 latest replace 失败后，`revision-1.json` 存在而 `latest.json` 不存在；同一合法重试得到 `FileExistsError`，新服务实例同样无法恢复。若在写 revision 过程中中断，还会留下半写 JSON。不能将原子替换 latest 等同于整项保存事务的原子性。

修复要求：完整 revision 先 staging+fsync，再发布；对提交中断的孤立完整版本确定性恢复或移除未提交临时产物；重试不覆盖不同内容；revision 读路径核验完整性；多进程串行保存和启动精确 hash 仍须保持。

## IR02 — P1：旧/未认证 WebSocket 的 finally 可断开新手机连接

位置：`composable_api.mobile` 的 finally、`ComposableService.disconnect_input`。旧连接保留原 source 对象，重配对产生新的 source，但最终只传 `input_id` 执行 disconnect，实际断开的是当前替代连接。即使旧连接认证失败也进入 finally，能撤销合法新连接的许可。

修复要求：连接持有不可伪造 generation/connection ID，断开必须比较该 ID 与当前 owner；未取得输入连接所有权的认证失败分支不可影响当前连接；旧 challenge/epoch 不能续租或执行；双连接与重新配对分别验收。

## IR03 — P1：跨输入 deadman 被错误输入帧同名字段替代

位置：编译器 binding deadman 检查、`BindingController.step`。`source=phone_a.scalar`、`deadman=phone_b.hold_to_control` 能通过编译，映射器仅取 deadman 的端口后缀，使用 phone_a 自己的 `hold_to_control` 许可动作；phone_b 从未提供输入。源实例身份参与权限语义，不能只按端口后缀读取。

修复要求：当前单帧映射器拒绝跨来源 deadman；确需组合时使用明确多来源聚合器及 freshness 规则。运行时输入 identity 同样核验，避免仅靠编译器防护。

本审查者已获主 agent 指示，修改编译器拒绝跨源 deadman，并新增 `test_cross_source_deadman_never_authorizes_other_frame`。映射器由其作者独立修复。

## IR04 — P1：交换相机模型槽位仍通过握手

位置：`ModelManifest`、`validate_model`。保持特征、geometry、normalization 完全一致，仅把 manifest 中 `model_left -> cam_left` / `model_right -> cam_right` 交换为相反关系，握手仍通过。`camera_map` 不参与当前 feature hash，函数也不比较预期槽位映射；同维度图像因此可以喂入错误模型输入。

修复要求：模型槽位、预处理、颜色/分辨率等视觉变换有冻结身份，启动显式提供并比较；未知映射不得默认接受。测试包括交换、缺失、重复和额外相机。

## IR05 — P1：训练参数覆盖已经验证的 capacity/camera_map/type

位置：`TrainingSpec.config`。返回 policy 时最后展开 `**parameters`，允许 `parameters={'capacity':0,'camera_map':{'wrong':'missing'},'type':'unregistered_backend'}` 覆盖校验后的保留字段。`TrainingSpec.from_dict` 接受且 `.config()` 输出无效配置；Web training API 恰好只调用 config，未经过 argv 白名单。

修复要求：parameters 中保留字段拒绝；允许训练参数列表与 backend adapter 版本绑定；config、CLI、Web 都调用同一校验，不依赖另一路 argv 再检验。

## IR06 — P1：必需流零样本也标 complete

位置：`ComposableRecorder.stop`、服务 capture_tick。一个必需 state/action/相机流均无样本的 episode stop 后 `recording_status='complete'`；没有缺流 reason 或质量条件。服务直接读取 values 并以落盘时间附加，未携带采样来源/源序列/测量年龄；部分相机/传感器无 adapter 数据时被跳过，也会形成看似完整产物。

修复要求：complete 表示明确完成规则已满足，必需流缺失/写入失败/来源不可追踪须保持 incomplete/failed；样本来源必须区分 measured、command_echo、fake、sim，并保留源时刻与质量。不能凭 HDF5 close 成功认定可训练。

## 已验证保留的保护

同维 state 特征名称或单位变化后，`validate_model` 拒绝握手。此前 core 新用例还覆盖自定义三自由度 descriptor 的盲接编译、12 个旧 Profile 原 hash、跨进程 hashseed 一致、资源冲突、重复/非有限配置、图环与 schema v3 独立动作维度。

## 后续审查状态

以上六项已在修复前发送主 agent；作者按职责修复。修复后应将复现脚本转换为保护性断言再运行，并记录各项结果。本文首次结果不能当作缺陷已经关闭的证据。ROS 实机、Xbox 实物、手机浏览器设备、锁定 ACT/OpenPI checkpoint 加载、GPU 训练及长稳测试未在此审查环境执行。

## 修复后复审（2026-10-28）

复现脚本已经改为**安全行为断言**。首轮“缺陷存在”断言仅作为本文历史证据保留。

- IR01：修复后恢复 ASCII revision 成功；审查继续发现非 ASCII（中文 title）序列化差异导致相同版本误判冲突，已通知作者修复比较方式。最终回归同时覆盖 ASCII、中文标题和服务重建后恢复。
- IR02：验证旧 connection fencing ID 退出不会断开新配对连接；旧 token 认证失败也不会断开当前连接；当前 connection 正常断开才清理源。
- IR03：编译器与映射器构造均拒绝跨输入 deadman；输入帧 source_id 伪装被拒绝；重连 epoch 后必须重新观察 release 才能继续。
- IR04：交换视觉模型槽位在指定冻结 camera_map 后被拒绝；同维异名/异单位保护仍保留。
- IR05：训练 parameters 覆盖 type/capacity/camera_map 被构造阶段拒绝。
- IR06：零样本必需流 stop 返回 failed，并保存明确缺失原因；新增完整 canary 验证真实落盘的 fake state 标为 `source=fake, synthetic=true, measured=false`，动作标为 `dispatched_command`。
- IR07（复审新增并关闭）：Web start 原先未把注入 catalog 传入 runtime 校验器。现在增加安装式盲接测试：创建临时已安装分发元数据及静态 descriptor，通过自动目录发现一个三自由度新臂，不直接注册 catalog；完成 Web service save/start、runtime prepare/activate、授权 gateway 下发 `[0.1,0.2,0.3]`、采集 complete，逐元素检查 HDF5 state/action 值和来源标记。
- 额外权限保护：去掉 resolved_hash 的下载配置不能被 RuntimeSession 当成测试 fixture 接受。

物理场景复审另外发现并修复两项：只接受 child base/mount（相机 optical 根帧除外）的挂载，避免未实现的非根帧反变换被误当 identity；canonical joint → MJCF joint 使用 descriptor 中显式具名双射，移除按列表位置 zip 猜测。关节规范顺序倒置仍映射同一实际关节；映射缺失严格拒绝。这两项为审查者本人修改，由 runtime 作者先独立指出，回归位于 `src/robot_description/test/test_composable_scene.py`。

最终复审命令（从仓库根目录，包路径可按已安装环境替换）：

```bash
export PYTHONPATH="/tmp/morla-composable-deps:$(find src/common -mindepth 1 -maxdepth 1 -type d | paste -sd: -)"
/Users/a1234/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 -m pytest \
  docs/architecture_refactor/reviews/integration_repros.py \
  src/common/morla_core/test/contracts/test_composition.py -q
```

最终结果：**37 passed**（11 个集成安全回归、26 个核心配置/契约回归）。物理场景测试另用具备 MuJoCo 的 conda Python 执行，**7 passed**。测试通过仅覆盖这些具体软件条件；不扩展为未执行的部署验收。

补充复审：SDK ABI 结构检查与 fake 行为 harness、输入/手部 retarget 参考桥、YAML/JSON 深度/循环/展开大小属性测试加入后，执行 core contracts 全目录、retargeting tests、integration_repros 共 **53 passed**（应用 Python 3.12）。纯 JSON 的科学计数法数值使用严格 JSON parser 保留类型，避免 YAML 1.1 将合法 JSON 数字解析成字符串；cyclic alias、过深/过大文档和扩展 DAG 均有明确拒绝证据。性能采样独立见 `performance.json`，包含编译器、runtime session、采样脚本源码 SHA256 与 catalog hash；不替代部署实时性或长稳验收。
