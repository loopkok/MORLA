# Vader 5 Pro 实机部署与文档对齐推送回执：2026-11-01

记录时区：Asia/Shanghai。仓库 <https://github.com/loopkok/MORLA>，分支 `main`；未使用 force，未重写既有提交。起点 `16a7fc8`（`docs: 记录统一组件重构提交与推送回执`），推送前 `origin/main` 与本地 HEAD 一致。

## 提交内容

| 提交 | 主题 | 范围 |
| --- | --- | --- |
| `26fee54` | `docs(vader5): record USB bench results and add real input deployment` | 7 files changed、140 insertions、9 deletions |

### 真实输入部署

- 新增 `src/common/morla_vader5_input/config/input.real.yaml`，记录 2026-10-30 台架实际使用的参数：`input_topic=/vader5/events`、`deadzone=0.08`、`max_age_ms=250`、`deadman_button=M1`、`rotation_button=M2`、默认 `action_buttons`（START/B/Y/X）、`axis_signs=[1,1,1,1,1,1]`。
- 设备身份：USB `37d7:2401`，USB iSerial 与 HID_UNIQ 都等于产品字符串 “Flydigi Vader 5 Pro”，SDK `ControllerInfo(device_id=130, model='Vader 5 Pro', firmware='7.1.5.0')` 无 SN。**该型号不暴露唯一硬件序列号**；部署以 VID:PID + 台架资产标签 `VADER5_37D7_2401` 作为 `device_id/resource_key`，元数据写 `serial: none-exposed`。同型号多台必须另起标签，否则资源键冲突；不得把产品字符串或 130 当作出厂序列号。
- `device_path_note` 说明台架捕获时为 `/dev/hidraw5`，hidraw 编号会随插拔变化，应按厂商 37d7 选择或使用 udev 符号链接。

### 文档对齐

- `src/common/morla_vader5_input/README.md`：验收状态改为 USB 实物已测；补充 status 话题无初始消息、受管 reader 退出不被会话感知、启动期陈旧事件、LT 精细倍率只乘六轴 jog（scalar/RT 直通）、`start_recording()` 不检查会话状态等行为。
- `docs/integrations/vader5.md`：新增 2026-10-30/11-01 台架补记（V01–V04、两项高影响缺陷、设备身份、NOT_RUN 清单）。
- `docs/integrations/teleoperation_rates_and_parameters.md`：补入手柄实测频率条目。

### EPG 台架实配补录

- 新增 `src/robots/jodell/morla_epg_driver/config/deployment.real.yaml`（资产号 `EPG50_DSEAb114J19`、by-id 串口、站号 9、115200、实测 descriptor 参数、`stop_verified=true`），README 增加指针。断链/watchdog 仍按操作员决定记 OUT_OF_SCOPE。

## 提交前验证

- `git diff --check` 无输出。
- `python3 -m morla_core.composition.cli compile src/common/morla_vader5_input/config/cr5.fake.yaml --deployment src/common/morla_vader5_input/config/input.real.yaml`：`compile_valid=true`、`backend_available=true`，`pad` 解析为 `real`，参数与配置文件一致。
- 设备复核（USB 重插后）：`find_config_hidraws()` 仅返回 `/dev/hidraw5`；SDK 读到 `device_id=130`、固件 `7.1.5.0`、电量 60%（充电中）。原始属性见 `$HOME/morla-acceptance/vader-20261030-171327/replug-identity.txt` 与 `step1-recheck-20261101.json`。
- 本次仅改配置与文档，未改运行时代码，未重跑完整测试套件；既有台架证据见 `$HOME/morla-acceptance/vader-20261030-171327/VADER_BENCH_REPORT.md`。

## 实际推送与核对

1. `git fetch origin main`：`origin/main` 为 `16a7fc8ec3dbd8fc128d5269d23eb93b68aaf4ce`。
2. `git push origin main` 退出 0：`16a7fc8..26fee54  main -> main`（SSH `git@github.com:loopkok/MORLA.git`）。
3. 推送后 `git rev-parse HEAD origin/main` 两行均为 `26fee5463a719d49d88eb02768cd86d69a6134cc`。

本回执文件按既有惯例作为后续文档提交单独推送；其自身 SHA 由 Git 历史记录。CI 由推送触发，结果以 GitHub Actions 为准（本机未粘贴 run id）。

## 边界

- 该手柄没有唯一序列号，`verified=false` 不表示身份已验证为某台具体设备；同型号替换不会被软件区分，需靠资产标签与操作流程管理。
- 两个仍需修复的行为：CR5 IK 失败被升级为 `input_errors.pad` 并撤掉整会话；受管 reader 退出不触发会话断开/撤权/重启。
- 未验证：2.4G/无线、FN/TURBO/HOME 键位与功能、真实 CR5 跟随、物理拔插 USB 的完整会话行为、LT 在 jog 上的定量减速比、长时连续操作与端到端时延。
