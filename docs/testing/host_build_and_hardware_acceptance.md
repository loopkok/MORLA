# MORLA 重构后主机编译、迁移与真机测试手册

更新：2026-10-29，Asia/Shanghai。适用于可组合架构重构，以及后续CR5本地URDF/IK、LoopHand、EPG、Vader5接入后的代码。复制到Linux主机后，以**现场checkout的完整Git SHA**为测试身份；不要仅写“最新版”。本文是待执行手册，不是主机或真机已经通过的报告。

## 1. 当前哪些已经做了，哪些还没做

已留存的证据：

- 本轮macOS/Python3.13：pure **172 passed**（包含11项Vader5输入/整链），fake数据/策略/Web **23 passed**，SDK **5 passed**；SDK移至同级目录后，5项SDK与11项输入专项再次通过。见[Vader5证据](../integrations/evidence/2026-10-29-vader5/manifest.json)。
- CR5：已做纯本体模型迁移/网格闭包核对、本地Pinocchio数学、限位/步长/FK回验、右手柄/右手追踪逻辑和安装布局测试。离线链使用真实CR5几何，反馈为fake；见[CR5模型与IK记录](../integrations/CR5_URDF_IK_2026-10-29.md)。
- LoopHand、EPG：已做独立协议/映射/生命周期的mock或fake测试；EPG包含原厂SDK配录制Modbus master的离线测试。不能替代实物串口与固件测试。
- 重构阶段做过7项场景测试、TypeScript/Vite构建、浏览器E2E、原有18项Node测试；属于当时本机证据，不是本轮Linux ROS/硬件验收。见[重构记录](../architecture_refactor/CHANGE_LOG_2026-10-28.md)。

**仍缺、需要在主机逐项补齐的验收：**

- [ ] B01–B05：Humble全量编译、IDL/C++、系统Python依赖、开发/普通安装、脱离源码路径的插件和资源发现。
- [ ] R01–R04：真实DDS通信/QoS、ROS fake正负命令、两个进程争用、旧入口与新runtime互斥及退出清理。
- [ ] C01–C04：CR5固件/TCP反馈、实测关节与URDF对齐、物理base/TCP、低速运动、停止/设备watchdog。
- [ ] Q01–Q03：Quest实物到fake CR5，再到真实CR5；右手柄与右手追踪分别验收，含切换/断流/重连。
- [ ] L01–L03：LoopHand Linux安装、真实串口只读、7维紧凑映射；**真实运动当前被代码禁止，需固件/停止/watchdog能力补齐后才能另行验收**。
- [ ] V01–V04：Vader5 Linux SDK/ROS、USB及2.4G、27按钮/IMU/轴方向、真实手柄→fake机器人→真实执行器。
- [ ] E01–E03：EPG完整型号/固件/接线、真实状态语义、初始化寻程、停止地址与设备watchdog。
- [ ] N01–N02：Nero/Astral及XHand/Wuji在**新runtime**中的回归；旧VR遥操成功不能直接算新路径通过。
- [ ] S01–S03：真实相机身份/时钟/采集、USB脚踏/手机/手套等输入、传感器来源与失联。
- [ ] D01–D03：真实输入/硬件录制、数据处理与实际loader、锁定后端少步训练/checkpoint推理/回放。
- [ ] W01–W02、P01–P03：主机Web/浏览器、实测性能/长稳、发布与回退，以及独立开发者扩展考试。

Python3.10/3.12的目标矩阵、远端CI结果、完整[VALIDATION规格](../architecture_refactor/VALIDATION.md)编号也需核对，不能由本机172项代替。当前不能整体标记为“重构真机验收完成”。

## 2. 测试顺序、状态和已知阻塞

按顺序执行：**环境/编译 → 离线测试 → 安装检查 → ROS fake → 真实输入+fake执行器 → 真实设备只读/标定 → 独立低速运动 → 遥操 → 录制/模型 → 长稳/回退**。各组每完成一个用例即填写[测试报告模板](host_acceptance_report_template.md)。

结果只使用PASS、FAIL、NOT_RUN、BLOCKED_ENV、BLOCKED_CAPABILITY、NOT_APPLICABLE。缺设备/依赖是BLOCKED_ENV；当前代码明确不支持是BLOCKED_CAPABILITY；预期拒绝成功可记为该**拒绝用例**PASS，不能把被拒绝的运动功能记为PASS。skip需逐条说明。

已知阻塞应保留原始错误：

1. **Nero模型哈希不一致**：主资产清单中的nero_left_arm与现有URDF不符，已在CR5记录中留证。完整资产检查/部分仿真可能因此失败。比较模型实际变动、来源和消费者后修复并重做身份验收，不能只把manifest哈希改成当前值来“过测试”。
2. **LoopHand real只读**：无确认停止ACK和已验证通信watchdog；`verified=true`也不能放开。其real运动、CR5+真实LoopHand抓取、VR/手套重定向当前均不是可直接执行的完成链路。
3. **Astral夹爪反馈**：现有桥的command_echo不是实测反馈，会阻止相关real组合激活。若现场仍如此，记能力阻塞，不标measured=true绕过。
4. CR5/EPG real模板保留false和占位符。首次物理停止/watchdog、关节方向等证据，先在厂商支持的维护流程中取得，退出该工具后再进MORLA。不能为了首次测试而把所有声明直接改true。
5. CR5目前是fake协议/运动学验证，未交付CR5物理MuJoCo场景；LoopHand的触觉、URDF/碰撞、VR重定向也未迁移。缺实现与“实现了但没测”需分别记录。

真机测试由现场操作员控制工作区并能立即使用物理急停；先空载/支撑。CR5当前软件stop使用DisableRobot，可能失去保持力；EPG activate可能开合寻程。先固定这些测试条件，再执行相应移动用例。本文不提供一键自动执行的真机故障注入。

## 3. H01 环境与证据目录

目标基线：Ubuntu22.04、ROS2 Humble、系统Python3.10；其他Ubuntu/ROS、x86_64/ARM64应分别出报告。先在独立测试checkout操作，保留生产工作区、原始数据和现场标定。

以下代码块用Bash。将路径替换为主机实际位置，每个新终端重新设置变量并source同一个安装树：

```bash
export MORLA_WS=/absolute/path/to/MORLA
export MORLA_RUN_ID="$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S)"
export MORLA_EVIDENCE="$HOME/morla-acceptance/$MORLA_RUN_ID"
mkdir -p "$MORLA_EVIDENCE"
cd "$MORLA_WS"
set -o pipefail
git rev-parse HEAD | tee "$MORLA_EVIDENCE/commit.txt"
git status --short --branch | tee "$MORLA_EVIDENCE/worktree.txt"
uname -a | tee "$MORLA_EVIDENCE/uname.txt"
cat /etc/os-release | tee "$MORLA_EVIDENCE/os-release.txt"
/usr/bin/python3 --version
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=91
export ROS_LOCALHOST_ONLY=1
ros2 doctor --report > "$MORLA_EVIDENCE/ros-doctor.txt" 2>&1
/usr/bin/python3 -m pip freeze > "$MORLA_EVIDENCE/system-python.txt"
```

91是示例测试DDS域；核对主机无其他该域测试。**DDS隔离不隔离TCP、CAN、串口、hidraw或实体设备**。全套fake/编译阶段不接通执行器运动；真机阶段再逐台接入。不要运行全局pkill、删除资源锁或清空整个ROS日志来“恢复”。

报告另记CPU/内存/GPU、ROS/RMW、设备资产编号与固件、SDK版本及安装位置、USB拓扑、IP/端口、所用Assembly/Deployment及SHA256；不记录私钥或长期凭据。

## 4. B01–B05 编译与安装验收

### B01 包发现和依赖

```bash
cd "$MORLA_WS"
colcon list --base-paths src | tee "$MORLA_EVIDENCE/colcon-list.txt"
rosdep check --from-paths src --ignore-src --rosdistro humble
# 依赖尚未安装时，在测试主机执行；未知rosdep key须留错，不用忽略参数掩盖。
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

应发现公共core/runtime/input/teleop/learning、morla_interfaces、robot_description及各驱动。`src/common/vader5_sdk/COLCON_IGNORE`应存在，SDK不作为ROS包进入列表。

供应商SDK不是rosdep自动提供：Vader5在本仓库同级目录；EPG使用用户原wheel；Nero使用third_party/pyAgxArm和其安装脚本；Astral使用已有的外部astral_robot_sdk。不要把缺SDK等同于配置compile失败，也不要在catalog时连接硬件。

ROS构建及生成的入口使用`/usr/bin/python3`；先退出conda。ROS运行依赖需对该解释器可见。纯逻辑测试使用下一节独立venv，**不要将整份composable-test.txt装入ROS系统Python**：其中PyPI pin和numpy版本可能与apt Pinocchio/cv_bridge等二进制依赖冲突。缺包按rosdep和实际import错误处理，记录安装版本；需要pip的厂商SDK可安装到当前系统解释器的用户site。

```bash
/usr/bin/python3 -m pip install --user ./src/common/vader5_sdk
/usr/bin/python3 -c 'import rclpy, numpy, scipy, yaml, jsonschema, h5py, fastapi, uvicorn, pinocchio; from vader5_sdk import Vader5'
/usr/bin/python3 -c 'import sys, pinocchio, vader5_sdk; print(sys.executable); print(pinocchio.__file__); print(vader5_sdk.__file__)'
```

CR5使用ROS pinocchio包；若缺失先核对目标Humble仓库可用性，再安装`ros-humble-pinocchio`。离线venv中包名是`pin`，不是另一个同名PyPI pinocchio包。ARM64、glibc或厂商native库不兼容应明确阻塞。

### B02 开发安装完整编译

```bash
bash scripts/build.sh --mode symlink 2>&1 | tee "$MORLA_EVIDENCE/build-symlink.txt"
source install/setup.bash
ros2 interface show morla_interfaces/msg/CommandEnvelope
ros2 interface show morla_interfaces/msg/ComponentTarget
ros2 interface show morla_interfaces/srv/RuntimeOperation
ros2 interface show morla_interfaces/srv/ControlAuthority
```

通过：全量构建退出0，无failed/aborted包；IDL可由Python导入、C++目标/接口生成完成，入口、launch和share文件存在。只完成Python import不算IDL/C++编译通过。

若全量被无关设备依赖阻塞，可额外跑以下**局部**编译定位，但全量B02仍未通过：

```bash
bash scripts/build.sh --mode symlink \
  --packages-up-to morla_cr5_driver morla_loophand_driver morla_epg_driver morla_vader5_input
```

Web的package.xml仍可能拉入其他旧机器人包；不能把`--packages-up-to morla_web_manager`称为完全无其他品牌依赖的最小构建。

### B03 普通安装

```bash
bash scripts/build.sh --mode regular 2>&1 | tee "$MORLA_EVIDENCE/build-regular.txt"
```

脚本普通安装到install_regular，并复用build目录/重配CMake。**开一个干净终端**，只source ROS和install_regular/setup.bash，不叠加先前install；重新设置MORLA_WS/MORLA_EVIDENCE等变量。

```bash
source /opt/ros/humble/setup.bash
source "$MORLA_WS/install_regular/setup.bash"
unset PYTHONPATH
# unset后再source同一安装树，避免来自源码测试的路径混入。
source "$MORLA_WS/install_regular/setup.bash"
cd /tmp
python3 -m morla_core.composition.cli catalog --json
ros2 pkg prefix morla_cr5_driver
ros2 pkg executables morla_vader5_input
ros2 launch morla_cr5_driver cr5_vr.launch.py --show-args
ros2 launch morla_vader5_input vader5_teleop.launch.py --show-args
ros2 run morla_cr5_driver cr5_kinematics_check --offline
```

通过：普通安装下从/tmp运行，不额外注入源码PYTHONPATH，Python模块来自install_regular而非src；SDK来自显式安装位置。检查模型、网页、配置、LICENSE/PROVENANCE、descriptor和entry point均被安装。测试机可进一步用**独立子shell**暂时重命名测试checkout的src再验证，测试后恢复；不移动生产源码，也不让后台进程依赖被改名目录。

### B04 资产闭包与模型加载

在已source目标安装的终端，回到仓库：

```bash
cd "$MORLA_WS"
python3 src/robot_description/scripts/check_assets.py \
  --require-regular-install 2>&1 | tee "$MORLA_EVIDENCE/assets-regular.txt"
python3 src/robot_description/scripts/check_assets.py \
  --require-regular-install --loaders 2>&1 | tee "$MORLA_EVIDENCE/assets-loaders.txt"
ros2 run morla_cr5_driver cr5_kinematics_check --offline \
  --joints 0 0 0 0 0 0 | tee "$MORLA_EVIDENCE/cr5-offline-ik.json"
```

--loaders需Pinocchio与MuJoCo均对当前Python可用。全库检查当前可能先报Nero target hash mismatch；保留FAIL及错误，CR5专项结果单独记录，不能合并成全库PASS。CR5离线FK→IK→FK通过也不证明物理标定。

### B05 colcon测试与CI

```bash
cd "$MORLA_WS"
colcon test --base-paths src --event-handlers console_direct+ \
  2>&1 | tee "$MORLA_EVIDENCE/colcon-test.txt"
colcon test-result --verbose 2>&1 | tee "$MORLA_EVIDENCE/colcon-test-result.txt"
```

colcon未注册的pytest不会自动执行，仍需第5节和ROS脚本。记录实际发现/运行/skip数量。检查`.github/workflows/composable.yml`的Python3.10/3.12、fake/sim和前端任务实际运行结果；工作流存在不等于CI已通过，亦未覆盖ROS硬件。

## 5. T01–T04 目标主机离线回归

独立venv仅运行源码/数学/数据测试，不拿它的成功代替ROS系统解释器测试：

```bash
cd "$MORLA_WS"
/usr/bin/python3 -m venv .venv-test
.venv-test/bin/python -m pip install -r dependencies/composable-test.txt
MORLA_PYTHON="$MORLA_WS/.venv-test/bin/python" \
  bash scripts/test_composable.sh --layer pure 2>&1 | tee "$MORLA_EVIDENCE/pure.txt"
MORLA_PYTHON="$MORLA_WS/.venv-test/bin/python" \
  bash scripts/test_composable.sh --layer fake 2>&1 | tee "$MORLA_EVIDENCE/fake.txt"
PYTHONPATH=src/common/vader5_sdk .venv-test/bin/python \
  -m unittest discover -s src/common/vader5_sdk/tests -v \
  2>&1 | tee "$MORLA_EVIDENCE/vader-sdk.txt"
.venv-test/bin/python -m pip install mujoco==3.3.7
MORLA_PYTHON="$MORLA_WS/.venv-test/bin/python" \
  bash scripts/test_composable.sh --layer sim 2>&1 | tee "$MORLA_EVIDENCE/sim.txt"
MORLA_DATA_PYTHON="$MORLA_WS/.venv-test/bin/python" \
  bash scripts/test_offline.sh 2>&1 | tee "$MORLA_EVIDENCE/data-offline.txt"
```

参考现有pure172/fake23/SDK5，源码未改却少测/skip要解释。EPG未安装用户SDK时，相关真实SDK录制总线测试会skip；其他pure通过不能覆盖这两项。需要该项时将原wheel装入**该测试venv**再复验。

T01对应pure（含CR5模型数学和Quest逻辑），T02对应fake/SDK，均不含真实DDS/USB；T03场景测试并未提供CR5/LoopHand物理场景；T04离线导出成功也未验证真实训练loader。

## 6. R01–R04 ROS fake及授权/资源隔离

回到干净ROS系统Python终端，只source选定安装树。`test_ros.sh`和`test_policy.sh`固定读取install/setup.bash，适合开发安装；不要在普通安装结果中混用它们。两类安装分别记报告。

```bash
cd "$MORLA_WS"
MORLA_TEST_ROS_DOMAIN_ID=91 bash scripts/test_ros.sh \
  2>&1 | tee "$MORLA_EVIDENCE/ros-suites.txt"
bash scripts/test_policy.sh 2>&1 | tee "$MORLA_EVIDENCE/policy-ros.txt"
```

test_ros.sh按测试组递增DDS域，并强制/usr/bin/python3。**它没有覆盖新CR5/LoopHand/EPG/Vader5的实际ROS驱动链**，需继续以下步骤。系统Python缺pytest/测试依赖时记环境阻塞，按实际需要补齐后复跑，不能回退venv结果充当DDS证据。

### R01 各独立驱动fake启动

先编译快照；本段compile只校验配置，不打开设备：

```bash
python3 -m morla_core.composition.cli compile \
  src/robots/dobot/morla_cr5_driver/config/assembly.fake.yaml \
  --output "$MORLA_EVIDENCE/cr5.fake.json"
python3 -m morla_core.composition.cli compile \
  src/robots/loophand/morla_loophand_driver/config/assembly.fake.yaml \
  --output "$MORLA_EVIDENCE/loophand.fake.json"
python3 -m morla_core.composition.cli compile \
  src/robots/jodell/morla_epg_driver/config/assembly.fake.yaml \
  --output "$MORLA_EVIDENCE/epg.fake.json"
ros2 launch morla_cr5_driver cr5_driver.launch.py \
  resolved_path:="$MORLA_EVIDENCE/cr5.fake.json"
```

另开终端观察；然后停CR5再**分别**启动LoopHand、EPG，替换launch与快照路径：

```bash
ros2 topic echo /cr5_driver/state --once
ros2 topic info /cr5_driver/commands --verbose
ros2 topic info /cr5_driver/arm/joint_states --verbose
ros2 topic hz /cr5_driver/arm/joint_states
# CR5退出后逐个运行，勿并发持有全局runtime锁：
ros2 launch morla_loophand_driver loophand_driver.launch.py \
  resolved_path:="$MORLA_EVIDENCE/loophand.fake.json"
# 退出LoopHand后再运行：
ros2 launch morla_epg_driver epg_driver.launch.py \
  resolved_path:="$MORLA_EVIDENCE/epg.fake.json"
```

对应state/topic分别为`/loophand_driver/hand/joint_states`、`/epg_driver/gripper/joint_states`。通过：fake来源synthetic=true、measured=false；CR5/LoopHand/EPG分别6/7/1维；EPG位置单位m，其余rad；启动不自动ACTIVE。

### R02 实际ROS服务与命令回执

提供[仅fake的ROS检查脚本](fake_ros_gateway_smoke.py)。必须在R01刚启动、尚未被手工激活的独立fake节点上运行；脚本核对节点实际resolved_path、单fake执行器及synthetic状态，拒绝real和组合配置。它通过真实ROS服务获取权限，发送合法小步目标并检查反馈，再验证重复序号、错token/epoch/session、过期/未来时间、错名称/单位、NaN和撤权后旧权限拒绝。**该脚本在文档编写机仅做静态/帮助检查，真实DDS执行仍为NOT_RUN**。

```bash
python3 docs/testing/fake_ros_gateway_smoke.py \
  --node /cr5_driver --component arm --resolved "$MORLA_EVIDENCE/cr5.fake.json" \
  2>&1 | tee "$MORLA_EVIDENCE/cr5-ros-gateway.txt"
# 对分别新启动的另外两个fake节点运行：
python3 docs/testing/fake_ros_gateway_smoke.py \
  --node /loophand_driver --component hand --resolved "$MORLA_EVIDENCE/loophand.fake.json"
python3 docs/testing/fake_ros_gateway_smoke.py \
  --node /epg_driver --component gripper --resolved "$MORLA_EVIDENCE/epg.fake.json"
```

脚本结束撤权但不关闭驱动。保存每次日志，不与手工服务测试并行；继续其他用例时重新启动干净fake节点。以下为手工定位流程：

仅对上述fake快照/节点执行。先取state中的session_id，替换下面SID；每个用例用唯一request_id。观察`/cr5_driver/receipts`、state.events和state.operations。

```bash
ros2 service call /cr5_driver/operation morla_interfaces/srv/RuntimeOperation \
  "{request_id: host-prepare-1, session_id: SID, operation: prepare, component_ids: []}"
ros2 service call /cr5_driver/operation morla_interfaces/srv/RuntimeOperation \
  "{request_id: host-activate-1, session_id: SID, operation: activate, component_ids: []}"
ros2 service call /cr5_driver/authority morla_interfaces/srv/ControlAuthority \
  "{session_id: SID, request_id: host-acquire-1, operation: acquire, producer: host_fake_test, resources: [arm], ttl_ns: 5000000000}"
```

服务RUNNING不是操作完成；在state.operations或同request_id的`operation: status`中等到SUCCEEDED。acquire保存返回token/epoch；5秒只是本次租约，超时须重新显式激活/获取。不要在终端手抄过期issued_ns再期待通过。

正命令用同一主机的rclpy客户端生成`issued_ns=time.monotonic_ns()`，在ttl内publish CommandEnvelope；fields使用刚取得的session/producer/token/epoch、递增sequence，target为`component_id=arm`、`names=[joint1..joint6]`、`units=[rad]*6`、`mode=position`、fake目标例如`[0,0,0,0,0,0.01]`。该客户端需等待publisher匹配后再发，**只用于fake，不把此固定关节向量用于真机**。消息字段以`ros2 interface show`为准，所有步骤都必须真实跨DDS，而非直接调用RuntimeSession Python对象。

逐项记录：有效目标accepted=true、dispatched含arm，随后fake反馈最后一维变为0.01；未授权/错token/错epoch/旧session/重复sequence/过期TTL/未来issued_ns/错关节名/degree冒充rad/NaN应accepted=false且dispatched为空。无授权负例可直接发布`{}`验证拒绝，其余仅改一个字段，避免命中另一拒绝原因。

重复同request_id的prepare/activate只执行一次；revoke后再次发旧token目标应拒绝。LoopHand、EPG用各自精确名称/单位和资源重复上述授权链。每条同时核对回执、事件和反馈；仅publish返回0不算通过。

### R03 进程互斥与退出清理

在fake阶段启动第一个runtime后，第二个独立驱动/组合宿主应在设备打开前报告资源忙。分别测试新runtime vs 新runtime、新runtime vs旧Nero/Astral入口、同一真实hidraw reader的两个实例；后者先在真实输入+fake执行器阶段进行。

正常退出第一个后重新启动应成功；异常关闭若停止/清理不确认，应保留故障/锁，不靠删锁重试。记录进程PID、资源key和错误，确认没有遗留子驱动/端口。对实体执行器的kill -9、拔线与掉电放在设备watchdog已确认后的专门用例，不在本段批量执行。

### R04 组合与部分故障

编译两个实际fake示例，检查CR5+LoopHand为13维、CR5+EPG为7维，组件顺序清晰：

```bash
python3 -m morla_core.composition.cli compile examples/cr5_loophand/assembly.fake.yaml \
  --output "$MORLA_EVIDENCE/cr5-loophand.fake.json"
python3 -m morla_core.composition.cli compile examples/cr5_epg/assembly.fake.yaml \
  --output "$MORLA_EVIDENCE/cr5-epg.fake.json"
ros2 run morla_runtime joint_state_runtime_node --ros-args \
  -p resolved_path:="$MORLA_EVIDENCE/cr5-loophand.fake.json"
```

该入口只负责runtime网关与反馈，不会自动运行输入映射/录制；完整teleop/capture用现有ComposableService宿主。不要把联合快照交给单设备驱动，另一个组合需停前一个后单独启动。

验证只向授权组件下发、同组部分失败撤权、停止各独立设备、请求幂等、释放后不发迟到IK结果。首次只在fake/故障桩做。**CR5+真实LoopHand目前不进入运动验收**；装配中TCP/安装变换也不能从示例推断。

若要验收公共层可选ros2_control原型，另在安装相应controller_manager/GenericSystem依赖后运行`ros2 launch morla_runtime ros2_control_mock.launch.py`，检查mock控制器激活、具名目标映射和状态；它只测试框架适配原型，不是物理仿真，也不将ros2_control加入CR5驱动。未使用该可选能力可注明范围后记NOT_APPLICABLE。

## 7. C01–C04 CR5迁移和独立真机验收

先阅读[CR5驱动](../../src/robots/dobot/morla_cr5_driver/README.md)和[VR指南](../integrations/cr5_vr.md)。不要同时运行原cr5_loophand_graspnet、厂商写入工具、独立cr5_driver和VR组合宿主。

**C01 身份/网络/只读反馈**：核对实际固件为当前TCP协议兼容版本；29999/30003/30004、6关节、反馈1440字节/魔数/设备时钟符合。第一次用只读探针：

```bash
ros2 run morla_cr5_driver cr5_kinematics_check \
  --host YOUR_CONTROLLER_IP --device-id YOUR_CR5_ASSET_ID \
  | tee "$MORLA_EVIDENCE/cr5-readonly.json"
```

通过：无EnableRobot、ServoJ、HOME写入；实测6关节rad与控制器显示逐轴一致；局部FK→IK→FK达到0.2 mm/0.002 rad，报告仍保持physical_base_tcp_verified=false。这个测试不检查实际TCP。需要对比控制器FK时才加--compare-controller-fk，并先声明两套base/tool定义。

**C02 物理模型/坐标对齐**：在厂商支持的低速维护流程取得多个已知安全姿态，不在奇异零姿态强行沿任意方向运动。记录每轴正负、零位、URDF与编码器映射、六轴实际限位、Link6到TCP、base_link安装方向和负载。模型关系为`q_URDF=signs*q_hardware+offsets`；用多个姿态对照TCP/外部测量，不能只做模型内部闭环。保留测量误差、方法和预先约定的物理容差。

**C03 独立低速控制**：从`config/assembly.fake.yaml`和`deployment.real.example.yaml`复制现场文件；填写身份、规范资源key、IP、joint_lower/upper（rad）。先从维护验收取得停止/设备watchdog证据，按事实设置verified/stop_verified/kinematics_verified等字段；read-only不自动使能。使用编译后的real快照启动独立cr5_driver，经operation→activate确认→authority→基于**当前实测关节**的小步目标；不要使用fake零目标。不需要VR即可先验证设备控制。

通过：单轴小步方向正确、未选轴无目标变化、实测跟随、限位/步长/速度拒绝正确，停止后旧token/epoch无效。起步速度和目标幅度由现场工况确定并记录，不把模板上限当成首测速度。

**C04 停止/失效**：空载支撑下分别验证撤权、软件急停、输入中断、反馈停更、控制网络中断、主机进程退出、设备重启。先测试可控的软件停止，设备watchdog及物理急停均能独立工作后再做断线/掉电。记录“检测→最后一次dispatch→设备停止确认”的时间与真实运动；DisableRobot的失重影响单独记录。重连/复位不得自动使能、重放或继续旧目标。代码host watchdog不是板端watchdog证据。

## 8. Q01–Q03 VR遥操CR5

### Q01 真实Quest + fake CR5

```bash
adb devices
adb reverse tcp:8000 tcp:8000
ros2 launch morla_cr5_driver cr5_vr.launch.py \
  store:="$MORLA_EVIDENCE/quest-store"
```

启动后才在Quest开始推流，打开主机127.0.0.1:8080。远程浏览器用SSH端口转发，不为测试直接公开控制端口。fake标识必须可见，8000仅一个接收器；不要用nero_web_start.sh来启动CR5。

- **右手柄**：准备完成→激活→握把先释放再按住；首次建锚不跳变，再验证三个平移和三个旋转方向。松开即撤权。
- **右手追踪**：开启手追踪及完整21点landmarks；先张开再捏合，用右手腕位姿运动。验证捏合中间区、手遮挡、缺点和非有限包，不把手势直接当作LoopHand目标。
- **来源与时效**：手柄↔裸手、Quest重定位/会话改变、停流/拔USB、重新连接时清锚；握把/捏合仍保持也不自动恢复。姿态与保持端口各自过期，新的手腕不能续旧捏合。
- **控制**：可达、不可达、奇异附近、快速输入跳变分别测；失败应撤权/保持错误，不追逐迟到worker结果。记录实际源频率、IK/FK耗时分位数、结果年龄和撤权原因。

### Q02 真实Quest + 真实CR5

仅C01–C04及Q01通过后，复制`quest_right.fake.yaml`为现场Assembly，复制`quest_right.real.example.yaml`为Deployment。填写关节映射/限位，binding的input_rotation、solver_tip_from_tool及calibration_verified，Deployment的真实身份与验证证据。它们不会由calibrations中的名字自动计算。

```bash
ros2 launch morla_cr5_driver cr5_vr.launch.py \
  assembly:=/absolute/path/to/quest_right.assembly.yaml \
  deployment:=/absolute/path/to/quest_right.deployment.yaml \
  store:="$MORLA_EVIDENCE/quest-real-store"
```

页面必须REAL，臂反馈hardware/measured=true。保持小范围低速，分别对**右手柄、右手追踪**重复Q01；每个方向确认TCP和基座变换。两种模式各自记录PASS，不能一项代另一项。求解应是cr5_urdf_pinocchio_dls，不应向控制器发送InverseSolution用于遥操。

### Q03 录制与恢复

两种输入各录短段，包含释放/再次激活与一次受控停流；先结束保存再停会话。核对机器人state/action各6维rad、输入来源、无跨源旧锚点、无fake/真机混标。日志不能只留下“画面能动”，需反馈/回执和数据文件证据。重启应用/ROS/浏览器后默认disarmed。

## 9. L01–L03 LoopHand迁移

**L01 fake/安装**：独立构建并执行R01/R02；无需安装CR5。核对7个紧凑关节名称与rad，16关节展开是耦合模型，不是16个独立主动/实测自由度。

**L02 真实只读**：复制`morla_loophand_driver/config/deployment.real.example.yaml`到现场目录，填写资产身份和稳定/dev/serial/by-id串口，保持验证标志false。先确认厂商工具已退出，波特率921600与实物匹配：

```bash
python3 -m morla_core.composition.cli compile \
  src/robots/loophand/morla_loophand_driver/config/assembly.fake.yaml \
  --deployment /absolute/path/to/loophand.deployment.yaml \
  --output "$MORLA_EVIDENCE/loophand.real.json"
ros2 launch morla_loophand_driver loophand_driver.launch.py \
  resolved_path:="$MORLA_EVIDENCE/loophand.real.json"
```

核对GET_POS编码器反馈、帧长/短包/超时、手型变化方向；框架输出source=estimated、measured=false。启动不homing、不改力矩。按状态判断，不因JointState有数值就标实测关节。

**L03 阻止真实运动**：发activate请求应失败并说明stop/watchdog未验证；将此“拒绝保护”记PASS，运动能力记BLOCKED_CAPABILITY。CTRL_TOR无执行ACK，STOP_UNCONFIRMED不能删锁或改true吞掉。未来需要补固件确认停止/禁能、通信watchdog，再测断线/退出/重上电与低速运动；触觉、VR/手套重定向和物理模型分别作为新接入，不计入本轮通过。

## 10. V01–V04 Vader5迁移与遥操

见[输入包说明](../../src/common/morla_vader5_input/README.md)。SDK应在src/common/vader5_sdk，与morla_vader5_input同级，import来自显式pip安装，不依赖tools或临时PYTHONPATH。ROS包自身不应import CR5/Nero/Astral来发现设备。

**V01 真实手柄输入诊断**：USB数据线、PC模式、正确厂商hidraw权限；关闭Steam、飞智配置工具和第二个SDK reader。检查SDK README的精确权限规则；多个设备不得自动猜测。

```bash
ros2 launch morla_vader5_input vader5_input.launch.py device:=/dev/hidrawN
```

另终端观察/vader5/events、/vader5_input/joy、/vader5_input/imu、/vader5_input/status，保存实际rosbag或原始JSON。逐个按下/放开UP RIGHT DOWN LEFT A B X Y SELECT START LB RB LT RT L3 R3 C Z M1 M2 M3 M4 LM RM FN TURBO HOME，核对独立位、短按和同按；TURBO是约50ms脉冲，不能当保持键。检查四摇杆轴满行程/回中、LT/RT范围、gyro三轴rad/s、accel三轴m/s²含重力；orientation未知。USB与2.4G各自有独立结果，未测另一种就保留NOT_RUN。

**V02 真实手柄 + fake CR5/EPG**：先退出单独reader，launch会拉起reader，不得开两份。复制input.real.example.yaml填写资产身份，仅pad改real，arm仍fake。

```bash
ros2 launch morla_vader5_input vader5_teleop.launch.py \
  device:=/dev/hidrawN fake_input:=false \
  deployment:=/absolute/path/to/input.real.yaml \
  store:="$MORLA_EVIDENCE/vader-fake-store"
```

先在Web准备，再测试START激活；M1先释放再按下才控制，M2切平移/旋转，LT从1到0.2精细倍率，RT为闭合比例。验证六方向、回中、M1释放、B暂停、Y开始录制/X结束保存、START+SELECT优先急停。启动已按住Y/START不触发；放开SELECT不补激活；一个按键只执行一次操作，capture中能找回短按边沿。

EPG fake用`assembly:="$MORLA_WS/src/common/morla_vader5_input/config/epg.fake.yaml"`，真实输入仍用同一仅pad的覆盖文件；RT对应0.05至0m，带0.05m/s目标变化率。确认首次deadman无目标跳变。Nero/Astral配置示例单独compile并测试；它们可编译不等于实物已跟随。

**V03 故障/策略**：拔USB/关手柄/停reader、批次短按/释放、重启reader后epoch改变、M1持续按住重连，均需旧权失效、无自动恢复。持续输入时暂停模型/切policy owner，普通摇杆/M1不抢权或额外跑人工IK，B/急停仍是显式操作。ROS事件RELIABLE/depth64的实际匹配和积压行为需实测，不能只看50Hz发布目标。

**V04 真实执行器**：相应C01–C04或EPG实体验收、V01–V03通过后，复制cr5.real.example.yaml与cr5.fake.yaml为现场配置，填足身份、限位、输入basis/TCP及标定声明；用上述launch的assembly/deployment参数运行。右手柄速度点动与Quest绝对追踪分别验收；CR5 jog仍用自己的IK，接受精度10µm/0.0001rad，需测求解耗时和微小输入响应。Nero/Astral同样需各自真实部署/坐标/停止证据，不把更换一个plugin当作已验收。

## 11. E01–E03 EPG50-060完整型号

型号EPG50-060-0PF-L200-C7-N-A-P40-S00；先核对实物定制后缀、接口和接线，不由通用族手册猜测。把用户原厂JodellTool0.1.5 wheel提供给测试主机，按[驱动README](../../src/robots/jodell/morla_epg_driver/README.md)安装；不从同名网络包替代。

**E01 只读probe**：RS485 A/B/地/24V按实物资料，确认真实站号和波特率。probe持有锁，只读状态：

```bash
ros2 run morla_epg_driver epg_probe \
  --port /dev/serial/by-id/YOUR_USB_RS485 \
  --device-id YOUR_EPG_ASSET_ID --slave-id YOUR_ID --baudrate 115200
```

**E02 状态与初始化**：验证SDK位置0打开/255闭合和真实宽度方向，0.05m只是名义行程；外部测量/受控阻挡区分设定回读与实际位置，才能声明position_feedback_verified。电流标幺不是N/A，速度回读不是实测m/s；温度/电压/状态字节序逐项核对。IO模式应拒绝而不自动改模式。初始化可能寻程，保持工件/人员远离行程，再核对新状态确认。

**E03 停止/watchdog及real控制**：验证0x03E8 rSTOP=0x0005的实际固件行为与gACT/gSTA/gGTO/故障确认；不能因为SDK.stopClaw写0x03FF就改回未确认地址。通信丢失1秒故障不自动等于物理停止。完成这些证据后，才填写真实Deployment及activation_motion_verified/position_feedback_verified/device_watchdog_verified，做低速宽度目标、夹持/抱闸、取消迟到初始化、掉线与恢复。再做CR5+EPG的7维录制或Vader5 RT控制；安装TCP/碰撞和夹持力需独立测量。

## 12. N01–N02 原有Nero/Astral与手部回归

旧链路曾成功的硬件与当前链路分开记录：

- **N01 Nero**：已有SDK/CAN初始化、enable确认、J2/左右镜像、7关节顺序、J7、HOME/工作位、VR右/左/双臂、XHand12关节与触觉身份；在新runtime中验证真实反馈/时效/权限/停止及与旧piano/维护入口互斥。先解决影响该模型的哈希不一致。独立装配的预期state/action从resolved读取，不将固定14维套到所有配置。
- **N02 Astral**：一个板级会话下左右臂/头腰作用范围，选择右臂不向未选关节发目标，auto_ready/full-body默认保护、stop/enable作用范围；夹爪command_echo不得冒充实测。VR和Vader分别验证。Wuji20关节/XHand12关节的实物左右手、重定向、触觉和总线也需各自记录。

可先跑test_ros.sh既有套件；实物阶段保持对应驱动/标定原参数，比较同设备同负载新旧路径的目标/反馈/延迟。通过旧Profile脚本只算legacy回归，不算新Assembly/Deployment/runtime路径验收。

## 13. S01–S03 相机、其他输入和传感器

**S01 相机**：0/1/3路配置分别测试；UVC绑定/dev/v4l/by-id和对应device_id，ROS image绑定明确topic/encoding/QoS。交换USB口、同型号两台相机重插后语义角色不能互换；核对分辨率/曝光/fps、黑帧/坏JPEG/重复帧/源时间、CameraInfo与optical frame、断流/重连。关闭Web预览后仍可采集；开预览/录盘/推理压力下运动通道不被阻塞。

当前UVC时间是主机完成read时间，不能当曝光时刻；组合Web录制尚未完整透传所有相机source_ns/fingerprint元数据。需要严格同步时先补完整接线与HDF5证据，再标同步验收PASS。RGB通过不覆盖深度/点云/双目/力传感器；这些能力若未实现记BLOCKED_CAPABILITY。

**S02 USB脚踏/语音/手机/Xbox/手套**：按实际已有设备逐个测试输入身份、按下/松开/短按/同时操作、重复start/stop幂等、失联与恢复。旧morla_operator_control脚踏/语音主要走旧Profile路由，不因新Vader接入就自动接入新组合Service；先核对实际路由，未接通则明确列为待接入。不要同时让旧/新两个路由响应同一按钮。手机锁屏/切后台/touchcancel/断网独立测试，手机连接方式按实际配对/HTTPS及网络部署另记。

**S03 来源/功能范围**：力、触觉、IMU等确认shape/names/unit、标定/饱和/缺样/时间域；未测值标invalid，不补零装实测。跨设备脚踏deadman+VR位姿需要显式多源聚合，不能直接引用另一input的deadman绕过同源约束。尚未接入的PS/NS/国产手柄、其他VR、外骨骼等记NOT_APPLICABLE或BLOCKED_CAPABILITY，并说明设备/实现情况，不冒称通用支持已实测。

## 14. D01–D03 数采、导出、训练、推理与回放

**D01 实际录制**：先fake/真实输入，再独立真机；每种适用装配录至少一段正常运动和一段主动失联。确认SchemaV3、独立state/action名字/顺序/单位、resolved与geometry身份、source/validity/synthetic、raw输入/按钮边沿、相机角色与时间、command action和measured state的区别。无有效动作的command监督片段应failed/incomplete，不能保存成训练成功段。

CR5单臂6/6、CR5+EPG7/7、CR5+LoopHand fake13/13；Vader本例端口是auxiliary，不自动增加模型action维度。模型使用输入意图作观测时需显式改变capture/feature契约与训练配置。

**D02 在工作副本处理数据**：Web/API返回真实episode路径，从其父session目录选择测试源；不要假定旧Profile目录布局。设MORLA_DATA_PYTHON为具备numpy/h5py/cv2/pyarrow/av的解释器，复制到新目录后处理，原始文件hash前后一致：

```bash
export MORLA_DATA_PYTHON=/absolute/path/to/data/python
bash "$MORLA_WS/scripts/data_pipeline.sh" prepare --help
bash "$MORLA_WS/scripts/data_pipeline.sh" validate --session /absolute/path/to/work-session
bash "$MORLA_WS/scripts/data_pipeline.sh" align --session /absolute/path/to/work-session
bash "$MORLA_WS/scripts/data_pipeline.sh" export-openpi \
  --session /absolute/path/to/work-session --output /absolute/path/to/new-lerobot-v21
bash "$MORLA_WS/scripts/data_pipeline.sh" export-act \
  --session /absolute/path/to/work-session --output /absolute/path/to/new-act-v3 \
  --check-python /absolute/path/to/locked-lerobot/python
```

各命令参数用当前--help核对。validate可能写报告、align新建aligned；不要对原始唯一副本执行--apply/--overwrite/--force。检查Parquet/HDF5/视频帧数、丢帧、独立动作布局、source/所有权标签；缺相机或无图像任务按所选模型能力处理，不制造黑图。

**D03 真实后端**：锁定ACT/LeRobot/OpenPI版本和backend_revision，用实际数据各跑loader取batch、归一化stats、少步训练、保存/重载checkpoint、一次真实predict；然后先对fake机器人走runtime policy/回放。后端环境与ROS隔离，用ModelManifest/TrainingSpec记录feature、geometry、normalization、camera_map、动作语义。仅`morla_learning.cli`生成配置或假预测器通过不算真实训练/推理通过。

故意换同维不同关节顺序/单位/左右臂、错TCP、旧checkpoint或camera_map时应握手拒绝。策略失联/过期动作按协议停写，人工手柄不自动抢权。真实执行器推理/回放在该设备手动链与停止验收完成后另做小范围测试；Rerun可视化回放不等于机器人执行回放。

## 15. W01–W02 Web与主机端到端

在独立测试checkout构建前端，保存Node/npm版本与lock：

```bash
cd "$MORLA_WS/src/common/morla_web_manager/web"
npm ci
npm run build 2>&1 | tee "$MORLA_EVIDENCE/web-build.txt"
node --test test/*.test.cjs 2>&1 | tee "$MORLA_EVIDENCE/web-node-tests.txt"
```

源码web/dist重建后，需重新安装morla_web_manager或核对实际服务的是哪份dist；新JS不应被旧安装覆盖。浏览器强制刷新，实际检查组合组件发现、编辑/校验、revision保存、预检、prepare/activate、手机配对、采集保存、急停/复位/停止，刷新/两个浏览器不能授予多writer。

仓库`web/test/composable_browser.mjs`需要单独Playwright+Chromium，不在package.json依赖内。可在测试工具目录安装并记录版本，以NODE_PATH提供给脚本；Linux显式设置BROWSER_EXECUTABLE为其Chromium路径，避免脚本的macOS备用路径。先启动仅fake的隔离Composable API（例如127.0.0.1:18082，独立store/resource_root），再以BASE_URL/BROWSER_OUTPUT执行脚本。这个脚本会创建fake会话/配置，**不要对生产或已有real会话的API执行**。截图、result.json、浏览器错误及API响应一并保存。

API终端（已source安装、MORLA_EVIDENCE已设置）：

```bash
python3 - <<'PY'
import os
from pathlib import Path
import uvicorn
from morla_web_manager.composable_api import create_app
root = Path(os.environ['MORLA_EVIDENCE'])
app = create_app(str(root / 'browser-store'), resource_root=str(root / 'browser-locks'))
uvicorn.run(app, host='127.0.0.1', port=18082)
PY
```

浏览器终端，在web目录运行；安装版本由工具目录package-lock及npm输出留证，之后复测复用同一lock：

```bash
npm install --prefix "$MORLA_EVIDENCE/browser-tools" playwright
"$MORLA_EVIDENCE/browser-tools/node_modules/.bin/playwright" install chromium
export NODE_PATH="$MORLA_EVIDENCE/browser-tools/node_modules"
export BROWSER_EXECUTABLE="$(node -e 'console.log(require("playwright").chromium.executablePath())')"
BASE_URL=http://127.0.0.1:18082 BROWSER_OUTPUT="$MORLA_EVIDENCE/browser" \
  node test/composable_browser.mjs 2>&1 | tee "$MORLA_EVIDENCE/browser-e2e.txt"
```

W02还需实际Quest/Vader链路页面、录制与故障提示的人工检查；浏览器用例不覆盖USB、硬件运动或真实模型。

## 16. P01–P03 性能、长稳、恢复与发布

**P01 频率与延迟**：逐层采样输入设备→ROS→normalizer→controller/IK/FK→runtime dispatch→SDK→硬件反馈→recorder。区分目标Hz、topic发布Hz、源样本Hz和缓存重复；`ros2 topic hz`不能测IK/FK或设备执行延迟。对每段报告p50/p95/p99/max、源年龄、超期拒绝、队列峰值、丢帧、CPU/RSS。频率/参数清单见[现有核对](../integrations/teleoperation_rates_and_parameters.md)。

先单臂/单输入，再加实际相机、录盘、Web预览和模型负载；同主机同配置比较新旧路径。[VALIDATION第12节](../architecture_refactor/VALIDATION.md)的p99路由开销、相对增幅、至少8小时软件长稳等是**拟定验收门槛**，不是已测数据；按现场设备/任务预先固定适用预算，不用放宽TTL掩盖超时。

**P02 长稳/故障恢复**：先fake长稳，再对已验收的单硬件链逐步延长，记录至少8小时软件集成结果及真实链实际时长。周期性切录制段/开关预览/暂停模型/重连输入，确认无持续内存增长、无旧目标积压、无未授权dispatch。硬件故障注入须按各设备C04/E03流程；停止未确认则阻塞，不自动恢复ACTIVE。

**P03 回退/发布与扩展**：保存已验收commit、普通安装、配置revision、SDK/native依赖、设备固件与标定引用；先在fake演练退回上一已验收版本，确认旧数据只读可加载，不原地降级Schema或覆盖标定。验证重启/回滚不自动使能。由未参与实现的人按接入指南增加一个独立fake执行器/传感器/输入，完成发现→编译→运行→采集→回放；独立扩展考试未做就保留NOT_RUN。

## 17. 每轮交付与通过标准

使用[测试报告模板](host_acceptance_report_template.md)逐项登记本文件ID；附原始命令/退出码/日志/rosbag/截图/配置摘要/异常工单/复测结果。不能只交一张“全部打勾”的截图。

- 编译通过：B01–B05适用项通过，解释每个skip/环境阻塞。
- 某设备迁移通过：安装、ROS fake、真实只读/身份/来源与该设备适用测试均有证据；运动阻塞设备只声明只读能力通过。
- 某遥操链通过：指定输入模式+具体机器人+固件+配置+停止行为+录制及实际性能分别通过，换输入/设备不沿用结论。
- 重构整体完成：除本手册外，[VALIDATION](../architecture_refactor/VALIDATION.md)适用编号、实际后端、长稳、回退与独立扩展均闭环。本文未实施的功能保持能力阻塞。

出现未授权运动、停止失效、数据静默错位/破坏或模型语义错配，停止该组测试并记录FAIL；修复后用同用例复测。软件测试、Git推送、页面能打开都不能替代这一组现场证据。
