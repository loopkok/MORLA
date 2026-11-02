# Composable runtime

`RuntimeSession` consumes a compiler-produced `resolved_system.json`. The
snapshot is deep-copied and loaded as-is: `resolved_hash` is emitted by the
compiler but **not verified**, and the former `allow_unverified_test_fixture`
gate no longer exists. Schema/contract validation of the assembly and
deployment was removed at the same time.

```python
from morla_runtime import RuntimeSession
session = RuntimeSession(resolved, registry=trusted_catalog)
print(session.preflight())  # Metadata only; never imports a vendor SDK or enables.
session.start()            # Builds adapters, then READY_DISARMED.
session.prepare("prepare-1")
operation = session.activate("activate-1")
# Native/simulation operations may return RUNNING. Poll session.operation(request_id).
permit = session.acquire("human", resources=["arm_a", "arm_b"])  # Records a lease only.
receipt = session.submit({"sequence": 1, "commands": targets})
session.revoke("human")    # Stops adapters and returns READY_DISARMED (or ESTOP_LATCHED).
session.stop("stop-1")
```

A target has `component_id`, exact ordered `names`, finite `values`, exact
`units`, and declared `mode`. Transport dispatch is distinguished from physical
execution acknowledgement. Never assume a successful ROS publish proves motion
or arrival. All deadlines use the host monotonic clock; simulation time is
recorded separately.

Native prepare/activate and declared home/workpos/disable operations execute
outside the runtime control-plane lock. A cancelled/late enable cannot transition
the system to ACTIVE (generation fencing is retained). Native HOME/workpos requests
return DISPATCHED with explicit pending operator confirmation; the legacy drivers'
measured, staged trajectories are preserved. Dispatch is not completion, and stages
are not automatically advanced. `reset` stops the adapters, clears the estop latch
and returns READY_DISARMED; it no longer requires fresh state or a prior stop ACK.

## Shared control host and process ownership (2026-10-30)

`morla-runtime resolved.json --inputs` uses ControlHost/BindingEngine; the ROS
`system.launch.py` entry accepts `connect_inputs:=true` and exposes runtime services.
The ordinary CLI itself does not expose those services or start a recorder. Web and
headless hosts share target generation and operator action routing; a control-only
host must inject recording callbacks to handle recording actions.

Input bridges register `morla.ros_inputs`; acquisition plans register `morla.processes`.
`auto` owns selected registered acquisition plans, `external` subscribes without
owning an existing process, and `managed` requires a plan. Shared key/argv/env are
started once; conflicts are reported. Input process state and native health process
state expose owned PID/exit codes. SDK adapters can remain in-process.

Native Nero/Astral/XHand/Wuji plans and ROS transports now live in the installed
`morla_native_bridge` device package. This package's `native_ros.py` only re-exports
compatibility names. Runtime loads `plugin_id:real` factories, whose context includes
all resolved `instances` and `shared_device_sessions`; Astral arm/gripper factories
reuse one physical session. Missing registration reports an unavailable backend.

**UR11 remains open:** ROS `operation(stop)` stops Runtime actuators but does not
immediately close ControlHost inputs; exiting the host closes them. Web stop closes
its input manager directly. A STOPPED actuator state is not proof all input PIDs exited.

Current scope, tests and target-host gaps: [implementation](../../../docs/architecture_refactor/UNIFIED_IMPLEMENTATION.md),
[host acceptance](../../../docs/testing/host_build_and_hardware_acceptance.md).

## Backends

- `fake`: registered protocol reference arm/input/sensor/camera adapters. JPEG
  samples and numeric samples explicitly carry `source=fake`, `synthetic=true`,
  `measured=false`. This is not physical simulation.
- `sim`: a single shared MuJoCo scene built from the installed
  `robot_description/scripts/composable_scene.py`. Explicit canonical-to-MJCF
  mappings drive joint and actuator indices. A dedicated stepping worker samples
  state. Optional offscreen cameras use a separate graphics worker and separate
  MjData snapshot. Missing graphics context produces SIM_RENDERING_UNAVAILABLE;
  no fake image is substituted. Collision and inertia claims retain the builder's
  validation level. Physical real/fake actuators cannot enter this same scene.
- `real`, Nero/Astral/XHand/Wuji: installed morla_native_bridge factories own ROS driver argv, session-private
  command/armed topics, fresh measured feedback, bounded asynchronous services,
  and owned process groups. Astral logical ports share one board connection;
  selected sides still route through the driver. The former board-scope gate,
  deployment/watchdog verification gate and Nero host-side sequence correlation
  were removed; service ACKs are used as the driver reports them.
- `real`, camera: ROS Image/CompressedImage providers and UVC worker acquisition.
  UVC requires an explicit existing `/dev/v4l/by-id/...` path and matching
  `device_id` basename. A ROS image source requires an explicit topic and reports
  fingerprint confidence as deployment metadata. It does not infer serial identity
  from a topic name. Capture has no dependency on browser preview.

Astral's current gripper feedback remains a command-angle echo. The bridge converts
it to closure ratio but labels `source=command_echo`, `measured=false`, `ready=false`.
The former activate gate that rejected this combination is gone, so the label is now
the only warning; it must not be read as measured feedback. No physical device,
board watchdog, firmware, ROS build, or offscreen GPU context is certified by
offline tests.

## Third-party adapters

Device packages remain independent: each owns its descriptor, protocol/SDK,
adapter factory, deployment configuration, launch entry point and tests.
`CachedActuatorAdapter` in `cached_actuator.py` provides optional brand-neutral
cached single-thread I/O. A device subclass supplies its transport, configuration
validation and feedback provenance; this helper imports no robot or hand packages.
Its former host command watchdog, target step/limit validation and activation gate
were removed, so a stalled teleop producer leaves the last target in effect. It
does not certify hardware stop.

The generic `joint_state_runtime_node` uses the same ROS command/operation/
authority gateway as `runtime_node`, with per-actuator `~/<instance>/joint_states`
outputs for fresh samples. Independent device wrappers still pass a
`required_plugin` name, but that check is no longer enforced. Multi-device
assemblies use the generic runtime; they do not require a new combined device
driver package.

Install a static component descriptor through `morla_core`'s catalogue mechanism
and register an installed Python entry point. AssemblySpec never supplies an import
path or command:

```toml
[project.entry-points."morla.adapters"]
"vendor.arm:real" = "vendor_morla_adapter:create_adapter"
```

`create_adapter(instance, context)` returns an object implementing
`configure(context)`, nonblocking cached `health(now_ns)`, `close()`, and for
actuators `prepare()`, `activate()`, `submit(target)`, `stop()`.
Optional `read(now_ns)` returns `ports` samples; `operate(name)` handles only
operations declared by the descriptor. The former `deployment_verified` /
`watchdog_verified` adapter attributes are no longer part of the contract and do
not gate activation.

Context contains session ID, monotonic clock, resource root, inherited lane fd,
immutable device-session metadata, and a per-runtime `shared_device_sessions`
registry for physical connection reuse. Only selected entry points load, after
host/device locks are held. Duplicate selected entry points fail. SDK reads/writes
that can block belong in an `AdapterWorker` or owned driver process; cached health
must never call a vendor SDK synchronously. `device_scoped_stop=True` is allowed
only if stopping one logical port really stops its entire physical session.

## Locks and isolation

New runtime takes an exclusive host lane. Standard legacy Nero/Astral/XHand/Wuji
entry points take the shared legacy lane before opening hardware. Owned children
inherit the runtime's locked file description and validate its inode; an environment
string alone is insufficient. Multiple legacy component drivers may coexist, while
new and legacy sessions exclude each other. A failed/uncertain close retains the
runtime's host lock. This prevents accidental writers; use ROS security and host
permissions for hostile participant isolation.

Independent devices receive stop requests concurrently. SDK worker timeouts fence
and terminate only the owned worker. Terminating a process is not a hardware stop:
the runtime no longer requires a verified board/driver watchdog before activation,
so that evidence is an operator responsibility. No stop request writes all-zero
joint targets.

**Soft release vs power-down.** `revoke()`/`disarm` (and the `switch` handover) use
`adapter.release()` when the adapter provides it: output stops, but the device keeps
its enable state and holds its last target, so a following `activate()` does not
re-enable a gravity-sagged arm. `estop()`, `stop()`, `reset()` and failure
compensation always call `adapter.stop()`, which is the device's power-down path
(Nero/Astral `~/driver/estop`, CR5 `DisableRobot`). Built-ins: `FakeAdapter` records
a `release` call, the MuJoCo adapter's `stop` leaves the last command in `ctrl`,
`NativeRosAdapter.release` lets the driver's command watchdog issue one hold, and
`CachedActuatorAdapter.release` rewrites the last commanded target (else the measured
pose) every cycle so a streaming servo keeps holding; it falls back to `stop()` when
feedback is stale or faulted. Holding a *measured* pose instead of the target hands
back the gravity droop and sags a little on every release.

## Validation

Tests under `test/` cover lifecycle/idempotence, resolved round-trip, estop latch
and reset, operation fencing, group partial failures, missing ACK, feedback
freshness, stop retry, cancellation/late enable, native shared-board transport,
lane contention/delegation, worker containment, selected plugin loading, camera
timestamp and provenance, and actual mixed-brand single-scene MuJoCo state/command
mapping. Authority/sequence/TTL enforcement and health-based group stop were
removed, so tests assert the relaxed behavior.
ROS transport and IDL require a target ROS Humble build; this macOS host has no
rclpy. Real hardware, UVC camera, and offscreen graphics validation remain separate
deployment gates.

### 公共仿真模式与显示选择（2026-11-02）

CR5/Nero/Astral 公共仿真支持运动学与物理动力学，并可选择网页、本地 MuJoCo viewer 或同时显示。CR5 新增独立 `cr5_mujoco_sim` 包；Nero/Astral 保留旧入口并新增公共 launch。配置、运行时切换和限制见 [公共仿真说明](../../../docs/integrations/SHARED_MUJOCO_2026-11-02.md)。

### 公共遥操运行诊断日志（2026-11-02）

公共 Runtime 每次启动默认自动保存输入、控制映射/IK 报告、命令回执、反馈与生命周期，不依赖数据采集。Web「工作站 → 检查与运行」可配置并查看/下载历史日志。Web 默认保存于 `<store>/logs/`，直接 Runtime 默认 `~/.local/state/morla/teleop/`，可用 `--log-dir` 或 `MORLA_DIAGNOSTIC_LOG_ROOT` 指定。日志异步写入，有丢弃和失败计数；反馈是缓存采样，旧独立 Profile 入口没有自动迁移。文件格式、配置、公共扩展接口和完整性边界见 [公共日志说明](../../../docs/integrations/COMMON_TELEOP_DIAGNOSTICS_2026-11-02.md)。
