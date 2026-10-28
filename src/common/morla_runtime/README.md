# Composable runtime

`RuntimeSession` consumes a compiler-produced immutable `resolved_system.json`.
The hash and trusted recompilation are mandatory. An explicit
`allow_unverified_test_fixture=True` escape hatch exists for injected offline
unit-test fixtures; production CLI, ROS and Web never set it.

```python
from morla_runtime import RuntimeSession
session = RuntimeSession(resolved, registry=trusted_catalog)
print(session.preflight())  # Metadata only; never imports a vendor SDK or enables.
session.start()            # READY_DISARMED, or PREPARING pending actual samples.
session.prepare("prepare-1")
operation = session.activate("activate-1")
# Native/simulation operations may return RUNNING. Poll session.operation(request_id).
permit = session.acquire("human", resources=["arm_a", "arm_b"])
receipt = session.submit({**permit, "sequence": 1, "issued_ns": session.clock(),
                          "ttl_ns": 100_000_000, "commands": targets})
session.revoke("human")    # Stop, fence authority, return disarmed after ACK.
session.stop("stop-1")
```

A target has `component_id`, exact ordered `names`, finite `values`, exact
`units`, and declared `mode`. Transport dispatch is distinguished from physical
execution acknowledgement. Never assume a successful ROS publish proves motion
or arrival. All deadlines use the host monotonic clock; simulation time is
recorded separately.

Native prepare/activate and declared home/workpos/disable operations execute
outside the runtime control-plane lock. A cancelled/late enable cannot transition
the system to ACTIVE. Native HOME/workpos requests return DISPATCHED with explicit
pending operator confirmation; the legacy drivers' measured, staged trajectories
are preserved. Dispatch is not completion, and stages are not automatically
advanced. Reset requires fresh state and a new successful stop acknowledgement.

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
- `real`, Nero/Astral/XHand/Wuji: trusted registered ROS driver argv, session-private
  command/armed topics, fresh measured feedback, bounded asynchronous services,
  and owned process groups. Astral logical ports share one board connection;
  selected sides are guarded, auto_ready/full-body/head are disabled. Board-level
  operations require verified board scope and unselected-joint policy. Physical
  activation/prepare require explicit deployment and watchdog verification.
  Nero service/status ACKs correlate exact driver-generated sequence numbers.
- `real`, camera: ROS Image/CompressedImage providers and UVC worker acquisition.
  UVC requires an explicit existing `/dev/v4l/by-id/...` path and matching
  `device_id` basename. A ROS image source requires an explicit topic and reports
  fingerprint confidence as deployment metadata. It does not infer serial identity
  from a topic name. Capture has no dependency on browser preview.

Astral's current gripper feedback is a command-angle echo. The bridge converts it
to closure ratio but labels `source=command_echo`, `measured=false`, `ready=false`.
That real combination cannot become ACTIVE until a measured or explicitly validated
feedback capability is implemented. No physical device, board watchdog, firmware,
ROS build, or offscreen GPU context is certified by offline tests.

## Third-party adapters

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
operations declared by the descriptor. Actuators also expose
`deployment_verified` and `watchdog_verified`; they cannot bypass runtime checks.

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
physical activation still requires a verified board/driver watchdog and stop policy.
No stop request writes all-zero joint targets.

## Validation

Tests under `test/` cover lifecycle/idempotence, exact contracts, fencing, epochs,
monotonic deadlines, group partial failures, missing ACK, stale feedback, stop
retry/reset, cancellation/late enable, native shared-board transport, lane
contention/delegation, worker containment, selected plugin loading, camera timestamp
and provenance, and actual mixed-brand single-scene MuJoCo state/command mapping.
ROS transport and IDL require a target ROS Humble build; this macOS host has no
rclpy. Real hardware, UVC camera, and offscreen graphics validation remain separate
deployment gates.
