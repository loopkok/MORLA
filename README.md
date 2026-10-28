# MORLA robot platform

2026-10-27 用户曲谱归档与三仓库提交说明见[详细日志](docs/piano/DEBUG_LOG_2026-10-27.md)。本体本轮仅文档更新，现场标定和Web曲谱库仍需独立备份。

2026-10-23 钢琴调速、曲谱与双仓库更新说明见[本轮详细日志](docs/piano/DEBUG_LOG_2026-10-23.md)。包含本次软件复验、历史现场证据与未完成验收边界。

MORLA is the unified ROS 2 Humble workspace for Astral, Nero, Wuji Hand, and XHand. The software migration through P10 is complete: common input, profile-driven robot assembly, description assets, collection/pipeline, policy/replay/HITL, and Web operation/monitoring are integrated. Hardware and deployed-model acceptance is tracked separately as P11; no software simulation result implies real-hardware acceptance. See [`docs/migration_progress.md`](docs/migration_progress.md) for evidence and remaining deployment conditions, [`docs/MIGRATION_PLAN.md`](docs/MIGRATION_PLAN.md) for contracts, and [`docs/SOURCE_BASELINE.json`](docs/SOURCE_BASELINE.json) for upstream provenance.

## Workspace layout

Recent Nero/XHand piano integration changes are recorded in [CHANGELOG](CHANGELOG.md) and the [2026-10-20—22 piano debugging log](docs/piano/DEBUG_LOG_2026-10-20_22.md). The independent engine lives in sibling `MORLA-piano`; deployed calibration data is stored separately in `.morla_piano/`. See the log for current behavior, parameter snapshots and hardware-validation limits.

Single-operator voice (host/Quest microphone), existing controller buttons, and USB pedal inputs share the operator command router. See [operator control](docs/operator_control.md) for setup, commands, and the Quest Windows build workflow.

- `src/common/`: shared input, collection, policy, web management, and bringup packages.
- `src/robots/astral/`, `nero/`, `wuji/`, and `xhand/`: robot and hand packages grouped by platform.
- `tools/legacy/`: archived utilities excluded from ROS package discovery.
- `docs/legacy/astral/`: original Astral workspace entry documents for reference.

## Build

Use the system Python that matches ROS 2 Humble; do not build ROS packages under a conda Python. From the SDK root:

```bash
cd MORLA
bash scripts/build.sh --mode symlink
source install/setup.bash
python3 -m morla_core.profile validate-all
```

The reusable checks are `scripts/test_ros.sh`, `scripts/test_offline.sh`,
`scripts/test_policy.sh`, `scripts/test_sim_integration.sh`, and
`scripts/test_astral_sim_integration.sh`. Simulation checks write synthetic
raw and exported data under `/tmp` by default; set `MORLA_TEST_OUTPUT_ROOT` or
`MORLA_ASTRAL_TEST_OUTPUT_ROOT` to choose another empty directory.

For new robots, hands, input devices, cameras, and sensors, start with the
[device integration guide](docs/device_integration_guide.md). It documents the
post-refactor interfaces, coordinate/time/data contracts, required implementation
work, and acceptance checklist against the actual code.
[CR5](docs/integrations/cr5.md) and
[Aero Hand Open](docs/integrations/loophand.md) are independent ROS 2 driver
packages and composable plugins. Their [combined example](examples/cr5_loophand/README.md)
contains assembly/deployment configuration only. Real-hardware gates and the
hand's current read-only boundary are documented explicitly.
The composable architecture implementation was committed on 2026-10-28; see the
[implementation record](docs/architecture_refactor/CHANGE_LOG_2026-10-28.md) and
[remaining acceptance gates](docs/architecture_refactor/progress.md).
The [refactoring plan](docs/COMPOSABLE_ARCHITECTURE_PLAN.md) and
[`docs/TODO.md`](docs/TODO.md) retain design and follow-up context; implementation
does not imply real-device or deployed-model acceptance.
The [coordinate and kinematics optimization plan](docs/coordinate_kinematics_optimization_plan.md)
records the implemented coordinate contract, Nero J2 zero correction, named
URDF tool-chain resolver, measured-state TF bridge, and validation evidence.
The active tool remains the legacy solver tip until the real SDK flange,
connector, and task TCP are checked against the physical assembly; see the
plan for that hardware boundary.

For the current Nero bilateral installation, use the [joint mirror convention and HOME posture](docs/nero_joint_conventions.md) when deriving mirrored arm targets: J1/J3/J5/J6 reverse sign; J2/J4/J7 keep the same sign.

The web UI source is in `src/common/morla_web_manager/web`; build it with `npm ci && npm run build` there.

## Migration state

Use one validated `profile_id` across teleop/simulation, data collection, policy, and Web. Typical checks:

```bash
source /opt/ros/humble/setup.bash
source MORLA/install/setup.bash
ros2 run morla_core morla-profile validate-all
```

Run hardware only after completing the device-specific P11 checklist. The current acceptance boundary and test limitations are recorded in `docs/migration_progress.md`.

## Nero hardware Web startup

Use `bash scripts/nero_web_start.sh` on the robot host, or add `--no-quest` to skip ADB. It checks USB-CAN/serial devices, activates the host CAN mapping, grants serial access, loads ROS, stops old MORLA services and port 8080 listeners, prepares Quest port forwarding, then starts Web in the foreground. See [the complete startup guide](docs/nero_web_start.md) for prerequisites and read-only `--dry-run`.
