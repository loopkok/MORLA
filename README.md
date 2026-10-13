# MORLA robot platform

MORLA is the unified ROS 2 Humble workspace for Astral, Nero, Wuji Hand, and XHand. The software migration through P10 is complete: common input, profile-driven robot assembly, description assets, collection/pipeline, policy/replay/HITL, and Web operation/monitoring are integrated. Hardware and deployed-model acceptance is tracked separately as P11; no software simulation result implies real-hardware acceptance. See [`docs/migration_progress.md`](docs/migration_progress.md) for evidence and remaining deployment conditions, [`docs/MIGRATION_PLAN.md`](docs/MIGRATION_PLAN.md) for contracts, and [`docs/SOURCE_BASELINE.json`](docs/SOURCE_BASELINE.json) for upstream provenance.

## Workspace layout

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
`scripts/test_policy.sh`, and `scripts/test_sim_integration.sh`. The simulation
test writes its synthetic raw and exported data under `/tmp` by default; set
`MORLA_TEST_OUTPUT_ROOT` to choose another empty directory.

The web UI source is in `src/common/morla_web_manager/web`; build it with `npm ci && npm run build` there.

## Migration state

Use one validated `profile_id` across teleop/simulation, data collection, policy, and Web. Typical checks:

```bash
source /opt/ros/humble/setup.bash
source MORLA/install/setup.bash
ros2 run morla_core morla-profile validate-all
```

Run hardware only after completing the device-specific P11 checklist. The current acceptance boundary and test limitations are recorded in `docs/migration_progress.md`.
