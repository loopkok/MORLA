# MORLA robot platform

MORLA is the independent ROS 2 Humble workspace where Astral capabilities are being consolidated and Nero/XHand are being integrated. The migration follows [`docs/MIGRATION_PLAN.md`](docs/MIGRATION_PLAN.md), with source commits and exact upstream file hashes in [`docs/SOURCE_BASELINE.json`](docs/SOURCE_BASELINE.json). Current phase evidence is tracked in [`docs/migration_progress.md`](docs/migration_progress.md).

## Workspace layout

- `src/common/`: shared input, collection, policy, web management, and bringup packages.
- `src/robots/astral/`, `nero/`, `wuji/`, and `xhand/`: robot and hand packages grouped by platform.
- `tools/legacy/`: archived utilities excluded from ROS package discovery.
- `docs/legacy/astral/`: original Astral workspace entry documents for reference.

## Build

Use the system Python that matches ROS 2 Humble; do not build ROS packages under a conda Python. From the SDK root:

```bash
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:/opt/ros/humble/bin:$PATH
/usr/bin/colcon build --base-paths MORLA/src --build-base MORLA/build --install-base MORLA/install
source MORLA/install/setup.bash
```

The web UI source is in `src/common/morla_web_manager/web`; build it with `npm ci && npm run build` there.

## Migration state

MORLA currently contains the Astral baseline in the target directory layout plus initial component package shells. Nero/XHand migration and unified profiles, description, collection, policy, and web operation support are in progress. Treat only items marked complete and software-verified in `docs/migration_progress.md` as delivered. Do not connect real hardware until its phase-specific acceptance is recorded.
