# Astral profile registry

Each YAML file is a complete robot/data layout registered in
`morla_bringup/config/robots.yaml`. It references the existing calibrated arm,
gripper, and Wuji parameter files; state/action order and camera selection live
in that profile. The stable OpenPI feature names remain `left_arm_N`,
`right_arm_N`, and `*_ee_N`.

Validate and materialize all layouts from the MORLA root:

```bash
PYTHONPATH=src/common/morla_core python3 -m morla_core.profile validate-all \
  --index src/common/morla_bringup/config/robots.yaml --out-dir build/resolved_profiles
```

The layouts cover 8, 16, 27, 35, and 54 dimensions. The 35/54 dimensional
profiles are valid collection layouts; standard 32-dimensional pi05 models must
reject them before inference.
