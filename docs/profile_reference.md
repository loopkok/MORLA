# MORLA profile reference

`src/common/morla_bringup/config/robots.yaml` is the one registry. Each entry
points to a full profile in `morla_astral_robot` or `morla_nero_robot`; component
templates live in `morla_wuji_hand` and `morla_xhand_hand`. Runtime lanes must
resolve this registry by `profile_id` and persist the resulting `schema_hash`
and `layout_hash`.

| Profile | State/action dimension | Ordered layout | Cameras |
| --- | ---: | --- | --- |
| `astral_left_gripper` | 8 | left arm, left gripper ratio | base, left wrist |
| `astral_left_gripper_3cam` | 8 | left arm, left gripper ratio | base, left wrist, right wrist |
| `astral_right_gripper` | 8 | right arm, right gripper ratio | base, right wrist |
| `astral_dual_gripper` | 16 | left arm, right arm, left gripper, right gripper | base, both wrists |
| `astral_dual_gripper_wuji` | 35 | left arm, right arm, left gripper, right Wuji | base, both wrists |
| `astral_left_wuji` | 27 | left arm, left Wuji | base, left wrist |
| `astral_right_wuji` | 27 | right arm, right Wuji | base, right wrist |
| `astral_dual_wuji` | 54 | left arm, right arm, left Wuji, right Wuji | base, both wrists |
| `nero_left_xhand` | 19 | left Nero arm, left XHand | base, left wrist |
| `nero_right_xhand` | 19 | right Nero arm, right XHand | base, right wrist |
| `nero_dual_xhand` | 38 | left arm, right arm, left XHand, right XHand | base, both wrists |
| `nero_dual_arms` | 14 | left Nero arm, right Nero arm | base, both wrists |

The per-joint `feature_names` remain stable (`left_arm_0..6`, `left_ee_0..N`)
for Astral exports. Physical ROS/driver joint names are a separate field. XHand
hardware names use the 12-joint order documented in `MIGRATION_PLAN.md`; the
URDF names are side-prefixed. Tactile streams remain observation sidecars and do
not increase state/action dimensions.

Validate profiles and materialize the shared runtime inputs with:

```bash
PYTHONPATH=src/common/morla_core python3 -m morla_core.profile validate-all \
  --index src/common/morla_bringup/config/robots.yaml \
  --out-dir build/resolved_profiles
```

This writes `resolved_profile.json` and `frozen_schema.json` per profile.
Profile validation does not assert camera availability, CAN/serial connectivity,
driver feedback, model compatibility, or physical safety. Those belong to later
software-integration and hardware/model acceptance stages.
