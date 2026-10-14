# Robot asset provenance and license notes

`config/asset_manifest.yaml` records the original workspace-relative path, source SHA256, installed path/hash, model purpose, joint map, and mesh closure for each migrated asset. It does not transfer or replace the original author's license.

Assets sourced from Astral URDF packages and `xhand_nero_description` declare BSD in their source package metadata. Astral RobotMain and MuJoCo assets, Wuji retarget/simulation assets, and XNero MJCF declare MIT in their source package metadata. The `xhand_retargeting` source package declares `TODO: License declaration`; its left/right retarget URDFs and mesh variants are preserved for compatibility, but redistribution outside this migration requires resolving that missing license with the original owner.

The description package therefore carries a mixed-license notice. Preserve the per-asset provenance and original source package license when redistributing or packaging MORLA.
