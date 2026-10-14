#!/usr/bin/env python3
"""Validate installed robot_description files, hashes, closures and optional loaders."""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

import yaml
from ament_index_python.packages import get_package_share_directory


def fail(message: str) -> None:
    raise SystemExit(f"robot_description check failed: {message}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--share", type=Path, help="use this package share instead of ament index")
    parser.add_argument("--loaders", action="store_true", help="load every unique URDF/MJCF using Pinocchio and MuJoCo")
    parser.add_argument("--require-regular-install", action="store_true", help="reject symlinks, for ordinary install verification")
    args = parser.parse_args(argv)
    raw_share = args.share if args.share else Path(get_package_share_directory("robot_description"))
    share = raw_share.resolve()
    manifest_path = raw_share / "config" / "asset_manifest.yaml"
    if not manifest_path.is_file():
        fail(f"missing {manifest_path}")
    manifest_real = manifest_path.resolve()
    source_share = None
    if not manifest_real.is_relative_to(share):
        possible_source = manifest_real.parent.parent
        if possible_source.name == "robot_description" and (possible_source / "CMakeLists.txt").is_file():
            source_share = possible_source
        else:
            fail("manifest symlink escapes its package and is not a robot_description source tree")
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("package") != "robot_description" or manifest.get("schema_version") != 1:
        fail("unsupported asset manifest identity/version")
    models = manifest.get("models")
    if not isinstance(models, list):
        fail("manifest models must be a list")
    by_id = {}
    for model in models:
        mid = model.get("model_id")
        if not isinstance(mid, str) or not mid or mid in by_id:
            fail(f"invalid or duplicate model_id {mid!r}")
        by_id[mid] = model
    for model in models:
        if not model.get("path"):
            continue
        rel = Path(model["path"])
        if rel.is_absolute() or ".." in rel.parts:
            fail(f"model path escapes package: {model['model_id']} -> {rel}")
        lexical_path = raw_share / rel
        path = lexical_path.resolve()
        allowed_root = source_share if source_share and path.is_relative_to(source_share) else share
        if not path.is_relative_to(allowed_root) or not path.is_file():
            fail(f"model file missing/outside share: {model['model_id']} -> {rel}")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != model.get("target_sha256"):
            fail(f"target hash mismatch: {model['model_id']} -> {rel}")
        for mesh in model.get("mesh_closure", []):
            mesh_rel = Path(mesh["path"])
            if mesh_rel.is_absolute() or ".." in mesh_rel.parts:
                fail(f"mesh path escapes package: {model['model_id']} -> {mesh_rel}")
            mesh_path = (raw_share / mesh_rel).resolve()
            allowed_root = source_share if source_share and mesh_path.is_relative_to(source_share) else share
            if not mesh_path.is_relative_to(allowed_root) or not mesh_path.is_file():
                fail(f"mesh missing/outside share: {model['model_id']} -> {mesh_rel}")
            if hashlib.sha256(mesh_path.read_bytes()).hexdigest() != mesh.get("sha256"):
                fail(f"mesh hash mismatch: {model['model_id']} -> {mesh_rel}")
    for asset in manifest.get("assets", []):
        rel = Path(asset["path"])
        if rel.is_absolute() or ".." in rel.parts:
            fail(f"asset path escapes package: {rel}")
        path = (raw_share / rel).resolve()
        allowed_root = source_share if source_share and path.is_relative_to(source_share) else share
        if not path.is_relative_to(allowed_root) or not path.is_file():
            fail(f"asset missing/outside share: {rel}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != asset.get("target_sha256"):
            fail(f"asset hash mismatch: {rel}")
    profile_ids = set(by_id)
    manifest_hash = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    try:
        for package in ("morla_astral_robot", "morla_nero_robot"):
            profile_dir = Path(get_package_share_directory(package)) / "config" / "profiles"
            for profile_path in profile_dir.glob("*.yaml"):
                profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
                if profile.get("description_revision") != f"sha256:{manifest_hash}":
                    fail(f"{profile_path.name} does not pin this asset manifest revision")
                for component in profile.get("components", []):
                    model_id = component.get("description_model_id")
                    if model_id and model_id not in profile_ids:
                        fail(f"{profile_path.name} references unknown model_id {model_id!r}")
                    if model_id:
                        model = by_id[model_id]
                        joint_map = model.get("ros_to_urdf_joint_map", [])
                        if joint_map:
                            profile_names = component.get("joint_names", [])
                            mapped_names = [entry.get("ros_joint") for entry in joint_map]
                            if profile_names != mapped_names:
                                fail(
                                    f"{profile_path.name}:{component.get('component_id')} joint order "
                                    f"does not match asset map for {model_id!r}"
                                )
    except Exception as exc:
        if isinstance(exc, SystemExit):
            raise
        fail(f"could not cross-check installed robot profiles: {exc}")
    if args.require_regular_install:
        if raw_share.is_symlink() or any(p.is_symlink() for p in raw_share.rglob("*")):
            fail("ordinary installation contains symlinks")
    if args.loaders:
        try:
            import mujoco
            import pinocchio as pin
        except ImportError as exc:
            fail(f"--loaders requires pinocchio and mujoco: {exc}")
        package_dirs = [str(share.parent), str(share)]
        for rel in sorted({m["path"] for m in models if str(m.get("path") or "").endswith(".urdf")}):
            wrapper = pin.RobotWrapper.BuildFromURDF(str(share / rel), package_dirs=package_dirs)
            print(f"Pinocchio PASS {rel}: nq={wrapper.model.nq}")
        for rel in sorted({m["path"] for m in models if str(m.get("path") or "").endswith(".xml")}):
            model = mujoco.MjModel.from_xml_path(str(share / rel))
            print(f"MuJoCo PASS {rel}: nq={model.nq} ngeom={model.ngeom}")
    print(f"PASS {len(models)} models, {len(manifest.get('assets', []))} assets; all closures and hashes match")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
