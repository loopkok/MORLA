"""Build profile-scoped robot description and measured joint-state bridges."""
from __future__ import annotations

import json
from typing import Any

from morla_core.resource_locator import package_share, resolve_robot_asset


_STATE_COMPONENT_TYPES = {"arm", "nero_arm", "xhand", "wuji_hand"}


def profile_state_sources(profile: dict[str, Any], manifest: dict[str, Any]) -> list[dict[str, Any]]:
    """Resolve enabled measured JointState inputs into URDF joint names.

    Commands and scalar gripper topics are intentionally excluded. The bridge
    emits one mapped message per source callback, preserving its measurement
    timestamp instead of inventing a merged or command-derived robot state.
    """
    models = {row["model_id"]: row for row in manifest.get("models", [])}
    sources: list[dict[str, Any]] = []
    for component in profile.get("components", []):
        if not component.get("enabled") or component.get("type") not in _STATE_COMPONENT_TYPES:
            continue
        topic = str(component.get("state_topic", "")).strip()
        if not topic.endswith("/joint_states"):
            continue
        source_names = list(component.get("joint_names", []))
        if not source_names:
            raise ValueError(f"{component.get('component_id')}: missing source joint_names")
        explicit_target_names = component.get("urdf_joint_names")
        if explicit_target_names is not None:
            target_names = list(explicit_target_names)
        else:
            model_id = str(component.get("description_model_id", ""))
            model = models.get(model_id)
            if model is None:
                raise ValueError(
                    f"{component.get('component_id')}: unknown description model {model_id!r}"
                )
            mapping = {
                row["ros_joint"]: row["urdf_joint"]
                for row in model.get("ros_to_urdf_joint_map", [])
            }
            target_names = [mapping.get(name, name) for name in source_names]
        if len(target_names) != len(source_names) or len(set(target_names)) != len(target_names):
            raise ValueError(f"{component.get('component_id')}: invalid URDF joint mapping")
        sources.append({
            "component_id": str(component["component_id"]),
            "topic": topic,
            "source_joint_names": source_names,
            "urdf_joint_names": target_names,
        })
    return sources


def remap_joint_state_fields(
    names: list[str],
    position: list[float],
    velocity: list[float],
    effort: list[float],
    source_joint_names: list[str],
    urdf_joint_names: list[str],
) -> dict[str, list]:
    """Select and order a complete measured component state by configured names."""
    if len(names) != len(position) or not names or len(set(names)) != len(names):
        raise ValueError("JointState names and positions must be nonempty, unique and aligned")
    if velocity and len(velocity) != len(names):
        raise ValueError("JointState velocities do not match names")
    if effort and len(effort) != len(names):
        raise ValueError("JointState efforts do not match names")
    if len(source_joint_names) != len(urdf_joint_names):
        raise ValueError("configured source/URDF joint mappings differ in length")
    by_name = {name: index for index, name in enumerate(names)}
    missing = [name for name in source_joint_names if name not in by_name]
    if missing:
        raise ValueError(f"JointState is missing configured joints: {missing}")
    indices = [by_name[name] for name in source_joint_names]
    return {
        "name": list(urdf_joint_names),
        "position": [position[index] for index in indices],
        "velocity": [velocity[index] for index in indices] if velocity else [],
        "effort": [effort[index] for index in indices] if effort else [],
    }


def profile_description_actions(profile: Any) -> list[Any]:
    """Create RSP + measured-state bridge actions for a validated robot Profile."""
    from launch_ros.actions import Node
    import yaml

    values = profile.values if hasattr(profile, "values") else profile
    arm_components = [
        row for row in values.get("components", [])
        if row.get("enabled") and row.get("type") in {"arm", "nero_arm"}
    ]
    if not arm_components:
        return []
    model_ids = {str(row.get("description_model_id", "")) for row in arm_components}
    if "" in model_ids:
        raise ValueError(f"Profile {values.get('profile_id')} arm is missing description_model_id")
    model_paths = {resolve_robot_asset(model_id).resolve() for model_id in model_ids}
    if len(model_paths) != 1:
        raise ValueError(
            f"Profile {values.get('profile_id')} arms resolve to different robot descriptions"
        )
    model_path = next(iter(model_paths))
    description = model_path.read_text(encoding="utf-8")
    manifest_path = package_share("robot_description") / "config" / "asset_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    specs = profile_state_sources(values, manifest)
    if not specs:
        raise ValueError(f"Profile {values.get('profile_id')} has no measured joint-state inputs")

    from morla_core.coordinate_contract import build_coordinate_contract

    contract = build_coordinate_contract(values)
    return [
        Node(
            package="morla_bringup",
            executable="profile_joint_state_bridge",
            name="profile_joint_state_bridge",
            output="screen",
            parameters=[{
                "profile_id": profile.profile_id if hasattr(profile, "profile_id") else values["profile_id"],
                "source_specs_json": json.dumps(specs, separators=(",", ":")),
                "kinematics_hash": contract.kinematics_hash,
                "coordinate_contract_version": contract.version,
                "robot_body_frame": contract.robot_body_frame,
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": description,
                "publish_frequency": 50.0,
            }],
        ),
    ]
