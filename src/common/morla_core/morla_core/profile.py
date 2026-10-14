"""Robot-profile loading and validation shared by MORLA runtime lanes."""
from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .resource_locator import ResourceLookupError, morla_source_root, package_share
from .schema import ActionBlockSpec, FrozenSchema, StateBlockSpec, StreamSpec

_ID = re.compile(r"^[a-z][a-z0-9_-]*$")
_PROFILE_REQUIRED = {
    "profile_version", "profile_id", "robot_id", "components", "streams", "state_blocks",
    "action_blocks", "cameras", "capabilities", "writers", "backend_requirements", "data", "hardware",
}


class ProfileValidationError(ValueError):
    """A robot profile is incomplete or internally inconsistent."""


_ALLOWED_SOURCE_KINDS = {"state", "command", "command_echo", "camera", "sensor"}
_EXPECTED_UNITS = {
    "sensor_msgs/msg/JointState": "rad",
    "std_msgs/msg/Float64": "ratio",
    "sensor_msgs/msg/CompressedImage": "rgb",
    "xhand_control_interfaces/msg/XHandStateArray": "source_native",
}


@dataclass(frozen=True)
class RobotProfile:
    """Validated source mapping plus its immutable shared schema."""

    values: dict[str, Any]
    frozen_schema: FrozenSchema

    @property
    def profile_id(self) -> str:
        return self.values["profile_id"]

    @property
    def robot_id(self) -> str:
        return self.values["robot_id"]

    def resolved(self) -> dict[str, Any]:
        return {
            **self.values,
            "frozen_schema": self.frozen_schema.to_dict(),
            "schema_hash": self.frozen_schema.schema_hash,
            "layout_hash": self.frozen_schema.layout_hash,
        }

    def write_resolved(self, output_dir: str | Path) -> tuple[Path, Path]:
        """Materialize the profile and its immutable schema for all runtime lanes."""
        directory = Path(output_dir)
        directory.mkdir(parents=True, exist_ok=True)
        profile_path = directory / "resolved_profile.json"
        schema_path = directory / "frozen_schema.json"
        profile_path.write_text(
            json.dumps(self.resolved(), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        schema_path.write_text(self.frozen_schema.to_json() + "\n", encoding="utf-8")
        return profile_path, schema_path


def validate_profile(value: dict[str, Any]) -> RobotProfile:
    """Validate all profile structure that affects streams, layout or writers."""
    if not isinstance(value, dict):
        raise ProfileValidationError("profile must be a mapping")
    missing = _PROFILE_REQUIRED - set(value)
    if missing:
        raise ProfileValidationError(f"profile is missing required fields: {sorted(missing)}")
    if isinstance(value["profile_version"], bool) or value["profile_version"] != 1:
        raise ProfileValidationError(f"unsupported profile_version {value['profile_version']!r}")
    for key in ("profile_id", "robot_id"):
        if not isinstance(value[key], str) or not _ID.fullmatch(value[key]):
            raise ProfileValidationError(f"{key} must be a lower-case stable identifier")
    if not isinstance(value["components"], list) or not value["components"]:
        raise ProfileValidationError("components must be a nonempty list")
    component_ids: set[str] = set()
    enabled_sides: set[str] = set()
    for component in value["components"]:
        _require_mapping(component, {"component_id", "enabled", "side", "type", "package", "params_file"}, "component")
        cid = component["component_id"]
        if not isinstance(cid, str) or not _ID.fullmatch(cid) or cid in component_ids:
            raise ProfileValidationError(f"invalid or duplicate component_id {cid!r}")
        component_ids.add(cid)
        if not isinstance(component["enabled"], bool):
            raise ProfileValidationError(f"component {cid!r} enabled must be boolean")
        if component["side"] not in {"left", "right", "base"}:
            raise ProfileValidationError(f"component {cid!r} has invalid side")
        if not all(isinstance(component[k], str) and component[k] for k in ("type", "package", "params_file")):
            raise ProfileValidationError(f"component {cid!r} needs type, package and params_file")
        if component["enabled"]:
            enabled_sides.add(component["side"])

    for section in ("streams", "state_blocks", "action_blocks", "cameras", "writers"):
        if not isinstance(value[section], list):
            raise ProfileValidationError(f"{section} must be a list")

    try:
        streams = tuple(StreamSpec.from_dict(row) for row in value["streams"])
        state_blocks = tuple(StateBlockSpec.from_dict(row) for row in value["state_blocks"])
        action_blocks = tuple(ActionBlockSpec.from_dict(row) for row in value["action_blocks"])
        data = value["data"]
        if not isinstance(data, dict):
            raise ValueError("data must be a mapping")
        schema = FrozenSchema(
            profile_id=value["profile_id"], robot_id=value["robot_id"],
            streams=streams, state_blocks=state_blocks, action_blocks=action_blocks,
            cameras=tuple(value["cameras"]), dataset_fps=data["dataset_fps"],
            action_source=data["action_source"], hold_frames=data["hold_frames"],
            max_gap_ms=data["max_gap_ms"], profile_version=value["profile_version"],
            sensor_specs=tuple(value.get("sensors", ())),
            metadata={"description_revision": value.get("description_revision", "unresolved")},
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ProfileValidationError(str(exc)) from exc

    if not isinstance(value["capabilities"], dict):
        raise ProfileValidationError("capabilities must be a mapping")
    if not isinstance(value["backend_requirements"], dict):
        raise ProfileValidationError("backend_requirements must be a mapping")
    if not isinstance(value["hardware"], dict):
        raise ProfileValidationError("hardware must be a mapping")
    if not isinstance(value["writers"], list):
        raise ProfileValidationError("writers must be a list")

    # A topic has one ROS type in a profile. Allowing two types on a topic is
    # just as ambiguous as registering the same topic/type twice.
    topic_types: dict[str, str] = {}
    for stream in streams:
        if stream.source_kind not in _ALLOWED_SOURCE_KINDS:
            raise ProfileValidationError(f"stream {stream.stream_key!r} has unknown source_kind")
        expected_unit = _EXPECTED_UNITS.get(stream.message_type)
        if expected_unit is not None and stream.unit != expected_unit:
            raise ProfileValidationError(
                f"stream {stream.stream_key!r} unit {stream.unit!r} does not match {stream.message_type} ({expected_unit})"
            )
        if stream.message_type == "sensor_msgs/msg/JointState":
            if len(stream.shape) != 1 or not stream.joint_names:
                raise ProfileValidationError(f"JointState stream {stream.stream_key!r} requires a 1-D named layout")
        if stream.source_kind == "camera" and stream.message_type != "sensor_msgs/msg/CompressedImage":
            raise ProfileValidationError(f"camera stream {stream.stream_key!r} must use CompressedImage")
        if stream.source_kind == "sensor" and stream.message_type != "xhand_control_interfaces/msg/XHandStateArray":
            raise ProfileValidationError(f"sensor stream {stream.stream_key!r} has no registered decoder")
        previous = topic_types.setdefault(stream.topic, stream.message_type)
        if previous != stream.message_type:
            raise ProfileValidationError(
                f"topic {stream.topic!r} has conflicting types {previous!r} and {stream.message_type!r}"
            )

    component_by_id = {c["component_id"]: c for c in value["components"]}
    stream_by_key = {stream.stream_key: stream for stream in streams}
    state_features: list[str] = []
    action_features: list[str] = []
    for block in state_blocks:
        stream = stream_by_key[block.stream_key]
        if block.unit != stream.unit:
            raise ProfileValidationError(
                f"state block {block.block_id!r} unit {block.unit!r} != stream unit {stream.unit!r}"
            )
        if block.physical_joint_names and stream.joint_names:
            mapped_names = tuple(stream.joint_names[i] for i in block.stream_indices)
            if mapped_names != block.physical_joint_names:
                raise ProfileValidationError(f"state block {block.block_id!r} joint names do not match its stream")
        if block.block_id.endswith(("_arm", "_ee")):
            component = component_by_id.get(block.block_id)
            if component is None or not component["enabled"]:
                raise ProfileValidationError(f"state block {block.block_id!r} has no enabled component")
        state_features.extend(block.feature_names)

    for block in action_blocks:
        action_features.extend(block.feature_names)
        if block.writable:
            stream = stream_by_key[block.command_stream]
            if stream.source_kind != "command":
                raise ProfileValidationError(
                    f"action block {block.block_id!r} command stream is marked {stream.source_kind!r}"
                )
            if block.unit != stream.unit or stream.shape[-1] != block.dim:
                raise ProfileValidationError(f"action block {block.block_id!r} does not match its command stream")
    if len(state_features) != len(set(state_features)) or len(action_features) != len(set(action_features)):
        raise ProfileValidationError("feature names must be unique within each state/action layout")

    camera_specs = value.get("camera_specs")
    if camera_specs is not None:
        if not isinstance(camera_specs, list) or [row.get("label") for row in camera_specs] != value["cameras"]:
            raise ProfileValidationError("camera_specs must be ordered exactly like cameras")
        if [row.get("slot") for row in camera_specs] != list(range(len(value["cameras"]))):
            raise ProfileValidationError("camera_specs slots must be contiguous from zero")

    data = value["data"]
    if not isinstance(data, dict):
        raise ProfileValidationError("data must be a mapping")
    if data.get("action_source") not in {"next_state", "command"}:
        raise ProfileValidationError("data.action_source must be next_state or command")
    if data["action_source"] == "command" and any(not block.writable for block in action_blocks):
        raise ProfileValidationError("command action_source requires every action block to be writable")

    # Any left/right model block must correspond to an enabled robot component;
    # this prevents stale inactive-side blocks and ACK writers from leaking in.
    for block in (*state_blocks, *action_blocks):
        if block.block_id.startswith(("left_", "right_")) and block.block_id.endswith(("_arm", "_ee")):
            component = component_by_id.get(block.block_id)
            if component is None or component["side"] not in enabled_sides or not component["enabled"]:
                raise ProfileValidationError(f"inactive-side block {block.block_id!r} is not allowed")

    writer_by_id: dict[str, dict[str, Any]] = {}
    enabled_components = {c["component_id"] for c in value["components"] if c["enabled"]}
    for writer in value["writers"]:
        _require_mapping(writer, {"writer_id", "component_id", "command_stream", "ack_topic"}, "writer")
        wid = writer["writer_id"]
        if not isinstance(wid, str) or not _ID.fullmatch(wid) or wid in writer_by_id:
            raise ProfileValidationError(f"invalid or duplicate writer_id {wid!r}")
        if writer["component_id"] not in enabled_components:
            raise ProfileValidationError(f"writer {wid!r} refers to a disabled or unknown component")
        stream = stream_by_key.get(writer["command_stream"])
        if stream is None or stream.source_kind != "command":
            raise ProfileValidationError(f"writer {wid!r} command_stream is not a command stream")
        if writer["ack_topic"] and not str(writer["ack_topic"]).startswith("/"):
            raise ProfileValidationError(f"writer {wid!r} ack_topic must be absolute")
        writer_by_id[wid] = writer
    for block in action_blocks:
        if block.writable:
            writer = writer_by_id.get(block.writer_id or "")
            if writer is None or writer["command_stream"] != block.command_stream:
                raise ProfileValidationError(f"writable action block {block.block_id!r} has no matching writer")

    camera_streams = {s.stream_key.removeprefix("camera:") for s in streams if s.source_kind == "camera"}
    if set(value["cameras"]) != camera_streams:
        raise ProfileValidationError("camera labels must match the registered camera streams exactly")
    return RobotProfile(values=value, frozen_schema=schema)


class ProfileRegistry:
    """Index-based profile loader; resources are always resolved from ament share."""

    def __init__(self, entries: dict[str, dict[str, str]], *, source_root: Path | None = None):
        self.entries = entries
        self.source_root = source_root
        if not entries:
            raise ProfileValidationError("profile registry is empty")
        for profile_id, row in entries.items():
            if not _ID.fullmatch(profile_id):
                raise ProfileValidationError(f"invalid registry profile id {profile_id!r}")
            _require_mapping(row, {"package", "path", "title"}, f"registry profile {profile_id}")

    @classmethod
    def from_index(cls, index_path: str | Path | None = None) -> "ProfileRegistry":
        if index_path is None:
            try:
                index = package_share("morla_bringup") / "config/robots.yaml"
                return cls._from_yaml_index(index, source_root=None)
            except ResourceLookupError:
                source_root = morla_source_root()
                index = source_root / "src/common/morla_bringup/config/robots.yaml"
                return cls._from_yaml_index(index, source_root=source_root)
        path = Path(index_path).expanduser().resolve()
        source_root = None
        for parent in path.parents:
            if (parent / "docs/MIGRATION_PLAN.md").is_file():
                source_root = parent
                break
        return cls._from_yaml_index(path, source_root=source_root)

    @classmethod
    def _from_yaml_index(cls, path: Path, *, source_root: Path | None) -> "ProfileRegistry":
        yaml = _yaml_module()
        if not path.is_file():
            raise ProfileValidationError(f"robot profile index not found: {path}")
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(raw, dict) or set(raw) != {"profiles"} or not isinstance(raw["profiles"], dict):
            raise ProfileValidationError("robots.yaml must contain only a profiles mapping")
        return cls(raw["profiles"], source_root=source_root)

    def ids(self) -> tuple[str, ...]:
        return tuple(sorted(self.entries))

    def load(self, profile_id: str) -> RobotProfile:
        try:
            entry = self.entries[profile_id]
        except KeyError as exc:
            raise ProfileValidationError(f"unknown profile_id {profile_id!r}; available={self.ids()}") from exc
        share = self._package_share(entry["package"])
        relative = Path(entry["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ProfileValidationError(f"profile path escapes package: {entry['path']!r}")
        path = (share / relative).resolve()
        if not path.is_relative_to(share) or not path.is_file():
            raise ProfileValidationError(f"profile resource missing or outside package: {entry['package']}/{relative}")
        raw = _yaml_module().safe_load(path.read_text(encoding="utf-8"))
        if raw.get("profile_id") != profile_id:
            raise ProfileValidationError(f"profile id mismatch: registry {profile_id!r}, file {raw.get('profile_id')!r}")
        profile = validate_profile(raw)
        for component in raw["components"]:
            relative = Path(component["params_file"])
            if relative.is_absolute() or ".." in relative.parts:
                raise ProfileValidationError(f"component params path escapes package: {component['params_file']!r}")
            component_share = self._package_share(component["package"])
            params = (component_share / relative).resolve()
            if not params.is_relative_to(component_share) or not params.is_file():
                raise ProfileValidationError(
                    f"component params resource missing: {component['package']}/{relative}"
                )
        return profile

    def validate_all(self) -> list[RobotProfile]:
        return [self.load(profile_id) for profile_id in self.ids()]

    def _package_share(self, package: str) -> Path:
        if self.source_root is not None:
            for manifest in (self.source_root / "src").rglob("package.xml"):
                try:
                    import xml.etree.ElementTree as ET
                    if ET.parse(manifest).getroot().findtext("name") == package:
                        return manifest.parent.resolve()
                except Exception:
                    continue
        try:
            return package_share(package)
        except ResourceLookupError:
            raise ProfileValidationError(f"profile package {package!r} is unavailable")


def validate_model_capacity(profile: RobotProfile, capacity: int) -> None:
    if isinstance(capacity, bool) or capacity < 1:
        raise ValueError("model capacity must be a positive integer")
    if profile.frozen_schema.action_dim > capacity:
        raise ProfileValidationError(
            f"profile {profile.profile_id!r} action_dim={profile.frozen_schema.action_dim} "
            f"exceeds model capacity={capacity}; define a named reduced layout or use a larger model"
        )


def _require_mapping(value: Any, required: set[str], label: str) -> None:
    if not isinstance(value, dict):
        raise ProfileValidationError(f"{label} must be a mapping")
    missing = required - set(value)
    if missing:
        raise ProfileValidationError(f"{label} missing fields: {sorted(missing)}")


def _yaml_module():
    try:
        import yaml
        return yaml
    except ImportError as exc:
        raise RuntimeError("profile YAML support requires PyYAML/python3-yaml") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m morla_core.profile")
    sub = parser.add_subparsers(dest="command", required=True)
    all_parser = sub.add_parser("validate-all", help="validate every profile in the registry")
    all_parser.add_argument("--index", help="explicit robots.yaml path")
    all_parser.add_argument("--out-dir", help="write resolved profile and frozen schema JSON files here")
    one_parser = sub.add_parser("validate", help="validate one profile YAML file")
    one_parser.add_argument("path")
    args = parser.parse_args(argv)
    try:
        if args.command == "validate-all":
            registry = ProfileRegistry.from_index(args.index)
            profiles = registry.validate_all()
            for profile in profiles:
                schema = profile.frozen_schema
                print(f"PASS {profile.profile_id}: state={schema.state_dim} action={schema.action_dim} "
                      f"layout={schema.layout_hash[:12]} schema={schema.schema_hash[:12]}")
                if args.out_dir:
                    paths = profile.write_resolved(Path(args.out_dir) / profile.profile_id)
                    print(f"  wrote {paths[0]} and {paths[1]}")
            print(f"validated {len(profiles)} robot profiles")
            return 0
        raw = _yaml_module().safe_load(Path(args.path).read_text(encoding="utf-8"))
        profile = validate_profile(raw)
        print(json.dumps(profile.resolved(), ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        print(f"profile validation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
