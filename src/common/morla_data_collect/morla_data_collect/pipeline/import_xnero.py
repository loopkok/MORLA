"""Import one legacy XNero raw episode into MORLA's profile-frozen raw format.

The importer is deliberately read-only: XNero's shared-tick/latest-cache data
does not contain per-stream validity, commands, ownership events, or a tactile
stream.  It preserves those uncertainties in metadata rather than synthesizing
measurements from zero-filled arrays.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import shutil
from typing import Any

import h5py
import numpy as np

from morla_core.profile import ProfileRegistry
from morla_data_collect.pipeline.artifacts import atomic_directory, reject_overlap
from morla_data_collect.data_writer import CAMERA_H5, META_JSON, ROBOT_H5


LEGACY_FORMAT = "xnero_v1"
TCP_DATASETS = {"left_arm": "left_arm/tcp_pose", "right_arm": "right_arm/tcp_pose"}
JOINT_DATASETS = {
    "left_arm": "left_arm/joints", "right_arm": "right_arm/joints",
    "left_ee": "left_hand/joints", "right_ee": "right_hand/joints",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_camera_map(value: str) -> dict[str, str]:
    mapping = {}
    for pair in value.split(","):
        if not pair.strip():
            continue
        if "=" not in pair:
            raise ValueError("camera-map entries must use legacy_id=profile_label")
        old, new = (part.strip() for part in pair.split("=", 1))
        if not old or not new or old in mapping:
            raise ValueError(f"invalid or duplicate camera-map entry {pair!r}")
        mapping[old] = new
    if not mapping or len(set(mapping.values())) != len(mapping):
        raise ValueError("camera-map must be nonempty and one-to-one")
    return mapping


def _parse_sides(value: str) -> tuple[str, ...]:
    sides = tuple(part.strip().lower() for part in value.split(",") if part.strip())
    if not sides or len(set(sides)) != len(sides) or any(s not in {"left", "right"} for s in sides):
        raise ValueError("active-sides must be left, right, or left,right")
    return tuple(side for side in ("left", "right") if side in sides)


def _legacy_cameras(camera_h5: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    cameras = {}
    with h5py.File(camera_h5, "r") as source:
        for camera, group in source.items():
            if not isinstance(group, h5py.Group) or not {"images", "timestamps"}.issubset(group.keys()):
                continue
            images = group["images"][:]
            timestamps = np.asarray(group["timestamps"][:], dtype=np.float64)
            if timestamps.ndim != 1 or len(timestamps) != len(images) or len(timestamps) < 2:
                raise ValueError(f"legacy camera {camera}: image/timestamp shape invalid")
            if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
                raise ValueError(f"legacy camera {camera}: timestamps must be finite and strictly increasing")
            if any(len(image) == 0 for image in images):
                raise ValueError(f"legacy camera {camera}: empty JPEG frame")
            cameras[camera] = (images, timestamps)
    if not cameras:
        raise ValueError("legacy camera_data.h5 contains no camera groups")
    return cameras


def import_xnero(source: str, out: str, *, profile_id: str, camera_map: str | dict[str, str],
                 task: str, active_sides: str | tuple[str, ...],
                 camera_calibration: str, tcp_frame: str,
                 format_version: str = LEGACY_FORMAT, overwrite: bool = False,
                 index_path: str | None = None) -> dict[str, Any]:
    """Import one XNero episode.  ``out`` is a new MORLA session directory."""
    source_path = Path(source).expanduser().resolve()
    output_path = Path(out).expanduser().resolve()
    if format_version != LEGACY_FORMAT:
        raise ValueError(f"unsupported XNero format {format_version!r}; expected {LEGACY_FORMAT!r}")
    if not task.strip():
        raise ValueError("task must be explicit and nonempty")
    if not camera_calibration.strip() or not tcp_frame.strip():
        raise ValueError("camera calibration and TCP frame must be explicit (use 'unknown' if unavailable)")
    if not source_path.is_dir():
        raise FileNotFoundError(f"legacy episode directory not found: {source_path}")
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"output exists: {output_path} (pass overwrite=True to replace it)")
    reject_overlap(str(source_path), str(output_path))
    robot_src = source_path / ROBOT_H5
    camera_src = source_path / CAMERA_H5
    if not robot_src.is_file() or not camera_src.is_file():
        raise FileNotFoundError("XNero source requires robot_data.h5 and camera_data.h5")

    registry = ProfileRegistry.from_index(index_path)
    profile = registry.load(profile_id)
    # The importer's target Profile defines the new layout, but XNero episodes
    # do not identify the URDF/J2/tool revision that produced their poses.
    # Preserve that uncertainty instead of stamping today's kinematics on old data.
    schema_metadata = dict(profile.frozen_schema.metadata)
    target_coordinate_contract = schema_metadata.pop("coordinate_contract", None)
    schema_metadata.pop("kinematics_hash", None)
    schema_metadata["legacy_kinematics_status"] = "unknown"
    schema = replace(profile.frozen_schema, metadata=schema_metadata)
    mapping = _parse_camera_map(camera_map) if isinstance(camera_map, str) else dict(camera_map)
    sides = _parse_sides(active_sides) if isinstance(active_sides, str) else tuple(active_sides)
    profile_sides = tuple(side for side in ("left", "right")
                          if any(block.block_id.startswith(f"{side}_") for block in schema.state_blocks))
    if tuple(sides) != profile_sides:
        raise ValueError(f"active-sides {tuple(sides)} do not match profile {profile_id!r} sides {profile_sides}")
    if schema.action_source == "command":
        raise ValueError("legacy XNero episodes contain no command streams; command-action profiles cannot be imported")
    if profile.robot_id != "nero":
        raise ValueError("the xnero_v1 importer only supports Nero robot profiles")

    before_hashes = {name: _sha256(path) for name, path in ((ROBOT_H5, robot_src), (CAMERA_H5, camera_src))}
    cameras = _legacy_cameras(camera_src)
    if set(mapping) != set(cameras):
        raise ValueError(f"camera-map source ids must exactly match legacy cameras {sorted(cameras)}")
    if set(mapping.values()) != set(schema.cameras):
        raise ValueError(f"camera-map targets must exactly match profile cameras {list(schema.cameras)}")

    with h5py.File(robot_src, "r") as legacy:
        if "timestamps" not in legacy:
            raise ValueError("legacy robot_data.h5 has no shared timestamps dataset")
        timestamps = np.asarray(legacy["timestamps"][:], dtype=np.float64)
        if timestamps.ndim != 1 or len(timestamps) < 2 or not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
            raise ValueError("legacy shared timestamps must be finite, strictly increasing, and contain at least two samples")
        state_streams = {}
        for block in schema.state_blocks:
            source_key = JOINT_DATASETS.get(block.block_id)
            if source_key is None:
                raise ValueError(f"legacy importer has no mapping for profile state block {block.block_id!r}")
            if source_key not in legacy:
                raise ValueError(f"legacy robot_data.h5 missing {source_key}")
            values = np.asarray(legacy[source_key][:], dtype=np.float32)
            if values.shape != (len(timestamps), block.dim):
                raise ValueError(f"{source_key}: expected {(len(timestamps), block.dim)}, found {values.shape}")
            if not np.isfinite(values).all():
                raise ValueError(f"{source_key}: non-finite joint values")
            state_streams[block.stream_key] = values
        tcp_diagnostics = {}
        for side in sides:
            arm_key = f"{side}_arm"
            source_key = TCP_DATASETS[arm_key]
            if source_key not in legacy:
                raise ValueError(f"legacy robot_data.h5 missing {source_key}")
            pose = np.asarray(legacy[source_key][:], dtype=np.float32)
            if pose.shape != (len(timestamps), 7) or not np.isfinite(pose).all():
                raise ValueError(f"{source_key}: expected finite xyz+rpy+padding shape {(len(timestamps), 7)}")
            if not np.allclose(pose[:, 6], 0.0, atol=1e-7, rtol=0):
                raise ValueError(f"{source_key}: seventh field is not zero padding; refusing to reinterpret quaternion data")
            tcp_diagnostics[f"{side}_arm_tcp_pose"] = pose[:, :6].copy()

    episode_dir_name = "episode000000"
    episode_out = output_path / episode_dir_name
    if episode_out.exists() and not overwrite:
        raise FileExistsError(f"episode output exists: {episode_out}")
    duration = float(timestamps[-1] - timestamps[0])
    source_files = {name: {"path": str((source_path / name).resolve()), "sha256": digest}
                    for name, digest in before_hashes.items()}
    provenance = {
        "importer": "morla_data_collect.pipeline.import_xnero",
        "format_version": LEGACY_FORMAT,
        "source_episode": str(source_path),
        "source_files": source_files,
        "source_timestamp_mode": "legacy_host_tick",
        "source_timestamp_note": "One host timestamp was shared across latest-cache state arrays; it is not a native per-stream sensor clock.",
        "active_sides": list(sides),
        "camera_map": mapping,
        "camera_calibration": camera_calibration,
        "tcp_pose": {"source_semantics": "xyz+rpy+zero_padding", "target_semantics": "xyz+rpy",
                     "discarded_column": 6, "frame": tcp_frame},
        "legacy_validity_unknown": True,
        "unavailable_sources": ["per-stream sample validity", "joint command streams", "ownership/intervention events", "tactile measurements", "camera arrival timestamps"],
    }

    with atomic_directory(str(episode_out)) as staged:
        with h5py.File(os.path.join(staged, ROBOT_H5), "w") as robot:
            streams_group = robot.create_group("streams")
            for name, values in state_streams.items():
                group = streams_group.create_group(name)
                group.create_dataset("values", data=values, dtype="f4")
                group.create_dataset("timestamps", data=timestamps, dtype="f8")
            for name, values in tcp_diagnostics.items():
                group = streams_group.create_group(name)
                group.create_dataset("values", data=values, dtype="f4")
                group.create_dataset("timestamps", data=timestamps, dtype="f8")
        with h5py.File(os.path.join(staged, CAMERA_H5), "w") as camera_file:
            for old_name, new_name in mapping.items():
                images, cam_ts = cameras[old_name]
                group = camera_file.create_group(new_name)
                ds = group.create_dataset("images", shape=(len(images),),
                                          dtype=h5py.special_dtype(vlen=np.dtype("uint8")))
                for i, image in enumerate(images):
                    ds[i] = image
                group.create_dataset("timestamps", data=cam_ts, dtype="f8")
        meta = {
            "package": "morla_data_collect", "format": "raw_hdf5_v1",
            "profile_id": profile_id, "robot_id": profile.robot_id,
            "schema": schema.to_dict(), "schema_hash": schema.schema_hash,
            "layout_hash": schema.layout_hash,
            "kinematics_hash": None,
            "kinematics_status": "unknown",
            "target_coordinate_contract": target_coordinate_contract,
            "task": task.strip(), "episode_index": 0,
            "start_time_wall": float(timestamps[0]), "end_time_wall": float(timestamps[-1]),
            "duration_s": duration, "timestamp_mode": "legacy_host_tick",
            "legacy_validity_unknown": True, "legacy_active_sides": list(sides),
            "events": [],
            "streams": {name: {"dim": int(values.shape[1]), "count": len(values), "validity": "unknown"}
                        for name, values in state_streams.items()},
            "cameras": {mapping[old]: {"count": len(data[0]), "timestamp_mode": "legacy_host_tick"}
                        for old, data in cameras.items()},
            "sensors": {}, "dropped": {}, "provenance": provenance,
        }
        with open(os.path.join(staged, META_JSON), "w", encoding="utf-8") as stream:
            json.dump(meta, stream, ensure_ascii=False, indent=2)
        with open(os.path.join(staged, "import_report.json"), "w", encoding="utf-8") as stream:
            json.dump({"source": str(source_path), "output": str(episode_out),
                       "source_sha256": before_hashes, "legacy_validity_unknown": True,
                       "profile_id": profile_id, "schema_hash": schema.schema_hash,
                       "layout_hash": schema.layout_hash, "camera_map": mapping,
                       "kinematics_status": "unknown",
                       "task": task.strip(), "active_sides": list(sides)},
                      stream, ensure_ascii=False, indent=2)

    after_hashes = {name: _sha256(path) for name, path in ((ROBOT_H5, robot_src), (CAMERA_H5, camera_src))}
    if after_hashes != before_hashes:
        shutil.rmtree(episode_out, ignore_errors=True)
        raise RuntimeError("legacy XNero source changed during import; imported output was removed")
    return {"source": str(source_path), "output": str(episode_out), "profile_id": profile_id,
            "frames": len(timestamps), "duration_s": duration, "source_sha256": before_hashes,
            "legacy_validity_unknown": True, "status": "imported_with_provenance_warning"}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--source", required=True, help="XNero episode directory containing robot_data.h5 and camera_data.h5")
    parser.add_argument("--out", required=True, help="new MORLA session directory")
    parser.add_argument("--profile", required=True, dest="profile_id")
    parser.add_argument("--camera-map", required=True, help="comma-separated legacy_id=profile_camera pairs")
    parser.add_argument("--task", required=True)
    parser.add_argument("--active-sides", required=True, help="left, right, or left,right; must agree with profile")
    parser.add_argument("--camera-calibration", required=True, help="calibration reference or 'unknown'")
    parser.add_argument("--tcp-frame", required=True, help="TCP frame name or 'unknown'")
    parser.add_argument("--format-version", required=True, choices=[LEGACY_FORMAT])
    parser.add_argument("--index", help="explicit robots.yaml path for isolated installations")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    result = import_xnero(args.source, args.out, profile_id=args.profile_id,
                          camera_map=args.camera_map, task=args.task,
                          active_sides=args.active_sides,
                          camera_calibration=args.camera_calibration,
                          tcp_frame=args.tcp_frame, format_version=args.format_version,
                          overwrite=args.overwrite, index_path=args.index)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
