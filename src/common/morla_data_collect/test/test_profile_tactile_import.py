"""MORLA profile data contracts: tactile provenance and legacy XNero import."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import time

import h5py
import numpy as np
import pytest

from morla_core.profile import ProfileRegistry
from morla_data_collect.pipeline.align_data import align_episode
from morla_data_collect.pipeline.convert_to_lerobot import convert_session
from morla_data_collect.pipeline.convert_to_lerobot_v3 import convert_dataset_v21_to_v30
from morla_data_collect.pipeline.import_xnero import import_xnero
from morla_data_collect.pipeline.repair_aligned import repair_episode
from morla_data_collect.pipeline.validate_data import validate_episode
from morla_data_collect.pipeline.verify_aligned import verify_session

from conftest import make_jpeg


SOURCE_ROOT = Path(__file__).resolve().parents[4]
PROFILE_INDEX = SOURCE_ROOT / "src/common/morla_bringup/config/robots.yaml"


def _profile(profile_id="nero_left_xhand"):
    return ProfileRegistry.from_index(PROFILE_INDEX).load(profile_id)


def _write_profile_episode(ep: Path, profile, *, include_sensor=True):
    schema = profile.frozen_schema
    ep.mkdir(parents=True)
    fps, t0, duration = schema.dataset_fps, 1_700_000_000.0, 3.0
    ts = t0 + np.arange(int(duration * fps) + 1, dtype=np.float64) / fps
    joint_ts = t0 + np.arange(int(duration * 100) + 1, dtype=np.float64) / 100
    with h5py.File(ep / "robot_data.h5", "w") as robot:
        streams = robot.create_group("streams")
        for ordinal, (name, dim) in enumerate({
            block.stream_key: block.dim for block in schema.state_blocks
        }.items()):
            values = np.tile(np.arange(dim, dtype=np.float32) * 0.01, (len(joint_ts), 1))
            values += np.arange(len(joint_ts), dtype=np.float32)[:, None] * 0.001
            group = streams.create_group(name)
            group.create_dataset("values", data=values)
            group.create_dataset("timestamps", data=joint_ts)
        if include_sensor and schema.sensor_specs:
            sensor = schema.sensor_specs[0]
            n, fingers = 151, len(sensor["finger_order"])
            group = robot.create_group(f"sensors/{sensor['sensor_key']}")
            values = np.arange(n * fingers * 120 * 3, dtype=np.float32).reshape(n, fingers, 120, 3)
            group.create_dataset("force_raw", data=values)
            group.create_dataset("force_sum", data=values.sum(axis=2))
            group.create_dataset("temperature_raw", data=np.full((n, fingers, 20), 25, np.int32))
            group.create_dataset("temperature_mean", data=np.full((n, fingers), 25, np.float32))
            valid = np.ones((n, fingers), dtype=bool)
            valid[40, 1] = False
            group.create_dataset("valid", data=valid)
            group.create_dataset("temperature_valid", data=valid.copy())
            sensor_ts = t0 + np.arange(n, dtype=np.float64) / 50
            group.create_dataset("timestamps", data=sensor_ts)
            group.create_dataset("arrival_timestamps", data=sensor_ts + 0.002)
    with h5py.File(ep / "camera_data.h5", "w") as camera:
        image = np.frombuffer(make_jpeg(w=32, h=24), dtype=np.uint8)
        for cam in schema.cameras:
            group = camera.create_group(cam)
            ds = group.create_dataset("images", shape=(len(ts),), dtype=h5py.special_dtype(vlen=np.dtype("uint8")))
            for i in range(len(ts)):
                ds[i] = image
            group.create_dataset("timestamps", data=ts)
            group.create_dataset("arrival_timestamps", data=ts + 0.003)
    meta = {
        "package": "morla_data_collect", "format": "raw_hdf5_v1",
        "profile_id": profile.profile_id, "robot_id": profile.robot_id,
        "schema": schema.to_dict(), "schema_hash": schema.schema_hash,
        "layout_hash": schema.layout_hash, "task": "tactile integration fixture",
        "episode_index": 0, "start_time_wall": float(ts[0]),
        "end_time_wall": float(ts[-1]), "duration_s": duration,
        "events": [], "sensors": {
            s["sensor_key"]: {"source_kind": "sim", "force_unit": "N",
                              "force_frame": "finger_link2", "temperature_source_kind": "synthetic_zero"}
            for s in schema.sensor_specs
        },
    }
    (ep / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
    return ep


def test_tactile_sidecar_survives_align_repair_verify_and_export(tmp_path):
    profile = _profile()
    source_session = tmp_path / "raw"
    ep = _write_profile_episode(source_session / "episode000000", profile)
    assert align_episode(str(ep)) is not None
    report = validate_episode(str(ep), min_duration_s=0, require_aligned=True)
    assert report.status != "fail", report.issues
    assert not any(issue["rule"] == "W9" for issue in report.issues), report.issues
    with h5py.File(ep / "aligned_data.h5", "r") as aligned:
        group = aligned["sensors/left_hand_tactile"]
        assert group["force_raw"].shape == (101, 5, 120, 3)
        valid_rows = np.flatnonzero(group["source_index"][:] >= 0)
        for row in valid_rows[:3]:
            src_index = int(group["source_index"][row])
            np.testing.assert_array_equal(group["force_raw"][row], np.arange(151 * 5 * 120 * 3, dtype=np.float32).reshape(151, 5, 120, 3)[src_index])
        assert not group["valid"][-1].any()  # terminal holds are explicitly synthetic
    repaired = tmp_path / "repaired" / "episode000000"
    repair_episode(str(ep), str(repaired), None, False)
    assert verify_session(str(source_session), str(tmp_path / "repaired"))["status"] == "pass"
    output = tmp_path / "lerobot"
    convert_session(str(source_session), str(output), codec="h264", image_size=32, log=lambda *_: None)
    contract = json.loads((output / "meta/morla_schema.json").read_text())
    assert contract["layout_hash"] == profile.frozen_schema.layout_hash
    assert contract["sensor_policy"]["policy_features_enabled"] is False
    with h5py.File(output / "meta/sensors/episode_000000.h5", "r") as sidecar, \
            h5py.File(ep / "aligned_data.h5", "r") as aligned:
        np.testing.assert_array_equal(sidecar["left_hand_tactile/force_raw"][:],
                                      aligned["sensors/left_hand_tactile/force_raw"][:])
    v3 = tmp_path / "lerobot-v3"
    convert_dataset_v21_to_v30(str(output), str(v3), log=lambda *_: None)
    assert (v3 / "meta/morla_schema.json").is_file()
    with h5py.File(v3 / "meta/sensors/episode_000000.h5", "r") as sidecar:
        assert sidecar["left_hand_tactile/force_raw"].shape == (101, 5, 120, 3)
    with pytest.raises(ValueError, match="capacity"):
        convert_session(str(source_session), str(tmp_path / "too-small"), model_capacity=18,
                        codec="h264", image_size=32, log=lambda *_: None)


def test_profile_collector_writes_hand_id_selected_sensor_with_sim_validity(tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    from morla_data_collect.data_collect_node import DataCollectNode

    rclpy.init()
    node = DataCollectNode(parameter_overrides=[
        Parameter("save_root", value=str(tmp_path)),
        Parameter("session", value="live-fixture"),
        Parameter("profile_id", value="nero_left_xhand"),
        Parameter("runtime_mode", value="sim"),
    ])
    try:
        spec = node._sensor_specs[0]
        node._begin_episode()
        stamp = time.time()
        sec = int(stamp)
        header = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((stamp - sec) * 1e9)))
        fingers = []
        for finger_index, finger in enumerate(spec["finger_order"]):
            label = "mid" if finger == "middle" else finger
            force = [SimpleNamespace(x=float(finger_index), y=2.0, z=3.0)
                     for _ in range(spec["point_count_per_finger"])]
            fingers.append(SimpleNamespace(
                location=label, raw_force=force,
                calc_force=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                raw_temperature=[25] * 20, calc_temperature=25.0,
            ))
        sensor_state = SimpleNamespace(finger_sensor_states=fingers)
        message = SimpleNamespace(header=header, hand_id=[0], sensor_states=[sensor_state])
        node._mk_sensor_cb(spec)(message)
        node._drain_buffers()
        episode = Path(node._episode_dir)
        node._end_episode(save=True)
        with h5py.File(episode / "robot_data.h5", "r") as raw:
            group = raw["sensors/left_hand_tactile"]
            assert group["force_raw"].shape == (1, 5, 120, 3)
            np.testing.assert_array_equal(group["force_raw"][0, 2, 0], [2.0, 2.0, 3.0])
            assert group["valid"][0].all()
            assert not group["temperature_valid"][0].any()
        meta = json.loads((episode / "meta.json").read_text())
        assert meta["sensors"]["left_hand_tactile"]["source_kind"] == "synthetic"
        assert meta["sensors"]["left_hand_tactile"]["temperature_source_kind"] == "synthetic_zero"
        assert meta["sensors"]["left_hand_tactile"]["valid_samples_by_finger"] == [1] * 5
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _write_xnero_episode(ep: Path):
    ep.mkdir(parents=True)
    n, t0 = 91, 1_700_001_000.0
    ts = t0 + np.arange(n, dtype=np.float64) / 30
    with h5py.File(ep / "robot_data.h5", "w") as robot:
        for side in ("left", "right"):
            robot.create_dataset(f"{side}_arm/joints", data=np.zeros((n, 7), np.float32))
            robot.create_dataset(f"{side}_arm/tcp_pose", data=np.zeros((n, 7), np.float32))
            robot.create_dataset(f"{side}_hand/joints", data=np.zeros((n, 12), np.float32))
        robot.create_dataset("timestamps", data=ts)
    with h5py.File(ep / "camera_data.h5", "w") as camera:
        image = np.frombuffer(make_jpeg(w=32, h=24), dtype=np.uint8)
        for index, name in enumerate(("camera_0", "camera_1", "camera_2")):
            group = camera.create_group(name)
            images = group.create_dataset("images", shape=(n,), dtype=h5py.special_dtype(vlen=np.dtype("uint8")))
            for i in range(n):
                images[i] = image
            group.create_dataset("timestamps", data=ts + index * 0.0001)
    return ts


def test_import_xnero_is_read_only_explicit_and_validatable(tmp_path):
    source = tmp_path / "legacy" / "episode000000"
    ts = _write_xnero_episode(source)
    original = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                for name in ("robot_data.h5", "camera_data.h5")}
    out = tmp_path / "imported"
    result = import_xnero(str(source), str(out), profile_id="nero_dual_xhand",
                          camera_map="camera_0=base,camera_1=left_wrist,camera_2=right_wrist",
                          task="legacy pick", active_sides="left,right",
                          camera_calibration="unknown", tcp_frame="unknown",
                          index_path=str(PROFILE_INDEX))
    assert result["legacy_validity_unknown"]
    assert validate_episode(str(out / "episode000000"), min_duration_s=0).status != "fail"
    assert align_episode(str(out / "episode000000")) is not None
    report = validate_episode(str(out / "episode000000"), min_duration_s=0, require_aligned=True)
    assert report.status != "fail", report.issues
    with h5py.File(out / "episode000000/robot_data.h5", "r") as robot:
        assert "left_arm_cmd" not in robot["streams"]
        np.testing.assert_array_equal(robot["streams/left_arm_tcp_pose/values"][:], np.zeros((len(ts), 6)))
    meta = json.loads((out / "episode000000/meta.json").read_text())
    assert meta["legacy_validity_unknown"] is True
    assert meta["provenance"]["tcp_pose"]["discarded_column"] == 6
    after = {name: hashlib.sha256((source / name).read_bytes()).hexdigest()
             for name in original}
    assert after == original
    with pytest.raises(ValueError, match="camera-map"):
        import_xnero(str(source), str(tmp_path / "bad"), profile_id="nero_dual_xhand",
                     camera_map="camera_0=base", task="legacy", active_sides="left,right",
                     camera_calibration="unknown", tcp_frame="unknown",
                     index_path=str(PROFILE_INDEX))
