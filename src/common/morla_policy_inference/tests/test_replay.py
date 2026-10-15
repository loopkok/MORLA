#!/usr/bin/env python3
"""Tests for replay loading + PlaybackSession offset semantics."""

import json
import os
import tempfile
import unittest

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from morla_core.profile import ProfileRegistry
from morla_data_collect.schema import CollectSchema
from morla_policy_inference.replay import (
    PlaybackSession,
    ReplayEpisode,
    load_aligned_actions,
    load_raw_actions,
    load_replay,
)

DIM = 8


def episode(n=10, offset=0.0):
    acts = np.zeros((n, DIM), dtype=np.float64)
    acts[:, 0] = np.arange(n) + offset
    return ReplayEpisode(actions=acts, fps=30, action_dim=DIM, source="t")


class TestPlaybackSession(unittest.TestCase):
    def test_steps_and_finishes(self):
        ps = PlaybackSession(episode(10))
        self.assertEqual(ps.remaining, 10)
        n = 0
        while not ps.done and n < 20:
            ps.advance()
            n += 1
        self.assertTrue(ps.done)
        self.assertEqual(n, 10)
        self.assertIsNone(ps.target())

    def test_target_sequence(self):
        ps = PlaybackSession(episode(5))
        first = ps.target()
        ps.advance()
        second = ps.target()
        self.assertAlmostEqual(second[0] - first[0], 1.0)

    def test_reanchor_continues_from_robot_state(self):
        ps = PlaybackSession(episode(20))
        ps.advance()
        ps.advance()  # idx == 2, episode action[2][0] == 2
        # human moved the robot arm to 10.0 rad (simplified single-dim check)
        robot = np.zeros(DIM)
        robot[0] = 10.0
        ps.reanchor(robot)
        tgt = ps.target()
        # next target keeps following the recorded *shape* from the robot pose
        self.assertAlmostEqual(tgt[0], 10.0)
        ps.advance()
        tgt2 = ps.target()
        self.assertAlmostEqual(tgt2[0], 11.0)

    def test_reanchor_wrong_dim_raises(self):
        ps = PlaybackSession(episode(5))
        with self.assertRaises(ValueError):
            ps.reanchor(np.zeros(DIM + 2))

    def test_reanchor_does_not_advance(self):
        ps = PlaybackSession(episode(5))
        ps.reanchor(np.full(DIM, 2.0))
        self.assertEqual(ps.idx, 0)
        self.assertEqual(ps.remaining, 5)

    def test_reset_offset(self):
        ps = PlaybackSession(episode(5))
        ps.reanchor(np.full(DIM, 9.0))
        ps.reset_offset()
        np.testing.assert_allclose(ps.target(), ps.episode.actions[0])

    def test_duration(self):
        ps = PlaybackSession(episode(31, offset=1.0))
        self.assertAlmostEqual(ps.episode.duration_s(), 1.0)


class TestLoading(unittest.TestCase):
    def test_load_aligned_h5(self):
        schema = CollectSchema(
            arms=["left"], end_effector_left="gripper",
            cameras=["base"], dataset_fps=30,
        )
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "aligned_data.h5")
            import h5py

            with h5py.File(path, "w") as f:
                f.create_dataset("action", data=np.zeros((7, DIM)))
                f.attrs["fps"] = 30
                f.attrs["schema"] = schema.to_json()
            ep = load_aligned_actions(path)
            self.assertEqual(ep.num_frames, 7)
            self.assertEqual(ep.action_dim, DIM)
            self.assertEqual(ep.fps, 30)
            self.assertIsNotNone(ep.schema)

    def test_load_replay_detects_directory_vs_file(self):
        with tempfile.TemporaryDirectory() as d:
            h5 = os.path.join(d, "aligned_data.h5")
            import h5py

            with h5py.File(h5, "w") as f:
                f.create_dataset("action", data=np.zeros((3, DIM)))
                f.attrs["fps"] = 30
            self.assertEqual(load_replay(h5).num_frames, 3)
            with self.assertRaises(FileNotFoundError):
                load_replay(os.path.join(d, "no_such_dir"))

    def test_lerobot_directory_with_missing_meta(self):
        import tempfile as tf

        with tf.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, "meta"))
            # missing info.json -> parse error surfaces (not silent)
            with self.assertRaises(Exception):
                load_replay(d)

    def test_lerobot_replay_uses_wrapped_frozen_profile_schema(self):
        profile = ProfileRegistry.from_index().load("nero_left_xhand")
        with tempfile.TemporaryDirectory() as d:
            meta = os.path.join(d, "meta")
            data = os.path.join(d, "data", "chunk-000")
            os.makedirs(meta)
            os.makedirs(data)
            with open(os.path.join(meta, "info.json"), "w", encoding="utf-8") as f:
                json.dump({
                    "fps": 30,
                    "data_path": (
                        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
                    ),
                    "features": {"action": {"shape": [19]}},
                }, f)
            contract = {
                "schema": profile.frozen_schema.to_dict(),
                "profile_id": profile.profile_id,
                "schema_hash": profile.frozen_schema.schema_hash,
                "layout_hash": profile.frozen_schema.layout_hash,
            }
            with open(os.path.join(meta, "morla_schema.json"), "w", encoding="utf-8") as f:
                json.dump(contract, f)
            actions = np.zeros((4, profile.frozen_schema.action_dim), dtype=np.float32)
            pq.write_table(pa.table({"action": actions.tolist()}),
                           os.path.join(data, "episode_000000.parquet"))

            replay = load_replay(d, 0)
            self.assertEqual(replay.action_dim, 19)
            self.assertEqual(replay.schema.layout_hash, profile.frozen_schema.layout_hash)
            self.assertEqual(replay.schema.state_names(), profile.frozen_schema.state_names())


class TestRawLoading(unittest.TestCase):
    def test_raw_replay_resamples_measured_state_and_causal_gripper(self):
        import h5py

        schema = CollectSchema(
            arms=["left"], end_effector_left="gripper", end_effector_right="none",
            cameras=["base"], dataset_fps=10, max_gap_ms=300, action_source="command",
        )
        ts = np.array([10.0, 10.05, 10.15, 10.25, 10.35, 10.4])
        with tempfile.TemporaryDirectory() as session:
            episode_dir = os.path.join(session, "episode000005")
            os.mkdir(episode_dir)
            robot_path = os.path.join(episode_dir, "robot_data.h5")
            with h5py.File(robot_path, "w") as f:
                for name, times, values in (
                    ("left_arm_state", ts, np.repeat((ts - 10)[:, None], 7, axis=1)),
                    ("left_gripper_ratio", [10, 10.15, 10.4], [[0], [1], [1]]),
                    ("left_arm_cmd", ts, np.full((len(ts), 7), 99.0)),
                ):
                    group = f.create_group(f"streams/{name}")
                    group.create_dataset("timestamps", data=times)
                    group.create_dataset("values", data=values)
            with open(os.path.join(episode_dir, "meta.json"), "w", encoding="utf-8") as f:
                json.dump({
                    "schema": schema.to_dict(), "start_time_wall": 10.0,
                    "end_time_wall": 10.4, "events": [],
                }, f)

            before = os.stat(robot_path).st_mtime_ns
            expected = load_raw_actions(episode_dir)
            self.assertEqual(expected.fps, 10)
            self.assertEqual(expected.action_dim, 8)
            self.assertEqual(expected.num_frames, 5)
            np.testing.assert_allclose(expected.actions[:, 0], [0, .1, .2, .3, .4], atol=1e-12)
            np.testing.assert_array_equal(expected.actions[:, 7], [0, 0, 1, 1, 1])
            self.assertEqual(os.stat(robot_path).st_mtime_ns, before)
            self.assertFalse(os.path.exists(os.path.join(episode_dir, "aligned_data.h5")))
            for source in (episode_dir, session, robot_path):
                replay = load_replay(source, 5 if source == session else 0)
                np.testing.assert_array_equal(replay.actions, expected.actions)

    def test_frozen_profile_layout_maps_measured_state_blocks_to_action_indices(self):
        import h5py

        profile = ProfileRegistry.from_index().load("nero_left_xhand")
        schema = profile.frozen_schema
        times = 1.0 + np.arange(41, dtype=np.float64) / 40.0
        with tempfile.TemporaryDirectory() as episode_dir:
            with h5py.File(os.path.join(episode_dir, "robot_data.h5"), "w") as f:
                streams = f.create_group("streams")
                for stream_name in sorted({b.stream_key for b in schema.state_blocks}):
                    spec = next(s for s in schema.streams if s.stream_key == stream_name)
                    dim = int(spec.shape[-1])
                    offset = 10 if stream_name == "left_hand_state" else 0
                    group = streams.create_group(stream_name)
                    group.create_dataset("timestamps", data=times)
                    group.create_dataset(
                        "values", data=np.repeat((times + offset)[:, None], dim, axis=1),
                    )
            with open(os.path.join(episode_dir, "meta.json"), "w", encoding="utf-8") as f:
                json.dump({
                    "schema": schema.to_dict(), "start_time_wall": 1.0,
                    "end_time_wall": 2.0, "events": [],
                    "streams": {name: {"count": len(times)}
                                for name in {b.stream_key for b in schema.state_blocks}},
                }, f)

            replay = load_raw_actions(episode_dir)
            self.assertEqual(replay.action_dim, schema.action_dim)
            self.assertEqual(replay.num_frames, 31)
            self.assertEqual(replay.schema.layout_hash, schema.layout_hash)
            for state in schema.state_blocks:
                action = next(b for b in schema.action_blocks if b.block_id == state.block_id)
                expected = times[0] + np.arange(31) / schema.dataset_fps
                if state.stream_key == "left_hand_state":
                    expected = expected + 10
                np.testing.assert_allclose(
                    replay.actions[:, np.asarray(action.action_indices)],
                    np.repeat(expected[:, None], state.dim, axis=1), atol=1e-10,
                )

    def test_raw_replay_rejects_pauses_invalid_counts_and_large_gaps(self):
        import h5py

        schema = CollectSchema(
            arms=["left"], end_effector_left="none", end_effector_right="none",
            cameras=["base"], dataset_fps=10, max_gap_ms=50,
        )
        with tempfile.TemporaryDirectory() as episode_dir:
            ts = np.array([10.0, 10.05, 10.15, 10.25, 10.35, 10.4])
            with h5py.File(os.path.join(episode_dir, "robot_data.h5"), "w") as f:
                group = f.create_group("streams/left_arm_state")
                group.create_dataset("timestamps", data=ts)
                group.create_dataset("values", data=np.zeros((len(ts), 7)))
            meta = {
                "schema": schema.to_dict(), "start_time_wall": 10.0,
                "end_time_wall": 10.4, "events": [],
            }
            path = os.path.join(episode_dir, "meta.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(meta, f)
            with self.assertRaisesRegex(ValueError, "sampling gap"):
                load_raw_actions(episode_dir)
            meta["schema"]["max_gap_ms"] = 300
            meta["streams"] = {"left_arm_state": {"count": 99}}
            with open(path, "w", encoding="utf-8") as f:
                json.dump(meta, f)
            with self.assertRaisesRegex(ValueError, "sample count mismatch"):
                load_raw_actions(episode_dir)
            meta["streams"]["left_arm_state"]["count"] = len(ts)
            meta["events"] = [
                {"t": 10.1, "name": "pause", "data": True},
                {"t": 10.2, "name": "pause", "data": False},
            ]
            with open(path, "w", encoding="utf-8") as f:
                json.dump(meta, f)
            with self.assertRaisesRegex(ValueError, "contains a pause"):
                load_raw_actions(episode_dir)


if __name__ == "__main__":
    unittest.main(verbosity=2)
