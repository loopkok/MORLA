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
                    "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
