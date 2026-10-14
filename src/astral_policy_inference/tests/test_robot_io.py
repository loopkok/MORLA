#!/usr/bin/env python3
"""Tests for schema-driven obs/action layout & image helpers."""

import unittest

import cv2
import numpy as np

from astral_data_collect.schema import CollectSchema
from astral_policy_inference.robot_io import (
    ObsLayout,
    ObsSource,
    UnsupportedActionBlockError,
    decode_jpeg_rgb,
    letterbox,
    split_camera_map,
)


def schema(**kw):
    base = dict(
        arms=["left"],
        end_effector_left="gripper",
        end_effector_right="none",
        include_waist=False,
        include_head=False,
        cameras=["base", "left_wrist", "right_wrist"],
        dataset_fps=30,
    )
    base.update(kw)
    return CollectSchema(**base)


class TestSources(unittest.TestCase):
    def test_single_left_gripper_layout(self):
        lay = ObsLayout(schema())
        self.assertEqual(lay.state_dim, 8)
        keys = set(lay.sources)
        self.assertEqual(keys, {"left_arm_state", "left_gripper_ratio"})
        arm = lay.sources["left_arm_state"]
        self.assertEqual((arm.topic, arm.dim, arm.kind),
                         ("/left_arm/joint_states", 7, "joints"))
        grip = lay.sources["left_gripper_ratio"]
        self.assertEqual((grip.topic, grip.dim, grip.kind),
                         ("/left_gripper/command", 1, "ratio"))

    def test_dual_arm_wuji_and_head_sources(self):
        lay = ObsLayout(schema(
            arms=["left", "right"],
            end_effector_right="wuji",
            include_head=True,
        ))
        self.assertIn("right_hand_state", lay.sources)
        self.assertEqual(lay.sources["right_hand_state"].dim, 20)
        self.assertIn("body_state", lay.sources)

    def test_invalid_source_key_rejected(self):
        with self.assertRaises(ValueError):
            ObsSource("waist", "/astral/joint_states", 2, "body")

    def test_assemble_missing_and_full(self):
        lay = ObsLayout(schema())
        state, missing = lay.assemble_state({})
        self.assertIsNone(state)
        self.assertIn("left_arm_state", missing)
        full = {
            "left_arm_state": np.arange(7, dtype=np.float64),
            "left_gripper_ratio": np.array([0.5]),
        }
        state, missing = lay.assemble_state(full)
        self.assertEqual(missing, [])
        self.assertEqual(state.shape, (8,))
        np.testing.assert_allclose(state[:7], np.arange(7))

    def test_head_falls_back_to_head_state(self):
        lay = ObsLayout(schema(include_head=True))
        state, missing = lay.assemble_state({
            "left_arm_state": np.zeros(7),
            "left_gripper_ratio": np.array([0.5]),
            "head_state": np.full(2, 3.0),
        })
        self.assertIsNotNone(state)
        self.assertEqual(missing, [])
        self.assertAlmostEqual(state[-1], 3.0)

    def test_waist_head_sliced_from_body_state(self):
        lay = ObsLayout(schema(include_waist=True, include_head=True))
        body = np.zeros(18)
        body[14:16] = [0.2, -0.3]   # waist
        body[16:18] = [0.4, 0.5]    # head
        state, missing = lay.assemble_state({
            "left_arm_state": np.full(7, 1.0),
            "left_gripper_ratio": np.array([0.5]),
            "body_state": body,
        })
        self.assertEqual(missing, [])
        self.assertEqual(state.shape, (12,))  # 7 arm + 1 ee + 2 waist + 2 head
        np.testing.assert_allclose(state[:7], np.full(7, 1.0))
        self.assertAlmostEqual(state[8], 0.2)     # waist_0
        self.assertAlmostEqual(state[9], -0.3)    # waist_1
        self.assertAlmostEqual(state[10], 0.4)    # head_yaw
        self.assertAlmostEqual(state[11], 0.5)    # head_pitch


class TestSplitAction(unittest.TestCase):
    def test_split_single_left_gripper(self):
        lay = ObsLayout(schema())
        action = np.arange(8, dtype=np.float64)
        targets = lay.split_action(action)
        kinds = [(t.stream, t.kind) for t in targets]
        self.assertEqual(kinds, [("left_arm_cmd", "joints"),
                                 ("left_ee_cmd", "ratio")])
        self.assertEqual(targets[0].topic, "/left_arm/joint_commands")
        self.assertEqual(targets[1].topic, "/left_gripper/command")
        np.testing.assert_allclose(targets[0].values, np.arange(7))
        np.testing.assert_allclose(targets[1].values, np.array([7.0]))

    def test_split_dim_mismatch_raises(self):
        lay = ObsLayout(schema())
        with self.assertRaises(ValueError):
            lay.split_action(np.zeros(9))

    def test_waist_block_is_not_writable(self):
        lay = ObsLayout(schema(include_waist=True))
        with self.assertRaises(UnsupportedActionBlockError):
            lay.split_action(np.zeros(lay.state_dim))

    def test_split_wuji_hand_target_is_joints(self):
        lay = ObsLayout(schema(end_effector_left="wuji"))
        targets = lay.split_action(np.zeros(lay.state_dim))
        self.assertEqual(targets[-1].topic, "/left_hand/joint_commands")
        self.assertEqual(targets[-1].kind, "joints")


class TestImageHelpers(unittest.TestCase):
    def test_letterbox_keeps_channels_and_size(self):
        img = np.random.randint(0, 255, (30, 12, 3), dtype=np.uint8)
        out = letterbox(img, 32)
        self.assertEqual(out.shape, (32, 32, 3))

    def test_letterbox_identity_when_already_square(self):
        img = np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8)
        out = letterbox(img, 64)
        self.assertTrue(np.array_equal(img, out))

    def test_decode_roundtrip_rgb(self):
        img = np.zeros((24, 24, 3), dtype=np.uint8)
        img[:, :, 0] = 200  # red channel
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        self.assertTrue(ok)
        rgb = decode_jpeg_rgb(buf.tobytes())
        self.assertIsNotNone(rgb)
        self.assertEqual(rgb.shape, img.shape)
        self.assertGreater(rgb[:, :, 0].mean(), 150)  # red came back in channel 0

    def test_decode_garbage_returns_none(self):
        self.assertIsNone(decode_jpeg_rgb(b"not a jpeg"))

    def test_split_camera_map_keeps_slot_to_label(self):
        rows = split_camera_map({"base_0_rgb": "base"}, 224)
        self.assertEqual(rows, [("base_0_rgb", "base", 224)])


if __name__ == "__main__":
    unittest.main(verbosity=2)
