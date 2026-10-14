#!/usr/bin/env python3
"""Tests for SafeExecutor: nan/inf, clamps, slew-rate safety."""

import unittest

import numpy as np

from astral_policy_inference.executor import SafeExecutor
from astral_policy_inference.robot_io import CmdTarget


def arm(values):
    return CmdTarget("left_arm_cmd", "/left_arm/joint_commands", "joints",
                     np.asarray(values, dtype=np.float64))


def ratio(values):
    return CmdTarget("left_ee_cmd", "/left_gripper/command", "ratio",
                     np.asarray(values, dtype=np.float64))


class TestSafeExecutor(unittest.TestCase):
    def test_ratio_clipped_to_unit_interval(self):
        ex = SafeExecutor()
        out = ex.step([ratio([-0.5]), ratio([1.7])], 0.1)
        self.assertAlmostEqual(out[0].values[0], 0.0)
        self.assertAlmostEqual(out[1].values[0], 1.0)
        kinds = [e.kind for e in ex.events]
        self.assertEqual(kinds, ["clip", "clip"])

    def test_nan_dropped_holds_last(self):
        ex = SafeExecutor()
        out0 = ex.step([arm(np.zeros(7))], 0.1)
        np.testing.assert_allclose(out0[0].values, 0)
        out1 = ex.step([arm([np.nan] * 7)], 0.1)
        np.testing.assert_allclose(out1[0].values, 0)
        self.assertEqual(ex.events[0].kind, "nan")

    def test_nan_without_last_drops_row(self):
        ex = SafeExecutor()
        out = ex.step([arm([np.inf, *np.zeros(6)])], 0.1)
        self.assertEqual(out, [])
        self.assertEqual(ex.events[0].kind, "nan")

    def test_slew_limit_respects_max_joint_vel(self):
        ex = SafeExecutor(max_joint_vel=1.0)
        dt = 0.1
        ex.step([arm(np.zeros(7))], dt)
        out = ex.step([arm(np.full(7, 5.0))], dt)
        np.testing.assert_allclose(out[0].values, np.full(7, 0.1), atol=1e-9)
        self.assertEqual(ex.events[0].kind, "slew")

    def test_slew_disabled_when_zero(self):
        ex = SafeExecutor(max_joint_vel=0.0)
        ex.step([arm(np.zeros(7))], 0.1)
        out = ex.step([arm(np.full(7, 5.0))], 0.1)
        np.testing.assert_allclose(out[0].values, 5.0)

    def test_joint_limits_clip(self):
        lo = np.full(7, -1.0)
        hi = np.full(7, 1.0)
        ex = SafeExecutor(max_joint_vel=0.0, joint_limits={
            "left_arm_cmd": (lo, hi),
        })
        out = ex.step([arm(np.full(7, 3.0))], 0.1)
        np.testing.assert_allclose(out[0].values, 1.0)
        self.assertEqual(ex.events[0].kind, "clip")

    def test_reset_clears_state_and_events(self):
        ex = SafeExecutor(max_joint_vel=1.0)
        ex.step([arm(np.zeros(7))], 0.1)
        ex.step([arm(np.full(7, 5.0))], 0.1)
        self.assertTrue(ex.events)  # drained by property read
        self.assertTrue(ex.last_for("left_arm_cmd") is not None)
        ex.reset()
        self.assertIsNone(ex.last_for("left_arm_cmd"))
        self.assertEqual(ex.events, [])

    def test_invalid_dt_clamped_positive(self):
        ex = SafeExecutor(max_joint_vel=1.0)
        out = ex.step([arm(np.zeros(7))], -3.0)
        self.assertEqual(out[0].values.shape, (7,))


if __name__ == "__main__":
    unittest.main(verbosity=2)
