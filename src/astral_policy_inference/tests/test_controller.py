#!/usr/bin/env python3
"""Tests for the mode state machine (arbitration logic)."""

import unittest

from astral_policy_inference.controller import Controller, InvalidTransition


def fresh():
    return Controller()


class TestBasicFlows(unittest.TestCase):
    def test_idle_to_policy_and_pause_cycle(self):
        c = fresh()
        self.assertEqual(c.request("policy"), "POLICY")
        self.assertEqual(c.request("pause"), "POLICY_PAUSED")
        self.assertEqual(c.request("resume"), "POLICY")

    def test_playback_pause_resume(self):
        c = fresh()
        c.request("playback")
        self.assertEqual(c.state, "PLAYBACK")
        c.request("pause")
        self.assertEqual(c.state, "PLAYBACK_PAUSED")
        c.request("resume")
        self.assertEqual(c.state, "PLAYBACK")

    def test_takeover_and_release_restores_activity(self):
        c = fresh()
        c.request("policy")
        c.request("pause")
        c.request("resume")
        c.request("takeover")
        self.assertEqual(c.state, "HUMAN")
        self.assertEqual(c.activity, "human")
        c.request("release")
        self.assertEqual(c.state, "POLICY")

    def test_release_keeps_pause_flag_of_interrupted_activity(self):
        c = fresh()
        c.request("policy")
        c.request("pause")          # POLICY_PAUSED
        c.request("takeover")       # HUMAN (interrupted POLICY_PAUSED)
        c.request("release")
        self.assertEqual(c.state, "POLICY_PAUSED")
        self.assertTrue(c.snapshot()["paused"])

    def test_release_from_takeover_of_playback(self):
        c = fresh()
        c.request("playback")
        c.request("pause")
        c.request("takeover")
        c.request("release")
        self.assertEqual(c.state, "PLAYBACK_PAUSED")
        self.assertTrue(c.snapshot()["paused"])

    def test_stop_from_anywhere_to_idle(self):
        for path in (["policy"], ["playback"], ["policy", "pause"],
                     ["playback", "pause", "takeover"]):
            c = fresh()
            for verb in path:
                c.request(verb)
            self.assertEqual(c.request("stop"), "IDLE")
            self.assertIsNone(c.activity)
            self.assertFalse(c.snapshot()["paused"])

    def test_start_activity_from_human(self):
        c = fresh()
        c.request("policy")
        c.request("takeover")
        self.assertEqual(c.request("playback"), "PLAYBACK")
        self.assertEqual(c.activity, "playback")

    def test_switch_activity_from_another(self):
        c = fresh()
        c.request("policy")
        self.assertEqual(c.request("playback"), "PLAYBACK")


class TestInvalidTransitions(unittest.TestCase):
    def _raises(self, verbs, from_path=()):
        c = fresh()
        for verb in from_path:
            c.request(verb)
        for verb in verbs:
            with self.assertRaises(InvalidTransition):
                c.request(verb)

    def test_pause_only_while_active(self):
        self._raises(["pause"], [])
        self._raises(["pause"], ["policy", "pause"])

    def test_resume_only_while_paused(self):
        self._raises(["resume"], [])
        self._raises(["resume"], ["policy"])

    def test_takeover_needs_an_activity(self):
        self._raises(["takeover"], [])

    def test_release_only_from_human(self):
        self._raises(["release"], [])
        self._raises(["release"], ["policy"])

    def test_unknown_verb(self):
        c = fresh()
        with self.assertRaises(InvalidTransition):
            c.request("launch")

    def test_state_unchanged_on_rejection(self):
        c = fresh()
        c.request("policy")
        try:
            c.request("resume")  # not paused
        except InvalidTransition:
            pass
        self.assertEqual(c.state, "POLICY")


class TestRevert(unittest.TestCase):
    def test_revert_restores_pre_request_state(self):
        c = fresh()
        c.request("policy")          # POLICY
        c.request("takeover")        # HUMAN (in case start would fail)
        c.revert()
        self.assertEqual(c.state, "POLICY")

    def test_revert_after_failed_policy_from_idle(self):
        c = fresh()
        with self.assertRaises(Exception):
            raise Exception
        c.request("policy")
        c.revert()
        self.assertEqual(c.state, "IDLE")

    def test_snapshot_shape(self):
        c = fresh()
        c.request("policy")
        snap = c.snapshot()
        self.assertEqual(set(snap), {"state", "activity", "paused"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
