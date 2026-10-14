"""Offline checks for the standalone wire format and Astral observation layout."""

import json
import sys
import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from protocol import pack, unpack
from infer_astral_single import letterbox_bgr_to_rgb, validate_actions
from serve_pi05 import validate_request
import infer_astral_single as runner


class StandaloneTests(unittest.TestCase):
    def test_wire_roundtrip(self):
        request = {"observation/state": np.arange(8, dtype=np.float32),
                   "observation/camera/base_0_rgb": np.zeros((224, 224, 3), dtype=np.uint8),
                   "prompt": "pick up the container"}
        decoded = unpack(pack(request))
        np.testing.assert_array_equal(decoded["observation/state"], request["observation/state"])
        self.assertEqual(decoded["observation/camera/base_0_rgb"].shape, (224, 224, 3))

    def test_letterbox_geometry_and_rgb(self):
        frame = np.zeros((2, 4, 3), np.uint8)
        frame[:, :, 0] = 255  # blue in BGR
        out = letterbox_bgr_to_rgb(frame, size=8)
        self.assertEqual(out.shape, (8, 8, 3))
        self.assertTrue(np.all(out[:2] == 0))
        self.assertTrue(np.all(out[2:6, :, 2] == 255))
        self.assertTrue(np.all(out[6:] == 0))

    def test_request_and_action_contract(self):
        request = {"observation/state": np.zeros(8, np.float32), "prompt": "red container"}
        for slot in ("base_0_rgb", "left_wrist_0_rgb"):
            request[f"observation/camera/{slot}"] = np.zeros((224, 224, 3), np.uint8)
        self.assertEqual(validate_request(request)["observation/state"].shape, (8,))
        self.assertEqual(validate_actions(np.zeros((50, 8), np.float32)).shape, (50, 8))
        with self.assertRaises(ValueError):
            validate_actions(np.zeros((50, 7), np.float32))
        request["prompt"] = ""
        with self.assertRaises(ValueError):
            validate_request(request)

    def test_control_loop_with_fake_devices(self):
        class FakeCamera:
            def __init__(self, name, settings):
                self.name = name
            def start(self):
                pass
            def latest(self, max_age):
                return np.zeros((224, 224, 3), np.uint8), 0.001
            def close(self):
                pass

        class FakeRobot:
            instance = None
            def __init__(self, settings, ratio):
                self.commands = []
                self.init_commands = []
                self.state = np.zeros(8, np.float32)
                FakeRobot.instance = self
            def read(self, max_age):
                return self.state.copy(), 0.001
            def assert_ready(self):
                pass
            def send_arm_only(self, joints):
                self.init_commands.append(np.asarray(joints).copy())
                self.state[:7] = joints
            def send(self, action, observed_state, period, max_joint_vel):
                self.commands.append(action.copy())
                return action
            def close(self):
                pass

        class FakePlanner:
            def __init__(self, url, snapshot, timeout):
                self.pending = False
            def request(self):
                self.pending = True
            def take(self):
                if not self.pending:
                    return None, None
                self.pending = False
                return (np.zeros((50, 8), np.float32), 1.0, 0.5, {}), None
            def close(self):
                pass

        config = {"prompt": "red container", "initial_gripper_ratio": 0,
                  "cameras": {"base": {}, "left_wrist": {}}, "robot": {},
                  "server_url": "ws://unused", "control_hz": 100, "replan_after_rows": 10,
                  "init_waypoints": [0.01] * 7, "init_pose": [0.02] * 7,
                  "init_joint_vel_rad_s": 1, "init_settle_s": 0.01}
        with TemporaryDirectory() as tmp, patch.object(runner, "Camera", FakeCamera), \
                patch.object(runner, "Robot", FakeRobot), \
                patch.object(runner, "Planner", FakePlanner), \
                patch.object(runner, "confirm_word") as confirm:
            runner.run(config, "execute", False, 0.15, str(Path(tmp) / "log.jsonl"))
            self.assertEqual([call.args[0] for call in confirm.call_args_list], ["MOVE", "START"])
            self.assertGreaterEqual(len(FakeRobot.instance.commands), 10)
            self.assertGreaterEqual(len(FakeRobot.instance.init_commands), 4)
            np.testing.assert_allclose(FakeRobot.instance.init_commands[-1], [0.02] * 7)
            records = (Path(tmp) / "log.jsonl").read_text().splitlines()
            self.assertEqual(len(records), len(FakeRobot.instance.commands))
            runner.run(config, "infer", False, 0.2, str(Path(tmp) / "infer.jsonl"))
            self.assertEqual(FakeRobot.instance.commands, [])
            infer_records = [json.loads(s) for s in
                             (Path(tmp) / "infer.jsonl").read_text().splitlines()]
            self.assertTrue(any(r["raw_action"] is not None for r in infer_records))
            self.assertTrue(all(r["command"] is None for r in infer_records))

    def test_execute_requires_init_path_and_rejects_ready_before_devices(self):
        cfg = {"prompt": "task", "initial_gripper_ratio": 0,
               "cameras": {"base": {}, "left_wrist": {}}, "robot": {}}
        with self.assertRaisesRegex(ValueError, "init_waypoints"):
            runner.run(cfg, "execute", False, 1, "")
        with self.assertRaisesRegex(ValueError, "--ready"):
            runner.run(cfg, "execute", True, 1, "")
        cfg["init_waypoints"] = [-1.6, .2, 0, -1.92, -.2, 0, 0]
        cfg["init_pose"] = [-.35, .2, 0, -1.92, -.2, 0, 0]
        path = runner.init_path(cfg)
        self.assertEqual(len(path), 2)
        np.testing.assert_allclose(path[0], cfg["init_waypoints"])
        np.testing.assert_allclose(path[1], cfg["init_pose"])

    def test_init_does_not_advance_without_measured_arrival(self):
        class StalledRobot:
            def __init__(self):
                self.commands = []
            def read(self, max_age):
                return np.zeros(8, np.float32), 0.001
            def assert_ready(self):
                pass
            def send_arm_only(self, joints):
                self.commands.append(np.asarray(joints).copy())
        robot = StalledRobot()
        cfg = {"control_hz": 100, "init_joint_vel_rad_s": 10,
               "init_follow_tol_rad": 0.1, "init_timeout_s": 0.08,
               "init_settle_s": 0.01}
        snapshot = lambda: ({"observation/state": np.zeros(8, np.float32)}, {})
        with self.assertRaisesRegex(RuntimeError, "segment 1 timed out"):
            runner.move_to_init(robot, [np.ones(7, np.float32),
                                        np.ones(7, np.float32) * 2],
                                cfg, threading.Event(), snapshot)
        self.assertTrue(robot.commands)
        self.assertTrue(all(np.max(command) <= 0.101 for command in robot.commands))

    def test_left_arm_command_mapping_and_rate_limit(self):
        class FakeSDK:
            def __init__(self):
                self.positions = None
                self.gripper = None
            def set_target_positions(self, values):
                self.positions = values
            def set_gripper_angle(self, value, right_hand):
                self.gripper = value, right_hand
        robot = runner.Robot.__new__(runner.Robot)
        robot.ids = list(range(1, 8))
        robot.sdk = FakeSDK()
        robot.gripper_ratio = 0
        robot.last_arm_target = None
        raw = np.r_[np.ones(7, np.float32), np.float32(1)]
        actual = robot.send(raw, np.zeros(8, np.float32), 1 / 30, 6)
        self.assertEqual(set(robot.sdk.positions), set(range(1, 8)))
        np.testing.assert_allclose(actual[:7], 0.2)
        self.assertEqual(robot.sdk.gripper, (0.0, False))
        self.assertEqual(robot.gripper_ratio, 1.0)

    def test_close_never_disables_robot(self):
        events = []
        class Context:
            def stop_threads(self):
                events.append("stop")
        class Comm:
            def disconnect(self):
                events.append("socket_close")
        class SDK:
            _ctx = Context()
            _comm = Comm()
            _connected = True
            def disconnect(self):
                raise AssertionError("public SDK disconnect sends DISABLE")
        robot = runner.Robot.__new__(runner.Robot)
        robot.sdk = SDK()
        robot.close()
        self.assertEqual(events, ["stop", "socket_close"])


if __name__ == "__main__":
    unittest.main()
