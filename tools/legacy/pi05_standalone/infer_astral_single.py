#!/usr/bin/env python3
"""Standalone pi0.5 runner for Astral's left arm, gripper and two cameras."""

import argparse
import json
import logging
import select
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from protocol import pack, unpack

LOG = logging.getLogger("pi05_robot")
SLOTS = {"base": "base_0_rgb", "left_wrist": "left_wrist_0_rgb"}


def letterbox_bgr_to_rgb(frame, size=224):
    h, w = frame.shape[:2]
    if h <= 0 or w <= 0:
        raise ValueError("empty camera frame")
    ratio = max(w / size, h / size)
    new_w, new_h = int(w / ratio), int(h / ratio)
    rgb = cv2.cvtColor(cv2.resize(frame, (new_w, new_h),
                                  interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
    top, left = (size - new_h) // 2, (size - new_w) // 2
    return cv2.copyMakeBorder(rgb, top, size - new_h - top,
                              left, size - new_w - left, cv2.BORDER_CONSTANT, value=0)


class Camera:
    def __init__(self, name, settings):
        self.name = name
        self.settings = settings
        self._lock = threading.Lock()
        self._image = None
        self._time = 0.0
        self._stop = threading.Event()
        self._thread = None
        self._cap = None

    def start(self):
        device = self.settings["device"]
        if isinstance(device, str) and device.startswith("/dev/") and not Path(device).exists():
            raise RuntimeError(f"{self.name} device missing: {device}")
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open {self.name}: {device}")
        fourcc = self.settings.get("fourcc")
        if fourcc:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.settings["width"]))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.settings["height"]))
        self._cap.set(cv2.CAP_PROP_FPS, int(self.settings.get("fps", 30)))
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        LOG.info("%s camera %s actual=%dx%d %.1ffps fourcc=%s", self.name, device,
                 int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                 int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                 self._cap.get(cv2.CAP_PROP_FPS),
                 int(self._cap.get(cv2.CAP_PROP_FOURCC)))
        self._thread = threading.Thread(target=self._loop, name=f"camera-{self.name}", daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            now = time.monotonic()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            try:
                image = letterbox_bgr_to_rgb(frame)
            except Exception:
                LOG.exception("%s frame conversion failed", self.name)
                continue
            with self._lock:
                self._image, self._time = image, now

    def latest(self, max_age):
        with self._lock:
            age = time.monotonic() - self._time
            if self._image is None or age > max_age:
                raise RuntimeError(f"{self.name} image stale ({age:.3f}s)")
            return self._image.copy(), age

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._cap is not None:
            self._cap.release()


class Robot:
    def __init__(self, settings, initial_gripper_ratio):
        from astral_robot_sdk import RobotFactory, RobotModel, create_robot_config, LEFT_ARM_IDS
        self.ids = LEFT_ARM_IDS
        self.gripper_ratio = float(initial_gripper_ratio)
        self.last_arm_target = None
        cfg = create_robot_config(robot=RobotModel.ROBOTMAIN,
                                  control_board_ip=settings["board_ip"],
                                  board_cmd_port=int(settings.get("board_port", 5001)),
                                  local_ip=settings.get("local_ip", "0.0.0.0"),
                                  local_port=int(settings.get("local_port", 8081)),
                                  obs_hz=100, ctrl_hz=100)
        self.sdk = RobotFactory.create_robot(cfg)
        self.sdk.connect()

    def read(self, max_age):
        data = self.sdk.get_joint_angles()
        age = time.time() - data.timestamp
        q = np.asarray(data.msg.positions, dtype=np.float32)
        if age < 0 or age > max_age or q.shape != (18,) or not np.isfinite(q).all():
            raise RuntimeError(f"joint feedback invalid/stale: age={age:.3f}s shape={q.shape}")
        return np.r_[q[:7], np.float32(self.gripper_ratio)].astype(np.float32), age

    def assert_ready(self):
        data = self.sdk.get_system_status()
        age = time.time() - data.timestamp
        status = data.msg
        if age < 0 or age > 1 or not status.is_work_mode or not status.is_position_mode or not status.enabled:
            raise RuntimeError(f"robot not ready: status_age={age:.2f}s status={status.to_dict()}")

    def send(self, action, observed_state, period, max_joint_vel):
        a = validate_actions(action.reshape(1, -1))[0].copy()
        previous = self.last_arm_target
        if previous is None:
            previous = np.asarray(observed_state[:7], dtype=np.float32)
        max_step = max_joint_vel * period
        a[:7] = previous + np.clip(a[:7] - previous, -max_step, max_step)
        self.send_arm_only(a[:7])
        rad = 2.5 * (1.0 - float(a[7]))
        self.sdk.set_gripper_angle(rad, right_hand=False)
        self.gripper_ratio = float(a[7])
        return a

    def send_arm_only(self, joints):
        q = np.asarray(joints, dtype=np.float32)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError("left arm target must contain seven finite joint angles")
        self.sdk.set_target_positions({mid: float(value) for mid, value in zip(self.ids, q)})
        self.last_arm_target = q.copy()

    def close(self):
        # SDK's public disconnect() sends DISABLE first. That can make a live
        # arm drop, including after an observation-only run. Stop local threads
        # and close the UDP socket directly; this sends no power/motion command.
        self.sdk._ctx.stop_threads()
        self.sdk._comm.disconnect()
        self.sdk._connected = False


def validate_actions(actions):
    a = np.asarray(actions, dtype=np.float32)
    if a.ndim != 2 or a.shape[1] != 8 or len(a) < 1 or not np.isfinite(a).all():
        raise ValueError(f"invalid actions shape/values: {a.shape}")
    if np.any((a[:, 7] < 0) | (a[:, 7] > 1)):
        raise ValueError("gripper ratio must be in [0,1]")
    return a


def init_path(cfg):
    """Return hardware-convention left joint targets, without ROS sign changes."""
    if "init_waypoints" not in cfg or "init_pose" not in cfg:
        raise ValueError("execute requires init_waypoints and init_pose in config")
    via = np.asarray(cfg["init_waypoints"], dtype=np.float32)
    if via.shape == (7,):
        via = via.reshape(1, 7)
    pose = np.asarray(cfg["init_pose"], dtype=np.float32)
    if via.ndim != 2 or via.shape[1] != 7 or len(via) < 1 or pose.shape != (7,):
        raise ValueError("init_waypoints must be 7D rows and init_pose must be 7D")
    if not np.isfinite(via).all() or not np.isfinite(pose).all():
        raise ValueError("initialization joints must be finite")
    return [*via, pose]


def confirm_word(word, stop):
    if not sys.stdin.isatty():
        raise RuntimeError(f"{word} confirmation requires an interactive terminal")
    print(f"Type {word} and press Enter to continue (anything else aborts): ",
          end="", flush=True)
    while not stop.is_set():
        readable, _, _ = select.select([sys.stdin], [], [], 0.1)
        if readable:
            if sys.stdin.readline().strip() != word:
                raise RuntimeError(f"{word} confirmation denied")
            return
    raise RuntimeError("interrupted before confirmation")


def move_to_init(robot, path, cfg, stop, snapshot):
    """Slew each segment, then require stable measured arrival before advancing."""
    hz = float(cfg.get("control_hz", 30))
    speed = float(cfg.get("init_joint_vel_rad_s", 0.6))
    tolerance = float(cfg.get("init_arrive_tol_rad", 0.05))
    follow_tol = float(cfg.get("init_follow_tol_rad", 0.25))
    timeout = float(cfg.get("init_timeout_s", 40.0))
    settle = float(cfg.get("init_settle_s", 0.3))
    if any(not np.isfinite(x) or x <= 0 for x in (hz, speed, tolerance, follow_tol, timeout, settle)):
        raise ValueError("initialization timing and tolerances must be positive and finite")
    period = 1.0 / hz
    state, _ = robot.read(float(cfg.get("max_state_age_s", 0.5)))
    command = state[:7].copy()
    for number, target in enumerate(path, 1):
        LOG.warning("initialization %d/%d target=%s", number, len(path), target.tolist())
        started = time.monotonic()
        next_tick = started
        in_tolerance_since = None
        last_report = started
        measured = command.copy()
        gap = 0.0
        while not stop.is_set():
            now = time.monotonic()
            if now < next_tick:
                stop.wait(next_tick - now)
            if stop.is_set():
                break
            now = time.monotonic()
            if now - started > timeout:
                measured_error = float(np.max(np.abs(measured - target)))
                command_error = float(np.max(np.abs(command - target)))
                raise RuntimeError(
                    f"initialization segment {number} timed out after {now-started:.1f}s: "
                    f"max measured-target error={measured_error:.3f} rad, "
                    f"max command-target error={command_error:.3f} rad, "
                    f"max measured-command gap={gap:.3f} rad; "
                    f"measured={np.round(measured, 3).tolist()}, "
                    f"command={np.round(command, 3).tolist()}, "
                    f"target={np.round(target, 3).tolist()}"
                )
            robot.assert_ready()
            obs, _ = snapshot()  # joint and both camera streams must stay fresh
            measured = obs["observation/state"][:7]
            gap = float(np.max(np.abs(measured - command)))
            if gap > follow_tol:
                # Hold the last target until the real arm catches up; never
                # build a large unseen command lead while feedback is delayed.
                LOG.warning("initialization segment %d tracking gap %.3f rad; holding", number, gap)
            else:
                command = command + np.clip(target - command, -speed * period, speed * period)
            robot.send_arm_only(command)
            if now - last_report >= 2.0:
                LOG.info(
                    "initialization %d/%d progress: measured=%s command=%s "
                    "target_error=%.3f rad follow_gap=%.3f rad",
                    number, len(path), np.round(measured, 3).tolist(),
                    np.round(command, 3).tolist(),
                    float(np.max(np.abs(measured - target))), gap,
                )
                last_report = now
            if np.max(np.abs(command - target)) <= 1e-5 and np.max(np.abs(measured - target)) <= tolerance:
                if in_tolerance_since is None:
                    in_tolerance_since = now
                if now - in_tolerance_since >= settle:
                    LOG.warning("initialization %d/%d measured at target", number, len(path))
                    break
            else:
                in_tolerance_since = None
            next_tick = max(next_tick + period, now + period)
        if stop.is_set():
            raise RuntimeError("interrupted during initialization")


class Planner:
    def __init__(self, url, snapshot, timeout):
        self.url, self.snapshot, self.timeout = url, snapshot, timeout
        self._request = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._result = None
        self._error = None
        self._thread = threading.Thread(target=self._loop, name="pi05-planner", daemon=True)
        self._thread.start()

    def request(self):
        self._request.set()

    def take(self):
        with self._lock:
            result, error = self._result, self._error
            self._result = self._error = None
        return result, error

    def _loop(self):
        from websockets.sync.client import connect
        try:
            with connect(self.url, open_timeout=self.timeout, close_timeout=1,
                         compression=None, max_size=None) as ws:
                metadata = unpack(ws.recv(timeout=self.timeout))
                # The existing astral_ws server sends only {"model": "pi05"};
                # our standalone server also sends action_dim. Validate the
                # returned array shape in both cases.
                if metadata.get("model") != "pi05" or metadata.get("action_dim", 8) != 8:
                    raise RuntimeError(f"unexpected server metadata: {metadata}")
                while not self._stop.is_set():
                    if not self._request.wait(0.1):
                        continue
                    self._request.clear()
                    try:
                        obs, ages = self.snapshot()
                        started = time.monotonic()
                        ws.send(pack(obs))
                        raw = ws.recv(timeout=self.timeout)
                        if isinstance(raw, str):
                            raise RuntimeError(f"server error: {raw}")
                        response = unpack(raw)
                        actions = validate_actions(response["actions"])
                        server_ms = response.get("server_ms")
                        if server_ms is None:
                            server_ms = (response.get("server_timing") or {}).get("infer_ms")
                        result = (actions, (time.monotonic() - started) * 1000,
                                  server_ms, ages)
                        with self._lock:
                            self._result = result
                    except Exception as exc:
                        with self._lock:
                            self._error = str(exc)
                        break
        except Exception as exc:
            with self._lock:
                self._error = str(exc)

    def close(self):
        self._stop.set()
        self._request.set()
        self._thread.join(timeout=2)


def run(cfg, mode, ready, duration, log_file):
    if not cfg["prompt"].strip():
        raise ValueError("prompt must be nonempty and match training task")
    ratio = float(cfg["initial_gripper_ratio"])
    if not 0 <= ratio <= 1:
        raise ValueError("initial_gripper_ratio must be in [0,1]")
    rows_per_plan = int(cfg.get("replan_after_rows", 10))
    if rows_per_plan < 1 or rows_per_plan >= 50:
        raise ValueError("replan_after_rows must be in [1,49] for a 50-row pi0.5 chunk")
    max_joint_vel = float(cfg.get("max_joint_vel_rad_s", 6.0))
    if not np.isfinite(max_joint_vel) or max_joint_vel <= 0:
        raise ValueError("max_joint_vel_rad_s must be positive and finite")
    if ready:
        raise ValueError("--ready homes all joints and is disabled; prepare WORK/POSITION/enabled externally")
    path = init_path(cfg) if mode == "execute" else None
    cameras = {name: Camera(name, cfg["cameras"][name]) for name in SLOTS}
    robot = None
    planner = None
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    log = open(log_file, "a", buffering=1) if log_file else None
    try:
        for camera in cameras.values():
            camera.start()
        robot = Robot(cfg["robot"], ratio)
        def snapshot():
            state, state_age = robot.read(float(cfg.get("max_state_age_s", 0.5)))
            obs = {"observation/state": state, "prompt": cfg["prompt"]}
            ages = {"state": state_age}
            for name, slot in SLOTS.items():
                img, age = cameras[name].latest(float(cfg.get("max_image_age_s", 0.5)))
                obs[f"observation/camera/{slot}"] = img
                ages[name] = age
            return obs, ages

        preflight_deadline = time.monotonic() + 5
        while True:
            try:
                snapshot()
                if mode == "execute":
                    robot.assert_ready()
                break
            except RuntimeError as exc:
                if time.monotonic() >= preflight_deadline:
                    raise RuntimeError(f"initial observation unavailable: {exc}") from exc
                time.sleep(0.05)

        fps = float(cfg.get("control_hz", 30))
        if fps <= 0 or fps > 100:
            raise ValueError("control_hz must be in (0,100]")
        period = 1 / fps
        if mode == "execute":
            LOG.warning("left arm route: measured pose -> via point(s) -> init pose; gripper is unchanged")
            confirm_word("MOVE", stop)
            move_to_init(robot, path, cfg, stop, snapshot)
            LOG.warning("initialization complete; policy commands remain paused")
            confirm_word("START", stop)
            obs, _ = snapshot()
            robot.assert_ready()
            error = float(np.max(np.abs(obs["observation/state"][:7] - path[-1])))
            if error > float(cfg.get("init_arrive_tol_rad", 0.05)):
                raise RuntimeError(f"left arm left init pose before START: error={error:.3f} rad")
        deadline = time.monotonic() + duration
        if mode != "observe":
            planner = Planner(cfg["server_url"], snapshot, float(cfg.get("server_timeout_s", 3)))
            planner.request()
        actions, index, consumed, generation = None, 0, 0, 0
        next_tick = time.monotonic()
        last_log = 0.0
        while not stop.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_tick:
                stop.wait(next_tick - now)
            now = time.monotonic()
            lag_ms = max(0.0, (now - next_tick) * 1000)
            next_tick += period
            if next_tick < now:
                next_tick = now + period
            try:
                obs, ages = snapshot()
            except RuntimeError as exc:
                raise RuntimeError(f"observation lost; execution stopped: {exc}") from exc
            if mode == "execute":
                robot.assert_ready()
            result, error = planner.take() if planner else (None, None)
            if error:
                raise RuntimeError(f"planning stopped: {error}")
            if result:
                pending, rtt_ms, server_ms, plan_ages = result
                actions = pending
                index = 0
                consumed = 0
                generation += 1
                LOG.info("plan=%d rtt=%.1fms server=%s rows=%d ages=%s",
                         generation, rtt_ms, server_ms, len(actions), plan_ages)
            command = raw_action = None
            if actions is not None and index < len(actions):
                raw_action = actions[index]
                index += 1
                consumed += 1
                if consumed == rows_per_plan and planner:
                    planner.request()
                if mode == "execute":
                    command = robot.send(raw_action, obs["observation/state"], period,
                                         max_joint_vel)
            if log:
                log.write(json.dumps({"t": time.time(), "mode": mode, "plan": generation,
                                      "row": index, "lag_ms": lag_ms, "ages_s": ages,
                                      "state": obs["observation/state"].tolist(),
                                      "raw_action": raw_action.tolist() if raw_action is not None else None,
                                      "command": command.tolist() if command is not None else None}) + "\n")
            if now - last_log >= 1:
                LOG.info("mode=%s plan=%d row=%d lag=%.1fms ages=%s", mode,
                         generation, index, lag_ms, ages)
                last_log = now
            if actions is not None and index >= len(actions) and mode == "execute":
                raise RuntimeError("action chunk exhausted before next plan; stopped")
    finally:
        if planner:
            planner.close()
        if robot:
            robot.close()
        for camera in cameras.values():
            camera.close()
        if log:
            log.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(Path(__file__).with_name("config.example.yaml")))
    ap.add_argument("--mode", choices=("observe", "infer", "execute"), default="observe")
    ap.add_argument("--ready", action="store_true", help="deprecated unsafe option; rejected")
    ap.add_argument("--duration", type=float, default=10)
    ap.add_argument("--log-file", default="")
    args = ap.parse_args()
    if args.duration <= 0:
        ap.error("--duration must be positive")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    run(cfg, args.mode, args.ready, args.duration, args.log_file)


if __name__ == "__main__":
    main()
