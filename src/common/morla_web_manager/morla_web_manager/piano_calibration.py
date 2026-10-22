"""Bounded manual calibration; reuses Nero FK and native work-position services.

The public boundary uses degrees/mm. The robot transport remains native radians.
No ROS/NumPy import is needed to validate an HTTP request or read a saved config.
"""
from __future__ import annotations

import copy
import math
import time

WORK_POSE_DEG = {
    "left": [-25.915, 90.973, -37.488, 72.655, 155., -40.201, 5.965],
    "right": [28.249, 98., 38.828, 58.867, -155., 46.206, -3.757],
}
MANUAL_OPERATIONS = {"joint", "hand_pose", "cartesian", "work_pose"}
WORK_WAYPOINT_TOLERANCE_RAD = math.radians(3.)


def work_waypoint_arrived(measured, target):
    return (len(measured) == len(target) == 7
            and all(math.isfinite(a) and math.isfinite(b) for a, b in zip(measured, target))
            and max(abs(a-b) for a, b in zip(measured, target)) <= WORK_WAYPOINT_TOLERANCE_RAD)


def scalar(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} 必须为 {low} 至 {high} 之间的有限数值")
    return value


def angle_degrees(value, label, bounds_rad):
    # Compare the exact value sent to the driver, avoiding a rad -> deg round
    # trip that can incorrectly reject the documented -60 degree endpoint.
    low, high = bounds_rad
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= math.radians(value) <= high):
        raise ValueError(f"{label} 必须为 {math.degrees(low):g} 至 {math.degrees(high):g} 之间的有限数值")
    return value


def work_poses(config):
    calibration = config.get("calibration", {})
    if not isinstance(calibration, dict):
        raise ValueError("calibration 必须为对象")
    return copy.deepcopy(calibration.get("work_pose_deg", WORK_POSE_DEG))


def validate_work_poses(config):
    poses = work_poses(config)
    if not isinstance(poses, dict) or set(poses) != {"left", "right"}:
        raise ValueError("钢琴工作位须包含左右臂")
    for side, pose in poses.items():
        if not isinstance(pose, list) or len(pose) != 7:
            raise ValueError("钢琴工作位需要7个原生关节角，单位度")
        for i, (value, bounds) in enumerate(zip(pose, config["hands"][side]["arm_limits_rad"])):
            angle_degrees(value, f"{side} 工作位 J{i+1}", bounds)
    return poses


def validate_command(operation, message, config):
    """Identical validation at HTTP and execution boundaries."""
    if message.get("calibration_confirmed") is not True:
        raise ValueError("手动标定运动须明确勾选允许标定试动")
    if operation == "work_pose":
        validate_work_poses(config)
        side = message.get("side")
        if side not in ("left", "right", "both"):
            raise ValueError("工作位请选择左臂、右臂或双臂")
        return dict(side=side)
    side = message.get("side")
    if side not in ("left", "right"):
        raise ValueError("请选择左手或右手")
    if operation == "hand_pose":
        values = message.get("target_deg")
        if not isinstance(values, list) or len(values) != 12:
            raise ValueError("悬停目标须包含12个角度")
        for value, bounds in zip(values, config["hands"][side]["hand_limits_rad"]):
            angle_degrees(value, "手姿态目标角度", bounds)
        return dict(side=side, target_deg=list(values))
    if operation == "joint":
        part, index = message.get("part"), message.get("joint_index")
        if part not in ("arm", "hand") or isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < (7 if part == "arm" else 12):
            raise ValueError("无效机械臂/灵巧手关节索引")
        bounds = config["hands"][side][part + "_limits_rad"][index]
        angle = angle_degrees(message.get("target_deg"), "关节角度", bounds)
        return dict(side=side, part=part, joint_index=index, target_deg=angle)
    if operation == "cartesian":
        axis, frame = message.get("axis"), message.get("frame")
        if axis not in ("x", "y", "z", "rx", "ry", "rz") or frame not in ("base", "tool"):
            raise ValueError("无效点动轴或坐标系")
        step = scalar(message.get("step"), "点动步长 mm/deg", -5 if len(axis) == 1 else -3, 5 if len(axis) == 1 else 3)
        if step == 0:
            raise ValueError("点动步长不能为零")
        return dict(side=side, axis=axis, frame=frame, step=step)
    raise ValueError("未知手动标定操作")


class CalibrationMotion:
    def __init__(self, engine, backend, config, software=False):
        self.engine, self.backend, self.config, self.software = engine, backend, config, software
        self.path_active = False
        self.progress = None
        self._ik = None
        self.kinematics_error = None
        self.work_sides = ("left", "right")
        self.work_arm_only = False

    def ik(self):
        if self._ik is None:
            from nero_arm_teleop.ik_solver import IKSolver
            self._ik = IKSolver()
        return self._ik

    def telemetry(self):
        positions = copy.deepcopy(self.backend.measured)
        now = time.monotonic()
        ages = {key: (max(0., now - self.backend.received[key]) if key in getattr(self.backend, "received", {}) else None) for key in positions}
        poses = {}
        try:
            import numpy as np
            from scipy.spatial.transform import Rotation
            for side in ("left", "right"):
                if side + "_arm" in positions:
                    transform = self.ik().fk(positions[side + "_arm"])
                    poses[side] = dict(xyz_mm=(transform[:3, 3] * 1000).tolist(),
                                       rpy_deg=Rotation.from_matrix(transform[:3, :3]).as_euler("xyz", degrees=True).tolist(),
                                       base_frame=side + "_base_link", tip_frame=side + "_link7")
            self.kinematics_error = None
        except Exception as exc:
            self.kinematics_error = str(exc)
        command_targets = (copy.deepcopy(self.engine.targets) if self.backend.claimed
                           and self.engine.state != "FAULT" and not self.path_active
                           and self.engine.targets is not None else {})
        return dict(positions=positions, command_targets=command_targets, feedback_age_s=ages, poses=poses,
                    kinematics_error=self.kinematics_error, work_progress=self.progress)

    def claim_at_measured(self, keys=None):
        # Only a first takeover seeds from feedback. During an uninterrupted
        # hold, measured joint error is not a new command: rebasing every click
        # would move all holding joints by their tracking error, repeatedly.
        e = self.engine
        scope = frozenset(self.backend.measured if keys is None else keys)
        keep = (self.backend.claimed and e.state != "FAULT" and e.targets is not None
                and getattr(self.backend, "motion_keys", None) == scope
                and frozenset(e.targets) == scope)
        previous = copy.deepcopy(e.targets) if keep else None
        if keys is None:
            self.backend.claim()
        else:
            self.backend.claim(keys=keys)
        e._check()
        e.targets = previous if keep else e._command_snapshot()
        e.log("manual_control_seed", source="retained_command" if keep else "measured_takeover",
              scope=sorted(scope), targets_rad=copy.deepcopy(e.targets))
        self.engine._manual_calibration = True
        self.engine.pressed = None
        self.engine.state = "CALIBRATING"
        self.engine.error = None

    def hold_measured(self):
        self.engine.targets = self.engine._command_snapshot()
        self.engine._send()
        self.engine.pressed = None
        self.engine.state = "STOPPED"
        self.engine.log("manual_stop_hold")

    def move_targets(self, targets, minimum=0.3, arm_speed=None, hand_speed=.25, arm_arrival=False,
                     joint_finish_ratios=None):
        e, c = self.engine, self.config
        if arm_arrival and any(not key.endswith("_arm") for key in targets):
            raise ValueError("arm arrival tolerance cannot be used for hand targets")
        start = copy.deepcopy(e.targets)
        finish = {key: [1.] * len(goal) for key, goal in targets.items()}
        for key, joints in (joint_finish_ratios or {}).items():
            if key not in finish:
                raise ValueError("early-finish joint outside motion scope")
            for index, ratio in joints.items():
                if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(finish[key]):
                    raise ValueError("invalid early-finish joint")
                finish[key][index] = scalar(ratio, "关节提前完成比例", .1, 1)
        duration = minimum
        for key, goal in targets.items():
            side, part = key.split("_")
            from morla_piano.model import check_limits
            check_limits(goal, c["hands"][side][part + "_limits_rad"], key)
            speed = (arm_speed or c["timing"]["max_arm_velocity_rad_s"]) if part == "arm" else hand_speed
            duration = max(duration, max(1.5 * abs(a-b) / (speed*r)
                                        for a, b, r in zip(start[key], goal, finish[key])))
        from morla_piano.engine import smooth
        e.log("manual_target", targets_rad=copy.deepcopy(targets), start_rad={k:list(start[k]) for k in targets}, duration_s=duration,
              joint_finish_ratios=copy.deepcopy(joint_finish_ratios or {}))
        begin = e.clock.now()
        while e.clock.now() - begin < duration:
            e._check()
            measured = self.backend.snapshot()
            if any(max(abs(a-b) for a, b in zip(e.targets[key], measured[key])) > c["timing"]["max_tracking_error_rad"] for key in targets):
                raise RuntimeError("标定运动实测跟踪误差超限")
            elapsed = (e.clock.now() - begin) / duration
            for key, goal in targets.items():
                e.targets[key] = [a + smooth(elapsed/r) * (b-a)
                                  for a, b, r in zip(start[key], goal, finish[key])]
            e._send()
            e.clock.sleep(1 / c["timing"]["control_hz"])
        e.targets.update(copy.deepcopy(targets))
        if arm_arrival:
            e._await_arrival(targets, tolerance_rad=WORK_WAYPOINT_TOLERANCE_RAD, dwell_s=.2)
        else:
            e._await_arrival(targets)

    def return_piano_home(self):
        """Piano-only return: zero, J4=95, J1=0, J2=60, then HOME.

        All targets are native angles. Derive bilateral signs once; never
        mirror the independently calibrated piano zero or measured feedback.
        """
        from morla_piano.model import SIDES, FINGERS, arm_target, check_limits
        from morla_piano.engine import Stopped
        from nero_robot_control.joint_conventions import NERO_BILATERAL_SIGNS
        from nero_robot_control.home_trajectory import HOME_FINAL_POSE_DEG
        e = self.engine
        zero = {s+'_arm': arm_target(self.config, s, 0) for s in SIDES}
        stages = [("演奏零位", copy.deepcopy(zero))]
        current = copy.deepcopy(zero)
        for index, value, name in ((3, 95., "J4到95°"), (0, 0., "J1到0°"), (1, 60., "J2到60°")):
            for side in SIDES:
                sign = NERO_BILATERAL_SIGNS[index] if side == 'left' else 1
                current[side+'_arm'][index] = math.radians(value * sign)
            stages.append((name, copy.deepcopy(current)))
        final = {s+'_arm': [math.radians(v) for v in HOME_FINAL_POSE_DEG] for s in SIDES}
        stages.append(("起始位（J4提前完成）", final))
        # Check the entire route before claiming or issuing the first target.
        for _, targets in stages:
            for key, goal in targets.items():
                check_limits(goal, self.config['hands'][key.split('_')[0]]['arm_limits_rad'], key)
        try:
            self.claim_at_measured()
            for side in SIDES:
                for finger in FINGERS:
                    e._finger(side, finger, False)
            e._send()
            e._await_arrival({s+'_hand': e.targets[s+'_hand'] for s in SIDES})
            for index, (name, targets) in enumerate(stages, 1):
                self.progress = dict(stage=index, total=len(stages), mode='piano_return', side='both', name=name)
                e.log('home_stage', **self.progress)
                early = {key: {3: .8} for key in targets} if index == len(stages) else None
                self.move_targets(targets, minimum=3, arm_arrival=True, joint_finish_ratios=early)
            e.state, e.error, e.pressed = 'STOPPED', None, None
            e.log('home_complete', route='piano_return', final_pose_deg=list(HOME_FINAL_POSE_DEG), j4_finish_ratio=.8)
        except Stopped:
            self.hold_measured()
            raise
        except Exception as exc:
            e._fault(exc)
            raise
        finally:
            self.progress = None

    def cartesian_path(self, positions, side, axis, frame, step):
        """Precompute the entire short Cartesian segment before any robot output."""
        import numpy as np
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        solver = self.ik()
        q = np.array(positions[side + "_arm"], dtype=float)
        origin = solver.fk(q)
        bounds = np.array(self.config["hands"][side]["arm_limits_rad"])
        if np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]):
            raise ValueError("当前实测角在严格目标限位外，不能做笛卡尔点动")
        count = max(1, math.ceil(abs(step) / (1 if len(axis) == 1 else .25)))
        path = []
        for i in range(1, count + 1):
            goal = origin.copy()
            increment = np.zeros(3)
            increment[("xyz".index(axis) if len(axis) == 1 else "xyz".index(axis[1]))] = step * i / count
            if len(axis) == 1:
                goal[:3, 3] += (origin[:3, :3] @ increment if frame == "tool" else increment) / 1000
            else:
                rotation = Rotation.from_rotvec(np.deg2rad(increment)).as_matrix()
                goal[:3, :3] = origin[:3, :3] @ rotation if frame == "tool" else rotation @ origin[:3, :3]
            seed = q.copy()
            def residual(candidate):
                actual = solver.fk(candidate)
                return np.r_[10 * (actual[:3, 3] - goal[:3, 3]),
                             Rotation.from_matrix(goal[:3, :3] @ actual[:3, :3].T).as_rotvec(),
                             .0001 * (candidate - seed)]
            result = least_squares(residual, q, bounds=(bounds[:, 0], bounds[:, 1]), max_nfev=80,
                                   ftol=1e-10, xtol=1e-10, gtol=1e-10)
            q = result.x
            actual = solver.fk(q)
            if (not result.success or np.linalg.norm(actual[:3, 3] - goal[:3, 3]) > .0001
                    or Rotation.from_matrix(goal[:3, :3] @ actual[:3, :3].T).magnitude() > .001
                    or max(abs(q - seed)) > math.radians(3)):
                raise ValueError("点动无连续可用逆解或临近奇异位形；请减小步长或先调整关节")
            path.append(q.tolist())
        return path

    def path_health(self):
        """J mode intentionally revokes JS permission; all other checks stay active."""
        b, c = self.backend, self.config
        if b.fault:
            raise RuntimeError(b.fault)
        keys = {s+"_"+p for s in self.work_sides for p in (("arm",) if self.work_arm_only else ("arm", "hand"))}
        errors = [v for k,v in getattr(b, "input_errors", {}).items() if k.split('/')[0] in keys]
        if errors:
            raise RuntimeError("; ".join(errors))
        now = time.monotonic()
        for side in self.work_sides:
            for part in (("arm",) if self.work_arm_only else ("arm", "hand")):
                key = side + "_" + part
                status = b.statuses.get(key, {})
                if now - b.received.get(key, -math.inf) > c["timing"]["feedback_timeout_s"] or now - b.status_at.get(key, -math.inf) > 1.5 or status.get("feedback_fresh") is not True:
                    raise RuntimeError(key + " 工作位反馈过期")
                if part == "arm":
                    if (status.get("backend") != b.expected_backend or not status.get("connected")
                            or not status.get("motors_enabled") or not status.get("motor_status_fresh")):
                        raise RuntimeError(key + " 工作位电机状态异常")
                    if not self.software and b.expected_backend == "hardware" and (status.get("controller_status_fresh") is not True or status.get("controller_arm_status") != 0):
                        raise RuntimeError(key + " 工作位控制器状态异常")
                elif not status.get("lease_fresh") or status.get("lease_activity") != "playback" or status.get("rejections", 0) > b.baseline_rejections.get(side, 0):
                    raise RuntimeError(key + " 工作位手部控制权异常")

    def native_work_path(self, sides=("left", "right"), *, home=False):
        from nero_robot_control.waypoint_trajectory import TRAJECTORY_REVISION, waypoint_targets
        from nero_robot_control.home_trajectory import HOME_FINAL_POSE_DEG, home_targets
        from morla_piano.model import check_limits
        kind = "home" if home else "workpos"
        label = "回起始位" if home else "工作位"
        event = "home_stage" if home else "work_pose_stage"
        count = 5 if home else 4
        verbs = (("home", "home_point3", "home_point2", "home_point1", "home_final_pose") if home
                 else ("position", "position_point2", "position_point3", "position_point4"))
        # Freeze and validate every waypoint before claim/enable or the first segment.
        paths = {}
        self.work_sides = tuple(sides)
        self.work_arm_only = len(sides) == 1
        for side in sides:
            if self.software:
                paths[side] = waypoint_targets(side)
            else:
                status = self.backend.statuses[side + "_arm"]
                if status.get("trajectory_revision") != TRAJECTORY_REVISION:
                    raise ValueError("驱动工作位轨迹版本不匹配，请重启/更新驱动")
                paths[side] = waypoint_targets(side, status.get("workpos_waypoints_deg", []))
            if home:
                if not self.software and self.backend.statuses[side+"_arm"].get("home_final_pose_deg") != list(HOME_FINAL_POSE_DEG):
                    raise ValueError(side + " HOME 起始位版本不匹配，请更新驱动")
                paths[side] = home_targets(side, None if self.software else status.get("workpos_waypoints_deg", []))
            for q in paths[side]:
                check_limits(q, self.config["hands"][side]["arm_limits_rad"], side)
        self.claim_at_measured(keys=[s+"_arm" for s in sides] if self.work_arm_only else None)
        try:
            if home:
                # Clear fingers before either arm leaves the keyboard.
                from morla_piano.model import FINGERS
                for side in sides:
                    for finger in FINGERS:
                        self.engine._finger(side, finger, False)
                self.engine._send()
                self.engine._await_arrival({s+"_hand": self.engine.targets[s+"_hand"] for s in sides})
            if self.software:
                for index in range(count):
                    self.progress = dict(stage=index+1, total=5, mode="software", side=sides[0] if len(sides)==1 else "both")
                    self.engine.log(event, **self.progress)
                    self.move_targets({s + "_arm": paths[s][index].tolist() for s in paths}, minimum=.4, arm_arrival=True)
                return
            self.path_active = True
            baseline = {s: self.backend.statuses[s + "_arm"]["position_seq"] for s in paths}
            for stage, verb in enumerate(verbs, 1):
                self.progress = dict(stage=stage, total=5, mode="native_j", side=sides[0] if len(sides)==1 else "both")
                self.engine.log(event, **self.progress)
                for side in paths:
                    self.backend.clients[(side, verb)] = self.backend.node.create_client(self.backend.Trigger, f"/{side}_arm/nero_robot_driver/{verb}")
                    self.backend._call(side, verb)
                deadline = time.monotonic() + 40
                arrived_since = None
                while time.monotonic() < deadline:
                    self.engine._check()
                    done = True
                    for side in paths:
                        status = self.backend.statuses[side + "_arm"]
                        if status.get("position_status") in ("failed", "timed_out", "cancelled"):
                            raise RuntimeError(side + " " + label + "驱动失败")
                        if status.get("position_seq", -1) >= baseline[side] + stage:
                            target = status.get("position_target", [])
                            if (status.get("position_seq") != baseline[side] + stage or status.get("position_kind") != kind
                                    or status.get("position_stage") != stage or len(target) != 7
                                    or max(abs(a-b) for a, b in zip(target, paths[side][stage-1])) > 1e-6):
                                raise RuntimeError(side + " " + label + "目标或阶段被其他操作改写")
                        wanted = "arrived" if stage == count else "stage_arrived"
                        done &= (status.get("position_seq") == baseline[side] + stage
                                 and status.get("position_kind") == kind and status.get("position_stage") == stage
                                 and status.get("position_stage_count") == count and status.get("position_status") == wanted
                                 and work_waypoint_arrived(self.backend.measured[side+"_arm"], paths[side][stage-1]))
                    if done:
                        arrived_since = arrived_since or time.monotonic()
                        if time.monotonic() - arrived_since >= .2:
                            break
                    else:
                        arrived_since = None
                    time.sleep(.01)
                else:
                    errors = []
                    for side in paths:
                        actual, target = self.backend.measured[side+'_arm'], paths[side][stage-1]
                        joint = max(range(7), key=lambda i: abs(actual[i]-target[i]))
                        errors.append(f"{side} 第{stage}段 J{joint+1} 目标={math.degrees(target[joint]):.3f}°，"
                                      f"实测={math.degrees(actual[joint]):.3f}°，误差={math.degrees(abs(actual[joint]-target[joint])):.3f}°")
                    errors = "; ".join(errors)
                    raise RuntimeError(f"{label}途经点实测到位超时（容差3°；{errors}）")
            # Enable JS explicitly again only after both drivers finish their J path.
            for side in paths:
                self.backend._call(side, "enable")
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                self.engine._check()
                if all(self.backend.statuses[s+"_arm"].get("streaming_enabled") for s in paths):
                    break
                time.sleep(.01)
            else:
                raise RuntimeError(label + "后流式使能未确认")
            self.path_active = False
            self.engine.targets = self.engine._command_snapshot()
        except BaseException as exc:
            # A streaming hold cannot cancel a native J trajectory. Stop both arms.
            diagnostics = {}
            for side in paths:
                q = self.backend.measured.get(side + "_arm", [])
                stage = min(count, max(1, (self.progress or {}).get("stage", 1)))
                target = paths[side][stage - 1].tolist()
                diagnostics[side] = {
                    "measured_rad": list(q), "target_rad": target,
                    "error_deg": [math.degrees(a-b) for a, b in zip(q, target)],
                    "driver": self.backend.statuses.get(side + "_arm", {}),
                }
            self.engine.log("home_fault" if home else "work_pose_fault", error=str(exc), progress=self.progress, diagnostics=diagnostics)
            self.engine._fault(exc)
            raise
        finally:
            self.path_active = False

    def execute(self, operation, message):
        parameters = validate_command(operation, message, self.config)
        e = self.engine
        path = None
        if operation == "cartesian":
            positions = self.backend.snapshot()
            path = self.cartesian_path(positions, **parameters)
        elif operation in ("joint", "hand_pose"):
            positions = self.backend.snapshot()
            part = parameters.get("part", "hand")
            key = parameters["side"] + "_" + part
            delta = (abs(math.radians(parameters["target_deg"]) - positions[key][parameters["joint_index"]])
                     if operation == "joint" else max(abs(math.radians(a)-b) for a, b in zip(parameters["target_deg"], positions[key])))
            if delta > math.radians(15 if part == "arm" else 30) + 1e-9:
                raise ValueError("单次关节调节最多机械臂15° / 手指30°，请分步移动")
        try:
            # Uncertified config may be used only for an explicitly requested calibration.
            certified = self.config["calibrated"]
            self.config["calibrated"] = True
            try:
                if operation == "work_pose":
                    sides = (parameters["side"],) if parameters["side"] != "both" else ("left", "right")
                    self.native_work_path(sides)
                    self.progress = dict(stage=5, total=5, mode="streaming", side=parameters["side"])
                    e.log("work_pose_stage", **self.progress)
                    self.move_targets({s+"_arm": [math.radians(v) for v in work_poses(self.config)[s]] for s in sides}, minimum=3, arm_arrival=True)
                else:
                    key = parameters["side"] + "_" + parameters.get("part", "hand" if operation == "hand_pose" else "arm")
                    e.log("manual_request", operation=operation, **parameters)
                    self.claim_at_measured(keys=[key])
                    if operation in ("joint", "hand_pose"):
                        if operation == "joint":
                            target = list(e.targets[key])
                            target[parameters["joint_index"]] = math.radians(parameters["target_deg"])
                        else:
                            target = [math.radians(v) for v in parameters["target_deg"]]
                        self.move_targets({key: target}, arm_arrival=operation == "joint" and parameters["part"] == "arm")
                    else:
                        side = parameters["side"]
                        # Reject if feedback moved during IK computation; never snap to an old seed.
                        if max(abs(a-b) for a, b in zip(e.targets[side+"_arm"], positions[side+"_arm"])) > .005:
                            raise ValueError("逆解期间机械臂发生移动，请重新点动")
                        minimum = abs(parameters["step"]) / (5 if len(parameters["axis"]) == 1 else 3) / len(path)
                        for goal in path:
                            self.move_targets({side+"_arm": goal}, minimum=minimum)
            finally:
                self.config["calibrated"] = certified
            e.state = "CALIBRATION_IDLE"
            e.shifts = dict.fromkeys(("left", "right"), 0)
            e.log("manual_calibration", operation=operation, **parameters)
            return {"positions": self.backend.snapshot()}
        except Exception as exc:
            from morla_piano.engine import Stopped
            if isinstance(exc, Stopped) and e.state != "FAULT":
                self.hold_measured()
            elif e.state != "FAULT":
                e._fault(exc)
            raise
        finally:
            self.progress = None
