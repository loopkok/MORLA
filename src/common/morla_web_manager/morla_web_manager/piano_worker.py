"""One isolated piano session. JSON-lines IPC; only this process owns ROS motion."""
from __future__ import annotations

import argparse
from collections import deque
import contextlib
import json
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time

from .piano_calibration import CalibrationMotion, MANUAL_OPERATIONS
from .piano_maintenance import maintenance_allowed, validate_maintenance, check_estop_before_motion, wait_maintenance, maintenance_interface_ready
from .piano_processes import stop_owned


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--mode", choices=("software", "ros_fake", "hardware"), required=True)
    parser.add_argument("--log", required=True)
    parser.add_argument("--driver-pgid", type=int)
    parser.add_argument("--driver-scope")
    parser.add_argument("--failsafe", action="store_true")
    args = parser.parse_args()
    sys.path.insert(0, str(Path(args.project).resolve()))
    from morla_piano.engine import Clock, Engine, FakeBackend, Stopped, MotionConfirmationError, smooth
    from morla_piano.model import FINGER_JOINTS, FINGERS, SIDES, arm_target, load_config, plan_score, hand_command_seed, performance_timing

    config = load_config(args.config)
    immutable_calibrated = config["calibrated"]
    clock = Clock()
    if args.mode == "software":
        backend = FakeBackend(config, clock)
        backend.history = deque(maxlen=2)
    else:
        from morla_piano.ros_backend import RosBackend
        backend = RosBackend(config, "hardware" if args.mode == "hardware" else "fake")
    if args.failsafe:
        # Stop-only recovery after an execution process crash: never claim, enable or move.
        backend.emergency_stop()
        backend.close()
        return
    engine = Engine(config, backend, clock, args.log)
    calibration = CalibrationMotion(engine, backend, config, software=args.mode == "software")
    engine.events = deque(maxlen=1000)
    commands = queue.Queue(maxsize=8)
    parent_seen = [time.monotonic()]
    parent_alive = threading.Event()
    parent_alive.set()
    closing = threading.Event()
    emergency = threading.Event()
    out_lock = threading.Lock()
    active_job = [None]
    connected = [False]
    reported_at = [0.0]
    cancel_epoch = [0]
    job_epoch = [0]
    finger_pending = {}
    maintenance_active = [False]
    connection_checked_at = [0.0]

    def emit(kind, **fields):
        with out_lock:
            try:
                sys.__stdout__.write(json.dumps(dict(kind=kind, **fields), ensure_ascii=False, allow_nan=False) + "\n")
                sys.__stdout__.flush()
            except (BrokenPipeError, OSError):
                parent_alive.clear()
                engine.request_stop()

    def status():
        state = engine.status()
        state["events"] = list(engine.events)[-20:]
        state["job_id"] = active_job[0]
        state["connected"] = connected[0]
        state["motion_claimed"] = backend.claimed
        state["replay_direct_allowed"] = (engine.state == "FINISHED" or
            (engine.state == "STOPPED" and engine.playback is not None
             and not getattr(engine, "_manual_calibration", False)))
        state["maintenance_allowed"] = (maintenance_allowed(args.mode, engine.state, backend.claimed)
                                         and maintenance_interface_ready(backend))
        state["reset_required"] = args.mode == "hardware" and any(
            getattr(backend, "statuses", {}).get(hand + "_arm", {}).get("controller_arm_status") == 1
            for hand in SIDES)
        state["arm_speed"] = {side: {key: getattr(backend, "statuses", {}).get(side + "_arm", {}).get(key)
            for key in ("position_speed_percent", "home_speed_percent", "stream_j_speed_percent", "stream_applied_speed_percent")}
            for side in SIDES}
        state["hand_sdk"] = getattr(backend, "sdk_statuses", {})
        state["feedback_warnings"] = getattr(backend, "feedback_warnings", {})
        state["hand_feedback_start_allowed"] = (
            maintenance_allowed(args.mode, engine.state, backend.claimed)
            and all(time.monotonic() - getattr(backend, "sdk_status_at", {}).get(side, 0) < 1.5
                    and type(backend.sdk_statuses.get(side, {}).get("feedback_generation")) is int for side in SIDES))
        state.update(calibration.telemetry())
        return state

    def reader():
        for line in sys.stdin:
            try:
                message = json.loads(line)
                operation = message.get("operation")
                if operation == "heartbeat":
                    parent_seen[0] = time.monotonic()
                    continue
                if operation in ("stop", "shutdown", "estop"):
                    cancel_epoch[0] += 1
                    engine.request_stop()
                    if operation == "estop" or (operation in ("stop", "shutdown") and calibration.path_active):
                        emergency.set()
                    if operation == "shutdown":
                        closing.set()
                message["cancel_epoch"] = cancel_epoch[0]
                commands.put_nowait(message)
            except (ValueError, queue.Full) as exc:
                emit("error", error=str(exc))
        parent_alive.clear()
        engine.request_stop()

    original_pump = backend.pump

    def guarded_pump():
        if not parent_alive.is_set() or time.monotonic() - parent_seen[0] > 2:
            raise RuntimeError("Web 服务心跳中断，停止钢琴控制")
        if emergency.is_set():
            raise RuntimeError("操作员请求急停")
        if maintenance_active[0] and (closing.is_set() or job_epoch[0] != cancel_epoch[0]):
            raise RuntimeError("维护操作被停止；须核验控制器状态")
        result = original_pump()
        for (hand, digit), request in list(finger_pending.items()):
            actual = backend.measured.get(hand + "_hand", [])
            if actual and all(abs(actual[i] - q) <= config["timing"]["arrival_tolerance_rad"]
                              for i, q in request["target"].items()):
                request["arrived"] = True
            if not request["arrived"] and time.monotonic() > request["deadline"]:
                raise MotionConfirmationError(f"{hand} {digit} 按下目标未获得实测确认")
        if time.monotonic() - reported_at[0] > .2:
            emit("snapshot", **status())
            reported_at[0] = time.monotonic()
        return result

    backend.pump = guarded_pump
    original_health = backend.health

    if args.mode != "software":
        original_graph_check = backend._graph_check

        def guarded_graph_check():
            original_graph_check()
            for topic in ("/morla/control/state", "/data_collect/state"):
                if backend.node.count_publishers(topic):
                    raise RuntimeError(f"外部机器人/采集节点进入钢琴会话：{topic}")
            for hand in SIDES:
                for suffix in ("xhand_command", "xhand_state"):
                    topic = f"/{hand}_hand/driver/{suffix}"
                    if backend.node.count_publishers(topic) != 1:
                        raise RuntimeError(f"原生双手接口存在缺失或重复发布者：{topic}")

        backend._graph_check = guarded_graph_check

    if args.mode != "software":
        def guarded_health(require_enabled=True):
            if calibration.path_active:
                return calibration.path_health()
            original_health(require_enabled=require_enabled)
            if args.mode == "hardware" and require_enabled:
                for hand in SIDES:
                    scope = getattr(backend, "motion_keys", None)
                    if scope is not None and hand + "_arm" not in scope:
                        continue
                    driver = backend.statuses.get(hand + "_arm", {})
                    if driver.get("controller_status_fresh") is not True or driver.get("controller_arm_status") != 0:
                        raise RuntimeError(f"{hand} 控制器状态异常或过期，停止输出")
        backend.health = guarded_health
    original_check = engine._check

    def cancellable_check():
        if emergency.is_set():
            raise RuntimeError("操作员请求急停")
        if job_epoch[0] != cancel_epoch[0]:
            raise Stopped("操作已被停止请求取消")
        original_check()

    engine._check = cancellable_check
    original_finger = engine._finger

    def checked_finger(hand, digit, press):
        previous = finger_pending.get((hand, digit))
        if not press and previous and not previous["arrived"] and not engine.stop_event.is_set():
            raise MotionConfirmationError(f"{hand} {digit} 尚未实测按下，不能报告按键成功")
        original_finger(hand, digit, press)
        if press and config["hands"][hand].get("arrival_check_enabled", True) is False:
            finger_pending.pop((hand, digit), None)
            engine.log("finger_press_unverified", side=hand, finger=digit, measured_confirmed=False)
        elif press:
            finger_pending[(hand, digit)] = {
                "target": {i: engine.targets[hand + "_hand"][i] for i in FINGER_JOINTS[digit]},
                "arrived": False, "deadline": time.monotonic() + config["timing"]["arrival_timeout_s"],
            }
        else:
            finger_pending.pop((hand, digit), None)

    engine._finger = checked_finger
    original_fault = engine._fault

    def checked_fault(error):
        finger_pending.clear()
        emergency.clear()
        original_fault(error)

    engine._fault = checked_fault
    threading.Thread(target=reader, daemon=True).start()

    def connect():
        connected[0] = False
        try:
            if args.mode == "software":
                backend.connect()
            else:
                original_pump()
                backend.check_connection()
            connected[0] = True
            engine.state, engine.error = "CONNECTED", None
        except Exception as exc:
            engine.state, engine.error = "WAITING_FEEDBACK", str(exc)
        emit("snapshot", **status())

    def require_motion(message):
        if not connected[0]:
            raise ValueError("先恢复四路反馈并重新连接")
        if engine.state == "FAULT":
            raise ValueError("故障会话须断开并重新建立")
        check_estop_before_motion(args.mode, getattr(backend, "statuses", {}))
        if args.mode == "hardware" and not immutable_calibrated and message.get("calibration_confirmed") is not True:
            raise ValueError("未认证配置只能在明确允许标定试动后执行")

    def healthy_stop():
        if backend.claimed and engine.targets is not None and engine.state != "FAULT":
            try:
                if getattr(engine, "_manual_calibration", False):
                    calibration.hold_measured()
                else:
                    engine._normal_stop()
            except Exception as exc:
                engine._fault(exc)

    def execute(message):
        operation = message["operation"]
        if operation == "start_hand_feedback":
            validate_maintenance(args.mode, engine.state, backend.claimed, message.get("calibration_confirmed"))
            if not status()["hand_feedback_start_allowed"]:
                raise ValueError("双手 SDK 维护接口未就绪；未发送无力初始化")
            baselines = {side: backend.sdk_statuses[side]["feedback_generation"] for side in SIDES}
            for side in SIDES:
                if emergency.is_set() or closing.is_set() or job_epoch[0] != cancel_epoch[0]:
                    raise ValueError("双手反馈初始化已取消")
                verb = "start_hand_feedback"
                if (side, verb) not in backend.clients:
                    backend.clients[(side, verb)] = backend.node.create_client(backend.Trigger, f"/{side}_hand/driver/start_feedback")
                backend._call(side, verb)
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                backend.pump()
                if emergency.is_set() or closing.is_set() or job_epoch[0] != cancel_epoch[0]:
                    raise ValueError("双手反馈初始化已取消")
                if all(backend.sdk_statuses.get(side, {}).get("feedback_generation") == baselines[side]+1
                       and backend.sdk_statuses[side].get("feedback_valid") is True
                       and time.monotonic() - backend.received.get(side+"_hand", 0) < .5 for side in SIDES):
                    connect()
                    return {"message": "双手无力模式反馈已确认；机械臂维护与运动仍须单独操作"}
                time.sleep(.01)
            reasons = [value for key, value in getattr(backend, "input_errors", {}).items()
                       if key.startswith(("left_hand/", "right_hand/"))]
            if not reasons:
                reasons = [f"{side}_hand: {backend.sdk_statuses.get(side, {}).get('error') or '等待新鲜手部读数'}"
                           for side in SIDES if time.monotonic() - backend.received.get(side+"_hand", 0) >= .5]
            raise ValueError("双手无力反馈未获得有效实测确认；" + "; ".join(reasons)
                             + "；未请求手指位置运动")
        if operation in ("initialize_can", "reset"):
            validate_maintenance(args.mode, engine.state, backend.claimed, message.get("calibration_confirmed"))
            if not maintenance_interface_ready(backend):
                raise ValueError("双臂维护接口尚未就绪；不要求手部反馈，请等待双臂状态消息")
            maintenance_active[0] = True
            def check_maintenance_cancelled():
                if emergency.is_set() or closing.is_set() or job_epoch[0] != cancel_epoch[0]:
                    raise RuntimeError("维护操作被停止；须核验控制器状态")
            try:
                backend.pump()
                prefix = "reset" if operation == "reset" else "can_init"
                baselines = {}
                for side in SIDES:
                    state = backend.statuses.get(side + "_arm", {})
                    if (time.monotonic() - backend.status_at.get(side + "_arm", -float("inf")) > 1.5
                            or not isinstance(state.get(prefix + "_seq"), int)):
                        raise RuntimeError(side + " 维护状态过期或驱动版本不匹配，未发送维护请求")
                    baselines[side] = state[prefix + "_seq"]
                for side in SIDES:
                    check_maintenance_cancelled()
                    if (side, operation) not in backend.clients:
                        backend.clients[(side, operation)] = backend.node.create_client(
                            backend.Trigger, f"/{side}_arm/nero_robot_driver/{operation}")
                    backend._call(side, operation)
                wait_maintenance(backend, operation, baselines, check_maintenance_cancelled)
            except Exception:
                backend.emergency_stop()
                engine.state = "FAULT"
                raise
            finally:
                maintenance_active[0] = False
            connected[0] = False
            engine.state = "WAITING_FEEDBACK"
            return {"message": "双臂驱动反馈已确认维护完成；点击重新检查反馈，未自动恢复运动"}
        if operation == "refresh":
            if backend.claimed or engine.state == "FAULT":
                raise ValueError("取得控制权或故障后须先断开会话")
            connect()
            return {}
        if operation == "capture":
            if not connected[0] or active_job[0] is None:
                raise ValueError("反馈未就绪")
            positions = backend.snapshot()
            return {"positions": positions, "hand_command_seeds": {
                side: hand_command_seed(positions[side+"_hand"], config["hands"][side]["hand_limits_rad"],
                                        positions[side+"_hand"]) for side in SIDES}}
        if operation in MANUAL_OPERATIONS:
            require_motion(message)
            if engine.state not in ("CONNECTED", "CALIBRATION_IDLE", "READY", "STOPPED", "FINISHED", "WORK_READY") or engine.pressed:
                raise ValueError("当前状态不能进行手动标定")
            engine.stop_event.clear()
            return calibration.execute(operation, message)
        if operation == "home":
            require_motion(message)
            if engine.state not in ("CONNECTED", "CALIBRATION_IDLE", "READY", "STOPPED", "FINISHED", "WORK_READY") or engine.pressed:
                raise ValueError("请先停止当前动作，再按钢琴回位轨迹回起始位")
            engine.stop_event.clear()
            config["calibrated"] = immutable_calibrated or message.get("calibration_confirmed") is True
            try:
                calibration.return_piano_home()
                engine.state, engine.error = "STOPPED", None
                engine.pressed = None
                engine._manual_calibration = True
                return {"message": "双臂已按钢琴回位轨迹回起始位；演奏前须重新准备"}
            finally:
                config["calibrated"] = immutable_calibrated
                calibration.progress = None
        if operation == "prepare":
            require_motion(message)
            engine._manual_calibration = False
            # An explicit calibration trial may enable only the bounded, pinned config.
            config["calibrated"] = immutable_calibrated or message.get("calibration_confirmed") is True
            try:
                engine.prepare(approach=lambda: calibration.execute(
                    "work_pose", {"side": "both", "calibration_confirmed": True}))
            finally:
                config["calibrated"] = immutable_calibrated
            return {}
        if operation == "play":
            require_motion(message)
            if args.mode == "hardware" and not immutable_calibrated:
                raise ValueError("先完成标定并认证保存配置，再重新建立演奏会话")
            plan = plan_score(message["score_path"], config)
            if message.get("prepare_first") is True:
                if engine.state not in ("FINISHED", "STOPPED"):
                    raise ValueError("当前状态不能重新准备并演奏")
                if not status()["replay_direct_allowed"]:
                    raise ValueError("当前不是可续用的演奏位置，请先执行移动到初始位")
                engine._manual_calibration = False
                engine.log("replay_return_to_zero", route="direct_piano_zero")
                engine.prepare()
                engine._check()
            engine.play(plan)
            return {}
        if operation == "finger":
            require_motion(message)
            if engine.state != "READY" or engine.pressed:
                raise ValueError("单指测试前先准备到位")
            side, finger = message["side"], message["finger"]
            if side not in SIDES or finger not in FINGERS:
                raise ValueError("无效手指")
            engine.stop_event.clear()
            engine.state = "CALIBRATING"
            try:
                engine._finger(side, finger, True)
                engine.pressed = (side, finger)
                engine._send()
                engine._await_arrival({side + "_hand": engine.targets[side + "_hand"]})
                engine._wait(.3)
                engine._finger(side, finger, False)
                engine.pressed = None
                engine._send()
                engine._await_arrival({side + "_hand": engine.targets[side + "_hand"]})
                engine.state = "READY"
            except Stopped:
                healthy_stop()
            except Exception as exc:
                engine._fault(exc)
                raise
            return {}
        if operation == "shift":
            require_motion(message)
            if engine.state != "READY" or engine.pressed:
                raise ValueError("移位测试前先准备到位")
            side, shift = message["side"], message["shift_white"]
            if side not in SIDES or isinstance(shift, bool) or not isinstance(shift, int):
                raise ValueError("无效移位参数")
            q0, q1 = list(engine.targets[side + "_arm"]), arm_target(config, side, shift)
            timing = performance_timing(config)
            duration = max(timing["move_duration_s"], 1.5 * max(abs(a - b) for a, b in zip(q0, q1)) / timing["max_arm_velocity_rad_s"])
            engine.stop_event.clear()
            engine.state = "CALIBRATING"
            try:
                engine.performance_speed(True)
                begin = time.monotonic()
                while time.monotonic() - begin < duration:
                    engine._check()
                    ratio = smooth((time.monotonic() - begin) / duration)
                    engine.targets[side + "_arm"] = [a + ratio * (b - a) for a, b in zip(q0, q1)]
                    engine._send()
                    time.sleep(1 / timing["control_hz"])
                engine.targets[side + "_arm"] = q1
                engine._await_arrival({side + "_arm": q1})
                engine.shifts[side] = shift
                engine.state = "READY"
            except Stopped:
                healthy_stop()
            except Exception as exc:
                engine._fault(exc)
                raise
            finally:
                if backend.claimed and engine.state != "FAULT":
                    try:
                        engine.performance_speed(False)
                    except Exception as exc:
                        engine._fault(exc)
                        raise
            return {}
        if operation in ("stop", "shutdown"):
            healthy_stop()
            return {}
        if operation == "estop":
            backend.emergency_stop()
            engine.state, engine.error = "FAULT", "操作员请求急停"
            return {}
        raise ValueError("未知钢琴操作")

    # Keep non-protocol output on stderr, including vendor Python logging.
    with contextlib.redirect_stdout(sys.stderr):
        try:
            connect()
            last_report = 0.0
            while True:
                try:
                    message = commands.get(timeout=.01)
                except queue.Empty:
                    message = None
                if message is not None:
                    active_job[0] = message.get("job_id")
                    job_epoch[0] = message["cancel_epoch"]
                    emit("snapshot", **status())
                    try:
                        if job_epoch[0] != cancel_epoch[0] and message["operation"] not in ("stop", "shutdown", "estop"):
                            raise Stopped("排队的操作已取消")
                        result = execute(message)
                        emit("result", job_id=active_job[0], ok=engine.state != "FAULT", result=result, **engine.status())
                    except Exception as exc:
                        emit("result", job_id=active_job[0], ok=False, **{**engine.status(), "error": str(exc)})
                    active_job[0] = None
                    emit("snapshot", **status())
                    if message["operation"] == "shutdown":
                        break
                if not parent_alive.is_set() or time.monotonic() - parent_seen[0] > 2:
                    if backend.claimed:
                        engine._fault(RuntimeError("Web 服务失联"))
                    break
                if not connected[0] and args.mode != "software":
                    # Waiting for cold-start/reset still needs subscriptions;
                    # this pump creates no publisher and grants no ownership.
                    original_pump()
                if (args.mode != "software" and not backend.claimed and engine.state in {"WAITING_FEEDBACK", "CONNECTED"}
                        and time.monotonic() - connection_checked_at[0] >= .2):
                    connection_checked_at[0] = time.monotonic()
                    # Discovery is incremental. A missing hand must not keep
                    # reset/CAN requests trapped behind a connection timeout.
                    try:
                        backend.check_connection()
                        connected[0] = True
                        engine.state, engine.error = "CONNECTED", None
                    except Exception as exc:
                        connected[0] = False
                        engine.state, engine.error = "WAITING_FEEDBACK", str(exc)
                if connected[0]:
                    if engine.state == "CALIBRATION_IDLE" and engine.targets is not None:
                        try:
                            engine._check()
                            engine._send()
                        except Stopped:
                            healthy_stop()
                        except Exception as exc:
                            engine._fault(exc)
                    else:
                        engine.idle()
                if time.monotonic() - last_report > .2:
                    emit("snapshot", **status())
                    last_report = time.monotonic()
        finally:
            if backend.claimed:
                healthy_stop()
            backend.close()
            # Also reap our launch group if the Web parent was killed before its supervisor could run.
            if args.driver_scope and Path("/proc/self/stat").exists():
                remaining = stop_owned(args.driver_scope, args.driver_pgid)
                if remaining:
                    print(f"Owned piano drivers did not exit: {remaining}", file=sys.stderr)
            elif args.driver_pgid and args.driver_pgid != os.getpgrp():
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                    try:
                        if sig == signal.SIGINT:
                            os.kill(args.driver_pgid, sig)
                        else:
                            os.killpg(args.driver_pgid, sig)
                    except ProcessLookupError:
                        break
                    time.sleep(.3)
            emit("closed")


if __name__ == "__main__":
    main()
