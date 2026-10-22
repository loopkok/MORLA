"""Shared fail-closed gate for non-motion hardware maintenance."""

import time


def maintenance_interface_ready(backend):
    now = time.monotonic()
    return all(now - backend.status_at.get(side + "_arm", -float("inf")) <= 1.5
               and backend.statuses.get(side + "_arm", {}).get("connected") is True
               and backend.statuses[side + "_arm"].get("backend") == "hardware"
               and type(backend.statuses[side + "_arm"].get("reset_seq")) is int
               and type(backend.statuses[side + "_arm"].get("can_init_seq")) is int
               for side in ("left", "right"))


def wait_maintenance(backend, operation, baselines, check_cancelled, timeout=18.0):
    """RPC acceptance is not SDK completion. Require a newer status sequence."""
    prefix = "reset" if operation == "reset" else "can_init"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_cancelled()
        backend.pump()
        complete = True
        for side, baseline in baselines.items():
            key = side + "_arm"
            state = backend.statuses.get(key, {})
            fresh = time.monotonic() - backend.status_at.get(key, -float("inf")) <= 1.5
            newer = state.get(prefix + "_seq", -1) == baseline + 1
            outcome = state.get(prefix + "_status")
            if newer and outcome in {"failed", "timed_out", "cancelled"}:
                raise RuntimeError(f"{side} {operation}: {state.get(prefix + '_error') or outcome}")
            if state.get(prefix + "_seq", -1) > baseline + 1:
                raise RuntimeError(f"{side} {operation}: 维护序号被其他请求改写")
            good = fresh and newer and outcome == ("cleared" if operation == "reset" else "ready")
            good &= state.get("controller_status_fresh") is True and state.get("controller_arm_status") in {0, 6}
            if operation != "reset":
                good &= state.get("feedback_fresh") is True and state.get("motor_status_fresh") is True and state.get("motors_enabled") is True
            complete &= good
        if complete:
            return
        time.sleep(.01)
    details = {s: {k: v for k, v in backend.statuses.get(s + "_arm", {}).items()
                   if k.startswith(prefix) or k in {"controller_arm_status", "controller_status_fresh", "feedback_fresh"}}
               for s in baselines}
    raise RuntimeError(f"{operation} 未取得双臂实测完成确认：{details}")


def check_estop_before_motion(mode, statuses):
    if mode == "hardware":
        stopped = [side for side in ("left", "right")
                   if statuses.get(side + "_arm", {}).get("controller_arm_status") == 1]
        if stopped:
            raise ValueError(" / ".join(stopped) + " 控制器急停尚未解除；先点击复位急停并重新检查反馈，未取得运动控制权")


def maintenance_allowed(mode, state, motion_claimed):
    return mode == "hardware" and state in {"CONNECTED", "WAITING_FEEDBACK"} and motion_claimed is False


def validate_maintenance(mode, state, motion_claimed, confirmed):
    if confirmed is not True:
        raise ValueError("维护动作须明确确认")
    if mode != "hardware":
        raise ValueError("控制器维护仅用于真机模式")
    if not maintenance_allowed(mode, state, motion_claimed):
        raise ValueError("急停/故障或已取得运动控制权后，请先断开并重建会话，再复位；不会自动恢复运动")
