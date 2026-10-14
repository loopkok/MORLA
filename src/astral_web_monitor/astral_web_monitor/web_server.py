"""FastAPI control plane + WebSocket telemetry + SPA static hosting.

Architecture follows rob_station's control_server:
  * REST  /api/v1/*  for control (start/stop/pause/resume/presets/health)
  * WS    /ws/telemetry  pushes a 30 Hz `ui_state` frame built from the ROS
    snapshot + launch state; broadcast is skipped when there are no clients
  * Static files served from web/dist (production) with SPA history fallback

The ROS node runs in a background thread (monitor_node.init_node); this
module runs uvicorn on the main thread.
"""
from __future__ import annotations

import asyncio
import json
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from . import config
from .data_process_manager import DataProcessManager, PipelineConflict
from .config import STOP_SIGINT_TIMEOUT_S, log_tail_for_push
from .launch_manager import (
    LaunchManager,
    PAUSED,
    RUNNING,
    STOPPED,
    Preset,
    load_presets,
    preset_with_teleop_log,
)
from .monitor_node import get_node, init_node, shutdown_node
from .schemas import (
    ApiEnvelope,
    DataProcessRequest,
    CollectControlRequest,
    CollectTaskRequest,
    CollectSessionRequest,
    InferCmdRequest,
    InferTaskRequest,
    InferLaunchRequest,
    PresetInfo,
    StartRequest,
    VideoCamerasRequest,
    VideoPushRequest,
)


# --- globals ---------------------------------------------------------------
_launch_mgr = LaunchManager()
# 数采独立泳道：与遥操预设生命周期完全解耦（用户决策：web 单独启停）。
# 纯订阅者、不抢 joint_commands，start 时跳过孤儿检测；遥操侧孤儿检测
# 对 astral_data_collect 命令行有对称豁免（launch_manager._find_orphan）。
_collect_mgr = LaunchManager(lane_name="数采")
# 推理独立泳道：配置化构建 policy_inference.launch.py 命令。推理节点设计上就与
# 遥操共存（takeover 仲裁），启动也跳过孤儿检测——否则遥操栈运行时会挡启动。
_policy_mgr = LaunchManager(lane_name="推理")
# 最近一次推理启动配置（restart 复用；前端表单驱动）
_policy_cfg: dict = {}
_data_mgr = DataProcessManager(config.DATA_ROOT, config.DATA_PIPELINE_PYTHON,
                               config.DATA_PIPELINE_PACKAGE, config.DATA_CHECK_PYTHON)
_presets = load_presets()
_web_dist = Path(os.environ.get("ASTRAL_WEB_MONITOR_DIST") or
                 str(config.SDK_ROOT / 'astral_ws/src/astral_web_monitor/web/dist'))
if not _web_dist.is_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        _web_dist = Path(get_package_share_directory('astral_web_monitor')) / 'web/dist'
    except Exception:
        pass


def _build_ui_state() -> dict[str, Any]:
    """Aggregate one telemetry frame from ROS snapshot + launch state."""
    node = get_node()
    ros = node.snapshot() if node else {"joints": {}, "rates_hz": {}}
    return {
        "type": "ui_state",
        "ts": time.time(),
        "teleop": {
            "state": _launch_mgr.state,
            "preset": _launch_mgr.preset,
            "uptime_s": _launch_mgr.uptime_s(),
            "pid": _launch_mgr.pid,
        },
        "joints": ros.get("joints", {}),
        "rates_hz": ros.get("rates_hz", {}),
        "state_rates_hz": ros.get("state_rates_hz", {}),
        "health": ros.get("health", {"overall": "ok", "entities": {}}),
        "latency": ros.get("latency", {"stages": {}}),
        "video_gate": ros.get("video_gate"),
        "data_collect": ros.get("data_collect"),
        "data_process": _data_mgr.snapshot(),
        "infer": ros.get("infer"),
        "collect_launch": {
            "state": _collect_mgr.state,
            "preset": _collect_mgr.preset,
            "uptime_s": _collect_mgr.uptime_s(),
            "pid": _collect_mgr.pid,
            "log_tail": log_tail_for_push(_collect_mgr.log_tail()),
        },
        "infer_launch": {
            "state": _policy_mgr.state,
            "preset": _policy_mgr.preset,
            "uptime_s": _policy_mgr.uptime_s(),
            "pid": _policy_mgr.pid,
            "log_tail": log_tail_for_push(_policy_mgr.log_tail()),
        },
        "log_tail": log_tail_for_push(_launch_mgr.log_tail()),
    }


# --- connection manager ---------------------------------------------------
class _ConnectionManager:
    def __init__(self) -> None:
        self.active: list[WebSocket] = []

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self.active:
            self.active.remove(ws)

    @property
    def count(self) -> int:
        return len(self.active)

    async def broadcast(self, message: dict) -> None:
        text = json.dumps(message, ensure_ascii=False)
        dead: list[WebSocket] = []
        for ws in self.active:
            try:
                await ws.send_text(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


_conn = _ConnectionManager()


# --- lifespan --------------------------------------------------------------
@asynccontextmanager
async def _lifespan(app: FastAPI):
    init_node()
    broadcaster = asyncio.create_task(_ui_state_broadcaster())
    try:
        yield
    finally:
        broadcaster.cancel()
        try:
            await broadcaster
        except asyncio.CancelledError:
            pass
        shutdown_node()
        await asyncio.to_thread(_data_mgr.close)


async def _ui_state_broadcaster() -> None:
    interval = 1.0 / config.WS_PUSH_HZ
    while True:
        await asyncio.sleep(interval)
        if _conn.count <= 0:
            continue
        msg = await asyncio.to_thread(_build_ui_state)
        await _conn.broadcast(msg)


# --- app -------------------------------------------------------------------
app = FastAPI(title="astral_web_monitor", version="0.1.0", lifespan=_lifespan)


@app.get("/api/v1/health")
async def health() -> ApiEnvelope:
    node = get_node()
    ros = node.snapshot() if node else {"health": {"overall": "down", "entities": {}}}
    return ApiEnvelope(
        ok=True,
        data={
            "ros_ok": node is not None,
            "launch_state": _launch_mgr.state,
            "launch_pid": _launch_mgr.pid,
            "uptime_s": _launch_mgr.uptime_s(),
            "ws_clients": _conn.count,
            "health": ros.get("health", {"overall": "down", "entities": {}}),
        },
    )


@app.get("/api/v1/presets", response_model=ApiEnvelope)
async def list_presets() -> ApiEnvelope:
    items = [
        PresetInfo(
            name=p.name, package=p.package, launch=p.launch,
            args=p.args, description=p.description,
        )
        for p in _presets.values()
    ]
    return ApiEnvelope(ok=True, data=[m.model_dump() for m in items])


@app.get("/api/v1/state")
async def get_state() -> ApiEnvelope:
    return ApiEnvelope(ok=True, data=_build_ui_state())


@app.get('/api/v1/data/sessions')
async def data_sessions() -> ApiEnvelope:
    sessions = await asyncio.to_thread(_data_mgr.sessions)
    return ApiEnvelope(ok=True, data={**_data_mgr.capabilities(), 'sessions': sessions})


@app.get('/api/v1/data/jobs')
async def data_jobs() -> ApiEnvelope:
    return ApiEnvelope(ok=True, data=_data_mgr.snapshot())


@app.post('/api/v1/data/jobs')
async def data_job_start(req: DataProcessRequest) -> ApiEnvelope:
    try:
        source = _data_mgr.resolve_source(req.source)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    node = get_node()
    if node:
        collect = node.snapshot().get('data_collect') or {}
        if not collect.get('stale', True) and collect.get('state') in ('RECORDING', 'PAUSED', 'SAVING'):
            folder = collect.get('folder')
            if folder:
                if Path(folder).resolve().is_relative_to(source):
                    raise HTTPException(status_code=409, detail='该 session 正在录制或保存，请先停止录制')
    try:
        job = await asyncio.to_thread(_data_mgr.start, req.model_dump())
    except PipelineConflict as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except (ValueError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return ApiEnvelope(ok=True, message='数据处理任务已启动', data=job)


@app.post('/api/v1/data/jobs/cancel')
async def data_job_cancel() -> ApiEnvelope:
    try:
        _data_mgr.cancel()
    except PipelineConflict as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return ApiEnvelope(ok=True, message='正在停止数据处理；已完成产物保留')


@app.get('/api/v1/data/jobs/{job_id}')
async def data_job_get(job_id: str) -> ApiEnvelope:
    try:
        return ApiEnvelope(ok=True, data=_data_mgr.job(job_id))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail='任务不存在') from exc


@app.get('/api/v1/data/jobs/{job_id}/files/{relative:path}')
async def data_job_file(job_id: str, relative: str):
    try:
        path = _data_mgr.artifact(job_id, relative)
    except (KeyError, FileNotFoundError) as exc:
        raise HTTPException(status_code=404, detail='文件不存在') from exc
    except ValueError as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    return FileResponse(path)


@app.get("/api/v1/logs")
async def get_logs() -> ApiEnvelope:
    """全量环形缓冲（默认 8000 行），供系统页复制/下载；WS 只推尾 800。"""
    return ApiEnvelope(
        ok=True,
        data={
            "teleop": _launch_mgr.log_tail(),
            "collect": _collect_mgr.log_tail(),
            "infer": _policy_mgr.log_tail(),
        },
    )


@app.post("/api/v1/start")
async def start(req: StartRequest) -> ApiEnvelope:
    preset = _presets.get(req.preset)
    if preset is None:
        raise HTTPException(status_code=404, detail=f"未知预设: {req.preset}")
    if req.log:
        # 遥操日志开关：给白名单预设注入 log_dir 根目录（副本，不改 presets.yaml 源）。
        # launch 每次自动建 {log_dir}/{stamp}[_tag]/ 运行子目录、按侧拆 _left/_right。
        preset = preset_with_teleop_log(preset, config.TELEOP_LOG_ROOT)
    ok, msg = _launch_mgr.start(preset)
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/stop")
async def stop() -> ApiEnvelope:
    ok, msg = _launch_mgr.stop()
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/pause")
async def pause() -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    if _launch_mgr.state not in (RUNNING,):
        raise HTTPException(status_code=409, detail=f"当前状态 {_launch_mgr.state} 无法暂停")
    node.publish_disarm()
    _launch_mgr.mark_paused()
    return ApiEnvelope(ok=True, message="已暂停 (disarm)")


@app.post("/api/v1/resume")
async def resume() -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    if _launch_mgr.state != PAUSED:
        raise HTTPException(status_code=409, detail=f"当前状态 {_launch_mgr.state} 非暂停")
    node.publish_arm()
    _launch_mgr.mark_resumed()
    return ApiEnvelope(ok=True, message="已恢复 (arm)")


@app.post("/api/v1/teleop/start")
async def teleop_start() -> ApiEnvelope:
    """One-shot /teleop/start: arm nodes capture vr_init from current pose + arm.

    Use after the teleop nodes are running with require_start_signal:=true and
    the user has placed their hands at the desired initial pose. Re-sending
    re-captures the zero (re-center). The arm node is the authority: it ignores
    the trigger while homing or before any VR pose arrives (logs a warning), so
    this endpoint publishes unconditionally — it works whether the teleop launch
    was started via this monitor or from a separate CLI.
    """
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    # 先发 /teleop/armed=true：工作位/HOME/暂停发过的 latched disarm 会把
    # 夹爪 pinch 仲裁门关死——门只在 armed=true 时重开（arm 节点未校准会
    # 忽略 armed，无害；真正 arm 由随后的 /teleop/start 完成）。
    node.publish_arm()
    node.publish_start()
    return ApiEnvelope(
        ok=True,
        message="已发送 /teleop/armed + /teleop/start（臂节点记 vr_init 并 arm；homing 中或无 VR 时会忽略并告警）",
    )


@app.post("/api/v1/teleop/home")
async def teleop_home() -> ApiEnvelope:
    """HOME（归零 / park-to-zero）：双臂从当前位姿沿
    init_pose → init_waypoints → 零位 慢速收回并停在零位。

    Sequence: ① 真机预设先确保电机上电（~/enable 不回零，幂等；急停失能后
    正好借这次使能）② 发布 /teleop/disarm + /teleop/home——臂节点收到 HOME
    会自己 disarm 并走轨迹（遥操中/未校准都会忽略之外的输入）。之后要再遥操
    需重新 /teleop/start。仅当前 launch 运行中可用。
    """
    if _launch_mgr.state not in (RUNNING, PAUSED):
        raise HTTPException(
            status_code=409,
            detail=f"当前状态 {_launch_mgr.state} 无法 HOME（需遥操运行中）",
        )
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    preset = _current_preset()
    if _preset_has_driver(preset):
        confirmed, emsg = await asyncio.to_thread(
            _ensure_driver_enabled, config.DRIVER_ENABLE_WAIT_S
        )
        if not confirmed:
            raise HTTPException(status_code=503, detail=f"电机使能失败，不能 HOME：{emsg}")
    node.publish_disarm()
    await asyncio.sleep(0.1)
    node.publish_home()
    return ApiEnvelope(
        ok=True,
        message="已下发 HOME：双臂收回 init_pose → init_waypoints → 零位",
    )


@app.post("/api/v1/teleop/workpos")
async def teleop_workpos() -> ApiEnvelope:
    """工作位（go-to-init）：双臂从当前位姿沿 init_waypoints → init_pose
    慢速走到初始工作位并保持——替代原启动自动归位（move_to_init_pose=false
    后启动只拉节点，回工作位靠本按钮）。

    Sequence: 发布 /teleop/disarm + /teleop/init——臂节点收到 init 会自己
    disarm 并走轨迹；到 init_pose 后保持并锚 VR 原点。之后要遥操需重新
    /teleop/start（未先按工作位直接 start 时臂节点会把原点重锚到当前实测，
    纯增量开始）。仅当前 launch 运行中可用。
    """
    if _launch_mgr.state not in (RUNNING, PAUSED):
        raise HTTPException(
            status_code=409,
            detail=f"当前状态 {_launch_mgr.state} 无法移到工作位（需遥操运行中）",
        )
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    node.publish_disarm()
    await asyncio.sleep(0.1)
    node.publish_init()
    return ApiEnvelope(
        ok=True,
        message="已下发工作位：双臂移向 init_waypoints → init_pose",
    )


@app.post("/api/v1/teleop/workpos/direct")
async def teleop_workpos_direct() -> ApiEnvelope:
    """段间回位直达（go-to-init direct）：双臂从当前位姿**直接**（关节空间
    直线插补，不经 init_waypoints）回到 init_pose——数采段与段之间快速回
    工作位，与左 X / 数采卡片「段间回位」同功能。区别于「工作位」（途经点）。

    Sequence: 发布 /teleop/disarm + /teleop/init_direct（disarm 先于 init，
    避免臂节点先收 init 启动回位、再收 disarm 取消回位）。仅遥操运行中可用。
    """
    if _launch_mgr.state not in (RUNNING, PAUSED):
        raise HTTPException(
            status_code=409,
            detail=f"当前状态 {_launch_mgr.state} 无法段间回位（需遥操运行中）",
        )
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    node.publish_disarm()
    await asyncio.sleep(0.1)
    node.publish_init_direct()
    return ApiEnvelope(
        ok=True,
        message="已下发段间回位：双臂直接移向 init_pose（不经途径点）",
    )



@app.post("/api/v1/restart")
async def restart() -> ApiEnvelope:
    """Restart the current preset: stop then start the same preset.

    Only valid for a launch started via this monitor's preset manager. If the
    teleop was started from a separate CLI, use stop+start manually instead.
    """
    name = _launch_mgr.preset
    if not name or name not in _presets:
        raise HTTPException(status_code=409, detail="无当前预设可重启（CLI 启动的遥操不支持重启）")
    ok_stop, _ = _launch_mgr.stop()
    # Wait briefly for the stop to take effect before restarting.
    await asyncio.sleep(1.0)
    ok_start, msg = _launch_mgr.start(_presets[name])
    if not ok_start:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=f"已重启预设: {name}")


# --- Robot hardware mode (driver services) ----------------------------------
# These call the driver's already-exposed Trigger services. The driver is the
# hardware authority; the monitor only invokes its services (non-intrusive).
# All run in a worker thread so the ROS spin thread can complete the future.
#
# 手动硬件模式按钮（急停/阻尼/一键就绪/归零/位置保持）会改变臂的受控目标或
# 释放臂。若遥操臂节点仍在 armed 或启动归位(homing)中持续发流，driver 显式
# 下发的一次性目标会被其下一帧命令覆盖（症状：阻尼后点一键就绪，臂回到阻尼
# 前位姿 / 阻尼中归零无效）。因此每次先发 /teleop/disarm 把臂节点带出轨迹，
# 再调 driver 服务；之后要遥操需重新 /teleop/start。

def _driver_call(name: str) -> tuple[bool, str]:
    node = get_node()
    if node is None:
        return False, "ROS 节点未就绪"
    return node.call_driver_service(name)


def _disarm_before_hardware() -> None:
    """硬件模式前置：发布 /teleop/disarm（臂节点同时取消进行中的 homing）。

    若当前没跑遥操（无订阅者）发布是无害空操作；对 CLI 起的遥操同样生效。
    """
    node = get_node()
    if node is None:
        return
    try:
        node.publish_disarm()
    except Exception:  # noqa: BLE001
        pass


# --- Driver-enable helpers (HOME only) ------------------------------------
# 启动/重启端点**不**再自动调 ~/enable（曾导致"启动后即 503 / 臂不动"，
# 已回退：启动只是把栈拉起来，使能交给 driver auto_ready/一键就绪）。
# 这里仅 HOME 端点使用：先把电机使能起来，再 disarm + 走收回轨迹。
def _preset_has_driver(preset) -> bool:
    """该预设是否把 astral_robot_control（真机 driver）拉起来。"""
    if preset is None:
        return False
    wa = str(preset.args.get("with_arm_driver", "")).lower()
    if wa in ("true", "1", "yes"):
        return True
    return preset.package == "astral_robot_control"


def _current_preset():
    name = _launch_mgr.preset
    return _presets.get(name) if name else None


def _ensure_driver_enabled(timeout_s: float) -> tuple[bool, str]:
    """确保真机电机已使能（**不回零**：~/enable = WORK→POSITION→enable）。

    仅 HOME 端点使用：HOME 需要电机带载走收回轨迹，先调用本函数确认上电。
    driver 侧已做幂等：已上电直接成功返回（跳过重复使能——实机发现已使能后
    再 enable 会因板端电源位不确认而误报"电机未使能"）；板端在线但电源位未
    确认也算下发成功。服务在超时内一直未出现视为无需使能（无 driver / sim），
    不算错误。
    """
    node = get_node()
    if node is None:
        return False, "ROS 节点未就绪"
    deadline = time.monotonic() + max(1.0, float(timeout_s))
    retryable = ("未就绪", "超时", "not confirmed", "unavailable", "robot not connected")
    last = ""
    while time.monotonic() < deadline:
        ok, msg = node.call_driver_service("enable", timeout_s=3.0)
        if ok:
            return True, msg or "电机已使能"
        last = msg
        if not any(t in msg for t in retryable):
            return False, f"使能失败: {msg}"
        time.sleep(0.4)
    if last and "未就绪" in last:
        # ~/enable 服务一直没出现：该预设没拉 driver（sim/调试）→ 跳过
        return True, "跳过（无 driver 服务）"
    return False, f"使能超时: {last or '~/enable 服务未出现'}"


@app.post("/api/v1/robot/ready")
async def robot_ready() -> ApiEnvelope:
    """一键就绪：先 disarm 遥操（停掉 homing/armed 发流），再 driver one_click_ready。"""
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "ready")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "一键就绪 OK")


@app.post("/api/v1/robot/home")
async def robot_home() -> ApiEnvelope:
    """归零：先 disarm 遥操，再 driver ~/home（阻尼中会自动切回 POSITION 再归零）。"""
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "home")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "全关节归零")


@app.post("/api/v1/robot/estop")
async def robot_estop() -> ApiEnvelope:
    """真急停：先 disarm 遥操（断电后不能让陈旧命令流挂着），再 driver ~/estop。"""
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "estop")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "已断电 (e_stop)")


@app.post("/api/v1/robot/damping")
async def robot_damping() -> ApiEnvelope:
    """阻尼释放：先 disarm 遥操，再 driver ~/damping → motion_mode=0，可手动拖拽。"""
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "damping")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "阻尼释放 (可手动拖拽)")


@app.post("/api/v1/robot/position")
async def robot_position() -> ApiEnvelope:
    """位置保持：先 disarm 遥操，再 driver ~/position → motion_mode=1。"""
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "position")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "位置保持")


# --- Video return gate (quest3_video_streamer) ------------------------------
# Non-intrusive: the streamer owns the cameras; the monitor only mirrors its
# latched gate_state (which carries the auto-scanned camera list), calls its
# SetBool master switch, and publishes the latched active-cameras subset.
# When the streamer is offline the monitor scans the host itself so the user
# can pre-select cameras (latched topic is delivered once the streamer boots).

def _sysfs_video_name(device: str) -> str:
    """Human-readable camera name for /dev/videoN from sysfs ('' if unknown)."""
    if not device.startswith("/dev/video"):
        return ""
    try:
        with open(f"/sys/class/video4linux/{device[5:]}/name", "r", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def _host_scan() -> list[dict[str, Any]]:
    """Auto-scan host capture devices (same heuristic as the streamer)."""
    try:
        from quest3_video_streamer.scan import enumerate_capture_devices
        found = enumerate_capture_devices()
        return [
            {
                "label": d["label"],
                "device": d["device"],
                "source": "webcam",
                "preset": "",
                "exists": True,
                "sysfs_name": d.get("sysfs_name", ""),
            }
            for d in found
        ]
    except Exception:
        return []


def _with_live_info(cameras: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Refresh exists/sysfs_name per poll (devices may be (un)plugged live)."""
    out = []
    for cam in cameras:
        device = str(cam.get("device", ""))
        exists = bool(device.startswith("/dev/")) and os.path.exists(device)
        out.append({
            "label": str(cam.get("label", "")),
            "device": device,
            "source": str(cam.get("source", "")),
            "preset": str(cam.get("preset", "")),
            "exists": exists,
            "sysfs_name": _sysfs_video_name(device) if exists else str(cam.get("sysfs_name", "")),
        })
    return out


@app.get("/api/v1/video/status")
async def video_status() -> ApiEnvelope:
    node = get_node()
    gate = None
    alive = False
    if node is not None:
        gate = node.snapshot().get("video_gate")
        try:
            # gate_state is latched: a stale copy survives the streamer's death,
            # so also require a live publisher before calling it "online".
            # (count_publishers needs the fully-resolved name, not "~".)
            alive = node.count_publishers(
                f"{config.VIDEO_NODE}/gate_state"
            ) > 0
        except Exception:
            alive = gate is not None
    online = gate is not None and alive
    if online:
        cameras = _with_live_info(list(gate.get("cameras") or []))
    else:
        cameras = _host_scan()
    return ApiEnvelope(ok=True, data={
        "configured": cameras,
        "gate": gate,  # {push_enabled, configured, active, cameras} or None when offline
        "online": online,
    })


@app.post("/api/v1/video/push")
async def video_push(req: VideoPushRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    ok, msg = await asyncio.to_thread(node.set_video_push, req.enabled)
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or ("视频推送已开启" if req.enabled else "视频推送已关闭"))


@app.post("/api/v1/video/cameras")
async def video_cameras(req: VideoCamerasRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    node.publish_video_cameras(req.cameras)
    label = ", ".join(req.cameras) if req.cameras else "全部已配置相机"
    return ApiEnvelope(ok=True, message=f"已下发推送相机: {label}")


# --- Web live preview (MJPEG relay of the streamer's ~/preview/{label}) -----

@app.get("/api/v1/video/feed/{label}")
async def video_feed(label: str) -> StreamingResponse:
    """MJPEG stream for one camera label (browser <img> compatible)."""
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    if label not in node.preview_labels():
        raise HTTPException(status_code=404, detail=f"未知相机: {label}")

    async def gen():
        last_seq = -1
        try:
            while True:
                jpeg, seq = node.preview_frame(label)
                if jpeg is not None and seq != last_seq:
                    last_seq = seq
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                await asyncio.sleep(1.0 / 15.0)
        except (asyncio.CancelledError, GeneratorExit):
            return

    return StreamingResponse(
        gen(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/api/v1/video/snapshot/{label}")
async def video_snapshot(label: str) -> Response:
    """Latest preview frame as a single JPEG."""
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    if label not in node.preview_labels():
        raise HTTPException(status_code=404, detail=f"未知相机: {label}")
    jpeg, _ = node.preview_frame(label)
    if jpeg is None:
        raise HTTPException(status_code=404, detail=f"{label} 暂无预览帧")
    return Response(content=jpeg, media_type="image/jpeg")


# --- WebSocket -------------------------------------------------------------
# --- data collection (astral_data_collect) --------------------------------
# 控制面是纯话题接口：采集节点不在本监控的启动管理内也能工作（CLI 启动同样
# 收得到命令）；节点离线时命令会无人接收，前端靠 state 徽标提示在线性。


@app.post("/api/v1/collect/control")
async def collect_control(req: CollectControlRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    cmd = req.cmd.strip().lower()
    if cmd not in config.DC_COMMANDS:
        raise HTTPException(
            status_code=400, detail=f"未知命令 {cmd!r}，可选 {config.DC_COMMANDS}"
        )
    node.publish_dc_control(cmd)
    return ApiEnvelope(ok=True, message=f"已发送录制命令: {cmd}")


@app.post("/api/v1/collect/task")
async def collect_task(req: CollectTaskRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    text = req.text.strip()
    if not text:
        raise HTTPException(status_code=400, detail="任务文本不能为空")
    node.publish_dc_task(text)
    return ApiEnvelope(ok=True, message=f"已设置下一段任务: {text}")


@app.post("/api/v1/collect/session")
async def collect_session(req: CollectSessionRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    text = req.text.strip()
    if not text:
        raise HTTPException(status_code=400, detail="目录名不能为空")
    node.publish_dc_session(text)
    return ApiEnvelope(ok=True, message=f"已切换录制目录: {text}")


# 数采节点泳道（独立 LaunchManager，与遥操预设生命周期解耦）。
# 预设来源：presets.yaml 里 package=astral_data_collect 的条目（单一可调来源）。


def _collect_preset():
    for p in _presets.values():
        if p.package == "astral_data_collect":
            return p
    return None


@app.post("/api/v1/collect/launch/start")
async def collect_launch_start() -> ApiEnvelope:
    preset = _collect_preset()
    if preset is None:
        raise HTTPException(
            status_code=404, detail="presets.yaml 中没有 package=astral_data_collect 的预设"
        )
    ok, msg = _collect_mgr.start(preset, check_orphan=False)
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/collect/launch/stop")
async def collect_launch_stop() -> ApiEnvelope:
    ok, msg = _collect_mgr.stop()
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/collect/launch/restart")
async def collect_launch_restart() -> ApiEnvelope:
    """重启数采节点——改了 data_collect.yaml 的 schema 配置后用它生效。

    必须等旧进程真正退出再启动：采集节点持 domain 级单例锁，旧进程没死透
    时新节点会拒绝启动（SIGINT 优雅期最长 30s）。
    """
    preset = _collect_preset()
    if preset is None:
        raise HTTPException(
            status_code=404, detail="presets.yaml 中没有 package=astral_data_collect 的预设"
        )
    _collect_mgr.stop()
    deadline = time.monotonic() + STOP_SIGINT_TIMEOUT_S + 5.0
    while _collect_mgr.state != STOPPED:
        if time.monotonic() > deadline:
            raise HTTPException(status_code=409, detail="旧数采进程未能在超时内退出")
        await asyncio.sleep(0.5)
    ok, msg = _collect_mgr.start(preset, check_orphan=False)
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=f"已重启数采节点: {preset.name}")


# --- 推理节点泳道 + 控制面（astral_policy_inference）--------------------------
# 控制面是纯话题（/policy_inference/cmd + /task），web 按钮与 policy_keyboard
# 完全等价，CLI 启动的推理节点同样可控。泳道启动跳过孤儿检测：推理节点设计上
# 与遥操栈共存（takeover 仲裁），不能被"有遥操 launch 在跑"挡掉。
# launch 参数构建/命令白名单在 config（无 ROS 依赖，可离线单测）。


def _policy_preset(cfg: dict) -> Preset:
    return Preset(
        name="Policy inference (web)",
        package="astral_policy_inference",
        launch="policy_inference.launch.py",
        args=config.policy_launch_args(cfg),
        description="推理节点（配置化 web 泳道）",
    )


@app.post("/api/v1/infer/launch/start")
async def infer_launch_start(req: InferLaunchRequest) -> ApiEnvelope:
    global _policy_cfg
    cfg = req.model_dump()
    _policy_cfg = cfg
    preset = _policy_preset(cfg)
    ok, msg = _policy_mgr.start(preset, check_orphan=False)
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/infer/launch/stop")
async def infer_launch_stop() -> ApiEnvelope:
    ok, msg = _policy_mgr.stop()
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message=msg)


@app.post("/api/v1/infer/launch/restart")
async def infer_launch_restart() -> ApiEnvelope:
    """重启推理节点——用最近一次 web 启动配置（改了 host/尺寸/日志后用它生效）。"""
    if not _policy_cfg:
        raise HTTPException(status_code=409, detail="无推理启动配置（请先经 web 启动一次）")
    _policy_mgr.stop()
    deadline = time.monotonic() + STOP_SIGINT_TIMEOUT_S + 5.0
    while _policy_mgr.state != STOPPED:
        if time.monotonic() > deadline:
            raise HTTPException(status_code=409, detail="旧推理进程未能在超时内退出")
        await asyncio.sleep(0.5)
    ok, msg = _policy_mgr.start(_policy_preset(_policy_cfg), check_orphan=False)
    if not ok:
        raise HTTPException(status_code=409, detail=msg)
    return ApiEnvelope(ok=True, message="已重启推理节点（沿用最近配置）")


@app.post("/api/v1/infer/cmd")
async def infer_cmd(req: InferCmdRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    cmd = req.cmd.strip().lower()
    if not config.valid_infer_cmd(cmd):
        raise HTTPException(
            status_code=400,
            detail=f"未知推理命令 {cmd!r}，可选 {config.PI_COMMANDS}（或 playback:<源>）",
        )
    published = await asyncio.to_thread(node.publish_pi_cmd, cmd)
    if not published:
        raise HTTPException(
            status_code=503,
            detail=(
                "推理命令未发送：2 秒内未发现 /policy_inference/cmd 订阅者；"
                "请等待推理节点状态显示在线后重试"
            ),
        )
    return ApiEnvelope(ok=True, message=f"已发送推理命令: {cmd}")


@app.post("/api/v1/infer/task")
async def infer_task(req: InferTaskRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    text = req.text.strip()
    if not text:
        raise HTTPException(status_code=400, detail="任务文本不能为空")
    node.publish_pi_task(text)
    return ApiEnvelope(ok=True, message=f"已设置任务: {text}")


@app.websocket("/ws/telemetry")
async def ws_telemetry(websocket: WebSocket):
    await _conn.connect(websocket)
    # Send an immediate snapshot so the client doesn't wait for the next tick.
    snapshot = await asyncio.to_thread(_build_ui_state)
    await websocket.send_text(json.dumps(snapshot, ensure_ascii=False))
    try:
        while True:
            raw = await websocket.receive_text()
            if raw == "ping":
                await websocket.send_text(json.dumps({"type": "pong", "ts": time.time()}))
    except WebSocketDisconnect:
        _conn.disconnect(websocket)
    except Exception:
        _conn.disconnect(websocket)


# --- SPA static hosting ----------------------------------------------------
if _web_dist.is_dir():
    assets = _web_dist / "assets"
    if assets.is_dir():
        app.mount("/assets", StaticFiles(directory=str(assets)), name="assets")

    @app.get("/{full_path:path}")
    async def spa_fallback(full_path: str):
        # API and WS routes are matched first; anything else serves the SPA.
        if full_path.startswith(("api/", "ws/")):
            raise HTTPException(status_code=404)
        index = _web_dist / "index.html"
        if index.is_file():
            return FileResponse(str(index))
        raise HTTPException(status_code=404, detail="前端未构建")


def main() -> None:
    """Entry point (console_scripts). Runs uvicorn on the main thread."""
    import uvicorn
    uvicorn.run(
        app,
        host=config.WEB_HOST,
        port=config.WEB_PORT,
        log_level=os.environ.get("ASTRAL_WEB_MONITOR_LOG_LEVEL", "info"),
    )


if __name__ == "__main__":
    main()
