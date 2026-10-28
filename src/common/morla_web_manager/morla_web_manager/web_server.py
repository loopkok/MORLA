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
import logging
import os
import time
from urllib.parse import urlsplit
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from . import config
from .data_process_manager import DataProcessManager, PipelineConflict
from .replay_browser import browse_replay_sources
from .quest_preview import QuestPreview
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
from .piano_manager import PianoManager
from .piano_store import PianoConflict, PianoError, PianoStore
from .composable_service import ComposableService
from .composable_api import router as composable_router
try:
    from morla_core.profile import ProfileRegistry
    from morla_bringup.robot_profile_launch import select_robot_stack
    _profile_registry = ProfileRegistry.from_index()
    _robot_profiles = {
        profile_id: _profile_registry.load(profile_id)
        for profile_id in _profile_registry.ids()
    }
    _profile_error = ""
except Exception as exc:  # keep health/log endpoints usable in partial installs
    _profile_registry = None
    _robot_profiles = {}
    _profile_error = str(exc)
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
    ProfileSelectRequest,
    ProfileLaunchRequest,
)


# --- globals ---------------------------------------------------------------
_launch_mgr = LaunchManager()
_quest_preview = QuestPreview(
    enabled=os.environ.get("MORLA_QUEST_PREVIEW", "false").lower() == "true",
    port=int(os.environ.get("MORLA_QUEST_PREVIEW_PORT", "8000")),
)
# 数采独立泳道：与遥操预设生命周期完全解耦（用户决策：web 单独启停）。
# 纯订阅者、不抢 joint_commands，start 时跳过孤儿检测；遥操侧孤儿检测
# 对 morla_data_collect 命令行有对称豁免（launch_manager._find_orphan）。
_collect_mgr = LaunchManager(lane_name="数采")
# 推理独立泳道：配置化构建 policy_inference.launch.py 命令。推理节点设计上就与
# 遥操共存（takeover 仲裁），启动也跳过孤儿检测——否则遥操栈运行时会挡启动。
_policy_mgr = LaunchManager(lane_name="推理")
# 最近一次推理启动配置（restart 复用；前端表单驱动）
_policy_cfg: dict = {}
_data_mgr = DataProcessManager(config.DATA_ROOT, config.DATA_PIPELINE_PYTHON,
                               config.DATA_PIPELINE_PACKAGE, config.DATA_CHECK_PYTHON)
_piano_store = PianoStore(
    Path(os.environ.get("MORLA_PIANO_PROJECT", str(config.MORLA_WS.parent / "MORLA-piano"))),
    Path(os.environ.get("MORLA_PIANO_DATA", str(config.MORLA_WS / ".morla_piano"))),
)
_piano_mgr = PianoManager(_piano_store)
_presets = load_presets()
_selected_profile_id = (
    "astral_dual_gripper_wuji" if "astral_dual_gripper_wuji" in _robot_profiles
    else next(iter(_robot_profiles), "")
)
_selected_operating_mode = "sim"
_composable = ComposableService(
    os.environ.get("MORLA_SYSTEM_STORE", str(Path.home() / ".local/share/morla")),
    blockers=lambda: [label for label, manager in (("teleop", _launch_mgr), ("collection", _collect_mgr),
                                                   ("policy", _policy_mgr))
                      if manager.state not in {STOPPED, "start_failed"}]
                     + (["piano"] if _piano_mgr.active else [])
                     + (["ROS coordinator"] if _live_control_state() else []),
)
_web_dist = Path(config._web_env(
    "DIST", str(config.MORLA_WS / 'src/common/morla_web_manager/web/dist')
))
if not _web_dist.is_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        _web_dist = Path(get_package_share_directory('morla_web_manager')) / 'web/dist'
    except Exception:
        pass


def _live_control_state(snapshot: dict | None = None) -> dict | None:
    node = get_node()
    if node is None:
        return None
    snap = snapshot if snapshot is not None else node.snapshot()
    state = snap.get("control")
    return state if isinstance(state, dict) and not state.get("stale") else None


def _effective_profile_id(snapshot: dict | None = None) -> str:
    control = _live_control_state(snapshot)
    if control and control.get("profile_id") in _robot_profiles:
        return str(control["profile_id"])
    return _selected_profile_id


def _effective_operating_mode(snapshot: dict | None = None) -> str:
    control = _live_control_state(snapshot)
    if control and control.get("operating_mode") in {"sim", "real"}:
        return str(control["operating_mode"])
    return _selected_operating_mode


def _profile_launch_preset(profile_id: str, operating_mode: str) -> Preset:
    """Build a validated robot stack launch from one immutable profile."""
    if profile_id not in _robot_profiles:
        raise ValueError(f"unknown robot profile {profile_id!r}")
    if operating_mode not in {"sim", "real"}:
        raise ValueError("operating_mode must be sim or real")
    profile = _robot_profiles[profile_id]
    sdk_backend = "hardware" if operating_mode == "real" else "fake"
    select_robot_stack(profile, operating_mode, sdk_backend)
    args = {
        "profile_id": profile_id,
        "mode": operating_mode,
        "sdk_backend": sdk_backend,
        "with_input": "true",
        "allow_legacy_packets": "false",
        "enable_viewer": "false",
        "with_video": "true" if operating_mode == "real" else "false",
        "with_head_teleop": "false",
    }
    return Preset(
        name=f"{profile_id} ({operating_mode})", package="morla_bringup",
        launch="robot.launch.py", args=args,
        description="Unified MORLA profile-driven robot stack",
    )


def _profile_switch_blockers(profile_id: str, operating_mode: str) -> list[str]:
    """Refuse a profile/mode change while any lane or CLI writer is active."""
    changed = profile_id != _selected_profile_id or operating_mode != _selected_operating_mode
    blockers = []
    if changed:
        for label, manager in (("teleop", _launch_mgr), ("collector", _collect_mgr), ("inference", _policy_mgr)):
            if manager.state != STOPPED:
                blockers.append(f"{label} web lane is {manager.state}")
    node = get_node()
    snap = node.snapshot() if node is not None else {}
    control = _live_control_state(snap)
    if control:
        active_id = str(control.get("profile_id", ""))
        active_mode = str(control.get("operating_mode", ""))
        if active_id != profile_id or active_mode != operating_mode:
            blockers.append(f"CLI/ROS coordinator is active for {active_id} ({active_mode})")
    elif node is not None and changed:
        writers = node.robot_command_publishers()
        if writers:
            blockers.append("robot command writers are active: " + ", ".join(sorted(writers)))
    if node is not None:
        dc = snap.get("data_collect") or {}
        dc_profile = dc.get("profile_id") or (dc.get("schema") or {}).get("profile_id")
        dc_mode = dc.get("runtime_mode") or dc.get("recording_mode")
        dc_live = not dc.get("stale") and int(dc.get("node_count", 0)) > 0
        if dc_live and (dc_profile != profile_id or dc_mode not in (None, "unknown", operating_mode)):
            blockers.append(
                f"collector is configured for {dc_profile or 'unknown'} ({dc_mode or 'unknown'})"
            )
        elif changed and dc_live:
            blockers.append("a CLI or web collector is still publishing state")
        infer = snap.get("infer") or {}
        infer_live = not infer.get("stale") and int(infer.get("node_count", 0)) > 0
        if infer_live and infer.get("profile_id") != profile_id:
            blockers.append(f"policy node is configured for {infer.get('profile_id') or 'unknown'}")
        elif changed and infer_live:
            blockers.append("a CLI or web policy node is still publishing state")
    return blockers


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
        "quest_preview": _quest_preview.snapshot(),
        "data_process": _data_mgr.snapshot(),
        "infer": ros.get("infer"),
        "control": ros.get("control"),
        "piano": _piano_mgr.snapshot(),
        "profile_id": ros.get("profile_id"),
        "robot_streams": ros.get("robot_streams", {}),
        "profile_selection": {
            "profile_id": _effective_profile_id(ros),
            "requested_profile_id": _selected_profile_id,
            "operating_mode": _effective_operating_mode(ros),
            "active_profile_id": (ros.get("control") or {}).get("profile_id")
            if ros.get("control") and not ros.get("control", {}).get("stale") else None,
        },
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


async def _try_coordinated_control(operation: str) -> ApiEnvelope | None:
    """Use the active profile coordinator when its latched state is fresh."""
    node = get_node()
    if node is None:
        return None
    if not node.control_available():
        if node.control_state_seen():
            raise HTTPException(
                status_code=503,
                detail="MORLA coordinator state is stale; refusing legacy control fallback",
            )
        return None
    ok, message = await asyncio.to_thread(node.call_control_service, operation)
    return ApiEnvelope(ok=ok, message=message)


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
    preview = asyncio.create_task(_quest_preview_supervisor())
    async def composable_tick():
        while True:
            await asyncio.to_thread(_composable.tick)
            await asyncio.sleep(0.02)
    composed = asyncio.create_task(composable_tick())
    try:
        yield
    finally:
        composed.cancel()
        try:
            await composed
        except asyncio.CancelledError:
            pass
        await asyncio.to_thread(_composable.close)
        await asyncio.to_thread(_piano_mgr.close)
        preview.cancel()
        try:
            await preview
        except asyncio.CancelledError:
            pass
        await asyncio.to_thread(_quest_preview.close)
        broadcaster.cancel()
        try:
            await broadcaster
        except asyncio.CancelledError:
            pass
        shutdown_node()
        await asyncio.to_thread(_data_mgr.close)


async def _quest_preview_supervisor():
    previous = None
    while True:
        active = _launch_mgr.state not in {STOPPED, "start_failed"} or _live_control_state() is not None
        await asyncio.to_thread(_quest_preview.tick, active)
        state = _quest_preview.snapshot()
        if state["enabled"] and state["state"] != previous:
            logging.getLogger("uvicorn.error").info("Quest tracking receiver :%s: %s", state["port"], state["state"])
        previous = state["state"]
        await asyncio.sleep(0.5)


async def _start_teleop_preset(preset):
    return await asyncio.to_thread(_quest_preview.start_stack, lambda: _launch_mgr.start(preset))


async def _ui_state_broadcaster() -> None:
    interval = 1.0 / config.WS_PUSH_HZ
    while True:
        await asyncio.sleep(interval)
        if _conn.count <= 0:
            continue
        msg = await asyncio.to_thread(_build_ui_state)
        await _conn.broadcast(msg)


# --- app -------------------------------------------------------------------
app = FastAPI(title="morla_web_manager", version="0.1.0", lifespan=_lifespan)
app.include_router(composable_router(_composable))
from .request_boundary import ConfigurationBoundary
app.add_middleware(ConfigurationBoundary)


@app.get("/mobile")
def composable_mobile_page():
    return FileResponse(Path(__file__).with_name("static") / "mobile.html")

# Serialize all Web motion/lifecycle mutations with piano acquisition. Emergency
# requests bypass the mutex so a slow legacy service cannot delay stopping.
_motion_mutation_lock = asyncio.Lock()
_piano_exclusive_paths = ("/api/v1/control/", "/api/v1/robot/", "/api/v1/teleop/",
                          "/api/v1/profile/", "/api/v1/collect/", "/api/v1/infer/")
_piano_legacy_controls = {"/api/v1/start", "/api/v1/stop", "/api/v1/pause", "/api/v1/resume", "/api/v1/restart"}
_emergency_paths = {"/api/v1/control/estop", "/api/v1/robot/estop"}


@app.middleware("http")
async def piano_exclusion(request, call_next):
    path = request.url.path
    if request.method not in {"POST", "PUT", "PATCH", "DELETE"}:
        return await call_next(request)
    if path.startswith("/api/v1/piano/"):
        origin = request.headers.get("origin")
        if origin and urlsplit(origin).netloc != request.headers.get("host"):
            return JSONResponse(status_code=403, content={"detail": "拒绝跨来源钢琴控制请求"})
        try:
            if int(request.headers.get("content-length", "0")) > 1_000_000:
                return JSONResponse(status_code=413, content={"detail": "钢琴请求体过大"})
        except ValueError:
            return JSONResponse(status_code=400, content={"detail": "无效请求长度"})
    if path in _emergency_paths and _piano_mgr.active:
        try:
            _piano_mgr.stop(emergency=True)
            return JSONResponse({"ok": True, "message": "钢琴急停请求已发送；请检查实体反馈", "data": None})
        except PianoError as exc:
            return JSONResponse(status_code=503, content={"detail": str(exc)})
    guarded = path in _piano_legacy_controls or path.startswith(_piano_exclusive_paths)
    piano_mutation = path.startswith("/api/v1/piano/") and not path.endswith(("/heartbeat", "/stop", "/close"))
    if (guarded or piano_mutation) and _composable.active:
        return JSONResponse(status_code=409, content={"detail": "组合会话占用机器人；请停止组合会话后操作旧入口"})
    if guarded or piano_mutation:
        async with _motion_mutation_lock:
            if guarded and _piano_mgr.active:
                return JSONResponse(status_code=409, content={"detail": "钢琴会话占用机器人；停止并断开钢琴后可恢复原操作"})
            return await call_next(request)
    return await call_next(request)


def _piano_blockers(mode=None):
    blockers = []
    for label, manager in (("机器人/遥操", _launch_mgr), ("采集", _collect_mgr), ("推理", _policy_mgr)):
        if manager.state not in {STOPPED, "start_failed"}:
            blockers.append(f"{label}通道仍为 {manager.state}")
    node = get_node()
    if node is None and mode in {"ros_fake", "hardware"}:
        blockers.append("ROS监控尚未连接，不能核验现有硬件驱动")
    if node is not None:
        snap = node.snapshot()
        for key, label, topic in (("control", "机器人协调器", "/morla/control/state"),
                                  ("data_collect", "采集节点", "/data_collect/state"),
                                  ("infer", "策略节点", "/policy_inference/state")):
            state = snap.get(key) or {}
            if not state.get("stale", True) and int(state.get("node_count", 0)) > 0:
                # Monitor snapshots cache node_count at reception. Recheck the graph:
                # the previous piano lease can remain fresh after its publisher exits.
                if node.count_publishers(topic):
                    blockers.append(f"{label}仍在运行（可能从命令行启动）")
        if node.robot_command_publishers():
            blockers.append("已有 ROS 运动指令发布者，请先停止")
        if mode in {"ros_fake", "hardware"}:
            for side in ("left", "right"):
                for topic in (f"/{side}_arm/nero_robot_driver/status", f"/{side}_arm/joint_states",
                              f"/{side}_hand/adapter/status", f"/{side}_hand/driver/xhand_state"):
                    try:
                        if node.count_publishers(topic):
                            blockers.append(f"现有驱动仍占用 {topic}")
                    except Exception:
                        blockers.append(f"无法核验驱动占用 {topic}")
    return blockers


def _piano_call(function, *args, **kwargs):
    try:
        return function(*args, **kwargs)
    except PianoConflict as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except (PianoError, ValueError, TypeError, KeyError, OSError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/v1/piano/state")
async def piano_state():
    error = None
    try:
        _piano_store.model()
    except Exception as exc:
        error = str(exc)
    return ApiEnvelope(ok=True, data={**_piano_mgr.snapshot(), "available": error is None,
                                     "project_error": error, "blockers": [] if _piano_mgr.active else _piano_blockers()})


@app.get("/api/v1/piano/config")
async def piano_config():
    return ApiEnvelope(ok=True, data=_piano_call(_piano_store.config))


@app.post("/api/v1/piano/config")
async def piano_save_config(body: dict):
    _piano_call(_piano_mgr.editable)
    return ApiEnvelope(ok=True, message="参数已保存；重新连接后生效", data=_piano_call(
        _piano_store.save_config, body.get("config"), body.get("revision"), body.get("certify") is True))


@app.post("/api/v1/piano/config/validate")
async def piano_validate_config(body: dict):
    return ApiEnvelope(ok=True, data=_piano_call(_piano_store.validate, body.get("config")))


@app.get("/api/v1/piano/scores")
async def piano_scores():
    _piano_call(_piano_store.seed)
    return ApiEnvelope(ok=True, data=_piano_call(_piano_store.list_scores))


@app.get("/api/v1/piano/scores/{score_id}")
async def piano_score(score_id: str):
    return ApiEnvelope(ok=True, data=_piano_call(_piano_store.score, score_id))


@app.post("/api/v1/piano/scores")
async def piano_import_score(body: dict):
    _piano_call(_piano_mgr.editable)
    return ApiEnvelope(ok=True, message="曲谱已导入；演奏前请预检", data=_piano_call(
        _piano_store.import_score, body.get("name", ""), body.get("content", ""), body.get("format", "json"),
        body.get("tempo_bpm", 60), body.get("tonic", "C4")))


@app.delete("/api/v1/piano/scores/{score_id}")
async def piano_delete_score(score_id: str):
    _piano_call(_piano_mgr.editable)
    _piano_call(lambda: _piano_store._score_path(score_id).unlink(missing_ok=True))
    return ApiEnvelope(ok=True, message="曲谱已删除")


@app.post("/api/v1/piano/plan")
async def piano_plan(body: dict):
    _, plan = _piano_call(_piano_store.plan, body.get("score_id", ""), body.get("tempo_bpm"))
    return ApiEnvelope(ok=True, data={**plan, "config_revision": _piano_call(_piano_store.config)["revision"]})


@app.post("/api/v1/piano/session")
async def piano_open(body: dict):
    blockers = _piano_blockers(body.get("mode", "software"))
    if blockers:
        raise HTTPException(status_code=409, detail="；".join(blockers))
    result = _piano_call(_piano_mgr.open, body.get("mode", "software"), body.get("revision"), body.get("options"))
    return ApiEnvelope(ok=True, message="独立钢琴会话已启动，等待反馈", data=result)


@app.post("/api/v1/piano/heartbeat")
async def piano_heartbeat(body: dict):
    _piano_call(_piano_mgr.heartbeat, body.get("token"))
    return ApiEnvelope(ok=True)


@app.post("/api/v1/piano/command")
async def piano_command(body: dict):
    arguments = {key: value for key, value in body.items() if key not in {"token", "operation"}}
    result = _piano_call(_piano_mgr.command, body.get("token"), body.get("operation"), **arguments)
    return ApiEnvelope(ok=True, message="操作已排队，以实际状态为准", data=result)


@app.post("/api/v1/piano/stop")
async def piano_stop(body: dict | None = None):
    _piano_call(_piano_mgr.stop, emergency=(body or {}).get("emergency") is True)
    return ApiEnvelope(ok=True, message="停止请求已发送，请等待执行状态更新")


@app.post("/api/v1/piano/close")
async def piano_close(body: dict):
    _piano_call(_piano_mgr.disconnect, body.get("token"))
    return ApiEnvelope(ok=True, message="正在停止并释放钢琴会话")


@app.post("/api/v1/control/{operation}")
async def control_operation(operation: str) -> ApiEnvelope:
    """Profile-driven shared control endpoint; success means accepted, not complete."""
    node = get_node()
    if node is None or not node.control_available():
        raise HTTPException(status_code=503, detail="MORLA profile control coordinator is offline")
    ok, message = await asyncio.to_thread(node.call_control_service, operation)
    return ApiEnvelope(ok=ok, message=message)


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


@app.get("/api/v1/profiles")
async def list_robot_profiles() -> ApiEnvelope:
    if not _robot_profiles:
        raise HTTPException(status_code=503, detail=f"robot profiles unavailable: {_profile_error}")
    node = get_node()
    snap = node.snapshot() if node is not None else {}
    active = _live_control_state(snap)
    items = []
    for profile_id, profile in sorted(_robot_profiles.items()):
        values = profile.values
        arms = [row for row in values.get("components", [])
                if row.get("enabled") and row.get("type") in {"arm", "nero_arm"}]
        hands = [row for row in values.get("components", [])
                 if row.get("enabled") and row.get("type") in {"wuji_hand", "xhand"}]
        modes = []
        for mode in ("sim", "real"):
            try:
                _profile_launch_preset(profile_id, mode)
                modes.append(mode)
            except ValueError:
                continue
        items.append({
            "profile_id": profile_id,
            "title": values.get("title", profile_id),
            "robot_id": profile.robot_id,
            "state_dim": profile.frozen_schema.state_dim,
            "action_dim": profile.frozen_schema.action_dim,
            "kinematics_hash": profile.frozen_schema.kinematics_hash,
            "coordinate_contract": profile.frozen_schema.metadata.get("coordinate_contract", {}),
            "arms": [{"side": row["side"], "type": row["type"]} for row in arms],
            "hands": [{"side": row["side"], "type": row["type"],
                       "dim": len(row.get("joint_names", []))} for row in hands],
            "capabilities": dict(values.get("capabilities", {})),
            "launch_modes": modes,
        })
    return ApiEnvelope(ok=True, data={
        "profiles": items,
        "profile_id": _effective_profile_id(snap),
        "requested_profile_id": _selected_profile_id,
        "operating_mode": _selected_operating_mode,
        "active_profile_id": active.get("profile_id") if active else None,
        "active_operating_mode": active.get("operating_mode") if active else None,
    })


@app.post("/api/v1/profile/select")
async def select_robot_profile(req: ProfileSelectRequest) -> ApiEnvelope:
    global _selected_profile_id, _selected_operating_mode
    if req.profile_id not in _robot_profiles:
        raise HTTPException(status_code=404, detail=f"unknown robot profile {req.profile_id!r}")
    blockers = _profile_switch_blockers(req.profile_id, req.operating_mode)
    if blockers:
        raise HTTPException(status_code=409, detail="profile change blocked: " + "; ".join(blockers))
    _selected_profile_id = req.profile_id
    _selected_operating_mode = req.operating_mode
    return ApiEnvelope(ok=True, message=f"selected {req.profile_id} ({req.operating_mode})")


@app.post("/api/v1/profile/launch")
async def launch_robot_profile(req: ProfileLaunchRequest) -> ApiEnvelope:
    global _selected_profile_id, _selected_operating_mode
    if req.profile_id not in _robot_profiles:
        raise HTTPException(status_code=404, detail=f"unknown robot profile {req.profile_id!r}")
    try:
        preset = _profile_launch_preset(req.profile_id, req.operating_mode)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    blockers = _profile_switch_blockers(req.profile_id, req.operating_mode)
    if blockers:
        raise HTTPException(status_code=409, detail="profile launch blocked: " + "; ".join(blockers))
    node = get_node()
    active = _live_control_state(node.snapshot() if node is not None else {})
    if active and int(active.get("node_count", 0)) > 0:
        raise HTTPException(status_code=409, detail="a profile-driven robot stack is already active")
    _selected_profile_id = req.profile_id
    _selected_operating_mode = req.operating_mode
    ok, message = await _start_teleop_preset(preset)
    if not ok:
        raise HTTPException(status_code=409, detail=message)
    return ApiEnvelope(ok=True, message=message)


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
    global _selected_profile_id, _selected_operating_mode
    preset = _presets.get(req.preset)
    if preset is None:
        raise HTTPException(status_code=404, detail=f"未知预设: {req.preset}")
    profile_id = preset.args.get("control_profile_id") or preset.args.get("profile_id")
    if profile_id in _robot_profiles:
        mode = ("real" if preset.args.get("mode") == "real"
                or preset.args.get("sdk_backend") == "hardware"
                or preset.args.get("with_arm_driver") == "true" else "sim")
        blockers = _profile_switch_blockers(profile_id, mode)
        if blockers:
            raise HTTPException(status_code=409, detail="profile launch blocked: " + "; ".join(blockers))
        _selected_profile_id = profile_id
        _selected_operating_mode = mode
    if req.log:
        # 遥操日志开关：给白名单预设注入 log_dir 根目录（副本，不改 presets.yaml 源）。
        # launch 每次自动建 {log_dir}/{stamp}[_tag]/ 运行子目录、按侧拆 _left/_right。
        preset = preset_with_teleop_log(preset, config.TELEOP_LOG_ROOT)
    ok, msg = await _start_teleop_preset(preset)
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
    coordinated = await _try_coordinated_control("pause")
    if coordinated is not None:
        return coordinated
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
    coordinated = await _try_coordinated_control("resume")
    if coordinated is not None:
        return coordinated
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
    coordinated = await _try_coordinated_control("start")
    if coordinated is not None:
        return coordinated
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
    coordinated = await _try_coordinated_control("home")
    if coordinated is not None:
        return coordinated
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
    coordinated = await _try_coordinated_control("workpos")
    if coordinated is not None:
        return coordinated
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
    coordinated = await _try_coordinated_control("workpos_direct")
    if coordinated is not None:
        return coordinated
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
    ok_start, msg = await _start_teleop_preset(_presets[name])
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
    coordinated = await _try_coordinated_control("ready")
    if coordinated is not None:
        return coordinated
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "ready")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "一键就绪 OK")


@app.post("/api/v1/robot/home")
async def robot_home() -> ApiEnvelope:
    """归零：先 disarm 遥操，再 driver ~/home（阻尼中会自动切回 POSITION 再归零）。"""
    coordinated = await _try_coordinated_control("home")
    if coordinated is not None:
        return coordinated
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "home")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "全关节归零")


@app.post("/api/v1/robot/estop")
async def robot_estop() -> ApiEnvelope:
    """真急停：先 disarm 遥操（断电后不能让陈旧命令流挂着），再 driver ~/estop。"""
    coordinated = await _try_coordinated_control("estop")
    if coordinated is not None:
        return coordinated
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "estop")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "已断电 (e_stop)")


@app.post("/api/v1/robot/damping")
async def robot_damping() -> ApiEnvelope:
    """阻尼释放：先 disarm 遥操，再 driver ~/damping → motion_mode=0，可手动拖拽。"""
    coordinated = await _try_coordinated_control("damping")
    if coordinated is not None:
        return coordinated
    _disarm_before_hardware()
    ok, msg = await asyncio.to_thread(_driver_call, "damping")
    if not ok:
        raise HTTPException(status_code=503, detail=msg)
    return ApiEnvelope(ok=True, message=msg or "阻尼释放 (可手动拖拽)")


@app.post("/api/v1/robot/position")
async def robot_position() -> ApiEnvelope:
    """位置保持：先 disarm 遥操，再 driver ~/position → motion_mode=1。"""
    coordinated = await _try_coordinated_control("position")
    if coordinated is not None:
        return coordinated
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
# --- data collection (morla_data_collect) --------------------------------
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
# 预设来源：presets.yaml 里 package=morla_data_collect 的条目（单一可调来源）。


def _collect_preset():
    for p in _presets.values():
        if p.package == "morla_data_collect":
            profile_id = _effective_profile_id()
            operating_mode = _effective_operating_mode()
            return Preset(
                name=f"{p.name} [{profile_id}]" if profile_id else p.name,
                package=p.package, launch=p.launch,
                args={**p.args, "profile_id": profile_id, "runtime_mode": operating_mode,
                      "keyboard": "false", "vr_control": "false"},
                description=p.description,
            )
    return None


@app.post("/api/v1/collect/launch/start")
async def collect_launch_start() -> ApiEnvelope:
    preset = _collect_preset()
    if preset is None:
        raise HTTPException(
            status_code=404, detail="presets.yaml 中没有 package=morla_data_collect 的预设"
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
            status_code=404, detail="presets.yaml 中没有 package=morla_data_collect 的预设"
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


# --- 推理节点泳道 + 控制面（morla_policy_inference）--------------------------
# 控制面是纯话题（/policy_inference/cmd + /task），web 按钮与 policy_keyboard
# 完全等价，CLI 启动的推理节点同样可控。泳道启动跳过孤儿检测：推理节点设计上
# 与遥操栈共存（takeover 仲裁），不能被"有遥操 launch 在跑"挡掉。
# launch 参数构建/命令白名单在 config（无 ROS 依赖，可离线单测）。


def _policy_preset(cfg: dict) -> Preset:
    return Preset(
        name="Policy inference (web)",
        package="morla_policy_inference",
        launch="policy_inference.launch.py",
        args=config.policy_launch_args(cfg),
        description="推理节点（配置化 web 泳道）",
    )


@app.post("/api/v1/infer/launch/start")
async def infer_launch_start(req: InferLaunchRequest) -> ApiEnvelope:
    global _policy_cfg
    cfg = req.model_dump()
    cfg["profile_id"] = _effective_profile_id()
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


@app.get("/api/v1/infer/replay/sources")
def infer_replay_sources(path: str = "") -> ApiEnvelope:
    """Read-only playback source browsing on configured data roots."""
    try:
        data = browse_replay_sources(config.REPLAY_BROWSE_ROOTS, path)
    except PermissionError as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except (OSError, ValueError, RuntimeError) as exc:
        raise HTTPException(status_code=400, detail=f"无法浏览目录：{exc}") from exc
    return ApiEnvelope(ok=True, data=data)


@app.post("/api/v1/infer/cmd")
async def infer_cmd(req: InferCmdRequest) -> ApiEnvelope:
    node = get_node()
    if node is None:
        raise HTTPException(status_code=503, detail="ROS 节点未就绪")
    cmd = config.normalize_infer_cmd(req.cmd)
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
        log_level=config._web_env("LOG_LEVEL", "info"),
    )


if __name__ == "__main__":
    main()
