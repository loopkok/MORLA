"""Pydantic models for REST request/response envelopes.

Follows the rob_station convention: REST returns {ok, message, data} for
success and HTTP 409 with {detail} for command conflicts.
"""
from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class ApiEnvelope(BaseModel):
    ok: bool
    message: str = ""
    data: Any = None


class DataProcessRequest(BaseModel):
    action: Literal['full', 'raw-check', 'quantify', 'align', 'validate', 'repair-preview',
                    'repair', 'verify', 'export', 'preview', 'upgrade-v3'] = 'full'
    source: str
    target: Literal['openpi', 'act', 'both'] = 'openpi'
    image_size: int = Field(default=224, ge=0, le=2048)
    camera_map: str = ''
    episodes: str = ''
    quantify: bool = True
    repair: bool = False
    reference: str = ''


class StartRequest(BaseModel):
    preset: str
    # 勾选则给遥操预设注入 teleop_log_file（仅 morla_bringup/astral_arm_teleop 预设，
    # 落 /tmp/teleop_teleop_{left,right}.jsonl；见 config.TELEOP_LOG_*）
    log: bool = False


class ProfileSelectRequest(BaseModel):
    profile_id: str
    operating_mode: Literal["sim", "real"] = "sim"


class ProfileLaunchRequest(BaseModel):
    profile_id: str
    operating_mode: Literal["sim", "real"] = "sim"


class PresetInfo(BaseModel):
    name: str
    package: str
    launch: str
    args: dict[str, str] = {}
    description: str = ""


class VideoPushRequest(BaseModel):
    enabled: bool


class VideoCamerasRequest(BaseModel):
    # Active camera label subset; empty list = all configured cameras.
    cameras: list[str] = []


class CollectControlRequest(BaseModel):
    # 录制命令：start/stop/discard/next/pause/resume（白名单在后端校验）
    cmd: str


class CollectTaskRequest(BaseModel):
    # 任务文本（应用于下一段 episode，latched）
    text: str


class CollectSessionRequest(BaseModel):
    # 录制目录（session 名，仅 IDLE 生效；latched）
    text: str


class InferCmdRequest(BaseModel):
    # 推理命令：policy/playback/pause/resume/takeover/release/stop
    # （playback 可带源 "playback:<path>[:<ep>]"；白名单在后端校验）
    cmd: str


class InferTaskRequest(BaseModel):
    # 任务文本（语言指令，latched）
    text: str


class InferLaunchRequest(BaseModel):
    """推理节点泳道启动配置 → policy_inference.launch.py 显式参数。"""
    backend_type: str = "remote"       # remote | inproc | stub
    model: str = "pi05"                # act | pi05 | ...
    host: str = "127.0.0.1"            # GPU 主机 IP
    port: int = 8001                   # serve 端口
    camera_image_size: int = 224       # 模型输入尺寸（pi0.5 默认 224）
    engine_mode: str = "queue_async"   # queue_async | queue_sync | rtc
    log: bool = False                  # 开启则自动记录控制、指令、指标及相机全链路诊断
