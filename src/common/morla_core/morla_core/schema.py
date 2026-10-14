"""采集 schema：可配置的 state/action 布局与数据流映射。

录制前通过参数/yaml 决定：
  * 左/右末端类型（gripper / wuji / none）
  * 是否纳入腰（waist 2 维）与头（head 2 维）
  * 参与采集的相机 label 列表
  * 数据集帧率与 action 语义（next_state | command）

schema 在 episode 开始时冻结并写入 meta.json，离线对齐/导出全程以它为准。
"""

from __future__ import annotations

import json
import hashlib
import math
from dataclasses import dataclass, field, asdict
from typing import Any

EE_GRIPPER = "gripper"
EE_WUJI = "wuji"
EE_NONE = "none"
EE_TYPES = (EE_GRIPPER, EE_WUJI, EE_NONE)

ACTION_NEXT_STATE = "next_state"
ACTION_COMMAND = "command"
ACTION_SOURCES = (ACTION_NEXT_STATE, ACTION_COMMAND)

SIDES = ("left", "right")

# astral 全身 18 维反馈 /astral/joint_states 的固定布局：
# [left_arm(7), right_arm(7), waist(2), head(2)]
BODY_STATE_DIM = 18
BODY_WAIST_SLICE = (14, 16)
BODY_HEAD_SLICE = (16, 18)

ARM_DIM = 7
HAND_DIM = 20
HEAD_DIM = 2
WAIST_DIM = 2
GRIPPER_DIM = 1

# 关节流名 → (ROS 话题后缀, 维度)。全部流名同时是 HDF5 /streams/ 下的组名。
STREAM_TOPICS = {
    "body_state": ("/astral/joint_states", BODY_STATE_DIM),
    "left_arm_state": ("/left_arm/joint_states", ARM_DIM),
    "right_arm_state": ("/right_arm/joint_states", ARM_DIM),
    "left_arm_cmd": ("/left_arm/joint_commands", ARM_DIM),
    "right_arm_cmd": ("/right_arm/joint_commands", ARM_DIM),
    "left_hand_state": ("/left_hand/joint_states", HAND_DIM),
    "right_hand_state": ("/right_hand/joint_states", HAND_DIM),
    "left_hand_cmd": ("/left_hand/joint_commands", HAND_DIM),
    "right_hand_cmd": ("/right_hand/joint_commands", HAND_DIM),
    # 夹爪：/command 是 Float64 闭合比(0..1)，joint_states 是 rad 回显
    "left_gripper_ratio": ("/left_gripper/command", GRIPPER_DIM),
    "right_gripper_ratio": ("/right_gripper/command", GRIPPER_DIM),
    "left_gripper_rad": ("/left_gripper/joint_states", GRIPPER_DIM),
    "right_gripper_rad": ("/right_gripper/joint_states", GRIPPER_DIM),
    "head_state": ("/head/joint_states", HEAD_DIM),
    "head_cmd": ("/head/joint_commands", HEAD_DIM),
}


@dataclass(frozen=True)
class StateBlock:
    """state/action 向量中的一个连续段。"""

    name: str          # 块名（left_arm / right_ee / waist / ...）
    dim: int
    state_stream: str  # 该块 state 来源流（robot_data.h5 /streams/<name>）
    cmd_stream: str | None  # command 模式下的 action 来源流；None=回退 next_state
    body_slice: tuple[int, int] | None = None  # 从 body_state 切片时给出

    @property
    def desc(self) -> str:
        return f"{self.name}[{self.dim}]"


@dataclass
class CollectSchema:
    """一次采集会话的完整布局描述（episode 开始时冻结）。"""

    arms: list[str] = field(default_factory=lambda: ["left", "right"])
    end_effector_left: str = EE_GRIPPER
    end_effector_right: str = EE_GRIPPER
    include_waist: bool = False
    include_head: bool = False
    # 列表第一个 = 对齐参考相机（默认 d435i 主视角，腕部随后）
    cameras: list[str] = field(
        default_factory=lambda: ["d435i", "wrist_left", "wrist_right"]
    )
    dataset_fps: int = 30
    action_source: str = ACTION_NEXT_STATE
    hold_frames: int = 10
    max_gap_ms: float = 100.0
    jpeg_quality: int = 90

    def __post_init__(self) -> None:
        # arms：启用的臂侧，["left"] / ["right"] / ["left", "right"]（默认双臂）
        self.arms = [str(s).strip().lower() for s in self.arms]
        if not self.arms or any(s not in SIDES for s in self.arms):
            raise ValueError(f"arms={self.arms!r} 无效，可选子集 {SIDES}（至少一侧）")
        if len(set(self.arms)) != len(self.arms):
            raise ValueError(f"arms={self.arms!r} 有重复侧")
        for side in SIDES:
            ee = self.end_effector_for(side)
            if side in self.arms and ee not in EE_TYPES:
                raise ValueError(f"end_effector_{side}={ee!r} 无效，可选 {EE_TYPES}")
        if self.action_source not in ACTION_SOURCES:
            raise ValueError(f"action_source={self.action_source!r} 无效，可选 {ACTION_SOURCES}")
        if not self.cameras:
            raise ValueError("cameras 不能为空（对齐以相机为参考时钟）")
        if isinstance(self.dataset_fps, bool) or not isinstance(self.dataset_fps, int) or self.dataset_fps < 1:
            raise ValueError("dataset_fps 必须为正整数")
        if len(set(self.cameras)) != len(self.cameras) or any(
                not isinstance(c, str) or not c or "/" in c or c in (".", "..") for c in self.cameras):
            raise ValueError("cameras must be unique nonempty labels without path separators")
        import math
        if not math.isfinite(self.max_gap_ms) or self.max_gap_ms <= 0:
            raise ValueError("max_gap_ms must be finite and positive")
        if not isinstance(self.hold_frames, int) or self.hold_frames < 0:
            raise ValueError("hold_frames must be a nonnegative integer")

    def end_effector_for(self, side: str) -> str:
        return self.end_effector_left if side == "left" else self.end_effector_right

    # -- 布局 ----------------------------------------------------------------

    def state_blocks(self) -> list[StateBlock]:
        """state 向量的有序分段（顺序即训练侧观测向量顺序）。

        仅纳入 arms 启用的侧；末端块跟随所属臂（臂未启用则该侧 ee 不录）。
        """
        blocks: list[StateBlock] = []
        for side in self.arms:
            blocks.append(StateBlock(
                f"{side}_arm", ARM_DIM, f"{side}_arm_state", f"{side}_arm_cmd",
            ))
        for side in self.arms:
            ee = self.end_effector_for(side)
            if ee == EE_GRIPPER:
                # 夹爪无反馈：state 用最近一次指令闭合比（0=开 1=合），
                # 与 pi0.5 数据约定（dim6 gripper [0,1]）一致。
                blocks.append(StateBlock(
                    f"{side}_ee", GRIPPER_DIM,
                    f"{side}_gripper_ratio", f"{side}_gripper_ratio",
                ))
            elif ee == EE_WUJI:
                blocks.append(StateBlock(
                    f"{side}_ee", HAND_DIM,
                    f"{side}_hand_state", f"{side}_hand_cmd",
                ))
        if self.include_waist:
            blocks.append(StateBlock(
                "waist", WAIST_DIM, "body_state", None, body_slice=BODY_WAIST_SLICE,
            ))
        if self.include_head:
            blocks.append(StateBlock(
                "head", HEAD_DIM, "body_state", None, body_slice=BODY_HEAD_SLICE,
            ))
        return blocks

    @property
    def state_dim(self) -> int:
        return sum(b.dim for b in self.state_blocks())

    @property
    def action_dim(self) -> int:
        # action 与 state 同布局（next_state）或同维（command 模式逐块同维）
        return self.state_dim

    def action_block_stream(self, block: StateBlock) -> str:
        """command 模式：块对应的指令流；无指令流（腰/头）回退 state 流。

        腰/头在采集层面没有独立指令话题（经 /robot_target 内部合成），
        command 模式下这两块实际仍是 next_state 语义 —— README 明示。
        """
        return block.cmd_stream or block.state_stream

    def state_names(self) -> list[str]:
        """每维一个名字，写入 LeRobot features names。"""
        names: list[str] = []
        for b in self.state_blocks():
            names.extend(f"{b.name}_{i}" for i in range(b.dim))
        return names

    # -- 需要订阅的流 ---------------------------------------------------------

    def required_streams(self) -> dict[str, int]:
        """本 schema 需要录制的全部流：{流名: 维度}。

        按 arms 录各臂 state+cmd；按末端类型录 hand/gripper；include_waist/head
        时录 body_state（真机 18 维全身反馈）；head_state/head_cmd 作为
        无 body_state（如仿真）时 head 块的回退源也一并订阅。
        """
        streams: dict[str, int] = {}
        for side in self.arms:
            streams[f"{side}_arm_state"] = ARM_DIM
            streams[f"{side}_arm_cmd"] = ARM_DIM
        for side in self.arms:
            ee = self.end_effector_for(side)
            if ee == EE_GRIPPER:
                streams[f"{side}_gripper_ratio"] = GRIPPER_DIM
                streams[f"{side}_gripper_rad"] = GRIPPER_DIM
            elif ee == EE_WUJI:
                streams[f"{side}_hand_state"] = HAND_DIM
                streams[f"{side}_hand_cmd"] = HAND_DIM
        if self.include_waist or self.include_head:
            streams["body_state"] = BODY_STATE_DIM
        if self.include_head:
            streams["head_state"] = HEAD_DIM
            streams["head_cmd"] = HEAD_DIM
        return streams

    def block_state_source(self, block: StateBlock) -> tuple[str, tuple[int, int] | None]:
        """块 state 的实际来源：(流名, 切片)。

        head/waist 优先 body_state 切片；body_state 缺席时 head 回退
        head_state 流（由 align 在运行时按可用性选择）。
        """
        if block.name == "head" and block.body_slice is not None:
            return ("body_state", block.body_slice)
        if block.body_slice is not None:
            return ("body_state", block.body_slice)
        return (block.state_stream, None)

    # -- 序列化 ----------------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["state_dim"] = self.state_dim
        d["state_blocks"] = [
            {"name": b.name, "dim": b.dim} for b in self.state_blocks()
        ]
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "CollectSchema":
        known = {
            "arms", "end_effector_left", "end_effector_right", "include_waist",
            "include_head", "cameras", "dataset_fps", "action_source",
            "hold_frames", "max_gap_ms", "jpeg_quality",
        }
        return cls(**{k: v for k, v in d.items() if k in known})

    @classmethod
    def from_json(cls, s: str) -> "CollectSchema":
        return cls.from_dict(json.loads(s))


def _canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


@dataclass(frozen=True)
class StreamSpec:
    """One explicitly typed input/output stream in a frozen MORLA schema."""

    stream_key: str
    topic: str
    message_type: str
    shape: tuple[int | None, ...]
    dtype: str
    unit: str
    qos: str
    timestamp_mode: str
    required: bool
    source_kind: str
    joint_names: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.stream_key or not self.topic.startswith("/") or not self.message_type:
            raise ValueError("stream_key, absolute topic and message_type are required")
        if not self.shape or any(d is not None and (isinstance(d, bool) or d < 1) for d in self.shape):
            raise ValueError(f"invalid shape for stream {self.stream_key!r}: {self.shape!r}")
        if self.joint_names and len(self.joint_names) != self.shape[-1]:
            raise ValueError(f"joint_names dimension mismatch for stream {self.stream_key!r}")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError(f"duplicate joint names in stream {self.stream_key!r}")
        if not self.dtype or not self.unit or not self.qos or not self.timestamp_mode or not self.source_kind:
            raise ValueError(f"stream {self.stream_key!r} is missing a data contract field")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "StreamSpec":
        allowed = {"stream_key", "topic", "message_type", "shape", "dtype", "unit", "qos",
                   "timestamp_mode", "required", "source_kind", "joint_names"}
        extra = set(value) - allowed
        missing = allowed - {"joint_names"} - set(value)
        if extra or missing:
            raise ValueError(f"stream fields invalid; missing={sorted(missing)}, unknown={sorted(extra)}")
        return cls(
            stream_key=str(value["stream_key"]), topic=str(value["topic"]),
            message_type=str(value["message_type"]), shape=tuple(value["shape"]),
            dtype=str(value["dtype"]), unit=str(value["unit"]), qos=str(value["qos"]),
            timestamp_mode=str(value["timestamp_mode"]), required=_strict_bool(value["required"]),
            source_kind=str(value["source_kind"]),
            joint_names=tuple(value.get("joint_names", ())),
        )

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["shape"] = list(self.shape)
        value["joint_names"] = list(self.joint_names)
        return value


@dataclass(frozen=True)
class StateBlockSpec:
    """A named, ordered state-vector range sourced from one stream."""

    block_id: str
    stream_key: str
    dim: int
    feature_names: tuple[str, ...]
    unit: str
    state_indices: tuple[int, ...]
    physical_joint_names: tuple[str, ...] = ()
    limits: tuple[tuple[float, float], ...] = ()
    stream_indices: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        _validate_block(self.block_id, self.dim, self.feature_names, self.state_indices,
                        self.physical_joint_names, self.limits, "state")
        if len(self.stream_indices) != self.dim or any(isinstance(i, bool) or i < 0 for i in self.stream_indices):
            raise ValueError(f"state block {self.block_id!r} stream_indices must match dim")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "StateBlockSpec":
        allowed = {"block_id", "stream_key", "dim", "feature_names", "unit", "state_indices",
                   "physical_joint_names", "limits", "stream_indices"}
        return cls(
            block_id=str(value["block_id"]), stream_key=str(value["stream_key"]),
            dim=int(value["dim"]), feature_names=tuple(value["feature_names"]),
            unit=str(value["unit"]), state_indices=tuple(value["state_indices"]),
            physical_joint_names=tuple(value.get("physical_joint_names", ())),
            limits=_limits_tuple(value.get("limits", ())),
            stream_indices=tuple(value.get("stream_indices", range(int(value["dim"])))),
        ) if not (set(value) - allowed) and {"block_id", "stream_key", "dim", "feature_names", "unit", "state_indices"} <= set(value) else _bad_fields("state block", value, allowed)

    def to_dict(self) -> dict[str, Any]:
        return _block_dict(self, "state_indices")


@dataclass(frozen=True)
class ActionBlockSpec:
    """A named model action range and the writer that can apply it."""

    block_id: str
    dim: int
    feature_names: tuple[str, ...]
    unit: str
    action_indices: tuple[int, ...]
    semantics: str
    writable: bool
    command_stream: str | None
    writer_id: str | None
    limits: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        _validate_block(self.block_id, self.dim, self.feature_names, self.action_indices,
                        (), self.limits, "action")
        if self.semantics not in {"absolute_position", "next_state", "command", "delta"}:
            raise ValueError(f"unsupported action semantics {self.semantics!r}")
        if self.writable and (not self.command_stream or not self.writer_id):
            raise ValueError(f"writable action block {self.block_id!r} needs command_stream and writer_id")
        if not self.writable and self.writer_id is not None:
            raise ValueError(f"read-only action block {self.block_id!r} cannot declare a writer")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ActionBlockSpec":
        allowed = {"block_id", "dim", "feature_names", "unit", "action_indices", "semantics",
                   "writable", "command_stream", "writer_id", "limits"}
        required = {"block_id", "dim", "feature_names", "unit", "action_indices", "semantics", "writable"}
        if set(value) - allowed or not required <= set(value):
            return _bad_fields("action block", value, allowed, required)
        return cls(
            block_id=str(value["block_id"]), dim=int(value["dim"]),
            feature_names=tuple(value["feature_names"]), unit=str(value["unit"]),
            action_indices=tuple(value["action_indices"]), semantics=str(value["semantics"]),
            writable=_strict_bool(value["writable"]), command_stream=value.get("command_stream"),
            writer_id=value.get("writer_id"), limits=_limits_tuple(value.get("limits", ())),
        )

    def to_dict(self) -> dict[str, Any]:
        return _block_dict(self, "action_indices") | {
            "semantics": self.semantics, "writable": self.writable,
            "command_stream": self.command_stream, "writer_id": self.writer_id,
        }


@dataclass(frozen=True)
class FrozenSchema:
    """Versioned, hashable schema shared by collection, policy and replay."""

    profile_id: str
    robot_id: str
    streams: tuple[StreamSpec, ...]
    state_blocks: tuple[StateBlockSpec, ...]
    action_blocks: tuple[ActionBlockSpec, ...]
    cameras: tuple[str, ...]
    dataset_fps: int
    action_source: str
    hold_frames: int = 10
    max_gap_ms: float = 100.0
    schema_version: int = 2
    profile_version: int = 1
    sensor_specs: tuple[dict[str, Any], ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    @property
    def state_dim(self) -> int:
        return max((max(block.state_indices) for block in self.state_blocks if block.state_indices), default=-1) + 1

    @property
    def action_dim(self) -> int:
        return max((max(block.action_indices) for block in self.action_blocks if block.action_indices), default=-1) + 1

    def validate(self) -> None:
        if self.schema_version != 2:
            raise ValueError(f"unsupported frozen schema version {self.schema_version}")
        if not self.profile_id or not self.robot_id:
            raise ValueError("profile_id and robot_id are required")
        if isinstance(self.dataset_fps, bool) or self.dataset_fps < 1:
            raise ValueError("dataset_fps must be a positive integer")
        if self.action_source not in ACTION_SOURCES:
            raise ValueError(f"action_source must be one of {ACTION_SOURCES}")
        if isinstance(self.hold_frames, bool) or self.hold_frames < 0:
            raise ValueError("hold_frames must be a nonnegative integer")
        if not math.isfinite(self.max_gap_ms) or self.max_gap_ms <= 0:
            raise ValueError("max_gap_ms must be finite and positive")
        if not self.cameras or any(not _valid_label(c) for c in self.cameras):
            raise ValueError("cameras must contain unique safe labels")
        if len(set(self.cameras)) != len(self.cameras):
            raise ValueError("camera labels must be unique")
        stream_keys = [stream.stream_key for stream in self.streams]
        if len(set(stream_keys)) != len(stream_keys):
            raise ValueError("stream_key values must be unique")
        topics = [(stream.topic, stream.message_type) for stream in self.streams]
        if len(set(topics)) != len(topics):
            raise ValueError("the same topic/message type cannot be declared twice")
        stream_by_key = set(stream_keys)
        for block in self.state_blocks:
            if block.stream_key not in stream_by_key:
                raise ValueError(f"state block {block.block_id!r} references unknown stream {block.stream_key!r}")
            stream = next(s for s in self.streams if s.stream_key == block.stream_key)
            if len(block.stream_indices) != block.dim or any(i < 0 or i >= stream.shape[-1] for i in block.stream_indices):
                raise ValueError(f"state block {block.block_id!r} stream_indices do not fit {block.stream_key!r}")
        for block in self.action_blocks:
            if block.writable and block.command_stream not in stream_by_key:
                raise ValueError(f"action block {block.block_id!r} references unknown command stream")
        _validate_coverage([i for b in self.state_blocks for i in b.state_indices], "state")
        _validate_coverage([i for b in self.action_blocks for i in b.action_indices], "action")
        state_ids = [b.block_id for b in self.state_blocks]
        action_ids = [b.block_id for b in self.action_blocks]
        if len(set(state_ids)) != len(state_ids) or len(set(action_ids)) != len(action_ids):
            raise ValueError("block IDs must be unique within state/action layouts")

    def layout_dict(self) -> dict[str, Any]:
        return {
            "profile_id": self.profile_id, "robot_id": self.robot_id,
            "streams": [s.to_dict() for s in self.streams],
            "state_blocks": [b.to_dict() for b in self.state_blocks],
            "action_blocks": [b.to_dict() for b in self.action_blocks],
            "cameras": list(self.cameras),
        }

    @property
    def layout_hash(self) -> str:
        return hashlib.sha256(_canonical_json(self.layout_dict()).encode("utf-8")).hexdigest()

    @property
    def schema_hash(self) -> str:
        value = self.to_dict(include_hashes=False)
        value.pop("metadata", None)
        return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()

    def to_dict(self, include_hashes: bool = True) -> dict[str, Any]:
        value = {
            "schema_version": self.schema_version, "profile_version": self.profile_version,
            "profile_id": self.profile_id, "robot_id": self.robot_id,
            "streams": [s.to_dict() for s in self.streams],
            "state_blocks": [b.to_dict() for b in self.state_blocks],
            "action_blocks": [b.to_dict() for b in self.action_blocks],
            "cameras": list(self.cameras), "dataset_fps": self.dataset_fps,
            "action_source": self.action_source, "hold_frames": self.hold_frames,
            "max_gap_ms": self.max_gap_ms, "sensor_specs": list(self.sensor_specs),
            "metadata": dict(self.metadata), "state_dim": self.state_dim,
            "action_dim": self.action_dim,
        }
        if include_hashes:
            value["schema_hash"] = self.schema_hash
            value["layout_hash"] = self.layout_hash
        return value

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2, sort_keys=True)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "FrozenSchema":
        allowed = {"schema_version", "profile_version", "profile_id", "robot_id", "streams",
                   "state_blocks", "action_blocks", "cameras", "dataset_fps", "action_source",
                   "hold_frames", "max_gap_ms", "sensor_specs", "metadata", "state_dim", "action_dim",
                   "schema_hash", "layout_hash"}
        required = {"schema_version", "profile_version", "profile_id", "robot_id", "streams",
                    "state_blocks", "action_blocks", "cameras", "dataset_fps", "action_source"}
        if set(value) - allowed or not required <= set(value):
            return _bad_fields("frozen schema", value, allowed, required)
        if value["schema_version"] != 2:
            raise ValueError(f"unsupported schema_version {value['schema_version']!r}")
        schema = cls(
            schema_version=value["schema_version"], profile_version=value["profile_version"],
            profile_id=str(value["profile_id"]), robot_id=str(value["robot_id"]),
            streams=tuple(StreamSpec.from_dict(x) for x in value["streams"]),
            state_blocks=tuple(StateBlockSpec.from_dict(x) for x in value["state_blocks"]),
            action_blocks=tuple(ActionBlockSpec.from_dict(x) for x in value["action_blocks"]),
            cameras=tuple(value["cameras"]), dataset_fps=value["dataset_fps"],
            action_source=str(value["action_source"]), hold_frames=value.get("hold_frames", 10),
            max_gap_ms=value.get("max_gap_ms", 100.0),
            sensor_specs=tuple(value.get("sensor_specs", ())), metadata=dict(value.get("metadata", {})),
        )
        for key, actual in (("state_dim", schema.state_dim), ("action_dim", schema.action_dim)):
            if key in value and value[key] != actual:
                raise ValueError(f"{key} mismatch: stored={value[key]} derived={actual}")
        if "layout_hash" in value and value["layout_hash"] != schema.layout_hash:
            raise ValueError("layout_hash does not match frozen layout")
        if "schema_hash" in value and value["schema_hash"] != schema.schema_hash:
            raise ValueError("schema_hash does not match frozen schema")
        return schema


def upgrade_v1_schema(value: dict[str, Any], profile_id: str | None = None) -> FrozenSchema:
    """Explicitly upgrade an Astral ``CollectSchema``/meta layout to FrozenSchema v2."""
    if not isinstance(value, dict):
        raise TypeError("v1 schema must be a mapping")
    version = value.get("schema_version", value.get("version", 1))
    if version not in (1, "1", None):
        raise ValueError(f"upgrade_v1_schema accepts only v1 input, got version {version!r}")
    legacy_fields = {"arms", "end_effector_left", "end_effector_right", "include_waist",
                     "include_head", "cameras", "dataset_fps", "action_source", "hold_frames",
                     "max_gap_ms", "jpeg_quality"}
    schema = CollectSchema.from_dict({k: v for k, v in value.items() if k in legacy_fields})
    supplied_dim = value.get("state_dim")
    if supplied_dim is not None and supplied_dim != schema.state_dim:
        raise ValueError(f"v1 state_dim mismatch: stored={supplied_dim}, derived={schema.state_dim}")
    profile = profile_id or value.get("profile_id") or "legacy_astral"
    streams: list[StreamSpec] = []
    stream_joint_names: dict[str, tuple[str, ...]] = {}
    for key, dim in schema.required_streams().items():
        topic, _ = STREAM_TOPICS[key]
        is_ratio = key.endswith("_ratio")
        names = _legacy_joint_names(key, dim)
        stream_joint_names[key] = names
        streams.append(StreamSpec(
            stream_key=key, topic=topic,
            message_type="std_msgs/msg/Float64" if is_ratio else "sensor_msgs/msg/JointState",
            shape=(dim,), dtype="float32", unit="ratio" if is_ratio else "rad",
            qos="sensor_data", timestamp_mode="arrival" if key.endswith("_cmd") or is_ratio else "source_header",
            required=True, source_kind="command_echo" if is_ratio else ("command" if key.endswith("_cmd") else "state"),
            joint_names=names,
        ))
    for camera in schema.cameras:
        streams.append(StreamSpec(
            stream_key=f"camera:{camera}", topic=f"/quest3_video_streamer/collect/{camera}",
            message_type="sensor_msgs/msg/CompressedImage", shape=(None, None, 3),
            dtype="uint8", unit="rgb", qos="sensor_data", timestamp_mode="source_header",
            required=True, source_kind="camera",
        ))
    state_blocks: list[StateBlockSpec] = []
    action_blocks: list[ActionBlockSpec] = []
    cursor = 0
    feature_names = schema.state_names()
    for block in schema.state_blocks():
        start = cursor
        names = tuple(feature_names[start:start + block.dim])
        cursor += block.dim
        source, _ = schema.block_state_source(block)
        physical_names = _legacy_joint_names(source, block.dim)
        stream_indices = tuple(range(*block.body_slice)) if block.body_slice else tuple(range(block.dim))
        state_blocks.append(StateBlockSpec(
            block_id=block.name, stream_key=source, dim=block.dim,
            feature_names=names, unit="ratio" if block.dim == 1 else "rad",
            state_indices=tuple(range(start, cursor)), physical_joint_names=physical_names,
            stream_indices=stream_indices,
        ))
        cmd_stream = schema.action_block_stream(block) if block.cmd_stream else None
        writer = _legacy_writer_id(block.name) if cmd_stream else None
        action_blocks.append(ActionBlockSpec(
            block_id=block.name, dim=block.dim, feature_names=names,
            unit="ratio" if block.dim == 1 else "rad",
            action_indices=tuple(range(start, cursor)),
            semantics="next_state" if schema.action_source == ACTION_NEXT_STATE else "command",
            writable=cmd_stream is not None, command_stream=cmd_stream, writer_id=writer,
        ))
    known = legacy_fields | {"schema_version", "version", "profile_id", "state_dim", "state_blocks"}
    metadata = {"upgraded_from": "CollectSchema-v1", "legacy_metadata": {k: v for k, v in value.items() if k not in known}}
    return FrozenSchema(
        profile_id=profile, robot_id=str(value.get("robot_id", "astral")),
        streams=tuple(streams), state_blocks=tuple(state_blocks), action_blocks=tuple(action_blocks),
        cameras=tuple(schema.cameras), dataset_fps=schema.dataset_fps,
        action_source=schema.action_source, hold_frames=schema.hold_frames,
        max_gap_ms=schema.max_gap_ms, metadata=metadata,
    )


def _strict_bool(value: Any) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"expected boolean, got {value!r}")
    return value


def _valid_label(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip()) and "/" not in value and value not in {".", ".."}


def _limits_tuple(value: Any) -> tuple[tuple[float, float], ...]:
    if any(not isinstance(pair, (list, tuple)) or len(pair) != 2 for pair in value):
        raise ValueError("limits must be finite [min,max] pairs")
    result = tuple((float(pair[0]), float(pair[1])) for pair in value)
    if any(len(pair) != 2 or not all(math.isfinite(x) for x in pair) or pair[0] > pair[1] for pair in result):
        raise ValueError("limits must be finite [min,max] pairs")
    return result


def _validate_block(block_id: str, dim: int, feature_names: tuple[str, ...], indices: tuple[int, ...],
                    physical_names: tuple[str, ...], limits: tuple[tuple[float, float], ...], kind: str) -> None:
    if not block_id or isinstance(dim, bool) or dim < 1:
        raise ValueError(f"{kind} block needs a nonempty id and positive dim")
    if len(feature_names) != dim or len(set(feature_names)) != dim or any(not n for n in feature_names):
        raise ValueError(f"{kind} block {block_id!r} feature_names must be unique and match dim")
    if len(indices) != dim or any(isinstance(i, bool) or i < 0 for i in indices) or len(set(indices)) != dim:
        raise ValueError(f"{kind} block {block_id!r} indices must be unique and match dim")
    if physical_names and (len(physical_names) != dim or len(set(physical_names)) != dim):
        raise ValueError(f"{kind} block {block_id!r} physical names must match dim when specified")
    if limits and len(limits) != dim:
        raise ValueError(f"{kind} block {block_id!r} limits must match dim when specified")


def _validate_coverage(indices: list[int], kind: str) -> None:
    if not indices:
        return
    if len(set(indices)) != len(indices) or set(indices) != set(range(max(indices) + 1)):
        raise ValueError(f"{kind} indices must cover one contiguous zero-based range")


def _block_dict(block: Any, index_field: str) -> dict[str, Any]:
    result = {"block_id": block.block_id, "dim": block.dim,
              "feature_names": list(block.feature_names), "unit": block.unit,
              index_field: list(getattr(block, index_field)),
              "limits": [list(pair) for pair in block.limits]}
    if hasattr(block, "stream_key"):
        result["stream_key"] = block.stream_key
        result["physical_joint_names"] = list(block.physical_joint_names)
        result["stream_indices"] = list(block.stream_indices)
    return result


def _bad_fields(label: str, value: dict[str, Any], allowed: set[str], required: set[str] | None = None):
    required = required or set()
    missing = required - set(value)
    unknown = set(value) - allowed
    raise ValueError(f"{label} fields invalid; missing={sorted(missing)}, unknown={sorted(unknown)}")


def _legacy_joint_names(key: str, dim: int) -> tuple[str, ...]:
    if "_arm" in key and dim == ARM_DIM:
        side = key.split("_", 1)[0]
        if side == "left":
            return (
                "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_roll",
                "left_elbow_pitch", "left_forearm_roll", "left_wrist_pitch", "left_wrist_roll",
            )
        if side == "right":
            return (
                "right_shoulder_pitch", "right_shoulder_roll", "right_elbow_roll",
                "right_elbow_pitch", "right_forearm_roll", "right_wrist_pitch", "right_wrist_roll",
            )
    if "_gripper" in key and dim == GRIPPER_DIM:
        side = key.split("_", 1)[0]
        return (f"{side}_gripper_joint",)
    if key == "head_state" and dim == HEAD_DIM:
        return ("head_yaw", "head_pitch")
    if key == "body_state" and dim == BODY_STATE_DIM:
        return (
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_roll", "left_elbow_pitch",
            "left_forearm_roll", "left_wrist_pitch", "left_wrist_roll",
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow_roll", "right_elbow_pitch",
            "right_forearm_roll", "right_wrist_pitch", "right_wrist_roll",
            "waist_front", "waist_side", "head_yaw", "head_pitch",
        )
    # Historical 20-DoF hand streams did not freeze physical joint names.
    return ()


def _legacy_writer_id(block_id: str) -> str:
    if block_id.endswith("_arm"):
        return f"{block_id}_driver"
    if block_id.endswith("_ee"):
        return f"{block_id}_writer"
    return f"{block_id}_writer"
