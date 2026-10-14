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
