"""Schema-driven observation/action layout for policy inference & replay.

The single source of truth is ``astral_data_collect.schema.CollectSchema``:
the same layout that teleop data collection froze into ``meta.json``. Feeding
that schema here yields:

* the ROS topics to observe (``obs_sources``) in state-vector order,
* ``assemble_state``: per-tick obs vector (``None`` when any live block stale),
* ``split_action``: an absolute action vector into per-stream command targets
  consumed by :mod:`executor`.

So switching robot config (right arm / gripper→wuji / waist/head / camera
labels) requires editing only the same yaml used at collection time — nothing
else in this package changes.
"""

from __future__ import annotations

import dataclasses

import cv2
import numpy as np

from astral_data_collect.schema import (
    EE_GRIPPER,
    EE_WUJI,
    ARM_DIM,
    HAND_DIM,
    HEAD_DIM,
    WAIST_DIM,
    CollectSchema,
    STREAM_TOPICS,
)

# stream keys whose values we can obtain at inference time as *state feedback*.
_OBS_STATE_STREAMS = {
    "left_arm_state",
    "right_arm_state",
    "left_hand_state",
    "right_hand_state",
    "left_gripper_ratio",
    "right_gripper_ratio",
    "body_state",
    "head_state",
}

# blocks that do NOT have a standalone command topic we can write (waist is
# only reachable via the 18-dim /astral/joint_commands full-body stream, which
# v1 does not write to avoid clobbering arm topics).
_WRITABLE_BLOCK_PREFIXES = ("arm",)  # handled via side-specific arm topics


class UnsupportedActionBlockError(RuntimeError):
    """A block has no v1 command channel (e.g. waist); refusing to guess."""


def topic_for(stream_key: str) -> str:
    topic, _ = STREAM_TOPICS[stream_key]
    return topic


def dim_for(stream_key: str) -> int:
    _, dim = STREAM_TOPICS[stream_key]
    return dim


@dataclasses.dataclass(frozen=True)
class ObsSource:
    """One live ROS subscription feeding a slice of the obs vector."""

    key: str            # stream key (matches schema.stream names / HDF5 group)
    topic: str          # ROS topic to subscribe
    dim: int            # vector length published on that topic
    kind: str           # 'joints' | 'ratio' | 'body' | 'hand'

    def __post_init__(self):
        if self.key not in _OBS_STATE_STREAMS:
            raise ValueError(f"{self.key} is not an observable state stream")


@dataclasses.dataclass(frozen=True)
class CmdTarget:
    """One command a robot-side writer must emit for a state/action block."""

    stream: str   # 'left_arm_cmd' etc.
    topic: str
    kind: str     # 'joints' -> JointState position; 'ratio' -> Float64 ratio
    values: np.ndarray  # joints: (7,) rad ; ratio: (1,) in [0,1]

    @property
    def name(self) -> str:
        return self.stream

    def __str__(self) -> str:
        vals = np.round(self.values, 4)
        return f"{self.stream}@{self.topic} {vals.tolist()}"


class ObsLayout:
    """Topics + assembly rules derived once from a frozen CollectSchema."""

    def __init__(self, schema: CollectSchema):
        self.schema = schema
        self.blocks = schema.state_blocks()
        self.state_dim = schema.state_dim
        # stream sources we must keep fresh, in stable key order.
        sources: dict[str, ObsSource] = {}
        for side in schema.arms:
            sources[f"{side}_arm_state"] = ObsSource(
                f"{side}_arm_state",
                topic_for(f"{side}_arm_state"),
                ARM_DIM,
                "joints",
            )
        for side in schema.arms:
            ee = schema.end_effector_for(side)
            if ee == EE_GRIPPER:
                sources[f"{side}_gripper_ratio"] = ObsSource(
                    f"{side}_gripper_ratio",
                    topic_for(f"{side}_gripper_ratio"),
                    1,
                    "ratio",
                )
            elif ee == EE_WUJI:
                sources[f"{side}_hand_state"] = ObsSource(
                    f"{side}_hand_state",
                    topic_for(f"{side}_hand_state"),
                    HAND_DIM,
                    "joints",
                )
        if schema.include_waist or schema.include_head:
            sources["body_state"] = ObsSource(
                "body_state", topic_for("body_state"), dim_for("body_state"), "body"
            )
        if schema.include_head:
            # sim fallback when no 18-dim body_state publisher exists.
            sources.setdefault(
                "head_state",
                ObsSource(
                    "head_state", topic_for("head_state"), HEAD_DIM, "joints"
                ),
            )
        self.sources = sources

        # command topics reachable per block (for split_action).
        self._cmd_topic: dict[str, str] = {}
        for side in schema.arms:
            self._cmd_topic[f"{side}_arm"] = topic_for(f"{side}_arm_cmd")
        for side in schema.arms:
            ee = schema.end_effector_for(side)
            if ee == EE_GRIPPER:
                self._cmd_topic[f"{side}_ee"] = topic_for(f"{side}_gripper_ratio")
            elif ee == EE_WUJI:
                self._cmd_topic[f"{side}_ee"] = topic_for(f"{side}_hand_cmd")
        if schema.include_head:
            self._cmd_topic["head"] = topic_for("head_cmd")

    # -- obs assembly ------------------------------------------------------

    def assemble_state(
        self, vectors: dict[str, np.ndarray | None]
    ) -> tuple[np.ndarray | None, list[str]]:
        """Return ``(state_vector, missing_keys)``.

        Missing = stale/never-received source required by the schema. Waist/
        head prefer the body_state slice and fall back to head_state, mirroring
        align_data's availability logic. Returns (None, missing) if any
        required block cannot be filled (caller decides whether that is fatal).
        """
        parts: list[np.ndarray] = []
        missing: list[str] = []
        body: np.ndarray | None = vectors.get("body_state")
        for b in self.blocks:
            if b.name.endswith("_arm"):
                src = f"{b.name}_state"
                v = vectors.get(src)
                if v is None:
                    missing.append(src)
                    continue
                parts.append(np.asarray(v, dtype=np.float32)[: b.dim])
            elif b.name.endswith("_ee"):
                side = b.name.split("_")[0]
                ee = self.schema.end_effector_for(side)
                if ee == EE_GRIPPER:
                    src = f"{side}_gripper_ratio"
                    v = vectors.get(src)
                    if v is None:
                        missing.append(src)
                        continue
                    parts.append(np.asarray(v, dtype=np.float32).reshape(1))
                elif ee == EE_WUJI:
                    src = f"{side}_hand_state"
                    v = vectors.get(src)
                    if v is None:
                        missing.append(src)
                        continue
                    parts.append(np.asarray(v, dtype=np.float32)[: b.dim])
            elif b.name == "waist":
                if body is not None:
                    # body_slice is a (start, stop) pair -> a real slice on the
                    # 1-D body vector (waist 14:16 / head 16:18).
                    parts.append(
                        np.asarray(body, dtype=np.float32)[slice(*b.body_slice)]
                    )
                else:
                    missing.append("body_state")
            elif b.name == "head":
                if body is not None:
                    parts.append(
                        np.asarray(body, dtype=np.float32)[slice(*b.body_slice)]
                    )
                elif vectors.get("head_state") is not None:
                    parts.append(
                        np.asarray(vectors["head_state"], dtype=np.float32)[: b.dim]
                    )
                else:
                    missing.append("body_state|head_state")
            else:
                missing.append(b.name)
        if missing:
            return None, missing
        try:
            state = np.concatenate(parts)
        except ValueError as exc:
            raise RuntimeError(
                f"obs assembly failed for schema blocks "
                f"{[b.name for b in self.blocks]}: {exc}"
            ) from exc
        if state.shape[0] != self.state_dim:
            raise RuntimeError(
                f"obs dim mismatch: assembled {state.shape[0]} != schema "
                f"{self.state_dim}"
            )
        return state, []

    # -- action split --------------------------------------------------------

    def split_action(self, action: np.ndarray) -> list[CmdTarget]:
        """Split one absolute action row into per-stream command targets.

        Waist blocks raise ``UnsupportedActionBlockError``; everything else
        maps to the standalone topics used by the arm driver / sim.
        """
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape[0] != self.state_dim:
            raise ValueError(
                f"action dim {action.shape[0]} != schema state_dim "
                f"{self.state_dim}"
            )
        targets: list[CmdTarget] = []
        cursor = 0
        for b in self.blocks:
            seg = action[cursor : cursor + b.dim]
            cursor += b.dim
            if b.name.endswith("_arm"):
                targets.append(
                    CmdTarget(
                        f"{b.name}_cmd",
                        self._cmd_topic[b.name],
                        "joints",
                        seg.copy(),
                    )
                )
            elif b.name.endswith("_ee"):
                targets.append(
                    CmdTarget(
                        f"{b.name}_cmd",
                        self._cmd_topic[b.name],
                        "ratio" if seg.size == 1 else "joints",
                        seg.copy(),
                    )
                )
            elif b.name == "head":
                targets.append(
                    CmdTarget(
                        "head_cmd",
                        self._cmd_topic["head"],
                        "joints",
                        seg.copy(),
                    )
                )
            elif b.name == "waist":
                raise UnsupportedActionBlockError(
                    "waist block has no v1 command channel; refuse to guess"
                )
            else:
                raise UnsupportedActionBlockError(f"block {b.name} unwritable")
        return targets


# ----------------------------------------------------------------- image helpers


def decode_jpeg_rgb(jpeg: bytes) -> np.ndarray | None:
    """CompressedImage JPEG -> HxWx3 uint8 RGB (None on decode failure)."""
    buf = np.frombuffer(jpeg, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def letterbox(img: np.ndarray, size: int) -> np.ndarray:
    """Aspect-preserving resize + zero pad to ``size``x``size``.

    Mirrors the geometry used at conversion time (convert_to_lerobot._letterbox)
    and openpi's resize_with_pad: ratio=max(w,h)/size, symmetric borders with
    remainder pushed to bottom/right.
    """
    h, w = img.shape[:2]
    if (h, w) == (size, size):
        return img
    ratio = max(w / size, h / size)
    new_w, new_h = int(w / ratio), int(h / ratio)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    pad_h0 = (size - new_h) // 2
    pad_h1 = size - new_h - pad_h0
    pad_w0 = (size - new_w) // 2
    pad_w1 = size - new_w - pad_w0
    return cv2.copyMakeBorder(
        resized, pad_h0, pad_h1, pad_w0, pad_w1, cv2.BORDER_CONSTANT, value=0
    )


def split_camera_map(
    camera_map: dict[str, str], image_size: int
) -> list[tuple[str, str, int]]:
    """(slot, collect_label, image_size) rows for each mapped camera."""
    out = []
    for slot, label in camera_map.items():
        out.append((slot, label, int(image_size)))
    return out
