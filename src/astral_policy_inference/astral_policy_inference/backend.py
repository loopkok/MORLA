"""Policy backends — swap a checkpoint / server, nothing else changes.

``PolicyBackend`` protocol + concrete backends. ``backend_type`` names the
**transport** (remote / inproc / stub); the **model family** (act / pi05 / …)
is a separate ``model`` parameter consumed by :func:`make_backend` and
``serve.py``.

* :class:`RemoteBackend` — talks to a remote policy server (``scripts/serve.py``,
  model-agnostic websocket protocol) using the self-contained
  :mod:`astral_policy_inference.client`. The *server* owns the checkpoint, so
  changing models never touches the robot side.
* :class:`InprocBackend` — in-process local (non-remote) loader: resolves the
  concrete lerobot policy class from the checkpoint's own ``config.json``
  (``get_policy_class``) + its serialized pre/post processor pipeline.

Both emit **absolute** action rows (arm joint rad + gripper ratio 0..1) in the
dataset layout order, exactly what the driver/MuJoCo consumers take.

Heavy imports (websockets / torch / lerobot) happen lazily inside ``open()`` so
the module imports on machines that do not have them.
"""

from __future__ import annotations

import dataclasses
import json
import os
import sys
import time
from abc import ABC, abstractmethod
from typing import Callable

import numpy as np


class PolicyError(RuntimeError):
    """Raised when a backend cannot produce an action (network/policy/etc.)."""


@dataclasses.dataclass
class ObsBatch:
    """Neutral observation handed to any backend."""

    state: np.ndarray          # (state_dim,) unnormalized, dataset layout order
    images: dict[str, np.ndarray]  # logical key -> HxWx3 uint8
    prompt: str
    # 只供诊断使用的轻量元数据（时间戳、龄期、来源状态）；绝不传给模型。
    # default_factory 保持现有 backend/unit test 的构造方式兼容。
    metadata: dict = dataclasses.field(default_factory=dict)


class PolicyBackend(ABC):
    """Backend contract consumed by :mod:`engine`."""

    name: str = "backend"

    @abstractmethod
    def open(self) -> None:
        """Connect / load weights. May block for servers coming up."""

    @abstractmethod
    def close(self) -> None:
        """Release the connection / model."""

    @abstractmethod
    def infer(self, obs: ObsBatch) -> np.ndarray:
        """Return an absolute action chunk (>=1, action_dim); never None."""

    def infer_rtc(self, obs: ObsBatch, rtc: dict) -> np.ndarray:
        """Return a chunk conditioned on an RTC prefix, when supported."""
        raise PolicyError(f"{self.name} backend does not support guided RTC")

    @abstractmethod
    def reset(self) -> None:
        """Drop any queued actions / language context on the policy side."""


def _actions_from_response(
    response: dict, action_dim: int, max_rows: int
) -> np.ndarray:
    actions = np.asarray(response["actions"], dtype=np.float64)
    if actions.ndim == 1:  # single row → make (1, dim)
        actions = actions.reshape(1, -1)
    if actions.ndim != 2 or actions.shape[0] == 0 or actions.shape[1] != action_dim:
        raise PolicyError(
            f"server actions shape {actions.shape} != (rows>=1, {action_dim})"
        )
    if not np.isfinite(actions).all():
        raise PolicyError("server actions contain NaN or infinity")
    return actions[:max_rows]


class RemoteBackend(PolicyBackend):
    """Remote policy backend — websocket to ``scripts/serve.py`` (model-agnostic).

    Speaks the same protocol for any served model family (act / pi05 / …); the
    payload (``observation/state`` + ``observation/camera/<slot>`` + ``prompt``)
    is assembled here and the server does the inference.
    """

    name = "remote"

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 8000,
        action_dim: int,
        slot_keys: dict[str, str],  # camera_map: slot -> collect label
        state_key: str = "observation/state",
        image_prefix: str = "observation/camera",
        default_prompt: str = "",
        client_factory: Callable | None = None,  # test seam
        jpeg_transport: bool = False,
        jpeg_quality: int = 92,
        connect_timeout_s: float = 5.0,
        infer_timeout_s: float = 3.0,
        rtc_guided: bool = False,
        rtc_horizon: int = 10,
    ):
        self.host = host
        self.port = int(port)
        self.action_dim = int(action_dim)
        self.slot_keys = dict(slot_keys)
        self.state_key = state_key
        self.image_prefix = image_prefix
        self.default_prompt = default_prompt
        self._client_factory = client_factory
        self._client = None
        self.last_infer_s = 0.0
        self.last_server_timing: dict | None = None
        # 上行 JPEG：True 时 camera 槽位编码成 JPEG 字节（1.38MB→~0.2MB），
        # serve 端 decode_jpeg 还原。像素 = 节点 letterbox 后 RGB 再编码（有损）。
        self.jpeg_transport = bool(jpeg_transport)
        self.jpeg_quality = int(jpeg_quality)
        self.connect_timeout_s = float(connect_timeout_s)
        self.infer_timeout_s = float(infer_timeout_s)
        self.rtc_guided = bool(rtc_guided)
        self.rtc_horizon = int(rtc_horizon)

    def open(self) -> None:
        if self._client is not None:
            return
        if self._client_factory is not None:
            self._client = self._client_factory(self.host, self.port)
            return
        # 自包含 client（vendored __ndarray__ 序列化 + websockets），无需 openpi_client
        from astral_policy_inference.client import WebsocketClient

        try:
            client = WebsocketClient(
                self.host, self.port,
                connect_timeout_s=self.connect_timeout_s,
                infer_timeout_s=self.infer_timeout_s,
            )
        except Exception as exc:
            raise PolicyError(f"remote server connect failed: {exc}") from exc
        try:
            metadata = client.get_server_metadata()
            server_dim = metadata.get("action_dim")
            if server_dim is not None and int(server_dim) != self.action_dim:
                raise PolicyError(
                    f"server action_dim {server_dim} != robot action_dim {self.action_dim}"
                )
            if self.rtc_guided:
                if metadata.get("model") != "pi05" or not metadata.get("rtc_guided"):
                    raise PolicyError("guided RTC requires a pi05 server started with --rtc-guided")
                if int(metadata.get("rtc_horizon", 0)) != self.rtc_horizon:
                    raise PolicyError(
                        f"RTC horizon mismatch: node={self.rtc_horizon}, "
                        f"server={metadata.get('rtc_horizon')}"
                    )
        except (AttributeError, TypeError, ValueError, PolicyError) as exc:
            client.close()
            raise PolicyError(f"invalid server metadata: {exc}") from exc
        self._client = client

    def close(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except Exception:  # noqa: BLE001
                pass
        self._client = None

    def reset(self) -> None:
        if self._client is not None:
            try:
                self._client.reset()
            except Exception:  # noqa: BLE001
                pass

    def infer(self, obs: ObsBatch, *, rtc: dict | None = None) -> np.ndarray:
        if self._client is None:
            raise PolicyError("RemoteBackend not open()ed")
        payload: dict = {self.state_key: obs.state.astype(np.float32)}
        for slot, label in self.slot_keys.items():
            img = obs.images.get(label)
            if img is None:
                # AstralInputs zero-pads missing slots; server keeps image_mask.
                continue
            if self.jpeg_transport:
                from astral_policy_inference.image_codec import encode_jpeg

                payload[f"{self.image_prefix}/{slot}"] = encode_jpeg(
                    img, quality=self.jpeg_quality
                )
            else:
                payload[f"{self.image_prefix}/{slot}"] = np.asarray(
                    img, dtype=np.uint8
                )
        if self.jpeg_transport:
            payload["image_format"] = "jpeg"
        payload["prompt"] = obs.prompt or self.default_prompt
        if rtc is not None:
            if not self.rtc_guided:
                raise PolicyError("RTC request sent through a non-RTC backend")
            payload.update({
                "rtc_prefix": rtc["prefix"],
                "rtc_prefix_valid": int(rtc["valid"]),
                "rtc_delay": int(rtc["delay"]),
                "rtc_max_guidance_weight": float(rtc["max_guidance_weight"]),
                "rtc_schedule": str(rtc["schedule"]),
            })
        t0 = time.perf_counter()
        try:
            response = self._client.infer(payload)
        except Exception as exc:  # noqa: BLE001
            raise PolicyError(f"remote server infer failed: {exc}") from exc
        if "actions" not in (response or {}):
            raise PolicyError("remote server response missing 'actions' key")
        self.last_server_timing = (
            response.get("server_timing") if isinstance(response, dict) else None
        )
        if self.last_server_timing is None and isinstance(response, dict) and "server_ms" in response:
            self.last_server_timing = {"total_ms": float(response["server_ms"])}
        actions = _actions_from_response(response, self.action_dim, max_rows=1 << 20)
        self.last_infer_s = time.perf_counter() - t0
        return actions

    def infer_rtc(self, obs: ObsBatch, rtc: dict) -> np.ndarray:
        return self.infer(obs, rtc=rtc)


class InprocBackend(PolicyBackend):
    """Local (non-remote) in-process policy backend loaded from a checkpoint dir.

    Resolves the concrete lerobot policy class from the checkpoint's own
    ``config.json`` via ``get_policy_class`` (works for any lerobot policy
    family, not just ACT). ``infer()`` returns the **full action chunk**
    (``n_action_steps`` rows) via ``predict_action_chunk`` — the engine owns
    chunk pacing, so ``queue_async`` reaches the policy rate and the observation
    payload is uploaded once per chunk instead of once per action row.
    Checkpoints with a ``temporal_ensemble_coeff`` fall back to ``select_action``
    (single row, exponential weighting preserved). Requires lerobot+torch in
    this interpreter (py3.12).
    """

    name = "inproc"

    def __init__(
        self,
        *,
        checkpoint_dir: str,
        action_dim: int,
        image_keys: dict[str, str],  # feature label -> collect label (usually 1:1)
        device: str | None = None,
        default_prompt: str = "",
    ):
        self.checkpoint_dir = str(checkpoint_dir)
        self.action_dim = int(action_dim)
        self.image_keys = dict(image_keys)
        self.device = device
        self.default_prompt = default_prompt
        self._policy = None
        self._pre = None
        self._post = None
        self.last_infer_s = 0.0
        self.last_timing: dict[str, float] = {}  # prep/pre/infer/post/total ms

    def open(self) -> None:
        if self._policy is not None:
            return
        try:
            from lerobot.policies import (
                get_policy_class,
                make_pre_post_processors,
            )
        except ImportError as exc:  # pragma: no cover
            raise PolicyError(
                "lerobot (this fork, v0.6.x) not importable in this interpreter "
                f"(python {sys.version_info.major}.{sys.version_info.minor}); "
                "use backend_type=remote — start serve.py --model act in a py3.12 "
                "lerobot env and set host/port on this node"
            ) from exc
        try:
            # PreTrainedPolicy is the *abstract* base in lerobot 0.6.x; the
            # concrete policy class (ACTPolicy etc.) is resolved via the factory
            # from the checkpoint's own config.json "type". Directly calling
            # PreTrainedPolicy.from_pretrained cannot instantiate a real
            # checkpoint ("Can't instantiate abstract class").
            with open(
                os.path.join(self.checkpoint_dir, "config.json"), "r", encoding="utf-8"
            ) as f:
                policy_type = str(json.load(f).get("type", "act"))
            policy_cls = get_policy_class(policy_type)
            policy = policy_cls.from_pretrained(self.checkpoint_dir)
            if self.device is not None:
                policy.to(self.device)
            pre, post = make_pre_post_processors(
                policy.config,
                pretrained_path=self.checkpoint_dir,
            )
        except Exception as exc:  # noqa: BLE001
            raise PolicyError(
                f"checkpoint load failed from {self.checkpoint_dir}: {exc}"
            ) from exc
        self._policy = policy
        self._pre = pre
        self._post = post

    def close(self) -> None:
        self._policy = None
        self._pre = None
        self._post = None

    def reset(self) -> None:
        if self._policy is not None:
            try:
                self._policy.reset()
            except Exception:  # noqa: BLE001
                pass

    def infer(self, obs: ObsBatch) -> np.ndarray:
        if self._policy is None or self._pre is None or self._post is None:
            raise PolicyError("InprocBackend not open()ed")
        import torch

        raw: dict[str, np.ndarray] = {"observation.state": obs.state.astype(np.float32)}
        for label, cam in self.image_keys.items():
            img = obs.images.get(cam)
            if img is None:
                continue
            raw[f"observation.images.{label}"] = np.asarray(img, dtype=np.uint8)
        try:
            from lerobot.policies.utils import prepare_observation_for_inference

            timing: dict[str, float] = {}
            with torch.inference_mode():
                t0 = time.perf_counter()
                obs_in = prepare_observation_for_inference(
                    raw,
                    next(self._policy.parameters()).device,
                    task=obs.prompt or self.default_prompt,
                    robot_type=None,
                )
                timing["prep_ms"] = (time.perf_counter() - t0) * 1000.0
                t1 = time.perf_counter()
                obs_in = self._pre(obs_in)
                timing["pre_ms"] = (time.perf_counter() - t1) * 1000.0
                t2 = time.perf_counter()
                if getattr(self._policy.config, "temporal_ensemble_coeff", None) is not None:
                    # 时序融合 checkpoint：select_action 内部做指数加权，返回单行；
                    # 引擎按 1 行 chunk 兜底（queue_async 会掉速，用 queue_sync）。
                    action = self._policy.select_action(obs_in)
                else:
                    # 常规策略（本仓库 pickup_act_480）：一次前向返回完整 chunk
                    # （n_action_steps 行），引擎拿回分块权 → queue_async 可达 30Hz、
                    # 图像上传频率降 n_action_steps 倍（每 chunk 一次而非每行一次）。
                    action = self._policy.predict_action_chunk(obs_in)
                timing["infer_ms"] = (time.perf_counter() - t2) * 1000.0
                t3 = time.perf_counter()
                action = self._post(action)
                timing["post_ms"] = (time.perf_counter() - t3) * 1000.0
            timing["total_ms"] = (time.perf_counter() - t0) * 1000.0
            self.last_timing = timing
            self.last_infer_s = time.perf_counter() - t0
            arr = np.asarray(action.detach().cpu().squeeze(0), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001
            raise PolicyError(f"inproc infer failed: {exc}") from exc
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if arr.shape[1] != self.action_dim:
            raise PolicyError(
                f"inproc action_dim {arr.shape[1]} != robot action_dim {self.action_dim}"
            )
        return arr


class StubBackend(PolicyBackend):
    """Deterministic no-op backend for tests / MuJoCo smoke runs (no network).

    Inferences a short constant-ish chunk so engines/nodes can be exercised
    without openpi/lerobot installed. Not for real deployment.
    """

    name = "stub"

    def __init__(self, *, action_dim: int, camera_map: dict[str, str], **_: object):
        self.action_dim = int(action_dim)
        self.camera_map = dict(camera_map)
        self._n = 0
        self.last_infer_s = 0.0

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def dummy_obs(self, action_dim: int, camera: int = 8) -> ObsBatch:
        return ObsBatch(
            state=np.zeros(action_dim, dtype=np.float64),
            images={label: np.zeros((camera, camera, 3), np.uint8)
                    for label in self.camera_map.values()},
            prompt="",
        )

    def infer(self, obs: ObsBatch) -> np.ndarray:
        self._n += 1
        self.last_infer_s = 1e-3
        # a gentle absolute drift so ticks visibly differ (sim smoke checks).
        phase = float(self._n % 4)
        out = np.zeros((4, self.action_dim), dtype=np.float64)
        out[:] = 0.05 * np.cos(phase + np.arange(self.action_dim))
        return out


def make_backend(
    *,
    backend_type: str,
    model: str = "act",
    action_dim: int,
    camera_map: dict[str, str],
    checkpoint_dir: str | None = None,
    host: str = "127.0.0.1",
    port: int = 8000,
    default_prompt: str = "",
    device: str | None = None,
    jpeg_transport: bool = False,
    jpeg_quality: int = 92,
    connect_timeout_s: float = 5.0,
    infer_timeout_s: float = 3.0,
    rtc_guided: bool = False,
    rtc_horizon: int = 10,
) -> PolicyBackend:
    """Backend factory. ``backend_type`` = transport: remote | inproc | stub.

    ``model`` = model family (act | pi05 | …) — consumed by the inproc loader
    for validation and by ``serve.py`` to pick the loading path; the remote
    backend is model-agnostic (the server owns the model).
    """
    bt = (backend_type or "remote").strip().lower()
    if bt == "remote":
        return RemoteBackend(
            host=host,
            port=port,
            action_dim=action_dim,
            slot_keys=camera_map,
            default_prompt=default_prompt,
            jpeg_transport=jpeg_transport,
            jpeg_quality=jpeg_quality,
            connect_timeout_s=connect_timeout_s,
            infer_timeout_s=infer_timeout_s,
            rtc_guided=rtc_guided,
            rtc_horizon=rtc_horizon,
        )
    if bt == "inproc":
        if not checkpoint_dir:
            raise PolicyError("inproc backend requires checkpoint_dir")
        return InprocBackend(
            checkpoint_dir=checkpoint_dir,
            action_dim=action_dim,
            image_keys={label: label for label in camera_map.values()},
            device=device,
            default_prompt=default_prompt,
        )
    if bt == "stub":
        return StubBackend(action_dim=action_dim, camera_map=camera_map)
    raise PolicyError(f"unknown backend_type={backend_type!r} (remote|inproc|stub)")
