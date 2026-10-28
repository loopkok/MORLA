#!/usr/bin/env python3
"""Unified policy server — serve any model family over one websocket protocol.

替代独立的 serve_act.py / serve_policy.py 入口。``--model`` 选择模型族，推理适配
分派到对应加载路径；协议层（metadata 首帧、msgpack、字符串错误、每连接 reset）完全共享。

模型族：
  --model act    （默认）lerobot 策略，进程内 ``InprocBackend`` 加载 checkpoint，
                 需 py3.12 lerobot + torch 环境（本机 conda lerobot env）；
  --model pi05   openpi pi0.5，经 ``create_trained_policy(get_config("pi05_astral"),
                 checkpoint_dir)`` 加载，需 openpi 环境（jax/torch + CUDA）+ **openpi
                 格式 checkpoint**（含 params/ 或 model.safetensors + assets/<asset_id>/
                 norm_stats.json；本仓库 astral_ckpt 是 ACT 格式，serve pi05 前需先有）。

机器人侧节点（py3.10/rclpy）用 ``backend_type=remote`` + ``host``/``port`` 连它——
线上动作恒为**绝对量**（(n, action_dim)，dataset 布局序）。

协议（与 openpi websocket_policy_server 一致，客户端用包内 RemoteBackend/client 即可）：
  * 连接后服务端先发 msgpack metadata；
  * 每请求 msgpack ``{observation/state, observation/camera/<slot>, prompt}``；
  * 每响应 msgpack ``{actions: (n, action_dim), server_timing?}``；
  * 字符串响应视为错误。序列化用包内 vendored ``protocol``（键 __ndarray__）。

用法：
  # ACT
  <py3.12 lerobot-env>/bin/python serve.py --model act \
      --checkpoint-dir <lerobot checkpoint> --port 8001
  # pi05（需 openpi env + openpi 格式 checkpoint）
  <openpi-env>/bin/python serve.py --model pi05 \
      --checkpoint-dir <openpi checkpoint> --port 8001
  # 节点：-p backend_type:=remote -p host:=<server> -p port:=8001 -p camera_image_size:=480
"""

from __future__ import annotations

import argparse
import asyncio
import json
import pathlib
import sys

# 脚本可直接运行：按脚本自身位置定位 Astral 包，不依赖部署主机用户名、
# 工作目录或开发机的绝对路径。布局为 package/scripts/serve.py 与
# package/morla_policy_inference/ 并列。
_PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
_WORKSPACE_SRC = _PACKAGE_ROOT.parent
for _path in (_PACKAGE_ROOT, _WORKSPACE_SRC / "morla_data_collect"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from morla_policy_inference import protocol as _proto
from morla_policy_inference.image_codec import normalize_request_images

_CONNECTIONS = 0  # 连接计数：观测断线/重连频率

# ---- pi05 诊断捕获：把每次推理的输入(图像/state/prompt)与输出首行动作落盘 ----
_CAPTURE_DIR = None  # 非空时启用（--capture-dir）
_CAPTURE_N = 0


def _maybe_capture(request: dict, response: dict) -> None:
    """每 30 次推理存一次输入图像/state（~1Hz），每次存首行动作到 actions.jsonl。"""
    global _CAPTURE_DIR, _CAPTURE_N
    if not _CAPTURE_DIR:
        return
    _CAPTURE_N += 1
    n = _CAPTURE_N
    if n % 30 == 1:
        import json
        import os

        import numpy as np

        for slot in ("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"):
            img = request.get(f"observation/camera/{slot}")
            if img is not None:
                np.save(
                    os.path.join(_CAPTURE_DIR, f"req{n:06d}_{slot}.npy"),
                    np.asarray(img),
                )
        with open(os.path.join(_CAPTURE_DIR, f"req{n:06d}_meta.json"), "w") as f:
            json.dump(
                {
                    "req": n,
                    "state": [float(x) for x in request.get("observation/state", [])],
                    "prompt": request.get("prompt", ""),
                },
                f,
            )
    actions = response.get("actions")
    if actions is not None:
        import json
        import os

        import numpy as np

        with open(os.path.join(_CAPTURE_DIR, "actions.jsonl"), "a") as f:
            f.write(
                json.dumps({"req": n, "action0": [float(x) for x in np.asarray(actions)[0]]})
                + "\n"
            )


# ------------------------------------------------------------ 推理适配

def _build_act_backend(args) -> object:
    from morla_policy_inference.backend import InprocBackend

    return InprocBackend(
        checkpoint_dir=args.checkpoint_dir,
        action_dim=args.action_dim,
        image_keys={label: label for label in args.slot_map.values()},
        device=args.device,
        default_prompt=args.default_prompt,
    )


def _infer_act(backend, slot_map: dict[str, str], request: dict) -> dict:
    from morla_policy_inference.backend import ObsBatch

    state = request.get("observation/state")
    if state is None:
        raise ValueError("request missing observation/state")
    images = {}
    for slot, label in slot_map.items():
        key = f"observation/camera/{slot}"
        if key in request:
            images[label] = request[key]
    obs = ObsBatch(
        state=state,
        images=images,
        prompt=request.get("prompt", ""),
    )
    actions = backend.infer(obs)
    response: dict = {"actions": actions}
    timing = getattr(backend, "last_timing", None)
    if timing:
        response["server_timing"] = timing  # prep/pre/infer/post/total ms
    return response


def _build_pi05_policy(args):
    try:
        from openpi.policies import policy_config
        from openpi.training import config as _config
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "--model pi05 requires the openpi environment "
            "(run in VLA/openpi uv venv with PYTHONPATH=openpi/src): "
            f"{exc}"
        ) from exc
    # 持久化 XLA 编译缓存（与训练侧同一目录，见 scripts/train.py）：换进程 / 换
    # checkpoint 重启 serve 时，首轮推理可从磁盘复用已编译图，不必每次重编 2-5 分钟。
    try:
        import pathlib as _pathlib

        import jax

        jax.config.update(
            "jax_compilation_cache_dir", str(_pathlib.Path("~/.cache/jax").expanduser())
        )
    except Exception:  # pragma: no cover - 缓存只是加速，失败不影响 serve
        pass
    train_config = _config.get_config(args.policy_config)
    policy = policy_config.create_trained_policy(
        train_config,
        args.checkpoint_dir,
        default_prompt=args.default_prompt or None,
    )
    return policy


def _infer_pi05(policy, request: dict, *, rtc_guided: bool = False, rtc_horizon: int = 10) -> dict:
    rtc = None
    if "rtc_prefix" in request:
        if not rtc_guided:
            raise ValueError("server was not started with --rtc-guided")
        import numpy as np

        prefix = np.asarray(request["rtc_prefix"], dtype=np.float32)
        if prefix.ndim != 2 or prefix.shape[0] != rtc_horizon:
            raise ValueError(f"RTC prefix must have {rtc_horizon} rows, got {prefix.shape}")
        rtc = {
            "prefix": prefix,
            "valid": int(request["rtc_prefix_valid"]),
            "delay": int(request["rtc_delay"]),
            "max_guidance_weight": float(request["rtc_max_guidance_weight"]),
            "schedule": str(request["rtc_schedule"]),
        }
        request = {k: v for k, v in request.items() if not k.startswith("rtc_")}
    resp = policy.infer(request, rtc=rtc) if rtc is not None else policy.infer(request)
    actions = resp.get("actions")
    if actions is None:
        raise ValueError("openpi policy returned no actions")
    response: dict = {"actions": actions}
    timing = resp.get("policy_timing")
    if timing:
        response["server_timing"] = timing
    _maybe_capture(request, response)
    return response


def _warmup_policy(
    policy, state_dim: int, camera_map: dict[str, str],
    *, rtc_guided: bool = False, rtc_horizon: int = 10,
) -> None:
    """进程内预热：pi05 首轮推理要 XLA 编译（2-5 分钟），启动时先做一次 dummy
    infer 把编译做掉，真机连上后第一个 chunk 不用干等。"""
    import time

    import numpy as np

    req: dict = {
        "observation/state": np.zeros(state_dim, dtype=np.float32),
        "prompt": "warmup",
    }
    for slot in camera_map:
        req[f"observation/camera/{slot}"] = np.zeros((224, 224, 3), dtype=np.uint8)
    t0 = time.monotonic()
    policy.infer(req)
    if rtc_guided:
        # Guided requests have a different JAX graph.  Compile it before a
        # robot starts consuming the initial chunk.
        policy.infer(req, rtc={
            "prefix": np.zeros((rtc_horizon, state_dim), dtype=np.float32),
            "valid": rtc_horizon, "delay": min(1, rtc_horizon),
            "max_guidance_weight": 10.0, "schedule": "linear",
        })
    print(f"serve[pi05]: warmup done in {time.monotonic() - t0:.1f}s", flush=True)


# ------------------------------------------------------------ 协议层

async def _handle(
    websocket, infer_fn, reset_fn, packer, model: str, action_dim: int,
    rtc_guided: bool, rtc_horizon: int, layout_id: str,
    layout_hash: str, model_capacity: int, kinematics_hash: str,
    model_manifest: dict | None = None,
) -> None:
    import websockets.exceptions

    global _CONNECTIONS
    _CONNECTIONS += 1
    print(f"serve[{model}]: connection #{_CONNECTIONS} opened", flush=True)

    # 新连接 = 新节点 POLICY 会话：重置策略内部状态（如 ACT 的 50 行缓存队列，
    # 否则上次会话未排完的缓存会被吐出 → 陈旧动作）。协议无 reset 消息，服务端每连接重置。
    reset_fn()

    try:
        await websocket.send(packer.pack({
            "model": model, "action_dim": action_dim,
            "rtc_guided": rtc_guided, "rtc_horizon": rtc_horizon if rtc_guided else 0,
            "layout_id": layout_id, "layout_hash": layout_hash,
            "model_capacity": model_capacity,
            "kinematics_hash": kinematics_hash,
            **({"model_manifest":model_manifest} if model_manifest else {}),
        }))
        while True:
            data = await websocket.recv()
            if isinstance(data, str):
                # 客户端对任何字符串响应抛错；我方 client 不会发字符串请求
                await websocket.send("unexpected string request")
                continue
            request = _proto.unpackb(data)
            try:
                # 节点 jpeg_transport 时 camera 槽位是 JPEG 字节 → 归一化为 RGB 数组，
                # 并移除 image_format 标志（ACT/pi05 两分支都消费 RGB，无需各自处理）。
                request = normalize_request_images(request)
                response = infer_fn(request)
            except Exception as exc:  # noqa: BLE001
                await websocket.send(f"infer failed: {exc}")
                continue
            await websocket.send(packer.pack(response))
    except websockets.exceptions.ConnectionClosed:
        pass  # 客户端正常断开，非错误


async def _serve(
    infer_fn, reset_fn, host: str, port: int, model: str, action_dim: int,
    rtc_guided: bool = False, rtc_horizon: int = 10, layout_id: str = "",
    layout_hash: str = "", model_capacity: int | None = None,
    kinematics_hash: str = "",
    model_manifest: dict | None = None,
) -> None:
    from websockets.asyncio.server import serve

    packer = _proto.Packer()
    capacity = int(model_capacity or action_dim)
    if capacity < action_dim:
        raise ValueError(f"action_dim {action_dim} exceeds model_capacity {capacity}")
    async with serve(
        lambda ws: _handle(ws, infer_fn, reset_fn, packer, model, action_dim,
                            rtc_guided, rtc_horizon, layout_id, layout_hash, capacity,
                            kinematics_hash,model_manifest),
        host,
        port,
        compression=None,
        max_size=None,
    ) as server:
        print(f"serve[{model}] ready: {model} on ws://{host}:{port}", flush=True)
        await server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="act", choices=["act", "pi05"],
                        help="模型族：act(lerobot) | pi05(openpi)")
    parser.add_argument("--checkpoint-dir", required=True,
                        help="act=lerobot checkpoint；pi05=openpi 格式 checkpoint")
    parser.add_argument('--model-manifest',default='',help='schema v3 checkpoint manifest, hash checked before loading')
    parser.add_argument('--training-spec',default='',help='OpenPI composed TrainingSpec; avoids fixed embodiment transforms')
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--action-dim", type=int, default=8, help="act 用")
    parser.add_argument("--layout-id", default="", help="冻结 action layout 的稳定 ID")
    parser.add_argument("--layout-hash", default="", help="profile FrozenSchema.layout_hash")
    parser.add_argument("--kinematics-hash", default="", help="profile resolved coordinate/kinematics contract hash")
    parser.add_argument("--model-capacity", type=int, default=0,
                        help="模型可输出的最大动作维度；默认等于 --action-dim")
    parser.add_argument("--policy-config", default="pi05_astral", help="pi05 用：openpi 训练配置名")
    parser.add_argument("--device", default=None, help="act 用：torch device")
    parser.add_argument(
        "--warmup",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="pi05 用：加载模型后先做一次 dummy 推理预热 XLA 编译（默认开；--no-warmup 关）",
    )
    parser.add_argument(
        "--capture-dir",
        default="",
        help="pi05 诊断：把每次推理的输入图像(state/图像)与输出首行动作落盘到该目录（每 30 次存图）",
    )
    parser.add_argument("--default-prompt", default="")
    parser.add_argument("--rtc-guided", action="store_true",
                        help="pi05：预编译 LeRobot 式 guided RTC 并接受 RTC 前缀请求")
    parser.add_argument("--rtc-execution-horizon", type=int, default=10,
                        help="RTC 前缀长度，须与 policy_node 参数一致")
    parser.add_argument(
        "--slot-map",
        # 值 = **模型 input_features 里的图像键**（观察传到 observation.images.<值>），
        # 不是 collect label。旧模型（video8/video0 训的）= video8/video0；新模型
        # （语义名 base/left_wrist 训的）= base/left_wrist。键=传输槽名，须与节点 camera_map 键一致。
        default='{"base_0_rgb": "video8", "left_wrist_0_rgb": "video0"}',
        help="JSON: model slot -> model feature image key（须与模型 input_features 一致）",
    )
    args = parser.parse_args()
    args.model_manifest_value=None
    if args.model_manifest:
        from morla_learning import ModelManifest
        from morla_learning.backends import checkpoint_hash
        manifest=ModelManifest.from_dict(json.loads(pathlib.Path(args.model_manifest).read_text()))
        if manifest.checkpoint_hash!=checkpoint_hash(args.checkpoint_dir): parser.error('checkpoint content hash mismatch')
        if manifest.backend!=('act' if args.model=='act' else 'openpi'): parser.error('manifest/backend mismatch')
        if manifest.validation!='inference_verified': parser.error('model manifest has no inference verification')
        args.model_manifest_value=manifest.to_dict()
        args.action_dim=len(manifest.features['action_features'])
        args.model_capacity=manifest.capacity
        args.slot_map=json.dumps(manifest.camera_map)
    if args.rtc_execution_horizon < 1:
        parser.error("--rtc-execution-horizon must be >= 1")
    if args.rtc_guided and args.model != "pi05":
        parser.error("--rtc-guided requires --model pi05")
    if args.rtc_guided and not args.warmup:
        parser.error("--rtc-guided requires warmup so the guided graph compiles before robot control")
    if args.action_dim < 1 or (args.model_capacity and args.action_dim > args.model_capacity):
        parser.error("--action-dim must be positive and cannot exceed --model-capacity")
    try:
        args.slot_map = {str(k): str(v) for k, v in json.loads(args.slot_map).items()}
    except json.JSONDecodeError as exc:
        parser.error(f"--slot-map must be JSON object: {exc}")

    if args.capture_dir:
        import os

        global _CAPTURE_DIR
        _CAPTURE_DIR = args.capture_dir
        os.makedirs(_CAPTURE_DIR, exist_ok=True)
        print(f"serve: 诊断捕获开启 → {_CAPTURE_DIR}", flush=True)

    print(f"serve[{args.model}]: loading checkpoint {args.checkpoint_dir} ...", flush=True)
    if args.model == "pi05":
        from openpi.training import config as _config

        train_config = _config.get_config(args.policy_config)
        if args.rtc_guided and args.rtc_execution_horizon > train_config.model.action_horizon:
            parser.error("RTC execution horizon exceeds the model action horizon")
        policy = _build_pi05_policy(args)
        if args.warmup:
            _warmup_policy(
                policy, args.action_dim, getattr(train_config.data, "camera_map", {}),
                rtc_guided=args.rtc_guided, rtc_horizon=args.rtc_execution_horizon,
            )
        infer_fn = lambda req: _infer_pi05(  # noqa: E731
            policy, req, rtc_guided=args.rtc_guided, rtc_horizon=args.rtc_execution_horizon
        )
        reset_fn = lambda: getattr(policy, "reset", lambda: None)()  # noqa: E731
    else:
        backend = _build_act_backend(args)
        backend.open()
        infer_fn = lambda req: _infer_act(backend, args.slot_map, req)  # noqa: E731
        reset_fn = backend.reset
    print(f"serve[{args.model}]: loaded; serving over websocket", flush=True)
    try:
        asyncio.run(_serve(
            infer_fn, reset_fn, args.host, args.port, args.model, args.action_dim,
            args.rtc_guided, args.rtc_execution_horizon, args.layout_id,
            args.layout_hash, args.model_capacity, args.kinematics_hash,
            args.model_manifest_value,
        ))
    except KeyboardInterrupt:
        print(f"serve[{args.model}] stopped")


if __name__ == "__main__":
    main()
