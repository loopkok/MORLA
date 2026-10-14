#!/usr/bin/env python3
"""Minimal pi0.5 inference server; run in the OpenPI environment on the GPU host."""

import argparse
import json
import logging
import time
from pathlib import Path

import numpy as np

from protocol import pack, unpack

LOG = logging.getLogger("pi05_server")
SLOTS = ("base_0_rgb", "left_wrist_0_rgb")


def validate_request(request):
    if not isinstance(request, dict):
        raise ValueError("request must be a mapping")
    state = np.asarray(request.get("observation/state"), dtype=np.float32)
    if state.shape != (8,) or not np.isfinite(state).all():
        raise ValueError("observation/state must be finite float32[8]")
    prompt = request.get("prompt")
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("prompt must be nonempty")
    clean = {"observation/state": state, "prompt": prompt}
    for slot in SLOTS:
        key = f"observation/camera/{slot}"
        image = np.asarray(request.get(key))
        if image.shape != (224, 224, 3) or image.dtype != np.uint8:
            raise ValueError(f"{key} must be uint8[224,224,3]")
        clean[key] = image
    return clean


def validate_actions(actions):
    arr = np.asarray(actions, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[0] < 1 or arr.shape[1] != 8 or not np.isfinite(arr).all():
        raise ValueError(f"policy actions must be finite [N,8], got {arr.shape}")
    gripper = arr[:, 7]
    outside = (gripper < 0.0) | (gripper > 1.0)
    if np.any(outside):
        LOG.warning(
            "clipping %d/%d predicted gripper ratios to [0,1] (range %.4f..%.4f)",
            int(np.count_nonzero(outside)), len(gripper),
            float(np.min(gripper)), float(np.max(gripper)),
        )
    arr[:, 7] = np.clip(arr[:, 7], 0.0, 1.0)
    return arr


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint-dir", required=True)
    ap.add_argument("--policy-config", default="pi05_astral")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--warmup", action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    import jax
    jax.config.update("jax_compilation_cache_dir", str(Path("~/.cache/jax").expanduser()))

    from openpi.policies import policy_config
    from openpi.training import config as train_config
    from websockets.sync.server import serve

    cfg = train_config.get_config(args.policy_config)
    camera_map = getattr(cfg.data, "camera_map", {})
    if set(camera_map) != set(SLOTS):
        raise SystemExit(f"checkpoint config must map exactly {SLOTS}; got {camera_map}")
    LOG.info("loading %s from %s", args.policy_config, args.checkpoint_dir)
    policy = policy_config.create_trained_policy(cfg, args.checkpoint_dir)
    if args.warmup:
        warm = {"observation/state": np.zeros(8, dtype=np.float32), "prompt": "warmup"}
        for slot in SLOTS:
            warm[f"observation/camera/{slot}"] = np.zeros((224, 224, 3), dtype=np.uint8)
        start = time.monotonic()
        validate_actions(policy.infer(warm)["actions"])
        LOG.info("warmup done in %.2fs", time.monotonic() - start)

    def handle(ws):
        ws.send(pack({"model": "pi05", "policy_config": args.policy_config,
                      "camera_slots": SLOTS, "action_dim": 8}))
        for data in ws:
            start = time.monotonic()
            try:
                request = validate_request(unpack(data))
                actions = validate_actions(policy.infer(request)["actions"])
                elapsed_ms = (time.monotonic() - start) * 1000
                ws.send(pack({"actions": actions, "server_ms": elapsed_ms}))
                LOG.info("infer %.1fms rows=%d", elapsed_ms, len(actions))
            except Exception as exc:
                LOG.exception("inference failed")
                ws.send(json.dumps({"error": str(exc)}))

    with serve(handle, args.host, args.port, compression=None, max_size=None) as server:
        LOG.info("ready ws://%s:%d", args.host, args.port)
        server.serve_forever()


if __name__ == "__main__":
    main()
