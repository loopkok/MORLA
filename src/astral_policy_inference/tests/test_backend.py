#!/usr/bin/env python3
"""Tests for policy backends + factory (no network / no torch needed)."""

import json
import os
import sys
import tempfile
import types
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from astral_policy_inference.backend import (
    InprocBackend,
    ObsBatch,
    PolicyError,
    RemoteBackend,
    StubBackend,
    _actions_from_response,
    make_backend,
)

ACT_DIM = 8
CAM_MAP = {"base_0_rgb": "base", "left_wrist_0_rgb": "left_wrist"}


class FakeClient:
    def __init__(self, host, port, response=None, raise_on_infer=False):
        self.host, self.port = host, port
        self.response = response if response is not None else {
            "actions": np.zeros((5, ACT_DIM)),
        }
        self.raise_on_infer = raise_on_infer
        self.payloads = []
        self.reset_calls = 0
        self.closed = False

    def infer(self, payload):
        if self.raise_on_infer:
            raise ConnectionError("boom")
        self.payloads.append(payload)
        return self.response

    def reset(self):
        self.reset_calls += 1

    def close(self):
        self.closed = True


class TestPayloadMapping(unittest.TestCase):
    def _make(self, client):
        return RemoteBackend(
            host="h", port=8000, action_dim=ACT_DIM,
            slot_keys=CAM_MAP, client_factory=lambda *a, **k: client,
        )

    def test_remote_payload_keys(self):
        client = FakeClient("h", 8000)
        bk = self._make(client)
        bk.open()
        obs = ObsBatch(
            state=np.arange(ACT_DIM, dtype=np.float64),
            images={"base": np.zeros((8, 8, 3), np.uint8)},
            prompt="pick up",
        )
        out = bk.infer(obs)
        self.assertEqual(out.shape[1], ACT_DIM)
        payload = client.payloads[-1]
        self.assertTrue(np.array_equal(payload["observation/state"], obs.state))
        self.assertEqual(payload["prompt"], "pick up")
        self.assertIn("observation/camera/base_0_rgb", payload)
        # missing slot stays absent -> server zero-pads (AstralInputs.image_mask)
        self.assertNotIn("observation/camera/left_wrist_0_rgb", payload)

    def test_missing_actions_key_raises(self):
        client = FakeClient("h", 8000, response={"not_actions": []})
        bk = self._make(client)
        bk.open()
        with self.assertRaises(PolicyError):
            bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={}, prompt=""))

    def test_backend_raises_when_not_open(self):
        bk = self._make(FakeClient("h", 8000))
        with self.assertRaises(PolicyError):
            bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={}, prompt=""))

    def test_dim_mismatch_surfaces_policy_error(self):
        client = FakeClient("h", 8000,
                            response={"actions": np.zeros((3, ACT_DIM + 1))})
        bk = self._make(client)
        bk.open()
        with self.assertRaises(PolicyError):
            bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={}, prompt=""))

    def test_standalone_server_timing_is_visible(self):
        client = FakeClient("h", 8000, response={
            "actions": np.zeros((2, ACT_DIM)), "server_ms": 88.5,
        })
        bk = self._make(client)
        bk.open()
        bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={}, prompt="test"))
        self.assertEqual(bk.last_server_timing, {"total_ms": 88.5})

    def test_handshake_rejects_wrong_action_dim_and_closes(self):
        client = FakeClient("h", 8000)
        client.get_server_metadata = lambda: {"model": "pi05", "action_dim": 9}
        bk = RemoteBackend(host="h", port=8000, action_dim=ACT_DIM, slot_keys=CAM_MAP)
        with mock.patch("astral_policy_inference.client.WebsocketClient", return_value=client):
            with self.assertRaisesRegex(PolicyError, "action_dim"):
                bk.open()
        self.assertTrue(client.closed)

    def test_close_resets_client(self):
        client = FakeClient("h", 8000)
        bk = self._make(client)
        bk.open()
        bk.reset()
        bk.close()
        self.assertEqual(client.reset_calls, 1)


class TestActionsFromResponse(unittest.TestCase):
    def test_1d_and_2d(self):
        a = _actions_from_response({"actions": np.zeros(ACT_DIM)}, ACT_DIM, 50)
        self.assertEqual(a.shape, (1, ACT_DIM))
        b = _actions_from_response({"actions": np.zeros((4, ACT_DIM))}, ACT_DIM, 2)
        self.assertEqual(b.shape, (2, ACT_DIM))

    def test_dim_mismatch(self):
        with self.assertRaises(PolicyError):
            _actions_from_response({"actions": np.zeros((4, 9))}, ACT_DIM, 50)

    def test_empty_and_nonfinite_actions_fail(self):
        for actions in (np.zeros((0, ACT_DIM)), np.full((2, ACT_DIM), np.nan)):
            with self.subTest(shape=actions.shape), self.assertRaises(PolicyError):
                _actions_from_response({"actions": actions}, ACT_DIM, 50)


class TestFactory(unittest.TestCase):
    def test_remote_selected(self):
        bk = make_backend(backend_type="remote", action_dim=ACT_DIM, camera_map=CAM_MAP)
        self.assertIsInstance(bk, RemoteBackend)

    def test_inproc_requires_checkpoint(self):
        with self.assertRaises(PolicyError):
            make_backend(backend_type="inproc", action_dim=ACT_DIM, camera_map=CAM_MAP)

    def test_stub_and_unknown(self):
        bk = make_backend(backend_type="stub", action_dim=ACT_DIM, camera_map={})
        self.assertIsInstance(bk, StubBackend)
        with self.assertRaises(PolicyError):
            make_backend(backend_type="nope", action_dim=ACT_DIM, camera_map={})


class TestInprocBackendLoad(unittest.TestCase):
    """Regression: ``InprocBackend.open()`` must resolve the concrete policy
    class through the lerobot factory. In lerobot 0.6.2 (both this fork and
    VLA/lerobot) ``PreTrainedPolicy`` is the *abstract* base — calling
    ``PreTrainedPolicy.from_pretrained`` directly cannot instantiate a real
    checkpoint (``Can't instantiate abstract class``). The factory's
    ``get_policy_class`` maps ``config.json["type"]`` ("act") -> ``ACTPolicy``.

    Hermetic: lerobot is faked via ``sys.modules`` so this runs without lerobot
    installed (matches the file's "no torch needed" contract)."""

    def _fake_lerobot(self, seen):
        fake = types.ModuleType("lerobot.policies")

        class FakePolicy:
            @classmethod
            def from_pretrained(cls, path):
                seen.append(("from_pretrained", path))
                return SimpleNamespace(
                    config=SimpleNamespace(type="act"), to=lambda *a, **k: None
                )

        fake.get_policy_class = (
            lambda t: seen.append(("get_policy_class", t)) or FakePolicy
        )
        fake.make_pre_post_processors = (
            lambda cfg, pretrained_path=None: ((lambda x: x), (lambda x: x))
        )
        return fake

    def test_open_resolves_concrete_policy_via_factory(self):
        seen = []
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "config.json"), "w") as f:
                json.dump({"type": "act"}, f)
            fake = self._fake_lerobot(seen)
            with mock.patch.dict(
                sys.modules,
                {"lerobot": types.ModuleType("lerobot"), "lerobot.policies": fake},
            ):
                bk = InprocBackend(checkpoint_dir=d, action_dim=8, image_keys={})
                bk.open()
        self.assertEqual(
            seen,
            [("get_policy_class", "act"), ("from_pretrained", d)],
        )
        self.assertIsNotNone(bk._policy)
        self.assertIsNotNone(bk._pre)
        self.assertIsNotNone(bk._post)

    def test_open_unknown_type_raises_policy_error(self):
        seen = []
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "config.json"), "w") as f:
                json.dump({"type": "nope"}, f)
            fake = self._fake_lerobot(seen)

            def boom(t):
                raise ValueError(f"unknown policy {t}")

            fake.get_policy_class = boom
            with mock.patch.dict(
                sys.modules,
                {"lerobot": types.ModuleType("lerobot"), "lerobot.policies": fake},
            ):
                bk = InprocBackend(checkpoint_dir=d, action_dim=8, image_keys={})
                with self.assertRaises(PolicyError):
                    bk.open()


class TestProtocol(unittest.TestCase):
    """Vendored __ndarray__ serializer: round-trip + wire format lock.

    The wire keys must stay ``b"__ndarray__"`` (openpi/ACT serve compatible),
    NOT pip msgpack-numpy's ``b"nd"`` — both the node client and serve.py use
    this one serializer, so the two ends always agree.
    """

    def test_ndarray_round_trip(self):
        from astral_policy_inference import protocol

        for arr in (np.arange(8, dtype=np.float32),
                    np.zeros((50, 8), dtype=np.float64),
                    np.ones((480, 480, 3), dtype=np.uint8),
                    np.float32(3.5)):
            blob = protocol.packb(arr)
            out = protocol.unpackb(blob)
            self.assertTrue(np.array_equal(np.asarray(out), np.asarray(arr)),
                            f"round-trip failed for {np.asarray(arr).shape}")

    def test_wire_key_is_ndarray(self):
        from astral_policy_inference import protocol

        blob = protocol.packb(np.zeros(4, dtype=np.float32))
        self.assertIn(b"__ndarray__", blob, "wire format must use __ndarray__")


class TestJpegTransport(unittest.TestCase):
    """上行 JPEG 传输：RemoteBackend 编码 / serve 解码 / 保真。"""

    def _jpeg_backend(self, client, on: bool):
        return RemoteBackend(
            host="h", port=8000, action_dim=ACT_DIM,
            slot_keys=CAM_MAP, client_factory=lambda *a, **k: client,
            jpeg_transport=on,
        )

    def test_jpeg_on_sends_encoded_bytes_and_flag(self):
        from astral_policy_inference.image_codec import decode_jpeg, JPEG_MAGIC

        client = FakeClient("h", 8000)
        bk = self._jpeg_backend(client, on=True)
        bk.open()
        rng = np.random.default_rng(0)
        img = rng.integers(0, 256, (64, 64, 3), np.uint8)
        bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={"base": img}, prompt=""))
        payload = client.payloads[0]
        self.assertEqual(payload["image_format"], "jpeg")
        v = payload["observation/camera/base_0_rgb"]
        self.assertIsInstance(v, bytes, "jpeg 传输下 camera 槽位应为字节")
        self.assertTrue(v.startswith(JPEG_MAGIC), "应为 JPEG magic 0xFFD8")
        back = decode_jpeg(v)
        self.assertEqual(back.shape, (64, 64, 3))
        bk.close()

    def test_jpeg_off_keeps_rgb_arrays_no_flag(self):
        client = FakeClient("h", 8000)
        bk = self._jpeg_backend(client, on=False)
        bk.open()
        img = np.zeros((8, 8, 3), np.uint8)
        bk.infer(ObsBatch(state=np.zeros(ACT_DIM), images={"base": img}, prompt=""))
        payload = client.payloads[0]
        self.assertNotIn("image_format", payload)
        v = payload["observation/camera/base_0_rgb"]
        self.assertIsInstance(v, np.ndarray)
        self.assertEqual(v.shape, (8, 8, 3))
        bk.close()

    def test_jpeg_roundtrip_shape_and_fidelity(self):
        from astral_policy_inference.image_codec import decode_jpeg, encode_jpeg

        y, x = np.mgrid[0:64, 0:64]
        img = np.stack([(x / 64 * 255).astype(np.uint8),
                        (y / 64 * 255).astype(np.uint8),
                        np.full((64, 64), 128, np.uint8)], axis=-1)
        back = decode_jpeg(encode_jpeg(img, quality=92))
        self.assertEqual(back.shape, (64, 64, 3))
        self.assertEqual(back.dtype, np.uint8)
        self.assertLess(np.abs(back.astype(int) - img.astype(int)).max(), 25,
                        "quality=92 平滑图 round-trip 最大偏差应 <25/255")

    def test_normalize_request_images_decodes_and_removes_flag(self):
        from astral_policy_inference.image_codec import (
            decode_jpeg, encode_jpeg, normalize_request_images,
        )

        img = np.zeros((16, 16, 3), np.uint8)
        img[..., 0] = 255
        blob = encode_jpeg(img, quality=92)
        req = {"observation/state": np.zeros(ACT_DIM),
               "observation/camera/cam0": blob, "image_format": "jpeg"}
        out = normalize_request_images(req)
        self.assertNotIn("image_format", out, "标志必须移除（避免污染 openpi AstralInputs）")
        v = out["observation/camera/cam0"]
        self.assertIsInstance(v, np.ndarray)
        self.assertEqual(v.shape, (16, 16, 3))
        # 非 jpeg 载荷原样透传（引用不变）
        req2 = {"observation/camera/cam0": np.zeros((16, 16, 3), np.uint8)}
        self.assertIs(normalize_request_images(req2)["observation/camera/cam0"],
                      req2["observation/camera/cam0"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
