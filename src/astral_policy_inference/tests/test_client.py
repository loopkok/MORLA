"""Regression checks for bounded policy-server connection and reads."""

import sys
import time
import types
import unittest
from unittest import mock

import numpy as np

from astral_policy_inference.client import WebsocketClient
from astral_policy_inference import protocol


class ClientTimeoutTests(unittest.TestCase):
    def _modules(self, connect):
        client_mod = types.ModuleType("websockets.sync.client")
        client_mod.connect = connect
        sync_mod = types.ModuleType("websockets.sync")
        sync_mod.client = client_mod
        websockets_mod = types.ModuleType("websockets")
        websockets_mod.sync = sync_mod
        return {
            "websockets": websockets_mod,
            "websockets.sync": sync_mod,
            "websockets.sync.client": client_mod,
        }

    def test_unavailable_server_times_out(self):
        def unavailable(*_args, **_kwargs):
            raise ConnectionRefusedError("down")

        started = time.monotonic()
        with mock.patch.dict(sys.modules, self._modules(unavailable)):
            with self.assertRaisesRegex(TimeoutError, "unavailable"):
                WebsocketClient("127.0.0.1", 8001, connect_timeout_s=0.05)
        self.assertLess(time.monotonic() - started, 0.5)

    def test_legacy_metadata_and_response_timeout(self):
        class Connection:
            closed = False
            def recv(self, timeout=None):
                if not hasattr(self, "seen_metadata"):
                    self.seen_metadata = True
                    return protocol.packb({"model": "pi05"})
                raise TimeoutError("infer stalled")
            def send(self, _data):
                pass
            def close(self):
                self.closed = True

        conn = Connection()
        with mock.patch.dict(sys.modules, self._modules(lambda *_a, **_k: conn)):
            client = WebsocketClient("127.0.0.1", 8001, infer_timeout_s=0.01)
            self.assertEqual(client.get_server_metadata(), {"model": "pi05"})
            with self.assertRaisesRegex(TimeoutError, "infer stalled"):
                client.infer({"observation/state": np.zeros(8, np.float32)})
        self.assertTrue(conn.closed)
