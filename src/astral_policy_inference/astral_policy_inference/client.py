"""Self-contained websocket policy client (no ``openpi_client`` dependency).

Mirrors ``openpi_client/websocket_client_policy.py`` so it is wire-compatible
with both ``serve.py`` and the VLA/openpi ``serve_policy.py``:

* connect -> the server first sends packed metadata (``_wait_for_server``);
  connection and inference reads have finite timeouts;
* ``infer(obs)`` -> pack -> send -> recv; a *string* response is an error;
* ``reset()`` is a no-op (matching openpi's client);
* ``close()`` actually closes the socket.

Serialization uses :mod:`astral_policy_inference.protocol` (the vendored
``__ndarray__`` format), so the package needs no external install beyond the
light ``websockets`` dependency.
"""

from __future__ import annotations

import time
from typing import Optional

from astral_policy_inference import protocol as _proto


class WebsocketClient:
    """A blocking websocket client speaking the policy-inference protocol."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8001,
        api_key: Optional[str] = None,
        connect_timeout_s: float = 5.0,
        infer_timeout_s: float = 3.0,
    ):
        self.host = host
        self.port = int(port)
        self.api_key = api_key
        self.connect_timeout_s = float(connect_timeout_s)
        self.infer_timeout_s = float(infer_timeout_s)
        if self.connect_timeout_s <= 0 or self.infer_timeout_s <= 0:
            raise ValueError("websocket timeouts must be positive")
        self._packer = _proto.Packer()
        self._ws, self._server_metadata = self._wait_for_server()

    def get_server_metadata(self) -> dict:
        return self._server_metadata

    def _wait_for_server(self):
        import websockets.sync.client

        url = f"ws://{self.host}:{self.port}"
        deadline = time.monotonic() + self.connect_timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"policy server {url} unavailable after {self.connect_timeout_s:g}s")
            try:
                conn = websockets.sync.client.connect(
                    url, open_timeout=remaining, close_timeout=1,
                    compression=None, max_size=None,
                )
                try:
                    metadata = _proto.unpackb(conn.recv(timeout=remaining))
                except Exception:
                    conn.close()
                    raise
                if not isinstance(metadata, dict):
                    conn.close()
                    raise ValueError(f"invalid policy server metadata: {metadata!r}")
                return conn, metadata
            except (OSError, ConnectionError):
                time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))

    def infer(self, obs: dict) -> dict:
        data = self._packer.pack(obs)
        self._ws.send(data)
        try:
            response = self._ws.recv(timeout=self.infer_timeout_s)
        except Exception:
            self.close()
            raise
        if isinstance(response, str):
            # server sends a string on error — treat as RuntimeError
            raise RuntimeError(f"Error in inference server:\n{response}")
        return _proto.unpackb(response)

    def reset(self) -> None:
        pass

    def close(self) -> None:
        ws = getattr(self, "_ws", None)
        if ws is not None:
            try:
                ws.close()
            except Exception:  # noqa: BLE001
                pass
            self._ws = None
