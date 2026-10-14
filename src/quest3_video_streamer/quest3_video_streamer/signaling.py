"""WebSocket signaling server for WebRTC setup/control.

Self-contained (vendored, no hand-tracking-sdk dependency).
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Any

from quest3_video_streamer.schemas import SignalingMessage, parse_signaling_message

_LOG = logging.getLogger(__name__)


@dataclass(slots=True)
class SignalingConnection:
    """One connected signaling client."""

    websocket: Any
    session_id: str | None = None


class VideoSignalingServer:
    """Async WebSocket signaling server."""

    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        port: int = 8765,
        on_message: Callable[[SignalingConnection, SignalingMessage], Awaitable[None]],
        on_connect: Callable[[SignalingConnection], Awaitable[None]] | None = None,
        on_disconnect: Callable[[SignalingConnection], Awaitable[None]] | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._on_message = on_message
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._server: Any = None
        self._connections: list[SignalingConnection] = []
        self._session_map: dict[str, SignalingConnection] = {}
        self._lock = asyncio.Lock()

    async def start(self) -> None:
        websockets = self._import_websockets()
        self._server = await websockets.serve(self._handle_client, self._host, self._port)

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        for connection in list(self._connections):
            try:
                await connection.websocket.close()
            except Exception:
                pass
        self._connections.clear()
        self._session_map.clear()

    async def send(self, connection: SignalingConnection, message: SignalingMessage) -> None:
        await connection.websocket.send(message.to_json())

    async def _handle_client(self, websocket: Any) -> None:
        connection = SignalingConnection(websocket=websocket)
        async with self._lock:
            self._connections.append(connection)
        try:
            if self._on_connect is not None:
                await self._on_connect(connection)
            async for raw in websocket:
                message = parse_signaling_message(str(raw))
                connection.session_id = message.session_id
                async with self._lock:
                    self._session_map[message.session_id] = connection
                await self._on_message(connection, message)
        except Exception as exc:
            # Quest 端退出/重连时会硬重置 WebSocket，属正常情况，降级为 info 日志。
            name = type(exc).__name__
            _LOG.info("client connection ended: %s: %s", name, exc)
        finally:
            async with self._lock:
                if connection in self._connections:
                    self._connections.remove(connection)
                if connection.session_id is not None:
                    self._session_map.pop(connection.session_id, None)
            if self._on_disconnect is not None:
                await self._on_disconnect(connection)

    def _import_websockets(self) -> Any:
        try:
            return __import__("websockets", fromlist=["serve"])
        except Exception as exc:
            raise RuntimeError(
                "websockets is required for video signaling. "
                "Install with: python3 -m pip install websockets"
            ) from exc
