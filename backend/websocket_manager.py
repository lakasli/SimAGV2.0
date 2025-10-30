from __future__ import annotations
from typing import Any, Set
from fastapi import WebSocket
import json

class WebSocketManager:
    def __init__(self) -> None:
        self._connections: Set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._connections.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        try:
            self._connections.discard(ws)
        except Exception:
            pass

    async def send_json_to(self, ws: WebSocket, data: Any) -> None:
        try:
            await ws.send_text(json.dumps(data, ensure_ascii=False, default=str))
        except Exception:
            self.disconnect(ws)

    async def broadcast_json(self, data: Any) -> None:
        message = json.dumps(data, ensure_ascii=False, default=str)
        for ws in list(self._connections):
            try:
                await ws.send_text(message)
            except Exception:
                self.disconnect(ws)