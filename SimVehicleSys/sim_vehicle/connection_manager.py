from __future__ import annotations
from typing import Any

from SimVehicleSys.mqtt.handlers import MqttHandler


class ConnectionManager:
    """连接管理封装：使用新架构下的 MQTT 处理器。"""

    def __init__(self, config: Any, simulator: Any) -> None:
        self._handler = MqttHandler(config, simulator)

    def start(self) -> None:
        self._handler.start()