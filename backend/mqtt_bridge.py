from __future__ import annotations
import json
import uuid
from typing import Optional
from pathlib import Path

import paho.mqtt.client as mqtt
from SimVehicleSys.config.settings import get_config

from .agv_manager import AGVManager
from .schemas import DynamicConfigUpdate, Position


class MQTTBridge:
    def __init__(self, config_path: Path, ws_broadcast, loop, agv_manager: Optional[AGVManager] = None):
        self.config_path = config_path
        self.ws_broadcast = ws_broadcast  # async function to broadcast json
        self.loop = loop
        self.agv_manager = agv_manager
        self.client: Optional[mqtt.Client] = None
        # 初始化占位，真正值在 _load_config 中从集中设置读取
        self.host: str = ""
        self.port: int = 0
        self.vda_interface: str = ""
        self._equip_interface: str = "uequip"
        self._load_config()

    def _load_config(self) -> None:
        # 从 SimVehicleSys.config.settings 加载集中配置（严格模式）
        cfg = get_config()
        self.host = str(cfg.mqtt_broker.host)
        self.port = int(str(cfg.mqtt_broker.port))
        self.vda_interface = str(cfg.mqtt_broker.vda_interface)
        try:
            env_equip_iface = __import__("os").getenv("SIMAGV_MQTT_EQUIP_INTERFACE")
            if env_equip_iface:
                self._equip_interface = str(env_equip_iface)
        except Exception:
            pass

    def start(self) -> None:
        self.client = mqtt.Client(client_id=str(uuid.uuid4()), protocol=mqtt.MQTTv5)
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.on_message = self._on_message
        try:
            self.client.connect(self.host, self.port, keepalive=60)
        except Exception as e:
            print(f"MQTTBridge connect failed: {e}")
            return
        try:
            self.client.subscribe(f"{self.vda_interface}/#", qos=1)
        except Exception:
            pass
        try:
            # 设备主题桥接（如 DoorSim 使用 uequip/v2/...）
            self.client.subscribe(f"{self._equip_interface}/#", qos=1)
        except Exception:
            pass
        self.client.loop_start()

    def stop(self) -> None:
        try:
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
        except Exception:
            pass

    @staticmethod
    def _get(payload: dict, snake: str, camel: str, default=None):
        return payload.get(snake, payload.get(camel, default))

    def _sink_state_into_manager(self, payload: dict) -> None:
        # 提取序列号（支持 snake/camel）
        serial = str(self._get(payload, "serial_number", "serialNumber", "")).strip()
        if not serial or not self.agv_manager:
            return
        # 位置（支持 snake/camel）
        agv_pos = self._get(payload, "agv_position", "agvPosition", {}) or {}
        x = self._get(agv_pos, "x", "x", None)
        y = self._get(agv_pos, "y", "y", None)
        theta = self._get(agv_pos, "theta", "theta", None)
        map_id = self._get(agv_pos, "map_id", "mapId", None)
        # 电量（支持 snake/camel）
        batt = self._get(payload, "battery_state", "batteryState", {}) or {}
        battery_charge = self._get(batt, "battery_charge", "batteryCharge", None)

        dyn = DynamicConfigUpdate()
        try:
            if map_id is not None:
                dyn.current_map = str(map_id)
        except Exception:
            pass
        try:
            if battery_charge is not None:
                # 转为 int 百分比
                dyn.battery_level = int(float(battery_charge))
        except Exception:
            pass
        pos = None
        try:
            if x is not None or y is not None or theta is not None:
                pos = Position(
                    x=float(x) if x is not None else 0.0,
                    y=float(y) if y is not None else 0.0,
                    theta=float(theta) if theta is not None else None,
                )
        except Exception:
            pos = None
        if pos:
            dyn.position = pos
        try:
            self.agv_manager.update_dynamic(serial, dyn)
        except Exception as e:
            # 保守容错，不影响消息转发
            print(f"MQTTBridge state sink failed for {serial}: {e}")

    def _on_message(self, client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8", errors="ignore")
        try:
            payload = json.loads(payload_raw)
        except Exception:
            payload = payload_raw
        # type by last path segment
        topic_type = topic.rsplit("/", 1)[-1] if "/" in topic else topic
        message = {"type": f"mqtt_{topic_type}", "topic": topic, "payload": payload}
        try:
            import asyncio
            asyncio.run_coroutine_threadsafe(self.ws_broadcast(message), self.loop)
        except Exception as e:
            print(f"MQTTBridge forward failed: {e}")
        # 将 state 消息同步到后端实例状态
        try:
            if isinstance(payload, dict) and str(topic_type).lower() == "state":
                self._sink_state_into_manager(payload)
        except Exception:
            pass
