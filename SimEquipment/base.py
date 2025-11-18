from __future__ import annotations
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import paho.mqtt.client as mqtt

from SimVehicleSys.config.mqtt_config import generate_vda_mqtt_base_topic
from SimVehicleSys.mqtt.client import publish_json
from SimVehicleSys.utils.helpers import get_timestamp


@dataclass
class MqttConfig:
    host: str
    port: int
    vda_interface: str


@dataclass
class EquipmentIdentity:
    manufacturer: str
    serial_number: str
    vda_version: str
    vda_full_version: str


@dataclass
class EquipmentSettings:
    name: str
    ip: str
    site: Optional[str] = None
    map_id: Optional[str] = None
    state_frequency: int = 1
    action_time: float = 0.0
    trigger_mode: str = "instant"


@dataclass
class EquipmentConfig:
    mqtt: MqttConfig
    identity: EquipmentIdentity
    settings: EquipmentSettings


def load_config(dir_path: Path) -> EquipmentConfig:
    fp = dir_path / "config.json"
    data = json.loads(fp.read_text(encoding="utf-8"))
    mqtt = MqttConfig(
        host=str(data.get("mqtt", {}).get("host", "127.0.0.1")),
        port=int(str(data.get("mqtt", {}).get("port", 9527))),
        vda_interface=str(data.get("mqtt", {}).get("vda_interface", "uequip")),
    )
    ident = EquipmentIdentity(
        manufacturer=str(data.get("manufacturer", data.get("identity", {}).get("manufacturer", "SimEquip"))),
        serial_number=str(data.get("serial_number", data.get("identity", {}).get("serial_number", data.get("name", "Device")))),
        vda_version=str(data.get("vda_version", data.get("identity", {}).get("vda_version", "v2"))),
        vda_full_version=str(data.get("vda_full_version", data.get("identity", {}).get("vda_full_version", "2.0.0"))),
    )
    settings = EquipmentSettings(
        name=str(data.get("name", data.get("settings", {}).get("name", ident.serial_number))),
        ip=str(data.get("ip", data.get("settings", {}).get("ip", "0.0.0.0"))),
        site=(data.get("site") or data.get("settings", {}).get("site")),
        map_id=(data.get("map_id") or data.get("settings", {}).get("map_id") or "default"),
        state_frequency=int(data.get("state_frequency", data.get("settings", {}).get("state_frequency", 1))),
        action_time=float(data.get("action_time", data.get("settings", {}).get("action_time", 0.0))),
        trigger_mode=str(data.get("trigger_mode", data.get("settings", {}).get("trigger_mode", "instant"))),
    )
    return EquipmentConfig(mqtt=mqtt, identity=ident, settings=settings)


class EquipmentDevice:
    def __init__(self, cfg: EquipmentConfig) -> None:
        self.cfg = cfg
        base = generate_vda_mqtt_base_topic(
            cfg.mqtt.vda_interface,
            cfg.identity.vda_version,
            cfg.identity.manufacturer,
            cfg.identity.serial_number,
        )
        self.topic_connection = f"{base}/connection"
        self.topic_state = f"{base}/state"
        self.topic_factsheet = f"{base}/factsheet"
        self.topic_actions = f"{base}/instantActions"
        self.client: Optional[mqtt.Client] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = False
        self._header_id = 0

    def _build_connection(self, state: str) -> dict:
        return {
            "header_id": self._header_id,
            "timestamp": get_timestamp(),
            "version": self.cfg.identity.vda_full_version,
            "manufacturer": self.cfg.identity.manufacturer,
            "serial_number": self.cfg.identity.serial_number,
            "connection_state": state,
        }

    def _build_base_state(self) -> dict:
        return {
            "header_id": self._header_id,
            "timestamp": get_timestamp(),
            "version": self.cfg.identity.vda_full_version,
            "manufacturer": self.cfg.identity.manufacturer,
            "serial_number": self.cfg.identity.serial_number,
            "driving": False,
            "operating_mode": "MANUAL",
            "node_states": [],
            "edge_states": [],
            "last_node_id": str(self.cfg.settings.site or ""),
            "order_id": "",
            "order_update_id": 0,
            "last_node_sequence_id": 0,
            "action_states": [],
            "information": [],
            "loads": [],
            "battery_state": {"battery_charge": 100, "charging": False},
            "safety_state": {"e_stop": "NONE", "field_violation": False},
            "paused": False,
            "new_base_request": False,
            "agv_position": None,
            "velocity": None,
            "zone_set_id": str(self.cfg.settings.map_id or "default"),
            "waiting_for_interaction_zone_release": False,
        }

    def _build_factsheet(self, device_type: str) -> dict:
        return {
            "header_id": 0,
            "timestamp": get_timestamp(),
            "version": self.cfg.identity.vda_full_version,
            "manufacturer": self.cfg.identity.manufacturer,
            "serial_number": self.cfg.identity.serial_number,
            "typeSpecification": {
                "seriesName": self.cfg.settings.name,
                "seriesDescription": self.cfg.settings.name,
                "agvKinematic": "STEER",
                "agvClass": device_type,
                "maxLoadMass": 0.0,
                "localizationTypes": ["SLAM"],
                "navigationTypes": ["VIRTUAL_LINE_GUIDED"],
            },
            "physicalParameters": {
                "speedMin": 0.0,
                "speedMax": 0.0,
                "accelerationMax": 0.0,
                "decelerationMax": 0.0,
                "heightMin": 0.0,
                "heightMax": 0.0,
                "width": 1.0,
                "length": 1.0,
            },
            "protocolLimits": None,
            "protocolFeatures": None,
            "agvGeometry": None,
            "loadSpecification": None,
            "localizationParameters": None,
        }

    def _create_client(self) -> mqtt.Client:
        c = mqtt.Client(protocol=mqtt.MQTTv5)
        c.reconnect_delay_set(min_delay=1, max_delay=10)
        will_payload = json.dumps({
            "headerId": 999999999,
            "manufacturer": self.cfg.identity.manufacturer,
            "serialNumber": self.cfg.identity.serial_number,
            "timestamp": get_timestamp(),
            "version": self.cfg.identity.vda_full_version,
            "connectionState": "CONNECTIONBROKEN",
        }, ensure_ascii=False)
        c.will_set(self.topic_connection, payload=will_payload, qos=1, retain=False)
        c.on_message = self._on_message
        return c

    def _connect(self) -> None:
        self.client = self._create_client()
        self.client.connect(self.cfg.mqtt.host, int(self.cfg.mqtt.port), keepalive=60)
        self.client.loop_start()
        self.client.subscribe(self.topic_actions, qos=1)

    def _publish_connection_online(self) -> None:
        payload = self._build_connection("ONLINE")
        publish_json(self.client, self.topic_connection, payload, qos=1, retain=False)  # type: ignore[arg-type]

    def _publish_factsheet(self, device_type: str) -> None:
        payload = self._build_factsheet(device_type)
        publish_json(self.client, self.topic_factsheet, payload, qos=1, retain=True)  # type: ignore[arg-type]

    def _publish_state(self, state_payload: dict) -> None:
        self._header_id += 1
        state_payload["header_id"] = self._header_id
        state_payload["timestamp"] = get_timestamp()
        publish_json(self.client, self.topic_state, state_payload, qos=1, retain=False)  # type: ignore[arg-type]

    def _on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        try:
            data = json.loads(msg.payload.decode("utf-8", errors="ignore"))
        except Exception:
            return
        self._handle_instant_actions(data)

    def _handle_instant_actions(self, data: dict) -> None:
        pass

    def start(self) -> None:
        self._connect()
        self._header_id = 0
        self._publish_connection_online()
        self._start_thread()

    def stop(self) -> None:
        self._stop = True
        try:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
        except Exception:
            pass

    def _start_thread(self) -> None:
        self._stop = False
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self) -> None:
        while not self._stop:
            time.sleep(1.0)