from __future__ import annotations
import json
import uuid
from typing import Any

import paho.mqtt.client as mqtt

from SimVehicleSys.utils.helpers import to_camel_json


def generate_vda_mqtt_base_topic(vda_interface: str, vda_version: str, manufacturer: str, serial_number: str) -> str:
    return f"{vda_interface}/{vda_version}/{manufacturer}/{serial_number}"


def create_client(config: Any) -> mqtt.Client:
    client_id = str(uuid.uuid4())
    client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)
    client.reconnect_delay_set(min_delay=1, max_delay=10)
    try:
        setup_connection_will(client, config)
    except Exception:
        pass
    return client


def connect(client: mqtt.Client, config: Any) -> None:
    host = getattr(config.mqtt_broker, "host", "127.0.0.1")
    port = int(getattr(config.mqtt_broker, "port", 1884))
    client.connect(str(host), int(port), keepalive=60)


def publish_json(client: mqtt.Client, topic: str, obj: Any, qos: int = 1, retain: bool = False) -> None:
    payload = to_camel_json(obj)
    client.publish(topic, payload, qos=qos, retain=retain)


def setup_connection_will(client: mqtt.Client, config: Any) -> None:
    """
    设置 MQTT 遗嘱消息为 connectionState=CONNECTIONBROKEN（设备断线时由 broker 发布）。
    """
    try:
        base = generate_vda_mqtt_base_topic(
            config.mqtt_broker.vda_interface,
            config.vehicle.vda_version,
            config.vehicle.manufacturer,
            config.vehicle.serial_number,
        )
        topic = f"{base}/connection"
        will_payload = {
            "headerId": 999999999,
            "manufacturer": str(getattr(config.vehicle, "manufacturer", "")),
            "serialNumber": str(getattr(config.vehicle, "serial_number", "")),
            "timestamp": _iso_now(),
            "version": str(getattr(config.vehicle, "vda_full_version", "2.0.0")),
            "connectionState": "CONNECTIONBROKEN",
        }
        payload = json.dumps(will_payload, ensure_ascii=False)
        client.will_set(topic, payload=payload, qos=1, retain=False)
    except Exception:
        pass


def _iso_now() -> str:
    import datetime as _dt
    now = _dt.datetime.utcnow()
    return now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"