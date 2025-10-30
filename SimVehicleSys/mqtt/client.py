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
    return client


def connect(client: mqtt.Client, config: Any) -> None:
    host = getattr(config.mqtt_broker, "host", "127.0.0.1")
    port = int(getattr(config.mqtt_broker, "port", 1883))
    client.connect(str(host), int(port), keepalive=60)


def publish_json(client: mqtt.Client, topic: str, obj: Any, qos: int = 1, retain: bool = False) -> None:
    payload = to_camel_json(obj)
    client.publish(topic, payload, qos=qos, retain=retain)