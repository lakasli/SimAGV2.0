from __future__ import annotations
from SimVehicleSys.mqtt.client import generate_vda_mqtt_base_topic as _generate_base


def generate_vda_mqtt_base_topic(vda_interface: str, vda_version: str, manufacturer: str, serial_number: str) -> str:
    return _generate_base(vda_interface, vda_version, manufacturer, serial_number)