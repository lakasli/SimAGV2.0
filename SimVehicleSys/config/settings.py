from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import pathlib


@dataclass
class MqttBrokerConfig:
    host: str
    port: str
    vda_interface: str


@dataclass
class VehicleConfig:
    manufacturer: str
    serial_number: str
    vda_version: str
    vda_full_version: str


@dataclass
class Settings:
    action_time: float
    speed: float
    robot_count: int
    state_frequency: int
    visualization_frequency: int
    map_id: str
    sim_time_scale: float
    battery_default: float
    battery_idle_drain_per_min: float
    battery_move_empty_multiplier: float
    battery_move_loaded_multiplier: float
    battery_charge_per_min: float
    frontend_poll_interval_ms: int


@dataclass
class Config:
    mqtt_broker: MqttBrokerConfig
    vehicle: VehicleConfig
    settings: Settings


def _project_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[2]


def get_config(config_path: Optional[str] = None) -> Config:
    # 如需覆盖配置，可在调用处对返回对象做修改（例如覆盖 serial_number）。
    return Config(
        mqtt_broker=MqttBrokerConfig(
            host="127.0.0.1",
            port="1883",
            vda_interface="uagv",
        ),
        vehicle=VehicleConfig(
            manufacturer="SEER",
            serial_number="AMB-01",
            vda_version="v2",
            vda_full_version="v2",
        ),
        settings=Settings(
            action_time=1.0,
            speed=0.05,
            robot_count=1,
            state_frequency=1,
            visualization_frequency=1,
            map_id="default",
            sim_time_scale=1.0,
            battery_default=100.0,
            battery_idle_drain_per_min=1.0,
            battery_move_empty_multiplier=1.5,
            battery_move_loaded_multiplier=2.5,
            battery_charge_per_min=8.0,
            frontend_poll_interval_ms=1000,
        ),
    )