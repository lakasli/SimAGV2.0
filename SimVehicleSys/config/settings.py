from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import pathlib
import json
import os


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
    # 物理参数（factsheet.physicalParameters）：用于约束运动控制与碰撞几何
    speed_min: float
    speed_max: float
    acceleration_max: float
    deceleration_max: float
    height_min: float
    height_max: float
    width: float
    length: float
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
    # 前向中心偏移量（米）：几何中心沿车头方向平移，用于安全范围计算/渲染
    center_forward_offset_m: float


@dataclass
class Config:
    mqtt_broker: MqttBrokerConfig
    vehicle: VehicleConfig
    settings: Settings


def _project_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[2]


def get_config(config_path: Optional[str] = None) -> Config:
    # 如需覆盖配置，可在调用处对返回对象做修改（例如覆盖 serial_number）。
    # 尝试读取项目根目录 factsheet.json 的物理参数作为默认值
    pr = _project_root()
    fp_defaults = {
        "speedMin": 0.01,
        "speedMax": 2.0,
        "accelerationMax": 2,
        "decelerationMax": 2,
        "heightMin": 0.01,
        "heightMax": 0.10,
        "width": 0.745,
        "length": 1.03,
    }
    try:
        fs_path = pr / "factsheet.json"
        if fs_path.exists():
            with fs_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            p = (data or {}).get("physicalParameters") or {}
            for k in fp_defaults.keys():
                try:
                    v = p.get(k)
                    if v is not None:
                        fp_defaults[k] = float(v)
                except Exception:
                    pass
    except Exception:
        # 读取失败则使用内置默认值
        pass
    cfg = Config(
        mqtt_broker=MqttBrokerConfig(
            host=os.getenv("SIMAGV_MQTT_HOST", "127.0.0.1"),
            port=os.getenv("SIMAGV_MQTT_PORT", "1884"),
            vda_interface=os.getenv("SIMAGV_MQTT_INTERFACE", "uagv"),
        ),
        vehicle=VehicleConfig(
            manufacturer="SEER",
            serial_number="AMB-",
            vda_version="v2",
            vda_full_version="2.0.0",
        ),
        settings=Settings(
            action_time=1.0,
            # 默认运行速度以 factsheet 的最大速度为准
            speed=float(fp_defaults["speedMax"]),
            speed_min=float(fp_defaults["speedMin"]),
            speed_max=float(fp_defaults["speedMax"]),
            acceleration_max=float(fp_defaults["accelerationMax"]),
            deceleration_max=float(fp_defaults["decelerationMax"]),
            height_min=float(fp_defaults["heightMin"]),
            height_max=float(fp_defaults["heightMax"]),
            width=float(fp_defaults["width"]),
            length=float(fp_defaults["length"]),
            state_frequency=10,
            visualization_frequency=1,
            map_id="default",
            sim_time_scale=1.0,
            battery_default=100.0,
            battery_idle_drain_per_min=1.0,
            battery_move_empty_multiplier=1.5,
            battery_move_loaded_multiplier=2.5,
            battery_charge_per_min=10.0,
            frontend_poll_interval_ms=1000,
            center_forward_offset_m=0.1,
        ),
    )

    # 环境变量覆盖：允许通过 SIMAGV_MQTT_HOST / SIMAGV_MQTT_PORT / SIMAGV_MQTT_INTERFACE 覆盖默认值
    try:
        env_host = os.getenv("SIMAGV_MQTT_HOST")
        if env_host:
            cfg.mqtt_broker.host = str(env_host)
        env_port = os.getenv("SIMAGV_MQTT_PORT")
        if env_port:
            cfg.mqtt_broker.port = str(env_port)
        env_iface = os.getenv("SIMAGV_MQTT_INTERFACE")
        if env_iface:
            cfg.mqtt_broker.vda_interface = str(env_iface)
        # 车辆身份覆盖：SIMAGV_MANUFACTURER / SIMAGV_SERIAL
        env_manu = os.getenv("SIMAGV_MANUFACTURER")
        if env_manu:
            cfg.vehicle.manufacturer = str(env_manu)
        env_serial = os.getenv("SIMAGV_SERIAL")
        if env_serial:
            cfg.vehicle.serial_number = str(env_serial)
    except Exception:
        # 保守处理：忽略环境读取错误
        pass

    return cfg