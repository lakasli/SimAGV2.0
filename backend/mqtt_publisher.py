from __future__ import annotations
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import paho.mqtt.client as mqtt
from SimVehicleSys.config.settings import get_config

from .schemas import AGVInfo, AGVRuntime
from .schemas import SimSettingsPatch


def _timestamp() -> str:
    # YYYY-MM-DDTHH:mm:ss.sssZ
    now = datetime.now(timezone.utc)
    return now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


class MqttPublisher:
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.client: Optional[mqtt.Client] = None
        self.host: str = "127.0.0.1"
        self.port: int = 1883
        self.vda_interface: str = "uagv"
        self.vda_full_version: str = "2.0.0"
        self._load_config()

    def _load_config(self) -> None:
        # 改为从 SimVehicleSys.config.settings 加载默认配置
        try:
            cfg = get_config()
            self.host = str(cfg.mqtt_broker.host or self.host)
            # cfg.mqtt_broker.port 存储为字符串；转换为 int
            try:
                self.port = int(str(cfg.mqtt_broker.port or self.port))
            except Exception:
                pass
            self.vda_interface = str(cfg.mqtt_broker.vda_interface or self.vda_interface)
            self.vda_full_version = str(cfg.vehicle.vda_full_version or self.vda_full_version)
        except Exception:
            pass

    def start(self) -> None:
        self.client = mqtt.Client(client_id=str(uuid.uuid4()), protocol=mqtt.MQTTv5)
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        try:
            self.client.connect(self.host, self.port, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            print(f"MqttPublisher connect failed: {e}")
            # 确保失败时不可用
            try:
                self.client.loop_stop()
            except Exception:
                pass
            self.client = None

    def stop(self) -> None:
        try:
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
        except Exception:
            pass

    def publish_init_position(self, info: AGVInfo, rt: AGVRuntime) -> None:
        if not self.client:
            raise RuntimeError("MQTT publisher not connected")
        base = f"{self.vda_interface}/{info.vda_version}/{info.manufacturer}/{info.serial_number}"
        topic = f"{base}/instantActions"
        action_id = f"backend-move-{uuid.uuid4()}"
        # VDA5050 camelCase payload
        payload = {
            "headerId": 0,
            "timestamp": _timestamp(),
            "version": self.vda_full_version,
            "manufacturer": info.manufacturer,
            "serialNumber": info.serial_number,
            "actions": [
                {
                    "actionType": "initPosition",
                    "actionId": action_id,
                    "blockingType": "SOFT",
                    "actionParameters": [
                        {"key": "x", "value": rt.position.x},
                        {"key": "y", "value": rt.position.y},
                        {"key": "theta", "value": rt.position.theta},
                        {"key": "mapId", "value": rt.current_map or "default"},
                    ],
                }
            ],
        }
        try:
            info_obj = self.client.publish(topic, json.dumps(payload), qos=1, retain=False)
            rc = getattr(info_obj, "rc", mqtt.MQTT_ERR_SUCCESS)
            if rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish initPosition failed rc={rc}")
        except Exception as e:
            raise

    def publish_start_station_navigation(self, info: AGVInfo, station_id: str, map_id: Optional[str]) -> None:
        if not self.client:
            raise RuntimeError("MQTT publisher not connected")
        base = f"{self.vda_interface}/{info.vda_version}/{info.manufacturer}/{info.serial_number}"
        topic = f"{base}/instantActions"
        action_id = f"backend-nav-{uuid.uuid4()}"
        payload = {
            "headerId": 0,
            "timestamp": _timestamp(),
            "version": self.vda_full_version,
            "manufacturer": info.manufacturer,
            "serialNumber": info.serial_number,
            "actions": [
                {
                    "actionType": "startStationNavigation",
                    "actionId": action_id,
                    "blockingType": "SOFT",
                    "actionParameters": [
                        {"key": "stationId", "value": station_id},
                        {"key": "mapId", "value": map_id or ""},
                    ],
                }
            ],
        }
        try:
            info_obj = self.client.publish(topic, json.dumps(payload), qos=1, retain=False)
            rc = getattr(info_obj, "rc", mqtt.MQTT_ERR_SUCCESS)
            if rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish startStationNavigation failed rc={rc}")
        except Exception as e:
            raise

    def publish_order(self, info: AGVInfo, order_payload: dict) -> None:
        """发布 VDA5050 订单到 MQTT `.../order` 主题。

        `order_payload` 应为蛇形命名的 VDA 2.0 订单结构，本方法不转换字段命名。
        """
        if not self.client:
            raise RuntimeError("MQTT publisher not connected")
        base = f"{self.vda_interface}/{info.vda_version}/{info.manufacturer}/{info.serial_number}"
        topic = f"{base}/order"
        try:
            info_obj = self.client.publish(topic, json.dumps(order_payload), qos=1, retain=False)
            rc = getattr(info_obj, "rc", mqtt.MQTT_ERR_SUCCESS)
            if rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish order failed rc={rc}")
        except Exception as e:
            raise

    def publish_sim_settings(self, info: AGVInfo, patch: SimSettingsPatch | dict) -> None:
        """发布仿真设置热更新到 MQTT `.../simConfig` 主题。

        载荷为局部更新（仅包含需更新的键）。支持 snake_case（推荐）。
        """
        if not self.client:
            raise RuntimeError("MQTT publisher not connected")
        base = f"{self.vda_interface}/{info.vda_version}/{info.manufacturer}/{info.serial_number}"
        topic = f"{base}/simConfig"
        payload = patch if isinstance(patch, dict) else patch.dict(exclude_none=True)
        try:
            info_obj = self.client.publish(topic, json.dumps(payload), qos=1, retain=False)
            rc = getattr(info_obj, "rc", mqtt.MQTT_ERR_SUCCESS)
            if rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish simConfig failed rc={rc}")
        except Exception as e:
            raise