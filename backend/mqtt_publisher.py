from __future__ import annotations
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import paho.mqtt.client as mqtt
from SimVehicleSys.config.settings import get_config
from SimVehicleSys.utils.helpers import canonicalize_map_id

from .schemas import AGVInfo, AGVRuntime
from .schemas import SimSettingsPatch
from typing import Optional as _Opt


def _timestamp() -> str:
    # YYYY-MM-DDTHH:mm:ss.sssZ
    now = datetime.now(timezone.utc)
    return now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


class MqttPublisher:
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.client: Optional[mqtt.Client] = None
        self.host: str = "127.0.0.1"
        self.port: int = 9527
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
        # 尝试解析提交坐标所在地图的最近站点，并取其 points.name 作为 lastNodeId
        last_node_label: _Opt[str] = None
        try:
            from SimVehicleSys.sim_vehicle.navigation import (
                resolve_scene_path,
                parse_scene_topology,
                nearest_station,
                find_point_name_by_id,
            )
            map_id = str(rt.current_map or "")
            if map_id:
                fp = resolve_scene_path(map_id)
                topo = parse_scene_topology(fp)
                stations = topo.get("stations") or []
                try:
                    x = float(getattr(rt.position, "x", 0.0))
                    y = float(getattr(rt.position, "y", 0.0))
                except Exception:
                    x, y = 0.0, 0.0
                sid = nearest_station((x, y), stations)  # 返回最近站点的 id（string）
                if sid:
                    # 将 id 映射为 .scene 中的 name；若失败则退回 id 字符串
                    label = find_point_name_by_id(fp, str(sid))
                    last_node_label = label or str(sid)
        except Exception:
            # 地图解析失败时忽略，不影响 initPosition 发布
            last_node_label = None
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
                    "blockingType": "HARD",
                    "actionParameters": [
                        {"key": "x", "value": rt.position.x},
                        {"key": "y", "value": rt.position.y},
                        {"key": "theta", "value": rt.position.theta},
                        # mapId 仅为 VehicleMap 下的 scene 文件名（不含扩展名）
                        {"key": "mapId", "value": (canonicalize_map_id(getattr(rt, "current_map", None)))},
                        # 将 lastNodeId 作为额外参数下发，车辆在处理 initPosition 时会直接更新 state.last_node_id
                        *(([{"key": "lastNodeId", "value": last_node_label}] ) if last_node_label else [])
                    ],
                }
            ],
        }
        try:
            # 使用 retain=True，保证后续新连接的仿真实例也能收到并应用初始位置
            info_obj = self.client.publish(topic, json.dumps(payload), qos=1, retain=True)
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

        `order_payload` 应为 camelCase 的 VDA 2.0 订单结构；本方法不转换字段命名。
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
            # 使用 retain=True，保证连接后的实例能获取最近一次仿真设置
            info_obj = self.client.publish(topic, json.dumps(payload), qos=1, retain=True)
            rc = getattr(info_obj, "rc", mqtt.MQTT_ERR_SUCCESS)
            if rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish simConfig failed rc={rc}")
        except Exception as e:
            raise

    def publish_switch_map(self, info: AGVInfo, *, map_name: str, switch_point: Optional[str] = None, center_x: Optional[float] = None, center_y: Optional[float] = None, initiate_angle: Optional[float] = None) -> None:
        """发布地图切换即时动作到 MQTT `.../instantActions` 主题。

        参数名与设备端约定保持一致：map、switchPoint、center_x、center_y、initiate_angle。
        """
        if not self.client:
            raise RuntimeError("MQTT publisher not connected")
        base = f"{self.vda_interface}/{info.vda_version}/{info.manufacturer}/{info.serial_number}"
        topic = f"{base}/instantActions"
        action_id = f"backend-switchmap-{uuid.uuid4()}"
        params = [{"key": "map", "value": map_name}]
        if switch_point:
            params.append({"key": "switchPoint", "value": switch_point})
        if center_x is not None:
            params.append({"key": "center_x", "value": center_x})
        if center_y is not None:
            params.append({"key": "center_y", "value": center_y})
        if initiate_angle is not None:
            params.append({"key": "initiate_angle", "value": initiate_angle})
        payload = {
            "headerId": 0,
            "timestamp": _timestamp(),
            "version": self.vda_full_version,
            "manufacturer": info.manufacturer,
            "serialNumber": info.serial_number,
            "actions": [
                {
                    "actionType": "switchMap",
                    "actionId": action_id,
                    "actionDescription": "Manual switchMap",
                    "blockingType": "HARD",
                    "actionParameters": params,
                }
            ],
        }
        try:
            info_obj = self.client.publish(topic, json.dumps(payload), qos=1, retain=False)
            rc = getattr(info_obj, "rc", mqtt.MQTT_ERR_SUCCESS)
            if rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"MQTT publish switchMap failed rc={rc}")
        except Exception:
            raise