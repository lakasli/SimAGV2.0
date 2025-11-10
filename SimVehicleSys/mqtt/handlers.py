from __future__ import annotations
import threading
import time
from typing import List

import paho.mqtt.client as mqtt

from SimVehicleSys.config.settings import Config
from SimVehicleSys.mqtt.client import create_client, connect, publish_json
from SimVehicleSys.config.mqtt_config import generate_vda_mqtt_base_topic
from SimVehicleSys.utils.helpers import get_topic_type
from SimVehicleSys.sim_vehicle.vehicle import VehicleSimulator
from SimVehicleSys.sim_vehicle.battery_manager import BatteryManager
from SimVehicleSys.sim_vehicle.time_manager import SimClock
from SimVehicleSys.protocol.vda_2_0_0.order import Order
from SimVehicleSys.protocol.vda_2_0_0.instant_actions import InstantActions
from SimVehicleSys.protocol.vda_2_0_0.action import Action, ActionParameter, BlockingType


class MqttHandler:
    def __init__(self, config: Config, simulator: VehicleSimulator):
        self.config = config
        self.sim = simulator
        self.client = create_client(config)
        self.client.on_message = self._on_message
        self._battery_manager = BatteryManager(self.sim, self.config)
        self._clock = SimClock(self.sim, self.config, self._battery_manager, tick_ms=50)

    def start(self) -> None:
        connect(self.client, self.config)
        self.client.loop_start()
        base = generate_vda_mqtt_base_topic(
            self.config.mqtt_broker.vda_interface,
            self.config.vehicle.vda_version,
            self.config.vehicle.manufacturer,
            self.config.vehicle.serial_number,
        )
        topics = [f"{base}/order", f"{base}/instantActions", f"{base}/simConfig"]
        for t in topics:
            self.client.subscribe(t, qos=1)
        self.sim.publish_connection(self.client)
        self.sim.publish_factsheet(self.client)
        self._clock.start(self.client)

    def _publish_loop(self) -> None:
        tick_ms = 50
        state_elapsed_ms = 0.0
        visualization_elapsed_ms = 0.0
        while True:
            self.sim.update_state()
            scale = max(0.0001, float(self.config.settings.sim_time_scale))
            eff_state_freq = max(1e-6, float(self.config.settings.state_frequency) * scale)
            eff_vis_freq = max(1e-6, float(self.config.settings.visualization_frequency) * scale)
            state_elapsed_ms += tick_ms
            if state_elapsed_ms >= (1000.0 / eff_state_freq):
                state_elapsed_ms = 0.0
                self.sim.publish_state(self.client)
            visualization_elapsed_ms += tick_ms
            if visualization_elapsed_ms >= (1000.0 / eff_vis_freq):
                visualization_elapsed_ms = 0.0
                self.sim.publish_visualization(self.client)
            time.sleep(tick_ms / 1000.0)

    def _parse_instant_actions(self, data: dict) -> InstantActions:
        def g(d, snake, camel, default=None):
            return d.get(snake, d.get(camel, default))
        actions_raw = g(data, "actions", "actions", []) or []
        actions: List[Action] = []
        for a in actions_raw:
            at = g(a, "action_type", "actionType", "")
            aid = g(a, "action_id", "actionId", "")
            bt = g(a, "blocking_type", "blockingType", "NONE")
            params_raw = g(a, "action_parameters", "actionParameters", []) or []
            params = [ActionParameter(key=g(p, "key", "key", ""), value=g(p, "value", "value", None)) for p in params_raw]
            try:
                bt_enum = BlockingType(bt)
            except Exception:
                bt_enum = BlockingType.None_
            actions.append(Action(action_type=at, action_id=aid, blocking_type=bt_enum, action_parameters=params))
        return InstantActions(
            header_id=g(data, "header_id", "headerId", 0),
            # 支持 'timestamp' 与 'timeStamp' 两种键名
            timestamp=(data.get("timestamp", data.get("timeStamp", ""))),
            version=g(data, "version", "version", self.config.vehicle.vda_full_version),
            manufacturer=g(data, "manufacturer", "manufacturer", self.config.vehicle.manufacturer),
            serial_number=g(data, "serial_number", "serialNumber", self.config.vehicle.serial_number),
            actions=actions,
        )

    def _apply_settings_update(self, data: dict) -> None:
        # 支持 snake_case 与 camelCase 键名
        def g(d, snake, camel, default=None):
            return d.get(snake, d.get(camel, default))
        s = self.config.settings
        try:
            v = g(data, "speed", "speed")
            if v is not None:
                s.speed = float(v)
            v = g(data, "sim_time_scale", "simTimeScale")
            if v is not None:
                s.sim_time_scale = float(v)
            v = g(data, "state_frequency", "stateFrequency")
            if v is not None:
                s.state_frequency = int(v)
            v = g(data, "visualization_frequency", "visualizationFrequency")
            if v is not None:
                s.visualization_frequency = int(v)
            v = g(data, "action_time", "actionTime")
            if v is not None:
                s.action_time = float(v)
            v = g(data, "map_id", "mapId")
            if v is not None:
                s.map_id = str(v)
            v = g(data, "battery_default", "batteryDefault")
            if v is not None:
                s.battery_default = float(v)
            v = g(data, "battery_idle_drain_per_min", "batteryIdleDrainPerMin")
            if v is not None:
                s.battery_idle_drain_per_min = float(v)
            v = g(data, "battery_move_empty_multiplier", "batteryMoveEmptyMultiplier")
            if v is not None:
                s.battery_move_empty_multiplier = float(v)
            v = g(data, "battery_move_loaded_multiplier", "batteryMoveLoadedMultiplier")
            if v is not None:
                s.battery_move_loaded_multiplier = float(v)
            v = g(data, "battery_charge_per_min", "batteryChargePerMin")
            if v is not None:
                s.battery_charge_per_min = float(v)
            v = g(data, "frontend_poll_interval_ms", "frontendPollIntervalMs")
            if v is not None:
                s.frontend_poll_interval_ms = int(v)
            print(
                "[SimVehicleSys] Settings hot-updated:",
                {
                    "speed": s.speed,
                    "sim_time_scale": s.sim_time_scale,
                    "state_frequency": s.state_frequency,
                    "visualization_frequency": s.visualization_frequency,
                    "action_time": s.action_time,
                    "map_id": s.map_id,
                },
            )
        except Exception as e:
            print(f"Apply settings update failed: {e}")

    def _on_message(self, client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="ignore")
        topic_type = get_topic_type(topic)
        try:
            import json
            data = json.loads(payload)
        except Exception as e:
            print(f"Invalid JSON on {topic}: {e}")
            return
        if topic_type == "order":
            try:
                # 将 snake_case 键统一转换为 camelCase，以适配仅支持 camelCase 的解析器
                def _snake_to_camel_key(k: str) -> str:
                    if "_" not in k:
                        return k
                    parts = k.split("_")
                    return parts[0] + "".join(p[:1].upper() + p[1:] for p in parts[1:])

                def _snake_to_camel(obj):
                    if isinstance(obj, list):
                        return [_snake_to_camel(x) for x in obj]
                    if not isinstance(obj, dict):
                        return obj
                    out = {}
                    for kk, vv in obj.items():
                        nk = _snake_to_camel_key(str(kk))
                        out[nk] = _snake_to_camel(vv)
                    return out

                # 委托给订单管理模块进行解析、校验和分发
                from SimVehicleSys.sim_vehicle.order_manager import process_order as _process_order
                _process_order(self.sim, _snake_to_camel(data))
            except Exception as e:
                print(f"Error processing order: {e}")
        elif topic_type == "instantActions":
            try:
                ia = self._parse_instant_actions(data)
                self.sim.accept_instant_actions(ia)
            except Exception as e:
                print(f"Error parsing instantActions: {e}")
        elif topic_type == "simConfig":
            try:
                self._apply_settings_update(data)
            except Exception as e:
                print(f"Error applying simConfig: {e}")
        else:
            print(f"Unknown topic type: {topic_type}")