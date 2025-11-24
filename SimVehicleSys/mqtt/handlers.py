from __future__ import annotations
import threading
import time
import math
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
from SimVehicleSys.sim_vehicle.state_manager import global_store


class MqttHandler:
    def __init__(self, config: Config, simulator: VehicleSimulator):
        self.config = config
        self.sim = simulator
        self.client = create_client(config)
        self.client.on_message = self._on_message
        self._battery_manager = BatteryManager(self.sim, self.config)
        self._clock = SimClock(self.sim, self.config, self._battery_manager, tick_ms=50)
        self._collision_subscriber = None
        

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
        try:
            self._start_collision_subscriber()
        except Exception:
            pass
        

    def _publish_loop(self) -> None:
        tick_ms = 50
        state_elapsed_ms = 0.0
        visualization_elapsed_ms = 0.0
        while True:
            self.sim.update_state()
            scale = max(0.0001, float(self.config.settings.sim_time_scale))
            eff_state_freq = max(1e-6, float(self.config.settings.state_frequency) * scale)
            eff_vis_freq = eff_state_freq
            state_elapsed_ms += tick_ms
            if state_elapsed_ms >= (1000.0 / eff_state_freq):
                state_elapsed_ms = 0.0
                self.sim.publish_state(self.client)
            visualization_elapsed_ms += tick_ms
            if visualization_elapsed_ms >= (1000.0 / eff_vis_freq):
                visualization_elapsed_ms = 0.0
                self.sim.publish_visualization(self.client)
            time.sleep(tick_ms / 1000.0)

    # --- Collision subscriber ---
    def _start_collision_subscriber(self) -> None:
        import threading
        import paho.mqtt.client as mqtt
        import uuid
        import math
        cfg = self.config
        vda = str(cfg.mqtt_broker.vda_interface)
        ver = str(cfg.vehicle.vda_version)
        manu = str(cfg.vehicle.manufacturer)
        self_serial = str(cfg.vehicle.serial_number)
        topic = f"{vda}/{ver}/+/+/state"
        cli = mqtt.Client(client_id=str(uuid.uuid4()), protocol=mqtt.MQTTv5)
        cli.reconnect_delay_set(min_delay=1, max_delay=10)
        cli.connect(str(cfg.mqtt_broker.host), int(str(cfg.mqtt_broker.port)), keepalive=60)

        def _on_msg(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
            try:
                payload = msg.payload.decode("utf-8", errors="ignore")
                import json
                data = json.loads(payload)
            except Exception:
                return
            try:
                parts = msg.topic.split("/")
                sn = parts[-2] if len(parts) >= 2 else ""
            except Exception:
                sn = ""
            if sn == self_serial:
                return
            try:
                ap = data.get("agvPosition") or data.get("agv_position") or {}
                x = float(ap.get("x", 0.0))
                y = float(ap.get("y", 0.0))
                theta = float(ap.get("theta", 0.0))
                mp = str(ap.get("mapId", ap.get("map_id", cfg.settings.map_id)))
            except Exception:
                return
            try:
                loads = data.get("loads") or []
                max_l = 0.0
                max_w = 0.0
                for ld in (loads or []):
                    try:
                        dim = ld.get("loadDimensions") or ld.get("load_dimensions") or {}
                        ll = float(dim.get("length", 0.0)) if dim is not None else 0.0
                        ww = float(dim.get("width", 0.0)) if dim is not None else 0.0
                        max_l = max(max_l, ll)
                        max_w = max(max_w, ww)
                    except Exception:
                        pass
            except Exception:
                max_l = 0.0
                max_w = 0.0
            try:
                try:
                    from SimVehicleSys.sim_vehicle.vehicle import VehicleSimulator
                    w = float(getattr(getattr(self.sim, "config", None).settings, "width", cfg.settings.width))
                    l = float(getattr(getattr(self.sim, "config", None).settings, "length", cfg.settings.length))
                    off = float(getattr(getattr(self.sim, "config", None).settings, "center_forward_offset_m", cfg.settings.center_forward_offset_m))
                except Exception:
                    w = float(cfg.settings.width)
                    l = float(cfg.settings.length)
                    off = float(cfg.settings.center_forward_offset_m)
                s_len = 1.1 * max(l, max_l)
                s_wid = 1.1 * max(w, max_w)
                cx = x - math.cos(theta) * off
                cy = y - math.sin(theta) * off
                env = {"center": {"x": cx, "y": cy}, "length": s_len, "width": s_wid, "theta": theta, "mapId": mp}
                try:
                    if not hasattr(self.sim, "_neighbors") or self.sim._neighbors is None:
                        self.sim._neighbors = {}
                    self.sim._neighbors[str(sn)] = env
                except Exception:
                    pass
            except Exception:
                pass

        cli.on_message = _on_msg
        cli.subscribe(topic, qos=1)
        t = threading.Thread(target=cli.loop_forever, daemon=True)
        t.start()
        self._collision_subscriber = cli

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
            # 物理参数：速度上下限与加减速（来自 factsheet.physicalParameters）
            v = g(data, "speed_min", "speedMin")
            if v is not None:
                s.speed_min = float(v)
            v = g(data, "speed_max", "speedMax")
            if v is not None:
                s.speed_max = float(v)
            v = g(data, "acceleration_max", "accelerationMax")
            if v is not None:
                s.acceleration_max = float(v)
            v = g(data, "deceleration_max", "decelerationMax")
            if v is not None:
                s.deceleration_max = float(v)
            v = g(data, "height_min", "heightMin")
            if v is not None:
                s.height_min = float(v)
            v = g(data, "height_max", "heightMax")
            if v is not None:
                s.height_max = float(v)
            v = g(data, "width", "width")
            if v is not None:
                s.width = float(v)
            v = g(data, "length", "length")
            if v is not None:
                s.length = float(v)
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
            # 将 speed 限制在 [speed_min, speed_max] 范围内
            try:
                if s.speed_min is not None and s.speed_max is not None:
                    s.speed = max(float(s.speed_min), min(float(s.speed), float(s.speed_max)))
            except Exception:
                pass
            # 根据加减速上限估算加速/减速距离（v^2 = 2*a*s -> s = v^2/(2*a)）
            try:
                v_eff = float(max(0.0, s.speed))
                a_eff = float(max(1e-6, s.acceleration_max))
                d_eff = float(max(1e-6, s.deceleration_max))
                accel_m = float(v_eff * v_eff) / (2.0 * a_eff)
                decel_m = float(v_eff * v_eff) / (2.0 * d_eff)
                # 更新仿真车的起步/终点减速距离
                if hasattr(self, "sim") and self.sim:
                    try:
                        self.sim.nav_accel_distance_m = max(0.0, accel_m)
                        self.sim.nav_decel_distance_m = max(0.0, decel_m)
                    except Exception:
                        pass
            except Exception:
                pass
            print(
                "[SimVehicleSys] Settings hot-updated:",
                {
                    "speed": s.speed,
                    "speed_min": getattr(s, "speed_min", None),
                    "speed_max": getattr(s, "speed_max", None),
                    "acceleration_max": getattr(s, "acceleration_max", None),
                    "deceleration_max": getattr(s, "deceleration_max", None),
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

