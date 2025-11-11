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
from SimVehicleSys.world_manager import (
    computeSafetyRectForVehicle,
    computeSafetyRectForStatePayload,
    rects_overlap,
)
from SimVehicleSys.sim_vehicle.state_manager import global_store


class MqttHandler:
    def __init__(self, config: Config, simulator: VehicleSimulator):
        self.config = config
        self.sim = simulator
        self.client = create_client(config)
        self.client.on_message = self._on_message
        self._battery_manager = BatteryManager(self.sim, self.config)
        self._clock = SimClock(self.sim, self.config, self._battery_manager, tick_ms=50)
        # 对等状态缓存：接收其他 AGV 的 MQTT state
        self._peer_states_lock = threading.Lock()
        self._peer_states: dict[str, dict] = {}
        # 碰撞检测线程控制
        self._collision_thread: threading.Thread | None = None
        self._collision_stop = False
        # 由本地碰撞检测触发的暂停标记，仅用于恢复判定，避免干扰其它暂停来源
        self._collision_paused: bool = False

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
        # 额外订阅所有设备的 state，用于多车碰撞检测
        try:
            vda = str(self.config.mqtt_broker.vda_interface or "uagv")
            # uagv/{vdaVersion}/{manufacturer}/{serial}/state
            # 正确的通配符为 3 个 '+': vdaVersion/manufacturer/serial
            self.client.subscribe(f"{vda}/+/+/+/state", qos=1)
        except Exception:
            pass
        self.sim.publish_connection(self.client)
        self.sim.publish_factsheet(self.client)
        self._clock.start(self.client)
        # 启动后台碰撞检测循环
        try:
            self._start_collision_loop()
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
        elif topic_type == "state":
            # 吸收其他设备的状态（排除自身），用于碰撞检测
            try:
                serial = str(data.get("serial_number", data.get("serialNumber", "")) or "")
                if serial and serial != str(self.config.vehicle.serial_number):
                    with self._peer_states_lock:
                        self._peer_states[serial] = data
            except Exception:
                pass
        else:
            print(f"Unknown topic type: {topic_type}")

    # --- Collision monitoring ---
    def _start_collision_loop(self) -> None:
        if self._collision_thread and self._collision_thread.is_alive():
            return
        self._collision_stop = False
        self._collision_thread = threading.Thread(target=self._collision_loop, daemon=True)
        self._collision_thread.start()

    def _stop_collision_loop(self) -> None:
        self._collision_stop = True

    def _collision_loop(self) -> None:
        # 基础碰撞检测步长（毫秒）；实际休眠时间将按 sim_time_scale 缩放
        base_tick_ms = 100
        # 默认底盘尺寸：支持运行时覆盖
        try:
            length_default = float(getattr(self.config.settings, "length", 1.03))
            width_default = float(getattr(self.config.settings, "width", 0.745))
        except Exception:
            length_default, width_default = 1.03, 0.745
        while not self._collision_stop:
            # 计算基于时间缩放的有效睡眠秒数：scale 越大，检测越频繁
            eff_sleep_s = base_tick_ms / 1000.0
            try:
                scale = max(0.0001, float(getattr(self.config.settings, "sim_time_scale", 1.0)))
                eff_sleep_s = (base_tick_ms / scale) / 1000.0
            except Exception:
                pass
            try:
                # 自身安全包围盒
                my_rect = None
                try:
                    my_rect = computeSafetyRectForVehicle(self.sim, safeFactor=1.05)
                except Exception:
                    my_rect = None
                if my_rect is None:
                    time.sleep(eff_sleep_s)
                    continue
                # 拷贝对等状态快照，避免长锁
                with self._peer_states_lock:
                    peers = list(self._peer_states.items())
                any_overlap = False
                for peer_serial, peer_state in peers:
                    try:
                        peer_rect = computeSafetyRectForStatePayload(peer_state, length_default=length_default, width_default=width_default, safeFactor=1.05)
                        if not peer_rect:
                            continue
                        if rects_overlap(my_rect, peer_rect):
                            any_overlap = True
                            # 触发本机急停与错误上报（54231）
                            try:
                                st = getattr(self.sim, "state", None)
                                if st:
                                    st.driving = False
                                setattr(self.sim, "nav_paused", True)
                                # 标记由碰撞导致的暂停，用于后续自动恢复
                                self._collision_paused = True
                            except Exception:
                                pass
                            try:
                                payload_err = {
                                    "code": 54231,
                                    "level": "Warning",
                                    "type": "Navigation",
                                    "reason": "CollisionOverlapSafety",
                                    "message": "Caution: robot is blocked",
                                    "with": peer_serial,
                                    "descriptionCN": "注意机器人被阻挡",
                                }
                                my_serial = str(self.config.vehicle.serial_number)
                                global_store.set_error(my_serial, payload_err)
                            except Exception:
                                pass
                    except Exception:
                        pass
                # 恢复逻辑：若此前因碰撞暂停且当前不存在任何重叠，则恢复为可行驶/可接单
                try:
                    if self._collision_paused and not any_overlap:
                        # 清除暂停标记，仅恢复由碰撞导致的暂停
                        self._collision_paused = False
                        try:
                            self.sim.nav_paused = False
                            self.sim.state.paused = False
                        except Exception:
                            pass
                        # 若导航仍在运行，则恢复 driving；否则保持 driving=False，但此时可接单
                        try:
                            if getattr(self.sim, "nav_running", False):
                                self.sim.state.driving = True
                            else:
                                self.sim.state.driving = False
                        except Exception:
                            pass
                except Exception:
                    pass
            except Exception:
                pass
            time.sleep(eff_sleep_s)