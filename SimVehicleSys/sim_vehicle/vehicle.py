from __future__ import annotations
import time
import math
from dataclasses import dataclass
from typing import Optional, List

import paho.mqtt.client as mqtt

from SimVehicleSys.config.settings import Config
from SimVehicleSys.config.mqtt_config import generate_vda_mqtt_base_topic
from SimVehicleSys.mqtt.client import publish_json
from SimVehicleSys.utils.helpers import get_timestamp, get_distance, iterate_position, iterate_position_with_trajectory, canonicalize_map_id
from SimVehicleSys.protocol.vda5050_common import AgvPosition, NodePosition, Velocity
from SimVehicleSys.protocol.vda_2_0_0.connection import Connection, ConnectionState
from SimVehicleSys.protocol.vda_2_0_0.state import (
    State,
    ActionState,
    ActionStatus,
    NodeState,
    EdgeState,
    OperatingMode,
    BatteryState,
    SafetyState,
    EStop,
)
from SimVehicleSys.protocol.vda_2_0_0.order import Order, Node, Edge
from SimVehicleSys.protocol.vda_2_0_0.instant_actions import InstantActions
from SimVehicleSys.protocol.vda_2_0_0.action import Action
from .error_manager import emit_error
from .state_manager import global_store

from .navigation import (
    resolve_scene_path,
    parse_scene_topology,
    a_star,
    route_polyline,
    augment_with_corner_turns,
    find_station_position,
    find_point_name_by_id,
    nearest_station,
)


@dataclass
class VehicleSimulator:
    connection_topic: str
    state_topic: str
    visualization_topic: str
    factsheet_topic: str
    connection: Connection
    state: State
    visualization: State
    order: Optional[Order]
    instant_actions: Optional[InstantActions]
    config: Config
    action_start_time: Optional[float]
    nav_points: Optional[List[dict]]
    nav_idx: int
    nav_running: bool
    nav_paused: bool
    nav_target_station: Optional[str]
    nav_map_name: Optional[str]
    # 订单路由的站点名序列（用于到站检测与状态裁剪）
    nav_route_node_ids: Optional[List[str]]
    charging_requested: bool = False

    @staticmethod
    def create(config: Config) -> "VehicleSimulator":
        base_topic = generate_vda_mqtt_base_topic(
            config.mqtt_broker.vda_interface,
            config.vehicle.vda_version,
            config.vehicle.manufacturer,
            config.vehicle.serial_number,
        )
        connection_topic = f"{base_topic}/connection"
        state_topic = f"{base_topic}/state"
        visualization_topic = f"{base_topic}/visualization"
        factsheet_topic = f"{base_topic}/factsheet"
        connection = VehicleSimulator._create_initial_connection(config)
        state, agv_position = VehicleSimulator._create_initial_state(config)
        visualization = VehicleSimulator._create_initial_visualization(config, agv_position)
        return VehicleSimulator(
            connection_topic=connection_topic,
            state_topic=state_topic,
            visualization_topic=visualization_topic,
            factsheet_topic=factsheet_topic,
            connection=connection,
            state=state,
            visualization=visualization,
            order=None,
            instant_actions=None,
            config=config,
            action_start_time=None,
            nav_points=None,
            nav_idx=0,
            nav_running=False,
            nav_paused=False,
            nav_target_station=None,
            nav_map_name=None,
            nav_route_node_ids=None,
        )

    @staticmethod
    def _create_initial_connection(config: Config) -> Connection:
        return Connection(
            header_id=0,
            timestamp=get_timestamp(),
            version=config.vehicle.vda_full_version,
            manufacturer=config.vehicle.manufacturer,
            serial_number=config.vehicle.serial_number,
            connection_state=ConnectionState.ConnectionBroken,
        )

    @staticmethod
    def _create_initial_state(config: Config) -> tuple[State, AgvPosition]:
        import random
        random_x = random.random() * 5.0 - 2.5
        random_y = random.random() * 5.0 - 2.5
        agv_position = AgvPosition(
            x=random_x,
            y=random_y,
            position_initialized=False,
            theta=0.0,
            map_id=canonicalize_map_id(config.settings.map_id),
            deviation_range=None,
            map_description=None,
            localization_score=None,
        )
        state = State(
            header_id=0,
            timestamp=get_timestamp(),
            version=config.vehicle.vda_full_version,
            manufacturer=config.vehicle.manufacturer,
            serial_number=config.vehicle.serial_number,
            driving=False,
            operating_mode=OperatingMode.Automatic,
            node_states=[],
            edge_states=[],
            last_node_id="",
            order_id="",
            order_update_id=0,
            last_node_sequence_id=0,
            action_states=[],
            information=[],
            loads=[],
            battery_state=BatteryState(battery_charge=float(config.settings.battery_default), charging=False),
            safety_state=SafetyState(e_stop=EStop.None_, field_violation=False),
            paused=False,
            new_base_request=False,
            agv_position=agv_position,
            velocity=Velocity(vx=0.0, vy=0.0, omega=0.0),
            zone_set_id=None,
            waiting_for_interaction_zone_release=False,
        )
        return state, agv_position

    @staticmethod
    def _create_initial_visualization(config: Config, agv_position: AgvPosition) -> State:
        vis = State(
            header_id=0,
            timestamp=get_timestamp(),
            version=config.vehicle.vda_full_version,
            manufacturer=config.vehicle.manufacturer,
            serial_number=config.vehicle.serial_number,
            driving=False,
            operating_mode=OperatingMode.Automatic,
            node_states=[],
            edge_states=[],
            last_node_id="",
            order_id="",
            order_update_id=0,
            last_node_sequence_id=0,
            action_states=[],
            information=[],
            loads=[],
            battery_state=BatteryState(battery_charge=float(config.settings.battery_default), charging=False),
            safety_state=SafetyState(e_stop=EStop.None_, field_violation=False),
            paused=False,
            new_base_request=False,
            agv_position=agv_position,
            velocity=Velocity(vx=0.0, vy=0.0, omega=0.0),
            zone_set_id=None,
            waiting_for_interaction_zone_release=False,
        )
        return vis

    def publish_connection(self, mqtt_cli: mqtt.Client) -> None:
        payload_broken = self.connection
        publish_json(mqtt_cli, self.connection_topic, payload_broken, qos=1, retain=False)
        scale = max(0.0001, float(self.config.settings.sim_time_scale))
        time.sleep(max(0.01, 1.0 / scale))
        self.connection.header_id += 1
        self.connection.timestamp = get_timestamp()
        self.connection.connection_state = ConnectionState.Online
        publish_json(mqtt_cli, self.connection_topic, self.connection, qos=1, retain=False)

    def publish_state(self, mqtt_cli: mqtt.Client) -> None:
        self.state.header_id += 1
        self.state.timestamp = get_timestamp()
        # 同步集中式错误列表到 VDA5050 state，确保始终包含 errors 列表
        try:
            rt = global_store.get_runtime(self.config.vehicle.serial_number)
            # 无错误时为空数组
            self.state.errors = list(getattr(rt, "errors", []))
            # 同步地图名称到 agvPosition.mapId
            try:
                cm = getattr(rt, "current_map", None)
            except Exception:
                cm = None
            nav_map = self.nav_map_name if self.nav_running else None
            raw_map = cm or nav_map or (self.state.agv_position.map_id if self.state.agv_position else None) or self.config.settings.map_id
            canonical = canonicalize_map_id(raw_map)
            if self.state.agv_position:
                self.state.agv_position.map_id = str(canonical)
        except Exception:
            self.state.errors = []
        # 计算速度（vx, vy, omega），基于位置增量与时间间隔
        try:
            now_ts = time.time()
            pos = self.state.agv_position
            prev = getattr(self, "_last_publish_state", None)
            vx = vy = omega = 0.0
            if pos is not None:
                if prev and isinstance(prev, dict):
                    dt = max(1e-3, now_ts - float(prev.get("ts", now_ts)))
                    dx = float(pos.x) - float(prev.get("x", pos.x))
                    dy = float(pos.y) - float(prev.get("y", pos.y))
                    dtheta = float(pos.theta) - float(prev.get("theta", pos.theta))
                    vx = dx / dt
                    vy = dy / dt
                    omega = dtheta / dt
                # 记录当前用于下次增量计算
                self._last_publish_state = {"ts": now_ts, "x": float(pos.x), "y": float(pos.y), "theta": float(pos.theta)}
            self.state.velocity = Velocity(vx=vx, vy=vy, omega=omega)
        except Exception:
            # 保守容错，保持原值
            pass
        publish_json(mqtt_cli, self.state_topic, self.state, qos=1, retain=False)

    def publish_visualization(self, mqtt_cli: mqtt.Client) -> None:
        self.visualization.header_id += 1
        self.visualization.timestamp = get_timestamp()
        self.visualization.agv_position = self.state.agv_position
        self.visualization.battery_state = self.state.battery_state
        self.visualization.safety_state = self.state.safety_state
        publish_json(mqtt_cli, self.visualization_topic, self.visualization, qos=1, retain=False)

    def accept_instant_actions(self, instant_action_request: InstantActions) -> None:
        self.instant_actions = instant_action_request
        for ia in self.instant_actions.actions:
            self.state.action_states.append(ActionState(
                action_id=ia.action_id,
                action_status=ActionStatus.Waiting,
                action_type=ia.action_type,
                result_description=None,
                action_description=None,
            ))

    def process_order(self, order_request: Order) -> None:
        if order_request.order_id != self.state.order_id:
            self._handle_new_order(order_request)
        else:
            self._handle_order_update(order_request)

    def _handle_new_order(self, order_request: Order) -> None:
        if not self._can_accept_new_order():
            return
        if self.is_vehicle_ready_for_new_order():
            self.state.action_states.clear()
            self._accept_order(order_request)
        else:
            self._reject_order("There are active order states or edge states")

    def _handle_order_update(self, order_request: Order) -> None:
        if order_request.order_update_id > self.state.order_update_id:
            if not self._can_accept_new_order():
                return
            self.state.action_states.clear()
            self._accept_order(order_request)
        else:
            self._reject_order("Order update ID is lower than current")

    def _can_accept_new_order(self) -> bool:
        # 若存在已释放的节点，要求车辆已到达“最新”释放节点（按最大 sequence_id 判断）
        released_nodes = [n for n in self.state.node_states if n.released]
        if released_nodes:
            try:
                latest = max(released_nodes, key=lambda n: int(n.sequence_id))
            except Exception:
                latest = released_nodes[-1]
            if self.state.last_node_sequence_id != latest.sequence_id:
                self._reject_order("Vehicle has not arrived at the latest released node")
                return False
            if not self._is_vehicle_close_to_last_released_node():
                self._reject_order("Vehicle is not close enough to last released node")
                return False
        return True

    def _is_vehicle_close_to_last_released_node(self) -> bool:
        # 使用最新释放节点（最大 sequence_id）进行距离校验
        released_nodes = [n for n in self.state.node_states if n.released]
        if not released_nodes:
            return True
        try:
            latest = max(released_nodes, key=lambda n: int(n.sequence_id))
        except Exception:
            latest = released_nodes[-1]
        if self.state.agv_position and latest and latest.node_position:
            np = latest.node_position
            vp = self.state.agv_position
            distance = get_distance(vp.x, vp.y, np.x, np.y)
            return distance <= 0.1
        # 无坐标信息时不阻塞接单
        return True

    def is_vehicle_ready_for_new_order(self) -> bool:
        return (
            len(self.state.node_states) == 0 and
            len(self.state.edge_states) == 0 and
            bool(self.state.agv_position and self.state.agv_position.position_initialized)
        )

    def _accept_order(self, order_request: Order) -> None:
        self.order = order_request
        self.state.order_id = self.order.order_id
        self.state.order_update_id = self.order.order_update_id
        if self.state.order_update_id == 0:
            self.state.last_node_sequence_id = 0
        self.state.action_states.clear()
        self.state.node_states.clear()
        self.state.edge_states.clear()
        self._process_order_nodes()
        self._process_order_edges()
        try:
            if self.order and self.order.nodes:
                ordered_nodes = sorted(list(self.order.nodes), key=lambda n: int(n.sequence_id))
                route_node_ids = [str(n.node_id) for n in ordered_nodes]
                map_name = self.state.agv_position.map_id if self.state.agv_position else None
                self.start_path_navigation_by_nodes(route_node_ids, map_name)
        except Exception as e:
            print(f"order path navigation failed: {e}")

    def _process_order_nodes(self) -> None:
        for node in list(self.order.nodes):
            self.state.node_states.append(NodeState(
                node_id=node.node_id,
                sequence_id=node.sequence_id,
                released=node.released,
                node_description=node.node_description,
                node_position=node.node_position,
            ))
            for action in node.actions or []:
                self._add_action_state(action)

    def _process_order_edges(self) -> None:
        for edge in list(self.order.edges):
            self.state.edge_states.append(EdgeState(
                edge_id=edge.edge_id,
                sequence_id=edge.sequence_id,
                released=edge.released,
                edge_description=edge.edge_description,
                trajectory=edge.trajectory,
            ))
            for action in edge.actions or []:
                self._add_action_state(action)

    def _add_action_state(self, action: Action) -> None:
        self.state.action_states.append(ActionState(
            action_id=action.action_id,
            action_type=action.action_type,
            action_description=action.action_description,
            action_status=ActionStatus.Waiting,
            result_description=None,
        ))

    def _reject_order(self, reason: str) -> None:
        print(f"Rejecting order: {reason}")

    def update_state(self) -> None:
        # 记录更新间隔，用于将速度按秒换算为每次更新的位移
        now_ts = time.time()
        prev_ts = getattr(self, "_last_update_time", None)
        dt = max(1e-3, now_ts - prev_ts) if prev_ts is not None else 0.05
        self._last_update_time = now_ts
        self._last_delta_seconds = dt
        if self._is_action_in_progress():
            return
        self._process_instant_actions()
        self._update_navigation()
        if not self.order:
            return
        self._process_node_actions()
        if not self.nav_running:
            self._update_vehicle_position()

    def _is_action_in_progress(self) -> bool:
        if self.action_start_time is not None:
            current = time.time()
            scale = max(0.0001, float(self.config.settings.sim_time_scale))
            duration = float(self.config.settings.action_time) / scale
            return current < self.action_start_time + duration
        return False

    def _process_instant_actions(self) -> None:
        if self.instant_actions:
            for action in list(self.instant_actions.actions):
                for st in self.state.action_states:
                    if st.action_id == action.action_id and st.action_status == ActionStatus.Waiting:
                        self.run_action(action)

    def run_action(self, action: Action) -> None:
        idx = next((i for i, x in enumerate(self.state.action_states) if x.action_id == action.action_id), None)
        if idx is None:
            return
        self.state.action_states[idx].action_status = ActionStatus.Running
        if action.action_type == "initPosition":
            self._handle_init_position_action(action)
        elif action.action_type in ("startStationNavigation", "navigateToStation"):
            self._handle_start_station_navigation_action(action)
        else:
            print(f"Unknown action type: {action.action_type}")
        self.state.action_states[idx].action_status = ActionStatus.Finished

    def _handle_init_position_action(self, action: Action) -> None:
        def extract_float(key: str) -> float:
            params = action.action_parameters or []
            for p in params:
                if p.key == key:
                    if isinstance(p.value, (int, float)):
                        return float(p.value)
                    try:
                        return float(p.value)  # type: ignore
                    except Exception:
                        return 0.0
            return 0.0
        def extract_string(key: str) -> str:
            params = action.action_parameters or []
            for p in params:
                if p.key == key:
                    return str(p.value)
            return ""
        new_pos = AgvPosition(
            x=extract_float("x"),
            y=extract_float("y"),
            position_initialized=True,
            theta=extract_float("theta"),
            map_id=extract_string("mapId"),
        )
        self.state.agv_position = new_pos
        self.state.last_node_id = extract_string("lastNodeId")
        self.visualization.agv_position = new_pos

    def _handle_start_station_navigation_action(self, action: Action) -> None:
        def extract_string(key: str) -> str:
            params = action.action_parameters or []
            for p in params:
                if p.key == key:
                    return str(p.value)
            return ""
        station_id = extract_string("stationId")
        map_id = extract_string("mapId") or (self.state.agv_position.map_id if self.state.agv_position else "")
        try:
            self.start_station_navigation(station_id, map_id)
        except Exception as e:
            print(f"startStationNavigation failed: {e}")

    def _process_node_actions(self) -> None:
        idx = self._find_order_last_node_index()
        if idx is None:
            return
        # 仅在到站且停止后才执行当前节点的动作
        vp = self.state.agv_position
        if vp is None:
            return
        # 若导航仍在进行或车辆仍在行驶，等待停止
        if self.nav_running or getattr(self.state, "driving", False):
            return
        # 计算与当前节点的距离，并要求 <= 0.05m（停止阈值）
        try:
            cur_node = self.order.nodes[idx]
        except Exception:
            cur_node = None
        if not cur_node:
            return
        target_x = None
        target_y = None
        try:
            np = getattr(cur_node, "node_position", None)
            if np:
                target_x = float(getattr(np, "x", vp.x))
                target_y = float(getattr(np, "y", vp.y))
            else:
                # 无节点坐标时，尝试通过地图解析站点位置
                try:
                    fp = resolve_scene_path(self.nav_map_name or (vp.map_id if vp else None))
                    pos = find_station_position(fp, str(getattr(cur_node, "node_id", "") or ""))
                    if pos:
                        target_x = float(pos[0])
                        target_y = float(pos[1])
                except Exception:
                    pass
        except Exception:
            pass
        if target_x is not None and target_y is not None:
            try:
                dist = get_distance(vp.x, vp.y, target_x, target_y)
                if dist > 0.01:
                    return
            except Exception:
                # 距离计算异常则保守不执行
                return
        # 执行动作：支持托盘顶升/顶降（JackLoad/JackUnload）以及充电任务（StartCharging）
        node_actions = self.order.nodes[idx].actions or []
        if node_actions:
            for st in self.state.action_states:
                for a in node_actions:
                    if st.action_id == a.action_id and st.action_status == ActionStatus.Waiting:
                        try:
                            atype = str(a.action_type or st.action_type or "").strip()
                            if atype in ("JackLoad", "JackUnload"):
                                from SimVehicleSys.sim_vehicle.action_executor import execute_pallet_action_in_sim
                                execute_pallet_action_in_sim(self, atype, a.action_parameters)
                            elif atype in ("StartCharging", "startCharging", "ChargingStart"):
                                from SimVehicleSys.sim_vehicle.action_executor import execute_charging_action_in_sim
                                execute_charging_action_in_sim(self, a.action_parameters)
                            # 默认行为：标记完成，并设置动作时长计时器
                            st.action_status = ActionStatus.Finished
                            self.action_start_time = time.time()
                            return
                        except Exception as e:
                            print(f"process node action failed: {e}")
                            st.action_status = ActionStatus.Finished
                            self.action_start_time = time.time()
                            return

    def _find_order_last_node_index(self) -> Optional[int]:
        for i, n in enumerate(self.order.nodes):
            if n.sequence_id == self.state.last_node_sequence_id:
                return i
        return None

    def _update_vehicle_position(self) -> None:
        if not self.state.agv_position or not self.state.node_states:
            return
        if len(self.state.node_states) == 1:
            self.state.node_states.pop(0)
            return
        next_node = self._get_next_node()
        if not next_node or not next_node.released:
            return
        vp = self.state.agv_position
        np = next_node.node_position
        if not np:
            return
        updated_x, updated_y, updated_theta = self._calculate_new_position(vp, np, next_node)
        distance = get_distance(vp.x, vp.y, np.x, np.y)
        scale = max(0.0001, float(self.config.settings.sim_time_scale))
        dt = max(1e-3, float(getattr(self, "_last_delta_seconds", 0.05)))
        should_arrive = distance < (float(self.config.settings.speed) * scale * dt) + 0.1
        next_node_id = next_node.node_id
        next_node_seq = next_node.sequence_id
        self.state.agv_position.x = updated_x
        self.state.agv_position.y = updated_y
        self.state.agv_position.theta = updated_theta
        self.visualization.agv_position = self.state.agv_position
        if should_arrive:
            if self.state.node_states:
                self.state.node_states.pop(0)
            if self.state.edge_states:
                self.state.edge_states.pop(0)
            # 使用地图 points.name 作为 lastNodeId（若可解析）
            try:
                map_name = self.nav_map_name or (self.state.agv_position.map_id if self.state.agv_position else None)
                if map_name:
                    fp = resolve_scene_path(map_name)
                    label = find_point_name_by_id(fp, str(next_node_id))
                    self.state.last_node_id = label or str(next_node_id)
                else:
                    self.state.last_node_id = str(next_node_id)
            except Exception:
                self.state.last_node_id = str(next_node_id)
            self.state.last_node_sequence_id = next_node_seq

    def _get_next_node(self) -> Optional[NodeState]:
        last_idx = 0
        for i, ns in enumerate(self.state.node_states):
            if ns.sequence_id == self.state.last_node_sequence_id:
                last_idx = i
                break
        if last_idx >= len(self.state.node_states) - 1:
            return None
        return self.state.node_states[last_idx + 1]

    def start_station_navigation(self, station_id: str, map_name: Optional[str]) -> None:
        if not station_id:
            raise ValueError("station_id required")
        fp = resolve_scene_path(map_name)
        topo = parse_scene_topology(fp)
        cur = self.state.agv_position
        if not cur:
            raise RuntimeError("AGV position not initialized")
        sp = find_station_position(fp, station_id)
        if sp is None:
            raise ValueError(f"Station '{station_id}' not found")
        start_station = nearest_station((cur.x, cur.y), topo["stations"])  # type: ignore
        end_station = nearest_station(sp, topo["stations"])  # type: ignore
        if not start_station or not end_station:
            raise ValueError("Nearest stations not found")
        route = a_star(start_station, end_station, topo["stations"], topo["paths"])  # type: ignore
        if not route:
            raise ValueError("No route found between stations")
        self.start_path_navigation_by_nodes(route, map_name)
        self.nav_target_station = station_id

    def _update_navigation(self) -> None:
        if not self.nav_running or self.nav_paused:
            return
        if not self.nav_points or self.nav_idx >= len(self.nav_points):
            self.nav_running = False
            self.state.driving = False
            try:
                self.nav_map_name = None
            except Exception:
                pass
            try:
                target_id = self.nav_target_station
                if target_id:
                    # 导航结束时优先将 lastNodeId 设为站点名称（如可解析）；并用解析后的名称更新序列号
                    try:
                        map_name = self.nav_map_name or (self.state.agv_position.map_id if self.state.agv_position else None)
                        label = None
                        if map_name:
                            fp = resolve_scene_path(map_name)
                            label = find_point_name_by_id(fp, str(target_id))
                        self.state.last_node_id = label or str(target_id)
                        try:
                            match_key = label or str(target_id)
                            ns = next((n for n in self.state.node_states if str(n.node_id) == str(match_key)), None)
                            if ns:
                                self.state.last_node_sequence_id = ns.sequence_id
                        except Exception:
                            pass
                    except Exception:
                        self.state.last_node_id = str(target_id)
            except Exception:
                pass
            # 清空旧订单的节点/边状态，避免新订单被拒绝为“存在活动状态”
            try:
                self.state.node_states.clear()
                self.state.edge_states.clear()
            except Exception:
                pass
            return
        vp = self.state.agv_position
        if not vp:
            return
        scale = max(0.0001, float(self.config.settings.sim_time_scale))
        dt = max(1e-3, float(getattr(self, "_last_delta_seconds", 0.05)))
        step = max(1e-9, float(self.config.settings.speed) * scale * dt)
        remain = step
        guard = 200
        while remain > 1e-9 and self.nav_idx < len(self.nav_points) and guard > 0:
            pt = self.nav_points[self.nav_idx]
            tx = float(pt.get("x", vp.x))
            ty = float(pt.get("y", vp.y))
            ttheta = float(pt.get("theta", vp.theta or 0.0))
            dx = tx - vp.x
            dy = ty - vp.y
            dist = math.hypot(dx, dy)
            if dist <= 1e-9:
                vp.theta = ttheta
                self.nav_idx += 1
                guard -= 1
                continue
            if dist <= remain:
                vp.x = tx
                vp.y = ty
                vp.theta = ttheta
                self.visualization.agv_position = vp
                remain -= dist
                self.nav_idx += 1
                guard -= 1
            else:
                ratio = remain / max(dist, 1e-9)
                vp.x = vp.x + dx * ratio
                vp.y = vp.y + dy * ratio
                vp.theta = math.atan2(dx, dy)
                self.visualization.agv_position = vp
                remain = 0.0
        # 导航过程中：到站检测与状态裁剪
        try:
            self._detect_arrival_and_prune_states()
        except Exception:
            pass
        if self.nav_idx >= len(self.nav_points):
            self.nav_running = False
            self.state.driving = False
            try:
                self.nav_map_name = None
            except Exception:
                pass
            try:
                target_id = self.nav_target_station
                if target_id:
                    # 完成导航时也将 lastNodeId 设为站点名称（如可解析）
                    try:
                        map_name = self.nav_map_name or (self.state.agv_position.map_id if self.state.agv_position else None)
                        if map_name:
                            fp = resolve_scene_path(map_name)
                            label = find_point_name_by_id(fp, str(target_id))
                            self.state.last_node_id = label or str(target_id)
                        else:
                            self.state.last_node_id = str(target_id)
                    except Exception:
                        self.state.last_node_id = str(target_id)
                    try:
                        match_key = label or str(target_id)
                        ns = next((n for n in self.state.node_states if str(n.node_id) == str(match_key)), None)
                        if ns:
                            self.state.last_node_sequence_id = ns.sequence_id
                    except Exception:
                        pass
            except Exception:
                pass
            # 清空旧订单的节点/边状态，避免新订单被拒绝为“存在活动状态”
            try:
                self.state.node_states.clear()
                self.state.edge_states.clear()
            except Exception:
                pass
        else:
            self.state.driving = True

    def start_path_navigation_by_nodes(self, route_node_ids: List[str], map_name: Optional[str]) -> None:
        if not route_node_ids or len(route_node_ids) < 2:
            # 示例触发：节点不足导致路径规划失败
            try:
                emit_error(52702, {"serial_number": self.config.vehicle.serial_number, "map": map_name, "route_node_ids": route_node_ids})
            except Exception:
                pass
            raise ValueError("route_node_ids must contain at least 2 nodes")
        try:
            fp = resolve_scene_path(map_name)
        except FileNotFoundError:
            try:
                emit_error(50101, {"serial_number": self.config.vehicle.serial_number, "map": map_name})  # map load error
            except Exception:
                pass
            raise
        topo = parse_scene_topology(fp)
        stations = topo["stations"]  # type: ignore
        paths = topo["paths"]         # type: ignore
        station_ids = {str(s["id"]) for s in stations}
        route: List[str] = []
        for nid in route_node_ids:
            sid = str(nid)
            if sid in station_ids:
                route.append(sid)
                continue
            # 允许使用站点名称（points.name）作为 node_id：映射到最近锚点
            try:
                pos = find_station_position(fp, sid)
            except Exception:
                pos = None
            if pos is None:
                try:
                    emit_error(52701, {"serial_number": self.config.vehicle.serial_number, "map": map_name, "missing_node": sid})  # can not find target id
                except Exception:
                    pass
                raise ValueError(f"Node '{sid}' not found as anchor or station name")
            nearest = nearest_station(pos, stations)  # type: ignore
            if not nearest:
                try:
                    emit_error(52700, {"serial_number": self.config.vehicle.serial_number, "map": map_name, "station": sid})  # can not find a feasible path
                except Exception:
                    pass
                raise ValueError(f"No nearest station found for station '{sid}'")
            route.append(str(nearest))
        try:
            pts = route_polyline(route, stations, paths, steps_per_edge=20)
        except Exception:
            try:
                emit_error(52702, {"serial_number": self.config.vehicle.serial_number, "map": map_name, "route": route})  # path plan failed
            except Exception:
                pass
            raise
        pts = augment_with_corner_turns(pts, theta_threshold=0.1, step_delta=0.08)
        self.nav_points = pts
        self.nav_idx = 0
        self.nav_running = True
        self.nav_paused = False
        self.state.driving = True
        # 保存订单路由的站点名序列（用于到站检测与状态裁剪）
        try:
            self.nav_route_node_ids = list(route_node_ids)
        except Exception:
            self.nav_route_node_ids = route_node_ids
        # 保留原始目标标识（站点名称或锚点 ID），以便后续依据 NodeState 匹配序列号
        self.nav_target_station = route_node_ids[-1]
        self.nav_map_name = map_name

    def _detect_arrival_and_prune_states(self) -> None:
        """导航过程中检测是否接近下一个订单节点，并进行状态裁剪。"""
        if not self.order or not self.state or not self.state.agv_position:
            return
        try:
            ordered_nodes = sorted(list(self.order.nodes or []), key=lambda n: int(getattr(n, "sequence_id", 0) or 0))
        except Exception:
            ordered_nodes = list(self.order.nodes or [])
        if not ordered_nodes:
            return
        cur_seq = int(self.state.last_node_sequence_id or 0)
        next_node = None
        for n in ordered_nodes:
            try:
                s = int(getattr(n, "sequence_id", 0) or 0)
            except Exception:
                s = 0
            if s > cur_seq:
                next_node = n
                break
        if not next_node:
            return
        next_node_id = str(getattr(next_node, "node_id", "") or "")
        if not next_node_id:
            return
        # 站点位置解析
        pos = None
        try:
            fp = resolve_scene_path(self.nav_map_name or (self.state.agv_position.map_id if self.state.agv_position else None))
            pos = find_station_position(fp, next_node_id)
        except Exception:
            pos = None
        if not pos:
            return
        # 距离阈值判定（find_station_position 返回 (x, y) 元组）
        arrive_threshold = 0.2
        dx = float(self.state.agv_position.x) - float(pos[0])
        dy = float(self.state.agv_position.y) - float(pos[1])
        if dx * dx + dy * dy > arrive_threshold * arrive_threshold:
            return
        # 到站：更新 lastNodeId 与 lastNodeSequenceId
        self.state.last_node_id = next_node_id
        try:
            next_seq = int(getattr(next_node, "sequence_id", 0) or 0)
        except Exception:
            next_seq = cur_seq
        self.state.last_node_sequence_id = next_seq
        # 状态裁剪：保留当前及之后，删除之前已完成的节点与边
        self._prune_states_from_sequence(next_seq)

    def _prune_states_from_sequence(self, seq: int) -> None:
        """按 sequenceId 裁剪 node_states 与 edge_states：保留 >= seq 的项。"""
        try:
            self.state.node_states = [ns for ns in (self.state.node_states or []) if int(getattr(ns, "sequence_id", 0) or 0) >= int(seq)]
        except Exception:
            pass
        try:
            self.state.edge_states = [es for es in (self.state.edge_states or []) if int(getattr(es, "sequence_id", 0) or 0) >= int(seq)]
        except Exception:
            pass

    def _calculate_new_position(self, vehicle_position: AgvPosition, next_node_position: NodePosition, next_node: NodeState):
        next_edge = next((e for e in self.state.edge_states if e.sequence_id == next_node.sequence_id - 1), None)
        scale = max(0.0001, float(self.config.settings.sim_time_scale))
        dt = max(1e-3, float(getattr(self, "_last_delta_seconds", 0.05)))
        speed = float(self.config.settings.speed) * scale * dt
        if next_edge and next_edge.trajectory is not None:
            return iterate_position_with_trajectory(
                vehicle_position.x,
                vehicle_position.y,
                next_node_position.x,
                next_node_position.y,
                speed,
                next_edge.trajectory,
            )
        else:
            return iterate_position(
                vehicle_position.x,
                vehicle_position.y,
                next_node_position.x,
                next_node_position.y,
                speed,
            )

    def publish_factsheet(self, mqtt_cli: mqtt.Client) -> None:
        payload = {
            "header_id": 0,
            "timestamp": get_timestamp(),
            "version": self.config.vehicle.vda_full_version,
            "manufacturer": self.config.vehicle.manufacturer,
            "serial_number": self.config.vehicle.serial_number,
            "type": "AGV",
            "capabilities": ["navigation", "translation", "rotation"],
            "load_capacity_kg": 1000,
        }
        publish_json(mqtt_cli, self.factsheet_topic, payload, qos=1, retain=True)


def create_vehicle(config: Config) -> VehicleSimulator:
    return VehicleSimulator.create(config)