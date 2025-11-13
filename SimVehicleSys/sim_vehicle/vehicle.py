from __future__ import annotations
import time
import math
from dataclasses import dataclass
from typing import Optional, List, Tuple

import paho.mqtt.client as mqtt

from SimVehicleSys.config.settings import Config
from SimVehicleSys.config.mqtt_config import generate_vda_mqtt_base_topic
from SimVehicleSys.mqtt.client import publish_json
from SimVehicleSys.utils.helpers import (
    get_timestamp,
    get_distance,
    iterate_position,
    iterate_position_with_trajectory,
    canonicalize_map_id,
    normalize_trajectory,
)
from SimVehicleSys.utils.logger import setup_logger
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
from SimVehicleSys.sim_vehicle.action_executor import applyRotationStepInSim, applyTranslateStepInSim

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
    # 订单边的速度上限映射，用于导航步进时限速（key: edgeId/"from->to"/"from-to"）
    nav_edge_speed_caps: Optional[dict] = None

    # 统一日志器：用于在状态发布时打印关键字段到终端
    logger = setup_logger()
    # 导航最后一条边的标识，以及最终目标点坐标（用于最后一段的减速与到站判定）
    nav_last_edge_id: Optional[str] = None
    nav_final_point: Optional[Tuple[float, float]] = None
    # 加减速距离（米）：仅在起步/转弯后前 1m 加速，终点前 1m 减速
    nav_accel_distance_m: float = 1.0
    nav_decel_distance_m: float = 1.0

    # 可选：首段边与起步点（动态设置）
    nav_first_edge_id: Optional[str] = None
    nav_start_point: Optional[Tuple[float, float]] = None

    # 动作模式："order" 表示订单动作模式；"instant" 表示即时动作模式
    action_mode: str = "order"

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
            deviation_range=0.0,
            map_description="",
            localization_score=0.99,
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
            battery_state=BatteryState(
                battery_charge=float(config.settings.battery_default),
                battery_voltage=50.0,
                battery_health=100,
                reach=0.0,
                charging=False,
            ),
            safety_state=SafetyState(e_stop=EStop.None_, field_violation=False),
            paused=False,
            new_base_request=False,
            agv_position=agv_position,
            velocity=Velocity(vx=0.0, vy=0.0, omega=0.0),
            zone_set_id=canonicalize_map_id(config.settings.map_id),
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
            battery_state=BatteryState(
                battery_charge=float(config.settings.battery_default),
                battery_voltage=50.0,
                battery_health=100,
                reach=0.0,
                charging=False,
            ),
            safety_state=SafetyState(e_stop=EStop.None_, field_violation=False),
            paused=False,
            new_base_request=False,
            agv_position=agv_position,
            velocity=Velocity(vx=0.0, vy=0.0, omega=0.0),
            zone_set_id=canonicalize_map_id(config.settings.map_id),
            waiting_for_interaction_zone_release=False,
        )
        return vis

    def publish_connection(self, mqtt_cli: mqtt.Client, initial: bool = True) -> None:
        """Publish connection status.

        - When `initial` is True: emit a transitional ConnectionBroken then Online.
        - When `initial` is False: emit periodic Online updates (1 Hz scheduled by clock).
        """
        if initial:
            payload_broken = self.connection
            publish_json(mqtt_cli, self.connection_topic, payload_broken, qos=1, retain=False)
            scale = max(0.0001, float(self.config.settings.sim_time_scale))
            time.sleep(max(0.01, 1.0 / scale))
            self.connection.header_id += 1
            self.connection.timestamp = get_timestamp()
            self.connection.connection_state = ConnectionState.Online
            publish_json(mqtt_cli, self.connection_topic, self.connection, qos=1, retain=False)
            return
        # periodic update: keep Online and refresh header/timestamp
        try:
            self.connection.connection_state = ConnectionState.Online
        except Exception:
            pass
        self.connection.header_id += 1
        self.connection.timestamp = get_timestamp()
        publish_json(mqtt_cli, self.connection_topic, self.connection, qos=1, retain=False)

    def publish_state(self, mqtt_cli: mqtt.Client) -> None:
        self.state.header_id += 1
        self.state.timestamp = get_timestamp()
        # 同步集中式错误列表到 VDA5050 state，确保始终包含 errors 列表
        try:
            rt = global_store.get_runtime(self.config.vehicle.serial_number)
            # 无错误时为空数组
            self.state.errors = list(getattr(rt, "errors", []))
            # 清洗 actionDescriptions 为空字符串
            try:
                if isinstance(self.state.action_states, list):
                    for a in self.state.action_states:
                        try:
                            a.action_description = ""
                            a.result_description = ""
                        except Exception:
                            pass
            except Exception:
                pass
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
                # 强制将 mapDescription 置为空字符串
                self.state.agv_position.map_description = ""
                # 默认将定位置信度设置为 0.99
                self.state.agv_position.localization_score = 0.99
                # 默认将 deviationRange 设置为 0
                try:
                    self.state.agv_position.deviation_range = 0.0
                except Exception:
                    pass
            # 统一填充电池默认字段，确保上报包含期望值
            try:
                bs = getattr(self.state, "battery_state", None)
                if bs is not None:
                    bs.battery_voltage = 50.0
                    bs.battery_health = 100
                    bs.reach = 0.0
            except Exception:
                pass
            # 同步 zoneSetId 与地图（使用规范化后的 mapId）
            try:
                self.state.zone_set_id = str(canonical)
            except Exception:
                pass
            # 同步电量到集中式运行时，以便统一门控逻辑与状态上报
            try:
                charge = float(getattr(self.state.battery_state, "battery_charge", 0.0))
                # 取整到 0-100 区间
                lvl = int(max(0, min(100, round(charge))))
                setattr(rt, "battery_level", lvl)
            except Exception:
                pass
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
        # 接收即时动作时：清空 actionStates，仅显示本次 instantActions 的动作
        self.instant_actions = instant_action_request
        try:
            self.state.action_states.clear()
        except Exception:
            self.state.action_states = []
        try:
            self.action_mode = "instant"
        except Exception:
            setattr(self, "action_mode", "instant")
        for ia in self.instant_actions.actions:
            # 即时动作不累计，直接加入等待队列
            self.state.action_states.append(ActionState(
                action_id=ia.action_id,
                action_status=ActionStatus.Waiting,
                action_type=ia.action_type,
                result_description="",
                action_description="",
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
            # 新订单（orderId 变化）：清空 actionStates，并切换到订单动作模式
            try:
                self.state.action_states.clear()
            except Exception:
                self.state.action_states = []
            try:
                self.action_mode = "order"
            except Exception:
                setattr(self, "action_mode", "order")
            # 清除即时动作上下文
            try:
                self.instant_actions = None
            except Exception:
                pass
            self._accept_order(order_request)
        else:
            self._reject_order("There are active order states or edge states")

    def _handle_order_update(self, order_request: Order) -> None:
        if order_request.order_update_id > self.state.order_update_id:
            if not self._can_accept_new_order():
                return
            # 同一 orderId 的订单更新：在即时动作模式下需要清空；在订单模式下累计 actionStates
            try:
                mode = getattr(self, "action_mode", "order")
            except Exception:
                mode = "order"
            if str(mode) == "instant":
                try:
                    self.state.action_states.clear()
                except Exception:
                    self.state.action_states = []
            # 切换到订单模式，并清除即时动作上下文
            try:
                self.action_mode = "order"
            except Exception:
                setattr(self, "action_mode", "order")
            try:
                self.instant_actions = None
            except Exception:
                pass
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
        # 导航未运行且位姿已初始化时即可接收新订单；
        # 是否到达最新释放节点由 _can_accept_new_order 进一步约束。
        return (
            not getattr(self, "nav_running", False) and
            bool(self.state.agv_position and self.state.agv_position.position_initialized)
        )

    def _accept_order(self, order_request: Order) -> None:
        self.order = order_request
        self.state.order_id = self.order.order_id
        self.state.order_update_id = self.order.order_update_id
        if self.state.order_update_id == 0:
            self.state.last_node_sequence_id = 0
        # 接单时重置导航内部状态，防止沿用上一单的 nav_points/nav_idx 导致在路径未生成前被判定为“已完成”，从而回退到订单位姿推进
        try:
            self.nav_points = None
            self.nav_idx = 0
            self.nav_last_edge_id = None
            self.nav_final_point = None
        except Exception:
            self.nav_points = None
            self.nav_idx = 0
        # 提前标记导航运行占位，避免路径未生成窗口触发订单位姿推进导致起步漂移
        self.nav_running = True
        self.state.driving = True
        self.state.node_states.clear()
        self.state.edge_states.clear()
        self._process_order_nodes()
        self._process_order_edges()
        # 构建订单边的限速映射，供导航步进时使用
        try:
            caps = {}
            for e in list(self.order.edges or []):
                ms = getattr(e, "max_speed", None)
                if ms is None:
                    continue
                try:
                    s = str(getattr(e, "start_node_id", "") or "")
                    t = str(getattr(e, "end_node_id", "") or "")
                except Exception:
                    s = str(getattr(e, "start_node_id", "") or "")
                    t = str(getattr(e, "end_node_id", "") or "")
                if s and t:
                    caps[f"{s}->{t}"] = float(ms)
                    caps[f"{s}-{t}"] = float(ms)
                eid = str(getattr(e, "edge_id", "") or "")
                if eid:
                    caps[eid] = float(ms)
            self.nav_edge_speed_caps = caps
        except Exception:
            self.nav_edge_speed_caps = None
        try:
            if self.order and self.order.nodes:
                ordered_nodes = sorted(list(self.order.nodes), key=lambda n: int(n.sequence_id))
                # 若存在锁点（released=false），导航仅至第一个锁点的前一个站点
                cutoff_idx = None
                for i, n in enumerate(ordered_nodes):
                    if not bool(getattr(n, "released", True)):
                        cutoff_idx = max(0, i - 1)
                        break
                if cutoff_idx is None:
                    route_node_ids = [str(n.node_id) for n in ordered_nodes]
                else:
                    # 至少保留两个节点用于路径规划（起点->目标），否则抛错提示不可规划
                    if cutoff_idx < 1:
                        raise ValueError("Route locked at the first target; cannot plan navigation towards a locked first node")
                    route_node_ids = [str(ordered_nodes[j].node_id) for j in range(cutoff_idx + 1)]
                # 为路径规划选择有效的地图：优先 AGV 当前地图；失败时回退到首节点的 mapId；仍失败则回退到配置默认
                map_name = self.state.agv_position.map_id if self.state.agv_position else None
                try:
                    _ = resolve_scene_path(map_name)
                except Exception:
                    try:
                        first_np = ordered_nodes[0].node_position if ordered_nodes and ordered_nodes[0] else None
                        fallback_map = getattr(first_np, "map_id", None) if first_np else None
                        map_name = fallback_map or self.config.settings.map_id
                    except Exception:
                        map_name = self.config.settings.map_id
                self.start_path_navigation_by_nodes(route_node_ids, map_name)
        except Exception as e:
            print(f"order path navigation failed: {e}")
            # 路径生成失败时回退导航标记，避免长时间处于运行态
            self.nav_running = False

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
                # propagate speed/rotation constraints from order edge
                max_speed=edge.max_speed,
                rotation_allowed=edge.rotation_allowed,
                max_rotation_speed=edge.max_rotation_speed,
                # propagate height constraints from order edge (meters)
                max_height=edge.max_height,
                min_height=edge.min_height,
            ))
            for action in edge.actions or []:
                self._add_action_state(action)

    def _add_action_state(self, action: Action) -> None:
        # 累计规则：同一 orderId 下 actionStates 可累计；避免重复 actionId
        try:
            if any(str(getattr(st, "action_id", "")) == str(getattr(action, "action_id", "")) for st in self.state.action_states):
                return
        except Exception:
            pass
        self.state.action_states.append(ActionState(
            action_id=action.action_id,
            action_type=action.action_type,
            action_description="",
            action_status=ActionStatus.Waiting,
            result_description="",
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
        atype = str(action.action_type or "")
        if atype == "initPosition":
            self._handle_init_position_action(action)
        elif atype in ("startStationNavigation", "navigateToStation"):
            self._handle_start_station_navigation_action(action)
        elif atype in ("stopPause", "StopPause"):
            self._handle_stop_pause_action(action)
        elif atype in ("startPause", "StartPause"):
            self._handle_start_pause_action(action)
        elif atype in ("cancelOrder", "CancelOrder"):
            self._handle_cancel_order_action(action)
        elif atype in ("factsheetRequest", "FactsheetRequest"):
            self._handle_factsheet_request_action(action)
        elif atype in ("stateRequest", "StateRequest"):
            self._handle_state_request_action(action)
        elif atype in ("startCharging", "StartCharging", "ChargingStart"):
            # 已在 _process_node_actions 使用 execute_charging_action_in_sim，这里也提供即时触发入口
            from SimVehicleSys.sim_vehicle.action_executor import execute_charging_action_in_sim
            try:
                execute_charging_action_in_sim(self, action.action_parameters)
            except Exception:
                pass
        elif atype in ("stopCharging", "StopCharging"):
            self._handle_stop_charging_action(action)
        elif atype in ("motion", "Motion"):
            self._handle_motion_action(action)
        elif atype in ("translate", "Translate"):
            self._handle_translate_action(action)
        elif atype in ("turn", "Turn"):
            self._handle_turn_action(action)
        elif atype in ("switchMap", "SwitchMap"):
            self._handle_switch_map_action(action)
        elif atype in ("rotateAgv", "RotateAgv"):
            self._handle_rotate_agv_action(action)
        elif atype in ("rotateLoad", "RotateLoad"):
            self._handle_rotate_load_action(action)
        elif atype in ("clearErrors", "ClearErrors"):
            self._handle_clear_errors_action(action)
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
            map_description="",
            localization_score=0.99,
            deviation_range=0.0,
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

    # === InstantActions skeleton handlers ===
    def _handle_stop_pause_action(self, action: Action) -> None:
        # 暂停导航步进：不清除已规划路径，但停止运动
        self.nav_paused = True
        self.state.paused = True
        try:
            self.state.driving = False
        except Exception:
            pass

    def _handle_start_pause_action(self, action: Action) -> None:
        # 恢复导航步进：保留路径，若导航仍在进行则恢复为行驶态
        self.nav_paused = False
        self.state.paused = False
        try:
            if getattr(self, "nav_running", False):
                self.state.driving = True
        except Exception:
            pass

    def _handle_cancel_order_action(self, action: Action) -> None:
        # 取消当前订单与导航状态：停止运动、清除已规划路径，并重置相关标志
        try:
            self.order = None
        except Exception:
            pass
        try:
            self.state.node_states.clear()
            self.state.edge_states.clear()
        except Exception:
            pass
        # 清空订单标识与更新号
        try:
            self.state.order_id = ""
            self.state.order_update_id = 0
        except Exception:
            pass
        # 停止导航与运动
        self.nav_running = False
        self.state.driving = False
        # 取消暂停状态（恢复为未暂停，以避免后续导航受残留影响）
        try:
            self.nav_paused = False
            self.state.paused = False
        except Exception:
            pass
        # 清除路径规划与导航内部状态
        try:
            self.nav_points = None
            self.nav_idx = 0
            self.nav_map_name = None
            self.nav_target_station = None
            self.nav_route_node_ids = None
            self.nav_first_edge_id = None
            self.nav_start_point = None
            self.nav_last_edge_id = None
            self.nav_final_point = None
            self.nav_edge_speed_caps = None
        except Exception:
            pass
        # 可选：调用占位的错误注入接口，后续可扩展为上报/记录
        try:
            self.inject_navigation_cancel()
        except Exception:
            pass

    def _handle_factsheet_request_action(self, action: Action) -> None:
        # 立即发布一次 factsheet（需在调用点传入 mqtt 客户端时触发）
        try:
            # 仅设置标志，由外部周期/控制器实际调用 publish_factsheet
            setattr(self, "_factsheet_request", True)
        except Exception:
            pass

    def _handle_state_request_action(self, action: Action) -> None:
        # 设置标志，外部周期将立即触发一次 publish_state
        try:
            setattr(self, "_state_request", True)
        except Exception:
            pass

    def _handle_stop_charging_action(self, action: Action) -> None:
        try:
            from SimVehicleSys.sim_vehicle.action_executor import execute_charging_action_in_sim
            # 关闭充电：设置 charging=False（用负逻辑表示停止）
            bs = getattr(self.state, "battery_state", None)
            if bs:
                bs.charging = False
            # 清除请求标志
            setattr(self, "charging_requested", False)
        except Exception:
            pass

    def _handle_motion_action(self, action: Action) -> None:
        # 开环速度 vx/vy/w/duration 的骨架，不做具体位移更新（留给后续 AGVManager/global_store 落地）
        pass

    def _handle_translate_action(self, action: Action) -> None:
        # 平动骨架
        pass

    def _handle_turn_action(self, action: Action) -> None:
        # 旋转骨架
        pass

    def _handle_switch_map_action(self, action: Action) -> None:
        # 切换地图与重定位：支持参数
        # map/mapId/map_id: 地图标识（如 "duoc" 或 "VehicleMap/duoc.scene"）
        # switchPoint: 站点名称或ID（优先用于定位）
        # center_x/center_y: 切换后位置坐标（米，作为兜底）
        # initiate_angle/theta: 切换后朝向（弧度，作为兜底）
        try:
            params = action.action_parameters or []

            def _get_str(*keys: str) -> Optional[str]:
                for k in keys:
                    for p in params:
                        if str(getattr(p, "key", "")).lower() == str(k).lower():
                            v = getattr(p, "value", None)
                            return None if v is None else str(v)
                return None

            def _get_float(*keys: str) -> Optional[float]:
                for k in keys:
                    for p in params:
                        if str(getattr(p, "key", "")).lower() == str(k).lower():
                            try:
                                v = getattr(p, "value", None)
                                return None if v is None else float(v)  # type: ignore
                            except Exception:
                                try:
                                    return float(str(getattr(p, "value", "0") or "0"))
                                except Exception:
                                    return None
                return None

            map_raw = _get_str("map", "mapId", "map_id")
            if not map_raw:
                # 无地图参数则不进行切换
                return
            # 解析目标站点与位置参数
            switch_point = _get_str("switchPoint", "stationId")
            cx = _get_float("center_x", "x")
            cy = _get_float("center_y", "y")
            ang = _get_float("initiate_angle", "theta")

            # 解析 .scene 文件以便按站点定位（若提供了 switchPoint）
            pos_x: Optional[float] = cx
            pos_y: Optional[float] = cy
            theta: Optional[float] = ang
            try:
                fp = resolve_scene_path(map_raw)
                if switch_point:
                    sp = find_station_position(fp, str(switch_point))
                    if isinstance(sp, tuple) and len(sp) == 2:
                        pos_x, pos_y = float(sp[0]), float(sp[1])
                # 若未提供 theta，则保持现朝向
                if theta is None and self.state.agv_position:
                    try:
                        theta = float(self.state.agv_position.theta or 0.0)
                    except Exception:
                        theta = 0.0
            except Exception:
                # 地图解析失败时，仍进行 mapId 切换，仅按提供坐标/角度更新
                if theta is None and self.state.agv_position:
                    try:
                        theta = float(self.state.agv_position.theta or 0.0)
                    except Exception:
                        theta = 0.0

            # 更新 VDA5050 state 的位置与地图
            try:
                if not self.state.agv_position:
                    self.state.agv_position = AgvPosition(x=float(pos_x or 0.0), y=float(pos_y or 0.0), theta=float(theta or 0.0), position_initialized=True, map_id=str(map_raw), map_description="", localization_score=0.99, deviation_range=0.0)
                else:
                    self.state.agv_position.x = float(pos_x or 0.0)
                    self.state.agv_position.y = float(pos_y or 0.0)
                    self.state.agv_position.theta = float(theta or 0.0)
                    self.state.agv_position.position_initialized = True
                    self.state.agv_position.map_id = str(map_raw)
                    # 切图时强制 mapDescription 为空字符串
                    try:
                        self.state.agv_position.map_description = ""
                    except Exception:
                        pass
                    # 切图时设置定位置信度默认值 0.99
                    try:
                        self.state.agv_position.localization_score = 0.99
                    except Exception:
                        pass
                    # 切图时设置 deviationRange 默认值 0
                    try:
                        self.state.agv_position.deviation_range = 0.0
                    except Exception:
                        pass
                # 切图时同步 zoneSetId（规范化后的 mapId）
                try:
                    self.state.zone_set_id = str(canonicalize_map_id(map_raw))
                except Exception:
                    pass
                # 可视化位置同步
                self.visualization.agv_position = self.state.agv_position
            except Exception:
                pass

            # 清理导航状态（切图通常中断当前导航）
            try:
                self.nav_running = False
                self.state.driving = False
                self.nav_points = None
                self.nav_idx = 0
                self.nav_map_name = None
                self.nav_target_station = None
                self.nav_last_edge_id = None
                self.nav_final_point = None
                self.nav_route_node_ids = None
            except Exception:
                pass

            # 同步到集中式运行态存储，以便后端与前端立即接收新的地图/位置
            try:
                from backend.schemas import DynamicConfigUpdate, Position
                dyn = DynamicConfigUpdate(
                    current_map=str(map_raw),
                    position=Position(x=float(pos_x or 0.0), y=float(pos_y or 0.0), theta=float(theta or 0.0)),
                )
                global_store.update_dynamic(self.config.vehicle.serial_number, dyn)
            except Exception:
                pass

            # 记录切图日志，便于调试
            try:
                self.logger.info(
                    f"[SWITCH_MAP] serial={self.config.vehicle.serial_number} map='{str(map_raw)}' pos=({float(pos_x or 0.0):.3f},{float(pos_y or 0.0):.3f}) theta={float(theta or 0.0):.3f} anchor='{switch_point or ''}'"
                )
            except Exception:
                pass
        except Exception:
            # 保守失败：不抛出异常以免影响其他动作
            pass

    def _handle_rotate_agv_action(self, action: Action) -> None:
        # 旋转车体骨架：仅更新 theta（后续对接 AGVManager）
        try:
            params = action.action_parameters or []
            angle = 0.0
            for p in params:
                if str(getattr(p, "key", "")) == "angle":
                    try:
                        angle = float(getattr(p, "value", 0.0))
                    except Exception:
                        angle = 0.0
                    break
            if self.state.agv_position:
                target_theta = float(self.state.agv_position.theta or 0.0) + float(angle or 0.0)
                applyRotationStepInSim(self, target_theta, base_step=0.15)
        except Exception:
            pass

    def _handle_rotate_load_action(self, action: Action) -> None:
        # 旋转载荷骨架：标记信息供前端显示/碰撞包络后续使用
        try:
            from SimVehicleSys.protocol.vda_2_0_0.state import Information, InfoReference
            params = action.action_parameters or []
            angle = 0.0
            for p in params:
                if str(getattr(p, "key", "")) == "angle":
                    try:
                        angle = float(getattr(p, "value", 0.0))
                    except Exception:
                        angle = 0.0
                    break
            info = Information(
                info_type="RotateLoad",
                info_references=[InfoReference(reference_key="angle", reference_value=str(angle))],
                info_description=None,
                info_level="INFO",
            )
            try:
                self.state.information = [x for x in self.state.information if x.info_type != "RotateLoad"]
            except Exception:
                self.state.information = []
            self.state.information.append(info)
        except Exception:
            pass

    def _handle_clear_errors_action(self, action: Action) -> None:
        try:
            from .state_manager import global_store
            serial = self.config.vehicle.serial_number
            # 清空集中式错误列表
            try:
                rt = global_store.get_runtime(serial)
                if hasattr(rt, "errors"):
                    rt.errors = []  # type: ignore[attr-defined]
            except Exception:
                pass
            # 同步设备状态
            self.state.errors = []
        except Exception:
            pass

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
        # 到站距离与角度偏差的判定由下方“节点坐标/地图坐标双判定”统一处理
        # 执行动作：支持托盘顶升/顶降（JackLoad/JackUnload）以及充电任务（StartCharging）
        # 到站判定：同时支持节点坐标与地图站点坐标，两者任一满足阈值即视为到站
        node_actions = self.order.nodes[idx].actions or []
        if node_actions:
            # 计算节点坐标与地图站点坐标的到站距离
            dist_np = None
            dist_map = None
            # 统一默认到站阈值为 0.1m
            eps_xy_np = 0.1
            eps_xy_map = 0.1
            try:
                cur_node = self.order.nodes[idx]
                vp = self.state.agv_position
                np_cur = getattr(cur_node, "node_position", None)
                if np_cur is not None and vp is not None:
                    try:
                        eps_xy_np = float(getattr(np_cur, "allowed_deviation_xy", eps_xy_np) or eps_xy_np)
                    except Exception:
                        pass
                    try:
                        tx_np = float(getattr(np_cur, "x", vp.x))
                        ty_np = float(getattr(np_cur, "y", vp.y))
                        dist_np = get_distance(vp.x, vp.y, tx_np, ty_np)
                    except Exception:
                        dist_np = None
                # 地图站点坐标作为兜底（即便节点上有坐标也尝试解析）
                try:
                    map_name = self.nav_map_name or (vp.map_id if vp else None)
                    if map_name:
                        fp = resolve_scene_path(map_name)
                        pos = find_station_position(fp, str(getattr(cur_node, "node_id", "") or ""))
                        if pos is not None and vp is not None:
                            try:
                                np_allowed = getattr(np_cur, "allowed_deviation_xy", None) if np_cur is not None else None
                                eps_xy_map = float(np_allowed) if (np_allowed is not None) else eps_xy_map
                            except Exception:
                                pass
                            dist_map = get_distance(vp.x, vp.y, float(pos[0]), float(pos[1]))
                except Exception:
                    dist_map = None
            except Exception:
                pass
            # 若任一距离满足阈值，允许执行动作
            def _arrived() -> bool:
                ok_np = (dist_np is not None) and (float(dist_np) <= float(eps_xy_np))
                ok_map = (dist_map is not None) and (float(dist_map) <= float(eps_xy_map))
                return bool(ok_np or ok_map)
            if not _arrived():
                return
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
        # 电量为 0 时禁止通过订单驱动的位姿更新
        try:
            charge = float(getattr(self.state.battery_state, "battery_charge", 0.0))
            if charge <= 0.0:
                self.state.driving = False
                try:
                    global_store.set_error(self.config.vehicle.serial_number, {
                        "type": "MovementDenied",
                        "reason": "BatteryZero",
                        "message": "Battery is zero; movement blocked"
                    })
                except Exception:
                    pass
                return
        except Exception:
            pass
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
        try:
            scale_dbg = max(0.0001, float(self.config.settings.sim_time_scale))
            dt_dbg = max(1e-3, float(getattr(self, "_last_delta_seconds", 0.05)))
            self.logger.info(
                f"[ORDER_STEP] last_seq={self.state.last_node_sequence_id} next_id='{next_node.node_id}' dt={dt_dbg:.3f}s scale={scale_dbg:.2f} "
                f"cur=({vp.x:.3f},{vp.y:.3f}) upd=({updated_x:.3f},{updated_y:.3f}) theta_upd={updated_theta:.3f}"
            )
        except Exception:
            pass
        # 起始站订单驱动位姿更新的纯转向门控：先对齐朝向，再进行平移
        try:
            def _norm(a: float) -> float:
                pi = math.pi
                return (a + pi) % (2 * pi) - pi
            is_start_of_order = int(getattr(self.state, "last_node_sequence_id", 0) or 0) == 0
            cur_theta = float(vp.theta or 0.0)
            angle_diff = _norm(float(updated_theta) - cur_theta)
            # 使用节点允许角度偏差作为对齐阈值（若提供）
            align_eps = 1e-3
            try:
                if np and getattr(np, "allowed_deviation_theta", None) is not None:
                    align_eps = float(np.allowed_deviation_theta)  # type: ignore[attr-defined]
            except Exception:
                pass
            if is_start_of_order and abs(angle_diff) > align_eps:
                # 起步阶段仅转向，不更新平移，避免漂移
                try:
                    self.logger.info(
                        f"[ORDER_ROT_GATE_START] angle_diff={angle_diff:.4f} eps={align_eps:.4f} cur_theta={cur_theta:.3f} target_theta={float(updated_theta):.3f}"
                    )
                except Exception:
                    pass
                applyRotationStepInSim(self, float(updated_theta), base_step=0.15)
                return
        except Exception:
            pass
        distance = get_distance(vp.x, vp.y, np.x, np.y)
        scale = max(0.0001, float(self.config.settings.sim_time_scale))
        dt = max(1e-3, float(getattr(self, "_last_delta_seconds", 0.05)))
        # 使用节点允许的 XY 偏差作为到站阈值
        xy_eps = 0.1
        try:
            if np and getattr(np, "allowed_deviation_xy", None) is not None:
                xy_eps = float(np.allowed_deviation_xy)  # type: ignore[attr-defined]
        except Exception:
            pass
        # Arrival check should use the capped base speed (respecting edge maxSpeed)
        try:
            prev_edge = next((e for e in self.state.edge_states if e.sequence_id == next_node.sequence_id - 1), None)
        except Exception:
            prev_edge = None
        capped_speed = float(self.config.settings.speed)
        # 应用速度上下限，再叠加边限速
        try:
            s_cfg = getattr(self.config, "settings", None)
            if s_cfg is not None:
                try:
                    capped_speed = max(float(getattr(s_cfg, "speed_min", 0.0)), min(capped_speed, float(getattr(s_cfg, "speed_max", capped_speed))))
                except Exception:
                    pass
        except Exception:
            pass
        try:
            if prev_edge is not None and getattr(prev_edge, "max_speed", None) is not None:
                capped_speed = min(capped_speed, float(getattr(prev_edge, "max_speed")))
        except Exception:
            pass
        should_arrive = distance < (float(capped_speed) * scale * dt) + xy_eps
        next_node_id = next_node.node_id
        next_node_seq = next_node.sequence_id
        applyTranslateStepInSim(self, float(updated_x), float(updated_y))
        # 订单驱动位姿更新：朝 updated_theta 逐步旋转，避免起始跳变
        def _norm(a: float) -> float:
            pi = math.pi
            return (a + pi) % (2 * pi) - pi
        # 若上一条边限制旋转，则跳过旋转；否则按最大旋转速度约束步进
        try:
            prev_edge = next((e for e in self.state.edge_states if e.sequence_id == next_node.sequence_id - 1), None)
            rotation_allowed = True if prev_edge is None else bool(getattr(prev_edge, "rotation_allowed", True))
            if rotation_allowed:
                max_rot_speed = None if prev_edge is None else getattr(prev_edge, "max_rotation_speed", None)
                base = 0.15
                if max_rot_speed is not None:
                    try:
                        base = float(max_rot_speed) * float(dt) / max(0.0001, float(self.config.settings.sim_time_scale))
                    except Exception:
                        base = 0.15

                applyRotationStepInSim(self, float(updated_theta), base_step=float(base))
        except Exception:
            applyRotationStepInSim(self, float(updated_theta), base_step=0.15)
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
        # 电量为 0 时暂停导航步进，保持在原地
        try:
            charge = float(getattr(self.state.battery_state, "battery_charge", 0.0))
            if charge <= 0.0:
                self.state.driving = False
                # 保持导航状态以便恢复后继续，但不移动
                try:
                    # 使用统一错误代码上报，并由错误存储做去重
                    from .error_manager import emit_error
                    emit_error(54211, {"serial_number": self.config.vehicle.serial_number, "message": "Battery is zero; navigation paused"})
                except Exception:
                    pass
                return
        except Exception:
            pass
        if not self.nav_running or self.nav_paused:
            return
        # 路径尚未就绪：保持导航占位，不关闭 nav_running/driving，等待下一周期（避免首单起步漂移）
        if not self.nav_points:
            return
        # 路径已完成：关闭导航并更新最后站点信息
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
            return
        vp = self.state.agv_position
        if not vp:
            return
        scale = max(0.0001, float(self.config.settings.sim_time_scale))
        dt = max(1e-3, float(getattr(self, "_last_delta_seconds", 0.05)))
        # 计算基础步长：按速度上下限夹取
        try:
            s_cfg = getattr(self.config, "settings", None)
            base_speed = float(self.config.settings.speed)
            if s_cfg is not None:
                try:
                    base_speed = max(float(getattr(s_cfg, "speed_min", 0.0)), min(base_speed, float(getattr(s_cfg, "speed_max", base_speed))))
                except Exception:
                    pass
        except Exception:
            base_speed = float(self.config.settings.speed)
        base_step = max(1e-9, base_speed * scale * dt)
        # 单步推进：每周期仅根据当前点与限速计算一次平移/旋转
        if self.nav_idx < len(self.nav_points):
            pt = self.nav_points[self.nav_idx]
            tx = float(pt.get("x", vp.x))
            ty = float(pt.get("y", vp.y))
            ttheta = float(pt.get("theta", vp.theta or 0.0))
            t_edge_id = str(pt.get("edgeId", "") or "")
            # 导航步进日志已移除
            # 针对订单边限速：按当前点所属边的 maxSpeed 限制步长
            try:
                edge_step = base_step
                caps = self.nav_edge_speed_caps or {}
                cap_val = None
                if t_edge_id:
                    cap_val = caps.get(t_edge_id)
                    if cap_val is None:
                        # 解析 "from->to" 或 "from-to" 形式
                        if "->" in t_edge_id:
                            parts = t_edge_id.split("->", 2)
                            if len(parts) == 2:
                                cap_val = caps.get(f"{parts[0]}->{parts[1]}") or caps.get(f"{parts[0]}-{parts[1]}")
                        elif "-" in t_edge_id:
                            parts = t_edge_id.split("-", 2)
                            if len(parts) == 2:
                                cap_val = caps.get(f"{parts[0]}->{parts[1]}") or caps.get(f"{parts[0]}-{parts[1]}")
                if cap_val is not None:
                    try:
                        edge_step = max(1e-9, float(min(float(self.config.settings.speed), float(cap_val))) * scale * dt)
                    except Exception:
                        edge_step = base_step
                # 起步/转弯后的加速：仅在起步点（或最新转弯完成点）2m 内生效
                try:
                    start_pt = self.nav_start_point
                    accel_m = max(0.0, float(self.nav_accel_distance_m))
                    if start_pt is not None and accel_m > 0.0:
                        sx, sy = start_pt
                        s_dist = math.hypot(vp.x - sx, vp.y - sy)
                        if s_dist < accel_m:
                            ramp = s_dist / max(accel_m, 1e-9)
                            # 线性加速阶梯：避免零步长
                            schedule_up = max(0.01, edge_step * float(ramp))
                            edge_step = min(edge_step, schedule_up)
                except Exception:
                    pass
                # 终点前 2m 的减速：不限定最后一条边，只要接近最终目标即减速
                try:
                    final_pt = self.nav_final_point
                    decel_m = max(0.0, float(self.nav_decel_distance_m))
                    if final_pt is not None and decel_m > 0.0:
                        fx, fy = final_pt
                        y_dist = math.hypot(vp.x - fx, vp.y - fy)
                        arrive_eps = 0.02
                        if y_dist <= arrive_eps:
                            # 到达最终目标：结束导航
                            self.nav_idx = len(self.nav_points)
                            edge_step = 0.0
                        elif y_dist < decel_m:
                            ramp_down = y_dist / max(decel_m, 1e-9)
                            schedule_x = max(0.01, edge_step * float(ramp_down))
                            edge_step = min(edge_step, schedule_x, y_dist)
                except Exception:
                    pass
                # 转弯前 2m 的预减速：若当前目标是转弯锚点，则在接近时减速
                try:
                    cur_pt = pt
                    if bool(cur_pt.get("turnAnchor", False)):
                        ax = float(cur_pt.get("x", vp.x))
                        ay = float(cur_pt.get("y", vp.y))
                        a_dist = math.hypot(vp.x - ax, vp.y - ay)
                        decel_m = max(0.0, float(self.nav_decel_distance_m))
                        if a_dist < decel_m:
                            ramp_down = a_dist / max(decel_m, 1e-9)
                            schedule_a = max(0.01, edge_step * float(ramp_down))
                            edge_step = min(edge_step, schedule_a, a_dist)
                except Exception:
                    pass
                step_len = min(base_step, edge_step)
            except Exception:
                step_len = base_step
            dx = tx - vp.x
            dy = ty - vp.y
            dist = math.hypot(dx, dy)
            # 所有点位上执行“先转向后平移”的门控：转向未对齐则只做转向，不做平移
            def _norm(a: float) -> float:
                pi = math.pi
                return (a + pi) % (2 * pi) - pi
            cur_theta = float(vp.theta or 0.0)
            angle_diff = _norm(ttheta - cur_theta)
            align_eps = 1e-3
            if abs(angle_diff) > align_eps:
                # 转向门控日志已移除
                applyRotationStepInSim(self, float(ttheta), base_step=0.15)
                return
            # 起步门控：在导航起始（第一个点位）时，先完成纯转向，再允许平移
            if self.nav_idx == 0:
                cur_theta = float(vp.theta or 0.0)
                angle_diff = _norm(ttheta - cur_theta)
                start_align_eps = 1e-3
                if abs(angle_diff) > start_align_eps:
                    try:
                        self.logger.info(
                            f"[NAV_START_ROT] angle_diff={angle_diff:.4f} eps={start_align_eps:.4f} applyRotation base=0.15"
                        )
                    except Exception:
                        pass
                    applyRotationStepInSim(self, float(ttheta), base_step=0.15)
                    return
            if dist <= 1e-9:
                # 纯转向点：统一为单步门控，每周期仅执行一次旋转步进
                # 纯转向点日志已移除
                aligned = applyRotationStepInSim(self, float(ttheta), base_step=0.15)
                if aligned:
                    # 仅当上一个“平移点”标记为转弯锚点时，才重置起步点
                    try:
                        prev_idx = self.nav_idx - 1
                        if prev_idx >= 0 and self.nav_points and prev_idx < len(self.nav_points):
                            prev_pt = self.nav_points[prev_idx]
                            if bool(prev_pt.get("turnAnchor", False)):
                                vp_now = self.state.agv_position
                                if vp_now:
                                    self.nav_start_point = (float(vp_now.x), float(vp_now.y))
                    except Exception:
                        pass
                    self.nav_idx += 1
                return
            if dist <= step_len:
                # 移除详细导航目标步进日志
                applyTranslateStepInSim(self, float(tx), float(ty))
                self.nav_idx += 1
            else:
                ratio = step_len / max(dist, 1e-9)
                new_x = vp.x + dx * ratio
                new_y = vp.y + dy * ratio
                # 移除详细导航平移步进日志
                applyTranslateStepInSim(self, float(new_x), float(new_y))
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
            # 允许使用站点名称（points.name）作为 node_id：映射到最近站点
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
            pts = route_polyline(route, stations, paths, steps_per_edge=1)
        except Exception:
            try:
                emit_error(52702, {"serial_number": self.config.vehicle.serial_number, "map": map_name, "route": route})  # path plan failed
            except Exception:
                pass
            raise
        # 使拐角处的纯转向点数量与仿真时间缩放因子线性相关：scale 越大，插入的转向步数越少，整体转向速度越快
        try:
            scale = max(0.0001, float(self.config.settings.sim_time_scale))
        except Exception:
            scale = 1.0
        dynamic_step_delta = max(0.02, 0.08 * scale)
        pts = augment_with_corner_turns(pts, theta_threshold=0.1, step_delta=dynamic_step_delta)
        # 记录最后一条边与最终点坐标，用于最后一段的减速控制与到站阈值判断
        try:
            if pts:
                last_pt = pts[-1]
                self.nav_last_edge_id = str(last_pt.get("edgeId", "") or "")
                self.nav_final_point = (float(last_pt.get("x", 0.0)), float(last_pt.get("y", 0.0)))
            else:
                self.nav_last_edge_id = None
                self.nav_final_point = None
        except Exception:
            self.nav_last_edge_id = None
            self.nav_final_point = None
        # 起步修正：将首个导航点的坐标替换为当前 AGV 姿态的坐标，避免导航开始阶段回跳到站点坐标
        try:
            vp = self.state.agv_position
            if vp and pts and len(pts) >= 1:
                orig_x = float(pts[0].get("x", vp.x))
                orig_y = float(pts[0].get("y", vp.y))
                cur_x = float(vp.x)
                cur_y = float(vp.y)
                pts[0]["x"] = cur_x
                pts[0]["y"] = cur_y
                # 记录起步加速的起点坐标
                try:
                    self.nav_start_point = (cur_x, cur_y)
                except Exception:
                    self.nav_start_point = None
                if len(pts) >= 2:
                    pts[0]["theta"] = float(pts[1].get("theta", pts[0].get("theta", 0.0)))
                # 规范化首段插入的纯转向点：若其坐标与原始首点坐标一致，则替换为当前坐标
                for i in range(1, len(pts)):
                    px = float(pts[i].get("x", cur_x))
                    py = float(pts[i].get("y", cur_y))
                    if abs(px - orig_x) <= 1e-9 and abs(py - orig_y) <= 1e-9:
                        pts[i]["x"] = cur_x
                        pts[i]["y"] = cur_y
                    else:
                        # 一旦遇到不同坐标的点，说明进入了实际平移段，可停止替换
                        break
                # 进一步起步清理：剔除紧邻起始坐标的小半径聚簇点，避免起步阶段在原地附近多次来回
                try:
                    prune_threshold = 0.35  # 米
                    k = 1
                    while k < len(pts):
                        px = float(pts[k].get("x", cur_x))
                        py = float(pts[k].get("y", cur_y))
                        if math.hypot(px - cur_x, py - cur_y) <= prune_threshold:
                            k += 1
                            continue
                        break
                    if k > 1 and k < len(pts):
                        removed = k - 1
                        pts = [pts[0]] + pts[k:]
                        try:
                            self.logger.info(
                                f"[ROUTE_START_PRUNE] removed={removed} threshold={prune_threshold:.2f} first_target=({pts[1].get('x', cur_x):.3f},{pts[1].get('y', cur_y):.3f})"
                            )
                        except Exception:
                            pass
                        # 记录首段边 ID（用于起步加速，仅在首个实际平移目标就绪后设置）
                        try:
                            self.nav_first_edge_id = str(pts[1].get("edgeId", "") or "") if len(pts) >= 2 else None
                        except Exception:
                            self.nav_first_edge_id = None
                except Exception:
                    pass
        except Exception:
            pass
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
        # 保留原始目标标识（站点名称或站点 ID），以便后续依据 NodeState 匹配序列号
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
        # 仅考虑当前导航路由中的节点，并跳过锁定（released=false）的节点
        try:
            route_ids = set(self.nav_route_node_ids or [])
        except Exception:
            route_ids = set(self.nav_route_node_ids or [])
        for n in ordered_nodes:
            try:
                s = int(getattr(n, "sequence_id", 0) or 0)
            except Exception:
                s = 0
            if s <= cur_seq:
                continue
            # 若存在路由约束，则仅在路由节点集合内进行到站检测
            nid = str(getattr(n, "node_id", "") or "")
            if route_ids and nid not in route_ids:
                continue
            # 锁点不参与到站推进，等待后续订单解锁
            try:
                if not bool(getattr(n, "released", True)):
                    # 命中锁点则直接停止到站检测（不向后跳过锁点）
                    next_node = None
                    break
            except Exception:
                pass
            next_node = n
            break
        if not next_node:
            return
        next_node_id = str(getattr(next_node, "node_id", "") or "")
        if not next_node_id:
            return
        # 站点位置解析
        pos = None
        fp = None
        pos = None
        try:
            map_name = self.nav_map_name or (self.state.agv_position.map_id if self.state.agv_position else None)
            fp = resolve_scene_path(map_name)
            pos = find_station_position(fp, next_node_id)
        except Exception:
            # 回退到节点自身的地图（nodePosition.mapId），提高首单成功率
            try:
                np_next = getattr(next_node, "node_position", None)
                fallback_map = getattr(np_next, "map_id", None) if np_next else None
                if fallback_map:
                    fp = resolve_scene_path(fallback_map)
                    pos = find_station_position(fp, next_node_id)
            except Exception:
                pos = None
        if not pos:
            return
        # 距离阈值判定（find_station位置返回 (x, y) 元组），支持节点允许偏差
        arrive_threshold = 0.02
        try:
            np_next = getattr(next_node, "node_position", None)
            if np_next and getattr(np_next, "allowed_deviation_xy", None) is not None:
                arrive_threshold = float(getattr(np_next, "allowed_deviation_xy"))
        except Exception:
            pass
        dx = float(self.state.agv_position.x) - float(pos[0])
        dy = float(self.state.agv_position.y) - float(pos[1])
        if dx * dx + dy * dy > arrive_threshold * arrive_threshold:
            return
        # 到站：优先用地图 points.name 作为 lastNodeId（如可解析），否则回退为节点 ID
        try:
            label = find_point_name_by_id(fp, str(next_node_id)) if fp else None
            self.state.last_node_id = label or str(next_node_id)
        except Exception:
            self.state.last_node_id = str(next_node_id)
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
        # 使用边的 maxSpeed 作为上限（若提供）
        base_speed = float(self.config.settings.speed)
        try:
            if next_edge is not None and getattr(next_edge, "max_speed", None) is not None:
                base_speed = min(base_speed, float(getattr(next_edge, "max_speed")))
        except Exception:
            pass
        speed = float(base_speed) * scale * dt
        try:
            deg = None
            has_traj = bool(next_edge and getattr(next_edge, "trajectory", None) is not None)
            if has_traj:
                try:
                    deg = int(getattr(getattr(next_edge, "trajectory", None), "degree", 0) or 0)
                except Exception:
                    deg = None
            self.logger.info(
                f"[ORDER_CALC] base_speed={float(base_speed):.3f} scale={scale:.2f} dt={dt:.3f}s step_speed={speed:.4f} traj={has_traj} degree={deg}"
            )
        except Exception:
            pass
        if next_edge and next_edge.trajectory is not None:
            try:
                normalize_trajectory(next_edge.trajectory)
            except Exception:
                pass
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
        payload = self._build_factsheet_payload()
        publish_json(mqtt_cli, self.factsheet_topic, payload, qos=1, retain=True)

    def _build_factsheet_payload(self) -> dict:
        """构建 factsheet 载荷骨架，结构与文档一致，便于后续填充真实参数。"""
        s = self.config.settings
        # 使用当前动态设置值（支持运行时热更新），缺失时采用合理默认
        try:
            speed_min = float(getattr(s, "speed_min", 0.01) or 0.01)
        except Exception:
            speed_min = 0.01
        try:
            speed_max = float(getattr(s, "speed_max", getattr(s, "speed", 2.0)) or float(getattr(s, "speed", 2.0)))
        except Exception:
            speed_max = float(getattr(s, "speed", 2.0))
        try:
            accel_max = float(getattr(s, "acceleration_max", 2) or 2)
        except Exception:
            accel_max = 2
        try:
            decel_max = float(getattr(s, "deceleration_max", 2) or 2)
        except Exception:
            decel_max = 2
        try:
            height_min = float(getattr(s, "height_min", 0.01) or 0.01)
        except Exception:
            height_min = 0.01
        try:
            height_max = float(getattr(s, "height_max", 0.10) or 0.10)
        except Exception:
            height_max = 0.10
        try:
            width = float(getattr(s, "width", 0.745) or 0.745)
        except Exception:
            width = 0.745
        try:
            length = float(getattr(s, "length", 1.03) or 1.03)
        except Exception:
            length = 1.03
        return {
            "header_id": 0,
            "timestamp": get_timestamp(),
            "version": self.config.vehicle.vda_full_version,
            "manufacturer": self.config.vehicle.manufacturer,
            "serial_number": self.config.vehicle.serial_number,
            "typeSpecification": {
                "seriesName": self.config.vehicle.serial_number,
                "seriesDescription": self.config.vehicle.serial_number,
                "agvKinematic": "STEER",
                "agvClass": "FORKLIFT",
                "maxLoadMass": 0.5,
                "localizationTypes": ["SLAM"],
                "navigationTypes": ["VIRTUAL_LINE_GUIDED"],
            },
            "physicalParameters": {
                "speedMin": speed_min,
                "speedMax": speed_max,
                "accelerationMax": accel_max,
                "decelerationMax": decel_max,
                "heightMin": height_min,
                "heightMax": height_max,
                "width": width,
                "length": length,
            },
            "protocolLimits": None,
            "protocolFeatures": None,
            "agvGeometry": None,
            "loadSpecification": None,
            "localizationParameters": None,
        }

    # === Active error injection skeletons ===
    def inject_navigation_cancel(self) -> None:
        """主动错误注入接口占位：导航取消。

        保留接口但不执行任何逻辑，后期再实现具体行为。
        """
        pass

    def inject_position_jitter(self, sigma: float = 0.05, duration_ms: int = 1000) -> None:
        """主动错误注入接口占位：位置抖动。

        保留接口但不执行任何逻辑，后期再实现具体行为。
        """
        pass

    def inject_reinitialize_random_pose(self, map_name: Optional[str] = None) -> None:
        """主动错误注入接口占位：随机重置姿态。

        保留接口但不执行任何逻辑，后期再实现具体行为。
        """
        pass


def create_vehicle(config: Config) -> VehicleSimulator:
    return VehicleSimulator.create(config)