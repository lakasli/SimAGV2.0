from __future__ import annotations
from typing import Any, Dict, List, Optional, Union, Iterable

from SimVehicleSys.protocol.vda_2_0_0.order import Order, Node, Edge
from SimVehicleSys.protocol.vda_2_0_0.action import Action, ActionParameter, BlockingType
from SimVehicleSys.protocol.vda5050_common import NodePosition, Trajectory, ControlPoint
from SimVehicleSys.sim_vehicle.action_executor import execute_pallet_action_in_sim


JsonDict = Dict[str, Any]


def _get(d: JsonDict, *keys: str, default: Any = None) -> Any:
    for k in keys:
        if k in d:
            return d[k]
    return default


def _to_blocking_type(val: Optional[str]) -> BlockingType:
    if not val:
        return BlockingType.Soft
    up = str(val).upper()
    if up in (BlockingType.None_.value, "NONE"):
        return BlockingType.None_
    if up in (BlockingType.Hard.value, "HARD"):
        return BlockingType.Hard
    return BlockingType.Soft


def _parse_action_params(arr: Optional[Iterable[JsonDict]]) -> Optional[List[ActionParameter]]:
    if not arr:
        return None
    params: List[ActionParameter] = []
    for p in arr:
        params.append(ActionParameter(
            key=str(_get(p, "key", default="")),
            value=_get(p, "value", default=""),
        ))
    return params or None


def _parse_actions(arr: Optional[Iterable[JsonDict]]) -> List[Action]:
    actions: List[Action] = []
    if not arr:
        return actions
    for a in arr:
        actions.append(Action(
            action_type=str(_get(a, "actionType", "action_type", default="")),
            action_id=str(_get(a, "actionId", "action_id", default="")),
            blocking_type=_to_blocking_type(_get(a, "blockingType", "blocking_type", default=None)),
            action_description=_get(a, "actionDescription", "action_description", default=None),
            action_parameters=_parse_action_params(_get(a, "actionParameters", "action_parameters", default=None)),
        ))
    return actions


def _parse_node_position(d: Optional[JsonDict], fallback_map_id: Optional[str]) -> Optional[NodePosition]:
    if not d:
        return None
    # map_id may be camelCase or snake_case
    map_id = _get(d, "mapId", "map_id", default=None) or fallback_map_id or ""
    try:
        return NodePosition(
            x=float(_get(d, "x", default=0.0)),
            y=float(_get(d, "y", default=0.0)),
            theta=_get(d, "theta", default=None),
            allowed_deviation_xy=_get(d, "allowedDeviationXY", "allowed_deviation_xy", default=None),
            allowed_deviation_theta=_get(d, "allowedDeviationTheta", "allowed_deviation_theta", default=None),
            map_id=str(map_id),
            map_description=_get(d, "mapDescription", "map_description", default=None),
        )
    except Exception:
        return None


def _parse_trajectory(d: Optional[JsonDict]) -> Optional[Trajectory]:
    if not d:
        return None
    try:
        degree = int(_get(d, "degree", default=0))
        knots = list(_get(d, "knotVector", "knot_vector", default=[]))
        cps_raw = list(_get(d, "controlPoints", "control_points", default=[]))
        cps: List[ControlPoint] = []
        for cp in cps_raw:
            cps.append(ControlPoint(
                x=float(_get(cp, "x", default=0.0)),
                y=float(_get(cp, "y", default=0.0)),
                weight=_get(cp, "weight", default=None),
                orientation=_get(cp, "orientation", default=None),
            ))
        # Build Trajectory dataclass using typed lists
        return Trajectory(degree=degree, knot_vector=[float(k) for k in knots], control_points=cps)
    except Exception:
        return None


def _parse_nodes(arr: Optional[Iterable[JsonDict]], fallback_map_id: Optional[str]) -> List[Node]:
    nodes: List[Node] = []
    if not arr:
        return nodes
    for n in arr:
        np = _parse_node_position(_get(n, "nodePosition", "node_position", default=None), fallback_map_id)
        actions = _parse_actions(_get(n, "actions", default=None))
        nodes.append(Node(
            node_id=str(_get(n, "nodeId", "node_id", default="")),
            sequence_id=int(_get(n, "sequenceId", "sequence_id", default=0)),
            released=bool(_get(n, "released", default=True)),
            node_description=_get(n, "nodeDescription", "node_description", default=None),
            node_position=np,
            actions=actions,
        ))
    return nodes


def _parse_edges(arr: Optional[Iterable[JsonDict]]) -> List[Edge]:
    edges: List[Edge] = []
    if not arr:
        return edges
    for e in arr:
        traj = _parse_trajectory(_get(e, "trajectory", default=None))
        actions = _parse_actions(_get(e, "actions", default=None))
        edges.append(Edge(
            edge_id=str(_get(e, "edgeId", "edge_id", default="")),
            sequence_id=int(_get(e, "sequenceId", "sequence_id", default=0)),
            released=bool(_get(e, "released", default=True)),
            start_node_id=str(_get(e, "startNodeId", "start_node_id", default="")),
            end_node_id=str(_get(e, "endNodeId", "end_node_id", default="")),
            edge_description=_get(e, "edgeDescription", "edge_description", default=None),
            max_speed=_get(e, "maxSpeed", "max_speed", default=None),
            max_height=_get(e, "maxHeight", "max_height", default=None),
            min_height=_get(e, "minHeight", "min_height", default=None),
            orientation=_get(e, "orientation", default=None),
            orientation_type=_get(e, "orientationType", "orientation_type", default=None),
            direction=_get(e, "direction", default=None),
            rotation_allowed=_get(e, "rotationAllowed", "rotation_allowed", default=None),
            max_rotation_speed=_get(e, "maxRotationSpeed", "max_rotation_speed", default=None),
            length=_get(e, "length", default=None),
            trajectory=traj,
            actions=actions,
        ))
    return edges


def _parse_order(raw: Union[Order, JsonDict], fallback_map_id: Optional[str], config_vehicle: Any) -> Order:
    if isinstance(raw, Order):
        return raw
    d = dict(raw or {})
    nodes = _parse_nodes(_get(d, "nodes", default=[]), fallback_map_id)
    edges = _parse_edges(_get(d, "edges", default=[]))
    order = Order(
        header_id=int(_get(d, "headerId", "header_id", default=0)),
        timestamp=str(_get(d, "timestamp", default="")),
        version=str(_get(d, "version", default=str(getattr(config_vehicle, "vda_full_version", "2.0.0")))),
        manufacturer=str(_get(d, "manufacturer", default=str(getattr(config_vehicle, "manufacturer", ""))))
                     or str(getattr(config_vehicle, "manufacturer", "")),
        serial_number=str(_get(d, "serialNumber", "serial_number", default=str(getattr(config_vehicle, "serial_number", ""))))
                      or str(getattr(config_vehicle, "serial_number", "")),
        order_id=str(_get(d, "orderId", "order_id", default="")),
        order_update_id=int(_get(d, "orderUpdateId", "order_update_id", default=0)),
        zone_set_id=_get(d, "zoneSetId", "zone_set_id", default=None),
        nodes=nodes,
        edges=edges,
    )
    return order


def _validate_order(sim_vehicle: Any, order: Order) -> None:
    cfg = getattr(sim_vehicle, "config", None)
    veh = getattr(cfg, "vehicle", None)
    expected_ver = str(getattr(veh, "vda_full_version", "2.0.0"))
    expected_manu = str(getattr(veh, "manufacturer", ""))
    expected_serial = str(getattr(veh, "serial_number", ""))

    if not order.order_id:
        raise ValueError("order_id is required")
    if order.order_update_id < 0:
        raise ValueError("order_update_id must be non-negative")
    if order.version and order.version != expected_ver:
        # 允许版本不同，但提示
        try:
            print(f"Warning: order version '{order.version}' differs from vehicle version '{expected_ver}'")
        except Exception:
            pass
    if order.manufacturer and expected_manu and order.manufacturer != expected_manu:
        raise ValueError("manufacturer mismatch between order and vehicle")
    if order.serial_number and expected_serial and order.serial_number != expected_serial:
        raise ValueError("serial_number mismatch between order and vehicle")

    # 节点与边校验
    ids = [str(n.node_id) for n in order.nodes or []]
    if len(ids) < 1:
        raise ValueError("order must contain at least one node")
    # sequence 递增检查
    seqs = [int(n.sequence_id) for n in order.nodes]
    if any(seqs[i] >= seqs[i + 1] for i in range(len(seqs) - 1)):
        raise ValueError("node sequence_id must be strictly increasing")
    # 边的起止节点存在检查
    idset = set(ids)
    for e in order.edges or []:
        if str(e.start_node_id) not in idset or str(e.end_node_id) not in idset:
            raise ValueError(f"edge '{e.edge_id}' references non-existing node")

    # NodePosition 的 map 校验与兜底
    agv_pos = getattr(getattr(sim_vehicle, "state", None), "agv_position", None)
    current_map = str(getattr(agv_pos, "map_id", "") or "")
    for n in order.nodes:
        if n.node_position is None:
            # 允许无坐标，由车辆根据地图节点 ID 走路由
            continue
        if not n.node_position.map_id:
            n.node_position.map_id = current_map


def process_order(sim_vehicle: Any, order_raw: Union[Order, JsonDict]) -> None:
    """
    接收、解析、校验并分发 Order。
    - 解析：支持 camelCase / snake_case 键；构造 VDA 2.0 dataclass。
    - 校验：制造商/序列号/版本一致性、节点序列递增、边引用合法性、必要字段存在。
    - 分发：交给仿真器 VehicleSimulator.process_order 进行状态更新与路径导航。
    """
    try:
        fallback_map_id = None
        try:
            fallback_map_id = str(getattr(getattr(sim_vehicle, "state", None), "agv_position", None).map_id)
        except Exception:
            fallback_map_id = None
        cfg_vehicle = getattr(getattr(sim_vehicle, "config", None), "vehicle", None)
        order = _parse_order(order_raw, fallback_map_id, cfg_vehicle)
        _validate_order(sim_vehicle, order)
        sim_vehicle.process_order(order)
        # 托盘动作不再在接收订单时立即触发，改为在仿真车到站且停止后触发
    except Exception as e:
        raise RuntimeError(f"order_manager.process_order failed: {e}")