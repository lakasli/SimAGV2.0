from __future__ import annotations
from typing import Any, Dict, List, Optional, Union, Iterable

from SimVehicleSys.protocol.vda_2_0_0.order import Order, Node, Edge
from SimVehicleSys.protocol.vda_2_0_0.action import Action, ActionParameter, BlockingType
from SimVehicleSys.protocol.vda5050_common import NodePosition, Trajectory, ControlPoint
from SimVehicleSys.sim_vehicle.action_executor import execute_pallet_action_in_sim
from SimVehicleSys.utils.helpers import canonicalize_map_id
from SimVehicleSys.sim_vehicle.error_manager import emit_error


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
            action_type=str(_get(a, "actionType", default="")),
            action_id=str(_get(a, "actionId", default="")),
            blocking_type=_to_blocking_type(_get(a, "blockingType", default=None)),
            action_description=_get(a, "actionDescription", default=None),
            action_parameters=_parse_action_params(_get(a, "actionParameters", default=None)),
        ))
    return actions


def _parse_node_position(d: Optional[JsonDict], fallback_map_id: Optional[str]) -> Optional[NodePosition]:
    if not d:
        return None
    # camelCase only
    map_id = _get(d, "mapId", default=None) or fallback_map_id or ""
    try:
        return NodePosition(
            x=float(_get(d, "x", default=0.0)),
            y=float(_get(d, "y", default=0.0)),
            theta=_get(d, "theta", default=None),
            allowed_deviation_xy=_get(d, "allowedDeviationXY", default=None),
            allowed_deviation_theta=_get(d, "allowedDeviationTheta", default=None),
            map_id=str(map_id),
            map_description=_get(d, "mapDescription", default=None),
        )
    except Exception:
        return None


def _parse_trajectory(d: Optional[JsonDict]) -> Optional[Trajectory]:
    if not d:
        return None
    try:
        degree = int(_get(d, "degree", default=0))
        knots = list(_get(d, "knotVector", default=[]))
        cps_raw = list(_get(d, "controlPoints", default=[]))
        cps: List[ControlPoint] = []
        for cp in cps_raw:
            cps.append(ControlPoint(
                x=float(_get(cp, "x", default=0.0)),
                y=float(_get(cp, "y", default=0.0)),
                weight=_get(cp, "weight", default=None),
                orientation=_get(cp, "orientation", default=None),
            ))
        # Build Trajectory dataclass using typed lists
        traj = Trajectory(degree=degree, knot_vector=[float(k) for k in knots], control_points=cps)
        # 保留 type 字段（如存在），兼容 Straight/CubicBezier/INFPNURBS
        try:
            t = _get(d, "type", default=None)
            if isinstance(t, str) and t:
                setattr(traj, "type", t)
        except Exception:
            pass
        return traj
    except Exception:
        return None


def _parse_nodes(arr: Optional[Iterable[JsonDict]], fallback_map_id: Optional[str]) -> List[Node]:
    nodes: List[Node] = []
    if not arr:
        return nodes
    for n in arr:
        np = _parse_node_position(_get(n, "nodePosition", default=None), fallback_map_id)
        actions = _parse_actions(_get(n, "actions", default=None))
        nodes.append(Node(
            node_id=str(_get(n, "nodeId", default="")),
            sequence_id=int(_get(n, "sequenceId", default=0)),
            released=bool(_get(n, "released", default=True)),
            node_description=_get(n, "nodeDescription", default=None),
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
            edge_id=str(_get(e, "edgeId", default="")),
            sequence_id=int(_get(e, "sequenceId", default=0)),
            released=bool(_get(e, "released", default=True)),
            start_node_id=str(_get(e, "startNodeId", default="")),
            end_node_id=str(_get(e, "endNodeId", default="")),
            edge_description=_get(e, "edgeDescription", default=None),
            max_speed=_get(e, "maxSpeed", default=None),
            max_height=_get(e, "maxHeight", default=None),
            min_height=_get(e, "minHeight", default=None),
            orientation=_get(e, "orientation", default=None),
            orientation_type=_get(e, "orientationType", default=None),
            direction=_get(e, "direction", default=None),
            rotation_allowed=_get(e, "rotationAllowed", default=None),
            max_rotation_speed=_get(e, "maxRotationSpeed", default=None),
            length=_get(e, "length", default=None),
            trajectory=traj,
            actions=actions,
        ))
        # 将边的方向/朝向信息附加到轨迹对象，供规划/运动模块使用
        try:
            if traj is not None:
                orient = _get(e, "orientation", default=None)
                if orient is not None:
                    setattr(traj, "orientation", orient)
                orient_type = _get(e, "orientationType", default=None)
                if orient_type is not None:
                    setattr(traj, "orientation_type", orient_type)
                direction = _get(e, "direction", default=None)
                if direction is not None:
                    setattr(traj, "direction", direction)
        except Exception:
            pass
    return edges


def _parse_order(raw: Union[Order, JsonDict], fallback_map_id: Optional[str], config_vehicle: Any) -> Order:
    if isinstance(raw, Order):
        return raw
    d = dict(raw or {})
    nodes = _parse_nodes(_get(d, "nodes", default=[]), fallback_map_id)
    edges = _parse_edges(_get(d, "edges", default=[]))
    order = Order(
        header_id=int(_get(d, "headerId", default=0)),
        timestamp=str(_get(d, "timestamp", default="")),
        version=str(_get(d, "version", default=str(getattr(config_vehicle, "vda_full_version", "2.0.0")))),
        manufacturer=str(_get(d, "manufacturer", default=str(getattr(config_vehicle, "manufacturer", ""))))
                     or str(getattr(config_vehicle, "manufacturer", "")),
        serial_number=str(_get(d, "serialNumber", default=str(getattr(config_vehicle, "serial_number", ""))))
                      or str(getattr(config_vehicle, "serial_number", "")),
        order_id=str(_get(d, "orderId", default="")),
        order_update_id=int(_get(d, "orderUpdateId", default=0)),
        zone_set_id=_get(d, "zoneSetId", default=None),
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
        raise ValueError("orderId is required")
    if order.order_update_id < 0:
        raise ValueError("orderUpdateId must be non-negative")
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

    # 兼容校验：若节点 mapId 与当前地图不同，允许以当前地图为准进行解析（不拒单）
    try:
        cfg = getattr(sim_vehicle, "config", None)
        settings = getattr(cfg, "settings", None)
        expected_raw = current_map or (getattr(settings, "map_id", None))
        expected_map_name = canonicalize_map_id(expected_raw)
    except Exception:
        expected_map_name = canonicalize_map_id(current_map)
    for n in order.nodes:
        np = getattr(n, "node_position", None)
        if np is None:
            continue
        try:
            node_map_name = canonicalize_map_id(getattr(np, "map_id", None))
        except Exception:
            node_map_name = expected_map_name
        if node_map_name != expected_map_name:
            try:
                np.map_id = expected_map_name
            except Exception:
                pass

    # 补充协议约束的骨架校验与规范化
    _enforce_node_actions_blocking_hard(order)
    _validate_and_trim_edge_actions(order)

    # 节点与边的锁定规则：同站点的 released=false 必须成对出现
    _validate_lock_pairs(order)


def _enforce_node_actions_blocking_hard(order: Order) -> None:
    """
    节点上的动作 blockingType 只能为 HARD。
    当前骨架选择将非 HARD 的值统一改写为 HARD，避免拒单；后续可改为严格抛错。
    """
    try:
        for n in order.nodes or []:
            for a in (n.actions or []):
                try:
                    if a.blocking_type is None or str(a.blocking_type.value).upper() != "HARD":
                        a.blocking_type = BlockingType.Hard
                except Exception:
                    a.blocking_type = BlockingType.Hard
    except Exception:
        pass


def _validate_and_trim_edge_actions(order: Order) -> None:
    """
    边只支持一个动作（否则裁剪到一个）；且仅允许“机构脚本动作”和“货叉升降动作”。
    骨架阶段仅做数量裁剪与 TODO 注释，不做具体类型过滤。
    """
    try:
        for e in order.edges or []:
            actions = list(e.actions or [])
            if len(actions) > 1:
                e.actions = [actions[0]]
            # TODO: 限制动作类型到机构脚本/货叉升降（如 JackLoad/JackUnload/ForkLoad/ForkUnload/Script）
    except Exception:
        pass


def _validate_lock_pairs(order: Order) -> None:
    """校验节点与终止到该节点的边的 released 锁定状态必须成对出现。

    规则:
    - 若某节点 node.released=False，则所有以该节点为 endNodeId 的边必须 released=False；
      若存在任意对应边 released=True，抛错。
    - 至少存在一条以该节点为 endNodeId 的边，否则抛错（防止“孤立锁点”造成不可达却无边约束）。
    - 若某条边 edge.released=False，则其 endNodeId 对应的节点必须 released=False，否则抛错。
    """
    try:
        node_rel = {str(n.node_id): bool(n.released) for n in (order.nodes or [])}
        edges = list(order.edges or [])
        # 反向索引：endNodeId -> list[Edge]
        end_index = {}
        for e in edges:
            end_index.setdefault(str(e.end_node_id), []).append(e)
        # 节点锁定 -> 边锁定一致性
        for nid, nrel in node_rel.items():
            if not nrel:  # 节点锁定
                related = list(end_index.get(str(nid), []) or [])
                if not related:
                    raise ValueError(f"Locked node '{nid}' must have at least one edge ending at it in the same order")
                inconsistent = [e.edge_id for e in related if bool(e.released)]
                if inconsistent:
                    raise ValueError(
                        f"Lock pair mismatch: node '{nid}' released=false but edges ending at it are released=true: {inconsistent}"
                    )
        # 边锁定 -> 节点锁定一致性
        for e in edges:
            if not bool(e.released):
                end_nid = str(e.end_node_id)
                if node_rel.get(end_nid, True):  # 默认为 True；若未找到节点，也视为不一致
                    raise ValueError(
                        f"Lock pair mismatch: edge '{e.edge_id}' released=false but end node '{end_nid}' released=true"
                    )
    except Exception as exc:
        # 将任何解析/一致性错误上抛为 ValueError，保持与其它校验一致的风格
        if isinstance(exc, ValueError):
            raise
        raise ValueError(f"Lock pair validation failed: {exc}")


def process_order(sim_vehicle: Any, order_raw: Union[Order, JsonDict]) -> None:
    """
    接收、解析、校验并分发 Order。
    - 解析：仅支持 camelCase 键名；构造 VDA 2.0 dataclass。
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