from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional

from ..utils.logger import setup_logger
from .state_manager import global_store
from ..utils.helpers import get_timestamp


logger = setup_logger()


class ErrorSeverity(Enum):
    WARNING = "Warning"
    FATAL = "Fatal"


@dataclass
class ErrorMeta:
    code: int
    severity: ErrorSeverity
    category: str
    message: str
    hint: Optional[str] = None
    recoverable: bool = True


ERRORS: Dict[int, ErrorMeta] = {
    # 电池与电源
    50401: ErrorMeta(50401, ErrorSeverity.FATAL, "battery", "battery too low and shutdown", "机器人电量过低自动关机", False),
    52503: ErrorMeta(52503, ErrorSeverity.FATAL, "battery", "battery is too low to move", "电池电量低于模型配置值", False),
    54211: ErrorMeta(54211, ErrorSeverity.WARNING, "battery", "low battery", "电量低", True),

    # 路径规划
    52700: ErrorMeta(52700, ErrorSeverity.FATAL, "path", "can not find a feasible path", "找不到可通行的路径", False),
    52701: ErrorMeta(52701, ErrorSeverity.FATAL, "path", "can not find target id", "找不到目标点", False),
    52702: ErrorMeta(52702, ErrorSeverity.FATAL, "path", "path plan failed", "路径规划失败", False),
    52705: ErrorMeta(52705, ErrorSeverity.FATAL, "path", "motion planning failed", "运动规划失败", False),
    52708: ErrorMeta(52708, ErrorSeverity.FATAL, "path", "destination has obstacles", "终点处有障碍物", False),
    52715: ErrorMeta(52715, ErrorSeverity.FATAL, "path", "current pos has obstacles", "机器人当前位置有障碍物", False),
    52201: ErrorMeta(52201, ErrorSeverity.FATAL, "path", "robot out of path", "机器人在线路外", False),

    # 导航相关
    54231: ErrorMeta(54231, ErrorSeverity.WARNING, "navigation", "Caution: robot is blocked", "注意机器人被阻挡", True),

    # 地图加载与解析
    50100: ErrorMeta(50100, ErrorSeverity.FATAL, "map", "map parse error", "地图解析错误", False),
    50101: ErrorMeta(50101, ErrorSeverity.FATAL, "map", "map load error", "地图加载错误", False),
    50102: ErrorMeta(50102, ErrorSeverity.FATAL, "map", "map is too large", "地图面积过大", False),
    50103: ErrorMeta(50103, ErrorSeverity.FATAL, "map", "map is empty", "地图数据为空", False),
    50104: ErrorMeta(50104, ErrorSeverity.FATAL, "map", "map meta error", "地图元数据错误", False),
    50105: ErrorMeta(50105, ErrorSeverity.FATAL, "map", "map resolution is illegal", "地图分辨率非法", False),
    50106: ErrorMeta(50106, ErrorSeverity.FATAL, "map", "map format invalid", "地图格式非法", False),

    # 地图站点与点位
    52015: ErrorMeta(52015, ErrorSeverity.FATAL, "station", "stations with the same id number in the map", "地图站点中有相同的id", False),
    52107: ErrorMeta(52107, ErrorSeverity.FATAL, "station", "Switch map error in current station point", "切换的地图中不存在对应的站点", False),

    # 通用
    90000: ErrorMeta(90000, ErrorSeverity.WARNING, "generic", "any - 任意错误", None, True),
    90001: ErrorMeta(90001, ErrorSeverity.WARNING, "generic", "operationMode is not AUTOMATIC - 操作模式不是自动模式", None, True),
    90002: ErrorMeta(90002, ErrorSeverity.WARNING, "generic", "send Order to rbk Failed - 向rbk发送订单失败", None, True),
    90003: ErrorMeta(90003, ErrorSeverity.WARNING, "generic", "new OrderId But NotLock - 收到新订单ID但未锁定", None, True),
    90004: ErrorMeta(90004, ErrorSeverity.WARNING, "generic", "edge.endNodeId != NodeId - 边的结束节点ID与当前节点ID不匹配", None, True),
    90005: ErrorMeta(90005, ErrorSeverity.WARNING, "generic", "edge.startNodeId != NodeId - 边的起始节点ID与当前节点ID不匹配", None, True),
    90006: ErrorMeta(90006, ErrorSeverity.WARNING, "generic", "newOrderId rec,But Order is Running - 收到新订单ID，但订单正在运行中", None, True),
    90007: ErrorMeta(90007, ErrorSeverity.WARNING, "generic", "node Or Edge is empty - 节点或边为空", None, True),
    90008: ErrorMeta(90008, ErrorSeverity.WARNING, "generic", "orderUpdateId is lower than last one,msg pass - 订单更新ID低于上一个，消息跳过", None, True),
    90009: ErrorMeta(90009, ErrorSeverity.WARNING, "generic", "orderUpdateId is == than last one,msg pass - 订单更新ID与上一个相同，消息跳过", None, True),
    90010: ErrorMeta(90010, ErrorSeverity.WARNING, "generic", "try to create Order Failed - 尝试创建订单失败", None, True),
    90011: ErrorMeta(90011, ErrorSeverity.WARNING, "generic", "new node base error - 新建节点基础错误", None, True),
    90012: ErrorMeta(90012, ErrorSeverity.WARNING, "generic", "order's nodePosition not in map - 订单的节点位置不在地图中", None, True),
    90013: ErrorMeta(90013, ErrorSeverity.WARNING, "generic", "actionPackEmpty - 动作包为空", None, True),
    90014: ErrorMeta(90014, ErrorSeverity.WARNING, "generic", "nonOrderCancel - 非订单取消", None, True),
}


def get_error_meta(code: int) -> Optional[ErrorMeta]:
    return ERRORS.get(code)


def register_error(meta: ErrorMeta) -> None:
    ERRORS[meta.code] = meta


def emit_error(code: int, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    meta = get_error_meta(code)
    ts = get_timestamp()
    if meta is None:
        payload = {
            "code": code,
            "severity": ErrorSeverity.WARNING.value,
            "category": "generic",
            "message": "unknown error code",
            "context": context or {},
            "recoverable": True,
            "timestamp": ts,
        }
        logger.warning(payload)
        return payload

    payload = {
        "code": meta.code,
        "severity": meta.severity.value,
        "category": meta.category,
        "message": meta.message,
        "hint": meta.hint,
        "context": context or {},
        "recoverable": meta.recoverable,
        "timestamp": ts,
    }

    if meta.severity is ErrorSeverity.FATAL:
        logger.error(payload)
    else:
        logger.warning(payload)

    # 联动保存到集中式状态存储，用于后端状态上报携带 error 字段
    try:
        serial = None
        if isinstance(context, dict):
            serial = context.get("serial_number") or context.get("serial")
        if isinstance(serial, str) and serial:
            global_store.set_error(serial, payload)
    except Exception:
        pass

    return payload