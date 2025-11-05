from __future__ import annotations
from typing import Iterable, Any, List, Tuple, Optional
import time

from .utils.helpers import (
    compute_agv_base_polygon,
    compute_outer_bounding_rect,
    expand_rect,
    rects_overlap,
)
from .sim_vehicle.state_manager import global_store


def check_collision(vehicles: Iterable[Any]) -> bool:
    """保留的简单占位：返回是否存在任何碰撞（使用外包络矩形快速检测）。"""
    rects: List[Tuple[float, float, float, float]] = []
    for v in vehicles:
        try:
            st = getattr(v, "state", None)
            pos = getattr(st, "agv_position", None)
            # TODO: 读取物理尺寸（默认值占位）
            length = 1.7
            width = 0.95
            if pos:
                poly = compute_agv_base_polygon(length, width, (float(pos.x), float(pos.y), float(pos.theta or 0.0)))
                rects.append(compute_outer_bounding_rect(poly))
        except Exception:
            pass
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            if rects_overlap(rects[i], rects[j]):
                return True
    return False


class CollisionService:
    """
    碰撞检测服务骨架：周期性检测车辆外包络矩形重叠，并触发紧急停止与错误上报。
    - tick_ms：检测周期（默认100ms）。
    - safety_margin：安全距离（按矩形扩张）。
    """

    def __init__(self, vehicles: Iterable[Any], tick_ms: int = 100, safety_margin: float = 0.1) -> None:
        self.vehicles = list(vehicles)
        self.tick_ms = int(tick_ms)
        self.safety_margin = float(safety_margin)
        self._stop = False

    def start(self) -> None:
        # 骨架：同步阻塞循环；后续可改为线程
        self._stop = False
        while not self._stop:
            try:
                self.detect_and_emit()
            except Exception:
                pass
            time.sleep(max(0.01, self.tick_ms / 1000.0))

    def stop(self) -> None:
        self._stop = True

    def detect_and_emit(self) -> None:
        rects: List[Tuple[float, float, float, float]] = []
        owners: List[Any] = []
        for v in self.vehicles:
            try:
                st = getattr(v, "state", None)
                pos = getattr(st, "agv_position", None)
                length = 1.7
                width = 0.95
                if pos:
                    poly = compute_agv_base_polygon(length, width, (float(pos.x), float(pos.y), float(pos.theta or 0.0)))
                    rect = compute_outer_bounding_rect(poly)
                    rects.append(expand_rect(rect, self.safety_margin))
                    owners.append(v)
            except Exception:
                pass
        for i in range(len(rects)):
            for j in range(i + 1, len(rects)):
                if rects_overlap(rects[i], rects[j]):
                    self.issue_emergency_stop(owners[i])
                    self.issue_emergency_stop(owners[j])
                    self.push_collision_error(owners[i], owners[j])

    def issue_emergency_stop(self, vehicle: Any) -> None:
        try:
            st = getattr(vehicle, "state", None)
            if st:
                st.driving = False
            setattr(vehicle, "nav_paused", True)
        except Exception:
            pass

    def push_collision_error(self, v1: Any, v2: Any) -> None:
        try:
            serial = getattr(getattr(v1, "config", None), "vehicle", None).serial_number
            rt = global_store.get_runtime(serial)
            payload = {
                "type": "Collision",
                "message": "Collision detected between vehicles",
                "with": getattr(getattr(v2, "config", None), "vehicle", None).serial_number,
            }
            global_store.set_error(serial, payload)
        except Exception:
            pass