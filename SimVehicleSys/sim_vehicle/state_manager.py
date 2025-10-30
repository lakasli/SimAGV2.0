from __future__ import annotations
import threading
import math
from datetime import datetime
from typing import Dict, Optional, Any

from backend.schemas import (
    AGVRuntime,
    DynamicConfigUpdate,
    Position,
    StatusResponse,
    AGVInfo,
)


def _normalize_angle(a: float) -> float:
    return ((a + math.pi) % (2 * math.pi)) - math.pi


def _step_towards(current: float, target: float, max_delta: float) -> float:
    diff = _normalize_angle(target - current)
    if abs(diff) <= max_delta:
        return current + diff
    return current + math.copysign(max_delta, diff)


class AGVStateStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._runtime: Dict[str, AGVRuntime] = {}

    def _ensure_runtime(self, serial: str) -> AGVRuntime:
        rt = self._runtime.get(serial)
        if rt is None:
            rt = AGVRuntime()
            self._runtime[serial] = rt
        return rt

    def update_dynamic(self, serial: str, dyn: DynamicConfigUpdate) -> Optional[AGVRuntime]:
        with self._lock:
            rt = self._ensure_runtime(serial)
            if dyn.battery_level is not None:
                rt.battery_level = dyn.battery_level
            if dyn.current_map is not None:
                rt.current_map = dyn.current_map
            if dyn.position is not None:
                old_x, old_y, old_theta = rt.position.x, rt.position.y, rt.position.theta
                new_x = dyn.position.x if dyn.position.x is not None else old_x
                new_y = dyn.position.y if dyn.position.y is not None else old_y
                provided_theta = dyn.position.theta
                rt.position.x = new_x
                rt.position.y = new_y
                try:
                    if provided_theta is not None:
                        rt.position.theta = provided_theta
                    else:
                        dx = new_x - old_x
                        dy = new_y - old_y
                        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                            movement_dir = math.atan2(dx, dy)
                            target_heading = movement_dir if rt.movement_state == "forward" else _normalize_angle(movement_dir + math.pi)
                            rt.position.theta = _step_towards(old_theta, target_heading, max_delta=0.15)
                        else:
                            rt.position.theta = old_theta
                except Exception:
                    rt.position.theta = old_theta
            if dyn.speed_limit is not None:
                rt.speed_limit = dyn.speed_limit
                rt.speed = dyn.speed_limit
            if dyn.movement_state is not None:
                mv = str(dyn.movement_state).lower()
                if mv in ("forward", "backward"):
                    rt.movement_state = mv
            rt.last_update = datetime.utcnow()
            return rt

    def move_translate(self, serial: str, dx: float, dy: float, movement_state: Optional[str] = None) -> Optional[AGVRuntime]:
        with self._lock:
            rt = self._ensure_runtime(serial)
            if movement_state is not None:
                mv = str(movement_state).lower()
                if mv in ("forward", "backward"):
                    rt.movement_state = mv
            rt.position.x += dx
            rt.position.y += dy
            try:
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    movement_dir = math.atan2(dx, dy)
                    if rt.movement_state == "forward":
                        target_heading = movement_dir
                    else:
                        target_heading = _normalize_angle(movement_dir + math.pi)
                    rt.position.theta = _step_towards(rt.position.theta, target_heading, max_delta=0.2)
            except Exception:
                pass
            rt.last_update = datetime.utcnow()
            return rt

    def move_rotate(self, serial: str, dtheta: float) -> Optional[AGVRuntime]:
        with self._lock:
            rt = self._ensure_runtime(serial)
            rt.position.theta += dtheta
            rt.last_update = datetime.utcnow()
            return rt

    def get_runtime(self, serial: str) -> AGVRuntime:
        with self._lock:
            return self._ensure_runtime(serial)

    def get_status(self, serial: str, info: Optional[AGVInfo]) -> Optional[StatusResponse]:
        if info is None:
            return None
        with self._lock:
            rt = self._ensure_runtime(serial)
            return StatusResponse(
                serial_number=info.serial_number,
                status="running",
                battery_level=rt.battery_level,
                current_map=rt.current_map,
                position=rt.position,
                speed=rt.speed,
                last_update=rt.last_update,
                error=rt.last_error,
            )

    def set_error(self, serial: str, error_payload: Dict[str, Any]) -> Optional[AGVRuntime]:
        """记录最近一次错误，用于状态上报携带。"""
        with self._lock:
            rt = self._ensure_runtime(serial)
            try:
                rt.last_error = dict(error_payload) if isinstance(error_payload, dict) else {"raw": error_payload}
            except Exception:
                rt.last_error = {"raw": str(error_payload)}
            rt.last_update = datetime.utcnow()
            return rt


global_store = AGVStateStore()