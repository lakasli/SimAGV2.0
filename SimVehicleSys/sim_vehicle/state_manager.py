from __future__ import annotations
import threading
import math
from datetime import datetime
from typing import Dict, Optional, Any, List
from SimVehicleSys.config.settings import get_config
from SimVehicleSys.utils.logger import setup_logger

from backend.schemas import (
    AGVRuntime,
    DynamicConfigUpdate,
    Position,
    StatusResponse,
    AGVInfo,
)

logger = setup_logger()

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
                        rt.position.theta = _normalize_angle(provided_theta)
                    else:
                        dx = new_x - old_x
                        dy = new_y - old_y
                        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                            movement_dir = math.atan2(dx, -dy)
                            target_heading = movement_dir if rt.movement_state == "forward" else _normalize_angle(movement_dir + math.pi)
                            try:
                                scale = max(0.0001, float(get_config().settings.sim_time_scale))
                            except Exception:
                                scale = 1.0
                            try:
                                new_theta = _normalize_angle(_step_towards(old_theta, target_heading, max_delta=0.15 * scale))
                                logger.info(
                                    f"[STORE_THETA_STEP] old={float(old_theta):.3f} target={float(target_heading):.3f} scale={scale:.2f} max_delta={0.15*scale:.3f} new={float(new_theta):.3f}"
                                )
                                rt.position.theta = new_theta
                            except Exception:
                                rt.position.theta = _normalize_angle(_step_towards(old_theta, target_heading, max_delta=0.15 * scale))
                        else:
                            rt.position.theta = _normalize_angle(old_theta)
                except Exception:
                    rt.position.theta = _normalize_angle(old_theta)
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
            # 当电量为 0 时禁止运动
            try:
                if rt.battery_level is not None and float(rt.battery_level) <= 0:
                    # 记录错误并返回不修改位置
                    try:
                        if not hasattr(rt, "errors") or rt.errors is None:
                            rt.errors = []  # type: ignore[attr-defined]
                        rt.errors.append({
                            "type": "MovementDenied",
                            "reason": "BatteryZero",
                            "message": "Battery is zero; movement blocked"
                        })  # type: ignore[attr-defined]
                        if len(rt.errors) > 20:  # type: ignore[attr-defined]
                            rt.errors = rt.errors[-20:]  # type: ignore[attr-defined]
                    except Exception:
                        pass
                    rt.last_update = datetime.utcnow()
                    return rt
            except Exception:
                pass
            if movement_state is not None:
                mv = str(movement_state).lower()
                if mv in ("forward", "backward"):
                    rt.movement_state = mv
            rt.position.x += dx
            rt.position.y += dy
            try:
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    movement_dir = math.atan2(dx, -dy)
                    if rt.movement_state == "forward":
                        target_heading = movement_dir
                    else:
                        target_heading = _normalize_angle(movement_dir + math.pi)
                    try:
                        scale = max(0.0001, float(get_config().settings.sim_time_scale))
                    except Exception:
                        scale = 1.0
                    rt.position.theta = _normalize_angle(_step_towards(rt.position.theta, target_heading, max_delta=0.2 * scale))
            except Exception:
                pass
            rt.last_update = datetime.utcnow()
            return rt

    def move_rotate(self, serial: str, dtheta: float) -> Optional[AGVRuntime]:
        with self._lock:
            rt = self._ensure_runtime(serial)
            # 当电量为 0 时禁止旋转
            try:
                if rt.battery_level is not None and float(rt.battery_level) <= 0:
                    try:
                        if not hasattr(rt, "errors") or rt.errors is None:
                            rt.errors = []  # type: ignore[attr-defined]
                        rt.errors.append({
                            "type": "MovementDenied",
                            "reason": "BatteryZero",
                            "message": "Battery is zero; rotation blocked"
                        })  # type: ignore[attr-defined]
                        if len(rt.errors) > 20:  # type: ignore[attr-defined]
                            rt.errors = rt.errors[-20:]  # type: ignore[attr-defined]
                    except Exception:
                        pass
                    rt.last_update = datetime.utcnow()
                    return rt
            except Exception:
                pass
            rt.position.theta = _normalize_angle(rt.position.theta + dtheta)
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
                # 始终返回 errors 列表：无错误则为空数组
                errors=list(rt.errors) if getattr(rt, "errors", None) else [],
            )

    def set_error(self, serial: str, error_payload: Dict[str, Any]) -> Optional[AGVRuntime]:
        """追加错误到错误列表，并对同一错误进行去重，用于状态上报携带。"""
        with self._lock:
            rt = self._ensure_runtime(serial)
            # 归一化错误载荷
            try:
                payload = dict(error_payload) if isinstance(error_payload, dict) else {"raw": error_payload}
            except Exception:
                payload = {"raw": str(error_payload)}
            # 构建去重键：
            # - 新格式优先使用 errorType（等价于 code），其次 errorName，再次 errorDescription
            # - 兼容旧格式：优先使用 code，其次 type+reason，再次 type，最后 message
            def _dedup_key(p: Dict[str, Any]) -> str:
                try:
                    # 新格式
                    if "errorType" in p and p["errorType"] is not None:
                        return f"code:{str(p['errorType'])}"
                    en = p.get("errorName")
                    if en is not None:
                        return f"name:{str(en)}"
                    ed = p.get("errorDescription")
                    if ed is not None:
                        return f"desc:{str(ed)}"
                    # 旧格式
                    if "code" in p and p["code"] is not None:
                        return f"code:{str(p['code'])}"
                    t = p.get("type")
                    r = p.get("reason")
                    if t is not None and r is not None:
                        return f"type:{str(t)}|reason:{str(r)}"
                    if t is not None:
                        return f"type:{str(t)}"
                    m = p.get("message")
                    if m is not None:
                        return f"message:{str(m)}"
                except Exception:
                    pass
                return f"raw:{str(p)}"
            # 追加到错误列表，并限制最大长度，避免无限增长
            try:
                if not hasattr(rt, "errors") or rt.errors is None:
                    rt.errors = []  # type: ignore[attr-defined]
                key_new = _dedup_key(payload)
                # 查找是否已有相同错误；如有则更新原记录而非重复插入
                idx = None
                try:
                    for i, e in enumerate(rt.errors):  # type: ignore[attr-defined]
                        if _dedup_key(e) == key_new:
                            idx = i
                            break
                except Exception:
                    idx = None
                if idx is not None:
                    try:
                        # 合并更新：以新载荷覆盖旧载荷的同名字段（保留已有未提供字段）
                        rt.errors[idx].update(payload)  # type: ignore[attr-defined]
                    except Exception:
                        rt.errors[idx] = payload  # type: ignore[attr-defined]
                else:
                    rt.errors.append(payload)  # type: ignore[attr-defined]
                # 仅保留最近的 20 条
                if len(rt.errors) > 20:  # type: ignore[attr-defined]
                    rt.errors = rt.errors[-20:]  # type: ignore[attr-defined]
            except Exception:
                pass
            rt.last_update = datetime.utcnow()
            return rt


global_store = AGVStateStore()