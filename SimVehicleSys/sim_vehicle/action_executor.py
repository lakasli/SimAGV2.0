from __future__ import annotations
import asyncio
import math
from typing import Optional, Any, List, Dict, Tuple

from backend.schemas import AGVRuntime, DynamicConfigUpdate, Position


async def execute_translation_movement(
    serial: str,
    dx: float,
    dy: float,
    agv_mgr: Any,
    ws_mgr: Any,
    movement_state: Optional[str] = None,
) -> Optional[AGVRuntime]:
    rt = agv_mgr.move_translate(serial, dx, dy, movement_state)
    try:
        status = agv_mgr.get_status(serial)
    except Exception:
        status = None
    if status:
        pass
    return rt


async def execute_rotation_movement(
    serial: str,
    dtheta: float,
    agv_mgr: Any,
    ws_mgr: Any,
) -> Optional[AGVRuntime]:
    rt = agv_mgr.move_rotate(serial, dtheta)
    try:
        status = agv_mgr.get_status(serial)
    except Exception:
        status = None
    if status:
        pass
    return rt


def _normalize_angle(a: float) -> float:
    return ((a + math.pi) % (2 * math.pi)) - math.pi

def _step_towards(current: float, target: float, max_delta: float) -> float:
    try:
        diff = _normalize_angle(target - current)
        if abs(diff) <= max_delta:
            return current + diff
        return current + math.copysign(max_delta, diff)
    except Exception:
        return target


async def execute_points_movement(
    serial: str,
    points: List[Dict[str, float]],
    agv_mgr: Any,
    ws_mgr: Any,
    delay_sec: float = 0.05,
) -> None:
    def _get_current_pose() -> Tuple[float, float, float]:
        try:
            st = agv_mgr.get_status(serial)
        except Exception:
            st = None
        x = float(getattr(getattr(st, "position", None), "x", 0.0)) if st else 0.0
        y = float(getattr(getattr(st, "position", None), "y", 0.0)) if st else 0.0
        theta = float(getattr(getattr(st, "position", None), "theta", 0.0)) if st else 0.0
        return x, y, theta

    def _bezier_control_points(sx: float, sy: float, stheta: float, ex: float, ey: float, etheta: float) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        P0 = (sx, sy)
        P3 = (ex, ey)
        dx = ex - sx
        dy = ey - sy
        dist = math.hypot(dx, dy)
        k = max(0.2, min(dist * 0.4, 2.0))
        P1 = (sx + k * math.sin(stheta), sy + k * math.cos(stheta))
        P2 = (ex - k * math.sin(etheta), ey - k * math.cos(etheta))
        return P0, P1, P2, P3

    def _bezier_point(P0, P1, P2, P3, t: float) -> Tuple[float, float]:
        u = 1.0 - t
        x = (u**3) * P0[0] + 3 * (u**2) * t * P1[0] + 3 * u * (t**2) * P2[0] + (t**3) * P3[0]
        y = (u**3) * P0[1] + 3 * (u**2) * t * P1[1] + 3 * u * (t**2) * P2[1] + (t**3) * P3[1]
        return x, y

    def _bezier_tangent_theta(P0, P1, P2, P3, t: float) -> float:
        u = 1.0 - t
        vx = 3 * (
            (u**2) * (P1[0] - P0[0]) + 2 * u * t * (P2[0] - P1[0]) + (t**2) * (P3[0] - P2[0])
        )
        vy = 3 * (
            (u**2) * (P1[1] - P0[1]) + 2 * u * t * (P2[1] - P1[1]) + (t**2) * (P3[1] - P2[1])
        )
        return math.atan2(vx, vy)

    for p in points:
        tx = float(p.get("x", 0.0))
        ty = float(p.get("y", 0.0))
        target_theta = float(p.get("theta", 0.0))

        sx, sy, current_theta = _get_current_pose()
        if "theta" not in p:
            target_theta = math.atan2(tx - sx, ty - sy)
        angle_diff = _normalize_angle(target_theta - current_theta)

        if abs(angle_diff) > 0.1:
            max_step = 0.08
            while abs(angle_diff) > 0.1:
                rot_step = math.copysign(min(abs(angle_diff), max_step), angle_diff)
                try:
                    agv_mgr.move_rotate(serial, rot_step)
                except Exception:
                    pass
                await asyncio.sleep(delay_sec)
                sx, sy, current_theta = _get_current_pose()
                angle_diff = _normalize_angle(target_theta - current_theta)
            try:
                agv_mgr.update_dynamic(serial, DynamicConfigUpdate(position=Position(x=tx, y=ty)))
            except Exception:
                pass
            await asyncio.sleep(delay_sec)
        else:
            P0, P1, P2, P3 = _bezier_control_points(sx, sy, current_theta, tx, ty, target_theta)
            dist = math.hypot(tx - sx, ty - sy)
            steps = max(12, min(60, int(dist * 20))) if dist > 0.0 else 12
            for i in range(1, steps + 1):
                t = i / steps
                bx, by = _bezier_point(P0, P1, P2, P3, t)
                btheta = _bezier_tangent_theta(P0, P1, P2, P3, t)
                _, _, cur_theta = _get_current_pose()
                smooth_theta = _step_towards(cur_theta, btheta, max_delta=0.15)
                try:
                    agv_mgr.update_dynamic(
                        serial,
                        DynamicConfigUpdate(position=Position(x=bx, y=by, theta=smooth_theta)),
                    )
                except Exception:
                    pass
                await asyncio.sleep(delay_sec)