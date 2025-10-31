from __future__ import annotations
import asyncio
import math
from typing import Optional, Any, List, Dict, Tuple

from backend.schemas import AGVRuntime, DynamicConfigUpdate, Position
from SimVehicleSys.protocol.vda_2_0_0.state import Information, InfoReference, Load
from SimVehicleSys.protocol.vda5050_common import LoadDimensions, BoundingBoxReference
from pathlib import Path
import time


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


def execute_pallet_action_in_sim(sim_vehicle: Any, action_type: str, action_parameters: Optional[List[Any]] = None) -> None:
    """在仿真器状态中写入托盘动作信息，并维护载荷列表，供前端消费。

    - action_type: "JackLoad" 或 "JackUnload"。
    - action_parameters: 列表，包含带 key/value 的参数，其中可能包括：
      - operation: 同 action_type
      - recfile: 例如 "shelf/BD1.shelf"
    """
    try:
        atype = str(action_type or "").strip()
        if atype not in ("JackLoad", "JackUnload"):
            return
        def pval(key: str, default: Any = None) -> Any:
            try:
                for p in (action_parameters or []):
                    if str(getattr(p, "key", "")) == key:
                        return getattr(p, "value", None)
            except Exception:
                pass
            return default
        op = str(pval("operation", atype))
        recfile = str(pval("recfile", ""))
        try:
            info = Information(
                info_type="PalletAction",
                info_references=[
                    InfoReference(reference_key="operation", reference_value=op),
                    InfoReference(reference_key="recfile", reference_value=recfile),
                ],
                info_description=None,
                info_level="INFO",
            )
            try:
                sim_vehicle.state.information = [x for x in sim_vehicle.state.information if x.info_type != "PalletAction"]
            except Exception:
                sim_vehicle.state.information = []
            sim_vehicle.state.information.append(info)
        except Exception:
            pass
        try:
            if atype == "JackLoad":
                # 解析托盘/货架模型文件，填充更完整的载荷信息
                lid = Path(recfile).stem or "shelf"
                ltype = "shelf"
                ldim = None
                bbox = None
                try:
                    # 解析 JSON 模型文件（例如 SimVehicleSys/shelf/BD1.shelf）
                    sim_root = Path(__file__).resolve().parents[1]  # 指向 SimVehicleSys 目录
                    candidate = None
                    if recfile:
                        candidate = sim_root / recfile
                        if not candidate.exists():
                            # 允许传入 "shelf/BD1.shelf" 或仅文件名，均可解析
                            candidate = sim_root / "shelf" / Path(recfile).name
                    import json
                    data = json.loads(candidate.read_text(encoding="utf-8")) if (candidate and candidate.exists()) else {}
                    ltype = str(data.get("type", ltype))
                    name = str(data.get("name", lid))
                    lid = name or lid
                    fp = data.get("footprint", {})
                    w = float(fp.get("width_m", 0.0))
                    l = float(fp.get("length_m", 0.0))
                    h = float(fp.get("height_m", 0.0)) if fp.get("height_m") is not None else None
                    if l > 0.0 or w > 0.0 or (h or 0.0) > 0.0:
                        ldim = LoadDimensions(length=l, width=w, height=h)
                    # 参考框设置为车辆坐标系原点（顶面），theta 使用 0
                    bbox = BoundingBoxReference(x=0.0, y=0.0, z=0.0, theta=0.0)
                except Exception:
                    pass
                sim_vehicle.state.loads = [
                    Load(
                        load_id=lid,
                        load_type=ltype,
                        load_position="top",
                        bounding_box_reference=bbox,
                        load_dimensions=ldim,
                    )
                ]
            else:
                sim_vehicle.state.loads = []
        except Exception:
            pass
        try:
            sim_vehicle.action_start_time = time.time()
        except Exception:
            pass
    except Exception:
        pass


def execute_charging_action_in_sim(sim_vehicle: Any, action_parameters: Optional[List[Any]] = None) -> None:
    """在仿真器状态中标记充电请求，并写入信息供前端展示。

    - action_parameters: 可选参数列表（key/value），如 { source: 'CPMenu' }。
    """
    try:
        # 信息上报：记录触发源与时间
        src = None
        try:
            for p in (action_parameters or []):
                if str(getattr(p, "key", getattr(p, "key", ""))) == "source":
                    src = getattr(p, "value", None)
                    break
        except Exception:
            pass
        info = Information(
            info_type="ChargingTask",
            info_references=[
                InfoReference(reference_key="source", reference_value=str(src or "order")),
            ],
            info_description=None,
            info_level="INFO",
        )
        try:
            sim_vehicle.state.information = [x for x in sim_vehicle.state.information if x.info_type != "ChargingTask"]
        except Exception:
            sim_vehicle.state.information = []
        sim_vehicle.state.information.append(info)
    except Exception:
        pass
    try:
        # 设置充电请求标志；由 BatteryManager 决定何时真的进入充电
        setattr(sim_vehicle, "charging_requested", True)
    except Exception:
        pass
    try:
        # 直接提示 charging 状态为 true（立即可见）；BatteryManager 会在下一周期统一维护
        bs = getattr(getattr(sim_vehicle, "state", None), "battery_state", None)
        if bs:
            bs.charging = True
    except Exception:
        pass
    try:
        sim_vehicle.action_start_time = time.time()
    except Exception:
        pass