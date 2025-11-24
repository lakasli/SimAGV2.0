from __future__ import annotations
import asyncio
import math
from typing import Optional, Any, List, Dict, Tuple

from backend.schemas import AGVRuntime, DynamicConfigUpdate, Position
from SimVehicleSys.config.settings import get_config
from SimVehicleSys.protocol.vda_2_0_0.state import Information, InfoReference, Load
from SimVehicleSys.protocol.vda5050_common import LoadDimensions, BoundingBoxReference
from pathlib import Path
import time
from SimVehicleSys.sim_vehicle.state_manager import global_store


async def execute_translation_movement(
    serial: str,
    dx: float,
    dy: float,
    agv_mgr: Any,
    ws_mgr: Any,
    movement_state: Optional[str] = None,
) -> Optional[AGVRuntime]:
    """执行平移运动并尝试获取最新状态。

    参数:
    - serial: 车辆序列号。
    - dx: X 方向位移，单位米。
    - dy: Y 方向位移，单位米。
    - agv_mgr: AGV 管理器，需支持 `move_translate` 与 `get_status`。
    - ws_mgr: WebSocket 管理器（当前未用，预留）。
    - movement_state: 可选的移动状态标签，传递给管理器。

    返回:
    - `AGVRuntime` 或 `None`，由管理器返回的运行态对象。
    """
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
    """执行旋转运动并尝试获取最新状态。

    参数:
    - serial: 车辆序列号。
    - dtheta: 旋转角增量，单位弧度。
    - agv_mgr: AGV 管理器，需支持 `move_rotate` 与 `get_status`。
    - ws_mgr: WebSocket 管理器（当前未用，预留）。

    返回:
    - `AGVRuntime` 或 `None`。
    """
    rt = agv_mgr.move_rotate(serial, dtheta)
    try:
        status = agv_mgr.get_status(serial)
    except Exception:
        status = None
    if status:
        pass
    return rt


def _normalize_angle(a: float) -> float:
    """将角度归一化到 (-π, π] 区间。

    参数:
    - a: 输入角度（弧度）。

    返回:
    - 归一化后的角度（弧度）。
    """
    return ((a + math.pi) % (2 * math.pi)) - math.pi

def _step_towards(current: float, target: float, max_delta: float) -> float:
    """在最大步长约束下朝目标角度前进。

    参数:
    - current: 当前角度（弧度）。
    - target: 目标角度（弧度）。
    - max_delta: 最大步进（弧度）。

    返回:
    - 下一步角度（弧度）。若异常则返回 `target`。
    """
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
    max_speed: Optional[float] = None,
    rotation_allowed: Optional[bool] = None,
    max_rotation_speed: Optional[float] = None,
    allowed_deviation_xy: Optional[float] = None,
    allowed_deviation_theta: Optional[float] = None,
) -> None:
    """沿点序列执行平滑运动（含旋转/平移）。

    行为:
    - 读取 `sim_time_scale` 做时间/步长缩放。
    - 若当前朝向与目标朝向差异较大，先按步进旋转至目标。
    - 否则使用三次贝塞尔曲线在起止姿态间平滑插值位置与朝向。
    - 通过 `agv_mgr.update_dynamic` 同步位置与姿态；每步 `await asyncio.sleep(delay_sec)`。

    参数:
    - serial: 车辆序列号。
    - points: 点列表，每个点包含 `x`, `y`，可选 `theta`（弧度）。
    - agv_mgr: AGV 管理器，需支持 `get_status` 与 `update_dynamic`。
    - ws_mgr: WebSocket 管理器（当前未用，预留）。
    - delay_sec: 每步延迟秒数。

    返回:
    - 无。
    """
    cfg = get_config()
    try:
        scale = max(0.0001, float(cfg.settings.sim_time_scale))
    except Exception:
        scale = 1.0
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
        P1 = (sx + k * math.cos(stheta), sy + k * math.sin(stheta))
        P2 = (ex - k * math.cos(etheta), ey - k * math.sin(etheta))
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
        return - (math.atan2(vx, vy) + math.pi / 2)

    for p in points:
        tx = float(p.get("x", 0.0))
        ty = float(p.get("y", 0.0))
        target_theta = float(p.get("theta", 0.0))

        sx, sy, current_theta = _get_current_pose()
        if "theta" not in p:
            target_theta = - (math.atan2(tx - sx, ty - sy) + math.pi / 2)
        angle_diff = _normalize_angle(target_theta - current_theta)
        
        # 旋转：遵循允许旋转与最大旋转速度、角度/距离偏差阈值
        rotate_ok = True if rotation_allowed is None else bool(rotation_allowed)
        theta_eps = 0.1 if allowed_deviation_theta is None else float(allowed_deviation_theta)
        xy_eps = 0.05 if allowed_deviation_xy is None else max(0.0, float(allowed_deviation_xy))
        dist_to_target = math.hypot(tx - sx, ty - sy)
        # 若当前位置与目标已在允许范围内，直接跳过该点
        if dist_to_target <= xy_eps and abs(angle_diff) <= theta_eps:
            await asyncio.sleep(delay_sec)
            continue
        if rotate_ok and abs(angle_diff) > theta_eps:
            # 每步最大旋转：优先使用 max_rotation_speed（rad/s）与 delay_sec 推导
            rlimit = 0.08 * scale
            try:
                if max_rotation_speed is not None:
                    rlimit = max(1e-6, float(max_rotation_speed) * float(delay_sec))
            except Exception:
                pass
            while abs(angle_diff) > theta_eps:
                rot_step = math.copysign(min(abs(angle_diff), rlimit), angle_diff)
                try:
                    agv_mgr.move_rotate(serial, rot_step)
                except Exception:
                    pass
                await asyncio.sleep(delay_sec)
                sx, sy, current_theta = _get_current_pose()
                angle_diff = _normalize_angle(target_theta - current_theta)
            # 旋转完成后，仅在距离超出阈值时才进行精确 XY 对齐
            try:
                sx2, sy2, _ = _get_current_pose()
                if math.hypot(tx - sx2, ty - sy2) > xy_eps:
                    agv_mgr.update_dynamic(serial, DynamicConfigUpdate(position=Position(x=tx, y=ty)))
            except Exception:
                pass
            await asyncio.sleep(delay_sec)
        else:
            P0, P1, P2, P3 = _bezier_control_points(sx, sy, current_theta, tx, ty, target_theta)
            dist = math.hypot(tx - sx, ty - sy)
            # 平移步数：遵循最大速度（米/秒），每步位移不超过 max_speed * delay_sec
            if max_speed is not None and max_speed > 0:
                try:
                    max_step_dist = float(max_speed) * float(delay_sec)
                    steps = max(12, min(200, int(math.ceil(dist / max_step_dist)))) if dist > 0.0 else 12
                except Exception:
                    steps = max(12, min(60, int(dist * 20))) if dist > 0.0 else 12
            else:
                steps = max(12, min(60, int(dist * 20))) if dist > 0.0 else 12
            for i in range(1, steps + 1):
                t = i / steps
                bx, by = _bezier_point(P0, P1, P2, P3, t)
                btheta = _bezier_tangent_theta(P0, P1, P2, P3, t)
                _, _, cur_theta = _get_current_pose()
                # 若禁止旋转，则保持当前朝向
                if rotate_ok:
                    smooth_theta = _step_towards(cur_theta, btheta, max_delta=0.15 * scale)
                else:
                    smooth_theta = cur_theta
                try:
                    agv_mgr.update_dynamic(
                        serial,
                        DynamicConfigUpdate(position=Position(x=bx, y=by, theta=smooth_theta)),
                    )
                except Exception:
                    pass
                await asyncio.sleep(delay_sec)
                # 若已在允许的 XY 距离范围内，提前结束该点的平移
                if math.hypot(tx - bx, ty - by) <= xy_eps:
                    break


def execute_pallet_action_in_sim(sim_vehicle: Any, action_type: str, action_parameters: Optional[List[Any]] = None) -> None:
    """在仿真器状态中写入托盘动作信息，并维护载荷列表，供前端消费。

    - action_type: "JackLoad" 或 "JackUnload"。
    - action_parameters: 列表，包含带 key/value 的参数，其中可能包括：
      - operation: 同 action_type
      - recfile: 例如 "shelf/BD1.shelf"
    """
    try:
        atype = str(action_type or "").strip()
        at_l = atype.lower()
        if at_l not in ("jackload", "jackunload", "pick", "drop"):
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
            if at_l in ("jackload", "pick"):
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
                try:
                    default_h = 0.1
                    try:
                        if not action_parameters:
                            cfg = getattr(sim_vehicle, "config", None)
                            s = getattr(cfg, "settings", None)
                            default_h = float(getattr(s, "height_max", default_h))
                    except Exception:
                        pass
                    min_h = None
                    max_h = None
                    try:
                        seq = int(getattr(sim_vehicle.state, "last_node_sequence_id", 0) or 0)
                        edges = list(getattr(sim_vehicle.state, "edge_states", []) or [])
                        for e in edges:
                            try:
                                s = int(getattr(e, "sequence_id", -999) or -999)
                            except Exception:
                                s = -999
                            if s in (seq - 1, seq):
                                if getattr(e, "min_height", None) is not None:
                                    try:
                                        min_h = float(getattr(e, "min_height"))
                                    except Exception:
                                        pass
                                if getattr(e, "max_height", None) is not None:
                                    try:
                                        max_h = float(getattr(e, "max_height"))
                                    except Exception:
                                        pass
                    except Exception:
                        pass
                    h = float(default_h)
                    if min_h is not None:
                        try:
                            h = max(h, float(min_h))
                        except Exception:
                            pass
                    if max_h is not None:
                        try:
                            h = min(h, float(max_h))
                        except Exception:
                            pass
                    sim_vehicle.state.fork_state.fork_height = float(h)
                except Exception:
                    pass
                try:
                    di_arr = [
                        {"id": 0, "source": "normal", "status": False, "valid": True},
                        {"id": 1, "source": "normal", "status": False, "valid": True},
                        {"id": 2, "source": "normal", "status": False, "valid": True},
                        {"id": 3, "source": "normal", "status": False, "valid": True},
                    ]
                    do_arr = [
                        {"id": 0, "status": False},
                    ]
                    di_info = Information(
                        info_type="DI",
                        info_references=[
                            InfoReference(reference_key="DI", reference_value=di_arr),
                        ],
                        info_description="info of DI",
                        info_level="INFO",
                    )
                    do_info = Information(
                        info_type="DO",
                        info_references=[
                            InfoReference(reference_key="DO", reference_value=do_arr),
                        ],
                        info_description="info of DO",
                        info_level="INFO",
                    )
                    try:
                        sim_vehicle.state.information = [x for x in sim_vehicle.state.information if x.info_type not in ("DI", "DO")]
                    except Exception:
                        sim_vehicle.state.information = []
                    sim_vehicle.state.information.append(di_info)
                    sim_vehicle.state.information.append(do_info)
                except Exception:
                    pass
            else:
                sim_vehicle.state.loads = []
                try:
                    default_h = 0.0
                    try:
                        if not action_parameters:
                            cfg = getattr(sim_vehicle, "config", None)
                            s = getattr(cfg, "settings", None)
                            default_h = float(getattr(s, "height_min", default_h))
                    except Exception:
                        pass
                    min_h = None
                    max_h = None
                    try:
                        seq = int(getattr(sim_vehicle.state, "last_node_sequence_id", 0) or 0)
                        edges = list(getattr(sim_vehicle.state, "edge_states", []) or [])
                        for e in edges:
                            try:
                                s = int(getattr(e, "sequence_id", -999) or -999)
                            except Exception:
                                s = -999
                            if s in (seq - 1, seq):
                                if getattr(e, "min_height", None) is not None:
                                    try:
                                        min_h = float(getattr(e, "min_height"))
                                    except Exception:
                                        pass
                                if getattr(e, "max_height", None) is not None:
                                    try:
                                        max_h = float(getattr(e, "max_height"))
                                    except Exception:
                                        pass
                    except Exception:
                        pass
                    h = float(default_h)
                    if min_h is not None:
                        try:
                            h = max(h, float(min_h))
                        except Exception:
                            pass
                    if max_h is not None:
                        try:
                            h = min(h, float(max_h))
                        except Exception:
                            pass
                    sim_vehicle.state.fork_state.fork_height = float(h)
                except Exception:
                    pass
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


def simulate_load_angle_jitter(sim_vehicle: Any, max_delta: float = 0.5, duration_ms: int = 1000) -> None:
    """主动错误注入接口占位：载荷角度抖动。

    保留接口但不执行任何逻辑，后期再实现具体行为。
    """
    pass


# === Synchronous helpers for simulation-driven motion control ===
def applyRotationStepInSim(sim_vehicle: Any, target_theta: float, base_step: float = 0.15) -> bool:
    """按步进将仿真车体朝 `target_theta` 旋转，返回是否已对齐。

    - 使用 `sim_time_scale` 做线性缩放；避免极小值导致停滞，取下限 1e-4。
    - 更新 `state.agv_position` 与 `visualization.agv_position`，并同步到后端 `global_store`。
    """
    try:
        def _norm(a: float) -> float:
            pi = math.pi
            return (a + pi) % (2 * pi) - pi
        vp = getattr(getattr(sim_vehicle, "state", None), "agv_position", None)
        if not vp:
            return True
        cur_theta = float(vp.theta or 0.0)
        angle_diff = _norm(float(target_theta) - cur_theta)
        scale = max(0.0001, float(getattr(getattr(sim_vehicle, "config", None).settings, "sim_time_scale", 1.0)))
        step = float(base_step) * scale
        aligned = abs(angle_diff) <= step
        new_theta = cur_theta + (angle_diff if aligned else math.copysign(step, angle_diff))
        # 统一归一化，避免角度在连续旋转中累计超界
        new_theta = _norm(new_theta)
        # 更新仿真状态与可视化
        vp.theta = new_theta
        try:
            sim_vehicle.visualization.agv_position = vp
        except Exception:
            pass
        # 同步到后端全局状态（若存在）
        try:
            serial = str(getattr(getattr(sim_vehicle, "config", None).vehicle, "serial_number", ""))
            x = float(vp.x)
            y = float(vp.y)
            global_store.update_dynamic(serial, DynamicConfigUpdate(position=Position(x=x, y=y, theta=new_theta)))
        except Exception:
            pass
        return aligned
    except Exception:
        return False


def applyTranslateStepInSim(sim_vehicle: Any, new_x: float, new_y: float) -> None:
    """更新仿真车体的平移位置，并同步到后端 `global_store`。"""
    try:
        vp = getattr(getattr(sim_vehicle, "state", None), "agv_position", None)
        if not vp:
            return
        vp.x = float(new_x)
        vp.y = float(new_y)
        try:
            sim_vehicle.visualization.agv_position = vp
        except Exception:
            pass
        try:
            serial = str(getattr(getattr(sim_vehicle, "config", None).vehicle, "serial_number", ""))
            theta = float(vp.theta or 0.0)
            global_store.update_dynamic(serial, DynamicConfigUpdate(position=Position(x=float(new_x), y=float(new_y), theta=theta)))
        except Exception:
            pass
    except Exception:
        pass
