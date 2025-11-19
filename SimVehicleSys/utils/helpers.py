from __future__ import annotations
import datetime as _dt
import json
from dataclasses import asdict, is_dataclass
from typing import Any, List, Optional
import math


def get_timestamp() -> str:
    now = _dt.datetime.utcnow()
    return now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def get_topic_type(path: str) -> str:
    return path.rsplit("/", 1)[-1] if "/" in path else path


def canonicalize_map_id(map_id: Optional[str]) -> str:
    """Normalize map identifier to plain name without directories or extension.

    Examples:
    - "VehicleMap/test1.scene" -> "test1"
    - "ViewerMap\\test2.scene" -> "test2"
    - "test3" -> "test3"
    - None/empty -> "default"
    """
    s = str(map_id or "").strip()
    if not s:
        return "default"
    s = s.replace("\\", "/")
    fname = s.split("/")[-1]
    if fname.lower().endswith(".scene"):
        fname = fname[:-6]
    return fname or "default"


def iterate_position(current_x: float, current_y: float, target_x: float, target_y: float, speed: float) -> tuple[float, float, float]:
    # 以 Y− 为 0 度参考：使用 atan2(dx, -dy)
    dx = target_x - current_x
    dy = target_y - current_y
    angle = math.atan2(dx, -dy)
    # 防止超步长导致越过目标：将步长限制为“剩余距离”
    remaining = get_distance(current_x, current_y, target_x, target_y)
    step = min(max(0.0, float(speed)), remaining)
    next_x = current_x + step * math.sin(angle)
    next_y = current_y - step * math.cos(angle)
    # 若步长覆盖目标，则直接返回目标坐标
    if step >= remaining - 1e-12:
        return (target_x, target_y, angle)
    return (next_x, next_y, angle)


def get_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x1 - x2, y1 - y2)


def basis_functions(degree: int, knot_vector: List[float], u: float) -> List[float]:
    n = len(knot_vector) - degree - 1
    basis = [0.0] * n
    span = degree
    for i in range(degree, len(knot_vector) - 1):
        if knot_vector[i] <= u < knot_vector[i + 1]:
            span = i
            break
    if u >= knot_vector[-1]:
        span = len(knot_vector) - degree - 2
    basis[span] = 1.0
    for k in range(1, degree + 1):
        temp = [0.0] * n
        for j in range(span - k, span + 1):
            if j >= n:
                continue
            saved = 0.0
            if basis[j] != 0.0:
                denom = knot_vector[j + k] - knot_vector[j]
                if denom != 0.0:
                    left = (u - knot_vector[j]) / denom
                    saved = basis[j] * left
            if j < n - 1 and basis[j + 1] != 0.0:
                denom = knot_vector[j + k + 1] - knot_vector[j + 1]
                if denom != 0.0:
                    right = (knot_vector[j + k + 1] - u) / denom
                    saved += basis[j + 1] * right
            temp[j] = saved
        basis = temp
    return basis


def evaluate_nurbs_with_tangent(trajectory: Any, u: float) -> tuple[float, float, float, float, float, bool]:
    basis = basis_functions(int(trajectory.degree), list(trajectory.knot_vector), u)
    x = y = theta = total_weight = theta_weight_sum = 0.0
    has_explicit_theta = False
    for i, weight in enumerate(basis):
        if weight > 0.0:
            cp = trajectory.control_points[i]
            point_weight = cp.weight if cp.weight is not None else 1.0
            w = weight * point_weight
            x += cp.x * w
            y += cp.y * w
            if cp.orientation is not None:
                theta += cp.orientation * w
                theta_weight_sum += w
                has_explicit_theta = True
            total_weight += w
    if total_weight > 0.0:
        x /= total_weight
        y /= total_weight
        if has_explicit_theta and theta_weight_sum > 0.0:
            theta /= theta_weight_sum
    delta = 0.001
    u_next = min(u + delta, 1.0)
    basis_next = basis_functions(int(trajectory.degree), list(trajectory.knot_vector), u_next)
    x_next = y_next = total_weight_next = 0.0
    for i, weight in enumerate(basis_next):
        if weight > 0.0:
            cp = trajectory.control_points[i]
            point_weight = cp.weight if cp.weight is not None else 1.0
            w = weight * point_weight
            x_next += cp.x * w
            y_next += cp.y * w
            total_weight_next += w
    if total_weight_next > 0.0:
        x_next /= total_weight_next
        y_next /= total_weight_next
    tangent_x = x_next - x
    tangent_y = y_next - y
    # 与全局约定一致：0 度朝向 Y−（使用 atan2(dx, -dy)）
    tangent_theta = math.atan2(tangent_x, -tangent_y)
    return (x, y, theta, tangent_x, tangent_theta, has_explicit_theta)


def find_closest_parameter(trajectory: Any, current_x: float, current_y: float) -> float:
    closest_u = 0.0
    min_distance = float("inf")
    num_samples = 100
    for i in range(num_samples + 1):
        u = i / num_samples
        x, y, *_ = evaluate_nurbs_with_tangent(trajectory, u)
        dist = get_distance(current_x, current_y, x, y)
        if dist < min_distance:
            min_distance = dist
            closest_u = u
    return closest_u


def iterate_position_with_straight_line(current_x: float, current_y: float, target_x: float, target_y: float, speed: float, trajectory: Any) -> tuple[float, float, float]:
    start_x = trajectory.control_points[0].x
    start_y = trajectory.control_points[0].y
    end_x = trajectory.control_points[1].x
    end_y = trajectory.control_points[1].y
    line_length = get_distance(start_x, start_y, end_x, end_y)
    if line_length > 0.0:
        dot = (current_x - start_x) * (end_x - start_x) + (current_y - start_y) * (end_y - start_y)
        t = max(0.0, min(1.0, dot / (line_length * line_length)))
    else:
        t = 0.0
    distance_to_move = speed
    new_t = min(1.0, t + distance_to_move / (line_length if line_length > 0 else speed))
    new_x = start_x + new_t * (end_x - start_x)
    new_y = start_y + new_t * (end_y - start_y)
    # 直线段朝向使用以 Y− 为参考的角度定义
    theta = math.atan2(end_x - start_x, -(end_y - start_y))
    if get_distance(new_x, new_y, target_x, target_y) <= speed:
        return (target_x, target_y, theta)
    return (new_x, new_y, theta)


def iterate_position_with_trajectory(current_x: float, current_y: float, target_x: float, target_y: float, speed: float, trajectory: Any) -> tuple[float, float, float]:
    # 直线快速路径（degree=1 且两个控制点）
    if int(getattr(trajectory, "degree", 0)) == 1 and len(getattr(trajectory, "control_points", [])) == 2:
        nx, ny, th = iterate_position_with_straight_line(current_x, current_y, target_x, target_y, speed, trajectory)
        # 方向/朝向修正
        try:
            direction = str(getattr(trajectory, "direction", "") or "").upper()
            orient = getattr(trajectory, "orientation", None)
            if direction in ("BACKWARD", "REVERSE", "BACK"):
                th = ((th + math.pi) + math.pi) % (2 * math.pi) - math.pi
            if orient is not None:
                th = float(orient)
        except Exception:
            pass
        return (nx, ny, th)
    # NURBS 通用路径
    current_u = find_closest_parameter(trajectory, current_x, current_y)
    num_samples = 100
    total_length = 0.0
    prev_x = prev_y = 0.0
    first = True
    for i in range(num_samples + 1):
        u = i / num_samples
        x, y, *_ = evaluate_nurbs_with_tangent(trajectory, u)
        if not first:
            total_length += get_distance(prev_x, prev_y, x, y)
        prev_x, prev_y = x, y
        first = False
    parameter_step = (speed / total_length) if total_length > 0 else speed * 0.5
    if int(getattr(trajectory, "degree", 0)) == 2:
        parameter_step = min(parameter_step, 0.05)
    else:
        parameter_step = min(parameter_step, 0.1)
    next_u = min(1.0, current_u + parameter_step)
    next_x, next_y, explicit_theta, _, tangent_theta, has_explicit_theta = evaluate_nurbs_with_tangent(trajectory, next_u)
    final_theta = explicit_theta if has_explicit_theta else tangent_theta
    # 方向/朝向修正
    try:
        direction = str(getattr(trajectory, "direction", "") or "").upper()
        orient = getattr(trajectory, "orientation", None)
        if direction in ("BACKWARD", "REVERSE", "BACK"):
            final_theta = ((final_theta + math.pi) + math.pi) % (2 * math.pi) - math.pi
        if orient is not None and str(getattr(trajectory, "orientation_type", "")).upper() in ("ABSOLUTE", "FIXED", "CONST"):
            final_theta = float(orient)
    except Exception:
        pass
    distance_to_target = get_distance(next_x, next_y, target_x, target_y)
    if distance_to_target <= speed:
        return (target_x, target_y, final_theta)
    if next_u >= 0.99 and distance_to_target > speed:
        # 末段指向目标的朝向也采用以 Y 轴为参考的角度定义
        angle = math.atan2(target_x - next_x, -(target_y - next_y))
        final_x = next_x + speed * math.sin(angle)
        final_y = next_y - speed * math.cos(angle)
        # 方向/朝向修正
        try:
            direction = str(getattr(trajectory, "direction", "") or "").upper()
            orient = getattr(trajectory, "orientation", None)
            if direction in ("BACKWARD", "REVERSE", "BACK"):
                angle = ((angle + math.pi) + math.pi) % (2 * math.pi) - math.pi
            if orient is not None and str(getattr(trajectory, "orientation_type", "")).upper() in ("ABSOLUTE", "FIXED", "CONST"):
                angle = float(orient)
        except Exception:
            pass
        return (final_x, final_y, angle)
    return (next_x, next_y, final_theta)


def _snake_to_camel(name: str) -> str:
    parts = name.split("_")
    return parts[0] + "".join(p.capitalize() for p in parts[1:])


def _convert_keys_to_camel(obj: Any) -> Any:
    if isinstance(obj, dict):
        return { _snake_to_camel(k): _convert_keys_to_camel(v) for k, v in obj.items() }
    elif isinstance(obj, list):
        return [ _convert_keys_to_camel(x) for x in obj ]
    else:
        return obj


def to_camel_json(obj: Any) -> str:
    if is_dataclass(obj):
        data = asdict(obj)
    elif hasattr(obj, "to_dict"):
        data = obj.to_dict()  # type: ignore
    else:
        data = obj if isinstance(obj, dict) else obj.__dict__
    camel_data = _convert_keys_to_camel(data)
    return json.dumps(camel_data, ensure_ascii=False)

# === Trajectory normalization and type helpers ===

def trajectory_type_of(trajectory: Any) -> str:
    """
    获取轨迹类型字符串：返回 "Straight" | "CubicBezier" | "INFPNURBS" | "NURBS" | "Unknown"。
    - 支持 dataclass Trajectory（degree/knot_vector/control_points），或 dict 含 type 字段。
    - 兼容别名："CubicBeizer" 视为 "CubicBezier"。
    """
    try:
        t = None
        if hasattr(trajectory, "type"):
            t = getattr(trajectory, "type")
        elif isinstance(trajectory, dict):
            t = trajectory.get("type")
        if isinstance(t, str) and t:
            up = t.strip()
            if up == "CubicBeizer":
                return "CubicBezier"
            return up
        # 根据 degree 推断 NURBS
        deg = int(getattr(trajectory, "degree", getattr(trajectory, "Degree", 0)) or 0)
        kv = getattr(trajectory, "knot_vector", getattr(trajectory, "knotVector", None))
        if deg >= 1 and (kv is not None):
            return "NURBS"
    except Exception:
        pass
    return "Unknown"

def _default_knot_vector(degree: int, cp_count: int) -> List[float]:
    """生成均匀结点向量（Open Uniform），用于 NURBS 默认值。"""
    m = degree + cp_count + 1
    if degree <= 0 or cp_count <= 0:
        return [0.0, 1.0]
    kv: List[float] = []
    for i in range(m):
        if i <= degree:
            kv.append(0.0)
        elif i >= m - degree - 1:
            kv.append(1.0)
        else:
            kv.append((i - degree) / (m - 2 * degree - 1))
    return kv

def normalize_trajectory(trajectory: Any) -> Any:
    """
    规范化轨迹：
    - Straight：确保两个控制点存在。
    - CubicBezier：提供4个控制点；若 knotVector 缺失，转换为等价 NURBS（degree=3）。
    - INFPNURBS/NURBS：若 knotVector 为空，自动生成默认 knot 向量；若权重缺失，设为1。
    返回原对象（就地补全）以保持现有调用兼容。
    """
    try:
        t = trajectory_type_of(trajectory)
        cps = list(getattr(trajectory, "control_points", getattr(trajectory, "controlPoints", [])) or [])
        # 补齐 weight
        for cp in cps:
            try:
                if isinstance(cp, dict):
                    cp.setdefault("weight", 1.0)
                elif hasattr(cp, "weight") and getattr(cp, "weight") is None:
                    setattr(cp, "weight", 1.0)
            except Exception:
                pass
        if t == "Straight":
            if len(cps) < 2:
                while len(cps) < 2:
                    # 追加零点或复制末点
                    if cps:
                        cps.append(cps[-1])
                    else:
                        cps.append(type("CP", (), {"x": 0.0, "y": 0.0, "weight": 1.0})())
            try:
                setattr(trajectory, "control_points", cps)
            except Exception:
                pass
            return trajectory
        if t in ("INFPNURBS", "NURBS"):
            deg = int(getattr(trajectory, "degree", 0) or 0)
            kv = list(getattr(trajectory, "knot_vector", getattr(trajectory, "knotVector", [])) or [])
            if not kv:
                kv = _default_knot_vector(deg, len(cps))
                try:
                    setattr(trajectory, "knot_vector", kv)
                except Exception:
                    pass
            return trajectory
        if t == "CubicBezier":
            # 缺 knotVector 时，视作 degree=3 NURBS
            deg = 3
            kv = list(getattr(trajectory, "knot_vector", getattr(trajectory, "knotVector", [])) or [])
            if not kv:
                kv = _default_knot_vector(deg, len(cps))
                try:
                    setattr(trajectory, "degree", deg)
                    setattr(trajectory, "knot_vector", kv)
                except Exception:
                    pass
            return trajectory
    except Exception:
        pass
    return trajectory

def evaluate_bezier(control_points: List[tuple[float, float]], u: float) -> tuple[float, float]:
    """简单的三次贝塞尔评估。control_points 需为4个 (x,y)。"""
    if len(control_points) != 4:
        return control_points[-1] if control_points else (0.0, 0.0)
    P0, P1, P2, P3 = control_points
    x = (1 - u) ** 3 * P0[0] + 3 * (1 - u) ** 2 * u * P1[0] + 3 * (1 - u) * u ** 2 * P2[0] + u ** 3 * P3[0]
    y = (1 - u) ** 3 * P0[1] + 3 * (1 - u) ** 2 * u * P1[1] + 3 * (1 - u) * u ** 2 * P2[1] + u ** 3 * P3[1]
    return (x, y)

# === Collision geometry helpers ===

def compute_agv_base_polygon(length: float, width: float, pose: tuple[float, float, float]) -> list[tuple[float, float]]:
    """根据车体长度/宽度与位姿 (x,y,theta)，返回底盘矩形四角的世界坐标点。"""
    x, y, theta = pose
    hl = float(length) / 2.0
    hw = float(width) / 2.0
    local = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    out: list[tuple[float, float]] = []
    s = math.sin(theta)
    c = math.cos(theta)
    for lx, ly in local:
        wx = x + lx * c + ly * s
        wy = y - lx * s + ly * c
        out.append((wx, wy))
    return out

def compute_outer_bounding_rect(points: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    """给定多边形点集，返回外包络矩形 (min_x, min_y, max_x, max_y)。"""
    if not points:
        return (0.0, 0.0, 0.0, 0.0)
    xs = [px for px, _ in points]
    ys = [py for _, py in points]
    return (min(xs), min(ys), max(xs), max(ys))

def expand_rect(rect: tuple[float, float, float, float], margin: float) -> tuple[float, float, float, float]:
    """按安全距离放大矩形。"""
    x1, y1, x2, y2 = rect
    m = float(margin)
    return (x1 - m, y1 - m, x2 + m, y2 + m)

def rects_overlap(r1: tuple[float, float, float, float], r2: tuple[float, float, float, float]) -> bool:
    """快速二维矩形重叠检测。"""
    return not (r1[2] < r2[0] or r2[2] < r1[0] or r1[3] < r2[1] or r2[3] < r1[1])