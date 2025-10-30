from __future__ import annotations
import datetime as _dt
import json
from dataclasses import asdict, is_dataclass
from typing import Any, List
import math


def get_timestamp() -> str:
    now = _dt.datetime.utcnow()
    return now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def get_topic_type(path: str) -> str:
    return path.rsplit("/", 1)[-1] if "/" in path else path


def iterate_position(current_x: float, current_y: float, target_x: float, target_y: float, speed: float) -> tuple[float, float, float]:
    angle = math.atan2(target_y - current_y, target_x - current_x)
    next_x = current_x + speed * math.cos(angle)
    next_y = current_y + speed * math.sin(angle)
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
    tangent_theta = math.atan2(tangent_y, tangent_x)
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
    theta = math.atan2(end_y - start_y, end_x - start_x)
    if get_distance(new_x, new_y, target_x, target_y) <= speed:
        return (target_x, target_y, theta)
    return (new_x, new_y, theta)


def iterate_position_with_trajectory(current_x: float, current_y: float, target_x: float, target_y: float, speed: float, trajectory: Any) -> tuple[float, float, float]:
    if int(trajectory.degree) == 1 and len(trajectory.control_points) == 2:
        return iterate_position_with_straight_line(current_x, current_y, target_x, target_y, speed, trajectory)
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
    if int(trajectory.degree) == 2:
        parameter_step = min(parameter_step, 0.05)
    else:
        parameter_step = min(parameter_step, 0.1)
    next_u = min(1.0, current_u + parameter_step)
    next_x, next_y, explicit_theta, _, tangent_theta, has_explicit_theta = evaluate_nurbs_with_tangent(trajectory, next_u)
    final_theta = explicit_theta if has_explicit_theta else tangent_theta
    distance_to_target = get_distance(next_x, next_y, target_x, target_y)
    if distance_to_target <= speed:
        return (target_x, target_y, final_theta)
    if next_u >= 0.99 and distance_to_target > speed:
        angle = math.atan2(target_y - next_y, target_x - next_x)
        final_x = next_x + speed * math.cos(angle)
        final_y = next_y + speed * math.sin(angle)
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