from __future__ import annotations
from typing import Iterable, Any, List, Tuple, Optional, Dict
import math
import json
from pathlib import Path

from ..utils.helpers import (
    compute_agv_base_polygon,
    compute_outer_bounding_rect,
)

def compute_front_radar(x: float, y: float, theta: float, length_m: float, width_m: float, forward_offset_m: float,
                        fov_deg: float = 70.0, radius_m: float = 0.8) -> dict:
    # 车头顶点（半圆顶部）在局部坐标为 (0, -length/2)
    # 世界坐标：先从车体中心按前向偏移 forward_offset_m，再到车头顶点偏移 length/2
    cx = float(x) - math.cos(float(theta)) * float(forward_offset_m + (length_m / 2.0))
    cy = float(y) - math.sin(float(theta)) * float(forward_offset_m + (length_m / 2.0))
    return {
        "origin": {"x": cx, "y": cy},
        "theta": float(theta),
        "fovDeg": float(fov_deg),
        "radius": float(radius_m),
    }

def oriented_rect_polygon(center: tuple[float, float], length_m: float, width_m: float, theta: float) -> list[tuple[float, float]]:
    cx, cy = center
    hl = float(length_m) / 2.0
    hw = float(width_m) / 2.0
    local = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    s = math.cos(theta)
    c = math.sin(theta)
    out: list[tuple[float, float]] = []
    for lx, ly in local:
        wx = cx + lx * c + ly * s
        wy = cy - lx * s + ly * c
        out.append((wx, wy))
    return out

def sector_polygon(origin: tuple[float, float], theta: float, fov_deg: float, radius_m: float, segments: int = 48) -> list[tuple[float, float]]:
    ox, oy = origin
    half = (float(fov_deg) * math.pi / 180.0) / 2.0
    alpha = float(theta) + math.pi
    pts: list[tuple[float, float]] = [(ox, oy)]
    for i in range(segments + 1):
        a = alpha - half + (2 * half) * (i / segments)
        px = ox + math.cos(a) * float(radius_m)
        py = oy + math.sin(a) * float(radius_m)
        pts.append((px, py))
    return pts

def _project(poly: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    ax, ay = axis
    dots = [px * ax + py * ay for (px, py) in poly]
    return (min(dots), max(dots))

def _edges(poly: list[tuple[float, float]]) -> list[tuple[float, float]]:
    eds: list[tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        eds.append((x2 - x1, y2 - y1))
    return eds

def polygons_overlap(poly_a: list[tuple[float, float]], poly_b: list[tuple[float, float]]) -> bool:
    for ed in _edges(poly_a) + _edges(poly_b):
        ex, ey = ed
        # 法向量
        ax, ay = -ey, ex
        # 归一化避免数值影响
        norm = math.hypot(ax, ay) or 1.0
        ax /= norm
        ay /= norm
        a_min, a_max = _project(poly_a, (ax, ay))
        b_min, b_max = _project(poly_b, (ax, ay))
        if a_max < b_min or b_max < a_min:
            return False
    return True

def _point_in_polygon(pt: tuple[float, float], poly: list[tuple[float, float]]) -> bool:
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cond = ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1)
        if cond:
            inside = not inside
    return inside

def _point_to_segment_distance(pt: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    px, py = pt
    x1, y1 = a
    x2, y2 = b
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx = x1 + t * dx
    cy = y1 + t * dy
    return math.hypot(px - cx, py - cy)

def min_distance_point_to_polygon(pt: tuple[float, float], poly: list[tuple[float, float]]) -> float:
    if not poly:
        return float('inf')
    if _point_in_polygon(pt, poly):
        return 0.0
    n = len(poly)
    md = float('inf')
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        d = _point_to_segment_distance(pt, a, b)
        if d < md:
            md = d
    return md

def circle_polygon_overlap(origin: tuple[float, float], radius_m: float, poly: list[tuple[float, float]]) -> bool:
    if _point_in_polygon(origin, poly):
        return True
    return min_distance_point_to_polygon(origin, poly) <= float(radius_m)

def _cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx

def _intersect_segment_with_line(s: tuple[float, float], e: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    sx, sy = s
    ex, ey = e
    ax_, ay_ = a
    bx_, by_ = b
    rx, ry = (bx_ - ax_), (by_ - ay_)
    sxv, syv = (ex - sx), (ey - sy)
    denom = _cross(rx, ry, sxv, syv)
    if abs(denom) < 1e-12:
        return (ex, ey)
    t = _cross(sx - ax_, sy - ay_, sxv, syv) / denom
    ix = ax_ + t * rx
    iy = ay_ + t * ry
    return (ix, iy)

def polygon_intersection(poly_a: list[tuple[float, float]], poly_b: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not poly_a or not poly_b or len(poly_a) < 3 or len(poly_b) < 3:
        return []
    cx = sum(p[0] for p in poly_b) / float(len(poly_b))
    cy = sum(p[1] for p in poly_b) / float(len(poly_b))
    output: list[tuple[float, float]] = list(poly_a)
    for i in range(len(poly_b)):
        a = poly_b[i]
        b = poly_b[(i + 1) % len(poly_b)]
        ax_, ay_ = a
        bx_, by_ = b
        s_center = _cross(bx_ - ax_, by_ - ay_, cx - ax_, cy - ay_)
        inp = output
        output = []
        if not inp:
            break
        S = inp[-1]
        for E in inp:
            val_e = _cross(bx_ - ax_, by_ - ay_, E[0] - ax_, E[1] - ay_) * s_center
            val_s = _cross(bx_ - ax_, by_ - ay_, S[0] - ax_, S[1] - ay_) * s_center
            inside_e = val_e >= -1e-12
            inside_s = val_s >= -1e-12
            if inside_e:
                if inside_s:
                    output.append(E)
                else:
                    inter = _intersect_segment_with_line(S, E, a, b)
                    output.append(inter)
                    output.append(E)
            else:
                if inside_s:
                    inter = _intersect_segment_with_line(S, E, a, b)
                    output.append(inter)
            S = E
    if len(output) < 3:
        return []
    return output
