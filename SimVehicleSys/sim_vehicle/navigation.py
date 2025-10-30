from __future__ import annotations
from pathlib import Path
from typing import Optional, List, Dict, Tuple
import json
import math


project_root = Path(__file__).resolve().parents[2]
maps_dir = project_root / "maps"


def _read_json(fp: Path) -> dict:
    return json.loads(fp.read_text(encoding="utf-8"))


def _bezier_point(P0, P1, P2, P3, t: float) -> Tuple[float, float]:
    x = (
        (1 - t) ** 3 * P0[0]
        + 3 * (1 - t) ** 2 * t * P1[0]
        + 3 * (1 - t) * t ** 2 * P2[0]
        + t ** 3 * P3[0]
    )
    y = (
        (1 - t) ** 3 * P0[1]
        + 3 * (1 - t) ** 2 * t * P1[1]
        + 3 * (1 - t) * t ** 2 * P2[1]
        + t ** 3 * P3[1]
    )
    return (x, y)


def _bezier_length(P0, P1, P2, P3, steps: int = 32) -> float:
    length = 0.0
    prev = P0
    for i in range(1, steps + 1):
        t = i / steps
        cur = _bezier_point(P0, P1, P2, P3, t)
        dx = cur[0] - prev[0]
        dy = cur[1] - prev[1]
        length += math.hypot(dx, dy)
        prev = cur
    return length


def _extract_scene_stations(data: dict | list) -> list[dict]:
    root = data[0] if isinstance(data, list) and data else (data if isinstance(data, dict) else {})
    points = root.get("points") or []
    out: list[dict] = []
    allowed_prefixes = {"AP", "CP", "PP", "LM", "WP"}

    def _classify(name: str) -> str:
        name = name.upper()
        if name.startswith("AP"): return "ActionPoint"
        if name.startswith("LM"): return "LocationMark"
        if name.startswith("CP"): return "ChargingPoint"
        if name.startswith("PP"): return "ParkingPoint"
        if name.startswith("WP"): return "WayPoint"
        return "LocationMark"

    for p in points:
        nm = str(p.get("name") or "").strip()
        if not nm:
            continue
        if not any(nm.upper().startswith(pref) for pref in allowed_prefixes):
            continue
        x = float(p.get("x", 0.0))
        y = float(p.get("y", 0.0))
        out.append({
            "id": str(p.get("id")) if p.get("id") is not None else nm,
            "instanceName": nm,
            "pointName": nm,
            "pos": {"x": x, "y": y},
        })
    return out


def parse_scene_topology(fp: Path) -> dict:
    data = _read_json(fp)
    root = data[0] if isinstance(data, list) and data else (data if isinstance(data, dict) else {})
    points = root.get("points") or []
    routes = root.get("routes") or []

    anchors_map: Dict[str, dict] = {}
    for p in points:
        pid = str(p.get("id")) if p.get("id") is not None else None
        if not pid:
            continue
        x = float(p.get("x", 0.0))
        y = float(p.get("y", 0.0))
        anchors_map[pid] = {"id": pid, "x": x, "y": y}

    paths: List[dict] = []
    for r in routes:
        rid = str(r.get("id")) if r.get("id") is not None else None
        a = str(r.get("from")) if r.get("from") is not None else None
        b = str(r.get("to")) if r.get("to") is not None else None
        if not a or not b or a not in anchors_map or b not in anchors_map:
            continue
        P0 = (anchors_map[a]["x"], anchors_map[a]["y"])
        P3 = (anchors_map[b]["x"], anchors_map[b]["y"])
        cp1 = r.get("c1") or r.get("cp1")
        cp2 = r.get("c2") or r.get("cp2")
        P1 = (float(cp1["x"]), float(cp1["y"])) if isinstance(cp1, dict) else P0
        P2 = (float(cp2["x"]), float(cp2["y"])) if isinstance(cp2, dict) else P3
        length = _bezier_length(P0, P1, P2, P3, steps=32)
        paths.append({
            "id": rid or f"{a}->{b}",
            "from": a,
            "to": b,
            "length": length,
            "cp1": {"x": P1[0], "y": P1[1]} if isinstance(cp1, dict) else None,
            "cp2": {"x": P2[0], "y": P2[1]} if isinstance(cp2, dict) else None,
        })

    stations = _extract_scene_stations(root)
    return {"stations": stations, "anchors": list(anchors_map.values()), "paths": paths}


def _build_graph(paths: list[dict]) -> Tuple[Dict[str, List[Tuple[str, float]]], Dict[Tuple[str, str], dict]]:
    adj: Dict[str, List[Tuple[str, float]]] = {}
    path_map: Dict[Tuple[str, str], dict] = {}
    for p in paths:
        a, b, w = p["from"], p["to"], p["length"]
        adj.setdefault(a, []).append((b, w))
        path_map[(a, b)] = p
    return adj, path_map


def _heuristic(a: str, b: str, anchors_map: Dict[str, dict]) -> float:
    pa = anchors_map.get(a)
    pb = anchors_map.get(b)
    if not pa or not pb:
        return 0.0
    dx = pa["x"] - pb["x"]
    dy = pa["y"] - pb["y"]
    return (dx * dx + dy * dy) ** 0.5


def a_star(start_id: str, end_id: str, anchors: List[dict], paths: List[dict]) -> List[str]:
    anchors_map = {a["id"]: a for a in anchors}
    adj, _ = _build_graph(paths)
    import heapq
    open_set: List[Tuple[float, str]] = []
    heapq.heappush(open_set, (0.0, start_id))
    came_from: Dict[str, Optional[str]] = {start_id: None}
    gscore: Dict[str, float] = {start_id: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == end_id:
            path_ids: List[str] = []
            cur = current
            while cur is not None:
                path_ids.append(cur)
                cur = came_from[cur]
            path_ids.reverse()
            return path_ids
        for (nbr, w) in adj.get(current, []):
            tentative = gscore[current] + w
            if tentative < gscore.get(nbr, float("inf")):
                came_from[nbr] = current
                gscore[nbr] = tentative
                fscore = tentative + _heuristic(nbr, end_id, anchors_map)
                heapq.heappush(open_set, (fscore, nbr))
    return []


def route_polyline(route: List[str], anchors: List[dict], paths: List[dict], steps_per_edge: int = 20) -> List[dict]:
    anchors_map = {a["id"]: a for a in anchors}
    edge_lookup: Dict[Tuple[str, str], dict] = {}
    for p in paths:
        edge_lookup[(p["from"], p["to"])] = p
    pts: List[dict] = []
    prev_x: Optional[float] = None
    prev_y: Optional[float] = None
    last_angle: float = 0.0
    for i in range(len(route) - 1):
        a = route[i]
        b = route[i + 1]
        edge = edge_lookup.get((a, b))
        if not edge:
            raise ValueError(f"No forward edge for {a}->{b} in polyline generation")
        else:
            P0 = (anchors_map[a]["x"], anchors_map[a]["y"]) 
            P3 = (anchors_map[b]["x"], anchors_map[b]["y"]) 
            cp1 = edge.get("cp1")
            cp2 = edge.get("cp2")
            P1 = (cp1["x"], cp1["y"]) if cp1 else P0
            P2 = (cp2["x"], cp2["y"]) if cp2 else P3
        for j in range(steps_per_edge + 1):
            t = j / steps_per_edge
            x, y = _bezier_point(P0, P1, P2, P3, t)
            angle = last_angle
            if prev_x is not None and prev_y is not None:
                dx = x - prev_x
                dy = y - prev_y
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    angle = math.atan2(dx, dy)
                    last_angle = angle
            pts.append({"x": x, "y": y, "theta": angle})
            prev_x, prev_y = x, y
    if len(pts) >= 2:
        pts[0]["theta"] = pts[1]["theta"]
    return pts


def _normalize_angle(a: float) -> float:
    pi = math.pi
    return (a + pi) % (2 * pi) - pi


def augment_with_corner_turns(
    pts: List[dict],
    theta_threshold: float = 0.1,
    step_delta: float = 0.08,
    pos_eps: float = 1e-9,
) -> List[dict]:
    if not pts:
        return pts
    new_pts: List[dict] = []
    new_pts.append({"x": float(pts[0]["x"]), "y": float(pts[0]["y"]), "theta": float(pts[0]["theta"])})
    for i in range(1, len(pts)):
        prev = new_pts[-1]
        curr_raw = pts[i]
        curr = {"x": float(curr_raw["x"]), "y": float(curr_raw["y"]), "theta": float(curr_raw["theta"]) }
        dx = curr["x"] - prev["x"]
        dy = curr["y"] - prev["y"]
        dpos2 = dx * dx + dy * dy
        dtheta = _normalize_angle(curr["theta"] - prev["theta"])
        if abs(dtheta) > theta_threshold and dpos2 > pos_eps:
            steps = max(1, int(abs(dtheta) / max(1e-6, step_delta)))
            for n in range(1, steps + 1):
                mid_theta = _normalize_angle(prev["theta"] + dtheta * (n / steps))
                new_pts.append({"x": prev["x"], "y": prev["y"], "theta": mid_theta})
        new_pts.append(curr)
    return new_pts


def resolve_scene_path(map_name: Optional[str]) -> Path:
    if not map_name:
        raise ValueError("map_name required")
    fname = str(map_name).split("/")[-1]
    if fname.endswith(".scene"):
        fp = maps_dir / "VehicleMap" / fname
        if not fp.exists():
            fp = maps_dir / "ViewerMap" / fname
    else:
        fp = maps_dir / "VehicleMap" / (fname + ".scene")
    if not fp.exists():
        raise FileNotFoundError(f"scene map '{fp.name}' not found")
    return fp


def find_station_position(fp: Path, station_id: str) -> Optional[Tuple[float, float]]:
    data = _read_json(fp)
    stations = _extract_scene_stations(data)
    for st in stations:
        ids = [st.get("id"), st.get("instanceName"), st.get("pointName")]
        if station_id and station_id in [str(i) for i in ids if i is not None]:
            px = st.get("pos") or {}
            x = float(px.get("x", 0.0))
            y = float(px.get("y", 0.0))
            return (x, y)
    return None

def find_point_name_by_id(fp: Path, node_id: str) -> Optional[str]:
    """根据 .scene 文件，在 points 列表中通过 id 查找并返回名称。

    如果找到匹配的点位且存在 name，则返回其 name；
    否则返回 None（调用方可回退为原始 id）。
    """
    try:
        data = _read_json(fp)
        root = data[0] if isinstance(data, list) and data else (data if isinstance(data, dict) else {})
        points = root.get("points") or []
        sid = str(node_id)
        for p in points:
            pid = p.get("id")
            if pid is None:
                continue
            if str(pid) == sid:
                nm = p.get("name")
                nm_str = str(nm) if nm is not None else ""
                return nm_str.strip() or None
        return None
    except Exception:
        return None


def nearest_anchor(pos: Tuple[float, float], anchors: List[dict]) -> Optional[str]:
    best_id = None
    best_d = float("inf")
    for a in anchors:
        dx = a["x"] - pos[0]
        dy = a["y"] - pos[1]
        d = math.hypot(dx, dy)
        if d < best_d:
            best_d = d
            best_id = a["id"]
    return best_id