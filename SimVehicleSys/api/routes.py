from __future__ import annotations
from typing import List, Optional
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from SimVehicleSys.sim_vehicle.navigation import (
    resolve_scene_path,
    parse_scene_topology,
    route_polyline,
    augment_with_corner_turns,
)

# 注意：避免在模块导入阶段从 backend.main 取对象，
# 会导致循环导入下 agv_manager 为 None。改为在路由函数内部按需导入。


router = APIRouter(prefix="/api/sim", tags=["SimVehicleSys"])


class PolylineRequest(BaseModel):
    route_node_ids: List[str]
    map_name: Optional[str] = None
    steps_per_edge: int = 20


@router.post("/nav/route_polyline")
def api_route_polyline(body: PolylineRequest):
    if not body.route_node_ids or len(body.route_node_ids) < 2:
        raise HTTPException(status_code=400, detail="route_node_ids must contain at least 2 nodes")
    try:
        fp = resolve_scene_path(body.map_name)
        topo = parse_scene_topology(fp)
        stations = topo["stations"]
        paths = topo["paths"]
        station_ids = {str(s["id"]) for s in stations}
        route: List[str] = []
        for nid in body.route_node_ids:
            sid = str(nid)
            if sid not in station_ids:
                raise HTTPException(status_code=404, detail=f"Node '{sid}' not found in map stations")
            route.append(sid)
        pts = route_polyline(route, stations, paths, steps_per_edge=int(body.steps_per_edge))
        pts = augment_with_corner_turns(pts, theta_threshold=0.1, step_delta=0.08)
        return {"points": pts, "route": route}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"route_polyline failed: {e}")


@router.post("/agv/{serial_number}/nav/station")
def api_nav_start_station(serial_number: str, station_id: str, map_name: Optional[str] = None):
    # 延迟导入，避免循环导入导致对象为 None
    try:
        from backend.main import agv_manager as _agv_manager, app as _backend_app  # type: ignore
    except Exception:
        _agv_manager = None  # type: ignore
        _backend_app = None  # type: ignore

    if not _agv_manager:
        raise HTTPException(status_code=500, detail="AGVManager unavailable")
    if serial_number not in [info.serial_number for info in _agv_manager.list_agvs()]:
        raise HTTPException(status_code=404, detail="AGV not found")
    status = _agv_manager.get_status(serial_number)
    if not status:
        raise HTTPException(status_code=404, detail="AGV not found or no status")
    # 目标地图选择：优先使用请求参数，其次使用当前地图
    target_map = map_name or status.current_map
    if not target_map:
        raise HTTPException(status_code=400, detail="No map available (both request and current_map empty)")

    try:
        publisher = getattr(_backend_app.state, "mqtt_publisher", None) if _backend_app else None
        info = _agv_manager.get_agv(serial_number)
        if not publisher or not info:
            raise RuntimeError("MQTT publisher or AGV info unavailable")
        publisher.publish_start_station_navigation(info, station_id, target_map)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Navigation command failed: {e}")
    return {"status": "started", "serial": serial_number, "station_id": station_id, "map": target_map}