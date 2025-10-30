from __future__ import annotations
from pathlib import Path
from typing import Optional

from .navigation import resolve_scene_path, parse_scene_topology
from .error_manager import emit_error


def load_scene_topology(map_name: Optional[str], serial_number: Optional[str] = None) -> dict:
    """加载并解析 .scene 地图拓扑，并在失败时发出集中式错误。"""
    try:
        fp = resolve_scene_path(map_name)
    except FileNotFoundError:
        try:
            emit_error(50101, {"serial_number": serial_number, "map": map_name})
        except Exception:
            pass
        raise
    try:
        topo = parse_scene_topology(fp)
    except Exception:
        try:
            emit_error(50102, {"serial_number": serial_number, "map": map_name})
        except Exception:
            pass
        raise
    return topo