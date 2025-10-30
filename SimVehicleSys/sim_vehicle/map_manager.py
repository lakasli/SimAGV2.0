from __future__ import annotations
from pathlib import Path
from typing import Optional

from .navigation import resolve_scene_path, parse_scene_topology


def load_scene_topology(map_name: Optional[str]) -> dict:
    """加载并解析 .scene 地图拓扑。"""
    fp = resolve_scene_path(map_name)
    return parse_scene_topology(fp)