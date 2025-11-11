from __future__ import annotations
import json
from pathlib import Path
from typing import Any, Dict, Optional

from SimVehicleSys.config.settings import get_config
from .schemas import AGVRuntime


class AgvConfigStore:
    """
    轻量级的仿真车实例配置存储：每个序列号一个 JSON 文件，记录物理参数与最近离线快照。

    存储字段（均为可选）：
    - speed_min, speed_max, acceleration_max, deceleration_max, height_min, height_max, width, length
    - sim_time_scale, state_frequency, visualization_frequency, action_time
    - map_id（最近加载的地图标识）
    - last_position: { x, y, theta }
    """

    def __init__(self, base_dir: Path) -> None:
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)

    def _path_for(self, serial: str) -> Path:
        safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in str(serial))
        return self.base_dir / f"{safe}.json"

    def load(self, serial: str) -> Optional[Dict[str, Any]]:
        fp = self._path_for(serial)
        if not fp.exists():
            return None
        try:
            return json.loads(fp.read_text(encoding="utf-8"))
        except Exception:
            return None

    def save(self, serial: str, data: Dict[str, Any]) -> None:
        fp = self._path_for(serial)
        try:
            fp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            # 失败时忽略；不影响主流程
            pass

    def ensure_default_for(self, serial: str) -> None:
        """若配置文件不存在，则创建默认配置（来源于全局 settings/factsheet）。"""
        if self.load(serial) is not None:
            return
        try:
            cfg = get_config()
            s = cfg.settings
            data = {
                "speed_min": float(getattr(s, "speed_min", 0.01)),
                "speed_max": float(getattr(s, "speed_max", 2.0)),
                "acceleration_max": float(getattr(s, "acceleration_max", 2.0)),
                "deceleration_max": float(getattr(s, "deceleration_max", 2.0)),
                "height_min": float(getattr(s, "height_min", 0.01)),
                "height_max": float(getattr(s, "height_max", 0.10)),
                "width": float(getattr(s, "width", 0.745)),
                "length": float(getattr(s, "length", 1.03)),
                "state_frequency": int(getattr(s, "state_frequency", 10)),
                "visualization_frequency": int(getattr(s, "visualization_frequency", 1)),
                "sim_time_scale": float(getattr(s, "sim_time_scale", 1.0)),
                "action_time": float(getattr(s, "action_time", 1.0)),
                "map_id": str(getattr(s, "map_id", "default")),
                "last_position": None,
            }
            self.save(serial, data)
        except Exception:
            # 若读取默认配置失败，仍创建一个最小结构的文件
            self.save(serial, {"map_id": "default", "last_position": None})

    def update_physical(self, serial: str, patch: Dict[str, Any]) -> None:
        """根据提交的仿真设置部分更新物理参数/运行参数。忽略 None 值。"""
        cur = self.load(serial) or {}
        keys = [
            "speed_min",
            "speed_max",
            "acceleration_max",
            "deceleration_max",
            "height_min",
            "height_max",
            "width",
            "length",
            "sim_time_scale",
            "state_frequency",
            "visualization_frequency",
            "action_time",
            "map_id",
        ]
        for k in keys:
            v = patch.get(k)
            if v is not None:
                cur[k] = v
        self.save(serial, cur)

    def update_runtime_snapshot(self, serial: str, rt: AGVRuntime) -> None:
        """保存运行态快照：当前地图与位置/朝向。"""
        cur = self.load(serial) or {}
        try:
            cur["map_id"] = str(getattr(rt, "current_map", cur.get("map_id")))
        except Exception:
            pass
        try:
            pos = getattr(rt, "position", None)
            if pos is not None:
                cur["last_position"] = {
                    "x": float(getattr(pos, "x", 0.0)),
                    "y": float(getattr(pos, "y", 0.0)),
                    "theta": float(getattr(pos, "theta", 0.0)),
                }
        except Exception:
            pass
        self.save(serial, cur)

    def delete(self, serial: str) -> None:
        """删除指定序列号的配置文件。"""
        fp = self._path_for(serial)
        try:
            if fp.exists():
                fp.unlink(missing_ok=True)
        except Exception:
            pass