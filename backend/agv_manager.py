from __future__ import annotations
import json
import threading
from pathlib import Path
from typing import Dict, List

from .schemas import AGVRegistration, AGVInfo, AGVRuntime, StaticConfigPatch, DynamicConfigUpdate, StatusResponse
from SimVehicleSys.sim_vehicle.state_manager import global_store


class AGVManager:
    """
    后端的轻量级注册管理器：仅负责 AGV 实例的注册/删除/静态配置。
    运行时状态一律委托给 SimAGV.global_store 管理，确保导航与运动逻辑可直接读写统一状态源。
    """

    def __init__(self, storage_path: Path):
        self.storage_path = storage_path
        self._lock = threading.Lock()
        self._agvs: Dict[str, AGVInfo] = {}
        self._ensure_storage_dir()
        self._load()

    def _ensure_storage_dir(self) -> None:
        self.storage_path.parent.mkdir(parents=True, exist_ok=True)

    def _load(self) -> None:
        if self.storage_path.exists():
            try:
                data = json.loads(self.storage_path.read_text(encoding="utf-8"))
                for item in data:
                    info = AGVInfo(**item)
                    self._agvs[info.serial_number] = info
            except Exception:
                # if corrupted, start fresh
                self._agvs = {}

    def _persist(self) -> None:
        items = [info.model_dump() for info in self._agvs.values()]
        self.storage_path.write_text(json.dumps(items, ensure_ascii=False, indent=2), encoding="utf-8")

    def list_agvs(self) -> List[AGVInfo]:
        with self._lock:
            return list(self._agvs.values())

    def register_many(self, regs: List[AGVRegistration]) -> tuple[List[str], List[str]]:
        registered: List[str] = []
        skipped: List[str] = []
        with self._lock:
            for r in regs:
                if r.serial_number in self._agvs:
                    skipped.append(r.serial_number)
                    continue
                info = AGVInfo(
                    serial_number=r.serial_number,
                    manufacturer=r.manufacturer,
                    type=r.type,
                    vda_version=r.vda_version,
                    IP=r.IP,
                )
                self._agvs[r.serial_number] = info
                registered.append(r.serial_number)
            self._persist()
        return registered, skipped

    def unregister(self, serial_number: str) -> bool:
        with self._lock:
            removed = self._agvs.pop(serial_number, None) is not None
            if removed:
                self._persist()
            return removed

    def get_agv(self, serial_number: str) -> AGVInfo | None:
        with self._lock:
            return self._agvs.get(serial_number)

    def update_static(self, serial_number: str, patch: StaticConfigPatch) -> AGVInfo | None:
        with self._lock:
            info = self._agvs.get(serial_number)
            if not info:
                return None
            if patch.IP is not None:
                info.IP = patch.IP
            # vda_version and others are ignored for static patch
            self._persist()
            return info

    def update_dynamic(self, serial_number: str, dyn: DynamicConfigUpdate) -> AGVRuntime | None:
        # 委托给集中式状态存储
        with self._lock:
            if serial_number not in self._agvs:
                return None
        return global_store.update_dynamic(serial_number, dyn)

    def get_status(self, serial_number: str) -> StatusResponse | None:
        # 从集中式状态存储读取运行时数据，包装为响应
        with self._lock:
            info = self._agvs.get(serial_number)
        return global_store.get_status(serial_number, info)

    def move_translate(self, serial_number: str, dx: float, dy: float, movement_state: str | None = None) -> AGVRuntime | None:
        # 委托给集中式状态存储
        with self._lock:
            if serial_number not in self._agvs:
                return None
        return global_store.move_translate(serial_number, dx, dy, movement_state)

    def move_rotate(self, serial_number: str, dtheta: float) -> AGVRuntime | None:
        # 委托给集中式状态存储
        with self._lock:
            if serial_number not in self._agvs:
                return None
        return global_store.move_rotate(serial_number, dtheta)