from __future__ import annotations
from typing import Iterable, Any, List, Tuple, Optional, Dict
import time
from pathlib import Path
import json
import math

from .utils.helpers import (
    compute_agv_base_polygon,
    compute_outer_bounding_rect,
    rects_overlap,
)
from .sim_vehicle.state_manager import global_store


# 前向中心偏移量（米）：将车辆几何中心沿车头方向平移，用于安全范围计算与渲染
centerForwardOffsetM: float = 0.1

def check_collision(vehicles: Iterable[Any]) -> bool:
    """快速检测是否存在两车安全范围重叠（考虑托盘并按 1. 扩大包围盒）。"""
    rects: List[Tuple[float, float, float, float]] = []
    for v in vehicles:
        try:
            rect = computeSafetyRectForVehicle(v, safeFactor=1.05)
            if rect:
                rects.append(rect)
        except Exception:
            pass
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            if rects_overlap(rects[i], rects[j]):
                return True
    return False


# =========================
# 辅助函数（驼峰命名）
# =========================
def getShelfFootprintFromLoads(vehicle: Any) -> Optional[Tuple[float, float, Optional[float]]]:
    """优先从 state.loads 提取托盘/货架 footprint (length, width, height) 单位米。"""
    try:
        loads = list(getattr(getattr(vehicle, "state", None), "loads", []) or [])
        for ld in loads:
            try:
                # 兼容 dataclass 与 dict(camelCase/snake_case)
                if isinstance(ld, dict):
                    ltype = ld.get("load_type", ld.get("loadType", None))
                else:
                    ltype = getattr(ld, "load_type", None)
            except Exception:
                ltype = None
            if str(ltype or "").lower() == "shelf":
                try:
                    if isinstance(ld, dict):
                        dim = ld.get("load_dimensions", ld.get("loadDimensions", None))
                    else:
                        dim = getattr(ld, "load_dimensions", None)
                    if dim is not None:
                        if isinstance(dim, dict):
                            length = float(dim.get("length", 0.0) or 0.0)
                            width = float(dim.get("width", 0.0) or 0.0)
                            hraw = dim.get("height", None)
                            h = float(hraw) if (hraw is not None) else None
                        else:
                            length = float(getattr(dim, "length", 0.0) or 0.0)
                            width = float(getattr(dim, "width", 0.0) or 0.0)
                            height = getattr(dim, "height", None)
                            h = float(height) if (height is not None) else None
                        if length > 0.0 or width > 0.0 or (h or 0.0) > 0.0:
                            return (length, width, h)
                except Exception:
                    pass
    except Exception:
        pass
    return None


def getShelfFootprintFromInfo(vehicle: Any) -> Optional[Tuple[float, float, Optional[float]]]:
    """若 loads 未给出尺寸，则根据 PalletAction 的 recfile 解析 SimVehicleSys/shelf 下的模型。"""
    try:
        infos = list(getattr(getattr(vehicle, "state", None), "information", []) or [])
        recfile = None
        for info in infos:
            try:
                # 兼容 dataclass 与 dict(camelCase/snake_case)
                if isinstance(info, dict):
                    itype = str(info.get("info_type", info.get("infoType", "")) or "")
                    if itype == "PalletAction":
                        refs = list(info.get("info_references", info.get("infoReferences", [])) or [])
                    else:
                        refs = []
                else:
                    itype = str(getattr(info, "info_type", "") or "")
                    if itype == "PalletAction":
                        refs = list(getattr(info, "info_references", []) or [])
                    else:
                        refs = []
                    for r in refs:
                        if isinstance(r, dict):
                            key = str(r.get("reference_key", r.get("referenceKey", "")) or "").lower()
                            if key == "recfile":
                                recfile = str(r.get("reference_value", r.get("referenceValue", "")) or "")
                        else:
                            if str(getattr(r, "reference_key", "") or "").lower() == "recfile":
                                recfile = str(getattr(r, "reference_value", "") or "")
                            break
            except Exception:
                pass
            if recfile:
                break
        if not recfile:
            return None
        sim_root = Path(__file__).resolve().parents[1]
        # 允许 "shelf/BD1.shelf" 或仅文件名
        candidate = sim_root / recfile
        if not candidate.exists():
            candidate = sim_root / "shelf" / Path(recfile).name
        if not candidate.exists():
            return None
        data = json.loads(candidate.read_text(encoding="utf-8"))
        fp = dict(data.get("footprint", {}))
        w = float(fp.get("width_m", 0.0) or 0.0)
        l = float(fp.get("length_m", 0.0) or 0.0)
        h_raw = fp.get("height_m", None)
        h = float(h_raw) if (h_raw is not None) else None
        if l > 0.0 or w > 0.0 or (h or 0.0) > 0.0:
            return (l, w, h)
    except Exception:
        pass
    return None


def getShelfFootprint(vehicle: Any) -> Optional[Tuple[float, float, Optional[float]]]:
    """综合方法：先 loads，再 Information.recfile 解析。返回 (length, width, height)。"""
    return getShelfFootprintFromLoads(vehicle) or getShelfFootprintFromInfo(vehicle)


def scaleRect(rect: Tuple[float, float, float, float], factor: float) -> Tuple[float, float, float, float]:
    """按比例缩放矩形尺寸（以中心为基准扩大/缩小）。rect=(xmin,ymin,xmax,ymax)。"""
    xmin, ymin, xmax, ymax = rect
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    w = max(0.0, xmax - xmin)
    h = max(0.0, ymax - ymin)
    nf = float(factor)
    nw = w * nf
    nh = h * nf
    dx = (nw - w) / 2.0
    dy = (nh - h) / 2.0
    return (xmin - dx, ymin - dy, xmax + dx, ymax + dy)


def computeSafetyRectForVehicle(vehicle: Any, safeFactor: float = 1.05) -> Optional[Tuple[float, float, float, float]]:
    """计算车辆的安全包围盒（考虑托盘尺寸），并按 safeFactor 扩大。

    尺寸来源：优先使用运行时配置 config.settings 的 length/width；否则采用默认值。
    不读取项目根的 factsheet.json。
    """
    try:
        st = getattr(vehicle, "state", None)
        pos = getattr(st, "agv_position", None)
        if not pos:
            return None
        # AGV 基体尺寸：优先运行时设置；否则使用默认值
        base_length = 1.03
        base_width = 0.745
        try:
            s_cfg = getattr(getattr(vehicle, "config", None), "settings", None)
            if s_cfg is not None:
                base_length = float(getattr(s_cfg, "length", base_length))
                base_width = float(getattr(s_cfg, "width", base_width))
        except Exception:
            pass
        theta = float(getattr(pos, "theta", 0.0) or 0.0)
        px = float(getattr(pos, "x", 0.0) or 0.0)
        py = float(getattr(pos, "y", 0.0) or 0.0)
        # 将几何中心沿车头方向（以 0 弧度为 Y−）平移 centerForwardOffsetM 米
        px_f = px + centerForwardOffsetM * math.sin(theta)
        py_f = py - centerForwardOffsetM * math.cos(theta)
        base_poly = compute_agv_base_polygon(base_length, base_width, (px_f, py_f, theta))
        base_rect = compute_outer_bounding_rect(base_poly)

        # 托盘/货架尺寸，若存在则与车辆包围盒合并
        shelf_fp = getShelfFootprint(vehicle)
        if shelf_fp is not None:
            shelf_len, shelf_wid, _ = shelf_fp
            if (shelf_len or 0.0) > 0.0 or (shelf_wid or 0.0) > 0.0:
                shelf_poly = compute_agv_base_polygon(float(shelf_len or 0.0), float(shelf_wid or 0.0), (px_f, py_f, theta))
                shelf_rect = compute_outer_bounding_rect(shelf_poly)
                # 合并矩形（取并集的外包矩形）
                xmin = min(base_rect[0], shelf_rect[0])
                ymin = min(base_rect[1], shelf_rect[1])
                xmax = max(base_rect[2], shelf_rect[2])
                ymax = max(base_rect[3], shelf_rect[3])
                base_rect = (xmin, ymin, xmax, ymax)

        # 安全范围：按 1.05 倍扩大（如需额外常量安全距，可叠加 expand）
        safe_rect = scaleRect(base_rect, max(1.0, float(safeFactor)))
        return safe_rect
    except Exception:
        return None


def _getShelfFootprintFromStatePayload(state_payload: Dict[str, Any]) -> Optional[Tuple[float, float, Optional[float]]]:
    """从 VDA5050 state JSON(payload) 中解析 shelf footprint (length,width,height)。"""
    try:
        loads = list(state_payload.get("loads", []) or [])
        for ld in loads:
            try:
                ltype = None
                if isinstance(ld, dict):
                    ltype = ld.get("load_type", ld.get("loadType", None))
                if str(ltype or "").lower() != "shelf":
                    continue
                dim = None
                if isinstance(ld, dict):
                    dim = ld.get("load_dimensions", ld.get("loadDimensions", None))
                if isinstance(dim, dict):
                    length = float(dim.get("length", 0.0) or 0.0)
                    width = float(dim.get("width", 0.0) or 0.0)
                    hraw = dim.get("height", None)
                    h = float(hraw) if (hraw is not None) else None
                    if length > 0.0 or width > 0.0 or (h or 0.0) > 0.0:
                        return (length, width, h)
            except Exception:
                pass
        # fallback: Information -> recfile
        infos = list(state_payload.get("information", []) or [])
        recfile = None
        for info in infos:
            try:
                itype = str(info.get("info_type", info.get("infoType", "")) or "")
                if itype != "PalletAction":
                    continue
                refs = list(info.get("info_references", info.get("infoReferences", [])) or [])
                for r in refs:
                    key = str(r.get("reference_key", r.get("referenceKey", "")) or "").lower()
                    if key == "recfile":
                        recfile = str(r.get("reference_value", r.get("referenceValue", "")) or "")
                        break
                if recfile:
                    break
            except Exception:
                pass
        if not recfile:
            return None
        sim_root = Path(__file__).resolve().parents[1]
        candidate = sim_root / recfile
        if not candidate.exists():
            candidate = sim_root / "shelf" / Path(recfile).name
        if not candidate.exists():
            return None
        data = json.loads(candidate.read_text(encoding="utf-8"))
        fp = dict(data.get("footprint", {}))
        w = float(fp.get("width_m", 0.0) or 0.0)
        l = float(fp.get("length_m", 0.0) or 0.0)
        h_raw = fp.get("height_m", None)
        h = float(h_raw) if (h_raw is not None) else None
        if l > 0.0 or w > 0.0 or (h or 0.0) > 0.0:
            return (l, w, h)
    except Exception:
        pass
    return None


def computeSafetyRectForStatePayload(state_payload: Dict[str, Any], length_default: float = 1.03, width_default: float = 0.745, safeFactor: float = 1.05) -> Optional[Tuple[float, float, float, float]]:
    """基于 VDA5050 state JSON(payload) 计算安全包围盒，支持 1.05 放大与托盘尺寸融合。
    """
    try:
        agv = state_payload.get("agv_position", state_payload.get("agvPosition", {})) or {}
        px = float(agv.get("x", 0.0) or 0.0)
        py = float(agv.get("y", 0.0) or 0.0)
        raw_theta = agv.get("theta", None)
        if raw_theta is None:
            raw_theta = agv.get("orientation", 0.0)
        theta = float(raw_theta or 0.0)
        # 将几何中心沿车头方向（以 0 弧度为 Y−）平移 centerForwardOffsetM 米
        px_f = px + centerForwardOffsetM * math.sin(theta)
        py_f = py - centerForwardOffsetM * math.cos(theta)
        base_poly = compute_agv_base_polygon(float(length_default or 1.03), float(width_default or 0.745), (px_f, py_f, theta))
        base_rect = compute_outer_bounding_rect(base_poly)
        shelf_fp = _getShelfFootprintFromStatePayload(state_payload)
        if shelf_fp is not None:
            shelf_len, shelf_wid, _ = shelf_fp
            if (shelf_len or 0.0) > 0.0 or (shelf_wid or 0.0) > 0.0:
                shelf_poly = compute_agv_base_polygon(float(shelf_len or 0.0), float(shelf_wid or 0.0), (px_f, py_f, theta))
                shelf_rect = compute_outer_bounding_rect(shelf_poly)
                xmin = min(base_rect[0], shelf_rect[0])
                ymin = min(base_rect[1], shelf_rect[1])
                xmax = max(base_rect[2], shelf_rect[2])
                ymax = max(base_rect[3], shelf_rect[3])
                base_rect = (xmin, ymin, xmax, ymax)
        safe_rect = scaleRect(base_rect, max(1.0, float(safeFactor)))
        return safe_rect
    except Exception:
        return None


class CollisionService:
    """
    碰撞检测服务骨架：周期性检测车辆外包络矩形重叠，并触发紧急停止与错误上报。
    - tick_ms：检测周期（默认100ms）。
    - safety_margin：安全距离（按矩形扩张）。
    """

    def __init__(self, vehicles: Iterable[Any], tick_ms: int = 100, safety_margin: float = 0.1) -> None:
        self.vehicles = list(vehicles)
        self.tick_ms = int(tick_ms)
        # safety_margin 保留为常量扩张（可选）；主要安全范围由 1.05 倍缩放确定
        self.safety_margin = float(safety_margin)
        self._stop = False
        # 记录因碰撞而触发的暂停序列号集合，用于恢复判断
        self._collision_paused_serials: set[str] = set()
        # 物理尺寸接口（占位，后续可用于三维/层高碰撞逻辑）
        self.height_min: float = 0.01
        self.height_max: float = 0.10
        self.width_default: float = 0.745
        self.length_default: float = 1.03
        try:
            if len(self.vehicles) > 0:
                s_cfg = getattr(getattr(self.vehicles[0], "config", None), "settings", None)
                if s_cfg is not None:
                    self.height_min = float(getattr(s_cfg, "height_min", self.height_min))
                    self.height_max = float(getattr(s_cfg, "height_max", self.height_max))
                    self.width_default = float(getattr(s_cfg, "width", self.width_default))
                    self.length_default = float(getattr(s_cfg, "length", self.length_default))
        except Exception:
            pass

    def start(self) -> None:
        # 骨架：同步阻塞循环；后续可改为线程
        self._stop = False
        while not self._stop:
            try:
                self.detect_and_emit()
            except Exception:
                pass
            time.sleep(max(0.01, self.tick_ms / 1000.0))

    def stop(self) -> None:
        self._stop = True

    def detect_and_emit(self) -> None:
        rects: List[Tuple[float, float, float, float]] = []
        owners: List[Any] = []
        for v in self.vehicles:
            try:
                safe_rect = computeSafetyRectForVehicle(v, safeFactor=1.05)
                if safe_rect is None:
                    continue
                # 如配置了常量安全边距，则叠加扩展（保证更保守）
                if self.safety_margin > 0.0:
                    xmin, ymin, xmax, ymax = safe_rect
                    # 以常量方式扩张
                    safe_rect = (xmin - self.safety_margin, ymin - self.safety_margin, xmax + self.safety_margin, ymax + self.safety_margin)
                rects.append(safe_rect)
                owners.append(v)
            except Exception:
                pass
        overlapped_serials: set[str] = set()
        for i in range(len(rects)):
            for j in range(i + 1, len(rects)):
                if rects_overlap(rects[i], rects[j]):
                    self.issue_emergency_stop(owners[i])
                    self.issue_emergency_stop(owners[j])
                    self.push_collision_error(owners[i], owners[j])
                    # 记录发生重叠的车辆序列号
                    try:
                        s1 = str(getattr(getattr(getattr(owners[i], "config", None), "vehicle", None), "serial_number", "") or "")
                        s2 = str(getattr(getattr(getattr(owners[j], "config", None), "vehicle", None), "serial_number", "") or "")
                        if s1:
                            overlapped_serials.add(s1)
                        if s2:
                            overlapped_serials.add(s2)
                    except Exception:
                        pass
        # 更新碰撞暂停集合
        try:
            self._collision_paused_serials |= overlapped_serials
        except Exception:
            pass
        # 对于此前因碰撞暂停但当前未重叠的车辆，执行恢复
        try:
            to_resume = {s for s in self._collision_paused_serials if s not in overlapped_serials}
            if to_resume:
                for v in owners:
                    try:
                        serial = str(getattr(getattr(getattr(v, "config", None), "vehicle", None), "serial_number", "") or "")
                        if serial and serial in to_resume:
                            self.issue_resume(v)
                    except Exception:
                        pass
                # 从集合中移除已恢复的序列号
                self._collision_paused_serials -= to_resume
        except Exception:
            pass

    def issue_emergency_stop(self, vehicle: Any) -> None:
        try:
            st = getattr(vehicle, "state", None)
            if st:
                st.driving = False
            setattr(vehicle, "nav_paused", True)
        except Exception:
            pass

    def issue_resume(self, vehicle: Any) -> None:
        """碰撞恢复：取消暂停，按导航状态恢复 driving。"""
        try:
            # 仅恢复暂停标记，不改变订单与导航结构
            try:
                setattr(vehicle, "nav_paused", False)
                st = getattr(vehicle, "state", None)
                if st:
                    st.paused = False
            except Exception:
                pass
            # 若导航仍在运行，则恢复为 driving；否则保持为非 driving（但此时可接单）
            try:
                if getattr(vehicle, "nav_running", False):
                    st = getattr(vehicle, "state", None)
                    if st:
                        st.driving = True
                else:
                    st = getattr(vehicle, "state", None)
                    if st:
                        st.driving = False
            except Exception:
                pass
        except Exception:
            pass
    def push_collision_error(self, v1: Any, v2: Any) -> None:
        try:
            serial = getattr(getattr(v1, "config", None), "vehicle", None).serial_number
            rt = global_store.get_runtime(serial)
            # 错误码参考：docs/SimAGV错误代码.md 行 27（54231, Warning, Caution: robot is blocked）
            payload = {
                "code": 54231,
                "level": "Warning",
                "type": "Navigation",
                "reason": "CollisionOverlapSafety",
                "message": "Caution: robot is blocked",
                "with": getattr(getattr(v2, "config", None), "vehicle", None).serial_number,
                "descriptionCN": "注意机器人被阻挡",
            }
            global_store.set_error(serial, payload)
        except Exception:
            pass