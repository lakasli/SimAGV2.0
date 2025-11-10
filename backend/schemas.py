from __future__ import annotations
from typing import List, Optional
from pydantic import BaseModel, Field
from datetime import datetime


class AGVRegistration(BaseModel):
    serial_number: str = Field(..., description="唯一识别码")
    manufacturer: str = Field(..., description="制造商")
    type: str = Field(..., description="类型，如 'AGV' 或 'Fleet'")
    vda_version: str = Field(..., description="协议版本，如 'v2'")
    IP: str = Field(..., description="实例 IP 地址")


class RegisterRequest(BaseModel):
    agvs: List[AGVRegistration]


class RegisterResponse(BaseModel):
    registered: List[str]
    skipped: List[str]


class UnregisterResponse(BaseModel):
    removed: bool


class AGVInfo(BaseModel):
    serial_number: str
    manufacturer: str
    type: str
    vda_version: str
    IP: str


class Position(BaseModel):
    x: float
    y: float
    theta: Optional[float] = None


class StaticConfigPatch(BaseModel):
    IP: Optional[str] = None
    vda_version: Optional[str] = None


class DynamicConfigUpdate(BaseModel):
    battery_level: Optional[int] = None
    current_map: Optional[str] = None
    position: Optional[Position] = None
    speed_limit: Optional[float] = None
    # 运动状态：'forward' 或 'backward'
    movement_state: Optional[str] = None


class AGVRuntime(BaseModel):
    battery_level: int = 100
    current_map: Optional[str] = None
    position: Position = Position(x=0.0, y=0.0, theta=0.0)
    speed_limit: float = 1.0
    speed: float = 0.0
    last_update: datetime = Field(default_factory=datetime.utcnow)
    movement_state: str = "forward"
    # 错误列表（可能同时存在多个），来自集中式错误管理的标准化载荷
    errors: List[dict] = Field(default_factory=list)


class StatusResponse(BaseModel):
    serial_number: str
    status: str
    battery_level: int
    current_map: Optional[str]
    position: Position
    speed: float
    last_update: datetime
    # 错误列表，便于前端/监控展示；无错误时为空数组
    errors: List[dict] = Field(default_factory=list)


class TranslateRequest(BaseModel):
    dx: float
    dy: float
    # 可选：伴随平移提交运动状态，以便后端同步更新
    movement_state: Optional[str] = None

class RotateRequest(BaseModel):
    dtheta: float


class SimSettingsPatch(BaseModel):
    # 对应 SimVehicleSys.config.settings.Settings 中的字段，均为可选以便局部更新
    action_time: Optional[float] = None
    speed: Optional[float] = None
    # 物理参数扩展（factsheet.physicalParameters）：用于约束运动与碰撞
    speed_min: Optional[float] = None
    speed_max: Optional[float] = None
    acceleration_max: Optional[float] = None
    deceleration_max: Optional[float] = None
    height_min: Optional[float] = None
    height_max: Optional[float] = None
    width: Optional[float] = None
    length: Optional[float] = None
    state_frequency: Optional[int] = None
    visualization_frequency: Optional[int] = None
    map_id: Optional[str] = None
    sim_time_scale: Optional[float] = None
    battery_default: Optional[float] = None
    battery_idle_drain_per_min: Optional[float] = None
    battery_move_empty_multiplier: Optional[float] = None
    battery_move_loaded_multiplier: Optional[float] = None
    battery_charge_per_min: Optional[float] = None
    frontend_poll_interval_ms: Optional[int] = None


class SwitchMapRequest(BaseModel):
    # 地图标识，例如 "duoc" 或 "VehicleMap/duoc.scene"
    map: str
    # 站点名称或 ID（可选）：若提供，则优先按站点定位
    switch_point: Optional[str] = None
    # 切换后中心坐标（米），在未提供站点时作为兜底
    center_x: Optional[float] = None
    center_y: Optional[float] = None
    # 切换后朝向（弧度），可选
    initiate_angle: Optional[float] = None