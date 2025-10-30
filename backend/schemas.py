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


class StatusResponse(BaseModel):
    serial_number: str
    status: str
    battery_level: int
    current_map: Optional[str]
    position: Position
    speed: float
    last_update: datetime


class TranslateRequest(BaseModel):
    dx: float
    dy: float
    # 可选：伴随平移提交运动状态，以便后端同步更新
    movement_state: Optional[str] = None

class RotateRequest(BaseModel):
    dtheta: float