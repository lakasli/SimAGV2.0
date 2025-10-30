from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List

HeaderId = int


@dataclass
class AgvPosition:
    x: float
    y: float
    theta: float
    map_id: str
    map_description: Optional[str] = None
    position_initialized: bool = False
    localization_score: Optional[float] = None
    deviation_range: Optional[float] = None


@dataclass
class BoundingBoxReference:
    x: float
    y: float
    z: float
    theta: Optional[float] = None


@dataclass
class LoadDimensions:
    length: float
    width: float
    height: Optional[float] = None


@dataclass
class ControlPoint:
    x: float
    y: float
    weight: Optional[float] = None
    orientation: Optional[float] = None


@dataclass
class NodePosition:
    x: float
    y: float
    theta: Optional[float]
    allowed_deviation_xy: Optional[float]
    allowed_deviation_theta: Optional[float]
    map_id: str
    map_description: Optional[str] = None


@dataclass
class Trajectory:
    degree: int
    knot_vector: List[float]
    control_points: List[ControlPoint]


@dataclass
class Velocity:
    vx: Optional[float] = None
    vy: Optional[float] = None
    omega: Optional[float] = None