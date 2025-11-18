from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from ..vda5050_common import (
    AgvPosition,
    BoundingBoxReference,
    HeaderId,
    LoadDimensions,
    NodePosition,
    Trajectory,
    Velocity,
)


class OperatingMode(str, Enum):
    Automatic = "AUTOMATIC"
    Semiautomatic = "SEMIAUTOMATIC"
    Manual = "MANUAL"
    Service = "SERVICE"
    Teachin = "TEACHIN"


class ActionStatus(str, Enum):
    Waiting = "WAITING"
    Initializing = "INITIALIZING"
    Paused = "PAUSED"
    Running = "RUNNING"
    Finished = "FINISHED"
    Failed = "FAILED"


@dataclass
class Load:
    load_id: Optional[str] = None
    load_type: Optional[str] = None
    load_position: Optional[str] = None
    bounding_box_reference: Optional[BoundingBoxReference] = None
    load_dimensions: Optional[LoadDimensions] = None
    weight: Optional[float] = None


@dataclass
class ActionState:
    action_id: str
    action_status: ActionStatus
    action_type: Optional[str] = None
    action_description: Optional[str] = None
    result_description: Optional[str] = None


@dataclass
class NodeState:
    node_id: str
    sequence_id: int
    released: bool
    node_description: Optional[str] = None
    node_position: Optional[NodePosition] = None
    rotation_allowed: Optional[bool] = None


@dataclass
class EdgeState:
    edge_id: str
    sequence_id: int
    released: bool
    edge_description: Optional[str] = None
    trajectory: Optional[Trajectory] = None
    # Speed and rotation constraints propagated from order edges
    max_speed: Optional[float] = None
    rotation_allowed: Optional[bool] = None
    max_rotation_speed: Optional[float] = None
    # Height constraints propagated from order edges (meters)
    max_height: Optional[float] = None
    min_height: Optional[float] = None


class EStop(str, Enum):
    None_ = "NONE"


@dataclass
class SafetyState:
    e_stop: EStop
    field_violation: bool


@dataclass
class BatteryState:
    battery_charge: float
    battery_voltage: Optional[float] = None
    battery_health: Optional[int] = None
    charging: bool = False
    reach: Optional[float] = None


@dataclass
class ForkState:
    # 货叉状态（单位：米），默认高度为 0.0m
    fork_height: float = 0.0


@dataclass
class Information:
    info_type: str
    info_references: List["InfoReference"]
    info_description: Optional[str] = None
    info_level: str = "INFO"


@dataclass
class InfoReference:
    reference_key: str
    reference_value: str


@dataclass
class State:
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    driving: bool
    operating_mode: OperatingMode
    node_states: List[NodeState]
    edge_states: List[EdgeState]
    last_node_id: str
    order_id: str
    order_update_id: int
    last_node_sequence_id: int
    action_states: List[ActionState]
    information: List[Information]
    loads: List[Load]
    battery_state: BatteryState
    safety_state: SafetyState
    paused: bool = False
    new_base_request: bool = False
    agv_position: Optional[AgvPosition] = None
    velocity: Optional[Velocity] = None
    zone_set_id: Optional[str] = None
    # 交互区域释放等待标记（默认 false）
    waiting_for_interaction_zone_release: bool = False
    # 货叉状态
    fork_state: ForkState = field(default_factory=ForkState)
    errors: List[dict] = field(default_factory=list)