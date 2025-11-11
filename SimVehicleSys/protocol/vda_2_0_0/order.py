from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional

from ..vda5050_common import HeaderId, NodePosition, Trajectory
from .action import Action


@dataclass
class Node:
    node_id: str
    sequence_id: int
    released: bool
    node_description: Optional[str] = None
    node_position: Optional[NodePosition] = None
    actions: List[Action] = None


@dataclass
class Edge:
    edge_id: str
    sequence_id: int
    released: bool
    start_node_id: str
    end_node_id: str
    edge_description: Optional[str] = None
    max_speed: Optional[float] = None
    max_height: Optional[float] = None
    min_height: Optional[float] = None
    orientation: Optional[float] = None
    orientation_type: Optional[str] = None
    direction: Optional[str] = None
    rotation_allowed: Optional[bool] = None
    max_rotation_speed: Optional[float] = None
    length: Optional[float] = None
    trajectory: Optional[Trajectory] = None
    actions: List[Action] = None


@dataclass
class Order:
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    order_id: str
    order_update_id: int
    zone_set_id: Optional[str]
    nodes: List[Node]
    edges: List[Edge]