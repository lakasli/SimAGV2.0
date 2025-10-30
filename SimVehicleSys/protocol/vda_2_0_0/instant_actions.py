from __future__ import annotations
from dataclasses import dataclass
from typing import List

from ..vda5050_common import HeaderId
from .action import Action


@dataclass
class InstantActions:
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    actions: List[Action]