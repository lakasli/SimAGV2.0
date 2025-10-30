from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from ..vda5050_common import HeaderId


class ConnectionState(str, Enum):
    Online = "ONLINE"
    Offline = "OFFLINE"
    ConnectionBroken = "CONNECTIONBROKEN"


@dataclass
class Connection:
    header_id: HeaderId
    timestamp: str
    version: str
    manufacturer: str
    serial_number: str
    connection_state: ConnectionState