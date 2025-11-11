from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Union


class BlockingType(str, Enum):
    None_ = "NONE"
    Soft = "SOFT"
    Hard = "HARD"


ActionParameterValue = Union[int, float, str]


@dataclass
class ActionParameter:
    key: str
    value: ActionParameterValue


@dataclass
class Action:
    action_type: str
    action_id: str
    blocking_type: BlockingType
    action_description: Optional[str] = None
    action_parameters: Optional[List[ActionParameter]] = None