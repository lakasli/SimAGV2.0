from __future__ import annotations
from typing import Any

def accept_instant_actions(sim_vehicle: Any, ia: Any) -> None:
    """即时动作管理封装。"""
    try:
        sim_vehicle.accept_instant_actions(ia)
    except Exception as e:
        raise RuntimeError(f"accept_instant_actions failed: {e}")