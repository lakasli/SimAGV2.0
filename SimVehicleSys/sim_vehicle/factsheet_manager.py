from __future__ import annotations
from typing import Any

def publish_factsheet(sim_vehicle: Any, mqtt_client: Any) -> None:
    """事实信息（物理参数）发布封装。委托仿真器实例执行。"""
    try:
        sim_vehicle.publish_factsheet(mqtt_client)
    except Exception as e:
        raise RuntimeError(f"publish_factsheet failed: {e}")