import json
import sys
from pathlib import Path

# 允许脚本直接运行时找到项目根作为模块路径
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from SimVehicleSys.config.settings import get_config
from SimVehicleSys.sim_vehicle.vehicle import create_vehicle
from SimVehicleSys.sim_vehicle.action_executor import execute_pallet_action_in_sim
from SimVehicleSys.sim_vehicle.order_manager import _parse_order

def main():
    cfg = get_config()
    veh = create_vehicle(cfg)
    print("height_min/max:", veh.config.settings.height_min, veh.config.settings.height_max)
    execute_pallet_action_in_sim(veh, "pick", None)
    print("after pick(None): fork_height=", veh.state.fork_state.fork_height)
    execute_pallet_action_in_sim(veh, "drop", None)
    print("after drop(None): fork_height=", veh.state.fork_state.fork_height)
    with open("order.json", "r", encoding="utf-8") as f:
        d = json.load(f)
    order = _parse_order(d, None, veh.config.vehicle)
    acts = order.nodes[-1].actions
    print("parsed actionType:", [a.action_type for a in acts])
    print("parsed actionParameters is None?:", [a.action_parameters is None for a in acts])

if __name__ == "__main__":
    main()
