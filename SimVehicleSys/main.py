from __future__ import annotations
import time
import sys
from typing import Optional

from SimVehicleSys.config.settings import get_config
from SimVehicleSys.sim_vehicle.vehicle import VehicleSimulator
from SimVehicleSys.sim_vehicle.connection_manager import ConnectionManager


def _parse_args() -> tuple[Optional[str], Optional[str]]:
    """Parse optional CLI args: --config and --serial.

    Returns (config_path, override_serial).
    """
    config_path: Optional[str] = None
    override_serial: Optional[str] = None
    try:
        # Minimal manual parsing to avoid adding dependencies
        argv = sys.argv[1:]
        i = 0
        while i < len(argv):
            arg = argv[i]
            if arg == "--config" and i + 1 < len(argv):
                config_path = argv[i + 1]
                i += 2
                continue
            if arg == "--serial" and i + 1 < len(argv):
                override_serial = argv[i + 1]
                i += 2
                continue
            i += 1
    except Exception:
        pass
    return config_path, override_serial


def main() -> None:
    cfg_path, override_serial = _parse_args()
    config = get_config(cfg_path)
    if override_serial:
        try:
            config.vehicle.serial_number = str(override_serial)
        except Exception:
            pass
    sim = VehicleSimulator.create(config)
    handler = ConnectionManager(config, sim)
    handler.start()
    print(
        f"SimVehicleSys: Robot '{config.vehicle.serial_number}' registered and publishing. Press Ctrl+C to stop."
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")


if __name__ == "__main__":
    main()