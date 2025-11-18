from __future__ import annotations
import time
import sys
from typing import Optional
import pathlib
import json

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
    try:
        sim_dir = pathlib.Path(__file__).resolve().parent
        base_dir = sim_dir / "agv_configs"
        base_dir.mkdir(parents=True, exist_ok=True)
        serial = str(config.vehicle.serial_number)
        safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in serial)
        fp = base_dir / f"{safe}.json"
        try:
            repo_root = pathlib.Path(__file__).resolve().parents[1]
            old_dir = repo_root / "backend" / "data" / "agv_configs"
            old_fp = old_dir / f"{safe}.json"
            if not fp.exists() and old_fp.exists():
                try:
                    import shutil as _shutil
                    _shutil.copy2(old_fp, fp)
                except Exception:
                    pass
        except Exception:
            pass
        if fp.exists():
            data = json.loads(fp.read_text(encoding="utf-8"))
            s = config.settings
            def g(name, default=None):
                v = data.get(name)
                return v if v is not None else default
            v = g("speed")
            if v is not None:
                s.speed = float(v)
            v = g("speed_min")
            if v is not None:
                s.speed_min = float(v)
            v = g("speed_max")
            if v is not None:
                s.speed_max = float(v)
            v = g("acceleration_max")
            if v is not None:
                s.acceleration_max = float(v)
            v = g("deceleration_max")
            if v is not None:
                s.deceleration_max = float(v)
            v = g("height_min")
            if v is not None:
                s.height_min = float(v)
            v = g("height_max")
            if v is not None:
                s.height_max = float(v)
            v = g("width")
            if v is not None:
                s.width = float(v)
            v = g("length")
            if v is not None:
                s.length = float(v)
            v = g("sim_time_scale")
            if v is not None:
                s.sim_time_scale = float(v)
            v = g("state_frequency")
            if v is not None:
                s.state_frequency = int(v)
            v = g("visualization_frequency")
            if v is not None:
                s.visualization_frequency = int(v)
            v = g("action_time")
            if v is not None:
                s.action_time = float(v)
            v = g("map_id")
            if v is not None:
                s.map_id = str(v)
    except Exception:
        pass
    sim = VehicleSimulator.create(config)
    try:
        sim_dir = pathlib.Path(__file__).resolve().parent
        fp = (sim_dir / "agv_configs" / ("".join(c if c.isalnum() or c in ("-", "_") else "_" for c in str(config.vehicle.serial_number)) + ".json"))
        if fp.exists():
            data = json.loads(fp.read_text(encoding="utf-8"))
            pos = data.get("last_position") or {}
            if pos:
                try:
                    sim.state.agv_position.x = float(pos.get("x", sim.state.agv_position.x))
                    sim.state.agv_position.y = float(pos.get("y", sim.state.agv_position.y))
                    th = pos.get("theta")
                    if th is not None:
                        sim.state.agv_position.theta = float(th)
                    sim.state.agv_position.position_initialized = True
                except Exception:
                    pass
            m = data.get("map_id")
            if m is not None:
                try:
                    sim.state.agv_position.map_id = str(m)
                    sim.state.zone_set_id = str(m)
                    sim.visualization.agv_position.map_id = str(m)
                    sim.visualization.zone_set_id = str(m)
                except Exception:
                    pass
    except Exception:
        pass
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