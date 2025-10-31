from __future__ import annotations
import threading
import time
from typing import Optional

from SimVehicleSys.config.settings import Config
from .vehicle import VehicleSimulator
from SimVehicleSys.utils.helpers import get_distance
from .error_manager import emit_error


class BatteryManager:
    """
    独立电池管理线程：按照现实时间进行耗电/充电。

    - 空闲：每分钟掉电 `battery_idle_drain_per_min` 百分点
    - 空载运动：掉电速度 = 空闲 × `battery_move_empty_multiplier`
    - 载货运动：掉电速度 = 空闲 × `battery_move_loaded_multiplier`
    - 充电：进入以 "CP" 开头的站点时，每分钟充电 `battery_charge_per_min` 百分点

    所有参数来源 config.settings，并可在 config.toml 配置。
    """

    def __init__(self, simulator: VehicleSimulator, config: Config):
        self.sim = simulator
        self.config = config
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self._last_ts = time.time()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    def _is_at_charging_point(self) -> bool:
        try:
            last_node_id = str(self.sim.state.last_node_id or "")
            return last_node_id.startswith("CP")
        except Exception:
            return False

    def _has_charging_command(self) -> bool:
        try:
            return bool(getattr(self.sim, "charging_requested", False))
        except Exception:
            return False

    def _is_loaded(self) -> bool:
        try:
            loads = self.sim.state.loads or []
            return len(loads) > 0
        except Exception:
            return False

    def _is_moving(self) -> bool:
        st = self.sim.state
        try:
            if not st.agv_position or not st.node_states:
                return False
            last_idx = 0
            for i, ns in enumerate(st.node_states):
                if ns.sequence_id == st.last_node_sequence_id:
                    last_idx = i
                    break
            if last_idx >= len(st.node_states) - 1:
                return False
            next_node = st.node_states[last_idx + 1]
            np = next_node.node_position
            vp = st.agv_position
            if not np or not next_node.released:
                return False
            distance = get_distance(vp.x, vp.y, np.x, np.y)
            return distance > 0.05
        except Exception:
            return False

    def _loop(self) -> None:
        while not self._stop:
            now = time.time()
            dt = max(0.0, now - self._last_ts)
            self._last_ts = now

            try:
                st = self.sim.state
                bs = st.battery_state
                charge = float(bs.battery_charge)

                if self._is_at_charging_point() and self._has_charging_command():
                    rate_per_min = float(self.config.settings.battery_charge_per_min)
                    delta = rate_per_min * (dt / 60.0)
                    charge = min(100.0, charge + delta)
                    bs.charging = True
                    # 充满则自动清除充电请求
                    if charge >= 99.9:
                        try:
                            setattr(self.sim, "charging_requested", False)
                        except Exception:
                            pass
                else:
                    base = float(self.config.settings.battery_idle_drain_per_min)
                    if self._is_moving():
                        mult = float(self.config.settings.battery_move_loaded_multiplier if self._is_loaded() else self.config.settings.battery_move_empty_multiplier)
                        rate_per_min = base * mult
                    else:
                        rate_per_min = base
                    delta = rate_per_min * (dt / 60.0)
                    charge = max(0.0, charge - delta)
                    bs.charging = False
                    # 离开 CP 则清除充电请求（避免下次自动充电）
                    if not self._is_at_charging_point():
                        try:
                            setattr(self.sim, "charging_requested", False)
                        except Exception:
                            pass

                bs.battery_charge = charge

                # 示例触发点：低电量告警与极低电量致命错误
                try:
                    serial = self.sim.config.vehicle.serial_number
                    ctx = {"serial_number": serial, "battery_charge": round(charge, 2)}
                    if charge <= 10.0:
                        emit_error(52503, ctx)  # battery is too low to move
                    elif charge <= 30.0:
                        emit_error(54211, ctx)  # low battery
                except Exception:
                    pass
            except Exception as e:
                print(f"BatteryManager error: {e}")

            time.sleep(0.5)