from __future__ import annotations
import threading
import time
from typing import Any, Optional
from SimVehicleSys.utils.logger import setup_logger


def set_sim_time_scale(config: Any, scale: float) -> None:
    """设置全局仿真时间缩放因子，影响状态/可视化发布速率与导航推进。"""
    try:
        config.settings.sim_time_scale = float(scale)
    except Exception as e:
        raise RuntimeError(f"set_sim_time_scale failed: {e}")


class SimClock:
    """
    统一时钟调度器：驱动仿真器动作（update_state）、电池充放电（可选）、以及状态/可视化发布周期。

    - tick_ms：基础时间步长（毫秒）。
    - 发布频率=配置频率×时间缩放因子（sim_time_scale）。
    """

    def __init__(self, simulator: Any, config: Any, battery_manager: Optional[Any] = None, tick_ms: int = 50) -> None:
        self.sim = simulator
        self.config = config
        self.battery_manager = battery_manager
        self.tick_ms = int(tick_ms)
        self._thread: Optional[threading.Thread] = None
        self._stop = False
        self._clock_logged = False
        self.logger = setup_logger()

    def start(self, mqtt_client: Any) -> None:
        if self._thread and self._thread.is_alive():
            return
        try:
            if self.battery_manager:
                self.battery_manager.start()
        except Exception:
            pass
        self._stop = False
        self._thread = threading.Thread(target=self._loop, args=(mqtt_client,), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    def _loop(self, mqtt_client: Any) -> None:
        state_elapsed_ms = 0.0
        visualization_elapsed_ms = 0.0
        base_tick = max(1, self.tick_ms)
        while not self._stop:
            try:
                self.sim.update_state()
                scale = max(0.0001, float(self.config.settings.sim_time_scale))
                eff_state_freq = max(1e-6, float(self.config.settings.state_frequency) * scale)
                eff_vis_freq = max(1e-6, float(self.config.settings.visualization_frequency) * scale)
                if not self._clock_logged:
                    try:
                        self.logger.info(
                            f"[Clock] tick={base_tick}ms scale={scale:.2f} eff_state_freq={eff_state_freq:.2f} eff_vis_freq={eff_vis_freq:.2f}"
                        )
                    except Exception:
                        pass
                    self._clock_logged = True
                state_elapsed_ms += base_tick
                if state_elapsed_ms >= (1000.0 / eff_state_freq):
                    state_elapsed_ms = 0.0
                    self.sim.publish_state(mqtt_client)
                visualization_elapsed_ms += base_tick
                if visualization_elapsed_ms >= (1000.0 / eff_vis_freq):
                    visualization_elapsed_ms = 0.0
                    self.sim.publish_visualization(mqtt_client)
            except Exception as e:
                try:
                    self.logger.error(f"SimClock tick error: {e}")
                except Exception:
                    print(f"SimClock tick error: {e}")
            time.sleep(base_tick / 1000.0)