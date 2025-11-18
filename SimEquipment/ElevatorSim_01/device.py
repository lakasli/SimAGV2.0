from __future__ import annotations
import json
from pathlib import Path
from typing import Any, List

from SimEquipment.base import EquipmentConfig, EquipmentDevice, load_config


class ElevatorSim(EquipmentDevice):
    def __init__(self, cfg: EquipmentConfig) -> None:
        super().__init__(cfg)
        self.door_open: bool = False
        self.current_floor: int = 1
        self.floors: List[int] = [1, 2, 3]
        self._state = self._build_base_state()
        self._act_running = False
        self._act_paused = False
        self._act_remaining = float(self.cfg.settings.action_time)
        self._last_trigger_time = 0.0
        self._act_id: str | None = None
        self._act_type: str | None = None

    def _start_thread(self) -> None:
        self._publish_factsheet("ELEVATOR")
        self._publish_state(self._state)
        super()._start_thread()

    def _run_loop(self) -> None:
        freq = max(1, int(self.cfg.settings.state_frequency))
        interval = 1.0 / float(freq)
        import time as _t
        while not self._stop:
            self._state["information"] = [
                {
                    "info_type": "ElevatorState",
                    "info_references": [
                        {"reference_key": "doorOpen", "reference_value": "true" if self.door_open else "false"},
                        {"reference_key": "floor", "reference_value": str(self.current_floor)},
                        {"reference_key": "site", "reference_value": str(self.cfg.settings.site or "")}
                    ],
                    "info_description": None,
                    "info_level": "INFO",
                }
            ]
            self._publish_state(self._state)
            try:
                if self._act_running:
                    mode = str(self.cfg.settings.trigger_mode or "instant")
                    if mode == "hold":
                        if (_t.time() - float(self._last_trigger_time)) > max(0.0, float(self.cfg.settings.action_time)):
                            if self._act_id and self._act_type:
                                self._state["action_states"] = [{"action_id": self._act_id, "action_status": "FAILED", "action_type": self._act_type, "result_description": "interrupted"}]
                                self._publish_state(self._state)
                            self._act_running = False
                    else:
                        if not self._act_paused:
                            self._act_remaining = max(0.0, float(self._act_remaining) - interval)
                            if self._act_remaining <= 0.0:
                                if self._act_id and self._act_type:
                                    self._state["action_states"] = [{"action_id": self._act_id, "action_status": "FINISHED", "action_type": self._act_type}]
                                    self._publish_state(self._state)
                                self._act_running = False
                                self._act_remaining = float(self.cfg.settings.action_time)
            except Exception:
                pass
            _t.sleep(interval)

    def _handle_instant_actions(self, data: dict) -> None:
        actions: List[dict] = data.get("actions", data.get("actions", [])) or []
        out_states = []
        for a in actions:
            at = a.get("action_type", a.get("actionType", ""))
            aid = a.get("action_id", a.get("actionId", ""))
            params = a.get("action_parameters", a.get("actionParameters", [])) or []
            if at == "factsheetRequest":
                self._publish_factsheet("ELEVATOR")
                out_states.append({"action_id": str(aid), "action_status": "FINISHED", "action_type": "factsheetRequest"})
                continue
            if at == "writeValue":
                cmd = None
                for p in params:
                    k = p.get("key", p.get("key", ""))
                    if k == "command":
                        cmd = str(p.get("value", ""))
                        break
                # 支持 cmd:press 或 cmd:press:<floor>
                if cmd == "cmd:press":
                    self.door_open = True
                    mode = str(self.cfg.settings.trigger_mode or "instant")
                    if mode == "cancel" and self._act_running:
                        self.door_open = False
                        out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "writeValue", "result_description": "canceled"})
                        self._act_running = False
                    elif mode == "pause" and self._act_running:
                        self._act_paused = not self._act_paused
                        out_states.append({"action_id": str(aid), "action_status": ("PAUSED" if self._act_paused else "RUNNING"), "action_type": "writeValue"})
                    else:
                        self._act_running = True
                        self._act_paused = False
                        self._act_remaining = float(self.cfg.settings.action_time)
                        self._last_trigger_time = __import__("time").time()
                        self._act_id = str(aid)
                        self._act_type = "writeValue"
                        out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "writeValue"})
                elif cmd and cmd.startswith("cmd:press:"):
                    try:
                        tgt = int(cmd.split(":")[-1])
                    except Exception:
                        tgt = None
                    if tgt is not None and tgt in self.floors:
                        self.current_floor = int(tgt)
                        mode = str(self.cfg.settings.trigger_mode or "instant")
                        if mode == "cancel" and self._act_running:
                            out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "writeValue", "result_description": "canceled"})
                            self._act_running = False
                        elif mode == "pause" and self._act_running:
                            self._act_paused = not self._act_paused
                            out_states.append({"action_id": str(aid), "action_status": ("PAUSED" if self._act_paused else "RUNNING"), "action_type": "writeValue"})
                        else:
                            self._act_running = True
                            self._act_paused = False
                            self._act_remaining = float(self.cfg.settings.action_time)
                            self._last_trigger_time = __import__("time").time()
                            self._act_id = str(aid)
                            self._act_type = "writeValue"
                            out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "writeValue"})
                    else:
                        out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "writeValue", "result_description": "invalidFloor"})
                else:
                    continue
            elif at == "openDoor":
                self.door_open = True
                mode = str(self.cfg.settings.trigger_mode or "instant")
                if mode == "cancel" and self._act_running:
                    self.door_open = False
                    out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "openDoor", "result_description": "canceled"})
                    self._act_running = False
                elif mode == "pause" and self._act_running:
                    self._act_paused = not self._act_paused
                    out_states.append({"action_id": str(aid), "action_status": ("PAUSED" if self._act_paused else "RUNNING"), "action_type": "openDoor"})
                else:
                    self._act_running = True
                    self._act_paused = False
                    self._act_remaining = float(self.cfg.settings.action_time)
                    self._last_trigger_time = __import__("time").time()
                    self._act_id = str(aid)
                    self._act_type = "openDoor"
                    out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "openDoor"})
            elif at == "closeDoor":
                self.door_open = False
                mode = str(self.cfg.settings.trigger_mode or "instant")
                if mode == "cancel" and self._act_running:
                    out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "closeDoor", "result_description": "canceled"})
                    self._act_running = False
                elif mode == "pause" and self._act_running:
                    self._act_paused = not self._act_paused
                    out_states.append({"action_id": str(aid), "action_status": ("PAUSED" if self._act_paused else "RUNNING"), "action_type": "closeDoor"})
                else:
                    self._act_running = True
                    self._act_paused = False
                    self._act_remaining = float(self.cfg.settings.action_time)
                    self._last_trigger_time = __import__("time").time()
                    self._act_id = str(aid)
                    self._act_type = "closeDoor"
                    out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "closeDoor"})
            elif at == "moveToFloor":
                tgt = None
                for p in params:
                    k = p.get("key", p.get("key", ""))
                    if k == "floor":
                        try:
                            tgt = int(p.get("value"))
                        except Exception:
                            tgt = None
                        break
                if tgt is not None and tgt in self.floors:
                    self.current_floor = int(tgt)
                    mode = str(self.cfg.settings.trigger_mode or "instant")
                    if mode == "cancel" and self._act_running:
                        out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "moveToFloor", "result_description": "canceled"})
                        self._act_running = False
                    elif mode == "pause" and self._act_running:
                        self._act_paused = not self._act_paused
                        out_states.append({"action_id": str(aid), "action_status": ("PAUSED" if self._act_paused else "RUNNING"), "action_type": "moveToFloor"})
                    else:
                        self._act_running = True
                        self._act_paused = False
                        self._act_remaining = float(self.cfg.settings.action_time)
                        self._last_trigger_time = __import__("time").time()
                        self._act_id = str(aid)
                        self._act_type = "moveToFloor"
                        out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "moveToFloor"})
                else:
                    out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "moveToFloor", "result_description": "invalidFloor"})
        if out_states:
            self._state["action_states"] = out_states
            self._publish_state(self._state)
            try:
                import time as _t
                _t.sleep(max(0.0, float(self.cfg.settings.action_time)))
            except Exception:
                pass
            for s in self._state.get("action_states", []):
                if s.get("action_status") != "FAILED":
                    s["action_status"] = "FINISHED"
            self._publish_state(self._state)


def run(dir_path: str) -> None:
    cfg = load_config(Path(dir_path))
    sim = ElevatorSim(cfg)
    sim.start()
    try:
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim.stop()