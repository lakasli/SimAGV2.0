from __future__ import annotations
import json
from pathlib import Path
from typing import Any, List

from SimEquipment.base import EquipmentConfig, EquipmentDevice, load_config


class ElevatorSim(EquipmentDevice):
    def __init__(self, cfg: EquipmentConfig) -> None:
        super().__init__(cfg)
        self.door_open: bool = True
        self.current_floor: int = 1
        self.floors: List[int] = list(range(-99, 99))
        self._state = self._build_base_state()
        self._act_running = False
        self._act_paused = False
        self._act_remaining = float(self.cfg.settings.action_time)
        self._last_trigger_time = 0.0
        self._act_id: str | None = None
        self._act_type: str | None = None
        self._info_type: str = "CUSTOM_REGISTER"
        self._info_desc: str | None = "REGISTER_STATE"
        self._info_level: str = "INFO"
        self._references_by_state: dict[str, list[dict]] = {
            "open": [
                {"reference_key": "name", "reference_value": "state"},
                {"reference_key": "value", "reference_value": "state:open"},
            ],
            "close": [
                {"reference_key": "name", "reference_value": "state"},
                {"reference_key": "value", "reference_value": "state:close"},
            ],
            "up": [
                {"reference_key": "name", "reference_value": "state"},
                {"reference_key": "value", "reference_value": "state:up"},
            ],
            "down": [
                {"reference_key": "name", "reference_value": "state"},
                {"reference_key": "value", "reference_value": "state:down"},
            ],
            "floor": [
                {"reference_key": "name", "reference_value": "state"},
                {"reference_key": "value", "reference_value": "state:floor"},
            ],
        }
        self._move_dir: str = "stop"
        self._door_close_due: float = 0.0
        self._door_open_due: float = 0.0
        self._next_floor_due: float = 0.0
        self._travel_plan: list[int] = []
        self._load_info_config()

    def _load_info_config(self) -> None:
        try:
            fp = Path(__file__).resolve().parent / "info_config.json"
            data = json.loads(fp.read_text(encoding="utf-8"))
            self._info_type = str(data.get("info_type", self._info_type))
            self._info_desc = data.get("info_description", self._info_desc)
            self._info_level = str(data.get("info_level", self._info_level))
            refs = data.get("references_by_state", {}) or {}
            for k in ("open","close","up","down","floor"):
                v = refs.get(k) or refs.get(k.upper())
                if isinstance(v, list):
                    self._references_by_state[k] = v
        except Exception:
            pass

    def _build_information(self) -> list[dict]:
        refs: list[dict] = []
        refs.append({"referenceKey": "name", "referenceValue": "state"})
        refs.append({"referenceKey": "value", "referenceValue": ("state:open" if self.door_open else "state:close")})
        refs.append({"referenceKey": "value", "referenceValue": f"state:floor:{self.current_floor}"})
        if self._move_dir == "up":
            refs.append({"referenceKey": "value", "referenceValue": "state:up"})
        elif self._move_dir == "down":
            refs.append({"referenceKey": "value", "referenceValue": "state:down"})
        return [
            {
                "infoType": "ELEVATORSTATE",
                "infoDescription": "STATE",
                "infoLevel": "INFO",
                "infoReferences": refs,
            }
        ]

    def _start_thread(self) -> None:
        self._publish_factsheet("ELEVATOR")
        self._publish_state(self._state)
        super()._start_thread()

    def _run_loop(self) -> None:
        freq = max(1, int(self.cfg.settings.state_frequency))
        interval = 1.0 / float(freq)
        import time as _t
        while not self._stop:
            now = _t.time()
            if self._door_close_due and now >= self._door_close_due and self.door_open:
                self.door_open = False
                self._door_close_due = 0.0
                if self._travel_plan:
                    self._next_floor_due = now + 2.0
            if self._door_open_due and now >= self._door_open_due and not self.door_open:
                self.door_open = True
                self._door_open_due = 0.0
            if self._next_floor_due and now >= self._next_floor_due and self._travel_plan:
                nxt = self._travel_plan.pop(0)
                prev = self.current_floor
                self.current_floor = int(nxt)
                if self._travel_plan:
                    self._move_dir = "up" if self.current_floor > prev else ("down" if self.current_floor < prev else self._move_dir)
                    self._next_floor_due = now + 2.0
                else:
                    self._move_dir = "stop"
                    self._door_open_due = now + 3.0
                    self._next_floor_due = 0.0
                    if self._act_id and self._act_type:
                        self._state["action_states"] = [{"action_id": self._act_id, "action_status": "FINISHED", "action_type": self._act_type}]
                        self._publish_state(self._state)
                        self._act_id = None
                        self._act_type = None
                        self._act_running = False
            self._state["information"] = self._build_information()
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
                            if not (self._travel_plan or self._door_close_due or self._next_floor_due):
                                self._act_remaining = max(0.0, float(self._act_remaining) - interval)
                        if (not (self._travel_plan or self._door_close_due or self._next_floor_due)) and (self._act_remaining <= 0.0):
                            if self._act_id and self._act_type:
                                self._state["action_states"] = [{"action_id": self._act_id, "action_status": "FINISHED", "action_type": self._act_type}]
                                self._publish_state(self._state)
                            self._move_dir = "stop"
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
                    self._move_dir = "stop"
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
                        self._door_close_due = __import__("time").time() + 3.0
                        self._act_id = str(aid)
                        self._act_type = "writeValue"
                        out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "writeValue"})
                elif cmd and cmd.startswith("cmd:press:"):
                    try:
                        tgt = int(cmd.split(":")[-1])
                    except Exception:
                        tgt = None
                    if tgt is not None and tgt in self.floors:
                        prev = self.current_floor
                        path: list[int] = []
                        if tgt > prev:
                            path = list(range(prev + 1, tgt + 1))
                            self._move_dir = "up"
                        elif tgt < prev:
                            path = list(range(prev - 1, tgt - 1, -1))
                            self._move_dir = "down"
                        else:
                            path = []
                            self._move_dir = "stop"
                        self._travel_plan = path
                        self._door_close_due = __import__("time").time() + 3.0
                        self._next_floor_due = 0.0
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
                            self._act_id = str(aid)
                            self._act_type = "writeValue"
                            out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "writeValue"})
                    else:
                        out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "writeValue", "result_description": "invalidFloor"})
                else:
                    continue
            elif at == "openDoor":
                self.door_open = True
                self._move_dir = "stop"
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
                self._move_dir = "stop"
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
                    prev = self.current_floor
                    path: list[int] = []
                    if tgt > prev:
                        path = list(range(prev + 1, tgt + 1))
                        self._move_dir = "up"
                    elif tgt < prev:
                        path = list(range(prev - 1, tgt - 1, -1))
                        self._move_dir = "down"
                    else:
                        path = []
                        self._move_dir = "stop"
                    self._travel_plan = path
                    self._door_close_due = __import__("time").time() + 3.0
                    self._next_floor_due = 0.0
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
                        self._act_id = str(aid)
                        self._act_type = "moveToFloor"
                        out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": "moveToFloor"})
                else:
                    out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "moveToFloor", "result_description": "invalidFloor"})
        if out_states:
            self._state["action_states"] = out_states
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
