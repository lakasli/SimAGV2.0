from __future__ import annotations
import json
from pathlib import Path
from typing import Any, List

from SimEquipment.base import EquipmentConfig, EquipmentDevice, load_config
from SimVehicleSys.mqtt.client import publish_json


class DoorSim(EquipmentDevice):
    def __init__(self, cfg: EquipmentConfig) -> None:
        super().__init__(cfg)
        self.door_open: bool = False
        self._state = self._build_base_state()
        self._act_running = False
        self._act_paused = False
        self._act_remaining = float(self.cfg.settings.action_time)
        self._last_trigger_time = 0.0
        self._act_id: str | None = None
        self._act_type: str | None = None
        self._pending_target: bool | None = None
        self._instant_cache: dict[str, dict] = {}
        self._delay_seconds: float = 3.0
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
        }
        self._load_info_config()

    def _load_info_config(self) -> None:
        try:
            fp = Path(__file__).resolve().parent / "info_config.json"
            data = json.loads(fp.read_text(encoding="utf-8"))
            self._info_type = str(data.get("info_type", self._info_type))
            self._info_desc = data.get("info_description", self._info_desc)
            self._info_level = str(data.get("info_level", self._info_level))
            refs = data.get("references_by_state", {}) or {}
            open_refs = refs.get("open") or refs.get("OPEN")
            close_refs = refs.get("close") or refs.get("CLOSE")
            if isinstance(open_refs, list) and isinstance(close_refs, list):
                self._references_by_state = {"open": open_refs, "close": close_refs}
        except Exception:
            pass

    def _build_information(self) -> list[dict]:
        refs = [
            {"referenceKey": "name", "referenceValue": "state"},
            {"referenceKey": "value", "referenceValue": ("state:open" if self.door_open else "state:close")},
        ]
        return [{
            "infoType": "DOORSTATE",
            "infoDescription": "STATE",
            "infoLevel": "INFO",
            "infoReferences": refs,
        }]

    def _start_thread(self) -> None:
        self._publish_factsheet("DOOR")
        self._publish_state(self._state)
        super()._start_thread()

    def _run_loop(self) -> None:
        freq = max(1, int(self.cfg.settings.state_frequency))
        interval = 1.0 / float(freq)
        import time as _t
        while not self._stop:
            self._state["information"] = self._build_information()
            if self._instant_cache:
                self._state["action_states"] = list(self._instant_cache.values())
            self._publish_state(self._state)
            # 处理动作计时/保持/完成
            try:
                if self._act_running:
                    mode = str(self.cfg.settings.trigger_mode or "instant")
                    if mode == "hold":
                        if (_t.time() - float(self._last_trigger_time)) > max(0.0, float(self.cfg.settings.action_time)):
                            self.door_open = False
                            if self._act_id and self._act_type:
                                self._state["action_states"] = [{"action_id": self._act_id, "action_status": "FAILED", "action_type": self._act_type, "result_description": "interrupted"}]
                                self._publish_state(self._state)
                            self._act_running = False
                    else:
                        if not self._act_paused:
                            self._act_remaining = max(0.0, float(self._act_remaining) - interval)
                            if self._act_remaining <= 0.0:
                                if self._act_id and self._act_type:
                                    if self._pending_target is not None:
                                        self.door_open = bool(self._pending_target)
                                    self._state["information"] = self._build_information()
                                    desc = None
                                    if self._act_type == "writeValue":
                                        desc = ("Command \"cmd:open\" written successfully to register \"set\"" if self.door_open else "Command \"cmd:close\" written successfully to register \"set\"")
                                    finished_entry = {
                                        "action_id": self._act_id,
                                        "action_status": "FINISHED",
                                        "action_type": self._act_type,
                                        "result_description": desc,
                                    }
                                    try:
                                        if self._act_type in self._instant_cache:
                                            base_entry = self._instant_cache.get(self._act_type, {})
                                            base_entry.update(finished_entry)
                                            self._state["action_states"] = [base_entry]
                                            self._publish_state(self._state)
                                            del self._instant_cache[self._act_type]
                                        else:
                                            self._state["action_states"] = [finished_entry]
                                            self._publish_state(self._state)
                                    except Exception:
                                        self._state["action_states"] = [finished_entry]
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
            if at:
                entry = {
                    "action_type": at,
                    "action_id": str(aid),
                    "action_status": "RUNNING",
                    "action_parameters": params,
                    "blocking_type": "HARD",
                    "timestamp": int(__import__("time").time() * 1000),
                }
                self._instant_cache[at] = entry
            if at == "factsheetRequest":
                self._publish_factsheet("DOOR")
                out_states.append({"action_id": str(aid), "action_status": "FINISHED", "action_type": "factsheetRequest"})
                continue
            if at == "writeValue":
                cmd = None
                for p in params:
                    k = p.get("key", p.get("key", ""))
                    if k == "command":
                        cmd = str(p.get("value", ""))
                        break
                if cmd == "cmd:open":
                    self._pending_target = True
                elif cmd == "cmd:close":
                    self._pending_target = False
                else:
                    continue
                try:
                    desc = f"Write command \"{cmd}\" to register \"set\""
                except Exception:
                    desc = None
                self._instant_cache["writeValue"] = {
                    "action_type": "writeValue",
                    "action_id": str(aid),
                    "action_status": "RUNNING",
                    "action_description": desc,
                    "action_parameters": [
                        {"key": "registerName", "value": "set"},
                        {"key": "command", "value": cmd},
                    ],
                    "blocking_type": "HARD",
                    "timestamp": int(__import__("time").time() * 1000),
                }
                mode = str(self.cfg.settings.trigger_mode or "instant")
                if mode == "cancel" and self._act_running:
                    # 第二次短按取消
                    self._pending_target = None
                    out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": "writeValue", "result_description": "canceled"})
                    self._act_running = False
                elif mode == "pause" and self._act_running:
                    # 第二次短按暂停/恢复
                    self._act_paused = not self._act_paused
                    out_states.append({"action_id": str(aid), "action_status": ("PAUSED" if self._act_paused else "RUNNING"), "action_type": "writeValue"})
                else:
                    # 启动或保持
                    self._act_running = True
                    self._act_paused = False
                    self._act_remaining = max(0.0, float(self._delay_seconds))
                    self._last_trigger_time = __import__("time").time()
                    self._act_id = str(aid)
                    self._act_type = "writeValue"
                    out_states.append({"action_id": str(aid), "action_status": ("RUNNING" if mode != "hold" else "RUNNING"), "action_type": "writeValue"})
            elif at in ("openDoor", "closeDoor", "toggleDoor"):
                if at == "openDoor":
                    self._pending_target = True
                elif at == "closeDoor":
                    self._pending_target = False
                else:
                    self._pending_target = (not self.door_open)
                self._instant_cache[at] = {
                    "action_type": at,
                    "action_id": str(aid),
                    "action_status": "RUNNING",
                    "blocking_type": "HARD",
                    "timestamp": int(__import__("time").time() * 1000),
                }
                mode = str(self.cfg.settings.trigger_mode or "instant")
                if mode == "cancel" and self._act_running:
                    self._pending_target = None
                    out_states.append({"action_id": str(aid), "action_status": "FAILED", "action_type": at, "result_description": "canceled"})
                    self._act_running = False
                elif mode == "pause" and self._act_running:
                    self._act_paused = not self._act_paused
                    out_states.append({"action_id": str(aid), "action_status": ("PAUSED" if self._act_paused else "RUNNING"), "action_type": at})
                else:
                    self._act_running = True
                    self._act_paused = False
                    self._act_remaining = max(0.0, float(self._delay_seconds))
                    self._last_trigger_time = __import__("time").time()
                    self._act_id = str(aid)
                    self._act_type = at
                    out_states.append({"action_id": str(aid), "action_status": "RUNNING", "action_type": at})
        if out_states:
            self._state["action_states"] = list(self._instant_cache.values())
            self._publish_state(self._state)


def run(dir_path: str) -> None:
    cfg = load_config(Path(dir_path))
    sim = DoorSim(cfg)
    sim.start()
    try:
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim.stop()
