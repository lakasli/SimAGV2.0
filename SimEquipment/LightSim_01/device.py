from __future__ import annotations
from pathlib import Path
from typing import List

from SimEquipment.base import EquipmentConfig, EquipmentDevice, load_config


class LightSim(EquipmentDevice):
    def __init__(self, cfg: EquipmentConfig) -> None:
        super().__init__(cfg)
        self.on: bool = False
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
        }
        self._load_info_config()

    def _load_info_config(self) -> None:
        try:
            fp = Path(__file__).resolve().parent / "info_config.json"
            import json as _json
            data = _json.loads(fp.read_text(encoding="utf-8"))
            self._info_type = str(data.get("info_type", self._info_type))
            self._info_desc = data.get("info_description", self._info_desc)
            self._info_level = str(data.get("info_level", self._info_level))
            refs = data.get("references_by_state", {}) or {}
            for k in ("open","close"):
                v = refs.get(k) or refs.get(k.upper())
                if isinstance(v, list):
                    self._references_by_state[k] = v
        except Exception:
            pass

    def _build_information(self) -> list[dict]:
        refs = [
            {"referenceKey": "name", "referenceValue": "state"},
            {"referenceKey": "value", "referenceValue": ("state:on" if self.on else "state:off")},
        ]
        return [{
            "infoType": "LIGHTSTATE",
            "infoDescription": "STATE",
            "infoLevel": "INFO",
            "infoReferences": refs,
        }]

    def _start_thread(self) -> None:
        self._publish_factsheet("LIGHT")
        self._publish_state(self._state)
        super()._start_thread()

    def _run_loop(self) -> None:
        freq = max(1, int(self.cfg.settings.state_frequency))
        interval = 1.0 / float(freq)
        import time as _t
        while not self._stop:
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
                self._publish_factsheet("LIGHT")
                out_states.append({"action_id": str(aid), "action_status": "FINISHED", "action_type": "factsheetRequest"})
                continue
            if at == "writeValue":
                cmd = None
                for p in params:
                    if p.get("key") == "command":
                        cmd = str(p.get("value", ""))
                        break
                if cmd == "cmd:on":
                    self.on = True
                    mode = str(self.cfg.settings.trigger_mode or "instant")
                    if mode == "cancel" and self._act_running:
                        self.on = False
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
                elif cmd == "cmd:off":
                    self.on = False
                    mode = str(self.cfg.settings.trigger_mode or "instant")
                    if mode == "hold":
                        if self._act_running:
                            out_states.append({"action_id": str(aid), "action_status": "FINISHED", "action_type": "writeValue"})
                        self._act_running = False
                    else:
                        out_states.append({"action_id": str(aid), "action_status": "FINISHED", "action_type": "writeValue"})
                else:
                    continue
                
        if out_states:
            self._state["action_states"] = out_states
            self._publish_state(self._state)
            if str(self.cfg.settings.trigger_mode or "instant") != "hold":
                try:
                    import time as _t
                    _t.sleep(max(0.0, float(self.cfg.settings.action_time)))
                except Exception:
                    pass
                for s in self._state.get("action_states", []):
                    if s.get("action_status") not in ("FAILED", "PAUSED"):
                        s["action_status"] = "FINISHED"
                self._publish_state(self._state)


def run(dir_path: str) -> None:
    cfg = load_config(Path(dir_path))
    sim = LightSim(cfg)
    sim.start()
    try:
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim.stop()
