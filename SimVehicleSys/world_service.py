from __future__ import annotations
import threading
import time
from typing import Dict, Any, List, Tuple

import paho.mqtt.client as mqtt

from SimVehicleSys.config.settings import get_config
from SimVehicleSys.config.mqtt_config import generate_vda_mqtt_base_topic
from SimVehicleSys.mqtt.client import publish_json
from SimVehicleSys.utils.helpers import get_topic_type, get_timestamp
from SimVehicleSys.world_manager import computeSafetyRectForStatePayload, rects_overlap


class WorldModelService:
    """
    世界模型服务：集中订阅所有设备的 VDA5050 state，做多车碰撞判定，并向双方下发 stopPause。
    - 使用 MQTT 进行跨进程状态收集与动作下发；不依赖进程内共享内存。
    - 默认每 100ms 进行一次碰撞检测。
    """

    def __init__(self, tickMs: int = 100) -> None:
        cfg = get_config()
        self.host = str(getattr(cfg.mqtt_broker, "host", "127.0.0.1"))
        self.port = int(getattr(cfg.mqtt_broker, "port", 1883))
        self.vdaInterface = str(getattr(cfg.mqtt_broker, "vda_interface", "uagv"))
        self.defaultLength = float(getattr(cfg.settings, "length", 1.03))
        self.defaultWidth = float(getattr(cfg.settings, "width", 0.745))
        self.tickMs = int(tickMs)
        # 集中服务的时间缩放因子：默认从配置读取，后续通过订阅 simConfig 动态更新
        self.simTimeScale: float = float(getattr(cfg.settings, "sim_time_scale", 1.0))
        self.client: mqtt.Client | None = None
        self.statesLock = threading.Lock()
        self.statesBySerial: Dict[str, Dict[str, Any]] = {}
        self._stop = False
        self._detectorThread: threading.Thread | None = None
        # 记录已下发 stopPause 的车辆对（用于解除重叠后恢复）
        self._stopped_pairs: set[frozenset[str]] = set()

    def start(self) -> None:
        import uuid
        self.client = mqtt.Client(client_id=str(uuid.uuid4()), protocol=mqtt.MQTTv5)
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.on_message = self._onMessage
        try:
            self.client.connect(self.host, self.port, keepalive=60)
            self.client.loop_start()
            # 订阅所有设备的 state：uagv/{vdaVersion}/{manufacturer}/{serial}/state
            # 正确的通配符为 3 个 '+': vdaVersion/manufacturer/serial
            self.client.subscribe(f"{self.vdaInterface}/+/+/+/state", qos=1)
            # 订阅仿真设置更新：用于动态更新集中服务的时间缩放因子
            self.client.subscribe(f"{self.vdaInterface}/+/+/+/simConfig", qos=1)
        except Exception as e:
            print(f"[WorldModel] MQTT connect/subscribe failed: {e}")
        self._stop = False
        self._detectorThread = threading.Thread(target=self._collisionLoop, daemon=True)
        self._detectorThread.start()

    def stop(self) -> None:
        self._stop = True
        try:
            if self._detectorThread and self._detectorThread.is_alive():
                self._detectorThread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
        except Exception:
            pass

    def _onMessage(self, client: mqtt.Client, userdata, msg: mqtt.MQTTMessage) -> None:
        tp = str(get_topic_type(msg.topic))
        if tp == "simConfig":
            # 动态接收仿真设置，提取并更新集中服务的时间缩放因子
            try:
                import json
                data = json.loads(msg.payload.decode("utf-8", errors="ignore"))
            except Exception:
                return
            try:
                v = data.get("sim_time_scale", data.get("simTimeScale"))
                if v is not None:
                    self.simTimeScale = float(v)
            except Exception:
                pass
            return
        if tp != "state":
            return
        try:
            import json
            data = json.loads(msg.payload.decode("utf-8", errors="ignore"))
        except Exception:
            return
        serial = str(data.get("serial_number", data.get("serialNumber", "")) or "")
        if not serial:
            return
        with self.statesLock:
            self.statesBySerial[serial] = data

    def _collisionLoop(self) -> None:
        while not self._stop:
            # 基于 sim_time_scale 的休眠时间缩放：scale 越大，检测越频繁
            eff_sleep_s = self.tickMs / 1000.0
            try:
                scale = max(0.0001, float(getattr(self, "simTimeScale", 1.0)))
                eff_sleep_s = (self.tickMs / scale) / 1000.0
            except Exception:
                pass
            try:
                snapshot: List[Tuple[str, Dict[str, Any]]] = []
                with self.statesLock:
                    snapshot = list(self.statesBySerial.items())
                n = len(snapshot)
                overlapped_pairs: set[frozenset[str]] = set()
                if n >= 2:
                    for i in range(n):
                        s1, p1 = snapshot[i]
                        r1 = computeSafetyRectForStatePayload(
                            p1,
                            length_default=self.defaultLength,
                            width_default=self.defaultWidth,
                            safeFactor=1.05,
                        )
                        if not r1:
                            continue
                        for j in range(i + 1, n):
                            s2, p2 = snapshot[j]
                            r2 = computeSafetyRectForStatePayload(
                                p2,
                                length_default=self.defaultLength,
                                width_default=self.defaultWidth,
                                safeFactor=1.05,
                            )
                            if not r2:
                                continue
                            if rects_overlap(r1, r2):
                                self._issueStopPauseForPair(p1, p2)
                                # 记录当前重叠车辆对
                                try:
                                    serial1 = str(p1.get("serial_number", p1.get("serialNumber", "")) or "")
                                    serial2 = str(p2.get("serial_number", p2.get("serialNumber", "")) or "")
                                    if serial1 and serial2:
                                        overlapped_pairs.add(frozenset([serial1, serial2]))
                                except Exception:
                                    pass
                # 对此前已停对但当前未重叠的车辆对，发布 startPause 恢复
                try:
                    to_resume_pairs = {pair for pair in self._stopped_pairs if pair not in overlapped_pairs}
                    for pair in to_resume_pairs:
                        for serial in pair:
                            try:
                                payload = self.statesBySerial.get(serial)
                                if payload:
                                    self._publishStartPause(payload)
                            except Exception:
                                pass
                        # 从集合移除已恢复的对
                        self._stopped_pairs.discard(pair)
                except Exception:
                    pass
                time.sleep(eff_sleep_s)
            except Exception:
                time.sleep(eff_sleep_s)

    def _issueStopPauseForPair(self, payload1: Dict[str, Any], payload2: Dict[str, Any]) -> None:
        try:
            self._publishStopPause(payload1)
        except Exception:
            pass
        try:
            self._publishStopPause(payload2)
        except Exception:
            pass
        # 记录已下发 stopPause 的车辆对
        try:
            s1 = str(payload1.get("serial_number", payload1.get("serialNumber", "")) or "")
            s2 = str(payload2.get("serial_number", payload2.get("serialNumber", "")) or "")
            if s1 and s2:
                self._stopped_pairs.add(frozenset([s1, s2]))
        except Exception:
            pass

    def _publishStopPause(self, statePayload: Dict[str, Any]) -> None:
        if not self.client:
            return
        vda_version = str(statePayload.get("version", "v2") or "v2")
        manufacturer = str(statePayload.get("manufacturer", "SEER") or "SEER")
        serial = str(statePayload.get("serial_number", statePayload.get("serialNumber", "")) or "")
        if not serial:
            return
        base = generate_vda_mqtt_base_topic(self.vdaInterface, vda_version, manufacturer, serial)
        topic = f"{base}/instantActions"
        ts = get_timestamp()
        payload = {
            "header_id": 0,
            "timestamp": ts,
            "version": vda_version,
            "manufacturer": manufacturer,
            "serial_number": serial,
            "actions": [
                {
                    "action_type": "stopPause",
                    "action_id": f"COLLISION_STOP_{ts}",
                    "blocking_type": "SOFT",
                    "action_parameters": [],
                }
            ],
        }
        publish_json(self.client, topic, payload, qos=1, retain=False)

    def _publishStartPause(self, statePayload: Dict[str, Any]) -> None:
        if not self.client:
            return
        vda_version = str(statePayload.get("version", "v2") or "v2")
        manufacturer = str(statePayload.get("manufacturer", "SEER") or "SEER")
        serial = str(statePayload.get("serial_number", statePayload.get("serialNumber", "")) or "")
        if not serial:
            return
        base = generate_vda_mqtt_base_topic(self.vdaInterface, vda_version, manufacturer, serial)
        topic = f"{base}/instantActions"
        ts = get_timestamp()
        payload = {
            "header_id": 0,
            "timestamp": ts,
            "version": vda_version,
            "manufacturer": manufacturer,
            "serial_number": serial,
            "actions": [
                {
                    "action_type": "startPause",
                    "action_id": f"COLLISION_START_{ts}",
                    "blocking_type": "SOFT",
                    "action_parameters": [],
                }
            ],
        }
        publish_json(self.client, topic, payload, qos=1, retain=False)