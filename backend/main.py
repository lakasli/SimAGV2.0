from __future__ import annotations
from pathlib import Path
from typing import List

from fastapi import FastAPI, HTTPException, Body
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from typing import Union
from .schemas import AGVRegistration, RegisterRequest, RegisterResponse, UnregisterResponse, AGVInfo, StaticConfigPatch, DynamicConfigUpdate, StatusResponse
from .schemas import SimSettingsPatch
from .schemas import TranslateRequest, RotateRequest
from .schemas import SwitchMapRequest
from .agv_manager import AGVManager
from .sim_process_manager import SimulatorProcessManager
from fastapi import WebSocket
import asyncio
from .websocket_manager import WebSocketManager
from SimVehicleSys.sim_vehicle.action_executor import execute_translation_movement, execute_rotation_movement
from SimVehicleSys.config.settings import get_config
from .mqtt_bridge import MQTTBridge
from .mqtt_publisher import MqttPublisher
import json
from SimVehicleSys.protocol.vda_2_0_0.order import Order as VDAOrder
from .agv_config_store import AgvConfigStore
import shutil

project_root = Path(__file__).resolve().parents[1]
app = FastAPI(title="SimAGV Backend", version="0.1.0")
try:
    from SimVehicleSys.api.routes import router as sim_router
    app.include_router(sim_router)
except Exception:
    pass

# CORS for local dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Managers
agv_storage = project_root / "backend" / "data" / "registered_agvs.json"
agv_manager = AGVManager(agv_storage)
ws_manager = WebSocketManager()
sim_manager = SimulatorProcessManager(project_root)
agv_config_dir = project_root / "SimVehicleSys" / "agv_configs"
agv_config_store = AgvConfigStore(agv_config_dir)
try:
    old_agv_config_dir = project_root / "backend" / "data" / "agv_configs"
    if old_agv_config_dir.exists():
        for p in old_agv_config_dir.iterdir():
            if p.is_file() and p.suffix.lower() == ".json":
                dst = agv_config_dir / p.name
                if not dst.exists():
                    shutil.copy2(p, dst)
except Exception:
    pass

# Static mounts: frontend and maps
frontend_dir = project_root / "frontend"
maps_dir = project_root / "maps"
shelf_dir = project_root / "SimVehicleSys" / "shelf"
if frontend_dir.exists():
    app.mount("/static", StaticFiles(directory=str(frontend_dir)), name="static")
if maps_dir.exists():
    app.mount("/maps", StaticFiles(directory=str(maps_dir)), name="maps")
if shelf_dir.exists():
    # 公开托盘/货架模型静态文件
    app.mount("/shelf", StaticFiles(directory=str(shelf_dir)), name="shelf")

@app.get("/api/equipments")
def list_equipments() -> List[dict]:
    equipments_dir = project_root / "SimEquipment"
    items: List[dict] = []
    try:
        if equipments_dir.exists():
            for p in equipments_dir.iterdir():
                try:
                    if not p.is_dir():
                        continue
                    cfg_fp = p / "config.json"
                    if not cfg_fp.exists():
                        continue
                    data = json.loads(cfg_fp.read_text(encoding="utf-8")) or {}
                    mqtt = data.get("mqtt", {}) or {}
                    t = data.get("type")
                    if not t:
                        nm = p.name.lower()
                        if "door" in nm:
                            t = "door"
                        elif "caller" in nm:
                            t = "caller"
                        elif "elevator" in nm:
                            t = "elevator"
                        elif "light" in nm:
                            t = "light"
                        else:
                            t = "device"
                    site_raw = data.get("site")
                    if isinstance(site_raw, list):
                        site_list = [str(x) for x in site_raw if x is not None]
                    elif isinstance(site_raw, str):
                        site_list = [site_raw]
                    else:
                        site_list = []
                    if str(t).lower() != "door" and len(site_list) > 1:
                        site_list = site_list[:1]
                    items.append({
                        "name": data.get("name"),
                        "type": t,
                        "manufacturer": data.get("manufacturer"),
                        "serial_number": data.get("serial_number"),
                        "vda_version": data.get("vda_version"),
                        "vda_full_version": data.get("vda_full_version"),
                        "ip": data.get("ip"),
                        "site": site_list,
                        "map_id": data.get("map_id"),
                        "state_frequency": data.get("state_frequency"),
                        "action_time": data.get("action_time"),
                        "trigger_mode": data.get("trigger_mode"),
                        "mqtt": {
                            "host": mqtt.get("host"),
                            "port": mqtt.get("port"),
                            "vda_interface": mqtt.get("vda_interface"),
                        },
                        "dir_name": p.name,
                    })
                except Exception:
                    pass
    except Exception:
        items = []
    return items

@app.get("/api/maps")
def list_maps() -> List[str]:
    vehicle_dir = maps_dir / "VehicleMap"
    if not vehicle_dir.exists():
        return []
    files: List[str] = []
    for p in vehicle_dir.iterdir():
        if p.is_file() and p.suffix.lower() == ".scene":
            files.append(f"VehicleMap/{p.name}")
    return files

@app.get("/api/maps/{kind}")
def list_maps_by_kind(kind: str) -> List[str]:
    # 仅支持列出 VehicleMap 或 ViewerMap 下的 .scene 文件
    if kind not in {"VehicleMap", "ViewerMap"}:
        raise HTTPException(status_code=404, detail=f"Map kind '{kind}' not found")
    target = maps_dir / kind
    if not target.exists() or not target.is_dir():
        raise HTTPException(status_code=404, detail=f"Map kind '{kind}' not found")
    files: List[str] = []
    for p in target.iterdir():
        if p.is_file() and p.suffix.lower() == ".scene":
            files.append(f"{kind}/{p.name}")
    return files

@app.get("/")
def index():
    index_file = frontend_dir / "index.html"
    if not index_file.exists():
        raise HTTPException(status_code=404, detail="Frontend not found")
    return FileResponse(index_file)

# 前端配置：提供轮询间隔（毫秒）与中心前移偏移量（米）
@app.get("/api/config")
def get_frontend_config():
    try:
        # 优先返回运行态覆盖值，其次回退到默认配置
        runtime_val = getattr(app.state, "frontend_poll_interval_ms", None)
        if runtime_val is not None:
            interval = int(runtime_val)
        else:
            cfg = get_config()
            interval = int(getattr(cfg.settings, "frontend_poll_interval_ms", 100))
        # 安全边界：至少 10ms，至多 5000ms
        interval = max(10, min(5000, interval))
        # 中心前移偏移量（米）：用于安全范围几何中心沿车头方向平移
        try:
            offset_rt = getattr(app.state, "center_forward_offset_m", None)
        except Exception:
            offset_rt = None
        if offset_rt is not None:
            offset = float(offset_rt)
        else:
            try:
                cfg = get_config()
                offset = float(getattr(cfg.settings, "center_forward_offset_m", 0.1))
            except Exception:
                offset = 0.1
        # 合理边界：0~5 米
        offset = max(0.0, min(5.0, offset))
        return {"polling_interval_ms": interval, "center_forward_offset_m": offset}
    except Exception:
        # 失败时回退到 100ms
        return {"polling_interval_ms": 100, "center_forward_offset_m": 0.1}

# --- Navigation APIs ---

# 已迁移：站点路径导航改由仿真车内置执行（如需保留，请改为发布 MQTT 指令）

# 旧的导航接口已移除；请改用 /api/sim 路由下的统一接口。

# 路径查询：后端不再维护，若需要可改为读取仿真车发布的可视化数据。

# --- AGV Registry APIs ---
@app.get("/api/agvs", response_model=List[AGVInfo])
def list_agvs():
    agvs = agv_manager.list_agvs()
    # 打印所有已注册机器人实例的 MQTT state 订阅地址（VDA5050 格式）
    try:
        for info in agvs:
            topic = f"uagv/{info.vda_version}/{info.manufacturer}/{info.serial_number}/state"
            print(f"[VDA5050] ID={info.serial_number} 订阅地址: {topic}")
    except Exception as e:
        print(f"[VDA5050] 订阅地址打印失败: {e}")
    return agvs

@app.post("/api/agvs/register", response_model=RegisterResponse)
def register_agvs(body: Union[RegisterRequest, List[AGVRegistration]]):
    regs = body.agvs if isinstance(body, RegisterRequest) else body
    registered, skipped = agv_manager.register_many(regs)
    # 为新注册的实例创建默认配置文件
    try:
        for sn in registered:
            agv_config_store.ensure_default_for(sn)
    except Exception as e:
        print(f"[ConfigStore] ensure default failed: {e}")
    try:
        sim_manager.ensure_running_for_serials(registered)
    except Exception as e:
        print(f"[SimProc] auto-start failed: {e}")
    # 若存在历史配置，注册后立即推送仿真设置并恢复位置
    try:
        publisher = getattr(app.state, "mqtt_publisher", None)
        for sn in registered:
            info = agv_manager.get_agv(sn)
            if not info:
                continue
            cfg = agv_config_store.load(sn)
            if not cfg:
                continue
            patch_dict = {
                k: cfg.get(k)
                for k in (
                    "speed_min",
                    "speed_max",
                    "acceleration_max",
                    "deceleration_max",
                    "height_min",
                    "height_max",
                    "width",
                    "length",
                    "sim_time_scale",
                    "state_frequency",
                    "visualization_frequency",
                    "action_time",
                )
                if cfg.get(k) is not None
            }
            if patch_dict and publisher:
                try:
                    publisher.publish_sim_settings(info, patch_dict)
                except Exception:
                    pass
            # 恢复地图与位置
            try:
                from .schemas import DynamicConfigUpdate, Position
                pos = cfg.get("last_position") or {}
                map_id = cfg.get("map_id")
                dyn = None
                if map_id or pos:
                    dyn = DynamicConfigUpdate(
                        current_map=str(map_id) if map_id is not None else None,
                        position=Position(
                            x=float(pos.get("x", 0.0)),
                            y=float(pos.get("y", 0.0)),
                            theta=pos.get("theta"),
                        ) if pos else None,
                    )
                if dyn:
                    rt = agv_manager.update_dynamic(sn, dyn)
                    if publisher and rt:
                        try:
                            publisher.publish_init_position(info, rt)
                        except Exception:
                            pass
            except Exception:
                pass
    except Exception as e:
        print(f"[ConfigStore] post-register restore failed: {e}")
    return RegisterResponse(registered=registered, skipped=skipped)

@app.delete("/api/agvs/{serial_number}", response_model=UnregisterResponse)
def unregister_agv(serial_number: str):
    ok = agv_manager.unregister(serial_number)
    if not ok:
        raise HTTPException(status_code=404, detail="AGV not found")
    # 在注销前保存一次离线快照（若可获取到状态）
    try:
        status = agv_manager.get_status(serial_number)
        if status:
            from .schemas import AGVRuntime, Position
            rt = AGVRuntime(
                battery_level=status.battery_level,
                current_map=status.current_map,
                position=Position(x=status.position.x, y=status.position.y, theta=status.position.theta),
            )
            agv_config_store.update_runtime_snapshot(serial_number, rt)
    except Exception:
        pass
    try:
        sim_manager.stop_for(serial_number)
    except Exception as e:
        print(f"[SimProc] stop failed: {e}")
    # 删除对应配置文件
    try:
        agv_config_store.delete(serial_number)
    except Exception as e:
        print(f"[ConfigStore] delete failed for {serial_number}: {e}")
    return UnregisterResponse(removed=True)

@app.patch("/api/agv/{serial_number}/config/static", response_model=AGVInfo)
def patch_static_config(serial_number: str, body: StaticConfigPatch):
    updated = agv_manager.update_static(serial_number, body)
    if not updated:
        raise HTTPException(status_code=404, detail="AGV not found")
    return updated

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws_manager.connect(ws)
    try:
        while True:
            await asyncio.sleep(3600)
    except Exception:
        pass
    finally:
        ws_manager.disconnect(ws)

@app.on_event("startup")
async def _start_ws_broadcast():
    # Start MQTT bridge (sink simulator state into AGVManager)
    loop_obj = asyncio.get_running_loop()
    bridge = MQTTBridge(project_root / "config.toml", ws_manager.broadcast_json, loop_obj, agv_manager)
    bridge.start()
    app.state.mqtt_bridge = bridge
    # Initialize MQTT publisher
    publisher = MqttPublisher(project_root / "config.toml")
    publisher.start()
    app.state.mqtt_publisher = publisher
    # 初始化前端轮询间隔覆盖值（用于热更新）
    try:
        cfg = get_config()
        app.state.frontend_poll_interval_ms = int(getattr(cfg.settings, "frontend_poll_interval_ms", 1000))
        # 初始化中心前移偏移量运行态值
        try:
            app.state.center_forward_offset_m = float(getattr(cfg.settings, "center_forward_offset_m", 0.2))
        except Exception:
            app.state.center_forward_offset_m = 0.2
    except Exception:
        app.state.frontend_poll_interval_ms = 1000
    # 记录每个 AGV 最近一次提交的仿真设置（用于前端预填）
    try:
        app.state.sim_settings_by_agv = {}
    except Exception:
        pass
    # 仿真车导航由其自身进程负责，后端仅保留 MQTT 桥与发布器
    # 进程管理器：在服务启动时为所有已注册 AGV 确保仿真实例在运行
    try:
        app.state.sim_manager = sim_manager
        serials = [info.serial_number for info in agv_manager.list_agvs()]
        started = sim_manager.ensure_running_for_serials(serials)
        if started:
            print(f"[SimProc] started on startup: {started}")
    except Exception as e:
        print(f"[SimProc] startup ensure failed: {e}")
    # 启动后加载每个实例的持久化配置，恢复仿真设置与离线快照
    try:
        publisher = getattr(app.state, "mqtt_publisher", None)
        overrides = getattr(app.state, "sim_settings_by_agv", {})
        for info in agv_manager.list_agvs():
            cfg = agv_config_store.load(info.serial_number)
            if not cfg:
                continue
            # 发布仿真设置（物理参数与关键运行参数）到设备端
            patch_dict = {
                k: cfg.get(k)
                for k in (
                    "speed_min",
                    "speed_max",
                    "acceleration_max",
                    "deceleration_max",
                    "height_min",
                    "height_max",
                    "width",
                    "length",
                    "map_id",
                    "sim_time_scale",
                    "state_frequency",
                    "visualization_frequency",
                    "action_time",
                )
                if cfg.get(k) is not None
            }
            if patch_dict and publisher:
                try:
                    publisher.publish_sim_settings(info, patch_dict)
                except Exception as e:
                    print(f"[ConfigStore] publish sim settings failed: {e}")
            try:
                prev = overrides.get(info.serial_number, {}) if isinstance(overrides, dict) else {}
                prev.update(patch_dict)
                overrides[info.serial_number] = prev  # type: ignore[index]
                app.state.sim_settings_by_agv = overrides
            except Exception:
                pass
            # 恢复地图与位置（若存在离线快照）
            try:
                from .schemas import DynamicConfigUpdate, Position
                pos = cfg.get("last_position") or {}
                map_id = cfg.get("map_id")
                dyn = None
                if map_id or pos:
                    try:
                        dyn = DynamicConfigUpdate(
                            current_map=str(map_id) if map_id is not None else None,
                            position=Position(
                                x=float(pos.get("x", 0.0)),
                                y=float(pos.get("y", 0.0)),
                                theta=pos.get("theta"),
                            ) if pos else None,
                        )
                    except Exception:
                        dyn = None
                if dyn:
                    rt = agv_manager.update_dynamic(info.serial_number, dyn)
                    if publisher and rt:
                        try:
                            publisher.publish_init_position(info, rt)
                        except Exception as e:
                            print(f"[ConfigStore] publish initPosition failed: {e}")
            except Exception as e:
                print(f"[ConfigStore] restore runtime failed: {e}")
    except Exception as e:
        print(f"[ConfigStore] startup restore failed: {e}")

@app.on_event("shutdown")
async def _stop_ws_broadcast():
    task = getattr(app.state, "ws_broadcast_task", None)
    if task:
        task.cancel()
    bridge = getattr(app.state, "mqtt_bridge", None)
    if bridge:
        bridge.stop()
    try:
        stopped = sim_manager.stop_all()
        if stopped:
            print(f"[SimProc] stopped on shutdown: {stopped}")
    except Exception:
        pass
    # 保存离线快照与最近仿真设置到配置文件
    try:
        overrides = getattr(app.state, "sim_settings_by_agv", {})
        for info in agv_manager.list_agvs():
            try:
                # 使用集中式运行态存储的值保存快照
                from SimVehicleSys.sim_vehicle.state_manager import global_store
                rt = global_store.get_runtime(info.serial_number)
                agv_config_store.update_runtime_snapshot(info.serial_number, rt)
            except Exception:
                # 回退到状态响应
                try:
                    status = agv_manager.get_status(info.serial_number)
                    if status:
                        from .schemas import AGVRuntime, Position
                        rt = AGVRuntime(
                            battery_level=status.battery_level,
                            current_map=status.current_map,
                            position=Position(x=status.position.x, y=status.position.y, theta=status.position.theta),
                            speed_limit=1.0,
                        )
                        agv_config_store.update_runtime_snapshot(info.serial_number, rt)
                except Exception:
                    pass
            # 保存最近一次提交的仿真设置覆盖
            try:
                patch = overrides.get(info.serial_number, {}) if isinstance(overrides, dict) else {}
                if patch:
                    agv_config_store.update_physical(info.serial_number, patch)
            except Exception:
                pass
    except Exception as e:
        print(f"[ConfigStore] shutdown persist failed: {e}")

@app.post("/api/agv/{serial_number}/config/dynamic", response_model=StatusResponse)
async def post_dynamic_config(serial_number: str, body: DynamicConfigUpdate):
    rt = agv_manager.update_dynamic(serial_number, body)
    if not rt:
        raise HTTPException(status_code=404, detail="AGV not found")
    status = agv_manager.get_status(serial_number)
    if status:
        # Publish initPosition to MQTT
        try:
            publisher = getattr(app.state, "mqtt_publisher", None)
            info = agv_manager.get_agv(serial_number)
            if publisher and info:
                publisher.publish_init_position(info, rt)
        except Exception as e:
            print(f"Publish initPosition failed: {e}")
    return status

@app.get("/api/agv/{serial_number}/status", response_model=StatusResponse)
def get_agv_status(serial_number: str):
    status = agv_manager.get_status(serial_number)
    if not status:
        raise HTTPException(status_code=404, detail="AGV not found")
    return status

@app.post("/api/agv/{serial_number}/move/translate", response_model=StatusResponse)
async def move_translate(serial_number: str, body: TranslateRequest):
    rt = await execute_translation_movement(
        serial_number,
        body.dx,
        body.dy,
        agv_manager,
        ws_manager,
        getattr(body, "movement_state", None),
    )
    if not rt:
        raise HTTPException(status_code=404, detail="AGV not found")
    # Publish initPosition to MQTT
    try:
        publisher = getattr(app.state, "mqtt_publisher", None)
        info = agv_manager.get_agv(serial_number)
        if publisher and info:
            publisher.publish_init_position(info, rt)
    except Exception as e:
        print(f"Publish initPosition failed: {e}")
    status = agv_manager.get_status(serial_number)
    return status

# 热更新仿真设置：发布到 MQTT simConfig，部分更新
@app.post("/api/agv/{serial_number}/sim/settings")
def post_sim_settings(serial_number: str, body: SimSettingsPatch):
    info = agv_manager.get_agv(serial_number)
    if not info:
        raise HTTPException(status_code=404, detail="AGV not found")
    publisher = getattr(app.state, "mqtt_publisher", None)
    if not publisher:
        raise HTTPException(status_code=500, detail="MQTT publisher not available")
    # 参数范围校验
    errors: list[str] = []
    def add_err(msg: str):
        errors.append(msg)
    if body.speed is not None and not (0.0 <= float(body.speed) <= 2.0):
        add_err("速度(speed)必须在[0,2]范围内")
    if body.sim_time_scale is not None:
        v = float(body.sim_time_scale)
        if not (v > 0.0 and v <= 10.0):
            add_err("时间缩放(sim_time_scale)必须在(0,10]范围内")
    if body.state_frequency is not None:
        v = int(body.state_frequency)
        if not (v >= 1 and v <= 10):
            add_err("状态频率(state_frequency)必须为[1,10]的正整数")
    if body.visualization_frequency is not None:
        v = int(body.visualization_frequency)
        if not (v >= 1 and v <= 10):
            add_err("可视化频率(visualization_frequency)必须为[1,10]的正整数")
    if body.action_time is not None:
        v = float(body.action_time)
        if not (v >= 1.0 and v <= 10.0):
            add_err("动作时长(action_time)必须在[1,10]范围内")
    if body.frontend_poll_interval_ms is not None:
        v = int(body.frontend_poll_interval_ms)
        if not (v >= 10 and v <= 1000):
            add_err("前端轮询(frontend_poll_interval_ms)必须为[10,1000]的正整数")
    if body.battery_default is not None:
        v = float(body.battery_default)
        if not (v > 0.0 and v <= 100.0):
            add_err("默认电量(battery_default)必须在(0,100]范围内")
    if body.battery_idle_drain_per_min is not None:
        v = float(body.battery_idle_drain_per_min)
        if not (v >= 1.0 and v <= 100.0):
            add_err("空闲耗电(battery_idle_drain_per_min)必须在[1,100]范围内")
    if body.battery_move_empty_multiplier is not None:
        v = float(body.battery_move_empty_multiplier)
        if not (v >= 1.0 and v <= 100.0):
            add_err("空载耗电系数(battery_move_empty_multiplier)必须在[1,100]范围内")
    if body.battery_move_loaded_multiplier is not None:
        v = float(body.battery_move_loaded_multiplier)
        if not (v >= 1.0 and v <= 100.0):
            add_err("载重耗电系数(battery_move_loaded_multiplier)必须在[1,100]范围内")
    if body.battery_charge_per_min is not None:
        v = float(body.battery_charge_per_min)
        if not (v >= 1.0 and v <= 100.0):
            add_err("充电速度(battery_charge_per_min)必须在[1,100]范围内")
    if errors:
        raise HTTPException(status_code=400, detail="; ".join(errors))
    try:
        # 先更新后端运行态的前端轮询覆盖值（若提交了该项）
        if body.frontend_poll_interval_ms is not None:
            try:
                app.state.frontend_poll_interval_ms = int(body.frontend_poll_interval_ms)
            except Exception:
                pass
        # 记录最近一次设置（用于前端预填）
        try:
            current = getattr(app.state, "sim_settings_by_agv", None)
            if current is None:
                app.state.sim_settings_by_agv = {}
                current = app.state.sim_settings_by_agv
            update = {k: v for k, v in body.model_dump().items() if v is not None}
            prev = current.get(serial_number, {})
            prev.update(update)
            current[serial_number] = prev
        except Exception:
            pass
        publisher.publish_sim_settings(info, body)
        # 持久化本次提交的仿真设置覆盖值
        try:
            agv_config_store.update_physical(serial_number, body.model_dump(exclude_none=True))
        except Exception:
            pass
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Publish sim settings failed: {e}")
    return {"updated": True, "serial_number": serial_number}

# 获取当前仿真设置：合并默认配置与最近一次提交的覆盖值
@app.get("/api/agv/{serial_number}/sim/settings")
def get_sim_settings(serial_number: str):
    info = agv_manager.get_agv(serial_number)
    if not info:
        raise HTTPException(status_code=404, detail="AGV not found")
    try:
        cfg = get_config()
    except Exception:
        cfg = None  # type: ignore
    base = getattr(cfg, "settings", None)
    # 运行态覆盖
    overrides = getattr(app.state, "sim_settings_by_agv", {})
    ov = overrides.get(serial_number, {}) if isinstance(overrides, dict) else {}
    def pick(name: str, default):
        return ov.get(name, default)
    # 前端轮询间隔从运行态覆盖值读取
    fp_default = getattr(base, "frontend_poll_interval_ms", 1000) if base else 1000
    fp_runtime = getattr(app.state, "frontend_poll_interval_ms", fp_default)
    result = {
        "action_time": pick("action_time", getattr(base, "action_time", 1.0) if base else 1.0),
        "speed": pick("speed", getattr(base, "speed", 1.0) if base else 1.0),
        # 物理参数（用于前端物理参数弹窗预填与仿真约束）
        "speed_min": pick("speed_min", getattr(base, "speed_min", 0.01) if base else 0.01),
        "speed_max": pick("speed_max", getattr(base, "speed_max", 2.0) if base else 2.0),
        "acceleration_max": pick("acceleration_max", getattr(base, "acceleration_max",2) if base else 2),
        "deceleration_max": pick("deceleration_max", getattr(base, "deceleration_max", 2) if base else 2),
        "height_min": pick("height_min", getattr(base, "height_min", 0.01) if base else 0.01),
        "height_max": pick("height_max", getattr(base, "height_max", 0.10) if base else 0.10),
        "width": pick("width", getattr(base, "width", 0.745) if base else 0.745),
        "length": pick("length", getattr(base, "length", 1.03) if base else 1.03),
        "state_frequency": pick("state_frequency", getattr(base, "state_frequency", 10) if base else 10),
        "visualization_frequency": pick("visualization_frequency", getattr(base, "visualization_frequency", 1) if base else 1),
        "map_id": pick("map_id", getattr(base, "map_id", "default") if base else "default"),
        "sim_time_scale": pick("sim_time_scale", getattr(base, "sim_time_scale", 1.0) if base else 1.0),
        "battery_default": pick("battery_default", getattr(base, "battery_default", 100.0) if base else 100.0),
        "battery_idle_drain_per_min": pick("battery_idle_drain_per_min", getattr(base, "battery_idle_drain_per_min", 1.0) if base else 1.0),
        "battery_move_empty_multiplier": pick("battery_move_empty_multiplier", getattr(base, "battery_move_empty_multiplier", 1.5) if base else 1.5),
        "battery_move_loaded_multiplier": pick("battery_move_loaded_multiplier", getattr(base, "battery_move_loaded_multiplier", 2.5) if base else 2.5),
        "battery_charge_per_min": pick("battery_charge_per_min", getattr(base, "battery_charge_per_min", 10.0) if base else 10.0),
        "frontend_poll_interval_ms": int(fp_runtime),
    }
    return result

# 全局仿真设置（热加载）：对所有已注册 AGV 生效
@app.post("/api/sim/settings")
def post_sim_settings_global(body: SimSettingsPatch):
    publisher = getattr(app.state, "mqtt_publisher", None)
    if not publisher:
        raise HTTPException(status_code=500, detail="MQTT publisher not available")
    # 参数范围校验（与按实例接口一致）
    errors: list[str] = []
    def add_err(msg: str):
        errors.append(msg)
    if body.speed is not None and not (0.0 <= float(body.speed) <= 2.0):
        add_err("速度(speed)必须在[0,2]范围内")
    if body.sim_time_scale is not None:
        v = float(body.sim_time_scale)
        if not (v > 0.0 and v <= 10.0):
            add_err("时间缩放(sim_time_scale)必须在(0,10]范围内")
    if body.state_frequency is not None:
        v = int(body.state_frequency)
        if not (v >= 1 and v <= 10):
            add_err("状态频率(state_frequency)必须为[1,10]的正整数")
    if body.visualization_frequency is not None:
        v = int(body.visualization_frequency)
        if not (v >= 1 and v <= 10):
            add_err("可视化频率(visualization_frequency)必须为[1,10]的正整数")
    if body.action_time is not None:
        v = float(body.action_time)
        if not (v >= 1.0 and v <= 10.0):
            add_err("动作时长(action_time)必须在[1,10]范围内")
    if body.frontend_poll_interval_ms is not None:
        v = int(body.frontend_poll_interval_ms)
        if not (v >= 10 and v <= 1000):
            add_err("前端轮询(frontend_poll_interval_ms)必须为[10,1000]的正整数")
    if body.battery_default is not None:
        v = float(body.battery_default)
        if not (v > 0.0 and v <= 100.0):
            add_err("默认电量(battery_default)必须在(0,100]范围内")
    if body.battery_idle_drain_per_min is not None:
        v = float(body.battery_idle_drain_per_min)
        if not (v >= 1.0 and v <= 100.0):
            add_err("空闲耗电(battery_idle_drain_per_min)必须在[1,100]范围内")
    if body.battery_move_empty_multiplier is not None:
        v = float(body.battery_move_empty_multiplier)
        if not (v >= 1.0 and v <= 100.0):
            add_err("空载耗电系数(battery_move_empty_multiplier)必须在[1,100]范围内")
    if body.battery_move_loaded_multiplier is not None:
        v = float(body.battery_move_loaded_multiplier)
        if not (v >= 1.0 and v <= 100.0):
            add_err("载重耗电系数(battery_move_loaded_multiplier)必须在[1,100]范围内")
    if body.battery_charge_per_min is not None:
        v = float(body.battery_charge_per_min)
        if not (v >= 1.0 and v <= 100.0):
            add_err("充电速度(battery_charge_per_min)必须在[1,100]范围内")
    if errors:
        raise HTTPException(status_code=400, detail="; ".join(errors))
    try:
        # 更新运行态的前端轮询覆盖值（若提交了该项）
        if body.frontend_poll_interval_ms is not None:
            try:
                app.state.frontend_poll_interval_ms = int(body.frontend_poll_interval_ms)
            except Exception:
                pass
        # 记录最近一次的全局设置覆盖值（用于前端预填）
        try:
            update = {k: v for k, v in body.model_dump().items() if v is not None}
            app.state.sim_settings_global = update
        except Exception:
            pass
        # 对所有已注册 AGV 发布 simConfig 并持久化到各自配置文件
        infos = agv_manager.list_agvs()
        for info in infos:
            try:
                publisher.publish_sim_settings(info, body)
            except Exception as e:
                # 任一发布失败不阻断其它实例
                print(f"[SimConfig] publish failed for {info.serial_number}: {e}")
            try:
                agv_config_store.update_physical(info.serial_number, body.model_dump(exclude_none=True))
            except Exception:
                pass
        # 同步运行态覆盖映射，保证按实例查询也能看到最新值
        try:
            current = getattr(app.state, "sim_settings_by_agv", None)
            if current is None:
                app.state.sim_settings_by_agv = {}
                current = app.state.sim_settings_by_agv
            for info in infos:
                prev = current.get(info.serial_number, {})
                prev.update({k: v for k, v in body.model_dump().items() if v is not None})
                current[info.serial_number] = prev
        except Exception:
            pass
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Publish global sim settings failed: {e}")
    return {"updated": True, "scope": "global", "count": len(infos)}

# 获取全局仿真设置：合并默认配置与最近一次提交的全局覆盖值
@app.get("/api/sim/settings")
def get_sim_settings_global():
    try:
        cfg = get_config()
    except Exception:
        cfg = None  # type: ignore
    base = getattr(cfg, "settings", None)
    ov = getattr(app.state, "sim_settings_global", {})
    def pick(name: str, default):
        return ov.get(name, default)
    # 前端轮询间隔从运行态覆盖值读取
    fp_default = getattr(base, "frontend_poll_interval_ms", 1000) if base else 1000
    fp_runtime = getattr(app.state, "frontend_poll_interval_ms", fp_default)
    result = {
        "action_time": pick("action_time", getattr(base, "action_time", 1.0) if base else 1.0),
        "speed": pick("speed", getattr(base, "speed", 1.0) if base else 1.0),
        # 物理参数（用于前端物理参数弹窗预填与仿真约束）
        "speed_min": pick("speed_min", getattr(base, "speed_min", 0.01) if base else 0.01),
        "speed_max": pick("speed_max", getattr(base, "speed_max", 2.0) if base else 2.0),
        "acceleration_max": pick("acceleration_max", getattr(base, "acceleration_max", 2) if base else 2),
        "deceleration_max": pick("deceleration_max", getattr(base, "deceleration_max", 2) if base else 2),
        "height_min": pick("height_min", getattr(base, "height_min", 0.01) if base else 0.01),
        "height_max": pick("height_max", getattr(base, "height_max", 0.10) if base else 0.10),
        "width": pick("width", getattr(base, "width", 0.745) if base else 0.745),
        "length": pick("length", getattr(base, "length", 1.03) if base else 1.03),
        "state_frequency": pick("state_frequency", getattr(base, "state_frequency", 10) if base else 10),
        "visualization_frequency": pick("visualization_frequency", getattr(base, "visualization_frequency", 1) if base else 1),
        "map_id": pick("map_id", getattr(base, "map_id", "default") if base else "default"),
        "sim_time_scale": pick("sim_time_scale", getattr(base, "sim_time_scale", 1.0) if base else 1.0),
        "battery_default": pick("battery_default", getattr(base, "battery_default", 100.0) if base else 100.0),
        "battery_idle_drain_per_min": pick("battery_idle_drain_per_min", getattr(base, "battery_idle_drain_per_min", 1.0) if base else 1.0),
        "battery_move_empty_multiplier": pick("battery_move_empty_multiplier", getattr(base, "battery_move_empty_multiplier", 1.5) if base else 1.5),
        "battery_move_loaded_multiplier": pick("battery_move_loaded_multiplier", getattr(base, "battery_move_loaded_multiplier", 2.5) if base else 2.5),
        "battery_charge_per_min": pick("battery_charge_per_min", getattr(base, "battery_charge_per_min", 10.0) if base else 10.0),
        "frontend_poll_interval_ms": int(fp_runtime),
    }
    return result

@app.post("/api/agv/{serial_number}/move/rotate", response_model=StatusResponse)
async def move_rotate(serial_number: str, body: RotateRequest):
    rt = await execute_rotation_movement(serial_number, body.dtheta, agv_manager, ws_manager)
    if not rt:
        raise HTTPException(status_code=404, detail="AGV not found")
    # Publish initPosition to MQTT
    try:
        publisher = getattr(app.state, "mqtt_publisher", None)
        info = agv_manager.get_agv(serial_number)
        if publisher and info:
            publisher.publish_init_position(info, rt)
    except Exception as e:
        print(f"Publish initPosition failed: {e}")
    status = agv_manager.get_status(serial_number)
    return status

def _convert_order_camel_to_snake(d: dict) -> dict:
    """Convert a VDA5050 order payload from camelCase to snake_case for dataclass validation.

    Keeps unknown keys as-is; recursively converts nodes, edges, actions, trajectory, and nodePosition.
    """
    def map_keys(obj):
        if isinstance(obj, list):
            return [map_keys(x) for x in obj]
        if not isinstance(obj, dict):
            return obj
        m = {}
        # top-level/order-level mappings
        key_map = {
            "headerId": "header_id",
            "serialNumber": "serial_number",
            "orderId": "order_id",
            "orderUpdateId": "order_update_id",
            "zoneSetId": "zone_set_id",
        }
        # node-level
        node_map = {
            "nodeId": "node_id",
            "sequenceId": "sequence_id",
            "nodeDescription": "node_description",
            "nodePosition": "node_position",
        }
        # nodePosition-level
        np_map = {
            "mapId": "map_id",
            "mapDescription": "map_description",
            "allowedDeviationXY": "allowed_deviation_xy",
            "allowedDeviationTheta": "allowed_deviation_theta",
        }
        # edge-level
        edge_map = {
            "edgeId": "edge_id",
            "sequenceId": "sequence_id",
            "startNodeId": "start_node_id",
            "endNodeId": "end_node_id",
            "edgeDescription": "edge_description",
            "maxSpeed": "max_speed",
            "maxHeight": "max_height",
            "minHeight": "min_height",
            "orientationType": "orientation_type",
            "rotationAllowed": "rotation_allowed",
            "maxRotationSpeed": "max_rotation_speed",
            "length": "length",
            "direction": "direction",
            "orientation": "orientation",
            "trajectory": "trajectory",
        }
        # trajectory-level
        traj_map = {
            "knotVector": "knot_vector",
            "controlPoints": "control_points",
        }
        # action-level
        act_map = {
            "actionType": "action_type",
            "actionId": "action_id",
            "blockingType": "blocking_type",
            "actionDescription": "action_description",
            "actionParameters": "action_parameters",
        }
        # decide which map to use per object
        # heuristics: look for sentinel keys
        if "nodes" in obj or "edges" in obj:
            # order-level
            for k, v in obj.items():
                nk = key_map.get(k, k)
                if nk in ("nodes", "edges"):
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        # detect node
        if any(k in obj for k in ("nodeId", "node_id")):
            for k, v in obj.items():
                nk = node_map.get(k, k)
                if nk == "node_position":
                    m[nk] = map_keys(v)
                elif nk == "actions":
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        # detect nodePosition
        if any(k in obj for k in ("mapId", "map_id")):
            for k, v in obj.items():
                nk = np_map.get(k, k)
                m[nk] = map_keys(v)
            return m
        # detect edge
        if any(k in obj for k in ("edgeId", "edge_id", "startNodeId", "start_node_id")):
            for k, v in obj.items():
                nk = edge_map.get(k, k)
                if nk == "trajectory":
                    m[nk] = map_keys(v)
                elif nk == "actions":
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        # detect trajectory
        if any(k in obj for k in ("knotVector", "knot_vector", "controlPoints", "control_points")):
            for k, v in obj.items():
                nk = traj_map.get(k, k)
                m[nk] = map_keys(v)
            return m
        # detect action
        if any(k in obj for k in ("actionType", "action_type", "actionId", "action_id")):
            for k, v in obj.items():
                nk = act_map.get(k, k)
                if nk == "action_parameters":
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        # default: passthrough keys
        for k, v in obj.items():
            m[k] = map_keys(v)
        return m

    return map_keys(d)

def _convert_order_snake_to_camel(d: dict) -> dict:
    """Convert a VDA5050 order payload from snake_case to camelCase for publishing.

    Recursively converts known keys across order, nodes, edges, actions, trajectory, and nodePosition.
    Unknown keys are preserved as-is.
    """
    def map_keys(obj):
        if isinstance(obj, list):
            return [map_keys(x) for x in obj]
        if not isinstance(obj, dict):
            return obj
        m = {}
        # top-level/order-level mappings
        key_map = {
            "header_id": "headerId",
            "serial_number": "serialNumber",
            "order_id": "orderId",
            "order_update_id": "orderUpdateId",
            "zone_set_id": "zoneSetId",
        }
        # node-level
        node_map = {
            "node_id": "nodeId",
            "sequence_id": "sequenceId",
            "node_description": "nodeDescription",
            "node_position": "nodePosition",
        }
        # nodePosition-level
        np_map = {
            "map_id": "mapId",
            "map_description": "mapDescription",
            "allowed_deviation_xy": "allowedDeviationXY",
            "allowed_deviation_theta": "allowedDeviationTheta",
        }
        # edge-level
        edge_map = {
            "edge_id": "edgeId",
            "sequence_id": "sequenceId",
            "start_node_id": "startNodeId",
            "end_node_id": "endNodeId",
            "edge_description": "edgeDescription",
            "max_speed": "maxSpeed",
            "max_height": "maxHeight",
            "min_height": "minHeight",
            "orientation_type": "orientationType",
            "rotation_allowed": "rotationAllowed",
            "max_rotation_speed": "maxRotationSpeed",
            "length": "length",
            "direction": "direction",
            "orientation": "orientation",
            "trajectory": "trajectory",
        }
        # trajectory-level
        traj_map = {
            "knot_vector": "knotVector",
            "control_points": "controlPoints",
        }
        # action-level
        act_map = {
            "action_type": "actionType",
            "action_id": "actionId",
            "blocking_type": "blockingType",
            "action_description": "actionDescription",
            "action_parameters": "actionParameters",
        }
        # decide which map to use per object
        if "nodes" in obj or "edges" in obj:
            for k, v in obj.items():
                nk = key_map.get(k, k)
                if nk in ("nodes", "edges"):
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        if any(k in obj for k in ("node_id", "nodeId")):
            for k, v in obj.items():
                nk = node_map.get(k, k)
                if nk == "nodePosition":
                    m[nk] = map_keys(v)
                elif nk == "actions":
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        if any(k in obj for k in ("map_id", "mapId")):
            for k, v in obj.items():
                nk = np_map.get(k, k)
                m[nk] = map_keys(v)
            return m
        if any(k in obj for k in ("edge_id", "edgeId", "start_node_id", "startNodeId")):
            for k, v in obj.items():
                nk = edge_map.get(k, k)
                if nk == "trajectory":
                    m[nk] = map_keys(v)
                elif nk == "actions":
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        if any(k in obj for k in ("knot_vector", "knotVector", "control_points", "controlPoints")):
            for k, v in obj.items():
                nk = traj_map.get(k, k)
                m[nk] = map_keys(v)
            return m
        if any(k in obj for k in ("action_type", "actionType", "action_id", "actionId")):
            for k, v in obj.items():
                nk = act_map.get(k, k)
                if nk == "actionParameters":
                    m[nk] = map_keys(v)
                else:
                    m[nk] = map_keys(v)
            return m
        for k, v in obj.items():
            m[k] = map_keys(v)
        return m

    return map_keys(d)

# 发布 VDA5050 订单到 MQTT
@app.post("/api/agv/{serial_number}/order")
def publish_order(serial_number: str, body: dict = Body(...)):
    info = agv_manager.get_agv(serial_number)
    if not info:
        raise HTTPException(status_code=404, detail="AGV not found")
    # 简单结构校验（兼容 camelCase：转换为 snake_case 进行 dataclass 构造）
    try:
        snake_body = _convert_order_camel_to_snake(body)
        # 确保节点/边包含 actions 字段；若缺失则补空数组，避免设备端解析后缺失
        try:
            if isinstance(snake_body.get("nodes"), list):
                for n in snake_body["nodes"]:
                    if isinstance(n, dict) and "actions" not in n:
                        n["actions"] = []
            if isinstance(snake_body.get("edges"), list):
                for e in snake_body["edges"]:
                    if isinstance(e, dict) and "actions" not in e:
                        e["actions"] = []
        except Exception:
            pass
        # dataclass 构造需要 zone_set_id 键，但不希望在发布到 MQTT 时出现 null
        original_has_zone_set = ("zoneSetId" in body) or ("zone_set_id" in body)
        if "zone_set_id" not in snake_body:
            snake_body["zone_set_id"] = None
        VDAOrder(**snake_body)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid order payload: {e}")
    sn_in_body = str(body.get("serialNumber", body.get("serial_number", "")).strip())
    if sn_in_body and sn_in_body != info.serial_number:
        raise HTTPException(status_code=400, detail="serial_number mismatch")
    publisher = getattr(app.state, "mqtt_publisher", None)
    if not publisher:
        raise HTTPException(status_code=500, detail="MQTT publisher not available")
    try:
        # 统一使用 camelCase 进行发布，保证设备端解析一致
        # 使用经过补全的 snake_body 进行转换，以确保 actions 存在
        camel_body = _convert_order_snake_to_camel(snake_body)
        # 若原始请求未提供 zoneSetId，且其值为 null，则移除该键，避免发布 "zoneSetId": null
        if not original_has_zone_set and camel_body.get("zoneSetId") is None:
            try:
                del camel_body["zoneSetId"]
            except Exception:
                pass
        publisher.publish_order(info, camel_body)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Publish order failed: {e}")
    # 返回兼容的 orderId（同时提供两种命名以兼容旧前端）
    _oid = body.get("orderId") or body.get("order_id")
    return {"published": True, "serial_number": serial_number, "orderId": _oid, "order_id": _oid}

# 发布地图切换即时动作到 MQTT
@app.post("/api/agv/{serial_number}/instant/switch-map")
def publish_switch_map(serial_number: str, body: SwitchMapRequest):
    info = agv_manager.get_agv(serial_number)
    if not info:
        raise HTTPException(status_code=404, detail="AGV not found")
    publisher = getattr(app.state, "mqtt_publisher", None)
    if not publisher:
        raise HTTPException(status_code=500, detail="MQTT publisher not available")
    try:
        publisher.publish_switch_map(
            info,
            map_name=body.map,
            switch_point=body.switch_point,
            center_x=body.center_x,
            center_y=body.center_y,
            initiate_angle=body.initiate_angle,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Publish switchMap failed: {e}")
    # 在地图切换时立即持久化 map_id（避免频繁持久化坐标）
    try:
        from SimVehicleSys.utils.helpers import canonicalize_map_id
        agv_config_store.update_physical(serial_number, {"map_id": canonicalize_map_id(body.map)})
    except Exception:
        pass
    return {"published": True, "serial_number": serial_number}

# 手动触发运行态快照持久化（用于优雅停止前确保坐标保存）
@app.post("/api/system/persist-runtime")
def persist_runtime_snapshots():
    saved = 0
    try:
        overrides = getattr(app.state, "sim_settings_by_agv", {})
    except Exception:
        overrides = {}
    for info in agv_manager.list_agvs():
        try:
            # 尝试使用后端集中管理的状态（来自 MQTTBridge）
            status = agv_manager.get_status(info.serial_number)
            if status:
                from .schemas import AGVRuntime, Position
                rt = AGVRuntime(
                    battery_level=status.battery_level,
                    current_map=status.current_map,
                    position=Position(x=status.position.x, y=status.position.y, theta=status.position.theta),
                    speed_limit=1.0,
                )
                agv_config_store.update_runtime_snapshot(info.serial_number, rt)
                saved += 1
        except Exception:
            pass
        # 同步最近一次仿真设置覆盖（可选）
        try:
            patch = overrides.get(info.serial_number, {}) if isinstance(overrides, dict) else {}
            if patch:
                agv_config_store.update_physical(info.serial_number, patch)
        except Exception:
            pass
    return {"saved": saved}
