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

# Static mounts: frontend and maps
frontend_dir = project_root / "frontend"
maps_dir = project_root / "maps"
if frontend_dir.exists():
    app.mount("/static", StaticFiles(directory=str(frontend_dir)), name="static")
if maps_dir.exists():
    app.mount("/maps", StaticFiles(directory=str(maps_dir)), name="maps")

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

# 已移除：SMAP 解析辅助函数（项目不再支持 .smap）

# 已移除：SMAP 拓扑接口（项目不再支持 .smap）

# 已移除：SMAP 站点详情接口（项目不再支持 .smap）


# 已移除：SMAP 路由接口（项目不再支持 .smap）
@app.get("/")
def index():
    index_file = frontend_dir / "index.html"
    if not index_file.exists():
        raise HTTPException(status_code=404, detail="Frontend not found")
    return FileResponse(index_file)

# 前端配置：提供轮询间隔（毫秒）
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
        return {"polling_interval_ms": interval}
    except Exception:
        # 失败时回退到 100ms
        return {"polling_interval_ms": 100}

# --- Navigation APIs ---

# 已迁移：锚点路径导航改由仿真车内置执行（如需保留，请改为发布 MQTT 指令）

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
    try:
        sim_manager.ensure_running_for_serials(registered)
    except Exception as e:
        print(f"[SimProc] auto-start failed: {e}")
    return RegisterResponse(registered=registered, skipped=skipped)

@app.delete("/api/agvs/{serial_number}", response_model=UnregisterResponse)
def unregister_agv(serial_number: str):
    ok = agv_manager.unregister(serial_number)
    if not ok:
        raise HTTPException(status_code=404, detail="AGV not found")
    try:
        sim_manager.stop_for(serial_number)
    except Exception as e:
        print(f"[SimProc] stop failed: {e}")
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
        if not (v > 0.0 and v < 10.0):
            add_err("时间缩放(sim_time_scale)必须在(0,10)范围内")
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

# 发布 VDA5050 订单到 MQTT
@app.post("/api/agv/{serial_number}/order")
def publish_order(serial_number: str, body: dict = Body(...)):
    info = agv_manager.get_agv(serial_number)
    if not info:
        raise HTTPException(status_code=404, detail="AGV not found")
    # 简单结构校验（使用 dataclass 尝试构造）
    try:
        VDAOrder(**body)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid order payload: {e}")
    sn_in_body = str(body.get("serial_number", "")).strip()
    if sn_in_body and sn_in_body != info.serial_number:
        raise HTTPException(status_code=400, detail="serial_number mismatch")
    publisher = getattr(app.state, "mqtt_publisher", None)
    if not publisher:
        raise HTTPException(status_code=500, detail="MQTT publisher not available")
    try:
        publisher.publish_order(info, body)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Publish order failed: {e}")
    return {"published": True, "serial_number": serial_number, "order_id": body.get("order_id")}