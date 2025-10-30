from __future__ import annotations
from pathlib import Path
from typing import List

from fastapi import FastAPI, HTTPException, Body
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from typing import Union
from .schemas import AGVRegistration, RegisterRequest, RegisterResponse, UnregisterResponse, AGVInfo, StaticConfigPatch, DynamicConfigUpdate, StatusResponse
from .schemas import TranslateRequest, RotateRequest
from .agv_manager import AGVManager
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
    return RegisterResponse(registered=registered, skipped=skipped)

@app.delete("/api/agvs/{serial_number}", response_model=UnregisterResponse)
def unregister_agv(serial_number: str):
    ok = agv_manager.unregister(serial_number)
    if not ok:
        raise HTTPException(status_code=404, detail="AGV not found")
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
    # 仿真车导航由其自身进程负责，后端仅保留 MQTT 桥与发布器

@app.on_event("shutdown")
async def _stop_ws_broadcast():
    task = getattr(app.state, "ws_broadcast_task", None)
    if task:
        task.cancel()
    bridge = getattr(app.state, "mqtt_bridge", None)
    if bridge:
        bridge.stop()

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