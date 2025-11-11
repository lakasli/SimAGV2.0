# SimAGV2.0 项目说明

SimAGV2.0 是一个跨平台（Linux/Windows）的多车 AGV 仿真系统，基于 FastAPI 后端、MQTT 通信、WebSocket 推送与前端页面展示。系统遵循 VDA5050 风格的主题约定并支持多台仿真车的并行模拟与集中式安全检测。

## 项目功能
- 多车仿真：通过进程管理为多个序列号启动仿真车（`SimVehicleSys/main.py`）。
- MQTT 通信：按 VDA5050 风格发布 `connection/state/visualization/factsheet` 等主题。
- 后端服务：FastAPI 提供统一接口（注册、状态、配置、运动控制），并向前端广播状态（WebSocket）。
- 地图与静态资源：从 `maps/`（`.scene` 文件）与 `SimVehicleSys/shelf/` 挂载静态内容到后端。
- 世界模型与安全：集中式世界服务（`WorldModelService`）进行状态汇总与碰撞检测。

## 环境准备（Linux + Bash）
- 安装 Python（推荐 3.11+）。
- 安装 Mosquitto MQTT Broker（例如基于 Debian/Ubuntu：`sudo apt-get install mosquitto`）。
- 在项目根目录创建并激活虚拟环境，安装依赖：
  - `python3 -m venv .venv`
  - `source .venv/bin/activate`
  - `pip install -r requirements.txt`

## 快速开始（一键启动所有服务）
- 进入项目根目录并在 Bash 中运行：
  - `python3 ./main.py`
- 该脚本将：
  - 检查并启动 `Mosquitto`（端口 `1883`）。
  - 启动后端服务（Uvicorn，`http://127.0.0.1:8000`）。
  - 启动世界模型服务线程（碰撞与安全管理）。
  - 根据 `backend\data\registered_agvs.json` 启动一个或多个仿真实例。
- 打开浏览器访问：
  - `http://127.0.0.1:8000/`（后端直接返回 `frontend/index.html`）

## 手动启动各组件（按需组合）
- 启动 MQTT Broker：
  - `mosquitto -v`
- 启动后端服务（Uvicorn）：
  - `python3 -m uvicorn backend.main:app --host 127.0.0.1 --port 8000`
- 启动单台仿真车（可指定序列号）：
  - `python3 -m SimVehicleSys.main --serial AMB-01`
- 启动多台仿真车的方式：
  - 修改 `backend/data/registered_agvs.json` 添加序列号。
  - 通过一键脚本 `python3 ./main.py` 或后端注册接口自动拉起仿真实例。

## 前端启动
- 推荐通过后端访问：`http://127.0.0.1:8000/`（已挂载 `frontend/`，静态资源位于 `/static`）。
- 如果需要独立静态服务（调试用途）：
  - 在 `frontend` 目录运行：`python3 -m http.server 8080`
  - 访问：`http://127.0.0.1:8080`
  - 注意：独立静态服务与后端跨域时，前端接口地址需指向后端 `http://127.0.0.1:8000`。

## 常用接口与示例（curl）
- 查看地图列表：
  - `curl http://127.0.0.1:8000/api/maps`
  - `curl http://127.0.0.1:8000/api/maps/VehicleMap`
- 注册 AGV（将同时尝试启动对应仿真实例）：
  - `curl -X POST -H "Content-Type: application/json" -d '{"agvs":[{"serial_number":"AMB-03","manufacturer":"SEER","type":"AGV","vda_version":"v2","IP":"192.168.9.3"}]}' http://127.0.0.1:8000/api/agvs/register`
- 更新运行态配置（例如初始位置/地图/限速）：
  - `curl -X POST -H "Content-Type: application/json" -d '{"position":{"x":1.0,"y":2.0,"theta":0.0},"current_map":"testmap.scene","speed_limit":1.5}' http://127.0.0.1:8000/api/agv/AMB-01/config/dynamic`
- 平移控制（单位：米）：
  - `curl -X POST -H "Content-Type: application/json" -d '{"dx":1.0,"dy":0.0}' http://127.0.0.1:8000/api/agv/AMB-01/move/translate`
- 旋转控制（`dtheta` 为弧度，例：`1.5708`≈90°）：
  - `curl -X POST -H "Content-Type: application/json" -d '{"dtheta":1.5708}' http://127.0.0.1:8000/api/agv/AMB-01/move/rotate`
- WebSocket（前端状态推送）：
  - `ws://127.0.0.1:8000/ws`

## 端口与主题约定
- 后端：`http://127.0.0.1:8000`
- MQTT Broker：`127.0.0.1:1883`
- VDA MQTT 基础主题：`uagv/{vda_version}/{manufacturer}/{serial_number}`
  - 示例：`uagv/v2/SEER/AMB-01/state`

## 目录结构（简要）
```
SimAGV2.0/
├── backend/                # FastAPI 后端与进程管理
├── SimVehicleSys/          # 仿真车系统（车辆、MQTT、协议、世界服务等）
├── frontend/               # 前端静态页面（index/order/battery）
├── maps/                   # 地图文件（.scene）
├── main.py                 # 一键启动脚本（Mosquitto/后端/世界服务/仿真器）
└── requirements.txt        # Python 依赖
```

## 常见问题
- Mosquitto 未找到：在 Linux 下请确保 `mosquitto` 已安装并在 PATH 中（端口 1883）。
- 端口占用：一键脚本会尝试释放 `8000/1883`；若仍失败，请手动关闭占用进程后重试。
- 前端无法打开：确认后端已运行且首页 `GET /` 能返回 `frontend/index.html`；也可直接访问静态目录 `/static`。
- 多车未启动：确认 `backend\data\registered_agvs.json` 已包含目标序列号，或通过注册接口提交后重试。

## 退出与停用
- 在终端中按 `Ctrl+C` 停止。一键脚本将在停止前尝试持久化运行态快照，并终止后端、仿真实例与（如由脚本启动）Mosquitto。

## 备注
- 仿真默认配置（MQTT 主机/端口、车辆参数等）定义在 `SimVehicleSys/config/settings.py`，可按需调整。
- 仿真车 CLI 支持 `--serial` 指定序列号；当前 `--config` 预留参数未启用自定义加载逻辑。