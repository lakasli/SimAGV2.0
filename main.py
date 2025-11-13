from __future__ import annotations
import subprocess
import sys
import time
import os
from pathlib import Path
import shutil
import json
import threading
import signal
import re

from SimVehicleSys.world_service import WorldModelService

PROJECT_ROOT = Path(__file__).resolve().parent


def _python_exec() -> list[str]:
    """Return a cross-platform Python launcher command.

    - On Windows: prefer the `py -3` launcher when available; fallback to `sys.executable`.
    - On Linux/macOS: use `sys.executable`.
    """
    try:
        if os.name == "nt":
            launcher = shutil.which("py")
            if launcher:
                return [launcher, "-3"]
            return [sys.executable]
        return [sys.executable]
    except Exception:
        return [sys.executable]


def _windows_creation_flags() -> int:
    try:
        if os.name == "nt":
            return getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    except Exception:
        return 0
    return 0

def start_backend(port: int) -> subprocess.Popen:
    host = os.getenv("SIMAGV_BACKEND_HOST", "0.0.0.0")
    cmd = _python_exec() + [
        "-m",
        "uvicorn",
        "backend.main:app",
        "--host",
        host,
        "--port",
        str(port),
    ]
    return subprocess.Popen(cmd, cwd=str(PROJECT_ROOT), creationflags=_windows_creation_flags())


def start_mosquitto(port: int | None = None) -> subprocess.Popen | None:
    # 优先在 Linux 使用 PATH 中的 'mosquitto' 可执行文件；Windows 作为回退
    if os.name != "nt":
        exe = shutil.which("mosquitto")
        if not exe:
            print("Mosquitto 未找到：请安装 mosquitto 并确保其在 PATH 中。")
            return None
        try:
            cmd = [exe]
            if port:
                cmd += ["-p", str(port)]
            return subprocess.Popen(
                cmd,
                cwd=str(Path(exe).parent),
                creationflags=_windows_creation_flags(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            print(f"启动 Mosquitto 失败: {e}")
            return None
    # Windows 回退路径
    exe_path = Path(r"D:\mosquitto\mosquitto.exe")
    if not exe_path.exists():
        print(f"Mosquitto 未找到: {exe_path}，请安装或检查路径。")
        return None
    try:
        cmd = [str(exe_path)]
        if port:
            cmd += ["-p", str(port)]
        return subprocess.Popen(
            cmd,
            cwd=str(exe_path.parent),
            creationflags=_windows_creation_flags(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print(f"启动 Mosquitto 失败: {e}")
        return None


# 端口清理工具

def find_pids_on_port(port: int) -> list[int]:
    """跨平台查询占用指定端口的进程 PID 列表。

    - 在 Linux：优先使用 `ss -lntp`，回退到 `lsof`。
    - 在 Windows：使用 `netstat -ano`。
    """
    try:
        pids: set[int] = set()
        if os.name == "nt":
            result = subprocess.run(["netstat", "-ano"], capture_output=True, text=True, check=False)
            for line in result.stdout.splitlines():
                if f":{port}" in line:
                    parts = line.split()
                    if parts and parts[0].upper() in ("TCP", "UDP"):
                        pid = parts[-1]
                        if pid.isdigit():
                            pids.add(int(pid))
            return list(pids)
        # Linux: ss
        try:
            result = subprocess.run(["ss", "-lntp"], capture_output=True, text=True, check=False)
            for line in result.stdout.splitlines():
                if f":{port} " in line or f":{port}" in line:
                    for m in re.finditer(r"pid=(\d+)", line):
                        try:
                            pids.add(int(m.group(1)))
                        except Exception:
                            pass
        except Exception:
            pass
        # Linux 回退：lsof
        if not pids:
            try:
                result = subprocess.run(["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN"], capture_output=True, text=True, check=False)
                lines = result.stdout.splitlines()
                for line in lines[1:]:  # 跳过标题行
                    cols = line.split()
                    if len(cols) >= 2 and cols[1].isdigit():
                        pids.add(int(cols[1]))
            except Exception:
                pass
        return list(pids)
    except Exception as e:
        print(f"查询端口 {port} 占用失败: {e}")
        return []


def free_port(port: int) -> None:
    pids = find_pids_on_port(port)
    if not pids:
        print(f"端口 {port} 未被占用。")
        return
    print(f"端口 {port} 被进程占用: {pids}，尝试结束...")
    for pid in pids:
        try:
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(pid), "/F"], check=False)
            else:
                try:
                    os.kill(pid, signal.SIGTERM)
                    time.sleep(0.2)
                except Exception:
                    pass
                try:
                    os.kill(pid, signal.SIGKILL)
                except Exception:
                    pass
        except Exception as e:
            print(f"结束 PID {pid} 失败: {e}")


def clean_necessary_ports() -> None:
    # 启动前清理必要端口：MQTT(9527) 与 后端(7000)
    for port in (9527, 7000):
        free_port(port)

# 新增：进程/端口检查工具

def is_process_running(image_name: str) -> bool:
    """跨平台进程存在性检查。

    - 在 Linux：使用 `pgrep -x`；接受不带 .exe 的名称。
    - 在 Windows：使用 `tasklist`。
    """
    try:
        if os.name == "nt":
            res = subprocess.run(["tasklist", "/FI", f"IMAGENAME eq {image_name}"], capture_output=True, text=True, check=False)
            return image_name.lower() in res.stdout.lower()
        # Linux
        names = [image_name]
        if image_name.endswith(".exe"):
            names.append(image_name.replace(".exe", ""))
        for nm in names:
            try:
                res = subprocess.run(["pgrep", "-x", nm], capture_output=True, text=True, check=False)
                if res.returncode == 0 and res.stdout.strip():
                    return True
            except Exception:
                pass
        return False
    except Exception:
        return False


def is_port_in_use(port: int) -> bool:
    return len(find_pids_on_port(port)) > 0


def find_free_port(start: int, max_steps: int = 50) -> int:
    """从起始端口起向上扫描，返回第一个未占用端口；若找不到则返回起始端口。"""
    port = int(start)
    for _ in range(max_steps):
        if not is_port_in_use(port):
            return port
        port += 1
    return int(start)


def _load_registered_agvs() -> list[dict]:
    """Load registered AGVs from backend storage file, if present."""
    data_path = PROJECT_ROOT / "backend" / "data" / "registered_agvs.json"
    if not data_path.exists():
        return []
    try:
        return json.loads(data_path.read_text(encoding="utf-8")) or []
    except Exception:
        return []


def start_simulators() -> list[subprocess.Popen]:
    # 根据注册列表启动多个仿真实例
    agvs = _load_registered_agvs()
    procs: list[subprocess.Popen] = []
    if not agvs:
        cmd = _python_exec() + ["-m", "SimVehicleSys.main"]
        procs.append(subprocess.Popen(cmd, cwd=str(PROJECT_ROOT), creationflags=_windows_creation_flags()))
        return procs
    for info in agvs:
        serial = str(info.get("serial_number", "")).strip()
        if not serial:
            continue
        cmd = _python_exec() + ["-m", "SimVehicleSys.main", "--serial", serial]
        try:
            proc = subprocess.Popen(cmd, cwd=str(PROJECT_ROOT), creationflags=_windows_creation_flags())
            procs.append(proc)
            print(f"启动仿真实例: {serial}")
        except Exception as e:
            print(f"启动仿真实例 {serial} 失败: {e}")
    return procs


def main() -> None:
    print("检查端口并选择可用端口...")
    mosq_proc = None
    img = "mosquitto.exe" if os.name == "nt" else "mosquitto"

    # 根据运动控制配置，将仿真车连接到本机 MQTT: 127.0.0.1:1884
    mqtt_host = os.getenv("SIMAGV_MQTT_HOST", "127.0.0.1")
    mqtt_port = os.getenv("SIMAGV_MQTT_PORT", "1884")

    # 将选定的 MQTT 主机/端口传播到后端与仿真器
    os.environ["SIMAGV_MQTT_HOST"] = str(mqtt_host)
    os.environ["SIMAGV_MQTT_PORT"] = str(mqtt_port)
    # 统一接口名称为 uagv（与仿真默认与订阅方一致）；可通过环境变量覆盖
    os.environ.setdefault("SIMAGV_MQTT_INTERFACE", os.getenv("SIMAGV_MQTT_INTERFACE", "uagv"))
    # 可选：覆盖 manufacturer 与 serial（与运动控制侧一致，若需要统一命名）
    if os.getenv("SIMAGV_MANUFACTURER"):
        os.environ["SIMAGV_MANUFACTURER"] = os.getenv("SIMAGV_MANUFACTURER", "")
    if os.getenv("SIMAGV_SERIAL"):
        os.environ["SIMAGV_SERIAL"] = os.getenv("SIMAGV_SERIAL", "")
    print(f"MQTT 服务器: {mqtt_host}:{mqtt_port} 接口: {os.getenv('SIMAGV_MQTT_INTERFACE', 'uagv')}")

    # 后端端口选择：若默认 7000 被占用则回退到下一个可用端口
    env_backend_port = os.getenv("SIMAGV_BACKEND_PORT")
    backend_port = int(env_backend_port) if (env_backend_port and env_backend_port.isdigit()) else 7000
    if env_backend_port is None and is_port_in_use(backend_port):
        backend_port = find_free_port(backend_port)
        print(f"默认后端端口 7000 已占用，回退到 {backend_port}")

    bind_host = os.getenv("SIMAGV_BACKEND_HOST", "127.0.0.1")
    print(f"启动后端服务 (Uvicorn) 于 http://{bind_host}:{backend_port} ...")
    backend_proc = start_backend(backend_port)
    time.sleep(1.5)

    # 启动世界模型服务线程（独立线程）
    print("启动世界模型服务线程...")
    world_service = WorldModelService(tickMs=100)
    world_thread = threading.Thread(target=world_service.start, daemon=True)
    world_thread.start()

    print("启动 AGV 仿真器...")
    simulator_procs: list[subprocess.Popen] = []
    try:
        simulator_procs = start_simulators()
    except Exception as e:
        print(f"Simulator start failed: {e}")
        simulator_procs = []
    print("所有服务已启动。按 Ctrl+C 停止。")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping services...")
        # 在停止前尝试请求后端持久化运行态快照（坐标与地图）；若不可达则静默跳过
        try:
            if is_port_in_use(backend_port):
                import urllib.request, time as _t
                import json as _json
                url = f"http://127.0.0.1:{backend_port}/api/system/persist-runtime"
                payload = _json.dumps({}).encode("utf-8")
                headers = {"Content-Type": "application/json"}
                attempts = 2
                for i in range(attempts):
                    try:
                        req = urllib.request.Request(url, data=payload, headers=headers, method="POST")
                        with urllib.request.urlopen(req, timeout=5.0) as resp:
                            _ = resp.read()
                            break
                    except Exception:
                        _t.sleep(0.5)
        except Exception:
            pass
        try:
            for p in simulator_procs:
                try:
                    p.terminate()
                except Exception:
                    pass
            backend_proc.terminate()
            try:
                world_service.stop()
            except Exception:
                pass
            if mosq_proc:
                mosq_proc.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()