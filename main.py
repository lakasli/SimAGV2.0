from __future__ import annotations
import subprocess
import sys
import time
import os
from pathlib import Path
import shutil
import json

PROJECT_ROOT = Path(__file__).resolve().parent


def start_backend() -> subprocess.Popen:
    cmd = [sys.executable, "-m", "uvicorn", "backend.main:app", "--host", "127.0.0.1", "--port", "8000"]
    return subprocess.Popen(cmd, cwd=str(PROJECT_ROOT))


def start_mosquitto() -> subprocess.Popen | None:
    exe_path = Path(r"D:\mosquitto\mosquitto.exe")
    if not exe_path.exists():
        print(f"Mosquitto 未找到: {exe_path}，请安装或检查路径。")
        return None
    try:
        cmd = [str(exe_path), "-v"]
        return subprocess.Popen(cmd, cwd=str(exe_path.parent))
    except Exception as e:
        print(f"启动 Mosquitto 失败: {e}")
        return None


# 端口清理工具

def find_pids_on_port(port: int) -> list[int]:
    try:
        result = subprocess.run(["netstat", "-ano"], capture_output=True, text=True, check=False)
        pids: set[int] = set()
        for line in result.stdout.splitlines():
            # 仅匹配包含端口的行
            if f":{port}" in line:
                parts = line.split()
                if parts and parts[0].upper() in ("TCP", "UDP"):
                    pid = parts[-1]
                    if pid.isdigit():
                        pids.add(int(pid))
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
            subprocess.run(["taskkill", "/PID", str(pid), "/F"], check=False)
        except Exception as e:
            print(f"结束 PID {pid} 失败: {e}")


def clean_necessary_ports() -> None:
    # 启动前清理必要端口：MQTT(1883) 与 后端(8000)
    for port in (1883, 8000):
        free_port(port)

# 新增：进程/端口检查工具

def is_process_running(image_name: str) -> bool:
    try:
        res = subprocess.run(["tasklist", "/FI", f"IMAGENAME eq {image_name}"], capture_output=True, text=True, check=False)
        return image_name.lower() in res.stdout.lower()
    except Exception:
        return False


def is_port_in_use(port: int) -> bool:
    return len(find_pids_on_port(port)) > 0


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
    # 根据注册列表启动多个仿真实例（如列表为空，则至少启动一个默认实例）
    agvs = _load_registered_agvs()
    procs: list[subprocess.Popen] = []
    if not agvs:
        cmd = [sys.executable, "-m", "SimVehicleSys.main"]
        procs.append(subprocess.Popen(cmd, cwd=str(PROJECT_ROOT)))
        return procs
    for info in agvs:
        serial = str(info.get("serial_number", "")).strip()
        if not serial:
            continue
        cmd = [sys.executable, "-m", "SimVehicleSys.main", "--serial", serial]
        try:
            proc = subprocess.Popen(cmd, cwd=str(PROJECT_ROOT))
            procs.append(proc)
            print(f"启动仿真实例: {serial}")
        except Exception as e:
            print(f"启动仿真实例 {serial} 失败: {e}")
    return procs


def main() -> None:
    print("检查 Mosquitto 运行状态...")
    mosq_proc = None
    if is_process_running("mosquitto.exe"):
        print("检测到 Mosquitto 已在运行")
    else:
        if is_port_in_use(1883):
            print("检测到 1883 已被占用，尝试释放...")
            free_port(1883)
        print("启动 MQTT Broker (Mosquitto)...")
        mosq_proc = start_mosquitto()
    time.sleep(1.0)

    # 仅清理后端端口，避免误杀已运行的 Mosquitto
    if is_port_in_use(8000):
        free_port(8000)

    print("启动后端服务 (Uvicorn)...")
    backend_proc = start_backend()
    time.sleep(1.5)

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
        try:
            for p in simulator_procs:
                try:
                    p.terminate()
                except Exception:
                    pass
            backend_proc.terminate()
            if mosq_proc:
                mosq_proc.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()