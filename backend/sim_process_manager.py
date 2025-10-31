from __future__ import annotations
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


class SimulatorProcessManager:
    """
    Windows 专用的仿真实例进程管理器：使用 PowerShell 启动 Python 仿真脚本。

    - 每个序列号一个独立进程：`powershell -NoProfile -ExecutionPolicy Bypass -Command "python -u SimVehicleSys/main.py --serial '<SN>'"`
    - 仅在未运行时启动，重复启动将被忽略。
    - 提供停止单个/全部实例的能力，应用关闭时清理。
    """

    def __init__(self, project_root: Path) -> None:
        self.project_root = Path(project_root)
        self.main_py = self.project_root / "SimVehicleSys" / "main.py"
        self._procs: Dict[str, subprocess.Popen] = {}

    def _powershell_cmd(self, serial: str) -> List[str]:
        """构造 PowerShell 启动命令参数列表。"""
        # 确保使用绝对路径并正确引用，避免空格路径问题
        py_path = str(self.main_py)
        serial_s = str(serial)
        ps_script = f'python -u "{py_path}" --serial "{serial_s}"'
        return [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            ps_script,
        ]

    def start_for(self, serial: str) -> bool:
        """为指定序列号启动仿真实例。返回是否新启动。"""
        serial = str(serial)
        if serial in self._procs and self._procs[serial] and self._procs[serial].poll() is None:
            # 已在运行
            return False
        if not self.main_py.exists():
            raise RuntimeError(f"Simulator entry not found: {self.main_py}")
        try:
            proc = subprocess.Popen(
                self._powershell_cmd(serial),
                cwd=str(self.project_root),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                creationflags=0,
                start_new_session=True,
            )
            self._procs[serial] = proc
            print(f"[SimProc] started {serial} (pid={proc.pid})")
            return True
        except Exception as e:
            print(f"[SimProc] start failed for {serial}: {e}")
            return False

    def stop_for(self, serial: str) -> bool:
        """停止指定序列号的仿真实例。返回是否成功停止。"""
        serial = str(serial)
        proc = self._procs.get(serial)
        if not proc:
            return False
        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except Exception:
                    proc.kill()
            print(f"[SimProc] stopped {serial}")
        except Exception as e:
            print(f"[SimProc] stop failed for {serial}: {e}")
            return False
        finally:
            self._procs.pop(serial, None)
        return True

    def ensure_running_for_serials(self, serials: List[str]) -> List[str]:
        """确保传入序列号列表的仿真实例均处于运行中。返回新启动的序列号列表。"""
        started: List[str] = []
        for sn in serials:
            if self.start_for(sn):
                started.append(sn)
        return started

    def list_running(self) -> List[str]:
        """列出当前处于运行中的序列号。"""
        running: List[str] = []
        for sn, p in list(self._procs.items()):
            if p and p.poll() is None:
                running.append(sn)
            else:
                # 清理已退出的句柄
                self._procs.pop(sn, None)
        return running

    def stop_all(self) -> int:
        """停止所有运行中的仿真实例，返回停止数量。"""
        count = 0
        for sn in list(self._procs.keys()):
            if self.stop_for(sn):
                count += 1
        return count