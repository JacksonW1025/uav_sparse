from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil


class SitlError(RuntimeError):
    pass


class SitlRunner:
    def __init__(self, config: dict[str, Any], repo_root: Path):
        self.config = config
        self.repo_root = repo_root
        self.planc_root = repo_root / "planc"
        self.work_root = self.planc_root / "work"
        self.logs_root = self.planc_root / "logs"
        self.process: subprocess.Popen[str] | None = None
        self.stdout_file = None
        self.work_dir: Path | None = None

    @property
    def binary(self) -> str:
        for raw in self.config["sitl"].get("vehicle_binary_candidates", []):
            if Path(raw).exists():
                return raw
        raise SitlError("No ArduCopter SITL binary found in configured candidates")

    @property
    def connection_string(self) -> str:
        return self.config["sitl"].get("connection", "tcp:127.0.0.1:5760")

    @property
    def defaults_file(self) -> str | None:
        for raw in self.config["sitl"].get("defaults_candidates", []):
            if Path(raw).exists():
                return raw
        return None

    def _home_arg(self) -> str:
        home = self.config["experiment"]["home"]
        return f"{home['lat']},{home['lon']},{home['alt_m']},{home['yaw_deg']}"

    def start(self, run_id: str) -> Path:
        if self.process is not None:
            raise SitlError("SITL process already running")
        self.work_dir = self.work_root / run_id
        if self.work_dir.exists():
            shutil.rmtree(self.work_dir)
        self.work_dir.mkdir(parents=True)
        self.logs_root.mkdir(parents=True, exist_ok=True)
        cmd = [
            self.binary,
            "--model",
            self.config["sitl"].get("model", "quad"),
            "--speedup",
            str(self.config["experiment"].get("speedup", 1)),
            "--wipe",
        ]
        defaults = self.defaults_file
        if defaults:
            cmd.extend(["--defaults", defaults])
        cmd.extend([
            "--home",
            self._home_arg(),
        ])
        (self.work_dir / "start_command.txt").write_text(" ".join(cmd) + "\n", encoding="utf-8")
        self.stdout_file = open(self.work_dir / "sitl_stdout.log", "w", encoding="utf-8")
        env = os.environ.copy()
        env["SIM_VEHICLE_SESSION"] = f"planc_{run_id}"
        self.process = subprocess.Popen(
            cmd,
            cwd=self.work_dir,
            stdout=self.stdout_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            env=env,
        )
        return self.work_dir

    def connect(self, timeout_s: float = 30.0):
        deadline = time.time() + timeout_s
        last_error: Exception | None = None
        while time.time() < deadline:
            if self.process and self.process.poll() is not None:
                raise SitlError(f"SITL exited before connection with code {self.process.returncode}")
            try:
                master = mavutil.mavlink_connection(
                    self.connection_string,
                    autoreconnect=False,
                    source_system=255,
                )
                heartbeat = master.wait_heartbeat(timeout=5)
                if heartbeat is not None:
                    master.target_system = heartbeat.get_srcSystem()
                    master.target_component = heartbeat.get_srcComponent()
                    return master
            except Exception as exc:
                last_error = exc
                time.sleep(0.5)
        raise SitlError(f"Timed out connecting to {self.connection_string}: {last_error}")

    def stop(self) -> None:
        proc = self.process
        self.process = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=8)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
                except Exception:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
        if self.stdout_file is not None:
            self.stdout_file.close()
            self.stdout_file = None

    def collect_dataflash(self, run_id: str) -> Path | None:
        if self.work_dir is None:
            return None
        candidates = list((self.work_dir / "logs").glob("*.BIN")) + list((self.work_dir / "logs").glob("*.bin"))
        if not candidates:
            candidates = list(self.work_dir.glob("*.BIN")) + list(self.work_dir.glob("*.bin"))
        if not candidates:
            return None
        newest = max(candidates, key=lambda p: p.stat().st_mtime)
        dst = self.logs_root / f"{run_id}.BIN"
        shutil.copy2(newest, dst)
        return dst
