from __future__ import annotations

import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


def _first_existing(paths: list[str]) -> str | None:
    for raw in paths:
        path = Path(raw).expanduser()
        if path.exists():
            return str(path)
    return None


def _run(cmd: list[str], cwd: str | None = None, timeout: int = 10) -> str | None:
    try:
        proc = subprocess.run(
            cmd,
            cwd=cwd,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
        )
    except Exception as exc:
        return f"ERROR: {exc}"
    return proc.stdout.strip()


def _python_package_versions() -> dict[str, str]:
    versions: dict[str, str] = {}
    for name in ("pymavlink", "yaml", "numpy", "matplotlib"):
        try:
            module = __import__(name)
            versions[name] = str(getattr(module, "__version__", "installed"))
        except Exception as exc:
            versions[name] = f"missing: {exc}"
    return versions


def probe_environment(config: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    sitl_cfg = config["sitl"]
    ardupilot_root = _first_existing(sitl_cfg.get("ardupilot_root_candidates", []))
    vehicle_binary = _first_existing(sitl_cfg.get("vehicle_binary_candidates", []))
    sim_vehicle = _first_existing(sitl_cfg.get("sim_vehicle_candidates", []))
    defaults = _first_existing(sitl_cfg.get("defaults_candidates", []))
    env: dict[str, Any] = {
        "repo_root": str(repo_root),
        "python": sys.version,
        "platform": platform.platform(),
        "packages": _python_package_versions(),
        "sim_vehicle_py": sim_vehicle,
        "vehicle_binary": vehicle_binary,
        "defaults_file": defaults,
        "ardupilot_root": ardupilot_root,
        "connection": sitl_cfg.get("connection"),
        "model": sitl_cfg.get("model", "quad"),
        "speedup": config["experiment"].get("speedup"),
        "sim_vehicle_on_path": shutil.which("sim_vehicle.py"),
        "env_PATH": os.environ.get("PATH", ""),
        "param_metadata": config.get("param_metadata", {}),
    }
    if ardupilot_root:
        env["ardupilot_commit"] = _run(["git", "rev-parse", "HEAD"], cwd=ardupilot_root)
        env["ardupilot_commit_short"] = _run(["git", "rev-parse", "--short", "HEAD"], cwd=ardupilot_root)
        env["ardupilot_status"] = _run(["git", "status", "--short", "--branch"], cwd=ardupilot_root)
    if vehicle_binary:
        env["vehicle_binary_help_head"] = _run([vehicle_binary, "--help"], timeout=5)
    env["selected_start_command_template"] = [
        vehicle_binary or "<missing-arducopter>",
        "--model",
        sitl_cfg.get("model", "quad"),
        "--speedup",
        str(config["experiment"].get("speedup", 1)),
        "--wipe",
        "--defaults",
        defaults or "<missing-defaults>",
        "--home",
        "<lat,lon,alt,yaw>",
    ]
    return env


def write_env(env: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(env, indent=2, sort_keys=True) + "\n", encoding="utf-8")
