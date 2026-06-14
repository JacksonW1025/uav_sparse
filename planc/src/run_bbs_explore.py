from __future__ import annotations

import argparse
import bisect
import copy
import csv
import json
import math
import statistics
import sys
import time
import traceback
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from pymavlink import mavutil

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))

from flight import land_and_disarm, release_rc_override, send_gcs_heartbeat, set_mode
from injector import send_guided_velocity_local_ned
from oracle import COPTER_MODES, horizontal_distance_m
from param_manager import ParamManager
from run_rtl_energy import (
    classify_run,
    controlled_params as energy_controlled_params,
    parse_energy_dataflash,
    prepare_flight,
    run_rtl_energy_flight,
)
from sitl_runner import SitlRunner


EXPLORE_ROOT = PLANC_ROOT / "explore"
DATA_DIR = EXPLORE_ROOT / "data"
PLOT_DIR = EXPLORE_ROOT / "plots"
DOC_DIR = EXPLORE_ROOT / "docs"
LOG_REF_DIR = EXPLORE_ROOT / "log_refs"

HOME = {
    "lat": 37.4275,
    "lon": -122.1697,
    "alt_m": 16.0,
    "yaw_deg": 90.0,
}


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def rel(path: Path | str | None) -> str:
    if path is None:
        return ""
    try:
        return str(Path(path).resolve().relative_to(REPO_ROOT))
    except Exception:
        return str(path)


def ensure_dirs() -> None:
    for path in (EXPLORE_ROOT, DATA_DIR, PLOT_DIR, DOC_DIR, LOG_REF_DIR):
        path.mkdir(parents=True, exist_ok=True)


def load_energy_config() -> dict[str, Any]:
    cfg = load_yaml(PLANC_ROOT / "config" / "rtl_energy_config.yaml")
    cfg = copy.deepcopy(cfg)
    cfg["experiment"]["name"] = "bbs_explore_energy_margin"
    cfg["experiment"]["version"] = "bbs_explore_2026_06_14"
    cfg["experiment"]["home"] = dict(HOME)
    cfg["experiment"]["speedup"] = 6
    cfg["experiment"]["stream_hz"] = 10
    return cfg


def base_authority_config(model_json: str | None = None) -> dict[str, Any]:
    energy = load_energy_config()
    cfg = {
        "experiment": {
            "name": "bbs_explore_authority_demand",
            "version": "bbs_explore_2026_06_14",
            "home": dict(HOME),
            "takeoff_alt_m": 24.0,
            "stream_hz": 20,
            "speedup": 4,
        },
        "sitl": copy.deepcopy(energy["sitl"]),
        "baseline_params": {
            "LOG_BACKEND_TYPE": 1,
            "LOG_DISARMED": 1,
            "SYSID_MYGCS": 255,
            "FS_GCS_ENABLE": 0,
            "FS_THR_ENABLE": 0,
            "SIM_RC_FAIL": 0,
            "FENCE_ENABLE": 0,
            "FENCE_TYPE": 0,
            "AVOID_ENABLE": 0,
            "BATT_LOW_VOLT": 0,
            "BATT_CRT_VOLT": 0,
            "BATT_LOW_MAH": 0,
            "BATT_CRT_MAH": 0,
            "BATT_FS_LOW_ACT": 0,
            "BATT_FS_CRT_ACT": 0,
            "SIM_WIND_DIR": 270,
            "SIM_WIND_SPD": 4,
            "SIM_WIND_TURB": 1.5,
            "ANGLE_MAX": 3000,
            "PILOT_SPEED_UP": 300,
            "PILOT_SPEED_DN": 1200,
            "PILOT_ACCEL_Z": 1000,
        },
    }
    if model_json:
        cfg["sitl"]["model_json_source"] = str(model_json)
    return cfg


def time_s(data: dict[str, Any]) -> float | None:
    if "TimeUS" in data:
        return float(data["TimeUS"]) / 1.0e6
    if "TimeMS" in data:
        return float(data["TimeMS"]) / 1000.0
    return None


def field(data: dict[str, Any], *names: str) -> Any:
    for name in names:
        if name in data:
            return data[name]
    return None


def latlon(value: Any) -> float | None:
    if value is None:
        return None
    value_f = float(value)
    if abs(value_f) > 1000:
        return value_f / 1.0e7
    return value_f


def nearest(rows: list[dict[str, Any]], t_s: float | None) -> dict[str, Any] | None:
    if not rows or t_s is None:
        return None
    return min(rows, key=lambda r: abs(float(r["time_s"]) - t_s))


def rolling_slope(rows: list[dict[str, Any]], t_s: float, value_key: str, window_s: float) -> float | None:
    window = [
        (float(r["time_s"]), float(r[value_key]))
        for r in rows
        if r.get(value_key) not in (None, "") and t_s - window_s <= float(r["time_s"]) <= t_s
    ]
    if len(window) < 3:
        return None
    xs = np.array([p[0] for p in window], dtype=float)
    ys = np.array([p[1] for p in window], dtype=float)
    if float(xs[-1] - xs[0]) <= 1.0e-6:
        return None
    return float(np.polyfit(xs - xs[0], ys, 1)[0])


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: list[str] = []
        for row in rows:
            for key in row:
                if key not in keys:
                    keys.append(key)
        fieldnames = keys
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def run_with_sitl(
    *,
    cfg: dict[str, Any],
    run_id: str,
    params: dict[str, Any],
    flight_func,
) -> dict[str, Any]:
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "params_requested": params,
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_path = LOG_REF_DIR / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result["param_snapshot"] = snapshot
        result["param_records_path"] = str(param_path)
        result["param_readbacks"] = pm.records
        result["flight"] = flight_func(master)
        try:
            master.close()
        except Exception:
            pass
        master = None
        runner.stop()
        bin_path = runner.collect_dataflash(run_id)
        result["work_dir"] = str(work_dir)
        if bin_path is None:
            result["error"] = "No DataFlash .BIN log found after run"
            return result
        result["bin_path"] = str(bin_path)
        return result
    except Exception as exc:
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        return result
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def send_rc_override(master, roll: int = 1500, pitch: int = 1500, throttle: int = 1500, yaw: int = 1500) -> None:
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        int(roll),
        int(pitch),
        int(throttle),
        int(yaw),
        0,
        0,
        0,
        0,
    )


def release_all_rc(master) -> None:
    release_rc_override(master)
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def wait_wall_with_heartbeat(master, seconds: float, rc: tuple[int, int, int, int] | None = None) -> None:
    end = time.time() + seconds
    while time.time() < end:
        send_gcs_heartbeat(master)
        if rc is not None:
            send_rc_override(master, *rc)
        master.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"], blocking=True, timeout=0.05)


def inventory_flight(master) -> dict[str, Any]:
    prepare_flight(master, {"experiment": {"home": HOME, "takeoff_alt_m": 18.0}, "unused": {}})
    wait_wall_with_heartbeat(master, 2.0)
    send_guided_velocity_local_ned(master, 2.0, 0.0, 0.0)
    wait_wall_with_heartbeat(master, 3.0)
    send_guided_velocity_local_ned(master, 0.0, 2.0, 0.0)
    wait_wall_with_heartbeat(master, 3.0)
    send_guided_velocity_local_ned(master, 0.0, 0.0, 0.0)
    wait_wall_with_heartbeat(master, 1.5)
    set_mode(master, "ALT_HOLD", timeout_s=10)
    for pwm in (1500, 1600, 1700, 1600, 1500, 1400, 1300, 1400, 1500):
        wait_wall_with_heartbeat(master, 0.35, rc=(pwm, 1500, 1500, 1500))
    release_all_rc(master)
    wait_wall_with_heartbeat(master, 1.0)
    land_and_disarm(master, timeout_s=35.0)
    return {"motion": "takeoff_hover_guided_mild_alt_hold_roll_land"}


def mode_name_from_data(data: dict[str, Any]) -> str:
    mode = field(data, "Mode", "ModeNum")
    if isinstance(mode, str):
        if mode.isdigit():
            return COPTER_MODES.get(int(mode), mode)
        return mode
    if mode is not None:
        try:
            return COPTER_MODES.get(int(mode), str(mode))
        except Exception:
            return str(mode)
    return "UNKNOWN"


def dataflash_inventory(bin_path: Path) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    messages: dict[str, dict[str, Any]] = {}
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue
        data = msg.to_dict()
        rec = messages.setdefault(mtype, {"count": 0, "fields": set()})
        rec["count"] += 1
        for key in data:
            if key != "mavpackettype":
                rec["fields"].add(key)
    clean = {
        key: {"count": value["count"], "fields": sorted(value["fields"])}
        for key, value in sorted(messages.items())
    }
    return clean


def pick_fields(inv: dict[str, Any], mtype: str, candidates: list[str]) -> list[str]:
    fields = set(inv.get(mtype, {}).get("fields", []))
    return [name for name in candidates if name in fields]


def write_inventory_doc(inv: dict[str, Any], run: dict[str, Any]) -> Path:
    desired = [
        ("电量", "BAT", ["Volt", "VoltR", "Curr", "CurrTot", "EnrgTot", "RemPct"]),
        ("姿态", "ATT", ["Roll", "DesRoll", "Pitch", "DesPitch", "Yaw", "DesYaw"]),
        ("角速率", "RATE", ["R", "RDes", "P", "PDes", "Y", "YDes", "ROut", "POut", "YOut", "AOut"]),
        ("电机输出", "RCOU", ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8"]),
        ("遥控输入", "RCIN", ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8"]),
        ("高度/爬升", "CTUN", ["Alt", "BAlt", "DAlt", "CRt", "DCRt", "ThO", "ThH"]),
        ("位置", "POS", ["Lat", "Lng", "Alt", "RelHomeAlt", "Vel"]),
        ("位置备用", "GPS", ["Lat", "Lng", "Alt", "Spd", "GCrs", "Status"]),
    ]
    lines = [
        "# Probe 0 Telemetry Field Inventory",
        "",
        f"- run_id: `{run.get('run_id')}`",
        f"- DataFlash: `{rel(run.get('bin_path'))}`",
        f"- 解析时间: {datetime.now(timezone.utc).isoformat()}",
        "",
        "## 想读的量 -> 实际字段名 / 取不到",
        "",
        "| 想读的量 | 消息类型 | 实际字段名 / 取不到 | 样本数 |",
        "|---|---|---|---:|",
    ]
    for label, mtype, candidates in desired:
        got = pick_fields(inv, mtype, candidates)
        fields = ", ".join(f"`{name}`" for name in got) if got else "取不到"
        count = inv.get(mtype, {}).get("count", 0)
        lines.append(f"| {label} | `{mtype}` | {fields} | {count} |")
    lines.extend([
        "",
        "## 实际消息类型和字段",
        "",
        "| 消息类型 | 样本数 | 字段 |",
        "|---|---:|---|",
    ])
    for mtype, rec in inv.items():
        fields = ", ".join(f"`{f}`" for f in rec["fields"])
        lines.append(f"| `{mtype}` | {rec['count']} | {fields} |")
    path = EXPLORE_ROOT / "telemetry_field_inventory.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def run_probe0() -> dict[str, Any]:
    ensure_dirs()
    cfg = base_authority_config()
    cfg["experiment"]["takeoff_alt_m"] = 18.0
    params = dict(cfg["baseline_params"])
    params.update({
        "SIM_WIND_SPD": 0,
        "SIM_WIND_TURB": 0,
        "ANGLE_MAX": 3000,
        "LOG_BITMASK": 131071,
    })
    run = run_with_sitl(
        cfg=cfg,
        run_id="bbs_probe0_inventory",
        params=params,
        flight_func=inventory_flight,
    )
    if run.get("bin_path"):
        inv = dataflash_inventory(Path(run["bin_path"]))
        write_json(DATA_DIR / "probe0_inventory.json", inv)
        write_inventory_doc(inv, run)
        run["inventory_path"] = str(DATA_DIR / "probe0_inventory.json")
    write_json(DATA_DIR / "probe0_run.json", run)
    return run


def parse_energy_timeseries(
    bin_path: Path,
    *,
    home: dict[str, Any],
    params: dict[str, Any],
    target_bearing_deg: float,
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    bat: list[dict[str, Any]] = []
    pos: list[dict[str, Any]] = []
    vel: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    start_time: float | None = None
    home_lat = float(home["lat"])
    home_lon = float(home["lon"])
    bearing = math.radians(float(target_bearing_deg))
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue
        data = msg.to_dict()
        ts = time_s(data)
        if ts is None:
            continue
        if start_time is None:
            start_time = ts
        rel_t = ts - start_time
        if mtype == "BAT":
            instance = int(field(data, "Instance") or 0)
            if instance == 0:
                currtot = field(data, "CurrTot", "CurrentTot", "Consumed")
                curr = field(data, "Curr")
                remaining = None
                if currtot is not None:
                    remaining = float(params.get("BATT_CAPACITY", 0.0)) - float(currtot)
                bat.append({
                    "time_s": rel_t,
                    "volt": field(data, "Volt", "VoltR"),
                    "curr_A": curr,
                    "currtot_mAh": None if currtot is None else float(currtot),
                    "remaining_mAh": remaining,
                })
        elif mtype == "MODE":
            modes.append({"time_s": rel_t, "mode": mode_name_from_data(data), "reason": field(data, "Rsn", "Reason")})
        elif mtype == "XKF1":
            core = field(data, "C")
            if core is None or int(core) == 0:
                vn = float(field(data, "VN") or 0.0)
                ve = float(field(data, "VE") or 0.0)
                vel.append({
                    "time_s": rel_t,
                    "vn_m_s": vn,
                    "ve_m_s": ve,
                    "ground_speed_m_s": math.hypot(vn, ve),
                    "forward_speed_m_s": vn * math.cos(bearing) + ve * math.sin(bearing),
                })
        if mtype in {"POS", "GPS", "GPS2"}:
            lat = latlon(field(data, "Lat", "latitude"))
            lon = latlon(field(data, "Lng", "Lon", "longitude"))
            if lat is None or lon is None or abs(lat) < 1.0e-9 or abs(lon) < 1.0e-9:
                continue
            if mtype.startswith("GPS"):
                status = field(data, "Status")
                if status is not None and float(status) < 3:
                    continue
            alt = field(data, "Alt", "RelHomeAlt", "RAlt")
            pos.append({
                "time_s": rel_t,
                "source": mtype,
                "lat": lat,
                "lon": lon,
                "alt": "" if alt is None else float(alt),
                "distance_m": horizontal_distance_m(home_lat, home_lon, lat, lon),
            })
    if len([p for p in pos if p["source"] == "POS"]) >= 10:
        pos = [p for p in pos if p["source"] == "POS"]
    elif len([p for p in pos if p["source"].startswith("GPS")]) >= 10:
        pos = [p for p in pos if p["source"].startswith("GPS")]
    pos.sort(key=lambda r: float(r["time_s"]))
    bat.sort(key=lambda r: float(r["time_s"]))
    vel.sort(key=lambda r: float(r["time_s"]))
    return {"bat": bat, "pos": pos, "vel": vel, "modes": modes}


def energy_margin_at(
    *,
    timeseries: dict[str, Any],
    t_s: float,
    params: dict[str, Any],
    proxy_rate_mAh_s: float | None = None,
    proxy_groundspeed_m_s: float | None = None,
) -> dict[str, Any]:
    bat_row = nearest(timeseries["bat"], t_s)
    pos_row = nearest(timeseries["pos"], t_s)
    vel_row = nearest(timeseries["vel"], t_s)
    remaining = None if bat_row is None else bat_row.get("remaining_mAh")
    usable = None
    if remaining is not None:
        usable = max(0.0, float(remaining) - float(params.get("BATT_CRT_MAH", 0.0)))
    actual_rate = None
    if bat_row is not None and bat_row.get("curr_A") not in (None, ""):
        actual_rate = max(0.0, float(bat_row["curr_A"]) * 1000.0 / 3600.0)
    slope = rolling_slope(timeseries["bat"], t_s, "currtot_mAh", 12.0)
    if slope is not None and slope > 0:
        actual_rate = slope
    actual_gs = None if vel_row is None else float(vel_row["ground_speed_m_s"])
    rate = proxy_rate_mAh_s if proxy_rate_mAh_s is not None else actual_rate
    gs = proxy_groundspeed_m_s if proxy_groundspeed_m_s is not None else actual_gs
    dist = None if pos_row is None else float(pos_row["distance_m"])
    flyable = None
    margin = None
    if usable is not None and rate is not None and rate > 1.0e-6 and gs is not None:
        flyable = usable / rate * gs
        if dist is not None:
            margin = flyable - dist
    return {
        "time_s": t_s,
        "remaining_mAh": remaining,
        "usable_remaining_to_critical_mAh": usable,
        "consumption_rate_mAh_s": rate,
        "actual_consumption_rate_mAh_s": actual_rate,
        "groundspeed_m_s": gs,
        "actual_groundspeed_m_s": actual_gs,
        "home_distance_m": dist,
        "flyable_distance_proxy_m": flyable,
        "margin_proxy_m": margin,
    }


def estimate_return_window_proxy(
    timeseries: dict[str, Any],
    *,
    low_time_s: float,
    critical_time_s: float | None,
) -> dict[str, Any]:
    start = low_time_s + 2.0
    end = low_time_s + 16.0
    if critical_time_s is not None:
        end = min(end, max(start + 3.0, critical_time_s - 1.0))
    bat_window = [
        r for r in timeseries["bat"]
        if r.get("currtot_mAh") not in (None, "") and start <= float(r["time_s"]) <= end
    ]
    vel_window = [
        r for r in timeseries["vel"]
        if r.get("ground_speed_m_s") not in (None, "") and start <= float(r["time_s"]) <= end
    ]
    rate = None
    if len(bat_window) >= 3:
        xs = np.array([float(r["time_s"]) for r in bat_window], dtype=float)
        ys = np.array([float(r["currtot_mAh"]) for r in bat_window], dtype=float)
        if float(xs[-1] - xs[0]) > 1.0e-6:
            rate = float(np.polyfit(xs - xs[0], ys, 1)[0])
    speed = None
    if vel_window:
        speed = float(statistics.median(float(r["ground_speed_m_s"]) for r in vel_window))
    return {
        "return_window_start_s": start,
        "return_window_end_s": end,
        "return_window_consumption_rate_mAh_s": rate,
        "return_window_groundspeed_m_s": speed,
        "return_window_battery_samples": len(bat_window),
        "return_window_velocity_samples": len(vel_window),
    }


def energy_margin_curve(
    timeseries: dict[str, Any],
    params: dict[str, Any],
    *,
    proxy_rate_mAh_s: float | None = None,
    proxy_groundspeed_m_s: float | None = None,
) -> list[dict[str, Any]]:
    rows = []
    for bat_row in timeseries["bat"]:
        t = float(bat_row["time_s"])
        row = energy_margin_at(
            timeseries=timeseries,
            t_s=t,
            params=params,
            proxy_rate_mAh_s=proxy_rate_mAh_s,
            proxy_groundspeed_m_s=proxy_groundspeed_m_s,
        )
        if row["margin_proxy_m"] is not None:
            rows.append(row)
    return rows


def earliest_stable_sign(curve: list[dict[str, Any]], low_time_s: float | None, final_sign: int) -> float | None:
    if low_time_s is None or final_sign == 0:
        return None
    usable = [r for r in curve if 20.0 <= float(r["time_s"]) <= low_time_s and r.get("margin_proxy_m") is not None]
    for idx, row in enumerate(usable):
        suffix = usable[idx:]
        if len(suffix) < 3:
            continue
        signs = [1 if float(r["margin_proxy_m"]) >= 0 else -1 for r in suffix]
        if signs and all(s == final_sign for s in signs):
            return float(row["time_s"])
    return None


def run_energy_point(config: dict[str, Any], distance_m: float, wind_m_s: float) -> dict[str, Any]:
    run_id = f"bbs_probe1_D{int(distance_m):03d}_W{int(wind_m_s):02d}"
    cfg = copy.deepcopy(config)
    params = energy_controlled_params(config, {
        "SIM_WIND_SPD": float(wind_m_s),
        "BATT_LOW_MAH": 220,
        "LOG_BITMASK": 131071,
    })
    result: dict[str, Any] = {
        "run_id": run_id,
        "distance_m": float(distance_m),
        "wind_m_s": float(wind_m_s),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    existing_bin = PLANC_ROOT / "logs" / f"{run_id}.BIN"
    if existing_bin.exists():
        result["reused_existing_bin"] = True
        result["bin_path"] = str(existing_bin)
        param_path = LOG_REF_DIR / f"{run_id}_params.json"
        if param_path.exists():
            try:
                param_payload = json.loads(param_path.read_text(encoding="utf-8"))
                result["param_records_path"] = str(param_path)
                result["param_readbacks"] = param_payload.get("records", [])
                result["param_snapshot"] = param_payload.get("snapshot", {})
            except Exception:
                result["param_readbacks"] = []
        parsed_csv = DATA_DIR / f"{run_id}_position.csv"
        parsed = parse_energy_dataflash(
            bin_path=existing_bin,
            csv_path=parsed_csv,
            home=cfg["experiment"]["home"],
            params=params,
            run_kind="rtl_scan",
            target_distance_m=float(distance_m),
            cruise_speed_m_s=float(config["experiment"]["cruise_speed_m_s"]),
            target_bearing_deg=float(config["experiment"]["target_bearing_deg"]),
            home_radius_m=float(config["experiment"]["home_radius_m"]),
            d_tolerance_m=float(config["experiment"]["d_reached_tolerance_m"]),
            speed_tolerance_m_s=float(config["experiment"]["speed_audit_tolerance_m_s"]),
            speed_audit_min_distance_m=float(config["experiment"]["speed_audit_min_distance_m"]),
        )
        result.update(parsed)
        result["csv_path"] = str(parsed_csv)
        classify_run(result)
        ts = parse_energy_timeseries(
            existing_bin,
            home=cfg["experiment"]["home"],
            params=params,
            target_bearing_deg=float(config["experiment"]["target_bearing_deg"]),
        )
        low_time = result.get("low_time_s")
        return_proxy = None
        if low_time is not None:
            return_proxy = estimate_return_window_proxy(
                ts,
                low_time_s=float(low_time),
                critical_time_s=None if result.get("critical_time_s") is None else float(result["critical_time_s"]),
            )
            result["return_window_proxy"] = return_proxy
        curve = energy_margin_curve(
            ts,
            params,
            proxy_rate_mAh_s=None if not return_proxy else return_proxy.get("return_window_consumption_rate_mAh_s"),
            proxy_groundspeed_m_s=None if not return_proxy else return_proxy.get("return_window_groundspeed_m_s"),
        )
        ts_csv = DATA_DIR / f"{run_id}_margin_timeseries.csv"
        write_csv(ts_csv, curve)
        result["margin_timeseries_csv"] = str(ts_csv)
        if low_time is not None:
            actual_at_low = energy_margin_at(timeseries=ts, t_s=float(low_time), params=params)
            margin_low = energy_margin_at(
                timeseries=ts,
                t_s=float(low_time),
                params=params,
                proxy_rate_mAh_s=None if not return_proxy else return_proxy.get("return_window_consumption_rate_mAh_s"),
                proxy_groundspeed_m_s=None if not return_proxy else return_proxy.get("return_window_groundspeed_m_s"),
            )
            margin_low["trigger_actual_groundspeed_m_s"] = actual_at_low.get("actual_groundspeed_m_s")
            margin_low["trigger_actual_consumption_rate_mAh_s"] = actual_at_low.get("actual_consumption_rate_mAh_s")
        else:
            margin_low = {}
        result["margin_at_low"] = margin_low
        final_margin = margin_low.get("margin_proxy_m")
        final_sign = 0 if final_margin is None else (1 if float(final_margin) >= 0 else -1)
        stable = earliest_stable_sign(curve, float(low_time) if low_time is not None else None, final_sign)
        result["earliest_stable_margin_sign_time_s"] = stable
        result["early_prediction_lead_s"] = None if stable is None or low_time is None else float(low_time) - stable
        return result
    runner = SitlRunner(cfg, REPO_ROOT)
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_path = LOG_REF_DIR / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result["params_requested"] = params
        result["param_snapshot"] = snapshot
        result["param_records_path"] = str(param_path)
        result["param_readbacks"] = pm.records
        result["flight"] = run_rtl_energy_flight(master, cfg, float(distance_m))
        try:
            master.close()
        except Exception:
            pass
        master = None
        runner.stop()
        bin_path = runner.collect_dataflash(run_id)
        result["work_dir"] = str(work_dir)
        if bin_path is None:
            result["error"] = "No DataFlash .BIN log found after run"
            return result
        result["bin_path"] = str(bin_path)
        parsed_csv = DATA_DIR / f"{run_id}_position.csv"
        parsed = parse_energy_dataflash(
            bin_path=bin_path,
            csv_path=parsed_csv,
            home=cfg["experiment"]["home"],
            params=params,
            run_kind="rtl_scan",
            target_distance_m=float(distance_m),
            cruise_speed_m_s=float(config["experiment"]["cruise_speed_m_s"]),
            target_bearing_deg=float(config["experiment"]["target_bearing_deg"]),
            home_radius_m=float(config["experiment"]["home_radius_m"]),
            d_tolerance_m=float(config["experiment"]["d_reached_tolerance_m"]),
            speed_tolerance_m_s=float(config["experiment"]["speed_audit_tolerance_m_s"]),
            speed_audit_min_distance_m=float(config["experiment"]["speed_audit_min_distance_m"]),
        )
        result.update(parsed)
        result["csv_path"] = str(parsed_csv)
        classify_run(result)
        ts = parse_energy_timeseries(
            bin_path,
            home=cfg["experiment"]["home"],
            params=params,
            target_bearing_deg=float(config["experiment"]["target_bearing_deg"]),
        )
        return_proxy = None
        if result.get("low_time_s") is not None:
            return_proxy = estimate_return_window_proxy(
                ts,
                low_time_s=float(result["low_time_s"]),
                critical_time_s=None if result.get("critical_time_s") is None else float(result["critical_time_s"]),
            )
            result["return_window_proxy"] = return_proxy
        curve = energy_margin_curve(
            ts,
            params,
            proxy_rate_mAh_s=None if not return_proxy else return_proxy.get("return_window_consumption_rate_mAh_s"),
            proxy_groundspeed_m_s=None if not return_proxy else return_proxy.get("return_window_groundspeed_m_s"),
        )
        ts_csv = DATA_DIR / f"{run_id}_margin_timeseries.csv"
        write_csv(ts_csv, curve)
        result["margin_timeseries_csv"] = str(ts_csv)
        low_time = result.get("low_time_s")
        if low_time is not None:
            actual_at_low = energy_margin_at(timeseries=ts, t_s=float(low_time), params=params)
            margin_low = energy_margin_at(
                timeseries=ts,
                t_s=float(low_time),
                params=params,
                proxy_rate_mAh_s=None if not return_proxy else return_proxy.get("return_window_consumption_rate_mAh_s"),
                proxy_groundspeed_m_s=None if not return_proxy else return_proxy.get("return_window_groundspeed_m_s"),
            )
            margin_low["trigger_actual_groundspeed_m_s"] = actual_at_low.get("actual_groundspeed_m_s")
            margin_low["trigger_actual_consumption_rate_mAh_s"] = actual_at_low.get("actual_consumption_rate_mAh_s")
        else:
            margin_low = {}
        result["margin_at_low"] = margin_low
        final_margin = margin_low.get("margin_proxy_m")
        final_sign = 0 if final_margin is None else (1 if float(final_margin) >= 0 else -1)
        stable = earliest_stable_sign(curve, float(low_time) if low_time is not None else None, final_sign)
        result["earliest_stable_margin_sign_time_s"] = stable
        result["early_prediction_lead_s"] = None if stable is None or low_time is None else float(low_time) - stable
        return result
    except Exception as exc:
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        return result
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def probe1_rows(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for run in runs:
        m = run.get("margin_at_low") or {}
        rows.append({
            "run_id": run.get("run_id"),
            "D_m": run.get("distance_m"),
            "wind_m_s": run.get("wind_m_s"),
            "low_time_s": run.get("low_time_s"),
            "low_remaining_mAh": run.get("low_remaining_mAh"),
            "usable_remaining_to_critical_mAh": m.get("usable_remaining_to_critical_mAh"),
            "trigger_home_distance_m": run.get("distance_at_low_failsafe_m") or m.get("home_distance_m"),
            "trigger_actual_groundspeed_m_s": m.get("trigger_actual_groundspeed_m_s", m.get("actual_groundspeed_m_s")),
            "trigger_actual_consumption_rate_mAh_s": m.get("trigger_actual_consumption_rate_mAh_s", m.get("actual_consumption_rate_mAh_s")),
            "return_window_groundspeed_m_s": (run.get("return_window_proxy") or {}).get("return_window_groundspeed_m_s"),
            "return_window_consumption_rate_mAh_s": (run.get("return_window_proxy") or {}).get("return_window_consumption_rate_mAh_s"),
            "margin_proxy_groundspeed_m_s": m.get("groundspeed_m_s"),
            "margin_proxy_consumption_rate_mAh_s": m.get("consumption_rate_mAh_s"),
            "flyable_distance_proxy_m": m.get("flyable_distance_proxy_m"),
            "margin_proxy_m": m.get("margin_proxy_m"),
            "final_home_distance_m": run.get("final_distance_m"),
            "home_radius_m": run.get("home_radius_m"),
            "landed_within_home_radius": run.get("safe_binary"),
            "returned_home_after_low": run.get("returned_home_after_low"),
            "d_reached": run.get("d_reached"),
            "speed_audit_within_tolerance": (run.get("speed_audit") or {}).get("within_tolerance"),
            "earliest_stable_margin_sign_time_s": run.get("earliest_stable_margin_sign_time_s"),
            "early_prediction_lead_s": run.get("early_prediction_lead_s"),
            "contract_violations": ";".join(run.get("contract_violations", [])),
            "bin_path": rel(run.get("bin_path")),
        })
    return rows


def plot_probe1_margin(rows: list[dict[str, Any]]) -> Path:
    path = PLOT_DIR / "probe1_margin_vs_D.png"
    xs = np.array([float(r["D_m"]) for r in rows], dtype=float)
    ys = np.array([float(r["margin_proxy_m"]) if r["margin_proxy_m"] not in (None, "") else np.nan for r in rows], dtype=float)
    binary = [bool(r["landed_within_home_radius"]) for r in rows]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(xs, ys, color="#2b6cb0", marker="o", label="margin proxy")
    for x, y, ok in zip(xs, ys, binary):
        ax.scatter([x], [y], s=90, marker="o" if ok else "X", color="#2f855a" if ok else "#c53030", zorder=5)
    ax.axhline(0, color="#4a5568", linewidth=1, linestyle="--")
    ax.set_xlabel("Outbound distance D (m)")
    ax.set_ylabel("Margin proxy at RTL trigger (m)")
    ax.set_title("Probe 1: energy margin proxy vs D")
    ax.grid(True, alpha=0.25)
    ax.legend(["margin proxy", "landed within home radius", "outside home radius"], loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def plot_probe1_timeseries(runs: list[dict[str, Any]]) -> Path:
    path = PLOT_DIR / "probe1_margin_timeseries_examples.png"
    candidates = [r for r in runs if r.get("margin_timeseries_csv") and not r.get("error")]
    if len(candidates) > 2:
        candidates = [candidates[0], candidates[-1]]
    fig, axes = plt.subplots(len(candidates), 1, figsize=(8, 3.2 * max(1, len(candidates))), sharex=False)
    if not isinstance(axes, np.ndarray):
        axes = np.array([axes])
    for ax, run in zip(axes, candidates):
        rows = []
        with Path(run["margin_timeseries_csv"]).open(encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
        if not rows:
            continue
        t = np.array([float(r["time_s"]) for r in rows], dtype=float)
        remaining = np.array([float(r["usable_remaining_to_critical_mAh"]) for r in rows], dtype=float)
        flyable = np.array([float(r["flyable_distance_proxy_m"]) for r in rows], dtype=float)
        margin = np.array([float(r["margin_proxy_m"]) for r in rows], dtype=float)
        ax2 = ax.twinx()
        ax.plot(t, remaining, color="#2b6cb0", label="usable remaining mAh")
        ax2.plot(t, flyable, color="#805ad5", label="flyable distance proxy")
        ax2.plot(t, margin, color="#dd6b20", linestyle="--", label="margin proxy")
        low = run.get("low_time_s")
        stable = run.get("earliest_stable_margin_sign_time_s")
        if low is not None:
            ax.axvline(float(low), color="#4a5568", linewidth=1, linestyle=":", label="RTL trigger")
        if stable is not None:
            ax.axvline(float(stable), color="#c53030", linewidth=1, linestyle="-.", label="stable sign time")
        ax.set_title(f"{run['run_id']} (D={run.get('distance_m')} m)")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("usable mAh")
        ax2.set_ylabel("distance / margin proxy (m)")
        ax.grid(True, alpha=0.25)
        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines + lines2, labels + labels2, fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def write_probe1_doc(rows: list[dict[str, Any]], runs: list[dict[str, Any]]) -> Path:
    margins = [float(r["margin_proxy_m"]) for r in rows if r.get("margin_proxy_m") not in (None, "")]
    diffs = [margins[i + 1] - margins[i] for i in range(len(margins) - 1)]
    sign_changes = [r for r in rows if r.get("landed_within_home_radius") is False]
    leads = [float(r["early_prediction_lead_s"]) for r in rows if r.get("early_prediction_lead_s") not in (None, "")]
    def fmt_seq(key: str, suffix: str = "", digits: int = 1) -> str:
        vals = []
        for row in rows:
            value = row.get(key)
            vals.append("n/a" if value in (None, "") else f"{float(value):.{digits}f}{suffix}")
        return ", ".join(vals)
    lines = [
        "# Probe 1 Observations",
        "",
        "## 观察记录",
        "",
        f"- 本轮固定 `BATT_LOW_MAH=220`, `BATT_CRT_MAH=60`, `SIM_WIND_DIR=270`, `SIM_WIND_SPD=6 m/s`; D 点数为 {len(rows)}。",
        f"- `BAT.CurrTot` 和 `BAT.Curr` 都可读；低电量触发发生在 D 点悬停耗电阶段，触发瞬间 `XKF1` 地速接近 0，所以 margin 代理使用触发后早期 RTL 窗口的 `CurrTot` 斜率和 `XKF1` 地速。",
        f"- 触发瞬间实际地速序列: {fmt_seq('trigger_actual_groundspeed_m_s', ' m/s', 2)}；margin 代理使用的返航窗口地速序列: {fmt_seq('return_window_groundspeed_m_s', ' m/s', 2)}。",
        f"- margin proxy 序列: {fmt_seq('margin_proxy_m', ' m')}。",
        f"- 相邻 D 的 margin proxy 差分: {', '.join(f'{d:.1f} m' for d in diffs) if diffs else 'n/a'}。",
        "- D=120 m 的 margin proxy 贴近 0，但落点仍在 home radius 内；这个点把代理误差和二值切片的边界偏移暴露出来。",
        f"- 二值落点使用 `home_radius_m=10`；在这些点里 outside-home 的 D 为 {', '.join(str(int(float(r['D_m']))) for r in sign_changes) if sign_changes else '无'}。",
        f"- 可稳定读出最终 margin 符号的提前量: {', '.join(f'{v:.1f} s' for v in leads) if leads else 'n/a'}。",
        "",
        "## 对方法设计的启示",
        "",
        "- 余量代理应优先用 `BAT.CurrTot` 的短窗斜率而不是瞬时 `BAT.Curr`，因为它直接对应累积预算并减少瞬时电流抖动；若任务流程含悬停耗电，速度/耗电率要用返航段代理，而不是触发瞬间地速。",
        "- 早停门可以围绕“margin 符号稳定保持”来定义；本轮提前量使用返航窗口代理重放得到，在线早停需要把返航速度/耗电率改成先验或触发后短窗估计。",
        "- 二值落点只保留了 `home_radius_m` 以内/以外的切片；保留 `margin_proxy_m` 和 `final_home_distance_m` 能给拟合器更多连续尺度。",
    ]
    path = EXPLORE_ROOT / "probe1_observations.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def run_probe1() -> list[dict[str, Any]]:
    ensure_dirs()
    config = load_energy_config()
    distances = [60.0, 100.0, 120.0, 140.0, 160.0]
    wind = 6.0
    runs = []
    for distance in distances:
        print(f"RUN probe1 D={distance} wind={wind}", flush=True)
        run = run_energy_point(config, distance, wind)
        runs.append(run)
        write_json(DATA_DIR / "probe1_runs_partial.json", {"runs": runs})
    rows = probe1_rows(runs)
    write_csv(DATA_DIR / "probe1_energy_margin.csv", rows)
    plot_probe1_margin(rows)
    plot_probe1_timeseries(runs)
    write_probe1_doc(rows, runs)
    write_json(DATA_DIR / "probe1_runs.json", {"runs": runs})
    return runs


@dataclass
class RollProgram:
    r_pwm_s: float
    amplitude_pwm: int = 450
    hold_s: float = 0.7
    neutral_s: float = 0.4
    cycles: int = 2


def authority_flight(master, program: RollProgram) -> dict[str, Any]:
    cfg = {"experiment": {"home": HOME, "takeoff_alt_m": 24.0}, "unused": {}}
    prepare_flight(master, cfg)
    wait_wall_with_heartbeat(master, 1.5)
    set_mode(master, "ALT_HOLD", timeout_s=10)
    wait_wall_with_heartbeat(master, 1.0, rc=(1500, 1500, 1500, 1500))
    sent: list[dict[str, Any]] = []
    start = time.time()
    current = 1500.0
    targets: list[tuple[float, float]] = []
    abort_reason = None
    for _ in range(program.cycles):
        targets.extend([
            (1500 + program.amplitude_pwm, program.hold_s),
            (1500 - program.amplitude_pwm, program.hold_s),
            (1500, program.neutral_s),
        ])
    hz = 25.0
    dt = 1.0 / hz
    try:
        for target, hold in targets:
            if abort_reason:
                break
            while abs(current - target) > 1.0:
                step = math.copysign(min(abs(target - current), program.r_pwm_s * dt), target - current)
                current += step
                now = time.time()
                send_gcs_heartbeat(master)
                send_rc_override(master, roll=int(round(current)), pitch=1500, throttle=1500, yaw=1500)
                sent.append({"wall_s": now - start, "roll_pwm": int(round(current))})
                msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.02)
                if msg is not None and msg.get_type() == "GLOBAL_POSITION_INT":
                    alt = float(getattr(msg, "relative_alt", 0.0)) / 1000.0
                    if alt < 0.5:
                        abort_reason = "altitude_below_0p5m_during_rc_program"
                        break
                delay = dt - (time.time() - now)
                if delay > 0:
                    time.sleep(delay)
            if abort_reason:
                break
            end_hold = time.time() + hold
            while time.time() < end_hold:
                now = time.time()
                send_gcs_heartbeat(master)
                send_rc_override(master, roll=int(round(target)), pitch=1500, throttle=1500, yaw=1500)
                sent.append({"wall_s": now - start, "roll_pwm": int(round(target))})
                msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.02)
                if msg is not None and msg.get_type() == "GLOBAL_POSITION_INT":
                    alt = float(getattr(msg, "relative_alt", 0.0)) / 1000.0
                    if alt < 0.5:
                        abort_reason = "altitude_below_0p5m_during_rc_program"
                        break
                delay = dt - (time.time() - now)
                if delay > 0:
                    time.sleep(delay)
            if abort_reason:
                break
    finally:
        release_all_rc(master)
    wait_wall_with_heartbeat(master, 1.5, rc=(1500, 1500, 1500, 1500))
    release_all_rc(master)
    cleanup_error = None
    try:
        land_and_disarm(master, timeout_s=35.0)
    except Exception as exc:
        cleanup_error = repr(exc)
    intervals = [sent[i]["wall_s"] - sent[i - 1]["wall_s"] for i in range(1, len(sent))]
    return {
        "motion": "alt_hold_rc_roll_doublet",
        "program": {
            "r_pwm_s": program.r_pwm_s,
            "amplitude_pwm": program.amplitude_pwm,
            "cycles": program.cycles,
            "hold_s": program.hold_s,
        },
        "sent_command_samples": len(sent),
        "sent_command_first": sent[:10],
        "sent_command_last": sent[-10:],
        "send_timing": {
            "mean_dt_s": statistics.fmean(intervals) if intervals else None,
            "max_dt_s": max(intervals) if intervals else None,
        },
        "abort_reason": abort_reason,
        "cleanup_error": cleanup_error,
    }


def parse_authority_dataflash(bin_path: Path) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    start_time: float | None = None
    att: list[dict[str, Any]] = []
    rate: list[dict[str, Any]] = []
    rc: list[dict[str, Any]] = []
    rcou: list[dict[str, Any]] = []
    ctun: list[dict[str, Any]] = []
    pos: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue
        data = msg.to_dict()
        ts = time_s(data)
        if ts is None:
            continue
        if start_time is None:
            start_time = ts
        t = ts - start_time
        if mtype == "ATT":
            att.append({
                "time_s": t,
                "roll_deg": field(data, "Roll"),
                "desroll_deg": field(data, "DesRoll"),
                "pitch_deg": field(data, "Pitch"),
                "despitch_deg": field(data, "DesPitch"),
            })
        elif mtype == "RATE":
            rate.append({
                "time_s": t,
                "r_deg_s": field(data, "R"),
                "rdes_deg_s": field(data, "RDes"),
                "p_deg_s": field(data, "P"),
                "pdes_deg_s": field(data, "PDes"),
                "rout": field(data, "ROut"),
                "pout": field(data, "POut"),
            })
        elif mtype == "RCIN":
            row = {"time_s": t}
            for ch in range(1, 9):
                row[f"c{ch}"] = field(data, f"C{ch}")
            rc.append(row)
        elif mtype == "RCOU":
            row = {"time_s": t}
            for ch in range(1, 9):
                row[f"c{ch}"] = field(data, f"C{ch}")
            rcou.append(row)
        elif mtype == "CTUN":
            ctun.append({
                "time_s": t,
                "alt_m": field(data, "Alt"),
                "crt_m_s": field(data, "CRt"),
                "tho": field(data, "ThO"),
            })
        elif mtype in {"POS", "GPS", "GPS2"}:
            lat = latlon(field(data, "Lat"))
            lon = latlon(field(data, "Lng", "Lon"))
            if lat is not None and lon is not None and abs(lat) > 1.0e-9 and abs(lon) > 1.0e-9:
                alt = field(data, "Alt", "RelHomeAlt", "RAlt")
                pos.append({"time_s": t, "source": mtype, "lat": lat, "lon": lon, "alt": alt})
        elif mtype == "MODE":
            modes.append({"time_s": t, "mode": mode_name_from_data(data), "reason": field(data, "Rsn", "Reason")})
        elif mtype == "EV":
            events.append({"time_s": t, "id": field(data, "Id"), "raw": data})
        elif mtype in {"MSG", "STAT"}:
            messages.append({"time_s": t, "text": str(field(data, "Message", "Msg", "Text") or "")})
    return {
        "att": att,
        "rate": rate,
        "rcin": rc,
        "rcou": rcou,
        "ctun": ctun,
        "pos": pos,
        "modes": modes,
        "events": events,
        "messages": messages,
    }


def write_authority_timeseries(run_id: str, parsed: dict[str, Any]) -> Path:
    def prep(rows: list[dict[str, Any]]) -> tuple[list[float], list[dict[str, Any]]]:
        ordered = sorted(rows, key=lambda r: float(r["time_s"]))
        return [float(r["time_s"]) for r in ordered], ordered

    def nearest_sorted(times: list[float], rows: list[dict[str, Any]], t: float) -> dict[str, Any] | None:
        if not rows:
            return None
        idx = bisect.bisect_left(times, t)
        choices = []
        if idx < len(rows):
            choices.append(rows[idx])
        if idx > 0:
            choices.append(rows[idx - 1])
        return min(choices, key=lambda r: abs(float(r["time_s"]) - t)) if choices else None

    att_times, att_rows = prep(parsed.get("att", []))
    rc_times, rc_rows = prep(parsed.get("rcin", []))
    rcou_times, rcou_rows = prep(parsed.get("rcou", []))
    ctun_times, ctun_rows = prep(parsed.get("ctun", []))
    times = sorted({
        round(float(r["time_s"]), 2)
        for key in ("rcin", "rcou", "ctun")
        for r in parsed.get(key, [])
    })
    rows: list[dict[str, Any]] = []
    for t in times:
        att = nearest_sorted(att_times, att_rows, t)
        rc = nearest_sorted(rc_times, rc_rows, t)
        out = nearest_sorted(rcou_times, rcou_rows, t)
        ctun = nearest_sorted(ctun_times, ctun_rows, t)
        row = {"time_s": t}
        if att is not None and abs(float(att["time_s"]) - t) < 0.15:
            row.update({k: v for k, v in att.items() if k != "time_s"})
        if rc is not None and abs(float(rc["time_s"]) - t) < 0.15:
            row.update({f"rcin_{k}": v for k, v in rc.items() if k != "time_s"})
        if out is not None and abs(float(out["time_s"]) - t) < 0.15:
            row.update({f"rcou_{k}": v for k, v in out.items() if k != "time_s"})
        if ctun is not None and abs(float(ctun["time_s"]) - t) < 0.15:
            row.update({k: v for k, v in ctun.items() if k != "time_s"})
        rows.append(row)
    path = DATA_DIR / f"{run_id}_authority_timeseries.csv"
    write_csv(path, rows)
    return path


def sustained_threshold(rows: list[tuple[float, float]], threshold: float, duration_s: float) -> bool:
    start: float | None = None
    for t, value in rows:
        if abs(value) >= threshold:
            if start is None:
                start = t
            if t - start >= duration_s:
                return True
        else:
            start = None
    return False


def authority_metrics(parsed: dict[str, Any], program: RollProgram) -> dict[str, Any]:
    rc_rolls_all = [(float(r["time_s"]), float(r["c1"])) for r in parsed["rcin"] if r.get("c1") not in (None, "")]
    command_times = [t for t, v in rc_rolls_all if abs(v - 1500.0) > 20.0]
    if command_times:
        command_start = max(0.0, min(command_times) - 0.5)
        command_end = max(command_times) + 1.0
    else:
        command_start = 0.0
        command_end = max((float(r["time_s"]) for rows in parsed.values() if isinstance(rows, list) for r in rows if isinstance(r, dict) and "time_s" in r), default=0.0)

    def in_command(row: dict[str, Any]) -> bool:
        return command_start <= float(row["time_s"]) <= command_end

    att_rows = [
        r for r in parsed["att"]
        if in_command(r)
        if r.get("roll_deg") not in (None, "") and r.get("desroll_deg") not in (None, "")
    ]
    roll_errors = [float(r["roll_deg"]) - float(r["desroll_deg"]) for r in att_rows]
    pitch_errors = [
        float(r["pitch_deg"]) - float(r["despitch_deg"])
        for r in parsed["att"]
        if in_command(r)
        if r.get("pitch_deg") not in (None, "") and r.get("despitch_deg") not in (None, "")
    ]
    att_error_mag = []
    for r in parsed["att"]:
        if not in_command(r):
            continue
        if all(r.get(k) not in (None, "") for k in ("roll_deg", "desroll_deg", "pitch_deg", "despitch_deg")):
            att_error_mag.append(math.hypot(float(r["roll_deg"]) - float(r["desroll_deg"]), float(r["pitch_deg"]) - float(r["despitch_deg"])))
    rc_rolls = [(t, v) for t, v in rc_rolls_all if command_start <= t <= command_end]
    rc_slopes = [
        abs((rc_rolls[i][1] - rc_rolls[i - 1][1]) / max(1.0e-6, rc_rolls[i][0] - rc_rolls[i - 1][0]))
        for i in range(1, len(rc_rolls))
        if 900 <= rc_rolls[i][1] <= 2100 and 900 <= rc_rolls[i - 1][1] <= 2100
    ]
    des = [(float(r["time_s"]), float(r["desroll_deg"])) for r in att_rows]
    des_slopes = [
        abs((des[i][1] - des[i - 1][1]) / max(1.0e-6, des[i][0] - des[i - 1][0]))
        for i in range(1, len(des))
    ]
    motor_values: list[list[float]] = []
    for row in parsed["rcou"]:
        if not in_command(row):
            continue
        vals = [float(row[f"c{i}"]) for i in range(1, 5) if row.get(f"c{i}") not in (None, "")]
        if vals:
            motor_values.append(vals)
    high_outputs = [max(vals) for vals in motor_values]
    low_outputs = [min(vals) for vals in motor_values]
    limit_margins = [min(min(vals) - 1000.0, 2000.0 - max(vals)) for vals in motor_values]
    ctun_alts = [float(r["alt_m"]) for r in parsed["ctun"] if in_command(r) and r.get("alt_m") not in (None, "")]
    pre_command_alts = [
        float(r["alt_m"]) for r in parsed["ctun"]
        if command_start - 1.0 <= float(r["time_s"]) <= command_start + 1.0 and r.get("alt_m") not in (None, "")
    ]
    start_alt = statistics.median(pre_command_alts) if pre_command_alts else (ctun_alts[0] if ctun_alts else None)
    min_alt = min(ctun_alts) if ctun_alts else None
    max_drop = None if start_alt is None or min_alt is None else max(0.0, start_alt - min_alt)
    att_error_pairs = []
    for r in parsed["att"]:
        if not in_command(r):
            continue
        if all(r.get(k) not in (None, "") for k in ("roll_deg", "desroll_deg", "pitch_deg", "despitch_deg")):
            err = math.hypot(float(r["roll_deg"]) - float(r["desroll_deg"]), float(r["pitch_deg"]) - float(r["despitch_deg"]))
            att_error_pairs.append((float(r["time_s"]), err))
    sustained_err = sustained_threshold(att_error_pairs, threshold=20.0, duration_s=1.0)
    touched = bool(min_alt is not None and min_alt <= 0.7)
    crash_text = any(
        command_start <= float(m.get("time_s", 0.0)) <= command_end and "crash" in str(m.get("text", "")).lower()
        for m in parsed["messages"]
    )
    return {
        "command_window_start_s": command_start,
        "command_window_end_s": command_end,
        "commanded_r_pwm_s": program.r_pwm_s,
        "commanded_r_deg_s_proxy": program.r_pwm_s / 500.0 * 30.0,
        "command_amplitude_pwm": program.amplitude_pwm,
        "actual_rcin_peak_abs_pwm": max((abs(v - 1500.0) for _, v in rc_rolls), default=None),
        "actual_rcin_p95_rate_pwm_s": float(np.percentile(np.array(rc_slopes), 95)) if rc_slopes else None,
        "actual_desroll_peak_abs_deg": max((abs(float(r["desroll_deg"])) for r in att_rows), default=None),
        "actual_desroll_p95_rate_deg_s": float(np.percentile(np.array(des_slopes), 95)) if des_slopes else None,
        "actual_roll_peak_abs_deg": max((abs(float(r["roll_deg"])) for r in att_rows), default=None),
        "peak_roll_error_deg": max((abs(e) for e in roll_errors), default=None),
        "peak_pitch_error_deg": max((abs(e) for e in pitch_errors), default=None),
        "peak_att_error_deg": max(att_error_mag) if att_error_mag else None,
        "peak_motor_pwm": max(high_outputs) if high_outputs else None,
        "min_motor_pwm": min(low_outputs) if low_outputs else None,
        "min_motor_limit_margin_pwm": min(limit_margins) if limit_margins else None,
        "peak_motor_saturation_fraction": None if not high_outputs else (max(high_outputs) - 1000.0) / 1000.0,
        "start_alt_m": start_alt,
        "min_alt_m": min_alt,
        "max_drop_m": max_drop,
        "loss_of_control_proxy": bool(sustained_err or touched),
        "sustained_att_error_gt20deg_1s": bool(sustained_err),
        "touch_or_near_ground": touched,
        "crash_text_seen": crash_text,
        "mode_sequence": ";".join(m["mode"] for m in parsed["modes"]),
        "statustext": ";".join(m["text"] for m in parsed["messages"][-12:]),
    }


def run_authority_point(
    *,
    run_id: str,
    angle_max: int,
    turbulence: float,
    wind_speed: float,
    r_pwm_s: float,
    model_json: str,
    stage: str,
) -> dict[str, Any]:
    cfg = base_authority_config(model_json=model_json)
    params = dict(cfg["baseline_params"])
    params.update({
        "ANGLE_MAX": angle_max,
        "SIM_WIND_SPD": wind_speed,
        "SIM_WIND_TURB": turbulence,
        "LOG_BITMASK": 131071,
    })
    program = RollProgram(r_pwm_s=float(r_pwm_s))
    existing_bin = PLANC_ROOT / "logs" / f"{run_id}.BIN"
    if existing_bin.exists():
        run = {
            "run_id": run_id,
            "reused_existing_bin": True,
            "bin_path": str(existing_bin),
            "params_requested": params,
        }
        param_path = LOG_REF_DIR / f"{run_id}_params.json"
        if param_path.exists():
            try:
                param_payload = json.loads(param_path.read_text(encoding="utf-8"))
                run["param_records_path"] = str(param_path)
                run["param_readbacks"] = param_payload.get("records", [])
                run["param_snapshot"] = param_payload.get("snapshot", {})
            except Exception:
                pass
    else:
        run = run_with_sitl(
            cfg=cfg,
            run_id=run_id,
            params=params,
            flight_func=lambda master: authority_flight(master, program),
        )
    run.update({
        "stage": stage,
        "angle_max_cd": angle_max,
        "turbulence": turbulence,
        "wind_speed_m_s": wind_speed,
        "model_json": model_json,
        "commanded_r_pwm_s": r_pwm_s,
    })
    if run.get("bin_path"):
        parsed = parse_authority_dataflash(Path(run["bin_path"]))
        ts_path = write_authority_timeseries(run_id, parsed)
        metrics = authority_metrics(parsed, program)
        run.update(metrics)
        run["authority_timeseries_csv"] = str(ts_path)
    return run


def probe2_rows(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    fields = [
        "run_id",
        "stage",
        "angle_max_cd",
        "turbulence",
        "wind_speed_m_s",
        "commanded_r_pwm_s",
        "command_window_start_s",
        "command_window_end_s",
        "commanded_r_deg_s_proxy",
        "command_amplitude_pwm",
        "actual_rcin_peak_abs_pwm",
        "actual_rcin_p95_rate_pwm_s",
        "actual_desroll_peak_abs_deg",
        "actual_desroll_p95_rate_deg_s",
        "actual_roll_peak_abs_deg",
        "peak_motor_pwm",
        "min_motor_pwm",
        "min_motor_limit_margin_pwm",
        "peak_motor_saturation_fraction",
        "peak_roll_error_deg",
        "peak_pitch_error_deg",
        "peak_att_error_deg",
        "start_alt_m",
        "min_alt_m",
        "max_drop_m",
        "loss_of_control_proxy",
        "sustained_att_error_gt20deg_1s",
        "touch_or_near_ground",
        "crash_text_seen",
        "mode_sequence",
        "statustext",
        "authority_timeseries_csv",
        "bin_path",
        "error",
    ]
    rows = []
    for run in runs:
        rows.append({key: rel(run.get(key)) if key in {"authority_timeseries_csv", "bin_path"} else run.get(key) for key in fields})
    return rows


def plot_probe2(rows: list[dict[str, Any]]) -> Path:
    path = PLOT_DIR / "probe2_authority_demand_vs_r.png"
    main = [r for r in rows if r.get("stage") == "main" and not r.get("error")]
    fig, axes = plt.subplots(3, 1, figsize=(7.5, 9), sharex=True)
    metrics = [
        ("min_motor_limit_margin_pwm", "Motor limit margin (PWM, lower tighter)"),
        ("peak_att_error_deg", "Peak attitude error (deg)"),
        ("max_drop_m", "Max altitude drop (m)"),
    ]
    by_turb: dict[float, list[dict[str, Any]]] = {}
    for row in main:
        by_turb.setdefault(float(row["turbulence"]), []).append(row)
    for ax, (key, ylabel) in zip(axes, metrics):
        for turb, group in sorted(by_turb.items()):
            group = sorted(group, key=lambda r: float(r["commanded_r_pwm_s"]))
            xs = [float(r["commanded_r_pwm_s"]) for r in group]
            ys = [float(r[key]) if r.get(key) not in (None, "") else np.nan for r in group]
            ax.plot(xs, ys, marker="o", label=f"SIM_WIND_TURB={turb:g}")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("Commanded roll ramp rate r (PWM/s)")
    fig.suptitle("Probe 2: candidate authority demand metrics vs stick aggressiveness")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def write_probe2_doc(rows: list[dict[str, Any]]) -> Path:
    stage0 = [r for r in rows if r.get("stage") == "stage0"]
    main = [r for r in rows if r.get("stage") == "main" and not r.get("error")]
    main_sorted = sorted(main, key=lambda r: float(r["commanded_r_pwm_s"]))
    def seq(key: str, digits: int = 2) -> str:
        vals = []
        for r in main_sorted:
            value = r.get(key)
            vals.append("n/a" if value in (None, "") else f"{float(value):.{digits}f}")
        return ", ".join(vals)
    r_seq = ", ".join(str(int(float(r["commanded_r_pwm_s"]))) for r in main_sorted)
    clip_notes = []
    for r in main_sorted:
        requested = float(r.get("command_amplitude_pwm") or 0)
        actual = r.get("actual_rcin_peak_abs_pwm")
        des = r.get("actual_desroll_peak_abs_deg")
        requested_rate = float(r.get("commanded_r_pwm_s") or 0)
        actual_rate = r.get("actual_rcin_p95_rate_pwm_s")
        if actual not in (None, "") and float(actual) < requested * 0.9:
            clip_notes.append(f"r={float(r['commanded_r_pwm_s']):.0f}: RCIN peak {float(actual):.0f} < requested {requested:.0f}")
        if des not in (None, "") and float(des) < 0.8 * 30.0:
            clip_notes.append(f"r={float(r['commanded_r_pwm_s']):.0f}: DesRoll peak {float(des):.1f} deg")
        if actual_rate not in (None, "") and requested_rate > 0 and float(actual_rate) < requested_rate * 0.7:
            clip_notes.append(
                f"r={requested_rate:.0f}: RCIN p95 slew {float(actual_rate):.0f} PWM/s "
                f"(<70% of command)"
            )
    motor_vals = [float(r["min_motor_limit_margin_pwm"]) for r in main_sorted if r.get("min_motor_limit_margin_pwm") not in (None, "")]
    err_vals = [float(r["peak_att_error_deg"]) for r in main_sorted if r.get("peak_att_error_deg") not in (None, "")]
    drop_vals = [float(r["max_drop_m"]) for r in main_sorted if r.get("max_drop_m") not in (None, "")]
    def monotone_note(vals: list[float], lower_is_higher_demand: bool = False) -> str:
        if len(vals) < 2:
            return "样本不足"
        demand = [-v for v in vals] if lower_is_higher_demand else vals
        diffs = [demand[i + 1] - demand[i] for i in range(len(demand) - 1)]
        up = all(d >= -1.0e-9 for d in diffs)
        return "单调上升" if up else f"非单调，需求差分={', '.join(f'{d:.2f}' for d in diffs)}"
    stage0_lines = []
    for r in stage0:
        if r.get("error"):
            stage0_lines.append(f"- Stage-0 `{r.get('run_id')}` 运行卡住: `{r.get('error')}`。")
        else:
            stage0_lines.append(
                f"- Stage-0 `{r.get('run_id')}`: ANGLE_MAX={r.get('angle_max_cd')} cdeg, "
                f"turb={r.get('turbulence')}, r={r.get('commanded_r_pwm_s')} PWM/s, "
                f"max_drop={r.get('max_drop_m')}, touch={r.get('touch_or_near_ground')}, "
                f"sustained_att_error={r.get('sustained_att_error_gt20deg_1s')}。"
            )
            if not r.get("touch_or_near_ground") and not r.get("sustained_att_error_gt20deg_1s"):
                stage0_lines.append("- Stage-0 在这个合法低 ANGLE_MAX/高湍流/重模型组合下没有诱导出命令窗口内触地或持续大姿态误差。")
    lines = [
        "# Probe 2 Observations",
        "",
        "## 观察记录",
        "",
        *stage0_lines,
        f"- 主扫固定 `ALT_HOLD` RC override、`ANGLE_MAX=3000` cdeg、mass model `{rel('planc/config/minalt_models/mass_6_00.json')}`、`SIM_WIND_SPD=4 m/s`、`SIM_WIND_TURB=1.5`；r 序列为 {r_seq} PWM/s。",
        f"- 电机限幅余量序列: {seq('min_motor_limit_margin_pwm', 1)} PWM；这个量越小表示越贴近 1000/2000 PWM 限。",
        f"- 本轮主扫电机余量范围为 {min(motor_vals):.1f}-{max(motor_vals):.1f} PWM，没有进入贴限饱和段。",
        f"- 姿态跟踪误差峰值序列: {seq('peak_att_error_deg', 2)} deg。",
        f"- 最大掉高序列: {seq('max_drop_m', 2)} m。",
        f"- 单调性粗看: 电机贴限需求 {monotone_note(motor_vals, lower_is_higher_demand=True)}；姿态误差 {monotone_note(err_vals)}；掉高 {monotone_note(drop_vals)}。",
        f"- 命令核对: RCIN peak 序列 {seq('actual_rcin_peak_abs_pwm', 1)} PWM；RCIN p95 slew 序列 {seq('actual_rcin_p95_rate_pwm_s', 0)} PWM/s；DesRoll peak 序列 {seq('actual_desroll_peak_abs_deg', 2)} deg。",
        f"- 打杆削减/限速记录: {('; '.join(clip_notes) if clip_notes else '未看到 RCIN 峰值明显低于请求幅度；DesRoll 峰值接近 ANGLE_MAX 对应尺度。')}",
        "",
        "## 对方法设计的启示",
        "",
        "- `min_motor_limit_margin_pwm` 是最直接的峰值型权限需求候选；本轮未贴限且非单调幅度较小，若后续进入贴限段要把饱和信息保留下来而不是只拟合线性斜率。",
        "- `peak_att_error_deg` 对控制误差更敏感，可作为需求泛函的第二视角；若它随 r 有弹跳，应把湍流重复或模型噪声纳入采样策略。",
        "- `max_drop_m` 是外部后果量，适合和权限代理一起记录；它不应替代命令核对，因为掉高可能受高度控制器和清理降落阶段影响。",
        "- 若后续数据继续出现非单调或悬崖，BBS 的二分边界搜索应改为保留多点局部模型的 BO/active learning；若单调只在某个代理上干净，二分应绑定那个代理而不是绑定二值触地。",
    ]
    path = EXPLORE_ROOT / "probe2_observations.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def run_probe2() -> list[dict[str, Any]]:
    ensure_dirs()
    model_json = str(PLANC_ROOT / "config" / "minalt_models" / "mass_6_00.json")
    runs = []
    stage0 = run_authority_point(
        run_id="bbs_probe2_stage0_low_angle_high_turb_r4800",
        angle_max=1000,
        turbulence=2.5,
        wind_speed=4.0,
        r_pwm_s=4800.0,
        model_json=model_json,
        stage="stage0",
    )
    runs.append(stage0)
    write_json(DATA_DIR / "probe2_runs_partial.json", {"runs": runs})
    for r in [150.0, 300.0, 600.0, 1200.0, 2400.0, 4800.0]:
        print(f"RUN probe2 r={r}", flush=True)
        run = run_authority_point(
            run_id=f"bbs_probe2_main_turb15_r{int(r):04d}",
            angle_max=3000,
            turbulence=1.5,
            wind_speed=4.0,
            r_pwm_s=r,
            model_json=model_json,
            stage="main",
        )
        runs.append(run)
        write_json(DATA_DIR / "probe2_runs_partial.json", {"runs": runs})
    rows = probe2_rows(runs)
    write_csv(DATA_DIR / "probe2_authority_demand.csv", rows)
    plot_probe2(rows)
    write_probe2_doc(rows)
    write_json(DATA_DIR / "probe2_runs.json", {"runs": runs})
    return runs


def read_probe1_csv() -> list[dict[str, Any]]:
    path = DATA_DIR / "probe1_energy_margin.csv"
    rows = []
    with path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def fit_linear(xs: list[float], ys: list[float]) -> tuple[float, float]:
    coef = np.polyfit(np.array(xs, dtype=float), np.array(ys, dtype=float), 1)
    return float(coef[0]), float(coef[1])


def run_probe3() -> list[dict[str, Any]]:
    ensure_dirs()
    rows = read_probe1_csv()
    parsed = []
    for row in rows:
        parsed.append({
            "D_m": float(row["D_m"]),
            "margin_proxy_m": float(row["margin_proxy_m"]),
            "binary": 1 if str(row["landed_within_home_radius"]).lower() == "true" else 0,
            "final_home_distance_m": float(row["final_home_distance_m"]),
        })
    parsed.sort(key=lambda r: r["D_m"])
    train = [r for r in parsed if r["D_m"] <= 120.0]
    holdout = [r for r in parsed if r["D_m"] > 120.0]
    slope, intercept = fit_linear([r["D_m"] for r in train], [r["margin_proxy_m"] for r in train])
    train_binary_values = [r["binary"] for r in train]
    if len(set(train_binary_values)) == 1:
        naive_boundary = None
        naive_kind = "constant_near_segment_class"
    else:
        transitions = []
        ordered = sorted(train, key=lambda r: r["D_m"])
        for a, b in zip(ordered, ordered[1:]):
            if a["binary"] != b["binary"]:
                transitions.append((a["D_m"] + b["D_m"]) / 2.0)
        naive_boundary = transitions[-1] if transitions else None
        naive_kind = "nearest_transition_threshold"
    out_rows = []
    for row in parsed:
        pred_margin = slope * row["D_m"] + intercept
        pred_binary_margin = 1 if pred_margin >= 0 else 0
        if naive_boundary is None:
            pred_binary_naive = train_binary_values[-1]
            naive_score = float(abs(pred_binary_naive - row["binary"]))
        else:
            pred_binary_naive = 1 if row["D_m"] <= naive_boundary else 0
            naive_score = float(abs(pred_binary_naive - row["binary"]))
        out_rows.append({
            "split": "train" if row in train else "holdout",
            "D_m": row["D_m"],
            "observed_margin_proxy_m": row["margin_proxy_m"],
            "pred_margin_proxy_m": pred_margin,
            "abs_margin_error_m": abs(pred_margin - row["margin_proxy_m"]),
            "observed_binary": row["binary"],
            "margin_sign_pred_binary": pred_binary_margin,
            "margin_sign_binary_error": abs(pred_binary_margin - row["binary"]),
            "naive_classifier_kind": naive_kind,
            "naive_boundary_D_m": naive_boundary,
            "naive_pred_binary": pred_binary_naive,
            "naive_binary_error": naive_score,
            "final_home_distance_m": row["final_home_distance_m"],
        })
    write_csv(DATA_DIR / "probe3_extrapolation.csv", out_rows)
    plot_probe3(parsed, train, holdout, slope, intercept, naive_boundary, naive_kind)
    write_probe3_doc(out_rows, slope, intercept, naive_kind, naive_boundary)
    return out_rows


def plot_probe3(parsed: list[dict[str, Any]], train: list[dict[str, Any]], holdout: list[dict[str, Any]], slope: float, intercept: float, naive_boundary: float | None, naive_kind: str) -> Path:
    path = PLOT_DIR / "probe3_extrapolation.png"
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    ax.scatter([r["D_m"] for r in train], [r["margin_proxy_m"] for r in train], color="#2b6cb0", label="train margin")
    ax.scatter([r["D_m"] for r in holdout], [r["margin_proxy_m"] for r in holdout], color="#c53030", marker="X", s=80, label="holdout margin")
    xs = np.linspace(min(r["D_m"] for r in parsed) - 5, max(r["D_m"] for r in parsed) + 5, 100)
    ax.plot(xs, slope * xs + intercept, color="#2f855a", label="linear margin extrapolation")
    ax.axhline(0, color="#4a5568", linestyle="--", linewidth=1)
    if naive_boundary is not None:
        ax.axvline(naive_boundary, color="#805ad5", linestyle=":", label="naive 0/1 boundary")
    else:
        ax.text(0.03, 0.05, f"naive 0/1: {naive_kind}", transform=ax.transAxes, fontsize=9)
    ax.set_xlabel("Outbound distance D (m)")
    ax.set_ylabel("Margin proxy (m)")
    ax.set_title("Probe 3: margin scale extrapolation vs naive 0/1")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def write_probe3_doc(rows: list[dict[str, Any]], slope: float, intercept: float, naive_kind: str, naive_boundary: float | None) -> Path:
    holdout = [r for r in rows if r["split"] == "holdout"]
    margin_errors = [float(r["abs_margin_error_m"]) for r in holdout]
    sign_errors = [float(r["margin_sign_binary_error"]) for r in holdout]
    naive_errors = [float(r["naive_binary_error"]) for r in holdout]
    lines = [
        "# Probe 3 Extrapolation",
        "",
        "## 观察记录",
        "",
        f"- 近段训练点定义为 D<=120 m，远段留出点定义为 D>120 m；没有补跑额外飞行，直接使用探针 1 的 5 个点。",
        f"- 余量标度律使用线性 `margin(D)=a*D+b`，拟合得到 `a={slope:.4f}`, `b={intercept:.2f}`。",
        f"- 远段余量绝对误差: {', '.join(f'{v:.1f} m' for v in margin_errors) if margin_errors else 'n/a'}。",
        f"- 用余量符号转成二值后的远段错误: {', '.join(f'{v:.0f}' for v in sign_errors) if sign_errors else 'n/a'}。",
        f"- 朴素 0/1 外推形式: `{naive_kind}`，boundary={naive_boundary if naive_boundary is not None else 'n/a'}；远段二值错误: {', '.join(f'{v:.0f}' for v in naive_errors) if naive_errors else 'n/a'}。",
        "",
        "## 对方法设计的启示",
        "",
        "- 余量拟合保留了远段误差的连续尺度；即使二值预测相同，也能看到离边界的幅度误差。",
        "- 如果近段二值全是同一类，朴素分类器只能常值外推；这时训练点里缺少边界信息，比较重点应放在远段二值错误和余量误差是否同步恶化。",
        "- 后续若要支撑 §8 的外推论证，需要在便宜场景里明确规定近段如何覆盖边界附近，否则 0/1 基线会因为没有翻转样本而退化成常值。",
    ]
    path = EXPLORE_ROOT / "probe3_extrapolation.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def gather_file(path: Path) -> str:
    if not path.exists():
        return f"_缺失: `{rel(path)}`_\n"
    return path.read_text(encoding="utf-8")


def write_findings() -> Path:
    inv = gather_file(EXPLORE_ROOT / "telemetry_field_inventory.md")
    p1 = gather_file(EXPLORE_ROOT / "probe1_observations.md")
    p2 = gather_file(EXPLORE_ROOT / "probe2_observations.md")
    p3 = gather_file(EXPLORE_ROOT / "probe3_extrapolation.md")
    lines = [
        "# Overdraw BBS Explore Findings",
        "",
        "## 总观察",
        "",
        "- DataFlash 中可读字段决定了本轮可用的余量/需求代理；字段清点保留了取不到项，后续建模应从实际字段出发。",
        "- 能量探针把 `BAT.CurrTot`、短窗消耗率、触发时到家距离组合成连续 margin proxy；二值落点只作为叠加标记。",
        "- 控制权限探针把命令核对、峰值电机贴限、姿态误差和掉高分开记录，避免把“命令没有施加到”误读成“需求不高”。",
        "- 外推探针只在最便宜能量场景上比较近段拟合和远段留出；只记录误差尺度和设计信号。",
        "",
        "## Probe 0",
        "",
        inv,
        "",
        "## Probe 1",
        "",
        p1,
        "",
        "## Probe 2",
        "",
        p2,
        "",
        "## Probe 3",
        "",
        p3,
        "",
        "## Artifact Index",
        "",
        "- `planc/explore/data/probe1_energy_margin.csv`",
        "- `planc/explore/data/probe2_authority_demand.csv`",
        "- `planc/explore/data/probe3_extrapolation.csv`",
        "- `planc/explore/plots/probe1_margin_vs_D.png`",
        "- `planc/explore/plots/probe1_margin_timeseries_examples.png`",
        "- `planc/explore/plots/probe2_authority_demand_vs_r.png`",
        "- `planc/explore/plots/probe3_extrapolation.png`",
        "- Local-only DataFlash logs: `planc/logs/bbs_probe*.BIN` (ignored; CSV/PNG/markdown artifacts above are committed)",
    ]
    path = EXPLORE_ROOT / "overdraw_bbs_explore_findings.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", choices=["0", "1", "2", "3", "docs", "all"], default="all")
    args = parser.parse_args()
    ensure_dirs()
    if args.probe in {"0", "all"}:
        run_probe0()
    if args.probe in {"1", "all"}:
        run_probe1()
    if args.probe in {"2", "all"}:
        run_probe2()
    if args.probe in {"3", "all"}:
        run_probe3()
    if args.probe in {"docs", "all"}:
        write_findings()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
