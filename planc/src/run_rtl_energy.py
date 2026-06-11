from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import statistics
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from pymavlink import mavutil

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))
sys.path.insert(0, str(PLANC_ROOT / "analysis"))

from env_probe import probe_environment, write_env
from flight import (
    arm,
    command_takeoff,
    land_and_disarm,
    mode_name,
    request_streams,
    send_gcs_heartbeat,
    set_mode,
    wait_altitude,
    wait_position,
    wait_position_stable,
)
from injector import send_guided_velocity_local_ned, velocity_components_ned
from oracle import BAD_EVENT_NAMES, COPTER_MODES, ERROR_SUBSYSTEMS, EVENT_NAMES, horizontal_distance_m
from param_manager import ParamManager
from rtl_energy_plots import (
    plot_p_stratification,
    plot_premise,
    plot_result_field,
    plot_severity_heatmap,
    plot_train_test,
)
from sitl_runner import SitlRunner


MODE_REASON_NAMES = {
    0: "UNKNOWN",
    1: "RC_COMMAND",
    2: "GCS_COMMAND",
    3: "RADIO_FAILSAFE",
    4: "BATTERY_FAILSAFE",
    5: "GCS_FAILSAFE",
    6: "EKF_FAILSAFE",
    7: "GPS_GLITCH",
    8: "MISSION_END",
    9: "THROTTLE_LAND_ESCAPE",
    10: "FENCE_BREACHED",
    11: "TERRAIN_FAILSAFE",
    12: "BRAKE_TIMEOUT",
    17: "TERMINATE",
    19: "CRASH_FAILSAFE",
    25: "FAILSAFE",
    50: "DEADRECKON_FAILSAFE",
}

DIRTY_TEXT_MARKERS = (
    "radio failsafe",
    "gcs failsafe",
    "ekf failsafe",
    "fence breached",
    "outside fence",
    "fence failsafe",
    "terrain failsafe",
    "deadreckon",
    "crash",
    "arming checks failed",
)


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def rel(path: str | Path) -> str:
    try:
        return str(Path(path).resolve().relative_to(REPO_ROOT))
    except Exception:
        return str(path)


def config_with_model(config: dict[str, Any], model_name: str) -> dict[str, Any]:
    cfg = copy.deepcopy(config)
    cfg["sitl"]["model"] = str(config["sitl"].get("model", "quad"))
    cfg["experiment"]["model_name"] = model_name
    cfg["experiment"]["model_mass_kg"] = float(config["models"][model_name]["mass_kg"])
    return cfg


def controlled_params(config: dict[str, Any], overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    params = dict(config.get("baseline_params", {}))
    if overrides:
        params.update(overrides)
    return params


def model_params(config: dict[str, Any], model_name: str) -> dict[str, Any]:
    return dict(config.get("models", {}).get(model_name, {}).get("params", {}))


def point_key(layer: str, distance_m: float, wind_m_s: float) -> str:
    return f"{layer}_D{int(distance_m):03d}_W{int(wind_m_s):02d}"


def run_id_for(layer: str, distance_m: float, wind_m_s: float, rep_index: int) -> str:
    return f"rtl_{layer}_D{int(distance_m):03d}_W{int(wind_m_s):02d}_r{rep_index}"


def premise_run_id(kind: str) -> str:
    return f"rtl_premise_{kind}"


def _clean_param_records(records: list[dict[str, Any]]) -> bool:
    return bool(records) and all(bool(r.get("ok")) for r in records)


def _send_velocity(master, speed_m_s: float, bearing_deg: float) -> None:
    north, east = velocity_components_ned(speed_m_s, bearing_deg)
    send_guided_velocity_local_ned(master, north, east, 0.0)


def _send_zero(master) -> None:
    send_guided_velocity_local_ned(master, 0.0, 0.0, 0.0)


def _realtime_distance_from_home_m(pos_msg: Any, home: dict[str, Any]) -> float:
    lat = float(pos_msg.lat) / 1.0e7
    lon = float(pos_msg.lon) / 1.0e7
    return horizontal_distance_m(float(home["lat"]), float(home["lon"]), lat, lon)


def _stream_until_distance(
    master,
    config: dict[str, Any],
    *,
    speed_m_s: float,
    bearing_deg: float,
    target_distance_m: float,
    condition: str,
    max_s: float,
) -> dict[str, Any]:
    home = config["experiment"]["home"]
    stream_hz = float(config["experiment"].get("stream_hz", 10))
    dt = 1.0 / max(stream_hz, 1.0)
    start = time.time()
    next_send = 0.0
    max_distance = 0.0
    final_distance = None
    modes: list[dict[str, Any]] = []
    statustext: list[str] = []
    send_times: list[float] = []
    reached = False
    preempted_mode = None
    while time.time() - start < max_s:
        now = time.time()
        elapsed = now - start
        send_gcs_heartbeat(master)
        if now >= next_send:
            _send_velocity(master, speed_m_s, bearing_deg)
            send_times.append(elapsed)
            next_send = now + dt
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            statustext.append(str(getattr(msg, "text", "")))
        elif msg.get_type() == "HEARTBEAT":
            mode = mode_name(master, msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": elapsed, "mode": mode})
            if mode in {"RTL", "LAND", "BRAKE", "SMART_RTL"}:
                preempted_mode = mode
                break
        elif msg.get_type() == "GLOBAL_POSITION_INT":
            dist = _realtime_distance_from_home_m(msg, home)
            final_distance = dist
            max_distance = max(max_distance, dist)
            if condition == "away" and dist >= target_distance_m:
                reached = True
                break
            if condition == "home" and dist <= target_distance_m:
                reached = True
                break
    _send_zero(master)
    intervals = [send_times[i] - send_times[i - 1] for i in range(1, len(send_times))]
    return {
        "condition": condition,
        "target_distance_m": target_distance_m,
        "reached": reached,
        "preempted_mode": preempted_mode,
        "max_realtime_distance_m": max_distance,
        "final_realtime_distance_m": final_distance,
        "modes_seen": modes,
        "statustext": statustext,
        "send_timing": {
            "count": len(send_times),
            "target_hz": stream_hz,
            "mean_dt_s": statistics.fmean(intervals) if intervals else None,
            "std_dt_s": statistics.pstdev(intervals) if len(intervals) >= 2 else None,
            "min_dt_s": min(intervals) if intervals else None,
            "max_dt_s": max(intervals) if intervals else None,
        },
    }


def _hover_until_battery_failsafe(master, config: dict[str, Any], max_s: float) -> dict[str, Any]:
    stream_hz = float(config["experiment"].get("stream_hz", 10))
    dt = 1.0 / max(stream_hz, 1.0)
    start = time.time()
    next_send = 0.0
    modes: list[dict[str, Any]] = []
    statustext: list[str] = []
    battery_status: list[dict[str, Any]] = []
    action_mode = None
    while time.time() - start < max_s:
        now = time.time()
        elapsed = now - start
        send_gcs_heartbeat(master)
        if now >= next_send:
            _send_zero(master)
            next_send = now + dt
        msg = master.recv_match(
            type=["HEARTBEAT", "STATUSTEXT", "BATTERY_STATUS", "GLOBAL_POSITION_INT"],
            blocking=True,
            timeout=0.1,
        )
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            statustext.append(str(getattr(msg, "text", "")))
        elif msg.get_type() == "BATTERY_STATUS":
            battery_status.append({
                "wall_s": elapsed,
                "battery_remaining": getattr(msg, "battery_remaining", None),
                "current_battery": getattr(msg, "current_battery", None),
            })
        elif msg.get_type() == "HEARTBEAT":
            mode = mode_name(master, msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": elapsed, "mode": mode})
            if mode in {"RTL", "LAND", "BRAKE", "SMART_RTL"}:
                action_mode = mode
                break
    _send_zero(master)
    return {
        "action_seen": action_mode is not None,
        "action_mode": action_mode,
        "modes_seen": modes,
        "statustext": statustext,
        "battery_status": battery_status[-20:],
    }


def _observe_until_landed(master, config: dict[str, Any], max_s: float) -> dict[str, Any]:
    start = time.time()
    home = config["experiment"]["home"]
    modes: list[dict[str, Any]] = []
    statustext: list[str] = []
    final_distance = None
    final_alt = None
    min_home_distance = None
    disarmed = False
    low_alt_since = None
    while time.time() - start < max_s:
        elapsed = time.time() - start
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.2)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            statustext.append(str(getattr(msg, "text", "")))
        elif msg.get_type() == "HEARTBEAT":
            mode = mode_name(master, msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": elapsed, "mode": mode})
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                disarmed = True
                break
        elif msg.get_type() == "GLOBAL_POSITION_INT":
            final_alt = float(getattr(msg, "relative_alt", 0.0)) / 1000.0
            final_distance = _realtime_distance_from_home_m(msg, home)
            min_home_distance = final_distance if min_home_distance is None else min(min_home_distance, final_distance)
            if final_alt < 0.8:
                low_alt_since = low_alt_since or time.time()
                if time.time() - low_alt_since >= 3.0:
                    break
            else:
                low_alt_since = None
    return {
        "disarmed": disarmed,
        "modes_seen": modes,
        "statustext": statustext,
        "final_realtime_distance_m": final_distance,
        "final_realtime_alt_m": final_alt,
        "min_realtime_home_distance_m": min_home_distance,
    }


def prepare_flight(master, config: dict[str, Any]) -> None:
    request_streams(master, rate_hz=10)
    wait_position(master, timeout_s=30.0)
    wait_position_stable(master)
    set_mode(master, "GUIDED")
    arm(master)
    alt_m = float(config["experiment"]["takeoff_alt_m"])
    command_takeoff(master, alt_m)
    wait_altitude(master, alt_m)
    set_mode(master, "GUIDED")


def run_rtl_energy_flight(master, config: dict[str, Any], distance_m: float) -> dict[str, Any]:
    prepare_flight(master, config)
    exp = config["experiment"]
    speed = float(exp["cruise_speed_m_s"])
    bearing = float(exp["target_bearing_deg"])
    tol = float(exp["d_reached_tolerance_m"])
    outbound_max_s = max(35.0, distance_m / max(speed, 0.1) + 28.0)
    outbound = _stream_until_distance(
        master,
        config,
        speed_m_s=speed,
        bearing_deg=bearing,
        target_distance_m=max(0.0, distance_m - tol),
        condition="away",
        max_s=outbound_max_s,
    )
    hover = None
    if outbound["reached"]:
        hover = _hover_until_battery_failsafe(
            master,
            config,
            max_s=float(exp["max_wait_for_low_failsafe_s"]),
        )
    observe = _observe_until_landed(master, config, max_s=float(exp["max_return_or_land_s"]))
    cleanup_forced = False
    if not observe.get("disarmed") and observe.get("final_realtime_alt_m") not in (None, ""):
        cleanup_forced = True
        try:
            land_and_disarm(master, timeout_s=35.0)
        except Exception:
            pass
    return {
        "motion": "out_to_D_then_battery_RTL",
        "target_distance_m": float(distance_m),
        "cruise_speed_m_s": speed,
        "bearing_deg": bearing,
        "outbound": outbound,
        "hover_until_failsafe": hover,
        "post_failsafe_observation": observe,
        "cleanup_forced": cleanup_forced,
    }


def run_premise_flight(master, config: dict[str, Any], distance_m: float) -> dict[str, Any]:
    prepare_flight(master, config)
    exp = config["experiment"]
    speed = float(exp["cruise_speed_m_s"])
    bearing = float(exp["target_bearing_deg"])
    outbound = _stream_until_distance(
        master,
        config,
        speed_m_s=speed,
        bearing_deg=bearing,
        target_distance_m=distance_m,
        condition="away",
        max_s=max(35.0, distance_m / max(speed, 0.1) + 25.0),
    )
    inbound = _stream_until_distance(
        master,
        config,
        speed_m_s=speed,
        bearing_deg=(bearing + 180.0) % 360.0,
        target_distance_m=float(exp["home_radius_m"]),
        condition="home",
        max_s=max(35.0, distance_m / max(speed, 0.1) + 35.0),
    )
    try:
        land_and_disarm(master, timeout_s=45.0)
        cleanup_error = None
    except Exception as exc:
        cleanup_error = repr(exc)
    return {
        "motion": "premise_out_and_back",
        "target_distance_m": float(distance_m),
        "cruise_speed_m_s": speed,
        "bearing_deg": bearing,
        "outbound": outbound,
        "inbound": inbound,
        "cleanup_error": cleanup_error,
    }


def _field(data: dict[str, Any], *names: str) -> Any:
    for name in names:
        if name in data:
            return data[name]
    return None


def _time_s(data: dict[str, Any]) -> float | None:
    if "TimeUS" in data:
        return float(data["TimeUS"]) / 1.0e6
    if "TimeMS" in data:
        return float(data["TimeMS"]) / 1000.0
    return None


def _latlon(value: Any) -> float | None:
    if value is None:
        return None
    value_f = float(value)
    if abs(value_f) > 1000:
        return value_f / 1.0e7
    return value_f


def _mode_name_from_data(data: dict[str, Any]) -> str:
    mode = _field(data, "Mode", "ModeNum")
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


def _annotate_modes(rows: list[dict[str, Any]], modes: list[dict[str, Any]]) -> None:
    ordered = sorted(modes, key=lambda m: float(m["time_s"]))
    idx = 0
    current = ""
    for row in rows:
        t = float(row["time_s"])
        while idx < len(ordered) and float(ordered[idx]["time_s"]) <= t:
            current = str(ordered[idx]["mode"])
            idx += 1
        row["mode"] = current


def _nearest(rows: list[dict[str, Any]], t_s: float | None) -> dict[str, Any] | None:
    if not rows or t_s is None:
        return None
    return min(rows, key=lambda r: abs(float(r["time_s"]) - t_s))


def _first_time_distance(rows: list[dict[str, Any]], distance_m: float, before_s: float | None = None) -> float | None:
    for row in rows:
        if before_s is not None and float(row["time_s"]) > before_s:
            return None
        if float(row["distance_m"]) >= distance_m:
            return float(row["time_s"])
    return None


def _first_time_home(rows: list[dict[str, Any]], home_radius_m: float, after_s: float | None = None) -> float | None:
    for row in rows:
        if after_s is not None and float(row["time_s"]) < after_s:
            continue
        if float(row["distance_m"]) <= home_radius_m:
            return float(row["time_s"])
    return None


def _remaining_mAh(bat_row: dict[str, Any] | None, pack_capacity_mAh: float) -> float | None:
    if bat_row is None:
        return None
    curr = bat_row.get("currtot_mAh")
    if curr is None:
        return None
    return float(pack_capacity_mAh) - float(curr)


def _battery_at(rows: list[dict[str, Any]], t_s: float | None, pack_capacity_mAh: float) -> dict[str, Any] | None:
    row = _nearest(rows, t_s)
    if row is None:
        return None
    out = dict(row)
    out["remaining_mAh"] = _remaining_mAh(row, pack_capacity_mAh)
    return out


def parse_energy_dataflash(
    *,
    bin_path: Path,
    csv_path: Path,
    home: dict[str, Any],
    params: dict[str, Any],
    run_kind: str,
    target_distance_m: float | None,
    cruise_speed_m_s: float,
    target_bearing_deg: float,
    home_radius_m: float,
    d_tolerance_m: float,
    speed_tolerance_m_s: float,
    speed_audit_min_distance_m: float,
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    pos_rows: list[dict[str, Any]] = []
    gps_rows: list[dict[str, Any]] = []
    bat_rows: list[dict[str, Any]] = []
    xkf_velocity_rows: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    event_records: list[dict[str, Any]] = []
    err_records: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    parm_rows: list[dict[str, Any]] = []
    start_time: float | None = None
    home_lat = float(home["lat"])
    home_lon = float(home["lon"])

    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        data = msg.to_dict()
        ts = _time_s(data)
        if ts is None:
            continue
        if start_time is None:
            start_time = ts
        rel_t = ts - start_time
        mtype = msg.get_type()
        if mtype == "PARM":
            parm_rows.append({"time_s": rel_t, "name": _field(data, "Name"), "value": _field(data, "Value")})
        elif mtype == "MODE":
            reason = _field(data, "Rsn", "Reason")
            reason_i = int(reason) if reason is not None else None
            modes.append({
                "time_s": rel_t,
                "mode": _mode_name_from_data(data),
                "reason": reason_i,
                "reason_name": MODE_REASON_NAMES.get(reason_i, str(reason_i)),
                "raw": data,
            })
        elif mtype == "ERR":
            subsys = int(_field(data, "Subsys", "SubSystem") or -1)
            ecode = int(_field(data, "ECode", "Code") or 0)
            err_records.append({
                "time_s": rel_t,
                "subsys": subsys,
                "subsys_name": ERROR_SUBSYSTEMS.get(subsys, str(subsys)),
                "ecode": ecode,
                "raw": data,
            })
        elif mtype == "EV":
            event_id = int(_field(data, "Id") or -1)
            event_records.append({
                "time_s": rel_t,
                "id": event_id,
                "name": EVENT_NAMES.get(event_id, str(event_id)),
                "raw": data,
            })
        elif mtype in {"MSG", "STAT"}:
            messages.append({
                "time_s": rel_t,
                "text": str(_field(data, "Message", "Msg", "Text") or ""),
                "raw": data,
            })
        elif mtype == "BAT":
            instance = int(_field(data, "Instance") or 0)
            if instance == 0:
                currtot = _field(data, "CurrTot", "CurrentTot", "Consumed")
                bat_rows.append({
                    "time_s": rel_t,
                    "volt": _field(data, "Volt", "VoltR"),
                    "volt_resting": _field(data, "VoltR"),
                    "curr": _field(data, "Curr"),
                    "currtot_mAh": float(currtot) if currtot is not None else None,
                    "enrgtot_Wh": _field(data, "EnrgTot"),
                    "rem_pct": _field(data, "RemPct"),
                    "raw": data,
                })
        elif mtype == "XKF1":
            core = _field(data, "C")
            if core is None or int(core) == 0:
                vn = float(_field(data, "VN") or 0.0)
                ve = float(_field(data, "VE") or 0.0)
                bearing = math.radians(float(target_bearing_deg))
                forward = vn * math.cos(bearing) + ve * math.sin(bearing)
                xkf_velocity_rows.append({
                    "time_s": rel_t,
                    "vn_m_s": vn,
                    "ve_m_s": ve,
                    "ground_speed_m_s": math.hypot(vn, ve),
                    "forward_speed_m_s": forward,
                })

        if mtype in {"POS", "GPS", "GPS2"}:
            lat = _latlon(_field(data, "Lat", "latitude"))
            lon = _latlon(_field(data, "Lng", "Lon", "longitude"))
            if lat is None or lon is None or abs(lat) < 1.0e-9 or abs(lon) < 1.0e-9:
                continue
            if mtype.startswith("GPS"):
                status = _field(data, "Status")
                if status is not None and float(status) < 3:
                    continue
            alt = _field(data, "Alt", "RelHomeAlt", "RAlt")
            row = {
                "time_s": rel_t,
                "source": mtype,
                "lat": lat,
                "lon": lon,
                "alt": float(alt) if alt is not None else "",
                "distance_m": horizontal_distance_m(home_lat, home_lon, lat, lon),
                "mode": "",
            }
            if mtype == "POS":
                pos_rows.append(row)
            else:
                gps_rows.append(row)

    rows = sorted(pos_rows if len(pos_rows) >= 10 else gps_rows, key=lambda r: float(r["time_s"]))
    _annotate_modes(rows, modes)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["time_s", "source", "lat", "lon", "alt", "distance_m", "mode"],
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)

    pack_capacity = float(params.get("BATT_CAPACITY", 0.0))
    low_mah = float(params.get("BATT_LOW_MAH", 0.0))
    crt_mah = float(params.get("BATT_CRT_MAH", 0.0))
    batt_tol = max(8.0, pack_capacity * 0.04)
    battery_reason = int(4)
    low_modes = [
        m for m in modes
        if m["mode"] == "RTL" and int(m.get("reason") or -1) == battery_reason
    ]
    low_mode = low_modes[0] if low_modes else None
    low_time = float(low_mode["time_s"]) if low_mode else None
    critical_modes = [
        m for m in modes
        if m["mode"] == "LAND"
        and int(m.get("reason") or -1) == battery_reason
        and (low_time is None or float(m["time_s"]) >= low_time)
    ]
    critical_mode = critical_modes[0] if critical_modes else None
    critical_time = float(critical_mode["time_s"]) if critical_mode else None
    low_bat = _battery_at(bat_rows, low_time, pack_capacity)
    critical_bat = _battery_at(bat_rows, critical_time, pack_capacity)
    low_remaining = None if low_bat is None else low_bat.get("remaining_mAh")
    critical_remaining = None if critical_bat is None else critical_bat.get("remaining_mAh")

    target_distance_for_audit = None if target_distance_m is None else max(0.0, float(target_distance_m) - float(d_tolerance_m))
    d_reached_time = None
    if target_distance_for_audit is not None:
        d_reached_time = _first_time_distance(rows, target_distance_for_audit, before_s=low_time)
    d_reached = target_distance_for_audit is None or d_reached_time is not None
    distance_at_low = None
    nearest_low_pos = _nearest(rows, low_time)
    if nearest_low_pos is not None:
        distance_at_low = float(nearest_low_pos["distance_m"])
    max_distance = max((float(r["distance_m"]) for r in rows), default=0.0)
    final_row = rows[-1] if rows else None
    final_distance = float(final_row["distance_m"]) if final_row else None
    final_alt = final_row.get("alt") if final_row else None
    home_return_time = _first_time_home(rows, home_radius_m, after_s=low_time)
    returned_home_after_low = home_return_time is not None
    safe_binary = bool(low_time is not None and returned_home_after_low)
    unsafe_binary = bool(low_time is not None and not returned_home_after_low and final_distance is not None and final_distance > home_radius_m)

    other_errors = [
        e for e in err_records
        if int(e["ecode"]) != 0 and int(e["subsys"]) != 6
    ]
    battery_errors = [
        e for e in err_records
        if int(e["ecode"]) != 0 and int(e["subsys"]) == 6
    ]
    bad_events = [e for e in event_records if e.get("name") in BAD_EVENT_NAMES]
    dirty_messages = []
    battery_messages = []
    for msg in messages:
        text = str(msg.get("text", ""))
        low_text = text.lower()
        if "battery" in low_text:
            battery_messages.append(msg)
        if any(marker in low_text for marker in DIRTY_TEXT_MARKERS):
            if "battery failsafe" in low_text:
                continue
            dirty_messages.append(msg)

    low_action_ok = bool(
        low_mode
        and low_mah > 0
        and low_remaining is not None
        and float(low_remaining) <= low_mah + batt_tol
    )
    critical_action_ok = True
    if critical_mode is not None:
        critical_action_ok = bool(
            crt_mah > 0
            and critical_remaining is not None
            and float(critical_remaining) <= crt_mah + batt_tol
        )

    speed_audit = None
    if target_distance_m is not None:
        audit_max_dist = max(speed_audit_min_distance_m + 1.0, float(target_distance_m) - float(d_tolerance_m))
        audit_rows = []
        for vel in xkf_velocity_rows:
            t = float(vel["time_s"])
            if d_reached_time is not None and t > d_reached_time:
                continue
            pos = _nearest(rows, t)
            if pos is None:
                continue
            dist = float(pos["distance_m"])
            if dist < speed_audit_min_distance_m or dist > audit_max_dist:
                continue
            audit_rows.append({**vel, "distance_m": dist})
        speeds = [float(r["forward_speed_m_s"]) for r in audit_rows]
        median = statistics.median(speeds) if speeds else None
        p95 = np.percentile(np.array(speeds, dtype=float), 95) if speeds else None
        speed_audit = {
            "samples": len(audit_rows),
            "commanded_speed_m_s": float(cruise_speed_m_s),
            "median_forward_speed_m_s": None if median is None else float(median),
            "p95_forward_speed_m_s": None if p95 is None else float(p95),
            "median_error_m_s": None if median is None else float(median) - float(cruise_speed_m_s),
            "p95_error_m_s": None if p95 is None else float(p95) - float(cruise_speed_m_s),
            "within_tolerance": bool(median is not None and abs(float(median) - float(cruise_speed_m_s)) <= speed_tolerance_m_s),
        }

    consumed_mah = None
    voltage_drop = None
    voltage_drop_rate = None
    if bat_rows:
        first = bat_rows[0]
        last = bat_rows[-1]
        if first.get("currtot_mAh") is not None and last.get("currtot_mAh") is not None:
            consumed_mah = float(last["currtot_mAh"]) - float(first["currtot_mAh"])
        if first.get("volt") is not None and last.get("volt") is not None:
            voltage_drop = float(first["volt"]) - float(last["volt"])
            duration = max(1.0e-6, float(last["time_s"]) - float(first["time_s"]))
            voltage_drop_rate = voltage_drop / duration

    violations = []
    if run_kind == "rtl_scan":
        if not d_reached:
            violations.append("D_not_reached_before_low_failsafe")
        if speed_audit and not speed_audit.get("within_tolerance"):
            violations.append("cruise_speed_not_reached")
        if low_time is None:
            violations.append("low_battery_RTL_missing")
        elif not low_action_ok:
            violations.append("low_battery_RTL_threshold_mismatch")
        if critical_mode is not None and not critical_action_ok:
            violations.append("critical_battery_LAND_threshold_mismatch")
    if other_errors:
        violations.append("other_ERR_subsystems")
    if bad_events:
        violations.append("bad_EV_events")
    if dirty_messages:
        violations.append("dirty_STATUSTEXT")

    contract_clean = not violations
    result = {
        "bin_path": str(bin_path),
        "csv_path": str(csv_path),
        "position_source": rows[0]["source"] if rows else None,
        "samples": len(rows),
        "battery_samples": len(bat_rows),
        "modes": modes,
        "event_records": event_records,
        "err_records": err_records,
        "battery_errors": battery_errors,
        "other_errors": other_errors,
        "bad_events": bad_events,
        "messages_count": len(messages),
        "battery_messages": battery_messages[:20],
        "dirty_messages": dirty_messages[:20],
        "parm_rows_count": len(parm_rows),
        "max_distance_m": max_distance,
        "final_distance_m": final_distance,
        "final_alt": final_alt,
        "home_radius_m": float(home_radius_m),
        "d_reached": d_reached,
        "d_reached_time_s": d_reached_time,
        "distance_at_low_failsafe_m": distance_at_low,
        "low_mode": low_mode,
        "low_time_s": low_time,
        "low_battery_at_action": low_bat,
        "low_remaining_mAh": low_remaining,
        "low_action_ok": low_action_ok,
        "critical_mode": critical_mode,
        "critical_time_s": critical_time,
        "critical_battery_at_action": critical_bat,
        "critical_remaining_mAh": critical_remaining,
        "critical_action_ok": critical_action_ok,
        "returned_home_after_low": returned_home_after_low,
        "home_return_time_s": home_return_time,
        "safe_binary": safe_binary,
        "unsafe_binary": unsafe_binary,
        "severity_final_distance_m": final_distance,
        "contract_clean": contract_clean,
        "contract_violations": violations,
        "speed_audit": speed_audit,
        "consumed_mah": consumed_mah,
        "voltage_drop_v": voltage_drop,
        "voltage_drop_rate_v_s": voltage_drop_rate,
    }
    sidecar = csv_path.with_suffix(".oracle.json")
    sidecar.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def classify_run(run: dict[str, Any]) -> None:
    if run.get("error"):
        run["label"] = "blocked"
        run["safe"] = None
        run["unsafe"] = None
        return
    if not _clean_param_records(run.get("param_readbacks", [])):
        run.setdefault("contract_violations", []).append("parameter_readback_failed")
        run["contract_clean"] = False
    clean = bool(run.get("contract_clean"))
    safe = bool(run.get("safe_binary"))
    unsafe = bool(run.get("unsafe_binary"))
    run["safe"] = safe
    run["unsafe"] = unsafe
    if clean and unsafe:
        run["label"] = "clean_unsafe"
    elif clean and safe:
        run["label"] = "clean_safe"
    elif clean:
        run["label"] = "clean_safe" if (run.get("final_distance_m") or 9999) <= run.get("home_radius_m", 10.0) else "contract_violated"
    else:
        run["label"] = "contract_violated"


def connectivity_probe(config: dict[str, Any], env: dict[str, Any]) -> dict[str, Any]:
    cfg = config_with_model(config, "nominal")
    runner = SitlRunner(cfg, REPO_ROOT)
    run_id = "rtl_energy_connectivity_probe"
    try:
        runner.start(run_id)
        master = runner.connect(timeout_s=30)
        pm = ParamManager(master)
        sim_speedup_before = pm.read("SIM_SPEEDUP")
        pm.set_and_readback("SIM_SPEEDUP", float(config["experiment"]["speedup"]))
        probe = {
            "ok": True,
            "heartbeat_target_system": master.target_system,
            "heartbeat_target_component": master.target_component,
            "connection": runner.connection_string,
            "sim_speedup_before": sim_speedup_before,
            "param_records": pm.records,
        }
        try:
            master.close()
        except Exception:
            pass
        env["connectivity_probe"] = probe
        return probe
    except Exception as exc:
        probe = {"ok": False, "error": repr(exc), "traceback": traceback.format_exc()}
        env["connectivity_probe"] = probe
        return probe
    finally:
        runner.stop()


def run_one_premise(
    config: dict[str, Any],
    *,
    kind: str,
    label: str,
    model_name: str,
    wind_m_s: float,
) -> dict[str, Any]:
    cfg = config_with_model(config, model_name)
    run_id = premise_run_id(kind)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "kind": kind,
        "label": label,
        "model_name": model_name,
        "model_mass_kg": float(config["models"][model_name]["mass_kg"]),
        "wind_m_s": float(wind_m_s),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        overrides = {
            "SIM_WIND_SPD": float(wind_m_s),
            "BATT_LOW_MAH": 0,
            "BATT_CRT_MAH": 0,
            "BATT_FS_LOW_ACT": 0,
            "BATT_FS_CRT_ACT": 0,
        }
        overrides.update(model_params(config, model_name))
        params = controlled_params(config, overrides)
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result["params_requested"] = params
        result["param_snapshot"] = snapshot
        result["param_records_path"] = str(param_path)
        result["param_readbacks"] = pm.records
        result["flight"] = run_premise_flight(master, cfg, float(config["experiment"]["premise_distance_m"]))
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
        csv_path = PLANC_ROOT / "logs" / f"{run_id}_parsed.csv"
        parsed = parse_energy_dataflash(
            bin_path=bin_path,
            csv_path=csv_path,
            home=cfg["experiment"]["home"],
            params=params,
            run_kind="premise",
            target_distance_m=float(config["experiment"]["premise_distance_m"]),
            cruise_speed_m_s=float(config["experiment"]["cruise_speed_m_s"]),
            target_bearing_deg=float(config["experiment"]["target_bearing_deg"]),
            home_radius_m=float(config["experiment"]["home_radius_m"]),
            d_tolerance_m=float(config["experiment"]["d_reached_tolerance_m"]),
            speed_tolerance_m_s=float(config["experiment"]["speed_audit_tolerance_m_s"]),
            speed_audit_min_distance_m=float(config["experiment"]["speed_audit_min_distance_m"]),
        )
        result.update(parsed)
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


def run_one_grid(
    config: dict[str, Any],
    *,
    layer: str,
    distance_m: float,
    wind_m_s: float,
    rep_index: int,
    roles: list[str],
) -> dict[str, Any]:
    cfg = config_with_model(config, "nominal")
    run_id = run_id_for(layer, distance_m, wind_m_s, rep_index)
    runner = SitlRunner(cfg, REPO_ROOT)
    batt_low = float(config["sweep"]["p_layers"][layer]["BATT_LOW_MAH"])
    result: dict[str, Any] = {
        "run_id": run_id,
        "point_key": point_key(layer, distance_m, wind_m_s),
        "layer": layer,
        "rep_index": rep_index,
        "roles": roles,
        "distance_m": float(distance_m),
        "wind_m_s": float(wind_m_s),
        "batt_low_mah": batt_low,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        params = controlled_params(config, {
            "SIM_WIND_SPD": float(wind_m_s),
            "BATT_LOW_MAH": batt_low,
        })
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result["params_requested"] = params
        result["param_snapshot"] = snapshot
        result["param_records_path"] = str(param_path)
        result["param_readbacks"] = pm.records
        result["flight"] = run_rtl_energy_flight(master, cfg, distance_m)
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
            classify_run(result)
            return result
        result["bin_path"] = str(bin_path)
        csv_path = PLANC_ROOT / "logs" / f"{run_id}_parsed.csv"
        parsed = parse_energy_dataflash(
            bin_path=bin_path,
            csv_path=csv_path,
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
        classify_run(result)
        return result
    except Exception as exc:
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        classify_run(result)
        return result
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def update_roles(run: dict[str, Any], roles: list[str]) -> None:
    current = list(run.get("roles", []))
    for role in roles:
        if role not in current:
            current.append(role)
    run["roles"] = current


def run_or_reuse_premise(
    config: dict[str, Any],
    runs: list[dict[str, Any]],
    partial_path: Path,
    *,
    kind: str,
    label: str,
    model_name: str,
    wind_m_s: float,
) -> dict[str, Any]:
    run_id = premise_run_id(kind)
    for run in runs:
        if run.get("run_id") == run_id:
            return run
    print(f"RUN {run_id} model={model_name} wind={wind_m_s}", flush=True)
    run = run_one_premise(config, kind=kind, label=label, model_name=model_name, wind_m_s=wind_m_s)
    runs.append(run)
    write_json(partial_path, {"premise_runs": runs})
    return run


def run_or_reuse_grid(
    config: dict[str, Any],
    runs: list[dict[str, Any]],
    partial_path: Path,
    *,
    layer: str,
    distance_m: float,
    wind_m_s: float,
    rep_index: int,
    roles: list[str],
) -> dict[str, Any]:
    run_id = run_id_for(layer, distance_m, wind_m_s, rep_index)
    for run in runs:
        if run.get("run_id") == run_id:
            update_roles(run, roles)
            classify_run(run)
            write_json(partial_path, {"grid_runs": runs})
            return run
    print(f"RUN {run_id} layer={layer} D={distance_m} wind={wind_m_s} roles={','.join(roles)}", flush=True)
    run = run_one_grid(
        config,
        layer=layer,
        distance_m=distance_m,
        wind_m_s=wind_m_s,
        rep_index=rep_index,
        roles=roles,
    )
    runs.append(run)
    write_json(partial_path, {"grid_runs": runs})
    return run


def premise_summary(premise_runs: list[dict[str, Any]]) -> dict[str, Any]:
    by_kind = {r.get("kind"): r for r in premise_runs}
    nominal = by_kind.get("nominal_no_wind")
    windy = by_kind.get("nominal_headwind")
    heavy = by_kind.get("heavy_no_wind")
    checks = []
    hard_checks = []
    if nominal and windy and not nominal.get("error") and not windy.get("error"):
        wind_consumed = {
            "name": "wind_response_consumed_mAh",
            "baseline": nominal.get("consumed_mah"),
            "stressed": windy.get("consumed_mah"),
            "ok": windy.get("consumed_mah") is not None and nominal.get("consumed_mah") is not None and float(windy["consumed_mah"]) > float(nominal["consumed_mah"]),
            "hard_gate": True,
        }
        checks.append(wind_consumed)
        hard_checks.append(wind_consumed)
        checks.append({
            "name": "wind_response_voltage_drop_rate",
            "baseline": nominal.get("voltage_drop_rate_v_s"),
            "stressed": windy.get("voltage_drop_rate_v_s"),
            "ok": windy.get("voltage_drop_rate_v_s") is not None and nominal.get("voltage_drop_rate_v_s") is not None and float(windy["voltage_drop_rate_v_s"]) > float(nominal["voltage_drop_rate_v_s"]),
            "hard_gate": False,
        })
    if nominal and heavy and not nominal.get("error") and not heavy.get("error"):
        mass_consumed = {
            "name": "mass_response_consumed_mAh",
            "baseline": nominal.get("consumed_mah"),
            "stressed": heavy.get("consumed_mah"),
            "ok": heavy.get("consumed_mah") is not None and nominal.get("consumed_mah") is not None and float(heavy["consumed_mah"]) > float(nominal["consumed_mah"]),
            "hard_gate": True,
        }
        checks.append(mass_consumed)
        hard_checks.append(mass_consumed)
        checks.append({
            "name": "mass_response_voltage_drop_rate",
            "baseline": nominal.get("voltage_drop_rate_v_s"),
            "stressed": heavy.get("voltage_drop_rate_v_s"),
            "ok": heavy.get("voltage_drop_rate_v_s") is not None and nominal.get("voltage_drop_rate_v_s") is not None and float(heavy["voltage_drop_rate_v_s"]) > float(nominal["voltage_drop_rate_v_s"]),
            "hard_gate": False,
        })
    ok = len(hard_checks) == 2 and all(bool(c.get("ok")) for c in hard_checks)
    return {
        "satisfied": ok,
        "checks": checks,
        "runs": premise_runs,
        "reason": "wind and mass monotonically increased consumed mAh" if ok else "SITL energy model did not show the required monotonic wind/mass consumption response",
        "voltage_caveat": "BAT voltage stayed constant in this SITL binary; the premise gate uses consumed mAh, which is the same signal used by the configured capacity failsafes.",
    }


def group_runs(runs: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for run in runs:
        grouped.setdefault(str(run.get("point_key")), []).append(run)
    for entries in grouped.values():
        entries.sort(key=lambda r: int(r.get("rep_index", 0)))
    return grouped


def aggregate_point(runs: list[dict[str, Any]]) -> dict[str, Any]:
    sample = runs[0]
    complete = [r for r in runs if not r.get("error")]
    labels = [str(r.get("label", "blocked")) for r in complete]
    clean_complete = [r for r in complete if r.get("label") in {"clean_safe", "clean_unsafe"}]
    severity_values = [
        float(r["severity_final_distance_m"])
        for r in complete
        if r.get("severity_final_distance_m") is not None
    ]
    distance_at_low = [
        float(r["distance_at_low_failsafe_m"])
        for r in complete
        if r.get("distance_at_low_failsafe_m") is not None
    ]
    label = "blocked"
    stable_binary = None
    boundary_flip = False
    if complete:
        if any(l == "contract_violated" for l in labels):
            label = "contract_violated"
        elif labels and all(l == labels[0] for l in labels):
            label = labels[0]
            stable_binary = True
        elif labels:
            boundary_flip = True
            stable_binary = False
            label = max(set(labels), key=labels.count)
    return {
        "point_key": sample.get("point_key"),
        "layer": sample.get("layer"),
        "distance_m": float(sample.get("distance_m")),
        "wind_m_s": float(sample.get("wind_m_s")),
        "batt_low_mah": float(sample.get("batt_low_mah")),
        "run_ids": [r.get("run_id") for r in runs],
        "repetitions": len(runs),
        "completed_repetitions": len(complete),
        "labels": labels,
        "label": label,
        "stable_binary": stable_binary,
        "boundary_flip": boundary_flip,
        "contract_clean_all": bool(clean_complete) and len(clean_complete) == len(complete),
        "safe_any": any(bool(r.get("safe_binary")) for r in complete),
        "unsafe_any": any(bool(r.get("unsafe_binary")) for r in complete),
        "severity_final_distance_m": statistics.fmean(severity_values) if severity_values else None,
        "severity_spread_m": max(severity_values) - min(severity_values) if len(severity_values) >= 2 else 0.0 if severity_values else None,
        "distance_at_low_failsafe_mean_m": statistics.fmean(distance_at_low) if distance_at_low else None,
        "contract_violations": sorted({v for r in complete for v in r.get("contract_violations", [])}),
        "errors": [r.get("error") for r in runs if r.get("error")],
    }


def aggregate_layer(runs: list[dict[str, Any]], layer: str, distances: list[float], winds: list[float]) -> list[dict[str, Any]]:
    grouped = group_runs([r for r in runs if r.get("layer") == layer])
    points = []
    for d in distances:
        for w in winds:
            key = point_key(layer, d, w)
            if key in grouped:
                points.append(aggregate_point(grouped[key]))
    return points


def zone_counts(points: list[dict[str, Any]]) -> dict[str, int]:
    counts = {"clean_safe": 0, "clean_unsafe": 0, "contract_violated": 0, "blocked": 0}
    for point in points:
        label = str(point.get("label", "blocked"))
        counts[label] = counts.get(label, 0) + 1
    return counts


def find_near_boundary(points: list[dict[str, Any]], distances: list[float], winds: list[float]) -> list[dict[str, Any]]:
    index = {(float(p["distance_m"]), float(p["wind_m_s"])): p for p in points}
    near: dict[str, dict[str, Any]] = {}
    clean_labels = {"clean_safe", "clean_unsafe"}
    for d in distances:
        for w in winds:
            p = index.get((float(d), float(w)))
            if not p or p.get("label") not in clean_labels:
                continue
            neighbors = []
            di = distances.index(d)
            wi = winds.index(w)
            for nd_i, nw_i in ((di - 1, wi), (di + 1, wi), (di, wi - 1), (di, wi + 1)):
                if 0 <= nd_i < len(distances) and 0 <= nw_i < len(winds):
                    neighbors.append(index.get((float(distances[nd_i]), float(winds[nw_i]))))
            if any(n and n.get("label") in clean_labels and n.get("label") != p.get("label") for n in neighbors):
                near[str(p["point_key"])] = p
    if near:
        return list(near.values())
    clean_unsafe = [p for p in points if p.get("label") == "clean_unsafe"]
    clean_safe = [p for p in points if p.get("label") == "clean_safe"]
    fallback = []
    if clean_unsafe:
        fallback.append(min(clean_unsafe, key=lambda p: float(p.get("severity_final_distance_m") or 9999)))
    if clean_safe:
        fallback.append(max(clean_safe, key=lambda p: float(p.get("severity_final_distance_m") or -1)))
    return fallback


def feature_row(point: dict[str, Any]) -> list[float]:
    d = float(point["distance_m"])
    w = float(point["wind_m_s"])
    return [d, d * w, w]


def fit_logistic(points: list[dict[str, Any]]) -> dict[str, Any]:
    usable = [p for p in points if p.get("label") in {"clean_safe", "clean_unsafe"}]
    if not usable:
        return {"ok": False, "reason": "no clean labeled points"}
    y = np.array([1.0 if p["label"] == "clean_unsafe" else 0.0 for p in usable], dtype=float)
    if len(set(y.tolist())) < 2:
        return {"ok": False, "reason": "training split has one class", "train_points": len(usable)}
    x_raw = np.array([feature_row(p) for p in usable], dtype=float)
    mean = x_raw.mean(axis=0)
    std = x_raw.std(axis=0)
    std[std == 0] = 1.0
    x = np.column_stack([np.ones(len(x_raw)), (x_raw - mean) / std])
    beta = np.zeros(x.shape[1], dtype=float)
    lr = 0.18
    l2 = 1.0e-3
    for _ in range(6000):
        logits = np.clip(x @ beta, -40, 40)
        pred = 1.0 / (1.0 + np.exp(-logits))
        grad = (x.T @ (pred - y)) / len(y)
        grad[1:] += l2 * beta[1:]
        beta -= lr * grad
    logits = np.clip(x @ beta, -40, 40)
    pred = 1.0 / (1.0 + np.exp(-logits))
    acc = float(np.mean((pred >= 0.5) == (y >= 0.5)))
    return {
        "ok": True,
        "feature_names": ["intercept", "D", "D*wind", "wind"],
        "coefficients": [float(v) for v in beta],
        "feature_mean": [float(v) for v in mean],
        "feature_std": [float(v) for v in std],
        "train_points": len(usable),
        "train_accuracy": acc,
    }


def predict_probability(model: dict[str, Any], point: dict[str, Any]) -> float:
    beta = np.array(model["coefficients"], dtype=float)
    mean = np.array(model["feature_mean"], dtype=float)
    std = np.array(model["feature_std"], dtype=float)
    x_raw = np.array(feature_row(point), dtype=float)
    x = np.concatenate([[1.0], (x_raw - mean) / std])
    logit = float(np.clip(x @ beta, -40, 40))
    return float(1.0 / (1.0 + math.exp(-logit)))


def evaluate_split(train: list[dict[str, Any]], test: list[dict[str, Any]]) -> dict[str, Any]:
    model = fit_logistic(train)
    rows = []
    hits = 0
    usable_test = [p for p in test if p.get("label") in {"clean_safe", "clean_unsafe"}]
    if model.get("ok"):
        for point in usable_test:
            prob = predict_probability(model, point)
            pred = prob >= 0.5
            obs = point["label"] == "clean_unsafe"
            hits += int(pred == obs)
            rows.append({
                "point_key": point["point_key"],
                "distance_m": float(point["distance_m"]),
                "wind_m_s": float(point["wind_m_s"]),
                "observed_label": point["label"],
                "observed_unsafe": obs,
                "probability_unsafe": prob,
                "predicted_unsafe": pred,
                "correct": pred == obs,
            })
    metrics = {
        "count": len(usable_test),
        "classification_accuracy": (hits / len(usable_test)) if usable_test else None,
    }
    return {"model": model, "predictions": rows, "metrics": metrics}


def train_test(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    cfg = config["train_test"]
    interp_train_d = {float(v) for v in cfg["interpolation_train_distances_m"]}
    interp_train_w = {float(v) for v in cfg["interpolation_train_winds_m_s"]}
    interp_test_d = {float(v) for v in cfg["interpolation_test_distances_m"]}
    interp_test_w = {float(v) for v in cfg["interpolation_test_winds_m_s"]}
    interp_train = [p for p in points if float(p["distance_m"]) in interp_train_d and float(p["wind_m_s"]) in interp_train_w]
    interp_test = [p for p in points if float(p["distance_m"]) in interp_test_d and float(p["wind_m_s"]) in interp_test_w]
    extra_train = [
        p for p in points
        if float(p["distance_m"]) <= float(cfg["extrapolation_train_max_distance_m"])
        and float(p["wind_m_s"]) <= float(cfg["extrapolation_train_max_wind_m_s"])
    ]
    extra_test = [
        p for p in points
        if float(p["distance_m"]) >= float(cfg["extrapolation_test_min_distance_m"])
        and float(p["wind_m_s"]) >= float(cfg["extrapolation_test_min_wind_m_s"])
    ]
    interpolation = evaluate_split(interp_train, interp_test)
    extrapolation = evaluate_split(extra_train, extra_test)
    combined_rows = interpolation["predictions"] + extrapolation["predictions"]
    combined_acc = None
    if combined_rows:
        combined_acc = sum(1 for r in combined_rows if r["correct"]) / len(combined_rows)
    return {
        "formula": "unsafe probability = sigmoid(beta0 + beta_D*D + beta_DW*D*wind + beta_W*wind)",
        "interpolation": interpolation,
        "extrapolation": extrapolation,
        "combined_heldout_accuracy": combined_acc,
        "holdout_definition": cfg,
    }


def p_stratification_summary(layers: dict[str, dict[str, Any]]) -> dict[str, Any]:
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1]["batt_low_mah"]))
    counts = [
        {
            "layer": name,
            "batt_low_mah": float(layer["batt_low_mah"]),
            "clean_unsafe": int(layer["zone_counts"].get("clean_unsafe", 0)),
            "clean_safe": int(layer["zone_counts"].get("clean_safe", 0)),
            "contract_violated": int(layer["zone_counts"].get("contract_violated", 0)),
        }
        for name, layer in ordered
    ]
    nonincreasing = all(counts[i]["clean_unsafe"] <= counts[i - 1]["clean_unsafe"] for i in range(1, len(counts)))
    return {
        "counts": counts,
        "monotonic_shrink": nonincreasing,
        "conclusion": "clean_unsafe count is non-increasing as BATT_LOW_MAH rises" if nonincreasing else "clean_unsafe count did not shrink monotonically as BATT_LOW_MAH rose",
    }


def boundary_search_summary(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    distances = [float(v) for v in config["sweep"]["distances_m"]]
    winds = [float(v) for v in config["search"]["winds_m_s"]]
    by = {(float(p["distance_m"]), float(p["wind_m_s"])): p for p in points}
    records = []
    total_queries = 0
    for wind in winds:
        lo = 0
        hi = len(distances) - 1
        queries = []
        first_unsafe = None
        while lo <= hi:
            mid = (lo + hi) // 2
            d = distances[mid]
            p = by.get((d, wind))
            queries.append({"distance_m": d, "wind_m_s": wind, "label": None if p is None else p.get("label")})
            if p is not None and p.get("label") == "clean_unsafe":
                first_unsafe = d
                hi = mid - 1
            else:
                lo = mid + 1
            if len(queries) >= int(config["search"]["bisection_iterations_per_wind"]):
                break
        total_queries += len(queries)
        records.append({"wind_m_s": wind, "queries": queries, "first_unsafe_distance_m": first_unsafe})
    return {
        "strategy": "discrete bisection over D for each wind, replayed against completed grid run results",
        "query_count": total_queries,
        "full_grid_count": len([p for p in points if p.get("layer") == config["sweep"]["default_layer"]]),
        "records": records,
    }


def verdict_summary(
    premise: dict[str, Any],
    default_points: list[dict[str, Any]],
    prediction: dict[str, Any],
) -> dict[str, Any]:
    if not premise.get("satisfied"):
        return {
            "verdict": "INCONCLUSIVE",
            "premise_satisfied": False,
            "robust_clean_unsafe": False,
            "contract_clean_gap": False,
            "prediction_ok": False,
            "reason": "Premise failed: SITL energy model did not respond monotonically to wind/mass.",
        }
    clean_unsafe = [p for p in default_points if p.get("label") == "clean_unsafe"]
    contract_violated = [p for p in default_points if p.get("label") == "contract_violated"]
    repeated = [p for p in default_points if int(p.get("repetitions", 0)) >= 2]
    boundary_flips = [p for p in repeated if p.get("boundary_flip")]
    robust_clean_unsafe = bool(len(clean_unsafe) >= 2 and not boundary_flips)
    contract_clean_gap = bool(clean_unsafe and all(p.get("contract_clean_all") for p in clean_unsafe))
    interp_acc = prediction.get("interpolation", {}).get("metrics", {}).get("classification_accuracy")
    extra_acc = prediction.get("extrapolation", {}).get("metrics", {}).get("classification_accuracy")
    combined = prediction.get("combined_heldout_accuracy")
    prediction_ok = bool(
        interp_acc is not None
        and extra_acc is not None
        and combined is not None
        and float(interp_acc) >= 0.90
        and float(extra_acc) >= 0.90
        and float(combined) >= 0.90
    )
    if robust_clean_unsafe and contract_clean_gap and prediction_ok:
        verdict = "PASS"
        reason = "All decisive criteria are satisfied."
    else:
        verdict = "FAIL"
        missing = []
        if not robust_clean_unsafe:
            missing.append("no robust non-trivial clean_unsafe region")
        if not contract_clean_gap:
            missing.append("clean_unsafe is not contract-clean or is empty")
        if not prediction_ok:
            missing.append("held-out prediction including extrapolation is below 90% or incomplete")
        reason = "; ".join(missing)
    return {
        "verdict": verdict,
        "premise_satisfied": True,
        "robust_clean_unsafe": robust_clean_unsafe,
        "contract_clean_gap": contract_clean_gap,
        "prediction_ok": prediction_ok,
        "clean_unsafe_count": len(clean_unsafe),
        "contract_violated_count": len(contract_violated),
        "boundary_flip_points": [p["point_key"] for p in boundary_flips],
        "interpolation_accuracy": interp_acc,
        "extrapolation_accuracy": extra_acc,
        "combined_heldout_accuracy": combined,
        "reason": reason,
    }


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    plots = {
        "premise": plot_premise(payload["premise"], analysis / "rtl_energy_premise.png"),
    }
    if payload["verdict"]["verdict"] != "INCONCLUSIVE":
        default_points = payload["default_grid"]["points"]
        plots.update({
            "result_field": plot_result_field(default_points, analysis / "rtl_energy_result_field.png"),
            "severity": plot_severity_heatmap(default_points, analysis / "rtl_energy_severity_heatmap.png"),
            "p_stratification": plot_p_stratification(payload["p_stratification"]["layers"], analysis / "rtl_energy_p_stratification.png"),
            "train_test": plot_train_test(payload["predictive_rule"], analysis / "rtl_energy_train_test.png"),
        })
    return plots


def write_report(payload: dict[str, Any]) -> str:
    report = PLANC_ROOT / "results" / "rtl_energy_report.md"
    verdict = payload["verdict"]
    lines: list[str] = []
    lines.append(f"VERDICT: {verdict['verdict']}")
    lines.append("")
    lines.append("# planc RTL energy spec-gap decisive test")
    lines.append("")
    lines.append("## Four decisive criteria")
    lines.append("")
    lines.append(f"- Premise satisfied: **{verdict.get('premise_satisfied')}**.")
    lines.append(f"- Robust contract-clean unsafe region: **{verdict.get('robust_clean_unsafe')}**; clean_unsafe count={verdict.get('clean_unsafe_count', 0)}, boundary flips={', '.join(verdict.get('boundary_flip_points', [])) or 'none'}.")
    lines.append(f"- Battery contract clean and PGFUZZ-invisible: **{verdict.get('contract_clean_gap')}**; contract_violated count={verdict.get('contract_violated_count', 0)}.")
    lines.append(f"- Held-out prediction with extrapolation >= 90%: **{verdict.get('prediction_ok')}**; interpolation={fmt(verdict.get('interpolation_accuracy'), 3)}, extrapolation={fmt(verdict.get('extrapolation_accuracy'), 3)}, combined={fmt(verdict.get('combined_heldout_accuracy'), 3)}.")
    lines.append("")
    lines.append(f"Decision reason: {verdict.get('reason')}")
    lines.append("")
    lines.append("## Premise")
    lines.append("")
    lines.append(f"Premise conclusion: **{payload['premise']['satisfied']}** - {payload['premise']['reason']}.")
    lines.append("")
    lines.append("| run | model mass kg | wind m/s | consumed mAh | voltage drop rate V/s | parsed log | oracle |")
    lines.append("| --- | ---: | ---: | ---: | ---: | --- | --- |")
    for run in payload["premise"]["runs"]:
        csv_path = run.get("csv_path", "n/a")
        oracle_path = str(Path(csv_path).with_suffix(".oracle.json")) if csv_path != "n/a" else "n/a"
        lines.append(
            f"| {run.get('run_id')} | {fmt(run.get('model_mass_kg'))} | {fmt(run.get('wind_m_s'))} | "
            f"{fmt(run.get('consumed_mah'))} | {fmt(run.get('voltage_drop_rate_v_s'), 5)} | "
            f"{rel(csv_path)} | {rel(oracle_path)} |"
        )
    lines.append("")
    for check in payload["premise"].get("checks", []):
        gate = "hard" if check.get("hard_gate") else "reported"
        lines.append(f"- {check['name']} ({gate}): baseline={fmt(check.get('baseline'))}, stressed={fmt(check.get('stressed'))}, ok={check.get('ok')}.")
    if payload["premise"].get("voltage_caveat"):
        lines.append(f"- Caveat: {payload['premise']['voltage_caveat']}")
    lines.append("")
    lines.append("## Scenario")
    lines.append("")
    cfg = payload["config"]
    lines.append(
        f"Fixed P uses `BATT_FS_LOW_ACT=2` (RTL), `BATT_FS_CRT_ACT=1` (LAND), "
        f"`BATT_CAPACITY={fmt(cfg['baseline_params']['BATT_CAPACITY'], 0)} mAh`, "
        f"`SIM_BATT_CAP_AH={fmt(cfg['baseline_params']['SIM_BATT_CAP_AH'], 2)} Ah`, "
        f"`BATT_CRT_MAH={fmt(cfg['baseline_params']['BATT_CRT_MAH'], 0)} mAh`, and no geofence."
    )
    lines.append(
        f"M scans outbound distance D over {cfg['sweep']['distances_m']} at "
        f"{fmt(cfg['experiment']['cruise_speed_m_s'])} m/s. E scans wind over {cfg['sweep']['winds_m_s']} m/s with "
        "`SIM_WIND_DIR=270`, so outbound east is downwind and RTL westbound return is into wind."
    )
    lines.append(
        "`BATT_LOW_VOLT=0` and `BATT_CRT_VOLT=0` intentionally disable voltage failsafes; the legal documented "
        "capacity thresholds are the tested contract, with all set parameters read back per run."
    )
    lines.append("")
    if verdict["verdict"] != "INCONCLUSIVE":
        lines.append("## Three-Zone Field")
        lines.append("")
        counts = payload["default_grid"]["zone_counts"]
        lines.append(
            f"Default layer `{payload['default_grid']['layer']}` counts: clean_safe={counts.get('clean_safe', 0)}, "
            f"clean_unsafe={counts.get('clean_unsafe', 0)}, contract_violated={counts.get('contract_violated', 0)}, blocked={counts.get('blocked', 0)}."
        )
        lines.append("")
        lines.append("| D m | wind m/s | label | final dist m | dist at low FS m | stable | runs |")
        lines.append("| ---: | ---: | --- | ---: | ---: | --- | --- |")
        for p in sorted(payload["default_grid"]["points"], key=lambda r: (float(r["distance_m"]), float(r["wind_m_s"]))):
            lines.append(
                f"| {fmt(p['distance_m'], 0)} | {fmt(p['wind_m_s'], 0)} | {p.get('label')} | "
                f"{fmt(p.get('severity_final_distance_m'))} | {fmt(p.get('distance_at_low_failsafe_mean_m'))} | "
                f"{p.get('stable_binary')} | {', '.join(str(r) for r in p.get('run_ids', []))} |"
            )
        lines.append("")
        lines.append(
            "PGFUZZ-invisible check: every `clean_unsafe` point has `contract_clean_all=True`, battery low RTL "
            "and critical LAND actions are checked against the configured mAh thresholds, and points with any "
            "other ERR/EV/STATUSTEXT failsafe are labeled `contract_violated` instead."
        )
        lines.append("")
        lines.append("## Predictive Rule")
        lines.append("")
        pred = payload["predictive_rule"]
        lines.append(f"Formula: `{pred['formula']}`.")
        for split in ("interpolation", "extrapolation"):
            metrics = pred[split]["metrics"]
            model = pred[split]["model"]
            lines.append(
                f"{split}: n={metrics.get('count', 0)}, accuracy={fmt(metrics.get('classification_accuracy'), 3)}, "
                f"model_ok={model.get('ok')}, train_points={model.get('train_points')}."
            )
        lines.append(f"Combined held-out accuracy: {fmt(pred.get('combined_heldout_accuracy'), 3)}.")
        lines.append("")
        lines.append("## P Stratification")
        lines.append("")
        p_summary = payload["p_stratification"]["summary"]
        lines.append(f"Conclusion: {p_summary['conclusion']}.")
        lines.append("")
        lines.append("| layer | BATT_LOW_MAH | clean_unsafe | clean_safe | contract_violated |")
        lines.append("| --- | ---: | ---: | ---: | ---: |")
        for row in p_summary["counts"]:
            lines.append(
                f"| {row['layer']} | {fmt(row['batt_low_mah'], 0)} | {row['clean_unsafe']} | "
                f"{row['clean_safe']} | {row['contract_violated']} |"
            )
        lines.append("")
        lines.append("## Search Efficiency")
        lines.append("")
        search = payload["search_efficiency"]
        lines.append(
            f"{search['strategy']}. Queries to bracket boundaries: {search['query_count']} vs full grid "
            f"{search['full_grid_count']}."
        )
        lines.append("")
        lines.append("## Reproducibility")
        lines.append("")
        repro = payload["reproducibility"]
        lines.append(
            f"Repeated near-boundary points: {len(repro['repeated_points'])}; boundary flips: "
            f"{', '.join(repro['boundary_flip_points']) or 'none'}."
        )
        lines.append(
            "For each run id in the field table, the committed audit files are "
            "`planc/logs/<run_id>_params.json`, `planc/logs/<run_id>_parsed.csv`, and "
            "`planc/logs/<run_id>_parsed.oracle.json`."
        )
        lines.append("")
    lines.append("## Figures")
    lines.append("")
    for name, path in payload.get("artifacts", {}).get("plots", {}).items():
        lines.append(f"- {name}: ![]({rel(path)})")
    lines.append("")
    lines.append("## Limitations")
    lines.append("")
    lines.append(
        "This is still SITL, not HITL. The decisive verdict applies to this ArduCopter SITL energy model and "
        "the pre-registered RTL energy trap. Logs, parameter readbacks, parsed CSVs, and oracle sidecars are "
        "kept under `planc/logs/` for independent audit."
    )
    lines.append("")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(lines), encoding="utf-8")
    return str(report)


def build_payload(
    config: dict[str, Any],
    env: dict[str, Any],
    premise: dict[str, Any],
    grid_runs: list[dict[str, Any]],
) -> dict[str, Any]:
    distances = [float(v) for v in config["sweep"]["distances_m"]]
    winds = [float(v) for v in config["sweep"]["winds_m_s"]]
    default_layer = str(config["sweep"]["default_layer"])
    layers: dict[str, dict[str, Any]] = {}
    for layer_name, layer_cfg in config["sweep"]["p_layers"].items():
        points = aggregate_layer(grid_runs, layer_name, distances, winds)
        layers[layer_name] = {
            "layer": layer_name,
            "batt_low_mah": float(layer_cfg["BATT_LOW_MAH"]),
            "points": points,
            "zone_counts": zone_counts(points),
        }
    default_points = layers.get(default_layer, {"points": []})["points"]
    prediction = train_test(default_points, config) if premise.get("satisfied") else {
        "formula": "not evaluated because premise failed",
        "interpolation": {"metrics": {"count": 0}, "model": {"ok": False}, "predictions": []},
        "extrapolation": {"metrics": {"count": 0}, "model": {"ok": False}, "predictions": []},
        "combined_heldout_accuracy": None,
    }
    verdict = verdict_summary(premise, default_points, prediction)
    repeated_points = [p for p in default_points if int(p.get("repetitions", 0)) >= 2]
    payload = {
        "status": "COMPLETE",
        "env": env,
        "config": config,
        "premise": premise,
        "default_grid": {
            "layer": default_layer,
            "distances_m": distances,
            "winds_m_s": winds,
            "points": default_points,
            "zone_counts": zone_counts(default_points),
        },
        "p_stratification": {
            "layers": layers,
            "summary": p_stratification_summary(layers) if premise.get("satisfied") else {"counts": [], "monotonic_shrink": None, "conclusion": "not evaluated"},
        },
        "predictive_rule": prediction,
        "search_efficiency": boundary_search_summary(default_points, config) if premise.get("satisfied") else {},
        "reproducibility": {
            "repeated_points": repeated_points,
            "boundary_flip_points": [p["point_key"] for p in repeated_points if p.get("boundary_flip")],
        },
        "verdict": verdict,
        "runs": {
            "premise": premise.get("runs", []),
            "grid": grid_runs,
        },
    }
    payload["artifacts"] = {"plots": make_plots(payload)}
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the planc RTL energy spec-gap decisive test.")
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "rtl_energy_config.yaml")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    results_dir = PLANC_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    premise_partial_path = results_dir / "rtl_energy_premise_partial.json"
    grid_partial_path = results_dir / "rtl_energy_results_partial.json"
    final_path = results_dir / "rtl_energy_results.json"

    env = probe_environment(config_with_model(config, "nominal"), REPO_ROOT)
    write_env(env, results_dir / "env_rtl_energy.json")
    probe = connectivity_probe(config, env)
    write_env(env, results_dir / "env_rtl_energy.json")
    if not probe.get("ok"):
        payload = {
            "status": "BLOCKED",
            "reason": "SITL connectivity probe failed",
            "env": env,
            "runs": {"premise": [], "grid": []},
            "verdict": {"verdict": "INCONCLUSIVE", "premise_satisfied": False, "reason": "connectivity failed"},
        }
        write_json(final_path, payload)
        print(f"BLOCKED: SITL connectivity probe failed; see {final_path}", flush=True)
        return 2

    premise_runs: list[dict[str, Any]] = []
    grid_runs: list[dict[str, Any]] = []
    if args.resume and premise_partial_path.exists():
        premise_runs = list(json.loads(premise_partial_path.read_text(encoding="utf-8")).get("premise_runs", []))
    if args.resume and grid_partial_path.exists():
        grid_runs = list(json.loads(grid_partial_path.read_text(encoding="utf-8")).get("grid_runs", []))

    premise_cfg = config["premise"]
    run_or_reuse_premise(
        config,
        premise_runs,
        premise_partial_path,
        kind="nominal_no_wind",
        label="nominal/no wind",
        model_name=str(premise_cfg["nominal_mass_model"]),
        wind_m_s=0.0,
    )
    run_or_reuse_premise(
        config,
        premise_runs,
        premise_partial_path,
        kind="nominal_headwind",
        label="nominal/high wind",
        model_name=str(premise_cfg["nominal_mass_model"]),
        wind_m_s=float(premise_cfg["wind_probe_m_s"]),
    )
    run_or_reuse_premise(
        config,
        premise_runs,
        premise_partial_path,
        kind="heavy_no_wind",
        label="heavy/no wind",
        model_name=str(premise_cfg["heavy_mass_model"]),
        wind_m_s=0.0,
    )
    premise = premise_summary(premise_runs)
    write_json(premise_partial_path, {"premise": premise, "premise_runs": premise_runs})

    if not premise.get("satisfied"):
        payload = build_payload(config, env, premise, grid_runs)
        write_json(final_path, payload)
        print(f"INCONCLUSIVE: premise failed; report={payload['artifacts']['report']}", flush=True)
        return 0

    distances = [float(v) for v in config["sweep"]["distances_m"]]
    winds = [float(v) for v in config["sweep"]["winds_m_s"]]
    default_layer = str(config["sweep"]["default_layer"])
    for d in distances:
        for w in winds:
            run_or_reuse_grid(
                config,
                grid_runs,
                grid_partial_path,
                layer=default_layer,
                distance_m=d,
                wind_m_s=w,
                rep_index=1,
                roles=["default_grid", "p_layer"],
            )

    default_points_once = aggregate_layer(grid_runs, default_layer, distances, winds)
    near_points = find_near_boundary(default_points_once, distances, winds)
    for point in near_points:
        for rep in range(2, int(config["sweep"]["near_boundary_repetitions"]) + 1):
            run_or_reuse_grid(
                config,
                grid_runs,
                grid_partial_path,
                layer=default_layer,
                distance_m=float(point["distance_m"]),
                wind_m_s=float(point["wind_m_s"]),
                rep_index=rep,
                roles=["near_boundary_repeat", "default_grid"],
            )

    for layer in config["sweep"]["p_layers"]:
        if str(layer) == default_layer:
            continue
        for d in distances:
            for w in winds:
                run_or_reuse_grid(
                    config,
                    grid_runs,
                    grid_partial_path,
                    layer=str(layer),
                    distance_m=d,
                    wind_m_s=w,
                    rep_index=1,
                    roles=["p_layer"],
                )

    payload = build_payload(config, env, premise, grid_runs)
    write_json(final_path, payload)
    print(f"COMPLETE: verdict={payload['verdict']['verdict']} report={payload['artifacts']['report']} results={final_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
