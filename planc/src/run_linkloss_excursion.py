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
from injector import destination_point
from linkloss_plots import (
    plot_p_stratification,
    plot_premise,
    plot_result_field,
    plot_severity_heatmap,
    plot_train_test,
)
from oracle import BAD_EVENT_NAMES, COPTER_MODES, ERROR_SUBSYSTEMS, EVENT_NAMES, horizontal_distance_m
from param_manager import ParamManager
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
    10: "FENCE_BREACHED",
    11: "TERRAIN_FAILSAFE",
    19: "CRASH_FAILSAFE",
    25: "FAILSAFE",
    50: "DEADRECKON_FAILSAFE",
}

INTENDED_GCS_ERR_SUBSYS = 8
INTENDED_RC_ERR_SUBSYS = 5
FENCE_ERR_SUBSYS = 9
GCS_FAILSAFE_REASON = 5
RADIO_FAILSAFE_REASON = 3

DIRTY_TEXT_MARKERS = (
    "battery failsafe",
    "ekf failsafe",
    "terrain failsafe",
    "deadreckon",
    "crash",
    "arming checks failed",
    "outside fence",
    "failed to set destination",
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


def config_copy(config: dict[str, Any]) -> dict[str, Any]:
    return copy.deepcopy(config)


def controlled_params(config: dict[str, Any], overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    params = dict(config.get("baseline_params", {}))
    if overrides:
        params.update(overrides)
    return params


def trigger_distance_m(config: dict[str, Any]) -> float:
    exp = config["experiment"]
    return float(exp["fence_radius_m"]) - float(exp["d_inside_m"])


def point_key(layer: str, speed_m_s: float, wind_m_s: float) -> str:
    return f"{layer}_v{int(speed_m_s):02d}_w{int(wind_m_s):02d}"


def run_id_for(layer: str, speed_m_s: float, wind_m_s: float, rep_index: int) -> str:
    return f"linkloss_{layer}_v{int(speed_m_s):02d}_w{int(wind_m_s):02d}_r{rep_index}"


def premise_run_id(kind: str) -> str:
    return f"linkloss_premise_{kind}"


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
        if float(row["distance_m"]) >= float(distance_m):
            return float(row["time_s"])
    return None


def _clean_param_records(records: list[dict[str, Any]]) -> bool:
    return bool(records) and all(bool(r.get("ok")) for r in records)


def _realtime_distance_from_home_m(pos_msg: Any, home: dict[str, Any]) -> float:
    lat = float(pos_msg.lat) / 1.0e7
    lon = float(pos_msg.lon) / 1.0e7
    return horizontal_distance_m(float(home["lat"]), float(home["lon"]), lat, lon)


def _command_change_speed(master, speed_m_s: float) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1.0,
        float(speed_m_s),
        -1.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def upload_outbound_mission(master, config: dict[str, Any], speed_m_s: float) -> dict[str, Any]:
    exp = config["experiment"]
    home = exp["home"]
    lat, lon = destination_point(
        float(home["lat"]),
        float(home["lon"]),
        float(exp["target_bearing_deg"]),
        float(exp["waypoint_distance_m"]),
    )
    items = [
        {
            "seq": 0,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            "current": 1,
            "autocontinue": 1,
            "param1": 0.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "x": int(round(float(home["lat"]) * 1.0e7)),
            "y": int(round(float(home["lon"]) * 1.0e7)),
            "lat": float(home["lat"]),
            "lon": float(home["lon"]),
            "z": float(exp["takeoff_alt_m"]),
        },
        {
            "seq": 1,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0,
            "autocontinue": 1,
            "param1": 0.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "x": int(round(lat * 1.0e7)),
            "y": int(round(lon * 1.0e7)),
            "lat": float(lat),
            "lon": float(lon),
            "z": float(exp["takeoff_alt_m"]),
        },
    ]
    item_by_seq = {int(item["seq"]): item for item in items}
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="MISSION_ACK", blocking=True, timeout=0.25)
        if msg is not None:
            break
    master.mav.mission_count_send(master.target_system, master.target_component, len(items))
    deadline = time.time() + 15.0
    requested: list[int] = []
    ack = None
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "MISSION_ACK":
            ack = msg.to_dict()
            break
        seq = int(getattr(msg, "seq", -1))
        if seq not in item_by_seq:
            continue
        requested.append(seq)
        item = item_by_seq[seq]
        if mtype == "MISSION_REQUEST_INT":
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                item["seq"],
                item["frame"],
                item["command"],
                item["current"],
                item["autocontinue"],
                item["param1"],
                item["param2"],
                item["param3"],
                item["param4"],
                item["x"],
                item["y"],
                item["z"],
            )
        else:
            master.mav.mission_item_send(
                master.target_system,
                master.target_component,
                item["seq"],
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                item["command"],
                item["current"],
                item["autocontinue"],
                item["param1"],
                item["param2"],
                item["param3"],
                item["param4"],
                item["lat"],
                item["lon"],
                item["z"],
            )
    if ack is None:
        raise RuntimeError("mission upload did not receive MISSION_ACK")
    if int(ack.get("type", -1)) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"mission upload failed: {ack}")
    master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
    _command_change_speed(master, speed_m_s)
    return {
        "waypoint_lat": lat,
        "waypoint_lon": lon,
        "waypoint_distance_m": float(exp["waypoint_distance_m"]),
        "commanded_speed_m_s": float(speed_m_s),
        "requested_sequences": requested,
        "mission_ack": ack,
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


def run_linkloss_flight(master, config: dict[str, Any], speed_m_s: float) -> dict[str, Any]:
    prepare_flight(master, config)
    exp = config["experiment"]
    home = exp["home"]
    mission = upload_outbound_mission(master, config, speed_m_s)
    set_mode(master, "AUTO")
    _command_change_speed(master, speed_m_s)
    stream_hz = float(exp.get("stream_hz", 10))
    heartbeat_dt = 1.0 / max(stream_hz, 1.0)
    trigger_dist = trigger_distance_m(config)
    outbound_start = time.time()
    next_hb = 0.0
    next_speed_cmd = 0.0
    trigger_wall_s = None
    max_pretrigger_distance = 0.0
    modes: list[dict[str, Any]] = []
    statustext: list[str] = []
    fence_status: list[dict[str, Any]] = []

    while time.time() - outbound_start < float(exp["max_auto_outbound_s"]):
        now = time.time()
        elapsed = now - outbound_start
        if now >= next_hb:
            send_gcs_heartbeat(master)
            next_hb = now + heartbeat_dt
        if now >= next_speed_cmd:
            _command_change_speed(master, speed_m_s)
            next_speed_cmd = now + 1.0
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT", "FENCE_STATUS"],
            blocking=True,
            timeout=0.1,
        )
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            dist = _realtime_distance_from_home_m(msg, home)
            max_pretrigger_distance = max(max_pretrigger_distance, dist)
            if dist >= trigger_dist:
                trigger_wall_s = elapsed
                break
        elif msg.get_type() == "HEARTBEAT":
            mode = mode_name(master, msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": elapsed, "mode": mode})
        elif msg.get_type() == "STATUSTEXT":
            statustext.append(str(getattr(msg, "text", "")))
        elif msg.get_type() == "FENCE_STATUS":
            fence_status.append({
                "wall_s": elapsed,
                "breach_status": int(getattr(msg, "breach_status", 0)),
                "breach_type": int(getattr(msg, "breach_type", 0)),
                "breach_count": int(getattr(msg, "breach_count", 0)),
            })

    action_seen_at = None
    max_linkloss_distance = max_pretrigger_distance
    linkloss_start = time.time()
    # From this point until reconnect, do not send GCS heartbeats. Receiving telemetry is passive.
    while time.time() - linkloss_start < float(exp["max_linkloss_observation_s"]):
        elapsed = time.time() - linkloss_start
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT", "FENCE_STATUS"],
            blocking=True,
            timeout=0.2,
        )
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            max_linkloss_distance = max(max_linkloss_distance, _realtime_distance_from_home_m(msg, home))
        elif msg.get_type() == "HEARTBEAT":
            mode = mode_name(master, msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": (trigger_wall_s or 0.0) + elapsed, "mode": mode})
            if mode in {"RTL", "LAND", "BRAKE", "SMART_RTL"} and action_seen_at is None:
                action_seen_at = time.time()
        elif msg.get_type() == "STATUSTEXT":
            statustext.append(str(getattr(msg, "text", "")))
        elif msg.get_type() == "FENCE_STATUS":
            fence_status.append({
                "wall_s": (trigger_wall_s or 0.0) + elapsed,
                "breach_status": int(getattr(msg, "breach_status", 0)),
                "breach_type": int(getattr(msg, "breach_type", 0)),
                "breach_count": int(getattr(msg, "breach_count", 0)),
            })
        if action_seen_at is not None and time.time() - action_seen_at >= float(exp["post_failsafe_observation_s"]):
            break

    reconnect_start = time.time()
    while time.time() - reconnect_start < 3.0:
        send_gcs_heartbeat(master)
        master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.1)
    cleanup_error = None
    try:
        land_and_disarm(master, timeout_s=45.0)
    except Exception as exc:
        cleanup_error = repr(exc)
    return {
        "motion": "auto_outbound_then_stop_gcs_heartbeat",
        "mission": mission,
        "commanded_speed_m_s": float(speed_m_s),
        "trigger_distance_m": trigger_dist,
        "trigger_wall_s": trigger_wall_s,
        "trigger_reached": trigger_wall_s is not None,
        "max_pretrigger_realtime_distance_m": max_pretrigger_distance,
        "max_linkloss_realtime_distance_m": max_linkloss_distance,
        "modes_seen": modes,
        "statustext": statustext,
        "fence_status": fence_status[-30:],
        "cleanup_error": cleanup_error,
    }


def parse_linkloss_dataflash(
    *,
    bin_path: Path,
    csv_path: Path,
    home: dict[str, Any],
    params: dict[str, Any],
    speed_m_s: float,
    wind_m_s: float,
    target_bearing_deg: float,
    fence_radius_m: float,
    trigger_distance_m_value: float,
    fence_tolerance_m: float,
    timeout_tolerance_s: float,
    speed_tolerance_m_s: float,
    speed_audit_min_distance_m: float,
    run_kind: str,
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    pos_rows: list[dict[str, Any]] = []
    gps_rows: list[dict[str, Any]] = []
    xkf_velocity_rows: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    event_records: list[dict[str, Any]] = []
    err_records: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    fence_msgs: list[dict[str, Any]] = []
    parm_rows: list[dict[str, Any]] = []
    start_time: float | None = None
    home_lat = float(home["lat"])
    home_lon = float(home["lon"])
    bearing_rad = math.radians(float(target_bearing_deg))

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
            rec = {"time_s": rel_t, "text": str(_field(data, "Message", "Msg", "Text") or ""), "raw": data}
            messages.append(rec)
            if "fence" in rec["text"].lower():
                fence_msgs.append(rec)
        elif mtype == "FNCE":
            fence_msgs.append({"time_s": rel_t, "text": "FNCE", "raw": data})
        elif mtype == "XKF1":
            core = _field(data, "C")
            if core is None or int(core) == 0:
                vn = float(_field(data, "VN") or 0.0)
                ve = float(_field(data, "VE") or 0.0)
                forward = vn * math.cos(bearing_rad) + ve * math.sin(bearing_rad)
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

    max_distance = max((float(r["distance_m"]) for r in rows), default=0.0)
    max_row = max(rows, key=lambda r: float(r["distance_m"])) if rows else None
    unsafe_binary = bool(max_distance > float(fence_radius_m) + float(fence_tolerance_m))
    safe_binary = bool(rows and not unsafe_binary)
    severity_overshoot = max(0.0, max_distance - float(fence_radius_m))
    trigger_time = _first_time_distance(rows, float(trigger_distance_m_value))

    intended_gcs_errors = [
        e for e in err_records
        if int(e["subsys"]) == INTENDED_GCS_ERR_SUBSYS and int(e["ecode"]) != 0
    ]
    gcs_resolved_errors = [
        e for e in err_records
        if int(e["subsys"]) == INTENDED_GCS_ERR_SUBSYS and int(e["ecode"]) == 0
    ]
    fence_report_errors = [
        e for e in err_records
        if int(e["subsys"]) == FENCE_ERR_SUBSYS
    ]
    first_gcs_error = intended_gcs_errors[0] if intended_gcs_errors else None
    failsafe_time = None if first_gcs_error is None else float(first_gcs_error["time_s"])
    action_modes = [
        m for m in modes
        if m["mode"] == "RTL"
        and (int(m.get("reason") or -1) == GCS_FAILSAFE_REASON or (failsafe_time is not None and float(m["time_s"]) >= failsafe_time))
    ]
    action_mode = action_modes[0] if action_modes else None
    action_time = None if action_mode is None else float(action_mode["time_s"])
    configured_timeout = float(params.get("FS_GCS_TIMEOUT", 0.0))
    timeout_observed = None
    if trigger_time is not None and failsafe_time is not None:
        timeout_observed = float(failsafe_time) - float(trigger_time)
    timeout_ok = bool(
        trigger_time is not None
        and failsafe_time is not None
        and configured_timeout > 0
        and abs(float(timeout_observed) - configured_timeout) <= float(timeout_tolerance_s)
    )
    action_ok = bool(action_mode is not None and timeout_ok)

    other_errors = [
        e for e in err_records
        if int(e["ecode"]) != 0 and int(e["subsys"]) not in {INTENDED_GCS_ERR_SUBSYS, FENCE_ERR_SUBSYS}
    ]
    bad_events = [e for e in event_records if e.get("name") in BAD_EVENT_NAMES]
    dirty_messages = []
    gcs_messages = []
    for msg in messages:
        text = str(msg.get("text", ""))
        low = text.lower()
        if "gcs failsafe" in low:
            gcs_messages.append(msg)
            continue
        if "fence breached" in low or "fence breach" in low:
            continue
        if any(marker in low for marker in DIRTY_TEXT_MARKERS):
            dirty_messages.append(msg)
        elif "radio failsafe" in low:
            dirty_messages.append(msg)

    audit_rows = []
    audit_max_dist = max(float(speed_audit_min_distance_m) + 1.0, float(trigger_distance_m_value) - 3.0)
    for vel in xkf_velocity_rows:
        t = float(vel["time_s"])
        if trigger_time is not None and t > trigger_time:
            continue
        pos = _nearest(rows, t)
        if pos is None:
            continue
        dist = float(pos["distance_m"])
        if dist < float(speed_audit_min_distance_m) or dist > audit_max_dist:
            continue
        audit_rows.append({**vel, "distance_m": dist})
    speeds = [float(r["forward_speed_m_s"]) for r in audit_rows]
    median = statistics.median(speeds) if speeds else None
    p95 = np.percentile(np.array(speeds, dtype=float), 95) if speeds else None
    speed_audit = {
        "source": "XKF1 primary core VN/VE projected onto outbound bearing before heartbeat stop",
        "samples": len(audit_rows),
        "commanded_speed_m_s": float(speed_m_s),
        "median_forward_speed_m_s": None if median is None else float(median),
        "p95_forward_speed_m_s": None if p95 is None else float(p95),
        "median_error_m_s": None if median is None else float(median) - float(speed_m_s),
        "p95_error_m_s": None if p95 is None else float(p95) - float(speed_m_s),
        "within_tolerance": bool(median is not None and abs(float(median) - float(speed_m_s)) <= float(speed_tolerance_m_s)),
        "audit_distance_window_m": [float(speed_audit_min_distance_m), float(audit_max_dist)],
    }

    violations = []
    if run_kind in {"grid", "premise"}:
        if trigger_time is None:
            violations.append("linkloss_trigger_distance_not_reached")
        if not intended_gcs_errors:
            violations.append("gcs_failsafe_missing")
        if not action_mode:
            violations.append("gcs_failsafe_RTL_missing")
        if failsafe_time is not None and not timeout_ok:
            violations.append("gcs_failsafe_timeout_mismatch")
        if not speed_audit.get("within_tolerance"):
            violations.append("commanded_speed_not_reached")
    if other_errors:
        violations.append("other_ERR_subsystems")
    if bad_events:
        violations.append("bad_EV_events")
    if dirty_messages:
        violations.append("dirty_STATUSTEXT")

    result = {
        "bin_path": str(bin_path),
        "csv_path": str(csv_path),
        "position_source": rows[0]["source"] if rows else None,
        "samples": len(rows),
        "modes": modes,
        "event_records": event_records,
        "err_records": err_records,
        "intended_gcs_errors": intended_gcs_errors,
        "gcs_resolved_errors": gcs_resolved_errors,
        "fence_report_only_errors": fence_report_errors,
        "fence_messages": fence_msgs[:30],
        "other_errors": other_errors,
        "bad_events": bad_events,
        "messages_count": len(messages),
        "gcs_messages": gcs_messages[:20],
        "dirty_messages": dirty_messages[:20],
        "parm_rows_count": len(parm_rows),
        "max_distance_m": max_distance,
        "max_distance_sample": max_row,
        "fence_radius_m": float(fence_radius_m),
        "fence_tolerance_m": float(fence_tolerance_m),
        "unsafe_binary": unsafe_binary,
        "safe_binary": safe_binary,
        "severity_overshoot_m": severity_overshoot,
        "trigger_distance_m": float(trigger_distance_m_value),
        "trigger_time_s": trigger_time,
        "gcs_failsafe_time_s": failsafe_time,
        "gcs_failsafe_timeout_config_s": configured_timeout,
        "gcs_failsafe_timeout_observed_s": timeout_observed,
        "gcs_failsafe_timeout_ok": timeout_ok,
        "gcs_action_ok": action_ok,
        "gcs_action_mode": action_mode,
        "gcs_action_time_s": action_time,
        "speed_audit": speed_audit,
        "contract_clean": not violations,
        "contract_violations": violations,
        "report_only_fence_breach_is_violation": False,
        "speed_m_s": float(speed_m_s),
        "wind_m_s": float(wind_m_s),
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
        run["label"] = "clean_safe"
    else:
        run["label"] = "contract_violated"


def connectivity_probe(config: dict[str, Any], env: dict[str, Any]) -> dict[str, Any]:
    runner = SitlRunner(config_copy(config), REPO_ROOT)
    run_id = "linkloss_connectivity_probe"
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


def run_one(
    config: dict[str, Any],
    *,
    run_id: str,
    run_kind: str,
    layer: str | None,
    speed_m_s: float,
    wind_m_s: float,
    timeout_s: float,
    rep_index: int,
    roles: list[str],
) -> dict[str, Any]:
    cfg = config_copy(config)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "run_kind": run_kind,
        "layer": layer,
        "point_key": None if layer is None else point_key(layer, speed_m_s, wind_m_s),
        "rep_index": rep_index,
        "roles": roles,
        "speed_m_s": float(speed_m_s),
        "wind_m_s": float(wind_m_s),
        "timeout_s": float(timeout_s),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        params = controlled_params(config, {
            "SIM_WIND_SPD": float(wind_m_s),
            "FS_GCS_TIMEOUT": float(timeout_s),
            "FENCE_RADIUS": float(config["experiment"]["fence_radius_m"]),
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
        result["flight"] = run_linkloss_flight(master, cfg, float(speed_m_s))
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
        parsed = parse_linkloss_dataflash(
            bin_path=bin_path,
            csv_path=csv_path,
            home=cfg["experiment"]["home"],
            params=params,
            speed_m_s=float(speed_m_s),
            wind_m_s=float(wind_m_s),
            target_bearing_deg=float(config["experiment"]["target_bearing_deg"]),
            fence_radius_m=float(config["experiment"]["fence_radius_m"]),
            trigger_distance_m_value=trigger_distance_m(config),
            fence_tolerance_m=float(config["experiment"]["fence_tolerance_m"]),
            timeout_tolerance_s=float(config["experiment"]["linkloss_timeout_tolerance_s"]),
            speed_tolerance_m_s=float(config["experiment"]["speed_audit_tolerance_m_s"]),
            speed_audit_min_distance_m=float(config["experiment"]["speed_audit_min_distance_m"]),
            run_kind=run_kind,
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


def run_or_reuse(
    config: dict[str, Any],
    runs: list[dict[str, Any]],
    partial_path: Path,
    *,
    run_id: str,
    run_kind: str,
    layer: str | None,
    speed_m_s: float,
    wind_m_s: float,
    timeout_s: float,
    rep_index: int,
    roles: list[str],
) -> dict[str, Any]:
    for run in runs:
        if run.get("run_id") == run_id:
            update_roles(run, roles)
            classify_run(run)
            write_json(partial_path, {"runs": runs})
            return run
    print(
        f"RUN {run_id} kind={run_kind} layer={layer} v={speed_m_s} wind={wind_m_s} timeout={timeout_s} roles={','.join(roles)}",
        flush=True,
    )
    run = run_one(
        config,
        run_id=run_id,
        run_kind=run_kind,
        layer=layer,
        speed_m_s=speed_m_s,
        wind_m_s=wind_m_s,
        timeout_s=timeout_s,
        rep_index=rep_index,
        roles=roles,
    )
    runs.append(run)
    write_json(partial_path, {"runs": runs})
    return run


def group_runs(runs: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for run in runs:
        key = str(run.get("point_key"))
        grouped.setdefault(key, []).append(run)
    for entries in grouped.values():
        entries.sort(key=lambda r: int(r.get("rep_index", 0)))
    return grouped


def aggregate_point(runs: list[dict[str, Any]]) -> dict[str, Any]:
    sample = runs[0]
    complete = [r for r in runs if not r.get("error")]
    labels = [str(r.get("label", "blocked")) for r in complete]
    clean_complete = [r for r in complete if r.get("label") in {"clean_safe", "clean_unsafe"}]
    overshoot_values = [
        float(r["severity_overshoot_m"])
        for r in complete
        if r.get("severity_overshoot_m") is not None
    ]
    max_distance_values = [
        float(r["max_distance_m"])
        for r in complete
        if r.get("max_distance_m") is not None
    ]
    observed_timeouts = [
        float(r["gcs_failsafe_timeout_observed_s"])
        for r in complete
        if r.get("gcs_failsafe_timeout_observed_s") is not None
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
        "speed_m_s": float(sample.get("speed_m_s")),
        "wind_m_s": float(sample.get("wind_m_s")),
        "timeout_s": float(sample.get("timeout_s")),
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
        "severity_overshoot_m": statistics.fmean(overshoot_values) if overshoot_values else None,
        "severity_spread_m": max(overshoot_values) - min(overshoot_values) if len(overshoot_values) >= 2 else 0.0 if overshoot_values else None,
        "max_distance_m": statistics.fmean(max_distance_values) if max_distance_values else None,
        "gcs_failsafe_timeout_observed_s": statistics.fmean(observed_timeouts) if observed_timeouts else None,
        "contract_violations": sorted({v for r in complete for v in r.get("contract_violations", [])}),
        "errors": [r.get("error") for r in runs if r.get("error")],
    }


def aggregate_layer(runs: list[dict[str, Any]], layer: str, speeds: list[float], winds: list[float]) -> list[dict[str, Any]]:
    grouped = group_runs([r for r in runs if r.get("layer") == layer])
    points = []
    for speed in speeds:
        for wind in winds:
            key = point_key(layer, speed, wind)
            if key in grouped:
                points.append(aggregate_point(grouped[key]))
    return points


def zone_counts(points: list[dict[str, Any]]) -> dict[str, int]:
    counts = {"clean_safe": 0, "clean_unsafe": 0, "contract_violated": 0, "blocked": 0}
    for point in points:
        label = str(point.get("label", "blocked"))
        counts[label] = counts.get(label, 0) + 1
    return counts


def find_near_boundary(points: list[dict[str, Any]], speeds: list[float], winds: list[float]) -> list[dict[str, Any]]:
    index = {(float(p["speed_m_s"]), float(p["wind_m_s"])): p for p in points}
    near: dict[str, dict[str, Any]] = {}
    clean_labels = {"clean_safe", "clean_unsafe"}
    for speed in speeds:
        for wind in winds:
            p = index.get((float(speed), float(wind)))
            if not p or p.get("label") not in clean_labels:
                continue
            neighbors = []
            si = speeds.index(speed)
            wi = winds.index(wind)
            for ns_i, nw_i in ((si - 1, wi), (si + 1, wi), (si, wi - 1), (si, wi + 1)):
                if 0 <= ns_i < len(speeds) and 0 <= nw_i < len(winds):
                    neighbors.append(index.get((float(speeds[ns_i]), float(winds[nw_i]))))
            if any(n and n.get("label") in clean_labels and n.get("label") != p.get("label") for n in neighbors):
                near[str(p["point_key"])] = p
    if near:
        return list(near.values())
    clean_unsafe = [p for p in points if p.get("label") == "clean_unsafe"]
    clean_safe = [p for p in points if p.get("label") == "clean_safe"]
    fallback = []
    if clean_unsafe:
        fallback.append(min(clean_unsafe, key=lambda p: float(p.get("severity_overshoot_m") or 9999)))
    if clean_safe:
        fallback.append(max(clean_safe, key=lambda p: float(p.get("max_distance_m") or -1)))
    return fallback


def premise_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    by_kind = {str(r.get("premise_kind")): r for r in runs}

    def clean_action(run: dict[str, Any] | None) -> bool:
        return bool(run and not run.get("error") and run.get("gcs_action_ok") and not [v for v in run.get("contract_violations", []) if v not in {"commanded_speed_not_reached"}])

    def overshoot(run: dict[str, Any] | None) -> float | None:
        if run is None or run.get("severity_overshoot_m") is None:
            return None
        return float(run["severity_overshoot_m"])

    speed_base = by_kind.get("speed_base")
    speed_high = by_kind.get("speed_high")
    wind_base = by_kind.get("wind_base")
    wind_high = by_kind.get("wind_high")
    timeout_short = by_kind.get("timeout_short")
    timeout_default = by_kind.get("timeout_default")
    timeout_long = by_kind.get("timeout_long")

    checks: list[dict[str, Any]] = []
    action_runs = [speed_base, speed_high, wind_base, wind_high, timeout_short, timeout_default, timeout_long]
    action_ok = all(clean_action(r) for r in action_runs)
    checks.append({
        "name": "gcs_failsafe_triggers_and_RTLs",
        "ok": action_ok,
        "hard_gate": True,
        "details": [None if r is None else {"run_id": r.get("run_id"), "violations": r.get("contract_violations", []), "action_ok": r.get("gcs_action_ok")} for r in action_runs],
    })

    speed_ok = overshoot(speed_high) is not None and overshoot(speed_base) is not None and overshoot(speed_high) > overshoot(speed_base)
    checks.append({
        "name": "overshoot_increases_with_speed",
        "baseline": overshoot(speed_base),
        "stressed": overshoot(speed_high),
        "ok": speed_ok,
        "hard_gate": True,
    })
    wind_ok = overshoot(wind_high) is not None and overshoot(wind_base) is not None and overshoot(wind_high) > overshoot(wind_base)
    checks.append({
        "name": "overshoot_increases_with_tailwind",
        "baseline": overshoot(wind_base),
        "stressed": overshoot(wind_high),
        "ok": wind_ok,
        "hard_gate": True,
    })
    timeout_vals = [overshoot(timeout_short), overshoot(timeout_default), overshoot(timeout_long)]
    timeout_ok = all(v is not None for v in timeout_vals) and timeout_vals[0] <= timeout_vals[1] <= timeout_vals[2] and timeout_vals[2] > timeout_vals[0]
    checks.append({
        "name": "overshoot_increases_with_timeout",
        "values": timeout_vals,
        "ok": timeout_ok,
        "hard_gate": True,
    })
    hard_ok = all(bool(c.get("ok")) for c in checks if c.get("hard_gate"))
    groups = {
        "speed_response": [
            {"run_id": speed_base.get("run_id") if speed_base else "missing", "x": speed_base.get("speed_m_s") if speed_base else None, "overshoot_m": overshoot(speed_base)},
            {"run_id": speed_high.get("run_id") if speed_high else "missing", "x": speed_high.get("speed_m_s") if speed_high else None, "overshoot_m": overshoot(speed_high)},
        ],
        "wind_response": [
            {"run_id": wind_base.get("run_id") if wind_base else "missing", "x": wind_base.get("wind_m_s") if wind_base else None, "overshoot_m": overshoot(wind_base)},
            {"run_id": wind_high.get("run_id") if wind_high else "missing", "x": wind_high.get("wind_m_s") if wind_high else None, "overshoot_m": overshoot(wind_high)},
        ],
        "timeout_response": [
            {"run_id": timeout_short.get("run_id") if timeout_short else "missing", "x": timeout_short.get("timeout_s") if timeout_short else None, "overshoot_m": overshoot(timeout_short)},
            {"run_id": timeout_default.get("run_id") if timeout_default else "missing", "x": timeout_default.get("timeout_s") if timeout_default else None, "overshoot_m": overshoot(timeout_default)},
            {"run_id": timeout_long.get("run_id") if timeout_long else "missing", "x": timeout_long.get("timeout_s") if timeout_long else None, "overshoot_m": overshoot(timeout_long)},
        ],
    }
    return {
        "satisfied": hard_ok,
        "checks": checks,
        "runs": runs,
        "response_groups": groups,
        "reason": "GCS failsafe triggered and excursion responded monotonically to speed, wind, and timeout" if hard_ok else "GCS failsafe or kinematic response premise did not hold",
    }


def feature_row(point: dict[str, Any]) -> list[float]:
    v = float(point["speed_m_s"])
    w = float(point["wind_m_s"])
    return [v, v * w, w]


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
    pred = 1.0 / (1.0 + np.exp(-np.clip(x @ beta, -40, 40)))
    acc = float(np.mean((pred >= 0.5) == (y >= 0.5)))
    return {
        "ok": True,
        "feature_names": ["intercept", "v", "v*wind", "wind"],
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
                "speed_m_s": float(point["speed_m_s"]),
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
    interp_train_v = {float(v) for v in cfg["interpolation_train_speeds_m_s"]}
    interp_train_w = {float(v) for v in cfg["interpolation_train_winds_m_s"]}
    interp_test_v = {float(v) for v in cfg["interpolation_test_speeds_m_s"]}
    interp_test_w = {float(v) for v in cfg["interpolation_test_winds_m_s"]}
    interp_train = [p for p in points if float(p["speed_m_s"]) in interp_train_v and float(p["wind_m_s"]) in interp_train_w]
    interp_test = [p for p in points if float(p["speed_m_s"]) in interp_test_v and float(p["wind_m_s"]) in interp_test_w]
    extra_train = [
        p for p in points
        if float(p["speed_m_s"]) <= float(cfg["extrapolation_train_max_speed_m_s"])
        and float(p["wind_m_s"]) <= float(cfg["extrapolation_train_max_wind_m_s"])
    ]
    extra_test = [
        p for p in points
        if float(p["speed_m_s"]) >= float(cfg["extrapolation_test_min_speed_m_s"])
        and float(p["wind_m_s"]) >= float(cfg["extrapolation_test_min_wind_m_s"])
    ]
    interpolation = evaluate_split(interp_train, interp_test)
    extrapolation = evaluate_split(extra_train, extra_test)
    combined_rows = interpolation["predictions"] + extrapolation["predictions"]
    combined_acc = None
    if combined_rows:
        combined_acc = sum(1 for r in combined_rows if r["correct"]) / len(combined_rows)
    return {
        "formula": "unsafe probability = sigmoid(beta0 + beta_v*v + beta_vw*v*wind + beta_w*wind)",
        "interpolation": interpolation,
        "extrapolation": extrapolation,
        "combined_heldout_accuracy": combined_acc,
        "holdout_definition": cfg,
    }


def p_stratification_summary(layers: dict[str, dict[str, Any]]) -> dict[str, Any]:
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1]["timeout_s"]))
    counts = [
        {
            "layer": name,
            "timeout_s": float(layer["timeout_s"]),
            "clean_unsafe": int(layer["zone_counts"].get("clean_unsafe", 0)),
            "clean_safe": int(layer["zone_counts"].get("clean_safe", 0)),
            "contract_violated": int(layer["zone_counts"].get("contract_violated", 0)),
        }
        for name, layer in ordered
    ]
    nondecreasing = all(counts[i]["clean_unsafe"] >= counts[i - 1]["clean_unsafe"] for i in range(1, len(counts)))
    return {
        "counts": counts,
        "monotonic_expansion_with_timeout": nondecreasing,
        "conclusion": "clean_unsafe count is nondecreasing as FS_GCS_TIMEOUT lengthens; shorter timeout shrinks the unsafe region" if nondecreasing else "clean_unsafe count did not expand monotonically as FS_GCS_TIMEOUT lengthened",
    }


def boundary_search_summary(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    speeds = [float(v) for v in config["sweep"]["speeds_m_s"]]
    winds = [float(v) for v in config["search"]["winds_m_s"]]
    by = {(float(p["speed_m_s"]), float(p["wind_m_s"])): p for p in points}
    records = []
    total_queries = 0
    for wind in winds:
        lo = 0
        hi = len(speeds) - 1
        queries = []
        first_unsafe = None
        while lo <= hi:
            mid = (lo + hi) // 2
            speed = speeds[mid]
            p = by.get((speed, wind))
            queries.append({"speed_m_s": speed, "wind_m_s": wind, "label": None if p is None else p.get("label")})
            if p is not None and p.get("label") == "clean_unsafe":
                first_unsafe = speed
                hi = mid - 1
            else:
                lo = mid + 1
            if len(queries) >= int(config["search"]["bisection_iterations_per_wind"]):
                break
        total_queries += len(queries)
        records.append({"wind_m_s": wind, "queries": queries, "first_unsafe_speed_m_s": first_unsafe})
    return {
        "strategy": "discrete bisection over speed for each wind, replayed against completed grid run results",
        "query_count": total_queries,
        "full_grid_count": len([p for p in points if p.get("layer") == config["sweep"]["default_layer"]]),
        "records": records,
    }


def verdict_summary(premise: dict[str, Any], default_points: list[dict[str, Any]], prediction: dict[str, Any]) -> dict[str, Any]:
    if not premise.get("satisfied"):
        return {
            "verdict": "INCONCLUSIVE",
            "premise_satisfied": False,
            "robust_clean_unsafe": False,
            "contract_clean_gap": False,
            "prediction_ok": False,
            "reason": "Premise failed: GCS failsafe did not trigger cleanly or excursion did not respond monotonically.",
        }
    clean_unsafe = [p for p in default_points if p.get("label") == "clean_unsafe"]
    contract_violated = [p for p in default_points if p.get("label") == "contract_violated"]
    repeated = [p for p in default_points if int(p.get("repetitions", 0)) >= 2]
    boundary_flips = [p for p in repeated if p.get("boundary_flip")]
    stable_repeated_unsafe = [
        p for p in repeated
        if p.get("label") == "clean_unsafe" and not p.get("boundary_flip")
    ]
    robust_clean_unsafe = bool(len(clean_unsafe) >= 2 and len(stable_repeated_unsafe) >= 2)
    contract_clean_gap = bool(clean_unsafe and all(p.get("contract_clean_all") for p in clean_unsafe) and not contract_violated)
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
            missing.append("clean_unsafe is empty, not contract-clean, or overlaps contract violations")
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
        "stable_repeated_clean_unsafe_count": len(stable_repeated_unsafe),
        "stable_repeated_clean_unsafe_points": [p["point_key"] for p in stable_repeated_unsafe],
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
        "premise": plot_premise(payload["premise"], analysis / "linkloss_premise.png"),
    }
    if payload["verdict"]["verdict"] != "INCONCLUSIVE":
        default_points = payload["default_grid"]["points"]
        plots.update({
            "result_field": plot_result_field(default_points, analysis / "linkloss_result_field.png"),
            "severity": plot_severity_heatmap(default_points, analysis / "linkloss_severity_heatmap.png"),
            "p_stratification": plot_p_stratification(payload["p_stratification"]["layers"], analysis / "linkloss_p_stratification.png"),
            "train_test": plot_train_test(payload["predictive_rule"], analysis / "linkloss_train_test.png"),
        })
    return plots


def write_report(payload: dict[str, Any]) -> str:
    report = PLANC_ROOT / "results" / "linkloss_excursion_report.md"
    verdict = payload["verdict"]
    cfg = payload["config"]
    lines: list[str] = []
    lines.append(f"VERDICT: {verdict['verdict']}")
    lines.append("")
    lines.append("# planc GCS link-loss boundary-excursion second scenario")
    lines.append("")
    lines.append("## Four decisive criteria")
    lines.append("")
    lines.append(f"- Premise satisfied: **{verdict.get('premise_satisfied')}**.")
    lines.append(
        f"- Robust contract-clean unsafe region: **{verdict.get('robust_clean_unsafe')}**; "
        f"clean_unsafe count={verdict.get('clean_unsafe_count', 0)}, "
        f"stable repeated clean_unsafe count={verdict.get('stable_repeated_clean_unsafe_count', 0)}, "
        f"boundary flips reported={', '.join(verdict.get('boundary_flip_points', [])) or 'none'}."
    )
    lines.append(f"- Link-loss failsafe contract clean and PGFUZZ-invisible: **{verdict.get('contract_clean_gap')}**; contract_violated count={verdict.get('contract_violated_count', 0)}.")
    lines.append(f"- Held-out prediction with extrapolation >= 90%: **{verdict.get('prediction_ok')}**; interpolation={fmt(verdict.get('interpolation_accuracy'), 3)}, extrapolation={fmt(verdict.get('extrapolation_accuracy'), 3)}, combined={fmt(verdict.get('combined_heldout_accuracy'), 3)}.")
    lines.append("")
    lines.append(f"Decision reason: {verdict.get('reason')}")
    lines.append("")
    lines.append("## Premise")
    lines.append("")
    lines.append(f"Premise conclusion: **{payload['premise']['satisfied']}** - {payload['premise']['reason']}.")
    lines.append("")
    lines.append("| run | speed m/s | wind m/s | timeout s | label | overshoot m | observed timeout s | parsed log | oracle |")
    lines.append("| --- | ---: | ---: | ---: | --- | ---: | ---: | --- | --- |")
    for run in payload["premise"]["runs"]:
        csv_path = run.get("csv_path", "n/a")
        oracle_path = str(Path(csv_path).with_suffix(".oracle.json")) if csv_path != "n/a" else "n/a"
        lines.append(
            f"| {run.get('run_id')} | {fmt(run.get('speed_m_s'))} | {fmt(run.get('wind_m_s'))} | "
            f"{fmt(run.get('timeout_s'))} | {run.get('label')} | {fmt(run.get('severity_overshoot_m'))} | "
            f"{fmt(run.get('gcs_failsafe_timeout_observed_s'))} | {rel(csv_path)} | {rel(oracle_path)} |"
        )
    lines.append("")
    for check in payload["premise"].get("checks", []):
        gate = "hard" if check.get("hard_gate") else "reported"
        detail = ""
        if "baseline" in check or "stressed" in check:
            detail = f": baseline={fmt(check.get('baseline'))}, stressed={fmt(check.get('stressed'))}"
        elif "values" in check:
            detail = f": values={[fmt(v) for v in check.get('values', [])]}"
        lines.append(f"- {check['name']} ({gate}){detail}, ok={check.get('ok')}.")
    lines.append("")
    lines.append("## Scenario")
    lines.append("")
    lines.append(
        f"Fixed P uses `FS_GCS_ENABLE=1` (RTL), `FS_OPTIONS=0`, `SYSID_MYGCS=255`, "
        f"`FENCE_ENABLE=1`, `FENCE_TYPE=2` circular fence, `FENCE_RADIUS={fmt(cfg['experiment']['fence_radius_m'], 0)} m`, "
        "`FENCE_ACTION=0` Report Only, and `AVOID_ENABLE=0`. All controlled parameters are read back per run."
    )
    lines.append(
        f"The AUTO mission waypoint is {fmt(cfg['experiment']['waypoint_distance_m'], 0)} m east of home, outside the operator fence. "
        f"The GCS heartbeat is stopped at {fmt(payload['derived']['trigger_distance_m'], 0)} m from home, "
        f"which is {fmt(cfg['experiment']['d_inside_m'], 0)} m inside the fence."
    )
    lines.append(
        f"M scans command speed over {cfg['sweep']['speeds_m_s']} m/s using `MAV_CMD_DO_CHANGE_SPEED`; "
        f"E scans outbound tailwind over {cfg['sweep']['winds_m_s']} m/s with `SIM_WIND_DIR=270` and `SIM_WIND_TURB=0`. "
        "`WPNAV_SPEED` is fixed above the maximum command speed, so it is a cap rather than the scanned input."
    )
    lines.append(
        f"Oracle: unsafe means `max_distance > R + {fmt(cfg['experiment']['fence_tolerance_m'])} m`; severity is `max_distance - R`. "
        "The boundary is the configured operator fence, not a constructed analysis line."
    )
    lines.append("")
    if verdict["verdict"] != "INCONCLUSIVE":
        lines.append("## Three-Zone Field")
        lines.append("")
        counts = payload["default_grid"]["zone_counts"]
        lines.append(
            f"Default layer `{payload['default_grid']['layer']}` (`FS_GCS_TIMEOUT={fmt(payload['default_grid']['timeout_s'], 0)} s`) counts: "
            f"clean_safe={counts.get('clean_safe', 0)}, clean_unsafe={counts.get('clean_unsafe', 0)}, "
            f"contract_violated={counts.get('contract_violated', 0)}, blocked={counts.get('blocked', 0)}."
        )
        lines.append("")
        lines.append("| speed m/s | wind m/s | label | overshoot m | max dist m | observed timeout s | stable | runs |")
        lines.append("| ---: | ---: | --- | ---: | ---: | ---: | --- | --- |")
        for p in sorted(payload["default_grid"]["points"], key=lambda r: (float(r["speed_m_s"]), float(r["wind_m_s"]))):
            lines.append(
                f"| {fmt(p['speed_m_s'], 0)} | {fmt(p['wind_m_s'], 0)} | {p.get('label')} | "
                f"{fmt(p.get('severity_overshoot_m'))} | {fmt(p.get('max_distance_m'))} | "
                f"{fmt(p.get('gcs_failsafe_timeout_observed_s'))} | {p.get('stable_binary')} | "
                f"{', '.join(str(r) for r in p.get('run_ids', []))} |"
            )
        lines.append("")
        lines.append(
            "PGFUZZ-invisible check: every `clean_unsafe` point requires a GCS failsafe ERR plus RTL mode change at the configured timeout, "
            "with no unrelated ERR/EV/STATUSTEXT failsafes and with parameter readback success. "
            "Report-Only fence breach ERR/STATUSTEXT records are explicitly treated as oracle measurement reports, not contract violations."
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
        lines.append("| layer | FS_GCS_TIMEOUT s | clean_unsafe | clean_safe | contract_violated |")
        lines.append("| --- | ---: | ---: | ---: | ---: |")
        for row in p_summary["counts"]:
            lines.append(
                f"| {row['layer']} | {fmt(row['timeout_s'], 0)} | {row['clean_unsafe']} | "
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
            "For each run id in the field table, audit files are `planc/logs/<run_id>_params.json`, "
            "`planc/logs/<run_id>_parsed.csv`, and `planc/logs/<run_id>_parsed.oracle.json`."
        )
        lines.append("")
    lines.append("## Unified Method Statement")
    lines.append("")
    lines.append(
        "The RTL energy scenario and this GCS link-loss scenario are both threshold-insufficiency specification gaps: "
        "the former is an energy-budget threshold, while this one is a data-link time-budget threshold. "
        "In both, ArduCopter follows the configured failsafe contract, but a legal operating condition crosses an external safety oracle."
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
        "This is ArduCopter SITL, not HITL. The verdict applies to this SITL vehicle, parameter set, and pre-registered link-loss boundary excursion scenario. "
        "Logs, readbacks, parsed CSVs, and oracle sidecars are kept under `planc/logs/` for independent audit."
    )
    lines.append("")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(lines), encoding="utf-8")
    return str(report)


def build_payload(config: dict[str, Any], env: dict[str, Any], premise: dict[str, Any], grid_runs: list[dict[str, Any]]) -> dict[str, Any]:
    speeds = [float(v) for v in config["sweep"]["speeds_m_s"]]
    winds = [float(v) for v in config["sweep"]["winds_m_s"]]
    default_layer = str(config["sweep"]["default_layer"])
    layers: dict[str, dict[str, Any]] = {}
    for layer_name, layer_cfg in config["sweep"]["p_layers"].items():
        points = aggregate_layer(grid_runs, layer_name, speeds, winds)
        layers[layer_name] = {
            "layer": layer_name,
            "timeout_s": float(layer_cfg["FS_GCS_TIMEOUT"]),
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
        "derived": {
            "trigger_distance_m": trigger_distance_m(config),
        },
        "premise": premise,
        "default_grid": {
            "layer": default_layer,
            "timeout_s": float(config["sweep"]["p_layers"][default_layer]["FS_GCS_TIMEOUT"]),
            "speeds_m_s": speeds,
            "winds_m_s": winds,
            "points": default_points,
            "zone_counts": zone_counts(default_points),
        },
        "p_stratification": {
            "layers": layers,
            "summary": p_stratification_summary(layers) if premise.get("satisfied") else {"counts": [], "monotonic_expansion_with_timeout": None, "conclusion": "not evaluated"},
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


def premise_plan(config: dict[str, Any]) -> list[dict[str, Any]]:
    p = config["premise"]
    return [
        {
            "kind": "speed_base",
            "speed_m_s": float(p["speed_response"]["baseline_speed_m_s"]),
            "wind_m_s": float(p["speed_response"]["wind_m_s"]),
            "timeout_s": float(p["speed_response"]["timeout_s"]),
        },
        {
            "kind": "speed_high",
            "speed_m_s": float(p["speed_response"]["stressed_speed_m_s"]),
            "wind_m_s": float(p["speed_response"]["wind_m_s"]),
            "timeout_s": float(p["speed_response"]["timeout_s"]),
        },
        {
            "kind": "wind_base",
            "speed_m_s": float(p["wind_response"]["speed_m_s"]),
            "wind_m_s": float(p["wind_response"]["baseline_wind_m_s"]),
            "timeout_s": float(p["wind_response"]["timeout_s"]),
        },
        {
            "kind": "wind_high",
            "speed_m_s": float(p["wind_response"]["speed_m_s"]),
            "wind_m_s": float(p["wind_response"]["stressed_wind_m_s"]),
            "timeout_s": float(p["wind_response"]["timeout_s"]),
        },
        {
            "kind": "timeout_short",
            "speed_m_s": float(p["timeout_response"]["speed_m_s"]),
            "wind_m_s": float(p["timeout_response"]["wind_m_s"]),
            "timeout_s": float(p["timeout_response"]["timeouts_s"][0]),
        },
        {
            "kind": "timeout_default",
            "speed_m_s": float(p["timeout_response"]["speed_m_s"]),
            "wind_m_s": float(p["timeout_response"]["wind_m_s"]),
            "timeout_s": float(p["timeout_response"]["timeouts_s"][1]),
        },
        {
            "kind": "timeout_long",
            "speed_m_s": float(p["timeout_response"]["speed_m_s"]),
            "wind_m_s": float(p["timeout_response"]["wind_m_s"]),
            "timeout_s": float(p["timeout_response"]["timeouts_s"][2]),
        },
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the planc GCS link-loss boundary-excursion second scenario.")
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "linkloss_excursion_config.yaml")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    results_dir = PLANC_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    premise_partial_path = results_dir / "linkloss_excursion_premise_partial.json"
    grid_partial_path = results_dir / "linkloss_excursion_results_partial.json"
    final_path = results_dir / "linkloss_excursion_results.json"

    env = probe_environment(config, REPO_ROOT)
    write_env(env, results_dir / "env_linkloss_excursion.json")
    probe = connectivity_probe(config, env)
    write_env(env, results_dir / "env_linkloss_excursion.json")
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
        premise_runs = list(json.loads(premise_partial_path.read_text(encoding="utf-8")).get("runs", []))
    if args.resume and grid_partial_path.exists():
        grid_runs = list(json.loads(grid_partial_path.read_text(encoding="utf-8")).get("runs", []))

    for spec in premise_plan(config):
        run_id = premise_run_id(str(spec["kind"]))
        run = run_or_reuse(
            config,
            premise_runs,
            premise_partial_path,
            run_id=run_id,
            run_kind="premise",
            layer=None,
            speed_m_s=float(spec["speed_m_s"]),
            wind_m_s=float(spec["wind_m_s"]),
            timeout_s=float(spec["timeout_s"]),
            rep_index=1,
            roles=["premise", str(spec["kind"])],
        )
        run["premise_kind"] = str(spec["kind"])
        write_json(premise_partial_path, {"runs": premise_runs})

    premise = premise_summary(premise_runs)
    write_json(premise_partial_path, {"premise": premise, "runs": premise_runs})
    if not premise.get("satisfied"):
        payload = build_payload(config, env, premise, grid_runs)
        write_json(final_path, payload)
        print(f"INCONCLUSIVE: premise failed; report={payload['artifacts']['report']}", flush=True)
        return 0

    speeds = [float(v) for v in config["sweep"]["speeds_m_s"]]
    winds = [float(v) for v in config["sweep"]["winds_m_s"]]
    default_layer = str(config["sweep"]["default_layer"])
    for speed in speeds:
        for wind in winds:
            run_or_reuse(
                config,
                grid_runs,
                grid_partial_path,
                run_id=run_id_for(default_layer, speed, wind, 1),
                run_kind="grid",
                layer=default_layer,
                speed_m_s=speed,
                wind_m_s=wind,
                timeout_s=float(config["sweep"]["p_layers"][default_layer]["FS_GCS_TIMEOUT"]),
                rep_index=1,
                roles=["default_grid", "p_layer"],
            )

    default_points_once = aggregate_layer(grid_runs, default_layer, speeds, winds)
    near_points = find_near_boundary(default_points_once, speeds, winds)
    for point in near_points:
        for rep in range(2, int(config["sweep"]["near_boundary_repetitions"]) + 1):
            run_or_reuse(
                config,
                grid_runs,
                grid_partial_path,
                run_id=run_id_for(default_layer, float(point["speed_m_s"]), float(point["wind_m_s"]), rep),
                run_kind="grid",
                layer=default_layer,
                speed_m_s=float(point["speed_m_s"]),
                wind_m_s=float(point["wind_m_s"]),
                timeout_s=float(config["sweep"]["p_layers"][default_layer]["FS_GCS_TIMEOUT"]),
                rep_index=rep,
                roles=["near_boundary_repeat", "default_grid"],
            )

    for layer in config["sweep"]["p_layers"]:
        if str(layer) == default_layer:
            continue
        for speed in speeds:
            for wind in winds:
                run_or_reuse(
                    config,
                    grid_runs,
                    grid_partial_path,
                    run_id=run_id_for(str(layer), speed, wind, 1),
                    run_kind="grid",
                    layer=str(layer),
                    speed_m_s=speed,
                    wind_m_s=wind,
                    timeout_s=float(config["sweep"]["p_layers"][layer]["FS_GCS_TIMEOUT"]),
                    rep_index=1,
                    roles=["p_layer"],
                )

    payload = build_payload(config, env, premise, grid_runs)
    write_json(final_path, payload)
    print(f"COMPLETE: verdict={payload['verdict']['verdict']} report={payload['artifacts']['report']} results={final_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
