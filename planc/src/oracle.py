from __future__ import annotations

import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any

from pymavlink import mavutil


EARTH_RADIUS_M = 6378137.0

COPTER_MODES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
}

ERROR_SUBSYSTEMS = {
    1: "MAIN",
    2: "RADIO",
    3: "COMPASS",
    5: "FAILSAFE_RADIO",
    6: "FAILSAFE_BATT",
    7: "FAILSAFE_GPS",
    8: "FAILSAFE_GCS",
    9: "FAILSAFE_FENCE",
    10: "FLIGHT_MODE",
    11: "GPS",
    12: "CRASH_CHECK",
    13: "FLIP",
    16: "EKFCHECK",
    17: "FAILSAFE_EKFINAV",
    18: "BARO",
    19: "CPU",
    20: "FAILSAFE_ADSB",
    21: "TERRAIN",
    22: "NAVIGATION",
    23: "FAILSAFE_TERRAIN",
    24: "EKF_PRIMARY",
    25: "THRUST_LOSS_CHECK",
    26: "FAILSAFE_SENSORS",
    27: "FAILSAFE_LEAK",
    28: "PILOT_INPUT",
    29: "FAILSAFE_VIBE",
    30: "INTERNAL_ERROR",
    31: "FAILSAFE_DEADRECKON",
}

ERROR_CODES = {
    (22, 2): "FAILED_TO_SET_DESTINATION",
    (22, 3): "RESTARTED_RTL",
    (22, 4): "FAILED_CIRCLE_INIT",
    (22, 5): "DEST_OUTSIDE_FENCE",
    (22, 6): "RTL_MISSING_RNGFND",
}

EVENT_NAMES = {
    10: "ARMED",
    11: "DISARMED",
    15: "AUTO_ARMED",
    17: "LAND_COMPLETE_MAYBE",
    18: "LAND_COMPLETE",
    19: "LOST_GPS",
    21: "FLIP_START",
    22: "FLIP_END",
    25: "SET_HOME",
    26: "SET_SIMPLE_ON",
    27: "SET_SIMPLE_OFF",
    28: "NOT_LANDED",
    29: "SET_SUPERSIMPLE_ON",
    30: "AUTOTUNE_INITIALISED",
    31: "AUTOTUNE_OFF",
    32: "AUTOTUNE_RESTART",
    33: "AUTOTUNE_SUCCESS",
    34: "AUTOTUNE_FAILED",
    35: "AUTOTUNE_REACHED_LIMIT",
    36: "AUTOTUNE_PILOT_TESTING",
    37: "AUTOTUNE_SAVEDGAINS",
    38: "SAVE_TRIM",
    39: "SAVEWP_ADD_WP",
    41: "FENCE_ENABLE",
    42: "FENCE_DISABLE",
    43: "ACRO_TRAINER_OFF",
    44: "ACRO_TRAINER_LEVELING",
    45: "ACRO_TRAINER_LIMITED",
    46: "GRIPPER_GRAB",
    47: "GRIPPER_RELEASE",
    49: "PARACHUTE_DISABLED",
    50: "PARACHUTE_ENABLED",
    51: "PARACHUTE_RELEASED",
    52: "LANDING_GEAR_DEPLOYED",
    53: "LANDING_GEAR_RETRACTED",
    54: "MOTORS_EMERGENCY_STOPPED",
    55: "MOTORS_EMERGENCY_STOP_CLEARED",
    56: "MOTORS_INTERLOCK_DISABLED",
    57: "MOTORS_INTERLOCK_ENABLED",
    58: "ROTOR_RUNUP_COMPLETE",
    59: "ROTOR_SPEED_BELOW_CRITICAL",
    60: "EKF_ALT_RESET",
    61: "LAND_CANCELLED_BY_PILOT",
    62: "EKF_YAW_RESET",
    63: "AVOIDANCE_ADSB_ENABLE",
    64: "AVOIDANCE_ADSB_DISABLE",
    65: "AVOIDANCE_PROXIMITY_ENABLE",
    66: "AVOIDANCE_PROXIMITY_DISABLE",
    67: "GPS_PRIMARY_CHANGED",
    71: "ZIGZAG_STORE_A",
    72: "ZIGZAG_STORE_B",
    73: "LAND_REPO_ACTIVE",
    74: "STANDBY_ENABLE",
    75: "STANDBY_DISABLE",
    80: "FENCE_FLOOR_ENABLE",
    81: "FENCE_FLOOR_DISABLE",
    85: "EK3_SOURCES_SET_TO_PRIMARY",
    86: "EK3_SOURCES_SET_TO_SECONDARY",
    87: "EK3_SOURCES_SET_TO_TERTIARY",
    90: "AIRSPEED_PRIMARY_CHANGED",
    163: "SURFACED",
    164: "NOT_SURFACED",
    165: "BOTTOMED",
    166: "NOT_BOTTOMED",
}

BAD_EVENT_NAMES = {
    "LOST_GPS",
    "AUTOTUNE_FAILED",
    "PARACHUTE_RELEASED",
    "MOTORS_EMERGENCY_STOPPED",
    "LAND_CANCELLED_BY_PILOT",
    "ROTOR_SPEED_BELOW_CRITICAL",
}

BAD_TEXT_MARKERS = (
    "crash",
    "ekf failsafe",
    "battery failsafe",
    "radio failsafe",
    "gcs failsafe",
    "arming checks failed",
)


def _field(data: dict[str, Any], *names: str) -> Any:
    for name in names:
        if name in data:
            return data[name]
    return None


def _time_s(data: dict[str, Any], msg: Any) -> float | None:
    if "TimeUS" in data:
        return float(data["TimeUS"]) / 1.0e6
    if "TimeMS" in data:
        return float(data["TimeMS"]) / 1000.0
    return None


def _latlon(value: Any) -> float | None:
    if value is None:
        return None
    value = float(value)
    if abs(value) > 1000:
        return value / 1.0e7
    return value


def horizontal_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def _mode_name(data: dict[str, Any]) -> str:
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


def _nearest_distance(rows: list[dict[str, Any]], t_s: float | None) -> float | None:
    if not rows or t_s is None:
        return None
    row = min(rows, key=lambda r: abs(float(r["time_s"]) - t_s))
    return float(row["distance_m"])


def _annotate_modes(rows: list[dict[str, Any]], modes: list[dict[str, Any]]) -> None:
    modes = sorted(modes, key=lambda m: m["time_s"])
    idx = 0
    current = ""
    for row in rows:
        t = float(row["time_s"])
        while idx < len(modes) and float(modes[idx]["time_s"]) <= t:
            current = str(modes[idx]["mode"])
            idx += 1
        row["mode"] = current


def _nearest_row(rows: list[dict[str, Any]], t_s: float | None) -> dict[str, Any] | None:
    if not rows or t_s is None:
        return None
    return min(rows, key=lambda r: abs(float(r["time_s"]) - t_s))


def _percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (rank - lo)


def parse_dataflash(
    bin_path: Path,
    csv_path: Path,
    home: dict[str, Any],
    fence_radius_m: float,
    expected_action_modes: list[str],
    action_latency_s: float,
    expect_breach: bool,
    target_bearing_deg: float | None = None,
    commanded_speed_m_s: float | None = None,
    speed_audit_min_distance_m: float = 35.0,
    speed_audit_max_distance_m: float | None = None,
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    pos_rows: list[dict[str, Any]] = []
    gps_rows: list[dict[str, Any]] = []
    xkf_velocity_rows: list[dict[str, Any]] = []
    gps_speed_rows: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    event_records: list[dict[str, Any]] = []
    fence_events: list[dict[str, Any]] = []
    other_errors: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    fence_msgs: list[dict[str, Any]] = []
    parm_rows: list[dict[str, Any]] = []
    start_time: float | None = None

    home_lat = float(home["lat"])
    home_lon = float(home["lon"])
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue
        data = msg.to_dict()
        ts = _time_s(data, msg)
        if ts is None:
            continue
        if start_time is None:
            start_time = ts
        rel_t = ts - start_time

        if mtype == "PARM":
            parm_rows.append({"time_s": rel_t, "name": _field(data, "Name"), "value": _field(data, "Value")})
        elif mtype == "MODE":
            modes.append({
                "time_s": rel_t,
                "mode": _mode_name(data),
                "reason": _field(data, "Rsn", "Reason"),
                "raw": data,
            })
        elif mtype == "ERR":
            subsys = int(_field(data, "Subsys", "SubSystem") or -1)
            ecode = int(_field(data, "ECode", "Code") or 0)
            rec = {
                "time_s": rel_t,
                "subsys": subsys,
                "subsys_name": ERROR_SUBSYSTEMS.get(subsys, str(subsys)),
                "ecode": ecode,
                "ecode_name": ERROR_CODES.get((subsys, ecode), str(ecode)),
                "raw": data,
            }
            if subsys == 9:
                fence_events.append(rec)
            elif ecode != 0:
                other_errors.append(rec)
        elif mtype == "EV":
            event_id = int(_field(data, "Id") or -1)
            event_records.append({
                "time_s": rel_t,
                "id": event_id,
                "name": EVENT_NAMES.get(event_id, str(event_id)),
                "raw": data,
            })
        elif mtype in {"MSG", "STAT"}:
            text = str(_field(data, "Message", "Msg", "Text") or "")
            rec = {"time_s": rel_t, "text": text, "raw": data}
            messages.append(rec)
            if "Fence" in text or "fence" in text:
                fence_msgs.append(rec)
        elif mtype == "FNCE":
            fence_msgs.append({"time_s": rel_t, "text": "FNCE", "raw": data})
        elif mtype == "XKF1":
            core = _field(data, "C")
            if core is None or int(core) == 0:
                vn = float(_field(data, "VN") or 0.0)
                ve = float(_field(data, "VE") or 0.0)
                forward = None
                if target_bearing_deg is not None:
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
                spd = _field(data, "Spd")
                if spd is not None:
                    gps_speed_rows.append({
                        "time_s": rel_t,
                        "ground_speed_m_s": float(spd),
                        "course_deg": _field(data, "GCrs"),
                    })
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

    rows = pos_rows if len(pos_rows) >= 10 else gps_rows
    rows = sorted(rows, key=lambda r: r["time_s"])
    _annotate_modes(rows, modes)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["time_s", "source", "lat", "lon", "alt", "distance_m", "mode"])
        writer.writeheader()
        writer.writerows(rows)

    max_distance = max((float(r["distance_m"]) for r in rows), default=0.0)
    max_row = max(rows, key=lambda r: float(r["distance_m"])) if rows else None
    first_fence_event = next((e for e in fence_events if int(e["ecode"]) != 0), None)
    first_fence_msg = next((m for m in fence_msgs if "fence breached" in str(m.get("text", "")).lower()), None)
    fence_time = None
    if first_fence_event is not None:
        fence_time = float(first_fence_event["time_s"])
    elif first_fence_msg is not None:
        fence_time = float(first_fence_msg["time_s"])

    action_started = False
    action_mode = None
    action_time = None
    if fence_time is not None:
        for mode in modes:
            dt = float(mode["time_s"]) - fence_time
            if 0 <= dt <= action_latency_s and str(mode["mode"]) in expected_action_modes:
                action_started = True
                action_mode = str(mode["mode"])
                action_time = float(mode["time_s"])
                break

    bad_messages = []
    for rec in messages:
        text = str(rec.get("text", ""))
        low = text.lower()
        if "fence breached" in low:
            continue
        if any(marker in low for marker in BAD_TEXT_MARKERS):
            bad_messages.append(rec)

    fence_breach_detected = fence_time is not None
    bad_events = [rec for rec in event_records if rec["name"] in BAD_EVENT_NAMES]
    if expect_breach:
        contract_clean = fence_breach_detected and action_started and not other_errors and not bad_messages and not bad_events
    else:
        contract_clean = (not fence_breach_detected) and not other_errors and not bad_messages and not bad_events

    speed_audit: dict[str, Any] | None = None
    if target_bearing_deg is not None and commanded_speed_m_s is not None:
        audit_max_dist = speed_audit_max_distance_m if speed_audit_max_distance_m is not None else fence_radius_m * 0.95
        audit_rows = []
        for vel in xkf_velocity_rows:
            nearest = _nearest_row(rows, float(vel["time_s"]))
            if nearest is None:
                continue
            dist = float(nearest["distance_m"])
            if dist < speed_audit_min_distance_m or dist > audit_max_dist:
                continue
            if fence_time is not None and float(vel["time_s"]) > fence_time:
                continue
            forward = vel.get("forward_speed_m_s")
            if forward is None:
                continue
            audit_rows.append({
                **vel,
                "distance_m": dist,
            })
        forward_speeds = [float(r["forward_speed_m_s"]) for r in audit_rows]
        ground_speeds = [float(r["ground_speed_m_s"]) for r in audit_rows]
        median_forward = statistics.median(forward_speeds) if forward_speeds else None
        p95_forward = _percentile(forward_speeds, 0.95)
        max_forward = max(forward_speeds) if forward_speeds else None
        speed_audit = {
            "source": "XKF1 primary core VN/VE projected onto target bearing",
            "target_bearing_deg": float(target_bearing_deg),
            "commanded_speed_m_s": float(commanded_speed_m_s),
            "audit_distance_window_m": [float(speed_audit_min_distance_m), float(audit_max_dist)],
            "samples": len(audit_rows),
            "median_forward_speed_m_s": median_forward,
            "mean_forward_speed_m_s": statistics.fmean(forward_speeds) if forward_speeds else None,
            "p95_forward_speed_m_s": p95_forward,
            "max_forward_speed_m_s": max_forward,
            "median_ground_speed_m_s": statistics.median(ground_speeds) if ground_speeds else None,
            "max_ground_speed_m_s": max(ground_speeds) if ground_speeds else None,
            "median_forward_error_m_s": None if median_forward is None else median_forward - float(commanded_speed_m_s),
            "p95_forward_error_m_s": None if p95_forward is None else p95_forward - float(commanded_speed_m_s),
            "max_forward_error_m_s": None if max_forward is None else max_forward - float(commanded_speed_m_s),
        }

    result = {
        "bin_path": str(bin_path),
        "csv_path": str(csv_path),
        "position_source": rows[0]["source"] if rows else None,
        "samples": len(rows),
        "fence_radius_m": fence_radius_m,
        "max_distance_m": max_distance,
        "max_overshoot_m": max(0.0, max_distance - fence_radius_m),
        "max_distance_sample": max_row,
        "fence_breach_detected": fence_breach_detected,
        "fence_breach_time_s": fence_time,
        "fence_breach_distance_m": _nearest_distance(rows, fence_time),
        "fence_events": fence_events,
        "fence_messages": fence_msgs[:20],
        "modes": modes,
        "expected_action_modes": expected_action_modes,
        "action_started": action_started,
        "action_mode": action_mode,
        "action_time_s": action_time,
        "event_records": event_records,
        "bad_events": bad_events,
        "other_errors": other_errors,
        "bad_messages": bad_messages,
        "contract_clean": contract_clean,
        "expect_breach": expect_breach,
        "parm_rows_count": len(parm_rows),
        "velocity_rows_count": len(xkf_velocity_rows),
        "gps_speed_rows_count": len(gps_speed_rows),
        "speed_audit": speed_audit,
    }
    sidecar = csv_path.with_suffix(".oracle.json")
    sidecar.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result
