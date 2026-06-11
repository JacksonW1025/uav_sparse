from __future__ import annotations

import csv
import json
import math
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


def parse_dataflash(
    bin_path: Path,
    csv_path: Path,
    home: dict[str, Any],
    fence_radius_m: float,
    expected_action_modes: list[str],
    action_latency_s: float,
    expect_breach: bool,
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    pos_rows: list[dict[str, Any]] = []
    gps_rows: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
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
        elif mtype in {"MSG", "STAT"}:
            text = str(_field(data, "Message", "Msg", "Text") or "")
            rec = {"time_s": rel_t, "text": text, "raw": data}
            messages.append(rec)
            if "Fence" in text or "fence" in text:
                fence_msgs.append(rec)
        elif mtype == "FNCE":
            fence_msgs.append({"time_s": rel_t, "text": "FNCE", "raw": data})

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
    if expect_breach:
        contract_clean = fence_breach_detected and action_started and not other_errors and not bad_messages
    else:
        contract_clean = (not fence_breach_detected) and not other_errors and not bad_messages

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
        "other_errors": other_errors,
        "bad_messages": bad_messages,
        "contract_clean": contract_clean,
        "expect_breach": expect_breach,
        "parm_rows_count": len(parm_rows),
    }
    sidecar = csv_path.with_suffix(".oracle.json")
    sidecar.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result
