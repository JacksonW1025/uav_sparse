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
from injector import send_guided_velocity_local_ned
from oracle import BAD_EVENT_NAMES, COPTER_MODES, ERROR_SUBSYSTEMS, EVENT_NAMES
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

FENCE_ERR_SUBSYS = 9
CRASH_ERR_SUBSYS = 12
FENCE_ALT_MIN_BIT = 8
FENCE_BREACHED_REASON = 10
GCS_COMMAND_REASON = 2
CONFIDENT_LABELS = {"clean_safe", "clean_unsafe"}
ZONE_COLORS = {
    "clean_safe": "#2a9d8f",
    "clean_unsafe": "#d62828",
    "ambiguous": "#f4a261",
    "contract_violated": "#6c757d",
    "blocked": "#adb5bd",
}
DIRTY_TEXT_MARKERS = (
    "battery failsafe",
    "ekf failsafe",
    "terrain failsafe",
    "deadreckon",
    "radio failsafe",
    "gcs failsafe",
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


def load_json(path: Path, default: dict[str, Any] | None = None) -> dict[str, Any]:
    if not path.exists():
        return {} if default is None else default
    return json.loads(path.read_text(encoding="utf-8"))


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


def model_spec(config: dict[str, Any], model_name: str) -> dict[str, Any]:
    return dict(config["models"][model_name])


def model_path(config: dict[str, Any], model_name: str) -> Path:
    raw = Path(str(model_spec(config, model_name)["json"]))
    return raw if raw.is_absolute() else REPO_ROOT / raw


def config_with_model(config: dict[str, Any], model_name: str) -> dict[str, Any]:
    cfg = config_copy(config)
    base_model = str(config["sitl"].get("model", "quad"))
    src = model_path(config, model_name)
    cfg["sitl"]["model"] = f"{base_model}:{src.name}"
    cfg["sitl"]["model_json_source"] = str(src)
    cfg["experiment"]["model_name"] = model_name
    cfg["experiment"]["model_mass_kg"] = float(model_spec(config, model_name)["mass_kg"])
    cfg["experiment"]["model_mass_multiplier"] = float(model_spec(config, model_name)["mass_multiplier"])
    return cfg


def controlled_params(config: dict[str, Any], overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    params = dict(config.get("baseline_params", {}))
    if overrides:
        params.update(overrides)
    return params


def active_fence_params(config: dict[str, Any], fence_alt_min_m: float) -> dict[str, Any]:
    params = dict(config.get("active_fence_params", {}))
    params["FENCE_ALT_MIN"] = float(fence_alt_min_m)
    return params


def layer_alt_min(config: dict[str, Any], layer: str) -> float:
    return float(config["sweep"]["p_layers"][layer]["FENCE_ALT_MIN"])


def effective_alt_layer(config: dict[str, Any], layer: str) -> str:
    if str(layer).startswith("noise_"):
        return str(config["sweep"]["default_layer"])
    return str(layer)


def point_key(layer: str, descent_rate_m_s: float, model_name: str) -> str:
    return f"{layer}_d{int(round(float(descent_rate_m_s) * 10)):03d}_{model_name}"


def run_id_for(layer: str, descent_rate_m_s: float, model_name: str, rep_index: int, prefix: str = "minalt") -> str:
    return f"{prefix}_{layer}_d{int(round(float(descent_rate_m_s) * 10)):03d}_{model_name}_r{int(rep_index):02d}"


def premise_run_id(kind: str, model_name: str | None = None) -> str:
    suffix = "" if model_name is None else f"_{model_name}"
    return f"minalt_premise_{kind}{suffix}"


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


def _clean_param_records(records: list[dict[str, Any]]) -> bool:
    return bool(records) and all(bool(r.get("ok")) for r in records)


def _is_armed(heartbeat: Any) -> bool:
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def send_fence_enable(master, enable: bool = True, timeout_s: float = 5.0) -> dict[str, Any]:
    command = mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        1.0 if enable else 0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if int(getattr(msg, "command", -1)) == int(command):
            return msg.to_dict()
    return {"command": int(command), "result": None, "timeout": True}


def prepare_flight(master, config: dict[str, Any]) -> None:
    request_streams(master, rate_hz=int(config["experiment"].get("stream_hz", 20)))
    wait_position(master, timeout_s=30.0)
    wait_position_stable(master)
    set_mode(master, "GUIDED")
    arm(master)
    alt_m = float(config["experiment"]["takeoff_alt_m"])
    command_takeoff(master, alt_m)
    wait_altitude(master, alt_m, timeout_s=55.0)
    set_mode(master, "GUIDED")


def run_minalt_flight(
    master,
    config: dict[str, Any],
    *,
    fence_alt_min_m: float,
    descent_rate_m_s: float,
    active_pm: ParamManager,
) -> dict[str, Any]:
    prepare_flight(master, config)
    active_params = active_fence_params(config, fence_alt_min_m)
    active_pm.apply(active_params)
    fence_ack = send_fence_enable(master, True)

    exp = config["experiment"]
    stream_hz = float(exp.get("stream_hz", 20))
    dt = 1.0 / max(stream_hz, 1.0)
    settle_end = time.time() + float(exp.get("pre_fence_activation_settle_s", 1.5))
    statustext: list[dict[str, Any]] = []
    fence_status: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    realtime_alt_samples: list[dict[str, Any]] = []
    armed = True

    while time.time() < settle_end:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT", "FENCE_STATUS"], blocking=True, timeout=0.2)
        if msg is None:
            continue
        now = time.time()
        if msg.get_type() == "GLOBAL_POSITION_INT":
            realtime_alt_samples.append({"wall_s": None, "relative_alt_m": float(msg.relative_alt) / 1000.0})
        elif msg.get_type() == "HEARTBEAT":
            mode = mode_name(master, msg)
            armed = _is_armed(msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": None, "mode": mode})
        elif msg.get_type() == "STATUSTEXT":
            statustext.append({"wall_s": None, "text": str(getattr(msg, "text", ""))})
        elif msg.get_type() == "FENCE_STATUS":
            fence_status.append({
                "wall_s": None,
                "breach_status": int(getattr(msg, "breach_status", 0)),
                "breach_type": int(getattr(msg, "breach_type", 0)),
                "breach_count": int(getattr(msg, "breach_count", 0)),
            })

    set_mode(master, "GUIDED")
    descent_start = time.time()
    next_hb = 0.0
    next_vel = 0.0
    action_wall_s = None
    first_breach_wall_s = None
    min_realtime_alt = None
    ground_contact_wall_s = None
    post_action_start = None

    while time.time() - descent_start < float(exp["max_descent_observation_s"]):
        now = time.time()
        elapsed = now - descent_start
        if now >= next_hb:
            send_gcs_heartbeat(master)
            next_hb = now + dt
        if now >= next_vel and action_wall_s is None:
            send_guided_velocity_local_ned(master, 0.0, 0.0, float(descent_rate_m_s))
            next_vel = now + dt
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT", "FENCE_STATUS", "COMMAND_ACK"],
            blocking=True,
            timeout=0.08,
        )
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "GLOBAL_POSITION_INT":
            rel_alt = float(msg.relative_alt) / 1000.0
            min_realtime_alt = rel_alt if min_realtime_alt is None else min(min_realtime_alt, rel_alt)
            realtime_alt_samples.append({"wall_s": elapsed, "relative_alt_m": rel_alt})
            if rel_alt <= float(exp["ground_contact_alt_m"]) and ground_contact_wall_s is None:
                ground_contact_wall_s = elapsed
        elif mtype == "HEARTBEAT":
            mode = mode_name(master, msg)
            armed = _is_armed(msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": elapsed, "mode": mode})
            if mode in {"RTL", "LAND", "BRAKE", "SMART_RTL"} and action_wall_s is None:
                action_wall_s = elapsed
                post_action_start = now
        elif mtype == "STATUSTEXT":
            text = str(getattr(msg, "text", ""))
            statustext.append({"wall_s": elapsed, "text": text})
            if "fence" in text.lower() and first_breach_wall_s is None:
                first_breach_wall_s = elapsed
            if "crash" in text.lower() and ground_contact_wall_s is None:
                ground_contact_wall_s = elapsed
        elif mtype == "FENCE_STATUS":
            rec = {
                "wall_s": elapsed,
                "breach_status": int(getattr(msg, "breach_status", 0)),
                "breach_type": int(getattr(msg, "breach_type", 0)),
                "breach_count": int(getattr(msg, "breach_count", 0)),
            }
            fence_status.append(rec)
            if rec["breach_status"] and first_breach_wall_s is None:
                first_breach_wall_s = elapsed

        if post_action_start is not None and now - post_action_start >= float(exp["post_failsafe_observation_s"]):
            break
        if ground_contact_wall_s is not None and action_wall_s is not None and now - post_action_start >= 1.0:
            break
        if not armed and action_wall_s is not None:
            break

    cleanup_error = None
    if float(exp.get("skip_cleanup_land", 0.0)) >= 0.5:
        cleanup_error = "skipped_cleanup_land_after_oracle_window"
    else:
        try:
            land_and_disarm(master, timeout_s=float(exp.get("cleanup_land_timeout_s", 28.0)))
        except Exception as exc:
            cleanup_error = repr(exc)

    return {
        "motion": "guided_local_ned_down_velocity_until_min_alt_fence_recovery",
        "commanded_descent_rate_m_s": float(descent_rate_m_s),
        "fence_alt_min_m": float(fence_alt_min_m),
        "fence_enable_ack": fence_ack,
        "modes_seen": modes,
        "statustext": statustext[-50:],
        "fence_status": fence_status[-50:],
        "realtime_alt_samples_tail": realtime_alt_samples[-80:],
        "min_realtime_alt_m": min_realtime_alt,
        "first_breach_wall_s": first_breach_wall_s,
        "action_wall_s": action_wall_s,
        "ground_contact_wall_s": ground_contact_wall_s,
        "cleanup_error": cleanup_error,
    }


def parse_minalt_dataflash(
    *,
    bin_path: Path,
    csv_path: Path,
    home: dict[str, Any],
    params: dict[str, Any],
    active_params: dict[str, Any],
    run_kind: str,
    fence_alt_min_m: float,
    descent_rate_m_s: float,
    model_name: str,
    model_mass_kg: float,
    model_mass_multiplier: float,
    h_floor_m: float,
    descent_rate_tolerance_m_s: float,
    fence_trigger_tolerance_m: float,
    ground_contact_alt_m: float,
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    pos_rows: list[dict[str, Any]] = []
    xkf_rows: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    event_records: list[dict[str, Any]] = []
    err_records: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    fence_msgs: list[dict[str, Any]] = []
    parm_rows: list[dict[str, Any]] = []
    start_time: float | None = None
    home_alt = float(home["alt_m"])

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
                vd = float(_field(data, "VD") or 0.0)
                xkf_rows.append({"time_s": rel_t, "vd_m_s": vd, "raw": data})

        if mtype == "POS":
            abs_alt = _field(data, "Alt")
            rel_alt = _field(data, "RelHomeAlt", "RAlt")
            if rel_alt is None and abs_alt is not None:
                rel_alt = float(abs_alt) - home_alt
            if rel_alt is None:
                continue
            pos_rows.append({
                "time_s": rel_t,
                "source": "POS",
                "abs_alt_m": float(abs_alt) if abs_alt is not None else "",
                "rel_alt_m": float(rel_alt),
                "mode": "",
            })

    rows = sorted(pos_rows, key=lambda r: float(r["time_s"]))
    _annotate_modes(rows, modes)

    # Window starts at the commanded descent, not during takeoff.  It is detected by
    # positive NED down velocity after the vehicle has reached the takeoff band.
    takeoff_band = max(float(fence_alt_min_m) + 1.0, 0.75 * max((float(r["rel_alt_m"]) for r in rows), default=0.0))
    descent_start = None
    for vel in xkf_rows:
        if float(vel["vd_m_s"]) < 0.5:
            continue
        pos = _nearest(rows, float(vel["time_s"]))
        if pos is not None and float(pos["rel_alt_m"]) >= takeoff_band:
            descent_start = float(vel["time_s"])
            break

    fence_errors = [
        e for e in err_records
        if int(e["subsys"]) == FENCE_ERR_SUBSYS and int(e["ecode"]) != 0
    ]
    min_alt_fence_errors = [e for e in fence_errors if int(e["ecode"]) & FENCE_ALT_MIN_BIT]
    other_fence_errors = [e for e in fence_errors if int(e["ecode"]) & ~FENCE_ALT_MIN_BIT]
    fence_breach_time = float(min_alt_fence_errors[0]["time_s"]) if min_alt_fence_errors else None
    action_modes = [
        m for m in modes
        if m["mode"] == "RTL" and int(m.get("reason") or -1) == FENCE_BREACHED_REASON
    ]
    action_mode = action_modes[0] if action_modes else None
    action_time = None if action_mode is None else float(action_mode["time_s"])

    cleanup_land_modes = [
        m for m in modes
        if m["mode"] == "LAND"
        and int(m.get("reason") or -1) == GCS_COMMAND_REASON
        and descent_start is not None
        and float(m["time_s"]) > float(descent_start)
    ]
    window_end = float(cleanup_land_modes[0]["time_s"]) if cleanup_land_modes else None
    if window_end is None:
        window_end = max((float(r["time_s"]) for r in rows), default=None)

    window_rows = [
        r for r in rows
        if descent_start is not None
        and float(r["time_s"]) >= float(descent_start)
        and (window_end is None or float(r["time_s"]) < float(window_end))
    ]
    min_row = min(window_rows, key=lambda r: float(r["rel_alt_m"])) if window_rows else None
    min_agl = None if min_row is None else float(min_row["rel_alt_m"])

    trigger_row = _nearest(rows, fence_breach_time)
    trigger_alt = None if trigger_row is None else float(trigger_row["rel_alt_m"])
    if descent_start is not None and fence_breach_time is not None:
        audit_vel_rows = [
            v for v in xkf_rows
            if float(v["time_s"]) >= float(descent_start)
            and float(v["time_s"]) <= float(fence_breach_time)
            and float(v["vd_m_s"]) > 0.2
        ]
    else:
        audit_vel_rows = []
    audit_values = [float(v["vd_m_s"]) for v in audit_vel_rows]
    median_vd = statistics.median(audit_values) if audit_values else None
    p10_vd = float(np.percentile(np.array(audit_values, dtype=float), 10)) if audit_values else None
    p90_vd = float(np.percentile(np.array(audit_values, dtype=float), 90)) if audit_values else None
    descent_audit = {
        "source": "XKF1 primary core VD in NED frame, between commanded descent start and FENCE_ALT_MIN breach",
        "samples": len(audit_values),
        "commanded_descent_rate_m_s": float(descent_rate_m_s),
        "median_actual_down_m_s": None if median_vd is None else float(median_vd),
        "p10_actual_down_m_s": p10_vd,
        "p90_actual_down_m_s": p90_vd,
        "median_error_m_s": None if median_vd is None else float(median_vd) - float(descent_rate_m_s),
        "within_tolerance": bool(
            median_vd is not None
            and abs(float(median_vd) - float(descent_rate_m_s)) <= float(descent_rate_tolerance_m_s)
        ),
    }

    crash_errors = [
        e for e in err_records
        if int(e["subsys"]) == CRASH_ERR_SUBSYS and int(e["ecode"]) != 0
    ]
    crash_messages = [m for m in messages if "crash" in str(m.get("text", "")).lower()]
    ground_contact = bool(
        (min_agl is not None and min_agl <= float(ground_contact_alt_m))
        or crash_errors
        or crash_messages
    )
    unsafe_binary = bool(min_agl is not None and min_agl < float(h_floor_m))
    safe_binary = bool(min_agl is not None and min_agl >= float(h_floor_m))
    severity_depth = None if min_agl is None else max(0.0, float(h_floor_m) - float(min_agl))

    other_errors = [
        e for e in err_records
        if int(e["ecode"]) != 0 and int(e["subsys"]) not in {FENCE_ERR_SUBSYS, CRASH_ERR_SUBSYS}
    ]
    bad_events = [e for e in event_records if e.get("name") in BAD_EVENT_NAMES]
    dirty_messages = []
    for msg in messages:
        text = str(msg.get("text", ""))
        low = text.lower()
        if "fence breached" in low or "fence breach" in low:
            continue
        if (
            "prearm: vehicle outside fence" in low
            and fence_breach_time is not None
            and float(msg.get("time_s", 0.0)) > float(fence_breach_time)
        ):
            continue
        if "crash" in low:
            continue
        if any(marker in low for marker in DIRTY_TEXT_MARKERS):
            dirty_messages.append(msg)

    violations: list[str] = []
    if descent_start is None:
        violations.append("commanded_descent_not_observed")
    if not descent_audit.get("within_tolerance"):
        violations.append("commanded_descent_rate_not_reached")
    if not min_alt_fence_errors:
        violations.append("min_alt_fence_breach_missing")
    if not action_mode:
        violations.append("min_alt_fence_RTL_action_missing")
    if trigger_alt is not None and abs(float(trigger_alt) - float(fence_alt_min_m)) > float(fence_trigger_tolerance_m):
        violations.append("min_alt_fence_trigger_altitude_mismatch")
    if other_fence_errors:
        violations.append("unexpected_fence_breach_type")
    if other_errors:
        violations.append("other_ERR_subsystems")
    if bad_events:
        violations.append("bad_EV_events")
    if dirty_messages:
        violations.append("dirty_STATUSTEXT")

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["time_s", "source", "abs_alt_m", "rel_alt_m", "mode"],
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)

    result = {
        "bin_path": str(bin_path),
        "csv_path": str(csv_path),
        "position_source": "POS" if rows else None,
        "samples": len(rows),
        "modes": modes,
        "event_records": event_records,
        "err_records": err_records,
        "messages_count": len(messages),
        "fence_messages": fence_msgs[:40],
        "dirty_messages": dirty_messages[:20],
        "bad_events": bad_events,
        "fence_errors": fence_errors,
        "min_alt_fence_errors": min_alt_fence_errors,
        "other_fence_errors": other_fence_errors,
        "crash_errors_as_outcome": crash_errors,
        "crash_messages_as_outcome": crash_messages[:20],
        "other_errors": other_errors,
        "parm_rows_count": len(parm_rows),
        "descent_start_time_s": descent_start,
        "fence_breach_time_s": fence_breach_time,
        "fence_action_time_s": action_time,
        "fence_action_mode": action_mode,
        "cleanup_window_end_s": window_end,
        "fence_trigger_alt_m": trigger_alt,
        "fence_trigger_alt_error_m": None if trigger_alt is None else float(trigger_alt) - float(fence_alt_min_m),
        "min_agl_m": min_agl,
        "min_agl_sample": min_row,
        "h_floor_m": float(h_floor_m),
        "ground_contact": ground_contact,
        "unsafe_binary": unsafe_binary,
        "safe_binary": safe_binary,
        "severity_depth_m": severity_depth,
        "descent_rate_audit": descent_audit,
        "contract_clean": not violations,
        "contract_violations": violations,
        "crash_or_ground_contact_is_violation": False,
        "run_kind": run_kind,
        "fence_alt_min_m": float(fence_alt_min_m),
        "descent_rate_m_s": float(descent_rate_m_s),
        "model_name": model_name,
        "model_mass_kg": float(model_mass_kg),
        "model_mass_multiplier": float(model_mass_multiplier),
        "params_requested": params,
        "active_params_requested": active_params,
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
    all_records = list(run.get("param_readbacks", [])) + list(run.get("active_param_readbacks", []))
    if not _clean_param_records(all_records):
        run.setdefault("contract_violations", []).append("parameter_readback_failed")
        run["contract_clean"] = False
    clean = bool(run.get("contract_clean"))
    unsafe = bool(run.get("unsafe_binary"))
    safe = bool(run.get("safe_binary"))
    run["unsafe"] = unsafe
    run["safe"] = safe
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
    run_id = "minalt_connectivity_probe"
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
    layer: str,
    descent_rate_m_s: float,
    model_name: str,
    rep_index: int,
    roles: list[str],
) -> dict[str, Any]:
    cfg = config_with_model(config, model_name)
    model = model_spec(config, model_name)
    fence_alt_min = layer_alt_min(config, effective_alt_layer(config, layer))
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "run_kind": run_kind,
        "layer": layer,
        "point_key": point_key(layer, descent_rate_m_s, model_name),
        "rep_index": int(rep_index),
        "roles": roles,
        "descent_rate_m_s": float(descent_rate_m_s),
        "fence_alt_min_m": float(fence_alt_min),
        "model_name": model_name,
        "model_mass_kg": float(model["mass_kg"]),
        "model_mass_multiplier": float(model["mass_multiplier"]),
        "model_json": str(model_path(config, model_name)),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        params = controlled_params(config)
        pm = ParamManager(master)
        pm.apply(params)
        active_pm = ParamManager(master)
        result["flight"] = run_minalt_flight(
            master,
            cfg,
            fence_alt_min_m=fence_alt_min,
            descent_rate_m_s=float(descent_rate_m_s),
            active_pm=active_pm,
        )
        active_params = active_fence_params(config, fence_alt_min)
        snapshot_names = sorted(set(params) | set(active_params))
        snapshot = active_pm.snapshot(snapshot_names)
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        payload = {
            "records": pm.records,
            "active_records": active_pm.records,
            "snapshot": snapshot,
            "model": {
                "name": model_name,
                "mass_kg": float(model["mass_kg"]),
                "mass_multiplier": float(model["mass_multiplier"]),
                "json": str(model_path(config, model_name)),
            },
        }
        param_path.parent.mkdir(parents=True, exist_ok=True)
        param_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        result["params_requested"] = params
        result["active_params_requested"] = active_params
        result["param_snapshot"] = snapshot
        result["param_records_path"] = str(param_path)
        result["param_readbacks"] = pm.records
        result["active_param_readbacks"] = active_pm.records
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
        parsed = parse_minalt_dataflash(
            bin_path=bin_path,
            csv_path=csv_path,
            home=cfg["experiment"]["home"],
            params=params,
            active_params=active_params,
            run_kind=run_kind,
            fence_alt_min_m=fence_alt_min,
            descent_rate_m_s=float(descent_rate_m_s),
            model_name=model_name,
            model_mass_kg=float(model["mass_kg"]),
            model_mass_multiplier=float(model["mass_multiplier"]),
            h_floor_m=float(config["experiment"]["h_floor_m"]),
            descent_rate_tolerance_m_s=float(config["experiment"]["descent_rate_tolerance_m_s"]),
            fence_trigger_tolerance_m=float(config["experiment"]["fence_trigger_tolerance_m"]),
            ground_contact_alt_m=float(config["experiment"]["ground_contact_alt_m"]),
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
    layer: str,
    descent_rate_m_s: float,
    model_name: str,
    rep_index: int,
    roles: list[str],
    max_new_runs_state: dict[str, int | None],
) -> bool:
    for run in runs:
        if run.get("run_id") == run_id:
            update_roles(run, roles)
            classify_run(run)
            write_json(partial_path, {"runs": runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})
            return True
    remaining = max_new_runs_state.get("remaining")
    if remaining is not None and int(remaining) <= 0:
        return False
    if remaining is not None:
        max_new_runs_state["remaining"] = int(remaining) - 1
    print(
        f"RUN {run_id} kind={run_kind} layer={layer} H={layer_alt_min(config, effective_alt_layer(config, layer))} "
        f"down={descent_rate_m_s} model={model_name} roles={','.join(roles)}",
        flush=True,
    )
    run = run_one(
        config,
        run_id=run_id,
        run_kind=run_kind,
        layer=layer,
        descent_rate_m_s=descent_rate_m_s,
        model_name=model_name,
        rep_index=rep_index,
        roles=roles,
    )
    runs.append(run)
    write_json(partial_path, {"runs": runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})
    return True


def pooled_sigma(points: list[dict[str, Any]]) -> float:
    ss = 0.0
    df = 0
    for point in points:
        values = [float(v) for v in point.get("min_agl_values_m", [])]
        if len(values) < 2:
            continue
        mean = statistics.fmean(values)
        ss += sum((v - mean) ** 2 for v in values)
        df += len(values) - 1
    if df <= 0:
        return 0.0
    return math.sqrt(ss / df)


def completed_runs_for_point(runs: list[dict[str, Any]], layer: str, descent_rate_m_s: float, model_name: str) -> list[dict[str, Any]]:
    key = point_key(layer, descent_rate_m_s, model_name)
    return [r for r in runs if r.get("point_key") == key and not r.get("error")]


def summarize_noise_runs(noise_runs: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = {"boundary": [], "unsafe": []}
    default_layer = str(config["sweep"]["default_layer"])
    for group_name, cfg_name in (("boundary", "boundary_points"), ("unsafe", "unsafe_points")):
        for spec in config["noise_floor"][cfg_name]:
            rate = float(spec["descent_rate_m_s"])
            model_name = str(spec["model"])
            runs = completed_runs_for_point(noise_runs, f"noise_{group_name}", rate, model_name)
            values = [float(r["min_agl_m"]) for r in runs if r.get("min_agl_m") is not None]
            point = {
                "point_key": point_key(default_layer, rate, model_name),
                "noise_point_key": point_key(f"noise_{group_name}", rate, model_name),
                "group": group_name,
                "descent_rate_m_s": rate,
                "model_name": model_name,
                "model_mass_kg": float(model_spec(config, model_name)["mass_kg"]),
                "model_mass_multiplier": float(model_spec(config, model_name)["mass_multiplier"]),
                "run_ids": [r.get("run_id") for r in runs],
                "completed_repetitions": len(runs),
                "min_agl_values_m": values,
                "mean_min_agl_m": statistics.fmean(values) if values else None,
                "sample_std_min_agl_m": statistics.stdev(values) if len(values) >= 2 else 0.0 if values else None,
                "contract_clean_all": bool(runs) and all(bool(r.get("contract_clean")) for r in runs),
                "contract_violations": sorted({v for r in runs for v in r.get("contract_violations", [])}),
            }
            groups[group_name].append(point)
    sigma_boundary = pooled_sigma(groups["boundary"])
    sigma_unsafe = pooled_sigma(groups["unsafe"])
    k = float(config["noise_floor"]["k_sigma_margin"])
    c = float(config["noise_floor"]["c_sigma_mae_bound"])
    ci_mult = float(config["noise_floor"]["ci_sigma_multiplier"])
    d_margin = k * sigma_boundary
    h_floor = float(config["experiment"]["h_floor_m"])
    return {
        "measured_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": "SITL repeated min-AGL measurements at preregistered boundary and unsafe points",
        "h_floor_m": h_floor,
        "boundary": {"points": groups["boundary"], "sigma_m": sigma_boundary},
        "unsafe": {"points": groups["unsafe"], "sigma_m": sigma_unsafe},
        "sigma_boundary_m": sigma_boundary,
        "sigma_unsafe_m": sigma_unsafe,
        "k_sigma_margin": k,
        "d_margin_m": d_margin,
        "ci_sigma_multiplier": ci_mult,
        "ambiguous_band_min_agl_m": [h_floor - ci_mult * sigma_boundary, h_floor + ci_mult * sigma_boundary],
        "ambiguous_band_width_m": 2.0 * ci_mult * sigma_boundary,
        "c_sigma_mae_bound": c,
        "mae_bound_m": c * sigma_boundary,
        "label_rule": "unsafe iff mean min-AGL + 2*sigma_boundary < h_floor; safe iff mean min-AGL - 2*sigma_boundary > h_floor",
    }


def aggregate_point(
    runs: list[dict[str, Any]],
    *,
    layer: str,
    descent_rate_m_s: float,
    model_name: str,
    required_repetitions: int,
    oracle: dict[str, Any],
) -> dict[str, Any]:
    complete = completed_runs_for_point(runs, layer, descent_rate_m_s, model_name)
    values = [float(r["min_agl_m"]) for r in complete if r.get("min_agl_m") is not None]
    depths = [float(r["severity_depth_m"]) for r in complete if r.get("severity_depth_m") is not None]
    actual_rates = [
        float(r["descent_rate_audit"]["median_actual_down_m_s"])
        for r in complete
        if r.get("descent_rate_audit", {}).get("median_actual_down_m_s") is not None
    ]
    mean = statistics.fmean(values) if values else None
    sigma = float(oracle["sigma_boundary_m"])
    ci_mult = float(oracle["ci_sigma_multiplier"])
    h_floor = float(oracle["h_floor_m"])
    ci_low = None if mean is None else mean - ci_mult * sigma
    ci_high = None if mean is None else mean + ci_mult * sigma
    contract_violations = sorted({v for r in complete for v in r.get("contract_violations", [])})
    contract_clean_all = bool(complete) and len(complete) >= required_repetitions and all(bool(r.get("contract_clean")) for r in complete)
    errors = [
        r.get("error")
        for r in runs
        if r.get("point_key") == point_key(layer, descent_rate_m_s, model_name) and r.get("error")
    ]
    if len(complete) < required_repetitions or mean is None:
        label = "blocked"
        reason = "incomplete_repetitions"
    elif not contract_clean_all:
        label = "contract_violated"
        reason = "contract_violated"
    elif ci_high is not None and ci_high < h_floor:
        label = "clean_unsafe"
        reason = "mean_plus_2sigma_below_h_floor"
    elif ci_low is not None and ci_low > h_floor:
        label = "clean_safe"
        reason = "mean_minus_2sigma_above_h_floor"
    else:
        label = "ambiguous"
        reason = "label_CI_crosses_h_floor"
    model = model_spec(config_global, model_name)
    return {
        "point_key": point_key(layer, descent_rate_m_s, model_name),
        "layer": layer,
        "fence_alt_min_m": layer_alt_min(config_global, layer),
        "descent_rate_m_s": float(descent_rate_m_s),
        "model_name": model_name,
        "model_mass_kg": float(model["mass_kg"]),
        "model_mass_multiplier": float(model["mass_multiplier"]),
        "required_repetitions": int(required_repetitions),
        "repetitions": len([r for r in runs if r.get("point_key") == point_key(layer, descent_rate_m_s, model_name)]),
        "completed_repetitions": len(complete),
        "run_ids": [r.get("run_id") for r in complete],
        "label": label,
        "label_reason": reason,
        "stable_binary": label in CONFIDENT_LABELS,
        "mean_min_agl_m": mean,
        "min_agl_m": mean,
        "sample_std_min_agl_m": statistics.stdev(values) if len(values) >= 2 else 0.0 if values else None,
        "min_agl_values_m": values,
        "mean_severity_depth_m": statistics.fmean(depths) if depths else None,
        "severity_depth_m": statistics.fmean(depths) if depths else None,
        "label_ci_low_m": ci_low,
        "label_ci_high_m": ci_high,
        "h_floor_m": h_floor,
        "actual_descent_rate_m_s": statistics.fmean(actual_rates) if actual_rates else None,
        "contract_clean_all": contract_clean_all,
        "contract_violations": contract_violations,
        "ground_contact_any": any(bool(r.get("ground_contact")) for r in complete),
        "crash_or_ground_contact_is_violation": False,
        "errors": errors,
    }


# The aggregation helpers are intentionally close to the linkloss v2 script; this
# module-level handle keeps signatures small without changing completed results.
config_global: dict[str, Any] = {}


def aggregate_layer(
    runs: list[dict[str, Any]],
    *,
    config: dict[str, Any],
    layer: str,
    required_repetitions: int,
    oracle: dict[str, Any],
) -> list[dict[str, Any]]:
    global config_global
    config_global = config
    points = []
    for rate in config["sweep"]["descent_rates_m_s"]:
        for model_name in config["sweep"]["model_order"]:
            points.append(
                aggregate_point(
                    runs,
                    layer=layer,
                    descent_rate_m_s=float(rate),
                    model_name=str(model_name),
                    required_repetitions=required_repetitions,
                    oracle=oracle,
                )
            )
    return points


def zone_counts(points: list[dict[str, Any]]) -> dict[str, int]:
    counts = {"clean_safe": 0, "clean_unsafe": 0, "ambiguous": 0, "contract_violated": 0, "blocked": 0}
    for point in points:
        label = str(point.get("label", "blocked"))
        counts[label] = counts.get(label, 0) + 1
    return counts


def premise_summary(runs: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    by_id = {str(r.get("run_id")): r for r in runs}
    mech_cfg = config["premise"]["mechanism"]
    rate_cfg = config["premise"]["descent_rate_application"]
    mass_cfg = config["premise"]["mass_response"]
    mechanism = by_id.get(premise_run_id("mechanism", str(mech_cfg["model"])))
    rate_run = by_id.get(premise_run_id("descent_rate", str(rate_cfg["model"])))
    mass_runs = [by_id.get(premise_run_id("mass_response", str(m))) for m in mass_cfg["models"]]
    checks = []
    mechanism_ok = bool(
        mechanism
        and not mechanism.get("error")
        and mechanism.get("min_alt_fence_errors")
        and mechanism.get("fence_action_mode")
        and (mechanism.get("unsafe_binary") or mechanism.get("ground_contact"))
    )
    checks.append({
        "name": "mechanism_fence_RTL_then_danger_floor",
        "ok": mechanism_ok,
        "run_id": None if mechanism is None else mechanism.get("run_id"),
        "min_agl_m": None if mechanism is None else mechanism.get("min_agl_m"),
        "fence_action_mode": None if mechanism is None else mechanism.get("fence_action_mode"),
        "contract_violations": [] if mechanism is None else mechanism.get("contract_violations", []),
    })
    rate_ok = bool(
        rate_run
        and not rate_run.get("error")
        and rate_run.get("descent_rate_audit", {}).get("within_tolerance")
    )
    checks.append({
        "name": "commanded_descent_rate_applied",
        "ok": rate_ok,
        "run_id": None if rate_run is None else rate_run.get("run_id"),
        "audit": None if rate_run is None else rate_run.get("descent_rate_audit"),
    })
    mass_complete = [r for r in mass_runs if r and not r.get("error") and r.get("min_agl_m") is not None]
    mass_values = [
        {
            "model_name": r["model_name"],
            "mass_multiplier": float(r["model_mass_multiplier"]),
            "mass_kg": float(r["model_mass_kg"]),
            "min_agl_m": float(r["min_agl_m"]),
            "run_id": r["run_id"],
        }
        for r in mass_complete
    ]
    mass_values.sort(key=lambda x: x["mass_multiplier"])
    monotone = bool(
        len(mass_values) == len(mass_cfg["models"])
        and all(mass_values[i]["min_agl_m"] <= mass_values[i - 1]["min_agl_m"] + 0.25 for i in range(1, len(mass_values)))
        and (mass_values[0]["min_agl_m"] - mass_values[-1]["min_agl_m"] >= 0.2)
    )
    checks.append({
        "name": "mass_increase_lowers_min_agl",
        "ok": monotone,
        "values": mass_values,
    })
    satisfied = mechanism_ok and rate_ok and monotone
    return {
        "satisfied": satisfied,
        "checks": checks,
        "runs": runs,
        "reason": "height fence recovery, descent-rate application, and mass response all held"
        if satisfied
        else "one or more premise gates failed; verdict is not meaningful as a PASS/FAIL",
        "fallback_trigger_used": False,
    }


def split_points(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    cfg = config["train_test"]
    interp_train_v = {float(v) for v in cfg["interpolation_train_descent_rates_m_s"]}
    interp_test_v = {float(v) for v in cfg["interpolation_test_descent_rates_m_s"]}
    interp_train_m = {str(m) for m in cfg["interpolation_train_models"]}
    interp_test_m = {str(m) for m in cfg["interpolation_test_models"]}
    extra_train_m = {str(m) for m in cfg["extrapolation_train_models"]}
    extra_test_m = {str(m) for m in cfg["extrapolation_test_models"]}
    return {
        "interpolation_train": [
            p for p in points
            if float(p["descent_rate_m_s"]) in interp_train_v and str(p["model_name"]) in interp_train_m
        ],
        "interpolation_test": [
            p for p in points
            if float(p["descent_rate_m_s"]) in interp_test_v and str(p["model_name"]) in interp_test_m
        ],
        "extrapolation_train": [
            p for p in points
            if float(p["descent_rate_m_s"]) <= float(cfg["extrapolation_train_max_descent_rate_m_s"])
            and str(p["model_name"]) in extra_train_m
        ],
        "extrapolation_test": [
            p for p in points
            if float(p["descent_rate_m_s"]) >= float(cfg["extrapolation_test_min_descent_rate_m_s"])
            and str(p["model_name"]) in extra_test_m
        ],
    }


def severity_feature_row(point: dict[str, Any]) -> list[float]:
    v = float(point["descent_rate_m_s"])
    m = float(point["model_mass_multiplier"])
    vm = v * m
    return [1.0, v, m, vm, vm * vm, v * v, m * m]


def fit_min_agl_regression(points: list[dict[str, Any]], ridge: float = 1.0e-6) -> dict[str, Any]:
    usable = [p for p in points if p.get("min_agl_m") is not None and p.get("label") != "blocked"]
    if len(usable) < 7:
        return {"ok": False, "reason": "not enough training points", "train_points": len(usable)}
    x = np.array([severity_feature_row(p) for p in usable], dtype=float)
    y = np.array([float(p["min_agl_m"]) for p in usable], dtype=float)
    penalty = ridge * np.eye(x.shape[1], dtype=float)
    penalty[0, 0] = 0.0
    beta = np.linalg.pinv(x.T @ x + penalty) @ (x.T @ y)
    pred = np.maximum(0.0, x @ beta)
    train_mae = float(np.mean(np.abs(pred - y)))
    return {
        "ok": True,
        "feature_names": ["intercept", "v", "mass", "v*mass", "(v*mass)^2", "v^2", "mass^2"],
        "coefficients": [float(v) for v in beta],
        "ridge": ridge,
        "train_points": len(usable),
        "train_mae_m": train_mae,
    }


def predict_min_agl(model: dict[str, Any], point: dict[str, Any]) -> float:
    beta = np.array(model["coefficients"], dtype=float)
    pred = float(np.array(severity_feature_row(point), dtype=float) @ beta)
    return max(0.0, pred)


def evaluate_classification_split(train: list[dict[str, Any]], test: list[dict[str, Any]], oracle: dict[str, Any]) -> dict[str, Any]:
    model = fit_min_agl_regression(train)
    rows = []
    hits = 0
    confident_test = [p for p in test if p.get("label") in CONFIDENT_LABELS]
    excluded = [p for p in test if p.get("label") == "ambiguous"]
    if model.get("ok"):
        for point in confident_test:
            predicted_min_agl = predict_min_agl(model, point)
            scale = max(float(oracle["sigma_boundary_m"]), 1.0e-6)
            prob = 1.0 / (1.0 + math.exp(-max(-40.0, min(40.0, (float(oracle["h_floor_m"]) - predicted_min_agl) / scale))))
            pred = prob >= 0.5
            obs = point["label"] == "clean_unsafe"
            hits += int(pred == obs)
            rows.append({
                "point_key": point["point_key"],
                "descent_rate_m_s": float(point["descent_rate_m_s"]),
                "model_name": point["model_name"],
                "model_mass_multiplier": float(point["model_mass_multiplier"]),
                "observed_label": point["label"],
                "observed_unsafe": obs,
                "predicted_min_agl_m": predicted_min_agl,
                "probability_unsafe": prob,
                "predicted_unsafe": pred,
                "correct": pred == obs,
            })
    return {
        "model": model,
        "predictions": rows,
        "excluded_ambiguous": [
            {
                "point_key": p["point_key"],
                "descent_rate_m_s": p["descent_rate_m_s"],
                "model_name": p["model_name"],
                "mean_min_agl_m": p.get("mean_min_agl_m"),
                "label_ci_low_m": p.get("label_ci_low_m"),
                "label_ci_high_m": p.get("label_ci_high_m"),
                "h_floor_m": p.get("h_floor_m"),
                "exclusion_basis": "label CI crosses h_floor",
            }
            for p in excluded
        ],
        "metrics": {
            "count": len(confident_test),
            "ambiguous_excluded_count": len(excluded),
            "classification_accuracy": (hits / len(confident_test)) if confident_test else None,
        },
    }


def train_test_classification(points: list[dict[str, Any]], config: dict[str, Any], oracle: dict[str, Any]) -> dict[str, Any]:
    splits = split_points(points, config)
    interpolation = evaluate_classification_split(splits["interpolation_train"], splits["interpolation_test"], oracle)
    extrapolation = evaluate_classification_split(splits["extrapolation_train"], splits["extrapolation_test"], oracle)
    combined_rows = interpolation["predictions"] + extrapolation["predictions"]
    combined_acc = None
    if combined_rows:
        combined_acc = sum(1 for r in combined_rows if r["correct"]) / len(combined_rows)
    return {
        "formula": "unsafe probability = sigmoid((h_floor - min_agl_model(v, mass)) / sigma_boundary)",
        "interpolation": interpolation,
        "extrapolation": extrapolation,
        "combined_heldout_accuracy": combined_acc,
        "holdout_definition": config["train_test"],
        "ambiguous_exclusion_basis": "Only label uncertainty is excluded from classification; prediction correctness is not consulted.",
    }


def evaluate_severity_regression(points: list[dict[str, Any]], config: dict[str, Any], oracle: dict[str, Any]) -> dict[str, Any]:
    splits = split_points(points, config)
    holdout_by_key: dict[str, dict[str, Any]] = {}
    for point in splits["interpolation_test"] + splits["extrapolation_test"]:
        holdout_by_key[str(point["point_key"])] = point
    holdout_keys = set(holdout_by_key)
    train = [p for p in points if str(p["point_key"]) not in holdout_keys and p.get("label") != "blocked"]
    holdout = [holdout_by_key[k] for k in sorted(holdout_by_key)]
    model = fit_min_agl_regression(train)
    rows = []
    if model.get("ok"):
        for point in holdout:
            pred = predict_min_agl(model, point)
            obs = float(point.get("min_agl_m") or 0.0)
            rows.append({
                "point_key": point["point_key"],
                "descent_rate_m_s": float(point["descent_rate_m_s"]),
                "model_name": point["model_name"],
                "model_mass_multiplier": float(point["model_mass_multiplier"]),
                "observed_min_agl_m": obs,
                "predicted_min_agl_m": pred,
                "abs_error_m": abs(pred - obs),
                "label": point.get("label"),
                "included_boundary_or_ambiguous": point.get("label") == "ambiguous",
            })
    mae = float(np.mean([r["abs_error_m"] for r in rows])) if rows else None
    return {
        "formula": "min_agl_m = max(0, beta0 + beta_v*v + beta_m*m + beta_vm*v*m + beta_vm2*(v*m)^2 + beta_v2*v^2 + beta_m2*m^2)",
        "model": model,
        "train_point_count": len(train),
        "holdout_point_count": len(holdout),
        "holdout_points_include_ambiguous": any(p.get("label") == "ambiguous" for p in holdout),
        "predictions": rows,
        "metrics": {
            "mae_m": mae,
            "mae_bound_m": float(oracle["mae_bound_m"]),
            "pass": bool(mae is not None and mae <= float(oracle["mae_bound_m"])),
        },
    }


def p_stratification_summary(layers: dict[str, dict[str, Any]]) -> dict[str, Any]:
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1]["fence_alt_min_m"]))
    counts = [
        {
            "layer": name,
            "fence_alt_min_m": float(layer["fence_alt_min_m"]),
            "clean_unsafe": int(layer["zone_counts"].get("clean_unsafe", 0)),
            "clean_safe": int(layer["zone_counts"].get("clean_safe", 0)),
            "ambiguous": int(layer["zone_counts"].get("ambiguous", 0)),
            "contract_violated": int(layer["zone_counts"].get("contract_violated", 0)),
        }
        for name, layer in ordered
    ]
    nonincreasing = all(counts[i]["clean_unsafe"] <= counts[i - 1]["clean_unsafe"] for i in range(1, len(counts)))
    return {
        "counts": counts,
        "monotonic_shrink_with_higher_fence_alt_min": nonincreasing,
        "conclusion": "clean_unsafe count shrinks or stays flat as FENCE_ALT_MIN increases"
        if nonincreasing
        else "clean_unsafe count did not shrink monotonically as FENCE_ALT_MIN increased",
    }


def boundary_search_summary(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    rates = [float(v) for v in config["sweep"]["descent_rates_m_s"]]
    models = [str(m) for m in config["search"]["models"]]
    by = {(float(p["descent_rate_m_s"]), str(p["model_name"])): p for p in points}
    records = []
    total_queries = 0
    for model_name in models:
        lo = 0
        hi = len(rates) - 1
        queries = []
        first_unsafe = None
        while lo <= hi:
            mid = (lo + hi) // 2
            rate = rates[mid]
            p = by.get((rate, model_name))
            label = None if p is None else p.get("label")
            queries.append({"descent_rate_m_s": rate, "model_name": model_name, "label": label})
            if label == "clean_unsafe":
                first_unsafe = rate
                hi = mid - 1
            else:
                lo = mid + 1
            if len(queries) >= int(config["search"]["bisection_iterations_per_model"]):
                break
        total_queries += len(queries)
        records.append({"model_name": model_name, "queries": queries, "first_clean_unsafe_descent_rate_m_s": first_unsafe})
    return {
        "strategy": "discrete bisection over descent rate for each mass, replayed against completed noise-aware grid results",
        "query_count": total_queries,
        "full_grid_count": len([p for p in points if p.get("layer") == config["sweep"]["default_layer"]]),
        "records": records,
    }


def verdict_summary(
    premise: dict[str, Any],
    default_points: list[dict[str, Any]],
    classification: dict[str, Any],
    severity: dict[str, Any],
) -> dict[str, Any]:
    if not premise.get("satisfied"):
        return {
            "verdict": "INCONCLUSIVE",
            "premise_satisfied": False,
            "robust_clean_unsafe": False,
            "contract_clean_gap": False,
            "classification_ok": False,
            "severity_ok": False,
            "prediction_ok": False,
            "reason": "Premise failed.",
        }
    clean_unsafe = [p for p in default_points if p.get("label") == "clean_unsafe"]
    contract_violated = [p for p in default_points if p.get("label") == "contract_violated"]
    blocked = [p for p in default_points if p.get("label") == "blocked"]
    robust_clean_unsafe = bool(len(clean_unsafe) >= 2 and all(p.get("stable_binary") for p in clean_unsafe))
    contract_clean_gap = bool(clean_unsafe and not contract_violated and not blocked and all(p.get("contract_clean_all") for p in clean_unsafe))
    interp_acc = classification.get("interpolation", {}).get("metrics", {}).get("classification_accuracy")
    extra_acc = classification.get("extrapolation", {}).get("metrics", {}).get("classification_accuracy")
    combined_acc = classification.get("combined_heldout_accuracy")
    extra_count = int(classification.get("extrapolation", {}).get("metrics", {}).get("count") or 0)
    classification_ok = bool(
        interp_acc is not None
        and extra_acc is not None
        and combined_acc is not None
        and float(interp_acc) >= 0.90
        and float(extra_acc) >= 0.90
        and float(combined_acc) >= 0.90
        and extra_count > 0
    )
    severity_ok = bool(severity.get("metrics", {}).get("pass"))
    prediction_ok = classification_ok and severity_ok
    if robust_clean_unsafe and contract_clean_gap and prediction_ok:
        verdict = "PASS"
        reason = "All decisive criteria are satisfied."
    else:
        verdict = "FAIL"
        missing = []
        if not robust_clean_unsafe:
            missing.append("no robust noise-confident clean_unsafe region")
        if not contract_clean_gap:
            missing.append("contract_violated or blocked point present, or clean_unsafe is not contract-clean")
        if not classification_ok:
            missing.append("noise-aware held-out classification is below 90% or lacks extrapolation")
        if not severity_ok:
            missing.append("severity regression MAE exceeds the preregistered noise-scale bound")
        reason = "; ".join(missing)
    return {
        "verdict": verdict,
        "premise_satisfied": True,
        "robust_clean_unsafe": robust_clean_unsafe,
        "contract_clean_gap": contract_clean_gap,
        "classification_ok": classification_ok,
        "severity_ok": severity_ok,
        "prediction_ok": prediction_ok,
        "clean_unsafe_count": len(clean_unsafe),
        "contract_violated_count": len(contract_violated),
        "blocked_count": len(blocked),
        "ambiguous_count": len([p for p in default_points if p.get("label") == "ambiguous"]),
        "interpolation_accuracy": interp_acc,
        "extrapolation_accuracy": extra_acc,
        "combined_heldout_accuracy": combined_acc,
        "severity_mae_m": severity.get("metrics", {}).get("mae_m"),
        "severity_mae_bound_m": severity.get("metrics", {}).get("mae_bound_m"),
        "reason": reason,
    }


def plot_premise_minalt(premise: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    mass_check = next((c for c in premise.get("checks", []) if c.get("name") == "mass_increase_lowers_min_agl"), {})
    vals = mass_check.get("values", [])
    if vals:
        ax.plot([v["mass_multiplier"] for v in vals], [v["min_agl_m"] for v in vals], marker="o", color="#264653")
    ax.axhline(2.0, color="#d62828", linestyle="--", linewidth=1.2, label="h_floor")
    ax.set_xlabel("Mass multiplier")
    ax.set_ylabel("min-AGL (m)")
    ax.set_title("Premise mass response")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_result_field(points: list[dict[str, Any]], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    for label, color in ZONE_COLORS.items():
        subset = [p for p in points if p.get("label") == label]
        if not subset:
            continue
        ax.scatter(
            [float(p["descent_rate_m_s"]) for p in subset],
            [float(p["model_mass_multiplier"]) for p in subset],
            s=520,
            marker="s",
            color=color,
            edgecolor="black",
            linewidth=0.8,
            label=label,
        )
    abbrev = {"clean_safe": "S", "clean_unsafe": "U", "ambiguous": "A", "contract_violated": "V", "blocked": "B"}
    for p in points:
        ax.text(
            float(p["descent_rate_m_s"]),
            float(p["model_mass_multiplier"]),
            abbrev.get(str(p.get("label")), "?"),
            ha="center",
            va="center",
            color="white" if p.get("label") in {"clean_unsafe", "contract_violated"} else "black",
            fontsize=10,
        )
    ax.set_xlabel("Commanded descent rate (m/s)")
    ax.set_ylabel("Mass multiplier")
    ax.set_title("Noise-aware min-alt result field")
    ax.grid(True, alpha=0.22)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_severity_heatmap(points: list[dict[str, Any]], oracle: dict[str, Any], out_path: Path) -> str:
    pts = [p for p in points if p.get("min_agl_m") is not None]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    if pts:
        xs = np.array([float(p["descent_rate_m_s"]) for p in pts])
        ys = np.array([float(p["model_mass_multiplier"]) for p in pts])
        zs = np.array([float(p.get("min_agl_m") or 0.0) for p in pts])
        if len(pts) >= 4 and len(set(xs)) > 1 and len(set(ys)) > 1 and float(np.nanmax(zs)) > float(np.nanmin(zs)):
            levels = np.linspace(float(np.nanmin(zs)), float(np.nanmax(zs)), 18)
            cf = ax.tricontourf(xs, ys, zs, levels=levels, cmap="viridis")
            fig.colorbar(cf, ax=ax, label="Mean min-AGL (m)")
            low, high = oracle["ambiguous_band_min_agl_m"]
            contour_levels = sorted({float(low), float(oracle["h_floor_m"]), float(high)})
            contour_levels = [v for v in contour_levels if float(np.nanmin(zs)) <= v <= float(np.nanmax(zs))]
            if contour_levels:
                cs = ax.tricontour(xs, ys, zs, levels=contour_levels, colors=["#ffdd00"], linewidths=1.5)
                ax.clabel(cs, inline=True, fontsize=8, fmt="%.2f m")
        else:
            sc = ax.scatter(xs, ys, c=zs, cmap="viridis", s=95, edgecolor="black", linewidth=0.7)
            fig.colorbar(sc, ax=ax, label="Mean min-AGL (m)")
        for p in pts:
            ax.text(float(p["descent_rate_m_s"]), float(p["model_mass_multiplier"]), f"{float(p.get('min_agl_m') or 0.0):.1f}", ha="center", va="center", fontsize=7, color="white")
    ax.set_xlabel("Commanded descent rate (m/s)")
    ax.set_ylabel("Mass multiplier")
    ax.set_title("min-AGL severity with noise-aware floor band")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_p_stratification(layers: dict[str, dict[str, Any]], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1]["fence_alt_min_m"]))
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    xs = [float(layer["fence_alt_min_m"]) for _, layer in ordered]
    ys = [int(layer["zone_counts"].get("clean_unsafe", 0)) for _, layer in ordered]
    ax.plot(xs, ys, marker="o", linewidth=2.0, color="#d62828")
    for x, y, (name, _) in zip(xs, ys, ordered):
        ax.text(x, y + 0.15, name, ha="center", fontsize=9)
    ax.set_xlabel("FENCE_ALT_MIN (m)")
    ax.set_ylabel("clean_unsafe count")
    ax.set_title("P stratification: higher floor triggers earlier recovery")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_train_test(classification: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.0, 6.0))
    styles = {
        "interpolation": {"marker": "o", "color": "#2a9d8f", "label": "Interpolation"},
        "extrapolation": {"marker": "^", "color": "#e76f51", "label": "Extrapolation"},
    }
    for split, style in styles.items():
        rows = classification.get(split, {}).get("predictions", [])
        if not rows:
            continue
        ax.scatter(
            [float(r["probability_unsafe"]) for r in rows],
            [1.0 if r["observed_unsafe"] else 0.0 for r in rows],
            marker=style["marker"],
            color=style["color"],
            edgecolor="black",
            s=90,
            label=style["label"],
        )
    ax.axvline(0.5, color="#444", linestyle="--", linewidth=1.0)
    ax.set_xlabel("Predicted unsafe probability")
    ax.set_ylabel("Observed unsafe label")
    ax.set_yticks([0, 1], ["safe", "unsafe"])
    ax.set_title("Held-out classification")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_severity_regression(severity: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = severity.get("predictions", [])
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    if rows:
        obs = np.array([float(r["observed_min_agl_m"]) for r in rows], dtype=float)
        pred = np.array([float(r["predicted_min_agl_m"]) for r in rows], dtype=float)
        labels = [str(r.get("label")) for r in rows]
        colors = ["#f4a261" if lab == "ambiguous" else "#2a9d8f" if lab == "clean_safe" else "#d62828" for lab in labels]
        ax.scatter(obs, pred, c=colors, edgecolor="black", s=85)
        lo = min(float(np.min(obs)), float(np.min(pred)), 0.0)
        hi = max(float(np.max(obs)), float(np.max(pred)), 2.5)
        ax.plot([lo, hi], [lo, hi], color="#444", linestyle="--", linewidth=1.0)
    ax.set_xlabel("Observed min-AGL (m)")
    ax.set_ylabel("Predicted min-AGL (m)")
    ax.set_title("Severity regression holdout")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    plots = {"premise": plot_premise_minalt(payload["premise"], analysis / "minalt_premise.png")}
    if payload["verdict"]["verdict"] != "INCONCLUSIVE" or payload.get("default_grid", {}).get("points"):
        default_points = payload.get("default_grid", {}).get("points", [])
        if default_points:
            plots.update({
                "result_field": plot_result_field(default_points, analysis / "minalt_result_field.png"),
                "severity": plot_severity_heatmap(default_points, payload["oracle"], analysis / "minalt_severity_heatmap.png"),
                "p_stratification": plot_p_stratification(payload["p_stratification"]["layers"], analysis / "minalt_p_stratification.png"),
                "train_test": plot_train_test(payload["classification"], analysis / "minalt_train_test.png"),
                "severity_regression": plot_severity_regression(payload["severity_regression"], analysis / "minalt_severity_regression.png"),
            })
    return plots


def write_report(payload: dict[str, Any]) -> str:
    report = PLANC_ROOT / "results" / "minalt_groundcontact_report.md"
    report.parent.mkdir(parents=True, exist_ok=True)
    verdict = payload["verdict"]
    oracle = payload.get("oracle", {})
    lines: list[str] = []
    lines.append(f"VERDICT: {verdict['verdict']}")
    lines.append("")
    lines.append("# Minimum Altitude Fence Ground-Contact Scenario")
    lines.append("")
    lines.append("## Decisive Criteria")
    lines.append("")
    lines.append(f"- Premise satisfied: **{verdict.get('premise_satisfied')}**")
    lines.append(f"- Robust clean unsafe region: **{verdict.get('robust_clean_unsafe')}**")
    lines.append(f"- Fence-triggered, contract-clean PGFUZZ-invisible gap: **{verdict.get('contract_clean_gap')}**")
    lines.append(f"- Prediction gates passed: **{verdict.get('prediction_ok')}** (classification={verdict.get('classification_ok')}, severity={verdict.get('severity_ok')})")
    lines.append(f"- Reason: {verdict.get('reason')}")
    lines.append("")
    lines.append("## Premise")
    lines.append("")
    lines.append(f"Premise conclusion: **{payload['premise']['satisfied']}** - {payload['premise']['reason']}.")
    lines.append("")
    lines.append("| check | ok | detail |")
    lines.append("|---|---:|---|")
    for check in payload["premise"].get("checks", []):
        detail = check.get("run_id") or json.dumps(check.get("values", check.get("audit", "")), sort_keys=True)
        lines.append(f"| {check.get('name')} | {check.get('ok')} | `{detail}` |")
    lines.append("")
    lines.append("The fence floor is activated after takeoff because ArduCopter rejects arming below an enabled `FENCE_ALT_MIN`; the active P parameters are read back before the descent stimulus.")
    lines.append("")
    lines.append("## Measurement Precision")
    lines.append("")
    lines.append(
        f"`sigma_boundary={fmt(oracle.get('sigma_boundary_m'), 4)} m`, "
        f"`sigma_unsafe={fmt(oracle.get('sigma_unsafe_m'), 4)} m`, "
        f"`d_margin={fmt(oracle.get('d_margin_m'), 4)} m`, "
        f"`h_floor={fmt(oracle.get('h_floor_m'), 2)} m`, "
        f"`mae_bound={fmt(oracle.get('mae_bound_m'), 4)} m`."
    )
    lines.append("")
    lines.append("Ambiguous points are excluded from classification only when their label CI crosses `h_floor`; they remain in severity-regression holdout accounting.")
    lines.append("")
    lines.append("| point | label | mean min-AGL m | CI low | CI high | basis |")
    lines.append("|---|---:|---:|---:|---:|---|")
    for point in payload.get("default_grid", {}).get("points", []):
        if point.get("label") == "ambiguous":
            lines.append(
                f"| {point['point_key']} | {point['label']} | {fmt(point.get('mean_min_agl_m'), 3)} | "
                f"{fmt(point.get('label_ci_low_m'), 3)} | {fmt(point.get('label_ci_high_m'), 3)} | {point.get('label_reason')} |"
            )
    lines.append("")
    lines.append("## Three-Zone Field")
    lines.append("")
    counts = payload.get("default_grid", {}).get("zone_counts", {})
    lines.append(
        f"Default layer counts: clean_safe={counts.get('clean_safe', 0)}, "
        f"clean_unsafe={counts.get('clean_unsafe', 0)}, ambiguous={counts.get('ambiguous', 0)}, "
        f"contract_violated={counts.get('contract_violated', 0)}, blocked={counts.get('blocked', 0)}."
    )
    lines.append("")
    lines.append("| point | down m/s | mass x | mean min-AGL m | label | contract violations | runs |")
    lines.append("|---|---:|---:|---:|---|---|---|")
    for p in sorted(payload.get("default_grid", {}).get("points", []), key=lambda r: (float(r["descent_rate_m_s"]), float(r["model_mass_multiplier"]))):
        lines.append(
            f"| {p['point_key']} | {fmt(p['descent_rate_m_s'], 1)} | {fmt(p['model_mass_multiplier'], 2)} | "
            f"{fmt(p.get('mean_min_agl_m'), 3)} | {p.get('label')} | "
            f"{', '.join(p.get('contract_violations', [])) or 'none'} | {p.get('completed_repetitions')}/{p.get('required_repetitions')} |"
        )
    lines.append("")
    lines.append("`CRASH` and ground-contact records are treated as unsafe outcome signals, not preventive contract violations. Preventive violations are parameter/readback errors, missing or mistimed min-alt fence action, unrelated failsafes, or unrelated dirty `STATUSTEXT`/`ERR` records.")
    lines.append("")
    lines.append("## Prediction")
    lines.append("")
    cls = payload.get("classification", {})
    sev = payload.get("severity_regression", {})
    lines.append(
        f"Classification: interpolation accuracy={fmt(cls.get('interpolation', {}).get('metrics', {}).get('classification_accuracy'), 3)}, "
        f"extrapolation accuracy={fmt(cls.get('extrapolation', {}).get('metrics', {}).get('classification_accuracy'), 3)}, "
        f"combined={fmt(cls.get('combined_heldout_accuracy'), 3)}."
    )
    lines.append(
        f"Severity regression: MAE={fmt(sev.get('metrics', {}).get('mae_m'), 4)} m, "
        f"bound={fmt(sev.get('metrics', {}).get('mae_bound_m'), 4)} m, pass={sev.get('metrics', {}).get('pass')}."
    )
    lines.append("")
    lines.append("## P Stratification")
    lines.append("")
    p_summary = payload.get("p_stratification", {}).get("summary", {})
    lines.append(p_summary.get("conclusion", "n/a"))
    lines.append("")
    lines.append("| layer | FENCE_ALT_MIN m | clean_unsafe | clean_safe | ambiguous | contract_violated |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for row in p_summary.get("counts", []):
        lines.append(
            f"| {row['layer']} | {fmt(row['fence_alt_min_m'], 1)} | {row['clean_unsafe']} | "
            f"{row['clean_safe']} | {row['ambiguous']} | {row['contract_violated']} |"
        )
    lines.append("")
    lines.append("## Search Efficiency")
    lines.append("")
    search = payload.get("search_efficiency", {})
    lines.append(f"{search.get('strategy', 'n/a')}: {search.get('query_count')} queries versus {search.get('full_grid_count')} full-grid points.")
    lines.append("")
    lines.append("## Three-Dimensional Unified Claim")
    lines.append("")
    lines.append("The three planc scenarios use the same threshold-insufficiency machine across three subsystems and dimensions: energy budget (`BATT_LOW_MAH`), time budget (`FS_GCS_TIMEOUT`), and height budget (`FENCE_ALT_MIN`). In this scenario the configured minimum-altitude fence triggers the specified RTL recovery, but the height budget can be insufficient under legal high descent rate and mass conditions.")
    lines.append("")
    lines.append("## Limits")
    lines.append("")
    lines.append("This remains SITL evidence. The unsafe consequence is defined by min-AGL relative to a 2 m danger floor and by ground-contact/CRASH outcome records. The `(b)` energy result remains the main result; this is the third subsystem generalization.")
    lines.append("")
    lines.append("## Artifacts")
    lines.append("")
    for name, path in payload.get("artifacts", {}).get("plots", {}).items():
        lines.append(f"- {name}: `{rel(path)}`")
    lines.append("- Structured results: `planc/results/minalt_groundcontact_results.json`")
    lines.append("- Parsed logs and sidecars: `planc/logs/minalt_*_params.json`, `planc/logs/minalt_*_parsed.csv`, `planc/logs/minalt_*_parsed.oracle.json`")
    lines.append("- Local raw DataFlash logs: `planc/logs/minalt_*.BIN` (ignored by Git, retained in this workspace when present)")
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(report)


def write_preregister(config: dict[str, Any], path: Path) -> dict[str, Any]:
    payload = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "scenario": config["experiment"]["name"],
        "version": config["experiment"]["version"],
        "h_floor_m": float(config["experiment"]["h_floor_m"]),
        "k_sigma_margin": float(config["noise_floor"]["k_sigma_margin"]),
        "c_sigma_mae_bound": float(config["noise_floor"]["c_sigma_mae_bound"]),
        "ci_sigma_multiplier": float(config["noise_floor"]["ci_sigma_multiplier"]),
        "noise_repetitions": int(config["noise_floor"]["noise_repetitions"]),
        "grid_repetitions": int(config["noise_floor"]["grid_repetitions"]),
        "boundary_points": config["noise_floor"]["boundary_points"],
        "unsafe_points": config["noise_floor"]["unsafe_points"],
        "label_rule": config["noise_floor"]["preregistered_oracle_note"],
    }
    if not path.exists():
        write_json(path, payload)
    return load_json(path, payload)


def build_payload(config: dict[str, Any], env: dict[str, Any], premise_runs: list[dict[str, Any]], noise_runs: list[dict[str, Any]], grid_runs: list[dict[str, Any]]) -> dict[str, Any]:
    premise = premise_summary(premise_runs, config)
    oracle = summarize_noise_runs(noise_runs, config)
    default_layer = str(config["sweep"]["default_layer"])
    default_points = aggregate_layer(
        grid_runs,
        config=config,
        layer=default_layer,
        required_repetitions=int(config["noise_floor"]["grid_repetitions"]),
        oracle=oracle,
    )
    layers: dict[str, dict[str, Any]] = {}
    for layer in config["sweep"]["p_layers"]:
        req = int(config["noise_floor"]["grid_repetitions"]) if layer == default_layer else int(config["noise_floor"]["p_layer_repetitions"])
        pts = aggregate_layer(grid_runs, config=config, layer=str(layer), required_repetitions=req, oracle=oracle)
        layers[str(layer)] = {
            "fence_alt_min_m": layer_alt_min(config, str(layer)),
            "required_repetitions": req,
            "points": pts,
            "zone_counts": zone_counts(pts),
        }
    classification = train_test_classification(default_points, config, oracle)
    severity = evaluate_severity_regression(default_points, config, oracle)
    p_summary = p_stratification_summary(layers)
    search = boundary_search_summary(default_points, config)
    verdict = verdict_summary(premise, default_points, classification, severity)
    payload = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "config": config,
        "environment": env,
        "premise": premise,
        "oracle": oracle,
        "default_grid": {
            "layer": default_layer,
            "fence_alt_min_m": layer_alt_min(config, default_layer),
            "points": default_points,
            "zone_counts": zone_counts(default_points),
        },
        "p_stratification": {
            "layers": layers,
            "summary": p_summary,
        },
        "classification": classification,
        "severity_regression": severity,
        "search_efficiency": search,
        "verdict": verdict,
        "runs": {
            "premise": premise_runs,
            "noise": noise_runs,
            "grid": grid_runs,
        },
    }
    payload["artifacts"] = {"plots": make_plots(payload)}
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def load_runs(path: Path, resume: bool) -> list[dict[str, Any]]:
    if not resume or not path.exists():
        return []
    return list(load_json(path).get("runs", []))


def schedule_premise(config: dict[str, Any], runs: list[dict[str, Any]], partial_path: Path, state: dict[str, int | None]) -> None:
    mech = config["premise"]["mechanism"]
    run_or_reuse(
        config,
        runs,
        partial_path,
        run_id=premise_run_id("mechanism", str(mech["model"])),
        run_kind="premise",
        layer=str(mech["layer"]),
        descent_rate_m_s=float(mech["descent_rate_m_s"]),
        model_name=str(mech["model"]),
        rep_index=1,
        roles=["premise_mechanism"],
        max_new_runs_state=state,
    )
    rate = config["premise"]["descent_rate_application"]
    run_or_reuse(
        config,
        runs,
        partial_path,
        run_id=premise_run_id("descent_rate", str(rate["model"])),
        run_kind="premise",
        layer=str(rate["layer"]),
        descent_rate_m_s=float(rate["descent_rate_m_s"]),
        model_name=str(rate["model"]),
        rep_index=1,
        roles=["premise_descent_rate_application"],
        max_new_runs_state=state,
    )
    mass = config["premise"]["mass_response"]
    for model_name in mass["models"]:
        run_or_reuse(
            config,
            runs,
            partial_path,
            run_id=premise_run_id("mass_response", str(model_name)),
            run_kind="premise",
            layer=str(mass["layer"]),
            descent_rate_m_s=float(mass["descent_rate_m_s"]),
            model_name=str(model_name),
            rep_index=1,
            roles=["premise_mass_response"],
            max_new_runs_state=state,
        )


def schedule_noise(config: dict[str, Any], runs: list[dict[str, Any]], partial_path: Path, state: dict[str, int | None]) -> None:
    reps = int(config["noise_floor"]["noise_repetitions"])
    for group_name, cfg_name in (("boundary", "boundary_points"), ("unsafe", "unsafe_points")):
        layer = f"noise_{group_name}"
        for spec in config["noise_floor"][cfg_name]:
            for rep in range(1, reps + 1):
                run_or_reuse(
                    config,
                    runs,
                    partial_path,
                    run_id=run_id_for(layer, float(spec["descent_rate_m_s"]), str(spec["model"]), rep, prefix="minalt_noise"),
                    run_kind="noise",
                    layer=layer,
                    descent_rate_m_s=float(spec["descent_rate_m_s"]),
                    model_name=str(spec["model"]),
                    rep_index=rep,
                    roles=[f"noise_{group_name}"],
                    max_new_runs_state=state,
                )


def schedule_grid(config: dict[str, Any], runs: list[dict[str, Any]], partial_path: Path, state: dict[str, int | None]) -> None:
    default_layer = str(config["sweep"]["default_layer"])
    for layer in config["sweep"]["p_layers"]:
        reps = int(config["noise_floor"]["grid_repetitions"]) if str(layer) == default_layer else int(config["noise_floor"]["p_layer_repetitions"])
        for rate in config["sweep"]["descent_rates_m_s"]:
            for model_name in config["sweep"]["model_order"]:
                for rep in range(1, reps + 1):
                    run_or_reuse(
                        config,
                        runs,
                        partial_path,
                        run_id=run_id_for(str(layer), float(rate), str(model_name), rep),
                        run_kind="grid",
                        layer=str(layer),
                        descent_rate_m_s=float(rate),
                        model_name=str(model_name),
                        rep_index=rep,
                        roles=["default_grid" if str(layer) == default_layer else "p_stratification"],
                        max_new_runs_state=state,
                    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "minalt_groundcontact_config.yaml")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-new-runs", type=int, default=None)
    parser.add_argument("--phase", choices=["all", "premise", "noise", "grid", "report"], default="all")
    args = parser.parse_args()

    config = load_config(args.config)
    results_dir = PLANC_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    premise_partial_path = results_dir / "minalt_groundcontact_premise_partial.json"
    noise_partial_path = results_dir / "minalt_groundcontact_noise_partial.json"
    grid_partial_path = results_dir / "minalt_groundcontact_grid_partial.json"
    final_path = results_dir / "minalt_groundcontact_results.json"
    prereg_path = results_dir / "minalt_groundcontact_oracle_preregistered.json"
    env_path = results_dir / "env_minalt_groundcontact.json"

    prereg = write_preregister(config, prereg_path)
    env = probe_environment(config, REPO_ROOT)
    env["oracle_preregistration"] = prereg
    write_env(env, env_path)
    if not args.resume and args.phase != "report":
        probe = connectivity_probe(config, env)
        write_env(env, env_path)
        if not probe.get("ok"):
            payload = {
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "config": config,
                "environment": env,
                "premise": {"satisfied": False, "reason": "SITL connectivity failed", "checks": [], "runs": []},
                "oracle": {},
                "default_grid": {"points": [], "zone_counts": {}},
                "p_stratification": {"layers": {}, "summary": {}},
                "classification": {},
                "severity_regression": {},
                "search_efficiency": {},
                "verdict": {"verdict": "INCONCLUSIVE", "premise_satisfied": False, "reason": "connectivity failed"},
                "runs": {"premise": [], "noise": [], "grid": []},
            }
            write_json(final_path, payload)
            print(f"BLOCKED: SITL connectivity probe failed; see {final_path}", flush=True)
            return 1

    premise_runs = load_runs(premise_partial_path, args.resume or args.phase == "report")
    noise_runs = load_runs(noise_partial_path, args.resume or args.phase == "report")
    grid_runs = load_runs(grid_partial_path, args.resume or args.phase == "report")
    state: dict[str, int | None] = {"remaining": args.max_new_runs}

    if args.phase in {"all", "premise"}:
        schedule_premise(config, premise_runs, premise_partial_path, state)
        write_json(premise_partial_path, {"runs": premise_runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})

    current_premise = premise_summary(premise_runs, config)
    if args.phase == "premise" or (args.phase == "all" and args.max_new_runs is not None and int(state.get("remaining") or 0) <= 0):
        payload = build_payload(config, env, premise_runs, noise_runs, grid_runs)
        write_json(final_path, payload)
        print(f"PARTIAL: verdict={payload['verdict']['verdict']} report={payload['artifacts']['report']}", flush=True)
        return 0

    if args.phase == "all" and not current_premise.get("satisfied"):
        payload = build_payload(config, env, premise_runs, noise_runs, grid_runs)
        write_json(final_path, payload)
        print(f"INCONCLUSIVE: premise failed; report={payload['artifacts']['report']}", flush=True)
        return 0

    if args.phase in {"all", "noise"}:
        schedule_noise(config, noise_runs, noise_partial_path, state)
        write_json(noise_partial_path, {"runs": noise_runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})

    if args.phase == "noise" or (args.phase == "all" and args.max_new_runs is not None and int(state.get("remaining") or 0) <= 0):
        payload = build_payload(config, env, premise_runs, noise_runs, grid_runs)
        write_json(final_path, payload)
        print(f"PARTIAL: verdict={payload['verdict']['verdict']} report={payload['artifacts']['report']}", flush=True)
        return 0

    if args.phase in {"all", "grid"}:
        schedule_grid(config, grid_runs, grid_partial_path, state)
        write_json(grid_partial_path, {"runs": grid_runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})

    payload = build_payload(config, env, premise_runs, noise_runs, grid_runs)
    write_json(final_path, payload)
    print(f"COMPLETE: verdict={payload['verdict']['verdict']} report={payload['artifacts']['report']} results={final_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
