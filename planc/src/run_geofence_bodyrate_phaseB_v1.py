"""geofence_bodyrate Phase-B v1 dynamic SITL campaign.

This runner adjudicates one matrix cell:

    horizontal geofence x GUIDED SET_ATTITUDE_TARGET body-rate

The tested interface sends a legal body roll-rate command with the attitude
quaternion ignored.  ArduCopter 4.4.1 routes that command to
ModeGuided::set_angle and then AC_AttitudeControl::input_rate_bf_roll_pitch_yaw
when the quaternion is zero, so the dynamic witness is a supported MAVLink
body-rate path rather than an ACRO fallback.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import statistics
import subprocess
import sys
import time
import traceback
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml
from pymavlink import mavutil

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
RESULT_ROOT = REPO_ROOT / "results" / "geofence_bodyrate_phaseB_v1"
PREREG_ROOT = REPO_ROOT / "prereg"
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
    wait_position_stable,
)
from oracle import COPTER_MODES, ERROR_CODES, ERROR_SUBSYSTEMS, EVENT_NAMES
from param_manager import ParamManager
from run_oracleA_v1 import MODE_REASONS
from sitl_runner import SitlRunner

csv.field_size_limit(10_000_000)

FENCE_SUBSYS = 9
NAV_SUBSYS = 22
DEST_OUTSIDE_FENCE = 5

BODYRATE_TYPE_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default if default is not None else {}
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    try:
        if isinstance(value, float) and math.isinf(value):
            return "inf"
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def percentile(values: list[float], q: float) -> float | None:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * q
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return clean[lo]
    return clean[lo] + (clean[hi] - clean[lo]) * (rank - lo)


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
    except Exception as exc:  # noqa: BLE001
        return f"ERROR: {exc}"
    return proc.stdout.strip()


def firmware_actual(config: dict[str, Any]) -> dict[str, Any]:
    root = str(config["firmware_anchor"]["source_tree"])
    return {
        "source_tree": root,
        "expected_tag": config["firmware_anchor"]["expected_tag"],
        "expected_sha": config["firmware_anchor"]["expected_sha"],
        "actual_sha": _run(["git", "rev-parse", "HEAD"], cwd=root),
        "actual_describe": _run(["git", "describe", "--tags", "--always", "--dirty"], cwd=root),
        "actual_status": _run(["git", "status", "--short", "--branch"], cwd=root),
        "binary": next((p for p in config["sitl"]["vehicle_binary_candidates"] if Path(p).exists()), None),
        "source_anchors": config["firmware_anchor"],
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


def _mode_name(data: dict[str, Any]) -> str:
    raw = _field(data, "ModeNum", "Mode")
    try:
        return COPTER_MODES.get(int(raw), str(raw))
    except Exception:
        return str(raw)


def _reason_name(data: dict[str, Any]) -> str:
    raw = _field(data, "Rsn", "Reason")
    try:
        return MODE_REASONS.get(int(raw), str(raw))
    except Exception:
        return str(raw)


def _latlon(raw: Any) -> float | None:
    if raw is None:
        return None
    val = float(raw)
    if abs(val) > 1000:
        val /= 1.0e7
    return val


def _home_ne(home: dict[str, Any], lat: float, lon: float) -> tuple[float, float]:
    home_lat = float(home["lat"])
    home_lon = float(home["lon"])
    north = (lat - home_lat) * 111_320.0
    east = (lon - home_lon) * 111_320.0 * math.cos(math.radians(home_lat))
    return north, east


def expected_action_modes(config: dict[str, Any], action: int) -> list[str]:
    if int(action) == int(config["param_metadata"]["fence_action_brake"]):
        return list(config["oracle"]["expected_action_modes_for_brake"])
    return list(config["oracle"]["expected_action_modes_for_rtl"])


def distance_from_global_int(msg: Any, home: dict[str, Any]) -> float:
    lat = float(getattr(msg, "lat")) / 1.0e7
    lon = float(getattr(msg, "lon")) / 1.0e7
    north, east = _home_ne(home, lat, lon)
    return math.hypot(north, east)


def update_online_from_msg(
    master: Any,
    msg: Any,
    online: dict[str, Any],
    *,
    start_wall: float,
    home: dict[str, Any],
    action_modes: set[str],
) -> bool:
    elapsed = time.time() - start_wall
    typ = msg.get_type()
    if typ == "HEARTBEAT":
        mode = mode_name(master, msg)
        if not online["modes"] or online["modes"][-1]["mode"] != mode:
            online["modes"].append({"wall_s": elapsed, "mode": mode})
        if mode in action_modes:
            online.setdefault("action_seen_wall_s", elapsed)
            return True
        if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            online.setdefault("disarmed_seen_wall_s", elapsed)
    elif typ == "STATUSTEXT":
        online["statustext"].append({"wall_s": elapsed, "text": str(getattr(msg, "text", ""))})
    elif typ == "FENCE_STATUS":
        rec = {
            "wall_s": elapsed,
            "breach_status": int(getattr(msg, "breach_status", 0)),
            "breach_type": int(getattr(msg, "breach_type", 0)),
            "breach_count": int(getattr(msg, "breach_count", 0)),
        }
        online["fence_status"].append(rec)
        if rec["breach_status"] and "breach_seen_wall_s" not in online:
            online["breach_seen_wall_s"] = elapsed
    elif typ == "GLOBAL_POSITION_INT":
        try:
            online["positions"].append({
                "wall_s": elapsed,
                "distance_m": distance_from_global_int(msg, home),
                "relative_alt_m": float(getattr(msg, "relative_alt", 0.0)) / 1000.0,
            })
        except Exception:
            pass
    return False


def send_timing_summary(send_wall_times: list[float], target_hz: float, speedup: float) -> dict[str, Any]:
    intervals = [b - a for a, b in zip(send_wall_times, send_wall_times[1:]) if b > a]
    mean_wall_dt = statistics.fmean(intervals) if intervals else None
    sim_hz = None if mean_wall_dt is None else 1.0 / (mean_wall_dt * max(speedup, 1.0))
    return {
        "count": len(send_wall_times),
        "target_sim_hz": float(target_hz),
        "mean_wall_dt_s": mean_wall_dt,
        "mean_estimated_sim_hz": sim_hz,
        "min_wall_dt_s": min(intervals) if intervals else None,
        "max_wall_dt_s": max(intervals) if intervals else None,
    }


def prepare_vehicle(master: Any, config: dict[str, Any]) -> None:
    request_streams(master, int(config["experiment"]["telemetry_rate_hz"]))
    wait_position_stable(master, min_samples=5, timeout_s=45)
    set_mode(master, "GUIDED", timeout_s=20)
    arm(master, timeout_s=45)
    alt_m = float(config["experiment"]["takeoff_alt_m"])
    command_takeoff(master, alt_m)
    wait_altitude(master, alt_m, timeout_s=70)
    set_mode(master, "GUIDED", timeout_s=20)


def point_run_id(config: dict[str, Any], point: dict[str, Any]) -> str:
    prefix = str(config["experiment"]["run_prefix"])
    role = str(point["role"]).replace("_", "")
    rate = int(round(float(point.get("rate_deg_s", 0.0))))
    wind = int(round(float(point.get("wind_m_s", 0.0))))
    margin10 = int(round(float(point.get("fence_margin_m", config["baseline_params"]["FENCE_MARGIN"])) * 10.0))
    rep = int(point.get("rep", 0))
    return f"{prefix}_{role}_r{rate:03d}_w{wind:02d}_m{margin10:03d}_n{rep:02d}"


def point_params(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    params = dict(config["baseline_params"])
    params["SIM_SPEEDUP"] = float(config["experiment"]["speedup"])
    params["SIM_WIND_SPD"] = float(point.get("wind_m_s", params["SIM_WIND_SPD"]))
    params["SIM_WIND_TURB"] = float(point.get("wind_turbulence", params["SIM_WIND_TURB"]))
    params["FENCE_ACTION"] = float(point.get("fence_action", params["FENCE_ACTION"]))
    params["FENCE_RADIUS"] = float(point.get("fence_radius_m", config["geometry"]["fence_radius_m"]))
    params["FENCE_MARGIN"] = float(point.get("fence_margin_m", params["FENCE_MARGIN"]))
    return params


def send_bodyrate_target(master: Any, roll_rate_deg_s: float, thrust: float) -> None:
    master.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        BODYRATE_TYPE_MASK,
        [0.0, 0.0, 0.0, 0.0],
        math.radians(float(roll_rate_deg_s)),
        0.0,
        0.0,
        float(thrust),
    )


def commanded_rate_sample(config: dict[str, Any], rate_deg_s: float, elapsed_s: float) -> float:
    sign = float(config["command"]["outward_roll_sign"])
    peak = abs(float(rate_deg_s)) * sign
    ramp_s = max(0.0, float(config["command"]["rate_ramp_s"]))
    pulse_s = max(ramp_s, float(config["command"]["rate_pulse_s"]))
    if ramp_s > 0.0 and elapsed_s < ramp_s:
        return peak * (elapsed_s / ramp_s)
    if elapsed_s < pulse_s:
        return peak
    return 0.0


def send_rate_segment(
    master: Any,
    config: dict[str, Any],
    *,
    duration_s: float,
    rate_deg_s: float,
    speedup: float,
    hz: float,
    start_wall: float,
    online: dict[str, Any],
    action_modes: set[str],
    profile: list[dict[str, Any]],
    label: str,
) -> bool:
    dt_wall = 1.0 / (hz * speedup)
    seg_start = time.time()
    action_seen = False
    thrust = float(config["command"]["thrust_field"])
    while (time.time() - seg_start) * speedup < duration_s:
        elapsed_sim = (time.time() - seg_start) * speedup
        send_gcs_heartbeat(master)
        send_bodyrate_target(master, rate_deg_s, thrust)
        online["send_wall_times"].append(time.time() - start_wall)
        profile.append({
            "wall_s": time.time() - start_wall,
            "segment": label,
            "segment_elapsed_sim_s": elapsed_sim,
            "body_roll_rate_deg_s": float(rate_deg_s),
            "type_mask": BODYRATE_TYPE_MASK,
        })
        drain_end = time.time() + dt_wall
        while time.time() < drain_end:
            msg = master.recv_match(
                type=["HEARTBEAT", "STATUSTEXT", "GLOBAL_POSITION_INT", "FENCE_STATUS"],
                blocking=True,
                timeout=max(0.0, drain_end - time.time()),
            )
            if msg is None:
                break
            action_seen = update_online_from_msg(
                master,
                msg,
                online,
                start_wall=start_wall,
                home=config["experiment"]["home"],
                action_modes=action_modes,
            ) or action_seen
        if action_seen:
            break
    return action_seen


def run_bodyrate_once(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    run_id = point_run_id(config, point)
    cfg = copy.deepcopy(config)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "kind": "bodyrate",
        "point": point,
        "started_at_utc": utc_now(),
    }
    master = None
    profile: list[dict[str, Any]] = []
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=35)
        params = point_params(config, point)
        pm = ParamManager(master)
        pm.apply(params)
        snapshot_names = sorted(set(params) | {"FENCE_TOTAL", "ACRO_RP_RATE", "ATC_RATE_FF_ENAB"})
        snapshot = pm.snapshot(snapshot_names)
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result.update({
            "work_dir": str(work_dir),
            "params_requested": params,
            "param_snapshot": snapshot,
            "param_records_path": str(param_path),
            "param_readbacks": pm.records,
        })

        prepare_vehicle(master, config)
        speedup = max(1.0, float(config["experiment"]["speedup"]))
        hz = float(config["experiment"]["stream_hz"])
        dt_wall = 1.0 / (hz * speedup)
        action_modes = set(expected_action_modes(config, int(params["FENCE_ACTION"])))
        online: dict[str, Any] = {"modes": [], "statustext": [], "fence_status": [], "positions": [], "send_wall_times": []}
        thrust = float(config["command"]["thrust_field"])
        start_wall = time.time()

        pre_end = time.time() + float(config["experiment"]["pre_stream_hold_s"]) / speedup
        while time.time() < pre_end:
            send_gcs_heartbeat(master)
            send_bodyrate_target(master, 0.0, thrust)
            online["send_wall_times"].append(time.time() - start_wall)
            time.sleep(dt_wall)

        max_stream_s = float(point.get("stream_limit_s", config["experiment"]["max_bodyrate_stream_s"]))
        stream_start_wall = time.time()
        action_seen = False
        commanded_peak = abs(float(point["rate_deg_s"]))
        while (time.time() - stream_start_wall) * speedup < max_stream_s:
            elapsed_sim = (time.time() - stream_start_wall) * speedup
            cmd_rate = commanded_rate_sample(config, commanded_peak, elapsed_sim)
            send_gcs_heartbeat(master)
            send_bodyrate_target(master, cmd_rate, thrust)
            online["send_wall_times"].append(time.time() - start_wall)
            profile.append({
                "wall_s": time.time() - start_wall,
                "segment": "outward_rate_then_hold",
                "segment_elapsed_sim_s": elapsed_sim,
                "body_roll_rate_deg_s": cmd_rate,
                "commanded_peak_abs_deg_s": commanded_peak,
                "type_mask": BODYRATE_TYPE_MASK,
            })
            drain_end = time.time() + dt_wall
            while time.time() < drain_end:
                msg = master.recv_match(
                    type=["HEARTBEAT", "STATUSTEXT", "GLOBAL_POSITION_INT", "FENCE_STATUS"],
                    blocking=True,
                    timeout=max(0.0, drain_end - time.time()),
                )
                if msg is None:
                    break
                action_seen = update_online_from_msg(
                    master,
                    msg,
                    online,
                    start_wall=start_wall,
                    home=config["experiment"]["home"],
                    action_modes=action_modes,
                ) or action_seen
            if action_seen:
                break

        online["bodyrate_stream_stopped_wall_s"] = time.time() - start_wall

        if not action_seen:
            recovery_rate = -float(config["command"]["outward_roll_sign"]) * commanded_peak
            action_seen = send_rate_segment(
                master,
                config,
                duration_s=float(config["command"]["rate_pulse_s"]),
                rate_deg_s=recovery_rate,
                speedup=speedup,
                hz=hz,
                start_wall=start_wall,
                online=online,
                action_modes=action_modes,
                profile=profile,
                label="cleanup_recover_level",
            ) or action_seen
            if not action_seen:
                _ = send_rate_segment(
                    master,
                    config,
                    duration_s=float(config["command"]["recovery_zero_hold_s"]),
                    rate_deg_s=0.0,
                    speedup=speedup,
                    hz=hz,
                    start_wall=start_wall,
                    online=online,
                    action_modes=action_modes,
                    profile=profile,
                    label="cleanup_zero_rate",
                )

        observe_end = time.time() + float(config["experiment"]["post_action_observation_s"]) / speedup
        while time.time() < observe_end:
            send_gcs_heartbeat(master)
            msg = master.recv_match(
                type=["HEARTBEAT", "STATUSTEXT", "GLOBAL_POSITION_INT", "FENCE_STATUS"],
                blocking=True,
                timeout=0.1,
            )
            if msg is not None:
                update_online_from_msg(
                    master,
                    msg,
                    online,
                    start_wall=start_wall,
                    home=config["experiment"]["home"],
                    action_modes=action_modes,
                )

        result["online_observation"] = {
            **online,
            "send_timing": send_timing_summary(online["send_wall_times"], hz, speedup),
            "mavlink_message_summary": {
                "SET_ATTITUDE_TARGET": len(online["send_wall_times"]),
                "SET_POSITION_TARGET_GLOBAL_INT": 0,
                "GUIDED_MODE_SET": 1,
                "GCS_HEARTBEAT": "continuous",
                "RC_CHANNELS_OVERRIDE": 0,
            },
        }
        profile_path = PLANC_ROOT / "logs" / f"{run_id}_command_profile.json"
        write_json(profile_path, {"run_id": run_id, "point": point, "type_mask": BODYRATE_TYPE_MASK, "samples": profile})
        result["command_profile"] = {
            "path": str(profile_path),
            "samples": len(profile),
            "commanded_peak_abs_deg_s": commanded_peak,
            "rate_pulse_s": float(config["command"]["rate_pulse_s"]),
            "max_stream_s": max_stream_s,
        }

        try:
            land_and_disarm(master, timeout_s=float(config["experiment"]["cleanup_land_timeout_s"]))
        except Exception as exc:  # noqa: BLE001
            result["cleanup_error"] = repr(exc)
        try:
            master.close()
        except Exception:
            pass
        master = None
        runner.stop()
        bin_path = runner.collect_dataflash(run_id)
        if bin_path is None:
            result["error"] = "No DataFlash .BIN log found after run"
            return result
        result["bin_path"] = str(bin_path)
        parsed = parse_bodyrate_dataflash(
            bin_path=bin_path,
            csv_path=PLANC_ROOT / "logs" / f"{run_id}_parsed.csv",
            oracle_path=PLANC_ROOT / "logs" / f"{run_id}_parsed.oracle.json",
            config=config,
            point=point,
        )
        result.update(parsed)
        return result
    except Exception as exc:  # noqa: BLE001
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


def first_crossing(rows: list[dict[str, Any]], radius_m: float) -> dict[str, Any]:
    rows = sorted(rows, key=lambda r: float(r["time_s"]))
    for prev, cur in zip(rows, rows[1:]):
        d0 = float(prev["distance_m"])
        d1 = float(cur["distance_m"])
        if d0 < radius_m <= d1:
            span = d1 - d0
            frac = 0.0 if abs(span) < 1.0e-9 else (radius_m - d0) / span
            t = float(prev["time_s"]) + frac * (float(cur["time_s"]) - float(prev["time_s"]))
            nearest = cur if frac >= 0.5 else prev
            return {
                "crossed": True,
                "time_s": t,
                "distance_m": radius_m,
                "nearest_row": nearest,
                "radial_speed_m_s": nearest.get("radial_speed_m_s"),
                "ground_speed_m_s": nearest.get("ground_speed_m_s"),
            }
    if rows and float(rows[0]["distance_m"]) >= radius_m:
        return {"crossed": True, "time_s": float(rows[0]["time_s"]), "distance_m": float(rows[0]["distance_m"]), "nearest_row": rows[0]}
    return {"crossed": False, "time_s": None}


def first_reentry(rows: list[dict[str, Any]], radius_m: float, cross_t: float | None, margin_m: float) -> float | None:
    if cross_t is None:
        return None
    threshold = radius_m - margin_m
    for row in sorted(rows, key=lambda r: float(r["time_s"])):
        t = float(row["time_s"])
        if t <= cross_t:
            continue
        if float(row["distance_m"]) <= threshold:
            return t
    return None


def sustained_arrest_time(rows: list[dict[str, Any]], cross_t: float | None, threshold_m_s: float, sustain_s: float) -> float | None:
    if cross_t is None:
        return None
    start: float | None = None
    prev_t: float | None = None
    for row in sorted(rows, key=lambda r: float(r["time_s"])):
        t = float(row["time_s"])
        if t < cross_t:
            continue
        radial = row.get("radial_speed_m_s")
        if radial is None:
            continue
        if float(radial) <= threshold_m_s:
            if start is None:
                start = t
            if t - start >= sustain_s:
                return start
        else:
            if prev_t is not None and start is not None and prev_t - start >= sustain_s:
                return start
            start = None
        prev_t = t
    return None


def max_depth(rows: list[dict[str, Any]], radius_m: float, lo: float | None, hi: float | None) -> float | None:
    if lo is None:
        return 0.0
    vals = []
    for row in rows:
        t = float(row["time_s"])
        if t < lo:
            continue
        if hi is not None and t > hi:
            continue
        vals.append(max(0.0, float(row["distance_m"]) - radius_m))
    return max(vals) if vals else 0.0


def nearest(rows: list[dict[str, Any]], t_s: float | None) -> dict[str, Any] | None:
    if not rows or t_s is None:
        return None
    return min(rows, key=lambda r: abs(float(r["time_s"]) - float(t_s)))


def infer_active_start(rate_rows: list[dict[str, Any]], command_peak: float) -> float | None:
    threshold = max(5.0, 0.2 * abs(command_peak))
    for row in rate_rows:
        des = _field(row, "RDes", "DesR")
        if des is None:
            continue
        if abs(float(des)) >= threshold:
            return float(row["time_s"])
    return None


def parse_bodyrate_dataflash(
    *,
    bin_path: Path,
    csv_path: Path,
    oracle_path: Path,
    config: dict[str, Any],
    point: dict[str, Any],
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    home = config["experiment"]["home"]
    radius_m = float(point.get("fence_radius_m", config["geometry"]["fence_radius_m"]))
    expected_modes = expected_action_modes(config, int(point.get("fence_action", config["baseline_params"]["FENCE_ACTION"])))
    start_t: float | None = None
    rows_all: list[dict[str, Any]] = []
    pos_rows: list[dict[str, Any]] = []
    xkf_rows: list[dict[str, Any]] = []
    att_rows: list[dict[str, Any]] = []
    rate_rows: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []

    while True:
        msg = mlog.recv_match(type=["POS", "XKF1", "ATT", "RATE", "MODE", "ERR", "MSG", "STAT", "EV", "PARM"], blocking=False)
        if msg is None:
            break
        typ = msg.get_type()
        if typ == "BAD_DATA":
            continue
        data = msg.to_dict()
        t_abs = _time_s(data)
        if t_abs is None:
            continue
        if start_t is None:
            start_t = t_abs
        t = t_abs - start_t
        row = {"time_s": t, "type": typ}
        row.update({k: v for k, v in data.items() if k not in {"mavpackettype", "TimeUS", "TimeMS"}})

        if typ == "POS":
            lat = _latlon(_field(data, "Lat"))
            lon = _latlon(_field(data, "Lng", "Lon"))
            if lat is not None and lon is not None and abs(lat) > 1.0e-9 and abs(lon) > 1.0e-9:
                north, east = _home_ne(home, lat, lon)
                prow = {
                    "time_s": t,
                    "type": "POS",
                    "lat": lat,
                    "lon": lon,
                    "north_m": north,
                    "east_m": east,
                    "distance_m": math.hypot(north, east),
                    "rel_alt_m": _field(data, "RelHomeAlt"),
                }
                pos_rows.append(prow)
                rows_all.append(prow)
        elif typ == "XKF1":
            core = _field(data, "C")
            if core is None or int(core) == 0:
                pn = float(_field(data, "PN") or 0.0)
                pe = float(_field(data, "PE") or 0.0)
                vn = float(_field(data, "VN") or 0.0)
                ve = float(_field(data, "VE") or 0.0)
                dist = math.hypot(pn, pe)
                radial = None if dist < 1.0e-6 else (pn * vn + pe * ve) / dist
                bearing = math.radians(float(config["experiment"]["target_bearing_deg"]))
                forward = vn * math.cos(bearing) + ve * math.sin(bearing)
                xrow = {
                    "time_s": t,
                    "type": "XKF1",
                    "north_m": pn,
                    "east_m": pe,
                    "distance_m": dist,
                    "vn_m_s": vn,
                    "ve_m_s": ve,
                    "ground_speed_m_s": math.hypot(vn, ve),
                    "radial_speed_m_s": radial,
                    "target_bearing_speed_m_s": forward,
                }
                xkf_rows.append(xrow)
                rows_all.append(xrow)
        elif typ == "ATT":
            att_rows.append(row)
            rows_all.append(row)
        elif typ == "RATE":
            rate_rows.append(row)
            rows_all.append(row)
        elif typ == "MODE":
            rec = {
                "time_s": t,
                "mode": _mode_name(data),
                "reason": _field(data, "Rsn", "Reason"),
                "reason_name": _reason_name(data),
                "raw": data,
            }
            modes.append(rec)
            rows_all.append({"time_s": t, "type": "MODE", "mode": rec["mode"], "reason": rec["reason"], "reason_name": rec["reason_name"]})
        elif typ == "ERR":
            subsys = int(_field(data, "Subsys", "SubSystem") or -1)
            ecode = int(_field(data, "ECode", "Code") or 0)
            rec = {
                "time_s": t,
                "subsys": subsys,
                "subsys_name": ERROR_SUBSYSTEMS.get(subsys, str(subsys)),
                "ecode": ecode,
                "ecode_name": ERROR_CODES.get((subsys, ecode), str(ecode)),
                "raw": data,
            }
            errors.append(rec)
            rows_all.append({"time_s": t, "type": "ERR", **{k: v for k, v in rec.items() if k != "raw"}})
        elif typ in {"MSG", "STAT"}:
            text = str(_field(data, "Message", "Msg", "Text") or "")
            rec = {"time_s": t, "text": text, "raw": data}
            messages.append(rec)
            rows_all.append({"time_s": t, "type": typ, "text": text})
        elif typ == "EV":
            event_id = int(_field(data, "Id") or -1)
            rec = {"time_s": t, "id": event_id, "name": EVENT_NAMES.get(event_id, str(event_id)), "raw": data}
            events.append(rec)
            rows_all.append({"time_s": t, "type": "EV", "id": event_id, "name": rec["name"]})

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if rows_all:
        fields = ["time_s", "type"]
        extra = sorted({k for row in rows_all for k in row if k not in fields})
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields + extra, lineterminator="\n")
            writer.writeheader()
            for row in sorted(rows_all, key=lambda r: float(r["time_s"])):
                writer.writerow(row)

    motion_rows = xkf_rows if len(xkf_rows) >= 10 else pos_rows
    motion_rows = sorted(motion_rows, key=lambda r: float(r["time_s"]))
    crossing = first_crossing(motion_rows, radius_m)
    cross_t = crossing.get("time_s")
    reentry_t = first_reentry(
        motion_rows,
        radius_m,
        float(cross_t) if cross_t is not None else None,
        float(config["oracle"].get("reentry_margin_m", 0.5)),
    )
    arrest_t = sustained_arrest_time(
        motion_rows,
        float(cross_t) if cross_t is not None else None,
        float(config["oracle"]["arrest_radial_speed_m_s"]),
        float(config["oracle"]["arrest_sustain_s"]),
    )
    max_distance = max((float(r["distance_m"]) for r in motion_rows), default=0.0)
    cross_row = nearest(motion_rows, float(cross_t) if cross_t is not None else None)

    fence_events = [e for e in errors if int(e["subsys"]) == FENCE_SUBSYS and int(e["ecode"]) != 0]
    fence_msgs = [m for m in messages if "fence" in str(m.get("text", "")).lower()]
    breach_msg = next((m for m in fence_msgs if "breached" in str(m.get("text", "")).lower()), None)
    fence_breach_time = float(fence_events[0]["time_s"]) if fence_events else (float(breach_msg["time_s"]) if breach_msg else None)
    nav_reject_events = [
        e for e in errors
        if int(e["subsys"]) == NAV_SUBSYS and int(e["ecode"]) == DEST_OUTSIDE_FENCE
    ]

    action_time = None
    action_mode = None
    for mode_row in modes:
        mt = float(mode_row["time_s"])
        if cross_t is not None and mt < float(cross_t) - 0.5:
            continue
        reason = str(mode_row.get("reason_name"))
        mode = str(mode_row.get("mode"))
        if reason == "FENCE_BREACHED" or (
            fence_breach_time is not None
            and 0 <= mt - float(fence_breach_time) <= float(config["oracle"]["action_latency_s"])
            and mode in expected_modes
        ):
            action_time = mt
            action_mode = mode
            break

    command_peak = abs(float(point.get("rate_deg_s", 0.0)))
    command_sign = float(config["command"]["outward_roll_sign"])
    active_start = infer_active_start(rate_rows, command_peak)
    cleanup_land_time = None
    for mode_row in modes:
        if str(mode_row.get("mode")) == "LAND" and str(mode_row.get("reason_name")) == "GCS_COMMAND":
            mt = float(mode_row["time_s"])
            if active_start is None or mt > active_start:
                cleanup_land_time = mt
                break
    pulse_end = None if active_start is None else active_start + float(config["command"]["rate_pulse_s"]) + 0.25
    rate_window = [
        row for row in rate_rows
        if active_start is not None
        and float(row["time_s"]) >= active_start
        and (pulse_end is None or float(row["time_s"]) <= pulse_end)
    ]
    actual_roll_rates = [command_sign * float(_field(r, "R", "Roll") or 0.0) for r in rate_window]
    desired_roll_rates = [command_sign * float(_field(r, "RDes", "DesR") or 0.0) for r in rate_window]
    peak_actual = max(actual_roll_rates) if actual_roll_rates else None
    peak_desired = max(desired_roll_rates) if desired_roll_rates else None
    actual_ratio = None if peak_actual is None or command_peak <= 0.0 else peak_actual / command_peak
    desired_ratio = None if peak_desired is None or command_peak <= 0.0 else peak_desired / command_peak

    active_hi = reentry_t or arrest_t or action_time
    if active_start is not None and active_hi is None:
        active_hi = active_start + float(config["experiment"]["max_bodyrate_stream_s"]) + 3.0
    active_att = [
        row for row in att_rows
        if active_start is not None
        and float(row["time_s"]) >= active_start
        and (active_hi is None or float(row["time_s"]) <= active_hi)
    ]
    aligned_roll = [command_sign * float(r.get("Roll", 0.0) or 0.0) for r in active_att]
    abs_roll = [abs(float(r.get("Roll", 0.0) or 0.0)) for r in active_att]
    abs_pitch = [abs(float(r.get("Pitch", 0.0) or 0.0)) for r in active_att]

    approach_window = config["oracle"]["speed_audit_inside_window_m"]
    approach_rows = [
        r for r in motion_rows
        if r.get("radial_speed_m_s") is not None
        and float(approach_window[0]) <= float(r["distance_m"]) <= float(approach_window[1])
        and (cross_t is None or float(r["time_s"]) <= float(cross_t))
    ]
    approach_speeds = [float(r["radial_speed_m_s"]) for r in approach_rows]
    ground_speeds = [float(r.get("ground_speed_m_s", 0.0)) for r in approach_rows if r.get("ground_speed_m_s") is not None]

    measurement_end = cleanup_land_time
    if measurement_end is None and active_start is not None:
        measurement_end = active_start + float(config["experiment"]["max_bodyrate_stream_s"]) + float(config["experiment"]["post_action_observation_s"]) + 2.0
    pos_alt = [
        float(r.get("rel_alt_m"))
        for r in pos_rows
        if r.get("rel_alt_m") not in (None, "")
        and (active_start is None or float(r["time_s"]) >= active_start)
        and (measurement_end is None or float(r["time_s"]) <= measurement_end)
    ]
    min_alt = min(pos_alt) if pos_alt else None
    crash_text = [
        m for m in messages
        if any(marker in str(m.get("text", "")).lower() for marker in ("crash", "sim hit ground"))
        and (measurement_end is None or float(m["time_s"]) <= measurement_end)
    ]
    low_altitude = bool(min_alt is not None and min_alt < float(config["oracle"]["min_altitude_agl_m"]))

    preventive_subsystems = set(config["oracle"]["preventive_failsafe_subsystems"])
    preventive_reasons = set(config["oracle"]["preventive_mode_reasons"])
    hard_time = action_time or fence_breach_time or arrest_t or reentry_t
    preventive_errors = [
        e for e in errors
        if int(e["ecode"]) != 0
        and str(e["subsys_name"]) in preventive_subsystems
        and (hard_time is None or float(e["time_s"]) <= float(hard_time))
    ]
    preventive_modes = [
        m for m in modes
        if str(m.get("reason_name")) in preventive_reasons
        and (hard_time is None or float(m["time_s"]) <= float(hard_time))
    ]
    bad_messages = []
    for m in messages:
        low = str(m.get("text", "")).lower()
        if "fence breached" in low or "new fence" in low or "fence requires position" in low:
            continue
        normal_ekf = (
            "ekf3" in low
            and any(marker in low for marker in ("initialised", "alignment complete", "origin set", "is using gps", "active"))
        )
        if normal_ekf:
            continue
        if any(marker in low for marker in ("failsafe", "ekf failsafe", "gps glitch", "terrain failsafe", "terminate")):
            bad_messages.append(m)

    ratio_bounds = config["oracle"]["rate_tracking_ratio_bounds"]
    input_applied_ok = bool(
        actual_ratio is not None
        and float(ratio_bounds[0]) <= float(actual_ratio) <= float(ratio_bounds[1])
        and desired_ratio is not None
        and float(ratio_bounds[0]) <= float(desired_ratio) <= float(ratio_bounds[1])
    )

    result: dict[str, Any] = {
        "bin_path": str(bin_path),
        "csv_path": str(csv_path),
        "oracle_path": str(oracle_path),
        "kind": "bodyrate",
        "interface_path": "GUIDED + SET_ATTITUDE_TARGET body roll-rate; attitude quaternion ignored",
        "fence_radius_m": radius_m,
        "fence_margin_m": float(point.get("fence_margin_m", config["baseline_params"]["FENCE_MARGIN"])),
        "wind_m_s": float(point.get("wind_m_s", 0.0)),
        "wind_turbulence": float(point.get("wind_turbulence", 0.0)),
        "position_source": "XKF1" if len(xkf_rows) >= 10 else "POS",
        "samples": {"xkf1": len(xkf_rows), "pos": len(pos_rows), "att": len(att_rows), "rate": len(rate_rows)},
        "crossing": crossing,
        "reentry_time_s": reentry_t,
        "outside_duration_s": None if cross_t is None or reentry_t is None else float(reentry_t) - float(cross_t),
        "fence_breach_detected": fence_breach_time is not None,
        "fence_breach_time_s": fence_breach_time,
        "fence_breach_latency_from_cross_s": None if fence_breach_time is None or cross_t is None else float(fence_breach_time) - float(cross_t),
        "fence_events": fence_events,
        "fence_messages": fence_msgs[:30],
        "action_started": action_time is not None,
        "action_time_s": action_time,
        "action_mode": action_mode,
        "action_latency_from_cross_s": None if action_time is None or cross_t is None else float(action_time) - float(cross_t),
        "arrest_time_s": arrest_t,
        "arrest_latency_from_cross_s": None if arrest_t is None or cross_t is None else float(arrest_t) - float(cross_t),
        "max_distance_m": max_distance,
        "max_depth_m": max(0.0, max_distance - radius_m),
        "max_depth_until_reentry_m": max_depth(motion_rows, radius_m, float(cross_t) if cross_t is not None else None, reentry_t),
        "max_depth_before_action_m": max_depth(motion_rows, radius_m, float(cross_t) if cross_t is not None else None, action_time),
        "max_depth_before_arrest_m": max_depth(motion_rows, radius_m, float(cross_t) if cross_t is not None else None, arrest_t),
        "achieved_cross_speed_m_s": None if cross_row is None else cross_row.get("radial_speed_m_s"),
        "achieved_cross_ground_speed_m_s": None if cross_row is None else cross_row.get("ground_speed_m_s"),
        "speed_audit": {
            "source": "XKF1 primary-core radial velocity in the inside approach window",
            "inside_window_m": approach_window,
            "samples": len(approach_rows),
            "median_radial_speed_m_s": median(approach_speeds),
            "mean_radial_speed_m_s": mean(approach_speeds),
            "max_radial_speed_m_s": max(approach_speeds) if approach_speeds else None,
            "median_ground_speed_m_s": median(ground_speeds),
        },
        "bodyrate_tracking": {
            "active_start_s": active_start,
            "pulse_window_end_s": pulse_end,
            "commanded_peak_abs_deg_s": command_peak,
            "desired_peak_aligned_deg_s": peak_desired,
            "actual_peak_aligned_deg_s": peak_actual,
            "desired_to_command_ratio": desired_ratio,
            "actual_to_command_ratio": actual_ratio,
            "ratio_bounds": ratio_bounds,
            "input_applied_ok": input_applied_ok,
        },
        "attitude_extrema": {
            "max_aligned_roll_deg": max(aligned_roll) if aligned_roll else None,
            "max_abs_roll_deg": max(abs_roll) if abs_roll else None,
            "max_abs_pitch_deg": max(abs_pitch) if abs_pitch else None,
        },
        "altitude": {
            "min_rel_alt_m": min_alt,
            "low_altitude_or_crash": bool(low_altitude or crash_text),
            "crash_text": crash_text[:10],
        },
        "destination_admission_reject_count": len(nav_reject_events),
        "destination_admission_rejects": nav_reject_events,
        "modes": modes,
        "messages_tail": messages[-30:],
        "events": events,
        "errors": errors,
        "preventive_contract": {
            "preventive_errors": preventive_errors,
            "preventive_modes": preventive_modes,
            "bad_messages": bad_messages,
            "violated": bool(preventive_errors or preventive_modes or bad_messages),
            "fence_action_is_reactive_not_violation": True,
            "destination_guard_has_no_position_target_on_bodyrate_path": True,
        },
    }
    write_json(oracle_path, result)
    return result


def stage_points(config: dict[str, Any], stage: str) -> list[dict[str, Any]]:
    sweep = config["sweep"]
    base_margin = float(config["baseline_params"]["FENCE_MARGIN"])
    radius = float(config["geometry"]["fence_radius_m"])
    points: list[dict[str, Any]] = []

    def add(role: str, rate: float, wind: float, rep: int, *, margin: float | None = None, turb: float | None = None) -> None:
        points.append({
            "kind": "bodyrate",
            "role": role,
            "rate_deg_s": float(rate),
            "wind_m_s": float(wind),
            "wind_turbulence": float(sweep["noise_turbulence"] if turb is None else turb),
            "fence_margin_m": float(base_margin if margin is None else margin),
            "fence_radius_m": radius,
            "fence_action": int(config["baseline_params"]["FENCE_ACTION"]),
            "rep": int(rep),
        })

    if stage in {"premise", "all"}:
        add("premise", float(sweep["premise_rate_deg_s"]), float(sweep["premise_wind_m_s"]), 0, turb=0.0)

    if stage in {"noise", "all"}:
        for rep in range(int(sweep["noise_repetitions"])):
            add("noise", float(sweep["noise_rate_deg_s"]), float(sweep["noise_wind_m_s"]), rep)

    if stage in {"scan", "all"}:
        reps = int(sweep["scan_repetitions"])
        for rate in sweep["scan_rates_deg_s"]:
            for wind in sweep["scan_winds_m_s"]:
                for rep in range(reps):
                    add("scan", float(rate), float(wind), rep)
        extra = int(sweep.get("boundary_extra_repetitions", 0))
        for rate in sweep.get("boundary_rates_deg_s", []):
            for wind in sweep.get("boundary_winds_m_s", []):
                for rep in range(reps, reps + extra):
                    add("scan", float(rate), float(wind), rep)

    if stage in {"stratify", "all"}:
        reps = int(sweep["margin_layer_repetitions"])
        for margin in sweep["margin_layers_m"]:
            for rate in sweep["scan_rates_deg_s"]:
                for wind in sweep["scan_winds_m_s"]:
                    for rep in range(reps):
                        add("stratify", float(rate), float(wind), rep, margin=float(margin))

    seen = set()
    out = []
    for point in points:
        rid = point_run_id(config, point)
        if rid in seen:
            continue
        seen.add(rid)
        out.append(point)
    return out


def run_cached(
    config: dict[str, Any],
    point: dict[str, Any],
    partial_path: Path,
    runs: list[dict[str, Any]],
    resume: bool,
) -> list[dict[str, Any]]:
    run_id = point_run_id(config, point)
    if resume:
        for existing in runs:
            if existing.get("run_id") == run_id and not existing.get("error"):
                print(f"CACHED {run_id}", flush=True)
                return runs
    print(f"RUN {run_id}", flush=True)
    run = run_bodyrate_once(config, point)
    runs = [r for r in runs if r.get("run_id") != run_id] + [run]
    write_json(partial_path, {"runs": runs, "updated_at_utc": utc_now()})
    print(
        f"DONE {run_id} err={bool(run.get('error'))} crossed={run.get('crossing', {}).get('crossed')} "
        f"rate={fmt(run.get('bodyrate_tracking', {}).get('actual_peak_aligned_deg_s'))}/{fmt(run.get('bodyrate_tracking', {}).get('commanded_peak_abs_deg_s'))} "
        f"roll={fmt(run.get('attitude_extrema', {}).get('max_aligned_roll_deg'))} "
        f"depth={fmt(run.get('max_depth_until_reentry_m'))} action={run.get('action_mode')}",
        flush=True,
    )
    return runs


def run_role(run: dict[str, Any]) -> str:
    return str(run.get("point", {}).get("role", ""))


def complete_runs(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [r for r in runs if not r.get("error")]


def depth_for_label(run: dict[str, Any]) -> float:
    if run.get("max_depth_until_reentry_m") is not None:
        return float(run.get("max_depth_until_reentry_m") or 0.0)
    return float(run.get("max_depth_m") or 0.0)


def label_run(run: dict[str, Any], d_margin_m: float) -> str:
    if run.get("error"):
        return "blocked"
    if run.get("preventive_contract", {}).get("violated"):
        return "contract_violated"
    if run.get("altitude", {}).get("low_altitude_or_crash"):
        return "ambiguous"
    if not run.get("bodyrate_tracking", {}).get("input_applied_ok"):
        return "ambiguous"
    if not run.get("crossing", {}).get("crossed"):
        return "clean_safe"
    depth = depth_for_label(run)
    if depth > d_margin_m:
        return "clean_unsafe"
    if depth > 0.0:
        return "ambiguous"
    return "clean_safe"


def attach_labels(runs: list[dict[str, Any]], d_margin_m: float) -> None:
    for run in runs:
        run["label"] = label_run(run, d_margin_m)


def noise_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    vals = [
        depth_for_label(r)
        for r in complete_runs(runs)
        if run_role(r) == "noise" and r.get("max_depth_until_reentry_m") is not None
    ]
    sigma = stdev(vals)
    return {"depths_m": vals, "n": len(vals), "sigma_m": sigma, "d_margin_m": 3.0 * sigma}


def premise_checks(config: dict[str, Any], runs: list[dict[str, Any]], d_margin_m: float) -> dict[str, Any]:
    complete = complete_runs(runs)
    expected_rate = float(config["sweep"]["premise_rate_deg_s"])
    premise = next(
        (
            r for r in complete
            if run_role(r) == "premise"
            and math.isclose(float(r.get("point", {}).get("rate_deg_s", -1.0)), expected_rate)
        ),
        None,
    ) or next((r for r in complete if run_role(r) == "premise"), None)
    bodyrate = [r for r in complete if r.get("kind") == "bodyrate"]
    all_no_dest_reject = all(int(r.get("destination_admission_reject_count", 0)) == 0 for r in bodyrate)
    send_rates = [
        r.get("online_observation", {}).get("send_timing", {}).get("mean_estimated_sim_hz")
        for r in bodyrate
        if r.get("online_observation", {}).get("send_timing", {}).get("mean_estimated_sim_hz") is not None
    ]
    target_hz = float(config["experiment"]["stream_hz"])
    required_hz = float(config["experiment"].get("min_required_setpoint_hz", target_hz * 0.5))
    p = {
        "P0.1_guided_bodyrate_supported_and_applied": {
            "ok": bool(premise and premise.get("bodyrate_tracking", {}).get("input_applied_ok")),
            "premise_run_id": None if premise is None else premise.get("run_id"),
            "tracking": None if premise is None else premise.get("bodyrate_tracking"),
            "interface_path": "GUIDED + SET_ATTITUDE_TARGET with ATTITUDE_IGNORE",
        },
        "P0.2_bodyrate_has_no_destination_admission_reject": {
            "ok": bool(bodyrate and all_no_dest_reject),
            "destination_reject_count": sum(int(r.get("destination_admission_reject_count", 0)) for r in bodyrate),
            "static_anchor": config["firmware_anchor"].get("source_destination_guard"),
        },
        "P0.3_bodyrate_drives_horizontal_crossing": {
            "ok": bool(
                premise
                and premise.get("crossing", {}).get("crossed")
                and abs(float(premise.get("achieved_cross_speed_m_s") or 0.0)) >= float(config["oracle"]["premise_min_cross_speed_m_s"])
            ),
            "achieved_cross_speed_m_s": None if premise is None else premise.get("achieved_cross_speed_m_s"),
            "speed_audit": None if premise is None else premise.get("speed_audit"),
            "crossing": None if premise is None else premise.get("crossing"),
        },
        "P0.4_reactive_fence_action_fires": {
            "ok": bool(premise and premise.get("fence_breach_detected") and premise.get("action_started")),
            "fence_breach_time_s": None if premise is None else premise.get("fence_breach_time_s"),
            "action_time_s": None if premise is None else premise.get("action_time_s"),
            "action_mode": None if premise is None else premise.get("action_mode"),
        },
        "P0.5_fence_and_avoidance_params_legal": {
            "ok": bool(bodyrate and all(
                int(round(float(r.get("param_snapshot", {}).get("FENCE_ENABLE", 0)))) == 1
                and int(round(float(r.get("param_snapshot", {}).get("FENCE_TYPE", 0)))) & int(config["param_metadata"]["fence_type_circle"])
                and int(round(float(r.get("param_snapshot", {}).get("AVOID_ENABLE", 1)))) == 0
                and int(round(float(r.get("param_snapshot", {}).get("FENCE_ACTION", 0)))) != int(config["param_metadata"]["fence_action_report_only"])
                for r in bodyrate
            )),
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": int(config["param_metadata"]["fence_type_circle"]),
            "AVOID_ENABLE": 0,
        },
        "P0.6_stream_rate_and_altitude_fidelity": {
            "ok": bool(
                bodyrate
                and send_rates
                and min(send_rates) >= required_hz
                and premise
                and not premise.get("altitude", {}).get("low_altitude_or_crash")
            ),
            "target_stream_hz_sim": target_hz,
            "min_required_stream_hz_sim": required_hz,
            "min_estimated_stream_hz_sim": min(send_rates) if send_rates else None,
            "premise_min_alt_m": None if premise is None else premise.get("altitude", {}).get("min_rel_alt_m"),
            "d_margin_m": d_margin_m,
        },
    }
    p["all_ok"] = all(bool(row["ok"]) for row in p.values() if isinstance(row, dict) and "ok" in row)
    return p


def cell_summary(runs: list[dict[str, Any]], role: str = "scan") -> list[dict[str, Any]]:
    groups: dict[tuple[float, float, float], list[dict[str, Any]]] = defaultdict(list)
    for run in complete_runs(runs):
        if run_role(run) != role:
            continue
        point = run.get("point", {})
        groups[(float(point["rate_deg_s"]), float(point["wind_m_s"]), float(point["fence_margin_m"]))].append(run)
    out = []
    for (rate, wind, margin), rows in sorted(groups.items()):
        labels = Counter(str(r.get("label")) for r in rows)
        depths = [depth_for_label(r) for r in rows if r.get("label") != "blocked"]
        speeds = [float(r["achieved_cross_speed_m_s"]) for r in rows if r.get("achieved_cross_speed_m_s") is not None]
        rolls = [float(r["attitude_extrema"]["max_aligned_roll_deg"]) for r in rows if r.get("attitude_extrema", {}).get("max_aligned_roll_deg") is not None]
        nonamb = [str(r.get("label")) for r in rows if r.get("label") not in {"ambiguous", "blocked"}]
        stable = bool(len(rows) >= 2 and nonamb and len(set(nonamb)) == 1)
        out.append({
            "rate_deg_s": rate,
            "wind_m_s": wind,
            "fence_margin_m": margin,
            "runs": len(rows),
            "labels": dict(labels),
            "stable": stable,
            "mean_depth_m": mean(depths),
            "median_depth_m": median(depths),
            "max_depth_m": max(depths) if depths else None,
            "mean_cross_speed_m_s": mean(speeds),
            "mean_peak_roll_deg": mean(rolls),
            "run_ids": [r["run_id"] for r in rows],
        })
    return out


def _average_ranks(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i + 1
        while j < len(order) and values[order[j]] == values[order[i]]:
            j += 1
        rank = (i + 1 + j) / 2.0
        for k in range(i, j):
            ranks[order[k]] = rank
        i = j
    return ranks


def _pearson(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) != len(ys) or len(xs) < 2:
        return None
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    dx = [x - mx for x in xs]
    dy = [y - my for y in ys]
    sx = math.sqrt(sum(x * x for x in dx))
    sy = math.sqrt(sum(y * y for y in dy))
    if sx <= 0.0 or sy <= 0.0:
        return None
    return sum(x * y for x, y in zip(dx, dy)) / (sx * sy)


def spearman_rho(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) != len(ys) or len(xs) < 2:
        return None
    return _pearson(_average_ranks(xs), _average_ranks(ys))


def feature_row(run: dict[str, Any]) -> list[float]:
    point = run["point"]
    roll = run.get("attitude_extrema", {}).get("max_aligned_roll_deg")
    if roll is None:
        roll = 0.0
    return [
        1.0,
        float(point["rate_deg_s"]),
        float(point["wind_m_s"]),
        float(roll),
    ]


def prediction_eval(config: dict[str, Any], runs: list[dict[str, Any]], d_margin_m: float, sigma_m: float) -> dict[str, Any]:
    target = float(config["prediction"]["target_classification_accuracy"])
    train_rate_max = float(config["prediction"]["train_rate_max_deg_s"])
    train_wind_max = float(config["prediction"]["train_wind_max_m_s"])
    extra_rate_min = float(config["prediction"]["extrapolate_rate_min_deg_s"])
    extra_wind_min = float(config["prediction"]["extrapolate_wind_min_m_s"])

    population = [
        r for r in complete_runs(runs)
        if run_role(r) == "scan"
        and math.isclose(float(r.get("point", {}).get("fence_margin_m", 0.0)), float(config["baseline_params"]["FENCE_MARGIN"]))
        and r.get("label") in {"clean_safe", "clean_unsafe"}
    ]
    train = [
        r for r in population
        if float(r["point"]["rate_deg_s"]) <= train_rate_max
        and float(r["point"]["wind_m_s"]) <= train_wind_max
    ]
    test = [r for r in population if r not in train]
    extrapolate = [
        r for r in test
        if float(r["point"]["rate_deg_s"]) >= extra_rate_min
        and float(r["point"]["wind_m_s"]) >= extra_wind_min
    ]
    if len(train) < 4 or len(test) < 1:
        return {
            "classification": {"applicable": False, "reason": "insufficient train/test after fixed extrapolation split", "passed": False},
            "severity": {"applicable": False, "passed": False},
            "regression": {"applicable": False},
        }

    xs = np.array([feature_row(r) for r in train], dtype=float)
    ys = np.array([depth_for_label(r) for r in train], dtype=float)
    coeff = np.linalg.lstsq(xs, ys, rcond=None)[0]

    def predict_depth(run: dict[str, Any]) -> float:
        return float(np.dot(np.array(feature_row(run), dtype=float), coeff))

    preds = []
    for run in test:
        pd = predict_depth(run)
        pred_label = "clean_unsafe" if pd > d_margin_m else "clean_safe"
        preds.append({
            "run_id": run["run_id"],
            "rate_deg_s": run["point"]["rate_deg_s"],
            "wind_m_s": run["point"]["wind_m_s"],
            "actual": run["label"],
            "predicted": pred_label,
            "actual_depth_m": depth_for_label(run),
            "predicted_depth_m": pd,
            "is_extrapolation": run in extrapolate,
        })
    accuracy = sum(1 for p in preds if p["actual"] == p["predicted"]) / len(preds) if preds else None
    extra_preds = [p for p in preds if p["is_extrapolation"]]
    extra_accuracy = sum(1 for p in extra_preds if p["actual"] == p["predicted"]) / len(extra_preds) if extra_preds else None

    actual_depths = [p["actual_depth_m"] for p in preds]
    pred_depths = [p["predicted_depth_m"] for p in preds]
    rho = spearman_rho(pred_depths, actual_depths)
    all_depths = [depth_for_label(r) for r in population]
    depth_range = max(all_depths) - min(all_depths) if all_depths else None
    if depth_range is None:
        range_over_sigma = None
    elif sigma_m > 0.0:
        range_over_sigma = depth_range / sigma_m
    else:
        range_over_sigma = math.inf if depth_range > 0.0 else 0.0
    errors = [abs(a - p) for a, p in zip(actual_depths, pred_depths)]
    mae = statistics.fmean(errors) if errors else None
    mae_over_range = None if mae is None or not depth_range else mae / depth_range
    rho_min = float(config["prediction"]["severity_spearman_min"])
    ros_min = float(config["prediction"]["severity_range_over_sigma_min"])
    classification_passed = bool(
        accuracy is not None
        and accuracy >= target
        and extra_accuracy is not None
        and extra_accuracy >= target
        and len(extra_preds) > 0
    )
    severity_passed = bool(
        rho is not None
        and rho >= rho_min
        and range_over_sigma is not None
        and range_over_sigma >= ros_min
    )
    return {
        "classification": {
            "applicable": True,
            "train_runs": len(train),
            "test_runs": len(test),
            "extrapolation_test_runs": len(extra_preds),
            "train_condition": {"rate_deg_s_lte": train_rate_max, "wind_m_s_lte": train_wind_max},
            "extrapolation_condition": {"rate_deg_s_gte": extra_rate_min, "wind_m_s_gte": extra_wind_min},
            "accuracy": accuracy,
            "extrapolation_accuracy": extra_accuracy,
            "target_accuracy": target,
            "passed": classification_passed,
            "predictions": preds,
        },
        "severity": {
            "applicable": True,
            "spearman_rho_predicted_vs_actual_depth": rho,
            "spearman_min": rho_min,
            "depth_dynamic_range_m": depth_range,
            "sigma_m": sigma_m,
            "range_over_sigma": range_over_sigma,
            "range_over_sigma_min": ros_min,
            "passed": severity_passed,
            "role_in_verdict": "load-bearing PASS gate",
        },
        "regression": {
            "applicable": True,
            "features": ["intercept", "commanded_rate_deg_s", "wind_m_s", "achieved_peak_roll_deg"],
            "coefficients": coeff.tolist(),
            "mae_m": mae,
            "depth_range_m": depth_range,
            "mae_over_depth_range": mae_over_range,
            "relative_reference_max": float(config["prediction"]["regression_relative_reference_max"]),
            "role_in_verdict": "reporting only",
        },
    }


def robustness_eval(config: dict[str, Any], runs: list[dict[str, Any]], d_margin_m: float) -> dict[str, Any]:
    scan = [
        r for r in complete_runs(runs)
        if run_role(r) == "scan"
        and math.isclose(float(r.get("point", {}).get("fence_margin_m", 0.0)), float(config["baseline_params"]["FENCE_MARGIN"]))
    ]
    clean_unsafe = [r for r in scan if r.get("label") == "clean_unsafe"]
    depths = [depth_for_label(r) for r in clean_unsafe]
    cells = cell_summary(runs, role="scan")
    base_cells = [c for c in cells if math.isclose(float(c["fence_margin_m"]), float(config["baseline_params"]["FENCE_MARGIN"]))]
    stable_cu = [c for c in base_cells if c["stable"] and c["labels"].get("clean_unsafe", 0) > 0]
    distinct_cu_cells = [
        c for c in base_cells
        if c["labels"].get("clean_unsafe", 0) > 0
    ]
    nontrivial_depth = bool(depths and max(depths) > max(d_margin_m, 1.0))
    passed = bool(len(clean_unsafe) >= 5 and len(distinct_cu_cells) >= 3 and len(stable_cu) >= 2 and nontrivial_depth)
    return {
        "clean_unsafe_count": len(clean_unsafe),
        "clean_unsafe_depth_range_m": [min(depths), max(depths)] if depths else None,
        "clean_unsafe_window_range_s": [
            min(float(r.get("outside_duration_s")) for r in clean_unsafe if r.get("outside_duration_s") is not None),
            max(float(r.get("outside_duration_s")) for r in clean_unsafe if r.get("outside_duration_s") is not None),
        ] if any(r.get("outside_duration_s") is not None for r in clean_unsafe) else None,
        "distinct_clean_unsafe_cells": len(distinct_cu_cells),
        "stable_clean_unsafe_cells": stable_cu,
        "d_margin_m": d_margin_m,
        "nontrivial_depth": nontrivial_depth,
        "passed": passed,
    }


def contract_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    complete = complete_runs(runs)
    violated = [r for r in complete if r.get("label") == "contract_violated"]
    clean_unsafe_contract = [
        r for r in complete
        if r.get("label") == "clean_unsafe" and r.get("preventive_contract", {}).get("violated")
    ]
    return {
        "label_counts": dict(Counter(str(r.get("label")) for r in complete)),
        "contract_violated_count": len(violated),
        "contract_violated_run_ids": [r["run_id"] for r in violated],
        "clean_unsafe_intersects_contract_violated": bool(clean_unsafe_contract),
        "bodyrate_destination_admission_reject_count": sum(int(r.get("destination_admission_reject_count", 0)) for r in complete),
        "preventive_contract_passed": bool(len(violated) == 0 and not clean_unsafe_contract),
    }


def margin_stratification(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[float, list[dict[str, Any]]] = defaultdict(list)
    base_margin = float(config["baseline_params"]["FENCE_MARGIN"])
    for run in complete_runs(runs):
        if run_role(run) == "scan":
            margin = float(run["point"]["fence_margin_m"])
            if math.isclose(margin, base_margin):
                groups[margin].append(run)
        elif run_role(run) == "stratify":
            groups[float(run["point"]["fence_margin_m"])].append(run)
    rows = []
    for margin, rs in sorted(groups.items()):
        depths = [depth_for_label(r) for r in rs]
        labels = Counter(str(r.get("label")) for r in rs)
        rows.append({
            "fence_margin_m": margin,
            "runs": len(rs),
            "labels": dict(labels),
            "clean_unsafe_count": int(labels.get("clean_unsafe", 0)),
            "mean_depth_m": mean(depths),
            "median_depth_m": median(depths),
            "max_depth_m": max(depths) if depths else None,
        })
    means = [r["mean_depth_m"] for r in rows if r["mean_depth_m"] is not None]
    monotone_nonincreasing = all(b <= a + 1.0e-9 for a, b in zip(means, means[1:])) if len(means) >= 2 else False
    return {
        "note": "AC_Fence circle breach check in this SHA compares home distance directly to FENCE_RADIUS; FENCE_MARGIN is not used by check_fence_circle with AVOID_ENABLE=0. The layer is reported empirically, not forced into the verdict.",
        "rows": rows,
        "monotone_nonincreasing_mean_depth": monotone_nonincreasing,
    }


def summarize_fence_frequency(runs: list[dict[str, Any]]) -> dict[str, Any]:
    latencies = [
        float(r["fence_breach_latency_from_cross_s"])
        for r in runs
        if r.get("fence_breach_latency_from_cross_s") is not None
        and -0.1 <= float(r["fence_breach_latency_from_cross_s"]) <= 2.0
    ]
    action_latencies = [
        float(r["action_latency_from_cross_s"])
        for r in runs
        if r.get("action_latency_from_cross_s") is not None
        and -0.1 <= float(r["action_latency_from_cross_s"]) <= 5.0
    ]
    return {
        "source_static": "Copter::three_hz_loop calls fence_check; reactive check is based on actual position.",
        "static_scheduler_hz": 3.0,
        "dataflash_cross_to_fence_event_latency_s": {
            "samples": len(latencies),
            "mean": mean(latencies),
            "min": min(latencies) if latencies else None,
            "max": max(latencies) if latencies else None,
            "p95": percentile(latencies, 0.95),
        },
        "dataflash_cross_to_action_latency_s": {
            "samples": len(action_latencies),
            "mean": mean(action_latencies),
            "min": min(action_latencies) if action_latencies else None,
            "max": max(action_latencies) if action_latencies else None,
            "p95": percentile(action_latencies, 0.95),
        },
    }


def write_premise_record(config: dict[str, Any], runs: list[dict[str, Any]], d_margin_m: float) -> dict[str, Any]:
    payload = {
        "scenario_id": "geofence_bodyrate",
        "version": "v1",
        "written_at_utc": utc_now(),
        "firmware_anchor": firmware_actual(config),
        "interface_path": "GUIDED + SET_ATTITUDE_TARGET body roll-rate with attitude quaternion ignored",
        "premise_checks": premise_checks(config, runs, d_margin_m),
        "premise_runs": [r for r in runs if run_role(r) == "premise"],
    }
    write_json(PREREG_ROOT / "geofence_bodyrate_premise.json", payload)
    return payload


def write_preregister(config: dict[str, Any], runs: list[dict[str, Any]], env: dict[str, Any]) -> dict[str, Any]:
    noise = noise_summary(runs)
    payload = {
        "scenario_id": "geofence_bodyrate",
        "phase": "B_dynamic",
        "version": "v1",
        "status": "preregistered_after_premise_and_noise_before_grid",
        "written_at_utc": utc_now(),
        "firmware_anchor": firmware_actual(config),
        "environment": env,
        "interface_path": {
            "selected": "GUIDED + SET_ATTITUDE_TARGET body roll-rate",
            "type_mask": BODYRATE_TYPE_MASK,
            "availability": "supported MAVLink; no ACRO fallback used unless this path fails",
            "source_route": config["firmware_anchor"]["source_guided_bodyrate_route"],
        },
        "decision_block": {
            "PASS": "premises satisfied; robust clean_unsafe body-rate region; zero preventive contract violations and no destination-admission rejects on body-rate path; classification and severity prediction gates pass",
            "FAIL": "premises satisfied but PASS criteria fail: C-ABSENT if no robust clean_unsafe, or contract-not-clean if unsafe requires preventive/failsafe opposition",
            "INCONCLUSIVE": "premise failure: body-rate path not faithfully applied, fence action absent, SITL/logging fidelity insufficient, or noise/altitude contamination prevents adjudication",
        },
        "fixed_P": {
            "baseline_params": config["baseline_params"],
            "FENCE_RADIUS": config["baseline_params"]["FENCE_RADIUS"],
            "FENCE_ACTION": config["baseline_params"]["FENCE_ACTION"],
            "FENCE_MARGIN_baseline": config["baseline_params"]["FENCE_MARGIN"],
            "AVOID_ENABLE": 0,
        },
        "M_axis": {
            "commanded_peak_body_roll_rates_deg_s": config["sweep"]["scan_rates_deg_s"],
            "schedule": {
                "ramp_s": config["command"]["rate_ramp_s"],
                "pulse_s": config["command"]["rate_pulse_s"],
                "then": "zero body roll-rate while holding the resulting guided angle target until action or stream limit",
            },
            "input_applied_gate": config["oracle"]["rate_tracking_ratio_bounds"],
        },
        "E_axis": {
            "steady_wind_m_s": config["sweep"]["scan_winds_m_s"],
            "wind_direction_deg": config["baseline_params"]["SIM_WIND_DIR"],
            "turbulence": config["sweep"]["noise_turbulence"],
        },
        "noise_and_labeling": {
            "sigma_m": noise["sigma_m"],
            "d_margin_m": noise["d_margin_m"],
            "d_margin_rule": "3*sigma of fixed body-rate/wind condition max outside depth",
            "ambiguous_band_m": [0.0, noise["d_margin_m"]],
            "clean_unsafe_rule": "max outside depth > d_margin and Oracle-B clean",
            "mae_bound_is_reporting_only": True,
        },
        "prediction_gates": {
            "classification_accuracy_min": config["prediction"]["target_classification_accuracy"],
            "classification_split": {
                "train": {
                    "rate_deg_s_lte": config["prediction"]["train_rate_max_deg_s"],
                    "wind_m_s_lte": config["prediction"]["train_wind_max_m_s"],
                },
                "extrapolation_test": {
                    "rate_deg_s_gte": config["prediction"]["extrapolate_rate_min_deg_s"],
                    "wind_m_s_gte": config["prediction"]["extrapolate_wind_min_m_s"],
                },
            },
            "severity_spearman_min": config["prediction"]["severity_spearman_min"],
            "range_over_sigma_min": config["prediction"]["severity_range_over_sigma_min"],
        },
        "P_layer_note": "FENCE_MARGIN is swept as requested, but AC_Fence::check_fence_circle in this SHA compares distance to FENCE_RADIUS and does not use margin with AVOID_ENABLE=0. The report will present empirical monotonicity honestly.",
    }
    write_json(PREREG_ROOT / "geofence_bodyrate_prereg.json", payload)
    return payload


def make_plots(config: dict[str, Any], runs: list[dict[str, Any]], result: dict[str, Any]) -> dict[str, str]:
    fig_dir = RESULT_ROOT / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    paths: dict[str, str] = {}

    scan = [
        r for r in complete_runs(runs)
        if run_role(r) == "scan"
        and math.isclose(float(r["point"]["fence_margin_m"]), float(config["baseline_params"]["FENCE_MARGIN"]))
    ]
    color = {"clean_safe": "#2ca02c", "clean_unsafe": "#d62728", "contract_violated": "#9467bd", "ambiguous": "#7f7f7f", "blocked": "#111111"}
    fig, ax = plt.subplots(figsize=(7.4, 4.8))
    for label in sorted({str(r.get("label")) for r in scan}):
        rows = [r for r in scan if str(r.get("label")) == label]
        ax.scatter(
            [float(r["point"]["rate_deg_s"]) for r in rows],
            [float(r["point"]["wind_m_s"]) for r in rows],
            s=[38 + 7 * min(depth_for_label(r), 20.0) for r in rows],
            color=color.get(label, "#1f77b4"),
            alpha=0.82,
            label=label,
            edgecolor="black",
            linewidth=0.25,
        )
    ax.set_xlabel("Commanded peak body roll-rate (deg/s)")
    ax.set_ylabel("Steady wind speed (m/s)")
    ax.set_title("M x E labels, baseline FENCE_MARGIN")
    ax.legend(fontsize=8)
    fig.tight_layout()
    p = fig_dir / "grid_labels.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    paths["grid_labels"] = str(p)

    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    winds = sorted({float(r["point"]["wind_m_s"]) for r in scan})
    for wind in winds:
        rows = sorted([r for r in scan if math.isclose(float(r["point"]["wind_m_s"]), wind)], key=lambda r: (float(r["point"]["rate_deg_s"]), int(r["point"]["rep"])))
        ax.plot(
            [float(r["point"]["rate_deg_s"]) for r in rows],
            [depth_for_label(r) for r in rows],
            marker="o",
            linestyle="-",
            label=f"wind {wind:g} m/s",
            alpha=0.85,
        )
    ax.axhline(float(result["noise"]["d_margin_m"]), color="black", linewidth=1.0, linestyle="--", label="d_margin")
    ax.fill_between(
        [min(config["sweep"]["scan_rates_deg_s"]), max(config["sweep"]["scan_rates_deg_s"])],
        0.0,
        float(result["noise"]["d_margin_m"]),
        color="#dddddd",
        alpha=0.35,
        label="ambiguous band",
    )
    ax.set_xlabel("Commanded peak body roll-rate (deg/s)")
    ax.set_ylabel("Max outside depth (m)")
    ax.set_title("Outside depth vs body-rate")
    ax.legend(fontsize=8)
    fig.tight_layout()
    p = fig_dir / "depth_vs_rate_by_wind.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    paths["depth_vs_rate_by_wind"] = str(p)

    layers = result["p_stratification"]["rows"]
    if layers:
        fig, ax = plt.subplots(figsize=(6.8, 4.2))
        ax.plot(
            [row["fence_margin_m"] for row in layers],
            [row["mean_depth_m"] or 0.0 for row in layers],
            marker="o",
            label="mean depth",
        )
        ax.plot(
            [row["fence_margin_m"] for row in layers],
            [row["clean_unsafe_count"] for row in layers],
            marker="s",
            label="clean_unsafe count",
        )
        ax.set_xlabel("FENCE_MARGIN (m)")
        ax.set_title("FENCE_MARGIN layer")
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = fig_dir / "fence_margin_layer.png"
        fig.savefig(p, dpi=160)
        plt.close(fig)
        paths["fence_margin_layer"] = str(p)

    witness = next((r for r in scan if r.get("label") == "clean_unsafe"), None)
    if witness and witness.get("csv_path"):
        rows = []
        with Path(str(witness["csv_path"])).open(encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if row.get("type") in {"XKF1", "POS"} and row.get("distance_m") not in (None, ""):
                    rows.append((float(row["time_s"]), float(row["distance_m"])))
        if rows:
            fig, ax = plt.subplots(figsize=(7.4, 4.2))
            radius = float(witness["fence_radius_m"])
            ax.plot([t for t, _ in rows], [d - radius for _, d in rows], linewidth=1.3)
            ax.axhline(0.0, color="black", linewidth=1.0)
            for key, label in (("crossing", "cross"), ("action_time_s", "FENCE_ACTION"), ("reentry_time_s", "inside")):
                if key == "crossing":
                    t = witness.get("crossing", {}).get("time_s")
                else:
                    t = witness.get(key)
                if t is not None:
                    ax.axvline(float(t), linestyle="--", linewidth=1.0, label=label)
            ax.set_xlabel("DataFlash time (s)")
            ax.set_ylabel("Outside depth (m)")
            ax.set_title(f"Representative clean window: {witness['run_id']}")
            ax.legend(fontsize=8)
            fig.tight_layout()
            p = fig_dir / "representative_trajectory.png"
            fig.savefig(p, dpi=160)
            plt.close(fig)
            paths["representative_trajectory"] = str(p)

    return paths


def summarize(config: dict[str, Any], runs: list[dict[str, Any]], prereg_path: Path) -> dict[str, Any]:
    noise = noise_summary(runs)
    d_margin = float(noise["d_margin_m"])
    attach_labels(runs, d_margin)
    premises = premise_checks(config, runs, d_margin)
    cells = cell_summary(runs, role="scan")
    p_layers = margin_stratification(config, runs)
    contracts = contract_summary(runs)
    robust = robustness_eval(config, runs, d_margin)
    pred = prediction_eval(config, runs, d_margin, float(noise["sigma_m"]))
    frequency = summarize_fence_frequency(complete_runs(runs))

    criteria = {
        "premise": bool(premises["all_ok"]),
        "robust_clean_unsafe": bool(robust["passed"]),
        "zero_preventive_contract_violations": bool(contracts["preventive_contract_passed"] and contracts["bodyrate_destination_admission_reject_count"] == 0),
        "prediction_gates": bool(pred.get("classification", {}).get("passed") and pred.get("severity", {}).get("passed")),
    }
    if not criteria["premise"]:
        verdict = "INCONCLUSIVE"
        matrix = "INCONCLUSIVE"
        reason = "one or more Phase-0 body-rate/fence fidelity premises failed"
    elif all(criteria.values()):
        verdict = "PASS"
        matrix = "CONFIRMED-GAP(red)"
        reason = "body-rate path produced robust clean_unsafe outside-depth witnesses with clean contract accounting and prediction gates passed"
    elif robust["clean_unsafe_count"] == 0:
        verdict = "FAIL"
        matrix = "C-ABSENT"
        reason = "premises held but no robust clean_unsafe body-rate witness exceeded d_margin"
    elif not criteria["zero_preventive_contract_violations"]:
        verdict = "FAIL"
        matrix = "contract-not-clean"
        reason = "unsafe outcomes intersected preventive contract violations or destination rejects"
    else:
        verdict = "FAIL"
        matrix = "C-ABSENT / contract-not-clean"
        reason = "premises held but one or more preregistered PASS gates failed"

    result: dict[str, Any] = {
        "scenario_id": "geofence_bodyrate",
        "phase": "B_dynamic",
        "version": "v1",
        "generated_at_utc": utc_now(),
        "verdict": verdict,
        "matrix": matrix,
        "verdict_reason": reason,
        "criteria": criteria,
        "firmware_anchor": firmware_actual(config),
        "config_path": str(PLANC_ROOT / "config" / "geofence_bodyrate_phaseB_v1_config.yaml"),
        "prereg_path": str(prereg_path),
        "premises": premises,
        "noise": noise,
        "fence_check_frequency": frequency,
        "scan_cells": cells,
        "robustness": robust,
        "contract_summary": contracts,
        "prediction": pred,
        "p_stratification": p_layers,
        "mavlink_message_summary": {
            "SET_ATTITUDE_TARGET": sum(int(r.get("online_observation", {}).get("mavlink_message_summary", {}).get("SET_ATTITUDE_TARGET", 0)) for r in runs),
            "SET_POSITION_TARGET_GLOBAL_INT": 0,
            "RC_CHANNELS_OVERRIDE": 0,
            "GUIDED_MODE_SET": sum(int(r.get("online_observation", {}).get("mavlink_message_summary", {}).get("GUIDED_MODE_SET", 0)) for r in runs),
        },
        "runs": runs,
    }
    result["figures"] = make_plots(config, runs, result)
    return result


def report_lines(config: dict[str, Any], result: dict[str, Any]) -> list[str]:
    lines = []
    lines.append(f"VERDICT: {result['verdict']}")
    lines.append(f"MATRIX: {result['matrix']}")
    lines.append("")
    lines.append("# geofence_bodyrate Phase-B v1 Report")
    lines.append("")
    lines.append(f"Reason: {result['verdict_reason']}.")
    fw = result["firmware_anchor"]
    lines.append(f"Firmware: actual `{fw.get('actual_describe')}` / `{fw.get('actual_sha')}`, expected `{fw.get('expected_tag')}` / `{fw.get('expected_sha')}`. SITL binary `{fw.get('binary')}`.")
    lines.append("Interface path: supported MAVLink `GUIDED + SET_ATTITUDE_TARGET` with `ATTITUDE_IGNORE`, body roll-rate field active, thrust field `0.5` as zero climb-rate. No ACRO fallback and no RC override were used.")
    lines.append("")
    lines.append("## Four Criteria")
    lines.append("")
    lines.append("| criterion | passed | evidence |")
    lines.append("|---|---:|---|")
    robust = result["robustness"]
    contracts = result["contract_summary"]
    cls = result["prediction"]["classification"]
    sev = result["prediction"]["severity"]
    lines.append(f"| premise | {result['criteria']['premise']} | all Phase-0 checks `{result['premises']['all_ok']}` |")
    lines.append(f"| robust clean_unsafe | {result['criteria']['robust_clean_unsafe']} | count `{robust['clean_unsafe_count']}`, depth range `{robust['clean_unsafe_depth_range_m']}`, window range `{robust['clean_unsafe_window_range_s']}` |")
    lines.append(f"| zero preventive violations / PGFUZZ invisible | {result['criteria']['zero_preventive_contract_violations']} | contract_violated `{contracts['contract_violated_count']}`, destination rejects `{contracts['bodyrate_destination_admission_reject_count']}` |")
    lines.append(f"| prediction gates | {result['criteria']['prediction_gates']} | classification `{fmt(cls.get('accuracy'))}`, extrapolation `{fmt(cls.get('extrapolation_accuracy'))}`, Spearman `{fmt(sev.get('spearman_rho_predicted_vs_actual_depth'), 3)}`, range/sigma `{fmt(sev.get('range_over_sigma'), 1)}` |")
    lines.append("")
    lines.append("## Premises")
    lines.append("")
    lines.append("| premise | ok | evidence |")
    lines.append("|---|---:|---|")
    for key, val in result["premises"].items():
        if key == "all_ok":
            continue
        evidence = []
        if "tracking" in val and val.get("tracking"):
            tr = val["tracking"]
            evidence.append(f"rate actual/cmd {fmt(tr.get('actual_peak_aligned_deg_s'))}/{fmt(tr.get('commanded_peak_abs_deg_s'))} deg/s")
        if "achieved_cross_speed_m_s" in val:
            evidence.append(f"cross speed {fmt(val.get('achieved_cross_speed_m_s'))} m/s")
        if "action_mode" in val:
            evidence.append(f"action {val.get('action_mode')} at {fmt(val.get('action_time_s'))} s")
        if "destination_reject_count" in val:
            evidence.append(f"destination rejects {val.get('destination_reject_count')}")
        if "min_estimated_stream_hz_sim" in val:
            evidence.append(f"stream {fmt(val.get('min_estimated_stream_hz_sim'))} Hz sim")
        lines.append(f"| `{key}` | {val.get('ok')} | {'; '.join(evidence) or 'see verdict.json'} |")
    lines.append("")
    lines.append("## Noise And Labels")
    lines.append("")
    n = result["noise"]
    lines.append(f"Noise fixed point: sigma `{fmt(n['sigma_m'])}` m over `{n['n']}` runs; `d_margin = 3*sigma = {fmt(n['d_margin_m'])}` m. Labels use `clean_unsafe` only for outside depth `> d_margin`; depths in `(0, d_margin]` are ambiguous.")
    lines.append("")
    lines.append("## M x E Grid")
    lines.append("")
    lines.append("| rate deg/s | wind m/s | margin m | runs | stable | labels | mean depth m | mean cross speed m/s | mean peak roll deg |")
    lines.append("|---:|---:|---:|---:|---:|---|---:|---:|---:|")
    for cell in result["scan_cells"]:
        if not math.isclose(float(cell["fence_margin_m"]), float(config["baseline_params"]["FENCE_MARGIN"])):
            continue
        lines.append(f"| {fmt(cell['rate_deg_s'], 0)} | {fmt(cell['wind_m_s'], 0)} | {fmt(cell['fence_margin_m'], 1)} | {cell['runs']} | {cell['stable']} | `{cell['labels']}` | {fmt(cell.get('mean_depth_m'))} | {fmt(cell.get('mean_cross_speed_m_s'))} | {fmt(cell.get('mean_peak_roll_deg'))} |")
    lines.append("")
    lines.append("## Prediction")
    lines.append("")
    reg = result["prediction"]["regression"]
    lines.append(f"Classification split: train low conditions `{cls.get('train_condition')}`, extrapolate high conditions `{cls.get('extrapolation_condition')}`. Accuracy `{fmt(cls.get('accuracy'))}`, extrapolation accuracy `{fmt(cls.get('extrapolation_accuracy'))}`, target `{fmt(cls.get('target_accuracy'))}`, passed `{cls.get('passed')}`.")
    lines.append(f"Severity gate: Spearman predicted-vs-actual depth `{fmt(sev.get('spearman_rho_predicted_vs_actual_depth'), 3)}` vs min `{fmt(sev.get('spearman_min'), 2)}`; depth range/sigma `{fmt(sev.get('range_over_sigma'), 1)}` vs min `{fmt(sev.get('range_over_sigma_min'), 1)}`; passed `{sev.get('passed')}`.")
    lines.append(f"Severity regression is reporting-only: features `{reg.get('features')}`, MAE `{fmt(reg.get('mae_m'))}` m, MAE/range `{fmt(reg.get('mae_over_depth_range'), 3)}`.")
    lines.append("")
    lines.append("## P Layer")
    lines.append("")
    lines.append(result["p_stratification"]["note"])
    lines.append("")
    lines.append("| FENCE_MARGIN m | runs | labels | clean_unsafe count | mean depth m | median depth m | max depth m |")
    lines.append("|---:|---:|---|---:|---:|---:|---:|")
    for row in result["p_stratification"]["rows"]:
        lines.append(f"| {fmt(row['fence_margin_m'], 1)} | {row['runs']} | `{row['labels']}` | {row['clean_unsafe_count']} | {fmt(row.get('mean_depth_m'))} | {fmt(row.get('median_depth_m'))} | {fmt(row.get('max_depth_m'))} |")
    lines.append(f"Observed monotone non-increasing mean depth: `{result['p_stratification']['monotone_nonincreasing_mean_depth']}`.")
    lines.append("")
    lines.append("## Figures")
    lines.append("")
    for name, path in result.get("figures", {}).items():
        rel = Path(path).relative_to(REPO_ROOT)
        lines.append(f"- `{name}`: `{rel}`")
    lines.append("")
    lines.append("## Honest Boundary Checklist")
    lines.append("")
    lines.append("- Availability is `supported MAVLink/GUIDED + SET_ATTITUDE_TARGET body-rate`; no RC override was used.")
    lines.append("- Geofence action is a reactive fallback, not a preventive contract violation; clean windows stop at return/arrest and no mode switch back into GUIDED is used after FENCE_ACTION.")
    lines.append("- The consequence is horizontal outside depth and outside duration, not crash or natural environmental reachability.")
    lines.append("- `FENCE_MARGIN` is reported empirically because the current circle breach source does not use it as an early trigger with avoidance disabled.")
    lines.append("- No claims are made about scaling laws, area laws, or search complexity.")
    return lines


def write_outputs(config: dict[str, Any], result: dict[str, Any]) -> None:
    write_json(RESULT_ROOT / "verdict.json", result)
    write_json(RESULT_ROOT / "result.json", result)
    (RESULT_ROOT / "REPORT.md").write_text("\n".join(report_lines(config, result)) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "geofence_bodyrate_phaseB_v1_config.yaml")
    parser.add_argument("--stage", choices=["preregister", "premise", "noise", "scan", "stratify", "all", "report"], default="all")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    config = load_yaml(args.config)
    RESULT_ROOT.mkdir(parents=True, exist_ok=True)
    PREREG_ROOT.mkdir(parents=True, exist_ok=True)
    partial_path = RESULT_ROOT / "partial.json"
    prereg_path = PREREG_ROOT / "geofence_bodyrate_prereg.json"
    env_path = RESULT_ROOT / "env.json"
    env = probe_environment(config, REPO_ROOT)
    write_env(env, env_path)

    partial = load_json(partial_path, {"runs": []})
    runs = list(partial.get("runs", [])) if args.resume or partial_path.exists() else []

    if args.stage == "preregister":
        write_preregister(config, runs, env)
        print(f"WROTE {prereg_path}", flush=True)
        return

    if args.stage != "report":
        for point in stage_points(config, args.stage):
            runs = run_cached(config, point, partial_path, runs, args.resume)
        write_json(partial_path, {"runs": runs, "updated_at_utc": utc_now()})

    noise = noise_summary(runs)
    d_margin = float(noise["d_margin_m"])
    write_premise_record(config, runs, d_margin)
    if any(run_role(r) == "noise" and not r.get("error") for r in runs):
        write_preregister(config, runs, env)

    result = summarize(config, runs, prereg_path)
    write_outputs(config, result)
    print(f"RESULT {result['verdict']} {RESULT_ROOT / 'verdict.json'}", flush=True)


if __name__ == "__main__":
    main()
