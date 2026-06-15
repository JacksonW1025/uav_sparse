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

import matplotlib.pyplot as plt
import yaml
from pymavlink import mavutil

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))

from env_probe import probe_environment, write_env
from flight import (
    FlightError,
    arm,
    command_takeoff,
    land_and_disarm,
    request_streams,
    send_gcs_heartbeat,
    set_mode,
    wait_altitude,
    wait_position_stable,
)
from oracle import COPTER_MODES, ERROR_SUBSYSTEMS
from param_manager import ParamManager
from sitl_runner import SitlRunner


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_json(path: Path, default: dict[str, Any] | None = None) -> dict[str, Any]:
    if not path.exists():
        return default or {}
    return json.loads(path.read_text(encoding="utf-8"))


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def artifact_stem(config: dict[str, Any]) -> str:
    return str(config["experiment"].get("artifact_stem", "stage0_v2"))


def run_prefix(config: dict[str, Any]) -> str:
    return str(config["experiment"].get("run_prefix", "stage0v2"))


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def percentile(values: list[float], pct: float) -> float | None:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * pct
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return clean[lo]
    return clean[lo] + (clean[hi] - clean[lo]) * (rank - lo)


def q_from_euler(roll_rad: float, pitch_rad: float, yaw_rad: float) -> list[float]:
    cr = math.cos(roll_rad / 2.0)
    sr = math.sin(roll_rad / 2.0)
    cp = math.cos(pitch_rad / 2.0)
    sp = math.sin(pitch_rad / 2.0)
    cy = math.cos(yaw_rad / 2.0)
    sy = math.sin(yaw_rad / 2.0)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def send_attitude_target(
    master,
    *,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    roll_rate_deg_s: float,
    pitch_rate_deg_s: float,
    yaw_rate_deg_s: float,
    thrust: float,
    attitude_ignore: bool = False,
) -> None:
    if attitude_ignore:
        type_mask = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
        q = [0.0, 0.0, 0.0, 0.0]
    else:
        type_mask = 0
        q = q_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    master.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        type_mask,
        q,
        math.radians(roll_rate_deg_s),
        math.radians(pitch_rate_deg_s),
        math.radians(yaw_rate_deg_s),
        float(thrust),
    )


def command_amplitude_deg(config: dict[str, Any], angle_max_cd: float) -> float:
    angle_max_deg = float(angle_max_cd) / 100.0
    cmd = config["command"]
    guard = float(config["oracle"]["command_angle_guard_deg"])
    return max(
        0.0,
        min(
            float(cmd["max_nominal_amplitude_deg"]),
            angle_max_deg * float(cmd["amplitude_fraction_of_angle_max"]),
            angle_max_deg - guard,
        ),
    )


def doublet_profile(config: dict[str, Any], r_deg_s: float, angle_max_cd: float) -> list[dict[str, float]]:
    cmd = config["command"]
    hz = float(config["experiment"]["stream_hz"])
    dt = 1.0 / hz
    amp = command_amplitude_deg(config, angle_max_cd)
    hold_s = float(cmd["hold_s"])
    cycles = int(cmd["cycles"])
    axis = str(cmd.get("axis", "roll"))

    samples: list[dict[str, float]] = []
    t_s = 0.0
    current = 0.0

    def append(value: float, rate: float) -> None:
        nonlocal t_s
        row = {
            "t_s": t_s,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "roll_rate_deg_s": 0.0,
            "pitch_rate_deg_s": 0.0,
        }
        if axis == "pitch":
            row["pitch_deg"] = value
            row["pitch_rate_deg_s"] = rate
        else:
            row["roll_deg"] = value
            row["roll_rate_deg_s"] = rate
        samples.append(row)
        t_s += dt

    def ramp_to(target: float) -> None:
        nonlocal current
        if math.isclose(current, target, abs_tol=1.0e-9):
            append(current, 0.0)
            return
        sign = 1.0 if target > current else -1.0
        rate = sign * abs(float(r_deg_s))
        while (target - current) * sign > 1.0e-9:
            step = min(abs(target - current), abs(float(r_deg_s)) * dt)
            current += sign * step
            append(current, rate)

    def hold(value: float, duration_s: float) -> None:
        nonlocal current
        current = value
        count = max(1, int(round(duration_s * hz)))
        for _ in range(count):
            append(current, 0.0)

    for _ in range(cycles):
        ramp_to(amp)
        hold(amp, hold_s)
        ramp_to(-amp)
        hold(-amp, hold_s)
        neutral_s = float(cmd.get("neutral_between_cycles_s", 0.0))
        if neutral_s > 0:
            ramp_to(0.0)
            hold(0.0, neutral_s)
    ramp_to(0.0)
    hold(0.0, float(config["experiment"].get("post_maneuver_hold_s", 1.0)))
    return samples


def profile_active_bounds(config: dict[str, Any], profile: list[dict[str, float]]) -> tuple[float | None, float | None]:
    angle_threshold = float(config["oracle"]["attitude_active_threshold_deg"])
    rate_threshold = float(config["oracle"]["rate_active_threshold_deg_s"])
    active = [
        row
        for row in profile
        if abs(float(row.get("roll_deg", 0.0))) >= angle_threshold
        or abs(float(row.get("pitch_deg", 0.0))) >= angle_threshold
        or abs(float(row.get("roll_rate_deg_s", 0.0))) >= rate_threshold
        or abs(float(row.get("pitch_rate_deg_s", 0.0))) >= rate_threshold
    ]
    if not active:
        return None, None
    return float(active[0]["t_s"]), float(active[-1]["t_s"])


def command_at(profile: list[dict[str, float]], rel_t_s: float) -> dict[str, float]:
    if not profile:
        return {"roll_deg": 0.0, "pitch_deg": 0.0, "roll_rate_deg_s": 0.0, "pitch_rate_deg_s": 0.0}
    if rel_t_s <= float(profile[0]["t_s"]):
        return dict(profile[0])
    if rel_t_s >= float(profile[-1]["t_s"]):
        return dict(profile[-1])
    lo = 0
    hi = len(profile) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if float(profile[mid]["t_s"]) <= rel_t_s:
            lo = mid
        else:
            hi = mid
    a = profile[lo]
    b = profile[hi]
    span = float(b["t_s"]) - float(a["t_s"])
    frac = 0.0 if span <= 0.0 else (rel_t_s - float(a["t_s"])) / span
    out = {"t_s": rel_t_s}
    for key in ("roll_deg", "pitch_deg", "roll_rate_deg_s", "pitch_rate_deg_s"):
        out[key] = float(a[key]) + (float(b[key]) - float(a[key])) * frac
    return out


def model_path(config: dict[str, Any], model_name: str) -> Path | None:
    spec = config.get("models", {}).get(model_name)
    if not spec:
        return None
    raw = Path(str(spec["json"]))
    if raw.is_absolute():
        return raw
    return REPO_ROOT / raw


def point_config(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    cfg = copy.deepcopy(config)
    model_name = str(point.get("model", config["pressure"].get("default_model", "m100")))
    path = model_path(config, model_name)
    if path is not None:
        cfg["sitl"]["model_json_source"] = str(path)
        spec = config["models"][model_name]
        cfg["experiment"]["model_name"] = model_name
        cfg["experiment"]["model_mass_kg"] = float(spec["mass_kg"])
        cfg["experiment"]["model_mass_multiplier"] = float(spec["mass_multiplier"])
    return cfg


def point_params(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    params = dict(config.get("baseline_params", {}))
    params.update(
        {
            "SIM_SPEEDUP": float(config["experiment"]["speedup"]),
            "ANGLE_MAX": float(point["angle_max_cd"]),
            "SIM_WIND_SPD": float(point.get("wind_m_s", 0.0)),
            "SIM_WIND_TURB": float(point.get("turbulence_m_s", 0.0)),
            "SIM_WIND_DIR": float(point.get("wind_dir_deg", params.get("SIM_WIND_DIR", 270))),
            "AVOID_ENABLE": 0,
        }
    )
    return params


def run_id_for(config: dict[str, Any], point: dict[str, Any]) -> str:
    role = str(point.get("role", "run")).replace("_", "")
    layer = str(point.get("layer", "d")).replace("_", "")
    model = str(point.get("model", "m100"))
    return (
        f"{run_prefix(config)}_{role}_{layer}"
        f"_r{int(round(float(point['r_deg_s']))):04d}"
        f"_w{int(round(float(point.get('wind_m_s', 0.0)))):02d}"
        f"_t{int(round(float(point.get('turbulence_m_s', 0.0)))):02d}"
        f"_a{int(round(float(point['angle_max_cd']))):04d}"
        f"_{model}_s{int(point.get('seed', 0)):02d}"
    )


def write_preregister(config: dict[str, Any], path: Path) -> dict[str, Any]:
    payload = {
        "status": "preregistered",
        "written_at_utc": utc_now(),
        "experiment": config["experiment"],
        "fixed_P": {
            "mode": config["experiment"].get("use_mode", "GUIDED_NOGPS"),
            "baseline_params": config["baseline_params"],
            "angle_max_default_cd": config["baseline_params"]["ANGLE_MAX"],
            "angle_max_legal_range_cd": config["param_metadata"]["angle_max_legal_range_cd"],
            "avoidance_disabled": config["baseline_params"].get("AVOID_ENABLE") == 0,
        },
        "input_path": {
            "mavlink_message": "SET_ATTITUDE_TARGET",
            "mode_sequence": "GUIDED takeoff, then GUIDED_NOGPS attitude/rate stream",
            "r_definition": "r_deg_s is the signed ramp slope and angular-rate feed-forward amplitude used when the bounded attitude doublet reverses between +/- amplitude.",
            "command": config["command"],
        },
        "oracle_A": {
            "divergence": f"|actual-command attitude error| > {config['oracle']['divergence_error_deg']} deg for {config['oracle']['divergence_duration_s']} s",
            "consequence": "SITL crash/ground hit before cleanup LAND",
        },
        "oracle_B": {
            "preventive_failsafe_subsystems": config["oracle"]["preventive_failsafe_subsystems"],
            "preventive_text_markers": config["oracle"]["preventive_text_markers"],
            "command_angle_limit": "max command tilt touches ANGLE_MAX within configured tolerance",
        },
        "seed_group": config["pressure"]["seeds"],
        "seed_limitation": config["param_metadata"]["turbulence_seed_note"],
        "p1_fidelity": config["fidelity"],
        "p2_pressure": config["pressure"],
        "p3_boundary_grid": config["boundary_grid"],
        "decision_table": {
            "GO": "robust nontrivial OverDraw: A inside and B outside for legal commands, stable across seeds",
            "NO_GO_contract_bug_side": "all A points are also B inside",
            "NO_GO_mechanism_inconclusive": "configured legal pressure knobs are exhausted without entering A",
            "NEEDS_TUNING": "path fidelity or consequence path is blocked by a tunable issue",
        },
    }
    write_json(path, payload)
    return payload


def _time_s(data: dict[str, Any]) -> float | None:
    if "TimeUS" in data:
        return float(data["TimeUS"]) / 1.0e6
    if "TimeMS" in data:
        return float(data["TimeMS"]) / 1000.0
    return None


def _mode_name(data: dict[str, Any]) -> str:
    mode = data.get("ModeNum", data.get("Mode"))
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


def _subsystem_name(data: dict[str, Any]) -> str:
    try:
        return ERROR_SUBSYSTEMS.get(int(data.get("Subsys")), str(data.get("Subsys")))
    except Exception:
        return str(data.get("Subsys"))


def sustained_exceed(rows: list[tuple[float, float]], threshold: float, duration_s: float) -> dict[str, Any]:
    over_start: float | None = None
    best = 0.0
    intervals: list[dict[str, float]] = []
    previous_t: float | None = None
    for t_s, value in rows:
        if value > threshold:
            if over_start is None:
                over_start = t_s
        else:
            if over_start is not None:
                end_t = previous_t if previous_t is not None else t_s
                elapsed = max(0.0, end_t - over_start)
                best = max(best, elapsed)
                if elapsed >= duration_s:
                    intervals.append({"start_s": over_start, "end_s": end_t, "duration_s": elapsed})
                over_start = None
        previous_t = t_s
    if over_start is not None and previous_t is not None:
        elapsed = max(0.0, previous_t - over_start)
        best = max(best, elapsed)
        if elapsed >= duration_s:
            intervals.append({"start_s": over_start, "end_s": previous_t, "duration_s": elapsed})
    return {"ok": bool(intervals), "max_duration_s": best, "intervals": intervals}


def nearest_rows(source: list[dict[str, Any]], targets: list[dict[str, Any]], max_dt_s: float) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    if not source or not targets:
        return []
    out: list[tuple[dict[str, Any], dict[str, Any]]] = []
    idx = 0
    for target in targets:
        t = float(target["time_s"])
        while idx + 1 < len(source) and abs(float(source[idx + 1]["time_s"]) - t) <= abs(float(source[idx]["time_s"]) - t):
            idx += 1
        if abs(float(source[idx]["time_s"]) - t) <= max_dt_s:
            out.append((source[idx], target))
    return out


def extract_window(rows: list[dict[str, Any]], lo: float | None, hi: float | None) -> list[dict[str, Any]]:
    if lo is None or hi is None:
        return []
    return [row for row in rows if lo <= float(row["time_s"]) <= hi]


def parse_stage0_dataflash(
    *,
    bin_path: Path,
    csv_path: Path,
    oracle_path: Path,
    config: dict[str, Any],
    point: dict[str, Any],
    command_profile: list[dict[str, float]],
) -> dict[str, Any]:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    msg_types = ["ATT", "RATE", "GUIA", "MOTB", "RCOU", "MODE", "ERR", "EV", "MSG", "POS", "CTUN"]
    att_rows: list[dict[str, Any]] = []
    rate_rows: list[dict[str, Any]] = []
    guia_rows: list[dict[str, Any]] = []
    motb_rows: list[dict[str, Any]] = []
    rcou_rows: list[dict[str, Any]] = []
    pos_rows: list[dict[str, Any]] = []
    modes: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    csv_rows: list[dict[str, Any]] = []

    while True:
        msg = mlog.recv_match(type=msg_types, blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        data = msg.to_dict()
        t_s = _time_s(data)
        if t_s is None:
            continue
        typ = msg.get_type()
        row = {"time_s": t_s, "type": typ}
        row.update({k: v for k, v in data.items() if k not in {"mavpackettype", "TimeUS", "TimeMS"}})
        csv_rows.append(row)
        if typ == "ATT":
            att_rows.append(row)
        elif typ == "RATE":
            rate_rows.append(row)
        elif typ == "GUIA":
            guia_rows.append(row)
        elif typ == "MOTB":
            motb_rows.append(row)
        elif typ == "RCOU":
            rcou_rows.append(row)
        elif typ == "POS":
            pos_rows.append(row)
        elif typ == "MODE":
            row["mode_name"] = _mode_name(data)
            modes.append(row)
        elif typ == "ERR":
            row["subsystem_name"] = _subsystem_name(data)
            errors.append(row)
        elif typ == "MSG":
            messages.append(row)
        elif typ == "EV":
            events.append(row)

    active_rate = float(config["oracle"]["rate_active_threshold_deg_s"])
    active_guia = [
        row
        for row in guia_rows
        if abs(float(row.get("RollRt", 0.0))) >= active_rate
        or abs(float(row.get("PitchRt", 0.0))) >= active_rate
    ]
    profile_active_start, profile_active_end = profile_active_bounds(config, command_profile)
    if active_guia:
        active_start = min(float(row["time_s"]) for row in active_guia)
        if profile_active_start is not None and profile_active_end is not None:
            active_start = active_start - profile_active_start
            active_end = active_start + profile_active_end
        else:
            active_end = max(float(row["time_s"]) for row in active_guia)
    else:
        active_angle = float(config["oracle"]["attitude_active_threshold_deg"])
        active_att = [
            row
            for row in att_rows
            if abs(float(row.get("DesRoll", 0.0))) >= active_angle
            or abs(float(row.get("DesPitch", 0.0))) >= active_angle
        ]
        active_start = min((float(row["time_s"]) for row in active_att), default=None)
        active_end = max((float(row["time_s"]) for row in active_att), default=None)

    att_w = extract_window(att_rows, active_start, active_end)
    rate_w = extract_window(rate_rows, active_start, active_end)
    guia_w = extract_window(guia_rows, active_start, active_end)
    motb_w = extract_window(motb_rows, active_start, active_end)
    rcou_w = extract_window(rcou_rows, active_start, active_end)

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if csv_rows:
        fields = ["time_s", "type"]
        extra = sorted({k for row in csv_rows for k in row if k not in fields})
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields + extra)
            writer.writeheader()
            for row in csv_rows:
                writer.writerow(row)

    angle_max_deg = float(point["angle_max_cd"]) / 100.0
    tolerance = float(config["oracle"]["command_angle_limit_tolerance_deg"])
    command_angles = [
        math.hypot(float(row.get("roll_deg", 0.0)), float(row.get("pitch_deg", 0.0)))
        for row in command_profile
    ]
    max_command_angle = max(command_angles, default=None)
    command_touched_limit = (
        max_command_angle is not None and max_command_angle >= angle_max_deg - tolerance
    )
    command_exceeded_limit = (
        max_command_angle is not None and max_command_angle > angle_max_deg + tolerance
    )

    att_errors = []
    for row in att_w:
        rel_t = float(row["time_s"]) - active_start if active_start is not None else 0.0
        cmd = command_at(command_profile, rel_t)
        att_errors.append(
            (
                float(row["time_s"]),
                math.hypot(
                    float(row.get("Roll", 0.0)) - float(cmd.get("roll_deg", 0.0)),
                    float(row.get("Pitch", 0.0)) - float(cmd.get("pitch_deg", 0.0)),
                ),
            )
        )
    max_error = max((value for _, value in att_errors), default=None)
    divergence = sustained_exceed(
        att_errors,
        float(config["oracle"]["divergence_error_deg"]),
        float(config["oracle"]["divergence_duration_s"]),
    )

    cleanup_land_times = [float(row["time_s"]) for row in modes if row.get("mode_name") == "LAND" and active_start is not None and float(row["time_s"]) >= active_start]
    cleanup_or_mode_land_s = min(cleanup_land_times, default=None)
    consequence_cutoff = cleanup_or_mode_land_s if cleanup_or_mode_land_s is not None else active_end
    crash_markers = []
    for row in messages:
        t_s = float(row["time_s"])
        text = str(row.get("Message", ""))
        low = text.lower()
        if active_start is not None and t_s >= active_start and (consequence_cutoff is None or t_s <= consequence_cutoff):
            if "sim hit ground" in low or "crash" in low:
                crash_markers.append({"time_s": t_s, "message": text})
    low_alt_markers = []
    if active_start is not None:
        for row in pos_rows:
            t_s = float(row["time_s"])
            if t_s < active_start or (consequence_cutoff is not None and t_s > consequence_cutoff):
                continue
            alt = row.get("RelHomeAlt")
            if alt is None:
                continue
            # RelHomeAlt is logged in meters on this ArduCopter version.
            try:
                rel_alt_m = float(alt)
            except Exception:
                continue
            if rel_alt_m < 0.5:
                low_alt_markers.append({"time_s": t_s, "rel_home_alt_m": rel_alt_m})
                break

    preventive_subsystems = set(config["oracle"]["preventive_failsafe_subsystems"])
    fail_events = []
    for row in errors:
        t_s = float(row["time_s"])
        if active_start is None or t_s < active_start:
            continue
        if active_end is not None and t_s > active_end:
            continue
        if row.get("subsystem_name") in preventive_subsystems:
            fail_events.append(row)
    text_markers = tuple(str(v).lower() for v in config["oracle"]["preventive_text_markers"])
    fail_texts = []
    for row in messages:
        t_s = float(row["time_s"])
        if active_start is None or t_s < active_start:
            continue
        if active_end is not None and t_s > active_end:
            continue
        text = str(row.get("Message", ""))
        low = text.lower()
        if any(marker in low for marker in text_markers):
            fail_texts.append({"time_s": t_s, "message": text})
    mode_switches = []
    for row in modes:
        t_s = float(row["time_s"])
        if active_start is None or t_s < active_start:
            continue
        if active_end is not None and t_s > active_end:
            continue
        mode = str(row.get("mode_name"))
        if mode not in {"GUIDED", "GUIDED_NOGPS"}:
            mode_switches.append(row)

    command_rate_values = []
    rdes_values = []
    ractual_values = []
    for rate in rate_w:
        if active_start is None:
            continue
        cmd = command_at(command_profile, float(rate["time_s"]) - active_start)
        command_rate_values.append(abs(float(cmd.get("roll_rate_deg_s", 0.0))) + abs(float(cmd.get("pitch_rate_deg_s", 0.0))))
        rdes_values.append(abs(float(rate.get("RDes", 0.0))) + abs(float(rate.get("PDes", 0.0))))
        ractual_values.append(abs(float(rate.get("R", 0.0))) + abs(float(rate.get("P", 0.0))))
    matched_rate_count = len(command_rate_values)
    command_rate_p90 = percentile(command_rate_values, 0.90)
    rdes_p90 = percentile(rdes_values, 0.90)
    ractual_p90 = percentile(ractual_values, 0.90)
    pass_through_ratio = None if not command_rate_p90 else (rdes_p90 or 0.0) / command_rate_p90
    achieved_ratio = None if not command_rate_p90 else (ractual_p90 or 0.0) / command_rate_p90

    command_angle_values = []
    des_angle_values = []
    for att in att_w:
        if active_start is None:
            continue
        cmd = command_at(command_profile, float(att["time_s"]) - active_start)
        command_angle_values.append(math.hypot(float(cmd.get("roll_deg", 0.0)), float(cmd.get("pitch_deg", 0.0))))
        des_angle_values.append(math.hypot(float(att.get("DesRoll", 0.0)), float(att.get("DesPitch", 0.0))))
    command_angle_p90 = percentile(command_angle_values, 0.90)
    des_angle_p90 = percentile(des_angle_values, 0.90)
    angle_pass_through_ratio = None if not command_angle_p90 else (des_angle_p90 or 0.0) / command_angle_p90

    thr_margins = []
    throttle_near = []
    thlimit_values = []
    for row in motb_w:
        if "ThLimit" in row:
            thlimit_values.append(float(row["ThLimit"]))
        try:
            thr_out = float(row["ThrOut"])
            thr_av = float(row["ThrAvMx"])
        except Exception:
            continue
        thr_margins.append(thr_av - thr_out)
        throttle_near.append(thr_av > 0.0 and thr_out >= 0.92 * thr_av)

    pwm_values: list[float] = []
    pwm_near: list[bool] = []
    for row in rcou_w:
        channels = [float(v) for k, v in row.items() if k.startswith("C") and isinstance(v, (float, int))]
        if not channels:
            continue
        pwm_values.extend(channels)
        pwm_near.append(max(channels) >= 1950.0 or min(channels) <= 1050.0)

    rate_out_values = [
        abs(float(row.get("ROut", 0.0))) + abs(float(row.get("POut", 0.0)))
        for row in rate_w
    ]
    aout_slew_values = [abs(float(row.get("AOutSlew", 0.0))) for row in rate_w if "AOutSlew" in row]

    authority = {
        "motb_thlimit_min": min(thlimit_values, default=None),
        "motb_thlimit_max": max(thlimit_values, default=None),
        "motb_thlimit_nonzero_fraction": None if not thlimit_values else statistics.fmean(1.0 if v != 0.0 else 0.0 for v in thlimit_values),
        "throttle_margin_min": min(thr_margins, default=None),
        "throttle_near_limit_fraction": None if not throttle_near else statistics.fmean(1.0 if v else 0.0 for v in throttle_near),
        "rco_pwm_min": min(pwm_values, default=None),
        "rco_pwm_max": max(pwm_values, default=None),
        "rco_pwm_near_limit_fraction": None if not pwm_near else statistics.fmean(1.0 if v else 0.0 for v in pwm_near),
        "rate_out_abs_p95": percentile(rate_out_values, 0.95),
        "aout_slew_p95": percentile(aout_slew_values, 0.95),
        "aout_slew_max": max(aout_slew_values, default=None),
    }
    authority["authority_limited"] = bool(
        (authority["throttle_near_limit_fraction"] is not None and authority["throttle_near_limit_fraction"] >= 0.05)
        or (authority["rco_pwm_near_limit_fraction"] is not None and authority["rco_pwm_near_limit_fraction"] >= 0.05)
    )

    consequence_crash = bool(crash_markers or low_alt_markers)
    a_inside = bool(divergence["ok"] or consequence_crash)
    preventive = bool(fail_events or fail_texts or mode_switches)
    b_inside = bool(preventive or command_touched_limit or command_exceeded_limit)
    if a_inside and not b_inside:
        label = "overdraw"
    elif a_inside and b_inside:
        label = "bug_side"
    else:
        label = "safe"

    result = {
        "bin_path": str(bin_path),
        "csv_path": str(csv_path),
        "oracle_path": str(oracle_path),
        "active_window_s": {"start": active_start, "end": active_end},
        "samples": {
            "att": len(att_rows),
            "rate": len(rate_rows),
            "guia": len(guia_rows),
            "motb": len(motb_rows),
            "rcou": len(rcou_rows),
        },
        "command": {
            "angle_max_cd": float(point["angle_max_cd"]),
            "angle_max_deg": angle_max_deg,
            "max_command_angle_deg": max_command_angle,
            "command_touched_angle_max": command_touched_limit,
            "command_exceeded_angle_max": command_exceeded_limit,
            "command_rate_p90_deg_s": command_rate_p90,
        },
        "path_fidelity": {
            "matched_rate_samples": matched_rate_count,
            "rate_command_p90_deg_s": command_rate_p90,
            "rate_desired_p90_deg_s": rdes_p90,
            "rate_actual_p90_deg_s": ractual_p90,
            "rate_desired_over_command": pass_through_ratio,
            "rate_actual_over_command": achieved_ratio,
            "angle_command_p90_deg": command_angle_p90,
            "angle_desired_p90_deg": des_angle_p90,
            "angle_desired_over_command": angle_pass_through_ratio,
        },
        "oracle_A": {
            "inside": a_inside,
            "max_attitude_error_deg": max_error,
            "sustained_divergence": divergence,
            "consequence_crash_or_ground_hit": consequence_crash,
            "crash_markers": crash_markers,
            "low_alt_markers": low_alt_markers[:3],
        },
        "oracle_B": {
            "inside": b_inside,
            "preventive_failsafe": preventive,
            "fail_events": fail_events,
            "fail_texts": fail_texts,
            "mode_switches_during_maneuver": mode_switches,
            "command_touched_angle_max": command_touched_limit,
            "command_exceeded_angle_max": command_exceeded_limit,
        },
        "authority": authority,
        "modes": modes,
        "messages_tail": messages[-20:],
        "label": label,
    }
    write_json(oracle_path, result)
    return result


def run_one(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    run_id = run_id_for(config, point)
    cfg = point_config(config, point)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "point": point,
        "started_at_utc": utc_now(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=35)
        request_streams(master, int(config["experiment"]["stream_hz"]))
        params = point_params(config, point)
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result.update(
            {
                "work_dir": str(work_dir),
                "params_requested": params,
                "param_snapshot": snapshot,
                "param_records_path": str(param_path),
                "param_readbacks": pm.records,
            }
        )

        wait_position_stable(master, min_samples=5, timeout_s=45)
        set_mode(master, "GUIDED", timeout_s=20)
        arm(master, timeout_s=45)
        command_takeoff(master, float(config["experiment"]["takeoff_alt_m"]))
        wait_altitude(master, float(config["experiment"]["takeoff_alt_m"]), timeout_s=55)
        requested_mode = str(config["experiment"].get("use_mode", "GUIDED_NOGPS"))
        if requested_mode != "GUIDED":
            set_mode(master, requested_mode, timeout_s=20)

        yaw_deg = float(config["experiment"].get("yaw_deg", config["experiment"]["home"].get("yaw_deg", 0.0)))
        thrust = float(config["command"].get("thrust", 0.5))
        attitude_ignore = str(config["command"].get("attitude_target_mode", "attitude_and_rate")) == "rate_only"
        speedup = max(1.0, float(config["experiment"].get("speedup", 1.0)))
        hz = float(config["experiment"]["stream_hz"])
        neutral_dt_wall = 1.0 / (hz * speedup)
        pre_hold_end = time.time() + float(config["experiment"].get("pre_maneuver_hold_s", 1.0)) / speedup
        while time.time() < pre_hold_end:
            send_gcs_heartbeat(master)
            send_attitude_target(
                master,
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=yaw_deg,
                roll_rate_deg_s=0.0,
                pitch_rate_deg_s=0.0,
                yaw_rate_deg_s=0.0,
                thrust=thrust,
                attitude_ignore=attitude_ignore,
            )
            time.sleep(neutral_dt_wall)

        profile = doublet_profile(config, float(point["r_deg_s"]), float(point["angle_max_cd"]))
        profile_path = PLANC_ROOT / "logs" / f"{run_id}_command_profile.json"
        write_json(
            profile_path,
            {
                "run_id": run_id,
                "point": point,
                "attitude_target_mode": config["command"].get("attitude_target_mode", "attitude_and_rate"),
                "samples": profile,
            },
        )
        result["command_profile"] = {
            "samples": len(profile),
            "duration_s": profile[-1]["t_s"] if profile else 0.0,
            "amplitude_deg": command_amplitude_deg(config, float(point["angle_max_cd"])),
            "wall_timing_scaled_by_speedup": speedup,
            "path": str(profile_path),
        }
        live_events: list[dict[str, Any]] = []
        t0 = time.time()
        early_stop = None
        for sample in profile:
            deadline = t0 + float(sample["t_s"]) / speedup
            while time.time() < deadline:
                time.sleep(min(0.002, deadline - time.time()))
            send_gcs_heartbeat(master)
            send_attitude_target(
                master,
                roll_deg=float(sample["roll_deg"]),
                pitch_deg=float(sample["pitch_deg"]),
                yaw_deg=yaw_deg,
                roll_rate_deg_s=float(sample["roll_rate_deg_s"]),
                pitch_rate_deg_s=float(sample["pitch_rate_deg_s"]),
                yaw_rate_deg_s=0.0,
                thrust=thrust,
                attitude_ignore=attitude_ignore,
            )
            while True:
                msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT", "GLOBAL_POSITION_INT"], blocking=False)
                if msg is None:
                    break
                typ = msg.get_type()
                if typ == "STATUSTEXT":
                    text = str(getattr(msg, "text", ""))
                    live_events.append({"time_wall_s": time.time() - t0, "type": typ, "text": text})
                    if "crash" in text.lower() or "failsafe" in text.lower():
                        early_stop = text
                elif typ == "HEARTBEAT":
                    if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        early_stop = "disarmed during maneuver"
                elif typ == "GLOBAL_POSITION_INT":
                    rel_alt_m = float(getattr(msg, "relative_alt", 0.0)) / 1000.0
                    if rel_alt_m < 0.7:
                        early_stop = f"low altitude during maneuver: {rel_alt_m:.2f} m"
                if early_stop:
                    break
            if early_stop:
                break
        result["live_events"] = live_events[-20:]
        result["early_stop"] = early_stop

        try:
            land_and_disarm(master, timeout_s=float(config["experiment"].get("cleanup_land_timeout_s", 22.0)))
        except Exception as exc:
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
        parsed = parse_stage0_dataflash(
            bin_path=bin_path,
            csv_path=PLANC_ROOT / "logs" / f"{run_id}_parsed.csv",
            oracle_path=PLANC_ROOT / "logs" / f"{run_id}_parsed.oracle.json",
            config=config,
            point=point,
            command_profile=profile,
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


def run_cached(
    config: dict[str, Any],
    point: dict[str, Any],
    partial_path: Path,
    runs: list[dict[str, Any]],
    resume: bool,
) -> dict[str, Any]:
    run_id = run_id_for(config, point)
    if resume:
        for existing in runs:
            if existing.get("run_id") == run_id:
                return existing
    print(f"RUN {run_id}", flush=True)
    run = run_one(config, point)
    runs = [r for r in runs if r.get("run_id") != run_id] + [run]
    write_json(partial_path, {"runs": runs, "updated_at_utc": utc_now()})
    return run


def fidelity_pass(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    min_ratio = float(config["oracle"]["path_fidelity_min_ratio"])
    rate_only = str(config["command"].get("attitude_target_mode", "attitude_and_rate")) == "rate_only"
    checks = []
    ok = True
    for run in runs:
        pf = run.get("path_fidelity", {})
        pass_ratio = pf.get("rate_desired_over_command")
        achieved_ratio = pf.get("rate_actual_over_command")
        angle_ratio = pf.get("angle_desired_over_command")
        run_ok = (
            not run.get("error")
            and pass_ratio is not None
            and achieved_ratio is not None
            and float(pass_ratio) >= min_ratio
            and float(achieved_ratio) >= min_ratio
            and (rate_only or (angle_ratio is not None and float(angle_ratio) >= min_ratio))
        )
        checks.append(
            {
                "run_id": run.get("run_id"),
                "ok": run_ok,
                "rate_desired_over_command": pass_ratio,
                "rate_actual_over_command": achieved_ratio,
                "angle_desired_over_command": angle_ratio,
                "error": run.get("error"),
            }
        )
        ok = ok and run_ok
    reason = "direct attitude/rate path met the 90% fidelity preregistration" if ok else "direct attitude/rate path fidelity failed"
    return {"ok": ok, "min_required_ratio": min_ratio, "reason": reason, "checks": checks}


def make_default_point(config: dict[str, Any], *, role: str, r: float, seed: int, layer: str = "default", wind: float | None = None, turb: float | None = None, angle_max_cd: float | None = None, model: str | None = None) -> dict[str, Any]:
    pressure = config["pressure"]
    return {
        "role": role,
        "layer": layer,
        "r_deg_s": float(r),
        "seed": int(seed),
        "wind_m_s": float(pressure["default_wind_m_s"] if wind is None else wind),
        "turbulence_m_s": float(pressure["default_turbulence_m_s"] if turb is None else turb),
        "angle_max_cd": float(config["baseline_params"]["ANGLE_MAX"] if angle_max_cd is None else angle_max_cd),
        "model": str(pressure.get("default_model", "m100") if model is None else model),
    }


def pressure_batches(config: dict[str, Any]) -> list[dict[str, Any]]:
    pressure = config["pressure"]
    seeds = [int(s) for s in pressure["seeds"]]
    r_sweep = [float(r) for r in pressure["r_sweep_deg_s"]]
    batches: list[dict[str, Any]] = []

    batches.append(
        {
            "name": "default",
            "knob": "baseline",
            "points": [
                make_default_point(config, role="p2", layer="default", r=r, seed=s)
                for r in r_sweep
                for s in seeds
            ],
        }
    )
    for layer in pressure["wind_turbulence_layers"]:
        if str(layer["layer"]) == "default":
            continue
        batches.append(
            {
                "name": f"wind_turbulence_{layer['layer']}",
                "knob": "wind_turbulence",
                "points": [
                    make_default_point(
                        config,
                        role="p2",
                        layer=str(layer["layer"]),
                        r=r,
                        seed=s,
                        wind=float(layer["wind_m_s"]),
                        turb=float(layer["turbulence_m_s"]),
                    )
                    for r in r_sweep
                    for s in seeds
                ],
            }
        )
    for model_name in config.get("models", {}):
        if model_name == str(pressure.get("default_model", "m100")):
            continue
        batches.append(
            {
                "name": f"lower_twr_{model_name}",
                "knob": "lower_twr",
                "points": [
                    make_default_point(config, role="p2", layer=f"twr_{model_name}", r=r, seed=s, model=model_name)
                    for r in r_sweep
                    for s in seeds
                ],
            }
        )
    for layer in pressure["angle_max_layers"]:
        if str(layer["layer"]) in {"default", "tight_direction_check"}:
            continue
        batches.append(
            {
                "name": f"angle_max_{layer['layer']}",
                "knob": "angle_max",
                "points": [
                    make_default_point(
                        config,
                        role="p2",
                        layer=str(layer["layer"]),
                        r=r,
                        seed=s,
                        angle_max_cd=float(layer["angle_max_cd"]),
                    )
                    for r in r_sweep
                    for s in seeds
                ],
            }
        )
    batches.append(
        {
            "name": "sharper_doublet",
            "knob": "sharper_doublet",
            "points": [
                make_default_point(config, role="p2", layer="sharp", r=float(r), seed=s)
                for r in pressure["sharper_r_sweep_deg_s"]
                for s in seeds
            ],
        }
    )
    return batches


def summarize_pressure(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    complete = [r for r in runs if not r.get("error")]
    a_runs = [r for r in complete if r.get("oracle_A", {}).get("inside")]
    authority_runs = [r for r in complete if r.get("authority", {}).get("authority_limited")]
    max_error = max((float(r.get("oracle_A", {}).get("max_attitude_error_deg") or 0.0) for r in complete), default=None)
    first_a = min(
        a_runs,
        key=lambda r: (
            float(r["point"].get("r_deg_s", 0.0)),
            float(r["point"].get("turbulence_m_s", 0.0)),
            float(r["point"].get("wind_m_s", 0.0)),
        ),
        default=None,
    )
    return {
        "completed": len(complete),
        "errors": len([r for r in runs if r.get("error")]),
        "a_count": len(a_runs),
        "authority_limited_count": len(authority_runs),
        "max_attitude_error_deg": max_error,
        "target_error_deg": float(config["pressure"]["target_error_deg"]),
        "first_A_run_id": None if first_a is None else first_a.get("run_id"),
        "first_A_point": None if first_a is None else first_a.get("point"),
        "pressure_sufficient": bool(
            a_runs
            or authority_runs
            or (max_error is not None and max_error >= float(config["pressure"]["target_error_deg"]))
        ),
    }


def selected_boundary_center(config: dict[str, Any], p2_runs: list[dict[str, Any]]) -> dict[str, Any] | None:
    candidates = [r for r in p2_runs if not r.get("error") and r.get("oracle_A", {}).get("inside")]
    if not candidates:
        return None
    overdraw = [r for r in candidates if r.get("label") == "overdraw"]
    chosen_pool = overdraw or candidates
    chosen = min(
        chosen_pool,
        key=lambda r: (
            float(r["point"].get("r_deg_s", 0.0)),
            float(r["point"].get("turbulence_m_s", 0.0)),
            float(r["point"].get("wind_m_s", 0.0)),
            float(r["point"].get("angle_max_cd", 0.0)),
        ),
    )
    return dict(chosen["point"])


def boundary_grid_points(config: dict[str, Any], center: dict[str, Any]) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    base_r = float(center["r_deg_s"])
    for layer in config["boundary_grid"]["turbulence_layers"]:
        for step in config["boundary_grid"]["r_radius_steps"]:
            r = max(1.0, base_r + float(step))
            for seed in config["boundary_grid"]["seeds"]:
                p = make_default_point(
                    config,
                    role="p3",
                    layer=str(layer["layer"]),
                    r=r,
                    seed=int(seed),
                    wind=float(layer["wind_m_s"]),
                    turb=float(layer["turbulence_m_s"]),
                    angle_max_cd=float(center["angle_max_cd"]),
                    model=str(center.get("model", config["pressure"].get("default_model", "m100"))),
                )
                points.append(p)
    unique: dict[str, dict[str, Any]] = {}
    for point in points:
        unique[run_id_for(config, point)] = point
    return list(unique.values())


def classify_grid(runs: list[dict[str, Any]]) -> dict[str, Any]:
    complete = [r for r in runs if not r.get("error")]
    counts = {"overdraw": 0, "bug_side": 0, "safe": 0, "blocked": 0}
    for run in runs:
        counts[run.get("label", "blocked") if not run.get("error") else "blocked"] = counts.get(run.get("label", "blocked"), 0) + 1
    cells: dict[tuple[float, str], list[dict[str, Any]]] = {}
    for run in complete:
        point = run["point"]
        key = (float(point["r_deg_s"]), str(point.get("layer", "")))
        cells.setdefault(key, []).append(run)
    stable_overdraw_cells = []
    stable_bug_cells = []
    for (r, layer), rows in cells.items():
        over = len([row for row in rows if row.get("label") == "overdraw"])
        bug = len([row for row in rows if row.get("label") == "bug_side"])
        a = len([row for row in rows if row.get("oracle_A", {}).get("inside")])
        total = len(rows)
        cell = {"r_deg_s": r, "layer": layer, "total": total, "A": a, "overdraw": over, "bug_side": bug}
        if over >= 2:
            stable_overdraw_cells.append(cell)
        if bug >= 2:
            stable_bug_cells.append(cell)
    return {
        "counts": counts,
        "stable_overdraw_cells": stable_overdraw_cells,
        "stable_bug_side_cells": stable_bug_cells,
        "complete": len(complete),
        "blocked": len([r for r in runs if r.get("error")]),
    }


def boundary_separation(runs: list[dict[str, Any]]) -> dict[str, Any]:
    complete = [r for r in runs if not r.get("error")]
    by_layer: dict[str, list[dict[str, Any]]] = {}
    for run in complete:
        by_layer.setdefault(str(run["point"].get("layer", "")), []).append(run)
    layers = []
    for layer, rows in sorted(by_layer.items()):
        r_values = sorted({float(row["point"]["r_deg_s"]) for row in rows})
        a_rs = [float(row["point"]["r_deg_s"]) for row in rows if row.get("oracle_A", {}).get("inside")]
        b_rs = [float(row["point"]["r_deg_s"]) for row in rows if row.get("oracle_B", {}).get("inside")]
        over_rs = [float(row["point"]["r_deg_s"]) for row in rows if row.get("label") == "overdraw"]
        a_boundary = min(a_rs) if a_rs else None
        b_boundary = min(b_rs) if b_rs else None
        if a_boundary is not None and b_boundary is not None:
            separation = b_boundary - a_boundary
        elif a_boundary is not None and b_boundary is None and r_values:
            separation = f">={max(r_values) - a_boundary:.1f}"
        else:
            separation = None
        layers.append(
            {
                "layer": layer,
                "r_values_deg_s": r_values,
                "A_boundary_r_deg_s": a_boundary,
                "B_boundary_r_deg_s": b_boundary,
                "separation_r_deg_s": separation,
                "overdraw_r_values_deg_s": sorted(set(over_rs)),
            }
        )
    return {"layers": layers}


def overdraw_region_summary(p3_summary: dict[str, Any], separation: dict[str, Any]) -> dict[str, Any]:
    stable = list(p3_summary.get("stable_overdraw_cells", []))
    counts = dict(p3_summary.get("counts", {}))
    first_entry_zero = [
        layer
        for layer in separation.get("layers", [])
        if layer.get("A_boundary_r_deg_s") is not None
        and layer.get("B_boundary_r_deg_s") is not None
        and layer.get("A_boundary_r_deg_s") == layer.get("B_boundary_r_deg_s")
        and layer.get("overdraw_r_values_deg_s")
    ]
    return {
        "overdraw_run_count": int(counts.get("overdraw", 0)),
        "stable_overdraw_cell_count": len(stable),
        "stable_overdraw_cells": stable,
        "first_entry_boundaries_overlap_but_overdraw_exists": bool(first_entry_zero),
        "interpretation": (
            "The first A and B entry thresholds can coincide while B is non-monotone in r; "
            "the observed separation is an OverDraw island, not a simple first-threshold offset."
            if first_entry_zero
            else "Observed OverDraw is represented by stable cells and run count."
        ),
    }


def monotonicity_pre_read(runs: list[dict[str, Any]]) -> dict[str, Any]:
    complete = [r for r in runs if not r.get("error")]
    by_layer: dict[str, list[dict[str, Any]]] = {}
    for run in complete:
        by_layer.setdefault(str(run["point"].get("layer", "")), []).append(run)
    layers = []
    for layer, rows in sorted(by_layer.items()):
        by_r: dict[float, list[dict[str, Any]]] = {}
        for run in rows:
            by_r.setdefault(float(run["point"]["r_deg_s"]), []).append(run)
        series = []
        for r_value, r_rows in sorted(by_r.items()):
            throttle = [float(row.get("authority", {}).get("throttle_near_limit_fraction") or 0.0) for row in r_rows]
            slew = [float(row.get("authority", {}).get("aout_slew_p95") or 0.0) for row in r_rows]
            rate_out = [float(row.get("authority", {}).get("rate_out_abs_p95") or 0.0) for row in r_rows]
            series.append(
                {
                    "r_deg_s": r_value,
                    "n": len(r_rows),
                    "throttle_near_limit_fraction_mean": statistics.fmean(throttle) if throttle else None,
                    "aout_slew_p95_mean": statistics.fmean(slew) if slew else None,
                    "rate_out_abs_p95_mean": statistics.fmean(rate_out) if rate_out else None,
                }
            )
        def nondecreasing(key: str, tol: float = 1.0e-9) -> bool | None:
            vals = [row.get(key) for row in series if row.get(key) is not None]
            if len(vals) < 2:
                return None
            return all(float(vals[i]) + tol >= float(vals[i - 1]) for i in range(1, len(vals)))

        layers.append(
            {
                "layer": layer,
                "series": series,
                "throttle_near_limit_monotone": nondecreasing("throttle_near_limit_fraction_mean"),
                "aout_slew_monotone": nondecreasing("aout_slew_p95_mean"),
                "rate_out_monotone": nondecreasing("rate_out_abs_p95_mean"),
            }
        )
    any_authority = any(bool(r.get("authority", {}).get("authority_limited")) for r in complete)
    return {
        "evaluated": any_authority,
        "reason": "authority signals entered the limited region" if any_authority else "authority did not enter a limited region",
        "layers": layers if any_authority else [],
    }


def decide_verdict(
    *,
    fidelity: dict[str, Any],
    p2_summary: dict[str, Any],
    p2_runs: list[dict[str, Any]],
    p3_summary: dict[str, Any],
    exhausted_pressure: bool,
) -> dict[str, Any]:
    if not fidelity.get("ok"):
        return {
            "verdict": "NEEDS-TUNING",
            "reason": "P1 path fidelity failed; do not adjudicate boundary separation.",
        }
    if p3_summary.get("stable_overdraw_cells"):
        return {
            "verdict": "GO",
            "reason": "Robust OverDraw exists: at least one P3 cell is A inside, B outside in >=2 seeds.",
        }
    all_a = [r for r in p2_runs if not r.get("error") and r.get("oracle_A", {}).get("inside")]
    if all_a and not [r for r in all_a if r.get("label") == "overdraw"]:
        return {
            "verdict": "NO-GO (contract/bug side)",
            "reason": "Every observed A point is also B inside.",
        }
    if exhausted_pressure and not all_a:
        return {
            "verdict": "NO-GO (mechanism)/INCONCLUSIVE",
            "reason": "Configured legal pressure knobs were exhausted without entering oracle A.",
        }
    if p2_summary.get("a_count", 0) > 0 and not p3_summary.get("stable_overdraw_cells"):
        return {
            "verdict": "INCONCLUSIVE",
            "reason": "A was observed, but P3 did not show seed-stable OverDraw.",
        }
    return {
        "verdict": "NEEDS-TUNING",
        "reason": "Pressure/consequence path stopped before a decisive A/B boundary adjudication.",
    }


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis_dir = PLANC_ROOT / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    stem = artifact_stem(payload["config"])
    p2 = [r for r in payload.get("runs", {}).get("p2", []) if not r.get("error")]
    p3 = [r for r in payload.get("runs", {}).get("p3", []) if not r.get("error")]
    all_runs = p2 + p3
    paths: dict[str, str] = {}

    if all_runs:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for label, color in [("safe", "#2a9d8f"), ("overdraw", "#e76f51"), ("bug_side", "#6d597a")]:
            subset = [r for r in all_runs if r.get("label") == label]
            if not subset:
                continue
            ax.scatter(
                [float(r["point"]["r_deg_s"]) for r in subset],
                [float(r.get("oracle_A", {}).get("max_attitude_error_deg") or 0.0) for r in subset],
                label=label,
                color=color,
                alpha=0.8,
            )
        ax.axhline(float(payload["config"]["oracle"]["divergence_error_deg"]), color="#222222", linestyle="--", linewidth=1)
        ax.set_xlabel("r (deg/s)")
        ax.set_ylabel("max attitude tracking error (deg)")
        ax.set_title("Stage-0 v2 boundary A pressure")
        ax.grid(True, alpha=0.25)
        ax.legend()
        path = analysis_dir / f"{stem}_boundary_A.png"
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths["boundary_A"] = str(path)

    if p3:
        layer_order = {name: i for i, name in enumerate(sorted({str(r["point"].get("layer", "")) for r in p3}))}
        fig, ax = plt.subplots(figsize=(8, 4.5))
        colors = {"safe": "#2a9d8f", "overdraw": "#e76f51", "bug_side": "#6d597a"}
        for run in p3:
            y = layer_order[str(run["point"].get("layer", ""))] + 0.08 * int(run["point"].get("seed", 0))
            ax.scatter(float(run["point"]["r_deg_s"]), y, color=colors.get(run.get("label"), "#777777"), s=45, alpha=0.85)
        ax.set_yticks(list(layer_order.values()))
        ax.set_yticklabels(list(layer_order.keys()))
        ax.set_xlabel("r (deg/s)")
        ax.set_ylabel("turbulence layer")
        ax.set_title("Boundary separation and OverDraw/Bug-side scatter")
        ax.grid(True, axis="x", alpha=0.25)
        path = analysis_dir / f"{stem}_boundary_separation.png"
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths["boundary_separation"] = str(path)

    if all_runs:
        fig, ax1 = plt.subplots(figsize=(8, 4.5))
        xs = [float(r["point"]["r_deg_s"]) for r in all_runs]
        throttle = [float(r.get("authority", {}).get("throttle_near_limit_fraction") or 0.0) for r in all_runs]
        slew = [float(r.get("authority", {}).get("aout_slew_p95") or 0.0) for r in all_runs]
        ax1.scatter(xs, throttle, color="#264653", label="Thr near-limit fraction", alpha=0.75)
        ax1.set_xlabel("r (deg/s)")
        ax1.set_ylabel("ThrOut near ThrAvMx fraction")
        ax1.set_ylim(-0.03, 1.03)
        ax2 = ax1.twinx()
        ax2.scatter(xs, slew, color="#f4a261", label="AOutSlew p95", alpha=0.65)
        ax2.set_ylabel("RATE.AOutSlew p95")
        ax1.set_title("Authority signal evidence")
        ax1.grid(True, alpha=0.25)
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left")
        path = analysis_dir / f"{stem}_authority_evidence.png"
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths["authority_evidence"] = str(path)

    return paths


def write_report(payload: dict[str, Any]) -> str:
    stem = artifact_stem(payload["config"])
    path = PLANC_ROOT / "results" / f"{stem}_report.md"
    verdict = payload["verdict"]
    fidelity = payload["p1_fidelity"]
    p2 = payload["p2_pressure"]
    p3 = payload["p3_boundary"]
    sep = payload["boundary_separation"]
    overdraw = payload.get("overdraw_region", {})
    p4 = payload.get("p4_monotonicity_pre_read", {})

    lines = [
        f"# Stage-0 v2 Control Authority Probe",
        "",
        f"VERDICT: **{verdict['verdict']}**",
        "",
        verdict["reason"],
        "",
        "## Preregistered Criteria",
        "",
        f"- Oracle A: attitude tracking error > {payload['config']['oracle']['divergence_error_deg']} deg for {payload['config']['oracle']['divergence_duration_s']} s, or crash/ground-hit before cleanup.",
        "- Oracle B: preventive failsafe/mode intervention, or command angle touching ANGLE_MAX.",
        f"- Command path: SET_ATTITUDE_TARGET in GUIDED_NOGPS after GUIDED takeoff; r is bounded-attitude doublet ramp rate in deg/s.",
        f"- Seed group: {payload['config']['pressure']['seeds']} ({payload['config']['param_metadata']['turbulence_seed_note']})",
        "",
        "## P1 Path Fidelity",
        "",
        f"Conclusion: **{fidelity['ok']}** - {fidelity['reason']}.",
        "",
        "| run | RATE desired/cmd | RATE actual/cmd | ATT target/cmd diagnostic | error |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for check in fidelity["checks"]:
        lines.append(
            f"| {check['run_id']} | {fmt(check['rate_desired_over_command'], 3)} | {fmt(check['rate_actual_over_command'], 3)} | {fmt(check['angle_desired_over_command'], 3)} | {check.get('error') or ''} |"
        )

    lines.extend(
        [
            "",
            "## P2 Pressure",
            "",
            f"Completed: {p2['completed']} runs; A count: {p2['a_count']}; authority-limited count: {p2['authority_limited_count']}; max error: {fmt(p2['max_attitude_error_deg'], 2)} deg.",
            f"First A: `{p2.get('first_A_run_id')}` at `{json.dumps(p2.get('first_A_point'), sort_keys=True)}`.",
            "",
            "## P3 Boundary Separation",
            "",
            f"Counts: `{json.dumps(p3['counts'], sort_keys=True)}`.",
            f"Stable overdraw cells: `{json.dumps(p3['stable_overdraw_cells'], sort_keys=True)}`.",
            f"OverDraw scale: {overdraw.get('overdraw_run_count', 0)} runs; {overdraw.get('stable_overdraw_cell_count', 0)} stable cells.",
            f"Boundary interpretation: {overdraw.get('interpretation', 'n/a')}",
            f"Stable bug-side cells: `{json.dumps(p3['stable_bug_side_cells'], sort_keys=True)}`.",
            "",
            "| layer | A boundary r | B boundary r | separation | overdraw r values |",
            "| --- | ---: | ---: | ---: | --- |",
        ]
    )
    for layer in sep["layers"]:
        lines.append(
            f"| {layer['layer']} | {fmt(layer['A_boundary_r_deg_s'], 1)} | {fmt(layer['B_boundary_r_deg_s'], 1)} | {layer['separation_r_deg_s']} | {layer['overdraw_r_values_deg_s']} |"
        )

    lines.extend(
        [
            "",
            "## P4 Monotonicity Pre-read",
            "",
            f"Evaluated: **{p4.get('evaluated')}** - {p4.get('reason', 'n/a')}.",
            "",
            "| layer | throttle near-limit monotone | AOutSlew monotone | rate-out monotone |",
            "| --- | --- | --- | --- |",
        ]
    )
    for layer in p4.get("layers", []):
        lines.append(
            f"| {layer['layer']} | {layer.get('throttle_near_limit_monotone')} | {layer.get('aout_slew_monotone')} | {layer.get('rate_out_monotone')} |"
        )

    lines.extend(
        [
            "",
            "## Evidence Artifacts",
            "",
            f"- Preregistration: `planc/results/{stem}_prereg.json`",
            f"- Result JSON: `planc/results/{stem}_result.json`",
            f"- Partial runs: `planc/results/{stem}_partial.json`",
        ]
    )
    for name, plot_path in payload.get("artifacts", {}).get("plots", {}).items():
        lines.append(f"- Plot {name}: `{Path(plot_path).relative_to(REPO_ROOT)}`")
    lines.append(f"- Parsed logs: `planc/logs/{run_prefix(payload['config'])}_*_parsed.csv` and `*_parsed.oracle.json`")
    lines.append(f"- Raw DataFlash: `planc/logs/{run_prefix(payload['config'])}_*.BIN` (local workspace evidence, ignored by Git)")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(path)


def build_payload(
    config: dict[str, Any],
    env: dict[str, Any],
    prereg: dict[str, Any],
    p1_runs: list[dict[str, Any]],
    p2_runs: list[dict[str, Any]],
    p3_runs: list[dict[str, Any]],
    exhausted_pressure: bool,
) -> dict[str, Any]:
    fidelity = fidelity_pass(config, p1_runs)
    p2_summary = summarize_pressure(config, p2_runs)
    p3_summary = classify_grid(p3_runs)
    separation = boundary_separation(p3_runs or p2_runs)
    overdraw_region = overdraw_region_summary(p3_summary, separation)
    p4_monotonicity = monotonicity_pre_read(p3_runs or p2_runs)
    verdict = decide_verdict(
        fidelity=fidelity,
        p2_summary=p2_summary,
        p2_runs=p2_runs,
        p3_summary=p3_summary,
        exhausted_pressure=exhausted_pressure,
    )
    payload = {
        "status": "complete",
        "generated_at_utc": utc_now(),
        "config": config,
        "env": env,
        "preregistration": prereg,
        "verdict": verdict,
        "p1_fidelity": fidelity,
        "p2_pressure": p2_summary,
        "p3_boundary": p3_summary,
        "boundary_separation": separation,
        "overdraw_region": overdraw_region,
        "p4_monotonicity_pre_read": p4_monotonicity,
        "runs": {"p1": p1_runs, "p2": p2_runs, "p3": p3_runs},
        "stage0_v2_result": {
            "boundaries_separated": bool(p3_summary.get("stable_overdraw_cells")),
            "overdraw_count": int(p3_summary.get("counts", {}).get("overdraw", 0)),
            "stable_overdraw_cell_count": int(overdraw_region["stable_overdraw_cell_count"]),
            "overdraw_region": overdraw_region,
            "bug_side_count": int(p3_summary.get("counts", {}).get("bug_side", 0)),
            "path_fidelity": fidelity,
            "transition_reason_if_no_go": verdict["reason"] if verdict["verdict"] != "GO" else None,
        },
        "artifacts": {},
    }
    payload["artifacts"]["plots"] = make_plots(payload)
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "stage0_v2_config.yaml")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--p1-only", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    stem = artifact_stem(config)
    results_dir = PLANC_ROOT / "results"
    partial_path = results_dir / f"{stem}_partial.json"
    final_path = results_dir / f"{stem}_result.json"
    prereg_path = results_dir / f"{stem}_prereg.json"
    env_path = results_dir / f"env_{stem}.json"

    env = probe_environment(config, REPO_ROOT)
    write_env(env, env_path)
    prereg = write_preregister(config, prereg_path)

    partial = load_json(partial_path, {"runs": []}) if args.resume else {"runs": []}
    all_runs: list[dict[str, Any]] = list(partial.get("runs", []))

    p1_runs = []
    for spec in config["fidelity"]["runs"]:
        point = make_default_point(
            config,
            role=str(spec["role"]),
            layer="fidelity",
            r=float(spec["r_deg_s"]),
            seed=int(spec["seed"]),
            wind=float(spec["wind_m_s"]),
            turb=float(spec["turbulence_m_s"]),
            angle_max_cd=float(config["baseline_params"]["ANGLE_MAX"]),
        )
        p1_runs.append(run_cached(config, point, partial_path, all_runs, args.resume))
        all_runs = list(load_json(partial_path, {"runs": []}).get("runs", []))

    p1 = fidelity_pass(config, p1_runs)
    if args.p1_only or not p1["ok"]:
        payload = build_payload(config, env, prereg, p1_runs, [], [], exhausted_pressure=False)
        write_json(final_path, payload)
        print(f"COMPLETE: verdict={payload['verdict']['verdict']} result={final_path}", flush=True)
        return

    p2_runs: list[dict[str, Any]] = []
    found_a = False
    exhausted_pressure = True
    for batch in pressure_batches(config):
        for point in batch["points"]:
            run = run_cached(config, point, partial_path, all_runs, args.resume)
            p2_runs.append(run)
            all_runs = list(load_json(partial_path, {"runs": []}).get("runs", []))
            if not run.get("error") and run.get("oracle_A", {}).get("inside"):
                found_a = True
        if found_a:
            exhausted_pressure = False
            break

    center = selected_boundary_center(config, p2_runs)
    p3_runs: list[dict[str, Any]] = []
    if center is not None:
        for point in boundary_grid_points(config, center):
            run = run_cached(config, point, partial_path, all_runs, args.resume)
            p3_runs.append(run)
            all_runs = list(load_json(partial_path, {"runs": []}).get("runs", []))

        tight = next((layer for layer in config["pressure"]["angle_max_layers"] if str(layer["layer"]) == "tight_direction_check"), None)
        if tight is not None:
            for seed in config["pressure"]["seeds"]:
                point = make_default_point(
                    config,
                    role="p3",
                    layer="tight_direction_check",
                    r=float(center["r_deg_s"]),
                    seed=int(seed),
                    angle_max_cd=float(tight["angle_max_cd"]),
                    model=str(center.get("model", config["pressure"].get("default_model", "m100"))),
                )
                run = run_cached(config, point, partial_path, all_runs, args.resume)
                p3_runs.append(run)
                all_runs = list(load_json(partial_path, {"runs": []}).get("runs", []))

    payload = build_payload(config, env, prereg, p1_runs, p2_runs, p3_runs, exhausted_pressure=exhausted_pressure)
    write_json(final_path, payload)
    print(
        f"COMPLETE: verdict={payload['verdict']['verdict']} result={final_path} report={payload['artifacts']['report']}",
        flush=True,
    )


if __name__ == "__main__":
    main()
