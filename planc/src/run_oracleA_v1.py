from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
import traceback
from collections import Counter, defaultdict
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
    arm,
    command_takeoff,
    request_streams,
    send_gcs_heartbeat,
    set_mode,
    wait_altitude,
    wait_position_stable,
)
from oracle import COPTER_MODES, ERROR_SUBSYSTEMS
from param_manager import ParamManager
from run_stage0_v2 import (
    command_amplitude_deg,
    command_at,
    doublet_profile,
    fmt,
    point_config,
    point_params,
    profile_active_bounds,
    run_id_for,
    send_attitude_target,
)
from sitl_runner import SitlRunner


MODE_REASONS = {
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
    13: "FLIP_COMPLETE",
    14: "AVOIDANCE",
    15: "AVOIDANCE_RECOVERY",
    16: "THROW_COMPLETE",
    17: "TERMINATE",
    18: "TOY_MODE",
    19: "CRASH_FAILSAFE",
    25: "FAILSAFE",
    26: "INITIALISED",
    29: "LEAK_FAILSAFE",
    50: "DEADRECKON_FAILSAFE",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_json(path: Path, default: dict[str, Any] | None = None) -> dict[str, Any]:
    if not path.exists():
        return default or {}
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def artifact_stem(config: dict[str, Any]) -> str:
    return str(config["experiment"].get("artifact_stem", "oracleA_v1"))


def reason_name(reason_value: Any) -> str:
    try:
        return MODE_REASONS.get(int(reason_value), str(reason_value))
    except Exception:
        return str(reason_value)


def mode_name(mode_value: Any) -> str:
    try:
        return COPTER_MODES.get(int(mode_value), str(mode_value))
    except Exception:
        return str(mode_value)


def time_s(data: dict[str, Any]) -> float | None:
    if "TimeUS" in data:
        return float(data["TimeUS"]) / 1.0e6
    if "TimeMS" in data:
        return float(data["TimeMS"]) / 1000.0
    return None


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


def nearest_value(rows: list[tuple[float, float]], t_s: float | None) -> float | None:
    if not rows or t_s is None:
        return None
    return min(rows, key=lambda item: abs(item[0] - t_s))[1]


def sustained_exceed(rows: list[tuple[float, float]], threshold: float, duration_s: float) -> dict[str, Any]:
    over_start: float | None = None
    previous_t: float | None = None
    best = 0.0
    intervals: list[dict[str, float]] = []
    for t_s, value in rows:
        if value > threshold:
            if over_start is None:
                over_start = t_s
        elif over_start is not None:
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


def slope_deg_s(rows: list[tuple[float, float]]) -> float | None:
    if len(rows) < 2:
        return None
    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    xbar = statistics.fmean(xs)
    ybar = statistics.fmean(ys)
    denom = sum((x - xbar) ** 2 for x in xs)
    if denom <= 0.0:
        return None
    return sum((x - xbar) * (y - ybar) for x, y in zip(xs, ys)) / denom


def first_time(rows: list[dict[str, Any]]) -> float | None:
    times = [float(r["time_s"]) for r in rows if r.get("time_s") is not None]
    return min(times) if times else None


def disarm_for_cleanup(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def wait_disarmed(master, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is not None and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def point_from_spec(config: dict[str, Any], spec: dict[str, Any], r: float, seed: int, *, role: str) -> dict[str, Any]:
    return {
        "role": role,
        "phase": str(spec.get("name", role)),
        "layer": str(spec.get("layer", "default")),
        "r_deg_s": float(r),
        "seed": int(seed),
        "wind_m_s": float(spec.get("wind_m_s", config["pressure"]["default_wind_m_s"])),
        "turbulence_m_s": float(spec.get("turbulence_m_s", config["pressure"]["default_turbulence_m_s"])),
        "angle_max_cd": float(spec.get("angle_max_cd", config["baseline_params"]["ANGLE_MAX"])),
        "model": str(spec.get("model", config["pressure"]["default_model"])),
    }


def points_for_block(config: dict[str, Any], spec: dict[str, Any], *, role: str) -> list[dict[str, Any]]:
    return [
        point_from_spec(config, spec, float(r), int(seed), role=role)
        for r in spec["r_deg_s"]
        for seed in spec["seeds"]
    ]


def unique_points(points: list[dict[str, Any]], config: dict[str, Any]) -> list[dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for point in points:
        out[run_id_for(config, point)] = point
    return list(out.values())


def write_preregister(config: dict[str, Any], path: Path) -> dict[str, Any]:
    payload = {
        "status": "preregistered",
        "written_at_utc": utc_now(),
        "experiment": config["experiment"],
        "source": {
            "stage0_tag": config["experiment"]["source_stage0_tag"],
            "island_tag": config["experiment"]["source_island_tag"],
        },
        "fixed_P": {
            "mode": config["experiment"]["use_mode"],
            "mavlink_message": "SET_ATTITUDE_TARGET",
            "command": config["command"],
            "baseline_params": config["baseline_params"],
            "cleanup": "MAV_CMD_COMPONENT_ARM_DISARM after the oracle window; no harness LAND inside oracle B.",
        },
        "hardened_oracle_A": {
            "crash_or_contact": (
                f"SITL ground/contact message, or RelHomeAlt < {config['oracle']['h_floor_m']} m "
                f"with downward speed > {config['oracle']['floor_descent_rate_m_s']} m/s"
            ),
            "unrecovered_divergence": (
                f"attitude error > {config['oracle']['divergence_error_deg']} deg for "
                f"{config['oracle']['divergence_duration_s']} s and window-end error remains above "
                f"{config['oracle']['divergence_error_deg']} deg, or tail slope >= "
                f"{config['oracle']['divergence_worsening_slope_deg_s']} deg/s with final error >= "
                f"{config['oracle']['divergence_worsening_min_final_deg']} deg"
            ),
            "altitude_loss": f"RelHomeAlt drop > {config['oracle']['altitude_loss_thresh_m']} m inside oracle window.",
            "transient_recovery": "A transient >60 deg/0.5s that recovers before window end is safe under hardened A.",
        },
        "oracle_B": {
            "B_clamp": "command touches/exceeds ANGLE_MAX within tolerance",
            "B_preventive": "flight-controller preventive MODE reason, ERR subsystem, or failsafe text before the hard A trigger; MODE.Rsn=GCS_COMMAND is harness and excluded",
            "preventive_mode_reasons": config["oracle"]["preventive_mode_reasons"],
            "preventive_failsafe_subsystems": config["oracle"]["preventive_failsafe_subsystems"],
        },
        "sampling_plan": config["sampling_plan"],
        "decision": config["decision"],
    }
    write_json(path, payload)
    return payload


def run_one(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    run_id = run_id_for(config, point)
    cfg = point_config(config, point)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {"run_id": run_id, "point": point, "started_at_utc": utc_now()}
    master = None
    profile: list[dict[str, float]] = []
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
        wait_altitude(master, float(config["experiment"]["takeoff_alt_m"]), timeout_s=75)
        requested_mode = str(config["experiment"].get("use_mode", "GUIDED_NOGPS"))
        if requested_mode != "GUIDED":
            set_mode(master, requested_mode, timeout_s=20)

        yaw_deg = float(config["experiment"].get("yaw_deg", config["experiment"]["home"].get("yaw_deg", 0.0)))
        thrust = float(config["command"].get("thrust", 0.5))
        attitude_ignore = str(config["command"].get("attitude_target_mode", "attitude_and_rate")) == "rate_only"
        speedup = max(1.0, float(config["experiment"].get("speedup", 1.0)))
        hz = float(config["experiment"]["stream_hz"])
        dt_wall = 1.0 / (hz * speedup)
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
            time.sleep(dt_wall)

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
        disarmed_during_window = False
        for sample in profile:
            deadline = t0 + float(sample["t_s"]) / speedup
            while time.time() < deadline:
                sleep_s = max(0.0, min(0.002, deadline - time.time()))
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
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
                    live_events.append({"time_wall_s": time.time() - t0, "type": typ, "text": str(getattr(msg, "text", ""))})
                elif typ == "HEARTBEAT" and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    live_events.append({"time_wall_s": time.time() - t0, "type": typ, "text": "disarmed during oracle window"})
                    disarmed_during_window = True
                elif typ == "GLOBAL_POSITION_INT":
                    live_events.append(
                        {
                            "time_wall_s": time.time() - t0,
                            "type": typ,
                            "rel_alt_m": float(getattr(msg, "relative_alt", 0.0)) / 1000.0,
                        }
                    )
            if disarmed_during_window:
                break

        observe_end = time.time() + float(config["experiment"]["observation_after_profile_s"]) / speedup
        while time.time() < observe_end:
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
            msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT", "GLOBAL_POSITION_INT"], blocking=True, timeout=dt_wall)
            if msg is None:
                continue
            typ = msg.get_type()
            if typ == "STATUSTEXT":
                live_events.append({"time_wall_s": time.time() - t0, "type": typ, "text": str(getattr(msg, "text", ""))})
            elif typ == "HEARTBEAT" and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                live_events.append({"time_wall_s": time.time() - t0, "type": typ, "text": "disarmed during oracle window"})
            elif typ == "GLOBAL_POSITION_INT":
                live_events.append(
                    {
                        "time_wall_s": time.time() - t0,
                        "type": typ,
                        "rel_alt_m": float(getattr(msg, "relative_alt", 0.0)) / 1000.0,
                    }
                )

        result["live_events"] = live_events[-80:]
        disarm_for_cleanup(master)
        result["cleanup_disarmed"] = wait_disarmed(master, float(config["experiment"].get("cleanup_disarm_timeout_s", 8.0)))
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
        parsed = parse_dataflash_hardened(
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
                disarm_for_cleanup(master)
            except Exception:
                pass
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def extract_window(rows: list[dict[str, Any]], lo: float | None, hi: float | None) -> list[dict[str, Any]]:
    if lo is None or hi is None:
        return []
    return [row for row in rows if lo <= float(row["time_s"]) <= hi]


def parse_dataflash_hardened(
    *,
    bin_path: Path,
    csv_path: Path,
    oracle_path: Path,
    config: dict[str, Any],
    point: dict[str, Any],
    command_profile: list[dict[str, float]],
) -> dict[str, Any]:
    msg_types = ["ATT", "RATE", "GUIA", "MOTB", "RCOU", "MODE", "ERR", "EV", "MSG", "POS", "XKF1", "XKF4", "IMU"]
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    rows_by_type: dict[str, list[dict[str, Any]]] = defaultdict(list)
    csv_rows: list[dict[str, Any]] = []

    while True:
        msg = mlog.recv_match(type=msg_types, blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        data = msg.to_dict()
        t = time_s(data)
        if t is None:
            continue
        typ = msg.get_type()
        row = {"time_s": t, "type": typ}
        row.update({k: v for k, v in data.items() if k not in {"mavpackettype", "TimeUS", "TimeMS"}})
        if typ == "MODE":
            row["mode_name"] = mode_name(data.get("ModeNum", data.get("Mode")))
            row["reason_name"] = reason_name(data.get("Rsn"))
        elif typ == "ERR":
            try:
                row["subsystem_name"] = ERROR_SUBSYSTEMS.get(int(data.get("Subsys")), str(data.get("Subsys")))
            except Exception:
                row["subsystem_name"] = str(data.get("Subsys"))
        rows_by_type[typ].append(row)
        csv_rows.append(row)

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if csv_rows:
        fields = ["time_s", "type"]
        extra = sorted({k for row in csv_rows for k in row if k not in fields})
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields + extra)
            writer.writeheader()
            for row in csv_rows:
                writer.writerow(row)

    guia_rows = rows_by_type["GUIA"]
    active_rate = float(config["oracle"]["rate_active_threshold_deg_s"])
    active_guia = [
        row
        for row in guia_rows
        if abs(float(row.get("RollRt", 0.0))) >= active_rate
        or abs(float(row.get("PitchRt", 0.0))) >= active_rate
    ]
    profile_active_start, profile_active_end = profile_active_bounds(config, command_profile)
    if active_guia and profile_active_start is not None and profile_active_end is not None:
        active_start = min(float(row["time_s"]) for row in active_guia) - profile_active_start
        maneuver_end = active_start + profile_active_end
    else:
        active_angle = float(config["oracle"]["attitude_active_threshold_deg"])
        active_att = [
            row
            for row in rows_by_type["ATT"]
            if abs(float(row.get("DesRoll", 0.0))) >= active_angle
            or abs(float(row.get("DesPitch", 0.0))) >= active_angle
        ]
        active_start = min((float(row["time_s"]) for row in active_att), default=None)
        maneuver_end = max((float(row["time_s"]) for row in active_att), default=None)

    profile_end_rel = float(command_profile[-1]["t_s"]) if command_profile else 0.0
    oracle_end = None if active_start is None else active_start + profile_end_rel + float(config["experiment"]["observation_after_profile_s"])
    att_w = extract_window(rows_by_type["ATT"], active_start, oracle_end)
    pos_w = extract_window(rows_by_type["POS"], active_start, oracle_end)
    xkf1_w = [r for r in extract_window(rows_by_type["XKF1"], active_start, oracle_end) if int(r.get("C", 0)) == 0]
    xkf4_w = [r for r in extract_window(rows_by_type["XKF4"], active_start, oracle_end) if int(r.get("C", 0)) == 0]
    mode_w = extract_window(rows_by_type["MODE"], active_start, oracle_end)
    err_w = extract_window(rows_by_type["ERR"], active_start, oracle_end)
    msg_w = extract_window(rows_by_type["MSG"], active_start, oracle_end)
    rate_w = extract_window(rows_by_type["RATE"], active_start, oracle_end)
    motb_w = extract_window(rows_by_type["MOTB"], active_start, oracle_end)
    rcou_w = extract_window(rows_by_type["RCOU"], active_start, oracle_end)

    angle_max_deg = float(point["angle_max_cd"]) / 100.0
    tolerance = float(config["oracle"]["command_angle_limit_tolerance_deg"])
    command_angles = [math.hypot(float(row.get("roll_deg", 0.0)), float(row.get("pitch_deg", 0.0))) for row in command_profile]
    max_command_angle = max(command_angles, default=None)
    command_touched_limit = max_command_angle is not None and max_command_angle >= angle_max_deg - tolerance
    command_exceeded_limit = max_command_angle is not None and max_command_angle > angle_max_deg + tolerance

    att_errors: list[tuple[float, float]] = []
    for row in att_w:
        rel_t = float(row["time_s"]) - float(active_start or 0.0)
        cmd = command_at(command_profile, rel_t)
        err = math.hypot(
            float(row.get("Roll", 0.0)) - float(cmd.get("roll_deg", 0.0)),
            float(row.get("Pitch", 0.0)) - float(cmd.get("pitch_deg", 0.0)),
        )
        att_errors.append((float(row["time_s"]), err))
    div = sustained_exceed(
        att_errors,
        float(config["oracle"]["divergence_error_deg"]),
        float(config["oracle"]["divergence_duration_s"]),
    )
    tail_start = None if oracle_end is None else oracle_end - float(config["oracle"]["divergence_tail_s"])
    tail_rows = [(t, v) for t, v in att_errors if tail_start is not None and t >= tail_start]
    trend_start = None if oracle_end is None else oracle_end - float(config["oracle"]["divergence_trend_s"])
    trend_rows = [(t, v) for t, v in att_errors if trend_start is not None and t >= trend_start]
    final_error = percentile([v for _, v in tail_rows], 0.50) if tail_rows else (att_errors[-1][1] if att_errors else None)
    tail_slope = slope_deg_s(trend_rows)
    unrecovered_divergence = bool(
        div["ok"]
        and final_error is not None
        and float(final_error) > float(config["oracle"]["divergence_error_deg"])
    )
    worsening_divergence = bool(
        div["ok"]
        and final_error is not None
        and tail_slope is not None
        and float(final_error) >= float(config["oracle"]["divergence_worsening_min_final_deg"])
        and float(tail_slope) >= float(config["oracle"]["divergence_worsening_slope_deg_s"])
    )

    altitude_rows = [(float(r["time_s"]), float(r.get("RelHomeAlt", 0.0))) for r in pos_w if r.get("RelHomeAlt") is not None]
    start_alt = nearest_value(altitude_rows, active_start)
    min_alt = min((v for _, v in altitude_rows), default=None)
    end_alt = nearest_value(altitude_rows, oracle_end)
    altitude_loss = None if start_alt is None or min_alt is None else start_alt - min_alt
    altitude_loss_true = bool(altitude_loss is not None and altitude_loss > float(config["oracle"]["altitude_loss_thresh_m"]))
    altitude_loss_marker = None
    if start_alt is not None:
        for t, alt in altitude_rows:
            loss = start_alt - alt
            if loss > float(config["oracle"]["altitude_loss_thresh_m"]):
                altitude_loss_marker = {"time_s": t, "message": f"altitude loss {loss:.2f} m"}
                break

    vd_rows = [(float(r["time_s"]), float(r.get("VD", 0.0))) for r in xkf1_w if r.get("VD") is not None]
    ground_messages = []
    for row in msg_w:
        text = str(row.get("Message", ""))
        if "sim hit ground" in text.lower():
            ground_messages.append({"time_s": float(row["time_s"]), "message": text})
    low_floor_markers = []
    for t, alt in altitude_rows:
        vd = nearest_value(vd_rows, t)
        if alt < float(config["oracle"]["h_floor_m"]) and vd is not None and vd > float(config["oracle"]["floor_descent_rate_m_s"]):
            low_floor_markers.append({"time_s": t, "rel_home_alt_m": alt, "down_velocity_m_s": vd})
            break
    crash_or_contact = bool(ground_messages or low_floor_markers)

    hard_a_causes = {
        "crash_or_contact": crash_or_contact,
        "unrecovered_divergence": unrecovered_divergence,
        "worsening_divergence": worsening_divergence,
        "altitude_loss": altitude_loss_true,
    }
    hard_a_inside = any(hard_a_causes.values())
    transient_recovered = bool(div["ok"] and not hard_a_inside)
    if hard_a_inside and crash_or_contact:
        outcome = "crashed"
    elif hard_a_inside and (unrecovered_divergence or worsening_divergence):
        outcome = "diverged"
    elif hard_a_inside and altitude_loss_true:
        outcome = "altitude_loss"
    elif transient_recovered:
        outcome = "recovered"
    else:
        outcome = "safe"

    hard_a_markers: list[dict[str, Any]] = []
    hard_a_markers.extend(ground_messages)
    hard_a_markers.extend(low_floor_markers)
    if altitude_loss_true and altitude_loss_marker is not None:
        hard_a_markers.append(altitude_loss_marker)
    if unrecovered_divergence or worsening_divergence:
        hard_a_markers.append({"time_s": oracle_end, "message": f"unrecovered divergence final={fmt(final_error)} slope={fmt(tail_slope)}"})
    hard_a_first = first_time([m for m in hard_a_markers if m.get("time_s") is not None])

    preventive_reasons = set(config["oracle"]["preventive_mode_reasons"])
    harness_reason = str(config["oracle"]["harness_cleanup_mode_reason"])
    mode_preventive = [m for m in mode_w if m.get("reason_name") in preventive_reasons]
    mode_harness = [m for m in mode_w if m.get("reason_name") == harness_reason]
    preventive_subsystems = set(config["oracle"]["preventive_failsafe_subsystems"])
    preventive_errors = [e for e in err_w if e.get("subsystem_name") in preventive_subsystems]
    text_markers = tuple(str(v).lower() for v in config["oracle"]["preventive_text_markers"])
    preventive_texts = []
    for row in msg_w:
        text = str(row.get("Message", ""))
        low = text.lower()
        if any(marker in low for marker in text_markers):
            preventive_texts.append({"time_s": float(row["time_s"]), "message": text})

    all_b_events: list[dict[str, Any]] = []
    for row in mode_preventive:
        all_b_events.append({"time_s": float(row["time_s"]), "kind": "MODE", "detail": row})
    for row in preventive_errors:
        all_b_events.append({"time_s": float(row["time_s"]), "kind": "ERR", "detail": row})
    for row in preventive_texts:
        all_b_events.append({"time_s": float(row["time_s"]), "kind": "MSG", "detail": row})
    if hard_a_first is None:
        b_preventive = bool(all_b_events)
        b_preventive_before_hard_a = bool(all_b_events)
    else:
        b_preventive_before_hard_a = any(float(e["time_s"]) <= hard_a_first for e in all_b_events)
        b_preventive = b_preventive_before_hard_a

    b_clamp = bool(command_touched_limit or command_exceeded_limit)
    command_legal = not b_clamp
    if hard_a_inside and command_legal and not b_preventive:
        label = "clean_unsafe"
    elif hard_a_inside and (b_preventive or b_clamp):
        label = "bug_side"
    elif outcome == "recovered":
        label = "recovered"
    else:
        label = "safe"

    rate_cmd = []
    rate_des = []
    rate_actual = []
    for row in rate_w:
        rel_t = float(row["time_s"]) - float(active_start or 0.0)
        cmd = command_at(command_profile, rel_t)
        rate_cmd.append(abs(float(cmd.get("roll_rate_deg_s", 0.0))) + abs(float(cmd.get("pitch_rate_deg_s", 0.0))))
        rate_des.append(abs(float(row.get("RDes", 0.0))) + abs(float(row.get("PDes", 0.0))))
        rate_actual.append(abs(float(row.get("R", 0.0))) + abs(float(row.get("P", 0.0))))

    thlimit_values = [float(r["ThLimit"]) for r in motb_w if r.get("ThLimit") is not None]
    thr_near = []
    for row in motb_w:
        try:
            thr_out = float(row["ThrOut"])
            thr_av = float(row["ThrAvMx"])
            thr_near.append(thr_av > 0.0 and thr_out >= 0.92 * thr_av)
        except Exception:
            pass
    pwm_values = []
    for row in rcou_w:
        pwm_values.extend([float(v) for k, v in row.items() if k.startswith("C") and isinstance(v, (float, int))])

    result = {
        "bin_path": str(bin_path),
        "csv_path": str(csv_path),
        "oracle_path": str(oracle_path),
        "active_window_s": {"start": active_start, "maneuver_end": maneuver_end, "oracle_end": oracle_end},
        "command": {
            "angle_max_cd": float(point["angle_max_cd"]),
            "angle_max_deg": angle_max_deg,
            "max_command_angle_deg": max_command_angle,
            "command_touched_angle_max": command_touched_limit,
            "command_exceeded_angle_max": command_exceeded_limit,
            "command_rate_p90_deg_s": percentile(rate_cmd, 0.90),
        },
        "path_fidelity": {
            "rate_command_p90_deg_s": percentile(rate_cmd, 0.90),
            "rate_desired_p90_deg_s": percentile(rate_des, 0.90),
            "rate_actual_p90_deg_s": percentile(rate_actual, 0.90),
            "rate_desired_over_command": None if not percentile(rate_cmd, 0.90) else (percentile(rate_des, 0.90) or 0.0) / (percentile(rate_cmd, 0.90) or 1.0),
            "rate_actual_over_command": None if not percentile(rate_cmd, 0.90) else (percentile(rate_actual, 0.90) or 0.0) / (percentile(rate_cmd, 0.90) or 1.0),
        },
        "hardened_oracle_A": {
            "inside": hard_a_inside,
            "outcome": outcome,
            "causes": hard_a_causes,
            "first_hard_a_time_s": hard_a_first,
            "transient_recovered": transient_recovered,
            "max_attitude_error_deg": max((v for _, v in att_errors), default=None),
            "sustained_transient": div,
            "final_error_deg": final_error,
            "tail_slope_deg_s": tail_slope,
            "altitude": {
                "start_m": start_alt,
                "min_m": min_alt,
                "end_m": end_alt,
                "loss_m": altitude_loss,
                "threshold_m": float(config["oracle"]["altitude_loss_thresh_m"]),
            },
            "ground_messages": ground_messages,
            "low_floor_markers": low_floor_markers,
        },
        "oracle_B": {
            "B_clamp": b_clamp,
            "B_preventive": b_preventive,
            "B_preventive_before_hard_A": b_preventive_before_hard_a,
            "B_any_window": bool(all_b_events),
            "B_events": sorted(all_b_events, key=lambda e: float(e["time_s"])),
            "mode_harness_GCS_COMMAND": mode_harness,
            "command_legal": command_legal,
        },
        "authority": {
            "motb_thlimit_max": max(thlimit_values, default=None),
            "motb_thlimit_nonzero_fraction": None if not thlimit_values else statistics.fmean(1.0 if v != 0.0 else 0.0 for v in thlimit_values),
            "throttle_near_limit_fraction": None if not thr_near else statistics.fmean(1.0 if v else 0.0 for v in thr_near),
            "rco_pwm_min": min(pwm_values, default=None),
            "rco_pwm_max": max(pwm_values, default=None),
        },
        "ekf": {
            "fs_flag_max": max((float(r.get("FS", 0.0)) for r in xkf4_w), default=None),
            "max_SV": max((float(r.get("SV", 0.0)) for r in xkf4_w), default=None),
            "max_SP": max((float(r.get("SP", 0.0)) for r in xkf4_w), default=None),
            "max_SH": max((float(r.get("SH", 0.0)) for r in xkf4_w), default=None),
            "max_SM": max((float(r.get("SM", 0.0)) for r in xkf4_w), default=None),
        },
        "samples": {
            "att": len(rows_by_type["ATT"]),
            "rate": len(rows_by_type["RATE"]),
            "guia": len(rows_by_type["GUIA"]),
            "pos": len(rows_by_type["POS"]),
            "xkf1": len(rows_by_type["XKF1"]),
            "xkf4": len(rows_by_type["XKF4"]),
        },
        "modes": mode_w,
        "messages_tail": msg_w[-30:],
        "label": label,
    }
    write_json(oracle_path, result)
    return result


def run_cached(config: dict[str, Any], point: dict[str, Any], partial_path: Path, runs: list[dict[str, Any]], resume: bool) -> dict[str, Any]:
    run_id = run_id_for(config, point)
    if resume:
        for existing in runs:
            if existing.get("run_id") == run_id and not existing.get("error"):
                return existing
    print(f"RUN {run_id}", flush=True)
    run = run_one(config, point)
    runs = [r for r in runs if r.get("run_id") != run_id] + [run]
    write_json(partial_path, {"runs": runs, "updated_at_utc": utc_now()})
    return run


def summarize_cells(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    cells: dict[tuple[str, float, str], list[dict[str, Any]]] = defaultdict(list)
    for run in runs:
        if run.get("error"):
            continue
        point = run["point"]
        cells[(str(point.get("layer")), float(point["r_deg_s"]), str(point.get("model")))].append(run)
    out = []
    for (layer, r_value, model), rows in sorted(cells.items()):
        labels = Counter(row.get("label", "blocked") for row in rows)
        hard = sum(1 for row in rows if row.get("hardened_oracle_A", {}).get("inside"))
        clean = labels.get("clean_unsafe", 0)
        bug = labels.get("bug_side", 0)
        out.append(
            {
                "layer": layer,
                "model": model,
                "r_deg_s": r_value,
                "n": len(rows),
                "hard_A": hard,
                "clean_unsafe": clean,
                "bug_side": bug,
                "recovered": labels.get("recovered", 0),
                "safe": labels.get("safe", 0),
                "p_clean_unsafe": clean / len(rows) if rows else None,
                "p_hard_A": hard / len(rows) if rows else None,
            }
        )
    return out


def decide(config: dict[str, Any], runs: list[dict[str, Any]], cell_summary: list[dict[str, Any]], escalation_exhausted: bool) -> dict[str, Any]:
    min_clean = int(config["oracle"]["robust_min_clean_unsafe_per_cell"])
    robust_clean = [cell for cell in cell_summary if cell["clean_unsafe"] >= min_clean]
    hard_runs = [r for r in runs if not r.get("error") and r.get("hardened_oracle_A", {}).get("inside")]
    hard_bug = [r for r in hard_runs if r.get("label") == "bug_side"]
    if robust_clean:
        return {
            "verdict": "REAL-GAP",
            "reason": f"Hardened oracle A is met with B_preventive=0 in {len(robust_clean)} reproducible cell(s).",
            "robust_clean_cells": robust_clean,
            "hard_bug_side_count": len(hard_bug),
        }
    if hard_runs and hard_bug and len(hard_bug) == len(hard_runs):
        return {
            "verdict": "bug-side",
            "reason": "Every hardened-A run observed so far also has flight-controller preventive B before the hard consequence.",
            "robust_clean_cells": [],
            "hard_bug_side_count": len(hard_bug),
        }
    if escalation_exhausted:
        return {
            "verdict": "SOFT-ORACLE",
            "reason": "The v2 clean-overdraw region does not meet hardened oracle A after the preregistered legal pressure escalation.",
            "robust_clean_cells": [],
            "hard_bug_side_count": len(hard_bug),
        }
    return {
        "verdict": "SOFT-ORACLE",
        "reason": "The representative v2 overdraw cells recovered under hardened oracle A; escalation is required for a true LOC boundary.",
        "robust_clean_cells": [],
        "hard_bug_side_count": len(hard_bug),
    }


def select_trajectory_run(runs: list[dict[str, Any]]) -> dict[str, Any] | None:
    clean = [r for r in runs if r.get("label") == "clean_unsafe"]
    if clean:
        return max(clean, key=lambda r: float(r.get("hardened_oracle_A", {}).get("altitude", {}).get("loss_m") or 0.0))
    recovered = [r for r in runs if r.get("label") == "recovered"]
    if recovered:
        return max(recovered, key=lambda r: float(r.get("hardened_oracle_A", {}).get("max_attitude_error_deg") or 0.0))
    complete = [r for r in runs if not r.get("error")]
    if complete:
        return max(complete, key=lambda r: float(r.get("hardened_oracle_A", {}).get("max_attitude_error_deg") or 0.0))
    return None


def trajectory_series(run: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    bin_path = Path(str(run["bin_path"]))
    profile_path = Path(str(run["command_profile"]["path"]))
    profile = load_json(profile_path)["samples"]
    start = float(run["active_window_s"]["start"])
    end = float(run["active_window_s"]["oracle_end"])
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    att_rows: list[tuple[float, float]] = []
    alt_rows: list[tuple[float, float]] = []
    vd_rows: list[tuple[float, float]] = []
    while True:
        msg = mlog.recv_match(type=["ATT", "POS", "XKF1"], blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        data = msg.to_dict()
        t = time_s(data)
        if t is None or t < start or t > end:
            continue
        typ = msg.get_type()
        if typ == "ATT":
            rel_t = t - start
            cmd = command_at(profile, rel_t)
            err = math.hypot(
                float(data.get("Roll", 0.0)) - float(cmd.get("roll_deg", 0.0)),
                float(data.get("Pitch", 0.0)) - float(cmd.get("pitch_deg", 0.0)),
            )
            att_rows.append((t - start, err))
        elif typ == "POS":
            alt_rows.append((t - start, float(data.get("RelHomeAlt", 0.0))))
        elif typ == "XKF1" and int(data.get("C", 0)) == 0:
            vd_rows.append((t - start, float(data.get("VD", 0.0))))
    common = [t for t, _ in att_rows]
    return {
        "run_id": run["run_id"],
        "label": run.get("label"),
        "time_s": common,
        "att_error_deg": [nearest_value(att_rows, t) or 0.0 for t in common],
        "rel_alt_m": [nearest_value(alt_rows, t) for t in common],
        "down_velocity_m_s": [nearest_value(vd_rows, t) for t in common],
    }


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    stem = artifact_stem(payload["config"])
    runs = [r for r in payload["runs"] if not r.get("error")]
    paths: dict[str, str] = {}

    fig, ax = plt.subplots(figsize=(8, 4.5))
    colors = {"safe": "#2a9d8f", "recovered": "#e9c46a", "altitude_loss": "#f4a261", "clean_unsafe": "#e76f51", "bug_side": "#6d597a"}
    y_labels = ["safe", "recovered", "clean_unsafe", "bug_side"]
    y_index = {name: idx for idx, name in enumerate(y_labels)}
    for run in runs:
        label = str(run.get("label"))
        y = y_index.get(label, 0) + 0.03 * int(run["point"].get("seed", 0))
        ax.scatter(float(run["point"]["r_deg_s"]), y, color=colors.get(label, "#777777"), alpha=0.75, s=35)
    ax.set_yticks(list(y_index.values()))
    ax.set_yticklabels(y_labels)
    ax.set_xlabel("r (deg/s)")
    ax.set_title("Hardened oracle A outcome by r")
    ax.grid(True, axis="x", alpha=0.25)
    path = analysis / f"{stem}_outcomes_vs_r.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["outcomes_vs_r"] = str(path)

    cells = payload["cell_summary"]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for layer in sorted({c["layer"] for c in cells}):
        subset = [c for c in cells if c["layer"] == layer]
        ax.plot([c["r_deg_s"] for c in subset], [c["p_clean_unsafe"] for c in subset], marker="o", label=f"{layer} clean")
        ax.plot([c["r_deg_s"] for c in subset], [c["p_hard_A"] for c in subset], marker="x", linestyle="--", label=f"{layer} hard-A")
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlabel("r (deg/s)")
    ax.set_ylabel("probability")
    ax.set_title("True consequence boundary under hardened oracle A")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    path = analysis / f"{stem}_true_boundary.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["true_boundary"] = str(path)

    series = payload.get("trajectory_series", {})
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ts = series.get("time_s", [])
    axes[0].plot(ts, series.get("att_error_deg", []), color="#e76f51")
    axes[0].axhline(float(payload["config"]["oracle"]["divergence_error_deg"]), color="#222222", linestyle="--", linewidth=1)
    axes[0].set_ylabel("attitude error deg")
    axes[1].plot(ts, series.get("rel_alt_m", []), color="#264653", label="altitude")
    axes[1].set_ylabel("RelHomeAlt m")
    axes[1].set_xlabel("seconds from oracle-window start")
    axes[0].set_title(f"Representative trajectory: {series.get('run_id', 'n/a')} ({series.get('label', 'n/a')})")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    path = analysis / f"{stem}_trajectory.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["trajectory"] = str(path)
    return paths


def build_report(payload: dict[str, Any]) -> str:
    stem = artifact_stem(payload["config"])
    path = PLANC_ROOT / "results" / f"{stem}_report.md"
    verdict = payload["verdict"]
    summary = payload["summary"]
    lines = [
        "# Oracle-A v1 Hardening Probe",
        "",
        f"VERDICT: **{verdict['verdict']}**",
        "",
        verdict["reason"],
        "",
        "## Criteria",
        "",
        f"- Hard A: SITL contact/low-floor descent, unrecovered or worsening >60 deg attitude error, or altitude loss > {payload['config']['oracle']['altitude_loss_thresh_m']} m.",
        "- Recovered transient: >60 deg/0.5s that returns before the oracle window closes is not unsafe.",
        "- B: command clamp or flight-controller preventive failsafe before hard A; `MODE.Rsn=GCS_COMMAND` is excluded as harness.",
        "",
        "## Summary",
        "",
        f"- Completed runs: {summary['completed_runs']}; errors: {summary['error_runs']}.",
        f"- Hard-A runs: {summary['hard_A_runs']}; clean unsafe: {summary['clean_unsafe_runs']}; bug-side: {summary['bug_side_runs']}; recovered transient: {summary['recovered_runs']}; safe: {summary['safe_runs']}.",
        f"- Hardened outcomes: `{json.dumps(summary['outcome_counts'], sort_keys=True)}`.",
        f"- Hard-A causes: `{json.dumps(summary['hard_A_cause_counts'], sort_keys=True)}`.",
        f"- B_preventive before hard A: {summary['B_preventive_runs']}; B_any_window: {summary['B_any_window_runs']}; command clamp: {summary['B_clamp_runs']}.",
        f"- Command margin: max command {fmt(summary['max_command_angle_deg'], 2)} deg vs ANGLE_MAX {fmt(summary['angle_max_deg'], 2)} deg; minimum margin {fmt(summary['min_command_margin_deg'], 2)} deg.",
        f"- Escalation exhausted: {payload['escalation_exhausted']}; boundary scan run: {payload['boundary_scan_run']}.",
        "",
        "## Decision Evidence",
        "",
        "| criterion | result | evidence |",
        "| --- | --- | --- |",
        f"| legal bounded command | {summary['B_clamp_runs'] == 0} | max command angle stays below ANGLE_MAX by the configured guard. |",
        f"| hard consequence | {summary['hard_A_runs'] > 0} | hard-A count {summary['hard_A_runs']} across completed runs. |",
        f"| no preventive guard | {summary['B_preventive_runs'] == 0} | B_preventive before hard A count {summary['B_preventive_runs']}; harness GCS_COMMAND excluded. |",
        f"| cross-seed reproducibility | {bool(verdict.get('robust_clean_cells'))} | robust r cells: `{summary['robust_clean_r_values_deg_s']}`. |",
        "",
        "## Cell Probabilities",
        "",
        "| layer | model | r | n | hard_A | clean_unsafe | bug_side | recovered | safe | p_clean |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for cell in payload["cell_summary"]:
        lines.append(
            f"| {cell['layer']} | {cell['model']} | {fmt(cell['r_deg_s'], 0)} | {cell['n']} | {cell['hard_A']} | {cell['clean_unsafe']} | {cell['bug_side']} | {cell['recovered']} | {cell['safe']} | {fmt(cell['p_clean_unsafe'], 2)} |"
        )
    lines.extend(
        [
            "",
        "## Representative Trajectory",
        "",
        f"- Run: `{payload.get('trajectory_series', {}).get('run_id', 'n/a')}`",
        f"- Label: `{payload.get('trajectory_series', {}).get('label', 'n/a')}`",
        f"- Example clean unsafe: `{summary.get('example_clean_unsafe_run_id')}`; outcome `{summary.get('example_clean_unsafe_outcome')}`; altitude loss {fmt(summary.get('example_clean_unsafe_altitude_loss_m'), 2)} m.",
        "",
            "## Artifacts",
            "",
            f"- Preregistration: `planc/results/{stem}_prereg.json`",
            f"- Result JSON: `planc/results/{stem}_result.json`",
        ]
    )
    for name, plot in payload.get("artifacts", {}).get("plots", {}).items():
        lines.append(f"- Plot {name}: `{Path(plot).relative_to(REPO_ROOT)}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(path)


def summarize_runs(runs: list[dict[str, Any]]) -> dict[str, int]:
    complete = [r for r in runs if not r.get("error")]
    outcome_counts = Counter(r.get("hardened_oracle_A", {}).get("outcome", "unknown") for r in complete)
    cause_counts = Counter()
    for run in complete:
        for name, value in run.get("hardened_oracle_A", {}).get("causes", {}).items():
            if value:
                cause_counts[name] += 1
    margins = []
    command_maxes = []
    angle_maxes = []
    for run in complete:
        cmd = run.get("command", {})
        if cmd.get("max_command_angle_deg") is not None and cmd.get("angle_max_deg") is not None:
            command_maxes.append(float(cmd["max_command_angle_deg"]))
            angle_maxes.append(float(cmd["angle_max_deg"]))
            margins.append(float(cmd["angle_max_deg"]) - float(cmd["max_command_angle_deg"]))
    robust_cells = [cell for cell in summarize_cells(runs) if cell["clean_unsafe"] >= 2]
    example = next((r for r in complete if r.get("label") == "clean_unsafe"), None)
    return {
        "completed_runs": len(complete),
        "error_runs": len([r for r in runs if r.get("error")]),
        "hard_A_runs": len([r for r in complete if r.get("hardened_oracle_A", {}).get("inside")]),
        "clean_unsafe_runs": len([r for r in complete if r.get("label") == "clean_unsafe"]),
        "bug_side_runs": len([r for r in complete if r.get("label") == "bug_side"]),
        "recovered_runs": len([r for r in complete if r.get("label") == "recovered"]),
        "safe_runs": len([r for r in complete if r.get("label") == "safe"]),
        "B_preventive_runs": len([r for r in complete if r.get("oracle_B", {}).get("B_preventive")]),
        "B_any_window_runs": len([r for r in complete if r.get("oracle_B", {}).get("B_any_window")]),
        "B_clamp_runs": len([r for r in complete if r.get("oracle_B", {}).get("B_clamp")]),
        "outcome_counts": dict(outcome_counts),
        "hard_A_cause_counts": dict(cause_counts),
        "max_command_angle_deg": max(command_maxes, default=None),
        "angle_max_deg": max(angle_maxes, default=None),
        "min_command_margin_deg": min(margins, default=None),
        "robust_clean_r_values_deg_s": [cell["r_deg_s"] for cell in robust_cells],
        "example_clean_unsafe_run_id": None if example is None else example.get("run_id"),
        "example_clean_unsafe_outcome": None if example is None else example.get("hardened_oracle_A", {}).get("outcome"),
        "example_clean_unsafe_altitude_loss_m": None
        if example is None
        else example.get("hardened_oracle_A", {}).get("altitude", {}).get("loss_m"),
    }


def build_payload(config: dict[str, Any], env: dict[str, Any], prereg: dict[str, Any], runs: list[dict[str, Any]], *, escalation_exhausted: bool, boundary_scan_run: bool) -> dict[str, Any]:
    cell_summary = summarize_cells(runs)
    verdict = decide(config, runs, cell_summary, escalation_exhausted=escalation_exhausted)
    representative = select_trajectory_run(runs)
    series = trajectory_series(representative, config) if representative is not None and not representative.get("error") else {}
    payload = {
        "status": "complete",
        "generated_at_utc": utc_now(),
        "config": config,
        "env": env,
        "preregistration": prereg,
        "verdict": verdict,
        "summary": summarize_runs(runs),
        "cell_summary": cell_summary,
        "escalation_exhausted": escalation_exhausted,
        "boundary_scan_run": boundary_scan_run,
        "trajectory_series": series,
        "runs": runs,
        "artifacts": {},
    }
    payload["artifacts"]["plots"] = make_plots(payload)
    payload["artifacts"]["report"] = build_report(payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "oracleA_v1_config.yaml")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--core-only", action="store_true")
    args = parser.parse_args()

    config = load_yaml(args.config)
    stem = artifact_stem(config)
    results_dir = PLANC_ROOT / "results"
    prereg_path = results_dir / f"{stem}_prereg.json"
    result_path = results_dir / f"{stem}_result.json"
    env_path = results_dir / f"env_{stem}.json"
    partial_path = results_dir / f"{stem}_partial.json"

    env = probe_environment(config, REPO_ROOT)
    write_env(env, env_path)
    prereg = write_preregister(config, prereg_path)
    partial = load_json(partial_path, {"runs": []}) if args.resume else {"runs": []}
    all_runs: list[dict[str, Any]] = list(partial.get("runs", []))

    core_points = unique_points(points_for_block(config, config["sampling_plan"]["core"], role="grid"), config)
    for point in core_points:
        run = run_cached(config, point, partial_path, all_runs, args.resume)
        all_runs = list(load_json(partial_path, {"runs": []}).get("runs", []))
        print(f"DONE {run.get('run_id')} label={run.get('label')} outcome={run.get('hardened_oracle_A', {}).get('outcome')} error={run.get('error')}", flush=True)

    core_payload = build_payload(config, env, prereg, all_runs, escalation_exhausted=False, boundary_scan_run=False)
    has_real = core_payload["verdict"]["verdict"] == "REAL-GAP"
    boundary_scan_run = False
    escalation_exhausted = False

    if not args.core_only and has_real:
        boundary_points = unique_points(points_for_block(config, config["sampling_plan"]["boundary_if_real"], role="grid"), config)
        boundary_scan_run = True
        for point in boundary_points:
            run = run_cached(config, point, partial_path, all_runs, args.resume)
            all_runs = list(load_json(partial_path, {"runs": []}).get("runs", []))
            print(f"DONE {run.get('run_id')} label={run.get('label')} outcome={run.get('hardened_oracle_A', {}).get('outcome')} error={run.get('error')}", flush=True)
    elif not args.core_only:
        for spec in config["sampling_plan"]["escalation_if_soft"]:
            esc_points = unique_points(points_for_block(config, spec, role="grid"), config)
            for point in esc_points:
                run = run_cached(config, point, partial_path, all_runs, args.resume)
                all_runs = list(load_json(partial_path, {"runs": []}).get("runs", []))
                print(f"DONE {run.get('run_id')} label={run.get('label')} outcome={run.get('hardened_oracle_A', {}).get('outcome')} error={run.get('error')}", flush=True)
            interim = build_payload(config, env, prereg, all_runs, escalation_exhausted=False, boundary_scan_run=False)
            if interim["verdict"]["verdict"] == "REAL-GAP":
                has_real = True
                break
        escalation_exhausted = not has_real

    payload = build_payload(config, env, prereg, all_runs, escalation_exhausted=escalation_exhausted, boundary_scan_run=boundary_scan_run)
    write_json(result_path, payload)
    print(f"COMPLETE: verdict={payload['verdict']['verdict']} result={result_path} report={payload['artifacts']['report']}", flush=True)


if __name__ == "__main__":
    main()
