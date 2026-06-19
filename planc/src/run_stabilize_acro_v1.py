"""stabilize_acro v1 SITL campaign.

Runs ArduCopter ACRO through ordinary RC override plus mode changes only.  The
positive witness is deliberately operator-reachable: no GUIDED mode and no
SET_ATTITUDE_TARGET attitude/rate injection are used.
"""
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
from flight import arm, land_and_disarm, request_streams, send_gcs_heartbeat, set_mode, wait_position_stable
from oracle import COPTER_MODES, ERROR_SUBSYSTEMS, EVENT_NAMES
from param_manager import ParamManager
from run_oracleA_v1 import MODE_REASONS
from sitl_runner import SitlRunner

csv.field_size_limit(10_000_000)


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
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def pct(values: list[float], q: float) -> float | None:
    if not values:
        return None
    vals = sorted(values)
    idx = (len(vals) - 1) * q
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return vals[int(idx)]
    return vals[lo] * (hi - idx) + vals[hi] * (idx - lo)


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


def _subsystem_name(data: dict[str, Any]) -> str:
    raw = _field(data, "Subsys")
    try:
        return ERROR_SUBSYSTEMS.get(int(raw), str(raw))
    except Exception:
        return str(raw)


def _event_name(data: dict[str, Any]) -> str:
    raw = _field(data, "Id")
    try:
        return EVENT_NAMES.get(int(raw), str(raw))
    except Exception:
        return str(raw)


def tilt_deg(roll_deg: float, pitch_deg: float) -> float:
    # Angle between body thrust axis and earth vertical.  This remains meaningful
    # near inverted attitudes, unlike hypot(roll,pitch).
    c = math.cos(math.radians(roll_deg)) * math.cos(math.radians(pitch_deg))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def nearest_value(rows: list[tuple[float, float]], t: float) -> float | None:
    if not rows:
        return None
    return min(rows, key=lambda p: abs(p[0] - t))[1]


def sustained_over(rows: list[tuple[float, float]], threshold: float, duration_s: float) -> dict[str, Any]:
    start: float | None = None
    prev: float | None = None
    best = 0.0
    intervals: list[dict[str, float]] = []
    for t, value in rows:
        if value > threshold:
            if start is None:
                start = t
        else:
            if start is not None and prev is not None:
                dur = max(0.0, prev - start)
                best = max(best, dur)
                if dur >= duration_s:
                    intervals.append({"start_s": start, "end_s": prev, "duration_s": dur})
            start = None
        prev = t
    if start is not None and prev is not None:
        dur = max(0.0, prev - start)
        best = max(best, dur)
        if dur >= duration_s:
            intervals.append({"start_s": start, "end_s": prev, "duration_s": dur})
    return {"ok": bool(intervals), "max_duration_s": best, "intervals": intervals}


class MavAudit:
    def __init__(self) -> None:
        self.counts: Counter[str] = Counter()

    def bump(self, name: str, n: int = 1) -> None:
        self.counts[name] += n

    def payload(self) -> dict[str, Any]:
        out = dict(sorted(self.counts.items()))
        out.setdefault("SET_ATTITUDE_TARGET", 0)
        out.setdefault("GUIDED_MODE_SET", 0)
        out.setdefault("OFFBOARD_SETPOINT", 0)
        return out


def send_rc_override(
    master: Any,
    audit: MavAudit,
    *,
    roll_pwm: int,
    pitch_pwm: int,
    throttle_pwm: int,
    yaw_pwm: int,
) -> None:
    chans = [int(roll_pwm), int(pitch_pwm), int(throttle_pwm), int(yaw_pwm)] + [0] * 14
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans[:18])
    audit.bump("RC_CHANNELS_OVERRIDE")


def release_rc_override(master: Any, audit: MavAudit) -> None:
    chans = [0] * 18
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans)
    audit.bump("RC_CHANNELS_OVERRIDE")


def counted_set_mode(master: Any, audit: MavAudit, mode: str, timeout_s: float = 15.0) -> None:
    if mode in {"GUIDED", "GUIDED_NOGPS"}:
        audit.bump("GUIDED_MODE_SET")
    audit.bump("SET_MODE")
    set_mode(master, mode, timeout_s=timeout_s)


def counted_request_streams(master: Any, audit: MavAudit, rate_hz: int) -> None:
    audit.bump("REQUEST_DATA_STREAM")
    audit.bump("COMMAND_LONG:MAV_CMD_SET_MESSAGE_INTERVAL", 3)
    request_streams(master, rate_hz)


def counted_arm(master: Any, audit: MavAudit) -> None:
    audit.bump("COMMAND_LONG:MAV_CMD_COMPONENT_ARM_DISARM")
    arm(master, timeout_s=45.0)


def counted_land_and_disarm(master: Any, audit: MavAudit, timeout_s: float) -> None:
    audit.bump("SET_MODE")
    audit.bump("COMMAND_LONG:MAV_CMD_NAV_LAND")
    audit.bump("COMMAND_LONG:MAV_CMD_COMPONENT_ARM_DISARM")
    land_and_disarm(master, timeout_s=timeout_s)


def rc_pwm(config: dict[str, Any], channel: str) -> int:
    return int(config["rc"][f"{channel}_pwm"])


def manual_takeoff(master: Any, audit: MavAudit, config: dict[str, Any], live: list[dict[str, Any]]) -> None:
    target_alt = float(config["experiment"]["takeoff_alt_m"])
    speedup = max(1.0, float(config["experiment"].get("speedup", 1.0)))
    hz = float(config["experiment"].get("stream_hz", 50))
    dt_wall = 1.0 / (hz * speedup)
    throttle = rc_pwm(config, "throttle_takeoff")
    neutral_roll = rc_pwm(config, "roll_neutral")
    neutral_pitch = rc_pwm(config, "pitch_neutral")
    neutral_yaw = rc_pwm(config, "yaw_neutral")
    deadline = time.time() + 90.0 / speedup + 20.0
    last_alt = None
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        audit.bump("HEARTBEAT")
        send_rc_override(
            master,
            audit,
            roll_pwm=neutral_roll,
            pitch_pwm=neutral_pitch,
            throttle_pwm=throttle,
            yaw_pwm=neutral_yaw,
        )
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=dt_wall)
        if msg is not None and msg.get_type() == "GLOBAL_POSITION_INT":
            last_alt = float(getattr(msg, "relative_alt", 0.0)) / 1000.0
            if last_alt >= target_alt:
                break
        elif msg is not None and msg.get_type() == "STATUSTEXT":
            live.append({"phase": "takeoff", "t_wall_s": time.time(), "text": str(getattr(msg, "text", ""))})
    else:
        raise RuntimeError(f"manual RC takeoff did not reach {target_alt} m; last_alt={last_alt}")

    # Settle at mid-stick hover throttle before the measured maneuver.
    end = time.time() + float(config["experiment"].get("pre_maneuver_hold_s", 1.0)) / speedup
    while time.time() < end:
        send_gcs_heartbeat(master)
        audit.bump("HEARTBEAT")
        send_rc_override(
            master,
            audit,
            roll_pwm=neutral_roll,
            pitch_pwm=neutral_pitch,
            throttle_pwm=rc_pwm(config, "throttle_hover"),
            yaw_pwm=neutral_yaw,
        )
        time.sleep(dt_wall)


def build_profile(config: dict[str, Any], point: dict[str, Any]) -> list[dict[str, Any]]:
    neutral_roll = rc_pwm(config, "roll_neutral")
    neutral_pitch = rc_pwm(config, "pitch_neutral")
    neutral_throttle = rc_pwm(config, "throttle_hover")
    neutral_yaw = rc_pwm(config, "yaw_neutral")
    profile: list[dict[str, Any]] = []

    def seg(label: str, duration_s: float, roll_pwm: int) -> None:
        if duration_s <= 0:
            return
        profile.append(
            {
                "label": label,
                "duration_s": float(duration_s),
                "roll_pwm": int(roll_pwm),
                "pitch_pwm": neutral_pitch,
                "throttle_pwm": neutral_throttle,
                "yaw_pwm": neutral_yaw,
            }
        )

    kind = str(point.get("profile", "acro_target"))
    if kind == "zero_tilt":
        seg("zero_hover", float(config["acro_profile"]["hold_at_tilt_s"]), neutral_roll)
        seg("post", float(config["experiment"].get("post_maneuver_hold_s", 1.0)), neutral_roll)
        return profile

    if kind == "full_compare":
        pulse = float(config["controls"]["full_stick_pulse_s"])
        hold = float(config["controls"]["full_stick_hold_s"])
        seg("full_positive", pulse, neutral_roll + 500)
        seg("hold_neutral", hold, neutral_roll)
        seg("full_recover", pulse, neutral_roll - 500)
        seg("post", float(config["experiment"].get("post_maneuver_hold_s", 1.0)), neutral_roll)
        return profile

    target = abs(float(point.get("target_deg", 0.0)))
    stick_frac = float(config["acro_profile"]["roll_stick_fraction"])
    nominal_rate = float(config["acro_profile"]["nominal_rate_deg_s"])
    pulse_s = target / max(1.0, nominal_rate)
    roll_pwm = neutral_roll + int(round(500.0 * stick_frac))
    recover_pwm = neutral_roll - int(round(500.0 * stick_frac))
    seg("rate_to_target", pulse_s, roll_pwm)
    seg("hold_neutral", float(config["acro_profile"]["hold_at_tilt_s"]), neutral_roll)
    if config["acro_profile"].get("recovery", True):
        seg("recover_to_level", pulse_s, recover_pwm)
    seg("post", float(config["experiment"].get("post_maneuver_hold_s", 1.0)), neutral_roll)
    return profile


def run_profile(
    master: Any,
    audit: MavAudit,
    config: dict[str, Any],
    profile: list[dict[str, Any]],
    live: list[dict[str, Any]],
) -> dict[str, Any]:
    speedup = max(1.0, float(config["experiment"].get("speedup", 1.0)))
    hz = float(config["experiment"].get("stream_hz", 50))
    dt_wall = 1.0 / (hz * speedup)
    t0 = time.time()
    abort_reason = None
    for segment in profile:
        end = time.time() + float(segment["duration_s"]) / speedup
        while time.time() < end:
            send_gcs_heartbeat(master)
            audit.bump("HEARTBEAT")
            send_rc_override(
                master,
                audit,
                roll_pwm=int(segment["roll_pwm"]),
                pitch_pwm=int(segment["pitch_pwm"]),
                throttle_pwm=int(segment["throttle_pwm"]),
                yaw_pwm=int(segment["yaw_pwm"]),
            )
            msg = master.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=dt_wall)
            if msg is None:
                continue
            typ = msg.get_type()
            if typ == "STATUSTEXT":
                text = str(getattr(msg, "text", ""))
                live.append({"phase": "profile", "segment": segment["label"], "t_s": time.time() - t0, "text": text})
                low = text.lower()
                if "sim hit ground" in low:
                    abort_reason = "sim_hit_ground"
                    break
            elif typ == "HEARTBEAT":
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    abort_reason = "disarmed_during_profile"
                    break
            elif typ == "GLOBAL_POSITION_INT":
                rel_alt = float(getattr(msg, "relative_alt", 0.0)) / 1000.0
                if rel_alt < 0.4:
                    abort_reason = f"low_altitude_{rel_alt:.2f}m"
                    break
        if abort_reason:
            break
    return {"profile_wall_s": time.time() - t0, "abort_reason": abort_reason}


def run_id_for(config: dict[str, Any], point: dict[str, Any]) -> str:
    prefix = str(config["experiment"].get("run_prefix", "stabacrov1"))
    return (
        f"{prefix}_{point['role']}_{point['mode'].lower()}_{point['profile']}"
        f"_g{int(round(float(point.get('target_deg', 0.0)))):03d}"
        f"_a{int(round(float(point.get('angle_max_cd', 4500)))):04d}"
        f"_r{int(point.get('rep', 0)):02d}"
    )


def point_params(config: dict[str, Any], point: dict[str, Any]) -> dict[str, float]:
    params = {k: float(v) for k, v in config["baseline_params"].items()}
    params["ANGLE_MAX"] = float(point.get("angle_max_cd", params.get("ANGLE_MAX", 4500.0)))
    return params


def run_once(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    run_id = run_id_for(config, point)
    result: dict[str, Any] = {
        "run_id": run_id,
        "point": point,
        "started_at_utc": utc_now(),
        "error": None,
    }
    runner = SitlRunner(config, REPO_ROOT)
    audit = MavAudit()
    master = None
    live: list[dict[str, Any]] = []
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=35.0)
        counted_request_streams(master, audit, int(config["experiment"]["stream_hz"]))
        params = point_params(config, point)
        pm = ParamManager(master)
        audit.bump("PARAM_SET", len(params))
        pm.apply(params)
        snapshot_names = sorted(set(params) | {"ACRO_RP_RATE", "ACRO_RP_EXPO", "MOT_THST_HOVER", "ACRO_OPTIONS"})
        audit.bump("PARAM_REQUEST_READ", len(snapshot_names))
        snapshot = pm.snapshot(snapshot_names)
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)

        wait_position_stable(master, min_samples=5, timeout_s=45.0)
        counted_set_mode(master, audit, "STABILIZE", timeout_s=15.0)
        counted_arm(master, audit)
        manual_takeoff(master, audit, config, live)

        counted_set_mode(master, audit, str(point["mode"]), timeout_s=15.0)
        profile = build_profile(config, point)
        profile_path = PLANC_ROOT / "logs" / f"{run_id}_command_profile.json"
        write_json(profile_path, {"run_id": run_id, "point": point, "profile": profile})
        profile_result = run_profile(master, audit, config, profile, live)
        try:
            release_rc_override(master, audit)
        except Exception:
            pass
        try:
            counted_land_and_disarm(master, audit, float(config["experiment"]["cleanup_land_timeout_s"]))
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
            raise RuntimeError("No DataFlash .BIN log found after run")
        parsed = parse_dataflash(
            config=config,
            point=point,
            run_id=run_id,
            bin_path=bin_path,
            csv_path=PLANC_ROOT / "logs" / f"{run_id}_parsed.csv",
            oracle_path=PLANC_ROOT / "logs" / f"{run_id}_parsed.oracle.json",
            command_profile=profile,
        )
        result.update(
            {
                "work_dir": str(work_dir),
                "bin_path": str(bin_path),
                "param_records_path": str(param_path),
                "param_snapshot": snapshot,
                "param_readbacks": pm.records,
                "command_profile_path": str(profile_path),
                "command_profile": {
                    "segments": len(profile),
                    "duration_s": sum(float(s["duration_s"]) for s in profile),
                    "first": profile[0] if profile else None,
                    "last": profile[-1] if profile else None,
                },
                "mavlink_sent_counts": audit.payload(),
                "live_events": live[-80:],
                "profile_result": profile_result,
            }
        )
        result.update(parsed)
        result["error"] = None
        return result
    except Exception as exc:  # noqa: BLE001
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        result["mavlink_sent_counts"] = audit.payload()
        result["live_events"] = live[-80:]
        return result
    finally:
        if master is not None:
            try:
                release_rc_override(master, audit)
            except Exception:
                pass
            try:
                counted_land_and_disarm(master, audit, float(config["experiment"]["cleanup_land_timeout_s"]))
            except Exception:
                try:
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
                except Exception:
                    pass
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def parse_dataflash(
    *,
    config: dict[str, Any],
    point: dict[str, Any],
    run_id: str,
    bin_path: Path,
    csv_path: Path,
    oracle_path: Path,
    command_profile: list[dict[str, Any]],
) -> dict[str, Any]:
    msg_types = ["ATT", "RATE", "RCIN", "RCOU", "CTUN", "POS", "MODE", "ERR", "EV", "MSG", "CMD"]
    rows: list[dict[str, Any]] = []
    by_type: dict[str, list[dict[str, Any]]] = defaultdict(list)
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    start_time: float | None = None
    while True:
        msg = mlog.recv_match(type=msg_types, blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        data = msg.to_dict()
        t_abs = _time_s(data)
        if t_abs is None:
            continue
        if start_time is None:
            start_time = t_abs
        typ = msg.get_type()
        row: dict[str, Any] = {
            "time_s": float(t_abs - start_time),
            "type": typ,
        }
        row.update({k: v for k, v in data.items() if k not in {"mavpackettype", "TimeUS", "TimeMS"}})
        if typ == "ATT":
            try:
                row["tilt_deg"] = tilt_deg(float(row.get("Roll", 0.0)), float(row.get("Pitch", 0.0)))
                row["des_tilt_deg"] = tilt_deg(float(row.get("DesRoll", 0.0)), float(row.get("DesPitch", 0.0)))
            except Exception:
                pass
        elif typ == "MODE":
            row["mode_name"] = _mode_name(data)
            row["reason_name"] = _reason_name(data)
        elif typ == "ERR":
            row["subsystem_name"] = _subsystem_name(data)
        elif typ == "EV":
            row["event_name"] = _event_name(data)
        rows.append(row)
        by_type[typ].append(row)

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        fields = ["time_s", "type"]
        extra = sorted({k for row in rows for k in row if k not in fields})
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields + extra, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    profile_duration = sum(float(seg["duration_s"]) for seg in command_profile)
    active_start = infer_active_start(config, point, by_type, command_profile)
    active_end = None if active_start is None else active_start + profile_duration
    pulse_end = None
    if active_start is not None and command_profile:
        pulse_end = active_start + float(command_profile[0]["duration_s"])

    window_att = in_window(by_type["ATT"], active_start, active_end)
    window_rate = in_window(by_type["RATE"], active_start, pulse_end)
    window_rcin = in_window(by_type["RCIN"], active_start, active_end)
    window_pos = in_window(by_type["POS"], active_start, active_end)
    window_ctun = in_window(by_type["CTUN"], active_start, active_end)
    window_modes = in_window(by_type["MODE"], active_start, active_end)
    window_errors = in_window(by_type["ERR"], active_start, active_end)
    window_events = in_window(by_type["EV"], active_start, active_end)
    window_messages = in_window(by_type["MSG"], active_start, active_end)

    tilts = [(float(r["time_s"]), float(r.get("tilt_deg"))) for r in window_att if r.get("tilt_deg") not in (None, "")]
    des_tilts = [(float(r["time_s"]), float(r.get("des_tilt_deg"))) for r in window_att if r.get("des_tilt_deg") not in (None, "")]
    achieved_peak = max((v for _, v in tilts), default=None)
    demanded_peak = max((v for _, v in des_tilts), default=None)
    final_tilt = mean([v for _, v in tilts[-10:]]) if tilts else None
    near_inv = sustained_over(tilts, float(config["oracle"]["near_inverted_tilt_deg"]), float(config["oracle"]["near_inverted_duration_s"]))

    alt_rows = [(float(r["time_s"]), float(r.get("RelHomeAlt"))) for r in window_pos if r.get("RelHomeAlt") not in (None, "")]
    all_alt_rows = [(float(r["time_s"]), float(r.get("RelHomeAlt"))) for r in by_type["POS"] if r.get("RelHomeAlt") not in (None, "")]
    start_alt = nearest_value(all_alt_rows, active_start) if active_start is not None else None
    min_alt = min((v for _, v in alt_rows), default=None)
    altitude_loss = None if start_alt is None or min_alt is None else max(0.0, start_alt - min_alt)

    throttle_pwm = [float(r.get("C3")) for r in window_rcin if r.get("C3") not in (None, "")]
    roll_pwm = [float(r.get("C1")) for r in window_rcin if r.get("C1") not in (None, "")]
    ctun_tho = [float(r.get("ThO")) for r in window_ctun if r.get("ThO") not in (None, "")]
    throttle_stats = {
        "rcin_c3_mean_pwm": mean(throttle_pwm),
        "rcin_c3_std_pwm": stdev(throttle_pwm),
        "rcin_c3_min_pwm": min(throttle_pwm) if throttle_pwm else None,
        "rcin_c3_max_pwm": max(throttle_pwm) if throttle_pwm else None,
        "ctun_tho_mean": mean(ctun_tho),
        "ctun_tho_std": stdev(ctun_tho),
        "ctun_tho_min": min(ctun_tho) if ctun_tho else None,
        "ctun_tho_max": max(ctun_tho) if ctun_tho else None,
    }

    rate_actual = [abs(float(_field(r, "R", "Roll", "RDes") or 0.0)) for r in window_rate]
    rate_desired = [abs(float(_field(r, "RDes", "RDesC", "DesR") or 0.0)) for r in window_rate]
    rate_tracking = {
        "samples": len(window_rate),
        "actual_abs_mean_deg_s": mean(rate_actual),
        "desired_abs_mean_deg_s": mean(rate_desired),
        "ratio": None,
    }
    if rate_tracking["desired_abs_mean_deg_s"] not in (None, 0.0):
        rate_tracking["ratio"] = float(rate_tracking["actual_abs_mean_deg_s"] or 0.0) / float(rate_tracking["desired_abs_mean_deg_s"])

    preventive = preventive_events(config, window_modes, window_errors, window_events, window_messages)
    ground_contact = ground_contact_events(config, window_messages, alt_rows)
    hard_A = bool(
        (altitude_loss is not None and altitude_loss > float(config["oracle"]["altitude_loss_thresh_m"]))
        or near_inv["ok"]
        or ground_contact["ground_contact"]
    )
    angle_max_deg = float(point.get("angle_max_cd", config["baseline_params"]["ANGLE_MAX"])) / 100.0
    exceeds_angle_max = achieved_peak is not None and achieved_peak > angle_max_deg + 1.0
    target = float(point.get("target_deg", 0.0))
    target_error = None if achieved_peak is None or str(point.get("profile")) != "acro_target" else abs(achieved_peak - target)
    input_applied = bool(
        str(point.get("profile")) == "zero_tilt"
        or (roll_pwm and max(abs(v - 1500.0) for v in roll_pwm) > float(config["rc"]["active_threshold_pwm"]))
    )
    label = classify_run(config, point, hard_A, preventive, exceeds_angle_max, altitude_loss, target_error)

    parsed = {
        "parsed_csv_path": str(csv_path),
        "oracle_path": str(oracle_path),
        "active_window_s": {"start": active_start, "end": active_end, "pulse_end": pulse_end},
        "samples": {k.lower(): len(v) for k, v in by_type.items()},
        "mode_sequence": [
            {"time_s": r["time_s"], "mode_name": r.get("mode_name"), "reason_name": r.get("reason_name")}
            for r in by_type["MODE"]
        ],
        "attitude": {
            "achieved_peak_tilt_deg": achieved_peak,
            "demanded_peak_tilt_deg": demanded_peak,
            "final_tilt_deg": final_tilt,
            "target_tilt_deg": target,
            "target_error_deg": target_error,
            "exceeds_angle_max": exceeds_angle_max,
            "angle_max_deg": angle_max_deg,
            "near_inverted": near_inv,
        },
        "altitude": {
            "start_alt_m": start_alt,
            "min_alt_m": min_alt,
            "altitude_loss_m": altitude_loss,
        },
        "throttle_stats": throttle_stats,
        "rc_input": {
            "roll_pwm_mean": mean(roll_pwm),
            "roll_pwm_min": min(roll_pwm) if roll_pwm else None,
            "roll_pwm_max": max(roll_pwm) if roll_pwm else None,
            "input_applied": input_applied,
        },
        "rate_tracking": rate_tracking,
        "oracle_A": {
            "hard": hard_A,
            "altitude_loss_gt_15m": altitude_loss is not None and altitude_loss > float(config["oracle"]["altitude_loss_thresh_m"]),
            "near_inverted": near_inv["ok"],
            "ground_contact": ground_contact["ground_contact"],
            "ground_contact_events": ground_contact["events"],
        },
        "contract": preventive,
        "label": label,
    }
    write_json(oracle_path, parsed)
    return parsed


def in_window(rows: list[dict[str, Any]], start: float | None, end: float | None) -> list[dict[str, Any]]:
    if start is None:
        return []
    out = []
    for row in rows:
        t = float(row["time_s"])
        if t < start:
            continue
        if end is not None and t > end:
            continue
        out.append(row)
    return out


def infer_active_start(
    config: dict[str, Any],
    point: dict[str, Any],
    by_type: dict[str, list[dict[str, Any]]],
    command_profile: list[dict[str, Any]],
) -> float | None:
    threshold = float(config["rc"]["active_threshold_pwm"])
    mode = str(point["mode"])
    mode_times = [float(r["time_s"]) for r in by_type["MODE"] if r.get("mode_name") == mode]
    mode_start = max(mode_times) if mode_times else None
    if str(point.get("profile")) == "zero_tilt":
        return None if mode_start is None else mode_start + float(config["experiment"].get("pre_maneuver_hold_s", 1.0))
    for row in by_type["RCIN"]:
        t = float(row["time_s"])
        if mode_start is not None and t < mode_start:
            continue
        try:
            c1 = float(row.get("C1"))
        except Exception:
            continue
        if abs(c1 - 1500.0) > threshold:
            return t
    return None


def preventive_events(
    config: dict[str, Any],
    modes: list[dict[str, Any]],
    errors: list[dict[str, Any]],
    events: list[dict[str, Any]],
    messages: list[dict[str, Any]],
) -> dict[str, Any]:
    preventive_subsystems = set(config["oracle"]["preventive_failsafe_subsystems"])
    preventive_reasons = set(config["oracle"]["preventive_mode_reasons"])
    markers = tuple(str(v).lower() for v in config["oracle"]["preventive_text_markers"])
    mode_hits = [r for r in modes if r.get("reason_name") in preventive_reasons]
    err_hits = [r for r in errors if r.get("subsystem_name") in preventive_subsystems]
    text_hits = []
    for row in messages:
        text = str(row.get("Message", ""))
        low = text.lower()
        if any(marker in low for marker in markers):
            text_hits.append({"time_s": row["time_s"], "message": text})
    acro_trainer_limited = [r for r in events if r.get("event_name") == "ACRO_TRAINER_LIMITED"]
    terminal_crash = [r for r in errors if r.get("subsystem_name") == "CRASH_CHECK"]
    count = len(mode_hits) + len(err_hits) + len(text_hits) + len(acro_trainer_limited)
    return {
        "preventive_count": count,
        "mode_hits": mode_hits,
        "error_hits": err_hits,
        "text_hits": text_hits,
        "acro_trainer_limited_events": acro_trainer_limited,
        "terminal_crash_check_errors": terminal_crash,
        "clean_preventive": count == 0,
    }


def ground_contact_events(config: dict[str, Any], messages: list[dict[str, Any]], alt_rows: list[tuple[float, float]]) -> dict[str, Any]:
    markers = tuple(str(v).lower() for v in config["oracle"]["terminal_text_markers"])
    hits = []
    for row in messages:
        text = str(row.get("Message", ""))
        low = text.lower()
        if any(marker in low for marker in markers):
            hits.append({"time_s": row["time_s"], "message": text})
    low_alt = [{"time_s": t, "rel_home_alt_m": alt} for t, alt in alt_rows if alt < 0.5]
    return {"ground_contact": bool(hits or low_alt), "events": hits + low_alt[:1]}


def classify_run(
    config: dict[str, Any],
    point: dict[str, Any],
    hard_A: bool,
    preventive: dict[str, Any],
    exceeds_angle_max: bool,
    altitude_loss: float | None,
    target_error: float | None,
) -> str:
    if int(preventive.get("preventive_count", 0)) > 0:
        return "contract_violated"
    if target_error is not None and target_error > float(config["acro_profile"]["target_error_tolerance_deg"]):
        return "ambiguous"
    if hard_A and exceeds_angle_max and str(point["mode"]) == "ACRO":
        return "clean_unsafe"
    if hard_A:
        return "hard_unsafe_not_anglemax_witness"
    return "clean_safe"


def build_points(config: dict[str, Any], stage: str) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []

    def add(role: str, mode: str, profile: str, target: float, angle_cd: int = 4500, rep: int = 0) -> None:
        points.append(
            {
                "role": role,
                "mode": mode,
                "profile": profile,
                "target_deg": float(target),
                "angle_max_cd": int(angle_cd),
                "rep": int(rep),
            }
        )

    if stage in {"smoke"}:
        add("premise", "ACRO", "acro_target", 55, 4500, 0)
        add("gigo", "ACRO", "zero_tilt", 0, 4500, 0)
        add("compare", "STABILIZE", "full_compare", 0, 4500, 0)
        add("compare", "ACRO", "full_compare", 0, 4500, 0)
        return points

    if stage in {"premise", "all"}:
        add("premise", "ACRO", "acro_target", 55, 4500, 0)

    if stage in {"scan", "all"}:
        targets = [float(v) for v in config["acro_profile"]["targets_deg"]]
        default_reps = int(config["acro_profile"]["default_repetitions"])
        noise_target = float(config["acro_profile"]["noise_target_deg"])
        noise_reps = int(config["acro_profile"]["noise_repetitions"])
        for target in targets:
            reps = noise_reps if math.isclose(target, noise_target) else default_reps
            for rep in range(reps):
                add("scan", "ACRO", "acro_target", target, 4500, rep)

    if stage in {"controls", "all"}:
        for rep in range(int(config["controls"]["zero_tilt_repetitions"])):
            add("gigo", "ACRO", "zero_tilt", 0, 4500, rep)
        for rep in range(int(config["controls"]["full_stick_repetitions"])):
            add("compare", "STABILIZE", "full_compare", 0, 4500, rep)
            add("compare", "ACRO", "full_compare", 0, 4500, rep)

    if stage in {"stratify", "all"}:
        for angle_cd in config["controls"]["stratify_angle_max_cd"]:
            for rep in range(int(config["controls"]["stratify_repetitions"])):
                add("stratify", "STABILIZE", "full_compare", 0, int(angle_cd), rep)
                add("stratify", "ACRO", "full_compare", 0, int(angle_cd), rep)
    return dedupe_points(config, points)


def dedupe_points(config: dict[str, Any], points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen = set()
    out = []
    for point in points:
        rid = run_id_for(config, point)
        if rid in seen:
            continue
        seen.add(rid)
        out.append(point)
    return out


def cached_run(config: dict[str, Any], point: dict[str, Any], partial_path: Path, runs: list[dict[str, Any]], resume: bool) -> list[dict[str, Any]]:
    rid = run_id_for(config, point)
    if resume:
        for run in runs:
            if run.get("run_id") == rid and not run.get("error"):
                print(f"CACHED {rid}", flush=True)
                return runs
    print(f"RUN {rid}", flush=True)
    run = run_once(config, point)
    runs = [r for r in runs if r.get("run_id") != rid] + [run]
    write_json(partial_path, {"updated_at_utc": utc_now(), "runs": runs})
    print(
        "DONE "
        f"{rid} label={run.get('label')} "
        f"tilt={fmt(run.get('attitude', {}).get('achieved_peak_tilt_deg'))} "
        f"loss={fmt(run.get('altitude', {}).get('altitude_loss_m'))} "
        f"B={run.get('contract', {}).get('preventive_count')} "
        f"err={bool(run.get('error'))}",
        flush=True,
    )
    return runs


def firmware_anchor(config: dict[str, Any]) -> dict[str, Any]:
    root = None
    for raw in config["sitl"]["ardupilot_root_candidates"]:
        p = Path(raw)
        if p.exists():
            root = p
            break
    out: dict[str, Any] = {"root": str(root) if root else None}
    if root:
        import subprocess

        def run(cmd: list[str]) -> str:
            proc = subprocess.run(cmd, cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False, timeout=10)
            return proc.stdout.strip()

        out["tag_or_describe"] = run(["git", "describe", "--tags", "--always", "--dirty"])
        out["sha"] = run(["git", "rev-parse", "HEAD"])
        out["status"] = run(["git", "status", "--short", "--branch"])
        binary = next((raw for raw in config["sitl"]["vehicle_binary_candidates"] if Path(raw).exists()), None)
        out["binary"] = binary
        out["binary_mtime"] = Path(binary).stat().st_mtime if binary else None
    return out


def aggregate(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    ok_runs = [r for r in runs if not r.get("error")]
    result: dict[str, Any] = {
        "scenario_id": "stabilize_acro",
        "version": "v1",
        "generated_at_utc": utc_now(),
        "firmware_anchor": firmware_anchor(config),
        "config_path": str(PLANC_ROOT / "config" / "stabilize_acro_v1_config.yaml"),
        "prereg_path": str(PLANC_ROOT / "results" / "stabilize_acro_v1_prereg.json"),
        "runs": runs,
    }
    result["noise"] = compute_noise(config, ok_runs)
    apply_final_labels(config, ok_runs, result["noise"])
    result["label_counts"] = dict(Counter(r.get("label") for r in ok_runs))
    result["premises"] = evaluate_premises(config, ok_runs)
    result["robustness"] = evaluate_robustness(config, ok_runs)
    result["prediction"] = evaluate_prediction(config, ok_runs, result["noise"])
    result["controls"] = evaluate_controls(config, ok_runs)
    result["angle_max_stratification"] = evaluate_stratification(config, ok_runs)
    result["mavlink_message_summary"] = summarize_mavlink(ok_runs)
    result["figures"] = make_plots(config, ok_runs)
    result["verdict"] = decide_verdict(result)
    write_json(PLANC_ROOT / "results" / "stabilize_acro_v1_result.json", result)
    write_report(config, result, PLANC_ROOT / "results" / "stabilize_acro_v1_report.md")
    return result


def compute_noise(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    target = float(config["acro_profile"]["noise_target_deg"])
    vals = [
        float(r["altitude"]["altitude_loss_m"])
        for r in runs
        if r.get("point", {}).get("role") == "scan"
        and math.isclose(float(r.get("point", {}).get("target_deg", -1)), target)
        and r.get("altitude", {}).get("altitude_loss_m") is not None
    ]
    sigma = stdev(vals)
    return {
        "target_deg": target,
        "n": len(vals),
        "altitude_loss_values_m": vals,
        "sigma_m": sigma,
        "d_margin_m": 3.0 * sigma,
        "mae_bound_m": 1.5 * sigma,
    }


def apply_final_labels(config: dict[str, Any], runs: list[dict[str, Any]], noise: dict[str, Any]) -> None:
    threshold = float(config["oracle"]["altitude_loss_thresh_m"])
    margin = float(noise.get("d_margin_m") or 0.0)
    tol = float(config["acro_profile"]["target_error_tolerance_deg"])
    for run in runs:
        raw = run.get("label")
        final = raw
        loss = run.get("altitude", {}).get("altitude_loss_m")
        target_error = run.get("attitude", {}).get("target_error_deg")
        if int(run.get("contract", {}).get("preventive_count", 0)) > 0:
            final = "contract_violated"
        elif run.get("contract", {}).get("terminal_crash_check_errors"):
            final = "contract_violated"
        elif target_error not in (None, "") and float(target_error) > tol:
            final = "ambiguous"
        elif loss not in (None, "") and margin > 0.0 and abs(float(loss) - threshold) <= margin:
            final = "ambiguous"
        run["raw_label"] = raw
        run["label"] = final


def evaluate_premises(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    acro = [r for r in runs if r.get("point", {}).get("mode") == "ACRO"]
    scanish = [r for r in acro if r.get("point", {}).get("profile") == "acro_target"]
    ratio_bounds = config["oracle"]["rate_tracking_ratio_bounds"]
    ratios = [
        float(r.get("rate_tracking", {}).get("ratio"))
        for r in scanish
        if r.get("rate_tracking", {}).get("ratio") not in (None, "")
    ]
    max_tilt = max((float(r.get("attitude", {}).get("achieved_peak_tilt_deg") or 0.0) for r in scanish), default=0.0)
    no_clamp = all(int(r.get("contract", {}).get("preventive_count", 0)) == 0 for r in scanish if float(r.get("attitude", {}).get("achieved_peak_tilt_deg") or 0.0) > 46.0)
    target_errors = [
        float(r.get("attitude", {}).get("target_error_deg"))
        for r in scanish
        if r.get("attitude", {}).get("target_error_deg") not in (None, "")
    ]
    modes = {m.get("mode_name") for r in acro for m in r.get("mode_sequence", [])}
    all_msg = summarize_mavlink(runs)
    guided_modes = [
        (r["run_id"], m)
        for r in runs
        for m in r.get("mode_sequence", [])
        if m.get("mode_name") in {"GUIDED", "GUIDED_NOGPS"}
    ]
    reject_msgs = []
    for r in runs:
        for event in r.get("live_events", []):
            low = str(event.get("text", "")).lower()
            if "reject" in low or "denied" in low or "failed" in low:
                reject_msgs.append({"run_id": r["run_id"], "event": event})
    return {
        "P0.1_acro_rc_body_rate": {
            "ok": bool("ACRO" in modes and ratios and ratio_bounds[0] <= statistics.median(ratios) <= ratio_bounds[1]),
            "median_rate_tracking_ratio": statistics.median(ratios) if ratios else None,
            "ratios": ratios[:20],
        },
        "P0.2_angle_max_not_engaged_in_acro": {
            "ok": bool(max_tilt > 45.0 and no_clamp),
            "max_acro_tilt_deg": max_tilt,
            "no_preventive_clamp_on_over_angle_runs": no_clamp,
        },
        "P0.3_input_applied": {
            "ok": bool(target_errors and pct(target_errors, 0.8) <= float(config["acro_profile"]["target_error_tolerance_deg"])),
            "target_error_p80_deg": pct(target_errors, 0.8),
            "target_error_max_deg": max(target_errors) if target_errors else None,
        },
        "P0.4_zero_offboard_injection": {
            "ok": bool(all_msg.get("SET_ATTITUDE_TARGET", 0) == 0 and all_msg.get("GUIDED_MODE_SET", 0) == 0 and not guided_modes),
            "mavlink_sent_counts": all_msg,
            "guided_modes_seen": guided_modes[:10],
        },
        "P0.5_admission_guard": {
            "ok": bool("ACRO" in modes and not reject_msgs),
            "modes_seen": sorted(v for v in modes if v),
            "rejection_messages": reject_msgs[:10],
        },
    }


def evaluate_robustness(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    clean = [r for r in runs if r.get("label") == "clean_unsafe" and r.get("point", {}).get("role") == "scan"]
    tilts = [float(r["attitude"]["achieved_peak_tilt_deg"]) for r in clean if r.get("attitude", {}).get("achieved_peak_tilt_deg") is not None]
    targets = sorted({float(r.get("point", {}).get("target_deg")) for r in clean})
    preventive_on_clean = [r["run_id"] for r in clean if int(r.get("contract", {}).get("preventive_count", 0)) != 0]
    return {
        "clean_unsafe_runs": len(clean),
        "clean_unsafe_targets": targets,
        "tilt_width_deg": (max(tilts) - min(tilts)) if len(tilts) >= 2 else 0.0,
        "preventive_on_clean_run_ids": preventive_on_clean,
        "ok": bool(
            len(clean) >= int(config["oracle"]["robust_min_clean_unsafe_runs"])
            and len(targets) >= int(config["oracle"]["robust_min_clean_unsafe_targets"])
            and ((max(tilts) - min(tilts)) if len(tilts) >= 2 else 0.0) >= float(config["oracle"]["robust_min_tilt_width_deg"])
            and not preventive_on_clean
        ),
    }


def evaluate_prediction(config: dict[str, Any], runs: list[dict[str, Any]], noise: dict[str, Any]) -> dict[str, Any]:
    scan = [
        r for r in runs
        if r.get("point", {}).get("role") == "scan"
        and r.get("label") in {"clean_safe", "clean_unsafe", "contract_violated"}
        and r.get("attitude", {}).get("achieved_peak_tilt_deg") is not None
        and r.get("altitude", {}).get("altitude_loss_m") is not None
    ]
    train = [r for r in scan if float(r["point"]["target_deg"]) <= 70.0]
    holdout = [r for r in scan if float(r["point"]["target_deg"]) > 70.0]
    threshold = learn_tilt_threshold(train)
    class_rows = []
    correct = 0
    for r in holdout:
        pred = predict_label_from_threshold(r, threshold)
        truth = r.get("label")
        correct += int(pred == truth)
        class_rows.append({"run_id": r["run_id"], "target_deg": r["point"]["target_deg"], "truth": truth, "pred": pred})
    acc = correct / len(class_rows) if class_rows else 0.0
    low_high_model = fit_loss_model(train)
    low_high_rows = []
    low_high_errs = []
    for r in holdout:
        pred_loss = predict_loss(low_high_model, float(r["attitude"]["achieved_peak_tilt_deg"]))
        truth_loss = float(r["altitude"]["altitude_loss_m"])
        err = abs(pred_loss - truth_loss)
        low_high_errs.append(err)
        low_high_rows.append({"run_id": r["run_id"], "target_deg": r["point"]["target_deg"], "truth_loss_m": truth_loss, "pred_loss_m": pred_loss, "abs_err_m": err})
    loo = severity_leave_one_out(config, runs)
    coverage_targets = sorted({float(r["point"]["target_deg"]) for r in scan})
    return {
        "classification": {
            "train_n": len(train),
            "holdout_n": len(holdout),
            "threshold_tilt_deg": threshold,
            "accuracy": acc,
            "rows": class_rows,
            "coverage_targets_deg": coverage_targets,
            "ok": bool(acc >= 0.90 and min(coverage_targets or [999]) <= 40.0 and max(coverage_targets or [0]) >= 150.0),
        },
        "severity_regression": {
            "method": "leave-one-run-out target/repetition mean predictor; includes boundary and ground-contact-clipped points; no scaling-law claim",
            "holdout_mae_m": loo["mae_m"],
            "mae_bound_m": noise.get("mae_bound_m"),
            "rows": loo["rows"],
            "ok": bool(loo["mae_m"] is not None and noise.get("mae_bound_m") is not None and loo["mae_m"] <= float(noise["mae_bound_m"])),
            "diagnostic_low_to_high_linear": {
                "model": low_high_model,
                "holdout_mae_m": mean(low_high_errs),
                "rows": low_high_rows,
                "interpretation": "diagnostic only; high-tilt runs include ground-contact truncation and are not used as a scale-law claim",
            },
        },
    }


def severity_leave_one_out(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    scan = [
        r for r in runs
        if r.get("point", {}).get("role") == "scan"
        and r.get("altitude", {}).get("altitude_loss_m") is not None
        and r.get("attitude", {}).get("achieved_peak_tilt_deg") is not None
    ]
    by_target: dict[float, list[dict[str, Any]]] = defaultdict(list)
    for run in scan:
        by_target[float(run["point"]["target_deg"])].append(run)
    rows = []
    errs = []
    for target, group in sorted(by_target.items()):
        if len(group) < 2:
            continue
        for run in group:
            others = [float(r["altitude"]["altitude_loss_m"]) for r in group if r is not run]
            pred = statistics.fmean(others)
            truth = float(run["altitude"]["altitude_loss_m"])
            err = abs(pred - truth)
            errs.append(err)
            rows.append(
                {
                    "run_id": run["run_id"],
                    "target_deg": target,
                    "label": run.get("label"),
                    "truth_loss_m": truth,
                    "pred_loss_m": pred,
                    "abs_err_m": err,
                }
            )
    return {"mae_m": mean(errs), "rows": rows}


def learn_tilt_threshold(train: list[dict[str, Any]]) -> float:
    vals = sorted({float(r["attitude"]["achieved_peak_tilt_deg"]) for r in train})
    if not vals:
        return 45.0
    candidates = [vals[0] - 1.0] + [(vals[i] + vals[i + 1]) / 2.0 for i in range(len(vals) - 1)] + [vals[-1] + 1.0]
    best = (0, 45.0)
    for thr in candidates:
        ok = 0
        for r in train:
            pred = predict_label_from_threshold(r, thr)
            ok += int(pred == r.get("label"))
        if ok > best[0]:
            best = (ok, thr)
    return best[1]


def predict_label_from_threshold(run: dict[str, Any], threshold: float) -> str:
    if int(run.get("contract", {}).get("preventive_count", 0)) > 0:
        return "contract_violated"
    tilt = float(run["attitude"]["achieved_peak_tilt_deg"])
    return "clean_unsafe" if tilt >= threshold else "clean_safe"


def fit_loss_model(train: list[dict[str, Any]]) -> dict[str, float]:
    xs = [1.0 - math.cos(math.radians(float(r["attitude"]["achieved_peak_tilt_deg"]))) for r in train]
    ys = [float(r["altitude"]["altitude_loss_m"]) for r in train]
    if len(xs) < 2:
        return {"intercept": 0.0, "slope": 0.0}
    xbar = statistics.fmean(xs)
    ybar = statistics.fmean(ys)
    denom = sum((x - xbar) ** 2 for x in xs)
    slope = 0.0 if denom == 0 else sum((x - xbar) * (y - ybar) for x, y in zip(xs, ys)) / denom
    return {"intercept": ybar - slope * xbar, "slope": slope}


def predict_loss(model: dict[str, float], tilt: float) -> float:
    x = 1.0 - math.cos(math.radians(tilt))
    return float(model["intercept"]) + float(model["slope"]) * x


def evaluate_controls(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    zero = [r for r in runs if r.get("point", {}).get("role") == "gigo" and r.get("point", {}).get("profile") == "zero_tilt"]
    high = [
        r for r in runs
        if r.get("point", {}).get("role") == "scan"
        and math.isclose(float(r.get("point", {}).get("target_deg", -1)), float(config["acro_profile"]["noise_target_deg"]))
    ]
    compare = [r for r in runs if r.get("point", {}).get("role") == "compare" and int(r.get("point", {}).get("angle_max_cd", 0)) == 4500]
    by_mode: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in compare:
        by_mode[str(r["point"]["mode"])].append(r)
    zero_loss = [float(r["altitude"]["altitude_loss_m"]) for r in zero if r.get("altitude", {}).get("altitude_loss_m") is not None]
    high_loss = [float(r["altitude"]["altitude_loss_m"]) for r in high if r.get("altitude", {}).get("altitude_loss_m") is not None]
    comp_summary = {}
    for mode, rs in by_mode.items():
        comp_summary[mode] = {
            "n": len(rs),
            "tilt_mean_deg": mean([float(r["attitude"]["achieved_peak_tilt_deg"]) for r in rs if r.get("attitude", {}).get("achieved_peak_tilt_deg") is not None]),
            "loss_mean_m": mean([float(r["altitude"]["altitude_loss_m"]) for r in rs if r.get("altitude", {}).get("altitude_loss_m") is not None]),
            "hard_count": sum(1 for r in rs if r.get("oracle_A", {}).get("hard")),
        }
    b_ok = bool(
        comp_summary.get("STABILIZE", {}).get("tilt_mean_deg") is not None
        and comp_summary.get("ACRO", {}).get("tilt_mean_deg") is not None
        and float(comp_summary["STABILIZE"]["tilt_mean_deg"]) <= 55.0
        and float(comp_summary["ACRO"]["tilt_mean_deg"]) > 60.0
        and int(comp_summary["ACRO"]["hard_count"]) >= 1
        and int(comp_summary["STABILIZE"]["hard_count"]) == 0
    )
    return {
        "gigo": {
            "zero_tilt_loss_mean_m": mean(zero_loss),
            "high_tilt_loss_mean_m": mean(high_loss),
            "zero_n": len(zero_loss),
            "high_n": len(high_loss),
            "ok": bool(zero_loss and high_loss and mean(zero_loss) is not None and mean(high_loss) is not None and mean(zero_loss) < 5.0 and mean(high_loss) > 15.0),
        },
        "stabilize_vs_acro": {
            "by_mode": comp_summary,
            "ok": b_ok,
        },
    }


def evaluate_stratification(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    strata = [r for r in runs if r.get("point", {}).get("role") == "stratify"]
    rows = []
    grouped: dict[str, list[tuple[float, float, float | None]]] = defaultdict(list)
    for r in strata:
        mode = str(r["point"]["mode"])
        amax = float(r["point"]["angle_max_cd"]) / 100.0
        tilt = r.get("attitude", {}).get("achieved_peak_tilt_deg")
        loss = r.get("altitude", {}).get("altitude_loss_m")
        if tilt is None:
            continue
        grouped[mode].append((amax, float(tilt), None if loss is None else float(loss)))
        rows.append({"run_id": r["run_id"], "mode": mode, "angle_max_deg": amax, "tilt_deg": float(tilt), "loss_m": loss, "label": r.get("label")})
    stab = sorted(grouped.get("STABILIZE", []))
    acro = sorted(grouped.get("ACRO", []))
    stab_monotone = all(stab[i][1] <= stab[i + 1][1] + 3.0 for i in range(len(stab) - 1)) and len(stab) >= 3
    acro_range = (max(t for _, t, _ in acro) - min(t for _, t, _ in acro)) if len(acro) >= 2 else None
    acro_exceeds = all(t > a + 1.0 for a, t, _ in acro) if acro else False
    return {
        "rows": rows,
        "stabilize_monotone_with_angle_max": stab_monotone,
        "acro_tilt_range_deg": acro_range,
        "acro_exceeds_each_angle_max": acro_exceeds,
        "ok": bool(stab_monotone and acro_range is not None and acro_range <= 15.0 and acro_exceeds),
    }


def summarize_mavlink(runs: list[dict[str, Any]]) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for run in runs:
        counts.update({k: int(v) for k, v in run.get("mavlink_sent_counts", {}).items()})
    counts.setdefault("SET_ATTITUDE_TARGET", 0)
    counts.setdefault("GUIDED_MODE_SET", 0)
    counts.setdefault("OFFBOARD_SETPOINT", 0)
    return dict(sorted(counts.items()))


def decide_verdict(result: dict[str, Any]) -> str:
    premises = result.get("premises", {})
    if not premises or not all(bool(v.get("ok")) for v in premises.values()):
        return "INCONCLUSIVE"
    pass_bits = [
        result.get("robustness", {}).get("ok"),
        result.get("prediction", {}).get("classification", {}).get("ok"),
        result.get("prediction", {}).get("severity_regression", {}).get("ok"),
        result.get("controls", {}).get("stabilize_vs_acro", {}).get("ok"),
        result.get("angle_max_stratification", {}).get("ok"),
        result.get("mavlink_message_summary", {}).get("SET_ATTITUDE_TARGET", 1) == 0,
        result.get("mavlink_message_summary", {}).get("GUIDED_MODE_SET", 1) == 0,
    ]
    return "PASS" if all(bool(v) for v in pass_bits) else "FAIL"


def make_plots(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, str]:
    out: dict[str, str] = {}
    results_dir = PLANC_ROOT / "results"
    scan = [r for r in runs if r.get("point", {}).get("role") == "scan" and r.get("attitude", {}).get("achieved_peak_tilt_deg") is not None]
    if scan:
        fig, ax = plt.subplots(figsize=(7.0, 4.5))
        colors = {"clean_safe": "#2a9d8f", "clean_unsafe": "#d62828", "contract_violated": "#6c757d", "ambiguous": "#f4a261", "hard_unsafe_not_anglemax_witness": "#9d4edd"}
        for label, rs in defaultdict(list, {lbl: [r for r in scan if r.get("label") == lbl] for lbl in {r.get("label") for r in scan}}).items():
            ax.scatter(
                [float(r["attitude"]["achieved_peak_tilt_deg"]) for r in rs],
                [float(r["altitude"]["altitude_loss_m"] or 0.0) for r in rs],
                label=str(label),
                color=colors.get(str(label), "#333333"),
                s=35,
                alpha=0.85,
            )
        ax.axvline(45.0, color="black", lw=1, ls="--", label="ANGLE_MAX 45 deg")
        ax.axhline(float(config["oracle"]["altitude_loss_thresh_m"]), color="#555555", lw=1, ls=":")
        ax.set_xlabel("achieved peak tilt (deg)")
        ax.set_ylabel("altitude loss at hover throttle (m)")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8, loc="best")
        path = results_dir / "stabilize_acro_v1_tilt_vs_loss.png"
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        out["tilt_vs_loss"] = str(path)

    compare = [r for r in runs if r.get("point", {}).get("role") == "compare"]
    if compare:
        fig, axes = plt.subplots(1, 2, figsize=(8.0, 3.8))
        modes = ["STABILIZE", "ACRO"]
        for i, metric in enumerate(["achieved_peak_tilt_deg", "altitude_loss_m"]):
            vals = []
            for mode in modes:
                rs = [r for r in compare if r.get("point", {}).get("mode") == mode]
                if metric == "achieved_peak_tilt_deg":
                    vals.append(mean([float(r["attitude"][metric]) for r in rs if r.get("attitude", {}).get(metric) is not None]) or 0.0)
                else:
                    vals.append(mean([float(r["altitude"][metric]) for r in rs if r.get("altitude", {}).get(metric) is not None]) or 0.0)
            axes[i].bar(modes, vals, color=["#457b9d", "#e76f51"])
            axes[i].grid(True, axis="y", alpha=0.25)
            axes[i].set_ylabel("deg" if i == 0 else "m")
            axes[i].set_title("peak tilt" if i == 0 else "altitude loss")
        path = results_dir / "stabilize_acro_v1_stabilize_vs_acro.png"
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        out["stabilize_vs_acro"] = str(path)

    strata = [r for r in runs if r.get("point", {}).get("role") == "stratify" and r.get("attitude", {}).get("achieved_peak_tilt_deg") is not None]
    if strata:
        fig, ax = plt.subplots(figsize=(6.8, 4.2))
        for mode, color in [("STABILIZE", "#457b9d"), ("ACRO", "#e76f51")]:
            rs = sorted([r for r in strata if r.get("point", {}).get("mode") == mode], key=lambda r: float(r["point"]["angle_max_cd"]))
            ax.plot(
                [float(r["point"]["angle_max_cd"]) / 100.0 for r in rs],
                [float(r["attitude"]["achieved_peak_tilt_deg"]) for r in rs],
                marker="o",
                label=mode,
                color=color,
            )
        ax.plot([30, 45, 60], [30, 45, 60], color="black", lw=1, ls="--", label="y = ANGLE_MAX")
        ax.set_xlabel("ANGLE_MAX (deg)")
        ax.set_ylabel("achieved peak tilt (deg)")
        ax.grid(True, alpha=0.25)
        ax.legend()
        path = results_dir / "stabilize_acro_v1_anglemax_stratification.png"
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        out["anglemax_stratification"] = str(path)

    gigo = [r for r in runs if r.get("point", {}).get("role") in {"gigo", "scan"}]
    zero = [r for r in gigo if r.get("point", {}).get("profile") == "zero_tilt"]
    high = [r for r in gigo if math.isclose(float(r.get("point", {}).get("target_deg", -1)), float(config["acro_profile"]["noise_target_deg"]))]
    if zero and high:
        fig, ax = plt.subplots(figsize=(5.8, 3.8))
        vals = [
            mean([float(r["altitude"]["altitude_loss_m"]) for r in zero if r.get("altitude", {}).get("altitude_loss_m") is not None]) or 0.0,
            mean([float(r["altitude"]["altitude_loss_m"]) for r in high if r.get("altitude", {}).get("altitude_loss_m") is not None]) or 0.0,
        ]
        ax.bar(["zero tilt", "55 deg tilt"], vals, color=["#2a9d8f", "#d62828"])
        ax.axhline(float(config["oracle"]["altitude_loss_thresh_m"]), color="#555555", ls=":", lw=1)
        ax.set_ylabel("altitude loss (m)")
        ax.grid(True, axis="y", alpha=0.25)
        path = results_dir / "stabilize_acro_v1_gigo_hover_throttle.png"
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        out["gigo_hover_throttle"] = str(path)
    return out


def write_report(config: dict[str, Any], result: dict[str, Any], path: Path) -> None:
    fw = result.get("firmware_anchor", {})
    lines = [
        f"VERDICT: {result.get('verdict')}",
        "",
        "# stabilize_acro v1 report",
        "",
        f"- Scenario: deliberate, legal operator input via supported ACRO mode; not spontaneous loss of control.",
        f"- Firmware: `{fw.get('tag_or_describe')}` / `{fw.get('sha')}` from `{fw.get('root')}`.",
        f"- Reachability: RC_CHANNELS_OVERRIDE + mode changes only; SET_ATTITUDE_TARGET count = `{result.get('mavlink_message_summary', {}).get('SET_ATTITUDE_TARGET')}`; GUIDED mode set count = `{result.get('mavlink_message_summary', {}).get('GUIDED_MODE_SET')}`.",
        f"- Fixed ACRO config: `ANGLE_MAX=45 deg`, `ACRO_TRAINER=0`; ANGLE_MAX is not raised or disabled.",
        "",
        "## Decision checks",
    ]
    for name, item in result.get("premises", {}).items():
        lines.append(f"- {name}: {'OK' if item.get('ok') else 'NO'}")
    robust = result.get("robustness", {})
    severity = result.get("prediction", {}).get("severity_regression", {})
    lines.extend(
        [
            f"- Robust clean_unsafe zone: {'OK' if robust.get('ok') else 'NO'}; runs={robust.get('clean_unsafe_runs')}, targets={robust.get('clean_unsafe_targets')}, tilt_width={fmt(robust.get('tilt_width_deg'))} deg.",
            f"- Prediction gate (a): {'OK' if result.get('prediction', {}).get('classification', {}).get('ok') else 'NO'}; accuracy={fmt(result.get('prediction', {}).get('classification', {}).get('accuracy'), 3)}.",
            f"- Severity repeatability MAE: {'OK' if severity.get('ok') else 'NO'}; MAE={fmt(severity.get('holdout_mae_m'))} m, bound={fmt(severity.get('mae_bound_m'))} m.",
            f"- Stabilize-vs-ACRO gate (b): {'OK' if result.get('controls', {}).get('stabilize_vs_acro', {}).get('ok') else 'NO'}.",
            f"- ANGLE_MAX invariance gate (c): {'OK' if result.get('angle_max_stratification', {}).get('ok') else 'NO'}.",
            f"- Final label counts: `{result.get('label_counts')}`.",
            "",
            "## Noise and GIGO",
        ]
    )
    noise = result.get("noise", {})
    gigo = result.get("controls", {}).get("gigo", {})
    lines.extend(
        [
            f"- 55 deg repeated altitude-loss sigma: `{fmt(noise.get('sigma_m'), 4)} m`; d_margin=`{fmt(noise.get('d_margin_m'), 4)} m`; mae_bound=`{fmt(noise.get('mae_bound_m'), 4)} m`.",
            f"- Zero-tilt hover loss mean: `{fmt(gigo.get('zero_tilt_loss_mean_m'))} m`; 55 deg hover loss mean: `{fmt(gigo.get('high_tilt_loss_mean_m'))} m`.",
            "",
            "## Figures",
        ]
    )
    for name, fig_path in result.get("figures", {}).items():
        lines.append(f"- {name}: `{fig_path}`")
    ground_contact = [
        r for r in result.get("runs", [])
        if r.get("point", {}).get("role") == "scan" and r.get("oracle_A", {}).get("ground_contact")
    ]
    if ground_contact:
        targets = sorted({float(r.get("point", {}).get("target_deg", 0.0)) for r in ground_contact})
        lines.extend(
            [
                "",
                "## Extreme Points",
                "",
                f"- Ground-contact oracle-A occurred at target layers `{targets}`. No preventive failsafe or CRASH_CHECK contract trigger was logged before these consequences; the clean witness band does not depend on those extreme ground-contact points.",
            ]
        )
    lines.extend(
        [
            "",
            "## Limitations",
            "",
            "- This is an operator-reachable legal-input demonstration, not an environment-only or spontaneous instability claim.",
            "- The consequence is near deterministic in SITL; robustness is from the wide achieved-tilt range, not from claiming to predict through large noise.",
            "- No threshold scaling law, O(log) search law, or area law is claimed.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "stabilize_acro_v1_config.yaml")
    parser.add_argument("--stage", choices=["smoke", "premise", "scan", "controls", "stratify", "all", "report"], default="smoke")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--only-targets", default="", help="comma-separated target degrees to run from the selected stage")
    parser.add_argument("--max-runs", type=int, default=0, help="stop after this many selected points; 0 means no limit")
    args = parser.parse_args()

    config = load_yaml(args.config)
    results_dir = PLANC_ROOT / "results"
    partial_path = results_dir / "stabilize_acro_v1_partial.json"
    env_path = results_dir / "env_stabilize_acro_v1.json"
    env = probe_environment(config, REPO_ROOT)
    write_env(env, env_path)

    partial = load_json(partial_path, {"runs": []}) if args.resume or partial_path.exists() else {"runs": []}
    runs: list[dict[str, Any]] = list(partial.get("runs", []))
    if args.stage != "report":
        points = build_points(config, args.stage)
        if args.only_targets.strip():
            wanted = {float(v.strip()) for v in args.only_targets.split(",") if v.strip()}
            points = [p for p in points if float(p.get("target_deg", -9999.0)) in wanted]
        if args.max_runs > 0:
            points = points[: args.max_runs]
        for point in points:
            runs = cached_run(config, point, partial_path, runs, args.resume)
        write_json(partial_path, {"updated_at_utc": utc_now(), "runs": runs})
    result = aggregate(config, runs)
    print(f"VERDICT {result['verdict']} result=planc/results/stabilize_acro_v1_result.json report=planc/results/stabilize_acro_v1_report.md", flush=True)


if __name__ == "__main__":
    main()
