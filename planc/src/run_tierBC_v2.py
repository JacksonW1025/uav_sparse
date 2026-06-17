"""Control-authority Tier B re-anchor + Tier C verification (v2).

Re-anchors the OverDraw contract-clean unsafe demonstration on a *supported*
ArduCopter interface and tests whether a <=ANGLE_MAX command plus legal wind can
itself be unsafe.

Part B (re-anchor): GUIDED + SET_ATTITUDE_TARGET, type_mask=0x07 (ignore the
three body-rate fields) driven by a unit attitude quaternion. A NON-ZERO
quaternion routes (mode_guided.cpp:964-967) to AC_AttitudeControl::input_quaternion
(AC_AttitudeControl.cpp:231-266) which applies only ang_vel_limit -- no ANGLE_MAX
clamp. This is the documented-supported path; the predecessor oracleA_v1 used the
doc-discouraged body-rate fields. Part B commands attitudes > ANGLE_MAX and
checks (i) the demanded attitude target really exceeds ANGLE_MAX (bypass holds),
(ii) a real hard consequence, (iii) interface-aware Tier-1 (B) = 0.

Part C (verification): same supported interface, command attitude held strictly
< ANGLE_MAX, plus legal wind/turbulence. Classifies each run as recovered /
diverged (C-attitude) / altitude_loss (C-altitude) / safe.

Interface-aware B: command attitude > ANGLE_MAX is NOT a contract clamp on the
quaternion/rate interface (by design -- angle_max_scope_v1); it is scored under
Tier-2. CRASH_FAILSAFE / CRASH_CHECK are post-impact consequence detectors and
are excluded from preventive B (config preventive sets omit them). So
interface-aware B = a genuine preventive failsafe firing before the hard
consequence.

Reuses: run_oracleA_v1.parse_dataflash_hardened (hardened oracle A + B parse),
run_stage0_v2.q_from_euler / point_config / point_params / command_at, and the
harness hygiene (disarm after the oracle window, GCS_COMMAND excluded as harness,
never reads *.oracle.json for any contract decision).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
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
    release_rc_override,
    request_streams,
    send_gcs_heartbeat,
    set_mode,
    wait_altitude,
    wait_position_stable,
)
from param_manager import ParamManager
from run_oracleA_v1 import (
    disarm_for_cleanup,
    first_time,
    nearest_value,
    percentile,
    parse_dataflash_hardened,
    slope_deg_s,
    sustained_exceed,
    wait_disarmed,
)
from run_stage0_v2 import fmt, point_config, point_params, q_from_euler
from sitl_runner import SitlRunner

csv.field_size_limit(10_000_000)

ATTITUDE_QUATERNION_TYPE_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)  # 0x07 -- the documented "supported" GUIDED attitude-target mask


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


# --------------------------------------------------------------------------- #
# Supported interface: SET_ATTITUDE_TARGET quaternion (type_mask=0x07)
# --------------------------------------------------------------------------- #
def send_attitude_quaternion(master, *, roll_deg: float, pitch_deg: float, yaw_deg: float, thrust: float) -> None:
    """Send a unit attitude quaternion with all three body-rate fields ignored.

    type_mask=0x07 is exactly what ArduPilot's GUIDED dev docs prescribe. A
    non-zero quaternion is required for angle_control_run to route to
    input_quaternion (the supported, ANGLE_MAX-unclamped path) rather than the
    body-rate primitive.
    """
    q = q_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    master.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        ATTITUDE_QUATERNION_TYPE_MASK,
        q,
        0.0,
        0.0,
        0.0,
        float(thrust),
    )


def send_rc_override(master, *, roll_pwm: int = 0, pitch_pwm: int = 0, throttle_pwm: int = 0, yaw_pwm: int = 0) -> None:
    """RC override for ACRO corroboration. 0 == release that channel."""
    chans = [int(roll_pwm), int(pitch_pwm), int(throttle_pwm), int(yaw_pwm)] + [0] * 14
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans[:18])


# --------------------------------------------------------------------------- #
# Command profiles
# --------------------------------------------------------------------------- #
def ramp_hold_profile(config: dict[str, Any], *, target_deg: float, slew_rate_deg_s: float, hold_s: float, axis: str) -> list[dict[str, float]]:
    """Ramp (slew) to target_deg at slew_rate, hold, ramp back to 0, post-hold.

    Used for Part B (target > ANGLE_MAX) and Part C (target < ANGLE_MAX). On the
    quaternion interface the body-rate fields are ignored, so the per-sample
    *_rate fields are descriptive only (zeroed on the wire).
    """
    hz = float(config["experiment"]["stream_hz"])
    dt = 1.0 / hz
    rate = max(1.0, abs(float(slew_rate_deg_s)))
    samples: list[dict[str, float]] = []
    t_s = 0.0
    current = 0.0

    def append(value: float, rate_val: float) -> None:
        nonlocal t_s
        row = {"t_s": t_s, "roll_deg": 0.0, "pitch_deg": 0.0, "roll_rate_deg_s": 0.0, "pitch_rate_deg_s": 0.0}
        if axis == "pitch":
            row["pitch_deg"] = value
            row["pitch_rate_deg_s"] = rate_val
        else:
            row["roll_deg"] = value
            row["roll_rate_deg_s"] = rate_val
        samples.append(row)
        t_s += dt

    def ramp_to(target: float) -> None:
        nonlocal current
        if math.isclose(current, target, abs_tol=1.0e-9):
            append(current, 0.0)
            return
        sign = 1.0 if target > current else -1.0
        while (target - current) * sign > 1.0e-9:
            step = min(abs(target - current), rate * dt)
            current += sign * step
            append(current, sign * rate)

    def hold(value: float, duration_s: float) -> None:
        nonlocal current
        current = value
        for _ in range(max(1, int(round(duration_s * hz)))):
            append(current, 0.0)

    if abs(target_deg) < 1.0e-9:
        # level-hover probe: just hold 0 for the requested duration
        hold(0.0, hold_s)
    else:
        ramp_to(float(target_deg))
        hold(float(target_deg), hold_s)
        ramp_to(0.0)
    hold(0.0, float(config["experiment"].get("post_maneuver_hold_s", 1.0)))
    return samples


# --------------------------------------------------------------------------- #
# Hover-throttle calibration (so a 0 deg command is altitude-neutral)
# --------------------------------------------------------------------------- #
def read_hover_thrust(master, *, default: float = 0.5) -> float:
    master.mav.param_request_read_send(master.target_system, master.target_component, b"MOT_THST_HOVER", -1)
    deadline = time.time() + 4.0
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is not None and getattr(msg, "param_id", "") == "MOT_THST_HOVER":
            v = float(msg.param_value)
            if 0.1 < v < 0.9:
                return v
    return default


# --------------------------------------------------------------------------- #
# Per-run drivers
# --------------------------------------------------------------------------- #
def tierbc_run_id(config: dict[str, Any], point: dict[str, Any]) -> str:
    prefix = str(config["experiment"].get("run_prefix", "tierBCv2"))
    return (
        f"{prefix}_{point['part']}_{point['name']}"
        f"_g{int(round(float(point['target_deg']))):03d}"
        f"_w{int(round(float(point.get('wind_m_s', 0.0)))):02d}"
        f"_t{int(round(float(point.get('turbulence_m_s', 0.0)))):02d}"
        f"_a{int(round(float(point['angle_max_cd']))):04d}"
        f"_{point.get('model', 'm100')}_s{int(point.get('seed', 0)):02d}"
    )


def _point_for_helpers(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    """Shape a point dict the reused point_config/point_params helpers expect."""
    return {
        "role": point["part"],
        "phase": point["name"],
        "layer": point["name"],
        "r_deg_s": float(point.get("slew_rate_deg_s", 0.0)),
        "seed": int(point.get("seed", 0)),
        "wind_m_s": float(point.get("wind_m_s", 0.0)),
        "turbulence_m_s": float(point.get("turbulence_m_s", 0.0)),
        "angle_max_cd": float(point["angle_max_cd"]),
        "model": str(point.get("model", config["pressure"]["default_model"])),
    }


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        if value in (None, ""):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def rows_by_type_from_csv(csv_path: Path) -> dict[str, list[dict[str, Any]]]:
    rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
    if not csv_path.exists():
        return rows
    with csv_path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            typ = row.get("type")
            if typ:
                rows[str(typ)].append(row)
    return rows


def fallback_window(rows_by_type: dict[str, list[dict[str, Any]]], parsed: dict[str, Any], config: dict[str, Any], profile: list[dict[str, float]]) -> tuple[float | None, float | None]:
    """Return an oracle window even for 0 deg hover probes with no active ATT target."""
    win = parsed.get("active_window_s", {})
    start = win.get("start")
    end = win.get("oracle_end")
    if start is not None and end is not None:
        return float(start), float(end)

    mode_rows = rows_by_type.get("MODE", [])
    requested = str(config["experiment"].get("use_mode", "GUIDED_NOGPS"))
    mode_times = [
        _to_float(r.get("time_s"))
        for r in mode_rows
        if str(r.get("mode_name", "")) == requested
    ]
    att_rows = rows_by_type.get("ATT", [])
    if mode_times:
        start = max(mode_times)
    elif att_rows:
        start = _to_float(att_rows[0].get("time_s"))
    else:
        return None, None
    duration = (
        float(config["experiment"].get("pre_maneuver_hold_s", 0.0))
        + (float(profile[-1]["t_s"]) if profile else 0.0)
        + float(config["experiment"].get("observation_after_profile_s", 0.0))
    )
    return start, start + duration


def attitude_extrema_from_csv(
    csv_path: Path,
    start: float | None,
    end: float | None,
    *,
    angle_max_deg: float | None = None,
    tolerance_deg: float = 0.1,
) -> dict[str, Any]:
    """Max demanded/achieved lean inside the oracle window -- the no-clamp evidence.

    DesRoll/DesPitch is the attitude controller's TARGET (post input_quaternion).
    If it exceeds ANGLE_MAX, the supported path accepted an attitude target the
    lean-angle clamp would have rejected.
    """
    max_des = 0.0
    max_ach = 0.0
    des_roll_peak = 0.0
    des_pitch_peak = 0.0
    first_desroll_exceed_time = None
    first_demanded_lean_exceed_time = None
    n = 0
    if not csv_path.exists():
        return {"applicable": False}
    with csv_path.open() as fh:
        for row in csv.DictReader(fh):
            if row.get("type") != "ATT":
                continue
            try:
                t = float(row.get("time_s"))
            except (TypeError, ValueError):
                continue
            if start is not None and end is not None and not (start <= t <= end):
                continue
            try:
                dr = float(row.get("DesRoll", 0.0) or 0.0)
                dp = float(row.get("DesPitch", 0.0) or 0.0)
                r = float(row.get("Roll", 0.0) or 0.0)
                p = float(row.get("Pitch", 0.0) or 0.0)
            except (TypeError, ValueError):
                continue
            max_des = max(max_des, math.hypot(dr, dp))
            max_ach = max(max_ach, math.hypot(r, p))
            des_roll_peak = max(des_roll_peak, abs(dr))
            des_pitch_peak = max(des_pitch_peak, abs(dp))
            if angle_max_deg is not None:
                demanded = math.hypot(dr, dp)
                if first_desroll_exceed_time is None and abs(dr) > angle_max_deg + tolerance_deg:
                    first_desroll_exceed_time = t
                if first_demanded_lean_exceed_time is None and demanded > angle_max_deg + tolerance_deg:
                    first_demanded_lean_exceed_time = t
            n += 1
    out = {
        "applicable": n > 0,
        "samples": n,
        "max_demanded_lean_deg": max_des,
        "max_achieved_lean_deg": max_ach,
        "max_abs_des_roll_deg": des_roll_peak,
        "max_abs_des_pitch_deg": des_pitch_peak,
    }
    if angle_max_deg is not None:
        out.update({
            "angle_max_deg": angle_max_deg,
            "validity_tolerance_deg": tolerance_deg,
            "desroll_le_angle_max": des_roll_peak <= angle_max_deg + tolerance_deg,
            "demanded_lean_le_angle_max": max_des <= angle_max_deg + tolerance_deg,
            "desroll_exceeds_angle_max": first_desroll_exceed_time is not None,
            "demanded_lean_exceeds_angle_max": first_demanded_lean_exceed_time is not None,
            "first_desroll_exceeds_angle_max_time_s": first_desroll_exceed_time,
            "first_demanded_lean_exceeds_angle_max_time_s": first_demanded_lean_exceed_time,
        })
    return out


def hardened_oracle_v2_from_csv(
    csv_path: Path,
    parsed: dict[str, Any],
    config: dict[str, Any],
    point: dict[str, Any],
    profile: list[dict[str, float]],
) -> dict[str, Any]:
    """v2 oracle: crash/contact, actual-vs-DesRoll divergence, and altitude loss.

    The reused oracleA parser remains on disk for continuity, but v2 decisions use
    ATT.DesRoll/DesPitch as the demanded attitude. This is the preregistered
    validity basis for Part C and the no-clamp evidence for Part B.
    """
    rows = rows_by_type_from_csv(csv_path)
    start, end = fallback_window(rows, parsed, config, profile)
    if start is None or end is None:
        return {
            "inside": False,
            "outcome": "no_window",
            "active_window_s": {"start": start, "oracle_end": end},
            "causes": {},
        }

    att_w = [r for r in rows.get("ATT", []) if start <= _to_float(r.get("time_s")) <= end]
    pos_w = [r for r in rows.get("POS", []) if start <= _to_float(r.get("time_s")) <= end]
    xkf1_w = [
        r for r in rows.get("XKF1", [])
        if start <= _to_float(r.get("time_s")) <= end and int(_to_float(r.get("C"), 0.0)) == 0
    ]
    msg_w = [r for r in rows.get("MSG", []) if start <= _to_float(r.get("time_s")) <= end]

    att_errors: list[tuple[float, float]] = []
    for row in att_w:
        t = _to_float(row.get("time_s"))
        err = math.hypot(
            _to_float(row.get("Roll")) - _to_float(row.get("DesRoll")),
            _to_float(row.get("Pitch")) - _to_float(row.get("DesPitch")),
        )
        att_errors.append((t, err))

    div = sustained_exceed(
        att_errors,
        float(config["oracle"]["divergence_error_deg"]),
        float(config["oracle"]["divergence_duration_s"]),
    )
    tail_start = end - float(config["oracle"]["divergence_tail_s"])
    trend_start = end - float(config["oracle"]["divergence_trend_s"])
    tail_rows = [(t, v) for t, v in att_errors if t >= tail_start]
    trend_rows = [(t, v) for t, v in att_errors if t >= trend_start]
    final_error = percentile([v for _, v in tail_rows], 0.50) if tail_rows else (att_errors[-1][1] if att_errors else None)
    tail_slope = slope_deg_s(trend_rows)
    first_divergence_time = None
    intervals = div.get("intervals", [])
    if intervals:
        first_divergence_time = float(intervals[0]["start_s"])
    unrecovered_divergence = bool(
        div.get("ok")
        and final_error is not None
        and float(final_error) > float(config["oracle"]["divergence_error_deg"])
    )
    worsening_divergence = bool(
        div.get("ok")
        and final_error is not None
        and tail_slope is not None
        and float(final_error) >= float(config["oracle"]["divergence_worsening_min_final_deg"])
        and float(tail_slope) >= float(config["oracle"]["divergence_worsening_slope_deg_s"])
    )
    attitude_diverged = bool(unrecovered_divergence or worsening_divergence)

    altitude_rows = [
        (_to_float(r.get("time_s")), _to_float(r.get("RelHomeAlt")))
        for r in pos_w
        if r.get("RelHomeAlt") not in (None, "")
    ]
    start_alt = nearest_value(altitude_rows, start)
    min_alt = min((v for _, v in altitude_rows), default=None)
    end_alt = nearest_value(altitude_rows, end)
    altitude_loss = None if start_alt is None or min_alt is None else start_alt - min_alt
    altitude_loss_true = bool(altitude_loss is not None and altitude_loss > float(config["oracle"]["altitude_loss_thresh_m"]))
    altitude_loss_marker = None
    if start_alt is not None:
        for t, alt in altitude_rows:
            loss = start_alt - alt
            if loss > float(config["oracle"]["altitude_loss_thresh_m"]):
                altitude_loss_marker = {"time_s": t, "message": f"altitude loss {loss:.2f} m"}
                break

    vd_rows = [(_to_float(r.get("time_s")), _to_float(r.get("VD"))) for r in xkf1_w if r.get("VD") not in (None, "")]
    ground_messages = []
    for row in msg_w:
        text = str(row.get("Message", ""))
        if "sim hit ground" in text.lower():
            ground_messages.append({"time_s": _to_float(row.get("time_s")), "message": text})
    low_floor_markers = []
    for t, alt in altitude_rows:
        vd = nearest_value(vd_rows, t)
        if alt < float(config["oracle"]["h_floor_m"]) and vd is not None and vd > float(config["oracle"]["floor_descent_rate_m_s"]):
            low_floor_markers.append({"time_s": t, "rel_home_alt_m": alt, "down_velocity_m_s": vd})
            break
    crash_or_contact = bool(ground_messages or low_floor_markers)
    first_ground_time = first_time(ground_messages + low_floor_markers)
    attitude_before_ground = bool(
        attitude_diverged
        and first_divergence_time is not None
        and (first_ground_time is None or first_divergence_time <= first_ground_time)
    )

    hard_a_causes = {
        "crash_or_contact": crash_or_contact,
        "attitude_diverged": attitude_diverged,
        "attitude_diverged_before_ground": attitude_before_ground,
        "altitude_loss": altitude_loss_true,
    }
    hard_a_inside = any(hard_a_causes.values())
    transient_recovered = bool(div.get("ok") and not hard_a_inside)

    if attitude_before_ground:
        outcome = "attitude_diverged"
    elif altitude_loss_true:
        outcome = "altitude_loss"
    elif crash_or_contact:
        outcome = "crashed"
    elif transient_recovered:
        outcome = "recovered"
    else:
        outcome = "safe"

    hard_a_markers: list[dict[str, Any]] = []
    hard_a_markers.extend(ground_messages)
    hard_a_markers.extend(low_floor_markers)
    if altitude_loss_true and altitude_loss_marker is not None:
        hard_a_markers.append(altitude_loss_marker)
    if attitude_diverged:
        hard_a_markers.append({
            "time_s": first_divergence_time if first_divergence_time is not None else end,
            "message": f"actual-vs-DesRoll divergence final={fmt(final_error)} slope={fmt(tail_slope)}",
        })

    return {
        "inside": hard_a_inside,
        "outcome": outcome,
        "active_window_s": {"start": start, "oracle_end": end},
        "causes": hard_a_causes,
        "first_hard_a_time_s": first_time([m for m in hard_a_markers if m.get("time_s") is not None]),
        "transient_recovered": transient_recovered,
        "max_attitude_error_deg": max((v for _, v in att_errors), default=None),
        "sustained_transient": div,
        "first_divergence_time_s": first_divergence_time,
        "first_ground_time_s": first_ground_time,
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
        "basis": "actual Roll/Pitch vs ATT.DesRoll/DesPitch",
    }


def run_quaternion_once(config: dict[str, Any], point: dict[str, Any]) -> dict[str, Any]:
    run_id = tierbc_run_id(config, point)
    helper_point = _point_for_helpers(config, point)
    cfg = point_config(config, helper_point)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {"run_id": run_id, "point": point, "interface": point.get("interface", "guided_quaternion"), "started_at_utc": utc_now()}
    master = None
    profile: list[dict[str, float]] = []
    try:
        runner.start(run_id)
        master = runner.connect(timeout_s=35)
        request_streams(master, int(config["experiment"]["stream_hz"]))
        params = point_params(config, helper_point)
        guid_options = float(point.get("guid_options", config["baseline_params"].get("GUID_OPTIONS", 8)))
        params["GUID_OPTIONS"] = guid_options
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result.update({"params_requested": params, "param_snapshot": snapshot, "param_records_path": str(param_path)})

        wait_position_stable(master, min_samples=5, timeout_s=45)
        set_mode(master, "GUIDED", timeout_s=20)
        arm(master, timeout_s=45)
        takeoff_alt_m = float(point.get("takeoff_alt_m", config["experiment"]["takeoff_alt_m"]))
        command_takeoff(master, takeoff_alt_m)
        wait_altitude(master, takeoff_alt_m, timeout_s=90)
        requested_mode = str(config["experiment"].get("use_mode", "GUIDED_NOGPS"))
        if requested_mode != "GUIDED":
            set_mode(master, requested_mode, timeout_s=20)

        yaw_deg = float(config["experiment"].get("yaw_deg", 0.0))
        use_direct_thrust = bool(int(guid_options) & 8)
        if not use_direct_thrust:
            thrust = float(point.get("climb_hold_thrust", 0.5))
        elif str(config["command"].get("thrust_source", "fixed")) == "hover":
            thrust = read_hover_thrust(master, default=float(config["command"].get("thrust", 0.5)))
        else:
            thrust = float(config["command"].get("thrust", 0.5))
        result["thrust_used"] = thrust
        result["set_attitude_target_thrust_semantics"] = "direct_throttle" if use_direct_thrust else "climb_rate_alt_hold"
        speedup = max(1.0, float(config["experiment"].get("speedup", 1.0)))
        hz = float(config["experiment"]["stream_hz"])
        dt_wall = 1.0 / (hz * speedup)

        pre_hold_end = time.time() + float(config["experiment"].get("pre_maneuver_hold_s", 1.0)) / speedup
        while time.time() < pre_hold_end:
            send_gcs_heartbeat(master)
            send_attitude_quaternion(master, roll_deg=0.0, pitch_deg=0.0, yaw_deg=yaw_deg, thrust=thrust)
            time.sleep(dt_wall)

        profile = ramp_hold_profile(
            config,
            target_deg=float(point["target_deg"]),
            slew_rate_deg_s=float(point.get("slew_rate_deg_s", 45.0)),
            hold_s=float(point.get("hold_s", 10.0)),
            axis=str(point.get("axis", "roll")),
        )
        profile_path = PLANC_ROOT / "logs" / f"{run_id}_command_profile.json"
        max_command_angle = max((math.hypot(r["roll_deg"], r["pitch_deg"]) for r in profile), default=0.0)
        write_json(profile_path, {"run_id": run_id, "point": point, "interface": "guided_quaternion", "type_mask": ATTITUDE_QUATERNION_TYPE_MASK, "attitude_target_mode": "quaternion", "samples": profile})
        result["command_profile"] = {"samples": len(profile), "duration_s": profile[-1]["t_s"] if profile else 0.0, "amplitude_deg": max_command_angle, "path": str(profile_path)}

        live_events: list[dict[str, Any]] = []
        t0 = time.time()
        disarmed = False
        for sample in profile:
            deadline = t0 + float(sample["t_s"]) / speedup
            while time.time() < deadline:
                s = max(0.0, min(0.002, deadline - time.time()))
                if s > 0.0:
                    time.sleep(s)
            send_gcs_heartbeat(master)
            send_attitude_quaternion(master, roll_deg=float(sample["roll_deg"]), pitch_deg=float(sample["pitch_deg"]), yaw_deg=yaw_deg, thrust=thrust)
            while True:
                msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=False)
                if msg is None:
                    break
                if msg.get_type() == "STATUSTEXT":
                    live_events.append({"t_s": time.time() - t0, "text": str(getattr(msg, "text", ""))})
                elif msg.get_type() == "HEARTBEAT" and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    disarmed = True
            if disarmed:
                break

        observe_end = time.time() + float(config["experiment"]["observation_after_profile_s"]) / speedup
        while time.time() < observe_end:
            send_gcs_heartbeat(master)
            send_attitude_quaternion(master, roll_deg=0.0, pitch_deg=0.0, yaw_deg=yaw_deg, thrust=thrust)
            msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=dt_wall)
            if msg is not None and msg.get_type() == "STATUSTEXT":
                live_events.append({"t_s": time.time() - t0, "text": str(getattr(msg, "text", ""))})

        result["live_events"] = live_events[-60:]
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
        csv_path = PLANC_ROOT / "logs" / f"{run_id}_parsed.csv"
        parsed = parse_dataflash_hardened(
            bin_path=bin_path,
            csv_path=csv_path,
            oracle_path=PLANC_ROOT / "logs" / f"{run_id}_parsed.oracle.json",
            config=config,
            point=helper_point,
            command_profile=profile,
        )
        result.update(parsed)
        win = parsed.get("active_window_s", {})
        tol = float(config["oracle"].get("command_angle_limit_tolerance_deg", 0.1))
        v2_oracle = hardened_oracle_v2_from_csv(csv_path, parsed, config, point, profile)
        result["hardened_oracle_A_v2"] = v2_oracle
        v2_win = v2_oracle.get("active_window_s", win)
        angle_max_deg = float(point["angle_max_cd"]) / 100.0
        result["attitude_extrema"] = attitude_extrema_from_csv(
            csv_path,
            v2_win.get("start"),
            v2_win.get("oracle_end"),
            angle_max_deg=angle_max_deg,
            tolerance_deg=tol,
        )
        result["interface_label"] = interface_aware_label(result, result["attitude_extrema"], str(point["part"]))
        return result
    except Exception as exc:  # noqa: BLE001
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


# --------------------------------------------------------------------------- #
# Interface-aware labeling (the heart of the re-anchor)
# --------------------------------------------------------------------------- #
def interface_aware_label(run: dict[str, Any], extrema: dict[str, Any], part: str) -> dict[str, Any]:
    ha = run.get("hardened_oracle_A_v2") or run.get("hardened_oracle_A", {})
    ob = run.get("oracle_B", {})
    cmd = run.get("command", {})
    point = run.get("point", {})
    hard = bool(ha.get("inside"))
    outcome = str(ha.get("outcome"))
    angle_max_deg = float(cmd.get("angle_max_deg", 45.0))

    # Interface-aware B: a genuine preventive failsafe before the hard
    # consequence. Command attitude > ANGLE_MAX is NOT a B clamp here (Tier-2,
    # by design). Crash detectors are already excluded from the config sets.
    iface_B = bool(ob.get("B_preventive_before_hard_A"))

    cmd_peak = cmd.get("max_command_angle_deg")
    cmd_exceeds = bool(cmd.get("command_exceeded_angle_max"))
    cmd_touches = bool(cmd.get("command_touched_angle_max"))
    cmd_le_anglemax = (not cmd_exceeds) and (not cmd_touches)

    des_peak = extrema.get("max_demanded_lean_deg") if extrema.get("applicable") else None
    desroll_peak = extrema.get("max_abs_des_roll_deg") if extrema.get("applicable") else None
    desroll_le_anglemax = bool(extrema.get("desroll_le_angle_max"))
    demanded_exceeds = bool(extrema.get("desroll_exceeds_angle_max"))
    first_desroll_over = extrema.get("first_desroll_exceeds_angle_max_time_s")
    first_hard = ha.get("first_hard_a_time_s")
    desroll_crossed_before_hard = bool(
        first_desroll_over is not None
        and (first_hard is None or float(first_desroll_over) <= float(first_hard))
    )

    out: dict[str, Any] = {
        "part": part,
        "group": point.get("group"),
        "hard_A": hard,
        "outcome": outcome,
        "interface_aware_B": iface_B,
        "command_peak_deg": cmd_peak,
        "command_exceeds_angle_max": cmd_exceeds,
        "command_le_angle_max": cmd_le_anglemax,
        "demanded_target_peak_deg": des_peak,
        "desroll_peak_deg": desroll_peak,
        "desroll_le_angle_max": desroll_le_anglemax,
        "demanded_target_exceeds_angle_max": demanded_exceeds,
        "first_desroll_exceeds_angle_max_time_s": first_desroll_over,
        "desroll_crossed_before_hard_A": desroll_crossed_before_hard,
        "angle_max_deg": angle_max_deg,
    }
    if part == "B":
        if not demanded_exceeds:
            out["label"] = "B_invalid_desroll_not_over_limit"
        elif not desroll_crossed_before_hard:
            out["label"] = "B_invalid_terminated_before_desroll_over_limit"
        elif hard and not iface_B:
            out["label"] = "B_reanchor_clean"
        elif hard and iface_B:
            out["label"] = "B_blocked_by_failsafe"
        else:
            out["label"] = "B_no_consequence"
    elif part == "C":
        if not desroll_le_anglemax:
            out["label"] = "C_invalid_desroll_over_limit"
        elif hard and not iface_B:
            if outcome == "attitude_diverged":
                out["label"] = "C_attitude"
            elif outcome in ("altitude_loss", "crashed"):
                out["label"] = "C_altitude"
            else:
                out["label"] = "C_hard_other"
        elif hard and iface_B:
            out["label"] = "C_blocked_by_failsafe"
        elif outcome == "recovered":
            out["label"] = "recovered"
        else:
            out["label"] = "safe"
    else:  # fidelity / wind-bound probes
        out["label"] = f"probe_{outcome}"
    return out


# --------------------------------------------------------------------------- #
# ACRO_TRAINER=0 corroboration (best-effort RC-override probe)
# --------------------------------------------------------------------------- #
def run_acro_once(config: dict[str, Any], spec: dict[str, Any]) -> dict[str, Any]:
    point = {
        "part": "fid",
        "name": str(spec["name"]),
        "interface": "acro_rate",
        "target_deg": 0.0,
        "slew_rate_deg_s": 0.0,
        "seed": int(spec.get("seed", 0)),
        "wind_m_s": float(spec.get("wind_m_s", 0.0)),
        "turbulence_m_s": float(spec.get("turbulence_m_s", 0.0)),
        "angle_max_cd": float(spec["angle_max_cd"]),
        "model": str(spec.get("model", config["pressure"]["default_model"])),
    }
    run_id = tierbc_run_id(config, point)
    helper_point = _point_for_helpers(config, point)
    cfg = point_config(config, helper_point)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {"run_id": run_id, "point": point, "interface": "acro_rate", "started_at_utc": utc_now(), "corroboration": True}
    master = None
    try:
        runner.start(run_id)
        master = runner.connect(timeout_s=35)
        request_streams(master, int(config["experiment"]["stream_hz"]))
        params = point_params(config, helper_point)
        params["ACRO_TRAINER"] = float(spec.get("acro_trainer", 0))
        params["ACRO_RP_RATE"] = float(spec.get("acro_rp_rate_deg_s", 360.0))
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_path, snapshot=snapshot)
        result.update({"params_requested": params, "param_snapshot": snapshot})

        wait_position_stable(master, min_samples=5, timeout_s=45)
        set_mode(master, "GUIDED", timeout_s=20)
        arm(master, timeout_s=45)
        command_takeoff(master, float(config["experiment"]["takeoff_alt_m"]))
        wait_altitude(master, float(config["experiment"]["takeoff_alt_m"]), timeout_s=75)
        hover = read_hover_thrust(master, default=0.5)
        throttle_pwm = int(round(1000 + 1000 * max(0.1, min(0.9, hover))))  # mid-ish throttle stick
        speedup = max(1.0, float(config["experiment"].get("speedup", 1.0)))
        hz = float(config["experiment"]["stream_hz"])
        dt_wall = 1.0 / (hz * speedup)

        set_mode(master, "ACRO", timeout_s=20)
        # Hold throttle to keep airborne while commanding a roll-rate doublet.
        roll_frac = float(spec.get("roll_stick_frac", 1.0))
        hold_s = float(spec.get("hold_s", 1.2))
        cycles = int(spec.get("cycles", 2))
        t0 = time.time()
        live: list[dict[str, Any]] = []
        for c in range(cycles):
            for sign in (+1.0, -1.0):
                roll_pwm = int(round(1500 + sign * roll_frac * 500))
                seg_end = time.time() + hold_s / speedup
                while time.time() < seg_end:
                    send_gcs_heartbeat(master)
                    send_rc_override(master, roll_pwm=roll_pwm, pitch_pwm=1500, throttle_pwm=throttle_pwm, yaw_pwm=1500)
                    time.sleep(dt_wall)
                    msg = master.recv_match(type="STATUSTEXT", blocking=False)
                    if msg is not None:
                        live.append({"t_s": time.time() - t0, "text": str(getattr(msg, "text", ""))})
        release_rc_override(master)
        # brief observation
        obs_end = time.time() + 4.0 / speedup
        while time.time() < obs_end:
            send_gcs_heartbeat(master)
            time.sleep(dt_wall)

        result["live_events"] = live[-40:]
        disarm_for_cleanup(master)
        result["cleanup_disarmed"] = wait_disarmed(master, 8.0)
        try:
            master.close()
        except Exception:
            pass
        master = None
        runner.stop()
        bin_path = runner.collect_dataflash(run_id)
        if bin_path is None:
            result["error"] = "No DataFlash .BIN log found after ACRO run"
            return result
        result["bin_path"] = str(bin_path)
        csv_path = PLANC_ROOT / "logs" / f"{run_id}_parsed.csv"
        # light parse: just extract attitude extrema + mode/err for corroboration
        _light_parse_acro(bin_path, csv_path)
        ext = attitude_extrema_from_csv(csv_path, None, None)
        result["attitude_extrema"] = ext
        amax = float(point["angle_max_cd"]) / 100.0
        result["acro_attitude_exceeds_angle_max"] = bool(ext.get("applicable") and float(ext.get("max_achieved_lean_deg", 0.0)) > amax + 1.0)
        result["angle_max_deg"] = amax
        return result
    except Exception as exc:  # noqa: BLE001
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        return result
    finally:
        if master is not None:
            try:
                release_rc_override(master)
            except Exception:
                pass
            try:
                disarm_for_cleanup(master)
            except Exception:
                pass
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def _light_parse_acro(bin_path: Path, csv_path: Path) -> None:
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    rows: list[dict[str, Any]] = []
    while True:
        msg = mlog.recv_match(type=["ATT", "MODE", "ERR", "MSG"], blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        d = msg.to_dict()
        t = d.get("TimeUS")
        if t is None:
            continue
        row = {"time_s": float(t) / 1e6, "type": msg.get_type()}
        row.update({k: v for k, v in d.items() if k not in {"mavpackettype", "TimeUS"}})
        rows.append(row)
    if not rows:
        return
    fields = ["time_s", "type"]
    extra = sorted({k for r in rows for k in r if k not in fields})
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields + extra)
        w.writeheader()
        for r in rows:
            w.writerow(r)


# --------------------------------------------------------------------------- #
# Grid builders
# --------------------------------------------------------------------------- #
def partB_points(config: dict[str, Any]) -> list[dict[str, Any]]:
    b = config["partB"]
    pts = []
    for tgt in b["target_deg"]:
        for seed in b["seeds"]:
            pts.append({
                "part": "B",
                "name": "reanchor",
                "interface": "guided_quaternion",
                "group": str(b.get("group", "B")),
                "guid_options": float(b.get("guid_options", config["baseline_params"].get("GUID_OPTIONS", 8))),
                "takeoff_alt_m": float(b.get("takeoff_alt_m", config["experiment"]["takeoff_alt_m"])),
                "axis": b["axis"],
                "target_deg": float(tgt),
                "slew_rate_deg_s": float(b["slew_rate_deg_s"]),
                "hold_s": float(b["hold_s"]),
                "seed": int(seed),
                "wind_m_s": float(b["wind_m_s"]),
                "turbulence_m_s": float(b["turbulence_m_s"]),
                "angle_max_cd": float(b["angle_max_cd"]),
                "model": str(b["model"]),
            })
    return pts


def partC_points(config: dict[str, Any], group_key: str) -> list[dict[str, Any]]:
    c = config[group_key]
    levels = c["pressure_levels"]
    pts = []
    group = str(c.get("group", group_key))
    name_prefix = "att_hold" if group == "C_attitude" else "alt_nohold"
    for tgt in c["target_deg"]:
        for lvl in levels:
            for seed in c["seeds"]:
                pts.append({
                    "part": "C",
                    "name": f"{name_prefix}_{lvl['name']}",
                    "interface": "guided_quaternion",
                    "group": group,
                    "altitude_hold": bool(c.get("altitude_hold", False)),
                    "suspect": bool(c.get("suspect", False)),
                    "guid_options": float(c.get("guid_options", config["baseline_params"].get("GUID_OPTIONS", 8))),
                    "takeoff_alt_m": float(c.get("takeoff_alt_m", config["experiment"]["takeoff_alt_m"])),
                    "axis": c["axis"],
                    "target_deg": float(tgt),
                    "slew_rate_deg_s": float(c["slew_rate_deg_s"]),
                    "hold_s": float(c["hold_s"]),
                    "seed": int(seed),
                    "wind_m_s": float(lvl["wind_m_s"]),
                    "turbulence_m_s": float(lvl["turbulence_m_s"]),
                    "angle_max_cd": float(c["angle_max_cd"]),
                    "model": str(c["model"]),
                })
    return pts


def fidelity_point(config: dict[str, Any], key: str) -> dict[str, Any]:
    spec = config["fidelity"][key]
    return {
        "part": "fid",
        "name": str(spec["name"]),
        "interface": str(spec.get("interface", "guided_quaternion")),
        "group": str(spec.get("group", "fid")),
        "guid_options": float(spec.get("guid_options", config["baseline_params"].get("GUID_OPTIONS", 8))),
        "takeoff_alt_m": float(spec.get("takeoff_alt_m", config["experiment"]["takeoff_alt_m"])),
        "axis": str(spec.get("axis", "roll")),
        "target_deg": float(spec["target_deg"]),
        "slew_rate_deg_s": float(spec.get("slew_rate_deg_s", 45.0)),
        "hold_s": float(spec.get("hold_s", 10.0)),
        "seed": int(spec.get("seed", 0)),
        "wind_m_s": float(spec.get("wind_m_s", 0.0)),
        "turbulence_m_s": float(spec.get("turbulence_m_s", 0.0)),
        "angle_max_cd": float(spec["angle_max_cd"]),
        "model": str(spec.get("model", "m100")),
    }


def run_cached(config: dict[str, Any], point: dict[str, Any], partial_path: Path, runs: list[dict[str, Any]], resume: bool) -> list[dict[str, Any]]:
    run_id = tierbc_run_id(config, point)
    if resume:
        for ex in runs:
            if ex.get("run_id") == run_id and not ex.get("error"):
                print(f"CACHED {run_id}", flush=True)
                return runs
    print(f"RUN {run_id}", flush=True)
    run = run_quaternion_once(config, point)
    runs = [r for r in runs if r.get("run_id") != run_id] + [run]
    write_json(partial_path, {"runs": runs, "updated_at_utc": utc_now()})
    lbl = run.get("interface_label", {})
    print(f"DONE {run_id} label={lbl.get('label')} outcome={lbl.get('outcome')} hardA={lbl.get('hard_A')} B={lbl.get('interface_aware_B')} cmdpeak={fmt(lbl.get('command_peak_deg'))} despeak={fmt(lbl.get('demanded_target_peak_deg'))} err={bool(run.get('error'))}", flush=True)
    return runs


def fidelity_status(config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    by_name = {str(r.get("point", {}).get("name")): r for r in runs if r.get("point", {}).get("part") == "fid"}

    def row(name: str) -> dict[str, Any]:
        r = by_name.get(name)
        if r is None:
            return {"name": name, "ok": False, "reason": "missing"}
        if r.get("error"):
            return {"name": name, "ok": False, "reason": str(r.get("error"))}
        ext = r.get("attitude_extrema", {})
        ha = r.get("hardened_oracle_A_v2") or r.get("hardened_oracle_A", {})
        return {"name": name, "ok": True, "run_id": r.get("run_id"), "extrema": ext, "oracle": ha}

    fb_name = str(config["fidelity"]["partB_quaternion"]["name"])
    fc_name = str(config["fidelity"]["partC_quaternion"]["name"])
    fw_name = str(config["fidelity"]["wind_upper_bound"]["name"])
    fb = row(fb_name)
    fc = row(fc_name)
    fw = row(fw_name)

    checks = []
    b_ok = bool(fb.get("ok") and (fb.get("extrema") or {}).get("desroll_exceeds_angle_max"))
    checks.append({"check": "fidB DesRoll exceeds ANGLE_MAX", "ok": b_ok, "run": fb.get("run_id")})

    fc_ext = fc.get("extrema") or {}
    target_c = float(config["fidelity"]["partC_quaternion"]["target_deg"])
    c_ok = bool(
        fc.get("ok")
        and fc_ext.get("desroll_le_angle_max")
        and float(fc_ext.get("max_abs_des_roll_deg") or 0.0) >= target_c - 3.0
    )
    checks.append({"check": "fidC DesRoll tracks target and stays <= ANGLE_MAX", "ok": c_ok, "run": fc.get("run_id")})

    fw_ha = fw.get("oracle") or {}
    wind_ok = bool(fw.get("ok") and not fw_ha.get("inside"))
    checks.append({"check": "highest legal wind 0 deg hover is non-catastrophic", "ok": wind_ok, "run": fw.get("run_id")})

    ok = all(c["ok"] for c in checks)
    return {"ok": ok, "status": "PASS" if ok else "NEEDS-TUNING", "checks": checks}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "tierBC_v2_config.yaml")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--stage", choices=["fidelity", "B", "C-attitude", "C-altitude", "C", "acro", "all"], default="all")
    args = parser.parse_args()

    config = load_yaml(args.config)
    stem = str(config["experiment"]["artifact_stem"])
    results_dir = PLANC_ROOT / "results"
    partial_path = results_dir / f"{stem}_partial.json"
    env_path = results_dir / f"env_{stem}.json"

    env = probe_environment(config, REPO_ROOT)
    write_env(env, env_path)

    runs: list[dict[str, Any]] = list(load_json(partial_path, {"runs": []}).get("runs", [])) if args.resume else []

    if args.stage in ("fidelity", "all"):
        for key in ("partB_quaternion", "partC_quaternion", "wind_upper_bound"):
            runs = run_cached(config, fidelity_point(config, key), partial_path, runs, args.resume)
        status = fidelity_status(config, runs)
        write_json(partial_path, {"runs": runs, "updated_at_utc": utc_now(), "fidelity_status": status})
        print(f"S1 {status['status']} {status['checks']}", flush=True)
        if args.stage == "all" and not status["ok"]:
            print("NEEDS-TUNING: S1 interface fidelity failed; stopping before Part B/C grids.", flush=True)
            return
    if args.stage in ("B", "all"):
        for point in partB_points(config):
            runs = run_cached(config, point, partial_path, runs, args.resume)
    if args.stage in ("C-attitude", "C", "all"):
        for point in partC_points(config, "partC_attitude"):
            runs = run_cached(config, point, partial_path, runs, args.resume)
    if args.stage in ("C-altitude", "C", "all"):
        for point in partC_points(config, "partC_altitude"):
            runs = run_cached(config, point, partial_path, runs, args.resume)
    if args.stage in ("acro",):
        run = run_acro_once(config, config["fidelity"]["acro_trainer0"])
        runs = [r for r in runs if r.get("run_id") != run.get("run_id")] + [run]
        write_json(partial_path, {"runs": runs, "updated_at_utc": utc_now()})
        print(f"DONE acro {run.get('run_id')} exceeds_amax={run.get('acro_attitude_exceeds_angle_max')} err={bool(run.get('error'))}", flush=True)

    final_payload: dict[str, Any] = {"runs": runs, "updated_at_utc": utc_now()}
    if any(r.get("point", {}).get("part") == "fid" for r in runs):
        final_payload["fidelity_status"] = fidelity_status(config, runs)
    write_json(partial_path, final_payload)
    print(f"STAGE {args.stage} complete: {len(runs)} runs cached at {partial_path}", flush=True)


if __name__ == "__main__":
    main()
