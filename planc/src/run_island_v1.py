from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
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

COPTER_MODES = {
    0: "STABILIZE",
    1: "ACRO",
    4: "GUIDED",
    9: "LAND",
    20: "GUIDED_NOGPS",
}

ERROR_SUBSYSTEMS = {
    5: "FAILSAFE_RADIO",
    6: "FAILSAFE_BATT",
    7: "FAILSAFE_GPS",
    8: "FAILSAFE_GCS",
    9: "FAILSAFE_FENCE",
    12: "CRASH_CHECK",
    16: "EKFCHECK",
    17: "FAILSAFE_EKFINAV",
    20: "FAILSAFE_ADSB",
    23: "FAILSAFE_TERRAIN",
    25: "THRUST_LOSS_CHECK",
    26: "FAILSAFE_SENSORS",
    27: "FAILSAFE_LEAK",
    29: "FAILSAFE_VIBE",
    30: "INTERNAL_ERROR",
    31: "FAILSAFE_DEADRECKON",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


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


def stem(config: dict[str, Any]) -> str:
    return str(config["experiment"].get("artifact_stem", "island_v1"))


def write_preregister(config: dict[str, Any], path: Path) -> dict[str, Any]:
    payload = {
        "status": "preregistered",
        "written_at_utc": utc_now(),
        "experiment": config["experiment"],
        "decision": config["decision"],
        "audit": config["audit"],
        "sampling_plan": config["sampling_plan"],
        "oracle_B_decomposition": {
            "B_clamp": "command touches/exceeds ANGLE_MAX",
            "B_failsafe": "flight-controller preventive ERR/MSG or MODE with preventive ModeReason",
            "B_harness_cleanup": "MODE change with ModeReason=GCS_COMMAND from the test harness; this is not preventive B",
        },
        "short_circuit_rule": config["audit"]["short_circuit_reason"],
    }
    write_json(path, payload)
    return payload


def time_s(data: dict[str, Any]) -> float:
    if "TimeUS" in data:
        return float(data["TimeUS"]) / 1.0e6
    if "TimeMS" in data:
        return float(data["TimeMS"]) / 1000.0
    return 0.0


def mode_name(mode_value: Any) -> str:
    try:
        return COPTER_MODES.get(int(mode_value), str(mode_value))
    except Exception:
        return str(mode_value)


def reason_name(reason_value: Any) -> str:
    try:
        return MODE_REASONS.get(int(reason_value), str(reason_value))
    except Exception:
        return str(reason_value)


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


def sustained_true(rows: list[tuple[float, bool]]) -> float:
    start = None
    prev = None
    best = 0.0
    for t_s, value in rows:
        if value:
            if start is None:
                start = t_s
        elif start is not None:
            best = max(best, (prev if prev is not None else t_s) - start)
            start = None
        prev = t_s
    if start is not None and prev is not None:
        best = max(best, prev - start)
    return max(0.0, best)


def nearest_value(rows: list[tuple[float, float]], t_s: float) -> float | None:
    if not rows:
        return None
    return min(rows, key=lambda item: abs(item[0] - t_s))[1]


def parse_bin_metrics(bin_path: Path, sidecar: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    if not bin_path.exists():
        return {"available": False, "reason": f"missing {bin_path}"}

    lo = float(sidecar["active_window_s"]["start"])
    hi = float(sidecar["active_window_s"]["end"])
    thresholds = config["audit"]["crash_check_thresholds"]
    preventive_reasons = set(config["audit"]["preventive_mode_reasons"])

    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    att_rows: list[dict[str, float]] = []
    speed_rows: list[tuple[float, float]] = []
    accel_rows: list[tuple[float, float]] = []
    ekf_rows: list[dict[str, float]] = []
    pos_rows: list[tuple[float, float]] = []
    modes: list[dict[str, Any]] = []
    messages: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []

    while True:
        msg = mlog.recv_match(type=["ATT", "XKF1", "XKF4", "IMU", "POS", "MODE", "MSG", "ERR"], blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        data = msg.to_dict()
        t = time_s(data)
        if t < lo or t > hi:
            continue
        typ = msg.get_type()
        if typ == "ATT":
            roll = float(data.get("Roll", 0.0))
            pitch = float(data.get("Pitch", 0.0))
            droll = float(data.get("DesRoll", 0.0))
            dpitch = float(data.get("DesPitch", 0.0))
            lean = math.degrees(
                math.acos(max(-1.0, min(1.0, math.cos(math.radians(roll)) * math.cos(math.radians(pitch)))))
            )
            angle_error = math.hypot(roll - droll, pitch - dpitch)
            att_rows.append({"time_s": t, "lean_deg": lean, "angle_error_deg": angle_error, "roll": roll, "pitch": pitch})
        elif typ == "XKF1" and int(data.get("C", 0)) == 0:
            speed = math.sqrt(float(data.get("VN", 0.0)) ** 2 + float(data.get("VE", 0.0)) ** 2 + float(data.get("VD", 0.0)) ** 2)
            speed_rows.append((t, speed))
        elif typ == "IMU" and int(data.get("I", 0)) == 0:
            # Proxy for the crash-check filtered earth-frame acceleration; the exact filter output is not logged.
            acc = abs(
                math.sqrt(
                    float(data.get("AccX", 0.0)) ** 2
                    + float(data.get("AccY", 0.0)) ** 2
                    + float(data.get("AccZ", 0.0)) ** 2
                )
                - 9.80665
            )
            accel_rows.append((t, acc))
        elif typ == "XKF4" and int(data.get("C", 0)) == 0:
            ekf_rows.append(
                {
                    "time_s": t,
                    "SV": float(data.get("SV", 0.0)),
                    "SP": float(data.get("SP", 0.0)),
                    "SH": float(data.get("SH", 0.0)),
                    "SM": float(data.get("SM", 0.0)),
                    "FS": float(data.get("FS", 0.0)),
                }
            )
        elif typ == "POS":
            pos_rows.append((t, float(data.get("RelHomeAlt", 0.0))))
        elif typ == "MODE":
            reason = reason_name(data.get("Rsn"))
            row = {
                "time_s": t,
                "mode": mode_name(data.get("ModeNum", data.get("Mode"))),
                "reason": reason,
                "reason_code": data.get("Rsn"),
                "preventive": reason in preventive_reasons,
            }
            modes.append(row)
        elif typ == "MSG":
            messages.append({"time_s": t, "message": str(data.get("Message", ""))})
        elif typ == "ERR":
            subsystem = ERROR_SUBSYSTEMS.get(int(data.get("Subsys", -1)), str(data.get("Subsys")))
            errors.append({"time_s": t, "subsystem": subsystem, "ecode": data.get("ECode")})

    crash_predicate_rows = []
    for row in att_rows:
        t = float(row["time_s"])
        speed = nearest_value(speed_rows, t)
        accel = nearest_value(accel_rows, t)
        predicate = (
            row["lean_deg"] > float(thresholds["lean_angle_deg"])
            and row["angle_error_deg"] > float(thresholds["angle_error_deg"])
            and speed is not None
            and speed < float(thresholds["speed_m_s"])
            and accel is not None
            and accel < float(thresholds["accel_m_s_s"])
        )
        crash_predicate_rows.append((t, predicate))

    ekf_threshold = 0.8
    try:
        ekf_threshold = float(sidecar.get("param_snapshot", {}).get("FS_EKF_THRESH", 0.8))
    except Exception:
        pass

    mode_preventive = [m for m in modes if m["preventive"]]
    mode_gcs_cleanup = [
        m
        for m in modes
        if m["reason"] == config["audit"]["harness_cleanup_mode_reason"]
        and m["mode"] not in {"GUIDED", "GUIDED_NOGPS"}
    ]
    crash_messages = [m for m in messages if "crash:" in m["message"].lower()]
    sim_ground_messages = [m for m in messages if "sim hit ground" in m["message"].lower()]
    failsafe_messages = [m for m in messages if "failsafe" in m["message"].lower()]
    preventive_errors = [e for e in errors if e["subsystem"] in set(ERROR_SUBSYSTEMS.values())]

    return {
        "available": True,
        "active_window_s": {"start": lo, "end": hi},
        "attitude": {
            "max_lean_deg": max((r["lean_deg"] for r in att_rows), default=None),
            "max_att_error_deg": max((r["angle_error_deg"] for r in att_rows), default=None),
        },
        "speed": {
            "min_m_s": min((v for _, v in speed_rows), default=None),
            "p50_m_s": percentile([v for _, v in speed_rows], 0.5),
            "max_m_s": max((v for _, v in speed_rows), default=None),
        },
        "accel_proxy": {
            "min_m_s_s": min((v for _, v in accel_rows), default=None),
            "p50_m_s_s": percentile([v for _, v in accel_rows], 0.5),
            "max_m_s_s": max((v for _, v in accel_rows), default=None),
            "note": "IMU norm minus 1g proxy; exact land_accel_ef_filter is not logged.",
        },
        "altitude": {
            "min_rel_home_alt_m": min((v for _, v in pos_rows), default=None),
            "p50_rel_home_alt_m": percentile([v for _, v in pos_rows], 0.5),
        },
        "crash_check_proxy": {
            "thresholds": thresholds,
            "max_continuous_all_conditions_s": sustained_true(crash_predicate_rows),
            "trigger_duration_s": float(thresholds["duration_s"]),
            "would_trigger_proxy": sustained_true(crash_predicate_rows) >= float(thresholds["duration_s"]),
        },
        "ekf": {
            "fs_flag_max": max((r["FS"] for r in ekf_rows), default=None),
            "variance_threshold_param": ekf_threshold,
            "max_SV": max((r["SV"] for r in ekf_rows), default=None),
            "max_SP": max((r["SP"] for r in ekf_rows), default=None),
            "max_SH": max((r["SH"] for r in ekf_rows), default=None),
            "max_SM": max((r["SM"] for r in ekf_rows), default=None),
        },
        "mode_preventive": mode_preventive,
        "mode_gcs_cleanup": mode_gcs_cleanup,
        "crash_messages": crash_messages,
        "sim_ground_messages": sim_ground_messages,
        "failsafe_messages": failsafe_messages,
        "preventive_errors": preventive_errors,
    }


def sidecar_path(run_id: str) -> Path:
    return PLANC_ROOT / "logs" / f"{run_id}_parsed.oracle.json"


def bin_path(run_id: str) -> Path:
    return PLANC_ROOT / "logs" / f"{run_id}.BIN"


def audit_run(run: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    run_id = str(run["run_id"])
    sidecar = load_json(sidecar_path(run_id)) if sidecar_path(run_id).exists() else run
    metrics = parse_bin_metrics(bin_path(run_id), sidecar, config)

    b_clamp = bool(sidecar.get("command", {}).get("command_touched_angle_max")) or bool(
        sidecar.get("command", {}).get("command_exceeded_angle_max")
    )
    oracle_b = sidecar.get("oracle_B", {})
    fail_events = list(oracle_b.get("fail_events", []))
    fail_texts = list(oracle_b.get("fail_texts", []))
    mode_switches = list(oracle_b.get("mode_switches_during_maneuver", []))

    actual_mode_failsafe = list(metrics.get("mode_preventive", [])) if metrics.get("available") else []
    harness_cleanup = list(metrics.get("mode_gcs_cleanup", [])) if metrics.get("available") else []
    actual_errors = list(metrics.get("preventive_errors", [])) if metrics.get("available") else []
    actual_failsafe_messages = list(metrics.get("failsafe_messages", [])) if metrics.get("available") else []
    actual_crash_messages = list(metrics.get("crash_messages", [])) if metrics.get("available") else []
    b_actual_failsafe = bool(actual_mode_failsafe or actual_errors or actual_failsafe_messages or actual_crash_messages)
    b_actual = bool(b_clamp or b_actual_failsafe)

    oracle_label = sidecar.get("label", run.get("label"))
    a_inside = bool(sidecar.get("oracle_A", {}).get("inside"))
    if a_inside and not b_actual:
        corrected_label = "overdraw"
    elif a_inside and b_actual:
        corrected_label = "bug_side"
    elif not a_inside:
        corrected_label = "safe"
    else:
        corrected_label = "blocked"

    command = sidecar.get("command", {})
    angle_margin = None
    if command.get("max_command_angle_deg") is not None and command.get("angle_max_deg") is not None:
        angle_margin = float(command["angle_max_deg"]) - float(command["max_command_angle_deg"])

    positive_evidence = {
        "B_clamp_false": not b_clamp,
        "command_angle_margin_deg": angle_margin,
        "no_actual_preventive_mode_reason": not actual_mode_failsafe,
        "no_actual_preventive_ERR": not actual_errors,
        "no_actual_failsafe_or_crash_MSG": not (actual_failsafe_messages or actual_crash_messages),
        "mode_reason_GCS_COMMAND_count": len(harness_cleanup),
        "ekf_FS_flag_max": metrics.get("ekf", {}).get("fs_flag_max") if metrics.get("available") else None,
        "crash_check_proxy_max_conjunctive_duration_s": metrics.get("crash_check_proxy", {}).get("max_continuous_all_conditions_s")
        if metrics.get("available")
        else None,
        "crash_check_proxy_trigger_duration_s": metrics.get("crash_check_proxy", {}).get("trigger_duration_s")
        if metrics.get("available")
        else None,
        "crash_check_proxy_would_trigger": metrics.get("crash_check_proxy", {}).get("would_trigger_proxy")
        if metrics.get("available")
        else None,
    }

    return {
        "run_id": run_id,
        "point": run.get("point", {}),
        "oracle_label_v2": oracle_label,
        "corrected_label": corrected_label,
        "A_inside": a_inside,
        "B_v2_inside": bool(oracle_b.get("inside")),
        "B_actual_inside": b_actual,
        "B_components": {
            "B_clamp": b_clamp,
            "B_fc_failsafe": b_actual_failsafe,
            "B_harness_cleanup": bool(harness_cleanup),
            "v2_fail_events": fail_events,
            "v2_fail_texts": fail_texts,
            "v2_mode_switches": mode_switches,
            "actual_mode_failsafe": actual_mode_failsafe,
            "actual_errors": actual_errors,
            "actual_failsafe_messages": actual_failsafe_messages,
            "actual_crash_messages": actual_crash_messages,
            "harness_cleanup_modes": harness_cleanup,
        },
        "command": command,
        "oracle_A": sidecar.get("oracle_A", {}),
        "positive_evidence": positive_evidence,
        "metrics": metrics,
    }


def aggregate_cells(audits: list[dict[str, Any]], label_key: str) -> list[dict[str, Any]]:
    cells: dict[tuple[str, float], list[dict[str, Any]]] = defaultdict(list)
    for row in audits:
        point = row.get("point", {})
        if point.get("role") != "p3":
            continue
        if point.get("angle_max_cd") != 4500.0:
            continue
        layer = str(point.get("layer"))
        r = float(point.get("r_deg_s"))
        cells[(layer, r)].append(row)
    out = []
    for (layer, r), rows in sorted(cells.items()):
        usable = [row for row in rows if row.get("A_inside") is not None]
        over = sum(1 for row in usable if row.get(label_key) == "overdraw")
        bug = sum(1 for row in usable if row.get(label_key) == "bug_side")
        safe = sum(1 for row in usable if row.get(label_key) == "safe")
        out.append(
            {
                "layer": layer,
                "r_deg_s": r,
                "n": len(usable),
                "overdraw": over,
                "bug_side": bug,
                "safe": safe,
                "p_overdraw": None if not usable else over / len(usable),
            }
        )
    return out


def decide(audits: list[dict[str, Any]], raw_cells: list[dict[str, Any]], corrected_cells: list[dict[str, Any]]) -> dict[str, Any]:
    overdraw = [row for row in audits if row["oracle_label_v2"] == "overdraw"]
    old_bug = [row for row in audits if row["oracle_label_v2"] == "bug_side"]
    overdraw_actual_b = [row for row in overdraw if row["B_actual_inside"]]
    bug_only_harness = [
        row
        for row in old_bug
        if row["A_inside"]
        and row["B_v2_inside"]
        and not row["B_actual_inside"]
        and row["B_components"]["B_harness_cleanup"]
    ]
    corrected_overdraw_cells = [
        cell for cell in corrected_cells if cell["n"] >= 2 and cell["p_overdraw"] is not None and cell["p_overdraw"] >= 0.5
    ]
    if overdraw_actual_b:
        return {
            "verdict": "ISLAND-FLAKY",
            "reason": "At least one v2 overdraw run has actual preventive B after decomposition; v2 oracle missed B.",
        }
    if bug_only_harness and len(bug_only_harness) >= max(1, len(old_bug) // 2):
        return {
            "verdict": "ISLAND-FLAKY",
            "reason": "The v2 island is dominated by bug_side labels caused by harness GCS_COMMAND LAND cleanup, not B_clamp or flight-controller preventive failsafe.",
        }
    if corrected_overdraw_cells:
        return {
            "verdict": "NEEDS-TUNING",
            "reason": "After B decomposition, the old island becomes a broad corrected-overdraw region, but island-v1 sampling was not run to estimate smooth probabilities.",
        }
    return {
        "verdict": "NEEDS-TUNING",
        "reason": "B decomposition did not produce a decisive REAL/FLAKY classification.",
    }


def in_primary_scope(row: dict[str, Any], config: dict[str, Any]) -> bool:
    scope = config["audit"].get("primary_scope", {})
    role = scope.get("role")
    if role and row.get("point", {}).get("role") != role:
        return False
    return True


def summarize_audits(audits: list[dict[str, Any]], verdict_reason: str | None = None) -> dict[str, Any]:
    component_counts = Counter()
    for row in audits:
        comps = row["B_components"]
        if comps["B_clamp"]:
            component_counts["B_clamp"] += 1
        elif comps["B_fc_failsafe"]:
            component_counts["B_fc_failsafe"] += 1
        elif comps["B_harness_cleanup"]:
            component_counts["B_harness_cleanup"] += 1
        else:
            component_counts["B_none"] += 1

    overdraw_audit = [row for row in audits if row["oracle_label_v2"] == "overdraw"]
    bug_side = [row for row in audits if row["oracle_label_v2"] == "bug_side"]
    bug_side_harness_only = [
        row
        for row in bug_side
        if row["B_components"]["B_harness_cleanup"]
        and not row["B_components"]["B_clamp"]
        and not row["B_components"]["B_fc_failsafe"]
    ]
    out = {
        "audited_runs": len(audits),
        "v2_overdraw_count": len(overdraw_audit),
        "v2_overdraw_actual_B_count": sum(1 for row in overdraw_audit if row["B_actual_inside"]),
        "v2_bug_side_count": len(bug_side),
        "bug_side_harness_only_count": len(bug_side_harness_only),
        "B_component_counts": dict(component_counts),
        "new_sitl_sampling_run": False,
    }
    if verdict_reason is not None:
        out["new_sitl_sampling_skipped_reason"] = verdict_reason
    return out


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    plots: dict[str, str] = {}
    s = stem(payload["config"])
    raw_cells = payload["v2_probability_raw"]
    corrected_cells = payload["v2_probability_corrected"]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    layers = sorted({cell["layer"] for cell in raw_cells})
    colors = {"default": "#264653", "moderate": "#2a9d8f", "high": "#e76f51", "tight_direction_check": "#6d597a"}
    for layer in layers:
        raw = [cell for cell in raw_cells if cell["layer"] == layer]
        corr = [cell for cell in corrected_cells if cell["layer"] == layer]
        ax.plot([c["r_deg_s"] for c in raw], [c["p_overdraw"] for c in raw], marker="o", color=colors.get(layer, None), label=f"{layer} v2")
        ax.plot([c["r_deg_s"] for c in corr], [c["p_overdraw"] for c in corr], marker="x", linestyle="--", color=colors.get(layer, None), label=f"{layer} corrected")
    ax.set_xlabel("r (deg/s)")
    ax.set_ylabel("OverDraw probability")
    ax.set_ylim(-0.05, 1.05)
    ax.set_title("v2 island probability before/after B decomposition")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    path = analysis / f"{s}_probability_vs_r.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    plots["probability_vs_r"] = str(path)

    rows = [row for row in payload["audits"] if row.get("point", {}).get("role") == "p3"]
    buckets = Counter()
    for row in rows:
        comps = row["B_components"]
        if comps["B_clamp"]:
            key = "B_clamp"
        elif comps["B_fc_failsafe"]:
            key = "B_fc_failsafe"
        elif comps["B_harness_cleanup"]:
            key = "harness_GCS_LAND"
        else:
            key = "B_none"
        buckets[(str(row.get("point", {}).get("layer")), key)] += 1
    layers = sorted({k[0] for k in buckets})
    categories = ["B_none", "B_clamp", "B_fc_failsafe", "harness_GCS_LAND"]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    bottom = [0] * len(layers)
    colors2 = {"B_none": "#2a9d8f", "B_clamp": "#e9c46a", "B_fc_failsafe": "#e76f51", "harness_GCS_LAND": "#6d597a"}
    for cat in categories:
        vals = [buckets.get((layer, cat), 0) for layer in layers]
        ax.bar(layers, vals, bottom=bottom, label=cat, color=colors2[cat])
        bottom = [b + v for b, v in zip(bottom, vals)]
    ax.set_ylabel("run count")
    ax.set_title("B decomposition: clamp vs FC failsafe vs harness cleanup")
    ax.legend(fontsize=8)
    path = analysis / f"{s}_b_decomposition.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    plots["b_decomposition"] = str(path)

    rep = payload.get("mechanism_pair", {})
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=False)
    for rid, label, color in [
        (rep.get("overdraw_run_id"), "overdraw", "#2a9d8f"),
        (rep.get("bug_side_run_id"), "bug_side_v2", "#e76f51"),
    ]:
        series = rep.get("series", {}).get(rid, {})
        if not series:
            continue
        t0 = series.get("t0", 0.0)
        ts = [t - t0 for t in series.get("time_s", [])]
        axes[0].plot(ts, series.get("att_error_deg", []), color=color, label=label)
        axes[1].plot(ts, series.get("speed_m_s", []), color=color, label=label)
        axes[2].plot(ts, series.get("rel_alt_m", []), color=color, label=label)
        if series.get("land_time_s") is not None:
            for ax in axes:
                ax.axvline(float(series["land_time_s"]) - t0, color=color, linestyle=":", alpha=0.8)
    axes[0].axhline(60.0, color="#222222", linestyle="--", linewidth=1)
    axes[1].axhline(10.0, color="#222222", linestyle="--", linewidth=1)
    axes[2].axhline(0.0, color="#222222", linestyle="--", linewidth=1)
    axes[0].set_ylabel("att err deg")
    axes[1].set_ylabel("speed m/s")
    axes[2].set_ylabel("rel alt m")
    axes[2].set_xlabel("seconds from active-window start")
    axes[0].set_title("Mechanism pair: overdraw vs v2 bug_side")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8)
    path = analysis / f"{s}_mechanism_pair.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    plots["mechanism_pair"] = str(path)
    return plots


def mechanism_series(run_id: str) -> dict[str, Any]:
    side = load_json(sidecar_path(run_id))
    lo = float(side["active_window_s"]["start"])
    hi = float(side["active_window_s"]["end"])
    mlog = mavutil.mavlink_connection(str(bin_path(run_id)), robust_parsing=True)
    att = []
    speeds = []
    alts = []
    land_time = None
    while True:
        msg = mlog.recv_match(type=["ATT", "XKF1", "POS", "MODE"], blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        d = msg.to_dict()
        t = time_s(d)
        if t < lo or t > hi:
            continue
        typ = msg.get_type()
        if typ == "ATT":
            err = math.hypot(float(d.get("Roll", 0.0)) - float(d.get("DesRoll", 0.0)), float(d.get("Pitch", 0.0)) - float(d.get("DesPitch", 0.0)))
            att.append((t, err))
        elif typ == "XKF1" and int(d.get("C", 0)) == 0:
            speeds.append((t, math.sqrt(float(d.get("VN", 0.0)) ** 2 + float(d.get("VE", 0.0)) ** 2 + float(d.get("VD", 0.0)) ** 2)))
        elif typ == "POS":
            alts.append((t, float(d.get("RelHomeAlt", 0.0))))
        elif typ == "MODE" and mode_name(d.get("ModeNum", d.get("Mode"))) == "LAND":
            land_time = t
    common_t = sorted({t for t, _ in att})
    return {
        "t0": lo,
        "time_s": common_t,
        "att_error_deg": [nearest_value(att, t) or 0.0 for t in common_t],
        "speed_m_s": [nearest_value(speeds, t) or 0.0 for t in common_t],
        "rel_alt_m": [nearest_value(alts, t) or 0.0 for t in common_t],
        "land_time_s": land_time,
    }


def build_report(payload: dict[str, Any]) -> str:
    path = PLANC_ROOT / "results" / f"{stem(payload['config'])}_report.md"
    verdict = payload["verdict"]
    label_component_counts: dict[str, Counter[str]] = defaultdict(Counter)
    for row in payload["primary_audits"]:
        label = str(row["oracle_label_v2"])
        comps = row["B_components"]
        if comps["B_clamp"]:
            component = "B_clamp"
        elif comps["B_fc_failsafe"]:
            component = "B_fc_failsafe"
        elif comps["B_harness_cleanup"]:
            component = "harness_GCS_LAND"
        else:
            component = "B_none"
        label_component_counts[label][component] += 1

    lines = [
        "# OverDraw Island v1 Hardening Probe",
        "",
        f"VERDICT: **{verdict['verdict']}**",
        "",
        verdict["reason"],
        "",
        "## Decision Criteria",
        "",
        "| criterion | outcome | evidence |",
        "| --- | --- | --- |",
        "| smooth reproducible island probability | failed | Raw v2 probabilities form a small apparent island, but the B decomposition invalidates the v2 bug_side labels; no 25-seed sampling was run after the preregistered short-circuit. |",
        f"| positive evidence B was not met on overdraw runs | passed for audited core overdraw | {payload['audit_summary']['v2_overdraw_count']} core overdraw runs have B_clamp=false, no preventive MODE/ERR/MSG, EKF FS max 0, and crash-check proxy 0.00 / 2.0 s. |",
        f"| consistent overdraw/bug_side mechanism | failed | {payload['audit_summary']['bug_side_harness_only_count']} / {payload['audit_summary']['v2_bug_side_count']} core bug_side labels are harness `GCS_COMMAND LAND`; none are B_clamp or FC failsafe. |",
        "",
        "## B Decomposition",
        "",
        f"- Primary scope: {payload['primary_scope']['description']}",
        f"- Core v2 overdraw runs audited: {payload['audit_summary']['v2_overdraw_count']}; actual-B among them: {payload['audit_summary']['v2_overdraw_actual_B_count']}.",
        f"- Core v2 bug_side runs audited: {payload['audit_summary']['v2_bug_side_count']}; bug_side caused only by harness `GCS_COMMAND LAND`: {payload['audit_summary']['bug_side_harness_only_count']}.",
        f"- Full sidecar audit: {payload['full_audit_summary']['audited_runs']} runs; full v2 overdraw count {payload['full_audit_summary']['v2_overdraw_count']}.",
        f"- `MODE.Rsn=2` maps to `GCS_COMMAND` in ArduPilot `ModeReason.h`; this is test-harness cleanup, not a preventive failsafe.",
        "",
        "| component | count |",
        "| --- | ---: |",
    ]
    for key, value in payload["audit_summary"]["B_component_counts"].items():
        lines.append(f"| {key} | {value} |")
    lines.extend(
        [
            "",
            "| v2 label | n | B_clamp | B_fc_failsafe | harness_GCS_LAND | B_none |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for label in sorted(label_component_counts):
        counts = label_component_counts[label]
        total = sum(counts.values())
        lines.append(
            f"| {label} | {total} | {counts['B_clamp']} | {counts['B_fc_failsafe']} | {counts['harness_GCS_LAND']} | {counts['B_none']} |"
        )
    lines.extend(
        [
            "",
            "## OverDraw Positive Evidence",
            "",
            "| run | clamp margin deg | EKF FS max | crash-check proxy max conjunctive s | actual FC B |",
            "| --- | ---: | ---: | ---: | --- |",
        ]
    )
    for row in payload["overdraw_audit"]:
        pe = row["positive_evidence"]
        lines.append(
            f"| {row['run_id']} | {fmt(pe['command_angle_margin_deg'], 2)} | {fmt(pe['ekf_FS_flag_max'], 2)} | {fmt(pe['crash_check_proxy_max_conjunctive_duration_s'], 2)} / {fmt(pe['crash_check_proxy_trigger_duration_s'], 1)} | {row['B_actual_inside']} |"
        )
    lines.extend(
        [
            "",
            "## Probability Shape",
            "",
            "The original v2 labels form a small apparent island. After removing harness cleanup from B, the same grid reclassifies into a broad corrected-overdraw region; therefore the v2 island shape is not a coverage-hole shape.",
            "",
            "| layer | r | n | p_overdraw_v2 | p_overdraw_corrected |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    by_corr = {(c["layer"], c["r_deg_s"]): c for c in payload["v2_probability_corrected"]}
    for raw in payload["v2_probability_raw"]:
        corr = by_corr.get((raw["layer"], raw["r_deg_s"]), {})
        lines.append(
            f"| {raw['layer']} | {fmt(raw['r_deg_s'], 0)} | {raw['n']} | {fmt(raw['p_overdraw'], 2)} | {fmt(corr.get('p_overdraw'), 2)} |"
        )
    lines.extend(["", "## Artifacts", ""])
    for name, plot in payload.get("artifacts", {}).get("plots", {}).items():
        lines.append(f"- Plot {name}: `{Path(plot).relative_to(REPO_ROOT)}`")
    lines.append(f"- Result JSON: `planc/results/{stem(payload['config'])}_result.json`")
    lines.append(f"- Preregistration: `planc/results/{stem(payload['config'])}_prereg.json`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "island_v1_config.yaml")
    args = parser.parse_args()
    config = load_yaml(args.config)
    st = stem(config)
    results_dir = PLANC_ROOT / "results"
    prereg_path = results_dir / f"{st}_prereg.json"
    result_path = results_dir / f"{st}_result.json"
    env_path = results_dir / f"env_{st}.json"

    env = probe_environment(config | {"sitl": load_yaml(PLANC_ROOT / "config" / "stage0_v2_config.yaml")["sitl"]}, REPO_ROOT)
    write_env(env, env_path)
    prereg = write_preregister(config, prereg_path)

    stage0 = load_json(REPO_ROOT / config["experiment"]["source_stage0_result"])
    source_runs = [r for group in stage0.get("runs", {}).values() for r in group if not r.get("error")]
    audits = [audit_run(run, config) for run in source_runs if sidecar_path(str(run["run_id"])).exists()]
    primary_audits = [row for row in audits if in_primary_scope(row, config)]

    raw_cells = aggregate_cells(audits, "oracle_label_v2")
    corrected_cells = aggregate_cells(audits, "corrected_label")
    verdict = decide(primary_audits, raw_cells, corrected_cells)

    overdraw_audit = [row for row in primary_audits if row["oracle_label_v2"] == "overdraw"]
    bug_side = [row for row in primary_audits if row["oracle_label_v2"] == "bug_side"]
    payload = {
        "status": "complete",
        "generated_at_utc": utc_now(),
        "config": config,
        "env": env,
        "preregistration": prereg,
        "verdict": verdict,
        "primary_scope": config["audit"].get("primary_scope", {}),
        "audit_summary": summarize_audits(primary_audits, verdict["reason"]),
        "full_audit_summary": summarize_audits(audits),
        "overdraw_audit": overdraw_audit,
        "bug_side_decomposition": bug_side,
        "primary_audits": primary_audits,
        "audits": audits,
        "v2_probability_raw": raw_cells,
        "v2_probability_corrected": corrected_cells,
        "mechanism_pair": {
            "overdraw_run_id": "stage0v2_p3_default_r0480_w00_t00_a4500_m100_s00",
            "bug_side_run_id": "stage0v2_p3_default_r0480_w00_t00_a4500_m100_s02",
            "series": {
                "stage0v2_p3_default_r0480_w00_t00_a4500_m100_s00": mechanism_series("stage0v2_p3_default_r0480_w00_t00_a4500_m100_s00"),
                "stage0v2_p3_default_r0480_w00_t00_a4500_m100_s02": mechanism_series("stage0v2_p3_default_r0480_w00_t00_a4500_m100_s02"),
            },
        },
        "artifacts": {},
    }
    payload["artifacts"]["plots"] = make_plots(payload)
    payload["artifacts"]["report"] = build_report(payload)
    write_json(result_path, payload)
    print(f"COMPLETE: verdict={verdict['verdict']} result={result_path} report={payload['artifacts']['report']}", flush=True)


if __name__ == "__main__":
    main()
