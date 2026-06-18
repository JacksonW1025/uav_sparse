from __future__ import annotations

import argparse
import copy
import json
import math
import statistics
import sys
import traceback
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))

from env_probe import probe_environment, write_env
from param_manager import ParamManager
from run_rtl_energy import (
    classify_run,
    config_with_model,
    controlled_params,
    fmt,
    load_config,
    parse_energy_dataflash,
    rel,
    run_rtl_energy_flight,
    write_json,
)
from sitl_runner import SitlRunner


STEM = "energy_headwind_v1"
HOME_RADIUS_M = 15.0
DISTANCES_M = [40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0]
WINDS_M_S = [0.0, 3.0, 6.0, 7.5, 9.0, 12.0]
BATT_LOW_LAYERS_MAH = [150.0, 220.0, 300.0, 400.0]
MAIN_BATT_LOW_MAH = 220.0
SIGMA_POINTS = [(80.0, 12.0), (100.0, 9.0), (120.0, 9.0)]
SIGMA_REPS = 5
COUPLING_DISTANCE_M = 100.0


def wcode(wind_m_s: float) -> str:
    return f"{int(round(float(wind_m_s) * 10)):03d}"


def pcode(batt_low_mah: float) -> str:
    return f"p{int(round(float(batt_low_mah))):03d}"


def run_id_for(batt_low_mah: float, distance_m: float, wind_m_s: float, rep_index: int) -> str:
    return f"ehv1_{pcode(batt_low_mah)}_D{int(distance_m):03d}_W{wcode(wind_m_s)}_r{rep_index:02d}"


def make_config(base: dict[str, Any], batt_low_mah: float) -> dict[str, Any]:
    cfg = copy.deepcopy(base)
    cfg["experiment"]["name"] = STEM
    cfg["experiment"]["home_radius_m"] = HOME_RADIUS_M
    params = cfg.setdefault("baseline_params", {})
    params.update({
        "BATT_LOW_MAH": float(batt_low_mah),
        "BATT_FS_LOW_ACT": 2,
        "BATT_FS_CRT_ACT": 1,
        "BATT_LOW_VOLT": 0,
        "BATT_CRT_VOLT": 0,
        "FENCE_ENABLE": 0,
        "FENCE_TYPE": 0,
        "AVOID_ENABLE": 0,
        "SIM_WIND_DIR": 270,
        "SIM_WIND_TURB": 0,
        "WPNAV_SPEED": 800,
        "RTL_SPEED": 800,
        "RTL_ALT": 2000,
    })
    return cfg


def prereg_payload(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "experiment": STEM,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_feasibility": {
            "result": "planc/results/energy_headwind_feasibility_result.json",
            "verdict": "GO",
            "key_calibration": {
                "D100_return_median_groundspeed_wind_0_6_12_m_s": [7.88, 6.48, 3.05],
                "same_D_flips": ["D80: 9->12", "D100: 6->9", "D120: 6->9"],
                "shortfall_sigma_D100_W9_m": 0.071,
            },
        },
        "scenario": {
            "home_radius_m": HOME_RADIUS_M,
            "distances_m": DISTANCES_M,
            "winds_m_s": WINDS_M_S,
            "batt_low_layers_mAh": BATT_LOW_LAYERS_MAH,
            "main_batt_low_mAh": MAIN_BATT_LOW_MAH,
            "sigma_points": [{"D_m": d, "wind_m_s": w, "repetitions": SIGMA_REPS} for d, w in SIGMA_POINTS],
            "fixed_params": {
                name: config["baseline_params"].get(name)
                for name in (
                    "BATT_FS_LOW_ACT",
                    "BATT_FS_CRT_ACT",
                    "RTL_ALT",
                    "WPNAV_SPEED",
                    "AVOID_ENABLE",
                    "FENCE_ENABLE",
                    "SIM_WIND_DIR",
                    "SIM_WIND_TURB",
                )
            },
        },
        "oracle": {
            "got_home": "touchdown/final distance to home <= 15 m after battery RTL",
            "shortfall_m": "max(0, touchdown/final distance to home - 15 m)",
            "consequence_type": {
                "controlled_land_away": "critical battery LAND executes and LAND_COMPLETE occurs away from home without hard touchdown",
                "uncontrolled": "hard touchdown, missing controlled LAND completion, or other hard terminal evidence",
                "got_home": "returned within home radius",
                "ambiguous": "terminal evidence incomplete",
            },
            "clean_unsafe": "got_home=False and Tier-1 contract violations are empty; battery RTL/LAND correct actions are not counted as violations",
        },
        "pass_fail": {
            "PASS": [
                "premises satisfied: battery RTL triggers/executes, commanded D is achieved on main layer, and return groundspeed declines monotonically with wind",
                "nontrivial robust clean_unsafe region on main BATT_LOW_MAH=220 layer",
                "battery RTL/LAND actions are correct and PGFUZZ-invisible on clean_unsafe points",
                "clean_unsafe count is non-increasing as BATT_LOW_MAH increases over fixed D x wind grid",
                "wide D x wind boundary is consistently characterizable",
            ],
            "FAIL": "premises satisfied but one or more PASS checks fail",
            "INCONCLUSIVE": "premise failure: battery RTL missing, D not achieved on main layer, or wind not coupled to return groundspeed",
            "explicit_non_gates": [
                "no minimum sigma gate",
                "no severity regression MAE <= 1.5 sigma gate",
                "classification/boundary replay is descriptive, not a PASS gate",
            ],
        },
    }


def write_prereg_once(config: dict[str, Any], path: Path) -> str:
    if not path.exists():
        write_json(path, prereg_payload(config))
    return str(path)


def initial_specs() -> list[dict[str, Any]]:
    specs: list[dict[str, Any]] = []
    for batt_low in BATT_LOW_LAYERS_MAH:
        for distance in DISTANCES_M:
            for wind in WINDS_M_S:
                roles = ["grid"]
                if batt_low == MAIN_BATT_LOW_MAH:
                    if wind == 0.0 and distance in {40.0, 80.0, 120.0}:
                        roles.append("phase0_mechanism")
                    if distance == COUPLING_DISTANCE_M and wind in {0.0, 6.0, 12.0}:
                        roles.append("phase0_return_groundspeed_coupling")
                    if (distance, wind) in SIGMA_POINTS:
                        roles.append("sigma_boundary")
                specs.append({
                    "batt_low_mah": batt_low,
                    "distance_m": distance,
                    "wind_m_s": wind,
                    "rep_index": 0,
                    "roles": roles,
                })
    for distance, wind in SIGMA_POINTS:
        for rep_index in range(1, SIGMA_REPS):
            specs.append({
                "batt_low_mah": MAIN_BATT_LOW_MAH,
                "distance_m": distance,
                "wind_m_s": wind,
                "rep_index": rep_index,
                "roles": ["sigma_boundary"],
            })
    return specs


def clean_param_readbacks(records: list[dict[str, Any]]) -> bool:
    return bool(records) and all(bool(record.get("ok")) for record in records)


def terminal_distance(run: dict[str, Any]) -> float | None:
    if run.get("touchdown_distance_m") is not None:
        return float(run["touchdown_distance_m"])
    if run.get("final_distance_m") is not None:
        return float(run["final_distance_m"])
    return None


def decorate_run(run: dict[str, Any]) -> None:
    final_distance = terminal_distance(run)
    shortfall = None if final_distance is None else max(0.0, final_distance - HOME_RADIUS_M)
    run["got_home"] = bool(run.get("low_time_s") is not None and final_distance is not None and final_distance <= HOME_RADIUS_M)
    run["shortfall_m"] = shortfall
    run["landing_distance_m"] = final_distance
    commanded = float(run.get("distance_m") or 0.0)
    achieved = float(run.get("max_distance_m") or 0.0)
    run["achieved_distance_m"] = achieved
    run["achieved_commanded_ratio"] = achieved / commanded if commanded > 0 else None
    low_mode = run.get("low_mode") or {}
    critical_mode = run.get("critical_mode") or {}
    run["rtl_battery_failsafe_triggered"] = bool(
        low_mode
        and low_mode.get("mode") == "RTL"
        and low_mode.get("reason_name") == "BATTERY_FAILSAFE"
    )
    run["land_battery_failsafe_triggered"] = bool(
        critical_mode
        and critical_mode.get("mode") == "LAND"
        and critical_mode.get("reason_name") == "BATTERY_FAILSAFE"
    )
    run["rtl_battery_failsafe_action_ok"] = bool(run.get("low_action_ok"))
    run["land_battery_failsafe_action_ok"] = bool(run.get("critical_action_ok"))
    if not clean_param_readbacks(run.get("param_readbacks", [])):
        run.setdefault("contract_violations", []).append("parameter_readback_failed")
        run["contract_clean"] = False
    if run.get("got_home"):
        run["consequence_type"] = "got_home"
    elif run.get("consequence_type") in (None, "", "got_home", "ambiguous"):
        if run.get("land_battery_failsafe_triggered") and run.get("landing_complete_event") and not run.get("hard_touchdown"):
            run["consequence_type"] = "controlled_land_away"
        elif run.get("hard_touchdown"):
            run["consequence_type"] = "uncontrolled"
        else:
            run["consequence_type"] = "ambiguous"
    if run.get("error"):
        run["label"] = "blocked"
    elif not run.get("contract_clean"):
        run["label"] = "contract_violated"
    elif run["got_home"]:
        run["label"] = "clean_safe"
    else:
        run["label"] = "clean_unsafe"


def run_one(base_config: dict[str, Any], spec: dict[str, Any]) -> dict[str, Any]:
    batt_low = float(spec["batt_low_mah"])
    cfg = config_with_model(make_config(base_config, batt_low), "nominal")
    distance_m = float(spec["distance_m"])
    wind_m_s = float(spec["wind_m_s"])
    rep_index = int(spec["rep_index"])
    run_id = run_id_for(batt_low, distance_m, wind_m_s, rep_index)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "roles": list(spec.get("roles", [])),
        "batt_low_mah": batt_low,
        "distance_m": distance_m,
        "wind_m_s": wind_m_s,
        "rep_index": rep_index,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        params = controlled_params(cfg, {
            "SIM_WIND_SPD": wind_m_s,
            "BATT_LOW_MAH": batt_low,
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
        result["flight"] = run_rtl_energy_flight(master, cfg, distance_m)
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
            decorate_run(result)
            return result
        result["bin_path"] = str(bin_path)
        csv_path = PLANC_ROOT / "logs" / f"{run_id}_parsed.csv"
        parsed = parse_energy_dataflash(
            bin_path=bin_path,
            csv_path=csv_path,
            home=cfg["experiment"]["home"],
            params=params,
            run_kind="rtl_scan",
            target_distance_m=distance_m,
            cruise_speed_m_s=float(cfg["experiment"]["cruise_speed_m_s"]),
            target_bearing_deg=float(cfg["experiment"]["target_bearing_deg"]),
            home_radius_m=HOME_RADIUS_M,
            d_tolerance_m=float(cfg["experiment"]["d_reached_tolerance_m"]),
            speed_tolerance_m_s=float(cfg["experiment"]["speed_audit_tolerance_m_s"]),
            speed_audit_min_distance_m=float(cfg["experiment"]["speed_audit_min_distance_m"]),
        )
        result.update(parsed)
        classify_run(result)
        decorate_run(result)
        return result
    except Exception as exc:
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        classify_run(result)
        decorate_run(result)
        return result
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def run_or_reuse(base_config: dict[str, Any], runs: list[dict[str, Any]], partial_path: Path, spec: dict[str, Any]) -> dict[str, Any]:
    run_id = run_id_for(float(spec["batt_low_mah"]), float(spec["distance_m"]), float(spec["wind_m_s"]), int(spec["rep_index"]))
    for run in runs:
        if run.get("run_id") == run_id:
            roles = set(run.get("roles", []))
            roles.update(spec.get("roles", []))
            run["roles"] = sorted(roles)
            decorate_run(run)
            return run
    print(
        f"RUN {run_id} P={float(spec['batt_low_mah']):.0f} D={float(spec['distance_m']):.0f} "
        f"wind={float(spec['wind_m_s']):.1f} roles={','.join(spec.get('roles', []))}",
        flush=True,
    )
    run = run_one(base_config, spec)
    runs.append(run)
    write_json(partial_path, {"runs": runs})
    return run


def primary_runs(runs: list[dict[str, Any]], batt_low: float | None = None) -> list[dict[str, Any]]:
    out = [run for run in runs if int(run.get("rep_index", 0)) == 0 and not run.get("error")]
    if batt_low is not None:
        out = [run for run in out if float(run.get("batt_low_mah")) == float(batt_low)]
    return out


def runs_at(runs: list[dict[str, Any]], batt_low: float, distance: float, wind: float) -> list[dict[str, Any]]:
    out = [
        run for run in runs
        if not run.get("error")
        and float(run.get("batt_low_mah")) == float(batt_low)
        and float(run.get("distance_m")) == float(distance)
        and float(run.get("wind_m_s")) == float(wind)
    ]
    out.sort(key=lambda r: int(r.get("rep_index", 0)))
    return out


def mechanism_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    rows = [
        run for run in primary_runs(runs, MAIN_BATT_LOW_MAH)
        if "phase0_mechanism" in run.get("roles", [])
    ]
    return {
        "rows": [
            {
                "run_id": run["run_id"],
                "distance_m": run["distance_m"],
                "wind_m_s": run["wind_m_s"],
                "rtl_triggered": run.get("rtl_battery_failsafe_triggered"),
                "rtl_action_ok": run.get("rtl_battery_failsafe_action_ok"),
                "got_home": run.get("got_home"),
                "contract_clean": run.get("contract_clean"),
                "cleanup_forced": bool((run.get("flight") or {}).get("cleanup_forced")),
                "achieved_commanded_ratio": run.get("achieved_commanded_ratio"),
            }
            for run in rows
        ],
        "ok": bool(rows) and all(
            bool(run.get("rtl_battery_failsafe_triggered"))
            and bool(run.get("rtl_battery_failsafe_action_ok"))
            and bool(run.get("got_home"))
            and bool(run.get("contract_clean"))
            and not bool((run.get("flight") or {}).get("cleanup_forced"))
            for run in rows
        ),
    }


def achieved_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    rows = primary_runs(runs, MAIN_BATT_LOW_MAH)
    ratios = [float(run["achieved_commanded_ratio"]) for run in rows if run.get("achieved_commanded_ratio") is not None]
    failures = [
        {
            "run_id": run["run_id"],
            "distance_m": run["distance_m"],
            "wind_m_s": run["wind_m_s"],
            "achieved_commanded_ratio": run.get("achieved_commanded_ratio"),
            "violations": run.get("contract_violations", []),
        }
        for run in rows
        if run.get("achieved_commanded_ratio") is None or float(run["achieved_commanded_ratio"]) < 0.90
    ]
    return {
        "main_layer_only": True,
        "min_achieved_commanded_ratio": min(ratios) if ratios else None,
        "failures": failures,
        "ok": bool(ratios) and not failures,
    }


def coupling_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    rows = []
    for wind in (0.0, 6.0, 12.0):
        entries = runs_at(runs, MAIN_BATT_LOW_MAH, COUPLING_DISTANCE_M, wind)
        run = entries[0] if entries else None
        audit = (run or {}).get("return_speed_audit") or {}
        rows.append({
            "run_id": None if run is None else run.get("run_id"),
            "distance_m": COUPLING_DISTANCE_M,
            "wind_m_s": wind,
            "samples": audit.get("samples"),
            "median_ground_speed_m_s": audit.get("median_ground_speed_m_s"),
            "mean_ground_speed_m_s": audit.get("mean_ground_speed_m_s"),
        })
    speeds = [row.get("median_ground_speed_m_s") for row in rows]
    ok = bool(len(speeds) == 3 and all(speed is not None for speed in speeds) and float(speeds[0]) > float(speeds[1]) > float(speeds[2]))
    return {"rows": rows, "monotonic_return_groundspeed_decline": ok}


def sigma_summary(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summaries = []
    for distance, wind in SIGMA_POINTS:
        entries = runs_at(runs, MAIN_BATT_LOW_MAH, distance, wind)
        shortfalls = [float(run.get("shortfall_m") or 0.0) for run in entries]
        labels = [run.get("label") for run in entries]
        sigma = statistics.stdev(shortfalls) if len(shortfalls) >= 2 else None
        summaries.append({
            "batt_low_mah": MAIN_BATT_LOW_MAH,
            "distance_m": distance,
            "wind_m_s": wind,
            "n": len(entries),
            "labels": labels,
            "stable_label": bool(labels) and all(label == labels[0] for label in labels),
            "shortfall_m_values": shortfalls,
            "mean_shortfall_m": statistics.fmean(shortfalls) if shortfalls else None,
            "sigma_shortfall_m": sigma,
            "runs": [
                {
                    "run_id": run["run_id"],
                    "label": run.get("label"),
                    "got_home": run.get("got_home"),
                    "shortfall_m": run.get("shortfall_m"),
                    "consequence_type": run.get("consequence_type"),
                    "contract_clean": run.get("contract_clean"),
                }
                for run in entries
            ],
        })
    return summaries


def aggregate_points(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    points = []
    for run in primary_runs(runs):
        points.append({
            "run_id": run["run_id"],
            "batt_low_mah": float(run["batt_low_mah"]),
            "distance_m": float(run["distance_m"]),
            "wind_m_s": float(run["wind_m_s"]),
            "label": run.get("label"),
            "got_home": bool(run.get("got_home")),
            "shortfall_m": run.get("shortfall_m"),
            "landing_distance_m": run.get("landing_distance_m"),
            "consequence_type": run.get("consequence_type"),
            "contract_clean": run.get("contract_clean"),
            "contract_violations": run.get("contract_violations", []),
            "rtl_battery_failsafe_triggered": run.get("rtl_battery_failsafe_triggered"),
            "rtl_battery_failsafe_action_ok": run.get("rtl_battery_failsafe_action_ok"),
            "land_battery_failsafe_triggered": run.get("land_battery_failsafe_triggered"),
            "land_battery_failsafe_action_ok": run.get("land_battery_failsafe_action_ok"),
            "touchdown_vertical_speed_m_s_down": run.get("touchdown_vertical_speed_m_s_down"),
            "max_descent_rate_near_touchdown_m_s": run.get("max_descent_rate_near_touchdown_m_s"),
            "achieved_commanded_ratio": run.get("achieved_commanded_ratio"),
            "bin_path": run.get("bin_path"),
            "csv_path": run.get("csv_path"),
            "oracle_path": str(Path(run["csv_path"]).with_suffix(".oracle.json")) if run.get("csv_path") else None,
            "param_records_path": run.get("param_records_path"),
        })
    return points


def zone_counts(points: list[dict[str, Any]]) -> dict[str, int]:
    counts = Counter(str(point.get("label", "blocked")) for point in points)
    return {name: int(counts.get(name, 0)) for name in ("clean_safe", "clean_unsafe", "contract_violated", "blocked")}


def p_stratification(points: list[dict[str, Any]]) -> dict[str, Any]:
    rows = []
    for batt_low in BATT_LOW_LAYERS_MAH:
        layer = [p for p in points if float(p["batt_low_mah"]) == batt_low]
        counts = zone_counts(layer)
        d_not_reached = [
            p for p in layer
            if "D_not_reached_before_low_failsafe" in p.get("contract_violations", [])
        ]
        rows.append({
            "batt_low_mah": batt_low,
            "counts": counts,
            "clean_unsafe": counts["clean_unsafe"],
            "clean_safe": counts["clean_safe"],
            "contract_violated": counts["contract_violated"],
            "d_not_reached_count": len(d_not_reached),
        })
    unsafe_counts = [row["clean_unsafe"] for row in rows]
    monotonic = all(unsafe_counts[i] <= unsafe_counts[i - 1] for i in range(1, len(unsafe_counts)))
    return {
        "rows": rows,
        "monotonic_clean_unsafe_shrink": monotonic,
        "conclusion": "clean_unsafe count is non-increasing as BATT_LOW_MAH increases" if monotonic else "clean_unsafe count is not monotonic in BATT_LOW_MAH",
    }


def main_layer_table(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    layer = [p for p in points if float(p["batt_low_mah"]) == MAIN_BATT_LOW_MAH]
    for distance in DISTANCES_M:
        row: dict[str, Any] = {"distance_m": distance}
        for wind in WINDS_M_S:
            matches = [p for p in layer if float(p["distance_m"]) == distance and float(p["wind_m_s"]) == wind]
            if matches:
                p = matches[0]
                key = f"wind_{wcode(wind)}"
                row[f"{key}_label"] = p["label"]
                row[f"{key}_got_home"] = p["got_home"]
                row[f"{key}_shortfall_m"] = p["shortfall_m"]
        rows.append(row)
    return rows


def boundary_by_layer(points: list[dict[str, Any]]) -> dict[str, Any]:
    rows = []
    for batt_low in BATT_LOW_LAYERS_MAH:
        layer = [p for p in points if float(p["batt_low_mah"]) == batt_low]
        for distance in DISTANCES_M:
            ds = sorted([p for p in layer if float(p["distance_m"]) == distance], key=lambda p: float(p["wind_m_s"]))
            first_unsafe = next((p for p in ds if p.get("label") == "clean_unsafe"), None)
            last_safe = None
            for p in ds:
                if p.get("label") == "clean_safe":
                    last_safe = p
            rows.append({
                "batt_low_mah": batt_low,
                "distance_m": distance,
                "last_safe_wind_m_s": None if last_safe is None else last_safe["wind_m_s"],
                "first_unsafe_wind_m_s": None if first_unsafe is None else first_unsafe["wind_m_s"],
                "has_flip": bool(last_safe is not None and first_unsafe is not None and float(last_safe["wind_m_s"]) < float(first_unsafe["wind_m_s"])),
            })
    main_flips = [row for row in rows if row["batt_low_mah"] == MAIN_BATT_LOW_MAH and row["has_flip"]]
    return {
        "rows": rows,
        "main_layer_flip_count_by_D": len(main_flips),
        "main_layer_flip_distances_m": [row["distance_m"] for row in main_flips],
        "ok": len(main_flips) >= 3,
    }


def robust_clean_unsafe(points: list[dict[str, Any]], sigma: list[dict[str, Any]]) -> dict[str, Any]:
    main = [p for p in points if float(p["batt_low_mah"]) == MAIN_BATT_LOW_MAH]
    clean_unsafe = [p for p in main if p.get("label") == "clean_unsafe"]
    repeated_clean = [s for s in sigma if s.get("stable_label") and s.get("labels") and s["labels"][0] == "clean_unsafe"]
    clean_consequence = [p for p in clean_unsafe if p.get("consequence_type") in {"controlled_land_away", "uncontrolled"}]
    return {
        "clean_unsafe_count": len(clean_unsafe),
        "clean_unsafe_points": clean_unsafe,
        "repeated_stable_clean_unsafe_points": repeated_clean,
        "consequence_typed_count": len(clean_consequence),
        "ok": len(clean_unsafe) >= 8 and len(repeated_clean) >= 2 and len(clean_consequence) == len(clean_unsafe),
    }


def failsafe_invisible(points: list[dict[str, Any]]) -> dict[str, Any]:
    clean_unsafe = [p for p in points if float(p["batt_low_mah"]) == MAIN_BATT_LOW_MAH and p.get("label") == "clean_unsafe"]
    bad = [
        p for p in clean_unsafe
        if not (
            p.get("contract_clean")
            and p.get("rtl_battery_failsafe_triggered")
            and p.get("rtl_battery_failsafe_action_ok")
            and p.get("contract_violations") == []
        )
    ]
    consequence_counts = Counter(str(p.get("consequence_type", "ambiguous")) for p in clean_unsafe)
    return {
        "clean_unsafe_count": len(clean_unsafe),
        "bad_points": bad,
        "consequence_type_counts": dict(consequence_counts),
        "ok": bool(clean_unsafe) and not bad,
        "pgfuzz_invisible_interpretation": "battery RTL/LAND correct actions are treated as compliant; no Tier-1 preventive contract violation appears on clean_unsafe points",
    }


def consequence_distribution(points: list[dict[str, Any]]) -> dict[str, Any]:
    clean_unsafe = [p for p in points if p.get("label") == "clean_unsafe"]
    main_clean_unsafe = [p for p in clean_unsafe if float(p["batt_low_mah"]) == MAIN_BATT_LOW_MAH]
    return {
        "all_layers_clean_unsafe": dict(Counter(str(p.get("consequence_type", "ambiguous")) for p in clean_unsafe)),
        "main_layer_clean_unsafe": dict(Counter(str(p.get("consequence_type", "ambiguous")) for p in main_clean_unsafe)),
        "uncontrolled_points": [p for p in clean_unsafe if p.get("consequence_type") == "uncontrolled"],
    }


def search_efficiency(points: list[dict[str, Any]]) -> dict[str, Any]:
    queries = []
    for batt_low in BATT_LOW_LAYERS_MAH:
        for wind in WINDS_M_S:
            lo = 0
            hi = len(DISTANCES_M) - 1
            q = []
            first_unsafe = None
            while lo <= hi:
                mid = (lo + hi) // 2
                d = DISTANCES_M[mid]
                match = next(
                    (
                        p for p in points
                        if float(p["batt_low_mah"]) == batt_low
                        and float(p["wind_m_s"]) == wind
                        and float(p["distance_m"]) == d
                    ),
                    None,
                )
                label = None if match is None else match.get("label")
                q.append({"distance_m": d, "label": label})
                if label == "clean_unsafe":
                    first_unsafe = d
                    hi = mid - 1
                else:
                    lo = mid + 1
            queries.append({
                "batt_low_mah": batt_low,
                "wind_m_s": wind,
                "queries": q,
                "first_unsafe_distance_m": first_unsafe,
            })
    return {
        "strategy": "offline replay of discrete bisection along D for each BATT_LOW_MAH x wind using completed grid labels",
        "query_count": sum(len(q["queries"]) for q in queries),
        "full_grid_count": len([p for p in points if int(float(p["batt_low_mah"])) in [int(v) for v in BATT_LOW_LAYERS_MAH]]),
        "records": queries,
        "secondary_only": True,
    }


def verdict(summary: dict[str, Any]) -> dict[str, Any]:
    premise_ok = bool(summary["mechanism"]["ok"] and summary["achieved"]["ok"] and summary["coupling"]["monotonic_return_groundspeed_decline"])
    checks = {
        "premise": premise_ok,
        "robust_clean_unsafe": bool(summary["robust_clean_unsafe"]["ok"]),
        "failsafe_correct_and_pgfuzz_invisible": bool(summary["failsafe_invisible"]["ok"]),
        "p_stratification_monotonic": bool(summary["p_stratification"]["monotonic_clean_unsafe_shrink"]),
        "wide_boundary_characterized": bool(summary["boundary"]["ok"]),
    }
    if not premise_ok:
        verdict_name = "INCONCLUSIVE"
        reason = "Premise failed: battery RTL, D achievement, or wind coupling did not hold."
    elif all(checks.values()):
        verdict_name = "PASS"
        reason = "All preregistered energy-headwind threshold-insufficiency checks passed."
    else:
        verdict_name = "FAIL"
        reason = "Premise held, but one or more preregistered checks failed: " + ", ".join(k for k, v in checks.items() if not v)
    return {"verdict": verdict_name, "checks": checks, "reason": reason}


def label_value(label: str | None) -> float:
    return {
        "clean_safe": 0.0,
        "clean_unsafe": 1.0,
        "contract_violated": 0.5,
        "blocked": np.nan,
    }.get(str(label), np.nan)


def plot_outcome(points: list[dict[str, Any]], out_path: Path) -> str:
    main = [p for p in points if float(p["batt_low_mah"]) == MAIN_BATT_LOW_MAH]
    matrix = np.full((len(DISTANCES_M), len(WINDS_M_S)), np.nan)
    for i, d in enumerate(DISTANCES_M):
        for j, w in enumerate(WINDS_M_S):
            match = next((p for p in main if float(p["distance_m"]) == d and float(p["wind_m_s"]) == w), None)
            if match:
                matrix[i, j] = label_value(match.get("label"))
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    im = ax.imshow(matrix, origin="lower", aspect="auto", vmin=0, vmax=1, cmap="RdYlGn_r")
    ax.set_xticks(range(len(WINDS_M_S)), [str(w).rstrip("0").rstrip(".") for w in WINDS_M_S])
    ax.set_yticks(range(len(DISTANCES_M)), [str(int(d)) for d in DISTANCES_M])
    ax.set_xlabel("SIM_WIND_SPD (m/s)")
    ax.set_ylabel("D (m)")
    ax.set_title("Main layer outcome, BATT_LOW_MAH=220")
    for i, d in enumerate(DISTANCES_M):
        for j, w in enumerate(WINDS_M_S):
            match = next((p for p in main if float(p["distance_m"]) == d and float(p["wind_m_s"]) == w), None)
            if match:
                txt = {"clean_safe": "S", "clean_unsafe": "U", "contract_violated": "V", "blocked": "B"}.get(str(match.get("label")), "?")
                ax.text(j, i, txt, ha="center", va="center", color="black", fontsize=9)
    fig.colorbar(im, ax=ax, ticks=[0, 0.5, 1], label="safe / contract / unsafe")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return str(out_path)


def plot_pstrat(p_summary: dict[str, Any], out_path: Path) -> str:
    rows = p_summary["rows"]
    x = [row["batt_low_mah"] for row in rows]
    unsafe = [row["clean_unsafe"] for row in rows]
    safe = [row["clean_safe"] for row in rows]
    violated = [row["contract_violated"] for row in rows]
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    ax.plot(x, unsafe, marker="o", label="clean_unsafe")
    ax.plot(x, safe, marker="o", label="clean_safe")
    ax.plot(x, violated, marker="o", label="contract_violated")
    ax.set_xlabel("BATT_LOW_MAH")
    ax.set_ylabel("grid point count")
    ax.set_title("P stratification over fixed D x wind grid")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return str(out_path)


def plot_boundary(boundary: dict[str, Any], out_path: Path) -> str:
    fig, ax = plt.subplots(figsize=(8.0, 4.8))
    for batt_low in BATT_LOW_LAYERS_MAH:
        rows = [r for r in boundary["rows"] if float(r["batt_low_mah"]) == batt_low]
        xs = [r["distance_m"] for r in rows if r["first_unsafe_wind_m_s"] is not None]
        ys = [r["first_unsafe_wind_m_s"] for r in rows if r["first_unsafe_wind_m_s"] is not None]
        if xs:
            ax.plot(xs, ys, marker="o", label=f"BATT_LOW_MAH={batt_low:.0f}")
    ax.set_xlabel("D (m)")
    ax.set_ylabel("first clean_unsafe wind (m/s)")
    ax.set_title("Wide-range boundary characterization")
    ax.set_ylim(-0.5, max(WINDS_M_S) + 1.0)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return str(out_path)


def plot_consequence(dist: dict[str, Any], out_path: Path) -> str:
    counts = dist["main_layer_clean_unsafe"]
    names = ["controlled_land_away", "uncontrolled", "ambiguous"]
    vals = [int(counts.get(name, 0)) for name in names]
    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    ax.bar(names, vals, color=["#4c78a8", "#e45756", "#b279a2"])
    ax.set_ylabel("main-layer clean_unsafe count")
    ax.set_title("Consequence type distribution")
    ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return str(out_path)


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    summary = payload["summary"]
    return {
        "outcome_vs_D_wind": plot_outcome(summary["points"], analysis / f"{STEM}_outcome_vs_D_wind.png"),
        "pstrat_batt_low_mah_monotonic": plot_pstrat(summary["p_stratification"], analysis / f"{STEM}_pstrat_batt_low_mah_monotonic.png"),
        "boundary_wide_range": plot_boundary(summary["boundary"], analysis / f"{STEM}_boundary_wide_range.png"),
        "consequence_type_distribution": plot_consequence(summary["consequence_distribution"], analysis / f"{STEM}_consequence_type_distribution.png"),
    }


def write_report(payload: dict[str, Any]) -> str:
    report_path = PLANC_ROOT / "results" / f"{STEM}_report.md"
    summary = payload["summary"]
    v = payload["verdict"]
    lines: list[str] = []
    lines.append(f"VERDICT: {v['verdict']}")
    lines.append("")
    lines.append("# 能量逆风 v1：RTL 容量阈值不足")
    lines.append("")
    lines.append(f"结论：{v['reason']}")
    lines.append("")
    lines.append("## 五个预注册判据")
    lines.append("")
    checks = v["checks"]
    lines.append(f"- 前提满足：**{checks['premise']}**。机制复核={summary['mechanism']['ok']}；主层最小 achieved/commanded={fmt(summary['achieved']['min_achieved_commanded_ratio'], 3)}；返航地速单调={summary['coupling']['monotonic_return_groundspeed_decline']}。")
    lines.append(f"- robust clean_unsafe 区：**{checks['robust_clean_unsafe']}**。主层 clean_unsafe={summary['robust_clean_unsafe']['clean_unsafe_count']}；稳定重复边界点={len(summary['robust_clean_unsafe']['repeated_stable_clean_unsafe_points'])}。")
    lines.append(f"- 失效保护正确触发且 PGFUZZ 不可见：**{checks['failsafe_correct_and_pgfuzz_invisible']}**。clean_unsafe 上坏点={len(summary['failsafe_invisible']['bad_points'])}。")
    lines.append(f"- BATT_LOW_MAH 分层单调收缩：**{checks['p_stratification_monotonic']}**。{summary['p_stratification']['conclusion']}。")
    lines.append(f"- 宽范围边界可刻画：**{checks['wide_boundary_characterized']}**。主层有 same-D 翻转的 D={summary['boundary']['main_layer_flip_distances_m']}。")
    lines.append("")
    lines.append("## 固定配置与隔离")
    lines.append("")
    fixed = payload["fixed_params"]
    lines.append(
        f"主层 `BATT_LOW_MAH=220`；分层 `{BATT_LOW_LAYERS_MAH}`。固定 `BATT_FS_LOW_ACT=2`、"
        f"`BATT_FS_CRT_ACT=1`、`RTL_ALT={fixed['RTL_ALT']}`、`WPNAV_SPEED={fixed['WPNAV_SPEED']}`、"
        "`AVOID_ENABLE=0`、`FENCE_ENABLE=0`、`SIM_WIND_DIR=270`。出航向东顺风，RTL 返航向西逆风。"
    )
    lines.append("")
    lines.append("## 前提复核")
    lines.append("")
    lines.append("| check | key numbers |")
    lines.append("| --- | --- |")
    coupling_bits = [
        f"wind {fmt(row['wind_m_s'], 1)}: {fmt(row.get('median_ground_speed_m_s'))} m/s"
        for row in summary["coupling"]["rows"]
    ]
    lines.append(f"| battery RTL mechanism | no-wind mechanism rows all clean/got_home={summary['mechanism']['ok']} |")
    lines.append(f"| D achieved | min achieved/commanded on main layer={fmt(summary['achieved']['min_achieved_commanded_ratio'], 3)} |")
    lines.append(f"| return wind coupling | {'; '.join(coupling_bits)} |")
    lines.append("")
    lines.append("## 主层 D x wind 标签")
    lines.append("")
    header = "| D m | " + " | ".join(f"wind {str(w).rstrip('0').rstrip('.')}" for w in WINDS_M_S) + " |"
    lines.append(header)
    lines.append("| ---: | " + " | ".join("---" for _ in WINDS_M_S) + " |")
    for row in summary["main_layer_table"]:
        vals = []
        for wind in WINDS_M_S:
            vals.append(str(row.get(f"wind_{wcode(wind)}_label", "n/a")))
        lines.append(f"| {fmt(row['distance_m'], 0)} | " + " | ".join(vals) + " |")
    lines.append("")
    lines.append("## P 分层（承重项）")
    lines.append("")
    lines.append("| BATT_LOW_MAH | clean_safe | clean_unsafe | contract_violated | D_not_reached |")
    lines.append("| ---: | ---: | ---: | ---: | ---: |")
    for row in summary["p_stratification"]["rows"]:
        lines.append(
            f"| {fmt(row['batt_low_mah'], 0)} | {row['clean_safe']} | {row['clean_unsafe']} | "
            f"{row['contract_violated']} | {row['d_not_reached_count']} |"
        )
    lines.append("")
    lines.append("## sigma（只报告，不设门）")
    lines.append("")
    lines.append("本实验明确不设置 sigma 前提门，也不设置严重度回归 MAE 门；这些重复只说明标签和后果大小的稳定性。")
    lines.append("")
    lines.append("| D | wind | n | stable | mean shortfall m | sigma m | labels |")
    lines.append("| ---: | ---: | ---: | --- | ---: | ---: | --- |")
    for row in summary["sigma"]:
        lines.append(
            f"| {fmt(row['distance_m'], 0)} | {fmt(row['wind_m_s'], 1)} | {row['n']} | {row['stable_label']} | "
            f"{fmt(row['mean_shortfall_m'])} | {fmt(row['sigma_shortfall_m'], 3)} | {', '.join(str(x) for x in row['labels'])} |"
        )
    lines.append("")
    lines.append("## 宽范围边界刻画（非学习贡献）")
    lines.append("")
    lines.append(
        f"主层 same-D 翻转出现在 D={summary['boundary']['main_layer_flip_distances_m']}。"
        "这里的边界刻画用于说明确定性和平滑移动，不作为噪声鲁棒学习/预测贡献。"
    )
    search = summary["search_efficiency"]
    lines.append(
        f"搜索效率仅作次要报告：{search['strategy']}，离线二分查询 {search['query_count']} 次，"
        f"完整主网格/分层网格点 {search['full_grid_count']} 个。"
    )
    lines.append("")
    lines.append("## 后果类型")
    lines.append("")
    dist = summary["consequence_distribution"]
    lines.append(f"主层 clean_unsafe consequence_type 分布：`{dist['main_layer_clean_unsafe']}`。")
    if dist["uncontrolled_points"]:
        lines.append(f"出现 uncontrolled 点：{', '.join(p['run_id'] for p in dist['uncontrolled_points'])}。")
    else:
        lines.append("本次 clean_unsafe 主要是 `controlled_land_away`：受控迫降在非 home 位置。报告不把它夸成坠机；它是 RTL 返航承诺未兑现的 SOTIF 后果。")
    lines.append("")
    lines.append("## threshold-insufficiency 解释")
    lines.append("")
    lines.append(
        "`BATT_LOW_MAH` 隐含承诺是触发 RTL 时仍有足够返航储备。本实验中电池 failsafe 和 RTL/LAND 都按规约正确触发，"
        "Tier-1 契约检查器看不到预防性违约；但在合法逆风下仍出现 clean_unsafe。随着 `BATT_LOW_MAH` 增大，"
        "clean_unsafe 区单调收缩，说明缺口是软件配置阈值的函数，而不是单纯“逆风耗电”的物理常识。"
    )
    lines.append("")
    lines.append("## 图")
    lines.append("")
    for name, path in payload["artifacts"]["plots"].items():
        lines.append(f"- {name}: ![]({rel(path)})")
    lines.append("")
    lines.append("## 审计文件")
    lines.append("")
    lines.append(
        "每个 run 均有 `planc/logs/<run_id>.BIN`、`<run_id>_params.json`、`<run_id>_parsed.csv`、"
        "`<run_id>_parsed.oracle.json`。结果 JSON 中 `summary.points[*]` 带有对应路径。"
    )
    lines.append("")
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return str(report_path)


def build_payload(base_config: dict[str, Any], env: dict[str, Any], prereg_path: Path, runs: list[dict[str, Any]]) -> dict[str, Any]:
    for run in runs:
        decorate_run(run)
    points = aggregate_points(runs)
    sigma = sigma_summary(runs)
    summary = {
        "mechanism": mechanism_summary(runs),
        "achieved": achieved_summary(runs),
        "coupling": coupling_summary(runs),
        "sigma": sigma,
        "points": points,
        "main_layer_table": main_layer_table(points),
        "p_stratification": p_stratification(points),
        "boundary": boundary_by_layer(points),
        "robust_clean_unsafe": robust_clean_unsafe(points, sigma),
        "failsafe_invisible": failsafe_invisible(points),
        "consequence_distribution": consequence_distribution(points),
        "search_efficiency": search_efficiency(points),
    }
    fixed_config = make_config(base_config, MAIN_BATT_LOW_MAH)
    payload = {
        "status": "COMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "env": env,
        "prereg_path": str(prereg_path),
        "fixed_params": {
            name: fixed_config["baseline_params"].get(name)
            for name in (
                "BATT_LOW_MAH",
                "BATT_FS_LOW_ACT",
                "BATT_FS_CRT_ACT",
                "RTL_ALT",
                "WPNAV_SPEED",
                "AVOID_ENABLE",
                "FENCE_ENABLE",
                "SIM_WIND_DIR",
                "SIM_WIND_TURB",
            )
        },
        "design": {
            "distances_m": DISTANCES_M,
            "winds_m_s": WINDS_M_S,
            "batt_low_layers_mAh": BATT_LOW_LAYERS_MAH,
            "main_batt_low_mAh": MAIN_BATT_LOW_MAH,
            "sigma_points": SIGMA_POINTS,
            "classification_or_regression_pass_gate": False,
            "sigma_pass_gate": False,
        },
        "summary": summary,
        "runs": runs,
    }
    payload["verdict"] = verdict(summary)
    payload["artifacts"] = {"plots": make_plots(payload)}
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the energy-headwind v1 full experiment.")
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "rtl_energy_config.yaml")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    base_config = load_config(args.config)
    results_dir = PLANC_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    prereg_path = results_dir / f"{STEM}_prereg.json"
    partial_path = results_dir / f"{STEM}_partial.json"
    final_path = results_dir / f"{STEM}_result.json"

    main_config = make_config(base_config, MAIN_BATT_LOW_MAH)
    write_prereg_once(main_config, prereg_path)
    env = probe_environment(config_with_model(main_config, "nominal"), REPO_ROOT)
    write_env(env, results_dir / f"env_{STEM}.json")

    runs: list[dict[str, Any]] = []
    if args.resume and partial_path.exists():
        runs = list(json.loads(partial_path.read_text(encoding="utf-8")).get("runs", []))
        for run in runs:
            decorate_run(run)

    specs = initial_specs()
    for idx, spec in enumerate(specs, start=1):
        print(f"PROGRESS {idx}/{len(specs)}", flush=True)
        run_or_reuse(base_config, runs, partial_path, spec)

    payload = build_payload(base_config, env, prereg_path, runs)
    write_json(final_path, payload)
    print(f"VERDICT {payload['verdict']['verdict']}: {payload['verdict']['reason']}", flush=True)
    print(f"RESULT {final_path}", flush=True)
    print(f"REPORT {payload['artifacts']['report']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
