from __future__ import annotations

import argparse
import copy
import json
import statistics
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

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


STEM = "energy_headwind_feasibility"
HOME_RADIUS_M = 15.0
LEGAL_WINDS_M_S = [0.0, 6.0, 9.0, 12.0]
COUPLING_DISTANCE_M = 100.0
SIGMA_REPS = 5


def run_id_for(distance_m: float, wind_m_s: float, rep_index: int) -> str:
    return f"ehf_D{int(distance_m):03d}_W{int(wind_m_s):02d}_r{rep_index:02d}"


def make_config(base: dict[str, Any]) -> dict[str, Any]:
    cfg = copy.deepcopy(base)
    cfg["experiment"]["name"] = STEM
    cfg["experiment"]["home_radius_m"] = HOME_RADIUS_M
    params = cfg.setdefault("baseline_params", {})
    params.update({
        "BATT_FS_LOW_ACT": 2,
        "BATT_FS_CRT_ACT": 1,
        "BATT_LOW_MAH": float(params.get("BATT_LOW_MAH", 220)),
        "BATT_LOW_VOLT": 0,
        "BATT_CRT_VOLT": 0,
        "FENCE_ENABLE": 0,
        "FENCE_TYPE": 0,
        "AVOID_ENABLE": 0,
        "SIM_WIND_DIR": 270,
        "SIM_WIND_TURB": 0,
    })
    return cfg


def initial_specs() -> list[dict[str, Any]]:
    specs: list[dict[str, Any]] = [
        {"distance_m": 40.0, "wind_m_s": 0.0, "rep_index": 0, "roles": ["A_mechanism_near_no_wind", "B_cleanup_audit"]},
        {"distance_m": 80.0, "wind_m_s": 0.0, "rep_index": 0, "roles": ["A_mechanism_mid_no_wind", "C_flip_D80"]},
        {"distance_m": 120.0, "wind_m_s": 0.0, "rep_index": 0, "roles": ["A_mechanism_far_no_wind", "C_flip_D120"]},
    ]
    for wind in (0.0, 6.0, 12.0):
        specs.append({
            "distance_m": COUPLING_DISTANCE_M,
            "wind_m_s": wind,
            "rep_index": 0,
            "roles": ["B_return_groundspeed_coupling", "C_flip_D100"],
        })
    for wind in (6.0, 9.0, 12.0):
        specs.append({"distance_m": 80.0, "wind_m_s": wind, "rep_index": 0, "roles": ["C_flip_D80"]})
    specs.append({"distance_m": 100.0, "wind_m_s": 9.0, "rep_index": 0, "roles": ["C_flip_D100", "D_sigma_candidate"]})
    for wind in (6.0, 9.0, 12.0):
        specs.append({"distance_m": 120.0, "wind_m_s": wind, "rep_index": 0, "roles": ["C_flip_D120"]})
    return specs


def clean_param_readbacks(records: list[dict[str, Any]]) -> bool:
    return bool(records) and all(bool(record.get("ok")) for record in records)


def decorate_run(run: dict[str, Any], *, home_radius_m: float) -> None:
    final_distance = run.get("final_distance_m")
    shortfall = None
    if final_distance is not None:
        shortfall = max(0.0, float(final_distance) - float(home_radius_m))
    run["got_home"] = bool(run.get("low_time_s") is not None and final_distance is not None and float(final_distance) <= home_radius_m)
    run["shortfall_m"] = shortfall
    commanded = float(run.get("distance_m") or 0.0)
    achieved = float(run.get("max_distance_m") or 0.0)
    run["achieved_distance_m"] = achieved
    run["achieved_commanded_ratio"] = achieved / commanded if commanded > 0 else None
    low_mode = run.get("low_mode") or {}
    run["rtl_battery_failsafe_triggered"] = bool(
        low_mode
        and low_mode.get("mode") == "RTL"
        and low_mode.get("reason_name") == "BATTERY_FAILSAFE"
    )
    run["rtl_battery_failsafe_action_ok"] = bool(run.get("low_action_ok"))
    flight = run.get("flight") or {}
    observe = flight.get("post_failsafe_observation") or {}
    run["cleanup_audit"] = {
        "cleanup_forced": bool(flight.get("cleanup_forced")),
        "post_failsafe_disarmed": bool(observe.get("disarmed")),
        "post_failsafe_final_distance_m": observe.get("final_realtime_distance_m"),
        "post_failsafe_final_alt_m": observe.get("final_realtime_alt_m"),
    }


def run_one(config: dict[str, Any], spec: dict[str, Any]) -> dict[str, Any]:
    cfg = config_with_model(config, "nominal")
    distance_m = float(spec["distance_m"])
    wind_m_s = float(spec["wind_m_s"])
    rep_index = int(spec["rep_index"])
    run_id = run_id_for(distance_m, wind_m_s, rep_index)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "roles": list(spec.get("roles", [])),
        "distance_m": distance_m,
        "wind_m_s": wind_m_s,
        "rep_index": rep_index,
        "batt_low_mah": float(config["baseline_params"]["BATT_LOW_MAH"]),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        params = controlled_params(config, {
            "SIM_WIND_SPD": wind_m_s,
            "BATT_LOW_MAH": float(config["baseline_params"]["BATT_LOW_MAH"]),
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
            cruise_speed_m_s=float(config["experiment"]["cruise_speed_m_s"]),
            target_bearing_deg=float(config["experiment"]["target_bearing_deg"]),
            home_radius_m=HOME_RADIUS_M,
            d_tolerance_m=float(config["experiment"]["d_reached_tolerance_m"]),
            speed_tolerance_m_s=float(config["experiment"]["speed_audit_tolerance_m_s"]),
            speed_audit_min_distance_m=float(config["experiment"]["speed_audit_min_distance_m"]),
        )
        result.update(parsed)
        classify_run(result)
        if not clean_param_readbacks(result.get("param_readbacks", [])):
            result.setdefault("contract_violations", []).append("parameter_readback_failed")
            result["contract_clean"] = False
        decorate_run(result, home_radius_m=HOME_RADIUS_M)
        return result
    except Exception as exc:
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        classify_run(result)
        decorate_run(result, home_radius_m=HOME_RADIUS_M)
        return result
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def run_or_reuse(config: dict[str, Any], runs: list[dict[str, Any]], partial_path: Path, spec: dict[str, Any]) -> dict[str, Any]:
    run_id = run_id_for(float(spec["distance_m"]), float(spec["wind_m_s"]), int(spec["rep_index"]))
    for run in runs:
        if run.get("run_id") == run_id:
            roles = set(run.get("roles", []))
            roles.update(spec.get("roles", []))
            run["roles"] = sorted(roles)
            decorate_run(run, home_radius_m=HOME_RADIUS_M)
            return run
    print(
        f"RUN {run_id} D={float(spec['distance_m']):.0f} wind={float(spec['wind_m_s']):.0f} "
        f"roles={','.join(spec.get('roles', []))}",
        flush=True,
    )
    run = run_one(config, spec)
    runs.append(run)
    write_json(partial_path, {"runs": runs})
    return run


def primary_runs(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [run for run in runs if int(run.get("rep_index", 0)) == 0 and not run.get("error")]


def grouped_primary(runs: list[dict[str, Any]]) -> dict[float, list[dict[str, Any]]]:
    grouped: dict[float, list[dict[str, Any]]] = {}
    for run in primary_runs(runs):
        grouped.setdefault(float(run["distance_m"]), []).append(run)
    for entries in grouped.values():
        entries.sort(key=lambda r: float(r["wind_m_s"]))
    return dict(sorted(grouped.items()))


def find_flips(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    flips: list[dict[str, Any]] = []
    for distance_m, entries in grouped_primary(runs).items():
        ordered = [run for run in entries if float(run["wind_m_s"]) in LEGAL_WINDS_M_S]
        for idx in range(1, len(ordered)):
            lower = ordered[idx - 1]
            upper = ordered[idx]
            if bool(lower.get("got_home")) and not bool(upper.get("got_home")):
                flips.append({
                    "distance_m": distance_m,
                    "lower_wind_m_s": float(lower["wind_m_s"]),
                    "upper_wind_m_s": float(upper["wind_m_s"]),
                    "lower_run_id": lower["run_id"],
                    "upper_run_id": upper["run_id"],
                    "upper_shortfall_m": upper.get("shortfall_m"),
                })
    return flips


def choose_sigma_point(runs: list[dict[str, Any]]) -> tuple[float, float] | None:
    flips = find_flips(runs)
    if not flips:
        return None
    best = min(flips, key=lambda item: abs(float(item.get("upper_shortfall_m") or 9999.0)))
    return float(best["distance_m"]), float(best["upper_wind_m_s"])


def coupling_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    rows = []
    for wind in (0.0, 6.0, 12.0):
        matches = [
            run for run in primary_runs(runs)
            if float(run.get("distance_m")) == COUPLING_DISTANCE_M and float(run.get("wind_m_s")) == wind
        ]
        run = matches[0] if matches else None
        audit = (run or {}).get("return_speed_audit") or {}
        rows.append({
            "distance_m": COUPLING_DISTANCE_M,
            "wind_m_s": wind,
            "run_id": None if run is None else run.get("run_id"),
            "samples": audit.get("samples"),
            "median_ground_speed_m_s": audit.get("median_ground_speed_m_s"),
            "mean_ground_speed_m_s": audit.get("mean_ground_speed_m_s"),
            "median_inbound_component_m_s": audit.get("median_inbound_component_m_s"),
        })
    speeds = [row.get("median_ground_speed_m_s") for row in rows]
    monotonic = bool(
        len(speeds) == 3
        and all(speed is not None for speed in speeds)
        and float(speeds[0]) > float(speeds[1]) > float(speeds[2])
    )
    return {
        "distance_m": COUPLING_DISTANCE_M,
        "rows": rows,
        "monotonic_return_groundspeed_decline": monotonic,
    }


def sigma_summary(runs: list[dict[str, Any]], point: tuple[float, float] | None) -> dict[str, Any]:
    if point is None:
        return {"point": None, "runs": [], "n": 0, "sigma_shortfall_m": None, "nontrivial": None}
    distance_m, wind_m_s = point
    entries = [
        run for run in runs
        if not run.get("error")
        and float(run.get("distance_m")) == distance_m
        and float(run.get("wind_m_s")) == wind_m_s
    ]
    entries.sort(key=lambda r: int(r.get("rep_index", 0)))
    shortfalls = [float(run.get("shortfall_m") or 0.0) for run in entries]
    sigma = statistics.stdev(shortfalls) if len(shortfalls) >= 2 else 0.0 if shortfalls else None
    return {
        "point": {"distance_m": distance_m, "wind_m_s": wind_m_s},
        "runs": [
            {
                "run_id": run["run_id"],
                "got_home": bool(run.get("got_home")),
                "final_distance_m": run.get("final_distance_m"),
                "shortfall_m": run.get("shortfall_m"),
                "contract_clean": run.get("contract_clean"),
            }
            for run in entries
        ],
        "n": len(entries),
        "shortfall_m_values": shortfalls,
        "mean_shortfall_m": statistics.fmean(shortfalls) if shortfalls else None,
        "sigma_shortfall_m": sigma,
        "nontrivial": None if sigma is None else bool(float(sigma) > 0.05),
    }


def mechanism_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    mechanism_runs = [
        run for run in primary_runs(runs)
        if any(str(role).startswith("A_mechanism") for role in run.get("roles", []))
    ]
    rows = []
    for run in mechanism_runs:
        rows.append({
            "run_id": run["run_id"],
            "distance_m": run["distance_m"],
            "wind_m_s": run["wind_m_s"],
            "rtl_triggered": run.get("rtl_battery_failsafe_triggered"),
            "rtl_action_ok": run.get("rtl_battery_failsafe_action_ok"),
            "got_home": run.get("got_home"),
            "final_distance_m": run.get("final_distance_m"),
            "achieved_distance_m": run.get("achieved_distance_m"),
            "achieved_commanded_ratio": run.get("achieved_commanded_ratio"),
            "cleanup_forced": run.get("cleanup_audit", {}).get("cleanup_forced"),
            "contract_clean": run.get("contract_clean"),
            "contract_violations": run.get("contract_violations", []),
        })
    return {
        "rows": rows,
        "rtl_triggered_and_executed": bool(rows) and all(bool(row["rtl_triggered"]) and bool(row["rtl_action_ok"]) for row in rows),
        "all_got_home": bool(rows) and all(bool(row["got_home"]) for row in rows),
        "cleanup_not_misclassified": bool(rows) and all(not bool(row["cleanup_forced"]) and bool(row["contract_clean"]) for row in rows),
    }


def achieved_summary(runs: list[dict[str, Any]]) -> dict[str, Any]:
    rows = []
    for run in primary_runs(runs):
        rows.append({
            "run_id": run["run_id"],
            "distance_m": run["distance_m"],
            "wind_m_s": run["wind_m_s"],
            "achieved_distance_m": run.get("achieved_distance_m"),
            "achieved_commanded_ratio": run.get("achieved_commanded_ratio"),
        })
    ratios = [float(row["achieved_commanded_ratio"]) for row in rows if row.get("achieved_commanded_ratio") is not None]
    return {
        "rows": rows,
        "min_achieved_commanded_ratio": min(ratios) if ratios else None,
        "ok": bool(ratios) and min(ratios) >= 0.90,
    }


def flip_table(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    table = []
    for distance_m, entries in grouped_primary(runs).items():
        row: dict[str, Any] = {"distance_m": distance_m}
        for wind in LEGAL_WINDS_M_S:
            matches = [run for run in entries if float(run["wind_m_s"]) == wind]
            if matches:
                run = matches[0]
                row[f"wind_{int(wind):02d}_got_home"] = bool(run.get("got_home"))
                row[f"wind_{int(wind):02d}_shortfall_m"] = run.get("shortfall_m")
                row[f"wind_{int(wind):02d}_run_id"] = run.get("run_id")
        table.append(row)
    return table


def verdict(summary: dict[str, Any]) -> dict[str, Any]:
    mechanism = summary["mechanism"]
    achieved = summary["achieved"]
    coupling = summary["coupling"]
    flips = summary["flips"]
    primary = summary["primary_runs"]
    no_wind = [run for run in primary if float(run.get("wind_m_s")) == 0.0]
    max_wind = [run for run in primary if float(run.get("wind_m_s")) == 12.0]
    if not mechanism["rtl_triggered_and_executed"] or not mechanism["all_got_home"]:
        return {"verdict": "NO-GO", "branch": "degenerate_mechanism", "reason": "Battery RTL did not cleanly trigger/execute and return home in no-wind mechanism runs."}
    if not achieved["ok"]:
        return {"verdict": "NO-GO", "branch": "D_not_reached", "reason": "At least one run did not reach commanded D with achieved/commanded >= 0.9."}
    if not coupling["monotonic_return_groundspeed_decline"]:
        return {"verdict": "NO-GO", "branch": "wind_not_coupled", "reason": "Return ground speed did not monotonically decline with SIM_WIND_SPD."}
    if flips:
        return {"verdict": "GO", "branch": "clean_flip_in_legal_envelope", "reason": "At least one same-D got_home True->False flip occurs inside 0-12 m/s wind."}
    if no_wind and any(not bool(run.get("got_home")) for run in no_wind):
        return {"verdict": "NO-GO", "branch": "degenerate_no_wind_failure", "reason": "A no-wind run failed to get home, so the boundary is confounded."}
    if max_wind and all(bool(run.get("got_home")) for run in max_wind):
        return {"verdict": "NO-GO", "branch": "envelope_absent", "reason": "All tested D values still got home at the maximum legal wind 12 m/s."}
    return {"verdict": "ADJUST", "branch": "boundary_outside_first_guess", "reason": "Safe and unsafe points exist, but no same-D wind flip was bracketed by the first-guess D/wind set."}


def plot_got_home(table: list[dict[str, Any]], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    for row in table:
        xs = []
        ys = []
        for wind in LEGAL_WINDS_M_S:
            key = f"wind_{int(wind):02d}_got_home"
            if key in row:
                xs.append(wind)
                ys.append(1.0 if row[key] else 0.0)
        if xs:
            ax.plot(xs, ys, marker="o", linewidth=1.8, label=f"D={row['distance_m']:.0f} m")
    ax.set_xlabel("SIM_WIND_SPD (m/s)")
    ax.set_ylabel("got_home")
    ax.set_yticks([0, 1], labels=["False", "True"])
    ax.set_ylim(-0.15, 1.15)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return str(out_path)


def write_report(payload: dict[str, Any]) -> str:
    report_path = PLANC_ROOT / "results" / f"{STEM}_report.md"
    summary = payload["summary"]
    verdict_payload = payload["verdict"]
    lines: list[str] = []
    lines.append(f"VERDICT: {verdict_payload['verdict']}")
    lines.append("")
    lines.append("# Energy Headwind Feasibility Probe")
    lines.append("")
    lines.append(f"Decision branch: `{verdict_payload['branch']}`. {verdict_payload['reason']}")
    lines.append("")
    lines.append("## Four checks")
    lines.append("")
    mech = summary["mechanism"]
    achieved = summary["achieved"]
    coupling = summary["coupling"]
    sigma = summary["sigma"]
    lines.append(
        f"- Battery RTL triggered/executed: **{mech['rtl_triggered_and_executed']}**; "
        f"no-wind got_home all true={mech['all_got_home']}; cleanup audit clean={mech['cleanup_not_misclassified']}."
    )
    lines.append(
        f"- D reached: **{achieved['ok']}**; min achieved/commanded={fmt(achieved['min_achieved_commanded_ratio'], 3)}."
    )
    speed_bits = [
        f"{fmt(row['wind_m_s'], 0)} m/s -> {fmt(row.get('median_ground_speed_m_s'))} m/s"
        for row in coupling["rows"]
    ]
    lines.append(
        f"- Return headwind coupling: **{coupling['monotonic_return_groundspeed_decline']}**; "
        f"D={fmt(coupling['distance_m'], 0)} median return groundspeeds: {', '.join(speed_bits)}."
    )
    flip_bits = [
        f"D={fmt(flip['distance_m'], 0)}: {fmt(flip['lower_wind_m_s'], 0)} True -> {fmt(flip['upper_wind_m_s'], 0)} False"
        for flip in summary["flips"]
    ]
    lines.append(f"- Legal-envelope got_home flip: **{bool(summary['flips'])}**; {', '.join(flip_bits) or 'none'}.")
    if sigma["point"]:
        point = sigma["point"]
        lines.append(
            f"- Boundary sigma: D={fmt(point['distance_m'], 0)}, wind={fmt(point['wind_m_s'], 0)}, "
            f"n={sigma['n']}, mean shortfall={fmt(sigma['mean_shortfall_m'])} m, "
            f"sigma={fmt(sigma['sigma_shortfall_m'], 3)} m, nontrivial={sigma['nontrivial']}."
        )
    lines.append("")
    lines.append("## Fixed Scenario")
    lines.append("")
    fixed = payload["fixed_params"]
    lines.append(
        f"`BATT_LOW_MAH={fixed['BATT_LOW_MAH']}`, `BATT_FS_LOW_ACT=2`, `BATT_FS_CRT_ACT=1`, "
        f"`RTL_ALT={fixed['RTL_ALT']}`, `WPNAV_SPEED={fixed['WPNAV_SPEED']}`, "
        "`AVOID_ENABLE=0`, `FENCE_ENABLE=0`, `SIM_WIND_DIR=270`."
    )
    lines.append("Outbound is east/downwind; RTL return is west/headwind.")
    lines.append("")
    lines.append("## got_home vs wind")
    lines.append("")
    lines.append("| D m | wind 0 | wind 6 | wind 9 | wind 12 |")
    lines.append("| ---: | --- | --- | --- | --- |")
    for row in summary["flip_table"]:
        values = []
        for wind in LEGAL_WINDS_M_S:
            key = f"wind_{int(wind):02d}_got_home"
            values.append(str(row.get(key, "n/a")))
        lines.append(f"| {fmt(row['distance_m'], 0)} | {' | '.join(values)} |")
    lines.append("")
    lines.append(f"Figure: ![]({rel(payload['artifacts']['got_home_vs_wind_plot'])})")
    lines.append("")
    lines.append("## Runs")
    lines.append("")
    lines.append("| run | D | wind | got_home | shortfall m | final dist m | RTL | clean | return median GS |")
    lines.append("| --- | ---: | ---: | --- | ---: | ---: | --- | --- | ---: |")
    for run in sorted(payload["runs"], key=lambda r: (float(r["distance_m"]), float(r["wind_m_s"]), int(r["rep_index"]))):
        audit = run.get("return_speed_audit") or {}
        lines.append(
            f"| {run['run_id']} | {fmt(run['distance_m'], 0)} | {fmt(run['wind_m_s'], 0)} | "
            f"{run.get('got_home')} | {fmt(run.get('shortfall_m'))} | {fmt(run.get('final_distance_m'))} | "
            f"{run.get('rtl_battery_failsafe_triggered')} | {run.get('contract_clean')} | "
            f"{fmt(audit.get('median_ground_speed_m_s'))} |"
        )
    lines.append("")
    lines.append("No decisive tag was created.")
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(report_path)


def build_payload(config: dict[str, Any], env: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    for run in runs:
        decorate_run(run, home_radius_m=HOME_RADIUS_M)
    sigma_point = choose_sigma_point(runs)
    summary = {
        "mechanism": mechanism_summary(runs),
        "achieved": achieved_summary(runs),
        "coupling": coupling_summary(runs),
        "flips": find_flips(runs),
        "flip_table": flip_table(runs),
        "sigma": sigma_summary(runs, sigma_point),
        "primary_runs": primary_runs(runs),
    }
    payload = {
        "status": "COMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "env": env,
        "fixed_params": {
            name: config["baseline_params"].get(name)
            for name in (
                "BATT_LOW_MAH",
                "BATT_FS_LOW_ACT",
                "BATT_FS_CRT_ACT",
                "RTL_ALT",
                "WPNAV_SPEED",
                "AVOID_ENABLE",
                "FENCE_ENABLE",
                "SIM_WIND_DIR",
                "SIM_WIND_SPD",
                "SIM_WIND_TURB",
            )
        },
        "probe_scope": {
            "home_radius_m": HOME_RADIUS_M,
            "legal_winds_m_s": LEGAL_WINDS_M_S,
            "no_full_grid": True,
            "classification_or_regression": False,
            "decisive_tag_created": False,
        },
        "summary": summary,
        "runs": runs,
    }
    payload["verdict"] = verdict(summary)
    payload["artifacts"] = {
        "got_home_vs_wind_plot": plot_got_home(summary["flip_table"], PLANC_ROOT / "analysis" / f"{STEM}_got_home_vs_wind.png")
    }
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a minimal energy/headwind feasibility probe.")
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "rtl_energy_config.yaml")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    config = make_config(load_config(args.config))
    results_dir = PLANC_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    partial_path = results_dir / f"{STEM}_partial.json"
    final_path = results_dir / f"{STEM}_result.json"

    env = probe_environment(config_with_model(config, "nominal"), REPO_ROOT)
    write_env(env, results_dir / f"env_{STEM}.json")

    runs: list[dict[str, Any]] = []
    if args.resume and partial_path.exists():
        runs = list(json.loads(partial_path.read_text(encoding="utf-8")).get("runs", []))

    for spec in initial_specs():
        run_or_reuse(config, runs, partial_path, spec)

    sigma_point = choose_sigma_point(runs)
    if sigma_point is not None:
        distance_m, wind_m_s = sigma_point
        existing_reps = {
            int(run.get("rep_index", 0))
            for run in runs
            if float(run.get("distance_m", -1)) == distance_m and float(run.get("wind_m_s", -1)) == wind_m_s
        }
        for rep_index in range(SIGMA_REPS):
            if rep_index in existing_reps:
                continue
            run_or_reuse(config, runs, partial_path, {
                "distance_m": distance_m,
                "wind_m_s": wind_m_s,
                "rep_index": rep_index,
                "roles": ["D_sigma_boundary"],
            })

    payload = build_payload(config, env, runs)
    write_json(final_path, payload)
    print(f"VERDICT {payload['verdict']['verdict']}: {payload['verdict']['reason']}", flush=True)
    print(f"RESULT {final_path}", flush=True)
    print(f"REPORT {payload['artifacts']['report']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
