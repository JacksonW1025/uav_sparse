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

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))

import run_energy_headwind_v1 as v1
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


STEM = "energy_headwind_pstrat_ext"
V1_RESULT = PLANC_ROOT / "results" / "energy_headwind_v1_result.json"
HOME_RADIUS_M = v1.HOME_RADIUS_M
DISTANCES_M = v1.DISTANCES_M
WINDS_M_S = v1.WINDS_M_S
OLD_LAYERS_MAH = [150.0, 220.0, 300.0, 400.0]
NEW_LAYERS_MAH = [500.0, 600.0]
ALL_LAYERS_MAH = OLD_LAYERS_MAH + NEW_LAYERS_MAH
REPEAT_REPS_PER_RESIDUAL = 3
D_REACHED_RATIO_MIN = 0.90
REALISTIC_LOW_THRESHOLD_UPPER_PCT = 20.0
UNREALISTIC_CLOSURE_PCT = 40.0


def wcode(wind_m_s: float) -> str:
    return f"{int(round(float(wind_m_s) * 10)):03d}"


def pcode(batt_low_mah: float) -> str:
    return f"p{int(round(float(batt_low_mah))):03d}"


def run_id_for(batt_low_mah: float, distance_m: float, wind_m_s: float, rep_index: int) -> str:
    return f"ehpse_{pcode(batt_low_mah)}_D{int(distance_m):03d}_W{wcode(wind_m_s)}_r{rep_index:02d}"


def make_config(base: dict[str, Any], batt_low_mah: float) -> dict[str, Any]:
    cfg = v1.make_config(base, batt_low_mah)
    cfg["experiment"]["name"] = STEM
    return cfg


def capacity_summary(base_config: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    baseline = base_config.get("baseline_params", {})
    first_snapshot = next((r.get("param_snapshot") for r in runs if r.get("param_snapshot")), {})
    batt_capacity = float(first_snapshot.get("BATT_CAPACITY", baseline.get("BATT_CAPACITY", 0.0)) or 0.0)
    sim_batt_cap_ah = float(first_snapshot.get("SIM_BATT_CAP_AH", baseline.get("SIM_BATT_CAP_AH", 0.0)) or 0.0)
    model_capacity_mah = sim_batt_cap_ah * 1000.0 if sim_batt_cap_ah else None
    rows = []
    for batt in ALL_LAYERS_MAH:
        rows.append({
            "batt_low_mah": batt,
            "pct_of_BATT_CAPACITY": None if batt_capacity <= 0 else 100.0 * batt / batt_capacity,
            "pct_of_SIM_BATT_CAP_AH": None if not model_capacity_mah else 100.0 * batt / model_capacity_mah,
            "within_typical_operational_low_threshold": bool(
                batt_capacity > 0 and 100.0 * batt / batt_capacity <= REALISTIC_LOW_THRESHOLD_UPPER_PCT
            ),
            "above_unrealistic_closure_band": bool(
                batt_capacity > 0 and 100.0 * batt / batt_capacity > UNREALISTIC_CLOSURE_PCT
            ),
        })
    return {
        "BATT_CAPACITY_mAh": batt_capacity,
        "SIM_BATT_CAP_AH": sim_batt_cap_ah,
        "model_capacity_mAh_from_SIM_BATT_CAP_AH": model_capacity_mah,
        "typical_operational_low_threshold_upper_pct": REALISTIC_LOW_THRESHOLD_UPPER_PCT,
        "unrealistic_closure_pct_threshold": UNREALISTIC_CLOSURE_PCT,
        "rows": rows,
        "source": "first extension param_snapshot if available, else rtl_energy_config baseline_params",
    }


def grid_specs() -> list[dict[str, Any]]:
    specs = []
    for batt_low in NEW_LAYERS_MAH:
        for distance in DISTANCES_M:
            for wind in WINDS_M_S:
                specs.append({
                    "batt_low_mah": batt_low,
                    "distance_m": distance,
                    "wind_m_s": wind,
                    "rep_index": 0,
                    "roles": ["pstrat_ext_grid"],
                })
    return specs


def residual_repeat_specs(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    specs = []
    residuals = [
        p for p in points
        if float(p["batt_low_mah"]) in NEW_LAYERS_MAH
        and p.get("label") == "clean_unsafe"
        and bool(p.get("D_reached_for_stats"))
    ]
    for p in residuals:
        for rep_index in range(1, REPEAT_REPS_PER_RESIDUAL + 1):
            specs.append({
                "batt_low_mah": float(p["batt_low_mah"]),
                "distance_m": float(p["distance_m"]),
                "wind_m_s": float(p["wind_m_s"]),
                "rep_index": rep_index,
                "roles": ["pstrat_ext_residual_repeat"],
            })
    return specs


def d_reached_for_stats(row: dict[str, Any]) -> bool:
    ratio = row.get("achieved_commanded_ratio")
    if ratio is not None:
        try:
            return float(ratio) >= D_REACHED_RATIO_MIN
        except Exception:
            return False
    return bool(row.get("d_reached"))


def d_not_reached_for_degeneracy(row: dict[str, Any]) -> bool:
    violations = set(row.get("contract_violations", []) or [])
    return (not d_reached_for_stats(row)) or "D_not_reached_before_low_failsafe" in violations


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
            v1.decorate_run(result)
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
        v1.decorate_run(result)
        return result
    except Exception as exc:
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        classify_run(result)
        v1.decorate_run(result)
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
            v1.decorate_run(run)
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


def extension_points(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for run in runs:
        v1.decorate_run(run)
    points = v1.aggregate_points(runs)
    for point in points:
        point["source"] = "pstrat_ext"
        point["D_reached_for_stats"] = d_reached_for_stats(point)
        point["D_not_reached_for_stats"] = not point["D_reached_for_stats"]
        point["D_not_reached_or_early_low_failsafe"] = d_not_reached_for_degeneracy(point)
    return points


def load_v1_points(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    points = copy.deepcopy(payload["summary"]["points"])
    for point in points:
        point["source"] = "energy_headwind_v1"
        point["D_reached_for_stats"] = d_reached_for_stats(point)
        point["D_not_reached_for_stats"] = not point["D_reached_for_stats"]
        point["D_not_reached_or_early_low_failsafe"] = d_not_reached_for_degeneracy(point)
    return payload, points


def layer_counts(points: list[dict[str, Any]], cap: dict[str, Any]) -> dict[str, Any]:
    pct_by_layer = {float(row["batt_low_mah"]): row for row in cap["rows"]}
    rows = []
    for batt in ALL_LAYERS_MAH:
        layer = [p for p in points if float(p["batt_low_mah"]) == float(batt)]
        labels = Counter(str(p.get("label", "blocked")) for p in layer)
        d_not = [p for p in layer if bool(p.get("D_not_reached_or_early_low_failsafe"))]
        unsafe_d_reached = [
            p for p in layer
            if p.get("label") == "clean_unsafe" and bool(p.get("D_reached_for_stats"))
        ]
        rows.append({
            "batt_low_mah": batt,
            "capacity_pct": pct_by_layer.get(batt, {}).get("pct_of_BATT_CAPACITY"),
            "clean_safe": int(labels.get("clean_safe", 0)),
            "clean_unsafe": int(labels.get("clean_unsafe", 0)),
            "clean_unsafe_at_Dreached": len(unsafe_d_reached),
            "D_not_reached": len(d_not),
            "contract_violated": int(labels.get("contract_violated", 0)),
            "blocked": int(labels.get("blocked", 0)),
            "grid_point_count": len(layer),
            "D_not_reached_points": [
                {
                    "run_id": p.get("run_id"),
                    "distance_m": p.get("distance_m"),
                    "wind_m_s": p.get("wind_m_s"),
                    "label": p.get("label"),
                    "achieved_commanded_ratio": p.get("achieved_commanded_ratio"),
                    "D_reached_by_ratio": p.get("D_reached_for_stats"),
                    "contract_violations": p.get("contract_violations", []),
                }
                for p in d_not
            ],
        })
    counts = [row["clean_unsafe_at_Dreached"] for row in rows]
    return {
        "rows": rows,
        "monotonic_nonincreasing_clean_unsafe_at_Dreached": all(counts[i] <= counts[i - 1] for i in range(1, len(counts))),
    }


def point_lookup(points: list[dict[str, Any]], batt_low: float, distance: float, wind: float) -> dict[str, Any] | None:
    return next(
        (
            p for p in points
            if float(p["batt_low_mah"]) == float(batt_low)
            and float(p["distance_m"]) == float(distance)
            and float(p["wind_m_s"]) == float(wind)
        ),
        None,
    )


def previous_residual_tracking(points: list[dict[str, Any]]) -> dict[str, Any]:
    prev = [
        p for p in points
        if float(p["batt_low_mah"]) == 400.0
        and p.get("label") == "clean_unsafe"
        and bool(p.get("D_reached_for_stats"))
    ]
    rows = []
    for old in sorted(prev, key=lambda p: (float(p["distance_m"]), float(p["wind_m_s"]))):
        row = {
            "previous": {
                "run_id": old.get("run_id"),
                "batt_low_mah": old.get("batt_low_mah"),
                "distance_m": old.get("distance_m"),
                "wind_m_s": old.get("wind_m_s"),
                "shortfall_m": old.get("shortfall_m"),
                "consequence_type": old.get("consequence_type"),
            },
            "new_layers": [],
        }
        for batt in NEW_LAYERS_MAH:
            new = point_lookup(points, batt, float(old["distance_m"]), float(old["wind_m_s"]))
            if new is None:
                status = "missing"
            elif bool(new.get("D_not_reached_or_early_low_failsafe")):
                status = "D_not_reached"
            elif new.get("label") == "clean_safe" and new.get("got_home"):
                status = "closed_by_real_home"
            elif new.get("label") == "clean_unsafe":
                status = "persists"
            elif new.get("label") == "contract_violated":
                status = "contract_violated_after_Dreached"
            else:
                status = str(new.get("label", "unknown"))
            row["new_layers"].append({
                "batt_low_mah": batt,
                "status": status,
                "run_id": None if new is None else new.get("run_id"),
                "label": None if new is None else new.get("label"),
                "got_home": None if new is None else new.get("got_home"),
                "shortfall_m": None if new is None else new.get("shortfall_m"),
                "achieved_commanded_ratio": None if new is None else new.get("achieved_commanded_ratio"),
                "contract_violations": None if new is None else new.get("contract_violations", []),
                "consequence_type": None if new is None else new.get("consequence_type"),
            })
        rows.append(row)
    return {
        "source_layer_batt_low_mah": 400.0,
        "source_residual_count": len(prev),
        "rows": rows,
    }


def residual_repeat_summary(runs: list[dict[str, Any]], primary_points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    residuals = [
        p for p in primary_points
        if float(p["batt_low_mah"]) in NEW_LAYERS_MAH
        and p.get("label") == "clean_unsafe"
        and bool(p.get("D_reached_for_stats"))
    ]
    for p in residuals:
        entries = [
            run for run in runs
            if not run.get("error")
            and float(run.get("batt_low_mah")) == float(p["batt_low_mah"])
            and float(run.get("distance_m")) == float(p["distance_m"])
            and float(run.get("wind_m_s")) == float(p["wind_m_s"])
        ]
        entries.sort(key=lambda r: int(r.get("rep_index", 0)))
        labels = [r.get("label") for r in entries]
        shortfalls = [float(r.get("shortfall_m") or 0.0) for r in entries]
        out.append({
            "batt_low_mah": p["batt_low_mah"],
            "distance_m": p["distance_m"],
            "wind_m_s": p["wind_m_s"],
            "n": len(entries),
            "labels": labels,
            "stable_label": bool(labels) and all(label == labels[0] for label in labels),
            "mean_shortfall_m": statistics.fmean(shortfalls) if shortfalls else None,
            "sigma_shortfall_m": statistics.stdev(shortfalls) if len(shortfalls) >= 2 else None,
            "runs": [
                {
                    "run_id": r["run_id"],
                    "rep_index": r.get("rep_index"),
                    "label": r.get("label"),
                    "got_home": r.get("got_home"),
                    "shortfall_m": r.get("shortfall_m"),
                    "D_reached_for_stats": d_reached_for_stats(r),
                    "consequence_type": r.get("consequence_type"),
                    "contract_clean": r.get("contract_clean"),
                }
                for r in entries
            ],
        })
    return out


def extension_residual_points(points: list[dict[str, Any]], cap: dict[str, Any]) -> list[dict[str, Any]]:
    pct_by_layer = {float(row["batt_low_mah"]): row.get("pct_of_BATT_CAPACITY") for row in cap["rows"]}
    residuals = [
        p for p in points
        if float(p["batt_low_mah"]) in NEW_LAYERS_MAH
        and p.get("label") == "clean_unsafe"
        and bool(p.get("D_reached_for_stats"))
    ]
    return [
        {
            "run_id": p.get("run_id"),
            "batt_low_mah": p.get("batt_low_mah"),
            "capacity_pct": pct_by_layer.get(float(p["batt_low_mah"])),
            "distance_m": p.get("distance_m"),
            "wind_m_s": p.get("wind_m_s"),
            "shortfall_m": p.get("shortfall_m"),
            "landing_distance_m": p.get("landing_distance_m"),
            "consequence_type": p.get("consequence_type"),
            "achieved_commanded_ratio": p.get("achieved_commanded_ratio"),
            "contract_violations": p.get("contract_violations", []),
            "bin_path": p.get("bin_path"),
            "oracle_path": p.get("oracle_path"),
        }
        for p in sorted(residuals, key=lambda x: (float(x["batt_low_mah"]), float(x["distance_m"]), float(x["wind_m_s"])))
    ]


def adjudicate(summary: dict[str, Any]) -> dict[str, Any]:
    rows = {float(row["batt_low_mah"]): row for row in summary["pstrat"]["rows"]}
    counts = {b: rows[b]["clean_unsafe_at_Dreached"] for b in NEW_LAYERS_MAH}
    dnot = {b: rows[b]["D_not_reached"] for b in NEW_LAYERS_MAH}
    cap_pct = {float(row["batt_low_mah"]): row.get("pct_of_BATT_CAPACITY") for row in summary["capacity"]["rows"]}
    tracking_rows = summary["previous_400_residual_tracking"]["rows"]
    zero_layers = [b for b in NEW_LAYERS_MAH if counts[b] == 0]

    def statuses_for(layer: float) -> list[str]:
        statuses = []
        for row in tracking_rows:
            match = next((x for x in row["new_layers"] if float(x["batt_low_mah"]) == float(layer)), None)
            if match:
                statuses.append(str(match["status"]))
        return statuses

    degenerate_layers = []
    real_close_layers = []
    for layer in zero_layers:
        statuses = statuses_for(layer)
        dnot_closed = statuses.count("D_not_reached")
        real_closed = statuses.count("closed_by_real_home")
        if statuses and dnot_closed > 0 and dnot_closed >= real_closed:
            degenerate_layers.append(layer)
        elif statuses and all(status == "closed_by_real_home" for status in statuses):
            real_close_layers.append(layer)

    if all(counts[b] >= 1 for b in NEW_LAYERS_MAH):
        verdict_name = "RESIDUAL_PERSISTS"
        reason = "BATT_LOW_MAH=500 and 600 both retain clean_unsafe@Dreached>=1."
    elif degenerate_layers:
        verdict_name = "DEGENERATE"
        reason = (
            "clean_unsafe@Dreached drops to zero only where prior residual cells are dominated by D_not_reached; "
            "this is early RTL trigger, not real closure."
        )
    elif real_close_layers:
        first = min(real_close_layers)
        pct = cap_pct.get(first)
        if pct is not None and float(pct) > UNREALISTIC_CLOSURE_PCT:
            verdict_name = "RESIDUAL_PERSISTS"
            reason = (
                f"residual closes only at BATT_LOW_MAH={first:.0f} ({pct:.1f}% of capacity), "
                "above the preregistered realistic closure band; operationally this counts as residual persistence."
            )
        else:
            verdict_name = "RESIDUAL_CLOSES_REAL"
            reason = f"clean_unsafe@Dreached reaches zero at BATT_LOW_MAH={first:.0f}, and prior residual cells reached D and got home."
    elif any(counts[b] >= 1 for b in NEW_LAYERS_MAH):
        verdict_name = "RESIDUAL_PERSISTS"
        reason = "At least one added reserve layer still has clean_unsafe@Dreached, and no real closure layer is established."
    else:
        verdict_name = "DEGENERATE"
        reason = "No clean residual remains, but closure cannot be established from D-reached got_home evidence."

    return {
        "verdict": verdict_name,
        "reason": reason,
        "counts_500_600_clean_unsafe_at_Dreached": counts,
        "D_not_reached_500_600": dnot,
        "real_close_layers": real_close_layers,
        "degenerate_layers": degenerate_layers,
        "reality_rule": {
            "typical_operational_upper_pct": REALISTIC_LOW_THRESHOLD_UPPER_PCT,
            "unrealistic_closure_pct": UNREALISTIC_CLOSURE_PCT,
            "capacity_pct_500_600": {str(int(b)): cap_pct.get(b) for b in NEW_LAYERS_MAH},
        },
    }


def plot_residual_curve(pstrat: dict[str, Any], out_path: Path) -> str:
    rows = pstrat["rows"]
    x = [row["batt_low_mah"] for row in rows]
    unsafe = [row["clean_unsafe_at_Dreached"] for row in rows]
    dnot = [row["D_not_reached"] for row in rows]
    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    ax.plot(x, unsafe, marker="o", label="clean_unsafe@Dreached")
    ax.plot(x, dnot, marker="s", linestyle="--", label="D_not_reached")
    ax.axhline(0, color="black", linewidth=0.8)
    ax.axvspan(0, max(x), ymin=0, ymax=0, alpha=0)
    ax.set_xlabel("BATT_LOW_MAH")
    ax.set_ylabel("grid point count")
    ax.set_title("P-strat residual curve, v1 + extension")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return str(out_path)


def write_report(payload: dict[str, Any]) -> str:
    report_path = PLANC_ROOT / "results" / f"{STEM}_report.md"
    summary = payload["summary"]
    verdict = payload["verdict"]
    lines: list[str] = []
    lines.append(f"VERDICT: {verdict['verdict']}")
    lines.append("")
    lines.append("# 能量逆风 P 分层延伸探针")
    lines.append("")
    lines.append(f"结论：{verdict['reason']}")
    lines.append("")
    lines.append("## 拼接分层表")
    lines.append("")
    lines.append("| BATT_LOW_MAH | capacity % | clean_safe | clean_unsafe | clean_unsafe@Dreached | D_not_reached | contract_violated | blocked |")
    lines.append("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in summary["pstrat"]["rows"]:
        lines.append(
            f"| {fmt(row['batt_low_mah'], 0)} | {fmt(row['capacity_pct'], 1)} | {row['clean_safe']} | "
            f"{row['clean_unsafe']} | {row['clean_unsafe_at_Dreached']} | {row['D_not_reached']} | "
            f"{row['contract_violated']} | {row['blocked']} |"
        )
    lines.append("")
    cap = summary["capacity"]
    lines.append(
        f"`BATT_CAPACITY={fmt(cap['BATT_CAPACITY_mAh'], 0)} mAh`；"
        f"`SIM_BATT_CAP_AH={fmt(cap['SIM_BATT_CAP_AH'], 3)} Ah` "
        f"(模型容量 {fmt(cap['model_capacity_mAh_from_SIM_BATT_CAP_AH'], 0)} mAh)。"
    )
    lines.append(
        f"现实性标注：运营低电阈值通常到约 {fmt(cap['typical_operational_low_threshold_upper_pct'], 0)}%；"
        f"本探针把 >{fmt(cap['unrealistic_closure_pct_threshold'], 0)}% 的关闭视为不现实储备下的关闭，不记作 `RESIDUAL_CLOSES_REAL`。"
    )
    lines.append(
        "`D_not_reached` 在本报告中计入两类场景：`achieved/commanded < 0.9`，或 DataFlash/parser 明确标出 "
        "`D_not_reached_before_low_failsafe`。后一类表示低电 RTL 在命令距离达成前触发；即使惯性或后续轨迹让最大距离超过 0.9D，也不作为真实关闭证据。"
    )
    lines.append("")
    lines.append("## v1 400 mAh 残留格跟踪")
    lines.append("")
    lines.append("| old D | old wind | old shortfall m | new layer | status | label | got_home | achieved/commanded | shortfall m |")
    lines.append("| ---: | ---: | ---: | ---: | --- | --- | --- | ---: | ---: |")
    for row in summary["previous_400_residual_tracking"]["rows"]:
        old = row["previous"]
        for new in row["new_layers"]:
            lines.append(
                f"| {fmt(old['distance_m'], 0)} | {fmt(old['wind_m_s'], 1)} | {fmt(old['shortfall_m'])} | "
                f"{fmt(new['batt_low_mah'], 0)} | {new['status']} | {new['label']} | {new['got_home']} | "
                f"{fmt(new['achieved_commanded_ratio'], 3)} | {fmt(new['shortfall_m'])} |"
            )
    lines.append("")
    if verdict["verdict"] == "DEGENERATE":
        lines.append("## DEGENERATE 说明")
        lines.append("")
        lines.append("新增高储备档的下降由 `D_not_reached` 驱动：低电 RTL 触发太早，远距工况没有真正施加。因此这些档不能支持“残留关得掉”的结论。")
        lines.append("")
    elif verdict["verdict"] == "RESIDUAL_PERSISTS":
        lines.append("## 残留格清单")
        lines.append("")
        residuals = summary["extension_residual_points"]
        if residuals:
            lines.append("| BATT_LOW_MAH | capacity % | D | wind | shortfall m | consequence_type | run_id |")
            lines.append("| ---: | ---: | ---: | ---: | ---: | --- | --- |")
            for p in residuals:
                lines.append(
                    f"| {fmt(p['batt_low_mah'], 0)} | {fmt(p['capacity_pct'], 1)} | {fmt(p['distance_m'], 0)} | "
                    f"{fmt(p['wind_m_s'], 1)} | {fmt(p['shortfall_m'])} | {p['consequence_type']} | `{p['run_id']}` |"
                )
        else:
            lines.append("新增档没有 D-reached 残留格；本次 `RESIDUAL_PERSISTS` 来自现实性规则：关闭只发生在不现实高储备下。")
        lines.append("")
    lines.append("## 新档残留重复")
    lines.append("")
    if summary["residual_repeats"]:
        lines.append("| BATT_LOW_MAH | D | wind | n | stable | mean shortfall m | sigma m | labels |")
        lines.append("| ---: | ---: | ---: | ---: | --- | ---: | ---: | --- |")
        for row in summary["residual_repeats"]:
            lines.append(
                f"| {fmt(row['batt_low_mah'], 0)} | {fmt(row['distance_m'], 0)} | {fmt(row['wind_m_s'], 1)} | "
                f"{row['n']} | {row['stable_label']} | {fmt(row['mean_shortfall_m'])} | "
                f"{fmt(row['sigma_shortfall_m'], 3)} | {', '.join(str(x) for x in row['labels'])} |"
            )
    else:
        lines.append("无新增 D-reached clean_unsafe 残留格，因此没有触发残留重复。")
    lines.append("")
    lines.append("## 后果类型")
    lines.append("")
    lines.append(f"新增档 clean_unsafe consequence_type 分布：`{summary['extension_consequence_distribution']}`。")
    if summary["uncontrolled_points"]:
        lines.append(f"出现 uncontrolled：{', '.join(p['run_id'] for p in summary['uncontrolled_points'])}。")
    else:
        lines.append("未出现 `uncontrolled`；本探针不改变 v1 后果偏软、以 `controlled_land_away` 为主的定性。")
    lines.append("")
    lines.append("## 图")
    lines.append("")
    lines.append(f"- pstrat_residual_curve: ![]({rel(payload['artifacts']['plots']['pstrat_residual_curve'])})")
    lines.append("")
    lines.append("## 审计")
    lines.append("")
    lines.append("本探针只新增 `BATT_LOW_MAH=500/600` 两档；其它 P、D×wind 网格、harness、parser、oracle 均沿用 v1。每个新增 run 均落盘 DataFlash、param dump、parsed CSV 和 oracle sidecar。")
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return str(report_path)


def build_payload(base_config: dict[str, Any], env: dict[str, Any], runs: list[dict[str, Any]]) -> dict[str, Any]:
    v1_payload, old_points = load_v1_points(V1_RESULT)
    new_points = extension_points(runs)
    all_points = old_points + new_points
    cap = capacity_summary(base_config, runs)
    pstrat = layer_counts(all_points, cap)
    primary_new = [p for p in new_points if int(float(p.get("rep_index", 0))) == 0]
    summary = {
        "capacity": cap,
        "pstrat": pstrat,
        "previous_400_residual_tracking": previous_residual_tracking(all_points),
        "extension_residual_points": extension_residual_points(new_points, cap),
        "residual_repeats": residual_repeat_summary(runs, primary_new),
        "extension_consequence_distribution": dict(Counter(
            str(p.get("consequence_type", "ambiguous"))
            for p in new_points
            if p.get("label") == "clean_unsafe"
        )),
        "uncontrolled_points": [
            p for p in new_points
            if p.get("label") == "clean_unsafe" and p.get("consequence_type") == "uncontrolled"
        ],
        "v1_source": {
            "result": str(V1_RESULT),
            "verdict": v1_payload.get("verdict", {}).get("verdict"),
            "commit_context": "energy-headwind-v1 inputs are read-only and not regenerated by this extension",
        },
        "new_points": primary_new,
    }
    payload = {
        "status": "COMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "experiment": STEM,
        "env": env,
        "design": {
            "old_layers_mAh_from_v1": OLD_LAYERS_MAH,
            "new_layers_mAh": NEW_LAYERS_MAH,
            "distances_m": DISTANCES_M,
            "winds_m_s": WINDS_M_S,
            "home_radius_m": HOME_RADIUS_M,
            "D_reached_statistic": f"achieved_commanded_ratio >= {D_REACHED_RATIO_MIN}",
            "D_not_reached_degeneracy_count": "achieved_commanded_ratio < 0.9 OR parser contract violation D_not_reached_before_low_failsafe",
            "residual_repeat_reps_per_point": REPEAT_REPS_PER_RESIDUAL,
            "fixed_params_inherited_from_v1": {
                "BATT_FS_LOW_ACT": 2,
                "BATT_FS_CRT_ACT": 1,
                "RTL_ALT": 2000,
                "WPNAV_SPEED": 800,
                "AVOID_ENABLE": 0,
                "FENCE_ENABLE": 0,
                "SIM_WIND_DIR": 270,
            },
        },
        "preregistered_decision_block": {
            "RESIDUAL_PERSISTS": "BATT_LOW_MAH in {500,600} retains clean_unsafe@Dreached>=1, or closure occurs only at unrealistic reserve percentages.",
            "RESIDUAL_CLOSES_REAL": "some added layer reaches zero clean_unsafe@Dreached because previously failing cells reached D and got_home, within realistic reserve percentages.",
            "DEGENERATE": "apparent drop is driven by D_not_reached from early low-battery RTL.",
            "non_gates": ["no ML", "no oracle change", "no rescan of old layers"],
        },
        "summary": summary,
        "runs": runs,
    }
    payload["verdict"] = adjudicate(summary)
    analysis = PLANC_ROOT / "analysis"
    payload["artifacts"] = {
        "plots": {
            "pstrat_residual_curve": plot_residual_curve(pstrat, analysis / f"{STEM}_pstrat_residual_curve.png"),
        }
    }
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the energy-headwind P-stratification extension probe.")
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "rtl_energy_config.yaml")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    base_config = load_config(args.config)
    results_dir = PLANC_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    partial_path = results_dir / f"{STEM}_partial.json"
    final_path = results_dir / f"{STEM}_result.json"

    main_config = make_config(base_config, NEW_LAYERS_MAH[0])
    env = probe_environment(config_with_model(main_config, "nominal"), REPO_ROOT)
    write_env(env, results_dir / f"env_{STEM}.json")

    runs: list[dict[str, Any]] = []
    if args.resume and partial_path.exists():
        runs = list(json.loads(partial_path.read_text(encoding="utf-8")).get("runs", []))
        for run in runs:
            v1.decorate_run(run)

    specs = grid_specs()
    for idx, spec in enumerate(specs, start=1):
        print(f"GRID PROGRESS {idx}/{len(specs)}", flush=True)
        run_or_reuse(base_config, runs, partial_path, spec)

    primary_new = extension_points(runs)
    repeat_specs = residual_repeat_specs(primary_new)
    for idx, spec in enumerate(repeat_specs, start=1):
        print(f"REPEAT PROGRESS {idx}/{len(repeat_specs)}", flush=True)
        run_or_reuse(base_config, runs, partial_path, spec)

    payload = build_payload(base_config, env, runs)
    write_json(final_path, payload)
    print(f"VERDICT {payload['verdict']['verdict']}: {payload['verdict']['reason']}", flush=True)
    print(f"RESULT {final_path}", flush=True)
    print(f"REPORT {payload['artifacts']['report']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
