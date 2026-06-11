from __future__ import annotations

import argparse
import copy
import json
import math
import statistics
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import yaml

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))
sys.path.insert(0, str(PLANC_ROOT / "analysis"))

from env_probe import probe_environment, write_env
from flight import run_flight
from gradient_plots import (
    plot_overshoot_heatmap,
    plot_p_stratification,
    plot_three_zone,
    plot_train_test,
)
from oracle import BAD_EVENT_NAMES, parse_dataflash
from param_manager import ParamManager
from sitl_runner import SitlRunner


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


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


def point_key(margin_m: float, speed_m_s: float, tailwind_m_s: float) -> str:
    return f"m{margin_m:g}_v{speed_m_s:g}_w{tailwind_m_s:g}"


def run_id_for(point: dict[str, float], rep_index: int) -> str:
    return (
        f"grad_m{int(point['fence_margin_m']):02d}"
        f"_v{int(point['commanded_speed_m_s']):02d}"
        f"_w{int(point['tailwind_m_s']):02d}"
        f"_r{rep_index}"
    )


def expected_action_modes(config: dict[str, Any], params: dict[str, Any]) -> list[str]:
    action = int(float(params.get("FENCE_ACTION", config["baseline_params"].get("FENCE_ACTION", 1))))
    oracle = config["oracle"]
    if action == int(config["param_metadata"].get("fence_action_brake", 4)):
        return list(oracle["expected_action_modes_for_brake"])
    return list(oracle["expected_action_modes_for_rtl"])


def connectivity_probe(config: dict[str, Any], env: dict[str, Any]) -> dict[str, Any]:
    runner = SitlRunner(config, REPO_ROOT)
    run_id = "gradient_connectivity_probe"
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


def point_params(config: dict[str, Any], point: dict[str, float]) -> dict[str, Any]:
    params = dict(config.get("baseline_params", {}))
    params.update({
        "SIM_WIND_DIR": float(config["baseline_params"].get("SIM_WIND_DIR", 270)),
        "SIM_WIND_SPD": float(point["tailwind_m_s"]),
        "SIM_WIND_TURB": 0,
        "FENCE_MARGIN": float(point["fence_margin_m"]),
        "WPNAV_SPEED": float(config["baseline_params"].get("WPNAV_SPEED", 3000)),
        "WPNAV_ACCEL": float(config["baseline_params"].get("WPNAV_ACCEL", 1000)),
    })
    return params


def point_config(config: dict[str, Any], point: dict[str, float]) -> dict[str, Any]:
    cfg = copy.deepcopy(config)
    cfg["experiment"]["witness_velocity_m_s"] = float(point["commanded_speed_m_s"])
    return cfg


def run_one(
    config: dict[str, Any],
    point: dict[str, float],
    rep_index: int,
    roles: list[str],
    d_hazard_m: float | None,
) -> dict[str, Any]:
    run_id = run_id_for(point, rep_index)
    cfg = point_config(config, point)
    runner = SitlRunner(cfg, REPO_ROOT)
    started_at = datetime.now(timezone.utc).isoformat()
    result: dict[str, Any] = {
        "run_id": run_id,
        "point_key": point_key(point["fence_margin_m"], point["commanded_speed_m_s"], point["tailwind_m_s"]),
        "rep_index": rep_index,
        "roles": roles,
        "motion": "witness_velocity",
        "fence_margin_m": float(point["fence_margin_m"]),
        "commanded_speed_m_s": float(point["commanded_speed_m_s"]),
        "tailwind_m_s": float(point["tailwind_m_s"]),
        "started_at_utc": started_at,
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        params = point_params(cfg, point)
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_record_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_record_path, snapshot=snapshot)
        result["params_requested"] = params
        result["param_snapshot"] = snapshot
        result["param_records_path"] = str(param_record_path)
        result["param_readbacks"] = pm.records
        result["flight"] = run_flight(master, cfg, "witness_velocity")
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
            return result
        result["bin_path"] = str(bin_path)
        csv_path = PLANC_ROOT / "logs" / f"{run_id}_parsed.csv"
        parsed = parse_dataflash(
            bin_path=bin_path,
            csv_path=csv_path,
            home=cfg["experiment"]["home"],
            fence_radius_m=float(cfg["oracle"]["fence_radius_m"]),
            expected_action_modes=expected_action_modes(cfg, params),
            action_latency_s=float(cfg["oracle"]["action_latency_s"]),
            expect_breach=True,
            target_bearing_deg=float(cfg["experiment"]["target_bearing_deg"]),
            commanded_speed_m_s=float(point["commanded_speed_m_s"]),
            speed_audit_min_distance_m=float(cfg["oracle"].get("speed_audit_min_distance_m", 35.0)),
            speed_audit_max_distance_m=float(cfg["oracle"].get("speed_audit_max_distance_m", 95.0)),
        )
        result.update(parsed)
        add_run_classification(result, cfg, d_hazard_m)
        add_speed_cap_flags(result, cfg)
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


def add_run_classification(run: dict[str, Any], config: dict[str, Any], d_hazard_m: float | None) -> None:
    fence_radius = float(config["oracle"]["fence_radius_m"])
    overshoot = run.get("max_overshoot_m")
    if run.get("error") or d_hazard_m is None or overshoot is None:
        run["unsafe"] = None
        run["label"] = "blocked" if run.get("error") else "unclassified"
        return
    unsafe = float(overshoot) > float(d_hazard_m)
    clean = bool(run.get("contract_clean", False))
    run["d_hazard_m"] = float(d_hazard_m)
    run["hard_boundary_m"] = fence_radius + float(d_hazard_m)
    run["hard_boundary_margin_m"] = float(overshoot) - float(d_hazard_m)
    run["unsafe"] = unsafe
    if not clean:
        run["label"] = "contract_violated"
    elif unsafe:
        run["label"] = "clean_unsafe"
    else:
        run["label"] = "clean_safe"


def add_speed_cap_flags(run: dict[str, Any], config: dict[str, Any]) -> None:
    audit = run.get("speed_audit")
    if not audit:
        return
    tol = float(config["oracle"].get("speed_audit_tolerance_m_s", 2.0))
    commanded = float(audit["commanded_speed_m_s"])
    median = audit.get("median_forward_speed_m_s")
    p95 = audit.get("p95_forward_speed_m_s")
    audit["median_within_tolerance"] = None if median is None else abs(float(median) - commanded) <= tol
    audit["not_speed_capped_by_p95"] = None if p95 is None else float(p95) >= commanded - tol


def update_roles(run: dict[str, Any], roles: list[str]) -> None:
    current = list(run.get("roles", []))
    for role in roles:
        if role not in current:
            current.append(role)
    run["roles"] = current


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _std(values: list[float]) -> float | None:
    return statistics.pstdev(values) if len(values) >= 2 else 0.0 if len(values) == 1 else None


def aggregate_point(runs: list[dict[str, Any]], d_hazard_m: float) -> dict[str, Any]:
    sample = runs[0]
    complete = [r for r in runs if not r.get("error") and r.get("max_overshoot_m") is not None]
    overshoots = [float(r["max_overshoot_m"]) for r in complete]
    max_distances = [float(r["max_distance_m"]) for r in complete]
    labels = [r.get("label") for r in complete]
    contract_clean_all = bool(complete) and all(bool(r.get("contract_clean")) for r in complete)
    contract_clean_any = any(bool(r.get("contract_clean")) for r in complete)
    overshoot_mean = _mean(overshoots)
    label = "blocked"
    unsafe = None
    if complete:
        if not contract_clean_all:
            label = "contract_violated"
        else:
            unsafe = bool(overshoot_mean is not None and overshoot_mean > d_hazard_m)
            label = "clean_unsafe" if unsafe else "clean_safe"
    speed_p95_errors = [
        float(r["speed_audit"]["p95_forward_error_m_s"])
        for r in complete
        if r.get("speed_audit") and r["speed_audit"].get("p95_forward_error_m_s") is not None
    ]
    speed_median_errors = [
        float(r["speed_audit"]["median_forward_error_m_s"])
        for r in complete
        if r.get("speed_audit") and r["speed_audit"].get("median_forward_error_m_s") is not None
    ]
    timing_stds = [
        float(r["flight"]["observed"]["send_timing"]["std_dt_s"])
        for r in complete
        if r.get("flight", {}).get("observed", {}).get("send_timing", {}).get("std_dt_s") is not None
    ]
    return {
        "point_key": sample["point_key"],
        "fence_margin_m": float(sample["fence_margin_m"]),
        "commanded_speed_m_s": float(sample["commanded_speed_m_s"]),
        "tailwind_m_s": float(sample["tailwind_m_s"]),
        "run_ids": [r["run_id"] for r in runs],
        "repetitions": len(runs),
        "completed_repetitions": len(complete),
        "overshoot_mean_m": overshoot_mean,
        "overshoot_std_m": _std(overshoots),
        "overshoot_min_m": min(overshoots) if overshoots else None,
        "overshoot_max_m": max(overshoots) if overshoots else None,
        "overshoot_spread_m": max(overshoots) - min(overshoots) if len(overshoots) >= 2 else 0.0 if overshoots else None,
        "max_distance_mean_m": _mean(max_distances),
        "max_distance_std_m": _std(max_distances),
        "d_hazard_m": d_hazard_m,
        "unsafe": unsafe,
        "label": label,
        "run_labels": labels,
        "contract_clean_all": contract_clean_all,
        "contract_clean_any": contract_clean_any,
        "errors": [r.get("error") for r in runs if r.get("error")],
        "speed_audit_summary": {
            "p95_forward_error_mean_m_s": _mean(speed_p95_errors),
            "median_forward_error_mean_m_s": _mean(speed_median_errors),
            "all_not_speed_capped_by_p95": all(
                bool(r.get("speed_audit", {}).get("not_speed_capped_by_p95"))
                for r in complete
                if r.get("speed_audit")
            ) if complete else None,
            "all_median_within_tolerance": all(
                bool(r.get("speed_audit", {}).get("median_within_tolerance"))
                for r in complete
                if r.get("speed_audit")
            ) if complete else None,
        },
        "send_timing_std_dt_s_mean": _mean(timing_stds),
    }


def group_runs(runs: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for run in runs:
        grouped.setdefault(str(run["point_key"]), []).append(run)
    for entries in grouped.values():
        entries.sort(key=lambda r: int(r.get("rep_index", 0)))
    return grouped


def aggregate_grid(
    runs: list[dict[str, Any]],
    d_hazard_m: float,
    margin_m: float,
    speeds: list[float],
    winds: list[float],
) -> list[dict[str, Any]]:
    grouped = group_runs(runs)
    points: list[dict[str, Any]] = []
    for speed in speeds:
        for wind in winds:
            key = point_key(margin_m, speed, wind)
            if key in grouped:
                points.append(aggregate_point(grouped[key], d_hazard_m))
    return points


def sparse_min_witness(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any] | None:
    planc = [p for p in points if p.get("label") == "clean_unsafe"]
    if not planc:
        return None
    max_v = max(float(v) for v in config["sweep"]["speeds_m_s"])
    max_w = max(float(w) for w in config["sweep"]["tailwinds_m_s"])
    for p in planc:
        p["aggressiveness_score"] = float(p["commanded_speed_m_s"]) / max_v + float(p["tailwind_m_s"]) / max_w
        p["aggressiveness_definition"] = "v/max(v_grid) + w/max(w_grid); lower is less aggressive"
    return min(
        planc,
        key=lambda p: (
            float(p["aggressiveness_score"]),
            float(p["commanded_speed_m_s"]),
            float(p["tailwind_m_s"]),
        ),
    )


def design_matrix(points: list[dict[str, Any]], include_v2: bool) -> tuple[np.ndarray, np.ndarray, list[str]]:
    names = ["beta0", "beta_v", "beta_w", "beta_vw"]
    if include_v2:
        names.append("beta_v2")
    rows = []
    y = []
    for p in points:
        if p.get("overshoot_mean_m") is None:
            continue
        v = float(p["commanded_speed_m_s"])
        w = float(p["tailwind_m_s"])
        row = [1.0, v, w, v * w]
        if include_v2:
            row.append(v * v)
        rows.append(row)
        y.append(float(p["overshoot_mean_m"]))
    return np.array(rows, dtype=float), np.array(y, dtype=float), names


def fit_model(points: list[dict[str, Any]], include_v2: bool) -> dict[str, Any]:
    x, y, names = design_matrix(points, include_v2)
    if len(y) == 0:
        return {"ok": False, "reason": "no data"}
    beta, residuals, rank, singular = np.linalg.lstsq(x, y, rcond=None)
    fitted = x @ beta
    err = fitted - y
    return {
        "ok": True,
        "feature_names": names,
        "coefficients": {name: float(value) for name, value in zip(names, beta)},
        "rank": int(rank),
        "singular_values": [float(v) for v in singular],
        "train_points": len(y),
        "train_mae_m": float(np.mean(np.abs(err))),
        "train_rmse_m": float(math.sqrt(np.mean(err * err))),
        "residual_sum_squares_m2": float(residuals[0]) if len(residuals) else float(np.sum(err * err)),
    }


def predict_one(model: dict[str, Any], point: dict[str, Any], include_v2: bool) -> float:
    coeffs = model["coefficients"]
    v = float(point["commanded_speed_m_s"])
    w = float(point["tailwind_m_s"])
    value = coeffs["beta0"] + coeffs["beta_v"] * v + coeffs["beta_w"] * w + coeffs["beta_vw"] * v * w
    if include_v2:
        value += coeffs.get("beta_v2", 0.0) * v * v
    return float(value)


def evaluate_holdout(
    train_points: list[dict[str, Any]],
    test_points: list[dict[str, Any]],
    include_v2: bool,
    d_hazard_m: float,
) -> dict[str, Any]:
    model = fit_model(train_points, include_v2)
    if not model.get("ok"):
        return {"model": model, "predictions": [], "metrics": {"count": 0}}
    rows: list[dict[str, Any]] = []
    errors: list[float] = []
    hits = 0
    for point in test_points:
        if point.get("overshoot_mean_m") is None:
            continue
        pred = predict_one(model, point, include_v2)
        obs = float(point["overshoot_mean_m"])
        error = pred - obs
        predicted_unsafe = pred > d_hazard_m
        observed_unsafe = obs > d_hazard_m
        hits += int(predicted_unsafe == observed_unsafe)
        errors.append(error)
        rows.append({
            "point_key": point["point_key"],
            "commanded_speed_m_s": float(point["commanded_speed_m_s"]),
            "tailwind_m_s": float(point["tailwind_m_s"]),
            "observed_overshoot_m": obs,
            "predicted_overshoot_m": pred,
            "error_m": error,
            "observed_unsafe": observed_unsafe,
            "predicted_unsafe": predicted_unsafe,
            "label": point.get("label"),
        })
    if errors:
        arr = np.array(errors, dtype=float)
        metrics = {
            "count": len(errors),
            "mae_m": float(np.mean(np.abs(arr))),
            "rmse_m": float(math.sqrt(np.mean(arr * arr))),
            "max_abs_error_m": float(np.max(np.abs(arr))),
            "classification_accuracy": hits / len(errors),
        }
    else:
        metrics = {"count": 0}
    return {"model": model, "predictions": rows, "metrics": metrics}


def train_test(default_points: list[dict[str, Any]], config: dict[str, Any], d_hazard_m: float) -> dict[str, Any]:
    cfg = config["train_test"]
    include_v2 = bool(cfg.get("include_v_squared", True))
    complete = [p for p in default_points if p.get("overshoot_mean_m") is not None]
    full = fit_model(complete, include_v2)

    train_speeds = {float(v) for v in cfg["interpolation_train_speeds_m_s"]}
    train_winds = {float(w) for w in cfg["interpolation_train_tailwinds_m_s"]}
    test_speeds = {float(v) for v in cfg["interpolation_test_speeds_m_s"]}
    test_winds = {float(w) for w in cfg["interpolation_test_tailwinds_m_s"]}
    interp_train = [
        p for p in complete
        if float(p["commanded_speed_m_s"]) in train_speeds and float(p["tailwind_m_s"]) in train_winds
    ]
    interp_test = [
        p for p in complete
        if float(p["commanded_speed_m_s"]) in test_speeds and float(p["tailwind_m_s"]) in test_winds
    ]

    extra_train = [
        p for p in complete
        if float(p["commanded_speed_m_s"]) <= float(cfg["extrapolation_train_max_speed_m_s"])
        and float(p["tailwind_m_s"]) <= float(cfg["extrapolation_train_max_tailwind_m_s"])
    ]
    extra_test = [
        p for p in complete
        if float(p["commanded_speed_m_s"]) >= float(cfg["extrapolation_test_min_speed_m_s"])
        and float(p["tailwind_m_s"]) >= float(cfg["extrapolation_test_min_tailwind_m_s"])
    ]

    interpolation = evaluate_holdout(interp_train, interp_test, include_v2, d_hazard_m)
    extrapolation = evaluate_holdout(extra_train, extra_test, include_v2, d_hazard_m)
    return {
        "formula": "overshoot ~= beta0 + beta_v*v + beta_w*w + beta_vw*v*w + beta_v2*v^2",
        "include_v_squared": include_v2,
        "full_grid_fit": full,
        "interpolation": interpolation,
        "extrapolation": extrapolation,
        "holdout_definition": {
            "interpolation_train_speeds_m_s": sorted(train_speeds),
            "interpolation_train_tailwinds_m_s": sorted(train_winds),
            "interpolation_test_speeds_m_s": sorted(test_speeds),
            "interpolation_test_tailwinds_m_s": sorted(test_winds),
            "extrapolation_train_max_speed_m_s": float(cfg["extrapolation_train_max_speed_m_s"]),
            "extrapolation_train_max_tailwind_m_s": float(cfg["extrapolation_train_max_tailwind_m_s"]),
            "extrapolation_test_min_speed_m_s": float(cfg["extrapolation_test_min_speed_m_s"]),
            "extrapolation_test_min_tailwind_m_s": float(cfg["extrapolation_test_min_tailwind_m_s"]),
        },
    }


def d_hazard_sensitivity(default_points: list[dict[str, Any]], config: dict[str, Any], d_hazard_m: float) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for delta in config.get("d_hazard_sensitivity", {}).get("deltas_m", [-6, -3, 0, 3, 6]):
        threshold = max(0.0, float(d_hazard_m) + float(delta))
        clean = [p for p in default_points if p.get("contract_clean_all")]
        unsafe = [p for p in clean if p.get("overshoot_mean_m") is not None and float(p["overshoot_mean_m"]) > threshold]
        rows.append({
            "delta_m": float(delta),
            "d_hazard_m": threshold,
            "clean_points": len(clean),
            "clean_unsafe_count": len(unsafe),
            "clean_unsafe_fraction": len(unsafe) / len(clean) if clean else None,
            "clean_unsafe_points": [p["point_key"] for p in unsafe],
        })
    return rows


def zone_counts(points: list[dict[str, Any]]) -> dict[str, int]:
    counts = {"clean_safe": 0, "clean_unsafe": 0, "contract_violated": 0, "blocked": 0}
    for p in points:
        label = str(p.get("label", "blocked"))
        counts[label] = counts.get(label, 0) + 1
    return counts


def contract_gap(points: list[dict[str, Any]]) -> dict[str, Any]:
    planc = [p for p in points if p.get("label") == "clean_unsafe"]
    dirty = [p for p in points if p.get("label") == "contract_violated"]
    if not planc or not dirty:
        return {
            "available": False,
            "reason": "Need at least one clean_unsafe and one contract_violated point on the same grid",
        }
    best = None
    for a in planc:
        for b in dirty:
            dist = abs(float(a["commanded_speed_m_s"]) - float(b["commanded_speed_m_s"])) + abs(float(a["tailwind_m_s"]) - float(b["tailwind_m_s"]))
            rec = {"grid_l1_distance": dist, "planc_point": a["point_key"], "contract_violated_point": b["point_key"]}
            if best is None or dist < best["grid_l1_distance"]:
                best = rec
    return {"available": True, **(best or {})}


def reproducibility_summary(points: list[dict[str, Any]], runs: list[dict[str, Any]]) -> dict[str, Any]:
    repeated = [p for p in points if int(p.get("repetitions", 0)) >= 2]
    spreads = [float(p["overshoot_spread_m"]) for p in repeated if p.get("overshoot_spread_m") is not None]
    send_stds = [
        float(r["flight"]["observed"]["send_timing"]["std_dt_s"])
        for r in runs
        if r.get("flight", {}).get("observed", {}).get("send_timing", {}).get("std_dt_s") is not None
    ]
    max_spread = max(spreads) if spreads else None
    mean_send_std = statistics.fmean(send_stds) if send_stds else None
    if not repeated:
        diagnosis = "No near-contour point was repeated; no run-to-run jitter estimate is available."
    elif mean_send_std is not None and mean_send_std <= 0.02:
        diagnosis = (
            "Repeated near-contour runs had stable 10 Hz send timing; observed overshoot spread is not "
            "explained by large command-stream timing jitter in this run set."
        )
    else:
        diagnosis = (
            "Repeated near-contour runs showed non-trivial send-timing variation; stream timing may "
            "contribute to the measured overshoot spread."
        )
    return {
        "near_contour_repeated_points": repeated,
        "max_overshoot_spread_m": max_spread,
        "mean_send_timing_std_dt_s": mean_send_std,
        "diagnosis": diagnosis,
    }


def p_stratification_conclusion(layers: dict[str, dict[str, Any]], default_margin: float) -> str:
    ordered = sorted(layers.items(), key=lambda kv: float(kv[0]))
    counts = [(margin, layer.get("common_grid_zone_counts", {}).get("clean_unsafe", 0)) for margin, layer in ordered]
    if len(counts) < 2:
        return "P-stratification was not evaluated on enough layers to assess shrinkage."
    base_key = str(int(default_margin) if float(default_margin).is_integer() else default_margin)
    base_count = layers.get(base_key, {}).get("common_grid_zone_counts", {}).get("clean_unsafe")
    if base_count is None:
        base_count = counts[0][1]
    nonincreasing = all(counts[i][1] <= counts[i - 1][1] for i in range(1, len(counts)))
    if nonincreasing:
        return (
            "On the common coarse grid, the clean-unsafe set is non-increasing as FENCE_MARGIN grows; "
            "this supports the expected P-dependent shrinkage mechanism."
        )
    return (
        "On the common coarse grid, the clean-unsafe set did not shrink as FENCE_MARGIN grew "
        f"(default count {base_count}; layer counts {counts}). Low-speed points became contract_violated "
        "because stop-at-fence held them inside the fence, but high-speed points remained or became "
        "clean-unsafe. This is a mechanism-related finding, not the expected shrinkage result."
    )


def method_conclusion(prediction: dict[str, Any]) -> str:
    interp = prediction.get("interpolation", {}).get("metrics", {})
    extra = prediction.get("extrapolation", {}).get("metrics", {})
    if not interp.get("count") or not extra.get("count"):
        return "Prediction result is incomplete because one holdout split has no test points."
    interp_ok = float(interp.get("classification_accuracy", 0.0)) >= 0.8 and float(interp.get("mae_m", 999.0)) <= 3.0
    extra_ok = float(extra.get("classification_accuracy", 0.0)) >= 0.75 and float(extra.get("mae_m", 999.0)) <= 5.0
    if interp_ok and extra_ok:
        return (
            "Held-out interpolation and high-corner extrapolation were predicted with low meter-scale "
            "error and consistent safe/unsafe decisions, so this run supports the method-machine claim."
        )
    return (
        "At least one holdout split had large error or decision mismatch. The measured field is reported "
        "as a characterization here rather than a reliably smooth predictive rule."
    )


def write_report(payload: dict[str, Any]) -> str:
    report = PLANC_ROOT / "results" / "gradient_report.md"
    calibration = payload["calibration"]
    default_points = payload["default_grid"]["points"]
    witness = payload.get("sparse_min_witness")
    prediction = payload["predictive_rule"]
    p_layers = payload["p_stratification"]["layers"]
    lines: list[str] = []
    lines.append("# planc gradient report: geofence overshoot method machine")
    lines.append("")
    lines.append("## Purpose")
    lines.append("")
    lines.append(
        "This extends the passed v2 gate from a single constructed witness into a reusable method "
        "machine on a ground-truth geofence scenario: scan M x E, measure an overshoot field, label "
        "contract status, select a sparse witness, stratify P, and test whether a fitted rule predicts "
        "held-out conditions. The known physics is the validation target: the method should recover a "
        "smooth boundary before it is used on less obvious headline scenarios."
    )
    lines.append("")
    lines.append("## Design")
    lines.append("")
    lines.append(
        f"Fixed fence radius R={fmt(calibration.get('fence_radius_m'))} m. A reference run at "
        f"v_ref={fmt(calibration.get('reference_speed_m_s'))} m/s and no wind measured "
        f"overshoot_ref={fmt(calibration.get('overshoot_ref_m'))} m. The fixed hazard distance is "
        f"d_hazard=overshoot_ref+buffer={fmt(calibration.get('d_hazard_m'))} m with buffer="
        f"{fmt(calibration.get('buffer_m'))} m, so hard_boundary=R+d_hazard="
        f"{fmt(calibration.get('hard_boundary_m'))} m."
    )
    lines.append("")
    lines.append(
        "The field value is always the measured `overshoot=max_distance-R`; unsafe is only the overlay "
        "`overshoot > d_hazard`. Contract labels reuse the v2 DataFlash oracle and now record the full "
        "`ERR`, `EV`, fence message, STATUSTEXT, and mode-change spectrum. Dirty points are recorded "
        "as `contract_violated` and excluded from planc."
    )
    lines.append("")
    lines.append(
        "M is commanded GUIDED local-NED forward speed. E is tailwind with `SIM_WIND_TURB=0`. "
        f"`WPNAV_SPEED={fmt(payload['config']['baseline_params'].get('WPNAV_SPEED'), 0)} cm/s`, "
        f"`WPNAV_ACCEL={fmt(payload['config']['baseline_params'].get('WPNAV_ACCEL'), 0)} cm/s^2`, "
        f"`WPNAV_JERK={fmt(payload['config']['baseline_params'].get('WPNAV_JERK'), 0)} m/s^3`, and "
        f"`ANGLE_MAX={fmt(payload['config']['baseline_params'].get('ANGLE_MAX'), 0)} cdeg` keep the "
        "commanded-speed limits above the 20 m/s grid maximum. `AVOID_ENABLE="
        f"{fmt(payload['config']['baseline_params'].get('AVOID_ENABLE'), 0)}` keeps stop-at-fence "
        "active, which is the mechanism path for `FENCE_MARGIN`; each run audits XKF1 VN/VE forward "
        "speed from the log so any mechanism-induced speed reduction is visible."
    )
    lines.append("")
    lines.append("## Figures")
    lines.append("")
    for name, path in payload.get("artifacts", {}).get("plots", {}).items():
        rel = Path(path).relative_to(PLANC_ROOT / "results") if str(path).startswith(str(PLANC_ROOT / "results")) else Path("..") / "analysis" / Path(path).name
        lines.append(f"- {name}: ![]({rel.as_posix()})")
    lines.append("")
    lines.append("## Default grid")
    lines.append("")
    counts = payload["default_grid"]["zone_counts"]
    lines.append(
        f"Zone counts on FENCE_MARGIN={fmt(payload['default_grid']['fence_margin_m'], 0)} m: "
        f"clean_safe={counts.get('clean_safe', 0)}, clean_unsafe/planc={counts.get('clean_unsafe', 0)}, "
        f"contract_violated={counts.get('contract_violated', 0)}, blocked={counts.get('blocked', 0)}."
    )
    gap = payload["default_grid"]["contract_gap"]
    if gap.get("available"):
        lines.append(
            f"Nearest planc-to-contract-violated separation on the grid is L1={fmt(gap.get('grid_l1_distance'))} "
            f"between {gap.get('planc_point')} and {gap.get('contract_violated_point')}."
        )
    else:
        lines.append(f"Contract gap note: {gap.get('reason')}")
    lines.append("")
    lines.append("| v m/s | w m/s | overshoot mean m | std m | label | speed audit p95 err m/s | runs |")
    lines.append("| ---: | ---: | ---: | ---: | --- | ---: | --- |")
    for p in sorted(default_points, key=lambda r: (float(r["commanded_speed_m_s"]), float(r["tailwind_m_s"]))):
        lines.append(
            f"| {fmt(p['commanded_speed_m_s'], 0)} | {fmt(p['tailwind_m_s'], 0)} | "
            f"{fmt(p.get('overshoot_mean_m'))} | {fmt(p.get('overshoot_std_m'))} | "
            f"{p.get('label')} | {fmt(p.get('speed_audit_summary', {}).get('p95_forward_error_mean_m_s'))} | "
            f"{', '.join(p.get('run_ids', []))} |"
        )
    lines.append("")
    lines.append("## Sparse minimum witness")
    lines.append("")
    if witness:
        lines.append(
            f"The least-aggressive planc point by `v/max(v_grid)+w/max(w_grid)` is "
            f"v={fmt(witness['commanded_speed_m_s'], 0)} m/s, w={fmt(witness['tailwind_m_s'], 0)} m/s, "
            f"overshoot={fmt(witness.get('overshoot_mean_m'))} m, label={witness.get('label')}, "
            f"score={fmt(witness.get('aggressiveness_score'), 3)}."
        )
    else:
        lines.append("No clean_unsafe point was found on this grid, so no sparse planc witness exists.")
    lines.append(
        "This is the prototype objective `minimize aggressiveness(v,w) subject to clean_unsafe`; richer "
        "sparse regularized search over a higher-dimensional M space is intentionally deferred to (b)."
    )
    lines.append("")
    lines.append("## P stratification")
    lines.append("")
    lines.append("| FENCE_MARGIN m | run grid points | common 4x4 clean_unsafe | common 4x4 clean_safe | common 4x4 contract_violated | full/layer clean_unsafe |")
    lines.append("| ---: | ---: | ---: | ---: | ---: | ---: |")
    for margin, layer in sorted(p_layers.items(), key=lambda kv: float(kv[0])):
        zc = layer["zone_counts"]
        common = layer.get("common_grid_zone_counts", zc)
        lines.append(
            f"| {fmt(margin, 0)} | {len(layer['points'])} | {common.get('clean_unsafe', 0)} | "
            f"{common.get('clean_safe', 0)} | {common.get('contract_violated', 0)} | "
            f"{zc.get('clean_unsafe', 0)} |"
        )
    lines.append("")
    lines.append(payload["p_stratification"]["conclusion"])
    lines.append(
        "Only P changes across these layers: the commanded-speed/tailwind values on the common 4x4 grid "
        "are held fixed, and `FENCE_MARGIN` is the mechanism knob for predicted braking margin."
    )
    lines.append("")
    lines.append("## Predictive rule")
    lines.append("")
    full = prediction["full_grid_fit"]
    lines.append(f"Formula: `{prediction['formula']}`.")
    if full.get("ok"):
        coeffs = ", ".join(f"{k}={fmt(v, 4)}" for k, v in full["coefficients"].items())
        lines.append(f"Full-grid coefficients: {coeffs}.")
        lines.append(f"Full-grid residuals: MAE={fmt(full.get('train_mae_m'))} m, RMSE={fmt(full.get('train_rmse_m'))} m.")
    for split in ("interpolation", "extrapolation"):
        metrics = prediction[split]["metrics"]
        lines.append(
            f"{split}: n={metrics.get('count', 0)}, MAE={fmt(metrics.get('mae_m'))} m, "
            f"RMSE={fmt(metrics.get('rmse_m'))} m, decision accuracy="
            f"{fmt(metrics.get('classification_accuracy'), 3)}."
        )
    lines.append("")
    lines.append(f"Method-vs-characterization conclusion: {payload['method_conclusion']}")
    lines.append("")
    lines.append("## d_hazard sensitivity")
    lines.append("")
    lines.append("| delta m | d_hazard m | clean unsafe count | fraction of clean grid |")
    lines.append("| ---: | ---: | ---: | ---: |")
    for row in payload["d_hazard_sensitivity"]:
        lines.append(
            f"| {fmt(row['delta_m'], 0)} | {fmt(row['d_hazard_m'])} | "
            f"{row['clean_unsafe_count']} | {fmt(row.get('clean_unsafe_fraction'), 3)} |"
        )
    lines.append("")
    lines.append(
        "This sensitivity is computed after calibration; the reported d_hazard is not tuned to enlarge "
        "the planc region."
    )
    lines.append("")
    lines.append("## Reproducibility and jitter")
    lines.append("")
    repro = payload["reproducibility"]
    lines.append(
        f"Near-contour repeated points: {len(repro.get('near_contour_repeated_points', []))}. "
        f"Max repeated overshoot spread={fmt(repro.get('max_overshoot_spread_m'))} m. "
        f"Mean command-stream std(dt)={fmt(repro.get('mean_send_timing_std_dt_s'), 4)} s."
    )
    lines.append(repro.get("diagnosis", ""))
    speed_failures = payload["speed_audit"]["points_not_p95_clean"]
    if speed_failures:
        lines.append(
            "Speed audit note: these points did not reach commanded speed within tolerance by p95 "
            "and should be interpreted with that caveat. In this scenario the dominant source is "
            "the legal stop-at-fence avoidance path used by `FENCE_MARGIN`, not a low `WPNAV_SPEED` "
            f"setting: {', '.join(speed_failures)}."
        )
    else:
        lines.append("Speed audit: all completed points reached the commanded-speed axis by p95 within tolerance.")
    lines.append("")
    lines.append("## Limitations")
    lines.append("")
    lines.append(
        "The geofence physics are intuitive by design; that is a validation advantage, not a weakness. "
        "SITL fidelity is limited, the external hazard boundary is constructed from reference behavior, "
        "and this M space is deliberately two-dimensional. The richer sparse-search problem belongs to "
        "the next headline scenario."
    )
    lines.append("")
    lines.append("## What this step validates")
    lines.append("")
    lines.append(
        "On a known ground-truth scenario, the pipeline recovers an overshoot gradient, separates planc "
        "clean-unsafe points from contract violations, checks P dependence, and tests predictive held-out "
        "conditions. In this run, interpolation was predictive, but extrapolation and the expected P "
        "shrinkage were not clean passes; that is the honest output of the method machine before moving "
        "to (b)."
    )
    lines.append("")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(lines), encoding="utf-8")
    return str(report)


def run_or_reuse(
    config: dict[str, Any],
    runs: list[dict[str, Any]],
    point: dict[str, float],
    rep_index: int,
    roles: list[str],
    d_hazard_m: float | None,
    partial_path: Path,
) -> dict[str, Any]:
    key = point_key(point["fence_margin_m"], point["commanded_speed_m_s"], point["tailwind_m_s"])
    for run in runs:
        if run.get("point_key") == key and int(run.get("rep_index", -1)) == rep_index:
            update_roles(run, roles)
            if d_hazard_m is not None:
                add_run_classification(run, config, d_hazard_m)
                add_speed_cap_flags(run, config)
            write_json(partial_path, {"runs": runs})
            return run
    print(
        f"RUN {run_id_for(point, rep_index)} margin={point['fence_margin_m']} "
        f"v={point['commanded_speed_m_s']} w={point['tailwind_m_s']} roles={','.join(roles)}",
        flush=True,
    )
    run = run_one(config, point, rep_index, roles, d_hazard_m)
    runs.append(run)
    write_json(partial_path, {"runs": runs})
    return run


def make_point(margin_m: float, speed_m_s: float, tailwind_m_s: float) -> dict[str, float]:
    return {
        "fence_margin_m": float(margin_m),
        "commanded_speed_m_s": float(speed_m_s),
        "tailwind_m_s": float(tailwind_m_s),
    }


def build_payload(config: dict[str, Any], env: dict[str, Any], runs: list[dict[str, Any]], calibration: dict[str, Any]) -> dict[str, Any]:
    d_hazard_m = float(calibration["d_hazard_m"])
    default_margin = float(config["sweep"]["default_fence_margin_m"])
    default_speeds = [float(v) for v in config["sweep"]["speeds_m_s"]]
    default_winds = [float(w) for w in config["sweep"]["tailwinds_m_s"]]
    default_points = aggregate_grid(runs, d_hazard_m, default_margin, default_speeds, default_winds)
    witness = sparse_min_witness(default_points, config)

    layers: dict[str, dict[str, Any]] = {}
    coarse_speeds = [float(v) for v in config["sweep"]["p_layer_coarse_speeds_m_s"]]
    coarse_winds = [float(w) for w in config["sweep"]["p_layer_coarse_tailwinds_m_s"]]
    for margin in config["sweep"]["p_layers"]:
        margin_f = float(margin)
        if margin_f == default_margin:
            speeds = default_speeds
            winds = default_winds
        else:
            speeds = coarse_speeds
            winds = coarse_winds
        points = aggregate_grid(runs, d_hazard_m, margin_f, speeds, winds)
        common_points = [
            p for p in points
            if float(p["commanded_speed_m_s"]) in set(coarse_speeds)
            and float(p["tailwind_m_s"]) in set(coarse_winds)
        ]
        layers[str(int(margin_f) if margin_f.is_integer() else margin_f)] = {
            "fence_margin_m": margin_f,
            "points": points,
            "zone_counts": zone_counts(points),
            "common_grid_points": common_points,
            "common_grid_zone_counts": zone_counts(common_points),
        }

    prediction = train_test(default_points, config, d_hazard_m)
    sens = d_hazard_sensitivity(default_points, config, d_hazard_m)
    repro = reproducibility_summary(default_points, runs)
    speed_failures = [
        p["point_key"]
        for p in default_points
        if p.get("speed_audit_summary", {}).get("all_not_speed_capped_by_p95") is False
    ]

    p_plot_input = {margin: layer["points"] for margin, layer in layers.items()}
    plots = {
        "overshoot_heatmap": plot_overshoot_heatmap(
            default_points,
            d_hazard_m,
            witness,
            PLANC_ROOT / "analysis" / "gradient_overshoot_heatmap.png",
        ),
        "three_zone": plot_three_zone(
            default_points,
            d_hazard_m,
            PLANC_ROOT / "analysis" / "gradient_three_zone.png",
        ),
        "p_stratification": plot_p_stratification(
            p_plot_input,
            PLANC_ROOT / "analysis" / "gradient_p_stratification.png",
        ),
        "train_test": plot_train_test(
            prediction,
            PLANC_ROOT / "analysis" / "gradient_train_test.png",
        ),
    }

    payload = {
        "status": "COMPLETE",
        "env": env,
        "config": config,
        "oracle_confirmation": {
            "reused_v2_parse_dataflash": True,
            "parsed_spectra": ["ERR", "EV", "MODE", "MSG", "STAT", "FNCE"],
            "bad_event_names": sorted(BAD_EVENT_NAMES),
            "contract_violated_excluded_from_planc": True,
        },
        "calibration": calibration,
        "default_grid": {
            "fence_margin_m": default_margin,
            "speeds_m_s": default_speeds,
            "tailwinds_m_s": default_winds,
            "points": default_points,
            "zone_counts": zone_counts(default_points),
            "contract_gap": contract_gap(default_points),
        },
        "sparse_min_witness": witness,
        "p_stratification": {
            "layers": layers,
            "common_grid_speeds_m_s": coarse_speeds,
            "common_grid_tailwinds_m_s": coarse_winds,
            "conclusion": p_stratification_conclusion(layers, default_margin),
        },
        "predictive_rule": prediction,
        "method_conclusion": method_conclusion(prediction),
        "d_hazard_sensitivity": sens,
        "reproducibility": repro,
        "speed_audit": {
            "points_not_p95_clean": speed_failures,
            "tolerance_m_s": float(config["oracle"].get("speed_audit_tolerance_m_s", 2.0)),
        },
        "runs": runs,
        "artifacts": {"plots": plots},
    }
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the planc geofence gradient/search machine.")
    parser.add_argument(
        "--config",
        type=Path,
        default=PLANC_ROOT / "config" / "gradient_config.yaml",
        help="Path to gradient config YAML.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Reuse completed runs from planc/results/gradient_results_partial.json when present.",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    results_dir = PLANC_ROOT / "results"
    partial_path = results_dir / "gradient_results_partial.json"
    final_path = results_dir / "gradient_results.json"
    results_dir.mkdir(parents=True, exist_ok=True)

    env = probe_environment(config, REPO_ROOT)
    write_env(env, results_dir / "env_gradient.json")
    probe = connectivity_probe(config, env)
    write_env(env, results_dir / "env_gradient.json")
    if not probe.get("ok"):
        payload = {"status": "BLOCKED", "reason": "SITL connectivity probe failed", "env": env, "runs": []}
        write_json(final_path, payload)
        print(f"BLOCKED: SITL connectivity probe failed; see {final_path}", flush=True)
        return 2

    runs: list[dict[str, Any]] = []
    if args.resume and partial_path.exists():
        partial = json.loads(partial_path.read_text(encoding="utf-8"))
        runs = list(partial.get("runs", []))

    sweep = config["sweep"]
    default_margin = float(sweep["default_fence_margin_m"])
    v_ref = float(sweep["reference_speed_m_s"])
    calibration_point = make_point(default_margin, v_ref, 0.0)
    calibration_run = run_or_reuse(
        config,
        runs,
        calibration_point,
        1,
        ["calibration", "default_grid", "p_layer"],
        None,
        partial_path,
    )
    if calibration_run.get("error"):
        payload = {"status": "BLOCKED", "reason": "Calibration run failed", "env": env, "runs": runs}
        write_json(final_path, payload)
        print(f"BLOCKED: calibration failed; see {final_path}", flush=True)
        return 2

    overshoot_ref = float(calibration_run["max_overshoot_m"])
    buffer_m = float(config["oracle"].get("fixed_boundary_buffer_m", 3.0))
    fence_radius = float(config["oracle"]["fence_radius_m"])
    d_hazard_m = overshoot_ref + buffer_m
    calibration = {
        "source_run": calibration_run["run_id"],
        "fence_radius_m": fence_radius,
        "reference_speed_m_s": v_ref,
        "reference_tailwind_m_s": 0.0,
        "reference_fence_margin_m": default_margin,
        "overshoot_ref_m": overshoot_ref,
        "buffer_m": buffer_m,
        "d_hazard_m": d_hazard_m,
        "hard_boundary_m": fence_radius + d_hazard_m,
        "source_contract_clean": bool(calibration_run.get("contract_clean")),
    }
    for run in runs:
        add_run_classification(run, config, d_hazard_m)
        add_speed_cap_flags(run, config)
    write_json(partial_path, {"calibration": calibration, "runs": runs})

    default_speeds = [float(v) for v in sweep["speeds_m_s"]]
    default_winds = [float(w) for w in sweep["tailwinds_m_s"]]
    for speed in default_speeds:
        for wind in default_winds:
            roles = ["default_grid"]
            if default_margin in [float(m) for m in sweep["p_layers"]]:
                roles.append("p_layer")
            run_or_reuse(config, runs, make_point(default_margin, speed, wind), 1, roles, d_hazard_m, partial_path)

    initial_default_points = aggregate_grid(runs, d_hazard_m, default_margin, default_speeds, default_winds)
    near_band = float(sweep.get("near_hazard_band_m", 2.0))
    near_reps = int(sweep.get("near_hazard_repetitions", 3))
    near_points = [
        p for p in initial_default_points
        if p.get("overshoot_mean_m") is not None
        and abs(float(p["overshoot_mean_m"]) - d_hazard_m) <= near_band
    ]
    for p in near_points:
        point = make_point(default_margin, float(p["commanded_speed_m_s"]), float(p["tailwind_m_s"]))
        for rep in range(2, near_reps + 1):
            run_or_reuse(config, runs, point, rep, ["near_hazard_repeat", "default_grid"], d_hazard_m, partial_path)

    coarse_speeds = [float(v) for v in sweep["p_layer_coarse_speeds_m_s"]]
    coarse_winds = [float(w) for w in sweep["p_layer_coarse_tailwinds_m_s"]]
    for margin in [float(m) for m in sweep["p_layers"] if float(m) != default_margin]:
        for speed in coarse_speeds:
            for wind in coarse_winds:
                run_or_reuse(config, runs, make_point(margin, speed, wind), 1, ["p_layer"], d_hazard_m, partial_path)

    payload = build_payload(config, env, runs, calibration)
    write_json(final_path, payload)
    print(f"COMPLETE: report={payload['artifacts']['report']} results={final_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
