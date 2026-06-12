from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
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
sys.path.insert(0, str(PLANC_ROOT / "analysis"))

from env_probe import probe_environment, write_env
from linkloss_plots import plot_premise
from run_linkloss_excursion import (
    load_config,
    premise_summary,
    rel,
    run_or_reuse,
    trigger_distance_m,
    write_json,
)


CONFIDENT_LABELS = {"clean_safe", "clean_unsafe"}
ZONE_COLORS = {
    "clean_safe": "#2a9d8f",
    "clean_unsafe": "#d62828",
    "ambiguous": "#f4a261",
    "contract_violated": "#6c757d",
    "blocked": "#adb5bd",
}


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def point_key(layer: str, speed_m_s: float, wind_m_s: float) -> str:
    return f"{layer}_v{int(speed_m_s):02d}_w{int(wind_m_s):02d}"


def v2_run_id(layer: str, speed_m_s: float, wind_m_s: float, rep_index: int) -> str:
    return f"linkloss_v2_{layer}_v{int(speed_m_s):02d}_w{int(wind_m_s):02d}_r{int(rep_index):02d}"


def load_json(path: Path, default: dict[str, Any] | None = None) -> dict[str, Any]:
    if not path.exists():
        return {} if default is None else default
    return json.loads(path.read_text(encoding="utf-8"))


def load_runs(path: Path, resume: bool) -> list[dict[str, Any]]:
    if not resume or not path.exists():
        return []
    return list(load_json(path).get("runs", []))


def run_exists(runs: list[dict[str, Any]], run_id: str) -> bool:
    return any(r.get("run_id") == run_id for r in runs)


def run_count_for_point(runs: list[dict[str, Any]], layer: str, speed_m_s: float, wind_m_s: float) -> int:
    key = point_key(layer, speed_m_s, wind_m_s)
    return sum(1 for r in runs if r.get("point_key") == key)


def tag_run(run: dict[str, Any], **fields: Any) -> None:
    for key, value in fields.items():
        run[key] = value


def maybe_schedule_run(
    config: dict[str, Any],
    runs: list[dict[str, Any]],
    partial_path: Path,
    *,
    run_id: str,
    run_kind: str,
    layer: str,
    speed_m_s: float,
    wind_m_s: float,
    timeout_s: float,
    rep_index: int,
    roles: list[str],
    max_new_runs_state: dict[str, int | None],
) -> bool:
    existed = run_exists(runs, run_id)
    if not existed:
        remaining = max_new_runs_state.get("remaining")
        if remaining is not None and int(remaining) <= 0:
            return False
        if remaining is not None:
            max_new_runs_state["remaining"] = int(remaining) - 1
    run = run_or_reuse(
        config,
        runs,
        partial_path,
        run_id=run_id,
        run_kind=run_kind,
        layer=layer,
        speed_m_s=speed_m_s,
        wind_m_s=wind_m_s,
        timeout_s=timeout_s,
        rep_index=rep_index,
        roles=roles,
    )
    tag_run(run, v2_campaign=True)
    write_json(partial_path, {"runs": runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})
    return True


def append_seed_run(runs: list[dict[str, Any]], run: dict[str, Any], *, layer: str | None = None, role: str = "v1_seed") -> None:
    run_id = str(run.get("run_id"))
    target_point_key = (
        point_key(layer, float(run["speed_m_s"]), float(run["wind_m_s"]))
        if layer is not None
        else run.get("point_key")
    )
    if any(str(existing.get("run_id")) == run_id and existing.get("point_key") == target_point_key for existing in runs):
        return
    seeded = dict(run)
    if layer is not None:
        seeded["layer"] = layer
        seeded["point_key"] = target_point_key
    roles = list(seeded.get("roles", []))
    for item in [role, "v1_existing_sitl_log"]:
        if item not in roles:
            roles.append(item)
    seeded["roles"] = roles
    seeded["v2_seed_source"] = {
        "campaign": "planc/linkloss-v1-20260611",
        "reason": "existing raw SITL repetition reused before backfilling to reduce duplicate runtime",
    }
    runs.append(seeded)


def seed_v1_grid_runs(v1: dict[str, Any], grid_runs: list[dict[str, Any]], partial_path: Path) -> int:
    before = len(grid_runs)
    for run in v1.get("runs", {}).get("grid", []):
        if run.get("layer") in {"conservative", "default", "lenient"}:
            append_seed_run(grid_runs, run, role="v1_grid_seed")
    if len(grid_runs) != before:
        write_json(partial_path, {"runs": grid_runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})
    return len(grid_runs) - before


def seed_v1_noise_runs(v1: dict[str, Any], config: dict[str, Any], noise_runs: list[dict[str, Any]], partial_path: Path) -> int:
    before = len(noise_runs)
    source = [
        r for r in v1.get("runs", {}).get("grid", [])
        if r.get("layer") == "default" and not r.get("error")
    ]
    for group_name, cfg_name in (("boundary", "boundary_points"), ("unsafe", "unsafe_points")):
        layer = f"noise_{group_name}"
        wanted = {(float(spec["speed_m_s"]), float(spec["wind_m_s"])) for spec in config["noise_floor"][cfg_name]}
        for run in source:
            key = (float(run.get("speed_m_s")), float(run.get("wind_m_s")))
            if key in wanted:
                append_seed_run(noise_runs, run, layer=layer, role=f"v1_noise_{group_name}_seed")
    if len(noise_runs) != before:
        write_json(partial_path, {"runs": noise_runs, "updated_at_utc": datetime.now(timezone.utc).isoformat()})
    return len(noise_runs) - before


def pooled_sigma(points: list[dict[str, Any]]) -> float:
    ss = 0.0
    df = 0
    for point in points:
        values = [float(v) for v in point.get("overshoot_values_m", [])]
        if len(values) < 2:
            continue
        mean = statistics.fmean(values)
        ss += sum((v - mean) ** 2 for v in values)
        df += len(values) - 1
    if df <= 0:
        return 0.0
    return math.sqrt(ss / df)


def summarize_noise_runs(noise_runs: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = {"boundary": [], "unsafe": []}
    for group_name, cfg_name in (("boundary", "boundary_points"), ("unsafe", "unsafe_points")):
        for spec in config["noise_floor"][cfg_name]:
            layer = f"noise_{group_name}"
            key = point_key(layer, float(spec["speed_m_s"]), float(spec["wind_m_s"]))
            runs = [
                r for r in noise_runs
                if r.get("point_key") == key and not r.get("error") and r.get("severity_overshoot_m") is not None
            ]
            values = [float(r["severity_overshoot_m"]) for r in runs]
            point = {
                "point_key": key,
                "group": group_name,
                "speed_m_s": float(spec["speed_m_s"]),
                "wind_m_s": float(spec["wind_m_s"]),
                "run_ids": [r.get("run_id") for r in runs],
                "completed_repetitions": len(runs),
                "overshoot_values_m": values,
                "mean_overshoot_m": statistics.fmean(values) if values else None,
                "sample_std_m": statistics.stdev(values) if len(values) >= 2 else 0.0 if values else None,
                "contract_clean_all": bool(runs) and all(bool(r.get("contract_clean")) for r in runs),
                "contract_violations": sorted({v for r in runs for v in r.get("contract_violations", [])}),
            }
            groups[group_name].append(point)

    sigma_boundary = pooled_sigma(groups["boundary"])
    sigma_unsafe = pooled_sigma(groups["unsafe"])
    k = float(config["noise_floor"]["k_sigma_margin"])
    c = float(config["noise_floor"]["c_sigma_mae_bound"])
    ci_mult = float(config["noise_floor"]["ci_sigma_multiplier"])
    d_margin = k * sigma_boundary
    return {
        "measured_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": "SITL repeated measurements at preregistered boundary and unsafe points",
        "boundary": {
            "points": groups["boundary"],
            "sigma_m": sigma_boundary,
        },
        "unsafe": {
            "points": groups["unsafe"],
            "sigma_m": sigma_unsafe,
        },
        "sigma_boundary_m": sigma_boundary,
        "sigma_unsafe_m": sigma_unsafe,
        "k_sigma_margin": k,
        "d_margin_m": d_margin,
        "hard_threshold_distance_m": float(config["experiment"]["fence_radius_m"]) + d_margin,
        "ci_sigma_multiplier": ci_mult,
        "ambiguous_band_overshoot_m": [d_margin - ci_mult * sigma_boundary, d_margin + ci_mult * sigma_boundary],
        "ambiguous_band_width_m": 2.0 * ci_mult * sigma_boundary,
        "c_sigma_mae_bound": c,
        "mae_bound_m": c * sigma_boundary,
    }


def completed_grid_runs(runs: list[dict[str, Any]], layer: str, speed_m_s: float, wind_m_s: float) -> list[dict[str, Any]]:
    key = point_key(layer, speed_m_s, wind_m_s)
    return [r for r in runs if r.get("point_key") == key and not r.get("error")]


def aggregate_point_v2(
    runs: list[dict[str, Any]],
    *,
    layer: str,
    speed_m_s: float,
    wind_m_s: float,
    timeout_s: float,
    required_repetitions: int,
    oracle: dict[str, Any],
    raw_tolerance_m: float,
) -> dict[str, Any]:
    complete = completed_grid_runs(runs, layer, speed_m_s, wind_m_s)
    overshoots = [
        float(r["severity_overshoot_m"])
        for r in complete
        if r.get("severity_overshoot_m") is not None
    ]
    max_distances = [
        float(r["max_distance_m"])
        for r in complete
        if r.get("max_distance_m") is not None
    ]
    observed_timeouts = [
        float(r["gcs_failsafe_timeout_observed_s"])
        for r in complete
        if r.get("gcs_failsafe_timeout_observed_s") is not None
    ]
    mean = statistics.fmean(overshoots) if overshoots else None
    sigma = float(oracle["sigma_boundary_m"])
    ci_mult = float(oracle["ci_sigma_multiplier"])
    d_margin = float(oracle["d_margin_m"])
    ci_low = None if mean is None else mean - ci_mult * sigma
    ci_high = None if mean is None else mean + ci_mult * sigma
    contract_violations = sorted({v for r in complete for v in r.get("contract_violations", [])})
    contract_clean_all = bool(complete) and len(complete) >= required_repetitions and all(bool(r.get("contract_clean")) for r in complete)
    errors = [r.get("error") for r in runs if r.get("point_key") == point_key(layer, speed_m_s, wind_m_s) and r.get("error")]

    if len(complete) < required_repetitions or mean is None:
        label = "blocked"
        reason = "incomplete_repetitions"
    elif not contract_clean_all:
        label = "contract_violated"
        reason = "contract_violated"
    elif mean > d_margin and ci_low is not None and ci_low > d_margin:
        label = "clean_unsafe"
        reason = "mean_and_lower_CI_exceed_noise_margin"
    elif mean < d_margin and ci_high is not None and ci_high < d_margin:
        label = "clean_safe"
        reason = "mean_and_upper_CI_below_noise_margin"
    else:
        label = "ambiguous"
        reason = "label_CI_crosses_noise_margin"

    return {
        "point_key": point_key(layer, speed_m_s, wind_m_s),
        "layer": layer,
        "speed_m_s": float(speed_m_s),
        "wind_m_s": float(wind_m_s),
        "timeout_s": float(timeout_s),
        "required_repetitions": int(required_repetitions),
        "repetitions": run_count_for_point(runs, layer, speed_m_s, wind_m_s),
        "completed_repetitions": len(complete),
        "run_ids": [r.get("run_id") for r in complete],
        "label": label,
        "label_reason": reason,
        "stable_binary": label in CONFIDENT_LABELS,
        "mean_overshoot_m": mean,
        "severity_overshoot_m": mean,
        "sample_std_overshoot_m": statistics.stdev(overshoots) if len(overshoots) >= 2 else 0.0 if overshoots else None,
        "overshoot_values_m": overshoots,
        "label_ci_low_m": ci_low,
        "label_ci_high_m": ci_high,
        "d_margin_m": d_margin,
        "max_distance_m": statistics.fmean(max_distances) if max_distances else None,
        "gcs_failsafe_timeout_observed_s": statistics.fmean(observed_timeouts) if observed_timeouts else None,
        "contract_clean_all": contract_clean_all,
        "contract_violations": contract_violations,
        "errors": errors,
        "raw_oracle_tolerance_m": float(raw_tolerance_m),
        "raw_oracle_mean_label": None if mean is None else ("unsafe" if mean > float(raw_tolerance_m) else "safe"),
    }


def aggregate_layer_v2(
    runs: list[dict[str, Any]],
    *,
    layer: str,
    timeout_s: float,
    speeds: list[float],
    winds: list[float],
    required_repetitions: int,
    oracle: dict[str, Any],
    raw_tolerance_m: float,
) -> list[dict[str, Any]]:
    points = []
    for speed in speeds:
        for wind in winds:
            points.append(
                aggregate_point_v2(
                    runs,
                    layer=layer,
                    speed_m_s=speed,
                    wind_m_s=wind,
                    timeout_s=timeout_s,
                    required_repetitions=required_repetitions,
                    oracle=oracle,
                    raw_tolerance_m=raw_tolerance_m,
                )
            )
    return points


def zone_counts(points: list[dict[str, Any]]) -> dict[str, int]:
    counts = {"clean_safe": 0, "clean_unsafe": 0, "ambiguous": 0, "contract_violated": 0, "blocked": 0}
    for point in points:
        label = str(point.get("label", "blocked"))
        counts[label] = counts.get(label, 0) + 1
    return counts


def split_points(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    cfg = config["train_test"]
    interp_train_v = {float(v) for v in cfg["interpolation_train_speeds_m_s"]}
    interp_train_w = {float(v) for v in cfg["interpolation_train_winds_m_s"]}
    interp_test_v = {float(v) for v in cfg["interpolation_test_speeds_m_s"]}
    interp_test_w = {float(v) for v in cfg["interpolation_test_winds_m_s"]}
    return {
        "interpolation_train": [p for p in points if float(p["speed_m_s"]) in interp_train_v and float(p["wind_m_s"]) in interp_train_w],
        "interpolation_test": [p for p in points if float(p["speed_m_s"]) in interp_test_v and float(p["wind_m_s"]) in interp_test_w],
        "extrapolation_train": [
            p for p in points
            if float(p["speed_m_s"]) <= float(cfg["extrapolation_train_max_speed_m_s"])
            and float(p["wind_m_s"]) <= float(cfg["extrapolation_train_max_wind_m_s"])
        ],
        "extrapolation_test": [
            p for p in points
            if float(p["speed_m_s"]) >= float(cfg["extrapolation_test_min_speed_m_s"])
            and float(p["wind_m_s"]) >= float(cfg["extrapolation_test_min_wind_m_s"])
        ],
    }


def evaluate_classification_split(train: list[dict[str, Any]], test: list[dict[str, Any]], oracle: dict[str, Any]) -> dict[str, Any]:
    model = fit_severity_regression(train)
    rows = []
    hits = 0
    confident_test = [p for p in test if p.get("label") in CONFIDENT_LABELS]
    excluded = [p for p in test if p.get("label") == "ambiguous"]
    if model.get("ok"):
        for point in confident_test:
            predicted_overshoot = predict_severity(model, point)
            scale = max(float(oracle["sigma_boundary_m"]), 1.0e-6)
            prob = 1.0 / (1.0 + math.exp(-max(-40.0, min(40.0, (predicted_overshoot - float(oracle["d_margin_m"])) / scale))))
            pred = prob >= 0.5
            obs = point["label"] == "clean_unsafe"
            hits += int(pred == obs)
            rows.append({
                "point_key": point["point_key"],
                "speed_m_s": float(point["speed_m_s"]),
                "wind_m_s": float(point["wind_m_s"]),
                "observed_label": point["label"],
                "observed_unsafe": obs,
                "predicted_overshoot_m": predicted_overshoot,
                "probability_unsafe": prob,
                "predicted_unsafe": pred,
                "correct": pred == obs,
            })
    return {
        "model": model,
        "predictions": rows,
        "excluded_ambiguous": [
            {
                "point_key": p["point_key"],
                "speed_m_s": p["speed_m_s"],
                "wind_m_s": p["wind_m_s"],
                "mean_overshoot_m": p.get("mean_overshoot_m"),
                "label_ci_low_m": p.get("label_ci_low_m"),
                "label_ci_high_m": p.get("label_ci_high_m"),
                "d_margin_m": p.get("d_margin_m"),
                "exclusion_basis": "label CI crosses d_margin",
            }
            for p in excluded
        ],
        "metrics": {
            "count": len(confident_test),
            "ambiguous_excluded_count": len(excluded),
            "classification_accuracy": (hits / len(confident_test)) if confident_test else None,
        },
    }


def train_test_classification(points: list[dict[str, Any]], config: dict[str, Any], oracle: dict[str, Any]) -> dict[str, Any]:
    splits = split_points(points, config)
    interpolation = evaluate_classification_split(splits["interpolation_train"], splits["interpolation_test"], oracle)
    extrapolation = evaluate_classification_split(splits["extrapolation_train"], splits["extrapolation_test"], oracle)
    combined_rows = interpolation["predictions"] + extrapolation["predictions"]
    combined_acc = None
    if combined_rows:
        combined_acc = sum(1 for r in combined_rows if r["correct"]) / len(combined_rows)
    return {
        "formula": "unsafe probability = sigmoid((severity_model(v, wind) - d_margin) / sigma_boundary)",
        "interpolation": interpolation,
        "extrapolation": extrapolation,
        "combined_heldout_accuracy": combined_acc,
        "holdout_definition": config["train_test"],
        "ambiguous_exclusion_basis": "Only label uncertainty is excluded from classification; prediction correctness is not consulted.",
    }


def severity_feature_row(point: dict[str, Any]) -> list[float]:
    v = float(point["speed_m_s"])
    w = float(point["wind_m_s"])
    return [1.0, v, v * v, w, v * w, w * w]


def fit_severity_regression(points: list[dict[str, Any]], ridge: float = 1.0e-6) -> dict[str, Any]:
    usable = [p for p in points if p.get("severity_overshoot_m") is not None and p.get("label") != "blocked"]
    if len(usable) < 6:
        return {"ok": False, "reason": "not enough training points", "train_points": len(usable)}
    x = np.array([severity_feature_row(p) for p in usable], dtype=float)
    y = np.array([float(p["severity_overshoot_m"]) for p in usable], dtype=float)
    penalty = ridge * np.eye(x.shape[1], dtype=float)
    penalty[0, 0] = 0.0
    beta = np.linalg.pinv(x.T @ x + penalty) @ (x.T @ y)
    pred = np.maximum(0.0, x @ beta)
    train_mae = float(np.mean(np.abs(pred - y)))
    return {
        "ok": True,
        "feature_names": ["intercept", "v", "v^2", "wind", "v*wind", "wind^2"],
        "coefficients": [float(v) for v in beta],
        "ridge": ridge,
        "train_points": len(usable),
        "train_mae_m": train_mae,
    }


def predict_severity(model: dict[str, Any], point: dict[str, Any]) -> float:
    beta = np.array(model["coefficients"], dtype=float)
    pred = float(np.array(severity_feature_row(point), dtype=float) @ beta)
    return max(0.0, pred)


def evaluate_severity_regression(points: list[dict[str, Any]], config: dict[str, Any], oracle: dict[str, Any]) -> dict[str, Any]:
    splits = split_points(points, config)
    holdout_by_key: dict[str, dict[str, Any]] = {}
    for point in splits["interpolation_test"] + splits["extrapolation_test"]:
        holdout_by_key[str(point["point_key"])] = point
    holdout_keys = set(holdout_by_key)
    train = [p for p in points if str(p["point_key"]) not in holdout_keys and p.get("label") != "blocked"]
    holdout = [holdout_by_key[k] for k in sorted(holdout_by_key)]
    model = fit_severity_regression(train)
    rows = []
    if model.get("ok"):
        for point in holdout:
            pred = predict_severity(model, point)
            obs = float(point.get("severity_overshoot_m") or 0.0)
            rows.append({
                "point_key": point["point_key"],
                "speed_m_s": float(point["speed_m_s"]),
                "wind_m_s": float(point["wind_m_s"]),
                "observed_overshoot_m": obs,
                "predicted_overshoot_m": pred,
                "abs_error_m": abs(pred - obs),
                "label": point.get("label"),
                "included_boundary_or_ambiguous": point.get("label") == "ambiguous",
            })
    mae = float(np.mean([r["abs_error_m"] for r in rows])) if rows else None
    return {
        "formula": "overshoot_m = max(0, beta0 + beta_v*v + beta_v2*v^2 + beta_w*wind + beta_vw*v*wind + beta_w2*wind^2)",
        "model": model,
        "train_point_count": len(train),
        "holdout_point_count": len(holdout),
        "holdout_points_include_ambiguous": any(p.get("label") == "ambiguous" for p in holdout),
        "predictions": rows,
        "metrics": {
            "mae_m": mae,
            "mae_bound_m": float(oracle["mae_bound_m"]),
            "pass": bool(mae is not None and mae <= float(oracle["mae_bound_m"])),
        },
    }


def p_stratification_summary(layers: dict[str, dict[str, Any]]) -> dict[str, Any]:
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1]["timeout_s"]))
    counts = [
        {
            "layer": name,
            "timeout_s": float(layer["timeout_s"]),
            "clean_unsafe": int(layer["zone_counts"].get("clean_unsafe", 0)),
            "clean_safe": int(layer["zone_counts"].get("clean_safe", 0)),
            "ambiguous": int(layer["zone_counts"].get("ambiguous", 0)),
            "contract_violated": int(layer["zone_counts"].get("contract_violated", 0)),
        }
        for name, layer in ordered
    ]
    nondecreasing = all(counts[i]["clean_unsafe"] >= counts[i - 1]["clean_unsafe"] for i in range(1, len(counts)))
    return {
        "counts": counts,
        "monotonic_expansion_with_timeout": nondecreasing,
        "conclusion": "noise-aware clean_unsafe count is nondecreasing as FS_GCS_TIMEOUT lengthens; shorter timeout shrinks the unsafe region"
        if nondecreasing
        else "noise-aware clean_unsafe count did not expand monotonically as FS_GCS_TIMEOUT lengthened",
    }


def boundary_search_summary(points: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    speeds = [float(v) for v in config["sweep"]["speeds_m_s"]]
    winds = [float(v) for v in config["search"]["winds_m_s"]]
    by = {(float(p["speed_m_s"]), float(p["wind_m_s"])): p for p in points}
    records = []
    total_queries = 0
    for wind in winds:
        lo = 0
        hi = len(speeds) - 1
        queries = []
        first_unsafe = None
        while lo <= hi:
            mid = (lo + hi) // 2
            speed = speeds[mid]
            p = by.get((speed, wind))
            label = None if p is None else p.get("label")
            queries.append({"speed_m_s": speed, "wind_m_s": wind, "label": label})
            if label == "clean_unsafe":
                first_unsafe = speed
                hi = mid - 1
            else:
                lo = mid + 1
            if len(queries) >= int(config["search"]["bisection_iterations_per_wind"]):
                break
        total_queries += len(queries)
        records.append({"wind_m_s": wind, "queries": queries, "first_clean_unsafe_speed_m_s": first_unsafe})
    return {
        "strategy": "discrete bisection over speed for each wind, replayed against completed noise-aware grid results",
        "query_count": total_queries,
        "full_grid_count": len([p for p in points if p.get("layer") == config["sweep"]["default_layer"]]),
        "records": records,
    }


def verdict_summary(
    premise: dict[str, Any],
    default_points: list[dict[str, Any]],
    classification: dict[str, Any],
    severity: dict[str, Any],
) -> dict[str, Any]:
    if not premise.get("satisfied"):
        return {
            "verdict": "INCONCLUSIVE",
            "premise_satisfied": False,
            "robust_clean_unsafe": False,
            "contract_clean_gap": False,
            "classification_ok": False,
            "severity_ok": False,
            "prediction_ok": False,
            "reason": "Premise failed.",
        }

    clean_unsafe = [p for p in default_points if p.get("label") == "clean_unsafe"]
    contract_violated = [p for p in default_points if p.get("label") == "contract_violated"]
    blocked = [p for p in default_points if p.get("label") == "blocked"]
    robust_clean_unsafe = bool(len(clean_unsafe) >= 2 and all(p.get("stable_binary") for p in clean_unsafe))
    contract_clean_gap = bool(clean_unsafe and not contract_violated and not blocked and all(p.get("contract_clean_all") for p in clean_unsafe))

    interp_acc = classification.get("interpolation", {}).get("metrics", {}).get("classification_accuracy")
    extra_acc = classification.get("extrapolation", {}).get("metrics", {}).get("classification_accuracy")
    combined_acc = classification.get("combined_heldout_accuracy")
    extra_count = int(classification.get("extrapolation", {}).get("metrics", {}).get("count") or 0)
    classification_ok = bool(
        interp_acc is not None
        and extra_acc is not None
        and combined_acc is not None
        and float(interp_acc) >= 0.90
        and float(extra_acc) >= 0.90
        and float(combined_acc) >= 0.90
        and extra_count > 0
    )
    severity_ok = bool(severity.get("metrics", {}).get("pass"))
    prediction_ok = classification_ok and severity_ok

    if robust_clean_unsafe and contract_clean_gap and prediction_ok:
        verdict = "PASS"
        reason = "All revised decisive criteria are satisfied."
    else:
        verdict = "FAIL"
        missing = []
        if not robust_clean_unsafe:
            missing.append("no robust noise-confident clean_unsafe region")
        if not contract_clean_gap:
            missing.append("contract_violated or blocked point present, or clean_unsafe is not contract-clean")
        if not classification_ok:
            missing.append("noise-aware held-out classification is below 90% or lacks extrapolation")
        if not severity_ok:
            missing.append("severity regression MAE exceeds the preregistered noise-scale bound")
        reason = "; ".join(missing)

    return {
        "verdict": verdict,
        "premise_satisfied": True,
        "robust_clean_unsafe": robust_clean_unsafe,
        "contract_clean_gap": contract_clean_gap,
        "classification_ok": classification_ok,
        "severity_ok": severity_ok,
        "prediction_ok": prediction_ok,
        "clean_unsafe_count": len(clean_unsafe),
        "contract_violated_count": len(contract_violated),
        "blocked_count": len(blocked),
        "ambiguous_count": len([p for p in default_points if p.get("label") == "ambiguous"]),
        "interpolation_accuracy": interp_acc,
        "extrapolation_accuracy": extra_acc,
        "combined_heldout_accuracy": combined_acc,
        "severity_mae_m": severity.get("metrics", {}).get("mae_m"),
        "severity_mae_bound_m": severity.get("metrics", {}).get("mae_bound_m"),
        "reason": reason,
    }


def plot_result_field_v2(points: list[dict[str, Any]], oracle: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    for label, color in ZONE_COLORS.items():
        subset = [p for p in points if p.get("label") == label]
        if not subset:
            continue
        ax.scatter(
            [float(p["speed_m_s"]) for p in subset],
            [float(p["wind_m_s"]) for p in subset],
            s=520,
            marker="s",
            color=color,
            edgecolor="black",
            linewidth=0.8,
            label=label,
        )
    abbrev = {"clean_safe": "S", "clean_unsafe": "U", "ambiguous": "A", "contract_violated": "V", "blocked": "B"}
    for p in points:
        ax.text(
            float(p["speed_m_s"]),
            float(p["wind_m_s"]),
            abbrev.get(str(p.get("label")), "?"),
            ha="center",
            va="center",
            color="white" if p.get("label") in {"clean_unsafe", "contract_violated"} else "black",
            fontsize=10,
        )
    ax.set_xlabel("Command speed (m/s)")
    ax.set_ylabel("Outbound tailwind (m/s)")
    ax.set_title(f"Noise-aware field, d_margin={float(oracle['d_margin_m']):.2f} m")
    ax.grid(True, alpha=0.22)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_severity_heatmap_v2(points: list[dict[str, Any]], oracle: dict[str, Any], out_path: Path) -> str:
    pts = [p for p in points if p.get("severity_overshoot_m") is not None]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    if pts:
        xs = np.array([float(p["speed_m_s"]) for p in pts])
        ys = np.array([float(p["wind_m_s"]) for p in pts])
        zs = np.array([float(p.get("severity_overshoot_m") or 0.0) for p in pts])
        if len(pts) >= 4 and len(set(xs)) > 1 and len(set(ys)) > 1 and float(np.nanmax(zs)) > float(np.nanmin(zs)):
            levels = np.linspace(float(np.nanmin(zs)), float(np.nanmax(zs)), 18)
            cf = ax.tricontourf(xs, ys, zs, levels=levels, cmap="magma")
            fig.colorbar(cf, ax=ax, label="Mean fence overshoot (m)")
            low, high = oracle["ambiguous_band_overshoot_m"]
            contour_levels = sorted({max(float(np.nanmin(zs)), float(low)), float(oracle["d_margin_m"]), min(float(np.nanmax(zs)), float(high))})
            contour_levels = [v for v in contour_levels if float(np.nanmin(zs)) <= v <= float(np.nanmax(zs))]
            if contour_levels:
                cs = ax.tricontour(xs, ys, zs, levels=contour_levels, colors=["#00f5d4"], linewidths=1.5)
                ax.clabel(cs, inline=True, fontsize=8, fmt="%.1f m")
        else:
            sc = ax.scatter(xs, ys, c=zs, cmap="magma", s=95, edgecolor="black", linewidth=0.7)
            fig.colorbar(sc, ax=ax, label="Mean fence overshoot (m)")
        for p in pts:
            ax.text(float(p["speed_m_s"]), float(p["wind_m_s"]), f"{float(p.get('severity_overshoot_m') or 0.0):.1f}", ha="center", va="center", fontsize=7, color="white")
    ax.set_xlabel("Command speed (m/s)")
    ax.set_ylabel("Outbound tailwind (m/s)")
    ax.set_title("Excursion severity with noise-aware boundary band")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_train_test_v2(classification: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.0, 6.0))
    styles = {
        "interpolation": {"marker": "o", "color": "#2a9d8f", "label": "Interpolation"},
        "extrapolation": {"marker": "^", "color": "#e76f51", "label": "Extrapolation"},
    }
    for split, style in styles.items():
        rows = classification.get(split, {}).get("predictions", [])
        if not rows:
            continue
        ax.scatter(
            [float(r["probability_unsafe"]) for r in rows],
            [1.0 if r["observed_unsafe"] else 0.0 for r in rows],
            s=95,
            marker=style["marker"],
            color=style["color"],
            edgecolor="black",
            linewidth=0.7,
            label=style["label"],
        )
        for r in rows:
            ax.text(float(r["probability_unsafe"]), 1.0 if r["observed_unsafe"] else 0.0, f"{int(float(r['speed_m_s']))}/{int(float(r['wind_m_s']))}", fontsize=7, ha="left", va="bottom")
    ax.axvline(0.5, color="black", linestyle="--", linewidth=1.1, label="decision threshold")
    ax.set_xlim(-0.03, 1.03)
    ax.set_ylim(-0.18, 1.18)
    ax.set_yticks([0, 1], labels=["safe", "unsafe"])
    ax.set_xlabel("Predicted unsafe probability")
    ax.set_ylabel("Observed confident outcome")
    ax.set_title("Noise-aware held-out classification")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_p_stratification_v2(layers: dict[str, dict[str, Any]], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.5, 5.4), gridspec_kw={"width_ratios": [1.0, 1.35]})
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1].get("timeout_s", 0.0)))
    timeouts = [float(layer.get("timeout_s", 0.0)) for _, layer in ordered]
    unsafe_counts = [int(layer.get("zone_counts", {}).get("clean_unsafe", 0)) for _, layer in ordered]
    ambiguous_counts = [int(layer.get("zone_counts", {}).get("ambiguous", 0)) for _, layer in ordered]
    ax0.plot(timeouts, unsafe_counts, marker="o", color="#d62828", linewidth=2.0, label="clean_unsafe")
    ax0.plot(timeouts, ambiguous_counts, marker="s", color="#f4a261", linewidth=2.0, label="ambiguous")
    ax0.set_xlabel("FS_GCS_TIMEOUT (s)")
    ax0.set_ylabel("points")
    ax0.set_title("Region size by timeout")
    ax0.grid(True, alpha=0.25)
    ax0.legend(loc="best")
    markers = ["o", "s", "^", "D"]
    colors = ["#2a9d8f", "#4361ee", "#e76f51", "#f77f00"]
    for idx, (name, layer) in enumerate(ordered):
        pts = [p for p in layer.get("points", []) if p.get("label") == "clean_unsafe"]
        if not pts:
            continue
        ax1.scatter(
            [float(p["speed_m_s"]) for p in pts],
            [float(p["wind_m_s"]) for p in pts],
            s=110,
            marker=markers[idx % len(markers)],
            color=colors[idx % len(colors)],
            edgecolor="black",
            linewidth=0.7,
            label=f"{name}: {float(layer.get('timeout_s', 0.0)):.0f} s",
        )
    ax1.set_xlabel("Command speed (m/s)")
    ax1.set_ylabel("Outbound tailwind (m/s)")
    ax1.set_title("noise-aware clean_unsafe points by timeout")
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_severity_regression(severity: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = severity.get("predictions", [])
    fig, ax = plt.subplots(figsize=(7.2, 6.2))
    if rows:
        labels = [str(r.get("label")) for r in rows]
        colors = [ZONE_COLORS.get(label, "#4361ee") for label in labels]
        obs = [float(r["observed_overshoot_m"]) for r in rows]
        pred = [float(r["predicted_overshoot_m"]) for r in rows]
        ax.scatter(obs, pred, c=colors, s=95, edgecolor="black", linewidth=0.7)
        maxv = max(max(obs), max(pred), 1.0)
        ax.plot([0, maxv], [0, maxv], color="black", linestyle="--", linewidth=1.1)
        for r in rows:
            ax.text(float(r["observed_overshoot_m"]), float(r["predicted_overshoot_m"]), f"{int(float(r['speed_m_s']))}/{int(float(r['wind_m_s']))}", fontsize=7, ha="left", va="bottom")
    mae = severity.get("metrics", {}).get("mae_m")
    bound = severity.get("metrics", {}).get("mae_bound_m")
    ax.set_xlabel("Observed mean overshoot (m)")
    ax.set_ylabel("Predicted overshoot (m)")
    ax.set_title(f"Severity regression MAE={fmt(mae)} m, bound={fmt(bound)} m")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    plots = {
        "premise": plot_premise(payload["premise"], analysis / "linkloss_v2_premise.png"),
        "result_field": plot_result_field_v2(payload["default_grid"]["points"], payload["noise_floor_oracle"], analysis / "linkloss_v2_result_field.png"),
        "severity": plot_severity_heatmap_v2(payload["default_grid"]["points"], payload["noise_floor_oracle"], analysis / "linkloss_v2_severity_heatmap.png"),
        "p_stratification": plot_p_stratification_v2(payload["p_stratification"]["layers"], analysis / "linkloss_v2_p_stratification.png"),
        "train_test": plot_train_test_v2(payload["predictive_rule"]["classification"], analysis / "linkloss_v2_train_test.png"),
        "severity_regression": plot_severity_regression(payload["predictive_rule"]["severity_regression"], analysis / "linkloss_v2_severity_regression.png"),
    }
    return plots


def ambiguous_audit(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "point_key": p["point_key"],
            "speed_m_s": p["speed_m_s"],
            "wind_m_s": p["wind_m_s"],
            "mean_overshoot_m": p.get("mean_overshoot_m"),
            "label_ci_low_m": p.get("label_ci_low_m"),
            "label_ci_high_m": p.get("label_ci_high_m"),
            "d_margin_m": p.get("d_margin_m"),
            "basis": p.get("label_reason"),
        }
        for p in points
        if p.get("label") == "ambiguous"
    ]


def seed_reuse_summary(noise_runs: list[dict[str, Any]], grid_runs: list[dict[str, Any]]) -> dict[str, Any]:
    def summarize(runs: list[dict[str, Any]]) -> dict[str, Any]:
        seeded = [r for r in runs if r.get("v2_seed_source")]
        fresh = [r for r in runs if r.get("v2_campaign")]
        return {
            "total": len(runs),
            "v1_seeded_raw_sitl_runs": len(seeded),
            "v2_new_sitl_runs": len(fresh),
            "seed_run_ids_sample": [str(r.get("run_id")) for r in seeded[:12]],
            "new_run_ids_sample": [str(r.get("run_id")) for r in fresh[:12]],
        }

    return {
        "noise": summarize(noise_runs),
        "grid": summarize(grid_runs),
        "note": "Existing v1 raw SITL runs were reused as repetitions, then v2 ran missing repetitions to satisfy the configured N. The noise-aware thresholds were derived only from preregistered noise points and not from prediction correctness.",
    }


def write_report(payload: dict[str, Any]) -> str:
    report = PLANC_ROOT / "results" / "linkloss_excursion_v2_report.md"
    verdict = payload["verdict"]
    oracle = payload["noise_floor_oracle"]
    original = payload["original_oracle_comparison"]
    cfg = payload["config"]
    lines: list[str] = []
    lines.append(f"VERDICT: {verdict['verdict']}")
    lines.append("")
    lines.append("# planc GCS link-loss boundary-excursion second scenario, v2")
    lines.append("")
    lines.append("## Revised Four Criteria")
    lines.append("")
    lines.append(f"- Premise satisfied: **{verdict.get('premise_satisfied')}** (carried forward from v1, already true).")
    lines.append(f"- Robust contract-clean unsafe region under the noise-aware oracle: **{verdict.get('robust_clean_unsafe')}**; clean_unsafe={verdict.get('clean_unsafe_count')}, ambiguous={verdict.get('ambiguous_count')}.")
    lines.append(f"- Contract clean / PGFUZZ-invisible: **{verdict.get('contract_clean_gap')}**; contract_violated={verdict.get('contract_violated_count')}, blocked={verdict.get('blocked_count')}.")
    lines.append(
        f"- Prediction gates: **{verdict.get('prediction_ok')}**; classification_ok={verdict.get('classification_ok')} "
        f"(interpolation={fmt(verdict.get('interpolation_accuracy'), 3)}, extrapolation={fmt(verdict.get('extrapolation_accuracy'), 3)}, combined={fmt(verdict.get('combined_heldout_accuracy'), 3)}), "
        f"severity_ok={verdict.get('severity_ok')} (MAE={fmt(verdict.get('severity_mae_m'))} m <= bound={fmt(verdict.get('severity_mae_bound_m'))} m)."
    )
    lines.append("")
    lines.append(f"Decision reason: {verdict.get('reason')}")
    lines.append("")
    lines.append("## Measurement Precision And Oracle Commitment")
    lines.append("")
    lines.append(
        f"Noise floor was measured before grid aggregation. Boundary sigma={fmt(oracle['sigma_boundary_m'])} m; "
        f"unsafe-region sigma={fmt(oracle['sigma_unsafe_m'])} m. The committed margin is "
        f"`d_margin = k * sigma_boundary = {fmt(oracle['k_sigma_margin'], 1)} * {fmt(oracle['sigma_boundary_m'])} = {fmt(oracle['d_margin_m'])} m`, "
        f"so `hard_threshold = R + d_margin = {fmt(oracle['hard_threshold_distance_m'])} m`."
    )
    lines.append(
        f"Ambiguous label band uses mean overshoot +/- {fmt(oracle['ci_sigma_multiplier'], 1)}*sigma_boundary and spans "
        f"{fmt(oracle['ambiguous_band_overshoot_m'][0])} to {fmt(oracle['ambiguous_band_overshoot_m'][1])} m overshoot "
        f"(width {fmt(oracle['ambiguous_band_width_m'])} m). Severity MAE bound is "
        f"`c * sigma_boundary = {fmt(oracle['c_sigma_mae_bound'], 1)} * {fmt(oracle['sigma_boundary_m'])} = {fmt(oracle['mae_bound_m'])} m`."
    )
    lines.append("")
    lines.append(f"Oracle commitment file: `{rel(payload['artifacts']['oracle_commitment'])}`. It was written before the grid scheduler started.")
    lines.append("")
    seed = payload.get("seed_reuse", {})
    if seed:
        lines.append(
            "Seed reuse audit: v2 reused existing v1 raw SITL repetitions where available, then backfilled missing repetitions with new v2 runs. "
            f"Noise stage total={seed['noise']['total']} (v1 seed={seed['noise']['v1_seeded_raw_sitl_runs']}, v2 new={seed['noise']['v2_new_sitl_runs']}); "
            f"grid stage total={seed['grid']['total']} (v1 seed={seed['grid']['v1_seeded_raw_sitl_runs']}, v2 new={seed['grid']['v2_new_sitl_runs']}). "
            "Thresholds were derived only from the preregistered noise points and not from prediction correctness."
        )
        lines.append("")
    lines.append("| group | speed | wind | N | mean overshoot m | sample std m | contract clean |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | --- |")
    for group in ("boundary", "unsafe"):
        for point in oracle[group]["points"]:
            lines.append(
                f"| {group} | {fmt(point['speed_m_s'], 0)} | {fmt(point['wind_m_s'], 0)} | {point['completed_repetitions']} | "
                f"{fmt(point.get('mean_overshoot_m'))} | {fmt(point.get('sample_std_m'))} | {point.get('contract_clean_all')} |"
            )
    lines.append("")
    audit = payload["noise_floor_oracle"].get("ambiguous_audit", [])
    lines.append("Ambiguous exclusion audit; these points are excluded from classification only because their label CI crosses `d_margin`:")
    lines.append("")
    if audit:
        lines.append("| point | speed | wind | mean overshoot m | CI low | CI high | d_margin | basis |")
        lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |")
        for row in audit:
            lines.append(
                f"| {row['point_key']} | {fmt(row['speed_m_s'], 0)} | {fmt(row['wind_m_s'], 0)} | "
                f"{fmt(row.get('mean_overshoot_m'))} | {fmt(row.get('label_ci_low_m'))} | {fmt(row.get('label_ci_high_m'))} | "
                f"{fmt(row.get('d_margin_m'))} | {row.get('basis')} |"
            )
    else:
        lines.append("No ambiguous default-layer points.")
    lines.append("")
    lines.append("## Dual Oracle Comparison")
    lines.append("")
    lines.append(
        f"Original v1 oracle (`R + 1.5 m`) reported combined held-out accuracy **{fmt(original['v1_combined_heldout_accuracy'], 3)}** "
        f"with interpolation **{fmt(original['v1_interpolation_accuracy'], 3)}** and extrapolation **{fmt(original['v1_extrapolation_accuracy'], 3)}**."
    )
    cls = payload["predictive_rule"]["classification"]
    sev = payload["predictive_rule"]["severity_regression"]
    lines.append(
        f"Noise-aware v2 oracle reports classification interpolation **{fmt(cls['interpolation']['metrics'].get('classification_accuracy'), 3)}**, "
        f"extrapolation **{fmt(cls['extrapolation']['metrics'].get('classification_accuracy'), 3)}**, combined **{fmt(cls.get('combined_heldout_accuracy'), 3)}**, "
        f"and severity-regression MAE **{fmt(sev['metrics'].get('mae_m'))} m** over the whole held-out field including ambiguous/boundary points."
    )
    lines.append("")
    lines.append("## Three-Zone Field")
    lines.append("")
    counts = payload["default_grid"]["zone_counts"]
    lines.append(
        f"Default layer `{payload['default_grid']['layer']}` (`FS_GCS_TIMEOUT={fmt(payload['default_grid']['timeout_s'], 0)} s`) counts: "
        f"clean_safe={counts.get('clean_safe', 0)}, clean_unsafe={counts.get('clean_unsafe', 0)}, ambiguous={counts.get('ambiguous', 0)}, "
        f"contract_violated={counts.get('contract_violated', 0)}, blocked={counts.get('blocked', 0)}."
    )
    lines.append("")
    lines.append("| speed | wind | label | mean overshoot m | CI low | CI high | reps | timeout s | raw R+1.5 label |")
    lines.append("| ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |")
    for p in sorted(payload["default_grid"]["points"], key=lambda r: (float(r["speed_m_s"]), float(r["wind_m_s"]))):
        lines.append(
            f"| {fmt(p['speed_m_s'], 0)} | {fmt(p['wind_m_s'], 0)} | {p.get('label')} | "
            f"{fmt(p.get('mean_overshoot_m'))} | {fmt(p.get('label_ci_low_m'))} | {fmt(p.get('label_ci_high_m'))} | "
            f"{p.get('completed_repetitions')} | {fmt(p.get('gcs_failsafe_timeout_observed_s'))} | {p.get('raw_oracle_mean_label')} |"
        )
    lines.append("")
    lines.append(
        "PGFUZZ-invisible check: `clean_unsafe` requires the intended GCS failsafe ERR plus RTL at the configured timeout, "
        "successful parameter readback, commanded-speed audit within tolerance, and no unrelated failsafe/error events. "
        "Report-Only fence breach records are retained as measurement evidence and are not contract violations."
    )
    lines.append("")
    lines.append("## Two-Stage Prediction Evaluation")
    lines.append("")
    lines.append(f"Classification formula: `{cls['formula']}`. Ambiguous points are excluded only by label CI.")
    for split in ("interpolation", "extrapolation"):
        metrics = cls[split]["metrics"]
        lines.append(
            f"{split}: confident n={metrics.get('count')}, ambiguous excluded={metrics.get('ambiguous_excluded_count')}, "
            f"accuracy={fmt(metrics.get('classification_accuracy'), 3)}, model_ok={cls[split]['model'].get('ok')}."
        )
    lines.append(f"Combined confident held-out classification accuracy: {fmt(cls.get('combined_heldout_accuracy'), 3)}.")
    lines.append("")
    lines.append(f"Severity formula: `{sev['formula']}`.")
    lines.append(
        f"Severity regression: train n={sev.get('train_point_count')}, held-out n={sev.get('holdout_point_count')}, "
        f"contains ambiguous={sev.get('holdout_points_include_ambiguous')}, MAE={fmt(sev['metrics'].get('mae_m'))} m, "
        f"bound={fmt(sev['metrics'].get('mae_bound_m'))} m, pass={sev['metrics'].get('pass')}."
    )
    lines.append("")
    lines.append("## P Stratification")
    lines.append("")
    p_summary = payload["p_stratification"]["summary"]
    lines.append(f"Conclusion: {p_summary['conclusion']}.")
    lines.append("")
    lines.append("| layer | FS_GCS_TIMEOUT s | clean_unsafe | clean_safe | ambiguous | contract_violated |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: |")
    for row in p_summary["counts"]:
        lines.append(
            f"| {row['layer']} | {fmt(row['timeout_s'], 0)} | {row['clean_unsafe']} | {row['clean_safe']} | "
            f"{row['ambiguous']} | {row['contract_violated']} |"
        )
    lines.append("")
    lines.append("## Search Efficiency")
    lines.append("")
    search = payload["search_efficiency"]
    lines.append(f"{search['strategy']}. Queries to bracket boundaries: {search['query_count']} vs full grid {search['full_grid_count']}.")
    lines.append("")
    lines.append("## Unified Method Statement")
    lines.append("")
    lines.append(
        "This scenario and the RTL energy scenario are both threshold-insufficiency specification gaps: one is a data-link time-budget threshold, "
        "the other an energy-budget threshold. In both, ArduCopter follows the configured failsafe contract while a legal operating condition crosses an external safety oracle."
    )
    lines.append("")
    lines.append("## Figures")
    lines.append("")
    for name, path in payload.get("artifacts", {}).get("plots", {}).items():
        lines.append(f"- {name}: ![]({rel(path)})")
    lines.append("")
    lines.append("## Limitations")
    lines.append("")
    lines.append(
        "This remains ArduCopter SITL, not HITL. The noise-aware oracle is justified by the measured SITL run-to-run sigma, "
        "not by post-hoc tuning against prediction mistakes. The original strict-gate result for scenario (b) remains the main result; "
        "this v2 rerun only repairs the GCS link-loss measurement precision mismatch."
    )
    lines.append("")
    lines.append(
        "Audit files are under `planc/logs/` for each run id, with parsed CSV and `.oracle.json` sidecars. "
        f"Structured results: `{rel(payload['artifacts']['results_json'])}`."
    )
    lines.append("")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(lines), encoding="utf-8")
    return str(report)


def build_payload(
    config: dict[str, Any],
    env: dict[str, Any],
    v1: dict[str, Any],
    noise_runs: list[dict[str, Any]],
    grid_runs: list[dict[str, Any]],
    oracle: dict[str, Any],
    oracle_commitment_path: Path,
    final_path: Path,
) -> dict[str, Any]:
    speeds = [float(v) for v in config["sweep"]["speeds_m_s"]]
    winds = [float(v) for v in config["sweep"]["winds_m_s"]]
    default_layer = str(config["sweep"]["default_layer"])
    raw_tolerance = float(config["experiment"]["fence_tolerance_m"])
    default_reps = int(config["noise_floor"]["grid_repetitions"])
    p_reps = int(config["noise_floor"]["p_layer_repetitions"])
    layers: dict[str, dict[str, Any]] = {}
    for layer_name, layer_cfg in config["sweep"]["p_layers"].items():
        reps = default_reps if str(layer_name) == default_layer else p_reps
        points = aggregate_layer_v2(
            grid_runs,
            layer=str(layer_name),
            timeout_s=float(layer_cfg["FS_GCS_TIMEOUT"]),
            speeds=speeds,
            winds=winds,
            required_repetitions=reps,
            oracle=oracle,
            raw_tolerance_m=raw_tolerance,
        )
        layers[str(layer_name)] = {
            "layer": str(layer_name),
            "timeout_s": float(layer_cfg["FS_GCS_TIMEOUT"]),
            "required_repetitions": reps,
            "points": points,
            "zone_counts": zone_counts(points),
        }
    default_points = layers[default_layer]["points"]
    oracle = dict(oracle)
    oracle["ambiguous_audit"] = ambiguous_audit(default_points)
    premise = v1.get("premise", {})
    classification = train_test_classification(default_points, config, oracle) if premise.get("satisfied") else {}
    severity = evaluate_severity_regression(default_points, config, oracle) if premise.get("satisfied") else {}
    verdict = verdict_summary(premise, default_points, classification, severity)
    original_verdict = v1.get("verdict", {})
    all_points = [p for layer in layers.values() for p in layer["points"]]
    payload = {
        "status": "COMPLETE" if not any(p.get("label") == "blocked" for p in all_points) else "INCOMPLETE",
        "env": env,
        "config": config,
        "derived": {"trigger_distance_m": trigger_distance_m(config)},
        "v1_source": {
            "results_json": str(PLANC_ROOT / "results" / "linkloss_excursion_results.json"),
            "tag": "planc/linkloss-v1-20260611",
            "verdict": original_verdict.get("verdict"),
        },
        "premise": premise,
        "noise_floor_oracle": oracle,
        "seed_reuse": seed_reuse_summary(noise_runs, grid_runs),
        "default_grid": {
            "layer": default_layer,
            "timeout_s": float(config["sweep"]["p_layers"][default_layer]["FS_GCS_TIMEOUT"]),
            "required_repetitions": default_reps,
            "speeds_m_s": speeds,
            "winds_m_s": winds,
            "points": default_points,
            "zone_counts": zone_counts(default_points),
        },
        "p_stratification": {
            "layers": layers,
            "summary": p_stratification_summary(layers),
        },
        "predictive_rule": {
            "classification": classification,
            "severity_regression": severity,
        },
        "original_oracle_comparison": {
            "oracle": "v1 R + 1.5 m",
            "v1_verdict": original_verdict.get("verdict"),
            "v1_combined_heldout_accuracy": original_verdict.get("combined_heldout_accuracy"),
            "v1_interpolation_accuracy": original_verdict.get("interpolation_accuracy"),
            "v1_extrapolation_accuracy": original_verdict.get("extrapolation_accuracy"),
        },
        "search_efficiency": boundary_search_summary(default_points, config),
        "verdict": verdict,
        "runs": {
            "noise": noise_runs,
            "grid": grid_runs,
        },
        "artifacts": {
            "oracle_commitment": str(oracle_commitment_path),
            "results_json": str(final_path),
        },
    }
    payload["artifacts"]["plots"] = make_plots(payload)
    payload["artifacts"]["report"] = write_report(payload)
    return payload


def ensure_v1_premise(v1_path: Path) -> dict[str, Any]:
    v1 = load_json(v1_path)
    if not v1.get("premise", {}).get("satisfied"):
        # Fall back to recomputing if an older result shape is ever present.
        premise_runs = list(v1.get("runs", {}).get("premise", []))
        v1["premise"] = premise_summary(premise_runs)
    return v1


def run_noise_stage(
    config: dict[str, Any],
    runs: list[dict[str, Any]],
    partial_path: Path,
    max_new_runs_state: dict[str, int | None],
) -> bool:
    timeout_s = float(config["noise_floor"]["default_timeout_s"])
    reps = int(config["noise_floor"]["noise_repetitions"])
    for group_name, cfg_name in (("boundary", "boundary_points"), ("unsafe", "unsafe_points")):
        layer = f"noise_{group_name}"
        for spec in config["noise_floor"][cfg_name]:
            speed = float(spec["speed_m_s"])
            wind = float(spec["wind_m_s"])
            for rep in range(1, reps + 1):
                if len(completed_grid_runs(runs, layer, speed, wind)) >= reps:
                    break
                ok = maybe_schedule_run(
                    config,
                    runs,
                    partial_path,
                    run_id=v2_run_id(layer, speed, wind, rep),
                    run_kind="grid",
                    layer=layer,
                    speed_m_s=speed,
                    wind_m_s=wind,
                    timeout_s=timeout_s,
                    rep_index=rep,
                    roles=["noise_floor", f"noise_{group_name}"],
                    max_new_runs_state=max_new_runs_state,
                )
                if not ok:
                    return False
    return True


def run_grid_stage(
    config: dict[str, Any],
    runs: list[dict[str, Any]],
    partial_path: Path,
    max_new_runs_state: dict[str, int | None],
) -> bool:
    speeds = [float(v) for v in config["sweep"]["speeds_m_s"]]
    winds = [float(v) for v in config["sweep"]["winds_m_s"]]
    default_layer = str(config["sweep"]["default_layer"])
    default_reps = int(config["noise_floor"]["grid_repetitions"])
    p_reps = int(config["noise_floor"]["p_layer_repetitions"])
    for layer_name, layer_cfg in config["sweep"]["p_layers"].items():
        layer = str(layer_name)
        reps = default_reps if layer == default_layer else p_reps
        timeout_s = float(layer_cfg["FS_GCS_TIMEOUT"])
        for speed in speeds:
            for wind in winds:
                for rep in range(1, reps + 1):
                    if len(completed_grid_runs(runs, layer, speed, wind)) >= reps:
                        break
                    ok = maybe_schedule_run(
                        config,
                        runs,
                        partial_path,
                        run_id=v2_run_id(layer, speed, wind, rep),
                        run_kind="grid",
                        layer=layer,
                        speed_m_s=speed,
                        wind_m_s=wind,
                        timeout_s=timeout_s,
                        rep_index=rep,
                        roles=["default_grid" if layer == default_layer else "p_layer", "noise_aware_grid"],
                        max_new_runs_state=max_new_runs_state,
                    )
                    if not ok:
                        return False
    return True


def all_noise_runs_present(config: dict[str, Any], noise_runs: list[dict[str, Any]]) -> bool:
    reps = int(config["noise_floor"]["noise_repetitions"])
    for group_name, cfg_name in (("boundary", "boundary_points"), ("unsafe", "unsafe_points")):
        layer = f"noise_{group_name}"
        for spec in config["noise_floor"][cfg_name]:
            complete = completed_grid_runs(noise_runs, layer, float(spec["speed_m_s"]), float(spec["wind_m_s"]))
            if len(complete) < reps:
                return False
    return True


def write_oracle_commitment(path: Path, oracle: dict[str, Any]) -> None:
    payload = dict(oracle)
    payload["committed_at_utc"] = datetime.now(timezone.utc).isoformat()
    payload["commitment"] = "Grid labels must use this sigma_boundary-derived d_margin; ambiguous exclusions are based only on label CI crossing d_margin."
    write_json(path, payload)


def main() -> int:
    parser = argparse.ArgumentParser(description="Noise-floor-aware rerun of the planc GCS link-loss boundary-excursion scenario.")
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "linkloss_excursion_v2_config.yaml")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--seed-v1-grid", action="store_true", help="reuse existing v1 grid SITL runs as raw repetitions before backfilling")
    parser.add_argument("--seed-v1-noise", action="store_true", help="reuse matching v1 default-layer runs as noise-floor repetitions before backfilling")
    parser.add_argument("--max-new-runs", type=int, default=None, help="testing/debug guard; omit for a complete rerun")
    args = parser.parse_args()

    config = load_config(args.config)
    results_dir = PLANC_ROOT / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    noise_partial_path = results_dir / "linkloss_excursion_v2_noise_partial.json"
    grid_partial_path = results_dir / "linkloss_excursion_v2_grid_partial.json"
    oracle_commitment_path = results_dir / "linkloss_excursion_v2_oracle_preregistered.json"
    final_path = results_dir / "linkloss_excursion_v2_results.json"
    v1_path = results_dir / "linkloss_excursion_results.json"
    max_new_runs_state: dict[str, int | None] = {"remaining": args.max_new_runs}

    env = probe_environment(config, REPO_ROOT)
    write_env(env, results_dir / "env_linkloss_excursion_v2.json")
    v1 = ensure_v1_premise(v1_path)
    if not v1.get("premise", {}).get("satisfied"):
        payload = {
            "status": "INCONCLUSIVE",
            "reason": "v1 premise is not satisfied",
            "env": env,
            "verdict": {"verdict": "INCONCLUSIVE", "premise_satisfied": False},
        }
        write_json(final_path, payload)
        print(f"INCONCLUSIVE: v1 premise failed; results={final_path}", flush=True)
        return 0

    noise_runs = load_runs(noise_partial_path, args.resume)
    grid_runs = load_runs(grid_partial_path, args.resume)
    if args.seed_v1_noise:
        added = seed_v1_noise_runs(v1, config, noise_runs, noise_partial_path)
        if added:
            print(f"SEEDED noise_runs_from_v1={added}", flush=True)
    if args.seed_v1_grid:
        added = seed_v1_grid_runs(v1, grid_runs, grid_partial_path)
        if added:
            print(f"SEEDED grid_runs_from_v1={added}", flush=True)

    start = time.time()
    if not run_noise_stage(config, noise_runs, noise_partial_path, max_new_runs_state):
        print(f"PAUSED: max-new-runs reached during noise stage; partial={noise_partial_path}", flush=True)
        return 0
    if not all_noise_runs_present(config, noise_runs):
        print(f"INCOMPLETE: noise stage missing runs; partial={noise_partial_path}", flush=True)
        return 1

    if oracle_commitment_path.exists() and grid_runs:
        oracle = load_json(oracle_commitment_path)
        print(
            f"ORACLE_REUSED sigma_boundary={oracle['sigma_boundary_m']:.3f} "
            f"d_margin={oracle['d_margin_m']:.3f} path={oracle_commitment_path}",
            flush=True,
        )
    else:
        oracle = summarize_noise_runs(noise_runs, config)
        write_oracle_commitment(oracle_commitment_path, oracle)
        print(
            f"ORACLE_COMMITTED sigma_boundary={oracle['sigma_boundary_m']:.3f} "
            f"d_margin={oracle['d_margin_m']:.3f} path={oracle_commitment_path}",
            flush=True,
        )

    if not run_grid_stage(config, grid_runs, grid_partial_path, max_new_runs_state):
        payload = build_payload(config, env, v1, noise_runs, grid_runs, oracle, oracle_commitment_path, final_path)
        write_json(final_path, payload)
        print(f"PAUSED: max-new-runs reached during grid stage; partial={grid_partial_path} results={final_path}", flush=True)
        return 0

    payload = build_payload(config, env, v1, noise_runs, grid_runs, oracle, oracle_commitment_path, final_path)
    write_json(final_path, payload)
    elapsed = time.time() - start
    print(
        f"COMPLETE: verdict={payload['verdict']['verdict']} elapsed_s={elapsed:.1f} "
        f"report={payload['artifacts']['report']} results={final_path}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
