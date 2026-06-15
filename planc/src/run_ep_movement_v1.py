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
from matplotlib.lines import Line2D
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
from oracle import COPTER_MODES
from param_manager import ParamManager
from run_oracleA_v1 import disarm_for_cleanup, parse_dataflash_hardened, run_one as run_oracleA_one, wait_disarmed
from run_stage0_v2 import (
    command_amplitude_deg,
    command_at,
    doublet_profile,
    fmt,
    point_config,
    point_params,
    run_id_for,
    send_attitude_target,
)
from sitl_runner import SitlRunner


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
    return str(config["experiment"].get("artifact_stem", "ep_movement_v1"))


def mean_dt(times: list[float], fallback: float = 0.02) -> float:
    diffs = [b - a for a, b in zip(times, times[1:]) if b > a]
    return statistics.fmean(diffs) if diffs else fallback


def frow(row: dict[str, Any], name: str, default: float = 0.0) -> float:
    try:
        raw = row.get(name, "")
        return default if raw == "" else float(raw)
    except Exception:
        return default


def sanitize_id(value: str) -> str:
    return "".join(ch for ch in value if ch.isalnum()).lower()


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


def auc(values: list[float], labels: list[int]) -> float | None:
    pairs = [(float(v), int(y)) for v, y in zip(values, labels) if v is not None and math.isfinite(float(v))]
    positives = [v for v, y in pairs if y == 1]
    negatives = [v for v, y in pairs if y == 0]
    if not positives or not negatives:
        return None
    score = 0.0
    for pos in positives:
        for neg in negatives:
            if pos > neg:
                score += 1.0
            elif pos == neg:
                score += 0.5
    return score / (len(positives) * len(negatives))


def threshold_scan(values: list[float], labels: list[int]) -> dict[str, Any]:
    pairs = [(float(v), int(y)) for v, y in zip(values, labels) if v is not None and math.isfinite(float(v))]
    if not pairs:
        return {"threshold": None}
    best: dict[str, Any] | None = None
    for threshold in sorted({v for v, _ in pairs}):
        tp = fp = tn = fn = 0
        for value, label in pairs:
            pred = value >= threshold
            if pred and label == 1:
                tp += 1
            elif pred and label == 0:
                fp += 1
            elif not pred and label == 0:
                tn += 1
            else:
                fn += 1
        tpr = tp / (tp + fn) if tp + fn else 0.0
        fpr = fp / (fp + tn) if fp + tn else 0.0
        row = {
            "threshold": threshold,
            "direction": ">=",
            "tp": tp,
            "fp": fp,
            "tn": tn,
            "fn": fn,
            "tpr": tpr,
            "fpr": fpr,
            "balanced_accuracy": 0.5 * (tpr + (1.0 - fpr)),
            "youden_j": tpr - fpr,
        }
        if best is None or (row["youden_j"], row["balanced_accuracy"]) > (
            best["youden_j"],
            best["balanced_accuracy"],
        ):
            best = row
    return best or {"threshold": None}


def read_csv_by_type(path: Path) -> dict[str, list[dict[str, Any]]]:
    by_type: dict[str, list[dict[str, Any]]] = defaultdict(list)
    with path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                row["_t"] = float(row["time_s"])
            except Exception:
                continue
            by_type[str(row.get("type", ""))].append(row)
    return by_type


def nearest_value(rows: list[tuple[float, float]], t_s: float | None) -> float | None:
    if not rows or t_s is None:
        return None
    return min(rows, key=lambda item: abs(item[0] - t_s))[1]


def first_consequence_time(run: dict[str, Any], rows: dict[str, list[dict[str, Any]]], config: dict[str, Any]) -> float | None:
    active = run.get("active_window_s", {})
    start = active.get("start")
    oracle_end = active.get("oracle_end")
    maneuver_end = active.get("maneuver_end")
    if start is None or oracle_end is None:
        return maneuver_end

    hard = run.get("hardened_oracle_A", {})
    candidates: list[float] = []
    for marker in hard.get("ground_messages", []):
        if marker.get("time_s") is not None:
            candidates.append(float(marker["time_s"]))
    for marker in hard.get("low_floor_markers", []):
        if marker.get("time_s") is not None:
            candidates.append(float(marker["time_s"]))

    causes = hard.get("causes", {})
    if causes.get("unrecovered_divergence") or causes.get("worsening_divergence"):
        candidates.append(float(oracle_end))

    pos_rows = [
        row
        for row in rows.get("POS", [])
        if float(start) <= float(row["_t"]) <= float(oracle_end) and row.get("RelHomeAlt") not in (None, "")
    ]
    if pos_rows:
        start_alt = nearest_value([(float(r["_t"]), frow(r, "RelHomeAlt")) for r in pos_rows], float(start))
        if start_alt is not None:
            loss_thresh = float(config["oracle"]["altitude_loss_thresh_m"])
            for row in pos_rows:
                loss = start_alt - frow(row, "RelHomeAlt")
                if loss > loss_thresh:
                    candidates.append(float(row["_t"]))
                    break

    if not candidates:
        return maneuver_end
    return min(candidates)


def run_feature_row(
    run: dict[str, Any],
    config: dict[str, Any],
    *,
    high_bank_threshold_deg: float,
    cached_rows: dict[str, list[dict[str, Any]]] | None = None,
) -> dict[str, Any]:
    active = run.get("active_window_s", {})
    start = active.get("start")
    maneuver_end = active.get("maneuver_end")
    oracle_end = active.get("oracle_end")
    rows = cached_rows if cached_rows is not None else read_csv_by_type(Path(str(run["csv_path"])))
    feature_end = None
    if start is not None and maneuver_end is not None:
        consequence = first_consequence_time(run, rows, config)
        feature_end = min(float(maneuver_end), float(consequence if consequence is not None else maneuver_end))

    rate_rows = [
        row
        for row in rows.get("RATE", [])
        if start is not None and feature_end is not None and float(start) <= float(row["_t"]) <= float(feature_end)
    ]
    rate_times = [float(row["_t"]) for row in rate_rows]
    dt_rate = mean_dt(rate_times)
    actual_rate = [math.hypot(frow(row, "R"), frow(row, "P")) for row in rate_rows]

    att_rows = [
        row
        for row in rows.get("ATT", [])
        if start is not None and feature_end is not None and float(start) <= float(row["_t"]) <= float(feature_end)
    ]
    att_times = [float(row["_t"]) for row in att_rows]
    dt_att = mean_dt(att_times)
    tilts = [math.hypot(frow(row, "Roll"), frow(row, "Pitch")) for row in att_rows]
    high_bank_impulse = sum(max(0.0, tilt - high_bank_threshold_deg) for tilt in tilts) * dt_att if tilts else None
    high_bank_time = sum(1 for tilt in tilts if tilt >= high_bank_threshold_deg) * dt_att if tilts else None

    hard = run.get("hardened_oracle_A", {})
    alt = hard.get("altitude", {})
    point = run.get("point", {})
    return {
        "run_id": run.get("run_id"),
        "label": run.get("label"),
        "clean_unsafe": 1 if run.get("label") == config["mechanisms"]["clean_label"] else 0,
        "r_deg_s": point.get("r_deg_s"),
        "seed": point.get("seed"),
        "E_layer": point.get("E_layer", point.get("e_layer", "")),
        "P_layer": point.get("P_layer", point.get("p_layer", "")),
        "layer": point.get("layer", ""),
        "wind_m_s": point.get("wind_m_s"),
        "turbulence_m_s": point.get("turbulence_m_s"),
        "angle_max_cd": point.get("angle_max_cd"),
        "actual_rate_peak_deg_s": max(actual_rate, default=None),
        "actual_rate_rms_deg_s": math.sqrt(sum(v * v for v in actual_rate) / len(actual_rate)) if actual_rate else None,
        "actual_rate_impulse_deg": sum(actual_rate) * dt_rate if actual_rate else None,
        "high_bank_threshold_deg": high_bank_threshold_deg,
        "high_bank_dwell_impulse_deg_s": high_bank_impulse,
        "high_bank_dwell_time_s": high_bank_time,
        "feature_window_start_s": start,
        "feature_window_end_s": feature_end,
        "feature_window_duration_s": None if start is None or feature_end is None else feature_end - float(start),
        "hardened_A_inside": bool(hard.get("inside")),
        "hardened_A_outcome": hard.get("outcome"),
        "hardened_A_causes": hard.get("causes", {}),
        "altitude_loss_m": alt.get("loss_m"),
        "max_attitude_error_deg": hard.get("max_attitude_error_deg"),
        "final_error_deg": hard.get("final_error_deg"),
    }


def zscore_score(row: dict[str, Any], classifier: dict[str, Any]) -> float | None:
    rate = row.get("actual_rate_peak_deg_s")
    bank = row.get("high_bank_dwell_impulse_deg_s")
    if rate is None or bank is None:
        return None
    mean_rate, mean_bank = classifier["mean"]
    std_rate, std_bank = classifier["std"]
    std_rate = std_rate if std_rate > 0.0 else 1.0
    std_bank = std_bank if std_bank > 0.0 else 1.0
    return (float(rate) - mean_rate) / std_rate + (float(bank) - mean_bank) / std_bank


def build_classifier(feature_rows: list[dict[str, Any]]) -> dict[str, Any]:
    rows = [
        row
        for row in feature_rows
        if row.get("actual_rate_peak_deg_s") is not None and row.get("high_bank_dwell_impulse_deg_s") is not None
    ]
    labels = [int(row["clean_unsafe"]) for row in rows]
    rate_values = [float(row["actual_rate_peak_deg_s"]) for row in rows]
    bank_values = [float(row["high_bank_dwell_impulse_deg_s"]) for row in rows]
    mean = [statistics.fmean(rate_values), statistics.fmean(bank_values)]
    std = [
        statistics.pstdev(rate_values) if len(rate_values) > 1 else 1.0,
        statistics.pstdev(bank_values) if len(bank_values) > 1 else 1.0,
    ]
    classifier = {
        "type": "monotone_zscore_sum",
        "features": ["actual_rate_peak_deg_s", "high_bank_dwell_impulse_deg_s"],
        "mean": mean,
        "std": std,
    }
    scores = [zscore_score(row, classifier) for row in rows]
    clean_scores = [float(v) for v in scores if v is not None]
    classifier["auc"] = auc(clean_scores, labels)
    classifier["threshold"] = threshold_scan(clean_scores, labels)
    return classifier


def label_mechanisms(feature_rows: list[dict[str, Any]], classifier: dict[str, Any]) -> dict[str, Any]:
    labels = [int(row["clean_unsafe"]) for row in feature_rows]
    rate_values = [float(row["actual_rate_peak_deg_s"]) for row in feature_rows]
    bank_values = [float(row["high_bank_dwell_impulse_deg_s"]) for row in feature_rows]
    rate_threshold = threshold_scan(rate_values, labels)
    bank_threshold = threshold_scan(bank_values, labels)
    rate_thr = float(rate_threshold["threshold"])
    bank_thr = float(bank_threshold["threshold"])
    mean_rate, mean_bank = classifier["mean"]
    std_rate, std_bank = classifier["std"]
    std_rate = std_rate if std_rate > 0.0 else 1.0
    std_bank = std_bank if std_bank > 0.0 else 1.0

    resolved_by_dominance = 0
    for row in feature_rows:
        if row.get("clean_unsafe") != 1:
            row["mechanism"] = "safe_or_recovered"
            continue
        rate = float(row["actual_rate_peak_deg_s"])
        bank = float(row["high_bank_dwell_impulse_deg_s"])
        tumble = rate >= rate_thr
        altitude_bleed = bank >= bank_thr and bool(row.get("hardened_A_causes", {}).get("altitude_loss"))
        if tumble and altitude_bleed:
            mechanism = "both"
        elif tumble:
            mechanism = "tumble"
        elif altitude_bleed:
            mechanism = "altitude_bleed"
        else:
            resolved_by_dominance += 1
            z_rate = (rate - mean_rate) / std_rate
            z_bank = (bank - mean_bank) / std_bank
            mechanism = "tumble" if z_rate >= z_bank else "altitude_bleed"
        row["mechanism"] = mechanism
    return {
        "rate_threshold": rate_threshold,
        "high_bank_threshold": bank_threshold,
        "resolved_by_dominance": resolved_by_dominance,
    }


def make_mechanism_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    rows = payload["features"]
    classifier = payload["best_2d_classifier"]
    high_bank_angle = float(classifier["high_bank_threshold_deg"])
    paths: dict[str, str] = {}

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True, sharey=True)
    label_colors = {"clean_unsafe": "#d1495b", "recovered": "#edae49", "safe": "#2a9d8f"}
    mechanism_colors = {
        "tumble": "#7b2cbf",
        "altitude_bleed": "#f77f00",
        "both": "#d62828",
        "safe_or_recovered": "#8d99ae",
    }
    for row in rows:
        axes[0].scatter(
            float(row["actual_rate_peak_deg_s"]),
            float(row["high_bank_dwell_impulse_deg_s"]),
            color=label_colors.get(str(row["label"]), "#777777"),
            s=38,
            alpha=0.82,
            edgecolor="none",
        )
        axes[1].scatter(
            float(row["actual_rate_peak_deg_s"]),
            float(row["high_bank_dwell_impulse_deg_s"]),
            color=mechanism_colors.get(str(row["mechanism"]), "#777777"),
            s=38,
            alpha=0.82,
            edgecolor="none",
        )

    rate_thr = payload["mechanism_thresholds"]["rate_threshold"]["threshold"]
    bank_thr = payload["mechanism_thresholds"]["high_bank_threshold"]["threshold"]
    for ax in axes:
        ax.axvline(float(rate_thr), color="#222222", linestyle="--", linewidth=1)
        ax.axhline(float(bank_thr), color="#222222", linestyle=":", linewidth=1)
        score_thr = classifier["threshold"]["threshold"]
        mean_rate, mean_bank = classifier["mean"]
        std_rate, std_bank = classifier["std"]
        xs = [
            min(float(row["actual_rate_peak_deg_s"]) for row in rows),
            max(float(row["actual_rate_peak_deg_s"]) for row in rows),
        ]
        ys = [mean_bank + std_bank * (float(score_thr) - (x - mean_rate) / std_rate) for x in xs]
        ax.plot(xs, ys, color="#003049", linewidth=1.5)
        ax.grid(True, alpha=0.22)
        ax.set_xlabel("achieved rate peak before consequence (deg/s)")
    axes[0].set_ylabel(f"high-bank dwell impulse above {high_bank_angle:.0f} deg (deg*s)")
    axes[0].set_title("Clean unsafe labels in 2D feature space")
    axes[1].set_title("Mechanism labels")
    axes[0].legend(
        handles=[
            Line2D([0], [0], marker="o", color="w", label="clean_unsafe", markerfacecolor=label_colors["clean_unsafe"], markersize=7),
            Line2D([0], [0], marker="o", color="w", label="recovered", markerfacecolor=label_colors["recovered"], markersize=7),
            Line2D([0], [0], marker="o", color="w", label="safe", markerfacecolor=label_colors["safe"], markersize=7),
            Line2D([0], [0], color="#003049", label="2D boundary"),
        ],
        loc="upper right",
        fontsize=8,
        frameon=True,
    )
    axes[1].legend(
        handles=[
            Line2D([0], [0], marker="o", color="w", label="tumble", markerfacecolor=mechanism_colors["tumble"], markersize=7),
            Line2D([0], [0], marker="o", color="w", label="altitude_bleed", markerfacecolor=mechanism_colors["altitude_bleed"], markersize=7),
            Line2D([0], [0], marker="o", color="w", label="both", markerfacecolor=mechanism_colors["both"], markersize=7),
            Line2D([0], [0], marker="o", color="w", label="safe/recovered", markerfacecolor=mechanism_colors["safe_or_recovered"], markersize=7),
        ],
        loc="upper right",
        fontsize=8,
        frameon=True,
    )
    path = analysis / "ep_movement_mechanisms_2d_boundary.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    paths["mechanisms_2d_boundary"] = str(path)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    counts = payload["mechanism_counts"]
    names = ["tumble", "altitude_bleed", "both"]
    ax.bar(names, [counts.get(name, 0) for name in names], color=[mechanism_colors[name] for name in names])
    ax.set_ylabel("hard-A clean unsafe runs")
    ax.set_title("Separated hard-A mechanisms on oracleA v1 runs")
    ax.grid(True, axis="y", alpha=0.22)
    path = analysis / "ep_movement_mechanism_counts.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    paths["mechanism_counts"] = str(path)
    return paths


def build_mechanisms(config: dict[str, Any]) -> dict[str, Any]:
    oracle = load_json(REPO_ROOT / config["experiment"]["source_oracleA_result"])
    source_runs = [
        run
        for run in oracle["runs"]
        if not run.get("error") and run.get("label") in [config["mechanisms"]["clean_label"], *config["mechanisms"]["negative_labels"]]
    ]
    candidates = [float(v) for v in config["mechanisms"]["high_bank_threshold_candidates_deg"]]
    candidate_payloads: list[dict[str, Any]] = []
    rows_cache: dict[str, dict[str, list[dict[str, Any]]]] = {}
    for run in source_runs:
        rows_cache[str(run["run_id"])] = read_csv_by_type(Path(str(run["csv_path"])))
    for threshold in candidates:
        rows = [
            run_feature_row(run, config, high_bank_threshold_deg=threshold, cached_rows=rows_cache[str(run["run_id"])])
            for run in source_runs
        ]
        classifier = build_classifier(rows)
        candidate_payloads.append(
            {
                "high_bank_threshold_deg": threshold,
                "classifier": classifier,
                "single_feature_auc": {
                    "actual_rate_peak_deg_s": auc(
                        [float(row["actual_rate_peak_deg_s"]) for row in rows],
                        [int(row["clean_unsafe"]) for row in rows],
                    ),
                    "high_bank_dwell_impulse_deg_s": auc(
                        [float(row["high_bank_dwell_impulse_deg_s"]) for row in rows],
                        [int(row["clean_unsafe"]) for row in rows],
                    ),
                },
            }
        )
    best_candidate = max(candidate_payloads, key=lambda row: float(row["classifier"].get("auc") or 0.0))
    selected_threshold = float(best_candidate["high_bank_threshold_deg"])
    features = [
        run_feature_row(run, config, high_bank_threshold_deg=selected_threshold, cached_rows=rows_cache[str(run["run_id"])])
        for run in source_runs
    ]
    classifier = build_classifier(features)
    classifier["high_bank_threshold_deg"] = selected_threshold
    mechanism_thresholds = label_mechanisms(features, classifier)
    scores = []
    for row in features:
        score = zscore_score(row, classifier)
        row["mechanism_score"] = score
        row["mechanism_classifier_predicted_unsafe"] = (
            score is not None and score >= float(classifier["threshold"]["threshold"])
        )
        scores.append(float(score) if score is not None else float("nan"))

    hard_features = [row for row in features if int(row["clean_unsafe"]) == 1]
    mechanism_counts = Counter(row["mechanism"] for row in hard_features)
    result = {
        "status": "complete",
        "generated_at_utc": utc_now(),
        "source": {
            "oracleA_result": config["experiment"]["source_oracleA_result"],
            "oracleA_tag": config["experiment"]["source_oracleA_tag"],
            "source_runs": len(source_runs),
        },
        "feature_window": config["mechanisms"]["feature_window"],
        "features": features,
        "candidate_2d_classifiers": candidate_payloads,
        "best_2d_feature_pair": [
            "actual_rate_peak_deg_s",
            f"high_bank_dwell_impulse_gt_{int(selected_threshold)}deg_deg_s",
        ],
        "best_2d_classifier": classifier,
        "best_2d_auc": classifier["auc"],
        "auc_threshold": float(config["mechanisms"]["classifier_auc_threshold"]),
        "monotonicity_2d_restored": bool(
            classifier.get("auc") is not None
            and float(classifier["auc"]) >= float(config["mechanisms"]["classifier_auc_threshold"])
        ),
        "mechanism_thresholds": mechanism_thresholds,
        "mechanism_counts": dict(mechanism_counts),
        "clean_unsafe_count": len(hard_features),
    }
    result["artifacts"] = {"plots": make_mechanism_plots(result)}
    return result


def ep_points(config: dict[str, Any]) -> list[dict[str, Any]]:
    points = []
    grid = config["ep_grid"]
    for e_layer in grid["turbulence_layers"]:
        for p_layer in grid["angle_max_layers"]:
            for r_value in grid["r_deg_s"]:
                for seed in grid["seeds"]:
                    layer = f"{e_layer['name']}_{p_layer['name']}"
                    points.append(
                        {
                            "role": "epgrid",
                            "phase": "ep_grid",
                            "layer": layer,
                            "E_layer": e_layer["name"],
                            "P_layer": p_layer["name"],
                            "r_deg_s": float(r_value),
                            "seed": int(seed),
                            "wind_m_s": float(e_layer["wind_m_s"]),
                            "turbulence_m_s": float(e_layer["turbulence_m_s"]),
                            "angle_max_cd": float(p_layer["angle_max_cd"]),
                            "model": str(grid.get("model", config["pressure"]["default_model"])),
                        }
                    )
    return points


def run_cached_grid(
    config: dict[str, Any],
    point: dict[str, Any],
    partial_path: Path,
    all_runs: list[dict[str, Any]],
    *,
    resume: bool,
) -> tuple[dict[str, Any], bool]:
    run_id = run_id_for(config, point)
    if resume:
        for existing in all_runs:
            if existing.get("run_id") == run_id and not existing.get("error"):
                return existing, False
    print(f"RUN grid {run_id}", flush=True)
    run = run_oracleA_one(config, point)
    partial = load_json(partial_path, {"grid_runs": [], "reachability_runs": []})
    grid_runs = [r for r in partial.get("grid_runs", []) if r.get("run_id") != run_id] + [run]
    partial["grid_runs"] = grid_runs
    partial["updated_at_utc"] = utc_now()
    write_json(partial_path, partial)
    return run, True


def sustained_bank_profile(config: dict[str, Any], spec: dict[str, Any]) -> list[dict[str, float]]:
    hz = float(config["experiment"]["stream_hz"])
    dt = 1.0 / hz
    angle_max_cd = float(spec["angle_max_cd"])
    angle_max_deg = angle_max_cd / 100.0
    guard = float(config["oracle"]["command_angle_guard_deg"])
    amp = max(
        0.0,
        min(
            float(spec.get("max_nominal_amplitude_deg", angle_max_deg)),
            angle_max_deg * float(spec.get("amplitude_fraction_of_angle_max", 0.98)),
            angle_max_deg - guard,
        ),
    )
    ramp_rate = abs(float(spec.get("ramp_rate_deg_s", 90.0)))
    hold_s = float(spec.get("hold_s", 20.0))
    axis = str(spec.get("axis", "roll"))
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
        while (target - current) * sign > 1.0e-9:
            step = min(abs(target - current), ramp_rate * dt)
            current += sign * step
            append(current, sign * ramp_rate)

    def hold(value: float, duration_s: float) -> None:
        nonlocal current
        current = value
        for _ in range(max(1, int(round(duration_s * hz)))):
            append(current, 0.0)

    ramp_to(amp)
    hold(amp, hold_s)
    ramp_to(0.0)
    hold(0.0, float(config["experiment"].get("post_maneuver_hold_s", 1.0)))
    return samples


def reach_run_id(config: dict[str, Any], spec: dict[str, Any]) -> str:
    prefix = str(config["experiment"].get("run_prefix", "epmovev1"))
    return (
        f"{prefix}_reach_{sanitize_id(str(spec['name']))}"
        f"_a{int(round(float(spec['angle_max_cd']))):04d}"
        f"_w{int(round(float(spec.get('wind_m_s', 0.0)))):02d}"
        f"_t{int(round(float(spec.get('turbulence_m_s', 0.0)))):02d}"
        f"_s{int(spec.get('seed', 0)):02d}"
    )


def run_profile_once(config: dict[str, Any], spec: dict[str, Any]) -> dict[str, Any]:
    run_id = reach_run_id(config, spec)
    point = {
        "role": "reachability",
        "phase": "reachability",
        "layer": sanitize_id(str(spec["name"])),
        "E_layer": "Ereach",
        "P_layer": f"P{int(round(float(spec['angle_max_cd']) / 100.0))}",
        "r_deg_s": float(spec.get("ramp_rate_deg_s", 0.0)),
        "seed": int(spec.get("seed", 0)),
        "wind_m_s": float(spec.get("wind_m_s", 0.0)),
        "turbulence_m_s": float(spec.get("turbulence_m_s", 0.0)),
        "angle_max_cd": float(spec["angle_max_cd"]),
        "model": str(spec.get("model", config["pressure"]["default_model"])),
        "input_name": spec["name"],
    }
    cfg = point_config(config, point)
    runner = SitlRunner(cfg, REPO_ROOT)
    result: dict[str, Any] = {"run_id": run_id, "point": point, "input_spec": spec, "started_at_utc": utc_now()}
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
        thrust = float(spec.get("thrust", config["command"].get("thrust", 0.5)))
        attitude_ignore = str(spec.get("attitude_target_mode", config["command"].get("attitude_target_mode", "rate_only"))) == "rate_only"
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

        profile = sustained_bank_profile(config, spec)
        profile_path = PLANC_ROOT / "logs" / f"{run_id}_command_profile.json"
        max_command_angle = max(
            (math.hypot(float(row.get("roll_deg", 0.0)), float(row.get("pitch_deg", 0.0))) for row in profile),
            default=0.0,
        )
        write_json(
            profile_path,
            {
                "run_id": run_id,
                "point": point,
                "input_spec": spec,
                "profile_kind": spec.get("profile", "sustained_bank"),
                "attitude_target_mode": spec.get("attitude_target_mode", "attitude_and_rate"),
                "samples": profile,
            },
        )
        result["command_profile"] = {
            "samples": len(profile),
            "duration_s": profile[-1]["t_s"] if profile else 0.0,
            "amplitude_deg": max_command_angle,
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


def run_cached_reachability(
    config: dict[str, Any],
    spec: dict[str, Any],
    partial_path: Path,
    all_runs: list[dict[str, Any]],
    *,
    resume: bool,
) -> tuple[dict[str, Any], bool]:
    run_id = reach_run_id(config, spec)
    if resume:
        for existing in all_runs:
            if existing.get("run_id") == run_id and not existing.get("error"):
                return existing, False
    print(f"RUN reachability {run_id}", flush=True)
    run = run_profile_once(config, spec)
    partial = load_json(partial_path, {"grid_runs": [], "reachability_runs": []})
    reachability_runs = [r for r in partial.get("reachability_runs", []) if r.get("run_id") != run_id] + [run]
    partial["reachability_runs"] = reachability_runs
    partial["updated_at_utc"] = utc_now()
    write_json(partial_path, partial)
    return run, True


def feature_rows_for_runs(runs: list[dict[str, Any]], config: dict[str, Any], high_bank_threshold_deg: float) -> list[dict[str, Any]]:
    rows = []
    for run in runs:
        if run.get("error") or not run.get("csv_path"):
            continue
        try:
            rows.append(run_feature_row(run, config, high_bank_threshold_deg=high_bank_threshold_deg))
        except Exception as exc:
            rows.append({"run_id": run.get("run_id"), "error": repr(exc)})
    return rows


def cell_key(run: dict[str, Any]) -> tuple[str, str, float]:
    point = run.get("point", {})
    return (
        str(point.get("E_layer", "")),
        str(point.get("P_layer", "")),
        float(point.get("r_deg_s", 0.0)),
    )


def summarize_grid(
    runs: list[dict[str, Any]],
    features: list[dict[str, Any]],
    mechanisms: dict[str, Any],
    config: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    feature_by_id = {row.get("run_id"): row for row in features if not row.get("error")}
    classifier = mechanisms["best_2d_classifier"]
    score_threshold = float(classifier["threshold"]["threshold"])
    by_cell: dict[tuple[str, str, float], list[dict[str, Any]]] = defaultdict(list)
    for run in runs:
        if not run.get("error"):
            by_cell[cell_key(run)].append(run)

    cell_rows: list[dict[str, Any]] = []
    for (e_layer, p_layer, r_value), cell_runs in sorted(by_cell.items()):
        labels = Counter(run.get("label") for run in cell_runs)
        scores = [zscore_score(feature_by_id[run["run_id"]], classifier) for run in cell_runs if run.get("run_id") in feature_by_id]
        score_hits = [score for score in scores if score is not None and score >= score_threshold]
        b_clamp = sum(1 for run in cell_runs if run.get("oracle_B", {}).get("B_clamp"))
        b_preventive = sum(1 for run in cell_runs if run.get("oracle_B", {}).get("B_preventive"))
        b_any = sum(1 for run in cell_runs if run.get("oracle_B", {}).get("B_any_window"))
        margins = []
        max_commands = []
        angle_maxes = []
        for run in cell_runs:
            command = run.get("command", {})
            if command.get("max_command_angle_deg") is not None and command.get("angle_max_deg") is not None:
                max_commands.append(float(command["max_command_angle_deg"]))
                angle_maxes.append(float(command["angle_max_deg"]))
                margins.append(float(command["angle_max_deg"]) - float(command["max_command_angle_deg"]))
        n = len(cell_runs)
        clean = labels.get("clean_unsafe", 0)
        hard = sum(1 for run in cell_runs if run.get("hardened_oracle_A", {}).get("inside"))
        cell_rows.append(
            {
                "E_layer": e_layer,
                "P_layer": p_layer,
                "r_deg_s": r_value,
                "n": n,
                "hard_A": hard,
                "clean_unsafe": clean,
                "recovered": labels.get("recovered", 0),
                "safe": labels.get("safe", 0),
                "bug_side": labels.get("bug_side", 0),
                "p_clean_unsafe": clean / n if n else None,
                "p_hard_A": hard / n if n else None,
                "mechanism_score_ge_threshold": len(score_hits),
                "p_mechanism_score_ge_threshold": len(score_hits) / n if n else None,
                "B_clamp_count": b_clamp,
                "B_preventive_count": b_preventive,
                "B_any_window_count": b_any,
                "contract_invariant_trigger_count": b_clamp + b_any,
                "max_command_angle_deg": max(max_commands, default=None),
                "angle_max_deg": max(angle_maxes, default=None),
                "min_command_margin_deg": min(margins, default=None),
                "command_clean": b_clamp == 0 and (min(margins, default=1.0) >= -1.0e-6),
                "B_clean": b_preventive == 0 and b_any == 0,
            }
        )

    by_ep: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in cell_rows:
        by_ep[(row["E_layer"], row["P_layer"])].append(row)
    ep_rows: list[dict[str, Any]] = []
    boundary_probability = float(config["ep_grid"]["boundary_probability"])
    for (e_layer, p_layer), rows in sorted(by_ep.items()):
        rows = sorted(rows, key=lambda row: float(row["r_deg_s"]))
        boundary = None
        for row in rows:
            if row["p_clean_unsafe"] is not None and float(row["p_clean_unsafe"]) >= boundary_probability:
                boundary = float(row["r_deg_s"])
                break
        clean = sum(int(row["clean_unsafe"]) for row in rows)
        n = sum(int(row["n"]) for row in rows)
        ep_rows.append(
            {
                "E_layer": e_layer,
                "P_layer": p_layer,
                "n": n,
                "clean_unsafe": clean,
                "p_clean_unsafe": clean / n if n else None,
                "unsafe_area_mean_over_r": statistics.fmean(float(row["p_clean_unsafe"]) for row in rows if row["p_clean_unsafe"] is not None),
                "score_area_mean_over_r": statistics.fmean(
                    float(row["p_mechanism_score_ge_threshold"])
                    for row in rows
                    if row["p_mechanism_score_ge_threshold"] is not None
                ),
                "boundary_r50_deg_s": boundary,
                "B_clamp_count": sum(int(row["B_clamp_count"]) for row in rows),
                "B_preventive_count": sum(int(row["B_preventive_count"]) for row in rows),
                "B_any_window_count": sum(int(row["B_any_window_count"]) for row in rows),
                "contract_invariant_trigger_count": sum(int(row["contract_invariant_trigger_count"]) for row in rows),
                "command_clean": all(bool(row["command_clean"]) for row in rows),
                "B_clean": all(bool(row["B_clean"]) for row in rows),
                "min_command_margin_deg": min(float(row["min_command_margin_deg"]) for row in rows if row["min_command_margin_deg"] is not None),
            }
        )

    monotonic = monotonicity_summary(ep_rows, config)
    return cell_rows, ep_rows, monotonic


def monotonic_non_decreasing(values: list[float], tol: float = 1.0e-9) -> bool:
    return all(b + tol >= a for a, b in zip(values, values[1:]))


def monotonic_non_increasing(values: list[float], tol: float = 1.0e-9) -> bool:
    return all(b <= a + tol for a, b in zip(values, values[1:]))


def boundary_value(value: float | None) -> float:
    return float("inf") if value is None else float(value)


def monotonicity_summary(ep_rows: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    e_order = [str(row["name"]) for row in config["ep_grid"]["turbulence_layers"]]
    p_order = [str(row["name"]) for row in config["ep_grid"]["angle_max_layers"]]
    by_key = {(row["E_layer"], row["P_layer"]): row for row in ep_rows}
    e_checks = []
    for p_layer in p_order:
        rows = [by_key[(e_layer, p_layer)] for e_layer in e_order if (e_layer, p_layer) in by_key]
        areas = [float(row["unsafe_area_mean_over_r"]) for row in rows]
        boundaries = [boundary_value(row["boundary_r50_deg_s"]) for row in rows]
        e_checks.append(
            {
                "P_layer": p_layer,
                "E_order": e_order,
                "unsafe_area": areas,
                "boundary_r50": [None if math.isinf(v) else v for v in boundaries],
                "unsafe_area_nondecreasing": monotonic_non_decreasing(areas),
                "boundary_nonincreasing": monotonic_non_increasing(boundaries),
            }
        )
    p_checks = []
    for e_layer in e_order:
        rows = [by_key[(e_layer, p_layer)] for p_layer in p_order if (e_layer, p_layer) in by_key]
        areas = [float(row["unsafe_area_mean_over_r"]) for row in rows]
        boundaries = [boundary_value(row["boundary_r50_deg_s"]) for row in rows]
        p_checks.append(
            {
                "E_layer": e_layer,
                "P_order": p_order,
                "unsafe_area": areas,
                "boundary_r50": [None if math.isinf(v) else v for v in boundaries],
                "unsafe_area_nondecreasing": monotonic_non_decreasing(areas),
                "boundary_nonincreasing": monotonic_non_increasing(boundaries),
            }
        )
    return {
        "E_monotone_region_expands": all(row["unsafe_area_nondecreasing"] for row in e_checks),
        "E_monotone_boundary_moves_forward": all(row["boundary_nonincreasing"] for row in e_checks),
        "P_monotone_region_expands": all(row["unsafe_area_nondecreasing"] for row in p_checks),
        "P_monotone_boundary_moves_forward": all(row["boundary_nonincreasing"] for row in p_checks),
        "E_checks_by_P": e_checks,
        "P_checks_by_E": p_checks,
    }


def trajectory_series(run: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    if not run or not run.get("csv_path"):
        return {}
    active = run.get("active_window_s", {})
    start = active.get("start")
    end = active.get("oracle_end")
    if start is None or end is None:
        return {}
    profile_path = Path(str(run["command_profile"]["path"]))
    profile = load_json(profile_path).get("samples", [])
    rows = read_csv_by_type(Path(str(run["csv_path"])))
    att_rows = [row for row in rows.get("ATT", []) if float(start) <= float(row["_t"]) <= float(end)]
    pos_rows = [(float(row["_t"]) - float(start), frow(row, "RelHomeAlt")) for row in rows.get("POS", []) if float(start) <= float(row["_t"]) <= float(end)]
    series = {"time_s": [], "att_error_deg": [], "rel_alt_m": [], "bank_deg": []}
    for row in att_rows:
        rel_t = float(row["_t"]) - float(start)
        cmd = command_at(profile, rel_t)
        roll = frow(row, "Roll")
        pitch = frow(row, "Pitch")
        error = math.hypot(roll - float(cmd.get("roll_deg", 0.0)), pitch - float(cmd.get("pitch_deg", 0.0)))
        series["time_s"].append(rel_t)
        series["att_error_deg"].append(error)
        series["rel_alt_m"].append(nearest_value(pos_rows, rel_t))
        series["bank_deg"].append(math.hypot(roll, pitch))
    series["run_id"] = run.get("run_id")
    series["label"] = run.get("label")
    return series


def make_ep_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    paths: dict[str, str] = {}
    config = payload["config"]
    e_order = [str(row["name"]) for row in config["ep_grid"]["turbulence_layers"]]
    p_order = [str(row["name"]) for row in config["ep_grid"]["angle_max_layers"]]
    ep_by_key = {(row["E_layer"], row["P_layer"]): row for row in payload["ep_summary"]}

    area = [[ep_by_key.get((e, p), {}).get("unsafe_area_mean_over_r", float("nan")) for p in p_order] for e in e_order]
    boundary = [[ep_by_key.get((e, p), {}).get("boundary_r50_deg_s") for p in p_order] for e in e_order]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    im = axes[0].imshow(area, vmin=0.0, vmax=1.0, cmap="magma", aspect="auto")
    axes[0].set_xticks(range(len(p_order)), p_order)
    axes[0].set_yticks(range(len(e_order)), e_order)
    axes[0].set_title("Unsafe region area over r")
    for i, e_layer in enumerate(e_order):
        for j, p_layer in enumerate(p_order):
            value = area[i][j]
            axes[0].text(j, i, "n/a" if math.isnan(value) else f"{value:.2f}", ha="center", va="center", color="white")
    fig.colorbar(im, ax=axes[0], fraction=0.046, pad=0.04)
    axes[1].imshow([[0.0 for _ in p_order] for _ in e_order], vmin=0.0, vmax=1.0, cmap="Greys", aspect="auto")
    axes[1].set_xticks(range(len(p_order)), p_order)
    axes[1].set_yticks(range(len(e_order)), e_order)
    axes[1].set_title("Input-axis boundary r50")
    for i, e_layer in enumerate(e_order):
        for j, p_layer in enumerate(p_order):
            value = boundary[i][j]
            axes[1].text(j, i, "none" if value is None else f"{float(value):.0f}", ha="center", va="center", color="#111111")
    path = analysis / "ep_movement_boundary_surface.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    paths["boundary_surface"] = str(path)

    rows = payload["cell_summary"]
    fig, axes = plt.subplots(len(e_order), len(p_order), figsize=(13, 9), sharex=True, sharey=True)
    if len(e_order) == 1:
        axes = [axes]
    boundary_handles = None
    for i, e_layer in enumerate(e_order):
        for j, p_layer in enumerate(p_order):
            ax = axes[i][j]
            subset = sorted(
                [row for row in rows if row["E_layer"] == e_layer and row["P_layer"] == p_layer],
                key=lambda row: float(row["r_deg_s"]),
            )
            clean_line = ax.plot(
                [row["r_deg_s"] for row in subset],
                [row["p_clean_unsafe"] for row in subset],
                marker="o",
                color="#d1495b",
            )[0]
            score_line = ax.plot(
                [row["r_deg_s"] for row in subset],
                [row["p_mechanism_score_ge_threshold"] for row in subset],
                marker="x",
                linestyle="--",
                color="#003049",
            )[0]
            if boundary_handles is None:
                boundary_handles = [clean_line, score_line]
            ax.set_title(f"{e_layer} / {p_layer}", fontsize=10)
            ax.grid(True, alpha=0.22)
            ax.set_ylim(-0.05, 1.05)
    for ax in axes[-1]:
        ax.set_xlabel("r (deg/s)")
    for row_axes in axes:
        row_axes[0].set_ylabel("probability")
    path = analysis / "ep_movement_r_boundaries_by_cell.png"
    if boundary_handles is not None:
        fig.legend(
            boundary_handles,
            ["clean hard-A rate", "2D score >= threshold"],
            loc="lower center",
            ncol=2,
            frameon=False,
        )
    fig.tight_layout(rect=(0.0, 0.05, 1.0, 1.0))
    fig.savefig(path, dpi=180)
    plt.close(fig)
    paths["r_boundaries_by_cell"] = str(path)

    fig, ax = plt.subplots(figsize=(9, 3.8))
    table_rows = []
    for row in payload["ep_summary"]:
        table_rows.append(
            [
                row["E_layer"],
                row["P_layer"],
                str(row["contract_invariant_trigger_count"]),
                str(row["B_preventive_count"]),
                str(row["B_clamp_count"]),
            ]
        )
    ax.axis("off")
    table = ax.table(
        cellText=table_rows,
        colLabels=["E", "P", "contract triggers", "preventive B", "command clamp"],
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.35)
    ax.set_title("Contract/failsafe triggers by E/P cell")
    path = analysis / "ep_movement_contract_trigger_table.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    paths["contract_trigger_table"] = str(path)

    series = payload.get("reachability_trajectory_series", {})
    if series:
        fig, axes = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True)
        ts = series.get("time_s", [])
        axes[0].plot(ts, series.get("bank_deg", []), color="#f77f00")
        axes[0].set_ylabel("bank deg")
        axes[1].plot(ts, series.get("att_error_deg", []), color="#d1495b")
        axes[1].axhline(float(config["oracle"]["divergence_error_deg"]), color="#222222", linestyle="--", linewidth=1)
        axes[1].set_ylabel("att error deg")
        axes[2].plot(ts, series.get("rel_alt_m", []), color="#003049")
        axes[2].set_ylabel("RelHomeAlt m")
        axes[2].set_xlabel("seconds from oracle-window start")
        fig.suptitle("Non-doublet reachability trajectory", fontsize=12)
        axes[0].set_title(f"{series.get('run_id')} ({series.get('label')})", fontsize=9, pad=4)
        for ax in axes:
            ax.grid(True, alpha=0.22)
        path = analysis / "ep_movement_reachability_trajectory.png"
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
        fig.savefig(path, dpi=180)
        plt.close(fig)
        paths["reachability_trajectory"] = str(path)
    return paths


def select_reachability_witness(runs: list[dict[str, Any]]) -> dict[str, Any] | None:
    clean = [run for run in runs if not run.get("error") and run.get("label") == "clean_unsafe"]
    if clean:
        return max(clean, key=lambda run: float(run.get("hardened_oracle_A", {}).get("altitude", {}).get("loss_m") or 0.0))
    complete = [run for run in runs if not run.get("error")]
    if complete:
        return max(complete, key=lambda run: float(run.get("hardened_oracle_A", {}).get("altitude", {}).get("loss_m") or 0.0))
    return None


def build_report(payload: dict[str, Any]) -> str:
    path = PLANC_ROOT / "results" / "ep_movement_report.md"
    mechanisms = payload["mechanisms"]
    monotonic = payload["monotonicity"]
    lines = [
        "# E/P Boundary Movement v1",
        "",
        "## Mechanism Separation",
        "",
        f"- Source oracleA runs: {mechanisms['source']['source_runs']}.",
        f"- Mechanism counts among clean hard-A runs: `{json.dumps(mechanisms['mechanism_counts'], sort_keys=True)}`.",
        f"- Best 2D pair: `{mechanisms['best_2d_feature_pair'][0]}` x `{mechanisms['best_2d_feature_pair'][1]}`.",
        f"- 2D AUC: {fmt(mechanisms['best_2d_auc'], 3)}; restored: {mechanisms['monotonicity_2d_restored']}.",
        "",
        "## E/P Grid",
        "",
        f"- Completed grid runs: {payload['summary']['completed_grid_runs']}; errors: {payload['summary']['error_grid_runs']}.",
        f"- Command clean in all E/P cells: {payload['summary']['all_commands_within_angle_max']}.",
        f"- Flight-controller B clean in all E/P cells: {payload['summary']['all_B_zero']}.",
        f"- E unsafe-area monotone: {monotonic['E_monotone_region_expands']}; E boundary moves forward: {monotonic['E_monotone_boundary_moves_forward']}.",
        f"- P unsafe-area monotone: {monotonic['P_monotone_region_expands']}; P boundary moves forward: {monotonic['P_monotone_boundary_moves_forward']}.",
        "",
        "| E | P | n | p_clean | r50 | contract | B_preventive | clamp | min cmd margin |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in payload["ep_summary"]:
        lines.append(
            f"| {row['E_layer']} | {row['P_layer']} | {row['n']} | {fmt(row['p_clean_unsafe'], 3)} | {fmt(row['boundary_r50_deg_s'], 0)} | {row['contract_invariant_trigger_count']} | {row['B_preventive_count']} | {row['B_clamp_count']} | {fmt(row['min_command_margin_deg'], 2)} |"
        )
    lines.extend(
        [
            "",
            "## Reachability",
            "",
            f"- Completed reachability runs: {payload['summary']['completed_reachability_runs']}; errors: {payload['summary']['error_reachability_runs']}.",
            f"- Non-doublet clean unsafe found: {payload['reachability']['clean_unsafe_found']}.",
            f"- Witness: `{payload['reachability'].get('witness_run_id')}`.",
            "",
            "## Artifacts",
            "",
            "- Mechanisms JSON: `planc/results/mechanisms_result.json`",
            "- E/P result JSON: `planc/results/ep_movement_result.json`",
        ]
    )
    for name, plot in payload.get("artifacts", {}).get("plots", {}).items():
        lines.append(f"- Plot {name}: `{Path(plot).relative_to(REPO_ROOT)}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(path)


def build_ep_payload(
    config: dict[str, Any],
    env: dict[str, Any],
    prereg: dict[str, Any],
    mechanisms: dict[str, Any],
    grid_runs: list[dict[str, Any]],
    reachability_runs: list[dict[str, Any]],
    *,
    status: str,
) -> dict[str, Any]:
    high_bank_threshold = float(mechanisms["best_2d_classifier"].get("high_bank_threshold_deg") or mechanisms["features"][0]["high_bank_threshold_deg"])
    grid_features = feature_rows_for_runs(grid_runs, config, high_bank_threshold)
    for row in grid_features:
        if row.get("error"):
            continue
        score = zscore_score(row, mechanisms["best_2d_classifier"])
        row["mechanism_score"] = score
        row["mechanism_classifier_predicted_unsafe"] = (
            score is not None and score >= float(mechanisms["best_2d_classifier"]["threshold"]["threshold"])
        )
    cell_summary, ep_summary, monotonicity = summarize_grid(grid_runs, grid_features, mechanisms, config)
    reachability_features = feature_rows_for_runs(reachability_runs, config, high_bank_threshold)
    for row in reachability_features:
        if row.get("error"):
            continue
        score = zscore_score(row, mechanisms["best_2d_classifier"])
        row["mechanism_score"] = score
        row["mechanism_classifier_predicted_unsafe"] = (
            score is not None and score >= float(mechanisms["best_2d_classifier"]["threshold"]["threshold"])
        )
    witness = select_reachability_witness(reachability_runs)
    witness_series = trajectory_series(witness, config) if witness is not None else {}
    complete_grid = [run for run in grid_runs if not run.get("error")]
    complete_reach = [run for run in reachability_runs if not run.get("error")]
    payload = {
        "status": status,
        "generated_at_utc": utc_now(),
        "config": config,
        "env": env,
        "preregistration": prereg,
        "mechanisms": mechanisms,
        "grid_features": grid_features,
        "cell_summary": cell_summary,
        "ep_summary": ep_summary,
        "monotonicity": monotonicity,
        "reachability_features": reachability_features,
        "reachability": {
            "clean_unsafe_found": any(run.get("label") == "clean_unsafe" for run in complete_reach),
            "witness_run_id": None if witness is None else witness.get("run_id"),
            "witness_label": None if witness is None else witness.get("label"),
        },
        "reachability_trajectory_series": witness_series,
        "summary": {
            "completed_grid_runs": len(complete_grid),
            "error_grid_runs": len([run for run in grid_runs if run.get("error")]),
            "completed_reachability_runs": len(complete_reach),
            "error_reachability_runs": len([run for run in reachability_runs if run.get("error")]),
            "all_commands_within_angle_max": all(bool(row["command_clean"]) for row in cell_summary) if cell_summary else False,
            "all_B_zero": all(bool(row["B_clean"]) for row in cell_summary) if cell_summary else False,
            "contract_invariant_trigger_count": sum(int(row["contract_invariant_trigger_count"]) for row in cell_summary),
            "preventive_failsafe_trigger_count": sum(int(row["B_preventive_count"]) for row in cell_summary),
            "command_clamp_count": sum(int(row["B_clamp_count"]) for row in cell_summary),
        },
        "grid_runs": grid_runs,
        "reachability_runs": reachability_runs,
        "artifacts": {},
    }
    payload["artifacts"]["plots"] = make_ep_plots(payload)
    payload["artifacts"]["report"] = build_report(payload)
    return payload


def write_preregister(config: dict[str, Any], path: Path) -> dict[str, Any]:
    payload = {
        "status": "preregistered",
        "written_at_utc": utc_now(),
        "experiment": config["experiment"],
        "mechanisms": config["mechanisms"],
        "ep_grid": config["ep_grid"],
        "reachability": config["reachability"],
        "oracle": "Reuses oracleA v1 hardened A/B hygiene: MODE.Rsn=GCS_COMMAND excluded; cleanup disarm after oracle window.",
    }
    write_json(path, payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "ep_movement_v1_config.yaml")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--analysis-only", action="store_true")
    parser.add_argument("--grid-only", action="store_true")
    parser.add_argument("--reachability-only", action="store_true")
    parser.add_argument("--max-new-runs", type=int, default=None)
    args = parser.parse_args()

    config = load_yaml(args.config)
    stem = artifact_stem(config)
    results_dir = PLANC_ROOT / "results"
    prereg_path = results_dir / f"{stem}_prereg.json"
    result_path = results_dir / "ep_movement_result.json"
    mechanisms_path = results_dir / "mechanisms_result.json"
    env_path = results_dir / f"env_{stem}.json"
    partial_path = results_dir / f"{stem}_partial.json"

    env = probe_environment(config, REPO_ROOT)
    write_env(env, env_path)
    prereg = write_preregister(config, prereg_path)
    mechanisms = build_mechanisms(config)
    # Record the selected high-bank threshold on the classifier for downstream scoring.
    mechanisms["best_2d_classifier"]["high_bank_threshold_deg"] = mechanisms["features"][0]["high_bank_threshold_deg"]
    write_json(mechanisms_path, mechanisms)
    print(
        f"MECHANISMS: AUC={fmt(mechanisms['best_2d_auc'], 3)} counts={dict(mechanisms['mechanism_counts'])} result={mechanisms_path}",
        flush=True,
    )
    if args.analysis_only:
        return

    partial = load_json(partial_path, {"grid_runs": [], "reachability_runs": []}) if args.resume else {"grid_runs": [], "reachability_runs": []}
    grid_runs: list[dict[str, Any]] = list(partial.get("grid_runs", []))
    reachability_runs: list[dict[str, Any]] = list(partial.get("reachability_runs", []))
    new_runs = 0

    if not args.reachability_only:
        for point in ep_points(config):
            run, did_run = run_cached_grid(config, point, partial_path, grid_runs, resume=args.resume)
            if did_run:
                new_runs += 1
            partial = load_json(partial_path, {"grid_runs": [], "reachability_runs": []})
            grid_runs = list(partial.get("grid_runs", []))
            print(
                f"DONE grid {run.get('run_id')} label={run.get('label')} outcome={run.get('hardened_oracle_A', {}).get('outcome')} error={run.get('error')}",
                flush=True,
            )
            if args.max_new_runs is not None and new_runs >= args.max_new_runs:
                payload = build_ep_payload(config, env, prereg, mechanisms, grid_runs, reachability_runs, status="partial")
                write_json(result_path, payload)
                print(f"PARTIAL: result={result_path}", flush=True)
                return

    if not args.grid_only:
        for spec in config["reachability"]["inputs"]:
            run, did_run = run_cached_reachability(config, spec, partial_path, reachability_runs, resume=args.resume)
            if did_run:
                new_runs += 1
            partial = load_json(partial_path, {"grid_runs": [], "reachability_runs": []})
            reachability_runs = list(partial.get("reachability_runs", []))
            print(
                f"DONE reachability {run.get('run_id')} label={run.get('label')} outcome={run.get('hardened_oracle_A', {}).get('outcome')} error={run.get('error')}",
                flush=True,
            )
            if args.max_new_runs is not None and new_runs >= args.max_new_runs:
                payload = build_ep_payload(config, env, prereg, mechanisms, grid_runs, reachability_runs, status="partial")
                write_json(result_path, payload)
                print(f"PARTIAL: result={result_path}", flush=True)
                return

    payload = build_ep_payload(config, env, prereg, mechanisms, grid_runs, reachability_runs, status="complete")
    write_json(result_path, payload)
    print(f"COMPLETE: result={result_path} report={payload['artifacts']['report']}", flush=True)


if __name__ == "__main__":
    main()
