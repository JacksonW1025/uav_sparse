from __future__ import annotations

import argparse
import csv
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

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))

from env_probe import probe_environment, write_env
from run_stage0_v2 import command_at, fmt


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


def artifact_stem(config: dict[str, Any]) -> str:
    return str(config["experiment"].get("artifact_stem", "demand_v1"))


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


def ranks(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    out = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        rank = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            out[order[k]] = rank
        i = j + 1
    return out


def spearman(values: list[float], labels: list[int]) -> float | None:
    pairs = [(v, y) for v, y in zip(values, labels) if v is not None and math.isfinite(v)]
    if len(pairs) < 3:
        return None
    xs = [v for v, _ in pairs]
    ys = [float(y) for _, y in pairs]
    if len(set(xs)) < 2 or len(set(ys)) < 2:
        return None
    rx = ranks(xs)
    ry = ranks(ys)
    mx = statistics.fmean(rx)
    my = statistics.fmean(ry)
    num = sum((x - mx) * (y - my) for x, y in zip(rx, ry))
    den = math.sqrt(sum((x - mx) ** 2 for x in rx) * sum((y - my) ** 2 for y in ry))
    return None if den <= 0.0 else num / den


def auc(values: list[float], labels: list[int]) -> float | None:
    pairs = [(v, y) for v, y in zip(values, labels) if v is not None and math.isfinite(v)]
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


def oriented_auc(raw_auc: float | None) -> tuple[float | None, int]:
    if raw_auc is None:
        return None, 1
    if raw_auc >= 0.5:
        return raw_auc, 1
    return 1.0 - raw_auc, -1


def threshold_scan(values: list[float], labels: list[int], direction: int) -> dict[str, Any]:
    pairs = [(float(v) * direction, int(y), float(v)) for v, y in zip(values, labels) if v is not None and math.isfinite(v)]
    if not pairs:
        return {"threshold": None}
    candidates = sorted(set(v for v, _, _ in pairs))
    best: dict[str, Any] | None = None
    for thr_oriented in candidates:
        tp = fp = tn = fn = 0
        for oriented, y, raw in pairs:
            pred = oriented >= thr_oriented
            if pred and y == 1:
                tp += 1
            elif pred and y == 0:
                fp += 1
            elif not pred and y == 0:
                tn += 1
            else:
                fn += 1
        tpr = tp / (tp + fn) if tp + fn else 0.0
        fpr = fp / (fp + tn) if fp + tn else 0.0
        bal_acc = 0.5 * (tpr + (1.0 - fpr))
        youden = tpr - fpr
        row = {
            "threshold": thr_oriented * direction,
            "direction": ">=" if direction > 0 else "<=",
            "tp": tp,
            "fp": fp,
            "tn": tn,
            "fn": fn,
            "tpr": tpr,
            "fpr": fpr,
            "balanced_accuracy": bal_acc,
            "youden_j": youden,
        }
        if best is None or (row["youden_j"], row["balanced_accuracy"]) > (best["youden_j"], best["balanced_accuracy"]):
            best = row
    return best or {"threshold": None}


def mean_dt(times: list[float], fallback: float) -> float:
    if len(times) < 2:
        return fallback
    diffs = [b - a for a, b in zip(times, times[1:]) if b > a]
    return statistics.fmean(diffs) if diffs else fallback


def read_rows(path: Path, start: float, end: float) -> dict[str, list[dict[str, Any]]]:
    by_type: dict[str, list[dict[str, Any]]] = defaultdict(list)
    with path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["time_s"])
            except Exception:
                continue
            if t < start or t > end:
                continue
            row["_t"] = t
            by_type[str(row["type"])].append(row)
    return by_type


def frow(row: dict[str, Any], name: str, default: float = 0.0) -> float:
    try:
        raw = row.get(name, "")
        return default if raw == "" else float(raw)
    except Exception:
        return default


def command_features(profile: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, float | None]:
    if not profile:
        return {}
    times = [float(row["t_s"]) for row in profile]
    dt = mean_dt(times, float(config["feature_extraction"]["csv_time_step_fallback_s"]))
    rate_mag = [
        math.hypot(float(row.get("roll_rate_deg_s", 0.0)), float(row.get("pitch_rate_deg_s", 0.0)))
        for row in profile
    ]
    roll_rates = [float(row.get("roll_rate_deg_s", 0.0)) for row in profile]
    accel = [abs((b - a) / dt) for a, b in zip(roll_rates, roll_rates[1:])] if dt > 0 else []
    threshold = float(config["feature_extraction"]["rate_high_thresholds_deg_s"][0])
    return {
        "command_rate_impulse_deg": sum(rate_mag) * dt,
        "command_peak_accel_proxy_deg_s2": max(accel, default=0.0),
        "command_high_rate_time_s": sum(1 for v in rate_mag if v >= threshold) * dt,
    }


def zero_crossings(values: list[tuple[float, float]], min_abs_before: float = 5.0) -> list[float]:
    out: list[float] = []
    prev_t = None
    prev_v = None
    for t, value in values:
        if prev_t is not None and prev_v is not None:
            if abs(prev_v) >= min_abs_before and prev_v * value <= 0.0:
                span = value - prev_v
                frac = 0.0 if span == 0 else (0.0 - prev_v) / span
                out.append(prev_t + max(0.0, min(1.0, frac)) * (t - prev_t))
        prev_t = t
        prev_v = value
    return out


def reversal_phase_ratio(
    profile: list[dict[str, Any]],
    att_roll: list[tuple[float, float]],
    active_start: float,
    config: dict[str, Any],
) -> float | None:
    if not profile or not att_roll:
        return None
    min_rate = float(config["feature_extraction"]["reversal_phase"]["min_command_rate_deg_s"])
    cmd_rates = [(active_start + float(row["t_s"]), float(row.get("roll_rate_deg_s", 0.0))) for row in profile]
    reversals = zero_crossings(cmd_rates, min_abs_before=min_rate)
    roll_cross = zero_crossings(att_roll, min_abs_before=5.0)
    lags = []
    for rev in reversals:
        after = [t for t in roll_cross if t >= rev]
        if after:
            lags.append(after[0] - rev)
    if not lags:
        return None
    # Normalize by the nominal command half-cycle duration.
    duration = float(profile[-1]["t_s"]) - float(profile[0]["t_s"])
    cycles = max(1, len(reversals))
    nominal = duration / cycles
    return statistics.fmean(lags) / nominal if nominal > 0 else None


def extract_features(run: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    start = float(run["active_window_s"]["start"])
    maneuver_end = float(run["active_window_s"]["maneuver_end"])
    oracle_end = float(run["active_window_s"]["oracle_end"])
    rows = read_rows(Path(str(run["csv_path"])), start, oracle_end)
    profile = load_json(Path(str(run["command_profile"]["path"])))["samples"]
    features: dict[str, Any] = {
        "run_id": run["run_id"],
        "label": run["label"],
        "clean_unsafe": 1 if run["label"] == config["labels"]["positive"] else 0,
        "r_deg_s": float(run["point"]["r_deg_s"]),
        "seed": int(run["point"]["seed"]),
        "layer": str(run["point"].get("layer", "")),
        "angle_max_cd": float(run["point"].get("angle_max_cd", 0.0)),
        "outcome": run.get("hardened_oracle_A", {}).get("outcome"),
    }
    features.update(command_features(profile, config))

    rate_rows = [row for row in rows.get("RATE", []) if float(row["_t"]) <= maneuver_end]
    rate_times = [float(row["_t"]) for row in rate_rows]
    dt_rate = mean_dt(rate_times, float(config["feature_extraction"]["csv_time_step_fallback_s"]))
    actual_rate = [math.hypot(frow(row, "R"), frow(row, "P")) for row in rate_rows]
    desired_rate = [math.hypot(frow(row, "RDes"), frow(row, "PDes")) for row in rate_rows]
    features.update(
        {
            "actual_rate_peak_deg_s": max(actual_rate, default=None),
            "actual_rate_rms_deg_s": math.sqrt(sum(v * v for v in actual_rate) / len(actual_rate)) if actual_rate else None,
            "actual_rate_p95_deg_s": percentile(actual_rate, 0.95),
            "actual_rate_impulse_deg": sum(actual_rate) * dt_rate if actual_rate else None,
            "desired_rate_impulse_deg": sum(desired_rate) * dt_rate if desired_rate else None,
            "desired_rate_peak_deg_s": max(desired_rate, default=None),
            "desired_rate_p95_deg_s": percentile(desired_rate, 0.95),
        }
    )
    cmd_p90 = run.get("path_fidelity", {}).get("rate_command_p90_deg_s")
    features["desired_over_command_rate_ratio"] = run.get("path_fidelity", {}).get("rate_desired_over_command")
    features["actual_over_command_rate_ratio"] = run.get("path_fidelity", {}).get("rate_actual_over_command")
    if cmd_p90:
        features["rate_peak_over_command_p90"] = (features["actual_rate_peak_deg_s"] or 0.0) / float(cmd_p90)

    att_rows = rows.get("ATT", [])
    att_times = [float(row["_t"]) for row in att_rows]
    dt_att = mean_dt(att_times, float(config["feature_extraction"]["csv_time_step_fallback_s"]))
    att_errors = []
    roll_series: list[tuple[float, float]] = []
    roll_maneuver = []
    for row in att_rows:
        t = float(row["_t"])
        rel = t - start
        cmd = command_at(profile, rel)
        roll = frow(row, "Roll")
        pitch = frow(row, "Pitch")
        err = math.hypot(roll - float(cmd.get("roll_deg", 0.0)), pitch - float(cmd.get("pitch_deg", 0.0)))
        att_errors.append(err)
        roll_series.append((t, roll))
        if t <= maneuver_end:
            roll_maneuver.append(roll)
    err_threshold = float(config["feature_extraction"]["attitude_error_threshold_deg"])
    features.update(
        {
            "max_attitude_error_deg": max(att_errors, default=None),
            "final_error_deg": run.get("hardened_oracle_A", {}).get("final_error_deg"),
            "attitude_error_impulse_deg_s": sum(att_errors) * dt_att if att_errors else None,
            "attitude_error_high_time_s": sum(1 for v in att_errors if v >= err_threshold) * dt_att if att_errors else None,
            "actual_roll_range_maneuver_deg": (max(roll_maneuver) - min(roll_maneuver)) if roll_maneuver else None,
            "actual_roll_abs_peak_maneuver_deg": max((abs(v) for v in roll_maneuver), default=None),
            "reversal_phase_ratio": reversal_phase_ratio(profile, roll_series, start, config),
        }
    )
    alt = run.get("hardened_oracle_A", {}).get("altitude", {})
    features["altitude_loss_m"] = alt.get("loss_m")
    features["min_altitude_neg_m"] = None if alt.get("min_m") is None else -float(alt["min_m"])
    return features


def feature_metrics(features: list[dict[str, Any]], config: dict[str, Any]) -> list[dict[str, Any]]:
    labels = [int(row["clean_unsafe"]) for row in features]
    names = sorted({k for row in features for k in row if isinstance(row.get(k), (int, float)) and k not in {"clean_unsafe", "seed"}})
    eligible = set(config["candidate_phi"]["eligible_primary"])
    outcome_proxy = set(config["candidate_phi"]["diagnostic_outcome_proxies"])
    metrics = []
    for name in names:
        values = [row.get(name) for row in features]
        raw_auc = auc(values, labels)
        best_auc, direction = oriented_auc(raw_auc)
        rho = spearman(values, labels)
        oriented_rho = None if rho is None else rho * direction
        metrics.append(
            {
                "feature": name,
                "eligible_primary": name in eligible,
                "diagnostic_outcome_proxy": name in outcome_proxy,
                "raw_auc": raw_auc,
                "auc": best_auc,
                "direction": ">=" if direction > 0 else "<=",
                "spearman_rho": rho,
                "oriented_spearman_rho": oriented_rho,
                "threshold": threshold_scan(values, labels, direction),
                "valid_n": sum(1 for v in values if v is not None and isinstance(v, (int, float)) and math.isfinite(float(v))),
            }
        )
    metrics.sort(key=lambda row: (row["eligible_primary"], row["auc"] or 0.0, abs(row["oriented_spearman_rho"] or 0.0)), reverse=True)
    return metrics


def r_shape(features: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_r: dict[float, list[dict[str, Any]]] = defaultdict(list)
    for row in features:
        by_r[float(row["r_deg_s"])].append(row)
    out = []
    for r_value, rows in sorted(by_r.items()):
        n = len(rows)
        clean = sum(int(row["clean_unsafe"]) for row in rows)
        labels = Counter(row["label"] for row in rows)
        out.append(
            {
                "r_deg_s": r_value,
                "n": n,
                "clean_unsafe": clean,
                "p_clean_unsafe": clean / n if n else None,
                "recovered": labels.get("recovered", 0),
                "safe": labels.get("safe", 0),
            }
        )
    return out


def phi_bins(features: list[dict[str, Any]], feature_name: str, bins: int = 8) -> list[dict[str, Any]]:
    rows = [row for row in features if isinstance(row.get(feature_name), (int, float)) and math.isfinite(float(row[feature_name]))]
    rows = sorted(rows, key=lambda row: float(row[feature_name]))
    if not rows:
        return []
    out = []
    for i in range(bins):
        lo = round(i * len(rows) / bins)
        hi = round((i + 1) * len(rows) / bins)
        chunk = rows[lo:hi]
        if not chunk:
            continue
        clean = sum(int(row["clean_unsafe"]) for row in chunk)
        out.append(
            {
                "bin": i,
                "n": len(chunk),
                "phi_min": min(float(row[feature_name]) for row in chunk),
                "phi_max": max(float(row[feature_name]) for row in chunk),
                "phi_mean": statistics.fmean(float(row[feature_name]) for row in chunk),
                "p_clean_unsafe": clean / len(chunk),
            }
        )
    return out


def decide(config: dict[str, Any], metrics: list[dict[str, Any]]) -> dict[str, Any]:
    auc_threshold = float(config["decision"]["auc_threshold"])
    rho_threshold = float(config["decision"]["spearman_abs_threshold"])
    eligible = [row for row in metrics if row["eligible_primary"]]
    best = max(eligible, key=lambda row: (row["auc"] or 0.0, abs(row["oriented_spearman_rho"] or 0.0)))
    ok = bool(
        (best.get("auc") is not None and float(best["auc"]) >= auc_threshold)
        or (best.get("oriented_spearman_rho") is not None and abs(float(best["oriented_spearman_rho"])) >= rho_threshold)
    )
    return {
        "verdict": "LEARNABLE-BOUNDARY" if ok else "COMPLEX-BOUNDARY",
        "reason": (
            f"Best eligible Phi `{best['feature']}` reaches AUC={best['auc']:.3f}; the r-axis baseline is not used."
            if ok
            else "No eligible achieved-trajectory Phi reaches the preregistered monotonicity threshold."
        ),
        "best_phi": best,
    }


def write_preregister(config: dict[str, Any], path: Path) -> dict[str, Any]:
    payload = {
        "status": "preregistered",
        "written_at_utc": utc_now(),
        "experiment": config["experiment"],
        "decision": config["decision"],
        "labels": config["labels"],
        "candidate_phi": config["candidate_phi"],
        "feature_extraction": config["feature_extraction"],
        "confirm_grid_if_needed": config["confirm_grid_if_needed"],
        "reachability_check": config["reachability_check"],
    }
    write_json(path, payload)
    return payload


def make_plots(payload: dict[str, Any]) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    stem = artifact_stem(payload["config"])
    paths: dict[str, str] = {}
    features = payload["features"]
    best = payload["verdict"]["best_phi"]["feature"]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    r_rows = payload["r_shape"]
    ax.plot([row["r_deg_s"] for row in r_rows], [row["p_clean_unsafe"] for row in r_rows], marker="o", color="#6d597a")
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlabel("r (deg/s)")
    ax.set_ylabel("P(clean_unsafe)")
    ax.set_title("Original r axis: non-monotone W-shaped response")
    ax.grid(True, alpha=0.25)
    path = analysis / f"{stem}_r_w_shape.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["r_w_shape"] = str(path)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    xs = [float(row[best]) for row in features if isinstance(row.get(best), (int, float))]
    ys = [0.08 * (int(row["seed"]) % 10) + int(row["clean_unsafe"]) for row in features if isinstance(row.get(best), (int, float))]
    colors = ["#e76f51" if int(row["clean_unsafe"]) else "#2a9d8f" for row in features if isinstance(row.get(best), (int, float))]
    ax.scatter(xs, ys, color=colors, alpha=0.75, s=35)
    thr = payload["verdict"]["best_phi"]["threshold"].get("threshold")
    if thr is not None:
        ax.axvline(float(thr), color="#222222", linestyle="--", linewidth=1)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["recovered/safe", "clean_unsafe"])
    ax.set_xlabel(best)
    ax.set_title("Best Phi ordering of clean unsafe outcomes")
    ax.grid(True, axis="x", alpha=0.25)
    path = analysis / f"{stem}_phi_ordering.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["phi_ordering"] = str(path)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    bins = payload["phi_bins"]
    ax.plot([row["phi_mean"] for row in bins], [row["p_clean_unsafe"] for row in bins], marker="o", color="#264653")
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlabel(best)
    ax.set_ylabel("binned P(clean_unsafe)")
    ax.set_title("Phi-binned monotonicity diagnostic")
    ax.grid(True, alpha=0.25)
    path = analysis / f"{stem}_phi_monotonicity.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["phi_monotonicity"] = str(path)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    top = [row for row in payload["feature_metrics"] if row["eligible_primary"]][:10]
    ax.barh([row["feature"] for row in reversed(top)], [row["auc"] or 0.0 for row in reversed(top)], color="#2a9d8f")
    ax.axvline(float(payload["config"]["decision"]["auc_threshold"]), color="#222222", linestyle="--", linewidth=1)
    ax.set_xlim(0.45, 1.0)
    ax.set_xlabel("oriented AUC")
    ax.set_title("Candidate Phi ranking")
    path = analysis / f"{stem}_feature_auc.png"
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    paths["feature_auc"] = str(path)
    return paths


def build_report(payload: dict[str, Any]) -> str:
    stem = artifact_stem(payload["config"])
    path = PLANC_ROOT / "results" / f"{stem}_report.md"
    verdict = payload["verdict"]
    best = verdict["best_phi"]
    r_metric = next(row for row in payload["feature_metrics"] if row["feature"] == "r_deg_s")
    lines = [
        "# Demand v1 Reparameterization Probe",
        "",
        f"VERDICT: **{verdict['verdict']}**",
        "",
        verdict["reason"],
        "",
        "## Best Phi",
        "",
        f"- Phi: `{best['feature']}`.",
        f"- AUC: {fmt(best['auc'], 3)}; Spearman rho: {fmt(best['oriented_spearman_rho'], 3)}.",
        f"- Threshold Phi*: `{best['threshold']['direction']} {fmt(best['threshold']['threshold'], 2)}`; balanced accuracy {fmt(best['threshold']['balanced_accuracy'], 3)}.",
        f"- Baseline r AUC: {fmt(r_metric['auc'], 3)}; r Spearman rho: {fmt(r_metric['oriented_spearman_rho'], 3)}.",
        "",
        "## R Axis Shape",
        "",
        "| r | n | clean_unsafe | recovered | safe | p_clean |",
        "| ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in payload["r_shape"]:
        lines.append(
            f"| {fmt(row['r_deg_s'], 0)} | {row['n']} | {row['clean_unsafe']} | {row['recovered']} | {row['safe']} | {fmt(row['p_clean_unsafe'], 2)} |"
        )
    lines.extend(
        [
            "",
            "## Phi Bins",
            "",
            "| bin | Phi min | Phi max | n | p_clean |",
            "| ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in payload["phi_bins"]:
        lines.append(
            f"| {row['bin']} | {fmt(row['phi_min'], 2)} | {fmt(row['phi_max'], 2)} | {row['n']} | {fmt(row['p_clean_unsafe'], 2)} |"
        )
    lines.extend(
        [
            "",
            "## Top Candidate Phi",
            "",
            "| feature | eligible | AUC | rho | threshold |",
            "| --- | --- | ---: | ---: | --- |",
        ]
    )
    for row in payload["feature_metrics"][:12]:
        lines.append(
            f"| {row['feature']} | {row['eligible_primary']} | {fmt(row['auc'], 3)} | {fmt(row['oriented_spearman_rho'], 3)} | {row['threshold'].get('direction')} {fmt(row['threshold'].get('threshold'), 2)} |"
        )
    lines.extend(
        [
            "",
            "## E/P And Reachability",
            "",
            "- New E/P SITL confirmation grid was not run in this analysis-short-circuit pass.",
            "- Available hard-A evidence covers only default turbulence and ANGLE_MAX=4500.",
            "- Because no eligible Phi reaches the monotonicity threshold, Phi* is not defined well enough for an E/P movement test in this branch.",
            "- The preregistered E/P grid and non-doublet reachability inputs are still recorded in `demand_v1_prereg.json` if a later Phi family is proposed.",
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=PLANC_ROOT / "config" / "demand_v1_config.yaml")
    args = parser.parse_args()

    config = load_yaml(args.config)
    stem = artifact_stem(config)
    results_dir = PLANC_ROOT / "results"
    prereg_path = results_dir / f"{stem}_prereg.json"
    result_path = results_dir / f"{stem}_result.json"
    env_path = results_dir / f"env_{stem}.json"

    oracleA = load_json(REPO_ROOT / config["experiment"]["source_oracleA_result"])
    env_config = {"sitl": oracleA["config"]["sitl"], "experiment": {"speedup": oracleA["config"]["experiment"].get("speedup")}, "param_metadata": oracleA["config"].get("param_metadata", {})}
    env = probe_environment(env_config, REPO_ROOT)
    write_env(env, env_path)
    prereg = write_preregister(config, prereg_path)

    source_runs = [run for run in oracleA["runs"] if not run.get("error") and run.get("label") in [config["labels"]["positive"], *config["labels"]["negative"]]]
    features = [extract_features(run, config) for run in source_runs]
    metrics = feature_metrics(features, config)
    verdict = decide(config, metrics)
    best_name = verdict["best_phi"]["feature"]
    payload = {
        "status": "complete",
        "generated_at_utc": utc_now(),
        "config": config,
        "env": env,
        "preregistration": prereg,
        "source_oracleA_summary": oracleA.get("summary", {}),
        "verdict": verdict,
        "analysis_scope": {
            "source_runs": len(source_runs),
            "new_sitl_runs": 0,
            "short_circuit_used": bool(config["experiment"]["analysis_short_circuit"]),
            "ep_confirm_grid_status": "not_run",
            "reachability_status": "not_run",
        },
        "r_shape": r_shape(features),
        "phi_bins": phi_bins(features, best_name),
        "feature_metrics": metrics,
        "features": features,
        "artifacts": {},
    }
    payload["artifacts"]["plots"] = make_plots(payload)
    payload["artifacts"]["report"] = build_report(payload)
    write_json(result_path, payload)
    print(f"COMPLETE: verdict={verdict['verdict']} best_phi={best_name} result={result_path} report={payload['artifacts']['report']}", flush=True)


if __name__ == "__main__":
    main()
