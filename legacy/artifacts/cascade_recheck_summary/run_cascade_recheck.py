from __future__ import annotations

import math
import random
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from cadet.query import read_parsed_log  # noqa: E402


OUTPUT_DIR = Path("artifacts/cascade_recheck_summary")
PLATFORM = "PX4"
SCENARIO = "px4_position"
SCOPE_LABEL = "exploratory / verification"
PROPERTY = "post_neutral_xy_velocity"
REPEATS = 5
SIGMA_MULTIPLIER = 2.0
TERMINAL_WINDOW = (11.0, 13.0)
PROFILE_WINDOWS = [(5.0, 7.0), (7.0, 9.0), (9.0, 11.0), (11.0, 13.0)]
SPOTCHECK_SEED = 20260608
SEEDS = [0, 1, 2]

DIRECTION_A_ARTIFACTS = {
    0: Path("artifacts/direction_a_px4_position_seed0_v0"),
    1: Path("artifacts/direction_a_px4_position_seed1_v0"),
    2: Path("artifacts/direction_a_px4_position_seed2_v0"),
}
DIRECTION_A_RUNS = {
    0: Path("runs/direction_a_px4_position_seed0_v0"),
    1: Path("runs/direction_a_px4_position_seed1_v0"),
    2: Path("runs/direction_a_px4_position_seed2_v0"),
}
DDMIN_ARTIFACTS = {
    0: Path("artifacts/direction_a_ddmin_px4_position_seed0_v1"),
    1: Path("artifacts/direction_a_ddmin_px4_position_seed1_v0"),
    2: Path("artifacts/direction_a_ddmin_px4_position_seed2_v0"),
}
DDMIN_RUNS = {
    0: Path("runs/direction_a_ddmin_px4_position_seed0_v1"),
    1: Path("runs/direction_a_ddmin_px4_position_seed1_v0"),
    2: Path("runs/direction_a_ddmin_px4_position_seed2_v0"),
}


def main() -> None:
    start = time.monotonic()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    threshold = _grid_c1_xy_threshold()
    point_frames, query_frames, final_frames = _load_archived_frames()
    final_lookup = _ddmin_final_lookup(final_frames)

    candidates, safe_rows = _enumerate_candidates(point_frames)
    spotcheck_specs = _spotcheck_specs(safe_rows)
    seed12_specs = _seed12_decay_specs(point_frames)
    clean_final_specs = _clean_final_specs(final_frames)
    for spec in candidates:
        spec["audit_scope"] = "candidate"
    for spec in spotcheck_specs:
        spec["audit_scope"] = "spotcheck_safe"
    for spec in clean_final_specs:
        spec["audit_scope"] = "ddmin_clean_final"

    all_metric_specs = candidates + spotcheck_specs + clean_final_specs
    recovery_rows = [_recovery_row(spec, query_frames) for spec in all_metric_specs]
    recovery_df = pd.DataFrame(recovery_rows)

    flagged_specs = candidates
    flagged_rows = []
    per_repeat_cache: dict[str, dict[str, Any]] = {}
    read_counter = {"logs": 0}
    for spec in flagged_specs:
        row = _evaluate_spec(spec, query_frames, threshold, per_repeat_cache, read_counter)
        row.update(final_lookup.get((int(spec["seed"]), str(spec["theta_hash"])), {}))
        flagged_rows.append(row)
    flagged = pd.DataFrame(flagged_rows)
    flagged = _ordered_columns(flagged)

    survivors = flagged[flagged["terminal_label"] == "robust_violation"].copy()

    seed12_decay = pd.DataFrame(
        [
            _evaluate_spec(spec, query_frames, threshold, per_repeat_cache, read_counter)
            | {"selection": "seed1_seed2_top3_most_negative_direction_a_robust_violation"}
            for spec in seed12_specs
        ]
    )
    seed12_decay = _ordered_columns(seed12_decay)

    spotcheck_safe = pd.DataFrame(
        [
            _evaluate_spec(spec, query_frames, threshold, per_repeat_cache, read_counter)
            | {"spotcheck_sample_seed": SPOTCHECK_SEED}
            for spec in spotcheck_specs
        ]
    )
    spotcheck_safe = _ordered_columns(spotcheck_safe)

    clean_final_rows = [
        _evaluate_spec(spec, query_frames, threshold, per_repeat_cache, read_counter)
        | {"ddmin_final_is_clean": True}
        for spec in clean_final_specs
    ]
    clean_final_df = pd.DataFrame(clean_final_rows)
    if not clean_final_df.empty:
        clean_final_df = _ordered_columns(clean_final_df)

    flagged.to_csv(OUTPUT_DIR / "flagged_points.csv", index=False)
    survivors.to_csv(OUTPUT_DIR / "survivors.csv", index=False)
    seed12_decay.to_csv(OUTPUT_DIR / "seed12_decay.csv", index=False)
    spotcheck_safe.to_csv(OUTPUT_DIR / "spotcheck_safe.csv", index=False)
    clean_final_df.to_csv(OUTPUT_DIR / "ddmin_clean_finals.csv", index=False)
    recovery_df.to_csv(OUTPUT_DIR / "recoverability_audit.csv", index=False)

    elapsed = time.monotonic() - start
    report = _write_report(
        flagged=flagged,
        survivors=survivors,
        seed12_decay=seed12_decay,
        spotcheck_safe=spotcheck_safe,
        clean_final_df=clean_final_df,
        recovery_df=recovery_df,
        read_logs=read_counter["logs"],
        elapsed_s=elapsed,
    )
    print(f"cascade_recheck_complete report={report} elapsed_s={elapsed:.3f}")


def _load_archived_frames() -> tuple[dict[tuple[str, int], pd.DataFrame], dict[tuple[str, int], pd.DataFrame], dict[int, pd.DataFrame]]:
    point_frames: dict[tuple[str, int], pd.DataFrame] = {}
    query_frames: dict[tuple[str, int], pd.DataFrame] = {}
    final_frames: dict[int, pd.DataFrame] = {}
    for seed in SEEDS:
        direction_root = DIRECTION_A_ARTIFACTS[seed]
        ddmin_root = DDMIN_ARTIFACTS[seed]
        point_frames[("direction_a", seed)] = pd.read_csv(direction_root / "reports" / "point_evaluations.csv")
        query_frames[("direction_a", seed)] = pd.read_csv(direction_root / "reports" / "query_repeats.csv")
        point_frames[("ddmin", seed)] = pd.read_csv(ddmin_root / "reports" / "ddmin_point_evaluations.csv")
        query_frames[("ddmin", seed)] = pd.read_csv(ddmin_root / "reports" / "ddmin_query_repeats.csv")
        final_frames[seed] = pd.read_csv(ddmin_root / "reports" / "minimized_triggers.csv")
    return point_frames, query_frames, final_frames


def _enumerate_candidates(point_frames: dict[tuple[str, int], pd.DataFrame]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    candidates: list[dict[str, Any]] = []
    safe_rows: list[dict[str, Any]] = []
    for seed in SEEDS:
        direction = point_frames[("direction_a", seed)]
        for _, row in direction.iterrows():
            spec = _direction_a_spec(seed, row)
            if spec["orig_label"] in {"robust_violation", "noise_band"}:
                candidates.append(spec)
            elif spec["orig_label"] == "robust_safe":
                safe_rows.append(spec)

        ddmin = point_frames[("ddmin", seed)]
        for _, row in ddmin.iterrows():
            spec = _ddmin_spec(seed, row)
            if spec["orig_label"] in {"robust_violation", "noise_band"}:
                candidates.append(spec)
            elif spec["orig_label"] == "robust_safe":
                safe_rows.append(spec)
    return candidates, safe_rows


def _direction_a_spec(seed: int, row: pd.Series) -> dict[str, Any]:
    return {
        "source": "direction_a",
        "seed": int(seed),
        "arm": str(row["arm"]),
        "eval_id": int(row["eval_id"]),
        "trigger_id": math.nan,
        "theta_hash": str(row["theta_hash"]),
        "theta_path": str(row["theta_path"]),
        "stage": str(row["stage"]),
        "label": str(row["label"]),
        "orig_label": str(row["robustness_class"]),
        "orig_rho_mean": float(row[f"rho_mean_{PROPERTY}"]),
        "orig_rho_std": float(row[f"rho_std_{PROPERTY}"]),
        "orig_rho_min": float(row[f"rho_min_{PROPERTY}"]),
        "orig_rho_max": float(row[f"rho_max_{PROPERTY}"]),
        "source_kind": "direction_a_three_arm",
    }


def _ddmin_spec(seed: int, row: pd.Series) -> dict[str, Any]:
    return {
        "source": "ddmin",
        "seed": int(seed),
        "arm": "ddmin",
        "eval_id": int(row["eval_id"]),
        "trigger_id": int(row["trigger_id"]),
        "theta_hash": str(row["theta_hash"]),
        "theta_path": str(row["theta_path"]),
        "stage": str(row["phase"]),
        "label": str(row["label"]),
        "orig_label": str(row["robustness_class"]),
        "orig_rho_mean": float(row[f"rho_mean_{PROPERTY}"]),
        "orig_rho_std": float(row[f"rho_std_{PROPERTY}"]),
        "orig_rho_min": float(row[f"rho_min_{PROPERTY}"]),
        "orig_rho_max": float(row[f"rho_max_{PROPERTY}"]),
        "source_kind": "ddmin_point_evaluation",
    }


def _spotcheck_specs(safe_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    direction_safe = [row for row in safe_rows if row["source"] == "direction_a"]
    rng = random.Random(SPOTCHECK_SEED)
    return rng.sample(direction_safe, 10)


def _seed12_decay_specs(point_frames: dict[tuple[str, int], pd.DataFrame]) -> list[dict[str, Any]]:
    specs: list[dict[str, Any]] = []
    for seed in [1, 2]:
        df = point_frames[("direction_a", seed)]
        robust = df[df["robustness_class"] == "robust_violation"].copy()
        robust = robust.sort_values(f"rho_mean_{PROPERTY}", ascending=True).head(3)
        for _, row in robust.iterrows():
            specs.append(_direction_a_spec(seed, row))
    return specs


def _clean_final_specs(final_frames: dict[int, pd.DataFrame]) -> list[dict[str, Any]]:
    specs: list[dict[str, Any]] = []
    for seed, frame in final_frames.items():
        clean = frame[frame["is_clean"] == True].copy()  # noqa: E712
        for _, row in clean.iterrows():
            specs.append(
                {
                    "source": "ddmin_clean_final",
                    "seed": int(seed),
                    "arm": "ddmin_clean_final",
                    "eval_id": math.nan,
                    "trigger_id": int(row["trigger_id"]),
                    "theta_hash": str(row["final_theta_hash"]),
                    "theta_path": str(row["final_theta_path"]),
                    "stage": "minimized_final",
                    "label": f"T{int(row['trigger_id']):02d}_final",
                    "orig_label": str(row["final_robustness_class"]),
                    "orig_rho_mean": float(row[f"final_rho_mean_{PROPERTY}"]),
                    "orig_rho_std": float(row[f"final_rho_std_{PROPERTY}"]),
                    "orig_rho_min": math.nan,
                    "orig_rho_max": math.nan,
                    "source_kind": "ddmin_clean_minimized_output",
                }
            )
    return specs


def _ddmin_final_lookup(final_frames: dict[int, pd.DataFrame]) -> dict[tuple[int, str], dict[str, Any]]:
    lookup: dict[tuple[int, str], dict[str, Any]] = {}
    for seed, frame in final_frames.items():
        for _, row in frame.iterrows():
            key = (int(seed), str(row["final_theta_hash"]))
            lookup[key] = {
                "ddmin_final_trigger_id": int(row["trigger_id"]),
                "ddmin_final_is_clean": bool(row["is_clean"]),
                "ddmin_final_support_size_abs_gt_0p1": int(row["final_support_size_abs_gt_0p1"]),
                "ddmin_final_active_channels_abs_gt_0p1": str(row["final_active_channels_abs_gt_0p1"]),
            }
    return lookup


def _recovery_row(spec: dict[str, Any], query_frames: dict[tuple[str, int], pd.DataFrame]) -> dict[str, Any]:
    rows = _query_rows_for_spec(spec, query_frames)
    parsed_paths = []
    raw_ulg_paths = []
    for _, query_row in rows.iterrows():
        parsed = _find_parsed_log(str(query_row["query_id"]), _roots_for_spec(spec))
        if parsed:
            parsed_paths.append(str(parsed))
            continue
        raw = _find_raw_ulg(str(query_row["query_id"]), _roots_for_spec(spec))
        if raw:
            raw_ulg_paths.append(str(raw))
    return {
        "scope_label": SCOPE_LABEL,
        "platform": PLATFORM,
        "scenario": SCENARIO,
        "audit_scope": spec.get("audit_scope", ""),
        "source": spec["source"],
        "seed": spec["seed"],
        "arm": spec["arm"],
        "eval_id": spec["eval_id"],
        "trigger_id": spec["trigger_id"],
        "theta_hash": spec["theta_hash"],
        "orig_label": spec["orig_label"],
        "expected_repeats": REPEATS,
        "query_repeat_rows": int(len(rows)),
        "parsed_log_repeats": int(len(parsed_paths)),
        "raw_ulg_repeats_without_parsed": int(len(raw_ulg_paths)),
        "recoverable_from_parsed_logs": bool(len(parsed_paths) == REPEATS),
        "recoverable_from_existing_logs": bool(len(parsed_paths) == REPEATS or len(parsed_paths) + len(raw_ulg_paths) == REPEATS),
        "needs_resimulation": bool(len(parsed_paths) + len(raw_ulg_paths) < REPEATS),
    }


def _evaluate_spec(
    spec: dict[str, Any],
    query_frames: dict[tuple[str, int], pd.DataFrame],
    threshold: float,
    per_repeat_cache: dict[str, dict[str, Any]],
    read_counter: dict[str, int],
) -> dict[str, Any]:
    rows = _query_rows_for_spec(spec, query_frames)
    if len(rows) != REPEATS:
        raise RuntimeError(f"Expected {REPEATS} query rows for {spec['source']} seed={spec['seed']} hash={spec['theta_hash']}; got {len(rows)}")

    per_repeat = []
    for _, query_row in rows.iterrows():
        query_id = str(query_row["query_id"])
        parsed_path = _find_parsed_log(query_id, _roots_for_spec(spec))
        if parsed_path is None:
            raise FileNotFoundError(f"Missing parsed log for query_id={query_id}")
        if query_id not in per_repeat_cache:
            parsed_log = read_parsed_log(parsed_path)
            per_repeat_cache[query_id] = _per_repeat_metrics(parsed_log)
            read_counter["logs"] += 1
        item = dict(per_repeat_cache[query_id])
        item["query_id"] = query_id
        item["repeat_idx"] = int(query_row["repeat_idx"])
        item["parsed_log_path"] = str(parsed_path)
        per_repeat.append(item)

    terminal_peaks = [float(item["peak_11_13"]) for item in per_repeat]
    terminal_rhos = [threshold - value for value in terminal_peaks]
    peak_stats = _stats(terminal_peaks)
    rho_stats = _stats(terminal_rhos)
    profile_stats = {key: _stats([float(item[f"peak_{key}"]) for item in per_repeat]) for key in ["5_7", "7_9", "9_11", "11_13"]}

    row = {
        "scope_label": SCOPE_LABEL,
        "platform": PLATFORM,
        "scenario": SCENARIO,
        "source": spec["source"],
        "source_kind": spec["source_kind"],
        "seed": int(spec["seed"]),
        "arm": spec["arm"],
        "eval_id": spec["eval_id"],
        "trigger_id": spec["trigger_id"],
        "theta_hash": spec["theta_hash"],
        "theta_path": spec["theta_path"],
        "stage": spec["stage"],
        "label": spec["label"],
        "J": REPEATS,
        "orig_metric_window": "[5,13]s tail from t_neutral_s to log end",
        "orig_metric_property": PROPERTY,
        "orig_label": spec["orig_label"],
        "orig_rho_mean": spec["orig_rho_mean"],
        "orig_rho_std": spec["orig_rho_std"],
        "orig_rho_min": spec["orig_rho_min"],
        "orig_rho_max": spec["orig_rho_max"],
        "terminal_metric_window": "[11,13]s absolute",
        "terminal_metric": "per-repeat peak |v_xy|, rho = 1.0 - peak, J=5 mean/std, 2sigma label",
        "terminal_threshold_mps": threshold,
        "terminal_peak_11_13_mean": peak_stats["mean"],
        "terminal_peak_11_13_std": peak_stats["std"],
        "terminal_peak_11_13_min": peak_stats["min"],
        "terminal_peak_11_13_max": peak_stats["max"],
        "terminal_rho_mean": rho_stats["mean"],
        "terminal_rho_std": rho_stats["std"],
        "terminal_rho_min": rho_stats["min"],
        "terminal_rho_max": rho_stats["max"],
        "terminal_label": classify_robustness(rho_stats["mean"], rho_stats["std"]),
        "query_ids": ";".join(item["query_id"] for item in sorted(per_repeat, key=lambda x: x["repeat_idx"])),
        "parsed_log_paths": ";".join(item["parsed_log_path"] for item in sorted(per_repeat, key=lambda x: x["repeat_idx"])),
    }
    for key, stats in profile_stats.items():
        row[f"peak_{key}_mean"] = stats["mean"]
        row[f"peak_{key}_std"] = stats["std"]
        row[f"peak_{key}_min"] = stats["min"]
        row[f"peak_{key}_max"] = stats["max"]
    return row


def _query_rows_for_spec(spec: dict[str, Any], query_frames: dict[tuple[str, int], pd.DataFrame]) -> pd.DataFrame:
    source = "ddmin" if str(spec["source"]).startswith("ddmin") else "direction_a"
    frame = query_frames[(source, int(spec["seed"]))]
    if _is_nan(spec.get("eval_id")):
        rows = frame[frame["theta_hash"].astype(str) == str(spec["theta_hash"])].copy()
    else:
        rows = frame[
            (frame["eval_id"].astype(int) == int(spec["eval_id"]))
            & (frame["theta_hash"].astype(str) == str(spec["theta_hash"]))
        ].copy()
    return rows.sort_values("repeat_idx")


def _roots_for_spec(spec: dict[str, Any]) -> list[Path]:
    seed = int(spec["seed"])
    if str(spec["source"]).startswith("ddmin"):
        return [DDMIN_ARTIFACTS[seed], DDMIN_RUNS[seed]]
    return [DIRECTION_A_ARTIFACTS[seed], DIRECTION_A_RUNS[seed]]


def _find_parsed_log(query_id: str, roots: list[Path]) -> Path | None:
    for root in roots:
        for filename in ["parsed_log.parquet", "parsed_log.csv"]:
            path = root / "queries" / query_id / filename
            if path.exists():
                return path
    return None


def _find_raw_ulg(query_id: str, roots: list[Path]) -> Path | None:
    for root in roots:
        path = root / "queries" / query_id / "raw_log.ulg"
        if path.exists():
            return path
    return None


def _per_repeat_metrics(parsed_log: pd.DataFrame) -> dict[str, float]:
    return {
        "peak_5_7": _speed_peak(parsed_log, (5.0, 7.0)),
        "peak_7_9": _speed_peak(parsed_log, (7.0, 9.0)),
        "peak_9_11": _speed_peak(parsed_log, (9.0, 11.0)),
        "peak_11_13": _speed_peak(parsed_log, (11.0, 13.0)),
    }


def _speed_peak(parsed_log: pd.DataFrame, window: tuple[float, float]) -> float:
    times = parsed_log["time_s"].to_numpy(dtype=float)
    speed = np.hypot(parsed_log["vx_mps"].to_numpy(dtype=float), parsed_log["vy_mps"].to_numpy(dtype=float))
    lo, hi = window
    mask = (times >= float(lo)) & (times <= float(hi))
    if not np.any(mask):
        raise ValueError(f"No telemetry samples in window [{lo}, {hi}]")
    return float(np.max(speed[mask]))


def _stats(values: list[float]) -> dict[str, float]:
    arr = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


def classify_robustness(mean: float, std: float) -> str:
    if float(mean) + SIGMA_MULTIPLIER * float(std) < 0.0:
        return "robust_violation"
    if float(mean) - SIGMA_MULTIPLIER * float(std) > 0.0:
        return "robust_safe"
    return "noise_band"


def _grid_c1_xy_threshold() -> float:
    params = pd.read_csv("artifacts/contract_grid_summary/params_used.csv")
    row = params[(params["contract"] == "C1 Brake") & (params["measured_axis"] == "xy_velocity")].iloc[0]
    return float(row["derived_threshold"])


def _ordered_columns(frame: pd.DataFrame) -> pd.DataFrame:
    preferred = [
        "scope_label",
        "platform",
        "scenario",
        "source",
        "source_kind",
        "seed",
        "arm",
        "eval_id",
        "trigger_id",
        "theta_hash",
        "theta_path",
        "stage",
        "label",
        "J",
        "orig_label",
        "orig_rho_mean",
        "orig_rho_std",
        "orig_rho_min",
        "orig_rho_max",
        "terminal_label",
        "terminal_rho_mean",
        "terminal_rho_std",
        "terminal_rho_min",
        "terminal_rho_max",
        "terminal_peak_11_13_mean",
        "terminal_peak_11_13_std",
        "peak_5_7_mean",
        "peak_5_7_std",
        "peak_7_9_mean",
        "peak_7_9_std",
        "peak_9_11_mean",
        "peak_9_11_std",
        "peak_11_13_mean",
        "peak_11_13_std",
    ]
    columns = [col for col in preferred if col in frame.columns]
    columns.extend([col for col in frame.columns if col not in columns])
    return frame.loc[:, columns]


def _write_report(
    *,
    flagged: pd.DataFrame,
    survivors: pd.DataFrame,
    seed12_decay: pd.DataFrame,
    spotcheck_safe: pd.DataFrame,
    clean_final_df: pd.DataFrame,
    recovery_df: pd.DataFrame,
    read_logs: int,
    elapsed_s: float,
) -> Path:
    report_path = OUTPUT_DIR / "cascade_recheck_report.md"
    candidate_counts = _candidate_count_table(flagged)
    summary_counts = _summary_count_table(flagged)
    recovery_counts = _recovery_count_table(recovery_df)
    clean_counts = _clean_final_count_table(clean_final_df)
    postprocess_per_log = elapsed_s / max(read_logs, 1)
    needs_resim = int(recovery_df["needs_resimulation"].sum())
    recoverable = int(recovery_df["recoverable_from_parsed_logs"].sum())
    total_audit = int(len(recovery_df))
    candidate_audit = recovery_df[recovery_df["audit_scope"] == "candidate"].copy()
    extra_audit = recovery_df[recovery_df["audit_scope"] != "candidate"].copy()
    candidate_recoverable = int(candidate_audit["recoverable_from_parsed_logs"].sum())
    candidate_total = int(len(candidate_audit))
    extra_recoverable = int(extra_audit["recoverable_from_parsed_logs"].sum())
    extra_total = int(len(extra_audit))
    resim_est_s = 19.0 * REPEATS * needs_resim

    lines = [
        "# Cascade Recheck Report",
        "",
        f"Label: {SCOPE_LABEL}. Platform: {PLATFORM}. Scenario: {SCENARIO}. Seeds: 0/1/2. J={REPEATS}.",
        "",
        "## Metric Scope",
        "",
        "- Original Direction A/ddmin labels are the archived `post_neutral_xy_velocity` 2sigma labels.",
        "- Terminal metric is absolute [11,13] s peak |v_xy| with threshold 1.0 m/s: rho = 1.0 - peak.",
        "- Four profile windows use the same `_speed_peak(parsed_log, window)` implementation as `artifacts/monotonicity_check_summary/run_monotonicity_check.py`.",
        "",
        "## Step 1: Monotonic Window Filter",
        "",
        "For each repeat, `[11,13]` is a subwindow of the original `[5,13]` tail, so `peak|v_xy|[11,13] <= peak|v_xy|[5,13]` and `rho_terminal >= rho_orig`. By this filter, archived `robust_safe` points were not exhaustively rejudged; only 10 were spot-checked.",
        "",
        "Candidate set for exhaustive rejudgment is archived `robust_violation` plus `noise_band`:",
        "",
        _markdown_table(candidate_counts),
        "",
        "## Step 2: Recoverability",
        "",
        f"- Exhaustive candidate points recoverable from parsed logs: {candidate_recoverable}/{candidate_total}.",
        f"- Extra verification objects recoverable from parsed logs (safe spotcheck + ddmin clean finals): {extra_recoverable}/{extra_total}.",
        f"- Parsed-log recoverable points audited: {recoverable}/{total_audit}.",
        f"- Needs resimulation: {needs_resim}.",
        f"- Estimated postprocessing time: {read_logs} parsed logs x {postprocess_per_log:.4f}s/log = {elapsed_s:.2f}s measured wall time.",
        f"- Estimated resimulation time: 19s x {REPEATS} repeats x {needs_resim} points = {resim_est_s:.1f}s.",
        "",
        _markdown_table(recovery_counts),
        "",
        "## Step 3/6: Terminal Rejudgment Counts",
        "",
        _markdown_table(summary_counts),
        "",
        "Headline question: any arm/ddmin/seed retaining terminal-window robust violations?",
        "",
        _headline_lines(survivors),
        "",
        "## Step 4: ddmin Clean Finals",
        "",
        _markdown_table(clean_counts),
        "",
        "## Step 5: Seed 1/2 Decay Profiles",
        "",
        _markdown_table(
            seed12_decay[
                [
                    "seed",
                    "arm",
                    "eval_id",
                    "theta_hash",
                    "orig_rho_mean",
                    "orig_label",
                    "peak_5_7_mean",
                    "peak_7_9_mean",
                    "peak_9_11_mean",
                    "peak_11_13_mean",
                    "terminal_rho_mean",
                    "terminal_label",
                ]
            ]
        ),
        "",
        "## Step 5: Safe Spotcheck",
        "",
        _markdown_table(
            spotcheck_safe[
                [
                    "seed",
                    "arm",
                    "eval_id",
                    "theta_hash",
                    "orig_rho_mean",
                    "orig_label",
                    "terminal_rho_mean",
                    "terminal_rho_std",
                    "terminal_label",
                ]
            ]
        ),
        "",
        "## Outputs",
        "",
        "- `flagged_points.csv`: archived non-safe Direction A/ddmin points rejudged with terminal and profile metrics.",
        "- `survivors.csv`: subset of `flagged_points.csv` with terminal `robust_violation`.",
        "- `seed12_decay.csv`: seed 1/2 top-three archived Direction A robust violations by original rho, with four-window profiles.",
        "- `spotcheck_safe.csv`: 10 archived Direction A robust-safe points rejudged under [11,13].",
        "- `ddmin_clean_finals.csv`: ddmin clean minimized outputs rejudged under [11,13].",
        "- `recoverability_audit.csv`: parsed-log availability audit.",
        "",
        "Archive hygiene: this artifact does not copy or retain `*.ulg` files.",
    ]
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report_path


def _candidate_count_table(flagged: pd.DataFrame) -> pd.DataFrame:
    table = (
        flagged.groupby(["source", "seed", "arm", "orig_label"])
        .size()
        .unstack(fill_value=0)
        .reset_index()
    )
    for col in ["robust_violation", "noise_band"]:
        if col not in table.columns:
            table[col] = 0
    table["candidate_total"] = table["robust_violation"] + table["noise_band"]
    return table[["source", "seed", "arm", "robust_violation", "noise_band", "candidate_total"]].sort_values(["source", "seed", "arm"])


def _summary_count_table(flagged: pd.DataFrame) -> pd.DataFrame:
    grouped = flagged.groupby(["source", "seed", "arm"])
    rows = []
    for (source, seed, arm), group in grouped:
        rows.append(
            {
                "source": source,
                "seed": seed,
                "arm": arm,
                "orig_robust_violation_count": int((group["orig_label"] == "robust_violation").sum()),
                "orig_noise_band_count": int((group["orig_label"] == "noise_band").sum()),
                "terminal_robust_violation_count": int((group["terminal_label"] == "robust_violation").sum()),
                "terminal_noise_band_count": int((group["terminal_label"] == "noise_band").sum()),
                "terminal_robust_safe_count": int((group["terminal_label"] == "robust_safe").sum()),
            }
        )
    return pd.DataFrame(rows).sort_values(["source", "seed", "arm"])


def _recovery_count_table(recovery_df: pd.DataFrame) -> pd.DataFrame:
    grouped = recovery_df.groupby(["audit_scope", "source", "seed", "arm"])
    rows = []
    for (audit_scope, source, seed, arm), group in grouped:
        rows.append(
            {
                "audit_scope": audit_scope,
                "source": source,
                "seed": seed,
                "arm": arm,
                "points": int(len(group)),
                "parsed_recoverable": int(group["recoverable_from_parsed_logs"].sum()),
                "needs_resimulation": int(group["needs_resimulation"].sum()),
            }
        )
    return pd.DataFrame(rows).sort_values(["audit_scope", "source", "seed", "arm"])


def _clean_final_count_table(clean_final_df: pd.DataFrame) -> pd.DataFrame:
    if clean_final_df.empty:
        return pd.DataFrame(columns=["seed", "clean_finals", "orig_robust_violation_count", "terminal_robust_violation_count", "terminal_noise_band_count", "terminal_robust_safe_count"])
    rows = []
    for seed, group in clean_final_df.groupby("seed"):
        rows.append(
            {
                "seed": int(seed),
                "clean_finals": int(len(group)),
                "orig_robust_violation_count": int((group["orig_label"] == "robust_violation").sum()),
                "terminal_robust_violation_count": int((group["terminal_label"] == "robust_violation").sum()),
                "terminal_noise_band_count": int((group["terminal_label"] == "noise_band").sum()),
                "terminal_robust_safe_count": int((group["terminal_label"] == "robust_safe").sum()),
            }
        )
    return pd.DataFrame(rows).sort_values("seed")


def _headline_lines(survivors: pd.DataFrame) -> str:
    if survivors.empty:
        return "No terminal-window robust violations survived in `flagged_points.csv`."
    compact = survivors[
        [
            "source",
            "seed",
            "arm",
            "eval_id",
            "trigger_id",
            "theta_hash",
            "orig_rho_mean",
            "terminal_rho_mean",
            "terminal_rho_std",
            "terminal_label",
        ]
    ].sort_values(["source", "seed", "arm", "eval_id", "trigger_id"])
    return _markdown_table(compact)


def _markdown_table(frame: pd.DataFrame) -> str:
    if frame.empty:
        return "(empty)"
    display = frame.copy()
    for column in display.columns:
        if pd.api.types.is_float_dtype(display[column]):
            display[column] = display[column].map(lambda value: "" if pd.isna(value) else f"{float(value):.6f}")
        else:
            display[column] = display[column].map(lambda value: "" if pd.isna(value) else str(value))
    headers = [str(column) for column in display.columns]
    rows = display.astype(str).values.tolist()
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(value).replace("|", "<bar>") for value in row) + " |")
    return "\n".join(lines)


def _is_nan(value: Any) -> bool:
    try:
        return bool(pd.isna(value))
    except TypeError:
        return False


if __name__ == "__main__":
    main()
