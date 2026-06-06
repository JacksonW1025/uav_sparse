from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.properties import compute_robustness
from cadet.query import read_parsed_log, theta_hash
from cadet.runners.contract_grid_phase0 import remove_ulg_files
from cadet.runners.direction_a_probe import (
    _config_for_probe,
    _run_query_with_retry_count,
    _safe_label,
    classify_robustness,
    support_summary,
)


OUTPUT_DIR = Path("artifacts/monotonicity_check_summary")
CONFIG_PATH = Path("configs/rq1_minimal.yaml")
SCENARIO_ID = "px4_position"
SEED = 0
REPEATS = 5
STICK_LIMIT = 1.0
PROPERTY = "post_neutral_xy_velocity"
GRID_WINDOW = (11.0, 13.0)
PROFILE_WINDOWS = [(5.0, 7.0), (7.0, 9.0), (9.0, 11.0), (11.0, 13.0)]


@dataclass(frozen=True)
class ThetaSpec:
    source_id: str
    source_kind: str
    signature: str
    theta_path: Path
    archived_eval_id: str = ""
    archived_label: str = ""
    archived_rho_mean: float | None = None
    archived_rho_std: float | None = None


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    base_config = load_config(CONFIG_PATH)
    config = _config_for_probe(base_config, OUTPUT_DIR, STICK_LIMIT)
    scenario = config.scenario_by_id(SCENARIO_ID)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    grid_threshold = _grid_c1_xy_threshold()
    orig_threshold = float(config.properties[PROPERTY]["v_max_mps"])

    theta_specs = _selected_theta_specs()
    rows: list[dict[str, Any]] = []
    repeat_rows: list[dict[str, Any]] = []

    for spec in theta_specs:
        theta = np.load(spec.theta_path).astype(float)
        thash = theta_hash(theta)
        support = support_summary(theta, groups)
        grid_peaks: list[float] = []
        grid_rhos: list[float] = []
        orig_rhos: list[float] = []
        profile_peaks: dict[tuple[float, float], list[float]] = {window: [] for window in PROFILE_WINDOWS}
        t_neutral_values: list[float] = []
        max_time_values: list[float] = []

        for repeat_idx in range(REPEATS):
            cache_tag = _safe_label(f"monotonicity_check_{spec.source_id}_repeat{repeat_idx}")
            result, retry_count = _run_query_with_retry_count(
                theta,
                scenario,
                SEED,
                "monotonicity_check",
                OUTPUT_DIR,
                config,
                use_cache=False,
                cache_tag=cache_tag,
            )
            parsed_log = read_parsed_log(result.parsed_log_path)
            t_neutral = float(parsed_log["t_neutral_s"].iloc[0])
            max_time = float(parsed_log["time_s"].max())
            t_neutral_values.append(t_neutral)
            max_time_values.append(max_time)

            grid_peak = _speed_peak(parsed_log, GRID_WINDOW)
            grid_rho = grid_threshold - grid_peak
            orig_rho = compute_robustness(parsed_log, PROPERTY, config)
            grid_peaks.append(grid_peak)
            grid_rhos.append(grid_rho)
            orig_rhos.append(orig_rho)

            repeat_row: dict[str, Any] = {
                "source_id": spec.source_id,
                "source_kind": spec.source_kind,
                "signature": spec.signature,
                "repeat_idx": repeat_idx,
                "query_id": result.query_id,
                "theta_hash": result.theta_hash,
                "grid_peak_11_13": grid_peak,
                "grid_rho": grid_rho,
                "orig_rho": orig_rho,
                "t_neutral_s": t_neutral,
                "t_max_s": max_time,
                "query_retry_count": retry_count,
            }
            for window in PROFILE_WINDOWS:
                peak = _speed_peak(parsed_log, window)
                profile_peaks[window].append(peak)
                repeat_row[f"peak_{_window_key(window)}"] = peak
            repeat_rows.append(repeat_row)

        grid_stats = _stats(grid_rhos)
        grid_peak_stats = _stats(grid_peaks)
        orig_stats = _stats(orig_rhos)
        orig_label = classify_robustness(orig_stats["mean"], orig_stats["std"])
        grid_label = classify_robustness(grid_stats["mean"], grid_stats["std"])
        t_neutral_mean = float(np.mean(t_neutral_values))
        t_max_mean = float(np.mean(max_time_values))

        row: dict[str, Any] = {
            "scope_label": "exploratory / verification",
            "platform": "PX4",
            "scenario": SCENARIO_ID,
            "seed": SEED,
            "J": REPEATS,
            "source_id": spec.source_id,
            "source_kind": spec.source_kind,
            "archived_eval_id": spec.archived_eval_id,
            "archived_label": spec.archived_label,
            "signature": spec.signature,
            "theta_hash": thash,
            "theta_path": str(spec.theta_path),
            "max_abs_theta": float(np.max(np.abs(theta))) if theta.size else 0.0,
            "support_size_abs_gt_0p1": int(support["support_size"]),
            "active_channels_abs_gt_0p1": ",".join(support["active_channels"]),
            "grid_metric_window": "[11,13]s absolute",
            "grid_metric_aggregation": "per-repeat peak |v_xy|; J=5 rho mean/std",
            "grid_threshold_mps": grid_threshold,
            "grid_peak_[11,13]": grid_peak_stats["mean"],
            "grid_peak_[11,13]_std": grid_peak_stats["std"],
            "grid_rho_mean": grid_stats["mean"],
            "grid_rho_std": grid_stats["std"],
            "grid_rho_min": grid_stats["min"],
            "grid_rho_max": grid_stats["max"],
            "grid_label": grid_label,
            "orig_metric_window": f"tail from t_neutral_s to log end; mean t_neutral={t_neutral_mean:.3f}s, mean end={t_max_mean:.3f}s",
            "orig_metric_aggregation": "per-repeat peak |v_xy| over tail; J=5 rho mean/std",
            "orig_metric_threshold_mps": orig_threshold,
            "orig_rho_archived": _none_to_nan(spec.archived_rho_mean),
            "orig_rho_archived_std": _none_to_nan(spec.archived_rho_std),
            "orig_rho_fresh_mean": orig_stats["mean"],
            "orig_rho_fresh_std": orig_stats["std"],
            "orig_rho_fresh_min": orig_stats["min"],
            "orig_rho_fresh_max": orig_stats["max"],
            "orig_label": orig_label,
            "fresh_minus_archived_orig_rho_mean": (
                orig_stats["mean"] - float(spec.archived_rho_mean)
                if spec.archived_rho_mean is not None
                else math.nan
            ),
        }
        for window in PROFILE_WINDOWS:
            stats = _stats(profile_peaks[window])
            key = _window_key(window)
            row[f"peak_{key}_mean"] = stats["mean"]
            row[f"peak_{key}_std"] = stats["std"]
            row[f"peak_{key}_min"] = stats["min"]
            row[f"peak_{key}_max"] = stats["max"]
        rows.append(row)

        pd.DataFrame(rows).to_csv(OUTPUT_DIR / "per_theta.csv", index=False)
        pd.DataFrame(repeat_rows).to_csv(OUTPUT_DIR / "per_repeat.csv", index=False)

    per_theta = pd.DataFrame(rows)
    per_theta.to_csv(OUTPUT_DIR / "per_theta.csv", index=False)
    pd.DataFrame(repeat_rows).to_csv(OUTPUT_DIR / "per_repeat.csv", index=False)
    _write_report(per_theta, OUTPUT_DIR / "monotonicity_check_report.md")
    remove_ulg_files(OUTPUT_DIR)
    print(f"monotonicity_check_complete report={OUTPUT_DIR / 'monotonicity_check_report.md'}")


def _selected_theta_specs() -> list[ThetaSpec]:
    cadet = pd.read_csv("artifacts/recut_distinct_v0/signatures_cadet.csv")
    point = pd.read_csv("artifacts/direction_a_px4_position_seed0_v0/reports/point_evaluations.csv")
    selected_eval_ids = [164, 174, 182, 199, 234]
    specs: list[ThetaSpec] = []
    for eval_id in selected_eval_ids:
        c = cadet[(cadet["seed"] == SEED) & (cadet["eval_id"] == eval_id)].iloc[0]
        p = point[point["eval_id"] == eval_id].iloc[0]
        theta_path = _resolve_theta_path(str(c["theta_path"]))
        specs.append(
            ThetaSpec(
                source_id=f"ArmC_eval{eval_id}",
                source_kind="known_internal_violation",
                signature=str(c["main_signature"]),
                theta_path=theta_path,
                archived_eval_id=str(eval_id),
                archived_label=str(c["label"]),
                archived_rho_mean=float(p[f"rho_mean_{PROPERTY}"]),
                archived_rho_std=float(p[f"rho_std_{PROPERTY}"]),
            )
        )
    specs.extend(
        [
            ThetaSpec(
                source_id="G01_roll_plus_full",
                source_kind="saturated_control",
                signature="G01 C1 roll_plus_full; roll:+; active [0,5] then neutral",
                theta_path=Path("artifacts/contract_grid_summary/thetas/G01_roll_plus_full_b96ead7c8720e928.npy"),
            ),
            ThetaSpec(
                source_id="G02_pitch_plus_full",
                source_kind="saturated_control",
                signature="G02 C1 pitch_plus_full; pitch:+; active [0,5] then neutral",
                theta_path=Path("artifacts/contract_grid_summary/thetas/G02_pitch_plus_full_a08edfccf9989958.npy"),
            ),
        ]
    )
    for spec in specs:
        if not spec.theta_path.exists():
            raise FileNotFoundError(f"Missing theta for {spec.source_id}: {spec.theta_path}")
    return specs


def _resolve_theta_path(raw: str) -> Path:
    path = Path(raw)
    artifact_path = Path(str(path).replace("runs/direction_a_px4_position_seed0_v0", "artifacts/direction_a_px4_position_seed0_v0"))
    if artifact_path.exists():
        return artifact_path
    if path.exists():
        return path
    raise FileNotFoundError(f"Cannot resolve archived theta path: {raw}")


def _grid_c1_xy_threshold() -> float:
    params = pd.read_csv("artifacts/contract_grid_summary/params_used.csv")
    row = params[(params["contract"] == "C1 Brake") & (params["measured_axis"] == "xy_velocity")].iloc[0]
    return float(row["derived_threshold"])


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


def _window_key(window: tuple[float, float]) -> str:
    return f"{int(window[0])}_{int(window[1])}"


def _none_to_nan(value: float | None) -> float:
    return math.nan if value is None else float(value)


def _fmt(value: Any, digits: int = 3) -> str:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(f):
        return "NA"
    return f"{f:.{digits}f}"


def _write_report(per_theta: pd.DataFrame, path: Path) -> None:
    internal = per_theta[per_theta["source_kind"] == "known_internal_violation"].copy()
    saturated = per_theta[per_theta["source_kind"] == "saturated_control"].copy()
    internal_not_grid_violation = internal[internal["grid_label"] != "robust_violation"]
    saturated_orig_violation = saturated[saturated["orig_label"] == "robust_violation"]
    saturated_orig_not_safe = saturated[saturated["orig_label"] != "robust_safe"]

    if internal_not_grid_violation.empty and saturated_orig_not_safe.empty:
        verdict = (
            "真非单调: known internal theta remain robust violations under grid [11,13], "
            "and saturated controls are robust safe under the original xy_velocity metric."
        )
    elif not internal_not_grid_violation.empty or not saturated_orig_violation.empty:
        reasons = []
        if not internal_not_grid_violation.empty:
            ids = ", ".join(internal_not_grid_violation["source_id"].astype(str))
            reasons.append(f"internal theta not robust-violating under grid [11,13]: {ids}")
        if not saturated_orig_violation.empty:
            ids = ", ".join(saturated_orig_violation["source_id"].astype(str))
            reasons.append(f"saturated controls violate the original metric: {ids}")
        verdict = "口径不一致: " + "; ".join(reasons) + "."
    else:
        ids = ", ".join(saturated_orig_not_safe["source_id"].astype(str))
        verdict = f"更细情形: internal theta agree with grid, but saturated original labels are not robust_safe ({ids})."

    lines = [
        "# Monotonicity Check Report",
        "",
        "Label: exploratory / verification. Platform: PX4. Scenario: px4_position. Seed: 0. J=5.",
        "",
        "## Inputs",
        "",
        "| source | kind | eval_id | signature | theta | archived rho mean/std |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for row in per_theta.to_dict("records"):
        archived = (
            "NA"
            if math.isnan(float(row["orig_rho_archived"]))
            else f"{row['orig_rho_archived']:.6f} / {row['orig_rho_archived_std']:.6f}"
        )
        lines.append(
            f"| {row['source_id']} | {row['source_kind']} | {row['archived_eval_id'] or 'NA'} | "
            f"{str(row['signature']).replace('|', '<bar>')} | `{row['theta_path']}` | {archived} |"
        )

    lines.extend(
        [
            "",
            "## Metric Definitions",
            "",
            "- Grid C1: absolute [11,13] s peak |v_xy|, threshold 1.0 m/s from contract-grid C1 Brake xy_velocity, rho = threshold - peak, label by rho_mean +/- 2*rho_std.",
            "- Original xy_velocity: existing `compute_robustness(..., post_neutral_xy_velocity, ...)`, tail from `t_neutral_s` to log end, peak |v_xy|, threshold 1.0 m/s from config.",
            "",
            "## Per-Theta Results",
            "",
            "| source | grid peak [11,13] mean/std | grid rho mean/std | grid label | orig fresh rho mean/std | orig label | archived orig rho | [5,7] | [7,9] | [9,11] | [11,13] |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        ]
    )
    for row in per_theta.to_dict("records"):
        archived = "NA" if math.isnan(float(row["orig_rho_archived"])) else _fmt(row["orig_rho_archived"], 3)
        lines.append(
            f"| {row['source_id']} | {_fmt(row['grid_peak_[11,13]'])} / {_fmt(row['grid_peak_[11,13]_std'])} | "
            f"{_fmt(row['grid_rho_mean'])} / {_fmt(row['grid_rho_std'])} | {row['grid_label']} | "
            f"{_fmt(row['orig_rho_fresh_mean'])} / {_fmt(row['orig_rho_fresh_std'])} | {row['orig_label']} | {archived} | "
            f"{_fmt(row['peak_5_7_mean'])} | {_fmt(row['peak_7_9_mean'])} | {_fmt(row['peak_9_11_mean'])} | {_fmt(row['peak_11_13_mean'])} |"
        )

    lines.extend(
        [
            "",
            "## Run-To-Run Check",
            "",
            "| source | archived orig rho mean | fresh orig rho mean | fresh - archived |",
            "| --- | --- | --- | --- |",
        ]
    )
    for row in internal.to_dict("records"):
        lines.append(
            f"| {row['source_id']} | {_fmt(row['orig_rho_archived'])} | {_fmt(row['orig_rho_fresh_mean'])} | "
            f"{_fmt(row['fresh_minus_archived_orig_rho_mean'])} |"
        )

    lines.extend(
        [
            "",
            "## Archive Hygiene",
            "",
            "- `*.ulg` files are removed from this artifact directory after the run; parsed telemetry CSV/parquet and JSON metadata are retained.",
            "",
            "## Judgment",
            "",
            verdict,
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
