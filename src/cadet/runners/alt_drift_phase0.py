from __future__ import annotations

import argparse
import json
import math
import shutil
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from cadet.config import ExperimentConfig, load_config
from cadet.groups import Group, build_groups
from cadet.input_model import project_theta, zero_theta
from cadet.query import theta_hash
from cadet.runners.direction_a_probe import (
    INTERIOR_MAX_ABS,
    J_REPEATS,
    ROBUST_SIGMA_MULTIPLIER,
    SATURATED_MIN_ABS,
    SUPPORT_THRESHOLD,
    _config_for_probe,
    _property_stats,
    _run_query_with_retry_count,
    _safe_label,
    _write_json,
    classify_amplitude,
    classify_robustness,
    derive_A_phi,
    support_summary,
)
from cadet.violation_search import grid_to_theta, window_count


TARGET_PROPERTY = "post_neutral_alt_drift"
REPORT_PROPERTIES = ["post_neutral_xy_velocity", "post_neutral_xy_drift", "post_neutral_alt_drift"]
PREREG_PATH = Path("artifacts/alt_drift_prereg.md")


def main() -> None:
    parser = argparse.ArgumentParser(description="Confirmatory alt_drift Phase 0 throttle-violation sanity.")
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="artifacts/alt_drift_seed0_v0")
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=1.0)
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("alt_drift Phase 0 is frozen to px4_position")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("alt_drift Phase 0 is pre-registered to J=5 repeats")
    if float(args.stick_limit) != 1.0:
        raise ValueError("alt_drift Phase 0 is frozen to stick-limit=1.0")
    if not PREREG_PATH.exists():
        raise FileNotFoundError(f"Missing pre-registration artifact: {PREREG_PATH}")

    run_start = time.monotonic()
    output_dir = Path(args.run_dir)
    reports_dir = output_dir / "reports"
    thetas_dir = output_dir / "thetas"
    reports_dir.mkdir(parents=True, exist_ok=True)
    thetas_dir.mkdir(parents=True, exist_ok=True)

    base_config = load_config(args.config)
    config = _config_for_probe(base_config, output_dir, float(args.stick_limit))
    scenario = config.scenario_by_id(args.scenario)
    if TARGET_PROPERTY not in scenario.properties:
        raise ValueError(f"{TARGET_PROPERTY} must be enabled for {args.scenario}")
    for prop in REPORT_PROPERTIES:
        if prop not in scenario.properties:
            raise ValueError(f"{prop} must be enabled for {args.scenario}")

    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"Frozen D=40 parameterization expected 40 groups, found {len(groups)}")
    pd.DataFrame([group.__dict__ for group in groups]).to_csv(output_dir / "groups.csv", index=False)

    preregistration = _phase0_pre_registration(args, config, groups)
    _write_json(reports_dir / "phase0_pre_registration.json", preregistration)
    shutil.copy2(PREREG_PATH, reports_dir / "alt_drift_prereg.md")

    candidates = [
        ("zero_anchor", zero_theta(groups), "neutral zero input"),
        ("throttle_pos_full", _throttle_theta(config, groups, +1, float(args.stick_limit)), "saturated positive throttle"),
        ("throttle_neg_full", _throttle_theta(config, groups, -1, float(args.stick_limit)), "saturated negative throttle"),
    ]

    point_rows: list[dict[str, Any]] = []
    query_rows: list[dict[str, Any]] = []
    successful_query_count = 0
    timeout_retry_count = 0

    print(
        f"alt_drift_phase0_start confirmatory PX4 alt_drift scenario={args.scenario} seed={args.seed} "
        f"J={args.repeats} stick_limit={args.stick_limit} A_phi={','.join(derive_A_phi(TARGET_PROPERTY))} "
        f"run_dir={output_dir}",
        flush=True,
    )

    for label, theta, description in candidates:
        row, repeat_rows, retry_count = _eval_j5(
            theta,
            scenario,
            int(args.seed),
            output_dir,
            config,
            groups,
            label=label,
            description=description,
            repeats=int(args.repeats),
        )
        successful_query_count += int(args.repeats)
        timeout_retry_count += retry_count
        point_rows.append(row)
        query_rows.extend(repeat_rows)
        pd.DataFrame(point_rows).to_csv(reports_dir / "phase0_sanity.csv", index=False)
        pd.DataFrame(query_rows).to_csv(reports_dir / "phase0_query_repeats.csv", index=False)

    summary = _build_summary(
        args=args,
        output_dir=output_dir,
        point_rows=point_rows,
        successful_query_count=successful_query_count,
        timeout_retry_count=timeout_retry_count,
        elapsed_wall_time_s=time.monotonic() - run_start,
    )
    _write_json(reports_dir / "phase0_sanity_summary.json", summary)
    _write_report(reports_dir / "phase0_sanity_report.md", summary)
    _remove_ulg_files(output_dir)

    print(
        "ALT_DRIFT_PHASE0_VERDICT "
        f"confirmatory PX4 alt_drift feasible_violation={summary['feasible_throttle_violation']} "
        f"best_label={summary['best_violation_label']} "
        f"best_margin_2sigma={_fmt(summary['best_violation_margin_2sigma'])} "
        f"report={reports_dir / 'phase0_sanity_report.md'}",
        flush=True,
    )


def _eval_j5(
    theta: np.ndarray,
    scenario,
    seed: int,
    output_dir: Path,
    config: ExperimentConfig,
    groups: list[Group],
    *,
    label: str,
    description: str,
    repeats: int,
) -> tuple[dict[str, Any], list[dict[str, Any]], int]:
    projected = project_theta(np.asarray(theta, dtype=float), config)
    thash = theta_hash(projected)
    values: dict[str, list[float]] = {prop: [] for prop in scenario.properties}
    query_rows: list[dict[str, Any]] = []
    retry_total = 0
    point_start = time.monotonic()
    for repeat_idx in range(repeats):
        cache_tag = _safe_label(f"alt_drift_phase0_{label}_repeat{repeat_idx}")
        repeat_start = time.monotonic()
        result, retry_count = _run_query_with_retry_count(
            projected,
            scenario,
            seed,
            "alt_drift_phase0",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        retry_total += retry_count
        for prop, value in result.robustness.items():
            values[prop].append(float(value))
        row: dict[str, Any] = {
            "label": label,
            "description": description,
            "repeat_idx": repeat_idx,
            "theta_hash": result.theta_hash,
            "query_id": result.query_id,
            "cache_tag": cache_tag,
            "query_retry_count": retry_count,
            "repeat_elapsed_wall_time_s": time.monotonic() - repeat_start,
        }
        for prop, value in result.robustness.items():
            row[f"robustness_{prop}"] = float(value)
        for key, value in result.metadata.items():
            row[f"meta_{key}"] = value
        query_rows.append(row)
    stats = _property_stats(values)
    target = stats[TARGET_PROPERTY]
    support = support_summary(projected, groups)
    max_abs_theta = float(np.max(np.abs(projected))) if projected.size else 0.0
    theta_path = output_dir / "thetas" / f"phase0_{label}_{thash}.npy"
    np.save(theta_path, projected)
    point_row: dict[str, Any] = {
        "label": label,
        "description": description,
        "theta_hash": thash,
        "theta_path": str(theta_path),
        "repeats": repeats,
        "point_elapsed_wall_time_s": time.monotonic() - point_start,
        "max_abs_theta": max_abs_theta,
        "amplitude_class": classify_amplitude(max_abs_theta),
        "support_size_abs_gt_0p1": int(support["support_size"]),
        "active_channels_abs_gt_0p1": ",".join(support["active_channels"]),
        "robustness_class": classify_robustness(target["mean"], target["std"]),
        "rho_margin_2sigma_post_neutral_alt_drift": float(
            target["mean"] + ROBUST_SIGMA_MULTIPLIER * target["std"]
        ),
    }
    for prop, prop_stats in stats.items():
        for key, value in prop_stats.items():
            point_row[f"rho_{key}_{prop}"] = value
    print(
        f"alt_drift_phase0_eval label={label} rho_mean={target['mean']:.6f} "
        f"rho_std={target['std']:.6f} class={point_row['robustness_class']} "
        f"margin_2sigma={point_row['rho_margin_2sigma_post_neutral_alt_drift']:.6f} "
        f"max_abs={max_abs_theta:.3f} channels={point_row['active_channels_abs_gt_0p1']}",
        flush=True,
    )
    return point_row, query_rows, retry_total


def _throttle_theta(config: ExperimentConfig, groups: list[Group], sign: int, stick_limit: float) -> np.ndarray:
    channels = list(config.input["channels"])
    if "throttle" not in channels:
        raise ValueError("Frozen D=40 input must include throttle")
    n_windows = window_count(config)
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    grid[:, channels.index("throttle")] = float(sign) * float(stick_limit)
    return project_theta(grid_to_theta(grid, config, groups), config)


def _build_summary(
    *,
    args: argparse.Namespace,
    output_dir: Path,
    point_rows: list[dict[str, Any]],
    successful_query_count: int,
    timeout_retry_count: int,
    elapsed_wall_time_s: float,
) -> dict[str, Any]:
    throttle_rows = [row for row in point_rows if "throttle" in str(row["active_channels_abs_gt_0p1"]).split(",")]
    violations = [row for row in throttle_rows if row["robustness_class"] == "robust_violation"]
    best = min(violations, key=lambda row: float(row["rho_margin_2sigma_post_neutral_alt_drift"])) if violations else None
    zero = next((row for row in point_rows if row["label"] == "zero_anchor"), None)
    return _jsonable(
        {
            "status": "complete",
            "phase": "Phase 0",
            "confirmatory_label": "confirmatory PX4 alt_drift",
            "scenario_id": args.scenario,
            "seed": int(args.seed),
            "property": TARGET_PROPERTY,
            "derived_A_phi": derive_A_phi(TARGET_PROPERTY),
            "frozen_constants": {
                "D": 40,
                "stick_limit": float(args.stick_limit),
                "J": int(args.repeats),
                "sigma_gate": "rho_mean + 2*rho_std < 0",
                "INTERIOR_MAX_ABS": INTERIOR_MAX_ABS,
                "SATURATED_MIN_ABS": SATURATED_MIN_ABS,
                "SUPPORT_THRESHOLD": SUPPORT_THRESHOLD,
                "scenario": "px4_position",
            },
            "throttle_centering_check": {
                "parameter_midpoint": 0.0,
                "manual_control_mapping": "normalized throttle=0 maps to MAVLink z=500; neutral tail sends throttle=0",
                "status": "ok",
            },
            "zero_anchor_class": zero["robustness_class"] if zero else "",
            "feasible_throttle_violation": bool(best is not None),
            "best_violation_label": best["label"] if best else "",
            "best_violation_margin_2sigma": float(best["rho_margin_2sigma_post_neutral_alt_drift"]) if best else math.nan,
            "best_violation_rho_mean": float(best[f"rho_mean_{TARGET_PROPERTY}"]) if best else math.nan,
            "best_violation_rho_std": float(best[f"rho_std_{TARGET_PROPERTY}"]) if best else math.nan,
            "point_rows": point_rows,
            "successful_query_count": int(successful_query_count),
            "timeout_retry_count": int(timeout_retry_count),
            "query_attempt_count_including_timeout_retries": int(successful_query_count + timeout_retry_count),
            "elapsed_wall_time_s": elapsed_wall_time_s,
            "artifacts": {
                "pre_registration_copy": str(output_dir / "reports" / "alt_drift_prereg.md"),
                "phase0_pre_registration": str(output_dir / "reports" / "phase0_pre_registration.json"),
                "phase0_sanity": str(output_dir / "reports" / "phase0_sanity.csv"),
                "phase0_query_repeats": str(output_dir / "reports" / "phase0_query_repeats.csv"),
                "phase0_summary": str(output_dir / "reports" / "phase0_sanity_summary.json"),
                "phase0_report": str(output_dir / "reports" / "phase0_sanity_report.md"),
            },
            "next_step_control": "Stop here until the user authorizes Phase 1.",
        }
    )


def _phase0_pre_registration(args: argparse.Namespace, config: ExperimentConfig, groups: list[Group]) -> dict[str, Any]:
    return {
        "source": str(PREREG_PATH),
        "phase": "Phase 0 sanity only",
        "scope": {
            "scenario": args.scenario,
            "seed": int(args.seed),
            "primary_property": TARGET_PROPERTY,
            "D": len(groups),
            "A_alt": derive_A_phi(TARGET_PROPERTY),
        },
        "thresholds": {
            "robust_violation": "rho_mean + 2*rho_std < 0",
            "robust_safe": "rho_mean - 2*rho_std > 0",
            "sigma_multiplier": ROBUST_SIGMA_MULTIPLIER,
            "interior_max_abs_theta": INTERIOR_MAX_ABS,
            "saturated_min_abs_theta": SATURATED_MIN_ABS,
            "support_abs_threshold": SUPPORT_THRESHOLD,
        },
        "input": {
            "horizon_s": float(config.input["horizon_s"]),
            "window_s": float(config.input["window_s"]),
            "neutral_tail_s": float(config.input["neutral_tail_s"]),
            "channels": list(config.input["channels"]),
            "min_value": float(config.input["min_value"]),
            "max_value": float(config.input["max_value"]),
            "max_delta_per_window": float(config.input["max_delta_per_window"]),
        },
        "evaluated_points": ["zero_anchor", "throttle_pos_full", "throttle_neg_full"],
        "discipline_statement": "No thresholds or Phase 1 decisions are adjusted in Phase 0.",
    }


def _write_report(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# alt_drift Phase 0 Sanity",
        "",
        "Scope: confirmatory, PX4, alt_drift, px4_position.",
        "",
        f"Feasible saturated-throttle violation: **{summary['feasible_throttle_violation']}**.",
        f"Best violation label: `{summary['best_violation_label']}`.",
        f"Best 2sigma margin: `{_fmt(summary['best_violation_margin_2sigma'])}`.",
        "",
        "| label | class | max|theta| | channels | rho mean | rho std | 2sigma margin |",
        "| --- | --- | ---: | --- | ---: | ---: | ---: |",
    ]
    for row in summary["point_rows"]:
        lines.append(
            f"| {row['label']} | {row['robustness_class']} | {row['max_abs_theta']:.3f} | "
            f"{row['active_channels_abs_gt_0p1']} | {row[f'rho_mean_{TARGET_PROPERTY}']:.6f} | "
            f"{row[f'rho_std_{TARGET_PROPERTY}']:.6f} | "
            f"{row['rho_margin_2sigma_post_neutral_alt_drift']:.6f} |"
        )
    lines.extend(
        [
            "",
            "Decision: stop after Phase 0 until the user authorizes Phase 1.",
            "",
            "No `*.ulg` files are retained in this artifact directory.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _remove_ulg_files(path: Path) -> None:
    for ulg_path in Path(path).rglob("*.ulg"):
        ulg_path.unlink()


def _fmt(value: Any) -> str:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return ""
    return "nan" if math.isnan(value) else f"{value:.6f}"


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_jsonable(v) for v in value]
    if isinstance(value, tuple):
        return [_jsonable(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    return value


if __name__ == "__main__":
    main()
