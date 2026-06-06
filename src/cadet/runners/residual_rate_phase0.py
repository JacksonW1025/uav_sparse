from __future__ import annotations

import argparse
import math
import shutil
import time
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from cadet.config import ExperimentConfig, load_config
from cadet.groups import Group, build_groups
from cadet.input_model import project_theta, zero_theta
from cadet.properties import (
    compute_residual_rate_metrics,
    is_residual_rate_property,
    summarize_residual_rate_repeats,
)
from cadet.query import read_parsed_log, theta_hash
from cadet.runners.direction_a_probe import (
    J_REPEATS,
    ROBUST_SIGMA_MULTIPLIER,
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


PREREG_PATH = Path("artifacts/residual_rate_prereg.md")
TARGET_PROPERTIES = ["post_neutral_climb_rate", "post_neutral_yaw_rate"]


def main() -> None:
    parser = argparse.ArgumentParser(description="Confirmatory residual-rate Phase 0 saturated-channel sanity.")
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--property", required=True, choices=TARGET_PROPERTIES)
    parser.add_argument("--run-dir", default=None)
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=1.0)
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Residual-rate Phase 0 is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Residual-rate Phase 0 is frozen to seed 0")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("Residual-rate Phase 0 is pre-registered to J=5 repeats")
    if float(args.stick_limit) != 1.0:
        raise ValueError("Residual-rate Phase 0 is frozen to stick-limit=1.0")
    if not PREREG_PATH.exists():
        raise FileNotFoundError(f"Missing pre-registration artifact: {PREREG_PATH}")
    if not is_residual_rate_property(args.property):
        raise ValueError(f"Not a residual-rate property: {args.property}")

    run_start = time.monotonic()
    property_label = _property_label(args.property)
    output_dir = Path(args.run_dir or f"artifacts/residual_rate_{property_label}_seed0_v0")
    reports_dir = output_dir / "reports"
    thetas_dir = output_dir / "thetas"
    reports_dir.mkdir(parents=True, exist_ok=True)
    thetas_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(PREREG_PATH, reports_dir / "residual_rate_prereg.md")

    base_config = load_config(args.config)
    config = _config_for_probe(base_config, output_dir, float(args.stick_limit))
    scenario = config.scenario_by_id(args.scenario)
    if args.property not in scenario.properties:
        raise ValueError(f"{args.property} must be enabled for {args.scenario}")

    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"Frozen D=40 parameterization expected 40 groups, found {len(groups)}")
    pd.DataFrame([group.__dict__ for group in groups]).to_csv(output_dir / "groups.csv", index=False)

    active_channels = derive_A_phi(args.property)
    if len(active_channels) != 1:
        raise ValueError(f"Phase 0 expects one predicted channel, got {active_channels}")
    active_channel = active_channels[0]
    candidates = [
        ("zero_anchor", zero_theta(groups), "neutral zero input", 0),
        (
            f"{active_channel}_pos_full",
            _channel_theta(config, groups, active_channel, +1, float(args.stick_limit)),
            f"saturated positive {active_channel}",
            +1,
        ),
        (
            f"{active_channel}_neg_full",
            _channel_theta(config, groups, active_channel, -1, float(args.stick_limit)),
            f"saturated negative {active_channel}",
            -1,
        ),
    ]

    point_rows: list[dict[str, Any]] = []
    query_rows: list[dict[str, Any]] = []
    successful_query_count = 0
    timeout_retry_count = 0

    print(
        f"residual_rate_phase0_start confirmatory PX4 property={args.property} "
        f"scenario={args.scenario} seed={args.seed} J={args.repeats} "
        f"A_phi={','.join(active_channels)} run_dir={output_dir}",
        flush=True,
    )

    for label, theta, description, sign in candidates:
        row, repeat_rows, retry_count = _eval_j5(
            theta,
            scenario,
            int(args.seed),
            output_dir,
            config,
            groups,
            target_property=args.property,
            label=label,
            description=description,
            active_channel=active_channel,
            sign=sign,
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
        active_channel=active_channel,
        successful_query_count=successful_query_count,
        timeout_retry_count=timeout_retry_count,
        elapsed_wall_time_s=time.monotonic() - run_start,
    )
    _write_json(reports_dir / "phase0_sanity_summary.json", summary)
    _write_report(reports_dir / "phase0_sanity_report.md", summary)
    _remove_ulg_files(output_dir)
    print(
        "RESIDUAL_RATE_PHASE0_VERDICT "
        f"confirmatory PX4 property={args.property} "
        f"tier1_violable={summary['tier1_violable_with_saturated_predicted_channel']} "
        f"tier2_violable={summary['tier2_violable_with_saturated_predicted_channel']} "
        f"best_label={summary['best_tier1_violation_label']} "
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
    target_property: str,
    label: str,
    description: str,
    active_channel: str,
    sign: int,
    repeats: int,
) -> tuple[dict[str, Any], list[dict[str, Any]], int]:
    projected = project_theta(np.asarray(theta, dtype=float), config)
    thash = theta_hash(projected)
    values: dict[str, list[float]] = {prop: [] for prop in scenario.properties}
    residual_repeat_metrics: list[dict[str, Any]] = []
    query_rows: list[dict[str, Any]] = []
    retry_total = 0
    point_start = time.monotonic()
    for repeat_idx in range(repeats):
        repeat_start = time.monotonic()
        result, retry_count = _run_query_with_retry_count(
            projected,
            scenario,
            seed,
            "residual_rate_phase0",
            output_dir,
            config,
            cache_tag=_safe_label(f"residual_rate_phase0_{_property_label(target_property)}_{label}_repeat{repeat_idx}"),
            use_cache=True,
        )
        retry_total += retry_count
        for prop, value in result.robustness.items():
            values[prop].append(float(value))
        parsed_log = read_parsed_log(result.parsed_log_path)
        residual_metrics = compute_residual_rate_metrics(parsed_log, target_property, config)
        residual_repeat_metrics.append(residual_metrics)
        row: dict[str, Any] = {
            "label": label,
            "description": description,
            "active_channel": active_channel,
            "sign": sign,
            "repeat_idx": repeat_idx,
            "theta_hash": result.theta_hash,
            "query_id": result.query_id,
            "cache_tag": _safe_label(f"residual_rate_phase0_{_property_label(target_property)}_{label}_repeat{repeat_idx}"),
            "query_retry_count": retry_count,
            "repeat_elapsed_wall_time_s": time.monotonic() - repeat_start,
        }
        for prop, value in result.robustness.items():
            row[f"robustness_{prop}"] = float(value)
        for key, value in residual_metrics.items():
            row[f"residual_rate_{key}"] = value
        for key, value in result.metadata.items():
            row[f"meta_{key}"] = value
        query_rows.append(row)

    stats = _property_stats(values)
    target = stats[target_property]
    tier_summary = summarize_residual_rate_repeats(
        residual_repeat_metrics,
        sigma_multiplier=ROBUST_SIGMA_MULTIPLIER,
    )
    support = support_summary(projected, groups)
    max_abs_theta = float(np.max(np.abs(projected))) if projected.size else 0.0
    theta_path = output_dir / "thetas" / f"phase0_{label}_{thash}.npy"
    np.save(theta_path, projected)
    point_row: dict[str, Any] = {
        "label": label,
        "description": description,
        "active_channel": active_channel,
        "sign": sign,
        "theta_hash": thash,
        "theta_path": str(theta_path),
        "repeats": repeats,
        "point_elapsed_wall_time_s": time.monotonic() - point_start,
        "max_abs_theta": max_abs_theta,
        "amplitude_class": classify_amplitude(max_abs_theta),
        "support_size_abs_gt_0p1": int(support["support_size"]),
        "active_channels_abs_gt_0p1": ",".join(support["active_channels"]),
        "robustness_class": classify_robustness(target["mean"], target["std"]),
        "tier1_robustness_class": tier_summary["tier1_robustness_class"],
        "tier2_robustness_class": tier_summary["tier2_robustness_class"],
        "tier2_nondecay_robust": tier_summary["tier2_nondecay_robust"],
        f"rho_margin_2sigma_{target_property}": float(target["mean"] + ROBUST_SIGMA_MULTIPLIER * target["std"]),
    }
    for key, value in tier_summary.items():
        if key not in point_row:
            point_row[f"tier_metric_{key}"] = value
    for prop, prop_stats in stats.items():
        for key, value in prop_stats.items():
            point_row[f"rho_{key}_{prop}"] = value
    print(
        f"residual_rate_phase0_eval property={target_property} label={label} "
        f"rho_mean={target['mean']:.6f} rho_std={target['std']:.6f} "
        f"tier1={point_row['tier1_robustness_class']} tier2={point_row['tier2_robustness_class']} "
        f"terminal_peak={float(tier_summary['terminal_peak_abs_rate_mean']):.6f} "
        f"max_abs={max_abs_theta:.3f} channels={point_row['active_channels_abs_gt_0p1']}",
        flush=True,
    )
    return point_row, query_rows, retry_total


def _channel_theta(
    config: ExperimentConfig,
    groups: list[Group],
    channel: str,
    sign: int,
    stick_limit: float,
) -> np.ndarray:
    channels = list(config.input["channels"])
    if channel not in channels:
        raise ValueError(f"Frozen D=40 input must include {channel}")
    n_windows = window_count(config)
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    grid[:, channels.index(channel)] = float(sign) * float(stick_limit)
    return project_theta(grid_to_theta(grid, config, groups), config)


def _build_summary(
    *,
    args: argparse.Namespace,
    output_dir: Path,
    point_rows: list[dict[str, Any]],
    active_channel: str,
    successful_query_count: int,
    timeout_retry_count: int,
    elapsed_wall_time_s: float,
) -> dict[str, Any]:
    predicted_rows = [
        row
        for row in point_rows
        if row["active_channel"] == active_channel and int(row["sign"]) != 0
    ]
    tier1 = [row for row in predicted_rows if row["tier1_robustness_class"] == "robust_violation"]
    tier2 = [row for row in predicted_rows if row["tier2_robustness_class"] == "robust_violation"]
    best_tier1 = (
        min(tier1, key=lambda row: float(row[f"rho_margin_2sigma_{args.property}"]))
        if tier1
        else None
    )
    zero = next((row for row in point_rows if row["label"] == "zero_anchor"), None)
    return {
        "status": "complete",
        "phase": "Phase 0",
        "confirmatory_label": f"confirmatory PX4 {args.property}",
        "scenario_id": args.scenario,
        "seed": int(args.seed),
        "property": args.property,
        "derived_A_phi": derive_A_phi(args.property),
        "active_channel": active_channel,
        "zero_anchor_class": zero["tier1_robustness_class"] if zero else "",
        "tier1_violable_with_saturated_predicted_channel": bool(best_tier1 is not None),
        "tier2_violable_with_saturated_predicted_channel": bool(tier2),
        "best_tier1_violation_label": best_tier1["label"] if best_tier1 else "",
        "best_tier1_violation_sign": int(best_tier1["sign"]) if best_tier1 else 0,
        "best_tier1_violation_margin_2sigma": float(best_tier1[f"rho_margin_2sigma_{args.property}"])
        if best_tier1
        else math.nan,
        "point_rows": point_rows,
        "successful_query_count": successful_query_count,
        "timeout_retry_count": timeout_retry_count,
        "query_attempt_count_including_timeout_retries": successful_query_count + timeout_retry_count,
        "elapsed_wall_time_s": elapsed_wall_time_s,
        "artifacts": {
            "pre_registration_copy": str(output_dir / "reports" / "residual_rate_prereg.md"),
            "phase0_sanity": str(output_dir / "reports" / "phase0_sanity.csv"),
            "phase0_query_repeats": str(output_dir / "reports" / "phase0_query_repeats.csv"),
            "summary": str(output_dir / "reports" / "phase0_sanity_summary.json"),
            "report": str(output_dir / "reports" / "phase0_sanity_report.md"),
        },
    }


def _write_report(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# Residual-rate Phase 0",
        "",
        f"Scope: confirmatory, PX4, `{summary['property']}`, `px4_position`.",
        "",
        f"Tier 1 saturated predicted-channel violable: `{summary['tier1_violable_with_saturated_predicted_channel']}`.",
        f"Tier 2 saturated predicted-channel violable: `{summary['tier2_violable_with_saturated_predicted_channel']}`.",
        f"Best Tier 1 label: `{summary['best_tier1_violation_label']}`.",
        "",
        "| label | sign | max abs theta | channels | Tier 1 | Tier 2 | rho mean | rho std | terminal peak mean |",
        "| --- | ---: | ---: | --- | --- | --- | ---: | ---: | ---: |",
    ]
    prop = summary["property"]
    for row in summary["point_rows"]:
        lines.append(
            f"| {row['label']} | {row['sign']} | {row['max_abs_theta']:.3f} | "
            f"{row['active_channels_abs_gt_0p1']} | {row['tier1_robustness_class']} | "
            f"{row['tier2_robustness_class']} | {row[f'rho_mean_{prop}']:.6f} | "
            f"{row[f'rho_std_{prop}']:.6f} | "
            f"{row['tier_metric_terminal_peak_abs_rate_mean']:.6f} |"
        )
    lines.extend(
        [
            "",
            "Artifacts:",
            "",
        ]
    )
    for key, value in summary["artifacts"].items():
        lines.append(f"- {key}: `{value}`")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _remove_ulg_files(path: Path) -> None:
    for ulg_path in Path(path).rglob("*.ulg"):
        ulg_path.unlink()


def _property_label(property_name: str) -> str:
    if property_name == "post_neutral_climb_rate":
        return "climb_rate"
    if property_name == "post_neutral_yaw_rate":
        return "yaw_rate"
    return str(property_name).replace("post_neutral_", "")


if __name__ == "__main__":
    main()
