from __future__ import annotations

import argparse
import json
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
from cadet.runners.residual_rate_phase0 import PREREG_PATH, TARGET_PROPERTIES, _channel_theta, _property_label


DELTA_PROBE = 0.2
DEFAULT_BISECTION_ITERS = 7


def main() -> None:
    parser = argparse.ArgumentParser(description="Confirmatory residual-rate Phase 1 H-1 channel-mass probe.")
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--property", required=True, choices=TARGET_PROPERTIES)
    parser.add_argument("--run-dir", default=None)
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=1.0)
    parser.add_argument("--delta-probe", type=float, default=DELTA_PROBE)
    parser.add_argument("--bisection-iters", type=int, default=DEFAULT_BISECTION_ITERS)
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Residual-rate H-1 is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Residual-rate H-1 is seed-0 only")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("Residual-rate H-1 is pre-registered to J=5 repeats")
    if float(args.stick_limit) != 1.0:
        raise ValueError("Residual-rate H-1 is frozen to stick-limit=1.0")
    if float(args.delta_probe) != DELTA_PROBE:
        raise ValueError("Residual-rate H-1 is frozen to delta_probe=0.2")
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

    phase0_summary_path = reports_dir / "phase0_sanity_summary.json"
    phase0 = _read_json(phase0_summary_path)
    if not bool(phase0.get("tier1_violable_with_saturated_predicted_channel")):
        raise RuntimeError(f"Phase 0 did not establish Tier 1 violability: {phase0_summary_path}")
    boundary_sign = int(phase0["best_tier1_violation_sign"])
    if boundary_sign == 0:
        raise RuntimeError(f"Phase 0 did not record a nonzero violating sign: {phase0_summary_path}")

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
        raise ValueError(f"H-1 expects one predicted channel, got {active_channels}")
    predicted_channel = active_channels[0]

    point_rows: list[dict[str, Any]] = []
    query_rows: list[dict[str, Any]] = []
    successful_query_count = 0
    timeout_retry_count = 0
    print(
        f"residual_rate_h1_start confirmatory PX4 property={args.property} "
        f"scenario={args.scenario} seed={args.seed} J={args.repeats} "
        f"delta={args.delta_probe} boundary_channel={predicted_channel} "
        f"boundary_sign={boundary_sign} run_dir={output_dir}",
        flush=True,
    )

    zero = zero_theta(groups)
    full = _channel_theta(config, groups, predicted_channel, boundary_sign, float(args.stick_limit))
    zero_eval, rows, retries = _eval_j5(
        zero,
        scenario,
        int(args.seed),
        output_dir,
        config,
        groups,
        target_property=args.property,
        label="boundary_zero_safe",
        stage="boundary_bracket",
        repeats=int(args.repeats),
    )
    point_rows.append(zero_eval)
    query_rows.extend(rows)
    successful_query_count += int(args.repeats)
    timeout_retry_count += retries

    full_eval, rows, retries = _eval_j5(
        full,
        scenario,
        int(args.seed),
        output_dir,
        config,
        groups,
        target_property=args.property,
        label=f"boundary_{predicted_channel}_{'pos' if boundary_sign > 0 else 'neg'}_full",
        stage="boundary_bracket",
        repeats=int(args.repeats),
    )
    point_rows.append(full_eval)
    query_rows.extend(rows)
    successful_query_count += int(args.repeats)
    timeout_retry_count += retries
    if zero_eval["tier1_robustness_class"] != "robust_safe" or full_eval["tier1_robustness_class"] != "robust_violation":
        raise RuntimeError("H-1 boundary bracket failed: zero must be Tier 1 robust safe and full stick Tier 1 violation")

    safe_theta = zero
    unsafe_theta = full
    boundary_candidates = [zero_eval, full_eval]
    for iteration in range(int(args.bisection_iters)):
        mid_theta = project_theta(0.5 * (safe_theta + unsafe_theta), config)
        row, rows, retries = _eval_j5(
            mid_theta,
            scenario,
            int(args.seed),
            output_dir,
            config,
            groups,
            target_property=args.property,
            label=f"boundary_bisect_iter{iteration:02d}",
            stage="boundary_bisection",
            repeats=int(args.repeats),
        )
        point_rows.append(row)
        boundary_candidates.append(row)
        query_rows.extend(rows)
        successful_query_count += int(args.repeats)
        timeout_retry_count += retries
        if row["tier1_robustness_class"] == "robust_violation" or float(row[f"rho_mean_{args.property}"]) < 0.0:
            unsafe_theta = mid_theta
        else:
            safe_theta = mid_theta
        _write_boundary_progress(reports_dir, point_rows, query_rows)

    boundary_row = min(boundary_candidates, key=lambda row: abs(float(row[f"rho_mean_{args.property}"])))
    boundary_theta = np.load(boundary_row["theta_path"])
    boundary_terminal_peak = float(boundary_row[f"terminal_peak_abs_rate_mean_{args.property}"])
    probe_rows: list[dict[str, Any]] = []
    probe_eval_rows: list[dict[str, Any]] = []
    probe_query_rows: list[dict[str, Any]] = []

    for group in groups:
        signed_rows: dict[str, dict[str, Any]] = {}
        for sign_label, delta in [("plus", float(args.delta_probe)), ("minus", -float(args.delta_probe))]:
            raw = boundary_theta.copy()
            raw[group.group_id] += delta
            projected = project_theta(raw, config)
            row, rows, retries = _eval_j5(
                projected,
                scenario,
                int(args.seed),
                output_dir,
                config,
                groups,
                target_property=args.property,
                label=f"g{group.group_id:02d}_{sign_label}",
                stage="delta_probe",
                repeats=int(args.repeats),
            )
            signed_rows[sign_label] = row
            probe_eval_rows.append(row)
            probe_query_rows.extend(rows)
            successful_query_count += int(args.repeats)
            timeout_retry_count += retries

        plus_metric = float(signed_rows["plus"][f"terminal_peak_abs_rate_mean_{args.property}"])
        minus_metric = float(signed_rows["minus"][f"terminal_peak_abs_rate_mean_{args.property}"])
        plus_rho = float(signed_rows["plus"][f"rho_mean_{args.property}"])
        minus_rho = float(signed_rows["minus"][f"rho_mean_{args.property}"])
        span = plus_metric - minus_metric
        sensitivity = abs(span) / (2.0 * float(args.delta_probe))
        probe_rows.append(
            {
                **group.__dict__,
                "boundary_theta_hash": boundary_row["theta_hash"],
                "boundary_terminal_peak_abs_rate": boundary_terminal_peak,
                "delta_probe": float(args.delta_probe),
                "rho_plus_mean": plus_rho,
                "rho_minus_mean": minus_rho,
                "terminal_peak_plus_mean": plus_metric,
                "terminal_peak_minus_mean": minus_metric,
                "metric_span_plus_minus": span,
                "directional_sensitivity": sensitivity,
                "plus_increase_over_boundary": plus_metric - boundary_terminal_peak,
                "minus_increase_over_boundary": minus_metric - boundary_terminal_peak,
                "plus_tier1_class": signed_rows["plus"]["tier1_robustness_class"],
                "minus_tier1_class": signed_rows["minus"]["tier1_robustness_class"],
            }
        )
        print(
            f"residual_rate_h1_group property={args.property} g{group.group_id:02d} "
            f"{group.channel}@w{group.window_id} sens={sensitivity:.6f} "
            f"plus={plus_metric:.6f} minus={minus_metric:.6f}",
            flush=True,
        )
        if len(probe_rows) % 4 == 0:
            _write_probe_progress(reports_dir, probe_rows, probe_eval_rows, probe_query_rows)

    _write_boundary_progress(reports_dir, point_rows, query_rows)
    _write_probe_progress(reports_dir, probe_rows, probe_eval_rows, probe_query_rows)
    channel_rows = _marginal_rows(probe_rows, "channel", "directional_sensitivity")
    window_rows = _marginal_rows(probe_rows, "window_id", "directional_sensitivity")
    channel_sign_rows = _channel_sign_rows(probe_rows)
    decision = _h1_decision(channel_rows, predicted_channel)
    pd.DataFrame(channel_rows).to_csv(reports_dir / "h1_channel_mass.csv", index=False)
    pd.DataFrame(window_rows).to_csv(reports_dir / "h1_window_mass.csv", index=False)
    pd.DataFrame(channel_sign_rows).to_csv(reports_dir / "h1_channel_sign_mass.csv", index=False)
    pd.DataFrame(channel_rows).to_csv(reports_dir / "channel_mass.csv", index=False)

    summary = {
        "status": "complete",
        "phase": "Phase 1 H-1",
        "confirmatory_label": f"confirmatory PX4 {args.property}",
        "scenario_id": args.scenario,
        "seed": int(args.seed),
        "property": args.property,
        "derived_A_phi": derive_A_phi(args.property),
        "predicted_channel": predicted_channel,
        "boundary_sign": boundary_sign,
        "delta_probe": float(args.delta_probe),
        "boundary": boundary_row,
        "channel_mass": channel_rows,
        "window_mass": window_rows,
        "channel_sign_mass": channel_sign_rows,
        "h1_decision": decision,
        "successful_query_count": successful_query_count,
        "timeout_retry_count": timeout_retry_count,
        "query_attempt_count_including_timeout_retries": successful_query_count + timeout_retry_count,
        "elapsed_wall_time_s": time.monotonic() - run_start,
        "artifacts": {
            "pre_registration_copy": str(reports_dir / "residual_rate_prereg.md"),
            "phase0_summary": str(phase0_summary_path),
            "boundary_points": str(reports_dir / "h1_boundary_points.csv"),
            "probe_groups": str(reports_dir / "h1_probe_groups.csv"),
            "channel_mass": str(reports_dir / "h1_channel_mass.csv"),
            "channel_sign_mass": str(reports_dir / "h1_channel_sign_mass.csv"),
            "summary": str(reports_dir / "h1_summary.json"),
            "report": str(reports_dir / "h1_report.md"),
        },
    }
    _write_json(reports_dir / "h1_summary.json", summary)
    _write_report(reports_dir / "h1_report.md", summary)
    _remove_ulg_files(output_dir)
    print(
        "RESIDUAL_RATE_H1_VERDICT "
        f"confirmatory PX4 property={args.property} decision={decision['decision']} "
        f"top_channel={decision['top_channel']} predicted_share={decision['predicted_share']:.6f} "
        f"report={reports_dir / 'h1_report.md'}",
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
    stage: str,
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
        result, retry_count = _run_query_with_retry_count(
            projected,
            scenario,
            seed,
            "residual_rate_h1",
            output_dir,
            config,
            cache_tag=_safe_label(f"residual_rate_h1_{_property_label(target_property)}_{stage}_{label}_repeat{repeat_idx}"),
            use_cache=True,
        )
        retry_total += retry_count
        for prop, value in result.robustness.items():
            values[prop].append(float(value))
        parsed_log = read_parsed_log(result.parsed_log_path)
        residual_metrics = compute_residual_rate_metrics(parsed_log, target_property, config)
        residual_repeat_metrics.append(residual_metrics)
        row: dict[str, Any] = {
            "stage": stage,
            "label": label,
            "repeat_idx": repeat_idx,
            "theta_hash": result.theta_hash,
            "query_id": result.query_id,
            "query_retry_count": retry_count,
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
    theta_path = output_dir / "thetas" / f"h1_{stage}_{label}_{thash}.npy"
    np.save(theta_path, projected)
    point_row: dict[str, Any] = {
        "stage": stage,
        "label": label,
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
    }
    for key, value in tier_summary.items():
        point_row[f"{key}_{target_property}" if key.endswith(("_mean", "_std", "_min", "_max")) else key] = value
    for prop, prop_stats in stats.items():
        for key, value in prop_stats.items():
            point_row[f"rho_{key}_{prop}"] = value
    print(
        f"residual_rate_h1_eval property={target_property} stage={stage} label={label} "
        f"rho_mean={target['mean']:.6f} rho_std={target['std']:.6f} "
        f"tier1={point_row['tier1_robustness_class']} tier2={point_row['tier2_robustness_class']} "
        f"terminal_peak={float(tier_summary['terminal_peak_abs_rate_mean']):.6f} "
        f"max_abs={max_abs_theta:.3f}",
        flush=True,
    )
    return point_row, query_rows, retry_total


def _marginal_rows(rows: list[dict[str, Any]], key: str, weight_key: str) -> list[dict[str, Any]]:
    accum: dict[str, float] = {}
    for row in rows:
        label = str(row[key])
        accum[label] = accum.get(label, 0.0) + float(row[weight_key])
    total = sum(accum.values())
    return sorted(
        [
            {
                key: _maybe_int(label),
                "weight": weight,
                "share": weight / total if total > 0.0 else 0.0,
            }
            for label, weight in accum.items()
        ],
        key=lambda row: row["weight"],
        reverse=True,
    )


def _channel_sign_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    accum: dict[tuple[str, str], float] = {}
    for row in rows:
        channel = str(row["channel"])
        for sign_label, key in [("plus", "plus_increase_over_boundary"), ("minus", "minus_increase_over_boundary")]:
            accum[(channel, sign_label)] = accum.get((channel, sign_label), 0.0) + max(float(row[key]), 0.0)
    total = sum(accum.values())
    return sorted(
        [
            {
                "channel": channel,
                "sign": sign_label,
                "positive_increase_weight": weight,
                "share": weight / total if total > 0.0 else 0.0,
            }
            for (channel, sign_label), weight in accum.items()
        ],
        key=lambda row: row["positive_increase_weight"],
        reverse=True,
    )


def _h1_decision(channel_rows: list[dict[str, Any]], predicted_channel: str) -> dict[str, Any]:
    shares = {str(row["channel"]): float(row["share"]) for row in channel_rows}
    top = str(channel_rows[0]["channel"]) if channel_rows else ""
    predicted_share = shares.get(predicted_channel, 0.0)
    decision = "confirm" if top == predicted_channel and predicted_share >= 0.50 else "falsify"
    return {
        "decision": decision,
        "top_channel": top,
        "predicted_channel": predicted_channel,
        "predicted_share": predicted_share,
        "rule": "confirm if predicted channel is top and share >= 0.50",
    }


def _write_boundary_progress(reports_dir: Path, point_rows: list[dict[str, Any]], query_rows: list[dict[str, Any]]) -> None:
    pd.DataFrame(point_rows).to_csv(reports_dir / "h1_boundary_points.csv", index=False)
    pd.DataFrame(query_rows).to_csv(reports_dir / "h1_boundary_query_repeats.csv", index=False)


def _write_probe_progress(
    reports_dir: Path,
    probe_rows: list[dict[str, Any]],
    probe_eval_rows: list[dict[str, Any]],
    probe_query_rows: list[dict[str, Any]],
) -> None:
    pd.DataFrame(probe_rows).to_csv(reports_dir / "h1_probe_groups.csv", index=False)
    pd.DataFrame(probe_eval_rows).to_csv(reports_dir / "h1_probe_point_evaluations.csv", index=False)
    pd.DataFrame(probe_query_rows).to_csv(reports_dir / "h1_probe_query_repeats.csv", index=False)


def _write_report(path: Path, summary: dict[str, Any]) -> None:
    decision = summary["h1_decision"]
    prop = summary["property"]
    lines = [
        "# Residual-rate Phase 1 H-1",
        "",
        f"Scope: confirmatory, PX4, `{summary['property']}`, `px4_position`.",
        "",
        f"Decision: **{decision['decision']}**.",
        f"Top channel: `{decision['top_channel']}`.",
        f"Predicted channel: `{decision['predicted_channel']}`.",
        f"Predicted share: `{decision['predicted_share']:.6f}`.",
        "",
        "| channel | weight | share |",
        "| --- | ---: | ---: |",
    ]
    for row in summary["channel_mass"]:
        lines.append(f"| {row['channel']} | {row['weight']:.6f} | {row['share']:.6f} |")
    lines.extend(
        [
            "",
            "Channel/sign positive-increase mass:",
            "",
            "| channel | sign | weight | share |",
            "| --- | --- | ---: | ---: |",
        ]
    )
    for row in summary["channel_sign_mass"]:
        lines.append(
            f"| {row['channel']} | {row['sign']} | {row['positive_increase_weight']:.6f} | {row['share']:.6f} |"
        )
    lines.extend(
        [
            "",
            "Boundary point:",
            "",
            f"- theta hash: `{summary['boundary']['theta_hash']}`",
            f"- rho mean: `{summary['boundary'][f'rho_mean_{prop}']:.6f}`",
            f"- rho std: `{summary['boundary'][f'rho_std_{prop}']:.6f}`",
            f"- Tier 1: `{summary['boundary']['tier1_robustness_class']}`",
            f"- Tier 2: `{summary['boundary']['tier2_robustness_class']}`",
            "",
            "Artifacts:",
            "",
        ]
    )
    for key, value in summary["artifacts"].items():
        lines.append(f"- {key}: `{value}`")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return data


def _remove_ulg_files(path: Path) -> None:
    for ulg_path in Path(path).rglob("*.ulg"):
        ulg_path.unlink()


def _maybe_int(value: str) -> str | int:
    try:
        return int(value)
    except ValueError:
        return value


if __name__ == "__main__":
    main()
