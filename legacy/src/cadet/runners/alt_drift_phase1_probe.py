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
from cadet.query import theta_hash
from cadet.runners.alt_drift_phase0 import PREREG_PATH, REPORT_PROPERTIES, TARGET_PROPERTY
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


DELTA_PROBE = 0.2
DEFAULT_BISECTION_ITERS = 7


def main() -> None:
    parser = argparse.ArgumentParser(description="Confirmatory alt_drift Phase 1 H-alt-1 channel-mass probe.")
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="artifacts/alt_drift_seed0_v0")
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=1.0)
    parser.add_argument("--delta-probe", type=float, default=DELTA_PROBE)
    parser.add_argument("--bisection-iters", type=int, default=DEFAULT_BISECTION_ITERS)
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("alt_drift Phase 1 H-alt-1 is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Phase 1 H-alt-1 is seed-0 only")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("alt_drift H-alt-1 is pre-registered to J=5 repeats")
    if float(args.stick_limit) != 1.0:
        raise ValueError("alt_drift H-alt-1 is frozen to stick-limit=1.0")
    if float(args.delta_probe) != DELTA_PROBE:
        raise ValueError("alt_drift H-alt-1 is frozen to delta_probe=0.2")
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
    shutil.copy2(PREREG_PATH, reports_dir / "alt_drift_prereg.md")
    _write_json(reports_dir / "h_alt_1_pre_registration.json", _pre_registration(args, config, groups))

    point_rows: list[dict[str, Any]] = []
    query_rows: list[dict[str, Any]] = []
    successful_query_count = 0
    timeout_retry_count = 0

    print(
        f"alt_drift_h1_start confirmatory PX4 alt_drift scenario={args.scenario} seed={args.seed} "
        f"J={args.repeats} delta={args.delta_probe} run_dir={output_dir}",
        flush=True,
    )

    zero = zero_theta(groups)
    full = _throttle_theta(config, groups, +1, float(args.stick_limit))
    zero_eval, rows, retries = _eval_j5(
        zero,
        scenario,
        int(args.seed),
        output_dir,
        config,
        groups,
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
        label="boundary_throttle_pos_full",
        stage="boundary_bracket",
        repeats=int(args.repeats),
    )
    point_rows.append(full_eval)
    query_rows.extend(rows)
    successful_query_count += int(args.repeats)
    timeout_retry_count += retries
    if zero_eval["robustness_class"] != "robust_safe" or full_eval["robustness_class"] != "robust_violation":
        raise RuntimeError(
            "Phase 1 H-alt-1 boundary bracket failed: zero must be robust_safe and positive throttle must be robust_violation"
        )

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
            label=f"boundary_bisect_iter{iteration:02d}",
            stage="boundary_bisection",
            repeats=int(args.repeats),
        )
        point_rows.append(row)
        boundary_candidates.append(row)
        query_rows.extend(rows)
        successful_query_count += int(args.repeats)
        timeout_retry_count += retries
        if row["robustness_class"] == "robust_violation" or float(row[f"rho_mean_{TARGET_PROPERTY}"]) < 0.0:
            unsafe_theta = mid_theta
        else:
            safe_theta = mid_theta
        _write_progress(reports_dir, point_rows, query_rows)

    boundary_row = min(boundary_candidates, key=lambda row: abs(float(row[f"rho_mean_{TARGET_PROPERTY}"])))
    boundary_theta = np.load(boundary_row["theta_path"])
    probe_rows: list[dict[str, Any]] = []
    raw_rows: list[dict[str, Any]] = []
    for group in groups:
        plus_values: list[float] = []
        minus_values: list[float] = []
        plus_projection: list[float] = []
        minus_projection: list[float] = []
        for sign, values, projections in [("+", plus_values, plus_projection), ("-", minus_values, minus_projection)]:
            for repeat_idx in range(int(args.repeats)):
                raw = boundary_theta.copy()
                raw[group.group_id] += float(args.delta_probe) if sign == "+" else -float(args.delta_probe)
                projected = project_theta(raw, config)
                projections.append(float(np.max(np.abs(projected - raw))) if raw.size else 0.0)
                result, retry_count = _run_query_with_retry_count(
                    projected,
                    scenario,
                    int(args.seed),
                    "alt_drift_h_alt_1",
                    output_dir,
                    config,
                    cache_tag=_safe_label(
                        f"alt_drift_h1_g{group.group_id:02d}_{'plus' if sign == '+' else 'minus'}_repeat{repeat_idx}"
                    ),
                    use_cache=True,
                )
                successful_query_count += 1
                timeout_retry_count += retry_count
                rho = float(result.robustness[TARGET_PROPERTY])
                values.append(rho)
                raw_row: dict[str, Any] = {
                    **group.__dict__,
                    "boundary_theta_hash": boundary_row["theta_hash"],
                    "sign": sign,
                    "repeat_idx": repeat_idx,
                    "delta_probe": float(args.delta_probe),
                    "projection_linf": projections[-1],
                    "theta_hash": result.theta_hash,
                    "query_id": result.query_id,
                    "query_retry_count": retry_count,
                    f"rho_{TARGET_PROPERTY}": rho,
                }
                for prop, value in result.robustness.items():
                    raw_row[f"rho_{prop}"] = float(value)
                raw_rows.append(raw_row)

        plus = np.asarray(plus_values, dtype=float)
        minus = np.asarray(minus_values, dtype=float)
        plus_mean = float(np.mean(plus))
        minus_mean = float(np.mean(minus))
        plus_std = float(np.std(plus, ddof=1)) if plus.size > 1 else 0.0
        minus_std = float(np.std(minus, ddof=1)) if minus.size > 1 else 0.0
        span = plus_mean - minus_mean
        abs_span = abs(span)
        sensitivity = abs_span / (2.0 * float(args.delta_probe))
        se_span = math.sqrt((plus_std**2 + minus_std**2) / int(args.repeats))
        probe_rows.append(
            {
                **group.__dict__,
                "boundary_theta_hash": boundary_row["theta_hash"],
                "delta_probe": float(args.delta_probe),
                "repeat_count": int(args.repeats),
                f"base_rho_mean_{TARGET_PROPERTY}": float(boundary_row[f"rho_mean_{TARGET_PROPERTY}"]),
                f"rho_plus_mean_{TARGET_PROPERTY}": plus_mean,
                f"rho_minus_mean_{TARGET_PROPERTY}": minus_mean,
                f"rho_plus_std_{TARGET_PROPERTY}": plus_std,
                f"rho_minus_std_{TARGET_PROPERTY}": minus_std,
                "rho_span_plus_minus": span,
                "abs_delta_rho_span": abs_span,
                "directional_sensitivity": sensitivity,
                "directional_sensitivity_se": se_span / (2.0 * float(args.delta_probe)),
                "plus_projection_linf_max": float(np.max(plus_projection)) if plus_projection else 0.0,
                "minus_projection_linf_max": float(np.max(minus_projection)) if minus_projection else 0.0,
            }
        )
        print(
            f"alt_drift_h1_group g{group.group_id:02d} {group.channel}@w{group.window_id} "
            f"sens={sensitivity:.6f} abs_span={abs_span:.6f}",
            flush=True,
        )
        if len(probe_rows) % 4 == 0:
            _write_probe_progress(reports_dir, raw_rows, probe_rows)

    _write_progress(reports_dir, point_rows, query_rows)
    _write_probe_progress(reports_dir, raw_rows, probe_rows)
    channel_rows = _marginal_rows(probe_rows, "channel")
    window_rows = _marginal_rows(probe_rows, "window_id")
    decision = _h_alt_1_decision(channel_rows)
    pd.DataFrame(channel_rows).to_csv(reports_dir / "h_alt_1_channel_mass.csv", index=False)
    pd.DataFrame(window_rows).to_csv(reports_dir / "h_alt_1_window_mass.csv", index=False)
    pd.DataFrame(channel_rows).to_csv(reports_dir / "channel_mass.csv", index=False)

    summary = _jsonable(
        {
            "status": "complete",
            "phase": "Phase 1 H-alt-1",
            "confirmatory_label": "confirmatory PX4 alt_drift",
            "scenario_id": args.scenario,
            "seed": int(args.seed),
            "property": TARGET_PROPERTY,
            "derived_A_phi": derive_A_phi(TARGET_PROPERTY),
            "delta_probe": float(args.delta_probe),
            "boundary": boundary_row,
            "channel_mass": channel_rows,
            "window_mass": window_rows,
            "h_alt_1_decision": decision,
            "successful_query_count": successful_query_count,
            "timeout_retry_count": timeout_retry_count,
            "query_attempt_count_including_timeout_retries": successful_query_count + timeout_retry_count,
            "elapsed_wall_time_s": time.monotonic() - run_start,
            "artifacts": {
                "pre_registration_copy": str(reports_dir / "alt_drift_prereg.md"),
                "pre_registration": str(reports_dir / "h_alt_1_pre_registration.json"),
                "boundary_points": str(reports_dir / "h_alt_1_boundary_points.csv"),
                "boundary_query_repeats": str(reports_dir / "h_alt_1_boundary_query_repeats.csv"),
                "probe_raw": str(reports_dir / "h_alt_1_probe_raw.csv"),
                "probe_groups": str(reports_dir / "h_alt_1_probe_groups.csv"),
                "channel_mass": str(reports_dir / "h_alt_1_channel_mass.csv"),
                "summary": str(reports_dir / "h_alt_1_summary.json"),
                "report": str(reports_dir / "h_alt_1_report.md"),
            },
        }
    )
    _write_json(reports_dir / "h_alt_1_summary.json", summary)
    _write_report(reports_dir / "h_alt_1_report.md", summary)
    _remove_ulg_files(output_dir)
    print(
        "ALT_DRIFT_H_ALT_1_VERDICT "
        f"confirmatory PX4 alt_drift decision={decision['decision']} "
        f"top_channel={decision['top_channel']} throttle_share={decision['throttle_share']:.6f} "
        f"A_alt={','.join(decision['recommended_A_alt'])} report={reports_dir / 'h_alt_1_report.md'}",
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
    stage: str,
    repeats: int,
) -> tuple[dict[str, Any], list[dict[str, Any]], int]:
    projected = project_theta(np.asarray(theta, dtype=float), config)
    thash = theta_hash(projected)
    values: dict[str, list[float]] = {prop: [] for prop in scenario.properties}
    query_rows: list[dict[str, Any]] = []
    retry_total = 0
    point_start = time.monotonic()
    for repeat_idx in range(repeats):
        result, retry_count = _run_query_with_retry_count(
            projected,
            scenario,
            seed,
            "alt_drift_h_alt_1",
            output_dir,
            config,
            cache_tag=_safe_label(f"alt_drift_h1_{stage}_{label}_repeat{repeat_idx}"),
            use_cache=True,
        )
        retry_total += retry_count
        for prop, value in result.robustness.items():
            values[prop].append(float(value))
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
        for key, value in result.metadata.items():
            row[f"meta_{key}"] = value
        query_rows.append(row)

    stats = _property_stats(values)
    target = stats[TARGET_PROPERTY]
    support = support_summary(projected, groups)
    max_abs_theta = float(np.max(np.abs(projected))) if projected.size else 0.0
    theta_path = output_dir / "thetas" / f"h1_{label}_{thash}.npy"
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
        f"rho_margin_2sigma_{TARGET_PROPERTY}": float(
            target["mean"] + ROBUST_SIGMA_MULTIPLIER * target["std"]
        ),
    }
    for prop, prop_stats in stats.items():
        for key, value in prop_stats.items():
            point_row[f"rho_{key}_{prop}"] = value
    print(
        f"alt_drift_h1_eval stage={stage} label={label} rho_mean={target['mean']:.6f} "
        f"rho_std={target['std']:.6f} class={point_row['robustness_class']} "
        f"margin_2sigma={point_row[f'rho_margin_2sigma_{TARGET_PROPERTY}']:.6f} "
        f"max_abs={max_abs_theta:.3f} channels={point_row['active_channels_abs_gt_0p1']}",
        flush=True,
    )
    return point_row, query_rows, retry_total


def _throttle_theta(config: ExperimentConfig, groups: list[Group], sign: int, stick_limit: float) -> np.ndarray:
    channels = list(config.input["channels"])
    n_windows = window_count(config)
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    grid[:, channels.index("throttle")] = float(sign) * float(stick_limit)
    return project_theta(grid_to_theta(grid, config, groups), config)


def _marginal_rows(group_rows: list[dict[str, Any]], key: str) -> list[dict[str, Any]]:
    accum: dict[str, float] = {}
    for row in group_rows:
        label = str(row[key])
        accum[label] = accum.get(label, 0.0) + float(row["directional_sensitivity"])
    total = sum(accum.values())
    rows = [
        {
            key: _maybe_int(label),
            "weight": weight,
            "share": weight / total if total > 0.0 else 0.0,
        }
        for label, weight in accum.items()
    ]
    return sorted(rows, key=lambda row: row["weight"], reverse=True)


def _h_alt_1_decision(channel_rows: list[dict[str, Any]]) -> dict[str, Any]:
    shares = {str(row["channel"]): float(row["share"]) for row in channel_rows}
    top = str(channel_rows[0]["channel"]) if channel_rows else ""
    throttle_share = shares.get("throttle", 0.0)
    tilt_shares = {channel: shares.get(channel, 0.0) for channel in ["roll", "pitch"]}
    largest_tilt = max(tilt_shares, key=tilt_shares.get)
    throttle_plus_largest_tilt = throttle_share + tilt_shares[largest_tilt]
    if top == "throttle" and throttle_share >= 0.50:
        decision = "confirm"
        recommended = ["throttle"]
    elif top == "throttle" and (0.30 <= throttle_share < 0.50 or throttle_plus_largest_tilt >= 0.70):
        decision = "narrow"
        recommended = ["throttle", largest_tilt] if tilt_shares[largest_tilt] > 0.0 else ["throttle"]
    else:
        decision = "falsify"
        recommended = []
    return {
        "decision": decision,
        "top_channel": top,
        "throttle_share": throttle_share,
        "largest_tilt_channel": largest_tilt,
        "largest_tilt_share": tilt_shares[largest_tilt],
        "throttle_plus_largest_tilt_share": throttle_plus_largest_tilt,
        "recommended_A_alt": recommended,
        "rule": (
            "confirm if throttle is top and share>=0.50; narrow if throttle top and share in [0.30,0.50) "
            "or throttle+largest tilt>=0.70; falsify if throttle is not top"
        ),
    }


def _pre_registration(args: argparse.Namespace, config: ExperimentConfig, groups: list[Group]) -> dict[str, Any]:
    return {
        "source": str(PREREG_PATH),
        "phase": "Phase 1 H-alt-1",
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
            "delta_probe": DELTA_PROBE,
            "h_alt_1_confirm": "throttle top channel and throttle share >= 0.50",
            "h_alt_1_narrow": "throttle top and share in [0.30,0.50) or throttle+largest tilt >= 0.70",
            "h_alt_1_falsify": "throttle is not top channel",
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
        "bisection_iters": int(args.bisection_iters),
        "discipline_statement": "No thresholds are changed after seeing alt_drift data.",
    }


def _write_progress(reports_dir: Path, point_rows: list[dict[str, Any]], query_rows: list[dict[str, Any]]) -> None:
    pd.DataFrame(point_rows).to_csv(reports_dir / "h_alt_1_boundary_points.csv", index=False)
    pd.DataFrame(query_rows).to_csv(reports_dir / "h_alt_1_boundary_query_repeats.csv", index=False)


def _write_probe_progress(
    reports_dir: Path,
    raw_rows: list[dict[str, Any]],
    probe_rows: list[dict[str, Any]],
) -> None:
    pd.DataFrame(raw_rows).to_csv(reports_dir / "h_alt_1_probe_raw.csv", index=False)
    pd.DataFrame(probe_rows).to_csv(reports_dir / "h_alt_1_probe_groups.csv", index=False)


def _write_report(path: Path, summary: dict[str, Any]) -> None:
    decision = summary["h_alt_1_decision"]
    lines = [
        "# alt_drift Phase 1 H-alt-1 Channel-Mass Probe",
        "",
        "Scope: confirmatory, PX4, alt_drift, px4_position.",
        "",
        f"Decision: **{decision['decision']}**.",
        f"Top channel: `{decision['top_channel']}`.",
        f"Throttle share: `{decision['throttle_share']:.6f}`.",
        f"Recommended A_alt for H-alt-2: `{','.join(decision['recommended_A_alt'])}`.",
        "",
        "| channel | weight | share |",
        "| --- | ---: | ---: |",
    ]
    for row in summary["channel_mass"]:
        lines.append(f"| {row['channel']} | {row['weight']:.6f} | {row['share']:.6f} |")
    lines.extend(
        [
            "",
            "Boundary point:",
            "",
            f"- theta hash: `{summary['boundary']['theta_hash']}`",
            f"- rho mean: `{summary['boundary'][f'rho_mean_{TARGET_PROPERTY}']:.6f}`",
            f"- rho std: `{summary['boundary'][f'rho_std_{TARGET_PROPERTY}']:.6f}`",
            f"- class: `{summary['boundary']['robustness_class']}`",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _remove_ulg_files(path: Path) -> None:
    for ulg_path in Path(path).rglob("*.ulg"):
        ulg_path.unlink()


def _maybe_int(value: str) -> str | int:
    try:
        return int(value)
    except ValueError:
        return value


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
