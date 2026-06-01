from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pandas as pd

from sparsepilot.config import ExperimentConfig, load_config
from sparsepilot.groups import Group, build_groups
from sparsepilot.input_model import project_theta
from sparsepilot.query import theta_hash
from sparsepilot.runners.fd_snapshot import _run_query_with_retry


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "MarginSearch Stage 0 gate: shrink candidate 117 along alpha*theta, "
            "find a just-safe interior boundary-side point, and verify it with J repeats."
        )
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/margin_stage0_v1")
    parser.add_argument("--candidate-npz", default="runs/rq1_boundary_v0/candidates/candidate_thetas_refined.npz")
    parser.add_argument("--candidate-index", type=int, default=117)
    parser.add_argument("--property", default="post_neutral_xy_drift")
    parser.add_argument("--bisection-iters", type=int, default=8)
    parser.add_argument("--verify-repeats", type=int, default=5)
    parser.add_argument("--target-rho-low", type=float, default=0.0)
    parser.add_argument("--target-rho-high", type=float, default=0.3)
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Stage 0 is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Stage 0 is frozen to seed 0")

    output_dir = Path(args.run_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "reports").mkdir(parents=True, exist_ok=True)
    config = _config_for_run_dir(load_config(args.config), output_dir)
    scenario = config.scenario_by_id(args.scenario)
    if args.property not in scenario.properties:
        raise ValueError(f"{args.property} is not enabled for {scenario.id}")

    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(output_dir / "groups.csv", index=False)

    theta_117 = _load_candidate_theta(Path(args.candidate_npz), args.candidate_index)
    projected = project_theta(theta_117, config)
    if not np.allclose(projected, theta_117, atol=1e-9, rtol=0.0):
        max_projection_delta = float(np.max(np.abs(projected - theta_117)))
        raise ValueError(f"candidate theta is not feasible before scaling; projection delta={max_projection_delta}")
    np.save(output_dir / "theta_117.npy", theta_117)

    t0 = time.monotonic()
    curve_rows: list[dict] = []

    low_alpha = 0.0
    low_rho = _eval_alpha(low_alpha, theta_117, scenario, args.seed, args.property, config, output_dir, "bracket", 0, curve_rows)
    high_alpha = 1.0
    high_rho = _eval_alpha(high_alpha, theta_117, scenario, args.seed, args.property, config, output_dir, "bracket", 1, curve_rows)
    if low_rho < 0.0 or high_rho >= 0.0:
        _write_curve(output_dir, curve_rows)
        summary = {
            "status": "bracket_failed",
            "property": args.property,
            "low_alpha": low_alpha,
            "low_rho": low_rho,
            "high_alpha": high_alpha,
            "high_rho": high_rho,
            "elapsed_wall_time_s": time.monotonic() - t0,
        }
        _write_json(output_dir / "reports" / "stage0_summary.json", summary)
        raise RuntimeError(f"alpha bracket failed: rho(0)={low_rho:.6f}, rho(1)={high_rho:.6f}")

    for iteration in range(args.bisection_iters):
        mid_alpha = 0.5 * (low_alpha + high_alpha)
        mid_rho = _eval_alpha(
            mid_alpha,
            theta_117,
            scenario,
            args.seed,
            args.property,
            config,
            output_dir,
            "bisection",
            iteration,
            curve_rows,
        )
        if mid_rho >= 0.0:
            low_alpha = mid_alpha
            low_rho = mid_rho
        else:
            high_alpha = mid_alpha
            high_rho = mid_rho

    theta_b0 = low_alpha * theta_117
    np.save(output_dir / "theta_b0.npy", theta_b0)
    _write_curve(output_dir, curve_rows)

    verify_rows = _verify_theta(theta_b0, low_alpha, scenario, args.seed, config, output_dir, args.verify_repeats)
    verify_df = pd.DataFrame(verify_rows)
    verify_df.to_csv(output_dir / "reports" / "stage0_verify.csv", index=False)

    verify_summary = _summarize_verify(verify_df, scenario.properties)
    interior, group_rows = _fd_interior_summary(theta_b0, config, groups)
    pd.DataFrame(group_rows).to_csv(output_dir / "reports" / "stage0_fd_interior_groups.csv", index=False)

    curve_shape = _curve_shape(curve_rows, args.property)
    target_hit = bool(args.target_rho_low <= low_rho <= args.target_rho_high)
    summary = {
        "status": "complete",
        "scenario_id": scenario.id,
        "seed": args.seed,
        "candidate_index": args.candidate_index,
        "theta_117_hash": theta_hash(theta_117),
        "property": args.property,
        "alpha_star_safe": low_alpha,
        "rho_alpha_star_single": low_rho,
        "alpha_unsafe": high_alpha,
        "rho_alpha_unsafe_single": high_rho,
        "target_rho_interval": [args.target_rho_low, args.target_rho_high],
        "target_hit": target_hit,
        "verify_summary": verify_summary,
        "interior": interior,
        "curve_shape": curve_shape,
        "theta_117": str(output_dir / "theta_117.npy"),
        "theta_b0": str(output_dir / "theta_b0.npy"),
        "alpha_curve": str(output_dir / "reports" / "stage0_alpha_curve.csv"),
        "verify_rows": str(output_dir / "reports" / "stage0_verify.csv"),
        "fd_interior_rows": str(output_dir / "reports" / "stage0_fd_interior_groups.csv"),
        "elapsed_wall_time_s": time.monotonic() - t0,
    }
    _write_json(output_dir / "reports" / "stage0_summary.json", summary)
    _write_report(output_dir / "reports" / "stage0_report.md", summary)
    print(
        "stage0_complete "
        f"alpha_star={low_alpha:.6f} rho={low_rho:.6f} "
        f"target_hit={target_hit} fd_clean={interior['fd_clean_two_sided_groups']}/{interior['group_count']} "
        f"report={output_dir / 'reports' / 'stage0_report.md'}",
        flush=True,
    )


def _config_for_run_dir(config: ExperimentConfig, output_dir: Path) -> ExperimentConfig:
    logging = dict(config.logging)
    logging["jsonl"] = str(output_dir / "logs" / "queries.jsonl")
    return replace(config, experiment_id=output_dir.name, logging=logging)


def _load_candidate_theta(path: Path, candidate_index: int) -> np.ndarray:
    key = f"candidate_{candidate_index:03d}"
    with np.load(path) as data:
        if key not in data.files:
            raise KeyError(f"{key} not found in {path}")
        return np.asarray(data[key], dtype=float)


def _eval_alpha(
    alpha: float,
    theta_base: np.ndarray,
    scenario,
    seed: int,
    prop: str,
    config: ExperimentConfig,
    output_dir: Path,
    stage: str,
    iteration: int,
    rows: list[dict],
) -> float:
    theta = alpha * theta_base
    projected = project_theta(theta, config)
    projection_linf = float(np.max(np.abs(projected - theta))) if theta.size else 0.0
    cache_tag = f"stage0_alpha_{stage}_{iteration:02d}_{alpha:.8f}"
    result = _run_query_with_retry(
        theta,
        scenario,
        seed,
        "margin_stage0_alpha",
        output_dir,
        config,
        cache_tag=cache_tag,
        use_cache=True,
    )
    row = {
        "stage": stage,
        "iteration": iteration,
        "alpha": alpha,
        "theta_hash": result.theta_hash,
        "query_id": result.query_id,
        "cache_tag": cache_tag,
        "projection_linf": projection_linf,
        "query_total_wall_time_s": float(result.metadata.get("total_wall_time_s", math.nan)),
    }
    for name, value in result.robustness.items():
        row[f"robustness_{name}"] = float(value)
    rows.append(row)
    print(
        "stage0_alpha "
        f"{stage}[{iteration}] alpha={alpha:.8f} "
        + " ".join(f"{name}={result.robustness[name]:.6f}" for name in scenario.properties),
        flush=True,
    )
    return float(result.robustness[prop])


def _write_curve(output_dir: Path, rows: list[dict]) -> None:
    pd.DataFrame(rows).to_csv(output_dir / "reports" / "stage0_alpha_curve.csv", index=False)


def _verify_theta(
    theta: np.ndarray,
    alpha: float,
    scenario,
    seed: int,
    config: ExperimentConfig,
    output_dir: Path,
    repeats: int,
) -> list[dict]:
    rows = []
    for repeat_idx in range(repeats):
        cache_tag = f"stage0_verify_alpha_{alpha:.8f}_repeat{repeat_idx}"
        result = _run_query_with_retry(
            theta,
            scenario,
            seed,
            "margin_stage0_verify",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        row = {
            "repeat_idx": repeat_idx,
            "alpha": alpha,
            "theta_hash": result.theta_hash,
            "query_id": result.query_id,
            "cache_tag": cache_tag,
            "query_total_wall_time_s": float(result.metadata.get("total_wall_time_s", math.nan)),
        }
        for prop, value in result.robustness.items():
            row[f"robustness_{prop}"] = float(value)
        rows.append(row)
        print(
            "stage0_verify "
            f"repeat={repeat_idx}/{repeats - 1} "
            + " ".join(f"{prop}={result.robustness[prop]:.6f}" for prop in scenario.properties),
            flush=True,
        )
    return rows


def _summarize_verify(verify_df: pd.DataFrame, properties: list[str]) -> list[dict]:
    rows = []
    for prop in properties:
        values = verify_df[f"robustness_{prop}"].to_numpy(dtype=float)
        rows.append(
            {
                "property": prop,
                "mean": float(np.mean(values)),
                "std": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
                "min": float(np.min(values)),
                "max": float(np.max(values)),
                "count_nonpositive": int(np.sum(values <= 0.0)),
                "repeats": int(values.size),
            }
        )
    return rows


def _fd_interior_summary(theta: np.ndarray, config: ExperimentConfig, groups: list[Group]) -> tuple[dict, list[dict]]:
    theta = np.asarray(theta, dtype=float)
    delta = float(config.input["perturb_delta"])
    min_value = float(config.input["min_value"])
    max_value = float(config.input["max_value"])
    value_margin = np.minimum(theta - min_value, max_value - theta)
    near_upper = theta >= max_value - 0.02
    near_lower = theta <= min_value + 0.02

    clean_group_ids: list[int] = []
    blocked_group_ids: list[int] = []
    group_rows: list[dict] = []
    for group in groups:
        raw_plus = theta.copy()
        raw_minus = theta.copy()
        raw_plus[group.group_id] += delta
        raw_minus[group.group_id] -= delta
        plus_clean = np.allclose(project_theta(raw_plus, config), raw_plus, atol=1e-9, rtol=0.0)
        minus_clean = np.allclose(project_theta(raw_minus, config), raw_minus, atol=1e-9, rtol=0.0)
        clean = bool(plus_clean and minus_clean)
        if clean:
            clean_group_ids.append(group.group_id)
        else:
            blocked_group_ids.append(group.group_id)
        group_rows.append(
            {
                **group.__dict__,
                "theta": float(theta[group.group_id]),
                "value_margin": float(value_margin[group.group_id]),
                "value_margin_gt_delta": bool(value_margin[group.group_id] > delta),
                "fd_plus_clean": bool(plus_clean),
                "fd_minus_clean": bool(minus_clean),
                "fd_two_sided_clean": clean,
            }
        )

    summary = {
        "delta": delta,
        "group_count": len(groups),
        "value_margin_gt_delta_count": int(np.sum(value_margin > delta)),
        "value_margin_le_delta_count": int(np.sum(value_margin <= delta)),
        "fd_clean_two_sided_groups": len(clean_group_ids),
        "fd_blocked_groups": len(blocked_group_ids),
        "fd_clean_group_ids": ",".join(str(int(g)) for g in clean_group_ids),
        "fd_blocked_group_ids": ",".join(str(int(g)) for g in blocked_group_ids),
        "fd_interior": len(clean_group_ids) == len(groups),
        "near_upper_count": int(np.sum(near_upper)),
        "near_lower_count": int(np.sum(near_lower)),
        "near_abs_limit_count": int(np.sum(near_upper | near_lower)),
        "max_abs_theta": float(np.max(np.abs(theta))) if theta.size else 0.0,
        "mean_abs_theta": float(np.mean(np.abs(theta))) if theta.size else 0.0,
        "min_value_margin": float(np.min(value_margin)) if theta.size else 0.0,
        "median_value_margin": float(np.median(value_margin)) if theta.size else 0.0,
    }
    return summary, group_rows


def _curve_shape(rows: list[dict], prop: str) -> dict:
    if len(rows) < 3:
        return {"label": "insufficient_points"}
    df = pd.DataFrame(rows).sort_values("alpha")
    values = df[["alpha", f"robustness_{prop}"]].drop_duplicates(subset=["alpha"]).to_numpy(dtype=float)
    if values.shape[0] < 3:
        return {"label": "insufficient_unique_points"}
    drops = []
    for (a0, r0), (a1, r1) in zip(values[:-1], values[1:]):
        if a1 <= a0:
            continue
        drops.append(max(0.0, float(r0 - r1)))
    total_drop = float(sum(drops))
    max_drop = float(max(drops)) if drops else 0.0
    concentration = max_drop / total_drop if total_drop > 0 else 0.0
    if concentration >= 0.60:
        label = "cliff_like"
    elif concentration <= 0.35:
        label = "ramp_like"
    else:
        label = "mixed"
    return {
        "label": label,
        "drop_concentration_max_segment": concentration,
        "total_observed_drop": total_drop,
        "max_segment_drop": max_drop,
        "unique_alpha_count": int(values.shape[0]),
    }


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _write_report(path: Path, summary: dict) -> None:
    interior = summary["interior"]
    lines = [
        "# MarginSearch Stage 0 Report",
        "",
        f"Scope: `{summary['scenario_id']}`, seed {summary['seed']}, theta = alpha * candidate {summary['candidate_index']}.",
        "",
        "## Alpha Gate",
        "",
        "| alpha* safe | single rho | unsafe alpha | unsafe single rho | target rho interval | target hit | curve shape |",
        "| ---: | ---: | ---: | ---: | --- | --- | --- |",
        (
            f"| {summary['alpha_star_safe']:.8f} | {summary['rho_alpha_star_single']:.6f} | "
            f"{summary['alpha_unsafe']:.8f} | {summary['rho_alpha_unsafe_single']:.6f} | "
            f"[{summary['target_rho_interval'][0]:.3f}, {summary['target_rho_interval'][1]:.3f}] | "
            f"{summary['target_hit']} | {summary['curve_shape']['label']} |"
        ),
        "",
        "## J=5 Verification",
        "",
        "| property | mean rho | std | min | max | nonpositive repeats |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary["verify_summary"]:
        lines.append(
            f"| {row['property']} | {row['mean']:.6f} | {row['std']:.6f} | {row['min']:.6f} | "
            f"{row['max']:.6f} | {row['count_nonpositive']}/{row['repeats']} |"
        )
    lines.extend(
        [
            "",
            "## Interior Check",
            "",
            "| value margin > delta | clean two-sided FD groups | blocked groups | max abs theta | mean abs theta | min value margin | FD interior |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | --- |",
            (
                f"| {interior['value_margin_gt_delta_count']}/{interior['group_count']} | "
                f"{interior['fd_clean_two_sided_groups']}/{interior['group_count']} | "
                f"{interior['fd_blocked_groups']} | {interior['max_abs_theta']:.6f} | "
                f"{interior['mean_abs_theta']:.6f} | {interior['min_value_margin']:.6f} | "
                f"{interior['fd_interior']} |"
            ),
            "",
            "## Artifacts",
            "",
            f"- theta_117: `{summary['theta_117']}`",
            f"- theta_b0: `{summary['theta_b0']}`",
            f"- alpha curve: `{summary['alpha_curve']}`",
            f"- J=5 rows: `{summary['verify_rows']}`",
            f"- FD-interior rows: `{summary['fd_interior_rows']}`",
            f"- summary JSON: `{path.with_name('stage0_summary.json')}`",
            "",
            f"Elapsed wall time: {summary['elapsed_wall_time_s']:.1f}s.",
            "",
            "Stop point: wait for author sign-off before Stage 1.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
