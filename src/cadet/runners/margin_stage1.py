from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pandas as pd

from cadet.config import ExperimentConfig, load_config
from cadet.groups import Group, build_groups
from cadet.input_model import project_theta
from cadet.metrics import effective_sparsity, topk_coverage
from cadet.query import theta_hash
from cadet.runners.fd_snapshot import _run_query_with_retry
from cadet.runners.margin_stage0 import _curve_shape, _fd_interior_summary, _load_candidate_theta
from cadet.runners.repeated_fd import run_repeated_snapshot


POINT_PROPS = {
    "V": "post_neutral_xy_velocity",
    "D": "post_neutral_xy_drift",
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "MarginSearch Stage 1: H1/RQ3 boundary measurements at xy_velocity and xy_drift "
            "points on px4_position seed 0."
        )
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/margin_stage1_v1")
    parser.add_argument("--stage0-run-dir", default="runs/margin_stage0_v1")
    parser.add_argument("--candidate-npz", default="runs/rq1_boundary_v0/candidates/candidate_thetas_refined.npz")
    parser.add_argument("--candidate-index", type=int, default=117)
    parser.add_argument("--v-bisection-iters", type=int, default=8)
    parser.add_argument("--verify-repeats", type=int, default=5)
    parser.add_argument("--fd-repeats", type=int, default=5)
    parser.add_argument("--crossing-repeats", type=int, default=3)
    parser.add_argument("--target-low", type=float, default=0.0)
    parser.add_argument("--target-high", type=float, default=0.2)
    parser.add_argument("--d-alpha", type=float, default=None)
    parser.add_argument("--locate-only", action="store_true")
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Stage 1 is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Stage 1 is frozen to seed 0")
    if args.crossing_repeats > args.fd_repeats:
        raise ValueError("--crossing-repeats cannot exceed --fd-repeats")

    t0 = time.monotonic()
    output_dir = Path(args.run_dir)
    reports_dir = output_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    config = _config_for_run_dir(load_config(args.config), output_dir)
    scenario = config.scenario_by_id(args.scenario)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(output_dir / "groups.csv", index=False)

    theta_117 = _load_candidate_theta(Path(args.candidate_npz), args.candidate_index)
    theta_117 = project_theta(theta_117, config)
    np.save(output_dir / "theta_117.npy", theta_117)

    v_curve_rows: list[dict] = []
    alpha_v, rho_v_single, alpha_v_unsafe, rho_v_unsafe = _bisect_alpha(
        theta_117,
        scenario,
        args.seed,
        POINT_PROPS["V"],
        config,
        output_dir,
        args.v_bisection_iters,
        v_curve_rows,
        point="V",
    )
    _write_rows(reports_dir / "stage1_pointV_alpha_curve.csv", v_curve_rows)

    alpha_d, d_curve_rows = _select_d_alpha(
        Path(args.stage0_run_dir),
        args.d_alpha,
        POINT_PROPS["D"],
        args.target_low,
        args.target_high,
    )
    d_eval_rows: list[dict] = []
    rho_d_single = _eval_alpha(
        alpha_d,
        theta_117,
        scenario,
        args.seed,
        POINT_PROPS["D"],
        config,
        output_dir,
        point="D",
        stage="selected",
        iteration=0,
        rows=d_eval_rows,
    )
    combined_d_curve_rows = d_curve_rows + d_eval_rows
    _write_rows(reports_dir / "stage1_pointD_alpha_curve.csv", combined_d_curve_rows)

    point_specs = {
        "V": {
            "alpha": alpha_v,
            "theta": alpha_v * theta_117,
            "boundary_property": POINT_PROPS["V"],
            "alpha_curve_rows": v_curve_rows,
            "unsafe_alpha": alpha_v_unsafe,
            "unsafe_rho_single": rho_v_unsafe,
            "rho_single": rho_v_single,
        },
        "D": {
            "alpha": alpha_d,
            "theta": alpha_d * theta_117,
            "boundary_property": POINT_PROPS["D"],
            "alpha_curve_rows": combined_d_curve_rows,
            "unsafe_alpha": None,
            "unsafe_rho_single": None,
            "rho_single": rho_d_single,
        },
    }

    point_summaries = {}
    for point_name, spec in point_specs.items():
        theta = np.asarray(spec["theta"], dtype=float)
        point_dir = output_dir / f"point_{point_name}"
        point_dir.mkdir(parents=True, exist_ok=True)
        np.save(point_dir / "theta.npy", theta)
        verify_rows = _verify_theta(
            theta,
            float(spec["alpha"]),
            scenario,
            args.seed,
            config,
            output_dir,
            args.verify_repeats,
            point_name,
        )
        verify_df = pd.DataFrame(verify_rows)
        verify_path = reports_dir / f"stage1_point{point_name}_verify.csv"
        verify_df.to_csv(verify_path, index=False)
        verify_summary = _summarize_verify(verify_df, scenario.properties)

        interior, group_rows = _fd_interior_summary(theta, config, groups)
        interior_path = reports_dir / f"stage1_point{point_name}_fd_interior_groups.csv"
        pd.DataFrame(group_rows).to_csv(interior_path, index=False)

        if args.locate_only:
            point_summaries[point_name] = {
                "point": point_name,
                "alpha": float(spec["alpha"]),
                "theta_hash": theta_hash(theta),
                "theta_path": str(point_dir / "theta.npy"),
                "boundary_property": str(spec["boundary_property"]),
                "rho_single": float(spec["rho_single"]),
                "unsafe_alpha": spec["unsafe_alpha"],
                "unsafe_rho_single": spec["unsafe_rho_single"],
                "verify_summary": verify_summary,
                "interior": interior,
                "curve_shape": _curve_shape(spec["alpha_curve_rows"], str(spec["boundary_property"])),
                "verify_rows": str(verify_path),
                "interior_rows": str(interior_path),
            }
            continue

        phase_tag = f"stage1_point{point_name}_fd"
        print(
            f"stage1_fd_start point={point_name} property={spec['boundary_property']} "
            f"alpha={float(spec['alpha']):.8f} repeats={args.fd_repeats}",
            flush=True,
        )
        snap_dir = run_repeated_snapshot(
            theta,
            scenario,
            args.seed,
            args.fd_repeats,
            config,
            output_dir,
            groups,
            phase_tag,
            cache_namespace=phase_tag,
        )
        fd_metrics = _extended_fd_metrics(snap_dir, scenario.properties, groups, args.fd_repeats)
        fd_metrics_path = reports_dir / f"stage1_point{point_name}_fd_metrics.csv"
        pd.DataFrame(fd_metrics).to_csv(fd_metrics_path, index=False)

        crossing_rows, crossing_summary = _crossing_count(
            snap_dir,
            str(spec["boundary_property"]),
            args.crossing_repeats,
            verify_summary,
        )
        crossing_path = reports_dir / f"stage1_point{point_name}_crossing.csv"
        pd.DataFrame(crossing_rows).to_csv(crossing_path, index=False)

        curve_summary = _curve_shape(spec["alpha_curve_rows"], str(spec["boundary_property"]))
        point_summaries[point_name] = {
            "point": point_name,
            "alpha": float(spec["alpha"]),
            "theta_hash": theta_hash(theta),
            "theta_path": str(point_dir / "theta.npy"),
            "boundary_property": str(spec["boundary_property"]),
            "rho_single": float(spec["rho_single"]),
            "unsafe_alpha": spec["unsafe_alpha"],
            "unsafe_rho_single": spec["unsafe_rho_single"],
            "verify_summary": verify_summary,
            "interior": interior,
            "curve_shape": curve_summary,
            "snapshot_dir": str(snap_dir),
            "fd_metrics": fd_metrics,
            "crossing_summary": crossing_summary,
            "verify_rows": str(verify_path),
            "fd_metrics_rows": str(fd_metrics_path),
            "crossing_rows": str(crossing_path),
            "interior_rows": str(interior_path),
        }

    summary = {
        "status": "complete",
        "scenario_id": scenario.id,
        "seed": args.seed,
        "candidate_index": args.candidate_index,
        "theta_117_hash": theta_hash(theta_117),
        "target_rho_interval": [args.target_low, args.target_high],
        "verify_repeats": args.verify_repeats,
        "fd_repeats": args.fd_repeats,
        "crossing_repeats_from_fd": args.crossing_repeats,
        "points": point_summaries,
        "elapsed_wall_time_s": time.monotonic() - t0,
    }
    if args.locate_only:
        summary["status"] = "locate_only"
        _write_json(reports_dir / "stage1_locate_summary.json", summary)
        _write_locate_report(reports_dir / "stage1_locate_report.md", summary)
        print(
            f"stage1_locate_complete report={reports_dir / 'stage1_locate_report.md'} "
            f"elapsed={summary['elapsed_wall_time_s']:.1f}s",
            flush=True,
        )
        return
    _write_json(reports_dir / "stage1_summary.json", summary)
    _write_report(reports_dir / "stage1_report.md", summary)
    print(
        f"stage1_complete report={reports_dir / 'stage1_report.md'} "
        f"elapsed={summary['elapsed_wall_time_s']:.1f}s",
        flush=True,
    )


def _config_for_run_dir(config: ExperimentConfig, output_dir: Path) -> ExperimentConfig:
    logging = dict(config.logging)
    logging["jsonl"] = str(output_dir / "logs" / "queries.jsonl")
    return replace(config, experiment_id=output_dir.name, logging=logging)


def _bisect_alpha(
    theta_base: np.ndarray,
    scenario,
    seed: int,
    prop: str,
    config: ExperimentConfig,
    output_dir: Path,
    iterations: int,
    rows: list[dict],
    *,
    point: str,
) -> tuple[float, float, float, float]:
    low_alpha = 0.0
    low_rho = _eval_alpha(low_alpha, theta_base, scenario, seed, prop, config, output_dir, point, "bracket", 0, rows)
    high_alpha = 1.0
    high_rho = _eval_alpha(high_alpha, theta_base, scenario, seed, prop, config, output_dir, point, "bracket", 1, rows)
    if low_rho < 0.0 or high_rho >= 0.0:
        raise RuntimeError(f"{point} alpha bracket failed for {prop}: rho(0)={low_rho:.6f}, rho(1)={high_rho:.6f}")

    for iteration in range(iterations):
        mid_alpha = 0.5 * (low_alpha + high_alpha)
        mid_rho = _eval_alpha(
            mid_alpha,
            theta_base,
            scenario,
            seed,
            prop,
            config,
            output_dir,
            point,
            "bisection",
            iteration,
            rows,
        )
        if mid_rho >= 0.0:
            low_alpha = mid_alpha
            low_rho = mid_rho
        else:
            high_alpha = mid_alpha
            high_rho = mid_rho
    return low_alpha, low_rho, high_alpha, high_rho


def _eval_alpha(
    alpha: float,
    theta_base: np.ndarray,
    scenario,
    seed: int,
    prop: str,
    config: ExperimentConfig,
    output_dir: Path,
    point: str,
    stage: str,
    iteration: int,
    rows: list[dict],
) -> float:
    theta = alpha * theta_base
    projected = project_theta(theta, config)
    projection_linf = float(np.max(np.abs(projected - theta))) if theta.size else 0.0
    cache_tag = f"stage1_point{point}_alpha_{stage}_{iteration:02d}_{alpha:.8f}"
    result = _run_query_with_retry(
        theta,
        scenario,
        seed,
        "margin_stage1_alpha",
        output_dir,
        config,
        cache_tag=cache_tag,
        use_cache=True,
    )
    row = {
        "point": point,
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
        f"stage1_alpha point={point} {stage}[{iteration}] alpha={alpha:.8f} "
        + " ".join(f"{name}={result.robustness[name]:.6f}" for name in scenario.properties),
        flush=True,
    )
    return float(result.robustness[prop])


def _select_d_alpha(
    stage0_run_dir: Path,
    explicit_alpha: float | None,
    prop: str,
    target_low: float,
    target_high: float,
) -> tuple[float, list[dict]]:
    if explicit_alpha is not None:
        return explicit_alpha, []
    curve_path = stage0_run_dir / "reports" / "stage0_alpha_curve.csv"
    summary_path = stage0_run_dir / "reports" / "stage0_summary.json"
    if not curve_path.exists() or not summary_path.exists():
        raise FileNotFoundError("Stage 0 curve/summary not found; pass --d-alpha explicitly")
    stage0_summary = json.loads(summary_path.read_text(encoding="utf-8"))
    alpha_star = float(stage0_summary["alpha_star_safe"])
    curve_df = pd.read_csv(curve_path)
    rho_col = f"robustness_{prop}"
    candidates = curve_df[
        (curve_df["alpha"] < alpha_star)
        & (curve_df[rho_col] >= target_low)
        & (curve_df[rho_col] <= target_high)
    ].sort_values("alpha", ascending=False)
    if candidates.empty:
        alpha = alpha_star
    else:
        alpha = float(candidates.iloc[0]["alpha"])
    rows = curve_df.to_dict(orient="records")
    for row in rows:
        row["point"] = "D"
        row["source"] = "stage0_curve"
    return alpha, rows


def _verify_theta(
    theta: np.ndarray,
    alpha: float,
    scenario,
    seed: int,
    config: ExperimentConfig,
    output_dir: Path,
    repeats: int,
    point: str,
) -> list[dict]:
    rows = []
    for repeat_idx in range(repeats):
        cache_tag = f"stage1_point{point}_verify_alpha_{alpha:.8f}_repeat{repeat_idx}"
        result = _run_query_with_retry(
            theta,
            scenario,
            seed,
            "margin_stage1_verify",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        row = {
            "point": point,
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
            f"stage1_verify point={point} repeat={repeat_idx}/{repeats - 1} "
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


def _extended_fd_metrics(snap_dir: Path, properties: list[str], groups: list[Group], repeats: int) -> list[dict]:
    rows = []
    for prop in properties:
        matrix = []
        for repeat_idx in range(repeats):
            path = snap_dir / f"repeat_{repeat_idx}" / f"gradient_{prop}.csv"
            df = pd.read_csv(path).sort_values("group_id")
            matrix.append(df["g"].to_numpy(dtype=float))
        values = np.vstack(matrix)
        mean_g = np.mean(values, axis=0)
        std_g = np.std(values, axis=0, ddof=1) if repeats > 1 else np.zeros(values.shape[1])
        se_g = std_g / math.sqrt(repeats) if repeats > 0 else std_g
        abs_mean = np.abs(mean_g)
        order = list(np.argsort(abs_mean)[::-1])
        sum_abs = float(np.sum(abs_mean))
        noise_after_l1 = float(np.sum(se_g))
        row = {
            "property": prop,
            "repeat_count": repeats,
            "top4_coverage": topk_coverage(abs_mean, 4),
            "top8_coverage": topk_coverage(abs_mean, 8),
            "top16_coverage": topk_coverage(abs_mean, 16),
            "effective_sparsity": effective_sparsity(mean_g),
            "max_abs_mean_g": float(np.max(abs_mean)) if abs_mean.size else 0.0,
            "sum_abs_mean_g": sum_abs,
            "repeat_se_l1_after": noise_after_l1,
            "noise_after_over_sum": noise_after_l1 / sum_abs if sum_abs > 0 else float("nan"),
            "top1_group": int(order[0]) if order else "",
            "top4_groups": ",".join(str(int(i)) for i in order[:4]),
            "top8_groups": ",".join(str(int(i)) for i in order[:8]),
            "top16_groups": ",".join(str(int(i)) for i in order[:16]),
        }
        for k, gid in enumerate(order[:16], start=1):
            group = groups[int(gid)]
            row[f"top{k}_group_detail"] = f"{int(gid)}:{group.channel}:w{group.window_id}"
        rows.append(row)
    return rows


def _crossing_count(
    snap_dir: Path,
    prop: str,
    crossing_repeats: int,
    verify_summary: list[dict],
) -> tuple[list[dict], dict]:
    boundary_verify = next(row for row in verify_summary if row["property"] == prop)
    base_mean = float(boundary_verify["mean"])
    query_rows = []
    for repeat_idx in range(crossing_repeats):
        path = snap_dir / f"repeat_{repeat_idx}" / "query_metadata.csv"
        df = pd.read_csv(path)
        df["repeat_idx"] = repeat_idx
        query_rows.append(df)
    all_queries = pd.concat(query_rows, ignore_index=True)
    rho_col = f"robustness_{prop}"
    rows = []
    for (group_id, sign), part in all_queries.groupby(["group_id", "sign"]):
        values = part[rho_col].to_numpy(dtype=float)
        count_nonpositive = int(np.sum(values <= 0.0))
        mean_rho = float(np.mean(values))
        rows.append(
            {
                "group_id": int(group_id),
                "sign": sign,
                "repeat_count": int(values.size),
                "rho_mean": mean_rho,
                "rho_std": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
                "rho_min": float(np.min(values)),
                "rho_max": float(np.max(values)),
                "rho_delta_vs_base_mean": mean_rho - base_mean,
                "count_nonpositive": count_nonpositive,
                "crossing_mean_negative": bool(mean_rho < 0.0),
                "crossing_majority_nonpositive": bool(count_nonpositive >= math.ceil(values.size / 2)),
                "crossing_any_nonpositive": bool(count_nonpositive > 0),
            }
        )
    crossing_df = pd.DataFrame(rows)
    mean_negative = crossing_df[crossing_df["crossing_mean_negative"]]
    majority = crossing_df[crossing_df["crossing_majority_nonpositive"]]
    any_nonpositive = crossing_df[crossing_df["crossing_any_nonpositive"]]
    summary = {
        "property": prop,
        "base_mean_rho": base_mean,
        "repeat_count": crossing_repeats,
        "crossing_steps_mean_negative": int(len(mean_negative)),
        "crossing_groups_mean_negative": int(mean_negative["group_id"].nunique()),
        "crossing_steps_majority_nonpositive": int(len(majority)),
        "crossing_groups_majority_nonpositive": int(majority["group_id"].nunique()),
        "crossing_steps_any_nonpositive": int(len(any_nonpositive)),
        "crossing_groups_any_nonpositive": int(any_nonpositive["group_id"].nunique()),
        "total_steps": int(len(crossing_df)),
        "total_groups": int(crossing_df["group_id"].nunique()),
        "mean_negative_group_ids": ",".join(str(int(g)) for g in sorted(mean_negative["group_id"].unique())),
    }
    return rows, summary


def _write_rows(path: Path, rows: list[dict]) -> None:
    pd.DataFrame(rows).to_csv(path, index=False)


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _boundary_prop_metric(point_summary: dict) -> dict:
    prop = point_summary["boundary_property"]
    return next(row for row in point_summary["fd_metrics"] if row["property"] == prop)


def _verify_prop(point_summary: dict, prop: str) -> dict:
    return next(row for row in point_summary["verify_summary"] if row["property"] == prop)


def _write_report(path: Path, summary: dict) -> None:
    lines = [
        "# MarginSearch Stage 1 Report",
        "",
        f"Scope: `{summary['scenario_id']}`, seed {summary['seed']}, candidate {summary['candidate_index']}.",
        "",
        "## Boundary Points",
        "",
        "| point | property | alpha | single rho | J=5 mean rho | interior FD clean | curve shape |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for point_name in ["V", "D"]:
        point = summary["points"][point_name]
        prop = point["boundary_property"]
        verify = _verify_prop(point, prop)
        interior = point["interior"]
        lines.append(
            f"| {point_name} | {prop} | {point['alpha']:.8f} | {point['rho_single']:.6f} | "
            f"{verify['mean']:.6f} | {interior['fd_clean_two_sided_groups']}/{interior['group_count']} | "
            f"{point['curve_shape']['label']} |"
        )
    lines.extend(
        [
            "",
            "## H1 Metrics",
            "",
            "| point | property | eff sparsity | top4 | top8 | top16 | noise_after/sum | crossing groups | crossing steps | top8 groups |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for point_name in ["V", "D"]:
        point = summary["points"][point_name]
        metric = _boundary_prop_metric(point)
        crossing = point["crossing_summary"]
        lines.append(
            f"| {point_name} | {point['boundary_property']} | {metric['effective_sparsity']:.3f} | "
            f"{metric['top4_coverage']:.3f} | {metric['top8_coverage']:.3f} | {metric['top16_coverage']:.3f} | "
            f"{metric['noise_after_over_sum']:.3f} | "
            f"{crossing['crossing_groups_mean_negative']}/{crossing['total_groups']} | "
            f"{crossing['crossing_steps_mean_negative']}/{crossing['total_steps']} | "
            f"{metric['top8_groups']} |"
        )
    lines.extend(
        [
            "",
            "## J=5 Robustness",
            "",
            "| point | property | mean | std | min | max | nonpositive |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for point_name in ["V", "D"]:
        for row in summary["points"][point_name]["verify_summary"]:
            lines.append(
                f"| {point_name} | {row['property']} | {row['mean']:.6f} | {row['std']:.6f} | "
                f"{row['min']:.6f} | {row['max']:.6f} | {row['count_nonpositive']}/{row['repeats']} |"
            )
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            f"- summary JSON: `{path.with_name('stage1_summary.json')}`",
        ]
    )
    for point_name in ["V", "D"]:
        point = summary["points"][point_name]
        lines.extend(
            [
                f"- point {point_name} theta: `{point['theta_path']}`",
                f"- point {point_name} snapshot: `{point['snapshot_dir']}`",
                f"- point {point_name} FD metrics: `{point['fd_metrics_rows']}`",
                f"- point {point_name} crossing rows: `{point['crossing_rows']}`",
            ]
        )
    lines.extend(
        [
            "",
            f"Elapsed wall time: {summary['elapsed_wall_time_s']:.1f}s.",
            "",
            "Stop point: Stage 2 not started.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_locate_report(path: Path, summary: dict) -> None:
    lines = [
        "# MarginSearch Stage 1 Locate Report",
        "",
        f"Scope: `{summary['scenario_id']}`, seed {summary['seed']}, candidate {summary['candidate_index']}.",
        "",
        "| point | property | alpha | single rho | J=5 mean rho | interior FD clean | curve shape |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for point_name in ["V", "D"]:
        point = summary["points"][point_name]
        prop = point["boundary_property"]
        verify = _verify_prop(point, prop)
        interior = point["interior"]
        lines.append(
            f"| {point_name} | {prop} | {point['alpha']:.8f} | {point['rho_single']:.6f} | "
            f"{verify['mean']:.6f} | {interior['fd_clean_two_sided_groups']}/{interior['group_count']} | "
            f"{point['curve_shape']['label']} |"
        )
    lines.extend(
        [
            "",
            "## J=5 Robustness",
            "",
            "| point | property | mean | std | min | max | nonpositive |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for point_name in ["V", "D"]:
        for row in summary["points"][point_name]["verify_summary"]:
            lines.append(
                f"| {point_name} | {row['property']} | {row['mean']:.6f} | {row['std']:.6f} | "
                f"{row['min']:.6f} | {row['max']:.6f} | {row['count_nonpositive']}/{row['repeats']} |"
            )
    lines.extend(
        [
            "",
            "Locate-only run: FD snapshots and crossing counts were not started.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
