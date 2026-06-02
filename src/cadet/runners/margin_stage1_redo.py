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
from cadet.runners.margin_stage0 import _fd_interior_summary, _load_candidate_theta
from cadet.runners.margin_stage1 import _summarize_verify
from cadet.runners.repeated_fd import run_repeated_snapshot


PROPS_FOR_CURVE = ["post_neutral_xy_velocity", "post_neutral_alt_drift"]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Stage 1-redo: boundary-noise-robust xy_velocity normal readout at Point V."
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/margin_stage1_redo_v1")
    parser.add_argument("--stage1-run-dir", default="runs/margin_stage1_v1")
    parser.add_argument("--candidate-npz", default="runs/rq1_boundary_v0/candidates/candidate_thetas_refined.npz")
    parser.add_argument("--candidate-index", type=int, default=117)
    parser.add_argument("--v-alpha", type=float, default=None)
    parser.add_argument("--delta-probe", type=float, default=0.2)
    parser.add_argument("--probe-repeats", type=int, default=3)
    parser.add_argument("--intermediate-repeats", type=int, default=3)
    parser.add_argument("--v-repeats", type=int, default=5)
    parser.add_argument("--alpha", action="append", type=float, default=None)
    parser.add_argument(
        "--center-snapshot",
        default=(
            "runs/archive/rq1_zero_theta_sph_rejected/legacy_rq1_minimal_v0/snapshots/"
            "px4_position_seed0_7b6436b0c98f6238_phase2_px4_seed0_j5_denoise_j5"
        ),
    )
    parser.add_argument(
        "--v-snapshot",
        default="runs/margin_stage1_v1/snapshots/px4_position_seed0_3248eac8d31a9542_stage1_pointV_fd_j5",
    )
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Stage 1-redo is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Stage 1-redo is frozen to seed 0")

    t0 = time.monotonic()
    output_dir = Path(args.run_dir)
    reports_dir = output_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    config = _config_for_run_dir(load_config(args.config), output_dir)
    scenario = config.scenario_by_id(args.scenario)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(output_dir / "groups.csv", index=False)

    theta_117 = project_theta(_load_candidate_theta(Path(args.candidate_npz), args.candidate_index), config)
    np.save(output_dir / "theta_117.npy", theta_117)

    stage1_summary = _load_stage1_summary(Path(args.stage1_run_dir))
    v_alpha = float(args.v_alpha if args.v_alpha is not None else stage1_summary["points"]["V"]["alpha"])
    theta_v = v_alpha * theta_117
    np.save(output_dir / "theta_V.npy", theta_v)
    v_interior, v_group_rows = _fd_interior_summary(theta_v, config, groups)
    pd.DataFrame(v_group_rows).to_csv(reports_dir / "redo_pointV_fd_interior_delta008_groups.csv", index=False)
    probe_margin_rows = _probe_margin_rows(theta_v, args.delta_probe, config, groups)
    pd.DataFrame(probe_margin_rows).to_csv(reports_dir / "redo_pointV_probe_margin_groups.csv", index=False)

    print(
        f"redo_probe_start alpha_V={v_alpha:.8f} delta_probe={args.delta_probe:.3f} "
        f"probe_repeats={args.probe_repeats}",
        flush=True,
    )
    v_verify_rows = _verify_theta(theta_v, v_alpha, scenario, args.seed, config, output_dir, args.v_repeats, "V")
    v_verify = _summarize_verify(pd.DataFrame(v_verify_rows), scenario.properties)
    pd.DataFrame(v_verify_rows).to_csv(reports_dir / "redo_pointV_verify.csv", index=False)
    probe_rows, probe_summary, probe_channel, probe_window = _run_directional_probe(
        theta_v,
        v_alpha,
        scenario,
        args.seed,
        config,
        output_dir,
        groups,
        args.delta_probe,
        args.probe_repeats,
        _verify_mean(v_verify, "post_neutral_xy_velocity"),
    )
    pd.DataFrame(probe_rows).to_csv(reports_dir / "redo_pointV_delta020_directional_probe.csv", index=False)
    pd.DataFrame(probe_channel).to_csv(reports_dir / "redo_pointV_delta020_channel_marginal.csv", index=False)
    pd.DataFrame(probe_window).to_csv(reports_dir / "redo_pointV_delta020_window_marginal.csv", index=False)

    alphas = args.alpha or [0.0, 0.15, 0.25, 0.35, v_alpha]
    alpha_rows = []
    alpha_metric_rows = []
    for alpha in alphas:
        alpha = float(alpha)
        theta = alpha * theta_117
        label = _alpha_label(alpha, v_alpha)
        repeats = args.v_repeats if abs(alpha - v_alpha) < 1e-9 else args.intermediate_repeats
        verify_rows = _verify_theta(theta, alpha, scenario, args.seed, config, output_dir, repeats, label)
        verify_df = pd.DataFrame(verify_rows)
        verify_df.to_csv(reports_dir / f"redo_alpha_{label}_verify.csv", index=False)
        verify_summary = _summarize_verify(verify_df, scenario.properties)
        snap_dir, source = _snapshot_for_alpha(
            alpha,
            v_alpha,
            theta,
            scenario,
            args.seed,
            config,
            output_dir,
            groups,
            repeats,
            Path(args.center_snapshot),
            Path(args.v_snapshot),
            label,
        )
        metrics = _curve_metrics(snap_dir, PROPS_FOR_CURVE, groups, repeats)
        for prop in PROPS_FOR_CURVE:
            rho = _verify_row(verify_summary, prop)
            metric = metrics[prop]
            row = {
                "alpha": alpha,
                "label": label,
                "snapshot_source": source,
                "snapshot_dir": str(snap_dir),
                "property": prop,
                "rho_mean": rho["mean"],
                "rho_std": rho["std"],
                "abs_rho_mean": abs(float(rho["mean"])),
                **metric,
            }
            alpha_metric_rows.append(row)
        alpha_rows.append(
            {
                "alpha": alpha,
                "label": label,
                "theta_hash": theta_hash(theta),
                "repeats": repeats,
                "snapshot_source": source,
                "snapshot_dir": str(snap_dir),
                **{f"rho_mean_{row['property']}": row["mean"] for row in verify_summary},
                **{f"rho_std_{row['property']}": row["std"] for row in verify_summary},
            }
        )
    pd.DataFrame(alpha_rows).to_csv(reports_dir / "redo_alpha_verify_summary.csv", index=False)
    pd.DataFrame(alpha_metric_rows).to_csv(reports_dir / "redo_alpha_fd_curve_metrics.csv", index=False)

    summary = {
        "status": "complete",
        "scenario_id": scenario.id,
        "seed": args.seed,
        "candidate_index": args.candidate_index,
        "theta_117_hash": theta_hash(theta_117),
        "alpha_V": v_alpha,
        "theta_V_hash": theta_hash(theta_v),
        "delta_probe": args.delta_probe,
        "probe_repeats": args.probe_repeats,
        "intermediate_repeats": args.intermediate_repeats,
        "v_repeats": args.v_repeats,
        "pointV_verify_summary": v_verify,
        "pointV_fd_interior_delta008": v_interior,
        "pointV_probe_margin_summary": _probe_margin_summary(probe_margin_rows),
        "directional_probe_summary": probe_summary,
        "directional_probe_channel_marginal": probe_channel,
        "directional_probe_window_marginal": probe_window,
        "alpha_curve_metrics": alpha_metric_rows,
        "artifacts": {
            "theta_V": str(output_dir / "theta_V.npy"),
            "directional_probe": str(reports_dir / "redo_pointV_delta020_directional_probe.csv"),
            "channel_marginal": str(reports_dir / "redo_pointV_delta020_channel_marginal.csv"),
            "window_marginal": str(reports_dir / "redo_pointV_delta020_window_marginal.csv"),
            "alpha_curve_metrics": str(reports_dir / "redo_alpha_fd_curve_metrics.csv"),
        },
        "elapsed_wall_time_s": time.monotonic() - t0,
    }
    _write_json(reports_dir / "stage1_redo_summary.json", summary)
    _write_report(reports_dir / "stage1_redo_report.md", summary)
    print(
        f"stage1_redo_complete report={reports_dir / 'stage1_redo_report.md'} "
        f"elapsed={summary['elapsed_wall_time_s']:.1f}s",
        flush=True,
    )


def _config_for_run_dir(config: ExperimentConfig, output_dir: Path) -> ExperimentConfig:
    logging = dict(config.logging)
    logging["jsonl"] = str(output_dir / "logs" / "queries.jsonl")
    return replace(config, experiment_id=output_dir.name, logging=logging)


def _load_stage1_summary(stage1_run_dir: Path) -> dict:
    path = stage1_run_dir / "reports" / "stage1_summary.json"
    if not path.exists():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def _probe_margin_rows(theta: np.ndarray, delta_probe: float, config: ExperimentConfig, groups: list[Group]) -> list[dict]:
    rows = []
    min_value = float(config.input["min_value"])
    max_value = float(config.input["max_value"])
    for group in groups:
        raw_plus = theta.copy()
        raw_minus = theta.copy()
        raw_plus[group.group_id] += delta_probe
        raw_minus[group.group_id] -= delta_probe
        projected_plus = project_theta(raw_plus, config)
        projected_minus = project_theta(raw_minus, config)
        value_margin = min(float(theta[group.group_id] - min_value), float(max_value - theta[group.group_id]))
        rows.append(
            {
                **group.__dict__,
                "theta": float(theta[group.group_id]),
                "value_margin": value_margin,
                "value_margin_gt_delta_probe": bool(value_margin > delta_probe),
                "plus_projection_linf": float(np.max(np.abs(projected_plus - raw_plus))),
                "minus_projection_linf": float(np.max(np.abs(projected_minus - raw_minus))),
                "plus_rate_or_bound_projection_hit": bool(not np.allclose(projected_plus, raw_plus, atol=1e-9, rtol=0.0)),
                "minus_rate_or_bound_projection_hit": bool(not np.allclose(projected_minus, raw_minus, atol=1e-9, rtol=0.0)),
            }
        )
    return rows


def _verify_theta(
    theta: np.ndarray,
    alpha: float,
    scenario,
    seed: int,
    config: ExperimentConfig,
    output_dir: Path,
    repeats: int,
    label: str,
) -> list[dict]:
    rows = []
    for repeat_idx in range(repeats):
        cache_tag = f"stage1_redo_verify_{label}_alpha_{alpha:.8f}_repeat{repeat_idx}"
        result = _run_query_with_retry(
            theta,
            scenario,
            seed,
            "margin_stage1_redo_verify",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        row = {
            "label": label,
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
            f"redo_verify label={label} repeat={repeat_idx}/{repeats - 1} alpha={alpha:.8f} "
            + " ".join(f"{prop}={result.robustness[prop]:.6f}" for prop in scenario.properties),
            flush=True,
        )
    return rows


def _run_directional_probe(
    theta_v: np.ndarray,
    alpha_v: float,
    scenario,
    seed: int,
    config: ExperimentConfig,
    output_dir: Path,
    groups: list[Group],
    delta_probe: float,
    repeats: int,
    base_rho_velocity: float,
) -> tuple[list[dict], dict, list[dict], list[dict]]:
    raw_rows = []
    group_rows = []
    for group in groups:
        plus_values = []
        minus_values = []
        plus_projection = []
        minus_projection = []
        for repeat_idx in range(repeats):
            for sign, values, projections in [("+", plus_values, plus_projection), ("-", minus_values, minus_projection)]:
                raw = theta_v.copy()
                raw[group.group_id] += delta_probe if sign == "+" else -delta_probe
                projected = project_theta(raw, config)
                projection_linf = float(np.max(np.abs(projected - raw))) if raw.size else 0.0
                cache_tag = (
                    f"stage1_redo_delta020_pointV_g{group.group_id:02d}_{'plus' if sign == '+' else 'minus'}"
                    f"_repeat{repeat_idx}"
                )
                result = _run_query_with_retry(
                    raw,
                    scenario,
                    seed,
                    "margin_stage1_redo_delta_probe",
                    output_dir,
                    config,
                    cache_tag=cache_tag,
                    use_cache=True,
                )
                rho = float(result.robustness["post_neutral_xy_velocity"])
                values.append(rho)
                projections.append(projection_linf)
                raw_rows.append(
                    {
                        **group.__dict__,
                        "alpha": alpha_v,
                        "sign": sign,
                        "repeat_idx": repeat_idx,
                        "delta_probe": delta_probe,
                        "projection_linf": projection_linf,
                        "theta_hash": result.theta_hash,
                        "query_id": result.query_id,
                        "rho_xy_velocity": rho,
                        "rho_xy_drift": float(result.robustness["post_neutral_xy_drift"]),
                        "rho_alt_drift": float(result.robustness["post_neutral_alt_drift"]),
                    }
                )
        plus_arr = np.asarray(plus_values, dtype=float)
        minus_arr = np.asarray(minus_values, dtype=float)
        plus_mean = float(np.mean(plus_arr))
        minus_mean = float(np.mean(minus_arr))
        plus_std = float(np.std(plus_arr, ddof=1)) if plus_arr.size > 1 else 0.0
        minus_std = float(np.std(minus_arr, ddof=1)) if minus_arr.size > 1 else 0.0
        span = plus_mean - minus_mean
        abs_span = abs(span)
        sensitivity = abs_span / (2.0 * delta_probe)
        se_span = math.sqrt((plus_std**2 + minus_std**2) / repeats) if repeats > 0 else float("nan")
        sensitivity_se = se_span / (2.0 * delta_probe) if repeats > 0 else float("nan")
        max_abs_delta_from_base = max(abs(plus_mean - base_rho_velocity), abs(minus_mean - base_rho_velocity))
        group_rows.append(
            {
                **group.__dict__,
                "alpha": alpha_v,
                "delta_probe": delta_probe,
                "repeat_count": repeats,
                "base_rho_xy_velocity": base_rho_velocity,
                "rho_plus_mean": plus_mean,
                "rho_minus_mean": minus_mean,
                "rho_plus_std": plus_std,
                "rho_minus_std": minus_std,
                "rho_span_plus_minus": span,
                "abs_delta_rho_span": abs_span,
                "directional_sensitivity": sensitivity,
                "directional_sensitivity_se": sensitivity_se,
                "directional_snr": sensitivity / sensitivity_se if sensitivity_se and sensitivity_se > 0 else float("nan"),
                "max_abs_delta_rho_from_base": max_abs_delta_from_base,
                "plus_projection_linf_max": float(np.max(plus_projection)) if plus_projection else 0.0,
                "minus_projection_linf_max": float(np.max(minus_projection)) if minus_projection else 0.0,
            }
        )
        print(
            f"redo_probe_group g{group.group_id} {group.channel}@w{group.window_id} "
            f"sens={sensitivity:.6f} abs_span={abs_span:.6f}",
            flush=True,
        )

    rows = group_rows
    scores = np.asarray([row["directional_sensitivity"] for row in rows], dtype=float)
    channel_rows = _marginal_rows(rows, "channel", scores)
    window_rows = _marginal_rows(rows, "window_id", scores)
    summary = {
        "score_definition": "abs(mean rho(+delta_probe) - mean rho(-delta_probe)) / (2*delta_probe)",
        "base_rho_xy_velocity": base_rho_velocity,
        "distribution_abs_delta_rho_span": _distribution([row["abs_delta_rho_span"] for row in rows]),
        "distribution_directional_sensitivity": _distribution([row["directional_sensitivity"] for row in rows]),
        "distribution_max_abs_delta_rho_from_base": _distribution([row["max_abs_delta_rho_from_base"] for row in rows]),
        "channel_participation_ratio": _participation_ratio([row["weight"] for row in channel_rows]),
        "window_participation_ratio": _participation_ratio([row["weight"] for row in window_rows]),
        "active_channels_top80": _top_cumulative(channel_rows, 0.80),
        "active_windows_top80": _top_cumulative(window_rows, 0.80),
        "top8_groups_by_sensitivity": ",".join(
            str(int(row["group_id"])) for row in sorted(rows, key=lambda r: r["directional_sensitivity"], reverse=True)[:8]
        ),
        "projection_hit_groups": int(
            sum((row["plus_projection_linf_max"] > 1e-9) or (row["minus_projection_linf_max"] > 1e-9) for row in rows)
        ),
    }
    raw_path = output_dir / "reports" / "redo_pointV_delta020_directional_probe_raw_repeats.csv"
    pd.DataFrame(raw_rows).to_csv(raw_path, index=False)
    summary["raw_repeat_rows"] = str(raw_path)
    return rows, summary, channel_rows, window_rows


def _snapshot_for_alpha(
    alpha: float,
    v_alpha: float,
    theta: np.ndarray,
    scenario,
    seed: int,
    config: ExperimentConfig,
    output_dir: Path,
    groups: list[Group],
    repeats: int,
    center_snapshot: Path,
    v_snapshot: Path,
    label: str,
) -> tuple[Path, str]:
    if abs(alpha) < 1e-12 and center_snapshot.exists():
        return center_snapshot, "reused_center_snapshot"
    if abs(alpha - v_alpha) < 1e-9 and v_snapshot.exists():
        return v_snapshot, "reused_pointV_snapshot"
    phase_tag = f"stage1_redo_alpha_{label}_fd"
    snap_dir = run_repeated_snapshot(
        theta,
        scenario,
        seed,
        repeats,
        config,
        output_dir,
        groups,
        phase_tag,
        cache_namespace=phase_tag,
    )
    return snap_dir, "new_snapshot"


def _curve_metrics(snap_dir: Path, properties: list[str], groups: list[Group], requested_repeats: int) -> dict[str, dict]:
    metrics = {}
    for prop in properties:
        matrix = []
        repeat_dirs = sorted(p for p in snap_dir.glob("repeat_*") if p.is_dir())
        if not repeat_dirs:
            raise FileNotFoundError(f"No repeat_* dirs in {snap_dir}")
        for repeat_dir in repeat_dirs[:requested_repeats]:
            path = repeat_dir / f"gradient_{prop}.csv"
            if not path.exists():
                raise FileNotFoundError(path)
            df = pd.read_csv(path).sort_values("group_id")
            value_col = "g" if "g" in df.columns else "mean_g"
            matrix.append(df[value_col].to_numpy(dtype=float))
        values = np.vstack(matrix)
        mean_g = np.mean(values, axis=0)
        std_g = np.std(values, axis=0, ddof=1) if values.shape[0] > 1 else np.zeros(values.shape[1])
        se_g = std_g / math.sqrt(values.shape[0]) if values.shape[0] > 0 else std_g
        abs_mean = np.abs(mean_g)
        sum_abs = float(np.sum(abs_mean))
        channel_rows = _marginal_from_group_values(abs_mean, groups, "channel")
        window_rows = _marginal_from_group_values(abs_mean, groups, "window_id")
        metrics[prop] = {
            "fd_repeat_count": int(values.shape[0]),
            "effective_sparsity": effective_sparsity(mean_g),
            "top4_coverage": topk_coverage(abs_mean, 4),
            "top8_coverage": topk_coverage(abs_mean, 8),
            "top16_coverage": topk_coverage(abs_mean, 16),
            "sum_abs_mean_g": sum_abs,
            "noise_after_l1_local": float(np.sum(se_g)),
            "noise_after_over_sum": float(np.sum(se_g)) / sum_abs if sum_abs > 0 else float("nan"),
            "channel_participation_ratio": _participation_ratio([row["weight"] for row in channel_rows]),
            "window_participation_ratio": _participation_ratio([row["weight"] for row in window_rows]),
            "active_channels_top80": _top_cumulative(channel_rows, 0.80),
            "active_windows_top80": _top_cumulative(window_rows, 0.80),
            "channel_shares": _share_string(channel_rows),
            "window_shares": _share_string(window_rows),
            "top8_groups": ",".join(str(int(i)) for i in np.argsort(abs_mean)[::-1][:8]),
        }
    return metrics


def _marginal_rows(group_rows: list[dict], key: str, scores: np.ndarray) -> list[dict]:
    accum: dict[str, float] = {}
    for row, score in zip(group_rows, scores):
        label = str(row[key])
        accum[label] = accum.get(label, 0.0) + float(score)
    total = sum(accum.values())
    rows = [
        {
            key: _maybe_int(label),
            "weight": weight,
            "share": weight / total if total > 0 else 0.0,
        }
        for label, weight in accum.items()
    ]
    return sorted(rows, key=lambda r: r["weight"], reverse=True)


def _marginal_from_group_values(values: np.ndarray, groups: list[Group], key: str) -> list[dict]:
    rows = []
    for group in groups:
        rows.append({**group.__dict__, "score": float(values[group.group_id])})
    return _marginal_rows(rows, key, np.asarray([row["score"] for row in rows], dtype=float))


def _participation_ratio(weights) -> float:
    values = np.asarray(list(weights), dtype=float)
    denom = float(np.sum(values * values))
    if denom <= 0:
        return 0.0
    total = float(np.sum(values))
    return (total * total) / denom


def _top_cumulative(rows: list[dict], threshold: float) -> str:
    total = sum(float(row["weight"]) for row in rows)
    if total <= 0:
        return ""
    running = 0.0
    labels = []
    for row in rows:
        running += float(row["weight"])
        label = row.get("channel", row.get("window_id"))
        labels.append(str(label))
        if running / total >= threshold:
            break
    return ",".join(labels)


def _share_string(rows: list[dict]) -> str:
    return ",".join(f"{row.get('channel', row.get('window_id'))}:{row['share']:.3f}" for row in rows)


def _distribution(values) -> dict:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return {}
    return {
        "min": float(np.min(arr)),
        "p25": float(np.quantile(arr, 0.25)),
        "median": float(np.median(arr)),
        "p75": float(np.quantile(arr, 0.75)),
        "p90": float(np.quantile(arr, 0.90)),
        "p95": float(np.quantile(arr, 0.95)),
        "max": float(np.max(arr)),
    }


def _probe_margin_summary(rows: list[dict]) -> dict:
    return {
        "groups_with_value_margin_gt_delta_probe": int(sum(row["value_margin_gt_delta_probe"] for row in rows)),
        "group_count": len(rows),
        "groups_with_projection_hit": int(
            sum(row["plus_rate_or_bound_projection_hit"] or row["minus_rate_or_bound_projection_hit"] for row in rows)
        ),
        "min_value_margin": float(min(row["value_margin"] for row in rows)) if rows else 0.0,
        "max_plus_projection_linf": float(max(row["plus_projection_linf"] for row in rows)) if rows else 0.0,
        "max_minus_projection_linf": float(max(row["minus_projection_linf"] for row in rows)) if rows else 0.0,
    }


def _verify_row(verify_summary: list[dict], prop: str) -> dict:
    return next(row for row in verify_summary if row["property"] == prop)


def _verify_mean(verify_summary: list[dict], prop: str) -> float:
    return float(_verify_row(verify_summary, prop)["mean"])


def _maybe_int(value: str):
    try:
        return int(value)
    except ValueError:
        return value


def _alpha_label(alpha: float, v_alpha: float) -> str:
    if abs(alpha - v_alpha) < 1e-9:
        return "V"
    return f"a{int(round(alpha * 1000)):03d}"


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _write_report(path: Path, summary: dict) -> None:
    probe = summary["directional_probe_summary"]
    lines = [
        "# MarginSearch Stage 1-redo Report",
        "",
        f"Scope: `{summary['scenario_id']}`, seed {summary['seed']}, Point V alpha={summary['alpha_V']:.8f}.",
        "",
        "## Point V",
        "",
        "| item | value |",
        "| --- | ---: |",
        f"| xy_velocity J mean rho | {_verify_mean(summary['pointV_verify_summary'], 'post_neutral_xy_velocity'):.6f} |",
        f"| interior FD clean groups at delta=0.08 | {summary['pointV_fd_interior_delta008']['fd_clean_two_sided_groups']}/{summary['pointV_fd_interior_delta008']['group_count']} |",
        f"| probe value-margin > 0.2 groups | {summary['pointV_probe_margin_summary']['groups_with_value_margin_gt_delta_probe']}/{summary['pointV_probe_margin_summary']['group_count']} |",
        f"| probe projection-hit groups | {summary['pointV_probe_margin_summary']['groups_with_projection_hit']} |",
        "",
        "## Delta 0.2 Directional Sensitivity",
        "",
        "| metric | value |",
        "| --- | ---: |",
        f"| channel participation ratio / 4 | {probe['channel_participation_ratio']:.3f} |",
        f"| window participation ratio / 10 | {probe['window_participation_ratio']:.3f} |",
        f"| active channels top80 | {probe['active_channels_top80']} |",
        f"| active windows top80 | {probe['active_windows_top80']} |",
        f"| top8 groups | {probe['top8_groups_by_sensitivity']} |",
        "",
        "Sensitivity distribution uses `abs(mean rho(+0.2) - mean rho(-0.2)) / 0.4`.",
        "",
        "| distribution | min | p25 | median | p75 | p90 | p95 | max |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for name in ["distribution_abs_delta_rho_span", "distribution_directional_sensitivity", "distribution_max_abs_delta_rho_from_base"]:
        d = probe[name]
        lines.append(
            f"| {name} | {d['min']:.6f} | {d['p25']:.6f} | {d['median']:.6f} | {d['p75']:.6f} | "
            f"{d['p90']:.6f} | {d['p95']:.6f} | {d['max']:.6f} |"
        )
    lines.extend(
        [
            "",
            "## Eff Sparsity Vs Rho",
            "",
            "| alpha | property | rho mean | eff sparsity | top8 | noise/sum | channel PR | window PR | active channels | active windows | top8 groups |",
            "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |",
        ]
    )
    for row in summary["alpha_curve_metrics"]:
        lines.append(
            f"| {row['alpha']:.6f} | {row['property']} | {row['rho_mean']:.6f} | "
            f"{row['effective_sparsity']:.3f} | {row['top8_coverage']:.3f} | "
            f"{row['noise_after_over_sum']:.3f} | {row['channel_participation_ratio']:.3f} | "
            f"{row['window_participation_ratio']:.3f} | {row['active_channels_top80']} | "
            f"{row['active_windows_top80']} | {row['top8_groups']} |"
        )
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            f"- summary JSON: `{path.with_name('stage1_redo_summary.json')}`",
        ]
    )
    for key, artifact in summary["artifacts"].items():
        lines.append(f"- {key}: `{artifact}`")
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


if __name__ == "__main__":
    main()
