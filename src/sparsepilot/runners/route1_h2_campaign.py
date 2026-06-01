from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import pandas as pd

from sparsepilot.config import ExperimentConfig, load_config
from sparsepilot.groups import Group, build_groups
from sparsepilot.input_model import project_theta, zero_theta
from sparsepilot.query import theta_hash
from sparsepilot.runners.fd_snapshot import _run_query_with_retry
from sparsepilot.violation_search import generate_initial_candidates


TARGET_PROPERTY = "post_neutral_xy_velocity"
ACTIVE_CHANNELS = ["roll", "pitch"]


@dataclass(frozen=True)
class Condition:
    index: int
    v_max: float
    label: str
    config: ExperimentConfig


@dataclass(frozen=True)
class EvalMean:
    theta: np.ndarray
    theta_hash: str
    rho_mean: float
    rho_std: float
    rho_min: float
    rho_max: float
    nonpositive: int
    repeats: int
    query_count: int


@dataclass(frozen=True)
class BoundaryResult:
    method: str
    condition_index: int
    v_max: float
    status: str
    theta: np.ndarray | None
    theta_hash: str
    rho_mean: float
    rho_std: float
    query_count: int
    detail: dict


class CampaignEvaluator:
    def __init__(self, scenario_id: str, seed: int, output_dir: Path, groups: list[Group]):
        self.scenario_id = scenario_id
        self.seed = seed
        self.output_dir = Path(output_dir)
        self.groups = groups
        self.reports_dir = self.output_dir / "reports"
        self.eval_rows: list[dict] = []
        self.eval_counter = 0
        self.query_counter = 0
        self.eval_rows_path = self.reports_dir / "query_evaluations.csv"

    def query_count(self) -> int:
        return self.query_counter

    def eval_mean(
        self,
        theta: np.ndarray,
        condition: Condition,
        method: str,
        stage: str,
        repeats: int,
        label: str,
    ) -> EvalMean:
        scenario = condition.config.scenario_by_id(self.scenario_id)
        projected = project_theta(np.asarray(theta, dtype=float), condition.config)
        values: list[float] = []
        thash = theta_hash(projected)
        eval_id = self.eval_counter
        self.eval_counter += 1
        for repeat_idx in range(repeats):
            cache_tag = _safe_label(
                f"h2_{method}_{condition.label}_{stage}_{eval_id:05d}_{label}_repeat{repeat_idx}"
            )
            result = _run_query_with_retry(
                projected,
                scenario,
                self.seed,
                f"route1_h2_{method}",
                self.output_dir,
                condition.config,
                cache_tag=cache_tag,
                use_cache=True,
            )
            rho = float(result.robustness[TARGET_PROPERTY])
            values.append(rho)
            self.query_counter += 1
            row = {
                "eval_id": eval_id,
                "condition_index": condition.index,
                "v_max": condition.v_max,
                "condition_label": condition.label,
                "method": method,
                "stage": stage,
                "label": label,
                "repeat_idx": repeat_idx,
                "theta_hash": result.theta_hash,
                "query_id": result.query_id,
                "cache_tag": cache_tag,
                "query_total_wall_time_s": float(result.metadata.get("total_wall_time_s", math.nan)),
                "projection_linf": float(np.max(np.abs(projected - np.asarray(theta, dtype=float)))),
            }
            for prop, value in result.robustness.items():
                row[f"robustness_{prop}"] = float(value)
            self.eval_rows.append(row)
            if len(self.eval_rows) % 10 == 0:
                self.write_eval_rows()
            print(
                f"h2_eval method={method} condition={condition.label} stage={stage} repeat={repeat_idx}/{repeats - 1} "
                f"rho_xy_velocity={rho:.6f}",
                flush=True,
            )
        arr = np.asarray(values, dtype=float)
        return EvalMean(
            theta=projected,
            theta_hash=thash,
            rho_mean=float(np.mean(arr)),
            rho_std=float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
            rho_min=float(np.min(arr)),
            rho_max=float(np.max(arr)),
            nonpositive=int(np.sum(arr <= 0.0)),
            repeats=int(arr.size),
            query_count=int(arr.size),
        )

    def write_eval_rows(self) -> None:
        if self.eval_rows:
            pd.DataFrame(self.eval_rows).to_csv(self.eval_rows_path, index=False)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Route-1 H2 pilot: cross-condition warm-start on px4_position seed 0 "
            "for post_neutral_xy_velocity under disclosed v_max tightening."
        )
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/route1_h2_px4_position_seed0_vmax_pilot_v0")
    parser.add_argument("--theta-v", default="runs/margin_stage1_v1/point_V/theta.npy")
    parser.add_argument("--v-max", action="append", type=float, default=None)
    parser.add_argument("--rng-seed", type=int, default=20260530)
    parser.add_argument("--structured-random-count", type=int, default=52)
    parser.add_argument("--uniform-max-samples", type=int, default=120)
    parser.add_argument("--descent-max-steps", type=int, default=80)
    parser.add_argument("--bisection-iters", type=int, default=8)
    parser.add_argument("--localization-repeats", type=int, default=5)
    parser.add_argument("--channel-delta", type=float, default=0.2)
    parser.add_argument("--channel-repeats", type=int, default=5)
    parser.add_argument("--skip-channel-measurement", action="store_true")
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Route-1 H2 pilot is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Route-1 H2 pilot is frozen to seed 0")

    t0 = time.monotonic()
    output_dir = Path(args.run_dir)
    reports_dir = output_dir / "reports"
    boundaries_dir = output_dir / "boundaries"
    reports_dir.mkdir(parents=True, exist_ok=True)
    boundaries_dir.mkdir(parents=True, exist_ok=True)

    base_config = load_config(args.config)
    if float(base_config.input["perturb_delta"]) != 0.08:
        raise ValueError("Frozen measurement requires perturb_delta=0.08 in the config")
    v_values = args.v_max or [1.0, 0.9, 0.8]
    if v_values != sorted(v_values, reverse=True):
        raise ValueError("v_max conditions must be supplied from loose to tight")
    conditions = [
        Condition(
            index=i + 1,
            v_max=float(v),
            label=f"v{_v_label(v)}",
            config=_config_for_condition(base_config, output_dir, float(v)),
        )
        for i, v in enumerate(v_values)
    ]
    scenario = conditions[0].config.scenario_by_id(args.scenario)
    if TARGET_PROPERTY not in scenario.properties:
        raise ValueError(f"{TARGET_PROPERTY} must be enabled for {args.scenario}")

    groups = build_groups(base_config.input["horizon_s"], base_config.input["window_s"], base_config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"Frozen D=40 parameterization expected 40 groups, found {len(groups)}")
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(output_dir / "groups.csv", index=False)

    theta_v_path = Path(args.theta_v)
    if not theta_v_path.exists():
        fallback = Path("runs/margin_stage1_redo_v1/theta_V.npy")
        if fallback.exists():
            theta_v_path = fallback
        else:
            raise FileNotFoundError(args.theta_v)
    theta_v = project_theta(np.load(theta_v_path), conditions[0].config)
    np.save(output_dir / "theta_V_condition1_anchor.npy", theta_v)

    evaluator = CampaignEvaluator(args.scenario, args.seed, output_dir, groups)
    rng = np.random.default_rng(args.rng_seed)
    boundary_rows: list[dict] = []
    cold_results: dict[str, list[BoundaryResult]] = {name: [] for name in ["structured", "uniform", "descent"]}

    print(
        f"route1_h2_start scenario={args.scenario} seed={args.seed} "
        f"v_max={','.join(str(v) for v in v_values)} run_dir={output_dir}",
        flush=True,
    )

    anchor_eval = evaluator.eval_mean(
        theta_v,
        conditions[0],
        "warm",
        "condition1_pointV_anchor",
        args.localization_repeats,
        "pointV",
    )
    anchor = BoundaryResult(
        method="warm_anchor",
        condition_index=conditions[0].index,
        v_max=conditions[0].v_max,
        status="reused_pointV",
        theta=theta_v,
        theta_hash=theta_hash(theta_v),
        rho_mean=anchor_eval.rho_mean,
        rho_std=anchor_eval.rho_std,
        query_count=anchor_eval.query_count,
        detail={"theta_source": str(theta_v_path), "role": "v_max_1p0_boundary_anchor"},
    )
    _save_boundary(boundaries_dir, anchor)
    boundary_rows.append(_boundary_row(anchor, boundaries_dir))
    _write_boundary_rows(reports_dir, boundary_rows)

    for condition in conditions:
        print(f"h2_cold_condition_start condition={condition.label} v_max={condition.v_max}", flush=True)
        for method, runner in [
            ("structured", _run_structured_cold),
            ("uniform", _run_uniform_cold),
            ("descent", _run_descent_cold),
        ]:
            before = evaluator.query_count()
            result = runner(condition, evaluator, groups, rng, args)
            result = _with_query_count(result, evaluator.query_count() - before)
            cold_results[method].append(result)
            _save_boundary(boundaries_dir, result)
            boundary_rows.append(_boundary_row(result, boundaries_dir))
            _write_boundary_rows(reports_dir, boundary_rows)
            print(
                f"h2_cold_result method={method} condition={condition.label} status={result.status} "
                f"queries={result.query_count} rho_mean={result.rho_mean:.6f}",
                flush=True,
            )

    warm_results: list[BoundaryResult] = [anchor]
    previous_theta = theta_v
    for condition in conditions[1:]:
        before = evaluator.query_count()
        result = _run_warm_active_search(condition, evaluator, groups, previous_theta, args)
        result = _with_query_count(result, evaluator.query_count() - before)
        warm_results.append(result)
        if result.theta is not None:
            previous_theta = result.theta
        _save_boundary(boundaries_dir, result)
        boundary_rows.append(_boundary_row(result, boundaries_dir))
        _write_boundary_rows(reports_dir, boundary_rows)
        print(
            f"h2_warm_result condition={condition.label} status={result.status} "
            f"queries={result.query_count} rho_mean={result.rho_mean:.6f}",
            flush=True,
        )

    channel_measurements = []
    if not args.skip_channel_measurement:
        channel_targets = [(conditions[0], warm_results[0]), (conditions[-1], warm_results[-1])]
        for condition, boundary in channel_targets:
            if boundary.theta is None:
                continue
            before = evaluator.query_count()
            measurement = _run_channel_measurement(
                condition,
                boundary,
                evaluator,
                groups,
                delta=float(args.channel_delta),
                repeats=int(args.channel_repeats),
            )
            measurement["query_count"] = evaluator.query_count() - before
            channel_measurements.append(measurement)
            _write_json(reports_dir / "channel_measurements_summary.json", channel_measurements)
            print(
                f"h2_channel_measurement condition={condition.label} "
                f"active_top80={measurement['active_channels_top80']} queries={measurement['query_count']}",
                flush=True,
            )

    evaluator.write_eval_rows()
    summary = _build_summary(
        output_dir=output_dir,
        scenario_id=args.scenario,
        seed=args.seed,
        conditions=conditions,
        theta_v_path=theta_v_path,
        anchor=anchor,
        warm_results=warm_results,
        cold_results=cold_results,
        channel_measurements=channel_measurements,
        total_queries=evaluator.query_count(),
        elapsed_wall_time_s=time.monotonic() - t0,
    )
    _write_json(reports_dir / "route1_h2_summary.json", summary)
    _write_report(reports_dir / "route1_h2_report.md", summary)
    print(
        f"route1_h2_complete queries={summary['total_query_count']} "
        f"elapsed={summary['elapsed_wall_time_s']:.1f}s report={reports_dir / 'route1_h2_report.md'}",
        flush=True,
    )


def _config_for_condition(config: ExperimentConfig, output_dir: Path, v_max: float) -> ExperimentConfig:
    properties = {name: dict(values) for name, values in config.properties.items()}
    properties[TARGET_PROPERTY] = dict(properties[TARGET_PROPERTY])
    properties[TARGET_PROPERTY]["v_max_mps"] = float(v_max)
    logging = dict(config.logging)
    logging["jsonl"] = str(output_dir / "logs" / "queries.jsonl")
    return replace(config, experiment_id=output_dir.name, properties=properties, logging=logging)


def _run_structured_cold(
    condition: Condition,
    evaluator: CampaignEvaluator,
    groups: list[Group],
    rng: np.random.Generator,
    args: argparse.Namespace,
) -> BoundaryResult:
    candidates = generate_initial_candidates(condition.config, groups, int(args.structured_random_count), rng)
    candidate_rows = []
    for candidate in candidates:
        result = evaluator.eval_mean(
            candidate.theta,
            condition,
            "structured",
            "coarse_candidate",
            1,
            f"{candidate.index:03d}_{candidate.label}",
        )
        candidate_rows.append(
            {
                "condition_index": condition.index,
                "v_max": condition.v_max,
                "candidate_index": candidate.index,
                "source": candidate.source,
                "label": candidate.label,
                "theta_hash": result.theta_hash,
                "rho_single": result.rho_mean,
            }
        )
    path = evaluator.reports_dir / f"structured_candidates_{condition.label}.csv"
    pd.DataFrame(candidate_rows).to_csv(path, index=False)

    zero = zero_theta(groups)
    negative_candidates = [row for row in candidate_rows if float(row["rho_single"]) < 0.0]
    candidate_by_index = {candidate.index: candidate for candidate in candidates}
    for row in sorted(negative_candidates, key=lambda r: float(r["rho_single"])):
        candidate = candidate_by_index[int(row["candidate_index"])]
        result = _bisect_segment(
            condition,
            evaluator,
            zero,
            candidate.theta,
            "structured",
            f"candidate{candidate.index:03d}",
            int(args.localization_repeats),
            int(args.bisection_iters),
        )
        if result.status == "complete":
            result.detail.update(
                {
                    "coarse_candidates": len(candidate_rows),
                    "selected_candidate_index": candidate.index,
                    "selected_candidate_label": candidate.label,
                    "candidate_rows": str(path),
                }
            )
            return result
    return BoundaryResult(
        method="structured",
        condition_index=condition.index,
        v_max=condition.v_max,
        status="failed_no_j5_straddle",
        theta=None,
        theta_hash="",
        rho_mean=float("nan"),
        rho_std=float("nan"),
        query_count=0,
        detail={"coarse_candidates": len(candidate_rows), "candidate_rows": str(path)},
    )


def _run_uniform_cold(
    condition: Condition,
    evaluator: CampaignEvaluator,
    groups: list[Group],
    rng: np.random.Generator,
    args: argparse.Namespace,
) -> BoundaryResult:
    sampled_rows = []
    bracket_failures = []
    safe_theta = None
    unsafe_theta = None
    safe_rho = None
    unsafe_rho = None
    for sample_idx in range(int(args.uniform_max_samples)):
        theta = _sample_uniform_feasible(condition.config, groups, rng)
        result = evaluator.eval_mean(theta, condition, "uniform", "random_sample", 1, f"s{sample_idx:03d}")
        row = {
            "condition_index": condition.index,
            "v_max": condition.v_max,
            "sample_idx": sample_idx,
            "theta_hash": result.theta_hash,
            "rho_single": result.rho_mean,
        }
        sampled_rows.append(row)
        if result.rho_mean >= 0.0:
            safe_theta = result.theta
            safe_rho = result.rho_mean
        else:
            unsafe_theta = result.theta
            unsafe_rho = result.rho_mean
        if safe_theta is not None and unsafe_theta is not None:
            partial_path = evaluator.reports_dir / f"uniform_samples_{condition.label}.csv"
            pd.DataFrame(sampled_rows).to_csv(partial_path, index=False)
            boundary = _bisect_segment(
                condition,
                evaluator,
                safe_theta,
                unsafe_theta,
                "uniform",
                "first_straddle" if not bracket_failures else f"straddle_after_sample{sample_idx:03d}",
                int(args.localization_repeats),
                int(args.bisection_iters),
            )
            if boundary.status == "complete":
                boundary.detail.update(
                    {
                        "samples_to_j5_straddle": len(sampled_rows),
                        "single_safe_rho": safe_rho,
                        "single_unsafe_rho": unsafe_rho,
                        "sample_rows": str(partial_path),
                        "j5_bracket_failures": bracket_failures,
                    }
                )
                return boundary
            bracket_failures.append(
                {
                    "after_sample_idx": sample_idx,
                    "single_safe_rho": safe_rho,
                    "single_unsafe_rho": unsafe_rho,
                    **boundary.detail,
                }
            )
            if float(boundary.detail.get("safe_rho_mean", 0.0)) < 0.0:
                safe_theta = None
                safe_rho = None
            if float(boundary.detail.get("unsafe_rho_mean", -1.0)) >= 0.0:
                unsafe_theta = None
                unsafe_rho = None
    path = evaluator.reports_dir / f"uniform_samples_{condition.label}.csv"
    pd.DataFrame(sampled_rows).to_csv(path, index=False)
    if safe_theta is None or unsafe_theta is None:
        return BoundaryResult(
            method="uniform",
            condition_index=condition.index,
            v_max=condition.v_max,
            status="failed_no_j5_straddle",
            theta=None,
            theta_hash="",
            rho_mean=float("nan"),
            rho_std=float("nan"),
            query_count=0,
            detail={"samples": len(sampled_rows), "sample_rows": str(path), "j5_bracket_failures": bracket_failures},
        )
    return BoundaryResult(
        method="uniform",
        condition_index=condition.index,
        v_max=condition.v_max,
        status="failed_no_j5_straddle",
        theta=None,
        theta_hash="",
        rho_mean=float("nan"),
        rho_std=float("nan"),
        query_count=0,
        detail={"samples": len(sampled_rows), "sample_rows": str(path), "j5_bracket_failures": bracket_failures},
    )


def _run_descent_cold(
    condition: Condition,
    evaluator: CampaignEvaluator,
    groups: list[Group],
    rng: np.random.Generator,
    args: argparse.Namespace,
) -> BoundaryResult:
    current = _sample_uniform_feasible(condition.config, groups, rng)
    current_eval = evaluator.eval_mean(current, condition, "descent", "initial", 1, "initial")
    best_theta = current_eval.theta
    best_rho = current_eval.rho_mean
    current_theta = current_eval.theta
    current_rho = current_eval.rho_mean
    rows = [
        {
            "condition_index": condition.index,
            "v_max": condition.v_max,
            "step": 0,
            "theta_hash": current_eval.theta_hash,
            "rho_single": current_rho,
            "accepted": True,
            "best_rho": best_rho,
        }
    ]
    step_scale = 0.30
    no_improve = 0
    for step in range(1, int(args.descent_max_steps) + 1):
        if best_rho < 0.0:
            break
        if no_improve >= 12:
            proposal = _sample_uniform_feasible(condition.config, groups, rng)
            no_improve = 0
        else:
            proposal = project_theta(current_theta + rng.normal(0.0, step_scale, size=current_theta.shape), condition.config)
        proposal_eval = evaluator.eval_mean(proposal, condition, "descent", "local_step", 1, f"step{step:03d}")
        accepted = bool(proposal_eval.rho_mean <= current_rho)
        if accepted:
            current_theta = proposal_eval.theta
            current_rho = proposal_eval.rho_mean
        if proposal_eval.rho_mean < best_rho:
            best_theta = proposal_eval.theta
            best_rho = proposal_eval.rho_mean
            no_improve = 0
        else:
            no_improve += 1
        step_scale = max(0.06, step_scale * 0.985)
        rows.append(
            {
                "condition_index": condition.index,
                "v_max": condition.v_max,
                "step": step,
                "theta_hash": proposal_eval.theta_hash,
                "rho_single": proposal_eval.rho_mean,
                "accepted": accepted,
                "best_rho": best_rho,
            }
        )
    path = evaluator.reports_dir / f"descent_trace_{condition.label}.csv"
    pd.DataFrame(rows).to_csv(path, index=False)
    if best_rho >= 0.0:
        return BoundaryResult(
            method="descent",
            condition_index=condition.index,
            v_max=condition.v_max,
            status="failed_no_single_query_violation",
            theta=None,
            theta_hash="",
            rho_mean=float("nan"),
            rho_std=float("nan"),
            query_count=0,
            detail={"steps": len(rows), "best_single_rho": best_rho, "trace_rows": str(path)},
        )
    result = _bisect_segment(
        condition,
        evaluator,
        zero_theta(groups),
        best_theta,
        "descent",
        "best_descent",
        int(args.localization_repeats),
        int(args.bisection_iters),
    )
    result.detail.update({"descent_steps": len(rows), "best_single_rho": best_rho, "trace_rows": str(path)})
    return result


def _run_warm_active_search(
    condition: Condition,
    evaluator: CampaignEvaluator,
    groups: list[Group],
    previous_theta: np.ndarray,
    args: argparse.Namespace,
) -> BoundaryResult:
    active_ids = [group.group_id for group in groups if group.channel in ACTIVE_CHANNELS]
    scale_evals = []

    def theta_at(scale: float) -> np.ndarray:
        theta = np.asarray(previous_theta, dtype=float).copy()
        theta[active_ids] = theta[active_ids] * float(scale)
        return project_theta(theta, condition.config)

    high_scale = 1.0
    high_eval = evaluator.eval_mean(
        theta_at(high_scale),
        condition,
        "warm",
        "active_scale_probe",
        int(args.localization_repeats),
        "scale1p000",
    )
    scale_evals.append({"scale": high_scale, "rho_mean": high_eval.rho_mean, "theta_hash": high_eval.theta_hash})
    if high_eval.rho_mean >= 0.0:
        return BoundaryResult(
            method="warm",
            condition_index=condition.index,
            v_max=condition.v_max,
            status="failed_previous_boundary_not_violating_under_tightened_contract",
            theta=None,
            theta_hash="",
            rho_mean=float("nan"),
            rho_std=float("nan"),
            query_count=0,
            detail={"active_channels": ",".join(ACTIVE_CHANNELS), "scale_evals": scale_evals},
        )

    safe_scale = None
    safe_eval = None
    for scale in [0.8, 0.6, 0.4, 0.2, 0.0]:
        result = evaluator.eval_mean(
            theta_at(scale),
            condition,
            "warm",
            "active_scale_probe",
            int(args.localization_repeats),
            f"scale{_scale_label(scale)}",
        )
        scale_evals.append({"scale": scale, "rho_mean": result.rho_mean, "theta_hash": result.theta_hash})
        if result.rho_mean >= 0.0:
            safe_scale = scale
            safe_eval = result
            break
        high_scale = scale
        high_eval = result
    if safe_scale is None or safe_eval is None:
        return BoundaryResult(
            method="warm",
            condition_index=condition.index,
            v_max=condition.v_max,
            status="failed_no_safe_active_scale",
            theta=None,
            theta_hash="",
            rho_mean=float("nan"),
            rho_std=float("nan"),
            query_count=0,
            detail={"active_channels": ",".join(ACTIVE_CHANNELS), "scale_evals": scale_evals},
        )

    points = [(safe_scale, safe_eval), (high_scale, high_eval)]
    low_scale = safe_scale
    for iteration in range(int(args.bisection_iters)):
        mid_scale = 0.5 * (low_scale + high_scale)
        result = evaluator.eval_mean(
            theta_at(mid_scale),
            condition,
            "warm",
            "active_scale_bisection",
            int(args.localization_repeats),
            f"iter{iteration:02d}_scale{_scale_label(mid_scale)}",
        )
        points.append((mid_scale, result))
        if result.rho_mean >= 0.0:
            low_scale = mid_scale
            safe_eval = result
        else:
            high_scale = mid_scale
            high_eval = result

    best_scale, best_eval = min(points, key=lambda item: abs(item[1].rho_mean))
    detail = {
        "active_channels": ",".join(ACTIVE_CHANNELS),
        "best_scale": best_scale,
        "safe_scale_final": low_scale,
        "unsafe_scale_final": high_scale,
        "scale_evals": scale_evals,
        "bisection_iters": int(args.bisection_iters),
        "localization_repeats": int(args.localization_repeats),
    }
    return BoundaryResult(
        method="warm",
        condition_index=condition.index,
        v_max=condition.v_max,
        status="complete",
        theta=best_eval.theta,
        theta_hash=best_eval.theta_hash,
        rho_mean=best_eval.rho_mean,
        rho_std=best_eval.rho_std,
        query_count=0,
        detail=detail,
    )


def _bisect_segment(
    condition: Condition,
    evaluator: CampaignEvaluator,
    safe_theta: np.ndarray,
    unsafe_theta: np.ndarray,
    method: str,
    label: str,
    repeats: int,
    iterations: int,
) -> BoundaryResult:
    safe_eval = evaluator.eval_mean(safe_theta, condition, method, "j5_bracket_safe", repeats, label)
    unsafe_eval = evaluator.eval_mean(unsafe_theta, condition, method, "j5_bracket_unsafe", repeats, label)
    if safe_eval.rho_mean < 0.0 or unsafe_eval.rho_mean >= 0.0:
        return BoundaryResult(
            method=method,
            condition_index=condition.index,
            v_max=condition.v_max,
            status="failed_j5_bracket",
            theta=None,
            theta_hash="",
            rho_mean=float("nan"),
            rho_std=float("nan"),
            query_count=0,
            detail={
                "safe_rho_mean": safe_eval.rho_mean,
                "unsafe_rho_mean": unsafe_eval.rho_mean,
                "label": label,
            },
        )

    safe = safe_eval.theta
    unsafe = unsafe_eval.theta
    points = [safe_eval, unsafe_eval]
    for iteration in range(iterations):
        mid = project_theta(0.5 * (safe + unsafe), condition.config)
        mid_eval = evaluator.eval_mean(
            mid,
            condition,
            method,
            "j5_bisection",
            repeats,
            f"{label}_iter{iteration:02d}",
        )
        points.append(mid_eval)
        if mid_eval.rho_mean >= 0.0:
            safe = mid_eval.theta
            safe_eval = mid_eval
        else:
            unsafe = mid_eval.theta
            unsafe_eval = mid_eval

    best = min(points, key=lambda value: abs(value.rho_mean))
    detail = {
        "label": label,
        "safe_theta_hash_final": safe_eval.theta_hash,
        "safe_rho_mean_final": safe_eval.rho_mean,
        "unsafe_theta_hash_final": unsafe_eval.theta_hash,
        "unsafe_rho_mean_final": unsafe_eval.rho_mean,
        "bisection_iters": iterations,
        "localization_repeats": repeats,
    }
    return BoundaryResult(
        method=method,
        condition_index=condition.index,
        v_max=condition.v_max,
        status="complete",
        theta=best.theta,
        theta_hash=best.theta_hash,
        rho_mean=best.rho_mean,
        rho_std=best.rho_std,
        query_count=0,
        detail=detail,
    )


def _run_channel_measurement(
    condition: Condition,
    boundary: BoundaryResult,
    evaluator: CampaignEvaluator,
    groups: list[Group],
    *,
    delta: float,
    repeats: int,
) -> dict:
    if boundary.theta is None:
        raise ValueError("channel measurement requires a boundary theta")
    raw_rows = []
    group_rows = []
    for group in groups:
        plus_values: list[float] = []
        minus_values: list[float] = []
        projection_plus = []
        projection_minus = []
        for sign, values, projections in [("+", plus_values, projection_plus), ("-", minus_values, projection_minus)]:
            for repeat_idx in range(repeats):
                raw = boundary.theta.copy()
                raw[group.group_id] += delta if sign == "+" else -delta
                projected = project_theta(raw, condition.config)
                projections.append(float(np.max(np.abs(projected - raw))))
                result = evaluator.eval_mean(
                    raw,
                    condition,
                    "channel",
                    f"delta{_scale_label(delta)}_{'plus' if sign == '+' else 'minus'}",
                    1,
                    f"g{group.group_id:02d}_repeat{repeat_idx}",
                )
                values.append(result.rho_mean)
                raw_rows.append(
                    {
                        **group.__dict__,
                        "condition_index": condition.index,
                        "v_max": condition.v_max,
                        "boundary_method": boundary.method,
                        "boundary_theta_hash": boundary.theta_hash,
                        "sign": sign,
                        "repeat_idx": repeat_idx,
                        "delta": delta,
                        "projection_linf": projections[-1],
                        "theta_hash": result.theta_hash,
                        "rho_xy_velocity": result.rho_mean,
                    }
                )
        plus = np.asarray(plus_values, dtype=float)
        minus = np.asarray(minus_values, dtype=float)
        plus_mean = float(np.mean(plus))
        minus_mean = float(np.mean(minus))
        plus_std = float(np.std(plus, ddof=1)) if plus.size > 1 else 0.0
        minus_std = float(np.std(minus, ddof=1)) if minus.size > 1 else 0.0
        span = plus_mean - minus_mean
        sensitivity = abs(span) / (2.0 * delta)
        se_span = math.sqrt((plus_std**2 + minus_std**2) / repeats) if repeats > 0 else float("nan")
        group_rows.append(
            {
                **group.__dict__,
                "condition_index": condition.index,
                "v_max": condition.v_max,
                "boundary_theta_hash": boundary.theta_hash,
                "delta": delta,
                "repeat_count": repeats,
                "base_rho_xy_velocity": boundary.rho_mean,
                "rho_plus_mean": plus_mean,
                "rho_minus_mean": minus_mean,
                "rho_plus_std": plus_std,
                "rho_minus_std": minus_std,
                "rho_span_plus_minus": span,
                "abs_delta_rho_span": abs(span),
                "directional_sensitivity": sensitivity,
                "directional_sensitivity_se": se_span / (2.0 * delta) if repeats > 0 else float("nan"),
                "plus_projection_linf_max": float(np.max(projection_plus)) if projection_plus else 0.0,
                "minus_projection_linf_max": float(np.max(projection_minus)) if projection_minus else 0.0,
            }
        )
        print(
            f"h2_channel_group condition={condition.label} g{group.group_id} {group.channel}@w{group.window_id} "
            f"sens={sensitivity:.6f}",
            flush=True,
        )

    scores = np.asarray([row["directional_sensitivity"] for row in group_rows], dtype=float)
    channel_rows = _marginal_rows(group_rows, "channel", scores)
    window_rows = _marginal_rows(group_rows, "window_id", scores)
    prefix = evaluator.reports_dir / f"channel_{condition.label}"
    pd.DataFrame(raw_rows).to_csv(prefix.with_name(f"{prefix.name}_delta020_raw.csv"), index=False)
    pd.DataFrame(group_rows).to_csv(prefix.with_name(f"{prefix.name}_delta020_groups.csv"), index=False)
    pd.DataFrame(channel_rows).to_csv(prefix.with_name(f"{prefix.name}_delta020_channel_marginal.csv"), index=False)
    pd.DataFrame(window_rows).to_csv(prefix.with_name(f"{prefix.name}_delta020_window_marginal.csv"), index=False)
    top2 = [str(row["channel"]) for row in channel_rows[:2]]
    return {
        "condition_index": condition.index,
        "v_max": condition.v_max,
        "boundary_method": boundary.method,
        "boundary_theta_hash": boundary.theta_hash,
        "delta": delta,
        "repeat_count": repeats,
        "base_rho_xy_velocity": boundary.rho_mean,
        "channel_participation_ratio": _participation_ratio([row["weight"] for row in channel_rows]),
        "window_participation_ratio": _participation_ratio([row["weight"] for row in window_rows]),
        "active_channels_top80": _top_cumulative(channel_rows, 0.80),
        "active_windows_top80": _top_cumulative(window_rows, 0.80),
        "top2_channels": ",".join(top2),
        "roll_pitch_top2": set(top2) == set(ACTIVE_CHANNELS),
        "channel_shares": _share_string(channel_rows, "channel"),
        "window_shares": _share_string(window_rows, "window_id"),
        "top8_groups_by_sensitivity": ",".join(
            str(int(row["group_id"])) for row in sorted(group_rows, key=lambda r: r["directional_sensitivity"], reverse=True)[:8]
        ),
        "distribution_directional_sensitivity": _distribution([row["directional_sensitivity"] for row in group_rows]),
        "raw_rows": str(prefix.with_name(f"{prefix.name}_delta020_raw.csv")),
        "group_rows": str(prefix.with_name(f"{prefix.name}_delta020_groups.csv")),
        "channel_marginal": str(prefix.with_name(f"{prefix.name}_delta020_channel_marginal.csv")),
        "window_marginal": str(prefix.with_name(f"{prefix.name}_delta020_window_marginal.csv")),
    }


def _sample_uniform_feasible(config: ExperimentConfig, groups: list[Group], rng: np.random.Generator) -> np.ndarray:
    channels = list(config.input["channels"])
    n_windows = int(round(float(config.input["horizon_s"]) / float(config.input["window_s"])))
    min_value = float(config.input["min_value"])
    max_value = float(config.input["max_value"])
    max_step = float(config.input["max_delta_per_window"])
    by_key = {(group.window_id, group.channel): group.group_id for group in groups}
    theta = np.zeros(len(groups), dtype=float)
    for channel in channels:
        prev = 0.0
        for window_id in range(n_windows):
            lo = max(min_value, prev - max_step)
            hi = min(max_value, prev + max_step)
            value = float(rng.uniform(lo, hi))
            theta[by_key[(window_id, channel)]] = value
            prev = value
    return project_theta(theta, config)


def _build_summary(
    *,
    output_dir: Path,
    scenario_id: str,
    seed: int,
    conditions: list[Condition],
    theta_v_path: Path,
    anchor: BoundaryResult,
    warm_results: list[BoundaryResult],
    cold_results: dict[str, list[BoundaryResult]],
    channel_measurements: list[dict],
    total_queries: int,
    elapsed_wall_time_s: float,
) -> dict:
    cold_summary = {method: [_result_summary(row) for row in rows] for method, rows in cold_results.items()}
    warm_summary = [_result_summary(row) for row in warm_results]
    ratios = []
    warm_tail_queries = sum(row.query_count for row in warm_results[1:])
    for method, rows in cold_results.items():
        cold_all = sum(row.query_count for row in rows)
        cold_condition1 = next((row.query_count for row in rows if row.condition_index == 1), 0)
        campaign = cold_condition1 + warm_tail_queries
        ratios.append(
            {
                "cold_baseline": method,
                "cold_condition1_queries": cold_condition1,
                "warm_conditions_2_3_queries": warm_tail_queries,
                "warm_campaign_queries": campaign,
                "cold_all_3_conditions_queries": cold_all,
                "campaign_query_ratio": campaign / cold_all if cold_all > 0 else float("nan"),
                "cold_over_warm_speedup": cold_all / campaign if campaign > 0 else float("nan"),
            }
        )
    displacements = _boundary_displacements(warm_results)
    stable = bool(channel_measurements) and all(row.get("roll_pitch_top2") for row in channel_measurements)
    min_speedup = min((row["cold_over_warm_speedup"] for row in ratios if math.isfinite(row["cold_over_warm_speedup"])), default=float("nan"))
    return {
        "status": "complete",
        "scenario_id": scenario_id,
        "seed": seed,
        "property": TARGET_PROPERTY,
        "cross_condition_axis": "post_neutral_xy_velocity.v_max_mps",
        "v_max_conditions": [condition.v_max for condition in conditions],
        "theta_V_source": str(theta_v_path),
        "active_channels_reused_for_warm_start": ACTIVE_CHANNELS,
        "warm_start_framing": "empirically measured property-conditioned active channels",
        "localization_repeats": 5,
        "cold_results": cold_summary,
        "warm_results": warm_summary,
        "campaign_query_ratios": ratios,
        "warm_boundary_displacements": displacements,
        "channel_measurements": channel_measurements,
        "active_channel_stability_roll_pitch_top2": stable,
        "minimum_cold_over_warm_speedup": min_speedup,
        "headline_pass_speedup_ge_2_all_cold_baselines": bool(math.isfinite(min_speedup) and min_speedup >= 2.0),
        "total_query_count": total_queries,
        "elapsed_wall_time_s": elapsed_wall_time_s,
        "artifacts": {
            "report": str(output_dir / "reports" / "route1_h2_report.md"),
            "summary": str(output_dir / "reports" / "route1_h2_summary.json"),
            "query_evaluations": str(output_dir / "reports" / "query_evaluations.csv"),
            "boundaries": str(output_dir / "reports" / "boundary_results.csv"),
        },
        "caveat": (
            "v_max tightening moves the boundary largely along the input-scale direction; "
            "a follow-on initial-state axis is needed before making a stronger method claim."
        ),
    }


def _boundary_displacements(results: list[BoundaryResult]) -> list[dict]:
    rows = []
    complete = [row for row in results if row.theta is not None]
    for left, right in zip(complete[:-1], complete[1:]):
        diff = np.asarray(right.theta, dtype=float) - np.asarray(left.theta, dtype=float)
        rows.append(
            {
                "from_v_max": left.v_max,
                "to_v_max": right.v_max,
                "from_theta_hash": left.theta_hash,
                "to_theta_hash": right.theta_hash,
                "l2": float(np.linalg.norm(diff)),
                "linf": float(np.max(np.abs(diff))) if diff.size else 0.0,
                "relative_to_from_l2": float(np.linalg.norm(diff) / np.linalg.norm(left.theta))
                if np.linalg.norm(left.theta) > 0
                else float("nan"),
            }
        )
    return rows


def _result_summary(result: BoundaryResult) -> dict:
    return {
        "method": result.method,
        "condition_index": result.condition_index,
        "v_max": result.v_max,
        "status": result.status,
        "theta_hash": result.theta_hash,
        "rho_mean": result.rho_mean,
        "rho_std": result.rho_std,
        "query_count": result.query_count,
        "detail": result.detail,
    }


def _with_query_count(result: BoundaryResult, query_count: int) -> BoundaryResult:
    return BoundaryResult(
        method=result.method,
        condition_index=result.condition_index,
        v_max=result.v_max,
        status=result.status,
        theta=result.theta,
        theta_hash=result.theta_hash,
        rho_mean=result.rho_mean,
        rho_std=result.rho_std,
        query_count=query_count,
        detail=result.detail,
    )


def _save_boundary(boundaries_dir: Path, result: BoundaryResult) -> None:
    if result.theta is None:
        return
    path = _boundary_path(boundaries_dir, result)
    np.save(path, result.theta)


def _boundary_path(boundaries_dir: Path, result: BoundaryResult) -> Path:
    return boundaries_dir / f"{result.method}_condition{result.condition_index}_v{_v_label(result.v_max)}_{result.theta_hash}.npy"


def _boundary_row(result: BoundaryResult, boundaries_dir: Path) -> dict:
    row = _result_summary(result)
    row.pop("detail")
    row["detail_json"] = json.dumps(result.detail, sort_keys=True)
    row["theta_path"] = str(_boundary_path(boundaries_dir, result)) if result.theta is not None else ""
    return row


def _write_boundary_rows(reports_dir: Path, rows: list[dict]) -> None:
    pd.DataFrame(rows).to_csv(reports_dir / "boundary_results.csv", index=False)


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
    return sorted(rows, key=lambda row: row["weight"], reverse=True)


def _participation_ratio(weights) -> float:
    values = np.asarray(list(weights), dtype=float)
    denom = float(np.sum(values * values))
    if denom <= 0.0:
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


def _share_string(rows: list[dict], key: str) -> str:
    return ",".join(f"{row[key]}:{row['share']:.3f}" for row in rows)


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


def _write_report(path: Path, summary: dict) -> None:
    lines = [
        "# Route-1 H2 Cross-Condition Warm-Start Pilot",
        "",
        f"Scope: `{summary['scenario_id']}`, seed {summary['seed']}, property `{summary['property']}`.",
        f"Cross-condition axis: `{summary['cross_condition_axis']}` = {summary['v_max_conditions']}.",
        "",
        "Warm-start uses the empirically measured property-conditioned active channels: "
        + ", ".join(summary["active_channels_reused_for_warm_start"])
        + ".",
        "",
        "## Campaign Query Ratios",
        "",
        "| cold baseline | cold c1 | warm c2+c3 | warm campaign | cold all 3 | ratio | speedup |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary["campaign_query_ratios"]:
        lines.append(
            f"| {row['cold_baseline']} | {row['cold_condition1_queries']} | "
            f"{row['warm_conditions_2_3_queries']} | {row['warm_campaign_queries']} | "
            f"{row['cold_all_3_conditions_queries']} | {row['campaign_query_ratio']:.3f} | "
            f"{row['cold_over_warm_speedup']:.2f} |"
        )
    lines.extend(
        [
            "",
            "## Warm Boundaries",
            "",
            "| condition | v_max | status | rho mean | rho std | queries | theta hash |",
            "| ---: | ---: | --- | ---: | ---: | ---: | --- |",
        ]
    )
    for row in summary["warm_results"]:
        lines.append(
            f"| {row['condition_index']} | {row['v_max']:.3f} | {row['status']} | "
            f"{row['rho_mean']:.6f} | {row['rho_std']:.6f} | {row['query_count']} | {row['theta_hash']} |"
        )
    lines.extend(
        [
            "",
            "## Boundary Displacement",
            "",
            "| from v_max | to v_max | L2 | Linf | relative L2 |",
            "| ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in summary["warm_boundary_displacements"]:
        lines.append(
            f"| {row['from_v_max']:.3f} | {row['to_v_max']:.3f} | {row['l2']:.6f} | "
            f"{row['linf']:.6f} | {row['relative_to_from_l2']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Cold Boundaries",
            "",
            "| method | condition | v_max | status | rho mean | rho std | queries | theta hash |",
            "| --- | ---: | ---: | --- | ---: | ---: | ---: | --- |",
        ]
    )
    for method in ["structured", "uniform", "descent"]:
        for row in summary["cold_results"][method]:
            lines.append(
                f"| {method} | {row['condition_index']} | {row['v_max']:.3f} | {row['status']} | "
                f"{row['rho_mean']:.6f} | {row['rho_std']:.6f} | {row['query_count']} | {row['theta_hash']} |"
            )
    lines.extend(
        [
            "",
            "## Active-Channel Stability",
            "",
            "| condition | v_max | top2 channels | top80 channels | channel shares | roll+pitch top2 | queries |",
            "| ---: | ---: | --- | --- | --- | --- | ---: |",
        ]
    )
    for row in summary["channel_measurements"]:
        lines.append(
            f"| {row['condition_index']} | {row['v_max']:.3f} | {row['top2_channels']} | "
            f"{row['active_channels_top80']} | {row['channel_shares']} | {row['roll_pitch_top2']} | "
            f"{row.get('query_count', 0)} |"
        )
    lines.extend(
        [
            "",
            "## Decision Inputs",
            "",
            f"- speedup >= 2x for all cold baselines: `{summary['headline_pass_speedup_ge_2_all_cold_baselines']}`",
            f"- minimum cold/warm speedup: `{summary['minimum_cold_over_warm_speedup']:.2f}`",
            f"- roll+pitch top2 at measured conditions: `{summary['active_channel_stability_roll_pitch_top2']}`",
            "",
            "Caveat: "
            + summary["caveat"],
            "",
            "## Artifacts",
            "",
        ]
    )
    for key, value in summary["artifacts"].items():
        lines.append(f"- {key}: `{value}`")
    lines.extend(
        [
            "",
            f"Total counted queries: {summary['total_query_count']}.",
            f"Elapsed wall time: {summary['elapsed_wall_time_s']:.1f}s.",
            "",
            "Stop point: 3-condition pilot only.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_json(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _maybe_int(value: str):
    try:
        return int(value)
    except ValueError:
        return value


def _v_label(v_max: float) -> str:
    return f"{int(round(float(v_max) * 1000)):04d}"


def _scale_label(value: float) -> str:
    return f"{float(value):.3f}".replace(".", "p").replace("-", "m")


def _safe_label(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in value)[:180]


if __name__ == "__main__":
    main()
