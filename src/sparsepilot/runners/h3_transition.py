from __future__ import annotations

import argparse
import json
import math
import re
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from pyulog import ULog

from sparsepilot.config import ExperimentConfig, ScenarioCfg, load_config
from sparsepilot.groups import Group, build_groups
from sparsepilot.input_model import project_theta, theta_to_sequence, zero_theta
from sparsepilot.query import QueryResult, read_parsed_log, theta_hash
from sparsepilot.runners.fd_snapshot import _run_query_with_retry
from sparsepilot.violation_search import (
    grid_to_theta,
    random_block_theta,
    random_walk_theta,
    saturation_summary,
    theta_from_channel_values,
    theta_to_grid,
    window_count,
)


PROPERTIES = ["post_neutral_xy_drift", "post_neutral_alt_drift", "post_neutral_xy_velocity"]
DEFAULT_T_GRID = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
AUTO_LOITER_NAV_STATE = 4
POSCTL_NAV_STATE = 2


@dataclass(frozen=True)
class CandidateU:
    label: str
    theta: np.ndarray
    source: str


@dataclass
class EvalResult:
    eval_id: int
    label: str
    scenario_id: str
    t_switch_s: float | None
    theta_hash: str
    query_ids: list[str]
    parsed_log_paths: list[str]
    raw_ulg_paths: list[str]
    stats: dict[str, dict[str, float]]
    metadata: list[dict[str, Any]]


class H3Evaluator:
    def __init__(self, config: ExperimentConfig, seed: int, output_dir: Path, groups: list[Group], repeats: int):
        self.config = config
        self.seed = seed
        self.output_dir = Path(output_dir)
        self.groups = groups
        self.repeats = repeats
        self.reports_dir = self.output_dir / "reports"
        self.reports_dir.mkdir(parents=True, exist_ok=True)
        self.eval_counter = 0
        self.query_counter = 0
        self.repeat_rows: list[dict[str, Any]] = []
        self.eval_rows: list[dict[str, Any]] = []
        self.repeat_rows_path = self.reports_dir / "h3_query_repeats.csv"
        self.eval_rows_path = self.reports_dir / "h3_eval_stats.csv"

    def eval(self, theta: np.ndarray, scenario: ScenarioCfg, label: str, query_type: str) -> EvalResult:
        projected = project_theta(np.asarray(theta, dtype=float), self.config)
        eval_id = self.eval_counter
        self.eval_counter += 1
        safe_label = _safe_label(label)
        results: list[QueryResult] = []
        for repeat_idx in range(self.repeats):
            cache_tag = _safe_label(f"h3_{query_type}_{eval_id:05d}_{safe_label}_repeat{repeat_idx}", limit=120)
            result = _run_query_with_retry(
                projected,
                scenario,
                self.seed,
                f"h3_{query_type}",
                self.output_dir,
                self.config,
                cache_tag=cache_tag,
                use_cache=True,
            )
            results.append(result)
            self.query_counter += 1
            row = {
                "eval_id": eval_id,
                "repeat_idx": repeat_idx,
                "label": label,
                "query_type": query_type,
                "scenario_id": scenario.id,
                "t_switch_s": getattr(scenario, "t_switch_s", None),
                "theta_hash": result.theta_hash,
                "query_id": result.query_id,
                "parsed_log_path": str(result.parsed_log_path),
                "raw_ulg_path": str(result.parsed_log_path.parent / "raw_log.ulg"),
                "query_total_wall_time_s": float(result.metadata.get("total_wall_time_s", math.nan)),
                "transition_request_count": result.metadata.get("adapter_transition_request_count", math.nan),
                "transition_observed_t_s": result.metadata.get("adapter_transition_observed_t_s", math.nan),
            }
            for prop, value in result.robustness.items():
                row[f"rho_{prop}"] = float(value)
            self.repeat_rows.append(row)
            self._write_rows()
            print(
                f"h3_query eval={eval_id:05d} repeat={repeat_idx + 1}/{self.repeats} "
                f"scenario={scenario.id} t_switch={getattr(scenario, 't_switch_s', None)} "
                + " ".join(f"{p}={result.robustness[p]:.4f}" for p in scenario.properties),
                flush=True,
            )

        stats = _stats_from_results(results, scenario.properties)
        for prop, stat in stats.items():
            self.eval_rows.append(
                {
                    "eval_id": eval_id,
                    "label": label,
                    "query_type": query_type,
                    "scenario_id": scenario.id,
                    "t_switch_s": getattr(scenario, "t_switch_s", None),
                    "theta_hash": results[0].theta_hash,
                    "property": prop,
                    **stat,
                }
            )
        self._write_rows()
        return EvalResult(
            eval_id=eval_id,
            label=label,
            scenario_id=scenario.id,
            t_switch_s=getattr(scenario, "t_switch_s", None),
            theta_hash=results[0].theta_hash,
            query_ids=[r.query_id for r in results],
            parsed_log_paths=[str(r.parsed_log_path) for r in results],
            raw_ulg_paths=[str(r.parsed_log_path.parent / "raw_log.ulg") for r in results],
            stats=stats,
            metadata=[r.metadata for r in results],
        )

    def _write_rows(self) -> None:
        if self.repeat_rows:
            pd.DataFrame(self.repeat_rows).to_csv(self.repeat_rows_path, index=False)
        if self.eval_rows:
            pd.DataFrame(self.eval_rows).to_csv(self.eval_rows_path, index=False)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="H3 PX4 POSCTL->AUTO_LOITER transition-discontinuity probe with J=5 gates."
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/h3_transition_seed0_v0")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--rng-seed", type=int, default=20260531)
    parser.add_argument("--aware-candidates", type=int, default=6)
    parser.add_argument("--joint-random-count", type=int, default=6)
    parser.add_argument("--t-grid", default="1,2,3,4,5,6,7")
    parser.add_argument("--stage-b-query-budget", type=int, default=120)
    parser.add_argument("--skip-stage-b", action="store_true")
    parser.add_argument("--stop-after-stage0", action="store_true")
    args = parser.parse_args()

    if args.seed != 0:
        raise ValueError("H3 probe is frozen to seed 0")
    if args.repeats != 5:
        raise ValueError("H3 discipline requires J=5 for every rho")

    t0 = time.monotonic()
    output_dir = Path(args.run_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    config = _config_for_run_dir(load_config(args.config), output_dir)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"H3 is frozen to D=40, found D={len(groups)}")
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(output_dir / "groups.csv", index=False)

    t_grid = [float(x) for x in args.t_grid.split(",") if x.strip()]
    rng = np.random.default_rng(args.rng_seed)
    evaluator = H3Evaluator(config, args.seed, output_dir, groups, args.repeats)
    summary: dict[str, Any] = {
        "scenario": "px4_transition",
        "seed": args.seed,
        "repeats": args.repeats,
        "t_grid": t_grid,
        "stage_b_query_budget": args.stage_b_query_budget,
    }

    print(f"h3_start run_dir={output_dir} seed={args.seed} repeats={args.repeats}", flush=True)
    stage0 = run_stage0(evaluator, t_grid)
    summary["stage0"] = stage0
    _write_json(evaluator.reports_dir / "h3_summary.json", {**summary, "elapsed_wall_time_s": time.monotonic() - t0})
    if args.stop_after_stage0:
        summary["outcome"] = "stage0_passed" if stage0["gate_pass"] else "stage0_failed"
        summary["total_query_count"] = evaluator.query_counter
        summary["elapsed_wall_time_s"] = time.monotonic() - t0
        _finalize_report(evaluator, summary)
        print(f"h3_stop after_stage0 outcome={summary['outcome']} report={evaluator.reports_dir / 'h3_report.md'}", flush=True)
        return
    if not stage0["gate_pass"]:
        summary["outcome"] = "stage0_failed"
        summary["total_query_count"] = evaluator.query_counter
        summary["elapsed_wall_time_s"] = time.monotonic() - t0
        _finalize_report(evaluator, summary)
        print(f"h3_stop stage0_failed report={evaluator.reports_dir / 'h3_report.md'}", flush=True)
        return

    stage_a = run_stage_a(
        evaluator,
        rng,
        t_grid,
        aware_candidate_limit=int(args.aware_candidates),
        joint_random_count=int(args.joint_random_count),
    )
    summary["stage_a"] = stage_a
    robust_hits = stage_a["robust_transition_violation_count"]
    if robust_hits == 0:
        summary["outcome"] = "stage_a_failed_no_robust_transition_violation"
        summary["total_query_count"] = evaluator.query_counter
        summary["elapsed_wall_time_s"] = time.monotonic() - t0
        _finalize_report(evaluator, summary)
        print(f"h3_stop stage_a_no_robust_hit report={evaluator.reports_dir / 'h3_report.md'}", flush=True)
        return

    if args.skip_stage_b:
        summary["outcome"] = "stage_a_passed_stage_b_skipped"
    else:
        stage_b = run_stage_b(evaluator, rng, t_grid, int(args.stage_b_query_budget))
        summary["stage_b"] = stage_b
        summary["outcome"] = "stage_b_complete"

    summary["total_query_count"] = evaluator.query_counter
    summary["elapsed_wall_time_s"] = time.monotonic() - t0
    _finalize_report(evaluator, summary)
    print(
        f"h3_complete outcome={summary['outcome']} queries={evaluator.query_counter} "
        f"elapsed={summary['elapsed_wall_time_s']:.1f}s report={evaluator.reports_dir / 'h3_report.md'}",
        flush=True,
    )


def run_stage0(evaluator: H3Evaluator, t_grid: list[float]) -> dict[str, Any]:
    config = evaluator.config
    groups = evaluator.groups
    zero = zero_theta(groups)
    posctl = config.scenario_by_id("px4_position")
    transition_t5 = _transition_scenario(config, 5.0)
    maneuver_t_switch = 4.0
    transition_maneuver = _transition_scenario(config, maneuver_t_switch)
    auto_loiter = _auto_loiter_scenario(config)
    maneuver = _stage0_maneuver(config, groups)

    print("h3_stage0_start", flush=True)
    neutral_posctl = evaluator.eval(zero, posctl, "stage0_zero_posctl", "stage0")
    neutral_auto = evaluator.eval(zero, auto_loiter, "stage0_zero_auto_loiter", "stage0")
    neutral_switch = evaluator.eval(zero, transition_t5, "stage0_zero_switch_t5", "stage0")
    maneuver_switch = evaluator.eval(
        maneuver, transition_maneuver, f"stage0_maneuver_switch_t{_t_label(maneuver_t_switch)}", "stage0"
    )
    maneuver_posctl = evaluator.eval(maneuver, posctl, "stage0_maneuver_posctl", "stage0")

    switch_validations = _validate_transition_eval(neutral_switch, 5.0) + _validate_transition_eval(
        maneuver_switch, maneuver_t_switch
    )
    neutral_safe = all(_all_properties_robust_safe(x) for x in [neutral_posctl, neutral_auto, neutral_switch])
    switch_clean = all(row["gate_ok"] for row in switch_validations)
    diff = _trajectory_difference(maneuver_switch, maneuver_posctl, start_s=maneuver_t_switch)
    maneuver_induced_motion = diff["posctl_speed_at_switch_mps"] > 0.20 or diff["posctl_xy_span_m"] > 0.25
    switched_differs = (
        diff["max_xy_delta_m"] > 0.15
        or diff["max_alt_delta_m"] > 0.15
        or max(abs(v) for v in diff["rho_mean_delta"].values()) > 0.05
    )
    gate_pass = bool(neutral_safe and switch_clean and maneuver_induced_motion and switched_differs)

    validation_path = evaluator.reports_dir / "stage0_transition_validation.csv"
    pd.DataFrame(switch_validations).to_csv(validation_path, index=False)
    stage0 = {
        "gate_pass": gate_pass,
        "neutral_safe_2std": neutral_safe,
        "switch_clean": switch_clean,
        "maneuver_induced_motion": maneuver_induced_motion,
        "switched_run_differs_from_posctl": switched_differs,
        "trajectory_difference": diff,
        "transition_validation_rows": str(validation_path),
        "eval_ids": {
            "neutral_posctl": neutral_posctl.eval_id,
            "neutral_auto_loiter": neutral_auto.eval_id,
            "neutral_switch_t5": neutral_switch.eval_id,
            f"maneuver_switch_t{_t_label(maneuver_t_switch)}": maneuver_switch.eval_id,
            "maneuver_posctl": maneuver_posctl.eval_id,
        },
    }
    print(
        f"h3_stage0_complete gate={gate_pass} neutral_safe={neutral_safe} "
        f"switch_clean={switch_clean} switched_differs={switched_differs}",
        flush=True,
    )
    return stage0


def run_stage_a(
    evaluator: H3Evaluator,
    rng: np.random.Generator,
    t_grid: list[float],
    *,
    aware_candidate_limit: int,
    joint_random_count: int,
) -> dict[str, Any]:
    print(
        f"h3_stageA_start aware_candidates={aware_candidate_limit} "
        f"joint_random_count={joint_random_count} t_grid={t_grid}",
        flush=True,
    )
    config = evaluator.config
    groups = evaluator.groups
    posctl = config.scenario_by_id("px4_position")
    pair_rows: list[dict[str, Any]] = []
    no_switch_cache: dict[str, EvalResult] = {}
    full_sweep_hashes: set[tuple[str, str]] = set()

    def no_switch_eval(candidate: CandidateU, source: str) -> EvalResult:
        thash = theta_hash(project_theta(candidate.theta, config))
        if thash not in no_switch_cache:
            no_switch_cache[thash] = evaluator.eval(candidate.theta, posctl, f"stageA_{source}_{candidate.label}_no_switch", "stageA")
        return no_switch_cache[thash]

    candidates = _stage_a_candidates(config, groups, rng, aware_candidate_limit)
    candidate_by_hash = {theta_hash(project_theta(candidate.theta, config)): candidate for candidate in candidates}
    for idx, candidate in enumerate(candidates):
        ns = no_switch_eval(candidate, "aware")
        safe_props = [prop for prop in PROPERTIES if _robust_safe(ns.stats[prop], sigma=2.0)]
        if not safe_props:
            pair_rows.extend(_skipped_no_switch_rows(candidate, ns, "transition_aware", t_grid))
            _write_pair_rows(evaluator.reports_dir / "stageA_pair_classification.csv", pair_rows)
            continue
        _sweep_candidate(
            evaluator,
            candidate,
            ns,
            t_grid,
            "transition_aware",
            pair_rows,
            full_sweep_hashes,
        )
        print(
            f"h3_stageA_aware_candidate {idx + 1}/{len(candidates)} label={candidate.label} "
            f"queries={evaluator.query_counter}",
            flush=True,
        )

    random_probe_hits_before = _robust_hit_count(pair_rows)
    for sample_idx in range(joint_random_count):
        candidate = CandidateU(
            label=f"joint_random_{sample_idx:03d}",
            theta=_sample_uniform_interior(config, groups, rng),
            source="joint_random",
        )
        candidate_by_hash[theta_hash(project_theta(candidate.theta, config))] = candidate
        t_switch = round(float(rng.uniform(min(t_grid), max(t_grid))), 2)
        ns = no_switch_eval(candidate, "joint_random")
        sw = evaluator.eval(
            candidate.theta,
            _transition_scenario(config, t_switch),
            f"stageA_joint_random_{sample_idx:03d}_t{_t_label(t_switch)}",
            "stageA",
        )
        pair_rows.extend(_classify_pair(candidate, sw, ns, "joint_random_probe", t_switch, config, groups))
        _write_pair_rows(evaluator.reports_dir / "stageA_pair_classification.csv", pair_rows)

    random_probe_hits_after = _robust_hit_count(pair_rows)
    random_probe_hit_count = random_probe_hits_after - random_probe_hits_before

    robust_hit_candidates = _robust_hit_candidates(pair_rows)
    for thash, label in robust_hit_candidates:
        if (thash, label) in full_sweep_hashes:
            continue
        candidate = candidate_by_hash.get(thash)
        if candidate is None:
            continue
        ns = no_switch_eval(candidate, "robust_hit_sweep")
        _sweep_candidate(evaluator, candidate, ns, t_grid, "post_hit_sweep", pair_rows, full_sweep_hashes)

    pair_path = evaluator.reports_dir / "stageA_pair_classification.csv"
    _write_pair_rows(pair_path, pair_rows)
    robust_rows = [r for r in pair_rows if r["robust_transition_violation"]]
    weak_rows = [r for r in pair_rows if r["weak_1std_candidate"]]
    noise_rows = [r for r in pair_rows if r["noise_straddle_not_counted"]]
    robust_specific_rows = [r for r in robust_rows if r["t_specific_window_observed"]]
    clusters = _cluster_summary(robust_specific_rows)
    cluster_path = evaluator.reports_dir / "stageA_robust_clusters.csv"
    pd.DataFrame(clusters).to_csv(cluster_path, index=False)
    stage_a = {
        "robust_transition_violation_count": len(robust_specific_rows),
        "raw_robust_pair_count_before_t_specific_filter": len(robust_rows),
        "weak_1std_candidate_count": len(weak_rows),
        "noise_straddle_not_counted_count": len(noise_rows),
        "joint_random_probe_hit_count": random_probe_hit_count,
        "distinct_cluster_count": len(clusters),
        "pair_rows": str(pair_path),
        "cluster_rows": str(cluster_path),
        "query_count_after_stage_a": evaluator.query_counter,
    }
    print(
        f"h3_stageA_complete robust={stage_a['robust_transition_violation_count']} "
        f"weak={len(weak_rows)} noise_straddles={len(noise_rows)} "
        f"joint_random_hits={random_probe_hit_count}",
        flush=True,
    )
    return stage_a


def run_stage_b(
    evaluator: H3Evaluator,
    rng: np.random.Generator,
    t_grid: list[float],
    query_budget: int,
) -> dict[str, Any]:
    print(f"h3_stageB_start matched_query_budget={query_budget}", flush=True)
    rows: list[dict[str, Any]] = []
    summaries = []
    for method, runner in [
        ("baseline_uniform", _stage_b_uniform),
        ("baseline_descent", _stage_b_descent),
        ("transition_aware", _stage_b_transition_aware),
    ]:
        start_queries = evaluator.query_counter
        method_rows = runner(evaluator, rng, t_grid, query_budget)
        rows.extend(method_rows)
        used = evaluator.query_counter - start_queries
        robust_rows = [r for r in method_rows if r["robust_transition_violation"] and r["t_specific_window_observed"]]
        clusters = _cluster_summary(robust_rows)
        summaries.append(
            {
                "method": method,
                "query_budget": query_budget,
                "query_count": used,
                "evaluated_pairs": len({(r["method"], r["candidate_label"], r["t_switch_s"]) for r in method_rows}),
                "robust_transition_violations": len(robust_rows),
                "distinct_clusters": len(clusters),
                "t_window_coverage": ",".join(sorted({r["t_window"] for r in robust_rows})),
            }
        )
        print(
            f"h3_stageB_method method={method} queries={used} "
            f"robust={len(robust_rows)} clusters={len(clusters)}",
            flush=True,
        )

    rows_path = evaluator.reports_dir / "stageB_pair_classification.csv"
    summary_path = evaluator.reports_dir / "stageB_method_summary.csv"
    pd.DataFrame(rows).to_csv(rows_path, index=False)
    pd.DataFrame(summaries).to_csv(summary_path, index=False)
    return {"pair_rows": str(rows_path), "method_summary_rows": str(summary_path), "methods": summaries}


def _stage_b_uniform(
    evaluator: H3Evaluator, rng: np.random.Generator, t_grid: list[float], query_budget: int
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    config = evaluator.config
    groups = evaluator.groups
    posctl = config.scenario_by_id("px4_position")
    start = evaluator.query_counter
    idx = 0
    while evaluator.query_counter - start + evaluator.repeats * 2 <= query_budget:
        candidate = CandidateU(f"b_uniform_{idx:03d}", _sample_uniform_interior(config, groups, rng), "stageB_uniform")
        t_switch = round(float(rng.uniform(min(t_grid), max(t_grid))), 2)
        ns = evaluator.eval(candidate.theta, posctl, f"stageB_uniform_{idx:03d}_no_switch", "stageB_uniform")
        sw = evaluator.eval(
            candidate.theta,
            _transition_scenario(config, t_switch),
            f"stageB_uniform_{idx:03d}_t{_t_label(t_switch)}",
            "stageB_uniform",
        )
        rows.extend(_classify_pair(candidate, sw, ns, "baseline_uniform", t_switch, config, groups))
        idx += 1
    return _mark_t_specific(rows)


def _stage_b_descent(
    evaluator: H3Evaluator, rng: np.random.Generator, t_grid: list[float], query_budget: int
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    config = evaluator.config
    groups = evaluator.groups
    posctl = config.scenario_by_id("px4_position")
    start = evaluator.query_counter
    current = _sample_uniform_interior(config, groups, rng)
    current_t = float(rng.choice(t_grid))
    best_obj = math.inf
    step_scale = 0.22
    idx = 0
    while evaluator.query_counter - start + evaluator.repeats * 2 <= query_budget:
        candidate = CandidateU(f"b_descent_{idx:03d}", current, "stageB_descent")
        ns = evaluator.eval(candidate.theta, posctl, f"stageB_descent_{idx:03d}_no_switch", "stageB_descent")
        sw = evaluator.eval(
            candidate.theta,
            _transition_scenario(config, current_t),
            f"stageB_descent_{idx:03d}_t{_t_label(current_t)}",
            "stageB_descent",
        )
        rows.extend(_classify_pair(candidate, sw, ns, "baseline_descent", current_t, config, groups))
        obj = min(float(sw.stats[prop]["mean"]) for prop in PROPERTIES)
        if obj < best_obj:
            best_obj = obj
        proposal = current + rng.normal(0.0, step_scale, size=current.shape)
        current = project_theta(proposal, config)
        current_t = float(np.clip(current_t + rng.normal(0.0, 0.75), min(t_grid), max(t_grid)))
        step_scale = max(0.06, step_scale * 0.96)
        idx += 1
    return _mark_t_specific(rows)


def _stage_b_transition_aware(
    evaluator: H3Evaluator, rng: np.random.Generator, t_grid: list[float], query_budget: int
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    config = evaluator.config
    groups = evaluator.groups
    posctl = config.scenario_by_id("px4_position")
    start = evaluator.query_counter
    idx = 0
    while evaluator.query_counter - start + evaluator.repeats <= query_budget:
        candidate = CandidateU(f"b_aware_{idx:03d}", _sample_uniform_interior(config, groups, rng), "stageB_aware")
        ns = evaluator.eval(candidate.theta, posctl, f"stageB_aware_{idx:03d}_no_switch", "stageB_aware")
        shuffled_t = list(t_grid)
        rng.shuffle(shuffled_t)
        for t_switch in shuffled_t:
            if evaluator.query_counter - start + evaluator.repeats > query_budget:
                break
            sw = evaluator.eval(
                candidate.theta,
                _transition_scenario(config, t_switch),
                f"stageB_aware_{idx:03d}_t{_t_label(t_switch)}",
                "stageB_aware",
            )
            rows.extend(_classify_pair(candidate, sw, ns, "transition_aware", t_switch, config, groups))
        idx += 1
    return _mark_t_specific(rows)


def _sweep_candidate(
    evaluator: H3Evaluator,
    candidate: CandidateU,
    no_switch: EvalResult,
    t_grid: list[float],
    method: str,
    pair_rows: list[dict[str, Any]],
    full_sweep_hashes: set[tuple[str, str]],
) -> None:
    config = evaluator.config
    groups = evaluator.groups
    for t_switch in t_grid:
        sw = evaluator.eval(
            candidate.theta,
            _transition_scenario(config, t_switch),
            f"stageA_{method}_{candidate.label}_t{_t_label(t_switch)}",
            "stageA",
        )
        pair_rows.extend(_classify_pair(candidate, sw, no_switch, method, t_switch, config, groups))
        _write_pair_rows(evaluator.reports_dir / "stageA_pair_classification.csv", _mark_t_specific(pair_rows))
    full_sweep_hashes.add((theta_hash(project_theta(candidate.theta, config)), candidate.label))


def _classify_pair(
    candidate: CandidateU,
    switch_eval: EvalResult,
    no_switch_eval: EvalResult,
    method: str,
    t_switch: float,
    config: ExperimentConfig,
    groups: list[Group],
) -> list[dict[str, Any]]:
    theta = project_theta(candidate.theta, config)
    sat = saturation_summary(theta, config, groups, tol=0.02)
    active = _active_channel_pattern(theta, groups)
    switch_validation = []
    if getattr(switch_eval, "parsed_log_paths", None) and getattr(switch_eval, "raw_ulg_paths", None):
        switch_validation = _validate_transition_eval(switch_eval, t_switch)
    switch_clean = all(row["gate_ok"] for row in switch_validation) if switch_validation else True
    first_auto_lags = [
        float(row["mode_field_first_auto_loiter_t_s"]) - float(t_switch)
        for row in switch_validation
        if math.isfinite(float(row["mode_field_first_auto_loiter_t_s"]))
    ]
    max_first_auto_lag_s = max(first_auto_lags) if first_auto_lags else math.nan
    max_transition_requests = max(
        (float(meta.get("adapter_transition_request_count", math.nan)) for meta in getattr(switch_eval, "metadata", [])),
        default=math.nan,
    )
    rows = []
    for prop in PROPERTIES:
        sw = switch_eval.stats[prop]
        ns = no_switch_eval.stats[prop]
        robust_rho = _robust_violation(sw, 2.0) and _robust_safe(ns, 2.0)
        weak_rho = (not robust_rho) and _robust_violation(sw, 1.0) and _robust_safe(ns, 1.0)
        robust = robust_rho and switch_clean
        weak = weak_rho and switch_clean
        noise_straddle = (not robust) and float(sw["mean"]) < 0.0
        rows.append(
            {
                "method": method,
                "candidate_label": candidate.label,
                "candidate_source": candidate.source,
                "theta_hash": theta_hash(theta),
                "theta_path": "",
                "t_switch_s": float(t_switch),
                "t_window": _t_window(t_switch),
                "property": prop,
                "switch_eval_id": switch_eval.eval_id,
                "no_switch_eval_id": no_switch_eval.eval_id,
                "switch_mean": sw["mean"],
                "switch_std": sw["std"],
                "switch_mean_plus_2std": sw["mean"] + 2.0 * sw["std"],
                "switch_mean_plus_1std": sw["mean"] + sw["std"],
                "no_switch_mean": ns["mean"],
                "no_switch_std": ns["std"],
                "no_switch_mean_minus_2std": ns["mean"] - 2.0 * ns["std"],
                "no_switch_mean_minus_1std": ns["mean"] - ns["std"],
                "robust_transition_violation": robust,
                "weak_1std_candidate": weak,
                "noise_straddle_not_counted": noise_straddle,
                "rho_gate_robust_transition_violation": robust_rho,
                "rho_gate_weak_1std_candidate": weak_rho,
                "switch_clean_j5": switch_clean,
                "switch_max_first_auto_lag_s": max_first_auto_lag_s,
                "switch_max_request_count": max_transition_requests,
                "no_switch_robust_safe_2std": _robust_safe(ns, 2.0),
                "switch_robust_violation_2std": _robust_violation(sw, 2.0),
                "return_to_neutral_by_t_switch": _return_to_neutral_by(theta, t_switch, config, groups),
                "active_channel_pattern": active,
                "non_saturated": not bool(sat["amplitude_saturated"]) and float(sat["max_abs_theta"]) <= 0.62,
                "max_abs_theta": sat["max_abs_theta"],
                "mean_abs_theta": sat["mean_abs_theta"],
                "t_specific_window_observed": False,
            }
        )
    return rows


def _mark_t_specific(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_key: dict[tuple[str, str, str, str], list[dict[str, Any]]] = {}
    for row in rows:
        key = (row["method"], row["theta_hash"], row["candidate_label"], row["property"])
        by_key.setdefault(key, []).append(row)
    for key_rows in by_key.values():
        robust_t = {float(r["t_switch_s"]) for r in key_rows if r["robust_transition_violation"]}
        all_t = {float(r["t_switch_s"]) for r in key_rows}
        t_specific = bool(robust_t) and len(robust_t) < len(all_t)
        for row in key_rows:
            row["t_specific_window_observed"] = bool(row["robust_transition_violation"] and t_specific)
            row["robust_t_switch_values_for_u_property"] = ",".join(f"{x:.2f}" for x in sorted(robust_t))
            row["evaluated_t_switch_values_for_u_property"] = ",".join(f"{x:.2f}" for x in sorted(all_t))
    return rows


def _skipped_no_switch_rows(
    candidate: CandidateU, no_switch: EvalResult, method: str, t_grid: list[float]
) -> list[dict[str, Any]]:
    rows = []
    for t_switch in t_grid:
        for prop in PROPERTIES:
            ns = no_switch.stats[prop]
            rows.append(
                {
                    "method": method,
                    "candidate_label": candidate.label,
                    "candidate_source": candidate.source,
                    "theta_hash": no_switch.theta_hash,
                    "theta_path": "",
                    "t_switch_s": float(t_switch),
                    "t_window": _t_window(t_switch),
                    "property": prop,
                    "switch_eval_id": "",
                    "no_switch_eval_id": no_switch.eval_id,
                    "switch_mean": math.nan,
                    "switch_std": math.nan,
                    "switch_mean_plus_2std": math.nan,
                    "switch_mean_plus_1std": math.nan,
                    "no_switch_mean": ns["mean"],
                    "no_switch_std": ns["std"],
                    "no_switch_mean_minus_2std": ns["mean"] - 2.0 * ns["std"],
                    "no_switch_mean_minus_1std": ns["mean"] - ns["std"],
                    "robust_transition_violation": False,
                    "weak_1std_candidate": False,
                    "noise_straddle_not_counted": False,
                    "rho_gate_robust_transition_violation": False,
                    "rho_gate_weak_1std_candidate": False,
                    "switch_clean_j5": False,
                    "switch_max_first_auto_lag_s": math.nan,
                    "switch_max_request_count": math.nan,
                    "no_switch_robust_safe_2std": False,
                    "switch_robust_violation_2std": False,
                    "return_to_neutral_by_t_switch": False,
                    "active_channel_pattern": "",
                    "non_saturated": False,
                    "max_abs_theta": math.nan,
                    "mean_abs_theta": math.nan,
                    "t_specific_window_observed": False,
                    "skip_reason": "no robust-safe no-switch property under 2std",
                }
            )
    return rows


def _validate_transition_eval(eval_result: EvalResult, expected_t_switch: float) -> list[dict[str, Any]]:
    rows = []
    for repeat_idx, (parsed_path, raw_ulg_path) in enumerate(zip(eval_result.parsed_log_paths, eval_result.raw_ulg_paths)):
        parsed = read_parsed_log(Path(parsed_path))
        mode_summary = _validate_parsed_mode_switch(parsed, expected_t_switch)
        ulog_summary = _validate_ulog_switch(Path(raw_ulg_path))
        gate_ok = bool(
            mode_summary["mode_field_transition_ok"]
            and ulog_summary["posctl_to_auto_loiter_seen"]
            and ulog_summary["manual_valid_before_transition"]
        )
        rows.append(
            {
                "eval_id": eval_result.eval_id,
                "repeat_idx": repeat_idx,
                "expected_t_switch_s": expected_t_switch,
                "gate_ok": gate_ok,
                **mode_summary,
                **ulog_summary,
                "parsed_log_path": parsed_path,
                "raw_ulg_path": raw_ulg_path,
            }
        )
    return rows


def _validate_parsed_mode_switch(parsed: pd.DataFrame, expected_t_switch: float) -> dict[str, Any]:
    early = parsed[parsed["time_s"] < max(0.0, expected_t_switch - 0.25)]
    pre = parsed[(parsed["time_s"] >= max(0.0, expected_t_switch - 1.0)) & (parsed["time_s"] < expected_t_switch)]
    post = parsed[parsed["time_s"] >= expected_t_switch + 0.4]
    auto_mask = parsed.apply(_row_is_auto_loiter, axis=1)
    posctl_mask = parsed.apply(_row_is_posctl, axis=1)
    auto_after = parsed[(parsed["time_s"] >= expected_t_switch - 0.05) & auto_mask]
    first_auto_t = float(auto_after["time_s"].min()) if not auto_after.empty else math.nan
    pre_posctl = bool(posctl_mask.loc[pre.index].any()) if not pre.empty else False
    post_auto = bool(auto_mask.loc[post.index].any()) if not post.empty else False
    no_early_auto = not bool(auto_mask.loc[early.index].any()) if not early.empty else True
    observed_close = math.isfinite(first_auto_t) and expected_t_switch - 0.1 <= first_auto_t <= expected_t_switch + 1.5
    return {
        "mode_field_transition_ok": bool(pre_posctl and post_auto and no_early_auto and observed_close),
        "mode_field_pre_posctl": pre_posctl,
        "mode_field_post_auto_loiter": post_auto,
        "mode_field_no_early_auto_loiter": no_early_auto,
        "mode_field_first_auto_loiter_t_s": first_auto_t,
        "mode_values": ",".join(sorted(str(x) for x in parsed["mode"].dropna().unique())) if "mode" in parsed else "",
    }


def _validate_ulog_switch(raw_ulg_path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "ulog_parse_ok": False,
        "posctl_to_auto_loiter_seen": False,
        "manual_valid_before_transition": False,
        "ulog_transition_timestamp_us": math.nan,
        "ulog_manual_valid_fraction_1s_pre": math.nan,
        "ulog_nav_state_values": "",
        "ulog_error": "",
    }
    try:
        ulog = ULog(str(raw_ulg_path), ["vehicle_status", "manual_control_setpoint"])
        by_name = {dataset.name: dataset for dataset in ulog.data_list}
        status = by_name["vehicle_status"].data
        manual = by_name["manual_control_setpoint"].data
        nav = np.asarray(status["nav_state"], dtype=int)
        ts = np.asarray(status["timestamp"], dtype=np.int64)
        transitions = []
        for idx in range(1, len(nav)):
            if nav[idx - 1] == POSCTL_NAV_STATE and nav[idx] == AUTO_LOITER_NAV_STATE:
                transitions.append((int(ts[idx]), int(nav[idx - 1]), int(nav[idx])))
        result["ulog_parse_ok"] = True
        result["ulog_nav_state_values"] = ",".join(str(int(x)) for x in sorted(set(nav.tolist())))
        result["posctl_to_auto_loiter_seen"] = bool(transitions)
        if transitions:
            t_us = transitions[-1][0]
            m_ts = np.asarray(manual["timestamp"], dtype=np.int64)
            valid = np.asarray(manual["valid"], dtype=int)
            mask = (m_ts >= t_us - 1_000_000) & (m_ts < t_us) & (m_ts > 0)
            valid_window = valid[mask]
            valid_fraction = float(np.mean(valid_window == 1)) if valid_window.size else 0.0
            result["ulog_transition_timestamp_us"] = t_us
            result["ulog_manual_valid_fraction_1s_pre"] = valid_fraction
            result["manual_valid_before_transition"] = bool(valid_window.size > 0 and valid_fraction >= 0.95)
    except Exception as exc:
        result["ulog_error"] = str(exc)
    return result


def _trajectory_difference(switch_eval: EvalResult, posctl_eval: EvalResult, start_s: float) -> dict[str, Any]:
    switch_log = read_parsed_log(Path(switch_eval.parsed_log_paths[0]))
    posctl_log = read_parsed_log(Path(posctl_eval.parsed_log_paths[0]))
    sw = switch_log[switch_log["time_s"] >= start_s].copy()
    po = posctl_log[posctl_log["time_s"] >= start_s].copy()
    if sw.empty or po.empty:
        return {
            "max_xy_delta_m": math.nan,
            "max_alt_delta_m": math.nan,
            "posctl_xy_span_m": math.nan,
            "posctl_max_xy_speed_mps": math.nan,
            "posctl_speed_at_switch_mps": math.nan,
            "rho_mean_delta": {},
        }
    times = np.asarray(sw["time_s"], dtype=float)
    po_x = np.interp(times, po["time_s"], po["x_m"])
    po_y = np.interp(times, po["time_s"], po["y_m"])
    po_alt = np.interp(times, po["time_s"], po["alt_m"])
    xy_delta = np.sqrt((np.asarray(sw["x_m"], dtype=float) - po_x) ** 2 + (np.asarray(sw["y_m"], dtype=float) - po_y) ** 2)
    alt_delta = np.abs(np.asarray(sw["alt_m"], dtype=float) - po_alt)
    po_xy_span = math.hypot(float(po["x_m"].max() - po["x_m"].min()), float(po["y_m"].max() - po["y_m"].min()))
    po_speed = np.sqrt(np.asarray(po["vx_mps"], dtype=float) ** 2 + np.asarray(po["vy_mps"], dtype=float) ** 2)
    switch_idx = (po["time_s"] - start_s).abs().idxmin()
    switch_row = po.loc[switch_idx]
    switch_speed = math.hypot(float(switch_row["vx_mps"]), float(switch_row["vy_mps"]))
    return {
        "max_xy_delta_m": float(np.max(xy_delta)),
        "max_alt_delta_m": float(np.max(alt_delta)),
        "posctl_xy_span_m": float(po_xy_span),
        "posctl_max_xy_speed_mps": float(np.max(po_speed)),
        "posctl_speed_at_switch_mps": float(switch_speed),
        "rho_mean_delta": {
            prop: float(switch_eval.stats[prop]["mean"] - posctl_eval.stats[prop]["mean"]) for prop in PROPERTIES
        },
    }


def _stage_a_candidates(
    config: ExperimentConfig, groups: list[Group], rng: np.random.Generator, limit: int
) -> list[CandidateU]:
    candidates: list[CandidateU] = []
    seen: set[str] = set()

    def add(label: str, theta: np.ndarray, source: str) -> None:
        projected = project_theta(theta, config)
        thash = theta_hash(projected)
        if thash in seen:
            return
        seen.add(thash)
        candidates.append(CandidateU(label, projected, source))

    for path in [
        Path("runs/margin_stage1_redo_v1/theta_V.npy"),
        Path("runs/rq1_boundary_v0/theta_boundary.npy"),
        Path("runs/margin_stage0_v1/theta_117.npy"),
    ]:
        if path.exists():
            base = np.load(path)
            for scale in [0.35, 0.50, 0.65]:
                add(f"{path.stem}_s{int(scale * 100):03d}", base * scale, "prior_scaled")

    n = window_count(config)
    add(
        "pitch_forward_release_w0_w4",
        theta_from_channel_values(config, groups, {"pitch": [0.25, 0.50, 0.50, 0.25] + [0.0] * (n - 4)}),
        "hand_interior",
    )
    add(
        "roll_right_release_w0_w4",
        theta_from_channel_values(config, groups, {"roll": [0.25, 0.50, 0.50, 0.25] + [0.0] * (n - 4)}),
        "hand_interior",
    )
    add(
        "diag_roll_pitch_release_w0_w5",
        theta_from_channel_values(
            config,
            groups,
            {
                "roll": [0.25, 0.50, 0.50, 0.25, 0.0] + [0.0] * (n - 5),
                "pitch": [-0.25, -0.50, -0.50, -0.25, 0.0] + [0.0] * (n - 5),
            },
        ),
        "hand_interior",
    )
    add(
        "late_pitch_push_w6_w9",
        theta_from_channel_values(config, groups, {"pitch": [0.0] * 6 + [0.25, 0.50, 0.50, 0.25]}),
        "hand_interior",
    )

    attempts = 0
    while len(candidates) < max(limit, 1) and attempts < limit * 40 + 40:
        attempts += 1
        theta = random_walk_theta(config, groups, rng) if attempts % 2 else random_block_theta(config, groups, rng)
        grid = theta_to_grid(theta, config, groups)
        grid *= 0.75
        add(f"random_interior_{attempts:03d}", grid_to_theta(grid, config, groups), "random_interior")

    interior = [c for c in candidates if float(np.max(np.abs(c.theta))) <= 0.62]
    return (interior or candidates)[:limit]


def _stage0_maneuver(config: ExperimentConfig, groups: list[Group]) -> np.ndarray:
    n = window_count(config)
    return theta_from_channel_values(
        config,
        groups,
        {
            "pitch": [0.25, 0.50, 0.50, 0.50, 0.25] + [0.0] * (n - 5),
            "roll": [0.0, 0.25, 0.50, 0.50, 0.25] + [0.0] * (n - 5),
        },
    )


def _sample_uniform_interior(config: ExperimentConfig, groups: list[Group], rng: np.random.Generator) -> np.ndarray:
    n_windows = window_count(config)
    channels = list(config.input["channels"])
    max_step = min(float(config.input["max_delta_per_window"]), 0.20)
    min_value = max(float(config.input["min_value"]), -0.58)
    max_value = min(float(config.input["max_value"]), 0.58)
    values: dict[str, list[float]] = {}
    for channel in channels:
        current = 0.0
        channel_values = []
        for _ in range(n_windows):
            current = float(np.clip(current + rng.uniform(-max_step, max_step), min_value, max_value))
            if rng.random() < 0.15:
                current *= 0.5
            channel_values.append(current)
        values[channel] = channel_values
    return theta_from_channel_values(config, groups, values)


def _config_for_run_dir(config: ExperimentConfig, output_dir: Path) -> ExperimentConfig:
    logging = dict(config.logging)
    logging["jsonl"] = str(output_dir / "logs" / "queries.jsonl")
    scenarios = list(config.scenarios)
    if not any(s.id == "px4_transition" for s in scenarios):
        scenarios.append(
            ScenarioCfg(
                id="px4_transition",
                platform="px4",
                perturb_mode="Position",
                observe_mode="Hold",
                takeoff_alt_m=5.0,
                properties=PROPERTIES,
                t_switch_s=5.0,
            )
        )
    return replace(config, experiment_id=output_dir.name, scenarios=scenarios, logging=logging)


def _transition_scenario(config: ExperimentConfig, t_switch_s: float) -> ScenarioCfg:
    base = config.scenario_by_id("px4_transition")
    return replace(base, t_switch_s=float(t_switch_s))


def _auto_loiter_scenario(config: ExperimentConfig) -> ScenarioCfg:
    base = config.scenario_by_id("px4_transition")
    return replace(base, id="px4_auto_loiter", perturb_mode="Hold", observe_mode="Hold", t_switch_s=None)


def _stats_from_results(results: list[QueryResult], properties: list[str]) -> dict[str, dict[str, float]]:
    stats: dict[str, dict[str, float]] = {}
    for prop in properties:
        values = np.asarray([float(result.robustness[prop]) for result in results], dtype=float)
        stats[prop] = {
            "mean": float(np.mean(values)),
            "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
            "min": float(np.min(values)),
            "max": float(np.max(values)),
            "repeats": int(len(values)),
            "count_negative": int(np.sum(values < 0.0)),
        }
    return stats


def _all_properties_robust_safe(eval_result: EvalResult) -> bool:
    return all(_robust_safe(eval_result.stats[prop], 2.0) for prop in PROPERTIES)


def _robust_safe(stats: dict[str, float], sigma: float) -> bool:
    return float(stats["mean"]) - sigma * float(stats["std"]) > 0.0


def _robust_violation(stats: dict[str, float], sigma: float) -> bool:
    return float(stats["mean"]) + sigma * float(stats["std"]) < 0.0


def _row_is_posctl(row: pd.Series) -> bool:
    mode = str(row.get("mode", "")).upper()
    if mode in {"POSCTL", "POSITION"}:
        return True
    try:
        return int(row.get("px4_main_mode")) == 3
    except Exception:
        return False


def _row_is_auto_loiter(row: pd.Series) -> bool:
    mode = str(row.get("mode", "")).upper()
    if mode in {"LOITER", "HOLD", "AUTO.LOITER", "AUTO_LOITER"}:
        return True
    try:
        return int(row.get("px4_main_mode")) == 4 and int(row.get("px4_sub_mode")) == 3
    except Exception:
        return False


def _return_to_neutral_by(theta: np.ndarray, t_switch: float, config: ExperimentConfig, groups: list[Group]) -> bool:
    sequence = theta_to_sequence(theta, groups, config)
    active = sequence[(sequence["t_s"] >= t_switch) & (sequence["t_s"] < float(config.input["horizon_s"]))]
    if active.empty:
        return True
    manual_cols = list(config.input["channels"])
    return float(active[manual_cols].abs().max().max()) <= 1e-9


def _active_channel_pattern(theta: np.ndarray, groups: list[Group], eps: float = 0.05) -> str:
    active = sorted({g.channel for g in groups if abs(float(theta[g.group_id])) > eps})
    return "+".join(active) if active else "neutral"


def _robust_hit_count(rows: list[dict[str, Any]]) -> int:
    return sum(1 for row in rows if row.get("robust_transition_violation"))


def _robust_hit_candidates(rows: list[dict[str, Any]]) -> list[tuple[str, str]]:
    result = []
    seen = set()
    for row in rows:
        if row.get("robust_transition_violation"):
            key = (row["theta_hash"], row["candidate_label"])
            if key not in seen:
                seen.add(key)
                result.append(key)
    return result


def _cluster_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    clusters: dict[tuple[str, str, str], dict[str, Any]] = {}
    for row in rows:
        key = (row["active_channel_pattern"], row["t_window"], row["property"])
        cluster = clusters.setdefault(
            key,
            {
                "active_channel_pattern": row["active_channel_pattern"],
                "t_window": row["t_window"],
                "property": row["property"],
                "count": 0,
                "methods": set(),
                "theta_hashes": set(),
            },
        )
        cluster["count"] += 1
        cluster["methods"].add(row["method"])
        cluster["theta_hashes"].add(row["theta_hash"])
    result = []
    for cluster in clusters.values():
        result.append(
            {
                "active_channel_pattern": cluster["active_channel_pattern"],
                "t_window": cluster["t_window"],
                "property": cluster["property"],
                "count": cluster["count"],
                "methods": ",".join(sorted(cluster["methods"])),
                "distinct_theta_hashes": len(cluster["theta_hashes"]),
            }
        )
    return sorted(result, key=lambda r: (r["property"], r["active_channel_pattern"], r["t_window"]))


def _write_pair_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    if rows:
        pd.DataFrame(_mark_t_specific(rows)).to_csv(path, index=False)


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _finalize_report(evaluator: H3Evaluator, summary: dict[str, Any]) -> None:
    summary_path = evaluator.reports_dir / "h3_summary.json"
    report_path = evaluator.reports_dir / "h3_report.md"
    _write_json(summary_path, summary)
    lines = [
        "# H3 Transition-Discontinuity Probe",
        "",
        f"- outcome: `{summary.get('outcome')}`",
        f"- seed: `{summary.get('seed')}`",
        f"- repeats per rho: `{summary.get('repeats')}`",
        f"- total query calls: `{summary.get('total_query_count')}`",
        f"- elapsed wall time: `{summary.get('elapsed_wall_time_s', 0.0):.1f}s`",
        "",
        "## Stage 0",
    ]
    stage0 = summary.get("stage0", {})
    lines.extend(
        [
            f"- gate_pass: `{stage0.get('gate_pass')}`",
            f"- neutral_safe_2std: `{stage0.get('neutral_safe_2std')}`",
            f"- switch_clean: `{stage0.get('switch_clean')}`",
            f"- maneuver_induced_motion: `{stage0.get('maneuver_induced_motion')}`",
            f"- switched_run_differs_from_posctl: `{stage0.get('switched_run_differs_from_posctl')}`",
            f"- transition validation: `{stage0.get('transition_validation_rows')}`",
        ]
    )
    if "stage_a" in summary:
        stage_a = summary["stage_a"]
        lines.extend(
            [
                "",
                "## Stage A",
                f"- robust transition-caused violations: `{stage_a.get('robust_transition_violation_count')}`",
                f"- weak 1std candidates: `{stage_a.get('weak_1std_candidate_count')}`",
                f"- noise straddles not counted: `{stage_a.get('noise_straddle_not_counted_count')}`",
                f"- joint-random probe hits: `{stage_a.get('joint_random_probe_hit_count')}`",
                f"- distinct clusters: `{stage_a.get('distinct_cluster_count')}`",
                f"- pair rows: `{stage_a.get('pair_rows')}`",
                f"- cluster rows: `{stage_a.get('cluster_rows')}`",
            ]
        )
    if "stage_b" in summary:
        lines.extend(["", "## Stage B", f"- method summary: `{summary['stage_b'].get('method_summary_rows')}`"])
        for row in summary["stage_b"].get("methods", []):
            lines.append(
                f"- {row['method']}: queries={row['query_count']} robust={row['robust_transition_violations']} "
                f"clusters={row['distinct_clusters']} t_windows={row['t_window_coverage']}"
            )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _t_label(t_switch: float) -> str:
    return f"{float(t_switch):.2f}".replace(".", "p")


def _t_window(t_switch: float) -> str:
    lo = math.floor(float(t_switch))
    hi = lo + 1
    return f"{lo:.0f}-{hi:.0f}s"


def _safe_label(label: str, limit: int = 80) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", str(label))[:limit]


if __name__ == "__main__":
    main()
