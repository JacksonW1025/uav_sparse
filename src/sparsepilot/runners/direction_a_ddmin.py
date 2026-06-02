from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from sparsepilot.config import ExperimentConfig, load_config
from sparsepilot.groups import Group, build_groups
from sparsepilot.input_model import project_theta
from sparsepilot.query import theta_hash
from sparsepilot.runners.direction_a_probe import (
    CHANNEL_RELEVANT_SET,
    J_REPEATS,
    ROBUST_SIGMA_MULTIPLIER,
    SUPPORT_THRESHOLD,
    TARGET_PROPERTY,
    _config_for_probe,
    _distribution,
    _property_stats,
    _run_query_with_retry_count,
    _safe_label,
    _write_json,
    classify_robustness,
    support_summary,
)


CLEAN_CHANNELS = set(CHANNEL_RELEVANT_SET)
DEFAULT_PROBE_DIR = Path("runs/direction_a_px4_position_seed0_v0")
DEFAULT_RUN_DIR = Path("runs/direction_a_ddmin_px4_position_seed0_v0")
ARM_C_J5_POINTS = 80
ARM_C_INTERIOR_TRIGGERS = 18


@dataclass(frozen=True)
class StartingPoint:
    trigger_id: int
    source_rank: int
    selection_bucket: str
    eval_id: int
    stage: str
    label: str
    theta_hash: str
    theta_path: Path
    max_abs_theta: float
    support_size: int
    active_channels: str
    rho_mean: float
    rho_std: float
    theta: np.ndarray


class DdminEvaluator:
    def __init__(
        self,
        scenario_id: str,
        seed: int,
        output_dir: Path,
        groups: list[Group],
    ) -> None:
        self.scenario_id = scenario_id
        self.seed = int(seed)
        self.output_dir = Path(output_dir)
        self.groups = groups
        self.reports_dir = self.output_dir / "reports"
        self.thetas_dir = self.output_dir / "thetas"
        self.reports_dir.mkdir(parents=True, exist_ok=True)
        self.thetas_dir.mkdir(parents=True, exist_ok=True)
        self.point_rows: list[dict[str, Any]] = []
        self.query_rows: list[dict[str, Any]] = []
        self.decision_rows: list[dict[str, Any]] = []
        self.memo: dict[str, dict[str, Any]] = {}
        self.eval_counter = 0
        self.successful_query_count = 0
        self.timeout_retry_count = 0

    def eval_j5(
        self,
        theta: np.ndarray,
        scenario,
        config: ExperimentConfig,
        *,
        trigger_id: int,
        phase: str,
        label: str,
        repeats: int,
    ) -> tuple[dict[str, Any], bool]:
        projected = project_theta(np.asarray(theta, dtype=float), config)
        thash = theta_hash(projected)
        if thash in self.memo:
            return self.memo[thash], True

        eval_id = self.eval_counter
        self.eval_counter += 1
        values: dict[str, list[float]] = {prop: [] for prop in scenario.properties}
        point_start = time.monotonic()
        for repeat_idx in range(repeats):
            cache_tag = _safe_label(
                f"direction_a_ddmin_t{trigger_id:02d}_p{eval_id:04d}_{phase}_{label}_repeat{repeat_idx}"
            )
            repeat_start = time.monotonic()
            result, retry_count = _run_query_with_retry_count(
                projected,
                scenario,
                self.seed,
                "direction_a_ddmin",
                self.output_dir,
                config,
                cache_tag=cache_tag,
                use_cache=True,
            )
            repeat_elapsed = time.monotonic() - repeat_start
            self.successful_query_count += 1
            self.timeout_retry_count += retry_count
            for prop, value in result.robustness.items():
                values[prop].append(float(value))
            query_row: dict[str, Any] = {
                "eval_id": eval_id,
                "trigger_id": trigger_id,
                "phase": phase,
                "label": label,
                "repeat_idx": repeat_idx,
                "theta_hash": result.theta_hash,
                "query_id": result.query_id,
                "cache_tag": cache_tag,
                "query_retry_count": retry_count,
                "repeat_elapsed_wall_time_s": repeat_elapsed,
            }
            for prop, value in result.robustness.items():
                query_row[f"robustness_{prop}"] = float(value)
            for key, value in result.metadata.items():
                query_row[f"meta_{key}"] = value
            self.query_rows.append(query_row)

        point_elapsed = time.monotonic() - point_start
        stats = _property_stats(values)
        target = stats[TARGET_PROPERTY]
        support = support_summary(projected, self.groups)
        row: dict[str, Any] = {
            "eval_id": eval_id,
            "trigger_id": trigger_id,
            "phase": phase,
            "label": label,
            "theta_hash": thash,
            "theta_path": str(self._save_theta(eval_id, trigger_id, thash, projected)),
            "repeats": repeats,
            "point_elapsed_wall_time_s": point_elapsed,
            "max_abs_theta": float(np.max(np.abs(projected))) if projected.size else 0.0,
            "support_size_abs_gt_0p1": int(support["support_size"]),
            "active_channels_abs_gt_0p1": ",".join(support["active_channels"]),
            "active_group_ids_abs_gt_0p1": ",".join(str(group_id) for group_id in support["active_group_ids"]),
            "robustness_class": classify_robustness(target["mean"], target["std"]),
            "is_robust_violation_2sigma": bool(target["mean"] + ROBUST_SIGMA_MULTIPLIER * target["std"] < 0.0),
        }
        for prop, prop_stats in stats.items():
            for key, value in prop_stats.items():
                row[f"rho_{key}_{prop}"] = value
        self.point_rows.append(row)
        self.memo[thash] = row
        if len(self.point_rows) % 5 == 0:
            self.write_rows()
        print(
            f"ddmin_eval trigger={trigger_id} point={eval_id + 1} phase={phase} "
            f"rho_mean={target['mean']:.6f} rho_std={target['std']:.6f} "
            f"class={row['robustness_class']} support={row['support_size_abs_gt_0p1']} "
            f"channels={row['active_channels_abs_gt_0p1']} max_abs={row['max_abs_theta']:.3f}",
            flush=True,
        )
        return row, False

    def record_decision(self, row: dict[str, Any]) -> None:
        self.decision_rows.append(row)
        if len(self.decision_rows) % 20 == 0:
            self.write_rows()

    def write_rows(self) -> None:
        if self.point_rows:
            pd.DataFrame(self.point_rows).to_csv(self.reports_dir / "ddmin_point_evaluations.csv", index=False)
        if self.query_rows:
            pd.DataFrame(self.query_rows).to_csv(self.reports_dir / "ddmin_query_repeats.csv", index=False)
        if self.decision_rows:
            pd.DataFrame(self.decision_rows).to_csv(self.reports_dir / "ddmin_decisions.csv", index=False)

    def _save_theta(self, eval_id: int, trigger_id: int, thash: str, theta: np.ndarray) -> Path:
        path = self.thetas_dir / f"T{trigger_id:02d}_{eval_id:05d}_{thash}.npy"
        np.save(path, theta)
        return path


class TriggerMinimizer:
    def __init__(
        self,
        start: StartingPoint,
        evaluator: DdminEvaluator,
        scenario,
        config: ExperimentConfig,
        groups: list[Group],
        *,
        repeats: int,
        budget_j5_points: int,
        amplitude_iters: int,
        max_outer_passes: int,
    ) -> None:
        self.start = start
        self.evaluator = evaluator
        self.scenario = scenario
        self.config = config
        self.groups = groups
        self.repeats = int(repeats)
        self.budget_j5_points = int(budget_j5_points)
        self.amplitude_iters = int(amplitude_iters)
        self.max_outer_passes = int(max_outer_passes)
        self.current_theta = project_theta(start.theta, config)
        self.current_eval = self._cached_start_eval()
        self.points_used = 0
        self.new_query_repeats_used = 0
        self.memo_hits = 0
        self.step_index = 0
        self.accepted_steps = 0
        self.budget_exhausted = False

    def run(self) -> dict[str, Any]:
        start_time = time.monotonic()
        self.evaluator.memo[self.start.theta_hash] = self.current_eval
        self.evaluator.record_decision(
            self._decision_row(
                phase="start",
                action="cached_arm_b_start",
                candidate_eval=self.current_eval,
                accepted=True,
                memo_hit=True,
                details={"selection_bucket": self.start.selection_bucket},
            )
        )

        for channel in ["throttle", "yaw"]:
            if self.budget_exhausted:
                break
            ids = [group.group_id for group in self.groups if group.channel == channel]
            candidate = _zero_group_ids(self.current_theta, ids, self.config)
            self._try_candidate(candidate, phase="whole_channel", action=f"zero_{channel}", details={"group_ids": ids})

        for outer_pass in range(self.max_outer_passes):
            if self.budget_exhausted:
                break
            before_hash = theta_hash(self.current_theta)
            amplitude_reserve = min(
                self.amplitude_iters,
                max(0, self.budget_j5_points - self.points_used),
            )
            group_changed = self._ddmin_support_pass(outer_pass, reserve_j5_points=amplitude_reserve)
            if self.budget_exhausted:
                break
            amplitude_changed = self._amplitude_bisection(outer_pass)
            after_hash = theta_hash(self.current_theta)
            if not group_changed and not amplitude_changed and after_hash == before_hash:
                break

        elapsed = time.monotonic() - start_time
        final_support = support_summary(self.current_theta, self.groups)
        final_eval = self.current_eval
        clean = _is_clean(final_support)
        result = {
            "trigger_id": self.start.trigger_id,
            "source_eval_id": self.start.eval_id,
            "source_theta_hash": self.start.theta_hash,
            "selection_bucket": self.start.selection_bucket,
            "source_stage": self.start.stage,
            "source_label": self.start.label,
            "source_theta_path": str(self.start.theta_path),
            "source_max_abs_theta": self.start.max_abs_theta,
            "source_support_size_abs_gt_0p1": self.start.support_size,
            "source_active_channels_abs_gt_0p1": self.start.active_channels,
            "source_rho_mean_post_neutral_xy_velocity": self.start.rho_mean,
            "source_rho_std_post_neutral_xy_velocity": self.start.rho_std,
            "final_theta_hash": theta_hash(self.current_theta),
            "final_theta_path": str(self._save_final_theta()),
            "final_max_abs_theta": float(np.max(np.abs(self.current_theta))) if self.current_theta.size else 0.0,
            "final_support_size_abs_gt_0p1": int(final_support["support_size"]),
            "final_active_channels_abs_gt_0p1": ",".join(final_support["active_channels"]),
            "final_active_group_ids_abs_gt_0p1": ",".join(str(group_id) for group_id in final_support["active_group_ids"]),
            "final_robustness_class": final_eval["robustness_class"],
            "final_rho_mean_post_neutral_xy_velocity": float(final_eval[f"rho_mean_{TARGET_PROPERTY}"]),
            "final_rho_std_post_neutral_xy_velocity": float(final_eval[f"rho_std_{TARGET_PROPERTY}"]),
            "final_rho_margin_2sigma_post_neutral_xy_velocity": float(
                final_eval[f"rho_mean_{TARGET_PROPERTY}"] + ROBUST_SIGMA_MULTIPLIER * final_eval[f"rho_std_{TARGET_PROPERTY}"]
            ),
            "is_clean": bool(clean),
            "is_roll_pitch_only": bool(set(final_support["active_channels"]) <= CLEAN_CHANNELS),
            "is_support_le_8": bool(int(final_support["support_size"]) <= 8),
            "j5_points_used": int(self.points_used),
            "query_repeats_used": int(self.new_query_repeats_used),
            "memo_hits": int(self.memo_hits),
            "accepted_steps": int(self.accepted_steps),
            "budget_exhausted": bool(self.budget_exhausted),
            "elapsed_wall_time_s": elapsed,
        }
        print(
            f"ddmin_trigger_done trigger={self.start.trigger_id} clean={clean} "
            f"support={result['final_support_size_abs_gt_0p1']} "
            f"channels={result['final_active_channels_abs_gt_0p1']} "
            f"max_abs={result['final_max_abs_theta']:.3f} j5_points={self.points_used}",
            flush=True,
        )
        return result

    def _ddmin_support_pass(self, outer_pass: int, *, reserve_j5_points: int) -> bool:
        changed = False
        active_ids = _active_group_ids(self.current_theta, self.groups)
        if not active_ids:
            return False
        granularity = 2
        while active_ids and not self.budget_exhausted:
            if self.points_used >= self.budget_j5_points - reserve_j5_points:
                break
            granularity = min(granularity, len(active_ids))
            partitions = _split_evenly(active_ids, granularity)
            removed = False
            for subset in sorted(partitions, key=len, reverse=True):
                if not subset or self.budget_exhausted:
                    continue
                if self.points_used >= self.budget_j5_points - reserve_j5_points:
                    break
                candidate = _zero_group_ids(self.current_theta, subset, self.config)
                accepted = self._try_candidate(
                    candidate,
                    phase="support_ddmin",
                    action=f"outer{outer_pass}_remove_{len(subset):02d}_of_{len(active_ids):02d}",
                    details={"group_ids": subset, "granularity": granularity, "outer_pass": outer_pass},
                )
                if accepted:
                    changed = True
                    removed = True
                    active_ids = _active_group_ids(self.current_theta, self.groups)
                    granularity = max(2, granularity - 1)
                    break
            if removed:
                continue
            if granularity >= len(active_ids):
                break
            granularity = min(len(active_ids), granularity * 2)
        return changed

    def _amplitude_bisection(self, outer_pass: int) -> bool:
        changed = False
        base_theta = self.current_theta.copy()
        low = 0.0
        high = 1.0
        for iteration in range(self.amplitude_iters):
            if self.budget_exhausted:
                break
            mid = 0.5 * (low + high)
            candidate = project_theta(base_theta * mid, self.config)
            result = self._try_candidate(
                candidate,
                phase="amplitude_bisection",
                action=f"outer{outer_pass}_iter{iteration:02d}_scale{_scale_label(mid)}",
                details={"scale": mid, "outer_pass": outer_pass, "iteration": iteration},
            )
            if result:
                changed = True
                high = mid
            else:
                low = mid
        return changed

    def _try_candidate(
        self,
        candidate: np.ndarray,
        *,
        phase: str,
        action: str,
        details: dict[str, Any],
    ) -> bool:
        candidate = project_theta(candidate, self.config)
        candidate_hash = theta_hash(candidate)
        current_hash = theta_hash(self.current_theta)
        if candidate_hash == current_hash:
            self.evaluator.record_decision(
                self._decision_row(
                    phase=phase,
                    action=action,
                    candidate_eval=self.current_eval,
                    accepted=False,
                    memo_hit=True,
                    skipped_reason="same_as_current",
                    details=details,
                )
            )
            return False

        if candidate_hash in self.evaluator.memo:
            candidate_eval = self.evaluator.memo[candidate_hash]
            memo_hit = True
            self.memo_hits += 1
        else:
            if self.points_used >= self.budget_j5_points:
                self.budget_exhausted = True
                self.evaluator.record_decision(
                    self._decision_row(
                        phase=phase,
                        action=action,
                        candidate_eval=None,
                        candidate_theta=candidate,
                        accepted=False,
                        memo_hit=False,
                        skipped_reason="budget_exhausted",
                        details=details,
                    )
                )
                return False
            candidate_eval, memo_hit = self.evaluator.eval_j5(
                candidate,
                self.scenario,
                self.config,
                trigger_id=self.start.trigger_id,
                phase=phase,
                label=action,
                repeats=self.repeats,
            )
            self.points_used += 1
            self.new_query_repeats_used += self.repeats

        accepted = bool(candidate_eval["robustness_class"] == "robust_violation")
        if accepted:
            self.current_theta = candidate
            self.current_eval = candidate_eval
            self.accepted_steps += 1
        self.evaluator.record_decision(
            self._decision_row(
                phase=phase,
                action=action,
                candidate_eval=candidate_eval,
                accepted=accepted,
                memo_hit=memo_hit,
                details=details,
            )
        )
        return accepted

    def _decision_row(
        self,
        *,
        phase: str,
        action: str,
        candidate_eval: dict[str, Any] | None,
        accepted: bool,
        memo_hit: bool,
        details: dict[str, Any],
        candidate_theta: np.ndarray | None = None,
        skipped_reason: str = "",
    ) -> dict[str, Any]:
        self.step_index += 1
        theta = candidate_theta
        if candidate_eval is not None:
            theta_path = candidate_eval.get("theta_path")
            if theta_path and Path(theta_path).exists():
                theta = np.load(theta_path)
        support = support_summary(theta, self.groups) if theta is not None else {"support_size": math.nan, "active_channels": [], "active_group_ids": []}
        row: dict[str, Any] = {
            "trigger_id": self.start.trigger_id,
            "step_index": self.step_index,
            "phase": phase,
            "action": action,
            "accepted": bool(accepted),
            "memo_hit": bool(memo_hit),
            "skipped_reason": skipped_reason,
            "j5_points_used_after": int(self.points_used),
            "query_repeats_used_after": int(self.new_query_repeats_used),
            "candidate_theta_hash": theta_hash(theta) if theta is not None else "",
            "candidate_support_size_abs_gt_0p1": int(support["support_size"]) if not _is_nan(support["support_size"]) else math.nan,
            "candidate_active_channels_abs_gt_0p1": ",".join(support["active_channels"]),
            "candidate_active_group_ids_abs_gt_0p1": ",".join(str(group_id) for group_id in support["active_group_ids"]),
            "details_json": json.dumps(_jsonable(details), sort_keys=True),
        }
        if candidate_eval is not None:
            row.update(
                {
                    "candidate_eval_id": candidate_eval.get("eval_id", "cached_start"),
                    "candidate_robustness_class": candidate_eval["robustness_class"],
                    "candidate_max_abs_theta": float(candidate_eval["max_abs_theta"]),
                    "candidate_rho_mean_post_neutral_xy_velocity": float(candidate_eval[f"rho_mean_{TARGET_PROPERTY}"]),
                    "candidate_rho_std_post_neutral_xy_velocity": float(candidate_eval[f"rho_std_{TARGET_PROPERTY}"]),
                    "candidate_rho_margin_2sigma_post_neutral_xy_velocity": float(
                        candidate_eval[f"rho_mean_{TARGET_PROPERTY}"]
                        + ROBUST_SIGMA_MULTIPLIER * candidate_eval[f"rho_std_{TARGET_PROPERTY}"]
                    ),
                }
            )
        return row

    def _cached_start_eval(self) -> dict[str, Any]:
        return {
            "eval_id": f"arm_b_{self.start.eval_id}",
            "trigger_id": self.start.trigger_id,
            "phase": "cached_arm_b_start",
            "label": self.start.label,
            "theta_hash": self.start.theta_hash,
            "theta_path": str(self.start.theta_path),
            "repeats": J_REPEATS,
            "point_elapsed_wall_time_s": 0.0,
            "max_abs_theta": self.start.max_abs_theta,
            "support_size_abs_gt_0p1": self.start.support_size,
            "active_channels_abs_gt_0p1": self.start.active_channels,
            "robustness_class": "robust_violation",
            "is_robust_violation_2sigma": True,
            f"rho_mean_{TARGET_PROPERTY}": self.start.rho_mean,
            f"rho_std_{TARGET_PROPERTY}": self.start.rho_std,
        }

    def _save_final_theta(self) -> Path:
        path = self.evaluator.thetas_dir / f"T{self.start.trigger_id:02d}_final_{theta_hash(self.current_theta)}.npy"
        np.save(path, self.current_theta)
        return path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Direction-A necessity test: strong channel-agnostic ddmin baseline on cached Arm B robust violations."
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--probe-dir", default=str(DEFAULT_PROBE_DIR))
    parser.add_argument("--run-dir", default=str(DEFAULT_RUN_DIR))
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=1.0)
    parser.add_argument("--budget-j5-points", type=int, default=40)
    parser.add_argument("--moderate-starts", type=int, default=3)
    parser.add_argument("--max-starts", type=int, default=10)
    parser.add_argument("--amplitude-iters", type=int, default=7)
    parser.add_argument("--max-outer-passes", type=int, default=4)
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Direction-A ddmin test is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Direction-A ddmin test is frozen to seed 0")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("Direction-A ddmin test is pre-registered to J=5 repeats")
    if float(args.stick_limit) != 1.0:
        raise ValueError("Direction-A ddmin test uses the probe's fixed stick limit +/-1.0")

    run_start = time.monotonic()
    output_dir = Path(args.run_dir)
    reports_dir = output_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    base_config = load_config(args.config)
    config = _config_for_probe(base_config, output_dir, float(args.stick_limit))
    scenario = config.scenario_by_id(args.scenario)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"Frozen D=40 parameterization expected 40 groups, found {len(groups)}")
    pd.DataFrame([group.__dict__ for group in groups]).to_csv(output_dir / "groups.csv", index=False)

    preregistration = _pre_registration(args, config, groups)
    _write_json(reports_dir / "pre_registration.json", preregistration)

    starting_points = select_starting_points(
        Path(args.probe_dir),
        moderate_starts=int(args.moderate_starts),
        max_starts=int(args.max_starts),
    )
    pd.DataFrame([_starting_point_row(point) for point in starting_points]).to_csv(
        reports_dir / "starting_points.csv", index=False
    )
    evaluator = DdminEvaluator(args.scenario, int(args.seed), output_dir, groups)

    print(
        f"ddmin_start scenario={args.scenario} seed={args.seed} starts={len(starting_points)} "
        f"budget_j5_per_start={args.budget_j5_points} run_dir={output_dir}",
        flush=True,
    )

    final_rows: list[dict[str, Any]] = []
    for point in starting_points:
        minimizer = TriggerMinimizer(
            point,
            evaluator,
            scenario,
            config,
            groups,
            repeats=int(args.repeats),
            budget_j5_points=int(args.budget_j5_points),
            amplitude_iters=int(args.amplitude_iters),
            max_outer_passes=int(args.max_outer_passes),
        )
        final_rows.append(minimizer.run())
        evaluator.write_rows()
        pd.DataFrame(final_rows).to_csv(reports_dir / "minimized_triggers.csv", index=False)

    evaluator.write_rows()
    final_df = pd.DataFrame(final_rows)
    final_df.to_csv(reports_dir / "minimized_triggers.csv", index=False)
    summary = build_summary(
        final_df=final_df,
        starting_points=starting_points,
        probe_dir=Path(args.probe_dir),
        output_dir=output_dir,
        evaluator=evaluator,
        args=args,
        elapsed_wall_time_s=time.monotonic() - run_start,
    )
    _write_json(reports_dir / "direction_a_ddmin_summary.json", summary)
    _write_report(reports_dir / "direction_a_ddmin_report.md", summary)
    print(
        f"ddmin_complete clean={summary['clean_trigger_count']}/{summary['starting_trigger_count']} "
        f"successful_queries={summary['successful_query_count']} timeout_retries={summary['timeout_retry_count']} "
        f"elapsed={summary['elapsed_wall_time_s']:.1f}s report={reports_dir / 'direction_a_ddmin_report.md'}",
        flush=True,
    )


def select_starting_points(probe_dir: Path, *, moderate_starts: int, max_starts: int) -> list[StartingPoint]:
    probe_dir = Path(probe_dir)
    point_df = pd.read_csv(Path(probe_dir) / "reports" / "point_evaluations.csv")
    b_robust = point_df[(point_df["arm"] == "B") & (point_df["robustness_class"] == "robust_violation")].copy()
    interior = b_robust[b_robust["amplitude_class"] == "interior"].sort_values("eval_id").copy()
    moderate = b_robust[b_robust["amplitude_class"] == "moderate"].copy()
    moderate["rho_margin_2sigma"] = (
        moderate[f"rho_mean_{TARGET_PROPERTY}"] + ROBUST_SIGMA_MULTIPLIER * moderate[f"rho_std_{TARGET_PROPERTY}"]
    )
    moderate = moderate.sort_values(
        ["support_size_abs_gt_0p1", "rho_margin_2sigma", "eval_id"],
        ascending=[False, True, True],
    )
    selected = pd.concat([interior, moderate.head(max(0, int(moderate_starts)))], ignore_index=True)
    selected = selected.head(int(max_starts)).copy()
    starts: list[StartingPoint] = []
    for idx, row in selected.iterrows():
        theta_path = _resolve_probe_theta_path(probe_dir, Path(row["theta_path"]))
        theta = np.load(theta_path)
        starts.append(
            StartingPoint(
                trigger_id=int(idx),
                source_rank=int(idx),
                selection_bucket="arm_b_interior" if row["amplitude_class"] == "interior" else "arm_b_densest_moderate",
                eval_id=int(row["eval_id"]),
                stage=str(row["stage"]),
                label=str(row["label"]),
                theta_hash=str(row["theta_hash"]),
                theta_path=theta_path,
                max_abs_theta=float(row["max_abs_theta"]),
                support_size=int(row["support_size_abs_gt_0p1"]),
                active_channels=str(row["active_channels_abs_gt_0p1"]),
                rho_mean=float(row[f"rho_mean_{TARGET_PROPERTY}"]),
                rho_std=float(row[f"rho_std_{TARGET_PROPERTY}"]),
                theta=theta,
            )
        )
    return starts


def _resolve_probe_theta_path(probe_dir: Path, theta_path: Path) -> Path:
    if theta_path.exists():
        return theta_path
    artifact_path = probe_dir / "thetas" / theta_path.name
    if artifact_path.exists():
        return artifact_path
    raise FileNotFoundError(f"theta file not found: {theta_path}; also checked {artifact_path}")


def build_summary(
    *,
    final_df: pd.DataFrame,
    starting_points: list[StartingPoint],
    probe_dir: Path,
    output_dir: Path,
    evaluator: DdminEvaluator,
    args: argparse.Namespace,
    elapsed_wall_time_s: float,
) -> dict[str, Any]:
    arm_c = _arm_c_comparison(Path(probe_dir))
    clean_df = final_df[final_df["is_clean"]].copy()
    total_j5 = int(final_df["j5_points_used"].sum()) if not final_df.empty else 0
    clean_count = int(len(clean_df))
    ddmin_cost_per_clean = float(total_j5 / clean_count) if clean_count else math.inf
    arm_c_cost_per_interior = float(ARM_C_J5_POINTS / ARM_C_INTERIOR_TRIGGERS)
    support_values = final_df["final_support_size_abs_gt_0p1"].to_numpy(dtype=float) if not final_df.empty else np.array([])
    max_abs_values = final_df["final_max_abs_theta"].to_numpy(dtype=float) if not final_df.empty else np.array([])
    decision = _decision(final_df, ddmin_cost_per_clean, arm_c_cost_per_interior)
    return _jsonable(
        {
            "status": "complete",
            "scenario_id": args.scenario,
            "seed": int(args.seed),
            "property": TARGET_PROPERTY,
            "pre_registered_definitions": {
                "robust_violation": "rho_mean + 2*rho_std < 0 with J=5; accepted minimization steps must preserve this",
                "clean_trigger": "support |theta|>0.1 <= 8 and active channels subset of {roll,pitch}",
                "ddmin_baseline": [
                    "whole-channel throttle removal then yaw removal",
                    "standard greedy ddmin over channel-time groups, largest subsets first",
                    "global amplitude bisection toward zero",
                    "alternate support and amplitude passes within budget",
                ],
            },
            "starting_trigger_count": int(len(starting_points)),
            "starting_points": [_starting_point_row(point) for point in starting_points],
            "clean_trigger_count": clean_count,
            "clean_yield": float(clean_count / len(final_df)) if len(final_df) else 0.0,
            "roll_pitch_only_count": int(final_df["is_roll_pitch_only"].sum()) if not final_df.empty else 0,
            "support_le_8_count": int(final_df["is_support_le_8"].sum()) if not final_df.empty else 0,
            "final_support_distribution": _distribution(support_values),
            "final_max_abs_theta_distribution": _distribution(max_abs_values),
            "final_channel_distribution": _value_counts(final_df, "final_active_channels_abs_gt_0p1"),
            "j5_points_used_distribution": _distribution(final_df["j5_points_used"].to_numpy(dtype=float))
            if not final_df.empty
            else {},
            "total_ddmin_j5_points_used": total_j5,
            "ddmin_j5_points_per_clean_trigger": ddmin_cost_per_clean,
            "arm_c_comparison": arm_c,
            "arm_c_amortized_j5_points_per_interior_trigger": arm_c_cost_per_interior,
            "cost_ratio_ddmin_per_clean_vs_arm_c_per_interior": (
                float(ddmin_cost_per_clean / arm_c_cost_per_interior)
                if math.isfinite(ddmin_cost_per_clean) and arm_c_cost_per_interior > 0.0
                else math.inf
            ),
            "decision": decision,
            "minimized_triggers": final_df.to_dict(orient="records"),
            "successful_query_count": evaluator.successful_query_count,
            "timeout_retry_count": evaluator.timeout_retry_count,
            "query_attempt_count_including_timeout_retries": evaluator.successful_query_count
            + evaluator.timeout_retry_count,
            "elapsed_wall_time_s": elapsed_wall_time_s,
            "artifacts": {
                "pre_registration": str(output_dir / "reports" / "pre_registration.json"),
                "starting_points": str(output_dir / "reports" / "starting_points.csv"),
                "minimized_triggers": str(output_dir / "reports" / "minimized_triggers.csv"),
                "ddmin_point_evaluations": str(output_dir / "reports" / "ddmin_point_evaluations.csv"),
                "ddmin_query_repeats": str(output_dir / "reports" / "ddmin_query_repeats.csv"),
                "ddmin_decisions": str(output_dir / "reports" / "ddmin_decisions.csv"),
                "summary": str(output_dir / "reports" / "direction_a_ddmin_summary.json"),
                "report": str(output_dir / "reports" / "direction_a_ddmin_report.md"),
            },
            "replication_caveat": "Single seed/scenario probe only; replicate across seeds before any paper claim.",
        }
    )


def _pre_registration(args: argparse.Namespace, config: ExperimentConfig, groups: list[Group]) -> dict[str, Any]:
    return {
        "scope": {
            "scenario": args.scenario,
            "seed": int(args.seed),
            "primary_property": TARGET_PROPERTY,
            "D": len(groups),
            "starting_points": "cached Arm B robust violations from Direction-A probe",
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
        "thresholds": {
            "robust_violation": "rho_mean + 2*rho_std < 0",
            "sigma_multiplier": ROBUST_SIGMA_MULTIPLIER,
            "support_abs_threshold": SUPPORT_THRESHOLD,
            "clean_support_max": 8,
            "clean_active_channels_subset": sorted(CLEAN_CHANNELS),
        },
        "ddmin_budget": {
            "j5_points_per_start": int(args.budget_j5_points),
            "repeats_per_point": int(args.repeats),
            "amplitude_bisection_iters_per_pass": int(args.amplitude_iters),
            "support_pass_reserves_amplitude_points": True,
            "max_outer_passes": int(args.max_outer_passes),
            "moderate_starts": int(args.moderate_starts),
            "max_starts": int(args.max_starts),
        },
        "decision_rule": {
            "replaceable": "majority of starts become clean robust triggers at cost comparable to Arm C",
            "necessary_or_better": "fails cleanliness, reliability, or cost comparison",
        },
    }


def _write_report(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# Direction-A Necessity Test: Channel-Agnostic ddmin",
        "",
        f"Scope: `{summary['scenario_id']}`, seed {summary['seed']}, property `{summary['property']}`.",
        (
            f"Starts: {summary['starting_trigger_count']} cached Arm B robust violations; "
            f"clean yield: {summary['clean_trigger_count']}/{summary['starting_trigger_count']} "
            f"({summary['clean_yield']:.3f})."
        ),
        "",
        "## Decision",
        "",
        f"Headline: **{summary['decision']['headline']}**",
        "",
        summary["decision"]["rationale"],
        "",
        "## Starting Points",
        "",
        "| trigger | bucket | Arm B eval | stage | max|theta| | support | channels | rho mean | rho std | theta hash |",
        "| ---: | --- | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |",
    ]
    for row in summary["starting_points"]:
        lines.append(
            f"| {row['trigger_id']} | {row['selection_bucket']} | {row['eval_id']} | {row['stage']} | "
            f"{row['max_abs_theta']:.6f} | {row['support_size_abs_gt_0p1']} | "
            f"{row['active_channels_abs_gt_0p1']} | {row['rho_mean_post_neutral_xy_velocity']:.6f} | "
            f"{row['rho_std_post_neutral_xy_velocity']:.6f} | {row['theta_hash']} |"
        )
    lines.extend(
        [
            "",
            "## Minimized Triggers",
            "",
            "| trigger | clean | support | channels | max|theta| | rho mean | rho std | 2sigma margin | J=5 points | memo hits | budget | theta hash |",
            "| ---: | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
        ]
    )
    for row in summary["minimized_triggers"]:
        lines.append(
            f"| {row['trigger_id']} | {row['is_clean']} | {row['final_support_size_abs_gt_0p1']} | "
            f"{row['final_active_channels_abs_gt_0p1']} | {row['final_max_abs_theta']:.6f} | "
            f"{row['final_rho_mean_post_neutral_xy_velocity']:.6f} | "
            f"{row['final_rho_std_post_neutral_xy_velocity']:.6f} | "
            f"{row['final_rho_margin_2sigma_post_neutral_xy_velocity']:.6f} | "
            f"{row['j5_points_used']} | {row['memo_hits']} | "
            f"{'exhausted' if row['budget_exhausted'] else 'ok'} | {row['final_theta_hash']} |"
        )
    lines.extend(["", "## Distributions", ""])
    lines.append(f"- ddmin final support: {_dist_text(summary['final_support_distribution'])}")
    lines.append(f"- Arm C interior support: {_dist_text(summary['arm_c_comparison']['support_distribution'])}")
    lines.append(f"- ddmin final max|theta|: {_dist_text(summary['final_max_abs_theta_distribution'])}")
    lines.append(f"- Arm C interior max|theta|: {_dist_text(summary['arm_c_comparison']['max_abs_theta_distribution'])}")
    lines.append(f"- ddmin final channels: {summary['final_channel_distribution']}")
    lines.append(f"- Arm C interior channels: {summary['arm_c_comparison']['channel_distribution']}")
    lines.extend(
        [
            "",
            "## Cost",
            "",
            f"- ddmin J=5 points used: {summary['total_ddmin_j5_points_used']}",
            f"- ddmin J=5 points per clean trigger: {_fmt_float(summary['ddmin_j5_points_per_clean_trigger'])}",
            (
                f"- Arm C amortized J=5 points per interior violation: "
                f"{summary['arm_c_amortized_j5_points_per_interior_trigger']:.3f} "
                f"({ARM_C_J5_POINTS}/{ARM_C_INTERIOR_TRIGGERS})"
            ),
            f"- cost ratio ddmin/Arm C: {_fmt_float(summary['cost_ratio_ddmin_per_clean_vs_arm_c_per_interior'])}",
            "",
            "## Robustness",
            "",
            "Every accepted ddmin step and every reported minimized trigger satisfies the fixed J=5 two-sigma robust-violation gate.",
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
            f"Successful PX4 queries: {summary['successful_query_count']}.",
            f"Timeout retries: {summary['timeout_retry_count']}.",
            f"Query attempts including timeout retries: {summary['query_attempt_count_including_timeout_retries']}.",
            f"Elapsed wall time: {summary['elapsed_wall_time_s']:.1f}s.",
            "",
            summary["replication_caveat"],
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _arm_c_comparison(probe_dir: Path) -> dict[str, Any]:
    interior = pd.read_csv(probe_dir / "reports" / "interior_violations.csv")
    arm_c = interior[interior["arm"] == "C"].copy()
    return {
        "interior_trigger_count": int(len(arm_c)),
        "support_distribution": _distribution(arm_c["support_size_abs_gt_0p1"].to_numpy(dtype=float)),
        "max_abs_theta_distribution": _distribution(arm_c["max_abs_theta"].to_numpy(dtype=float)),
        "channel_distribution": _value_counts(arm_c, "active_channels_abs_gt_0p1"),
    }


def _decision(final_df: pd.DataFrame, ddmin_cost_per_clean: float, arm_c_cost_per_interior: float) -> dict[str, Any]:
    if final_df.empty:
        return {"headline": "channel-direction necessary / clearly better", "rationale": "No starting triggers were minimized."}
    clean_count = int(final_df["is_clean"].sum())
    majority_clean = clean_count > len(final_df) / 2.0
    median_support = float(final_df["final_support_size_abs_gt_0p1"].median())
    roll_pitch_fraction = float(final_df["is_roll_pitch_only"].mean())
    cost_ratio = (
        float(ddmin_cost_per_clean / arm_c_cost_per_interior)
        if math.isfinite(ddmin_cost_per_clean) and arm_c_cost_per_interior > 0.0
        else math.inf
    )
    comparable_cost = bool(cost_ratio <= 2.0)
    if majority_clean and comparable_cost:
        headline = "channel-direction replaceable in this single-seed probe"
        rationale = (
            f"ddmin produced clean triggers for {clean_count}/{len(final_df)} starts and its cost ratio "
            f"to Arm C was {cost_ratio:.2f}, within this harness's fixed <=2x comparable-cost flag."
        )
    else:
        headline = "channel-direction necessary / clearly better in this single-seed probe"
        failures = []
        if median_support > 8:
            failures.append(f"median final support stayed above 8 ({median_support:.1f})")
        if roll_pitch_fraction < 0.5:
            failures.append(f"roll/pitch-only fraction was below half ({roll_pitch_fraction:.3f})")
        if not majority_clean:
            failures.append(f"clean yield was below majority ({clean_count}/{len(final_df)})")
        if not comparable_cost:
            failures.append(f"cost ratio was much larger than Arm C ({_fmt_float(cost_ratio)})")
        rationale = "; ".join(failures) if failures else "ddmin did not satisfy the full pre-registered replaceability condition."
    return {
        "headline": headline,
        "rationale": rationale,
        "majority_clean": majority_clean,
        "median_support": median_support,
        "roll_pitch_only_fraction": roll_pitch_fraction,
        "cost_ratio": cost_ratio,
        "comparable_cost_flag_threshold_ratio": 2.0,
    }


def _starting_point_row(point: StartingPoint) -> dict[str, Any]:
    return {
        "trigger_id": point.trigger_id,
        "source_rank": point.source_rank,
        "selection_bucket": point.selection_bucket,
        "eval_id": point.eval_id,
        "stage": point.stage,
        "label": point.label,
        "theta_hash": point.theta_hash,
        "theta_path": str(point.theta_path),
        "max_abs_theta": point.max_abs_theta,
        "support_size_abs_gt_0p1": point.support_size,
        "active_channels_abs_gt_0p1": point.active_channels,
        "rho_mean_post_neutral_xy_velocity": point.rho_mean,
        "rho_std_post_neutral_xy_velocity": point.rho_std,
    }


def _active_group_ids(theta: np.ndarray, groups: list[Group]) -> list[int]:
    return [group.group_id for group in groups if abs(float(theta[group.group_id])) > SUPPORT_THRESHOLD]


def _zero_group_ids(theta: np.ndarray, group_ids: list[int], config: ExperimentConfig) -> np.ndarray:
    candidate = np.asarray(theta, dtype=float).copy()
    if group_ids:
        candidate[np.asarray(group_ids, dtype=int)] = 0.0
    return project_theta(candidate, config)


def _split_evenly(items: list[int], parts: int) -> list[list[int]]:
    if parts <= 1:
        return [list(items)]
    return [chunk.astype(int).tolist() for chunk in np.array_split(np.asarray(items, dtype=int), parts) if len(chunk)]


def _is_clean(support: dict[str, Any]) -> bool:
    return int(support["support_size"]) <= 8 and set(support["active_channels"]) <= CLEAN_CHANNELS


def _value_counts(df: pd.DataFrame, column: str) -> dict[str, int]:
    if df.empty or column not in df:
        return {}
    return {str(key): int(value) for key, value in df[column].fillna("").value_counts().sort_index().items()}


def _dist_text(dist: dict[str, float]) -> str:
    if not dist:
        return "none"
    return (
        f"min={dist['min']:.3f}, p25={dist['p25']:.3f}, median={dist['median']:.3f}, "
        f"p75={dist['p75']:.3f}, max={dist['max']:.3f}"
    )


def _fmt_float(value: float) -> str:
    return "inf" if not math.isfinite(float(value)) else f"{float(value):.3f}"


def _scale_label(value: float) -> str:
    return f"{float(value):.4f}".replace(".", "p").replace("-", "m")


def _is_nan(value: Any) -> bool:
    try:
        return bool(math.isnan(float(value)))
    except (TypeError, ValueError):
        return False


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
    if isinstance(value, (np.ndarray,)):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    return value


if __name__ == "__main__":
    main()
