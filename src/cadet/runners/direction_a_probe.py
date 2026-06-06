from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path
from itertools import product
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
from cadet.query import QueryResult, read_parsed_log, run_query, theta_hash
from cadet.runners.route1_h2_campaign import _sample_uniform_feasible
from cadet.violation_search import grid_to_theta, window_count


TARGET_PROPERTY = "post_neutral_xy_velocity"
REPORT_PROPERTIES = ["post_neutral_xy_velocity", "post_neutral_xy_drift", "post_neutral_alt_drift"]
J_REPEATS = 5
INTERIOR_MAX_ABS = 0.5
SATURATED_MIN_ABS = 0.9
SUPPORT_THRESHOLD = 0.1
ROBUST_SIGMA_MULTIPLIER = 2.0
DEFAULT_RNG_SEED = 20260530


def derive_A_phi(property_name: str) -> list[str]:
    """Return the control-allocation predicted active channel set for a property."""
    # Multirotor manual-control allocation gives roll/pitch direct authority over
    # lateral acceleration, so post-neutral xy velocity is predicted to live on
    # {roll,pitch}. Vertical thrust is allocated directly through collective
    # throttle; roll/pitch affect altitude only through a second-order tilt/cos
    # loss that POSCTL tends to compensate, so alt_drift predicts {throttle}.
    # PX4 manual vertical control maps collective throttle to climb/descent
    # velocity through MPC_Z_VEL_MAX_UP/DN, so climb-rate residuals predict
    # {throttle}. PX4 manual yaw maps yaw stick to yawspeed through
    # MPC_MAN_Y_MAX, so yaw-rate residuals predict {yaw}.
    if property_name == "post_neutral_xy_velocity":
        return ["roll", "pitch"]
    if property_name == "post_neutral_alt_drift":
        return ["throttle"]
    if property_name == "post_neutral_climb_rate":
        return ["throttle"]
    if property_name == "post_neutral_yaw_rate":
        return ["yaw"]
    raise ValueError(f"No control-allocation A_phi rule registered for property: {property_name}")


@dataclass(frozen=True)
class EnvelopeSpec:
    index: int
    angle_rad: float
    amplitude: float
    onset_window: int
    duration_windows: int

    @property
    def label(self) -> str:
        deg = int(round(math.degrees(self.angle_rad))) % 360
        return f"env{self.index:04d}_deg{deg:03d}_w{self.onset_window:02d}_d{self.duration_windows:02d}"


@dataclass(frozen=True)
class DirectedEnvelopeSpec:
    index: int
    channels: tuple[str, ...]
    signs: tuple[int, ...]
    amplitude: float
    onset_window: int
    duration_windows: int

    @property
    def label(self) -> str:
        sign_text = "".join("p" if sign > 0 else "m" for sign in self.signs)
        channel_text = "-".join(self.channels)
        return f"swp{self.index:04d}_{channel_text}_{sign_text}_w{self.onset_window:02d}_d{self.duration_windows:02d}"

    @property
    def signature(self) -> str:
        return envelope_signature(self.channels, self.signs, self.onset_window, self.duration_windows)


class DuplicatePointError(RuntimeError):
    pass


class DirectionAProbeEvaluator:
    def __init__(
        self,
        scenario_id: str,
        seed: int,
        output_dir: Path,
        groups: list[Group],
        target_property: str = TARGET_PROPERTY,
    ):
        self.scenario_id = scenario_id
        self.seed = int(seed)
        self.output_dir = Path(output_dir)
        self.groups = groups
        self.target_property = str(target_property)
        self.reports_dir = self.output_dir / "reports"
        self.thetas_dir = self.output_dir / "thetas"
        self.reports_dir.mkdir(parents=True, exist_ok=True)
        self.thetas_dir.mkdir(parents=True, exist_ok=True)

        self.point_rows: list[dict[str, Any]] = []
        self.query_rows: list[dict[str, Any]] = []
        self.seen_by_arm: dict[str, set[str]] = {"A": set(), "B": set(), "C": set()}
        self.gate_rejects_by_arm: dict[str, int] = {"A": 0, "B": 0, "C": 0}
        self.eval_counter = 0
        self.successful_query_count = 0
        self.timeout_retry_count = 0
        self.point_rows_path = self.reports_dir / "point_evaluations.csv"
        self.query_rows_path = self.reports_dir / "query_repeats.csv"

    def arm_eval_count(self, arm: str) -> int:
        return len([row for row in self.point_rows if row["arm"] == arm])

    def can_eval(self, arm: str, theta: np.ndarray, config: ExperimentConfig) -> bool:
        thash = theta_hash(project_theta(theta, config))
        return thash not in self.seen_by_arm[arm]

    def eval_j5(
        self,
        theta: np.ndarray,
        scenario,
        config: ExperimentConfig,
        *,
        arm: str,
        stage: str,
        label: str,
        repeats: int,
        gate_candidate: bool = True,
        distinct_signature: str = "",
    ) -> dict[str, Any]:
        projected = project_theta(np.asarray(theta, dtype=float), config)
        thash = theta_hash(projected)
        if thash in self.seen_by_arm[arm]:
            raise DuplicatePointError(f"duplicate point in arm {arm}: {thash}")
        self.seen_by_arm[arm].add(thash)

        eval_id = self.eval_counter
        point_index = self.arm_eval_count(arm)
        self.eval_counter += 1
        values: dict[str, list[float]] = {prop: [] for prop in scenario.properties}
        residual_repeat_metrics: list[dict[str, Any]] = []
        point_start = time.monotonic()
        for repeat_idx in range(repeats):
            cache_tag = _safe_label(
                f"direction_a_{arm}_{stage}_{eval_id:05d}_{label}_repeat{repeat_idx}"
            )
            repeat_start = time.monotonic()
            result, retry_count = _run_query_with_retry_count(
                projected,
                scenario,
                self.seed,
                "direction_a_probe",
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
            residual_metrics: dict[str, Any] = {}
            if is_residual_rate_property(self.target_property):
                parsed_log = read_parsed_log(result.parsed_log_path)
                residual_metrics = compute_residual_rate_metrics(parsed_log, self.target_property, config)
                residual_repeat_metrics.append(residual_metrics)
            row: dict[str, Any] = {
                "eval_id": eval_id,
                "point_index": point_index,
                "arm": arm,
                "stage": stage,
                "label": label,
                "repeat_idx": repeat_idx,
                "theta_hash": result.theta_hash,
                "query_id": result.query_id,
                "cache_tag": cache_tag,
                "query_retry_count": retry_count,
                "repeat_elapsed_wall_time_s": repeat_elapsed,
            }
            for prop, value in result.robustness.items():
                row[f"robustness_{prop}"] = float(value)
            for key, value in residual_metrics.items():
                if key in {"property", "unit"}:
                    row[f"residual_rate_{key}"] = value
                else:
                    row[f"residual_rate_{key}_{self.target_property}"] = value
            for key, value in result.metadata.items():
                row[f"meta_{key}"] = value
            self.query_rows.append(row)
        point_elapsed = time.monotonic() - point_start

        stats = _property_stats(values)
        target = stats[self.target_property]
        robustness_class = classify_robustness(target["mean"], target["std"])
        rejected_by_gate = bool(gate_candidate and target["mean"] < 0.0 and robustness_class != "robust_violation")
        if rejected_by_gate:
            self.gate_rejects_by_arm[arm] += 1
        support = support_summary(projected, self.groups)
        max_abs_theta = float(np.max(np.abs(projected))) if projected.size else 0.0
        theta_path = self.thetas_dir / f"{arm}_{eval_id:05d}_{thash}.npy"
        np.save(theta_path, projected)

        point_row: dict[str, Any] = {
            "eval_id": eval_id,
            "point_index": point_index,
            "arm": arm,
            "stage": stage,
            "label": label,
            "theta_hash": thash,
            "theta_path": str(theta_path),
            "repeats": repeats,
            "point_elapsed_wall_time_s": point_elapsed,
            "max_abs_theta": max_abs_theta,
            "amplitude_class": classify_amplitude(max_abs_theta),
            "support_size_abs_gt_0p1": support["support_size"],
            "active_channels_abs_gt_0p1": ",".join(support["active_channels"]),
            "robustness_class": robustness_class,
            "negative_mean_rejected_by_2sigma_gate": rejected_by_gate,
            "distinct_signature": distinct_signature,
        }
        point_row.update(_signature_columns(distinct_signature))
        if is_residual_rate_property(self.target_property):
            point_row.update(
                _residual_rate_point_columns(
                    summarize_residual_rate_repeats(
                        residual_repeat_metrics,
                        sigma_multiplier=ROBUST_SIGMA_MULTIPLIER,
                    ),
                    self.target_property,
                )
            )
        for prop, prop_stats in stats.items():
            for key, value in prop_stats.items():
                point_row[f"rho_{key}_{prop}"] = value
        self.point_rows.append(point_row)
        if len(self.point_rows) % 5 == 0:
            self.write_rows()
        print(
            f"direction_a_eval arm={arm} point={point_index + 1} stage={stage} "
            f"rho_mean={target['mean']:.6f} rho_std={target['std']:.6f} "
            f"class={robustness_class} max_abs={max_abs_theta:.3f}",
            flush=True,
        )
        return point_row

    def write_rows(self) -> None:
        if self.point_rows:
            pd.DataFrame(self.point_rows).to_csv(self.point_rows_path, index=False)
        if self.query_rows:
            pd.DataFrame(self.query_rows).to_csv(self.query_rows_path, index=False)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Direction-A discriminating probe: matched-budget Arm A/B/C search for robust, "
            "non-saturated px4_position seed-0 xy-velocity violations."
        )
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/direction_a_px4_position_seed0_v0")
    parser.add_argument("--property", default=TARGET_PROPERTY)
    parser.add_argument(
        "--active-channels",
        default=None,
        help="Optional comma-separated A_phi override, used only for pre-registered narrowed continuations.",
    )
    parser.add_argument("--rng-seed", type=int, default=DEFAULT_RNG_SEED)
    parser.add_argument("--points-per-arm", type=int, default=80)
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=1.0)
    parser.add_argument("--bisection-iters", type=int, default=7)
    parser.add_argument(
        "--allow-nonzero-seed",
        action="store_true",
        help="Explicitly unlock nonzero seed replication while keeping all pre-registered thresholds fixed.",
    )
    args = parser.parse_args()

    if args.scenario != "px4_position":
        raise ValueError("Direction-A probe is frozen to px4_position")
    nonzero_seed_unlocked = bool(int(args.seed) != 0 and args.allow_nonzero_seed)
    if int(args.seed) != 0 and not args.allow_nonzero_seed:
        raise ValueError("Direction-A probe is frozen to seed 0 unless --allow-nonzero-seed is passed")
    if int(args.seed) != 0 and int(args.rng_seed) == DEFAULT_RNG_SEED:
        raise ValueError("Nonzero seed replication must pass a seed-specific --rng-seed")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("Direction-A probe is pre-registered to J=5 repeats")
    if int(args.points_per_arm) <= 0:
        raise ValueError("--points-per-arm must be positive")
    if float(args.stick_limit) <= SATURATED_MIN_ABS:
        raise ValueError("stick limit must exceed 0.9 so the pre-registered saturated class is reachable")
    target_property = str(args.property)
    active_channels = _parse_channels_arg(args.active_channels) if args.active_channels else derive_A_phi(target_property)

    run_start = time.monotonic()
    output_dir = Path(args.run_dir)
    reports_dir = output_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    base_config = load_config(args.config)
    config = _config_for_probe(base_config, output_dir, float(args.stick_limit))
    scenario = config.scenario_by_id(args.scenario)
    if target_property not in scenario.properties:
        raise ValueError(f"{target_property} must be enabled for {args.scenario}")
    for prop in REPORT_PROPERTIES:
        if prop not in scenario.properties:
            raise ValueError(f"{prop} must be enabled for {args.scenario}")

    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"Frozen D=40 parameterization expected 40 groups, found {len(groups)}")
    pd.DataFrame([group.__dict__ for group in groups]).to_csv(output_dir / "groups.csv", index=False)

    preregistration = _pre_registration(args, config, groups, target_property, active_channels)
    _write_json(reports_dir / "pre_registration.json", preregistration)

    rng = np.random.default_rng(args.rng_seed)
    evaluator = DirectionAProbeEvaluator(args.scenario, args.seed, output_dir, groups, target_property=target_property)
    arm_details: dict[str, dict[str, Any]] = {}
    print(
        f"direction_a_start scenario={args.scenario} seed={args.seed} "
        f"property={target_property} A_phi={','.join(active_channels)} "
        f"rng_seed={args.rng_seed} N={args.points_per_arm} J={args.repeats} "
        f"nonzero_seed_unlocked={nonzero_seed_unlocked} "
        f"thresholds_frozen_from_seed0={nonzero_seed_unlocked} run_dir={output_dir}",
        flush=True,
    )

    arm_details["A"] = run_arm_a(evaluator, scenario, config, groups, rng, int(args.points_per_arm), int(args.repeats))
    arm_details["B"] = run_arm_b(
        evaluator,
        scenario,
        config,
        groups,
        rng,
        int(args.points_per_arm),
        int(args.repeats),
        int(args.bisection_iters),
    )
    arm_details["C"] = run_arm_c(
        evaluator,
        scenario,
        config,
        groups,
        rng,
        int(args.points_per_arm),
        int(args.repeats),
        int(args.bisection_iters),
        float(args.stick_limit),
        target_property,
        active_channels,
    )

    evaluator.write_rows()
    point_df = pd.DataFrame(evaluator.point_rows)
    summary = build_summary(
        point_df=point_df,
        evaluator=evaluator,
        scenario_id=args.scenario,
        seed=int(args.seed),
        output_dir=output_dir,
        preregistration=preregistration,
        arm_details=arm_details,
        target_property=target_property,
        active_channels=active_channels,
        elapsed_wall_time_s=time.monotonic() - run_start,
        nonzero_seed_unlocked=nonzero_seed_unlocked,
    )
    _write_json(reports_dir / "direction_a_summary.json", summary)
    _write_report(reports_dir / "direction_a_report.md", summary)
    print(
        f"direction_a_complete successful_queries={summary['successful_query_count']} "
        f"timeout_retries={summary['timeout_retry_count']} elapsed={summary['elapsed_wall_time_s']:.1f}s "
        f"report={reports_dir / 'direction_a_report.md'}",
        flush=True,
    )


def run_arm_a(
    evaluator: DirectionAProbeEvaluator,
    scenario,
    config: ExperimentConfig,
    groups: list[Group],
    rng: np.random.Generator,
    points_per_arm: int,
    repeats: int,
) -> dict[str, Any]:
    attempts = 0
    while evaluator.arm_eval_count("A") < points_per_arm:
        attempts += 1
        theta = _sample_uniform_feasible(config, groups, rng)
        if not evaluator.can_eval("A", theta, config):
            continue
        evaluator.eval_j5(
            theta,
            scenario,
            config,
            arm="A",
            stage="uniform_random",
            label=f"sample{attempts:04d}",
            repeats=repeats,
        )
    return {"uniform_draw_attempts": attempts}


def run_arm_b(
    evaluator: DirectionAProbeEvaluator,
    scenario,
    config: ExperimentConfig,
    groups: list[Group],
    rng: np.random.Generator,
    points_per_arm: int,
    repeats: int,
    bisection_iters: int,
) -> dict[str, Any]:
    details: dict[str, Any] = {
        "random_draw_attempts": 0,
        "robust_unsafe_endpoints": 0,
        "scale_brackets_started": 0,
        "zero_anchor_class": "",
    }
    zero = zero_theta(groups)
    zero_eval = evaluator.eval_j5(
        zero,
        scenario,
        config,
        arm="B",
        stage="zero_anchor",
        label="zero",
        repeats=repeats,
        gate_candidate=False,
    )
    details["zero_anchor_class"] = zero_eval["robustness_class"]

    while evaluator.arm_eval_count("B") < points_per_arm:
        details["random_draw_attempts"] += 1
        theta = _sample_uniform_feasible(config, groups, rng)
        if not evaluator.can_eval("B", theta, config):
            continue
        row = evaluator.eval_j5(
            theta,
            scenario,
            config,
            arm="B",
            stage="random_endpoint",
            label=f"rand{details['random_draw_attempts']:04d}",
            repeats=repeats,
        )
        if zero_eval["robustness_class"] == "robust_safe" and row["robustness_class"] == "robust_violation":
            details["robust_unsafe_endpoints"] += 1
            details["scale_brackets_started"] += 1
            _run_scale_bisection(
                evaluator,
                scenario,
                config,
                arm="B",
                base_theta=np.asarray(theta, dtype=float),
                label=f"rand{details['random_draw_attempts']:04d}",
                points_per_arm=points_per_arm,
                repeats=repeats,
                bisection_iters=bisection_iters,
                target_property=evaluator.target_property,
            )
    return details


def run_arm_c(
    evaluator: DirectionAProbeEvaluator,
    scenario,
    config: ExperimentConfig,
    groups: list[Group],
    rng: np.random.Generator,
    points_per_arm: int,
    repeats: int,
    bisection_iters: int,
    stick_limit: float,
    target_property: str,
    active_channels: list[str],
) -> dict[str, Any]:
    details: dict[str, Any] = {
        "envelope_candidates_attempted": 0,
        "robust_unsafe_endpoints": 0,
        "amplitude_brackets_started": 0,
        "zero_anchor_class": "",
        "channel_relevant_set": list(active_channels),
        "channel_relevant_source": "derive_A_phi" if list(active_channels) == derive_A_phi(target_property) else "h1_narrowed_override",
    }
    zero = zero_theta(groups)
    zero_eval = evaluator.eval_j5(
        zero,
        scenario,
        config,
        arm="C",
        stage="zero_anchor",
        label="zero",
        repeats=repeats,
        gate_candidate=False,
    )
    details["zero_anchor_class"] = zero_eval["robustness_class"]

    if target_property != "post_neutral_xy_velocity":
        details["sweep_design"] = "duration d=1..10, start-window sweep, per-envelope amplitude bisection"
        details["sweep_order"] = "round-robin over start rank then d=1..10 to expose all durations within N=80"
        specs = _duration_sweep_envelope_specs(config, active_channels, stick_limit)
        spec_index = 0
        while evaluator.arm_eval_count("C") < points_per_arm and spec_index < len(specs):
            spec = specs[spec_index]
            spec_index += 1
            details["envelope_candidates_attempted"] += 1
            theta = directed_envelope_theta(spec, config, groups)
            if not evaluator.can_eval("C", theta, config):
                continue
            row = evaluator.eval_j5(
                theta,
                scenario,
                config,
                arm="C",
                stage="duration_sweep_endpoint",
                label=spec.label,
                repeats=repeats,
                distinct_signature=spec.signature,
            )
            if zero_eval["robustness_class"] == "robust_safe" and row["robustness_class"] == "robust_violation":
                details["robust_unsafe_endpoints"] += 1
                details["amplitude_brackets_started"] += 1
                _run_directed_envelope_bisection(
                    evaluator,
                    scenario,
                    config,
                    groups,
                    spec,
                    points_per_arm=points_per_arm,
                    repeats=repeats,
                    bisection_iters=bisection_iters,
                )
        return details

    spec_index = 0
    initial_specs = _initial_envelope_specs(config, rng, stick_limit)
    while evaluator.arm_eval_count("C") < points_per_arm:
        spec = (
            initial_specs[spec_index]
            if spec_index < len(initial_specs)
            else _random_envelope_spec(config, rng, stick_limit, spec_index)
        )
        spec_index += 1
        details["envelope_candidates_attempted"] += 1
        theta = envelope_theta(spec, config, groups)
        if not evaluator.can_eval("C", theta, config):
            continue
        row = evaluator.eval_j5(
            theta,
            scenario,
            config,
            arm="C",
            stage="envelope_endpoint",
            label=spec.label,
            repeats=repeats,
        )
        if zero_eval["robustness_class"] == "robust_safe" and row["robustness_class"] == "robust_violation":
            details["robust_unsafe_endpoints"] += 1
            details["amplitude_brackets_started"] += 1
            _run_envelope_bisection(
                evaluator,
                scenario,
                config,
                groups,
                spec,
                points_per_arm=points_per_arm,
                repeats=repeats,
                bisection_iters=bisection_iters,
            )
    return details


def _run_scale_bisection(
    evaluator: DirectionAProbeEvaluator,
    scenario,
    config: ExperimentConfig,
    *,
    arm: str,
    base_theta: np.ndarray,
    label: str,
    points_per_arm: int,
    repeats: int,
    bisection_iters: int,
    target_property: str,
) -> None:
    low_alpha = 0.0
    high_alpha = 1.0
    for iteration in range(bisection_iters):
        if evaluator.arm_eval_count(arm) >= points_per_arm:
            return
        mid_alpha = 0.5 * (low_alpha + high_alpha)
        theta = project_theta(base_theta * mid_alpha, config)
        if not evaluator.can_eval(arm, theta, config):
            return
        row = evaluator.eval_j5(
            theta,
            scenario,
            config,
            arm=arm,
            stage="scale_bisection",
            label=f"{label}_iter{iteration:02d}_a{_scale_label(mid_alpha)}",
            repeats=repeats,
        )
        low_alpha, high_alpha = _update_bracket_from_row(row, low_alpha, high_alpha, mid_alpha, target_property)


def _run_envelope_bisection(
    evaluator: DirectionAProbeEvaluator,
    scenario,
    config: ExperimentConfig,
    groups: list[Group],
    spec: EnvelopeSpec,
    *,
    points_per_arm: int,
    repeats: int,
    bisection_iters: int,
) -> None:
    low_amp = 0.0
    high_amp = float(spec.amplitude)
    for iteration in range(bisection_iters):
        if evaluator.arm_eval_count("C") >= points_per_arm:
            return
        mid_amp = 0.5 * (low_amp + high_amp)
        mid_spec = EnvelopeSpec(spec.index, spec.angle_rad, mid_amp, spec.onset_window, spec.duration_windows)
        theta = envelope_theta(mid_spec, config, groups)
        if not evaluator.can_eval("C", theta, config):
            return
        row = evaluator.eval_j5(
            theta,
            scenario,
            config,
            arm="C",
            stage="amplitude_bisection",
            label=f"{spec.label}_iter{iteration:02d}_a{_scale_label(mid_amp)}",
            repeats=repeats,
        )
        low_amp, high_amp = _update_bracket_from_row(row, low_amp, high_amp, mid_amp, evaluator.target_property)


def _run_directed_envelope_bisection(
    evaluator: DirectionAProbeEvaluator,
    scenario,
    config: ExperimentConfig,
    groups: list[Group],
    spec: DirectedEnvelopeSpec,
    *,
    points_per_arm: int,
    repeats: int,
    bisection_iters: int,
) -> None:
    low_amp = 0.0
    high_amp = float(spec.amplitude)
    for iteration in range(bisection_iters):
        if evaluator.arm_eval_count("C") >= points_per_arm:
            return
        mid_amp = 0.5 * (low_amp + high_amp)
        mid_spec = DirectedEnvelopeSpec(
            spec.index,
            spec.channels,
            spec.signs,
            mid_amp,
            spec.onset_window,
            spec.duration_windows,
        )
        theta = directed_envelope_theta(mid_spec, config, groups)
        if not evaluator.can_eval("C", theta, config):
            return
        row = evaluator.eval_j5(
            theta,
            scenario,
            config,
            arm="C",
            stage="amplitude_bisection",
            label=f"{spec.label}_iter{iteration:02d}_a{_scale_label(mid_amp)}",
            repeats=repeats,
            distinct_signature=spec.signature,
        )
        low_amp, high_amp = _update_bracket_from_row(row, low_amp, high_amp, mid_amp, evaluator.target_property)


def _update_bracket_from_row(
    row: dict[str, Any],
    low: float,
    high: float,
    mid: float,
    target_property: str = TARGET_PROPERTY,
) -> tuple[float, float]:
    if row["robustness_class"] == "robust_violation":
        return low, mid
    if row["robustness_class"] == "robust_safe":
        return mid, high
    mean = float(row[f"rho_mean_{target_property}"])
    if mean < 0.0:
        return low, mid
    return mid, high


def classify_robustness(mean: float, std: float) -> str:
    if float(mean) + ROBUST_SIGMA_MULTIPLIER * float(std) < 0.0:
        return "robust_violation"
    if float(mean) - ROBUST_SIGMA_MULTIPLIER * float(std) > 0.0:
        return "robust_safe"
    return "noise_band"


def classify_amplitude(max_abs_theta: float) -> str:
    value = float(max_abs_theta)
    if value > SATURATED_MIN_ABS:
        return "saturated"
    if value <= INTERIOR_MAX_ABS:
        return "interior"
    return "moderate"


def support_summary(theta: np.ndarray, groups: list[Group], threshold: float = SUPPORT_THRESHOLD) -> dict[str, Any]:
    theta = np.asarray(theta, dtype=float)
    active_groups = [group for group in groups if abs(float(theta[group.group_id])) > threshold]
    return {
        "support_size": len(active_groups),
        "active_channels": sorted({group.channel for group in active_groups}),
        "active_group_ids": [group.group_id for group in active_groups],
    }


def envelope_theta(spec: EnvelopeSpec, config: ExperimentConfig, groups: list[Group]) -> np.ndarray:
    channels = list(config.input["channels"])
    n_windows = window_count(config)
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    roll_weight, pitch_weight = _linf_unit_direction(spec.angle_rad)
    start = max(0, int(spec.onset_window))
    stop = min(n_windows, start + max(1, int(spec.duration_windows)))
    if "roll" in channels:
        grid[start:stop, channels.index("roll")] = float(spec.amplitude) * roll_weight
    if "pitch" in channels:
        grid[start:stop, channels.index("pitch")] = float(spec.amplitude) * pitch_weight
    return project_theta(grid_to_theta(grid, config, groups), config)


def directed_envelope_theta(spec: DirectedEnvelopeSpec, config: ExperimentConfig, groups: list[Group]) -> np.ndarray:
    channels = list(config.input["channels"])
    n_windows = window_count(config)
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    start = max(0, int(spec.onset_window))
    stop = min(n_windows, start + max(1, int(spec.duration_windows)))
    for channel, sign in zip(spec.channels, spec.signs):
        if channel not in channels:
            raise ValueError(f"Directed envelope channel is absent from config: {channel}")
        grid[start:stop, channels.index(channel)] = float(spec.amplitude) * float(sign)
    return project_theta(grid_to_theta(grid, config, groups), config)


def envelope_signature(
    channels: tuple[str, ...] | list[str],
    signs: tuple[int, ...] | list[int],
    onset_window: int,
    duration_windows: int,
) -> str:
    ordered = sorted(zip(channels, signs), key=lambda item: item[0])
    channel_text = ",".join(channel for channel, _ in ordered)
    sign_text = "|".join(f"{channel}:{'+' if int(sign) > 0 else '-'}" for channel, sign in ordered)
    start = int(onset_window)
    stop = start + max(1, int(duration_windows)) - 1
    return f"channels={channel_text};time=w{start:02d}-w{stop:02d};signs={sign_text}"


def _signature_columns(signature: str) -> dict[str, str]:
    if not signature:
        return {
            "signature_active_channels": "",
            "signature_window_band": "",
            "signature_channel_signs": "",
        }
    pieces = dict(piece.split("=", 1) for piece in signature.split(";") if "=" in piece)
    return {
        "signature_active_channels": pieces.get("channels", ""),
        "signature_window_band": pieces.get("time", ""),
        "signature_channel_signs": pieces.get("signs", ""),
    }


def _residual_rate_point_columns(summary: dict[str, Any], target_property: str) -> dict[str, Any]:
    if not summary:
        return {}
    columns: dict[str, Any] = {
        "tier1_robustness_class": summary["tier1_robustness_class"],
        "tier2_robustness_class": summary["tier2_robustness_class"],
        "tier2_nondecay_robust": summary["tier2_nondecay_robust"],
        "tier2_nondecay_ratio_robust": summary["tier2_nondecay_ratio_robust"],
        "tier2_nondecay_slope_robust": summary["tier2_nondecay_slope_robust"],
        "tier2_nondecay_ratio": summary["tier2_nondecay_ratio"],
        "residual_rate_unit": summary["residual_rate_unit"],
    }
    metric_keys = [
        "threshold",
        "tail_start_peak_abs_rate",
        "terminal_peak_abs_rate",
        "rho_tier1",
        "nondecay_ratio_margin",
        "nondecay_slope_margin",
    ]
    for key in metric_keys:
        for stat in ["mean", "std", "min", "max"]:
            summary_key = f"{key}_{stat}"
            columns[f"{summary_key}_{target_property}"] = summary.get(summary_key)
    return columns


def _duration_sweep_envelope_specs(
    config: ExperimentConfig,
    active_channels: list[str],
    stick_limit: float,
) -> list[DirectedEnvelopeSpec]:
    n_windows = window_count(config)
    channels = tuple(active_channels)
    specs: list[DirectedEnvelopeSpec] = []
    for start_rank in range(n_windows):
        for duration in range(1, n_windows + 1):
            if start_rank > n_windows - duration:
                continue
            for signs in product([-1, 1], repeat=len(channels)):
                specs.append(
                    DirectedEnvelopeSpec(
                        index=len(specs),
                        channels=channels,
                        signs=tuple(int(sign) for sign in signs),
                        amplitude=float(stick_limit),
                        onset_window=start_rank,
                        duration_windows=duration,
                    )
                )
    return specs


def _parse_channels_arg(value: str) -> list[str]:
    channels = [item.strip() for item in str(value).split(",") if item.strip()]
    if not channels:
        raise ValueError("--active-channels cannot be empty")
    return channels


def trigger_signature_from_theta(
    theta: np.ndarray,
    groups: list[Group],
    threshold: float = SUPPORT_THRESHOLD,
) -> str:
    theta = np.asarray(theta, dtype=float)
    active = [
        (group.channel, group.window_id, float(theta[group.group_id]))
        for group in groups
        if abs(float(theta[group.group_id])) > threshold
    ]
    if not active:
        return "channels=none;time=none;signs=none"
    channels = sorted({channel for channel, _, _ in active})
    windows = [window for _, window, _ in active]
    signs = []
    for channel in channels:
        channel_values = [value for row_channel, _, value in active if row_channel == channel]
        has_pos = any(value > 0.0 for value in channel_values)
        has_neg = any(value < 0.0 for value in channel_values)
        sign = "mixed" if has_pos and has_neg else ("+" if has_pos else "-")
        signs.append(f"{channel}:{sign}")
    return f"channels={','.join(channels)};time=w{min(windows):02d}-w{max(windows):02d};signs={'|'.join(signs)}"


def build_summary(
    *,
    point_df: pd.DataFrame,
    evaluator: DirectionAProbeEvaluator,
    scenario_id: str,
    seed: int,
    output_dir: Path,
    preregistration: dict[str, Any],
    arm_details: dict[str, dict[str, Any]],
    target_property: str,
    active_channels: list[str],
    elapsed_wall_time_s: float,
    nonzero_seed_unlocked: bool,
) -> dict[str, Any]:
    arm_metrics = [
        _arm_metrics(point_df, arm, evaluator.gate_rejects_by_arm[arm], target_property)
        for arm in ["A", "B", "C"]
    ]
    reported_properties = list(dict.fromkeys([target_property] + REPORT_PROPERTIES))
    robust = point_df[point_df["robustness_class"] == "robust_violation"].copy()
    interior = robust[robust["amplitude_class"] == "interior"].copy()
    overall_gentlest = _row_with_theta(_gentlest_row(robust), target_property=target_property)
    decision_inputs = _decision_inputs(arm_metrics)

    robust.to_csv(output_dir / "reports" / "robust_violations.csv", index=False)
    interior.to_csv(output_dir / "reports" / "interior_violations.csv", index=False)
    pd.DataFrame(arm_metrics).to_csv(output_dir / "reports" / "arm_metrics.csv", index=False)

    return _jsonable(
        {
            "status": "complete",
            "scenario_id": scenario_id,
            "seed": seed,
            "property": target_property,
            "reported_properties": reported_properties,
            "pre_registration": preregistration,
            "derived_A_phi": {
                "property": target_property,
                "active_channels": list(active_channels),
                "source": "derive_A_phi" if list(active_channels) == derive_A_phi(target_property) else "h1_narrowed_override",
            },
            "seed_freeze_control": {
                "nonzero_seed_unlocked": bool(nonzero_seed_unlocked),
                "allow_nonzero_seed_flag": bool(preregistration["seed_freeze_control"]["allow_nonzero_seed_flag"]),
                "statement": (
                    "Seed was explicitly unlocked for replication; all pre-registered thresholds and budgets "
                    "remain identical to the seed-0 probe."
                    if nonzero_seed_unlocked
                    else "Seed-0 frozen run or no nonzero seed override was used."
                ),
            },
            "arm_details": arm_details,
            "arm_metrics": arm_metrics,
            "overall_gentlest_robust_violation": overall_gentlest,
            "overall_gentlest_xy_velocity_robust_violation": overall_gentlest,
            "interior_violations": [
                _interior_detail(row, target_property=target_property) for _, row in interior.iterrows()
            ],
            "decision_inputs": decision_inputs,
            "successful_query_count": evaluator.successful_query_count,
            "timeout_retry_count": evaluator.timeout_retry_count,
            "query_attempt_count_including_timeout_retries": evaluator.successful_query_count
            + evaluator.timeout_retry_count,
            "elapsed_wall_time_s": elapsed_wall_time_s,
            "artifacts": {
                "pre_registration": str(output_dir / "reports" / "pre_registration.json"),
                "point_evaluations": str(output_dir / "reports" / "point_evaluations.csv"),
                "query_repeats": str(output_dir / "reports" / "query_repeats.csv"),
                "arm_metrics": str(output_dir / "reports" / "arm_metrics.csv"),
                "robust_violations": str(output_dir / "reports" / "robust_violations.csv"),
                "interior_violations": str(output_dir / "reports" / "interior_violations.csv"),
                "summary": str(output_dir / "reports" / "direction_a_summary.json"),
                "report": str(output_dir / "reports" / "direction_a_report.md"),
                "groups": str(output_dir / "groups.csv"),
            },
            "replication_caveat": "Single seed/scenario probe only; replicate across seeds before any paper claim.",
        }
    )


def _arm_metrics(
    point_df: pd.DataFrame,
    arm: str,
    gate_reject_count: int,
    target_property: str = TARGET_PROPERTY,
) -> dict[str, Any]:
    df = point_df[point_df["arm"] == arm].copy()
    robust = df[df["robustness_class"] == "robust_violation"].copy()
    gentlest = _row_with_theta(_gentlest_row(robust), include_theta=False, target_property=target_property)
    return {
        "arm": arm,
        "j5_point_count": int(len(df)),
        "successful_query_count": int(df["repeats"].sum()) if not df.empty else 0,
        "robust_violation_count": int(len(robust)),
        "robust_safe_count": int((df["robustness_class"] == "robust_safe").sum()),
        "noise_band_count": int((df["robustness_class"] == "noise_band").sum()),
        "interior_robust_violation_count": int((robust["amplitude_class"] == "interior").sum()),
        "moderate_robust_violation_count": int((robust["amplitude_class"] == "moderate").sum()),
        "saturated_robust_violation_count": int((robust["amplitude_class"] == "saturated").sum()),
        "negative_mean_rejected_by_2sigma_gate_count": int(gate_reject_count),
        "amplitude_distribution_robust_violations": _distribution(robust["max_abs_theta"].to_numpy(dtype=float))
        if not robust.empty
        else {},
        "gentlest_robust_violation": gentlest,
        "interior_violation_supports": [
            _interior_detail(row, target_property=target_property)
            for _, row in robust[robust["amplitude_class"] == "interior"].iterrows()
        ],
    }


def _decision_inputs(arm_metrics: list[dict[str, Any]]) -> dict[str, Any]:
    by_arm = {row["arm"]: row for row in arm_metrics}
    b_best = by_arm["B"]["gentlest_robust_violation"]
    c_best = by_arm["C"]["gentlest_robust_violation"]
    b_interior = _best_interior(by_arm["B"])
    c_interior = _best_interior(by_arm["C"])
    channel_reduction_gentler = bool(
        c_interior is not None
        and (b_interior is None or c_interior["max_abs_theta"] < b_interior["max_abs_theta"])
    )
    channel_reduction_cleaner = bool(
        c_interior is not None
        and (b_interior is None or c_interior["support_size_abs_gt_0p1"] < b_interior["support_size_abs_gt_0p1"])
    )
    return {
        "premise_arm_a_interior_robust_violation_count": by_arm["A"]["interior_robust_violation_count"],
        "premise_arm_a_robust_violation_count": by_arm["A"]["robust_violation_count"],
        "premise_arm_a_amplitude_distribution": by_arm["A"]["amplitude_distribution_robust_violations"],
        "interior_targeting_value_condition": (
            by_arm["B"]["interior_robust_violation_count"] > 0
            and by_arm["A"]["interior_robust_violation_count"] == 0
        ),
        "channel_reduction_gentler_than_arm_b": channel_reduction_gentler,
        "channel_reduction_cleaner_than_arm_b": channel_reduction_cleaner,
        "channel_reduction_value_condition": channel_reduction_gentler or channel_reduction_cleaner,
        "arm_b_gentlest_robust_violation": b_best,
        "arm_c_gentlest_robust_violation": c_best,
        "arm_b_gentlest_interior_violation": b_interior,
        "arm_c_gentlest_interior_violation": c_interior,
        "full_direction_a_confirmed_strict_no_interior_arm_a": (
            by_arm["A"]["interior_robust_violation_count"] == 0
            and by_arm["C"]["interior_robust_violation_count"] > by_arm["A"]["interior_robust_violation_count"]
            and (channel_reduction_gentler or channel_reduction_cleaner)
        ),
        "note": "The pre-registered decision language is qualitative for 'few/readily/clearly'; this block exposes exact inputs without retuning thresholds.",
    }


def _best_interior(arm_metric: dict[str, Any]) -> dict[str, Any] | None:
    rows = arm_metric["interior_violation_supports"]
    if not rows:
        return None
    return min(rows, key=lambda row: (row["max_abs_theta"], row["support_size_abs_gt_0p1"]))


def _interior_detail(row: pd.Series, target_property: str = TARGET_PROPERTY) -> dict[str, Any]:
    return {
        "arm": row["arm"],
        "eval_id": int(row["eval_id"]),
        "theta_hash": row["theta_hash"],
        "theta_path": row["theta_path"],
        "max_abs_theta": float(row["max_abs_theta"]),
        "support_size_abs_gt_0p1": int(row["support_size_abs_gt_0p1"]),
        "active_channels_abs_gt_0p1": row["active_channels_abs_gt_0p1"],
        f"rho_mean_{target_property}": float(row[f"rho_mean_{target_property}"]),
        f"rho_std_{target_property}": float(row[f"rho_std_{target_property}"]),
        "distinct_signature": row.get("distinct_signature", ""),
        "signature_active_channels": row.get("signature_active_channels", ""),
        "signature_window_band": row.get("signature_window_band", ""),
        "signature_channel_signs": row.get("signature_channel_signs", ""),
    }


def _gentlest_row(df: pd.DataFrame) -> pd.Series | None:
    if df.empty:
        return None
    ordered = df.sort_values(["max_abs_theta", "support_size_abs_gt_0p1", "eval_id"], ascending=[True, True, True])
    return ordered.iloc[0]


def _row_with_theta(
    row: pd.Series | None,
    *,
    include_theta: bool = True,
    target_property: str = TARGET_PROPERTY,
) -> dict[str, Any] | None:
    if row is None:
        return None
    result = {
        "arm": row["arm"],
        "eval_id": int(row["eval_id"]),
        "theta_hash": row["theta_hash"],
        "theta_path": row["theta_path"],
        "stage": row["stage"],
        "label": row["label"],
        "max_abs_theta": float(row["max_abs_theta"]),
        "amplitude_class": row["amplitude_class"],
        "support_size_abs_gt_0p1": int(row["support_size_abs_gt_0p1"]),
        "active_channels_abs_gt_0p1": row["active_channels_abs_gt_0p1"],
    }
    for prop in list(dict.fromkeys([target_property] + REPORT_PROPERTIES)):
        mean_key = f"rho_mean_{prop}"
        std_key = f"rho_std_{prop}"
        if mean_key in row and std_key in row:
            result[mean_key] = float(row[mean_key])
            result[std_key] = float(row[std_key])
    if include_theta:
        result["theta"] = np.load(row["theta_path"]).astype(float).tolist()
    return result


def _property_stats(values: dict[str, list[float]]) -> dict[str, dict[str, float]]:
    stats = {}
    for prop, prop_values in values.items():
        arr = np.asarray(prop_values, dtype=float)
        stats[prop] = {
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "nonpositive": int(np.sum(arr <= 0.0)),
        }
    return stats


def _distribution(values: np.ndarray) -> dict[str, float]:
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {}
    percentiles = {
        "min": 0.0,
        "p05": 0.05,
        "p10": 0.10,
        "p25": 0.25,
        "median": 0.50,
        "p75": 0.75,
        "p90": 0.90,
        "p95": 0.95,
        "max": 1.0,
    }
    return {key: float(np.quantile(arr, q)) for key, q in percentiles.items()}


def _initial_envelope_specs(config: ExperimentConfig, rng: np.random.Generator, stick_limit: float) -> list[EnvelopeSpec]:
    n_windows = window_count(config)
    directions = [0, 45, 90, 135, 180, 225, 270, 315]
    specs: list[EnvelopeSpec] = []
    for duration in [2, 3, 4, 5, 6, 8, 10]:
        if duration > n_windows:
            continue
        step = max(1, duration // 2)
        for onset in range(0, n_windows - duration + 1, step):
            for degrees in directions:
                specs.append(
                    EnvelopeSpec(
                        index=len(specs),
                        angle_rad=math.radians(degrees),
                        amplitude=float(stick_limit),
                        onset_window=onset,
                        duration_windows=duration,
                    )
                )
    rng.shuffle(specs)
    return specs


def _random_envelope_spec(
    config: ExperimentConfig,
    rng: np.random.Generator,
    stick_limit: float,
    index: int,
) -> EnvelopeSpec:
    n_windows = window_count(config)
    duration = int(rng.integers(2, n_windows + 1))
    onset = int(rng.integers(0, n_windows - duration + 1))
    return EnvelopeSpec(
        index=index,
        angle_rad=float(rng.uniform(0.0, 2.0 * math.pi)),
        amplitude=float(stick_limit),
        onset_window=onset,
        duration_windows=duration,
    )


def _linf_unit_direction(angle_rad: float) -> tuple[float, float]:
    roll = math.cos(float(angle_rad))
    pitch = math.sin(float(angle_rad))
    denom = max(abs(roll), abs(pitch), 1e-12)
    return roll / denom, pitch / denom


def _config_for_probe(config: ExperimentConfig, output_dir: Path, stick_limit: float) -> ExperimentConfig:
    input_cfg = dict(config.input)
    input_cfg["min_value"] = -float(stick_limit)
    input_cfg["max_value"] = float(stick_limit)
    logging = dict(config.logging)
    logging["jsonl"] = str(Path(output_dir) / "logs" / "queries.jsonl")
    return replace(config, experiment_id=Path(output_dir).name, input=input_cfg, logging=logging)


def _pre_registration(
    args: argparse.Namespace,
    config: ExperimentConfig,
    groups: list[Group],
    target_property: str = TARGET_PROPERTY,
    active_channels: list[str] | None = None,
) -> dict[str, Any]:
    derived_channels = derive_A_phi(target_property)
    active_channels = list(active_channels) if active_channels is not None else derived_channels
    return {
        "scope": {
            "scenario": args.scenario,
            "seed": int(args.seed),
            "primary_property": target_property,
            "reported_cross_properties": [prop for prop in REPORT_PROPERTIES if prop != target_property],
            "D": len(groups),
            "pid_firmware_sensors": "frozen by simulator/config; runner only overrides stick min/max for this probe",
        },
        "matched_budget": {
            "j5_points_per_arm": int(args.points_per_arm),
            "repeats_per_point": int(args.repeats),
            "arms": ["A_uniform_random", "B_random_interior_bracketing", "C_channel_directed"],
        },
        "thresholds": {
            "robust_violation": "rho_mean + 2*rho_std < 0",
            "robust_safe": "rho_mean - 2*rho_std > 0",
            "sigma_multiplier": ROBUST_SIGMA_MULTIPLIER,
            "interior_max_abs_theta": INTERIOR_MAX_ABS,
            "saturated_min_abs_theta": SATURATED_MIN_ABS,
            "moderate_interval": "(0.5, 0.9]",
            "support_abs_threshold": SUPPORT_THRESHOLD,
        },
        "derive_A_phi": {
            "rule": "control-allocation predicted active channels",
            "xy_velocity": derive_A_phi("post_neutral_xy_velocity"),
            "alt_drift": derive_A_phi("post_neutral_alt_drift"),
            "climb_rate": derive_A_phi("post_neutral_climb_rate"),
            "yaw_rate": derive_A_phi("post_neutral_yaw_rate"),
            "active_for_primary_property": active_channels,
            "active_source": "derive_A_phi" if active_channels == derived_channels else "h1_narrowed_override",
        },
        "channel_relevant_set_for_xy_velocity": derive_A_phi("post_neutral_xy_velocity"),
        "input": {
            "horizon_s": float(config.input["horizon_s"]),
            "window_s": float(config.input["window_s"]),
            "neutral_tail_s": float(config.input["neutral_tail_s"]),
            "channels": list(config.input["channels"]),
            "min_value": float(config.input["min_value"]),
            "max_value": float(config.input["max_value"]),
            "max_delta_per_window": float(config.input["max_delta_per_window"]),
        },
        "rng_seed": int(args.rng_seed),
        "bisection_iters": int(args.bisection_iters),
        "seed_freeze_control": {
            "allow_nonzero_seed_flag": bool(getattr(args, "allow_nonzero_seed", False)),
            "nonzero_seed_unlocked": bool(int(args.seed) != 0 and getattr(args, "allow_nonzero_seed", False)),
            "discipline_statement": (
                "Nonzero seed replication is allowed only through an explicit flag; constants remain frozen "
                "to the seed-0 pre-registration."
            ),
        },
    }


def _run_query_with_retry_count(
    theta,
    scenario,
    seed: int,
    query_type: str,
    output_dir: Path,
    config,
    *,
    cache_tag: str | None = None,
    use_cache: bool = True,
) -> tuple[QueryResult, int]:
    max_attempts = int(config.simulator.get(scenario.platform, {}).get("query_timeout_retries", 2)) + 1
    retry_count = 0
    for attempt in range(1, max_attempts + 1):
        try:
            return (
                run_query(theta, scenario, seed, query_type, output_dir, config, use_cache=use_cache, cache_tag=cache_tag),
                retry_count,
            )
        except TimeoutError as exc:
            if attempt >= max_attempts:
                raise
            retry_count += 1
            print(
                f"direction_a_query_retry scenario={scenario.id} seed={seed} type={query_type} "
                f"attempt={attempt}/{max_attempts} error={exc}",
                flush=True,
            )
            time.sleep(2.0)
    raise RuntimeError("unreachable query retry state")


def _write_report(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# Direction-A Discriminating Probe",
        "",
        f"Scope: `{summary['scenario_id']}`, seed {summary['seed']}, property `{summary['property']}`.",
        (
            f"Matched budget: N={summary['pre_registration']['matched_budget']['j5_points_per_arm']} "
            f"J=5 points per arm, {summary['successful_query_count']} successful PX4 queries total."
        ),
        "",
        "## Arm Outcomes",
        "",
        "| arm | J=5 points | robust violations | interior | moderate | saturated | safe | noise band | 2sigma gate rejects | gentlest max|theta| | support | active channels |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in summary["arm_metrics"]:
        gentlest = row["gentlest_robust_violation"] or {}
        lines.append(
            f"| {row['arm']} | {row['j5_point_count']} | {row['robust_violation_count']} | "
            f"{row['interior_robust_violation_count']} | {row['moderate_robust_violation_count']} | "
            f"{row['saturated_robust_violation_count']} | {row['robust_safe_count']} | "
            f"{row['noise_band_count']} | {row['negative_mean_rejected_by_2sigma_gate_count']} | "
            f"{_fmt(gentlest.get('max_abs_theta'))} | {_fmt(gentlest.get('support_size_abs_gt_0p1'), integer=True)} | "
            f"{gentlest.get('active_channels_abs_gt_0p1', '')} |"
        )
    lines.extend(["", "## Robust-Violation Amplitude Percentiles", ""])
    for row in summary["arm_metrics"]:
        dist = row["amplitude_distribution_robust_violations"]
        if not dist:
            lines.append(f"- Arm {row['arm']}: no robust violations.")
            continue
        lines.append(
            f"- Arm {row['arm']}: min={dist['min']:.3f}, p25={dist['p25']:.3f}, "
            f"median={dist['median']:.3f}, p75={dist['p75']:.3f}, p90={dist['p90']:.3f}, "
            f"p95={dist['p95']:.3f}, max={dist['max']:.3f}."
        )

    target_property = summary["property"]
    gentlest = summary["overall_gentlest_robust_violation"]
    lines.extend(["", "## Gentlest Robust Violation", ""])
    if gentlest is None:
        lines.append(f"No robust {target_property} violation was found by any arm.")
    else:
        lines.extend(
            [
                f"- arm: `{gentlest['arm']}`",
                f"- theta hash: `{gentlest['theta_hash']}`",
                f"- theta path: `{gentlest['theta_path']}`",
                f"- max|theta|: {gentlest['max_abs_theta']:.6f} ({gentlest['amplitude_class']})",
                f"- support size |theta|>0.1: {gentlest['support_size_abs_gt_0p1']}",
                f"- active channels: `{gentlest['active_channels_abs_gt_0p1']}`",
                (
                    "- cross-property rho means: "
                    + ", ".join(
                        f"{prop}={gentlest[f'rho_mean_{prop}']:.6f}"
                        for prop in summary.get("reported_properties", REPORT_PROPERTIES)
                        if f"rho_mean_{prop}" in gentlest
                    )
                ),
                "",
                "Theta (D=40 group order):",
                "",
                "```json",
                json.dumps(gentlest["theta"]),
                "```",
            ]
        )

    lines.extend(["", "## Interior Violations", ""])
    interior = summary["interior_violations"]
    if not interior:
        lines.append("No interior robust violations were found.")
    else:
        lines.extend(
            [
                "| arm | eval | max|theta| | support | active channels | rho mean | rho std | theta hash |",
                "| --- | ---: | ---: | ---: | --- | ---: | ---: | --- |",
            ]
        )
        for row in interior:
            lines.append(
                f"| {row['arm']} | {row['eval_id']} | {row['max_abs_theta']:.6f} | "
                f"{row['support_size_abs_gt_0p1']} | {row['active_channels_abs_gt_0p1']} | "
                f"{row[f'rho_mean_{target_property}']:.6f} | "
                f"{row[f'rho_std_{target_property}']:.6f} | {row['theta_hash']} |"
            )

    decision = summary["decision_inputs"]
    lines.extend(
        [
            "",
            "## Decision Inputs",
            "",
            f"- Arm A interior robust violations: {decision['premise_arm_a_interior_robust_violation_count']}",
            f"- Arm A robust violations total: {decision['premise_arm_a_robust_violation_count']}",
            f"- Interior-targeting value condition: `{decision['interior_targeting_value_condition']}`",
            f"- Channel-reduction gentler than Arm B: `{decision['channel_reduction_gentler_than_arm_b']}`",
            f"- Channel-reduction cleaner than Arm B: `{decision['channel_reduction_cleaner_than_arm_b']}`",
            f"- Strict no-interior-Arm-A confirmation flag: `{decision['full_direction_a_confirmed_strict_no_interior_arm_a']}`",
            "",
            "The qualitative terms in the decision rule (`few`, `readily`, `clearly`) are left as exact counts and distributions here; no thresholds were tuned after seeing data.",
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
            "Single seed/scenario probe only; replicate across seeds before any paper claim.",
            "",
            "Stop point: three arms plus classifier only.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _fmt(value: Any, *, integer: bool = False) -> str:
    if value is None:
        return ""
    try:
        if not math.isfinite(float(value)):
            return ""
    except (TypeError, ValueError):
        return ""
    return str(int(value)) if integer else f"{float(value):.3f}"


def _safe_label(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in value)[:180]


def _scale_label(value: float) -> str:
    return f"{float(value):.4f}".replace(".", "p").replace("-", "m")


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(_jsonable(payload), indent=2, sort_keys=True), encoding="utf-8")


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
        value = float(value)
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    return value


if __name__ == "__main__":
    main()
