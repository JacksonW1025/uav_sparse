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

from cadet.config import ExperimentConfig, ScenarioCfg, load_config
from cadet.groups import Group, build_groups
from cadet.input_model import project_theta, theta_to_sequence
from cadet.properties import compute_robustness
from cadet.query import QueryResult, read_parsed_log, theta_hash
from cadet.runners.fd_snapshot import _run_query_with_retry
from cadet.violation_search import grid_to_theta, saturation_summary, theta_to_grid, window_count


PROPERTY = "post_neutral_xy_velocity"
TERMINAL_OFFSET_S = (6.0, 8.0)
SUBWINDOW_OFFSETS_S = ((0.0, 2.0), (2.0, 4.0), (4.0, 6.0), (6.0, 8.0))
V_STRESS_MPS = 2.0
PARAM_PINS = {"MPC_ACC_HOR": 3.0, "MPC_JERK_MAX": 8.0}


@dataclass(frozen=True)
class ManeuverProfile:
    index: int
    label: str
    theta: np.ndarray
    requested_amplitude: float
    effective_amplitude: float
    t_switch_s: float
    channel_pattern: str
    variant: str
    start_window: int
    profile_values: str
    request_saturated: bool


def main() -> None:
    parser = argparse.ArgumentParser(description="Transition handoff contract test v2: PX4 Position->Hold.")
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/transition_handoff_v2_a04_seed0")
    parser.add_argument("--stress-repeats", type=int, default=1)
    parser.add_argument("--diff-repeats", type=int, default=5)
    parser.add_argument("--repeats", type=int, default=None, help="Deprecated alias for --diff-repeats.")
    parser.add_argument("--t-grid", default="5,6,8,10")
    parser.add_argument("--amplitudes", default="0.85,1.0")
    parser.add_argument("--channel-patterns", default="pitch")
    parser.add_argument("--variants", default="long_hold")
    parser.add_argument("--tail-margin-s", type=float, default=1.0)
    parser.add_argument("--step2-max-profiles", type=int, default=12)
    parser.add_argument("--output-csv", default="recon_v0/transition_exploratory_v2.csv")
    parser.add_argument("--summary-json", default="recon_v0/transition_exploratory_v2_summary.json")
    args = parser.parse_args()

    if int(args.seed) != 0:
        raise ValueError("Phase T1 is exploratory and frozen to seed 0")
    if args.repeats is not None:
        args.diff_repeats = int(args.repeats)
    if int(args.stress_repeats) != 1:
        raise ValueError("Amendment 04 stress map is frozen to J=1")
    if int(args.diff_repeats) != 5:
        raise ValueError("Transition v2 differential labels require J=5")

    output_dir = Path(args.run_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    t_grid = _parse_float_list(args.t_grid)
    config = _config_for_run_dir(
        load_config(args.config),
        output_dir,
        max_t_switch_s=max(t_grid),
        terminal_margin_s=float(args.tail_margin_s),
    )
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(output_dir / "groups.csv", index=False)

    amplitudes = _parse_float_list(args.amplitudes)
    channel_patterns = [item.strip() for item in args.channel_patterns.split(",") if item.strip()]
    variants = [item.strip() for item in args.variants.split(",") if item.strip()]
    profiles = generate_profiles(config, groups, amplitudes, t_grid, channel_patterns, variants)

    print(
        "transition_v2_t1_start "
        f"profiles={len(profiles)} seed={args.seed} stress_repeats={args.stress_repeats} "
        f"diff_repeats={args.diff_repeats} horizon={config.input['horizon_s']} "
        f"neutral_tail={config.input['neutral_tail_s']} t_grid={t_grid} amplitudes={amplitudes} "
        f"patterns={channel_patterns} variants={variants}",
        flush=True,
    )
    t0 = time.monotonic()
    rows: list[dict[str, Any]] = []
    stress_sw_by_index: dict[int, dict[str, Any]] = {}
    sw_scenario_by_t = {t_switch: _transition_scenario(config, t_switch) for t_switch in sorted({p.t_switch_s for p in profiles})}
    ns_scenario = _position_scenario(config)

    for ordinal, profile in enumerate(profiles, start=1):
        summary_row = evaluate_profile(
            profile,
            arm="SW",
            scenario=sw_scenario_by_t[profile.t_switch_s],
            seed=args.seed,
            repeats=args.stress_repeats,
            stage="stress_map",
            config=config,
            groups=groups,
            output_dir=output_dir,
        )
        rows.append(summary_row)
        stress_sw_by_index[profile.index] = summary_row
        _write_rows(Path(args.output_csv), rows)
        print(
            "transition_v2_stress "
            f"{ordinal}/{len(profiles)} label={profile.label} "
            f"vtrans={summary_row['velocity_at_transition_mps']:.3f} "
            f"terminal_peak={summary_row['terminal_peak']:.3f} "
            f"rho={summary_row['rho_mean']:.3f} class={summary_row['label']}",
            flush=True,
        )

    stress_valid = [
        profile
        for profile in profiles
        if float(stress_sw_by_index[profile.index].get("velocity_at_transition_mps", math.nan)) >= V_STRESS_MPS
    ]
    max_velocity_mean = max((float(row["velocity_at_transition_mps"]) for row in rows), default=math.nan)
    max_velocity_repeat = max((float(row["velocity_at_transition_mps_max"]) for row in rows), default=math.nan)

    if not stress_valid:
        outcome = "STRUCTURAL_INCONCLUSIVE"
        summary = _summary_payload(
            args,
            config,
            rows,
            stress_valid,
            output_dir,
            elapsed=time.monotonic() - t0,
            outcome=outcome,
            max_velocity_mean=max_velocity_mean,
            max_velocity_repeat=max_velocity_repeat,
            step2_profiles=[],
            stress_sw_by_index=stress_sw_by_index,
            diff_sw_by_index={},
            ns_by_index={},
        )
        _write_json(Path(args.summary_json), summary)
        _write_rows(Path(args.output_csv), rows)
        print(
            "transition_v2_t1_stop "
            f"outcome={outcome} max_velocity_at_transition_mean={max_velocity_mean:.3f} "
            f"max_repeat={max_velocity_repeat:.3f}",
            flush=True,
        )
        return

    step2_profiles = _select_step2_profiles(profiles, stress_sw_by_index, int(args.step2_max_profiles))
    print(
        "transition_v2_step2_start "
        f"stress_valid={len(stress_valid)} selected={len(step2_profiles)} diff_repeats={args.diff_repeats}",
        flush=True,
    )
    diff_sw_by_index: dict[int, dict[str, Any]] = {}
    ns_by_index: dict[int, dict[str, Any]] = {}
    for ordinal, profile in enumerate(step2_profiles, start=1):
        sw_summary_row = evaluate_profile(
            profile,
            arm="SW",
            scenario=sw_scenario_by_t[profile.t_switch_s],
            seed=args.seed,
            repeats=args.diff_repeats,
            stage="differential",
            config=config,
            groups=groups,
            output_dir=output_dir,
        )
        rows.append(sw_summary_row)
        diff_sw_by_index[profile.index] = sw_summary_row
        _write_rows(Path(args.output_csv), rows)
        print(
            "transition_v2_sw_diff "
            f"{ordinal}/{len(step2_profiles)} label={profile.label} "
            f"vtrans={sw_summary_row['velocity_at_transition_mps']:.3f} "
            f"terminal_peak={sw_summary_row['terminal_peak']:.3f} "
            f"rho={sw_summary_row['rho_mean']:.3f} class={sw_summary_row['label']}",
            flush=True,
        )

        ns_summary_row = evaluate_profile(
            profile,
            arm="NS",
            scenario=ns_scenario,
            seed=args.seed,
            repeats=args.diff_repeats,
            stage="differential",
            config=config,
            groups=groups,
            output_dir=output_dir,
        )
        rows.append(ns_summary_row)
        ns_by_index[profile.index] = ns_summary_row
        _annotate_pair_rows(rows, diff_sw_by_index, ns_by_index)
        _write_rows(Path(args.output_csv), rows)
        print(
            "transition_v2_ns "
            f"{ordinal}/{len(step2_profiles)} label={profile.label} "
            f"terminal_peak={ns_summary_row['terminal_peak']:.3f} "
            f"rho={ns_summary_row['rho_mean']:.3f} class={ns_summary_row['label']}",
            flush=True,
        )

    _annotate_pair_rows(rows, diff_sw_by_index, ns_by_index)
    outcome = _classify_t1(rows, stress_valid, diff_sw_by_index, ns_by_index)
    summary = _summary_payload(
        args,
        config,
        rows,
        stress_valid,
        output_dir,
        elapsed=time.monotonic() - t0,
        outcome=outcome,
        max_velocity_mean=max_velocity_mean,
        max_velocity_repeat=max_velocity_repeat,
        step2_profiles=step2_profiles,
        stress_sw_by_index=stress_sw_by_index,
        diff_sw_by_index=diff_sw_by_index,
        ns_by_index=ns_by_index,
    )
    _write_rows(Path(args.output_csv), rows)
    _write_json(Path(args.summary_json), summary)
    print(
        "transition_v2_t1_stop "
        f"outcome={outcome} max_velocity_at_transition_mean={max_velocity_mean:.3f} "
        f"attributable_candidates={summary['attributable_candidate_count']}",
        flush=True,
    )


def generate_profiles(
    config: ExperimentConfig,
    groups: list[Group],
    amplitudes: list[float],
    t_grid: list[float],
    channel_patterns: list[str],
    variants: list[str],
) -> list[ManeuverProfile]:
    profiles: list[ManeuverProfile] = []
    for t_switch in t_grid:
        _validate_t_switch_grid(config, t_switch)
        for requested_amp in amplitudes:
            for pattern in channel_patterns:
                channels = _channels_for_pattern(pattern)
                for variant in variants:
                    profile = _build_profile(
                        len(profiles),
                        config,
                        groups,
                        requested_amp=requested_amp,
                        t_switch_s=t_switch,
                        channels=channels,
                        pattern=pattern,
                        variant=variant,
                    )
                    profiles.append(profile)
    return profiles


def _build_profile(
    index: int,
    config: ExperimentConfig,
    groups: list[Group],
    *,
    requested_amp: float,
    t_switch_s: float,
    channels: list[str],
    pattern: str,
    variant: str,
) -> ManeuverProfile:
    n_windows = window_count(config)
    window_s = float(config.input["window_s"])
    switch_window = int(round(float(t_switch_s) / window_s))
    max_step = float(config.input["max_delta_per_window"])
    max_value = float(config.input["max_value"])
    effective_amp = min(abs(float(requested_amp)), max_value)
    ramp_windows = max(1, int(math.ceil(effective_amp / max_step)))
    latest_start = max(0, switch_window - 2 * ramp_windows)
    if variant == "latest":
        start_window = latest_start
    elif variant in {"long", "long_hold"}:
        start_window = 0
    elif variant == "mid":
        start_window = max(0, latest_start // 2)
    else:
        raise ValueError(f"Unknown profile variant: {variant}")

    values = [0.0] * n_windows
    for window_id in range(start_window, switch_window):
        up_limit = max_step * float(window_id - start_window + 1)
        down_limit = max_step * float(switch_window - window_id)
        values[window_id] = min(effective_amp, up_limit, down_limit)

    grid = np.zeros((n_windows, len(config.input["channels"])), dtype=float)
    for channel in channels:
        if channel not in config.input["channels"]:
            raise ValueError(f"Unknown input channel: {channel}")
        grid[:, list(config.input["channels"]).index(channel)] = np.asarray(values, dtype=float)

    theta = project_theta(grid_to_theta(grid, config, groups), config)
    profile_grid = theta_to_grid(theta, config, groups)
    if not np.allclose(profile_grid[switch_window:, :], 0.0, atol=1e-12):
        raise ValueError(f"Profile is not neutral from t_switch={t_switch_s}")
    sequence = theta_to_sequence(theta, groups, config)
    post = sequence[sequence["t_s"] >= float(t_switch_s)]
    manual_cols = list(config.input["channels"])
    if not post.empty and float(post[manual_cols].abs().max().max()) > 1e-12:
        raise ValueError(f"Sequence is not neutral from t_switch={t_switch_s}")

    value_text = ";".join(f"{channel}:{','.join(f'{v:.2f}' for v in values)}" for channel in channels)
    label = (
        f"{pattern}_{variant}_a{_float_label(requested_amp)}"
        f"_ts{_float_label(t_switch_s)}"
    )
    return ManeuverProfile(
        index=index,
        label=label,
        theta=theta,
        requested_amplitude=float(requested_amp),
        effective_amplitude=float(effective_amp),
        t_switch_s=float(t_switch_s),
        channel_pattern=pattern,
        variant=variant,
        start_window=start_window,
        profile_values=value_text,
        request_saturated=bool(abs(float(requested_amp)) > max_value),
    )


def evaluate_profile(
    profile: ManeuverProfile,
    *,
    arm: str,
    scenario: ScenarioCfg,
    seed: int,
    repeats: int,
    stage: str,
    config: ExperimentConfig,
    groups: list[Group],
    output_dir: Path,
) -> dict[str, Any]:
    repeat_rows = []
    projected = project_theta(profile.theta, config)
    thash = theta_hash(projected)
    for repeat_idx in range(repeats):
        cache_tag = _safe_label(
            f"a04_transition_v2_{stage}_{arm}_{thash}_ts{_float_label(profile.t_switch_s)}_repeat{repeat_idx}",
            limit=130,
        )
        result = _run_query_with_retry(
            projected,
            scenario,
            seed,
            f"transition_v2_{stage}",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        parsed = read_parsed_log(result.parsed_log_path)
        repeat_rows.append(
            {
                **_repeat_metrics(parsed, result, profile, config),
                "repeat_idx": repeat_idx,
                "query_id": result.query_id,
                "parsed_log_path": str(result.parsed_log_path),
                "raw_ulg_path": str(Path(result.parsed_log_path).parent / "raw_log.ulg"),
            }
        )
    sat = saturation_summary(projected, config, groups, tol=0.02)
    return _collapse_eval(profile, arm, scenario, seed, stage, repeat_rows, sat)


def _repeat_metrics(
    parsed_log: pd.DataFrame,
    result: QueryResult,
    profile: ManeuverProfile,
    config: ExperimentConfig,
) -> dict[str, Any]:
    parsed = _with_xy_speed(parsed_log)
    threshold = float(config.properties[PROPERTY]["v_max_mps"])
    terminal_window = _terminal_window(profile.t_switch_s)
    terminal_rho = compute_robustness(parsed, PROPERTY, config, window=terminal_window)
    terminal_peak = threshold - terminal_rho
    meta = result.metadata
    transition_observed_t = _first_finite_from_meta(meta, "transition_observed_t_s")
    if not math.isfinite(transition_observed_t):
        transition_observed_t = _first_finite_from_meta(meta, "adapter_transition_observed_t_s")
    velocity_at_transition = _first_finite_from_meta(meta, "velocity_at_transition_mps")
    if not math.isfinite(velocity_at_transition) and math.isfinite(transition_observed_t):
        velocity_at_transition = _speed_at_time(parsed, transition_observed_t)
    speed_at_nominal_switch = _speed_at_time(parsed, profile.t_switch_s)
    if not math.isfinite(velocity_at_transition):
        velocity_at_transition = speed_at_nominal_switch
    manual_post = _manual_abs_max(parsed, profile.t_switch_s, math.inf)
    manual_pre = _manual_abs_max(parsed, 0.0, profile.t_switch_s)
    mode_timeline = _mode_timeline(parsed)
    row = {
        "rho": float(terminal_rho),
        "terminal_peak": float(terminal_peak),
        "terminal_window_lo_s": float(terminal_window[0]),
        "terminal_window_hi_s": float(terminal_window[1]),
        "transition_observed_t_s": float(transition_observed_t),
        "velocity_at_transition_mps": float(velocity_at_transition),
        "speed_at_nominal_switch_mps": float(speed_at_nominal_switch),
        "manual_post_switch_abs_max": float(manual_post),
        "manual_pre_switch_abs_max": float(manual_pre),
        "mode_timeline": mode_timeline,
        "param_acc_readback": _first_finite_from_meta(meta, "adapter_param_override_MPC_ACC_HOR_readback"),
        "param_jerk_readback": _first_finite_from_meta(meta, "adapter_param_override_MPC_JERK_MAX_readback"),
        "transition_request_count": _first_finite_from_meta(meta, "transition_request_count"),
        "query_total_wall_time_s": float(meta.get("total_wall_time_s", math.nan)),
    }
    for lo, hi in SUBWINDOW_OFFSETS_S:
        abs_lo = profile.t_switch_s + lo
        abs_hi = profile.t_switch_s + hi
        row[f"peak_rel_{int(lo)}_{int(hi)}"] = _xy_peak(parsed, abs_lo, abs_hi)
    return row


def _collapse_eval(
    profile: ManeuverProfile,
    arm: str,
    scenario: ScenarioCfg,
    seed: int,
    stage: str,
    repeat_rows: list[dict[str, Any]],
    sat: dict[str, Any],
) -> dict[str, Any]:
    rho_values = _values(repeat_rows, "rho")
    rho_mean = _mean(rho_values)
    rho_std = _std(rho_values)
    label = _robustness_label(rho_mean, rho_std)
    row: dict[str, Any] = {
        "phase": "T1",
        "exploratory": True,
        "step": stage,
        "pair": "PX4 Position->Hold",
        "arm": arm,
        "scenario_id": scenario.id,
        "seed": int(seed),
        "J": len(repeat_rows),
        "profile_index": profile.index,
        "profile_label": profile.label,
        "channel_pattern": profile.channel_pattern,
        "variant": profile.variant,
        "start_window": profile.start_window,
        "requested_amplitude": profile.requested_amplitude,
        "effective_amplitude": profile.effective_amplitude,
        "t_switch": profile.t_switch_s,
        "t_switch_s": profile.t_switch_s,
        "profile_values": profile.profile_values,
        "theta_hash": theta_hash(profile.theta),
        "max_abs": float(sat["max_abs_theta"]),
        "saturated": bool(profile.request_saturated or sat["amplitude_saturated"]),
        "request_saturated": bool(profile.request_saturated),
        "amplitude_saturated": bool(sat["amplitude_saturated"]),
        "manual_post_switch_abs_max": _max(_values(repeat_rows, "manual_post_switch_abs_max")),
        "manual_pre_switch_abs_max": _max(_values(repeat_rows, "manual_pre_switch_abs_max")),
        "rho_mean": rho_mean,
        "rho_std": rho_std,
        "rho_min": _min(rho_values),
        "rho_max": _max(rho_values),
        "label": label,
        "threshold": 1.0,
        "V_stress": V_STRESS_MPS,
        "terminal_window_lo_s": _mean(_values(repeat_rows, "terminal_window_lo_s")),
        "terminal_window_hi_s": _mean(_values(repeat_rows, "terminal_window_hi_s")),
        "terminal_window_relative": "t_switch+6..t_switch+8",
        "mode_timeline": _join_unique(row["mode_timeline"] for row in repeat_rows),
        "query_ids": ";".join(str(row["query_id"]) for row in repeat_rows),
        "parsed_log_paths": ";".join(str(row["parsed_log_path"]) for row in repeat_rows),
        "raw_ulg_paths": ";".join(str(row["raw_ulg_path"]) for row in repeat_rows),
        "param_acc_readback_min": _min(_values(repeat_rows, "param_acc_readback")),
        "param_acc_readback_max": _max(_values(repeat_rows, "param_acc_readback")),
        "param_jerk_readback_min": _min(_values(repeat_rows, "param_jerk_readback")),
        "param_jerk_readback_max": _max(_values(repeat_rows, "param_jerk_readback")),
        "transition_request_count_max": _max(_values(repeat_rows, "transition_request_count")),
        "query_total_wall_time_s_sum": float(sum(v for v in _values(repeat_rows, "query_total_wall_time_s") if math.isfinite(v))),
        "attributable_candidate": False,
        "pair_status": "",
    }
    for key in [
        "velocity_at_transition_mps",
        "speed_at_nominal_switch_mps",
        "transition_observed_t_s",
        "terminal_peak",
        "peak_rel_0_2",
        "peak_rel_2_4",
        "peak_rel_4_6",
        "peak_rel_6_8",
    ]:
        values = _values(repeat_rows, key)
        row[key] = _mean(values)
        row[f"{key}_std"] = _std(values)
        row[f"{key}_min"] = _min(values)
        row[f"{key}_max"] = _max(values)
    return row


def _annotate_pair_rows(
    rows: list[dict[str, Any]],
    sw_by_index: dict[int, dict[str, Any]],
    ns_by_index: dict[int, dict[str, Any]],
) -> None:
    for profile_index, sw in sw_by_index.items():
        ns = ns_by_index.get(profile_index)
        if ns is None:
            continue
        sw_robust_violation = sw["label"] == "robust_violation"
        ns_robust_safe = ns["label"] == "robust_safe"
        stress_valid = float(sw["velocity_at_transition_mps"]) >= V_STRESS_MPS
        attributable = bool(stress_valid and sw_robust_violation and ns_robust_safe)
        if attributable:
            status = "attributable_candidate"
        elif sw_robust_violation and not ns_robust_safe:
            status = "velocity_artifact_or_ns_not_safe"
        elif sw["label"] == "robust_safe" and ns_robust_safe:
            status = "both_safe"
        else:
            status = "noise_or_mixed"
        for row in rows:
            if int(row["profile_index"]) == int(profile_index):
                row["attributable_candidate"] = attributable
                row["pair_status"] = status
                row["paired_sw_terminal_peak"] = float(sw["terminal_peak"])
                row["paired_ns_terminal_peak"] = float(ns["terminal_peak"])
                row["paired_sw_velocity_at_transition_mps"] = float(sw["velocity_at_transition_mps"])
                row["paired_sw_label"] = sw["label"]
                row["paired_ns_label"] = ns["label"]


def _classify_t1(
    rows: list[dict[str, Any]],
    stress_valid: list[ManeuverProfile],
    sw_by_index: dict[int, dict[str, Any]],
    ns_by_index: dict[int, dict[str, Any]],
) -> str:
    if not stress_valid:
        return "STRUCTURAL_INCONCLUSIVE"
    if any(bool(row.get("attributable_candidate")) for row in rows):
        return "T1_ATTRIBUTABLE_CANDIDATE"
    paired = [
        sw_by_index[p.index]
        for p in stress_valid
        if p.index in sw_by_index and p.index in ns_by_index
    ]
    if not paired:
        return "T1_STRESS_VALID_NEEDS_DIFFERENTIAL"
    robust_sw_violations = [row for row in paired if row["label"] == "robust_violation"]
    if robust_sw_violations:
        ns_not_safe = [
            row
            for row in robust_sw_violations
            if ns_by_index[int(row["profile_index"])]["label"] != "robust_safe"
        ]
        if len(ns_not_safe) == len(robust_sw_violations):
            return "T1_VELOCITY_ARTIFACT_TREND"
        return "T1_SW_CROSSING_NEEDS_REVIEW"
    return "T1_TENTATIVE_H_NULL"


def _select_step2_profiles(
    profiles: list[ManeuverProfile],
    sw_by_index: dict[int, dict[str, Any]],
    max_profiles: int,
) -> list[ManeuverProfile]:
    stress_valid = [
        profile
        for profile in profiles
        if float(sw_by_index[profile.index].get("velocity_at_transition_mps", math.nan)) >= V_STRESS_MPS
    ]
    sw_crossing = [
        profile
        for profile in stress_valid
        if float(sw_by_index[profile.index]["terminal_peak"]) > float(sw_by_index[profile.index]["threshold"])
    ]
    ranked = sorted(stress_valid, key=lambda p: float(sw_by_index[p.index]["velocity_at_transition_mps"]), reverse=True)
    selected: list[ManeuverProfile] = []
    seen = set()
    for profile in sw_crossing + ranked:
        if profile.index in seen:
            continue
        selected.append(profile)
        seen.add(profile.index)
        if len(selected) >= max_profiles:
            break
    return selected


def _summary_payload(
    args: argparse.Namespace,
    config: ExperimentConfig,
    rows: list[dict[str, Any]],
    stress_valid: list[ManeuverProfile],
    output_dir: Path,
    *,
    elapsed: float,
    outcome: str,
    max_velocity_mean: float,
    max_velocity_repeat: float,
    step2_profiles: list[ManeuverProfile],
    stress_sw_by_index: dict[int, dict[str, Any]],
    diff_sw_by_index: dict[int, dict[str, Any]],
    ns_by_index: dict[int, dict[str, Any]],
) -> dict[str, Any]:
    sw_rows = [row for row in rows if row["arm"] == "SW"]
    stress_rows = [row for row in sw_rows if row["step"] == "stress_map"]
    diff_sw_rows = [row for row in sw_rows if row["step"] == "differential"]
    ns_rows = [row for row in rows if row["arm"] == "NS"]
    attributable = [row for row in diff_sw_rows if bool(row.get("attributable_candidate"))]
    velocity_relation = _velocity_relation(diff_sw_rows, ns_rows)
    top_stress = sorted(stress_rows, key=lambda row: float(row["velocity_at_transition_mps"]), reverse=True)[:8]
    top_terminal = sorted(diff_sw_rows or stress_rows, key=lambda row: float(row["terminal_peak"]), reverse=True)[:8]
    return {
        "phase": "T1",
        "exploratory": True,
        "amendment": "04",
        "pair": "PX4 Position->Hold",
        "seed": int(args.seed),
        "J_stress": int(args.stress_repeats),
        "J_differential": int(args.diff_repeats),
        "threshold_mps": float(config.properties[PROPERTY]["v_max_mps"]),
        "V_stress_mps": V_STRESS_MPS,
        "terminal_window_offset_s": list(TERMINAL_OFFSET_S),
        "subwindow_offsets_s": [list(w) for w in SUBWINDOW_OFFSETS_S],
        "param_pins": PARAM_PINS,
        "configured_horizon_s": float(config.input["horizon_s"]),
        "configured_neutral_tail_s": float(config.input["neutral_tail_s"]),
        "current_F_max_value": float(config.input["max_value"]),
        "output_csv": str(args.output_csv),
        "run_dir": str(output_dir),
        "elapsed_wall_time_s": float(elapsed),
        "outcome": outcome,
        "go_no_go_preliminary": _go_no_go(outcome, velocity_relation),
        "max_velocity_at_transition_mean_mps": float(max_velocity_mean),
        "max_velocity_at_transition_repeat_mps": float(max_velocity_repeat),
        "max_j5_velocity_at_transition_mean_mps": _max(
            [float(row["velocity_at_transition_mps"]) for row in diff_sw_rows]
        ),
        "ns_safe_upper_bound_mps": _ns_safe_upper_bound(diff_sw_by_index, ns_by_index),
        "stress_valid_profile_count": len(stress_valid),
        "step2_selected_profile_count": len(step2_profiles),
        "attributable_candidate_count": len(attributable),
        "stress_sw_rows": len(stress_rows),
        "diff_sw_rows": len(diff_sw_rows),
        "ns_rows": len(ns_rows),
        "velocity_relation": velocity_relation,
        "top_stress_rows": _compact_rows(top_stress),
        "top_terminal_rows": _compact_rows(top_terminal),
        "attributable_candidates": _compact_rows(attributable),
        "stress_valid_profiles": _compact_rows(
            [stress_sw_by_index[p.index] for p in stress_valid if p.index in stress_sw_by_index]
        ),
    }


def _velocity_relation(sw_rows: list[dict[str, Any]], ns_rows: list[dict[str, Any]]) -> dict[str, Any]:
    paired = []
    ns_by_index = {int(row["profile_index"]): row for row in ns_rows}
    for sw in sw_rows:
        ns = ns_by_index.get(int(sw["profile_index"]))
        if ns is None:
            continue
        paired.append(
            {
                "profile_index": int(sw["profile_index"]),
                "velocity_at_transition_mps": float(sw["velocity_at_transition_mps"]),
                "sw_terminal_peak": float(sw["terminal_peak"]),
                "ns_terminal_peak": float(ns["terminal_peak"]),
                "sw_residual_over_threshold": float(sw["terminal_peak"]) - float(sw["threshold"]),
                "ns_residual_over_threshold": float(ns["terminal_peak"]) - float(ns["threshold"]),
            }
        )
    return {
        "paired_count": len(paired),
        "sw_spearman_velocity_terminal": _spearman(
            [row["velocity_at_transition_mps"] for row in paired],
            [row["sw_terminal_peak"] for row in paired],
        ),
        "ns_spearman_velocity_terminal": _spearman(
            [row["velocity_at_transition_mps"] for row in paired],
            [row["ns_terminal_peak"] for row in paired],
        ),
        "paired_points": paired,
    }


def _ns_safe_upper_bound(
    diff_sw_by_index: dict[int, dict[str, Any]],
    ns_by_index: dict[int, dict[str, Any]],
) -> float:
    safe_velocities = []
    for profile_index, ns in ns_by_index.items():
        sw = diff_sw_by_index.get(profile_index)
        if sw is None:
            continue
        if ns["label"] == "robust_safe":
            safe_velocities.append(float(sw["velocity_at_transition_mps"]))
    return _max(safe_velocities)


def _go_no_go(outcome: str, velocity_relation: dict[str, Any]) -> str:
    if outcome == "STRUCTURAL_INCONCLUSIVE":
        return "HOLD"
    if outcome == "T1_ATTRIBUTABLE_CANDIDATE":
        return "T1_GO_TO_T2"
    if outcome == "T1_TENTATIVE_H_NULL":
        return "NO_GO_TENTATIVE"
    if outcome == "T1_VELOCITY_ARTIFACT_TREND":
        return "STOP_ARTIFACT_TREND"
    sw_corr = velocity_relation.get("sw_spearman_velocity_terminal")
    ns_corr = velocity_relation.get("ns_spearman_velocity_terminal")
    if isinstance(sw_corr, float) and isinstance(ns_corr, float) and sw_corr > 0.5:
        return "GO_CAUTIOUS_TREND_ONLY"
    return "STOP_REVIEW"


def _compact_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    keys = [
        "profile_index",
        "profile_label",
        "arm",
        "requested_amplitude",
        "effective_amplitude",
        "t_switch_s",
        "velocity_at_transition_mps",
        "terminal_peak",
        "terminal_window_lo_s",
        "terminal_window_hi_s",
        "rho_mean",
        "rho_std",
        "label",
        "saturated",
        "pair_status",
    ]
    return [{key: row.get(key) for key in keys if key in row} for row in rows]


def _config_for_run_dir(
    config: ExperimentConfig,
    output_dir: Path,
    *,
    max_t_switch_s: float | None = None,
    terminal_margin_s: float = 1.0,
) -> ExperimentConfig:
    output_dir = Path(output_dir)
    logging = dict(config.logging)
    logging["jsonl"] = str(output_dir / "logs" / "queries.jsonl")
    input_cfg = dict(config.input)
    if max_t_switch_s is not None:
        window_s = float(input_cfg["window_s"])
        horizon_s = math.ceil(float(max_t_switch_s) / window_s) * window_s
        input_cfg["horizon_s"] = max(float(input_cfg["horizon_s"]), float(horizon_s))
        input_cfg["neutral_tail_s"] = max(
            float(input_cfg["neutral_tail_s"]),
            float(TERMINAL_OFFSET_S[1]) + float(terminal_margin_s),
        )
    return replace(config, experiment_id=output_dir.name, input=input_cfg, logging=logging)


def _position_scenario(config: ExperimentConfig) -> ScenarioCfg:
    base = config.scenario_by_id("px4_position")
    return _with_param_pins(replace(base, properties=[PROPERTY]))


def _transition_scenario(config: ExperimentConfig, t_switch_s: float) -> ScenarioCfg:
    base = config.scenario_by_id("px4_transition")
    return _with_param_pins(replace(base, t_switch_s=float(t_switch_s), properties=[PROPERTY]))


def _with_param_pins(scenario: ScenarioCfg) -> ScenarioCfg:
    overrides = dict(PARAM_PINS)
    overrides.update(dict(getattr(scenario, "param_overrides", {}) or {}))
    return replace(scenario, param_overrides=overrides)


def _channels_for_pattern(pattern: str) -> list[str]:
    if pattern == "pitch":
        return ["pitch"]
    if pattern == "roll":
        return ["roll"]
    if pattern == "diag":
        return ["pitch", "roll"]
    raise ValueError(f"Unknown channel pattern: {pattern}")


def _validate_t_switch_grid(config: ExperimentConfig, t_switch: float) -> None:
    window_s = float(config.input["window_s"])
    horizon_s = float(config.input["horizon_s"])
    k = float(t_switch) / window_s
    if not math.isclose(k, round(k), abs_tol=1e-9):
        raise ValueError(f"t_switch must align with window_s={window_s}: {t_switch}")
    if float(t_switch) <= 0.0 or float(t_switch) > horizon_s:
        raise ValueError(f"t_switch must be in (0, horizon_s]: {t_switch}")


def _terminal_window(t_switch_s: float) -> tuple[float, float]:
    return float(t_switch_s + TERMINAL_OFFSET_S[0]), float(t_switch_s + TERMINAL_OFFSET_S[1])


def _relative_subwindows(t_switch_s: float) -> list[tuple[float, float]]:
    return [(float(t_switch_s + lo), float(t_switch_s + hi)) for lo, hi in SUBWINDOW_OFFSETS_S]


def _with_xy_speed(parsed: pd.DataFrame) -> pd.DataFrame:
    df = parsed.copy()
    if "xy_speed_mps" not in df and {"vx_mps", "vy_mps"}.issubset(df.columns):
        df["xy_speed_mps"] = np.sqrt(
            pd.to_numeric(df["vx_mps"], errors="coerce") ** 2
            + pd.to_numeric(df["vy_mps"], errors="coerce") ** 2
        )
    return df


def _speed_at_time(parsed: pd.DataFrame, t_s: float) -> float:
    if not math.isfinite(float(t_s)) or "time_s" not in parsed or "xy_speed_mps" not in parsed:
        return math.nan
    times = pd.to_numeric(parsed["time_s"], errors="coerce")
    speed = pd.to_numeric(parsed["xy_speed_mps"], errors="coerce")
    valid = times.notna() & speed.notna()
    if not bool(valid.any()):
        return math.nan
    idx = (times[valid] - float(t_s)).abs().idxmin()
    return float(speed.loc[idx])


def _xy_peak(parsed: pd.DataFrame, lo: float, hi: float) -> float:
    if "time_s" not in parsed or "xy_speed_mps" not in parsed:
        return math.nan
    times = pd.to_numeric(parsed["time_s"], errors="coerce")
    speed = pd.to_numeric(parsed["xy_speed_mps"], errors="coerce")
    mask = (times >= float(lo)) & (times <= float(hi)) & speed.notna()
    if not bool(mask.any()):
        return math.nan
    return float(speed.loc[mask].max())


def _manual_abs_max(parsed: pd.DataFrame, lo: float, hi: float) -> float:
    manual_cols = [name for name in ["manual_roll", "manual_pitch", "manual_yaw", "manual_throttle"] if name in parsed]
    if not manual_cols or "time_s" not in parsed:
        return 0.0
    times = pd.to_numeric(parsed["time_s"], errors="coerce")
    mask = times >= float(lo)
    if math.isfinite(float(hi)):
        mask &= times < float(hi)
    subset = parsed.loc[mask, manual_cols]
    if subset.empty:
        return 0.0
    return float(subset.abs().max().max())


def _mode_timeline(parsed: pd.DataFrame) -> str:
    if "time_s" not in parsed or "mode" not in parsed:
        return ""
    pieces = []
    last_mode = None
    for _, row in parsed[["time_s", "mode"]].dropna().iterrows():
        mode = str(row["mode"])
        if mode != last_mode:
            pieces.append(f"{float(row['time_s']):.2f}:{mode}")
            last_mode = mode
    return "|".join(pieces[:12])


def _robustness_label(mean: float, std: float) -> str:
    if mean + 2.0 * std < 0.0:
        return "robust_violation"
    if mean - 2.0 * std > 0.0:
        return "robust_safe"
    return "noise_band"


def _spearman(xs: list[float], ys: list[float]) -> float:
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(float(x)) and math.isfinite(float(y))]
    if len(pairs) < 3:
        return math.nan
    x_arr = np.asarray([p[0] for p in pairs], dtype=float)
    y_arr = np.asarray([p[1] for p in pairs], dtype=float)
    if float(np.std(x_arr)) == 0.0 or float(np.std(y_arr)) == 0.0:
        return math.nan
    return float(np.corrcoef(_ranks(x_arr), _ranks(y_arr))[0, 1])


def _ranks(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values)
    ranks = np.empty_like(order, dtype=float)
    ranks[order] = np.arange(len(values), dtype=float)
    return ranks


def _values(rows: list[dict[str, Any]], key: str) -> list[float]:
    return [float(row.get(key, math.nan)) for row in rows]


def _mean(values: list[float]) -> float:
    arr = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    return float(np.mean(arr)) if arr.size else math.nan


def _std(values: list[float]) -> float:
    arr = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    return float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0 if arr.size == 1 else math.nan


def _min(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return float(min(finite)) if finite else math.nan


def _max(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return float(max(finite)) if finite else math.nan


def _first_finite_from_meta(meta: dict[str, Any], key: str) -> float:
    try:
        value = float(meta.get(key, math.nan))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _join_unique(values: Any) -> str:
    seen = []
    for value in values:
        text = str(value)
        if text and text not in seen:
            seen.append(text)
    return " || ".join(seen[:5])


def _parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def _float_label(value: float) -> str:
    return f"{float(value):.2f}".replace("-", "m").replace(".", "p")


def _safe_label(label: str, limit: int = 80) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", str(label))[:limit]


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(path, index=False)


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    main()
