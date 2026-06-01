from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import pandas as pd

from sparsepilot.config import ExperimentConfig, load_config
from sparsepilot.groups import Group, build_groups
from sparsepilot.input_model import project_theta
from sparsepilot.query import theta_hash
from sparsepilot.runners.fd_snapshot import _run_query_with_retry


PROPERTY_CEILING_KEYS = {
    "post_neutral_xy_drift": "d_max_m",
    "post_neutral_alt_drift": "h_max_m",
    "post_neutral_xy_velocity": "v_max_mps",
}


@dataclass(frozen=True)
class Candidate:
    index: int
    label: str
    source: str
    theta: np.ndarray
    parent_index: int | None = None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Coarse full-amplitude violation-boundary search. This is a base-finding "
            "probe only; it does not perform FD, sparse probing, or active-set logic."
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default="runs/rq1_boundary_v0")
    parser.add_argument("--random-count", type=int, default=96)
    parser.add_argument("--rng-seed", type=int, default=20260529)
    parser.add_argument("--verify-repeats", type=int, default=5)
    parser.add_argument("--refine-top", type=int, default=3)
    parser.add_argument("--refine-steps", type=int, default=4)
    parser.add_argument("--saturation-tol", type=float, default=0.02)
    args = parser.parse_args()

    output_dir = Path(args.run_dir)
    config = _config_for_run_dir(load_config(args.config), output_dir)
    scenario = config.scenario_by_id(args.scenario)
    if scenario.id != "px4_position":
        raise ValueError("Step C is frozen to px4_position only")
    if int(args.seed) != 0:
        raise ValueError("Step C is frozen to seed 0 until boundary-SPH is reviewed")

    output_dir.mkdir(parents=True, exist_ok=True)
    reports_dir = output_dir / "reports"
    candidates_dir = output_dir / "candidates"
    reports_dir.mkdir(parents=True, exist_ok=True)
    candidates_dir.mkdir(parents=True, exist_ok=True)

    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    rng = np.random.default_rng(args.rng_seed)

    candidates = generate_initial_candidates(config, groups, args.random_count, rng)
    _write_candidate_plan(candidates, candidates_dir / "candidate_plan.csv")
    _write_candidate_thetas(candidates, candidates_dir / "candidate_thetas_initial.npz")

    rows: list[dict] = []
    candidate_rows_path = reports_dir / "violation_search_candidates.csv"
    print(
        f"violation_search_start scenario={scenario.id} seed={args.seed} "
        f"initial_candidates={len(candidates)}",
        flush=True,
    )
    t0 = time.monotonic()
    for candidate in candidates:
        row = evaluate_candidate(candidate, scenario, args.seed, config, output_dir)
        rows.append(row)
        _write_rows(candidate_rows_path, rows)
        print(_progress_line(row, len(rows), len(candidates)), flush=True)

    if args.refine_top > 0 and args.refine_steps > 0:
        rows_by_index = {int(row["candidate_index"]): row for row in rows}
        initial_best = sorted(rows, key=lambda r: float(r["min_robustness"]))[: args.refine_top]
        next_index = max(candidate.index for candidate in candidates) + 1
        refined: list[Candidate] = []
        print(
            f"violation_search_refine_start top={len(initial_best)} steps={args.refine_steps}",
            flush=True,
        )
        for base_row in initial_best:
            parent = next(c for c in candidates if c.index == int(base_row["candidate_index"]))
            current = parent
            current_best = float(base_row["min_robustness"])
            for step_idx in range(args.refine_steps):
                proposal = make_refinement_candidate(next_index, current, step_idx, config, groups, rng)
                next_index += 1
                refined.append(proposal)
                row = evaluate_candidate(proposal, scenario, args.seed, config, output_dir)
                rows.append(row)
                rows_by_index[proposal.index] = row
                _write_rows(candidate_rows_path, rows)
                print(_progress_line(row, len(rows), len(candidates) + len(refined)), flush=True)
                if float(row["min_robustness"]) < current_best:
                    current = proposal
                    current_best = float(row["min_robustness"])
        _write_candidate_thetas(refined, candidates_dir / "candidate_thetas_refined.npz")

    candidate_df = pd.DataFrame(rows).sort_values("min_robustness", ascending=True)
    best_row = candidate_df.iloc[0].to_dict()
    best_index = int(best_row["candidate_index"])
    all_candidates = {c.index: c for c in candidates}
    if args.refine_top > 0 and args.refine_steps > 0:
        for cand in refined:
            all_candidates[cand.index] = cand
    theta_boundary = all_candidates[best_index].theta
    np.save(output_dir / "theta_boundary.npy", theta_boundary)
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(output_dir / "groups.csv", index=False)

    print(
        f"violation_search_verify_start best_index={best_index} "
        f"single_min={float(best_row['min_robustness']):.6f} "
        f"property={best_row['min_property']}",
        flush=True,
    )
    verify_rows = verify_candidate(
        theta_boundary,
        scenario,
        args.seed,
        config,
        output_dir,
        repeats=args.verify_repeats,
        label=f"best{best_index:03d}",
    )
    verify_df = pd.DataFrame(verify_rows)
    verify_path = reports_dir / "violation_search_verify.csv"
    verify_df.to_csv(verify_path, index=False)

    verify_summary = summarize_verify(verify_df, scenario.properties, config)
    saturation = saturation_summary(theta_boundary, config, groups, args.saturation_tol)
    outcome = classify_outcome(verify_summary)
    summary = {
        "scenario_id": scenario.id,
        "seed": args.seed,
        "rng_seed": args.rng_seed,
        "initial_candidates": len(candidates),
        "total_candidates": len(rows),
        "elapsed_wall_time_s": time.monotonic() - t0,
        "best_candidate": best_row,
        "verify_summary": verify_summary,
        "outcome": outcome,
        "saturation": saturation,
        "theta_boundary": str(output_dir / "theta_boundary.npy"),
        "candidate_rows": str(candidate_rows_path),
        "verify_rows": str(verify_path),
    }
    summary_path = reports_dir / "violation_search_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    report_path = reports_dir / "violation_search_report.md"
    write_report(report_path, summary)
    print(
        f"violation_search_complete outcome={outcome['branch']} "
        f"verified_min_mean={outcome['lowest_mean_robustness']:.6f} "
        f"property={outcome['property']} report={report_path}",
        flush=True,
    )


def _config_for_run_dir(config: ExperimentConfig, output_dir: Path) -> ExperimentConfig:
    logging = dict(config.logging)
    logging["jsonl"] = str(output_dir / "logs" / "queries.jsonl")
    return replace(config, experiment_id=output_dir.name, logging=logging)


def generate_initial_candidates(
    config: ExperimentConfig, groups: list[Group], random_count: int, rng: np.random.Generator
) -> list[Candidate]:
    candidates: list[Candidate] = []
    seen: set[str] = set()

    for label, theta in hand_designed_candidates(config, groups):
        _append_unique(candidates, seen, label, "hand", theta, config)

    attempts = 0
    while len([c for c in candidates if c.source == "random"]) < random_count:
        attempts += 1
        if attempts > random_count * 20:
            raise RuntimeError("Unable to generate enough unique random feasible candidates")
        kind = attempts % 3
        if kind == 0:
            theta = random_walk_theta(config, groups, rng)
            label = f"random_walk_{attempts:03d}"
        elif kind == 1:
            theta = random_block_theta(config, groups, rng)
            label = f"random_block_{attempts:03d}"
        else:
            theta = random_projected_theta(config, groups, rng)
            label = f"random_projected_{attempts:03d}"
        _append_unique(candidates, seen, label, "random", theta, config)

    return [replace(candidate, index=i) for i, candidate in enumerate(candidates)]


def hand_designed_candidates(config: ExperimentConfig, groups: list[Group]) -> list[tuple[str, np.ndarray]]:
    max_value = float(config.input["max_value"])
    result: list[tuple[str, np.ndarray]] = []
    for channel in config.input["channels"]:
        result.append((f"sustain_{channel}_pos", theta_from_channel_values(config, groups, {channel: [max_value] * window_count(config)})))
        result.append((f"sustain_{channel}_neg", theta_from_channel_values(config, groups, {channel: [-max_value] * window_count(config)})))

    for roll_sign in [-1.0, 1.0]:
        for pitch_sign in [-1.0, 1.0]:
            label = f"diag_roll_{_sign_label(roll_sign)}_pitch_{_sign_label(pitch_sign)}"
            result.append(
                (
                    label,
                    theta_from_channel_values(
                        config,
                        groups,
                        {
                            "roll": [roll_sign * max_value] * window_count(config),
                            "pitch": [pitch_sign * max_value] * window_count(config),
                        },
                    ),
                )
            )

    for channel in ["roll", "pitch"]:
        for phase in [0, 1]:
            values = [max_value if (w + phase) % 2 == 0 else -max_value for w in range(window_count(config))]
            result.append((f"oscillate_{channel}_phase{phase}", theta_from_channel_values(config, groups, {channel: values})))

    ramp_pos = _ramp_then_hold(max_value, window_count(config), start_window=1, ramp_windows=3)
    ramp_neg = [-x for x in _ramp_then_hold(max_value, window_count(config), start_window=1, ramp_windows=3)]
    result.append(("ramp_then_hold_roll_pos", theta_from_channel_values(config, groups, {"roll": ramp_pos})))
    result.append(("ramp_then_hold_pitch_neg", theta_from_channel_values(config, groups, {"pitch": ramp_neg})))

    late_pos = [0.0] * window_count(config)
    late_neg = [0.0] * window_count(config)
    late_pos[-2:] = [max_value, max_value]
    late_neg[-2:] = [-max_value, -max_value]
    result.append(("late_step_pitch_pos_w8_w9", theta_from_channel_values(config, groups, {"pitch": late_pos})))
    result.append(("late_step_throttle_neg_w8_w9", theta_from_channel_values(config, groups, {"throttle": late_neg})))

    return result


def random_walk_theta(config: ExperimentConfig, groups: list[Group], rng: np.random.Generator) -> np.ndarray:
    channels = list(config.input["channels"])
    n_windows = window_count(config)
    max_step = float(config.input["max_delta_per_window"])
    min_value = float(config.input["min_value"])
    max_value = float(config.input["max_value"])
    values: dict[str, list[float]] = {}
    for channel in channels:
        current = 0.0
        sign = float(rng.choice([-1.0, 1.0]))
        channel_values = []
        for _ in range(n_windows):
            if rng.random() > 0.70:
                sign = float(rng.choice([-1.0, 1.0]))
            step_mag = max_step if rng.random() < 0.82 else float(rng.uniform(0.4 * max_step, max_step))
            current = float(np.clip(current + sign * step_mag, min_value, max_value))
            channel_values.append(current)
        values[channel] = channel_values
    return theta_from_channel_values(config, groups, values)


def random_block_theta(config: ExperimentConfig, groups: list[Group], rng: np.random.Generator) -> np.ndarray:
    n_windows = window_count(config)
    max_value = float(config.input["max_value"])
    values: dict[str, list[float]] = {}
    for channel in config.input["channels"]:
        channel_values = [0.0] * n_windows
        for _ in range(2):
            start = int(rng.integers(0, n_windows))
            max_width = min(6, n_windows - start)
            width = 1 if max_width < 2 else int(rng.integers(2, max_width + 1))
            amp = float(rng.beta(4.0, 1.0) * max_value)
            sign = float(rng.choice([-1.0, 1.0]))
            for window_id in range(start, min(n_windows, start + width)):
                channel_values[window_id] = sign * amp
        values[channel] = channel_values
    return theta_from_channel_values(config, groups, values)


def random_projected_theta(config: ExperimentConfig, groups: list[Group], rng: np.random.Generator) -> np.ndarray:
    n_windows = window_count(config)
    max_value = float(config.input["max_value"])
    values: dict[str, list[float]] = {}
    for channel in config.input["channels"]:
        amps = rng.beta(3.0, 1.0, size=n_windows) * max_value
        signs = rng.choice([-1.0, 1.0], size=n_windows)
        values[channel] = list((amps * signs).astype(float))
    return theta_from_channel_values(config, groups, values)


def make_refinement_candidate(
    index: int,
    parent: Candidate,
    step_idx: int,
    config: ExperimentConfig,
    groups: list[Group],
    rng: np.random.Generator,
) -> Candidate:
    grid = theta_to_grid(parent.theta, config, groups)
    channels = list(config.input["channels"])
    channel = channels[(step_idx + int(rng.integers(0, len(channels)))) % len(channels)]
    channel_idx = channels.index(channel)
    n_windows = window_count(config)
    max_value = float(config.input["max_value"])
    sign = float(rng.choice([-1.0, 1.0]))
    pattern = step_idx % 4
    if pattern == 0:
        grid[:, channel_idx] = sign * max_value
        label = f"refine_parent{parent.index:03d}_{channel}_all_{_sign_label(sign)}"
    elif pattern == 1:
        grid[n_windows // 2 :, channel_idx] = sign * max_value
        label = f"refine_parent{parent.index:03d}_{channel}_latehalf_{_sign_label(sign)}"
    elif pattern == 2:
        start = max(0, n_windows - 2)
        grid[start:, channel_idx] = sign * max_value
        label = f"refine_parent{parent.index:03d}_{channel}_w8w9_{_sign_label(sign)}"
    else:
        start = int(rng.integers(0, max(1, n_windows - 3)))
        grid[start : start + 3, channel_idx] = sign * max_value
        label = f"refine_parent{parent.index:03d}_{channel}_block{start}_{_sign_label(sign)}"
    theta = project_theta(grid_to_theta(grid, config, groups), config)
    return Candidate(index=index, label=label, source="refine", theta=theta, parent_index=parent.index)


def theta_from_channel_values(
    config: ExperimentConfig, groups: list[Group], channel_values: dict[str, list[float]]
) -> np.ndarray:
    channels = list(config.input["channels"])
    n_windows = window_count(config)
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    for channel, values in channel_values.items():
        if channel not in channels:
            continue
        if len(values) != n_windows:
            raise ValueError(f"{channel} has {len(values)} windows, expected {n_windows}")
        grid[:, channels.index(channel)] = np.asarray(values, dtype=float)
    return project_theta(grid_to_theta(grid, config, groups), config)


def theta_to_grid(theta: np.ndarray, config: ExperimentConfig, groups: list[Group]) -> np.ndarray:
    channels = list(config.input["channels"])
    grid = np.zeros((window_count(config), len(channels)), dtype=float)
    for group in groups:
        grid[group.window_id, channels.index(group.channel)] = float(theta[group.group_id])
    return grid


def grid_to_theta(grid: np.ndarray, config: ExperimentConfig, groups: list[Group]) -> np.ndarray:
    channels = list(config.input["channels"])
    theta = np.zeros(len(groups), dtype=float)
    for group in groups:
        theta[group.group_id] = float(grid[group.window_id, channels.index(group.channel)])
    return theta


def evaluate_candidate(candidate: Candidate, scenario, seed: int, config: ExperimentConfig, output_dir: Path) -> dict:
    cache_tag = f"violation_search_{candidate.index:03d}_{_safe_label(candidate.label)}"
    start = time.monotonic()
    result = _run_query_with_retry(
        candidate.theta,
        scenario,
        seed,
        "violation_search",
        output_dir,
        config,
        cache_tag=cache_tag,
        use_cache=True,
    )
    elapsed = time.monotonic() - start
    min_property = min(result.robustness, key=lambda prop: float(result.robustness[prop]))
    row = {
        "candidate_index": candidate.index,
        "source": candidate.source,
        "label": candidate.label,
        "parent_index": "" if candidate.parent_index is None else candidate.parent_index,
        "theta_hash": result.theta_hash,
        "query_id": result.query_id,
        "cache_tag": cache_tag,
        "min_property": min_property,
        "min_robustness": float(result.robustness[min_property]),
        "eval_elapsed_wall_time_s": elapsed,
        "query_total_wall_time_s": float(result.metadata.get("total_wall_time_s", math.nan)),
    }
    for prop, value in result.robustness.items():
        row[f"robustness_{prop}"] = float(value)
    return row


def verify_candidate(
    theta: np.ndarray,
    scenario,
    seed: int,
    config: ExperimentConfig,
    output_dir: Path,
    *,
    repeats: int,
    label: str,
) -> list[dict]:
    rows = []
    for repeat_idx in range(repeats):
        cache_tag = f"violation_search_verify_{label}_repeat{repeat_idx}"
        result = _run_query_with_retry(
            theta,
            scenario,
            seed,
            "violation_search_verify",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        row = {
            "repeat_idx": repeat_idx,
            "theta_hash": result.theta_hash,
            "query_id": result.query_id,
            "cache_tag": cache_tag,
            "query_total_wall_time_s": float(result.metadata.get("total_wall_time_s", math.nan)),
        }
        for prop, value in result.robustness.items():
            row[f"robustness_{prop}"] = float(value)
        rows.append(row)
        print(
            "violation_search_verify "
            f"repeat={repeat_idx}/{repeats - 1} "
            + " ".join(f"{prop}={result.robustness[prop]:.6f}" for prop in scenario.properties),
            flush=True,
        )
    return rows


def summarize_verify(verify_df: pd.DataFrame, properties: list[str], config: ExperimentConfig) -> list[dict]:
    rows = []
    for prop in properties:
        col = f"robustness_{prop}"
        values = verify_df[col].to_numpy(dtype=float)
        ceiling = property_ceiling(prop, config)
        rows.append(
            {
                "property": prop,
                "ceiling": ceiling,
                "half_ceiling": 0.5 * ceiling,
                "mean": float(np.mean(values)),
                "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                "min": float(np.min(values)),
                "max": float(np.max(values)),
                "count_nonpositive": int(np.sum(values <= 0.0)),
                "repeats": int(len(values)),
            }
        )
    return rows


def classify_outcome(verify_summary: list[dict]) -> dict:
    lowest = min(verify_summary, key=lambda row: float(row["mean"]))
    if float(lowest["mean"]) <= 0.0:
        branch = "true_violation"
        description = "true violation reached: verified mean robustness is nonpositive for at least one property"
    elif float(lowest["mean"]) < float(lowest["half_ceiling"]):
        branch = "near_boundary"
        description = "near-boundary reached: verified mean robustness is below 0.5x the property ceiling"
    else:
        branch = "no_violation_reachable"
        description = "no violation reachable in this coarse full-amplitude search"
    return {
        "branch": branch,
        "description": description,
        "property": lowest["property"],
        "lowest_mean_robustness": float(lowest["mean"]),
        "half_ceiling": float(lowest["half_ceiling"]),
        "count_nonpositive": int(lowest["count_nonpositive"]),
        "repeats": int(lowest["repeats"]),
    }


def saturation_summary(theta: np.ndarray, config: ExperimentConfig, groups: list[Group], tol: float) -> dict:
    theta = np.asarray(theta, dtype=float)
    min_value = float(config.input["min_value"])
    max_value = float(config.input["max_value"])
    delta = float(config.input["perturb_delta"])
    near_upper = theta >= max_value - tol
    near_lower = theta <= min_value + tol
    value_margin_lt_delta = (theta > max_value - delta) | (theta < min_value + delta)

    clean_groups = []
    blocked_groups = []
    for group in groups:
        raw_plus = theta.copy()
        raw_minus = theta.copy()
        raw_plus[group.group_id] += delta
        raw_minus[group.group_id] -= delta
        plus_clean = np.allclose(project_theta(raw_plus, config), raw_plus, atol=1e-9, rtol=0.0)
        minus_clean = np.allclose(project_theta(raw_minus, config), raw_minus, atol=1e-9, rtol=0.0)
        if plus_clean and minus_clean:
            clean_groups.append(group.group_id)
        else:
            blocked_groups.append(group.group_id)

    near_limit_count = int(np.sum(near_upper | near_lower))
    saturated_threshold = max(8, int(math.ceil(0.25 * len(theta))))
    return {
        "near_value_limit_tolerance": tol,
        "near_upper_count": int(np.sum(near_upper)),
        "near_lower_count": int(np.sum(near_lower)),
        "near_abs_limit_count": near_limit_count,
        "value_margin_less_than_delta_count": int(np.sum(value_margin_lt_delta)),
        "fd_clean_two_sided_groups": len(clean_groups),
        "fd_blocked_groups": len(blocked_groups),
        "fd_blocked_group_ids": ",".join(str(int(g)) for g in blocked_groups),
        "amplitude_saturated": near_limit_count >= saturated_threshold,
        "fd_interior": len(clean_groups) == len(theta),
        "max_abs_theta": float(np.max(np.abs(theta))) if theta.size else 0.0,
        "mean_abs_theta": float(np.mean(np.abs(theta))) if theta.size else 0.0,
    }


def property_ceiling(prop: str, config: ExperimentConfig) -> float:
    key = PROPERTY_CEILING_KEYS[prop]
    return float(config.properties[prop][key])


def window_count(config: ExperimentConfig) -> int:
    return int(round(float(config.input["horizon_s"]) / float(config.input["window_s"])))


def _append_unique(
    candidates: list[Candidate],
    seen: set[str],
    label: str,
    source: str,
    theta: np.ndarray,
    config: ExperimentConfig,
) -> None:
    projected = project_theta(theta, config)
    thash = theta_hash(projected)
    if thash in seen:
        return
    seen.add(thash)
    candidates.append(Candidate(index=-1, label=label, source=source, theta=projected))


def _ramp_then_hold(max_value: float, n_windows: int, *, start_window: int, ramp_windows: int) -> list[float]:
    values = [0.0] * n_windows
    for window_id in range(start_window, n_windows):
        ramp_pos = min(1.0, (window_id - start_window + 1) / float(ramp_windows))
        values[window_id] = ramp_pos * max_value
    return values


def _sign_label(sign: float) -> str:
    return "pos" if sign > 0 else "neg"


def _safe_label(label: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", label)[:80]


def _write_candidate_plan(candidates: list[Candidate], path: Path) -> None:
    rows = [
        {
            "candidate_index": candidate.index,
            "source": candidate.source,
            "label": candidate.label,
            "theta_hash": theta_hash(candidate.theta),
            "max_abs_theta": float(np.max(np.abs(candidate.theta))) if candidate.theta.size else 0.0,
            "mean_abs_theta": float(np.mean(np.abs(candidate.theta))) if candidate.theta.size else 0.0,
        }
        for candidate in candidates
    ]
    pd.DataFrame(rows).to_csv(path, index=False)


def _write_candidate_thetas(candidates: list[Candidate], path: Path) -> None:
    if not candidates:
        return
    arrays = {f"candidate_{candidate.index:03d}": candidate.theta for candidate in candidates}
    labels = {
        str(candidate.index): {
            "source": candidate.source,
            "label": candidate.label,
            "theta_hash": theta_hash(candidate.theta),
            "parent_index": candidate.parent_index,
        }
        for candidate in candidates
    }
    arrays["labels_json"] = np.asarray(json.dumps(labels, sort_keys=True))
    np.savez_compressed(path, **arrays)


def _write_rows(path: Path, rows: list[dict]) -> None:
    pd.DataFrame(rows).to_csv(path, index=False)


def _progress_line(row: dict, done: int, total: int) -> str:
    return (
        f"violation_search_candidate {done}/{total} "
        f"idx={int(row['candidate_index']):03d} source={row['source']} "
        f"min={float(row['min_robustness']):.6f} prop={row['min_property']} "
        f"label={row['label']}"
    )


def write_report(report_path: Path, summary: dict) -> None:
    best = summary["best_candidate"]
    saturation = summary["saturation"]
    outcome = summary["outcome"]
    lines = [
        "# Boundary Violation Search Report",
        "",
        "Scope: `px4_position`, seed 0, full-amplitude feasible theta search over D=40. "
        "This is a coarse base-finding probe only; it does not use FD, sparse probing, active sets, "
        "gradient reuse, or persistence-gated logic.",
        "",
        f"Outcome branch: **{outcome['branch']}**. {outcome['description']}.",
        "",
        "## Best Search Candidate",
        "",
        "| index | source | label | single-query min property | single-query min rho | theta hash |",
        "| ---: | --- | --- | --- | ---: | --- |",
        (
            f"| {int(best['candidate_index'])} | {best['source']} | {best['label']} | "
            f"{best['min_property']} | {float(best['min_robustness']):.6f} | {best['theta_hash']} |"
        ),
        "",
        "## J=5 Verification",
        "",
        "| property | mean rho | std | min | max | nonpositive repeats | ceiling | 0.5x ceiling |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary["verify_summary"]:
        lines.append(
            f"| {row['property']} | {row['mean']:.6f} | {row['std']:.6f} | {row['min']:.6f} | "
            f"{row['max']:.6f} | {row['count_nonpositive']}/{row['repeats']} | "
            f"{row['ceiling']:.3f} | {row['half_ceiling']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Saturation And FD Interior Check",
            "",
            "| near abs limit | margin < delta | clean two-sided FD groups | blocked groups | max abs theta | mean abs theta | saturated | FD interior |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
            (
                f"| {saturation['near_abs_limit_count']} | {saturation['value_margin_less_than_delta_count']} | "
                f"{saturation['fd_clean_two_sided_groups']} | {saturation['fd_blocked_groups']} | "
                f"{saturation['max_abs_theta']:.3f} | {saturation['mean_abs_theta']:.3f} | "
                f"{saturation['amplitude_saturated']} | {saturation['fd_interior']} |"
            ),
            "",
        ]
    )
    if not saturation["fd_interior"]:
        lines.extend(
            [
                "The selected base is not cleanly two-sided-interior for all FD groups under the frozen "
                "`perturb_delta=0.08`; blocked group ids are:",
                "",
                f"`{saturation['fd_blocked_group_ids']}`",
                "",
            ]
        )
    lines.extend(
        [
            "## Artifacts",
            "",
            f"- theta_boundary: `{summary['theta_boundary']}`",
            f"- candidate rows: `{summary['candidate_rows']}`",
            f"- verification rows: `{summary['verify_rows']}`",
            f"- summary JSON: `{report_path.with_name('violation_search_summary.json')}`",
            "",
            f"Elapsed wall time: {summary['elapsed_wall_time_s']:.1f}s across {summary['total_candidates']} search candidates plus verification.",
            "",
            "Stop point: wait for author confirmation before Step D boundary FD/persistence.",
            "",
        ]
    )
    report_path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
