from __future__ import annotations

import argparse
import itertools
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
from cadet.input_model import project_theta
from cadet.query import read_parsed_log, theta_hash
from cadet.runners.direction_a_probe import (
    J_REPEATS,
    _config_for_probe,
    _property_stats,
    _run_query_with_retry_count,
    _safe_label,
)
from cadet.violation_search import grid_to_theta, window_count


PREREG_PATH = Path("artifacts/cross_coupling_prereg.md")
RESIDUAL_PREREG_PATH = Path("artifacts/residual_rate_prereg.md")
DEFAULT_OUTPUT_DIR = Path("artifacts/cross_coupling_summary")
OUTPUTS = ["vx", "vy", "vz", "yaw_rate"]
INPUTS = ["roll", "pitch", "yaw", "throttle"]
NATURAL_INPUT_BY_OUTPUT = {
    "vx": "roll",
    "vy": "pitch",
    "vz": "throttle",
    "yaw_rate": "yaw",
}
NATURAL_OUTPUT_BY_INPUT = {value: key for key, value in NATURAL_INPUT_BY_OUTPUT.items()}
XY_OUTPUTS = {"vx", "vy"}
BASE_AMPLITUDE = 0.5
DELTA_PROBE = 0.2
ONSET_WINDOW = 3
DURATION_WINDOWS = 4
TERMINAL_WINDOW_RELATIVE_S = (6.0, 8.0)
STICK_LIMIT = 1.0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Step A empirical cross-coupling interaction matrix and RGA analogue."
    )
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=STICK_LIMIT)
    parser.add_argument("--base-amplitude", type=float, default=BASE_AMPLITUDE)
    parser.add_argument("--delta-probe", type=float, default=DELTA_PROBE)
    parser.add_argument("--onset-window", type=int, default=ONSET_WINDOW)
    parser.add_argument("--duration-windows", type=int, default=DURATION_WINDOWS)
    args = parser.parse_args()

    _validate_args(args)
    run_start = time.monotonic()
    output_dir = Path(args.run_dir)
    reports_dir = output_dir
    thetas_dir = output_dir / "thetas"
    reports_dir.mkdir(parents=True, exist_ok=True)
    thetas_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(PREREG_PATH, reports_dir / "cross_coupling_prereg.md")

    base_config = load_config(args.config)
    config = _config_for_probe(base_config, output_dir, float(args.stick_limit))
    scenario = config.scenario_by_id(args.scenario)
    if scenario.platform != "px4" or scenario.id != "px4_position":
        raise ValueError("Cross-coupling Step A is frozen to PX4 px4_position")
    if "post_neutral_xy_velocity" not in scenario.properties:
        raise ValueError("Scenario must include post_neutral_xy_velocity")

    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"Frozen D=40 parameterization expected 40 groups, found {len(groups)}")
    if list(config.input["channels"]) != INPUTS:
        raise ValueError(f"Frozen input channel order expected {INPUTS}, got {list(config.input['channels'])}")

    pd.DataFrame([group.__dict__ for group in groups]).to_csv(output_dir / "groups.csv", index=False)

    print(
        "cross_coupling_stepA_start "
        f"protocol=local_prereg PX4 scenario={args.scenario} seed={args.seed} "
        f"J={args.repeats} base={args.base_amplitude} delta={args.delta_probe} "
        f"mid_windows={args.onset_window}:{args.onset_window + args.duration_windows - 1} "
        f"run_dir={output_dir}",
        flush=True,
    )

    point_rows: list[dict[str, Any]] = []
    query_rows: list[dict[str, Any]] = []
    successful_query_count = 0
    timeout_retry_count = 0
    by_channel_sign: dict[tuple[str, str], dict[str, Any]] = {}

    for input_channel in INPUTS:
        for sign_label, amplitude in [
            ("minus", float(args.base_amplitude) - float(args.delta_probe)),
            ("plus", float(args.base_amplitude) + float(args.delta_probe)),
        ]:
            theta = _mid_envelope_theta(
                config,
                groups,
                input_channel,
                amplitude=amplitude,
                onset_window=int(args.onset_window),
                duration_windows=int(args.duration_windows),
            )
            row, rows, retries = _eval_signed_j5(
                theta,
                scenario,
                int(args.seed),
                output_dir,
                config,
                groups,
                input_channel=input_channel,
                sign_label=sign_label,
                amplitude=amplitude,
                repeats=int(args.repeats),
            )
            by_channel_sign[(input_channel, sign_label)] = row
            point_rows.append(row)
            query_rows.extend(rows)
            successful_query_count += int(args.repeats)
            timeout_retry_count += retries
            _write_step_a_progress(output_dir, point_rows, query_rows)

    matrix_rows, g_matrix = _interaction_matrix_rows(by_channel_sign, delta_probe=float(args.delta_probe))
    rga_rows, lambda_matrix, rga_summary = _rga_rows(g_matrix)
    candidate_rows = _candidate_rows(matrix_rows, rga_rows)
    h_rga_1 = bool(candidate_rows)

    pd.DataFrame(matrix_rows).to_csv(output_dir / "interaction_matrix.csv", index=False)
    pd.DataFrame(rga_rows).to_csv(output_dir / "rga.csv", index=False)
    pd.DataFrame(candidate_rows, columns=_candidate_columns()).to_csv(output_dir / "candidates.csv", index=False)
    _write_step_b_placeholders(output_dir)

    summary = {
        "status": "complete",
        "phase": "Step A",
        "protocol_label": "exploratory-hypothesis with frozen local protocol copy",
        "protocol_provenance_gap": (
            "artifacts/cross_coupling_prereg.md was absent at task start and was generated from "
            "the operator protocol before cross-coupling data collection; the generated horizontal "
            "natural-pair labels were corrected after an aborted initial runner start, so conclusions "
            "are labeled exploratory-hypothesis"
        ),
        "scenario_id": scenario.id,
        "platform": scenario.platform,
        "seed": int(args.seed),
        "property": "post_neutral_xy_velocity",
        "tier_scope": "Step A structural matrix only; Tier 1/Tier 2 violation evidence requires Step B",
        "repeats": int(args.repeats),
        "base_amplitude": float(args.base_amplitude),
        "delta_probe": float(args.delta_probe),
        "onset_window": int(args.onset_window),
        "duration_windows": int(args.duration_windows),
        "terminal_window_absolute_s": [11.0, 13.0],
        "output_order": OUTPUTS,
        "input_order": INPUTS,
        "natural_input_by_output": NATURAL_INPUT_BY_OUTPUT,
        "g_matrix": _matrix_to_nested_dict(g_matrix),
        "lambda_matrix": _matrix_to_nested_dict(lambda_matrix),
        "rga_summary": rga_summary,
        "cross_coupling_candidate_count": len(
            [row for row in candidate_rows if row["candidate_type"] == "cross_coupling"]
        ),
        "interaction_candidate_count": len(
            [row for row in candidate_rows if row["candidate_type"] == "interaction"]
        ),
        "h_rga_1": h_rga_1,
        "step_a_go_no_go": "go_step_b_candidates_present" if h_rga_1 else "no_go_stop_no_candidates",
        "candidates": candidate_rows,
        "successful_query_count": successful_query_count,
        "timeout_retry_count": timeout_retry_count,
        "query_attempt_count_including_timeout_retries": successful_query_count + timeout_retry_count,
        "elapsed_wall_time_s": time.monotonic() - run_start,
        "artifacts": {
            "pre_registration_copy": str(output_dir / "cross_coupling_prereg.md"),
            "groups": str(output_dir / "groups.csv"),
            "point_evaluations": str(output_dir / "stepA_point_evaluations.csv"),
            "query_repeats": str(output_dir / "stepA_query_repeats.csv"),
            "interaction_matrix": str(output_dir / "interaction_matrix.csv"),
            "rga": str(output_dir / "rga.csv"),
            "candidates": str(output_dir / "candidates.csv"),
            "arm_purity_tier1": str(output_dir / "arm_purity_tier1.csv"),
            "arm_purity_tier2": str(output_dir / "arm_purity_tier2.csv"),
            "signatures": str(output_dir / "signatures.csv"),
            "summary": str(output_dir / "cross_coupling_summary.json"),
            "report": str(output_dir / "cross_coupling_report.md"),
        },
    }
    _write_json(output_dir / "cross_coupling_summary.json", summary)
    _write_report(output_dir / "cross_coupling_report.md", summary, matrix_rows, rga_rows, candidate_rows)
    _remove_ulg_files(output_dir)

    print(
        "CROSS_COUPLING_STEP_A_VERDICT "
        f"label=exploratory-hypothesis PX4 property=post_neutral_xy_velocity "
        f"h_rga_1={h_rga_1} go_no_go={summary['step_a_go_no_go']} "
        f"cross_candidates={summary['cross_coupling_candidate_count']} "
        f"interaction_candidates={summary['interaction_candidate_count']} "
        f"report={output_dir / 'cross_coupling_report.md'}",
        flush=True,
    )


def _validate_args(args: argparse.Namespace) -> None:
    if args.scenario != "px4_position":
        raise ValueError("Cross-coupling Step A is frozen to px4_position")
    if int(args.seed) != 0:
        raise ValueError("Cross-coupling Step A is frozen to seed 0")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("Cross-coupling Step A is pre-registered to J=5 repeats")
    if float(args.stick_limit) != STICK_LIMIT:
        raise ValueError("Cross-coupling Step A is frozen to stick-limit=1.0")
    if float(args.base_amplitude) != BASE_AMPLITUDE:
        raise ValueError("Cross-coupling Step A is frozen to base amplitude 0.5")
    if float(args.delta_probe) != DELTA_PROBE:
        raise ValueError("Cross-coupling Step A is frozen to delta_probe=0.2")
    if int(args.onset_window) != ONSET_WINDOW or int(args.duration_windows) != DURATION_WINDOWS:
        raise ValueError("Cross-coupling Step A is frozen to mid-envelope windows 3..6")
    if not PREREG_PATH.exists():
        raise FileNotFoundError(f"Missing cross-coupling pre-registration artifact: {PREREG_PATH}")
    if not RESIDUAL_PREREG_PATH.exists():
        raise FileNotFoundError(f"Missing inherited residual-rate preregistration: {RESIDUAL_PREREG_PATH}")


def _mid_envelope_theta(
    config: ExperimentConfig,
    groups: list[Group],
    channel: str,
    *,
    amplitude: float,
    onset_window: int,
    duration_windows: int,
) -> np.ndarray:
    channels = list(config.input["channels"])
    if channel not in channels:
        raise ValueError(f"Unknown channel: {channel}")
    n_windows = window_count(config)
    if onset_window < 0 or onset_window + duration_windows > n_windows:
        raise ValueError("Envelope exceeds horizon")
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    grid[onset_window : onset_window + duration_windows, channels.index(channel)] = float(amplitude)
    return project_theta(grid_to_theta(grid, config, groups), config)


def _eval_signed_j5(
    theta: np.ndarray,
    scenario,
    seed: int,
    output_dir: Path,
    config: ExperimentConfig,
    groups: list[Group],
    *,
    input_channel: str,
    sign_label: str,
    amplitude: float,
    repeats: int,
) -> tuple[dict[str, Any], list[dict[str, Any]], int]:
    projected = project_theta(np.asarray(theta, dtype=float), config)
    thash = theta_hash(projected)
    signed_values: dict[str, list[float]] = {output: [] for output in OUTPUTS}
    robustness_values: dict[str, list[float]] = {prop: [] for prop in scenario.properties}
    query_rows: list[dict[str, Any]] = []
    retry_total = 0
    point_start = time.monotonic()
    for repeat_idx in range(repeats):
        cache_tag = _safe_label(
            f"cross_coupling_stepA_{input_channel}_{sign_label}_a{amplitude:.3f}_repeat{repeat_idx}"
        )
        repeat_start = time.monotonic()
        result, retry_count = _run_query_with_retry_count(
            projected,
            scenario,
            seed,
            "cross_coupling_stepA",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        retry_total += retry_count
        parsed_log = read_parsed_log(result.parsed_log_path)
        signed = _signed_terminal_residuals(parsed_log)
        for output in OUTPUTS:
            signed_values[output].append(float(signed[output]))
        for prop, value in result.robustness.items():
            robustness_values[prop].append(float(value))
        row: dict[str, Any] = {
            "input_channel": input_channel,
            "sign_label": sign_label,
            "amplitude": float(amplitude),
            "repeat_idx": repeat_idx,
            "theta_hash": result.theta_hash,
            "query_id": result.query_id,
            "cache_tag": cache_tag,
            "query_retry_count": retry_count,
            "repeat_elapsed_wall_time_s": time.monotonic() - repeat_start,
            "terminal_window_lo_s": signed["terminal_window_lo_s"],
            "terminal_window_hi_s": signed["terminal_window_hi_s"],
        }
        for output in OUTPUTS:
            row[f"signed_terminal_mean_{output}"] = float(signed[output])
        for prop, value in result.robustness.items():
            row[f"robustness_{prop}"] = float(value)
        for key, value in result.metadata.items():
            row[f"meta_{key}"] = value
        query_rows.append(row)

    signed_stats = {
        output: _mean_std_min_max(np.asarray(values, dtype=float)) for output, values in signed_values.items()
    }
    robustness_stats = _property_stats(robustness_values)
    theta_path = output_dir / "thetas" / f"stepA_{input_channel}_{sign_label}_{thash}.npy"
    np.save(theta_path, projected)
    point_row: dict[str, Any] = {
        "input_channel": input_channel,
        "sign_label": sign_label,
        "amplitude": float(amplitude),
        "theta_hash": thash,
        "theta_path": str(theta_path),
        "repeats": repeats,
        "point_elapsed_wall_time_s": time.monotonic() - point_start,
        "max_abs_theta": float(np.max(np.abs(projected))) if projected.size else 0.0,
    }
    for output, stats in signed_stats.items():
        for stat_name, value in stats.items():
            point_row[f"{output}_{stat_name}"] = value
    for prop, prop_stats in robustness_stats.items():
        for stat_name, value in prop_stats.items():
            point_row[f"rho_{stat_name}_{prop}"] = value
    print(
        "cross_coupling_stepA_eval "
        f"input={input_channel} sign={sign_label} amp={amplitude:.3f} "
        f"vx={signed_stats['vx']['mean']:.6f} vy={signed_stats['vy']['mean']:.6f} "
        f"vz={signed_stats['vz']['mean']:.6f} yaw_rate={signed_stats['yaw_rate']['mean']:.6f} "
        f"rho_xy_vel={robustness_stats['post_neutral_xy_velocity']['mean']:.6f}",
        flush=True,
    )
    return point_row, query_rows, retry_total


def _signed_terminal_residuals(parsed_log: pd.DataFrame) -> dict[str, float]:
    t_neutral = float(parsed_log["t_neutral_s"].iloc[0])
    lo = t_neutral + TERMINAL_WINDOW_RELATIVE_S[0]
    hi = t_neutral + TERMINAL_WINDOW_RELATIVE_S[1]
    mask = (parsed_log["time_s"] >= lo) & (parsed_log["time_s"] <= hi)
    if not bool(mask.any()):
        raise ValueError(f"No telemetry samples in terminal window [{lo}, {hi}]")
    yaw_rate = _yaw_rate_series(parsed_log)
    values = {
        "vx": float(parsed_log.loc[mask, "vx_mps"].mean()),
        "vy": float(parsed_log.loc[mask, "vy_mps"].mean()),
        "vz": float(parsed_log.loc[mask, "vz_mps"].mean()),
        "yaw_rate": float(np.mean(yaw_rate[mask.to_numpy()])),
        "terminal_window_lo_s": float(lo),
        "terminal_window_hi_s": float(hi),
    }
    return values


def _yaw_rate_series(parsed_log: pd.DataFrame) -> np.ndarray:
    if "yaw_rate_rps" in parsed_log:
        return parsed_log["yaw_rate_rps"].to_numpy(dtype=float)
    times = parsed_log["time_s"].to_numpy(dtype=float)
    yaw = np.unwrap(parsed_log["yaw_rad"].to_numpy(dtype=float))
    if times.size < 2:
        return np.zeros_like(times)
    return np.gradient(yaw, times)


def _mean_std_min_max(values: np.ndarray) -> dict[str, float]:
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def _interaction_matrix_rows(
    point_by_channel_sign: dict[tuple[str, str], dict[str, Any]],
    *,
    delta_probe: float,
) -> tuple[list[dict[str, Any]], np.ndarray]:
    g_matrix = np.zeros((len(OUTPUTS), len(INPUTS)), dtype=float)
    rows: list[dict[str, Any]] = []
    plus_minus: dict[tuple[str, str], tuple[dict[str, Any], dict[str, Any]]] = {}
    for input_index, input_channel in enumerate(INPUTS):
        plus = point_by_channel_sign[(input_channel, "plus")]
        minus = point_by_channel_sign[(input_channel, "minus")]
        plus_minus[(input_channel, "rows")] = (plus, minus)
        for output_index, output in enumerate(OUTPUTS):
            sensitivity = (float(plus[f"{output}_mean"]) - float(minus[f"{output}_mean"])) / (2.0 * delta_probe)
            g_matrix[output_index, input_index] = sensitivity

    row_abs_sums = np.sum(np.abs(g_matrix), axis=1)
    for output_index, output in enumerate(OUTPUTS):
        row_abs_sum = float(row_abs_sums[output_index])
        natural_input = NATURAL_INPUT_BY_OUTPUT[output]
        for input_index, input_channel in enumerate(INPUTS):
            plus, minus = plus_minus[(input_channel, "rows")]
            sensitivity = float(g_matrix[output_index, input_index])
            rows.append(
                {
                    "output": output,
                    "input": input_channel,
                    "sensitivity_G": sensitivity,
                    "row_abs_sum": row_abs_sum,
                    "row_abs_share": abs(sensitivity) / row_abs_sum if row_abs_sum > 0.0 else 0.0,
                    "natural_input": natural_input,
                    "is_natural_pair": input_channel == natural_input,
                    "plus_mean": float(plus[f"{output}_mean"]),
                    "plus_std": float(plus[f"{output}_std"]),
                    "minus_mean": float(minus[f"{output}_mean"]),
                    "minus_std": float(minus[f"{output}_std"]),
                    "delta_probe": float(delta_probe),
                }
            )
    return rows, g_matrix


def _rga_rows(g_matrix: np.ndarray) -> tuple[list[dict[str, Any]], np.ndarray, dict[str, Any]]:
    method = "exact_inverse"
    error = ""
    cond = math.inf
    lambda_matrix = np.full_like(g_matrix, np.nan, dtype=float)
    try:
        cond = float(np.linalg.cond(g_matrix))
        inverse = np.linalg.inv(g_matrix)
        lambda_matrix = g_matrix * inverse.T
    except np.linalg.LinAlgError as exc:
        method = "failed_exact_inverse"
        error = str(exc)

    rows: list[dict[str, Any]] = []
    for output_index, output in enumerate(OUTPUTS):
        natural_input = NATURAL_INPUT_BY_OUTPUT[output]
        for input_index, input_channel in enumerate(INPUTS):
            value = float(lambda_matrix[output_index, input_index])
            rows.append(
                {
                    "output": output,
                    "input": input_channel,
                    "lambda": value,
                    "natural_input": natural_input,
                    "is_natural_pair": input_channel == natural_input,
                    "is_finite": bool(np.isfinite(value)),
                }
            )
    row_sums = np.nansum(lambda_matrix, axis=1)
    col_sums = np.nansum(lambda_matrix, axis=0)
    return rows, lambda_matrix, {
        "inverse_method": method,
        "inverse_error": error,
        "condition_number": cond,
        "row_sums": {OUTPUTS[i]: float(row_sums[i]) for i in range(len(OUTPUTS))},
        "column_sums": {INPUTS[i]: float(col_sums[i]) for i in range(len(INPUTS))},
    }


def _candidate_rows(matrix_rows: list[dict[str, Any]], rga_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    lambda_by_key = {(row["output"], row["input"]): float(row["lambda"]) for row in rga_rows}
    matrix_by_key = {(row["output"], row["input"]): row for row in matrix_rows}
    rows: list[dict[str, Any]] = []
    candidate_index = 0

    for output in ["vx", "vy"]:
        natural_input = NATURAL_INPUT_BY_OUTPUT[output]
        for input_channel in INPUTS:
            if input_channel == natural_input:
                continue
            metric_row = matrix_by_key[(output, input_channel)]
            lambda_value = lambda_by_key[(output, input_channel)]
            criteria: list[str] = []
            if float(metric_row["row_abs_share"]) >= 0.20:
                criteria.append("row_abs_share>=0.20")
            if np.isfinite(lambda_value) and abs(lambda_value) >= 0.30:
                criteria.append("abs(lambda)>=0.30")
            if criteria:
                rows.append(
                    {
                        "candidate_id": f"C{candidate_index:03d}",
                        "candidate_type": "cross_coupling",
                        "input_set": input_channel,
                        "input": input_channel,
                        "output": output,
                        "natural_input": natural_input,
                        "criterion": ";".join(criteria),
                        "row_abs_share": float(metric_row["row_abs_share"]),
                        "lambda": lambda_value,
                        "max_abs_lambda": abs(lambda_value) if np.isfinite(lambda_value) else math.nan,
                        "min_lambda": lambda_value if np.isfinite(lambda_value) else math.nan,
                        "triggered_entries": f"{output}:{input_channel}={lambda_value:.6g}",
                        "h_rga_1_component": True,
                        "step_b_status": "not_run_stepA_gate",
                    }
                )
                candidate_index += 1

    for input_a, input_b in itertools.combinations(INPUTS, 2):
        inspected_entries: list[tuple[str, str, float]] = []
        for output in [NATURAL_OUTPUT_BY_INPUT[input_a], NATURAL_OUTPUT_BY_INPUT[input_b]]:
            for input_channel in [input_a, input_b]:
                value = lambda_by_key[(output, input_channel)]
                inspected_entries.append((output, input_channel, value))
        finite_values = [value for _, _, value in inspected_entries if np.isfinite(value)]
        if not finite_values:
            continue
        triggered = [
            (output, input_channel, value)
            for output, input_channel, value in inspected_entries
            if np.isfinite(value) and (abs(value) >= 1.5 or value <= 0.0)
        ]
        if triggered:
            rows.append(
                {
                    "candidate_id": f"C{candidate_index:03d}",
                    "candidate_type": "interaction",
                    "input_set": f"{input_a}+{input_b}",
                    "input": "",
                    "output": "",
                    "natural_input": "",
                    "criterion": "any relevant lambda abs>=1.5 or <=0",
                    "row_abs_share": math.nan,
                    "lambda": math.nan,
                    "max_abs_lambda": float(max(abs(value) for value in finite_values)),
                    "min_lambda": float(min(finite_values)),
                    "triggered_entries": ";".join(
                        f"{output}:{input_channel}={value:.6g}" for output, input_channel, value in triggered
                    ),
                    "h_rga_1_component": True,
                    "step_b_status": "not_run_stepA_gate",
                }
            )
            candidate_index += 1

    return rows


def _candidate_columns() -> list[str]:
    return [
        "candidate_id",
        "candidate_type",
        "input_set",
        "input",
        "output",
        "natural_input",
        "criterion",
        "row_abs_share",
        "lambda",
        "max_abs_lambda",
        "min_lambda",
        "triggered_entries",
        "h_rga_1_component",
        "step_b_status",
    ]


def _write_step_b_placeholders(output_dir: Path) -> None:
    purity_columns = [
        "candidate_id",
        "input_set",
        "arm",
        "points_evaluated",
        "interior_robust_violations",
        "channel_pure_interior_robust_violations",
        "channel_pure_ratio",
        "status",
    ]
    signature_columns = [
        "candidate_id",
        "input_set",
        "tier",
        "signature",
        "support_channels",
        "max_abs_theta",
        "rho_mean_post_neutral_xy_velocity",
        "rho_std_post_neutral_xy_velocity",
        "tier1_class",
        "tier2_class",
        "status",
    ]
    pd.DataFrame(columns=purity_columns).to_csv(output_dir / "arm_purity_tier1.csv", index=False)
    pd.DataFrame(columns=purity_columns).to_csv(output_dir / "arm_purity_tier2.csv", index=False)
    pd.DataFrame(columns=signature_columns).to_csv(output_dir / "signatures.csv", index=False)


def _write_step_a_progress(output_dir: Path, point_rows: list[dict[str, Any]], query_rows: list[dict[str, Any]]) -> None:
    pd.DataFrame(point_rows).to_csv(output_dir / "stepA_point_evaluations.csv", index=False)
    pd.DataFrame(query_rows).to_csv(output_dir / "stepA_query_repeats.csv", index=False)


def _write_report(
    path: Path,
    summary: dict[str, Any],
    matrix_rows: list[dict[str, Any]],
    rga_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
) -> None:
    lines = [
        "# Cross-coupling Residual-rate Step A",
        "",
        "Scope: `exploratory-hypothesis`, PX4, `post_neutral_xy_velocity`, Step A structural matrix.",
        "",
        "Protocol provenance: `artifacts/cross_coupling_prereg.md` was absent at task start; a local copy was "
        "generated from the operator protocol before collecting cross-coupling data. This is a data gap for "
        "`confirmatory-protocol` labeling. A generated horizontal natural-pair label was also corrected after "
        "an aborted initial runner start; measured G values are unchanged by that label correction, but all "
        "conclusions here are labeled `exploratory-hypothesis`.",
        "",
        f"Step A go/no-go: **{summary['step_a_go_no_go']}**.",
        f"H-RGA-1: `{summary['h_rga_1']}`.",
        f"Cross-coupling candidates: `{summary['cross_coupling_candidate_count']}`.",
        f"Interaction candidates: `{summary['interaction_candidate_count']}`.",
        f"RGA inverse method: `{summary['rga_summary']['inverse_method']}`; "
        f"condition number: `{summary['rga_summary']['condition_number']:.6g}`.",
        "",
        "G matrix, signed terminal residual sensitivity:",
        "",
        _markdown_matrix(matrix_rows, "sensitivity_G"),
        "",
        "G row-normalized absolute mass share:",
        "",
        _markdown_matrix(matrix_rows, "row_abs_share"),
        "",
        "RGA analogue Lambda:",
        "",
        _markdown_matrix(rga_rows, "lambda"),
        "",
        "Candidates:",
        "",
    ]
    if candidate_rows:
        lines.extend(
            [
                "| id | type | input set | output | criterion | share | lambda/max | triggered entries |",
                "| --- | --- | --- | --- | --- | ---: | ---: | --- |",
            ]
        )
        for row in candidate_rows:
            lambda_or_max = row["lambda"]
            if isinstance(lambda_or_max, float) and math.isnan(lambda_or_max):
                lambda_or_max = row["max_abs_lambda"]
            share = row["row_abs_share"]
            share_text = "" if isinstance(share, float) and math.isnan(share) else f"{float(share):.6f}"
            lines.append(
                f"| {row['candidate_id']} | {row['candidate_type']} | {row['input_set']} | "
                f"{row['output']} | {row['criterion']} | {share_text} | {float(lambda_or_max):.6f} | "
                f"{row['triggered_entries']} |"
            )
    else:
        lines.append("No Step A candidates.")

    lines.extend(
        [
            "",
            "Step B status: not run in this Step A go/no-go pass.",
            "",
            "Artifacts:",
            "",
        ]
    )
    for key, value in summary["artifacts"].items():
        lines.append(f"- {key}: `{value}`")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _markdown_matrix(rows: list[dict[str, Any]], value_key: str) -> str:
    by_key = {(row["output"], row["input"]): row for row in rows}
    lines = [
        "| output \\ input | roll | pitch | yaw | throttle |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for output in OUTPUTS:
        values: list[str] = []
        for input_channel in INPUTS:
            value = float(by_key[(output, input_channel)][value_key])
            if math.isnan(value):
                values.append("nan")
            else:
                values.append(f"{value:.6f}")
        lines.append(f"| {output} | " + " | ".join(values) + " |")
    return "\n".join(lines)


def _matrix_to_nested_dict(matrix: np.ndarray) -> dict[str, dict[str, float]]:
    return {
        output: {input_channel: float(matrix[i, j]) for j, input_channel in enumerate(INPUTS)}
        for i, output in enumerate(OUTPUTS)
    }


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _remove_ulg_files(path: Path) -> None:
    for ulg_path in Path(path).rglob("*.ulg"):
        ulg_path.unlink()


if __name__ == "__main__":
    main()
