from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import pandas as pd

from sparsepilot.config import load_config
from sparsepilot.groups import Group, build_groups
from sparsepilot.input_model import perturb_group, zero_theta
from sparsepilot.metrics import effective_sparsity, topk_coverage
from sparsepilot.plots import plot_gradient_heatmap
from sparsepilot.query import QueryResult, read_parsed_log, run_query, theta_hash


PRIMARY_PROPERTIES = {
    "px4_position": {"post_neutral_xy_drift", "post_neutral_alt_drift", "post_neutral_xy_velocity"},
    "px4_hold": {"post_neutral_xy_drift", "post_neutral_alt_drift", "post_neutral_xy_velocity"},
    "ap_loiter": {"post_neutral_alt_drift"},
    "ap_althold": {"post_neutral_alt_drift"},
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run-dir", default=None)
    parser.add_argument("--theta", default="zero", choices=["zero"])
    parser.add_argument("--phase-tag", default="phase2a_seed0")
    args = parser.parse_args()

    config = load_config(args.config)
    scenario = config.scenario_by_id(args.scenario)
    output_dir = Path(args.run_dir) if args.run_dir else Path("runs") / config.experiment_id
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = zero_theta(groups)
    run_snapshot(theta, scenario, args.seed, config, output_dir, groups, args.phase_tag)


def run_snapshot(theta, scenario, seed: int, config, output_dir: Path, groups: list[Group], phase_tag: str) -> Path:
    output_dir = Path(output_dir)
    delta = float(config.input["perturb_delta"])
    snapshot_id = f"{scenario.id}_seed{seed}_{theta_hash(np.asarray(theta, dtype=float))}_{phase_tag}"
    snap_dir = output_dir / "snapshots" / snapshot_id
    snap_dir.mkdir(parents=True, exist_ok=True)
    figures_dir = output_dir / "figures" / phase_tag
    figures_dir.mkdir(parents=True, exist_ok=True)

    np.save(snap_dir / "theta.npy", np.asarray(theta, dtype=float))
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(snap_dir / "groups.csv", index=False)

    gradients = {prop: np.zeros(len(groups), dtype=float) for prop in scenario.properties}
    query_rows = []
    t0 = time.monotonic()
    for group in groups:
        print(
            f"fd_snapshot {scenario.id} seed={seed} g{group.group_id} "
            f"{group.channel}@{group.t_start:.1f}-{group.t_end:.1f}",
            flush=True,
        )
        theta_plus = perturb_group(theta, group.group_id, delta, +1, config)
        theta_minus = perturb_group(theta, group.group_id, delta, -1, config)
        plus = _run_query_with_retry(theta_plus, scenario, seed, "fd_plus", output_dir, config)
        minus = _run_query_with_retry(theta_minus, scenario, seed, "fd_minus", output_dir, config)
        query_rows.append(_query_row(plus, group, "+", scenario, seed))
        query_rows.append(_query_row(minus, group, "-", scenario, seed))
        for prop in scenario.properties:
            gradients[prop][group.group_id] = (plus.robustness[prop] - minus.robustness[prop]) / (2.0 * delta)

    metric_rows = []
    primary_props = PRIMARY_PROPERTIES.get(scenario.id, set(scenario.properties))
    for prop, values in gradients.items():
        rows = []
        for group in groups:
            value = float(values[group.group_id])
            rows.append({**group.__dict__, "g": value, "abs_g": abs(value)})
        gradient_csv = snap_dir / f"gradient_{prop}.csv"
        pd.DataFrame(rows).to_csv(gradient_csv, index=False)
        fig_path = figures_dir / f"heatmap_{scenario.id}_seed{seed}_{prop}.png"
        plot_gradient_heatmap(gradient_csv, fig_path)
        abs_g = np.abs(values)
        metric_rows.append(
            {
                "scenario_id": scenario.id,
                "seed": seed,
                "property": prop,
                "role": "primary" if prop in primary_props else "diagnostic",
                "top4_coverage": topk_coverage(abs_g, 4),
                "top8_coverage": topk_coverage(abs_g, 8),
                "effective_sparsity": effective_sparsity(values),
                "max_abs_g": float(np.max(abs_g)) if abs_g.size else 0.0,
                "sum_abs_g": float(np.sum(abs_g)),
                "heatmap": str(fig_path),
            }
        )

    pd.DataFrame(query_rows).to_csv(snap_dir / "query_metadata.csv", index=False)
    pd.DataFrame(metric_rows).to_csv(snap_dir / "snapshot_metrics.csv", index=False)
    metadata = {
        "snapshot_id": snapshot_id,
        "scenario_id": scenario.id,
        "seed": seed,
        "delta": delta,
        "phase_tag": phase_tag,
        "properties": list(scenario.properties),
        "primary_properties": sorted(primary_props),
        "elapsed_wall_time_s": time.monotonic() - t0,
    }
    (snap_dir / "snapshot_metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
    print(f"snapshot_complete {snapshot_id} elapsed={metadata['elapsed_wall_time_s']:.1f}s", flush=True)
    return snap_dir


def _run_query_with_retry(
    theta,
    scenario,
    seed: int,
    query_type: str,
    output_dir: Path,
    config,
    *,
    cache_tag: str | None = None,
    use_cache: bool = True,
) -> QueryResult:
    max_attempts = int(config.simulator.get(scenario.platform, {}).get("query_timeout_retries", 2)) + 1
    for attempt in range(1, max_attempts + 1):
        try:
            return run_query(theta, scenario, seed, query_type, output_dir, config, use_cache=use_cache, cache_tag=cache_tag)
        except TimeoutError as exc:
            if attempt >= max_attempts:
                raise
            print(
                f"query_retry scenario={scenario.id} seed={seed} type={query_type} "
                f"attempt={attempt}/{max_attempts} error={exc}",
                flush=True,
            )
            time.sleep(2.0)
    raise RuntimeError("unreachable query retry state")


def _query_row(result: QueryResult, group: Group, sign: str, scenario, seed: int) -> dict:
    parsed = read_parsed_log(result.parsed_log_path)
    validation = _validate_parsed_log(parsed)
    row = {
        "scenario_id": scenario.id,
        "seed": seed,
        "query_id": result.query_id,
        "theta_hash": result.theta_hash,
        "group_id": group.group_id,
        "channel": group.channel,
        "window_id": group.window_id,
        "t_start": group.t_start,
        "t_end": group.t_end,
        "sign": sign,
        "parsed_log_path": str(result.parsed_log_path),
        **{f"robustness_{k}": v for k, v in result.robustness.items()},
        **{f"meta_{k}": v for k, v in result.metadata.items()},
        **validation,
    }
    return row


def _validate_parsed_log(parsed: pd.DataFrame) -> dict:
    t_zero = float(parsed["t_zero_s"].iloc[0])
    t_neutral = float(parsed["t_neutral_s"].iloc[0])
    tail_settle_s = 0.5
    manual_cols = [c for c in ["manual_roll", "manual_pitch", "manual_yaw", "manual_throttle"] if c in parsed]
    perturb = parsed[(parsed["time_s"] >= t_zero) & (parsed["time_s"] < t_neutral)]
    tail = parsed[parsed["time_s"] >= t_neutral]
    settled_tail = parsed[parsed["time_s"] >= t_neutral + tail_settle_s]
    result = {
        "manual_perturb_abs_max": float(perturb[manual_cols].abs().max().max()) if manual_cols and not perturb.empty else 0.0,
        "manual_tail_abs_max": float(tail[manual_cols].abs().max().max()) if manual_cols and not tail.empty else 0.0,
        "tail_settle_s": tail_settle_s,
        "tail_mode_values": ",".join(sorted(str(x) for x in tail["mode"].dropna().unique())) if "mode" in tail else "",
        "settled_tail_mode_values": ",".join(sorted(str(x) for x in settled_tail["mode"].dropna().unique()))
        if "mode" in settled_tail
        else "",
    }
    for col in ["base_mode", "custom_mode", "px4_main_mode", "px4_sub_mode"]:
        if col in tail and not tail.empty:
            values = sorted(set(int(x) for x in tail[col].dropna().tolist()))
            result[f"tail_{col}_values"] = ",".join(str(x) for x in values)
        else:
            result[f"tail_{col}_values"] = ""
        if col in settled_tail and not settled_tail.empty:
            values = sorted(set(int(x) for x in settled_tail[col].dropna().tolist()))
            result[f"settled_tail_{col}_values"] = ",".join(str(x) for x in values)
        else:
            result[f"settled_tail_{col}_values"] = ""
    return result


if __name__ == "__main__":
    main()
