from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from cadet.config import load_config
from cadet.groups import Group, build_groups
from cadet.input_model import perturb_group, zero_theta
from cadet.query import read_parsed_log, run_query


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run-dir", default=None)
    parser.add_argument("--skip-probes", action="store_true")
    parser.add_argument("--probes-only", action="store_true")
    parser.add_argument("--artifact-tag", default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    scenario = config.scenario_by_id(args.scenario)
    output_dir = Path(args.run_dir) if args.run_dir else Path("runs") / config.experiment_id
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta0 = zero_theta(groups)

    print(f"scenario={scenario.id} seed={args.seed} platform={scenario.platform}")
    smoke_dir = output_dir / "smoke" / f"{scenario.id}_seed{args.seed}"
    smoke_dir.mkdir(parents=True, exist_ok=True)
    repeat_results = []
    previous_summary = _load_previous_summary(smoke_dir)
    if args.probes_only and previous_summary:
        robustness_stats = previous_summary["robustness_stats"]
        print("using existing nominal robustness stats")
    else:
        for repeat in range(3):
            result = run_query(
                theta0,
                scenario,
                args.seed,
                "smoke_nominal",
                output_dir,
                config,
                use_cache=False,
                cache_tag=f"nominal_repeat{repeat}",
            )
            repeat_results.append(result)
            print(
                f"nominal_repeat{repeat}: {json.dumps(result.robustness, sort_keys=True)} "
                f"wall={result.metadata.get('wall_time_s', 0):.2f}s"
            )
        robustness_stats = _robustness_stats(repeat_results)
    for prop, stats in robustness_stats.items():
        print(
            f"noise {prop}: mean={stats['mean']:.6f} std={stats['std']:.6f} "
            f"rel_std={stats['rel_std']:.3%}"
        )

    cache_first = run_query(theta0, scenario, args.seed, "smoke_cache_refresh", output_dir, config, use_cache=False)
    start = time.monotonic()
    cache_second = run_query(theta0, scenario, args.seed, "smoke_cache", output_dir, config)
    cache_hit_wall_s = time.monotonic() - start
    print(f"cache_second_wall_s={cache_hit_wall_s:.3f} query_id={cache_second.query_id}")

    figures_dir = output_dir / "figures"
    default_log_path = repeat_results[0].parsed_log_path if repeat_results else cache_first.parsed_log_path
    default_log = read_parsed_log(default_log_path)
    _plot_manual_and_mode(default_log, figures_dir / f"smoke_manual_mode_{scenario.id}_seed{args.seed}_zero.png")

    probe_rows = []
    if not args.skip_probes:
        for group in _representative_groups(groups):
            theta_plus = perturb_group(theta0, group.group_id, config.input["perturb_delta"], +1, config)
            theta_minus = perturb_group(theta0, group.group_id, config.input["perturb_delta"], -1, config)
            plus = run_query(
                theta_plus,
                scenario,
                args.seed,
                "smoke_fd_probe_plus",
                output_dir,
                config,
                use_cache=False,
                cache_tag=f"probe_g{group.group_id}_plus",
            )
            minus = run_query(
                theta_minus,
                scenario,
                args.seed,
                "smoke_fd_probe_minus",
                output_dir,
                config,
                use_cache=False,
                cache_tag=f"probe_g{group.group_id}_minus",
            )
            _plot_manual_and_mode(
                read_parsed_log(plus.parsed_log_path),
                figures_dir / f"smoke_manual_mode_{scenario.id}_seed{args.seed}_probe_g{group.group_id}_plus.png",
            )
            for prop in scenario.properties:
                rho_plus = plus.robustness[prop]
                rho_minus = minus.robustness[prop]
                delta = float(config.input["perturb_delta"])
                signed_g = (rho_plus - rho_minus) / (2.0 * delta)
                noise_std = robustness_stats[prop]["std"]
                diff_to_noise = abs(rho_plus - rho_minus) / max(noise_std, 1e-9)
                row = {
                    "scenario_id": scenario.id,
                    "seed": args.seed,
                    "group_id": group.group_id,
                    "channel": group.channel,
                    "window_id": group.window_id,
                    "t_start": group.t_start,
                    "t_end": group.t_end,
                    "property": prop,
                    "rho_plus": rho_plus,
                    "rho_minus": rho_minus,
                    "signed_g": signed_g,
                    "nominal_std": noise_std,
                    "abs_rho_diff_over_noise": diff_to_noise,
                }
                probe_rows.append(row)
                print(
                    f"probe g{group.group_id} {group.channel}@{group.t_start:.1f}-{group.t_end:.1f} "
                    f"{prop}: rho_plus={rho_plus:.6f} rho_minus={rho_minus:.6f} "
                    f"g={signed_g:.6f} diff/noise={diff_to_noise:.2f}"
                )

    summary = {
        "scenario_id": scenario.id,
        "seed": args.seed,
        "artifact_tag": args.artifact_tag,
        "nominal": [r.robustness for r in repeat_results] or (previous_summary or {}).get("nominal", []),
        "robustness_stats": robustness_stats,
        "cache_hit_wall_s": cache_hit_wall_s,
        "probe_rows": probe_rows,
    }
    (smoke_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    if args.artifact_tag:
        (smoke_dir / "artifact_marker.json").write_text(
            json.dumps({"artifact_tag": args.artifact_tag}, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    if repeat_results:
        pd.DataFrame([r.robustness | {"repeat": i} for i, r in enumerate(repeat_results)]).to_csv(
            smoke_dir / "nominal_robustness.csv", index=False
        )
    if probe_rows:
        pd.DataFrame(probe_rows).to_csv(smoke_dir / "signed_fd_probe.csv", index=False)


def _robustness_stats(results) -> dict[str, dict[str, float]]:
    props = list(results[0].robustness)
    stats = {}
    for prop in props:
        values = [float(r.robustness[prop]) for r in results]
        mean = statistics.fmean(values)
        std = statistics.stdev(values) if len(values) > 1 else 0.0
        stats[prop] = {"mean": mean, "std": std, "rel_std": std / max(abs(mean), 1e-9)}
    return stats


def _load_previous_summary(smoke_dir: Path) -> dict | None:
    path = smoke_dir / "summary.json"
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _representative_groups(groups: list[Group]) -> list[Group]:
    wanted = [("roll", 1), ("pitch", 4), ("throttle", 5)]
    selected = []
    for channel, window_id in wanted:
        selected.append(next(g for g in groups if g.channel == channel and g.window_id == window_id))
    return selected


def _plot_manual_and_mode(parsed_log: pd.DataFrame, output_png: Path) -> None:
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True, height_ratios=[2, 1])
    for channel in ["manual_roll", "manual_pitch", "manual_yaw", "manual_throttle"]:
        if channel in parsed_log:
            axes[0].plot(parsed_log["time_s"], parsed_log[channel], label=channel.replace("manual_", ""))
    axes[0].axvline(float(parsed_log["t_neutral_s"].iloc[0]), color="black", linewidth=1, linestyle="--")
    axes[0].set_ylabel("manual")
    axes[0].legend(loc="upper right", ncol=4, fontsize=8)
    mode_codes = pd.Categorical(parsed_log["mode"].astype(str))
    axes[1].step(parsed_log["time_s"], mode_codes.codes, where="post")
    axes[1].set_yticks(range(len(mode_codes.categories)), labels=list(mode_codes.categories))
    axes[1].set_xlabel("time_s")
    axes[1].set_ylabel("mode")
    axes[1].axvline(float(parsed_log["t_neutral_s"].iloc[0]), color="black", linewidth=1, linestyle="--")
    fig.tight_layout()
    fig.savefig(output_png)
    plt.close(fig)


if __name__ == "__main__":
    main()
