from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import pandas as pd

from cadet.config import load_config
from cadet.groups import Group, build_groups
from cadet.input_model import perturb_group, zero_theta
from cadet.metrics import effective_sparsity, jaccard, mass_overlap, normalized_entropy, topk_coverage
from cadet.plots import plot_gradient_heatmap, plot_jaccard_grid, plot_topk_coverage_curve
from cadet.query import QueryResult, theta_hash
from cadet.runners.fd_snapshot import _query_row, _run_query_with_retry


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--scenario", action="append", default=None)
    parser.add_argument("--phase-tag", default="phase2_px4_seed0_j5_denoise")
    parser.add_argument("--cache-namespace", default=None)
    parser.add_argument("--run-dir", default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    output_dir = Path(args.run_dir) if args.run_dir else Path("runs") / config.experiment_id
    scenario_ids = args.scenario or ["px4_position", "px4_hold"]
    scenarios = [config.scenario_by_id(sid) for sid in scenario_ids]
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    cache_namespace = args.cache_namespace or args.phase_tag
    theta = zero_theta(groups)
    snap_dirs = []
    for scenario in scenarios:
        snap_dirs.append(
            run_repeated_snapshot(
                theta,
                scenario,
                args.seed,
                args.repeats,
                config,
                output_dir,
                groups,
                args.phase_tag,
                cache_namespace,
            )
        )
    if {s.id for s in scenarios} >= {"px4_position", "px4_hold"}:
        write_mode_conditioned(output_dir, args.seed, args.repeats, args.phase_tag, snap_dirs)


def run_repeated_snapshot(
    theta,
    scenario,
    seed: int,
    repeats: int,
    config,
    output_dir: Path,
    groups: list[Group],
    phase_tag: str,
    cache_namespace: str,
) -> Path:
    output_dir = Path(output_dir)
    delta = float(config.input["perturb_delta"])
    snapshot_id = f"{scenario.id}_seed{seed}_{theta_hash(np.asarray(theta, dtype=float))}_{phase_tag}_j{repeats}"
    snap_dir = output_dir / "snapshots" / snapshot_id
    figures_dir = output_dir / "figures" / phase_tag
    snap_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)
    np.save(snap_dir / "theta.npy", np.asarray(theta, dtype=float))
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(snap_dir / "groups.csv", index=False)

    repeat_gradients: dict[str, list[np.ndarray]] = {prop: [] for prop in scenario.properties}
    t0 = time.monotonic()
    for repeat_idx in range(repeats):
        repeat_dir = snap_dir / f"repeat_{repeat_idx}"
        repeat_dir.mkdir(parents=True, exist_ok=True)
        gradients = {prop: np.zeros(len(groups), dtype=float) for prop in scenario.properties}
        query_rows = []
        cache_tag = f"{cache_namespace}_fd_repeat{repeat_idx}"
        for group in groups:
            print(
                f"repeated_fd {scenario.id} seed={seed} repeat={repeat_idx}/{repeats - 1} "
                f"g{group.group_id} {group.channel}@{group.t_start:.1f}-{group.t_end:.1f}",
                flush=True,
            )
            theta_plus = perturb_group(theta, group.group_id, delta, +1, config)
            theta_minus = perturb_group(theta, group.group_id, delta, -1, config)
            plus = _run_repeat_query(theta_plus, scenario, seed, "fd_plus", output_dir, config, cache_tag)
            minus = _run_repeat_query(theta_minus, scenario, seed, "fd_minus", output_dir, config, cache_tag)
            query_rows.append(_repeat_query_row(plus, group, "+", scenario, seed, repeat_idx, cache_tag))
            query_rows.append(_repeat_query_row(minus, group, "-", scenario, seed, repeat_idx, cache_tag))
            for prop in scenario.properties:
                gradients[prop][group.group_id] = (plus.robustness[prop] - minus.robustness[prop]) / (2.0 * delta)
        for prop, values in gradients.items():
            repeat_gradients[prop].append(values.copy())
            rows = []
            for group in groups:
                value = float(values[group.group_id])
                rows.append({**group.__dict__, "g": value, "abs_g": abs(value)})
            pd.DataFrame(rows).to_csv(repeat_dir / f"gradient_{prop}.csv", index=False)
        pd.DataFrame(query_rows).to_csv(repeat_dir / "query_metadata.csv", index=False)

    mean_dir = snap_dir / "mean"
    mean_dir.mkdir(parents=True, exist_ok=True)
    metric_rows = []
    sig_rows = []
    for prop, values_by_repeat in repeat_gradients.items():
        matrix = np.vstack(values_by_repeat)
        mean_g = np.mean(matrix, axis=0)
        std_g = np.std(matrix, axis=0, ddof=1) if repeats > 1 else np.zeros(matrix.shape[1], dtype=float)
        se_g = std_g / math.sqrt(repeats) if repeats > 0 else std_g
        with np.errstate(divide="ignore", invalid="ignore"):
            t_stat = np.divide(mean_g, se_g, out=np.zeros_like(mean_g), where=se_g > 0)
        rows = []
        for group in groups:
            gid = group.group_id
            rows.append(
                {
                    **group.__dict__,
                    "mean_g": float(mean_g[gid]),
                    "abs_mean_g": float(abs(mean_g[gid])),
                    "std_g": float(std_g[gid]),
                    "se_g": float(se_g[gid]),
                    "t_stat": float(t_stat[gid]),
                    "repeat_count": repeats,
                }
            )
        mean_csv = mean_dir / f"gradient_{prop}.csv"
        pd.DataFrame(rows).to_csv(mean_csv, index=False)
        plot_gradient_heatmap(mean_csv, figures_dir / f"heatmap_{scenario.id}_seed{seed}_{prop}_mean_j{repeats}.png")
        metric_rows.append(
            _metric_row(scenario.id, prop, seed, repeats, mean_g, mean_dir / f"gradient_{prop}.csv", output_dir)
        )
        sig_rows.append(_significance_row(scenario.id, prop, seed, repeats, mean_g, se_g))
    metrics = pd.DataFrame(metric_rows)
    metrics.to_csv(mean_dir / "denoised_metrics.csv", index=False)
    sig = pd.DataFrame(sig_rows)
    sig.to_csv(mean_dir / "significance_diagnostics.csv", index=False)
    _write_topk_curve(scenario.id, seed, repeats, mean_dir, figures_dir)
    metadata = {
        "snapshot_id": snapshot_id,
        "scenario_id": scenario.id,
        "seed": seed,
        "delta": delta,
        "phase_tag": phase_tag,
        "cache_namespace": cache_namespace,
        "repeats": repeats,
        "properties": list(scenario.properties),
        "elapsed_wall_time_s": time.monotonic() - t0,
    }
    (snap_dir / "snapshot_metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
    print(f"repeated_fd_complete {snapshot_id} elapsed={metadata['elapsed_wall_time_s']:.1f}s", flush=True)
    return snap_dir


def _run_repeat_query(theta, scenario, seed: int, query_type: str, output_dir: Path, config, cache_tag: str) -> QueryResult:
    return _run_query_with_retry(
        theta,
        scenario,
        seed,
        query_type,
        output_dir,
        config,
        cache_tag=cache_tag,
        use_cache=True,
    )


def _repeat_query_row(result: QueryResult, group: Group, sign: str, scenario, seed: int, repeat_idx: int, cache_tag: str) -> dict:
    row = _query_row(result, group, sign, scenario, seed)
    row["repeat_idx"] = repeat_idx
    row["fd_repeat_cache_tag"] = cache_tag
    return row


def _metric_row(scenario_id: str, prop: str, seed: int, repeats: int, mean_g: np.ndarray, gradient_path: Path, output_dir: Path) -> dict:
    abs_mean = np.abs(mean_g)
    top_order = list(np.argsort(abs_mean)[::-1])
    nominal_std = _nominal_std(output_dir, scenario_id, prop)
    noise_before = 282.0 * nominal_std if nominal_std is not None else float("nan")
    noise_after = noise_before / math.sqrt(repeats) if nominal_std is not None and repeats > 0 else float("nan")
    return {
        "scenario_id": scenario_id,
        "property": prop,
        "seed": seed,
        "repeat_count": repeats,
        "top4_coverage_mean": topk_coverage(abs_mean, 4),
        "top8_coverage_mean": topk_coverage(abs_mean, 8),
        "effective_sparsity_mean": effective_sparsity(mean_g),
        "normalized_entropy_mean": normalized_entropy(abs_mean),
        "max_abs_mean_g": float(np.max(abs_mean)) if abs_mean.size else 0.0,
        "sum_abs_mean_g": float(np.sum(abs_mean)),
        "estimated_noise_l1_before": noise_before,
        "estimated_noise_l1_after": noise_after,
        "top1_group": int(top_order[0]) if top_order else "",
        "top4_groups": ",".join(str(int(x)) for x in top_order[:4]),
        "top8_groups": ",".join(str(int(x)) for x in top_order[:8]),
        "gradient_path": str(gradient_path),
    }


def _nominal_std(output_dir: Path, scenario_id: str, prop: str) -> float | None:
    path = output_dir / "smoke" / f"{scenario_id}_seed0" / "summary.json"
    if not path.exists():
        return None
    summary = json.loads(path.read_text(encoding="utf-8"))
    stats = summary.get("robustness_stats", {}).get(prop)
    if not stats:
        return None
    return float(stats["std"])


def _significance_row(scenario_id: str, prop: str, seed: int, repeats: int, mean_g: np.ndarray, se_g: np.ndarray) -> dict:
    abs_mean = np.abs(mean_g)
    abs_t = np.abs(np.divide(mean_g, se_g, out=np.zeros_like(mean_g), where=se_g > 0))
    pass2 = abs_t >= 2.0
    pass3 = abs_t >= 3.0
    return {
        "scenario_id": scenario_id,
        "property": prop,
        "seed": seed,
        "repeat_count": repeats,
        "groups_abs_mean_ge_2se": ",".join(str(int(i)) for i in np.where(pass2)[0]),
        "groups_abs_mean_ge_3se": ",".join(str(int(i)) for i in np.where(pass3)[0]),
        "count_abs_mean_ge_2se": int(np.sum(pass2)),
        "count_abs_mean_ge_3se": int(np.sum(pass3)),
        "mass_retained_2se": _mass_retained(abs_mean, pass2),
        "mass_retained_3se": _mass_retained(abs_mean, pass3),
        "top4_coverage_masked_2se": _masked_topk(abs_mean, pass2, 4),
        "top8_coverage_masked_2se": _masked_topk(abs_mean, pass2, 8),
        "top4_coverage_masked_3se": _masked_topk(abs_mean, pass3, 4),
        "top8_coverage_masked_3se": _masked_topk(abs_mean, pass3, 8),
    }


def _mass_retained(abs_values: np.ndarray, mask: np.ndarray) -> float:
    total = float(np.sum(abs_values))
    if total <= 0:
        return 0.0
    return float(np.sum(abs_values[mask]) / total)


def _masked_topk(abs_values: np.ndarray, mask: np.ndarray, k: int) -> float:
    masked = np.where(mask, abs_values, 0.0)
    return topk_coverage(masked, k)


def _write_topk_curve(scenario_id: str, seed: int, repeats: int, mean_dir: Path, figures_dir: Path) -> None:
    rows = []
    for grad_path in sorted(mean_dir.glob("gradient_*.csv")):
        prop = grad_path.stem[len("gradient_") :]
        df = pd.read_csv(grad_path)
        abs_values = df["abs_mean_g"].to_numpy(dtype=float)
        for k in range(1, len(abs_values) + 1):
            rows.append(
                {
                    "series": f"{scenario_id}:{prop}",
                    "scenario_id": scenario_id,
                    "property": prop,
                    "role": "primary",
                    "k": k,
                    "coverage": topk_coverage(abs_values, k),
                }
            )
    topk = pd.DataFrame(rows)
    topk.to_csv(mean_dir / "topk_coverage_curve.csv", index=False)
    plot_topk_coverage_curve(topk, figures_dir / f"topk_coverage_curve_{scenario_id}_seed{seed}_mean_j{repeats}.png")


def write_mode_conditioned(output_dir: Path, seed: int, repeats: int, phase_tag: str, snap_dirs: list[Path]) -> None:
    by_scenario = {p.name.split("_seed", maxsplit=1)[0]: p for p in snap_dirs}
    position = by_scenario.get("px4_position")
    hold = by_scenario.get("px4_hold")
    if position is None or hold is None:
        return
    rows = []
    for prop in ["post_neutral_xy_drift", "post_neutral_alt_drift", "post_neutral_xy_velocity"]:
        pos = _read_mean_abs(position, prop)
        hol = _read_mean_abs(hold, prop)
        if pos is None or hol is None:
            continue
        pos_top4 = _topk_set(pos, 4)
        hol_top4 = _topk_set(hol, 4)
        pos_top8 = _topk_set(pos, 8)
        hol_top8 = _topk_set(hol, 8)
        rows.append(
            {
                "property": prop,
                "seed": seed,
                "repeat_count": repeats,
                "jaccard_top4": jaccard(pos_top4, hol_top4),
                "jaccard_top8": jaccard(pos_top8, hol_top8),
                "mass_overlap_top4_position_to_hold": mass_overlap(pos_top4, hol),
                "mass_overlap_top4_hold_to_position": mass_overlap(hol_top4, pos),
                "mass_overlap_top8_position_to_hold": mass_overlap(pos_top8, hol),
                "mass_overlap_top8_hold_to_position": mass_overlap(hol_top8, pos),
                "top4_position": ",".join(str(x) for x in sorted(pos_top4)),
                "top4_hold": ",".join(str(x) for x in sorted(hol_top4)),
                "top8_position": ",".join(str(x) for x in sorted(pos_top8)),
                "top8_hold": ",".join(str(x) for x in sorted(hol_top8)),
            }
        )
    reports_dir = output_dir / "reports"
    figures_dir = output_dir / "figures" / phase_tag
    reports_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)
    mode = pd.DataFrame(rows)
    mode_path = reports_dir / f"{phase_tag}_seed{seed}_mode_conditioned_denoised.csv"
    mode.to_csv(mode_path, index=False)
    if not mode.empty:
        plot_rows = []
        for row in mode.itertuples(index=False):
            for k in [4, 8]:
                plot_rows.append(
                    {
                        "comparison": "px4_position_vs_px4_hold",
                        "property_a": row.property,
                        "property_b": row.property,
                        "k": k,
                        "jaccard": getattr(row, f"jaccard_top{k}"),
                        "mass_overlap_mean": (
                            getattr(row, f"mass_overlap_top{k}_position_to_hold")
                            + getattr(row, f"mass_overlap_top{k}_hold_to_position")
                        )
                        / 2.0,
                    }
                )
        plot_jaccard_grid(
            pd.DataFrame(plot_rows),
            figures_dir / f"mode_conditioned_jaccard_massoverlap_px4_seed{seed}_mean_j{repeats}.png",
        )


def _read_mean_abs(snapshot_dir: Path, prop: str) -> np.ndarray | None:
    path = snapshot_dir / "mean" / f"gradient_{prop}.csv"
    if not path.exists():
        return None
    return pd.read_csv(path)["abs_mean_g"].to_numpy(dtype=float)


def _topk_set(abs_values: np.ndarray, k: int) -> set[int]:
    if abs_values.size == 0 or k <= 0:
        return set()
    return set(int(x) for x in np.argsort(abs_values)[-min(k, abs_values.size) :])


if __name__ == "__main__":
    main()
