from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import numpy as np
import pandas as pd

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.input_model import project_theta, zero_theta
from cadet.metrics import effective_sparsity, jaccard, mass_overlap, normalized_entropy, topk_coverage
from cadet.query import QueryResult, theta_hash
from cadet.runners.fd_snapshot import _query_row, _run_query_with_retry
from cadet.runners.repeated_fd import run_repeated_snapshot


PATHS = {
    "path_xy_drift": "post_neutral_xy_drift",
    "path_xy_velocity": "post_neutral_xy_velocity",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--scenario", default="px4_position")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--steps", type=int, default=3)
    parser.add_argument("--phase-tag", default="persistence_pilot_px4_seed0")
    parser.add_argument("--theta0-phase-tag", default="phase2_px4_seed0_j5_denoise")
    parser.add_argument("--cache-namespace", default=None)
    parser.add_argument("--run-dir", default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    output_dir = Path(args.run_dir) if args.run_dir else Path("runs") / config.experiment_id
    scenario = config.scenario_by_id(args.scenario)
    if scenario.id != "px4_position" or args.seed != 0:
        raise ValueError("persistence_pilot is intentionally restricted to px4_position seed 0")
    if args.steps != 3:
        raise ValueError("persistence_pilot is intentionally restricted to 3 steps")
    if args.repeats != 5:
        raise ValueError("persistence_pilot is intentionally restricted to J=5 repeats")

    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta0 = zero_theta(groups)
    theta0_snapshot = _theta0_snapshot_dir(output_dir, scenario.id, args.seed, theta0, args.theta0_phase_tag, args.repeats)
    _require_snapshot(theta0_snapshot, PATHS.values())

    cache_namespace = args.cache_namespace or args.phase_tag
    paths_root = output_dir / "paths" / f"{scenario.id}_seed{args.seed}_{args.phase_tag}"
    reports_dir = output_dir / "reports"
    paths_root.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)

    all_step_rows: list[dict] = []
    all_transition_rows: list[dict] = []
    all_robustness_rows: list[dict] = []
    all_update_rows: list[dict] = []
    all_snapshot_rows: list[dict] = []
    path_summaries = []

    t0 = time.monotonic()
    for path_id, path_property in PATHS.items():
        print(f"persistence_pilot_path_start {path_id} property={path_property}", flush=True)
        result = run_path(
            path_id=path_id,
            path_property=path_property,
            theta0=theta0,
            theta0_snapshot=theta0_snapshot,
            scenario=scenario,
            seed=args.seed,
            repeats=args.repeats,
            steps=args.steps,
            config=config,
            output_dir=output_dir,
            paths_root=paths_root,
            groups=groups,
            phase_tag=args.phase_tag,
            cache_namespace=cache_namespace,
        )
        all_step_rows.extend(result["step_rows"])
        all_transition_rows.extend(result["transition_rows"])
        all_robustness_rows.extend(result["robustness_rows"])
        all_update_rows.extend(result["update_rows"])
        all_snapshot_rows.extend(result["snapshot_rows"])
        path_summaries.append(result["summary"])

    step_df = pd.DataFrame(all_step_rows)
    transition_df = pd.DataFrame(all_transition_rows)
    robustness_df = pd.DataFrame(all_robustness_rows)
    update_df = pd.DataFrame(all_update_rows)
    snapshot_df = pd.DataFrame(all_snapshot_rows)

    step_path = reports_dir / f"{args.phase_tag}_step_metrics.csv"
    transition_path = reports_dir / f"{args.phase_tag}_transitions.csv"
    robustness_path = reports_dir / f"{args.phase_tag}_robustness.csv"
    update_path = reports_dir / f"{args.phase_tag}_updates.csv"
    snapshot_path = reports_dir / f"{args.phase_tag}_snapshot_validation.csv"
    step_df.to_csv(step_path, index=False)
    transition_df.to_csv(transition_path, index=False)
    robustness_df.to_csv(robustness_path, index=False)
    update_df.to_csv(update_path, index=False)
    snapshot_df.to_csv(snapshot_path, index=False)

    median_top16 = float(statistics.median(transition_df["mass_overlap_top16"])) if not transition_df.empty else float("nan")
    if median_top16 >= 0.6:
        decision = "persistent"
    elif median_top16 < 0.4:
        decision = "not_persistent"
    else:
        decision = "ambiguous"

    summary = {
        "phase_tag": args.phase_tag,
        "scenario_id": scenario.id,
        "seed": args.seed,
        "steps": args.steps,
        "repeats": args.repeats,
        "theta0_snapshot": str(theta0_snapshot),
        "paths_root": str(paths_root),
        "median_mass_overlap_top16": median_top16,
        "decision": decision,
        "elapsed_wall_time_s": time.monotonic() - t0,
        "path_summaries": path_summaries,
        "outputs": {
            "step_metrics": str(step_path),
            "transitions": str(transition_path),
            "robustness": str(robustness_path),
            "updates": str(update_path),
            "snapshot_validation": str(snapshot_path),
        },
    }
    summary_path = paths_root / "pilot_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    report_path = reports_dir / f"{args.phase_tag}_report.md"
    _write_report(report_path, summary, step_df, transition_df, robustness_df, snapshot_df)
    print(f"persistence_pilot_complete decision={decision} median_top16={median_top16:.3f}", flush=True)
    print(f"persistence_pilot_report {report_path}", flush=True)


def run_path(
    *,
    path_id: str,
    path_property: str,
    theta0: np.ndarray,
    theta0_snapshot: Path,
    scenario,
    seed: int,
    repeats: int,
    steps: int,
    config,
    output_dir: Path,
    paths_root: Path,
    groups,
    phase_tag: str,
    cache_namespace: str,
) -> dict:
    eta = float(config.persistence_path["eta"])
    top_m = int(config.persistence_path["top_m_update_groups"])
    path_dir = paths_root / path_id
    path_dir.mkdir(parents=True, exist_ok=True)
    theta = np.asarray(theta0, dtype=float).copy()

    snapshots: list[Path] = []
    thetas: list[np.ndarray] = []
    step_rows: list[dict] = []
    robustness_rows: list[dict] = []
    update_rows: list[dict] = []
    snapshot_rows: list[dict] = []

    for step_idx in range(steps):
        step_dir = path_dir / f"step_{step_idx}"
        step_dir.mkdir(parents=True, exist_ok=True)
        np.save(step_dir / "theta.npy", theta)

        if step_idx == 0:
            snap_dir = theta0_snapshot
            reused = True
        else:
            step_phase = f"{phase_tag}_{path_id}_step{step_idx}"
            step_cache = f"{cache_namespace}_{path_id}_step{step_idx}"
            snap_dir = run_repeated_snapshot(
                theta,
                scenario,
                seed,
                repeats,
                config,
                output_dir,
                groups,
                step_phase,
                step_cache,
            )
            reused = False
        snapshots.append(snap_dir)
        thetas.append(theta.copy())
        (step_dir / "snapshot_ref.json").write_text(
            json.dumps({"snapshot_dir": str(snap_dir), "reused_theta0": reused}, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        snapshot_rows.append(_snapshot_validation_row(path_id, step_idx, snap_dir, reused))

        for prop in ["post_neutral_xy_drift", "post_neutral_xy_velocity"]:
            if prop in scenario.properties:
                step_rows.append(_step_metric_row(path_id, path_property, prop, step_idx, snap_dir))
        robustness_rows.append(
            _robustness_row(
                path_id,
                path_property,
                step_idx,
                theta,
                scenario,
                seed,
                output_dir,
                config,
                phase_tag,
                reused_theta0=reused,
            )
        )

        if step_idx < steps - 1:
            g = _read_mean_gradient(snap_dir, path_property)
            update = _update_from_gradient(g, top_m, eta, theta, config)
            update_rows.append(
                {
                    "path_id": path_id,
                    "property": path_property,
                    "step_r": step_idx,
                    "eta": eta,
                    "top_m_update_groups": top_m,
                    "update_groups": ",".join(str(int(i)) for i in update["groups"]),
                    "update_signs": ",".join(str(int(s)) for s in update["signs"]),
                    "theta_hash_before": theta_hash(theta),
                    "theta_hash_after": theta_hash(update["theta_next"]),
                    "projection_l2_delta": float(np.linalg.norm(update["theta_next"] - (theta - eta * update["direction"]))),
                }
            )
            theta = update["theta_next"]

    transition_rows = []
    for step_idx in range(steps - 1):
        transition_rows.append(_transition_row(path_id, path_property, step_idx, snapshots[step_idx], snapshots[step_idx + 1]))

    path_summary = {
        "path_id": path_id,
        "property": path_property,
        "snapshots": [str(p) for p in snapshots],
        "theta_hashes": [theta_hash(t) for t in thetas],
        "robustness": [row["robustness"] for row in robustness_rows],
    }
    (path_dir / "path_summary.json").write_text(json.dumps(path_summary, indent=2, sort_keys=True), encoding="utf-8")
    return {
        "step_rows": step_rows,
        "transition_rows": transition_rows,
        "robustness_rows": robustness_rows,
        "update_rows": update_rows,
        "snapshot_rows": snapshot_rows,
        "summary": path_summary,
    }


def _theta0_snapshot_dir(output_dir: Path, scenario_id: str, seed: int, theta0: np.ndarray, phase_tag: str, repeats: int) -> Path:
    return output_dir / "snapshots" / f"{scenario_id}_seed{seed}_{theta_hash(theta0)}_{phase_tag}_j{repeats}"


def _require_snapshot(snapshot_dir: Path, properties) -> None:
    if not snapshot_dir.exists():
        raise FileNotFoundError(f"theta0 J=5 snapshot not found: {snapshot_dir}")
    for prop in properties:
        path = snapshot_dir / "mean" / f"gradient_{prop}.csv"
        if not path.exists():
            raise FileNotFoundError(f"theta0 mean gradient not found: {path}")


def _read_mean_gradient(snapshot_dir: Path, prop: str) -> np.ndarray:
    df = pd.read_csv(snapshot_dir / "mean" / f"gradient_{prop}.csv").sort_values("group_id")
    return df["mean_g"].to_numpy(dtype=float)


def _read_abs_mean(snapshot_dir: Path, prop: str) -> np.ndarray:
    df = pd.read_csv(snapshot_dir / "mean" / f"gradient_{prop}.csv").sort_values("group_id")
    return df["abs_mean_g"].to_numpy(dtype=float)


def _topk_set(abs_values: np.ndarray, k: int) -> set[int]:
    if abs_values.size == 0 or k <= 0:
        return set()
    return set(int(x) for x in np.argsort(abs_values)[-min(k, abs_values.size) :])


def _topk_csv(abs_values: np.ndarray, k: int) -> str:
    order = np.argsort(abs_values)[::-1][:k]
    return ",".join(str(int(x)) for x in order)


def _step_metric_row(path_id: str, path_property: str, prop: str, step_idx: int, snapshot_dir: Path) -> dict:
    g = _read_mean_gradient(snapshot_dir, prop)
    abs_g = np.abs(g)
    metrics = _read_metric_lookup(snapshot_dir).get(prop, {})
    noise_after = _float_or_nan(metrics.get("estimated_noise_l1_after"))
    sum_abs = float(np.sum(abs_g))
    return {
        "path_id": path_id,
        "path_property": path_property,
        "property": prop,
        "is_path_property": prop == path_property,
        "step_r": step_idx,
        "snapshot_dir": str(snapshot_dir),
        "top4_coverage": topk_coverage(abs_g, 4),
        "top8_coverage": topk_coverage(abs_g, 8),
        "top16_coverage": topk_coverage(abs_g, 16),
        "effective_sparsity": effective_sparsity(g),
        "normalized_entropy": normalized_entropy(abs_g),
        "max_abs_mean_g": float(np.max(abs_g)) if abs_g.size else 0.0,
        "sum_abs_mean_g": sum_abs,
        "estimated_noise_l1_after": noise_after,
        "noise_after_over_sum": noise_after / sum_abs if sum_abs > 0 and np.isfinite(noise_after) else float("nan"),
        "top4_groups": _topk_csv(abs_g, 4),
        "top8_groups": _topk_csv(abs_g, 8),
        "top16_groups": _topk_csv(abs_g, 16),
    }


def _read_metric_lookup(snapshot_dir: Path) -> dict[str, dict]:
    path = snapshot_dir / "mean" / "denoised_metrics.csv"
    if not path.exists():
        return {}
    df = pd.read_csv(path)
    return {str(row.property): row._asdict() for row in df.itertuples(index=False)}


def _float_or_nan(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _update_from_gradient(g: np.ndarray, top_m: int, eta: float, theta: np.ndarray, config) -> dict:
    abs_g = np.abs(g)
    groups = np.argsort(abs_g)[::-1][:top_m]
    signs = np.sign(g[groups]).astype(int)
    direction = np.zeros_like(theta, dtype=float)
    direction[groups] = signs
    theta_next = project_theta(np.asarray(theta, dtype=float) - eta * direction, config)
    return {"groups": groups, "signs": signs, "direction": direction, "theta_next": theta_next}


def _transition_row(path_id: str, path_property: str, step_idx: int, snap_r: Path, snap_next: Path) -> dict:
    abs_r = _read_abs_mean(snap_r, path_property)
    abs_next = _read_abs_mean(snap_next, path_property)
    row = {
        "path_id": path_id,
        "property": path_property,
        "step_r": step_idx,
        "step_next": step_idx + 1,
        "snapshot_r": str(snap_r),
        "snapshot_next": str(snap_next),
    }
    for k in [4, 8, 16]:
        top_r = _topk_set(abs_r, k)
        top_next = _topk_set(abs_next, k)
        row[f"jaccard_top{k}"] = jaccard(top_r, top_next)
        row[f"mass_overlap_top{k}"] = mass_overlap(top_r, abs_next)
        row[f"top{k}_r"] = ",".join(str(int(i)) for i in sorted(top_r))
        row[f"top{k}_next"] = ",".join(str(int(i)) for i in sorted(top_next))
    return row


def _robustness_row(
    path_id: str,
    path_property: str,
    step_idx: int,
    theta: np.ndarray,
    scenario,
    seed: int,
    output_dir: Path,
    config,
    phase_tag: str,
    *,
    reused_theta0: bool,
) -> dict:
    if reused_theta0:
        robustness = _theta0_smoke_mean(output_dir, scenario.id, path_property)
        return {
            "path_id": path_id,
            "property": path_property,
            "step_r": step_idx,
            "theta_hash": theta_hash(theta),
            "robustness": robustness,
            "source": "smoke_nominal_mean",
            "query_id": "",
            "cache_tag": "",
            "total_wall_time_s": 0.0,
        }
    cache_tag = f"{phase_tag}_{path_id}_step{step_idx}_nominal"
    result = _run_query_with_retry(
        theta,
        scenario,
        seed,
        "path_nominal",
        output_dir,
        config,
        cache_tag=cache_tag,
        use_cache=True,
    )
    return {
        "path_id": path_id,
        "property": path_property,
        "step_r": step_idx,
        "theta_hash": result.theta_hash,
        "robustness": float(result.robustness[path_property]),
        "source": "path_nominal_single_query",
        "query_id": result.query_id,
        "cache_tag": cache_tag,
        "total_wall_time_s": float(result.metadata.get("total_wall_time_s", 0.0)),
    }


def _theta0_smoke_mean(output_dir: Path, scenario_id: str, prop: str) -> float:
    path = output_dir / "smoke" / f"{scenario_id}_seed0" / "summary.json"
    summary = json.loads(path.read_text(encoding="utf-8"))
    return float(summary["robustness_stats"][prop]["mean"])


def _snapshot_validation_row(path_id: str, step_idx: int, snapshot_dir: Path, reused: bool) -> dict:
    rows = []
    for meta_path in sorted(snapshot_dir.glob("repeat_*/query_metadata.csv")):
        rows.extend(pd.read_csv(meta_path).to_dict("records"))
    if not rows:
        return {
            "path_id": path_id,
            "step_r": step_idx,
            "snapshot_dir": str(snapshot_dir),
            "reused_theta0": reused,
            "query_rows": 0,
            "unique_query_ids": 0,
            "unique_repeat_cache_tags": 0,
            "manual_perturb_abs_max_min": float("nan"),
            "manual_perturb_abs_max_max": float("nan"),
            "manual_tail_abs_max_min": float("nan"),
            "manual_tail_abs_max_max": float("nan"),
            "median_total_wall_time_s": float("nan"),
            "settled_tail_mode_value_counts": "",
            "settled_tail_main_value_counts": "",
            "settled_tail_sub_value_counts": "",
        }
    return {
        "path_id": path_id,
        "step_r": step_idx,
        "snapshot_dir": str(snapshot_dir),
        "reused_theta0": reused,
        "query_rows": len(rows),
        "unique_query_ids": len({str(r["query_id"]) for r in rows}),
        "unique_repeat_cache_tags": len({str(r.get("fd_repeat_cache_tag", "")) for r in rows}),
        "manual_perturb_abs_max_min": min(float(r["manual_perturb_abs_max"]) for r in rows),
        "manual_perturb_abs_max_max": max(float(r["manual_perturb_abs_max"]) for r in rows),
        "manual_tail_abs_max_min": min(float(r["manual_tail_abs_max"]) for r in rows),
        "manual_tail_abs_max_max": max(float(r["manual_tail_abs_max"]) for r in rows),
        "median_total_wall_time_s": statistics.median(float(r["meta_total_wall_time_s"]) for r in rows),
        "settled_tail_mode_value_counts": _counts_csv(rows, "settled_tail_mode_values"),
        "settled_tail_main_value_counts": _counts_csv(rows, "settled_tail_px4_main_mode_values"),
        "settled_tail_sub_value_counts": _counts_csv(rows, "settled_tail_px4_sub_mode_values"),
    }


def _counts_csv(rows: list[dict], key: str) -> str:
    values = sorted({str(r.get(key, "")) for r in rows})
    return ";".join(f"{value}:{sum(1 for r in rows if str(r.get(key, '')) == value)}" for value in values)


def _write_report(report_path: Path, summary: dict, step_df: pd.DataFrame, transition_df: pd.DataFrame, robustness_df: pd.DataFrame, snapshot_df: pd.DataFrame) -> None:
    lines = [
        "# PX4 Seed0 Persistence Pilot",
        "",
        "Scope: `px4_position`, seed 0, zero-theta start, two paths, three steps, J=5 full reference FD at each step. Step 0 reuses the prior PX4 seed0 J=5 denoise snapshot. No seeds 1/2, broader Phase 3, AP persistence, or nonzero-base run is included.",
        "",
        f"Decision rule median `mass_overlap_top16` across four transitions: **{summary['median_mass_overlap_top16']:.3f}** -> **{summary['decision']}**.",
        "",
        "## Outputs",
        "",
    ]
    for name, path in summary["outputs"].items():
        lines.append(f"- {name}: `{path}`")
    lines.extend(
        [
            f"- paths root: `{summary['paths_root']}`",
            "",
            "## Per-Step Path-Property Metrics",
            "",
            "| path | property | step | top4 | top8 | top16 | eff | max | sum_abs | noise_after/sum | top16 groups |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in step_df[step_df["is_path_property"]].itertuples(index=False):
        lines.append(
            f"| {row.path_id} | {row.property} | {row.step_r} | {row.top4_coverage:.3f} | {row.top8_coverage:.3f} | "
            f"{row.top16_coverage:.3f} | {row.effective_sparsity:.2f} | {row.max_abs_mean_g:.3f} | "
            f"{row.sum_abs_mean_g:.3f} | {row.noise_after_over_sum:.2f} | {row.top16_groups} |"
        )
    lines.extend(
        [
            "",
            "## Transitions",
            "",
            "| path | property | r->next | mass16 | jacc16 | mass8 | jacc8 | mass4 | jacc4 |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in transition_df.itertuples(index=False):
        lines.append(
            f"| {row.path_id} | {row.property} | {row.step_r}->{row.step_next} | {row.mass_overlap_top16:.3f} | "
            f"{row.jaccard_top16:.3f} | {row.mass_overlap_top8:.3f} | {row.jaccard_top8:.3f} | "
            f"{row.mass_overlap_top4:.3f} | {row.jaccard_top4:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Robustness",
            "",
            "| path | property | step | robustness | source |",
            "| --- | --- | ---: | ---: | --- |",
        ]
    )
    for row in robustness_df.itertuples(index=False):
        lines.append(f"| {row.path_id} | {row.property} | {row.step_r} | {row.robustness:.6f} | {row.source} |")
    lines.extend(
        [
            "",
            "## Snapshot Validation",
            "",
            "| path | step | reused | rows | unique ids | repeat tags | median query s | manual perturb | manual tail | mode counts | main counts | sub counts |",
            "| --- | ---: | --- | ---: | ---: | ---: | ---: | --- | --- | --- | --- | --- |",
        ]
    )
    for row in snapshot_df.itertuples(index=False):
        lines.append(
            f"| {row.path_id} | {row.step_r} | {row.reused_theta0} | {row.query_rows} | {row.unique_query_ids} | "
            f"{row.unique_repeat_cache_tags} | {row.median_total_wall_time_s:.2f} | "
            f"{row.manual_perturb_abs_max_min:.2f}-{row.manual_perturb_abs_max_max:.2f} | "
            f"{row.manual_tail_abs_max_min:.2f}-{row.manual_tail_abs_max_max:.2f} | "
            f"{row.settled_tail_mode_value_counts} | {row.settled_tail_main_value_counts} | {row.settled_tail_sub_value_counts} |"
        )
    lines.append("")
    report_path.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
