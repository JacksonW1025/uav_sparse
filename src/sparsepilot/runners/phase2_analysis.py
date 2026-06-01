from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from sparsepilot.config import load_config
from sparsepilot.metrics import jaccard, mass_overlap, topk_coverage
from sparsepilot.plots import plot_jaccard_grid, plot_topk_coverage_curve
from sparsepilot.runners.fd_snapshot import PRIMARY_PROPERTIES


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--phase-tag", default="phase2a_seed0")
    parser.add_argument("--run-dir", default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    output_dir = Path(args.run_dir) if args.run_dir else Path("runs") / config.experiment_id
    run_analysis(config, output_dir, args.seed, args.phase_tag)


def run_analysis(config, output_dir: Path, seed: int, phase_tag: str) -> None:
    output_dir = Path(output_dir)
    reports_dir = output_dir / "reports"
    figures_dir = output_dir / "figures" / phase_tag
    reports_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)

    gradients = _load_gradients(config, output_dir, seed, phase_tag)
    topk = _topk_curves(gradients)
    topk_path = reports_dir / f"{phase_tag}_seed{seed}_topk_coverage_curve.csv"
    topk.to_csv(topk_path, index=False)
    plot_topk_coverage_curve(topk, figures_dir / f"topk_coverage_curve_seed{seed}.png")

    mode_summary = _mode_conditioned_summary(gradients)
    mode_path = reports_dir / f"{phase_tag}_seed{seed}_mode_conditioned_summary.csv"
    mode_summary.to_csv(mode_path, index=False)
    if not mode_summary.empty:
        plot_jaccard_grid(mode_summary, figures_dir / f"mode_conditioned_jaccard_massoverlap_seed{seed}.png")

    primary = topk[topk["role"] == "primary"]
    median = primary.groupby("k", as_index=False)["coverage"].median()
    k4 = float(median.loc[median["k"] == 4, "coverage"].iloc[0]) if (median["k"] == 4).any() else 0.0
    k8 = float(median.loc[median["k"] == 8, "coverage"].iloc[0]) if (median["k"] == 8).any() else 0.0
    print(f"topk_curve {topk_path}", flush=True)
    print(f"mode_conditioned {mode_path}", flush=True)
    print(f"primary_median top4={k4:.3f} top8={k8:.3f}", flush=True)


def _load_gradients(config, output_dir: Path, seed: int, phase_tag: str) -> dict[tuple[str, str], dict]:
    result = {}
    for scenario in config.scenarios:
        matches = sorted((output_dir / "snapshots").glob(f"{scenario.id}_seed{seed}_*_{phase_tag}"))
        if not matches:
            continue
        snap_dir = matches[-1]
        primary = PRIMARY_PROPERTIES.get(scenario.id, set(scenario.properties))
        for prop in scenario.properties:
            path = snap_dir / f"gradient_{prop}.csv"
            if not path.exists():
                continue
            df = pd.read_csv(path).sort_values("group_id")
            values = df["g"].to_numpy(dtype=float)
            abs_values = np.abs(values)
            result[(scenario.id, prop)] = {
                "scenario_id": scenario.id,
                "property": prop,
                "role": "primary" if prop in primary else "diagnostic",
                "gradient": values,
                "abs_gradient": abs_values,
                "group_ids": df["group_id"].to_numpy(dtype=int),
                "snapshot_dir": snap_dir,
            }
    return result


def _topk_curves(gradients: dict[tuple[str, str], dict]) -> pd.DataFrame:
    rows = []
    for (scenario_id, prop), item in gradients.items():
        abs_values = item["abs_gradient"]
        label = f"{scenario_id}:{prop}"
        for k in range(1, len(abs_values) + 1):
            rows.append(
                {
                    "series": label,
                    "scenario_id": scenario_id,
                    "property": prop,
                    "role": item["role"],
                    "k": k,
                    "coverage": topk_coverage(abs_values, k),
                }
            )
    df = pd.DataFrame(rows)
    primary = df[df["role"] == "primary"]
    if not primary.empty:
        med = primary.groupby("k", as_index=False)["coverage"].median()
        for row in med.itertuples(index=False):
            rows.append(
                {
                    "series": "primary_median",
                    "scenario_id": "primary_median",
                    "property": "primary_median",
                    "role": "primary",
                    "k": int(row.k),
                    "coverage": float(row.coverage),
                }
            )
    return pd.DataFrame(rows)


def _mode_conditioned_summary(gradients: dict[tuple[str, str], dict]) -> pd.DataFrame:
    comparisons = []
    for prop in ["post_neutral_xy_drift", "post_neutral_alt_drift", "post_neutral_xy_velocity"]:
        comparisons.append(("px4_position_vs_px4_hold", "px4_position", prop, "px4_hold", prop))
    comparisons.extend(
        [
            (
                "px4_position_alt_vs_ap_loiter_alt",
                "px4_position",
                "post_neutral_alt_drift",
                "ap_loiter",
                "post_neutral_alt_drift",
            ),
            (
                "ap_loiter_alt_vs_ap_althold_alt",
                "ap_loiter",
                "post_neutral_alt_drift",
                "ap_althold",
                "post_neutral_alt_drift",
            ),
        ]
    )
    rows = []
    for comparison, scenario_a, prop_a, scenario_b, prop_b in comparisons:
        a = gradients.get((scenario_a, prop_a))
        b = gradients.get((scenario_b, prop_b))
        if a is None or b is None:
            continue
        for k in [4, 8]:
            set_a = _topk_set(a["abs_gradient"], k)
            set_b = _topk_set(b["abs_gradient"], k)
            overlap_b_on_a = mass_overlap(set_a, b["abs_gradient"])
            overlap_a_on_b = mass_overlap(set_b, a["abs_gradient"])
            rows.append(
                {
                    "comparison": comparison,
                    "scenario_a": scenario_a,
                    "property_a": prop_a,
                    "scenario_b": scenario_b,
                    "property_b": prop_b,
                    "k": k,
                    "jaccard": jaccard(set_a, set_b),
                    "mass_overlap_b_on_top_a": overlap_b_on_a,
                    "mass_overlap_a_on_top_b": overlap_a_on_b,
                    "mass_overlap_mean": (overlap_b_on_a + overlap_a_on_b) / 2.0,
                    "top_a": ",".join(str(x) for x in sorted(set_a)),
                    "top_b": ",".join(str(x) for x in sorted(set_b)),
                }
            )
    return pd.DataFrame(rows)


def _topk_set(abs_values, k: int) -> set[int]:
    values = np.asarray(abs_values, dtype=float)
    if values.size == 0 or k <= 0:
        return set()
    k = min(k, values.size)
    order = np.argsort(values)[-k:]
    return set(int(x) for x in order)


if __name__ == "__main__":
    main()
