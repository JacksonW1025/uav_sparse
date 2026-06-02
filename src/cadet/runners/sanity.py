from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from cadet.config import load_config
from cadet.gradients import finite_difference_snapshot
from cadet.groups import build_groups
from cadet.input_model import zero_theta
from cadet.metrics import mass_overlap
from cadet.support import topk_support
from cadet.vehicle.synthetic import SyntheticAdapter


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--noise-fraction", type=float, default=0.0)
    args = parser.parse_args()

    config = load_config(args.config)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    print(f"experiment_id={config.experiment_id}")
    print(f"scenarios={[s.id for s in config.scenarios]}")
    print(f"D={len(groups)}")
    print(f"properties={list(config.properties)}")
    if args.dry_run:
        return

    scenario = config.scenarios[0]
    theta = zero_theta(groups)
    snapshot = finite_difference_snapshot(theta, scenario, config.seeds[0], config, Path("runs") / config.experiment_id)
    for prop, g in snapshot.gradients.items():
        support = SyntheticAdapter.TRUE_SUPPORT[prop]
        estimated = topk_support(np.abs(g), 5)
        recall = len(estimated & support) / len(support)
        overlap = mass_overlap(support, np.abs(g))
        print(f"{prop}: support_recall@5={recall:.3f} mass_overlap={overlap:.3f}")


if __name__ == "__main__":
    main()
