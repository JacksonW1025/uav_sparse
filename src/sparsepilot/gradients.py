from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from sparsepilot.config import ScenarioCfg
from sparsepilot.groups import build_groups
from sparsepilot.input_model import perturb_group
from sparsepilot.query import run_query, theta_hash


@dataclass
class GradientSnapshot:
    snapshot_id: str
    theta: np.ndarray
    gradients: dict[str, np.ndarray]
    output_dir: Path


def finite_difference_snapshot(theta, scenario: ScenarioCfg, seed: int, config, output_dir: Path) -> GradientSnapshot:
    output_dir = Path(output_dir)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    delta = float(config.input["perturb_delta"])
    snapshot_id = f"{scenario.id}_seed{seed}_{theta_hash(np.asarray(theta, dtype=float))}"
    snap_dir = output_dir / "snapshots" / snapshot_id
    snap_dir.mkdir(parents=True, exist_ok=True)
    np.save(snap_dir / "theta.npy", np.asarray(theta, dtype=float))
    pd.DataFrame([g.__dict__ for g in groups]).to_csv(snap_dir / "groups.csv", index=False)

    gradients = {p: np.zeros(len(groups), dtype=float) for p in scenario.properties}
    for group in groups:
        theta_plus = perturb_group(theta, group.group_id, delta, +1, config)
        theta_minus = perturb_group(theta, group.group_id, delta, -1, config)
        r_plus = run_query(theta_plus, scenario, seed, "fd_plus", output_dir, config)
        r_minus = run_query(theta_minus, scenario, seed, "fd_minus", output_dir, config)
        for prop in scenario.properties:
            gradients[prop][group.group_id] = (r_plus.robustness[prop] - r_minus.robustness[prop]) / (2.0 * delta)

    for prop, g in gradients.items():
        rows = []
        for group in groups:
            value = float(g[group.group_id])
            rows.append({**group.__dict__, "g": value, "abs_g": abs(value)})
        pd.DataFrame(rows).to_csv(snap_dir / f"gradient_{prop}.csv", index=False)
    metadata = {"snapshot_id": snapshot_id, "scenario_id": scenario.id, "seed": seed, "delta": delta}
    (snap_dir / "snapshot_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return GradientSnapshot(snapshot_id, np.asarray(theta, dtype=float), gradients, snap_dir)


def cheap_sparse_snapshot(*args, **kwargs):
    raise NotImplementedError("cheap sparse probing belongs to a later RQ")
