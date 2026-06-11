from __future__ import annotations

from pathlib import Path

import numpy as np

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.input_model import project_theta
from cadet.runners.route1_h2_campaign import (
    BoundaryResult,
    Condition,
    _boundary_displacements,
    _build_summary,
    _config_for_condition,
    _sample_uniform_feasible,
)


def test_config_for_condition_only_changes_xy_velocity_vmax(tmp_path: Path):
    config = load_config("configs/rq1_minimal.yaml")
    changed = _config_for_condition(config, tmp_path / "run", 0.8)

    assert changed.properties["post_neutral_xy_velocity"]["v_max_mps"] == 0.8
    assert config.properties["post_neutral_xy_velocity"]["v_max_mps"] == 1.0
    assert changed.properties["post_neutral_xy_drift"] == config.properties["post_neutral_xy_drift"]
    assert changed.logging["jsonl"].endswith("run/logs/queries.jsonl")


def test_sample_uniform_feasible_respects_bounds_and_rate_limits():
    config = load_config("configs/rq1_minimal.yaml")
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    rng = np.random.default_rng(123)

    theta = _sample_uniform_feasible(config, groups, rng)

    assert theta.shape == (40,)
    assert np.all(theta <= config.input["max_value"])
    assert np.all(theta >= config.input["min_value"])
    assert np.allclose(project_theta(theta, config), theta)


def test_boundary_displacements_use_adjacent_warm_results():
    left = BoundaryResult("warm_anchor", 1, 1.0, "reused", np.array([1.0, 0.0]), "a", 0.0, 0.0, 5, {})
    mid = BoundaryResult("warm", 2, 0.9, "complete", np.array([0.5, 0.0]), "b", 0.0, 0.0, 10, {})
    right = BoundaryResult("warm", 3, 0.8, "complete", np.array([0.5, 0.5]), "c", 0.0, 0.0, 10, {})

    rows = _boundary_displacements([left, mid, right])

    assert [row["from_v_max"] for row in rows] == [1.0, 0.9]
    assert [row["to_v_max"] for row in rows] == [0.9, 0.8]
    assert rows[0]["l2"] == 0.5
    assert rows[1]["l2"] == 0.5


def test_build_summary_campaign_query_ratios(tmp_path: Path):
    config = load_config("configs/rq1_minimal.yaml")
    conditions = [
        Condition(i, v, f"v{i}", _config_for_condition(config, tmp_path, v))
        for i, v in enumerate([1.0, 0.9, 0.8], start=1)
    ]
    anchor = BoundaryResult("warm_anchor", 1, 1.0, "reused", np.ones(2), "a", 0.0, 0.0, 5, {})
    warm_results = [
        anchor,
        BoundaryResult("warm", 2, 0.9, "complete", np.ones(2) * 0.9, "b", 0.0, 0.0, 20, {}),
        BoundaryResult("warm", 3, 0.8, "complete", np.ones(2) * 0.8, "c", 0.0, 0.0, 20, {}),
    ]
    cold_results = {
        "structured": [
            BoundaryResult("structured", 1, 1.0, "complete", np.ones(2), "s1", 0.0, 0.0, 100, {}),
            BoundaryResult("structured", 2, 0.9, "complete", np.ones(2), "s2", 0.0, 0.0, 100, {}),
            BoundaryResult("structured", 3, 0.8, "complete", np.ones(2), "s3", 0.0, 0.0, 100, {}),
        ],
        "uniform": [
            BoundaryResult("uniform", 1, 1.0, "complete", np.ones(2), "u1", 0.0, 0.0, 80, {}),
            BoundaryResult("uniform", 2, 0.9, "complete", np.ones(2), "u2", 0.0, 0.0, 80, {}),
            BoundaryResult("uniform", 3, 0.8, "complete", np.ones(2), "u3", 0.0, 0.0, 80, {}),
        ],
        "descent": [
            BoundaryResult("descent", 1, 1.0, "complete", np.ones(2), "d1", 0.0, 0.0, 60, {}),
            BoundaryResult("descent", 2, 0.9, "complete", np.ones(2), "d2", 0.0, 0.0, 60, {}),
            BoundaryResult("descent", 3, 0.8, "complete", np.ones(2), "d3", 0.0, 0.0, 60, {}),
        ],
    }

    summary = _build_summary(
        output_dir=tmp_path,
        scenario_id="px4_position",
        seed=0,
        conditions=conditions,
        theta_v_path=Path("theta.npy"),
        anchor=anchor,
        warm_results=warm_results,
        cold_results=cold_results,
        channel_measurements=[],
        total_queries=1,
        elapsed_wall_time_s=2.0,
    )

    structured = next(row for row in summary["campaign_query_ratios"] if row["cold_baseline"] == "structured")
    assert structured["warm_campaign_queries"] == 140
    assert structured["cold_all_3_conditions_queries"] == 300
    assert structured["cold_over_warm_speedup"] == 300 / 140
