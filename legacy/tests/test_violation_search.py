from __future__ import annotations

import numpy as np

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.input_model import project_theta, zero_theta
from cadet.violation_search import generate_initial_candidates, saturation_summary


def test_violation_search_candidates_are_feasible():
    config = load_config("configs/rq1_minimal.yaml")
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    candidates = generate_initial_candidates(config, groups, random_count=12, rng=np.random.default_rng(123))

    assert len(candidates) == 32
    assert len({candidate.label for candidate in candidates}) == len(candidates)
    for candidate in candidates:
        theta = candidate.theta
        assert theta.shape == (40,)
        assert np.all(theta <= config.input["max_value"])
        assert np.all(theta >= config.input["min_value"])
        assert np.allclose(project_theta(theta, config), theta)


def test_zero_theta_is_fd_interior():
    config = load_config("configs/rq1_minimal.yaml")
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    summary = saturation_summary(zero_theta(groups), config, groups, tol=0.02)

    assert summary["fd_interior"] is True
    assert summary["fd_clean_two_sided_groups"] == 40
    assert summary["amplitude_saturated"] is False
