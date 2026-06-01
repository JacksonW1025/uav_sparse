from __future__ import annotations

import numpy as np

from sparsepilot.config import load_config
from sparsepilot.groups import build_groups
from sparsepilot.runners.h3_transition import (
    PROPERTIES,
    CandidateU,
    _classify_pair,
    _config_for_run_dir,
    _mark_t_specific,
    _return_to_neutral_by,
    _transition_scenario,
)


class DummyEval:
    def __init__(self, eval_id, scenario_id, t_switch_s, theta_hash, stats):
        self.eval_id = eval_id
        self.scenario_id = scenario_id
        self.t_switch_s = t_switch_s
        self.theta_hash = theta_hash
        self.stats = stats


def _stats(value: float, std: float = 0.01):
    return {prop: {"mean": value, "std": std, "min": value, "max": value, "repeats": 5} for prop in PROPERTIES}


def test_transition_scenario_keeps_px4_transition_defaults(tmp_path):
    config = _config_for_run_dir(load_config("configs/rq1_minimal.yaml"), tmp_path / "h3")
    scenario = _transition_scenario(config, 4.0)

    assert scenario.id == "px4_transition"
    assert scenario.perturb_mode == "Position"
    assert scenario.observe_mode == "Hold"
    assert scenario.t_switch_s == 4.0
    assert config.logging["jsonl"].endswith("h3/logs/queries.jsonl")


def test_return_to_neutral_by_switch_uses_input_sequence():
    config = load_config("configs/rq1_minimal.yaml")
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = np.zeros(40)
    theta[0] = 0.25

    assert _return_to_neutral_by(theta, 0.5, config, groups) is True
    assert _return_to_neutral_by(theta, 0.0, config, groups) is False
    assert _return_to_neutral_by(theta, 5.0, config, groups) is True


def test_classify_pair_requires_2std_switch_and_no_switch_separation():
    config = load_config("configs/rq1_minimal.yaml")
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = np.zeros(40)
    candidate = CandidateU("u", theta, "test")
    switch_eval = DummyEval(1, "px4_transition", 5.0, "abc", _stats(-0.05, 0.01))
    no_switch_eval = DummyEval(2, "px4_position", None, "abc", _stats(0.08, 0.01))

    rows = _classify_pair(candidate, switch_eval, no_switch_eval, "test", 5.0, config, groups)

    assert all(row["robust_transition_violation"] for row in rows)
    assert not any(row["weak_1std_candidate"] for row in rows)


def test_mark_t_specific_excludes_all_t_switch_violators():
    rows = [
        {
            "method": "m",
            "theta_hash": "h",
            "candidate_label": "u",
            "property": "p",
            "t_switch_s": 1.0,
            "robust_transition_violation": True,
        },
        {
            "method": "m",
            "theta_hash": "h",
            "candidate_label": "u",
            "property": "p",
            "t_switch_s": 2.0,
            "robust_transition_violation": True,
        },
    ]

    marked = _mark_t_specific(rows)

    assert not any(row["t_specific_window_observed"] for row in marked)


def test_mark_t_specific_flags_partial_t_switch_window():
    rows = [
        {
            "method": "m",
            "theta_hash": "h",
            "candidate_label": "u",
            "property": "p",
            "t_switch_s": 1.0,
            "robust_transition_violation": True,
        },
        {
            "method": "m",
            "theta_hash": "h",
            "candidate_label": "u",
            "property": "p",
            "t_switch_s": 2.0,
            "robust_transition_violation": False,
        },
    ]

    marked = _mark_t_specific(rows)

    assert marked[0]["t_specific_window_observed"] is True
    assert marked[1]["t_specific_window_observed"] is False
