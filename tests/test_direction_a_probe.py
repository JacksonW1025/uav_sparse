from __future__ import annotations

from argparse import Namespace
from pathlib import Path

import numpy as np

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.input_model import project_theta
from cadet.runners.direction_a_probe import (
    INTERIOR_MAX_ABS,
    SATURATED_MIN_ABS,
    EnvelopeSpec,
    _config_for_probe,
    _pre_registration,
    classify_amplitude,
    classify_robustness,
    derive_A_phi,
    envelope_theta,
    support_summary,
)


def test_robustness_classifier_uses_two_sigma_gate():
    assert classify_robustness(-0.30, 0.10) == "robust_violation"
    assert classify_robustness(-0.10, 0.10) == "noise_band"
    assert classify_robustness(0.30, 0.10) == "robust_safe"
    assert classify_robustness(0.10, 0.10) == "noise_band"


def test_amplitude_classifier_registered_bins():
    assert classify_amplitude(INTERIOR_MAX_ABS) == "interior"
    assert classify_amplitude(0.50001) == "moderate"
    assert classify_amplitude(SATURATED_MIN_ABS) == "moderate"
    assert classify_amplitude(0.90001) == "saturated"


def test_support_summary_reports_size_and_channels():
    config = load_config("configs/rq1_minimal.yaml")
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    theta = np.zeros(len(groups))
    theta[0] = 0.11
    theta[1] = -0.20
    theta[2] = 0.09

    summary = support_summary(theta, groups)

    assert summary["support_size"] == 2
    assert summary["active_channels"] == ["pitch", "roll"]
    assert summary["active_group_ids"] == [0, 1]


def test_derive_a_phi_includes_residual_rate_channels():
    assert derive_A_phi("post_neutral_xy_velocity") == ["roll", "pitch"]
    assert derive_A_phi("post_neutral_alt_drift") == ["throttle"]
    assert derive_A_phi("post_neutral_climb_rate") == ["throttle"]
    assert derive_A_phi("post_neutral_yaw_rate") == ["yaw"]


def test_probe_config_uses_stick_limit_without_mutating_base(tmp_path: Path):
    base = load_config("configs/rq1_minimal.yaml")
    changed = _config_for_probe(base, tmp_path / "direction_a", 1.0)

    assert changed.input["min_value"] == -1.0
    assert changed.input["max_value"] == 1.0
    assert base.input["max_value"] == 0.7
    assert changed.logging["jsonl"].endswith("direction_a/logs/queries.jsonl")


def test_envelope_theta_uses_only_roll_pitch_and_stays_feasible():
    base = load_config("configs/rq1_minimal.yaml")
    config = _config_for_probe(base, Path("runs/test_direction_a"), 1.0)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    spec = EnvelopeSpec(index=0, angle_rad=np.pi / 4.0, amplitude=1.0, onset_window=2, duration_windows=4)

    theta = envelope_theta(spec, config, groups)

    active = support_summary(theta, groups)
    assert set(active["active_channels"]) <= {"roll", "pitch"}
    assert np.allclose(project_theta(theta, config), theta)
    assert np.max(np.abs(theta)) <= 1.0


def test_pre_registration_records_fixed_thresholds(tmp_path: Path):
    base = load_config("configs/rq1_minimal.yaml")
    config = _config_for_probe(base, tmp_path / "direction_a", 1.0)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    args = Namespace(
        scenario="px4_position",
        seed=0,
        points_per_arm=80,
        repeats=5,
        rng_seed=123,
        bisection_iters=7,
    )

    pre = _pre_registration(args, config, groups)

    assert pre["matched_budget"]["j5_points_per_arm"] == 80
    assert pre["thresholds"]["interior_max_abs_theta"] == 0.5
    assert pre["thresholds"]["saturated_min_abs_theta"] == 0.9
    assert pre["channel_relevant_set_for_xy_velocity"] == ["roll", "pitch"]
