from __future__ import annotations

import numpy as np
import pytest

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.input_model import theta_to_sequence
from cadet.runners.transition_handoff_v2 import (
    PARAM_PINS,
    V_STRESS_MPS,
    _config_for_run_dir,
    _position_scenario,
    _relative_subwindows,
    _robustness_label,
    _terminal_window,
    _transition_scenario,
    generate_profiles,
)


def test_generated_profiles_are_neutral_from_t_switch():
    config = _config_for_run_dir(load_config("configs/rq1_minimal.yaml"), "runs/test_a04", max_t_switch_s=10.0)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    profiles = generate_profiles(config, groups, [0.85], [5.0, 10.0], ["pitch"], ["long_hold"])

    assert profiles
    for profile in profiles:
        sequence = theta_to_sequence(profile.theta, groups, config)
        post = sequence[sequence["t_s"] >= profile.t_switch_s]
        assert float(post[config.input["channels"]].abs().max().max()) == pytest.approx(0.0)


def test_long_hold_profile_respects_rate_limit_and_has_hold_segment():
    config = _config_for_run_dir(load_config("configs/rq1_minimal.yaml"), "runs/test_a04", max_t_switch_s=10.0)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    [profile] = generate_profiles(config, groups, [0.85], [10.0], ["pitch"], ["long_hold"])
    sequence = theta_to_sequence(profile.theta, groups, config)
    pitch = sequence.loc[sequence["t_s"] < profile.t_switch_s, "pitch"].to_numpy()
    window_pitch = pitch[:: int(float(config.input.get("manual_control_hz", 50)) * float(config.input["window_s"]))]
    diffs = np.diff(np.r_[0.0, window_pitch])

    assert np.max(np.abs(diffs)) <= float(config.input["max_delta_per_window"]) + 1e-9
    assert np.count_nonzero(np.isclose(window_pitch, profile.effective_amplitude)) >= 4


def test_requested_amplitudes_are_projected_without_relaxing_f():
    config = _config_for_run_dir(load_config("configs/rq1_minimal.yaml"), "runs/test_a04", max_t_switch_s=10.0)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    profiles = generate_profiles(config, groups, [0.85, 1.0], [10.0], ["pitch"], ["long_hold"])

    assert {profile.requested_amplitude for profile in profiles} == {0.85, 1.0}
    for profile in profiles:
        assert profile.request_saturated is True
        assert profile.effective_amplitude == pytest.approx(config.input["max_value"])
        assert float(np.max(np.abs(profile.theta))) <= float(config.input["max_value"])


def test_terminal_and_subwindows_shift_with_t_switch():
    assert _terminal_window(8.0) == pytest.approx((14.0, 16.0))
    assert _relative_subwindows(8.0) == pytest.approx([(8.0, 10.0), (10.0, 12.0), (12.0, 14.0), (14.0, 16.0)])


def test_default_params_are_explicit_in_transition_and_position_scenarios():
    config = _config_for_run_dir(load_config("configs/rq1_minimal.yaml"), "runs/test_a04", max_t_switch_s=10.0)

    assert dict(_transition_scenario(config, 10.0).param_overrides) == PARAM_PINS
    assert dict(_position_scenario(config).param_overrides) == PARAM_PINS


def test_robustness_label_uses_frozen_two_sigma_rules():
    assert _robustness_label(-0.10, 0.04) == "robust_violation"
    assert _robustness_label(0.10, 0.04) == "robust_safe"
    assert _robustness_label(0.01, 0.04) == "noise_band"


def test_v_stress_is_frozen_to_twice_terminal_threshold():
    config = load_config("configs/rq1_minimal.yaml")

    assert V_STRESS_MPS == pytest.approx(2.0 * config.properties["post_neutral_xy_velocity"]["v_max_mps"])
