import numpy as np
import pandas as pd
import pytest

from cadet.config import load_config
from cadet.properties import compute_all_properties, compute_residual_rate_metrics, compute_robustness, summarize_residual_rate_repeats


def make_log():
    times = np.arange(0.0, 13.02, 0.02)
    neutral = 5.0
    tail = np.clip(times - neutral, 0.0, None)
    return pd.DataFrame(
        {
            "time_s": times,
            "x_m": 0.5 * (tail > 0),
            "y_m": np.zeros_like(times),
            "z_m": 5.0 + 0.2 * (tail > 0),
            "alt_m": 5.0 + 0.2 * (tail > 0),
            "vx_mps": 0.3 * (tail > 0),
            "vy_mps": np.zeros_like(times),
            "vz_mps": 0.2 * (tail > 0),
            "roll_rad": np.zeros_like(times),
            "pitch_rad": np.zeros_like(times),
            "yaw_rad": np.zeros_like(times),
            "yaw_rate_rps": 0.1 * (tail > 0),
            "mode": "Synthetic",
            "t_zero_s": 0.0,
            "t_neutral_s": neutral,
        }
    )


def test_compute_robustness_values_are_finite_and_positive():
    config = load_config("configs/synthetic_sanity.yaml")
    log = make_log()
    assert compute_robustness(log, "post_neutral_xy_drift", config) == pytest.approx(1.5)
    assert compute_robustness(log, "post_neutral_alt_drift", config) == pytest.approx(0.8)
    assert compute_robustness(log, "post_neutral_xy_velocity", config) == pytest.approx(0.7)


def test_compute_all_properties():
    config = load_config("configs/synthetic_sanity.yaml")
    log = make_log()
    values = compute_all_properties(log, config.scenarios[0].properties, config)
    assert set(values) == set(config.scenarios[0].properties)
    assert all(np.isfinite(v) for v in values.values())


def test_residual_rate_properties_use_terminal_tail_thresholds():
    config = load_config("configs/rq1_minimal.yaml")
    log = make_log()

    assert compute_robustness(log, "post_neutral_climb_rate", config) == pytest.approx(0.1)
    assert compute_robustness(log, "post_neutral_yaw_rate", config) == pytest.approx(0.161799388)

    metrics = compute_residual_rate_metrics(log, "post_neutral_climb_rate", config)
    summary = summarize_residual_rate_repeats([metrics] * 5)

    assert summary["terminal_peak_abs_rate_mean"] == pytest.approx(0.2)
    assert summary["tier1_robustness_class"] == "robust_safe"
    assert summary["tier2_robustness_class"] == "not_tier1_violation"
