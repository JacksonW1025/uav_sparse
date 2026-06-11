import numpy as np
import pandas as pd
import pytest

from cadet.config import load_config
from cadet.properties import compute_residual_rate_metrics, compute_robustness, summarize_residual_rate_repeats
from cadet.mantis.contracts import residual_rate_repeat_summary


def _rate_log(roll_rate, pitch_rate, yaw_rate=0.0):
    times = np.arange(0.0, 13.02, 0.02)
    neutral = 5.0
    tail = np.clip(times - neutral, 0.0, None)
    roll = roll_rate(tail)
    pitch = pitch_rate(tail)
    yaw = yaw_rate(tail) if callable(yaw_rate) else np.full_like(times, float(yaw_rate))
    return pd.DataFrame(
        {
            "time_s": times,
            "x_m": np.zeros_like(times),
            "y_m": np.zeros_like(times),
            "z_m": np.full_like(times, 5.0),
            "alt_m": np.full_like(times, 5.0),
            "vx_mps": np.zeros_like(times),
            "vy_mps": np.zeros_like(times),
            "vz_mps": np.zeros_like(times),
            "roll_rad": np.zeros_like(times),
            "pitch_rad": np.zeros_like(times),
            "yaw_rad": np.zeros_like(times),
            "roll_rate_rps": roll,
            "pitch_rate_rps": pitch,
            "yaw_rate_rps": yaw,
            "mode": "Synthetic",
            "t_zero_s": 0.0,
            "t_neutral_s": neutral,
        }
    )


def test_roll_residual_rate_decay_is_safe():
    config = load_config("configs/mantis_pilot.yaml")
    log = _rate_log(lambda tail: np.where(tail > 0, np.exp(-tail / 1.0), 0.0), lambda tail: np.zeros_like(tail))

    assert compute_robustness(log, "post_neutral_roll_rate", config) > 0.0
    summary = residual_rate_repeat_summary([log] * 3, "post_neutral_roll_rate", config)

    assert summary["tier1_robustness_class"] == "robust_safe"
    assert summary["contract_class"] == "safe"


def test_roll_terminal_high_and_nondecay_is_violation_like():
    config = load_config("configs/mantis_pilot.yaml")
    log = _rate_log(lambda tail: np.where(tail > 0, 0.5, 0.0), lambda tail: np.zeros_like(tail))
    summary = residual_rate_repeat_summary([log] * 3, "post_neutral_roll_rate", config)

    assert summary["terminal_peak_abs_rate_mean"] == pytest.approx(0.5)
    assert summary["tier2_robustness_class"] == "robust_violation"
    assert summary["contract_class"] == "violation_like"


def test_pitch_terminal_high_and_nondecay_is_violation_like():
    config = load_config("configs/mantis_pilot.yaml")
    log = _rate_log(lambda tail: np.zeros_like(tail), lambda tail: np.where(tail > 0, -0.6, 0.0))
    summary = residual_rate_repeat_summary([log] * 3, "post_neutral_pitch_rate", config)

    assert summary["terminal_peak_abs_rate_mean"] == pytest.approx(0.6)
    assert summary["tier2_robustness_class"] == "robust_violation"


def test_yaw_residual_rate_existing_logic_is_unchanged():
    config = load_config("configs/mantis_pilot.yaml")
    log = _rate_log(lambda tail: np.zeros_like(tail), lambda tail: np.zeros_like(tail), yaw_rate=0.1)
    summary = residual_rate_repeat_summary([log] * 3, "post_neutral_yaw_rate", config)

    assert compute_robustness(log, "post_neutral_yaw_rate", config) == pytest.approx(0.161799388)
    metrics = compute_residual_rate_metrics(log, "post_neutral_yaw_rate", config)
    assert summarize_residual_rate_repeats([metrics])["tier1_robustness_class"] == "robust_safe"
    assert summary["contract_class"] == "safe"
