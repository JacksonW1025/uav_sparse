import numpy as np
import pandas as pd
import pytest

from cadet.config import load_config
from cadet.properties import compute_residual_rate_metrics, compute_robustness


def make_tail_aliasing_log():
    times = np.arange(0.0, 13.02, 0.02)
    neutral = 5.0
    early = (times >= 5.0) & (times <= 7.0)
    terminal = (times >= 11.0) & (times <= 13.0)
    vx = np.zeros_like(times)
    vz = np.zeros_like(times)
    yaw_rate = np.zeros_like(times)
    vx[early] = 1.2
    vz[early] = 0.5
    yaw_rate[early] = 0.4
    vx[terminal] = 0.05
    vz[terminal] = 0.05
    yaw_rate[terminal] = 0.05
    return pd.DataFrame(
        {
            "time_s": times,
            "x_m": np.zeros_like(times),
            "y_m": np.zeros_like(times),
            "z_m": np.full_like(times, 5.0),
            "alt_m": np.full_like(times, 5.0),
            "vx_mps": vx,
            "vy_mps": np.zeros_like(times),
            "vz_mps": vz,
            "roll_rad": np.zeros_like(times),
            "pitch_rad": np.zeros_like(times),
            "yaw_rad": np.zeros_like(times),
            "yaw_rate_rps": yaw_rate,
            "mode": "Synthetic",
            "t_zero_s": 0.0,
            "t_neutral_s": neutral,
        }
    )


def test_terminal_window_separates_release_transient_from_terminal_residual():
    config = load_config("configs/rq1_minimal.yaml")
    log = make_tail_aliasing_log()

    full_tail_rho = compute_robustness(log, "post_neutral_xy_velocity", config)
    terminal_rho = compute_robustness(log, "post_neutral_xy_velocity", config, window=(11.0, 13.0))

    assert full_tail_rho == pytest.approx(-0.2)
    assert terminal_rho == pytest.approx(0.95)
    assert full_tail_rho + 2.0 * 0.0 < 0.0
    assert terminal_rho - 2.0 * 0.0 > 0.0


def test_terminal_window_does_not_change_default_xy_velocity_path():
    config = load_config("configs/rq1_minimal.yaml")
    log = make_tail_aliasing_log()

    default_rho = compute_robustness(log, "post_neutral_xy_velocity", config)
    _ = compute_robustness(log, "post_neutral_xy_velocity", config, window=(11.0, 13.0))

    assert compute_robustness(log, "post_neutral_xy_velocity", config, window=None) == pytest.approx(default_rho)
    assert compute_robustness(log, "post_neutral_xy_velocity", config) == pytest.approx(default_rho)


def test_explicit_terminal_window_is_supported_for_residual_rate_properties():
    config = load_config("configs/rq1_minimal.yaml")
    log = make_tail_aliasing_log()

    climb_terminal = compute_robustness(log, "post_neutral_climb_rate", config, window=(11.0, 13.0))
    climb_early = compute_robustness(log, "post_neutral_climb_rate", config, window=(5.0, 7.0))
    yaw_terminal = compute_robustness(log, "post_neutral_yaw_rate", config, window=(11.0, 13.0))
    yaw_early = compute_robustness(log, "post_neutral_yaw_rate", config, window=(5.0, 7.0))

    assert climb_terminal == pytest.approx(0.25)
    assert climb_early == pytest.approx(-0.2)
    assert yaw_terminal == pytest.approx(0.211799388)
    assert yaw_early == pytest.approx(-0.138200612)


def test_default_residual_rate_path_matches_existing_terminal_metrics():
    config = load_config("configs/rq1_minimal.yaml")
    log = make_tail_aliasing_log()

    metrics = compute_residual_rate_metrics(log, "post_neutral_climb_rate", config)

    assert compute_robustness(log, "post_neutral_climb_rate", config) == pytest.approx(metrics["rho_tier1"])
    assert compute_robustness(log, "post_neutral_climb_rate", config, window=None) == pytest.approx(metrics["rho_tier1"])
