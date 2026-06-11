import math

import numpy as np
import pandas as pd
import pytest

from cadet.query import _augment_parsed_log_diagnostics, _telemetry_diagnostics


def make_transition_log() -> pd.DataFrame:
    times = np.asarray([0.0, 5.0, 5.5, 6.0, 8.0, 10.0, 12.0, 13.0])
    vx = np.asarray([0.0, 1.0, 2.0, 0.5, 0.7, 0.9, 0.3, 0.2])
    return pd.DataFrame(
        {
            "time_s": times,
            "vx_mps": vx,
            "vy_mps": np.zeros_like(vx),
            "mode": ["POSCTL", "POSCTL", "LOITER", "LOITER", "LOITER", "LOITER", "LOITER", "LOITER"],
            "transition_t_switch_s": 5.0,
            "transition_first_request_t_s": 5.0,
            "transition_observed_t_s": 5.5,
            "transition_request_count": 3,
        }
    )


def test_augment_parsed_log_adds_xy_speed_and_velocity_at_transition():
    augmented = _augment_parsed_log_diagnostics(make_transition_log())

    assert "xy_speed_mps" in augmented
    assert "velocity_at_transition_mps" in augmented
    assert augmented.loc[augmented["time_s"] == 5.5, "xy_speed_mps"].iloc[0] == pytest.approx(2.0)
    assert augmented["velocity_at_transition_mps"].iloc[0] == pytest.approx(2.0)


def test_telemetry_diagnostics_records_transition_and_window_peaks():
    augmented = _augment_parsed_log_diagnostics(make_transition_log())
    diagnostics = _telemetry_diagnostics(augmented)

    assert diagnostics["transition_t_switch_s"] == pytest.approx(5.0)
    assert diagnostics["transition_observed_t_s"] == pytest.approx(5.5)
    assert diagnostics["transition_request_to_observed_delay_s"] == pytest.approx(0.5)
    assert diagnostics["velocity_at_transition_mps"] == pytest.approx(2.0)
    assert diagnostics["xy_speed_peak_5_7_mps"] == pytest.approx(2.0)
    assert diagnostics["xy_speed_peak_7_9_mps"] == pytest.approx(0.7)
    assert diagnostics["xy_speed_peak_9_11_mps"] == pytest.approx(0.9)
    assert diagnostics["xy_speed_peak_11_13_mps"] == pytest.approx(0.3)


def test_telemetry_diagnostics_omits_transition_fields_when_no_transition():
    log = pd.DataFrame({"time_s": [0.0, 12.0], "vx_mps": [0.0, 0.1], "vy_mps": [0.0, 0.0]})
    diagnostics = _telemetry_diagnostics(_augment_parsed_log_diagnostics(log))

    assert "transition_observed_t_s" not in diagnostics
    assert diagnostics["xy_speed_peak_11_13_mps"] == pytest.approx(0.1)
    assert math.isnan(diagnostics["xy_speed_peak_5_7_mps"])
