import numpy as np
import pandas as pd

from cadet.mantis.tracking_contracts import evaluate_tracking_contract_from_topics


def _topics(*, actual_scale=0.2, saturated=True):
    t = np.arange(0.0, 1.5, 0.02)
    manual = np.zeros_like(t)
    manual[(t >= 0.2) & (t <= 1.0)] = 0.8
    sp = np.zeros_like(t)
    sp[(t >= 0.2) & (t <= 1.0)] = 2.0
    actual = sp * actual_scale
    motor = np.full_like(t, 0.5)
    if saturated:
        motor[(t >= 0.3) & (t <= 0.9)] = 1.0
    return {
        "manual_control_setpoint": pd.DataFrame(
            {"time_s": t, "roll": manual, "pitch": np.zeros_like(t), "yaw": np.zeros_like(t)}
        ),
        "vehicle_rates_setpoint": pd.DataFrame(
            {"time_s": t, "roll": sp, "pitch": np.zeros_like(t), "yaw": np.zeros_like(t)}
        ),
        "vehicle_angular_velocity": pd.DataFrame(
            {"time_s": t, "xyz[0]": actual, "xyz[1]": np.zeros_like(t), "xyz[2]": np.zeros_like(t)}
        ),
        "actuator_motors": pd.DataFrame({"time_s": t, "control[0]": motor}),
    }


def test_tracking_violation_requires_high_error_and_nonlinear_overlap():
    result = evaluate_tracking_contract_from_topics(_topics(), axis="roll", baseline_nte_median=0.1)

    assert result["C_track_available"] is True
    assert result["nte"] >= 0.65
    assert result["peak_err"] >= 1.0
    assert result["high_err_duration_s"] >= 0.25
    assert result["saturation_error_overlap_s"] >= 0.20
    assert result["C_track_violation"] is True


def test_tracking_high_error_without_overlap_is_safe():
    result = evaluate_tracking_contract_from_topics(_topics(saturated=False), axis="roll", baseline_nte_median=0.1)

    assert result["high_err_duration_s"] >= 0.25
    assert result["saturation_error_overlap_s"] == 0.0
    assert result["C_track_violation"] is False
    assert result["C_track_safe"] is True


def test_tracking_missing_setpoint_is_unavailable_not_violation():
    topics = _topics()
    topics.pop("vehicle_rates_setpoint")

    result = evaluate_tracking_contract_from_topics(topics, axis="roll", baseline_nte_median=0.1)

    assert result["C_track_status"] == "C_track_unavailable"
    assert result["C_track_violation"] is False


def test_tracking_no_active_window_is_safe():
    topics = _topics()
    topics["manual_control_setpoint"]["roll"] = 0.0
    topics["vehicle_rates_setpoint"]["roll"] = 0.0

    result = evaluate_tracking_contract_from_topics(topics, axis="roll", baseline_nte_median=0.1)

    assert result["active_window_available"] is False
    assert result["C_track_status"] == "safe"
