import numpy as np
import pandas as pd

from cadet.mantis.nonlinear import analyze_nonlinear_topics


def _time(n=100, dt=0.02):
    return np.arange(n, dtype=float) * dt


def _manual(active_axis="roll", n=100):
    t = _time(n)
    values = np.zeros(n)
    values[(t >= 0.2) & (t <= 0.8)] = 0.8
    return pd.DataFrame(
        {
            "time_s": t,
            "roll": values if active_axis == "roll" else np.zeros(n),
            "pitch": values if active_axis == "pitch" else np.zeros(n),
            "yaw": np.zeros(n),
        }
    )


def test_normalized_actuator_sustained_saturation_activates():
    t = _time()
    motor = np.full_like(t, 0.5)
    motor[(t >= 0.3) & (t <= 0.7)] = 1.0
    diag = analyze_nonlinear_topics(
        {
            "manual_control_setpoint": _manual(),
            "actuator_motors": pd.DataFrame({"time_s": t, "control[0]": motor}),
        },
        active_axis="roll",
    )

    assert diag["nonlinear_observability"] is True
    assert diag["actuator_available"] is True
    assert diag["actuator_sat_ratio"] >= 0.05
    assert diag["actuator_sat_consecutive_s"] >= 0.20
    assert diag["nonlinear_activated"] is True


def test_one_sample_actuator_saturation_does_not_activate():
    t = _time()
    motor = np.full_like(t, 0.5)
    motor[20] = 1.0
    diag = analyze_nonlinear_topics(
        {
            "manual_control_setpoint": _manual(),
            "actuator_motors": pd.DataFrame({"time_s": t, "control[0]": motor}),
        },
        active_axis="roll",
    )

    assert diag["nonlinear_observability"] is True
    assert diag["nonlinear_activated"] is False


def test_no_actuator_or_status_topics_is_not_observable():
    t = _time()
    diag = analyze_nonlinear_topics(
        {
            "vehicle_angular_velocity": pd.DataFrame(
                {"time_s": t, "xyz[0]": np.sin(t), "xyz[1]": np.zeros_like(t), "xyz[2]": np.zeros_like(t)}
            )
        },
        active_axis="roll",
    )

    assert diag["rate_energy_available"] is True
    assert diag["nonlinear_observability"] is False
    assert diag["nonlinear_activated"] is False


def test_explicit_saturation_flag_activates():
    t = _time()
    flag = np.zeros_like(t)
    flag[10] = 1
    diag = analyze_nonlinear_topics(
        {
            "rate_ctrl_status": pd.DataFrame({"time_s": t, "roll_saturation": flag}),
        },
        active_axis="roll",
    )

    assert diag["explicit_saturation_flag_available"] is True
    assert diag["explicit_saturation_flag_active"] is True
    assert diag["nonlinear_observability"] is True
    assert diag["nonlinear_activated"] is True


def test_cross_axis_energy_does_not_activate_by_itself():
    t = _time()
    diag = analyze_nonlinear_topics(
        {
            "vehicle_angular_velocity": pd.DataFrame(
                {"time_s": t, "xyz[0]": np.ones_like(t), "xyz[1]": np.ones_like(t), "xyz[2]": np.zeros_like(t)}
            )
        },
        active_axis="roll",
    )

    assert diag["cross_axis_energy_ratio"] > 0.0
    assert diag["nonlinear_observability"] is False
    assert diag["nonlinear_activated"] is False
