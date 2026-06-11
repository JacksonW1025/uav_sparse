import numpy as np
import pandas as pd

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.mantis.maneuvers import ManeuverSpec, default_maneuvers, stress_metrics


def _changed_channels(theta, config):
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    return {group.channel for group in groups if abs(theta[group.group_id]) > 1e-12}


def test_m0_is_all_neutral():
    config = load_config("configs/mantis_pilot.yaml")
    maneuver = default_maneuvers("roll")["M0"][0]

    assert np.allclose(maneuver.to_theta(config), 0.0)
    assert np.allclose(maneuver.to_sequence(config)[config.input["channels"]].to_numpy(), 0.0)


def test_small_step_roll_only_changes_roll_support():
    config = load_config("configs/mantis_pilot.yaml")
    maneuver = ManeuverSpec("small_step_roll", "M_small", "roll", "step", 0.20, 1)

    assert _changed_channels(maneuver.to_theta(config), config) == {"roll"}


def test_strong_doublet_roll_returns_to_neutral_and_has_neutral_tail():
    config = load_config("configs/mantis_pilot.yaml")
    maneuver = ManeuverSpec("strong_doublet_roll", "M_strong", "roll", "doublet", 0.7, 1)
    sequence = maneuver.to_sequence(config)
    tail = sequence[sequence["t_s"] >= config.input["horizon_s"]]

    assert sequence.loc[sequence["t_s"] >= 1.0, "roll"].iloc[0] == 0.0
    assert np.allclose(tail[config.input["channels"]].to_numpy(), 0.0)


def test_pitch_support_does_not_touch_roll():
    config = load_config("configs/mantis_pilot.yaml")
    maneuver = ManeuverSpec("small_doublet_pitch", "M_small", "pitch", "doublet", 0.20, 1)

    assert _changed_channels(maneuver.to_theta(config), config) == {"pitch"}


def test_coupled_maneuver_sets_secondary_channel_and_returns_neutral():
    config = load_config("configs/mantis_pilot.yaml")
    maneuver = ManeuverSpec(
        "pitch_with_yaw",
        "M_strong",
        "pitch",
        "step",
        0.9,
        hold_windows=2,
        couplings={"yaw": 0.3},
    )
    sequence = maneuver.to_sequence(config)
    active = sequence[sequence["t_s"] < maneuver.release_time_s(config)]
    released = sequence[sequence["t_s"] >= maneuver.release_time_s(config)]

    assert _changed_channels(maneuver.to_theta(config), config) == {"pitch", "yaw"}
    assert active["pitch"].abs().max() > 0.0
    assert active["yaw"].abs().max() > 0.0
    assert released["yaw"].abs().max() == 0.0


def test_expanded_stress_maneuvers_have_release_time_and_neutral_tail():
    config = load_config("configs/mantis_pilot.yaml")
    required = {
        "strong_doublet_A0p7_hold1",
        "strong_doublet_A0p9_hold1",
        "strong_doublet_A0p9_hold2",
        "pulse_train_A0p7_repeat3",
        "pulse_train_A0p9_repeat3",
        "reversal_A0p9_fast",
    }
    maneuvers = {m.name: m for m in default_maneuvers("roll", max_strong=10)["M_strong"]}

    assert required.issubset(maneuvers)
    for name in required:
        maneuver = maneuvers[name]
        record = maneuver.to_record(config)
        sequence = maneuver.to_sequence(config)
        tail = sequence[sequence["t_s"] >= config.input["horizon_s"]]

        assert 0.0 < record["release_time_s"] <= config.input["horizon_s"]
        assert record["neutral_tail_s"] >= 8.0
        assert np.allclose(tail[config.input["channels"]].to_numpy(), 0.0)


def test_stress_metrics_record_active_excitation_and_setpoint_energy():
    config = load_config("configs/mantis_pilot.yaml")
    times = np.array([0.0, 0.5, 1.0, 5.5])
    parsed = pd.DataFrame(
        {
            "time_s": times,
            "roll_rate_rps": [0.0, 0.4, -0.2, 9.0],
            "pitch_rate_rps": [0.0, 0.1, 0.0, 9.0],
            "yaw_rate_rps": [0.0, 0.0, 0.0, 9.0],
            "manual_roll": [0.0, 0.5, 0.5, 0.0],
            "roll_rate_setpoint_rps": [0.0, 0.3, 0.3, 0.0],
        }
    )

    metrics = stress_metrics(parsed, ManeuverSpec("strong", "M_strong", "roll", "step", 0.5, 1), config)

    assert metrics["peak_abs_roll_rate_active"] == 0.4
    assert metrics["manual_axis_energy"] > 0.0
    assert metrics["rate_setpoint_energy"] > 0.0
