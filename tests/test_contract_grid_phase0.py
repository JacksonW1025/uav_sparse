import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from cadet.config import load_config
from cadet.groups import build_groups
from cadet.runners.contract_grid_phase0 import (
    CELL_SPECS,
    build_cell_probes,
    contract_thresholds,
    load_px4_parameter_defaults,
    peak_for_axis,
)
from cadet.violation_search import theta_to_grid


def make_log() -> pd.DataFrame:
    times = np.arange(0.0, 13.02, 0.02)
    terminal = (times >= 11.0) & (times <= 13.0)
    active = (times >= 2.0) & (times <= 5.0)
    return pd.DataFrame(
        {
            "time_s": times,
            "vx_mps": np.where(terminal, 0.6, 0.0),
            "vy_mps": np.where(terminal, 0.8, 0.0),
            "vz_mps": np.where(active, -1.2, np.where(terminal, 0.25, 0.0)),
            "yaw_rate_rps": np.where(terminal, -0.2, 0.0),
            "roll_rad": np.where(terminal, 0.03, 0.0),
            "pitch_rad": np.where(terminal, 0.04, 0.0),
        }
    )


def test_contract_axis_peaks_match_preregistered_definitions():
    log = make_log()

    assert peak_for_axis(log, "xy_velocity", (11.0, 13.0)) == pytest.approx(1.0)
    assert peak_for_axis(log, "climb_rate", (11.0, 13.0)) == pytest.approx(0.25)
    assert peak_for_axis(log, "climb_rate_down", (2.0, 5.0)) == pytest.approx(1.2)
    assert peak_for_axis(log, "yaw_rate", (11.0, 13.0)) == pytest.approx(0.2)
    assert peak_for_axis(log, "tilt", (11.0, 13.0)) == pytest.approx(
        math.acos(math.cos(0.03) * math.cos(0.04))
    )


def test_px4_parameter_defaults_are_converted_to_metric_units(tmp_path: Path):
    params_path = tmp_path / "parameters.json"
    rows = [
        ("MPC_HOLD_DZ", 0.1, ""),
        ("MPC_VEL_MANUAL", 10.0, "m/s"),
        ("MPC_Z_VEL_MAX_UP", 3.0, "m/s"),
        ("MPC_Z_VEL_MAX_DN", 1.5, "m/s"),
        ("MPC_MAN_Y_MAX", 150.0, "deg/s"),
        ("MPC_XY_VEL_MAX", 12.0, "m/s"),
        ("MPC_MAN_TILT_MAX", 35.0, "deg"),
    ]
    params_path.write_text(
        json.dumps({"parameters": [{"name": name, "default": value, "units": units} for name, value, units in rows]}),
        encoding="utf-8",
    )

    params = load_px4_parameter_defaults(params_path)
    thresholds = contract_thresholds(params)

    assert params["MPC_MAN_Y_MAX"].value == pytest.approx(math.radians(150.0))
    assert params["MPC_MAN_TILT_MAX"].value == pytest.approx(math.radians(35.0))
    assert thresholds["C1"]["yaw_rate"] == pytest.approx(math.radians(15.0))
    assert thresholds["C2"]["tilt"] == pytest.approx(math.radians(3.5))
    assert thresholds["C3"]["tilt"] == pytest.approx(math.radians(35.0))


def test_saturated_probe_generation_obeys_rate_projection():
    base = load_config("configs/rq1_minimal.yaml")
    config = base.__class__(
        path=base.path,
        experiment_id=base.experiment_id,
        input={**base.input, "min_value": -1.0, "max_value": 1.0},
        properties=base.properties,
        scenarios=base.scenarios,
        seeds=base.seeds,
        persistence_path=base.persistence_path,
        simulator=base.simulator,
        logging=base.logging,
    )
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    g01 = next(cell for cell in CELL_SPECS if cell.cell_id == "G01")
    probes = build_cell_probes(g01, config, groups)
    plus = next(probe for probe in probes if probe.label == "roll_plus_full")
    grid = theta_to_grid(plus.theta, config, groups)
    roll = grid[:, list(config.input["channels"]).index("roll")]

    assert np.max(np.abs(plus.theta)) == pytest.approx(1.0)
    assert roll[:4].tolist() == pytest.approx([0.25, 0.5, 0.75, 1.0])
    assert np.all(roll[3:] == pytest.approx(1.0))
