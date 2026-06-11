from dataclasses import replace

import pytest
from pymavlink import mavutil

from cadet.config import load_config
from cadet.vehicle.ardupilot import ArduPilotAdapter


def test_ardupilot_param_overrides_use_readback_type_and_verify(monkeypatch):
    config = load_config("configs/mantis_pilot.yaml")
    scenario = config.scenario_by_id("ap_stabilize_roll")
    scenario = replace(scenario, param_overrides={"ATC_RAT_RLL_P": 0.22})
    adapter = ArduPilotAdapter(config)
    reads = []
    sets = []

    def fake_read(name, timeout_s=5.0):
        reads.append(name)
        value = 0.10 if len(reads) == 1 else 0.22
        return value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    def fake_set(name, value, param_type, timeout_s=5.0):
        sets.append((name, value, param_type))
        return True

    monkeypatch.setattr(adapter, "_read_param", fake_read)
    monkeypatch.setattr(adapter, "_set_param", fake_set)

    adapter._apply_param_overrides(scenario)

    assert sets == [("ATC_RAT_RLL_P", 0.22, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)]
    assert adapter.timing["param_override_ATC_RAT_RLL_P_target"] == pytest.approx(0.22)
    assert adapter.timing["param_override_ATC_RAT_RLL_P_readback"] == pytest.approx(0.22)
