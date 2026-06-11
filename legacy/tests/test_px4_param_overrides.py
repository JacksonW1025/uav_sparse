from dataclasses import replace

import pytest
from pymavlink import mavutil

import cadet.vehicle.px4 as px4_module
from cadet.config import load_config
from cadet.vehicle.px4 import PX4Adapter


class DummyMav:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.modes = []

    def set_mode(self, mode):
        self.modes.append(mode)


def test_scenarios_default_to_empty_param_overrides():
    config = load_config("configs/rq1_minimal.yaml")

    assert all(dict(scenario.param_overrides) == {} for scenario in config.scenarios)


def test_px4_prepare_applies_param_overrides_after_setup_with_readback(monkeypatch):
    config = load_config("configs/rq1_minimal.yaml")
    scenario = replace(config.scenario_by_id("px4_position"), param_overrides={"MPC_ACC_HOR": 0.5})
    adapter = PX4Adapter(config)
    calls = []
    read_requests = []
    dummy = DummyMav()

    monkeypatch.setattr(px4_module, "start_process", lambda *args, **kwargs: object())
    monkeypatch.setattr(adapter, "shutdown", lambda: None)
    monkeypatch.setattr(adapter, "_connect", lambda *args, **kwargs: dummy)
    monkeypatch.setattr(adapter, "_request_message_interval", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_arm", lambda: None)
    monkeypatch.setattr(adapter, "_wait_altitude", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_send_neutral_manual_for", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_set_mode", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_manual_climb_to_altitude", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_set_scenario_mode", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_wait_hover_stable", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_parameter_metadata", lambda name: {"name": name, "type": "Float"})

    def fake_set_param(name, value, param_type, timeout_s=5.0):
        calls.append((name, value, param_type))
        return True

    def fake_read_param(name, timeout_s=5.0):
        read_requests.append(name)
        values = {
            "MPC_ACC_HOR": 0.5,
            "MPC_JERK_MAX": 8.0,
        }
        return values[name], mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    monkeypatch.setattr(adapter, "_set_param", fake_set_param)
    monkeypatch.setattr(adapter, "_read_param", fake_read_param)

    adapter.prepare(scenario, seed=0)

    assert calls == [
        ("COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
        ("MIS_TAKEOFF_ALT", 5.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ("MPC_ACC_HOR", 0.5, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ("MPC_JERK_MAX", 8.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ]
    assert read_requests == ["MPC_ACC_HOR", "MPC_JERK_MAX"]
    assert adapter.timing["param_override_MPC_ACC_HOR_target"] == pytest.approx(0.5)
    assert adapter.timing["param_override_MPC_ACC_HOR_readback"] == pytest.approx(0.5)
    assert adapter.timing["param_override_MPC_ACC_HOR_reboot_required"] is False
    assert adapter.timing["param_override_MPC_JERK_MAX_target"] == pytest.approx(8.0)
    assert adapter.timing["param_override_MPC_JERK_MAX_readback"] == pytest.approx(8.0)


def test_px4_prepare_empty_param_overrides_applies_explicit_defaults(monkeypatch):
    config = load_config("configs/rq1_minimal.yaml")
    scenario = config.scenario_by_id("px4_position")
    adapter = PX4Adapter(config)
    calls = []
    read_requests = []
    dummy = DummyMav()

    monkeypatch.setattr(px4_module, "start_process", lambda *args, **kwargs: object())
    monkeypatch.setattr(adapter, "shutdown", lambda: None)
    monkeypatch.setattr(adapter, "_connect", lambda *args, **kwargs: dummy)
    monkeypatch.setattr(adapter, "_request_message_interval", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_arm", lambda: None)
    monkeypatch.setattr(adapter, "_wait_altitude", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_send_neutral_manual_for", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_set_mode", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_manual_climb_to_altitude", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_set_scenario_mode", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_wait_hover_stable", lambda *args, **kwargs: None)
    monkeypatch.setattr(adapter, "_parameter_metadata", lambda name: {"name": name, "type": "Float"})

    def fake_read_param(name, timeout_s=5.0):
        read_requests.append(name)
        values = {
            "MPC_ACC_HOR": 3.0,
            "MPC_JERK_MAX": 8.0,
        }
        return values[name], mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    def fake_set_param(name, value, param_type, timeout_s=5.0):
        calls.append((name, value, param_type))
        return True

    monkeypatch.setattr(adapter, "_set_param", fake_set_param)
    monkeypatch.setattr(adapter, "_read_param", fake_read_param)

    adapter.prepare(scenario, seed=0)

    assert calls == [
        ("COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
        ("MIS_TAKEOFF_ALT", 5.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ("MPC_ACC_HOR", 3.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ("MPC_JERK_MAX", 8.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ]
    assert read_requests == ["MPC_ACC_HOR", "MPC_JERK_MAX"]
    assert adapter.timing["param_override_MPC_ACC_HOR_readback"] == pytest.approx(3.0)
    assert adapter.timing["param_override_MPC_JERK_MAX_readback"] == pytest.approx(8.0)


def test_px4_param_override_readback_mismatch_raises(monkeypatch):
    config = load_config("configs/rq1_minimal.yaml")
    scenario = replace(config.scenario_by_id("px4_position"), param_overrides={"MPC_ACC_HOR": 0.5})
    adapter = PX4Adapter(config)
    monkeypatch.setattr(adapter, "_parameter_metadata", lambda name: {"name": name, "type": "Float"})
    monkeypatch.setattr(adapter, "_set_param", lambda *args, **kwargs: True)
    monkeypatch.setattr(adapter, "_read_param", lambda *args, **kwargs: (0.6, mavutil.mavlink.MAV_PARAM_TYPE_REAL32))

    with pytest.raises(RuntimeError, match="readback mismatch"):
        adapter._apply_param_overrides(scenario)


def test_px4_param_override_reboot_required_raises_before_setting(monkeypatch):
    config = load_config("configs/rq1_minimal.yaml")
    scenario = replace(config.scenario_by_id("px4_position"), param_overrides={"MPC_ACC_HOR": 0.5})
    adapter = PX4Adapter(config)
    monkeypatch.setattr(
        adapter,
        "_parameter_metadata",
        lambda name: {"name": name, "type": "Float", "rebootRequired": True},
    )
    monkeypatch.setattr(adapter, "_set_param", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("unexpected set")))

    with pytest.raises(RuntimeError, match="requires reboot"):
        adapter._apply_param_overrides(scenario)
