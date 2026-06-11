import json
import time
from pathlib import Path

import numpy as np
import pytest

from cadet.config import load_config
from cadet.query import run_query
from cadet.vehicle.px4 import PX4Adapter


def test_px4_acro_roll_test_mode_is_acro_not_posctl():
    config = load_config("configs/mantis_pilot.yaml")
    scenario = config.scenario_by_id("px4_acro_roll")
    adapter = PX4Adapter(config)

    assert adapter._scenario_test_mode(scenario) == "ACRO"
    assert adapter._scenario_staging_mode(scenario) is None


def test_px4_stabilized_roll_test_mode_is_stabilized_not_posctl():
    config = load_config("configs/mantis_pilot.yaml")
    scenario = config.scenario_by_id("px4_stabilized_roll")
    adapter = PX4Adapter(config)

    assert adapter._scenario_test_mode(scenario) == "STABILIZED"
    assert adapter._scenario_staging_mode(scenario) is None


def test_px4_pitch_aliases_keep_generic_mode_harness():
    config = load_config("configs/mantis_pilot.yaml")
    acro_pitch = config.scenario_by_id("px4_acro_pitch")
    stabilized_pitch = config.scenario_by_id("px4_stabilized_pitch")
    adapter = PX4Adapter(config)

    assert adapter._scenario_test_mode(acro_pitch) == "ACRO"
    assert adapter._scenario_staging_mode(acro_pitch) is None
    assert adapter._scenario_test_mode(stabilized_pitch) == "STABILIZED"
    assert adapter._scenario_staging_mode(stabilized_pitch) is None


def test_old_px4_position_still_uses_posctl():
    config = load_config("configs/rq1_minimal.yaml")
    scenario = config.scenario_by_id("px4_position")
    adapter = PX4Adapter(config)

    assert adapter._scenario_test_mode(scenario) == "POSCTL"
    assert adapter._scenario_staging_mode(scenario) == "POSCTL"


def test_mode_trace_records_failure_entry():
    config = load_config("configs/mantis_pilot.yaml")
    adapter = PX4Adapter(config)
    adapter.timing = {}
    adapter.mode_trace = []
    adapter.mode_trace_zero_s = time.monotonic()

    adapter._record_mode_trace(
        requested_mode="ACRO",
        observed_mode_before="LOITER",
        ack_result=1,
        ack="result=1 progress=0 result_param2=0",
        observed_mode_after="LOITER",
        success=False,
        request_count=3,
        reason="last_ack=result=1 progress=0 result_param2=0",
    )

    assert adapter.timing["mode_trace"][0]["requested_mode"] == "ACRO"
    assert adapter.timing["mode_trace"][0]["success"] is False


def test_px4_manual_mode_prefeed_is_configurable():
    config = load_config("configs/mantis_pilot.yaml")
    adapter = PX4Adapter(config)

    assert adapter._manual_mode_prefeed_s() == 1.2

    adapter.sim_cfg = {"manual_mode_prefeed_s": 2.5}
    assert adapter._manual_mode_prefeed_s() == 2.5


def test_failed_test_mode_switch_is_harness_failure_not_contract_result(monkeypatch, tmp_path):
    class FailingAdapter:
        def __init__(self):
            self.timing = {
                "target_test_mode": "ACRO",
                "staging_mode_used": "",
                "mode_trace": [
                    {
                        "time_s": 1.0,
                        "requested_mode": "ACRO",
                        "observed_mode_before": "LOITER",
                        "ack_result": 1,
                        "ack": "result=1 progress=0 result_param2=0",
                        "observed_mode_after": "LOITER",
                        "success": False,
                        "request_count": 3,
                        "reason": "last_ack=result=1 progress=0 result_param2=0",
                    }
                ],
            }
            self.last_mode = "LOITER"
            self.mode_trace = self.timing["mode_trace"]

        def prepare(self, scenario, seed):
            raise TimeoutError("Mode did not switch to ACRO; last mode=LOITER; last_ack=result=1")

        def shutdown(self):
            return None

    config = load_config("configs/mantis_pilot.yaml")
    scenario = config.scenario_by_id("px4_acro_roll")
    monkeypatch.setattr("cadet.query.make_adapter", lambda platform, config: FailingAdapter())

    with pytest.raises(TimeoutError):
        run_query(
            np.zeros(40),
            scenario,
            0,
            "mantis_pilot",
            tmp_path,
            config,
            use_cache=False,
            cache_tag="mode_fail",
        )

    metadata_path = next(Path(tmp_path).glob("queries/*/metadata.json"))
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert metadata["harness_failure"] is True
    assert metadata["failure_stage"] == "prepare"
    assert metadata["adapter_target_test_mode"] == "ACRO"
    assert metadata["adapter_mode_trace"][0]["requested_mode"] == "ACRO"
    assert not (metadata_path.parent / "robustness.json").exists()
