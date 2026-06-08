from dataclasses import replace

from cadet.config import load_config
from cadet.query import _scenario_runtime_cache_tag


def test_runtime_cache_tag_includes_transition_switch_time():
    config = load_config("configs/rq1_minimal.yaml")
    base = config.scenario_by_id("px4_transition")

    assert _scenario_runtime_cache_tag(replace(base, t_switch_s=4.0)) != _scenario_runtime_cache_tag(
        replace(base, t_switch_s=5.0)
    )
    assert _scenario_runtime_cache_tag(replace(base, t_switch_s=4.0)).startswith("ts4p000")


def test_runtime_cache_tag_includes_param_overrides():
    config = load_config("configs/rq1_minimal.yaml")
    base = config.scenario_by_id("px4_position")

    default_tag = _scenario_runtime_cache_tag(base)
    acc_tag = _scenario_runtime_cache_tag(replace(base, param_overrides={"MPC_ACC_HOR": 3.0}))
    jerk_tag = _scenario_runtime_cache_tag(replace(base, param_overrides={"MPC_JERK_MAX": 8.0}))

    assert default_tag is None
    assert acc_tag is not None
    assert jerk_tag is not None
    assert acc_tag != jerk_tag
