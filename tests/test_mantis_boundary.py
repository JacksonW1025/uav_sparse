import pytest

from cadet.config import load_config
from cadet.mantis.params import adaptive_candidate_specs, build_param_candidates
from cadet.runners.mantis_pilot import _boundary_report_row, _boundary_score, _scenario_with_param_overrides


def _summary(contract_class, terminal_ratio, nondecay_ratio=1.0):
    return {
        "contract_class": contract_class,
        "terminal_peak_over_threshold": terminal_ratio,
        "terminal_over_start_peak": nondecay_ratio,
    }


def test_adaptive_boundary_candidate_generation_includes_refined_roll_grid():
    labels = [spec.label for spec in adaptive_candidate_specs("px4", "roll")]

    assert "rate_p_x1p55" in labels
    assert "rate_p_x1p60_rate_d_x0p85" in labels
    assert "rate_p_x1p70_att_p_x1p20" in labels
    assert "rate_p_x1p60_rate_i_x1p20" in labels


def test_pitch_adaptive_candidates_use_pitch_parameters_not_roll_parameters():
    defaults = {
        "MC_PITCHRATE_P": 0.3,
        "MC_PITCHRATE_I": 0.3,
        "MC_PITCHRATE_D": 0.001,
        "MC_PITCHRATE_K": 1.0,
        "MC_PITCH_P": 8.0,
    }

    candidates, skipped = build_param_candidates(
        "px4",
        "pitch",
        defaults,
        max_candidates=4,
        adaptive_boundary=True,
    )

    assert not skipped
    assert candidates
    for candidate in candidates:
        assert all("PITCH" in name for name in candidate.overrides)
        assert all("ROLL" not in name for name in candidate.overrides)


def test_metadata_clip_and_reboot_skip_apply_to_adaptive_candidates():
    defaults = {
        "MC_ROLLRATE_P": 0.3,
        "MC_ROLLRATE_I": 0.3,
        "MC_ROLLRATE_D": 0.001,
        "MC_ROLLRATE_K": 1.0,
        "MC_ROLL_P": 8.0,
    }
    metadata = {
        "MC_ROLLRATE_P": {"max": 0.45},
        "MC_ROLLRATE_D": {"rebootRequired": True},
    }

    candidates, skipped = build_param_candidates(
        "px4",
        "roll",
        defaults,
        metadata=metadata,
        max_candidates=8,
        adaptive_boundary=True,
    )

    assert candidates[0].overrides["MC_ROLLRATE_P"] == pytest.approx(0.45)
    assert any(row["reason"] == "reboot_required:MC_ROLLRATE_D" for row in skipped)


def test_boundary_ranking_does_not_override_m0_and_msmall_safety():
    unsafe_m0 = _summary("violation_like", 5.0)
    unsafe_small = _summary("safe", 0.2)
    safe_m0 = _summary("safe", 0.7)
    safe_small = _summary("safe", 0.8)

    assert _boundary_score(unsafe_m0, unsafe_small) > _boundary_score(safe_m0, safe_small)

    unsafe_row = _boundary_report_row("unsafe", {}, unsafe_m0, unsafe_small, retained_for_stage_c=False)
    safe_row = _boundary_report_row("safe", {}, safe_m0, safe_small, retained_for_stage_c=True)

    assert unsafe_row["retained_for_stage_c"] is False
    assert unsafe_row["rejection_reason"] == "m0_not_safe"
    assert safe_row["retained_for_stage_c"] is True
    assert safe_row["rejection_reason"] == ""


def test_candidate_scenario_resets_full_target_baseline_before_candidate_diff():
    scenario = load_config("configs/mantis_pilot.yaml").scenario_by_id("px4_acro_roll")
    baseline = {
        "MC_ROLLRATE_P": 0.3,
        "MC_ROLLRATE_I": 0.3,
        "MC_ROLLRATE_D": 0.0015,
        "MC_ROLLRATE_K": 1.0,
        "MC_ROLL_P": 9.75,
    }

    scenario_p = _scenario_with_param_overrides(scenario, baseline, {"MC_ROLLRATE_P": 0.45})

    assert scenario_p.param_overrides["MC_ROLLRATE_P"] == pytest.approx(0.45)
    assert scenario_p.param_overrides["MC_ROLLRATE_D"] == pytest.approx(0.0015)
    assert scenario_p.param_overrides["MC_ROLL_P"] == pytest.approx(9.75)
