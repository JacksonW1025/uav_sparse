import pytest

from cadet.mantis.calibration import arm_type_for_query, nonlinear_calibration_rows


def test_nonlinear_calibration_aggregates_by_arm_type():
    rows = [
        {
            "query_id": "abc_mantis_stage_b_m0_rate_p_hover_repeat0",
            "nonlinear_observability": True,
            "nonlinear_activated": False,
            "actuator_sat_ratio": 0.1,
            "actuator_sat_consecutive_s": 0.2,
            "explicit_saturation_flag_active": False,
            "nonlinear_activation_reasons": "",
        },
        {
            "query_id": "abc_mantis_stage_b_small_rate_p_small_repeat0",
            "nonlinear_observability": True,
            "nonlinear_activated": True,
            "actuator_sat_ratio": 0.3,
            "actuator_sat_consecutive_s": 0.4,
            "explicit_saturation_flag_active": True,
            "nonlinear_activation_reasons": "explicit_saturation_or_clipping_flag_active;sustained_actuator_near_limit",
        },
        {
            "query_id": "abc_mantis_stage_a_default_strong_strong_repeat0",
            "nonlinear_observability": True,
            "nonlinear_activated": True,
            "actuator_sat_ratio": 0.5,
            "actuator_sat_consecutive_s": 0.6,
            "explicit_saturation_flag_active": True,
            "nonlinear_activation_reasons": "explicit_saturation_or_clipping_flag_active",
        },
        {
            "query_id": "abc_mantis_stage_c_rate_p_strong_repeat0",
            "nonlinear_observability": False,
            "nonlinear_activated": False,
            "actuator_sat_ratio": 0.7,
            "actuator_sat_consecutive_s": 0.8,
            "explicit_saturation_flag_active": False,
            "nonlinear_activation_reasons": "",
        },
    ]

    by_arm = {row["arm_type"]: row for row in nonlinear_calibration_rows(rows)}

    assert by_arm["M0_at_P"]["n"] == 1
    assert by_arm["M0_at_P"]["nonlinear_activated_count"] == 0
    assert by_arm["Msmall_at_P"]["nonlinear_activated_rate"] == pytest.approx(1.0)
    assert by_arm["Msmall_at_P"]["explicit_saturation_flag_active_count"] == 1
    assert by_arm["Mstrong_at_P0"]["max_actuator_sat_ratio"] == pytest.approx(0.5)
    assert by_arm["Mstrong_at_P"]["nonlinear_observable_count"] == 0
    assert "explicit_saturation_or_clipping_flag_active:1" in by_arm["Msmall_at_P"][
        "top_nonlinear_activation_reasons"
    ]


def test_confirmation_query_ids_map_to_calibration_arm_types():
    assert arm_type_for_query("q_mantis_stage_e_rate_p_Mstrong_P0_123_repeat0") == "Mstrong_at_P0"
    assert arm_type_for_query("q_mantis_stage_e_rate_p_Mstrong_P_123_repeat0") == "Mstrong_at_P"
    assert arm_type_for_query("q_mantis_stage_e_rate_p_Msmall_P_123_repeat0") == "Msmall_at_P"
    assert arm_type_for_query("q_mantis_stage_e_rate_p_M0_P_123_repeat0") == "M0_at_P"
