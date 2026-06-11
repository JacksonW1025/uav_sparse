from cadet.mantis.classify import (
    ACCEPTED_MANTIS_BUG,
    CONDITIONAL_CANDIDATE_NOT_ACCEPTED,
    INVALID_PURE_INPUT,
    INVALID_PURE_PARAM,
    SLIVER_CANDIDATE,
    CandidateEvidence,
    classify_candidate,
)


def test_pure_parameter_failure_is_invalid():
    status = classify_candidate(
        CandidateEvidence(
            default_strong_safe=True,
            hover_safe=False,
            small_safe=True,
            strong_violation_like=True,
            nonlinear_observable=True,
            nonlinear_activated=True,
        )
    )

    assert status == INVALID_PURE_PARAM


def test_pure_input_failure_is_invalid():
    status = classify_candidate(
        CandidateEvidence(
            default_strong_safe=False,
            hover_safe=True,
            small_safe=True,
            strong_violation_like=True,
            nonlinear_observable=True,
            nonlinear_activated=True,
        )
    )

    assert status == INVALID_PURE_INPUT


def test_small_safe_strong_unsafe_without_observable_nonlinearity_is_conditional():
    status = classify_candidate(
        CandidateEvidence(
            default_strong_safe=True,
            hover_safe=True,
            small_safe=True,
            strong_violation_like=True,
            nonlinear_observable=False,
            nonlinear_activated=False,
            confirmed=True,
        )
    )

    assert status == CONDITIONAL_CANDIDATE_NOT_ACCEPTED


def test_unconfirmed_small_safe_strong_unsafe_is_sliver_candidate():
    status = classify_candidate(
        CandidateEvidence(
            default_strong_safe=True,
            hover_safe=True,
            small_safe=True,
            strong_violation_like=True,
            nonlinear_observable=False,
            nonlinear_activated=False,
            confirmed=False,
        )
    )

    assert status == SLIVER_CANDIDATE


def test_confirmed_without_nonlinear_activation_is_conditional():
    status = classify_candidate(
        CandidateEvidence(
            default_strong_safe=True,
            hover_safe=True,
            small_safe=True,
            strong_violation_like=True,
            nonlinear_observable=True,
            nonlinear_activated=False,
            confirmed=True,
        )
    )

    assert status == CONDITIONAL_CANDIDATE_NOT_ACCEPTED


def test_full_nonlinear_differential_is_accepted():
    status = classify_candidate(
        CandidateEvidence(
            default_strong_safe=True,
            hover_safe=True,
            small_safe=True,
            strong_violation_like=True,
            nonlinear_observable=True,
            nonlinear_activated=True,
            confirmed=True,
        )
    )

    assert status == ACCEPTED_MANTIS_BUG
