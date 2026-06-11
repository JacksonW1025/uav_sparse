from __future__ import annotations

from dataclasses import dataclass


BLOCKED_ENV = "BLOCKED_ENV"
READY_NO_SITL = "READY_NO_SITL"
NO_SLIVER_FOUND = "NO_SLIVER_FOUND"
SLIVER_CANDIDATE = "SLIVER_CANDIDATE"
CONDITIONAL_CANDIDATE_NOT_ACCEPTED = "CONDITIONAL_CANDIDATE_NOT_ACCEPTED"
ACCEPTED_MANTIS_BUG = "ACCEPTED_MANTIS_BUG"
INVALID_PURE_PARAM = "INVALID_PURE_PARAM"
INVALID_PURE_INPUT = "INVALID_PURE_INPUT"
NOISE_BAND = "NOISE_BAND"


@dataclass(frozen=True)
class CandidateEvidence:
    default_strong_safe: bool
    hover_safe: bool
    small_safe: bool
    strong_violation_like: bool
    nonlinear_observable: bool
    nonlinear_activated: bool
    confirmed: bool = False
    repeated_noise_band: bool = False


def classify_candidate(evidence: CandidateEvidence) -> str:
    if evidence.repeated_noise_band:
        return NOISE_BAND
    if not evidence.hover_safe or not evidence.small_safe:
        return INVALID_PURE_PARAM
    if not evidence.default_strong_safe:
        return INVALID_PURE_INPUT
    if not evidence.strong_violation_like:
        return NO_SLIVER_FOUND
    if not evidence.confirmed:
        return SLIVER_CANDIDATE
    if not evidence.nonlinear_observable:
        return CONDITIONAL_CANDIDATE_NOT_ACCEPTED
    if not evidence.nonlinear_activated:
        return CONDITIONAL_CANDIDATE_NOT_ACCEPTED
    return ACCEPTED_MANTIS_BUG


def top_level_status(rows: list[dict]) -> str:
    if not rows:
        return NO_SLIVER_FOUND
    order = [
        ACCEPTED_MANTIS_BUG,
        CONDITIONAL_CANDIDATE_NOT_ACCEPTED,
        SLIVER_CANDIDATE,
        NOISE_BAND,
        NO_SLIVER_FOUND,
        INVALID_PURE_PARAM,
        INVALID_PURE_INPUT,
    ]
    statuses = {str(row.get("candidate_status", "")) for row in rows}
    for status in order:
        if status in statuses:
            return status
    return NO_SLIVER_FOUND
