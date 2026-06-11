from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


PX4_PARAMS = {
    "roll": {
        "rate_p": "MC_ROLLRATE_P",
        "rate_i": "MC_ROLLRATE_I",
        "rate_d": "MC_ROLLRATE_D",
        "rate_k": "MC_ROLLRATE_K",
        "att_p": "MC_ROLL_P",
    },
    "pitch": {
        "rate_p": "MC_PITCHRATE_P",
        "rate_i": "MC_PITCHRATE_I",
        "rate_d": "MC_PITCHRATE_D",
        "rate_k": "MC_PITCHRATE_K",
        "att_p": "MC_PITCH_P",
    },
}

ARDUPILOT_PARAMS = {
    "roll": {
        "rate_p": "ATC_RAT_RLL_P",
        "rate_i": "ATC_RAT_RLL_I",
        "rate_d": "ATC_RAT_RLL_D",
        "att_p": "ATC_ANG_RLL_P",
    },
    "pitch": {
        "rate_p": "ATC_RAT_PIT_P",
        "rate_i": "ATC_RAT_PIT_I",
        "rate_d": "ATC_RAT_PIT_D",
        "att_p": "ATC_ANG_PIT_P",
    },
}


@dataclass(frozen=True)
class ParamCandidateSpec:
    label: str
    multipliers: dict[str, float]


@dataclass(frozen=True)
class ParamCandidate:
    label: str
    overrides: dict[str, float]
    notes: list[str]

    def to_record(self) -> dict[str, Any]:
        return {"label": self.label, "overrides": dict(self.overrides), "notes": "; ".join(self.notes)}


def target_param_names(platform: str, axis: str) -> list[str]:
    return list(_roles(platform, axis).values())


def candidate_specs(platform: str, axis: str, *, include_i: bool = True) -> list[ParamCandidateSpec]:
    roles = _roles(platform, axis)
    specs = [
        ParamCandidateSpec("rate_p_x1p5", {roles["rate_p"]: 1.5}),
        ParamCandidateSpec("rate_p_x2p0", {roles["rate_p"]: 2.0}),
        ParamCandidateSpec("rate_p_x3p0", {roles["rate_p"]: 3.0}),
        ParamCandidateSpec("rate_d_x0p5", {roles["rate_d"]: 0.5}),
        ParamCandidateSpec("rate_d_x1p5", {roles["rate_d"]: 1.5}),
        ParamCandidateSpec("rate_p_x2p0_rate_d_x0p5", {roles["rate_p"]: 2.0, roles["rate_d"]: 0.5}),
        ParamCandidateSpec("rate_p_x2p0_att_p_x1p5", {roles["rate_p"]: 2.0, roles["att_p"]: 1.5}),
    ]
    if include_i and "rate_i" in roles:
        specs.append(ParamCandidateSpec("rate_i_x1p5", {roles["rate_i"]: 1.5}))
    return specs


def adaptive_candidate_specs(platform: str, axis: str) -> list[ParamCandidateSpec]:
    roles = _roles(platform, axis)
    role_specs = [
        ("rate_p_x1p55", {"rate_p": 1.55}),
        ("rate_p_x1p60", {"rate_p": 1.60}),
        ("rate_p_x1p70", {"rate_p": 1.70}),
        ("rate_p_x1p80", {"rate_p": 1.80}),
        ("rate_p_x1p90", {"rate_p": 1.90}),
        ("rate_d_x0p75", {"rate_d": 0.75}),
        ("rate_d_x0p85", {"rate_d": 0.85}),
        ("rate_d_x1p15", {"rate_d": 1.15}),
        ("rate_d_x1p25", {"rate_d": 1.25}),
        ("att_p_x1p10", {"att_p": 1.10}),
        ("att_p_x1p20", {"att_p": 1.20}),
        ("att_p_x1p30", {"att_p": 1.30}),
        ("rate_p_x1p55_rate_d_x0p85", {"rate_p": 1.55, "rate_d": 0.85}),
        ("rate_p_x1p60_rate_d_x0p85", {"rate_p": 1.60, "rate_d": 0.85}),
        ("rate_p_x1p70_rate_d_x0p85", {"rate_p": 1.70, "rate_d": 0.85}),
        ("rate_p_x1p60_rate_d_x1p15", {"rate_p": 1.60, "rate_d": 1.15}),
        ("rate_p_x1p70_att_p_x1p10", {"rate_p": 1.70, "att_p": 1.10}),
        ("rate_p_x1p70_att_p_x1p20", {"rate_p": 1.70, "att_p": 1.20}),
        ("rate_p_x1p50_rate_i_x1p20", {"rate_p": 1.50, "rate_i": 1.20}),
        ("rate_p_x1p60_rate_i_x1p20", {"rate_p": 1.60, "rate_i": 1.20}),
    ]
    specs: list[ParamCandidateSpec] = []
    for label, multipliers in role_specs:
        if not all(role in roles for role in multipliers):
            continue
        specs.append(ParamCandidateSpec(label, {roles[role]: value for role, value in multipliers.items()}))
    return specs


def symbolic_candidate_records(
    platform: str,
    axis: str,
    max_candidates: int,
    *,
    adaptive_boundary: bool = False,
) -> list[dict[str, Any]]:
    specs = adaptive_candidate_specs(platform, axis) if adaptive_boundary else candidate_specs(platform, axis)
    return [
        {"label": spec.label, "symbolic_multipliers": dict(spec.multipliers)}
        for spec in specs[: int(max_candidates)]
    ]


def build_param_candidates(
    platform: str,
    axis: str,
    defaults: dict[str, float],
    *,
    metadata: dict[str, dict[str, Any]] | None = None,
    max_candidates: int = 8,
    adaptive_boundary: bool = False,
) -> tuple[list[ParamCandidate], list[dict[str, Any]]]:
    metadata = metadata or {}
    candidates: list[ParamCandidate] = []
    skipped: list[dict[str, Any]] = []
    specs = adaptive_candidate_specs(platform, axis) if adaptive_boundary else candidate_specs(platform, axis)
    for spec in specs[: int(max_candidates)]:
        overrides: dict[str, float] = {}
        notes: list[str] = []
        skip_reason = ""
        for name, multiplier in spec.multipliers.items():
            if name not in defaults:
                skip_reason = f"default_unavailable:{name}"
                break
            meta = metadata.get(name, {})
            if bool(meta.get("rebootRequired", False)):
                skip_reason = f"reboot_required:{name}"
                break
            value = float(defaults[name]) * float(multiplier)
            clipped = _clip_to_metadata(value, meta)
            if clipped != value:
                notes.append(f"{name} clipped {value:g}->{clipped:g}")
            overrides[name] = clipped
        if skip_reason:
            skipped.append({"label": spec.label, "reason": skip_reason, "multipliers": dict(spec.multipliers)})
        else:
            candidates.append(ParamCandidate(spec.label, overrides, notes))
    return candidates, skipped


def default_readback_records(defaults: dict[str, float], types: dict[str, int], metadata: dict[str, dict[str, Any]]) -> list[dict]:
    rows = []
    for name, value in sorted(defaults.items()):
        meta = metadata.get(name, {})
        rows.append(
            {
                "name": name,
                "default_value": float(value),
                "readback_type": int(types.get(name, -1)),
                "min": meta.get("min", meta.get("minimum", "")),
                "max": meta.get("max", meta.get("maximum", "")),
                "rebootRequired": bool(meta.get("rebootRequired", False)),
            }
        )
    return rows


def _roles(platform: str, axis: str) -> dict[str, str]:
    if platform == "px4":
        return PX4_PARAMS[axis]
    if platform == "ardupilot":
        return ARDUPILOT_PARAMS[axis]
    raise ValueError(f"Unsupported MANTIS platform: {platform}")


def _clip_to_metadata(value: float, metadata: dict[str, Any]) -> float:
    lo = _maybe_float(metadata.get("min", metadata.get("minimum")))
    hi = _maybe_float(metadata.get("max", metadata.get("maximum")))
    if lo is not None:
        value = max(value, lo)
    if hi is not None:
        value = min(value, hi)
    return float(value)


def _maybe_float(value: Any) -> float | None:
    try:
        if value in (None, ""):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None
