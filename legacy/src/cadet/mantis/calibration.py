from __future__ import annotations

import math
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


ARM_TYPES = ("M0_at_P", "Msmall_at_P", "Mstrong_at_P0", "Mstrong_at_P")


def write_nonlinear_calibration(reports_dir: Path) -> Path:
    reports_dir = Path(reports_dir)
    diagnostics_csv = reports_dir / "mantis_nonlinear_diagnostics.csv"
    output_csv = reports_dir / "mantis_nonlinear_calibration.csv"
    try:
        diagnostics = pd.read_csv(diagnostics_csv).to_dict("records")
    except (FileNotFoundError, pd.errors.EmptyDataError):
        diagnostics = []
    pd.DataFrame(nonlinear_calibration_rows(diagnostics)).to_csv(output_csv, index=False)
    return output_csv


def nonlinear_calibration_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_arm = {arm_type: [] for arm_type in ARM_TYPES}
    for row in rows:
        arm_type = arm_type_for_query(str(row.get("query_id", "")))
        if arm_type in by_arm:
            by_arm[arm_type].append(row)

    result: list[dict[str, Any]] = []
    for arm_type in ARM_TYPES:
        arm_rows = by_arm[arm_type]
        n = len(arm_rows)
        observable = sum(1 for row in arm_rows if _as_bool(row.get("nonlinear_observability")))
        activated = sum(1 for row in arm_rows if _as_bool(row.get("nonlinear_activated")))
        explicit = sum(1 for row in arm_rows if _as_bool(row.get("explicit_saturation_flag_active")))
        sat_ratios = [_as_float(row.get("actuator_sat_ratio")) for row in arm_rows]
        sat_consecutive = [_as_float(row.get("actuator_sat_consecutive_s")) for row in arm_rows]
        reasons = Counter()
        for row in arm_rows:
            for reason in _split_reasons(row.get("nonlinear_activation_reasons")):
                reasons[reason] += 1
        result.append(
            {
                "arm_type": arm_type,
                "n": n,
                "nonlinear_observable_count": observable,
                "nonlinear_activated_count": activated,
                "nonlinear_activated_rate": float(activated / n) if n else 0.0,
                "median_actuator_sat_ratio": _median(sat_ratios),
                "max_actuator_sat_ratio": _max(sat_ratios),
                "median_actuator_sat_consecutive_s": _median(sat_consecutive),
                "max_actuator_sat_consecutive_s": _max(sat_consecutive),
                "explicit_saturation_flag_active_count": explicit,
                "top_nonlinear_activation_reasons": _format_top_reasons(reasons),
            }
        )
    return result


def arm_type_for_query(query_id: str) -> str:
    if "Mstrong_P0" in query_id or "stage_a_default_strong" in query_id:
        return "Mstrong_at_P0"
    if "Mstrong_P" in query_id or "stage_c_" in query_id:
        return "Mstrong_at_P"
    if "Msmall_P" in query_id or "stage_b_small" in query_id:
        return "Msmall_at_P"
    if "M0_P" in query_id or "stage_b_m0" in query_id:
        return "M0_at_P"
    return "unknown"


def _split_reasons(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, float) and math.isnan(value):
        return []
    if isinstance(value, list):
        return [str(item) for item in value if str(item)]
    return [item for item in str(value).split(";") if item]


def _format_top_reasons(reasons: Counter[str], limit: int = 5) -> str:
    return ";".join(f"{reason}:{count}" for reason, count in reasons.most_common(limit))


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return bool(value) and not pd.isna(value)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y"}
    return bool(value)


def _as_float(value: Any) -> float:
    try:
        if value in (None, ""):
            return math.nan
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _finite(values: list[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def _median(values: list[float]) -> float:
    finite = _finite(values)
    return float(np.median(finite)) if finite else math.nan


def _max(values: list[float]) -> float:
    finite = _finite(values)
    return float(np.max(finite)) if finite else math.nan
