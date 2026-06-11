from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from cadet.properties import compute_residual_rate_metrics, summarize_residual_rate_repeats


SAFE = "safe"
VIOLATION_LIKE = "violation_like"
NOISE = "noise_band"

NONLINEAR_PARSED_COLUMNS = (
    "actuator_output",
    "motor_output",
    "actuator_saturation",
    "motor_saturation",
    "mixer_clipping",
    "rate_limit",
    "angle_limit",
    "integrator",
    "thrust_headroom",
)


def residual_rate_repeat_summary(
    parsed_logs: list[pd.DataFrame],
    property_name: str,
    config,
    *,
    sigma_multiplier: float = 2.0,
) -> dict[str, Any]:
    metrics = [compute_residual_rate_metrics(log, property_name, config) for log in parsed_logs]
    summary = summarize_residual_rate_repeats(metrics, sigma_multiplier=sigma_multiplier)
    summary["contract_class"] = classify_residual_summary(summary)
    if summary:
        threshold = float(summary.get("threshold_mean", summary.get("threshold", math.nan)))
        terminal = float(summary.get("terminal_peak_abs_rate_mean", math.nan))
        start = float(summary.get("tail_start_peak_abs_rate_mean", math.nan))
        summary["terminal_peak_over_threshold"] = terminal / threshold if threshold > 0 else math.nan
        summary["terminal_over_start_peak"] = terminal / start if start > 1e-9 else math.nan
    return summary


def classify_residual_summary(summary: dict[str, Any]) -> str:
    tier1 = str(summary.get("tier1_robustness_class", ""))
    tier2 = str(summary.get("tier2_robustness_class", ""))
    if tier2 == "robust_violation":
        return VIOLATION_LIKE
    if tier1 == "noise_band":
        return NOISE
    return SAFE


def is_safe_contract(summary: dict[str, Any]) -> bool:
    return classify_residual_summary(summary) == SAFE


def is_violation_like_contract(summary: dict[str, Any]) -> bool:
    return classify_residual_summary(summary) == VIOLATION_LIKE


def nonlinear_diagnostics(parsed_log: pd.DataFrame, raw_log_path: Path | None = None) -> dict[str, Any]:
    raw_path = Path(raw_log_path) if raw_log_path is not None else None
    if raw_path is not None and raw_path.exists() and raw_path.suffix.lower() == ".ulg":
        try:
            from cadet.mantis.rawlog_px4 import diagnose_px4_ulog

            return diagnose_px4_ulog(raw_path, parsed_log=parsed_log, active_axis=_active_axis_from_log(parsed_log))
        except Exception as exc:
            return {
                **_parsed_nonlinear_diagnostics(parsed_log, raw_path),
                "raw_log_parser_status": f"ulog_diagnostic_error:{exc}",
            }
    return _parsed_nonlinear_diagnostics(parsed_log, raw_path)


def _parsed_nonlinear_diagnostics(parsed_log: pd.DataFrame, raw_path: Path | None) -> dict[str, Any]:
    try:
        from cadet.mantis.nonlinear import parsed_log_nonlinear_diagnostics

        diag = parsed_log_nonlinear_diagnostics(parsed_log, active_axis=_active_axis_from_log(parsed_log))
    except Exception:
        diag = {}
    columns = [str(col) for col in parsed_log.columns]
    observed_columns = [
        col
        for col in columns
        if any(token in col.lower() for token in NONLINEAR_PARSED_COLUMNS)
    ]
    near_limit_columns = [col for col in observed_columns if _column_near_limit(parsed_log[col])]
    raw_present = bool(raw_path is not None and raw_path.exists())
    diag.update({
        "nonlinear_observability": bool(diag.get("nonlinear_observability", False) or observed_columns),
        "nonlinear_activated": bool(diag.get("nonlinear_activated", False) or near_limit_columns),
        "observed_nonlinear_columns": ",".join(observed_columns),
        "activated_nonlinear_columns": ",".join(near_limit_columns),
        "raw_log_present": raw_present,
        "raw_log_path": str(raw_path) if raw_path is not None else "",
        "raw_log_parser_status": "raw_log_not_px4_ulog" if raw_present and raw_path and raw_path.suffix.lower() != ".ulg" else "",
    })
    return diag


def _active_axis_from_log(parsed_log: pd.DataFrame) -> str:
    roll_energy = _manual_energy(parsed_log, "roll")
    pitch_energy = _manual_energy(parsed_log, "pitch")
    return "pitch" if pitch_energy > roll_energy else "roll"


def _manual_energy(parsed_log: pd.DataFrame, axis: str) -> float:
    column = f"manual_{axis}"
    if column not in parsed_log:
        return 0.0
    values = pd.to_numeric(parsed_log[column], errors="coerce").fillna(0.0).to_numpy(dtype=float)
    if "time_s" in parsed_log:
        time = pd.to_numeric(parsed_log["time_s"], errors="coerce").fillna(0.0).to_numpy(dtype=float)
    else:
        time = np.arange(len(values), dtype=float)
    return _integral_abs(time, values)


def cross_axis_energy(parsed_log: pd.DataFrame, active_axis: str) -> dict[str, float]:
    time = parsed_log["time_s"].to_numpy(dtype=float) if "time_s" in parsed_log else np.arange(len(parsed_log), dtype=float)
    active = _rate_column(active_axis)
    energies: dict[str, float] = {}
    for axis in ["roll", "pitch", "yaw"]:
        col = _rate_column(axis)
        if col in parsed_log:
            values = np.asarray(parsed_log[col], dtype=float)
            energies[f"{axis}_rate_energy"] = _integral_abs(time, values)
    active_energy = energies.get(f"{active_axis}_rate_energy", 0.0)
    off_axis = sum(value for key, value in energies.items() if key != f"{active_axis}_rate_energy")
    energies["cross_axis_energy_ratio"] = off_axis / max(active_energy, 1e-9) if active in parsed_log else math.nan
    return energies


def _rate_column(axis: str) -> str:
    return f"{axis}_rate_rps"


def _column_near_limit(series: pd.Series) -> bool:
    values = pd.to_numeric(series, errors="coerce").dropna().to_numpy(dtype=float)
    if values.size == 0:
        return False
    if np.nanmax(np.abs(values)) >= 0.98 and np.mean(np.abs(values) >= 0.98) >= 0.05:
        return True
    return bool(np.nanmax(values) > 0.5 and np.nanmin(values) < -0.5 and np.nanstd(values) < 1e-6)


def _integral_abs(time: np.ndarray, values: np.ndarray) -> float:
    if time.size < 2:
        return float(np.sum(np.abs(values)))
    return float(np.trapezoid(np.abs(values), time))
