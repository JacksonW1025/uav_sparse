from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


C_TRACK_NTE_THRESHOLD = 0.65
C_TRACK_PEAK_ERR_THRESHOLD_RADPS = 1.0
C_TRACK_HIGH_ERR_DURATION_THRESHOLD_S = 0.25
C_TRACK_BASELINE_RATIO = 2.5
C_TRACK_OVERLAP_THRESHOLD_S = 0.20
C_TRACK_RMS_SP_FLOOR_RADPS = 0.25

AXIS_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}
EXPLICIT_SATURATION_TOKENS = ("saturation", "saturated", "clipping", "clipped")
LIMIT_TOKENS = ("limited", "limit")
ACTUATOR_TOPIC_NAMES = {"actuator_motors", "actuator_outputs", "actuator_controls_0", "actuator_servos"}
STATUS_TOPIC_HINTS = {"rate_ctrl_status", "control_allocator_status"}


def evaluate_tracking_contract_from_raw(
    raw_log_path: Path | None,
    *,
    parsed_log: pd.DataFrame | None = None,
    axis: str,
    baseline_nte_median: float | None = None,
) -> dict[str, Any]:
    if raw_log_path is None:
        return _unavailable("raw_log_missing")
    raw_log_path = Path(raw_log_path)
    if not raw_log_path.exists() or raw_log_path.suffix.lower() != ".ulg":
        return _unavailable("raw_log_missing_or_not_ulog")
    try:
        from cadet.mantis.rawlog_px4 import load_ulog_topics
    except ImportError:
        return _unavailable("pyulog_unavailable")
    try:
        topics = load_ulog_topics(raw_log_path)
    except Exception as exc:
        return _unavailable(f"ulog_parse_error:{exc}")
    if parsed_log is not None:
        topics["parsed_mavlink"] = parsed_log
    result = evaluate_tracking_contract_from_topics(topics, axis=axis, baseline_nte_median=baseline_nte_median)
    result["raw_log_path"] = str(raw_log_path)
    return result


def evaluate_tracking_contract_from_topics(
    topics: Mapping[str, pd.DataFrame],
    *,
    axis: str,
    baseline_nte_median: float | None = None,
) -> dict[str, Any]:
    if axis not in AXIS_INDEX:
        raise ValueError(f"C_track axis must be roll or pitch/yaw, got {axis}")
    setpoint_topic = _topic(topics, "vehicle_rates_setpoint")
    rate_topic = _topic(topics, "vehicle_angular_velocity")
    if setpoint_topic is None or rate_topic is None:
        return _unavailable("vehicle_rates_setpoint_or_vehicle_angular_velocity_missing")

    sp_times, sp_values = _axis_series(setpoint_topic, axis, setpoint=True)
    rate_times, rate_values = _axis_series(rate_topic, axis, setpoint=False)
    if sp_times.size < 2 or rate_times.size < 2:
        return _unavailable("rate_setpoint_or_rate_samples_missing")

    window = _active_window(topics, axis, sp_times, sp_values)
    if window is None:
        return {
            **_base_available(axis, baseline_nte_median),
            "active_window_available": False,
            "C_track_status": "safe",
            "C_track_safe": True,
            "C_track_violation": False,
            "C_track_unavailable_reason": "",
            "active_window_start_s": math.nan,
            "active_window_end_s": math.nan,
            "rms_sp": 0.0,
            "rms_err": 0.0,
            "nte": 0.0,
            "peak_err": 0.0,
            "high_err_duration_s": 0.0,
            "saturation_error_overlap_s": 0.0,
            "explicit_limit_or_saturation_during_high_error": False,
            "baseline_ratio_ok": False,
        }

    mask = (sp_times >= window[0]) & (sp_times <= window[1]) & np.isfinite(sp_values)
    if int(np.sum(mask)) < 2:
        return {
            **_base_available(axis, baseline_nte_median),
            "active_window_available": False,
            "C_track_status": "safe",
            "C_track_safe": True,
            "C_track_violation": False,
            "C_track_unavailable_reason": "",
            "active_window_start_s": float(window[0]),
            "active_window_end_s": float(window[1]),
            "rms_sp": 0.0,
            "rms_err": 0.0,
            "nte": 0.0,
            "peak_err": 0.0,
            "high_err_duration_s": 0.0,
            "saturation_error_overlap_s": 0.0,
            "explicit_limit_or_saturation_during_high_error": False,
            "baseline_ratio_ok": False,
        }

    times = sp_times[mask]
    sp = sp_values[mask]
    actual = _interp(rate_times, rate_values, times)
    valid = np.isfinite(times) & np.isfinite(sp) & np.isfinite(actual)
    times = times[valid]
    sp = sp[valid]
    actual = actual[valid]
    if times.size < 2:
        return _unavailable("aligned_rate_samples_missing")

    err = sp - actual
    rms_sp = _rms(sp)
    rms_err = _rms(err)
    nte = rms_err / max(rms_sp, C_TRACK_RMS_SP_FLOOR_RADPS)
    peak_err = float(np.nanmax(np.abs(err))) if err.size else 0.0
    high_err = np.abs(err) >= C_TRACK_PEAK_ERR_THRESHOLD_RADPS
    high_err_duration_s = _duration_where(times, high_err)
    sat_mask, explicit_mask, observable, active = _nonlinear_masks_on_grid(topics, times)
    saturation_error_overlap_s = _duration_where(times, high_err & sat_mask)
    explicit_high = bool(np.any(high_err & explicit_mask))

    baseline = _finite_or_nan(baseline_nte_median)
    baseline_ratio_ok = math.isfinite(baseline) and nte >= C_TRACK_BASELINE_RATIO * baseline
    violation = bool(
        nte >= C_TRACK_NTE_THRESHOLD
        and peak_err >= C_TRACK_PEAK_ERR_THRESHOLD_RADPS
        and high_err_duration_s >= C_TRACK_HIGH_ERR_DURATION_THRESHOLD_S
        and baseline_ratio_ok
        and (saturation_error_overlap_s >= C_TRACK_OVERLAP_THRESHOLD_S or explicit_high)
    )
    return {
        **_base_available(axis, baseline_nte_median),
        "active_window_available": True,
        "C_track_status": "violation_like" if violation else "safe",
        "C_track_safe": not violation,
        "C_track_violation": violation,
        "C_track_unavailable_reason": "",
        "active_window_start_s": float(window[0]),
        "active_window_end_s": float(window[1]),
        "rms_sp": float(rms_sp),
        "rms_err": float(rms_err),
        "nte": float(nte),
        "peak_err": float(peak_err),
        "high_err_duration_s": float(high_err_duration_s),
        "saturation_error_overlap_s": float(saturation_error_overlap_s),
        "explicit_limit_or_saturation_during_high_error": explicit_high,
        "baseline_ratio_ok": bool(baseline_ratio_ok),
        "nonlinear_observable": bool(observable),
        "nonlinear_activated_in_active_window": bool(active),
    }


def summarize_tracking_repeats(rows: list[dict[str, Any]]) -> dict[str, Any]:
    available = [row for row in rows if bool(row.get("C_track_available", False))]
    ntes = [_finite_or_nan(row.get("nte")) for row in available]
    ntes = [value for value in ntes if math.isfinite(value)]
    violation_count = sum(1 for row in available if bool(row.get("C_track_violation", False)))
    return {
        "C_track_available": bool(available),
        "C_track_available_count": int(len(available)),
        "C_track_unavailable_count": int(len(rows) - len(available)),
        "C_track_status": "violation_like" if violation_count else ("safe" if available else "C_track_unavailable"),
        "C_track_safe": bool(not violation_count),
        "C_track_violation_count": int(violation_count),
        "C_track_repeat_count": int(len(rows)),
        "median_nte": float(np.nanmedian(ntes)) if ntes else math.nan,
        "max_nte": float(np.nanmax(ntes)) if ntes else math.nan,
        "max_peak_err": _max_metric(available, "peak_err"),
        "max_high_err_duration_s": _max_metric(available, "high_err_duration_s"),
        "max_saturation_error_overlap_s": _max_metric(available, "saturation_error_overlap_s"),
    }


def _unavailable(reason: str) -> dict[str, Any]:
    return {
        "C_track_available": False,
        "C_track_status": "C_track_unavailable",
        "C_track_safe": True,
        "C_track_violation": False,
        "C_track_unavailable_reason": reason,
        "active_window_available": False,
        "active_window_start_s": math.nan,
        "active_window_end_s": math.nan,
        "rms_sp": math.nan,
        "rms_err": math.nan,
        "nte": math.nan,
        "peak_err": math.nan,
        "high_err_duration_s": math.nan,
        "saturation_error_overlap_s": math.nan,
        "explicit_limit_or_saturation_during_high_error": False,
        "baseline_ratio_ok": False,
        "baseline_nte_median": math.nan,
        "nonlinear_observable": False,
        "nonlinear_activated_in_active_window": False,
    }


def _base_available(axis: str, baseline_nte_median: float | None) -> dict[str, Any]:
    return {
        "C_track_available": True,
        "C_track_axis": axis,
        "baseline_nte_median": _finite_or_nan(baseline_nte_median),
        "nte_threshold": C_TRACK_NTE_THRESHOLD,
        "peak_err_threshold_radps": C_TRACK_PEAK_ERR_THRESHOLD_RADPS,
        "high_err_duration_threshold_s": C_TRACK_HIGH_ERR_DURATION_THRESHOLD_S,
        "baseline_ratio_threshold": C_TRACK_BASELINE_RATIO,
        "overlap_threshold_s": C_TRACK_OVERLAP_THRESHOLD_S,
    }


def _topic(topics: Mapping[str, pd.DataFrame], base_name: str) -> pd.DataFrame | None:
    if base_name in topics:
        return topics[base_name]
    for name, df in topics.items():
        if str(name).split("#", 1)[0] == base_name:
            return df
    return None


def _axis_series(df: pd.DataFrame, axis: str, *, setpoint: bool) -> tuple[np.ndarray, np.ndarray]:
    times = _time_seconds(df)
    lower_to_col = {str(col).lower(): str(col) for col in df.columns}
    candidates = [axis]
    if not setpoint:
        candidates.extend([f"xyz[{AXIS_INDEX[axis]}]", {"roll": "p", "pitch": "q", "yaw": "r"}[axis]])
    for name in candidates:
        if name in lower_to_col:
            values = pd.to_numeric(df[lower_to_col[name]], errors="coerce").to_numpy(dtype=float)
            return _valid_time_series(times, values)
    return np.array([], dtype=float), np.array([], dtype=float)


def _active_window(
    topics: Mapping[str, pd.DataFrame],
    axis: str,
    sp_times: np.ndarray,
    sp_values: np.ndarray,
    *,
    threshold: float = 0.05,
) -> tuple[float, float] | None:
    manual = _topic(topics, "manual_control_setpoint")
    if manual is not None:
        times = _time_seconds(manual)
        lower_to_col = {str(col).lower(): str(col) for col in manual.columns}
        if axis in lower_to_col:
            values = pd.to_numeric(manual[lower_to_col[axis]], errors="coerce").to_numpy(dtype=float)
            times, values = _valid_time_series(times, values)
            active = np.abs(values) > threshold
            if bool(np.any(active)):
                return _window_from_mask(times, active)
    active = np.abs(sp_values) > threshold
    if bool(np.any(active)):
        return _window_from_mask(sp_times, active)
    return None


def _nonlinear_masks_on_grid(topics: Mapping[str, pd.DataFrame], grid_times: np.ndarray) -> tuple[np.ndarray, np.ndarray, bool, bool]:
    sat_mask = np.zeros(grid_times.size, dtype=bool)
    explicit_mask = np.zeros(grid_times.size, dtype=bool)
    observable = False
    for name, df in topics.items():
        base = str(name).split("#", 1)[0]
        lower_name = str(name).lower()
        times = _time_seconds(df)
        if times.size != len(df) or times.size < 1:
            continue
        if base in ACTUATOR_TOPIC_NAMES or ("actuator" in lower_name and "status" not in lower_name):
            row_mask = _actuator_near_limit_rows(df)
            if row_mask.size:
                observable = True
                sat_mask |= _project_mask(times, row_mask, grid_times)
        if base in STATUS_TOPIC_HINTS or "allocator" in lower_name or "rate_ctrl" in lower_name:
            row_mask = _explicit_status_rows(df)
            if row_mask.size:
                observable = True
                projected = _project_mask(times, row_mask, grid_times)
                explicit_mask |= projected
                sat_mask |= projected
    return sat_mask, explicit_mask, observable, bool(np.any(sat_mask))


def _actuator_near_limit_rows(df: pd.DataFrame) -> np.ndarray:
    columns = [str(col) for col in df.columns if _is_actuator_column(str(col).lower())]
    if not columns:
        return np.array([], dtype=bool)
    values = pd.DataFrame({col: pd.to_numeric(df[col], errors="coerce") for col in columns})
    row_mask = np.zeros(len(values), dtype=bool)
    for col in values.columns:
        arr = values[col].to_numpy(dtype=float)
        finite = arr[np.isfinite(arr)]
        if finite.size == 0:
            continue
        lo = float(np.nanmin(finite))
        hi = float(np.nanmax(finite))
        if hi <= 0.1 and lo >= -0.01:
            continue
        if lo < -0.05 and hi <= 1.1:
            row_mask |= (arr >= 0.98) | (arr <= -0.98)
        elif hi <= 1.1:
            row_mask |= (arr >= 0.98) | (arr <= 0.02)
        elif 800.0 <= lo <= 2200.0 or 800.0 <= hi <= 2200.0:
            row_mask |= (arr >= 1950.0) | (arr <= 1050.0)
    return row_mask


def _explicit_status_rows(df: pd.DataFrame) -> np.ndarray:
    row_mask = np.zeros(len(df), dtype=bool)
    found = False
    for col in df.columns:
        lower = str(col).lower()
        if lower in {"timestamp", "timestamp_sample", "time_us", "time_usec", "time_s"}:
            continue
        if not any(token in lower for token in EXPLICIT_SATURATION_TOKENS + LIMIT_TOKENS):
            continue
        values = pd.to_numeric(df[col], errors="coerce").fillna(0.0).to_numpy(dtype=float)
        row_mask |= np.abs(values) > 0.0
        found = True
    return row_mask if found else np.array([], dtype=bool)


def _project_mask(source_times: np.ndarray, source_mask: np.ndarray, grid_times: np.ndarray) -> np.ndarray:
    valid = np.isfinite(source_times)
    source_times = source_times[valid]
    source_mask = np.asarray(source_mask, dtype=bool)[valid]
    if source_times.size == 0 or grid_times.size == 0:
        return np.zeros(grid_times.size, dtype=bool)
    order = np.argsort(source_times)
    source_times = source_times[order]
    source_mask = source_mask[order]
    projected = np.interp(grid_times, source_times, source_mask.astype(float), left=0.0, right=0.0)
    return projected >= 0.5


def _time_seconds(df: pd.DataFrame) -> np.ndarray:
    for col in ("timestamp", "timestamp_sample", "time_us", "time_usec", "time_s"):
        if col not in df:
            continue
        values = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            continue
        if col == "time_s":
            return values
        scale = 1_000_000.0 if float(np.nanmax(np.abs(finite))) > 100_000.0 else 1.0
        return values / scale
    return np.array([], dtype=float)


def _valid_time_series(times: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if times.size != values.size:
        return np.array([], dtype=float), np.array([], dtype=float)
    valid = np.isfinite(times) & np.isfinite(values)
    times = times[valid]
    values = values[valid]
    if times.size == 0:
        return times, values
    order = np.argsort(times)
    return times[order], values[order]


def _window_from_mask(times: np.ndarray, mask: np.ndarray) -> tuple[float, float]:
    active_times = times[np.asarray(mask, dtype=bool)]
    return float(np.nanmin(active_times)), float(np.nanmax(active_times))


def _interp(source_times: np.ndarray, source_values: np.ndarray, target_times: np.ndarray) -> np.ndarray:
    valid = np.isfinite(source_times) & np.isfinite(source_values)
    source_times = source_times[valid]
    source_values = source_values[valid]
    if source_times.size == 0:
        return np.full(target_times.shape, math.nan, dtype=float)
    order = np.argsort(source_times)
    return np.interp(target_times, source_times[order], source_values[order], left=math.nan, right=math.nan)


def _duration_where(times: np.ndarray, mask: np.ndarray) -> float:
    mask = np.asarray(mask, dtype=bool)
    if times.size < 2 or mask.size != times.size:
        return 0.0
    dt = np.diff(times)
    valid_dt = np.where(np.isfinite(dt) & (dt > 0), dt, 0.0)
    return float(np.sum(valid_dt[mask[:-1]]))


def _rms(values: np.ndarray) -> float:
    valid = values[np.isfinite(values)]
    if valid.size == 0:
        return math.nan
    return float(np.sqrt(np.mean(np.square(valid))))


def _max_metric(rows: list[dict[str, Any]], key: str) -> float:
    values = [_finite_or_nan(row.get(key)) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    return float(np.nanmax(values)) if values else math.nan


def _finite_or_nan(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _is_actuator_column(lower: str) -> bool:
    return (
        lower.startswith("control[")
        or lower.startswith("output[")
        or lower.startswith("motor[")
        or lower.startswith("actuator[")
    )
