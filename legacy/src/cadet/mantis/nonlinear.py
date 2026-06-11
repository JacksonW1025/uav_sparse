from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

import numpy as np
import pandas as pd


ACTUATOR_TOPIC_NAMES = {
    "actuator_motors",
    "actuator_outputs",
    "actuator_controls_0",
    "actuator_servos",
}
RATE_TOPIC_NAMES = {"vehicle_angular_velocity", "vehicle_rates_setpoint"}
MANUAL_TOPIC_NAMES = {"manual_control_setpoint"}
STATUS_TOPIC_HINTS = {
    "rate_ctrl_status",
    "control_allocator_status",
    "actuator_armed",
    "vehicle_status",
}
EXPLICIT_SATURATION_TOKENS = ("saturation", "saturated", "clipping", "clipped")
LIMIT_TOKENS = ("limited", "limit")
INTEGRATOR_TOKENS = ("integrator", "integral")

ACTUATOR_SAT_RATIO_THRESHOLD = 0.05
ACTUATOR_SAT_CONSECUTIVE_S_THRESHOLD = 0.20
NORMALIZED_UPPER = 0.98
NORMALIZED_01_LOWER = 0.02
NORMALIZED_PM_LOWER = -0.98
PWM_UPPER = 1950.0
PWM_LOWER = 1050.0


def analyze_nonlinear_topics(
    topics: Mapping[str, pd.DataFrame],
    *,
    active_axis: str = "roll",
    neutral_tail_s: float = 8.0,
) -> dict[str, Any]:
    window = infer_active_recovery_window(topics, neutral_tail_s=neutral_tail_s)
    selected = selected_diagnostic_topics(topics)
    actuator_metrics = _actuator_metrics(topics, selected["actuator_topics"], window)
    flag_metrics = _flag_metrics(topics, selected["status_topics"], window)
    cross_axis = _cross_axis_metrics(topics, active_axis, window)

    actuator_available = bool(actuator_metrics["actuator_available"])
    explicit_available = bool(flag_metrics["explicit_saturation_flag_available"])
    integrator_available = bool(flag_metrics["integrator_available"])
    limit_available = bool(flag_metrics["limit_flag_available"])
    allocator_available = bool(flag_metrics["allocator_saturation_flag_available"])
    nonlinear_observability = bool(
        actuator_available or explicit_available or integrator_available or limit_available or allocator_available
    )

    reasons: list[str] = []
    if bool(flag_metrics["explicit_saturation_flag_active"]):
        reasons.append("explicit_saturation_or_clipping_flag_active")
    if bool(flag_metrics["limit_flag_active"]):
        reasons.append("explicit_limit_flag_active")
    if bool(flag_metrics["integrator_saturation_flag_active"]):
        reasons.append("integrator_saturation_flag_active")
    if bool(flag_metrics["allocator_saturation_flag_active"]):
        reasons.append("control_allocator_saturation_flag_active")
    if (
        float(actuator_metrics["actuator_sat_ratio"]) >= ACTUATOR_SAT_RATIO_THRESHOLD
        and float(actuator_metrics["actuator_sat_consecutive_s"]) >= ACTUATOR_SAT_CONSECUTIVE_S_THRESHOLD
    ):
        reasons.append("sustained_actuator_near_limit")

    return {
        **actuator_metrics,
        **flag_metrics,
        **cross_axis,
        "nonlinear_observability": nonlinear_observability,
        "nonlinear_activated": bool(reasons),
        "nonlinear_activation_reasons": reasons,
        "selected_actuator_topics": selected["actuator_topics"],
        "selected_status_topics": selected["status_topics"],
        "selected_rate_topics": selected["rate_topics"],
        "selected_manual_topics": selected["manual_topics"],
        "diagnostic_window_source": "manual_control_setpoint" if window is not None else "full_log_no_manual_window",
        "diagnostic_window_start_s": float(window[0]) if window is not None else math.nan,
        "diagnostic_window_end_s": float(window[1]) if window is not None else math.nan,
    }


def topic_inventory(topics: Mapping[str, pd.DataFrame], selected: Mapping[str, list[str]] | None = None) -> dict[str, Any]:
    selected_names: set[str] = set()
    if selected:
        for names in selected.values():
            selected_names.update(str(name) for name in names)
    inventory: dict[str, Any] = {"topics": [], "selected_topics": sorted(selected_names)}
    for name, df in sorted(topics.items()):
        times = _time_seconds(df)
        if times.size:
            time_span_s = float(np.nanmax(times) - np.nanmin(times))
        else:
            time_span_s = math.nan
        inventory["topics"].append(
            {
                "name": str(name),
                "fields": [str(col) for col in df.columns],
                "sample_count": int(len(df)),
                "time_span_s": time_span_s,
                "selected_for_diagnostics": str(name) in selected_names,
            }
        )
    return inventory


def selected_diagnostic_topics(topics: Mapping[str, pd.DataFrame]) -> dict[str, list[str]]:
    selected = {
        "actuator_topics": [],
        "status_topics": [],
        "rate_topics": [],
        "manual_topics": [],
    }
    for name, df in topics.items():
        base = _base_topic_name(name)
        fields = [str(col) for col in df.columns]
        lower_name = str(name).lower()
        lower_fields = [field.lower() for field in fields]
        if base in ACTUATOR_TOPIC_NAMES or (
            "actuator" in lower_name
            and "status" not in lower_name
            and _has_any(lower_fields, ("control[", "output[", "motor"))
        ):
            selected["actuator_topics"].append(str(name))
        if base in RATE_TOPIC_NAMES or base == "vehicle_attitude":
            selected["rate_topics"].append(str(name))
        if base in MANUAL_TOPIC_NAMES:
            selected["manual_topics"].append(str(name))
        if (
            base in STATUS_TOPIC_HINTS
            or _has_any(lower_fields, EXPLICIT_SATURATION_TOKENS + LIMIT_TOKENS + INTEGRATOR_TOKENS)
            or ("allocator" in lower_name and _has_any(lower_fields, EXPLICIT_SATURATION_TOKENS + LIMIT_TOKENS))
        ):
            selected["status_topics"].append(str(name))
    return {key: sorted(set(values)) for key, values in selected.items()}


def infer_active_recovery_window(
    topics: Mapping[str, pd.DataFrame],
    *,
    neutral_tail_s: float = 8.0,
    threshold: float = 0.05,
) -> tuple[float, float] | None:
    starts: list[float] = []
    ends: list[float] = []
    for name, df in topics.items():
        if _base_topic_name(name) not in MANUAL_TOPIC_NAMES:
            continue
        columns = _manual_axis_columns(df)
        if not columns:
            continue
        times = _time_seconds(df)
        if times.size != len(df):
            continue
        values = _numeric_frame(df, columns)
        if values.empty:
            continue
        active = values.abs().max(axis=1).to_numpy(dtype=float) > float(threshold)
        if not bool(np.any(active)):
            continue
        active_times = times[active]
        starts.append(float(np.nanmin(active_times)))
        ends.append(float(np.nanmax(active_times)) + float(neutral_tail_s))
    if not starts:
        return None
    return min(starts), max(ends)


def parsed_log_nonlinear_diagnostics(parsed_log: pd.DataFrame, *, active_axis: str = "roll") -> dict[str, Any]:
    topics = {"parsed_mavlink": parsed_log}
    selected = selected_diagnostic_topics(topics)
    diag = analyze_nonlinear_topics(topics, active_axis=active_axis)
    diag["raw_log_parser_status"] = ""
    diag["raw_log_present"] = False
    diag["raw_log_path"] = ""
    diag["topic_inventory_status"] = "parsed_log_only"
    diag["observed_nonlinear_columns"] = ",".join(
        col for col in parsed_log.columns if _field_has_nonlinear_hint(str(col).lower())
    )
    diag["activated_nonlinear_columns"] = ""
    diag["selected_topics_count"] = sum(len(values) for values in selected.values())
    return diag


def _actuator_metrics(
    topics: Mapping[str, pd.DataFrame],
    topic_names: list[str],
    window: tuple[float, float] | None,
) -> dict[str, Any]:
    row_sat_segments: list[tuple[np.ndarray, np.ndarray]] = []
    near_upper_count = 0
    near_lower_count = 0
    sample_count = 0
    pwm_inferred = False
    actuator_available = False
    selected_columns: list[str] = []
    saturated_columns: set[str] = set()

    for name in topic_names:
        df = _windowed(topics[name], window)
        columns = _actuator_columns(df)
        if not columns:
            continue
        values = _numeric_frame(df, columns)
        if values.empty:
            continue
        actuator_available = True
        selected_columns.extend(f"{name}.{col}" for col in values.columns)
        row_near = np.zeros(len(values), dtype=bool)
        for col in values.columns:
            col_values = values[col].to_numpy(dtype=float)
            finite = col_values[np.isfinite(col_values)]
            if finite.size == 0:
                continue
            kind = _actuator_value_kind(str(name), str(col), finite)
            if kind == "zero_or_unknown":
                continue
            if kind == "normalized_0_1":
                upper = col_values >= NORMALIZED_UPPER
                lower = col_values <= NORMALIZED_01_LOWER
            elif kind == "normalized_minus1_1":
                upper = col_values >= NORMALIZED_UPPER
                lower = col_values <= NORMALIZED_PM_LOWER
            else:
                upper = col_values >= PWM_UPPER
                lower = col_values <= PWM_LOWER
                pwm_inferred = True
            near_upper_count += int(np.nansum(upper))
            near_lower_count += int(np.nansum(lower))
            if bool(np.any(upper | lower)):
                saturated_columns.add(f"{name}.{col}")
            row_near |= np.asarray(upper | lower, dtype=bool)
        sample_count += len(values)
        times = _time_seconds(df)
        if times.size != len(values):
            times = np.arange(len(values), dtype=float)
        row_sat_segments.append((times, row_near))

    if row_sat_segments:
        total_rows = int(sum(len(mask) for _, mask in row_sat_segments))
        sat_rows = int(sum(np.sum(mask) for _, mask in row_sat_segments))
        sat_ratio = sat_rows / max(total_rows, 1)
        consecutive_s = max(_longest_true_run_s(times, mask) for times, mask in row_sat_segments)
    else:
        sat_ratio = 0.0
        consecutive_s = 0.0

    return {
        "actuator_available": actuator_available,
        "actuator_sat_ratio": float(sat_ratio),
        "actuator_sat_consecutive_s": float(consecutive_s),
        "actuator_near_upper_count": int(near_upper_count),
        "actuator_near_lower_count": int(near_lower_count),
        "actuator_sample_count": int(sample_count),
        "actuator_pwm_inferred": bool(pwm_inferred),
        "actuator_selected_columns": selected_columns,
        "actuator_saturated_columns": sorted(saturated_columns),
    }


def _flag_metrics(
    topics: Mapping[str, pd.DataFrame],
    topic_names: list[str],
    window: tuple[float, float] | None,
) -> dict[str, Any]:
    explicit_cols: list[str] = []
    limit_cols: list[str] = []
    integrator_cols: list[str] = []
    integrator_sat_cols: list[str] = []
    allocator_cols: list[str] = []
    explicit_active = False
    limit_active = False
    integrator_sat_active = False
    allocator_active = False
    integrator_peak_abs = math.nan

    for name in topic_names:
        df = _windowed(topics[name], window)
        base = _base_topic_name(name)
        for col in df.columns:
            lower = str(col).lower()
            if _is_time_column(lower):
                continue
            if not _control_relevant_flag(name, lower):
                continue
            values = pd.to_numeric(df[col], errors="coerce")
            if values.notna().sum() == 0:
                continue
            full_name = f"{name}.{col}"
            if _has_token(lower, EXPLICIT_SATURATION_TOKENS):
                explicit_cols.append(full_name)
                active = _flag_active(values)
                explicit_active = explicit_active or active
                if _has_token(lower, INTEGRATOR_TOKENS):
                    integrator_sat_cols.append(full_name)
                    integrator_sat_active = integrator_sat_active or active
            if _has_token(lower, LIMIT_TOKENS):
                limit_cols.append(full_name)
                active = _flag_active(values)
                limit_active = limit_active or active
                if _has_token(lower, INTEGRATOR_TOKENS):
                    integrator_sat_cols.append(full_name)
                    integrator_sat_active = integrator_sat_active or active
            if _has_token(lower, INTEGRATOR_TOKENS):
                integrator_cols.append(full_name)
                peak = float(np.nanmax(np.abs(values.to_numpy(dtype=float))))
                integrator_peak_abs = max(float(integrator_peak_abs), peak) if math.isfinite(integrator_peak_abs) else peak
            if base == "control_allocator_status" and (
                _has_token(lower, EXPLICIT_SATURATION_TOKENS) or _has_token(lower, LIMIT_TOKENS)
            ):
                allocator_cols.append(full_name)
                allocator_active = allocator_active or _flag_active(values)

    return {
        "explicit_saturation_flag_available": bool(explicit_cols),
        "explicit_saturation_flag_active": bool(explicit_active),
        "explicit_saturation_flag_columns": explicit_cols,
        "integrator_available": bool(integrator_cols),
        "integrator_peak_abs": float(integrator_peak_abs) if math.isfinite(integrator_peak_abs) else math.nan,
        "integrator_columns": integrator_cols,
        "integrator_saturation_flag_available": bool(integrator_sat_cols),
        "integrator_saturation_flag_active": bool(integrator_sat_active),
        "integrator_saturation_flag_columns": integrator_sat_cols,
        "limit_flag_available": bool(limit_cols),
        "limit_flag_active": bool(limit_active),
        "limit_flag_columns": limit_cols,
        "allocator_saturation_flag_available": bool(allocator_cols),
        "allocator_saturation_flag_active": bool(allocator_active),
        "allocator_saturation_flag_columns": allocator_cols,
    }


def _cross_axis_metrics(
    topics: Mapping[str, pd.DataFrame],
    active_axis: str,
    window: tuple[float, float] | None,
) -> dict[str, Any]:
    best: dict[str, np.ndarray] | None = None
    best_time: np.ndarray | None = None
    for name, df0 in topics.items():
        df = _windowed(df0, window)
        rates = _rate_columns(df)
        if not rates:
            continue
        time_values = _time_seconds(df)
        if time_values.size != len(df):
            time_values = np.arange(len(df), dtype=float)
        best = {axis: pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float) for axis, col in rates.items()}
        best_time = time_values
        if _base_topic_name(name) in RATE_TOPIC_NAMES:
            break
    if best is None or best_time is None:
        return {"cross_axis_energy_ratio": math.nan, "rate_energy_available": False}
    energies = {axis: _integral_abs(best_time, values) for axis, values in best.items()}
    active_energy = energies.get(active_axis, 0.0)
    off_axis = sum(value for axis, value in energies.items() if axis != active_axis)
    return {
        "cross_axis_energy_ratio": float(off_axis / max(active_energy, 1e-9)),
        "rate_energy_available": True,
        **{f"{axis}_rate_energy": float(value) for axis, value in energies.items()},
    }


def _windowed(df: pd.DataFrame, window: tuple[float, float] | None) -> pd.DataFrame:
    if window is None:
        return df
    times = _time_seconds(df)
    if times.size != len(df):
        return df
    mask = (times >= float(window[0])) & (times <= float(window[1]))
    if not bool(np.any(mask)):
        return df.iloc[0:0].copy()
    return df.loc[mask].reset_index(drop=True)


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


def _actuator_columns(df: pd.DataFrame) -> list[str]:
    columns = []
    noutputs = _noutputs(df)
    for col in df.columns:
        lower = str(col).lower()
        if _is_time_column(lower):
            continue
        if lower == "noutputs":
            continue
        if noutputs is not None and lower.startswith("output[") and _array_index(lower) >= noutputs:
            continue
        if _is_actuator_value_column(lower):
            if pd.to_numeric(df[col], errors="coerce").notna().any():
                columns.append(str(col))
    return columns


def _manual_axis_columns(df: pd.DataFrame) -> list[str]:
    candidates = ("roll", "pitch", "yaw", "x", "y", "r")
    return [str(col) for col in df.columns if str(col).lower() in candidates]


def _rate_columns(df: pd.DataFrame) -> dict[str, str]:
    direct = {
        "roll": ("roll_rate_rps", "rollspeed", "roll_rate", "p"),
        "pitch": ("pitch_rate_rps", "pitchspeed", "pitch_rate", "q"),
        "yaw": ("yaw_rate_rps", "yawspeed", "yaw_rate", "r"),
    }
    result: dict[str, str] = {}
    lower_to_col = {str(col).lower(): str(col) for col in df.columns}
    for axis, names in direct.items():
        for name in names:
            if name in lower_to_col:
                result[axis] = lower_to_col[name]
                break
    array_map = {"roll": "xyz[0]", "pitch": "xyz[1]", "yaw": "xyz[2]"}
    for axis, name in array_map.items():
        if axis not in result and name in lower_to_col:
            result[axis] = lower_to_col[name]
    return result


def _numeric_frame(df: pd.DataFrame, columns: list[str]) -> pd.DataFrame:
    data = {col: pd.to_numeric(df[col], errors="coerce") for col in columns if col in df}
    if not data:
        return pd.DataFrame()
    return pd.DataFrame(data).dropna(axis=1, how="all")


def _actuator_value_kind(topic_name: str, column_name: str, finite_values: np.ndarray) -> str:
    lo = float(np.nanmin(finite_values))
    hi = float(np.nanmax(finite_values))
    lower_topic = topic_name.lower()
    lower_col = column_name.lower()
    if hi <= 0.1 and lo >= -0.01:
        return "zero_or_unknown"
    if lo < -0.05 and lo >= -1.1 and hi <= 1.1:
        return "normalized_minus1_1"
    if lo >= -0.05 and hi <= 1.1:
        return "normalized_0_1" if ("motor" in lower_topic or "output" in lower_col or "actuator_motors" in lower_topic) else "normalized_minus1_1"
    if 800.0 <= lo <= 2200.0 or 800.0 <= hi <= 2200.0:
        return "pwm"
    return "zero_or_unknown"


def _noutputs(df: pd.DataFrame) -> int | None:
    if "noutputs" not in df:
        return None
    values = pd.to_numeric(df["noutputs"], errors="coerce").dropna()
    if values.empty:
        return None
    value = int(max(0, values.median()))
    return value if value > 0 else None


def _array_index(lower: str) -> int:
    if "[" not in lower or "]" not in lower:
        return 0
    try:
        return int(lower.split("[", 1)[1].split("]", 1)[0])
    except ValueError:
        return 0


def _is_actuator_value_column(lower: str) -> bool:
    return (
        lower.startswith("control[")
        or lower.startswith("output[")
        or lower.startswith("motor[")
        or lower.startswith("actuator[")
    )


def _control_relevant_flag(topic_name: str, field_lower: str) -> bool:
    topic_lower = str(topic_name).lower()
    base = _base_topic_name(topic_name)
    if base in {"control_allocator_status", "rate_ctrl_status"}:
        return True
    if base in {"vehicle_rates_setpoint", "vehicle_angular_velocity"} and _has_token(field_lower, INTEGRATOR_TOKENS):
        return True
    if any(token in topic_lower for token in ("actuator", "allocator", "rate_ctrl", "mixer", "controller")):
        return True
    return any(token in field_lower for token in ("actuator", "motor", "rate_ctrl", "mixer", "allocator"))


def _flag_active(values: pd.Series) -> bool:
    arr = pd.to_numeric(values, errors="coerce").dropna().to_numpy(dtype=float)
    if arr.size == 0:
        return False
    return bool(np.nanmax(np.abs(arr)) > 0.0)


def _longest_true_run_s(times: np.ndarray, mask: np.ndarray) -> float:
    if mask.size == 0 or not bool(np.any(mask)):
        return 0.0
    if times.size != mask.size or times.size < 2:
        return float(np.sum(mask))
    best = 0.0
    start_idx: int | None = None
    for idx, active in enumerate(mask):
        if active and start_idx is None:
            start_idx = idx
        if (not active or idx == len(mask) - 1) and start_idx is not None:
            end_idx = idx if active and idx == len(mask) - 1 else max(start_idx, idx - 1)
            if end_idx == start_idx and len(times) > 1:
                dt = float(np.nanmedian(np.diff(times)))
                run_s = dt if math.isfinite(dt) and dt > 0 else 0.0
            else:
                run_s = float(times[end_idx] - times[start_idx])
            best = max(best, run_s)
            start_idx = None
    return best


def _integral_abs(time: np.ndarray, values: np.ndarray) -> float:
    valid = np.isfinite(time) & np.isfinite(values)
    if not bool(np.any(valid)):
        return 0.0
    t = time[valid]
    v = values[valid]
    if t.size < 2:
        return float(np.sum(np.abs(v)))
    return float(np.trapezoid(np.abs(v), t))


def _field_has_nonlinear_hint(lower: str) -> bool:
    return _has_token(lower, EXPLICIT_SATURATION_TOKENS + LIMIT_TOKENS + INTEGRATOR_TOKENS + ("actuator", "motor"))


def _base_topic_name(name: str) -> str:
    return str(name).split("#", 1)[0]


def _is_time_column(lower: str) -> bool:
    return lower in {"timestamp", "timestamp_sample", "time_us", "time_usec", "time_s"}


def _has_any(fields: list[str], tokens: tuple[str, ...]) -> bool:
    return any(any(token in field for token in tokens) for field in fields)


def _has_token(value: str, tokens: tuple[str, ...]) -> bool:
    return any(token in value for token in tokens)
