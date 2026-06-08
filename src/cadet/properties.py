from __future__ import annotations

import numpy as np


RESIDUAL_RATE_PROPERTIES = {
    "post_neutral_climb_rate": {
        "log_field": "vz_mps",
        "threshold_key": "rate_max_mps",
        "unit": "m/s",
    },
    "post_neutral_yaw_rate": {
        "log_field": "yaw_rate_rps",
        "threshold_key": "rate_max_radps",
        "unit": "rad/s",
    },
}

TAIL_START_WINDOW_S = (0.0, 2.0)
TERMINAL_WINDOW_S = (6.0, 8.0)
FULL_TAIL_WINDOW_S = (0.0, 8.0)
NONDECAY_RATIO = 0.80


def compute_robustness(parsed_log, property_name: str, config, window: tuple[float, float] | None = None) -> float:
    t_neutral = float(parsed_log["t_neutral_s"].iloc[0])
    tail = parsed_log[parsed_log["time_s"] >= t_neutral]
    if tail.empty:
        raise ValueError("No samples at or after t_neutral_s")
    eval_tail = tail if window is None else _window_tail(tail, window)
    first = tail.iloc[0]
    if property_name == "post_neutral_xy_drift":
        d_max = float(config.properties[property_name]["d_max_m"])
        drift = np.sqrt((eval_tail["x_m"] - first["x_m"]) ** 2 + (eval_tail["y_m"] - first["y_m"]) ** 2)
        return float(d_max - drift.max())
    if property_name == "post_neutral_alt_drift":
        h_max = float(config.properties[property_name]["h_max_m"])
        drift = np.abs(eval_tail["alt_m"] - float(first["alt_m"]))
        return float(h_max - drift.max())
    if property_name == "post_neutral_xy_velocity":
        v_max = float(config.properties[property_name]["v_max_mps"])
        speed = np.sqrt(eval_tail["vx_mps"] ** 2 + eval_tail["vy_mps"] ** 2)
        return float(v_max - speed.max())
    if is_residual_rate_property(property_name):
        if window is not None:
            return float(_compute_residual_rate_rho_window(parsed_log, property_name, config, t_neutral, window))
        return float(compute_residual_rate_metrics(parsed_log, property_name, config)["rho_tier1"])
    raise KeyError(f"Unknown property: {property_name}")


def compute_all_properties(parsed_log, property_names: list[str], config) -> dict[str, float]:
    return {name: compute_robustness(parsed_log, name, config) for name in property_names}


def is_residual_rate_property(property_name: str) -> bool:
    return property_name in RESIDUAL_RATE_PROPERTIES


def residual_rate_threshold(property_name: str, config) -> float:
    spec = RESIDUAL_RATE_PROPERTIES[property_name]
    return float(config.properties[property_name][spec["threshold_key"]])


def compute_residual_rate_metrics(parsed_log, property_name: str, config) -> dict[str, float | str]:
    """Compute the pre-registered per-repeat residual-rate Tier metrics."""
    if not is_residual_rate_property(property_name):
        raise KeyError(f"Not a residual-rate property: {property_name}")
    t_neutral = float(parsed_log["t_neutral_s"].iloc[0])
    times = parsed_log["time_s"].to_numpy(dtype=float)
    abs_rate = np.abs(_rate_series(parsed_log, property_name))
    threshold = residual_rate_threshold(property_name, config)

    start_lo, start_hi = _absolute_window(t_neutral, TAIL_START_WINDOW_S)
    terminal_lo, terminal_hi = _absolute_window(t_neutral, TERMINAL_WINDOW_S)
    tail_lo, tail_hi = _absolute_window(t_neutral, FULL_TAIL_WINDOW_S)
    start_peak = _window_peak(times, abs_rate, start_lo, start_hi)
    terminal_peak = _window_peak(times, abs_rate, terminal_lo, terminal_hi)
    slope = _window_slope(times, abs_rate, tail_lo, tail_hi)
    ratio_margin = terminal_peak - NONDECAY_RATIO * start_peak

    return {
        "property": property_name,
        "unit": RESIDUAL_RATE_PROPERTIES[property_name]["unit"],
        "threshold": threshold,
        "tail_start_window_lo_s": start_lo,
        "tail_start_window_hi_s": start_hi,
        "terminal_window_lo_s": terminal_lo,
        "terminal_window_hi_s": terminal_hi,
        "full_tail_window_lo_s": tail_lo,
        "full_tail_window_hi_s": tail_hi,
        "tail_start_peak_abs_rate": start_peak,
        "terminal_peak_abs_rate": terminal_peak,
        "rho_tier1": threshold - terminal_peak,
        "nondecay_ratio_margin": ratio_margin,
        "nondecay_slope_margin": slope,
    }


def summarize_residual_rate_repeats(
    repeat_metrics: list[dict[str, float | str]],
    *,
    sigma_multiplier: float = 2.0,
) -> dict[str, float | bool | str]:
    if not repeat_metrics:
        return {}
    numeric_keys = [
        "threshold",
        "tail_start_peak_abs_rate",
        "terminal_peak_abs_rate",
        "rho_tier1",
        "nondecay_ratio_margin",
        "nondecay_slope_margin",
    ]
    summary: dict[str, float | bool | str] = {
        "residual_rate_property": str(repeat_metrics[0]["property"]),
        "residual_rate_unit": str(repeat_metrics[0]["unit"]),
        "tier2_nondecay_ratio": NONDECAY_RATIO,
        "tier2_rule": "tier1 robust and (ratio margin robust >= 0 or slope margin robust >= 0)",
    }
    for key in numeric_keys:
        arr = np.asarray([float(row[key]) for row in repeat_metrics], dtype=float)
        summary[f"{key}_mean"] = float(np.mean(arr))
        summary[f"{key}_std"] = float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0
        summary[f"{key}_min"] = float(np.min(arr))
        summary[f"{key}_max"] = float(np.max(arr))

    rho_mean = float(summary["rho_tier1_mean"])
    rho_std = float(summary["rho_tier1_std"])
    if rho_mean + sigma_multiplier * rho_std < 0.0:
        tier1_class = "robust_violation"
    elif rho_mean - sigma_multiplier * rho_std > 0.0:
        tier1_class = "robust_safe"
    else:
        tier1_class = "noise_band"

    ratio_mean = float(summary["nondecay_ratio_margin_mean"])
    ratio_std = float(summary["nondecay_ratio_margin_std"])
    slope_mean = float(summary["nondecay_slope_margin_mean"])
    slope_std = float(summary["nondecay_slope_margin_std"])
    ratio_robust = ratio_mean - sigma_multiplier * ratio_std >= 0.0
    slope_robust = slope_mean - sigma_multiplier * slope_std >= 0.0
    nondecay_robust = bool(ratio_robust or slope_robust)
    if tier1_class == "robust_violation" and nondecay_robust:
        tier2_class = "robust_violation"
    elif tier1_class == "robust_violation":
        tier2_class = "tier1_only"
    else:
        tier2_class = "not_tier1_violation"

    summary.update(
        {
            "tier1_robustness_class": tier1_class,
            "tier2_robustness_class": tier2_class,
            "tier2_nondecay_ratio_robust": bool(ratio_robust),
            "tier2_nondecay_slope_robust": bool(slope_robust),
            "tier2_nondecay_robust": nondecay_robust,
        }
    )
    return summary


def _rate_series(parsed_log, property_name: str) -> np.ndarray:
    spec = RESIDUAL_RATE_PROPERTIES[property_name]
    field = spec["log_field"]
    if field in parsed_log:
        return parsed_log[field].to_numpy(dtype=float)
    if property_name == "post_neutral_yaw_rate":
        times = parsed_log["time_s"].to_numpy(dtype=float)
        yaw = np.unwrap(parsed_log["yaw_rad"].to_numpy(dtype=float))
        if times.size < 2:
            return np.zeros_like(times)
        return np.gradient(yaw, times)
    raise KeyError(f"Residual-rate log field missing for {property_name}: {field}")


def _absolute_window(t_neutral: float, relative_window: tuple[float, float]) -> tuple[float, float]:
    return float(t_neutral + relative_window[0]), float(t_neutral + relative_window[1])


def _window_tail(tail, window: tuple[float, float]):
    lo, hi = _validate_window(window)
    windowed = tail[(tail["time_s"] >= lo) & (tail["time_s"] <= hi)]
    if windowed.empty:
        raise ValueError(f"No telemetry samples in window [{lo}, {hi}] at or after t_neutral_s")
    return windowed


def _compute_residual_rate_rho_window(
    parsed_log,
    property_name: str,
    config,
    t_neutral: float,
    window: tuple[float, float],
) -> float:
    lo, hi = _validate_window(window)
    times = parsed_log["time_s"].to_numpy(dtype=float)
    abs_rate = np.abs(_rate_series(parsed_log, property_name))
    threshold = residual_rate_threshold(property_name, config)
    mask = (times >= t_neutral) & (times >= lo) & (times <= hi)
    if not np.any(mask):
        raise ValueError(f"No telemetry samples in window [{lo}, {hi}] at or after t_neutral_s")
    return float(threshold - np.max(abs_rate[mask]))


def _validate_window(window: tuple[float, float]) -> tuple[float, float]:
    if len(window) != 2:
        raise ValueError("window must be a (t0, t1) pair")
    lo, hi = float(window[0]), float(window[1])
    if hi < lo:
        raise ValueError(f"window end must be >= start: [{lo}, {hi}]")
    return lo, hi


def _window_peak(times: np.ndarray, values: np.ndarray, lo: float, hi: float) -> float:
    mask = (times >= lo) & (times <= hi)
    if not np.any(mask):
        raise ValueError(f"No telemetry samples in window [{lo}, {hi}]")
    return float(np.max(values[mask]))


def _window_slope(times: np.ndarray, values: np.ndarray, lo: float, hi: float) -> float:
    mask = (times >= lo) & (times <= hi)
    if np.sum(mask) < 2:
        raise ValueError(f"Need at least two telemetry samples in window [{lo}, {hi}]")
    x = times[mask] - float(times[mask][0])
    y = values[mask]
    denom = float(np.dot(x - np.mean(x), x - np.mean(x)))
    if denom <= 0.0:
        return 0.0
    return float(np.dot(x - np.mean(x), y - np.mean(y)) / denom)
