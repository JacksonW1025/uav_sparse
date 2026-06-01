from __future__ import annotations

import numpy as np


def compute_robustness(parsed_log, property_name: str, config) -> float:
    t_neutral = float(parsed_log["t_neutral_s"].iloc[0])
    tail = parsed_log[parsed_log["time_s"] >= t_neutral]
    if tail.empty:
        raise ValueError("No samples at or after t_neutral_s")
    first = tail.iloc[0]
    if property_name == "post_neutral_xy_drift":
        d_max = float(config.properties[property_name]["d_max_m"])
        drift = np.sqrt((tail["x_m"] - first["x_m"]) ** 2 + (tail["y_m"] - first["y_m"]) ** 2)
        return float(d_max - drift.max())
    if property_name == "post_neutral_alt_drift":
        h_max = float(config.properties[property_name]["h_max_m"])
        drift = np.abs(tail["alt_m"] - float(first["alt_m"]))
        return float(h_max - drift.max())
    if property_name == "post_neutral_xy_velocity":
        v_max = float(config.properties[property_name]["v_max_mps"])
        speed = np.sqrt(tail["vx_mps"] ** 2 + tail["vy_mps"] ** 2)
        return float(v_max - speed.max())
    raise KeyError(f"Unknown property: {property_name}")


def compute_all_properties(parsed_log, property_names: list[str], config) -> dict[str, float]:
    return {name: compute_robustness(parsed_log, name, config) for name in property_names}
