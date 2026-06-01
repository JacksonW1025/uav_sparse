from __future__ import annotations

import numpy as np
import pandas as pd

from sparsepilot.groups import Group


def zero_theta(groups: list[Group]) -> np.ndarray:
    return np.zeros(len(groups), dtype=float)


def _channel_order(groups: list[Group]) -> list[str]:
    return list(dict.fromkeys(group.channel for group in groups))


def project_theta(theta: np.ndarray, config) -> np.ndarray:
    groups = build_groups_from_config(config)
    projected = np.asarray(theta, dtype=float).copy()
    inp = config.input
    projected = np.clip(projected, inp["min_value"], inp["max_value"])

    channels = _channel_order(groups)
    by_key = {(g.window_id, g.channel): g.group_id for g in groups}
    num_windows = int(round(inp["horizon_s"] / inp["window_s"]))
    max_step = float(inp["max_delta_per_window"])
    for channel in channels:
        prev = 0.0
        for window_id in range(num_windows):
            idx = by_key[(window_id, channel)]
            projected[idx] = np.clip(projected[idx], prev - max_step, prev + max_step)
            prev = projected[idx]
    return projected


def perturb_group(theta: np.ndarray, group_id: int, delta: float, sign: int, config) -> np.ndarray:
    perturbed = np.asarray(theta, dtype=float).copy()
    perturbed[group_id] += sign * delta
    return project_theta(perturbed, config)


def theta_to_sequence(theta: np.ndarray, groups: list[Group], config) -> pd.DataFrame:
    theta = np.asarray(theta, dtype=float)
    inp = config.input
    hz = float(config.simulator.get("synthetic", {}).get("manual_control_hz", 50))
    if "manual_control_hz" in inp:
        hz = float(inp["manual_control_hz"])
    horizon_s = float(inp["horizon_s"])
    neutral_tail_s = float(inp["neutral_tail_s"])
    dt = 1.0 / hz
    total_s = horizon_s + neutral_tail_s
    times = np.round(np.arange(0.0, total_s + dt / 2.0, dt), 10)
    channels = _channel_order(groups)
    data = {"t_s": times}
    for channel in channels:
        data[channel] = np.zeros_like(times)
    for group in groups:
        mask = (times >= group.t_start) & (times < group.t_end) & (times < horizon_s)
        data[group.channel][mask] = theta[group.group_id]
    return pd.DataFrame(data)


def build_groups_from_config(config) -> list[Group]:
    from sparsepilot.groups import build_groups

    inp = config.input
    return build_groups(inp["horizon_s"], inp["window_s"], inp["channels"])
