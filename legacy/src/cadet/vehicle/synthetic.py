from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from cadet.config import ScenarioCfg
from cadet.groups import build_groups
from cadet.vehicle.base import VehicleAdapter


class SyntheticAdapter(VehicleAdapter):
    """Simulator-free adapter with a known sparse response."""

    TRUE_SUPPORT = {
        "post_neutral_xy_drift": {1, 6, 17, 28, 37},
        "post_neutral_alt_drift": {3, 10, 19, 26, 35},
        "post_neutral_xy_velocity": {0, 9, 18, 25, 34},
    }

    def __init__(self, config, noise_fraction: float = 0.0):
        self.config = config
        self.noise_fraction = noise_fraction
        self.seed = 0
        inp = config.input
        self.groups = build_groups(inp["horizon_s"], inp["window_s"], inp["channels"])
        self.theta: np.ndarray | None = None

    def prepare(self, scenario: ScenarioCfg, seed: int) -> None:
        self.seed = seed

    def run(self, input_sequence: pd.DataFrame, scenario: ScenarioCfg, output_dir: Path) -> Path:
        output_dir.mkdir(parents=True, exist_ok=True)
        theta = self._sequence_to_theta(input_sequence)
        raw_path = output_dir / "synthetic_raw.npz"
        np.savez(raw_path, theta=theta, seed=self.seed, noise_fraction=self.noise_fraction)
        return raw_path

    def parse_log(self, raw_log_path: Path) -> pd.DataFrame:
        data = np.load(raw_log_path)
        theta = data["theta"]
        rng = np.random.default_rng(int(data["seed"]))
        horizon_s = float(self.config.input["horizon_s"])
        neutral_tail_s = float(self.config.input["neutral_tail_s"])
        times = np.arange(0.0, horizon_s + neutral_tail_s + 0.02 / 2.0, 0.02)
        neutral_mask = times >= horizon_s
        local_t = np.clip(times - horizon_s, 0.0, None)

        base_alt = 5.0
        x = np.zeros_like(times)
        y = np.zeros_like(times)
        alt = np.full_like(times, base_alt)
        vx = np.zeros_like(times)
        vy = np.zeros_like(times)

        xy_amp = self._linear_response(theta, "post_neutral_xy_drift")
        alt_amp = self._linear_response(theta, "post_neutral_alt_drift")
        vel_amp = self._linear_response(theta, "post_neutral_xy_velocity")
        growth = np.where(neutral_mask, 1.0 - np.exp(-local_t / 2.0), 0.0)

        x += (0.35 + xy_amp) * growth
        alt += (0.20 + alt_amp) * growth
        vx += (0.20 + vel_amp) * np.where(neutral_mask, np.exp(-local_t / 5.0), 0.0)

        noise_fraction = float(data["noise_fraction"])
        if noise_fraction > 0:
            scale = noise_fraction * 0.08
            x += rng.normal(0.0, scale, size=times.shape) * neutral_mask
            alt += rng.normal(0.0, scale, size=times.shape) * neutral_mask
            vx += rng.normal(0.0, scale, size=times.shape) * neutral_mask

        return pd.DataFrame(
            {
                "time_s": times,
                "x_m": x,
                "y_m": y,
                "z_m": alt,
                "alt_m": alt,
                "vx_mps": vx,
                "vy_mps": vy,
                "vz_mps": np.zeros_like(times),
                "roll_rad": np.zeros_like(times),
                "pitch_rad": np.zeros_like(times),
                "yaw_rad": np.zeros_like(times),
                "mode": "Synthetic",
                "t_zero_s": 0.0,
                "t_neutral_s": horizon_s,
            }
        )

    def shutdown(self) -> None:
        return None

    def _sequence_to_theta(self, input_sequence: pd.DataFrame) -> np.ndarray:
        theta = np.zeros(len(self.groups), dtype=float)
        for group in self.groups:
            mask = (input_sequence["t_s"] >= group.t_start) & (input_sequence["t_s"] < group.t_end)
            theta[group.group_id] = float(input_sequence.loc[mask, group.channel].mean())
        return theta

    def _linear_response(self, theta: np.ndarray, property_name: str) -> float:
        support = self.TRUE_SUPPORT[property_name]
        weights = np.zeros_like(theta)
        for rank, group_id in enumerate(sorted(support)):
            weights[group_id] = 1.0 - rank * 0.08
        return float(np.dot(weights, theta))
