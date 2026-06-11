from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np
import pandas as pd

from cadet.groups import build_groups
from cadet.input_model import project_theta, theta_to_sequence, zero_theta


@dataclass(frozen=True)
class ManeuverSpec:
    name: str
    family: str
    axis: str
    kind: str
    amplitude: float
    hold_windows: int = 1
    repeat_count: int = 1
    scout_only: bool = False
    couplings: dict[str, float] = field(default_factory=dict)

    def to_record(self, config) -> dict[str, Any]:
        theta = self.to_theta(config)
        support = int(np.count_nonzero(np.abs(theta) > 1e-12))
        release_time_s = self.release_time_s(config)
        return {
            **asdict(self),
            "commanded_support_size": support,
            "release_time_s": release_time_s,
            "horizon_s": float(config.input["horizon_s"]),
            "window_s": float(config.input["window_s"]),
            "neutral_tail_s": float(config.input["neutral_tail_s"]),
        }

    def to_theta(self, config) -> np.ndarray:
        groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
        theta = zero_theta(groups)
        if self.family == "M0" or self.axis == "none":
            return theta
        channel_to_index = {(group.window_id, group.channel): group.group_id for group in groups}
        values = _window_values(self)
        for window_id, value in enumerate(values):
            key = (window_id, self.axis)
            if key in channel_to_index:
                theta[channel_to_index[key]] = value
            for channel, coupled_value in self.couplings.items():
                coupled_key = (window_id, channel)
                if coupled_key in channel_to_index:
                    theta[channel_to_index[coupled_key]] = float(coupled_value)
        return project_theta(theta, config)

    def to_sequence(self, config) -> pd.DataFrame:
        groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
        return theta_to_sequence(self.to_theta(config), groups, config)

    def release_time_s(self, config) -> float:
        if self.family == "M0" or self.axis == "none":
            return 0.0
        horizon_s = float(config.input["horizon_s"])
        window_s = float(config.input["window_s"])
        return min(horizon_s, len(_window_values(self)) * window_s)


def default_maneuvers(axis: str, *, max_strong: int | None = None) -> dict[str, list[ManeuverSpec]]:
    _validate_axis(axis)
    strong: list[ManeuverSpec] = []
    strong.extend(
        [
            ManeuverSpec(f"strong_step_{axis}_0p5", "M_strong", axis, "step", 0.5, 1),
            ManeuverSpec(f"strong_step_{axis}_0p5_hold2", "M_strong", axis, "step", 0.5, 2),
            ManeuverSpec(f"strong_doublet_{axis}_0p5", "M_strong", axis, "doublet", 0.5, 1),
            ManeuverSpec(f"strong_step_{axis}_0p7", "M_strong", axis, "step", 0.7, 1),
            ManeuverSpec("strong_doublet_A0p7_hold1", "M_strong", axis, "doublet", 0.7, 1),
            ManeuverSpec("strong_doublet_A0p9_hold1", "M_strong", axis, "doublet", 0.9, 1),
            ManeuverSpec("strong_doublet_A0p9_hold2", "M_strong", axis, "doublet", 0.9, 2),
            ManeuverSpec("pulse_train_A0p7_repeat3", "M_strong", axis, "pulse_train", 0.7, 1, 3),
            ManeuverSpec("pulse_train_A0p9_repeat3", "M_strong", axis, "pulse_train", 0.9, 1, 3),
            ManeuverSpec("reversal_A0p9_fast", "M_strong", axis, "reversal_fast", 0.9, 1, 1),
        ]
    )
    strong.append(ManeuverSpec(f"chirp_{axis}_scout", "M_strong", axis, "chirp", 0.5, 1, 1, scout_only=True))
    if max_strong is not None:
        strong = [m for m in strong if not m.scout_only][: int(max_strong)] + [m for m in strong if m.scout_only]
    return {
        "M0": [ManeuverSpec("hover_no_input", "M0", "none", "hover", 0.0, 0)],
        "M_small": [
            ManeuverSpec(f"small_step_{axis}", "M_small", axis, "step", 0.20, 1),
            ManeuverSpec(f"small_doublet_{axis}", "M_small", axis, "doublet", 0.20, 1),
        ],
        "M_strong": strong,
    }


def stress_metrics(parsed_log: pd.DataFrame, maneuver: ManeuverSpec, config) -> dict[str, Any]:
    horizon_s = float(config.input["horizon_s"])
    active = parsed_log[pd.to_numeric(parsed_log["time_s"], errors="coerce") < horizon_s]
    if active.empty:
        active = parsed_log
    metrics: dict[str, Any] = {
        "stress_proxy_note": "setpoint_unavailable; using actual rate and manual input as stress proxy",
    }
    for axis in ["roll", "pitch", "yaw"]:
        rate_col = f"{axis}_rate_rps"
        if rate_col in active:
            peak = float(np.nanmax(np.abs(pd.to_numeric(active[rate_col], errors="coerce"))))
            metrics[f"peak_abs_{axis}_rate"] = peak
            metrics[f"peak_abs_{axis}_rate_active"] = peak
        manual_col = f"manual_{axis}"
        if manual_col in active:
            energy = _manual_energy(active, manual_col)
            metrics[f"manual_{axis}_energy"] = energy
            if axis == maneuver.axis:
                metrics["manual_axis_energy"] = energy
        setpoint_col = _rate_setpoint_column(active, axis)
        if setpoint_col is not None:
            energy = _manual_energy(active, setpoint_col)
            metrics[f"rate_setpoint_{axis}_energy"] = energy
            if axis == maneuver.axis:
                metrics["rate_setpoint_energy"] = energy
    metrics["maneuver_axis"] = maneuver.axis
    metrics["maneuver_kind"] = maneuver.kind
    metrics["maneuver_amplitude"] = float(maneuver.amplitude)
    metrics["release_time_s"] = maneuver.release_time_s(config)
    return metrics


def _window_values(spec: ManeuverSpec) -> list[float]:
    if spec.kind == "step":
        return [spec.amplitude for _ in range(max(1, spec.hold_windows))]
    if spec.kind == "doublet":
        return [spec.amplitude for _ in range(max(1, spec.hold_windows))] + [
            -spec.amplitude for _ in range(max(1, spec.hold_windows))
        ]
    if spec.kind == "pulse_train":
        values: list[float] = []
        for _ in range(max(1, spec.repeat_count)):
            values.extend([spec.amplitude, -spec.amplitude])
        return values
    if spec.kind == "reversal_fast":
        amplitude = float(spec.amplitude)
        return [0.5 * amplitude, amplitude, 0.5 * amplitude, -0.1 * amplitude, -0.6 * amplitude, -amplitude, 0.0]
    if spec.kind == "chirp":
        return [0.2, -0.3, 0.4, -0.5, 0.5, -0.4, 0.3, -0.2]
    if spec.kind == "hover":
        return []
    raise KeyError(f"Unknown MANTIS maneuver kind: {spec.kind}")


def _manual_energy(df: pd.DataFrame, col: str) -> float:
    values = pd.to_numeric(df[col], errors="coerce").fillna(0.0).to_numpy(dtype=float)
    times = pd.to_numeric(df["time_s"], errors="coerce").fillna(0.0).to_numpy(dtype=float)
    if times.size < 2:
        return float(np.sum(np.abs(values)))
    return float(np.trapz(np.abs(values), times))


def _rate_setpoint_column(df: pd.DataFrame, axis: str) -> str | None:
    exact = [
        f"{axis}_rate_setpoint_rps",
        f"{axis}_rate_sp_rps",
        f"{axis}_rates_setpoint_rps",
        f"{axis}_rate_setpoint",
        f"{axis}_rate_sp",
    ]
    lower_to_col = {str(col).lower(): str(col) for col in df.columns}
    for name in exact:
        if name in lower_to_col:
            return lower_to_col[name]
    for col in df.columns:
        lowered = str(col).lower()
        if axis in lowered and "rate" in lowered and ("setpoint" in lowered or "sp" in lowered):
            return str(col)
    return None


def _validate_axis(axis: str) -> None:
    if axis not in {"roll", "pitch"}:
        raise ValueError(f"MANTIS pilot axis must be roll or pitch, got {axis}")


def _amp_label(value: float) -> str:
    return f"{value:.1f}".replace(".", "p")
