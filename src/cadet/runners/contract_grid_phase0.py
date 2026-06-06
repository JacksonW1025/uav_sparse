from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from cadet.config import ExperimentConfig, load_config
from cadet.groups import Group, build_groups
from cadet.input_model import project_theta, zero_theta
from cadet.query import read_parsed_log, theta_hash
from cadet.runners.direction_a_probe import (
    J_REPEATS,
    ROBUST_SIGMA_MULTIPLIER,
    _config_for_probe,
    _run_query_with_retry_count,
    _safe_label,
    classify_robustness,
    support_summary,
)
from cadet.violation_search import grid_to_theta, window_count


PREREG_PATH = Path("artifacts/contract_grid_prereg.md")
DEFAULT_PARAMS_PATH = Path("/home/car/PX4-Autopilot/build/px4_sitl_default/parameters.json")
DEFAULT_OUTPUT_DIR = Path("artifacts/contract_grid_summary")
PHASE = "Phase-0"
SCENARIO_ID = "px4_position"
SEED = 0
TERMINAL_WINDOW_S = (11.0, 13.0)
ACTIVE_WINDOW_S = (2.0, 5.0)
STICK_LIMIT = 1.0
REQUIRED_PARAMS = [
    "MPC_HOLD_DZ",
    "MPC_VEL_MANUAL",
    "MPC_Z_VEL_MAX_UP",
    "MPC_Z_VEL_MAX_DN",
    "MPC_MAN_Y_MAX",
    "MPC_XY_VEL_MAX",
    "MPC_MAN_TILT_MAX",
]


@dataclass(frozen=True)
class ParameterValue:
    name: str
    value: float
    units: str
    source_default: float
    source_units: str
    source_path: str


@dataclass(frozen=True)
class CellSpec:
    cell_id: str
    input_class: str
    contract_id: str
    contract_name: str
    commanded_channels: tuple[str, ...]
    measured_axes: tuple[str, ...]
    window: tuple[float, float]


@dataclass(frozen=True)
class ProbeSpec:
    cell: CellSpec
    label: str
    description: str
    shape: str
    channel_signs: tuple[tuple[str, int], ...]
    theta: np.ndarray


CELL_SPECS = [
    CellSpec("G01", "I1", "C1", "Brake", ("roll",), ("xy_velocity",), TERMINAL_WINDOW_S),
    CellSpec("G02", "I1", "C1", "Brake", ("pitch",), ("xy_velocity",), TERMINAL_WINDOW_S),
    CellSpec("G03", "I1", "C1", "Brake", ("throttle",), ("climb_rate",), TERMINAL_WINDOW_S),
    CellSpec("G04", "I1", "C1", "Brake", ("yaw",), ("yaw_rate",), TERMINAL_WINDOW_S),
    CellSpec("G05", "I1", "C2", "Level", ("roll",), ("tilt",), TERMINAL_WINDOW_S),
    CellSpec("G06", "I1", "C2", "Level", ("pitch",), ("tilt",), TERMINAL_WINDOW_S),
    CellSpec("G07", "I2", "C3", "Envelope", ("roll",), ("v_xy", "tilt"), ACTIVE_WINDOW_S),
    CellSpec("G08", "I2", "C3", "Envelope", ("pitch",), ("v_xy", "tilt"), ACTIVE_WINDOW_S),
    CellSpec("G09", "I2", "C3", "Envelope", ("throttle",), ("climb_rate_up", "climb_rate_down"), ACTIVE_WINDOW_S),
    CellSpec("G10", "I3", "C4", "Coupling", ("throttle",), ("xy_velocity",), TERMINAL_WINDOW_S),
    CellSpec("G11", "I3", "C4", "Coupling", ("roll", "throttle"), ("xy_velocity", "climb_rate"), TERMINAL_WINDOW_S),
    CellSpec("G12", "I3", "C4", "Coupling", ("pitch", "throttle"), ("xy_velocity", "climb_rate"), TERMINAL_WINDOW_S),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Contract grid Phase-0 screening for PX4 px4_position seed 0.")
    parser.add_argument("--config", default="configs/rq1_minimal.yaml")
    parser.add_argument("--scenario", default=SCENARIO_ID)
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument("--run-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--params", default=str(DEFAULT_PARAMS_PATH))
    parser.add_argument("--repeats", type=int, default=J_REPEATS)
    parser.add_argument("--stick-limit", type=float, default=STICK_LIMIT)
    args = parser.parse_args()

    if args.scenario != SCENARIO_ID:
        raise ValueError("Contract grid Phase-0 is frozen to px4_position")
    if int(args.seed) != SEED:
        raise ValueError("Contract grid Phase-0 is frozen to seed 0")
    if int(args.repeats) != J_REPEATS:
        raise ValueError("Contract grid Phase-0 is pre-registered to J=5 repeats")
    if float(args.stick_limit) != STICK_LIMIT:
        raise ValueError("Contract grid Phase-0 is frozen to stick limit +/-1.0")
    if not PREREG_PATH.exists():
        raise FileNotFoundError(f"Missing pre-registration artifact: {PREREG_PATH}")

    run_start = time.monotonic()
    output_dir = Path(args.run_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(PREREG_PATH, output_dir / "contract_grid_prereg.md")

    base_config = load_config(args.config)
    config = _config_for_probe(base_config, output_dir, float(args.stick_limit))
    scenario = config.scenario_by_id(args.scenario)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    if len(groups) != 40:
        raise ValueError(f"Frozen D=40 parameterization expected 40 groups, found {len(groups)}")
    pd.DataFrame([group.__dict__ for group in groups]).to_csv(output_dir / "groups.csv", index=False)

    provenance = preregistration_provenance(PREREG_PATH)
    params = load_px4_parameter_defaults(Path(args.params))
    thresholds = contract_thresholds(params)
    params_df = pd.DataFrame(params_used_rows(params, thresholds))
    params_df.to_csv(output_dir / "params_used.csv", index=False)

    probe_rows: list[dict[str, Any]] = []
    repeat_rows: list[dict[str, Any]] = []
    successful_query_count = 0
    timeout_retry_count = 0
    print(
        f"contract_grid_phase0_start provenance={provenance['label']} "
        f"scenario={args.scenario} seed={args.seed} J={args.repeats} run_dir={output_dir}",
        flush=True,
    )
    for cell in CELL_SPECS:
        for probe in build_cell_probes(cell, config, groups):
            rows, repeats_detail, retries = eval_probe(
                probe,
                scenario,
                int(args.seed),
                output_dir,
                config,
                groups,
                thresholds,
                provenance_label=provenance["label"],
                repeats=int(args.repeats),
            )
            probe_rows.extend(rows)
            repeat_rows.extend(repeats_detail)
            successful_query_count += int(args.repeats)
            timeout_retry_count += retries
            pd.DataFrame(probe_rows).to_csv(output_dir / "probe_points.csv", index=False)
            pd.DataFrame(repeat_rows).to_csv(output_dir / "probe_repeats.csv", index=False)

    probe_df = pd.DataFrame(probe_rows)
    cell_df = pd.DataFrame(grid_cell_rows(probe_df, provenance_label=provenance["label"]))
    cell_df.to_csv(output_dir / "grid_cells.csv", index=False)
    pd.DataFrame(repeat_rows).to_csv(output_dir / "probe_repeats.csv", index=False)

    report_path = output_dir / "contract_grid_report.md"
    write_report(
        report_path,
        provenance=provenance,
        params_df=params_df,
        cell_df=cell_df,
        probe_df=probe_df,
        elapsed_wall_time_s=time.monotonic() - run_start,
        successful_query_count=successful_query_count,
        timeout_retry_count=timeout_retry_count,
        output_dir=output_dir,
    )
    remove_ulg_files(output_dir)
    print(
        f"contract_grid_phase0_complete provenance={provenance['label']} "
        f"successful_queries={successful_query_count} timeout_retries={timeout_retry_count} "
        f"report={report_path}",
        flush=True,
    )


def load_px4_parameter_defaults(path: Path = DEFAULT_PARAMS_PATH) -> dict[str, ParameterValue]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    items = data.get("parameters", [])
    if not isinstance(items, list):
        raise ValueError(f"PX4 parameters JSON has unexpected 'parameters' type: {type(items).__name__}")
    by_name = {str(item.get("name")): item for item in items if isinstance(item, dict) and item.get("name")}
    missing = [name for name in REQUIRED_PARAMS if name not in by_name]
    if missing:
        raise KeyError(f"PX4 parameters missing required defaults: {', '.join(missing)}")
    result: dict[str, ParameterValue] = {}
    for name in REQUIRED_PARAMS:
        item = by_name[name]
        source_default = float(item["default"])
        source_units = str(item.get("units", ""))
        value, units = normalize_parameter_units(name, source_default, source_units)
        result[name] = ParameterValue(
            name=name,
            value=value,
            units=units,
            source_default=source_default,
            source_units=source_units,
            source_path=str(path),
        )
    return result


def normalize_parameter_units(name: str, value: float, units: str) -> tuple[float, str]:
    if name == "MPC_MAN_Y_MAX":
        return math.radians(float(value)), "rad/s"
    if name == "MPC_MAN_TILT_MAX":
        return math.radians(float(value)), "rad"
    return float(value), units or "scalar"


def contract_thresholds(params: dict[str, ParameterValue]) -> dict[str, dict[str, float]]:
    c1 = {
        "xy_velocity": 0.10 * params["MPC_VEL_MANUAL"].value,
        "climb_rate": 0.10 * params["MPC_Z_VEL_MAX_UP"].value,
        "yaw_rate": 0.10 * params["MPC_MAN_Y_MAX"].value,
    }
    c3 = {
        "v_xy": params["MPC_XY_VEL_MAX"].value,
        "tilt": params["MPC_MAN_TILT_MAX"].value,
        "climb_rate_up": params["MPC_Z_VEL_MAX_UP"].value,
        "climb_rate_down": params["MPC_Z_VEL_MAX_DN"].value,
    }
    return {
        "C1": c1,
        "C2": {"tilt": 0.10 * params["MPC_MAN_TILT_MAX"].value},
        "C3": c3,
        "C4": dict(c1),
    }


def params_used_rows(
    params: dict[str, ParameterValue],
    thresholds: dict[str, dict[str, float]],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []

    def add(name: str, contract: str, axis: str, threshold: float, threshold_units: str, derivation: str) -> None:
        param = params[name]
        rows.append(
            {
                "parameter_name": name,
                "parameter_value": param.value,
                "parameter_units": param.units,
                "source_default": param.source_default,
                "source_units": param.source_units,
                "source_path": param.source_path,
                "contract": contract,
                "measured_axis": axis,
                "derived_threshold": float(threshold),
                "threshold_units": threshold_units,
                "derivation": derivation,
            }
        )

    add("MPC_HOLD_DZ", "centered-stick neutral provenance for C1/C2/C4", "stick_deadzone", params["MPC_HOLD_DZ"].value, "normalized_stick", "compiled default")
    add("MPC_VEL_MANUAL", "C1 Brake", "xy_velocity", thresholds["C1"]["xy_velocity"], "m/s", "0.10*MPC_VEL_MANUAL")
    add("MPC_Z_VEL_MAX_UP", "C1 Brake", "climb_rate", thresholds["C1"]["climb_rate"], "m/s", "0.10*MPC_Z_VEL_MAX_UP")
    add("MPC_MAN_Y_MAX", "C1 Brake", "yaw_rate", thresholds["C1"]["yaw_rate"], "rad/s", "0.10*MPC_MAN_Y_MAX")
    add("MPC_MAN_TILT_MAX", "C2 Level", "tilt", thresholds["C2"]["tilt"], "rad", "0.10*MPC_MAN_TILT_MAX")
    add("MPC_XY_VEL_MAX", "C3 Envelope", "v_xy", thresholds["C3"]["v_xy"], "m/s", "MPC_XY_VEL_MAX")
    add("MPC_MAN_TILT_MAX", "C3 Envelope", "tilt", thresholds["C3"]["tilt"], "rad", "MPC_MAN_TILT_MAX")
    add("MPC_Z_VEL_MAX_UP", "C3 Envelope", "climb_rate_up", thresholds["C3"]["climb_rate_up"], "m/s", "MPC_Z_VEL_MAX_UP")
    add("MPC_Z_VEL_MAX_DN", "C3 Envelope", "climb_rate_down", thresholds["C3"]["climb_rate_down"], "m/s", "MPC_Z_VEL_MAX_DN")
    add("MPC_VEL_MANUAL", "C4 Coupling", "xy_velocity", thresholds["C4"]["xy_velocity"], "m/s", "0.10*MPC_VEL_MANUAL")
    add("MPC_Z_VEL_MAX_UP", "C4 Coupling", "climb_rate", thresholds["C4"]["climb_rate"], "m/s", "0.10*MPC_Z_VEL_MAX_UP")
    return rows


def build_cell_probes(cell: CellSpec, config: ExperimentConfig, groups: list[Group]) -> list[ProbeSpec]:
    probes = [
        ProbeSpec(
            cell=cell,
            label="zero_anchor",
            description="all-neutral zero anchor",
            shape="zero",
            channel_signs=(),
            theta=zero_theta(groups),
        )
    ]
    if cell.cell_id in {"G01", "G02", "G03", "G04", "G05", "G06", "G07", "G08", "G09", "G10"}:
        channel = cell.commanded_channels[0]
        shape = "I2_step_hold" if cell.input_class == "I2" else cell.input_class
        for sign in (+1, -1):
            label = f"{channel}_{sign_label(sign)}_full"
            probes.append(
                ProbeSpec(
                    cell=cell,
                    label=label,
                    description=f"saturated {sign_label(sign)} {channel}; active [0,5] then neutral tail",
                    shape=shape,
                    channel_signs=((channel, sign),),
                    theta=theta_for_channel_signs(config, groups, ((channel, sign),)),
                )
            )
        return probes
    if cell.cell_id in {"G11", "G12"}:
        lateral, throttle = cell.commanded_channels
        for signs in ((+1, +1), (+1, -1)):
            channel_signs = ((lateral, signs[0]), (throttle, signs[1]))
            sign_text = "".join(sign_label(sign) for sign in signs)
            probes.append(
                ProbeSpec(
                    cell=cell,
                    label=f"{lateral}_throttle_{sign_text}_full",
                    description=f"saturated coordinated {lateral} {sign_label(signs[0])}, throttle {sign_label(signs[1])}",
                    shape="I3_coordinated",
                    channel_signs=channel_signs,
                    theta=theta_for_channel_signs(config, groups, channel_signs),
                )
            )
        return probes
    raise ValueError(f"No probe rule for cell {cell.cell_id}")


def theta_for_channel_signs(
    config: ExperimentConfig,
    groups: list[Group],
    channel_signs: tuple[tuple[str, int], ...],
    amplitude: float = STICK_LIMIT,
) -> np.ndarray:
    channels = list(config.input["channels"])
    n_windows = window_count(config)
    grid = np.zeros((n_windows, len(channels)), dtype=float)
    for channel, sign in channel_signs:
        if channel not in channels:
            raise ValueError(f"Input channel missing from frozen config: {channel}")
        grid[:, channels.index(channel)] = float(sign) * float(amplitude)
    return project_theta(grid_to_theta(grid, config, groups), config)


def eval_probe(
    probe: ProbeSpec,
    scenario,
    seed: int,
    output_dir: Path,
    config: ExperimentConfig,
    groups: list[Group],
    thresholds: dict[str, dict[str, float]],
    *,
    provenance_label: str,
    repeats: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], int]:
    projected = project_theta(np.asarray(probe.theta, dtype=float), config)
    thash = theta_hash(projected)
    theta_path = output_dir / "thetas" / f"{probe.cell.cell_id}_{probe.label}_{thash}.npy"
    theta_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(theta_path, projected)
    support = support_summary(projected, groups)
    max_abs_theta = float(np.max(np.abs(projected))) if projected.size else 0.0
    axes = axes_for_probe(probe)
    peaks_by_axis: dict[str, list[float]] = {axis: [] for axis in axes}
    rhos_by_axis: dict[str, list[float]] = {axis: [] for axis in axes}
    query_ids: list[str] = []
    repeat_rows: list[dict[str, Any]] = []
    retry_total = 0
    for repeat_idx in range(repeats):
        cache_tag = _safe_label(f"contract_grid_{probe.cell.cell_id}_{probe.label}_repeat{repeat_idx}")
        result, retry_count = _run_query_with_retry_count(
            projected,
            scenario,
            seed,
            "contract_grid_phase0",
            output_dir,
            config,
            cache_tag=cache_tag,
            use_cache=True,
        )
        retry_total += retry_count
        query_ids.append(result.query_id)
        parsed_log = read_parsed_log(result.parsed_log_path)
        for axis in axes:
            peak = peak_for_axis(parsed_log, axis, probe.cell.window)
            threshold = threshold_for_axis(thresholds, probe.cell.contract_id, axis)
            rho = threshold - peak
            peaks_by_axis[axis].append(peak)
            rhos_by_axis[axis].append(rho)
            repeat_rows.append(
                {
                    "cell_id": probe.cell.cell_id,
                    "probe_label": probe.label,
                    "repeat_idx": repeat_idx,
                    "measured_axis": axis,
                    "peak": peak,
                    "threshold": threshold,
                    "rho": rho,
                    "query_id": result.query_id,
                    "theta_hash": result.theta_hash,
                    "cache_tag": cache_tag,
                    "query_retry_count": retry_count,
                }
            )

    rows: list[dict[str, Any]] = []
    for axis in axes:
        peaks = np.asarray(peaks_by_axis[axis], dtype=float)
        rhos = np.asarray(rhos_by_axis[axis], dtype=float)
        rho_mean = float(np.mean(rhos))
        rho_std = float(np.std(rhos, ddof=1)) if rhos.size > 1 else 0.0
        label = classify_robustness(rho_mean, rho_std)
        threshold = threshold_for_axis(thresholds, probe.cell.contract_id, axis)
        rows.append(
            {
                "provenance_label": provenance_label,
                "phase": PHASE,
                "scenario": SCENARIO_ID,
                "seed": seed,
                "cell_id": probe.cell.cell_id,
                "input": probe.cell.input_class,
                "contract": f"{probe.cell.contract_id} {probe.cell.contract_name}",
                "contract_id": probe.cell.contract_id,
                "commanded_channels": "+".join(probe.cell.commanded_channels),
                "probe_label": probe.label,
                "probe_description": probe.description,
                "probe_shape": probe.shape,
                "channel_signs": channel_sign_text(probe.channel_signs),
                "measured_axis": axis,
                "window": f"[{probe.cell.window[0]:.0f},{probe.cell.window[1]:.0f}]",
                "threshold": threshold,
                "peak": float(np.mean(peaks)),
                "peak_mean": float(np.mean(peaks)),
                "peak_std": float(np.std(peaks, ddof=1)) if peaks.size > 1 else 0.0,
                "peak_min": float(np.min(peaks)),
                "peak_max": float(np.max(peaks)),
                "rho_mean": rho_mean,
                "rho_std": rho_std,
                "rho_min": float(np.min(rhos)),
                "rho_max": float(np.max(rhos)),
                "rho_mean_plus_2rho_std": rho_mean + ROBUST_SIGMA_MULTIPLIER * rho_std,
                "rho_mean_minus_2rho_std": rho_mean - ROBUST_SIGMA_MULTIPLIER * rho_std,
                "robust_label": label,
                "theta_hash": thash,
                "theta_path": str(theta_path),
                "repeats": repeats,
                "query_ids": ";".join(query_ids),
                "timeout_retry_count": retry_total,
                "max_abs_theta": max_abs_theta,
                "support_size_abs_gt_0p1": int(support["support_size"]),
                "active_channels_abs_gt_0p1": ",".join(support["active_channels"]),
            }
        )
    rho2_text = ",".join(f"{row['rho_mean_plus_2rho_std']:.6f}" for row in rows)
    print(
        f"contract_grid_eval cell={probe.cell.cell_id} probe={probe.label} "
        f"axes={','.join(axes)} labels={','.join(row['robust_label'] for row in rows)} "
        f"rho2={rho2_text}",
        flush=True,
    )
    return rows, repeat_rows, retry_total


def axes_for_probe(probe: ProbeSpec) -> tuple[str, ...]:
    if probe.cell.cell_id == "G09" and probe.channel_signs:
        sign_by_channel = dict(probe.channel_signs)
        sign = int(sign_by_channel["throttle"])
        return ("climb_rate_up",) if sign > 0 else ("climb_rate_down",)
    return probe.cell.measured_axes


def peak_for_axis(parsed_log: pd.DataFrame, axis: str, window: tuple[float, float]) -> float:
    ensure_log_fields(parsed_log)
    times = parsed_log["time_s"].to_numpy(dtype=float)
    lo, hi = window
    if axis in {"xy_velocity", "v_xy"}:
        values = np.hypot(parsed_log["vx_mps"].to_numpy(dtype=float), parsed_log["vy_mps"].to_numpy(dtype=float))
    elif axis == "climb_rate":
        values = np.abs(parsed_log["vz_mps"].to_numpy(dtype=float))
    elif axis == "climb_rate_up":
        values = np.maximum(parsed_log["vz_mps"].to_numpy(dtype=float), 0.0)
    elif axis == "climb_rate_down":
        values = np.maximum(-parsed_log["vz_mps"].to_numpy(dtype=float), 0.0)
    elif axis == "yaw_rate":
        values = np.abs(parsed_log["yaw_rate_rps"].to_numpy(dtype=float))
    elif axis == "tilt":
        roll = parsed_log["roll_rad"].to_numpy(dtype=float)
        pitch = parsed_log["pitch_rad"].to_numpy(dtype=float)
        values = np.arccos(np.clip(np.cos(roll) * np.cos(pitch), -1.0, 1.0))
    else:
        raise KeyError(f"Unknown contract axis: {axis}")
    mask = (times >= float(lo)) & (times <= float(hi))
    if not np.any(mask):
        raise ValueError(f"No telemetry samples in window [{lo}, {hi}] for axis {axis}")
    return float(np.max(values[mask]))


def ensure_log_fields(parsed_log: pd.DataFrame) -> None:
    required = ["vx_mps", "vy_mps", "vz_mps", "yaw_rate_rps", "roll_rad", "pitch_rad", "time_s"]
    missing = [field for field in required if field not in parsed_log]
    if missing:
        raise KeyError(f"Parsed log missing required contract-grid fields: {', '.join(missing)}")


def threshold_for_axis(thresholds: dict[str, dict[str, float]], contract_id: str, axis: str) -> float:
    if contract_id == "C3" and axis in {"xy_velocity", "v_xy"}:
        axis = "v_xy"
    return float(thresholds[contract_id][axis])


def grid_cell_rows(probe_df: pd.DataFrame, *, provenance_label: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for cell in CELL_SPECS:
        df = probe_df[probe_df["cell_id"] == cell.cell_id]
        labels = set(df["robust_label"].astype(str)) if not df.empty else set()
        if "robust_violation" in labels:
            status = "violable"
            eligible = True
        elif labels == {"robust_safe"}:
            status = "robust_safe"
            eligible = False
        else:
            status = "noise_band"
            eligible = False
        rows.append(
            {
                "provenance_label": provenance_label,
                "phase": PHASE,
                "scenario": SCENARIO_ID,
                "seed": SEED,
                "cell_id": cell.cell_id,
                "input": cell.input_class,
                "contract": f"{cell.contract_id} {cell.contract_name}",
                "contract_id": cell.contract_id,
                "commanded_channels": "+".join(cell.commanded_channels),
                "measured_axis": ",".join(cell.measured_axes),
                "status": status,
                "phase1_eligible": bool(eligible),
            }
        )
    return rows


def preregistration_provenance(path: Path) -> dict[str, Any]:
    tracked = _git(["ls-files", "--error-unmatch", str(path)]).returncode == 0
    log = _git(["log", "-1", "--format=%H %cI %s", "--", str(path)])
    last_commit = log.stdout.strip() if log.returncode == 0 else ""
    status = _git(["status", "--short", "--", str(path)])
    status_text = status.stdout.strip() if status.returncode == 0 else ""
    committed = bool(tracked and last_commit)
    label = "confirmatory-protocol" if committed and not status_text.startswith("??") else "exploratory-hypothesis"
    return {
        "label": label,
        "path": str(path),
        "exists": path.exists(),
        "git_tracked": tracked,
        "git_last_commit": last_commit,
        "git_status_short": status_text,
        "rule": "confirmatory only if preregistration artifact was committed before grid evaluation",
    }


def _git(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["git", *args], text=True, capture_output=True, check=False)


def write_report(
    path: Path,
    *,
    provenance: dict[str, Any],
    params_df: pd.DataFrame,
    cell_df: pd.DataFrame,
    probe_df: pd.DataFrame,
    elapsed_wall_time_s: float,
    successful_query_count: int,
    timeout_retry_count: int,
    output_dir: Path,
) -> None:
    lines = [
        "# Contract Grid Phase-0 Report",
        "",
        f"Provenance label: `{provenance['label']}`, PX4, seed 0, Phase-0.",
        f"Pre-registration path: `{provenance['path']}`.",
        f"Pre-registration git status: `{provenance['git_status_short'] or 'clean/tracked'}`.",
        f"Pre-registration last commit: `{provenance['git_last_commit'] or 'none'}`.",
        f"Successful queries: `{successful_query_count}`; timeout retries: `{timeout_retry_count}`; elapsed wall time: `{elapsed_wall_time_s:.1f}s`.",
        "",
        "## Params Used",
        "",
        markdown_table(params_df),
        "",
        "## Grid Cells",
        "",
        markdown_table(cell_df),
        "",
        "## Probe Points",
        "",
    ]
    for cell_id in [cell.cell_id for cell in CELL_SPECS]:
        lines.extend(
            [
                f"### {cell_id}",
                "",
                markdown_table(probe_df[probe_df["cell_id"] == cell_id].copy()),
                "",
            ]
        )
    lines.extend(
        [
            "## Artifacts",
            "",
            f"- params_used: `{output_dir / 'params_used.csv'}`",
            f"- grid_cells: `{output_dir / 'grid_cells.csv'}`",
            f"- probe_points: `{output_dir / 'probe_points.csv'}`",
            f"- probe_repeats: `{output_dir / 'probe_repeats.csv'}`",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def markdown_table(df: pd.DataFrame) -> str:
    if df.empty:
        return "_empty_"
    show = df.copy()
    for col in show.columns:
        show[col] = show[col].map(format_cell)
    headers = list(show.columns)
    rows = show.to_dict("records")
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row[col]) for col in headers) + " |")
    return "\n".join(lines)


def format_cell(value: Any) -> str:
    if isinstance(value, (float, np.floating)):
        if math.isnan(float(value)):
            return "nan"
        return f"{float(value):.9g}"
    return str(value).replace("|", "\\|")


def remove_ulg_files(path: Path) -> None:
    for ulg_path in Path(path).rglob("*.ulg"):
        ulg_path.unlink()


def sign_label(sign: int) -> str:
    return "plus" if int(sign) > 0 else "minus"


def channel_sign_text(channel_signs: tuple[tuple[str, int], ...]) -> str:
    if not channel_signs:
        return "neutral"
    return ";".join(f"{channel}:{'+' if int(sign) > 0 else '-'}" for channel, sign in channel_signs)


if __name__ == "__main__":
    main()
