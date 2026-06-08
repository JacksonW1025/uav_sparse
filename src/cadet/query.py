from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from cadet.config import ScenarioCfg
from cadet.groups import build_groups
from cadet.input_model import project_theta, theta_to_sequence
from cadet.properties import compute_all_properties
from cadet.vehicle.ardupilot import ArduPilotAdapter
from cadet.vehicle.px4 import PX4Adapter
from cadet.vehicle.synthetic import SyntheticAdapter

XY_SPEED_DIAGNOSTIC_WINDOWS_S = ((5.0, 7.0), (7.0, 9.0), (9.0, 11.0), (11.0, 13.0))


@dataclass
class QueryResult:
    query_id: str
    theta_hash: str
    robustness: dict[str, float]
    parsed_log_path: Path
    metadata: dict


def theta_hash(theta: np.ndarray) -> str:
    arr = np.asarray(theta, dtype=np.float64)
    return hashlib.sha256(arr.tobytes()).hexdigest()[:16]


def make_adapter(platform: str, config):
    if platform == "synthetic":
        noise_fraction = float(config.simulator.get("synthetic", {}).get("noise_fraction", 0.0))
        return SyntheticAdapter(config, noise_fraction=noise_fraction)
    if platform == "px4":
        return PX4Adapter(config)
    if platform == "ardupilot":
        return ArduPilotAdapter(config)
    raise NotImplementedError(f"{platform} adapter belongs to Phase 1")


def run_query(
    theta,
    scenario: ScenarioCfg,
    seed: int,
    query_type: str,
    output_dir: Path,
    config,
    *,
    use_cache: bool = True,
    cache_tag: str | None = None,
) -> QueryResult:
    output_dir = Path(output_dir)
    projected = project_theta(theta, config)
    thash = theta_hash(projected)
    query_id = f"{thash}_{scenario.id}_{seed}"
    runtime_tag = _scenario_runtime_cache_tag(scenario)
    if runtime_tag:
        query_id = f"{query_id}_{runtime_tag}"
    if cache_tag:
        query_id = f"{query_id}_{cache_tag}"
    query_dir = output_dir / "queries" / query_id
    robustness_path = query_dir / "robustness.json"
    metadata_path = query_dir / "metadata.json"
    parsed_path = query_dir / "parsed_log.parquet"
    legacy_csv_path = query_dir / "parsed_log.csv"
    if use_cache and robustness_path.exists() and metadata_path.exists() and (parsed_path.exists() or legacy_csv_path.exists()):
        return QueryResult(
            query_id=query_id,
            theta_hash=thash,
            robustness=json.loads(robustness_path.read_text(encoding="utf-8")),
            parsed_log_path=parsed_path if parsed_path.exists() else legacy_csv_path,
            metadata=json.loads(metadata_path.read_text(encoding="utf-8")),
        )

    query_dir.mkdir(parents=True, exist_ok=True)
    groups = build_groups(config.input["horizon_s"], config.input["window_s"], config.input["channels"])
    sequence = theta_to_sequence(projected, groups, config)
    adapter = make_adapter(scenario.platform, config)
    query_start = time.monotonic()
    prepare_wall_time_s = 0.0
    run_wall_time_s = 0.0
    parse_wall_time_s = 0.0
    shutdown_wall_time_s = 0.0
    try:
        prepare_start = time.monotonic()
        adapter.prepare(scenario, seed)
        prepare_wall_time_s = time.monotonic() - prepare_start
        adapter_timing = getattr(adapter, "timing", {})
        run_start = time.monotonic()
        raw_log_path = adapter.run(sequence, scenario, query_dir)
        run_wall_time_s = time.monotonic() - run_start
        parse_start = time.monotonic()
        parsed_log = adapter.parse_log(raw_log_path)
        parsed_log = _augment_parsed_log_diagnostics(parsed_log)
        parse_wall_time_s = time.monotonic() - parse_start
    finally:
        shutdown_start = time.monotonic()
        adapter.shutdown()
        shutdown_wall_time_s = time.monotonic() - shutdown_start
    total_wall_time_s = time.monotonic() - query_start

    robustness = compute_all_properties(parsed_log, scenario.properties, config)
    np.save(query_dir / "input_theta.npy", projected)
    sequence.to_csv(query_dir / "input_sequence.csv", index=False)
    _write_parsed_log(parsed_log, parsed_path, legacy_csv_path)
    robustness_path.write_text(json.dumps(robustness, indent=2, sort_keys=True), encoding="utf-8")
    metadata = {
        "scenario_id": scenario.id,
        "seed": seed,
        "query_type": query_type,
        "theta_hash": thash,
        "cache_tag": cache_tag,
        "wall_time_s": run_wall_time_s,
        "prepare_wall_time_s": prepare_wall_time_s,
        "run_wall_time_s": run_wall_time_s,
        "parse_wall_time_s": parse_wall_time_s,
        "shutdown_wall_time_s": shutdown_wall_time_s,
        "total_wall_time_s": total_wall_time_s,
    }
    if getattr(scenario, "t_switch_s", None) is not None:
        metadata["t_switch_s"] = float(scenario.t_switch_s)
    metadata.update(_telemetry_diagnostics(parsed_log))
    for key, value in adapter_timing.items():
        metadata[f"adapter_{key}"] = value
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")
    _append_jsonl(output_dir, config, {"query_id": query_id, **metadata, "robustness": robustness})
    return QueryResult(query_id, thash, robustness, parsed_path, metadata)


def _scenario_runtime_cache_tag(scenario: ScenarioCfg) -> str | None:
    parts = []
    t_switch_s = getattr(scenario, "t_switch_s", None)
    if t_switch_s is not None:
        parts.append(f"ts{_cache_float_label(float(t_switch_s))}")
    overrides = dict(getattr(scenario, "param_overrides", {}) or {})
    if overrides:
        payload = json.dumps(
            {str(name): float(value) for name, value in sorted(overrides.items())},
            sort_keys=True,
            separators=(",", ":"),
        )
        parts.append(f"params{hashlib.sha256(payload.encode('utf-8')).hexdigest()[:10]}")
    return "_".join(parts) if parts else None


def _cache_float_label(value: float) -> str:
    return f"{float(value):.3f}".replace("-", "m").replace(".", "p")


def _augment_parsed_log_diagnostics(parsed_log: pd.DataFrame) -> pd.DataFrame:
    df = parsed_log.copy()
    if {"vx_mps", "vy_mps"}.issubset(df.columns):
        vx = pd.to_numeric(df["vx_mps"], errors="coerce")
        vy = pd.to_numeric(df["vy_mps"], errors="coerce")
        df["xy_speed_mps"] = np.sqrt(vx**2 + vy**2)
    if _has_transition_fields(df):
        df["velocity_at_transition_mps"] = _velocity_at_transition_mps(df)
    return df


def _telemetry_diagnostics(parsed_log: pd.DataFrame) -> dict[str, float]:
    diagnostics = {}
    if "xy_speed_mps" in parsed_log and "time_s" in parsed_log:
        for lo, hi in XY_SPEED_DIAGNOSTIC_WINDOWS_S:
            diagnostics[f"xy_speed_peak_{int(lo)}_{int(hi)}_mps"] = _xy_speed_peak(parsed_log, lo, hi)
    if _has_transition_fields(parsed_log):
        first_request_t_s = _first_finite(parsed_log, "transition_first_request_t_s")
        observed_t_s = _first_finite(parsed_log, "transition_observed_t_s")
        diagnostics.update(
            {
                "transition_t_switch_s": _first_finite(parsed_log, "transition_t_switch_s"),
                "transition_first_request_t_s": first_request_t_s,
                "transition_observed_t_s": observed_t_s,
                "transition_request_count": _first_finite(parsed_log, "transition_request_count"),
                "velocity_at_transition_mps": _velocity_at_transition_mps(parsed_log),
                "transition_request_to_observed_delay_s": (
                    observed_t_s - first_request_t_s
                    if math.isfinite(observed_t_s) and math.isfinite(first_request_t_s)
                    else math.nan
                ),
            }
        )
    return diagnostics


def _has_transition_fields(parsed_log: pd.DataFrame) -> bool:
    return any(
        name in parsed_log
        for name in ["transition_t_switch_s", "transition_first_request_t_s", "transition_observed_t_s"]
    )


def _velocity_at_transition_mps(parsed_log: pd.DataFrame) -> float:
    observed_t_s = _first_finite(parsed_log, "transition_observed_t_s")
    if not math.isfinite(observed_t_s) or "time_s" not in parsed_log:
        return math.nan
    if "xy_speed_mps" not in parsed_log and {"vx_mps", "vy_mps"}.issubset(parsed_log.columns):
        speed = np.sqrt(
            pd.to_numeric(parsed_log["vx_mps"], errors="coerce") ** 2
            + pd.to_numeric(parsed_log["vy_mps"], errors="coerce") ** 2
        )
    elif "xy_speed_mps" in parsed_log:
        speed = pd.to_numeric(parsed_log["xy_speed_mps"], errors="coerce")
    else:
        return math.nan
    times = pd.to_numeric(parsed_log["time_s"], errors="coerce")
    valid = times.notna() & speed.notna()
    if not bool(valid.any()):
        return math.nan
    idx = (times[valid] - observed_t_s).abs().idxmin()
    return float(speed.loc[idx])


def _xy_speed_peak(parsed_log: pd.DataFrame, lo: float, hi: float) -> float:
    times = pd.to_numeric(parsed_log["time_s"], errors="coerce")
    speed = pd.to_numeric(parsed_log["xy_speed_mps"], errors="coerce")
    mask = (times >= float(lo)) & (times <= float(hi)) & speed.notna()
    if not bool(mask.any()):
        return math.nan
    return float(speed.loc[mask].max())


def _first_finite(parsed_log: pd.DataFrame, column: str) -> float:
    if column not in parsed_log:
        return math.nan
    values = pd.to_numeric(parsed_log[column], errors="coerce").dropna()
    if values.empty:
        return math.nan
    return float(values.iloc[0])


def _append_jsonl(output_dir: Path, config, row: dict) -> None:
    jsonl = config.logging.get("jsonl")
    path = Path(jsonl) if jsonl else output_dir / "logs" / "queries.jsonl"
    if not path.is_absolute():
        path = Path.cwd() / path
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, sort_keys=True) + "\n")


def read_parsed_log(path: Path) -> pd.DataFrame:
    path = Path(path)
    if path.suffix == ".parquet":
        try:
            return pd.read_parquet(path)
        except Exception:
            csv_path = path.with_suffix(".csv")
            if csv_path.exists():
                return pd.read_csv(csv_path)
            raise
    return pd.read_csv(path)


def _write_parsed_log(parsed_log: pd.DataFrame, parquet_path: Path, csv_path: Path) -> None:
    try:
        parsed_log.to_parquet(parquet_path, index=False)
    except Exception:
        parsed_log.to_csv(csv_path, index=False)
