from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import pandas as pd

from cadet.mantis.classify import CandidateEvidence, classify_candidate
from cadet.mantis.calibration import write_nonlinear_calibration
from cadet.mantis.nonlinear import analyze_nonlinear_topics, selected_diagnostic_topics, topic_inventory


def diagnose_px4_ulog(
    raw_log_path: Path,
    *,
    parsed_log: pd.DataFrame | None = None,
    active_axis: str = "roll",
) -> dict[str, Any]:
    diagnostics, _ = diagnose_px4_ulog_with_inventory(raw_log_path, parsed_log=parsed_log, active_axis=active_axis)
    return diagnostics


def diagnose_px4_ulog_with_inventory(
    raw_log_path: Path,
    *,
    parsed_log: pd.DataFrame | None = None,
    active_axis: str = "roll",
) -> tuple[dict[str, Any], dict[str, Any]]:
    raw_log_path = Path(raw_log_path)
    if not raw_log_path.exists():
        return _blocked_diag(raw_log_path, "raw_log_missing"), {"topics": [], "selected_topics": []}
    try:
        topics = load_ulog_topics(raw_log_path)
    except ImportError:
        return _blocked_diag(raw_log_path, "pyulog_unavailable"), {"topics": [], "selected_topics": []}
    except Exception as exc:
        return _blocked_diag(raw_log_path, f"ulog_parse_error:{exc}"), {"topics": [], "selected_topics": []}

    if parsed_log is not None:
        topics["parsed_mavlink"] = parsed_log
    selected = selected_diagnostic_topics(topics)
    inventory = topic_inventory(topics, selected)
    diag = analyze_nonlinear_topics(topics, active_axis=active_axis)
    observed_columns = _observed_columns(diag)
    activated_columns = _activated_columns(diag)
    diag.update(
        {
            "raw_log_present": True,
            "raw_log_path": str(raw_log_path),
            "raw_log_parser_status": "ok",
            "topic_inventory_status": "ok",
            "available_topics_count": len(topics),
            "selected_topics_count": sum(len(values) for values in selected.values()),
            "observed_nonlinear_columns": ",".join(observed_columns),
            "activated_nonlinear_columns": ",".join(activated_columns),
        }
    )
    return diag, inventory


def load_ulog_topics(raw_log_path: Path) -> dict[str, pd.DataFrame]:
    try:
        from pyulog import ULog
    except ImportError:
        raise

    ulog = ULog(str(raw_log_path))
    topics: dict[str, pd.DataFrame] = {}
    for dataset in ulog.data_list:
        name = str(getattr(dataset, "name", "unknown"))
        multi_id = int(getattr(dataset, "multi_id", 0) or 0)
        topic_name = name if multi_id == 0 else f"{name}#{multi_id}"
        data = getattr(dataset, "data", {})
        if not data:
            continue
        topics[topic_name] = pd.DataFrame({str(key): value for key, value in data.items()})
    return topics


def backfill_run_dir(run_dir: Path, *, active_axis: str | None = None) -> dict[str, Any]:
    from cadet.query import read_parsed_log

    run_dir = Path(run_dir)
    reports_dir = run_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    axis = active_axis or _axis_from_run_dir(run_dir)
    rows: list[dict[str, Any]] = []
    inventories: dict[str, Any] = {}
    matched = 0

    for query_dir in sorted((run_dir / "queries").glob("*")):
        if not query_dir.is_dir():
            continue
        raw_log = query_dir / "raw_log.ulg"
        if not raw_log.exists():
            continue
        matched += 1
        parsed_log = _read_query_parsed_log(query_dir, read_parsed_log)
        diag, inventory = diagnose_px4_ulog_with_inventory(raw_log, parsed_log=parsed_log, active_axis=axis)
        diag_path = query_dir / "nonlinear_diagnostics.json"
        diag_path.write_text(json.dumps(_jsonable(diag), indent=2, sort_keys=True), encoding="utf-8")
        inventories[query_dir.name] = inventory
        rows.append({"query_id": query_dir.name, **_flat_row(diag)})

    if not rows:
        rows.append(
            {
                "query_id": "",
                **_flat_row(
                    {
                        "raw_log_present": False,
                        "raw_log_parser_status": "no_matching_px4_ulog_logs_in_run_dir",
                        "nonlinear_observability": False,
                        "nonlinear_activated": False,
                    }
                ),
            }
        )

    diagnostics_csv = reports_dir / "mantis_nonlinear_diagnostics.csv"
    pd.DataFrame(rows).to_csv(diagnostics_csv, index=False)
    inventory_json = reports_dir / "nonlinear_topics_inventory.json"
    inventory_json.write_text(json.dumps(_jsonable(inventories), indent=2, sort_keys=True), encoding="utf-8")
    _update_candidate_tables(reports_dir, rows)
    calibration_csv = write_nonlinear_calibration(reports_dir)
    return {
        "run_dir": str(run_dir),
        "query_raw_logs_matched": matched,
        "diagnostics_csv": str(diagnostics_csv),
        "calibration_csv": str(calibration_csv),
        "inventory_json": str(inventory_json),
        "nonlinear_observability": any(_as_bool(row.get("nonlinear_observability")) for row in rows),
        "nonlinear_activated": any(_as_bool(row.get("nonlinear_activated")) for row in rows),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Backfill PX4 ULog nonlinear diagnostics for MANTIS runs.")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--axis", choices=["roll", "pitch"], default=None)
    args = parser.parse_args()
    summary = backfill_run_dir(Path(args.run_dir), active_axis=args.axis)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)


def _read_query_parsed_log(query_dir: Path, reader) -> pd.DataFrame | None:
    for name in ("parsed_log.parquet", "parsed_log.csv"):
        path = query_dir / name
        if path.exists():
            try:
                return reader(path)
            except Exception:
                return None
    return None


def _blocked_diag(raw_log_path: Path, status: str) -> dict[str, Any]:
    return {
        "raw_log_present": raw_log_path.exists(),
        "raw_log_path": str(raw_log_path),
        "raw_log_parser_status": status,
        "topic_inventory_status": status,
        "nonlinear_observability": False,
        "nonlinear_activated": False,
        "nonlinear_activation_reasons": [],
        "actuator_available": False,
        "actuator_sat_ratio": 0.0,
        "actuator_sat_consecutive_s": 0.0,
        "explicit_saturation_flag_available": False,
        "explicit_saturation_flag_active": False,
        "integrator_available": False,
        "limit_flag_available": False,
        "limit_flag_active": False,
        "cross_axis_energy_ratio": math.nan,
    }


def _observed_columns(diag: dict[str, Any]) -> list[str]:
    columns: list[str] = []
    for key in [
        "actuator_selected_columns",
        "explicit_saturation_flag_columns",
        "integrator_columns",
        "limit_flag_columns",
        "allocator_saturation_flag_columns",
    ]:
        value = diag.get(key, [])
        if isinstance(value, list):
            columns.extend(str(item) for item in value)
    return sorted(set(columns))


def _activated_columns(diag: dict[str, Any]) -> list[str]:
    columns: list[str] = []
    if _as_bool(diag.get("explicit_saturation_flag_active")):
        columns.extend(str(item) for item in diag.get("explicit_saturation_flag_columns", []))
    if _as_bool(diag.get("limit_flag_active")):
        columns.extend(str(item) for item in diag.get("limit_flag_columns", []))
    if _as_bool(diag.get("integrator_saturation_flag_active")):
        columns.extend(str(item) for item in diag.get("integrator_saturation_flag_columns", []))
    if _as_bool(diag.get("allocator_saturation_flag_active")):
        columns.extend(str(item) for item in diag.get("allocator_saturation_flag_columns", []))
    if "sustained_actuator_near_limit" in diag.get("nonlinear_activation_reasons", []):
        columns.extend(str(item) for item in diag.get("actuator_saturated_columns", []))
    return sorted(set(columns))


def _update_candidate_tables(reports_dir: Path, rows: list[dict[str, Any]]) -> None:
    by_raw_path = {str(row.get("raw_log_path", "")): row for row in rows if row.get("raw_log_path")}
    for filename in ("mantis_candidates.csv", "mantis_confirmation.csv"):
        path = reports_dir / filename
        if not path.exists() or not by_raw_path:
            continue
        try:
            df = pd.read_csv(path)
        except pd.errors.EmptyDataError:
            continue
        if df.empty or "diag_raw_log_path" not in df.columns:
            continue
        for idx, value in df["diag_raw_log_path"].items():
            row = by_raw_path.get(str(value))
            if row is None:
                continue
            for key, diag_value in row.items():
                if key == "query_id":
                    continue
                column = f"diag_{key}"
                if column not in df.columns:
                    df[column] = pd.Series(dtype="object")
                elif df[column].dtype != "object" and isinstance(diag_value, str):
                    df[column] = df[column].astype("object")
                df.loc[idx, column] = diag_value
            if filename == "mantis_candidates.csv" and _as_bool(df.loc[idx].get("strong_violation_like", False)):
                evidence = CandidateEvidence(
                    default_strong_safe=True,
                    hover_safe=True,
                    small_safe=_as_bool(df.loc[idx].get("small_safe", True)),
                    strong_violation_like=True,
                    nonlinear_observable=_as_bool(row.get("nonlinear_observability")),
                    nonlinear_activated=_as_bool(row.get("nonlinear_activated")),
                    confirmed=False,
                )
                df.loc[idx, "candidate_status"] = classify_candidate(evidence)
        df.to_csv(path, index=False)


def _flat_row(diag: dict[str, Any]) -> dict[str, Any]:
    return {key: _flat_value(value) for key, value in diag.items()}


def _flat_value(value: Any) -> Any:
    if isinstance(value, list):
        return ";".join(str(item) for item in value)
    if isinstance(value, dict):
        return json.dumps(_jsonable(value), sort_keys=True)
    if isinstance(value, float) and not math.isfinite(value):
        return math.nan
    return value


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_jsonable(item) for item in value]
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    try:
        if pd.isna(value):
            return None
    except Exception:
        pass
    if hasattr(value, "item"):
        return _jsonable(value.item())
    return str(value)


def _axis_from_run_dir(run_dir: Path) -> str:
    name = str(run_dir).lower()
    if "pitch" in name:
        return "pitch"
    return "roll"


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return bool(value) and not pd.isna(value)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y"}
    return bool(value)


if __name__ == "__main__":
    main()
