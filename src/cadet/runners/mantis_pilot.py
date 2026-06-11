from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from cadet.config import ExperimentConfig, ScenarioCfg, load_config
from cadet.mantis.classify import (
    BLOCKED_ENV,
    READY_NO_SITL,
    CandidateEvidence,
    classify_candidate,
    top_level_status,
)
from cadet.mantis.calibration import write_nonlinear_calibration
from cadet.mantis.contracts import (
    is_safe_contract,
    is_violation_like_contract,
    nonlinear_diagnostics,
    residual_rate_repeat_summary,
)
from cadet.mantis.maneuvers import ManeuverSpec, default_maneuvers, stress_metrics
from cadet.mantis.params import (
    build_param_candidates,
    default_readback_records,
    symbolic_candidate_records,
    target_param_names,
)
from cadet.query import read_parsed_log, run_query


CSV_FILES = {
    "explore": "mantis_explore_results.csv",
    "filter": "mantis_param_filter.csv",
    "boundary": "mantis_boundary_refinement.csv",
    "candidates": "mantis_candidates.csv",
    "confirmation": "mantis_confirmation.csv",
}

MODE_SUPPORT = {
    "px4": {"Position", "Hold", "POSCTL", "ACRO", "STABILIZED"},
    "ardupilot": {"Loiter", "AltHold", "STABILIZE"},
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Small MANTIS Go/No-Go pilot runner.")
    parser.add_argument("--config")
    parser.add_argument("--scenario")
    parser.add_argument("--axis", choices=["roll", "pitch"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--explore-repeats", type=int, default=1)
    parser.add_argument("--confirm-repeats", type=int, default=3)
    parser.add_argument("--max-param-candidates", type=int, default=8)
    parser.add_argument("--max-strong-maneuvers", type=int, default=8)
    parser.add_argument("--adaptive-boundary", action="store_true")
    parser.add_argument("--max-boundary-candidates", type=int, default=24)
    parser.add_argument("--boundary-source", choices=["previous", "fresh"], default="fresh")
    parser.add_argument("--previous-run-dir", default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--platform", choices=["px4", "ardupilot"], default=None)
    parser.add_argument("--include-ardupilot", action="store_true")
    parser.add_argument("--skip-sitl", action="store_true")
    parser.add_argument("--confirm-candidates", type=_bool_arg, default=True)
    parser.add_argument("--restart-on-mode-fail", action="store_true")
    parser.add_argument("--restart-each-query", action="store_true")
    parser.add_argument("--max-mode-switch-attempts", type=int, default=None)
    parser.add_argument("--backfill-nonlinear", action="store_true")
    args = parser.parse_args()

    started = time.monotonic()
    run_dir = Path(args.run_dir)
    reports_dir = run_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    if args.backfill_nonlinear:
        from cadet.mantis.rawlog_px4 import backfill_run_dir

        summary = backfill_run_dir(run_dir, active_axis=args.axis)
        summary["mode_trace_rows"] = _write_mode_trace_report(run_dir)
        _copy_compact_artifacts(run_dir)
        print(f"MANTIS_BACKFILL nonlinear={summary['diagnostics_csv']} inventory={summary['inventory_json']}", flush=True)
        return

    missing = [name for name in ["config", "scenario", "axis"] if getattr(args, name) in (None, "")]
    if missing:
        raise SystemExit(f"--{', --'.join(missing)} required unless --backfill-nonlinear is used")
    if args.adaptive_boundary and args.boundary_source == "previous" and not args.previous_run_dir:
        raise SystemExit("--previous-run-dir is required with --adaptive-boundary --boundary-source previous")

    config = _config_for_run_dir(load_config(args.config), run_dir)
    scenario = config.scenario_by_id(args.scenario)
    config = _config_for_cli(config, scenario.platform, args)
    scenario = config.scenario_by_id(args.scenario)
    if args.platform and scenario.platform != args.platform:
        raise ValueError(f"Scenario {scenario.id} is platform={scenario.platform}, not --platform {args.platform}")

    readiness = readiness_audit(config, scenario, args.axis)
    _write_json(reports_dir / "readiness.json", readiness)
    _write_readiness_report(reports_dir / "readiness_report.md", readiness)

    plan = build_plan(
        config,
        scenario,
        args.axis,
        args.max_param_candidates,
        args.max_strong_maneuvers,
        adaptive_boundary=bool(args.adaptive_boundary),
        max_boundary_candidates=int(args.max_boundary_candidates),
        boundary_source=str(args.boundary_source),
        previous_run_dir=args.previous_run_dir,
    )
    _write_json(reports_dir / "mantis_plan.json", plan)

    if args.audit_only:
        _write_empty_tables(reports_dir)
        _write_empty_aux_reports(reports_dir)
        summary = _summary(
            READY_NO_SITL,
            run_dir,
            started,
            query_count=0,
            readiness=readiness,
            note="audit_only",
        )
        summary.update(_planned_counts(plan))
        _write_json(reports_dir / "mantis_summary.json", summary)
        _write_report(reports_dir / "mantis_report.md", summary, [], [], [], [])
        _copy_compact_artifacts(run_dir)
        print(f"MANTIS_AUDIT status={summary['status']} readiness={reports_dir / 'readiness_report.md'}", flush=True)
        return

    if args.dry_run or args.skip_sitl:
        _write_empty_tables(reports_dir)
        _write_empty_aux_reports(reports_dir)
        status = READY_NO_SITL if args.dry_run or args.skip_sitl else BLOCKED_ENV
        summary = _summary(
            status,
            run_dir,
            started,
            query_count=0,
            readiness=readiness,
            note="dry_run" if args.dry_run else "skip_sitl",
        )
        summary.update(_planned_counts(plan))
        _write_json(reports_dir / "mantis_summary.json", summary)
        _write_report(reports_dir / "mantis_report.md", summary, [], [], [], [])
        _copy_compact_artifacts(run_dir)
        print(f"MANTIS_DRY_RUN status={summary['status']} plan={reports_dir / 'mantis_plan.json'}", flush=True)
        return

    blockers = list(readiness.get("blockers", []))
    if blockers:
        _write_empty_tables(reports_dir)
        _write_empty_aux_reports(reports_dir)
        summary = _summary(BLOCKED_ENV, run_dir, started, query_count=0, readiness=readiness, note="; ".join(blockers))
        _write_json(reports_dir / "mantis_summary.json", summary)
        _write_report(reports_dir / "mantis_report.md", summary, [], [], [], [])
        _copy_compact_artifacts(run_dir)
        print(f"MANTIS_BLOCKED reason={summary['note']} readiness={reports_dir / 'readiness_report.md'}", flush=True)
        return

    try:
        explore_rows, filter_rows, candidate_rows, confirmation_rows, query_count = run_real_pilot(
            config,
            scenario,
            args,
            run_dir,
            plan,
        )
        mode_trace_rows = _write_mode_trace_report(run_dir)
        nonlinear_summary = _backfill_nonlinear_reports(run_dir, args.axis)
        candidate_rows = _read_csv_rows(reports_dir / CSV_FILES["candidates"])
        confirmation_rows = _read_csv_rows(reports_dir / CSV_FILES["confirmation"])
        status = top_level_status(candidate_rows + confirmation_rows)
        summary = _summary(status, run_dir, started, query_count=query_count, readiness=readiness, note="")
        summary["mode_trace_rows"] = mode_trace_rows
        summary["nonlinear_backfill"] = nonlinear_summary
        summary.update(_count_summary(filter_rows, candidate_rows, confirmation_rows))
        _write_json(reports_dir / "mantis_summary.json", summary)
        _write_report(reports_dir / "mantis_report.md", summary, explore_rows, filter_rows, candidate_rows, confirmation_rows)
        _copy_compact_artifacts(run_dir)
        print(
            f"MANTIS_DONE status={summary['status']} queries={query_count} report={reports_dir / 'mantis_report.md'}",
            flush=True,
        )
    except Exception as exc:
        _ensure_tables_exist(reports_dir)
        mode_trace_rows = _write_mode_trace_report(run_dir)
        nonlinear_summary = _backfill_nonlinear_reports(run_dir, args.axis)
        explore_rows = _read_csv_rows(reports_dir / CSV_FILES["explore"])
        filter_rows = _read_csv_rows(reports_dir / CSV_FILES["filter"])
        candidate_rows = _read_csv_rows(reports_dir / CSV_FILES["candidates"])
        confirmation_rows = _read_csv_rows(reports_dir / CSV_FILES["confirmation"])
        summary = _summary(
            BLOCKED_ENV,
            run_dir,
            started,
            query_count=_query_jsonl_count(run_dir),
            readiness=readiness,
            note=str(exc),
        )
        summary["mode_trace_rows"] = mode_trace_rows
        summary["nonlinear_backfill"] = nonlinear_summary
        summary.update(_count_summary(filter_rows, candidate_rows, confirmation_rows))
        _write_json(reports_dir / "mantis_summary.json", summary)
        _write_report(reports_dir / "mantis_report.md", summary, explore_rows, filter_rows, candidate_rows, confirmation_rows)
        _copy_compact_artifacts(run_dir)
        print(f"MANTIS_BLOCKED reason={exc} report={reports_dir / 'mantis_report.md'}", flush=True)


def readiness_audit(config: ExperimentConfig, scenario: ScenarioCfg, axis: str) -> dict[str, Any]:
    platform = scenario.platform
    root = _sim_root(platform, config)
    modes_to_check = [scenario.perturb_mode, scenario.observe_mode, getattr(scenario, "test_mode", None)]
    if getattr(scenario, "staging_mode", None):
        modes_to_check.append(getattr(scenario, "staging_mode"))
    mode_supported = all(mode in MODE_SUPPORT.get(platform, set()) for mode in modes_to_check if mode)
    target_property = f"post_neutral_{axis}_rate"
    blockers = []
    if platform in {"px4", "ardupilot"} and not root.exists():
        env_name = "PX4_ROOT" if platform == "px4" else "AP_ROOT"
        blockers.append(f"{env_name}/configured simulator root does not exist: {root}")
    if not mode_supported:
        blockers.append(f"mode mapping unsupported for {platform}: {scenario.perturb_mode}->{scenario.observe_mode}")
    if target_property not in scenario.properties:
        blockers.append(f"target property {target_property} is not enabled on scenario {scenario.id}")
    return {
        "commit": _git_commit(),
        "config": str(config.path),
        "scenario_id": scenario.id,
        "platform": platform,
        "axis": axis,
        "perturb_mode": scenario.perturb_mode,
        "observe_mode": scenario.observe_mode,
        "staging_mode": getattr(scenario, "staging_mode", None) or "",
        "test_mode": getattr(scenario, "test_mode", None) or scenario.perturb_mode,
        "mode_mapping_ready": bool(mode_supported),
        "sim_root": str(root),
        "sim_root_exists": bool(root.exists()),
        "param_override_ready": platform in {"px4", "ardupilot"},
        "param_readback_ready": "requires_sitl",
        "telemetry_rate_columns_ready_by_code": True,
        "telemetry_rate_columns": ["roll_rate_rps", "pitch_rate_rps", "yaw_rate_rps", "roll_rad", "pitch_rad", "yaw_rad"],
        "nonlinear_observability": False,
        "nonlinear_observability_reason": "requires per-query raw ULog diagnostics; readiness is code/parser-only before SITL execution",
        "nonlinear_rawlog_parser_ready": _pyulog_available() if platform == "px4" else False,
        "blockers": blockers,
    }


def build_plan(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    axis: str,
    max_param_candidates: int,
    max_strong_maneuvers: int,
    *,
    adaptive_boundary: bool = False,
    max_boundary_candidates: int = 24,
    boundary_source: str = "fresh",
    previous_run_dir: str | None = None,
) -> dict[str, Any]:
    maneuvers = default_maneuvers(axis, max_strong=max_strong_maneuvers)
    candidate_limit = max_boundary_candidates if adaptive_boundary else max_param_candidates
    return {
        "scenario": {
            "id": scenario.id,
            "platform": scenario.platform,
            "perturb_mode": scenario.perturb_mode,
            "observe_mode": scenario.observe_mode,
            "staging_mode": getattr(scenario, "staging_mode", None),
            "test_mode": getattr(scenario, "test_mode", None) or scenario.perturb_mode,
            "takeoff_alt_m": scenario.takeoff_alt_m,
            "properties": list(scenario.properties),
        },
        "axis": axis,
        "target_property": f"post_neutral_{axis}_rate",
        "input": dict(config.input),
        "maneuvers": {key: [maneuver.to_record(config) for maneuver in values] for key, values in maneuvers.items()},
        "param_target_names": target_param_names(scenario.platform, axis),
        "param_candidate_specs": symbolic_candidate_records(
            scenario.platform,
            axis,
            candidate_limit,
            adaptive_boundary=adaptive_boundary,
        ),
        "adaptive_boundary": bool(adaptive_boundary),
        "boundary_source": boundary_source,
        "previous_run_dir": str(previous_run_dir or ""),
        "max_boundary_candidates": int(max_boundary_candidates),
        "max_stage_c_param_candidates": int(max_param_candidates),
        "acceptance_gate": "four_arm_differential_plus_nonlinear_activated",
        "chirp_policy": "scout_only_not_primary_bug_oracle",
    }


def run_real_pilot(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    plan: dict[str, Any],
) -> tuple[list[dict], list[dict], list[dict], list[dict], int]:
    reports_dir = run_dir / "reports"
    target_property = str(plan["target_property"])
    maneuvers = default_maneuvers(args.axis, max_strong=args.max_strong_maneuvers)
    query_count = 0

    target_names = target_param_names(scenario.platform, args.axis)
    previous_defaults: dict[str, float] = {}
    if args.adaptive_boundary and args.boundary_source == "previous":
        previous_defaults = _load_previous_param_defaults(Path(args.previous_run_dir), target_names)
        missing_previous = [name for name in target_names if name not in previous_defaults]
        if missing_previous:
            raise ValueError(
                "previous-run-dir is missing target parameter defaults: " + ",".join(sorted(missing_previous))
            )
    defaults, default_types, metadata = read_default_params(
        config,
        scenario,
        args.axis,
        args.seed,
        reset_overrides=previous_defaults,
    )
    baseline_overrides = {name: float(defaults[name]) for name in target_names if name in defaults}
    scenario_baseline = _scenario_with_param_overrides(scenario, baseline_overrides)
    default_rows = default_readback_records(defaults, default_types, metadata)
    pd.DataFrame(default_rows).to_csv(reports_dir / "mantis_param_defaults.csv", index=False)
    param_candidate_limit = int(args.max_boundary_candidates if args.adaptive_boundary else args.max_param_candidates)
    candidates, skipped = build_param_candidates(
        scenario.platform,
        args.axis,
        defaults,
        metadata=metadata,
        max_candidates=param_candidate_limit,
        adaptive_boundary=bool(args.adaptive_boundary),
    )
    _write_json(reports_dir / "mantis_skipped_param_candidates.json", skipped)

    explore_rows: list[dict[str, Any]] = []
    safe_strong: list[ManeuverSpec] = []
    default_strong_by_name: dict[str, dict[str, Any]] = {}
    for maneuver in [m for m in maneuvers["M_strong"] if not m.scout_only][: int(args.max_strong_maneuvers)]:
        summary, rows, _, _ = _eval_maneuver_repeats(
            maneuver,
            maneuver.to_theta(config),
            scenario_baseline,
            args.seed,
            int(args.explore_repeats),
            "mantis_stage_a_default_strong",
            run_dir,
            config,
            target_property,
            use_cache=True,
        )
        query_count += int(args.explore_repeats)
        row = {
            "stage": "A_default_strong_safety",
            "maneuver": maneuver.name,
            "param_candidate": "P0",
            "contract_class": summary.get("contract_class", ""),
            "default_strong_safe": is_safe_contract(summary),
            **_summary_columns(summary),
        }
        explore_rows.append(row)
        default_strong_by_name[maneuver.name] = row
        if bool(row["default_strong_safe"]):
            safe_strong.append(maneuver)
        pd.DataFrame(explore_rows).to_csv(reports_dir / CSV_FILES["explore"], index=False)
    if not safe_strong:
        pd.DataFrame([]).to_csv(reports_dir / CSV_FILES["filter"], index=False)
        pd.DataFrame([]).to_csv(reports_dir / CSV_FILES["boundary"], index=False)
        pd.DataFrame([]).to_csv(reports_dir / CSV_FILES["candidates"], index=False)
        pd.DataFrame([]).to_csv(reports_dir / CSV_FILES["confirmation"], index=False)
        return explore_rows, [], [], [], query_count

    filter_rows: list[dict[str, Any]] = []
    boundary_rows: list[dict[str, Any]] = []
    retained: list[tuple[Any, dict[str, Any], dict[str, Any]]] = []
    m0 = maneuvers["M0"][0]
    msmall = maneuvers["M_small"][0]
    for candidate in candidates:
        scenario_p = _scenario_with_param_overrides(scenario, baseline_overrides, candidate.overrides)
        m0_summary, _, _, _ = _eval_maneuver_repeats(
            m0,
            m0.to_theta(config),
            scenario_p,
            args.seed,
            int(args.explore_repeats),
            f"mantis_stage_b_m0_{candidate.label}",
            run_dir,
            config,
            target_property,
            use_cache=True,
        )
        small_summary, _, _, _ = _eval_maneuver_repeats(
            msmall,
            msmall.to_theta(config),
            scenario_p,
            args.seed,
            int(args.explore_repeats),
            f"mantis_stage_b_small_{candidate.label}",
            run_dir,
            config,
            target_property,
            use_cache=True,
        )
        query_count += 2 * int(args.explore_repeats)
        m0_safe = is_safe_contract(m0_summary)
        small_safe = is_safe_contract(small_summary)
        row = {
            "stage": "B_param_small_signal_filter",
            "param_candidate": candidate.label,
            "overrides_json": json.dumps(candidate.overrides, sort_keys=True),
            "m0_class": m0_summary.get("contract_class", ""),
            "msmall_class": small_summary.get("contract_class", ""),
            "m0_safe": m0_safe,
            "msmall_safe": small_safe,
            "retained": bool(m0_safe and small_safe),
            "candidate_status": "" if m0_safe and small_safe else "INVALID_PURE_PARAM",
            "boundary_margin": _boundary_margin(m0_summary, small_summary),
            **{f"m0_{k}": v for k, v in _summary_columns(m0_summary).items()},
            **{f"msmall_{k}": v for k, v in _summary_columns(small_summary).items()},
        }
        filter_rows.append(row)
        boundary_rows.append(
            _boundary_report_row(
                candidate.label,
                candidate.overrides,
                m0_summary,
                small_summary,
                retained_for_stage_c=False,
            )
        )
        if m0_safe and small_safe:
            retained.append((candidate, m0_summary, small_summary))
        pd.DataFrame(filter_rows).to_csv(reports_dir / CSV_FILES["filter"], index=False)
    retained.sort(key=lambda item: _boundary_score(item[1], item[2]), reverse=True)
    stage_c_labels = {item[0].label for item in retained[: int(args.max_param_candidates)]}
    for row in boundary_rows:
        row["retained_for_stage_c"] = row["candidate_id"] in stage_c_labels
        if row["retained_for_stage_c"]:
            row["rejection_reason"] = ""
    pd.DataFrame(boundary_rows).to_csv(reports_dir / CSV_FILES["boundary"], index=False)

    candidate_rows: list[dict[str, Any]] = []
    for candidate, m0_summary, small_summary in retained[: int(args.max_param_candidates)]:
        scenario_p = _scenario_with_param_overrides(scenario, baseline_overrides, candidate.overrides)
        for maneuver in safe_strong:
            summary, _, parsed_log, result = _eval_maneuver_repeats(
                maneuver,
                maneuver.to_theta(config),
                scenario_p,
                args.seed,
                int(args.explore_repeats),
                f"mantis_stage_c_{candidate.label}_{maneuver.name}",
                run_dir,
                config,
                target_property,
                use_cache=True,
            )
            query_count += int(args.explore_repeats)
            diag = nonlinear_diagnostics(parsed_log, _raw_log_path(result, scenario.platform)) if parsed_log is not None else {}
            evidence = CandidateEvidence(
                default_strong_safe=True,
                hover_safe=True,
                small_safe=True,
                strong_violation_like=is_violation_like_contract(summary),
                nonlinear_observable=bool(diag.get("nonlinear_observability", False)),
                nonlinear_activated=bool(diag.get("nonlinear_activated", False)),
                confirmed=False,
            )
            row = {
                "stage": "C_strong_search",
                "param_candidate": candidate.label,
                "overrides_json": json.dumps(candidate.overrides, sort_keys=True),
                "maneuver": maneuver.name,
                "small_safe": True,
                "strong_violation_like": is_violation_like_contract(summary),
                "candidate_status": classify_candidate(evidence),
                **_summary_columns(summary),
                **{f"diag_{k}": v for k, v in diag.items()},
            }
            candidate_rows.append(row)
            pd.DataFrame(candidate_rows).to_csv(reports_dir / CSV_FILES["candidates"], index=False)

    confirmation_rows: list[dict[str, Any]] = []
    if args.confirm_candidates:
        slivers = [row for row in candidate_rows if bool(row.get("strong_violation_like"))]
        if slivers:
            confirmation_rows, confirm_queries = _confirm_top_candidate(
                slivers[0],
                retained,
                safe_strong,
                maneuvers,
                scenario_baseline,
                args,
                run_dir,
                config,
                target_property,
                default_strong_by_name,
            )
            query_count += confirm_queries
    if confirmation_rows:
        pd.DataFrame(confirmation_rows).to_csv(reports_dir / CSV_FILES["confirmation"], index=False)
    else:
        pd.DataFrame(
            columns=["stage", "arm", "param_candidate", "maneuver", "contract_class", "candidate_status"]
        ).to_csv(reports_dir / CSV_FILES["confirmation"], index=False)
    return explore_rows, filter_rows, candidate_rows, confirmation_rows, query_count


def read_default_params(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    axis: str,
    seed: int,
    *,
    reset_overrides: dict[str, float] | None = None,
) -> tuple[dict[str, float], dict[str, int], dict[str, dict[str, Any]]]:
    from cadet.query import make_adapter

    adapter = make_adapter(scenario.platform, config)
    defaults: dict[str, float] = {}
    types: dict[str, int] = {}
    metadata: dict[str, dict[str, Any]] = {}
    try:
        adapter.prepare(replace(scenario, param_overrides=dict(reset_overrides or {})), seed)
        for name in target_param_names(scenario.platform, axis):
            value, param_type = adapter._read_param(name)
            defaults[name] = float(value)
            types[name] = int(param_type)
            if scenario.platform == "px4":
                try:
                    metadata[name] = dict(adapter._parameter_metadata(name))
                except Exception as exc:
                    metadata[name] = {"metadata_error": str(exc)}
    finally:
        adapter.shutdown()
    return defaults, types, metadata


def _scenario_with_param_overrides(
    scenario: ScenarioCfg,
    baseline_overrides: dict[str, float],
    candidate_overrides: dict[str, float] | None = None,
) -> ScenarioCfg:
    merged = dict(baseline_overrides)
    merged.update(dict(candidate_overrides or {}))
    return replace(scenario, param_overrides=merged)


def _load_previous_param_defaults(previous_run_dir: Path, target_names: list[str]) -> dict[str, float]:
    path = Path(previous_run_dir) / "reports" / "mantis_param_defaults.csv"
    try:
        rows = pd.read_csv(path).to_dict("records")
    except (FileNotFoundError, pd.errors.EmptyDataError) as exc:
        raise ValueError(f"Unable to read previous parameter defaults: {path}") from exc
    wanted = set(target_names)
    defaults: dict[str, float] = {}
    for row in rows:
        name = str(row.get("name", ""))
        if name not in wanted:
            continue
        try:
            defaults[name] = float(row["default_value"])
        except (TypeError, ValueError, KeyError) as exc:
            raise ValueError(f"Invalid previous default value for {name} in {path}") from exc
    return defaults


def _confirm_top_candidate(
    row: dict[str, Any],
    retained: list[tuple[Any, dict[str, Any], dict[str, Any]]],
    safe_strong: list[ManeuverSpec],
    maneuvers: dict[str, list[ManeuverSpec]],
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    config: ExperimentConfig,
    target_property: str,
    default_strong_by_name: dict[str, dict[str, Any]],
) -> tuple[list[dict[str, Any]], int]:
    candidate_label = str(row["param_candidate"])
    maneuver_name = str(row["maneuver"])
    candidate = next(item[0] for item in retained if item[0].label == candidate_label)
    maneuver = next(m for m in safe_strong if m.name == maneuver_name)
    baseline_overrides = dict(getattr(scenario, "param_overrides", {}) or {})
    scenario_p = _scenario_with_param_overrides(scenario, baseline_overrides, candidate.overrides)
    arms = [
        ("Mstrong_P0", scenario, maneuver),
        ("M0_P", scenario_p, maneuvers["M0"][0]),
        ("Msmall_P", scenario_p, maneuvers["M_small"][0]),
        ("Mstrong_P", scenario_p, maneuver),
    ]
    rows: list[dict[str, Any]] = []
    summaries: dict[str, dict[str, Any]] = {}
    diag = {}
    query_count = 0
    unique = str(time.time_ns())
    for arm, arm_scenario, arm_maneuver in arms:
        summary, _, parsed_log, result = _eval_maneuver_repeats(
            arm_maneuver,
            arm_maneuver.to_theta(config),
            arm_scenario,
            args.seed,
            int(args.confirm_repeats),
            f"mantis_stage_e_{candidate.label}_{arm}_{unique}",
            run_dir,
            config,
            target_property,
            use_cache=False,
        )
        query_count += int(args.confirm_repeats)
        summaries[arm] = summary
        if arm == "Mstrong_P" and parsed_log is not None and result is not None:
            diag = nonlinear_diagnostics(parsed_log, _raw_log_path(result, scenario.platform))
        rows.append(
            {
                "stage": "E_confirmation",
                "arm": arm,
                "param_candidate": candidate.label,
                "maneuver": arm_maneuver.name,
                "contract_class": summary.get("contract_class", ""),
                **_summary_columns(summary),
            }
        )
    evidence = CandidateEvidence(
        default_strong_safe=is_safe_contract(summaries["Mstrong_P0"]),
        hover_safe=is_safe_contract(summaries["M0_P"]),
        small_safe=is_safe_contract(summaries["Msmall_P"]),
        strong_violation_like=is_violation_like_contract(summaries["Mstrong_P"]),
        nonlinear_observable=bool(diag.get("nonlinear_observability", False)),
        nonlinear_activated=bool(diag.get("nonlinear_activated", False)),
        confirmed=True,
        repeated_noise_band=any(summary.get("contract_class") == "noise_band" for summary in summaries.values()),
    )
    status = classify_candidate(evidence)
    for arm_row in rows:
        arm_row["candidate_status"] = status
        for key, value in diag.items():
            arm_row[f"diag_{key}"] = value
        arm_row["default_stage_a_class"] = default_strong_by_name.get(maneuver_name, {}).get("contract_class", "")
    return rows, query_count


def _eval_maneuver_repeats(
    maneuver: ManeuverSpec,
    theta: np.ndarray,
    scenario: ScenarioCfg,
    seed: int,
    repeats: int,
    cache_prefix: str,
    run_dir: Path,
    config: ExperimentConfig,
    target_property: str,
    *,
    use_cache: bool,
) -> tuple[dict[str, Any], list[dict[str, Any]], pd.DataFrame | None, Any | None]:
    parsed_logs: list[pd.DataFrame] = []
    rows: list[dict[str, Any]] = []
    last_log: pd.DataFrame | None = None
    last_result = None
    for repeat in range(repeats):
        tag = _safe_label(f"{cache_prefix}_{maneuver.name}_repeat{repeat}")
        result = run_query(theta, scenario, seed, "mantis_pilot", run_dir, config, use_cache=use_cache, cache_tag=tag)
        parsed = read_parsed_log(result.parsed_log_path)
        parsed_logs.append(parsed)
        last_log = parsed
        last_result = result
        row = {
            "repeat": repeat,
            "query_id": result.query_id,
            "theta_hash": result.theta_hash,
            "cache_tag": tag,
            "maneuver": maneuver.name,
            "scenario_id": scenario.id,
            "param_override_count": len(dict(getattr(scenario, "param_overrides", {}) or {})),
            "robustness": result.robustness.get(target_property, math.nan),
        }
        row.update(stress_metrics(parsed, maneuver, config))
        rows.append(row)
    summary = residual_rate_repeat_summary(parsed_logs, target_property, config)
    return summary, rows, last_log, last_result


def _raw_log_path(result, platform: str) -> Path:
    query_dir = Path(result.parsed_log_path).parent
    return query_dir / ("raw_log.ulg" if platform == "px4" else "raw_log.BIN")


def _config_for_run_dir(config: ExperimentConfig, run_dir: Path) -> ExperimentConfig:
    logging = dict(config.logging)
    logging["jsonl"] = str(run_dir / "logs" / "queries.jsonl")
    return replace(config, logging=logging)


def _config_for_cli(config: ExperimentConfig, platform: str, args: argparse.Namespace) -> ExperimentConfig:
    simulator = {key: dict(value) if isinstance(value, dict) else value for key, value in dict(config.simulator).items()}
    platform_cfg = dict(simulator.get(platform, {}))
    if args.max_mode_switch_attempts is not None:
        platform_cfg["max_mode_switch_attempts"] = int(args.max_mode_switch_attempts)
    if args.restart_each_query or args.restart_on_mode_fail:
        platform_cfg["cleanup_each_run"] = True
    platform_cfg["restart_on_mode_fail"] = bool(args.restart_on_mode_fail)
    platform_cfg["restart_each_query"] = bool(args.restart_each_query)
    simulator[platform] = platform_cfg
    return replace(config, simulator=simulator)


def _sim_root(platform: str, config: ExperimentConfig) -> Path:
    if platform == "px4":
        return Path(os.environ.get("PX4_ROOT", config.simulator.get("px4", {}).get("root", "/home/car/PX4-Autopilot")))
    if platform == "ardupilot":
        return Path(os.environ.get("AP_ROOT", config.simulator.get("ardupilot", {}).get("root", "/home/car/ardupilot")))
    return Path(".")


def _summary(
    status: str,
    run_dir: Path,
    started: float,
    *,
    query_count: int,
    readiness: dict[str, Any],
    note: str,
) -> dict[str, Any]:
    return {
        "status": status,
        "run_dir": str(run_dir),
        "query_count": int(query_count),
        "elapsed_wall_time_s": time.monotonic() - started,
        "note": note,
        "readiness": readiness,
    }


def _count_summary(filter_rows: list[dict], candidate_rows: list[dict], confirmation_rows: list[dict]) -> dict[str, int | bool]:
    generated = len(filter_rows)
    rejected = sum(1 for row in filter_rows if row.get("candidate_status") == "INVALID_PURE_PARAM")
    retained = sum(1 for row in filter_rows if bool(row.get("retained")))
    strong_unsafe = sum(1 for row in candidate_rows if bool(row.get("strong_violation_like")))
    confirmed = sum(1 for row in confirmation_rows if row.get("arm") == "Mstrong_P")
    accepted = sum(
        1
        for row in confirmation_rows
        if row.get("arm") == "Mstrong_P" and row.get("candidate_status") == "ACCEPTED_MANTIS_BUG"
    )
    return {
        "param_candidates_generated": generated,
        "boundary_candidates_generated": generated,
        "pure_param_rejected": rejected,
        "small_safe_retained": retained,
        "strong_unsafe_candidates": strong_unsafe,
        "confirmed_candidates": confirmed,
        "accepted_candidates": accepted,
        "guided_modal_sliver_found": bool(strong_unsafe),
    }


def _planned_counts(plan: dict[str, Any]) -> dict[str, int | bool]:
    return {
        "param_candidates_generated": len(plan.get("param_candidate_specs", [])),
        "boundary_candidates_generated": len(plan.get("param_candidate_specs", [])),
        "pure_param_rejected": 0,
        "small_safe_retained": 0,
        "strong_unsafe_candidates": 0,
        "confirmed_candidates": 0,
        "accepted_candidates": 0,
        "guided_modal_sliver_found": False,
        "planned_strong_maneuvers": len([row for row in plan.get("maneuvers", {}).get("M_strong", []) if not row.get("scout_only")]),
    }


def _summary_columns(summary: dict[str, Any]) -> dict[str, Any]:
    keep = [
        "contract_class",
        "tier1_robustness_class",
        "tier2_robustness_class",
        "threshold_mean",
        "tail_start_peak_abs_rate_mean",
        "terminal_peak_abs_rate_mean",
        "rho_tier1_mean",
        "rho_tier1_std",
        "nondecay_ratio_margin_mean",
        "nondecay_slope_margin_mean",
        "terminal_peak_over_threshold",
        "terminal_over_start_peak",
    ]
    return {key: summary.get(key, math.nan) for key in keep}


def _boundary_margin(m0_summary: dict[str, Any], small_summary: dict[str, Any]) -> float:
    return max(
        float(m0_summary.get("terminal_peak_over_threshold", 0.0) or 0.0),
        float(small_summary.get("terminal_peak_over_threshold", 0.0) or 0.0),
    )


def _boundary_score(m0_summary: dict[str, Any], small_summary: dict[str, Any]) -> float:
    terminal_ratio = _boundary_margin(m0_summary, small_summary)
    nondecay_ratio = max(
        _finite_float(m0_summary.get("terminal_over_start_peak"), 0.0),
        _finite_float(small_summary.get("terminal_over_start_peak"), 0.0),
    )
    return float(terminal_ratio + 0.05 * min(max(nondecay_ratio, 0.0), 2.0))


def _boundary_report_row(
    candidate_id: str,
    overrides: dict[str, float],
    m0_summary: dict[str, Any],
    small_summary: dict[str, Any],
    *,
    retained_for_stage_c: bool,
) -> dict[str, Any]:
    m0_safe = _summary_is_safe(m0_summary)
    small_safe = _summary_is_safe(small_summary)
    if not m0_safe and not small_safe:
        rejection_reason = "m0_and_msmall_not_safe"
    elif not m0_safe:
        rejection_reason = "m0_not_safe"
    elif not small_safe:
        rejection_reason = "msmall_not_safe"
    elif not retained_for_stage_c:
        rejection_reason = "ranked_below_stage_c_limit"
    else:
        rejection_reason = ""
    return {
        "candidate_id": candidate_id,
        "overrides": json.dumps(overrides, sort_keys=True),
        "M0_status": m0_summary.get("contract_class", ""),
        "Msmall_status": small_summary.get("contract_class", ""),
        "terminal_peak_ratio": _boundary_margin(m0_summary, small_summary),
        "nondecay_ratio": max(
            _finite_float(m0_summary.get("terminal_over_start_peak"), 0.0),
            _finite_float(small_summary.get("terminal_over_start_peak"), 0.0),
        ),
        "boundary_score": _boundary_score(m0_summary, small_summary),
        "retained_for_stage_c": bool(retained_for_stage_c),
        "rejection_reason": rejection_reason,
    }


def _summary_is_safe(summary: dict[str, Any]) -> bool:
    contract_class = summary.get("contract_class")
    if contract_class in {"safe", "violation_like", "noise_band"}:
        return contract_class == "safe"
    return is_safe_contract(summary)


def _finite_float(value: Any, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _write_empty_tables(reports_dir: Path) -> None:
    headers = {
        "explore": ["stage", "maneuver", "param_candidate", "contract_class", "default_strong_safe"],
        "filter": ["stage", "param_candidate", "overrides_json", "m0_class", "msmall_class", "retained", "candidate_status"],
        "boundary": [
            "candidate_id",
            "overrides",
            "M0_status",
            "Msmall_status",
            "terminal_peak_ratio",
            "nondecay_ratio",
            "boundary_score",
            "retained_for_stage_c",
            "rejection_reason",
        ],
        "candidates": ["stage", "param_candidate", "overrides_json", "maneuver", "strong_violation_like", "candidate_status"],
        "confirmation": ["stage", "arm", "param_candidate", "maneuver", "contract_class", "candidate_status"],
    }
    for key, filename in CSV_FILES.items():
        pd.DataFrame(columns=headers[key]).to_csv(reports_dir / filename, index=False)


def _write_empty_aux_reports(reports_dir: Path) -> None:
    pd.DataFrame(
        columns=[
            "query_id",
            "trace_index",
            "scenario_id",
            "cache_tag",
            "failure_stage",
            "failure_reason",
            "target_test_mode",
            "staging_mode_used",
            "final_pre_maneuver_mode",
            "time_s",
            "requested_mode",
            "observed_mode_before",
            "ack_result",
            "ack",
            "observed_mode_after",
            "success",
            "request_count",
            "reason",
        ]
    ).to_csv(reports_dir / "mode_trace.csv", index=False)
    pd.DataFrame(
        columns=[
            "query_id",
            "raw_log_present",
            "raw_log_parser_status",
            "nonlinear_observability",
            "nonlinear_activated",
            "actuator_available",
            "actuator_sat_ratio",
            "actuator_sat_consecutive_s",
        ]
    ).to_csv(reports_dir / "mantis_nonlinear_diagnostics.csv", index=False)
    pd.DataFrame(
        columns=[
            "arm_type",
            "n",
            "nonlinear_observable_count",
            "nonlinear_activated_count",
            "nonlinear_activated_rate",
            "median_actuator_sat_ratio",
            "max_actuator_sat_ratio",
            "median_actuator_sat_consecutive_s",
            "max_actuator_sat_consecutive_s",
            "explicit_saturation_flag_active_count",
            "top_nonlinear_activation_reasons",
        ]
    ).to_csv(reports_dir / "mantis_nonlinear_calibration.csv", index=False)
    (reports_dir / "nonlinear_topics_inventory.json").write_text("{}\n", encoding="utf-8")


def _ensure_tables_exist(reports_dir: Path) -> None:
    missing = [filename for filename in CSV_FILES.values() if not (reports_dir / filename).exists()]
    if not missing:
        return
    headers = {
        "explore": ["stage", "maneuver", "param_candidate", "contract_class", "default_strong_safe"],
        "filter": ["stage", "param_candidate", "overrides_json", "m0_class", "msmall_class", "retained", "candidate_status"],
        "boundary": [
            "candidate_id",
            "overrides",
            "M0_status",
            "Msmall_status",
            "terminal_peak_ratio",
            "nondecay_ratio",
            "boundary_score",
            "retained_for_stage_c",
            "rejection_reason",
        ],
        "candidates": ["stage", "param_candidate", "overrides_json", "maneuver", "strong_violation_like", "candidate_status"],
        "confirmation": ["stage", "arm", "param_candidate", "maneuver", "contract_class", "candidate_status"],
    }
    for key, filename in CSV_FILES.items():
        path = reports_dir / filename
        if not path.exists():
            pd.DataFrame(columns=headers[key]).to_csv(path, index=False)
    if not (reports_dir / "mode_trace.csv").exists() or not (reports_dir / "mantis_nonlinear_diagnostics.csv").exists():
        _write_empty_aux_reports(reports_dir)
    elif not (reports_dir / "mantis_nonlinear_calibration.csv").exists():
        write_nonlinear_calibration(reports_dir)


def _read_csv_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    try:
        return pd.read_csv(path).to_dict("records")
    except pd.errors.EmptyDataError:
        return []


def _query_jsonl_count(run_dir: Path) -> int:
    path = run_dir / "logs" / "queries.jsonl"
    if not path.exists():
        return 0
    return sum(1 for line in path.read_text(encoding="utf-8").splitlines() if line.strip())


def _write_mode_trace_report(run_dir: Path) -> int:
    reports_dir = run_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for metadata_path in sorted((run_dir / "queries").glob("*/metadata.json")):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        trace = metadata.get("adapter_mode_trace", [])
        if isinstance(trace, str):
            try:
                trace = json.loads(trace)
            except json.JSONDecodeError:
                trace = []
        if not isinstance(trace, list):
            trace = []
        for idx, entry in enumerate(trace):
            if not isinstance(entry, dict):
                continue
            rows.append(
                {
                    "query_id": metadata_path.parent.name,
                    "trace_index": idx,
                    "scenario_id": metadata.get("scenario_id", ""),
                    "cache_tag": metadata.get("cache_tag", ""),
                    "failure_stage": metadata.get("failure_stage", ""),
                    "failure_reason": metadata.get("failure_reason", ""),
                    "target_test_mode": metadata.get("adapter_target_test_mode", ""),
                    "staging_mode_used": metadata.get("adapter_staging_mode_used", ""),
                    "final_pre_maneuver_mode": metadata.get("adapter_final_pre_maneuver_mode", ""),
                    **entry,
                }
            )
    columns = [
        "query_id",
        "trace_index",
        "scenario_id",
        "cache_tag",
        "failure_stage",
        "failure_reason",
        "target_test_mode",
        "staging_mode_used",
        "final_pre_maneuver_mode",
        "time_s",
        "requested_mode",
        "observed_mode_before",
        "ack_result",
        "ack",
        "observed_mode_after",
        "success",
        "request_count",
        "reason",
    ]
    pd.DataFrame(rows, columns=columns).to_csv(reports_dir / "mode_trace.csv", index=False)
    return len(rows)


def _backfill_nonlinear_reports(run_dir: Path, axis: str | None) -> dict[str, Any]:
    try:
        from cadet.mantis.rawlog_px4 import backfill_run_dir

        summary = backfill_run_dir(run_dir, active_axis=axis)
        if "calibration_csv" not in summary:
            summary["calibration_csv"] = str(write_nonlinear_calibration(run_dir / "reports"))
        return summary
    except Exception as exc:
        reports_dir = run_dir / "reports"
        reports_dir.mkdir(parents=True, exist_ok=True)
        path = reports_dir / "mantis_nonlinear_diagnostics.csv"
        pd.DataFrame(
            [
                {
                    "query_id": "",
                    "raw_log_parser_status": f"backfill_error:{exc}",
                    "nonlinear_observability": False,
                    "nonlinear_activated": False,
                }
            ]
        ).to_csv(path, index=False)
        inventory_path = reports_dir / "nonlinear_topics_inventory.json"
        inventory_path.write_text(json.dumps({"backfill_error": str(exc)}, indent=2, sort_keys=True), encoding="utf-8")
        calibration_path = write_nonlinear_calibration(reports_dir)
        return {
            "query_raw_logs_matched": 0,
            "diagnostics_csv": str(path),
            "calibration_csv": str(calibration_path),
            "inventory_json": str(inventory_path),
            "nonlinear_observability": False,
            "nonlinear_activated": False,
            "error": str(exc),
        }


def _write_report(
    path: Path,
    summary: dict[str, Any],
    explore_rows: list[dict],
    filter_rows: list[dict],
    candidate_rows: list[dict],
    confirmation_rows: list[dict],
) -> None:
    counts = _count_summary(filter_rows, candidate_rows, confirmation_rows)
    if not filter_rows and "param_candidates_generated" in summary:
        counts.update({key: summary[key] for key in counts if key in summary})
    lines = [
        "# MANTIS Pilot Report",
        "",
        f"Status: `{summary['status']}`",
        f"Run dir: `{summary['run_dir']}`",
        f"Queries: `{summary['query_count']}`",
        f"Elapsed wall time: `{summary['elapsed_wall_time_s']:.2f}s`",
        f"Note: `{summary.get('note', '')}`",
        "",
        "## Counts",
        "",
        f"- Parameter candidates generated: `{counts['param_candidates_generated']}`",
        f"- Rejected as pure parameter bad: `{counts['pure_param_rejected']}`",
        f"- Small-safe retained: `{counts['small_safe_retained']}`",
        f"- Strong-unsafe candidates: `{counts['strong_unsafe_candidates']}`",
        f"- Confirmed candidates: `{counts['confirmed_candidates']}`",
        f"- Accepted candidates: `{counts['accepted_candidates']}`",
        "",
        "## Evidence Tables",
        "",
        f"- `{path.parent / CSV_FILES['explore']}`",
        f"- `{path.parent / CSV_FILES['filter']}`",
        f"- `{path.parent / CSV_FILES['boundary']}`",
        f"- `{path.parent / CSV_FILES['candidates']}`",
        f"- `{path.parent / CSV_FILES['confirmation']}`",
        f"- `{path.parent / 'mode_trace.csv'}`",
        f"- `{path.parent / 'mantis_nonlinear_diagnostics.csv'}`",
        f"- `{path.parent / 'mantis_nonlinear_calibration.csv'}`",
        f"- `{path.parent / 'nonlinear_topics_inventory.json'}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_readiness_report(path: Path, readiness: dict[str, Any]) -> None:
    lines = [
        "# MANTIS Readiness",
        "",
        f"Commit: `{readiness['commit']}`",
        f"Scenario: `{readiness['scenario_id']}`",
        f"Platform: `{readiness['platform']}`",
        f"Axis: `{readiness['axis']}`",
        f"Staging mode: `{readiness.get('staging_mode', '')}`",
        f"Test mode: `{readiness.get('test_mode', '')}`",
        f"Mode mapping ready: `{readiness['mode_mapping_ready']}`",
        f"Simulator root: `{readiness['sim_root']}`",
        f"Simulator root exists: `{readiness['sim_root_exists']}`",
        f"Param override ready: `{readiness['param_override_ready']}`",
        f"Rate telemetry ready by code: `{readiness['telemetry_rate_columns_ready_by_code']}`",
        f"Nonlinear telemetry observable: `{readiness['nonlinear_observability']}`",
        f"Nonlinear rawlog parser ready: `{readiness.get('nonlinear_rawlog_parser_ready', False)}`",
        f"Nonlinear observability reason: {readiness['nonlinear_observability_reason']}",
        "",
        "## Blockers",
        "",
    ]
    blockers = readiness.get("blockers", [])
    if blockers:
        lines.extend(f"- {blocker}" for blocker in blockers)
    else:
        lines.append("- none")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def _git_commit() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def _pyulog_available() -> bool:
    try:
        import pyulog  # noqa: F401
    except Exception:
        return False
    return True


def _copy_compact_artifacts(run_dir: Path) -> None:
    reports_dir = run_dir / "reports"
    if not reports_dir.exists():
        return
    dest = Path("artifacts") / "mantis_pilot_v2" / run_dir.name / "reports"
    dest.mkdir(parents=True, exist_ok=True)
    allowed_suffixes = {".csv", ".json", ".md", ".txt"}
    blocked_suffixes = {".ulg", ".BIN", ".bin", ".parquet", ".npy"}
    for path in reports_dir.iterdir():
        if not path.is_file():
            continue
        if path.suffix in blocked_suffixes:
            continue
        if path.suffix not in allowed_suffixes:
            continue
        shutil.copy2(path, dest / path.name)


def _safe_label(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in value)


def _bool_arg(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "y"}:
        return True
    if lowered in {"0", "false", "no", "n"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected true/false, got {value}")


if __name__ == "__main__":
    main()
