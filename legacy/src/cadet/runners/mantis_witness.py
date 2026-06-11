from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from cadet.config import ExperimentConfig, ScenarioCfg, load_config
from cadet.mantis.contracts import (
    is_safe_contract,
    is_violation_like_contract,
    nonlinear_diagnostics,
    residual_rate_repeat_summary,
)
from cadet.mantis.maneuvers import ManeuverSpec, default_maneuvers, stress_metrics
from cadet.mantis.params import ARDUPILOT_PARAMS, PX4_PARAMS, build_param_candidates, target_param_names
from cadet.mantis.tracking_contracts import (
    C_TRACK_BASELINE_RATIO,
    C_TRACK_HIGH_ERR_DURATION_THRESHOLD_S,
    C_TRACK_NTE_THRESHOLD,
    C_TRACK_OVERLAP_THRESHOLD_S,
    C_TRACK_PEAK_ERR_THRESHOLD_RADPS,
    evaluate_tracking_contract_from_raw,
    summarize_tracking_repeats,
)
from cadet.query import read_parsed_log, run_query
from cadet.runners.mantis_pilot import (
    _backfill_nonlinear_reports,
    _boundary_margin,
    _boundary_score,
    _config_for_cli,
    _config_for_run_dir,
    _copy_compact_artifacts,
    _finite_float,
    _git_commit,
    _raw_log_path,
    _safe_label,
    _summary_columns,
    _write_json,
    _write_mode_trace_report,
    readiness_audit,
    read_default_params,
)


ACCEPTED = "ACCEPTED_MANTIS_WITNESS"
CONDITIONAL = "CONDITIONAL_WITNESS_NOT_ACCEPTED"
NO_WITNESS = "NO_WITNESS_FOUND"
NO_WITNESS_HSTAR = "NO_WITNESS_FOUND_AT_HEADROOM_BOUNDARY"
BAD_HEADROOM = "BAD_HEADROOM"
INVALID_PURE_PARAM = "INVALID_PURE_PARAM"
INVALID_PURE_INPUT = "INVALID_PURE_INPUT"
BLOCKED_ENV = "BLOCKED_ENV"
READY_NO_SITL = "READY_NO_SITL"


@dataclass(frozen=True)
class HeadroomProfile:
    H_id: str
    description: str
    overrides: dict[str, float]
    changed_description: str


@dataclass
class EvalBundle:
    summary: dict[str, Any]
    repeat_rows: list[dict[str, Any]]
    parsed_logs: list[pd.DataFrame]
    results: list[Any]
    tracking_rows: list[dict[str, Any]]
    tracking_summary: dict[str, Any]
    nonlinear_rows: list[dict[str, Any]]


@dataclass
class BoundaryCandidate:
    candidate_id: str
    overrides: dict[str, float]
    support_size: int
    score: float
    row: dict[str, Any]


@dataclass(frozen=True)
class HstarAxisContext:
    axis: str
    scenario: ScenarioCfg
    target_property: str
    baseline_target: dict[str, float]
    metadata: dict[str, dict[str, Any]]
    m0: ManeuverSpec
    msmall: ManeuverSpec
    strong_maneuvers: list[ManeuverSpec]


class QueryBudget:
    def __init__(self, max_total: int):
        self.max_total = int(max_total)
        self.count = 0

    def reserve(self) -> None:
        if self.count >= self.max_total:
            raise RuntimeError(f"max_total_queries_exhausted:{self.max_total}")
        self.count += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Fast narrative-compliant MANTIS witness search.")
    parser.add_argument("--config", required=True)
    parser.add_argument("--platform", choices=["px4", "ardupilot"], default=None)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--axis", choices=["roll", "pitch"], required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--headroom-sweep", action="store_true")
    parser.add_argument("--headroom-boundary", action="store_true")
    parser.add_argument("--active-tracking-contract", action="store_true")
    parser.add_argument("--adaptive-boundary", action="store_true")
    parser.add_argument("--max-headroom-profiles", type=int, default=4)
    parser.add_argument("--max-headroom-evals", type=int, default=None)
    parser.add_argument("--max-boundary-candidates", type=int, default=20)
    parser.add_argument("--max-param-candidates", type=int, default=6)
    parser.add_argument("--max-strong-maneuvers", type=int, default=8)
    parser.add_argument("--confirm-repeats", type=int, default=3)
    parser.add_argument("--restart-each-query", action="store_true")
    parser.add_argument("--restart-on-mode-fail", action="store_true")
    parser.add_argument("--max-mode-switch-attempts", type=int, default=None)
    parser.add_argument("--max-total-queries", type=int, default=250)
    parser.add_argument("--stop-at-first-witness", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--reuse-best-headroom-from", default=None)
    args = parser.parse_args()

    started = time.monotonic()
    run_dir = Path(args.run_dir)
    reports_dir = run_dir / "reports"
    plots_dir = run_dir / "plots"
    reports_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)

    config = _config_for_run_dir(load_config(args.config), run_dir)
    scenario = config.scenario_by_id(args.scenario)
    config = _config_for_cli(config, scenario.platform, args)
    scenario = config.scenario_by_id(args.scenario)
    if args.platform and scenario.platform != args.platform:
        raise SystemExit(f"Scenario {scenario.id} is platform={scenario.platform}, not --platform {args.platform}")

    readiness = readiness_audit(config, scenario, args.axis)
    plan = _witness_plan(config, scenario, args, readiness)
    _write_json(reports_dir / "mantis_witness_plan.json", plan)

    if args.dry_run:
        _write_empty_reports(reports_dir)
        summary = _summary(
            READY_NO_SITL,
            run_dir,
            started,
            query_count=0,
            readiness=readiness,
            note="dry_run",
        )
        _write_json(reports_dir / "mantis_witness_summary.json", summary)
        _write_report(reports_dir / "mantis_witness_report.md", summary, plan, [], [], [], [], [])
        if args.headroom_boundary:
            hstar_summary = dict(summary)
            hstar_summary.update(_hstar_summary_defaults([], [], [], [], accepted=False))
            _write_hstar_reports(run_dir, reports_dir, hstar_summary, plan, [], [], [], [], [], [], [])
        print(f"MANTIS_WITNESS_DRY_RUN status={summary['status']} plan={reports_dir / 'mantis_witness_plan.json'}", flush=True)
        return

    try:
        result = run_witness_search(config, scenario, args, run_dir, plan, started, readiness)
    except Exception as exc:
        _ensure_empty_reports(reports_dir)
        mode_trace_rows = _write_mode_trace_report(run_dir)
        nonlinear_summary = _backfill_nonlinear_reports(run_dir, args.axis)
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
        _write_json(reports_dir / "mantis_witness_summary.json", summary)
        _write_report(
            reports_dir / "mantis_witness_report.md",
            summary,
            plan,
            _read_rows(reports_dir / "mantis_headroom_audit.csv"),
            _read_rows(reports_dir / "mantis_witness_boundary.csv"),
            _read_rows(reports_dir / "mantis_witness_candidates.csv"),
            _read_rows(reports_dir / "mantis_witness_confirmation.csv"),
            [],
        )
        print(f"MANTIS_WITNESS_BLOCKED reason={exc} report={reports_dir / 'mantis_witness_report.md'}", flush=True)
        return

    print(
        f"MANTIS_WITNESS_DONE status={result['status']} queries={result['query_count']} "
        f"report={reports_dir / 'mantis_witness_report.md'}",
        flush=True,
    )


def run_witness_search(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    plan: dict[str, Any],
    started: float,
    readiness: dict[str, Any],
) -> dict[str, Any]:
    reports_dir = run_dir / "reports"
    plots_dir = run_dir / "plots"
    budget = QueryBudget(args.max_total_queries)
    target_property = f"post_neutral_{args.axis}_rate"
    target_names = target_param_names(scenario.platform, args.axis)

    if bool(getattr(args, "headroom_boundary", False)):
        return run_hstar_headroom_boundary(config, scenario, args, run_dir, plan, started, readiness, budget)

    headroom_defaults, headroom_metadata = _read_headroom_defaults(config, scenario, args.seed)
    defaults, default_types, metadata = read_default_params(
        config,
        scenario,
        args.axis,
        args.seed,
        reset_overrides=headroom_defaults,
    )
    baseline_target = {name: float(defaults[name]) for name in target_names if name in defaults}
    pd.DataFrame(
        [
            {
                "name": name,
                "default_value": value,
                "readback_type": int(default_types.get(name, -1)),
                "min": metadata.get(name, {}).get("min", ""),
                "max": metadata.get(name, {}).get("max", ""),
                "rebootRequired": bool(metadata.get(name, {}).get("rebootRequired", False)),
            }
            for name, value in sorted(defaults.items())
        ]
    ).to_csv(reports_dir / "mantis_witness_param_defaults.csv", index=False)

    headroom_profiles = _headroom_profiles(headroom_defaults, headroom_metadata, int(args.max_headroom_profiles))
    if args.reuse_best_headroom_from:
        reused = _reuse_headroom(Path(args.reuse_best_headroom_from), headroom_defaults)
        if reused is not None:
            headroom_profiles = [reused]

    maneuvers = _witness_maneuvers(args.axis, int(args.max_strong_maneuvers), include_coupled=True)
    m0 = default_maneuvers(args.axis)["M0"][0]
    msmall = default_maneuvers(args.axis)["M_small"][0]
    tracking_rows: list[dict[str, Any]] = []
    nonlinear_rows: list[dict[str, Any]] = []

    headroom_rows, selected = _audit_headroom(
        config,
        scenario,
        args,
        run_dir,
        reports_dir,
        budget,
        baseline_target,
        headroom_defaults,
        headroom_profiles,
        maneuvers,
        m0,
        msmall,
        target_property,
        tracking_rows,
        nonlinear_rows,
    )
    pd.DataFrame(headroom_rows).to_csv(reports_dir / "mantis_headroom_audit.csv", index=False)
    if selected is None:
        summary = _summary(
            BAD_HEADROOM,
            run_dir,
            started,
            query_count=budget.count,
            readiness=readiness,
            note="no admissible low-headroom/default headroom profile",
        )
        _finalize_reports(run_dir, reports_dir, summary, plan, headroom_rows, [], [], [], tracking_rows, nonlinear_rows, [])
        return summary

    boundary_rows, retained = _boundary_search(
        config,
        scenario,
        args,
        run_dir,
        reports_dir,
        budget,
        selected,
        baseline_target,
        metadata,
        m0,
        msmall,
        target_property,
        tracking_rows,
        nonlinear_rows,
    )
    pd.DataFrame(boundary_rows).to_csv(reports_dir / "mantis_witness_boundary.csv", index=False)

    candidate_rows: list[dict[str, Any]] = []
    confirmation_rows: list[dict[str, Any]] = []
    plot_paths: list[str] = []
    accepted = False
    conditional_seen = False

    for candidate in retained[: int(args.max_param_candidates)]:
        scenario_p = _scenario_for(selected, scenario, baseline_target, candidate.overrides)
        for maneuver in selected["safe_strong_maneuvers"][: int(args.max_strong_maneuvers)]:
            baseline_nte = selected["strong_baseline_nte"].get(maneuver.name)
            bundle = _eval_repeats(
                maneuver,
                scenario_p,
                args.seed,
                1,
                f"witness_stage_c_{candidate.candidate_id}_{maneuver.name}",
                run_dir,
                config,
                target_property,
                args.axis,
                baseline_nte,
                budget,
                use_cache=False,
                retry_mode_failure=True,
            )
            tracking_rows.extend(bundle.tracking_rows)
            nonlinear_rows.extend(bundle.nonlinear_rows)
            recover_violation = is_violation_like_contract(bundle.summary)
            track_violation = bool(bundle.tracking_summary.get("C_track_violation_count", 0))
            nonlinear_observable = any(bool(row.get("nonlinear_observability", False)) for row in bundle.nonlinear_rows)
            nonlinear_activated = any(bool(row.get("nonlinear_activated", False)) for row in bundle.nonlinear_rows)
            track_overlap = _max_from_rows(bundle.tracking_rows, "saturation_error_overlap_s")
            violation_contract = _violation_contract(recover_violation, track_violation)
            status = NO_WITNESS
            if recover_violation or track_violation:
                status = CONDITIONAL
                conditional_seen = True
                if nonlinear_observable and (nonlinear_activated or track_overlap >= C_TRACK_OVERLAP_THRESHOLD_S):
                    status = "WITNESS_CANDIDATE"
            row = {
                "candidate_id": candidate.candidate_id,
                "H_id": selected["profile"].H_id,
                "param_overrides_json": json.dumps(candidate.overrides, sort_keys=True),
                "maneuver": maneuver.name,
                "C_recover_status": bundle.summary.get("contract_class", ""),
                "C_recover_violation": recover_violation,
                "C_track_status": bundle.tracking_summary.get("C_track_status", ""),
                "C_track_violation": track_violation,
                "violation_contract": violation_contract,
                "nonlinear_observable": nonlinear_observable,
                "nonlinear_activated": nonlinear_activated,
                "saturation_error_overlap_s": track_overlap,
                "candidate_status": status,
                **_summary_columns(bundle.summary),
            }
            candidate_rows.append(row)
            pd.DataFrame(candidate_rows).to_csv(reports_dir / "mantis_witness_candidates.csv", index=False)

            if recover_violation or track_violation:
                confirm, artifacts = _confirm_candidate(
                    config,
                    scenario,
                    args,
                    run_dir,
                    budget,
                    selected,
                    candidate,
                    maneuver,
                    m0,
                    msmall,
                    target_property,
                    baseline_nte,
                    tracking_rows,
                    nonlinear_rows,
                )
                confirmation_rows.extend(confirm)
                pd.DataFrame(confirmation_rows).to_csv(reports_dir / "mantis_witness_confirmation.csv", index=False)
                final_status = confirm[-1].get("candidate_status", CONDITIONAL) if confirm else CONDITIONAL
                if final_status == ACCEPTED:
                    accepted = True
                    plot_paths.extend(_write_candidate_plots(plots_dir, candidate.candidate_id, args.axis, artifacts, confirm))
                    if args.stop_at_first_witness:
                        break
                elif final_status == CONDITIONAL:
                    conditional_seen = True
                    plot_paths.extend(_write_candidate_plots(plots_dir, candidate.candidate_id, args.axis, artifacts, confirm))
        if accepted and args.stop_at_first_witness:
            break

    if not candidate_rows:
        pd.DataFrame(columns=_candidate_columns()).to_csv(reports_dir / "mantis_witness_candidates.csv", index=False)
    if not confirmation_rows:
        pd.DataFrame(columns=_confirmation_columns()).to_csv(reports_dir / "mantis_witness_confirmation.csv", index=False)

    status = ACCEPTED if accepted else (CONDITIONAL if conditional_seen else NO_WITNESS)
    note = _failed_condition(status, selected, retained, candidate_rows, confirmation_rows)
    summary = _summary(status, run_dir, started, query_count=budget.count, readiness=readiness, note=note)
    summary.update(
        {
            "selected_H_id": selected["profile"].H_id,
            "selected_H_description": selected["profile"].description,
            "boundary_candidates_generated": len(boundary_rows),
            "small_safe_retained": len(retained),
            "candidate_count": len(candidate_rows),
            "strong_unsafe_by_C_recover": sum(1 for row in candidate_rows if bool(row.get("C_recover_violation"))),
            "strong_unsafe_by_C_track": sum(1 for row in candidate_rows if bool(row.get("C_track_violation"))),
            "accepted_witness_count": 1 if accepted else 0,
        }
    )
    _finalize_reports(
        run_dir,
        reports_dir,
        summary,
        plan,
        headroom_rows,
        boundary_rows,
        candidate_rows,
        confirmation_rows,
        tracking_rows,
        nonlinear_rows,
        plot_paths,
    )
    return summary


def run_hstar_headroom_boundary(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    plan: dict[str, Any],
    started: float,
    readiness: dict[str, Any],
    budget: QueryBudget,
) -> dict[str, Any]:
    reports_dir = run_dir / "reports"
    plots_dir = run_dir / "plots"
    if scenario.platform != "px4":
        summary = _summary(BLOCKED_ENV, run_dir, started, query_count=budget.count, readiness=readiness, note="mode harness failed")
        _write_hstar_reports(run_dir, reports_dir, summary, plan, [], [], [], [], [], [], [])
        return summary

    headroom_defaults, headroom_metadata = _read_headroom_defaults(config, scenario, args.seed)
    if len(headroom_defaults) < 4:
        summary = _summary(BLOCKED_ENV, run_dir, started, query_count=budget.count, readiness=readiness, note="no admissible H*")
        _write_hstar_reports(run_dir, reports_dir, summary, plan, [], [], [], [], [], [], [])
        return summary

    contexts = _hstar_axis_contexts(config, scenario, args, run_dir, reports_dir, headroom_defaults)
    if not contexts:
        summary = _summary(BLOCKED_ENV, run_dir, started, query_count=budget.count, readiness=readiness, note="mode harness failed")
        _write_hstar_reports(run_dir, reports_dir, summary, plan, [], [], [], [], [], [], [])
        return summary

    tracking_rows: list[dict[str, Any]] = []
    nonlinear_rows: list[dict[str, Any]] = []
    headroom_rows, selected = _hstar_audit_headroom(
        config,
        args,
        run_dir,
        reports_dir,
        budget,
        headroom_defaults,
        headroom_metadata,
        contexts,
        tracking_rows,
        nonlinear_rows,
    )
    pd.DataFrame(headroom_rows).to_csv(reports_dir / "mantis_headroom_boundary.csv", index=False)
    if selected is None:
        summary = _summary(BAD_HEADROOM, run_dir, started, query_count=budget.count, readiness=readiness, note="no admissible H*")
        summary.update(_hstar_summary_defaults(headroom_rows, [], [], [], accepted=False))
        _write_hstar_reports(run_dir, reports_dir, summary, plan, headroom_rows, [], [], [], tracking_rows, nonlinear_rows, [])
        return summary

    boundary_rows, retained_by_axis = _hstar_boundary_search(
        config,
        args,
        run_dir,
        reports_dir,
        budget,
        selected,
        contexts,
        tracking_rows,
        nonlinear_rows,
    )
    pd.DataFrame(boundary_rows).to_csv(reports_dir / "mantis_Hstar_boundary_candidates.csv", index=False)

    retained = [candidate for candidates in retained_by_axis.values() for candidate in candidates]
    if not retained:
        summary = _summary(
            NO_WITNESS_HSTAR,
            run_dir,
            started,
            query_count=budget.count,
            readiness=readiness,
            note="no small-safe P",
        )
        summary.update(_hstar_summary_defaults(headroom_rows, boundary_rows, [], [], accepted=False))
        _write_hstar_reports(run_dir, reports_dir, summary, plan, headroom_rows, boundary_rows, [], [], tracking_rows, nonlinear_rows, [])
        return summary

    candidate_rows, confirmation_rows, plot_paths, accepted, conditional_seen = _hstar_witness_test(
        config,
        args,
        run_dir,
        plots_dir,
        budget,
        selected,
        contexts,
        retained_by_axis,
        tracking_rows,
        nonlinear_rows,
    )
    if accepted:
        status = ACCEPTED
    elif conditional_seen:
        status = CONDITIONAL
    else:
        status = NO_WITNESS_HSTAR
    note = _hstar_failed_condition(status, selected, retained, candidate_rows, confirmation_rows)
    summary = _summary(status, run_dir, started, query_count=budget.count, readiness=readiness, note=note)
    summary.update(_hstar_summary_defaults(headroom_rows, boundary_rows, candidate_rows, confirmation_rows, accepted=accepted))
    summary.update(
        {
            "selected_H_id": selected["profile"].H_id,
            "selected_H_description": selected["profile"].description,
            "selected_H_scale": selected["scale"],
            "default_safe_boundary_reached": bool(selected.get("default_safe_boundary_reached", False)),
            "first_bad_headroom_scale": selected.get("first_bad_scale", ""),
        }
    )
    _write_hstar_reports(
        run_dir,
        reports_dir,
        summary,
        plan,
        headroom_rows,
        boundary_rows,
        candidate_rows,
        confirmation_rows,
        tracking_rows,
        nonlinear_rows,
        plot_paths,
    )
    return summary


def _hstar_axis_contexts(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    headroom_defaults: dict[str, float],
) -> list[HstarAxisContext]:
    scenario_by_axis: dict[str, ScenarioCfg] = {args.axis: scenario}
    if scenario.platform == "px4":
        for axis, scenario_id in [("pitch", "px4_stabilized_pitch"), ("roll", "px4_acro_roll")]:
            if axis in scenario_by_axis:
                continue
            try:
                scenario_by_axis[axis] = config.scenario_by_id(scenario_id)
            except Exception:
                continue

    contexts: list[HstarAxisContext] = []
    for axis in ["pitch", "roll"]:
        axis_scenario = scenario_by_axis.get(axis)
        if axis_scenario is None or axis_scenario.platform != "px4":
            continue
        target_names = target_param_names(axis_scenario.platform, axis)
        defaults, default_types, metadata = read_default_params(
            config,
            axis_scenario,
            axis,
            args.seed,
            reset_overrides=headroom_defaults,
        )
        defaults = _hstar_registered_defaults(axis_scenario.platform, axis, defaults)
        pd.DataFrame(
            [
                {
                    "axis": axis,
                    "scenario_id": axis_scenario.id,
                    "name": name,
                    "default_value": value,
                    "readback_type": int(default_types.get(name, -1)),
                    "min": metadata.get(name, {}).get("min", ""),
                    "max": metadata.get(name, {}).get("max", ""),
                    "rebootRequired": bool(metadata.get(name, {}).get("rebootRequired", False)),
                }
                for name, value in sorted(defaults.items())
            ]
        ).to_csv(reports_dir / f"mantis_Hstar_param_defaults_{axis}.csv", index=False)
        if axis == args.axis:
            pd.DataFrame(
                [
                    {
                        "name": name,
                        "default_value": value,
                        "readback_type": int(default_types.get(name, -1)),
                        "min": metadata.get(name, {}).get("min", ""),
                        "max": metadata.get(name, {}).get("max", ""),
                        "rebootRequired": bool(metadata.get(name, {}).get("rebootRequired", False)),
                    }
                    for name, value in sorted(defaults.items())
                ]
            ).to_csv(reports_dir / "mantis_witness_param_defaults.csv", index=False)
        baseline_target = {name: float(defaults[name]) for name in target_names if name in defaults}
        contexts.append(
            HstarAxisContext(
                axis=axis,
                scenario=axis_scenario,
                target_property=f"post_neutral_{axis}_rate",
                baseline_target=baseline_target,
                metadata=metadata,
                m0=default_maneuvers(axis)["M0"][0],
                msmall=default_maneuvers(axis)["M_small"][0],
                strong_maneuvers=_selected_hstar_maneuvers(axis, axis_scenario.id, int(args.max_strong_maneuvers)),
            )
        )
    return contexts


def _hstar_registered_defaults(
    platform: str,
    axis: str,
    readback_defaults: dict[str, float],
) -> dict[str, float]:
    if platform != "px4":
        return dict(readback_defaults)
    defaults = dict(readback_defaults)
    if axis == "pitch":
        path = Path("runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_witness_param_defaults.csv")
    elif axis == "roll":
        path = Path("runs/mantis_witness_px4_roll_lowheadroom_seed0_v0/reports/mantis_witness_param_defaults.csv")
    else:
        return defaults
    try:
        rows = pd.read_csv(path).to_dict("records")
    except Exception:
        return defaults
    for row in rows:
        name = str(row.get("name", ""))
        if name in target_param_names(platform, axis):
            value = _maybe_float(row.get("default_value"))
            if value is not None:
                defaults[name] = float(value)
    return defaults


def _hstar_audit_headroom(
    config: ExperimentConfig,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    headroom_defaults: dict[str, float],
    headroom_metadata: dict[str, dict[str, Any]],
    contexts: list[HstarAxisContext],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    rows: list[dict[str, Any]] = []
    evaluated: dict[float, dict[str, Any]] = {}
    max_evals = getattr(args, "max_headroom_evals", None)
    max_evals = int(max_evals) if max_evals is not None else 8

    last_safe: dict[str, Any] | None = None
    first_bad_scale: float | None = None
    for scale in [0.65, 0.60, 0.55, 0.50]:
        if len(evaluated) >= max_evals:
            break
        item = _hstar_evaluate_headroom_scale(
            scale,
            config,
            args,
            run_dir,
            reports_dir,
            budget,
            headroom_defaults,
            headroom_metadata,
            contexts,
            tracking_rows,
            nonlinear_rows,
        )
        rows.append(item["row"])
        evaluated[scale] = item
        pd.DataFrame(rows).to_csv(reports_dir / "mantis_headroom_boundary.csv", index=False)
        if item["row"]["admissible"]:
            last_safe = item
            continue
        first_bad_scale = scale
        break

    boundary_reached = first_bad_scale is not None
    selected = last_safe
    if first_bad_scale is not None:
        safe_scale = float(last_safe["scale"]) if last_safe is not None else 0.70
        bad_scale = float(first_bad_scale)
        safe_item = last_safe
        for _ in range(3):
            if len(evaluated) >= max_evals:
                break
            mid = (safe_scale + bad_scale) / 2.0
            item = _hstar_evaluate_headroom_scale(
                mid,
                config,
                args,
                run_dir,
                reports_dir,
                budget,
                headroom_defaults,
                headroom_metadata,
                contexts,
                tracking_rows,
                nonlinear_rows,
            )
            rows.append(item["row"])
            evaluated[mid] = item
            pd.DataFrame(rows).to_csv(reports_dir / "mantis_headroom_boundary.csv", index=False)
            if item["row"]["admissible"]:
                safe_scale = mid
                safe_item = item
            else:
                bad_scale = mid
        if safe_item is None and 0.70 not in evaluated and len(evaluated) < max_evals:
            safe_item = _hstar_evaluate_headroom_scale(
                0.70,
                config,
                args,
                run_dir,
                reports_dir,
                budget,
                headroom_defaults,
                headroom_metadata,
                contexts,
                tracking_rows,
                nonlinear_rows,
            )
            rows.append(safe_item["row"])
            evaluated[0.70] = safe_item
        selected = safe_item

    if selected is not None:
        selected["default_safe_boundary_reached"] = boundary_reached
        selected["first_bad_scale"] = first_bad_scale if first_bad_scale is not None else ""
        for row in rows:
            row["selected_as_Hstar"] = abs(float(row["H_scale"]) - float(selected["scale"])) < 1e-9
    else:
        for row in rows:
            row["selected_as_Hstar"] = False
    return rows, selected


def _hstar_evaluate_headroom_scale(
    scale: float,
    config: ExperimentConfig,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    headroom_defaults: dict[str, float],
    headroom_metadata: dict[str, dict[str, Any]],
    contexts: list[HstarAxisContext],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    profile = _scaled_headroom_profile(scale, headroom_defaults, headroom_metadata)
    by_axis: dict[str, dict[str, Any]] = {}
    m0_recover: list[str] = []
    m0_track: list[str] = []
    small_recover: list[str] = []
    small_track: list[str] = []
    strong_recover_ratios: list[float] = []
    strong_track_ntes: list[float] = []
    strong_track_baseline_ratios: list[float] = []
    saturation_overlaps: list[float] = []
    nonlinear_observable = False
    nonlinear_activated_count = 0
    nonlinear_total = 0
    admissible = True
    rejection = ""

    for ctx in contexts:
        scenario_h = _scenario_for_profile(ctx.scenario, ctx.baseline_target, headroom_defaults, profile)
        strong_baseline_nte: dict[str, float] = {}
        safe_strong: list[ManeuverSpec] = []
        try:
            for arm_name, maneuver in [("M0", ctx.m0), ("Msmall", ctx.msmall)]:
                bundle = _eval_repeats(
                    maneuver,
                    scenario_h,
                    args.seed,
                    1,
                    f"Hboundary_{ctx.axis}_{profile.H_id}_{arm_name}",
                    run_dir,
                    config,
                    ctx.target_property,
                    ctx.axis,
                    None,
                    budget,
                    use_cache=False,
                    retry_mode_failure=True,
                )
                tracking_rows.extend(bundle.tracking_rows)
                nonlinear_rows.extend(bundle.nonlinear_rows)
                recover_status = str(bundle.summary.get("contract_class", ""))
                track_status = str(bundle.tracking_summary.get("C_track_status", ""))
                if arm_name == "M0":
                    m0_recover.append(recover_status)
                    m0_track.append(track_status)
                else:
                    small_recover.append(recover_status)
                    small_track.append(track_status)
                admissible = admissible and is_safe_contract(bundle.summary) and bool(bundle.tracking_summary.get("C_track_safe", True))
                nonlinear_observable = nonlinear_observable or any(
                    bool(row.get("nonlinear_observability", False)) for row in bundle.nonlinear_rows
                )
                nonlinear_activated_count += sum(1 for row in bundle.nonlinear_rows if bool(row.get("nonlinear_activated", False)))
                nonlinear_total += len(bundle.nonlinear_rows)
                saturation_overlaps.append(_max_from_rows(bundle.tracking_rows, "saturation_error_overlap_s"))

            for maneuver in ctx.strong_maneuvers:
                bundle = _eval_repeats(
                    maneuver,
                    scenario_h,
                    args.seed,
                    1,
                    f"Hboundary_{ctx.axis}_{profile.H_id}_Mstrong_P0",
                    run_dir,
                    config,
                    ctx.target_property,
                    ctx.axis,
                    None,
                    budget,
                    use_cache=False,
                    retry_mode_failure=True,
                )
                tracking_rows.extend(bundle.tracking_rows)
                nonlinear_rows.extend(bundle.nonlinear_rows)
                recover_safe = is_safe_contract(bundle.summary)
                track_safe = bool(bundle.tracking_summary.get("C_track_safe", True))
                admissible = admissible and recover_safe and track_safe
                strong_recover_ratios.append(_finite_float(bundle.summary.get("terminal_peak_over_threshold"), 0.0))
                if bundle.tracking_summary.get("C_track_available"):
                    nte = _finite_float(bundle.tracking_summary.get("median_nte"), math.nan)
                    strong_baseline_nte[maneuver.name] = nte
                    strong_track_ntes.append(_finite_float(bundle.tracking_summary.get("max_nte"), 0.0))
                strong_track_baseline_ratios.append(_max_track_baseline_ratio(bundle.tracking_rows))
                saturation_overlaps.append(_max_from_rows(bundle.tracking_rows, "saturation_error_overlap_s"))
                nonlinear_observable = nonlinear_observable or any(
                    bool(row.get("nonlinear_observability", False)) for row in bundle.nonlinear_rows
                )
                nonlinear_activated_count += sum(1 for row in bundle.nonlinear_rows if bool(row.get("nonlinear_activated", False)))
                nonlinear_total += len(bundle.nonlinear_rows)
                if recover_safe and track_safe:
                    safe_strong.append(maneuver)
            by_axis[ctx.axis] = {
                "context": ctx,
                "scenario_h": scenario_h,
                "safe_strong_maneuvers": safe_strong,
                "strong_baseline_nte": strong_baseline_nte,
            }
        except Exception as exc:
            admissible = False
            rejection = f"mode_harness_failed:{exc}"

    if admissible and not nonlinear_observable:
        admissible = False
        rejection = "nonlinear_not_observable"
    if admissible and any(len(by_axis.get(ctx.axis, {}).get("safe_strong_maneuvers", [])) < len(ctx.strong_maneuvers) for ctx in contexts):
        admissible = False
        rejection = "default_strong_gate_failed"
    if not admissible and not rejection:
        rejection = "BAD_HEADROOM"

    row = {
        "H_scale": float(scale),
        "admissible": bool(admissible),
        "rejection_reason": "" if admissible else rejection,
        "M0_C_recover_status": _worst_status(m0_recover),
        "M0_C_track_status": _worst_status(m0_track),
        "Msmall_C_recover_status": _worst_status(small_recover),
        "Msmall_C_track_status": _worst_status(small_track),
        "Mstrong_default_max_recover_ratio": max(strong_recover_ratios) if strong_recover_ratios else math.nan,
        "Mstrong_default_max_track_nte": max(strong_track_ntes) if strong_track_ntes else math.nan,
        "Mstrong_default_max_track_baseline_ratio": max(strong_track_baseline_ratios) if strong_track_baseline_ratios else math.nan,
        "nonlinear_observable": bool(nonlinear_observable),
        "nonlinear_activation_rate": float(nonlinear_activated_count / max(nonlinear_total, 1)),
        "max_saturation_overlap_s": max(saturation_overlaps) if saturation_overlaps else 0.0,
        "selected_as_Hstar": False,
    }
    return {"scale": float(scale), "profile": profile, "row": row, "by_axis": by_axis}


def _hstar_boundary_search(
    config: ExperimentConfig,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    contexts: list[HstarAxisContext],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, list[BoundaryCandidate]]]:
    rows: list[dict[str, Any]] = []
    retained_by_axis: dict[str, list[BoundaryCandidate]] = {}
    for ctx in contexts:
        retained: dict[str, BoundaryCandidate] = {}
        for family in _hstar_family_specs(ctx.axis):
            _hstar_search_family(
                family,
                ctx,
                config,
                args,
                run_dir,
                reports_dir,
                budget,
                selected,
                rows,
                retained,
                tracking_rows,
                nonlinear_rows,
            )
        retained_by_axis[ctx.axis] = sorted(retained.values(), key=lambda item: item.score, reverse=True)[:3]
    pd.DataFrame(rows).to_csv(reports_dir / "mantis_Hstar_boundary_candidates.csv", index=False)
    return rows, retained_by_axis


def _hstar_search_family(
    family: dict[str, Any],
    ctx: HstarAxisContext,
    config: ExperimentConfig,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    rows: list[dict[str, Any]],
    retained: dict[str, BoundaryCandidate],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> None:
    variable_role = str(family["variable_role"])
    target = float(family["target"])
    direction = str(family.get("direction", "high"))
    base = dict(family.get("base", {}))
    safe = 1.0
    bad: float | None = None

    ok = _hstar_evaluate_boundary_candidate(
        _hstar_family_label(family["label"], variable_role, target),
        {**base, variable_role: target},
        str(family["label"]),
        ctx,
        config,
        args,
        run_dir,
        reports_dir,
        budget,
        selected,
        rows,
        retained,
        tracking_rows,
        nonlinear_rows,
    )
    if ok:
        return
    bad = target
    for _ in range(3):
        if direction == "low":
            mid = (safe + bad) / 2.0
        else:
            mid = (safe + bad) / 2.0
        ok = _hstar_evaluate_boundary_candidate(
            _hstar_family_label(family["label"], variable_role, mid),
            {**base, variable_role: mid},
            str(family["label"]),
            ctx,
            config,
            args,
            run_dir,
            reports_dir,
            budget,
            selected,
            rows,
            retained,
            tracking_rows,
            nonlinear_rows,
        )
        if ok:
            safe = mid
        else:
            bad = mid


def _hstar_evaluate_boundary_candidate(
    candidate_id: str,
    multipliers: dict[str, float],
    family_label: str,
    ctx: HstarAxisContext,
    config: ExperimentConfig,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    rows: list[dict[str, Any]],
    retained: dict[str, BoundaryCandidate],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> bool:
    if candidate_id in {row.get("candidate_id") for row in rows}:
        return candidate_id in retained
    overrides, skipped_reason = _candidate_overrides(ctx.scenario.platform, ctx.axis, ctx.baseline_target, ctx.metadata, multipliers)
    if skipped_reason:
        row = _hstar_boundary_row(ctx, candidate_id, family_label, multipliers, overrides, skipped_reason=skipped_reason)
        rows.append(row)
        pd.DataFrame(rows).to_csv(reports_dir / "mantis_Hstar_boundary_candidates.csv", index=False)
        return False

    axis_selected = selected["by_axis"][ctx.axis]
    scenario_p = _scenario_with_hstar_candidate(axis_selected["scenario_h"], overrides)
    m0_bundle = _eval_repeats(
        ctx.m0,
        scenario_p,
        args.seed,
        1,
        f"Hstar_boundary_{ctx.axis}_{candidate_id}_M0",
        run_dir,
        config,
        ctx.target_property,
        ctx.axis,
        None,
        budget,
        use_cache=False,
        retry_mode_failure=True,
    )
    small_bundle = _eval_repeats(
        ctx.msmall,
        scenario_p,
        args.seed,
        1,
        f"Hstar_boundary_{ctx.axis}_{candidate_id}_Msmall",
        run_dir,
        config,
        ctx.target_property,
        ctx.axis,
        None,
        budget,
        use_cache=False,
        retry_mode_failure=True,
    )
    tracking_rows.extend(m0_bundle.tracking_rows + small_bundle.tracking_rows)
    nonlinear_rows.extend(m0_bundle.nonlinear_rows + small_bundle.nonlinear_rows)
    m0_safe = is_safe_contract(m0_bundle.summary) and bool(m0_bundle.tracking_summary.get("C_track_safe", True))
    small_safe = is_safe_contract(small_bundle.summary) and bool(small_bundle.tracking_summary.get("C_track_safe", True))
    retained_for_stage_c = bool(m0_safe and small_safe)
    rejection = ""
    if not m0_safe and not small_safe:
        rejection = "m0_and_msmall_not_safe"
    elif not m0_safe:
        rejection = "m0_not_safe"
    elif not small_safe:
        rejection = "msmall_not_safe"
    score = _hstar_candidate_score(m0_bundle, small_bundle, len(overrides))
    row = {
        "axis": ctx.axis,
        "scenario_id": ctx.scenario.id,
        "candidate_id": candidate_id,
        "family": family_label,
        "H_scale": selected["scale"],
        "multipliers_json": json.dumps(multipliers, sort_keys=True),
        "overrides_json": json.dumps(overrides, sort_keys=True),
        "M0_status": m0_bundle.summary.get("contract_class", ""),
        "Msmall_status": small_bundle.summary.get("contract_class", ""),
        "M0_C_track_status": m0_bundle.tracking_summary.get("C_track_status", ""),
        "Msmall_C_track_status": small_bundle.tracking_summary.get("C_track_status", ""),
        "C_recover_safe": is_safe_contract(m0_bundle.summary) and is_safe_contract(small_bundle.summary),
        "C_track_safe": bool(m0_bundle.tracking_summary.get("C_track_safe", True))
        and bool(small_bundle.tracking_summary.get("C_track_safe", True)),
        "retained_for_stage_c": retained_for_stage_c,
        "candidate_status": "" if retained_for_stage_c else INVALID_PURE_PARAM,
        "terminal_peak_ratio": _boundary_margin(m0_bundle.summary, small_bundle.summary),
        "max_tracking_nte_msmall": _finite_float(small_bundle.tracking_summary.get("max_nte"), math.nan),
        "nonlinear_overlap_msmall_s": _max_from_rows(small_bundle.tracking_rows, "saturation_error_overlap_s"),
        "nonlinear_observable": any(bool(row.get("nonlinear_observability", False)) for row in nonlinear_rows),
        "support_size": len(overrides),
        "boundary_score": score,
        "rejection_reason": rejection,
    }
    rows.append(row)
    pd.DataFrame(rows).to_csv(reports_dir / "mantis_Hstar_boundary_candidates.csv", index=False)
    if retained_for_stage_c:
        retained[candidate_id] = BoundaryCandidate(candidate_id, overrides, len(overrides), score, row)
    return retained_for_stage_c


def _hstar_witness_test(
    config: ExperimentConfig,
    args: argparse.Namespace,
    run_dir: Path,
    plots_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    contexts: list[HstarAxisContext],
    retained_by_axis: dict[str, list[BoundaryCandidate]],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[str], bool, bool]:
    reports_dir = run_dir / "reports"
    candidate_rows: list[dict[str, Any]] = []
    confirmation_rows: list[dict[str, Any]] = []
    plot_paths: list[str] = []
    accepted = False
    conditional_seen = False
    context_by_axis = {ctx.axis: ctx for ctx in contexts}

    for axis in ["pitch", "roll"]:
        ctx = context_by_axis.get(axis)
        if ctx is None:
            continue
        axis_selected = selected["by_axis"][axis]
        for candidate in retained_by_axis.get(axis, [])[:3]:
            scenario_p = _scenario_with_hstar_candidate(axis_selected["scenario_h"], candidate.overrides)
            for maneuver in axis_selected["safe_strong_maneuvers"][: int(args.max_strong_maneuvers)]:
                baseline_nte = axis_selected["strong_baseline_nte"].get(maneuver.name)
                bundle = _eval_repeats(
                    maneuver,
                    scenario_p,
                    args.seed,
                    1,
                    f"Hstar_witness_{axis}_{candidate.candidate_id}_{maneuver.name}",
                    run_dir,
                    config,
                    ctx.target_property,
                    axis,
                    baseline_nte,
                    budget,
                    use_cache=False,
                    retry_mode_failure=True,
                )
                tracking_rows.extend(bundle.tracking_rows)
                nonlinear_rows.extend(bundle.nonlinear_rows)
                recover_violation = is_violation_like_contract(bundle.summary)
                track_violation = bool(bundle.tracking_summary.get("C_track_violation_count", 0))
                nonlinear_observable = any(bool(row.get("nonlinear_observability", False)) for row in bundle.nonlinear_rows)
                nonlinear_activated = any(bool(row.get("nonlinear_activated", False)) for row in bundle.nonlinear_rows)
                track_overlap = _max_from_rows(bundle.tracking_rows, "saturation_error_overlap_s")
                status = NO_WITNESS_HSTAR
                if recover_violation or track_violation:
                    status = CONDITIONAL
                    conditional_seen = True
                    if nonlinear_observable and (nonlinear_activated or track_overlap >= C_TRACK_OVERLAP_THRESHOLD_S):
                        status = "WITNESS_CANDIDATE"
                row = {
                    "axis": axis,
                    "scenario_id": ctx.scenario.id,
                    "candidate_id": candidate.candidate_id,
                    "family": candidate.row.get("family", ""),
                    "H_scale": selected["scale"],
                    "param_overrides_json": json.dumps(candidate.overrides, sort_keys=True),
                    "maneuver": maneuver.name,
                    "C_recover_status": bundle.summary.get("contract_class", ""),
                    "C_recover_violation": recover_violation,
                    "C_track_status": bundle.tracking_summary.get("C_track_status", ""),
                    "C_track_violation": track_violation,
                    "violation_contract": _violation_contract(recover_violation, track_violation),
                    "nonlinear_observable": nonlinear_observable,
                    "nonlinear_activated": nonlinear_activated,
                    "saturation_error_overlap_s": track_overlap,
                    "candidate_status": status,
                    **_summary_columns(bundle.summary),
                }
                candidate_rows.append(row)
                pd.DataFrame(candidate_rows).to_csv(reports_dir / "mantis_Hstar_witness_candidates.csv", index=False)

                if recover_violation or track_violation:
                    confirm, artifacts = _hstar_confirm_candidate(
                        config,
                        args,
                        run_dir,
                        budget,
                        selected,
                        ctx,
                        candidate,
                        maneuver,
                        baseline_nte,
                        tracking_rows,
                        nonlinear_rows,
                    )
                    confirmation_rows.extend(confirm)
                    pd.DataFrame(confirmation_rows).to_csv(reports_dir / "mantis_Hstar_witness_confirmation.csv", index=False)
                    final_status = confirm[-1].get("candidate_status", CONDITIONAL) if confirm else CONDITIONAL
                    plot_paths.extend(
                        _write_candidate_plots(plots_dir, f"Hstar_{candidate.candidate_id}", axis, artifacts, confirm)
                    )
                    if final_status == ACCEPTED:
                        accepted = True
                        if args.stop_at_first_witness:
                            break
                    else:
                        conditional_seen = True
            if accepted and args.stop_at_first_witness:
                break
        if accepted and args.stop_at_first_witness:
            break

    if not candidate_rows:
        pd.DataFrame(columns=_hstar_candidate_columns()).to_csv(reports_dir / "mantis_Hstar_witness_candidates.csv", index=False)
    if not confirmation_rows:
        pd.DataFrame(columns=_hstar_confirmation_columns()).to_csv(reports_dir / "mantis_Hstar_witness_confirmation.csv", index=False)
    if candidate_rows:
        existing = set(plot_paths)
        for path in _write_hstar_top_candidate_plots_from_run(plots_dir, run_dir, selected, candidate_rows, tracking_rows):
            if path not in existing:
                plot_paths.append(path)
                existing.add(path)
    return candidate_rows, confirmation_rows, plot_paths, accepted, conditional_seen


def _hstar_confirm_candidate(
    config: ExperimentConfig,
    args: argparse.Namespace,
    run_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    ctx: HstarAxisContext,
    candidate: BoundaryCandidate,
    maneuver: ManeuverSpec,
    baseline_nte: float | None,
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, list[Any]]]:
    axis_selected = selected["by_axis"][ctx.axis]
    scenario_h = axis_selected["scenario_h"]
    scenario_p = _scenario_with_hstar_candidate(scenario_h, candidate.overrides)
    arms = [
        ("Mstrong_P0_H", scenario_h, maneuver, baseline_nte),
        ("M0_P_H", scenario_p, ctx.m0, None),
        ("Msmall_P_H", scenario_p, ctx.msmall, None),
        ("Mstrong_P_H", scenario_p, maneuver, baseline_nte),
    ]
    rows: list[dict[str, Any]] = []
    bundles: dict[str, EvalBundle] = {}
    unique = str(time.time_ns())
    for arm, arm_scenario, arm_maneuver, arm_baseline in arms:
        bundle = _eval_repeats(
            arm_maneuver,
            arm_scenario,
            args.seed,
            int(args.confirm_repeats),
            f"Hstar_confirm_{ctx.axis}_{candidate.candidate_id}_{arm}_{unique}",
            run_dir,
            config,
            ctx.target_property,
            ctx.axis,
            arm_baseline,
            budget,
            use_cache=False,
            retry_mode_failure=True,
        )
        bundles[arm] = bundle
        tracking_rows.extend(bundle.tracking_rows)
        nonlinear_rows.extend(bundle.nonlinear_rows)
        recover_count = sum(1 for row in bundle.repeat_rows if row.get("C_recover_repeat_status") == "violation_like")
        track_count = int(bundle.tracking_summary.get("C_track_violation_count", 0))
        rows.append(
            {
                "stage": "confirmation",
                "axis": ctx.axis,
                "scenario_id": ctx.scenario.id,
                "candidate_id": candidate.candidate_id,
                "H_scale": selected["scale"],
                "arm": arm,
                "maneuver": arm_maneuver.name,
                "C_recover_status": bundle.summary.get("contract_class", ""),
                "C_recover_violation_count": recover_count,
                "C_track_status": bundle.tracking_summary.get("C_track_status", ""),
                "C_track_violation_count": track_count,
                "nonlinear_observable_count": sum(
                    1 for row in bundle.nonlinear_rows if bool(row.get("nonlinear_observability", False))
                ),
                "nonlinear_activated_count": sum(1 for row in bundle.nonlinear_rows if bool(row.get("nonlinear_activated", False))),
                "candidate_status": "",
            }
        )

    control_safe = (
        is_safe_contract(bundles["Mstrong_P0_H"].summary)
        and is_safe_contract(bundles["M0_P_H"].summary)
        and is_safe_contract(bundles["Msmall_P_H"].summary)
        and not bundles["Mstrong_P0_H"].tracking_summary.get("C_track_violation_count", 0)
        and not bundles["M0_P_H"].tracking_summary.get("C_track_violation_count", 0)
        and not bundles["Msmall_P_H"].tracking_summary.get("C_track_violation_count", 0)
    )
    strong_recover_count = sum(
        1 for row in bundles["Mstrong_P_H"].repeat_rows if row.get("C_recover_repeat_status") == "violation_like"
    )
    strong_track_count = int(bundles["Mstrong_P_H"].tracking_summary.get("C_track_violation_count", 0))
    recover_repeat_robust = strong_recover_count >= 2
    track_repeat_robust = strong_track_count >= 2
    nonlinear_observable = any(bool(row.get("nonlinear_observability", False)) for row in bundles["Mstrong_P_H"].nonlinear_rows)
    nonlinear_activated = any(bool(row.get("nonlinear_activated", False)) for row in bundles["Mstrong_P_H"].nonlinear_rows)
    max_track_overlap = _max_from_rows(bundles["Mstrong_P_H"].tracking_rows, "saturation_error_overlap_s")
    accepted = bool(
        control_safe
        and (recover_repeat_robust or track_repeat_robust)
        and nonlinear_observable
        and (nonlinear_activated if recover_repeat_robust else True)
        and (max_track_overlap >= C_TRACK_OVERLAP_THRESHOLD_S if track_repeat_robust else True)
    )
    if accepted:
        status = ACCEPTED
        failed_condition = ""
    else:
        status = CONDITIONAL
        failed_condition = _hstar_confirmation_failed_condition(
            control_safe,
            recover_repeat_robust,
            track_repeat_robust,
            nonlinear_observable,
            nonlinear_activated,
            max_track_overlap,
        )
    for row in rows:
        row["candidate_status"] = status
        row["failed_condition"] = failed_condition
        row["strong_recover_violation_repeats"] = strong_recover_count
        row["strong_track_violation_repeats"] = strong_track_count
        row["max_saturation_error_overlap_s"] = max_track_overlap
    artifacts = {arm: bundles[arm].parsed_logs for arm in bundles}
    artifacts["tracking_rows"] = [row for bundle in bundles.values() for row in bundle.tracking_rows]
    return rows, artifacts


def _audit_headroom(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    baseline_target: dict[str, float],
    headroom_defaults: dict[str, float],
    profiles: list[HeadroomProfile],
    maneuvers: list[ManeuverSpec],
    m0: ManeuverSpec,
    msmall: ManeuverSpec,
    target_property: str,
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    rows: list[dict[str, Any]] = []
    admissible: list[dict[str, Any]] = []
    for profile in profiles:
        scenario_h = _scenario_for_profile(scenario, baseline_target, headroom_defaults, profile)
        profile_rows: list[dict[str, Any]] = []
        strong_baseline_nte: dict[str, float] = {}
        safe_strong: list[ManeuverSpec] = []
        all_safe = True
        rejection = ""
        nonlinear_activated = 0
        nonlinear_total = 0
        max_nte = 0.0
        try:
            for arm_name, maneuver in [("M0", m0), ("Msmall", msmall)]:
                bundle = _eval_repeats(
                    maneuver,
                    scenario_h,
                    args.seed,
                    1,
                    f"headroom_{profile.H_id}_{arm_name}",
                    run_dir,
                    config,
                    target_property,
                    args.axis,
                    None,
                    budget,
                    use_cache=False,
                    retry_mode_failure=True,
                )
                tracking_rows.extend(bundle.tracking_rows)
                nonlinear_rows.extend(bundle.nonlinear_rows)
                profile_rows.append(
                    {
                        "arm": arm_name,
                        "recover_safe": is_safe_contract(bundle.summary),
                        "track_safe": bool(bundle.tracking_summary.get("C_track_safe", True)),
                        "track_available": bool(bundle.tracking_summary.get("C_track_available", False)),
                    }
                )
                all_safe = all_safe and is_safe_contract(bundle.summary) and bool(bundle.tracking_summary.get("C_track_safe", True))
                if bundle.tracking_summary.get("C_track_available"):
                    max_nte = max(max_nte, _finite_float(bundle.tracking_summary.get("max_nte"), 0.0))
            for maneuver in maneuvers:
                bundle = _eval_repeats(
                    maneuver,
                    scenario_h,
                    args.seed,
                    1,
                    f"headroom_{profile.H_id}_Mstrong_P0",
                    run_dir,
                    config,
                    target_property,
                    args.axis,
                    None,
                    budget,
                    use_cache=False,
                    retry_mode_failure=True,
                )
                tracking_rows.extend(bundle.tracking_rows)
                nonlinear_rows.extend(bundle.nonlinear_rows)
                track_summary = bundle.tracking_summary
                if track_summary.get("C_track_available"):
                    strong_baseline_nte[maneuver.name] = _finite_float(track_summary.get("median_nte"), math.nan)
                    max_nte = max(max_nte, _finite_float(track_summary.get("max_nte"), 0.0))
                nonlinear_total += max(1, len(bundle.nonlinear_rows))
                nonlinear_activated += sum(1 for row in bundle.nonlinear_rows if bool(row.get("nonlinear_activated", False)))
                recover_safe = is_safe_contract(bundle.summary)
                track_safe = bool(track_summary.get("C_track_safe", True))
                all_safe = all_safe and recover_safe and track_safe
                if recover_safe and track_safe:
                    safe_strong.append(maneuver)
                profile_rows.append(
                    {
                        "arm": f"Mstrong_P0:{maneuver.name}",
                        "recover_safe": recover_safe,
                        "track_safe": track_safe,
                        "track_available": bool(track_summary.get("C_track_available", False)),
                    }
                )
        except Exception as exc:
            all_safe = False
            rejection = f"harness_or_query_failure:{exc}"

        if not rejection and not all_safe:
            failed = [row["arm"] for row in profile_rows if not bool(row["recover_safe"]) or not bool(row["track_safe"])]
            rejection = "default_gate_failed:" + ",".join(failed[:4])
        if not rejection and not safe_strong:
            rejection = "no_default_safe_strong_maneuver"
            all_safe = False
        activation_rate = nonlinear_activated / max(nonlinear_total, 1)
        row = {
            "H_id": profile.H_id,
            "H_description": profile.description,
            "changed_params_or_model_diff": profile.changed_description,
            "M0_status": _arm_status(profile_rows, "M0"),
            "Msmall_status": _arm_status(profile_rows, "Msmall"),
            "Mstrong_default_status": "safe" if safe_strong and all_safe else "not_safe",
            "C_recover_safe": all(row.get("recover_safe", False) for row in profile_rows) if profile_rows else False,
            "C_track_safe": all(row.get("track_safe", True) for row in profile_rows) if profile_rows else False,
            "nonlinear_observable": any(row.get("track_available", False) for row in profile_rows),
            "nonlinear_activation_rate": activation_rate,
            "max_tracking_nte_default": max_nte,
            "admissible": bool(all_safe),
            "rejection_reason": "" if all_safe else rejection,
        }
        rows.append(row)
        pd.DataFrame(rows).to_csv(reports_dir / "mantis_headroom_audit.csv", index=False)
        if all_safe:
            admissible.append(
                {
                    "profile": profile,
                    "safe_strong_maneuvers": safe_strong,
                    "strong_baseline_nte": strong_baseline_nte,
                    "activation_rate": activation_rate,
                    "max_nte": max_nte,
                    "scenario_h": scenario_h,
                }
            )

    if not admissible:
        return rows, None
    admissible.sort(key=lambda item: (item["activation_rate"], item["max_nte"], item["profile"].H_id != "H0"), reverse=True)
    return rows, admissible[0]


def _boundary_search(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    baseline_target: dict[str, float],
    metadata: dict[str, dict[str, Any]],
    m0: ManeuverSpec,
    msmall: ManeuverSpec,
    target_property: str,
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[BoundaryCandidate]]:
    rows: list[dict[str, Any]] = []
    retained: dict[str, BoundaryCandidate] = {}
    role_bounds: dict[str, float] = {}
    specs = [
        ("rate_p", 3.0),
        ("rate_i", 3.0),
        ("rate_d_high", 2.5),
        ("rate_d_low", 0.35),
        ("att_p", 2.0),
    ]
    for role, high_multiplier in specs:
        if len(rows) >= int(args.max_boundary_candidates):
            break
        safe_multiplier = _search_role_boundary(
            role,
            high_multiplier,
            config,
            scenario,
            args,
            run_dir,
            reports_dir,
            budget,
            selected,
            baseline_target,
            metadata,
            m0,
            msmall,
            target_property,
            rows,
            retained,
            tracking_rows,
            nonlinear_rows,
        )
        if safe_multiplier is not None:
            role_bounds[role] = safe_multiplier

    combo_specs = _combo_specs(role_bounds)
    for label, multipliers in combo_specs:
        if len(rows) >= int(args.max_boundary_candidates):
            break
        _evaluate_boundary_candidate(
            label,
            multipliers,
            config,
            scenario,
            args,
            run_dir,
            reports_dir,
            budget,
            selected,
            baseline_target,
            metadata,
            m0,
            msmall,
            target_property,
            rows,
            retained,
            tracking_rows,
            nonlinear_rows,
        )
    ranked = sorted(retained.values(), key=lambda item: item.score, reverse=True)
    return rows, ranked[: int(args.max_param_candidates)]


def _search_role_boundary(
    role: str,
    high_multiplier: float,
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    baseline_target: dict[str, float],
    metadata: dict[str, dict[str, Any]],
    m0: ManeuverSpec,
    msmall: ManeuverSpec,
    target_property: str,
    rows: list[dict[str, Any]],
    retained: dict[str, BoundaryCandidate],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> float | None:
    base_role = "rate_d" if role in {"rate_d_low", "rate_d_high"} else role
    lo = 1.0
    hi = float(high_multiplier)
    safe = 1.0
    first_bad: float | None = None
    candidates = [hi]
    for _ in range(3):
        if first_bad is None:
            break
        candidates.append((safe + first_bad) / 2.0)
    for multiplier in candidates:
        label = f"{role}_x{_mult_label(multiplier)}"
        ok = _evaluate_boundary_candidate(
            label,
            {base_role: multiplier},
            config,
            scenario,
            args,
            run_dir,
            reports_dir,
            budget,
            selected,
            baseline_target,
            metadata,
            m0,
            msmall,
            target_property,
            rows,
            retained,
            tracking_rows,
            nonlinear_rows,
        )
        if ok:
            safe = multiplier
        else:
            first_bad = multiplier
        if first_bad is None and not ok:
            first_bad = multiplier
        if first_bad is not None and len(candidates) == 1:
            for _ in range(3):
                mid = (safe + first_bad) / 2.0
                label = f"{role}_x{_mult_label(mid)}"
                ok = _evaluate_boundary_candidate(
                    label,
                    {base_role: mid},
                    config,
                    scenario,
                    args,
                    run_dir,
                    reports_dir,
                    budget,
                    selected,
                    baseline_target,
                    metadata,
                    m0,
                    msmall,
                    target_property,
                    rows,
                    retained,
                    tracking_rows,
                    nonlinear_rows,
                )
                if ok:
                    safe = mid
                else:
                    first_bad = mid
                if len(rows) >= int(args.max_boundary_candidates):
                    break
            break
    return safe if safe != 1.0 else None


def _evaluate_boundary_candidate(
    candidate_id: str,
    multipliers: dict[str, float],
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    reports_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    baseline_target: dict[str, float],
    metadata: dict[str, dict[str, Any]],
    m0: ManeuverSpec,
    msmall: ManeuverSpec,
    target_property: str,
    rows: list[dict[str, Any]],
    retained: dict[str, BoundaryCandidate],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> bool:
    if candidate_id in {row.get("candidate_id") for row in rows}:
        return candidate_id in retained
    overrides, skipped_reason = _candidate_overrides(scenario.platform, args.axis, baseline_target, metadata, multipliers)
    if skipped_reason:
        row = _boundary_row(candidate_id, overrides, skipped_reason=skipped_reason)
        rows.append(row)
        pd.DataFrame(rows).to_csv(reports_dir / "mantis_witness_boundary.csv", index=False)
        return False
    scenario_p = _scenario_for(selected, scenario, baseline_target, overrides)
    m0_bundle = _eval_repeats(
        m0,
        scenario_p,
        args.seed,
        1,
        f"boundary_{candidate_id}_M0",
        run_dir,
        config,
        target_property,
        args.axis,
        None,
        budget,
        use_cache=False,
        retry_mode_failure=True,
    )
    small_bundle = _eval_repeats(
        msmall,
        scenario_p,
        args.seed,
        1,
        f"boundary_{candidate_id}_Msmall",
        run_dir,
        config,
        target_property,
        args.axis,
        None,
        budget,
        use_cache=False,
        retry_mode_failure=True,
    )
    tracking_rows.extend(m0_bundle.tracking_rows + small_bundle.tracking_rows)
    nonlinear_rows.extend(m0_bundle.nonlinear_rows + small_bundle.nonlinear_rows)
    m0_safe = is_safe_contract(m0_bundle.summary) and bool(m0_bundle.tracking_summary.get("C_track_safe", True))
    small_safe = is_safe_contract(small_bundle.summary) and bool(small_bundle.tracking_summary.get("C_track_safe", True))
    retained_for_stage_c = bool(m0_safe and small_safe)
    if not m0_safe and not small_safe:
        rejection = "m0_and_msmall_not_safe"
    elif not m0_safe:
        rejection = "m0_not_safe"
    elif not small_safe:
        rejection = "msmall_not_safe"
    else:
        rejection = ""
    score = _boundary_score(m0_bundle.summary, small_bundle.summary)
    score += 0.2 * min(_finite_float(small_bundle.tracking_summary.get("max_nte"), 0.0), 5.0)
    score += 0.1 * min(_max_from_rows(small_bundle.tracking_rows, "saturation_error_overlap_s"), 2.0)
    support_size = len(overrides)
    row = {
        "candidate_id": candidate_id,
        "H_id": selected["profile"].H_id,
        "multipliers_json": json.dumps(multipliers, sort_keys=True),
        "overrides_json": json.dumps(overrides, sort_keys=True),
        "M0_status": m0_bundle.summary.get("contract_class", ""),
        "Msmall_status": small_bundle.summary.get("contract_class", ""),
        "M0_C_track_status": m0_bundle.tracking_summary.get("C_track_status", ""),
        "Msmall_C_track_status": small_bundle.tracking_summary.get("C_track_status", ""),
        "C_recover_safe": is_safe_contract(m0_bundle.summary) and is_safe_contract(small_bundle.summary),
        "C_track_safe": bool(m0_bundle.tracking_summary.get("C_track_safe", True))
        and bool(small_bundle.tracking_summary.get("C_track_safe", True)),
        "retained_for_stage_c": retained_for_stage_c,
        "candidate_status": "" if retained_for_stage_c else INVALID_PURE_PARAM,
        "terminal_peak_ratio": _boundary_margin(m0_bundle.summary, small_bundle.summary),
        "max_tracking_nte_msmall": _finite_float(small_bundle.tracking_summary.get("max_nte"), math.nan),
        "nonlinear_overlap_msmall_s": _max_from_rows(small_bundle.tracking_rows, "saturation_error_overlap_s"),
        "support_size": support_size,
        "boundary_score": score,
        "rejection_reason": rejection,
    }
    rows.append(row)
    pd.DataFrame(rows).to_csv(reports_dir / "mantis_witness_boundary.csv", index=False)
    if retained_for_stage_c:
        retained[candidate_id] = BoundaryCandidate(candidate_id, overrides, support_size, score, row)
    return retained_for_stage_c


def _confirm_candidate(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    args: argparse.Namespace,
    run_dir: Path,
    budget: QueryBudget,
    selected: dict[str, Any],
    candidate: BoundaryCandidate,
    maneuver: ManeuverSpec,
    m0: ManeuverSpec,
    msmall: ManeuverSpec,
    target_property: str,
    baseline_nte: float | None,
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, list[Any]]]:
    scenario_h = selected["scenario_h"]
    scenario_p = _scenario_for(selected, scenario, dict(getattr(scenario_h, "param_overrides", {}) or {}), candidate.overrides)
    arms = [
        ("Mstrong_P0_H", scenario_h, maneuver, baseline_nte),
        ("M0_P_H", scenario_p, m0, None),
        ("Msmall_P_H", scenario_p, msmall, None),
        ("Mstrong_P_H", scenario_p, maneuver, baseline_nte),
    ]
    rows: list[dict[str, Any]] = []
    bundles: dict[str, EvalBundle] = {}
    unique = str(time.time_ns())
    for arm, arm_scenario, arm_maneuver, arm_baseline in arms:
        bundle = _eval_repeats(
            arm_maneuver,
            arm_scenario,
            args.seed,
            int(args.confirm_repeats),
            f"confirm_{candidate.candidate_id}_{arm}_{unique}",
            run_dir,
            config,
            target_property,
            args.axis,
            arm_baseline,
            budget,
            use_cache=False,
            retry_mode_failure=True,
        )
        bundles[arm] = bundle
        tracking_rows.extend(bundle.tracking_rows)
        nonlinear_rows.extend(bundle.nonlinear_rows)
        recover_count = sum(1 for row in bundle.repeat_rows if row.get("C_recover_repeat_status") == "violation_like")
        track_count = int(bundle.tracking_summary.get("C_track_violation_count", 0))
        rows.append(
            {
                "stage": "confirmation",
                "candidate_id": candidate.candidate_id,
                "H_id": selected["profile"].H_id,
                "arm": arm,
                "maneuver": arm_maneuver.name,
                "C_recover_status": bundle.summary.get("contract_class", ""),
                "C_recover_violation_count": recover_count,
                "C_track_status": bundle.tracking_summary.get("C_track_status", ""),
                "C_track_violation_count": track_count,
                "nonlinear_observable_count": sum(1 for row in bundle.nonlinear_rows if bool(row.get("nonlinear_observability", False))),
                "nonlinear_activated_count": sum(1 for row in bundle.nonlinear_rows if bool(row.get("nonlinear_activated", False))),
                "candidate_status": "",
            }
        )

    control_safe = (
        is_safe_contract(bundles["Mstrong_P0_H"].summary)
        and is_safe_contract(bundles["M0_P_H"].summary)
        and is_safe_contract(bundles["Msmall_P_H"].summary)
        and not bundles["Mstrong_P0_H"].tracking_summary.get("C_track_violation_count", 0)
        and not bundles["M0_P_H"].tracking_summary.get("C_track_violation_count", 0)
        and not bundles["Msmall_P_H"].tracking_summary.get("C_track_violation_count", 0)
    )
    strong_recover_count = sum(
        1 for row in bundles["Mstrong_P_H"].repeat_rows if row.get("C_recover_repeat_status") == "violation_like"
    )
    strong_track_count = int(bundles["Mstrong_P_H"].tracking_summary.get("C_track_violation_count", 0))
    recover_repeat_robust = strong_recover_count >= 2
    track_repeat_robust = strong_track_count >= 2
    nonlinear_observable = any(bool(row.get("nonlinear_observability", False)) for row in bundles["Mstrong_P_H"].nonlinear_rows)
    nonlinear_activated = any(bool(row.get("nonlinear_activated", False)) for row in bundles["Mstrong_P_H"].nonlinear_rows)
    max_track_overlap = _max_from_rows(bundles["Mstrong_P_H"].tracking_rows, "saturation_error_overlap_s")
    accepted = bool(
        control_safe
        and (recover_repeat_robust or track_repeat_robust)
        and nonlinear_observable
        and (nonlinear_activated if recover_repeat_robust else True)
        and (max_track_overlap >= C_TRACK_OVERLAP_THRESHOLD_S if track_repeat_robust else True)
    )
    if accepted:
        status = ACCEPTED
        failed_condition = ""
    else:
        status = CONDITIONAL
        failed_condition = _confirmation_failed_condition(
            control_safe,
            recover_repeat_robust,
            track_repeat_robust,
            nonlinear_observable,
            nonlinear_activated,
            max_track_overlap,
        )
    for row in rows:
        row["candidate_status"] = status
        row["failed_condition"] = failed_condition
        row["strong_recover_violation_repeats"] = strong_recover_count
        row["strong_track_violation_repeats"] = strong_track_count
        row["max_saturation_error_overlap_s"] = max_track_overlap
    artifacts = {arm: bundles[arm].parsed_logs for arm in bundles}
    artifacts["tracking_rows"] = [row for bundle in bundles.values() for row in bundle.tracking_rows]
    return rows, artifacts


def _eval_repeats(
    maneuver: ManeuverSpec,
    scenario: ScenarioCfg,
    seed: int,
    repeats: int,
    cache_prefix: str,
    run_dir: Path,
    config: ExperimentConfig,
    target_property: str,
    axis: str,
    baseline_nte: float | None,
    budget: QueryBudget,
    *,
    use_cache: bool,
    retry_mode_failure: bool,
) -> EvalBundle:
    parsed_logs: list[pd.DataFrame] = []
    results: list[Any] = []
    rows: list[dict[str, Any]] = []
    tracking_rows: list[dict[str, Any]] = []
    nonlinear_rows: list[dict[str, Any]] = []
    theta = maneuver.to_theta(config)
    for repeat in range(int(repeats)):
        tag = _safe_label(f"{cache_prefix}_{maneuver.name}_repeat{repeat}")
        result = _run_query_counted(
            theta,
            scenario,
            seed,
            "mantis_witness",
            run_dir,
            config,
            use_cache=use_cache,
            cache_tag=tag,
            budget=budget,
            retry_mode_failure=retry_mode_failure,
        )
        parsed = read_parsed_log(result.parsed_log_path)
        parsed_logs.append(parsed)
        results.append(result)
        repeat_summary = residual_rate_repeat_summary([parsed], target_property, config)
        raw_path = _raw_log_path(result, scenario.platform)
        track = evaluate_tracking_contract_from_raw(raw_path, parsed_log=parsed, axis=axis, baseline_nte_median=baseline_nte)
        diag = nonlinear_diagnostics(parsed, raw_path)
        row = {
            "repeat": repeat,
            "query_id": result.query_id,
            "cache_tag": tag,
            "maneuver": maneuver.name,
            "scenario_id": scenario.id,
            "param_override_count": len(dict(getattr(scenario, "param_overrides", {}) or {})),
            "C_recover_repeat_status": repeat_summary.get("contract_class", ""),
            "robustness": result.robustness.get(target_property, math.nan),
            **stress_metrics(parsed, maneuver, config),
        }
        rows.append(row)
        tracking_rows.append(
            {
                "query_id": result.query_id,
                "cache_tag": tag,
                "maneuver": maneuver.name,
                "axis": axis,
                **_flat_row(track),
            }
        )
        nonlinear_rows.append({"query_id": result.query_id, "cache_tag": tag, "maneuver": maneuver.name, **_flat_row(diag)})
    summary = residual_rate_repeat_summary(parsed_logs, target_property, config)
    tracking_summary = summarize_tracking_repeats(tracking_rows)
    return EvalBundle(summary, rows, parsed_logs, results, tracking_rows, tracking_summary, nonlinear_rows)


def _run_query_counted(
    theta: np.ndarray,
    scenario: ScenarioCfg,
    seed: int,
    query_type: str,
    run_dir: Path,
    config: ExperimentConfig,
    *,
    use_cache: bool,
    cache_tag: str,
    budget: QueryBudget,
    retry_mode_failure: bool,
) -> Any:
    budget.reserve()
    try:
        return run_query(theta, scenario, seed, query_type, run_dir, config, use_cache=use_cache, cache_tag=cache_tag)
    except Exception as exc:
        if not retry_mode_failure or not _is_mode_failure(exc):
            raise
        _kill_sim()
        budget.reserve()
        retry_tag = f"{cache_tag}_mode_retry1"
        return run_query(theta, scenario, seed, query_type, run_dir, config, use_cache=False, cache_tag=retry_tag)


def _witness_maneuvers(axis: str, max_strong: int, *, include_coupled: bool) -> list[ManeuverSpec]:
    maneuvers = [
        ManeuverSpec("strong_step_A0p9_hold2", "M_strong", axis, "step", 0.9, 2),
        ManeuverSpec("strong_doublet_A0p9_hold2", "M_strong", axis, "doublet", 0.9, 2),
        ManeuverSpec("pulse_train_A0p9_repeat3", "M_strong", axis, "pulse_train", 0.9, 1, 3),
        ManeuverSpec("reversal_A0p9_fast", "M_strong", axis, "reversal_fast", 0.9, 1, 1),
        ManeuverSpec("pulse_train_A1p0_repeat3", "M_strong", axis, "pulse_train", 1.0, 1, 3),
        ManeuverSpec("strong_doublet_A0p7_hold1", "M_strong", axis, "doublet", 0.7, 1),
        ManeuverSpec(f"strong_step_{axis}_0p7", "M_strong", axis, "step", 0.7, 1),
        ManeuverSpec("pulse_train_A0p7_repeat3", "M_strong", axis, "pulse_train", 0.7, 1, 3),
    ]
    if include_coupled:
        maneuvers.extend(
            [
                ManeuverSpec(f"{axis}_strong_yaw_moderate", "M_strong", axis, "step", 0.9, 2, couplings={"yaw": 0.3}),
                ManeuverSpec(f"{axis}_strong_throttle_low", "M_strong", axis, "step", 0.9, 2, couplings={"throttle": -0.15}),
                ManeuverSpec(f"{axis}_strong_throttle_high", "M_strong", axis, "step", 0.9, 2, couplings={"throttle": 0.15}),
            ]
        )
    return maneuvers[: int(max_strong)]


def _read_headroom_defaults(
    config: ExperimentConfig,
    scenario: ScenarioCfg,
    seed: int,
) -> tuple[dict[str, float], dict[str, dict[str, Any]]]:
    if scenario.platform != "px4":
        return {}, {}
    metadata = _px4_metadata(config)
    names = [f"CA_ROTOR{i}_CT" for i in range(4) if f"CA_ROTOR{i}_CT" in metadata]
    if len(names) < 4:
        return {}, {}
    defaults: dict[str, float] = {}
    for name in names:
        default = _maybe_float(metadata[name].get("default"))
        if default is None:
            return {}, {}
        defaults[name] = float(default)
    return defaults, {name: metadata[name] for name in names}


def _headroom_profiles(
    defaults: dict[str, float],
    metadata: dict[str, dict[str, Any]],
    max_profiles: int,
) -> list[HeadroomProfile]:
    base = HeadroomProfile("H0", "normal/default headroom", {}, "none")
    if len(defaults) < 4:
        return [base]
    profiles = [base]
    for idx, scale in enumerate([0.9, 0.8, 0.7], start=1):
        overrides = {}
        rejected = False
        for name, default in defaults.items():
            meta = metadata.get(name, {})
            if bool(meta.get("rebootRequired", False)):
                rejected = True
                break
            overrides[name] = float(default) * scale
        if rejected:
            continue
        profiles.append(
            HeadroomProfile(
                f"H{idx}",
                f"PX4 CA rotor thrust coefficient scaled to {scale:.2f}",
                overrides,
                json.dumps({name: {"default": defaults[name], "override": overrides[name]} for name in overrides}, sort_keys=True),
            )
        )
    return profiles[: int(max_profiles)]


def _reuse_headroom(previous_run_dir: Path, headroom_defaults: dict[str, float]) -> HeadroomProfile | None:
    path = previous_run_dir / "reports" / "mantis_witness_summary.json"
    if not path.exists():
        return None
    try:
        summary = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    h_id = str(summary.get("selected_H_id", ""))
    audit_path = previous_run_dir / "reports" / "mantis_headroom_audit.csv"
    if not h_id or not audit_path.exists():
        return None
    rows = _read_rows(audit_path)
    row = next((item for item in rows if str(item.get("H_id")) == h_id), None)
    if row is None:
        return None
    desc = str(row.get("H_description", "reused headroom"))
    changed = str(row.get("changed_params_or_model_diff", ""))
    overrides = _parse_changed_headroom(changed, headroom_defaults)
    return HeadroomProfile(h_id, desc, overrides, changed)


def _parse_changed_headroom(changed: str, defaults: dict[str, float]) -> dict[str, float]:
    try:
        payload = json.loads(changed)
    except Exception:
        return {}
    overrides = {}
    for name, item in payload.items():
        if name not in defaults:
            continue
        if isinstance(item, dict) and "override" in item:
            overrides[name] = float(item["override"])
    return overrides


def _candidate_overrides(
    platform: str,
    axis: str,
    defaults: dict[str, float],
    metadata: dict[str, dict[str, Any]],
    multipliers: dict[str, float],
) -> tuple[dict[str, float], str]:
    roles = PX4_PARAMS[axis] if platform == "px4" else ARDUPILOT_PARAMS[axis]
    overrides: dict[str, float] = {}
    for role, multiplier in multipliers.items():
        if role not in roles:
            return {}, f"unsupported_role:{role}"
        name = roles[role]
        if name not in defaults:
            return {}, f"default_unavailable:{name}"
        meta = metadata.get(name, {})
        if bool(meta.get("rebootRequired", False)):
            return {}, f"reboot_required:{name}"
        value = float(defaults[name]) * float(multiplier)
        overrides[name] = _clip_to_metadata(value, meta)
    return overrides, ""


def _combo_specs(role_bounds: dict[str, float]) -> list[tuple[str, dict[str, float]]]:
    specs = []
    rp = role_bounds.get("rate_p", 1.6)
    low_d = role_bounds.get("rate_d_low", 0.75)
    high_i = role_bounds.get("rate_i", 1.5)
    att = role_bounds.get("att_p", 1.2)
    specs.append(("rate_p_boundary_low_d", {"rate_p": max(1.0, 0.95 * rp), "rate_d": low_d}))
    specs.append(("rate_p_boundary_high_i", {"rate_p": max(1.0, 0.95 * rp), "rate_i": high_i}))
    specs.append(("att_p_boundary_rate_p", {"att_p": att, "rate_p": max(1.0, 0.9 * rp)}))
    return specs


def _scenario_for_profile(
    scenario: ScenarioCfg,
    baseline_target: dict[str, float],
    headroom_defaults: dict[str, float],
    profile: HeadroomProfile,
) -> ScenarioCfg:
    overrides = dict(baseline_target)
    overrides.update(headroom_defaults)
    overrides.update(profile.overrides)
    return replace(scenario, param_overrides=overrides)


def _scenario_for(
    selected: dict[str, Any],
    scenario: ScenarioCfg,
    baseline_overrides: dict[str, float],
    candidate_overrides: dict[str, float],
) -> ScenarioCfg:
    scenario_h = selected.get("scenario_h")
    if isinstance(scenario_h, ScenarioCfg):
        overrides = dict(getattr(scenario_h, "param_overrides", {}) or {})
    else:
        overrides = dict(baseline_overrides)
    overrides.update(candidate_overrides)
    return replace(scenario, param_overrides=overrides)


def _witness_plan(config: ExperimentConfig, scenario: ScenarioCfg, args: argparse.Namespace, readiness: dict[str, Any]) -> dict[str, Any]:
    dry_headroom = _headroom_profiles_from_metadata(config, scenario, args.max_headroom_profiles)
    return {
        "commit": _git_commit(),
        "config": str(config.path),
        "scenario": scenario.id,
        "platform": scenario.platform,
        "axis": args.axis,
        "seed": int(args.seed),
        "readiness": readiness,
        "witness_definition": "Safe(Mstrong,P0,H) and Safe(M0,P,H) and Safe(Msmall,P,H) and Violation(Mstrong,P,H)",
        "contracts": {
            "C_recover": {
                "name": "post-neutral residual-rate",
                "roll_pitch_terminal_residual_rate_threshold_radps": 0.35,
                "rule": "existing terminal window, non-decay logic, and robust repeat rule",
            },
            "C_track": {
                "name": "active-window rate-tracking",
                "nte_threshold": C_TRACK_NTE_THRESHOLD,
                "peak_err_threshold_radps": C_TRACK_PEAK_ERR_THRESHOLD_RADPS,
                "high_err_duration_threshold_s": C_TRACK_HIGH_ERR_DURATION_THRESHOLD_S,
                "baseline_ratio": C_TRACK_BASELINE_RATIO,
                "saturation_error_overlap_threshold_s": C_TRACK_OVERLAP_THRESHOLD_S,
                "required_topics": ["vehicle_rates_setpoint", "vehicle_angular_velocity"],
            },
        },
        "threshold_tuning_policy": "fixed before rerun; do not tune after seeing data",
        "nonlinear_rule": "necessary but not sufficient; C_track requires overlap with high-error active window",
        "headroom_profiles_planned": [profile.__dict__ for profile in dry_headroom],
        "headroom_boundary_mode": bool(getattr(args, "headroom_boundary", False)),
        "headroom_boundary_scales": [0.65, 0.60, 0.55, 0.50],
        "headroom_boundary_known_safe_scale": 0.70,
        "headroom_boundary_refinement_points": 3,
        "headroom_boundary_policy": "H-induced P0 violations are BAD_HEADROOM, not accepted witnesses",
        "max_total_queries": int(args.max_total_queries),
        "stop_at_first_witness": bool(args.stop_at_first_witness),
    }


def _headroom_profiles_from_metadata(config: ExperimentConfig, scenario: ScenarioCfg, max_profiles: int) -> list[HeadroomProfile]:
    if scenario.platform != "px4":
        return [HeadroomProfile("H0", "normal/default headroom", {}, "none")]
    metadata = _px4_metadata(config)
    defaults = {}
    for name in [f"CA_ROTOR{i}_CT" for i in range(4)]:
        if name not in metadata:
            continue
        default = metadata[name].get("default")
        try:
            defaults[name] = float(default)
        except (TypeError, ValueError):
            defaults[name] = 6.5
    return _headroom_profiles(defaults, {name: metadata.get(name, {}) for name in defaults}, max_profiles)


def _px4_metadata(config: ExperimentConfig) -> dict[str, dict[str, Any]]:
    px4_cfg = config.simulator.get("px4", {})
    root = Path(px4_cfg.get("root", "/home/car/PX4-Autopilot"))
    path = Path(px4_cfg.get("parameters_json", root / "build/px4_sitl_default/parameters.json"))
    if not path.exists():
        return {}
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return {str(row["name"]): dict(row) for row in raw.get("parameters", []) if "name" in row}


def _selected_hstar_maneuvers(axis: str, scenario_id: str, max_strong: int) -> list[ManeuverSpec]:
    catalog = _hstar_maneuver_catalog(axis)
    prior_names = _prior_h3_strong_maneuver_names(axis, scenario_id, max_strong)
    selected = [catalog[name] for name in prior_names if name in catalog]
    if selected:
        return selected[: int(max_strong)]
    return _witness_maneuvers(axis, int(max_strong), include_coupled=False)


def _prior_h3_strong_maneuver_names(axis: str, scenario_id: str, max_strong: int) -> list[str]:
    if axis == "pitch":
        path = Path("runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_witness_tracking.csv")
    elif axis == "roll":
        path = Path("runs/mantis_witness_px4_roll_lowheadroom_seed0_v0/reports/mantis_witness_tracking.csv")
    else:
        return []
    if not path.exists():
        return []
    try:
        rows = pd.read_csv(path).to_dict("records")
    except Exception:
        return []
    scored: dict[str, float] = {}
    for row in rows:
        tag = str(row.get("cache_tag", ""))
        if "headroom_H3_Mstrong_P0" not in tag:
            continue
        name = str(row.get("maneuver", ""))
        if not name:
            continue
        nte = _finite_float(row.get("nte"), 0.0)
        overlap = _finite_float(row.get("saturation_error_overlap_s"), 0.0)
        scored[name] = max(scored.get(name, 0.0), nte + 0.01 * overlap)
    return [name for name, _ in sorted(scored.items(), key=lambda item: item[1], reverse=True)[: int(max_strong)]]


def _hstar_maneuver_catalog(axis: str) -> dict[str, ManeuverSpec]:
    maneuvers = _witness_maneuvers(axis, 8, include_coupled=False)
    maneuvers.extend(default_maneuvers(axis)["M_strong"])
    return {maneuver.name: maneuver for maneuver in maneuvers}


def _hstar_family_specs(axis: str) -> list[dict[str, Any]]:
    if axis == "pitch":
        return [
            {"label": "att_p_x2", "variable_role": "att_p", "target": 2.0, "direction": "high", "base": {}},
            {"label": "rate_d_high", "variable_role": "rate_d", "target": 2.5, "direction": "high", "base": {}},
            {"label": "rate_i_high", "variable_role": "rate_i", "target": 3.0, "direction": "high", "base": {}},
            {"label": "rate_d_low", "variable_role": "rate_d", "target": 0.35, "direction": "low", "base": {}},
        ]
    return [
        {"label": "rate_p_x1p5", "variable_role": "rate_p", "target": 3.0, "direction": "high", "base": {}},
        {
            "label": "att_p_boundary_rate_p",
            "variable_role": "rate_p",
            "target": 1.5,
            "direction": "high",
            "base": {"att_p": 2.0},
        },
        {
            "label": "rate_p_boundary_low_d",
            "variable_role": "rate_p",
            "target": 1.5,
            "direction": "high",
            "base": {"rate_d": 0.35},
        },
        {
            "label": "rate_p_boundary_high_i",
            "variable_role": "rate_p",
            "target": 1.5,
            "direction": "high",
            "base": {"rate_i": 3.0},
        },
    ]


def _hstar_family_label(family_label: str, variable_role: str, multiplier: float) -> str:
    if family_label in {"rate_d_high", "rate_d_low", "rate_i_high"}:
        base = {"rate_d_high": "rate_d_high", "rate_d_low": "rate_d_low", "rate_i_high": "rate_i"}[family_label]
        return f"{base}_x{_mult_label(multiplier)}"
    if family_label == "rate_p_x1p5":
        return f"rate_p_x{_mult_label(multiplier)}"
    if family_label in {"att_p_boundary_rate_p", "rate_p_boundary_low_d", "rate_p_boundary_high_i"}:
        return f"{family_label}_x{_mult_label(multiplier)}"
    return f"{family_label}_x{_mult_label(multiplier)}"


def _scaled_headroom_profile(
    scale: float,
    defaults: dict[str, float],
    metadata: dict[str, dict[str, Any]],
) -> HeadroomProfile:
    overrides: dict[str, float] = {}
    for name, default in defaults.items():
        meta = metadata.get(name, {})
        if bool(meta.get("rebootRequired", False)):
            continue
        overrides[name] = float(default) * float(scale)
    changed = json.dumps({name: {"default": defaults[name], "override": overrides[name]} for name in overrides}, sort_keys=True)
    return HeadroomProfile(
        f"Hscale_{_mult_label(scale)}",
        f"PX4 CA rotor thrust coefficient scaled to {scale:.5g}",
        overrides,
        changed,
    )


def _scenario_with_hstar_candidate(scenario_h: ScenarioCfg, candidate_overrides: dict[str, float]) -> ScenarioCfg:
    overrides = dict(getattr(scenario_h, "param_overrides", {}) or {})
    overrides.update(candidate_overrides)
    return replace(scenario_h, param_overrides=overrides)


def _hstar_boundary_row(
    ctx: HstarAxisContext,
    candidate_id: str,
    family_label: str,
    multipliers: dict[str, float],
    overrides: dict[str, float],
    *,
    skipped_reason: str,
) -> dict[str, Any]:
    return {
        "axis": ctx.axis,
        "scenario_id": ctx.scenario.id,
        "candidate_id": candidate_id,
        "family": family_label,
        "multipliers_json": json.dumps(multipliers, sort_keys=True),
        "overrides_json": json.dumps(overrides, sort_keys=True),
        "M0_status": "",
        "Msmall_status": "",
        "M0_C_track_status": "",
        "Msmall_C_track_status": "",
        "C_recover_safe": False,
        "C_track_safe": False,
        "retained_for_stage_c": False,
        "candidate_status": INVALID_PURE_PARAM,
        "terminal_peak_ratio": math.nan,
        "max_tracking_nte_msmall": math.nan,
        "nonlinear_overlap_msmall_s": math.nan,
        "nonlinear_observable": False,
        "support_size": len(overrides),
        "boundary_score": 0.0,
        "rejection_reason": skipped_reason,
    }


def _hstar_candidate_score(m0_bundle: EvalBundle, small_bundle: EvalBundle, support_size: int) -> float:
    score = _boundary_score(m0_bundle.summary, small_bundle.summary)
    score += 0.2 * min(_finite_float(small_bundle.tracking_summary.get("max_nte"), 0.0), 5.0)
    score += 0.1 * min(_max_from_rows(small_bundle.tracking_rows, "saturation_error_overlap_s"), 2.0)
    score -= 0.02 * max(0, int(support_size) - 1)
    return float(score)


def _max_track_baseline_ratio(rows: list[dict[str, Any]]) -> float:
    ratios = []
    for row in rows:
        nte = _finite_float(row.get("nte"), math.nan)
        baseline = _finite_float(row.get("baseline_nte_median"), math.nan)
        if math.isfinite(nte) and math.isfinite(baseline) and baseline > 1e-12:
            ratios.append(nte / baseline)
    return float(max(ratios)) if ratios else math.nan


def _worst_status(statuses: list[str]) -> str:
    cleaned = [status for status in statuses if status]
    if not cleaned:
        return ""
    if any(status == "violation_like" for status in cleaned):
        return "violation_like"
    if any(status == "noise_band" for status in cleaned):
        return "noise_band"
    if any(status == "C_track_unavailable" for status in cleaned):
        return "C_track_unavailable"
    if all(status == "safe" for status in cleaned):
        return "safe"
    return cleaned[0]


def _hstar_summary_defaults(
    headroom_rows: list[dict[str, Any]],
    boundary_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
    confirmation_rows: list[dict[str, Any]],
    *,
    accepted: bool,
) -> dict[str, Any]:
    selected = next((row for row in headroom_rows if bool(row.get("selected_as_Hstar", False))), {})
    return {
        "scales_tried": [row.get("H_scale") for row in headroom_rows],
        "lowest_admissible_Hstar": selected.get("H_scale", ""),
        "first_bad_headroom_scale": next((row.get("H_scale") for row in headroom_rows if not bool(row.get("admissible", False))), ""),
        "default_safe_boundary_reached": any(not bool(row.get("admissible", False)) for row in headroom_rows),
        "Hstar_default_max_recover_ratio": selected.get("Mstrong_default_max_recover_ratio", math.nan),
        "Hstar_default_max_track_nte": selected.get("Mstrong_default_max_track_nte", math.nan),
        "Hstar_nonlinear_observable": selected.get("nonlinear_observable", False),
        "Hstar_nonlinear_activation_rate": selected.get("nonlinear_activation_rate", math.nan),
        "Hstar_max_saturation_overlap_s": selected.get("max_saturation_overlap_s", math.nan),
        "boundary_candidates_generated": len(boundary_rows),
        "pure_param_rejected": sum(1 for row in boundary_rows if row.get("candidate_status") == INVALID_PURE_PARAM),
        "small_safe_retained": sum(1 for row in boundary_rows if bool(row.get("retained_for_stage_c", False))),
        "candidate_count": len(candidate_rows),
        "strong_unsafe_by_C_recover": sum(1 for row in candidate_rows if bool(row.get("C_recover_violation", False))),
        "strong_unsafe_by_C_track": sum(1 for row in candidate_rows if bool(row.get("C_track_violation", False))),
        "confirmation_rows": len(confirmation_rows),
        "accepted_witness_count": 1 if accepted else 0,
    }


def _hstar_failed_condition(
    status: str,
    selected: dict[str, Any] | None,
    retained: list[BoundaryCandidate],
    candidate_rows: list[dict[str, Any]],
    confirmation_rows: list[dict[str, Any]],
) -> str:
    if status == ACCEPTED:
        return ""
    if selected is None:
        return "no admissible H*"
    if not retained:
        return "no small-safe P"
    if not candidate_rows or not any(
        bool(row.get("C_recover_violation", False)) or bool(row.get("C_track_violation", False)) for row in candidate_rows
    ):
        return "no strong violation"
    if confirmation_rows and any(str(row.get("failed_condition", "")) == "nonlinear overlap missing" for row in confirmation_rows):
        return "nonlinear overlap missing"
    if confirmation_rows:
        return "confirmation failed"
    return "confirmation failed"


def _hstar_confirmation_failed_condition(
    control_safe: bool,
    recover_repeat_robust: bool,
    track_repeat_robust: bool,
    nonlinear_observable: bool,
    nonlinear_activated: bool,
    max_track_overlap: float,
) -> str:
    if not control_safe:
        return "confirmation failed"
    if not recover_repeat_robust and not track_repeat_robust:
        return "confirmation failed"
    if not nonlinear_observable:
        return "nonlinear overlap missing"
    if recover_repeat_robust and not nonlinear_activated:
        return "nonlinear overlap missing"
    if track_repeat_robust and max_track_overlap < C_TRACK_OVERLAP_THRESHOLD_S:
        return "nonlinear overlap missing"
    return "confirmation failed"


def _write_hstar_reports(
    run_dir: Path,
    reports_dir: Path,
    summary: dict[str, Any],
    plan: dict[str, Any],
    headroom_rows: list[dict[str, Any]],
    boundary_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
    confirmation_rows: list[dict[str, Any]],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
    plot_paths: list[str],
) -> None:
    reports_dir.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(headroom_rows, columns=_hstar_headroom_columns()).to_csv(reports_dir / "mantis_headroom_boundary.csv", index=False)
    pd.DataFrame(boundary_rows).to_csv(reports_dir / "mantis_Hstar_boundary_candidates.csv", index=False)
    pd.DataFrame(candidate_rows, columns=_hstar_candidate_columns() if not candidate_rows else None).to_csv(
        reports_dir / "mantis_Hstar_witness_candidates.csv", index=False
    )
    pd.DataFrame(confirmation_rows, columns=_hstar_confirmation_columns() if not confirmation_rows else None).to_csv(
        reports_dir / "mantis_Hstar_witness_confirmation.csv", index=False
    )
    pd.DataFrame(tracking_rows).to_csv(reports_dir / "mantis_witness_tracking.csv", index=False)
    pd.DataFrame(nonlinear_rows).to_csv(reports_dir / "mantis_nonlinear_diagnostics.csv", index=False)
    summary["mode_trace_rows"] = _write_mode_trace_report(run_dir)
    summary["nonlinear_backfill"] = _backfill_nonlinear_reports(run_dir, plan.get("axis"))
    summary["plot_paths"] = plot_paths
    _write_json(reports_dir / "mantis_Hstar_summary.json", summary)
    _write_json(reports_dir / "mantis_witness_summary.json", summary)
    _write_hstar_report(reports_dir / "mantis_Hstar_report.md", summary, plan, headroom_rows, boundary_rows, candidate_rows, confirmation_rows, plot_paths)
    _write_hstar_report(
        reports_dir / "mantis_witness_report.md",
        summary,
        plan,
        headroom_rows,
        boundary_rows,
        candidate_rows,
        confirmation_rows,
        plot_paths,
    )
    pd.DataFrame(headroom_rows).to_csv(reports_dir / "mantis_headroom_audit.csv", index=False)
    pd.DataFrame(boundary_rows).to_csv(reports_dir / "mantis_witness_boundary.csv", index=False)
    pd.DataFrame(candidate_rows).to_csv(reports_dir / "mantis_witness_candidates.csv", index=False)
    pd.DataFrame(confirmation_rows).to_csv(reports_dir / "mantis_witness_confirmation.csv", index=False)
    _copy_compact_artifacts(run_dir)


def _write_hstar_report(
    path: Path,
    summary: dict[str, Any],
    plan: dict[str, Any],
    headroom_rows: list[dict[str, Any]],
    boundary_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
    confirmation_rows: list[dict[str, Any]],
    plot_paths: list[str],
) -> None:
    selected = next((row for row in headroom_rows if bool(row.get("selected_as_Hstar", False))), {})
    first_bad = next((row.get("H_scale") for row in headroom_rows if not bool(row.get("admissible", False))), "")
    top_retained = [
        row.get("candidate_id")
        for row in sorted(
            [row for row in boundary_rows if bool(row.get("retained_for_stage_c", False))],
            key=lambda item: _finite_float(item.get("boundary_score"), 0.0),
            reverse=True,
        )[:6]
    ]
    lines = [
        "# MANTIS H* Witness Report",
        "",
        f"Status: `{summary['status']}`",
        f"Run dir: `{summary['run_dir']}`",
        f"Queries: `{summary['query_count']}`",
        f"Failed condition: `{summary.get('note', '')}`",
        "",
        "## A. H-Boundary Result",
        "",
        f"- Scales tried: `{summary.get('scales_tried', [])}`",
        f"- Lowest admissible H*: `{summary.get('lowest_admissible_Hstar', '')}`",
        f"- First bad-headroom scale: `{first_bad}`",
        f"- Default-safe boundary reached: `{summary.get('default_safe_boundary_reached', False)}`",
        "",
        "## B. H* Default Gates",
        "",
        f"- M0@P0: C_recover `{selected.get('M0_C_recover_status', '')}`, C_track `{selected.get('M0_C_track_status', '')}`",
        f"- Msmall@P0: C_recover `{selected.get('Msmall_C_recover_status', '')}`, C_track `{selected.get('Msmall_C_track_status', '')}`",
        f"- Strongest Mstrong@P0 max C_recover ratio: `{selected.get('Mstrong_default_max_recover_ratio', '')}`",
        f"- Strongest Mstrong@P0 max C_track nte: `{selected.get('Mstrong_default_max_track_nte', '')}`",
        f"- Nonlinear observable: `{selected.get('nonlinear_observable', '')}`; activation rate `{selected.get('nonlinear_activation_rate', '')}`; max overlap `{selected.get('max_saturation_overlap_s', '')}`",
        "",
        "## C. P Boundary Under H*",
        "",
        f"- Candidates generated: `{len(boundary_rows)}`",
        f"- Pure-param rejected: `{summary.get('pure_param_rejected', 0)}`",
        f"- Small-safe retained: `{summary.get('small_safe_retained', 0)}`",
        f"- Top retained candidates: `{top_retained}`",
        "",
        "## D. Witness Results",
        "",
        f"- C_recover violations: `{summary.get('strong_unsafe_by_C_recover', 0)}`",
        f"- C_track violations: `{summary.get('strong_unsafe_by_C_track', 0)}`",
        f"- Confirmation rows: `{len(confirmation_rows)}`",
        f"- Accepted witness count: `{summary.get('accepted_witness_count', 0)}`",
        "",
        "## E. Top-Level Status",
        "",
        f"`{summary['status']}`",
        "",
        "## F. Failed Condition",
        "",
        f"`{summary.get('note', '')}`",
        "",
        "## G. Recommendation",
        "",
    ]
    if summary["status"] == NO_WITNESS_HSTAR:
        lines.append("Pivot to the ArduPilot STABILIZE witness fast path or stop the PX4 MANTIS positive-bug search.")
    else:
        lines.append("Use the confirmation and candidate tables as the next decision point.")
    lines.extend(["", "## Plots", ""])
    if plot_paths:
        lines.extend(f"- `{item}`" for item in plot_paths)
    else:
        lines.append("- no candidate plots generated")
    lines.extend(
        [
            "",
            "## Evidence Tables",
            "",
            f"- `{path.parent / 'mantis_headroom_boundary.csv'}`",
            f"- `{path.parent / 'mantis_Hstar_boundary_candidates.csv'}`",
            f"- `{path.parent / 'mantis_Hstar_witness_candidates.csv'}`",
            f"- `{path.parent / 'mantis_Hstar_witness_confirmation.csv'}`",
            f"- `{path.parent / 'mantis_witness_tracking.csv'}`",
            f"- `{path.parent / 'mantis_nonlinear_diagnostics.csv'}`",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_hstar_top_candidate_plots_from_run(
    plots_dir: Path,
    run_dir: Path,
    selected: dict[str, Any],
    candidate_rows: list[dict[str, Any]],
    tracking_rows: list[dict[str, Any]],
) -> list[str]:
    query_index = _query_index_by_cache_tag(run_dir)
    tracking_by_tag = {str(row.get("cache_tag", "")): row for row in tracking_rows if row.get("cache_tag")}
    selected_by_axis = selected.get("by_axis", {})
    paths: list[str] = []
    for candidate_id, row in _top_candidate_plot_rows(candidate_rows).items():
        axis = str(row.get("axis", ""))
        maneuver = str(row.get("maneuver", ""))
        axis_selected = selected_by_axis.get(axis, {})
        if not axis or not maneuver or not axis_selected:
            continue
        tags = {
            "Mstrong_P0_H": f"Hboundary_{axis}_{selected['profile'].H_id}_Mstrong_P0_{maneuver}_repeat0",
            "M0_P_H": f"Hstar_boundary_{axis}_{candidate_id}_M0_hover_no_input_repeat0",
            "Msmall_P_H": f"Hstar_boundary_{axis}_{candidate_id}_Msmall_small_step_{axis}_repeat0",
            "Mstrong_P_H": f"Hstar_witness_{axis}_{candidate_id}_{maneuver}_{maneuver}_repeat0",
        }
        artifacts: dict[str, Any] = {}
        missing = False
        for arm, tag in tags.items():
            query_dir = query_index.get(tag)
            if query_dir is None:
                missing = True
                break
            try:
                artifacts[arm] = [read_parsed_log(query_dir / "parsed_log.parquet")]
            except Exception:
                missing = True
                break
        if missing:
            continue
        artifacts["tracking_rows"] = [tracking_by_tag[tag] for tag in tags.values() if tag in tracking_by_tag]
        paths.extend(_write_candidate_plots(plots_dir, f"Hstar_{candidate_id}", axis, artifacts, []))
    return paths


def _hstar_headroom_columns() -> list[str]:
    return [
        "H_scale",
        "admissible",
        "rejection_reason",
        "M0_C_recover_status",
        "M0_C_track_status",
        "Msmall_C_recover_status",
        "Msmall_C_track_status",
        "Mstrong_default_max_recover_ratio",
        "Mstrong_default_max_track_nte",
        "Mstrong_default_max_track_baseline_ratio",
        "nonlinear_observable",
        "nonlinear_activation_rate",
        "max_saturation_overlap_s",
        "selected_as_Hstar",
    ]


def _hstar_candidate_columns() -> list[str]:
    return [
        "axis",
        "scenario_id",
        "candidate_id",
        "family",
        "H_scale",
        "param_overrides_json",
        "maneuver",
        "C_recover_status",
        "C_recover_violation",
        "C_track_status",
        "C_track_violation",
        "violation_contract",
        "nonlinear_observable",
        "nonlinear_activated",
        "saturation_error_overlap_s",
        "candidate_status",
    ]


def _hstar_confirmation_columns() -> list[str]:
    return [
        "stage",
        "axis",
        "scenario_id",
        "candidate_id",
        "H_scale",
        "arm",
        "maneuver",
        "C_recover_status",
        "C_recover_violation_count",
        "C_track_status",
        "C_track_violation_count",
        "candidate_status",
        "failed_condition",
    ]


def _write_candidate_plots(
    plots_dir: Path,
    candidate_id: str,
    axis: str,
    artifacts: dict[str, Any],
    confirmation_rows: list[dict[str, Any]],
) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return []
    paths: list[str] = []
    arm_logs = {key: value[0] for key, value in artifacts.items() if isinstance(value, list) and value and isinstance(value[0], pd.DataFrame)}
    if not arm_logs:
        return []
    rate_col = f"{axis}_rate_rps"
    manual_col = f"manual_{axis}"

    fig, ax = plt.subplots(figsize=(9, 4))
    for arm, df in arm_logs.items():
        if "time_s" in df and rate_col in df:
            ax.plot(df["time_s"], df[rate_col], label=arm)
    ax.set_title(f"{candidate_id} four-arm {axis} rates")
    ax.set_xlabel("time_s")
    ax.set_ylabel("rate_radps")
    ax.legend(fontsize=7)
    path = plots_dir / f"{candidate_id}_four_arm_rates.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    paths.append(str(path))

    tracking_rows = artifacts.get("tracking_rows", [])
    labels = [str(row.get("cache_tag", ""))[-24:] for row in tracking_rows]
    nte = [_finite_float(row.get("nte"), 0.0) for row in tracking_rows]
    overlap = [_finite_float(row.get("saturation_error_overlap_s"), 0.0) for row in tracking_rows]
    if labels:
        fig, ax = plt.subplots(figsize=(9, 4))
        x = np.arange(len(labels))
        ax.bar(x - 0.2, nte, width=0.4, label="nte")
        ax.bar(x + 0.2, overlap, width=0.4, label="overlap_s")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=7)
        ax.set_title(f"{candidate_id} tracking error")
        ax.legend(fontsize=7)
        path = plots_dir / f"{candidate_id}_tracking_error.png"
        fig.tight_layout()
        fig.savefig(path)
        plt.close(fig)
        paths.append(str(path))

        fig, ax = plt.subplots(figsize=(9, 4))
        ax.bar(x, overlap)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=7)
        ax.set_title(f"{candidate_id} actuator saturation overlap")
        ax.set_ylabel("seconds")
        path = plots_dir / f"{candidate_id}_actuator_saturation.png"
        fig.tight_layout()
        fig.savefig(path)
        plt.close(fig)
        paths.append(str(path))

    fig, ax = plt.subplots(figsize=(9, 4))
    for arm, df in arm_logs.items():
        if "time_s" in df and manual_col in df:
            ax.plot(df["time_s"], df[manual_col], label=arm)
    ax.set_title(f"{candidate_id} manual {axis} input")
    ax.set_xlabel("time_s")
    ax.set_ylabel("manual")
    ax.legend(fontsize=7)
    path = plots_dir / f"{candidate_id}_manual_input.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    paths.append(str(path))
    return paths


def _finalize_reports(
    run_dir: Path,
    reports_dir: Path,
    summary: dict[str, Any],
    plan: dict[str, Any],
    headroom_rows: list[dict[str, Any]],
    boundary_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
    confirmation_rows: list[dict[str, Any]],
    tracking_rows: list[dict[str, Any]],
    nonlinear_rows: list[dict[str, Any]],
    plot_paths: list[str],
) -> None:
    pd.DataFrame(tracking_rows).to_csv(reports_dir / "mantis_witness_tracking.csv", index=False)
    pd.DataFrame(nonlinear_rows).to_csv(reports_dir / "mantis_nonlinear_diagnostics.csv", index=False)
    if candidate_rows:
        plot_paths = list(plot_paths)
        existing = set(plot_paths)
        for path in _write_top_candidate_plots_from_run(
            run_dir / "plots",
            run_dir,
            str(plan.get("axis", "")),
            str(summary.get("selected_H_id", "")),
            candidate_rows,
            tracking_rows,
        ):
            if path not in existing:
                plot_paths.append(path)
                existing.add(path)
    summary["mode_trace_rows"] = _write_mode_trace_report(run_dir)
    summary["nonlinear_backfill"] = _backfill_nonlinear_reports(run_dir, plan.get("axis"))
    summary["plot_paths"] = plot_paths
    _write_json(reports_dir / "mantis_witness_summary.json", summary)
    _write_report(
        reports_dir / "mantis_witness_report.md",
        summary,
        plan,
        headroom_rows,
        boundary_rows,
        candidate_rows,
        confirmation_rows,
        plot_paths,
    )
    _copy_compact_artifacts(run_dir)


def _write_top_candidate_plots_from_run(
    plots_dir: Path,
    run_dir: Path,
    axis: str,
    selected_h_id: str,
    candidate_rows: list[dict[str, Any]],
    tracking_rows: list[dict[str, Any]],
) -> list[str]:
    if not axis or not candidate_rows:
        return []
    plots_dir.mkdir(parents=True, exist_ok=True)
    query_index = _query_index_by_cache_tag(run_dir)
    tracking_by_tag = {str(row.get("cache_tag", "")): row for row in tracking_rows if row.get("cache_tag")}
    paths: list[str] = []
    for candidate_id, row in _top_candidate_plot_rows(candidate_rows).items():
        maneuver = str(row.get("maneuver", ""))
        if not maneuver:
            continue
        tags = {
            "Mstrong_P0_H": f"headroom_{selected_h_id}_Mstrong_P0_{maneuver}_repeat0",
            "M0_P_H": f"boundary_{candidate_id}_M0_hover_no_input_repeat0",
            "Msmall_P_H": f"boundary_{candidate_id}_Msmall_small_step_{axis}_repeat0",
            "Mstrong_P_H": f"witness_stage_c_{candidate_id}_{maneuver}_{maneuver}_repeat0",
        }
        artifacts: dict[str, Any] = {}
        missing = False
        for arm, tag in tags.items():
            query_dir = query_index.get(tag)
            if query_dir is None:
                missing = True
                break
            try:
                artifacts[arm] = [read_parsed_log(query_dir / "parsed_log.parquet")]
            except Exception:
                missing = True
                break
        if missing:
            continue
        artifacts["tracking_rows"] = [tracking_by_tag[tag] for tag in tags.values() if tag in tracking_by_tag]
        paths.extend(_write_candidate_plots(plots_dir, str(candidate_id), axis, artifacts, []))
    return paths


def _top_candidate_plot_rows(candidate_rows: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    selected: dict[str, dict[str, Any]] = {}
    order: list[str] = []
    for row in candidate_rows:
        candidate_id = str(row.get("candidate_id", ""))
        if not candidate_id:
            continue
        if candidate_id not in selected:
            selected[candidate_id] = row
            order.append(candidate_id)
            continue
        if _candidate_plot_score(row) > _candidate_plot_score(selected[candidate_id]):
            selected[candidate_id] = row
    return {candidate_id: selected[candidate_id] for candidate_id in order}


def _candidate_plot_score(row: dict[str, Any]) -> tuple[float, float, float]:
    return (
        _finite_float(row.get("terminal_peak_over_threshold"), 0.0),
        _finite_float(row.get("saturation_error_overlap_s"), 0.0),
        _finite_float(row.get("terminal_over_start_peak"), 0.0),
    )


def _query_index_by_cache_tag(run_dir: Path) -> dict[str, Path]:
    index: dict[str, Path] = {}
    for metadata_path in (run_dir / "queries").glob("*/metadata.json"):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        tag = str(metadata.get("cache_tag", ""))
        if tag:
            index[tag] = metadata_path.parent
    return index


def _write_report(
    path: Path,
    summary: dict[str, Any],
    plan: dict[str, Any],
    headroom_rows: list[dict[str, Any]],
    boundary_rows: list[dict[str, Any]],
    candidate_rows: list[dict[str, Any]],
    confirmation_rows: list[dict[str, Any]],
    plot_paths: list[str],
) -> None:
    accepted = int(summary.get("accepted_witness_count", 1 if summary.get("status") == ACCEPTED else 0))
    lines = [
        "# MANTIS Witness Report",
        "",
        f"Status: `{summary['status']}`",
        f"Run dir: `{summary['run_dir']}`",
        f"Queries: `{summary['query_count']}`",
        f"Note: `{summary.get('note', '')}`",
        "",
        "## A. Witness Plan",
        "",
        "- Contracts: `C_recover` post-neutral residual-rate and pre-registered `C_track` active-window rate-tracking.",
        f"- C_track thresholds: nte>={C_TRACK_NTE_THRESHOLD}, peak_err>={C_TRACK_PEAK_ERR_THRESHOLD_RADPS} rad/s, "
        f"high_err_duration>={C_TRACK_HIGH_ERR_DURATION_THRESHOLD_S}s, baseline ratio>={C_TRACK_BASELINE_RATIO}, "
        f"nonlinear overlap>={C_TRACK_OVERLAP_THRESHOLD_S}s.",
        "- C_track was pre-registered because recovery-only checks can miss poor active-window tracking that recovers quickly.",
        "- Low headroom is admissible only when default-parameter gates remain safe.",
        "",
        "## B. Headroom Audit",
        "",
        f"- Profiles tried: `{len(headroom_rows)}`",
        f"- Selected H: `{summary.get('selected_H_id', '')}` {summary.get('selected_H_description', '')}",
        "",
        "## C. Boundary Search",
        "",
        f"- Boundary rows: `{len(boundary_rows)}`",
        f"- Small-safe retained: `{summary.get('small_safe_retained', 0)}`",
        f"- Pure-param rejected: `{sum(1 for row in boundary_rows if row.get('candidate_status') == INVALID_PURE_PARAM)}`",
        "",
        "## D. Witness Results",
        "",
        f"- Candidate count: `{len(candidate_rows)}`",
        f"- Strong-unsafe by C_recover: `{summary.get('strong_unsafe_by_C_recover', 0)}`",
        f"- Strong-unsafe by C_track: `{summary.get('strong_unsafe_by_C_track', 0)}`",
        f"- Confirmation rows: `{len(confirmation_rows)}`",
        f"- Accepted witness count: `{accepted}`",
        "",
        "## E. Visual Evidence",
        "",
    ]
    if plot_paths:
        lines.extend(f"- `{item}`" for item in plot_paths)
    else:
        lines.append("- no candidate plots generated")
    lines.extend(
        [
            "",
            "## F. Top-Level Status",
            "",
            f"`{summary['status']}`",
            "",
            "## G. Failed Condition If Not Accepted",
            "",
            f"`{summary.get('note', '')}`",
            "",
            "## Evidence Tables",
            "",
            f"- `{path.parent / 'mantis_witness_plan.json'}`",
            f"- `{path.parent / 'mantis_headroom_audit.csv'}`",
            f"- `{path.parent / 'mantis_witness_boundary.csv'}`",
            f"- `{path.parent / 'mantis_witness_candidates.csv'}`",
            f"- `{path.parent / 'mantis_witness_confirmation.csv'}`",
            f"- `{path.parent / 'mantis_witness_tracking.csv'}`",
            f"- `{path.parent / 'mantis_nonlinear_diagnostics.csv'}`",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_empty_reports(reports_dir: Path) -> None:
    pd.DataFrame(columns=_headroom_columns()).to_csv(reports_dir / "mantis_headroom_audit.csv", index=False)
    pd.DataFrame(columns=_boundary_columns()).to_csv(reports_dir / "mantis_witness_boundary.csv", index=False)
    pd.DataFrame(columns=_candidate_columns()).to_csv(reports_dir / "mantis_witness_candidates.csv", index=False)
    pd.DataFrame(columns=_confirmation_columns()).to_csv(reports_dir / "mantis_witness_confirmation.csv", index=False)
    pd.DataFrame(columns=["query_id", "C_track_status", "nte"]).to_csv(reports_dir / "mantis_witness_tracking.csv", index=False)
    pd.DataFrame(columns=["query_id", "nonlinear_observability", "nonlinear_activated"]).to_csv(
        reports_dir / "mantis_nonlinear_diagnostics.csv", index=False
    )


def _ensure_empty_reports(reports_dir: Path) -> None:
    reports_dir.mkdir(parents=True, exist_ok=True)
    for name in [
        "mantis_headroom_audit.csv",
        "mantis_witness_boundary.csv",
        "mantis_witness_candidates.csv",
        "mantis_witness_confirmation.csv",
        "mantis_witness_tracking.csv",
        "mantis_nonlinear_diagnostics.csv",
    ]:
        path = reports_dir / name
        if not path.exists():
            _write_empty_reports(reports_dir)
            return


def _summary(status: str, run_dir: Path, started: float, *, query_count: int, readiness: dict[str, Any], note: str) -> dict[str, Any]:
    return {
        "status": status,
        "run_dir": str(run_dir),
        "query_count": int(query_count),
        "elapsed_wall_time_s": time.monotonic() - started,
        "note": note,
        "readiness": readiness,
    }


def _failed_condition(
    status: str,
    selected: dict[str, Any],
    retained: list[BoundaryCandidate],
    candidate_rows: list[dict[str, Any]],
    confirmation_rows: list[dict[str, Any]],
) -> str:
    if status == ACCEPTED:
        return ""
    if selected is None:
        return "no admissible low-headroom H"
    if not retained:
        return "pure-param failure or no small-safe boundary candidates"
    if not candidate_rows:
        return "no C_recover/C_track violation"
    if not any(bool(row.get("C_recover_violation")) or bool(row.get("C_track_violation")) for row in candidate_rows):
        return "no C_recover/C_track violation"
    if confirmation_rows and any(row.get("failed_condition") for row in confirmation_rows):
        return str(next(row.get("failed_condition") for row in confirmation_rows if row.get("failed_condition")))
    return "repeat confirmation failed"


def _confirmation_failed_condition(
    control_safe: bool,
    recover_repeat_robust: bool,
    track_repeat_robust: bool,
    nonlinear_observable: bool,
    nonlinear_activated: bool,
    max_track_overlap: float,
) -> str:
    if not control_safe:
        return "pure-param failure or pure-input/default P0 failure"
    if not recover_repeat_robust and not track_repeat_robust:
        return "repeat confirmation failed"
    if not nonlinear_observable:
        return "nonlinear observability missing"
    if recover_repeat_robust and not nonlinear_activated:
        return "nonlinear activation overlap missing"
    if track_repeat_robust and max_track_overlap < C_TRACK_OVERLAP_THRESHOLD_S:
        return "nonlinear activation overlap missing"
    return "conditional witness not accepted"


def _violation_contract(recover: bool, track: bool) -> str:
    if recover and track:
        return "C_recover_and_C_track"
    if recover:
        return "C_recover"
    if track:
        return "C_track"
    return ""


def _boundary_row(candidate_id: str, overrides: dict[str, float], *, skipped_reason: str) -> dict[str, Any]:
    return {
        "candidate_id": candidate_id,
        "overrides_json": json.dumps(overrides, sort_keys=True),
        "M0_status": "",
        "Msmall_status": "",
        "C_recover_safe": False,
        "C_track_safe": False,
        "retained_for_stage_c": False,
        "candidate_status": INVALID_PURE_PARAM,
        "boundary_score": 0.0,
        "rejection_reason": skipped_reason,
    }


def _headroom_columns() -> list[str]:
    return [
        "H_id",
        "H_description",
        "changed_params_or_model_diff",
        "M0_status",
        "Msmall_status",
        "Mstrong_default_status",
        "C_recover_safe",
        "C_track_safe",
        "nonlinear_observable",
        "nonlinear_activation_rate",
        "max_tracking_nte_default",
        "admissible",
        "rejection_reason",
    ]


def _boundary_columns() -> list[str]:
    return [
        "candidate_id",
        "H_id",
        "multipliers_json",
        "overrides_json",
        "M0_status",
        "Msmall_status",
        "C_recover_safe",
        "C_track_safe",
        "retained_for_stage_c",
        "candidate_status",
        "boundary_score",
        "rejection_reason",
    ]


def _candidate_columns() -> list[str]:
    return [
        "candidate_id",
        "H_id",
        "param_overrides_json",
        "maneuver",
        "C_recover_violation",
        "C_track_violation",
        "violation_contract",
        "candidate_status",
    ]


def _confirmation_columns() -> list[str]:
    return [
        "stage",
        "candidate_id",
        "H_id",
        "arm",
        "maneuver",
        "C_recover_status",
        "C_track_status",
        "candidate_status",
        "failed_condition",
    ]


def _flat_row(row: dict[str, Any]) -> dict[str, Any]:
    return {key: _flat_value(value) for key, value in row.items()}


def _flat_value(value: Any) -> Any:
    if isinstance(value, list):
        return ";".join(str(item) for item in value)
    if isinstance(value, dict):
        return json.dumps({str(key): _flat_value(item) for key, item in value.items()}, sort_keys=True)
    if isinstance(value, float) and not math.isfinite(value):
        return math.nan
    return value


def _arm_status(rows: list[dict[str, Any]], arm: str) -> str:
    matches = [row for row in rows if row.get("arm") == arm]
    if not matches:
        return ""
    return "safe" if all(bool(row.get("recover_safe")) and bool(row.get("track_safe")) for row in matches) else "not_safe"


def _max_from_rows(rows: list[dict[str, Any]], key: str) -> float:
    values = [_finite_float(row.get(key), math.nan) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    return float(np.nanmax(values)) if values else 0.0


def _mult_label(value: float) -> str:
    return f"{value:.3f}".rstrip("0").rstrip(".").replace(".", "p")


def _clip_to_metadata(value: float, metadata: dict[str, Any]) -> float:
    lo = _maybe_float(metadata.get("min", metadata.get("minimum")))
    hi = _maybe_float(metadata.get("max", metadata.get("maximum")))
    if lo is not None:
        value = max(value, lo)
    if hi is not None:
        value = min(value, hi)
    return float(value)


def _maybe_float(value: Any) -> float | None:
    try:
        if value in (None, ""):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _is_mode_failure(exc: Exception) -> bool:
    text = str(exc)
    return "Mode did not switch" in text or "mode" in text.lower() and "switch" in text.lower()


def _kill_sim() -> None:
    script = Path("scripts/kill_sim.sh")
    if script.exists():
        subprocess.run([str(script)], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        subprocess.run(["pkill", "-TERM", "-f", "jmavsim|px4_sitl"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _query_jsonl_count(run_dir: Path) -> int:
    path = run_dir / "logs" / "queries.jsonl"
    if not path.exists():
        return 0
    return sum(1 for line in path.read_text(encoding="utf-8").splitlines() if line.strip())


def _read_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    try:
        return pd.read_csv(path).to_dict("records")
    except pd.errors.EmptyDataError:
        return []


if __name__ == "__main__":
    main()
