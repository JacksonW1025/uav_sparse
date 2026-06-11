from __future__ import annotations

import json
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))
sys.path.insert(0, str(PLANC_ROOT / "analysis"))

from env_probe import probe_environment, write_env
from flight import run_flight
from oracle import parse_dataflash
from param_manager import ParamManager
from plots import plot_run
from sitl_runner import SitlRunner


def load_config() -> dict[str, Any]:
    with (PLANC_ROOT / "config" / "gate_config.yaml").open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def controlled_params(config: dict[str, Any], arm_cfg: dict[str, Any]) -> dict[str, Any]:
    params = dict(config.get("baseline_params", {}))
    params.update(arm_cfg.get("params", {}))
    return params


def expected_action_modes(config: dict[str, Any], params: dict[str, Any]) -> list[str]:
    action = int(float(params.get("FENCE_ACTION", config["baseline_params"].get("FENCE_ACTION", 1))))
    oracle = config["oracle"]
    if action == int(config["param_metadata"].get("fence_action_brake", 4)):
        return list(oracle["expected_action_modes_for_brake"])
    return list(oracle["expected_action_modes_for_rtl"])


def connectivity_probe(config: dict[str, Any], env: dict[str, Any]) -> dict[str, Any]:
    runner = SitlRunner(config, REPO_ROOT)
    run_id = "connectivity_probe"
    try:
        runner.start(run_id)
        master = runner.connect(timeout_s=30)
        pm = ParamManager(master)
        sim_speedup_before = pm.read("SIM_SPEEDUP")
        pm.set_and_readback("SIM_SPEEDUP", float(config["experiment"]["speedup"]))
        probe = {
            "ok": True,
            "heartbeat_target_system": master.target_system,
            "heartbeat_target_component": master.target_component,
            "connection": runner.connection_string,
            "sim_speedup_before": sim_speedup_before,
            "param_records": pm.records,
        }
        try:
            master.close()
        except Exception:
            pass
        env["connectivity_probe"] = probe
        return probe
    except Exception as exc:
        probe = {"ok": False, "error": repr(exc), "traceback": traceback.format_exc()}
        env["connectivity_probe"] = probe
        return probe
    finally:
        runner.stop()


def run_one(config: dict[str, Any], arm_name: str, arm_cfg: dict[str, Any], rep_index: int) -> dict[str, Any]:
    run_id = f"{arm_name}_r{rep_index}"
    runner = SitlRunner(config, REPO_ROOT)
    result: dict[str, Any] = {
        "run_id": run_id,
        "arm": arm_name,
        "label": arm_cfg.get("label", arm_name),
        "rep_index": rep_index,
        "motion": arm_cfg["motion"],
        "environment": arm_cfg.get("environment"),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    master = None
    try:
        work_dir = runner.start(run_id)
        master = runner.connect(timeout_s=30)
        params = controlled_params(config, arm_cfg)
        pm = ParamManager(master)
        pm.apply(params)
        snapshot = pm.snapshot(sorted(params))
        param_record_path = PLANC_ROOT / "logs" / f"{run_id}_params.json"
        pm.write_records(param_record_path, snapshot=snapshot)
        result["params_requested"] = params
        result["param_snapshot"] = snapshot
        result["param_records_path"] = str(param_record_path)
        result["param_readbacks"] = pm.records
        result["flight"] = run_flight(master, config, arm_cfg["motion"])
        try:
            master.close()
        except Exception:
            pass
        master = None
        runner.stop()
        bin_path = runner.collect_dataflash(run_id)
        result["work_dir"] = str(work_dir)
        if bin_path is None:
            result["error"] = "No DataFlash .BIN log found after run"
            return result
        result["bin_path"] = str(bin_path)
        csv_path = PLANC_ROOT / "logs" / f"{run_id}_parsed.csv"
        expect_breach = arm_cfg["motion"] == "witness_velocity"
        parsed = parse_dataflash(
            bin_path=bin_path,
            csv_path=csv_path,
            home=config["experiment"]["home"],
            fence_radius_m=float(config["oracle"]["fence_radius_m"]),
            expected_action_modes=expected_action_modes(config, params),
            action_latency_s=float(config["oracle"]["action_latency_s"]),
            expect_breach=expect_breach,
        )
        result.update(parsed)
        return result
    except Exception as exc:
        result["error"] = repr(exc)
        result["traceback"] = traceback.format_exc()
        return result
    finally:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
        runner.stop()


def classify_runs(config: dict[str, Any], runs: list[dict[str, Any]], hard_boundary_m: float | None) -> None:
    for run in runs:
        if run.get("error"):
            run["unsafe"] = None
            run["safe"] = None
            run["hard_boundary_margin_m"] = None
            run["verdict"] = "blocked"
            continue
        if hard_boundary_m is None:
            run["unsafe"] = None
            run["safe"] = None
            run["hard_boundary_margin_m"] = None
            run["verdict"] = "unclassified"
            continue
        max_distance = float(run.get("max_distance_m", 0.0))
        unsafe = max_distance > hard_boundary_m
        run["hard_boundary_m"] = hard_boundary_m
        run["hard_boundary_margin_m"] = max_distance - hard_boundary_m
        run["unsafe"] = unsafe
        run["safe"] = not unsafe
        clean = bool(run.get("contract_clean", False))
        if clean and unsafe:
            run["verdict"] = "unsafe_contract_clean"
        elif clean and not unsafe:
            run["verdict"] = "safe_contract_clean"
        elif unsafe:
            run["verdict"] = "unsafe_contract_dirty"
        else:
            run["verdict"] = "safe_contract_dirty"


def make_plots(config: dict[str, Any], runs: list[dict[str, Any]], hard_boundary_m: float | None) -> None:
    for run in runs:
        if run.get("error") or not run.get("csv_path"):
            continue
        paths = plot_run(
            run_id=run["run_id"],
            csv_path=Path(run["csv_path"]),
            out_dir=PLANC_ROOT / "analysis",
            home=config["experiment"]["home"],
            fence_radius_m=float(config["oracle"]["fence_radius_m"]),
            hard_boundary_m=hard_boundary_m,
            modes=run.get("modes", []),
        )
        run["plots"] = paths


def summarize_gate(runs: list[dict[str, Any]], hard_boundary_m: float | None) -> dict[str, Any]:
    by_arm: dict[str, list[dict[str, Any]]] = {}
    for run in runs:
        by_arm.setdefault(run["arm"], []).append(run)
    blocked = [r for r in runs if r.get("error")]
    a_runs = by_arm.get("A_witness", [])
    a_unsafe_clean = bool(a_runs) and all(r.get("verdict") == "unsafe_contract_clean" for r in a_runs)
    a_distances = [float(r.get("max_distance_m", 0.0)) for r in a_runs if not r.get("error")]
    a_spread = max(a_distances) - min(a_distances) if len(a_distances) >= 2 else 0.0
    a_consistent = len(a_runs) == 3 and a_unsafe_clean and a_spread <= 2.0
    b_safe = all(r.get("verdict") == "safe_contract_clean" for r in by_arm.get("B_nominal", []))
    c_safe = all(r.get("verdict") == "safe_contract_clean" for r in by_arm.get("C_hover", []))
    d_safe = all(r.get("verdict") == "safe_contract_clean" for r in by_arm.get("D_conservative", []))
    if blocked:
        gate = "BLOCKED"
        reason = "One or more runs failed before a complete oracle result was produced."
    elif hard_boundary_m is None:
        gate = "BLOCKED"
        reason = "Nominal arm did not produce a hard boundary calibration."
    elif a_consistent and b_safe and c_safe and d_safe:
        gate = "GATE PASSED"
        reason = "Arm A was unsafe and contract-clean across N=3, while B/C/D were safe and contract-clean."
    else:
        gate = "GATE FAILED / NO-GO"
        dest_rejects = [
            r["run_id"]
            for r in runs
            if r.get("motion") == "witness_velocity"
            and any(e.get("ecode_name") == "DEST_OUTSIDE_FENCE" for e in r.get("other_errors", []))
        ]
        if dest_rejects:
            reason = (
                "Witness GUIDED targets were rejected by ArduPilot as DEST_OUTSIDE_FENCE before a fence "
                f"crossing occurred: {', '.join(dest_rejects)}."
            )
        else:
            reason = "The required unsafe-and-contract-clean witness with safe controls was not established."
    return {
        "gate": gate,
        "reason": reason,
        "a_consistent": a_consistent,
        "a_max_distance_spread_m": a_spread,
        "b_safe": b_safe,
        "c_safe": c_safe,
        "d_safe": d_safe,
        "blocked_runs": [r["run_id"] for r in blocked],
    }


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "n/a"
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return str(value)


def write_report(
    config: dict[str, Any],
    env: dict[str, Any],
    runs: list[dict[str, Any]],
    calibration: dict[str, Any],
    summary: dict[str, Any],
) -> None:
    report = PLANC_ROOT / "results" / "gate_report.md"
    lines: list[str] = []
    lines.append("# planc gate report: ArduPilot geofence high-speed overshoot")
    lines.append("")
    lines.append("## v1 to v2 patch")
    lines.append("")
    lines.append(
        "v1 used a GUIDED position target outside the fence, which ArduPilot rejected before motion "
        "with `NAVIGATION:DEST_OUTSIDE_FENCE`. v2 changes the witness M to a streamed GUIDED local-NED "
        "velocity setpoint. The input has no destination, so aggressiveness is in legal conditions "
        "(speed and tailwind), not in an inadmissible command."
    )
    lines.append("")
    lines.append("## planc narrative")
    lines.append("")
    lines.append(
        "`planc` is a specification-gap test: under a fixed legal flight-control configuration P, "
        "it looks for sparse feasible pilot inputs M and environment conditions E that enter an "
        "externally anchored unsafe region while the controller still satisfies its own contract. "
        "This gate is a constructed existence test, not a search algorithm."
    )
    lines.append("")
    lines.append("## gate design")
    lines.append("")
    lines.append(
        "Stack: ArduPilot native ArduCopter SITL, direct `arducopter` binary, TCP MAVLink "
        f"`{env.get('connection')}`. ArduPilot commit "
        f"`{env.get('ardupilot_commit_short')}`."
    )
    lines.append("")
    lines.append(
        "P: circular fence enabled (`FENCE_TYPE=2`), `FENCE_RADIUS=100 m`, "
        "`FENCE_ACTION=1` (RTL-and-Land), `WPNAV_SPEED=2000 cm/s` for high-speed arms. "
        "D uses `WPNAV_SPEED=500 cm/s`. All set parameters were read back and recorded per run."
    )
    lines.append("")
    lines.append(
        "E: nominal no wind for B; tailwind with `SIM_WIND_DIR=270`, `SIM_WIND_SPD=15 m/s`, "
        "`SIM_WIND_TURB=0` for A/C/D. The target bearing is 90 deg, so 270 deg is wind coming "
        "from the west and pushing along the outbound path."
    )
    lines.append("")
    lines.append(
        "M: witness arms stream one constant GUIDED local-NED velocity setpoint at 10 Hz along the "
        "outbound bearing; C streams zero velocity. The stream stops when a fence breach or configured "
        "fence action is observed."
    )
    lines.append("")
    lines.append(
        "Oracle: B calibrates `overshoot_nominal`; "
        "`hard_boundary = R + overshoot_nominal + max(0.20 * overshoot_nominal, 3 m)`. "
        "Unsafe means `max_distance > hard_boundary`. Contract-clean means the expected fence "
        "breach/action is present for witness arms and no other failsafe or contract violation is logged."
    )
    lines.append("")
    if calibration:
        lines.append("## hard-boundary calibration")
        lines.append("")
        lines.append(f"- R: {fmt(calibration.get('fence_radius_m'))} m")
        lines.append(f"- overshoot_nominal: {fmt(calibration.get('overshoot_nominal_m'))} m")
        lines.append(f"- buffer: {fmt(calibration.get('buffer_m'))} m")
        lines.append(f"- hard_boundary: {fmt(calibration.get('hard_boundary_m'))} m")
        if not calibration.get("valid_nominal_breach"):
            lines.append(
                "- calibration validity: invalid for the intended overshoot oracle because the nominal "
                "witness did not cross the fence; the value above is recorded only as the mechanical "
                "formula result."
            )
        lines.append("")
    lines.append("## results")
    lines.append("")
    lines.append("| run | arm | max distance m | max overshoot m | hard boundary m | margin m | contract clean | contract/failsafe findings | verdict |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | --- | --- | --- |")
    for run in runs:
        other = run.get("other_errors") or []
        other_text = ", ".join(
            f"{e.get('subsys_name')}:{e.get('ecode_name', e.get('ecode'))}" for e in other
        ) if other else "none"
        if run.get("error"):
            other_text = f"error: {run.get('error')}"
        lines.append(
            f"| {run.get('run_id')} | {run.get('arm')} | {fmt(run.get('max_distance_m'))} | "
            f"{fmt(run.get('max_overshoot_m'))} | {fmt(run.get('hard_boundary_m'))} | "
            f"{fmt(run.get('hard_boundary_margin_m'))} | {run.get('contract_clean')} | "
            f"{other_text} | {run.get('verdict')} |"
        )
    lines.append("")
    lines.append("## key findings")
    lines.append("")
    lines.append(f"- Gate conclusion: **{summary['gate']}**.")
    lines.append(f"- Reason: {summary['reason']}")
    lines.append(f"- Arm A N=3 consistency: {summary.get('a_consistent')} with max-distance spread {fmt(summary.get('a_max_distance_spread_m'))} m.")
    lines.append(f"- B safe: {summary.get('b_safe')}; C safe: {summary.get('c_safe')}; D safe: {summary.get('d_safe')}.")
    fallback_runs = [
        run.get("run_id")
        for run in runs
        if run.get("flight", {}).get("observed", {}).get("fallback_used")
    ]
    if fallback_runs:
        lines.append(f"- RC override fallback used in: {', '.join(str(r) for r in fallback_runs)}.")
    else:
        lines.append("- RC override fallback was not used; all witness runs used admitted GUIDED velocity setpoints.")
    lines.append("")
    lines.append("## reproducibility")
    lines.append("")
    lines.append(
        "Each run starts a fresh ArduCopter SITL process with `--wipe`, fixed home, fixed speedup, "
        "and per-run parameter readback assertions. Arm A runs three repetitions. Raw DataFlash logs, "
        "parsed CSV, parameter readbacks, and plots are saved under `planc/logs/` and `planc/analysis/`."
    )
    lines.append("")
    lines.append("## limitations")
    lines.append("")
    lines.append(
        "Geofence is close to PGFUZZ/RVFuzzer territory, so the distinction is narrow: this gate "
        "does not claim a bug when the fence action is correctly triggered. It asks whether the "
        "correct action still permits externally unsafe overshoot. SITL fidelity is limited; this "
        "gate proves only scenario existence, not generality. The hard boundary is calibrated from "
        "nominal no-wind behavior rather than chosen arbitrarily."
    )
    lines.append("")
    lines.append("## GO/NO-GO")
    lines.append("")
    if summary["gate"] == "GATE PASSED":
        lines.append(
            "**GO: GATE PASSED.** The constructed witness demonstrates unsafe and contract-clean "
            "behavior, while B/C/D remain safe. Proceed to the full planc pipeline."
        )
    elif summary["gate"] == "BLOCKED":
        lines.append(
            "**NO-GO / BLOCKED.** The experiment did not complete cleanly. The blocking error is "
            "recorded above; no fabricated results are reported."
        )
    else:
        lines.append(
            "**NO-GO: GATE FAILED.** This scenario did not establish the required unsafe and "
            "contract-clean witness with safe controls. Revisit the scenario or oracle before "
            "building the full pipeline."
        )
    lines.append("")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    config = load_config()
    env = probe_environment(config, REPO_ROOT)
    (PLANC_ROOT / "results").mkdir(parents=True, exist_ok=True)
    write_env(env, PLANC_ROOT / "results" / "env.json")

    probe = connectivity_probe(config, env)
    write_env(env, PLANC_ROOT / "results" / "env.json")
    if not probe.get("ok"):
        payload = {
            "status": "BLOCKED",
            "reason": "SITL connectivity probe failed",
            "env": env,
            "runs": [],
        }
        write_json(PLANC_ROOT / "results" / "gate_results.json", payload)
        write_report(config, env, [], {}, {"gate": "BLOCKED", "reason": payload["reason"], "a_consistent": False, "b_safe": False, "c_safe": False, "d_safe": False, "a_max_distance_spread_m": None})
        print(f"NO-GO / BLOCKED: SITL connectivity probe failed; see {PLANC_ROOT / 'results' / 'gate_report.md'}")
        return 2

    runs: list[dict[str, Any]] = []
    order = ["B_nominal", "A_witness", "C_hover", "D_conservative"]
    for arm_name in order:
        arm_cfg = config["arms"][arm_name]
        reps = int(arm_cfg.get("repetitions", 1))
        for rep in range(1, reps + 1):
            print(f"RUN {arm_name} repetition {rep}/{reps}", flush=True)
            run = run_one(config, arm_name, arm_cfg, rep)
            runs.append(run)
            write_json(PLANC_ROOT / "results" / "gate_results_partial.json", {"runs": runs})

    b_runs = [r for r in runs if r["arm"] == "B_nominal" and not r.get("error")]
    calibration: dict[str, Any] = {}
    hard_boundary_m = None
    if b_runs:
        b = b_runs[0]
        overshoot = float(b.get("max_overshoot_m", 0.0))
        buffer_m = max(float(config["oracle"]["min_buffer_m"]), float(config["oracle"]["buffer_fraction"]) * overshoot)
        hard_boundary_m = float(config["oracle"]["fence_radius_m"]) + overshoot + buffer_m
        calibration = {
            "fence_radius_m": float(config["oracle"]["fence_radius_m"]),
            "overshoot_nominal_m": overshoot,
            "buffer_m": buffer_m,
            "hard_boundary_m": hard_boundary_m,
            "source_run": b["run_id"],
            "valid_nominal_breach": bool(b.get("fence_breach_detected")),
            "source_contract_clean": bool(b.get("contract_clean")),
        }
    classify_runs(config, runs, hard_boundary_m)
    make_plots(config, runs, hard_boundary_m)
    summary = summarize_gate(runs, hard_boundary_m)
    payload = {
        "status": summary["gate"],
        "summary": summary,
        "calibration": calibration,
        "env": env,
        "runs": runs,
        "config": config,
    }
    write_json(PLANC_ROOT / "results" / "gate_results.json", payload)
    write_report(config, env, runs, calibration, summary)
    print(f"{summary['gate']}: {summary['reason']} Report: {PLANC_ROOT / 'results' / 'gate_report.md'}", flush=True)
    return 0 if summary["gate"] == "GATE PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
