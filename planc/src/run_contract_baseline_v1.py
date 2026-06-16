"""Orchestrator for the contract-testing baseline (claim C2, section 4.3).

Runs the independent PGFUZZ-style contract checker (contract_baseline_v1.py) on:
  * the OverDraw set (oracleA_v1 + ep_movement_v1) -- expect Tier-1 == 0,
  * real contract-violation positive controls -- expect Tier-1 to fire,
performs the static anti-leakage audit and a BIN-vs-CSV cross-validation, then
emits the contract-blindness comparison table, the verdict, and the result JSON.

Reads only raw telemetry through the checker; the *.oracle.json sidecars are
opened ONLY in a clearly separated ground-truth cross-reference block (never fed
to the checker).
"""
from __future__ import annotations

import glob
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
sys.path.insert(0, str(SRC_ROOT))

import contract_baseline_v1 as cb  # noqa: E402

LOGS = PLANC_ROOT / "logs"
RESULTS = PLANC_ROOT / "results"

TIER1 = [p for p in cb.POLICY_META if cb.POLICY_META[p]["tier"] == 1]
TIER2 = [p for p in cb.POLICY_META if cb.POLICY_META[p]["tier"] == 2]


# --------------------------------------------------------------------------- #
def overdraw_stems(prefix: str) -> list[Path]:
    out = []
    for csv_path in sorted(LOGS.glob(f"{prefix}_*_parsed.csv")):
        stem = csv_path.name[: -len("_parsed.csv")]
        out.append(LOGS / stem)
    return out


def aggregate(results: list[dict]) -> dict:
    n = len(results)
    t1_any = sum(1 for r in results if r["tier1_any"])
    t2_any = sum(1 for r in results if r["tier2_any"])
    per_policy = {}
    for pid in TIER1 + TIER2:
        hits = sum(1 for r in results if pid in r["tier1_hit_policies"] + r["tier2_hit_policies"])
        applicable = sum(1 for r in results if r["policies"][pid].get("applicable", True))
        per_policy[pid] = {"hits": hits, "applicable": applicable, "tier": cb.POLICY_META[pid]["tier"]}
    return {
        "n_runs": n,
        "tier1_any_hits": t1_any,
        "tier2_any_hits": t2_any,
        "per_policy": per_policy,
    }


def run_overdraw() -> dict:
    out = {}
    for label, prefix in [("oracleA_v1", "oracleAv1"), ("ep_movement_v1", "epmovev1")]:
        stems = overdraw_stems(prefix)
        results = []
        for i, stem in enumerate(stems):
            run = cb.load_run_from_csv(stem)
            results.append(cb.check_run(run))
            if (i + 1) % 100 == 0:
                print(f"  [{label}] {i+1}/{len(stems)}", flush=True)
        agg = aggregate(results)
        # offenders (any OverDraw run with a Tier-1 hit -> would mean REFUTED)
        offenders = [{"run_id": r["run_id"], "tier1": r["tier1_hit_policies"]}
                     for r in results if r["tier1_any"]]
        out[label] = {"aggregate": agg, "tier1_offenders": offenders, "results": results}
        print(f"  [{label}] n={agg['n_runs']} tier1_any={agg['tier1_any_hits']} tier2_any={agg['tier2_any_hits']}", flush=True)
    return out


def run_controls() -> dict:
    controls = {
        "PC_FENCE": {
            "bins": [LOGS / b for b in ["A_witness_r1.BIN", "A_witness_r2.BIN", "A_witness_r3.BIN",
                                        "B_nominal_r1.BIN", "grad_m02_v06_w00_r1.BIN"]],
            "expected_tier1": ["T1b_no_preventive_failsafe", "T1c_configured_limit_compliance",
                               "T1d_mode_transition_legitimacy"],
        },
        "PC_GCS": {
            "bins": [LOGS / Path(b).name for b in
                     sorted(glob.glob(str(LOGS / "linkloss_conservative_*.BIN")))[:4]],
            "expected_tier1": ["T1b_no_preventive_failsafe", "T1d_mode_transition_legitimacy"],
        },
        "PC_CLAMP": {
            "bins": [LOGS / Path(b).name for b in sorted(glob.glob(str(LOGS / "pcclampv1_*.BIN")))],
            "command_profiles": True,
            "expected_tier1": ["T1a_angle_max_command_clamp"],
        },
    }
    out = {}
    for name, spec in controls.items():
        results = []
        for binp in spec["bins"]:
            if not binp.exists():
                continue
            run = cb.load_run_from_bin(binp, angle_max_deg=45.0)
            # PC_CLAMP: attach the operator command profile + ANGLE_MAX so T1a is checkable
            if spec.get("command_profiles"):
                stem = binp.with_suffix("")
                prof_path = LOGS / f"{stem.name}_command_profile.json"
                if prof_path.exists():
                    prof = json.loads(prof_path.read_text())
                    run.command_profile = prof.get("samples")
                params = cb._params_snapshot(LOGS / f"{stem.name}_params.json")
                if "ANGLE_MAX" in params:
                    run.angle_max_deg = params["ANGLE_MAX"] / 100.0
                    run.params = params
            results.append(cb.check_run(run))
        agg = aggregate(results) if results else {"n_runs": 0, "tier1_any_hits": 0, "tier2_any_hits": 0, "per_policy": {}}
        out[name] = {
            "aggregate": agg,
            "expected_tier1": spec["expected_tier1"],
            "results": results,
            "nontrivial": agg.get("tier1_any_hits", 0) > 0,
        }
        print(f"  [{name}] n={agg['n_runs']} tier1_any={agg.get('tier1_any_hits',0)} "
              f"hit_policies={sorted({p for r in results for p in r['tier1_hit_policies']})}", flush=True)
    return out


def anti_leakage_audit() -> dict:
    import inspect

    # (1) FIELD-SET disjointness: declared Tier-1 inputs vs oracle-A consequences.
    tier1_inputs = set()
    per_policy = {}
    for pid in TIER1:
        flds = set(cb.POLICY_META[pid]["input_fields"])
        per_policy[pid] = sorted(flds)
        tier1_inputs |= flds
    consequence = set(cb.ORACLE_A_CONSEQUENCE_FIELDS)
    intersection = tier1_inputs & consequence
    field_ok = len(intersection) == 0

    # (2) VALUE-LEVEL: the shared raw fields ERR.subsystem_name / MODE.reason_name
    # / MSG.text are read by Tier-1 only for VALUES that exclude the crash /
    # ground consequence values.
    value_checks = {
        "CRASH_CHECK not a Tier-1 preventive subsystem": "CRASH_CHECK" not in cb.PREVENTIVE_FAILSAFE_SUBSYSTEMS,
        "CRASH_FAILSAFE not a Tier-1 preventive reason": "CRASH_FAILSAFE" not in cb.PREVENTIVE_MODE_REASONS,
        "no ground/crash marker in Tier-1 failsafe text": not any(
            g in m for g in ("crash", "sim hit ground", "hit ground") for m in cb.PREVENTIVE_TEXT_MARKERS),
    }
    value_ok = all(value_checks.values())

    # (3) CODE-LEVEL: the Tier-1 evaluator functions must not reference any
    # achieved-state / consequence attribute of RunData, and the checker module
    # must never open an oracle sidecar. Docstrings and comments (which mention
    # these names *to document their exclusion*) are stripped via the AST first,
    # so the scan sees executable code only -- not the documentation about it.
    import ast

    def _code_only(src: str) -> str:
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                b = getattr(node, "body", [])
                if b and isinstance(b[0], ast.Expr) and isinstance(getattr(b[0], "value", None), ast.Constant) \
                        and isinstance(b[0].value.value, str):
                    b[0].value.value = ""          # blank the docstring
        return ast.unparse(tree)                   # drops all comments

    tier1_fn_names = ["_eval_T1a", "_eval_T1b", "_eval_T1c", "_eval_T1d", "_failsafe_events"]
    forbidden_tokens = ["run.pos", "run.xkf1_vd", "run.xkf4_fs", "rel_home_alt",
                        "'roll'", '"roll"', "'pitch'", '"pitch"', "sim hit ground"]
    code_findings = {}
    for fn in tier1_fn_names:
        src = _code_only(inspect.getsource(getattr(cb, fn)))
        hits = [tok for tok in forbidden_tokens if tok in src]
        code_findings[fn] = hits
    module_code = _code_only(inspect.getsource(cb))
    opens_oracle = "oracle.json" in module_code
    code_ok = (not any(code_findings.values())) and (not opens_oracle)

    return {
        "tier1_input_fields": sorted(tier1_inputs),
        "oracle_A_consequence_fields": sorted(consequence),
        "field_set_intersection": sorted(intersection),
        "field_set_disjoint": field_ok,
        "value_level_checks": value_checks,
        "value_level_ok": value_ok,
        "code_level_forbidden_token_hits": {k: v for k, v in code_findings.items() if v},
        "code_level_opens_oracle_sidecar": opens_oracle,
        "code_level_ok": code_ok,
        "passes": field_ok and value_ok and code_ok,
        "notes": "Tier-1 reads ERR.subsystem_name / MODE.reason_name / MSG.text only for "
                 "flight-controller failsafe VALUES that exclude crash-check and ground-contact "
                 "(those are Tier-2 consequence detectors). The operator command profile is an "
                 "INPUT, not a consequence. The achieved attitude (ATT.Roll/Pitch), altitude "
                 "(POS.RelHomeAlt), descent (XKF1.VD), ground markers and the raw EKF bitmask "
                 "(XKF4.FS) appear in NO Tier-1 policy -- confirmed at field, value and code level.",
    }


def bin_cross_validation(overdraw: dict, n_per: int = 6) -> dict:
    """Re-parse a sample of OverDraw BINs directly and confirm the failsafe/mode
    Tier-1 verdict matches the CSV path -- rebuts 'the CSV laundered the data'."""
    checks = []
    mismatches = 0
    for label, prefix in [("oracleA_v1", "oracleAv1"), ("ep_movement_v1", "epmovev1")]:
        res = overdraw[label]["results"]
        sample = res[:: max(1, len(res) // n_per)][:n_per]
        for r in sample:
            stem = r["run_id"]
            binp = LOGS / f"{stem}.BIN"
            if not binp.exists():
                continue
            run_bin = cb.load_run_from_bin(binp, angle_max_deg=r.get("angle_max_deg") or 45.0)
            chk = cb.check_run(run_bin)
            # compare the failsafe/mode Tier-1 policies (T1a needs the command JSON, identical in both)
            csv_fm = sorted(p for p in r["tier1_hit_policies"] if p != "T1a_angle_max_command_clamp")
            bin_fm = sorted(p for p in chk["tier1_hit_policies"] if p != "T1a_angle_max_command_clamp")
            ok = csv_fm == bin_fm
            mismatches += 0 if ok else 1
            checks.append({"run_id": stem, "csv_tier1_fm": csv_fm, "bin_tier1_fm": bin_fm,
                           "bin_tier2": chk["tier2_hit_policies"], "match": ok})
    return {"n_checked": len(checks), "mismatches": mismatches, "passes": mismatches == 0, "checks": checks}


def oracle_a_cross_reference(overdraw: dict) -> dict:
    """GROUND-TRUTH cross-reference ONLY (not a checker input): does the
    independent Tier-2 union track the recorded oracle-A label?"""
    out = {}
    for label in ("oracleA_v1", "ep_movement_v1"):
        agree = 0
        total = 0
        unsafe_labels = Counter()
        t2_among_unsafe = 0
        for r in overdraw[label]["results"]:
            sidecar = LOGS / f"{r['run_id']}_parsed.oracle.json"
            if not sidecar.exists():
                continue
            total += 1
            try:
                oa_label = json.loads(sidecar.read_text()).get("label")
            except Exception:
                continue
            unsafe_labels[oa_label] += 1
            oa_unsafe = oa_label == "clean_unsafe"
            t2 = r["tier2_any"]
            if oa_unsafe and t2:
                t2_among_unsafe += 1
            if oa_unsafe == t2:
                agree += 1
        out[label] = {"n": total, "tier2_vs_oracleA_agreement": agree,
                      "oracle_A_label_counts": dict(unsafe_labels),
                      "tier2_fires_on_clean_unsafe": t2_among_unsafe}
    return out


def decide(overdraw: dict, controls: dict, audit: dict) -> dict:
    overdraw_tier1 = sum(overdraw[l]["aggregate"]["tier1_any_hits"] for l in overdraw)
    nontrivial_controls = [c for c, v in controls.items() if v["nontrivial"]]
    cond1 = overdraw_tier1 == 0
    cond2 = len(nontrivial_controls) >= 1
    cond3 = audit["passes"]
    if cond1 and cond2 and cond3:
        verdict = "CONFIRMED"
    elif not cond1:
        verdict = "REFUTED"
    else:
        verdict = "INCONCLUSIVE"
    return {
        "verdict": verdict,
        "overdraw_total_tier1_hits": overdraw_tier1,
        "condition_1_overdraw_tier1_zero": cond1,
        "condition_2_nontrivial_control": cond2,
        "condition_2_controls": nontrivial_controls,
        "condition_3_anti_leakage_passes": cond3,
    }


def strip_results(d: dict) -> dict:
    """Drop the heavy per-run policy detail from a dataset block for the result
    JSON top level; keep aggregate + offenders + a few examples."""
    out = {}
    for k, v in d.items():
        ex = v.get("results", [])[:3]
        out[k] = {kk: vv for kk, vv in v.items() if kk != "results"}
        out[k]["example_runs"] = ex
    return out


def main() -> None:
    import pickle

    cache = RESULTS / ".contract_baseline_overdraw_cache.pkl"
    use_cache = "--use-cache" in sys.argv and cache.exists()
    print("== OverDraw set ==", flush=True)
    if use_cache:
        overdraw = pickle.loads(cache.read_bytes())
        for l in overdraw:
            a = overdraw[l]["aggregate"]
            print(f"  [{l}] (cached) n={a['n_runs']} tier1_any={a['tier1_any_hits']} tier2_any={a['tier2_any_hits']}", flush=True)
    else:
        overdraw = run_overdraw()
        cache.write_bytes(pickle.dumps(overdraw))
    print("== Positive controls ==", flush=True)
    controls = run_controls()
    print("== Anti-leakage audit ==", flush=True)
    audit = anti_leakage_audit()
    print(f"  passes={audit['passes']} field_set_disjoint={audit['field_set_disjoint']} "
          f"value_ok={audit['value_level_ok']} code_ok={audit['code_level_ok']} "
          f"intersection={audit['field_set_intersection']}", flush=True)
    print("== BIN cross-validation ==", flush=True)
    xval = bin_cross_validation(overdraw)
    print(f"  checked={xval['n_checked']} mismatches={xval['mismatches']}", flush=True)
    print("== oracle-A cross-reference (ground truth, not a checker input) ==", flush=True)
    oa_xref = oracle_a_cross_reference(overdraw)
    verdict = decide(overdraw, controls, audit)
    print(f"== VERDICT: {verdict['verdict']} ==", flush=True)

    payload = {
        "experiment": "contract_testing_baseline_v1",
        "claim": "C2",
        "verdict": verdict,
        "policy_meta": cb.POLICY_META,
        "anti_leakage_audit": audit,
        "bin_cross_validation": xval,
        "oracle_A_cross_reference": oa_xref,
        "overdraw": strip_results(overdraw),
        "positive_controls": controls,
    }
    out_path = RESULTS / "contract_baseline_result.json"
    out_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(f"wrote {out_path}", flush=True)


if __name__ == "__main__":
    main()
