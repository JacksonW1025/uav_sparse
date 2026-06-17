"""Tier B/C v2 analysis + report: dual VERDICT, interface-aware contract check, plots.

Decoupled from the SITL campaign (run_tierBC_v2.py) so it can be re-run on the
cached partial without re-flying. Consumes planc/results/tierBC_v2_partial.json,
runs the interface-aware Tier-1/Tier-2 contract checker (contract_baseline_v1)
over every run, performs the static anti-leakage audit, decides B and C, and
emits tierBC_v2_result.json + tierBC_v2_report.md + plots.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
sys.path.insert(0, str(SRC_ROOT))

import contract_baseline_v1 as cb  # noqa: E402

LOGS = PLANC_ROOT / "logs"
RESULTS = PLANC_ROOT / "results"
STEM = "tierBC_v2"
ROBUST_MIN = 2
ANGLE_MAX_DEG = 45.0


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def fmt(v: Any, d: int = 2) -> str:
    if v is None:
        return "n/a"
    try:
        return f"{float(v):.{d}f}"
    except (TypeError, ValueError):
        return str(v)


# --------------------------------------------------------------------------- #
# Interface-aware contract check over every cached run
# --------------------------------------------------------------------------- #
def contract_check_all(runs: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for run in runs:
        if run.get("error"):
            continue
        run_id = run["run_id"]
        stem = LOGS / run_id
        if not (LOGS / f"{run_id}_parsed.csv").exists():
            continue
        interface = str(run.get("interface", "guided_quaternion"))
        try:
            rd = cb.load_run_from_csv(stem)
            out[run_id] = cb.check_run(rd, interface=interface)
        except Exception as exc:  # noqa: BLE001
            out[run_id] = {"run_id": run_id, "error": repr(exc)}
    return out


def anti_leakage_audit() -> dict[str, Any]:
    """Tier-1 input fields (with T1a treated as N/A on the attitude interface)
    must be DISJOINT from the oracle-A consequence field set."""
    consequence = set(cb.ORACLE_A_CONSEQUENCE_FIELDS)
    rows = []
    leak = False
    for pid, meta in cb.POLICY_META.items():
        if meta["tier"] != 1:
            continue
        fields = set(meta.get("input_fields", []))
        # T1a is N/A on the attitude-target interface; report it but it does not
        # gate Tier-1 there.
        overlap = sorted(fields & consequence) if pid != cb.T1A_POLICY else []
        if overlap:
            leak = True
        rows.append({"policy": pid, "input_fields": sorted(fields), "overlap_with_consequence": overlap,
                     "in_scope_on_attitude_interface": pid != cb.T1A_POLICY})
    return {"leakage_detected": leak, "consequence_fields": sorted(consequence), "tier1_policies": rows}


# --------------------------------------------------------------------------- #
# Per-run feature extraction from the cached parse + interface label
# --------------------------------------------------------------------------- #
def run_feature(run: dict[str, Any], contract: dict[str, Any]) -> dict[str, Any]:
    pt = run.get("point", {})
    lbl = run.get("interface_label", {})
    ha = run.get("hardened_oracle_A_v2") or run.get("hardened_oracle_A", {})
    ob = run.get("oracle_B", {})
    cmd = run.get("command", {})
    ext = run.get("attitude_extrema", {})
    c = contract.get(run["run_id"], {})
    causes = ha.get("causes") or {}
    tier2_v2 = []
    if causes.get("attitude_diverged"):
        tier2_v2.append("T2a_attitude_diverged_actual_vs_desired")
    if causes.get("altitude_loss"):
        tier2_v2.append("T2b_altitude_loss")
    if causes.get("crash_or_contact"):
        tier2_v2.append("T2c_ground_contact")
    return {
        "run_id": run["run_id"],
        "part": pt.get("part"),
        "interface": run.get("interface"),
        "group": pt.get("group"),
        "altitude_hold": pt.get("altitude_hold"),
        "suspect": bool(pt.get("suspect", False)),
        "guid_options": pt.get("guid_options"),
        "target_deg": pt.get("target_deg"),
        "wind_m_s": pt.get("wind_m_s"),
        "turbulence_m_s": pt.get("turbulence_m_s"),
        "seed": pt.get("seed"),
        "pressure_name": pt.get("name"),
        "label": lbl.get("label"),
        "hard_A": bool(ha.get("inside")),
        "outcome": ha.get("outcome"),
        "iface_B": bool(lbl.get("interface_aware_B")),
        "command_peak_deg": cmd.get("max_command_angle_deg"),
        "command_exceeds_angle_max": bool(cmd.get("command_exceeded_angle_max")),
        "command_le_angle_max": bool(lbl.get("command_le_angle_max")),
        "desroll_le_angle_max": bool(lbl.get("desroll_le_angle_max")),
        "desroll_peak_deg": lbl.get("desroll_peak_deg"),
        "desroll_crossed_before_hard_A": bool(lbl.get("desroll_crossed_before_hard_A")),
        "demanded_peak_deg": ext.get("max_demanded_lean_deg"),
        "achieved_peak_deg": ext.get("max_achieved_lean_deg"),
        "demanded_exceeds_angle_max": bool(lbl.get("demanded_target_exceeds_angle_max")),
        "max_att_error_deg": ha.get("max_attitude_error_deg"),
        "altitude_loss_m": (ha.get("altitude") or {}).get("loss_m"),
        "altitude_min_m": (ha.get("altitude") or {}).get("min_m"),
        "tier1_hits": c.get("tier1_hit_policies", []),
        "tier2_hits": tier2_v2,
        "contract_checker_tier2_hits": c.get("tier2_hit_policies", []),
        "tier1_any": bool(c.get("tier1_any")),
        "tier2_any": bool(tier2_v2),
        "contract_checker_tier2_any": bool(c.get("tier2_any")),
        "thrust_used": run.get("thrust_used"),
    }


# --------------------------------------------------------------------------- #
# Verdicts
# --------------------------------------------------------------------------- #
def decide_partB(feats: list[dict[str, Any]]) -> dict[str, Any]:
    b = [f for f in feats if f["part"] == "B"]
    cells: dict[float, list[dict[str, Any]]] = defaultdict(list)
    for f in b:
        cells[float(f["target_deg"])].append(f)
    cell_rows = []
    robust_clean_cells = []
    for tgt, rows in sorted(cells.items()):
        clean = [r for r in rows if r["label"] == "B_reanchor_clean"]
        demanded_ok = sum(1 for r in rows if r["demanded_exceeds_angle_max"])
        crossed_before_hard = sum(1 for r in rows if r["desroll_crossed_before_hard_A"])
        tier1_clean = sum(1 for r in rows if not r["tier1_any"])
        row = {
            "target_deg": tgt, "n": len(rows),
            "reanchor_clean": len(clean),
            "blocked_by_failsafe": sum(1 for r in rows if r["label"] == "B_blocked_by_failsafe"),
            "no_consequence": sum(1 for r in rows if r["label"] == "B_no_consequence"),
            "demanded_exceeds_angle_max": demanded_ok,
            "desroll_crossed_before_hard_A": crossed_before_hard,
            "tier1_clean": tier1_clean,
            "max_demanded_peak_deg": max((r["demanded_peak_deg"] or 0.0) for r in rows),
            "outcomes": dict(Counter(r["outcome"] for r in rows)),
        }
        cell_rows.append(row)
        if len(clean) >= ROBUST_MIN and demanded_ok >= ROBUST_MIN and crossed_before_hard >= ROBUST_MIN and tier1_clean == len(rows):
            robust_clean_cells.append(tgt)
    verdict = "RE-ANCHORED" if robust_clean_cells else "FAILED"
    return {"verdict": verdict, "robust_clean_cells_deg": robust_clean_cells, "cells": cell_rows, "n": len(b)}


def decide_partC(feats: list[dict[str, Any]]) -> dict[str, Any]:
    c = [f for f in feats if f["part"] == "C"]
    cells: dict[tuple, list[dict[str, Any]]] = defaultdict(list)
    for f in c:
        cells[(str(f.get("group")), float(f["target_deg"]), f["pressure_name"])].append(f)
    cell_rows = []
    c_attitude_cells = []
    c_altitude_cells = []
    any_command_over = False
    for (group, tgt, pname), rows in sorted(cells.items(), key=lambda kv: (str(kv[0][0]), kv[0][1], str(kv[0][2]))):
        labels = Counter(r["label"] for r in rows)
        if any(not r["desroll_le_angle_max"] for r in rows):
            any_command_over = True
        row = {
            "group": group,
            "altitude_hold": rows[0].get("altitude_hold"),
            "suspect": any(r.get("suspect") for r in rows),
            "target_deg": tgt, "pressure": pname,
            "wind_m_s": rows[0]["wind_m_s"], "turbulence_m_s": rows[0]["turbulence_m_s"],
            "n": len(rows),
            "C_attitude": labels.get("C_attitude", 0),
            "C_altitude": labels.get("C_altitude", 0),
            "recovered": labels.get("recovered", 0),
            "safe": labels.get("safe", 0),
            "blocked_by_failsafe": labels.get("C_blocked_by_failsafe", 0),
            "invalid": labels.get("C_invalid_desroll_over_limit", 0),
            "max_att_error_deg": max((r["max_att_error_deg"] or 0.0) for r in rows),
            "max_altitude_loss_m": max((r["altitude_loss_m"] or 0.0) for r in rows),
            "max_desroll_deg": max((r["desroll_peak_deg"] or 0.0) for r in rows),
            "all_desroll_le_angle_max": all(r["desroll_le_angle_max"] for r in rows),
        }
        cell_rows.append(row)
        if group == "C_attitude" and labels.get("C_attitude", 0) >= ROBUST_MIN:
            c_attitude_cells.append((tgt, pname))
        if labels.get("C_altitude", 0) >= ROBUST_MIN:
            c_altitude_cells.append((group, tgt, pname))
    if c_attitude_cells:
        verdict = "C-ATTITUDE"
    elif c_altitude_cells:
        verdict = "C-ALTITUDE"
    else:
        verdict = "C-ABSENT"
    return {
        "verdict": verdict,
        "c_attitude_cells": c_attitude_cells,
        "c_altitude_cells": c_altitude_cells,
        "command_over_limit_detected": any_command_over,
        "cells": cell_rows,
        "n": len(c),
    }


# --------------------------------------------------------------------------- #
# Plots
# --------------------------------------------------------------------------- #
def read_trajectory(run_id: str) -> dict[str, list[float]]:
    csv_path = LOGS / f"{run_id}_parsed.csv"
    t0 = None
    ts, roll, desroll, alt = [], [], [], []
    if not csv_path.exists():
        return {"t": [], "roll": [], "desroll": [], "alt": []}
    att_rows = []
    pos_rows = []
    with csv_path.open() as fh:
        for row in csv.DictReader(fh):
            try:
                t = float(row.get("time_s"))
            except (TypeError, ValueError):
                continue
            if row.get("type") == "ATT":
                att_rows.append((t, float(row.get("Roll", 0) or 0), float(row.get("DesRoll", 0) or 0)))
            elif row.get("type") == "POS" and row.get("RelHomeAlt") not in (None, ""):
                pos_rows.append((t, float(row.get("RelHomeAlt"))))
    if not att_rows:
        return {"t": [], "roll": [], "desroll": [], "alt": []}
    t0 = att_rows[0][0]

    def nearest_alt(t):
        if not pos_rows:
            return None
        return min(pos_rows, key=lambda r: abs(r[0] - t))[1]

    for t, r, dr in att_rows:
        ts.append(t - t0)
        roll.append(r)
        desroll.append(dr)
        alt.append(nearest_alt(t))
    return {"t": ts, "roll": roll, "desroll": desroll, "alt": alt}


def make_plots(feats: list[dict[str, Any]], partB: dict, partC: dict) -> dict[str, str]:
    analysis = PLANC_ROOT / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    paths: dict[str, str] = {}
    colors = {"crashed": "#e76f51", "diverged": "#9b2226", "attitude_diverged": "#9b2226", "altitude_loss": "#f4a261",
              "recovered": "#e9c46a", "safe": "#2a9d8f"}

    # --- Part B: demanded attitude target peak vs ANGLE_MAX, by command target
    b = [f for f in feats if f["part"] == "B"]
    if b:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for f in b:
            ax.scatter(f["target_deg"], f["demanded_peak_deg"], s=45,
                       color=colors.get(f["outcome"], "#777"),
                       edgecolor="k" if f["label"] == "B_reanchor_clean" else "none", linewidth=0.8)
        ax.axhline(ANGLE_MAX_DEG, color="k", ls="--", lw=1.2, label=f"ANGLE_MAX = {ANGLE_MAX_DEG:.0f} deg")
        ax.plot([55, 155], [55, 155], color="#aaa", ls=":", lw=1, label="demanded = commanded")
        ax.set_xlabel("commanded attitude target (deg)")
        ax.set_ylabel("peak DEMANDED attitude target ATT.DesRoll/Pitch (deg)")
        ax.set_title("Part B (supported quaternion interface): demanded attitude exceeds ANGLE_MAX (no clamp)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.25)
        p = analysis / f"{STEM}_partB_demanded_vs_anglemax.png"
        fig.tight_layout(); fig.savefig(p, dpi=150); plt.close(fig)
        paths["partB_demanded_vs_anglemax"] = str(p)

        # representative Part B trajectory
        clean = [f for f in b if f["label"] == "B_reanchor_clean"]
        rep = (clean or b)[0]
        tr = read_trajectory(rep["run_id"])
        if tr["t"]:
            fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
            axes[0].plot(tr["t"], tr["desroll"], color="#e76f51", label="DesRoll (demanded)")
            axes[0].plot(tr["t"], tr["roll"], color="#264653", label="Roll (achieved)", alpha=0.8)
            axes[0].axhline(ANGLE_MAX_DEG, color="k", ls="--", lw=1); axes[0].axhline(-ANGLE_MAX_DEG, color="k", ls="--", lw=1)
            axes[0].set_ylabel("roll (deg)"); axes[0].legend(fontsize=8)
            axes[1].plot(tr["t"], tr["alt"], color="#2a9d8f"); axes[1].set_ylabel("RelHomeAlt (m)")
            axes[1].set_xlabel("s from maneuver start")
            axes[0].set_title(f"Part B representative: {rep['run_id']} ({rep['outcome']})")
            for a in axes: a.grid(True, alpha=0.25)
            p = analysis / f"{STEM}_partB_trajectory.png"
            fig.tight_layout(); fig.savefig(p, dpi=150); plt.close(fig)
            paths["partB_trajectory"] = str(p)

    # --- Part C: outcome by command angle and wind
    c = [f for f in feats if f["part"] == "C"]
    if c:
        fig, ax = plt.subplots(figsize=(8.5, 4.5))
        label_y = {"safe": 0, "recovered": 1, "C_altitude": 2, "C_attitude": 3, "C_blocked_by_failsafe": 4,
                   "C_invalid_desroll_over_limit": 5}
        for f in c:
            jitter = 0.06 * (float(f["seed"] or 0) - 1)
            group_offset = -0.12 if f.get("group") == "C_attitude" else 0.12
            ax.scatter(f["wind_m_s"] + jitter + group_offset, label_y.get(f["label"], 0) + 0.08 * (float(f["target_deg"]) - 40),
                       s=40, color=colors.get(f["outcome"], "#777"))
        ax.set_yticks(list(label_y.values())); ax.set_yticklabels(list(label_y.keys()))
        ax.set_xlabel("wind (m/s)"); ax.set_title("Part C (DesRoll <= ANGLE_MAX): outcome vs wind")
        ax.grid(True, alpha=0.25)
        p = analysis / f"{STEM}_partC_outcome_vs_wind.png"
        fig.tight_layout(); fig.savefig(p, dpi=150); plt.close(fig)
        paths["partC_outcome_vs_wind"] = str(p)

        fig, ax = plt.subplots(figsize=(8.5, 4.5))
        for group, tgt in sorted({(f["group"], f["target_deg"]) for f in c}):
            sub = sorted([f for f in c if f["target_deg"] == tgt and f["group"] == group], key=lambda r: r["wind_m_s"])
            ax.plot([f["wind_m_s"] for f in sub], [f["altitude_loss_m"] or 0 for f in sub],
                    marker="o", label=f"{group} cmd {tgt:.0f} deg")
        ax.axhline(15.0, color="k", ls="--", lw=1, label="altitude-loss oracle 15 m")
        ax.set_xlabel("wind (m/s)"); ax.set_ylabel("altitude loss (m)")
        ax.set_title("Part C: altitude loss vs wind (DesRoll <= ANGLE_MAX)")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.25)
        p = analysis / f"{STEM}_partC_altitude_loss.png"
        fig.tight_layout(); fig.savefig(p, dpi=150); plt.close(fig)
        paths["partC_altitude_loss"] = str(p)

        rep_c = ([f for f in c if f["label"].startswith("C_a")] or c)[0]
        tr = read_trajectory(rep_c["run_id"])
        if tr["t"]:
            fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
            axes[0].plot(tr["t"], tr["desroll"], color="#e76f51", label="DesRoll (demanded)")
            axes[0].plot(tr["t"], tr["roll"], color="#264653", label="Roll (achieved)", alpha=0.8)
            axes[0].axhline(ANGLE_MAX_DEG, color="k", ls="--", lw=1)
            axes[0].set_ylabel("roll (deg)"); axes[0].legend(fontsize=8)
            axes[1].plot(tr["t"], tr["alt"], color="#2a9d8f"); axes[1].set_ylabel("RelHomeAlt (m)")
            axes[1].set_xlabel("s from maneuver start")
            axes[0].set_title(f"Part C representative: {rep_c['run_id']} ({rep_c['label']})")
            for a in axes: a.grid(True, alpha=0.25)
            p = analysis / f"{STEM}_partC_trajectory.png"
            fig.tight_layout(); fig.savefig(p, dpi=150); plt.close(fig)
            paths["partC_trajectory"] = str(p)

    # --- Tier-1 / Tier-2 decomposition
    fig, ax = plt.subplots(figsize=(8, 4.5))
    parts = ["B", "C"]
    t1 = [sum(1 for f in feats if f["part"] == pp and f["tier1_any"]) for pp in parts]
    t2 = [sum(1 for f in feats if f["part"] == pp and f["tier2_any"]) for pp in parts]
    ntot = [sum(1 for f in feats if f["part"] == pp) for pp in parts]
    x = range(len(parts))
    ax.bar([i - 0.2 for i in x], t1, width=0.4, label="Tier-1 (contract) hits", color="#6d597a")
    ax.bar([i + 0.2 for i in x], t2, width=0.4, label="Tier-2 (result/SOTIF) hits", color="#e76f51")
    for i, n in enumerate(ntot):
        ax.text(i, max(t1 + t2 + [1]) * 0.92, f"n={n}", ha="center", fontsize=9)
    ax.set_xticks(list(x)); ax.set_xticklabels([f"Part {p}" for p in parts])
    ax.set_ylabel("runs with >=1 hit")
    ax.set_title("Interface-aware contract decomposition: Tier-1 clean, Tier-2 fires")
    ax.legend(fontsize=8); ax.grid(True, axis="y", alpha=0.25)
    p = analysis / f"{STEM}_tier_decomposition.png"
    fig.tight_layout(); fig.savefig(p, dpi=150); plt.close(fig)
    paths["tier_decomposition"] = str(p)
    return paths


# --------------------------------------------------------------------------- #
# Report
# --------------------------------------------------------------------------- #
def build_report(payload: dict[str, Any]) -> str:
    pb = payload["partB"]; pc = payload["partC"]
    feats = payload["features"]
    s1 = payload["S1_fidelity"]
    lines = [
        f"# Control-Authority Tier B Re-anchor + Tier C Verification (v2)",
        "",
        f"**VERDICT: B={pb['verdict']}; C={pc['verdict']}**",
        "",
        payload["headline"],
        "",
        "## Interface (the re-anchor)",
        "",
        "- Part B/C ride the **supported** GUIDED `SET_ATTITUDE_TARGET` path: `type_mask=0x07` "
        "(ignore all three body-rate fields) + a **unit attitude quaternion**. A non-zero quaternion "
        "routes (`mode_guided.cpp:964-967`) to `AC_AttitudeControl::input_quaternion` "
        "(`AC_AttitudeControl.cpp:231-266`), which applies only `ang_vel_limit` -- **no ANGLE_MAX clamp**. "
        "This is exactly the mask ArduPilot's GUIDED dev docs prescribe; the predecessor oracleA_v1 used "
        "the doc-discouraged body-rate fields.",
        "- Part C is split by vertical authority. `C_attitude` uses `GUID_OPTIONS=0`, so `thrust=0.5` is zero "
        "climb-rate and the z-controller holds altitude from a 90 m start. `C_altitude` uses `GUID_OPTIONS=8` "
        "(direct throttle) and is reported as the suspect secondary group.",
        "- Interface-aware **B**: command attitude > ANGLE_MAX is **not** a contract clamp on this interface "
        "(by design, angle_max_scope_v1) -- scored under Tier-2. `CRASH_FAILSAFE`/`CRASH_CHECK` are post-impact "
        "consequence detectors, excluded from preventive B. So B = a genuine preventive failsafe before the hard "
        "consequence.",
        "",
        "## S1 interface fidelity",
        "",
        "| probe | group | commanded | DesRoll peak | demanded lean peak | achieved peak | DesRoll valid? | outcome |",
        "| --- | --- | ---: | ---: | ---: | ---: | :--: | --- |",
    ]
    for f in s1:
        lines.append(
            f"| {f['run_id']} | {f.get('group')} | {fmt(f['target_deg'],0)} | {fmt(f.get('desroll_peak_deg'))} | "
            f"{fmt(f['demanded_peak_deg'])} | {fmt(f['achieved_peak_deg'])} | "
            f"{f.get('desroll_le_angle_max')} | {f['outcome']} |"
        )
    lines += [
        "",
        f"- S1 gate: `{payload.get('fidelity_status', {}).get('status', 'not recorded')}`.",
        f"- ACRO_TRAINER=0 corroboration: {payload['acro']['status']}"
        + (f" (achieved roll peak {fmt(payload['acro'].get('achieved_peak_deg'))} deg, exceeds ANGLE_MAX = {payload['acro'].get('exceeds_angle_max')})" if payload['acro'].get('ran') else ""),
        f"- Legal-wind upper-bound check (0 deg level hover at {fmt(payload['wind_bound'].get('wind_m_s'),0)} m/s wind / "
        f"{fmt(payload['wind_bound'].get('turbulence_m_s'),0)} m/s turb): outcome `{payload['wind_bound'].get('outcome')}` "
        f"-> {'within envelope (used as high pressure)' if payload['wind_bound'].get('safe') else 'OUT OF ENVELOPE (backed off)'}.",
        "",
        "## Part B re-anchor (command > ANGLE_MAX, supported interface)",
        "",
        f"Verdict **B = {pb['verdict']}**. Robust clean cells (deg): `{pb['robust_clean_cells_deg']}`.",
        "",
        "| cmd target (deg) | n | reanchor_clean | DesRoll>ANGLE_MAX | DesRoll crossed before hard A | Tier-1 clean | blocked_by_FS | no_consequence | max demanded peak (deg) | outcomes |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in pb["cells"]:
        lines.append(
            f"| {fmt(row['target_deg'],0)} | {row['n']} | {row['reanchor_clean']} | {row['demanded_exceeds_angle_max']} | "
            f"{row['desroll_crossed_before_hard_A']} | {row['tier1_clean']} | {row['blocked_by_failsafe']} | "
            f"{row['no_consequence']} | {fmt(row['max_demanded_peak_deg'])} | "
            f"`{json.dumps(row['outcomes'], sort_keys=True)}` |"
        )
    lines += [
        "",
        "## Part C verification (ATT.DesRoll <= ANGLE_MAX throughout + legal wind/turbulence)",
        "",
        f"Verdict **C = {pc['verdict']}**. DesRoll-over-limit anywhere: `{pc['command_over_limit_detected']}` (must be false for valid C cells). "
        f"C-attitude cells: `{pc['c_attitude_cells']}`; C-altitude cells: `{pc['c_altitude_cells']}`.",
        "",
        "| group | cmd (deg) | pressure | wind | turb | n | C-attitude | C-altitude | recovered | safe | blocked_FS | invalid | DesRoll<=AMAX | max DesRoll | max att err | max alt loss |",
        "| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | :--: | ---: | ---: | ---: |",
    ]
    for row in pc["cells"]:
        lines.append(
            f"| {row['group']} | {fmt(row['target_deg'],0)} | {row['pressure']} | {fmt(row['wind_m_s'],0)} | {fmt(row['turbulence_m_s'],0)} | "
            f"{row['n']} | {row['C_attitude']} | {row['C_altitude']} | {row['recovered']} | {row['safe']} | "
            f"{row['blocked_by_failsafe']} | {row['invalid']} | {row['all_desroll_le_angle_max']} | "
            f"{fmt(row['max_desroll_deg'])} | {fmt(row['max_att_error_deg'])} | {fmt(row['max_altitude_loss_m'])} |"
        )
    al = payload["anti_leakage"]
    lines += [
        "",
        "## Interface-aware contract cleanliness (Tier-1 / Tier-2)",
        "",
        f"- Anti-leakage audit: Tier-1 input fields disjoint from oracle-A consequence set -> "
        f"leakage_detected = **{al['leakage_detected']}** (T1a is N/A on the attitude interface and excluded).",
        f"- Part B: Tier-1 (contract) hits = {payload['contract_summary']['B']['tier1_any']}/{payload['contract_summary']['B']['n']}; "
        f"Tier-2 (result) hits = {payload['contract_summary']['B']['tier2_any']}/{payload['contract_summary']['B']['n']}.",
        f"- Part C: Tier-1 (contract) hits = {payload['contract_summary']['C']['tier1_any']}/{payload['contract_summary']['C']['n']}; "
        f"Tier-2 (result) hits = {payload['contract_summary']['C']['tier2_any']}/{payload['contract_summary']['C']['n']}.",
        "- Tier-1 policies scored on the attitude interface: T1b (no preventive failsafe), T1c (configured-limit "
        "compliance), T1d (mode-transition legitimacy). T1a (ANGLE_MAX command clamp) is reported **not-applicable**.",
        "",
        "## Artifacts",
        "",
        f"- Preregistration: `planc/results/{STEM}_prereg.json`",
        f"- Result JSON: `planc/results/{STEM}_result.json`",
    ]
    for name, p in payload.get("plots", {}).items():
        try:
            rel = Path(p).relative_to(REPO_ROOT)
        except ValueError:
            rel = Path(p)
        lines.append(f"- Plot {name}: `{rel}`")
    report_path = RESULTS / f"{STEM}_report.md"
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(report_path)


def headline(pb: dict, pc: dict) -> str:
    if pb["verdict"] == "RE-ANCHORED":
        b = ("On the documented-supported GUIDED quaternion interface, a commanded attitude target > ANGLE_MAX is "
             "accepted unclamped (demanded attitude exceeds ANGLE_MAX), produces a real hard consequence, and the "
             "flight controller raises no preventive failsafe (interface-aware Tier-1 = 0). The contract-clean unsafe "
             "zone re-anchors on solid ground.")
    else:
        b = ("The supported quaternion interface did not reproduce a contract-clean hard consequence; see the Part B table.")
    if pc["verdict"] == "C-ATTITUDE":
        c = ("Part C ACE PRESENT (strongest): in the altitude-hold C_attitude group, ATT.DesRoll stayed <= ANGLE_MAX "
             "while legal wind/turbulence drove actual-vs-demanded attitude divergence with B = 0.")
    elif pc["verdict"] == "C-ALTITUDE":
        c = ("Part C present but weaker: ATT.DesRoll stayed <= ANGLE_MAX, attitude did not diverge first, and the "
             "hard consequence was altitude loss / ground contact with B = 0. Treat as the secondary altitude-outcome claim.")
    else:
        c = ("Part C ABSENT: no run with ATT.DesRoll <= ANGLE_MAX under the legal wind/turbulence upper bound produced an unrecovered hard "
             "consequence with B = 0 in SITL; the demonstration rests on Tier B.")
    return b + " " + c


def main() -> None:
    partial = load_json(RESULTS / f"{STEM}_partial.json", {"runs": []})
    runs = partial.get("runs", [])
    fidelity_gate = partial.get("fidelity_status", {})
    contract = contract_check_all(runs)
    feats_all = [run_feature(r, contract) for r in runs if not r.get("error")]
    feats = [f for f in feats_all if f["part"] in ("B", "C")]

    # S1 fidelity rows
    s1 = []
    for r in runs:
        pt = r.get("point", {})
        if pt.get("part") == "fid" and pt.get("name", "").startswith(("fidB", "fidC")):
            s1.append(run_feature(r, contract))
    # wind bound
    wind_bound = {"outcome": None, "safe": None}
    for r in runs:
        pt = r.get("point", {})
        if pt.get("name", "").startswith("fidwind"):
            ha = r.get("hardened_oracle_A_v2") or r.get("hardened_oracle_A", {})
            wind_bound = {"wind_m_s": pt.get("wind_m_s"), "turbulence_m_s": pt.get("turbulence_m_s"),
                          "outcome": ha.get("outcome"), "safe": not bool(ha.get("inside"))}
    # acro
    acro = {"ran": False, "status": "not run (corroboration optional; ACRO by-design established in angle_max_scope_v1 from code+docs)"}
    for r in runs:
        if r.get("interface") == "acro_rate":
            if r.get("error"):
                acro = {"ran": True, "status": f"attempted, errored: {r.get('error')}", "exceeds_angle_max": None}
            else:
                ext = r.get("attitude_extrema", {})
                acro = {"ran": True, "status": "ran", "achieved_peak_deg": ext.get("max_achieved_lean_deg"),
                        "exceeds_angle_max": r.get("acro_attitude_exceeds_angle_max")}

    partB = decide_partB(feats)
    partC = decide_partC(feats)
    al = anti_leakage_audit()

    def csum(part):
        sub = [f for f in feats if f["part"] == part]
        return {"n": len(sub), "tier1_any": sum(1 for f in sub if f["tier1_any"]), "tier2_any": sum(1 for f in sub if f["tier2_any"])}

    payload = {
        "status": "complete",
        "generated_at_utc": utc_now(),
        "stem": STEM,
        "headline": headline(partB, partC),
        "verdict": {"B": partB["verdict"], "C": partC["verdict"]},
        "S1_fidelity": s1,
        "fidelity_status": fidelity_gate,
        "wind_bound": wind_bound,
        "acro": acro,
        "partB": partB,
        "partC": partC,
        "anti_leakage": al,
        "contract_summary": {"B": csum("B"), "C": csum("C")},
        "features": feats_all,
        "contract_per_run": contract,
        "n_runs_total": len(runs),
        "n_runs_error": sum(1 for r in runs if r.get("error")),
    }
    payload["plots"] = make_plots(feats, partB, partC)
    payload["report"] = build_report(payload)
    write_json(RESULTS / f"{STEM}_result.json", payload)
    print(f"VERDICT B={partB['verdict']}; C={partC['verdict']}", flush=True)
    print(f"report={payload['report']}", flush=True)


if __name__ == "__main__":
    main()
