from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import pandas as pd

from cadet.runners.direction_a_probe import derive_A_phi
from cadet.runners.residual_rate_phase0 import TARGET_PROPERTIES, _property_label


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize confirmatory PX4 residual-rate Phase 1 go/no-go.")
    parser.add_argument("--artifacts-dir", default="artifacts")
    parser.add_argument("--summary-dir", default="artifacts/residual_rate_summary")
    args = parser.parse_args()

    artifacts_dir = Path(args.artifacts_dir)
    summary_dir = Path(args.summary_dir)
    summary_dir.mkdir(parents=True, exist_ok=True)

    property_blocks: list[dict[str, Any]] = []
    channel_mass_rows: list[dict[str, Any]] = []
    arm_purity_tier1_rows: list[dict[str, Any]] = []
    arm_purity_tier2_rows: list[dict[str, Any]] = []
    signature_rows: list[dict[str, Any]] = []
    distinct_cost_rows: list[dict[str, Any]] = []

    for property_name in TARGET_PROPERTIES:
        label = _property_label(property_name)
        run_dir = artifacts_dir / f"residual_rate_{label}_seed0_v0"
        reports_dir = run_dir / "reports"
        phase0 = _read_json(reports_dir / "phase0_sanity_summary.json")
        allowed = set(derive_A_phi(property_name))
        h1_path = reports_dir / "h1_summary.json"
        point_path = reports_dir / "point_evaluations.csv"

        if not h1_path.exists() or not point_path.exists():
            channel_mass_rows.append(
                {
                    "property": property_name,
                    "seed": 0,
                    "channel": ",".join(sorted(allowed)),
                    "weight": math.nan,
                    "share": math.nan,
                    "status": "not_run_phase0_stop",
                }
            )
            tier1 = _not_run_arm_purity(property_name, allowed, "Tier 1")
            tier2 = _not_run_arm_purity(property_name, allowed, "Tier 2")
            arm_purity_tier1_rows.extend(tier1)
            arm_purity_tier2_rows.extend(tier2)
            distinct_cost_rows.extend(_not_run_distinct_costs(property_name, "Tier 1"))
            distinct_cost_rows.extend(_not_run_distinct_costs(property_name, "Tier 2"))
            h1_decision = "not_run_phase0_stop"
            h2_tier1 = "not_run_phase0_stop"
            h2_tier2 = "not_run_phase0_stop"
            property_blocks.append(
                {
                    "property": property_name,
                    "label": label,
                    "run_dir": str(run_dir),
                    "phase0": phase0,
                    "h1": {},
                    "h1_decision": h1_decision,
                    "h2_tier1_decision": h2_tier1,
                    "h2_tier2_decision": h2_tier2,
                    "go_no_go": "no_go_phase2",
                    "judgment": _property_judgment(property_name, phase0, h1_decision, h2_tier1, h2_tier2, tier1, tier2),
                }
            )
            continue

        h1 = _read_json(h1_path)
        point_df = pd.read_csv(point_path)

        h1_channel_mass = pd.read_csv(reports_dir / "h1_channel_mass.csv")
        for row in h1_channel_mass.to_dict(orient="records"):
            channel_mass_rows.append({"property": property_name, "seed": 0, **row})

        tier1 = _arm_purity(point_df, allowed, property_name, "tier1_robustness_class", "Tier 1")
        tier2 = _arm_purity(point_df, allowed, property_name, "tier2_robustness_class", "Tier 2")
        arm_purity_tier1_rows.extend(tier1)
        arm_purity_tier2_rows.extend(tier2)

        signatures_tier1 = _signatures(point_df, allowed, property_name, "tier1_robustness_class", "Tier 1")
        signatures_tier2 = _signatures(point_df, allowed, property_name, "tier2_robustness_class", "Tier 2")
        signature_rows.extend(signatures_tier1)
        signature_rows.extend(signatures_tier2)
        distinct_cost_rows.extend(_distinct_costs(tier1, signatures_tier1, property_name, "Tier 1"))
        distinct_cost_rows.extend(_distinct_costs(tier2, signatures_tier2, property_name, "Tier 2"))

        h1_decision = str(h1["h1_decision"]["decision"])
        h2_tier1 = _h2_decision(tier1)
        h2_tier2 = _h2_decision(tier2)
        go_no_go = "go_phase2" if h1_decision == "confirm" and h2_tier1 == "confirm" else "no_go_phase2"
        property_blocks.append(
            {
                "property": property_name,
                "label": label,
                "run_dir": str(run_dir),
                "phase0": phase0,
                "h1": h1,
                "h1_decision": h1_decision,
                "h2_tier1_decision": h2_tier1,
                "h2_tier2_decision": h2_tier2,
                "go_no_go": go_no_go,
                "judgment": _property_judgment(property_name, phase0, h1_decision, h2_tier1, h2_tier2, tier1, tier2),
            }
        )

    channel_mass = pd.DataFrame(channel_mass_rows)
    arm_purity_tier1 = pd.DataFrame(arm_purity_tier1_rows)
    arm_purity_tier2 = pd.DataFrame(arm_purity_tier2_rows)
    signatures = pd.DataFrame(signature_rows)
    distinct_costs = pd.DataFrame(distinct_cost_rows)
    if signatures.empty:
        signatures = pd.DataFrame(
            columns=[
                "property",
                "tier",
                "seed",
                "arm",
                "eval_id",
                "point_index",
                "stage",
                "label",
                "theta_hash",
                "theta_path",
                "max_abs_theta",
                "support_size_abs_gt_0p1",
                "active_channels_abs_gt_0p1",
                "distinct_signature",
                "signature_active_channels",
                "signature_window_band",
                "signature_channel_signs",
            ]
        )
    channel_mass.to_csv(summary_dir / "channel_mass.csv", index=False)
    arm_purity_tier1.to_csv(summary_dir / "arm_purity_tier1.csv", index=False)
    arm_purity_tier2.to_csv(summary_dir / "arm_purity_tier2.csv", index=False)
    distinct_costs.to_csv(summary_dir / "distinct_costs.csv", index=False)
    signatures.to_csv(summary_dir / "signatures.csv", index=False)

    report = _report(
        property_blocks=property_blocks,
        channel_mass=channel_mass,
        arm_purity_tier1=arm_purity_tier1,
        arm_purity_tier2=arm_purity_tier2,
        summary_dir=summary_dir,
    )
    (summary_dir / "residual_rate_report.md").write_text(report, encoding="utf-8")
    summary = {
        "status": "phase1_complete",
        "context": "confirmatory PX4 residual-rate migration",
        "properties": property_blocks,
        "overall_judgment": _overall_judgment(property_blocks),
        "artifacts": {
            "report": str(summary_dir / "residual_rate_report.md"),
            "channel_mass": str(summary_dir / "channel_mass.csv"),
            "arm_purity_tier1": str(summary_dir / "arm_purity_tier1.csv"),
            "arm_purity_tier2": str(summary_dir / "arm_purity_tier2.csv"),
            "distinct_costs": str(summary_dir / "distinct_costs.csv"),
            "signatures": str(summary_dir / "signatures.csv"),
        },
    }
    (summary_dir / "residual_rate_phase1_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(
        "RESIDUAL_RATE_PHASE1_GO_NO_GO "
        + " ".join(
            f"{block['label']}={block['go_no_go']}/H1:{block['h1_decision']}/H2T1:{block['h2_tier1_decision']}/H2T2:{block['h2_tier2_decision']}"
            for block in property_blocks
        )
        + f" report={summary_dir / 'residual_rate_report.md'}",
        flush=True,
    )


def _arm_purity(
    point_df: pd.DataFrame,
    allowed: set[str],
    property_name: str,
    tier_col: str,
    tier: str,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for arm in ["A", "B", "C"]:
        arm_df = point_df[point_df["arm"] == arm].copy()
        if tier_col not in arm_df:
            tier_class = arm_df["robustness_class"]
        else:
            tier_class = arm_df[tier_col]
        robust = arm_df[tier_class == "robust_violation"].copy()
        interior = robust[robust["amplitude_class"] == "interior"].copy()
        channel_pure = [
            _is_subset_csv(value, allowed)
            for value in interior.get("active_channels_abs_gt_0p1", pd.Series(dtype=str)).tolist()
        ]
        pure_count = int(sum(channel_pure))
        interior_count = int(len(interior))
        rows.append(
            {
                "property": property_name,
                "tier": tier,
                "seed": 0,
                "arm": arm,
                "point_count": int(len(arm_df)),
                "robust_violation_count": int(len(robust)),
                "interior_robust_violation_count": interior_count,
                "channel_pure_interior_count": pure_count,
                "channel_pure_ratio": float(pure_count / interior_count) if interior_count else 0.0,
                "channel_pure_denominator": interior_count,
                "A_phi": ",".join(sorted(allowed)),
            }
        )
    return rows


def _not_run_arm_purity(property_name: str, allowed: set[str], tier: str) -> list[dict[str, Any]]:
    return [
        {
            "property": property_name,
            "tier": tier,
            "seed": 0,
            "arm": arm,
            "point_count": 0,
            "robust_violation_count": 0,
            "interior_robust_violation_count": 0,
            "channel_pure_interior_count": 0,
            "channel_pure_ratio": 0.0,
            "channel_pure_denominator": 0,
            "A_phi": ",".join(sorted(allowed)),
            "status": "not_run_phase0_stop",
        }
        for arm in ["A", "B", "C"]
    ]


def _signatures(
    point_df: pd.DataFrame,
    allowed: set[str],
    property_name: str,
    tier_col: str,
    tier: str,
) -> list[dict[str, Any]]:
    df = point_df[point_df["arm"] == "C"].copy()
    if tier_col not in df:
        tier_class = df["robustness_class"]
    else:
        tier_class = df[tier_col]
    df["is_interior_robust_violation"] = (tier_class == "robust_violation") & (df["amplitude_class"] == "interior")
    df["is_channel_pure"] = [
        _is_subset_csv(value, allowed) for value in df.get("active_channels_abs_gt_0p1", pd.Series(dtype=str)).tolist()
    ]
    selected = df[df["is_interior_robust_violation"] & df["is_channel_pure"]].copy()
    rows: list[dict[str, Any]] = []
    for _, row in selected.iterrows():
        rows.append(
            {
                "property": property_name,
                "tier": tier,
                "seed": 0,
                "arm": "C",
                "eval_id": int(row["eval_id"]),
                "point_index": int(row["point_index"]),
                "stage": row["stage"],
                "label": row["label"],
                "theta_hash": row["theta_hash"],
                "theta_path": row["theta_path"],
                "max_abs_theta": float(row["max_abs_theta"]),
                "support_size_abs_gt_0p1": int(row["support_size_abs_gt_0p1"]),
                "active_channels_abs_gt_0p1": row["active_channels_abs_gt_0p1"],
                "distinct_signature": row.get("distinct_signature", ""),
                "signature_active_channels": row.get("signature_active_channels", ""),
                "signature_window_band": row.get("signature_window_band", ""),
                "signature_channel_signs": row.get("signature_channel_signs", ""),
                f"rho_mean_{property_name}": float(row[f"rho_mean_{property_name}"]),
                f"rho_std_{property_name}": float(row[f"rho_std_{property_name}"]),
            }
        )
    return rows


def _distinct_costs(
    arm_purity_rows: list[dict[str, Any]],
    signature_rows: list[dict[str, Any]],
    property_name: str,
    tier: str,
) -> list[dict[str, Any]]:
    arm_c = next(row for row in arm_purity_rows if row["arm"] == "C")
    signatures = {
        str(row.get("distinct_signature") or row.get("theta_hash"))
        for row in signature_rows
        if row["property"] == property_name and row["tier"] == tier
    }
    distinct_count = len(signatures)
    total_j5_points = int(arm_c["point_count"])
    return [
        {
            "property": property_name,
            "tier": tier,
            "phase": "Phase 1 seed 0",
            "method": "Arm C",
            "seed": 0,
            "total_j5_points": total_j5_points,
            "channel_pure_interior_count": int(arm_c["channel_pure_interior_count"]),
            "distinct_count": distinct_count,
            "j5_points_per_distinct": float(total_j5_points / distinct_count) if distinct_count else math.nan,
            "status": "phase2_not_run",
        }
    ]


def _not_run_distinct_costs(property_name: str, tier: str) -> list[dict[str, Any]]:
    return [
        {
            "property": property_name,
            "tier": tier,
            "phase": "Phase 1 seed 0",
            "method": "Arm C",
            "seed": 0,
            "total_j5_points": 0,
            "channel_pure_interior_count": 0,
            "distinct_count": 0,
            "j5_points_per_distinct": math.nan,
            "status": "not_run_phase0_stop",
        }
    ]


def _h2_decision(rows: list[dict[str, Any]]) -> str:
    by_arm = {row["arm"]: row for row in rows}
    arm_a_ok = int(by_arm["A"]["interior_robust_violation_count"]) == 0
    arm_b_ok = float(by_arm["B"]["channel_pure_ratio"]) <= 0.10
    arm_c_ok = (
        int(by_arm["C"]["interior_robust_violation_count"]) > 0
        and float(by_arm["C"]["channel_pure_ratio"]) >= 0.90
    )
    return "confirm" if arm_a_ok and arm_b_ok and arm_c_ok else "falsify"


def _property_judgment(
    property_name: str,
    phase0: dict[str, Any],
    h1_decision: str,
    h2_tier1: str,
    h2_tier2: str,
    tier1_rows: list[dict[str, Any]],
    tier2_rows: list[dict[str, Any]],
) -> str:
    active = ",".join(derive_A_phi(property_name))
    if not bool(phase0.get("tier1_violable_with_saturated_predicted_channel")):
        return f"confirmatory PX4 {property_name}: no Tier 1 saturated {active} violation; stop."
    if h1_decision != "confirm":
        return f"confirmatory PX4 {property_name}: H-1 falsifies predicted-channel dominance for {active}; bug class does not migrate by this criterion."
    if h2_tier1 == "confirm" and h2_tier2 == "confirm":
        return f"confirmatory PX4 {property_name}: residual-rate bug class migrates to {active} under Tier 1 and Tier 2."
    if h2_tier1 == "confirm":
        return f"confirmatory PX4 {property_name}: residual-rate bug class migrates to {active} under Tier 1; Tier 2 non-convergent interior migration is not confirmed."
    arm_c_t1 = next(row for row in tier1_rows if row["arm"] == "C")
    arm_c_t2 = next(row for row in tier2_rows if row["arm"] == "C")
    if int(arm_c_t1["interior_robust_violation_count"]) == 0 and int(arm_c_t2["interior_robust_violation_count"]) == 0:
        return f"confirmatory PX4 {property_name}: only saturated/no internal {active} violations were found; bug class does not migrate to this channel."
    return f"confirmatory PX4 {property_name}: H-2 falsifies direct synthesis for {active}; migration is not confirmed."


def _overall_judgment(property_blocks: list[dict[str, Any]]) -> str:
    confirmed = [
        block for block in property_blocks if block["h1_decision"] == "confirm" and block["h2_tier1_decision"] == "confirm"
    ]
    if confirmed:
        labels = ", ".join(block["label"] for block in confirmed)
        return f"confirmatory PX4 overall: residual-rate migration is confirmed at Tier 1 for {labels}; multi-channel method-paper premise survives Phase 1 pending Phase 2 and ArduPilot."
    return "confirmatory PX4 overall: neither throttle nor yaw residual-rate migration is confirmed at Tier 1; multi-channel method-paper premise is not established by this test."


def _report(
    *,
    property_blocks: list[dict[str, Any]],
    channel_mass: pd.DataFrame,
    arm_purity_tier1: pd.DataFrame,
    arm_purity_tier2: pd.DataFrame,
    summary_dir: Path,
) -> str:
    lines: list[str] = [
        "# Residual-rate migration report",
        "",
        "Context: confirmatory, PX4, scenario `px4_position`. Phase 2 and ddmin were not run.",
        "",
        "## One-line judgments",
        "",
    ]
    for block in property_blocks:
        lines.append(f"- {block['judgment']}")
    lines.append(f"- {_overall_judgment(property_blocks)}")
    lines.extend(["", "## Phase 0", "", "| property | Tier 1 saturated violable | Tier 2 saturated violable | best label |", "| --- | --- | --- | --- |"])
    for block in property_blocks:
        phase0 = block["phase0"]
        lines.append(
            f"| `{block['property']}` | {phase0['tier1_violable_with_saturated_predicted_channel']} | "
            f"{phase0['tier2_violable_with_saturated_predicted_channel']} | `{phase0['best_tier1_violation_label']}` |"
        )
    lines.extend(["", "## H-1 Channel Mass", "", _markdown_table(channel_mass), ""])
    lines.extend(["## H-2 Tier 1", "", _markdown_table(arm_purity_tier1), ""])
    lines.extend(["## H-2 Tier 2", "", _markdown_table(arm_purity_tier2), ""])
    lines.extend(["## Go/no-go", "", "| property | H-1 | H-2 Tier 1 | H-2 Tier 2 | Phase 2 |", "| --- | --- | --- | --- | --- |"])
    for block in property_blocks:
        lines.append(
            f"| `{block['property']}` | {block['h1_decision']} | {block['h2_tier1_decision']} | "
            f"{block['h2_tier2_decision']} | {block['go_no_go']} |"
        )
    lines.extend(
        [
            "",
            "## Data and parameter gaps",
            "",
            "- Phase 2 seed 1/2 and ddmin baseline were not run by instruction.",
            "- Cross-platform claims require ArduPilot.",
            "- Thresholds are bound to local PX4 SITL defaults recorded in `artifacts/residual_rate_prereg.md`.",
            "",
            "## Artifacts",
            "",
            f"- Summary dir: `{summary_dir}`",
            f"- CSV: `{summary_dir / 'channel_mass.csv'}`, `{summary_dir / 'arm_purity_tier1.csv'}`, `{summary_dir / 'arm_purity_tier2.csv'}`, `{summary_dir / 'distinct_costs.csv'}`, `{summary_dir / 'signatures.csv'}`",
            "",
        ]
    )
    return "\n".join(lines)


def _is_subset_csv(value: Any, allowed: set[str]) -> bool:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return False
    channels = {part.strip() for part in str(value).split(",") if part.strip()}
    return bool(channels) and channels.issubset(allowed)


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected object in {path}")
    return data


def _markdown_table(df: pd.DataFrame) -> str:
    if df.empty:
        return "_No rows._"
    columns = list(df.columns)
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for _, row in df.iterrows():
        lines.append("| " + " | ".join(_markdown_value(row[column]) for column in columns) + " |")
    return "\n".join(lines)


def _markdown_value(value: Any) -> str:
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"
    return str(value)


if __name__ == "__main__":
    main()
