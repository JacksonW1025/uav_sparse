from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Any

import pandas as pd

from cadet.runners.alt_drift_phase0 import TARGET_PROPERTY
from cadet.runners.direction_a_probe import derive_A_phi


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize confirmatory PX4 alt_drift Phase 1 go/no-go.")
    parser.add_argument("--seed-run-dir", default="artifacts/alt_drift_seed0_v0")
    parser.add_argument("--summary-dir", default="artifacts/alt_drift_summary")
    args = parser.parse_args()

    seed_dir = Path(args.seed_run_dir)
    reports_dir = seed_dir / "reports"
    summary_dir = Path(args.summary_dir)
    summary_dir.mkdir(parents=True, exist_ok=True)

    phase0 = _read_json(reports_dir / "phase0_sanity_summary.json")
    h1 = _read_json(reports_dir / "h_alt_1_summary.json")
    point_df = pd.read_csv(reports_dir / "point_evaluations.csv")
    arm_metrics_path = reports_dir / "arm_metrics.csv"
    arm_metrics_df = pd.read_csv(arm_metrics_path) if arm_metrics_path.exists() else pd.DataFrame()

    shutil.copy2(reports_dir / "h_alt_1_channel_mass.csv", summary_dir / "channel_mass.csv")
    arm_purity = _arm_purity(point_df, set(derive_A_phi(TARGET_PROPERTY)))
    arm_purity.to_csv(summary_dir / "arm_purity.csv", index=False)
    signatures = _signatures(point_df, set(derive_A_phi(TARGET_PROPERTY)))
    signatures.to_csv(summary_dir / "signatures_seed0.csv", index=False)

    phase0_decision = _phase0_decision(phase0)
    h1_decision_block = _h1_decision_block(h1)
    h1_decision = str(h1_decision_block.get("decision", "unknown"))
    h2_decision = _h2_decision(arm_purity)
    go_no_go = "go_phase2" if h1_decision == "confirm" and h2_decision == "confirm" else "no_go_phase2"

    report = _report(
        phase0_decision=phase0_decision,
        h1=h1,
        h1_decision_block=h1_decision_block,
        channel_mass=pd.read_csv(summary_dir / "channel_mass.csv"),
        arm_purity=arm_purity,
        arm_metrics=arm_metrics_df,
        h2_decision=h2_decision,
        go_no_go=go_no_go,
        summary_dir=summary_dir,
        seed_dir=seed_dir,
    )
    (summary_dir / "alt_drift_report.md").write_text(report, encoding="utf-8")

    summary = {
        "status": "phase1_complete",
        "context": "confirmatory PX4 alt_drift",
        "phase0_decision": phase0_decision,
        "h_alt_1_decision": h1_decision,
        "h_alt_2_decision": h2_decision,
        "go_no_go": go_no_go,
        "derived_A_alt": derive_A_phi(TARGET_PROPERTY),
        "artifacts": {
            "report": str(summary_dir / "alt_drift_report.md"),
            "channel_mass": str(summary_dir / "channel_mass.csv"),
            "arm_purity": str(summary_dir / "arm_purity.csv"),
            "signatures_seed0": str(summary_dir / "signatures_seed0.csv"),
        },
    }
    (summary_dir / "alt_drift_phase1_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
    )
    print(
        "ALT_DRIFT_PHASE1_GO_NO_GO "
        f"confirmatory PX4 alt_drift go_no_go={go_no_go} "
        f"h_alt_1={h1_decision} h_alt_2={h2_decision} "
        f"report={summary_dir / 'alt_drift_report.md'}",
        flush=True,
    )


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected object in {path}")
    return data


def _phase0_decision(summary: dict[str, Any]) -> str:
    if bool(summary.get("feasible_throttle_violation")) or bool(summary.get("violable_with_saturated_throttle")):
        return "confirm"
    return "falsify"


def _h1_decision_block(summary: dict[str, Any]) -> dict[str, Any]:
    block = summary.get("h_alt_1_decision")
    if isinstance(block, dict):
        return block
    return {
        "decision": summary.get("decision", "unknown"),
        "top_channel": summary.get("top_channel", "unknown"),
        "throttle_share": summary.get("throttle_share", float("nan")),
        "recommended_A_alt": summary.get("A_alt", derive_A_phi(TARGET_PROPERTY)),
    }


def _is_subset_csv(value: Any, allowed: set[str]) -> bool:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return False
    channels = {part.strip() for part in str(value).split(",") if part.strip()}
    return bool(channels) and channels.issubset(allowed)


def _arm_purity(point_df: pd.DataFrame, allowed: set[str]) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for arm in ["A", "B", "C"]:
        arm_df = point_df[point_df["arm"] == arm].copy()
        interior = arm_df[
            (arm_df["robustness_class"] == "robust_violation")
            & (arm_df["amplitude_class"] == "interior")
        ].copy()
        channel_pure = [
            _is_subset_csv(value, allowed)
            for value in interior.get("active_channels_abs_gt_0p1", pd.Series(dtype=str)).tolist()
        ]
        pure_count = int(sum(channel_pure))
        interior_count = int(len(interior))
        rows.append(
            {
                "seed": 0,
                "arm": arm,
                "point_count": int(len(arm_df)),
                "robust_violation_count": int((arm_df["robustness_class"] == "robust_violation").sum()),
                "interior_robust_violation_count": interior_count,
                "channel_pure_interior_count": pure_count,
                "channel_pure_ratio": float(pure_count / interior_count) if interior_count else 0.0,
                "channel_pure_denominator": interior_count,
                "A_alt": ",".join(sorted(allowed)),
            }
        )
    return pd.DataFrame(rows)


def _signatures(point_df: pd.DataFrame, allowed: set[str]) -> pd.DataFrame:
    df = point_df[point_df["arm"] == "C"].copy()
    df["is_interior_robust_violation"] = (
        (df["robustness_class"] == "robust_violation") & (df["amplitude_class"] == "interior")
    )
    df["is_channel_pure"] = [
        _is_subset_csv(value, allowed) for value in df.get("active_channels_abs_gt_0p1", pd.Series(dtype=str)).tolist()
    ]
    columns = [
        "eval_id",
        "point_index",
        "arm",
        "stage",
        "label",
        "theta_hash",
        "max_abs_theta",
        "amplitude_class",
        "support_size_abs_gt_0p1",
        "active_channels_abs_gt_0p1",
        "robustness_class",
        "negative_mean_rejected_by_2sigma_gate",
        "distinct_signature",
        "signature_active_channels",
        "signature_window_band",
        "signature_channel_signs",
        f"rho_mean_{TARGET_PROPERTY}",
        f"rho_std_{TARGET_PROPERTY}",
        "is_interior_robust_violation",
        "is_channel_pure",
    ]
    return df[[column for column in columns if column in df.columns]]


def _h2_decision(arm_purity: pd.DataFrame) -> str:
    by_arm = {row["arm"]: row for row in arm_purity.to_dict(orient="records")}
    arm_a_ok = int(by_arm["A"]["interior_robust_violation_count"]) == 0
    arm_b_ok = float(by_arm["B"]["channel_pure_ratio"]) <= 0.10
    arm_c_ok = (
        int(by_arm["C"]["interior_robust_violation_count"]) > 0
        and float(by_arm["C"]["channel_pure_ratio"]) >= 0.90
    )
    return "confirm" if arm_a_ok and arm_b_ok and arm_c_ok else "falsify"


def _report(
    *,
    phase0_decision: str,
    h1: dict[str, Any],
    h1_decision_block: dict[str, Any],
    channel_mass: pd.DataFrame,
    arm_purity: pd.DataFrame,
    arm_metrics: pd.DataFrame,
    h2_decision: str,
    go_no_go: str,
    summary_dir: Path,
    seed_dir: Path,
) -> str:
    lines: list[str] = [
        "# alt_drift Phase 1 report",
        "",
        "Context: confirmatory PX4 alt_drift, scenario px4_position.",
        "",
        "## Phase 0",
        "",
        f"Decision: {phase0_decision}. Saturated positive throttle is feasible and violates alt_drift under the frozen 2-sigma gate.",
        "",
        "## H-alt-1",
        "",
        (
            f"Decision: {h1_decision_block.get('decision', 'unknown')}. "
            f"Top channel: {h1_decision_block.get('top_channel', 'unknown')}. "
            f"Throttle share: {_markdown_value(h1_decision_block.get('throttle_share'))}. "
            f"A_alt: {','.join(h1_decision_block.get('recommended_A_alt', derive_A_phi(TARGET_PROPERTY)))}."
        ),
        "",
        _markdown_table(channel_mass),
        "",
        "## H-alt-2",
        "",
        f"Decision: {h2_decision}. Thresholds: Arm A interior=0; Arm B channel pure <=0.10; Arm C interior>0 and channel pure >=0.90.",
        "",
        _markdown_table(arm_purity),
        "",
    ]
    if not arm_metrics.empty:
        metric_cols = [
            column
            for column in [
                "arm",
                "j5_point_count",
                "robust_violation_count",
                "interior_robust_violation_count",
                "noise_band_count",
                "negative_mean_rejected_by_2sigma_gate_count",
            ]
            if column in arm_metrics.columns
        ]
        lines.extend(["Runner arm metrics:", "", _markdown_table(arm_metrics[metric_cols]), ""])
    lines.extend(
        [
            "## Go/no-go",
            "",
            f"Phase 1 go/no-go: {go_no_go}. Phase 2 was not run.",
            "",
            "## Artifacts",
            "",
            f"- Seed run dir: `{seed_dir}`",
            f"- Summary dir: `{summary_dir}`",
            f"- CSV: `{summary_dir / 'channel_mass.csv'}`, `{summary_dir / 'arm_purity.csv'}`, `{summary_dir / 'signatures_seed0.csv'}`",
            "",
            "Caveat: this is confirmatory PX4 alt_drift only; cross-platform claims require ArduPilot.",
            "",
        ]
    )
    return "\n".join(lines)


def _markdown_table(df: pd.DataFrame) -> str:
    if df.empty:
        return "(empty)"
    columns = [str(column) for column in df.columns]
    rows = df.to_dict(orient="records")
    rendered = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        rendered.append("| " + " | ".join(_markdown_value(row.get(column)) for column in df.columns) + " |")
    return "\n".join(rendered)


def _markdown_value(value: Any) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return ""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value).replace("|", "\\|")


if __name__ == "__main__":
    main()
