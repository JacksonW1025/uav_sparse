from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


SUPPORT_THRESHOLD = 0.1
CLEAN_CHANNELS = {"roll", "pitch"}


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize Direction-A seed replication results.")
    parser.add_argument("--artifacts-dir", default="artifacts")
    parser.add_argument("--runs-dir", default="runs")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--output-prefix", default="artifacts/seed_replication")
    args = parser.parse_args()

    artifacts_dir = Path(args.artifacts_dir)
    runs_dir = Path(args.runs_dir)
    seeds = [int(value.strip()) for value in str(args.seeds).split(",") if value.strip()]
    rows: list[dict[str, Any]] = []
    cost_rows: list[dict[str, Any]] = []
    criteria_rows: list[dict[str, Any]] = []

    for seed in seeds:
        probe_dir = _find_run_dir("direction_a_px4_position", seed, artifacts_dir, runs_dir)
        ddmin_dir = _find_run_dir("direction_a_ddmin_px4_position", seed, artifacts_dir, runs_dir)
        probe_summary = _read_json(probe_dir / "reports" / "direction_a_summary.json")
        ddmin_summary = _read_json(ddmin_dir / "reports" / "direction_a_ddmin_summary.json")
        arm_metrics = {row["arm"]: row for row in probe_summary["arm_metrics"]}

        cadet_distinct = _distinct_cadet_arm_c(probe_dir, runs_dir)
        ddmin_distinct = _distinct_ddmin(ddmin_dir, runs_dir)

        arm_c_j5 = int(arm_metrics["C"]["j5_point_count"])
        ddmin_j5 = int(ddmin_summary["total_ddmin_j5_points_used"])
        cadet_cost = _cost_per_distinct(arm_c_j5, cadet_distinct["distinct_count"])
        ddmin_cost = _cost_per_distinct(ddmin_j5, ddmin_distinct["distinct_count"])

        cost_rows.extend(
            [
                {
                    "seed": seed,
                    "method": "CADET_arm_C",
                    "total_j5_points": arm_c_j5,
                    "clean_trigger_count": cadet_distinct["clean_count"],
                    "distinct_clean_trigger_count": cadet_distinct["distinct_count"],
                    "j5_points_per_clean_trigger": _safe_div(arm_c_j5, cadet_distinct["clean_count"]),
                    "j5_points_per_distinct_clean_trigger": cadet_cost,
                    "signatures": "; ".join(cadet_distinct["signatures"]),
                },
                {
                    "seed": seed,
                    "method": "ddmin",
                    "total_j5_points": ddmin_j5,
                    "clean_trigger_count": ddmin_distinct["clean_count"],
                    "distinct_clean_trigger_count": ddmin_distinct["distinct_count"],
                    "j5_points_per_clean_trigger": _safe_div(ddmin_j5, ddmin_distinct["clean_count"]),
                    "j5_points_per_distinct_clean_trigger": ddmin_cost,
                    "signatures": "; ".join(ddmin_distinct["signatures"]),
                },
            ]
        )

        arm_b_support = _support_dist(arm_metrics["B"])
        arm_c_support = _support_dist(arm_metrics["C"])
        ddmin_residual_ratio = _throttle_yaw_residual_ratio(ddmin_dir)
        criteria = _criteria_status(
            arm_a=arm_metrics["A"],
            arm_b=arm_metrics["B"],
            arm_c=arm_metrics["C"],
            ddmin_summary=ddmin_summary,
            cadet_cost=cadet_cost,
            ddmin_cost=ddmin_cost,
        )
        criteria_rows.append({"seed": seed, **criteria})

        rows.append(
            {
                "seed": seed,
                "probe_dir": str(probe_dir),
                "ddmin_dir": str(ddmin_dir),
                "arm_a_interior": int(arm_metrics["A"]["interior_robust_violation_count"]),
                "arm_b_interior": int(arm_metrics["B"]["interior_robust_violation_count"]),
                "arm_b_support_median": arm_b_support.get("median", math.nan),
                "arm_b_active_channels": _channel_keys(arm_metrics["B"]),
                "arm_c_interior": int(arm_metrics["C"]["interior_robust_violation_count"]),
                "arm_c_support_median": arm_c_support.get("median", math.nan),
                "arm_c_support_min": arm_c_support.get("min", math.nan),
                "arm_c_support_max": arm_c_support.get("max", math.nan),
                "arm_c_active_channels": _channel_keys(arm_metrics["C"]),
                "ddmin_clean_ratio": _ratio_text(
                    int(ddmin_summary["clean_trigger_count"]),
                    int(ddmin_summary["starting_trigger_count"]),
                ),
                "ddmin_clean_yield": float(ddmin_summary["clean_yield"]),
                "ddmin_support_median": ddmin_summary["final_support_distribution"].get("median", math.nan),
                "ddmin_throttle_yaw_residual_ratio": ddmin_residual_ratio,
                "cadet_distinct_clean_triggers": cadet_distinct["distinct_count"],
                "cadet_j5_per_distinct_clean_trigger": cadet_cost,
                "ddmin_distinct_clean_triggers": ddmin_distinct["distinct_count"],
                "ddmin_j5_per_distinct_clean_trigger": ddmin_cost,
                "criteria_summary": criteria["summary"],
            }
        )

    output_prefix = Path(args.output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    table_path = output_prefix.with_name(output_prefix.name + "_table.csv")
    cost_path = output_prefix.with_name(output_prefix.name + "_distinct_costs.csv")
    criteria_path = output_prefix.with_name(output_prefix.name + "_criteria.csv")
    json_path = output_prefix.with_name(output_prefix.name + "_summary.json")
    report_path = output_prefix.with_name(output_prefix.name + "_report.md")

    table_df = pd.DataFrame(rows)
    cost_df = pd.DataFrame(cost_rows)
    criteria_df = pd.DataFrame(criteria_rows)
    table_df.to_csv(table_path, index=False)
    cost_df.to_csv(cost_path, index=False)
    criteria_df.to_csv(criteria_path, index=False)
    payload = {
        "table": _jsonable(rows),
        "distinct_costs": _jsonable(cost_rows),
        "criteria": _jsonable(criteria_rows),
        "signature_definition": {
            "active_channels": "sorted active channel set for |theta| > 0.1",
            "time_band": "min/max active window id",
            "envelope_shape": "per-active-channel 10-window sign and amplitude-bin sequence; bins are <=0.33, <=0.66, >0.66",
        },
    }
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    report_path.write_text(_render_report(rows, cost_rows, criteria_rows), encoding="utf-8")
    print(f"wrote {table_path}")
    print(f"wrote {cost_path}")
    print(f"wrote {criteria_path}")
    print(f"wrote {json_path}")
    print(f"wrote {report_path}")


def _find_run_dir(prefix: str, seed: int, artifacts_dir: Path, runs_dir: Path) -> Path:
    pattern = f"{prefix}_seed{seed}_v*"
    candidates = []
    candidates.extend(path for path in artifacts_dir.glob(pattern) if path.is_dir())
    candidates.extend(path for path in runs_dir.glob(pattern) if path.is_dir())
    candidates = [path for path in candidates if _has_summary(prefix, path)]
    if not candidates:
        raise FileNotFoundError(f"No completed {prefix} seed {seed} directory found")
    return sorted(candidates, key=lambda path: (path.parent == artifacts_dir, path.name), reverse=True)[0]


def _has_summary(prefix: str, path: Path) -> bool:
    if prefix.startswith("direction_a_ddmin"):
        return (path / "reports" / "direction_a_ddmin_summary.json").exists()
    return (path / "reports" / "direction_a_summary.json").exists()


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _distinct_cadet_arm_c(probe_dir: Path, runs_dir: Path) -> dict[str, Any]:
    rows = pd.read_csv(probe_dir / "reports" / "interior_violations.csv")
    rows = rows[rows["arm"] == "C"].copy()
    rows = rows[rows["support_size_abs_gt_0p1"].astype(int) <= 8].copy()
    rows = rows[rows["active_channels_abs_gt_0p1"].map(lambda value: _parse_channels(value) <= CLEAN_CHANNELS)].copy()
    return _distinct_from_rows(probe_dir, rows, "theta_path", runs_dir)


def _distinct_ddmin(ddmin_dir: Path, runs_dir: Path) -> dict[str, Any]:
    rows = pd.read_csv(ddmin_dir / "reports" / "minimized_triggers.csv")
    rows = rows[rows["is_clean"].map(_as_bool)].copy()
    return _distinct_from_rows(ddmin_dir, rows, "final_theta_path", runs_dir)


def _distinct_from_rows(run_dir: Path, rows: pd.DataFrame, theta_column: str, runs_dir: Path) -> dict[str, Any]:
    groups = pd.read_csv(_groups_path(run_dir, runs_dir))
    signatures: list[str] = []
    for _, row in rows.iterrows():
        theta_path = _resolve_theta_path(run_dir, Path(str(row[theta_column])))
        theta = np.load(theta_path)
        signatures.append(_trigger_signature(theta, groups))
    return {
        "clean_count": int(len(rows)),
        "distinct_count": int(len(set(signatures))),
        "signatures": sorted(set(signatures)),
    }


def _groups_path(run_dir: Path, runs_dir: Path) -> Path:
    path = run_dir / "groups.csv"
    if path.exists():
        return path
    candidate = runs_dir / run_dir.name / "groups.csv"
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"groups.csv not found in {run_dir} or {candidate}")


def _resolve_theta_path(run_dir: Path, theta_path: Path) -> Path:
    if theta_path.exists():
        return theta_path
    candidate = run_dir / "thetas" / theta_path.name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"theta path not found: {theta_path}; also checked {candidate}")


def _trigger_signature(theta: np.ndarray, groups: pd.DataFrame) -> str:
    theta = np.asarray(theta, dtype=float)
    active = []
    for _, group in groups.sort_values("group_id").iterrows():
        group_id = int(group["group_id"])
        if group_id < len(theta) and abs(float(theta[group_id])) > SUPPORT_THRESHOLD:
            active.append((str(group["channel"]), int(group["window_id"]), float(theta[group_id])))
    if not active:
        return "channels=none;time=none;shape=none"
    channels = sorted({channel for channel, _, _ in active})
    windows = [window for _, window, _ in active]
    shape_parts = []
    for channel in channels:
        channel_groups = groups[groups["channel"] == channel].sort_values("window_id")
        seq = []
        for _, group in channel_groups.iterrows():
            group_id = int(group["group_id"])
            value = float(theta[group_id]) if group_id < len(theta) else 0.0
            seq.append(_value_bin(value))
        shape_parts.append(f"{channel}:{','.join(seq)}")
    return (
        f"channels={','.join(channels)};"
        f"time=w{min(windows):02d}-w{max(windows):02d};"
        f"shape={'|'.join(shape_parts)}"
    )


def _value_bin(value: float) -> str:
    value = float(value)
    if abs(value) <= SUPPORT_THRESHOLD:
        return "0"
    sign = "p" if value > 0.0 else "m"
    abs_value = abs(value)
    if abs_value <= 0.33:
        bucket = "1"
    elif abs_value <= 0.66:
        bucket = "2"
    else:
        bucket = "3"
    return sign + bucket


def _support_dist(arm_metric: dict[str, Any]) -> dict[str, float]:
    supports = [float(row["support_size_abs_gt_0p1"]) for row in arm_metric["interior_violation_supports"]]
    if not supports:
        return {}
    arr = np.asarray(supports, dtype=float)
    return {
        "min": float(np.min(arr)),
        "median": float(np.median(arr)),
        "max": float(np.max(arr)),
    }


def _channel_keys(arm_metric: dict[str, Any]) -> str:
    values = sorted({str(row["active_channels_abs_gt_0p1"]) for row in arm_metric["interior_violation_supports"]})
    return ";".join(values)


def _throttle_yaw_residual_ratio(ddmin_dir: Path) -> str:
    rows = pd.read_csv(ddmin_dir / "reports" / "minimized_triggers.csv")
    if rows.empty:
        return "0/0"
    residual = 0
    for value in rows["final_active_channels_abs_gt_0p1"]:
        channels = _parse_channels(value)
        if "throttle" in channels or "yaw" in channels:
            residual += 1
    return _ratio_text(residual, len(rows))


def _criteria_status(
    *,
    arm_a: dict[str, Any],
    arm_b: dict[str, Any],
    arm_c: dict[str, Any],
    ddmin_summary: dict[str, Any],
    cadet_cost: float,
    ddmin_cost: float,
) -> dict[str, Any]:
    arm_a_interior = int(arm_a["interior_robust_violation_count"])
    arm_b_interior = int(arm_b["interior_robust_violation_count"])
    arm_c_interior = int(arm_c["interior_robust_violation_count"])
    arm_c_support = _support_dist(arm_c)
    arm_c_channels_ok = all(_parse_channels(row["active_channels_abs_gt_0p1"]) <= CLEAN_CHANNELS for row in arm_c["interior_violation_supports"])
    arm_c_support_ok = bool(arm_c_support) and arm_c_support["min"] >= 4 and arm_c_support["max"] <= 8
    ddmin_median = float(ddmin_summary["final_support_distribution"].get("median", math.inf))
    arm_c_median = float(arm_c_support.get("median", math.inf))
    ddmin_failed = (
        int(ddmin_summary["clean_trigger_count"]) < arm_c_interior
        and ddmin_median > arm_c_median
        and ddmin_cost > cadet_cost
    )
    checks = {
        "arm_a_zero_interior": arm_a_interior == 0,
        "arm_c_count_exceeds_arm_b": arm_c_interior > arm_b_interior,
        "arm_c_support_4_to_8": arm_c_support_ok,
        "arm_c_channels_subset_roll_pitch": arm_c_channels_ok,
        "ddmin_failure_relative_to_arm_c": ddmin_failed,
    }
    passed = [name for name, ok in checks.items() if ok]
    failed = [name for name, ok in checks.items() if not ok]
    return {
        **checks,
        "summary": f"pass={','.join(passed) if passed else 'none'}; fail={','.join(failed) if failed else 'none'}",
    }


def _parse_channels(value: Any) -> set[str]:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return set()
    text = str(value).strip()
    if not text:
        return set()
    return {part.strip() for part in text.split(",") if part.strip()}


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes"}


def _safe_div(numerator: float, denominator: float) -> float:
    return float(numerator / denominator) if float(denominator) else math.inf


def _cost_per_distinct(j5_points: int, distinct_count: int) -> float:
    return _safe_div(float(j5_points), float(distinct_count))


def _ratio_text(numerator: int, denominator: int) -> str:
    return f"{int(numerator)}/{int(denominator)}"


def _render_report(rows: list[dict[str, Any]], cost_rows: list[dict[str, Any]], criteria_rows: list[dict[str, Any]]) -> str:
    lines = [
        "# Direction-A Seed Replication Summary",
        "",
        "Trigger signature: active channel set, active window band, and per-channel sign/amplitude-bin envelope for |theta| > 0.1.",
        "",
        "## Cross-Seed Table",
        "",
        "| seed | Arm A interior | Arm B interior/support | Arm C interior/support/channels | ddmin clean | ddmin support median | throttle/yaw residual | per-distinct cost |",
        "| ---: | ---: | --- | --- | --- | ---: | --- | --- |",
    ]
    by_seed_cost = {}
    for row in cost_rows:
        by_seed_cost[(row["seed"], row["method"])] = row
    for row in rows:
        cadet = by_seed_cost[(row["seed"], "CADET_arm_C")]
        ddmin = by_seed_cost[(row["seed"], "ddmin")]
        lines.append(
            f"| {row['seed']} | {row['arm_a_interior']} | "
            f"{row['arm_b_interior']} / med {row['arm_b_support_median']:.1f} | "
            f"{row['arm_c_interior']} / med {row['arm_c_support_median']:.1f} / {row['arm_c_active_channels']} | "
            f"{row['ddmin_clean_ratio']} | {row['ddmin_support_median']:.1f} | "
            f"{row['ddmin_throttle_yaw_residual_ratio']} | "
            f"C {cadet['j5_points_per_distinct_clean_trigger']:.2f}; "
            f"ddmin {_fmt_float(ddmin['j5_points_per_distinct_clean_trigger'])} |"
        )
    lines.extend(["", "## Criteria", ""])
    criteria_by_seed = {row["seed"]: row for row in criteria_rows}
    for row in rows:
        criteria = criteria_by_seed[row["seed"]]
        lines.append(f"- seed {row['seed']}: {criteria['summary']}")
    lines.extend(["", "## Distinct Costs", ""])
    lines.extend(
        [
            "| seed | method | total J=5 points | clean triggers | distinct clean triggers | J=5 per distinct clean trigger |",
            "| ---: | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in cost_rows:
        lines.append(
            f"| {row['seed']} | {row['method']} | {row['total_j5_points']} | "
            f"{row['clean_trigger_count']} | {row['distinct_clean_trigger_count']} | "
            f"{_fmt_float(row['j5_points_per_distinct_clean_trigger'])} |"
        )
    lines.append("")
    return "\n".join(lines)


def _fmt_float(value: float) -> str:
    return "inf" if not math.isfinite(float(value)) else f"{float(value):.2f}"


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_jsonable(item) for item in value]
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, float) and math.isinf(value):
        return "inf"
    return value


if __name__ == "__main__":
    main()
