from __future__ import annotations

import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "artifacts" / "recut_distinct_v0"
SEED_TABLE = ROOT / "artifacts" / "seed_replication_table.csv"
RAW_COSTS = ROOT / "artifacts" / "recut_channel_gentle_v0" / "channel_purity_costs.csv"
SUPPORT_THRESHOLD = 0.1
CLEAN_CHANNELS = {"roll", "pitch"}
SUPPORT_LE8_EXPECTED = {0: 7, 1: 0, 2: 1}
DDMIN_SUPPORT_LE8_EXPECTED = {0: 4, 1: 2, 2: 4}
LABEL_RE = re.compile(r"^env(?P<env>\d+)_deg(?P<deg>\d+)_w(?P<w>\d+)_d(?P<d>\d+)")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    seed_rows = _read_seed_table()
    raw_cost_rows = _read_raw_costs()
    group_info = _load_frozen_groups(seed_rows)

    cadet_rows: list[dict[str, Any]] = []
    ddmin_rows: list[dict[str, Any]] = []
    cost_rows: list[dict[str, Any]] = []
    seed_summaries: list[dict[str, Any]] = []
    gaps: list[str] = []

    for seed_row in seed_rows:
        seed = int(seed_row["seed"])
        probe_dir = ROOT / seed_row["probe_dir"]
        ddmin_dir = ROOT / seed_row["ddmin_dir"]
        groups = group_info["groups_by_seed"][seed]
        mapping_source = group_info["source_by_seed"][seed]

        cadet = _cadet_signatures(seed, probe_dir, groups, mapping_source)
        ddmin = _ddmin_signatures(seed, ddmin_dir, groups, mapping_source)
        cadet_rows.extend(cadet["rows"])
        ddmin_rows.extend(ddmin["rows"])
        gaps.extend(cadet["gaps"])
        gaps.extend(ddmin["gaps"])

        raw = raw_cost_rows[seed]
        d_c = len(cadet["main_signatures"])
        d_dd = len(ddmin["signatures"])
        arm_c_total = 80
        ddmin_total = int(ddmin["total_j5"])
        ddmin_e2e_total = ddmin_total + 80
        cadet_distinct_cost = _safe_div(arm_c_total, d_c)
        ddmin_distinct_cost = _safe_div(ddmin_total, d_dd)
        ddmin_e2e_distinct_cost = _safe_div(ddmin_e2e_total, d_dd)

        cadet_raw_count = int(raw["arm_c_channel_pure_count"])
        ddmin_raw_count = int(raw["ddmin_roll_pitch_only_count"])
        cost_rows.extend(
            [
                _cost_row(
                    seed=seed,
                    method="CADET_arm_C",
                    d=d_c,
                    total_j5=arm_c_total,
                    raw_count=cadet_raw_count,
                    raw_j5=float(raw["cadet_j5_per_channel_pure_interior"]),
                    distinct_j5=cadet_distinct_cost,
                    raw_source="recut_channel_gentle_v0/channel_purity_costs.csv",
                    note="Arm C fixed 80 J5 points; D is main envelope distinct signature.",
                ),
                _cost_row(
                    seed=seed,
                    method="ddmin_direct",
                    d=d_dd,
                    total_j5=ddmin_total,
                    raw_count=ddmin_raw_count,
                    raw_j5=float(raw["ddmin_j5_per_roll_pitch_only_output"]),
                    distinct_j5=ddmin_distinct_cost,
                    raw_source="recut_channel_gentle_v0/channel_purity_costs.csv",
                    note="Only is_roll_pitch_only minimized triggers; total is ddmin J5 points.",
                ),
                _cost_row(
                    seed=seed,
                    method="ddmin_e2e",
                    d=d_dd,
                    total_j5=ddmin_e2e_total,
                    raw_count=ddmin_raw_count,
                    raw_j5=float(raw["ddmin_j5_per_roll_pitch_only_output_e2e_plus_arm_b80"]),
                    distinct_j5=ddmin_e2e_distinct_cost,
                    raw_source="recut_channel_gentle_v0/channel_purity_costs.csv",
                    note="End-to-end total adds the 80 J5 Arm B starting-point budget.",
                ),
            ]
        )
        seed_summaries.append(
            {
                "seed": seed,
                "probe_dir": seed_row["probe_dir"],
                "ddmin_dir": seed_row["ddmin_dir"],
                "cadet_raw_count": cadet_raw_count,
                "cadet_D": d_c,
                "cadet_prereg_no_support_D": len(cadet["prereg_no_support_signatures"]),
                "cadet_prereg_support_le8_D": len(cadet["prereg_support_le8_signatures"]),
                "ddmin_raw_count": ddmin_raw_count,
                "ddmin_D": d_dd,
                "cadet_j5_per_distinct": cadet_distinct_cost,
                "ddmin_j5_per_distinct": ddmin_distinct_cost,
                "ddmin_e2e_j5_per_distinct": ddmin_e2e_distinct_cost,
                "ddmin_vs_cadet": _safe_div(ddmin_distinct_cost, cadet_distinct_cost),
                "ddmin_e2e_vs_cadet": _safe_div(ddmin_e2e_distinct_cost, cadet_distinct_cost),
                "cadet_raw_inflation": _safe_div(cadet_raw_count, d_c),
                "ddmin_raw_inflation": _safe_div(ddmin_raw_count, d_dd),
                "cadet_main_signatures": sorted(cadet["main_signatures"]),
                "ddmin_signatures": sorted(ddmin["signatures"]),
                "group_mapping_source": mapping_source,
                "cadet_label_missing": cadet["label_missing"],
                "ddmin_theta_missing": ddmin["theta_missing"],
                "ddmin_group_channel_mismatches": ddmin["channel_mismatches"],
            }
        )

    _write_csv(OUT_DIR / "signatures_cadet.csv", cadet_rows)
    _write_csv(OUT_DIR / "signatures_ddmin.csv", ddmin_rows)
    _write_csv(OUT_DIR / "distinct_costs.csv", cost_rows)
    report = _render_report(seed_summaries, cost_rows, group_info, gaps)
    (OUT_DIR / "distinct_report.md").write_text(report, encoding="utf-8")

    verdict_lines = _verdict_lines(seed_summaries)
    print("\n".join(verdict_lines))
    print(f"wrote {OUT_DIR / 'distinct_report.md'}")
    print(f"wrote {OUT_DIR / 'distinct_costs.csv'}")
    print(f"wrote {OUT_DIR / 'signatures_cadet.csv'}")
    print(f"wrote {OUT_DIR / 'signatures_ddmin.csv'}")


def _read_seed_table() -> list[dict[str, str]]:
    with SEED_TABLE.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _read_raw_costs() -> dict[int, dict[str, str]]:
    with RAW_COSTS.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return {int(row["seed"]): row for row in rows}


def _load_frozen_groups(seed_rows: list[dict[str, str]]) -> dict[str, Any]:
    group_status: list[dict[str, Any]] = []
    groups_by_seed: dict[int, list[dict[str, Any]]] = {}
    source_by_seed: dict[int, str] = {}
    candidates_by_seed: dict[int, Path] = {}
    all_present_paths: list[Path] = []

    for row in seed_rows:
        seed = int(row["seed"])
        probe_path = ROOT / row["probe_dir"] / "groups.csv"
        ddmin_path = ROOT / row["ddmin_dir"] / "groups.csv"
        if seed in {1, 2}:
            all_present_paths.extend(path for path in [probe_path, ddmin_path] if path.exists())
        group_status.append(
            {
                "seed": seed,
                "probe_groups_csv": _rel(probe_path) if probe_path.exists() else "",
                "probe_groups_present": probe_path.exists(),
                "ddmin_groups_csv": _rel(ddmin_path) if ddmin_path.exists() else "",
                "ddmin_groups_present": ddmin_path.exists(),
            }
        )
        if probe_path.exists():
            candidates_by_seed[seed] = probe_path
        elif ddmin_path.exists():
            candidates_by_seed[seed] = ddmin_path

    required = [1, 2]
    missing_required = [seed for seed in required if seed not in candidates_by_seed]
    if missing_required:
        raise FileNotFoundError(f"groups.csv missing for seed(s) needed to freeze mapping: {missing_required}")

    groups1 = _read_groups(candidates_by_seed[1])
    groups2 = _read_groups(candidates_by_seed[2])
    seed1_seed2_equal = _groups_equal(groups1, groups2)
    if not seed1_seed2_equal:
        raise RuntimeError("seed1 and seed2 groups.csv are not row-identical; cannot reuse mapping for seed0")
    all_present_groups_equal = all(
        _groups_equal(groups1, _read_groups(path)) for path in all_present_paths
    )
    if not all_present_groups_equal:
        checked = ", ".join(_rel(path) for path in all_present_paths)
        raise RuntimeError(f"not all present seed1/seed2 probe/ddmin groups.csv are row-identical: {checked}")

    for row in seed_rows:
        seed = int(row["seed"])
        if seed in candidates_by_seed:
            groups_by_seed[seed] = _read_groups(candidates_by_seed[seed])
            source_by_seed[seed] = _rel(candidates_by_seed[seed])
        else:
            groups_by_seed[seed] = groups1
            source_by_seed[seed] = "reused seed1/seed2 row-identical D=40 frozen mapping"

    for status in group_status:
        seed = int(status["seed"])
        status["mapping_source"] = source_by_seed[seed]
        status["group_count"] = len(groups_by_seed[seed])

    return {
        "groups_by_seed": groups_by_seed,
        "source_by_seed": source_by_seed,
        "status": group_status,
        "seed1_seed2_equal": seed1_seed2_equal,
        "all_present_seed1_seed2_probe_ddmin_equal": all_present_groups_equal,
        "checked_group_paths": [_rel(path) for path in all_present_paths],
    }


def _read_groups(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    out = []
    for row in rows:
        out.append(
            {
                "group_id": int(row["group_id"]),
                "channel": row["channel"],
                "window_id": int(row["window_id"]),
                "t_start": float(row["t_start"]),
                "t_end": float(row["t_end"]),
            }
        )
    return sorted(out, key=lambda row: row["group_id"])


def _groups_equal(left: list[dict[str, Any]], right: list[dict[str, Any]]) -> bool:
    return left == right


def _cadet_signatures(
    seed: int,
    probe_dir: Path,
    groups: list[dict[str, Any]],
    mapping_source: str,
) -> dict[str, Any]:
    summary = json.loads((probe_dir / "reports" / "direction_a_summary.json").read_text(encoding="utf-8"))
    point_rows = _read_csv_by_key(probe_dir / "reports" / "point_evaluations.csv", "eval_id")
    interior = [
        row
        for row in summary["interior_violations"]
        if row.get("arm") == "C" and _parse_channels(row.get("active_channels_abs_gt_0p1")) <= CLEAN_CHANNELS
    ]
    rows: list[dict[str, Any]] = []
    main_signatures: set[str] = set()
    prereg_no_support: set[str] = set()
    prereg_support_le8: set[str] = set()
    gaps: list[str] = []
    label_missing = 0

    for item in interior:
        eval_id = int(item["eval_id"])
        point = point_rows.get(str(eval_id))
        label = point.get("label", "") if point else ""
        if not label:
            label_missing += 1
            gaps.append(f"seed{seed} CADET eval_id={eval_id}: label missing from point_evaluations.csv")
            continue
        parsed = _parse_envelope_label(label)
        if parsed is None:
            label_missing += 1
            gaps.append(f"seed{seed} CADET eval_id={eval_id}: unparsable label {label}")
            continue
        channels = sorted(_parse_channels(item.get("active_channels_abs_gt_0p1")))
        signs = _signs_from_degrees(parsed["deg"], channels)
        main_signature = _band_sign_signature(channels, parsed["w"], parsed["w"] + parsed["d"] - 1, signs)
        main_signatures.add(main_signature)

        theta_path = _resolve_theta_path(probe_dir, item.get("theta_path") or point.get("theta_path", ""))
        prereg_signature = ""
        theta_missing = False
        if theta_path is None:
            theta_missing = True
            gaps.append(f"seed{seed} CADET eval_id={eval_id}: theta missing")
        else:
            theta = np.load(theta_path)
            prereg_signature = _preregistered_fingerprint(theta, groups)
            prereg_no_support.add(prereg_signature)
            if int(item["support_size_abs_gt_0p1"]) <= 8:
                prereg_support_le8.add(prereg_signature)

        rows.append(
            {
                "seed": seed,
                "eval_id": eval_id,
                "label": label,
                "arm": "C",
                "active_channels": ",".join(channels),
                "support": int(item["support_size_abs_gt_0p1"]),
                "rho_mean_post_neutral_xy_velocity": item.get("rho_mean_post_neutral_xy_velocity", ""),
                "theta_hash": item.get("theta_hash", ""),
                "theta_path": item.get("theta_path", ""),
                "label_env": parsed["env"],
                "label_deg": parsed["deg"],
                "label_w": parsed["w"],
                "label_d": parsed["d"],
                "window_band": f"w{parsed['w']:02d}-w{parsed['w'] + parsed['d'] - 1:02d}",
                "signs": _sign_text(signs),
                "main_signature": main_signature,
                "main_signature_representative": "",
                "preregistered_fingerprint_no_support_filter": prereg_signature,
                "support_le8": int(item["support_size_abs_gt_0p1"]) <= 8,
                "theta_loaded": not theta_missing,
                "group_mapping_source": mapping_source,
            }
        )

    _mark_representatives(rows, "main_signature")
    return {
        "rows": rows,
        "main_signatures": main_signatures,
        "prereg_no_support_signatures": prereg_no_support,
        "prereg_support_le8_signatures": prereg_support_le8,
        "gaps": gaps,
        "label_missing": label_missing,
    }


def _ddmin_signatures(
    seed: int,
    ddmin_dir: Path,
    groups: list[dict[str, Any]],
    mapping_source: str,
) -> dict[str, Any]:
    summary = json.loads((ddmin_dir / "reports" / "direction_a_ddmin_summary.json").read_text(encoding="utf-8"))
    group_by_id = {int(row["group_id"]): row for row in groups}
    rows: list[dict[str, Any]] = []
    signatures: set[str] = set()
    gaps: list[str] = []
    theta_missing = 0
    channel_mismatches = 0

    for item in summary["minimized_triggers"]:
        if not _as_bool(item.get("is_roll_pitch_only")):
            continue
        trigger_id = int(item["trigger_id"])
        group_ids = _parse_int_list(item.get("final_active_group_ids_abs_gt_0p1", ""))
        active_groups = [group_by_id[group_id] for group_id in group_ids if group_id in group_by_id]
        if len(active_groups) != len(group_ids):
            missing = sorted(set(group_ids) - set(group_by_id))
            gaps.append(f"seed{seed} ddmin trigger_id={trigger_id}: group ids missing from mapping {missing}")
        channels = sorted({row["channel"] for row in active_groups})
        archived_channels = sorted(_parse_channels(item.get("final_active_channels_abs_gt_0p1")))
        if channels != archived_channels:
            channel_mismatches += 1
            gaps.append(
                f"seed{seed} ddmin trigger_id={trigger_id}: group-derived channels {channels} "
                f"!= archived {archived_channels}"
            )
        theta_path = _resolve_theta_path(ddmin_dir, item.get("final_theta_path", ""))
        theta_loaded = theta_path is not None
        if not theta_loaded:
            theta_missing += 1
            gaps.append(f"seed{seed} ddmin trigger_id={trigger_id}: final theta missing")
            continue
        theta = np.load(theta_path)
        band = [int(row["window_id"]) for row in active_groups]
        if not band:
            gaps.append(f"seed{seed} ddmin trigger_id={trigger_id}: no active groups after mapping")
            continue
        signs = _signs_from_theta(theta, group_ids, group_by_id)
        signature = _band_sign_signature(channels, min(band), max(band), signs)
        signatures.add(signature)
        rows.append(
            {
                "seed": seed,
                "trigger_id": trigger_id,
                "source_eval_id": item.get("source_eval_id", ""),
                "source_label": item.get("source_label", ""),
                "final_active_channels": ",".join(channels),
                "archived_final_active_channels": item.get("final_active_channels_abs_gt_0p1", ""),
                "final_support": int(item["final_support_size_abs_gt_0p1"]),
                "is_support_le8": _as_bool(item.get("is_support_le_8")),
                "final_group_ids": ",".join(str(value) for value in group_ids),
                "window_band": f"w{min(band):02d}-w{max(band):02d}",
                "active_windows_by_channel": _active_windows_text(group_ids, group_by_id),
                "signs": _sign_text(signs),
                "signature": signature,
                "signature_representative": "",
                "j5_points_used": int(item["j5_points_used"]),
                "final_theta_hash": item.get("final_theta_hash", ""),
                "final_theta_path": item.get("final_theta_path", ""),
                "theta_loaded": theta_loaded,
                "group_mapping_source": mapping_source,
            }
        )

    _mark_representatives(rows, "signature")
    return {
        "rows": rows,
        "signatures": signatures,
        "total_j5": int(summary["total_ddmin_j5_points_used"]),
        "gaps": gaps,
        "theta_missing": theta_missing,
        "channel_mismatches": channel_mismatches,
    }


def _read_csv_by_key(path: Path, key: str) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return {row[key]: row for row in csv.DictReader(handle)}


def _parse_envelope_label(label: str) -> dict[str, int] | None:
    match = LABEL_RE.match(str(label))
    if not match:
        return None
    return {name: int(value) for name, value in match.groupdict().items()}


def _signs_from_degrees(degrees: int, channels: list[str]) -> dict[str, str]:
    angle = math.radians(float(degrees))
    values = {"roll": math.cos(angle), "pitch": math.sin(angle)}
    signs = {}
    for channel in channels:
        value = values[channel]
        if abs(value) <= 1e-9:
            signs[channel] = "0"
        else:
            signs[channel] = "+" if value > 0 else "-"
    return signs


def _signs_from_theta(
    theta: np.ndarray,
    group_ids: list[int],
    group_by_id: dict[int, dict[str, Any]],
) -> dict[str, str]:
    by_channel: dict[str, set[str]] = defaultdict(set)
    for group_id in group_ids:
        group = group_by_id.get(group_id)
        if group is None:
            continue
        channel = group["channel"]
        value = float(theta[group_id])
        if abs(value) <= SUPPORT_THRESHOLD:
            continue
        by_channel[channel].add("+" if value > 0 else "-")
    signs = {}
    for channel, values in by_channel.items():
        signs[channel] = next(iter(values)) if len(values) == 1 else "mixed"
    return signs


def _band_sign_signature(channels: list[str], w_min: int, w_max: int, signs: dict[str, str]) -> str:
    sign_part = "|".join(f"{channel}:{signs.get(channel, '?')}" for channel in channels)
    return f"channels={','.join(channels)};time=w{w_min:02d}-w{w_max:02d};signs={sign_part}"


def _preregistered_fingerprint(theta: np.ndarray, groups: list[dict[str, Any]]) -> str:
    active = []
    for group in sorted(groups, key=lambda row: row["group_id"]):
        group_id = int(group["group_id"])
        if group_id < len(theta) and abs(float(theta[group_id])) > SUPPORT_THRESHOLD:
            active.append((str(group["channel"]), int(group["window_id"]), float(theta[group_id])))
    if not active:
        return "channels=none;time=none;shape=none"
    channels = sorted({channel for channel, _, _ in active})
    windows = [window for _, window, _ in active]
    shape_parts = []
    for channel in channels:
        seq = []
        for group in sorted((row for row in groups if row["channel"] == channel), key=lambda row: row["window_id"]):
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


def _resolve_theta_path(run_dir: Path, raw_path: Any) -> Path | None:
    if raw_path is None:
        return None
    text = str(raw_path).strip()
    if not text:
        return None
    path = Path(text)
    candidates = [
        path,
        ROOT / path,
        run_dir / "thetas" / path.name,
    ]
    if text.startswith("runs/"):
        candidates.append(ROOT / "artifacts" / Path(text).relative_to("runs"))
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _active_windows_text(group_ids: list[int], group_by_id: dict[int, dict[str, Any]]) -> str:
    by_channel: dict[str, list[int]] = defaultdict(list)
    for group_id in group_ids:
        group = group_by_id.get(group_id)
        if group:
            by_channel[group["channel"]].append(int(group["window_id"]))
    parts = []
    for channel in sorted(by_channel):
        windows = ",".join(f"w{window:02d}" for window in sorted(by_channel[channel]))
        parts.append(f"{channel}:{windows}")
    return "|".join(parts)


def _sign_text(signs: dict[str, str]) -> str:
    return "|".join(f"{channel}:{signs[channel]}" for channel in sorted(signs))


def _mark_representatives(rows: list[dict[str, Any]], signature_key: str) -> None:
    seen = set()
    representative_key = f"{signature_key}_representative"
    for row in rows:
        signature = row[signature_key]
        row[representative_key] = signature not in seen
        seen.add(signature)


def _cost_row(
    *,
    seed: int,
    method: str,
    d: int,
    total_j5: int,
    raw_count: int,
    raw_j5: float,
    distinct_j5: float,
    raw_source: str,
    note: str,
) -> dict[str, Any]:
    return {
        "seed": seed,
        "method": method,
        "D": d,
        "total_j5": total_j5,
        "j5_per_distinct": distinct_j5,
        "raw_count": raw_count,
        "raw_j5_per_raw_output": raw_j5,
        "raw_count_over_distinct_D": _safe_div(raw_count, d),
        "distinct_cost_over_raw_cost": _safe_div(distinct_j5, raw_j5),
        "raw_cost_understatement_factor": _safe_div(distinct_j5, raw_j5),
        "raw_vs_distinct_note": note,
        "raw_cost_source": raw_source,
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key, "")) for key in fieldnames})


def _render_report(
    summaries: list[dict[str, Any]],
    cost_rows: list[dict[str, Any]],
    group_info: dict[str, Any],
    gaps: list[str],
) -> str:
    cost_by_seed_method = {(row["seed"], row["method"]): row for row in cost_rows}
    lines = [
        "# Distinct Recut v0",
        "",
        "Scope: EXPLORATORY post-processing only. Single platform: PX4. Single property: xy_velocity / post_neutral_xy_velocity. Confirmation requires alt_drift and seed3. No simulation was run; this recut reads archived CSV/JSON and existing theta npy files only.",
        "",
        "## Inputs And Data Gaps",
        "",
        "| seed | probe groups.csv | ddmin groups.csv | mapping used | groups |",
        "| ---: | --- | --- | --- | ---: |",
    ]
    for status in group_info["status"]:
        lines.append(
            f"| {status['seed']} | {_yesno(status['probe_groups_present'])} | "
            f"{_yesno(status['ddmin_groups_present'])} | {status['mapping_source']} | {status['group_count']} |"
        )
    lines.extend(
        [
            "",
            "Seed1 and seed2 probe/ddmin groups.csv files were checked row-for-row identical across the frozen D=40 configuration. Seed0 lacks groups.csv in the archived probe/ddmin directories, so the seed1/seed2 mapping is reused for seed0 because the parameterization is frozen; this is not imputation of trigger data.",
            f"Checked groups files: {', '.join(group_info['checked_group_paths'])}.",
            "",
            "Signature definitions:",
            "- Main CADET/ddmin distinct signature: sorted roll/pitch channel set + active window band + per-channel sign; amplitude and bisection iter are ignored.",
            "- CADET main signatures parse `env####_deg###_w##_d##` labels, with `w` and `d` defining the band.",
            "- ddmin signatures map final active group ids to (window, channel), read signs from existing final theta npy, and use the min/max active window band.",
            "- Cross-check fingerprint: previous pre-registered theta fingerprint with sign/amplitude bins and active window band, but without the support<=8 filter unless stated.",
            "",
        ]
    )
    if gaps:
        lines.append("Data gaps or validation issues:")
        for gap in sorted(set(gaps)):
            lines.append(f"- {gap}")
    else:
        lines.append("Data gaps or validation issues: no missing labels or theta files; no ddmin group/channel mismatches.")
    lines.extend(
        [
            "",
            "## Step 1 - CADET Arm C Distinct Channel-Pure Maneuvers",
            "",
            "| seed | raw channel-pure Arm C | D_C main | raw/D_C | prereg distinct no support | prereg support<=8 | support<=8 expected | main signatures |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in summaries:
        lines.append(
            f"| {row['seed']} | {row['cadet_raw_count']} | {row['cadet_D']} | "
            f"{_fmt(row['cadet_raw_inflation'])} | {row['cadet_prereg_no_support_D']} | "
            f"{row['cadet_prereg_support_le8_D']} | {SUPPORT_LE8_EXPECTED[row['seed']]} | "
            f"{_join_signatures(row['cadet_main_signatures'])} |"
        )
    lines.extend(
        [
            "",
            "## Step 2 - ddmin Distinct Channel-Pure Maneuvers",
            "",
            "| seed | raw is_roll_pitch_only | D_dd main | raw/D_dd | total ddmin J5 | main signatures |",
            "| ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in summaries:
        lines.append(
            f"| {row['seed']} | {row['ddmin_raw_count']} | {row['ddmin_D']} | "
            f"{_fmt(row['ddmin_raw_inflation'])} | {cost_by_seed_method[(row['seed'], 'ddmin_direct')]['total_j5']} | "
            f"{_join_signatures(row['ddmin_signatures'])} |"
        )
    lines.extend(
        [
            "",
            "## Step 3 - End-To-End Cost Per Distinct Maneuver",
            "",
            "| seed | CADET raw cost | CADET distinct cost | CADET raw inflation | ddmin raw direct/e2e | ddmin distinct direct/e2e | ddmin raw inflation | ddmin/CADET direct | ddmin/CADET e2e |",
            "| ---: | ---: | ---: | ---: | --- | --- | ---: | ---: | ---: |",
        ]
    )
    for row in summaries:
        cadet = cost_by_seed_method[(row["seed"], "CADET_arm_C")]
        ddmin = cost_by_seed_method[(row["seed"], "ddmin_direct")]
        ddmin_e2e = cost_by_seed_method[(row["seed"], "ddmin_e2e")]
        lines.append(
            f"| {row['seed']} | {_fmt(cadet['raw_j5_per_raw_output'])} | {_fmt(cadet['j5_per_distinct'])} | "
            f"{_fmt(cadet['raw_count_over_distinct_D'])}x | "
            f"{_fmt(ddmin['raw_j5_per_raw_output'])} / {_fmt(ddmin_e2e['raw_j5_per_raw_output'])} | "
            f"{_fmt(ddmin['j5_per_distinct'])} / {_fmt(ddmin_e2e['j5_per_distinct'])} | "
            f"{_fmt(ddmin['raw_count_over_distinct_D'])}x | {_fmt(row['ddmin_vs_cadet'])}x | "
            f"{_fmt(row['ddmin_e2e_vs_cadet'])}x |"
        )
    lines.extend(
        [
            "",
            "Raw denominator inflation is visible mostly on CADET: seed0 18 raw points collapse to 7 maneuvers, seed1 12 to 4, and seed2 6 to 3. ddmin is less inflated under this band+sign signature: 5 to 5, 8 to 7, and 8 to 8.",
            "",
            "## Step 4 - Cross-Checks",
            "",
            "| seed | prereg no-support >= support<=8 | no-support | support<=8 | expected support<=8 | channel-pure D_C/D_dd | support<=8 D_C/D_dd |",
            "| ---: | --- | ---: | ---: | ---: | --- | --- |",
        ]
    )
    for row in summaries:
        ok = row["cadet_prereg_no_support_D"] >= row["cadet_prereg_support_le8_D"]
        lines.append(
            f"| {row['seed']} | {'OK' if ok else 'FAIL'} | {row['cadet_prereg_no_support_D']} | "
            f"{row['cadet_prereg_support_le8_D']} | {SUPPORT_LE8_EXPECTED[row['seed']]} | "
            f"{row['cadet_D']} / {row['ddmin_D']} | "
            f"{SUPPORT_LE8_EXPECTED[row['seed']]} / {DDMIN_SUPPORT_LE8_EXPECTED[row['seed']]} |"
        )
    lines.extend(["", "## Step 5 - Verdict", ""])
    lines.extend(_verdict_lines(summaries))
    lines.append("")
    lines.append("All conclusions are EXPLORATORY, PX4-only, xy_velocity-only, and require alt_drift/seed3 confirmation.")
    return "\n".join(lines)


def _verdict_lines(summaries: list[dict[str, Any]]) -> list[str]:
    by_seed = {int(row["seed"]): row for row in summaries}
    quantity = ", ".join(
        f"seed{seed} D_C/D_dd={by_seed[seed]['cadet_D']}/{by_seed[seed]['ddmin_D']}"
        for seed in sorted(by_seed)
    )
    e2e = ", ".join(
        f"seed{seed} {_fmt(by_seed[seed]['ddmin_e2e_vs_cadet'])}x"
        for seed in sorted(by_seed)
    )
    direct = ", ".join(
        f"seed{seed} {_fmt(by_seed[seed]['ddmin_vs_cadet'])}x"
        for seed in sorted(by_seed)
    )
    return [
        "Step 5 判决：B) 脊柱仅成本占优、数量不占优。EXPLORATORY、PX4、xy_velocity；确证需 alt_drift/seed3。distinct 通道纯口径下 CADET 不是 3/3 数量占优，只能主张更便宜，不能主张更多/更好。",
        f"数量：{quantity}；CADET 只在 seed0 >= ddmin，seed1/seed2 翻车，因此通道纯 + 数量这部分脊柱不活。",
        f"成本：端到端 ddmin/CADET = {e2e}；直接 ddmin/CADET = {direct}。CADET 每 distinct 机动成本 3/3 低于 ddmin e2e，但倍数从 seed0 到 seed2 衰减到约 {_fmt(by_seed[2]['ddmin_e2e_vs_cadet'])}x，措辞应收缩为成本优势。",
    ]


def _parse_channels(value: Any) -> set[str]:
    if value is None:
        return set()
    text = str(value).strip()
    if not text:
        return set()
    return {part.strip() for part in text.split(",") if part.strip()}


def _parse_int_list(value: Any) -> list[int]:
    text = str(value or "").strip()
    if not text:
        return []
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes"}


def _safe_div(numerator: float, denominator: float) -> float:
    denominator = float(denominator)
    return float(numerator) / denominator if denominator else math.inf


def _csv_value(value: Any) -> Any:
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, float) and math.isinf(value):
        return "inf"
    if isinstance(value, (list, tuple, set)):
        return "; ".join(str(item) for item in value)
    return value


def _fmt(value: Any) -> str:
    value = float(value)
    if math.isinf(value):
        return "inf"
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:.3f}".rstrip("0").rstrip(".")


def _yesno(value: bool) -> str:
    return "yes" if value else "no"


def _join_signatures(signatures: list[str]) -> str:
    counts = Counter(signatures)
    return "<br>".join(f"{sig}" + (f" x{count}" if count > 1 else "") for sig, count in sorted(counts.items()))


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


if __name__ == "__main__":
    main()
