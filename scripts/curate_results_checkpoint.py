from __future__ import annotations

import csv
import json
import math
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "runs"
ARTIFACTS = ROOT / "artifacts"

CAVEAT_ZH = (
    "PX4 px4_position（POSCTL）seed-0 单种子/单场景探针；"
    "多种子、第二属性、ArduPilot 复现为投稿前必做。"
)

BANNED_SUFFIXES = {".ulg", ".bin", ".parquet", ".npz"}
MAX_COPY_BYTES = 5 * 1024 * 1024
TEXT_SUFFIXES = {".csv", ".json", ".md", ".svg"}


def ensure_clean_small_file(src: Path) -> None:
    suffix = src.suffix.lower()
    if suffix in BANNED_SUFFIXES:
        raise RuntimeError(f"refusing to copy banned artifact: {src}")
    if src.stat().st_size > MAX_COPY_BYTES:
        raise RuntimeError(f"refusing to copy large artifact: {src}")


def copy_file(src: Path, dst: Path) -> None:
    ensure_clean_small_file(src)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    normalize_text_file(dst)


def normalize_text_file(path: Path) -> None:
    if path.suffix.lower() not in TEXT_SUFFIXES:
        return
    text = path.read_text(encoding="utf-8")
    normalized = "\n".join(line.rstrip() for line in text.splitlines()) + "\n"
    path.write_text(normalized, encoding="utf-8")


def copy_reports(src_run: Path, dst_run: Path) -> None:
    if (src_run / "groups.csv").exists():
        copy_file(src_run / "groups.csv", dst_run / "groups.csv")
    reports = src_run / "reports"
    if not reports.exists():
        return
    for src in sorted(reports.iterdir()):
        if src.is_file() and src.suffix.lower() in {".csv", ".json", ".md"}:
            copy_file(src, dst_run / "reports" / src.name)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def save_figure(fig: plt.Figure, path_no_suffix: Path) -> None:
    path_no_suffix.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path_no_suffix.with_suffix(".png"), dpi=180)
    fig.savefig(path_no_suffix.with_suffix(".svg"))
    normalize_text_file(path_no_suffix.with_suffix(".svg"))
    plt.close(fig)


def copy_sph_plateau() -> Path:
    src = RUNS / "archive/rq1_zero_theta_sph_rejected/legacy_rq1_minimal_v0"
    dst = ARTIFACTS / "sph_plateau/legacy_rq1_minimal_v0"
    report_names = [
        "phase2_px4_seed0_j5_denoise_denoised_metrics.csv",
        "phase2_px4_seed0_j5_denoise_runtime_mode_metadata.csv",
        "phase2_px4_seed0_j5_denoise_seed0_mode_conditioned_denoised.csv",
        "phase2_px4_seed0_j5_denoise_significance_diagnostics.csv",
        "phase2_px4_seed0_j5_denoise_topk_coverage_curve.csv",
        "persistence_pilot_px4_seed0_robustness.csv",
        "persistence_pilot_px4_seed0_snapshot_validation.csv",
        "persistence_pilot_px4_seed0_step_metrics.csv",
        "persistence_pilot_px4_seed0_transitions.csv",
    ]
    for name in report_names:
        copy_file(src / "reports" / name, dst / "reports" / name)
    copy_file(
        src / "paths/px4_position_seed0_persistence_pilot_px4_seed0/pilot_summary.json",
        dst / "reports/persistence_pilot_px4_seed0_pilot_summary.json",
    )
    copy_file(
        src
        / "paths/px4_position_seed0_persistence_pilot_px4_seed0/path_xy_drift/path_summary.json",
        dst / "reports/persistence_pilot_px4_seed0_path_xy_drift_summary.json",
    )
    copy_file(
        src
        / "paths/px4_position_seed0_persistence_pilot_px4_seed0/path_xy_velocity/path_summary.json",
        dst / "reports/persistence_pilot_px4_seed0_path_xy_velocity_summary.json",
    )
    write_text(
        dst / "README.md",
        f"""# SPH Plateau and Persistence: legacy_rq1_minimal_v0

Claim: safe-region finite differences behave like a plateau/noise measurement,
and support persistence fails along short projected paths.

Runner lineage: `cadet.runners.repeated_fd` and `cadet.runners.persistence_pilot`.
Original local run path: `runs/archive/rq1_zero_theta_sph_rejected/legacy_rq1_minimal_v0`.

Caveat: **{CAVEAT_ZH}**

Included small artifacts are J=5 denoised metrics, persistence step metrics,
transition overlap tables, and persistence summaries. Raw simulator logs,
query caches, seed-1 diagnostics, and ArduPilot diagnostic outputs are excluded.
The `noise≈282×step` prose number is not present as a named field in the
archived small files; this checkpoint keeps the underlying noise estimates but
does not re-state that ratio as a curated headline.
""",
    )
    return dst


def copy_boundary_anisotropy() -> Path:
    src = RUNS / "margin_stage1_redo_v1"
    dst = ARTIFACTS / "boundary_anisotropy/margin_stage1_redo_v1"
    copy_reports(src, dst)
    for name in ["theta_V.npy", "theta_117.npy"]:
        copy_file(src / name, dst / "thetas" / name)
    write_text(
        dst / "README.md",
        f"""# Boundary Anisotropy: margin_stage1_redo_v1

Claim: the boundary normal is channel-anisotropic at the δ=0.2 redo point.

Runner: `cadet.runners.margin_stage1_redo`.
Command:

```bash
python -m cadet.runners.margin_stage1_redo \\
  --config configs/rq1_minimal.yaml \\
  --scenario px4_position \\
  --seed 0 \\
  --run-dir runs/margin_stage1_redo_v1
```

Caveat: **{CAVEAT_ZH}**

Included small artifacts are report CSV/JSON files and the Point V theta
anchors. Raw `*.ulg` logs and per-query caches are excluded.
""",
    )
    return dst


def copy_h2_warmstart() -> Path:
    src = RUNS / "route1_h2_px4_position_seed0_vmax_pilot_v0"
    dst = ARTIFACTS / "h2_warmstart/route1_h2_px4_position_seed0_vmax_pilot_v0"
    copy_reports(src, dst)
    copy_file(src / "theta_V_condition1_anchor.npy", dst / "thetas/theta_V_condition1_anchor.npy")
    for src_theta in sorted((src / "boundaries").glob("*.npy")):
        copy_file(src_theta, dst / "thetas" / src_theta.name)
    write_text(
        dst / "README.md",
        f"""# H2 Warm-Start Pilot: route1_h2_px4_position_seed0_vmax_pilot_v0

Claim: cross-condition warm starts do not save enough queries in this seed-0
v_max pilot.

Runner: `cadet.runners.route1_h2_campaign`.
Command:

```bash
python -m cadet.runners.route1_h2_campaign \\
  --config configs/rq1_minimal.yaml \\
  --scenario px4_position \\
  --seed 0 \\
  --theta-v artifacts/margin_stage1_redo_v1/theta_V.npy \\
  --run-dir runs/route1_h2_px4_position_seed0_vmax_pilot_v0
```

Caveat: **{CAVEAT_ZH}**

Included small artifacts are campaign report CSV/JSON files plus canonical
boundary theta files. Raw logs and query caches are excluded.
""",
    )
    return dst


def copy_h3_transition() -> Path:
    src = RUNS / "h3_transition_seed0_v1"
    dst = ARTIFACTS / "h3_transition/h3_transition_seed0_v1"
    copy_reports(src, dst)
    write_text(
        dst / "README.md",
        f"""# H3 Transition Handoff: h3_transition_seed0_v1

Claim: the POSCTL-to-AUTO.LOITER handoff produced no robust transition-specific
violations in this seed-0 probe.

Runner: `cadet.runners.h3_transition`.
Command:

```bash
python -m cadet.runners.h3_transition \\
  --config configs/rq1_minimal.yaml \\
  --seed 0 \\
  --run-dir runs/h3_transition_seed0_v1
```

Caveat: **{CAVEAT_ZH}** The H3 runner uses `px4_transition`, but this result is
still only a seed-0 PX4 probe.

Included small artifacts are H3 report CSV/JSON files. Raw logs and query
caches are excluded.
""",
    )
    return dst


def copy_rq1_three_arm() -> Path:
    src = RUNS / "direction_a_px4_position_seed0_v0"
    dst = ARTIFACTS / "rq1_three_arm/direction_a_px4_position_seed0_v0"
    copy_reports(src, dst)
    selected_thetas = [
        "B_00151_a640eec192e42a4d.npy",
        "C_00186_115d1049daf89b7a.npy",
        "C_00196_9dfde040ab231b58.npy",
    ]
    for name in selected_thetas:
        copy_file(src / "thetas" / name, dst / "thetas" / name)

    example_query = (
        src
        / "queries/9dfde040ab231b58_px4_position_0_direction_a_C_amplitude_bisection_00196_env0164_deg180_w06_d04_iter05_a0p4844_repeat3"
    )
    for name in ["input_sequence.csv", "parsed_log.csv", "robustness.json", "metadata.json"]:
        copy_file(example_query / name, dst / "minimal_trigger_example/eval196_repeat3" / name)

    write_text(
        dst / "README.md",
        f"""# RQ1 Three-Arm Direction-A Probe: direction_a_px4_position_seed0_v0

Claim: channel-directed search exposes a thin target that random and
channel-agnostic search do not expose cleanly.

Runner: `cadet.runners.direction_a_probe`.
Command:

```bash
python -m cadet.runners.direction_a_probe \\
  --config configs/rq1_minimal.yaml \\
  --scenario px4_position \\
  --seed 0 \\
  --run-dir runs/direction_a_px4_position_seed0_v0
```

Caveat: **{CAVEAT_ZH}**

Included small artifacts are report CSV/JSON files, pre-registration, three
representative theta files, and one small parsed minimal-trigger example used
for the paper-hook figure. Raw `*.ulg` logs and full per-query caches are
excluded.
""",
    )
    return dst


def copy_rq2_ddmin() -> Path:
    src = RUNS / "direction_a_ddmin_px4_position_seed0_v1"
    dst = ARTIFACTS / "rq2_ddmin/direction_a_ddmin_px4_position_seed0_v1"
    copy_reports(src, dst)
    for src_theta in sorted((src / "thetas").glob("T??_final_*.npy")):
        copy_file(src_theta, dst / "thetas" / src_theta.name)
    write_text(
        dst / "README.md",
        f"""# RQ2 ddmin Necessity Baseline: direction_a_ddmin_px4_position_seed0_v1

Claim: post-hoc channel-agnostic delta debugging cannot match channel-directed
search in this seed-0 probe.

Runner: `cadet.runners.direction_a_ddmin`.
Command:

```bash
python -m cadet.runners.direction_a_ddmin \\
  --config configs/rq1_minimal.yaml \\
  --scenario px4_position \\
  --seed 0 \\
  --probe-dir runs/direction_a_px4_position_seed0_v0 \\
  --run-dir runs/direction_a_ddmin_px4_position_seed0_v1
```

Caveat: **{CAVEAT_ZH}**

Included small artifacts are report CSV/JSON files, pre-registration, and the
ten final minimized theta files. Raw logs and full query caches are excluded.
""",
    )
    return dst


def plot_sph(dst: Path) -> None:
    metrics = pd.read_csv(dst / "reports/phase2_px4_seed0_j5_denoise_denoised_metrics.csv")
    pos = metrics[metrics["scenario_id"] == "px4_position"].copy()
    pos["label"] = pos["property"].str.replace("post_neutral_", "", regex=False)

    pilot = read_json(dst / "reports/persistence_pilot_px4_seed0_pilot_summary.json")
    transitions = pd.read_csv(dst / "reports/persistence_pilot_px4_seed0_transitions.csv")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    ax = axes[0]
    ax.bar(pos["label"], pos["effective_sparsity_mean"], color="#4c78a8")
    ax.axhline(25.8, color="#222222", linestyle="--", linewidth=1.2, label="40-D noise baseline (~25.8)")
    ax.set_ylabel("effective sparsity")
    ax.set_ylim(0, 32)
    ax.set_title("Safe-region FD is diffuse")
    ax.tick_params(axis="x", rotation=25)
    ax.legend(fontsize=8)

    ax = axes[1]
    overlaps = transitions["mass_overlap_top16"].astype(float)
    labels = [f"{r.path_id}\n{int(r.step_r)}->{int(r.step_next)}" for r in transitions.itertuples()]
    ax.bar(range(len(overlaps)), overlaps, color="#f58518")
    ax.axhline(0.40, color="#222222", linestyle="--", linewidth=1.2, label="random baseline (~0.40)")
    ax.axhline(float(pilot["median_mass_overlap_top16"]), color="#b00020", linestyle=":", linewidth=1.5, label="median 0.375")
    ax.set_xticks(range(len(overlaps)), labels=labels, rotation=20, ha="right")
    ax.set_ylim(0, 0.55)
    ax.set_ylabel("top16 mass overlap")
    ax.set_title("Support persistence fails")
    ax.legend(fontsize=8)
    save_figure(fig, dst / "figures/sph_plateau_persistence")


def plot_boundary(dst: Path) -> None:
    summary = read_json(dst / "reports/stage1_redo_summary.json")
    channel = pd.read_csv(dst / "reports/redo_pointV_delta020_channel_marginal.csv")
    window = pd.read_csv(dst / "reports/redo_pointV_delta020_window_marginal.csv").sort_values("window_id")

    roll_pitch = channel[channel["channel"].isin(["roll", "pitch"])]["share"].sum()
    channel_pr = summary["directional_probe_summary"]["channel_participation_ratio"]
    window_pr = summary["directional_probe_summary"]["window_participation_ratio"]

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    axes[0].bar(channel["channel"], channel["share"], color=["#4c78a8", "#72b7b2", "#f58518", "#b279a2"])
    axes[0].set_ylim(0, 0.55)
    axes[0].set_ylabel("directional sensitivity share")
    axes[0].set_title(f"roll+pitch = {roll_pitch:.1%}; PR = {channel_pr:.2f}/4")

    axes[1].bar(window["window_id"].astype(str), window["share"], color="#54a24b")
    axes[1].set_ylabel("directional sensitivity share")
    axes[1].set_xlabel("window id")
    axes[1].set_title(f"time is dense; PR = {window_pr:.2f}/10")
    save_figure(fig, dst / "figures/boundary_channel_window_anisotropy")


def plot_h2(dst: Path) -> None:
    summary = read_json(dst / "reports/route1_h2_summary.json")
    rows = summary["campaign_query_ratios"]
    labels = [r["cold_baseline"] for r in rows]
    speedups = [r["cold_over_warm_speedup"] for r in rows]

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(labels, speedups, color=["#4c78a8", "#f58518", "#54a24b"])
    ax.axhline(2.0, color="#222222", linestyle="--", linewidth=1.2, label="pre-registered 2x bar")
    ax.set_ylim(0, 2.2)
    ax.set_ylabel("cold-over-warm speedup")
    ax.set_title("Warm start does not clear the 2x query-saving bar")
    for i, v in enumerate(speedups):
        ax.text(i, v + 0.04, f"{v:.2f}x", ha="center", fontsize=9)
    ax.legend(fontsize=8)
    save_figure(fig, dst / "figures/h2_warmstart_speedups")


def plot_h3(dst: Path) -> None:
    summary = read_json(dst / "reports/h3_summary.json")
    stage_a = summary["stage_a"]
    labels = ["robust transition\nviolations", "weak 1std\ncandidates", "noise straddles\nnot counted"]
    values = [
        stage_a["robust_transition_violation_count"],
        stage_a["weak_1std_candidate_count"],
        stage_a["noise_straddle_not_counted_count"],
    ]

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(labels, values, color=["#4c78a8", "#f58518", "#bab0ac"])
    ax.set_ylabel("count")
    ax.set_title(f"H3 stage A: 0 robust transition violations in {stage_a['query_count_after_stage_a']} queries")
    for i, v in enumerate(values):
        ax.text(i, v + 0.4, str(v), ha="center", fontsize=9)
    ax.set_ylim(0, max(values) + 5)
    save_figure(fig, dst / "figures/h3_zero_transition_violations")


def plot_rq1(dst: Path) -> None:
    arm = pd.read_csv(dst / "reports/arm_metrics.csv")
    interior = pd.read_csv(dst / "reports/interior_violations.csv")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    axes[0].bar(arm["arm"], arm["interior_robust_violation_count"], color=["#4c78a8", "#f58518", "#54a24b"])
    axes[0].set_ylabel("interior robust violations")
    axes[0].set_title("Thin target: A/B/C = 0/7/18")
    for i, v in enumerate(arm["interior_robust_violation_count"]):
        axes[0].text(i, int(v) + 0.4, str(int(v)), ha="center", fontsize=10)

    supports = [interior[interior["arm"] == a]["support_size_abs_gt_0p1"].astype(float).values for a in ["B", "C"]]
    axes[1].boxplot(supports, tick_labels=["Arm B", "Arm C"], vert=True, patch_artist=True)
    for i, vals in enumerate(supports, start=1):
        axes[1].scatter(np.full(len(vals), i), vals, alpha=0.7, color="#4c78a8" if i == 1 else "#54a24b", s=25)
    axes[1].set_ylabel("support size |theta| > 0.1")
    axes[1].set_title("Arm B dense (~27); Arm C clean (4-8)")
    save_figure(fig, dst / "figures/rq1_three_arm_counts_support")


def plot_money_trigger(dst: Path) -> None:
    example = dst / "minimal_trigger_example/eval196_repeat3"
    seq = pd.read_csv(example / "input_sequence.csv")
    log = pd.read_csv(example / "parsed_log.csv")
    point = pd.read_csv(dst / "reports/point_evaluations.csv")
    row = point[point["eval_id"] == 196].iloc[0]
    log["xy_speed_mps"] = np.sqrt(log["vx_mps"].astype(float) ** 2 + log["vy_mps"].astype(float) ** 2)
    rho_mean = float(row["rho_mean_post_neutral_xy_velocity"])
    rho_std = float(row["rho_std_post_neutral_xy_velocity"])
    robust_margin = rho_mean + 2.0 * rho_std
    max_idx = log["xy_speed_mps"].idxmax()
    max_row = log.loc[max_idx]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(seq["t_s"], seq["roll"], label="roll", color="#4c78a8", linewidth=2)
    axes[0].plot(seq["t_s"], seq["pitch"], label="pitch", color="#f58518", linewidth=1.2, alpha=0.75)
    axes[0].axvline(5.0, color="#222222", linestyle="--", linewidth=1.0, label="release to neutral")
    axes[0].set_ylabel("stick value")
    axes[0].set_title("Minimal clean trigger: four-cell roll pulse, then neutral")
    axes[0].legend(fontsize=8, loc="upper left")
    axes[0].grid(True, axis="y", alpha=0.25)

    axes[1].plot(log["time_s"], log["xy_speed_mps"], color="#b00020", linewidth=2)
    axes[1].axhline(1.0, color="#222222", linestyle="--", linewidth=1.2, label="1.0 m/s threshold")
    axes[1].axvline(5.0, color="#222222", linestyle="--", linewidth=1.0)
    axes[1].scatter([max_row["time_s"]], [max_row["xy_speed_mps"]], color="#b00020", zorder=3)
    axes[1].annotate(
        f"max {max_row['xy_speed_mps']:.3f} m/s",
        xy=(max_row["time_s"], max_row["xy_speed_mps"]),
        xytext=(max_row["time_s"] + 0.5, max_row["xy_speed_mps"] + 0.05),
        arrowprops={"arrowstyle": "->", "linewidth": 0.8},
        fontsize=9,
    )
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("post-neutral xy speed (m/s)")
    axes[1].set_title(
        f"rho={rho_mean:.3f}, std={rho_std:.3f}, rho+2sigma={robust_margin:.3f}; channel=roll, support=4"
    )
    axes[1].legend(fontsize=8, loc="upper right")
    axes[1].grid(True, axis="y", alpha=0.25)
    axes[1].set_xlim(0, 13)
    save_figure(fig, dst / "figures/minimal_trigger_roll_pulse_xy_velocity")


def plot_rq2(dst: Path) -> None:
    summary = read_json(dst / "reports/direction_a_ddmin_summary.json")
    triggers = pd.read_csv(dst / "reports/minimized_triggers.csv")

    clean = int(summary["clean_trigger_count"])
    total = len(triggers)
    not_clean = total - clean
    ddmin_median = float(summary["final_support_distribution"]["median"])
    arm_c_median = float(summary["arm_c_comparison"]["support_distribution"]["median"])
    cost_ratio = float(summary["cost_ratio_ddmin_per_clean_vs_arm_c_per_interior"])

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    axes[0].bar(["clean", "not clean"], [clean, not_clean], color=["#54a24b", "#bab0ac"])
    axes[0].set_title("ddmin clean yield = 4/10")
    axes[0].set_ylim(0, total)
    for i, v in enumerate([clean, not_clean]):
        axes[0].text(i, v + 0.2, str(v), ha="center", fontsize=9)

    axes[1].bar(["ddmin final", "Arm C"], [ddmin_median, arm_c_median], color=["#f58518", "#54a24b"])
    axes[1].set_ylabel("median support")
    axes[1].set_title("Support stays larger")
    for i, v in enumerate([ddmin_median, arm_c_median]):
        axes[1].text(i, v + 0.4, f"{v:.1f}", ha="center", fontsize=9)

    axes[2].bar(["cost ratio"], [cost_ratio], color="#4c78a8")
    axes[2].axhline(10, color="#222222", linestyle="--", linewidth=1.0, label="10x")
    axes[2].set_ylim(0, max(24, cost_ratio + 2))
    axes[2].set_title("Cost per clean trigger")
    axes[2].text(0, cost_ratio + 0.7, f"{cost_ratio:.1f}x", ha="center", fontsize=9)
    axes[2].legend(fontsize=8)
    save_figure(fig, dst / "figures/rq2_ddmin_clean_support_cost")


def write_results_index(paths: dict[str, Path]) -> None:
    write_text(
        ARTIFACTS / "README.md",
        f"""# CADET Result Artifacts

This directory freezes the current seed-0 CADET result checkpoint: small
CSV/JSON/theta artifacts plus document-ready figures generated from those
small artifacts. Raw simulator logs remain excluded under `runs/`.

Global caveat: **{CAVEAT_ZH}**

## Safe Region Is a Plateau; Persistence Fails

Claim: safe-region finite differences are diffuse/noise-like, and the measured
support is not persistent enough to guide a sparse global search.

Caveat: **{CAVEAT_ZH}**

![SPH plateau and persistence](sph_plateau/legacy_rq1_minimal_v0/figures/sph_plateau_persistence.png)

Key numbers:

- Effective sparsity for PX4 position J=5 denoise: `26.08`, `23.09`, `23.84`
  across the three post-neutral properties; this sits near the 40-D pure-noise
  reference level of about `25`.
- Persistence median top16 mass overlap: `0.375410262484114`, near the random
  baseline cited in the narrative.
- Source run: `runs/archive/rq1_zero_theta_sph_rejected/legacy_rq1_minimal_v0`.
- Runner lineage: `cadet.runners.repeated_fd` and `cadet.runners.persistence_pilot`.
- Supports paper narrative §3.1 / SPH negative result.

Gap: the prose number `noise≈282×step` is not present as a named field in the
local small artifacts found during this curation pass. The archived CSVs keep
the underlying `estimated_noise_l1_before/after` values, but this checkpoint
does not promote the 282x ratio as a curated headline until provenance is found.

## Boundary Channel Anisotropy

Claim: once measured at the δ=0.2 redo boundary point, sensitivity is sparse in
channels but dense in time.

Caveat: **{CAVEAT_ZH}**

![Boundary channel/window anisotropy](boundary_anisotropy/margin_stage1_redo_v1/figures/boundary_channel_window_anisotropy.png)

Key numbers:

- roll+pitch sensitivity share: `0.8622562334993128`.
- Channel participation ratio: `2.6205793568791225 / 4`.
- Window participation ratio: `7.618880107165337 / 10`.
- Source run: `runs/margin_stage1_redo_v1`.
- Runner: `cadet.runners.margin_stage1_redo`.
- Supports paper narrative §3.2 / H1.

## H2 Warm Start Does Not Save Queries

Claim: cross-condition warm starts preserve some local structure but do not
produce the required query savings.

Caveat: **{CAVEAT_ZH}**

![H2 warm-start speedups](h2_warmstart/route1_h2_px4_position_seed0_vmax_pilot_v0/figures/h2_warmstart_speedups.png)

Key numbers:

- Cold-over-warm speedups: `1.0132450331125828`, `1.111842105263158`,
  `1.6486486486486487`; all below the 2x bar.
- Source run: `runs/route1_h2_px4_position_seed0_vmax_pilot_v0`.
- Runner: `cadet.runners.route1_h2_campaign`.
- Supports paper narrative §4.2 / H2 negative result.

## H3 Transition Handoff Is Clean

Claim: the POSCTL-to-AUTO.LOITER transition handoff produced no robust
transition-specific violation in this probe.

Caveat: **{CAVEAT_ZH}** H3 uses `px4_transition`, but it is still a seed-0 PX4
probe.

![H3 zero transition violations](h3_transition/h3_transition_seed0_v1/figures/h3_zero_transition_violations.png)

Key numbers:

- Robust transition violation count: `0`.
- Distinct robust cluster count: `0`.
- Stage-A query count: `325`.
- Source run: `runs/h3_transition_seed0_v1`.
- Runner: `cadet.runners.h3_transition`.
- Supports paper narrative §4.3 / H3 negative result.

## RQ1 Thin Target / Three Arms

Claim: the target is thin and channel-directed search exposes clean triggers
that random and channel-agnostic probing miss or reach only densely.

Caveat: **{CAVEAT_ZH}**

![RQ1 three-arm comparison](rq1_three_arm/direction_a_px4_position_seed0_v0/figures/rq1_three_arm_counts_support.png)

Key numbers:

- Interior robust violations A/B/C: `0 / 7 / 18`.
- Arm B interior supports are dense, centered around about `27`.
- Arm C interior supports are clean and small, `4-8`.
- Source run: `runs/direction_a_px4_position_seed0_v0`.
- Runner: `cadet.runners.direction_a_probe`.
- Supports paper narrative §5.1 / RQ1.

Minimal trigger example:

![Minimal trigger roll pulse](rq1_three_arm/direction_a_px4_position_seed0_v0/figures/minimal_trigger_roll_pulse_xy_velocity.png)

This is Arm C `eval_id=196`, theta hash `9dfde040ab231b58`: a four-cell roll
pulse ending at neutral. The archived repeat shown crosses the `1.0 m/s`
post-neutral xy-speed threshold; the J=5 robust summary is
`rho_mean=-0.0488912449450468`, `rho_std=0.0117186427878129`, so
`rho+2sigma=-0.025453959369420998`.

## RQ2 ddmin Necessity Baseline

Claim: post-hoc channel-agnostic delta debugging cannot match the channel-
directed trigger synthesis in this seed-0 probe.

Caveat: **{CAVEAT_ZH}**

![RQ2 ddmin comparison](rq2_ddmin/direction_a_ddmin_px4_position_seed0_v1/figures/rq2_ddmin_clean_support_cost.png)

Key numbers:

- Clean minimized triggers: `4 / 10`.
- Final support median: `14.5`, versus Arm C median `6.0`.
- Cost ratio per clean trigger: `22.10625`.
- Source run: `runs/direction_a_ddmin_px4_position_seed0_v1`.
- Runner: `cadet.runners.direction_a_ddmin`.
- Supports paper narrative §5.2 / RQ2.

## Include / Exclude / Gap Register

INCLUDE:

- `runs/archive/rq1_zero_theta_sph_rejected/legacy_rq1_minimal_v0` SPH J=5
  denoise and persistence reports only.
- `runs/margin_stage1_redo_v1`.
- `runs/route1_h2_px4_position_seed0_vmax_pilot_v0`.
- `runs/h3_transition_seed0_v1`.
- `runs/direction_a_px4_position_seed0_v0`.
- `runs/direction_a_ddmin_px4_position_seed0_v1`.

EXCLUDE:

- `runs/margin_stage1_v1` and `runs/margin_stage1_v1_d05996_probe`: replaced by
  the δ=0.2 redo.
- `runs/direction_a_ddmin_px4_position_seed0_v0`: incomplete/no canonical
  summary, replaced by v1.
- `runs/h3_transition_seed0_v0`: earlier H3 stage0-focused run, replaced by v1.
- `runs/margin_stage0_v1`: support/anchor run, not a quoted claim result here.
- `runs/rq1_boundary_v0`, `runs/phase0_metrics`, `runs/synthetic_sanity_v0`:
  outside this checkpoint's claim table.
- `runs/archive` content outside the SPH J=5 denoise/persistence subtree.

Current gaps:

- No contradiction was found between the curated canonical runs and the quoted
  narrative numbers.
- The `noise≈282×step` ratio lacks direct small-artifact provenance in the
  local run tree and should be confirmed before paper use.

All figures in this index were generated from the curated small artifacts by
`scripts/curate_results_checkpoint.py`; no simulator experiment was run.
""",
    )


def main() -> None:
    paths = {
        "sph": copy_sph_plateau(),
        "boundary": copy_boundary_anisotropy(),
        "h2": copy_h2_warmstart(),
        "h3": copy_h3_transition(),
        "rq1": copy_rq1_three_arm(),
        "rq2": copy_rq2_ddmin(),
    }
    plot_sph(paths["sph"])
    plot_boundary(paths["boundary"])
    plot_h2(paths["h2"])
    plot_h3(paths["h3"])
    plot_rq1(paths["rq1"])
    plot_money_trigger(paths["rq1"])
    plot_rq2(paths["rq2"])
    write_results_index(paths)
    print("Curated CADET checkpoint artifacts under artifacts/.")


if __name__ == "__main__":
    main()
