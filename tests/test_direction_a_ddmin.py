from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from sparsepilot.runners.direction_a_ddmin import CLEAN_CHANNELS, _is_clean, select_starting_points


def test_select_starting_points_uses_all_interiors_then_densest_moderates(tmp_path: Path):
    probe_dir = tmp_path / "probe"
    reports_dir = probe_dir / "reports"
    thetas_dir = probe_dir / "thetas"
    reports_dir.mkdir(parents=True)
    thetas_dir.mkdir()

    rows = [
        _point_row(probe_dir, "B", "interior", 30, 12, -0.10, 0.01),
        _point_row(probe_dir, "B", "interior", 10, 9, -0.12, 0.01),
        _point_row(probe_dir, "B", "interior", 20, 11, -0.11, 0.01),
        _point_row(probe_dir, "B", "moderate", 100, 5, -0.20, 0.01),
        _point_row(probe_dir, "B", "moderate", 101, 9, -0.03, 0.001),
        _point_row(probe_dir, "B", "moderate", 102, 9, -0.06, 0.001),
        _point_row(probe_dir, "A", "interior", 1, 40, -1.00, 0.01),
        _point_row(probe_dir, "B", "interior", 40, 8, 1.00, 0.01, robustness_class="robust_safe"),
    ]
    with (reports_dir / "point_evaluations.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    starts = select_starting_points(probe_dir, moderate_starts=2, max_starts=5)

    buckets = [point.selection_bucket for point in starts]
    assert buckets == ["arm_b_interior", "arm_b_interior", "arm_b_interior"] + [
        "arm_b_densest_moderate",
        "arm_b_densest_moderate",
    ]

    interiors = [point for point in starts if point.selection_bucket == "arm_b_interior"]
    moderates = [point for point in starts if point.selection_bucket == "arm_b_densest_moderate"]

    assert [point.eval_id for point in interiors] == sorted(point.eval_id for point in interiors)
    assert len(interiors) == 3
    assert len(moderates) == 2
    assert min(point.support_size for point in moderates) >= 9
    assert all(point.theta_path.parent == thetas_dir for point in starts)
    assert all(point.theta.shape == (40,) for point in starts)


def _point_row(
    probe_dir: Path,
    arm: str,
    amplitude_class: str,
    eval_id: int,
    support_size: int,
    rho_mean: float,
    rho_std: float,
    *,
    robustness_class: str = "robust_violation",
) -> dict[str, object]:
    filename = f"{arm}_{eval_id:05d}_fixture.npy"
    np.save(probe_dir / "thetas" / filename, np.full(40, eval_id, dtype=float))
    return {
        "arm": arm,
        "robustness_class": robustness_class,
        "amplitude_class": amplitude_class,
        "rho_mean_post_neutral_xy_velocity": rho_mean,
        "rho_std_post_neutral_xy_velocity": rho_std,
        "support_size_abs_gt_0p1": support_size,
        "eval_id": eval_id,
        "theta_path": f"runs/original_probe/thetas/{filename}",
        "stage": "fixture",
        "label": f"{amplitude_class}_{eval_id}",
        "theta_hash": f"hash{eval_id}",
        "max_abs_theta": 0.25,
        "active_channels_abs_gt_0p1": "pitch,roll",
    }


def test_clean_definition_matches_pre_registered_thresholds():
    assert CLEAN_CHANNELS == {"roll", "pitch"}
    assert _is_clean({"support_size": 8, "active_channels": ["roll", "pitch"]})
    assert _is_clean({"support_size": 4, "active_channels": ["pitch"]})
    assert not _is_clean({"support_size": 9, "active_channels": ["roll", "pitch"]})
    assert not _is_clean({"support_size": 2, "active_channels": ["roll", "yaw"]})
