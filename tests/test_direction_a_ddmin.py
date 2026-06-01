from __future__ import annotations

from pathlib import Path

from sparsepilot.runners.direction_a_ddmin import CLEAN_CHANNELS, _is_clean, select_starting_points


def test_select_starting_points_uses_all_interiors_then_densest_moderates():
    starts = select_starting_points(Path("runs/direction_a_px4_position_seed0_v0"), moderate_starts=3, max_starts=10)

    assert [point.eval_id for point in starts[:7]] == [97, 107, 116, 119, 147, 150, 151]
    assert {point.selection_bucket for point in starts[:7]} == {"arm_b_interior"}
    assert [point.eval_id for point in starts[7:]] == [113, 105, 89]
    assert {point.selection_bucket for point in starts[7:]} == {"arm_b_densest_moderate"}


def test_clean_definition_matches_pre_registered_thresholds():
    assert CLEAN_CHANNELS == {"roll", "pitch"}
    assert _is_clean({"support_size": 8, "active_channels": ["roll", "pitch"]})
    assert _is_clean({"support_size": 4, "active_channels": ["pitch"]})
    assert not _is_clean({"support_size": 9, "active_channels": ["roll", "pitch"]})
    assert not _is_clean({"support_size": 2, "active_channels": ["roll", "yaw"]})
