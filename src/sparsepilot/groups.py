from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Group:
    group_id: int
    channel: str
    window_id: int
    t_start: float
    t_end: float


def build_groups(horizon_s: float, window_s: float, channels: list[str]) -> list[Group]:
    num_windows = int(round(horizon_s / window_s))
    groups: list[Group] = []
    group_id = 0
    for window_id in range(num_windows):
        t_start = window_id * window_s
        t_end = t_start + window_s
        for channel in channels:
            groups.append(Group(group_id, channel, window_id, t_start, t_end))
            group_id += 1
    return groups


def group_count(groups: list[Group]) -> int:
    return len(groups)
