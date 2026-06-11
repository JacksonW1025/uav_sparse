from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


EARTH_RADIUS_M = 6378137.0


def _read_rows(csv_path: Path) -> list[dict[str, Any]]:
    with csv_path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _xy_m(lat: float, lon: float, home_lat: float, home_lon: float) -> tuple[float, float]:
    x = math.radians(lon - home_lon) * EARTH_RADIUS_M * math.cos(math.radians(home_lat))
    y = math.radians(lat - home_lat) * EARTH_RADIUS_M
    return x, y


def plot_run(
    run_id: str,
    csv_path: Path,
    out_dir: Path,
    home: dict[str, Any],
    fence_radius_m: float,
    hard_boundary_m: float | None,
    modes: list[dict[str, Any]] | None = None,
) -> dict[str, str]:
    rows = _read_rows(csv_path)
    out_dir.mkdir(parents=True, exist_ok=True)
    home_lat = float(home["lat"])
    home_lon = float(home["lon"])
    xs: list[float] = []
    ys: list[float] = []
    ts: list[float] = []
    ds: list[float] = []
    for row in rows:
        lat = float(row["lat"])
        lon = float(row["lon"])
        x, y = _xy_m(lat, lon, home_lat, home_lon)
        xs.append(x)
        ys.append(y)
        ts.append(float(row["time_s"]))
        ds.append(float(row["distance_m"]))

    theta = [2 * math.pi * i / 240 for i in range(241)]
    fig, ax = plt.subplots(figsize=(7, 7))
    if xs and ys:
        ax.plot(xs, ys, lw=1.6, label="trajectory")
        max_i = max(range(len(ds)), key=lambda i: ds[i])
        ax.scatter([xs[0]], [ys[0]], s=30, label="start")
        ax.scatter([xs[max_i]], [ys[max_i]], s=35, label="max distance")
    ax.plot([fence_radius_m * math.cos(t) for t in theta], [fence_radius_m * math.sin(t) for t in theta], "--", label="fence R")
    if hard_boundary_m is not None:
        ax.plot(
            [hard_boundary_m * math.cos(t) for t in theta],
            [hard_boundary_m * math.sin(t) for t in theta],
            ":",
            label="hard boundary",
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East from home (m)")
    ax.set_ylabel("North from home (m)")
    ax.set_title(f"{run_id} trajectory")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    traj_path = out_dir / f"trajectory_{run_id}.png"
    fig.tight_layout()
    fig.savefig(traj_path, dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.8))
    if ts and ds:
        ax.plot(ts, ds, lw=1.6, label="distance from fence center")
    ax.axhline(fence_radius_m, color="black", linestyle="--", label="fence R")
    if hard_boundary_m is not None:
        ax.axhline(hard_boundary_m, color="tab:red", linestyle=":", label="hard boundary")
    for mode in modes or []:
        name = str(mode.get("mode", ""))
        if name in {"RTL", "LAND", "BRAKE", "SMART_RTL"}:
            t = float(mode.get("time_s", 0.0))
            ax.axvline(t, color="tab:orange", alpha=0.5)
            ax.text(t, ax.get_ylim()[1], name, rotation=90, va="top", ha="right", fontsize=8)
    ax.set_xlabel("Log time (s)")
    ax.set_ylabel("Horizontal distance (m)")
    ax.set_title(f"{run_id} distance")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    dist_path = out_dir / f"distance_{run_id}.png"
    fig.tight_layout()
    fig.savefig(dist_path, dpi=160)
    plt.close(fig)
    return {"trajectory": str(traj_path), "distance": str(dist_path)}
