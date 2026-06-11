from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ZONE_COLORS = {
    "clean_safe": "#2a9d8f",
    "clean_unsafe": "#d62828",
    "contract_violated": "#6c757d",
}


def _complete_points(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [p for p in points if p.get("overshoot_mean_m") is not None]


def plot_overshoot_heatmap(
    points: list[dict[str, Any]],
    d_hazard_m: float,
    witness: dict[str, Any] | None,
    out_path: Path,
) -> str:
    pts = _complete_points(points)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    xs = np.array([float(p["commanded_speed_m_s"]) for p in pts])
    ys = np.array([float(p["tailwind_m_s"]) for p in pts])
    zs = np.array([float(p["overshoot_mean_m"]) for p in pts])

    fig, ax = plt.subplots(figsize=(8.5, 6.2))
    if len(pts) >= 3:
        levels = np.linspace(float(np.nanmin(zs)), float(np.nanmax(zs)), 18)
        contourf = ax.tricontourf(xs, ys, zs, levels=levels, cmap="viridis")
        fig.colorbar(contourf, ax=ax, label="Overshoot beyond fence R (m)")
        if float(np.nanmin(zs)) <= d_hazard_m <= float(np.nanmax(zs)):
            contour = ax.tricontour(xs, ys, zs, levels=[d_hazard_m], colors="white", linewidths=2.0)
            ax.clabel(contour, fmt={d_hazard_m: "d_hazard"}, colors="white")
    scatter = ax.scatter(xs, ys, c=zs, cmap="viridis", edgecolor="black", s=70)
    if len(pts) < 3:
        fig.colorbar(scatter, ax=ax, label="Overshoot beyond fence R (m)")
    if witness:
        ax.scatter(
            [float(witness["commanded_speed_m_s"])],
            [float(witness["tailwind_m_s"])],
            marker="*",
            s=260,
            color="#ffba08",
            edgecolor="black",
            linewidth=0.8,
            label="Sparse witness",
            zorder=4,
        )
    ax.set_xlabel("Commanded forward speed v (m/s)")
    ax.set_ylabel("Tailwind w (m/s)")
    ax.set_title("Overshoot field with fixed hazard contour")
    ax.grid(True, alpha=0.25)
    if witness:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_three_zone(points: list[dict[str, Any]], d_hazard_m: float, out_path: Path) -> str:
    pts = _complete_points(points)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.5, 6.2))
    for label, color in ZONE_COLORS.items():
        subset = [p for p in pts if p.get("label") == label]
        if not subset:
            continue
        ax.scatter(
            [float(p["commanded_speed_m_s"]) for p in subset],
            [float(p["tailwind_m_s"]) for p in subset],
            s=500,
            marker="s",
            color=color,
            edgecolor="black",
            linewidth=0.8,
            label=label,
        )
    for p in pts:
        ax.text(
            float(p["commanded_speed_m_s"]),
            float(p["tailwind_m_s"]),
            f"{float(p['overshoot_mean_m']):.1f}",
            ha="center",
            va="center",
            fontsize=8,
            color="white" if p.get("label") != "clean_safe" else "black",
        )
    ax.set_xlabel("Commanded forward speed v (m/s)")
    ax.set_ylabel("Tailwind w (m/s)")
    ax.set_title(f"Three-zone contract map, d_hazard={d_hazard_m:.2f} m")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_p_stratification(layers: dict[str, list[dict[str, Any]]], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.5, 6.2))
    palette = {
        "2": "#d62828",
        "5": "#f77f00",
        "10": "#4361ee",
    }
    markers = {
        "2": "o",
        "5": "s",
        "10": "^",
    }
    for margin, points in sorted(layers.items(), key=lambda kv: float(kv[0])):
        pts = _complete_points(points)
        clean = [p for p in pts if p.get("contract_clean_all")]
        if clean:
            ax.scatter(
                [float(p["commanded_speed_m_s"]) for p in clean],
                [float(p["tailwind_m_s"]) for p in clean],
                s=35,
                color="#dee2e6",
                edgecolor="#adb5bd",
                linewidth=0.5,
                zorder=1,
            )
        unsafe = [p for p in pts if p.get("label") == "clean_unsafe"]
        if unsafe:
            key = str(int(float(margin)))
            ax.scatter(
                [float(p["commanded_speed_m_s"]) for p in unsafe],
                [float(p["tailwind_m_s"]) for p in unsafe],
                s=130,
                marker=markers.get(key, "o"),
                color=palette.get(key, None),
                edgecolor="black",
                linewidth=0.7,
                label=f"FENCE_MARGIN={margin} m planc",
                zorder=3,
            )
    ax.set_xlabel("Commanded forward speed v (m/s)")
    ax.set_ylabel("Tailwind w (m/s)")
    ax.set_title("P-stratification: clean-unsafe region by FENCE_MARGIN")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_train_test(prediction: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.2, 6.4))
    all_values: list[float] = []
    styles = {
        "interpolation": {"marker": "o", "color": "#2a9d8f", "label": "Interpolation holdout"},
        "extrapolation": {"marker": "^", "color": "#e76f51", "label": "Extrapolation holdout"},
    }
    for name, style in styles.items():
        rows = prediction.get(name, {}).get("predictions", [])
        if not rows:
            continue
        observed = [float(r["observed_overshoot_m"]) for r in rows]
        predicted = [float(r["predicted_overshoot_m"]) for r in rows]
        all_values.extend(observed)
        all_values.extend(predicted)
        ax.scatter(
            observed,
            predicted,
            s=80,
            marker=style["marker"],
            color=style["color"],
            edgecolor="black",
            linewidth=0.7,
            label=style["label"],
        )
    if all_values:
        lo = min(all_values) - 2.0
        hi = max(all_values) + 2.0
    else:
        lo, hi = 0.0, 1.0
    ax.plot([lo, hi], [lo, hi], color="black", linestyle="-", linewidth=1.2, label="y=x")
    ax.fill_between([lo, hi], [lo - 3, hi - 3], [lo + 3, hi + 3], color="#adb5bd", alpha=0.25, label="+/-3 m")
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_xlabel("Observed overshoot (m)")
    ax.set_ylabel("Predicted overshoot (m)")
    ax.set_title("Train/test prediction on held-out points")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)
