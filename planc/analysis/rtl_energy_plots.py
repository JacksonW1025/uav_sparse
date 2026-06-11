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
    "blocked": "#adb5bd",
}


def _complete(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [p for p in points if p.get("distance_m") is not None and p.get("wind_m_s") is not None]


def plot_result_field(points: list[dict[str, Any]], out_path: Path) -> str:
    pts = _complete(points)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    for label, color in ZONE_COLORS.items():
        subset = [p for p in pts if p.get("label") == label]
        if not subset:
            continue
        ax.scatter(
            [float(p["distance_m"]) for p in subset],
            [float(p["wind_m_s"]) for p in subset],
            s=520,
            marker="s",
            color=color,
            edgecolor="black",
            linewidth=0.8,
            label=label,
        )
    for p in pts:
        label = str(p.get("label", "blocked"))
        abbrev = {"clean_safe": "S", "clean_unsafe": "U", "contract_violated": "V", "blocked": "B"}.get(label, "?")
        text_color = "white" if label in {"clean_unsafe", "contract_violated"} else "black"
        ax.text(float(p["distance_m"]), float(p["wind_m_s"]), abbrev, ha="center", va="center", color=text_color, fontsize=10)
    ax.set_xlabel("Outbound distance D (m)")
    ax.set_ylabel("Return headwind / outbound tailwind (m/s)")
    ax.set_title("RTL energy three-zone field")
    ax.grid(True, alpha=0.22)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_severity_heatmap(points: list[dict[str, Any]], out_path: Path) -> str:
    pts = [p for p in _complete(points) if p.get("severity_final_distance_m") is not None]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    if pts:
        xs = np.array([float(p["distance_m"]) for p in pts])
        ys = np.array([float(p["wind_m_s"]) for p in pts])
        zs = np.array([float(p["severity_final_distance_m"]) for p in pts])
        if len(pts) >= 4 and len(set(xs)) > 1 and len(set(ys)) > 1:
            levels = np.linspace(float(np.nanmin(zs)), float(np.nanmax(zs)), 16)
            cf = ax.tricontourf(xs, ys, zs, levels=levels, cmap="magma")
            fig.colorbar(cf, ax=ax, label="Final distance from home (m)")
        sc = ax.scatter(xs, ys, c=zs, cmap="magma", s=95, edgecolor="black", linewidth=0.7)
        if len(pts) < 4:
            fig.colorbar(sc, ax=ax, label="Final distance from home (m)")
        for p in pts:
            ax.text(
                float(p["distance_m"]),
                float(p["wind_m_s"]),
                f"{float(p['severity_final_distance_m']):.0f}",
                ha="center",
                va="center",
                fontsize=8,
                color="white",
            )
    ax.set_xlabel("Outbound distance D (m)")
    ax.set_ylabel("Return headwind / outbound tailwind (m/s)")
    ax.set_title("Severity heatmap")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_p_stratification(layers: dict[str, dict[str, Any]], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.5, 5.4), gridspec_kw={"width_ratios": [1.0, 1.35]})
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1].get("batt_low_mah", 0.0)))
    thresholds = [float(layer.get("batt_low_mah", 0.0)) for _, layer in ordered]
    unsafe_counts = [int(layer.get("zone_counts", {}).get("clean_unsafe", 0)) for _, layer in ordered]
    ax0.plot(thresholds, unsafe_counts, marker="o", color="#d62828", linewidth=2.0)
    ax0.set_xlabel("BATT_LOW_MAH threshold (mAh)")
    ax0.set_ylabel("clean_unsafe points")
    ax0.set_title("Unsafe-region size")
    ax0.grid(True, alpha=0.25)

    markers = ["o", "s", "^", "D"]
    colors = ["#e76f51", "#f4a261", "#4361ee", "#2a9d8f"]
    for idx, (name, layer) in enumerate(ordered):
        pts = [p for p in layer.get("points", []) if p.get("label") == "clean_unsafe"]
        if not pts:
            continue
        ax1.scatter(
            [float(p["distance_m"]) for p in pts],
            [float(p["wind_m_s"]) for p in pts],
            s=110,
            marker=markers[idx % len(markers)],
            color=colors[idx % len(colors)],
            edgecolor="black",
            linewidth=0.7,
            label=f"{name}: {float(layer.get('batt_low_mah', 0.0)):.0f} mAh",
        )
    ax1.set_xlabel("Outbound distance D (m)")
    ax1.set_ylabel("Return headwind / outbound tailwind (m/s)")
    ax1.set_title("clean_unsafe points by P layer")
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_train_test(prediction: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.0, 6.0))
    styles = {
        "interpolation": {"marker": "o", "color": "#2a9d8f", "label": "Interpolation"},
        "extrapolation": {"marker": "^", "color": "#e76f51", "label": "Extrapolation"},
    }
    for split, style in styles.items():
        rows = prediction.get(split, {}).get("predictions", [])
        if not rows:
            continue
        ax.scatter(
            [float(r["probability_unsafe"]) for r in rows],
            [1.0 if r["observed_unsafe"] else 0.0 for r in rows],
            s=95,
            marker=style["marker"],
            color=style["color"],
            edgecolor="black",
            linewidth=0.7,
            label=style["label"],
        )
        for r in rows:
            ax.text(
                float(r["probability_unsafe"]),
                1.0 if r["observed_unsafe"] else 0.0,
                f"{int(float(r['distance_m']))}/{int(float(r['wind_m_s']))}",
                fontsize=7,
                ha="left",
                va="bottom",
            )
    ax.axvline(0.5, color="black", linestyle="--", linewidth=1.1, label="decision threshold")
    ax.set_xlim(-0.03, 1.03)
    ax.set_ylim(-0.18, 1.18)
    ax.set_yticks([0, 1], labels=["safe", "unsafe"])
    ax.set_xlabel("Predicted unsafe probability")
    ax.set_ylabel("Observed binary outcome")
    ax.set_title("Held-out prediction, including extrapolation")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_premise(premise: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = premise.get("runs", [])
    labels = [str(r.get("label", r.get("run_id", ""))) for r in rows]
    mah = [float(r.get("consumed_mah", 0.0) or 0.0) for r in rows]
    drop_rate = [float(r.get("voltage_drop_rate_v_s", 0.0) or 0.0) for r in rows]
    fig, ax0 = plt.subplots(figsize=(8.8, 5.8))
    x = np.arange(len(rows))
    ax0.bar(x - 0.18, mah, width=0.36, color="#4361ee", label="Consumed mAh")
    ax0.set_ylabel("Consumed mAh")
    ax1 = ax0.twinx()
    ax1.bar(x + 0.18, drop_rate, width=0.36, color="#f77f00", label="Voltage drop rate")
    ax1.set_ylabel("Voltage drop rate (V/s)")
    ax0.set_xticks(x, labels=labels, rotation=20, ha="right")
    ax0.set_title("Premise check: energy response to wind and mass")
    ax0.grid(True, axis="y", alpha=0.22)
    lines0, labels0 = ax0.get_legend_handles_labels()
    lines1, labels1 = ax1.get_legend_handles_labels()
    ax0.legend(lines0 + lines1, labels0 + labels1, loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)
