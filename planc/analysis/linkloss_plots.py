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


def _points(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [p for p in points if p.get("speed_m_s") is not None and p.get("wind_m_s") is not None]


def plot_premise(premise: dict[str, Any], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    groups = premise.get("response_groups", {})
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.6))
    specs = [
        ("speed_response", "Command speed (m/s)"),
        ("wind_response", "Outbound tailwind (m/s)"),
        ("timeout_response", "GCS timeout (s)"),
    ]
    for ax, (name, xlabel) in zip(axes, specs):
        rows = groups.get(name, [])
        xs = [float(r.get("x", 0.0)) for r in rows]
        ys = [float(r.get("overshoot_m", 0.0) or 0.0) for r in rows]
        labels = [str(r.get("run_id", "")) for r in rows]
        ax.plot(xs, ys, marker="o", linewidth=2.0, color="#4361ee")
        for x, y, label in zip(xs, ys, labels):
            ax.text(x, y, label.replace("linkloss_premise_", ""), fontsize=7, ha="left", va="bottom")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Fence overshoot (m)")
        ax.grid(True, alpha=0.25)
    axes[0].set_title("Speed response")
    axes[1].set_title("Wind response")
    axes[2].set_title("Timeout response")
    fig.suptitle("Premise: excursion response to speed, wind, and timeout")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_result_field(points: list[dict[str, Any]], out_path: Path) -> str:
    pts = _points(points)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    for label, color in ZONE_COLORS.items():
        subset = [p for p in pts if p.get("label") == label]
        if not subset:
            continue
        ax.scatter(
            [float(p["speed_m_s"]) for p in subset],
            [float(p["wind_m_s"]) for p in subset],
            s=520,
            marker="s",
            color=color,
            edgecolor="black",
            linewidth=0.8,
            label=label,
        )
    abbrev = {"clean_safe": "S", "clean_unsafe": "U", "contract_violated": "V", "blocked": "B"}
    for p in pts:
        label = str(p.get("label", "blocked"))
        text_color = "white" if label in {"clean_unsafe", "contract_violated"} else "black"
        ax.text(
            float(p["speed_m_s"]),
            float(p["wind_m_s"]),
            abbrev.get(label, "?"),
            ha="center",
            va="center",
            color=text_color,
            fontsize=10,
        )
    ax.set_xlabel("Command speed (m/s)")
    ax.set_ylabel("Outbound tailwind (m/s)")
    ax.set_title("GCS link-loss three-zone field")
    ax.grid(True, alpha=0.22)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_severity_heatmap(points: list[dict[str, Any]], out_path: Path) -> str:
    pts = [p for p in _points(points) if p.get("severity_overshoot_m") is not None]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    if pts:
        xs = np.array([float(p["speed_m_s"]) for p in pts])
        ys = np.array([float(p["wind_m_s"]) for p in pts])
        zs = np.array([float(p.get("severity_overshoot_m") or 0.0) for p in pts])
        if len(pts) >= 4 and len(set(xs)) > 1 and len(set(ys)) > 1 and float(np.nanmax(zs)) > float(np.nanmin(zs)):
            levels = np.linspace(float(np.nanmin(zs)), float(np.nanmax(zs)), 16)
            cf = ax.tricontourf(xs, ys, zs, levels=levels, cmap="magma")
            fig.colorbar(cf, ax=ax, label="Fence overshoot (m)")
        sc = ax.scatter(xs, ys, c=zs, cmap="magma", s=95, edgecolor="black", linewidth=0.7)
        if len(pts) < 4 or float(np.nanmax(zs)) <= float(np.nanmin(zs)):
            fig.colorbar(sc, ax=ax, label="Fence overshoot (m)")
        for p in pts:
            ax.text(
                float(p["speed_m_s"]),
                float(p["wind_m_s"]),
                f"{float(p.get('severity_overshoot_m') or 0.0):.0f}",
                ha="center",
                va="center",
                fontsize=8,
                color="white",
            )
    ax.set_xlabel("Command speed (m/s)")
    ax.set_ylabel("Outbound tailwind (m/s)")
    ax.set_title("Excursion severity")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fig.savefig(out_path, dpi=170)
    plt.close(fig)
    return str(out_path)


def plot_p_stratification(layers: dict[str, dict[str, Any]], out_path: Path) -> str:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.5, 5.4), gridspec_kw={"width_ratios": [1.0, 1.35]})
    ordered = sorted(layers.items(), key=lambda kv: float(kv[1].get("timeout_s", 0.0)))
    timeouts = [float(layer.get("timeout_s", 0.0)) for _, layer in ordered]
    unsafe_counts = [int(layer.get("zone_counts", {}).get("clean_unsafe", 0)) for _, layer in ordered]
    ax0.plot(timeouts, unsafe_counts, marker="o", color="#d62828", linewidth=2.0)
    ax0.set_xlabel("FS_GCS_TIMEOUT (s)")
    ax0.set_ylabel("clean_unsafe points")
    ax0.set_title("Unsafe-region size")
    ax0.grid(True, alpha=0.25)

    markers = ["o", "s", "^", "D"]
    colors = ["#2a9d8f", "#4361ee", "#e76f51", "#f77f00"]
    for idx, (name, layer) in enumerate(ordered):
        pts = [p for p in layer.get("points", []) if p.get("label") == "clean_unsafe"]
        if not pts:
            continue
        ax1.scatter(
            [float(p["speed_m_s"]) for p in pts],
            [float(p["wind_m_s"]) for p in pts],
            s=110,
            marker=markers[idx % len(markers)],
            color=colors[idx % len(colors)],
            edgecolor="black",
            linewidth=0.7,
            label=f"{name}: {float(layer.get('timeout_s', 0.0)):.0f} s",
        )
    ax1.set_xlabel("Command speed (m/s)")
    ax1.set_ylabel("Outbound tailwind (m/s)")
    ax1.set_title("clean_unsafe points by timeout layer")
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
                f"{int(float(r['speed_m_s']))}/{int(float(r['wind_m_s']))}",
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
